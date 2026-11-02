# AGENTS.md

Guide for AI coding agents (and humans) reading this repository: what the firmware does, how
the code is laid out, and how to run a hub with or without Home Assistant.

## What the system is

A hub that switches radiator booster fans between four steps: off, low, mid, high. The fans
sit on a chain of WS2811 chips ("nodes"), one data line from the hub. The hub reads two
DS18B20 probes on the heating pipes (flow and return) and, in `auto`, picks the step from the
flow temperature.

Two chips can sit on the hub PCB:

| Chip | Role | Firmware |
|------|------|----------|
| Seeed XIAO ESP32-C3 | always fitted: probes, fan curve, LED, button, WiFi/MQTT, OTA | `src/controller.cpp` + `components/` (ESP-IDF via PlatformIO) |
| ATtiny13A | optional manual-mode controller: button cycles off/low/mid/high without the XIAO's help | `attiny/src/t2_modes.c` (AVR via PlatformIO) |

The XIAO detects the ATtiny at boot over ISP (`hub_board_find_attiny`) and runs one of two
setups in `components/hub_setups/`:

- `run_xiao_alone()`: the XIAO owns the bus, LED and button. Button short press steps
  auto → low → mid → high; the mode is saved in NVS. Returns if an ATtiny powers up later.
- `run_xiao_with_attiny()`: the ATtiny owns the button. While the ATtiny is in `off` the
  XIAO drives the bus in auto (fan curve); in low/mid/high the ATtiny drives the bus and the
  XIAO lets go. The XIAO follows the ATtiny's mode by counting short presses on the shared
  button line, and changes it (for Home Assistant) by pressing that line itself
  (`hub_button_press`).

## Fan curve

`components/fan_curve/fan_curve.cpp`: step up at flow ≥ 28, 32, 36 °C (low, mid, high); each
step drops 1 °C below its threshold. A flow probe that does not answer keeps the current
step. The return probe is reported only; it does not control anything.

## Bus protocol

WS2811 low-speed mode (2.5 µs per bit), sent by the ESP32 RMT peripheral
(`components/ws2811_tx`) or bit-banged by the ATtiny (`send_byte` in `t2_modes.c`). Every
node gets the same three bytes; the frame is always 50 nodes long (`FAN_BUS_NODES`, `NODES`),
and nodes past the end of a shorter chain simply see nothing. The bytes per step:

| Step | Bytes |
|------|-------|
| off | 255 255 255 |
| low | 255 255 0 |
| mid | 255 0 255 |
| high | 0 255 255 |

Byte value 0 is the node channel for that speed; 255 leaves it off. The table exists twice,
in `components/fan_bus/fan_bus.cpp` (`TRIPLES`) and `attiny/src/t2_modes.c` (`frames`);
change both together. Frames are resent every second so a node that lost power picks the
step up again.

## Pins

XIAO (`components/hub_board/include/hub_pins.h`):

| XIAO pin | GPIO | Function | ATtiny pin when fitted |
|----------|------|----------|------------------------|
| D1 | 3 | bus data out | PB0 (MOSI) |
| D2 | 4 | LED red cathode | |
| D3 | 5 | LED blue cathode | |
| D4 | 6 | ATtiny RESET | RESET |
| D5 | 7 | sense: ATtiny holds it low while it drives the bus | PB2 (SCK) |
| D6 | 21 | LED green cathode | |
| D7 | 20 | button, open drain with pull-up, shared | PB1 (MISO) |
| D8 | 8 | return DS18B20 (4.7 kΩ pull-up to 3V3) | |
| D10 | 10 | flow DS18B20 (4.7 kΩ pull-up to 3V3) | |

The ISP lines double as the run-time link, so ISP detection must happen with the bus
released (`fan_bus_release`) and the button re-initialised afterwards. The ATtiny's LED is on
PB3 (red) and PB4 (green).

LED (common anode, XIAO): blue auto, green low, yellow mid, red high. With an ATtiny fitted
the XIAO shows blue only while it drives the bus; the ATtiny lights its own colours.

## Code layout and conventions

- `platformio.ini`: one environment per program. `src/CMakeLists.txt` builds only
  `src/${PROGRAM}.cpp`, chosen by `-DPROGRAM=` in the environment. Every file in `src/` is a
  separate `app_main`, not a shared module.
- `components/<name>/`: ESP-IDF components, each with `include/<name>.h` holding the
  contract and a `CMakeLists.txt` naming its `REQUIRES`. Read the header first; it says what
  each function does and when to call it.
- One cooperative loop: the setup calls `hub_board_wait_tick()` every 10 ms
  (`HUB_TICK_MS`) and each component gets a `*_tick(ms)` or `*_refresh(..., ms)` call to
  keep its own time. No component starts its own task. The exceptions are the ESP-IDF MQTT
  task, a 30 s link-check timer in `ha_link`, and the LED's 1 ms timer. MQTT commands cross
  into the loop through atomics in `components/hub_ha/hub_ha.cpp`.
- Errors use `ESP_ERROR_CHECK` and crash loudly; there is no defensive fallback code. Keep it
  that way.
- `hub_ha` is the hub-specific Home Assistant layer (what is published); `ha_link` is the
  WiFi/MQTT/OTA transport.

### Programs

| Environment | What it is |
|-------------|------------|
| `controller` | the installed firmware: probes, curve, setups, WiFi + MQTT + OTA |
| `hub-test` | `controller` plus a USB test console (`components/hub_console`) |
| `hub-nowifi` | `controller` without WiFi/MQTT: the radio never starts |
| `hub-noisp` | as `hub-nowifi` but skips the ATtiny check and assumes it is fitted |
| `bisect-*` | fault-finding builds that switch the LED and button on or off |
| `01-timing` … `07-colours`, `10-pins` | bench programs, one hardware check each (see README) |
| `05-isp` | flashes `src/attiny_image.h` into the ATtiny in place |
| `08-jig`, `09-hubjig` | test-jig firmware for a FireBeetle 2 ESP32-E |

`tools/hubrun.py` drives `09-hubjig` against a hub running `hub-test` over two serial ports.
The README's expected bench readings refer to notes that are not part of this repository.

## Building and flashing

Needs PlatformIO (`pio`). Board `seeed_xiao_esp32c3`, ESP-IDF framework, 4 MB flash with
two OTA slots (`partitions_ota.csv`).

```
cp src/secrets.h.example src/secrets.h   # only for controller and hub-test
pio run -e controller -t upload
pio device monitor
```

The monitor (115200) prints one status line a second:
`flow  31.50 C  return  27.00 C  mode auto  step low   bus XIAO`.

ATtiny (optional): build `t2-modes`, turn it into a header, flash it through the XIAO:

```
pio pkg install -g -p atmelavr
pio run -d attiny -e t2-modes
attiny/hex2h.py t2-modes
pio run -e 05-isp -t upload
pio device monitor        # type: go
```

Then flash `controller` again.

## Running without Home Assistant

### Option 1: standalone, no network

```
pio run -e hub-nowifi -t upload
```

No `secrets.h` needed. The hub runs the fan curve in auto, the button steps the mode, the
mode survives reboots (NVS, or the ATtiny's EEPROM), and status goes to USB serial. No OTA:
updates need USB.

`controller` with an unreachable WiFi also keeps working: the loop does not wait for the
network, and it retries the WiFi join every 30 s.

### Option 2: any MQTT broker

`controller` speaks plain MQTT; Home Assistant is only one consumer. Set `src/secrets.h`:

```c
#define WIFI_SSID "your-ssid"
#define WIFI_PASS "your-password"          // WPA2-PSK or better
#define MQTT_BROKER_URI "mqtt://192.0.2.10:1883"
```

The client sends no username or password, so the broker must allow anonymous clients on
that listener (Mosquitto: `allow_anonymous true`). To add credentials, set
`mqtt_cfg.credentials.username` and `mqtt_cfg.credentials.authentication.password` in
`ha_link_start` (`components/ha_link/ha_link.cpp`).

`<id>` is the last three bytes of the WiFi MAC (e.g. `F0BEBC`), printed at boot as
`hub F0BEBC`. The base topic is `radiator_hub_<id>`:

| Topic | Direction | Payload |
|-------|-----------|---------|
| `radiator_hub_<id>/status` | hub → | `online`, `offline` (last will), retained |
| `radiator_hub_<id>/flow` | hub → | °C, two decimals, retained |
| `radiator_hub_<id>/return` | hub → | °C, two decimals, retained |
| `radiator_hub_<id>/mode` | hub → | `auto` `low` `mid` `high`, retained |
| `radiator_hub_<id>/step` | hub → | `off` `low` `mid` `high` `unknown`, retained |
| `radiator_hub_<id>/mode/set` | → hub | `auto` `low` `mid` `high` |
| `radiator_hub_<id>/press` | → hub | anything: one button press |
| `radiator_hub_<id>/ota` | → hub | HTTP URL of a `firmware.bin` |

State is published on every mode or step change and every 10 s. A probe that does not answer
is not published, so its topic keeps the last good value.

The hub also publishes Home Assistant discovery configs under `homeassistant/...`. Without
HA they are retained messages nobody reads; they cost nothing.

Examples with Mosquitto's clients:

```
mosquitto_sub -h 192.0.2.10 -t 'radiator_hub_F0BEBC/#' -v
mosquitto_pub -h 192.0.2.10 -t radiator_hub_F0BEBC/mode/set -m high
```

OTA: build, serve the binary over HTTP, and publish its URL **without** the retain flag. A
retained URL is delivered again on every reconnect, and the hub would flash and restart on
each one.

```
pio run -e controller
python3 -m http.server -d .pio/build/controller 8000
mosquitto_pub -h 192.0.2.10 -t radiator_hub_F0BEBC/ota -m http://192.0.2.20:8000/firmware.bin
```

With an ATtiny fitted, `mode/set` is carried out by pressing the shared button line until the
ATtiny reaches the mode. Until the XIAO has seen the ATtiny in `off` once, it does not know
the ATtiny's mode: `mode` is not published and each request sends a single press.
