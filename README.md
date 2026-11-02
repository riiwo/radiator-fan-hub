# RadiatorFanBus

ESP-IDF bench firmware for the addressable radiator booster fans, built as a ladder of
single-purpose programs. Each program verifies exactly one thing and may only use
component code that the previous program proved. The expected readings for each program
live next to its build stage in `docs/tasks/radiator-fan-bus.md`.

| Program | Proves | Needs |
|---------|--------|-------|
| `01-timing` | bit timing of `components/ws2811_tx` on the wire | jumper D1 (GPIO3) → D2 (GPIO4), DMM |
| `02-level` | a chain of one or two WS2811 decodes a held `r g b [r g b]` frame | chips with 10 kΩ pull-ups on each OUT, DMM |
| `04-soak` | two chained node B circuits cycle OFF R G B, staggered, 10 min dwell, serial log | both nodes built and chained, two fans |
| `05-isp` | the XIAO flashes the ATtiny13A in place: signature, low fuse 0x7A, write, verify; `watch` logs button presses and decodes the ATtiny's frames | ATtiny on D4 RESET, D5 SCK, D7 MISO, D1 MOSI |
| `06-probes` | one DS18B20 per pin reads: flow D10, return D8, Skip ROM, CRC-checked, one line a second | probes with 4.7 kΩ pull-ups to the XIAO 3V3 |
| `controller` | The installed hub: finds the fitted chips and runs `run_xiao_alone` or `run_xiao_with_attiny` (`components/hub_setups`); WiFi, MQTT discovery and OTA as "Radiator hub <MAC>"; no console | ATtiny on `t2-modes` or absent, probes, nodes, Tambet IoT |
| `hub-test` | `controller` plus the test console the hub jig drives: `flow <C>` / `flow probe`, `mode <name>`, `nodes <n>`, `resend on\|off` | as `controller` |
| `07-colours` | the XIAO sinks red (D2), green (D6) and blue (D3) beside the ATtiny, cycling seven colours while the ATtiny is off; D6's boot log flickers green | R9, R10 1 kΩ to the red and green cathodes |
| `attiny/` `t1-blink` | ATtiny red and green alternate at 1 Hz | mode LED |
| `attiny/` `t2-modes` | button cycles off / low / mid / high, WS2811 low-speed frames to the chain, mode in EEPROM | button on PB1, LED, nodes |

`03-two-nodes` was dropped when node A was rebuilt as a second node B and chained; one GPIO drives both.

## Build

Copy `src/secrets.h.example` to `src/secrets.h` and fill in your WiFi and MQTT broker.

```
pio run -e 01-timing -t upload
pio device monitor
```

`02-level` takes `r g b` lines on the monitor:

```
pio run -e 02-level -t upload
pio device monitor --echo
```

Board: Seeed XIAO ESP32C3 (`seeed_xiao_esp32c3`).

ATtiny13A toolchain for the manual-mode controller: PlatformIO's `atmelavr` platform, installed
globally with `pio pkg install -g -p atmelavr` (brings `toolchain-atmelavr`, avr-gcc 7.3.0, and
`tool-avrdude` 8.1). No Homebrew packages.

`05-isp` flashes whatever `src/attiny_image.h` holds. The ATtiny programs live in `attiny/`;
build one and regenerate the header before building `05-isp`, then type `go` on the monitor:

```
pio run -d attiny -e t1-blink
attiny/hex2h.py t1-blink
pio run -e 05-isp -t upload
```

PlatformIO's default sdkconfig assumes 2 MB flash; the board has 4 MB. Harmless for the bench
programs, which are well under 200 KB.
