// Program 08: node test jig on the FireBeetle 2 ESP32-E.
//
// A FAN CONTROLLER plugs into the jig on XH-A (its INPUT), XH-B (its OUTPUT) and FAN. Relay A
// switches the node's 12 V (the buck makes its 5 V from it), relay B ties OUTPUT GND to the jig
// GND for the hub check, relay C connects the load resistors (off for the short test). Every measurement is logged: raw ADC counts, pin mV, line volts, and
// time series while lines settle, so boards and board variants can be compared afterwards.
// Each run ends with one CSV line.
//
// Console (115200 baud), one command per line:
//   run <label>    short test, power, steps, forwarding; compared with the learned node if any
//   learn <label>  same run on a known-good node, its values become the reference
//   short          short test only, node power off
//   read           every channel once, relays as they are
//   power on|off   relay C then A on, A then C off
//   loads on|off   relay C; stays on while A is closed
//   step off|low|mid|high
//   din 0|1        DIN held at a static level, for a meter on the node
//   hub <seconds>  hub check: hub FAN_OUT on XH-B, relay B on, frames decoded and logged
//   ref            print the learned reference

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "ws2811_tx.h"

#define PIN_DIN GPIO_NUM_25
#define PIN_DOUT GPIO_NUM_26
#define PIN_RELAY_A GPIO_NUM_22
#define PIN_RELAY_B GPIO_NUM_16
#define PIN_RELAY_C GPIO_NUM_21
#define PIN_INJ_IN12 GPIO_NUM_17
#define PIN_INJ_IN5 GPIO_NUM_18
#define PIN_INJ_FAN GPIO_NUM_19

// a stopped fan's electronics against the 122k FAN- divider leave ~1.0 V across it in off
#define FAN_OFF_MAX 2.0f
#define SAMPLES 64
#define RX_RESOLUTION_HZ 10000000
#define BIT_SPLIT_NS 850

enum { IN12, OUT12, FANM, IN5, OUT5, OGND, CH_COUNT };
struct Channel {
  const char *name;
  gpio_num_t pin;
  float scale;  // line volts per pin volt
  adc_unit_t unit;
  adc_channel_t ch;
};
static Channel CH[CH_COUNT] = {
  {"IN12", GPIO_NUM_15, 122.0f / 22.0f, ADC_UNIT_1, ADC_CHANNEL_0},
  {"OUT12", GPIO_NUM_35, 122.0f / 22.0f, ADC_UNIT_1, ADC_CHANNEL_0},
  {"FAN-", GPIO_NUM_34, 122.0f / 22.0f, ADC_UNIT_1, ADC_CHANNEL_0},
  {"IN5", GPIO_NUM_39, 69.0f / 22.0f, ADC_UNIT_1, ADC_CHANNEL_0},
  {"OUT5", GPIO_NUM_36, 69.0f / 22.0f, ADC_UNIT_1, ADC_CHANNEL_0},
  {"OGND", GPIO_NUM_4, 1.0f, ADC_UNIT_1, ADC_CHANNEL_0},
};

enum { OFF, LOW, MID, HIGH, STEPS };
static const char *STEP_NAMES[STEPS] = {"off", "low", "mid", "high"};
static const uint8_t TRIPLES[STEPS][3] = {{255, 255, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}};
static const uint8_t PROBE[3] = {0xA5, 0x5A, 0x3C};

// Short test sources: each lifts one line to 3.3 V through its resistor while all lines are read.
enum { SRC_NONE, SRC_IN12, SRC_IN5, SRC_FAN, SRC_DIN, SRC_DOUT, SRC_COUNT };
static const char *SRC_NAMES[SRC_COUNT] = {"none", "IN12", "IN5", "FAN-", "DIN", "DOUT"};
static const gpio_num_t SRC_PINS[SRC_COUNT] = {GPIO_NUM_NC, PIN_INJ_IN12, PIN_INJ_IN5, PIN_INJ_FAN, PIN_DIN, PIN_DOUT};

struct Reading {
  int raw;
  int raw_min, raw_max;
  int pin_mv;
  float line_v;
};

// What a run measures; the learned node's copy is kept in NVS.
struct Result {
  float shorts[SRC_COUNT][CH_COUNT];
  int din_level[SRC_COUNT], dout_level[SRC_COUNT];
  float power[CH_COUNT];
  float fan_v[5];
  float in12_at_step[5];
  int fwd_ok, fwd_tries;
  uint8_t fwd_bytes[3];
};

static adc_oneshot_unit_handle_t adc[2];
static adc_cali_handle_t cali[2];
static rmt_channel_handle_t rx_channel;
static QueueHandle_t rx_queue;
static rmt_symbol_word_t rx_buffer[64];
static bool rmt_ready = false;
static bool relay_a = false, relay_b = false, relay_c = false;
static nvs_handle_t nvs;
static Result ref;
static bool have_ref = false;
static int64_t t0;

static float now_s() { return (esp_timer_get_time() - t0) / 1e6f; }

static void adc_init() {
  for (int u = 0; u < 2; u++) {
    adc_oneshot_unit_init_cfg_t cfg = {};
    cfg.unit_id = u == 0 ? ADC_UNIT_1 : ADC_UNIT_2;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg, &adc[u]));
    adc_cali_line_fitting_config_t cc = {};
    cc.unit_id = cfg.unit_id;
    cc.atten = ADC_ATTEN_DB_12;
    cc.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cc, &cali[u]));
  }
  for (int i = 0; i < CH_COUNT; i++) {
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(CH[i].pin, &CH[i].unit, &CH[i].ch));
    adc_oneshot_chan_cfg_t cc = {};
    cc.atten = ADC_ATTEN_DB_12;
    cc.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc[CH[i].unit], CH[i].ch, &cc));
  }
}

static Reading read_ch(int i) {
  int u = CH[i].unit;
  long sum = 0;
  Reading r = {0, 4095, 0, 0, 0};
  for (int s = 0; s < SAMPLES; s++) {
    int v;
    ESP_ERROR_CHECK(adc_oneshot_read(adc[u], CH[i].ch, &v));
    sum += v;
    if (v < r.raw_min) r.raw_min = v;
    if (v > r.raw_max) r.raw_max = v;
  }
  r.raw = (int) (sum / SAMPLES);
  ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali[u], r.raw, &r.pin_mv));
  if (r.raw == 0) r.pin_mv = 0;  // calibration maps raw 0 to its ~130 mV floor
  r.line_v = r.pin_mv / 1000.0f * CH[i].scale;
  return r;
}

static void read_all(Reading out[CH_COUNT]) {
  for (int i = 0; i < CH_COUNT; i++) out[i] = read_ch(i);
}

static void print_header() {
  printf("    %7s", "t s");
  for (int i = 0; i < CH_COUNT; i++) printf(" | %-22s", CH[i].name);
  printf("\n    %7s", "");
  for (int i = 0; i < CH_COUNT; i++) printf(" | %-22s", "line V  pin mV  raw(min-max)");
  printf("\n");
}

static void print_row(const char *tag, const Reading r[CH_COUNT]) {
  printf("    %7s", tag);
  for (int i = 0; i < CH_COUNT; i++)
    printf(" | %6.3f %5d %4d(%d-%d)", r[i].line_v, r[i].pin_mv, r[i].raw, r[i].raw_min, r[i].raw_max);
  printf("\n");
}

static void set_relay(gpio_num_t pin, bool on, bool &state, const char *name) {
  gpio_set_level(pin, on);
  state = on;
  printf("  [%7.3f] relay %s %s\n", now_s(), name, on ? "closed" : "open");
}

static void hiz(gpio_num_t pin) {
  gpio_reset_pin(pin);
  gpio_set_direction(pin, GPIO_MODE_INPUT);
  gpio_set_pull_mode(pin, GPIO_FLOATING);
}

static void drive(gpio_num_t pin, int level) {
  gpio_reset_pin(pin);
  gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_level(pin, level);
}

static bool on_rx_done(rmt_channel_handle_t, const rmt_rx_done_event_data_t *data, void *) {
  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(rx_queue, data, &woken);
  return woken == pdTRUE;
}

static void rmt_up() {
  if (rmt_ready) return;
  ws2811_tx_init(PIN_DIN);
  rmt_rx_channel_config_t rc = {};
  rc.gpio_num = PIN_DOUT;
  rc.clk_src = RMT_CLK_SRC_DEFAULT;
  rc.resolution_hz = RX_RESOLUTION_HZ;
  rc.mem_block_symbols = 64;
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rc, &rx_channel));
  rmt_rx_event_callbacks_t cb = {};
  cb.on_recv_done = on_rx_done;
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cb, NULL));
  ESP_ERROR_CHECK(rmt_enable(rx_channel));
  rmt_ready = true;
}

// Gives DIN and DOUT back to plain GPIO for the short test.
static void rmt_down() {
  if (!rmt_ready) return;
  ws2811_tx_deinit();
  ESP_ERROR_CHECK(rmt_disable(rx_channel));
  ESP_ERROR_CHECK(rmt_del_channel(rx_channel));
  rmt_ready = false;
}

static void send_step(int step, bool log) {
  uint8_t f[3];
  memcpy(f, TRIPLES[step], 3);
  ws2811_tx_send(f, 3, WS2811_LATCH_US * 5);
  if (log) printf("  [%7.3f] frame %s: %3d %3d %3d\n", now_s(), STEP_NAMES[step], f[0], f[1], f[2]);
}

// Sends the node's triple followed by PROBE; the node keeps the first and must pass PROBE to DOUT.
// Returns the number of bits decoded; bytes and per-bit timing go to the log when dump is set.
static int forward_once(int step, uint8_t got[3], bool dump) {
  rmt_receive_config_t rcv = {};
  rcv.signal_range_min_ns = 100;
  rcv.signal_range_max_ns = 5000;
  ESP_ERROR_CHECK(rmt_receive(rx_channel, rx_buffer, sizeof(rx_buffer), &rcv));
  uint8_t f[6];
  memcpy(f, TRIPLES[step], 3);
  memcpy(f + 3, PROBE, 3);
  ws2811_tx_send(f, 6, WS2811_LATCH_US * 5);
  rmt_rx_done_event_data_t done;
  memset(got, 0, 3);
  if (xQueueReceive(rx_queue, &done, pdMS_TO_TICKS(50)) != pdTRUE) {
    if (dump) printf("      DOUT: nothing received\n");
    rmt_disable(rx_channel);
    rmt_enable(rx_channel);
    return 0;
  }
  int bits = 0;
  uint32_t hmin = UINT32_MAX, hmax = 0, lmin = UINT32_MAX, lmax = 0;
  for (size_t i = 0; i < done.num_symbols && bits < 24; i++) {
    const rmt_symbol_word_t &s = done.received_symbols[i];
    if (s.level0 != 1) continue;
    uint32_t hi = s.duration0 * (1000000000 / RX_RESOLUTION_HZ);
    uint32_t lo = s.duration1 * (1000000000 / RX_RESOLUTION_HZ);
    int bit = hi > BIT_SPLIT_NS;
    got[bits / 8] |= bit << (7 - bits % 8);
    if (hi < hmin) hmin = hi;
    if (hi > hmax) hmax = hi;
    if (lo && lo < lmin) lmin = lo;
    if (lo > lmax) lmax = lo;
    if (dump) printf("      bit %2d: high %4lu ns  low %4lu ns  -> %d\n", bits, (unsigned long) hi, (unsigned long) lo, bit);
    bits++;
  }
  if (dump) printf("      DOUT: %d symbols, %d bits, high %lu..%lu ns, low %lu..%lu ns, bytes %02X %02X %02X\n",
                   (int) done.num_symbols, bits, (unsigned long) hmin, (unsigned long) hmax,
                   (unsigned long) lmin, (unsigned long) lmax, got[0], got[1], got[2]);
  return bits;
}

static void short_test(Result &r) {
  printf("\n== 1. short test, node unpowered ==\n");
  if (relay_a) set_relay(PIN_RELAY_A, false, relay_a, "A");
  if (relay_b) set_relay(PIN_RELAY_B, false, relay_b, "B");
  if (relay_c) set_relay(PIN_RELAY_C, false, relay_c, "C");
  rmt_down();
  for (int s = 1; s < SRC_COUNT; s++) hiz(SRC_PINS[s]);
  vTaskDelay(pdMS_TO_TICKS(300));
  for (int s = 0; s < SRC_COUNT; s++) {
    // DIN is not lifted: the jig's 10k in series with DIN is bridged (it rounded the 500 ns
    // pulses so no frame decoded), and GPIO25 straight into an unpowered WS2811 would feed the
    // node's 5 V rail through the chip. Its row is read with DIN floating
    bool lift = s && s != SRC_DIN;
    printf("  lift %s%s\n", SRC_NAMES[s], !s ? " (baseline, all sources floating)" : lift ? " to 3.3 V" : " skipped, read floating");
    print_header();
    if (lift) drive(SRC_PINS[s], 1);
    Reading rd[CH_COUNT];
    const int at_ms[] = {50, 200, 500, 1000, 2000};
    int waited = 0;
    for (int t : at_ms) {
      vTaskDelay(pdMS_TO_TICKS(t - waited));
      waited = t;
      read_all(rd);
      char tag[12];
      snprintf(tag, sizeof(tag), "%d ms", t);
      print_row(tag, rd);
    }
    for (int i = 0; i < CH_COUNT; i++) r.shorts[s][i] = rd[i].line_v;
    r.din_level[s] = lift && s == SRC_DIN ? -1 : gpio_get_level(PIN_DIN);
    r.dout_level[s] = s == SRC_DOUT ? -1 : gpio_get_level(PIN_DOUT);
    printf("    DIN pin %s, DOUT pin %s\n",
           r.din_level[s] < 0 ? "driven" : r.din_level[s] ? "high" : "low",
           r.dout_level[s] < 0 ? "driven" : r.dout_level[s] ? "high" : "low");
    if (lift) {
      drive(SRC_PINS[s], 0);
      vTaskDelay(pdMS_TO_TICKS(1500));
      Reading after[CH_COUNT];
      read_all(after);
      print_row("drain", after);
      hiz(SRC_PINS[s]);
    }
  }
}

static void power_test(Result &r) {
  printf("\n== 2. power: relays C and A closed, loads 2 x 1k on OUTPUT 12V, 220 R on OUTPUT 5V ==\n");
  print_header();
  Reading rd[CH_COUNT];
  read_all(rd);
  print_row("before", rd);
  set_relay(PIN_RELAY_C, true, relay_c, "C");
  vTaskDelay(pdMS_TO_TICKS(50));
  set_relay(PIN_RELAY_A, true, relay_a, "A");
  int64_t start = esp_timer_get_time();
  const int at_ms[] = {20, 50, 100, 200, 500, 1000, 2000};
  for (int t : at_ms) {
    while ((esp_timer_get_time() - start) / 1000 < t) vTaskDelay(1);
    read_all(rd);
    char tag[12];
    snprintf(tag, sizeof(tag), "%d ms", t);
    print_row(tag, rd);
  }
  for (int i = 0; i < CH_COUNT; i++) r.power[i] = rd[i].line_v;
  printf("  drop 12V IN-OUT %+.3f V   5V IN-OUT %+.3f V   OUTPUT GND %.3f V\n",
         rd[IN12].line_v - rd[OUT12].line_v, rd[IN5].line_v - rd[OUT5].line_v, rd[OGND].line_v);
  printf("  load currents: 12V %.1f mA (2 x 1k)   5V %.1f mA (220 R)\n",
         (rd[OUT12].line_v - rd[OGND].line_v) / 500.0f * 1000.0f, (rd[OUT5].line_v - rd[OGND].line_v) / 220.0f * 1000.0f);
}

static void steps_test(Result &r) {
  printf("\n== 3. steps: fan voltage = IN12 - FAN- ==\n");
  rmt_up();
  const int seq[5] = {OFF, LOW, MID, HIGH, OFF};
  for (int k = 0; k < 5; k++) {
    send_step(seq[k], true);
    print_header();
    int64_t start = esp_timer_get_time();
    const int at_ms[] = {100, 300, 600, 1000, 2000, 3000};
    Reading rd[CH_COUNT];
    for (int t : at_ms) {
      while ((esp_timer_get_time() - start) / 1000 < t) vTaskDelay(1);
      read_all(rd);
      char tag[24];
      snprintf(tag, sizeof(tag), "%d ms", t);
      print_row(tag, rd);
      printf("            fan %.3f V\n", rd[IN12].line_v - rd[FANM].line_v);
      send_step(seq[k], false);
    }
    r.fan_v[k] = rd[IN12].line_v - rd[FANM].line_v;
    r.in12_at_step[k] = rd[IN12].line_v;
    printf("  step %d %-4s fan %.3f V  IN12 %.3f V\n", k + 1, STEP_NAMES[seq[k]], r.fan_v[k], r.in12_at_step[k]);
  }
}

static void forward_test(Result &r) {
  printf("\n== 4. forwarding: node triple + %02X %02X %02X, DOUT decoded ==\n", PROBE[0], PROBE[1], PROBE[2]);
  rmt_up();
  r.fwd_ok = 0;
  r.fwd_tries = 10;
  for (int i = 0; i < r.fwd_tries; i++) {
    uint8_t got[3];
    int bits = forward_once(OFF, got, i == 0);
    bool ok = bits == 24 && memcmp(got, PROBE, 3) == 0;
    if (ok) r.fwd_ok++;
    if (i == 0) memcpy(r.fwd_bytes, got, 3);
    printf("    try %2d: %2d bits  %02X %02X %02X  %s\n", i + 1, bits, got[0], got[1], got[2], ok ? "match" : "MISMATCH");
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  printf("  forwarded correctly %d / %d\n", r.fwd_ok, r.fwd_tries);
}

static void compare(const char *what, float got, float want, float tol, bool &all) {
  bool ok = fabsf(got - want) <= tol;
  if (!ok) all = false;
  printf("    %-28s %8.3f  ref %8.3f  diff %+7.3f  %s\n", what, got, want, got - want, ok ? "ok" : "DIFFERS");
}

static void verdict(const char *label, const Result &r) {
  printf("\n== 5. verdict ==\n");
  bool pass = true;
  float d12 = r.power[IN12] - r.power[OUT12], d5 = r.power[IN5] - r.power[OUT5];
  printf("  12V drop %.3f V %s\n", d12, fabsf(d12) < 0.1f ? "ok" : "HIGH");
  printf("  5V drop  %.3f V %s\n", d5, fabsf(d5) < 0.1f ? "ok" : "HIGH");
  printf("  GND drop %.3f V %s\n", r.power[OGND], r.power[OGND] < 0.1f ? "ok" : "HIGH");
  if (fabsf(d12) >= 0.1f || fabsf(d5) >= 0.1f || r.power[OGND] >= 0.1f) pass = false;
  bool order = r.fan_v[0] < FAN_OFF_MAX && r.fan_v[1] > r.fan_v[0] + 1.0f && r.fan_v[2] > r.fan_v[1] + 0.5f &&
               r.fan_v[3] > r.fan_v[2] + 0.5f && r.fan_v[4] < FAN_OFF_MAX;
  printf("  fan off %.2f  low %.2f  mid %.2f  high %.2f  off %.2f V: %s\n", r.fan_v[0], r.fan_v[1], r.fan_v[2],
         r.fan_v[3], r.fan_v[4], order ? "rising, off is off" : "NOT IN ORDER");
  if (!order) {
    pass = false;
    if (r.fan_v[0] >= FAN_OFF_MAX || r.fan_v[4] >= FAN_OFF_MAX) printf("    fan runs in off: a 2N7000 shorted, or a WS2811 output open\n");
    if (r.fan_v[1] <= r.fan_v[0] + 1.0f) printf("    low dead: B_2N7K / Z_6V2, or WS2811 blue output stuck\n");
    if (r.fan_v[2] <= r.fan_v[1] + 0.5f) printf("    mid not above low: G_2N7K / Z_2V4, or WS2811 green output stuck\n");
    if (r.fan_v[3] <= r.fan_v[2] + 0.5f) printf("    high not above mid: R_2N7K, or WS2811 red output stuck\n");
    printf("    to tell 2N7000 from WS2811: meter on that 2N7000's G leg with the step on, ~5 V = 2N7000, ~0 V = WS2811\n");
  }
  printf("  forwarding %d / %d %s\n", r.fwd_ok, r.fwd_tries, r.fwd_ok == r.fwd_tries ? "ok" : "FAIL: WS2811 or DIN/DOUT trace");
  if (r.fwd_ok != r.fwd_tries) pass = false;
  if (have_ref) {
    printf("  against the learned node:\n");
    bool same = true;
    for (int s = 0; s < SRC_COUNT; s++)
      for (int i = 0; i < CH_COUNT; i++) {
        char what[40];
        snprintf(what, sizeof(what), "short %s -> %s", SRC_NAMES[s], CH[i].name);
        // under ~200 mV at the pin the ADC sits in its dead zone: a raw count of a few already
        // calibrates to ~150 mV, 0.8 V on a 12 V line. Tolerance is at least that much at the pin;
        // a real short lifts a line to ~3 V
        compare(what, r.shorts[s][i], ref.shorts[s][i],
                fmaxf(0.15f + 0.1f * fabsf(ref.shorts[s][i]), 0.2f * CH[i].scale), same);
      }
    for (int k = 0; k < 5; k++) {
      char what[40];
      snprintf(what, sizeof(what), "fan step %d", k + 1);
      compare(what, r.fan_v[k], ref.fan_v[k], 0.5f, same);
    }
    printf("  matches the learned node: %s\n", same ? "yes" : "NO, see DIFFERS lines");
    if (!same) pass = false;
  } else {
    printf("  no learned node yet: short-test values are logged but not judged\n");
  }
  printf("\nRESULT %s %s\n", label, pass ? "PASS" : "FAIL");
  printf("CSV,%s,%s", label, pass ? "PASS" : "FAIL");
  for (int s = 0; s < SRC_COUNT; s++)
    for (int i = 0; i < CH_COUNT; i++) printf(",%.3f", r.shorts[s][i]);
  for (int i = 0; i < CH_COUNT; i++) printf(",%.3f", r.power[i]);
  for (int k = 0; k < 5; k++) printf(",%.3f", r.fan_v[k]);
  printf(",%d/%d,%02X%02X%02X\n", r.fwd_ok, r.fwd_tries, r.fwd_bytes[0], r.fwd_bytes[1], r.fwd_bytes[2]);
}

static void csv_header() {
  printf("CSVHEAD,label,result");
  for (int s = 0; s < SRC_COUNT; s++)
    for (int i = 0; i < CH_COUNT; i++) printf(",short_%s_%s", SRC_NAMES[s], CH[i].name);
  for (int i = 0; i < CH_COUNT; i++) printf(",power_%s", CH[i].name);
  for (int k = 0; k < 5; k++) printf(",fan_%d", k + 1);
  printf(",forward,fwd_bytes\n");
}

static void run(const char *label, bool learn) {
  Result r = {};
  t0 = esp_timer_get_time();
  printf("\n######## %s %s ########\n", learn ? "learn" : "run", label);
  csv_header();
  short_test(r);
  power_test(r);
  steps_test(r);
  forward_test(r);
  send_step(OFF, true);
  set_relay(PIN_RELAY_A, false, relay_a, "A");
  set_relay(PIN_RELAY_C, false, relay_c, "C");
  if (learn) {
    ref = r;
    have_ref = true;
    ESP_ERROR_CHECK(nvs_set_blob(nvs, "ref", &ref, sizeof(ref)));
    ESP_ERROR_CHECK(nvs_commit(nvs));
    printf("\nlearned: %s is the reference from now on\n", label);
  }
  verdict(label, r);
  printf("######## end %s ########\n", label);
}

static void hub_check(int seconds) {
  printf("\n######## hub check %d s: hub FAN_OUT on XH-B ########\n", seconds);
  t0 = esp_timer_get_time();
  if (relay_a) set_relay(PIN_RELAY_A, false, relay_a, "A");
  if (relay_c) set_relay(PIN_RELAY_C, false, relay_c, "C");
  set_relay(PIN_RELAY_B, true, relay_b, "B");
  rmt_up();
  print_header();
  rmt_receive_config_t rcv = {};
  rcv.signal_range_min_ns = 100;
  rcv.signal_range_max_ns = 5000;
  int64_t end = esp_timer_get_time() + (int64_t) seconds * 1000000;
  int frames = 0, dumped = 0;
  while (esp_timer_get_time() < end) {
    ESP_ERROR_CHECK(rmt_receive(rx_channel, rx_buffer, sizeof(rx_buffer), &rcv));
    rmt_rx_done_event_data_t done;
    if (xQueueReceive(rx_queue, &done, pdMS_TO_TICKS(1500)) != pdTRUE) {
      Reading rd[CH_COUNT];
      read_all(rd);
      char tag[16];
      snprintf(tag, sizeof(tag), "%.1f s", now_s());
      print_row(tag, rd);
      printf("            no frame in 1.5 s\n");
      rmt_disable(rx_channel);
      rmt_enable(rx_channel);
      continue;
    }
    frames++;
    uint8_t b[8] = {};
    int bits = 0;
    for (size_t i = 0; i < done.num_symbols && bits < 64; i++) {
      const rmt_symbol_word_t &s = done.received_symbols[i];
      if (s.level0 != 1) continue;
      uint32_t hi = s.duration0 * (1000000000 / RX_RESOLUTION_HZ);
      b[bits / 8] |= (hi > BIT_SPLIT_NS) << (7 - bits % 8);
      if (dumped < 1) printf("      bit %2d: high %4lu ns low %4lu ns\n", bits, (unsigned long) hi,
                             (unsigned long) (s.duration1 * (1000000000 / RX_RESOLUTION_HZ)));
      bits++;
    }
    dumped++;
    Reading rd[CH_COUNT];
    read_all(rd);
    char tag[16];
    snprintf(tag, sizeof(tag), "%.1f s", now_s());
    print_row(tag, rd);
    printf("            frame %d: %d symbols, first bytes %02X %02X %02X", frames, (int) done.num_symbols, b[0], b[1], b[2]);
    for (int s = 0; s < STEPS; s++)
      if (bits >= 24 && memcmp(b, TRIPLES[s], 3) == 0) printf("  = step %s", STEP_NAMES[s]);
    printf("\n");
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  set_relay(PIN_RELAY_B, false, relay_b, "B");
  printf("######## end hub check: %d frames seen ########\n", frames);
}

static void print_ref() {
  if (!have_ref) {
    printf("no learned node\n");
    return;
  }
  printf("learned reference:\n");
  for (int s = 0; s < SRC_COUNT; s++) {
    printf("  short lift %-5s:", SRC_NAMES[s]);
    for (int i = 0; i < CH_COUNT; i++) printf(" %s %.3f", CH[i].name, ref.shorts[s][i]);
    printf("\n");
  }
  printf("  power:");
  for (int i = 0; i < CH_COUNT; i++) printf(" %s %.3f", CH[i].name, ref.power[i]);
  printf("\n  fan: %.3f %.3f %.3f %.3f %.3f\n", ref.fan_v[0], ref.fan_v[1], ref.fan_v[2], ref.fan_v[3], ref.fan_v[4]);
}

static void console_init() {
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0));
  uart_vfs_dev_use_driver(UART_NUM_0);
  setvbuf(stdin, NULL, _IONBF, 0);
}

static bool read_line(char *buf, size_t len) {
  size_t n = 0;
  while (true) {
    int c = fgetc(stdin);
    if (c == EOF) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (c == '\r' || c == '\n') {
      if (n == 0) continue;
      buf[n] = 0;
      return true;
    }
    if (n < len - 1) buf[n++] = (char) c;
  }
}

extern "C" void app_main(void) {
  console_init();
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(nvs_open("jig", NVS_READWRITE, &nvs));
  size_t sz = sizeof(ref);
  esp_err_t err = nvs_get_blob(nvs, "ref", &ref, &sz);
  if (err != ESP_ERR_NVS_NOT_FOUND) ESP_ERROR_CHECK(err);
  have_ref = err == ESP_OK && sz == sizeof(ref);

  gpio_reset_pin(PIN_RELAY_A);
  gpio_set_direction(PIN_RELAY_A, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_RELAY_A, 0);
  gpio_reset_pin(PIN_RELAY_B);
  gpio_set_direction(PIN_RELAY_B, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_RELAY_B, 0);
  gpio_reset_pin(PIN_RELAY_C);
  gpio_set_direction(PIN_RELAY_C, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_RELAY_C, 0);
  for (int s = 1; s < SRC_COUNT; s++) hiz(SRC_PINS[s]);
  rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  adc_init();
  t0 = esp_timer_get_time();

  printf("\n08_jig: node test jig. %s\n", have_ref ? "learned node present" : "no learned node yet");
  for (int i = 0; i < CH_COUNT; i++)
    printf("  %-5s GPIO%-2d ADC%d ch%d x%.3f\n", CH[i].name, CH[i].pin, CH[i].unit + 1, CH[i].ch, CH[i].scale);
  printf("commands: run <label> | learn <label> | short | read | power on|off | loads on|off | step off|low|mid|high | hub <s> | ref\n");

  char line[64];
  while (true) {
    printf("> ");
    fflush(stdout);
    read_line(line, sizeof(line));
    printf("%s\n", line);
    char arg[48] = "";
    if (sscanf(line, "run %47s", arg) == 1) run(arg, false);
    else if (sscanf(line, "learn %47s", arg) == 1) run(arg, true);
    else if (strcmp(line, "short") == 0) {
      Result r = {};
      t0 = esp_timer_get_time();
      short_test(r);
    } else if (strcmp(line, "read") == 0) {
      Reading rd[CH_COUNT];
      print_header();
      read_all(rd);
      print_row("now", rd);
      printf("    relay A %s, relay B %s, relay C %s\n", relay_a ? "closed" : "open", relay_b ? "closed" : "open", relay_c ? "closed" : "open");
    } else if (strcmp(line, "power on") == 0) {
      // the three loads share one return: with relay C open, OUT 12V feeds OUT 5V through
      // 1k || 1k + 220R and lifts the 5 V rail to ~6.5 V. Loads close before power, open after
      set_relay(PIN_RELAY_C, true, relay_c, "C");
      set_relay(PIN_RELAY_A, true, relay_a, "A");
    } else if (strcmp(line, "power off") == 0) {
      set_relay(PIN_RELAY_A, false, relay_a, "A");
      set_relay(PIN_RELAY_C, false, relay_c, "C");
    } else if (strcmp(line, "loads on") == 0) set_relay(PIN_RELAY_C, true, relay_c, "C");
    else if (strcmp(line, "loads off") == 0) {
      if (relay_a) printf("  loads stay on while relay A is closed\n");
      else set_relay(PIN_RELAY_C, false, relay_c, "C");
    }
    else if (strcmp(line, "din 0") == 0 || strcmp(line, "din 1") == 0) {
      // static DIN level for a meter on the node's WS2811 pin 6
      rmt_down();
      gpio_reset_pin(PIN_DIN);
      gpio_set_direction(PIN_DIN, GPIO_MODE_OUTPUT);
      gpio_set_level(PIN_DIN, line[4] - '0');
      printf("  DIN held %s\n", line[4] == '1' ? "high" : "low");
    }
    else if (sscanf(line, "step %47s", arg) == 1) {
      rmt_up();
      for (int s = 0; s < STEPS; s++)
        if (strcmp(arg, STEP_NAMES[s]) == 0) send_step(s, true);
    } else if (sscanf(line, "hub %47s", arg) == 1) hub_check(atoi(arg));
    else if (strcmp(line, "ref") == 0) print_ref();
    else printf("? %s\n", line);
  }
}
