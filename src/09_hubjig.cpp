// Program 09: hub test jig on the FireBeetle 2 ESP32-E.
//
// A FAN HUB gets its 12 V from the jig on MAINS (relay A) and its FAN_OUT plugs into the jig's XH
// header. The hub's own XIAO runs `controller`; the flow and return probes sit in TEMP IN and
// TEMP OUT and are checked from the XIAO's log. Relay C connects the load resistors on FAN_OUT
// (off for the short test). Every measurement is logged: raw ADC counts, pin mV, line volts, time
// series while lines settle, and every frame the hub sends. Each run ends with one CSV line.
//
// Console (115200 baud), one command per line:
//   run <label> [s]    short test, power, then frames watched for s seconds (default 30)
//   learn <label> [s]  same run on a known-good hub, its values become the reference
//   short              short test only, hub power off
//   read               every channel once
//   power on|off       relay C then A on, A then C off
//   loads on|off       relay C; stays on while A is closed
//   frames <s>         decode and log the hub's frames for s seconds
//   ref                print the learned reference

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

#define PIN_DOUT GPIO_NUM_4
#define PIN_RELAY_A GPIO_NUM_22
#define PIN_RELAY_C GPIO_NUM_21
#define PIN_INJ_MAINS GPIO_NUM_17
#define PIN_INJ_F5 GPIO_NUM_18

#define SAMPLES 64
#define RX_RESOLUTION_HZ 10000000
#define BIT_SPLIT_NS 850

enum { MAINS, F12, F5, FGND, CH_COUNT };
struct Channel {
  const char *name;
  gpio_num_t pin;
  float scale;  // line volts per pin volt
  adc_unit_t unit;
  adc_channel_t ch;
};
static Channel CH[CH_COUNT] = {
  {"MAINS", GPIO_NUM_15, 122.0f / 22.0f * 12.23f / 12.511f, ADC_UNIT_1, ADC_CHANNEL_0},
  {"OUT12", GPIO_NUM_35, 122.0f / 22.0f * 12.21f / 12.433f, ADC_UNIT_1, ADC_CHANNEL_0},
  {"OUT5", GPIO_NUM_39, 69.0f / 22.0f * 5.15f / 5.253f, ADC_UNIT_1, ADC_CHANNEL_0},
  {"OGND", GPIO_NUM_25, 1.0f, ADC_UNIT_1, ADC_CHANNEL_0},
};

enum { OFF, LOW, MID, HIGH, STEPS };
static const char *STEP_NAMES[STEPS] = {"off", "low", "mid", "high"};
static const uint8_t TRIPLES[STEPS][3] = {{255, 255, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}};

enum { SRC_NONE, SRC_MAINS, SRC_F5, SRC_DOUT, SRC_COUNT };
static const char *SRC_NAMES[SRC_COUNT] = {"none", "MAINS", "OUT5", "DOUT"};
static const gpio_num_t SRC_PINS[SRC_COUNT] = {GPIO_NUM_NC, PIN_INJ_MAINS, PIN_INJ_F5, PIN_DOUT};

struct Reading {
  int raw;
  int raw_min, raw_max;
  int pin_mv;
  float line_v;
};

struct Result {
  float shorts[SRC_COUNT][CH_COUNT];
  int dout_level[SRC_COUNT];
  float power[CH_COUNT];
  int frames, bad_frames, nodes;
  int seen[STEPS];
  uint32_t high_min, high_max;
};

static adc_oneshot_unit_handle_t adc[2];
static adc_cali_handle_t cali[2];
static rmt_channel_handle_t rx_channel;
static QueueHandle_t rx_queue;
static rmt_symbol_word_t rx_buffer[512];
static bool rmt_ready = false;
static bool relay_a = false, relay_c = false;
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
  rmt_rx_channel_config_t rc = {};
  rc.gpio_num = PIN_DOUT;
  rc.clk_src = RMT_CLK_SRC_DEFAULT;
  rc.resolution_hz = RX_RESOLUTION_HZ;
  rc.mem_block_symbols = 512;
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rc, &rx_channel));
  rmt_rx_event_callbacks_t cb = {};
  cb.on_recv_done = on_rx_done;
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cb, NULL));
  ESP_ERROR_CHECK(rmt_enable(rx_channel));
  rmt_ready = true;
}

static void rmt_down() {
  if (!rmt_ready) return;
  ESP_ERROR_CHECK(rmt_disable(rx_channel));
  ESP_ERROR_CHECK(rmt_del_channel(rx_channel));
  rmt_ready = false;
}

static void short_test(Result &r) {
  printf("\n== 1. short test, hub unpowered (relays A and C open) ==\n");
  if (relay_a) set_relay(PIN_RELAY_A, false, relay_a, "A");
  if (relay_c) set_relay(PIN_RELAY_C, false, relay_c, "C");
  rmt_down();
  for (int s = 1; s < SRC_COUNT; s++) hiz(SRC_PINS[s]);
  vTaskDelay(pdMS_TO_TICKS(300));
  for (int s = 0; s < SRC_COUNT; s++) {
    printf("  lift %s%s\n", SRC_NAMES[s], s ? " to 3.3 V" : " (baseline, all sources floating)");
    print_header();
    if (s) drive(SRC_PINS[s], 1);
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
    r.dout_level[s] = s == SRC_DOUT ? -1 : gpio_get_level(PIN_DOUT);
    printf("    DOUT pin %s\n", r.dout_level[s] < 0 ? "driven" : r.dout_level[s] ? "high" : "low");
    if (s) {
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
  printf("\n== 2. power: relays C and A closed, loads 2 x 1k on FAN_OUT 12V, 220 R on FAN_OUT 5V ==\n");
  print_header();
  Reading rd[CH_COUNT];
  read_all(rd);
  print_row("before", rd);
  set_relay(PIN_RELAY_C, true, relay_c, "C");
  vTaskDelay(pdMS_TO_TICKS(50));
  set_relay(PIN_RELAY_A, true, relay_a, "A");
  int64_t start = esp_timer_get_time();
  const int at_ms[] = {20, 50, 100, 200, 500, 1000, 2000, 3000};
  for (int t : at_ms) {
    while ((esp_timer_get_time() - start) / 1000 < t) vTaskDelay(1);
    read_all(rd);
    char tag[12];
    snprintf(tag, sizeof(tag), "%d ms", t);
    print_row(tag, rd);
  }
  for (int i = 0; i < CH_COUNT; i++) r.power[i] = rd[i].line_v;
  printf("  drop MAINS->FAN_OUT 12V %+.3f V   FAN_OUT 5V %.3f V   FAN_OUT GND %.3f V\n",
         rd[MAINS].line_v - rd[F12].line_v, rd[F5].line_v, rd[FGND].line_v);
  printf("  load currents: 12V %.1f mA (2 x 1k)   5V %.1f mA (220 R)\n",
         (rd[F12].line_v - rd[FGND].line_v) / 500.0f * 1000.0f, (rd[F5].line_v - rd[FGND].line_v) / 220.0f * 1000.0f);
}

// Decodes every frame for the given time. Each frame: bytes of the first node, symbol count,
// node count, bit timing; changes of step are logged with time so the XIAO's mode changes and
// BTN presses line up with them.
static void frames_watch(Result &r, int seconds) {
  printf("\n== 3. frames on FAN_OUT DOUT for %d s ==\n", seconds);
  printf("  now: step the hub through its modes (XIAO console 'mode low|mid|high|auto' or BTN),\n");
  printf("  and watch the LED: blue auto, green low, yellow mid, red high\n");
  printf("  the hub XIAO must send short frames first: console 'nodes 1' (the receiver holds 512 bits)\n");
  rmt_up();
  rmt_receive_config_t rcv = {};
  rcv.signal_range_min_ns = 100;
  rcv.signal_range_max_ns = 5000;
  r.frames = r.bad_frames = r.nodes = 0;
  memset(r.seen, 0, sizeof(r.seen));
  r.high_min = UINT32_MAX;
  r.high_max = 0;
  int last_step = -2;
  int64_t end = esp_timer_get_time() + (int64_t) seconds * 1000000;
  int64_t last_level = 0;
  bool dumped = false;
  while (esp_timer_get_time() < end) {
    ESP_ERROR_CHECK(rmt_receive(rx_channel, rx_buffer, sizeof(rx_buffer), &rcv));
    rmt_rx_done_event_data_t done;
    if (xQueueReceive(rx_queue, &done, pdMS_TO_TICKS(1500)) != pdTRUE) {
      printf("  [%7.3f] no frame in 1.5 s\n", now_s());
      rmt_disable(rx_channel);
      rmt_enable(rx_channel);
      continue;
    }
    int bits = 0;
    uint8_t b[3] = {};
    for (size_t i = 0; i < done.num_symbols; i++) {
      const rmt_symbol_word_t &s = done.received_symbols[i];
      if (s.level0 != 1) continue;
      uint32_t hi = s.duration0 * (1000000000 / RX_RESOLUTION_HZ);
      if (hi < r.high_min) r.high_min = hi;
      if (hi > r.high_max) r.high_max = hi;
      if (bits < 24) b[bits / 8] |= (hi > BIT_SPLIT_NS) << (7 - bits % 8);
      if (!dumped && bits < 24)
        printf("      bit %2d: high %4lu ns  low %4lu ns\n", bits, (unsigned long) hi,
               (unsigned long) (s.duration1 * (1000000000 / RX_RESOLUTION_HZ)));
      bits++;
    }
    dumped = true;
    r.frames++;
    if (bits % 24) r.bad_frames++;
    if (bits / 24 > r.nodes) r.nodes = bits / 24;
    int step = -1;
    for (int s = 0; s < STEPS; s++)
      if (memcmp(b, TRIPLES[s], 3) == 0) step = s;
    if (step >= 0) r.seen[step]++;
    if (step != last_step) {
      printf("  [%7.3f] frame %d: %d bits (%d nodes), first node %02X %02X %02X = %s\n", now_s(), r.frames, bits, bits / 24,
             b[0], b[1], b[2], step >= 0 ? STEP_NAMES[step] : "UNKNOWN");
      last_step = step;
    }
    if (esp_timer_get_time() - last_level > 5000000) {
      Reading rd[CH_COUNT];
      read_all(rd);
      char tag[16];
      snprintf(tag, sizeof(tag), "%.1f s", now_s());
      print_header();
      print_row(tag, rd);
      last_level = esp_timer_get_time();
    }
  }
  printf("  frames %d, %d not a whole number of nodes, up to %d nodes per frame, high time %lu..%lu ns\n", r.frames,
         r.bad_frames, r.nodes, (unsigned long) r.high_min, (unsigned long) r.high_max);
  printf("  steps seen: off %d  low %d  mid %d  high %d\n", r.seen[OFF], r.seen[LOW], r.seen[MID], r.seen[HIGH]);
}

static void compare(const char *what, float got, float want, float tol, bool &all) {
  bool ok = fabsf(got - want) <= tol;
  if (!ok) all = false;
  printf("    %-28s %8.3f  ref %8.3f  diff %+7.3f  %s\n", what, got, want, got - want, ok ? "ok" : "DIFFERS");
}

static void verdict(const char *label, const Result &r) {
  printf("\n== 4. verdict ==\n");
  bool pass = true;
  float d12 = r.power[MAINS] - r.power[F12];
  printf("  12V drop MAINS->FAN_OUT %.3f V %s\n", d12, fabsf(d12) < 0.1f ? "ok" : "HIGH");
  printf("  FAN_OUT 5V %.3f V %s\n", r.power[F5], r.power[F5] > 4.75f && r.power[F5] < 5.25f ? "ok" : "OUT OF 4.75-5.25");
  printf("  FAN_OUT GND %.3f V %s\n", r.power[FGND], r.power[FGND] < 0.1f ? "ok" : "HIGH");
  if (fabsf(d12) >= 0.1f || r.power[F5] <= 4.75f || r.power[F5] >= 5.25f || r.power[FGND] >= 0.1f) pass = false;
  printf("  frames %d, bad %d: %s\n", r.frames, r.bad_frames, r.frames > 0 && r.bad_frames == 0 ? "ok" : "FAIL");
  if (r.frames == 0 || r.bad_frames) pass = false;
  int kinds = 0;
  for (int s = 0; s < STEPS; s++) kinds += r.seen[s] > 0;
  printf("  distinct steps seen %d of 4 %s\n", kinds, kinds == STEPS ? "ok" : "(step the hub through every mode)");
  if (kinds != STEPS) pass = false;
  if (have_ref) {
    printf("  against the learned hub:\n");
    bool same = true;
    for (int s = 0; s < SRC_COUNT; s++)
      for (int i = 0; i < CH_COUNT; i++) {
        char what[40];
        snprintf(what, sizeof(what), "short %s -> %s", SRC_NAMES[s], CH[i].name);
        compare(what, r.shorts[s][i], ref.shorts[s][i], 0.05f + 0.1f * fabsf(ref.shorts[s][i]), same);
      }
    for (int i = 0; i < CH_COUNT; i++) {
      char what[40];
      snprintf(what, sizeof(what), "power %s", CH[i].name);
      compare(what, r.power[i], ref.power[i], 0.15f, same);
    }
    printf("  matches the learned hub: %s\n", same ? "yes" : "NO, see DIFFERS lines");
    if (!same) pass = false;
  } else {
    printf("  no learned hub yet: short-test values are logged but not judged\n");
  }
  printf("  LED colours and the probes are judged from the XIAO side\n");
  printf("\nRESULT %s %s\n", label, pass ? "PASS" : "FAIL");
  printf("CSV,%s,%s", label, pass ? "PASS" : "FAIL");
  for (int s = 0; s < SRC_COUNT; s++)
    for (int i = 0; i < CH_COUNT; i++) printf(",%.3f", r.shorts[s][i]);
  for (int i = 0; i < CH_COUNT; i++) printf(",%.3f", r.power[i]);
  printf(",%d,%d,%d,%d,%d,%d,%lu,%lu\n", r.frames, r.bad_frames, r.seen[OFF], r.seen[LOW], r.seen[MID], r.seen[HIGH],
         (unsigned long) r.high_min, (unsigned long) r.high_max);
}

static void csv_header() {
  printf("CSVHEAD,label,result");
  for (int s = 0; s < SRC_COUNT; s++)
    for (int i = 0; i < CH_COUNT; i++) printf(",short_%s_%s", SRC_NAMES[s], CH[i].name);
  for (int i = 0; i < CH_COUNT; i++) printf(",power_%s", CH[i].name);
  printf(",frames,bad_frames,seen_off,seen_low,seen_mid,seen_high,high_min_ns,high_max_ns\n");
}

static void run(const char *label, int seconds, bool learn) {
  Result r = {};
  t0 = esp_timer_get_time();
  printf("\n######## %s %s ########\n", learn ? "learn" : "run", label);
  csv_header();
  short_test(r);
  power_test(r);
  frames_watch(r, seconds);
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

static void print_ref() {
  if (!have_ref) {
    printf("no learned hub\n");
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
  printf("\n");
}

static void console_init() {
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0));
  uart_vfs_dev_use_driver(UART_NUM_0);
  setvbuf(stdin, NULL, _IONBF, 0);
}

static void read_line(char *buf, size_t len) {
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
      return;
    }
    if (n < len - 1) buf[n++] = (char) c;
  }
}

extern "C" void app_main(void) {
  console_init();
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(nvs_open("hubjig", NVS_READWRITE, &nvs));
  size_t sz = sizeof(ref);
  esp_err_t err = nvs_get_blob(nvs, "ref", &ref, &sz);
  if (err != ESP_ERR_NVS_NOT_FOUND) ESP_ERROR_CHECK(err);
  have_ref = err == ESP_OK && sz == sizeof(ref);

  gpio_reset_pin(PIN_RELAY_A);
  gpio_set_direction(PIN_RELAY_A, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_RELAY_A, 0);
  gpio_reset_pin(PIN_RELAY_C);
  gpio_set_direction(PIN_RELAY_C, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_RELAY_C, 0);
  for (int s = 1; s < SRC_COUNT; s++) hiz(SRC_PINS[s]);
  rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  adc_init();
  t0 = esp_timer_get_time();

  printf("\n09_hubjig: hub test jig. %s\n", have_ref ? "learned hub present" : "no learned hub yet");
  for (int i = 0; i < CH_COUNT; i++)
    printf("  %-5s GPIO%-2d ADC%d ch%d x%.3f\n", CH[i].name, CH[i].pin, CH[i].unit + 1, CH[i].ch, CH[i].scale);
  printf("commands: run <label> [s] | learn <label> [s] | short | read | power on|off | loads on|off | frames <s> | ref\n");

  char line[64];
  while (true) {
    printf("> ");
    fflush(stdout);
    read_line(line, sizeof(line));
    printf("%s\n", line);
    char arg[48] = "";
    int secs = 30;
    if (sscanf(line, "run %47s %d", arg, &secs) >= 1) run(arg, secs, false);
    else if (sscanf(line, "learn %47s %d", arg, &secs) >= 1) run(arg, secs, true);
    else if (strcmp(line, "short") == 0) {
      Result r = {};
      t0 = esp_timer_get_time();
      short_test(r);
    } else if (strcmp(line, "read") == 0) {
      Reading rd[CH_COUNT];
      print_header();
      read_all(rd);
      print_row("now", rd);
      printf("    relay A %s, relay C %s\n", relay_a ? "closed" : "open", relay_c ? "closed" : "open");
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
    else if (sscanf(line, "frames %d", &secs) == 1) {
      Result r = {};
      frames_watch(r, secs);
    } else if (strcmp(line, "ref") == 0) print_ref();
    else printf("? %s\n", line);
  }
}
