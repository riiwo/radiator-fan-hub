// Program 04: soak a chain of two node B circuits.
//
// Cycles both chips through OFF, R, G, B with a fixed dwell, staggered so the
// two nodes are never in the same state. One log line per change. "d <s>" sets
// the dwell in seconds, "p <ms>" the refresh period (0 = send once per change).
// Boots with both nodes OFF for the first dwell.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_timer.h"

#include "ws2811_tx.h"

#define TX_PIN GPIO_NUM_3
#define DWELL_S 600
#define REFRESH_MS 10
#define CHIPS 2

struct State { const char *name; uint8_t rgb[3]; };
static const State STATES[] = {
  {"OFF", {255, 255, 255}},
  {"R  ", {0, 255, 255}},
  {"G  ", {255, 0, 255}},
  {"B  ", {255, 255, 0}},
};
#define NSTATES 4

static uint8_t frame[CHIPS * 3];
static uint32_t dwell_s = DWELL_S;
static uint32_t refresh_ms = REFRESH_MS;
static int step = -1;
static char line[32];
static size_t line_len = 0;

static void build_frame(int s, int idx[CHIPS]) {
  if (s < 0) { for (int c = 0; c < CHIPS; c++) idx[c] = 0; }
  else { idx[0] = s % NSTATES; idx[1] = (s + 2) % NSTATES; }
  for (int c = 0; c < CHIPS; c++) memcpy(&frame[c * 3], STATES[idx[c]].rgb, 3);
}

static void log_state(int idx[CHIPS]) {
  int64_t up = esp_timer_get_time() / 1000000;
  printf("%02lld:%02lld:%02lld  node1=%s  node2=%s  frame", up / 3600, (up / 60) % 60, up % 60,
         STATES[idx[0]].name, STATES[idx[1]].name);
  for (size_t i = 0; i < sizeof(frame); i++) printf(" %u", frame[i]);
  printf("\n");
}

static void apply_line() {
  int v;
  if (sscanf(line, "d %d", &v) == 1 && v > 0) { dwell_s = (uint32_t) v; printf("dwell %lu s\n", (unsigned long) dwell_s); return; }
  if (sscanf(line, "p %d", &v) == 1 && v >= 0) { refresh_ms = (uint32_t) v; printf("refresh %lu ms\n", (unsigned long) refresh_ms); return; }
  printf("? \"%s\"  usage: d <s> | p <ms>\n", line);
}

static void poll_console() {
  uint8_t c;
  while (usb_serial_jtag_read_bytes(&c, 1, 0) == 1) {
    if (c == '\n' || c == '\r') {
      if (line_len == 0) continue;
      line[line_len] = 0;
      apply_line();
      line_len = 0;
      continue;
    }
    if (line_len < sizeof(line) - 1) line[line_len++] = (char) c;
  }
}

extern "C" void app_main(void) {
  printf("\n04_soak: %d node B chips chained on GPIO%d, dwell %d s, refresh %d ms\n", CHIPS, TX_PIN, DWELL_S, REFRESH_MS);
  printf("  cycle OFF R G B, node2 two steps ahead of node1; d <s> sets dwell, p <ms> refresh\n\n");

  usb_serial_jtag_driver_config_t console = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&console));
  usb_serial_jtag_vfs_use_driver();
  ws2811_tx_init(TX_PIN);

  int idx[CHIPS];
  build_frame(step, idx);
  ws2811_tx_send(frame, sizeof(frame), WS2811_LATCH_US);
  log_state(idx);

  int64_t state_start = esp_timer_get_time();
  int64_t last_send = state_start;
  while (true) {
    poll_console();
    int64_t now = esp_timer_get_time();
    if (now - state_start >= (int64_t) dwell_s * 1000000) {
      step++;
      build_frame(step, idx);
      ws2811_tx_send(frame, sizeof(frame), WS2811_LATCH_US);
      log_state(idx);
      state_start = now;
      last_send = now;
    } else if (refresh_ms && now - last_send >= (int64_t) refresh_ms * 1000) {
      ws2811_tx_send(frame, sizeof(frame), WS2811_LATCH_US);
      last_send = now;
    }
    vTaskDelay(1);
  }
}
