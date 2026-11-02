// Program 02: hold one frame on a chain of one or two WS2811.
//
// Type "r g b" or "r g b r g b" (each 0..255) on the serial console and that
// frame is re-sent on GPIO3 every REFRESH_MS until the next command. The first
// triple goes to the first chip in the chain, the second to the next. "p <ms>"
// changes the refresh period; "p 0" sends the frame once and stops refreshing.
// Boots holding 255 on every output, every channel of a node B sinking = off.
// Uses only the ws2811_tx component 01-timing proved.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_timer.h"

#include "ws2811_tx.h"

#define TX_PIN GPIO_NUM_3
#define REFRESH_MS 10

static uint8_t frame[6] = {255, 255, 255, 255, 255, 255};
static size_t frame_len = 6;
static uint32_t refresh_ms = REFRESH_MS;
static bool resend = true;
static char line[32];
static size_t line_len = 0;

static void print_hold() {
  printf("hold");
  for (size_t i = 0; i < frame_len; i++) printf(" %u", frame[i]);
  printf("  (%u chip%s, OUTR OUTG OUTB each)\n", (unsigned) (frame_len / 3), frame_len > 3 ? "s" : "");
}

static void print_refresh() {
  if (refresh_ms == 0) printf("refresh off, frame sent once\n");
  else printf("refresh every %lu ms\n", (unsigned long) refresh_ms);
}

static void apply_line() {
  int v[6], ms;
  if (sscanf(line, "p %d", &ms) == 1 && ms >= 0) {
    refresh_ms = (uint32_t) ms;
    resend = true;
    print_refresh();
    return;
  }
  int n = sscanf(line, "%d %d %d %d %d %d", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
  bool ok = n == 3 || n == 6;
  for (int i = 0; ok && i < n; i++) ok = v[i] >= 0 && v[i] <= 255;
  if (!ok) {
    printf("? \"%s\"  usage: r g b [r g b] (each 0..255) | p <ms> (0 = no refresh)\n", line);
    return;
  }
  frame_len = (size_t) n;
  for (int i = 0; i < n; i++) frame[i] = (uint8_t) v[i];
  resend = true;
  print_hold();
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
  printf("\n02_level: WS2811 chain on GPIO%d, frame re-sent every %d ms\n", TX_PIN, REFRESH_MS);
  printf("  type: r g b [r g b]   (0..255 each; 255 = output sinking, 0 = output released)\n");
  printf("        first triple = first chip in the chain, second triple = next chip\n");
  printf("        p <ms>  refresh period, rounded up to the 10 ms tick; p 0 = send once\n");
  printf("  Stage 2 with 10k pull-ups to +5 on each OUT:\n");
  printf("    255 255 255 -> each OUT < 0.5 V\n");
  printf("      0   0   0 -> each OUT 4.9-5.2 V\n");
  printf("    128 128 128 -> each OUT 2.0-3.0 V DC average\n\n");

  usb_serial_jtag_driver_config_t console = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&console));
  usb_serial_jtag_vfs_use_driver();
  ws2811_tx_init(TX_PIN);
  print_hold();
  print_refresh();

  int64_t last_send = 0;
  while (true) {
    poll_console();
    int64_t now = esp_timer_get_time();
    if (resend || (refresh_ms && now - last_send >= (int64_t) refresh_ms * 1000)) {
      ws2811_tx_send(frame, frame_len, WS2811_LATCH_US);
      last_send = now;
      resend = false;
    }
    vTaskDelay(1);
  }
}
