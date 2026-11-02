// Program 05: flash the ATtiny13A in place over its ISP pins.
//
// Type "go" on the serial console. The XIAO holds the ATtiny in reset, enters
// serial programming, prints the signature (1E 90 07) and fuses, sets the low
// fuse to 0x7A (9.6 MHz internal, no divide-by-8), erases, writes the image in
// attiny_image.h and reads it back. On a clean verify blue lights on D3. Every
// ISP pin is then released to a floating input, so the ATtiny runs its program.
//
// "blue on" and "blue off" switch D3 by hand; "hold" parks RESET low and SCK,
// MOSI high for the meter; "release" floats the ISP lines again. "watch" logs,
// until the next keystroke, each button press on D7 with its length, each change
// of the mode sense on D5 and the first node's triple of every frame on D1.
//
// ISP lines: D4 RESET, D5 SCK, D7 MISO, D1 MOSI (the bus data line, so the nodes
// see noise while it runs). SCK runs near 25 kHz, under a quarter of the
// ATtiny's factory 1.2 MHz clock.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/rmt_rx.h"

#include "attiny_image.h"

#define PIN_MOSI GPIO_NUM_3
#define PIN_BLUE GPIO_NUM_5
#define PIN_RST GPIO_NUM_6
#define PIN_SCK GPIO_NUM_7
#define PIN_MISO GPIO_NUM_20
#define HALF_US 20
#define PAGE_WORDS 16
#define LOW_FUSE 0x7A

#define RX_RESOLUTION_HZ 10000000
#define RX_SYMBOLS 48
#define ONE_MIN_NS 850

static uint8_t echo;
static rmt_channel_handle_t rx_channel;
static rmt_symbol_word_t rx_buffer[RX_SYMBOLS];
static volatile bool rx_done;
static volatile size_t rx_count;
static volatile int64_t rx_at_us;

static void set_float(gpio_num_t pin) {
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_INPUT;
  ESP_ERROR_CHECK(gpio_config(&cfg));
}

static void set_out(gpio_num_t pin, int level) {
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&cfg));
  gpio_set_level(pin, level);
}

static uint8_t xfer(uint8_t out) {
  uint8_t in = 0;
  for (int bit = 7; bit >= 0; bit--) {
    gpio_set_level(PIN_MOSI, (out >> bit) & 1);
    esp_rom_delay_us(HALF_US);
    gpio_set_level(PIN_SCK, 1);
    in = (in << 1) | gpio_get_level(PIN_MISO);
    esp_rom_delay_us(HALF_US);
    gpio_set_level(PIN_SCK, 0);
  }
  return in;
}

static uint8_t cmd(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  xfer(a);
  xfer(b);
  echo = xfer(c);
  return xfer(d);
}

static void wait_ready() {
  while (cmd(0xF0, 0x00, 0x00, 0x00) & 1) vTaskDelay(1);
}

static bool enter() {
  set_float(PIN_BLUE);
  set_out(PIN_SCK, 0);
  set_out(PIN_MOSI, 0);
  set_float(PIN_MISO);
  gpio_set_pull_mode(PIN_MISO, GPIO_PULLUP_ONLY);
  set_out(PIN_RST, 0);
  for (int attempt = 0; attempt < 4; attempt++) {
    gpio_set_level(PIN_RST, 1);
    esp_rom_delay_us(100);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    cmd(0xAC, 0x53, 0x00, 0x00);
    if (echo == 0x53) return true;
  }
  return false;
}

static void release() {
  set_float(PIN_MOSI);
  set_float(PIN_SCK);
  set_float(PIN_MISO);
  set_float(PIN_RST);
}

static uint8_t read_byte(uint16_t addr) {
  uint16_t word = addr >> 1;
  return cmd(addr & 1 ? 0x28 : 0x20, word >> 8, word & 0xFF, 0x00);
}

static void run() {
  size_t len = sizeof(attiny_image);
  printf("05_isp: image %s, %u bytes\n", attiny_image_name, (unsigned) len);
  if (!enter()) {
    release();
    printf("FAIL: no sync, programming-enable echo 0x%02X, expected 0x53\n", echo);
    return;
  }
  uint8_t sig[3];
  for (int i = 0; i < 3; i++) sig[i] = cmd(0x30, 0x00, i, 0x00);
  printf("signature %02X %02X %02X", sig[0], sig[1], sig[2]);
  if (sig[0] != 0x1E || sig[1] != 0x90 || sig[2] != 0x07) {
    release();
    printf("  FAIL: expected 1E 90 07 (ATtiny13A)\n");
    return;
  }
  printf("  ATtiny13A\n");

  uint8_t low = cmd(0x50, 0x00, 0x00, 0x00), high = cmd(0x58, 0x08, 0x00, 0x00);
  printf("fuses low %02X high %02X\n", low, high);
  if (low != LOW_FUSE) {
    cmd(0xAC, 0xA0, 0x00, LOW_FUSE);
    wait_ready();
    low = cmd(0x50, 0x00, 0x00, 0x00);
    printf("low fuse written, reads %02X%s\n", low, low == LOW_FUSE ? "" : "  FAIL");
    if (low != LOW_FUSE) {
      release();
      return;
    }
  }

  cmd(0xAC, 0x80, 0x00, 0x00);
  wait_ready();
  for (size_t page = 0; page < len; page += PAGE_WORDS * 2) {
    for (size_t i = page; i < len && i < page + PAGE_WORDS * 2; i += 2) {
      uint8_t in_page = (i >> 1) & (PAGE_WORDS - 1);
      cmd(0x40, 0x00, in_page, attiny_image[i]);
      cmd(0x48, 0x00, in_page, attiny_image[i + 1]);
    }
    uint16_t word = page >> 1;
    cmd(0x4C, word >> 8, word & 0xFF, 0x00);
    wait_ready();
  }
  printf("written %u bytes\n", (unsigned) len);

  size_t bad = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t got = read_byte(i);
    if (got != attiny_image[i] && bad++ < 8) printf("  0x%03X: read %02X, expected %02X\n", (unsigned) i, got, attiny_image[i]);
  }
  release();
  if (bad) {
    printf("verify FAIL: %u bytes differ\n", (unsigned) bad);
    return;
  }
  printf("verify OK, ATtiny released and running %s\n", attiny_image_name);
  set_out(PIN_BLUE, 0);
}

static bool on_rx_done(rmt_channel_handle_t, const rmt_rx_done_event_data_t *data, void *) {
  rx_count = data->num_symbols;
  rx_at_us = esp_timer_get_time();
  rx_done = true;
  return false;
}

static void watch_init() {
  rmt_rx_channel_config_t cfg = {};
  cfg.gpio_num = PIN_MOSI;
  cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  cfg.resolution_hz = RX_RESOLUTION_HZ;
  cfg.mem_block_symbols = RX_SYMBOLS;
  ESP_ERROR_CHECK(rmt_new_rx_channel(&cfg, &rx_channel));
  rmt_rx_event_callbacks_t callbacks = {};
  callbacks.on_recv_done = on_rx_done;
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &callbacks, NULL));
  esp_log_level_set("rmt", ESP_LOG_NONE);
}

static void arm(const rmt_receive_config_t &receive) {
  rx_done = false;
  ESP_ERROR_CHECK(rmt_receive(rx_channel, rx_buffer, sizeof(rx_buffer), &receive));
}

static const char *mode_name(const uint8_t t[3]) {
  if (t[0] == 255 && t[1] == 255 && t[2] == 255) return "off";
  if (t[0] == 255 && t[1] == 255 && t[2] == 0) return "low";
  if (t[0] == 255 && t[1] == 0 && t[2] == 255) return "mid";
  if (t[0] == 0 && t[1] == 255 && t[2] == 255) return "high";
  return "?";
}

static void watch() {
  release();
  ESP_ERROR_CHECK(rmt_enable(rx_channel));
  rmt_receive_config_t receive = {};
  receive.signal_range_min_ns = 100;
  receive.signal_range_max_ns = 6000;
  arm(receive);
  printf("watch: D7 button, D5 sense, D1 frames; any key stops\n");
  int64_t start = esp_timer_get_time(), pressed_at = 0, last_frame = 0;
  int button = gpio_get_level(PIN_MISO), sense = gpio_get_level(PIN_SCK);
  printf("%7.2f  button %s, sense %s\n", 0.0, button ? "up" : "down", sense ? "high" : "low");
  uint8_t c = '\n';
  while (c == '\n' || c == '\r') {
    if (usb_serial_jtag_read_bytes(&c, 1, 0) != 1) c = '\n';
    int64_t now = esp_timer_get_time();
    double t = (now - start) / 1e6;
    int b = gpio_get_level(PIN_MISO), s = gpio_get_level(PIN_SCK);
    if (b != button) {
      button = b;
      if (!b) pressed_at = now, printf("%7.2f  button down\n", t);
      else printf("%7.2f  button up after %lld ms\n", t, (long long) ((now - pressed_at) / 1000));
    }
    if (s != sense) {
      sense = s;
      printf("%7.2f  sense %s\n", t, s ? "high" : "low");
    }
    if (rx_done) {
      uint8_t triple[3] = {};
      size_t n = rx_count < 24 ? rx_count : 24;
      uint32_t hi_min = UINT32_MAX, hi_max = 0;
      for (size_t i = 0; i < n; i++) {
        uint32_t ns = rx_buffer[i].duration0 * (1000000000 / RX_RESOLUTION_HZ);
        if (!rx_buffer[i].level0) ns = 0;
        if (ns < hi_min) hi_min = ns;
        if (ns > hi_max) hi_max = ns;
        triple[i / 8] = (triple[i / 8] << 1) | (ns > ONE_MIN_NS);
      }
      double at = (rx_at_us - start) / 1e6;
      printf("%7.2f  frame %02X %02X %02X %-4s  %u symbols, high %lu..%lu ns, %lld ms since last\n", at,
             triple[0], triple[1], triple[2], n == 24 ? mode_name(triple) : "?", (unsigned) rx_count,
             (unsigned long) hi_min, (unsigned long) hi_max, (long long) ((rx_at_us - last_frame) / 1000));
      last_frame = rx_at_us;
      arm(receive);
    }
    vTaskDelay(1);
  }
  ESP_ERROR_CHECK(rmt_disable(rx_channel));
  printf("watch stopped\n");
}

extern "C" void app_main(void) {
  usb_serial_jtag_driver_config_t console = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&console));
  usb_serial_jtag_vfs_use_driver();
  release();
  set_float(PIN_BLUE);
  watch_init();
  printf("\n05_isp: type go to flash %s into the ATtiny13A\n", attiny_image_name);

  char line[16];
  size_t line_len = 0;
  while (true) {
    uint8_t c;
    while (usb_serial_jtag_read_bytes(&c, 1, 0) == 1) {
      if (c == '\n' || c == '\r') {
        if (line_len == 0) continue;
        line[line_len] = 0;
        if (strcmp(line, "go") == 0) run();
        else if (strcmp(line, "hold") == 0) {
          set_out(PIN_RST, 0);
          set_out(PIN_SCK, 1);
          set_out(PIN_MOSI, 1);
          printf("hold: RESET low, SCK high, MOSI high; MISO reads %d\n", gpio_get_level(PIN_MISO));
        } else if (strcmp(line, "blue on") == 0) {
          set_out(PIN_BLUE, 0);
          printf("blue on\n");
        } else if (strcmp(line, "blue off") == 0) {
          set_float(PIN_BLUE);
          printf("blue off\n");
        } else if (strcmp(line, "release") == 0) {
          release();
          printf("released\n");
        } else if (strcmp(line, "watch") == 0) watch();
        else printf("? \"%s\"  usage: go | hold | release | blue on | blue off | watch\n", line);
        line_len = 0;
        continue;
      }
      if (line_len < sizeof(line) - 1) line[line_len++] = (char) c;
    }
    vTaskDelay(1);
  }
}
