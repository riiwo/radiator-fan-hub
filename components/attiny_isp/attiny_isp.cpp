#include "attiny_isp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

// SCK near 25 kHz, under a quarter of the ATtiny's slowest clock.
#define HALF_US 20
#define ATTEMPTS 4

static attiny_isp_pins pins;

// Makes the pin an input.
static void set_float(gpio_num_t pin, bool pullup) {
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&cfg));
}

// Makes the pin an output at level.
static void set_out(gpio_num_t pin, int level) {
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&cfg));
  gpio_set_level(pin, level);
}

// Shifts one ISP byte out and in.
static uint8_t xfer(uint8_t out) {
  uint8_t in = 0;
  for (int bit = 7; bit >= 0; bit--) {
    gpio_set_level(pins.mosi, (out >> bit) & 1);
    esp_rom_delay_us(HALF_US);
    gpio_set_level(pins.sck, 1);
    in = (in << 1) | gpio_get_level(pins.miso);
    esp_rom_delay_us(HALF_US);
    gpio_set_level(pins.sck, 0);
  }
  return in;
}

// Sends a four-byte ISP command; echo is byte three.
static uint8_t command(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t *echo) {
  xfer(a);
  xfer(b);
  *echo = xfer(c);
  return xfer(d);
}

// Drives the ISP lines and holds RESET low.
static void hold_in_reset() {
  set_out(pins.sck, 0);
  set_out(pins.mosi, 0);
  set_float(pins.miso, true);
  set_out(pins.reset, 0);
}

// Floats every ISP line, SCK with the pull-up.
static void release() {
  ESP_ERROR_CHECK(gpio_reset_pin(pins.mosi));
  set_float(pins.sck, true);
  set_float(pins.miso, false);
  set_float(pins.reset, false);
}

// Pulses RESET and sends Programming Enable.
static bool enter_programming() {
  uint8_t echo;
  gpio_set_level(pins.reset, 1);
  esp_rom_delay_us(100);
  gpio_set_level(pins.reset, 0);
  vTaskDelay(pdMS_TO_TICKS(30));
  command(0xAC, 0x53, 0x00, 0x00, &echo);
  return echo == 0x53;
}

// Reads the three signature bytes.
static bool signature_is_attiny13a() {
  uint8_t echo, sig[3];
  for (int i = 0; i < 3; i++) sig[i] = command(0x30, 0x00, i, 0x00, &echo);
  return sig[0] == 0x1E && sig[1] == 0x90 && sig[2] == 0x07;
}

bool attiny_isp_finds_attiny13a(const attiny_isp_pins *isp) {
  pins = *isp;
  hold_in_reset();
  bool found = false;
  for (int attempt = 0; attempt < ATTEMPTS && !found; attempt++) found = enter_programming() && signature_is_attiny13a();
  release();
  return found;
}
