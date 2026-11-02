// Program 10_pins: checks a loose XIAO ESP32-C3's pins. Each pin is read with the pull-up and
// with the pull-down (stuck high or low shows), then driven high and low in turn while every
// other pin is read with the opposite pull (a bridge to a neighbour shows). Repeats every 5 s.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

struct pin {
  const char *name;
  gpio_num_t gpio;
};

static const pin PINS[] = {
  {"D0", GPIO_NUM_2}, {"D1", GPIO_NUM_3}, {"D2", GPIO_NUM_4}, {"D3", GPIO_NUM_5},
  {"D4", GPIO_NUM_6}, {"D5", GPIO_NUM_7}, {"D6", GPIO_NUM_21}, {"D7", GPIO_NUM_20},
  {"D8", GPIO_NUM_8}, {"D9", GPIO_NUM_9}, {"D10", GPIO_NUM_10},
};
static const int COUNT = sizeof(PINS) / sizeof(PINS[0]);

// Input with only the given pull; returns the level after it settles.
static int read_with_pull(gpio_num_t gpio, bool up) {
  gpio_reset_pin(gpio);
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  gpio_set_pull_mode(gpio, up ? GPIO_PULLUP_ONLY : GPIO_PULLDOWN_ONLY);
  esp_rom_delay_us(500);
  return gpio_get_level(gpio);
}

// Every pin to an input with the given pull.
static void all_pulled(bool up) {
  for (int i = 0; i < COUNT; i++) read_with_pull(PINS[i].gpio, up);
}

// Drives one pin to level; lists the other pins that follow it against their pull.
static int bridges_from(int driven, int level) {
  all_pulled(!level);
  gpio_set_direction(PINS[driven].gpio, GPIO_MODE_OUTPUT);
  gpio_set_level(PINS[driven].gpio, level);
  esp_rom_delay_us(500);
  int found = 0;
  for (int i = 0; i < COUNT; i++) {
    if (i == driven || gpio_get_level(PINS[i].gpio) != level) continue;
    printf("  BRIDGE %s -> %s at %d\n", PINS[driven].name, PINS[i].name, level);
    found++;
  }
  return found;
}

static int stuck_pins() {
  int found = 0;
  for (int i = 0; i < COUNT; i++) {
    int up = read_with_pull(PINS[i].gpio, true), down = read_with_pull(PINS[i].gpio, false);
    if (up && !down) continue;
    printf("  STUCK %s (GPIO%d): pull-up reads %d, pull-down reads %d\n", PINS[i].name, PINS[i].gpio, up, down);
    found++;
  }
  return found;
}

extern "C" void app_main(void) {
  while (true) {
    printf("\n10_pins: %d pins\n", COUNT);
    int stuck = stuck_pins(), bridges = 0;
    for (int i = 0; i < COUNT; i++) bridges += bridges_from(i, 1) + bridges_from(i, 0);
    printf("stuck %d, bridges %d: %s\n", stuck, bridges, stuck + bridges == 0 ? "PASS" : "FAIL");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
