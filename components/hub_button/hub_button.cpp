#include "hub_button.h"

#define DEBOUNCE_MS 30
#define SHORT_MAX_MS 600
#define LONG_MS 1500
#define PRESS_MS 100
#define GAP_MS 200

static gpio_num_t pin;
static bool stable = true, long_sent = false;
static int count_ms = 0, held_ms = 0, presses = 0, press_ms = 0;

void hub_button_init(gpio_num_t button) {
  pin = button;
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
  cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&cfg));
  gpio_set_level(pin, 1);
}

void hub_button_press(int n) {
  presses += n;
}

bool hub_button_pressing() {
  return presses > 0;
}

hub_button_event hub_button_tick(int ms) {
  if (presses > 0) {
    press_ms += ms;
    gpio_set_level(pin, press_ms > PRESS_MS);
    if (press_ms >= PRESS_MS + GAP_MS) {
      presses--;
      press_ms = 0;
    }
  }

  hub_button_event event = HUB_BUTTON_NONE;
  bool up = gpio_get_level(pin);
  if (up == stable) count_ms = 0;
  else if ((count_ms += ms) >= DEBOUNCE_MS) {
    stable = up;
    count_ms = 0;
    if (!up) held_ms = 0;
    else if (long_sent) event = HUB_BUTTON_LONG_RELEASE;
    else if (held_ms <= SHORT_MAX_MS) event = HUB_BUTTON_SHORT;
    if (up) long_sent = false;
  }
  if (!stable) {
    held_ms += ms;
    if (held_ms >= LONG_MS && !long_sent) {
      long_sent = true;
      event = HUB_BUTTON_LONG;
    }
  }
  return event;
}
