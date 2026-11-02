#pragma once

#include "driver/gpio.h"

// The hub's button line, shared with the ATtiny: open drain with the pull-up, so it reads
// real presses and can press the line itself as a finger would.

enum hub_button_event {
  HUB_BUTTON_NONE,
  HUB_BUTTON_SHORT,         // released within SHORT_MAX_MS
  HUB_BUTTON_LONG,          // held for LONG_MS, once per press
  HUB_BUTTON_LONG_RELEASE,  // released after a LONG
};

// Configures the line; call again after anything else has reconfigured the pin.
void hub_button_init(gpio_num_t pin);

// Call every ms_per_tick ms: drives queued presses, then debounces the line.
hub_button_event hub_button_tick(int ms_per_tick);

// Queues n presses of PRESS_MS down and GAP_MS up; they are read back as SHORT events.
void hub_button_press(int n);

// True while queued presses are going out.
bool hub_button_pressing();
