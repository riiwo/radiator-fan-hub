#pragma once

#include "driver/gpio.h"

// Common-anode RGB LED, one cathode per pin, sunk to light and floated to go dark.

enum hub_led_colour { HUB_LED_OFF, HUB_LED_BLUE, HUB_LED_GREEN, HUB_LED_YELLOW, HUB_LED_RED };

// Floats the three pins and starts the 1 ms timer that mixes yellow and times blinks.
void hub_led_init(gpio_num_t red, gpio_num_t green, gpio_num_t blue);

void hub_led_show(hub_led_colour colour);

// Dark for BLINK_MS, then back to the colour shown.
void hub_led_blink();
