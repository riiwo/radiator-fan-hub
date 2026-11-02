#pragma once

#include "fan_bus.h"
#include "hub_led.h"

// What the XIAO knows of a fitted ATtiny. In low, mid and high the ATtiny holds sense low
// and drives the bus; sense high for TAKE_MS means off. Its mode is followed by counting
// the short presses on the button line, so it is unknown until the ATtiny is first seen off.

enum attiny_link_event { ATTINY_LINK_NONE, ATTINY_LINK_TOOK_BUS, ATTINY_LINK_LEFT_BUS };

// Call every ms_per_tick ms with the XIAO's bus claimed or not; says when to hand over.
attiny_link_event attiny_link_watch(int ms_per_tick);

// A short press stepped the ATtiny's mode.
void attiny_link_count_press();
int attiny_link_mode();                // HUB_MODE_UNKNOWN until first seen off
int attiny_link_presses_to(int mode);  // 0 when there or nothing asked, 1 while the mode is unknown

// The step on the bus: the fan curve's while the XIAO drives, else the ATtiny's mode.
int attiny_link_bus_step();

// Blue while the XIAO runs auto, dark while the ATtiny lights the LED.
hub_led_colour attiny_link_xiao_colour();

// The XIAO alone: true once when sense, held low, goes high for TAKE_MS (an ATtiny powering up).
bool attiny_link_powered_up(int ms_per_tick);
