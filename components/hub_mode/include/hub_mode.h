#pragma once

#include "fan_bus.h"
#include "hub_led.h"

// The hub's mode: auto follows the fan curve, low, mid and high are fixed steps. The
// ATtiny's off is auto. Kept in NVS while the XIAO owns it.

enum hub_mode { HUB_AUTO, HUB_LOW, HUB_MID, HUB_HIGH, HUB_MODES };

#define HUB_MODE_UNKNOWN -1

// Mode name, "unknown" for HUB_MODE_UNKNOWN.
const char *hub_mode_name(int mode);
int hub_mode_from_name(const char *name);  // HUB_MODE_UNKNOWN when it is no mode
// The LED colour that shows the mode.
hub_led_colour hub_mode_colour(int mode);

// The XIAO's own mode, from NVS.
void hub_mode_load();
// The XIAO's current mode.
int hub_mode_current();
// Sets the mode and saves it.
void hub_mode_set(int mode);
// Steps auto, low, mid, high, auto.
void hub_mode_next();
fan_step hub_mode_fan_step();  // the mode's step, the fan curve's in auto

// The ATtiny owns the modes: the XIAO's mode stays auto and hub_mode_owned() turns false.
void hub_mode_hand_to_attiny();
bool hub_mode_owned();
