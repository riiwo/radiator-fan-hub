#pragma once

#include "driver/gpio.h"

// The hub's FAN_OUT data line and the ATtiny's sense line beside it.

enum fan_step { FAN_OFF, FAN_LOW, FAN_MID, FAN_HIGH, FAN_STEPS };

#define FAN_BUS_NODES 50

// Remembers the data and sense pins.
void fan_bus_init(gpio_num_t data, gpio_num_t sense);
// Step name, "unknown" for -1.
const char *fan_bus_step_name(int step);

// The XIAO drives the data line, or floats it for the ATtiny.
void fan_bus_claim();
// Floats the data line for the ATtiny.
void fan_bus_release();
// True while the XIAO drives the line.
bool fan_bus_claimed();

// While claimed: sends the step when it changes, and again every second while resend is on.
void fan_bus_refresh(fan_step step, int ms_since_last_call);

// Test hooks: frames carry 1..FAN_BUS_NODES nodes; resend off sends on changes only.
void fan_bus_set_nodes(int nodes);
void fan_bus_set_resend(bool every_second);

// The ATtiny holds sense low while it drives the bus.
bool fan_bus_attiny_drives();
