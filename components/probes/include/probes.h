#pragma once

#include "driver/gpio.h"

// The flow and return DS18B20s, converted together once a second.

struct probes_reading {
  float flow, ret;
  bool flow_ok, ret_ok;  // false: the probe did not answer, its value is the last good one
  bool flow_fixed;       // flow comes from probes_fix_flow, flow_ok is then true
};

// Sets up both probe pins.
void probes_init(gpio_num_t flow, gpio_num_t ret);

// Call every ms_per_tick ms; true once a second, when the pair is new.
bool probes_tick(int ms_per_tick);
// The last reading pair.
const probes_reading *probes_latest();

// Test hooks: a fixed flow in place of the flow probe, and back.
void probes_fix_flow(float celsius);
void probes_flow_from_probe();
