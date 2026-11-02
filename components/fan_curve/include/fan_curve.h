#pragma once

#include "fan_bus.h"
#include "probes.h"

// Off, low, mid, high at flow >= 28, 32, 36 C rising; each step drops HYST_C below its
// threshold. A flow probe that does not answer keeps the step.
void fan_curve_follow(const probes_reading *probes);
// The curve's current step.
fan_step fan_curve_step();
