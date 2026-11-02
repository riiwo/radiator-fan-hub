#include "fan_curve.h"

#define HYST_C 1.0f

static const float UP_C[FAN_STEPS] = {0, 28, 32, 36};
static int step = FAN_OFF;

void fan_curve_follow(const probes_reading *probes) {
  if (!probes->flow_ok) return;
  while (step < FAN_HIGH && probes->flow >= UP_C[step + 1]) step++;
  while (step > FAN_OFF && probes->flow < UP_C[step] - HYST_C) step--;
}

fan_step fan_curve_step() {
  return (fan_step) step;
}
