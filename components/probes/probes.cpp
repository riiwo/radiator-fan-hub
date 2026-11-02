#include "probes.h"

#include "ds18b20.h"

#define PERIOD_MS 1000
#define READ_AT_MS 800

static gpio_num_t pin_flow, pin_ret;
static int ms_in_period = 0;
static probes_reading latest = {};
static float fixed_c = 0;

void probes_init(gpio_num_t flow, gpio_num_t ret) {
  pin_flow = flow;
  pin_ret = ret;
  ds18b20_init(flow);
  ds18b20_init(ret);
}

// Reads the flow probe, or takes the fixed flow.
static void read_flow() {
  float c;
  latest.flow_ok = latest.flow_fixed || ds18b20_read(pin_flow, &c) == DS18B20_OK;
  if (latest.flow_fixed) latest.flow = fixed_c;
  else if (latest.flow_ok) latest.flow = c;
}

// Reads the return probe.
static void read_return() {
  float c;
  latest.ret_ok = ds18b20_read(pin_ret, &c) == DS18B20_OK;
  if (latest.ret_ok) latest.ret = c;
}

bool probes_tick(int ms) {
  int was = ms_in_period;
  ms_in_period = (ms_in_period + ms) % PERIOD_MS;
  bool period_started = ms_in_period < was;
  bool conversion_done = was < READ_AT_MS && ms_in_period >= READ_AT_MS;
  if (period_started) {
    ds18b20_start(pin_flow);
    ds18b20_start(pin_ret);
  } else if (conversion_done) {
    read_flow();
    read_return();
  }
  return conversion_done;
}

const probes_reading *probes_latest() {
  return &latest;
}

void probes_fix_flow(float celsius) {
  latest.flow_fixed = true;
  fixed_c = celsius;
}

void probes_flow_from_probe() {
  latest.flow_fixed = false;
}
