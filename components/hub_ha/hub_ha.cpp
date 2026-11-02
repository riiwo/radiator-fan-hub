#include "hub_ha.h"

#include <atomic>
#include <stdio.h>

#include "fan_bus.h"
#include "ha_link.h"
#include "hub_mode.h"

#define PUBLISH_EVERY_MS 10000

static std::atomic<int> requested_mode{HUB_MODE_UNKNOWN};
static std::atomic<int> requested_presses{0};

// MQTT task: HA picked a mode.
static void on_mode(const char *name) {
  requested_mode = hub_mode_from_name(name);
}

// MQTT task: HA pressed Press.
static void on_press() {
  requested_presses++;
}

void hub_ha_start(const hub_network *network, const char *hub_id) {
  ha_link_config cfg = {};
  cfg.ssid = network->ssid;
  cfg.password = network->password;
  cfg.broker_uri = network->broker_uri;
  cfg.hub_id = hub_id;
  cfg.on_mode = on_mode;
  cfg.on_press = on_press;
  ha_link_start(&cfg);
}

bool hub_ha_take_press() {
  int waiting = requested_presses;
  return waiting > 0 && requested_presses.compare_exchange_strong(waiting, waiting - 1);
}

int hub_ha_requested_mode() {
  return requested_mode;
}

void hub_ha_mode_done() {
  requested_mode = HUB_MODE_UNKNOWN;
}

// Publishes a value with two decimals.
static void publish_number(const char *object, float value) {
  char text[12];
  snprintf(text, sizeof(text), "%.2f", value);
  ha_link_publish(object, text);
}

// Publishes flow, return, mode and step.
static void publish(int mode, int step, const probes_reading *probes) {
  if (probes->flow_ok) publish_number("flow", probes->flow);
  if (probes->ret_ok) publish_number("return", probes->ret);
  if (mode != HUB_MODE_UNKNOWN) ha_link_publish("mode", hub_mode_name(mode));
  ha_link_publish("step", fan_bus_step_name(step));
}

void hub_ha_publish_when_due(int mode, int step, const probes_reading *probes, int ms) {
  static int last_mode = HUB_MODE_UNKNOWN - 1, last_step = -2, since_ms = 0;
  bool changed = mode != last_mode || step != last_step;
  if (!changed && (since_ms += ms) < PUBLISH_EVERY_MS) return;
  last_mode = mode;
  last_step = step;
  since_ms = 0;
  publish(mode, step, probes);
}
