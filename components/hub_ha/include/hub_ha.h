#pragma once

#include "probes.h"

// The hub in Home Assistant. Requests arrive in the MQTT task and wait here until the
// running setup takes them.

struct hub_network {
  const char *ssid, *password, *broker_uri;
};

// Starts WiFi and MQTT as Radiator hub <hub_id>.
void hub_ha_start(const hub_network *network, const char *hub_id);

bool hub_ha_take_press();        // true once per HA Press
int hub_ha_requested_mode();     // HUB_MODE_UNKNOWN when none is waiting
// The requested mode is reached.
void hub_ha_mode_done();

// Publishes when mode or step changes, and every PUBLISH_EVERY_MS. HUB_MODE_UNKNOWN and a
// step of -1 publish as unknown.
void hub_ha_publish_when_due(int mode, int step, const probes_reading *probes, int ms_per_tick);
