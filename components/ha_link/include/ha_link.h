#pragma once

// The hub in Home Assistant: WiFi, MQTT discovery as device "Radiator hub <hub_id>", and
// HTTP OTA. Topics live under radiator_hub_<hub_id>/.

struct ha_link_config {
  const char *ssid, *password, *broker_uri;
  const char *hub_id;
  void (*on_mode)(const char *mode);  // HA picked auto, low, mid or high; runs in the MQTT task
  void (*on_press)();                 // HA's Press button; runs in the MQTT task
};

// Connects in the background and keeps reconnecting.
void ha_link_start(const ha_link_config *config);

// Retained state for "flow", "return", "mode" or "step"; dropped while MQTT is down.
void ha_link_publish(const char *object, const char *value);
