// Program hub_test: the controller with the test console the hub test jig drives.

#include "hub_board.h"
#include "hub_console.h"
#include "hub_ha.h"
#include "hub_setups.h"
#include "secrets.h"

static const hub_network NETWORK = {WIFI_SSID, WIFI_PASS, MQTT_BROKER_URI};

extern "C" void app_main(void) {
  hub_console_start();                        // USB console can read
  hub_board_on_tick(hub_console_poll);        // run console lines each tick
  hub_board_start();                          // pins, NVS, LED, button, probes
  hub_ha_start(&NETWORK, hub_board_id());     // WiFi, MQTT and HA discovery

  if (!hub_board_find_attiny()) run_xiao_alone();  // returns if an ATtiny appears
  run_xiao_with_attiny();                     // ATtiny owns the button
}
