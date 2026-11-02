#include "attiny_link.h"
#include "fan_bus.h"
#include "fan_curve.h"
#include "hub_board.h"
#include "hub_button.h"
#include "hub_ha.h"
#include "hub_led.h"
#include "hub_mode.h"
#include "hub_setups.h"
#include "probes.h"

void run_xiao_alone() {
  hub_mode_load();                  // mode from NVS
  fan_bus_claim();                  // XIAO drives the bus

  while (true) {
    hub_board_wait_tick();          // wait 10 ms, run hooks

    if (attiny_link_powered_up(HUB_TICK_MS)) {  // sense went low, then high
      fan_bus_release();            // free the lines for ISP
      if (hub_board_find_attiny()) return;      // ATtiny answered: switch setup
      fan_bus_claim();              // no ATtiny: drive again
    }

    switch (hub_button_tick(HUB_TICK_MS)) {     // debounce, report press events
      case HUB_BUTTON_SHORT: hub_mode_next(); break;  // auto, low, mid, high
      case HUB_BUTTON_LONG: hub_led_blink(); break;   // LED dark for 300 ms
      case HUB_BUTTON_LONG_RELEASE: break;
      case HUB_BUTTON_NONE: break;
    }

    if (hub_ha_take_press()) hub_mode_next();   // HA Press steps the mode
    if (hub_ha_requested_mode() != HUB_MODE_UNKNOWN) {  // HA picked a mode
      hub_mode_set(hub_ha_requested_mode());    // set and save it
      hub_ha_mode_done();           // request handled
    }

    if (probes_tick(HUB_TICK_MS)) {  // new probe pair, once a second
      fan_curve_follow(probes_latest());        // flow moves the curve step
      hub_board_print_status(hub_mode_current(), hub_mode_fan_step(), probes_latest());  // one status line
    }

    fan_bus_refresh(hub_mode_fan_step(), HUB_TICK_MS);  // send on change, each second
    hub_led_show(hub_mode_colour(hub_mode_current()));  // LED shows the mode
    hub_ha_publish_when_due(hub_mode_current(), hub_mode_fan_step(), probes_latest(), HUB_TICK_MS);  // state to HA
  }
}
