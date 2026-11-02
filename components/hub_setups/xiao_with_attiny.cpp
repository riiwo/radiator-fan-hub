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

void run_xiao_with_attiny() {
  hub_mode_hand_to_attiny();        // XIAO mode stays auto

  while (true) {
    hub_board_wait_tick();          // wait 10 ms, run hooks

    switch (attiny_link_watch(HUB_TICK_MS)) {   // follow the sense line
      case ATTINY_LINK_TOOK_BUS: fan_bus_release(); break;  // ATtiny drives: let go
      case ATTINY_LINK_LEFT_BUS: fan_bus_claim(); break;    // ATtiny off: XIAO drives
      case ATTINY_LINK_NONE: break;
    }

    switch (hub_button_tick(HUB_TICK_MS)) {     // debounce, report press events
      case HUB_BUTTON_SHORT: attiny_link_count_press(); break;  // ATtiny stepped its mode
      case HUB_BUTTON_LONG: if (fan_bus_claimed()) hub_led_blink(); break;  // blink while XIAO lights
      case HUB_BUTTON_LONG_RELEASE: break;
      case HUB_BUTTON_NONE: break;
    }

    if (hub_ha_take_press()) hub_button_press(1);  // HA Press presses once
    if (!hub_button_pressing()) hub_button_press(attiny_link_presses_to(hub_ha_requested_mode()));  // press toward HA's mode
    if (attiny_link_mode() == hub_ha_requested_mode()) hub_ha_mode_done();  // HA's mode reached

    if (probes_tick(HUB_TICK_MS)) {  // new probe pair, once a second
      fan_curve_follow(probes_latest());        // flow moves the curve step
      hub_board_print_status(attiny_link_mode(), attiny_link_bus_step(), probes_latest());  // one status line
    }

    fan_bus_refresh(fan_curve_step(), HUB_TICK_MS);  // sends only while XIAO drives
    hub_led_show(attiny_link_xiao_colour());    // blue in auto, else dark
    hub_ha_publish_when_due(attiny_link_mode(), attiny_link_bus_step(), probes_latest(), HUB_TICK_MS);  // state to HA
  }
}
