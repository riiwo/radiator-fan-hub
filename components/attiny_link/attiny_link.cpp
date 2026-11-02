#include "attiny_link.h"

#include "fan_curve.h"
#include "hub_mode.h"

#define TAKE_MS 50
#define STRAY_MS 200

static int mode = HUB_MODE_UNKNOWN;
static int free_ms = 0, stray_ms = 0;
static bool was_low = false;

// A press counted while the ATtiny is off must make it take the bus; when it does not
// within STRAY_MS the press never reached it (the line dropped with the ATtiny unpowered).
static void drop_press_the_attiny_ignored(bool drives, int ms) {
  bool waiting_for_attiny = fan_bus_claimed() && !drives && mode != HUB_AUTO;
  stray_ms = waiting_for_attiny ? stray_ms + ms : 0;
  if (stray_ms >= STRAY_MS) mode = HUB_AUTO;
}

attiny_link_event attiny_link_watch(int ms) {
  bool drives = fan_bus_attiny_drives();
  drop_press_the_attiny_ignored(drives, ms);
  if (drives) free_ms = 0;
  if (drives && fan_bus_claimed()) return ATTINY_LINK_TOOK_BUS;
  if (drives || fan_bus_claimed() || (free_ms += ms) < TAKE_MS) return ATTINY_LINK_NONE;
  mode = HUB_AUTO;
  return ATTINY_LINK_LEFT_BUS;
}

void attiny_link_count_press() {
  if (mode != HUB_MODE_UNKNOWN) mode = (mode + 1) % HUB_MODES;
}

int attiny_link_mode() {
  return mode;
}

int attiny_link_presses_to(int target) {
  if (target == HUB_MODE_UNKNOWN || target == mode) return 0;
  return mode == HUB_MODE_UNKNOWN ? 1 : (target - mode + HUB_MODES) % HUB_MODES;
}

int attiny_link_bus_step() {
  return fan_bus_claimed() ? fan_curve_step() : mode;
}

hub_led_colour attiny_link_xiao_colour() {
  return fan_bus_claimed() ? hub_mode_colour(HUB_AUTO) : HUB_LED_OFF;
}

bool attiny_link_powered_up(int ms) {
  if (fan_bus_attiny_drives()) {
    was_low = true;
    free_ms = 0;
    return false;
  }
  if (!was_low || (free_ms += ms) < TAKE_MS) return false;
  was_low = false;
  mode = HUB_AUTO;
  return true;
}
