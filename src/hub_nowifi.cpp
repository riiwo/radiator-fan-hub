// Program hub_nowifi: the controller without Home Assistant, so the WiFi radio never starts.
// For telling a fault caused by the radio from one caused by the rest of the controller.

#include "hub_board.h"
#include "hub_setups.h"

extern "C" void app_main(void) {
  hub_board_start();                          // pins, NVS, LED, button, probes

  if (!hub_board_find_attiny()) run_xiao_alone();  // returns if an ATtiny appears
  run_xiao_with_attiny();                     // ATtiny owns the button
}
