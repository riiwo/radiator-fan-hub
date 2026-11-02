// Program hub_noisp: the controller without Home Assistant and without the ATtiny check at
// boot; it assumes the ATtiny is fitted. For telling a fault caused by the ISP check from one
// caused by the rest of the controller.

#include "hub_board.h"
#include "hub_setups.h"

extern "C" void app_main(void) {
  hub_board_start();                          // pins, NVS, LED, button, probes
  run_xiao_with_attiny();                     // ATtiny owns the button
}
