#pragma once

#include "probes.h"

// The controller PCB: brings up its parts on the pins in hub_pins.h and paces the setups.

#define HUB_TICK_MS 10

// Brings up NVS, LED, button, bus and probes.
void hub_board_start();
const char *hub_board_id();  // the last three bytes of the WiFi MAC, e.g. F0BEBC

// Holds the ATtiny in reset over ISP, reads its signature and lets it run again. The bus
// must be released.
bool hub_board_find_attiny();

// Waits for the next tick and runs the hook given to hub_board_on_tick, if any.
void hub_board_wait_tick();
void hub_board_on_tick(void (*hook)());

// Prints one status line.
void hub_board_print_status(int mode, int step, const probes_reading *probes);
