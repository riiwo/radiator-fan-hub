#pragma once

#include "driver/gpio.h"

// XIAO ESP32-C3 pins on the controller PCB (FAN HUB). The ATtiny's ISP shares them:
// MOSI is bus data, SCK is sense, MISO is the button.

#define HUB_PIN_DATA GPIO_NUM_3     // D1, FAN_OUT DOUT
#define HUB_PIN_RED GPIO_NUM_4      // D2, LED red cathode
#define HUB_PIN_BLUE GPIO_NUM_5     // D3, LED blue cathode
#define HUB_PIN_RESET GPIO_NUM_6    // D4, ATtiny RESET
#define HUB_PIN_SENSE GPIO_NUM_7    // D5, ATtiny mode sense
#define HUB_PIN_RETURN GPIO_NUM_8   // D8, return probe
#define HUB_PIN_FLOW GPIO_NUM_10    // D10, flow probe
#define HUB_PIN_BUTTON GPIO_NUM_20  // D7, BTN
#define HUB_PIN_GREEN GPIO_NUM_21   // D6, LED green cathode
