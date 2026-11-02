#pragma once

#include "driver/gpio.h"

struct attiny_isp_pins {
  gpio_num_t mosi, sck, miso, reset;
};

// Holds the ATtiny in reset, reads its signature and releases it, which restarts it. Afterwards
// MOSI, MISO and RESET float and SCK floats with the pull-up.
bool attiny_isp_finds_attiny13a(const attiny_isp_pins *pins);
