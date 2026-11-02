#pragma once

#include "driver/gpio.h"

// One DS18B20 per pin, so every transaction is Skip ROM. 1-Wire is bit-banged
// open-drain against an external 4.7 kOhm pull-up.

#define DS18B20_CONVERT_MS 750

enum ds18b20_result { DS18B20_OK, DS18B20_ABSENT, DS18B20_CRC };

void ds18b20_init(gpio_num_t pin);

// Starts a conversion; false when no presence pulse answers the reset.
bool ds18b20_start(gpio_num_t pin);

// Reads the scratchpad of a finished conversion, CRC-checked.
ds18b20_result ds18b20_read(gpio_num_t pin, float *celsius);
