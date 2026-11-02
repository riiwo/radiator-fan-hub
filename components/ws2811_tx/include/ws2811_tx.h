#pragma once

#include <stddef.h>
#include <stdint.h>
#include "driver/gpio.h"

// WS2811 low-speed mode, SET tied to VDD, datasheet page 3: 2.5 us per bit.
#define WS2811_T0H_NS 500
#define WS2811_T0L_NS 2000
#define WS2811_T1H_NS 1200
#define WS2811_T1L_NS 1300
#define WS2811_TOLERANCE_NS 150
#define WS2811_LATCH_US 60

void ws2811_tx_init(gpio_num_t pin);

// Releases the RMT channel and its pin, so the pin can be used as a plain GPIO until the next init.
void ws2811_tx_deinit(void);

// Sends the bytes MSB first, blocks until the last bit is out, then holds the
// line low for gap_us. WS2811_LATCH_US or more makes the chips latch the frame.
void ws2811_tx_send(const uint8_t *bytes, size_t len, uint32_t gap_us);
