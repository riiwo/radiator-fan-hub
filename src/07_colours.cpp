// Program 07: the XIAO lights red (D2), green (D6) and blue (D3).
//
// Each colour pin only sinks its LED cathode through its own 1 kOhm, or floats;
// the ATtiny sinks red and green through R3 and R4 the same way. While the mode
// sense (D5) has been high for TAKE_MS, the ATtiny is off and the XIAO cycles
// red, green, blue, magenta, yellow, cyan and white, one second each. Sense low
// floats every XIAO colour pin so only the ATtiny's mode colour shows. The data
// pin (D1) stays an input throughout. D6 is the ROM boot log's TX, so every
// reset flickers green before this program claims the pin.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

#define PIN_RED GPIO_NUM_4
#define PIN_GREEN GPIO_NUM_21
#define PIN_BLUE GPIO_NUM_5
#define PIN_SENSE GPIO_NUM_7

#define TICK_MS 10
#define TAKE_MS 100
#define HOLD_MS 1000

enum { R = 1, G = 2, B = 4 };

struct Colour { const char *name; int mask; };
static const Colour CYCLE[] = {
  {"red", R}, {"green", G}, {"blue", B}, {"magenta", R | B},
  {"yellow", R | G}, {"cyan", G | B}, {"white", R | G | B},
};
#define NCOLOURS (sizeof(CYCLE) / sizeof(CYCLE[0]))

static void sink(gpio_num_t pin, bool on) {
  if (on) {
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
  } else gpio_set_direction(pin, GPIO_MODE_INPUT);
}

static void show(int mask) {
  sink(PIN_RED, mask & R);
  sink(PIN_GREEN, mask & G);
  sink(PIN_BLUE, mask & B);
}

extern "C" void app_main(void) {
  usb_serial_jtag_driver_config_t console = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&console));
  usb_serial_jtag_vfs_use_driver();

  gpio_config_t cfg = {};
  cfg.pin_bit_mask = (1ULL << PIN_RED) | (1ULL << PIN_GREEN) | (1ULL << PIN_BLUE) |
                     (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_6) | (1ULL << GPIO_NUM_20);
  cfg.mode = GPIO_MODE_INPUT;
  ESP_ERROR_CHECK(gpio_config(&cfg));
  gpio_config_t sense = {};
  sense.pin_bit_mask = 1ULL << PIN_SENSE;
  sense.mode = GPIO_MODE_INPUT;
  sense.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&sense));
  printf("\n07_colours: red D2, green D6, blue D3, cycling while the ATtiny is off\n");

  bool showing = false;
  int sense_high_ms = 0, held_ms = 0;
  size_t i = 0;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    if (gpio_get_level(PIN_SENSE)) sense_high_ms += TICK_MS;
    else sense_high_ms = 0;

    if (showing && sense_high_ms == 0) {
      showing = false;
      show(0);
      printf("ATtiny has the bus, XIAO colours off\n");
    } else if (!showing && sense_high_ms >= TAKE_MS) {
      showing = true;
      held_ms = HOLD_MS;
    }
    if (showing && (held_ms += TICK_MS) >= HOLD_MS) {
      held_ms = 0;
      show(CYCLE[i].mask);
      printf("%s\n", CYCLE[i].name);
      i = (i + 1) % NCOLOURS;
    }
  }
}
