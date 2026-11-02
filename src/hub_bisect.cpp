// Program hub_bisect: hub-noisp taken apart to find which part breaks hub 3's data. Build flags
// pick the parts: BISECT_LED starts the LED (1 ms timer), BISECT_BUTTON sets up the button
// line. Sense is read and the probes run in every build; the XIAO never drives the bus.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "attiny_link.h"
#include "fan_bus.h"
#include "hub_button.h"
#include "hub_led.h"
#include "hub_pins.h"
#include "probes.h"

extern "C" void app_main(void) {
  printf("\nhub_bisect: LED %d, button %d\n", BISECT_LED, BISECT_BUTTON);
  if (BISECT_LED) hub_led_init(HUB_PIN_RED, HUB_PIN_GREEN, HUB_PIN_BLUE);
  if (BISECT_BUTTON) hub_button_init(HUB_PIN_BUTTON);
  fan_bus_init(HUB_PIN_DATA, HUB_PIN_SENSE);
  probes_init(HUB_PIN_FLOW, HUB_PIN_RETURN);

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10));
    bool drives = fan_bus_attiny_drives();
    if (BISECT_BUTTON) hub_button_tick(10);
    if (probes_tick(10)) printf("flow %.2f  return %.2f  attiny drives %d\n", probes_latest()->flow, probes_latest()->ret, drives);
  }
}
