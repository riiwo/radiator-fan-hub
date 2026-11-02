// Program 06: read one DS18B20 on each probe pin, flow on D10 and return on D8.
//
// Each second both probes start a conversion together, then each scratchpad is
// read and CRC-checked and one line is printed: "flow 21.44 C  return 21.50 C".
// A pin with no presence pulse prints "absent", a bad CRC prints "crc".

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

#include "ds18b20.h"

#define PIN_FLOW GPIO_NUM_10
#define PIN_RETURN GPIO_NUM_8

static void print_reading(const char *name, gpio_num_t pin, bool started) {
  float c;
  ds18b20_result r = started ? ds18b20_read(pin, &c) : DS18B20_ABSENT;
  if (r == DS18B20_OK) printf("%s %6.2f C", name, c);
  else printf("%s %s", name, r == DS18B20_ABSENT ? "absent" : "crc");
}

extern "C" void app_main(void) {
  usb_serial_jtag_driver_config_t console = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&console));
  usb_serial_jtag_vfs_use_driver();
  ds18b20_init(PIN_FLOW);
  ds18b20_init(PIN_RETURN);
  printf("\n06_probes: flow on D10 (GPIO10), return on D8 (GPIO8)\n");

  while (true) {
    bool flow = ds18b20_start(PIN_FLOW);
    bool ret = ds18b20_start(PIN_RETURN);
    vTaskDelay(pdMS_TO_TICKS(DS18B20_CONVERT_MS));
    print_reading("flow", PIN_FLOW, flow);
    printf("  ");
    print_reading("return", PIN_RETURN, ret);
    printf("\n");
    vTaskDelay(pdMS_TO_TICKS(1000 - DS18B20_CONVERT_MS));
  }
}
