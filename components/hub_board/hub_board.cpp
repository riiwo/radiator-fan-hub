#include "hub_board.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "nvs_flash.h"

#include "attiny_isp.h"
#include "fan_bus.h"
#include "hub_button.h"
#include "hub_led.h"
#include "hub_mode.h"
#include "hub_pins.h"

static char id[7];
static void (*tick_hook)() = NULL;

// A partition table change leaves NVS in the old layout; it holds only the mode, so it is wiped.
static void start_nvs() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

// Builds the hub id from the MAC.
static void read_id() {
  uint8_t mac[6];
  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
  snprintf(id, sizeof(id), "%02X%02X%02X", mac[3], mac[4], mac[5]);
}

void hub_board_start() {
  start_nvs();
  read_id();
  hub_led_init(HUB_PIN_RED, HUB_PIN_GREEN, HUB_PIN_BLUE);
  hub_button_init(HUB_PIN_BUTTON);
  fan_bus_init(HUB_PIN_DATA, HUB_PIN_SENSE);
  probes_init(HUB_PIN_FLOW, HUB_PIN_RETURN);
  printf("\nhub %s\n", id);
}

const char *hub_board_id() {
  return id;
}

bool hub_board_find_attiny() {
  attiny_isp_pins pins = {HUB_PIN_DATA, HUB_PIN_SENSE, HUB_PIN_BUTTON, HUB_PIN_RESET};
  bool found = attiny_isp_finds_attiny13a(&pins);
  hub_button_init(HUB_PIN_BUTTON);
  printf(found ? "ATtiny fitted, it owns the button\n" : "no ATtiny\n");
  return found;
}

void hub_board_on_tick(void (*hook)()) {
  tick_hook = hook;
}

void hub_board_wait_tick() {
  vTaskDelay(pdMS_TO_TICKS(HUB_TICK_MS));
  if (tick_hook) tick_hook();
}

void hub_board_print_status(int mode, int step, const probes_reading *p) {
  printf("flow %6.2f C%s  return ", p->flow, p->flow_fixed ? " (fixed)" : p->flow_ok ? "" : " (probe lost)");
  if (p->ret_ok) printf("%6.2f C", p->ret);
  else printf("lost");
  printf("  mode %-4s  step %-4s  bus %s\n", hub_mode_name(mode), fan_bus_step_name(step),
         fan_bus_claimed() ? "XIAO" : "ATtiny");
}
