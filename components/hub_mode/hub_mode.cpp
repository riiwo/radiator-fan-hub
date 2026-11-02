#include "hub_mode.h"

#include <stdio.h>
#include <string.h>
#include "nvs.h"

#include "fan_curve.h"

static const char *NAMES[HUB_MODES] = {"auto", "low", "mid", "high"};
static const hub_led_colour COLOURS[HUB_MODES] = {HUB_LED_BLUE, HUB_LED_GREEN, HUB_LED_YELLOW, HUB_LED_RED};

static nvs_handle_t nvs;
static int mode = HUB_AUTO;
static bool owned = true;

const char *hub_mode_name(int m) {
  return m == HUB_MODE_UNKNOWN ? "unknown" : NAMES[m];
}

int hub_mode_from_name(const char *name) {
  for (int m = 0; m < HUB_MODES; m++)
    if (strcmp(name, NAMES[m]) == 0) return m;
  return HUB_MODE_UNKNOWN;
}

hub_led_colour hub_mode_colour(int m) {
  return COLOURS[m];
}

void hub_mode_load() {
  ESP_ERROR_CHECK(nvs_open("controller", NVS_READWRITE, &nvs));
  uint8_t saved = HUB_AUTO;
  esp_err_t err = nvs_get_u8(nvs, "mode", &saved);
  if (err != ESP_ERR_NVS_NOT_FOUND) ESP_ERROR_CHECK(err);
  mode = saved;
}

int hub_mode_current() {
  return mode;
}

void hub_mode_set(int m) {
  mode = m;
  ESP_ERROR_CHECK(nvs_set_u8(nvs, "mode", mode));
  ESP_ERROR_CHECK(nvs_commit(nvs));
  printf("mode %s\n", NAMES[mode]);
}

void hub_mode_next() {
  hub_mode_set((mode + 1) % HUB_MODES);
}

fan_step hub_mode_fan_step() {
  return mode == HUB_AUTO ? fan_curve_step() : (fan_step) mode;
}

void hub_mode_hand_to_attiny() {
  mode = HUB_AUTO;
  owned = false;
}

bool hub_mode_owned() {
  return owned;
}
