#include "hub_led.h"

#include <atomic>
#include "esp_timer.h"

// Yellow is red with green on MID_GREEN_ON of every MID_GREEN_PERIOD ms, so the mix reads yellow.
#define MID_GREEN_ON 3
#define MID_GREEN_PERIOD 8
#define BLINK_MS 300

static gpio_num_t pin_red, pin_green, pin_blue;
static volatile hub_led_colour shown = HUB_LED_OFF;
static std::atomic<int> blink_ms{0};
static int pwm = 0;

// Sinks the cathode to light it, or floats it.
static void sink(gpio_num_t pin, bool on) {
  if (on) {
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
  } else gpio_set_direction(pin, GPIO_MODE_INPUT);
}

// The colour to drive now: dark during a blink.
static hub_led_colour lit() {
  return blink_ms > 0 ? HUB_LED_OFF : shown;
}

// Drives the red and blue cathodes.
static void drive_red_and_blue() {
  hub_led_colour c = lit();
  sink(pin_blue, c == HUB_LED_BLUE);
  sink(pin_red, c == HUB_LED_YELLOW || c == HUB_LED_RED);
}

// Drives green, mixed in yellow.
static void drive_green() {
  hub_led_colour c = lit();
  sink(pin_green, c == HUB_LED_GREEN || (c == HUB_LED_YELLOW && pwm < MID_GREEN_ON));
}

// 1 ms timer: green mix and blink end.
static void on_millisecond(void *) {
  if (++pwm >= MID_GREEN_PERIOD) pwm = 0;
  if (blink_ms > 0 && blink_ms.fetch_sub(1) == 1) drive_red_and_blue();
  drive_green();
}

void hub_led_init(gpio_num_t red, gpio_num_t green, gpio_num_t blue) {
  pin_red = red;
  pin_green = green;
  pin_blue = blue;
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = (1ULL << red) | (1ULL << green) | (1ULL << blue);
  cfg.mode = GPIO_MODE_INPUT;
  ESP_ERROR_CHECK(gpio_config(&cfg));

  esp_timer_create_args_t args = {};
  args.callback = on_millisecond;
  args.name = "hub_led";
  esp_timer_handle_t timer;
  ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000));
}

void hub_led_show(hub_led_colour colour) {
  if (colour == shown) return;
  shown = colour;
  drive_red_and_blue();
}

void hub_led_blink() {
  blink_ms = BLINK_MS;
  drive_red_and_blue();
}
