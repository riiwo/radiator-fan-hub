#include "fan_bus.h"

#include <stdio.h>
#include <string.h>
#include "esp_rom_sys.h"
#include "ws2811_tx.h"

#define SENSE_READS 5
#define RESEND_MS 1000

// Byte 0 is full speed on the node's channel; 255 leaves it off.
static const uint8_t TRIPLES[FAN_STEPS][3] = {
  {255, 255, 255},
  {255, 255, 0},
  {255, 0, 255},
  {0, 255, 255},
};
static const char *STEP_NAMES[FAN_STEPS] = {"off", "low", "mid", "high"};

static gpio_num_t pin_data, pin_sense;
static uint8_t frame[FAN_BUS_NODES * 3];
static int nodes = FAN_BUS_NODES;
static bool claimed = false, resend = true;
static int sent = -1, since_ms = 0;

void fan_bus_init(gpio_num_t data, gpio_num_t sense) {
  pin_data = data;
  pin_sense = sense;
}

const char *fan_bus_step_name(int step) {
  return step < 0 ? "unknown" : STEP_NAMES[step];
}

void fan_bus_claim() {
  ws2811_tx_init(pin_data);
  claimed = true;
  sent = -1;
  printf("bus XIAO\n");
}

void fan_bus_release() {
  ws2811_tx_deinit();
  ESP_ERROR_CHECK(gpio_reset_pin(pin_data));
  claimed = false;
  printf("bus ATtiny\n");
}

bool fan_bus_claimed() {
  return claimed;
}

// Fills every node with the step and sends the frame.
static void send(fan_step step) {
  for (int n = 0; n < nodes; n++) memcpy(&frame[n * 3], TRIPLES[step], 3);
  ws2811_tx_send(frame, nodes * 3, WS2811_LATCH_US * 5);
  sent = step;
  since_ms = 0;
}

// True when the one-second resend is due.
static bool resend_due(int ms) {
  return resend && (since_ms += ms) >= RESEND_MS;
}

void fan_bus_refresh(fan_step step, int ms) {
  if (!claimed) return;
  if (step != sent || resend_due(ms)) send(step);
}

void fan_bus_set_nodes(int n) {
  nodes = n;
}

void fan_bus_set_resend(bool every_second) {
  resend = every_second;
}

// The XIAO's own frames couple short low glitches onto sense; the ATtiny's low is steady.
bool fan_bus_attiny_drives() {
  for (int i = 0; i < SENSE_READS; i++) {
    if (gpio_get_level(pin_sense)) return false;
    esp_rom_delay_us(200);
  }
  return true;
}
