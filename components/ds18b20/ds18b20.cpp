#include "ds18b20.h"

#include "freertos/FreeRTOS.h"
#include "esp_rom_sys.h"

static portMUX_TYPE slot_lock = portMUX_INITIALIZER_UNLOCKED;

void ds18b20_init(gpio_num_t pin) {
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
  ESP_ERROR_CHECK(gpio_config(&cfg));
  gpio_set_level(pin, 1);
}

static bool reset(gpio_num_t pin) {
  gpio_set_level(pin, 0);
  esp_rom_delay_us(480);
  portENTER_CRITICAL(&slot_lock);
  gpio_set_level(pin, 1);
  esp_rom_delay_us(70);
  bool present = gpio_get_level(pin) == 0;
  portEXIT_CRITICAL(&slot_lock);
  esp_rom_delay_us(410);
  return present;
}

static void write_bit(gpio_num_t pin, int bit) {
  portENTER_CRITICAL(&slot_lock);
  gpio_set_level(pin, 0);
  esp_rom_delay_us(bit ? 6 : 60);
  gpio_set_level(pin, 1);
  portEXIT_CRITICAL(&slot_lock);
  esp_rom_delay_us(bit ? 64 : 10);
}

static int read_bit(gpio_num_t pin) {
  portENTER_CRITICAL(&slot_lock);
  gpio_set_level(pin, 0);
  esp_rom_delay_us(6);
  gpio_set_level(pin, 1);
  esp_rom_delay_us(9);
  int bit = gpio_get_level(pin);
  portEXIT_CRITICAL(&slot_lock);
  esp_rom_delay_us(55);
  return bit;
}

static void write_byte(gpio_num_t pin, uint8_t b) {
  for (int i = 0; i < 8; i++) write_bit(pin, (b >> i) & 1);
}

static uint8_t read_byte(gpio_num_t pin) {
  uint8_t b = 0;
  for (int i = 0; i < 8; i++) b |= read_bit(pin) << i;
  return b;
}

static uint8_t crc8(const uint8_t *data, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    uint8_t b = data[i];
    for (int j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ b) & 1;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      b >>= 1;
    }
  }
  return crc;
}

bool ds18b20_start(gpio_num_t pin) {
  if (!reset(pin)) return false;
  write_byte(pin, 0xCC);
  write_byte(pin, 0x44);
  return true;
}

ds18b20_result ds18b20_read(gpio_num_t pin, float *celsius) {
  if (!reset(pin)) return DS18B20_ABSENT;
  write_byte(pin, 0xCC);
  write_byte(pin, 0xBE);
  uint8_t pad[9];
  for (int i = 0; i < 9; i++) pad[i] = read_byte(pin);
  if (crc8(pad, 8) != pad[8]) return DS18B20_CRC;
  *celsius = (int16_t) (pad[1] << 8 | pad[0]) / 16.0f;
  return DS18B20_OK;
}
