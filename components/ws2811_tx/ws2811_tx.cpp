#include "ws2811_tx.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"

// 40 MHz gives 25 ns per tick, so every datasheet time is a whole number of ticks.
#define RESOLUTION_HZ 40000000
#define NS_PER_TICK (1000000000 / RESOLUTION_HZ)

static rmt_channel_handle_t channel = NULL;
static rmt_encoder_handle_t encoder = NULL;

void ws2811_tx_init(gpio_num_t pin) {
  rmt_tx_channel_config_t tx_config = {};
  tx_config.gpio_num = pin;
  tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_config.resolution_hz = RESOLUTION_HZ;
  tx_config.mem_block_symbols = 64;
  tx_config.trans_queue_depth = 1;
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &channel));

  rmt_bytes_encoder_config_t enc_config = {};
  enc_config.bit0.level0 = 1;
  enc_config.bit0.duration0 = WS2811_T0H_NS / NS_PER_TICK;
  enc_config.bit0.level1 = 0;
  enc_config.bit0.duration1 = WS2811_T0L_NS / NS_PER_TICK;
  enc_config.bit1.level0 = 1;
  enc_config.bit1.duration0 = WS2811_T1H_NS / NS_PER_TICK;
  enc_config.bit1.level1 = 0;
  enc_config.bit1.duration1 = WS2811_T1L_NS / NS_PER_TICK;
  enc_config.flags.msb_first = 1;
  ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_config, &encoder));

  ESP_ERROR_CHECK(rmt_enable(channel));
}

void ws2811_tx_deinit(void) {
  ESP_ERROR_CHECK(rmt_disable(channel));
  ESP_ERROR_CHECK(rmt_del_channel(channel));
  ESP_ERROR_CHECK(rmt_del_encoder(encoder));
  channel = NULL;
  encoder = NULL;
}

void ws2811_tx_send(const uint8_t *bytes, size_t len, uint32_t gap_us) {
  rmt_transmit_config_t tx = {};
  ESP_ERROR_CHECK(rmt_transmit(channel, encoder, bytes, len, &tx));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(channel, portMAX_DELAY));
  esp_rom_delay_us(gap_us);
}
