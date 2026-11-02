// Program 01: prove the bit timing on the wire.
//
// Jumper D1 (GPIO3) -> D2 (GPIO4). The TX component streams frames of 0x0F
// back to back; an RMT receiver on GPIO4 measures every pulse and checks it
// against the datasheet windows. Once a second it prints the tally and the DC
// average a DMM on GPIO3 should show. Nothing else is exercised: no WS2811, no
// fan.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "ws2811_tx.h"

#define TX_PIN GPIO_NUM_3
#define RX_PIN GPIO_NUM_4

#define PATTERN 0x0F
#define FRAME_BYTES 6
#define FRAME_BITS (FRAME_BYTES * 8)
#define GAP_US 20
#define REPORT_US 1000000
#define GPIO_HIGH_V 3.3f

// 80 MHz gives 12.5 ns per tick. A level held longer than RANGE_MAX_NS ends the
// frame, so it must sit above the longest valid low (1000 ns) and below the gap.
#define RX_RESOLUTION_HZ 80000000
#define RANGE_MIN_NS 100
#define RANGE_MAX_NS 5000

static rmt_channel_handle_t rx_channel = NULL;
static QueueHandle_t rx_queue = NULL;
static rmt_symbol_word_t rx_buffer[64];

struct Window {
  const char *name;
  uint32_t high_ns;
  uint32_t low_ns;
};

static const Window ZERO = {"0-code", WS2811_T0H_NS, WS2811_T0L_NS};
static const Window ONE = {"1-code", WS2811_T1H_NS, WS2811_T1L_NS};

struct Stats {
  int count = 0;
  uint32_t high_min = UINT32_MAX, high_max = 0;
  uint32_t low_min = UINT32_MAX, low_max = 0;
  int out_of_window = 0;
};

static bool on_rx_done(rmt_channel_handle_t, const rmt_rx_done_event_data_t *data, void *) {
  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(rx_queue, data, &woken);
  return woken == pdTRUE;
}

static uint32_t ticks_to_ns(uint32_t ticks) {
  return (uint32_t) ((uint64_t) ticks * 1000000000ULL / RX_RESOLUTION_HZ);
}

static bool within(uint32_t measured, uint32_t target) {
  uint32_t diff = measured > target ? measured - target : target - measured;
  return diff <= WS2811_TOLERANCE_NS;
}

static void record(Stats &s, uint32_t high, uint32_t low, const Window &w, bool check_low) {
  s.count++;
  if (high < s.high_min) s.high_min = high;
  if (high > s.high_max) s.high_max = high;
  if (!within(high, w.high_ns)) s.out_of_window++;
  if (!check_low) return;
  if (low < s.low_min) s.low_min = low;
  if (low > s.low_max) s.low_max = low;
  if (!within(low, w.low_ns)) s.out_of_window++;
}

static void merge(Stats &into, const Stats &from) {
  into.count += from.count;
  if (from.high_min < into.high_min) into.high_min = from.high_min;
  if (from.high_max > into.high_max) into.high_max = from.high_max;
  if (from.low_min < into.low_min) into.low_min = from.low_min;
  if (from.low_max > into.low_max) into.low_max = from.low_max;
  into.out_of_window += from.out_of_window;
}

static void print_stats(const Window &w, const Stats &s, int expected) {
  printf("  %s: %d/%d  high %4lu..%4lu ns (target %lu)  low %4lu..%4lu ns (target %lu)  outside window: %d\n",
         w.name, s.count, expected,
         (unsigned long) s.high_min, (unsigned long) s.high_max, (unsigned long) w.high_ns,
         (unsigned long) s.low_min, (unsigned long) s.low_max, (unsigned long) w.low_ns,
         s.out_of_window);
}

static void count_bits(int &zeros, int &ones) {
  zeros = 0;
  ones = 0;
  for (int b = 0; b < 8; b++) ((PATTERN >> b) & 1) ? ones++ : zeros++;
  zeros *= FRAME_BYTES;
  ones *= FRAME_BYTES;
}

static uint32_t high_ns_per_frame() {
  int zeros, ones;
  count_bits(zeros, ones);
  return zeros * WS2811_T0H_NS + ones * WS2811_T1H_NS;
}

static float dmm_volts(uint32_t frames, int64_t elapsed_us) {
  float high_us = (float) frames * (float) high_ns_per_frame() / 1000.0f;
  return GPIO_HIGH_V * high_us / (float) elapsed_us;
}

static void print_expectations() {
  int zeros, ones;
  count_bits(zeros, ones);
  uint32_t min_period_ns = FRAME_BITS * (WS2811_T0H_NS + WS2811_T0L_NS) + GAP_US * 1000;
  float ceiling = GPIO_HIGH_V * (float) high_ns_per_frame() / (float) min_period_ns;

  printf("\n01_timing: pattern 0x%02X x %d bytes, %d bits per frame, %d us gap, frames back to back\n",
         PATTERN, FRAME_BYTES, FRAME_BITS, GAP_US);
  printf("  wire: GPIO%d -> GPIO%d jumper\n", TX_PIN, RX_PIN);
  printf("  expect per frame: %d x 0-code (%d/%d ns), %d x 1-code (%d/%d ns), tolerance +/-%d ns\n",
         zeros, WS2811_T0H_NS, WS2811_T0L_NS, ones, WS2811_T1H_NS, WS2811_T1L_NS, WS2811_TOLERANCE_NS);
  printf("  DMM on GPIO%d, DC volts: %lu ns high per frame x measured frame rate, printed every second\n",
         TX_PIN, (unsigned long) high_ns_per_frame());
  printf("  ceiling %.2f V at the %lu us minimum period; loop overhead brings it down\n\n",
         ceiling, (unsigned long) (min_period_ns / 1000));
}

extern "C" void app_main(void) {
  print_expectations();

  ws2811_tx_init(TX_PIN);

  rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));

  rmt_rx_channel_config_t rx_config = {};
  rx_config.gpio_num = RX_PIN;
  rx_config.clk_src = RMT_CLK_SRC_DEFAULT;
  rx_config.resolution_hz = RX_RESOLUTION_HZ;
  rx_config.mem_block_symbols = 64;
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &rx_channel));

  rmt_rx_event_callbacks_t callbacks = {};
  callbacks.on_recv_done = on_rx_done;
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &callbacks, NULL));
  ESP_ERROR_CHECK(rmt_enable(rx_channel));

  rmt_receive_config_t receive = {};
  receive.signal_range_min_ns = RANGE_MIN_NS;
  receive.signal_range_max_ns = RANGE_MAX_NS;

  uint8_t frame[FRAME_BYTES];
  memset(frame, PATTERN, sizeof(frame));

  int expected_zeros, expected_ones;
  count_bits(expected_zeros, expected_ones);

  bool dumped = false;
  int report_no = 0;

  Stats zero_total, one_total;
  uint32_t frames = 0, passed = 0;
  int wrong_polarity_total = 0, unclassified_total = 0;
  int64_t report_start = esp_timer_get_time();

  while (true) {
    ESP_ERROR_CHECK(rmt_receive(rx_channel, rx_buffer, sizeof(rx_buffer), &receive));
    ws2811_tx_send(frame, sizeof(frame), GAP_US);

    rmt_rx_done_event_data_t done;
    if (xQueueReceive(rx_queue, &done, pdMS_TO_TICKS(100)) != pdTRUE) {
      printf("nothing received on GPIO%d — check the jumper\n", RX_PIN);
      vTaskDelay(pdMS_TO_TICKS(1000));
      report_start = esp_timer_get_time();
      continue;
    }

    Stats zero, one;
    int wrong_polarity = 0, unclassified = 0;

    for (size_t i = 0; i < done.num_symbols; i++) {
      const rmt_symbol_word_t &s = done.received_symbols[i];
      uint32_t high = ticks_to_ns(s.duration0);
      uint32_t low = ticks_to_ns(s.duration1);
      // The final low runs into the gap and is cut at RANGE_MAX_NS or recorded as 0.
      bool check_low = i + 1 < done.num_symbols && s.duration1 != 0;

      if (!dumped) {
        printf("  [%2u] %s %4lu ns, %s %4lu ns\n", (unsigned) i,
               s.level0 ? "high" : "low ", (unsigned long) high,
               s.level1 ? "high" : "low ", (unsigned long) low);
      }

      if (s.level0 != 1 || s.level1 != 0) { wrong_polarity++; continue; }
      if (within(high, ZERO.high_ns)) record(zero, high, low, ZERO, check_low);
      else if (within(high, ONE.high_ns)) record(one, high, low, ONE, check_low);
      else unclassified++;
    }
    dumped = true;

    bool pass = zero.count == expected_zeros && one.count == expected_ones &&
                zero.out_of_window == 0 && one.out_of_window == 0 &&
                wrong_polarity == 0 && unclassified == 0;

    frames++;
    if (pass) passed++;
    merge(zero_total, zero);
    merge(one_total, one);
    wrong_polarity_total += wrong_polarity;
    unclassified_total += unclassified;

    int64_t elapsed = esp_timer_get_time() - report_start;
    if (elapsed < REPORT_US) continue;

    printf("%d s: %lu frames  %lu PASS  %lu FAIL  DMM expect %.2f V\n",
           ++report_no, (unsigned long) frames, (unsigned long) passed,
           (unsigned long) (frames - passed), dmm_volts(frames, elapsed));
    print_stats(ZERO, zero_total, expected_zeros * frames);
    print_stats(ONE, one_total, expected_ones * frames);
    if (wrong_polarity_total || unclassified_total)
      printf("  wrong polarity: %d  unclassified high: %d\n", wrong_polarity_total, unclassified_total);

    zero_total = Stats();
    one_total = Stats();
    frames = passed = 0;
    wrong_polarity_total = unclassified_total = 0;
    report_start = esp_timer_get_time();
  }
}
