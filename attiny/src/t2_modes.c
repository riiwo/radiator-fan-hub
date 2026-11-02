// Program t2: the button cycles off -> low -> mid -> high -> off.
//
// A short press steps the mode on release; a press held longer than SHORT_MAX_MS
// is left for the XIAO. The mode is kept in EEPROM. In low, mid and high the
// ATtiny owns the bus: PB0 is an output, the mode's frame goes to every node once
// a second, PB2 (sense) is pulled low and the LED shows green, yellow or red.
// Leaving off, sense goes low HANDOVER_MS before PB0 drives, so the XIAO has
// let go of the bus first.
// Stepping from high to off sends one all-off frame, then PB0 and PB2 go back to
// inputs and the LED goes dark. In mid green runs MID_GREEN_ON of every MID_GREEN_PERIOD ticks so red
// shows through and the mix reads yellow. Frames are WS2811 low-speed bits at 9.6 MHz,
// 24 cycles per bit.

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#define TICK_MS 1
#define DEBOUNCE_TICKS 30
#define SHORT_MAX_MS 600
#define FRAME_TICKS 1000
#define MID_GREEN_ON 3
#define MID_GREEN_PERIOD 8
#define HANDOVER_MS 20
#define NODES 50

enum { OFF, LOW, MID, HIGH, MODES };

static const uint8_t frames[MODES][3] = {
  {255, 255, 255},
  {255, 255, 0},
  {255, 0, 255},
  {0, 255, 255},
};

static uint8_t EEMEM saved_mode;

static void send_byte(uint8_t b) {
  uint8_t hi = PORTB | _BV(PB0), lo = PORTB & ~_BV(PB0), n = 8;
  asm volatile(
    "1: out %[port], %[hi]\n\t"
    "nop\n\tnop\n\tnop\n\t"
    "sbrs %[b], 7\n\t"
    "out %[port], %[lo]\n\t"
    "lsl %[b]\n\t"
    "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
    "out %[port], %[lo]\n\t"
    "dec %[n]\n\t"
    "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
    "brne 1b\n\t"
    : [b] "+r" (b), [n] "+r" (n)
    : [port] "I" (_SFR_IO_ADDR(PORTB)), [hi] "r" (hi), [lo] "r" (lo));
}

static void send_frame(uint8_t mode) {
  for (uint8_t node = 0; node < NODES; node++)
    for (uint8_t i = 0; i < 3; i++) send_byte(frames[mode][i]);
}

static void apply(uint8_t mode) {
  if (mode != OFF && !(DDRB & _BV(PB2))) {
    DDRB |= _BV(PB2);
    _delay_ms(HANDOVER_MS);
  }
  uint8_t ddr = 0;
  if (mode != OFF) ddr |= _BV(PB0) | _BV(PB2);
  if (mode == MID || mode == HIGH) ddr |= _BV(PB3);
  if (mode == LOW || mode == MID) ddr |= _BV(PB4);
  DDRB = ddr;
}

int main(void) {
  PORTB = _BV(PB1);
  uint8_t mode = eeprom_read_byte(&saved_mode);
  if (mode >= MODES) mode = OFF;
  apply(mode);
  if (mode != OFF) send_frame(mode);

  uint8_t stable = 1, count = 0, pwm = 0;
  uint16_t held_ms = 0, frame_ticks = 0;
  for (;;) {
    _delay_ms(TICK_MS);
    if (mode == MID) {
      if (++pwm >= MID_GREEN_PERIOD) pwm = 0;
      if (pwm >= MID_GREEN_ON) DDRB &= ~_BV(PB4);
      else DDRB |= _BV(PB4);
    }
    uint8_t up = (PINB >> PB1) & 1;
    if (up == stable) count = 0;
    else if (++count >= DEBOUNCE_TICKS) {
      stable = up;
      count = 0;
      if (!up) held_ms = 0;
      else if (held_ms <= SHORT_MAX_MS) {
        uint8_t next = (mode + 1) % MODES;
        if (next == OFF) {
          DDRB |= _BV(PB0);
          send_frame(OFF);
        }
        mode = next;
        apply(mode);
        if (mode != OFF) send_frame(mode);
        eeprom_update_byte(&saved_mode, mode);
        frame_ticks = 0;
      }
    }
    if (!stable && held_ms < 60000) held_ms += TICK_MS;
    if (mode != OFF && ++frame_ticks >= FRAME_TICKS) {
      send_frame(mode);
      frame_ticks = 0;
    }
  }
}
