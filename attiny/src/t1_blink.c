// Program t1: red and green of the mode LED alternate at 1 Hz.
//
// The LED is common anode on +5 V, so a colour is on while its pin is an output
// driven low and off while the pin is an input. PB3 is red, PB4 is green. PB0,
// the bus data line, stays an input. Needs the low fuse at 0x7A (9.6 MHz, no
// divide-by-8); with the factory fuse the blink runs eight times slower.

#include <avr/io.h>
#include <util/delay.h>

int main(void) {
  PORTB = 0;
  for (;;) {
    DDRB = _BV(PB3);
    _delay_ms(500);
    DDRB = _BV(PB4);
    _delay_ms(500);
  }
}
