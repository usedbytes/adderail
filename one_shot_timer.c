// More info about this program is here...
// http://wp.josh.com/2015/03/05/the-perfect-pulse-some-tricks-for-generating-precise-one-shots-on-avr8/

// Demo of a technique to generate narrow and precise one shot pulses using a
// timer module on an AVR. This demo code is writen for an Arduino and uses
// the Timer2 moudle, but this techniquie should would on other AVRs and timers.

#include <avr/io.h>
#include <stdint.h>
#include "one_shot_timer.h"

// Setup the one-shot pulse generator and initialize with a pulse width that is (cycles) clock counts long

void osp_setup(uint8_t cycles) {
	TCCR2B =  0;      // Halt counter by setting clock select bits to 0 (No clock source).
	// This keeps anyhting from happeneing while we get set up

	TCNT2 = 0x00;     // Start counting at bottom.
	OCR2A = 1;      // Set TOP to 0. This effectively keeps us from counting becuase the counter just keeps reseting back to 0.
	// We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX and then overflow back to 0 and get locked up again.
	OSP_SET_WIDTH(cycles);    // This also makes new OCR values get loaded frm the buffer on every clock cycle.

	TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // OC2B=Set on Match, clear on BOTTOM. Mode 7 Fast PWM.
	TCCR2B = _BV(WGM22) | (2 << CS20);         // Start counting now. WGM22=1 to select Fast PWM mode 7, div8 prescaler

	DDRD |= _BV(3);     // Set pin to output (Note that OC2B = GPIO port PD3 = Arduino Digital Pin 3)
}
