// More info about this program is here...
// http://wp.josh.com/2015/03/05/the-perfect-pulse-some-tricks-for-generating-precise-one-shots-on-avr8/
#ifndef __ONE_SHOT_TIMER_H__
#define __ONE_SHOT_TIMER_H__
#include <avr/io.h>
#include <stdint.h>

#define OSP_SET_WIDTH(cycles) (OCR2B = 0xff-(cycles-1))

// Fire a one-shot pulse. Use the most recently set width. 
#define OSP_FIRE() (TCNT2 = OCR2B - 1)

// Test there is currently a pulse still in progress
#define OSP_INPROGRESS() (TCNT2>0)

// Fire a one-shot pusle with the specififed width. 
// Order of operations in calculating m must avoid overflow of the unint8_t.
// TCNT2 starts one count lower than the match value becuase the chip will block any compare on the cycle after setting a TCNT. 
#define OSP_SET_AND_FIRE(cycles) {uint8_t m=0xff-(cycles-1); OCR2B=m;TCNT2 =m-1;}

void osp_setup(uint8_t cycles);

#endif /* __ONE_SHOT_TIMER_H__ */
