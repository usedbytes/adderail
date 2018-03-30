/*
 * Copyright Brian Starkey 2018 <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>

#include "lcd/lcd.h"
#include "uart/uart.h"
#include "one_shot_timer.h"
#include "speed_cntr.h"

#define STEPPER_DDR  DDRC
#define STEPPER_PORT PORTC
#define STEPPER_EN   3
#define STEPPER_DIR  4
#define STEPPER_STEP 5

#define BTN_RIGHT  0x000
#define BTN_UP     0x082
#define BTN_DOWN   0x132
#define BTN_LEFT   0x1df
#define BTN_SELECT 0x2d2

#define ADC_RIGHT   (0x000 + ((BTN_UP - BTN_RIGHT) >> 2))
#define ADC_UP      (BTN_UP + ((BTN_DOWN - BTN_UP) >> 2))
#define ADC_DOWN    (BTN_DOWN + ((BTN_LEFT - BTN_DOWN) >> 2))
#define ADC_LEFT    (BTN_LEFT + ((BTN_SELECT - BTN_LEFT) >> 2))
#define ADC_SELECT  (BTN_SELECT + ((0x3ff - BTN_SELECT) >> 2))

#define NONE   0
#define RIGHT  1
#define UP     2
#define DOWN   3
#define LEFT   4
#define SELECT 5

#define UM_PER_STEP 10

void motor_step(int count, char dir) {
	if (dir) {
		count = -count;
	}
	speed_cntr_Move(count, 100, 100, 2500);
	while(running);
}

void adc_init(void)
{
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (3 << ADPS0);
}

uint16_t adc_read(void) {
	uint16_t data;

	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));

	data = ADCL;
	data |= (ADCH << 8);

	return data;
}

static char get_button(void) {
	uint16_t adc = adc_read();

	if (adc < ADC_RIGHT) {
		return RIGHT;
	} else if (adc < ADC_UP) {
		return UP;
	} else if (adc < ADC_DOWN) {
		return DOWN;
	} else if (adc < ADC_LEFT) {
		return LEFT;
	} else if (adc < ADC_SELECT) {
		return SELECT;
	} else {
		return NONE;
	}
}

static void move_um(int um) {
	static char old_dir = -1;
	int steps;
	char dir = 0;
	if (um < 0) {
		dir = 1;
		um = -um;
	}

	if (dir != old_dir) {
		old_dir = dir;
		if (dir) {
			STEPPER_PORT |= (1 << STEPPER_DIR);
		} else {
			STEPPER_PORT &= ~(1 << STEPPER_DIR);
		}
	}

	steps = um / UM_PER_STEP;
	if (um % UM_PER_STEP >= (UM_PER_STEP >> 2)) {
		steps += 1;
	}

	motor_step(steps, dir);
}

char hex(uint8_t nibble) {
	nibble &= 0xf;
	return nibble < 0xa ? '0' + nibble : 'a' + (nibble - 0xa);
}

static char *uart_poll() {
#define STATE_SYNC 0
#define STATE_CMD  1
	static char cmd[16];
	static char *p;
	static uint8_t state;
	uint16_t u = uart_getc();
	char c = u & 0xff;

	if (u & 0xff00) {
		return NULL;
	}

	switch (state) {
	case STATE_SYNC:
		if (c != '>') {
			break;
		}
		uart_putc(c);
		state++;
		p = cmd;
		break;
	default:
		*p = c;
		p++;
		if (p >= cmd + sizeof(cmd)) {
			uart_putc('!');
			state = STATE_SYNC;
			return NULL;
		}
		uart_putc(c);
		if (c == ';') {
			*p = '\0';
			state = STATE_SYNC;
			uart_puts("\r\n");
			return cmd;
		}
	}

	return NULL;
}

int main(void) {
	DDRB = 1 << 5;
	STEPPER_DDR |= (1 << STEPPER_DIR) | (1 << STEPPER_STEP);

	lcd_init();
	lcd_on();
	lcd_clear();
	lcd_puts("Hello, World!");

	adc_init();

	uart_init(UART_BAUD_SELECT(115200,F_CPU));

	osp_setup(64);
	speed_cntr_Init_Timer1();
	sei();

#define UM_PER_PRESS 50000
	long int i = 0;
	char btn;
	uint8_t speed;
	char *cmd;
	while (1) {
		btn = get_button();
		if (btn == UP) {
			move_um(UM_PER_PRESS);
			i += UM_PER_PRESS;
		} else if (btn == DOWN) {
			move_um(-UM_PER_PRESS);
			i -= UM_PER_PRESS;
		} else if (btn == LEFT) {
			speed -= 1;
			//set_motor_speed(speed);
		} else if (btn == RIGHT) {
			speed += 1;
			//set_motor_speed(speed);
		}

		if (btn) {
			long int abs_i = i < 0 ? -i : i;
			long int um = abs_i % 1000;
			long int mm = abs_i / 1000;

			lcd_set_cursor(0, 1);

			if (i < 0) {
				lcd_printf("-");
			} else {
				lcd_printf(" ");
			}
			lcd_printf("%03ld", mm);
			lcd_printf(".");
			lcd_printf("%03ld", um);
			lcd_printf(" mm", um);
			lcd_printf("%4d", speed);
		}

		cmd = uart_poll();
		if (cmd) {
			uart_puts(cmd);
		}
		_delay_ms(16);
	}

	return 0;
}
