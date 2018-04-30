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
#include <avr/sleep.h>
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

#define MM_REV 4
#define UM_REV (MM_REV * 1000)
#define STEP_REV (200 * 16)
#define NM_PER_STEP (((long)UM_REV * 1000) / STEP_REV)

#define MAX_SPEED (long)((long)(UM_REV) * 5)
#define MAX_ACCEL (long)((long)(UM_REV) * 20)

#define ARRAY_SIZE(_x) (sizeof(_x) / sizeof((_x)[0]))

struct task {
	void (*tick)(struct task *);
};

struct task *current;

ISR(TIMER0_COMPA_vect)
{
	current->tick(current);
}

long speed = MAX_SPEED;
long accel = MAX_ACCEL;

long um_to_crad(long um) {
	return (um * (long)(3.14159 * 2 * 1000)) / ((long)UM_REV * 10);
}

void motor_step(long int count, char dir, char async) {
	static char old_dir = -1;
	if (dir != old_dir) {
		old_dir = dir;
		if (dir) {
			STEPPER_PORT |= (1 << STEPPER_DIR);
		} else {
			STEPPER_PORT &= ~(1 << STEPPER_DIR);
		}
	}
	speed_cntr_Move(count, um_to_crad(accel), um_to_crad(accel), um_to_crad(speed));

	while(!async && running);
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

static void move_um(long int um, char async) {
	static char old_dir = -1;
	long int steps;
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

	steps = (um * 1000) / NM_PER_STEP;

	motor_step(steps, dir, async);
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

struct rail {
	int32_t position_um;
	uint32_t speed_ums;
	uint32_t accel_ums2;
};

struct rail rail = {
	.position_um = 0,
	.speed_ums = MAX_SPEED / 2,
	.accel_ums2 = MAX_ACCEL / 2,
};

enum menu_item_type {
	MENU_TYPE_MENU,
	MENU_TYPE_U32,
	MENU_TYPE_TASK,
};

enum unit {
	UNIT_UM,
	UNIT_UM_PER_SECOND,
	UNIT_UM_PER_SECOND_PER_SECOND,
};

struct menu_item_u32 {
	uint32_t *val;
	uint32_t min, max, step;
	enum unit units;
};

struct menu_item_menu {
	uint8_t n_items;
	struct menu_item *items;
};

struct menu_item {
	// TODO: Progmem
	const char *text;
	enum menu_item_type type;
	union {
		struct menu_item_menu menu;
		struct menu_item_u32 u32;
		struct task *task;
	};
};

struct menu_item settings_menu[] = {
	{
		.text = "Speed",
		.type = MENU_TYPE_U32,
		.u32 = {
			.val = &rail.speed_ums,
			.min = MAX_SPEED / 16,
			.max = MAX_SPEED,
			.step = MAX_SPEED / 16,
			.units = UNIT_UM_PER_SECOND,
		},
	},
	{
		.text = "Acceleration",
		.type = MENU_TYPE_U32,
		.u32 = {
			.val = &rail.accel_ums2,
			.min = MAX_ACCEL / 16,
			.max = MAX_ACCEL,
			.step = MAX_ACCEL / 16,
			.units = UNIT_UM_PER_SECOND_PER_SECOND,
		},
	},
};

struct stack {
	int32_t step_dist;
	uint32_t pause_ms;
	uint16_t n_steps;
};

struct stack_task {
	void (*tick)(struct task *t);
	struct stack s;

	uint16_t n_steps;
	uint32_t pause_ms;
	uint8_t status;
};

static void stack_tick(struct task *t);
struct stack_task stack_task = {
	.tick = stack_tick,
	.s = {
		.step_dist = 2000,
		.pause_ms = 1000,
		.n_steps = 3,
	},
};

#define MENU_MAX_DEPTH 4

struct menu {
	struct menu_item *stack[MENU_MAX_DEPTH];
	struct menu_item *current;
	uint8_t idx[MENU_MAX_DEPTH];
	uint8_t depth;
};

struct menu_task {
	void (*tick)(struct task *);
	struct menu m;
};

struct menu_task menu_task;
void menu_show(struct menu *m);
static void stack_tick(struct task *t)
{
#define STACK_START    0
#define STACK_MOVE     1
#define STACK_RUNNING  2
#define STACK_PAUSE    3
#define STACK_WAITKEY  4
	struct stack_task *st = (struct stack_task *)t;
	static uint8_t cooldown = 250;
	char btn = NONE;

	if (cooldown) {
		cooldown--;
	} else {
		btn = get_button();
	}

	if (st->status == STACK_WAITKEY)// && btn != NONE)
	{
		/* Cooldown for the next time we enter. Hack! */
		cooldown = 250;
		st->status = STACK_START;
		current = &menu_task;
		menu_show(&menu_task.m);
		return;
	}

	if (btn == SELECT) {
		if (st->status & 0x80) {
			/* Resume */
			st->status &= ~0x80;

			lcd_clear();
			lcd_set_cursor(0, 0);
			lcd_printf("Stack Running");
			lcd_set_cursor(0, 1);
			lcd_printf("%4d Steps left", st->n_steps);
		} else {
			st->status |= 0x80;

			lcd_clear();
			lcd_set_cursor(0, 0);
			lcd_printf("Stack Paused");
			lcd_set_cursor(0, 1);
			lcd_printf("SELECT to resume");
		}
		cooldown = 250;
	} else if (btn != NONE) {
		/* Abort on button press other than SELECT */
		lcd_clear();
		lcd_set_cursor(0, 0);
		lcd_printf("Stack Aborted");
		lcd_set_cursor(0, 1);
		lcd_printf("Any key to exit");

		st->status = STACK_WAITKEY;
		cooldown = 250;
		return;
	}

	switch (st->status) {
	case STACK_START:
		st->n_steps = st->s.n_steps;

		lcd_clear();
		lcd_set_cursor(0, 0);
		lcd_printf("Stack Running");
		lcd_set_cursor(0, 1);
		lcd_printf("%4d Steps left", st->n_steps);

		st->pause_ms = st->s.pause_ms;
		st->status = STACK_PAUSE;
		break;
	case STACK_MOVE:
		move_um(st->s.step_dist, 1);
		st->status = STACK_RUNNING;

		break;
	case STACK_RUNNING:
		if (running) {
			break;
		}
		st->n_steps--;
		if (!st->n_steps) {
			st->status = STACK_WAITKEY;

			lcd_clear();
			lcd_set_cursor(0, 0);
			lcd_printf("Stack Finished");
			lcd_set_cursor(0, 1);
			lcd_printf("Any key to exit");
			break;
		}

		st->pause_ms = st->s.pause_ms;
		st->status = STACK_PAUSE;

		lcd_set_cursor(0, 1);
		lcd_printf("%4d", st->n_steps);
		break;
	case STACK_PAUSE:
		if (st->pause_ms--) {
			break;
		} else {
			st->status = STACK_MOVE;
		}
		break;
	default:
		break;
	}
};

struct menu_item stack_menu[] = {
	{
		.text = "Run",
		.type = MENU_TYPE_TASK,
		.task = &stack_task,
	},
	{
		.text = "# Steps",
		.type = MENU_TYPE_U32,
	},
	{
		.text = "Step Dist.",
		.type = MENU_TYPE_U32,
	},
	{
		.text = "Pause Time",
		.type = MENU_TYPE_U32,
	},
};

struct menu_item main_menu[] = {
	{
		.text="Stack",
		.type = MENU_TYPE_MENU,
		.menu = {
			.n_items = ARRAY_SIZE(stack_menu),
			.items = stack_menu,
		},
	},
	{
		.text="Settings",
		.type = MENU_TYPE_MENU,
		.menu = {
			.n_items = ARRAY_SIZE(settings_menu),
			.items = settings_menu,
		},
	},
};

struct menu_item menu_start = {
	.text = "Main Menu",
	.type = MENU_TYPE_MENU,
	.menu = {
		.n_items = ARRAY_SIZE(main_menu),
		.items = main_menu,
	},
};

void menu_init(struct menu *m, struct menu_item *entry)
{
	m->stack[0] = entry;
	m->idx[0] = 0;
	m->depth = 0;

	m->current = &entry->menu.items[0];
}

void menu_show(struct menu *m)
{
	lcd_clear();
	lcd_set_cursor(0, 0);
	if (m->depth > 0) {
		lcd_puts("^- ");
	} else {
		lcd_puts("   ");
	}
	lcd_puts(m->stack[m->depth]->text);
	lcd_set_cursor(0, 1);
	lcd_puts("< ");
	lcd_puts((char *)m->current->text);
	lcd_set_cursor(LCD_COL_COUNT - 2, 1);
	lcd_puts(" >");
}

static void __menu_set_idx(struct menu *m, uint8_t idx)
{
	m->current = &m->stack[m->depth]->menu.items[idx];
	m->idx[m->depth] = idx;
}

void menu_next(struct menu *m)
{
	uint8_t idx = m->idx[m->depth];

	idx++;
	if (idx == m->stack[m->depth]->menu.n_items) {
		idx = 0;
	}

	__menu_set_idx(m, idx);
}

void menu_prev(struct menu *m)
{
	uint8_t idx = m->idx[m->depth];

	idx--;
	if (idx == (uint8_t)-1) {
		idx = m->stack[m->depth]->menu.n_items - 1;
	}

	__menu_set_idx(m, idx);
}

static void __menu_push(struct menu *m)
{
	m->depth++;
	m->stack[m->depth] = m->current;

	__menu_set_idx(m, 0);
}

static void __menu_pop(struct menu *m)
{
	if (m->depth == 0) {
		return;
	}

	m->depth--;
	__menu_set_idx(m, m->idx[m->depth]);
}

void menu_select(struct menu *m)
{
	if (m->current->type == MENU_TYPE_MENU) {
		__menu_push(m);
	} else if (m->current->type == MENU_TYPE_TASK) {
		current = m->current->task;
	}
}

void menu_up(struct menu *m)
{
	__menu_pop(m);
}

static void menu_tick(struct task *t);
struct menu_task menu_task = {
	.tick = menu_tick,
};

static void menu_tick(struct task *t)
{
	struct menu_task *mt = (struct menu_task *)t;
	static uint8_t cooldown = 0;
	char btn;

	if (cooldown) {
		cooldown--;
		return;
	}

	btn = get_button();
	if (btn == LEFT) {
		menu_prev(&mt->m);
	} else if (btn == RIGHT) {
		menu_next(&mt->m);
	} else if (btn == SELECT) {
		menu_select(&mt->m);
	} else if (btn == UP) {
		menu_up(&mt->m);
	}

	if (btn != NONE) {
		menu_show(&mt->m);
		cooldown = 250;
	}
}

static void process_cmd(const char *cmd)
{
	uint8_t i = 10;
	const uint8_t *c = (uint8_t *)cmd;
	uint8_t *s = (uint8_t *)&stack_task.s;

	c = (uint8_t *)cmd;
	while (i--) {
		*s = *c;
		s++;
		c++;
	}

	current = &stack_task;
}

int main(void) {
	DDRB = 1 << 5;
	STEPPER_DDR |= (1 << STEPPER_DIR) | (1 << STEPPER_STEP);

	lcd_init();
	lcd_on();
	lcd_clear();
	lcd_set_cursor(0, 0);

	adc_init();

	uart_init(UART_BAUD_SELECT(115200,F_CPU));

	osp_setup(64);
	speed_cntr_Init_Timer1();
	sei();

	menu_init(&menu_task.m, &menu_start);
	menu_show(&menu_task.m);

	current = &menu_task;
	/* Prescale 1024, count 16 is 0.001024 seconds */
	OCR0A = 16;
	TCCR0A = (2 << WGM00);
	TCCR0B = (5 << CS00);
	TIMSK0 = (1 << OCIE0A);

	char *cmd;

	while (1) {
		//sleep_enable();
		cmd = uart_poll();
		if (cmd) {
			/*
			lcd_clear();
			lcd_set_cursor(0, 0);
			char i;
			for (i = 0; i < 12; i++) {
				lcd_printf("%2x", cmd[i]);
			}
			*/
			process_cmd(cmd);
		}
	}
#if 0
#define UM_PER_PRESS 10
	long int i = 0;
	char btn;
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
			//set_motor_speed(speed);
		} else if (btn == RIGHT) {
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
			//lcd_printf("%4d", speed);
		}

		cmd = uart_poll();
		if (cmd) {
			uart_puts(cmd);
		}
		_delay_ms(16);
	}
#endif

	return 0;
}
