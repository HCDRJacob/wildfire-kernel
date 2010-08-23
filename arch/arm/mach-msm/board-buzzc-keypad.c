/* arch/arm/mach-msm/board--buzzc.c
 * Copyright (C) 2010 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/


#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>

#include "board-buzzc.h"

static int opt_x_axis_threshold = 1, opt_y_axis_threshold = 1;

struct buzzc_axis_info {
	struct gpio_event_axis_info info;
	uint16_t in_state;
	uint16_t out_state;
	uint16_t temp_state;
};

static bool nav_just_on;
static int nav_on_jiffies;

static unsigned int buzzc_col_gpios[] = {
	BUZZC_GPIO_Q_KP_MKOUT0,
	BUZZC_GPIO_Q_KP_MKOUT1,
	BUZZC_GPIO_Q_KP_MKOUT2,
};

static unsigned int buzzc_row_gpios[] = {
	BUZZC_GPIO_Q_KP_MKIN0_1,
	BUZZC_GPIO_Q_KP_MKIN1_1,
	BUZZC_GPIO_Q_KP_MKIN2_1,
};

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(buzzc_row_gpios) + (row))

static unsigned short buzzc_keymap[ARRAY_SIZE(buzzc_col_gpios) *
					ARRAY_SIZE(buzzc_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = MATRIX_KEY(1, BTN_MOUSE),
	[KEYMAP_INDEX(0, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(0, 2)] = KEY_VOLUMEUP,

	[KEYMAP_INDEX(1, 0)] = KEY_SEARCH,
	[KEYMAP_INDEX(1, 1)] = KEY_BACK,
	[KEYMAP_INDEX(1, 2)] = KEY_VOLUMEDOWN,

	[KEYMAP_INDEX(2, 0)] = KEY_HOME,
	[KEYMAP_INDEX(2, 1)] = KEY_MENU,
	[KEYMAP_INDEX(2, 2)] = KEY_RESERVED,
};

static struct gpio_event_matrix_info buzzc_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = buzzc_keymap,
	.output_gpios = buzzc_col_gpios,
	.input_gpios = buzzc_row_gpios,
	.noutputs = ARRAY_SIZE(buzzc_col_gpios),
	.ninputs = ARRAY_SIZE(buzzc_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		GPIOKPF_REMOVE_PHANTOM_KEYS |
		GPIOKPF_PRINT_UNMAPPED_KEYS /* |
		GPIOKPF_PRINT_MAPPED_KEYS */),
};

static struct gpio_event_direct_entry buzzc_keypad_nav_map[] = {
	{ BUZZC_GPIO_POWER_KEY, KEY_POWER },
};

static struct gpio_event_input_info buzzc_keypad_nav_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = buzzc_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(buzzc_keypad_nav_map)
};

uint16_t buzzc_x_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct buzzc_axis_info *ai =
			container_of(info, struct buzzc_axis_info, info);
	uint16_t out = ai->out_state;

	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if ((ai->in_state ^ in) & 1)
		out--;
	if ((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	if (ai->out_state - ai->temp_state == opt_x_axis_threshold) {
		ai->temp_state++;
		ai->out_state = ai->temp_state;
	} else if (ai->temp_state - ai->out_state == opt_x_axis_threshold) {
		ai->temp_state--;
		ai->out_state = ai->temp_state;
	} else if (abs(ai->out_state - ai->temp_state) > opt_x_axis_threshold)
		ai->temp_state = ai->out_state;

	return ai->temp_state;
}

uint16_t buzzc_y_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct buzzc_axis_info *ai =
			container_of(info, struct buzzc_axis_info, info);
	uint16_t out = ai->out_state;

	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if ((ai->in_state ^ in) & 1)
		out--;
	if ((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	if (ai->out_state - ai->temp_state == opt_y_axis_threshold) {
		ai->temp_state++;
		ai->out_state = ai->temp_state;
	} else if (ai->temp_state - ai->out_state == opt_y_axis_threshold) {
		ai->temp_state--;
		ai->out_state = ai->temp_state;
	} else if (abs(ai->out_state - ai->temp_state) > opt_y_axis_threshold)
		ai->temp_state = ai->out_state;

	return ai->temp_state;
}


int buzzc_nav_power(const struct gpio_event_platform_data *pdata, bool on)
{
	/* gpio_set_value(BUZZ_GPIO_JOGBALL_EN, on); */
	if (on) {
		nav_just_on = 1;
		nav_on_jiffies = jiffies;
	}
	return 0;
}

static uint32_t buzzc_x_axis_gpios[] = {
	BUZZC_GPIO_BALL_LEFT, BUZZC_GPIO_BALL_RIGHT
};

static struct buzzc_axis_info buzzc_x_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(buzzc_x_axis_gpios),
		.dev = 1,
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(buzzc_x_axis_gpios),
		.map = buzzc_x_axis_map,
		.gpio = buzzc_x_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION
			/*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */,
	}
};

static uint32_t buzzc_y_axis_gpios[] = {
	BUZZC_GPIO_BALL_UP, BUZZC_GPIO_BALL_DOWN
};

static struct buzzc_axis_info buzzc_y_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(buzzc_y_axis_gpios),
		.dev = 1,
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(buzzc_y_axis_gpios),
		.map = buzzc_y_axis_map,
		.gpio = buzzc_y_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION
			/*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT  */
	}
};


static struct gpio_event_info *buzzc_input_info[] = {
	&buzzc_keypad_matrix_info.info,
	&buzzc_keypad_nav_info.info,
	&buzzc_x_axis.info.info,
	&buzzc_y_axis.info.info,
};

static struct gpio_event_platform_data buzzc_keypad_data = {
	.names = {
		"buzzc-keypad",
		"buzzc-nav",
		NULL,
	},
	.info = buzzc_input_info,
	.info_count = ARRAY_SIZE(buzzc_input_info),
	.power = buzzc_nav_power,
};

static struct platform_device buzzc_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &buzzc_keypad_data,
	},
};

static int buzzc_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};

static struct keyreset_platform_data buzzc_reset_keys_pdata = {
	.keys_up = buzzc_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

struct platform_device buzzc_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &buzzc_reset_keys_pdata,
};

int __init buzzc_init_keypad(void)
{
	if (platform_device_register(&buzzc_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&buzzc_keypad_device);
}

