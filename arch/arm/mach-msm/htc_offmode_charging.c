/* arch/arm/mach-msm/htc_offmode_charging.c
 *
 * Copyright (C) 2010 HTC Corporation.
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/gpio_event.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>

#include <mach/htc_offmode_charging.h>
#include "devices.h"
#include "board-buzz.h"

static int powerkey_monitor(struct offmode_charging_data *data)
{
	int cnt = data->duration_ms >> 6;

	while (cnt--) {
		if (gpio_get_value(data->gpio_powerkey) == 0) {
			data->reset_fnt();
		}
		msleep(64);
	}
	return 0;
}

static int offmode_charging_probe(struct platform_device *pdev)
{
	struct offmode_charging_data *pdata = pdev->dev.platform_data;

	printk(KERN_INFO "offmode_charging_probe\n");

	if (board_mfg_mode() == 5) {
		kthread_run(powerkey_monitor, pdata, "restart");
	}
	return 0;
}

static struct platform_driver htc_offmode_charging_driver = {
	.probe = offmode_charging_probe,
	.driver = {
		.name   = "htc_offmode_charging",
		.owner  = THIS_MODULE,
	},
};

static int __init offmode_charging_init(void)
{
	platform_driver_register(&htc_offmode_charging_driver);
	return 0;
}

fs_initcall(offmode_charging_init);
