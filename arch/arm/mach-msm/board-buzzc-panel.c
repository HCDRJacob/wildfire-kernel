/*
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
#ifdef CONFIG_HTC_PWRSINK
#include <mach/htc_pwrsink.h>
#endif

#include <mach/pmic.h>
#include "board-buzzc.h"
#include "proc_comm.h"
#include "devices.h"

#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while(0)
#endif

static struct led_trigger *buzzc_lcd_backlight;
static void buzzc_set_backlight(int on)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);

	if (on)
		led_trigger_event(buzzc_lcd_backlight, LED_FULL);
	else
		led_trigger_event(buzzc_lcd_backlight, LED_OFF);
}

struct mddi_table {
	uint32_t reg;
	uint32_t value;
	uint32_t msec;
};

static struct mddi_table buzzc_init_tb[] = {
        {0x01, 0x0000, 0},
        {0x3e, 0x0040, 0},
        {0x16, 0x0008, 0},
        {0x27, 0x000e, 0},
        {0x28, 0x000e, 0},
        {0x71, 0x0068, 0},
        {0x72, 0x0001, 0},
        {0x60, 0x0001, 0},
        {0x93, 0x000f, 10},
        {0x1c, 0x0004, 10},
        {0x1b, 0x0010, 40},
        {0x43, 0x0080, 0},
        {0x44, 0x003c, 0},
        {0x45, 0x000e, 60},
        {0xd0, 0x0000, 0},
        {0xd1, 0x000b, 0},
        {0xd2, 0x000a, 0},
        {0xd3, 0x000f, 0},
        {0xd4, 0x0011, 0},
        {0xd5, 0x003f, 0},
        {0xd6, 0x0018, 0},
        {0xd7, 0x001f, 0},
        {0xd8, 0x0007, 0},
        {0xd9, 0x000e, 0},
        {0xda, 0x0011, 0},
        {0xdb, 0x0017, 0},
        {0xdc, 0x0018, 0},
        {0xdd, 0x0016, 0},
        {0xde, 0x0016, 0},
        {0xdf, 0x000d, 0},
        {0xe0, 0x0013, 0},
        {0xe1, 0x0000, 0},
        {0xe2, 0x002e, 0},
        {0xe3, 0x0030, 0},
        {0xe4, 0x0035, 0},
        {0xe5, 0x0034, 0},
        {0xe6, 0x003f, 0},
        {0xe7, 0x0038, 0},
        {0xe8, 0x003f, 0},
        {0xe9, 0x000c, 0},
        {0xea, 0x0012, 0},
        {0xeb, 0x0009, 0},
        {0xec, 0x0009, 0},
        {0xed, 0x0007, 0},
        {0xee, 0x0008, 0},
        {0xef, 0x000e, 0},
        {0xf0, 0x0011, 0},
        {0xf1, 0x0018, 0},
        {0xf2, 0x000f, 0},
        {0x5f, 0x0000, 0},
        {0x94, 0x00ff, 0},
        {0x95, 0x0024, 0},
        {0x96, 0x0002, 1},
        {0x02, 0x0000, 0},
        {0x03, 0x0000, 0},
        {0x04, 0x0000, 0},
        {0x05, 0x00ef, 0},
        {0x06, 0x0000, 0},
        {0x07, 0x0000, 0},
        {0x08, 0x0001, 0},
        {0x09, 0x003f, 0},
        {0x26, 0x0024, 1},
        {0x26, 0x0038, 1},
        {0x26, 0x003c, 0},
};

#define GPIOSEL_VWAKEINT (1U << 0)
#define INTMASK_VWAKEOUT (1U << 0)

static void
buzzc_process_mddi_table(struct msm_mddi_client_data *client_data,
		struct mddi_table *table, ssize_t count)

{
	int i;
	uint32_t reg, value, msec;

	BUG_ON(!client_data);
	BUG_ON(!table);
	BUG_ON(!count);

	for(i = 0; i < count; i++) {
		reg = table[i].reg;
		value = table[i].value;
		msec = table[i].msec;

		client_data->remote_write(client_data, value, reg);
		if (msec)
			mdelay(msec);
	}
}

static struct vreg *vreg_lcm_2v85;
static struct vreg *vreg_lcm_2v6;

static void
buzzc_mddi_power_client(struct msm_mddi_client_data *cdata, int on)
{
	unsigned id, on_off;

	B("KERN_DEBUG %s: enter.\n", __func__);

	if(on) {

		gpio_set_value(82, 1);
		msleep(3);
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);

		msleep(10);
		id = PM_VREG_PDOWN_GP2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);

		msleep(10);
		gpio_set_value(82, 1);
		msleep(1);
	} else {
		gpio_set_value(82, 0);
		msleep(10);
		on_off = 1;

		vreg_disable(vreg_lcm_2v85);
		id = PM_VREG_PDOWN_GP2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		msleep(1);

		vreg_disable(vreg_lcm_2v6);
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		msleep(10);

		gpio_set_value(82, 0);
		msleep(200);

		id = PM_VREG_PDOWN_MDDI_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
	}

}

#if defined(CONFIG_ARCH_MSM7225) || defined(CONFIG_ARCH_MSM7625)
#define LCM_ID0 57
#define LCM_ID1 58
#else
#define LCM_ID0 _bad_id()
#define LCM_ID1 _bad_id()
#endif

static int
buzzc_mddi_client_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
        client_data->auto_hibernate(client_data, 0);

	buzzc_process_mddi_table(client_data,
			buzzc_init_tb,
			ARRAY_SIZE(buzzc_init_tb));
	client_data->auto_hibernate(client_data, 1);

        return 0;

}

static int
buzzc_mddi_client_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)

{
        return 0;
}

static int panel_inited = 0;
static int
buzzc_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
                  struct msm_mddi_client_data *client_data)
{

	BUG_ON(!bridge_data);
	BUG_ON(!client_data);

	B(KERN_DEBUG "%s: enter.\n", __func__);

        client_data->auto_hibernate(client_data, 0);

	if (!panel_inited)
	{
		panel_inited = 1;
	}
	else
	{
		buzzc_process_mddi_table(client_data,
			buzzc_init_tb,
			ARRAY_SIZE(buzzc_init_tb));
	}
	buzzc_set_backlight(1);
	client_data->auto_hibernate(client_data, 1);
	return 0;

}

static int
buzzc_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
                struct msm_mddi_client_data *client_data)
{
	//FIXME: add de-initial strings heere
	buzzc_set_backlight(0);
	return 0;
}

static void
buzzc_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	B("%s: enter.\n", __func__);
	*mfr_name = 0x0101;
	*product_code = 0x0154;
}

static struct resource resources_msm_fb[] = {
        {
                .start = MSM_FB_BASE,
                .end = MSM_FB_BASE + MSM_FB_SIZE - 1,
                .flags = IORESOURCE_MEM,
        },
};

static struct msm_mddi_bridge_platform_data himax_client_data = {
        .init = buzzc_mddi_client_init,
        .uninit = buzzc_mddi_client_uninit,
	.bridge_type = SAMSUNG_D,
        .blank = buzzc_panel_blank,
        .unblank = buzzc_panel_unblank,
        .fb_data = {
                .xres = 240,
                .yres = 320,
		.width = 42,
		.height = 56,
                .output_format = 0,
        },
};

static struct msm_mddi_platform_data buzzc_pdata = {
	.clk_rate = 61440000,		/*default MDDI frequenct*/
	/*.clk_rate = 68571000,	set MDDI frequency to 68.57Mhz*/
	.power_client = buzzc_mddi_power_client,
	.fixup = buzzc_fixup,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0x4858 << 16 | 0x8356),
			.name = "mddi_c_4858_8356",
			.id = 0,
			.client_data = &himax_client_data,
			.clk_rate = 0,
		},
	},
};

int __init buzzc_panel_init(void)
{
	int rc;

	if (!machine_is_buzzc())
		return 0;

	vreg_lcm_2v85 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcm_2v85))
		return PTR_ERR(vreg_lcm_2v85);

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;
	msm_device_mddi0.dev.platform_data = &buzzc_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	return 0;
}

device_initcall(buzzc_panel_init);
