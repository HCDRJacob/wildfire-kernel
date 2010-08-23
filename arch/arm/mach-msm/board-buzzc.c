/* arch/arm/mach-msm/board-buzzc.c
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
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/capella_cm3602.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>

#include <mach/board.h>
#include <mach/camera.h>

#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <mach/system.h>
#include <mach/vreg.h>
#include <mach/htc_headset_common.h>
#include <mach/audio_jack.h>
#include <mach/msm_serial_debugger.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>

#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/mmc.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/curcial_oj.h>
#include "board-buzzc.h"
#include "proc_comm.h"
#include "gpio_chip.h"

#include <mach/board_htc.h>
#include <mach/msm_serial_hs.h>

#include "devices.h"

#include <mach/atmega_microp.h>
#include <mach/msm_tssc.h>
#include <mach/htc_battery.h>

#include <mach/htc_pwrsink.h>
#include <mach/perflock.h>
#include <mach/drv_callback.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>
#include <mach/msm_flashlight.h>

void msm_init_irq(void);
void msm_init_gpio(void);
void msm_init_pmic_vibrator(void);
void config_buzzc_camera_on_gpios(void);
void config_buzzc_camera_off_gpios(void);
#ifdef CONFIG_MICROP_COMMON
void __init buzzc_microp_init(void);
#endif

#define HSUSB_API_INIT_PHY_PROC	2
#define HSUSB_API_PROG		0x30000064
#define HSUSB_API_VERS MSM_RPC_VERS(1, 1)

static void buzzc_phy_reset(void)
{
	struct msm_rpc_endpoint *usb_ep;
	int rc;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
	} req;

	printk(KERN_INFO "msm_hsusb_phy_reset\n");

	usb_ep = msm_rpc_connect(HSUSB_API_PROG, HSUSB_API_VERS, 0);
	if (IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(usb_ep));
		return;
	}
	rc = msm_rpc_call(usb_ep, HSUSB_API_INIT_PHY_PROC,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

	msm_rpc_close(usb_ep);
}

void config_buzzc_proximity_gpios(int on);

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = GUAGE_MODEM,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};
static int capella_cm3602_power(int pwr_device, uint8_t enable);
static struct microp_function_config microp_functions[] = {
	{
		.name   = "remote-key",
		.category = MICROP_FUNCTION_REMOTEKEY,
		.levels = {0, 33, 38, 82, 95, 167},
		.channel = 1,
		.int_pin = 1 << 5,
	},
	{
		.name   = "microp_intrrupt",
		.category = MICROP_FUNCTION_INTR,
	},
	{
		.name   = "reset-int",
		.category = MICROP_FUNCTION_RESET_INT,
		.int_pin = 1 << 8,
	},
};

static struct microp_function_config microp_lightsensor = {
	.name = "light_sensor",
	.category = MICROP_FUNCTION_LSENSOR,
	.levels = { 0, 0x23, 0x53, 0xED, 0x14C, 0x1AB, 0x20A, 0x3FF, 0x3FF, 0x3FF },
	.channel = 3,
	.int_pin = 1 << 9,
	.golden_adc = 0xCF,
	.ls_power = capella_cm3602_power,
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_lightsensor,
	.irq = MSM_uP_TO_INT(9),
};

static struct microp_led_config led_config[] = {
	{
		.name   = "amber",
		.type = LED_RGB,
	},
	{
		.name   = "green",
		.type = LED_RGB,
	},
	{
		.name   = "button-backlight",
		.type = LED_GPO,
		.mask_w = {0x00, 0x00, 0x40},
	},
};

static struct microp_led_platform_data microp_leds_data = {
	.num_leds	= ARRAY_SIZE(led_config),
	.led_config	= led_config,
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions   = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = BUZZC_GPIO_UP_RESET_N,
	.spi_devices = SPI_GSENSOR,
};

static struct i2c_board_info i2c_microp_devices = {
	I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
	.platform_data = &microp_data,
	.irq = MSM_GPIO_TO_INT(BUZZC_GPIO_UP_INT),
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_buzzc_camera_on_gpios,
	.camera_gpio_off = config_buzzc_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};


static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 118,
	.vcm_pwd        = BUZZC_GPIO_VCM_PWDN,
	.pdata          = &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name      = "msm_camera_mt9t013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = BUZZC_LAYOUTS,
	.project_name = BUZZC_PROJECT_NAME,
	//TODO
	//.reset = BUZZC_GPIO_COMPASS_RST_N,
	//.intr = BUZZC_GPIO_COMPASS_INT_N,
};

static struct i2c_board_info i2c_devices[] = {
	{
		/*I2C_BOARD_INFO("s5k4b2fx", 0x22 >> 1),*/
		I2C_BOARD_INFO("s5k4b2fx", 0x22),
		/* .irq = TROUT_GPIO_TO_INT(TROUT_GPIO_CAM_BTN_STEP1_N), */
	},
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),   /*3M bayer sensor driver*/
		.platform_data = &msm_camera_device_data,
	},
};

static struct i2c_board_info i2c_sensor[] = {
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		// .irq = MSM_GPIO_TO_INT(BUZZC_GPIO_COMPASS_INT_N), TODO
	},
};

static struct h2w_platform_data buzzc_h2w_data = {
};

static struct platform_device buzzc_h2w = {
	.name		= "htc_headset",
	.id			= -1,
	.dev		= {
		.platform_data	= &buzzc_h2w_data,
	},
};

static struct audio_jack_platform_data buzzc_jack_data = {
	.gpio	= BUZZC_GPIO_35MM_HEADSET_DET,
};


static struct platform_device buzzc_audio_jack = {
	.name		= "audio-jack",
	.id			= -1,
	.dev		= {
		.platform_data	= &buzzc_jack_data,
	},
};

static struct pwr_sink buzzc_pwrsink_table[] = {
	{
		.id     = PWRSINK_AUDIO,
		.ua_max = 100000,
	},
	{
		.id     = PWRSINK_BACKLIGHT,
		.ua_max = 125000,
	},
	{
		.id     = PWRSINK_LED_BUTTON,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_LED_KEYBOARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_GP_CLK,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_BLUETOOTH,
		.ua_max = 15000,
	},
	{
		.id     = PWRSINK_CAMERA,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_SDCARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_VIDEO,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id     = PWRSINK_SYSTEM_LOAD,
		.ua_max = 100000,
		.percent_util = 38,
	},
};

static int buzzc_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void buzzc_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void buzzc_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int buzzc_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data buzzc_pwrsink_data = {
	.num_sinks      = ARRAY_SIZE(buzzc_pwrsink_table),
	.sinks          = buzzc_pwrsink_table,
	.suspend_late	= buzzc_pwrsink_suspend_late,
	.resume_early	= buzzc_pwrsink_resume_early,
	.suspend_early	= buzzc_pwrsink_suspend_early,
	.resume_late	= buzzc_pwrsink_resume_late,
};

static struct platform_device buzzc_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev    = {
		.platform_data = &buzzc_pwrsink_data,
	},
};

static struct msm_pmem_setting pmem_setting = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};

static ssize_t buzzc_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":7:440:70:55"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":77:440:100:55"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":159:440:70:55"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":230:440:70:55"
	   "\n");
}

static struct kobj_attribute buzzc_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.tssc-manager",
		.mode = S_IRUGO,
	},
	.show = &buzzc_virtual_keys_show,
};

static struct attribute *buzzc_properties_attrs[] = {
	&buzzc_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group buzzc_properties_attr_group = {
	.attrs = buzzc_properties_attrs,
};

static struct tssc_ts_platform_data tssc_ts_device_data = {
	.version = 1,
	.x_min = 0,
	.x_max = 1023,
	.y_min = 0,
	.y_max = 1023,
	.cal_range_x = 653,
	.cal_range_y = 672,
	.cal_err = 50,
	.screen_width = 240,
	.screen_height = 400,
	.cal_x = { 15, 223, 15, 223, 119},
	.cal_y = { 20, 20, 378, 378, 199},
	.abs_pressure_min = 0,
	.abs_pressure_max = 255,
};

static struct platform_device tssc_ts_device = {
	.name	= "tssc-manager",
	.id	= -1,
	.dev	= {
		.platform_data	= &tssc_ts_device_data,
	},
};

static struct msm_i2c_device_platform_data buzzc_i2c_device_data = {
	.i2c_clock = 400000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_4MA,
};

static struct platform_device buzzc_rfkill = {
	.name = "buzzc_rfkill",
	.id = -1,
};

static int __capella_cm3602_power(int on)
{
	printk(KERN_DEBUG "%s: Turn the capella_cm3602 power %s\n",
		__func__, (on) ? "on" : "off");
	if (on) {
		config_buzzc_proximity_gpios(1);
		gpio_direction_output(BUZZC_GPIO_PROXIMITY_EN, 1);
		gpio_direction_output(BUZZC_PS_2V85_EN, 1);
	} else {
		gpio_direction_output(BUZZC_PS_2V85_EN, 0);
		gpio_direction_output(BUZZC_GPIO_PROXIMITY_EN, 0);
		config_buzzc_proximity_gpios(0);
	}
	return 0;
}

static DEFINE_MUTEX(capella_cm3602_lock);
static unsigned int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
}

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.p_out = BUZZC_GPIO_PROXIMITY_INT,
	.p_en = BUZZC_GPIO_PROXIMITY_EN,
	.power = capella_cm3602_power,
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};
/* End Proximity Sensor (Capella_CM3602)*/

/*
static struct gpio_led buzz_led_list[] = {
	{
		.name = "caps",
		.gpio = BUZZ_GPIO_LED_CAP_LED_EN,
	},
	{
		.name = "func",
		.gpio = BUZZ_GPIO_LED_FN_LED_EN,
	},
};

static struct gpio_led_platform_data buzz_leds_data = {
	.num_leds	= ARRAY_SIZE(buzz_led_list),
	.leds		= buzz_led_list,
};

static struct platform_device buzz_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &buzz_leds_data,
	},
};*/


static void config_buzzc_flashlight_gpios(void)
{
	static uint32_t buzzc_flashlight_gpio_table[] = {
		PCOM_GPIO_CFG(BUZZC_GPIO_FL_TORCH, 0, GPIO_OUTPUT, GPIO_NO_PULL,
								GPIO_2MA),
		PCOM_GPIO_CFG(BUZZC_GPIO_FL_FLASH, 0, GPIO_OUTPUT, GPIO_NO_PULL,
								GPIO_2MA),
	};
	config_gpio_table(buzzc_flashlight_gpio_table,
		ARRAY_SIZE(buzzc_flashlight_gpio_table));
}

static struct flashlight_platform_data buzzc_flashlight_data = {
	.gpio_init = config_buzzc_flashlight_gpios,
	.torch = BUZZC_GPIO_FL_TORCH,
	.flash = BUZZC_GPIO_FL_FLASH,
	.flash_duration_ms = 600,
};

static struct platform_device buzzc_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev = {
		.platform_data  = &buzzc_flashlight_data,
	},
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&buzzc_h2w,
	&htc_battery_pdev,
	&buzzc_audio_jack,
	&tssc_ts_device,
	&msm_camera_sensor_mt9t013,
	&buzzc_rfkill,
#ifdef CONFIG_HTC_PWRSINK
	&buzzc_pwr_sink,
#endif
	/* &buzzc_oj, TODO:JOGALL */
#ifdef CONFIG_INPUT_CAPELLA_CM3602
	&capella_cm3602,
#endif
	/* &buzz_leds, */
	&buzzc_flashlight_device,
};

extern struct sys_timer msm_timer;

static void __init buzzc_init_irq(void)
{
	printk("buzzc_init_irq()\n");
	msm_init_irq();
}

static uint cpld_iset;
static uint cpld_charger_en;
static uint cpld_usb_h2w_sw;
static uint opt_disable_uart3;
static char *keycaps = "";

module_param_named(iset, cpld_iset, uint, 0);
module_param_named(charger_en, cpld_charger_en, uint, 0);
module_param_named(usb_h2w_sw, cpld_usb_h2w_sw, uint, 0);
module_param_named(disable_uart3, opt_disable_uart3, uint, 0);
module_param_named(keycaps, keycaps, charp, 0);

static char bt_chip_id[10] = "bcm4329";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

static void buzzc_reset(void)
{
	gpio_set_value(BUZZC_GPIO_PS_HOLD, 0);
}

static uint32_t proximity_on_gpio_table[] = {
	PCOM_GPIO_CFG(BUZZC_GPIO_PROXIMITY_INT,
		0, GPIO_INPUT, GPIO_NO_PULL, 0), /* PS_VOUT */
};

static uint32_t proximity_off_gpio_table[] = {
	PCOM_GPIO_CFG(BUZZC_GPIO_PROXIMITY_INT,
		0, GPIO_INPUT, GPIO_PULL_DOWN, 0) /* PS_VOUT */
};

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

void config_buzzc_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_buzzc_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

/* for bcm */
static char bdaddress[20];
extern unsigned char *get_bt_bd_ram(void);

static void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);
	sprintf(bdaddress, "%02x:%02x:%02x:%02x:%02x:%02x",
		cTemp[0], cTemp[1], cTemp[2], cTemp[3], cTemp[4], cTemp[5]);
	printk(KERN_INFO "YoYo--BD_ADDRESS=%s\n", bdaddress);
}

module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");

void config_buzzc_proximity_gpios(int on)
{
	if (on)
		config_gpio_table(proximity_on_gpio_table,
			ARRAY_SIZE(proximity_on_gpio_table));
	else
		config_gpio_table(proximity_off_gpio_table,
			ARRAY_SIZE(proximity_off_gpio_table));
}

static uint32_t buzzc_serial_debug_table[] = {
	/* config as serial debug uart */
	PCOM_GPIO_CFG(BUZZC_GPIO_UART3_RX, 1,
			GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* UART3 RX */
	PCOM_GPIO_CFG(BUZZC_GPIO_UART3_TX, 1,
			GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* UART3 TX */
};

static void buzzc_config_serial_debug_gpios(void)
{
	config_gpio_table(buzzc_serial_debug_table,
			ARRAY_SIZE(buzzc_serial_debug_table));
}


static void __init config_gpios(void)
{
	buzzc_config_serial_debug_gpios();
	config_buzzc_camera_off_gpios();
}

static struct msm_acpu_clock_platform_data buzzc_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
#if defined(CONFIG_TURBO_MODE)
	.wait_for_irq_khz = 176000,
#else
	.wait_for_irq_khz = 128000,
#endif
};

static unsigned buzzc_perf_acpu_table[] = {
	245760000,
	480000000,
	528000000,
};

static struct perflock_platform_data buzzc_perflock_data = {
	.perf_acpu_table = buzzc_perf_acpu_table,
	.table_size = ARRAY_SIZE(buzzc_perf_acpu_table),
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(BUZZC_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = BUZZC_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = BUZZC_GPIO_BT_HOST_WAKE,

};
#endif

static void __init buzzc_init(void)
{
	int rc;
	struct kobject *properties_kobj;

	printk("buzzc_init() revision=%d\n", system_rev);
	printk(KERN_INFO "mfg_mode=%d\n", board_mfg_mode());
	if (board_mfg_mode() == 1) {
		tssc_ts_device_data.cal_err = 25;
		printk(KERN_INFO "cal_err=%d\n", tssc_ts_device_data.cal_err);
	}


	/* for bcm */
	bt_export_bd_address();

	/*
	 * Setup common MSM GPIOS
	 */
	config_gpios();

	/* We need to set this pin to 0 only once on power-up; we will
	 * not actually enable the chip until we apply power to it via
	 * vreg.
	 */
	gpio_direction_output(BUZZC_GPIO_LS_EN, 0);
	/* disable power for cm3602 chip */
	__capella_cm3602_power(0);

	msm_hw_reset_hook = buzzc_reset;

	msm_acpu_clock_init(&buzzc_clock_data);
	perflock_init(&buzzc_perflock_data);
	/* adjust GPIOs based on bootloader request */

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1,
				MSM_GPIO_TO_INT(BUZZC_GPIO_UART3_RX));
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_device_uart_dm1.name = "msm_serial_hs_bcm";	/* for bcm */
	msm_add_serial_devices(3);
#else
	msm_add_serial_devices(0);
#endif

	msm_add_serial_devices(2);
	msm_add_usb_id_pin_gpio(BUZZC_GPIO_USB_ID_PIN);
	msm_add_usb_devices(buzzc_phy_reset, NULL);

	msm_add_mem_devices(&pmem_setting);
	msm_init_pmic_vibrator();
#ifdef CONFIG_MICROP_COMMON
	buzzc_microp_init();
#endif

	rc = buzzc_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	properties_kobj = kobject_create_and_add("board_properties", NULL);

	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
					 &buzzc_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("failed to create board_properties\n");

	msm_device_i2c.dev.platform_data = &buzzc_i2c_device_data;

	platform_add_devices(devices, ARRAY_SIZE(devices));

	i2c_register_board_info(0, i2c_sensor, ARRAY_SIZE(i2c_sensor));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	i2c_register_board_info(0 , &i2c_microp_devices, 1);

	buzzc_init_keypad();
}

static void __init buzzc_fixup(struct machine_desc *desc, struct tag *tags,
				char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = MSM_LINUX_BASE1;
	mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE1);
	mi->bank[0].size = MSM_LINUX_SIZE1;
	mi->bank[1].start = MSM_LINUX_BASE2;
	mi->bank[1].node = PHYS_TO_NID(MSM_LINUX_BASE2);
	mi->bank[1].size = MSM_LINUX_SIZE2;
}

static void __init buzzc_map_io(void)
{
	printk("buzzc_init_map_io()\n");
	msm_map_common_io();
	msm_clock_init();
}

MACHINE_START(BUZZC, "buzzc")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x02E00100,
	.fixup          = buzzc_fixup,
	.map_io         = buzzc_map_io,
	.init_irq       = buzzc_init_irq,
	.init_machine   = buzzc_init,
	.timer          = &msm_timer,
MACHINE_END
