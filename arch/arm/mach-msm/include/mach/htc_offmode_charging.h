/* arch/arm/mach-msm/include/mach/htc_offmode_charging.h
 * Copyright (C) 2010 HTC Incorporated
 * Author: Andy.YS_Wang (Andy.YS_Wang@htc.com)
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

struct offmode_charging_data {
	int duration_ms;
	int gpio_powerkey;
	void (*reset_fnt)(void);
};
