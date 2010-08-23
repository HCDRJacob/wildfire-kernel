/* linux/arch/arm/mach-msm/board-buzzc.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_BUZZC_H
#define __ARCH_ARM_MACH_MSM_BOARD_BUZZC_H

#include <mach/board.h>

#define MSM_LINUX_BASE1                 0x02E00000
#define MSM_LINUX_SIZE1                 0x0D200000
#define MSM_LINUX_BASE2                 0x20000000
#define MSM_LINUX_SIZE2                 0x06F00000

#define MSM_PMEM_MDP_BASE               0x00000000
#define MSM_PMEM_MDP_SIZE               0x00800000

#define MSM_FB_BASE                     0x26F00000
#define MSM_FB_SIZE                     0x9b000

#define MSM_RAM_CONSOLE_BASE            MSM_FB_BASE + MSM_FB_SIZE
#define MSM_RAM_CONSOLE_SIZE            128 * SZ_1K

#define MSM_PMEM_ADSP_BASE              0x27000000
#define MSM_PMEM_ADSP_SIZE              0x00800000

#define MSM_PMEM_CAMERA_BASE            0x27800000
#define MSM_PMEM_CAMERA_SIZE            0x00800000

#define MSM_LINUX_BASE                  MSM_EBI1_CS0_BASE + 0x400000
#define MSM_LINUX_SIZE                  0x5100000

#define BUZZC_GPIO_USB_ID_PIN          (19)

/* key */
#define BUZZC_GPIO_POWER_KEY         (20)
#define BUZZC_GPIO_Q_KP_MKOUT0       (35)
#define BUZZC_GPIO_Q_KP_MKOUT1       (34)
#define BUZZC_GPIO_Q_KP_MKOUT2       (33)
#define BUZZC_GPIO_Q_KP_MKIN0_1      (42)
#define BUZZC_GPIO_Q_KP_MKIN1_1      (41)
#define BUZZC_GPIO_Q_KP_MKIN2_1      (40)

#define BUZZC_GPIO_BALL_UP     (94)
#define BUZZC_GPIO_BALL_LEFT   (17)
#define BUZZC_GPIO_BALL_DOWN   (83)
#define BUZZC_GPIO_BALL_RIGHT  (18)

#define BUZZC_GPIO_WIFI_IRQ1         (29)
#define BUZZC_GPIO_WIFI_EN           (116)
#define BUZZC_GPIO_SDMC_CD_N         (38)
#define BUZZC_GPIO_UP_INT            (27)
#define BUZZC_GPIO_UART3_RX          (86)
#define BUZZC_GPIO_UART3_TX          (87)
#define BUZZC_GPIO_VCM_PWDN          (31)
#define BUZZC_GPIO_UP_RESET_N        (76)
#define BUZZC_GPIO_PS_HOLD           (25)
/* flashlight */
#define BUZZC_GPIO_FL_TORCH	(84)
#define BUZZC_GPIO_FL_FLASH	(85)

/* 35mm headset */
#define BUZZC_GPIO_35MM_HEADSET_DET  (112)
// TODO
/* #define BUZZC_GPIO_COMPASS_RST_N     (93) */
/* #define BUZZC_GPIO_COMPASS_INT_N     (37) */
#define BUZZC_PROJECT_NAME           "buzzc"


#define BUZZC_LAYOUTS		{ \
		{ {  1,  0, 0}, {  0,  1, 0}, {0, 0,  1} },  \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0,  1} },  \
		{ {  0,  1, 0}, { -1,  0, 0}, {0, 0,  1} },  \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1,  0} }   \
					}

/* Proximity  */
#define BUZZC_GPIO_PROXIMITY_INT     114 // PS_VOUT
#define BUZZC_GPIO_PROXIMITY_EN      115 // PS_SHDN

#define BUZZC_GPIO_LS_EN             111
#define BUZZC_PS_2V85_EN             117

/* BT */
#define BUZZC_GPIO_BT_UART1_RTS      (43)
#define BUZZC_GPIO_BT_UART1_CTS      (44)
#define BUZZC_GPIO_BT_UART1_RX       (45)
#define BUZZC_GPIO_BT_UART1_TX       (46)
#define BUZZC_GPIO_BT_RESET_N        (131)
#define BUZZC_GPIO_BT_HOST_WAKE      (21)
#define BUZZC_GPIO_BT_CHIP_WAKE      (91)
#define BUZZC_GPIO_BT_SHUTDOWN_N     (113)

#define BUZZC_MDDI_RSTz              (82)

int buzzc_init_mmc(unsigned int);
int __init buzzc_init_keypad(void);

//struct sys_device;
unsigned int buzzc_get_hwid(void);
unsigned int buzzc_get_skuid(void);
unsigned buzzc_engineerid(void);
int buzzc_is_3M_camera(void);

#endif /* GUARD */
