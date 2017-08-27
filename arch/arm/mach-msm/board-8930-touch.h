/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * arch/arm/mach-msm/board-8960-touch.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#ifndef __ARCH_ARM_MACH_MSM_BOARD_MSM8960_TOUCH_H
#define __ARCH_ARM_MACH_MSM_BOARD_MSM8960_TOUCH_H

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT_KC
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c/atmel_mxt_kc.h>


#define MXT_TS_RESET_GPIO	6
#define MXT_TS_GPIO_IRQ		7

#define MXT_PANEL_HEIGHT	800
#define MXT_PANEL_WIDTH		480

#define MXT_BOARD_INFO		I2C_BOARD_INFO("mXT224S", 0x94 >> 1)

static inline int mxt_init_hw(void)
{
	int rc;

	rc = gpio_request(MXT_TS_GPIO_IRQ, "mxt_ts_irq_gpio");
	if (rc) {
		pr_err("%s: unable to request mxt_ts_irq gpio [%d]\n"
						, __func__, MXT_TS_GPIO_IRQ);
		goto err_irq_gpio_req;
	}

	rc = gpio_direction_input(MXT_TS_GPIO_IRQ);
	if (rc) {
		pr_err("%s: unable to set_direction for mxt_ts_irq gpio [%d]\n"
						, __func__, MXT_TS_GPIO_IRQ);
		goto err_irq_gpio_set;
	}

	rc = gpio_request(MXT_TS_RESET_GPIO, "mxt_reset_gpio");
	if (rc) {
		pr_err("%s: unable to request mxt_reset gpio [%d]\n"
						, __func__, MXT_TS_RESET_GPIO);
		goto err_reset_gpio_req;
	}

	rc = gpio_direction_output(MXT_TS_RESET_GPIO, 0);
	if (rc) {
		pr_err("%s: unable to set_direction for mxt_reset gpio [%d]\n"
						, __func__, MXT_TS_RESET_GPIO);
		goto err_reset_gpio_set;
	}

	return 0;

err_reset_gpio_set:
	gpio_free(MXT_TS_RESET_GPIO);
err_reset_gpio_req:
err_irq_gpio_set:
	gpio_free(MXT_TS_GPIO_IRQ);
err_irq_gpio_req:
	return rc;
}

static inline int mxt_reset_hw(void)
{
	gpio_set_value(MXT_TS_RESET_GPIO, 0);
	msleep(1);
	gpio_set_value(MXT_TS_RESET_GPIO, 1);

	return 0;
}

static inline int mxt_shutdown(void)
{
	gpio_set_value(MXT_TS_RESET_GPIO, 0);
	msleep(1);

	return 0;
}

/* Register No, Data Size */
/* Data[0], Data[1], ... */
static const u8 mxt_config_data0[] = {
	/* T37 Object */
	/* T44 Object */
	/* T5 Object */
	/* T6 Object */
	/* T38 Object */
	/* T7 Object */
	   7,   4,
	  32,   8,  15,   2,
	/* T8 Object */
	   8,  10,
	  24,   0,   1,   1,   0,   0,  10,   0, 120,   0,
	/* T9 Object */
	   9,  36,
	 143,   0,   0,  19,  11,   0,  96,  55,   1,   5,
	  10,  25,   5,  80,   5,   5,  75,   0,  95,   3,
	 223,   1,   2,   2,  32,  32, 230,  40, 178,  57,
	  80,   5,   0,   0,   1,   0,
	/* T15 Object */
	  15,  11,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	   0,
	/* T18 Object */
	/* T19 Object */
	  19,   6,
	   0,   0,   0,   0,   0,   0,
	/* T23 Object */
	  23,  15,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	   0,   0,   0,   0,   0,
	/* T25 Object */
	  25,  15,
	   0,   0,0x30,0x75,0x20,0x4e,   0,   0,   0,   0,
	   0,   0,   0,   0, 200,
	/* T40 Object */
	  40,   5,
	   0,   0,   0,   0,   0,
	/* T42 Object */
	  42,  10,
	   3,  38,  38,  35,   0,   0,   0,   0,   0,   0,
	/* T46 Object */
	  46,  10,
	   0,   0,  16,  32,   0,   0,   3,   0,   0,   1,
	/* T47 Object */
	  47,  13,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	   0,   0,   0,
	/* T55 Object */
	  55,   6,
	   1,  20,  64,  12, 128,   0,
	/* T56 Object */
	  56,  42,
	   0,   0,   1,  41,  12,  12,  12,  12,  12,  11,
	  10,  12,  12,  11,  12,  12,  12,  12,  12,  12,
	  11,  12,   0,   0,   0,   0,   0,   0,   0,   0,
	   1,   2,  20,   4,   0,   0,   0,   0,   0,   0,
	   0,   0,
	/* T57 Object */
	  57,   3,
	   0,   0,   0,
	/* T61 Object */
	  61,  10,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	/* T62 Object */
	  62,  54,
	   3,   3,   0,  18,   0,   0, 240,   0,   0,   0,
	   0,   0,   0,   0,   5,   0,  10,   5,   5,  96,
	  50,   5,  52,   0, 100,   6,   6,   4,  64,   0,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	   0,   0,   0,   0,
};

/* Default Register is [BOOT STATE:CALL OFF] */
/* Register No, Data Size */
/* Data[0], Data[1], ... */
static const u8 mxt_config_data1[] = {
	/* T37 Object */
	/* T44 Object */
	/* T5 Object */
	/* T6 Object */
	/* T38 Object */
	/* T7 Object */
	   7,   4,
	 255,   8,   5,  66,
	/* T8 Object */
	   8,  10,
	  20,   0,   1,   1,   0,   0,   0,   0,   3,   0,
	/* T9 Object */
	   9,  36,
	 139,   0,   0,  19,  11,   0,  96,  35,   1,   5,
	  10,  25,   7,  80,   5,   5,  75,  96,  95,   3,
	 223,   1,  15,   2,  32,  27, 202,  50, 191,  50,
	  15,   5,   0,   0,   1,   0,
	/* T18 Object */
	/* T19 Object */
	  19,   6,
	   0,   0,   0,   0,   0,   0,
	/* T25 Object */
	  25,   7,
	   0,   0,0x30,0x75,0x20,0x4e, 200,
	/* T42 Object */
	  42,  10,
	   3,  20,  20,  12,   0,   0,   0,   0,   0,   0,
	/* T46 Object */
	  46,  10,
	   0,   0,  16,  32,   0,   0,   3,   0,   0,   1,
	/* T47 Object */
	  47,  26,
	   9,   1,   1, 254, 216,  20,   0,   0,   0,   0,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	   0,   0,   0,   0,   0,   0,
	/* T55 Object */
	  55,   6,
	   1,  10,  64,   0,  64,   0,
	/* T56 Object */
	  56,  43,
	   0,   0,   1,  41,  12,  12,  12,  12,  12,  11,
	  10,  12,  12,  11,  12,  12,  12,  12,  12,  12,
	  11,  12,   0,   0,   0,   0,   0,   0,   0,   0,
	   1,   2,  20,   4,   0,   0,   0,   0,   0,   0,
	   0,   0,   0,
	/* T57 Object */
	  57,   3,
	   0,   0,   0,
	/* T61 Object */
	  61,  10,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	/* T62 Object */
	  62,  54,
	   1,   3,   3,  18,   0,   0,  80,   0,   0,   0,
	   0,   0,   0,   0,   5,   0,  10,   5,   5,  96,
	  30,   5,  52,   2, 100,   6,   6,   4,  64,   0,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	   0,   0,   0,   0,
	/* T65 Object */
	  65,  17,
	   1,   5,   1,   0,   1,   0,   1,   0,   1,   0,
	 254,   0,   0,   0, 254,   0,   0,
	/* T66 Object */
	  66,   5,
	   0,  40,  50,   0,   0,
};

const struct mxt_config mxt_configs[] = {
	{
		mxt_config_data0,
		ARRAY_SIZE(mxt_config_data0),
	},
	{
		mxt_config_data1,
		ARRAY_SIZE(mxt_config_data1),
	},
};

static struct mxt_platform_data mxt_platform_data = {
	.config			= mxt_configs,
	.x_size			= MXT_PANEL_HEIGHT,
	.y_size			= MXT_PANEL_WIDTH,
	.irq_gpio		= MXT_TS_GPIO_IRQ,
	.reset_gpio		= MXT_TS_RESET_GPIO,
	.orient			= MXT_DIAGONAL,
	.irqflags		= IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.i2c_pull_up		= true,
	.init_hw		= mxt_init_hw,
	.reset_hw		= mxt_reset_hw,
	.shutdown		= mxt_shutdown,
};

#endif /* CONFIG_TOUCHSCREEN_ATMEL_MXT_KC */
#endif /* __ARCH_ARM_MACH_MSM_BOARD_MSM8960_TOUCH_H */
