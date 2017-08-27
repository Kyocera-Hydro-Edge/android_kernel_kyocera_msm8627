/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */
/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/vibrator.h>

#include "../staging/android/timed_output.h"

#include <mach/hs_io_ctl_a.h>
#include <linux/mfd/pm8xxx/mpp.h>
#include <linux/gpio.h>
#include <mach/kc_board.h>

#define VIB_DRV			0x4A

#define VIB_DRV_SEL_MASK	0xf8
#define VIB_DRV_SEL_SHIFT	0x03
#define VIB_DRV_EN_MANUAL_MASK	0xfc
#define VIB_DRV_LOGIC_SHIFT	0x2
#define TEST_ACCESS1		0x7F

#define VIB_MAX_LEVEL_mV	3100
#define VIB_MIN_LEVEL_mV	1200

#define VIB_DEV_DRIVE_mV        3100

#define GPIO_VIB_EN		81		/* GPIO_81 */

#define VIB_HAPTICS_ON  (1)
#define VIB_HAPTICS_OFF (0)

#define HAPTICS_TIME_10         10  /* 10ms */
#define HAPTICS_TIME_20         20  /* 20ms */
#define HAPTICS_TIME_30         30  /* 30ms */
#define HAPTICS_TIME_40         40  /* 40ms */
#define HAPTICS_TIME_50         50  /* 50ms */
#define HAPTICS_TIME_60         60  /* 60ms */
#define HAPTICS_TIME_70         70  /* 70ms */
#define HAPTICS_TIME_80         80  /* 80ms */
#define HAPTICS_TIME_90         90  /* 90ms */

static atomic_t haptics;

struct pm8xxx_vib {
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	spinlock_t lock;
	struct work_struct work;
	struct device *dev;
	const struct pm8xxx_vibrator_platform_data *pdata;
	int state;
	int level;
	u8  reg_vib_drv;
};

static struct pm8xxx_vib *vib_dev;

int pm8xxx_vibrator_config(struct pm8xxx_vib_config *vib_config)
{
	u8 reg = 0;
	int rc;

	if (vib_dev == NULL) {
		pr_err("%s: vib_dev is NULL\n", __func__);
		return -EINVAL;
	}

	if (vib_config->drive_mV) {
		if ((vib_config->drive_mV < VIB_MIN_LEVEL_mV) ||
			(vib_config->drive_mV > VIB_MAX_LEVEL_mV)) {
			pr_err("Invalid vibrator drive strength\n");
			return -EINVAL;
		}
	}

	reg = (vib_config->drive_mV / 100) << VIB_DRV_SEL_SHIFT;

	reg |= (!!vib_config->active_low) << VIB_DRV_LOGIC_SHIFT;

	reg |= vib_config->enable_mode;

	rc = pm8xxx_writeb(vib_dev->dev->parent, VIB_DRV, reg);
	if (rc)
		pr_err("%s: pm8xxx write failed: rc=%d\n", __func__, rc);

	return rc;
}
EXPORT_SYMBOL(pm8xxx_vibrator_config);

/* REVISIT: just for debugging, will be removed in final working version */
static void __dump_vib_regs(struct pm8xxx_vib *vib, char *msg)
{
	u8 temp;

	dev_dbg(vib->dev, "%s\n", msg);

	pm8xxx_readb(vib->dev->parent, VIB_DRV, &temp);
	dev_dbg(vib->dev, "VIB_DRV - %X\n", temp);
}

static int pm8xxx_vib_read_u8(struct pm8xxx_vib *vib,
				 u8 *data, u16 reg)
{
	int rc;

	rc = pm8xxx_readb(vib->dev->parent, reg, data);
	if (rc < 0)
		dev_warn(vib->dev, "Error reading pm8xxx: %X - ret %X\n",
				reg, rc);

	return rc;
}

static int pm8xxx_vib_write_u8(struct pm8xxx_vib *vib,
				 u8 data, u16 reg)
{
	int rc;

	rc = pm8xxx_writeb(vib->dev->parent, reg, data);
	if (rc < 0)
		dev_warn(vib->dev, "Error writing pm8xxx: %X - ret %X\n",
				reg, rc);
	return rc;
}

static int pm8xxx_vib_set(struct pm8xxx_vib *vib, int on)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	int rc;
	u8 val;

	if (on) {
		val = vib->reg_vib_drv;
		val |= ((vib->level << VIB_DRV_SEL_SHIFT) & VIB_DRV_SEL_MASK);
		rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
		if (rc < 0)
			return rc;
		vib->reg_vib_drv = val;
	} else {
		val = vib->reg_vib_drv;
		val &= ~VIB_DRV_SEL_MASK;
		rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
		if (rc < 0)
			return rc;
		vib->reg_vib_drv = val;
	}
	__dump_vib_regs(vib, "vib_set_end");

	return rc;
#else /* QUALCOMM_ORIGINAL_FEATURE */

	if (on) {
		gpio_set_value( GPIO_VIB_EN, 1 );
	} else {
		gpio_set_value( GPIO_VIB_EN, 0 );
	}

	__dump_vib_regs(vib, "vib_set_end");

	return 0;
#endif /* QUALCOMM_ORIGINAL_FEATURE */
}

static int pm8xxx_vib_haptics(int value)
{
	if (OEM_get_board() < OEM_BOARD_LAB_TYPE)
	{
		return value;
	}

	if (value > HAPTICS_TIME_10 &&
	    value <= HAPTICS_TIME_80)
	{
		atomic_set(&haptics, VIB_HAPTICS_ON);
	}
	else
	{
		return value;
	}

	if (value > HAPTICS_TIME_10 &&
	    value <= HAPTICS_TIME_40)
	{
		value = value + HAPTICS_TIME_30;
	}
	else if (value > HAPTICS_TIME_40 &&
	         value <= HAPTICS_TIME_60)
	{
		value = value + HAPTICS_TIME_20;
	}
	else if (value > HAPTICS_TIME_60 &&
	         value <= HAPTICS_TIME_80)
	{
		value = value + HAPTICS_TIME_10;
	}

	return value;
}

static void pm8xxx_vib_enable(struct timed_output_dev *dev, int value)
{
	struct pm8xxx_vib *vib = container_of(dev, struct pm8xxx_vib,
					 timed_dev);
	unsigned long flags;

	if (OEM_get_board() >= OEM_BOARD_LAB_TYPE)
	{
		if (atomic_read(&haptics) && !(value))
		{
			atomic_set(&haptics, VIB_HAPTICS_OFF);
			return;
		}
	}

retry:
	spin_lock_irqsave(&vib->lock, flags);
	if (hrtimer_try_to_cancel(&vib->vib_timer) < 0) {
		spin_unlock_irqrestore(&vib->lock, flags);
		cpu_relax();
		goto retry;
	}

	/* add haptics time */
	value = pm8xxx_vib_haptics(value);

	if (value == 0)
		vib->state = 0;
	else {
		value = (value > vib->pdata->max_timeout_ms ?
				 vib->pdata->max_timeout_ms : value);
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&vib->lock, flags);
	schedule_work(&vib->work);
}

static void pm8xxx_vib_update(struct work_struct *work)
{
	struct pm8xxx_vib *vib = container_of(work, struct pm8xxx_vib,
					 work);

	pm8xxx_vib_set(vib, vib->state);
}

static int pm8xxx_vib_get_time(struct timed_output_dev *dev)
{
	struct pm8xxx_vib *vib = container_of(dev, struct pm8xxx_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart pm8xxx_vib_timer_func(struct hrtimer *timer)
{
	struct pm8xxx_vib *vib = container_of(timer, struct pm8xxx_vib,
							 vib_timer);

	vib->state = 0;
	schedule_work(&vib->work);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_PM
static int pm8xxx_vib_suspend(struct device *dev)
{
	struct pm8xxx_vib *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	/* turn-off vibrator */
	pm8xxx_vib_set(vib, 0);

	return 0;
}

static const struct dev_pm_ops pm8xxx_vib_pm_ops = {
	.suspend = pm8xxx_vib_suspend,
};
#endif

static int __devinit pm8xxx_vib_probe(struct platform_device *pdev)

{
	const struct pm8xxx_vibrator_platform_data *pdata =
						pdev->dev.platform_data;
	struct pm8xxx_vib *vib;
	u8 val;
	int rc;

	struct pm8xxx_vib_config my_vib_config = {
		.drive_mV = 3000,
		.active_low = 0,
		.enable_mode = PM8XXX_VIB_DTEST2,
	};

	if (OEM_get_board() >= OEM_BOARD_LAB_TYPE)
	{
		my_vib_config.drive_mV = VIB_DEV_DRIVE_mV;
	}

	atomic_set(&haptics, VIB_HAPTICS_OFF);

	if (!pdata)
		return -EINVAL;

	if (pdata->level_mV < VIB_MIN_LEVEL_mV ||
			 pdata->level_mV > VIB_MAX_LEVEL_mV)
		return -EINVAL;

	vib = kzalloc(sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->pdata	= pdata;
	if (OEM_get_board() >= OEM_BOARD_LAB_TYPE)
	{
		vib->level	= VIB_DEV_DRIVE_mV / 100;
	}
	else
	{
		vib->level	= pdata->level_mV / 100;
	}
	vib->dev	= &pdev->dev;

	spin_lock_init(&vib->lock);
	INIT_WORK(&vib->work, pm8xxx_vib_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = pm8xxx_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = pm8xxx_vib_get_time;
	vib->timed_dev.enable = pm8xxx_vib_enable;

	__dump_vib_regs(vib, "boot_vib_default");

	rc = pm8xxx_vib_read_u8(vib, &val, TEST_ACCESS1);
	if (rc < 0)
		goto err_read_vib;

	val |= 0x02;

	rc = pm8xxx_vib_write_u8(vib, val, TEST_ACCESS1);
	if (rc < 0)
		goto err_read_vib;

	/*
	 * Configure the vibrator, it operates in manual mode
	 * for timed_output framework.
	 */
	rc = pm8xxx_vib_read_u8(vib, &val, VIB_DRV);
	if (rc < 0)
		goto err_read_vib;
	val &= ~VIB_DRV_EN_MANUAL_MASK;
	rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
	if (rc < 0)
		goto err_read_vib;

	vib->reg_vib_drv = val;

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		goto err_read_vib;

	pm8xxx_vib_enable(&vib->timed_dev, pdata->initial_vibrate_ms);

	platform_set_drvdata(pdev, vib);

	vib_dev = vib;

	rc = pm8xxx_vibrator_config(&my_vib_config);
	if (rc < 0)
	{
		goto err_read_vib;
	}

	return 0;

err_read_vib:
	kfree(vib);
	return rc;
}

static int __devexit pm8xxx_vib_remove(struct platform_device *pdev)
{
	struct pm8xxx_vib *vib = platform_get_drvdata(pdev);

	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);
	platform_set_drvdata(pdev, NULL);
	kfree(vib);

	return 0;
}

static struct platform_driver pm8xxx_vib_driver = {
	.probe		= pm8xxx_vib_probe,
	.remove		= __devexit_p(pm8xxx_vib_remove),
	.driver		= {
		.name	= PM8XXX_VIBRATOR_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm8xxx_vib_pm_ops,
#endif
	},
};

static int __init pm8xxx_vib_init(void)
{
	gpio_set_value( GPIO_VIB_EN, 0 );
	return platform_driver_register(&pm8xxx_vib_driver);
}
module_init(pm8xxx_vib_init);

static void __exit pm8xxx_vib_exit(void)
{
	platform_driver_unregister(&pm8xxx_vib_driver);
}
module_exit(pm8xxx_vib_exit);

MODULE_ALIAS("platform:" PM8XXX_VIBRATOR_DEV_NAME);
MODULE_DESCRIPTION("pm8xxx vibrator driver");
MODULE_LICENSE("GPL v2");
