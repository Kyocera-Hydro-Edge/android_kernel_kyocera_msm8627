/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
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
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/err.h>

#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/pwm.h>
#include <linux/leds-pm8xxx.h>

#include <linux/delay.h>

#include <mach/msm_smd.h>

#include "leds.h"

#define LEDS_USE_MODEM           1
#define LEDS_BRIGHT_CHG_ENABLE   0



#define LPG_CTL_7 0x14d
#define SSBI_REG_ADDR_DRV_KEYPAD	0x48
#define PM8XXX_DRV_KEYPAD_BL_MASK	0xf0
#define PM8XXX_DRV_KEYPAD_BL_SHIFT	0x04

#define SSBI_REG_ADDR_FLASH_DRV0        0x49
#define PM8XXX_DRV_FLASH_MASK           0xf0
#define PM8XXX_DRV_FLASH_SHIFT          0x04

#define SSBI_REG_ADDR_FLASH_DRV1        0xFB

#define SSBI_REG_ADDR_LED_CTRL_BASE	0x131
#define SSBI_REG_ADDR_LED_CTRL(n)	(SSBI_REG_ADDR_LED_CTRL_BASE + (n))
#define PM8XXX_DRV_LED_CTRL_MASK	0xf8
#define PM8XXX_DRV_LED_CTRL_SHIFT	0x03

#define SSBI_REG_ADDR_WLED_CTRL_BASE	0x25A
#define SSBI_REG_ADDR_WLED_CTRL(n)	(SSBI_REG_ADDR_WLED_CTRL_BASE + (n) - 1)

/* wled control registers */
#define WLED_MOD_CTRL_REG		SSBI_REG_ADDR_WLED_CTRL(1)
#define WLED_MAX_CURR_CFG_REG(n)	SSBI_REG_ADDR_WLED_CTRL(n + 2)
#define WLED_BRIGHTNESS_CNTL_REG1(n)	SSBI_REG_ADDR_WLED_CTRL((2 * n) + 5)
#define WLED_BRIGHTNESS_CNTL_REG2(n)	SSBI_REG_ADDR_WLED_CTRL((2 * n) + 6)
#define WLED_SYNC_REG			SSBI_REG_ADDR_WLED_CTRL(11)
#define WLED_OVP_CFG_REG		SSBI_REG_ADDR_WLED_CTRL(13)
#define WLED_BOOST_CFG_REG		SSBI_REG_ADDR_WLED_CTRL(14)
#define WLED_HIGH_POLE_CAP_REG		SSBI_REG_ADDR_WLED_CTRL(16)

#define WLED_STRINGS			0x03
#define WLED_OVP_VAL_MASK		0x30
#define WLED_OVP_VAL_BIT_SHFT		0x04
#define WLED_BOOST_LIMIT_MASK		0xE0
#define WLED_BOOST_LIMIT_BIT_SHFT	0x05
#define WLED_BOOST_OFF			0x00
#define WLED_EN_MASK			0x01
#define WLED_CP_SELECT_MAX		0x03
#define WLED_CP_SELECT_MASK		0x03
#define WLED_DIG_MOD_GEN_MASK		0x70
#define WLED_CS_OUT_MASK		0x02//0x0E
#define WLED_CTL_DLY_STEP		200
#define WLED_CTL_DLY_MAX		1400
#define WLED_CTL_DLY_MASK		0xE0
#define WLED_CTL_DLY_BIT_SHFT		0x05
#define WLED_MAX_CURR			25
#define WLED_MAX_CURR_MASK		0x1F
#define WLED_OP_FDBCK_MASK		0x1C
#define WLED_OP_FDBCK_BIT_SHFT		0x02

#define WLED_MAX_LEVEL			255
#define WLED_8_BIT_MASK			0xFF
#define WLED_8_BIT_SHFT			0x08
#define WLED_MAX_DUTY_CYCLE		0xFFF

#define WLED_SYNC_VAL			0x07
#define WLED_SYNC_RESET_VAL		0x00
#define WLED_SYNC_MASK			0xF8

#define ONE_WLED_STRING			1
#define TWO_WLED_STRINGS		2
#define THREE_WLED_STRINGS		3

#define WLED_CABC_ONE_STRING		0x01
#define WLED_CABC_TWO_STRING		0x03
#define WLED_CABC_THREE_STRING		0x07

#define WLED_CABC_SHIFT			3

#define SSBI_REG_ADDR_RGB_CNTL1		0x12D
#define SSBI_REG_ADDR_RGB_CNTL2		0x12E

#define PM8XXX_DRV_RGB_RED_LED		BIT(2)
#define PM8XXX_DRV_RGB_GREEN_LED	BIT(1)
#define PM8XXX_DRV_RGB_BLUE_LED		BIT(0)

#define MAX_FLASH_LED_CURRENT		300
#define MAX_LC_LED_CURRENT		40
#define MAX_KP_BL_LED_CURRENT		300

#define PM8XXX_ID_LED_CURRENT_FACTOR	2  /* Iout = x * 2mA */
#define PM8XXX_ID_FLASH_CURRENT_FACTOR	20 /* Iout = x * 20mA */

#define PM8XXX_FLASH_MODE_DBUS1		1
#define PM8XXX_FLASH_MODE_DBUS2		2
#define PM8XXX_FLASH_MODE_PWM		3

#define MAX_LC_LED_BRIGHTNESS		20
#define MAX_FLASH_BRIGHTNESS		15
#define MAX_KB_LED_BRIGHTNESS		255
#define MAX_RGB_LED_BRIGHTNESS		255
#define MAX_RGB_LED_OFFCHARGING		43
#define MAX_FLASH_LED_BRIGHTNESS	20
#define MAX_WLED_LED_BRIGHTNESS		255
#define WLED_MOD_LEVEL 60


#define PM8XXX_LED_OFFSET(id) ((id) - PM8XXX_ID_LED_0)

#define PM8XXX_LED_PWM_FLAGS	(PM_PWM_LUT_LOOP | PM_PWM_LUT_RAMP_UP)

#ifdef QUALCOMM_ORIGINAL_FEATURE
#define LED_MAP(_version, _kb, _led0, _led1, _led2, _flash_led0, _flash_led1, \
	_wled, _rgb_led_red, _rgb_led_green, _rgb_led_blue)\
	{\
		.version = _version,\
		.supported = _kb << PM8XXX_ID_LED_KB_LIGHT | \
			_led0 << PM8XXX_ID_LED_0 | _led1 << PM8XXX_ID_LED_1 | \
			_led2 << PM8XXX_ID_LED_2  | \
			_flash_led0 << PM8XXX_ID_FLASH_LED_0 | \
			_flash_led1 << PM8XXX_ID_FLASH_LED_1 | \
			_wled << PM8XXX_ID_WLED | \
			_rgb_led_red << PM8XXX_ID_RGB_LED_RED | \
			_rgb_led_green << PM8XXX_ID_RGB_LED_GREEN | \
			_rgb_led_blue << PM8XXX_ID_RGB_LED_BLUE, \
	}

/**
 * supported_leds - leds supported for each PMIC version
 * @version - version of PMIC
 * @supported - which leds are supported on version
 */

struct supported_leds {
	enum pm8xxx_version version;
	u32 supported;
};

static const struct supported_leds led_map[] = {
	LED_MAP(PM8XXX_VERSION_8058, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0),
	LED_MAP(PM8XXX_VERSION_8921, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0),
	LED_MAP(PM8XXX_VERSION_8018, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
	LED_MAP(PM8XXX_VERSION_8922, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1),
	LED_MAP(PM8XXX_VERSION_8038, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1),
};
#endif /* QUALCOMM_ORIGINAL_FEATURE */

#if LEDS_USE_MODEM
static struct smd_channel *smdch = NULL;
static bool   smd_opend = false;
#define	LED_SMD_RETRY_COUNT	30			/* times */
#define	LED_SMD_RETRY_DELAY	20			/* ms */
#define LED_SMD_PORT_NAME	"apr_ledex_svc"

#define	LED_CTRL_QUEUE_NUM	10
static struct led_control_ex_data *led_ctrl_queue = NULL;
static int led_queue_head = 0;
static int led_queue_tail= 0;
static struct mutex led_queue_lock;

#define	LED_QUEUE_RETRY_DELAY	2000	/* ms */
static void led_queue_timer_cb(struct work_struct *);
static DECLARE_DELAYED_WORK(queue_timer, led_queue_timer_cb);

#define	LED_SEND_DEALY			20		/* ms */

#endif /* LEDS_USE_MODEM */

#define MAX_PWM_DUTY_PCTS		PM_PWM_LUT_SIZE
#define MIN_PWM_DUTY_MS			50
static int *pwm_duty_pcts = NULL;
static struct pm8xxx_pwm_duty_cycles led_pwm_duty_cycles;
struct led_control_ex_data		led_ctrl_data[LED_PRIORITY_NUMBER];

#define	LED_BLINK_DELAY			2000				/* ms */

static bool offcharging = false;

/**
 * struct pm8xxx_led_data - internal led data structure
 * @led_classdev - led class device
 * @id - led index
 * @work - workqueue for led
 * @lock - to protect the transactions
 * @reg - cached value of led register
 * @pwm_dev - pointer to PWM device if LED is driven using PWM
 * @pwm_channel - PWM channel ID
 * @pwm_period_us - PWM period in micro seconds
 * @pwm_duty_cycles - struct that describes PWM duty cycles info
 */
struct pm8xxx_led_data {
	struct led_classdev	cdev;
	int			id;
	u8			reg;
	u8			wled_mod_ctrl_val;
	struct device		*dev;
	struct work_struct	work;
	struct mutex		lock;
	struct pwm_device	*pwm_dev;
	int			pwm_channel;
	u32			pwm_period_us;
	struct pm8xxx_pwm_duty_cycles *pwm_duty_cycles;
	struct wled_config_data *wled_cfg;
	int			max_current;
};




#if LEDS_USE_MODEM
static int led_queue_init(void)
{
	mutex_init(&led_queue_lock);

	led_queue_head = 0;
	led_queue_tail = 0;
	led_ctrl_queue = kcalloc(LED_CTRL_QUEUE_NUM, sizeof(struct led_control_ex_data), GFP_KERNEL);

	if (!led_ctrl_queue)
		return -ENOMEM;

	return 0;
}

static int led_queue_add(struct led_control_ex_data *led_ctrl)
{
	int rc = 0;
	int led_next_queue;

	if (!led_ctrl_queue)
		return -ENOMEM;

	mutex_lock(&led_queue_lock);

	led_next_queue = (led_queue_head + 1) % LED_CTRL_QUEUE_NUM;

	if (led_next_queue == led_queue_tail)
	{
		pr_err("LED: out of queue\n");
		rc = -ENOMEM;
	}
	else
	{
		led_ctrl_queue[led_queue_head] = *led_ctrl;
		led_queue_head = led_next_queue;
	}

	mutex_unlock(&led_queue_lock);

	return rc;
}

static struct led_control_ex_data *led_queue_fetch(void)
{
	int led_next_queue;
	struct led_control_ex_data *result;

	if (!led_ctrl_queue)
		return NULL;

	mutex_lock(&led_queue_lock);

	if (led_queue_head == led_queue_tail)
	{
		result = NULL;
	}
	else
	{
		led_next_queue = (led_queue_tail + 1) % LED_CTRL_QUEUE_NUM;
		result = &led_ctrl_queue[led_queue_tail];
		led_queue_tail = led_next_queue;
	}

	mutex_unlock(&led_queue_lock);

	return result;
}
#endif /* LEDS_USE_MODEM */

static struct pm8xxx_led_data *pm8xxx_get_led_data_from_id(int led_id)
{
	struct list_head *p;
	struct led_classdev *led_cdev;
	struct pm8xxx_led_data *led;

	list_for_each(p, &leds_list)
	{
		led_cdev = list_entry(p, struct led_classdev, node);
		if (led_cdev)
		{
			led = container_of(led_cdev, struct pm8xxx_led_data, cdev);
			if (led)
			{
				if (led->id == led_id)
				{
					return led;
				}
			}
		}
	}

	return (struct pm8xxx_led_data *)NULL;
}

#ifdef QUALCOMM_ORIGINAL_FEATURE
static void led_kp_set(struct pm8xxx_led_data *led, enum led_brightness value)
{
	int rc;
	u8 level;

	level = (value << PM8XXX_DRV_KEYPAD_BL_SHIFT) &
				 PM8XXX_DRV_KEYPAD_BL_MASK;

	led->reg &= ~PM8XXX_DRV_KEYPAD_BL_MASK;
	led->reg |= level;

	rc = pm8xxx_writeb(led->dev->parent, SSBI_REG_ADDR_DRV_KEYPAD,
								led->reg);
	if (rc < 0)
		dev_err(led->cdev.dev,
			"can't set keypad backlight level rc=%d\n", rc);
}

static void led_lc_set(struct pm8xxx_led_data *led, enum led_brightness value)
{
	int rc, offset;
	u8 level;

	level = (value << PM8XXX_DRV_LED_CTRL_SHIFT) &
				PM8XXX_DRV_LED_CTRL_MASK;

	offset = PM8XXX_LED_OFFSET(led->id);

	led->reg &= ~PM8XXX_DRV_LED_CTRL_MASK;
	led->reg |= level;

	rc = pm8xxx_writeb(led->dev->parent, SSBI_REG_ADDR_LED_CTRL(offset),
								led->reg);
	if (rc)
		dev_err(led->cdev.dev, "can't set (%d) led value rc=%d\n",
				led->id, rc);
}
#endif /* QUALCOMM_ORIGINAL_FEATURE */

static void
led_flash_set(struct pm8xxx_led_data *led, enum led_brightness value)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	int rc;
	u8 level;
	u16 reg_addr;

	level = (value << PM8XXX_DRV_FLASH_SHIFT) &
				 PM8XXX_DRV_FLASH_MASK;

	led->reg &= ~PM8XXX_DRV_FLASH_MASK;
	led->reg |= level;

	if (led->id == PM8XXX_ID_FLASH_LED_0)
		reg_addr = SSBI_REG_ADDR_FLASH_DRV0;
	else
		reg_addr = SSBI_REG_ADDR_FLASH_DRV1;

	rc = pm8xxx_writeb(led->dev->parent, reg_addr, led->reg);
	if (rc < 0)
		dev_err(led->cdev.dev, "can't set flash led%d level rc=%d\n",
			 led->id, rc);
#else /* QUALCOMM_ORIGINAL_FEATURE */


#endif /* QUALCOMM_ORIGINAL_FEATURE */
}

extern int dev_lcdbl_hw_pwm_calc( int and );
struct pm8xxx_led_data *oem_save_wled = NULL;
extern struct mutex mipi_novatek_wvga_lcd_disp;
extern int lcd_bl_on_flg;
static int old_level = 0;
extern int bl_que_flag;

static int pm8xxx_led_set_pwm(struct led_control_ex_data *, bool);


static int
led_wled_set(struct pm8xxx_led_data *led, enum led_brightness value)
{
	int rc, duty;
	u8 val, i, num_wled_strings;
	
	if (value > WLED_MAX_LEVEL)
		value = WLED_MAX_LEVEL;

	oem_save_wled = led;
	old_level = value;
	value = dev_lcdbl_hw_pwm_calc(value);
	mutex_lock(&mipi_novatek_wvga_lcd_disp);
    if(value == 0){
    }else if(!bl_que_flag){
		printk("[DEV LCD]%s() Disp OFF!!!!! \n", __func__);
		mutex_unlock(&mipi_novatek_wvga_lcd_disp);
		return 0;
	}

	if (value == 0) {
		rc = pm8xxx_writeb(led->dev->parent, WLED_MOD_CTRL_REG,
				WLED_BOOST_OFF);
		if (rc) {
			dev_err(led->dev->parent, "can't write wled ctrl config"
				" register rc=%d\n", rc);
			mutex_unlock(&mipi_novatek_wvga_lcd_disp);
			return rc;
		}
	} else {
		rc = pm8xxx_writeb(led->dev->parent, WLED_MOD_CTRL_REG,
				led->wled_mod_ctrl_val);
		if (rc) {
			dev_err(led->dev->parent, "can't write wled ctrl config"
				" register rc=%d\n", rc);
			mutex_unlock(&mipi_novatek_wvga_lcd_disp);
			return rc;
		}
	}

	duty = (WLED_MAX_DUTY_CYCLE * value) / WLED_MAX_LEVEL;

	num_wled_strings = led->wled_cfg->num_strings;

	/* program brightness control registers */
	for (i = 0; i < num_wled_strings; i++) {
		rc = pm8xxx_readb(led->dev->parent,
				WLED_BRIGHTNESS_CNTL_REG1(i), &val);
		if (rc) {
			dev_err(led->dev->parent, "can't read wled brightnes ctrl"
				" register1 rc=%d\n", rc);
			mutex_unlock(&mipi_novatek_wvga_lcd_disp);
			return rc;
		}

		val = (val & ~WLED_MAX_CURR_MASK) | (duty >> WLED_8_BIT_SHFT);
		rc = pm8xxx_writeb(led->dev->parent,
				WLED_BRIGHTNESS_CNTL_REG1(i), val);
		if (rc) {
			dev_err(led->dev->parent, "can't write wled brightness ctrl"
				" register1 rc=%d\n", rc);
			mutex_unlock(&mipi_novatek_wvga_lcd_disp);
			return rc;
		}

		val = duty & WLED_8_BIT_MASK;
		rc = pm8xxx_writeb(led->dev->parent,
				WLED_BRIGHTNESS_CNTL_REG2(i), val);
		if (rc) {
			dev_err(led->dev->parent, "can't write wled brightness ctrl"
				" register2 rc=%d\n", rc);
			mutex_unlock(&mipi_novatek_wvga_lcd_disp);
			return rc;
		}
	}
	rc = pm8xxx_readb(led->dev->parent, WLED_SYNC_REG, &val);
	if (rc) {
		dev_err(led->dev->parent,
			"can't read wled sync register rc=%d\n", rc);
		mutex_unlock(&mipi_novatek_wvga_lcd_disp);
		return rc;
	}
	/* sync */
	val &= WLED_SYNC_MASK;
	val |= WLED_SYNC_VAL;
	rc = pm8xxx_writeb(led->dev->parent, WLED_SYNC_REG, val);
	if (rc) {
		dev_err(led->dev->parent,
			"can't read wled sync register rc=%d\n", rc);
		mutex_unlock(&mipi_novatek_wvga_lcd_disp);
		return rc;
	}
	val &= WLED_SYNC_MASK;
	val |= WLED_SYNC_RESET_VAL;
	rc = pm8xxx_writeb(led->dev->parent, WLED_SYNC_REG, val);
	if (rc) {
		dev_err(led->dev->parent,
			"can't read wled sync register rc=%d\n", rc);
		mutex_unlock(&mipi_novatek_wvga_lcd_disp);
		return rc;
	}
	mutex_unlock(&mipi_novatek_wvga_lcd_disp);
	return 0;
}

void oem_lcd_backlight_ctrl( int value )
{
  if(oem_save_wled != NULL){
    printk("%s(%d) \n", __func__, value);
    led_wled_set(oem_save_wled, (enum led_brightness)value);
  }
}

static void wled_dump_regs(struct pm8xxx_led_data *led)
{
	int i;
	u8 val;

	for (i = 1; i < 17; i++) {
		pm8xxx_readb(led->dev->parent,
				SSBI_REG_ADDR_WLED_CTRL(i), &val);
		pr_debug("WLED_CTRL_%d = 0x%x\n", i, val);
	}
}

static void
led_rgb_write(struct pm8xxx_led_data *led, u16 addr, enum led_brightness value)
{
	int rc;
	u8 val, mask;

	if (led->id != PM8XXX_ID_RGB_LED_BLUE &&
		led->id != PM8XXX_ID_RGB_LED_RED &&
		led->id != PM8XXX_ID_RGB_LED_GREEN)
		return;

	rc = pm8xxx_readb(led->dev->parent, addr, &val);
	if (rc) {
		dev_err(led->cdev.dev, "can't read rgb ctrl register rc=%d\n",
							rc);
		return;
	}

	switch (led->id) {
	case PM8XXX_ID_RGB_LED_RED:
		mask = PM8XXX_DRV_RGB_RED_LED;
		break;
	case PM8XXX_ID_RGB_LED_GREEN:
		mask = PM8XXX_DRV_RGB_GREEN_LED;
		break;
	case PM8XXX_ID_RGB_LED_BLUE:
		mask = PM8XXX_DRV_RGB_BLUE_LED;
		break;
	default:
		return;
	}

	if (value)
		val |= mask;
	else
		val &= ~mask;

	rc = pm8xxx_writeb(led->dev->parent, addr, val);
	if (rc < 0)
		dev_err(led->cdev.dev, "can't set rgb led %d level rc=%d\n",
			 led->id, rc);
}

#ifdef QUALCOMM_ORIGINAL_FEATURE
static void
led_rgb_set(struct pm8xxx_led_data *led, enum led_brightness value)
{
	if (value) {
		led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL1, value);
		led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL2, value);
	} else {
		led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL2, value);
		led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL1, value);
	}
}
#endif /* QUALCOMM_ORIGINAL_FEATURE */


static int pm8xxx_led_pwm_work(struct pm8xxx_led_data *led)
{
	int duty_us;
	int rc = 0;

	if (led->pwm_duty_cycles == NULL) {
		duty_us = (led->pwm_period_us * led->cdev.brightness) /
								LED_FULL;
		rc = pwm_config(led->pwm_dev, duty_us, led->pwm_period_us);

#ifdef QUALCOMM_ORIGINAL_FEATURE
		if (led->cdev.brightness) {
			led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL1,
				led->cdev.brightness);
			rc = pwm_enable(led->pwm_dev);
		} else {
			pwm_disable(led->pwm_dev);
			led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL1,
				led->cdev.brightness);
		}
#else /* QUALCOMM_ORIGINAL_FEATURE */
		/* always use pwm_enable() */
		led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL1,
			led->cdev.brightness);
		rc = pwm_enable(led->pwm_dev);
#endif /* QUALCOMM_ORIGINAL_FEATURE */
	} else {
		if (led->cdev.brightness)
			led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL1,
				led->cdev.brightness);
		rc = pm8xxx_pwm_lut_enable(led->pwm_dev, led->cdev.brightness);
		if (!led->cdev.brightness)
			led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL1,
				led->cdev.brightness);
	}

	pm8xxx_writeb(led->dev->parent, LPG_CTL_7, 0x0);
	return rc;
}


#if LEDS_USE_MODEM
static void pm8xxx_led_control_modem_notify(void *priv, unsigned event)
{
	switch (event)
	{
		case SMD_EVENT_DATA:
			break;
		case SMD_EVENT_OPEN:
			smd_opend = true;
			break;
		case SMD_EVENT_CLOSE:
			break;
		case SMD_EVENT_STATUS:
			break;
		case SMD_EVENT_REOPEN_READY:
			break;
	}
}

static void pm8xxx_led_control_modem_send(void)
{
	int rc;
	int count;
	int curr_prio;
	int i;

	struct led_control_ex_data *led_ctrl_queued;
	struct led_control_data    led_send_data;

	if (!smd_opend || !smdch)
	{
		smd_opend = false;
		rc = smd_open(LED_SMD_PORT_NAME, &smdch, NULL, pm8xxx_led_control_modem_notify);
		if (rc < 0)
		{
			smdch = NULL;
			pr_err("smd_open() open error (retry after %d ms)\n", LED_QUEUE_RETRY_DELAY);
		}
		else
		{
			for (count = 0; count < LED_SMD_RETRY_COUNT; ++count)
			{
				if (smd_opend)
					break;
				mdelay(LED_SMD_RETRY_DELAY);
			}
			if (count >= LED_SMD_RETRY_COUNT)
			{
				smd_close(smdch);
				smdch = NULL;
				pr_err("smd_open() internal error (retry after %d ms)\n", LED_QUEUE_RETRY_DELAY);
			}
		}

		if (!smdch)
		{
			schedule_delayed_work(&queue_timer, msecs_to_jiffies(LED_QUEUE_RETRY_DELAY));
			return;
		}
	}

	while (true)
	{
		led_ctrl_queued = led_queue_fetch();
		if (!led_ctrl_queued)
			break;

		curr_prio = pm8xxx_led_set_pwm(led_ctrl_queued, true);		/* start first bright, no blink */

		if (curr_prio < LED_PRIORITY_NUMBER)						/* LED status changed */
		{
			led_send_data.color    = led_ctrl_data[curr_prio].color;
			led_send_data.mode     = led_ctrl_data[curr_prio].mode;
			for (i = 0; i < LED_CTRL_MAX_SEQUENCE; ++i)
				led_send_data.pattern[i] = led_ctrl_data[curr_prio].pattern[i];

			mdelay(LED_SEND_DEALY);

			rc = smd_write(smdch, &led_send_data, sizeof (struct led_control_data));
		}
	}
}
#endif /* LEDS_USE_MODEM */

static void __pm8xxx_brightness_fix(struct pm8xxx_led_data *led)
{
	if (led->cdev.brightness)		/* if ON then MAX brightness */
	{
		led->cdev.brightness = led->cdev.max_brightness;
	}
}

static void pm8xxx_set_led_offcharging(struct led_control_ex_data *);

static void __pm8xxx_led_work(struct pm8xxx_led_data *led,
					enum led_brightness level)
{
	int rc;
	int mod_level = 0;
	struct led_control_ex_data led_ctrl;
	int index;

#if LEDS_USE_MODEM
	cancel_delayed_work_sync(&queue_timer);
#endif /* LEDS_USE_MODEM */

#if LEDS_BRIGHT_CHG_ENABLE
	return;
#endif /* LEDS_BRIGHT_CHG_ENABLE */

#ifdef QUALCOMM_ORIGINAL_FEATURE
	mutex_lock(&led->lock);

	switch (led->id) {
	case PM8XXX_ID_LED_KB_LIGHT:
		led_kp_set(led, level);
		break;
	case PM8XXX_ID_LED_0:
	case PM8XXX_ID_LED_1:
	case PM8XXX_ID_LED_2:
		led_lc_set(led, level);
		break;
	case PM8XXX_ID_FLASH_LED_0:
	case PM8XXX_ID_FLASH_LED_1:
		led_flash_set(led, level);
		break;
	case PM8XXX_ID_WLED:
		rc = led_wled_set(led, level);
		if (rc < 0)
			pr_err("wled brightness set failed %d\n", rc);
		break;
	case PM8XXX_ID_RGB_LED_RED:
	case PM8XXX_ID_RGB_LED_GREEN:
	case PM8XXX_ID_RGB_LED_BLUE:
		led_rgb_set(led, level);
		break;

	case PM8XXX_ID_RGB_LED_AMBER:
		break;

	default:
		dev_err(led->cdev.dev, "unknown led id %d", led->id);
		break;
	}

	mutex_unlock(&led->lock);
#else /* QUALCOMM_ORIGINAL_FEATURE */
	switch (led->id)
	{
		case PM8XXX_ID_WLED:
			if(old_level < (int)level){
				while(WLED_MOD_LEVEL < ((int)level-old_level)){
					mod_level = old_level + WLED_MOD_LEVEL;
					rc = led_wled_set(led, mod_level);
					if (rc < 0)
						pr_err("wled brightness set failed %d\n", rc);
					msleep(10);
				}
			}
			rc = led_wled_set(led, level);
			if (rc < 0)
				pr_err("wled brightness set failed %d\n", rc);
			break;
		case PM8XXX_ID_FLASH_LED_0:
			led_flash_set(led, level);
			break;

		case PM8XXX_ID_RGB_LED_RED:					/* RED LED */
			__pm8xxx_brightness_fix(led);
			if (level)
				led_ctrl.color = LED_COLOR_RED;
			else
				led_ctrl.color = LED_COLOR_OFF;
			led_ctrl.priority  = LED_PRIORITY_NOTIFICATION;
			led_ctrl.mode      = LED_BLINK_OFF;
			for (index = 0; index < LED_CTRL_MAX_SEQUENCE; ++index)
				led_ctrl.pattern[index] = 0;
#if LEDS_USE_MODEM
			if (offcharging)
			{
				pm8xxx_set_led_offcharging(&led_ctrl);
			}
			else
			{
				mutex_lock(&led->lock);
				led_queue_add(&led_ctrl);
				pm8xxx_led_control_modem_send();
				mutex_unlock(&led->lock);
			}
#else /* LEDS_USE_MODEM */
			pm8xxx_led_set_pwm(&led_ctrl, false);
#endif /* LEDS_USE_MODEM */
			break;

		case PM8XXX_ID_RGB_LED_GREEN:				/* GREEN LED */
			__pm8xxx_brightness_fix(led);
			if (level)
				led_ctrl.color = LED_COLOR_GREEN;
			else
				led_ctrl.color = LED_COLOR_OFF;
			led_ctrl.priority  = LED_PRIORITY_NOTIFICATION;
			led_ctrl.mode      = LED_BLINK_OFF;
			for (index = 0; index < LED_CTRL_MAX_SEQUENCE; ++index)
				led_ctrl.pattern[index] = 0;
#if LEDS_USE_MODEM
			if (offcharging)
			{
				pm8xxx_set_led_offcharging(&led_ctrl);
			}
			else
			{
				mutex_lock(&led->lock);
				led_queue_add(&led_ctrl);
				pm8xxx_led_control_modem_send();
				mutex_unlock(&led->lock);
			}
#else /* LEDS_USE_MODEM */
			pm8xxx_led_set_pwm(&led_ctrl, false);
#endif /* LEDS_USE_MODEM */
			break;

		case PM8XXX_ID_RGB_LED_AMBER:				/* RED+GREEN internal control */
			__pm8xxx_brightness_fix(led);
			if (level)
				led_ctrl.color = LED_COLOR_AMBER;
			else
				led_ctrl.color = LED_COLOR_OFF;
			led_ctrl.priority  = LED_PRIORITY_NOTIFICATION;
			led_ctrl.mode      = LED_BLINK_OFF;
			for (index = 0; index < LED_CTRL_MAX_SEQUENCE; ++index)
				led_ctrl.pattern[index] = 0;
#if LEDS_USE_MODEM
			if (offcharging)
			{
				pm8xxx_set_led_offcharging(&led_ctrl);
			}
			else
			{
				mutex_lock(&led->lock);
				led_queue_add(&led_ctrl);
				pm8xxx_led_control_modem_send();
				mutex_unlock(&led->lock);
			}
#else /* LEDS_USE_MODEM */
			pm8xxx_led_set_pwm(&led_ctrl, false);
#endif /* LEDS_USE_MODEM */
			break;

		case PM8XXX_ID_RGB_LED_BLUE:				/* Button-backlight (key LED) */
			__pm8xxx_brightness_fix(led);
			pm8xxx_led_pwm_work(led);
			break;

		default:
			dev_err(led->cdev.dev, "unknown led id %d", led->id);
			break;
	}
#endif /* QUALCOMM_ORIGINAL_FEATURE */
}

static void pm8xxx_led_work(struct work_struct *work)
{
	struct pm8xxx_led_data *led = container_of(work,
					 struct pm8xxx_led_data, work);

#if LEDS_BRIGHT_CHG_ENABLE
	pm8xxx_led_pwm_work(led);
#endif /* LEDS_BRIGHT_CHG_ENABLE */
	__pm8xxx_led_work(led, led->cdev.brightness);
}

static void pm8xxx_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct	pm8xxx_led_data *led;

	led = container_of(led_cdev, struct pm8xxx_led_data, cdev);

	if (value < LED_OFF || value > led->cdev.max_brightness) {
		dev_err(led->cdev.dev, "Invalid brightness value exceeds");
		return;
	}

	led->cdev.brightness = value;
	schedule_work(&led->work);		/* call pm8xxx_led_work() */
}

static int pm8xxx_set_led_mode_and_max_brightness(struct pm8xxx_led_data *led,
		enum pm8xxx_led_modes led_mode, int max_current)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	switch (led->id) {
	case PM8XXX_ID_LED_0:
	case PM8XXX_ID_LED_1:
	case PM8XXX_ID_LED_2:
		led->cdev.max_brightness = max_current /
						PM8XXX_ID_LED_CURRENT_FACTOR;
		if (led->cdev.max_brightness > MAX_LC_LED_BRIGHTNESS)
			led->cdev.max_brightness = MAX_LC_LED_BRIGHTNESS;
		led->reg = led_mode;
		break;
	case PM8XXX_ID_LED_KB_LIGHT:
	case PM8XXX_ID_FLASH_LED_0:
	case PM8XXX_ID_FLASH_LED_1:
		led->cdev.max_brightness = max_current /
						PM8XXX_ID_FLASH_CURRENT_FACTOR;
		if (led->cdev.max_brightness > MAX_FLASH_BRIGHTNESS)
			led->cdev.max_brightness = MAX_FLASH_BRIGHTNESS;

		switch (led_mode) {
		case PM8XXX_LED_MODE_PWM1:
		case PM8XXX_LED_MODE_PWM2:
		case PM8XXX_LED_MODE_PWM3:
			led->reg = PM8XXX_FLASH_MODE_PWM;
			break;
		case PM8XXX_LED_MODE_DTEST1:
			led->reg = PM8XXX_FLASH_MODE_DBUS1;
			break;
		case PM8XXX_LED_MODE_DTEST2:
			led->reg = PM8XXX_FLASH_MODE_DBUS2;
			break;
		default:
			led->reg = PM8XXX_LED_MODE_MANUAL;
			break;
		}
		break;
	case PM8XXX_ID_WLED:
		led->cdev.max_brightness = WLED_MAX_LEVEL;
		break;
	case PM8XXX_ID_RGB_LED_RED:
	case PM8XXX_ID_RGB_LED_GREEN:
	case PM8XXX_ID_RGB_LED_BLUE:
		led->cdev.max_brightness = LED_FULL;
		break;
	default:
		dev_err(led->cdev.dev, "LED Id is invalid");
		return -EINVAL;
	}
#else /* QUALCOMM_ORIGINAL_FEATURE */
	switch (led->id)
	{
		case PM8XXX_ID_RGB_LED_RED:
		case PM8XXX_ID_RGB_LED_GREEN:
		case PM8XXX_ID_RGB_LED_AMBER:
			led->cdev.max_brightness = max_current;
			if (led->cdev.max_brightness > MAX_RGB_LED_BRIGHTNESS)
				led->cdev.max_brightness = MAX_RGB_LED_BRIGHTNESS;
			break;
		case PM8XXX_ID_RGB_LED_BLUE:
			led->cdev.max_brightness = max_current;
			if (led->cdev.max_brightness > MAX_KB_LED_BRIGHTNESS)
				led->cdev.max_brightness = MAX_KB_LED_BRIGHTNESS;
			break;
		case PM8XXX_ID_FLASH_LED_0:
			led->cdev.max_brightness = max_current /
							PM8XXX_ID_FLASH_CURRENT_FACTOR;
			if (led->cdev.max_brightness > MAX_FLASH_BRIGHTNESS)
				led->cdev.max_brightness = MAX_FLASH_BRIGHTNESS;
			break;
		case PM8XXX_ID_WLED:
			led->cdev.max_brightness = WLED_MAX_LEVEL;
			break;
		default:
			dev_err(led->cdev.dev, "LED ID=%d is invalid", led->id);
			return -EINVAL;
	}
#endif /* QUALCOMM_ORIGINAL_FEATURE */
	return 0;
}

static enum led_brightness pm8xxx_led_get(struct led_classdev *led_cdev)
{
	struct pm8xxx_led_data *led;

	led = container_of(led_cdev, struct pm8xxx_led_data, cdev);

	return led->cdev.brightness;
}

static int __devinit init_wled(struct pm8xxx_led_data *led)
{
	int rc, i;
	u8 val, num_wled_strings;

	num_wled_strings = led->wled_cfg->num_strings;

	/* program over voltage protection threshold */
	if (led->wled_cfg->ovp_val > WLED_OVP_37V) {
		dev_err(led->dev->parent, "Invalid ovp value");
		return -EINVAL;
	}

	rc = pm8xxx_readb(led->dev->parent, WLED_OVP_CFG_REG, &val);
	if (rc) {
		dev_err(led->dev->parent, "can't read wled ovp config"
			" register rc=%d\n", rc);
		return rc;
	}

	val = (val & ~WLED_OVP_VAL_MASK) |
		(led->wled_cfg->ovp_val << WLED_OVP_VAL_BIT_SHFT);

	rc = pm8xxx_writeb(led->dev->parent, WLED_OVP_CFG_REG, val);
	if (rc) {
		dev_err(led->dev->parent, "can't write wled ovp config"
			" register rc=%d\n", rc);
		return rc;
	}

	/* program current boost limit and output feedback*/
	if (led->wled_cfg->boost_curr_lim > WLED_CURR_LIMIT_1680mA) {
		dev_err(led->dev->parent, "Invalid boost current limit");
		return -EINVAL;
	}

	rc = pm8xxx_readb(led->dev->parent, WLED_BOOST_CFG_REG, &val);
	if (rc) {
		dev_err(led->dev->parent, "can't read wled boost config"
			" register rc=%d\n", rc);
		return rc;
	}

	val = (val & ~WLED_BOOST_LIMIT_MASK) |
		(led->wled_cfg->boost_curr_lim << WLED_BOOST_LIMIT_BIT_SHFT);

	val = (val & ~WLED_OP_FDBCK_MASK) |
		(led->wled_cfg->op_fdbck << WLED_OP_FDBCK_BIT_SHFT);

	rc = pm8xxx_writeb(led->dev->parent, WLED_BOOST_CFG_REG, val);
	if (rc) {
		dev_err(led->dev->parent, "can't write wled boost config"
			" register rc=%d\n", rc);
		return rc;
	}

	/* program high pole capacitance */
	if (led->wled_cfg->cp_select > WLED_CP_SELECT_MAX) {
		dev_err(led->dev->parent, "Invalid pole capacitance");
		return -EINVAL;
	}

	rc = pm8xxx_readb(led->dev->parent, WLED_HIGH_POLE_CAP_REG, &val);
	if (rc) {
		dev_err(led->dev->parent, "can't read wled high pole"
			" capacitance register rc=%d\n", rc);
		return rc;
	}

	val = (val & ~WLED_CP_SELECT_MASK) | led->wled_cfg->cp_select;

	rc = pm8xxx_writeb(led->dev->parent, WLED_HIGH_POLE_CAP_REG, val);
	if (rc) {
		dev_err(led->dev->parent, "can't write wled high pole"
			" capacitance register rc=%d\n", rc);
		return rc;
	}

	/* program activation delay and maximum current */
	for (i = 0; i < num_wled_strings; i++) {
		rc = pm8xxx_readb(led->dev->parent,
				WLED_MAX_CURR_CFG_REG(i), &val);
		if (rc) {
			dev_err(led->dev->parent, "can't read wled max current"
				" config register rc=%d\n", rc);
			return rc;
		}

		if ((led->wled_cfg->ctrl_delay_us % WLED_CTL_DLY_STEP) ||
			(led->wled_cfg->ctrl_delay_us > WLED_CTL_DLY_MAX)) {
			dev_err(led->dev->parent, "Invalid control delay\n");
			return rc;
		}

		val = val / WLED_CTL_DLY_STEP;
		val = (val & ~WLED_CTL_DLY_MASK) |
			(led->wled_cfg->ctrl_delay_us << WLED_CTL_DLY_BIT_SHFT);

		if ((led->max_current > WLED_MAX_CURR)) {
			dev_err(led->dev->parent, "Invalid max current\n");
			return -EINVAL;
		}

		val = (val & ~WLED_MAX_CURR_MASK) | led->max_current;

		rc = pm8xxx_writeb(led->dev->parent,
				WLED_MAX_CURR_CFG_REG(i), val);
		if (rc) {
			dev_err(led->dev->parent, "can't write wled max current"
				" config register rc=%d\n", rc);
			return rc;
		}
	}

	if (led->wled_cfg->cabc_en) {
		rc = pm8xxx_readb(led->dev->parent, WLED_SYNC_REG, &val);
		if (rc) {
			dev_err(led->dev->parent,
				"can't read cabc register rc=%d\n", rc);
			return rc;
		}

		switch (num_wled_strings) {
		case ONE_WLED_STRING:
			val |= (WLED_CABC_ONE_STRING << WLED_CABC_SHIFT);
			break;
		case TWO_WLED_STRINGS:
			val |= (WLED_CABC_TWO_STRING << WLED_CABC_SHIFT);
			break;
		case THREE_WLED_STRINGS:
			val |= (WLED_CABC_THREE_STRING << WLED_CABC_SHIFT);
			break;
		default:
			break;
		}

		rc = pm8xxx_writeb(led->dev->parent, WLED_SYNC_REG, val);
		if (rc) {
			dev_err(led->dev->parent,
				"can't write to enable cabc rc=%d\n", rc);
			return rc;
		}
	}

	/* program digital module generator, cs out and enable the module */
	rc = pm8xxx_readb(led->dev->parent, WLED_MOD_CTRL_REG, &val);
	if (rc) {
		dev_err(led->dev->parent, "can't read wled module ctrl"
			" register rc=%d\n", rc);
		return rc;
	}

	if (led->wled_cfg->dig_mod_gen_en)
		val |= WLED_DIG_MOD_GEN_MASK;

	if (led->wled_cfg->cs_out_en)
		val |= WLED_CS_OUT_MASK;

	val |= WLED_EN_MASK;

	rc = pm8xxx_writeb(led->dev->parent, WLED_MOD_CTRL_REG, val);
	if (rc) {
		dev_err(led->dev->parent, "can't write wled module ctrl"
			" register rc=%d\n", rc);
		return rc;
	}
	led->wled_mod_ctrl_val = val;
	
	rc = pm8xxx_readb(led->dev->parent,WLED_BRIGHTNESS_CNTL_REG1(0), &val);
	if (rc) {
		dev_err(led->dev->parent, "can't read wled brightnes ctrl"
			" register1 rc=%d\n", rc);
		return rc;
	}
	val = val & 0x7F;/* FORCE CABC MODE */
	rc = pm8xxx_writeb(led->dev->parent,WLED_BRIGHTNESS_CNTL_REG1(0), val);
	if (rc) {
		dev_err(led->dev->parent, "can't write wled brightnes ctrl"
			" register1 rc=%d\n", rc);
		return rc;
	}

	/* dump wled registers */
	wled_dump_regs(led);

	return 0;
}

static int __devinit get_init_value(struct pm8xxx_led_data *led, u8 *val)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	int rc, offset;
	u16 addr;

	switch (led->id) {
	case PM8XXX_ID_LED_KB_LIGHT:
		addr = SSBI_REG_ADDR_DRV_KEYPAD;
		break;
	case PM8XXX_ID_LED_0:
	case PM8XXX_ID_LED_1:
	case PM8XXX_ID_LED_2:
		offset = PM8XXX_LED_OFFSET(led->id);
		addr = SSBI_REG_ADDR_LED_CTRL(offset);
		break;
	case PM8XXX_ID_FLASH_LED_0:
		addr = SSBI_REG_ADDR_FLASH_DRV0;
		break;
	case PM8XXX_ID_FLASH_LED_1:
		addr = SSBI_REG_ADDR_FLASH_DRV1;
		break;
	case PM8XXX_ID_WLED:
		rc = init_wled(led);
		if (rc)
			dev_err(led->cdev.dev, "can't initialize wled rc=%d\n",
								rc);
		return rc;
	case PM8XXX_ID_RGB_LED_RED:
	case PM8XXX_ID_RGB_LED_GREEN:
	case PM8XXX_ID_RGB_LED_BLUE:
		addr = SSBI_REG_ADDR_RGB_CNTL1;
		break;
	default:
		dev_err(led->cdev.dev, "unknown led id %d", led->id);
		return -EINVAL;
	}
#else /* QUALCOMM_ORIGINAL_FEATURE */
	int rc;
	u16 addr;

	switch (led->id)
	{
		case PM8XXX_ID_WLED:
			rc = init_wled(led);
			if (rc)
				dev_err(led->cdev.dev, "can't initialize wled rc=%d\n",
									rc);
			return rc;
		case PM8XXX_ID_RGB_LED_RED:
		case PM8XXX_ID_RGB_LED_GREEN:
		case PM8XXX_ID_RGB_LED_BLUE:
			addr = SSBI_REG_ADDR_RGB_CNTL1;
			break;
		case PM8XXX_ID_RGB_LED_AMBER:
			return 0;
		case PM8XXX_ID_FLASH_LED_0:
			addr = SSBI_REG_ADDR_FLASH_DRV0;
			break;
		default:
			dev_err(led->cdev.dev, "unknown led id %d", led->id);
			return -EINVAL;
	}
#endif /* QUALCOMM_ORIGINAL_FEATURE */

	rc = pm8xxx_readb(led->dev->parent, addr, val);
	if (rc)
		dev_err(led->cdev.dev, "can't get led(%d) level rc=%d\n",
							led->id, rc);

	return rc;
}

static int pm8xxx_led_pwm_configure(struct pm8xxx_led_data *led)
{
	int start_idx, idx_len, duty_us, rc;

	led->pwm_dev = pwm_request(led->pwm_channel,
					led->cdev.name);

	if (IS_ERR_OR_NULL(led->pwm_dev)) {
		pr_err("could not acquire PWM Channel %d, "
			"error %ld\n", led->pwm_channel,
			PTR_ERR(led->pwm_dev));
		led->pwm_dev = NULL;
		return -ENODEV;
	}

	if (led->pwm_duty_cycles != NULL) {
		start_idx = led->pwm_duty_cycles->start_idx;
		idx_len = led->pwm_duty_cycles->num_duty_pcts;

		if (idx_len >= PM_PWM_LUT_SIZE && start_idx) {
			pr_err("Wrong LUT size or index\n");
			return -EINVAL;
		}
		if ((start_idx + idx_len) > PM_PWM_LUT_SIZE) {
			pr_err("Exceed LUT limit\n");
			return -EINVAL;
		}

		rc = pm8xxx_pwm_lut_config(led->pwm_dev, led->pwm_period_us,
				led->pwm_duty_cycles->duty_pcts,
				led->pwm_duty_cycles->duty_ms,
				start_idx, idx_len, 0, 0,
				PM8XXX_LED_PWM_FLAGS);
	} else {
		duty_us = (led->pwm_period_us * led->cdev.brightness) / LED_FULL;
		rc = pwm_config(led->pwm_dev, duty_us, led->pwm_period_us);
	}

	pm8xxx_writeb(led->dev->parent, LPG_CTL_7, 0x0);

	return rc;
}



static int pm8xxx_cfg_pwm_duty_pcts(struct pm8xxx_led_data *led)
{
	int rc;
	int duty_us;

	if (led->pwm_duty_cycles != NULL)
	{
		pwm_disable(led->pwm_dev);
		rc = pm8xxx_pwm_lut_enable(led->pwm_dev, 0);
		if (rc < 0)
			return rc;
		rc = pm8xxx_pwm_lut_config(led->pwm_dev, led->pwm_period_us,
				led->pwm_duty_cycles->duty_pcts,
				led->pwm_duty_cycles->duty_ms,
				led->pwm_duty_cycles->start_idx,
				led->pwm_duty_cycles->num_duty_pcts, 0, 0,
				PM8XXX_LED_PWM_FLAGS);
		if (rc < 0)
			return rc;
		rc = pm8xxx_pwm_lut_enable(led->pwm_dev, led->cdev.brightness);
		if (rc < 0)
			return rc;
		rc = pwm_enable(led->pwm_dev);
	}
	else
	{
		duty_us = (led->pwm_period_us * led->cdev.brightness) / LED_FULL;
		rc = pwm_config(led->pwm_dev, duty_us, led->pwm_period_us);
		if (rc < 0)
			return rc;

		led_rgb_write(led, SSBI_REG_ADDR_RGB_CNTL1,
			led->cdev.brightness);
		rc = pwm_enable(led->pwm_dev);
	}

	return rc;
}

static int pm8xxx_ctrl_sub_pwm_duty_pcts(struct pm8xxx_led_data *led)
{
	int rc;

	if ((led_pwm_duty_cycles.num_duty_pcts > 1) &&
		(led->cdev.brightness))
	{
		led->pwm_duty_cycles = &led_pwm_duty_cycles;		/* use blink */
	}
	else
	{
		led->pwm_duty_cycles = NULL;						/* steady */
	}

	rc = pm8xxx_cfg_pwm_duty_pcts(led);
	if (rc < 0)
	{
		return -ENXIO;
	}

	rc = pm8xxx_led_pwm_work(led);
	if (rc < 0)
	{
		return -ENXIO;
	}

	return 0;
}

static int pm8xxx_ctrl_pwm_duty_pcts(struct pm8xxx_led_data *led,
					enum led_brightness brightness,
					uint32_t mode,
					int number)
{
	int rc;

	if (number < 0)
	{
		return -EINVAL;
	}

	led_pwm_duty_cycles.duty_pcts = pwm_duty_pcts;
	led_pwm_duty_cycles.num_duty_pcts = number;
	led_pwm_duty_cycles.duty_ms = mode;
	led_pwm_duty_cycles.start_idx = 0;

	led->cdev.brightness = brightness;
	rc = pm8xxx_ctrl_sub_pwm_duty_pcts(led);

	return rc;
}

static int pm8xxx_make_pwm_duty_pcts(struct led_control_ex_data *led_ctrl, enum led_brightness brightness, bool noblink_mode)
{
	uint32_t pattern;
	int index;
	int buf_ptr = 0;
	int pwm_duty_save;

	if (!pwm_duty_pcts)
	{
		pwm_duty_pcts = (int *)kcalloc(MAX_PWM_DUTY_PCTS, sizeof(int), GFP_KERNEL);
		if (!pwm_duty_pcts)
			return -ENOMEM;
	}

	pwm_duty_pcts[buf_ptr] = 0;
	if (!led_ctrl->mode)
	{
		return 0;
	}
	if (noblink_mode)
	{
		return 0;
	}


	for (index = 0; index < LED_CTRL_MAX_SEQUENCE; ++index)
	{
		pattern = led_ctrl->pattern[index];
		if (!pattern)
			break;

		while (pattern > 0)
		{
			if (buf_ptr >= MAX_PWM_DUTY_PCTS)	/* buffer over flow */
			{
				return -EINVAL;
			}

			if (pattern < led_ctrl->mode)
				pattern = led_ctrl->mode;

			if (index & 0x01)		/* odd is OFF time */
			{
				pwm_duty_pcts[buf_ptr++] = 0;
			}
			else					/* even is ON time */
			{
				pwm_duty_pcts[buf_ptr++] = (brightness * 100) / MAX_RGB_LED_BRIGHTNESS;	/* change to % */
			}

			pattern -= led_ctrl->mode;
		}
	}

	if (index < 2)
	{
		buf_ptr = 0;
	}

	if (buf_ptr > 1)
	{
		pwm_duty_save = pwm_duty_pcts[buf_ptr - 1];
		for (index = (buf_ptr - 2); index >= 0; --index)
		{
			pwm_duty_pcts[index + 1] = pwm_duty_pcts[index];
		}
		pwm_duty_pcts[0] = pwm_duty_save;
	}

	return buf_ptr;
}


static int led_ctrl_check_current_priority(void)
{
	int loop;

	for (loop = 0; loop < LED_PRIORITY_NUMBER; ++loop)
	{
		if (led_ctrl_data[loop].color != LED_COLOR_OFF)
		{
			break;
		}
	}

	return loop;
}


/*
 * Set the received new LED data to the LED control structure
 */
static int set_led_ctrl_data(struct led_control_ex_data *led_ctrl, bool noblink_mode)
{
	int		curr_prio;

	if (led_ctrl->priority < LED_PRIORITY_NUMBER)
	{
		curr_prio = led_ctrl_check_current_priority();

		led_ctrl_data[led_ctrl->priority] = *led_ctrl;

		if (!noblink_mode)
		{
			if ((led_ctrl_data[led_ctrl->priority].mode > 0) &&
				(led_ctrl_data[led_ctrl->priority].mode < MIN_PWM_DUTY_MS))
				led_ctrl_data[led_ctrl->priority].mode = MIN_PWM_DUTY_MS;
			else if (led_ctrl_data[led_ctrl->priority].mode > PM_PWM_LUT_DUTY_TIME_MAX)
				led_ctrl_data[led_ctrl->priority].mode = PM_PWM_LUT_DUTY_TIME_MAX;
		}

		if (led_ctrl->priority <= curr_prio)
		{
			return 1;
		}
	}

	return 0;
}

static void pm8xxx_led_adjust_pwm_time(struct led_control_ex_data *led_ctrl)
{
	uint32_t sum = 0;
	int index;

	for (index = 0; index < LED_CTRL_MAX_SEQUENCE; ++index)
	{
		if (!(led_ctrl->pattern[index]))		/* last of pattern */
			break;

		sum += led_ctrl->pattern[index];
	}

	if ((led_ctrl->mode < MIN_PWM_DUTY_MS) && (sum > 3200UL))
	{
		sum /= 32;
		if (sum > PM_PWM_LUT_DUTY_TIME_MAX)
			sum = PM_PWM_LUT_DUTY_TIME_MAX;
		led_ctrl->mode = sum;
	}
}


static int pm8xxx_led_set_pwm(struct led_control_ex_data *led_ctrl, bool noblink_mode)
{
	int state, num;
	int prio = LED_PRIORITY_NUMBER;
	struct pm8xxx_led_data *led_r, *led_g;

	led_r = pm8xxx_get_led_data_from_id(PM8XXX_ID_RGB_LED_RED);
	led_g = pm8xxx_get_led_data_from_id(PM8XXX_ID_RGB_LED_GREEN);
	if (!led_r || !led_g)
	{
		pr_err("could not get LED classdev\n");
		return LED_PRIORITY_NUMBER;
	}

	state = set_led_ctrl_data(led_ctrl, noblink_mode);
	if (state)											/* LED status changed */
	{
		prio = led_ctrl_check_current_priority();

		if (prio < LED_PRIORITY_NUMBER)
		{
			if ((led_ctrl_data[prio].color & LED_COLOR_RED) &&
				((led_ctrl_data[prio].color & LED_COLOR_GREEN) || (led_ctrl_data[prio].color & LED_COLOR_BLUE)))
			{
				/* amber */
				num = pm8xxx_make_pwm_duty_pcts(&led_ctrl_data[prio], led_r->cdev.max_brightness, noblink_mode);
				if (noblink_mode)
					pm8xxx_ctrl_pwm_duty_pcts(led_r, led_r->cdev.max_brightness, LED_BLINK_OFF, 0);
				else
					pm8xxx_ctrl_pwm_duty_pcts(led_r, led_r->cdev.max_brightness, led_ctrl_data[prio].mode, num);

				num = pm8xxx_make_pwm_duty_pcts(&led_ctrl_data[prio], led_g->cdev.max_brightness, noblink_mode);
				if (noblink_mode)
					pm8xxx_ctrl_pwm_duty_pcts(led_g, led_r->cdev.max_brightness, LED_BLINK_OFF, 0);
				else
					pm8xxx_ctrl_pwm_duty_pcts(led_g, led_g->cdev.max_brightness, led_ctrl_data[prio].mode, num);
			}
			else if (led_ctrl_data[prio].color & LED_COLOR_RED)
			{
				/* red */
				num = pm8xxx_make_pwm_duty_pcts(&led_ctrl_data[prio], led_r->cdev.max_brightness, noblink_mode);
				pm8xxx_ctrl_pwm_duty_pcts(led_g, LED_OFF, 0, 0);
				if (noblink_mode)
					pm8xxx_ctrl_pwm_duty_pcts(led_r, led_r->cdev.max_brightness, LED_BLINK_OFF, 0);
				else
					pm8xxx_ctrl_pwm_duty_pcts(led_r, led_r->cdev.max_brightness, led_ctrl_data[prio].mode, num);
			}
			else
			{
				/* green */
				num = pm8xxx_make_pwm_duty_pcts(&led_ctrl_data[prio], led_g->cdev.max_brightness, noblink_mode);
				pm8xxx_ctrl_pwm_duty_pcts(led_r, LED_OFF, 0, 0);
				if (noblink_mode)
					pm8xxx_ctrl_pwm_duty_pcts(led_g, led_r->cdev.max_brightness, LED_BLINK_OFF, 0);
				else
					pm8xxx_ctrl_pwm_duty_pcts(led_g, led_g->cdev.max_brightness, led_ctrl_data[prio].mode, num);
			}

			return prio;
		}
		else
		{
			led_r->cdev.brightness = LED_OFF;
			led_g->cdev.brightness = LED_OFF;
			pm8xxx_led_pwm_work(led_r);
			pm8xxx_led_pwm_work(led_g);
		}
	}

	return LED_PRIORITY_NUMBER;			/* LED status not changed */
}


static void pm8xxx_set_led_offcharging(struct led_control_ex_data *led_ctrl)
{
	struct pm8xxx_led_data *led_r, *led_g;
	int num;

	led_r = pm8xxx_get_led_data_from_id(PM8XXX_ID_RGB_LED_RED);
	led_g = pm8xxx_get_led_data_from_id(PM8XXX_ID_RGB_LED_GREEN);
	if (!led_r || !led_g)
	{
		pr_err("could not get LED classdev\n");
		return;
	}

	if (led_ctrl->mode != LED_BLINK_OFF)
	{
		if (!(led_ctrl->pattern[0]))
		{												/* set default blink time */
			led_ctrl->mode = 200;
			led_ctrl->pattern[0] = LED_BLINK_DELAY;
			led_ctrl->pattern[1] = LED_BLINK_DELAY;
			led_ctrl->pattern[2] = 0;
		}
	}

	mutex_lock(&led_r->lock);
	mutex_lock(&led_g->lock);

	pm8xxx_led_adjust_pwm_time(led_ctrl);

	if (led_ctrl->color & LED_COLOR_RED)
	{
		/* red */
		num = pm8xxx_make_pwm_duty_pcts(led_ctrl, led_r->cdev.max_brightness, false);
		pm8xxx_ctrl_pwm_duty_pcts(led_g, LED_OFF, 0, 0);
		pm8xxx_ctrl_pwm_duty_pcts(led_r, led_r->cdev.max_brightness, led_ctrl->mode, num);
	}
	else if (led_ctrl->color & LED_COLOR_GREEN)
	{
		/* green */
		num = pm8xxx_make_pwm_duty_pcts(led_ctrl, MAX_RGB_LED_OFFCHARGING, false);
		pm8xxx_ctrl_pwm_duty_pcts(led_r, LED_OFF, 0, 0);
		pm8xxx_ctrl_pwm_duty_pcts(led_g, MAX_RGB_LED_OFFCHARGING, led_ctrl->mode, num);
	}
	else
	{
		led_r->cdev.brightness = LED_OFF;
		led_g->cdev.brightness = LED_OFF;
		pm8xxx_led_pwm_work(led_r);
		pm8xxx_led_pwm_work(led_g);
	}

	mutex_unlock(&led_g->lock);
	mutex_unlock(&led_r->lock);
}


static void pm8xxx_set_control_ex(struct led_control_ex_data *led_ctrl)
{
	struct pm8xxx_led_data *led_r, *led_g;

	if (offcharging)
	{
		pm8xxx_set_led_offcharging(led_ctrl);
		return;
	}

	/* just in case used the mutex_lock. */
	led_r = pm8xxx_get_led_data_from_id(PM8XXX_ID_RGB_LED_RED);
	led_g = pm8xxx_get_led_data_from_id(PM8XXX_ID_RGB_LED_GREEN);
	if (!led_r || !led_g)
	{
		pr_err("could not get LED classdev\n");
		return;
	}

#if LEDS_USE_MODEM
	cancel_delayed_work_sync(&queue_timer);
#endif /* LEDS_USE_MODEM */

	mutex_lock(&led_r->lock);
	mutex_lock(&led_g->lock);
#if LEDS_USE_MODEM
	led_queue_add(led_ctrl);
	pm8xxx_led_control_modem_send();
#else /* LEDS_USE_MODEM */
	pm8xxx_led_set_pwm(led_ctrl, false);
#endif /* LEDS_USE_MODEM */
	mutex_unlock(&led_g->lock);
	mutex_unlock(&led_r->lock);
}

static void pm8xxx_led_control_ex(struct led_classdev *led_cdev,
		unsigned long onoff, unsigned long priority, unsigned long mode, unsigned long *pattern)
{
	struct pm8xxx_led_data *led;
	struct led_control_ex_data led_ctrl;
	int index;

#if LEDS_USE_MODEM
	cancel_delayed_work_sync(&queue_timer);
#endif /* LEDS_USE_MODEM */

	led = container_of(led_cdev, struct pm8xxx_led_data, cdev);
	switch (led->id)
	{
		case PM8XXX_ID_RGB_LED_RED:			/* RED LED */
			if (onoff)
				led_ctrl.color = LED_COLOR_RED;
			else
				led_ctrl.color = LED_COLOR_OFF;
			break;

		case PM8XXX_ID_RGB_LED_GREEN:		/* GREEN LED */
			if (onoff)
				led_ctrl.color = LED_COLOR_GREEN;
			else
				led_ctrl.color = LED_COLOR_OFF;
			break;

		case PM8XXX_ID_RGB_LED_AMBER:		/* AMBER LED */
			if (onoff)
				led_ctrl.color = LED_COLOR_AMBER;
			else
				led_ctrl.color = LED_COLOR_OFF;
			break;

		default:
			pr_err("Invalid LED ID:%d\n", led->id);
			return;
	}

	led_ctrl.priority  = priority;
	led_ctrl.mode      = mode;
	for (index = 0; index < LED_CTRL_MAX_SEQUENCE; ++index)
		led_ctrl.pattern[index] = pattern[index];

#if !LEDS_USE_MODEM
	pm8xxx_led_adjust_pwm_time(&led_ctrl);
#endif
	pm8xxx_set_control_ex(&led_ctrl);
}


#if LEDS_USE_MODEM
static void led_queue_timer_cb(struct work_struct *work)
{
	pm8xxx_led_control_modem_send();
}
#endif /* LEDS_USE_MODEM */

void pm8xxx_set_leddata_ex(struct led_control_ex_data *led_ctrl)
{
#if !LEDS_USE_MODEM
	pm8xxx_led_adjust_pwm_time(led_ctrl);
#endif

	if (led_ctrl->priority >= LED_PRIORITY_NUMBER)
	{
		pr_err("Invalid priority number [%d]\n", led_ctrl->priority);
		return;
	}

	pm8xxx_set_control_ex(led_ctrl);
}


static int __devinit pm8xxx_led_probe(struct platform_device *pdev)
{
	const struct pm8xxx_led_platform_data *pdata = pdev->dev.platform_data;
	const struct led_platform_data *pcore_data;
	struct led_info *curr_led;
	struct pm8xxx_led_data *led, *led_dat;
	struct pm8xxx_led_config *led_cfg;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	enum pm8xxx_version version;
	bool found = false;
	int rc, i, j;
#else
	int rc, i;
#endif
	unsigned int *power_on_status;
	unsigned int size;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data not supplied\n");
		return -EINVAL;
	}

	pcore_data = pdata->led_core;

	if (pcore_data->num_leds != pdata->num_configs) {
		dev_err(&pdev->dev, "#no. of led configs and #no. of led"
				"entries are not equal\n");
		return -EINVAL;
	}

#if LEDS_USE_MODEM
	rc = led_queue_init();
	if (rc < 0)
	{
		dev_err(&pdev->dev, "LED: failed to alloc memory\n");
		return -ENOMEM;
	}
#endif /* LEDS_USE_MODEM */

	offcharging = false;
	power_on_status = smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &size);
	if (power_on_status)
	{
		if (*power_on_status & 0x20)			/* if now on off-charging, then reject normal control. */
		{
			offcharging = true;
		}
	}
	else
	{
		pr_err("Can't find shared POWER_ON_STATUS!\n");
	}

	led = kcalloc(pcore_data->num_leds, sizeof(*led), GFP_KERNEL);
	if (led == NULL) {
		dev_err(&pdev->dev, "failed to alloc memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < pcore_data->num_leds; i++) {
		curr_led	= &pcore_data->leds[i];
		led_dat		= &led[i];
		led_cfg		= &pdata->configs[i];

		led_dat->id     = led_cfg->id;
		led_dat->pwm_channel = led_cfg->pwm_channel;
		led_dat->pwm_period_us = led_cfg->pwm_period_us;
		led_dat->pwm_duty_cycles = led_cfg->pwm_duty_cycles;
		led_dat->wled_cfg = led_cfg->wled_cfg;
		led_dat->max_current = led_cfg->max_current;

		if (!((led_dat->id >= PM8XXX_ID_LED_KB_LIGHT) &&
				(led_dat->id < PM8XXX_ID_MAX))) {
			dev_err(&pdev->dev, "invalid LED ID(%d) specified\n",
				led_dat->id);
			rc = -EINVAL;
			goto fail_id_check;

		}

#ifdef QUALCOMM_ORIGINAL_FEATURE
		found = false;
		version = pm8xxx_get_version(pdev->dev.parent);
		for (j = 0; j < ARRAY_SIZE(led_map); j++) {
			if (version == led_map[j].version
			&& (led_map[j].supported & (1 << led_dat->id))) {
				found = true;
				break;
			}
		}

		if (!found) {
			dev_err(&pdev->dev, "invalid LED ID(%d) specified\n",
				led_dat->id);
			rc = -EINVAL;
			goto fail_id_check;
		}
#endif /* QUALCOMM_ORIGINAL_FEATURE */

		led_dat->cdev.name		= curr_led->name;
		led_dat->cdev.default_trigger   = curr_led->default_trigger;
		led_dat->cdev.brightness_set    = pm8xxx_led_set;
		led_dat->cdev.brightness_get    = pm8xxx_led_get;
		led_dat->cdev.brightness	= LED_OFF;
/*
		led_dat->cdev.flags		= curr_led->flags;
*/
		led_dat->cdev.flags		= 0;        /* don't turn off LED at suspended */
		led_dat->cdev.control_ex = pm8xxx_led_control_ex;
		led_dat->dev			= &pdev->dev;

		rc =  get_init_value(led_dat, &led_dat->reg);
		if (rc < 0)
			goto fail_id_check;

		rc = pm8xxx_set_led_mode_and_max_brightness(led_dat,
					led_cfg->mode, led_cfg->max_current);
		if (rc < 0)
			goto fail_id_check;

		mutex_init(&led_dat->lock);
		INIT_WORK(&led_dat->work, pm8xxx_led_work);

		rc = led_classdev_register(&pdev->dev, &led_dat->cdev);
		if (rc) {
			dev_err(&pdev->dev, "unable to register led %d,rc=%d\n",
						 led_dat->id, rc);
			goto fail_id_check;
		}

		/* configure default state */
		if (led_cfg->default_state)
			led->cdev.brightness = led_dat->cdev.max_brightness;
		else
			led->cdev.brightness = LED_OFF;

		if (led_cfg->mode != PM8XXX_LED_MODE_MANUAL) {

			if (led_dat->pwm_channel != -1) {
#ifdef QUALCOMM_ORIGINAL_FEATURE
				led_dat->cdev.max_brightness = LED_FULL;
#else /* QUALCOMM_ORIGINAL_FEATURE */
#endif /* QUALCOMM_ORIGINAL_FEATURE */
				if (led_dat->id != PM8XXX_ID_RGB_LED_AMBER)
				{
					rc = pm8xxx_led_pwm_configure(led_dat);
					if (rc) {
						dev_err(&pdev->dev, "failed to "
						"configure LED, error: %d\n", rc);
						goto fail_id_check;
					}
				}
				schedule_work(&led->work);		/* call pm8xxx_led_work() */
			}
		} else {
		}
	}

	platform_set_drvdata(pdev, led);

	return 0;

fail_id_check:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--) {
			mutex_destroy(&led[i].lock);
			led_classdev_unregister(&led[i].cdev);
			if (led[i].pwm_dev != NULL)
				pwm_free(led[i].pwm_dev);
		}
	}
	kfree(led);
	return rc;
}

static int __devexit pm8xxx_led_remove(struct platform_device *pdev)
{
	int i;
	const struct led_platform_data *pdata =
				pdev->dev.platform_data;
	struct pm8xxx_led_data *led = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		cancel_work_sync(&led[i].work);
		mutex_destroy(&led[i].lock);
		led_classdev_unregister(&led[i].cdev);
		if (led[i].pwm_dev != NULL)
			pwm_free(led[i].pwm_dev);
	}

	kfree(led);

	return 0;
}

static struct platform_driver pm8xxx_led_driver = {
	.probe		= pm8xxx_led_probe,
	.remove		= __devexit_p(pm8xxx_led_remove),
	.driver		= {
		.name	= PM8XXX_LEDS_DEV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init pm8xxx_led_init(void)
{
	return platform_driver_register(&pm8xxx_led_driver);
}
subsys_initcall(pm8xxx_led_init);

static void __exit pm8xxx_led_exit(void)
{
	platform_driver_unregister(&pm8xxx_led_driver);
}
module_exit(pm8xxx_led_exit);

MODULE_DESCRIPTION("PM8XXX LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pm8xxx-led");
