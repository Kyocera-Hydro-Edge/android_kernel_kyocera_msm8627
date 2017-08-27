/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

#ifndef __PM8XXX_BMS_H
#define __PM8XXX_BMS_H

#include <linux/errno.h>
#include <linux/mfd/pm8xxx/batterydata-lib.h>

#define PM8921_BMS_DEV_NAME	"pm8921-bms"

#ifdef QUALCOMM_ORIGINAL_FEATURE
#else
#define RBATT_ROWS		22
#define RBATT_COLS		5

#define SOC_ADJUST_ROWS		6
#define SOC_ADJUST_COLS		8
struct rbatt_lut {
	int rows;
	int cols;
	int temp[RBATT_COLS];
	int vbatt[RBATT_ROWS];
	int rbatt[RBATT_ROWS][RBATT_COLS];
};

struct soc_adjust_lut {
	int rows;
	int cols;
	int temp[SOC_ADJUST_COLS];
	int ibatt[SOC_ADJUST_ROWS];
	int soc_adjust[SOC_ADJUST_ROWS][SOC_ADJUST_COLS];
};
#endif

struct pm8xxx_bms_core_data {
	unsigned int	batt_temp_channel;
	unsigned int	vbat_channel;
	unsigned int	ref625mv_channel;
	unsigned int	ref1p25v_channel;
	unsigned int	batt_id_channel;
};

/**
 * struct pm8921_bms_platform_data -
 * @batt_type:		allows to force chose battery calibration data
 * @r_sense_uohm:	sense resistor value in (micro Ohms)
 * @i_test:		current at which the unusable charger cutoff is to be
 *			calculated or the peak system current (mA)
 * @v_cutoff:		the loaded voltage at which the battery
 *			is considered empty(mV)
 * @enable_fcc_learning:	if set the driver will learn full charge
 *				capacity of the battery upon end of charge
 * @normal_voltage_calc_ms:	The period of soc calculation in ms when battery
 *				voltage higher than cutoff voltage
 * @low_voltage_calc_ms:	The period of soc calculation in ms when battery
 *				voltage is near cutoff voltage
 */
struct pm8921_bms_platform_data {
	struct pm8xxx_bms_core_data	bms_cdata;
	enum battery_type		battery_type;
	int				r_sense_uohm;
	unsigned int			i_test;
	unsigned int			v_cutoff;
	unsigned int			max_voltage_uv;
	unsigned int			rconn_mohm;
	int				enable_fcc_learning;
	int				shutdown_soc_valid_limit;
	int				ignore_shutdown_soc;
	int				adjust_soc_low_threshold;
	int				chg_term_ua;
	int				normal_voltage_calc_ms;
	int				low_voltage_calc_ms;
};

#ifdef QUALCOMM_ORIGINAL_FEATURE
#else
struct pm8921_bms_oem_battery_data {
	struct	rbatt_lut	*rbatt_initial_lut;
	struct	soc_adjust_lut	*soc_adjust_lut;
	struct	single_row_lut	*cycle_adjust_lut;
};
#endif
#if defined(CONFIG_PM8921_BMS) || defined(CONFIG_PM8921_BMS_MODULE)
#ifdef QUALCOMM_ORIGINAL_FEATURE
#else
extern struct pm8921_bms_oem_battery_data pm8921_bms_oem_data;

enum pm8921_bms_chg_state {
	CHG_STATE_NONCONNECTED,
	CHG_STATE_IDLE,
	CHG_STATE_TRICKLE,
	CHG_STATE_FAST_COOL,
	CHG_STATE_FAST_NORMAL,
	CHG_STATE_FAST_WARM,
	CHG_STATE_INTE_COOL,
	CHG_STATE_INTE_NORMAL,
	CHG_STATE_INTE_WARM,
	CHG_STATE_CHG_COMP,
	CHG_STATE_CHG_TIMEOUT,
	CHG_STATE_CHG_STAND,
	CHG_STATE_BATT_TEMP_COLD,
	CHG_STATE_BATT_TEMP_HOT,
	CHG_STATE_WAIT_TEMP,
	CHG_STATE_BATT_ID_ERROR,
	CHG_STATE_CHG_ERROR,
	CHG_STATE_INIT,
	CHG_STATE_MAX
};

enum pm8921_bms_chg_mode {
	CHG_MODE_DISCHARGE,
	CHG_MODE_CHARGING,
	CHG_MODE_FULL
};

enum pm8921_bms_chg_condition {
	CHG_CONDITION_4340MV,
	CHG_CONDITION_4240MV,
	CHG_CONDITION_4340MV_INTE,
	CHG_CONDITION_4240MV_INTE,
	CHG_CONDITION_MAX,
	CHG_CONDITION_NULL
};

struct pm8921_bms_correction {
	enum pm8921_bms_chg_state	chg_state;
	enum pm8921_bms_chg_mode	chg_mode;
	enum pm8921_bms_chg_condition	chg_condition_uim_valid;
	enum pm8921_bms_chg_condition	chg_condition_uim_invalid;
};

enum pm8921_bms_cyclecorrect_state {
	CHG_CYCLECORRECT_STATE_CHARGER_NO,
	CHG_CYCLECORRECT_STATE_CHARGER_DETECTED,
	CHG_CYCLECORRECT_STATE_OK1,
	CHG_CYCLECORRECT_STATE_OK2,
	CHG_CYCLECORRECT_STATE_PARAM_OBTAINED1,
	CHG_CYCLECORRECT_STATE_PARAM_OBTAINED2,
	CHG_CYCLECORRECT_STATE_CALCULATED
};
#endif
/**
 * pm8921_bms_get_vsense_avg - return the voltage across the sense
 *				resitor in microvolts
 * @result:	The pointer where the voltage will be updated. A -ve
 *		result means that the current is flowing in
 *		the battery - during battery charging
 *
 * RETURNS:	Error code if there was a problem reading vsense, Zero otherwise
 *		The result won't be updated in case of an error.
 *
 *
 */
int pm8921_bms_get_vsense_avg(int *result);

/**
 * pm8921_bms_get_battery_current - return the battery current based on vsense
 *				resitor in microamperes
 * @result:	The pointer where the voltage will be updated. A -ve
 *		result means that the current is flowing in
 *		the battery - during battery charging
 *
 * RETURNS:	Error code if there was a problem reading vsense, Zero otherwise
 *		The result won't be updated in case of an error.
 *
 */
int pm8921_bms_get_battery_current(int *result);

/**
 * pm8921_bms_get_percent_charge - returns the current battery charge in percent
 *
 */
#ifdef QUALCOMM_ORIGINAL_FEATURE
int pm8921_bms_get_percent_charge(void);
#else
int pm8921_bms_get_percent_charge(int chg_state, int uim_valid);
#endif

/**
 * pm8921_bms_get_fcc - returns fcc in mAh of the battery depending on its age
 *			and temperature
 *
 */
int pm8921_bms_get_fcc(void);

/**
 * pm8921_bms_charging_began - function to notify the bms driver that charging
 *				has started. Used by the bms driver to keep
 *				track of chargecycles
 */
void pm8921_bms_charging_began(void);
/**
 * pm8921_bms_charging_end - function to notify the bms driver that charging
 *				has stopped. Used by the bms driver to keep
 *				track of chargecycles
 */
void pm8921_bms_charging_end(int is_battery_full);

void pm8921_bms_calibrate_hkadc(void);
/**
 * pm8921_bms_get_simultaneous_battery_voltage_and_current
 *		- function to take simultaneous vbat and vsense readings
 *		  this puts the bms in override mode but keeps coulumb couting
 *		  on. Useful when ir compensation needs to be implemented
 */
int pm8921_bms_get_simultaneous_battery_voltage_and_current(int *ibat_ua,
								int *vbat_uv);
/**
 * pm8921_bms_get_rbatt - function to get the battery resistance in mOhm.
 */
int pm8921_bms_get_rbatt(void);
/**
 * pm8921_bms_invalidate_shutdown_soc - function to notify the bms driver that
 *					the battery was replaced between reboot
 *					and so it should not use the shutdown
 *					soc stored in a coincell backed register
 */
void pm8921_bms_invalidate_shutdown_soc(void);

/**
 * pm8921_bms_cc_uah -	function to get the coulomb counter based charge. Note
 *			that the coulomb counter are reset when the current
 *			consumption is low (below 8mA for more than 5 minutes),
 *			This will lead in a very low coulomb counter charge
 *			value upon wakeup from sleep.
 */
int pm8921_bms_cc_uah(int *cc_uah);
#ifdef QUALCOMM_ORIGINAL_FEATURE
#else
/**
 * oem_pm8921_bms_change_table - replace the parameters of the bms
 *
 */
void oem_pm8921_bms_change_table(void);

/**
 * oem_pm8921_bms_low_vol_detect_active - low-voltage detection in active
 *
 */
void oem_pm8921_bms_low_vol_detect_active(int vbatt, int batt_temp, int init_state);

/**
 * oem_pm8921_bms_low_vol_detect_standby - low-voltage detection in standby
 *
 */
void oem_pm8921_bms_low_vol_detect_standby(int batt_temp);

/**
 * oem_pm8921_bms_detected_low_vol - low-voltage detection in standby
 *
 */
void oem_pm8921_bms_detected_low_vol(int vol_mv);

int oem_pm8921_bms_get_chargecycles(void);
#endif
#else
static inline int pm8921_bms_get_vsense_avg(int *result)
{
	return -ENXIO;
}
static inline int pm8921_bms_get_battery_current(int *result)
{
	return -ENXIO;
}
#ifdef QUALCOMM_ORIGINAL_FEATURE
static inline int pm8921_bms_get_percent_charge(void)
#else
static inline int pm8921_bms_get_percent_charge(int chg_state)
#endif
{
	return -ENXIO;
}
static inline int pm8921_bms_get_fcc(void)
{
	return -ENXIO;
}
static inline void pm8921_bms_charging_began(void)
{
}
static inline void pm8921_bms_charging_end(int is_battery_full)
{
}
static inline void pm8921_bms_calibrate_hkadc(void)
{
}
static inline int pm8921_bms_get_simultaneous_battery_voltage_and_current(
						int *ibat_ua, int *vbat_uv)
{
	return -ENXIO;
}
static inline int pm8921_bms_get_rbatt(void)
{
	return -EINVAL;
}
static inline void pm8921_bms_invalidate_shutdown_soc(void)
{
}
static inline int pm8921_bms_cc_uah(int *cc_uah)
{
	return -ENXIO;
}
#ifdef QUALCOMM_ORIGINAL_FEATURE
#else
static inline void oem_pm8921_bms_change_table(void)
{
}
static inline void oem_pm8921_bms_low_vol_detect_active(int vbatt, int batt_temp, int init_state)
{
}
static inline void oem_pm8921_bms_low_vol_detect_standby(int batt_temp)
{
}
static inline void oem_pm8921_bms_detected_low_vol(int vol_mv)
{
}
static int oem_pm8921_bms_get_chargecycles(void)
{
}
#endif
#endif

#endif
