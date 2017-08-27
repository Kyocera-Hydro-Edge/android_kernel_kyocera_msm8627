/* 
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
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */

#include <linux/debugfs.h>
#include <mach/msm_smsm.h>

#define CHG_PARAM_SIZE		160
#define SMEM_CHG_PARAM_SHARE	8

#define SET_CHG_PARAM_INIT(val, type, member)			\
	if (0 == val ) {					\
		oem_param_##type.member = 			\
			ptr->oem_chg_param_##type.member;	\
	}

#define SET_CHG_PARAM(val, type, member)			\
	if (val != ptr->oem_chg_param_##type.member){		\
		oem_param_##type.member = 			\
			ptr->oem_chg_param_##type.member;	\
		pr_debug("chg param %s.%s : 0x%08x"		\
		    , #type, #member, oem_param_##type.member);	\
	}

#define SET_CHG_PARAM_MINMAX(val, min, max, type, member)		\
	if (val != ptr->oem_chg_param_##type.member){			\
		if (min > ptr->oem_chg_param_##type.member) {		\
			oem_param_##type.member = min;			\
		} else if (max < ptr->oem_chg_param_##type.member) {	\
			oem_param_##type.member = max;			\
		} else {						\
			oem_param_##type.member = 			\
				ptr->oem_chg_param_##type.member;	\
		}							\
		pr_debug("chg param %s.%s : 0x%08x"			\
		    , #type, #member, oem_param_##type.member);		\
	}

typedef struct {
	uint16_t	chg_stop_volt;
	uint16_t	reserve1_1;
	uint16_t	reserve1_2;
	uint16_t	reserve1_3;

	uint16_t	time_chg;
	uint16_t	time_cool_chg;
	uint16_t	time_chg_pm;
	uint16_t	time_trkl_pm;
	uint16_t	reserve2_1;
	uint16_t	reserve2_2;

	uint8_t		chg_cool_tmp;
	uint8_t		chg_warm_tmp;
	uint8_t		wait_chg_on;
	uint8_t		wait_chg_off;
	uint8_t		chg_adp_tmp1;
	uint8_t		chg_adp_tmp2;
	uint8_t		chg_adp_tmp3;
	uint8_t		reserve3_2;

	uint16_t	normal_chg;
	uint16_t	cool_chg;
	uint16_t	warm_chg;
	uint16_t	uim_undete_chg;
	uint16_t	reserve4_1;
	uint16_t	reserve4_2;

	uint16_t	rechg_delta_volt;
	uint16_t	initial_delta_volt;

	uint16_t	maint_chg_vin;
	uint16_t	reserve6_1;
	uint16_t	reserve6_2;
	uint16_t	reserve6_3;

	uint16_t	maint_wait_chg_on_volt;
	uint16_t	maint_wait_chg_off_volt;
	uint16_t	reserve7_1;
	uint16_t	reserve7_2;

	uint16_t	i_chg_norm;
	uint16_t	i_chg_cool;
	uint16_t	i_chg_warm;
	uint16_t	i_chg_finish;
	uint16_t	reserve8_1;
	uint16_t	reserve8_2;

	uint16_t	reserve9_1;
	uint16_t	reserve9_2;
	uint16_t	reserve9_3;
	uint16_t	reserve9_4;

	uint16_t	maint_wait_chg_on_time;
	uint16_t	maint_wait_chg_off_time;

	uint16_t	i_chg_adp_chk;
	uint16_t	reserve11_1;

	uint16_t	z_chg_adp_chk;
	uint16_t	reserve12_1;

	int8_t		chg_cam_tmp_off;
	int8_t		chg_cam_tmp_on;
	int8_t		chg_inte_cam_on;
	int8_t		chg_inte_cam_off;
	int8_t		chg_phone_tmp_off;
	int8_t		chg_phone_tmp_on;
	int8_t		chg_inte_phone_on;
	int8_t		chg_inte_phone_off;
	int8_t		chg_adp_tmp_delta;
	int8_t		reserve13_1;
	int8_t		reserve13_2;
	int8_t		reserve13_3;

	uint16_t	reserve14_1;
	uint16_t	reserve14_2;
	uint16_t	reserve14_3;
	uint16_t	reserve14_4;
}pm8921_oem_chg_param_charger;

typedef struct {
	uint32_t	cal_vbat1;
	uint32_t	cal_vbat2;

	uint16_t	reserve2_1;
	uint16_t	reserve2_2;
	uint16_t	reserve2_3;
	uint16_t	reserve2_4;
}pm8921_oem_chg_param_hkadc;

typedef struct {
	uint16_t	temp_normal_thresh_pow_off;
	uint16_t	temp_normal_thresh_low_batt;
	uint16_t	temp_low_thresh_pow_off;
	uint16_t	temp_low_thresh_low_batt;
	uint16_t	bat_alarm5;
	uint16_t	bat_alarm6;

	uint8_t		soc_rate_i_0;
	uint8_t		soc_rate_i_1;
	uint8_t		soc_rate_i_2;
	uint8_t		reserve2_1;

	uint16_t	reserve3_1;
	uint16_t	reserve3_2;
	uint16_t	reserve3_3;
	uint16_t	reserve3_4;
}pm8921_oem_chg_param_bms;

typedef struct {
	uint8_t		factory_mode_1;
	uint8_t		test_mode_chg_disable;
	uint8_t		reserve1_3;
	uint8_t		reserve1_4;

	uint32_t	last_chargecycles;
}pm8921_oem_chg_param_share;

extern pm8921_oem_chg_param_charger oem_param_charger;
extern pm8921_oem_chg_param_hkadc oem_param_hkadc;
extern pm8921_oem_chg_param_bms oem_param_bms;
extern pm8921_oem_chg_param_share oem_param_share;

void oem_chg_param_init(void);

