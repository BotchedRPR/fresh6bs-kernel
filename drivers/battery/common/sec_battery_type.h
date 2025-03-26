/*
 * sec_battery_type.h
 * Samsung Mobile Battery Type Header
 *
 * Copyright (C) 2022 Samsung Electronics, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SEC_BATTERY_TYPE_H
#define __SEC_BATTERY_TYPE_H __FILE__

#include <linux/battery/common/sb_type.h>
#include <linux/battery/control/sb_ctrl.h>

struct sec_charging_current {
	unsigned int input_current_limit;
	unsigned int fast_charging_current;

	/* time to full fast charging current */
	unsigned int ttf_fcc;

	/* flags */
	union {
		unsigned int flag;

		struct {
			/* step work */
			unsigned icl_step_work : 1,
				fcc_step_work : 1;

			/* set target device flag */
			unsigned icl_tdev_wpc : 1,
				fcc_tdev_wpc : 1;

			unsigned revs : 28;
		} flags;
	};

	/* control */
	struct sb_ctrl *ctrl;
};

#define sec_charging_current_t \
	struct sec_charging_current

struct sec_age_data {
	unsigned int cycle;
	unsigned int float_voltage;
	unsigned int recharge_condition_vcell;
	unsigned int full_condition_vcell;
	unsigned int full_condition_soc;
};

#define sec_age_data_t \
	struct sec_age_data

struct sec_battery_platform_data {
	char *chip_vendor;
	char *pmic_name;

	/* battery */
	char *vendor;
	int technology;
	int battery_type;
	void *battery_data;

	unsigned int *polling_time;

	sec_battery_cable_check_t cable_check_type;
	sec_battery_cable_source_t cable_source_type;

	/* Monitor setting */
	sec_battery_monitor_polling_t polling_type;
	/* for initial check */
	unsigned int monitor_initial_count;

	/* Battery check */
	sec_battery_check_t battery_check_type;
	/* how many times do we need to check battery */
	unsigned int check_count;

	/* OVP/UVLO check */
	sec_battery_ovp_uvlo_t ovp_uvlo_check_type;
	sec_battery_temp_check_t temp_check_type;
	unsigned int temp_check_count;

	/* charging current for type (0: not use, default) */
	sec_charging_current_t *charging_current;
	/* float voltage (mV) */
	unsigned int chg_float_voltage;

	unsigned int full_check_current_1st;
	unsigned int full_check_current_2nd;
	unsigned int ux_full_inow;
	unsigned int ux_rechg_inow;

	unsigned int expired_time;
	unsigned int recharging_expired_time;
	bool single_charger_path;
	int standard_curr;

	unsigned int store_wpc_current;
	unsigned int store_mode_chg_max;
	unsigned int store_mode_chg_min;

	unsigned int tct_low_current;

	unsigned int swelling_high_rechg_voltage;
	unsigned int swelling_low_rechg_voltage;

	/*
	 * limit can be ADC value or Temperature
	 * depending on temp_check_type
	 * temperature should be temp x 10 (0.1 degree)
	 */
	int wireless_cold_cool3_thresh;
	int wireless_cool3_cool2_thresh;
	int wireless_cool2_cool1_thresh;
	int wireless_cool1_normal_thresh;
	int wireless_normal_warm_thresh;
	int wireless_warm_overheat_thresh;
	int warm_recov_thresh;

	int wire_cold_cool3_thresh;
	int wire_cool3_cool2_thresh;
	int wire_cool2_cool1_thresh;
	int wire_cool1_normal_thresh;
	int wire_normal_warm_thresh;
	int wire_warm_overheat_thresh;

	int wire_warm_current;
	int wire_cool1_current;
	int wire_cool2_current;
	int wire_cool3_current;
	int wireless_warm_current;
	int wireless_cool1_current;
	int wireless_cool2_current;
	int wireless_cool3_current;
	int high_temp_topoff;
	int low_temp_topoff;
	int high_temp_float;
	int low_temp_float;

	unsigned int max_charging_current;

	/* If these is NOT full check type or NONE full check type,
	 * it is skipped
	 */
	/* 1st full check */
	sec_battery_full_charged_t full_check_type;
	/* 2nd full check */
	sec_battery_full_charged_t full_check_type_2nd;
	unsigned int full_check_count;
	unsigned int swell_full_check_count;
	int chg_gpio_full_check;
	/* 1 : active high, 0 : active low */
	int chg_polarity_full_check;
	sec_battery_full_condition_t full_condition_type;
	unsigned int full_condition_soc;
	unsigned int full_condition_vcell;
	unsigned int full_condition_avgvcell;
	unsigned int full_condition_ocv;

	unsigned int recharge_check_count;
	sec_battery_recharge_condition_t recharge_condition_type;
	unsigned int recharge_condition_soc;
	unsigned int recharge_condition_avgvcell;
	unsigned int recharge_condition_vcell;

	char *fuelgauge_name;
	char *charger_name;
	char *wireless_charger_name;

	char *fgsrc_switch_name;
	bool support_fgsrc_change;

	int num_age_step;
	int age_step;
	int age_data_length;
	sec_age_data_t *age_data;

	int num_age_cc_cv_threshold;
	int *age_cc_cv_threshold;

	bool fake_capacity;

	unsigned int battery_full_capacity;
};

#define sec_battery_platform_data_t \
	struct sec_battery_platform_data

#endif /* __SEC_BATTERY_EVENT_H */
