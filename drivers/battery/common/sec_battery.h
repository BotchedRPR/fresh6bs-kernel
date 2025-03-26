/*
 * sec_battery.h
 * Samsung Mobile Battery Header
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

#ifndef __SEC_BATTERY_H
#define __SEC_BATTERY_H __FILE__

#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/alarmtimer.h>
#include <linux/power_supply.h>

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/common/muic.h>
#include <linux/muic/common/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#include <linux/battery/sb_vote.h>
#include <linux/battery/sb_notify.h>

#include "sec_battery_type.h"

struct sec_cisd;
struct sec_ttf_data;
struct sec_bat_eng_data;

#define DEFAULT_HEALTH_CHECK_COUNT	5

struct sec_battery_info {
	struct device *dev;
	sec_battery_platform_data_t *pdata;

	/* power supply used in Android */
	struct power_supply *psy_bat;
	struct power_supply *psy_usb;
	struct power_supply *psy_ac;
	struct power_supply *psy_wireless;
	unsigned int irq;

	struct notifier_block sb_nb;

	struct sec_cisd *cisd;
	struct sec_ttf_data *ttf_d;
	struct sec_bat_eng_data *eng_data;

	struct notifier_block batt_nb;

	bool safety_timer_set;
	bool lcd_status;
	bool skip_swelling;

	int status;
	int health;
	bool present;
	unsigned int charger_mode;

	int voltage_now;		/* cell voltage (mV) */
	int voltage_avg;		/* average voltage (mV) */
	int voltage_ocv;		/* open circuit voltage (mV) */
	int current_now;		/* current (mA) */
	int inbat_adc;			/* inbat adc */
	int current_avg;		/* average current (mA) */
	int current_max;		/* input current limit (mA) */
	int charge_counter;		/* remaining capacity (uAh) */
	int current_adc;

	unsigned int capacity;			/* SOC (%) */
	unsigned int input_voltage;		/* CHGIN/WCIN input voltage (V) */
	unsigned int charge_power;		/* charge power (mW) */
	unsigned int max_charge_power;		/* max charge power (mW) */
	unsigned int aicl_current;

	/* keep awake until monitor is done */
	struct wakeup_source *monitor_ws;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;

	unsigned int polling_count;
	unsigned int polling_time;
	bool polling_in_sleep;
	bool polling_short;

	struct delayed_work polling_work;
	struct alarm polling_alarm;
	ktime_t last_poll_time;

	/* battery check */
	unsigned int check_count;

	/* health change check*/
	bool health_change;
	/* ovp-uvlo health check */
	int health_check_count;

	/* time check */
	unsigned long charging_start_time;
	unsigned long charging_passed_time;
	unsigned long charging_fullcharged_time;

	/* chg temperature check */
	unsigned int chg_limit;

	int cold_cool3_thresh;
	int cool3_cool2_thresh;
	int cool2_cool1_thresh;
	int cool1_normal_thresh;
	int normal_warm_thresh;
	int warm_overheat_thresh;
	int thermal_zone;
	int bat_thm_count;

	/* charging */
	unsigned int charging_mode;
	bool is_recharging;
	int wdt_kick_disable;
	int topoff_condition;

	bool is_jig_on;
	int cable_type;
	int muic_cable_type;

	struct wakeup_source *cable_ws;
	struct delayed_work cable_work;
	struct wakeup_source *vbus_ws;
	struct delayed_work parse_mode_dt_work;
	struct wakeup_source *parse_mode_dt_ws;

	struct delayed_work fcc_work;
	struct wakeup_source *fcc_ws;

	char batt_type[48];
	unsigned int full_check_cnt;
	unsigned int recharge_check_cnt;

	struct mutex iolock;
	unsigned int input_current;
	unsigned int charging_current;
	unsigned int topoff_current;
	unsigned int current_event;

	unsigned int cstep;
	unsigned int cstep_applied;
	/* wireless charging enable */
	int wc_enable;
	int wc_enable_cnt;
	int wc_enable_cnt_value;
	int wire_status;

	/* test mode */
	int test_mode;
	bool factory_mode;
	bool startup_state;
	bool store_mode;
	bool low_current_tct;

	int stability_test;
	int eng_not_full_status;

	int batt_cycle;

	struct mutex misclock;
	unsigned int misc_event;
	unsigned int prev_misc_event;
	struct delayed_work misc_event_work;
	struct wakeup_source *misc_event_ws;
	struct mutex batt_handlelock;
	struct mutex current_eventlock;

	bool stop_timer;
	unsigned long prev_safety_time;
	unsigned long expired_time;
	unsigned long cal_safety_time;
	int fg_reset;

	struct sb_vote *fcc_vote;
	struct sb_vote *input_vote;
	struct sb_vote *fv_vote;
	struct sb_vote *chgen_vote;
	struct sb_vote *topoff_vote;

	bool disable_temp_check;
	int backup_wire_thr[2];
};

void sec_bat_get_battery_info(struct sec_battery_info *battery);
void sec_bat_set_misc_event(struct sec_battery_info *battery, unsigned int misc_event_val, unsigned int misc_event_mask);
void sec_bat_set_current_event(struct sec_battery_info *battery, unsigned int current_event_val, unsigned int current_event_mask);
int sec_bat_set_charging_current(struct sec_battery_info *battery);
void sec_bat_aging_check(struct sec_battery_info *battery);

void sec_bat_temp_misc_test(struct sec_battery_info *battery, bool enable);
void sec_bat_thermal_check(struct sec_battery_info *battery);
void sec_bat_set_threshold(struct sec_battery_info *battery);
void sec_bat_set_charging_status(struct sec_battery_info *battery, int status);

#if IS_ENABLED(CONFIG_SB_CISD)
int sec_bat_set_cisd_state(struct sec_cisd *cisd, bool state);
int sec_bat_cisd_check(struct sec_battery_info *battery);
struct sec_cisd *sec_battery_cisd_init(struct sec_battery_info *battery);
#else
static inline int sec_bat_set_cisd_state(struct sec_cisd *cisd, bool state)
{ return 0; }
static inline int sec_bat_cisd_check(struct sec_battery_info *battery)
{ return 0; }
static inline struct sec_cisd *sec_battery_cisd_init(struct sec_battery_info *battery)
{ return NULL; }
#endif

#if IS_ENABLED(CONFIG_SB_STARTUP)
int sec_bat_startup_init(struct sec_battery_info *battery);
#else
static inline int sec_bat_startup_init(struct sec_battery_info *battery)
{ return 0; }
#endif

#endif /* __SEC_BATTERY_H */
