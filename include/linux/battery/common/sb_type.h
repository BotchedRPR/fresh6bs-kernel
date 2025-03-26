/*
 * sb_type.h
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SB_COMMON_H
#define __SB_COMMON_H __FILE__

#include <linux/types.h>

struct sb_info {
	int voltage_now;
	int voltage_avg;
	int current_now;
	int current_avg;
	unsigned int capacity;

	int status;
	int health;
	int thermal_zone;
	bool store_mode;
	bool startup_state;
};

enum sec_battery_cable {
	SB_CBL_UNKNOWN = 0,	/* default */
	SB_CBL_NONE,
	SB_CBL_TA,
	SB_CBL_USB,
	SB_CBL_UARTOFF,
	/* 5 ~ 9 : RESERVED */
	SB_CBL_WRL	= 10,
	SB_CBL_INBOX_WRL,
	SB_CBL_D2D_WRL,
	SB_CBL_INCOMP_WRL,
	SB_CBL_ERR_WRL,
	SB_CBL_FM_WRL,
	SB_CBL_BPACK_WRL,
	SB_CBL_AUTH_WRL_2W,
	SB_CBL_AUTH_WRL_5W,
	SB_CBL_ERR_AUTH_WRL,

	SB_CBL_MAX
};

#define is_auth_wrl_type(cable_type) ( \
	cable_type == SB_CBL_AUTH_WRL_2W || \
	cable_type == SB_CBL_AUTH_WRL_5W)

#define is_inbox_wrl_type(cable_type) ( \
	cable_type == SB_CBL_INBOX_WRL)

#define is_d2d_wrl_type(cable_type) ( \
	cable_type == SB_CBL_D2D_WRL)

#define is_incomp_wrl_type(cable_type) ( \
	cable_type == SB_CBL_INCOMP_WRL || \
	cable_type == SB_CBL_ERR_WRL)

#define is_bpack_wrl_type(cable_type) ( \
	cable_type == SB_CBL_BPACK_WRL)

#define is_wireless_type(cable_type) ( \
	cable_type == SB_CBL_WRL || \
	cable_type == SB_CBL_FM_WRL || \
	cable_type == SB_CBL_ERR_AUTH_WRL || \
	is_inbox_wrl_type(cable_type) || \
	is_d2d_wrl_type(cable_type) || \
	is_incomp_wrl_type(cable_type) || \
	is_auth_wrl_type(cable_type) || \
	is_bpack_wrl_type(cable_type))

#define is_nocharge_type(cable_type) ( \
	cable_type == SB_CBL_NONE)

#define is_slate_mode(battery) ((battery->current_event & SEC_BAT_CURRENT_EVENT_SLATE) \
		== SEC_BAT_CURRENT_EVENT_SLATE)

#define is_ta_type(cable_type) ( \
	cable_type == SB_CBL_TA || \
	cable_type == SB_CBL_UARTOFF)

#define is_usb_type(cable_type) ( \
	cable_type == SB_CBL_USB)

#define is_wired_type(cable_type) ( \
	is_ta_type(cable_type) || is_usb_type(cable_type))

enum sec_battery_voltage_mode {
	/* average voltage */
	SEC_BATTERY_VOLTAGE_AVERAGE = 0,
	/* open circuit voltage */
	SEC_BATTERY_VOLTAGE_OCV,
};

enum sec_battery_current_mode {
	/* uA */
	SEC_BATTERY_CURRENT_UA = 0,
	/* mA */
	SEC_BATTERY_CURRENT_MA,
};

enum sec_battery_capacity_mode {
	/* designed capacity */
	SEC_BATTERY_CAPACITY_DESIGNED = 0,
	/* absolute capacity by fuel gauge */
	SEC_BATTERY_CAPACITY_ABSOLUTE,
	/* temperary capacity in the time */
	SEC_BATTERY_CAPACITY_TEMPERARY,
	/* current capacity now */
	SEC_BATTERY_CAPACITY_CURRENT,
	/* cell aging information */
	SEC_BATTERY_CAPACITY_AGEDCELL,
	/* charge count */
	SEC_BATTERY_CAPACITY_CYCLE,
	/* full capacity rep */
	SEC_BATTERY_CAPACITY_FULL,
	/* QH capacity */
	SEC_BATTERY_CAPACITY_QH,
	/* vfsoc */
	SEC_BATTERY_CAPACITY_VFSOC,
};

/* charging mode */
enum sec_battery_charging_mode {
	/* no charging */
	SEC_BATTERY_CHARGING_NONE = 0,
	/* 1st charging */
	SEC_BATTERY_CHARGING_1ST,
	/* 2nd charging */
	SEC_BATTERY_CHARGING_2ND,
	/* recharging */
	SEC_BATTERY_CHARGING_RECHARGING,
};

/* monitor activation */
enum sec_battery_polling_time_type {
	/* same order with power supply status */
	SEC_BATTERY_POLLING_TIME_BASIC = 0,
	SEC_BATTERY_POLLING_TIME_CHARGING,
	SEC_BATTERY_POLLING_TIME_DISCHARGING,
	SEC_BATTERY_POLLING_TIME_NOT_CHARGING,
	SEC_BATTERY_POLLING_TIME_SLEEP,
};

enum sec_battery_monitor_polling {
	/* polling work queue */
	SEC_BATTERY_MONITOR_WORKQUEUE,
	/* alarm polling */
	SEC_BATTERY_MONITOR_ALARM,
	/* timer polling (NOT USE) */
	SEC_BATTERY_MONITOR_TIMER,
};
#define sec_battery_monitor_polling_t \
	enum sec_battery_monitor_polling

/* full charged check : POWER_SUPPLY_PROP_STATUS */
enum sec_battery_full_charged {
	SEC_BATTERY_FULLCHARGED_NONE = 0,
	/* current check by ADC */
	SEC_BATTERY_FULLCHARGED_ADC,
	/* fuel gauge current check */
	SEC_BATTERY_FULLCHARGED_FG_CURRENT,
	/* time check */
	SEC_BATTERY_FULLCHARGED_TIME,
	/* SOC check */
	SEC_BATTERY_FULLCHARGED_SOC,
	/* charger GPIO, NO additional full condition */
	SEC_BATTERY_FULLCHARGED_CHGGPIO,
	/* charger interrupt, NO additional full condition */
	SEC_BATTERY_FULLCHARGED_CHGINT,
	/* charger power supply property, NO additional full condition */
	SEC_BATTERY_FULLCHARGED_CHGPSY,
};

#define sec_battery_full_charged_t \
	enum sec_battery_full_charged

/* full check condition type (can be used overlapped) */
#define sec_battery_full_condition_t unsigned int
/* SEC_BATTERY_FULL_CONDITION_NOTIMEFULL
 * full-charged by absolute-timer only in high voltage
 */
#define SEC_BATTERY_FULL_CONDITION_NOTIMEFULL	1
/* SEC_BATTERY_FULL_CONDITION_NOSLEEPINFULL
 * do not set polling time as sleep polling time in full-charged
 */
#define SEC_BATTERY_FULL_CONDITION_NOSLEEPINFULL	2
/* SEC_BATTERY_FULL_CONDITION_SOC
 * use capacity for full-charged check
 */
#define SEC_BATTERY_FULL_CONDITION_SOC		4
/* SEC_BATTERY_FULL_CONDITION_VCELL
 * use VCELL for full-charged check
 */
#define SEC_BATTERY_FULL_CONDITION_VCELL	8
/* SEC_BATTERY_FULL_CONDITION_AVGVCELL
 * use average VCELL for full-charged check
 */
#define SEC_BATTERY_FULL_CONDITION_AVGVCELL	16
/* SEC_BATTERY_FULL_CONDITION_OCV
 * use OCV for full-charged check
 */
#define SEC_BATTERY_FULL_CONDITION_OCV		32

/* recharge check condition type (can be used overlapped) */
#define sec_battery_recharge_condition_t unsigned int
/* SEC_BATTERY_RECHARGE_CONDITION_SOC
 * use capacity for recharging check
 */
#define SEC_BATTERY_RECHARGE_CONDITION_SOC		1
/* SEC_BATTERY_RECHARGE_CONDITION_AVGVCELL
 * use average VCELL for recharging check
 */
#define SEC_BATTERY_RECHARGE_CONDITION_AVGVCELL		2
/* SEC_BATTERY_RECHARGE_CONDITION_VCELL
 * use VCELL for recharging check
 */
#define SEC_BATTERY_RECHARGE_CONDITION_VCELL		4

/* battery check : POWER_SUPPLY_PROP_PRESENT */
enum sec_battery_check {
	/* No Check for internal battery */
	SEC_BATTERY_CHECK_NONE,
	/* by ADC */
	SEC_BATTERY_CHECK_ADC,
	/* by PMIC */
	SEC_BATTERY_CHECK_PMIC,
	/* by fuel gauge */
	SEC_BATTERY_CHECK_FUELGAUGE,
	/* by charger */
	SEC_BATTERY_CHECK_CHARGER,
};
#define sec_battery_check_t \
	enum sec_battery_check

/* OVP, UVLO check : POWER_SUPPLY_PROP_HEALTH */
enum sec_battery_ovp_uvlo {
	/* by PMIC polling */
	SEC_BATTERY_OVP_UVLO_PMICPOLLING,
	/* by PMIC interrupt */
	SEC_BATTERY_OVP_UVLO_PMICINT,
	/* by charger polling */
	SEC_BATTERY_OVP_UVLO_CHGPOLLING,
	/* by charger interrupt */
	SEC_BATTERY_OVP_UVLO_CHGINT,
};
#define sec_battery_ovp_uvlo_t \
	enum sec_battery_ovp_uvlo

/* temperature check type */
enum sec_battery_temp_check {
	SEC_BATTERY_TEMP_CHECK_NONE = 0,	/* no temperature check */
	SEC_BATTERY_TEMP_CHECK_ADC,	/* by ADC value */
	SEC_BATTERY_TEMP_CHECK_TEMP,	/* by temperature */
};
#define sec_battery_temp_check_t \
	enum sec_battery_temp_check

/* cable check (can be used overlapped) */
#define sec_battery_cable_check_t unsigned int
/* SEC_BATTERY_CABLE_CHECK_NOUSBCHARGE
 * for USB cable in tablet model,
 * status is stuck into discharging,
 * but internal charging logic is working
 */
#define SEC_BATTERY_CABLE_CHECK_NOUSBCHARGE		1
/* SEC_BATTERY_CABLE_CHECK_NOINCOMPATIBLECHARGE
 * for incompatible charger
 * (Not compliant to USB specification,
 * cable type is SEC_BATTERY_CABLE_UNKNOWN),
 * do NOT charge and show message to user
 * (only for VZW)
 */
#define SEC_BATTERY_CABLE_CHECK_NOINCOMPATIBLECHARGE	2
/* SEC_BATTERY_CABLE_CHECK_PSY
 * check cable by power supply set_property
 */
#define SEC_BATTERY_CABLE_CHECK_PSY			4
/* SEC_BATTERY_CABLE_CHECK_INT
 * check cable by interrupt
 */
#define SEC_BATTERY_CABLE_CHECK_INT			8
/* SEC_BATTERY_CABLE_CHECK_CHGINT
 * check cable by charger interrupt
 */
#define SEC_BATTERY_CABLE_CHECK_CHGINT			16
/* SEC_BATTERY_CABLE_CHECK_POLLING
 * check cable by GPIO polling
 */
#define SEC_BATTERY_CABLE_CHECK_POLLING			32

/* check cable source (can be used overlapped) */
#define sec_battery_cable_source_t unsigned int
/* SEC_BATTERY_CABLE_SOURCE_EXTERNAL
 * already given by external argument
 */
#define	SEC_BATTERY_CABLE_SOURCE_EXTERNAL	1
/* SEC_BATTERY_CABLE_SOURCE_ADC
 * by ADC
 */
#define	SEC_BATTERY_CABLE_SOURCE_ADC		4

/* capacity calculation type (can be used overlapped) */
#define sec_fuelgauge_capacity_type_t int
/* SEC_FUELGAUGE_CAPACITY_TYPE_RESET
 * use capacity information to reset fuel gauge
 * (only for driver algorithm, can NOT be set by user)
 */
#define SEC_FUELGAUGE_CAPACITY_TYPE_RESET	(-1)
/* SEC_FUELGAUGE_CAPACITY_TYPE_RAW
 * use capacity information from fuel gauge directly
 */
#define SEC_FUELGAUGE_CAPACITY_TYPE_RAW		1
/* SEC_FUELGAUGE_CAPACITY_TYPE_SCALE
 * rescale capacity by scaling, need min and max value for scaling
 */
#define SEC_FUELGAUGE_CAPACITY_TYPE_SCALE	2
/* SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE
 * change only maximum capacity dynamically
 * to keep time for every SOC unit
 */
#define SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE	4
/* SEC_FUELGAUGE_CAPACITY_TYPE_ATOMIC
 * change capacity value by only -1 or +1
 * no sudden change of capacity
 */
#define SEC_FUELGAUGE_CAPACITY_TYPE_ATOMIC	8
/* SEC_FUELGAUGE_CAPACITY_TYPE_SKIP_ABNORMAL
 * skip current capacity value
 * if it is abnormal value
 */
#define SEC_FUELGAUGE_CAPACITY_TYPE_SKIP_ABNORMAL	16

#endif /* __SB_COMMON_H */
