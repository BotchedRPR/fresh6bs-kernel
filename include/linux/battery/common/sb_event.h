/*
 * sb_event.h
 * Samsung Mobile Battery Event Header
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

#ifndef __SB_EVENT_H
#define __SB_EVENT_H __FILE__

#define SEC_BAT_CURRENT_EVENT_NONE					0x000000
#define SEC_BAT_CURRENT_EVENT_AFC					0x000001
#define SEC_BAT_CURRENT_EVENT_CHARGE_DISABLE		0x000002
#define SEC_BAT_CURRENT_EVENT_SKIP_HEATING_CONTROL	0x000004
#define SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL1	0x000008
#define SEC_BAT_CURRENT_EVENT_HIGH_TEMP_SWELLING		0x000020
#define SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL2	0x000080
#define SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL3	0x000010
#define SEC_BAT_CURRENT_EVENT_SWELLING_MODE		(SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL1 | SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL2 | SEC_BAT_CURRENT_EVENT_HIGH_TEMP_SWELLING | SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL3)
#define SEC_BAT_CURRENT_EVENT_LOW_TEMP_MODE		(SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL1 | SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL2 | SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL3)
#define SEC_BAT_CURRENT_EVENT_CHG_LIMIT			0x000200
#define SEC_BAT_CURRENT_EVENT_CALL				0x000400
#define SEC_BAT_CURRENT_EVENT_SLATE				0x000800
#define SEC_BAT_CURRENT_EVENT_VBAT_OVP			0x001000
#define SEC_BAT_CURRENT_EVENT_VSYS_OVP			0x002000
#define SEC_BAT_CURRENT_EVENT_WPC_VOUT_LOCK		0x004000
#define SEC_BAT_CURRENT_EVENT_AICL				0x008000
#define SEC_BAT_CURRENT_EVENT_HV_DISABLE		0x010000
#define SEC_BAT_CURRENT_EVENT_SELECT_PDO		0x020000
/* #define SEC_BAT_CURRENT_EVENT_FG_RESET			0x040000 */
#define SEC_BAT_CURRENT_EVENT_LOW_CURRENT_TCT	0x040000

#define SEC_BAT_CURRENT_EVENT_DC_ERR			0x400000
#define SEC_BAT_CURRENT_EVENT_SIOP_LIMIT		0x800000
#define SEC_BAT_CURRENT_EVENT_TEMP_CTRL_TEST	0x1000000
#define SEC_BAT_CURRENT_EVENT_25W_OCP			0x2000000
#define SEC_BAT_CURRENT_EVENT_AFC_DISABLE		0x4000000
#define SEC_BAT_CURRENT_EVENT_SEND_UVDM			0x8000000

#define SEC_BAT_CURRENT_EVENT_USB_SUSPENDED		0x10000000
#define SEC_BAT_CURRENT_EVENT_USB_SUPER		0x20000000
#if IS_ENABLED(CONFIG_ENABLE_100MA_CHARGING_BEFORE_USB_CONFIGURED)
#define SEC_BAT_CURRENT_EVENT_USB_100MA		0x40000000
#else
#define SEC_BAT_CURRENT_EVENT_USB_100MA		0x00000000
#endif
#if IS_ENABLED(CONFIG_DISABLE_MFC_IC)
#define SEC_BAT_CURRENT_EVENT_WPC_EN		0x80000000
#else
#define SEC_BAT_CURRENT_EVENT_WPC_EN		0x00000000
#endif

#if IS_ENABLED(CONFIG_SEC_FACTORY)	// SEC_FACTORY
#define STORE_MODE_CHARGING_MAX 80
#define STORE_MODE_CHARGING_MIN 70
#else	// !SEC_FACTORY, STORE MODE
#define STORE_MODE_CHARGING_MAX 50
#define STORE_MODE_CHARGING_MIN 40
#define STORE_MODE_CHARGING_MAX_VZW 35
#define STORE_MODE_CHARGING_MIN_VZW 30
#endif //(CONFIG_SEC_FACTORY)

/* Not used in Watch.
#define BATT_MISC_EVENT_UNDEFINED_RANGE_TYPE	0x00000001
#define BATT_MISC_EVENT_WIRELESS_BACKPACK_TYPE	0x00000002
#define BATT_MISC_EVENT_TIMEOUT_OPEN_TYPE		0x00000004
*/
#define BATT_MISC_EVENT_FOTA_DOWNLOAD			0x00000001
#define BATT_MISC_EVENT_BATT_RESET_SOC			0x00000008
#define BATT_MISC_EVENT_WATER_HICCUP_TYPE		0x00000020
#define BATT_MISC_EVENT_WIRELESS_DET_LEVEL		0x00000040
#define BATT_MISC_EVENT_WIRELESS_FW_UPDATE		0x00000080
#define BATT_MISC_EVENT_WIRELESS_FOD			0x00000100
/*
#define BATT_MISC_EVENT_WIRELESS_AUTH_START		0x00000200
#define BATT_MISC_EVENT_WIRELESS_AUTH_RECVED	0x00000400
#define BATT_MISC_EVENT_WIRELESS_AUTH_FAIL		0x00000800
#define BATT_MISC_EVENT_WIRELESS_AUTH_PASS		0x00001000
*/
/* Move auth event to sb_wrl_data.h */
#define BATT_MISC_EVENT_WIRELESS_AUTH_SHIFT		9
#define BATT_MISC_EVENT_WIRELESS_AUTH_MASK		(0xF << BATT_MISC_EVENT_WIRELESS_AUTH_SHIFT)

#define BATT_MISC_EVENT_TEMP_HICCUP_TYPE		0x00002000
#define BATT_MISC_EVENT_BATTERY_HEALTH			0x000F0000
#define BATT_MISC_EVENT_HEALTH_OVERHEATLIMIT	0x00100000
#define BATT_MISC_EVENT_ABNORMAL_PAD			0x00200000
#define BATT_MISC_EVENT_WIRELESS_MISALIGN		0x00400000
#define BATT_MISC_EVENT_WIRELESS_INCOMPATIBLE	0x00800000
#define BATT_MISC_EVENT_WIRELESS_JIG_PAD		0x20000000

#define BATTERY_HEALTH_SHIFT	16
enum misc_battery_health {
	BATTERY_HEALTH_UNKNOWN = 0,
	BATTERY_HEALTH_GOOD,
	BATTERY_HEALTH_NORMAL,
	BATTERY_HEALTH_AGED,
	BATTERY_HEALTH_MAX = BATTERY_HEALTH_AGED,

	/* For event */
	BATTERY_HEALTH_BAD = 0xF,
};
#define BATT_MISC_EVENT_MUIC_ABNORMAL	(BATT_MISC_EVENT_UNDEFINED_RANGE_TYPE |\
					BATT_MISC_EVENT_WATER_HICCUP_TYPE |\
					BATT_MISC_EVENT_TEMP_HICCUP_TYPE)

#define BATT_MISC_EVENT_RESET_AFTER_READ_MASK	\
	(BATT_MISC_EVENT_WIRELESS_INCOMPATIBLE)

enum {
	BAT_THERMAL_COLD = 0,
	BAT_THERMAL_COOL3,
	BAT_THERMAL_COOL2,
	BAT_THERMAL_COOL1,
	BAT_THERMAL_NORMAL,
	BAT_THERMAL_WARM,
	BAT_THERMAL_OVERHEAT,
	BAT_THERMAL_OVERHEATLIMIT,
};

#endif /* __SB_EVENT_H */
