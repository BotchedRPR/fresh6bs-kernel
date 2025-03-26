/*
 * sb_cisd.h
 * Samsung Mobile Battery CISD Header
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

#ifndef __SB_CISD_H
#define __SB_CISD_H __FILE__

#include <linux/err.h>

#define FOREACH_CISD(GEN_CISD)	{ \
	GEN_CISD(RESET_ALG),    \
	GEN_CISD(ALG_INDEX),    \
	GEN_CISD(FULL_CNT),    \
	GEN_CISD(CAP_MAX),    \
	GEN_CISD(CAP_MIN),    \
	GEN_CISD(RECHARGING_CNT),    \
	GEN_CISD(VALERT_CNT),    \
	GEN_CISD(BATT_CYCLE),    \
	GEN_CISD(WIRE_CNT),    \
	GEN_CISD(WIRELESS_CNT),    \
	GEN_CISD(HIGH_SWELLING_CNT),    \
	GEN_CISD(LOW_SWELLING_CNT),    \
	GEN_CISD(WC_HIGH_SWELLING_CNT),    \
	GEN_CISD(SWELLING_FULL_CNT),    \
	GEN_CISD(SWELLING_RECOVERY_CNT),    \
	GEN_CISD(AICL_CNT),	\
	GEN_CISD(BATT_THM_MAX),	\
	GEN_CISD(BATT_THM_MIN),	\
	GEN_CISD(CHG_THM_MAX),	\
	GEN_CISD(CHG_THM_MIN),	\
	GEN_CISD(WPC_THM_MAX),	\
	GEN_CISD(WPC_THM_MIN),	\
	GEN_CISD(USB_THM_MAX),    \
	GEN_CISD(USB_THM_MIN),	\
	GEN_CISD(CHG_BATT_THM_MAX),	\
	GEN_CISD(CHG_BATT_THM_MIN),	\
	GEN_CISD(CHG_CHG_THM_MAX),	\
	GEN_CISD(CHG_CHG_THM_MIN),	\
	GEN_CISD(CHG_WPC_THM_MAX),	\
	GEN_CISD(CHG_WPC_THM_MIN),	\
	GEN_CISD(CHG_USB_THM_MAX),	\
	GEN_CISD(CHG_USB_THM_MIN),	\
	GEN_CISD(USB_OVERHEAT_CHARGING),	\
	GEN_CISD(UNSAFETY_VOLT),	\
	GEN_CISD(UNSAFETY_TEMP),	\
	GEN_CISD(SAFETY_TIMER),	\
	GEN_CISD(VSYS_OVP),	\
	GEN_CISD(VBAT_OVP),    \
	GEN_CISD(USB_OVERHEAT_RAPID_CHANGE),	\
	GEN_CISD(ASOC),	\
	GEN_CISD(USB_OVERHEAT_ALONE),	\
	GEN_CISD(CAP_NOM),	\
	GEN_CISD(RC0),	\
	GEN_CISD(FULL_CNT_D),    \
	GEN_CISD(CAP_MAX_D),    \
	GEN_CISD(CAP_MIN_D),    \
	GEN_CISD(RECHARGING_CNT_D),    \
	GEN_CISD(VALERT_CNT_D),    \
	GEN_CISD(WIRE_CNT_D),    \
	GEN_CISD(WIRELESS_CNT_D),    \
	GEN_CISD(HIGH_SWELLING_CNT_D),    \
	GEN_CISD(LOW_SWELLING_CNT_D),    \
	GEN_CISD(WC_HIGH_SWELLING_CNT_D),    \
	GEN_CISD(SWELLING_FULL_CNT_D),    \
	GEN_CISD(SWELLING_RECOVERY_CNT_D),    \
	GEN_CISD(AICL_CNT_D),    \
	GEN_CISD(BATT_THM_MAX_D),    \
	GEN_CISD(BATT_THM_MIN_D),    \
	GEN_CISD(SUB_BATT_THM_MAX_D),	\
	GEN_CISD(SUB_BATT_THM_MIN_D),	\
	GEN_CISD(CHG_THM_MAX_D),	\
	GEN_CISD(CHG_THM_MIN_D),	\
	GEN_CISD(USB_THM_MAX_D),	\
	GEN_CISD(USB_THM_MIN_D),	\
	GEN_CISD(CHG_BATT_THM_MAX_D),	\
	GEN_CISD(CHG_BATT_THM_MIN_D),    \
	GEN_CISD(CHG_SUB_BATT_THM_MAX_D),	\
	GEN_CISD(CHG_SUB_BATT_THM_MIN_D),	\
	GEN_CISD(CHG_CHG_THM_MAX_D),	\
	GEN_CISD(CHG_CHG_THM_MIN_D),	\
	GEN_CISD(CHG_USB_THM_MAX_D),	\
	GEN_CISD(CHG_USB_THM_MIN_D),	\
	GEN_CISD(USB_OVERHEAT_CHARGING_D),	\
	GEN_CISD(UNSAFETY_VOLT_D),	\
	GEN_CISD(UNSAFETY_TEMP_D),	\
	GEN_CISD(SAFETY_TIMER_D),	\
	GEN_CISD(VSYS_OVP_D),	\
	GEN_CISD(VBAT_OVP_D),	\
	GEN_CISD(USB_OVERHEAT_RAPID_CHANGE_D),	\
	GEN_CISD(USB_OVERHEAT_ALONE_D),    \
	GEN_CISD(DROP_SENSOR_D),	\
	GEN_CISD(CHG_TIME_D),	\
	GEN_CISD(TOTAL_CHG_TIME_D),	\
	GEN_CISD(CISD_MAX)	\
}

#define GEN_CISD_ENUM(ENUM) ENUM
#define GEN_CISD_STRING(STRING)	#STRING

enum CISD_ENUM
FOREACH_CISD(GEN_CISD_ENUM);

#define SB_CISD_DISABLE	(-3802)
#if IS_ENABLED(CONFIG_SB_CISD)
int sb_cisd_set_data(enum CISD_ENUM type, int value);
int sb_cisd_get_data(enum CISD_ENUM type, int *value);

int sb_cisd_count(unsigned int count, ...);
int sb_cisd_count_pad(unsigned int id);
#else
static inline int sb_cisd_set_data(enum CISD_ENUM type, int value)
{ return SB_CISD_DISABLE; }
static inline int sb_cisd_get_data(enum CISD_ENUM type, int *value)
{ return SB_CISD_DISABLE; }

static inline int sb_cisd_count(unsigned int count, ...)
{ return SB_CISD_DISABLE; }
static inline int sb_cisd_count_pad(unsigned int id)
{ return SB_CISD_DISABLE; }
#endif

#endif /* __SB_CISD_H */

