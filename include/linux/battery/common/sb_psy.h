/*
 * sb_psy.h
 * Samsung Mobile Battery Common Header
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

#ifndef __SB_PSY_H
#define __SB_PSY_H __FILE__

#include <linux/power_supply.h>

enum power_supply_ext_property {
	POWER_SUPPLY_EXT_PROP_CHARGING_ENABLED = 1000,

	POWER_SUPPLY_EXT_PROP_CHECK_SLAVE_I2C,
	POWER_SUPPLY_EXT_PROP_MULTI_CHARGER_MODE,
	POWER_SUPPLY_EXT_PROP_WIRELESS_OP_FREQ,
	POWER_SUPPLY_EXT_PROP_WIRELESS_TX_CMD,
	POWER_SUPPLY_EXT_PROP_WIRELESS_TX_VAL,
	POWER_SUPPLY_EXT_PROP_WIRELESS_TX_ID,
	POWER_SUPPLY_EXT_PROP_WIRELESS_TX_UNO_IIN,
	POWER_SUPPLY_EXT_PROP_WIRELESS_PHM,
	POWER_SUPPLY_EXT_PROP_WIRELESS_FREQ_STRENGTH,
	POWER_SUPPLY_EXT_PROP_AICL_CURRENT,
	POWER_SUPPLY_EXT_PROP_CHECK_MULTI_CHARGE,
	POWER_SUPPLY_EXT_PROP_CHIP_ID,
	POWER_SUPPLY_EXT_PROP_SYSOVLO,
	POWER_SUPPLY_EXT_PROP_VBAT_OVP,
	POWER_SUPPLY_EXT_PROP_FGSRC_SWITCHING,
	POWER_SUPPLY_EXT_PROP_FG_INBAT_VOLTAGE,

	POWER_SUPPLY_EXT_PROP_WATER_DETECT,
	POWER_SUPPLY_EXT_PROP_SURGE,

	POWER_SUPPLY_EXT_PROP_CHARGE_POWER,
	POWER_SUPPLY_EXT_PROP_MEASURE_SYS,
	POWER_SUPPLY_EXT_PROP_MEASURE_INPUT,

	POWER_SUPPLY_EXT_PROP_WC_CONTROL,
	POWER_SUPPLY_EXT_PROP_CHGINSEL,

	POWER_SUPPLY_EXT_PROP_MONITOR_WORK,
	POWER_SUPPLY_EXT_PROP_WDT_KICK,
	POWER_SUPPLY_EXT_PROP_WIRELESS_CHARGE_MONITOR,

	POWER_SUPPLY_EXT_PROP_UPDATE_BATTERY_DATA,
	POWER_SUPPLY_EXT_PROP_SLATE_MODE,
	POWER_SUPPLY_EXT_PROP_CHG_IN_OK,
	POWER_SUPPLY_EXT_PROP_FORCED_JIG_MODE,
	POWER_SUPPLY_EXT_PROP_STORE_MODE,
	POWER_SUPPLY_EXT_PROP_LONG_LIFE_STEP,
	POWER_SUPPLY_EXT_PROP_POWER_SHARING_CHARGE,
	POWER_SUPPLY_EXT_PROP_WIRELESS_OP_FREQ_STRENGTH,
	POWER_SUPPLY_EXT_PROP_CHECK_DROP_CV,
	POWER_SUPPLY_EXT_PROP_FACTORY_MODE,
	POWER_SUPPLY_EXT_PROP_MUIC_MSG,
	POWER_SUPPLY_EXT_PROP_TX_PWR_BUDG,
	POWER_SUPPLY_EXT_PROP_WIRELESS_AUTH_ADT_STATUS,

	POWER_SUPPLY_EXT_PROP_INPUT_VOLTAGE_REGULATION,
	POWER_SUPPLY_EXT_PROP_CHARGE_OTG_CONTROL,
	POWER_SUPPLY_EXT_PROP_CHARGE_POWERED_OTG_CONTROL,
	POWER_SUPPLY_EXT_PROP_SHIPMODE_TEST,
};

enum power_supply_muic_msg {
	POWER_SUPPLY_MUIC_UNKNOWN = 0,
	POWER_SUPPLY_MUIC_ACOK_FALLING,
	POWER_SUPPLY_MUIC_ACOK_RISING,
};

#define ACOK_NO_INPUT			0
#define ACOK_INPUT				1

enum power_supply_ext_health {
	POWER_SUPPLY_EXT_HEALTH_UNDERVOLTAGE = 20,
	POWER_SUPPLY_EXT_HEALTH_OVERHEATLIMIT,

	POWER_SUPPLY_EXT_HEALTH_VSYS_OVP,
	POWER_SUPPLY_EXT_HEALTH_VBAT_OVP,

	POWER_SUPPLY_EXT_HEALTH_MAX,
};

static inline struct power_supply *get_power_supply_by_name(char *name)
{
	if (!name)
		return (struct power_supply *)NULL;
	else
		return power_supply_get_by_name(name);
}

#define psy_do_property(name, function, property, value) \
({	\
	struct power_supply *psy;	\
	int ret = 0;	\
	psy = get_power_supply_by_name((name));	\
	if (!psy) {	\
		pr_err("%s: Fail to "#function" psy (%s)\n",	\
			__func__, (name));	\
		value.intval = 0;	\
		ret = -ENOENT;	\
	} else {	\
		if (psy->desc->function##_property != NULL) { \
			ret = psy->desc->function##_property(psy, \
				(enum power_supply_property) (property), &(value)); \
			if (ret < 0) {	\
				pr_err("%s: Fail to %s "#function" "#property" (%d)\n", \
						__func__, name, ret);	\
				value.intval = 0;	\
			}	\
		} else {	\
			ret = -ENOSYS;	\
		}	\
		power_supply_put(psy);	\
	}	\
	ret;	\
})

static inline bool psy_get_online(char *name)
{
	union power_supply_propval value = { 0, };
	int ret = 0;

	ret = psy_do_property(name, get, POWER_SUPPLY_PROP_ONLINE, value);
	return (ret < 0) ? false : !!value.intval;
}

#endif /* __SB_PSY_H */
