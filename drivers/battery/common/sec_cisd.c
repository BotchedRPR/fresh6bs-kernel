/*
 *  sec_cisd.c
 *  Samsung Mobile Battery CISD sub Module
 *
 *  Copyright (C) 2022 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/module.h>

#include <linux/battery/sb_sysfs.h>

#include <linux/battery/common/sb_ext.h>
#include <linux/battery/common/sb_psy.h>
#include <linux/battery/common/sb_type.h>
#include <linux/battery/common/sb_vote_event.h>

#include <linux/battery/module/sb_adc.h>
#include <linux/battery/module/sb_cisd.h>

#include "sec_battery.h"

struct sec_cisd {
	/* state */
	bool state;
	int prev_volt;
	int prev_temp;
	int prev_jig_on;
	int enable_update_data;
	int prev_chg_on;

	/* abnormal vbat */
	int max_voltage_thr;
	unsigned int ab_vbat_max_cnt;
	unsigned int ab_vbat_check_cnt;
};

static void sec_bat_cisd_check_ab_voltage(struct sec_cisd *cisd, int voltage)
{
	union power_supply_propval value = { true, };
	static bool is_protected;

	if (is_protected)
		return;

	if (voltage <= cisd->max_voltage_thr)
		return;

	if ((++cisd->ab_vbat_check_cnt) >= cisd->ab_vbat_max_cnt) {
		is_protected = true;
		psy_do_property("battery", set, POWER_SUPPLY_EXT_PROP_VBAT_OVP, value);
	}
}

int sec_bat_set_cisd_state(struct sec_cisd *cisd, bool state)
{
	if (cisd == NULL)
		return -EINVAL;

	cisd->state = state;

	return 0;
}
EXPORT_SYMBOL(sec_bat_set_cisd_state);

int sec_bat_cisd_check(struct sec_battery_info *battery)
{
	struct sec_cisd *cisd = battery->cisd;
	int batt_temp, temp = 0;

	if (cisd == NULL)
		return -EINVAL;

	if (battery->factory_mode || battery->is_jig_on || !cisd->state) {
		dev_dbg(battery->dev, "%s: No need to check in factory mode\n",
			__func__);
		return 0;
	}

	/* vbat */
	sec_bat_cisd_check_ab_voltage(cisd, battery->voltage_now);

	/* temperature */
	sb_adc_get_valuef(BATT_TEMP, &batt_temp);
	if ((sb_cisd_get_data(BATT_THM_MAX, &temp) < 0) ||
		(batt_temp > temp))
		sb_cisd_set_data(BATT_THM_MAX, batt_temp);
	if ((sb_cisd_get_data(BATT_THM_MAX_D, &temp) < 0) ||
		(batt_temp > temp))
		sb_cisd_set_data(BATT_THM_MAX_D, batt_temp);
	if ((sb_cisd_get_data(BATT_THM_MIN, &temp) < 0) ||
		(batt_temp < temp))
		sb_cisd_set_data(BATT_THM_MIN, batt_temp);
	if ((sb_cisd_get_data(BATT_THM_MIN_D, &temp) < 0) ||
		(batt_temp < temp))
		sb_cisd_set_data(BATT_THM_MIN_D, batt_temp);

	return 0;
}
EXPORT_SYMBOL(sec_bat_cisd_check);

static int parse_dt(struct device_node *np, struct sec_cisd *cisd)
{
	sb_of_parse_u32(np, cisd, max_voltage_thr, 5000);
	sb_of_parse_u32(np, cisd, ab_vbat_max_cnt, 2);
	return 0;
}

struct sec_cisd *sec_battery_cisd_init(struct sec_battery_info *battery)
{
	struct sec_cisd *cisd = NULL;

	cisd = kzalloc(sizeof(struct sec_cisd), GFP_KERNEL);
	if (!cisd)
		return NULL;

	if (parse_dt(battery->dev->of_node, cisd)) {
		kfree(cisd);
		return NULL;
	}

	cisd->state = true;
	cisd->ab_vbat_check_cnt = 0;
	return cisd;
}
EXPORT_SYMBOL(sec_battery_cisd_init);

static ssize_t sysfs_cisd_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t sysfs_cisd_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

#define SYSFS_CISD_ATTR(_name)						\
{									\
	.attr = {.name = #_name, .mode = 0664},	\
	.show = sysfs_cisd_show_attrs,					\
	.store = sysfs_cisd_store_attrs,					\
}

static struct device_attribute sysfs_cisd_attrs[] = {
	SYSFS_CISD_ATTR(cisd_fullcaprep_max),
	SYSFS_CISD_ATTR(prev_battery_data),
	SYSFS_CISD_ATTR(prev_battery_info),
};

enum {
	CISD_FULLCAPREP_MAX = 0,
	PREV_BATTERY_DATA,
	PREV_BATTERY_INFO,
};

static ssize_t sysfs_cisd_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct sec_battery_info *battery = power_supply_get_drvdata(psy);
	struct sec_cisd *cisd = battery->cisd;
	const ptrdiff_t offset = attr - sysfs_cisd_attrs;
	int i = 0;

	switch (offset) {
	case CISD_FULLCAPREP_MAX:
	{
		union power_supply_propval value = { 0, };

		value.intval = SEC_BATTERY_CAPACITY_FULL;
		psy_do_property(battery->pdata->fuelgauge_name, get,
			POWER_SUPPLY_PROP_ENERGY_NOW, value);

		i += scnprintf(buf, PAGE_SIZE, "%d\n", value.intval);
	}
		break;
	case PREV_BATTERY_DATA:
	{
		int batt_temp = 0;

		sb_adc_get_valuef(BATT_TEMP, &batt_temp);
		if (cisd->enable_update_data)
			i += scnprintf(buf, PAGE_SIZE, "%d, %d, %d, %d\n",
				battery->voltage_now, batt_temp, battery->is_jig_on,
				(battery->charger_mode == SB_CHG_MODE_CHARGING) ? 1 : 0);
	}
		break;
	case PREV_BATTERY_INFO:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d,%d,%d,%d\n",
			cisd->prev_volt, cisd->prev_temp,
			cisd->prev_jig_on, cisd->prev_chg_on);
		pr_info("%s: Read Prev Battery Info : %s", __func__, buf);
		break;
	default:
		return -EINVAL;
	}

	return i;
}

static ssize_t sysfs_cisd_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct sec_battery_info *battery = power_supply_get_drvdata(psy);
	struct sec_cisd *cisd = battery->cisd;
	const ptrdiff_t offset = attr - sysfs_cisd_attrs;

	switch (offset) {
	case CISD_FULLCAPREP_MAX:
		break;
	case PREV_BATTERY_DATA:
		if (sscanf(buf, "%10d, %10d, %10d, %10d\n",
				&cisd->prev_volt, &cisd->prev_temp,
				&cisd->prev_jig_on, &cisd->prev_chg_on) >= 4) {
			pr_info("%s: prev voltage : %d, prev_temp : %d, prev_jig_on : %d, prev_chg_on : %d\n",
				__func__, cisd->prev_volt, cisd->prev_temp,
				cisd->prev_jig_on, cisd->prev_chg_on);

			if (cisd->prev_volt >= 3700 && cisd->prev_temp >= 150 &&
					!cisd->prev_jig_on && battery->fg_reset)
				pr_info("%s: Battery have been Removed\n", __func__);
		}
		cisd->enable_update_data = 1;
		break;
	case PREV_BATTERY_INFO:
		break;

	default:
		return -EINVAL;
	}

	return count;
}

static int __init sec_cisd_init(void)
{
	return sb_sysfs_add_attrs("sec_cisd", PSY_BATTERY_NAME,
		sysfs_cisd_attrs, ARRAY_SIZE(sysfs_cisd_attrs));
}
module_init(sec_cisd_init);

MODULE_DESCRIPTION("Samsung Battery CISD Sub");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
