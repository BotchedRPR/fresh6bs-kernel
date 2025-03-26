/*
 *  sec_startup.c
 *  Samsung Mobile Battery Startup sub Module
 *
 *  Copyright (C) 2023 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/battery/common/sb_ext.h>
#include <linux/battery/common/sb_vote_event.h>
#include <linux/battery/module/sb_startup.h>

#include "sec_battery.h"

#if IS_ENABLED(CONFIG_SEC_FACTORY)
static bool sup_check_prevent_overheat(void)
{
	return false;
}
#else
static bool sup_check_prevent_overheat(void)
{
	if (sb_get_lpcharge())
		return false;

	return true;
}
#endif

static int sup_event(void *pdata, unsigned int event, sb_data time)
{
	struct sec_battery_info *battery = pdata;
	struct device_node *np = battery->dev->of_node;
	int ret = 0, temp = 450;

	pr_info("%s: battery - time = %d\n", __func__, (int)time);

	/* parse dt */
	ret = of_property_read_u32(np, "battery,wireless_warm_overheat_thresh",
		(unsigned int *)&temp);
	if (!ret)
		pr_info("%s: recover wireless_warm_overheat_thresh(%d --> %d)\n",
			__func__, battery->pdata->wireless_warm_overheat_thresh, temp);
	battery->pdata->wireless_warm_overheat_thresh = temp;

	/* clear vote */
	battery->startup_state = false;
	sb_vote_set(battery->fcc_vote, VOTER_STARTUP, false, 0);

	/* reset chg */
	sec_bat_set_threshold(battery);
	sec_bat_set_charging_current(battery);
	return 0;
}

int sec_bat_startup_init(struct sec_battery_info *battery)
{
	struct device_node *np = battery->dev->of_node;
	/* default sup_time = 5 min, chg_cur = 175mA */
	unsigned int sup_time = 300, sup_chg_cur = 175;
	int ret = 0;

	if (!sb_startup_is_activated()) {
		pr_info("%s: startup is not activated!\n", __func__);
		return 0;
	}

	/* parse dt */
	of_property_read_u32(np, "sup_chg_cur", &sup_chg_cur);
	if (sup_check_prevent_overheat()) {
		of_property_read_u32(np, "sup_time", &sup_time);
		of_property_read_u32(np, "battery,wireless_prevent_overheat_thresh",
			(unsigned int *)&battery->pdata->wireless_warm_overheat_thresh);
	} else {
		sup_time = 60; /* 1 min */
	}

	/* set fcc vote */
	sb_vote_set_pri(battery->fcc_vote, VOTER_STARTUP, VOTE_PRI_9);
	sb_vote_set(battery->fcc_vote, VOTER_STARTUP, true, sup_chg_cur);

	/* set startup */
	ret = sb_startup_register(battery, sup_event, 0, sup_time);
	battery->startup_state = (!ret);

	pr_info("%s: ret = %d, startup = %s, chg_cur = %d, sup_time = %d, thre = %d\n",
		__func__,
		ret,
		GET_BOOL_STR(battery->startup_state),
		sup_chg_cur, sup_time,
		battery->pdata->wireless_warm_overheat_thresh);
	return ret;
}
EXPORT_SYMBOL(sec_bat_startup_init);
