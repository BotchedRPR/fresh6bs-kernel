/*
 *  sysfs_fota.c
 *  Samsung Mobile Battery sysfs fota Module
 *
 *  Copyright (C) 2023 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/alarmtimer.h>
#include <linux/slab.h>

#include <linux/battery/sb_vote.h>
#include <linux/battery/sb_sysfs.h>
#include <linux/battery/common/sb_psy.h>
#include <linux/battery/common/sb_ext.h>
#include <linux/battery/common/sb_event.h>
#include <linux/battery/common/sb_vote_event.h>
#include <linux/battery/wireless/sb_wrl.h>
#include <linux/battery/module/sb_startup.h>

#include "sec_battery.h"

struct sysfs_fota {
	struct sec_battery_info *battery;

	struct alarm fota_alarm;
	struct wakeup_source *ws;
	struct delayed_work fota_work;
};

static enum alarmtimer_restart fota_alarm(struct alarm *alarm, ktime_t now)
{
	struct sysfs_fota *fota = container_of(alarm, struct sysfs_fota, fota_alarm);

	pr_info("%s\n", __func__);

	__pm_stay_awake(fota->ws);
	schedule_delayed_work(&fota->fota_work, 0);
	return ALARMTIMER_NORESTART;
}

static void fota_work(struct work_struct *work)
{
	struct sysfs_fota *fota = container_of(work, struct sysfs_fota, fota_work.work);
	struct sec_battery_info *battery = fota->battery;

	pr_info("%s\n", __func__);

	sb_vote_set(battery->chgen_vote, VOTER_FOTA_DOWNLOAD, false, 0);
	sb_wrl_set_vm_phm(VOTER_FOTA_DOWNLOAD, false, WPC_PHM_OFF);
	sec_bat_set_misc_event(battery, 0, BATT_MISC_EVENT_FOTA_DOWNLOAD);

	__pm_relax(fota->ws);
}

static struct sysfs_fota *fota_inst(struct sec_battery_info *battery)
{
	static struct sysfs_fota *fota;

	if (fota)
		return fota;

	if (battery == NULL)
		return ERR_PTR(-EINVAL);

	fota = kzalloc(sizeof(struct sysfs_fota), GFP_KERNEL);
	if (fota == NULL)
		goto err_alloc;

	fota->ws = wakeup_source_register(NULL, "sysfs_fota");
	if (!fota->ws)
		goto err_ws;

	alarm_init(&fota->fota_alarm, ALARM_BOOTTIME, fota_alarm);
	INIT_DELAYED_WORK(&fota->fota_work, fota_work);

	fota->battery = battery;
	return fota;

err_ws:
	kfree(fota);
err_alloc:
	fota = ERR_PTR(-ENOMEM);
	return fota;
}

static void start_fota_alarm(struct sysfs_fota *fota)
{
	ktime_t now_boot, target_time;

	if (IS_ERR(fota))
		return;

	now_boot = ktime_get_boottime();
	target_time = ktime_add(now_boot, ktime_set((10 * 60), 0));
	pr_info("%s: time = %lld, %lld\n", __func__, now_boot, target_time);

	alarm_start(&fota->fota_alarm, target_time);
}

static int cancel_fota_alarm(struct sysfs_fota *fota)
{
	if (IS_ERR(fota))
		return -ENOMEM;

	return (alarm_try_to_cancel(&fota->fota_alarm) >= 0);
}

static int set_fota_event(struct sec_battery_info *battery, int value)
{
	struct sysfs_fota *fota = fota_inst(battery);

	pr_info("%s: fota download = %d\n", __func__, value);

	if (value) {
		sec_bat_set_misc_event(battery,
			BATT_MISC_EVENT_FOTA_DOWNLOAD, BATT_MISC_EVENT_FOTA_DOWNLOAD);
		sb_wrl_set_vm_phm(VOTER_FOTA_DOWNLOAD, true, WPC_PHM_ON);
		sb_vote_set(battery->chgen_vote, VOTER_FOTA_DOWNLOAD, true, SB_CHG_MODE_CHARGING_OFF);

		start_fota_alarm(fota);
	} else {
		cancel_fota_alarm(fota);

		sb_vote_set(battery->chgen_vote, VOTER_FOTA_DOWNLOAD, false, 0);
		sb_wrl_set_vm_phm(VOTER_FOTA_DOWNLOAD, false, WPC_PHM_OFF);
		sec_bat_set_misc_event(battery,
			0, BATT_MISC_EVENT_FOTA_DOWNLOAD);
	}

	return 0;
}

static ssize_t sysfs_fota_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t sysfs_fota_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

#define SYSFS_FOTA_ATTR(_name)						\
{									\
	.attr = {.name = #_name, .mode = 0664},	\
	.show = sysfs_fota_show_attrs,					\
	.store = sysfs_fota_store_attrs,					\
}

static struct device_attribute sysfs_fota_attrs[] = {
	SYSFS_FOTA_ATTR(fota_download),
};

enum {
	FOTA_DOWNLOAD = 0,
};

static ssize_t sysfs_fota_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return -EINVAL;
}

static ssize_t sysfs_fota_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct sec_battery_info *battery = power_supply_get_drvdata(psy);
	const ptrdiff_t offset = attr - sysfs_fota_attrs;
	int x = 0;

	switch (offset) {
	case FOTA_DOWNLOAD:
		if (sscanf(buf, "%10d\n", &x) != 1)
			return -EINVAL;

		set_fota_event(battery, x);
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static int __init sysfs_fota_init(void)
{
	return sb_sysfs_add_attrs("fota_attr", PSY_BATTERY_NAME,
		sysfs_fota_attrs, ARRAY_SIZE(sysfs_fota_attrs));
}
module_init(sysfs_fota_init);

MODULE_DESCRIPTION("Samsung Battery Fota Sysfs");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
