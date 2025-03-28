/*
 *  sb_sysfs.c
 *  Samsung Mobile SysFS
 *
 *  Copyright (C) 2022 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>

#include <linux/battery/sb_sysfs.h>

struct sb_sysfs_mod {
	const char *psy_name;
	const char *name;

	struct device_attribute *attr;
	unsigned long size;
	unsigned long state;

	struct list_head list;
};

static LIST_HEAD(sysfs_list);
static DEFINE_MUTEX(sysfs_lock);

static struct device *get_battery_dev(const char *psy_name)
{
	struct power_supply *psy = NULL;

	if (psy_name == NULL)
		return NULL;

	psy = power_supply_get_by_name(psy_name);
	if (!IS_ERR_OR_NULL(psy))
		return &psy->dev;

	return NULL;
}

int sb_sysfs_create_attrs(struct device *dev)
{
	struct sb_sysfs_mod *sysfs_mod;
	int ret = 0;

	if (dev == NULL)
		return -EINVAL;

	mutex_lock(&sysfs_lock);

	list_for_each_entry(sysfs_mod, &sysfs_list, list) {
		const char *dev_name = kobject_name(&dev->kobj);
		unsigned long i = 0;

		if (sysfs_mod->state == sysfs_mod->size)
			continue;

		if ((dev_name != NULL) && strcmp(dev_name, sysfs_mod->psy_name))
			continue;

		for (i = sysfs_mod->state; i < sysfs_mod->size; i++) {
			if (device_create_file(dev, &sysfs_mod->attr[i])) {
				while (i--)
					device_remove_file(dev, &sysfs_mod->attr[i]);

				sysfs_mod->state = 0;
				break;
			}

			sysfs_mod->state++;
		}

		ret += sysfs_mod->state;
	}

	mutex_unlock(&sysfs_lock);

	return ret;
}
EXPORT_SYMBOL(sb_sysfs_create_attrs);

int sb_sysfs_add_attrs(const char *name, const char *psy_name, struct device_attribute *attr, unsigned long size)
{
	struct sb_sysfs_mod *sysfs_mod;

	if ((name == NULL) || (attr == NULL) || (size <= 0))
		return -EINVAL;

	sysfs_mod = kzalloc(sizeof(struct sb_sysfs_mod), GFP_KERNEL);
	if (!sysfs_mod)
		return -ENOMEM;

	sysfs_mod->psy_name = psy_name;
	sysfs_mod->name = name;
	sysfs_mod->attr = attr;
	sysfs_mod->size = size;
	sysfs_mod->state = 0;

	mutex_lock(&sysfs_lock);

	list_add(&sysfs_mod->list, &sysfs_list);

	mutex_unlock(&sysfs_lock);

	sb_sysfs_create_attrs(get_battery_dev(sysfs_mod->psy_name));

	return 0;
}
EXPORT_SYMBOL(sb_sysfs_add_attrs);

int sb_sysfs_remove_attrs(const char *name)
{
	struct sb_sysfs_mod *sysfs_mod;
	bool is_found = false;

	if (name == NULL)
		return -EINVAL;

	mutex_lock(&sysfs_lock);

	list_for_each_entry(sysfs_mod, &sysfs_list, list) {
		is_found = !strcmp(name, sysfs_mod->name);
		if (is_found)
			goto skip_loop;
	}

skip_loop:
	if (is_found) {
		struct device *dev = get_battery_dev(sysfs_mod->psy_name);

		if ((dev != NULL) && (sysfs_mod->state > 0)) {
			while (sysfs_mod->state--)
				device_remove_file(dev,
					&sysfs_mod->attr[sysfs_mod->state]);
		}

		list_del(&sysfs_mod->list);
		kfree(sysfs_mod);
	}

	mutex_unlock(&sysfs_lock);
	return (is_found) ? 0 : -ENODEV;
}
EXPORT_SYMBOL(sb_sysfs_remove_attrs);
