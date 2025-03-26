/*
 *  sb_ctrl.c
 *  Samsung Mobile Battery Control
 *
 *  Copyright (C) 2022 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/module.h>

#include <linux/battery/control/sb_ctrl.h>
#include <linux/battery/control/sb_ctrl_it.h>
#include <linux/battery/common/sb_vote_event.h>

#define SB_CTRL_NAME	"sb-ctrl"
#define VOTER_CTRL_MIN	VOTER_CTRL_0
#define VOTER_CTRL_MAX	VOTER_CTRL_9

static LIST_HEAD(it_list);
static DEFINE_MUTEX(it_lock);

struct sb_ctrl {
	struct list_head list;
	struct mutex lock;
};

struct ctrl_data {
	struct list_head list;
	struct sb_ctrl_it *it;

	void *pdata;
};

static struct device_node *find_np(struct device_node *np, const char *name)
{
	struct device_node *child = NULL;

	if (name == NULL)
		return NULL;

	for_each_child_of_node(np, child) {
		if (!strcmp(child->name, name))
			return child;
	}

	return NULL;
}

static struct sb_ctrl_it *find_it(const char *name)
{
	struct sb_ctrl_it *it;

	mutex_lock(&it_lock);
	list_for_each_entry(it, &it_list, list) {
		if (strstr(name, it->name) != NULL) {
			mutex_unlock(&it_lock);
			return it;
		}
	}
	mutex_unlock(&it_lock);

	return NULL;
}

static struct ctrl_data *create_ctrl_data(struct device_node *np, struct sb_ctrl_it *it, unsigned int event)
{
	struct ctrl_data *cd = NULL;

	cd = kzalloc(sizeof(struct ctrl_data), GFP_KERNEL);
	if (!cd)
		return NULL;

	INIT_LIST_HEAD(&cd->list);
	cd->it = it;
	cd->pdata = it->cb_create(np, event);
	if (cd->pdata == NULL) {
		kfree(cd);
		return NULL;
	}

	return cd;
}

struct sb_ctrl *sb_ctrl_create(struct device_node *np)
{
	struct device_node *child = NULL;
	struct sb_ctrl *ctrl;
	int event = VOTER_CTRL_MIN;

	if (!np)
		return ERR_PTR(-EINVAL);

	child = find_np(np, SB_CTRL_NAME);
	if (!child)
		return ERR_PTR(-ENODEV);

	/* create */
	ctrl = kzalloc(sizeof(struct sb_ctrl), GFP_KERNEL);
	if (!ctrl)
		return ERR_PTR(-ENOMEM);

	/* init */
	INIT_LIST_HEAD(&ctrl->list);
	mutex_init(&ctrl->lock);

	/* changed root */
	np = child;
	for_each_child_of_node(np, child) {
		struct sb_ctrl_it *it = find_it(child->name);

		if (event >= VOTER_CTRL_MAX)
			break;

		if ((it != NULL) &&
			(it->cb_create != NULL) &&
			(it->cb_event != NULL)) {
			struct ctrl_data *cd = create_ctrl_data(child, it, event);

			if (cd == NULL)
				continue;

			mutex_lock(&ctrl->lock);
			list_add(&cd->list, &ctrl->list);
			event++;
			mutex_unlock(&ctrl->lock);
		}
	}

	if (list_empty(&ctrl->list)) {
		kfree(ctrl);
		return ERR_PTR(-ENODEV);
	}

	return ctrl;
}
EXPORT_SYMBOL(sb_ctrl_create);

int sb_ctrl_set(struct sb_ctrl *ctrl, int event, sb_data data)
{
	struct ctrl_data *cd;
	int ret = 0;

	if (IS_ERR_OR_NULL(ctrl))
		return -EINVAL;

	mutex_lock(&ctrl->lock);
	list_for_each_entry(cd, &ctrl->list, list) {
		ret = cd->it->cb_event(cd->pdata, event, data);
	}
	mutex_unlock(&ctrl->lock);
	return 0;
}
EXPORT_SYMBOL(sb_ctrl_set);

int add_sb_ctrl_it(struct sb_ctrl_it *it)
{
	if (it == NULL)
		return -EINVAL;

	mutex_lock(&it_lock);
	list_add(&it->list, &it_list);
	mutex_unlock(&it_lock);

	return 0;
}
EXPORT_SYMBOL(add_sb_ctrl_it);

static int __init sb_ctrl_init(void)
{
	ctrl_log("module init\n");
	return 0;
}
module_init(sb_ctrl_init);

static void __exit sb_ctrl_exit(void)
{
	ctrl_log("module exit\n");
}
module_exit(sb_ctrl_exit);

MODULE_DESCRIPTION("Samsung Battery Control");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
