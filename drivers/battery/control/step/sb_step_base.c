/*
 *  sb_step_watch5.c
 *  Samsung Mobile Battery Step Charging for Watch5
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
#include <linux/module.h>
#include <linux/device.h>

#include <linux/battery/sb_vote.h>
#include <linux/battery/control/sb_ctrl.h>
#include <linux/battery/control/sb_ctrl_it.h>

#include <linux/battery/common/sb_ext.h>
#include <linux/battery/common/sb_type.h>
#include <linux/battery/common/sb_vote_event.h>

#define SB_CTRL_NAME		"step-base"

struct sb_ctrl_sb {
	unsigned int event;

	/* dt */
	unsigned int cc2_voltage;
	unsigned int cc2_current;
};

static int init(struct sb_ctrl_sb *sb)
{
	sb_vote_set_f(VN_FCC, sb->event, false, 0);
	return 0;
}

static int parse(struct device_node *np, struct sb_ctrl_sb *sb)
{
	sb_of_parse_u32(np, sb, cc2_voltage, 4120);
	sb_of_parse_u32(np, sb, cc2_current, 169);
	return 0;
}

static int monitor(struct sb_ctrl_sb *sb, sb_data data)
{
	struct sb_info *info = (struct sb_info *)data;

	if (info->voltage_avg > sb->cc2_voltage)
		sb_vote_set_f(VN_FCC, sb->event, SB_VOTER_ENABLE, sb->cc2_current);

	ctrl_log("check voltage(%d, %d)\n",
		info->voltage_avg, sb->cc2_voltage);
	return 0;
}

static void *cb_create(struct device_node *np, unsigned int data)
{
	struct sb_ctrl_sb *sb;
	int ret = 0;

	sb = kzalloc(sizeof(struct sb_ctrl_sb), GFP_KERNEL);
	if (!sb)
		return NULL;

	ret = parse(np, sb);
	if (ret) {
		kfree(sb);
		return NULL;
	}

	sb->event = data;

	init(sb);
	ctrl_log("finished!!\n");

	return sb;
}

static int cb_event(void *pdata, unsigned int event, sb_data data)
{
	struct sb_ctrl_sb *sb = pdata;
	int ret = 0;

	if (!sb)
		return -EINVAL;

	switch (event) {
	case SB_CTRL_EVENT_CABLE_ATTACH:
	case SB_CTRL_EVENT_CABLE_DETACH:
		ret = init(sb);
		break;
	case SB_CTRL_EVENT_MONITOR:
		ret = monitor(sb, data);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static struct sb_ctrl_it sb_it = {
	.name = SB_CTRL_NAME,
	.cb_create = cb_create,
	.cb_event = cb_event
};

static int __init sb_init(void)
{
	return add_sb_ctrl_it(&sb_it);
}
early_initcall(sb_init);

MODULE_DESCRIPTION("Samsung Battery Control Step Base");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
