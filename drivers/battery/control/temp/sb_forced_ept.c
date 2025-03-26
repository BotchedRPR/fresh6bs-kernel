/*
 *  sb_forced_ept.c
 *  Samsung Mobile Battery Control Forced Ept for Watch6
 *
 *  Copyright (C) 2023 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <linux/battery/sb_vote.h>
#include <linux/battery/control/sb_ctrl.h>
#include <linux/battery/control/sb_ctrl_it.h>
#include <linux/battery/common/sb_vote_event.h>
#include <linux/battery/module/sb_adc.h>

#define SB_CTRL_NAME		"forced-ept"

#define TRIGGER_NAME		"trigger"

struct sb_ctrl_fe {
	unsigned int event;

	/* dt */
	struct sb_adc_param_list *list;
};

static int init(struct sb_ctrl_fe *fe)
{
	sb_vote_set_f(VN_WPC_PHM, fe->event, false, WPC_PHM_OFF);

	return 0;
}

static int parse(struct device_node *np, struct sb_ctrl_fe *fe)
{
	int ret = 0;

	fe->list = sb_adc_get_param_list(np, TRIGGER_NAME);
	if (IS_ERR_OR_NULL(fe->list))
		ret = -ENOMEM;

	return ret;
}

static int monitor(struct sb_ctrl_fe *fe, sb_data data)
{
	bool state = false;

	state = (sb_adc_check_param_list(fe->list) > 0);
	ctrl_log("forced-ept(%s)\n", GET_BOOL_STR(state));

	sb_vote_set_f(VN_WPC_PHM, fe->event, state, WPC_PHM_OFF);
	sb_vote_set_f(VN_WPC_PHM, fe->event, state, WPC_PHM_FEPT_ON);

	return state;
}

static void *cb_create(struct device_node *np, unsigned int data)
{
	struct sb_ctrl_fe *fe;
	int ret = 0;

	fe = kzalloc(sizeof(struct sb_ctrl_fe), GFP_KERNEL);
	if (!fe)
		return NULL;

	ret = parse(np, fe);
	if (ret) {
		kfree(fe);
		return NULL;
	}

	fe->event = data;

	sb_vote_set_pri_f(VN_WPC_PHM, fe->event, VOTE_PRI_9); /* VOTER_WPC_DET - 1 */

	init(fe);
	ctrl_log("finished!!\n");

	return fe;
}

static int cb_event(void *pdata, unsigned int event, sb_data data)
{
	struct sb_ctrl_fe *fe = pdata;
	int ret = 0;

	if (!fe)
		return -EINVAL;

	switch (event) {
	case SB_CTRL_EVENT_CABLE_ATTACH:
	case SB_CTRL_EVENT_CABLE_DETACH:
		ret = init(fe);
		break;
	case SB_CTRL_EVENT_MONITOR:
		ret = monitor(fe, data);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static struct sb_ctrl_it fe_it = {
	.name = SB_CTRL_NAME,
	.cb_create = cb_create,
	.cb_event = cb_event
};

static int __init fe_init(void)
{
	ctrl_log("\n");

	return add_sb_ctrl_it(&fe_it);
}
module_init(fe_init);

static void __exit fe_exit(void)
{
}
module_exit(fe_exit);

MODULE_DESCRIPTION("Samsung Battery Control Forced Ept");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
