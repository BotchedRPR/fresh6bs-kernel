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
#include <linux/battery/common/sb_vote_event.h>
#include <linux/battery/module/sb_adc.h>
#include <linux/battery/wireless/sb_wrl.h>

#define SB_CTRL_NAME		"temp-base"

#define EPT_NAME			"ept"
#define RELEASE_CSTEP_NAME	"release_cstep"
#define TRIGGER_NAME		"trigger"
#define RELEASE_NAME		"release"

struct sb_ctrl_tb {
	unsigned int event;

	/* state */
	unsigned int state;
	unsigned int cstep;

	/* dt */
	struct sb_adc_param_list *ept_list;
	struct sb_adc_param_list *rls_cstep_list;
	struct sb_adc_param_list *trg_list;
	struct sb_adc_param_list *rls_list;

	unsigned int fcc;

	unsigned int cstep_cnt;
	unsigned int *cstep_table;
};

static int init(struct sb_ctrl_tb *tb)
{
	tb->state = 0;
	tb->cstep = 0;

	sb_wrl_set_vm_phm(tb->event, false, WPC_PHM_OFF);
	sb_vote_set_f(VN_FCC, tb->event, false, 0);

	return 0;
}

static int parse(struct device_node *np, struct sb_ctrl_tb *tb)
{
	int clen = 0, ret = -1;

	tb->ept_list = sb_adc_get_param_list(np, EPT_NAME);
	tb->rls_cstep_list = sb_adc_get_param_list(np, RELEASE_CSTEP_NAME);

	tb->trg_list = sb_adc_get_param_list(np, TRIGGER_NAME);
	if (IS_ERR_OR_NULL(tb->trg_list))
		goto fail_get_trg;
	tb->rls_list = sb_adc_get_param_list(np, RELEASE_NAME);
	if (IS_ERR_OR_NULL(tb->rls_list))
		goto fail_get_rls;

	sb_of_parse_u32(np, tb, fcc, 169);

	clen = of_property_count_u32_elems(np, "cstep");
	if (clen > 0) {
		tb->cstep_cnt = clen;

		tb->cstep_table = kcalloc(tb->cstep_cnt, sizeof(unsigned int), GFP_KERNEL);
		if (tb->cstep_table == NULL) {
			ret = -ENOMEM;
			goto fail_alloc_cstep;
		}
		if (of_property_read_u32_array(np, "cstep",
				(u32 *)tb->cstep_table, tb->cstep_cnt)) {
			ret = -EINVAL;
			goto fail_get_cstep;
		}
	}

	return 0;

fail_get_cstep:
	kfree(tb->cstep_table);
fail_alloc_cstep:
	sb_adc_free_param_list(tb->rls_list);
fail_get_rls:
	sb_adc_free_param_list(tb->trg_list);
fail_get_trg:
	sb_adc_free_param_list(tb->ept_list);
	return ret;
}

static int monitor(struct sb_ctrl_tb *tb, sb_data data)
{
	bool ept_state = false, fcc_state = false;

	if (!tb->state) {
		tb->state = (sb_adc_check_param_list(tb->trg_list) > 0);

		if (tb->cstep_cnt > 0) {
			if (tb->state)
				tb->cstep = min((tb->cstep + 1), (tb->cstep_cnt - 1));
			else if ((tb->cstep > 0) &&
				(sb_adc_check_param_list(tb->rls_cstep_list) > 0))
				tb->cstep = max((tb->cstep - 1), ((unsigned int)0));
		}
	} else {
		tb->state = !(sb_adc_check_param_list(tb->rls_list) > 0);
	}

	fcc_state = (tb->state) || (tb->cstep > 0);
	ept_state = (sb_adc_check_param_list(tb->ept_list) > 0);
	ctrl_log("check fcc(%s), ept(%s), state(%s), cstep(%d)\n",
		GET_BOOL_STR(fcc_state), GET_BOOL_STR(ept_state), GET_BOOL_STR(!!tb->state), tb->cstep);

	sb_wrl_set_vm_phm(tb->event, ept_state, WPC_PHM_EPT_ON);
	if (tb->state)
		sb_vote_set_f(VN_FCC, tb->event, true, tb->fcc);
	else if (tb->cstep > 0)
		sb_vote_set_f(VN_FCC, tb->event, true, tb->cstep_table[tb->cstep]);
	else
		sb_vote_set_f(VN_FCC, tb->event, false, 0);

	return tb->state;
}

static void *cb_create(struct device_node *np, unsigned int data)
{
	struct sb_ctrl_tb *tb;
	int ret = 0;

	tb = kzalloc(sizeof(struct sb_ctrl_tb), GFP_KERNEL);
	if (!tb)
		return NULL;

	ret = parse(np, tb);
	if (ret) {
		kfree(tb);
		return NULL;
	}

	tb->event = data;

	init(tb);
	ctrl_log("finished!!\n");

	return tb;
}

static int cb_event(void *pdata, unsigned int event, sb_data data)
{
	struct sb_ctrl_tb *tb = pdata;
	int ret = 0;

	if (!tb)
		return -EINVAL;

	switch (event) {
	case SB_CTRL_EVENT_CABLE_ATTACH:
	case SB_CTRL_EVENT_CABLE_DETACH:
		ret = init(tb);
		break;
	case SB_CTRL_EVENT_MONITOR:
		ret = monitor(tb, data);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static struct sb_ctrl_it tb_it = {
	.name = SB_CTRL_NAME,
	.cb_create = cb_create,
	.cb_event = cb_event
};

static int __init tb_init(void)
{
	return add_sb_ctrl_it(&tb_it);
}
early_initcall(tb_init);

MODULE_DESCRIPTION("Samsung Battery Control Temp Base");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
