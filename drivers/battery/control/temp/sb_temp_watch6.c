/*
 *  sb_temp_watch6.c
 *  Samsung Mobile Battery charging temperature controls for Watch6
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
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>

#include <linux/battery/sb_vote.h>
#include <linux/battery/sb_notify.h>
#include <linux/battery/control/sb_ctrl.h>
#include <linux/battery/control/sb_ctrl_it.h>
#include <linux/battery/common/sb_ext.h>
#include <linux/battery/common/sb_vote_event.h>
#include <linux/battery/module/sb_adc.h>

#define SB_CTRL_NAME		"temp-watch6"

#define RELEASE_CSTEP_NAME	"release_cstep"
#define TRIGGER_NAME		"trigger"
#define RELEASE_NAME		"release"

struct sb_ctrl_tb {
	unsigned int event;

	/* state */
	unsigned int state;
	unsigned int cstep;
	unsigned int min_cstep;

	/* dt */
	struct sb_adc_param_list *rls_cstep_list;
	struct sb_adc_param_list *trg_list;
	struct sb_adc_param_list *rls_list;

	unsigned int fcc;

	unsigned int cstep_cnt;
	unsigned int *cstep_table;

	struct notifier_block sb_nb;
};

static unsigned int find_cstep_number(
	unsigned int cstep_tab[], unsigned int c_len, unsigned int c_step, unsigned int min_cstep_curr)
{
	unsigned int ret = c_step;
	int len = c_len - 1;
	int i = 0;

	while (i != len && len > 0) {
		if (cstep_tab[i] <= min_cstep_curr) {
			ret = i;
			break;
		}
		i++;
	}

	return ret;
}

static int sb_noti_handler(struct notifier_block *nb, unsigned long action, void *data)
{
	struct sb_ctrl_tb *tb = container_of(nb, struct sb_ctrl_tb, sb_nb);

	switch (action) {
	case SB_NOTIFY_EVENT_CC_CHANGE:
		ctrl_log("min_cstep_current(%d)\n", *(unsigned int *)data);
		tb->min_cstep = find_cstep_number(tb->cstep_table, tb->cstep_cnt, tb->cstep, *(unsigned int *)data);
		ctrl_log("min_cstep(%d)\n", tb->min_cstep);
		break;
	}

	return 0;
}

static int init(struct sb_ctrl_tb *tb)
{
	tb->state = 0;
	tb->cstep = 0;

	sb_vote_set_f(VN_WPC_PHM, tb->event, false, WPC_PHM_OFF);
	sb_vote_set_f(VN_FCC, tb->event, false, 0);

	return 0;
}

static int parse(struct device_node *np, struct sb_ctrl_tb *tb)
{
	int clen = 0, ret = -1;

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
	return ret;
}

static int monitor(struct sb_ctrl_tb *tb, sb_data data)
{
	bool fcc_state = false;

	if (!tb->state) {
		tb->state = (sb_adc_check_param_list(tb->trg_list) > 0);

		if (tb->cstep_cnt > 0) {
			if (tb->state)
				tb->cstep = min((tb->cstep + 1), (tb->cstep_cnt - 1));
			else if ((tb->cstep > 0) &&
				(sb_adc_check_param_list(tb->rls_cstep_list) > 0))
				tb->cstep--;
		}
	} else {
		tb->state = !(sb_adc_check_param_list(tb->rls_list) > 0);
	}

	fcc_state = (tb->state) || (tb->cstep > 0);
	ctrl_log("check fcc(%s), state(%s), cstep(%d), min_cstep(%d)\n",
		GET_BOOL_STR(fcc_state), GET_BOOL_STR(!!tb->state), tb->cstep, tb->min_cstep);

	tb->cstep = (tb->cstep < tb->min_cstep) ? tb->min_cstep : tb->cstep;

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
	sb_notify_register(&tb->sb_nb,
		sb_noti_handler, SB_CTRL_NAME, SB_DEV_MODULE);

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

MODULE_DESCRIPTION("Samsung Battery Temp Control Watch6");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
