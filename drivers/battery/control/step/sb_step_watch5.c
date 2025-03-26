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

#include <linux/battery/common/sb_psy.h>
#include <linux/battery/common/sb_ext.h>
#include <linux/battery/common/sb_type.h>
#include <linux/battery/common/sb_event.h>
#include <linux/battery/common/sb_vote_event.h>

#include <linux/battery/wireless/sb_wrl_def.h>

#define SB_CTRL_NAME		"step-watch5"

#define step_status(status) (status == POWER_SUPPLY_STATUS_CHARGING)
#define step_thermal(thermal) ((thermal != BAT_THERMAL_COOL1) && \
			(thermal != BAT_THERMAL_COOL2) && \
			(thermal != BAT_THERMAL_COOL3) && \
			(thermal != BAT_THERMAL_COLD))
#define step_store(store) (store == true)

#define DEFAULT_STEP		(-1)
#define DEFAULT_VOLT_GAP	25

enum {
	STEP_MODE_CC = 0,
	STEP_MODE_CC_ADJ,
	STEP_MODE_CV_CHECK,
	STEP_MODE_CV_PRESET,
	STEP_MODE_CV,
	STEP_MODE_CV_ADJ,
	STEP_MODE_CHANGE
};

static const char *get_step_mode_str(int mode)
{
	switch (mode) {
	case STEP_MODE_CC:
		return "CC";
	case STEP_MODE_CC_ADJ:
		return "CC_ADJ";
	case STEP_MODE_CV_CHECK:
		return "CV_CHECK";
	case STEP_MODE_CV_PRESET:
		return "CV_PRESET";
	case STEP_MODE_CV:
		return "CV";
	case STEP_MODE_CV_ADJ:
		return "CV_ADJ";
	case STEP_MODE_CHANGE:
		return "Change";
	}

	return "Unknown";
}

struct step_chg {
	/* Set Value */
	int fv;
	int fcc;
	union sb_wrl_vm cc;
	union sb_wrl_vm cv;

	/* Check Value */
	int cc_cur;
	int cv_cur;
};

struct sb_ctrl_sw5 {
	unsigned int event;

	/* state */
	int step;
	int mode;

	/* dt */
	char *wrl_name;

	struct step_chg *st_table;
	unsigned int st_cnt;
};

static void set_vote_by_vm(struct sb_ctrl_sw5 *sw5, union sb_wrl_vm *vm, bool en)
{
	if ((vm == NULL) || (vm->base.type >= SB_WRL_VM_MAX))
		return;

	switch (vm->base.type) {
	case SB_WRL_VM_FX:
		sb_vote_set_f(VN_WPC_LOW_VOUT, sw5->event, false, 0);
		sb_vote_set_f(VN_WPC_VOUT, sw5->event, en, vm->fx.vout);
		sb_vote_set_f(VN_WPC_FX_HDR, sw5->event, en, vm->fx.hdr);
		break;
	case SB_WRL_VM_DC:
		sb_vote_set_f(VN_WPC_LOW_VOUT, sw5->event, false, 0);
		sb_vote_set_f(VN_WPC_DC_HDR, sw5->event, en, vm->dc.hdr);
		sb_vote_set_f(VN_WPC_ICHG, sw5->event, en, vm->dc.ichg);
		sb_vote_set_f(VN_WPC_DC_HIGH_VOUT, sw5->event, en, vm->dc.high_vout);
		break;
	case SB_WRL_VM_BT:
		sb_vote_set_f(VN_WPC_LOW_VOUT, sw5->event, en, vm->bt.low_vout);
		sb_vote_set_f(VN_WPC_BT_VBAT_HDR, sw5->event, en, vm->bt.vbat_hdr);
		sb_vote_set_f(VN_WPC_BT_HDR, sw5->event, en, vm->bt.hdr);
		break;
	}

	sb_vote_set_f(VN_WPC_VM, sw5->event, en, vm->base.type);
}

static int init(struct sb_ctrl_sw5 *sw5)
{
	ctrl_log("%s - step = %d\n",
		(sw5->step > DEFAULT_STEP) ? "refresh" : "init", sw5->step);

	sw5->step = DEFAULT_STEP;
	sw5->mode = STEP_MODE_CC;
	sb_vote_set_f(VN_FCC, sw5->event, false, 0);
	sb_vote_set_f(VN_FV, sw5->event, false, 0);

	/* Vout Mode vote */
	sb_vote_set_f(VN_WPC_VM, sw5->event, false, 0);
	/* DC vote */
	sb_vote_set_f(VN_WPC_DC_HDR, sw5->event, false, 0);
	sb_vote_set_f(VN_WPC_ICHG, sw5->event, false, 0);
	sb_vote_set_f(VN_WPC_DC_HIGH_VOUT, sw5->event, false, 0);
	/* BT vote */
	sb_vote_set_f(VN_WPC_LOW_VOUT, sw5->event, false, 0);
	sb_vote_set_f(VN_WPC_BT_VBAT_HDR, sw5->event, false, 0);
	sb_vote_set_f(VN_WPC_BT_HDR, sw5->event, false, 0);

	return 0;
}

static int parse(struct device_node *np, struct sb_ctrl_sw5 *sw5)
{
	int ret, cnt;

	ret = sb_of_parse_str(np, sw5, wrl_name);
	if (ret)
		return -EINVAL;

	cnt = of_property_count_u32_elems(np, "st_table");
	if (cnt > 0) {
		int temp_cnt =
			(cnt * sizeof(unsigned int)) / sizeof(struct step_chg);

		sw5->st_table = kcalloc(temp_cnt, sizeof(struct step_chg), GFP_KERNEL);
		if (!sw5->st_table)
			return -ENOMEM;

		ret = of_property_read_u32_array(np, "st_table",
			(u32 *)sw5->st_table, cnt);
		if (ret) {
			kfree(sw5->st_table);
			return -EINVAL;
		}

		sw5->st_cnt = temp_cnt;
		ctrl_log("st_table - count (%d)\n", temp_cnt);
	}

	return 0;
}

static int check_step_volt(struct step_chg *table, unsigned int size, unsigned int volt)
{
	int idx = 0;

	while ((idx < size - 1) &&
		(table[idx].fv < volt))
		idx++;

	return idx;
}

static void set_step(struct sb_ctrl_sw5 *sw5, int step)
{
	struct step_chg *stc = NULL;

	if ((step < 0) || (sw5->st_cnt <= step))
		return;

	stc = &sw5->st_table[step];

	/* set vm vote */
	set_vote_by_vm(sw5, &stc->cc, true);

	/* set chg vote */
	sb_vote_set_f(VN_FV, sw5->event, true, stc->fv);
	sb_vote_set_f(VN_FCC, sw5->event, true, stc->fcc);

	sw5->step = step;
	sw5->mode = STEP_MODE_CC;
}

static int monitor(struct sb_ctrl_sw5 *sw5, sb_data data)
{
	struct sb_info *info = (struct sb_info *)data;
	union power_supply_propval value = { 0, };
	struct step_chg *stc = NULL;
	int old_mode = STEP_MODE_CC;

	if (!step_status(info->status) ||
		!step_thermal(info->thermal_zone) ||
		step_store(info->store_mode)) {
		init(sw5);
		return 0;
	}

	old_mode = sw5->mode;
	/* check PHM */
	psy_do_property(sw5->wrl_name, get,
		POWER_SUPPLY_EXT_PROP_WIRELESS_PHM, value);
	if (value.intval) {
		ctrl_log("keep step(%d) because of phm\n", sw5->step);
		goto step_end;
	}

	if (sw5->step == DEFAULT_STEP) {
		/* set new CC mode */
		set_step(sw5, check_step_volt(sw5->st_table, sw5->st_cnt, info->voltage_avg));
		goto step_end;
	}

	stc = &sw5->st_table[sw5->step];
	switch (sw5->mode) {
	case STEP_MODE_CC:
	case STEP_MODE_CC_ADJ:
		if (stc->fv - DEFAULT_VOLT_GAP > info->voltage_now) {
			sw5->mode = STEP_MODE_CC;
			goto step_end;
		}

		psy_do_property(sw5->wrl_name, get,
			POWER_SUPPLY_EXT_PROP_WIRELESS_TX_UNO_IIN, value);
		sw5->mode = (stc->cc_cur > value.intval) ?
			(sw5->mode + 1) : STEP_MODE_CC;
		break;
	case STEP_MODE_CV_CHECK:
		/* clear old vm vote */
		set_vote_by_vm(sw5, &stc->cc, false);

		/* check iout */
		psy_do_property(sw5->wrl_name, get,
			POWER_SUPPLY_EXT_PROP_WIRELESS_TX_UNO_IIN, value);
		if (stc->cv_cur > value.intval) {
			/* set vm vote to next cc */
			set_step(sw5, sw5->step + 1);
			break;
		}
	case STEP_MODE_CV_PRESET:
		/* set vm vote to cv */
		set_vote_by_vm(sw5, &stc->cv, true);
		sw5->mode = STEP_MODE_CV;
		break;
	case STEP_MODE_CV:
	case STEP_MODE_CV_ADJ:
		psy_do_property(sw5->wrl_name, get,
			POWER_SUPPLY_EXT_PROP_WIRELESS_TX_UNO_IIN, value);
		sw5->mode = (stc->cv_cur > value.intval) ?
			(sw5->mode + 1) : STEP_MODE_CV;
		break;
	case STEP_MODE_CHANGE:
		/* clear old vm vote */
		set_vote_by_vm(sw5, &stc->cv, false);
		/* set vm vote to next cc */
		set_step(sw5, sw5->step + 1);
		break;
	default:
		break;
	}

step_end:
	ctrl_log("Step = %d, Mode = %s --> %s\n",
		sw5->step, get_step_mode_str(old_mode), get_step_mode_str(sw5->mode));
	return 0;
}

static void *cb_create(struct device_node *np, unsigned int data)
{
	struct sb_ctrl_sw5 *sw5;
	int ret = 0;

	sw5 = kzalloc(sizeof(struct sb_ctrl_sw5), GFP_KERNEL);
	if (!sw5)
		return NULL;

	ret = parse(np, sw5);
	if (ret) {
		kfree(sw5);
		return NULL;
	}

	sw5->event = data;

	init(sw5);
	ctrl_log("finished!!\n");

	return sw5;
}

static int cb_event(void *pdata, unsigned int event, sb_data data)
{
	struct sb_ctrl_sw5 *sw5 = pdata;
	int ret = 0;

	if (!sw5)
		return -EINVAL;

	switch (event) {
	case SB_CTRL_EVENT_CABLE_ATTACH:
	case SB_CTRL_EVENT_CABLE_DETACH:
		ret = init(sw5);
		break;
	case SB_CTRL_EVENT_MONITOR:
		ret = monitor(sw5, data);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static struct sb_ctrl_it sw5_it = {
	.name = SB_CTRL_NAME,
	.cb_create = cb_create,
	.cb_event = cb_event
};

static int __init sw5_init(void)
{
	return add_sb_ctrl_it(&sw5_it);
}
early_initcall(sw5_init);

MODULE_DESCRIPTION("Samsung Battery Control Step Watch5");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
