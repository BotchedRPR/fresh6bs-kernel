/*
 *  cps4019_startup.c
 *  Samsung Mobile CPS4019 Startup sub Module
 *
 *  Copyright (C) 2023 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>

#include <linux/battery/common/sb_type.h>
#include <linux/battery/common/sb_vote_event.h>
#include <linux/battery/module/sb_startup.h>

#include <linux/battery/wireless/sb_wrl_def.h>

#include "cps4019_core.h"
#include "cps4019_charger.h"

/* default fixed mode, vout 4.8V, headroom  0.2V */
#define DEFAULT_SUP_MODE	0x1412c000000003
static union sb_wrl_vm sup_mode;

static void sup_set_vm(struct cps4019_charger_data *charger,
			int event, bool en, union sb_wrl_vm *vm)
{
	if ((vm == NULL) || (vm->base.type >= SB_WRL_VM_MAX))
		return;

	switch (vm->base.type) {
	case SB_WRL_VM_FX:
		sb_vote_set(charger->vout_vote, event, en, vm->fx.vout);
		sb_vote_set(charger->fx_hdr_vote, event, en, vm->fx.hdr);
		break;
	case SB_WRL_VM_DC:
		sb_vote_set(charger->dc_hdr_vote, event, en, vm->dc.hdr);
		sb_vote_set(charger->ichg_vote, event, en, vm->dc.ichg);
		sb_vote_set(charger->dc_high_vout_vote, event, en, vm->dc.high_vout);
		break;
	case SB_WRL_VM_BT:
		sb_vote_set(charger->low_vout_vote, event, en, vm->bt.low_vout);
		sb_vote_set(charger->bt_vbat_hdr_vote, event, en, vm->bt.vbat_hdr);
		sb_vote_set(charger->bt_hdr_vote, event, en, vm->bt.hdr);
		break;
	}

	sb_vote_set(charger->vm_vote, event, en, vm->base.type);
}

static int sup_event(void *pdata, unsigned int event, sb_data time)
{
	struct cps4019_charger_data *charger = pdata;
	sb_data phm_result = WPC_PHM_OFF;
	int vout, vrect, i;
	bool reset_wireless_chg = false;

	pr_info("%s: charger - time = %d\n", __func__, (int)time);

	if (!is_wireless_type(charger->cable_type))
		goto end_event;

	sb_vote_get_result(charger->phm_vote, &phm_result);
	if (phm_result != WPC_PHM_OFF)
		goto end_event;

	/* WA code. Boot and put it on the charging pad 2 seconds later
	 * Vout can be abnormal value
	 */
	for (i = 0; i < 3; i++) {
		vout = cps4019_get_adc(charger->client, CPS4019_ADC_VOUT);
		vrect = cps4019_get_adc(charger->client, CPS4019_ADC_VRECT);

		if (vout >= 2000) {
			reset_wireless_chg = false;
			break;
		}

		reset_wireless_chg = (vrect >= 3500);
		pr_info("%s: vout(%d) vrect(%d) reset_wireless_chg(%d)\n",
			__func__, vout, vrect, reset_wireless_chg);
		msleep(100);
	}

	/* if vout is abnormal, reset charging using PHM */
	if (reset_wireless_chg) {
		pr_info("%s: vout & chgen off!!\n", __func__);
		sb_vote_set_f(VN_CHG_EN, VOTER_STARTUP, true, SB_CHG_MODE_CHARGING_OFF);
		sb_vote_set(charger->phm_vote, VOTER_STARTUP, true, WPC_PHM_ON);

		pr_info("%s: vout & chgen on!!\n", __func__);
		sb_vote_set(charger->phm_vote, VOTER_STARTUP, false, WPC_PHM_OFF);
		sb_vote_set_f(VN_CHG_EN, VOTER_STARTUP, false, SB_CHG_MODE_CHARGING_OFF);
	}

end_event:
	sup_set_vm(charger, VOTER_STARTUP, false, &sup_mode);
	return 0;
}

int cps4019_startup_init(struct cps4019_charger_data *charger)
{
	struct device_node *np = charger->dev->of_node;
	unsigned int sup_time = 60; /* default 1min */
	int ret = 0;

	if (!sb_startup_is_activated()) {
		pr_info("%s: startup is not activated!\n", __func__);
		return 0;
	}

	/* default fixed mode, vout 4.8V, headroom  0.2V */
	sup_mode.value = DEFAULT_SUP_MODE;
	/* parse dt */
	of_property_read_u32_array(np, "sup_mode", (unsigned int *)&sup_mode, 2);
	of_property_read_u32(np, "sup_time", &sup_time);

	/* set vote pri */
	sb_vote_set_pri(charger->vm_vote, VOTER_STARTUP, VOTE_PRI_8);
	sb_vote_set_pri(charger->dc_hdr_vote, VOTER_STARTUP, VOTE_PRI_10);
	sb_vote_set_pri(charger->low_vout_vote, VOTER_STARTUP, VOTE_PRI_10);
	sb_vote_set_pri(charger->bt_vbat_hdr_vote, VOTER_STARTUP, VOTE_PRI_10);
	sb_vote_set_pri(charger->bt_hdr_vote, VOTER_STARTUP, VOTE_PRI_10);
	sb_vote_set_pri(charger->vout_vote, VOTER_STARTUP, VOTE_PRI_10);
	sb_vote_set_pri(charger->fx_hdr_vote, VOTER_STARTUP, VOTE_PRI_10);

	sup_set_vm(charger, VOTER_STARTUP, true, &sup_mode);
	/* set startup */
	ret = sb_startup_register(charger, sup_event, 0, sup_time);
	pr_info("%s: ret = %d, sup_mode = 0x%llx, sup_time =%d\n",
		__func__, ret, sup_mode.value, sup_time);

	return ret;
}
EXPORT_SYMBOL(cps4019_startup_init);
