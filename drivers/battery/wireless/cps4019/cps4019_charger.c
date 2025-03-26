/*
 *  cps4019_charger.c
 *  Samsung CPS4019 Charger Driver
 *
 *  Copyright (C) 2015 Samsung Electronics
 * Yeongmi Ha <yeongmi86.ha@samsung.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/common/muic.h>
#include <linux/muic/common/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#include <linux/battery/common/sb_psy.h>
#include <linux/battery/common/sb_type.h>
#include <linux/battery/common/sb_event.h>
#include <linux/battery/common/sb_vote_event.h>
#include <linux/battery/wireless/sb_wrl_data.h>
#include <linux/battery/wireless/sb_wrl.h>

#include "cps4019_core.h"
#include "cps4019_charger.h"

/* Vout stabilization time is about 1.5sec after Vrect ocured. */
#define VOUT_STABLILZATION_TIME		1500
#define VOUT_MIN_LEVEL				1000

#define PHM_DETACH_DELAY			1700
#define WPC_AUTH_DELAY_TIME			1
#define UNKNOWN_TX_ID				0xFF

#define SEND_RX_ID_CNT				30
#define SEND_REQ_TX_ID_CNT			15
#define SEND_RX_PWR_CNT				15

#define AUTH_RETRY_CNT			3
#define AUTH_DONE_CHK_DELAY		60000

#define ALIGN_WORK_CHK_CNT		8
#define ALIGN_WORK_DELAY		1000
#define ALIGN_CHK_PERIOD		200
#define ALIGN_WORK_CHK_PERIOD	100
#define MISALIGN_TX_OFF_TIME	10
#define ALIGN_MIN_FREQ			1160

#define PHM_CHK_DELAY	2000
#define PHM_CHK_CNT		5

#define WAKE_LOCK_TIME_OUT	10000

#define ENABLE 1
#define DISABLE 0
#define CMD_CNT 3

#define CPS4019_I_OUT_OFFSET		12

static enum power_supply_property wpc_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int cps4019_set_en_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;
	bool enable = !!(v);

	pr_info("%s: set WPC-EN = %s\n", __func__, GET_BOOL_STR(enable));

	if (gpio_is_valid(charger->pdata->wpc_ping_en))
		gpio_direction_output(charger->pdata->wpc_ping_en, enable);

	if (gpio_is_valid(charger->pdata->wpc_en))
		gpio_direction_output(charger->pdata->wpc_en, !enable);

	return 0;
}

static int cps4019_set_det_vote(void *data, sb_data v)
{
	pr_info("%s: set WPC-DET = %lld\n", __func__, v);
	return 0;
}

static int cps4019_set_online(struct cps4019_charger_data *charger, int cable_type)
{
	union power_supply_propval value = { cable_type, };

	charger->cable_type = cable_type;
	return psy_do_property(charger->pdata->battery_name, set,
		POWER_SUPPLY_PROP_ONLINE, value);
}


static int cps4019_set_cmd_reg(struct cps4019_charger_data *charger, u8 val, u8 mask)
{
	int ret = 0, i = 0;
	u8 temp = 0;

	do {
		pr_info("%s: [%d] temp = 0x%x, val = 0x%x, mask = 0x%x\n",
			__func__, i, temp, val, mask);

		ret = cps4019_reg_update(charger->client, CPS4019_COMMAND_L_REG, val, mask); // command
		if (ret < 0)
			break;

		msleep(250);

		ret = cps4019_reg_read(charger->client, CPS4019_COMMAND_L_REG, &temp); // check out set bit exists
		if (ret < 0)
			break;

	} while ((temp & mask) && (i++ <= 3));

	return ret;
}

static int cps4019_set_watchdog_timer(struct cps4019_charger_data *charger, bool state)
{
	u8 data = 0;
	int ret = 0;

	data = (state) ? CPS4019_CMD_WATCHDOG_EN_MASK : 0;
	ret = cps4019_reg_update(charger->client,
		CPS4019_COMMAND_L_REG, data, CPS4019_CMD_WATCHDOG_EN_MASK);

	cps4019_reg_read(charger->client, CPS4019_COMMAND_L_REG, &data);
	pr_info("%s: set watchdog timer to %s, data = 0x%x\n",
		__func__, GET_BOOL_STR(state), data);
	return ret;
}

static int cps4019_send_packet(struct cps4019_charger_data *charger, u8 header, u8 rx_data_com, u8 data_val)
{
	int ret = 0;

	ret = cps4019_reg_write(charger->client, CPS4019_PPP_HEADER_REG, header);
	if (ret < 0) {
		pr_err("%s: failed to write header(0x%x), ret(%d)\n", __func__, header, ret);
		return ret;
	}

	ret = cps4019_reg_write(charger->client, CPS4019_RX_DATA_COM_REG, rx_data_com);
	if (ret < 0) {
		pr_err("%s: failed to write rx_data_com(0x%x), ret(%d)\n", __func__, rx_data_com, ret);
		return ret;
	}

	ret = cps4019_reg_write(charger->client, CPS4019_RX_DATA_VALUE_REG, data_val);
	if (ret < 0) {
		pr_err("%s: failed to write data_val(0x%x), ret(%d)\n", __func__, data_val, ret);
		return ret;
	}

	ret = cps4019_reg_update(charger->client, CPS4019_COMMAND_L_REG,
		CPS4019_CMD_SEND_RX_DATA_MASK, CPS4019_CMD_SEND_RX_DATA_MASK);
	if (ret < 0) {
		pr_err("%s: failed to send rx data, ret(%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

static void cps4019_set_gpio_state(struct cps4019_gpio *gpio, unsigned int state)
{
	struct irq_desc *desc;

	if (gpio->state == state)
		return;

	desc = irq_to_desc(gpio->irq);

	if ((desc->wake_depth == 0) && (state & CPS_IRQ_ENWAKE))
		enable_irq_wake(gpio->irq);

	if ((desc->wake_depth > 0) && (state & CPS_IRQ_DISWAKE))
		disable_irq_wake(gpio->irq);

	if ((desc->depth > 0) && (state & CPS_IRQ_EN))
		enable_irq(gpio->irq);

	if ((desc->depth == 0) && (state & CPS_IRQ_DIS))
		disable_irq(gpio->irq);

	if ((desc->depth == 0) && (state & CPS_IRQ_DISNOSYNC))
		disable_irq_nosync(gpio->irq);

	pr_info("%s: %s[%d, %d, 0x%lx] set 0x%x --> 0x%x\n",
		__func__,
		gpio->name, desc->depth, desc->wake_depth,
		irq_desc_get_chip(desc)->flags,
		gpio->state, state);

	gpio->state = state;
}

static void cps4019_set_gpio_irq(struct cps4019_charger_data *charger, int gpio_type, unsigned int state)
{
	mutex_lock(&charger->gpio_lock);

	cps4019_set_gpio_state(&charger->pdata->gpios[gpio_type], state);

	mutex_unlock(&charger->gpio_lock);
}

static bool cps4019_get_gpio_value(struct cps4019_gpio *gpio)
{
	if (!gpio_is_valid(gpio->gpio))
		return false;

	return (gpio->flags & OF_GPIO_ACTIVE_LOW) ?
		!gpio_get_value(gpio->gpio) : gpio_get_value(gpio->gpio);
}

static struct cps4019_gpio *cps4019_get_gpio_by_irq(struct cps4019_charger_data *charger, int irq)
{
	int i;

	for (i = 0; i < CPS4019_GPIO_MAX; i++)
		if (charger->pdata->gpios[i].irq == irq)
			return &charger->pdata->gpios[i];

	return NULL;
}

static void cps4019_test_read(struct cps4019_charger_data *charger)
{
#if IS_ENABLED(CONFIG_ENG_BATTERY_CONCEPT)
	static const u16 reg[17] = {
		0x0020, 0x0021, 0x0028, 0x002C, 0x0030, 0x0044, 0x0045, 0x0046, 0x0047, 0x006A,
		0x006B, 0x006C, 0x006D, 0x006F, 0x0070, 0x007E, 0x007F
	};
	u8 temp[17] = { 0, };
	char buf[512] = { 0, };
	int i, ret, len = 0;

	for (i = 0; i < 17; i++) {
		ret = cps4019_reg_read(charger->client, reg[i], &temp[i]);
		if (ret < 0) {
			pr_info("%s: failed to read i2c!!!(ret = %d)\n", __func__, ret);
			return;
		}
	}

	for (i = 0; i < 17; i++) {
		snprintf(buf + len, 512 - len, "0x%04x:0x%02x ", reg[i], temp[i]);
		len = strlen(buf);
	}

	pr_info("%s: %s\n", __func__, buf);
#endif
}

static bool cps4019_is_on_pad(struct cps4019_charger_data *charger)
{
	sb_data det_result = CPS4019_GPIO_GP1;

	sb_vote_get_result(charger->det_vote, &det_result);
	return cps4019_get_gpio_value(&charger->pdata->gpios[det_result]);
}

static bool cps4019_is_phm(struct cps4019_charger_data *charger)
{
	sb_data phm_result = WPC_PHM_OFF;

	sb_vote_get_result(charger->phm_vote, &phm_result);
	return (phm_result != WPC_PHM_OFF);
}

static bool cps4019_is_vout_off(struct cps4019_charger_data *charger)
{
	sb_data vm = SB_WRL_VM_INIT;

	sb_vote_get_result(charger->vout_off_vote, &vm);
	return (vm == SB_WRL_VM_OFF);
}

static bool cps4019_check_chg_in_status(struct cps4019_charger_data *charger)
{
	union power_supply_propval value = {0, };
	int vrect, vout;

	psy_do_property(charger->pdata->charger_name, get,
		POWER_SUPPLY_PROP_POWER_NOW, value);
	if (value.intval == ACOK_INPUT)
		return true;

	vrect = cps4019_get_adc(charger->client, CPS4019_ADC_VRECT);
	if (vrect <= 0)
		return false;

	vout = cps4019_get_adc(charger->client, CPS4019_ADC_VOUT);
	return ((vrect > (vout + 10)) && (vout > VOUT_MIN_LEVEL));
}

static bool cps4019_wait_chg_in_ok(struct cps4019_charger_data *charger, int retry)
{
	bool chg_in_ok;

	do {
		if (!cps4019_is_on_pad(charger)) {
			pr_info("%s: pad is off!\n", __func__);
			break;
		}

		chg_in_ok = cps4019_check_chg_in_status(charger);

		pr_info("%s: chg_in_ok = %d, retry = %d\n", __func__, chg_in_ok, retry);

		if (chg_in_ok)
			return true;
		msleep(300);

		retry--;
	} while (retry > 0);

	return false;
}

static bool cps4019_rx_ic_power_on_check(struct cps4019_charger_data *charger)
{
	int ret = 0;
	u8 data = 0;

	ret = cps4019_reg_read(charger->client, CPS4019_STATUS_L_REG, &data);
	pr_info("%s: ret = %d, data = 0x%x\n", __func__, ret, data);
	return (ret >= 0) && (data & CPS4019_STAT_VRECT_MASK);
}

static bool cps4019_is_rx_mode(struct cps4019_charger_data *charger)
{
	int ret = 0;
	u8 data = 0;

	ret = cps4019_reg_read(charger->client, CPS4019_SYS_OP_MODE_REG, &data);
	if (ret < 0)
		return false;

	ret = (data & CPS4019_RX_MODE_MASK) >> CPS4019_RX_MODE_SHIFT;
	return (ret == CPS4019_RX_MODE_WPC_BPP);
}

static bool cps4019_is_app2_request(struct cps4019_charger_data *charger)
{
	int ret = 0;
	u8 data = 0;

	ret = cps4019_reg_read(charger->client, CPS4019_STATUS_H_REG, &data);
	if (ret < 0)
		return false;

	return (data & CPS4019_APP2_REQUEST_MASK);
}

static int cps4019_get_ping_duration(struct cps4019_charger_data *charger)
{
	/* ping * 2 + margin */
	return (charger->pdata->ping_duration > 0) ?
		((charger->pdata->ping_duration * 3) + (charger->pdata->ping_duration / 2)) :
		500;
}

static void cps4019_rx_ic_reset(struct cps4019_charger_data *charger, bool hard_reset)
{
	int ping_duration = PHM_DETACH_DELAY;

	if (!gpio_is_valid(charger->pdata->wpc_en)) {
		pr_info("%s: could not set the wpc_en!!!\n", __func__);
		return;
	}

	if (hard_reset) {
		sb_vote_set(charger->en_vote, VOTER_WPC_DET, true, false);
		__pm_wakeup_event(charger->phm_free_ws, jiffies_to_msecs(ping_duration + 300));
		queue_delayed_work(charger->wqueue,
			&charger->phm_free_work, msecs_to_jiffies(ping_duration));
	} else {
		ping_duration = cps4019_get_ping_duration(charger);

		sb_vote_set(charger->en_vote, VOTER_WPC_MODE, true, false);
		msleep(ping_duration);
		sb_vote_set(charger->en_vote, VOTER_WPC_MODE, false, true);
	}

	pr_info("%s: ping_duration = %d, hard_reset = %s\n",
		__func__, ping_duration, GET_BOOL_STR(hard_reset));
}

static int cps4019_get_target_vout(struct cps4019_charger_data *charger)
{
	sb_data vm = SB_WRL_VM_INIT, tvout = 4500,
		vout = 4500, vbat = 0, margin = 0;

	sb_vote_get_result(charger->vm_vote, &vm);
	vbat = cps4019_get_adc(charger->client, CPS4019_ADC_VSYS);

	switch (vm) {
	case SB_WRL_VM_DC:
		/* Vout = Vsys + (ichg * Ron chg) */
		sb_vote_get_result(charger->dc_high_vout_vote, &vout);
		vout = vout * SB_WRL_VOUT_STEP;
		sb_vote_get_result(charger->ichg_vote, &margin);
		margin = margin * SB_WRL_ICHG_STEP;

		tvout = vbat + (margin * 540 / 1000) - 150;
		break;
	case SB_WRL_VM_BT:
		/* Vout = Vsys + vbat headroom */
		sb_vote_get_result(charger->low_vout_vote, &vout);
		vout = vout * SB_WRL_VOUT_STEP;
		sb_vote_get_result(charger->bt_vbat_hdr_vote, &margin);
		margin = margin * SB_WRL_HDR_STEP;

		tvout = (vbat <= (vout - margin)) ?
			(vout - 150) : (vbat + margin - 150);
		break;
	case SB_WRL_VM_FX:
		sb_vote_get_result(charger->vout_vote, &vout);
		vout = vout * SB_WRL_VOUT_STEP;
		margin = 160;

		tvout = ((vout - margin) < tvout) ?
			(vout - margin) : tvout;
		break;
	default:
		break;
	}

	pr_info("%s: [%s] tv = %lld, vo = %lld, vb = %lld, m = %lld\n",
		__func__, get_vout_mode_str(vm), tvout, vout, vbat, margin);
	return tvout;
}

static int cps4019_unsafe_vout_check(struct cps4019_charger_data *charger)
{
	int vout;
	int target_vout;

	vout = cps4019_get_adc(charger->client, CPS4019_ADC_VOUT);
	target_vout = cps4019_get_target_vout(charger);

	pr_info("%s: vout(%d) target_vout(%d)\n", __func__, vout, target_vout);
	if (vout < target_vout)
		return 1;
	return 0;
}

static bool cps4019_send_cs100(struct cps4019_charger_data *charger)
{
	int i = 0;

	for (i = 0; i < 6; i++) {
		if (cps4019_reg_write(charger->client,
				CPS4019_CHG_STATUS_REG, 100) < 0)
			return false;

		if (cps4019_set_cmd_reg(charger,
				CPS4019_CMD_SEND_CHG_STS_MASK, CPS4019_CMD_SEND_CHG_STS_MASK) < 0)
			return false;

		msleep(250);
	}

	return true;
}

static bool cps4019_wpc_send_tx_uno_mode_disable(struct cps4019_charger_data *charger, int cnt)
{
	bool ic_off_state = false;
	int i;

	for (i = 0; i < cnt; i++) {
		pr_info("%s: (cnt = %d, ic_off_state = %d)\n", __func__, i, ic_off_state);
		if (!cps4019_rx_ic_power_on_check(charger)) {
			if (ic_off_state)
				break;

			ic_off_state = true;
		}
		cps4019_send_packet(charger, CPS4019_HEADER_AFC_CONF, 0x19, 0xFF);
		msleep(100);
	}

	return ic_off_state;
}

static bool cps4019_send_eop(struct cps4019_charger_data *charger, int healt_mode)
{
	int i = 0;
	int ret = 0;

	if (!charger->wc_w_state) {
		pr_info("%s: prevent to send eop\n", __func__);
		return false;
	}

	switch (healt_mode) {
	case POWER_SUPPLY_HEALTH_COLD:
	case POWER_SUPPLY_HEALTH_OVERHEAT:
	case POWER_SUPPLY_EXT_HEALTH_OVERHEATLIMIT:
		pr_info("%s: ept-ot\n", __func__);
		for (i = 0; i < CMD_CNT; i++) {
			ret = cps4019_reg_write(charger->client,
				CPS4019_EPT_REG, CPS4019_EPT_OVER_TEMP);
			if (ret >= 0) {
				cps4019_set_cmd_reg(charger,
					CPS4019_CMD_SEND_EOP_MASK, CPS4019_CMD_SEND_EOP_MASK);
				msleep(250);
			} else
				break;
		}
		break;
	case POWER_SUPPLY_EXT_HEALTH_UNDERVOLTAGE:
		pr_info("%s: ept-undervoltage\n", __func__);
		if (is_d2d_pad_type(charger->now_tx.info.pad_type))
			return cps4019_wpc_send_tx_uno_mode_disable(charger, 30);

		ret = cps4019_reg_write(charger->client,
			CPS4019_EPT_REG, CPS4019_EPT_OVER_TEMP);
		if (ret >= 0) {
			cps4019_set_cmd_reg(charger,
				CPS4019_CMD_SEND_EOP_MASK, CPS4019_CMD_SEND_EOP_MASK);
			msleep(250);
		}
		break;
	default:
		break;
	}

	return !cps4019_rx_ic_power_on_check(charger);
}

static int cps4019_set_fcc_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;
	union power_supply_propval value = { v, };

	return psy_do_property(charger->pdata->charger_name, set,
		POWER_SUPPLY_PROP_CURRENT_NOW, value);
}

static void cps4019_set_vote_by_vm(struct cps4019_charger_data *charger,
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

static int cps4019_check_charge_mode(struct cps4019_charger_data *charger)
{
	union power_supply_propval value = {0, };
	int vbat = 0, vout = 0, cv_thr = 0, half_thr = 0;

	if (!charger->wc_w_state || cps4019_is_phm(charger) || cps4019_is_vout_off(charger)) {
		pr_info("%s: prevent func... now charge_mode(%d)\n", __func__, charger->charge_mode);
		return charger->charge_mode;
	}

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_VOLTAGE_MAX, value);
	cv_thr = value.intval - charger->pdata->cc_cv_threshold;
	half_thr = value.intval - charger->pdata->half_bridge_threshold;

	psy_do_property(charger->pdata->fuelgauge_name, get,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, value);
	vbat = value.intval;
	vout = cps4019_get_adc(charger->client, CPS4019_ADC_VOUT);
	pr_info("%s: vbat = %d, vout = %d, cv_thr = %d, half_thr = %d\n",
		__func__, vbat, vout, cv_thr, half_thr);

	switch (charger->charge_mode) {
	case SB_WRL_CHG_MODE_HALF_BRIDGE:
		return SB_WRL_CHG_MODE_HALF_BRIDGE;

	case SB_WRL_CHG_MODE_CV:
		if ((vbat >= half_thr) && (vout <= charger->pdata->half_bridge_vout))
			return SB_WRL_CHG_MODE_HALF_BRIDGE;
		break;

	case SB_WRL_CHG_MODE_CC:
		if (vbat >= cv_thr)
			return SB_WRL_CHG_MODE_CV;
		break;

	default:
		pr_info("%s: invalid chg_mode(%d)\n", __func__, charger->charge_mode);
		break;
	}

	return charger->charge_mode;
}

static int cps4019_get_vote_type(int charge_mode)
{
	if (charge_mode == SB_WRL_CHG_MODE_HALF_BRIDGE)
		return VOTER_WPC_HALF_BRIDGE;

	return VOTER_CABLE;
}

static int cps4019_set_charge_mode(struct cps4019_charger_data *charger, int charge_mode)
{
	switch (charge_mode) {
	case SB_WRL_CHG_MODE_HALF_BRIDGE:
	case SB_WRL_CHG_MODE_CC:
	case SB_WRL_CHG_MODE_CV:
	{
		union sb_wrl_vm temp_vm;

		pr_info("%s: charge_mode (%d <-- %d)\n",
			__func__, charge_mode, charger->charge_mode);

		if (charger->charge_mode != charge_mode) {
			temp_vm.value = charger->now_tx.chg_mode[charger->charge_mode];
			cps4019_set_vote_by_vm(charger,
				cps4019_get_vote_type(charger->charge_mode), false, &temp_vm);
		}

		charger->charge_mode = charge_mode;
		temp_vm.value = charger->now_tx.chg_mode[charge_mode];
		cps4019_set_vote_by_vm(charger,
			cps4019_get_vote_type(charge_mode), cps4019_is_on_pad(charger), &temp_vm);
	}
		break;
	default:
		pr_info("%s: invalid mode(%d)\n", __func__, charge_mode);
		return -EINVAL;
	}

	return 0;
}

static int cps4019_set_phm(struct cps4019_charger_data *charger, bool set)
{
	int i = 0, ret = -1;
	bool acok, ic_power_on;

	if ((!charger->wc_w_state) || (!charger->now_tx.info.phm)) {
		pr_info("%s: phm is not set, wc_w_state = %s, set = %s\n",
			__func__, GET_BOOL_STR(charger->wc_w_state), GET_BOOL_STR(set));
		return -EPERM;
	}

	pr_info("%s: set = %d, phm_state = %d\n", __func__,
		set, charger->now_tx.info.phm_state);

	cps4019_set_gpio_irq(charger, CPS4019_GPIO_GP1, CPS_IRQ_DISNOSYNC);
	sb_vote_set(charger->det_vote, VOTER_WPC_CONTROL, true, CPS4019_GPIO_PDETB);

	if (set) {
		for (i = 0; i < 5; i++) {
			msleep(200);

			ic_power_on = cps4019_rx_ic_power_on_check(charger);
			acok = cps4019_check_chg_in_status(charger);
			pr_info("%s: check ic_power_on = %s, acok = %s\n",
				__func__, GET_BOOL_STR(ic_power_on), GET_BOOL_STR(acok));
			if (!ic_power_on) {
				ret = 0;
				goto end_phm;
			}

			cps4019_send_packet(charger, 0x28, 0x18, 0x01);
		}

		sb_vote_set(charger->det_vote, VOTER_WPC_CONTROL, false, CPS4019_GPIO_GP1);
		cps4019_set_gpio_irq(charger, CPS4019_GPIO_GP1, CPS_IRQ_EN);
	} else {
		int ping_duration = charger->now_tx.info.phm_time;

		ic_power_on = cps4019_rx_ic_power_on_check(charger);
		if (ic_power_on) {
			acok = cps4019_check_chg_in_status(charger);
			sb_vote_set(charger->det_vote, VOTER_WPC_CONTROL, false, CPS4019_GPIO_GP1);
			pr_info("%s: no need to clear phm(ic = %s, acok = %s)\n",
				__func__, GET_BOOL_STR(ic_power_on), GET_BOOL_STR(acok));
			ret = 0;
			goto end_phm;
		}

		cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, CPS_IRQ_DISNOSYNC);
		for (i = 0; i < 2; i++) {
			/* power hold mode exit need 2 pings when wpc_en high status */
			sb_vote_set(charger->en_vote, VOTER_WPC_TX_PHM, true, false);
			msleep(ping_duration);
			sb_vote_set(charger->en_vote, VOTER_WPC_TX_PHM, false, true);
			msleep(100);

			/* Power transfer phase have to keep  over 5sec to Vout LDO turn on
			 * if Vrect is lower than target Vrect.
			 * Thus wait chg_in_ok over 5sec.
			 */
			acok = cps4019_wait_chg_in_ok(charger, 20);
			if (acok) {
				msleep(300);
				acok = cps4019_check_chg_in_status(charger);
				if (!acok) {
					cps4019_rx_ic_reset(charger, false);
					cps4019_wait_chg_in_ok(charger, 8);
					pr_err("%s: chg_in err!\n", __func__);
				} else {
					sb_vote_set(charger->det_vote, VOTER_WPC_CONTROL, false, CPS4019_GPIO_GP1);
					ret = 0;
					goto end_phm;
				}
			}
		}
	}

end_phm:
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, CPS_IRQ_EN);
	pr_info("%s: end(ret = %d, acok = %d)\n", __func__, ret, acok);
	return ret;
}

static int cps4019_set_vout_off(struct cps4019_charger_data *charger, int set)
{
	sb_data vm = SB_WRL_VM_INIT;
	int i = 0, ret = -1;
	bool phm;

	phm = cps4019_is_phm(charger);
	sb_vote_get_result(charger->vm_vote, &vm);
	pr_info("%s: set = %d, vm = %llx, phm = %d\n", __func__, set, vm, phm);

	if (!charger->wc_w_state || phm) {
		sb_vote_set(charger->vm_vote, VOTER_WPC_VM_PHM, (set), SB_WRL_VM_OFF);
		sb_vote_set(charger->ldo_vote, VOTER_WPC_VM_PHM, (set), WPC_LDO_OFF);
		ret = 0;
		goto end_vout_off;
	}

	if (set) {
		sb_vote_set(charger->vm_vote, VOTER_WPC_VM_PHM, true, SB_WRL_VM_OFF);
		msleep(100);
		sb_vote_set(charger->ldo_vote, VOTER_WPC_VM_PHM, true, WPC_LDO_OFF);

		for (i = 0; i < 3; i++) {
			if (cps4019_check_chg_in_status(charger)) {
				msleep(200);
			} else {
				ret = 0;
				break;
			}
		}
	} else {
		sb_vote_set(charger->ldo_vote, VOTER_WPC_VM_PHM, false, WPC_LDO_ON);
		msleep(150);
		sb_vote_set(charger->vm_vote, VOTER_WPC_VM_PHM, false, SB_WRL_VM_OFF);
		sb_vote_set(charger->vm_vote, VOTER_WPC_CONTROL, true, SB_WRL_VM_INIT);

		for (i = 0; i < 5; i++) {
			if (cps4019_wait_chg_in_ok(charger, 2)) {
				ret = 0;
				break;
			}

			sb_vote_refresh(charger->ldo_vote);
			msleep(150);
		}

		sb_vote_set(charger->vm_vote, VOTER_WPC_CONTROL, false, SB_WRL_VM_INIT);
	}

end_vout_off:
	return ret;
}

static int cps4019_set_vout_off_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;
	int ret = 0;

	ret = cps4019_set_vout_off(charger, (v == SB_WRL_VM_OFF));

	/* clear phm wa flags */
	if (v == SB_WRL_VM_OFF) {
		/* recover ssp missing flag */
		charger->ssp_missing = false;
		/* recover phm error vote */
		sb_vote_set(charger->phm_vote, VOTER_WPC_ERROR, false, WPC_PHM_OFF);
	}
	pr_info("%s: v = %lld, ret = %d\n", __func__, v, ret);
	return ret;
}

#define MAX_SSP_MISSING_CNT	5
static void cps4019_check_ssp_missing(struct cps4019_charger_data *charger)
{
	if (!charger->ssp_missing) {
		charger->ssp_missing_count = 0;
		return;
	}

	if (++charger->ssp_missing_count < MAX_SSP_MISSING_CNT) {
		charger->ssp_missing = false;
		return;
	}

	sb_wrl_inc_error(SB_WRL_ERROR_TYPE_SSP_MISSING);
	pr_err("%s: prevent to set phm because of ssp missing\n", __func__);
}

static int cps4019_set_phm_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;
	int ret = 0;

	pr_info("%s: v = %lld, charger->charge_mode = %d\n",
		__func__, v, charger->charge_mode);

	switch (v) {
	case WPC_PHM_FEPT_ON:
		cps4019_send_eop(charger, POWER_SUPPLY_HEALTH_OVERHEAT);
		break;
	case WPC_PHM_EPT_ON:
		if (!charger->now_tx.info.phm_state) {
			ret = cps4019_set_phm(charger, true);
			charger->now_tx.info.phm_state = (ret == 0);
		}

		if ((ret < 0) || cps4019_rx_ic_power_on_check(charger)) {
			/* send ept-ot if phm was not set or was pad did not support. */
			pr_info("%s: send ept-ot if PHM was not set or PAD did not support PHM.\n",
				__func__);
			cps4019_send_eop(charger, POWER_SUPPLY_HEALTH_OVERHEAT);
		}
		break;
	case WPC_PHM_ON:
		if (!charger->now_tx.info.phm_state) {
			ret = cps4019_set_phm(charger, true);
			charger->now_tx.info.phm_state = (ret == 0);
		}

		if (ret < 0) {
			__pm_wakeup_event(charger->power_hold_chk_ws,
				jiffies_to_msecs(WAKE_LOCK_TIME_OUT));
			queue_delayed_work(charger->wqueue,
				&charger->power_hold_chk_work, msecs_to_jiffies(PHM_CHK_DELAY));
		}
		break;
	case WPC_PHM_OFF:
	default:
		if (!charger->wc_w_state) {
			if (charger->now_tx.info.phm_state)
				cps4019_rx_ic_reset(charger, true);
		} else {
			if (charger->now_tx.info.phm_state) {
				cps4019_check_ssp_missing(charger);

				charger->force_wpc_det_chk = true;
				ret = cps4019_set_phm(charger, false);
				charger->now_tx.info.phm_state = (ret != 0);

				__pm_wakeup_event(charger->phm_free_ws, jiffies_to_msecs(1000));
				queue_delayed_work(charger->wqueue, &charger->phm_free_work, 0);
			}
		}
		break;
	}

	return ret;
}

static int cps4019_set_ldo_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;
	sb_data vm = SB_WRL_VM_INIT;
	bool acok = false;
	int ret = 0;

	if (!charger->wc_w_state)
		return 0;

	sb_vote_get_result(charger->vm_vote, &vm);
	acok = cps4019_check_chg_in_status(charger);
	pr_info("%s: v = %lld, vm = %lld, acok = %d\n", __func__, v, vm, acok);

	if ((v == WPC_LDO_OFF) && (vm == SB_WRL_VM_OFF))
		return 0;
	if ((v == WPC_LDO_ON) && acok)
		return 0;

	ret = cps4019_reg_update(charger->client, CPS4019_COMMAND_L_REG,
		CPS4019_CMD_TOGGLE_LDO_MASK, CPS4019_CMD_TOGGLE_LDO_MASK);

	pr_info("%s: v = %lld, ret = %d\n", __func__, v, ret);
	return ret;
}

static int cps4019_set_vm_phm(void *pdata, int event, int en, sb_data data)
{
	struct cps4019_charger_data *charger = pdata;
	int vm_ret, phm_ret;

	if (pdata == NULL)
		return -EINVAL;

	vm_ret = sb_vote_set(charger->vout_off_vote, event, en,
		(data == WPC_PHM_OFF) ? SB_WRL_VM_INIT : SB_WRL_VM_OFF);
	phm_ret = sb_vote_set(charger->phm_vote, event, en, data);

	return !vm_ret && !phm_ret;
}

static int cps4019_get_vm_phm(void *pdata, sb_data *data)
{
	struct cps4019_charger_data *charger = pdata;

	if (pdata == NULL)
		return -EINVAL;

	*data = (cps4019_is_vout_off(charger) || cps4019_is_phm(charger));
	return 0;
}

static int cps4019_set_dc_hdr_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;

	if (!charger->wc_w_state)
		return 0;

	return cps4019_set_byte(charger->client, CPS4019_DC_VOUT_HDR_REG, v);
}

static int cps4019_set_ichg_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;

	if (!charger->wc_w_state)
		return 0;

	return cps4019_set_byte(charger->client, CPS4019_I_CHG_SET_REG, v);
}

static int cps4019_set_dc_high_vout_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;

	if (!charger->wc_w_state)
		return 0;

	return cps4019_set_short(charger->client, CPS4019_DC_VOUT_HIGHEST_L_REG, v);
}

static int cps4019_set_low_vout_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;

	if (!charger->wc_w_state)
		return 0;

	return cps4019_set_short(charger->client, CPS4019_VOUT_LOWEST_L_REG, v);
}

static int cps4019_set_bt_vbat_hdr_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;

	if (!charger->wc_w_state)
		return 0;

	return cps4019_set_byte(charger->client, CPS4019_VBAT_HEADROOM_REG, v);
}

static int cps4019_set_bt_hdr_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;

	if (!charger->wc_w_state)
		return 0;

	return cps4019_set_byte(charger->client, CPS4019_BT_VOUT_HDR_REG, v);
}

static int cps4019_set_vout_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;

	if (!charger->wc_w_state)
		return 0;

	return cps4019_set_short(charger->client, CPS4019_VOUT_SET_L_REG, v);
}

static int cps4019_set_fx_hdr_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;

	if (!charger->wc_w_state)
		return 0;

	return cps4019_set_byte(charger->client, CPS4019_FX_VOUT_HDR_REG, v);
}

static void cps4019_get_vm(struct cps4019_charger_data *charger, unsigned int v, union sb_wrl_vm *vm)
{
	sb_data temp = 0;

	/* Init vm */
	vm->value = 0;
	vm->base.type = v;

	switch (v) {
	case SB_WRL_VM_FX:
		if (!sb_vote_get_result(charger->vout_vote, &temp))
			vm->fx.vout = temp;
		if (!sb_vote_get_result(charger->fx_hdr_vote, &temp))
			vm->fx.hdr = temp;
		break;
	case SB_WRL_VM_DC:
		if (!sb_vote_get_result(charger->dc_hdr_vote, &temp))
			vm->dc.hdr = temp;
		if (!sb_vote_get_result(charger->ichg_vote, &temp))
			vm->dc.ichg = temp;
		if (!sb_vote_get_result(charger->dc_high_vout_vote, &temp))
			vm->dc.high_vout = temp;
		break;
	case SB_WRL_VM_BT:
		if (!sb_vote_get_result(charger->low_vout_vote, &temp))
			vm->bt.low_vout = temp;
		if (!sb_vote_get_result(charger->bt_vbat_hdr_vote, &temp))
			vm->bt.vbat_hdr = temp;
		if (!sb_vote_get_result(charger->bt_hdr_vote, &temp))
			vm->bt.hdr = temp;
		break;
	case SB_WRL_VM_INIT:
	case SB_WRL_VM_OFF:
	case SB_WRL_VM_WAKE:
	case SB_WRL_VM_CM:
	case SB_WRL_VM_APM:
		break;
	}
}

static int cps4019_set_vm_vote(void *data, sb_data v)
{
	struct cps4019_charger_data *charger = data;
	union sb_wrl_vm new_vm =  { 0, };
	int ret = 0;
	u8 chk_vm_data = 0;

	/* set full turn on(charging current) state to default */
	sb_vote_set(charger->fcc_vote, VOTER_WPC_MODE, false, 0);

	/* set in2bat state to default */
	sb_vote_set_f(VN_IN2BAT, VOTER_WPC_MODE, false, 0);

	if (v < 0 || v >= SB_WRL_VM_MAX) {
		pr_info("%s: invalid type (%lld)\n", __func__, v);
		ret = -EINVAL;
		goto end_vm;
	}

	if (!charger->wc_w_state) {
		pr_info("%s: skip vm vote during discharging\n", __func__);
		v = SB_WRL_VM_INIT;
		goto end_vm;
	}

	/* clear min vout */
	sb_vote_set(charger->low_vout_vote, VOTER_WPC_MODE, false, 0);

	/* update new_vm */
	cps4019_get_vm(charger, v, &new_vm);

	/* set min vout */
	sb_vote_set(charger->low_vout_vote, VOTER_WPC_MODE, true,
		((v != SB_WRL_VM_BT) ? (3500 / SB_WRL_VOUT_STEP) : new_vm.bt.low_vout));

	ret = cps4019_reg_update(charger->client, CPS4019_VOUT_MODE_REG,
		((v) << CPS4019_VOUT_MODE_SHIFT), CPS4019_VOUT_MODE_MASK);
	if (ret < 0) {
		pr_err("%s: failed to set vm(%d)\n", __func__, ret);
		goto end_vm;
	}

	/* check vm */
	ret = cps4019_reg_read(charger->client, CPS4019_VOUT_MODE_REG, &chk_vm_data);
	pr_info("%s: vout mode(0x%x), ret(%d), vm(0x%llx)\n",
		__func__, chk_vm_data, ret, new_vm.value);
	ret = ((ret >= 0) && (((u8)v) == chk_vm_data)) ? 0 : -1;

end_vm:
	/* set the in2bat to false if dc */
	sb_vote_set_f(VN_IN2BAT, VOTER_WPC_MODE, ((!ret) && (v == SB_WRL_VM_DC)), 0);

	/* FULL TURN ON - DC */
	sb_vote_set(charger->fcc_vote, VOTER_WPC_MODE, ((!ret) && (v == SB_WRL_VM_DC)), INT_MAX);
	return ret;
}

void cps4019_send_multi_packet(struct cps4019_charger_data *charger, u8 header, u8 rx_data_com, u8 *data_val, int data_size)
{
	int ret;
	int i;

	ret = cps4019_reg_write(charger->client, CPS4019_PPP_HEADER_REG, header);
	ret = cps4019_reg_write(charger->client, CPS4019_RX_DATA_COM_REG, rx_data_com);

	data_size = (data_size > CPS4019_RX_DATA_MAX) ? CPS4019_RX_DATA_MAX : data_size;
	for (i = 0; i < data_size; i++)
		ret = cps4019_reg_write(charger->client, CPS4019_RX_DATA_VALUE_REG + i, data_val[i]);

	cps4019_set_cmd_reg(charger, CPS4019_CMD_SEND_RX_DATA_MASK, CPS4019_CMD_SEND_RX_DATA_MASK);
}

static bool cps4019_send_watchdog_err_packet(struct cps4019_charger_data *charger)
{
	bool ic_off_state = false;
	int i = 0;

	pr_info("%s\n", __func__);
	for (i = 0; i <= 3; i++) {
		if (!cps4019_rx_ic_power_on_check(charger)) {
			if (ic_off_state)
				break;

			ic_off_state = true;
		}

		cps4019_send_multi_packet(charger, 0x18, 0xE7, NULL, 0);
		msleep(200);
	}

	return ic_off_state;
}

static int cps4019_get_firmware_version(struct cps4019_charger_data *charger)
{
	int version = -1;
	int ret;

	if (!cps4019_is_on_pad(charger))
		return version;

	ret = cps4019_reg_bulk_read(charger->client, CPS4019_FW_MAJOR_REV_L_REG, (u8 *)&version, 4);
	if (ret < 0)
		return ret;

	pr_info("%s: fw_version = 0x%x\n", __func__, version);
	return version;
}

static void cps4019_fw_update(struct cps4019_charger_data *charger, int mode)
{
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_GP1, CPS_IRQ_DISNOSYNC);
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, CPS_IRQ_DISNOSYNC);
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_INTB, CPS_IRQ_DISNOSYNC);

	/* release pm */
	__pm_relax(charger->wpc_ws);
	__pm_relax(charger->det_ws);
	__pm_relax(charger->wpc_data_check_ws);
	__pm_relax(charger->wpc_id_check_ws);
	__pm_relax(charger->wpc_id_request_ws);
	__pm_relax(charger->power_hold_chk_ws);
	__pm_relax(charger->retry_phm_free_ws);
	__pm_relax(charger->darkzone_ws);
	__pm_relax(charger->align_check_ws);
	__pm_relax(charger->cs100_ws);
	__pm_relax(charger->check_auth_ws);

	/* cancel works */
	cancel_delayed_work(&charger->wpc_isr_work);
	cancel_delayed_work(&charger->wpc_det_work);
	cancel_delayed_work(&charger->wpc_id_check_work);
	cancel_delayed_work(&charger->wpc_id_request_work);
	cancel_delayed_work(&charger->power_hold_chk_work);
	cancel_delayed_work(&charger->retry_phm_free_work);
	cancel_delayed_work(&charger->darkzone_work);
	cancel_delayed_work(&charger->align_check_work);
	cancel_delayed_work(&charger->cs100_work);
	cancel_delayed_work(&charger->auth_done_work);

	sb_vote_set_f(VN_CHG_EN, VOTER_WPC_FW, true, SB_CHG_MODE_BUCK_OFF);
	sb_vote_set(charger->vout_off_vote, VOTER_WPC_FW, true, SB_WRL_VM_INIT);
	sb_vote_set(charger->phm_vote, VOTER_WPC_FW, true, WPC_PHM_OFF);
	sb_vote_set(charger->vm_vote, VOTER_WPC_FW, true, SB_WRL_VM_INIT);

	sb_wrl_fw_update(charger->fw, mode);
}

static void cps4019_fw_result_cb(struct i2c_client *i2c, int state)
{
	struct cps4019_charger_data *charger = i2c_get_clientdata(i2c);

	pr_info("%s: finished fw (state = %d)\n", __func__, state);

	sb_vote_set(charger->vm_vote, VOTER_WPC_FW, false, SB_WRL_VM_INIT);
	sb_vote_set(charger->phm_vote, VOTER_WPC_FW, false, WPC_PHM_OFF);
	sb_vote_set(charger->vout_off_vote, VOTER_WPC_FW, false, SB_WRL_VM_INIT);
	sb_vote_set_f(VN_CHG_EN, VOTER_WPC_FW, false, 0);
	sb_vote_set(charger->det_vote, VOTER_WPC_FW, false, CPS4019_GPIO_GP1);

	cps4019_set_gpio_irq(charger, CPS4019_GPIO_GP1, CPS_IRQ_EN);
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, CPS_IRQ_EN);
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_INTB, CPS_IRQ_EN);

	cps4019_rx_ic_reset(charger, true);
}

static bool cps4019_phm_state(struct cps4019_charger_data *charger)
{
	return !cps4019_rx_ic_power_on_check(charger);
}

static void cps4019_phm_check_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, power_hold_chk_work.work);

	if (cps4019_is_on_pad(charger) && cps4019_is_phm(charger)) {
		if (cps4019_phm_state(charger)) {
			charger->now_tx.info.phm_state = true;
			goto end_phm_chk;
		}

		if (charger->phm_chk_cnt < PHM_CHK_CNT) {
			pr_err("%s: retry to set phm, count = %d\n",
				__func__, charger->phm_chk_cnt);

			charger->phm_chk_cnt++;
			sb_vote_refresh(charger->phm_vote);
			return;
		}

		pr_err("%s: stop to retry phm\n", __func__);
		sb_wrl_inc_error(SB_WRL_ERROR_TYPE_PHM_FAIL);
		sb_vote_set(charger->phm_vote, VOTER_WPC_ERROR, true, WPC_PHM_OFF);
	}

end_phm_chk:
	charger->phm_chk_cnt = 0;
	__pm_relax(charger->power_hold_chk_ws);
}

static void cps4019_phm_free_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, phm_free_work.work);

	pr_info("%s: wc_w_state = %d\n", __func__, charger->wc_w_state);

	/* do not use the cps4019_is_on_pad() */
	if (charger->wc_w_state) {
		if (cps4019_phm_state(charger) && (charger->phm_chk_cnt < PHM_CHK_CNT)) {
			/* retry phm free */
			charger->phm_chk_cnt++;
			__pm_stay_awake(charger->retry_phm_free_ws);
			queue_delayed_work(charger->wqueue,
				&charger->retry_phm_free_work, msecs_to_jiffies(PHM_CHK_DELAY));
		} else {
			/* restart wpc auth */
			charger->phm_chk_cnt = 0;
			__pm_stay_awake(charger->det_ws);
			queue_delayed_work(charger->wqueue,
				&charger->wpc_det_work, msecs_to_jiffies(500));
		}
	}

	/* recover wpc_en to low */
	sb_vote_set(charger->en_vote, VOTER_WPC_DET, false, true);
	__pm_relax(charger->phm_free_ws);
}

static void cps4019_retry_phm_free_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, retry_phm_free_work.work);

	pr_info("%s: phm_chk_cnt = %d, wc_w_state = %d\n",
		__func__, charger->phm_chk_cnt, charger->wc_w_state);

	if (cps4019_is_on_pad(charger) && cps4019_phm_state(charger))
		sb_vote_refresh(charger->phm_vote);

	__pm_relax(charger->retry_phm_free_ws);
}

static void cps4019_cs100_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, cs100_work.work);

	if (!cps4019_is_on_pad(charger)) {
		sb_vote_set(charger->phm_vote, VOTER_WPC_CS100, false, WPC_PHM_OFF);
		__pm_relax(charger->cs100_ws);
		return;
	}

	charger->cs100_status = cps4019_send_cs100(charger);
	sb_vote_set(charger->phm_vote, VOTER_WPC_CS100, false, WPC_PHM_OFF);

	pr_info("%s: cs100 status = %s\n", __func__, GET_BOOL_STR(charger->cs100_status));
	__pm_relax(charger->cs100_ws);
}

static int cps4019_temperature_check(struct cps4019_charger_data *charger)
{
	if (charger->temperature >= charger->pdata->tx_off_high_temp) {
		/* send TX watchdog command in high temperature */
		pr_err("%s:HIGH TX OFF TEMP:%d\n", __func__, charger->temperature);
		cps4019_send_watchdog_err_packet(charger);
	}

	return 0;
}

static bool cps4019_check_low_current(struct cps4019_charger_data *charger, int vout, int curr)
{
	if ((vout < VOUT_MIN_LEVEL) ||
		((charger->battery_status != POWER_SUPPLY_STATUS_CHARGING) &&
		(charger->battery_status != POWER_SUPPLY_STATUS_FULL))) {
		charger->low_current_start_time = 0;
		return false;
	}

	if (curr < 10) {
		struct timespec64 ts = ktime_to_timespec64(ktime_get_boottime());

		if (charger->low_current_start_time <= 0) {
			charger->low_current_start_time = ts.tv_sec;
		} else {
			unsigned long low_current_time;

			if (charger->low_current_start_time > ts.tv_sec)
				low_current_time = 0xFFFFFFFF - charger->low_current_start_time + ts.tv_sec;
			else
				low_current_time = ts.tv_sec - charger->low_current_start_time;

			pr_info("%s: now sec(%lld), state(%ld)\n", __func__,
				ts.tv_sec, low_current_time);
			return (low_current_time >= charger->pdata->low_current_expired_time);
		}
	} else
		charger->low_current_start_time = 0;

	return false;
}

static bool cps4019_check_abnormal_acok(struct cps4019_charger_data *charger)
{
	if ((charger->battery_status != POWER_SUPPLY_STATUS_CHARGING) &&
		(charger->battery_status != POWER_SUPPLY_STATUS_FULL)) {
		charger->ab_acok_count = 0;
		charger->ab_acok_start_time = 0;
		return false;
	}

	if (charger->ab_acok_count > 0) {
		struct timespec64 ts = ktime_to_timespec64(ktime_get_boottime());

		if (charger->ab_acok_start_time <= 0) {
			/* re-set ab_acok_count for re-counting abnormal acok. */
			charger->ab_acok_count = 1;
			pr_info("%s: now sec(%lld), state(%d)\n", __func__, ts.tv_sec, charger->ab_acok_count);
			charger->ab_acok_start_time = ts.tv_sec;
			/* Prevent sleep state for checking abnormal acok count. */
			__pm_wakeup_event(charger->ab_acok_ws,
				jiffies_to_msecs(HZ * (charger->pdata->ab_acok_time + 1)));
		} else {
			unsigned long ab_acok_time;

			if (charger->ab_acok_start_time > ts.tv_sec)
				ab_acok_time = 0xFFFFFFFF - charger->ab_acok_start_time + ts.tv_sec;
			else
				ab_acok_time = ts.tv_sec - charger->ab_acok_start_time;

			pr_info("%s: now sec(%lld), state(%ld, %d)\n", __func__,
				ts.tv_sec, ab_acok_time, charger->ab_acok_count);
			if (charger->ab_acok_count >= charger->pdata->ab_acok_count) {
				if (ab_acok_time <= charger->pdata->ab_acok_time)
					return true;
			} else {
				if (ab_acok_time <= charger->pdata->ab_acok_time)
					return false;
			}
			charger->ab_acok_count = 0;
			charger->ab_acok_start_time = 0;
		}
	} else
		charger->ab_acok_start_time = 0;

	return false;
}

static bool cps4019_check_darkzone_state(struct cps4019_charger_data *charger, int vrect, int vout)
{
	int target_vout = 0;

	if ((charger->store_mode) || (!charger->now_tx.info.darkzone)) {
		charger->darkzone_reset = false;
		goto init_darkzone_flag;
	}

	target_vout = cps4019_get_target_vout(charger);
	pr_info("%s: %d <-> %d, %d\n", __func__, target_vout, vrect, vout);

	if ((vrect < target_vout) && (vout < target_vout) &&
		(charger->battery_health == POWER_SUPPLY_HEALTH_GOOD) &&
		(!charger->swelling_mode) &&
		(charger->battery_chg_mode != SEC_BATTERY_CHARGING_NONE)) {
		struct timespec64 c_ts = ktime_to_timespec64(ktime_get_boottime());

		if (charger->darkzone_start_time == 0) {
			charger->darkzone_start_time = c_ts.tv_sec;
		} else {
			unsigned long darkzone_time;

			if (charger->darkzone_start_time > c_ts.tv_sec)
				darkzone_time = 0xFFFFFFFF - charger->darkzone_start_time + c_ts.tv_sec;
			else
				darkzone_time = c_ts.tv_sec - charger->darkzone_start_time;

			pr_info("%s: darkzone(%ld)\n", __func__, darkzone_time);
			return (darkzone_time >= charger->pdata->darkzone_expired_time);
		}

		return false;
	} else if (vout >= target_vout) {
		charger->darkzone_reset = false;
	}

init_darkzone_flag:
	charger->darkzone_start_time = 0;
	return false;
}

static void cps4019_darkzone_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger = container_of(work,
		struct cps4019_charger_data, darkzone_work.work);
	bool wc_w_state = cps4019_is_on_pad(charger);

	pr_info("%s: wc_w_state(%s <--> %s)\n", __func__,
		GET_BOOL_STR(charger->wc_w_state), GET_BOOL_STR(wc_w_state));
	if (!wc_w_state || !gpio_is_valid(charger->pdata->wpc_en)) {
		__pm_relax(charger->darkzone_ws);
		return;
	}

	if (charger->wc_w_state) {
		union power_supply_propval value = {0, };

		charger->darkzone_reset = true;
		value.intval = POWER_SUPPLY_EXT_HEALTH_UNDERVOLTAGE;
		psy_do_property(charger->pdata->battery_name, set,
			POWER_SUPPLY_PROP_HEALTH, value);
	}

	/* hard reset IC */
	cps4019_rx_ic_reset(charger, true);
	__pm_relax(charger->darkzone_ws);
}

static void cps4019_init_wa_flag(struct cps4019_charger_data *charger)
{
	charger->darkzone_start_time = 0;
	charger->low_current_start_time = 0;
	charger->ab_acok_count = 0;
	charger->ab_acok_start_time = 0;
}

static void cps4019_send_d2d_strength(struct cps4019_charger_data *charger, int d2d_strength)
{
	union power_supply_propval value = {d2d_strength, };

	charger->d2d_vout_strength = d2d_strength;
	psy_do_property(charger->pdata->battery_name,
		set, POWER_SUPPLY_EXT_PROP_WIRELESS_FREQ_STRENGTH, value);
}

static void cps4019_wpc_align_check_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, align_check_work.work);

	struct timespec64 current_ts = {0, };
	long checking_time = 0;
	static int d2d_vout_sum, d2d_align_work_cnt;
	int vout = 0, vout_avr = 0, freq = 0;

	if (!cps4019_is_on_pad(charger))
		goto end_align_work;

	vout = cps4019_get_adc(charger->client, CPS4019_ADC_VOUT);
	if (charger->d2d_align_check_start.tv_sec == 0) {
		charger->d2d_align_check_start = ktime_to_timespec64(ktime_get_boottime());
		d2d_align_work_cnt = 0;
		d2d_vout_sum = 0;
	}
	current_ts = ktime_to_timespec64(ktime_get_boottime());
	checking_time = current_ts.tv_sec - charger->d2d_align_check_start.tv_sec;

	d2d_vout_sum += vout;
	d2d_align_work_cnt++;
	vout_avr = d2d_vout_sum / d2d_align_work_cnt;
	freq = cps4019_get_adc(charger->client, CPS4019_ADC_OP_FRQ);

	pr_info("%s: vout(%d), vout_avr(%d), freq(%d), work_cnt(%d), checking_time(%ld)\n",
		__func__, vout, vout_avr, freq, d2d_align_work_cnt, checking_time);

	if (d2d_align_work_cnt >= ALIGN_WORK_CHK_CNT) {
		if ((vout_avr >= cps4019_get_target_vout(charger)) &&
			(freq >= ALIGN_MIN_FREQ)) {
			pr_info("%s: Finish to check (Align)\n", __func__);

			cps4019_send_d2d_strength(charger, 100);
			goto end_align_work;
		} else if (checking_time >= MISALIGN_TX_OFF_TIME) {
			pr_info("%s: Keep Misalign during %d secs (Tx off)\n",
				__func__, MISALIGN_TX_OFF_TIME);

			cps4019_send_d2d_strength(charger, 0);
			if (cps4019_rx_ic_power_on_check(charger)) {
				cps4019_wpc_send_tx_uno_mode_disable(charger, 30);
				cps4019_rx_ic_reset(charger, true);

				sb_wrl_inc_error(SB_WRL_ERROR_TYPE_MISALIGN);
				goto end_align_work;
			} else {
				queue_delayed_work(charger->wqueue,
					&charger->align_check_work, msecs_to_jiffies(ALIGN_CHK_PERIOD));
			}
		} else {
			pr_info("%s: Continue to check until %d secs (Misalign)\n",
				__func__, MISALIGN_TX_OFF_TIME);
			cps4019_send_d2d_strength(charger, 0);

			d2d_align_work_cnt = 0;
			d2d_vout_sum = 0;
			queue_delayed_work(charger->wqueue,
				&charger->align_check_work, msecs_to_jiffies(ALIGN_CHK_PERIOD));
		}
	} else {
		queue_delayed_work(charger->wqueue,
			&charger->align_check_work, msecs_to_jiffies(ALIGN_WORK_CHK_PERIOD));
	}

	return;

end_align_work:
	sb_vote_set(charger->vout_off_vote, VOTER_WPC_ALIGN_CHK, false, SB_WRL_VM_INIT);
	sb_vote_set(charger->phm_vote, VOTER_WPC_ALIGN_CHK, false, WPC_PHM_OFF);
	charger->d2d_checking_align = false;
	charger->d2d_align_check_start.tv_sec = 0;
	__pm_relax(charger->align_check_ws);
}

static void cps4019_wpc_align_check(struct cps4019_charger_data *charger, unsigned int work_delay)
{
	bool is_power_hold_mode = false, is_vout_off_mode = false;

	if (!is_d2d_wrl_type(charger->cable_type)) {
		pr_info("%s: return, cable_type(%d)\n", __func__, charger->cable_type);
		return;
	}

	if (charger->d2d_checking_align) {
		pr_info("%s: return, d2d_checking_align(%d)\n", __func__, charger->d2d_checking_align);
		return;
	}

	is_vout_off_mode = cps4019_is_vout_off(charger);
	is_power_hold_mode = cps4019_is_phm(charger);
	if (!charger->is_charging || is_power_hold_mode || is_vout_off_mode) {
		pr_info("%s: return, chg(%d), phm(%d), vout_off(%d)\n",
			__func__, charger->is_charging, is_power_hold_mode, is_vout_off_mode);
		return;
	}

	if (charger->d2d_vout_strength >= 100) {
		if (!cps4019_rx_ic_power_on_check(charger)) {
			sb_vote_set(charger->phm_vote, VOTER_WPC_TX_PHM, true, WPC_PHM_ON);
			pr_info("%s: return, set phm state to tx_phm\n", __func__);
			return;
		}

		if (!cps4019_unsafe_vout_check(charger)) {
			pr_info("%s: return, safe vout\n", __func__);
			return;
		}
	}

	pr_info("%s: start\n", __func__);
	sb_vote_set(charger->phm_vote, VOTER_WPC_ALIGN_CHK, true, WPC_PHM_OFF);
	sb_vote_set(charger->vout_off_vote, VOTER_WPC_ALIGN_CHK, true, SB_WRL_VM_INIT);

	charger->d2d_checking_align = true;
	__pm_wakeup_event(charger->align_check_ws, jiffies_to_msecs(WAKE_LOCK_TIME_OUT));
	queue_delayed_work(charger->wqueue, &charger->align_check_work, msecs_to_jiffies(work_delay));
}

static bool cps4019_check_cable_type(int chg_cable_type, int batt_cable_type)
{
	if (chg_cable_type == batt_cable_type)
		return false;

	return ((is_wireless_type(chg_cable_type) && is_nocharge_type(batt_cable_type)) ||
		(is_nocharge_type(chg_cable_type) && is_wireless_type(batt_cable_type)));
}

static int cps4019_monitor_work(struct cps4019_charger_data *charger)
{
	union power_supply_propval value = {0, };
	sb_data now_vm = SB_WRL_VM_INIT;
	union sb_wrl_vm vm = {0, };
	int vrect = 0, vout = 0, freq = 0;
	int curr = 0, vol = 0;
	u8 vm_mode = 0;
	u8 full_bridge = 0;
	int capacity;

	value.intval = SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE;
	psy_do_property(charger->pdata->fuelgauge_name, get,
		POWER_SUPPLY_PROP_CAPACITY, value);
	capacity = value.intval;

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_HEALTH, value);
	charger->battery_health = value.intval;

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_STATUS, value);
	charger->battery_status = value.intval;

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_CHARGE_NOW, value);
	charger->battery_chg_mode = value.intval;

	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, value);
	charger->swelling_mode = value.intval;

	if (!is_wireless_type(charger->cable_type)) {
		pr_debug("%s: pad off!\n", __func__);

		/* logging for battery_dump */
		snprintf(charger->d_buf, 128, "%d,%d",
			cps4019_is_on_pad(charger), cps4019_check_chg_in_status(charger));
		return 0;
	}

	sb_vote_get_result(charger->vm_vote, &now_vm);
	cps4019_get_vm(charger, now_vm, &vm);

	if (!cps4019_is_phm(charger)) {
		vrect = cps4019_get_adc(charger->client, CPS4019_ADC_VRECT);
		vout = cps4019_get_adc(charger->client, CPS4019_ADC_VOUT);
		freq = cps4019_get_adc(charger->client, CPS4019_ADC_OP_FRQ);
		curr = cps4019_get_adc(charger->client, CPS4019_ADC_IOUT);
		vol = cps4019_get_adc(charger->client, CPS4019_ADC_VSYS);

		cps4019_reg_read(charger->client, CPS4019_RECT_MODE_REG, &full_bridge);
		cps4019_reg_read(charger->client, CPS4019_VOUT_MODE_REG, &vm_mode);
	}

	pr_info("%s: vrect(%dmV) vout(%dmV) iout(%dmA) freq(%d) vbat(%dmV) SOC(%d) "
		"tx_info(0x%llx) vm_mode(0x%llx) chg_mode(%d) "
		"fw(%x) full(%x) err(%llx)\n",
		__func__, vrect, vout, curr, freq, vol, capacity,
		charger->now_tx.info_data, vm.value, charger->charge_mode,
		charger->pdata->otp_firmware_ver, full_bridge, sb_wrl_error_raw_data());

	/* logging for battery_dump */
	snprintf(charger->d_buf, 128, "%d,%d,%d,%d,%d,%d,%d,%d,"
		"0x%llx,0x%llx,%d,0x%x,0x%x,0x%llx",
		cps4019_is_on_pad(charger), cps4019_check_chg_in_status(charger),
		vrect, vout, curr, freq, vol, capacity,
		charger->now_tx.info_data, vm.value, charger->charge_mode,
		charger->pdata->otp_firmware_ver, full_bridge, sb_wrl_error_raw_data());

	if (!cps4019_is_vout_off(charger) && !cps4019_is_phm(charger)) {
		sb_vote_set(charger->vm_vote, VOTER_WPC_CONTROL, false, SB_WRL_VM_INIT);

		if (cps4019_check_darkzone_state(charger, vrect, vout)) {
			cps4019_send_eop(charger, POWER_SUPPLY_EXT_HEALTH_UNDERVOLTAGE);
			pr_info("%s: Unsafe voltage\n", __func__);

			__pm_wakeup_event(charger->darkzone_ws, jiffies_to_msecs(WAKE_LOCK_TIME_OUT));
			queue_delayed_work(charger->wqueue, &charger->darkzone_work, 0);
		} else if (cps4019_check_low_current(charger, vout, curr) ||
				cps4019_check_abnormal_acok(charger)) {
			pr_info("%s: low iout or abnormal acokr event detected!!!\n", __func__);
			sb_vote_set(charger->phm_vote, VOTER_WPC_CONTROL, true, WPC_PHM_ON);
			sb_vote_set_f(VN_CHG_EN, VOTER_WPC_CONTROL, true, SB_CHG_MODE_CHARGING_OFF);
			sb_vote_set(charger->phm_vote, VOTER_WPC_CONTROL, false, WPC_PHM_OFF);
			sb_vote_set(charger->vm_vote, VOTER_WPC_CONTROL, true, SB_WRL_VM_WAKE);
			sb_vote_set_f(VN_CHG_EN, VOTER_WPC_CONTROL, false, false);

			cps4019_init_wa_flag(charger);
		} else {
			cps4019_wpc_align_check(charger, 0);
		}

		cps4019_test_read(charger);
	} else {
		cps4019_init_wa_flag(charger);
	}

	return 0;
}

static int cps4019_get_prop_manufacturer(struct cps4019_charger_data *charger,
		union power_supply_propval *val)
{
	switch (val->intval) {
	case SB_WRL_OTP_FIRM_RESULT:
		sb_wrl_fw_get_state(charger->fw, &val->intval);
		break;
	case SB_WRL_OTP_FIRM_VER_BIN:
		sb_wrl_fw_get_version(charger->fw, &val->intval);
		break;
	case SB_WRL_OTP_FIRM_VER:
		val->intval = charger->pdata->otp_firmware_ver;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cps4019_monitor_wdt_kick(struct cps4019_charger_data *charger)
{
	if (!charger->wc_w_state || charger->watchdog_test) {
		pr_info("%s: skip watchdog kick\n", __func__);
		return 0;
	}

	return cps4019_reg_update(charger->client, CPS4019_COMMAND_L_REG,
		CPS4019_CMD_WATCHDOG_RST_MASK, CPS4019_CMD_WATCHDOG_RST_MASK);
}

static int cps4019_chg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct cps4019_charger_data *charger = power_supply_get_drvdata(psy);
	int ret = 0;

	switch ((int)psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = cps4019_get_firmware_version(charger);
		pr_info("%s: rx major firmware version 0x%x\n", __func__, ret);

		if (ret >= 0)
			val->intval = 1;
		else
			val->intval = 0;

		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = (charger->darkzone_reset) ?
			POWER_SUPPLY_EXT_HEALTH_UNDERVOLTAGE : POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->strval = charger->d_buf;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		pr_debug("%s: cable_type =%d\n ", __func__, charger->cable_type);
		val->intval = charger->cable_type;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = (cps4019_is_on_pad(charger) &&
			(cps4019_is_rx_mode(charger) || cps4019_is_app2_request(charger)));
		pr_debug("%s: is on chg pad = %d\n ", __func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = charger->temperature;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	{
		sb_data ichg = 0, fcc = 0;

		sb_vote_get_result(charger->ichg_vote, &ichg);
		ichg = ichg * SB_WRL_ICHG_STEP;
		sb_vote_get_result(charger->fcc_vote, &fcc);

		val->intval = min(ichg, fcc);
		pr_info("%s: ichg(%lld), fcc(%lld)\n", __func__, ichg, fcc);
	}
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW: /* vout */
		if (charger->wc_w_state)
			val->intval = cps4019_get_adc(charger->client, CPS4019_ADC_VOUT);
		else
			val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_ENERGY_AVG: /* vrect */
		if (charger->wc_w_state)
			val->intval = cps4019_get_adc(charger->client, CPS4019_ADC_VRECT);
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		ret = cps4019_get_prop_manufacturer(charger, val);
		break;

	/* EXT PROPERTY */
	case POWER_SUPPLY_EXT_PROP_WIRELESS_FREQ_STRENGTH:
		pr_info("%s: d2d_vout_strength = %d\n",
			__func__, charger->d2d_vout_strength);
		if (!cps4019_is_on_pad(charger))
			val->intval = 0;
		else
			val->intval = (charger->now_tx.info.phm_state) ?
				100 : charger->d2d_vout_strength;
		break;
	case POWER_SUPPLY_EXT_PROP_WIRELESS_OP_FREQ:
			val->intval = cps4019_get_adc(charger->client, CPS4019_ADC_OP_FRQ);
		pr_info("%s: Operating FQ %dkHz\n", __func__, val->intval);
			break;
	case POWER_SUPPLY_EXT_PROP_WIRELESS_OP_FREQ_STRENGTH:
		val->intval = charger->d2d_vout_strength;
		pr_info("%s: vout strength = (%d)\n",
			__func__, charger->d2d_vout_strength);
			break;
	case POWER_SUPPLY_EXT_PROP_MONITOR_WORK:
		if (cps4019_check_cable_type(charger->cable_type, val->intval)) {
			/* try to run wpc_det work for cable type */
			charger->force_wpc_det_chk = (is_wireless_type(charger->cable_type) != charger->wc_w_state);
			__pm_stay_awake(charger->det_ws);
			queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
		} else {
			cps4019_monitor_work(charger);
		}
		break;
	case POWER_SUPPLY_EXT_PROP_POWER_SHARING_CHARGE:
		val->intval = is_d2d_pad_type(charger->now_tx.info.pad_type);
		break;
	case POWER_SUPPLY_EXT_PROP_WIRELESS_TX_ID:
		val->intval = charger->now_tx.info.id;
		break;
	case POWER_SUPPLY_EXT_PROP_WIRELESS_TX_UNO_IIN:
		val->intval = cps4019_get_adc(charger->client, CPS4019_ADC_IOUT);
		break;
	case POWER_SUPPLY_EXT_PROP_WIRELESS_PHM:
		val->intval = charger->now_tx.info.phm_state;
		break;
	case POWER_SUPPLY_EXT_PROP_TX_PWR_BUDG:
		val->intval = charger->now_tx.info.tx_pwr_budg;
		break;
	case POWER_SUPPLY_EXT_PROP_WIRELESS_AUTH_ADT_STATUS:
		val->strval = get_auth_status_str(charger->now_tx.info.auth_state);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void cps4019_chg_set_muic_msg(struct cps4019_charger_data *charger,
		enum power_supply_muic_msg muic_msg)
{
	if (!cps4019_is_on_pad(charger))
		return;

	switch (muic_msg) {
	case POWER_SUPPLY_MUIC_ACOK_FALLING:
		pr_info("%s: acok falling!!\n", __func__);
		break;
	case POWER_SUPPLY_MUIC_ACOK_RISING:
		pr_info("%s: acok rising!!\n", __func__);
		charger->ab_acok_count++;
		break;
	default:
		break;
	}
}

static int cps4019_chg_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct cps4019_charger_data *charger = power_supply_get_drvdata(psy);

	switch ((int)psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (val->intval == POWER_SUPPLY_STATUS_FULL) {
			if ((charger->battery_status != POWER_SUPPLY_STATUS_FULL) ||
				(!charger->cs100_status)) {
				int work_delay = cps4019_rx_ic_power_on_check(charger) ?
					0 : VOUT_STABLILZATION_TIME;

				pr_info("%s: set green led (work_delay = %d)\n", __func__, work_delay);
				sb_vote_set(charger->phm_vote, VOTER_WPC_CS100, true, WPC_PHM_OFF);
				__pm_wakeup_event(charger->cs100_ws, jiffies_to_msecs(WAKE_LOCK_TIME_OUT));
				queue_delayed_work(charger->wqueue,
					&charger->cs100_work, msecs_to_jiffies(work_delay));
			}
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		break;
	case POWER_SUPPLY_PROP_TEMP:
		charger->temperature = val->intval;
		cps4019_temperature_check(charger);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		sb_vote_set(charger->ichg_vote, VOTER_WPC_CONTROL,
			(charger->wc_w_state), (val->intval / SB_WRL_ICHG_STEP));
		sb_vote_set(charger->fcc_vote, VOTER_CABLE, true, val->intval);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		if (cps4019_is_on_pad(charger))
			cps4019_fw_update(charger, val->intval);
		else
			pr_info("%s: prevent fw update without pad.", __func__);
		break;

	/* EXT PROPERTY */
	case POWER_SUPPLY_EXT_PROP_CHARGING_ENABLED:
		switch (val->intval) {
		case SB_CHG_MODE_CHARGING:
			charger->is_charging = true;
			break;
		case SB_CHG_MODE_BUCK_OFF:
		case SB_CHG_MODE_CHARGING_OFF:
		default:
			charger->is_charging = false;
			break;
		}
		break;
	case POWER_SUPPLY_EXT_PROP_WDT_KICK:
		cps4019_monitor_wdt_kick(charger);
		break;
	case POWER_SUPPLY_EXT_PROP_WIRELESS_CHARGE_MONITOR:
		cps4019_set_charge_mode(charger,
			cps4019_check_charge_mode(charger));
		break;
	case POWER_SUPPLY_EXT_PROP_STORE_MODE:
		charger->store_mode = val->intval;
		break;
	case POWER_SUPPLY_EXT_PROP_MUIC_MSG:
		cps4019_chg_set_muic_msg(charger, val->intval);
		break;
	case POWER_SUPPLY_EXT_PROP_TX_PWR_BUDG:
		break;
	case POWER_SUPPLY_EXT_PROP_WIRELESS_AUTH_ADT_STATUS:
		/* if 1, test mode is on. if 0, test mode is off */
		if (cps4019_is_on_pad(charger))
			cps4019_reg_update(charger->client, CPS4019_INT_ENABLE_H_REG,
				((val->intval) ? 0 : CPS4019_ADT_RECV_MASK), CPS4019_ADT_RECV_MASK);
		else
			pr_info("%s: skip set adt_status\n", __func__);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cps4019_check_to_start_auth(struct sb_wrl_tx *tx)
{
	if (!tx->info.auth)
		return -1;

	if ((tx->info.auth_state == SB_WRL_AUTH_PASS) ||
		(tx->info.auth_state == SB_WRL_AUTH_FAIL))
		return -1;

#if IS_ENABLED(CONFIG_SEC_FACTORY)
	return SB_WRL_AUTH_PASS;
#else
	return (tx->info.auth_state == SB_WRL_AUTH_NONE) ?
		SB_WRL_AUTH_START : SB_WRL_AUTH_RETRY;
#endif
}

static void cps4019_set_auth_event(struct sb_wrl_tx *tx, int auth_state)
{
	tx->info.auth_state = auth_state;

	sb_notify_call_bit(SB_NOTIFY_EVENT_MISC,
		(auth_state << BATT_MISC_EVENT_WIRELESS_AUTH_SHIFT),
		BATT_MISC_EVENT_WIRELESS_AUTH_MASK);
}

static void cps4019_delay_auth_done_work(struct cps4019_charger_data *charger, unsigned int delay)
{
	cancel_delayed_work(&charger->auth_done_work);

	__pm_stay_awake(charger->check_auth_ws);
	sb_vote_set(charger->phm_vote, VOTER_WPC_AUTH, true, WPC_PHM_OFF);
	queue_delayed_work(charger->wqueue,
		&charger->auth_done_work, msecs_to_jiffies(delay));
}

static void cps4019_auth_done_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, auth_done_work.work);
	int auth_state = SB_WRL_AUTH_NONE;

	if (!cps4019_is_on_pad(charger))
		goto end_work;

	mutex_lock(&charger->auth_lock);
	auth_state = cps4019_check_to_start_auth(&charger->now_tx);
	if (auth_state < 0)
		goto skip_work;

	if (charger->now_tx.info.auth_state != SB_WRL_AUTH_ERROR)
		cps4019_set_auth_event(&charger->now_tx, SB_WRL_AUTH_TIMEOUT);

	/* try auth again */
	__pm_stay_awake(charger->auth_retry_ws);
	queue_delayed_work(charger->wqueue,
		&charger->auth_retry_work, msecs_to_jiffies(1000));

skip_work:
	mutex_unlock(&charger->auth_lock);
end_work:
	sb_vote_set(charger->phm_vote, VOTER_WPC_AUTH, false, WPC_PHM_OFF);
	__pm_relax(charger->check_auth_ws);
}

static void cps4019_auth_retry_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, auth_retry_work.work);
	int auth_state = SB_WRL_AUTH_NONE;

	if (!cps4019_is_on_pad(charger))
		goto end_work;

	if (cps4019_is_phm(charger))
		goto end_work;

	mutex_lock(&charger->auth_lock);
	auth_state = cps4019_check_to_start_auth(&charger->now_tx);
	if (auth_state < 0)
		goto skip_work;

	if ((++charger->auth_check_cnt) > AUTH_RETRY_CNT) {
		auth_log("auth disable!!\n");
		charger->now_tx.info.auth = false;
		sb_wrl_inc_error(SB_WRL_ERROR_TYPE_AUTH);
		goto skip_work;
	}

	auth_log("retry (%d)!\n", charger->auth_check_cnt);

	cps4019_delay_auth_done_work(charger, AUTH_DONE_CHK_DELAY);
	cps4019_set_auth_event(&charger->now_tx, SB_WRL_AUTH_RETRY);

	mutex_unlock(&charger->auth_lock);
	__pm_relax(charger->auth_retry_ws);
	return;

skip_work:
	mutex_unlock(&charger->auth_lock);
end_work:
	charger->auth_check_cnt = 0;
	__pm_relax(charger->auth_retry_ws);
}

static void cps4019_check_auth_state(struct cps4019_charger_data *charger)
{
	int auth_state = SB_WRL_AUTH_NONE;
	struct sb_wrl_tx *tx;

	if (!cps4019_is_on_pad(charger))
		return;

	mutex_lock(&charger->auth_lock);
	tx = &charger->now_tx;
	auth_log("check tx info for auth(0x%llx).\n", tx->info_data);
	auth_state = cps4019_check_to_start_auth(tx);
	if (auth_state < 0)
		goto end_auth;

	cps4019_set_auth_event(tx, auth_state);
	switch (auth_state) {
	case SB_WRL_AUTH_PASS:
		auth_log("skip auth!\n");
		/* run wpc set work */
		sb_vote_set(charger->phm_vote, VOTER_WPC_RX_PWR_CHK, true, WPC_PHM_OFF);
		__pm_stay_awake(charger->wpc_set_ws);
		queue_delayed_work(charger->wqueue,
			&charger->wpc_set_work, msecs_to_jiffies(1000));
		break;

	case SB_WRL_AUTH_START:
	case SB_WRL_AUTH_RETRY:
	default:
		auth_log("%s!\n", get_auth_status_str(auth_state));
		cps4019_delay_auth_done_work(charger, AUTH_DONE_CHK_DELAY);
		break;
	}

end_auth:
	mutex_unlock(&charger->auth_lock);
}

static void cps4019_wpc_id_check_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, wpc_id_check_work.work);

	pr_info("%s: wc_w_state = %d, tx_id = 0x%x\n",
		__func__, charger->wc_w_state, charger->now_tx.info.id);

	if (!cps4019_is_on_pad(charger)) {
		goto end_work;
	} else if (charger->now_tx.info.recv_id) {
		cps4019_check_auth_state(charger);
		goto end_work;
	}

	if (charger->pdata->support_legacy_pad == 0) {
		if (!cps4019_send_watchdog_err_packet(charger))
			cps4019_send_eop(charger, POWER_SUPPLY_EXT_HEALTH_UNDERVOLTAGE);

		cps4019_set_online(charger, SB_CBL_ERR_WRL);
	} else {
		cps4019_set_online(charger, SB_CBL_INCOMP_WRL);
	}

	sb_wrl_inc_error(SB_WRL_ERROR_TYPE_INCOMPATIBLE);

end_work:
	charger->tx_id_check_cnt = 0;
	sb_vote_set(charger->phm_vote, VOTER_WPC_ID_CHK, false, WPC_PHM_OFF);

	__pm_relax(charger->wpc_id_check_ws);
}

static int cps4019_wpc_id_delayed_work(struct cps4019_charger_data *charger, int sec)
{
	int delay = sec * 1000;
	int lock_timeout = sec + (sec / 2);

	pr_info("%s: delay = %d, lock_timeout = %d\n", __func__, delay, lock_timeout);

	cancel_delayed_work(&charger->wpc_id_check_work);

	if (!charger->wpc_id_check_ws->active)
		__pm_wakeup_event(charger->wpc_id_check_ws, jiffies_to_msecs(lock_timeout * HZ));

	queue_delayed_work(charger->wqueue,
		&charger->wpc_id_check_work, msecs_to_jiffies(delay));
	return 0;
}

static void cps4019_wpc_set_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, wpc_set_work.work);
	int ret = 0;

	if (!cps4019_is_on_pad(charger)) {
		auth_log("skip this work because of no vbus\n");
		goto end_work;
	}

	if (charger->now_tx.info.rx_pwr_state)
		goto end_work;
	if ((++charger->rx_pwr_check_cnt) > SEND_RX_PWR_CNT)
		goto end_work;

	ret = cps4019_send_packet(charger, CPS4019_HEADER_AFC_CONF, SB_WRL_RX_DATA_CMD_AFC_SET, 0x05);
	auth_log("send WPC-SET packet(%d), ret(%d)\n", charger->rx_pwr_check_cnt, ret);

	queue_delayed_work(charger->wqueue,
		&charger->wpc_set_work, msecs_to_jiffies(1000));
	return;

end_work:
	auth_log("finish work, wc_w_state(%d), rw_pwr_state(%d), rx_pwr_check_cnt(%d)\n",
		charger->wc_w_state,
		charger->now_tx.info.rx_pwr_state, charger->rx_pwr_check_cnt);

	charger->rx_pwr_check_cnt = 0;
	sb_vote_set(charger->phm_vote, VOTER_WPC_RX_PWR_CHK, false, WPC_PHM_OFF);
	__pm_relax(charger->wpc_set_ws);
}

static void cps4019_wpc_id_request_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, wpc_id_request_work.work);
	int work_delay = 1000;

	if (!cps4019_is_on_pad(charger)) {
		pr_info("%s: skip this work because of no vbus\n", __func__);
		goto end_work;
	}

	charger->tx_id_check_cnt++;
	if ((charger->now_tx.info.recv_id) &&
		(charger->tx_id_check_cnt > 1)) {
		work_delay = 500;
		pr_info("%s: send RX-ID(%x) to TX (%d)\n",
			__func__, charger->pdata->rx_id, charger->tx_id_check_cnt);
		if (cps4019_send_packet(charger, CPS4019_HEADER_AFC_CONF,
				SB_WRL_RX_DATA_CMD_RX_ID, (u8)charger->pdata->rx_id) < 0)
			goto end_work;

		if (charger->tx_id_check_cnt >= SEND_RX_ID_CNT)
			goto end_work;
	} else {
		pr_info("%s: request TX-ID to TX (%d)\n",
			__func__, charger->tx_id_check_cnt);
		if (cps4019_send_packet(charger, CPS4019_HEADER_AFC_CONF,
			SB_WRL_RX_DATA_CMD_REQ_TX_ID, 0x0) < 0)
			goto end_work;

		if (charger->tx_id_check_cnt >= SEND_REQ_TX_ID_CNT) {
			cps4019_wpc_id_delayed_work(charger, WPC_AUTH_DELAY_TIME);
			__pm_relax(charger->wpc_id_request_ws);
			return;
		}
	}

	queue_delayed_work(charger->wqueue,
		&charger->wpc_id_request_work, msecs_to_jiffies(work_delay));
	return;

end_work:
	cps4019_check_auth_state(charger);

	charger->tx_id_check_cnt = 0;
	sb_vote_set(charger->phm_vote, VOTER_WPC_ID_CHK, false, WPC_PHM_OFF);
	__pm_relax(charger->wpc_id_request_ws);
}

#define CPS4019_FW_MIN_VRECT	5000
#define CPS4019_FW_MIN_CAPACITY	50
static bool cps4019_check_fw_err_ver(int fw_ver)
{
	return (fw_ver == 0x00060000) || (fw_ver == 0x00070000) || (fw_ver == 0x00080000);
}

#define CPS4019_FW_NONE		0
#define CPS4019_FW_RETRY	1
#define CSP4019_FW_START	2
static int cps4019_check_app2_state(int otp_ver, int bin_ver, int capacity, int vrect)
{
	if (otp_ver == bin_ver)
		return CPS4019_FW_NONE;

	if (capacity <= CPS4019_FW_MIN_CAPACITY)
		return CPS4019_FW_NONE;

	if (vrect < CPS4019_FW_MIN_VRECT)
		return CPS4019_FW_NONE;

	return CSP4019_FW_START;
}

static int cps4019_check_fw_state(struct cps4019_charger_data *charger)
{
	union power_supply_propval value = {0, };
	int otp_ver = 0, bin_ver = 0;
	int vrect = 0, app2_state = 0;

	if (!cps4019_is_on_pad(charger))
		return CPS4019_FW_NONE;

	/* read fw version */
	otp_ver = cps4019_get_firmware_version(charger);
	sb_wrl_fw_get_version(charger->fw, &bin_ver);
	/* read status */
	app2_state = cps4019_is_app2_request(charger);
	/* read vrect */
	vrect = cps4019_get_adc(charger->client, CPS4019_ADC_VRECT);
	/* get capacity */
	psy_do_property(charger->pdata->battery_name, get,
		POWER_SUPPLY_PROP_CAPACITY, value);
	pr_info("%s: status(%d, %d), version(0x%04x, 0x%04x), vrect(%d), capacity(%d)\n",
		__func__,
		charger->now_tx.info.fw_state, app2_state,
		otp_ver, bin_ver,
		vrect, value.intval);

	if (charger->now_tx.info.fw_state)
		return CPS4019_FW_NONE;

	/* check app1 & app2 error state */
	if (!cps4019_check_fw_err_ver(otp_ver) && !app2_state)
		return cps4019_check_app2_state(otp_ver, bin_ver, value.intval, vrect);

	if (vrect < CPS4019_FW_MIN_VRECT)
		return CPS4019_FW_RETRY;

	return CSP4019_FW_START;
}

static void cps4019_check_fw_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, check_fw_work.work);
	int state = CPS4019_FW_NONE;

	state = cps4019_check_fw_state(charger);
	pr_info("%s: fw state = %d\n", __func__, state);
	switch (state) {
	case CPS4019_FW_RETRY:
		queue_delayed_work(charger->wqueue,
			&charger->check_fw_work, msecs_to_jiffies(1000));
		return;

	case CSP4019_FW_START:
		cps4019_fw_update(charger, SB_WRL_RX_BUILT_IN_MODE);
		break;

	case CPS4019_FW_NONE:
	default:
		charger->now_tx.info.fw_state = true;
		sb_vote_set(charger->vm_vote, VOTER_WPC_FW, false, SB_WRL_VM_INIT);
		sb_vote_set(charger->phm_vote, VOTER_WPC_FW, false, WPC_PHM_OFF);
		sb_vote_set(charger->vout_off_vote, VOTER_WPC_FW, false, SB_WRL_VM_INIT);
		sb_vote_set(charger->det_vote, VOTER_WPC_FW, false, CPS4019_GPIO_GP1);
		__pm_stay_awake(charger->wpc_id_request_ws);
		queue_delayed_work(charger->wqueue, &charger->wpc_id_request_work, 0);
		break;
	}

	__pm_relax(charger->check_fw_ws);
}

static void cps4019_wpc_det_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, wpc_det_work.work);
	bool wc_w_state = false;
	u8 pad_mode = 0;
	u8 vrect = 0;

	wc_w_state = cps4019_is_on_pad(charger);
	pr_info("%s: w(%d to %d) force(%d)\n",
		__func__, charger->wc_w_state, wc_w_state, charger->force_wpc_det_chk);
	if (!(charger->force_wpc_det_chk) && (charger->wc_w_state == wc_w_state)) {
		__pm_relax(charger->det_ws);
		return;
	}
	charger->force_wpc_det_chk = false;
	charger->wc_w_state = wc_w_state;
	sb_vote_set(charger->phm_vote, VOTER_WPC_ID_CHK, wc_w_state, WPC_PHM_OFF);
	sb_vote_set(charger->phm_vote, VOTER_WPC_DET, !wc_w_state, WPC_PHM_OFF);
	sb_vote_set(charger->phm_vote, VOTER_WPC_TX_PHM, false, WPC_PHM_OFF);
	sb_vote_set(charger->phm_vote, VOTER_WPC_ERROR, charger->ssp_missing, WPC_PHM_OFF);
	sb_vote_set(charger->det_vote, VOTER_WPC_DET, wc_w_state, CPS4019_GPIO_GP1);
	sb_vote_set(charger->det_vote, VOTER_WPC_CONTROL, false, CPS4019_GPIO_GP1);
	sb_vote_set(charger->ichg_vote, VOTER_WPC_CONTROL, wc_w_state, (348 / SB_WRL_ICHG_STEP));

	if (wc_w_state) {
		/* enable pdetb & gp1 irq */
		cps4019_set_gpio_irq(charger, CPS4019_GPIO_GP1, CPS_IRQ_EN);
		cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, CPS_IRQ_EN);

		/* enable Mode Change INT */
		cps4019_set_irq_en_l(charger->client, CPS4019_OP_MODE_MASK, true);
		cps4019_set_irq_en_h(charger->client, CPS4019_AC_MISSING_DET_MASK, true);

		/* read pad mode */
		cps4019_reg_read(charger->client, CPS4019_SYS_OP_MODE_REG, &pad_mode);

		if (!is_wireless_type(charger->cable_type)) {
			cps4019_send_packet(charger, CPS4019_HEADER_AFC_CONF, SB_WRL_RX_DATA_CMD_REQ_TX_PWR_BUDG, 0x00);
			cps4019_set_online(charger, SB_CBL_WRL);

			sb_vote_set(charger->vout_off_vote, VOTER_WPC_FW, true, SB_WRL_VM_INIT);
			sb_vote_set(charger->phm_vote, VOTER_WPC_FW, true, WPC_PHM_OFF);
			sb_vote_set(charger->vm_vote, VOTER_WPC_FW, true, SB_WRL_VM_INIT);
		}

		/* set default charge mode */
		cps4019_set_charge_mode(charger, SB_WRL_CHG_MODE_CC);
		/* refresh vm */
		sb_vote_refresh(charger->dc_hdr_vote);
		sb_vote_refresh(charger->dc_high_vout_vote);
		sb_vote_refresh(charger->low_vout_vote);
		sb_vote_refresh(charger->bt_vbat_hdr_vote);
		sb_vote_refresh(charger->bt_hdr_vote);
		sb_vote_refresh(charger->vout_vote);
		sb_vote_refresh(charger->fx_hdr_vote);

		sb_vote_refresh(charger->vm_vote);

		// fcc refresh
		sb_votef_refresh(VN_FCC);
		pr_info("%s: wpc activated(vrect = 0x%x, pad_mode = 0x%x)\n",
			__func__, vrect, pad_mode);

		/* set watchdog */
		cps4019_set_watchdog_timer(charger, true);

		__pm_stay_awake(charger->check_fw_ws);
		queue_delayed_work(charger->wqueue,
			&charger->check_fw_work, msecs_to_jiffies(1000));

		if (charger->cs100_status) {
			sb_vote_set(charger->phm_vote, VOTER_WPC_CS100, true, WPC_PHM_OFF);
			__pm_stay_awake(charger->cs100_ws);
			queue_delayed_work(charger->wqueue,
				&charger->cs100_work, msecs_to_jiffies(VOUT_STABLILZATION_TIME));
		}
	} else {
		union power_supply_propval value;
		int i;

		/* disable pdetb & gp1 irq */
		cps4019_set_gpio_irq(charger, CPS4019_GPIO_GP1, CPS_IRQ_DISNOSYNC);
		cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, CPS_IRQ_DISNOSYNC);

		/* Send last tx_id to battery to count tx_id */
		value.intval = charger->now_tx.info.id;
		psy_do_property(charger->pdata->wireless_name,
			set, POWER_SUPPLY_PROP_AUTHENTIC, value);

		sb_wrl_tx_init(&charger->now_tx);
		charger->cable_type = SB_CBL_NONE;
		charger->charge_mode = SB_WRL_CHG_MODE_CC;

		charger->d2d_vout_strength = 100;
		charger->darkzone_reset = false;
		charger->phm_chk_cnt = 0;
		charger->d2d_checking_align = false;
		charger->d2d_align_check_start.tv_sec = 0;
		charger->tx_id_check_cnt = 0;
		charger->rx_pwr_check_cnt = 0;
		charger->auth_check_cnt = 0;
		charger->cs100_status = false;

		charger->ssp_missing = false;
		charger->ssp_missing_count = 0;
		cps4019_init_wa_flag(charger);

		cps4019_set_online(charger, SB_CBL_NONE);

		sb_notify_call_bit(SB_NOTIFY_EVENT_MISC, 0, BATT_MISC_EVENT_WIRELESS_AUTH_MASK);

		pr_info("%s: wpc deactivated, set V_INT as PD\n", __func__);

		/* relax wake lock */
		__pm_relax(charger->check_fw_ws);
		__pm_relax(charger->wpc_data_check_ws);
		__pm_relax(charger->wpc_id_check_ws);
		__pm_relax(charger->wpc_id_request_ws);
		__pm_relax(charger->power_hold_chk_ws);
		__pm_relax(charger->retry_phm_free_ws);
		__pm_relax(charger->darkzone_ws);
		__pm_relax(charger->align_check_ws);
		__pm_relax(charger->cs100_ws);
		__pm_relax(charger->check_auth_ws);
		__pm_relax(charger->wpc_set_ws);
		__pm_relax(charger->auth_retry_ws);

		/* cancel work */
		cancel_delayed_work(&charger->check_fw_work);
		cancel_delayed_work(&charger->wpc_isr_work);
		cancel_delayed_work(&charger->wpc_id_check_work);
		cancel_delayed_work(&charger->wpc_id_request_work);
		cancel_delayed_work(&charger->power_hold_chk_work);
		cancel_delayed_work(&charger->retry_phm_free_work);
		cancel_delayed_work(&charger->darkzone_work);
		cancel_delayed_work(&charger->align_check_work);
		cancel_delayed_work(&charger->cs100_work);
		cancel_delayed_work(&charger->auth_done_work);
		cancel_delayed_work(&charger->wpc_set_work);
		cancel_delayed_work(&charger->auth_retry_work);

		/* init vote */
		for (i = 0; i < VOTER_MAX; i++) {
			if ((i == VOTER_STARTUP) || (i == VOTER_WPC_VM_PHM))
				continue;

			sb_vote_set(charger->vm_vote, i, false, SB_WRL_VM_INIT);

			sb_vote_set(charger->dc_hdr_vote, i, false, 0);
			sb_vote_set(charger->ichg_vote, i, false, 0);
			sb_vote_set(charger->dc_high_vout_vote, i, false, 0);
			sb_vote_set(charger->low_vout_vote, i, false, 0);
			sb_vote_set(charger->bt_vbat_hdr_vote, i, false, 0);
			sb_vote_set(charger->bt_hdr_vote, i, false, 0);
			sb_vote_set(charger->vout_vote, i, false, 0);
			sb_vote_set(charger->fx_hdr_vote, i, false, 0);
		}

		sb_vote_set(charger->fcc_vote, VOTER_WPC_MODE, false, 0);
		sb_vote_set_f(VN_IN2BAT, VOTER_WPC_MODE, false, 0);
		sb_vote_set(charger->phm_vote, VOTER_WPC_FW, false, WPC_PHM_OFF);
		sb_vote_set(charger->phm_vote, VOTER_WPC_CS100, false, WPC_PHM_OFF);
		sb_vote_set(charger->phm_vote, VOTER_WPC_ALIGN_CHK, false, WPC_PHM_OFF);
		sb_vote_set(charger->phm_vote, VOTER_WPC_AUTH, false, WPC_PHM_OFF);
		sb_vote_set(charger->phm_vote, VOTER_WPC_RX_PWR_CHK, false, WPC_PHM_OFF);
		sb_vote_set(charger->phm_vote, VOTER_WPC_ERROR, false, WPC_PHM_OFF);
		sb_vote_set(charger->vout_off_vote, VOTER_WPC_FW, false, SB_WRL_VM_INIT);
		sb_vote_set(charger->vout_off_vote, VOTER_WPC_ALIGN_CHK, false, SB_WRL_VM_INIT);

		/* init pqueue */
		sb_pq_clear(charger->pq);
	}

	__pm_relax(charger->det_ws);
}

static void cps4019_recv_tx_chg_stop(struct cps4019_charger_data *charger, u8 data)
{
	switch (data) {
	case SB_WRL_TX_CHG_STOP_TX_OTP:
	case SB_WRL_TX_CHG_STOP_TX_OCP:
	case SB_WRL_TX_CHG_STOP_DARKZONE:
	case SB_WRL_TX_CHG_STOP_FOD:
	case SB_WRL_TX_CHG_STOP_AUTH_FAIL:
	case SB_WRL_TX_CHG_STOP_DISCON_FOD:
	case SB_WRL_TX_CHG_STOP_OC_FOD:
	case SB_WRL_TX_CHG_STOP_WATCHDOG:
	default:
		break;
	}

	pr_info("%s: received chg_stop(%s)\n",
		__func__, get_tx_chg_stop_str(data));
}

static void cps4019_recv_tx_id(struct cps4019_charger_data *charger, u8 data)
{
	struct sb_wrl_tx *tx =
		sb_wrl_tx_find(charger->pdata->tx_table, charger->pdata->tx_table_size, data);

	if (!tx) {
		if (is_incomp_wrl_type(charger->cable_type))
			pr_info("%s: skip work to keep incompatible pad type.\n", __func__);
		else
			pr_info("%s: incompatible TX pad\n", __func__);
		return;
	}

	if (!is_wireless_type(charger->cable_type) ||
		(charger->now_tx.info.id == 0)) {
		/* set now charger type */
		sb_wrl_tx_copy(&charger->now_tx, tx);

		tx = &charger->now_tx;
		/* check pad type */
		switch (tx->info.pad_type) {
		case SB_WRL_PAD_TYPE_ILLEGAL:
			tx->info.recv_id = true;
		case SB_WRL_PAD_TYPE_MULTI:
		case SB_WRL_PAD_TYPE_FAST_MULTI:
		case SB_WRL_PAD_TYPE_BPACK:
			pr_info("%s: wait acok falling.", __func__);
			break;
		default:
			__pm_stay_awake(charger->det_ws);
			queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
			pr_info("%s: use tx_id_irq to fast detect the wireless charging.\n", __func__);
			break;
		}
		return;
	}

	if (cps4019_is_phm(charger)) {
		pr_info("%s: abnormal case - PHM was recovered by SSP Missing, try to start WPC auth again and set PHM.\n",
			__func__);
		charger->ssp_missing = is_multi_pad_type(charger->now_tx.info.pad_type);

		sb_vote_set(charger->phm_vote, VOTER_WPC_TX_PHM, false, WPC_PHM_OFF);
		sb_vote_set(charger->phm_vote, VOTER_WPC_ERROR, true, WPC_PHM_OFF);
		return;
	}

	if (charger->now_tx.info.id != tx->info.id) {
		/* set new charger type */
		sb_wrl_tx_copy(&charger->now_tx, tx);

		tx = &charger->now_tx;
		/* for illegal pad */
		if (is_illegal_pad_type(tx->info.pad_type))
			tx->info.recv_id = true;

		charger->tx_id_check_cnt = 0;
		/* re-run det work */
		charger->force_wpc_det_chk = true;
		__pm_stay_awake(charger->det_ws);
		queue_delayed_work(charger->wqueue, &charger->wpc_det_work, msecs_to_jiffies(500));
		return;
	}

	if (charger->now_tx.info.recv_id) {
		pr_info("%s: Skip the behavior below if it is the same TX ID.\n", __func__);
		return;
	}

	tx = &charger->now_tx;
	switch (tx->info.pad_type) {
	case SB_WRL_PAD_TYPE_FACTORY:
		pr_info("%s: Wireless JIG pad(0x%02x)\n", __func__, data);
		sb_notify_call_bit(SB_NOTIFY_EVENT_MISC,
			BATT_MISC_EVENT_WIRELESS_JIG_PAD,
			BATT_MISC_EVENT_WIRELESS_JIG_PAD);
	case SB_WRL_PAD_TYPE_NORMAL:
		charger->cable_type = SB_CBL_INBOX_WRL;
		break;
	case SB_WRL_PAD_TYPE_D2D:
		charger->cable_type = SB_CBL_D2D_WRL;
		cps4019_wpc_align_check(charger, ALIGN_WORK_DELAY);
		break;
	case SB_WRL_PAD_TYPE_FAST_MULTI:
		charger->cable_type = SB_CBL_FM_WRL;
		break;
	case SB_WRL_PAD_TYPE_BPACK:
		charger->cable_type = SB_CBL_BPACK_WRL;
		break;
	case SB_WRL_PAD_TYPE_ILLEGAL:
	case SB_WRL_PAD_TYPE_MULTI:
	default:
		charger->cable_type = SB_CBL_WRL;
		break;
	}

	tx->info.recv_id = true;
	charger->tx_id_check_cnt = 1;
	cps4019_set_online(charger, charger->cable_type);
}

static void cps4019_recv_rx_pwr(struct cps4019_charger_data *charger, u8 data)
{
	switch (data) {
	case SB_WRL_RX_PWR_2W:
		auth_log("RX POWER 2W\n");
		charger->cable_type = SB_CBL_AUTH_WRL_2W;
		break;
	case SB_WRL_RX_PWR_5W:
		auth_log("RX POWER 5W\n");
		charger->cable_type = SB_CBL_AUTH_WRL_5W;
		break;
	default:
		charger->cable_type = SB_CBL_AUTH_WRL_2W;
		auth_log("NOT DEFINED RX POWER(%d)\n", data);
		break;
	}
	charger->now_tx.info.tx_pwr_budg = SB_WRL_TX_PWR_BUDG_NONE;
	charger->now_tx.info.rx_pwr_state = true;
	cps4019_set_online(charger, charger->cable_type);
}

static void cps4019_recv_tx_pwr_budg(struct cps4019_charger_data *charger, u8 data)
{
	switch (data) {
	case SB_WRL_TX_PWR_BUDG_2W:
		pr_info("%s: TX POWER BUDGET 2W\n", __func__);
		charger->now_tx.info.tx_pwr_budg = SB_WRL_TX_PWR_BUDG_2W;
		break;
	case SB_WRL_TX_PWR_BUDG_5W:
		pr_info("%s: TX POWER BUDGET 5W\n", __func__);
		charger->now_tx.info.tx_pwr_budg = SB_WRL_TX_PWR_BUDG_5W;
		break;
	default:
		pr_info("%s: NOT DEFINED TX POWER BUDGET(%d)\n", __func__, data);
		charger->now_tx.info.tx_pwr_budg = SB_WRL_TX_PWR_BUDG_NONE;
		break;
	}
}

static void cps4019_wpc_isr_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, wpc_isr_work.work);
	union sb_wrl_trx_data trx_data = { 0, };
	u8 cmd_data, val_data;

	if (!cps4019_is_on_pad(charger)) {
		pr_info("%s: wc_w_state is 0. exit wpc_isr_work.\n", __func__);
		goto end_wpc_isr_work;
	}

	if (sb_pq_pop(charger->pq, (sb_data *)&trx_data))
		goto end_wpc_isr_work;

	cmd_data = trx_data.cmd;
	val_data = trx_data.data[0];
	pr_info("%s: cmd(%x) val(%x)\n", __func__, cmd_data, val_data);

	switch (cmd_data) {
	case SB_WRL_TX_DATA_CMD_CHG_STOP:
		cps4019_recv_tx_chg_stop(charger, val_data);
		break;
	case SB_WRL_TX_DATA_CMD_TX_ID:
		cps4019_recv_tx_id(charger, val_data);
		break;
	case SB_WRL_TX_DATA_CMD_RX_PWR:
		cps4019_recv_rx_pwr(charger, val_data);
		break;
	case SB_WRL_TX_DATA_CMD_TX_PWR_BUDG:
		cps4019_recv_tx_pwr_budg(charger, val_data);
		break;
	case SB_WRL_TX_DATA_CMD_INCOMP_PAD:
		pr_info("%s: incompatible TX pad\n", __func__);
		break;
	default:
		pr_info("%s: invalid cmd type\n", __func__);
		break;
	}

	if (!sb_pq_empty(charger->pq)) {
		queue_delayed_work(charger->wqueue, &charger->wpc_isr_work, 0);
		return;
	}

end_wpc_isr_work:
	__pm_relax(charger->wpc_data_check_ws);
}

static irqreturn_t cps4019_wpc_det_irq_thread(int irq, void *irq_data)
{
	struct cps4019_charger_data *charger = irq_data;
	struct cps4019_gpio *gpio, *det_gpio;
	sb_data det_result = CPS4019_GPIO_GP1;
	bool pdet_state;

	gpio = cps4019_get_gpio_by_irq(charger, irq);
	pdet_state = cps4019_get_gpio_value(gpio);

	pr_info("%s: irq = %d, name = %s, value = %d\n",
		__func__, irq, gpio->name, pdet_state);

	sb_vote_get_result(charger->det_vote, &det_result);
	det_gpio = &charger->pdata->gpios[det_result];
	if (irq != det_gpio->irq) {
		pdet_state = cps4019_get_gpio_value(det_gpio);
		pr_info("%s: update det gpio state(irq = %d, name = %s, value = %d)\n",
			__func__, det_gpio->irq, det_gpio->name, pdet_state);
	}

	/* prevent recognition during ping duration time.*/
	__pm_stay_awake(charger->det_ws);
	if (!pdet_state) {
		charger->force_wpc_det_chk = true;
		queue_delayed_work(charger->wqueue,
			&charger->wpc_det_work, msecs_to_jiffies(40));
	} else {
		queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
	}

	return IRQ_HANDLED;
}

static void cps4019_trx_data_irq(struct cps4019_charger_data *charger)
{
	union sb_wrl_trx_data trx_data = { 0, };
	int ret = 0;

	ret = cps4019_reg_bulk_read(charger->client, CPS4019_TX_DATA_COM_REG,
		(u8 *)&trx_data, sizeof(union sb_wrl_trx_data));
	if (ret < 0) {
		pr_err("%s: failed to read reg(%d)\n", __func__, ret);
		return;
	}

	ret = sb_pq_push(charger->pq, 0, trx_data.value);
	if (ret < 0) {
		pr_err("%s: failed to push data(%d)\n", __func__, ret);
		return;
	}

	if (!delayed_work_pending(&charger->wpc_isr_work)) {
		/* run work */
		__pm_wakeup_event(charger->wpc_data_check_ws, jiffies_to_msecs(HZ * 5));
		queue_delayed_work(charger->wqueue, &charger->wpc_isr_work, 0);
	}
}

static void cps4019_adt_data_irq(struct cps4019_charger_data *charger, u8 adt_irq)
{
	int auth_state = SB_WRL_AUTH_NONE;

	mutex_lock(&charger->auth_lock);
	auth_state = cps4019_check_to_start_auth(&charger->now_tx);
	if (auth_state < 0) {
		auth_log("invalid adt state\n");
		goto end_irq;
	}

	if (adt_irq & CPS4019_ADT_SENT_MASK) {
		auth_log("ADT SENT IRQ\n");

		cps4019_set_auth_event(&charger->now_tx, SB_WRL_AUTH_SENT);
	}

	if (adt_irq & CPS4019_ADT_RECV_MASK) {
		auth_log("ADT RECV IRQ\n");

		cps4019_set_auth_event(&charger->now_tx, SB_WRL_AUTH_RECEIVED);
	}

	if (adt_irq & CPS4019_ADT_ERROR_MASK) {
		auth_log("ADT ERROR IRQ\n");

		if (charger->now_tx.info.auth_state == SB_WRL_AUTH_TIMEOUT)
			goto end_irq;

		cps4019_set_auth_event(&charger->now_tx, SB_WRL_AUTH_ERROR);
		cps4019_delay_auth_done_work(charger, 1000);
	}

end_irq:
	mutex_unlock(&charger->auth_lock);
}

static irqreturn_t cps4019_wpc_irq_thread(int irq, void *irq_data)
{
	struct cps4019_charger_data *charger = irq_data;
	int ret;
	u8 irq_src[2];
	u8 reg_data;

	__pm_stay_awake(charger->wpc_ws);
	mutex_lock(&charger->charger_lock);

	pr_info("%s: start\n", __func__);

	ret = cps4019_reg_read(charger->client, CPS4019_INT_L_REG, &irq_src[0]);
	ret = cps4019_reg_read(charger->client, CPS4019_INT_H_REG, &irq_src[1]);
	if (ret < 0) {
		pr_err("%s: Failed to read interrupt source: %d\n",
			__func__, ret);
		__pm_relax(charger->wpc_ws);
		mutex_unlock(&charger->charger_lock);
		return IRQ_NONE;
	}

	/* check again firmware version support vbat monitoring */
	ret = cps4019_get_firmware_version(charger);
	if (ret > 0) {
		pr_debug("%s: rx major firmware version 0x%x\n", __func__, ret);
		charger->pdata->otp_firmware_ver = ret;
	}

	pr_info("%s: interrupt source(0x%x)\n", __func__, irq_src[1] << 8 | irq_src[0]);

	if (irq_src[0] & CPS4019_OP_MODE_MASK) {
		ret = cps4019_reg_read(charger->client, CPS4019_SYS_OP_MODE_REG, &reg_data);
		pr_info("%s: MODE CHANGE IRQ - SYS OP MODE = 0x%x\n", __func__, reg_data);
	}

	if (irq_src[0] & CPS4019_STAT_VOUT_MASK) {
		ret = cps4019_reg_read(charger->client, CPS4019_STATUS_L_REG, &reg_data);
		pr_info("%s: VOUT IRQ - STAT VOUT = 0x%x\n", __func__, reg_data);
	}

	if (irq_src[0] & CPS4019_TX_DATA_RECV_MASK) {
		pr_info("%s: TX RECEIVED IRQ\n", __func__);
		cps4019_trx_data_irq(charger);
	}

	if (irq_src[0] & CPS4019_OVER_CURRENT_MASK)
		pr_info("%s: OVER CURRENT IRQ\n", __func__);

	if (irq_src[0] & CPS4019_OVER_TEMP_MASK)
		pr_info("%s: OVER TEMP IRQ\n", __func__);

	if (irq_src[1] & CPS4019_AC_MISSING_DET_MASK) {
		pr_info("%s: AC MISSING IRQ\n", __func__);
		sb_vote_set_f(VN_IN2BAT, VOTER_WPC_MODE, false, 0);

		/* run wpc det work */
		__pm_stay_awake(charger->det_ws);
		queue_delayed_work(charger->wqueue, &charger->wpc_det_work, msecs_to_jiffies(500));
	}

	if (irq_src[1] & CPS4019_ADT_DATA_MASK)
		cps4019_adt_data_irq(charger, irq_src[1]);

	if (irq_src[1] & CPS4019_APP2_REQUEST_MASK) {
		pr_info("%s: PROGRAM APP2 REQUEST IRQ, gp1 = %d, pdetb = %d, rx_mode = %d, app2 = %d\n",
			__func__,
			cps4019_get_gpio_value(&charger->pdata->gpios[CPS4019_GPIO_GP1]),
			cps4019_get_gpio_value(&charger->pdata->gpios[CPS4019_GPIO_PDETB]),
			cps4019_is_rx_mode(charger),
			cps4019_is_app2_request(charger));

		charger->now_tx.info.fw_state = false;
		sb_vote_set(charger->det_vote, VOTER_WPC_FW, true, CPS4019_GPIO_PDETB);
		__pm_stay_awake(charger->check_fw_ws);
		queue_delayed_work(charger->wqueue, &charger->check_fw_work, 0);
	}

	if ((irq_src[1] << 8 | irq_src[0]) != 0) {
		/* clear intterupt */
		cps4019_reg_write(charger->client, CPS4019_INT_CLEAR_L_REG, irq_src[0]);
		cps4019_reg_write(charger->client, CPS4019_INT_CLEAR_H_REG, irq_src[1]);
	}
	/* debug */
	__pm_relax(charger->wpc_ws);
	mutex_unlock(&charger->charger_lock);

	pr_info("%s: finish\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t cps4019_irq_handler(int irq, void *irq_data)
{
	/* prevent to run irq_thread during probe because of wireless scenario */
	return IS_ERR_OR_NULL(((struct cps4019_charger_data *)irq_data)->psy_chg) ?
		IRQ_HANDLED : IRQ_WAKE_THREAD;
}

enum {
	WPC_FW_VER = 0,
	WPC_IBUCK,
	WPC_WATCHDOG,
	WPC_ADDR,
	WPC_SIZE,
	WPC_DATA,
};


static struct device_attribute cps4019_attributes[] = {
	SEC_WPC_ATTR(fw),
	SEC_WPC_ATTR(ibuck),
	SEC_WPC_ATTR(watchdog_test),
	SEC_WPC_ATTR(addr),
	SEC_WPC_ATTR(size),
	SEC_WPC_ATTR(data),
};

ssize_t sec_wpc_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct cps4019_charger_data *charger = power_supply_get_drvdata(psy);
	const ptrdiff_t offset = attr - cps4019_attributes;
	int i = 0;

	switch (offset) {
	case WPC_FW_VER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "0x%x\n", charger->pdata->otp_firmware_ver);
		break;
	case WPC_IBUCK:
	{
		int ibuck = 0;

		ibuck = cps4019_get_adc(charger->client, CPS4019_ADC_IOUT);
		ibuck -= CPS4019_I_OUT_OFFSET;
		pr_info("%s: raw iout %dmA\n", __func__, ibuck);

		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", ibuck);
	}
		break;
	case WPC_WATCHDOG:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", charger->watchdog_test);
		break;
	case WPC_ADDR:
		i += scnprintf(buf + i, PAGE_SIZE - i, "0x%x\n", charger->addr);
		break;
	case WPC_SIZE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "0x%x\n", charger->size);
		break;
	case WPC_DATA:
		if (charger->size == 0)
			charger->size = 1;

		if (charger->size + charger->addr <= 0xFFFF) {
			u8 data;
			int j;

			for (j = 0; j < charger->size; j++) {
				if (cps4019_reg_read(charger->client, charger->addr + j, &data) < 0) {
					dev_info(charger->dev, "%s: read fail\n", __func__);
					i += scnprintf(buf + i, PAGE_SIZE - i,
						"addr: 0x%x read fail\n", charger->addr + j);
					continue;
				}
				i += scnprintf(buf + i, PAGE_SIZE - i,
					"addr: 0x%x, data: 0x%x\n", charger->addr + j, data);
			}
		}
		break;
	default:
		break;
	}

	return i;
}

ssize_t sec_wpc_store_attrs(
					struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct cps4019_charger_data *charger =
		power_supply_get_drvdata(psy);
	const ptrdiff_t offset = attr - cps4019_attributes;
	int x, ret = -EINVAL;

	switch (offset) {
	case WPC_WATCHDOG:
		sscanf(buf, "%d\n", &x);
		if (x == 1)
			charger->watchdog_test = true;
		else if (x == 0)
			charger->watchdog_test = false;
		else
			pr_debug("%s: non valid input\n", __func__);
		pr_info("%s: watchdog test is set to %d\n", __func__, charger->watchdog_test);
		ret = count;
		break;
	case WPC_ADDR:
		if (sscanf(buf, "0x%x\n", &x) == 1)
			charger->addr = x;

		ret = count;
		break;
	case WPC_SIZE:
		if (sscanf(buf, "%d\n", &x) == 1)
			charger->size = x;

		ret = count;
		break;
	case WPC_DATA:
		if (sscanf(buf, "0x%x", &x) == 1) {
			u8 data = x;

			if (cps4019_reg_write(charger->client, charger->addr, data) < 0)
				dev_info(charger->dev,
						"%s: addr: 0x%x write fail\n", __func__, charger->addr);
		}
		ret = count;
		break;
	default:
		break;
	}
	return ret;
}

static int sec_wpc_create_attrs(struct device *dev)
{
	unsigned long i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(cps4019_attributes); i++) {
		rc = device_create_file(dev, &cps4019_attributes[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	while (i--)
		device_remove_file(dev, &cps4019_attributes[i]);
create_attrs_succeed:
	return rc;
}

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
static int cps4019_muic_handle(struct cps4019_charger_data *charger, unsigned long action, muic_attached_dev_t attached_dev)
{
	/* If wired cable is connected, Turn off the RX IC */
	sb_vote_set(charger->en_vote, VOTER_S2MPW03_CONTROL, (action == MUIC_NOTIFY_CMD_ATTACH), false);
	return 0;
}

static int cps4019_wpc_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
	struct cps4019_charger_data *charger =
		container_of(nb, struct cps4019_charger_data, wpc_nb);

	pr_info("%s: action=%lu, attached_dev=%d\n", __func__, action, attached_dev);
	if (attached_dev != ATTACHED_DEV_WIRELESS_TA_MUIC)
		return cps4019_muic_handle(charger, action, attached_dev);

	switch (action) {
	case MUIC_NOTIFY_CMD_DETACH: //acok rising
		pr_info("%s: MUIC_NOTIFY_CMD_DETACH(DET = %s)\n",
			__func__, GET_BOOL_STR(cps4019_is_on_pad(charger)));
		cps4019_wpc_align_check(charger, ALIGN_WORK_CHK_PERIOD);
		break;
	case MUIC_NOTIFY_CMD_ATTACH: //acok falling
		pr_info("%s: MUIC_NOTIFY_CMD_ATTACH(DET = %s)\n",
			__func__, GET_BOOL_STR(cps4019_is_on_pad(charger)));
		__pm_stay_awake(charger->det_ws);
		queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);
		break;
	default:
		break;
	}

	return 0;
}
#endif /* CONFIG_MUIC_NOTIFIER */

static void cps4019_init_work(struct work_struct *work)
{
	struct cps4019_charger_data *charger =
		container_of(work, struct cps4019_charger_data, init_work.work);
	bool pdetb_state = false, acok_state = false;

	pdetb_state = cps4019_get_gpio_value(&charger->pdata->gpios[CPS4019_GPIO_PDETB]);
	acok_state = cps4019_check_chg_in_status(charger);
	pr_info("%s: pdetb_state = %s, acok_state = %s\n",
		__func__, GET_BOOL_STR(pdetb_state), GET_BOOL_STR(acok_state));
	if (cps4019_rx_ic_power_on_check(charger)) {
		/* clear irq */
		int ret = 0;
		u8 irq_src[2];

		ret = cps4019_reg_read(charger->client, CPS4019_INT_L_REG, &irq_src[0]);
		ret = cps4019_reg_read(charger->client, CPS4019_INT_H_REG, &irq_src[1]);
		if (ret < 0) {
			pr_err("%s: Failed to read interrupt source: %d\n",
				__func__, ret);
		} else if ((irq_src[1] << 8 | irq_src[0]) != 0) {
			pr_info("%s: clear irq 0x%x\n", __func__, (irq_src[1] << 8 | irq_src[0]));

			if (irq_src[0] & CPS4019_TX_DATA_RECV_MASK)
				cps4019_trx_data_irq(charger);

			/* clear intterupt */
			cps4019_reg_write(charger->client, CPS4019_INT_CLEAR_L_REG, irq_src[0]);
			cps4019_reg_write(charger->client, CPS4019_INT_CLEAR_H_REG, irq_src[1]);
		}
	} else if (pdetb_state) {
		/* abnormal case - reset IC */
		pr_info("%s: anormal case or wired charger type - reset IC\n", __func__);
		cps4019_rx_ic_reset(charger, true);
	} else {
		pr_info("%s: does not need to check irq and case\n", __func__);
	}

	/* run wpc det work for detacting wireless charger */
	__pm_stay_awake(charger->det_ws);
	queue_delayed_work(charger->wqueue, &charger->wpc_det_work, 0);

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
	muic_notifier_register(&charger->wpc_nb, cps4019_wpc_handle_notification, MUIC_NOTIFY_DEV_CHARGER);
#endif /* CONFIG_MUIC_NOTIFIER */
}

static int cps4019_parse_gpio(struct device_node *np,
	const char *name, cps4019_charger_platform_data_t *pdata, int gpio_type)
{
	struct cps4019_gpio *gpio;
	unsigned int gpio_flags = 0;
	int ret = 0;

	ret = of_get_named_gpio_flags(np, name, 0, &gpio_flags);
	if (ret < 0) {
		pr_info("%s: failed to get %s, ret = %d\n", __func__, name, ret);
		return ret;
	}

	gpio = &pdata->gpios[gpio_type];
	gpio->name = name;
	gpio->gpio = ret;
	gpio->flags = gpio_flags;
	gpio->irq = gpio_to_irq(ret);

	return 0;
}

static int cps4019_parse_tx_table(struct device_node *np,
	cps4019_charger_platform_data_t *pdata)
{
	struct sb_wrl_tx *tx;
	int len, cnt, i, j, str_size;
	char log_buf[256] = { 0, };

	str_size = sizeof(struct sb_wrl_tx);
	len = of_property_count_u32_elems(np, "tx_table");
	if (len < str_size)
		return -EINVAL;

	cnt = (len * sizeof(u32)) / str_size;
	pdata->tx_table = kcalloc(cnt, str_size, GFP_KERNEL);
	if (!pdata->tx_table)
		return -ENOMEM;

	i = of_property_read_u32_array(np, "tx_table",
		(u32 *)pdata->tx_table, len);
	if (i) {
		kfree(pdata->tx_table);
		return -EINVAL;
	}

	pdata->tx_table_size = cnt;
	for (i = 0; i < cnt; i++) {
		tx = &pdata->tx_table[i];

		if (!gpio_is_valid(pdata->wpc_en))
			tx->info.phm = false;

		snprintf(log_buf, 256,
			"tx(0x%llx), id(0x%02x), pad_type(%d), phm(%d), auth(%d), phm_time(%d) darkzone(%d) ",
			tx->info_data, tx->info.id, tx->info.pad_type,
			tx->info.phm, tx->info.auth, tx->info.phm_time, tx->info.darkzone);

#if IS_ENABLED(CONFIG_SEC_FACTORY)
		/* prevent auto firmware update in factory binary. */
		tx->info.fw_state = true;
#endif

		for (j = 0; j < SB_WRL_CHG_MODE_MAX; j++) {
			str_size = strlen(log_buf);
			snprintf(log_buf + str_size, 256 - str_size,
				"%s(0x%llx) ", get_chg_mode_str(j), tx->chg_mode[j]);
		}

		pr_info("%s: %s\n", __func__, log_buf);
	}

	return 0;
}

static int cps4019_parse_dt(struct device *dev, cps4019_charger_platform_data_t *pdata)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags irq_gpio_flags;
	int ret;
	/*changes can be added later, when needed*/

	ret = of_property_read_string(np,
		"cps4019,charger_name", (char const **)&pdata->charger_name);
	if (ret) {
		pr_info("%s: Charger name is Empty\n", __func__);
		pdata->charger_name = "sec-charger";
	}

	ret = of_property_read_string(np,
		"cps4019,fuelgauge_name", (char const **)&pdata->fuelgauge_name);
	if (ret) {
		pr_info("%s: Fuelgauge name is Empty\n", __func__);
		pdata->fuelgauge_name = "sec-fuelgauge";
	}

	ret = of_property_read_string(np,
		"cps4019,battery_name", (char const **)&pdata->battery_name);
	if (ret) {
		pr_info("%s: battery_name is Empty\n", __func__);
		pdata->battery_name = "battery";
	}

	ret = of_property_read_string(np,
		"cps4019,wireless_name", (char const **)&pdata->wireless_name);
	if (ret) {
		pr_info("%s: wireless_name is Empty\n", __func__);
		pdata->wireless_name = "wireless";
	}

	ret = of_property_read_string(np, "cps4019,wireless_charger_name",
		(char const **)&pdata->wireless_charger_name);
	if (ret) {
		pr_info("%s: wireless_charger_name is Empty\n", __func__);
		pdata->wireless_charger_name = "wpc";
	}

	ret = cps4019_parse_gpio(np, "cps4019,wpc_gp1", pdata, CPS4019_GPIO_GP1);
	ret = cps4019_parse_gpio(np, "cps4019,wpc_det", pdata, CPS4019_GPIO_PDETB);
	ret = cps4019_parse_gpio(np, "cps4019,wpc_int", pdata, CPS4019_GPIO_INTB);

	/* wpc_en */
	ret = pdata->wpc_en = of_get_named_gpio_flags(np, "cps4019,wpc_en",
			0, &irq_gpio_flags);
	if (ret < 0) {
		dev_err(dev, "%s: can't wpc_en\r\n", __func__);
	} else {
		if (gpio_is_valid(pdata->wpc_en)) {
			ret = gpio_request(pdata->wpc_en, "wpc_en");
			if (ret)
				dev_err(dev, "%s: can't get wpc_en\r\n", __func__);
			else
				gpio_direction_output(pdata->wpc_en, 0);
		}
	}

	ret = pdata->wpc_ping_en = of_get_named_gpio_flags(np, "cps4019,wpc_ping_en",
			0, &irq_gpio_flags);
	if (ret < 0) {
		dev_err(dev, "%s: can't wpc_ping_en\r\n", __func__);
	} else {
		if (gpio_is_valid(pdata->wpc_ping_en)) {
			ret = gpio_request(pdata->wpc_ping_en, "wpc_ping_en");
			if (ret)
				dev_err(dev, "%s: can't get wpc_ping_en\r\n", __func__);
			else
				gpio_direction_output(pdata->wpc_ping_en, 1);
		}
	}

	ret = of_property_read_u32(np, "cps4019,cc_cv_threshold",
		&pdata->cc_cv_threshold);
	if (ret < 0) {
		pr_err("%s: error reading cc_cv_threshold(%d)\n", __func__, ret);
		pdata->cc_cv_threshold = 30;
	}

	ret = of_property_read_u32(np, "cps4019,half_bridge_threshold",
		&pdata->half_bridge_threshold);
	if (ret < 0) {
		pr_err("%s: error reading half_bridge_threshold(%d)\n", __func__, ret);
		pdata->half_bridge_threshold = 20;
	}

	ret = of_property_read_u32(np, "cps4019,half_bridge_vout",
		&pdata->half_bridge_vout);
	if (ret < 0) {
		pr_err("%s: error reading half_bridge_vout(%d)\n", __func__, ret);
		pdata->half_bridge_vout = 4500;
	}

	ret = of_property_read_u32(np, "cps4019,darkzone_expired_time",
		&pdata->darkzone_expired_time);
	if (ret < 0) {
		pr_err("%s: error reading darkzone_expired_time(%d)\n", __func__, ret);
		pdata->darkzone_expired_time = 120;
	}

	ret = of_property_read_u32(np, "cps4019,low_current_expired_time",
		&pdata->low_current_expired_time);
	if (ret < 0) {
		pr_err("%s: error reading low_current_expired_time(%d)\n", __func__, ret);
		pdata->low_current_expired_time = 300;
	}

	ret = of_property_read_u32(np, "cps4019,ab_acok_count",
		&pdata->ab_acok_count);
	if (ret < 0) {
		pr_err("%s: error reading ab_acok_count(%d)\n", __func__, ret);
		pdata->ab_acok_count = 60;
	}

	ret = of_property_read_u32(np, "cps4019,ab_acok_time",
		&pdata->ab_acok_time);
	if (ret < 0) {
		pr_err("%s: error reading ab_acok_time(%d)\n", __func__, ret);
		pdata->ab_acok_time = 10;
	}

	ret = of_property_read_u32(np, "cps4019,rx_id",
		&pdata->rx_id);
	if (ret < 0) {
		pr_err("%s: error reading rx_id %d\n", __func__, ret);
		pdata->rx_id = 0;
	}

	ret = of_property_read_u32(np, "cps4019,tx-off-high-temp",
		&pdata->tx_off_high_temp);
	if (ret) {
		pr_info("%s: TX-OFF-TEMP is Empty\n", __func__);
		pdata->tx_off_high_temp = INT_MAX;
	}

	ret = of_property_read_u32(np, "cps4019,ping_duration",
		&pdata->ping_duration);
	if (ret) {
		pr_info("%s: ping_duration Empty\n", __func__);
		pdata->ping_duration = 350;
	}

	ret = of_property_read_u32(np, "cps4019,support_legacy_pad",
		&pdata->support_legacy_pad);
	if (ret) {
		pr_info("%s: support_legacy_pad Empty\n", __func__);
		pdata->support_legacy_pad = 0;
	}

	ret = cps4019_parse_tx_table(np, pdata);
	return 0;
}

static const struct power_supply_desc wpc_power_supply_desc = {
	.name = "wpc",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = wpc_props,
	.num_properties = ARRAY_SIZE(wpc_props),
	.get_property = cps4019_chg_get_property,
	.set_property = cps4019_chg_set_property,
};

static int cps4019_request_irq(struct cps4019_charger_data *charger,
	int gpio_type, irq_handler_t thread_fn, unsigned int irqflags)
{
	struct cps4019_gpio *gpio = &charger->pdata->gpios[gpio_type];
	int ret = 0;

	if (gpio->irq <= 0)
		return 0;

	ret = request_threaded_irq(gpio->irq, cps4019_irq_handler, thread_fn, irqflags, gpio->name, charger);
	if (ret) {
		pr_err("%s: failed to request irq (%s, ret = %d)\n",
			__func__, gpio->name, ret);
		return ret;
	}

	return 0;
}

static void print_message(unsigned char *buf, unsigned int size, const char *fname)
{
	int start_idx = 0;

	do {
		char temp_buf[1024] = {0, };
		int size_temp = 0, str_len = 1024;
		int old_idx = start_idx;

		size_temp = ((start_idx + 0x7F) > size) ? size : (start_idx + 0x7F);
		for (; start_idx < size_temp; start_idx++) {
			snprintf(temp_buf + strlen(temp_buf), str_len, "0x%02x ", buf[start_idx]);
			str_len = 1024 - strlen(temp_buf);
		}
		auth_log("%s: (%04d ~ %04d) %s\n", fname, old_idx, start_idx - 1, temp_buf);
	} while (start_idx < size);
}

static int cps4019_send_misc_data(struct i2c_client *i2c, unsigned char *buf, unsigned int size)
{
	int ret = 0;

	print_message(buf, size, __func__);

	/* write type */
	ret = cps4019_reg_write(i2c, CPS4019_ADT_TYPE_REG,
			CPS4019_ADT_TYPE_GENERAL << CPS4019_ADT_TYPE_SHIFT);
	if (ret < 0)
		return ret;

	/* write data size */
	ret = cps4019_reg_write(i2c, CPS4019_ADT_DATA_SIZE_REG, size);
	if (ret < 0)
		return ret;

	/* write data */
	ret = cps4019_reg_bulk_write(i2c, CPS4019_ADT_DATA_L_REG, buf, size);
	if (ret < 0)
		return ret;

	/* send adt data */
	ret = cps4019_reg_update(i2c, CPS4019_COMMAND_H_REG,
		CPS4019_CMD_SEND_ADT_MASK, CPS4019_CMD_SEND_ADT_MASK);
	return ret;
}

static int cps4019_recv_misc_data(struct i2c_client *i2c, unsigned char *buf, unsigned int *size)
{
	unsigned char tmp_size = 0;
	int ret = 0;

	/* read data size */
	ret = cps4019_reg_read(i2c, CPS4019_ADT_DATA_SIZE_REG, &tmp_size);
	if (ret < 0)
		return ret;

	if (tmp_size > CPS4019_ADT_MAX_SIZE)
		return -EIO;

	/* read data */
	ret = cps4019_reg_bulk_read(i2c, CPS4019_ADT_DATA_L_REG, buf, tmp_size);
	if (ret < 0)
		return ret;

	*size = tmp_size;

	print_message(buf, *size, __func__);
	return 0;
}

static void cps4019_check_auth_result(struct cps4019_charger_data *charger, unsigned char buf)
{
	switch (buf) {
	case 0x1:
		auth_log("pass\n");
		cps4019_set_auth_event(&charger->now_tx, SB_WRL_AUTH_PASS);

		/* run wpc set work */
		sb_vote_set(charger->phm_vote, VOTER_WPC_RX_PWR_CHK, true, WPC_PHM_OFF);
		__pm_stay_awake(charger->wpc_set_ws);
		queue_delayed_work(charger->wqueue,
			&charger->wpc_set_work, msecs_to_jiffies(1000));
		break;
	case 0x2:
		auth_log("fail\n");
		cps4019_set_auth_event(&charger->now_tx, SB_WRL_AUTH_FAIL);

		charger->now_tx.info.tx_pwr_budg = SB_WRL_TX_PWR_BUDG_NONE;
		cps4019_set_online(charger, SB_CBL_ERR_AUTH_WRL);
		break;
	default:
		auth_log("invalid arg(%d)\n", buf);
		break;
	}

	/* cancel auth done work */
	cancel_delayed_work(&charger->auth_done_work);
	sb_vote_set(charger->phm_vote, VOTER_WPC_AUTH, false, WPC_PHM_OFF);
	__pm_relax(charger->check_auth_ws);
}

static int cps4019_copy_from_user(void *to, unsigned long from, unsigned long size)
{
#ifdef CONFIG_COMPAT
	return copy_from_user(to, compat_ptr(from), size);
#else
	return copy_from_user(to, (void __user *)from, size);
#endif
}

static int cps4019_copy_to_user(unsigned long to, const void *from, unsigned long size)
{
#ifdef CONFIG_COMPAT
	return copy_to_user(compat_ptr(to), from, size);
#else
	return copy_to_user((void __user *)to, from, size);
#endif
}

static int cps4019_check_misc_state(struct i2c_client *i2c, unsigned int cmd)
{
	struct cps4019_charger_data *charger = i2c_get_clientdata(i2c);

	if (cmd != SB_WRL_MISC_CMD) {
		auth_log("invalid cmd(0x%x) != 0x%lx\n", cmd, SB_WRL_MISC_CMD);
		return -EINVAL;
	}

	if (!(charger->now_tx.info.auth) ||
		(charger->now_tx.info.auth_state == SB_WRL_AUTH_NONE) ||
		(charger->now_tx.info.auth_state == SB_WRL_AUTH_ERROR)) {
		auth_log("ignore tx(0x%llx)\n", charger->now_tx.info_data);
		return -EACCES;
	}

	return 0;
}

static int sb_misc_ioctl(void *dev, unsigned int cmd, sb_data arg)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct cps4019_charger_data *charger = i2c_get_clientdata(i2c);
	struct sb_wrl_misc_data mdata;
	unsigned char *buf;
	unsigned int size = 0;
	int ret = 0;

	mutex_lock(&charger->auth_lock);
	ret = cps4019_check_misc_state(i2c, cmd);
	if (ret)
		goto end_misc;

	ret = cps4019_copy_from_user(&mdata, arg, sizeof(struct sb_wrl_misc_data));
	if (ret) {
		auth_log("failed to get user data(%d)\n", ret);
		ret = -EIO;
		goto end_misc;
	}

	buf = kzalloc(CPS4019_ADT_MAX_SIZE, GFP_KERNEL);
	if (!buf) {
		auth_log("failed to alloc buf\n");
		ret = -ENOMEM;
		goto end_misc;
	}

	switch (mdata.dir) {
	case SB_WRL_MISC_DIR_OUT:
		if ((mdata.size > CPS4019_ADT_MAX_SIZE) || (mdata.size <= 0)) {
			auth_log("invalid user data size(%d)\n", mdata.size);
			ret = -ENOMEM;
			goto fail_ioctl;
		}

		ret = cps4019_copy_from_user(buf, mdata.data, mdata.size);
		if (ret) {
			auth_log("failed to copy from user data(%d)\n", ret);
			ret = -EIO;
			goto fail_ioctl;
		}

		sb_notify_call_bit(SB_NOTIFY_EVENT_MISC, 0, BATT_MISC_EVENT_WIRELESS_AUTH_MASK);

		if (mdata.size > 1) {
			ret = cps4019_send_misc_data(i2c, buf, mdata.size);
			if (ret < 0) {
				auth_log("failed to send data(%d)\n", ret);
				goto fail_ioctl;
			}
		} else { /* only mdata.size == 1 */
			cps4019_check_auth_result(charger, buf[0]);
		}
		size = mdata.size;
		break;
	case SB_WRL_MISC_DIR_IN:
		ret = cps4019_recv_misc_data(i2c, buf, &size);
		if (ret < 0) {
			auth_log("failed to recv data(%d)\n", ret);
			goto fail_ioctl;
		}

		if (size <= 2) {
			auth_log("data from mcu has invalid size\n");
			cps4019_check_auth_result(charger, 0x2); /* fail */
			ret = -EINVAL;
			goto fail_ioctl;
		}

		ret = cps4019_copy_to_user(mdata.data, buf, size);
		if (ret) {
			auth_log("failed to copy to user data(%d)\n", ret);
			ret = -EIO;
			goto fail_ioctl;
		}
		break;
	default:
		auth_log("invalid dir(%d)\n", mdata.dir);
		ret = -EINVAL;
		goto fail_ioctl;
	}

fail_ioctl:
	kfree(buf);
end_misc:
	mutex_unlock(&charger->auth_lock);
	return (ret < 0) ? ret : size;
}

static bool pq_cmp_func(sb_data data1, sb_data data2)
{
	union sb_wrl_trx_data p1 = { .value = data1, },
		p2 = { .value = data2, };

	return (p1.cmd >= p2.cmd);
}

static int sb_noti_handler(struct notifier_block *nb, unsigned long action, void *data)
{
	return 0;
}

static const struct sb_wrl_op cps4019_wrl_op = {
	.set_vm_phm = cps4019_set_vm_phm,
	.get_vm_phm = cps4019_get_vm_phm,
};

static int cps4019_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cps4019_charger_data *charger;
	cps4019_charger_platform_data_t *pdata = client->dev.platform_data;
	struct power_supply_config wpc_cfg = {};
	int ret = 0;

	dev_info(&client->dev,
		"%s: cps4019 Charger Driver Loading\n", __func__);

	pdata = devm_kzalloc(&client->dev, sizeof(cps4019_charger_platform_data_t), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = cps4019_parse_dt(&client->dev, pdata);
	if (ret < 0)
		return ret;

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (charger == NULL) {
		ret = -ENOMEM;
		goto err_wpc_nomem;
	}
	charger->dev = &client->dev;

	ret = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK);
	if (!ret) {
		ret = i2c_get_functionality(client->adapter);
		dev_err(charger->dev, "I2C functionality is not supported.\n");
		ret = -ENOSYS;
		goto err_i2cfunc_not_support;
	}
	charger->client = client;
	charger->pdata = pdata;

	i2c_set_clientdata(client, charger);
	charger->fw = sb_wrl_fw_register(client, cps4019_fw_result_cb);
	charger->pq = sb_pq_create(0, 20, pq_cmp_func);
	charger->misc = sb_misc_init(charger->dev, "batt_misc", sb_misc_ioctl);

	mutex_init(&charger->charger_lock);
	mutex_init(&charger->auth_lock);
	mutex_init(&charger->gpio_lock);

	charger->wc_w_state = false;

	charger->en_vote = sb_vote_init(VN_WPC_EN, SB_VOTE_TYPE_MIN, VOTER_MAX,
		true, sb_voter_name, cps4019_set_en_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->en_vote)) {
		dev_err(&client->dev, "%s: failed to init en_vote\n", __func__);
		ret = -ENOMEM;
		goto err_en_vote;
	}

	charger->det_vote = sb_vote_init("WPC-DET", SB_VOTE_TYPE_MAX, VOTER_MAX,
		CPS4019_GPIO_GP1, sb_voter_name, cps4019_set_det_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->det_vote)) {
		dev_err(&client->dev, "%s: failed to init det_vote\n", __func__);
		ret = -ENOMEM;
		goto err_det_vote;
	}

	charger->fcc_vote = sb_vote_init("CHG-FCC", SB_VOTE_TYPE_MAX, VOTER_MAX,
		130, sb_voter_name, cps4019_set_fcc_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->fcc_vote)) {
		dev_err(&client->dev, "%s: failed to init fcc_vote\n", __func__);
		ret = -ENOMEM;
		goto err_fcc_vote;
	}
	sb_vote_set_pri(charger->fcc_vote, VOTER_WPC_MODE, VOTE_PRI_3);

	charger->phm_vote = sb_vote_init(VN_WPC_PHM, SB_VOTE_TYPE_MAX, VOTER_MAX,
		false, sb_voter_name, cps4019_set_phm_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->phm_vote)) {
		dev_err(&client->dev, "%s: failed to init phm_vote\n", __func__);
		ret = -ENOMEM;
		goto err_phm_vote;
	}
	sb_vote_set_pri(charger->phm_vote, VOTER_WPC_ERROR, VOTE_PRI_5);
	sb_vote_set_pri(charger->phm_vote, VOTER_WPC_CS100, VOTE_PRI_7);
	sb_vote_set_pri(charger->phm_vote, VOTER_WPC_RX_PWR_CHK, VOTE_PRI_8);
	sb_vote_set_pri(charger->phm_vote, VOTER_WPC_AUTH, VOTE_PRI_8);
	sb_vote_set_pri(charger->phm_vote, VOTER_WPC_ALIGN_CHK, VOTE_PRI_8);
	sb_vote_set_pri(charger->phm_vote, VOTER_WPC_ID_CHK, VOTE_PRI_8);
	sb_vote_set_pri(charger->phm_vote, VOTER_WPC_FW, VOTE_PRI_9);
	sb_vote_set_pri(charger->phm_vote, VOTER_WPC_DET, VOTE_PRI_10);

	charger->vm_vote = sb_vote_init(VN_WPC_VM, SB_VOTE_TYPE_MIN, VOTER_MAX,
		SB_WRL_VM_INIT, sb_voter_name, cps4019_set_vm_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->vm_vote)) {
		dev_err(&client->dev, "%s: failed to init vm_vote\n", __func__);
		ret = -ENOMEM;
		goto err_vm_vote;
	}
	sb_vote_set_pri(charger->vm_vote, VOTER_WPC_HALF_BRIDGE, VOTE_PRI_5);
	sb_vote_set_pri(charger->vm_vote, VOTER_WPC_CONTROL, VOTE_PRI_7);
	sb_vote_set_pri(charger->vm_vote, VOTER_WPC_VM_PHM, VOTE_PRI_8);
	sb_vote_set_pri(charger->vm_vote, VOTER_WPC_FW, VOTE_PRI_10);

	charger->dc_hdr_vote = sb_vote_init(VN_WPC_DC_HDR, SB_VOTE_TYPE_MAX, VOTER_MAX,
		(100 / SB_WRL_HDR_STEP), sb_voter_name, cps4019_set_dc_hdr_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->dc_hdr_vote)) {
		dev_err(&client->dev, "%s: failed to init dc_hdr_vote\n", __func__);
		ret = -ENOMEM;
		goto err_dc_hdr_vote;
	}
	sb_vote_set_pri(charger->dc_hdr_vote, VOTER_WPC_HALF_BRIDGE, VOTE_PRI_5);

	charger->ichg_vote = sb_vote_init(VN_WPC_ICHG, SB_VOTE_TYPE_MIN, VOTER_MAX,
		(348 / SB_WRL_ICHG_STEP), sb_voter_name, cps4019_set_ichg_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->ichg_vote)) {
		dev_err(&client->dev, "%s: failed to init ichg_vote\n", __func__);
		ret = -ENOMEM;
		goto err_ichg_vote;
	}

	charger->dc_high_vout_vote = sb_vote_init(VN_WPC_DC_HIGH_VOUT, SB_VOTE_TYPE_MAX, VOTER_MAX,
		(4800 / SB_WRL_VOUT_STEP), sb_voter_name, cps4019_set_dc_high_vout_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->dc_high_vout_vote)) {
		dev_err(&client->dev, "%s: failed to init dc_high_vout_vote\n", __func__);
		ret = -ENOMEM;
		goto err_dc_high_vout_vote;
	}

	charger->low_vout_vote = sb_vote_init(VN_WPC_LOW_VOUT, SB_VOTE_TYPE_MIN, VOTER_MAX,
		(3500 / SB_WRL_VOUT_STEP), sb_voter_name, cps4019_set_low_vout_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->low_vout_vote)) {
		dev_err(&client->dev, "%s: failed to init low_vout_vote\n", __func__);
		ret = -ENOMEM;
		goto err_low_vout_vote;
	}
	sb_vote_set_pri(charger->low_vout_vote, VOTER_WPC_HALF_BRIDGE, VOTE_PRI_5);

	charger->bt_vbat_hdr_vote = sb_vote_init(VN_WPC_BT_VBAT_HDR, SB_VOTE_TYPE_MAX, VOTER_MAX,
		(300 / SB_WRL_HDR_STEP), sb_voter_name, cps4019_set_bt_vbat_hdr_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->bt_vbat_hdr_vote)) {
		dev_err(&client->dev, "%s: failed to init bt_vbat_hdr_vote\n", __func__);
		ret = -ENOMEM;
		goto err_bt_vbat_hdr_vote;
	}
	sb_vote_set_pri(charger->bt_vbat_hdr_vote, VOTER_WPC_HALF_BRIDGE, VOTE_PRI_5);

	charger->bt_hdr_vote = sb_vote_init(VN_WPC_BT_HDR, SB_VOTE_TYPE_MAX, VOTER_MAX,
		(300 / SB_WRL_HDR_STEP), sb_voter_name, cps4019_set_bt_hdr_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->bt_hdr_vote)) {
		dev_err(&client->dev, "%s: failed to init bt_hdr_vote\n", __func__);
		ret = -ENOMEM;
		goto err_bt_hdr_vote;
	}
	sb_vote_set_pri(charger->bt_hdr_vote, VOTER_WPC_HALF_BRIDGE, VOTE_PRI_5);

	charger->vout_vote = sb_vote_init(VN_WPC_VOUT, SB_VOTE_TYPE_MIN, VOTER_MAX,
		(5000 / SB_WRL_VOUT_STEP), sb_voter_name, cps4019_set_vout_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->vout_vote)) {
		dev_err(&client->dev, "%s: failed to init vout_vote\n", __func__);
		ret = -ENOMEM;
		goto err_vout_vote;
	}
	sb_vote_set_pri(charger->vout_vote, VOTER_WPC_HALF_BRIDGE, VOTE_PRI_5);

	charger->fx_hdr_vote = sb_vote_init(VN_WPC_FX_HDR, SB_VOTE_TYPE_MAX, VOTER_MAX,
		(300 / SB_WRL_HDR_STEP), sb_voter_name, cps4019_set_fx_hdr_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->fx_hdr_vote)) {
		dev_err(&client->dev, "%s: failed to init fx_hdr_vote\n", __func__);
		ret = -ENOMEM;
		goto err_fx_hdr_vote;
	}
	sb_vote_set_pri(charger->fx_hdr_vote, VOTER_WPC_HALF_BRIDGE, VOTE_PRI_5);

	charger->ldo_vote = sb_vote_init(VN_WPC_LDO, SB_VOTE_TYPE_MAX, VOTER_MAX,
		WPC_LDO_ON, sb_voter_name, cps4019_set_ldo_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->ldo_vote)) {
		dev_err(&client->dev, "%s: failed to init ldo_vote\n", __func__);
		ret = -ENOMEM;
		goto err_ldo_vote;
	}

	charger->vout_off_vote = sb_vote_init(VN_WPC_VOUT_OFF, SB_VOTE_TYPE_MAX, VOTER_MAX,
		false, sb_voter_name, cps4019_set_vout_off_vote, NULL, charger);
	if (IS_ERR_OR_NULL(charger->vout_off_vote)) {
		dev_err(&client->dev, "%s: failed to init vout_off_vote\n", __func__);
		ret = -ENOMEM;
		goto err_vout_off_vote;
	}
	sb_vote_set_pri(charger->vout_off_vote, VOTER_WPC_ALIGN_CHK, VOTE_PRI_8);
	sb_vote_set_pri(charger->vout_off_vote, VOTER_WPC_FW, VOTE_PRI_10);

	/* set startup */
	cps4019_startup_init(charger);

	sb_wrl_tx_init(&charger->now_tx);
	charger->cable_type = SB_CBL_NONE;
	charger->charge_mode = SB_WRL_CHG_MODE_CC;

	charger->watchdog_test = false;
	charger->store_mode = 0;

	charger->force_wpc_det_chk = cps4019_is_on_pad(charger);
	charger->temperature = 250;
	charger->d2d_vout_strength = 100;
	charger->now_tx.info.auth_state = SB_WRL_AUTH_NONE;

	charger->wqueue = create_singlethread_workqueue("cps4019_workqueue");
	if (!charger->wqueue) {
		dev_err(&client->dev, "%s: failed to create wqueue\n", __func__);
		ret = -ENOMEM;
		goto err_wqueue;
	}

	charger->wpc_ws = wakeup_source_register(charger->dev, "cps4019-wpc");
	charger->det_ws = wakeup_source_register(charger->dev, "cps4019-det");
	charger->wpc_data_check_ws = wakeup_source_register(charger->dev, "cps4019-wpc_data_check");
	charger->wpc_id_check_ws = wakeup_source_register(charger->dev, "cps4019-wpc_id_check");
	charger->power_hold_chk_ws = wakeup_source_register(charger->dev, "cps4019-power_hold_chk");
	charger->wpc_id_request_ws = wakeup_source_register(charger->dev, "cps4019-wpc_id_request");
	charger->align_check_ws = wakeup_source_register(charger->dev, "cps4019-align_check");
	charger->phm_free_ws = wakeup_source_register(charger->dev, "cps4019-phm_free");
	charger->retry_phm_free_ws = wakeup_source_register(charger->dev, "cps4019-retry_phm_free");
	charger->cs100_ws = wakeup_source_register(charger->dev, "cps4019-cs100");
	charger->darkzone_ws = wakeup_source_register(charger->dev, "cps4019-darkzone");
	charger->ab_acok_ws = wakeup_source_register(charger->dev, "cps4019-ab_acok");
	charger->check_fw_ws = wakeup_source_register(charger->dev, "cps4019-check_fw");
	charger->check_auth_ws = wakeup_source_register(charger->dev, "cps4019-check_auth");
	charger->auth_retry_ws = wakeup_source_register(charger->dev, "cps4019-auth-retry");
	charger->wpc_set_ws = wakeup_source_register(charger->dev, "cps4019-wpc_set");

	INIT_DELAYED_WORK(&charger->align_check_work, cps4019_wpc_align_check_work);
	INIT_DELAYED_WORK(&charger->phm_free_work, cps4019_phm_free_work);
	INIT_DELAYED_WORK(&charger->retry_phm_free_work, cps4019_retry_phm_free_work);
	INIT_DELAYED_WORK(&charger->cs100_work, cps4019_cs100_work);
	INIT_DELAYED_WORK(&charger->darkzone_work, cps4019_darkzone_work);
	INIT_DELAYED_WORK(&charger->wpc_id_check_work, cps4019_wpc_id_check_work);
	INIT_DELAYED_WORK(&charger->power_hold_chk_work, cps4019_phm_check_work);
	INIT_DELAYED_WORK(&charger->wpc_id_request_work, cps4019_wpc_id_request_work);
	INIT_DELAYED_WORK(&charger->check_fw_work, cps4019_check_fw_work);
	INIT_DELAYED_WORK(&charger->auth_done_work, cps4019_auth_done_work);
	INIT_DELAYED_WORK(&charger->auth_retry_work, cps4019_auth_retry_work);
	INIT_DELAYED_WORK(&charger->wpc_set_work, cps4019_wpc_set_work);

	INIT_DELAYED_WORK(&charger->wpc_det_work, cps4019_wpc_det_work);
	INIT_DELAYED_WORK(&charger->wpc_isr_work, cps4019_wpc_isr_work);
	INIT_DELAYED_WORK(&charger->init_work, cps4019_init_work);

	ret = cps4019_request_irq(charger,
		CPS4019_GPIO_GP1, cps4019_wpc_det_irq_thread,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT);
	if (ret)
		goto err_supply_unreg;
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_GP1, CPS_IRQ_DISWAKE | CPS_IRQ_DIS);

	ret = cps4019_request_irq(charger,
		CPS4019_GPIO_PDETB, cps4019_wpc_det_irq_thread,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT);
	if (ret)
		goto err_supply_unreg;
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, CPS_IRQ_DISWAKE | CPS_IRQ_DIS);

	ret = cps4019_request_irq(charger,
		CPS4019_GPIO_INTB, cps4019_wpc_irq_thread,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT);
	if (ret)
		goto err_supply_unreg;
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_INTB, CPS_IRQ_DISWAKE);

	wpc_cfg.drv_data = charger;
	if (!wpc_cfg.drv_data) {
		dev_err(&client->dev,
			"%s: Failed to Register psy_chg\n", __func__);
		goto err_supply_unreg;
	}

	charger->psy_chg = power_supply_register(&client->dev,
		&wpc_power_supply_desc, &wpc_cfg);
	if (IS_ERR(charger->psy_chg))
		goto err_supply_unreg;


	ret = sec_wpc_create_attrs(&charger->psy_chg->dev);
	if (ret) {
		dev_err(&client->dev,
			"%s: Failed to Register psy_chg\n", __func__);
		goto err_pdata_free;
	}

	sb_notify_register(&charger->sb_nb,
		sb_noti_handler, "wpc", SB_DEV_WIRELESS_CHARGER);

	/* moved init process to init work because of msleep in cps4019_set_cmd_reg */
	queue_delayed_work(charger->wqueue, &charger->init_work, 0);

	sb_wrl_register(&cps4019_wrl_op, charger);

	dev_info(&client->dev,
		"%s: cps4019 Charger Driver Loaded\n", __func__);

	device_init_wakeup(charger->dev, 1);
	return 0;
err_pdata_free:
	power_supply_unregister(charger->psy_chg);
err_supply_unreg:
	wakeup_source_unregister(charger->det_ws);
	wakeup_source_unregister(charger->wpc_ws);
	wakeup_source_unregister(charger->wpc_data_check_ws);
	wakeup_source_unregister(charger->wpc_id_check_ws);
	wakeup_source_unregister(charger->power_hold_chk_ws);
	wakeup_source_unregister(charger->wpc_id_request_ws);
	wakeup_source_unregister(charger->align_check_ws);
	wakeup_source_unregister(charger->phm_free_ws);
	wakeup_source_unregister(charger->retry_phm_free_ws);
	wakeup_source_unregister(charger->cs100_ws);
	wakeup_source_unregister(charger->darkzone_ws);
	wakeup_source_unregister(charger->ab_acok_ws);
	wakeup_source_unregister(charger->check_fw_ws);
	wakeup_source_unregister(charger->check_auth_ws);
	wakeup_source_unregister(charger->auth_retry_ws);
	wakeup_source_unregister(charger->wpc_set_ws);
err_wqueue:
	sb_vote_destroy(charger->vout_off_vote);
err_vout_off_vote:
	sb_vote_destroy(charger->ldo_vote);
err_ldo_vote:
	sb_vote_destroy(charger->fx_hdr_vote);
err_fx_hdr_vote:
	sb_vote_destroy(charger->vout_vote);
err_vout_vote:
	sb_vote_destroy(charger->bt_hdr_vote);
err_bt_hdr_vote:
	sb_vote_destroy(charger->bt_vbat_hdr_vote);
err_bt_vbat_hdr_vote:
	sb_vote_destroy(charger->low_vout_vote);
err_low_vout_vote:
	sb_vote_destroy(charger->dc_high_vout_vote);
err_dc_high_vout_vote:
	sb_vote_destroy(charger->ichg_vote);
err_ichg_vote:
	sb_vote_destroy(charger->dc_hdr_vote);
err_dc_hdr_vote:
	sb_vote_destroy(charger->vm_vote);
err_vm_vote:
	sb_vote_destroy(charger->phm_vote);
err_phm_vote:
	sb_vote_destroy(charger->fcc_vote);
err_fcc_vote:
	sb_vote_destroy(charger->det_vote);
err_det_vote:
	sb_vote_destroy(charger->en_vote);
err_en_vote:
	mutex_destroy(&charger->gpio_lock);
	mutex_destroy(&charger->auth_lock);
	mutex_destroy(&charger->charger_lock);
err_i2cfunc_not_support:
	kfree(charger);
err_wpc_nomem:
	devm_kfree(&client->dev, pdata);
	return ret;
}

#if IS_ENABLED(CONFIG_PM)
static int cps4019_charger_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct cps4019_charger_data *charger = i2c_get_clientdata(i2c);
	unsigned int wake_irq_flag = CPS_IRQ_DISNOSYNC;

	pr_info("%s\n", __func__);

	if (device_may_wakeup(charger->dev))
		wake_irq_flag |= CPS_IRQ_ENWAKE;

	if (charger->wc_w_state)
		cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, wake_irq_flag);
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_INTB, wake_irq_flag);
	return 0;
}

static int cps4019_charger_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct cps4019_charger_data *charger = i2c_get_clientdata(i2c);
	unsigned int wake_irq_flag = CPS_IRQ_EN;

	pr_info("%s: charge_mode = %d, charger->pdata->otp_firmware_ver = %x\n",
		__func__, charger->charge_mode, charger->pdata->otp_firmware_ver);

	if (device_may_wakeup(charger->dev))
		wake_irq_flag |= CPS_IRQ_DISWAKE;

	cps4019_set_gpio_irq(charger, CPS4019_GPIO_INTB, wake_irq_flag);
	if (charger->wc_w_state)
		cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, wake_irq_flag);
	return 0;
}
#else
#define cps4019_charger_suspend NULL
#define cps4019_charger_resume NULL
#endif

static void cps4019_charger_shutdown(struct i2c_client *client)
{
	struct cps4019_charger_data *charger = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);

	sb_misc_exit(charger->misc);
	sb_notify_unregister(&charger->sb_nb);

	if (!cps4019_is_on_pad(charger) ||
		!gpio_is_valid(charger->pdata->wpc_en))
		return;

	/* disable irqs */
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_GP1, CPS_IRQ_DISNOSYNC);
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_PDETB, CPS_IRQ_DISNOSYNC);
	cps4019_set_gpio_irq(charger, CPS4019_GPIO_INTB, CPS_IRQ_DISNOSYNC);

	sb_vote_set(charger->en_vote, VOTER_WPC_DET, true, false);
	msleep(PHM_DETACH_DELAY);
}

static const struct i2c_device_id cps4019_charger_id[] = {
	{"cps4019-charger", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cps4019_charger_id);

static const struct of_device_id cps4019_i2c_match_table[] = {
	{ .compatible = "cps4019,i2c"},
	{},
};

static const struct dev_pm_ops cps4019_charger_pm = {
	.suspend = cps4019_charger_suspend,
	.resume = cps4019_charger_resume,
};

static struct i2c_driver cps4019_charger_driver = {
	.driver = {
		.name	= "cps4019-charger",
		.owner	= THIS_MODULE,
#if IS_ENABLED(CONFIG_PM)
		.pm	= &cps4019_charger_pm,
#endif /* CONFIG_PM */
		.of_match_table = cps4019_i2c_match_table,
	},
	.shutdown	= cps4019_charger_shutdown,
	.probe	= cps4019_charger_probe,
	//.remove	= cps4019_charger_remove,
	.id_table	= cps4019_charger_id,
};

static int __init cps4019_charger_init(void)
{
	pr_debug("%s\n", __func__);
	return i2c_add_driver(&cps4019_charger_driver);
}

static void __exit cps4019_charger_exit(void)
{
	pr_debug("%s\n", __func__);
	i2c_del_driver(&cps4019_charger_driver);
}

module_init(cps4019_charger_init);
module_exit(cps4019_charger_exit);

MODULE_DESCRIPTION("Samsung CPS4019 Charger Driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
