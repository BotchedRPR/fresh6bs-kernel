/*
 * muic_manager.c
 *
 * Copyright (C) 2019 Samsung Electronics
 * Sejong Park <sejong123.park@samsung.com>
 * Taejung Kim <tj.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#define pr_fmt(fmt)	"[MUIC] " fmt

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#if IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
#include <linux/host_notify.h>
#endif
#include <linux/string.h>
#if IS_ENABLED(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include <linux/misc/samsung/muic/common/muic.h>
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
#include <linux/misc/samsung/ifconn/ifconn_notifier.h>
#endif
#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
#include <linux/misc/samsung/muic/common/muic_notifier.h>
#endif
#include <linux/misc/samsung/muic/common/muic_interface.h>

#if IS_ENABLED(CONFIG_CCIC_NOTIFIER)
#include <linux/usb/typec/common/pdic_notifier.h>
#endif

#if IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
#include <linux/usb/typec/manager/usb_typec_manager_notifier.h>
#endif

#define MUIC_CCIC_NOTI_ATTACH (1)
#define MUIC_CCIC_NOTI_DETACH (-1)
#define MUIC_CCIC_NOTI_UNDEFINED (0)

static int __ccic_info;
static struct ccic_rid_desc_t ccic_rid_tbl[] = {
	[CCIC_RID_UNDEFINED] = {"UNDEFINED", ATTACHED_DEV_NONE_MUIC},
	[CCIC_RID_000K] = {"000K", ATTACHED_DEV_OTG_MUIC},
	[CCIC_RID_001K] = {"001K", ATTACHED_DEV_MHL_MUIC},
	[CCIC_RID_255K] = {"255K", ATTACHED_DEV_JIG_USB_OFF_MUIC},
	[CCIC_RID_301K] = {"301K", ATTACHED_DEV_JIG_USB_ON_MUIC},
	[CCIC_RID_523K] = {"523K", ATTACHED_DEV_JIG_UART_OFF_MUIC},
	[CCIC_RID_619K] = {"619K", ATTACHED_DEV_JIG_UART_ON_MUIC},
	[CCIC_RID_OPEN] = {"OPEN", ATTACHED_DEV_NONE_MUIC},
};

/*
 * __ccic_info :
 * b'0: 1 if an active ccic is present,
 *        0 when muic works without ccic chip or
 *              no ccic Noti. registration is needed
 *              even though a ccic chip is present.
 */
#if 0
static int set_ccic_info(char *str)
{
	get_option(&str, &__ccic_info);

	pr_info("%s: ccic_info: 0x%04x\n", __func__, __ccic_info);

	return __ccic_info;
}
__setup("ccic_info=", set_ccic_info);
#endif

int get_ccic_info(void)
{
	return __ccic_info;
}

static void _muic_manager_switch_uart_path(struct muic_interface_t *muic_if, int path)
{
	if (muic_if->pdata->gpio_uart_sel)
		muic_if->set_gpio_uart_sel(muic_if->muic_data, path);

	if (muic_if->set_switch_to_uart)
		muic_if->set_switch_to_uart(muic_if->muic_data);
	else
		pr_err("%s:function not set!\n", __func__);
}

static void _muic_manager_switch_usb_path(struct muic_interface_t *muic_if, int path)
{
	if (muic_if->pdata->gpio_usb_sel)
		muic_if->set_gpio_usb_sel(muic_if->muic_data, path);

	if (muic_if->set_switch_to_usb)
		muic_if->set_switch_to_usb(muic_if->muic_data);
	else
		pr_err("%s:function not set!\n", __func__);
}

static int muic_manager_switch_path(struct muic_interface_t *muic_if, int path)
{
	switch (path) {
	case MUIC_PATH_OPEN:
		if (muic_if->set_com_to_open)
			muic_if->set_com_to_open(muic_if->muic_data);
		break;

	case MUIC_PATH_USB_AP:
	case MUIC_PATH_USB_CP:
		_muic_manager_switch_usb_path(muic_if, muic_if->pdata->usb_path);
		break;
	case MUIC_PATH_UART_AP:
	case MUIC_PATH_UART_CP:
		_muic_manager_switch_uart_path(muic_if, muic_if->pdata->uart_path);
		break;

	default:
		pr_err("%s:A wrong com path!\n", __func__);
		return -1;
	}

	return 0;
}

static int muic_manager_get_vbus(struct muic_interface_t *muic_if)
{
	int ret = 0;

	if (muic_if->get_vbus)
		ret = muic_if->get_vbus(muic_if->muic_data);

	return ret;
}

static bool muic_manager_is_supported_dev(int attached_dev)
{
	switch (attached_dev) {
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_TA_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_VB_MUIC:
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
	case ATTACHED_DEV_OTG_MUIC:
	case ATTACHED_DEV_AFC_CHARGER_5V_MUIC:
	case ATTACHED_DEV_AFC_CHARGER_9V_MUIC:
	case ATTACHED_DEV_QC_CHARGER_5V_MUIC:
	case ATTACHED_DEV_QC_CHARGER_9V_MUIC:
		return true;
	default:
		break;
	}

	return false;
}

int muic_manager_is_ccic_supported_dev(muic_attached_dev_t new_dev)
{
	switch (new_dev) {
		/* Legacy TA/USB. Noti. will be sent when ATTACH is received from CCIC. */
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_TA_MUIC:
	case ATTACHED_DEV_TIMEOUT_OPEN_MUIC:
		return 1;
	default:
		break;
	}

	return 0;
}

void muic_manager_handle_ccic_detach_always(struct muic_interface_t *muic_if)
{
	pr_info("%s\n", __func__);

#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC) || IS_ENABLED(CONFIG_MUIC_HV)
	muic_if->is_afc_pdic_ready = false;
#endif
}

void muic_manager_handle_ccic_detach(struct muic_interface_t *muic_if)
{
	struct ccic_desc_t *ccic = muic_if->ccic;
	struct muic_platform_data *pdata = muic_if->pdata;

	pr_info("%s\n", __func__);

	if (ccic->ccic_evt_rprd) {
		/* FIXME : pvendor
		 * if (pvendor && pvendor->enable_chgdet)
		 * pvendor->enable_chgdet(muic_if->regmaccic, 1);
		 */
	}

	if (muic_if->set_cable_state)
		muic_if->set_cable_state(muic_if->muic_data, ATTACHED_DEV_NONE_MUIC);

	if (muic_manager_is_supported_dev(ccic->attached_dev)) {
		MUIC_SEND_NOTI_DETACH(ccic->attached_dev);
	} else if (muic_if->legacy_dev != ATTACHED_DEV_NONE_MUIC) {
		MUIC_SEND_NOTI_DETACH(muic_if->legacy_dev);
	}

	if (pdata->jig_uart_cb)
		pdata->jig_uart_cb(0);

	/* Reset status & flags */
	ccic->attached_dev = 0;
	ccic->ccic_evt_rid = 0;
	ccic->ccic_evt_rprd = 0;
	//ccic->ccic_evt_roleswap = 0;
	ccic->ccic_evt_dcdcnt = 0;
	ccic->ccic_evt_attached = MUIC_CCIC_NOTI_UNDEFINED;

	muic_if->legacy_dev = 0;
	muic_if->attached_dev = 0;
	muic_if->is_dcdtmr_intr = false;
}

#if !IS_ENABLED(CONFIG_SEC_FACTORY)
void muic_manager_handle_ccic_rid_open(struct muic_interface_t *muic_if)
{
	struct ccic_desc_t *ccic = muic_if->ccic;
	struct muic_platform_data *pdata = muic_if->pdata;

	pr_info("%s\n", __func__);

	if (muic_if->set_cable_state)
		muic_if->set_cable_state(muic_if->muic_data, ATTACHED_DEV_NONE_MUIC);

	muic_manager_switch_path(muic_if, MUIC_PATH_OPEN);
	if (muic_manager_is_supported_dev(ccic->attached_dev)) {
		MUIC_SEND_NOTI_DETACH(ccic->attached_dev);
	} else if (muic_if->legacy_dev != ATTACHED_DEV_NONE_MUIC) {
		MUIC_SEND_NOTI_DETACH(muic_if->legacy_dev);
	}

	if (pdata->jig_uart_cb)
		pdata->jig_uart_cb(0);

	muic_if->legacy_dev = 0;
	muic_if->attached_dev = 0;
	muic_if->is_dcdtmr_intr = false;
}
#endif

void muic_manager_set_legacy_dev(struct muic_interface_t *muic_if, int new_dev)
{
	pr_info("%s: %d->%d\n", __func__, muic_if->legacy_dev, new_dev);

	muic_if->legacy_dev = new_dev;
}

/* Get the charger type from muic interrupt or by reading the register directly */
int muic_manager_get_legacy_dev(struct muic_interface_t *muic_if)
{
	return muic_if->legacy_dev;
}

static void muic_manager_show_status(struct muic_interface_t *muic_if)
{
	struct ccic_desc_t *ccic = muic_if->ccic;

	pr_info("%s: attached_dev:%d rid:%d rprd:%d attached:%d legacy_dev:%d\n", __func__,
			ccic->attached_dev, ccic->ccic_evt_rid, ccic->ccic_evt_rprd,
			ccic->ccic_evt_attached, muic_if->legacy_dev);
}

int muic_manager_dcd_rescan(struct muic_interface_t *muic_if)
{
	struct ccic_desc_t *ccic = muic_if->ccic;
	int vbus = muic_manager_get_vbus(muic_if);

	pr_info("%s : ccic_evt_attached(%d), is_dcdtmr_intr(%d), ccic_evt_dcdcnt(%d)\n",
			__func__, ccic->ccic_evt_attached, muic_if->is_dcdtmr_intr, ccic->ccic_evt_dcdcnt);

	if (!(muic_if->opmode & S2M_OPMODE_CCIC)) {
		pr_info("%s : it's SMD board, skip rescan", __func__);
		goto SKIP_RESCAN;
	}

	/* W/A for Incomplete insertion case */
	if (muic_if->is_dcdtmr_intr && vbus && ccic->ccic_evt_dcdcnt < 1) {
		pr_info("%s: Incomplete insertion. Do chgdet again\n", __func__);
		ccic->ccic_evt_dcdcnt++;

		if (muic_if->set_dcd_rescan != NULL)
			muic_if->set_dcd_rescan(muic_if->muic_data);

		return 0;
	}

SKIP_RESCAN:
	return 1;
}

static int muic_manager_handle_legacy_dev(struct muic_interface_t *muic_if)
{
	struct ccic_desc_t *ccic = muic_if->ccic;
	int attached_dev = 0;

	pr_info("%s: vbvolt:%d legacy_dev:%d\n", __func__,
		muic_if->vps.t.vbvolt, muic_if->legacy_dev);

	/* 1. Run a charger detection algorithm manually if necessary. */
	msleep(200);

	/* 2. Get the result by polling or via an interrupt */
	attached_dev = muic_manager_get_legacy_dev(muic_if);
	pr_info("%s: detected legacy_dev=%d\n", __func__, attached_dev);

	/* 3. Noti. if supported. */
	if (!muic_manager_is_ccic_supported_dev(attached_dev)) {
		pr_info("%s: Unsupported legacy_dev=%d\n", __func__, attached_dev);
		return 0;
	}

	if (muic_manager_is_supported_dev(ccic->attached_dev)) {
		MUIC_SEND_NOTI_DETACH(ccic->attached_dev);
		ccic->attached_dev = 0;
	} else if (muic_if->legacy_dev != ATTACHED_DEV_NONE_MUIC) {
		MUIC_SEND_NOTI_DETACH(muic_if->legacy_dev);
		muic_if->legacy_dev = 0;
	}

	ccic->attached_dev = attached_dev;
	MUIC_SEND_NOTI_ATTACH(attached_dev);

	return 0;
}

void muic_manager_init_dev_desc(struct muic_interface_t *muic_if)
{
	struct ccic_desc_t *ccic = muic_if->ccic;

	pr_info("%s\n", __func__);
	ccic->attached_dev = 0;
	ccic->ccic_evt_rid = 0;
	ccic->ccic_evt_rprd = 0;
	ccic->ccic_evt_roleswap = 0;
	ccic->ccic_evt_dcdcnt = 0;
	ccic->ccic_evt_attached = MUIC_CCIC_NOTI_UNDEFINED;
}

static int muic_manager_conv_rid_to_dev(struct muic_interface_t *muic_if, int rid, int vbus)
{
	int attached_dev = 0;

	pr_info("%s rid=%d vbus=%d\n", __func__, rid, vbus);

	if (rid < 0 || rid > CCIC_RID_OPEN) {
		pr_err("%s:Out of RID range: %d\n", __func__, rid);
		return 0;
	}

	if ((rid == CCIC_RID_619K) && vbus)
		attached_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
	else
		attached_dev = muic_if->ccic->rid_desc[rid].attached_dev;

	return attached_dev;
}

static bool muic_manager_is_valid_rid_open(struct muic_interface_t *muic_if, int vbus)
{
	int i, retry = 5;

	if (vbus)
		return true;

	for (i = 0; i < retry; i++) {
		pr_info("%s: %dth ...\n", __func__, i);
		msleep(20);
		if (muic_manager_get_vbus(muic_if))
			return 1;
	}

	return 0;
}

static int muic_manager_handle_ccic_attach(struct muic_interface_t *muic_if, void *data)
{
	struct ccic_desc_t *ccic = muic_if->ccic;
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	struct ifconn_notifier_template *pnoti =
		(struct ifconn_notifier_template *)data;
#else
	CC_NOTI_ATTACH_TYPEDEF *pnoti = (CC_NOTI_ATTACH_TYPEDEF *) data;
#endif
	int vbus = muic_manager_get_vbus(muic_if);
	struct muic_platform_data *pdata = muic_if->pdata;

	pr_info("%s: src:%d dest:%d id:%d attach:%d cable_type:%d rprd:%d\n",
			__func__, pnoti->src, pnoti->dest, pnoti->id, pnoti->attach,
			pnoti->cable_type, pnoti->rprd);

#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	if (pnoti->event == IFCONN_NOTIFY_EVENT_ATTACH) {
		ccic->ccic_evt_attached = MUIC_CCIC_NOTI_ATTACH;
	} else if (pnoti->event == IFCONN_NOTIFY_EVENT_DETACH) {
		ccic->ccic_evt_attached = MUIC_CCIC_NOTI_DETACH;
	} else {
		ccic->ccic_evt_attached = MUIC_CCIC_NOTI_UNDEFINED;
	}
#else
	ccic->ccic_evt_attached = pnoti->attach ?
		MUIC_CCIC_NOTI_ATTACH : MUIC_CCIC_NOTI_DETACH;
#endif

	/* Attached */
	if (ccic->ccic_evt_attached == MUIC_CCIC_NOTI_ATTACH) {
		pr_info("%s: Attach\n", __func__);

		if (ccic->ccic_evt_roleswap) {
			pr_info("%s: roleswap event, attach USB\n", __func__);
			ccic->ccic_evt_roleswap = 0;
			if (muic_manager_get_vbus(muic_if)) {
				ccic->attached_dev = ATTACHED_DEV_USB_MUIC;
				if (muic_if->set_cable_state)
					muic_if->set_cable_state(muic_if->muic_data, ccic->attached_dev);
				muic_manager_switch_path(muic_if, MUIC_PATH_USB_AP);
				MUIC_SEND_NOTI_ATTACH(ccic->attached_dev);
			}
			return 0;
		}

		if (pnoti->rprd) {
			pr_info("%s: RPRD\n", __func__);
			ccic->ccic_evt_rprd = 1;
			ccic->attached_dev = ATTACHED_DEV_OTG_MUIC;
#if IS_ENABLED(CONFIG_MUIC_HV)
			if (muic_if->hv_reset)
				muic_if->hv_reset(muic_if->muic_data);
#endif
			if (muic_if->set_cable_state)
				muic_if->set_cable_state(muic_if->muic_data, ccic->attached_dev);
			muic_manager_switch_path(muic_if, MUIC_PATH_USB_AP);
			return 0;
		}

#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC)
		if (muic_if->is_afc_reset) {
			pr_info("%s: DCD RESCAN after afc reset\n", __func__);
			muic_if->is_afc_reset = false;
			if (muic_if->set_dcd_rescan != NULL && !muic_if->is_dcp_charger)
				muic_if->set_dcd_rescan(muic_if->muic_data);
		}
#endif

		if (muic_manager_is_valid_rid_open(muic_if, vbus))
			pr_info("%s: Valid VBUS-> handled in irq handler\n", __func__);
		else
			pr_info("%s: No VBUS-> doing nothing.\n", __func__);

		/* CCIC ATTACH means NO WATER */
		if (muic_if->afc_water_disable) {
			pr_info("%s: Water is not detected, AFC Enable\n", __func__);
			muic_if->afc_water_disable = false;
		}

		/* W/A for Incomplete insertion case */
		ccic->ccic_evt_dcdcnt = 0;
		if (muic_if->is_dcdtmr_intr && vbus) {
			if (muic_if->vps.t.chgdetrun) {
				pr_info("%s: Incomplete insertion. Chgdet runnung\n", __func__);
				return 0;
			}
			pr_info("%s: Incomplete insertion. Do chgdet again\n", __func__);
			muic_if->is_dcdtmr_intr = false;

			if (muic_if->set_dcd_rescan != NULL)
				muic_if->set_dcd_rescan(muic_if->muic_data);
		}

	} else {
		if (pnoti->rprd) {
			/* Role swap detach: attached=0, rprd=1 */
			pr_info("%s: role swap event\n", __func__);
			ccic->ccic_evt_roleswap = 1;
		} else if (vbus && !muic_core_get_ccic_cable_state(pdata)) {
				if (muic_if->is_dcp_charger) {
					pr_info("%s: reset afc\n", __func__);
#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC) || IS_ENABLED(CONFIG_MUIC_HV)
				if (muic_if->set_afc_reset)
					muic_if->set_afc_reset(muic_if->muic_data);
#endif
			}
			pr_info("%s: Valid VBUS, return\n", __func__);
		} else {
			/* Detached */
			muic_manager_handle_ccic_detach(muic_if);
		}

		muic_manager_handle_ccic_detach_always(muic_if);
	}

	return 0;
}

static int muic_manager_handle_ccic_factory_jig(struct muic_interface_t *muic_if, int rid, int vbus)
{
	struct ccic_desc_t *ccic = muic_if->ccic;
	struct muic_platform_data *pdata = muic_if->pdata;
	int attached_dev = 0;

	pr_info("%s: rid:%d vbus:%d\n", __func__, rid, vbus);

	switch (rid) {
	case CCIC_RID_255K:
	case CCIC_RID_301K:
		if (pdata->jig_uart_cb)
			pdata->jig_uart_cb(1);
		muic_manager_switch_path(muic_if, MUIC_PATH_USB_AP);
		break;
	case CCIC_RID_523K:
	case CCIC_RID_619K:
		if (pdata->jig_uart_cb)
			pdata->jig_uart_cb(1);
		muic_manager_switch_path(muic_if, MUIC_PATH_UART_AP);
		break;
	default:
		pr_info("%s: Unsupported rid\n", __func__);
		return 0;
	}

	attached_dev = muic_manager_conv_rid_to_dev(muic_if, rid, vbus);

	if (attached_dev != ccic->attached_dev) {
		if (muic_manager_is_supported_dev(ccic->attached_dev)) {
			MUIC_SEND_NOTI_DETACH(ccic->attached_dev);
			ccic->attached_dev = 0;
		} else if (muic_if->legacy_dev != ATTACHED_DEV_NONE_MUIC) {
			MUIC_SEND_NOTI_DETACH(muic_if->legacy_dev);
			muic_if->legacy_dev = 0;
		}

		ccic->attached_dev = attached_dev;
#if IS_ENABLED(CONFIG_MUIC_HV)
		if (muic_if->hv_reset)
			muic_if->hv_reset(muic_if->muic_data);
#endif
		if (muic_if->set_cable_state)
			muic_if->set_cable_state(muic_if->muic_data, ccic->attached_dev);
		MUIC_SEND_NOTI_ATTACH(attached_dev);
	}

	return 0;
}

static int muic_manager_handle_ccic_rid(struct muic_interface_t *muic_if, void *data)
{
	struct ccic_desc_t *ccic = muic_if->ccic;
	struct muic_platform_data *pdata = muic_if->pdata;
	int rid, vbus;
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	struct ifconn_notifier_template *pnoti =
	    (struct ifconn_notifier_template *)data;
#else
	CC_NOTI_RID_TYPEDEF *pnoti = (CC_NOTI_RID_TYPEDEF *) data;
#endif

	pr_info("%s: src:%d dest:%d id:%d rid:%d sub2:%d sub3:%d\n", __func__,
		pnoti->src, pnoti->dest, pnoti->id, pnoti->rid, pnoti->sub2,
		pnoti->sub3);

#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	rid = pnoti->event;
#else
	rid = pnoti->rid;
#endif
	if (rid > CCIC_RID_OPEN) {
		pr_info("%s: Out of range of RID\n", __func__);
		return 0;
	}

	if (ccic->ccic_evt_attached != MUIC_CCIC_NOTI_ATTACH) {
		pr_info("%s: RID but No ATTACH->discarded\n", __func__);
		return 0;
	}

	ccic->ccic_evt_rid = rid;

	switch (rid) {
	case CCIC_RID_000K:
		pr_info("%s: OTG -> RID000K\n", __func__);
		muic_manager_switch_path(muic_if, MUIC_PATH_USB_AP);
		vbus = muic_manager_get_vbus(muic_if);
		ccic->attached_dev = muic_manager_conv_rid_to_dev(muic_if, rid, vbus);
		return 0;
	case CCIC_RID_001K:
		pr_info("%s: MHL -> discarded.\n", __func__);
		return 0;
	case CCIC_RID_255K:
	case CCIC_RID_301K:
	case CCIC_RID_523K:
	case CCIC_RID_619K:
		vbus = muic_manager_get_vbus(muic_if);
		if (muic_if->set_jig_state)
			muic_if->set_jig_state(muic_if->muic_data, true);
		muic_manager_handle_ccic_factory_jig(muic_if, rid, vbus);
		break;
	case CCIC_RID_OPEN:
	case CCIC_RID_UNDEFINED:
		vbus = muic_manager_get_vbus(muic_if);
		if (ccic->ccic_evt_attached == MUIC_CCIC_NOTI_ATTACH &&
		    muic_manager_is_valid_rid_open(muic_if, vbus)) {
			if (pdata->jig_uart_cb)
				pdata->jig_uart_cb(0);
			/*
			 * USB team's requirement.
			 * Set AP USB for enumerations.
			 */
			muic_manager_switch_path(muic_if, MUIC_PATH_USB_AP);
			muic_manager_handle_legacy_dev(muic_if);
		} else {
			/* RID OPEN + No VBUS = Assume detach */
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
			muic_manager_handle_ccic_rid_open(muic_if);
#else
			muic_manager_handle_ccic_detach(muic_if);
#endif
		}
		if (muic_if->set_jig_state)
			muic_if->set_jig_state(muic_if->muic_data, false);
		break;
	default:
		pr_err("%s:Undefined RID\n", __func__);
		return 0;
	}

	return 0;
}

static int muic_manager_handle_ccic_water(struct muic_interface_t *muic_if, void *data)
{
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	struct ifconn_notifier_template *pnoti = (struct ifconn_notifier_template *)data;
#else
	CC_NOTI_ATTACH_TYPEDEF *pnoti = (CC_NOTI_ATTACH_TYPEDEF *) data;
#endif

	pr_info("%s: src:%d dest:%d id:%d attach:%d cable_type:%d rprd:%d\n", __func__,
		pnoti->src, pnoti->dest, pnoti->id, pnoti->attach, pnoti->cable_type, pnoti->rprd);

	muic_if->afc_water_disable = pnoti->attach ? true : false;
	if (muic_if->set_water_detect)
		muic_if->set_water_detect(muic_if->muic_data, muic_if->afc_water_disable);

	pr_info("%s: Water detect : %s\n", __func__, pnoti->attach ? "en":"dis");

	return 0;
}

#if !IS_ENABLED(CONFIG_SEC_FACTORY)
static int muic_manager_handle_ccic_water_from_boot(struct muic_interface_t *muic_if, void *data)
{
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	struct ifconn_notifier_template *pnoti = (struct ifconn_notifier_template *)data;
#else
	CC_NOTI_ATTACH_TYPEDEF *pnoti = (CC_NOTI_ATTACH_TYPEDEF *)data;
#endif

	pr_info("%s: src:%d dest:%d id:%d attach:%d cable_type:%d rprd:%d\n", __func__,
		pnoti->src, pnoti->dest, pnoti->id, pnoti->attach, pnoti->cable_type, pnoti->rprd);

	if (muic_if->set_water_detect_from_boot)
		muic_if->set_water_detect_from_boot(muic_if->muic_data, true);
	return 0;
}

#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC) || IS_ENABLED(CONFIG_MUIC_HV)
static int muic_manager_handle_ccic_TA(struct muic_interface_t *muic_if, void *data)
{
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	struct ifconn_notifier_template *pnoti = (struct ifconn_notifier_template *)data;
#else
	CC_NOTI_ATTACH_TYPEDEF *pnoti = (CC_NOTI_ATTACH_TYPEDEF *) data;
#endif

	pr_info("%s: src:%d dest:%d id:%d attach:%d cable_type:%d rprd:%d\n", __func__,
		pnoti->src, pnoti->dest, pnoti->id, pnoti->attach, pnoti->cable_type, pnoti->rprd);

	muic_if->is_afc_pdic_ready = true;
	if (muic_if->set_afc_ready)
		muic_if->set_afc_ready(muic_if->muic_data, muic_if->is_afc_pdic_ready);

	return 0;
}
#endif
#endif

static int muic_manager_handle_ccic_role_swap(struct muic_interface_t *muic_if, void *data)
{
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	struct ifconn_notifier_template *pnoti = (struct ifconn_notifier_template *)data;

	struct ccic_desc_t *ccic = muic_if->ccic;

	pr_info("%s: event:%d\n", __func__, pnoti->event);

	if (pnoti->event == IFCONN_NOTIFY_EVENT_PD_SINK) {
		ccic->ccic_evt_roleswap = 1;
	} else if (pnoti->event == IFCONN_NOTIFY_EVENT_PD_SOURCE) {
		MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_OTG_MUIC);
		ccic->ccic_evt_rprd = 1;
		ccic->attached_dev = ATTACHED_DEV_OTG_MUIC;
		if (muic_if->set_cable_state)
			muic_if->set_cable_state(muic_if->muic_data, ccic->attached_dev);
		muic_manager_switch_path(muic_if, MUIC_PATH_USB_AP);
	}
#endif
	return 0;
}

static int muic_manager_handle_otg(struct muic_interface_t *muic_if, void *data)
{
	struct ccic_desc_t *ccic = muic_if->ccic;
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	struct ifconn_notifier_template *pnoti = (struct ifconn_notifier_template *)data;
#else
	CC_NOTI_TYPEDEF *pnoti = (CC_NOTI_TYPEDEF *) data;
#endif

	pr_info("%s: src:%d dest:%d id:%d\n", __func__,
		pnoti->src, pnoti->dest, pnoti->id);

#if !IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	if (pnoti->sub1 == true) {
#endif
		MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_OTG_MUIC);
		ccic->ccic_evt_rprd = 1;
		ccic->attached_dev = ATTACHED_DEV_OTG_MUIC;
		if (muic_if->set_cable_state)
			muic_if->set_cable_state(muic_if->muic_data, ccic->attached_dev);
		muic_manager_switch_path(muic_if, MUIC_PATH_USB_AP);
#if !IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	}
#endif

	return 0;
}

#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
static int muic_manager_handle_notification(struct muic_interface_t *muic_if,
					    unsigned long action, void *data)
#else
static int muic_manager_handle_notification(struct notifier_block *nb,
					    unsigned long action, void *data)
#endif
{
#if IS_ENABLED(CONFIG_USB_TYPC_MANAGER_NOTIFIER)
	struct muic_interface_t *muic_if =
	    container_of(nb, struct muic_interface_t, ifconn_nb);
#endif

#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	struct ifconn_notifier_template *pnoti =
	    (struct ifconn_notifier_template *)data;
	int attach = IFCONN_NOTIFY_ID_ATTACH;
	int rid = IFCONN_NOTIFY_ID_RID;
	int water = IFCONN_NOTIFY_ID_WATER;
	int otg = IFCONN_NOTIFY_ID_OTG;
	int ta = IFCONN_NOTIFY_ID_TA;
	int role_swap = IFCONN_NOTIFY_ID_ROLE_SWAP;
#else
	CC_NOTI_TYPEDEF *pnoti = (CC_NOTI_TYPEDEF *) data;
	int attach = CCIC_NOTIFY_ID_ATTACH;
	int rid = CCIC_NOTIFY_ID_RID;
	int water = CCIC_NOTIFY_ID_WATER;
	int otg = CCIC_NOTIFY_ID_OTG;
	int ta = CCIC_NOTIFY_ID_TA;
#if IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
	if (pnoti->dest != CCIC_NOTIFY_DEV_MUIC) {
		pr_info("%s destination id is invalid\n", __func__);
		return 0;
	}
#endif
#endif
	muic_manager_show_status(muic_if);

	if (pnoti->id == attach) {
#if !IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
		pr_info("%s: NOTIFY_ID_ATTACH: %s\n", __func__,
			pnoti->attach ? "Attached" : "Detached");
#endif
		muic_manager_handle_ccic_attach(muic_if, data);
	} else if (pnoti->id == rid) {
		pr_info("%s: NOTIFY_ID_RID\n", __func__);
		muic_manager_handle_ccic_rid(muic_if, data);
	} else if (pnoti->id == water) {

#if !IS_ENABLED(CONFIG_SEC_FACTORY)
		pr_info("%s: NOTIFY_ID_WATER, boot water : %d\n", __func__, pnoti->sub2);
		if (pnoti->sub2)
			muic_manager_handle_ccic_water_from_boot(muic_if, data);
		else
			muic_manager_handle_ccic_water(muic_if, data);
#else
		pr_info("%s: NOTIFY_ID_WATER\n", __func__);
		muic_manager_handle_ccic_water(muic_if, data);
#endif
	} else if (pnoti->id == otg) {
		pr_info("%s: NOTIFY_ID_OTG\n", __func__);
		muic_manager_handle_otg(muic_if, data);
	} else if (pnoti->id == ta) {
		pr_info("%s: NOTIFY_ID_TA\n", __func__);
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC) || IS_ENABLED(CONFIG_MUIC_HV)
		muic_manager_handle_ccic_TA(muic_if, data);
#endif
#endif
	} else if (pnoti->id == role_swap) {
		pr_info("%s: NOTIFY_ID_ROLE_SWAP\n", __func__);
		muic_manager_handle_ccic_role_swap(muic_if, data);
	} else {
		pr_info("%s: Undefined Noti. ID\n", __func__);
	}

	muic_manager_show_status(muic_if);

	return NOTIFY_DONE;
}

#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
static int muic_manager_handle_ccic_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct muic_interface_t *muic_if = container_of(nb, struct muic_interface_t, ifconn_nb_ccic);

	return muic_manager_handle_notification(muic_if, action, data);
}

static int muic_manager_handle_pdic_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct muic_interface_t *muic_if = container_of(nb, struct muic_interface_t, ifconn_nb_pdic);

	return muic_manager_handle_notification(muic_if, action, data);
}
#endif

#if !IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
void _muic_delayed_notifier(struct work_struct *work)
{
	struct muic_interface_t *muic_if;
	int ret = 0;

	pr_info("%s\n", __func__);

	muic_if = container_of(work, struct muic_interface_t, ccic_work.work);

#if IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
	ret = manager_notifier_register(&muic_if->manager_nb,
		muic_manager_handle_notification, MANAGER_NOTIFY_CCIC_MUIC);
#else
	ret = ccic_notifier_register(&muic_if->ccic_nb,
		muic_manager_handle_notification, CCIC_NOTIFY_DEV_MUIC);
#endif

	if (ret < 0) {
		pr_info("%s: CCIC Noti. is not ready. Try again in 4sec...\n", __func__);
		schedule_delayed_work(&muic_if->ccic_work, msecs_to_jiffies(4000));
		return;
	}

	pr_info("%s: done.\n", __func__);
}

void muic_manager_register_notifier(struct muic_interface_t *muic_if)
{
	int ret = 0;

	pr_info("%s: Registering CCIC_NOTIFY_DEV_MUIC.\n", __func__);

	muic_manager_init_dev_desc(muic_if);

#if IS_ENABLED(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)
	ret = manager_notifier_register(&muic_if->manager_nb,
		muic_manager_handle_notification, MANAGER_NOTIFY_CCIC_MUIC);
#else
	ret = ccic_notifier_register(&muic_if->ccic_nb,
		muic_manager_handle_notification, CCIC_NOTIFY_DEV_MUIC);
#endif

	if (ret < 0) {
		pr_info("%s: CCIC Noti. is not ready. Try again in 8sec...\n", __func__);
		INIT_DELAYED_WORK(&muic_if->ccic_work, _muic_delayed_notifier);
		schedule_delayed_work(&muic_if->ccic_work, msecs_to_jiffies(8000));
		return;
	}

	pr_info("%s: done.\n", __func__);
}
#endif

struct muic_interface_t *muic_manager_init(void *pdata, void *drv_data)
{
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	int ret;
#endif
	struct muic_interface_t *muic_if;
	struct ccic_desc_t *ccic;

	pr_info("%s\n", __func__);

	muic_if = kzalloc(sizeof(*muic_if), GFP_KERNEL);
	if (unlikely(!muic_if)) {
		pr_err("%s failed to allocate driver data\n", __func__);
		return NULL;
	}

	ccic = kzalloc(sizeof(*ccic), GFP_KERNEL);
	if (unlikely(!ccic)) {
		pr_err("%s failed to allocate driver data\n", __func__);
		goto err_ccic_alloc;
	}

	muic_if->ccic = ccic;
	muic_if->muic_data = drv_data;
	muic_if->pdata = pdata;
	muic_if->ccic->rid_desc = ccic_rid_tbl;
	muic_if->is_afc_reset = false;
	muic_if->is_dcp_charger = false;
	muic_if->opmode = get_ccic_info() & 0xF;
	muic_if->is_dcdtmr_intr = false;
#if IS_ENABLED(CONFIG_MUIC_HV)
	muic_if->is_afc_pdic_ready = false;
#endif
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
	ret = ifconn_notifier_register(&muic_if->ifconn_nb_ccic,
				       muic_manager_handle_ccic_notification,
				       IFCONN_NOTIFY_MUIC, IFCONN_NOTIFY_CCIC);
	ret = ifconn_notifier_register(&muic_if->ifconn_nb_pdic,
				       muic_manager_handle_pdic_notification,
				       IFCONN_NOTIFY_MUIC, IFCONN_NOTIFY_PDIC);
	if (ret) {
		pr_err("%s failed register ifconn notifier\n", __func__);
		goto err_reg_noti;
	}
#else
	if (muic_if->opmode & S2M_OPMODE_CCIC)
		muic_manager_register_notifier(muic_if);
	else
		pr_info("S2M_OPMODE_MUIC CCIC NOTIFIER is not used\n");
#endif
	return muic_if;
#if IS_ENABLED(CONFIG_IFCONN_NOTIFIER)
err_reg_noti:
	kfree(ccic);
#endif
err_ccic_alloc:
	kfree(muic_if);
	return NULL;
}

#if IS_ENABLED(CONFIG_MUIC_SUPPORT_POWERMETER)
static enum power_supply_property muic_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *muic_supplied_to[] = {
	"battery",
};

static int muic_manager_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	switch (psp) {
	default:
		return -EINVAL;
	}

	return 0;
}

static int muic_manager_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct muic_interface_t *muic_if =
		power_supply_get_drvdata(psy);
	enum s2m_power_supply_property s2m_psp = (enum s2m_power_supply_property) psp;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_S2M_PROP_MIN ... POWER_SUPPLY_S2M_PROP_MAX:
		switch (s2m_psp) {
		case POWER_SUPPLY_S2M_PROP_AFC_CHARGER_MODE:
			MUIC_PDATA_FUNC_MULTI_PARAM(muic_if->pm_chgin_irq,
				muic_if->muic_data, val->intval, &ret);
			break;
		default:
			return -EINVAL;
		}
		return 0;
	default:
		return -EINVAL;
	}
	return 0;
}

int muic_manager_psy_init(struct muic_interface_t *muic_if, struct device *parent)
{
	struct power_supply_config psy_cfg = {};
	int ret = 0;

	if (muic_if == NULL || parent == NULL) {
		pr_err("%s NULL data\n", __func__);
		return -1;
	}

	muic_if->psy_muic_desc.name           = "muic-manager";
	muic_if->psy_muic_desc.type           = POWER_SUPPLY_TYPE_UNKNOWN;
	muic_if->psy_muic_desc.get_property   = muic_manager_get_property;
	muic_if->psy_muic_desc.set_property   = muic_manager_set_property;
	muic_if->psy_muic_desc.properties     = muic_props;
	muic_if->psy_muic_desc.num_properties = ARRAY_SIZE(muic_props);

	psy_cfg.drv_data = muic_if;
	psy_cfg.supplied_to = muic_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(muic_supplied_to);

	muic_if->psy_muic = power_supply_register(parent, &muic_if->psy_muic_desc, &psy_cfg);
	if (IS_ERR(muic_if->psy_muic)) {
		ret = (int)PTR_ERR(muic_if->psy_muic);
		pr_err("%s: Failed to Register psy_muic, ret : %d\n", __func__, ret);
	}
	return ret;
}
#endif

void muic_manager_exit(struct muic_interface_t *muic_if)
{
#if IS_ENABLED(CONFIG_MUIC_SUPPORT_POWERMETER)
	power_supply_unregister(muic_if->psy_muic);
#endif
	kfree(muic_if);
}
