/*
 * cps4019_charger.h
 * Samsung CPS4019 Charger Header
 *
 * Copyright (C) 2015 Samsung Electronics, Inc.
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

#ifndef __CPS4019_CHARGER_H
#define __CPS4019_CHARGER_H __FILE__

#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>

#include <linux/battery/sb_misc.h>
#include <linux/battery/sb_vote.h>
#include <linux/battery/sb_pqueue.h>
#include <linux/battery/sb_notify.h>

#include <linux/battery/wireless/sb_wrl_fw.h>
#include <linux/battery/wireless/sb_wrl_def.h>

#define CPS_IRQ_DISWAKE		(1 << 4)
#define CPS_IRQ_ENWAKE		(1 << 3)
#define CPS_IRQ_DISNOSYNC	(1 << 2)
#define CPS_IRQ_DIS			(1 << 1)
#define CPS_IRQ_EN			(1)

struct cps4019_gpio {
	const char *name;
	int gpio;
	int irq;
	unsigned int flags;
	unsigned int state;
};

enum {
	CPS4019_GPIO_GP1 = 0,
	CPS4019_GPIO_PDETB,
	CPS4019_GPIO_INTB,

	CPS4019_GPIO_MAX
};

struct cps4019_charger_platform_data {
	char *wireless_charger_name;
	char *charger_name;
	char *fuelgauge_name;
	char *wireless_name;
	char *battery_name;

	struct cps4019_gpio gpios[CPS4019_GPIO_MAX];
	int wpc_en;
	int wpc_ping_en;

	int cc_cv_threshold; /* cc/cv threshold */
	unsigned int half_bridge_threshold;
	unsigned int half_bridge_vout;
	unsigned int darkzone_expired_time;
	unsigned int low_current_expired_time;
	unsigned int ab_acok_count;
	unsigned int ab_acok_time;

	int otp_firmware_ver;

	int tx_off_high_temp;
	int ping_duration;
	int rx_id;
	int support_legacy_pad;

	struct sb_wrl_tx *tx_table;
	unsigned int tx_table_size;
};

#define CPS4019_OP_FREQ_CHECK_BUF_NUM		3
#define cps4019_charger_platform_data_t \
	struct cps4019_charger_platform_data

struct cps4019_charger_data {
	struct i2c_client				*client;
	struct device					*dev;
	cps4019_charger_platform_data_t	*pdata;
	struct mutex charger_lock;
	struct mutex auth_lock;

	struct sb_wrl_fw *fw;
	struct sb_pqueue *pq;
	struct sb_misc *misc;

	struct mutex gpio_lock;
	int det_gpio_type;

	struct power_supply *psy_chg;
	struct wakeup_source *wpc_ws;
	struct wakeup_source *det_ws;
	struct wakeup_source *wpc_data_check_ws;
	struct wakeup_source *wpc_id_check_ws;
	struct workqueue_struct *wqueue;
	struct delayed_work	init_work;
	struct delayed_work	wpc_det_work;
	struct delayed_work	wpc_isr_work;
	struct delayed_work wpc_id_check_work;

	struct wakeup_source *wpc_id_request_ws;
	struct delayed_work	wpc_id_request_work;

	struct wakeup_source *power_hold_chk_ws;
	struct delayed_work power_hold_chk_work;

	struct wakeup_source *phm_free_ws;
	struct delayed_work phm_free_work;

	struct wakeup_source *retry_phm_free_ws;
	struct delayed_work retry_phm_free_work;

	struct wakeup_source *cs100_ws;
	struct delayed_work cs100_work;

	struct wakeup_source *darkzone_ws;
	struct delayed_work darkzone_work;

	struct wakeup_source *ab_acok_ws;

	struct wakeup_source *check_fw_ws;
	struct delayed_work check_fw_work;

	struct wakeup_source *check_auth_ws;
	struct delayed_work auth_done_work;

	struct wakeup_source *auth_retry_ws;
	struct delayed_work auth_retry_work;

	struct wakeup_source *wpc_set_ws;
	struct delayed_work wpc_set_work;

	struct	notifier_block sb_nb;
	struct	notifier_block wpc_nb;

	struct sb_vote *en_vote;

	struct sb_vote *det_vote;
	struct sb_vote *fcc_vote;

	struct sb_vote *phm_vote;
	struct sb_vote *vm_vote;
	struct sb_vote *dc_hdr_vote;
	struct sb_vote *ichg_vote;
	struct sb_vote *dc_high_vout_vote;
	struct sb_vote *low_vout_vote;
	struct sb_vote *bt_vbat_hdr_vote;
	struct sb_vote *bt_hdr_vote;
	struct sb_vote *vout_vote;
	struct sb_vote *fx_hdr_vote;

	struct sb_vote *ldo_vote;
	struct sb_vote *vout_off_vote;

	struct wakeup_source *align_check_ws;
	struct delayed_work align_check_work;
	int d2d_vout_strength;
	struct timespec64 d2d_align_check_start;
	bool d2d_checking_align;

	struct sb_wrl_tx now_tx;

	bool wc_w_state;
	bool is_charging;
	bool force_wpc_det_chk;

	u16 addr;
	int size;
	char d_buf[128];

	int temperature;

	/* sec_battery's battery health */
	int battery_health;

	/* sec_battery's battery status */
	int battery_status;

	/* sec_battery's charge mode */
	int battery_chg_mode;

	/* sec_battery's swelling mode */
	int swelling_mode;

	/* sec_battery's store(LDU) mode */
	int store_mode;

	/* check is recharge status */
	int is_recharge;

	int cable_type;

	/* TX id checked twice, if tx id checked normarlly, skip second tx id check */
	unsigned int tx_id_check_cnt;

	/* RX PWR checked */
	unsigned int rx_pwr_check_cnt;

	/* AUTH checked */
	unsigned int auth_check_cnt;

	/* charge mode cc/cv mode */
	int charge_mode;

	/* cs100 flag */
	bool cs100_status;

	unsigned int phm_chk_cnt;

	/* watchdog test */
	bool watchdog_test;

	/* wireless wa */
	bool ssp_missing;
	unsigned int ssp_missing_count;
	bool darkzone_reset;
	unsigned long darkzone_start_time;
	unsigned long low_current_start_time;
	unsigned int ab_acok_count;
	unsigned long ab_acok_start_time;
};

ssize_t sec_wpc_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t sec_wpc_store_attrs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

#define SEC_WPC_ATTR(_name)						\
{									\
	.attr = {.name = #_name, .mode = 0664},	\
	.show = sec_wpc_show_attrs,					\
	.store = sec_wpc_store_attrs,					\
}

#if IS_ENABLED(CONFIG_SB_STARTUP)
int cps4019_startup_init(struct cps4019_charger_data *charger);
#else
static inline int cps4019_startup_init(struct cps4019_charger_data *charger)
{ return 0; }
#endif

#endif /* __CPS4019_CHARGER_H */
