// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include "mcu_ipc.h"
#include <soc/samsung/shm_ipc.h>
#include <soc/samsung/cal-if.h>
#include <soc/samsung/exynos-modem-ctrl.h>
#include <linux/suspend.h>
#if IS_ENABLED(CONFIG_EXYNOS_PMU_IF)
#include <soc/samsung/exynos-pmu-if.h>
#else
#include <soc/samsung/exynos-pmu.h>
#endif
#if IS_ENABLED(CONFIG_REINIT_VSS)
#include <sound/samsung/abox.h>
#endif
#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_ctrl.h"
#include "modem_notifier.h"
#include "link_device_memory.h"
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
#include "s51xx_pcie.h"
#endif
#if IS_ENABLED(CONFIG_SBD_BOOTLOG)
#include "link_device.h"
#endif
#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
#include "../../../video/fbdev/exynos/dpu30/decon.h"
static int s5000ap_lcd_notifier(struct notifier_block *notifier,
		unsigned long event, void *v);
#endif /* CONFIG_CP_LCD_NOTIFIER */

#if IS_ENABLED(CONFIG_UART_SEL)
extern void cp_recheck_uart_dir(void);
#endif

static struct modem_ctl *g_mc;

/*
 * CP_WDT interrupt handler
 */
#if IS_ENABLED(CONFIG_SOC_S5E9925) && !IS_ENABLED(CONFIG_SOC_S5E9925_EVT0)
#define PMU_CP_CTRL_NS		0x3910
#define PMU_CP_CTRL_S		0x3914
#define PMU_CP_OUT		0x3920
#define PMU_CP_IN		0x3924
#define PMU_CP_INT_IN		0x3930
#define PMU_CP_INT_EN		0x3934
#define PMU_CP_INT_TYPE		0x3938
#define PMU_CP_INT_DIR		0x393C
#define PMU_V_PWREN		0x3EF0
#endif
static irqreturn_t cp_wdt_handler(int irq, void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	enum modem_state new_state;
	struct link_device *ld = get_current_link(mc->bootd);
	u32 val = 0;

	mif_info("%s: CP_WDT occurred\n", mc->name);
	mif_disable_irq(&mc->irq_cp_wdt);

	if (mc->s2d_req_handle_support) {
		exynos_pmu_read(mc->s2d_req_pmu_reg, &val);
		mif_info("supports s2d request handling. pmu val: 0x%X\n", val);
		if (val & (0x1 << mc->s2d_req_reg_mask)) {
			mif_info("cp scandump request detected\n");
			cpif_try_ap_watchdog_reset();
			return IRQ_HANDLED;
		}
	}

	if (mc->phone_state == STATE_ONLINE)
		modem_notify_event(MODEM_EVENT_WATCHDOG, mc);

	mif_stop_logging();

	new_state = STATE_CRASH_WATCHDOG;
	ld->crash_reason.type = CRASH_REASON_CP_WDOG_CRASH;

	mif_info("new_state:%s\n", cp_state_str(new_state));

	change_modem_state(mc, new_state);

	return IRQ_HANDLED;
}

/*
 * ACTIVE mailbox interrupt handler
 */
static irqreturn_t cp_active_handler(int irq, void *arg)
{
	struct modem_ctl *mc = (struct modem_ctl *)arg;
	int cp_on = cal_cp_status();
	int cp_active = 0;
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;
	enum modem_state old_state = mc->phone_state;
	enum modem_state new_state = mc->phone_state;

	cp_active = extract_ctrl_msg(&mld->cp2ap_united_status, mc->sbi_lte_active_mask,
			mc->sbi_lte_active_pos);

	mif_info("old_state:%s cp_on:%d cp_active:%d\n",
		cp_state_str(old_state), cp_on, cp_active);

	if (!cp_active) {
		if (cp_on > 0) {
			new_state = STATE_OFFLINE;
			complete_all(&mc->off_cmpl);
		} else {
			mif_info("don't care!!!\n");
		}
	}

	if (old_state != new_state) {
		mif_info("new_state = %s\n", cp_state_str(new_state));

#if IS_ENABLED(CONFIG_REINIT_VSS)
		reinit_completion(&mc->vss_stop);
#endif
		if (old_state == STATE_ONLINE)
			modem_notify_event(MODEM_EVENT_RESET, mc);

		change_modem_state(mc, new_state);
	}

	return IRQ_HANDLED;
}

static int hw_rev;
#if IS_ENABLED(CONFIG_HW_REV_DETECT)
#if defined(MODULE)
/* GKI TODO */
#else /* MODULE */
static int __init console_setup(char *str)
{
	get_option(&str, &hw_rev);
	mif_info("hw_rev:0x%x\n", hw_rev);

	return 0;
}
__setup("androidboot.revision=", console_setup);

static int __init set_hw_revision(char *str)
{
	get_option(&str, &hw_rev);
	mif_info("Hardware revision:0x%x\n", hw_rev);

	return 0;
}
__setup("revision=", set_hw_revision);
#endif /* MODULE */
#else /* CONFIG_HW_REV_DETECT */
static int get_system_rev(struct device_node *np)
{
	int value, cnt, gpio_cnt;
	unsigned int gpio_hw_rev, hw_rev = 0;

	gpio_cnt = of_gpio_count(np);
	if (gpio_cnt < 0) {
		mif_err("failed to get gpio_count from DT(%d)\n", gpio_cnt);
		return 0;
	}

	for (cnt = 0; cnt < gpio_cnt; cnt++) {
		gpio_hw_rev = of_get_gpio(np, cnt);
		if (!gpio_is_valid(gpio_hw_rev)) {
			mif_err("gpio_hw_rev%d: Invalied gpio\n", cnt);
			return -EINVAL;
		}

		value = gpio_get_value(gpio_hw_rev);
		hw_rev |= (value & 0x1) << cnt;
	}

	return hw_rev;
}
#endif /* CONFIG_HW_REV_DETECT */

#if IS_ENABLED(CONFIG_GPIO_DS_DETECT)
static int get_ds_detect(struct device_node *np)
{
	unsigned int gpio_ds_det;

	gpio_ds_det = of_get_named_gpio(np, "mif,gpio_ds_det", 0);
	if (!gpio_is_valid(gpio_ds_det)) {
		mif_err("gpio_ds_det: Invalid gpio\n");
		return 0;
	}

	return gpio_get_value(gpio_ds_det);
}
#else
static int ds_detect = 1;
module_param(ds_detect, int, 0664);
MODULE_PARM_DESC(ds_detect, "Dual SIM detect");

static ssize_t ds_detect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", ds_detect);
}

static ssize_t ds_detect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	int value;

	ret = kstrtoint(buf, 0, &value);
	if (ret != 0) {
		mif_err("invalid value:%d with %d\n", value, ret);
		return -EINVAL;
	}

	ds_detect = value;
	mif_info("set ds_detect: %d\n", ds_detect);

	return count;
}
static DEVICE_ATTR_RW(ds_detect);

static struct attribute *sim_attrs[] = {
	&dev_attr_ds_detect.attr,
	NULL,
};

static const struct attribute_group sim_group = {
	.attrs = sim_attrs,
	.name = "sim",
};

static int get_ds_detect(struct device_node *np)
{
	if (ds_detect > 2 || ds_detect < 1)
		ds_detect = 2;

	mif_info("Dual SIM detect = %d\n", ds_detect);
	return ds_detect - 1;
}

#endif

static int init_control_messages(struct modem_ctl *mc)
{
	struct platform_device *pdev = to_platform_device(mc->dev);
	struct device_node *np = pdev->dev.of_node;
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;
	struct link_device *ld = get_current_link(mc->iod);
	unsigned int sbi_sys_rev_mask, sbi_sys_rev_pos;
	int ds_det;
#if IS_ENABLED(CONFIG_CP_BTL)
	unsigned int sbi_ext_backtrace_mask, sbi_ext_backtrace_pos;
#endif

	if (modem->offset_cmsg_offset)
		iowrite32(modem->cmsg_offset, mld->cmsg_offset);
	if (modem->offset_srinfo_offset)
		iowrite32(modem->srinfo_offset, mld->srinfo_offset);
	if (modem->offset_clk_table_offset)
		iowrite32(modem->clk_table_offset, mld->clk_table_offset);
	if (modem->offset_buff_desc_offset)
		iowrite32(modem->buff_desc_offset, mld->buff_desc_offset);
	if (ld->capability_check && modem->offset_capability_offset)
		iowrite32(modem->capability_offset, mld->capability_offset);

	set_ctrl_msg(&mld->ap2cp_united_status, 0);
	set_ctrl_msg(&mld->cp2ap_united_status, 0);
	set_ctrl_msg(&mld->ap2cp_msg, 0);
	set_ctrl_msg(&mld->cp2ap_msg, 0);
#if IS_ENABLED(CONFIG_CP_LLC)
	set_ctrl_msg(&mld->ap2cp_llc_status, 0);
	set_ctrl_msg(&mld->cp2ap_llc_status, 0);
#endif
	if (ld->capability_check) {
		int part;

		for (part = 0; part < AP_CP_CAP_PARTS; part++) {
			iowrite32(0, mld->ap_capability_offset[part]);
			iowrite32(0, mld->cp_capability_offset[part]);
		}
	}

	if (!np) {
		mif_err("non-DT project, can't set mailbox regs\n");
		return -1;
	}

#if IS_ENABLED(CONFIG_CP_BTL)
	mif_info("btl enable:%d\n", mc->mdm_data->btl.enabled);
	mif_dt_read_u32(np, "sbi_ext_backtrace_mask", sbi_ext_backtrace_mask);
	mif_dt_read_u32(np, "sbi_ext_backtrace_pos", sbi_ext_backtrace_pos);
	update_ctrl_msg(&mld->ap2cp_united_status, mc->mdm_data->btl.enabled,
				sbi_ext_backtrace_mask, sbi_ext_backtrace_pos);
#endif

	mif_dt_read_u32(np, "sbi_sys_rev_mask", sbi_sys_rev_mask);
	mif_dt_read_u32(np, "sbi_sys_rev_pos", sbi_sys_rev_pos);

	ds_det = get_ds_detect(np);
	if (ds_det < 0) {
		mif_err("ds_det error:%d\n", ds_det);
		return -EINVAL;
	}

	update_ctrl_msg(&mld->ap2cp_united_status, ds_det, mc->sbi_ds_det_mask,
			mc->sbi_ds_det_pos);
	mif_info("ds_det:%d\n", ds_det);

#if !IS_ENABLED(CONFIG_HW_REV_DETECT)
	hw_rev = get_system_rev(np);
#endif
	if ((hw_rev < 0) || (hw_rev > sbi_sys_rev_mask)) {
		mif_err("hw_rev error:0x%x. set to 0\n", hw_rev);
		hw_rev = 0;
	}
	update_ctrl_msg(&mld->ap2cp_united_status, hw_rev, sbi_sys_rev_mask,
			sbi_sys_rev_pos);
	mif_info("hw_rev:0x%x\n", hw_rev);

	return 0;
}

static bool _is_first_boot = true;
static int power_on_cp(struct modem_ctl *mc)
{
	mif_info("+++\n");

	change_modem_state(mc, STATE_OFFLINE);

	if (cal_cp_status() == 0) {
		if (_is_first_boot) {
			mif_info("First init\n");
			cal_cp_disable_dump_pc_no_pg();
			cal_cp_init();
			_is_first_boot = false;
		} else {
			mif_err("Not first time, but power is down\n");
		}
	}

#if IS_ENABLED(CONFIG_UART_SEL)
	mif_info("Recheck UART direction.\n");
	cp_recheck_uart_dir();
#endif

	mif_info("---\n");
	return 0;
}

static int power_off_cp(struct modem_ctl *mc)
{
	mif_info("+++\n");

	cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mc->int_cp_wakeup);
	usleep_range(5000, 10000);

	cal_cp_disable_dump_pc_no_pg();
	cal_cp_reset_assert();

	mif_info("---\n");
	return 0;
}

static int power_shutdown_cp(struct modem_ctl *mc)
{
	unsigned long timeout = msecs_to_jiffies(1000);
	unsigned long remain;

	mif_info("+++\n");

	if (mc->phone_state == STATE_OFFLINE || cal_cp_status() == 0)
		goto exit;

	reinit_completion(&mc->off_cmpl);
	remain = wait_for_completion_timeout(&mc->off_cmpl, timeout);
	if (remain == 0)
		change_modem_state(mc, STATE_OFFLINE);

exit:
	cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mc->int_cp_wakeup);
	usleep_range(5000, 10000);

	cal_cp_disable_dump_pc_no_pg();
	cal_cp_reset_assert();

	mif_info("---\n");
	return 0;
}


#if IS_ENABLED(CONFIG_SOC_S5E9925) && !IS_ENABLED(CONFIG_SOC_S5E9925_EVT0)
static void print_cal(void)
{
	u32 val = 0;

	mdelay(10);
	mif_info("Printing PMU registers\n");
	exynos_pmu_read(PMU_CP_CTRL_NS, &val);
	mif_info("CP_CTRL_NS: 0x%X\n", val);
	exynos_pmu_read(PMU_CP_CTRL_S, &val);
	mif_info("CP_CTRL_S (smc read): 0x%X\n", val);
	exynos_pmu_read(PMU_CP_OUT, &val);
	mif_info("CP_OUT: 0x%X\n", val);
	exynos_pmu_read(PMU_CP_IN, &val);
	mif_info("CP_IN: 0x%X\n", val);
	exynos_pmu_read(PMU_CP_INT_IN, &val);
	mif_info("CP_INT_IN: 0x%X\n", val);
	exynos_pmu_read(PMU_CP_INT_EN, &val);
	mif_info("CP_INT_EN: 0x%X\n", val);
	exynos_pmu_read(PMU_CP_INT_TYPE, &val);
	mif_info("CP_INT_TYPE: 0x%X\n", val);
	exynos_pmu_read(PMU_CP_INT_DIR, &val);
	mif_info("CP_INT_DIR: 0x%X\n", val);
	exynos_pmu_read(PMU_V_PWREN, &val);
	mif_info("V_PWREN: 0x%X\n", val);
}
#endif
static int power_reset_cp(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	int ret = 0;

	mif_info("+++\n");

	/* 2cp dump WA */
	if (timer_pending(&mld->crash_ack_timer))
		del_timer(&mld->crash_ack_timer);
	atomic_set(&mld->forced_cp_crash, 0);

	if (ld->sbd_ipc && hrtimer_active(&mld->sbd_print_timer))
		hrtimer_cancel(&mld->sbd_print_timer);

	if (mc->phone_state == STATE_OFFLINE) {
		mif_info("already offline\n");
		return 0;
	}

	if (mc->phone_state == STATE_ONLINE)
		modem_notify_event(MODEM_EVENT_RESET, mc);

	change_modem_state(mc, STATE_RESET);
	msleep(STATE_RESET_INTERVAL_MS);
	change_modem_state(mc, STATE_OFFLINE);

	if (cal_cp_status()) {
#if IS_ENABLED(CONFIG_CP_RESET_NOTI)
		mif_info("reset noti to cp");
		cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mc->int_cp_reset_noti);
#endif
		mif_info("CP aleady Init, try reset\n");
		cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mc->int_cp_wakeup);
		usleep_range(5000, 10000);

		cal_cp_disable_dump_pc_no_pg();
		cal_cp_reset_assert();
		usleep_range(5000, 10000);
		ret = cal_cp_reset_release();
		if (ret) {
			mif_err("failed to cal cp reset release\n");
#if IS_ENABLED(CONFIG_SOC_S5E9925) && !IS_ENABLED(CONFIG_SOC_S5E9925_EVT0)
			print_cal();
#endif
			cpif_try_ap_watchdog_reset();
			return ret;
		}
		cp_mbox_reset();
	}

	mif_info("---\n");
	return 0;
}

static int power_reset_dump_cp(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct mem_link_device *mld = to_mem_link_device(ld);
	int ret = 0;

	mif_info("+++\n");

	/* 2cp dump WA */
	if (timer_pending(&mld->crash_ack_timer))
		del_timer(&mld->crash_ack_timer);
	atomic_set(&mld->forced_cp_crash, 0);

	if (ld->sbd_ipc && hrtimer_active(&mld->sbd_print_timer))
		hrtimer_cancel(&mld->sbd_print_timer);

	/* mc->phone_state = STATE_OFFLINE; */
	if (mc->phone_state == STATE_OFFLINE) {
		mif_info("already offline\n");
		return 0;
	}

	if (mc->phone_state == STATE_ONLINE)
		modem_notify_event(MODEM_EVENT_RESET, mc);

	/* Change phone state to STATE_CRASH_EXIT */
	change_modem_state(mc, STATE_CRASH_EXIT);

	if (cal_cp_status()) {
		mif_info("CP aleady Init, try reset\n");
		cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mc->int_cp_wakeup);
		usleep_range(5000, 10000);

		cal_cp_enable_dump_pc_no_pg();
		cal_cp_reset_assert();
		usleep_range(5000, 10000);
		ret = cal_cp_reset_release();
		if (ret) {
			mif_err("failed to cal cp reset release\n");
#if IS_ENABLED(CONFIG_SOC_S5E9925) && !IS_ENABLED(CONFIG_SOC_S5E9925_EVT0)
			print_cal();
#endif
			cpif_try_ap_watchdog_reset();
			return ret;
		}
		cp_mbox_reset();
	}

	mif_info("---\n");
	return 0;
}

static int start_normal_boot(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;
	int cnt = 200;
	int ret = 0;
	int cp_status = 0;

	mif_info("+++\n");

	if (init_control_messages(mc))
		mif_err("Failed to initialize mbox regs\n");

	if (ld->link_prepare_normal_boot)
		ld->link_prepare_normal_boot(ld, mc->bootd);

	change_modem_state(mc, STATE_BOOTING);

	if (ld->link_start_normal_boot) {
		mif_info("link_start_normal_boot\n");
		ld->link_start_normal_boot(ld, mc->iod);
	}

	mif_info("cp_united_status:0x%08x\n", get_ctrl_msg(&mld->cp2ap_united_status));
	mif_info("ap_united_status:0x%08x\n", get_ctrl_msg(&mld->ap2cp_united_status));

	ret = modem_ctrl_check_offset_data(mc);
	if (ret) {
		mif_err("modem_ctrl_check_offset_data() error:%d\n", ret);
		return ret;
	}

	if (mc->ap2cp_cfg_ioaddr) {
		mif_info("Before setting AP2CP_CFG:0x%08x\n",
					__raw_readl(mc->ap2cp_cfg_ioaddr));
		__raw_writel(1, mc->ap2cp_cfg_ioaddr);
		ret = __raw_readl(mc->ap2cp_cfg_ioaddr);
		if (ret != 1) {
			mif_err("AP2CP_CFG setting is not correct:%d\n", ret);
			return -1;
		}
		mif_info("AP2CP_CFG is ok:0x%08x\n", ret);
	}

	while (extract_ctrl_msg(&mld->cp2ap_united_status, mld->sbi_cp_status_mask,
				mld->sbi_cp_status_pos) == 0) {
		if (--cnt > 0) {
			usleep_range(10000, 20000);
		} else {
			mif_err("cp_status is not set by CP bootloader:0x%08x\n",
						get_ctrl_msg(&mld->cp2ap_united_status));
			return -EACCES;
		}
	}

	mif_disable_irq(&mc->irq_cp_wdt);

	ret = modem_ctrl_check_offset_data(mc);
	if (ret) {
		mif_err("modem_ctrl_check_offset_data() error:%d\n", ret);
		return ret;
	}

	cp_status = extract_ctrl_msg(&mld->cp2ap_united_status,
				mld->sbi_cp_status_mask, mld->sbi_cp_status_pos);
	mif_info("cp_status=%u\n", cp_status);

	update_ctrl_msg(&mld->ap2cp_united_status, 1, mc->sbi_ap_status_mask,
			mc->sbi_ap_status_pos);
	mif_info("ap_status=%u\n", extract_ctrl_msg(&mld->ap2cp_united_status,
				mc->sbi_ap_status_mask, mc->sbi_ap_status_pos));

	update_ctrl_msg(&mld->ap2cp_united_status, 1, mc->sbi_pda_active_mask,
			mc->sbi_pda_active_pos);
	mif_info("ap_united_status:0x%08x\n", get_ctrl_msg(&mld->ap2cp_united_status));

	mif_info("---\n");
	return 0;
}

static int complete_normal_boot(struct modem_ctl *mc)
{
	unsigned long remain;
	int err = 0;
#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	int ret;
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;
#endif
#if IS_ENABLED(CONFIG_SBD_BOOTLOG)
	struct link_device *ld = get_current_link(mc->bootd);
#endif

	mif_info("+++\n");

	err = modem_ctrl_check_offset_data(mc);
	if (err) {
		mif_err("modem_ctrl_check_offset_data() error:%d\n", err);
		goto exit;
	}

	reinit_completion(&mc->init_cmpl);
	remain = wait_for_completion_timeout(&mc->init_cmpl, MIF_INIT_TIMEOUT);
	if (remain == 0) {
		mif_err("T-I-M-E-O-U-T\n");
		err = -EAGAIN;
		goto exit;
	}

	err = modem_ctrl_check_offset_data(mc);
	if (err) {
		mif_err("modem_ctrl_check_offset_data() error:%d\n", err);
		goto exit;
	}

	mif_enable_irq(&mc->irq_cp_wdt);

	change_modem_state(mc, STATE_ONLINE);

#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	if (mc->lcd_notifier.notifier_call == NULL) {
		mif_info("Register lcd notifier\n");
		mc->lcd_notifier.notifier_call = s5000ap_lcd_notifier;
		ret = register_lcd_status_notifier(&mc->lcd_notifier);
		if (ret) {
			mif_err("failed to register LCD notifier\n");
			return ret;
		}
	}

	mif_info("Set LCD_ON status\n");
	update_ctrl_msg(&mld->ap2cp_united_status, 1, mc->sbi_lcd_status_mask,
			mc->sbi_lcd_status_pos);
#endif /* CONFIG_CP_LCD_NOTIFIER */

#if IS_ENABLED(CONFIG_SBD_BOOTLOG)
	mif_add_timer(&ld->cplog_timer, (10 * HZ), shmem_pr_sbdcplog);
#endif
	mif_info("---\n");

exit:
	return err;
}

#if IS_ENABLED(CONFIG_CP_UART_NOTI)
#if IS_ENABLED(CONFIG_PMU_UART_SWITCH)
#if IS_ENABLED(CONFIG_SOC_S5E9925)
static void __iomem *uart_addr; /* SEL_TXD_RXD_GPIO_UART_DEBUG */
void change_to_cp_uart(void)
{
	if (uart_addr == NULL) {
		uart_addr = devm_ioremap(g_mc->dev, 0x11C301C0, SZ_64); /* GPG0_CON */
		if (uart_addr == NULL) {
			mif_err("Err: failed to ioremap UART DEBUG!\n");
			return;
		}
	}
	mif_info("CHANGE TO CP UART\n");
	__raw_writel(0x4400, uart_addr); /* GPG0[2], GPG0[3] - CP_UART0_TXD_RXD */
	mif_info("SEL_TXD_RXD_GPIO_UART_DEBUG val: %08X\n", __raw_readl(uart_addr));
}

void change_to_ap_uart(void)
{
	if (uart_addr == NULL) {
		uart_addr = devm_ioremap(g_mc->dev, 0x11C301C0, SZ_64); /* GPG0_CON */
		if (uart_addr == NULL) {
			mif_err("Err: failed to ioremap UART DEBUG!\n");
			return;
		}
	}
	mif_info("CHANGE TO AP UART\n");
	__raw_writel(0x3300, uart_addr); /* GPG0[2], GPG0[3] - UART_DBG_TXD_RXD */
	mif_info("SEL_TXD_RXD_GPIO_UART_DEBUG val: %08X\n", __raw_readl(uart_addr));
}
#elif IS_ENABLED(CONFIG_SOC_S5E8825)
static void __iomem *uart_txd_addr; /* SEL_TXD_GPIO_UART_DEBUG */
static void __iomem *uart_ap_rxd_addr; /* SEL_RXD_AP_UART */
static void __iomem *uart_cp_rxd_addr; /* SEL_RXD_CP_UART */
void change_to_cp_uart(void)
{
	if (uart_txd_addr == NULL) {
		uart_txd_addr = devm_ioremap(g_mc->dev, 0x1182062C, SZ_64);
		if (uart_txd_addr == NULL) {
			mif_err("Err: failed to ioremap UART TXD!\n");
			return;
		}
	}
	if (uart_cp_rxd_addr == NULL) {
		uart_cp_rxd_addr = devm_ioremap(g_mc->dev, 0x11820650, SZ_64);
		if (uart_cp_rxd_addr == NULL) {
			mif_err("Err: failed to ioremap UART RXD!\n");
			return;
		}
	}
	mif_info("CHANGE TO CP UART\n");
	__raw_writel(0x2, uart_txd_addr);
	mif_info("SEL_TXD_GPIO_UART_DEBUG val: %08X\n", __raw_readl(uart_txd_addr));
	__raw_writel(0x1, uart_cp_rxd_addr);
	mif_info("SEL_RXD_CP_UART val: %08X\n", __raw_readl(uart_cp_rxd_addr));
}

void change_to_ap_uart(void)
{
	if (uart_txd_addr == NULL) {
		uart_txd_addr = devm_ioremap(g_mc->dev, 0x1182062C, SZ_64);
		if (uart_txd_addr == NULL) {
			mif_err("Err: failed to ioremap UART TXD!\n");
			return;
		}
	}
	if (uart_ap_rxd_addr == NULL) {
		uart_ap_rxd_addr = devm_ioremap(g_mc->dev, 0x11820658, SZ_64);
		if (uart_ap_rxd_addr == NULL) {
			mif_err("Err: failed to ioremap UART RXD!\n");
			return;
		}
	}
	mif_info("CHANGE TO AP UART\n");
	__raw_writel(0x0, uart_txd_addr);
	mif_info("SEL_TXD_GPIO_UART_DEBUG val: %08X\n", __raw_readl(uart_txd_addr));
	__raw_writel(0x1, uart_ap_rxd_addr);
	mif_info("SEL_RXD_AP_UART val: %08X\n", __raw_readl(uart_ap_rxd_addr));
}
#endif /* CONFIG_SOC_EXYNOSxxxx */

void send_uart_noti_to_modem(int val)
{
	struct modem_data *modem;
	struct mem_link_device *mld;

	if (!g_mc) {
		mif_err("g_mc is NULL!\n");
		return;
	}

	modem = g_mc->mdm_data;
	mld = modem->mld;

#if IS_ENABLED(CONFIG_PMU_UART_SWITCH)
	switch (val) {
	case MODEM_CTRL_UART_CP:
		change_to_cp_uart();
		break;
	case MODEM_CTRL_UART_AP:
		change_to_ap_uart();
		break;
	default:
		mif_err("Invalid val:%d\n", val);
		return;
	}
#endif

	update_ctrl_msg(&mld->ap2cp_united_status, val, g_mc->sbi_uart_noti_mask,
			g_mc->sbi_uart_noti_pos);
	mif_info("val:%d ap_united_status:0x%08x\n", val, get_ctrl_msg(&mld->ap2cp_united_status));
	cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, g_mc->int_uart_noti);
}
EXPORT_SYMBOL(send_uart_noti_to_modem);
#endif /* CONFIG_PMU_UART_SWITCH */
#endif /* CONFIG_CP_UART_NOTI */

static int start_dump_boot(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->bootd);
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;
	int cnt = 200;
	int ret = 0;
	int cp_status = 0;

	mif_info("+++\n");

	/* Change phone state to STATE_CRASH_EXIT */
	change_modem_state(mc, STATE_CRASH_EXIT);

	if (init_control_messages(mc))
		mif_err("Failed to initialize mbox regs\n");

	if (!ld->link_start_dump_boot) {
		mif_err("%s: link_start_dump_boot is null\n", ld->name);
		return -EFAULT;
	}
	ret = ld->link_start_dump_boot(ld, mc->bootd);
	if (ret) {
		mif_err("link_start_dump_boot() error:%d\n", ret);
		return ret;
	}

	mif_info("cp_united_status:0x%08x\n", get_ctrl_msg(&mld->cp2ap_united_status));
	mif_info("ap_united_status:0x%08x\n", get_ctrl_msg(&mld->ap2cp_united_status));

	ret = modem_ctrl_check_offset_data(mc);
	if (ret) {
		mif_err("modem_ctrl_check_offset_data() error:%d\n", ret);
		return ret;
	}

	if (mc->ap2cp_cfg_ioaddr) {
		mif_info("Before setting AP2CP_CFG:0x%08x\n",
					__raw_readl(mc->ap2cp_cfg_ioaddr));
		__raw_writel(1, mc->ap2cp_cfg_ioaddr);
		ret = __raw_readl(mc->ap2cp_cfg_ioaddr);
		if (ret != 1) {
			mif_err("AP2CP_CFG setting is not correct:%d\n", ret);
			return -1;
		}
		mif_info("AP2CP_CFG is ok:0x%08x\n", ret);
	} else {
		ret = cal_cp_reset_release();
		if (ret) {
			mif_err("failed to cal cp reset release\n");
#if IS_ENABLED(CONFIG_SOC_S5E9925) && !IS_ENABLED(CONFIG_SOC_S5E9925_EVT0)
			print_cal();
#endif
			cpif_try_ap_watchdog_reset();
			return ret;
		}
	}

	while (extract_ctrl_msg(&mld->cp2ap_united_status, mld->sbi_cp_status_mask,
				mld->sbi_cp_status_pos) == 0) {
		if (--cnt > 0) {
			usleep_range(10000, 20000);
		} else {
			mif_err("cp_status error:%d\n", extract_ctrl_msg(&mld->cp2ap_united_status,
						mld->sbi_cp_status_mask, mld->sbi_cp_status_pos));
			return -EACCES;
		}
	}

	cp_status = extract_ctrl_msg(&mld->cp2ap_united_status,
				mld->sbi_cp_status_mask, mld->sbi_cp_status_pos);
	mif_info("cp_status=%u\n", cp_status);

	update_ctrl_msg(&mld->ap2cp_united_status, 1, mc->sbi_ap_status_mask,
			mc->sbi_ap_status_pos);
	mif_info("ap_status=%u\n", extract_ctrl_msg(&mld->ap2cp_united_status,
				mc->sbi_ap_status_mask, mc->sbi_cp_status_pos));

	update_ctrl_msg(&mld->ap2cp_united_status, 1, mc->sbi_pda_active_mask,
			mc->sbi_pda_active_pos);
	mif_info("ap_united_status:0x%08x\n", get_ctrl_msg(&mld->ap2cp_united_status));

	ret = modem_ctrl_check_offset_data(mc);
	if (ret) {
		mif_err("modem_ctrl_check_offset_data() error:%d\n", ret);
		return ret;
	}

	mif_info("---\n");
	return 0;
}

static int suspend_cp(struct modem_ctl *mc)
{
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;

	mld->link_dev.stop_timers(mld);

	modem_ctrl_set_kerneltime(mc);

	mif_info("%s: pda_active:0\n", mc->name);

	update_ctrl_msg(&mld->ap2cp_united_status, 0, mc->sbi_pda_active_mask,
			mc->sbi_pda_active_pos);

	cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mc->int_pda_active);

	return 0;
}

static int resume_cp(struct modem_ctl *mc)
{
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;

	modem_ctrl_set_kerneltime(mc);

	mld->link_dev.start_timers(mld);

	return 0;
}

static int s5000_pm_notifier(struct notifier_block *notifier,
				       unsigned long pm_event, void *v)
{
	struct modem_ctl *mc;
	struct modem_data *modem;
	struct mem_link_device *mld;

	mc = container_of(notifier, struct modem_ctl, pm_notifier);
	modem = mc->mdm_data;
	mld = modem->mld;

	switch (pm_event) {
	case PM_POST_SUSPEND:
		mif_info("Resume done\n");
		mif_info("%s: pda_active:1\n", mc->name);
		update_ctrl_msg(&mld->ap2cp_united_status, 1, mc->sbi_pda_active_mask,
			mc->sbi_pda_active_pos);
		cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mc->int_pda_active);
		break;
	default:
		mif_info("pm_event %lu\n", pm_event);
		break;
	}

	return NOTIFY_OK;
}

static void s5000ap_get_ops(struct modem_ctl *mc)
{
	mc->ops.power_on = power_on_cp;
	mc->ops.power_off = power_off_cp;
	mc->ops.power_shutdown = power_shutdown_cp;
	mc->ops.power_reset = power_reset_cp;
	mc->ops.power_reset_dump = power_reset_dump_cp;

	mc->ops.start_normal_boot = start_normal_boot;
	mc->ops.complete_normal_boot = complete_normal_boot;

	mc->ops.start_dump_boot = start_dump_boot;

	mc->ops.suspend = suspend_cp;
	mc->ops.resume = resume_cp;
}

static void s5000ap_get_pdata(struct modem_ctl *mc, struct modem_data *modem)
{
	struct modem_mbox *mbx = modem->mbx;

	mc->int_pda_active = mbx->int_ap2cp_active;
	mc->int_cp_wakeup = mbx->int_ap2cp_wakeup;
#if IS_ENABLED(CONFIG_CP_RESET_NOTI)
	mc->int_cp_reset_noti = mbx->int_ap2cp_cp_reset_noti;
#endif
	mc->int_uart_noti = mbx->int_ap2cp_uart_noti;
	mc->irq_phone_active = mbx->irq_cp2ap_active;

	mc->sbi_lte_active_mask = modem->sbi_lte_active_mask;
	mc->sbi_lte_active_pos = modem->sbi_lte_active_pos;
	mc->sbi_cp_status_mask = modem->sbi_cp_status_mask;
	mc->sbi_cp_status_pos = modem->sbi_cp_status_pos;

	mc->sbi_pda_active_mask = modem->sbi_pda_active_mask;
	mc->sbi_pda_active_pos = modem->sbi_pda_active_pos;
	mc->sbi_tx_flowctl_mask = modem->sbi_tx_flowctl_mask;
	mc->sbi_tx_flowctl_pos = modem->sbi_tx_flowctl_pos;
	mc->sbi_ap_status_mask = modem->sbi_ap_status_mask;
	mc->sbi_ap_status_pos = modem->sbi_ap_status_pos;

	mc->sbi_uart_noti_mask = modem->sbi_uart_noti_mask;
	mc->sbi_uart_noti_pos = modem->sbi_uart_noti_pos;

	mc->sbi_crash_type_mask = modem->sbi_crash_type_mask;
	mc->sbi_crash_type_pos = modem->sbi_crash_type_pos;

	mc->sbi_ds_det_mask = modem->sbi_ds_det_mask;
	mc->sbi_ds_det_pos = modem->sbi_ds_det_pos;

#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	mc->sbi_lcd_status_mask = modem->sbi_lcd_status_mask;
	mc->sbi_lcd_status_pos = modem->sbi_lcd_status_pos;
	mc->int_lcd_status = mbx->int_ap2cp_lcd_status;
#endif
}

#if IS_ENABLED(CONFIG_EXYNOS_ITMON) || IS_ENABLED(CONFIG_EXYNOS_ITMON_V2)
static int cp_itmon_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct modem_ctl *mc = container_of(nb, struct modem_ctl, itmon_nb);
	struct itmon_notifier *itmon_data = nb_data;

	if (IS_ERR_OR_NULL(itmon_data))
		return NOTIFY_DONE;

	if (itmon_data->port &&
	    (strncmp("MODEM_D1", itmon_data->port, sizeof("MODEM_D1") - 1) == 0 ||
	     strncmp("MODEM_D2", itmon_data->port, sizeof("MODEM_D2") - 1) == 0) &&
	    itmon_data->errcode == 1) {	/* force cp crash when decode error */
		mif_info("CP itmon notifier: cp crash request complete\n");
		modem_force_crash_exit(mc);
	}

	return NOTIFY_DONE;
}
#endif

#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
static int s5000ap_lcd_notifier(struct notifier_block *notifier,
		unsigned long event, void *v)
{
	struct modem_ctl *mc =
		container_of(notifier, struct modem_ctl, lcd_notifier);
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;

	switch (event) {
	case LCD_OFF:
		mif_info("LCD_OFF Notification\n");
		modem_ctrl_set_kerneltime(mc);
		update_ctrl_msg(&mld->ap2cp_united_status, 0,
				mc->sbi_lcd_status_mask,
				mc->sbi_lcd_status_pos);
		cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mc->int_lcd_status);
		break;

	case LCD_ON:
		mif_info("LCD_ON Notification\n");
		modem_ctrl_set_kerneltime(mc);
		update_ctrl_msg(&mld->ap2cp_united_status, 1,
				mc->sbi_lcd_status_mask,
				mc->sbi_lcd_status_pos);
		cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mc->int_lcd_status);
		break;

	default:
		mif_info("lcd_event %ld\n", event);
		break;
	}

	return NOTIFY_OK;
}
#endif /* CONFIG_CP_LCD_NOTIFIER */

#if IS_ENABLED(CONFIG_REINIT_VSS)
static int s5000ap_abox_call_state_notifier(struct notifier_block *nb,
					    unsigned long action, void *nb_data)
{
	struct modem_ctl *mc = container_of(nb, struct modem_ctl, abox_call_state_nb);

	mif_info("call event = %lu\n", action);

	switch (action) {
	case ABOX_CALL_EVENT_VSS_STOPPED:
		mif_info("vss stopped\n");
		complete_all(&mc->vss_stop);
		break;
	default:
		mif_err("undefined call event = %lu\n", action);
		break;
	}

	return NOTIFY_DONE;
}
#endif

int s5000ap_init_modemctl_device(struct modem_ctl *mc, struct modem_data *pdata)
{
	struct platform_device *pdev = to_platform_device(mc->dev);
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	unsigned int irq_num;
	unsigned long flags = IRQF_NO_SUSPEND | IRQF_NO_THREAD | IRQF_ONESHOT;

	mif_info("+++\n");

	/* To notify AP crash status to CP */
	g_mc = mc;

	s5000ap_get_ops(mc);
	s5000ap_get_pdata(mc, pdata);
	dev_set_drvdata(mc->dev, mc);

	/* Register CP_WDT */
	irq_num = platform_get_irq(pdev, 0);
	mif_init_irq(&mc->irq_cp_wdt, irq_num, "cp_wdt", flags);
	ret = mif_request_irq(&mc->irq_cp_wdt, cp_wdt_handler, mc);
	if (ret) {
		mif_err("Failed to request_irq with(%d)", ret);
		return ret;
	}
	/* CP_WDT interrupt must be enabled only after CP booting */
	mif_disable_irq(&mc->irq_cp_wdt);

	/* Register LTE_ACTIVE mailbox interrupt */
	ret = cp_mbox_register_handler(CP_MBOX_IRQ_IDX_0, mc->irq_phone_active,
			cp_active_handler, mc);
	if (ret) {
		mif_err("Failed to cp_mbox_register_handler %u with(%d)",
				mc->irq_phone_active, ret);
		return ret;
	}

	init_completion(&mc->init_cmpl);
	init_completion(&mc->off_cmpl);
#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	init_completion(&mc->clatinfo_ack);
#endif

	/* AP2CP_CFG */
	mif_dt_read_u32_noerr(np, "ap2cp_cfg_addr", mc->ap2cp_cfg_addr);
	if (mc->ap2cp_cfg_addr) {
		mif_info("AP2CP_CFG:0x%08x\n", mc->ap2cp_cfg_addr);
		mc->ap2cp_cfg_ioaddr = devm_ioremap(mc->dev, mc->ap2cp_cfg_addr, SZ_64);
		if (mc->ap2cp_cfg_ioaddr == NULL) {
			mif_err("%s: AP2CP_CFG ioremap failed.\n", __func__);
			return -EACCES;
		}
	}

	/* check s2d request handling support */
	mif_dt_read_u32_noerr(np, "s2d_req_handle_support", mc->s2d_req_handle_support);
	if (mc->s2d_req_handle_support) {
		mif_dt_read_u32(np, "s2d_req_pmu_reg", mc->s2d_req_pmu_reg);
		mif_dt_read_u32(np, "s2d_req_reg_mask", mc->s2d_req_reg_mask);
	}

#if !IS_ENABLED(CONFIG_GPIO_DS_DETECT)
	if (sysfs_create_group(&pdev->dev.kobj, &sim_group))
		mif_err("failed to create sysfs node related sim\n");
#endif

#if IS_ENABLED(CONFIG_EXYNOS_ITMON) || IS_ENABLED(CONFIG_EXYNOS_ITMON_V2)
	mc->itmon_nb.notifier_call = cp_itmon_notifier;
	itmon_notifier_chain_register(&mc->itmon_nb);
#endif

	/* Register PM notifier_call */
	mc->pm_notifier.notifier_call = s5000_pm_notifier;
	ret = register_pm_notifier(&mc->pm_notifier);
	if (ret) {
		mif_err("failed to register PM notifier_call\n");
		return ret;
	}

#if IS_ENABLED(CONFIG_REINIT_VSS)
	init_completion(&mc->vss_stop);

	mc->abox_call_state_nb.notifier_call = s5000ap_abox_call_state_notifier;
	register_abox_call_event_notifier(&mc->abox_call_state_nb);
#endif

	mif_info("---\n");
	return 0;
}

