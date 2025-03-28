/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung SoC MIPI-DSIM driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/pm_runtime.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/module.h>
#include <video/mipi_display.h>
#if IS_ENABLED(CONFIG_CAL_IF)
#include <soc/samsung/cal-if.h>
#endif

#if defined(CONFIG_SOC_S5E5515) && IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
#include <dt-bindings/soc/samsung/s5e5515-devfreq.h>
#include <dt-bindings/clock/s5e5515.h>
#endif

#include <soc/samsung/exynos-devfreq.h>

#if defined(CONFIG_CPU_IDLE)
#include <soc/samsung/exynos-cpupm.h>
#endif

#include <linux/string.h>
#include <linux/of_reserved_mem.h>
#include "../../../../../mm/internal.h"

#include "decon.h"
#include "dsim.h"
#include "./cal_5515/regs-dsim.h"
#include "./panels/exynos_panel_drv.h"

#if IS_ENABLED(CONFIG_EXYNOS_DPU_TC_SYSFS_ITF)
#include "sysfs_error.h"
#endif

#if IS_ENABLED(CONFIG_USDM_PANEL)
#include "mcd_dsim.h"
#include "mcd_helper.h"
#endif

int dsim_log_level = 6;

struct dsim_device *dsim_drvdata[MAX_DSIM_CNT];
EXPORT_SYMBOL(dsim_drvdata);

static char *dsim_state_names[] = {
	"INIT",
	"ON",
	"DOZE",
	"ULPS",
	"DOZE_SUSPEND",
	"OFF",
};

static int dsim_runtime_suspend(struct device *dev);
static int dsim_runtime_resume(struct device *dev);

#ifdef CONFIG_DPHY_APB_CONTROL
static int dsim_dphy_apb_enable(struct dsim_device *dsim, u32 en)
{
/* cam power control
 * dphy apb is located in IS BLK
 * so IS BLK power should be turned on in order to access dphy apb SFR
 */
#ifdef CONFIG_EXYNOS_PD
	if (en) {
		pm_runtime_get_sync(&dsim->dphy_apb_pdev->dev);
		dsim_dbg("dphy apb enable\n");
	} else {
		pm_runtime_put_sync(&dsim->dphy_apb_pdev->dev);
		dsim_dbg("dphy apb disable\n");
	}
#endif

	return 0;
}
#endif

int dsim_call_panel_ops(struct dsim_device *dsim, u32 cmd, void *arg)
{
	struct v4l2_subdev *sd;

	if (IS_ERR_OR_NULL(dsim->panel)) {
		dsim_err("%s: panel ptr is NULL\n", __func__);
		return -ENOMEM;
	}

	sd = &dsim->panel->sd;
	return v4l2_subdev_call(sd, core, ioctl, cmd, arg);
}

static void dsim_dump(struct dsim_device *dsim, bool panel_dump)
{
	struct dsim_regs regs;

	dsim_info("=== DSIM SFR DUMP ===\n");

	dsim_to_regs_param(dsim, &regs);
	__dsim_dump(dsim->id, &regs);

	if (panel_dump)
	/* Show panel status */
		dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_DUMP, NULL);
}
#if IS_ENABLED(CONFIG_USDM_PANEL)
static void dsim_panel_dump(struct dsim_device *dsim)
{
	/* Show panel status */
	dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_DUMP, NULL);
}
#endif

static void dsim_long_data_wr(struct dsim_device *dsim, unsigned long d0, u32 d1)
{
	unsigned int data_cnt = 0, payload = 0;

	/* in case that data count is more then 4 */
	for (data_cnt = 0; data_cnt < d1; data_cnt += 4) {
		/*
		 * after sending 4bytes per one time,
		 * send remainder data less then 4.
		 */
		if ((d1 - data_cnt) < 4) {
			if ((d1 - data_cnt) == 3) {
				payload = *(u8 *)(d0 + data_cnt) |
				    (*(u8 *)(d0 + (data_cnt + 1))) << 8 |
					(*(u8 *)(d0 + (data_cnt + 2))) << 16;
			dsim_dbg("count = 3 payload = %x, %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)));
			} else if ((d1 - data_cnt) == 2) {
				payload = *(u8 *)(d0 + data_cnt) |
					(*(u8 *)(d0 + (data_cnt + 1))) << 8;
			dsim_dbg("count = 2 payload = %x, %x %x\n", payload,
				*(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)));
			} else if ((d1 - data_cnt) == 1) {
				payload = *(u8 *)(d0 + data_cnt);
			}

			dsim_reg_wr_tx_payload(dsim->id, payload);
		/* send 4bytes per one time. */
		} else {
			payload = *(u8 *)(d0 + data_cnt) |
				(*(u8 *)(d0 + (data_cnt + 1))) << 8 |
				(*(u8 *)(d0 + (data_cnt + 2))) << 16 |
				(*(u8 *)(d0 + (data_cnt + 3))) << 24;

			dsim_dbg("count = 4 payload = %x, %x %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)),
				*(u8 *)(d0 + (data_cnt + 3)));

			dsim_reg_wr_tx_payload(dsim->id, payload);
		}
	}
	dsim->pl_cnt += ALIGN(d1, 4);
}

static int dsim_wait_for_cmd_fifo_empty(struct dsim_device *dsim, bool must_wait)
{
	int ret = 0;
	struct dsim_regs regs;

	if (!must_wait) {
		/* timer is running, but already command is transferred */
		if (dsim_reg_header_fifo_is_empty(dsim->id))
			del_timer(&dsim->cmd_timer);

		dsim_dbg("%s Doesn't need to wait fifo_completion\n", __func__);
		return ret;
	} else {
		del_timer(&dsim->cmd_timer);
		dsim_dbg("%s Waiting for fifo_completion...\n", __func__);
	}

	if (!wait_for_completion_timeout(&dsim->ph_wr_comp, MIPI_WR_TIMEOUT)) {
		if (dsim_reg_header_fifo_is_empty(dsim->id)) {
			reinit_completion(&dsim->ph_wr_comp);
			dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
			return 0;
		}
		ret = -ETIMEDOUT;
	}

	if (IS_DSIM_ON_STATE(dsim) && (ret == -ETIMEDOUT)) {
		dsim_err("%s have timed out\n", __func__);
		dsim_to_regs_param(dsim, &regs);
		__dsim_dump(dsim->id, &regs);
	}
	return ret;
}

/* wait for until SFR fifo is empty */
int dsim_wait_for_cmd_done(struct dsim_device *dsim)
{
	int ret = 0;
	/* FIXME: hiber only support for DECON0 */
	struct decon_device *decon = get_decon_drvdata(0);

	decon_hiber_block_exit(decon);

	mutex_lock(&dsim->cmd_lock);
	ret = dsim_wait_for_cmd_fifo_empty(dsim, true);
	mutex_unlock(&dsim->cmd_lock);

	decon_hiber_unblock(decon);

	return ret;
}

static bool dsim_is_writable_pl_fifo_status(struct dsim_device *dsim, u32 word_cnt)
{
	if ((DSIM_PL_FIFO_THRESHOLD - dsim->pl_cnt) > word_cnt)
		return true;
	else
		return false;
}

static bool dsim_is_fifo_empty_status(struct dsim_device *dsim)
{
	if (dsim_reg_header_fifo_is_empty(dsim->id) && dsim_reg_payload_fifo_is_empty(dsim->id)) {
		dsim->pl_cnt = 0;
		return true;
	} else
		return false;
}

int dsim_check_ph_threshold(struct dsim_device *dsim, u32 cmd_cnt)
{
	int cnt = 5000;
	u32 available = 0;

	available = dsim_reg_is_writable_ph_fifo_state(dsim->id, cmd_cnt);

	/* Wait FIFO empty status during 50ms */
	if (!available) {
		do {
			if (dsim_reg_header_fifo_is_empty(dsim->id))
				break;
			udelay(10);
			cnt--;
		} while (cnt);
	}
	return cnt;
}

int dsim_check_linecount(struct dsim_device *dsim)
{
	int cnt = 5000;
	bool fifo_empty = 0;
	int line_cnt = 0;

	dsim->line_cnt = dsim_reg_get_linecount(dsim->id, dsim->panel->lcd_info.mode);
	if (dsim->line_cnt == 0) {
		do {
			fifo_empty = dsim_is_fifo_empty_status(dsim);
			line_cnt = dsim_reg_get_linecount(dsim->id, dsim->panel->lcd_info.mode);
			if (fifo_empty || line_cnt)
				break;
			udelay(10);
			cnt--;
		} while (cnt);
	}
	return cnt;
}

int dsim_check_pl_threshold(struct dsim_device *dsim, u32 d1)
{
	int cnt = 5000;

	if (!dsim_is_writable_pl_fifo_status(dsim, d1)) {
		do {
			if (dsim_reg_payload_fifo_is_empty(dsim->id)) {
				dsim->pl_cnt = 0;
				break;
			}
			udelay(10);
			cnt--;
		} while (cnt);
	}

	return cnt;
}

#if IS_ENABLED(CONFIG_USDM_PANEL)

static void dsim_wr_payload(struct dsim_device *dsim, unsigned char *buf, u32 size)
{
	unsigned int data_cnt = 0;

	for (data_cnt = 0; data_cnt < size; data_cnt += 4) {
		dsim_reg_wr_tx_payload(dsim->id, *((unsigned int *)(buf + data_cnt)));
	}
	dsim->pl_cnt += size;
}
int __mockable dsim_sr_write_data(struct dsim_device *dsim, const u8 *cmd, u32 size, u32 align)
{
	int cnt;
	u8 c_start = 0, c_next = 0;
	/* TODO: 512 NEED TO CHANGE AS DSIM_FIFO_SIZE */
	u8 cmdbuf[2048];
	int tx_size, ret = 0, len = 0;
	int remained = size;


	mutex_lock(&dsim->cmd_lock);
	if (!IS_DSIM_ON_STATE(dsim)) {
		dsim_err("%s dsim%d not ready (%s)\n",
				__func__, dsim->id, dsim_state_names[dsim->state]);
		ret = -EINVAL;
		goto err_exit;
	}

	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);

	/* Check available status of PH FIFO before writing command */
	if (!dsim_check_ph_threshold(dsim, 1)) {
		ret = -EINVAL;
		dsim_err("ID(%d): DSIM cmd wr timeout @ don't available ph 0x%x\n",
			dsim->id, cmd[0]);
		goto err_exit;
	}

	/* Check linecount value for seperating idle and active range */
	if (!dsim_check_linecount(dsim)) {
		ret = -EINVAL;
		dsim_err("ID(%d): DSIM cmd wr timeout @ line count '0' pl_cnt = %d\n",
			dsim->id, dsim->pl_cnt);
		goto err_exit;
	}

	dsim_info("%s : size : %d align: %d\n", __func__, size, align);

	c_start = MIPI_DCS_WRITE_SIDE_RAM_START;
	c_next = MIPI_DCS_WRITE_SIDE_RAM_CONTINUE;

	do {
		cmdbuf[0] = (size == remained) ? c_start : c_next;
		tx_size = min(remained, 2047);

		if ((tx_size % align) > 0) {
			if (tx_size > align) {
				tx_size -= (tx_size % align);
			} else {
				dsim_warn("%s: byte align mismatch! data %d align %d\n",
					__func__, tx_size, align);
			}
		}

		memcpy(cmdbuf + 1, cmd + len, tx_size);

		dsim_reg_enable_packetgo(dsim->id, true);
		//decon_systrace(get_decon_drvdata(0), 'C', "mafpc", 1);
		dsim_wr_payload(dsim, cmdbuf, tx_size + 1);
		//decon_systrace(get_decon_drvdata(0), 'C', "mafpc", 0);
		dsim_reg_wr_tx_header(dsim->id, MIPI_DSI_DCS_LONG_WRITE, (tx_size + 1) & 0xff,
				((tx_size + 1) & 0xff00) >> 8, false);
		dsim_reg_enable_packetgo(dsim->id, false);

		len += tx_size;
		remained -= tx_size;

		cnt = 5000;
		do {
			if (dsim_is_fifo_empty_status(dsim))
				break;
			usleep_range(10, 11);
		} while (cnt--);

		if (!cnt) {
			dsim_err("ID(%d): DSIM command(%x) fail\n", dsim->id, cmd[0]);
			ret = -EINVAL;
		}
	} while (remained > 0);

err_exit:
	mutex_unlock(&dsim->cmd_lock);
	return ret;
}

#endif

int dsim_cal_pl_sum(struct exynos_dsim_cmd set_cmd[], int cmd_cnt, struct exynos_dsim_cmd_set *set)
{
	int i;
	int pl_size;
	int pl_sum_total = 0;
	int pl_sum = 0;
	struct exynos_dsim_cmd *cmd;

	set->cnt = 1;
	set->index[0] = cmd_cnt - 1;

	for (i = 0; i < cmd_cnt; i++) {
		cmd = &set_cmd[i];

		switch (cmd->type) {
			/* long packet types of packet types for command. */
			case MIPI_DSI_GENERIC_LONG_WRITE:
			case MIPI_DSI_DCS_LONG_WRITE:
			case MIPI_DSI_DSC_PPS:
				if (cmd->data_len > DSIM_PL_FIFO_THRESHOLD) {
					dsim_err("Don't support for pl size exceeding %d\n",
							DSIM_PL_FIFO_THRESHOLD);
					return -EINVAL;
				}
				pl_size = ALIGN(cmd->data_len, 4);
				pl_sum += pl_size;
				pl_sum_total += pl_size;
				if (pl_sum > DSIM_PL_FIFO_THRESHOLD) {
					set->index[set->cnt-1] = i - 1;
					set->cnt++;
					pl_sum = ALIGN(cmd->data_len, 4);
				}
				break;
		}
		dsim_dbg("pl_sum_total : %d\n", pl_sum_total);
	}

	return pl_sum_total;
}

int dsim_write_cmd_set(struct dsim_device *dsim, struct exynos_dsim_cmd cmd_list[],
		int cmd_cnt, bool wait_vsync)
{
	int i, j = 0;
	int ret = 0;
	int cnt = 5000;
	int pl_sum;
	struct decon_device *decon = get_decon_drvdata(dsim->id);
	struct exynos_dsim_cmd *cmd;
	struct exynos_dsim_cmd_set set;

	decon_hiber_block_exit(decon);
	mutex_lock(&dsim->cmd_lock);

	if (!IS_DSIM_ON_STATE(dsim)) {
		dsim_warn("DSIM is not ready. state(%d)\n", dsim->state);
		ret = -EINVAL;
		goto err_exit;
	}

	/* check PH available */
	if (!dsim_check_ph_threshold(dsim, cmd_cnt)) {
		ret = -EINVAL;
		dsim_err("DSIM_%d cmd wr timeout @ don't available ph\n", dsim->id);
		goto err_exit;
	}

	/* check PL available */
	pl_sum = dsim_cal_pl_sum(cmd_list, cmd_cnt, &set);
	if (!dsim_check_pl_threshold(dsim, pl_sum)) {
		ret = -EINVAL;
		dsim_err("DSIM don't available pl, pl_cnt @ fifo : %d, pl_sum : %d",
				dsim->pl_cnt, pl_sum);
		goto err_exit;
	}
	/* check line cnt value */
	if (!dsim_check_linecount(dsim)) {
		ret = -EINVAL;
		dsim_err("DSIM cmd wr timeout @ line count '0', pl_cnt @ = %d\n", dsim->pl_cnt);
		goto err_exit;
	}

	for (i = 0; i < set.cnt; i++) {

		/* packet go enable */
		dsim_reg_enable_packetgo(dsim->id, 1);

		for (; j < cmd_cnt; j++) {
			cmd = &cmd_list[j];
			if (!cmd->data_len)
				break;

			switch (cmd->type) {
			/* short packet types of packet types for command. */
			case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
			case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
			case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
			case MIPI_DSI_DCS_SHORT_WRITE:
			case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
			case MIPI_DSI_DSC_PRA:
			case MIPI_DSI_COLOR_MODE_OFF:
			case MIPI_DSI_COLOR_MODE_ON:
			case MIPI_DSI_SHUTDOWN_PERIPHERAL:
			case MIPI_DSI_TURN_ON_PERIPHERAL:
				if (cmd->data_len == 1)
					dsim_reg_wr_tx_header(dsim->id,
						cmd->type, cmd->data_buf[0],
						0, false);
				else
					dsim_reg_wr_tx_header(dsim->id,
						cmd->type, cmd->data_buf[0],
						cmd->data_buf[1], false);
				break;

				/* long packet types of packet types for command. */
			case MIPI_DSI_GENERIC_LONG_WRITE:
			case MIPI_DSI_DCS_LONG_WRITE:
			case MIPI_DSI_DSC_PPS:
				dsim_long_data_wr(dsim, (unsigned long)cmd->data_buf, cmd->data_len);
				dsim_reg_wr_tx_header(dsim->id, cmd->type, cmd->data_len & 0xff,
						(cmd->data_len & 0xff00) >> 8, false);
				break;

			default:
					dsim_info("data id %x is not supported.\n", cmd->type);
					ret = -EINVAL;
			}

			if (j == set.index[i])
				break;
		}

		if (set.cnt == 1) {
			if(wait_vsync)
				decon_wait_for_vsync(decon, VSYNC_TIMEOUT_MSEC);
		}
		/* set packet go ready*/
		dsim_reg_set_packetgo_ready(dsim->id);

		do {
			if (dsim_is_fifo_empty_status(dsim))
				break;
			udelay(10);
		} while (cnt--);
		if (!cnt) {
			dsim_err("DSIM command set fail, cmd_cnt : %d\n", cmd_cnt);
			ret = -EINVAL;
			goto err_exit;
		}
	}
	dsim_reg_enable_packetgo(dsim->id, 0);

err_exit:
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);

	return ret;

}

int dsim_write_data(struct dsim_device *dsim, u32 id, unsigned long d0, u32 d1, bool wait_empty)
{
	int ret = 0;
	struct decon_device *decon = get_decon_drvdata(0);
	int cnt = 5000; /* for wating empty status during 50ms */

	decon_hiber_block_exit(decon);

	mutex_lock(&dsim->cmd_lock);
	if (!IS_DSIM_ON_STATE(dsim)) {
		dsim_err("DSIM is not ready. state(%d)\n", dsim->state);
		ret = -EINVAL;
		goto err_exit;
	}

	reinit_completion(&dsim->ph_wr_comp);
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);

	/* Check available status of PH FIFO before writing command */
	if (!dsim_check_ph_threshold(dsim, 1)) {
		ret = -EINVAL;
		dsim_err("ID(%d): DSIM cmd wr timeout @ don't available ph 0x%lx\n", id, d0);
		goto err_exit;
	}

	/* Check linecount value for seperating idle and active range */
	if (!dsim_check_linecount(dsim)) {
		ret = -EINVAL;
		dsim_err("ID(%d): DSIM cmd wr timeout @ line count '0' 0x%lx, pl_cnt = %d\n", id, d0, dsim->pl_cnt);
		goto err_exit;
	}

	/* Run write-fail dectector */
	mod_timer(&dsim->cmd_timer, jiffies + MIPI_WR_TIMEOUT);

	switch (id) {
	/* short packet types of packet types for command. */
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
	case MIPI_DSI_DSC_PRA:
	case MIPI_DSI_COLOR_MODE_OFF:
	case MIPI_DSI_COLOR_MODE_ON:
	case MIPI_DSI_SHUTDOWN_PERIPHERAL:
	case MIPI_DSI_TURN_ON_PERIPHERAL:
		/* Enable packet go for blocking command transfer */
		dsim_reg_enable_packetgo(dsim->id, true);
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, false);
		break;

	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
	case MIPI_DSI_DCS_READ:
		/* Enable packet go for blocking command transfer */
		dsim_reg_enable_packetgo(dsim->id, true);
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, true);
		break;

	/* long packet types of packet types for command. */
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_DSC_PPS:
		if (d1 > DSIM_PL_FIFO_THRESHOLD) {
			dsim_err("Don't support payload size that exceeds 2048byte\n");
			ret = -EINVAL;
			goto err_exit;
		}
		if (!dsim_check_pl_threshold(dsim, ALIGN(d1, 4))) {
			ret = -EINVAL;
			dsim_err("ID(%d): DSIM don't available pl 0x%lx\n, pl_cnt : %d, wc : %d",
					id, d0, dsim->pl_cnt, d1);
			goto err_exit;
		}

		/* Enable packet go for blocking command transfer */
		dsim_reg_enable_packetgo(dsim->id, true);
		dsim_long_data_wr(dsim, d0, d1);
		dsim_reg_wr_tx_header(dsim->id, id, d1 & 0xff,
				(d1 & 0xff00) >> 8, false);
		break;

	default:
		dsim_info("data id %x is not supported.\n", id);
		ret = -EINVAL;
	}

	dsim_reg_enable_packetgo(dsim->id, false);

	if (wait_empty) {
		do {
			if (dsim_is_fifo_empty_status(dsim))
				break;
			udelay(10);
		} while (cnt--);

		if (!cnt) {
			dsim_err("ID(%d): DSIM command(%lx) fail\n", id, d0);
			ret = -EINVAL;
		}
	}

err_exit:
	DPU_EVENT_LOG_CMD(&dsim->sd, id, d0, d1);
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);

	return ret;
}

int dsim_read_data(struct dsim_device *dsim, u32 id, u32 addr, u32 cnt, u8 *buf)
{
	u32 rx_fifo, rx_size = 0;
	int i, j, ret = 0;
	u32 rx_fifo_depth = DSIM_RX_FIFO_MAX_DEPTH;
	struct decon_device *decon = get_decon_drvdata(0);
	struct dsim_regs regs;

	decon_hiber_block_exit(decon);

	if (IS_DSIM_OFF_STATE(dsim)) {
		dsim_err("DSIM is not ready. state(%d)\n", dsim->state);
		decon_hiber_unblock(decon);
		return -EINVAL;
	}
#ifdef CONFIG_READ_BY_LPDT
	dsim_reg_set_cmd_transfer_mode(dsim->id, 1);
#endif
	reinit_completion(&dsim->rd_comp);

	/* Init RX FIFO before read and clear DSIM_INTSRC */
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_RX_DATA_DONE);

	/* Set the maximum packet size returned */
	ret = dsim_write_data(dsim,
			MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, cnt, 0, false);
	if (ret < 0)
		dsim_err("fail to write MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE command.\n");

	/* Read request */
	ret = dsim_write_data(dsim, id, addr, 0, true);
	if (ret < 0)
		dsim_err("fail to write 0x%02x command.\n", id);

	dsim_wait_for_cmd_done(dsim);
#ifdef CONFIG_READ_BY_LPDT
	dsim_reg_set_cmd_transfer_mode(dsim->id, 0);
#endif

	if (!wait_for_completion_timeout(&dsim->rd_comp, MIPI_RD_TIMEOUT)) {
		dsim_err("MIPI DSIM read Timeout!\n");
		if (dsim_reg_get_datalane_status(dsim->id) == DSIM_DATALANE_STATUS_BTA) {
			if (decon_reg_get_run_status(dsim->id)) {
				dsim_reset_panel(dsim);
				dpu_hw_recovery_process(decon);
			} else {
				dsim_reset_panel(dsim);
				dsim_reg_recovery_process(dsim);
			}
		} else
			dsim_err("datalane status is %d\n", dsim_reg_get_datalane_status(dsim->id));

		return -ETIMEDOUT;
	}

	mutex_lock(&dsim->cmd_lock);
	DPU_EVENT_LOG_CMD(&dsim->sd, id, (char)addr, 0);

	do {
		rx_fifo = dsim_reg_get_rx_fifo(dsim->id);

		/* Parse the RX packet data types */
		switch (rx_fifo & 0xff) {
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			ret = dsim_reg_rx_err_handler(dsim->id, rx_fifo);
			if (ret < 0) {
				dsim_to_regs_param(dsim, &regs);
				__dsim_dump(dsim->id, &regs);
				goto exit;
			}
			break;
		case MIPI_DSI_RX_END_OF_TRANSMISSION:
			dsim_dbg("EoTp was received from LCD module.\n");
			break;
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
			dsim_dbg("1byte Short Packet was received from LCD\n");
			buf[0] = (rx_fifo >> 8) & 0xff;
			rx_size = 1;
			break;
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
			dsim_dbg("2bytes Short Packet was received from LCD\n");
			for (i = 0; i < 2; i++)
				buf[i] = (rx_fifo >> (8 + i * 8)) & 0xff;
			rx_size = 2;
			break;
		case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
			dsim_dbg("Long Packet was received from LCD module.\n");
			rx_size = (rx_fifo & 0x00ffff00) >> 8;
			dsim_dbg("rx fifo : %8x, response : %x, rx_size : %d\n",
					rx_fifo, rx_fifo & 0xff, rx_size);
			/* Read data from RX packet payload */
			for (i = 0; i < rx_size >> 2; i++) {
				rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
				for (j = 0; j < 4; j++)
					buf[(i*4)+j] = (u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			if (rx_size % 4) {
				rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
				for (j = 0; j < rx_size % 4; j++)
					buf[4 * i + j] =
						(u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			break;
		default:
			dsim_err("Packet format is invaild.\n");
			dsim_to_regs_param(dsim, &regs);
			__dsim_dump(dsim->id, &regs);
			ret = -EBUSY;
			goto exit;
		}
	} while (!dsim_reg_rx_fifo_is_empty(dsim->id) && --rx_fifo_depth);

	ret = rx_size;
	if (!rx_fifo_depth) {
		dsim_err("Check DPHY values about HS clk.\n");
		dsim_to_regs_param(dsim, &regs);
		__dsim_dump(dsim->id, &regs);
		ret = -EBUSY;
	}
exit:
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);

	return ret;
}

#if IS_ENABLED(CONFIG_USDM_PANEL)

static void dsim_write_timeout_fn(struct work_struct *work)
{
	struct dsim_device *dsim =
		container_of(work, struct dsim_device, wr_timeout_work);
	struct dsim_regs regs;
	struct decon_device *decon = get_decon_drvdata(0);

	dsim_dbg("%s +\n", __func__);
	decon_hiber_block(decon);
	mutex_lock(&dsim->cmd_lock);
	if (IS_DSIM_OFF_STATE(dsim)) {
		dsim_err("%s dsim%d not ready (%s)\n",
				__func__, dsim->id, dsim_state_names[dsim->state]);
		goto exit;
	}

	/* If already FIFO empty even though the timer is no pending */
	if (dsim_reg_header_fifo_is_empty(dsim->id)) {
		reinit_completion(&dsim->ph_wr_comp);
		dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
		goto exit;
	}

	dsim_to_regs_param(dsim, &regs);
	__dsim_dump(dsim->id, &regs);

exit:
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);
	dsim_dbg("%s -\n", __func__);
	return;
}
#endif

static void dsim_cmd_fail_detector(struct timer_list *arg)
{
	struct dsim_device *dsim = from_timer(dsim, arg, cmd_timer);
	struct decon_device *decon = get_decon_drvdata(0);
	struct dsim_regs regs;

	decon_hiber_block(decon);

	dsim_dbg("%s +\n", __func__);
	if (IS_DSIM_OFF_STATE(dsim)) {
		dsim_err("%s: DSIM is not ready. state(%d)\n", __func__,
				dsim->state);
		goto exit;
	}

	/* If already FIFO empty even though the timer is no pending */
	if (!timer_pending(&dsim->cmd_timer)
			&& dsim_reg_header_fifo_is_empty(dsim->id)) {
		reinit_completion(&dsim->ph_wr_comp);
		dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
		goto exit;
	}

#if IS_ENABLED(CONFIG_USDM_PANEL)
	queue_work(dsim->wq, &dsim->wr_timeout_work);
#endif

	dsim_to_regs_param(dsim, &regs);
	__dsim_dump(dsim->id, &regs);

exit:
	decon_hiber_unblock(decon);
	dsim_dbg("%s -\n", __func__);
	return;
}

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
static void dsim_bts_print_info(struct bts_decon_info *info)
{
	int i;

	for (i = 0; i < BTS_DPP_MAX; ++i) {
		if (!info->dpp[i].used)
			continue;

		dsim_info("\t\tDPP[%d] b(%d) s(%d %d) d(%d %d %d %d) r(%d)\n",
				i, info->dpp[i].bpp,
				info->dpp[i].src_w, info->dpp[i].src_h,
				info->dpp[i].dst.x1, info->dpp[i].dst.x2,
				info->dpp[i].dst.y1, info->dpp[i].dst.y2,
				info->dpp[i].rotation);
	}
}
#endif

static void dsim_underrun_info(struct dsim_device *dsim)
{
#if IS_ENABLED(CONFIG_EXYNOS_BTS)
	struct decon_device *decon = get_decon_drvdata(0);
	int i, decon_cnt;

#if IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ) && (LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0))
	dsim_info("\tMIF(%lu), INT(%lu), DISP(%lu)\n",
				cal_dfs_get_rate(ACPM_DVFS_MIF),
				cal_dfs_get_rate(ACPM_DVFS_INT),
				cal_dfs_get_rate(ACPM_DVFS_DISP));
#elif IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0))
	dsim_info("\tMIF(%lu), INT(%lu), DISP(%lu)\n",
			exynos_devfreq_get_domain_freq(DEVFREQ_MIF),
			exynos_devfreq_get_domain_freq(DEVFREQ_INT),
			exynos_devfreq_get_domain_freq(DEVFREQ_DISP));
#endif

	if (!decon)
		return;

	decon_cnt = get_decon_drvdata(0)->dt.decon_cnt;
	for (i = 0; i < decon_cnt; ++i) {
		decon = get_decon_drvdata(i);

		if (decon) {
			dsim_info("\tDECON%d: bw(%u %u), disp(%u %u), p(%u)\n",
					decon->id,
					decon->bts.prev_total_bw,
					decon->bts.total_bw,
					decon->bts.prev_max_disp_freq,
					decon->bts.max_disp_freq,
					decon->bts.peak);
			dsim_bts_print_info(&decon->bts.bts_info);
		}
	}
#endif
}

static irqreturn_t dsim_irq_handler(int irq, void *dev_id)
{
	unsigned int int_src;
	struct dsim_device *dsim = dev_id;
	struct decon_device *decon = get_decon_drvdata(0);
#ifdef CONFIG_EXYNOS_PD
	int active;
#endif
	u32 line_cnt;

	spin_lock(&dsim->slock);

#ifdef CONFIG_EXYNOS_PD
	active = pm_runtime_active(dsim->dev);
	if (!active) {
		dsim_info("dsim power(%d), state(%d)\n", active, dsim->state);
		spin_unlock(&dsim->slock);
		return IRQ_HANDLED;
	}
#endif

	int_src = dsim_reg_get_int_and_clear(dsim->id);

#if IS_ENABLED(CONFIG_EXYNOS_DPU_TC_SYSFS_ITF)
	// DSIM_INTSRC_ERR_RX_ECC
	// DSIM_INTSRC_UNDER_RUN
	if (int_src & DSIM_INTSRC_UNDER_RUN)
		dsim->irq_err_state |= SYSFS_ERR_DSIM_UNDER_RUN;

	if (int_src & DSIM_INTSRC_ERR_RX_ECC)
		dsim->irq_err_state |= SYSFS_ERR_DSIM_ERR_RX_ECC;
#endif

	if (int_src & DSIM_INTSRC_SFR_PH_FIFO_EMPTY) {
		del_timer(&dsim->cmd_timer);
		complete(&dsim->ph_wr_comp);
		dsim_dbg("dsim%d PH_FIFO_EMPTY irq occurs\n", dsim->id);
	}
	if (int_src & DSIM_INTSRC_RX_DATA_DONE)
		complete(&dsim->rd_comp);
	if (int_src & DSIM_INTSRC_FRAME_DONE) {
		dsim->continuous_underrun_cnt = 0;
		dsim_dbg("dsim%d framedone irq occurs\n", dsim->id);
	}
	if (int_src & DSIM_INTSRC_ERR_RX_ECC)
		dsim_err("RX ECC Multibit error was detected!\n");

	if (int_src & DSIM_INTSRC_UNDER_RUN) {
		dsim->total_underrun_cnt++;
		dsim->continuous_underrun_cnt++;
		dsim_info("dsim%d underrun irq occurs total(%d) cont(%d)\n", dsim->id,
				dsim->total_underrun_cnt, dsim->continuous_underrun_cnt);
		dsim_underrun_info(dsim);

		if (dsim->panel->lcd_info.mode == DECON_VIDEO_MODE) {
			line_cnt = dsim_reg_get_linecount(dsim->id, dsim->panel->lcd_info.mode);
			dsim_info("dsim%d line_count: (%08x)\n", dsim->id, line_cnt);
			if (decon && dsim->continuous_underrun_max > 0 &&
					(dsim->continuous_underrun_cnt >= dsim->continuous_underrun_max)) {
				decon_dump(decon, false);
				BUG();
			}
		}
	}
	if (int_src & DSIM_INTSRC_VT_STATUS) {
		dsim_dbg("dsim%d vt_status(vsync) irq occurs\n", dsim->id);
		if (decon) {
			if (decon->vsync.wait.head.prev) {
				decon->vsync.timestamp = ktime_get();
				wake_up_interruptible_all(&decon->vsync.wait);
			}
		}
	}

	spin_unlock(&dsim->slock);

	return IRQ_HANDLED;
}

static int dsim_get_clocks(struct dsim_device *dsim)
{
	dsim->res.aclk = devm_clk_get(dsim->dev, "aclk");
	if (IS_ERR_OR_NULL(dsim->res.aclk)) {
		dsim_err("failed to get aclk\n");
		return PTR_ERR(dsim->res.aclk);
	}

	return 0;
}
#if IS_ENABLED(CONFIG_USDM_PANEL)
int dsim_reset_disable_panel(struct dsim_device *dsim)
{
	struct v4l2_subdev *sd;
	if (IS_ERR_OR_NULL(dsim->panel)) {
		dsim_err("%s: panel ptr is NULL\n", __func__);
		return -ENOMEM;
	}

	sd = &dsim->panel->sd;
	return v4l2_subdev_call(sd, core, ioctl, EXYNOS_PANEL_IOC_RESET_DISABLE, NULL);
}
#endif

int dsim_reset_panel(struct dsim_device *dsim)
{
	struct v4l2_subdev *sd;

	if (IS_ERR_OR_NULL(dsim->panel)) {
		dsim_err("%s: panel ptr is NULL\n", __func__);
		return -ENOMEM;
	}

	sd = &dsim->panel->sd;
	return v4l2_subdev_call(sd, core, ioctl, EXYNOS_PANEL_IOC_RESET, NULL);
}

int dsim_set_panel_power(struct dsim_device *dsim, bool on)
{
	struct v4l2_subdev *sd;
	int ret = 0;

	if (IS_ERR_OR_NULL(dsim->panel)) {
		dsim_err("%s: panel ptr is NULL\n", __func__);
		return -ENOMEM;
	}

	sd = &dsim->panel->sd;
	if (on)
		ret = v4l2_subdev_call(sd, core, ioctl, EXYNOS_PANEL_IOC_POWERON, NULL);
	else
		ret = v4l2_subdev_call(sd, core, ioctl, EXYNOS_PANEL_IOC_POWEROFF, NULL);

	return ret;
}

static char *rpm_status_name[] = {
	"RPM_ACTIVE",
	"RPM_RESUMING",
	"RPM_SUSPENDED",
	"RPM_SUSPENDING",
};

static void dsim_print_phy_info(struct dsim_device *dsim)
{
	dsim_info("[PHY] power_count(%d), disable_depth(%d), runtime_status(%s)\n",
			dsim->phy->power_count, dsim->phy->dev.power.disable_depth,
			rpm_status_name[dsim->phy->dev.power.runtime_status]);

	dsim_info("[PHY_EX] power_count(%d), disable_depth(%d), runtime_status(%s)\n",
			dsim->phy_ex->power_count, dsim->phy_ex->dev.power.disable_depth,
			rpm_status_name[dsim->phy_ex->dev.power.runtime_status]);
}

static int dsim_phy_power_on(struct dsim_device *dsim)
{
	int ret = 0;

	ret = phy_power_on(dsim->phy);
	if (ret < 0) {
		dsim_err("failed to enable phy(%d)\n", ret);
		goto err;
	}
	if (dsim->phy_ex) {
		ret = phy_power_on(dsim->phy_ex);
		if (ret < 0) {
			dsim_err("failed to enable extra phy(%d)\n", ret);
			goto err;
		}
	}

	return 0;

err:
	dsim_print_phy_info(dsim);
	return ret;
}

static int dsim_phy_power_off(struct dsim_device *dsim)
{
	int ret = 0;

	ret = phy_power_off(dsim->phy);
	if (ret < 0) {
		dsim_err("failed to enable phy(%d)\n", ret);
		goto err;
	}
	if (dsim->phy_ex) {
		ret = phy_power_off(dsim->phy_ex);
		if (ret < 0) {
			dsim_err("failed to enable extra phy(%d)\n", ret);
			goto err;
		}
	}

	return 0;

err:
	dsim_print_phy_info(dsim);
	return ret;
}

static int _dsim_enable(struct dsim_device *dsim, enum dsim_state state)
{
	bool panel_ctrl;

	if (IS_DSIM_ON_STATE(dsim)) {
		dsim_warn("%s dsim already on(%s)\n",
				__func__, dsim_state_names[dsim->state]);
		dsim->state = state;
		return 0;
	}

	dsim_dbg("%s %s +\n", __func__, dsim_state_names[dsim->state]);

#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif


	pm_runtime_get_sync(dsim->dev);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 1);
#endif

	/* DPHY power on : iso release */
	dsim_phy_power_on(dsim);

#if IS_ENABLED(CONFIG_USDM_PANEL)
	panel_ctrl = (state == DSIM_STATE_ON || state == DSIM_STATE_DOZE) ? true : false;
#else
	panel_ctrl = (((dsim->state == DSIM_STATE_OFF) || (dsim->state == DSIM_STATE_INIT)) &&
			((state == DSIM_STATE_ON) || (state == DSIM_STATE_DOZE))) ?
			 true : false;
#endif

	dsim_reg_init(dsim->id, &dsim->panel->lcd_info, &dsim->clks, panel_ctrl);
	dsim_reg_start(dsim->id);

	dsim->state = state;

	enable_irq(dsim->res.irq);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 0);
#endif

	return 0;
}

static int dsim_read_panel_id(struct dsim_device *dsim, u32 *id);



static int dsim_enable(struct dsim_device *dsim)
{
	int ret;
	enum dsim_state prev_state = dsim->state;
	enum dsim_state next_state = DSIM_STATE_ON;

	if (prev_state == next_state) {
		dsim_warn("dsim-%d %s already %s state\n", dsim->id,
				__func__, dsim_state_names[dsim->state]);
		return 0;
	}

	dsim_info("dsim-%d %s +\n", dsim->id, __func__);
	ret = _dsim_enable(dsim, next_state);
	if (ret < 0) {
		dsim_err("dsim-%d failed to set %s (ret %d)\n",
				dsim->id, dsim_state_names[next_state], ret);
		goto out;
	}

	if (prev_state != DSIM_STATE_INIT) {
#if IS_ENABLED(CONFIG_USDM_PANEL)
		ret = dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_SLEEPOUT, NULL);
#else
		ret = dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_DISPLAYON, NULL);
#endif
		if (ret < 0) {
			dsim_err("dsim-%d failed to set %s (ret %d)\n",
					dsim->id, dsim_state_names[next_state], ret);
			goto out;
		}
	}
	dsim_info("dsim-%d %s - (state:%s -> %s)\n", dsim->id, __func__,
			dsim_state_names[prev_state],
			dsim_state_names[dsim->state]);

out:
	return ret;
}

static int dsim_doze(struct dsim_device *dsim)
{
	int ret;
	enum dsim_state prev_state = dsim->state;
	enum dsim_state next_state = DSIM_STATE_DOZE;

	if (prev_state == next_state) {
		dsim_warn("dsim-%d %s already %s state\n", dsim->id,
				__func__, dsim_state_names[dsim->state]);
		return 0;
	}

	dsim_info("dsim-%d %s +\n", dsim->id, __func__);
	ret = _dsim_enable(dsim, next_state);
	if (ret < 0) {
		dsim_err("dsim-%d failed to set %s (ret %d)\n",
				dsim->id, dsim_state_names[next_state], ret);
		goto out;
	}
#if IS_ENABLED(CONFIG_USDM_PANEL)
	if (prev_state != DSIM_STATE_INIT) {
		ret = dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_DOZE, NULL);
		if (ret < 0) {
			dsim_err("dsim-%d failed to set %s (ret %d)\n",
					dsim->id, dsim_state_names[next_state], ret);
			goto out;
		}
	}
#else
	if (prev_state == DSIM_STATE_OFF)
		dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_DOZE, NULL);
#endif

	dsim_info("dsim-%d %s - (state:%s -> %s)\n", dsim->id, __func__,
			dsim_state_names[prev_state],
			dsim_state_names[dsim->state]);

out:
	return ret;
}

/* This is needed only before lpd is started , don't use it for other purpose!!! */
/* This is added to wait supplier device of dsim-dev to complete suspend, like 13820000.sysmmu */
/* 13820000.sysmmu is asyncly suspended during pm_runtime_put_sync(dsim->dev) flow, */
/* even it is using same power domain, pd_dpu will be shutdown in lpd */
/* no need to add device_links_read_lock protection due to its supplier device is staticly bound. */
static int dsim_wait_suppliers_complete_suspend(struct device *dev)
{
	struct device_link *link;
	int result = 1;
	int wait_count = 0;   //idx,

	dsim_dbg("%s +\n", __func__);
	do {
		result = 0;

		list_for_each_entry_rcu(link, &dev->links.suppliers, c_node, 1) {
			if (link->supplier->power.runtime_status != RPM_SUSPENDED) {
				result = 1;
				break;
			}
		}

		if (result != 0) {
			msleep(20);
			wait_count++;
			if (wait_count > 100) {
				dsim_warn("%s , too many times =%d !\n",__func__,wait_count);
				break;
			}
		}
	} while ( result != 0);
	dsim_dbg("%s -\n", __func__);

	return result;
}

static int _dsim_disable(struct dsim_device *dsim, enum dsim_state state)
{
	struct dsim_regs regs;

	if (IS_DSIM_OFF_STATE(dsim)) {
		dsim_warn("%s dsim already off(%s)\n",
				__func__, dsim_state_names[dsim->state]);
		if (state == DSIM_STATE_OFF)
			dsim_set_panel_power(dsim, 0);
		dsim->state = state;
		return 0;
	}

	dsim_dbg("%s %s +\n", __func__, dsim_state_names[dsim->state]);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 1);
#endif

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	del_timer(&dsim->cmd_timer);
	dsim->state = state;
	mutex_unlock(&dsim->cmd_lock);

	if (dsim_reg_stop(dsim->id, dsim->data_lane) < 0) {
		dsim_to_regs_param(dsim, &regs);
		__dsim_dump(dsim->id, &regs);
	}
	disable_irq(dsim->res.irq);

	/* HACK */
	dsim_phy_power_off(dsim);

#if !IS_ENABLED(CONFIG_USDM_PANEL)
	if (state == DSIM_STATE_OFF)
		dsim_set_panel_power(dsim, 0);
#endif

	pm_runtime_put_sync(dsim->dev);

	// wait for supplier device to complete suspend only for doze suspend
	if (state == DSIM_STATE_DOZE_SUSPEND)
		dsim_wait_suppliers_complete_suspend(dsim->dev);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 0);
#endif

#if IS_ENABLED(CONFIG_USDM_PANEL)
	if (state == DSIM_STATE_OFF)
		dsim_set_panel_power(dsim, 0);
#endif

	dsim_dbg("%s %s -\n", __func__, dsim_state_names[dsim->state]);
#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	return 0;
}

static int dsim_disable(struct dsim_device *dsim)
{
	int ret;
	enum dsim_state prev_state = dsim->state;
	enum dsim_state next_state = DSIM_STATE_OFF;

	if (prev_state == next_state) {
		dsim_warn("dsim-%d %s already %s state\n", dsim->id,
				__func__, dsim_state_names[dsim->state]);
		return 0;
	}

	dsim_info("dsim-%d %s +\n", dsim->id, __func__);
	dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_SUSPEND, NULL);
	ret = _dsim_disable(dsim, next_state);
	if (ret < 0) {
		dsim_err("dsim-%d failed to set %s (ret %d)\n",
				dsim->id, dsim_state_names[next_state], ret);
		goto out;
	}
	dsim_info("dsim-%d %s - (state:%s -> %s)\n", dsim->id, __func__,
			dsim_state_names[prev_state],
			dsim_state_names[dsim->state]);

out:
	return ret;
}

static int dsim_doze_suspend(struct dsim_device *dsim)
{
	int ret;
	enum dsim_state prev_state = dsim->state;
	enum dsim_state next_state = DSIM_STATE_DOZE_SUSPEND;

	if (prev_state == next_state) {
		dsim_warn("dsim-%d %s already %s state\n", dsim->id,
				__func__, dsim_state_names[dsim->state]);
		return 0;
	}

	dsim_info("dsim-%d %s +\n", dsim->id, __func__);
	dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_DOZE_SUSPEND, NULL);
	ret = _dsim_disable(dsim, next_state);
	if (ret < 0) {
		dsim_err("dsim-%d failed to set %s (ret %d)\n",
				dsim->id, dsim_state_names[next_state], ret);
		goto out;
	}
	dsim_info("dsim-%d %s - (state:%s -> %s)\n", dsim->id, __func__,
			dsim_state_names[prev_state],
			dsim_state_names[dsim->state]);

out:
	return ret;
}

static int dsim_enter_ulps(struct dsim_device *dsim)
{
	int ret = 0;

	DPU_EVENT_START();
	dsim_dbg("%s +\n", __func__);

	if (!IS_DSIM_ON_STATE(dsim)) {
		ret = -EBUSY;
		goto err;
	}

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 1);
#endif

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	dsim->state = DSIM_STATE_ULPS;
	mutex_unlock(&dsim->cmd_lock);

	disable_irq(dsim->res.irq);
	ret = dsim_reg_stop_and_enter_ulps(dsim->id, dsim->panel->lcd_info.ddi_type,
			dsim->data_lane);

	dsim_phy_power_off(dsim);

	pm_runtime_put_sync(dsim->dev);
#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 1);
#endif

	DPU_EVENT_LOG(DPU_EVT_ENTER_ULPS, &dsim->sd, start);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 0);
#endif

err:
	dsim_dbg("%s -\n", __func__);
	return ret;
}

static int dsim_exit_ulps(struct dsim_device *dsim)
{
	int ret = 0;

	DPU_EVENT_START();
	dsim_dbg("%s +\n", __func__);

	if (dsim->state != DSIM_STATE_ULPS) {
		ret = -EBUSY;
		goto err;
	}
#if defined(CONFIG_CPU_IDLE)
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	pm_runtime_get_sync(dsim->dev);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 1);
#endif

	/* DPHY power on : iso release */
	dsim_phy_power_on(dsim);

	dsim_reg_init(dsim->id, &dsim->panel->lcd_info, &dsim->clks, false);
	ret = dsim_reg_exit_ulps_and_start(dsim->id, dsim->panel->lcd_info.ddi_type,
			dsim->data_lane);
	if (ret < 0)
		dsim_dump(dsim, false);

	enable_irq(dsim->res.irq);

	dsim->state = DSIM_STATE_ON;
	DPU_EVENT_LOG(DPU_EVT_EXIT_ULPS, &dsim->sd, start);

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim_dphy_apb_enable(dsim, 0);
#endif

err:
	dsim_dbg("%s -\n", __func__);

	return 0;
}

static int dsim_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dsim_device *dsim = container_of(sd, struct dsim_device, sd);

	if (enable)
		return dsim_enable(dsim);
	else
		return dsim_disable(dsim);
}

static int dsim_set_freq_hop(struct dsim_device *dsim, struct decon_freq_hop *freq)
{
#if defined(CONFIG_EXYNOS_FREQ_HOP)
	struct stdphy_pms *pms;

	if (!IS_DSIM_ON_STATE(dsim)) {
		dsim_err("%s: dsim%d is off state\n", __func__, dsim->id);
		return -EINVAL;
	}

	pms = &dsim->panel->lcd_info.dphy_pms;
	/* If target M value is 0, frequency hopping will be disabled */
	dsim_reg_set_dphy_freq_hopping(dsim->id, pms->p, freq->target_m,
			freq->target_k, (freq->target_m > 0) ? 1 : 0);
#endif

	return 0;
}

static int dsim_free_fb_resource(struct dsim_device *dsim)
{



	/* unreserve memory */
	of_reserved_mem_device_release(dsim->dev);

	/* update state */
	dsim->fb_handover.reserved = false;
	dsim->fb_handover.phys_addr = 0xdead;
	dsim->fb_handover.phys_size = 0;

	return 0;
}

static long dsim_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct dsim_device *dsim = container_of(sd, struct dsim_device, sd);
	int ret = 0;

	switch (cmd) {
	case DSIM_IOC_GET_LCD_INFO:
		v4l2_set_subdev_hostdata(sd, &dsim->panel->lcd_info);
		break;

	case DSIM_IOC_ENTER_ULPS:
		if ((unsigned long)arg)
			ret = dsim_enter_ulps(dsim);
		else
			ret = dsim_exit_ulps(dsim);
		break;

	case DSIM_IOC_DUMP:
		dsim_dump(dsim, *((bool *)arg));
		break;

#if IS_ENABLED(CONFIG_USDM_PANEL)
	case DSIM_IOC_PANEL_DUMP:
		dsim_panel_dump(dsim);
		break;
#endif

	case DSIM_IOC_GET_WCLK:
		v4l2_set_subdev_hostdata(sd, &dsim->clks.word_clk);
		break;

	case EXYNOS_DPU_GET_ACLK:
		return clk_get_rate(dsim->res.aclk);

	case DSIM_IOC_DOZE:
		ret = dsim_doze(dsim);
		break;

	case DSIM_IOC_DOZE_SUSPEND:
		ret = dsim_doze_suspend(dsim);
		break;

	case DSIM_IOC_SET_FREQ_HOP:
		ret = dsim_set_freq_hop(dsim, (struct decon_freq_hop *)arg);
		break;

	case DSIM_IOC_FREE_FB_RES:
		ret = dsim_free_fb_resource(dsim);
		break;

	case DSIM_IOC_RECOVERY_PROC:
		dsim_reg_recovery_process(dsim);
		break;

#if IS_ENABLED(CONFIG_USDM_PANEL)
	case DSIM_IOC_NOTIFY:
		dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_NOTIFY, arg);
		break;

	case DSIM_IOC_SET_ERROR_CB:
		if (arg == NULL) {
			dsim_err("%s invalid arg\n", __func__);
			ret = -EINVAL;
			break;
		}
		dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_SET_ERROR_CB, arg);
		break;
#endif

	default:
		dsim_err("unsupported ioctl");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_subdev_core_ops dsim_sd_core_ops = {
	.ioctl = dsim_ioctl,
};

static const struct v4l2_subdev_video_ops dsim_sd_video_ops = {
	.s_stream = dsim_s_stream,
};

static const struct v4l2_subdev_ops dsim_subdev_ops = {
	.core = &dsim_sd_core_ops,
	.video = &dsim_sd_video_ops,
};

static void dsim_init_subdev(struct dsim_device *dsim)
{
	struct v4l2_subdev *sd = &dsim->sd;

	v4l2_subdev_init(sd, &dsim_subdev_ops);
	sd->owner = THIS_MODULE;
	sd->grp_id = dsim->id;
	snprintf(sd->name, sizeof(sd->name), "%s.%d", "dsim-sd", dsim->id);
	v4l2_set_subdevdata(sd, dsim);
}

#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION) && defined(CONFIG_EXYNOS_READ_ESD_SOLUTION_TEST)
static ssize_t dsim_esd_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long cmd;
	struct dsim_device *dsim = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &cmd);
	if (ret)
		return ret;

	dsim->esd_test = cmd;

	return count;
}
static DEVICE_ATTR(esd_test, 0644, NULL, dsim_esd_test_store);

int dsim_create_esd_test_sysfs(struct dsim_device *dsim)
{
	int ret = 0;

	ret = device_create_file(dsim->dev, &dev_attr_esd_test);
	if (ret)
		dsim_err("failed to create command read & write sysfs\n");

	return ret;
}
#endif

static int dsim_cmd_sysfs_write(struct dsim_device *dsim, bool on)
{
	int ret = 0;

	if (on)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE,
			MIPI_DCS_SET_DISPLAY_ON, 0, false);
	else
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE,
			MIPI_DCS_SET_DISPLAY_OFF, 0, false);
	if (ret < 0)
		dsim_err("Failed to write test data!\n");
	else
		dsim_dbg("Succeeded to write test data!\n");

	return ret;
}

static int dsim_read_panel_id(struct dsim_device *dsim, u32 *id)
{
	int ret = 0;
	u8 buf[4];

	memset(buf, 0, sizeof(buf));

	/* dsim sends the request for the lcd id and gets it buffer */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
			MIPI_DCS_GET_DISPLAY_ID, DSIM_DDI_ID_LEN, buf);
	if (ret < 0) {
		dsim_err("failed to read panel id(%d)\n", ret);
		return ret;
	}

	memcpy(id, (unsigned int *)buf, sizeof(u32));
	dsim_info("suceeded to read panel id : 0x%08x\n", *id);

	return ret;
}

static int dsim_cmd_sysfs_read(struct dsim_device *dsim)
{
	u32 panel_id;

	return dsim_read_panel_id(dsim, &panel_id);
}

static ssize_t dsim_cmd_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t dsim_cmd_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long cmd;
	struct dsim_device *dsim = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &cmd);
	if (ret)
		return ret;

	switch (cmd) {
	case 1:
		ret = dsim_cmd_sysfs_read(dsim);
		dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_DUMP, NULL);
		if (ret)
			return ret;
		break;
	case 2:
		ret = dsim_cmd_sysfs_write(dsim, true);
		dsim_info("Dsim write command, display on!!\n");
		if (ret)
			return ret;
		break;
	case 3:
		ret = dsim_cmd_sysfs_write(dsim, false);
		dsim_info("Dsim write command, display off!!\n");
		if (ret)
			return ret;
		break;
	default :
		dsim_info("unsupportable command\n");
		break;
	}

	return count;
}
static DEVICE_ATTR(cmd_rw, 0644, dsim_cmd_sysfs_show, dsim_cmd_sysfs_store);

static ssize_t dsim_ddi_addr_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	int size = 0;
	int count;

	size = (ssize_t)sprintf(buf, "addr : 0x%02x   ", dsim->ddi_seq[0]);
	size = (ssize_t)sprintf(buf + size, "size : %d\n", dsim->ddi_seq_size);

	count = strlen(buf);
	return count;
}

static ssize_t dsim_ddi_addr_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	unsigned int res;
	int ret;

	char *cnt;
	char *addr;

	cnt = (char *)buf;
	addr = strsep(&cnt, " ");
	if (addr == NULL || cnt == NULL) {
		dsim_err("Usage : echo addr size > sysfs\n");
		goto end_func;
	}

	ret = kstrtoint(addr, 0, &res);
	if ((ret != 0) || (res > 255)) {
		dsim_err("Fail : addr(0x%x) value should be less than 0xFF\n", res);
		goto end_func;
	}
	dsim->ddi_seq[0] = (unsigned char)res;

	ret = kstrtoint(cnt, 0, &res);
	if (ret != 0)  {
		dsim_err("Fail : cnt wrong value\n");
		goto end_func;
	}

	if(res >= sizeof(dsim->ddi_seq) - 1) {
		dsim_err("Fail : cnt(0x%x) value should be less than 511\n", res);
		goto end_func;
	}

	dsim->ddi_seq_size = res;

	dsim_info("ddi_addr : 0x%x\n", dsim->ddi_seq[0]);
	dsim_info("ddi_seq_size : 0x%x\n", dsim->ddi_seq_size);

end_func:
	return count;
}
static DEVICE_ATTR(ddi_addr, 0600, dsim_ddi_addr_sysfs_show, dsim_ddi_addr_sysfs_store);

static ssize_t dsim_ddi_read_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	int ret = 0;
	int offset = 0;
	int i;
	int count;

	/* dsim read */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ, dsim->ddi_seq[0], dsim->ddi_seq_size, &dsim->ddi_seq[1]);

	if (ret < 0) {
		dsim_err("Failed to write test data!\n");
		count = 0;
		goto end_func;
	} else
		dsim_dbg("Succeeded to write test data!\n");

	/* print */
	for (i = 1; i <= dsim->ddi_seq_size; i++)
		offset += scnprintf(buf + offset, PAGE_SIZE - offset, "0x%02x ", dsim->ddi_seq[i]);

	offset += scnprintf(buf + offset, PAGE_SIZE - offset, "\n");
	count = strlen(buf);

end_func:
	return count;
}
static DEVICE_ATTR(ddi_read, 0400, dsim_ddi_read_sysfs_show, NULL);

static ssize_t dsim_ddi_write_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	char *start;
	char *find;
	char token[] = "\0\0\0\0\0\0\0\0\0\0";
	unsigned int num;
	unsigned int exit = 1;

	unsigned int val;
	int ret;
	int i = 0;

	start = (char *)buf;

	while (exit && i < sizeof(dsim->ddi_seq)) {
		/* parsing */
		find = strchr(start, ' ');
		if (find == NULL) {
			find = strchr(start, '\0');
			exit = 0;
		}

		num = find - start;

		if (sizeof(token) <= num)
		{
			dsim_err("Fail : size of input data exceeds the limit");
			goto end_func;
		}

		strncpy(token, start, num);
		token[num] = '\0';

		find++;
		start = find;

		/* convert str to number */
		if ((strncmp("0x", token, 2) == 0) || (strncmp("0X", token, 2) == 0))
			ret = kstrtouint(token+2, 16, &val);
		else
			ret = kstrtouint(token, 16, &val);

		if (ret != 0) {
			dsim_err("Fail : data(%d) wrong value (should 0 ~ 0xff)\n", (unsigned int)val);
			goto end_func;
		}

		if (val > 255) {
			dsim_err("Fail : data(%d) value should be less than 0xFF\n", (unsigned int)val);
			goto end_func;
		}

		dsim->ddi_seq[i] = (unsigned char)val;
		dsim_info("linmao test %d\n", (unsigned int)dsim->ddi_seq[i]); // for debug
		i++;
	}
	dsim->ddi_seq_size = i - 1; // for except addr

	/* dsim write */
	if (dsim->ddi_seq_size == 1)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE, dsim->ddi_seq[0], 0, false);
	else if (dsim->ddi_seq_size == 2)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE_PARAM, dsim->ddi_seq[0], dsim->ddi_seq[1], false);
	else
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_LONG_WRITE,
				(unsigned long)dsim->ddi_seq, dsim->ddi_seq_size + 1, false);

	if (ret < 0)
		dsim_err("Failed to write test data!\n");
	else
		dsim_dbg("Succeeded to write test data!\n");

end_func:
	return count;
}
static DEVICE_ATTR(ddi_write, 0200, NULL, dsim_ddi_write_sysfs_store);

static ssize_t dsim_underrun_max_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	int size = 0;

	size = (ssize_t)sprintf(buf, "%d\n", dsim->continuous_underrun_max);

	return size;
}

static ssize_t dsim_underrun_max_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	unsigned int cont_underrun_max;
	int ret;

	ret = kstrtouint(buf, 0, &cont_underrun_max);
	if (ret)
		return ret;

	dsim->continuous_underrun_max = cont_underrun_max;

	return count;
}

static DEVICE_ATTR(cont_underrun_max, 0600, dsim_underrun_max_sysfs_show, dsim_underrun_max_sysfs_store);

int dsim_create_cmd_rw_sysfs(struct dsim_device *dsim)
{
	int ret = 0;

	ret = device_create_file(dsim->dev, &dev_attr_cmd_rw);
	if (ret) {
		dsim_err("failed to create command read & write sysfs\n");
		goto error;
	}

	ret = device_create_file(dsim->dev, &dev_attr_ddi_addr);
	if (ret) {
		dsim_err("failed to create ddi_addr sysfs\n");
		goto error;
	}

	ret = device_create_file(dsim->dev, &dev_attr_ddi_read);
	if (ret) {
		dsim_err("failed to create ddi_read sysfs\n");
		goto error;
	}

	ret = device_create_file(dsim->dev, &dev_attr_ddi_write);
	if (ret) {
		dsim_err("failed to create ddi_write sysfs\n");
		goto error;
	}

	ret = device_create_file(dsim->dev, &dev_attr_cont_underrun_max);
	if (ret) {
		dsim_err("failed to create cont_underrun_max sysfs\n");
		goto error;
	}

error:

	return ret;
}

#if IS_ENABLED(CONFIG_EXYNOS_DPU_TC_SYSFS_ITF)
static ssize_t dsim_irq_err_sysfs_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);
	int size = 0;
	int count;

	size = (ssize_t)sprintf(buf, "0x%X", dsim->irq_err_state);
	dsim_info("DPP(%d) IRQ State : 0x%X\n", dsim->id, dsim->irq_err_state);

	count = strlen(buf);
	return count;
}

static ssize_t dsim_irq_err_sysfs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long cmd;
	struct dsim_device *dsim = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &cmd);
	if (ret)
		return ret;

	if (cmd == 0) {
		dsim_info("DSIM(%d) IRQ State: Clear cmd\n", dsim->id);
		dsim->irq_err_state = 0;
	}
	else {
		dsim_info("DSIM(%d) IRQ State: Unknown cmd = %d\n", dsim->id, cmd);
	}

	return count;
}
static DEVICE_ATTR(dsim_irq_err, 0600, dsim_irq_err_sysfs_show, dsim_irq_err_sysfs_store);

int dsim_create_irq_err_sysfs(struct dsim_device *dsim)
{
	int ret = 0;

	ret = device_create_file(dsim->dev, &dev_attr_dsim_irq_err);
	if (ret) {
		dsim_err("failed to create dsim irq err sysfs\n");
		goto error;
	}

	error:

	return ret;
}
#endif

static int dsim_parse_dt(struct dsim_device *dsim, struct device *dev)
{
	if (IS_ERR_OR_NULL(dev->of_node)) {
		dsim_err("no device tree information\n");
		return -EINVAL;
	}

	dsim->id = of_alias_get_id(dev->of_node, "dsim");
	dsim_info("dsim(%d) probe start..\n", dsim->id);

	dsim->phy = devm_phy_get(dev, "dsim_dphy");
	if (IS_ERR_OR_NULL(dsim->phy)) {
		dsim_err("failed to get phy\n");
		return PTR_ERR(dsim->phy);
	}

	dsim->phy_ex = devm_phy_get(dev, "dsim_dphy_extra");
	if (IS_ERR_OR_NULL(dsim->phy_ex)) {
		dsim_err("failed to get extra phy. It's not mandatary.\n");
		dsim->phy_ex = NULL;
	}

	dsim->dev = dev;

#ifdef CONFIG_DPHY_APB_CONTROL
	dsim->dphy_apb_pdev = of_find_device_by_node(of_find_node_by_name(NULL, "dsim_dphy_apb"));
#endif

	return 0;
}

static int dsim_get_data_lanes(struct dsim_device *dsim)
{
	int i;

	if (dsim->data_lane_cnt > MAX_DSIM_DATALANE_CNT) {
		dsim_err("%d data lane couldn't be supported\n",
				dsim->data_lane_cnt);
		return -EINVAL;
	}

	dsim->data_lane = DSIM_LANE_CLOCK;
	for (i = 1; i < dsim->data_lane_cnt + 1; ++i)
		dsim->data_lane |= 1 << i;

	dsim_info("%s: lanes(0x%x)\n", __func__, dsim->data_lane);

	return 0;
}

static int dsim_init_resources(struct dsim_device *dsim, struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dsim_err("failed to get mem resource\n");
		return -ENOENT;
	}
	dsim_info("res: start(0x%x), end(0x%x)\n", (u32)res->start, (u32)res->end);

	dsim->res.regs = devm_ioremap_resource(dsim->dev, res);
	if (!dsim->res.regs) {
		dsim_err("failed to remap DSIM SFR region\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dsim_info("no 2nd mem resource\n");
		dsim->res.phy_regs = NULL;
	} else {
		dsim_info("dphy res: start(0x%x), end(0x%x)\n",
				(u32)res->start, (u32)res->end);

		dsim->res.phy_regs = devm_ioremap_resource(dsim->dev, res);
		if (!dsim->res.phy_regs) {
			dsim_err("failed to remap DSIM DPHY SFR region\n");
			return -EINVAL;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dsim_info("no extra dphy resource\n");
		dsim->res.phy_regs_ex = NULL;
	} else {
		dsim_info("dphy_extra res: start(0x%x), end(0x%x)\n",
				(u32)res->start, (u32)res->end);

		dsim->res.phy_regs_ex = devm_ioremap_resource(dsim->dev, res);
		if (!dsim->res.phy_regs_ex) {
			dsim_err("failed to remap DSIM DPHY(EXTRA) SFR region\n");
			return -EINVAL;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dsim_err("failed to get irq resource\n");
		return -ENOENT;
	}

	dsim->res.irq = res->start;
	ret = devm_request_irq(dsim->dev, res->start,
			dsim_irq_handler, 0, pdev->name, dsim);
	if (ret) {
		dsim_err("failed to install DSIM irq\n");
		return -EINVAL;
	}
	disable_irq(dsim->res.irq);

	dsim->res.ss_regs = dpu_get_sysreg_addr();
	if (IS_ERR_OR_NULL(dsim->res.ss_regs)) {
		dsim_err("failed to get sysreg addr\n");
		return -EINVAL;
	}

	return 0;
}

static int dsim_register_panel(struct dsim_device *dsim)
{
	struct dsim_regs regs;
	u32 panel_id;
	int ret;

	dsim_info("dsim:%d %s +\n", dsim->id,
__func__);

#if IS_ENABLED(CONFIG_USDM_PANEL)
	dsim->panel = get_panel_drvdata(dsim->id);
#else
	dsim->panel = get_panel_drvdata();
#endif
	if ((dsim->panel != NULL ) && (dsim->panel->found == true)) {
		dsim_info("%s: xres: %d, yres: %d\n",
			__func__, dsim->panel->lcd_info.xres, dsim->panel->lcd_info.yres);
		/* clock and data lane count are stored for DSIM init */
		dsim->clks.hs_clk = dsim->panel->lcd_info.hs_clk;
		dsim->clks.esc_clk = dsim->panel->lcd_info.esc_clk;
		dsim->data_lane_cnt = dsim->panel->lcd_info.data_lane;
		dsim->continuous_underrun_max = dsim->panel->lcd_info.continuous_underrun_max;
		dsim_info("panel is already found in panel driver\n");
		return 0;
	}

#if IS_ENABLED(CONFIG_USDM_PANEL)
	dsim_err("%s can't found panel\n", __func__);
	BUG();
#endif
	pm_runtime_get_sync(dsim->dev);

	/* DPHY power on : iso release */
	phy_power_on(dsim->phy);
	if (dsim->phy_ex)
		phy_power_on(dsim->phy_ex);

	dsim_reg_preinit(dsim->id);
	dsim_reg_start(dsim->id);

	dsim->state = DSIM_STATE_ON;
	enable_irq(dsim->res.irq);

	dsim_read_panel_id(dsim, &panel_id);
	dsim_info("panel_id = 0x%x\n", panel_id);

	if (!IS_ENABLED(CONFIG_EXYNOS_VIRTUAL_DISPLAY)) {
		ret = dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_REGISTER, &panel_id);
		if (ret) {
			dsim_err("%s: cannot find proper panel\n", __func__);
			__dsim_dump(dsim->id, &regs);
			BUG();
		}
	}

	dsim->clks.hs_clk = dsim->panel->lcd_info.hs_clk;
	dsim->clks.esc_clk = dsim->panel->lcd_info.esc_clk;
	dsim->data_lane_cnt = dsim->panel->lcd_info.data_lane;
	dsim->data_lane = 0x1F; /* 4 data lane + 1 clock lane */
	dsim->continuous_underrun_max = dsim->panel->lcd_info.continuous_underrun_max;

	dsim->state = DSIM_STATE_OFF;

	if (dsim_reg_stop(dsim->id, dsim->data_lane) < 0) {
		dsim_to_regs_param(dsim, &regs);
		__dsim_dump(dsim->id, &regs);
		return -EBUSY;
	}
	disable_irq(dsim->res.irq);

	phy_power_off(dsim->phy);
	if (dsim->phy_ex)
		phy_power_off(dsim->phy_ex);

	pm_runtime_put_sync(dsim->dev);

	dsim_info("%s -\n", __func__);

	return 0;
}

static int dsim_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct dsim_device *dsim = NULL;
#if IS_ENABLED(CONFIG_USDM_PANEL)
	char name[32];
#endif

	dsim = devm_kzalloc(dev, sizeof(struct dsim_device), GFP_KERNEL);
	if (!dsim) {
		dsim_err("failed to allocate dsim device.\n");
		ret = -ENOMEM;
		goto err;
	}

	dma_set_mask(dev, DMA_BIT_MASK(32));

	ret = dsim_parse_dt(dsim, dev);
	if (ret)
		goto err_dt;

	dsim_drvdata[dsim->id] = dsim;
	ret = dsim_get_clocks(dsim);
	if (ret)
		goto err_dt;

	spin_lock_init(&dsim->slock);
	mutex_init(&dsim->cmd_lock);
	init_completion(&dsim->ph_wr_comp);
	init_completion(&dsim->rd_comp);

	ret = dsim_init_resources(dsim, pdev);
	if (ret)
		goto err_dt;

	dsim_init_subdev(dsim);
	platform_set_drvdata(pdev, dsim);
	timer_setup(&dsim->cmd_timer, dsim_cmd_fail_detector, 0);

#if IS_ENABLED(CONFIG_USDM_PANEL)
	snprintf(name, 32, "dsim%d-wq", dsim->id);
	INIT_WORK(&dsim->wr_timeout_work, dsim_write_timeout_fn);
	dsim->wq = create_workqueue(name);
#endif

#if defined(CONFIG_CPU_IDLE)
	dsim->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev),1);
	dsim_info("dsim idle_ip_index[%d]\n", dsim->idle_ip_index);
	if (dsim->idle_ip_index < 0)
		dsim_warn("idle ip index is not provided for dsim\n");
	exynos_update_ip_idle_status(dsim->idle_ip_index, 0);
#endif

	pm_runtime_enable(dev);

	iommu_register_device_fault_handler(dev, dpu_sysmmu_fault_handler,
			NULL);

	phy_init(dsim->phy);
	if (dsim->phy_ex)
		phy_init(dsim->phy_ex);

	dsim_register_panel(dsim);

	ret = dsim_get_data_lanes(dsim);
	if (ret)
		goto err_dt;

	dsim->state = DSIM_STATE_INIT;
	dsim_enable(dsim);

#if defined(BRINGUP_DSIM_BIST)
	/* TODO: This is for dsim BIST mode in zebu emulator. only for test*/
	dsim_call_panel_ops(dsim, EXYNOS_PANEL_IOC_DISPLAYON, NULL);
	dsim_reg_set_bist(dsim->id, true);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_DPU_TC_SYSFS_ITF)
	dsim_create_irq_err_sysfs(dsim);
#endif

	/* for debug */
	/* dsim_dump(dsim); */

	dsim_create_cmd_rw_sysfs(dsim);

#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION)
	dsim->esd_recovering = false;
#if defined(CONFIG_EXYNOS_READ_ESD_SOLUTION_TEST)
	dsim_create_esd_test_sysfs(dsim);
#endif
#endif

#ifdef DPHY_LOOP
	dsim_reg_set_dphy_loop_back_test(dsim->id);
#endif

	dsim_info("dsim%d driver(%s mode) has been probed.\n", dsim->id,
		dsim->panel->lcd_info.mode == DECON_MIPI_COMMAND_MODE ? "cmd" : "video");
	return 0;

err_dt:
	kfree(dsim);
err:
	return ret;
}

static int dsim_remove(struct platform_device *pdev)
{
	struct dsim_device *dsim = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	mutex_destroy(&dsim->cmd_lock);
	dsim_info("dsim%d driver removed\n", dsim->id);

	return 0;
}

static void dsim_shutdown(struct platform_device *pdev)
{
#if 0
	struct dsim_device *dsim = platform_get_drvdata(pdev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_SHUTDOWN, &dsim->sd, ktime_set(0, 0));
	dsim_info("%s + state:%d\n", __func__, dsim->state);

	dsim_disable(dsim);

	dsim_info("%s -\n", __func__);
#else
	dsim_info("%s +-\n", __func__);
#endif
}

static int dsim_runtime_suspend(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_SUSPEND, &dsim->sd, ktime_set(0, 0));
	dsim_dbg("%s +\n", __func__);
	clk_disable_unprepare(dsim->res.aclk);
	dsim_dbg("%s -\n", __func__);
	return 0;
}

static int dsim_runtime_resume(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_RESUME, &dsim->sd, ktime_set(0, 0));
	dsim_dbg("%s: +\n", __func__);
	clk_prepare_enable(dsim->res.aclk);
	dsim_dbg("%s -\n", __func__);
	return 0;
}

static const struct of_device_id dsim_of_match[] = {
	{ .compatible = "samsung,exynos9-dsim" },
	{},
};
MODULE_DEVICE_TABLE(of, dsim_of_match);

static const struct dev_pm_ops dsim_pm_ops = {
	.runtime_suspend	= dsim_runtime_suspend,
	.runtime_resume		= dsim_runtime_resume,
};

struct platform_driver dsim_driver __refdata = {
	.probe			= dsim_probe,
	.remove			= dsim_remove,
	.shutdown		= dsim_shutdown,
	.driver = {
		.name		= DSIM_MODULE_NAME,
		.owner		= THIS_MODULE,
		.pm		= &dsim_pm_ops,
		.of_match_table	= of_match_ptr(dsim_of_match),
		.suppress_bind_attrs = true,
	}
};


MODULE_AUTHOR("Yeongran Shin <yr613.shin@samsung.com>");
MODULE_DESCRIPTION("Samusung EXYNOS DSIM driver");
MODULE_LICENSE("GPL");
