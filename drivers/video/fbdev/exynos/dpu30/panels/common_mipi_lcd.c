/* drivers/video/exynos/decon/panels/common_mipi_lcd.c
 *
 * Samsung SoC MIPI LCD driver.
 *
 * Copyright (c) 2017 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <video/mipi_display.h>
#include <linux/platform_device.h>

#include "../decon.h"
#include "../dsim.h"
#include "../mcd_decon.h"
#include "exynos_panel_drv.h"
#include "exynos_panel.h"
#include "panel_drv_header.h"

#ifdef CONFIG_USDM_PANEL_DISPLAY_MODE
#include "exynos_panel_modes.h"
#endif

#define MCD_PANEL_PROBE_DELAY_MSEC (5000)

#define call_mcd_panel_func(p, func, args...) \
	    (((p) && (p)->funcs && (p)->funcs->func) ? (p)->funcs->func(p, ##args) : -EINVAL)

static DEFINE_MUTEX(cmd_lock);

#if defined(CONFIG_USDM_PANEL_DISPLAY_MODE)
static int panel_drv_get_panel_display_mode(struct exynos_panel_device *ctx)
{
	struct exynos_panel_info *info;
	struct panel_display_modes *pdms;
	struct exynos_display_modes *exynos_modes;
	int ret;

	if (!ctx)
		return -EINVAL;

	info = &ctx->lcd_info;
	ret = call_mcd_panel_func(ctx->mcd_panel_dev, get_display_mode, &pdms);
	if (ret < 0) {
		DPU_ERR_PANEL("%s: failed to ioctl(PANEL_IOC_GET_DISPLAY_MODE)\n", __func__);
		return ret;
	}

	/* create unique exynos_display_mode array using panel_display_modes */
	exynos_modes =
		exynos_display_modes_create_from_panel_display_modes(ctx, pdms);
	if (!exynos_modes) {
		DPU_ERR_PANEL("%s: could not create exynos_display_modes\n", __func__);
		return -ENOMEM;
	}

	/*
	 * initialize display mode information of exynos_panel_info
	 * using exynos_display_modes.
	 */
	exynos_display_modes_update_panel_info(ctx, exynos_modes);
	info->panel_modes = pdms;
	info->panel_mode_idx = pdms->native_mode;
	info->cmd_lp_ref =
		pdms->modes[pdms->native_mode]->cmd_lp_ref;

	return 0;
}
#endif

#define DSIM_TX_FLOW_CONTROL
static void print_tx(u8 cmd_id, const u8 *cmd, int size)
{
	char data[256];
	int i, len;
	bool newline = false;

	len = snprintf(data, ARRAY_SIZE(data), "(%02X) ", cmd_id);
	for (i = 0; i < min((int)size, 256); i++) {
		if (newline)
			len += snprintf(data + len, ARRAY_SIZE(data) - len, "     ");
		newline = (!((i + 1) % 16) || (i + 1 == size)) ? true : false;
		len += snprintf(data + len, ARRAY_SIZE(data) - len,
				"%02X%s", cmd[i], newline ? "\n" : " ");
		if (newline) {
			DPU_INFO_PANEL("%s: %s", __func__, data);
			len = 0;
		}
	}
}

static void print_rx(u8 addr, u8 *buf, int size)
{
	char data[256];
	int i, len;
	bool newline = false;

	len = snprintf(data, ARRAY_SIZE(data), "(%02X) ", addr);
	for (i = 0; i < min((int)size, 256); i++) {
		if (newline)
			len += snprintf(data + len, ARRAY_SIZE(data) - len, "	  ");
		newline = (!((i + 1) % 16) || (i + 1 == size)) ? true : false;
		len += snprintf(data + len, ARRAY_SIZE(data) - len,
				"%02X ", buf[i]);
		if (newline) {
			DPU_INFO_PANEL("%s: %s", __func__, data);
			len = 0;
		}
	}
	DPU_INFO_PANEL("%s: %s\n", __func__, data);
}

static void print_dsim_cmd(const struct exynos_dsim_cmd *cmd_set, int size)
{
	int i;

	for (i = 0; i < size; i++)
		print_tx(cmd_set[i].type,
				cmd_set[i].data_buf,
				cmd_set[i].data_len);
}

static int mipi_write(void *ctx, u8 cmd_id, const u8 *cmd, u32 offset, int size, u32 option)
{
	int ret, retry = 3;
	unsigned long d0;
	u32 type, d1;
	bool block = (option & DSIM_OPTION_WAIT_TX_DONE);
	struct dsim_device *dsim = get_dsim_device_from_exynos_panel(ctx);

	if (!cmd) {
		DPU_ERR_PANEL("%s: cmd is null\n", __func__);
		return -EINVAL;
	}

	if (cmd_id == MIPI_DSI_WR_DSC_CMD) {
		type = MIPI_DSI_DSC_PRA;
		d0 = (unsigned long)cmd[0];
		d1 = 0;
	} else if (cmd_id == MIPI_DSI_WR_PPS_CMD) {
		type = MIPI_DSI_DSC_PPS;
		d0 = (unsigned long)cmd;
		d1 = size;
	} else if (cmd_id == MIPI_DSI_WR_GEN_CMD) {
		if (size == 1) {
			type = MIPI_DSI_DCS_SHORT_WRITE;
			d0 = (unsigned long)cmd[0];
			d1 = 0;
		} else {
			type = MIPI_DSI_DCS_LONG_WRITE;
			d0 = (unsigned long)cmd;
			d1 = size;
		}
	} else {
		DPU_INFO_PANEL("%s: invalid cmd_id %d\n", __func__, cmd_id);
		return -EINVAL;
	}

	mutex_lock(&cmd_lock);
	while (--retry >= 0) {
		if (offset > 0) {
			int gpara_len = 1;
			u8 gpara[4] = { 0xB0, 0x00 };

			/* gpara 16bit offset */
			if (option & DSIM_OPTION_2BYTE_GPARA)
				gpara[gpara_len++] = (offset >> 8) & 0xFF;

			gpara[gpara_len++] = offset & 0xFF;

			/* pointing gpara */
			if (option & DSIM_OPTION_POINT_GPARA)
				gpara[gpara_len++] = cmd[0];

			if (panel_cmd_log_enabled(PANEL_CMD_LOG_DSI_TX))
				print_tx(MIPI_DSI_DCS_LONG_WRITE, gpara, gpara_len);
			if (dsim_write_data(dsim, MIPI_DSI_DCS_LONG_WRITE,
						(unsigned long)gpara, gpara_len, false)) {
				DPU_ERR_PANEL("%s: failed to write gpara %d (retry %d)\n", __func__,
						offset, retry);
				continue;
			}
		}
		if (panel_cmd_log_enabled(PANEL_CMD_LOG_DSI_TX))
			print_tx(type, cmd, size);
		if (dsim_write_data(dsim, type, d0, d1, block)) {
			DPU_ERR_PANEL("%s: failed to write cmd %02X size %d(retry %d)\n", __func__,
					cmd[0], size, retry);
			continue;
		}

		break;
	}

	if (retry < 0) {
		DPU_ERR_PANEL("%s: failed: exceed retry count (cmd %02X)\n", __func__,
				cmd[0]);
		ret = -EIO;
		goto error;
	}

	DPU_DEBUG_PANEL("%s: cmd_id %d, addr %02X offset %d size %d\n", __func__,
			cmd_id, cmd[0], offset, size);
	ret = size;

error:
	mutex_unlock(&cmd_lock);
	return ret;
}

#define MAX_DSIM_PH_SIZE (32)
#define MAX_DSIM_PL_SIZE (DSIM_PL_FIFO_THRESHOLD)
#define MAX_CMD_SET_SIZE (1024)
static struct exynos_dsim_cmd cmd_set[MAX_CMD_SET_SIZE];
static int mipi_write_table(void *ctx, const struct cmd_set *cmd, int size, u32 option)
{
	int ret, total_size = 0;
	struct dsim_device *dsim = get_dsim_device_from_exynos_panel(ctx);
	int i, from = 0, sz_pl = 0;
	s64 elapsed_usec;
	struct timespec64 cur_ts, last_ts, delta_ts;

	if (!cmd) {
		DPU_ERR_PANEL("%s: cmd is null\n", __func__);
		return -EINVAL;
	}

	if (size <= 0) {
		DPU_ERR_PANEL("%s: invalid cmd size %d\n", __func__, size);
		return -EINVAL;
	}

	if (size > MAX_CMD_SET_SIZE) {
		DPU_ERR_PANEL("%s: exceeded MAX_CMD_SET_SIZE(%d) %d\n", __func__,
				MAX_CMD_SET_SIZE, size);
		return -EINVAL;
	}

	ktime_get_ts64(&last_ts);
	mutex_lock(&cmd_lock);
	for (i = 0; i < size; i++) {
		if (cmd[i].buf == NULL) {
			DPU_ERR_PANEL("%s: cmd[%d].buf is null\n", __func__, i);
			continue;
		}

		if (cmd[i].cmd_id == MIPI_DSI_WR_DSC_CMD) {
			cmd_set[i].type = MIPI_DSI_DSC_PRA;
			cmd_set[i].data_buf = cmd[i].buf;
			cmd_set[i].data_len = 1;
		} else if (cmd[i].cmd_id == MIPI_DSI_WR_PPS_CMD) {
			cmd_set[i].type = MIPI_DSI_DSC_PPS;
			cmd_set[i].data_buf = cmd[i].buf;
			cmd_set[i].data_len = cmd[i].size;
		} else if (cmd[i].cmd_id == MIPI_DSI_WR_GEN_CMD) {
			if (cmd[i].size == 1) {
				cmd_set[i].type = MIPI_DSI_DCS_SHORT_WRITE;
				cmd_set[i].data_buf = cmd[i].buf;
				cmd_set[i].data_len = 1;
			} else {
				cmd_set[i].type = MIPI_DSI_DCS_LONG_WRITE;
				cmd_set[i].data_buf = cmd[i].buf;
				cmd_set[i].data_len = cmd[i].size;
			}
		} else {
			DPU_INFO_PANEL("%s: invalid cmd_id %d\n", __func__, cmd[i].cmd_id);
			ret = -EINVAL;
			goto error;
		}

#if defined(DSIM_TX_FLOW_CONTROL)
		if ((i - from >= MAX_DSIM_PH_SIZE) ||
			(sz_pl + ALIGN(cmd_set[i].data_len, 4) >= MAX_DSIM_PL_SIZE)) {
			if (dsim_write_cmd_set(dsim, &cmd_set[from], i - from, false)) {
				DPU_ERR_PANEL("%s: failed to write cmd_set\n", __func__);
				ret = -EIO;
				goto error;
			}
			DPU_DEBUG_PANEL("%s: cmd_set:%d pl:%d\n", __func__, i - from, sz_pl);
			if (panel_cmd_log_enabled(PANEL_CMD_LOG_DSI_TX))
				print_dsim_cmd(&cmd_set[from], i - from);
			from = i;
			sz_pl = 0;
		}
#endif
		sz_pl += ALIGN(cmd_set[i].data_len, 4);
		total_size += cmd_set[i].data_len;
	}

	if (dsim_write_cmd_set(dsim, &cmd_set[from], i - from, false)) {
		DPU_ERR_PANEL("%s: failed to write cmd_set\n", __func__);
		ret = -EIO;
		goto error;
	}

	ktime_get_ts64(&cur_ts);
	delta_ts = timespec64_sub(cur_ts, last_ts);
	elapsed_usec = timespec64_to_ns(&delta_ts) / 1000;
	DPU_DEBUG_PANEL("%s: done (cmd_set:%d size:%d elapsed %2lld.%03lld msec)\n", __func__,
			size, total_size,
			elapsed_usec / 1000, elapsed_usec % 1000);
	if (panel_cmd_log_enabled(PANEL_CMD_LOG_DSI_TX))
		print_dsim_cmd(&cmd_set[from], i - from);

	ret = total_size;

error:
	mutex_unlock(&cmd_lock);

	return ret;
}

static int mipi_sr_write(void *ctx, u8 cmd_id, const u8 *cmd, u32 offset, int size, u32 option)
{
	int ret = 0;
	struct dsim_device *dsim = get_dsim_device_from_exynos_panel(ctx);
	s64 elapsed_usec;
	struct timespec64 cur_ts, last_ts, delta_ts;
	int align = 0;

	if (!cmd) {
		DPU_ERR_PANEL("%s: cmd is null\n", __func__);
		return -EINVAL;
	}

	if (option & PKT_OPTION_SR_ALIGN_12)
		align = 12;
	else if (option & PKT_OPTION_SR_ALIGN_16)
		align = 16;

	if (align == 0) {
		/* protect for already released panel: 16byte align */
		DPU_ERR_PANEL("%s: sram packets need to align option, set force to 16\n", __func__);
		align = 16;
	}
	ktime_get_ts64(&last_ts);

	mutex_lock(&cmd_lock);
	ret = dsim_sr_write_data(dsim, cmd, size, align);
	mutex_unlock(&cmd_lock);

	ktime_get_ts64(&cur_ts);
	delta_ts = timespec64_sub(cur_ts, last_ts);
	elapsed_usec = timespec64_to_ns(&delta_ts) / 1000;
	DPU_DEBUG_PANEL("%s: done (size:%d elapsed %2lld.%03lld msec)\n", __func__,
			size, elapsed_usec / 1000, elapsed_usec % 1000);

	return ret;
}

static int mipi_read(void *ctx, u8 addr, u32 offset, u8 *buf, int size, u32 option)
{
	int ret, retry = 3;
	struct dsim_device *dsim = get_dsim_device_from_exynos_panel(ctx);

	if (!buf) {
		DPU_ERR_PANEL("%s: buf is null\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&cmd_lock);
	while (--retry >= 0) {
		if (offset > 0) {
			int gpara_len = 1;
#if defined(CONFIG_USDM_PANEL_FRESH) || defined(CONFIG_USDM_PANEL_WISE)			
			u8 gpara[4] = { 0xA0, 0x00 };
#else
			u8 gpara[4] = { 0xB0, 0x00 };
#endif

			/* gpara 16bit offset */
			if (option & DSIM_OPTION_2BYTE_GPARA)
				gpara[gpara_len++] = (offset >> 8) & 0xFF;

			gpara[gpara_len++] = offset & 0xFF;

			/* pointing gpara */
			if (option & DSIM_OPTION_POINT_GPARA)
				gpara[gpara_len++] = addr;

			if (panel_cmd_log_enabled(PANEL_CMD_LOG_DSI_TX))
				print_tx(MIPI_DSI_DCS_LONG_WRITE, gpara, gpara_len);
			if (dsim_write_data(dsim, MIPI_DSI_DCS_LONG_WRITE,
						(unsigned long)gpara, gpara_len, false)) {
				DPU_ERR_PANEL("%s: failed to write gpara %d (retry %d)\n", __func__,
						offset, retry);
				continue;
			}
		}

		if (panel_cmd_log_enabled(PANEL_CMD_LOG_DSI_TX)) {
			u8 read_cmd1[] = { size };
			u8 read_cmd2[] = { addr };

			print_tx(MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
					read_cmd1, ARRAY_SIZE(read_cmd1));
			print_tx(MIPI_DSI_DCS_READ, read_cmd2, ARRAY_SIZE(read_cmd2));
		}
		ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
				(u32)addr, size, buf);
		if (ret != size) {
			DPU_ERR_PANEL("%s: failed to read addr %02X ofs %d size %d (ret %d, retry %d)\n", __func__,
					addr, offset, size, ret, retry);
			continue;
		}
		if (panel_cmd_log_enabled(PANEL_CMD_LOG_DSI_RX))
			print_rx(addr, buf, size);
		break;
	}

	if (retry < 0) {
		DPU_ERR_PANEL("%s: failed: exceed retry count (addr %02X)\n", __func__, addr);
		ret = -EIO;
		goto error;
	}

	DPU_DEBUG_PANEL("%s: addr %02X ofs %d size %d, buf %02X done\n", __func__,
			addr, offset, size, buf[0]);

	ret = size;

error:
	mutex_unlock(&cmd_lock);
	return ret;
}

static int get_dsim_state(void *ctx)
{
	struct dsim_device *dsim = get_dsim_device_from_exynos_panel(ctx);

	if (dsim == NULL) {
		DPU_ERR_PANEL("%s: dsim is NULL\n", __func__);
		return -ENODEV;
	}

	return (dsim->state == DSIM_STATE_OFF ||
			dsim->state == DSIM_STATE_DOZE_SUSPEND) ?
		CTRL_INTERFACE_STATE_INACTIVE : CTRL_INTERFACE_STATE_ACTIVE;
}

static int parse_dt(void *ctx, struct device_node *np)
{
	extern void parse_lcd_info(struct device_node *np, struct exynos_panel_info *lcd_info);

	parse_lcd_info(np, &((struct exynos_panel_device *)ctx)->lcd_info);

	return 0;
}

static int wait_for_vsync(void *ctx, u32 timeout)
{
	struct decon_device *decon = get_decon_drvdata(0);
	int ret;

	if (!decon)
		return -EINVAL;

	decon_hiber_block_exit(decon);
	ret = decon_wait_for_vsync(decon, timeout);
	decon_hiber_unblock(decon);

	return ret;
}

static int set_bypass(void *ctx, bool on)
{
	struct decon_device *decon = get_decon_drvdata(0);

	if (!decon)
		return -EINVAL;

	if (on)
		decon_bypass_on(decon);
	else
		decon_bypass_off(decon);

	return 0;
}

static int get_bypass(void *ctx)
{
	struct decon_device *decon = get_decon_drvdata(0);

	if (!decon)
		return -EINVAL;

	return atomic_read(&decon->bypass);
}

static int wake_lock(void *ctx, unsigned long timeout)
{
	struct decon_device *decon = get_decon_drvdata(0);

	if (!decon)
		return -EINVAL;

	return decon_wake_lock(decon, timeout);
}

static int wake_unlock(void *ctx)
{
	struct decon_device *decon = get_decon_drvdata(0);

	if (!decon)
		return -EINVAL;

	decon_wake_unlock(decon);
	return 0;
}

static int flush_image(void *ctx)
{
	struct decon_device *decon = get_decon_drvdata(0);

	if (!decon)
		return -EINVAL;

	mcd_decon_flush_image(decon);
	return 0;
}

struct panel_adapter_funcs usdm_panel_adapter_funcs = {
	.read = mipi_read,
	.write = mipi_write,
	.write_table = mipi_write_table,
	.sr_write = mipi_sr_write,
	.get_state = get_dsim_state,
	.parse_dt = parse_dt,
	.wait_for_vsync = wait_for_vsync,
	.wait_for_fsync = NULL,
	.set_bypass = set_bypass,
	.get_bypass = get_bypass,
	.wake_lock = wake_lock,
	.wake_unlock = wake_unlock,
	.flush_image = flush_image,
	.set_lpdt = NULL,
};

void usdm_panel_probe_handler(struct work_struct *data);

work_func_t usdm_panel_adapter_wq_fns[MAX_USDM_PANEL_ADAPTER_WQ] = {
	/*
	[USDM_PANEL_ADAPTER_WQ_VSYNC] = usdm_vblank_handler,
	[USDM_PANEL_ADAPTER_WQ_FSYNC] = usdm_framedone_handler_fsync,
	[USDM_PANEL_ADAPTER_WQ_DISPON] = wq_framedone_handler_dispon,
	*/
	[USDM_PANEL_ADAPTER_WQ_PANEL_PROBE] = usdm_panel_probe_handler,
};

char *usdm_panel_adapter_wq_names[MAX_USDM_PANEL_ADAPTER_WQ] = {
	[USDM_PANEL_ADAPTER_WQ_VSYNC] = "wq_vsync",
	[USDM_PANEL_ADAPTER_WQ_FSYNC] = "wq_fsync",
	[USDM_PANEL_ADAPTER_WQ_DISPON] = "wq_dispon",
	[USDM_PANEL_ADAPTER_WQ_PANEL_PROBE] = "wq_panel_probe",
};

int usdm_panel_adapter_wq_exit(struct exynos_panel_device *ctx)
{
	struct usdm_panel_adapter_wq *w;
	int i;

	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < MAX_USDM_PANEL_ADAPTER_WQ; i++) {
		if (!usdm_panel_adapter_wq_fns[i])
			continue;

		w = &ctx->wqs[i];
		if (w->wq) {
			destroy_workqueue(w->wq);
			w->wq = NULL;
		}
	}
	return 0;
}

int usdm_panel_adapter_wq_init(struct exynos_panel_device *ctx)
{
	struct usdm_panel_adapter_wq *w;
	int ret, i;

	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < MAX_USDM_PANEL_ADAPTER_WQ; i++) {
		if (!usdm_panel_adapter_wq_fns[i])
			continue;

		w = &ctx->wqs[i];
		w->index = i;
		INIT_DELAYED_WORK(&w->dwork, usdm_panel_adapter_wq_fns[i]);
		w->wq = create_singlethread_workqueue(usdm_panel_adapter_wq_names[i]);
		if (!w->wq) {
			pr_err("%s failed to workqueue initialize %s\n", __func__, usdm_panel_adapter_wq_names[i]);
			ret = usdm_panel_adapter_wq_exit(ctx);
			if (ret < 0)
				pr_err("%s failed to wq_exit %d\n", __func__, ret);
			return -EINVAL;
		}
		w->name = usdm_panel_adapter_wq_names[i];
		init_waitqueue_head(&w->wait);
		atomic_set(&w->count, 0);
	}

	return 0;
}

int mcd_drm_panel_check_probe(struct exynos_panel_device *ctx)
{
	int ret = 0;

	if (!ctx || !ctx->mcd_panel_dev)
		return -ENODEV;

	mutex_lock(&ctx->probe_lock);
	if (ctx->mcd_panel_probed)
		goto out;

	ret = call_mcd_panel_func(ctx->mcd_panel_dev, probe);
	if (ret < 0) {
		DPU_ERR_PANEL("%s: mcd_panel probe failed %d", __func__, ret);
		goto out;
	}
	ctx->mcd_panel_probed = true;

	ret = call_mcd_panel_func(ctx->mcd_panel_dev, get_ddi_props, &ctx->ddi_props);
	if (ret < 0) {
		DPU_ERR_PANEL("%s: mcd_panel get ddi props failed %d", __func__, ret);
		goto out;
	}

out:
	mutex_unlock(&ctx->probe_lock);

	return ret;
}

void usdm_panel_probe_handler(struct work_struct *data)
{
	struct usdm_panel_adapter_wq *w = container_of(to_delayed_work(data),
				struct usdm_panel_adapter_wq, dwork);
	struct exynos_panel_device *ctx = container_of(w,
			struct exynos_panel_device, wqs[USDM_PANEL_ADAPTER_WQ_PANEL_PROBE]);
	int ret;

	pr_info("%s +\n", __func__);
	ret = mcd_drm_panel_check_probe(ctx);
	if (ret < 0)
		dev_err(ctx->dev, "%s mcd-panel not probed %d\n", __func__, ret);

	pr_info("%s -\n", __func__);
}

static int common_panel_connected(struct exynos_panel_device *ctx)
{
	int connect = 1;

	call_mcd_panel_func(ctx->mcd_panel_dev, get_panel_state, &connect);

	return connect;
 }

static int common_panel_init(struct exynos_panel_device *ctx)
{
	struct panel_adapter adapter = {
		.ctx = ctx,
		.fifo_size = DSIM_PL_FIFO_THRESHOLD,
		.funcs = &usdm_panel_adapter_funcs,
	};
	int ret;

	mutex_init(&ctx->probe_lock);

	ret = call_mcd_panel_func(ctx->mcd_panel_dev, attach_adapter, &adapter);
	if (ret < 0) {
		DPU_ERR_PANEL("%s: failed to attach adapter\n", __func__);
		return ret;
	}

	ret = call_mcd_panel_func(ctx->mcd_panel_dev, get_ddi_props, &ctx->ddi_props);
	if (ret < 0) {
		DPU_ERR_PANEL("%s: failed to get ddi props\n", __func__);
		return ret;
	}

	ret = usdm_panel_adapter_wq_init(ctx);
	if (ret < 0) {
		DPU_ERR_PANEL("%s: mcd_panel initialize workqueue failed %d", __func__, ret);
		return ret;
	}

#if defined(CONFIG_USDM_PANEL_DISPLAY_MODE)
	ret = panel_drv_get_panel_display_mode(ctx);
	if (ret < 0) {
		DPU_ERR_PANEL("%s: failed to get panel_display_modes\n", __func__);
		return ret;
	}
#endif

	queue_delayed_work(ctx->wqs[USDM_PANEL_ADAPTER_WQ_PANEL_PROBE].wq,
			&ctx->wqs[USDM_PANEL_ADAPTER_WQ_PANEL_PROBE].dwork,
			msecs_to_jiffies(MCD_PANEL_PROBE_DELAY_MSEC));

	return ret;
}

static int common_panel_probe(struct exynos_panel_device *ctx)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, probe);
}

static int common_panel_displayon(struct exynos_panel_device *ctx)
{
	int ret;

	ret = call_mcd_panel_func(ctx->mcd_panel_dev, display_on);
	if (ret) {
		DPU_ERR_PANEL("%s: failed to display on\n", __func__);
		return ret;
	}

	return 0;
}

static int common_panel_suspend(struct exynos_panel_device *ctx)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, sleep_in);
}

static int common_panel_resume(struct exynos_panel_device *ctx)
{
	return 0;
}

static int common_panel_dump(struct exynos_panel_device *ctx)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, debug_dump);
}

static int common_panel_setarea(struct exynos_panel_device *ctx, u32 l, u32 r, u32 t, u32 b)
{
	int ret = 0;
	char column[5];
	char page[5];
	int retry;
	struct dsim_device *dsim = get_dsim_device_from_exynos_panel(ctx);

	column[0] = MIPI_DCS_SET_COLUMN_ADDRESS;
	column[1] = (l >> 8) & 0xff;
	column[2] = l & 0xff;
	column[3] = (r >> 8) & 0xff;
	column[4] = r & 0xff;

	page[0] = MIPI_DCS_SET_PAGE_ADDRESS;
	page[1] = (t >> 8) & 0xff;
	page[2] = t & 0xff;
	page[3] = (b >> 8) & 0xff;
	page[4] = b & 0xff;

	mutex_lock(&cmd_lock);
	retry = 2;

	if (panel_cmd_log_enabled(PANEL_CMD_LOG_DSI_TX))
		print_tx(MIPI_DSI_DCS_LONG_WRITE, column, ARRAY_SIZE(column));
	while (dsim_write_data(dsim, MIPI_DSI_DCS_LONG_WRITE,
				(unsigned long)column, ARRAY_SIZE(column), false) != 0) {
		DPU_ERR_PANEL("%s: failed to write COLUMN_ADDRESS\n", __func__);
		if (--retry <= 0) {
			DPU_ERR_PANEL("%s: COLUMN_ADDRESS is failed: exceed retry count\n", __func__);
			ret = -EINVAL;
			goto error;
		}
	}

	retry = 2;
	if (panel_cmd_log_enabled(PANEL_CMD_LOG_DSI_TX))
		print_tx(MIPI_DSI_DCS_LONG_WRITE, page, ARRAY_SIZE(page));
	while (dsim_write_data(dsim, MIPI_DSI_DCS_LONG_WRITE,
				(unsigned long)page, ARRAY_SIZE(page), true) != 0) {
		DPU_ERR_PANEL("%s: failed to write PAGE_ADDRESS\n", __func__);
		if (--retry <= 0) {
			DPU_ERR_PANEL("%s: PAGE_ADDRESS is failed: exceed retry count\n", __func__);
			ret = -EINVAL;
			goto error;
		}
	}

	DPU_DEBUG_PANEL("%s: RECT [l:%d r:%d t:%d b:%d w:%d h:%d]\n", __func__,
			l, r, t, b, r - l + 1, b - t + 1);

error:
	mutex_unlock(&cmd_lock);
	return ret;
}

static int common_panel_poweron(struct exynos_panel_device *ctx)
{
	int ret;

	ret = mcd_drm_panel_check_probe(ctx);
	if (ret < 0)
		return ret;

	return call_mcd_panel_func(ctx->mcd_panel_dev, power_on);
}

static int common_panel_poweroff(struct exynos_panel_device *ctx)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, power_off);
}

static int common_panel_sleepin(struct exynos_panel_device *ctx)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, sleep_in);
}

static int common_panel_sleepout(struct exynos_panel_device *ctx)
{
	int ret;

	ret = mcd_drm_panel_check_probe(ctx);
	if (ret < 0)
		return ret;

	return call_mcd_panel_func(ctx->mcd_panel_dev, sleep_out);
}

static int common_panel_notify(struct exynos_panel_device *ctx, void *data)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, frame_done, data);
}

static int common_panel_reset(struct exynos_panel_device *ctx)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, reset_lp11);
}

static int common_panel_reset_disable(struct exynos_panel_device *ctx)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, reset_disable);
}

#ifdef CONFIG_USDM_PANEL_LPM
static int common_panel_doze(struct exynos_panel_device *ctx)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, doze);
}

static int common_panel_doze_suspend(struct exynos_panel_device *ctx)
{
	return call_mcd_panel_func(ctx->mcd_panel_dev, doze);
}
#else
static int common_panel_doze(struct exynos_panel_device *ctx) { return 0; }
static int common_panel_doze_suspend(struct exynos_panel_device *ctx) { return 0; }
#endif

#if defined(CONFIG_USDM_PANEL_DISPLAY_MODE)
static int common_panel_get_display_mode(struct exynos_panel_device *ctx, void *data)
{
	return panel_drv_get_panel_display_mode(ctx);
}

static int common_panel_set_display_mode(struct exynos_panel_device *ctx, void *data)
{
	struct panel_display_modes *pdms;
	struct panel_display_mode *pdm;
	int ret, panel_mode_idx;

	if (!data)
		return -EINVAL;

	ret = call_mcd_panel_func(ctx->mcd_panel_dev, get_display_mode, &pdms);
	if (ret < 0) {
		dev_err(ctx->dev, "%s: mcd_panel get_display_mode failed %d", __func__, ret);
		return ret;
	}

	/* TODO: replace with 'exynos_panel_find_panel_mode()' function */
	panel_mode_idx = *(int *)data;
	if (panel_mode_idx < 0 ||
		panel_mode_idx >= pdms->num_modes) {
		DPU_ERR_PANEL("%s: invalid panel_mode_idx(%d)\n", __func__, panel_mode_idx);
		return -EINVAL;
	}

	pdm = pdms->modes[panel_mode_idx];

	return call_mcd_panel_func(ctx->mcd_panel_dev, set_display_mode, pdm);
}
#endif

struct exynos_panel_ops common_panel_ops = {
	.init		= common_panel_init,
	.probe		= common_panel_probe,
	.suspend	= common_panel_suspend,
	.resume		= common_panel_resume,
	.dump		= common_panel_dump,
	.connected	= common_panel_connected,
	.setarea	= common_panel_setarea,
	.poweron	= common_panel_poweron,
	.poweroff	= common_panel_poweroff,
	.sleepin	= common_panel_sleepin,
	.sleepout	= common_panel_sleepout,
	.displayon	= common_panel_displayon,
	.notify		= common_panel_notify,
	.read_state	= NULL,
	.set_error_cb	= NULL,
	.reset_panel = common_panel_reset,
	.reset_disable_panel = common_panel_reset_disable,
	.doze		= common_panel_doze,
	.doze_suspend	= common_panel_doze_suspend,
	.mres = NULL,
#if defined(CONFIG_USDM_PANEL_DISPLAY_MODE)
	.get_display_mode = common_panel_get_display_mode,
	.set_display_mode = common_panel_set_display_mode,
#endif
};
