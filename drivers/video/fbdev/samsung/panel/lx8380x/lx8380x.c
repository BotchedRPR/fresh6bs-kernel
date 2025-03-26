/*
 * linux/drivers/video/fbdev/exynos/panel/rm69091/rm69091.c
 *
 * S6E3HAB Dimming Driver
 *
 * Copyright (c) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of_gpio.h>
#include <video/mipi_display.h>
#include "../panel.h"
#include "../panel_function.h"
#ifdef CONFIG_USDM_PANEL_DIMMING
#include "../dimming.h"
#include "../panel_dimming.h"
#endif
#include "../panel_drv.h"
#include "../panel_debug.h"
#include "lx8380x.h"
#include "oled_common.h"

#ifdef PANEL_PR_TAG
#undef PANEL_PR_TAG
#define PANEL_PR_TAG	"ddi"
#endif

//minwoo76945.kim Todo
//#ifdef CONFIG_USDM_PANEL_DIMMING
int generate_brt_step_table(struct brightness_table *brt_tbl)
{
	int ret = 0;
	int i = 0, j = 0, k = 0;

	if (unlikely(!brt_tbl || !brt_tbl->brt)) {
		panel_err("invalid parameter\n");
		return -EINVAL;
	}
	if (unlikely(!brt_tbl->step_cnt)) {
		if (likely(brt_tbl->brt_to_step)) {
			panel_info("we use static step table\n");
			return ret;
		} else {
			panel_err("invalid parameter, all table is NULL\n");
			return -EINVAL;
		}
	}

	brt_tbl->sz_brt_to_step = 0;
	for(i = 0; i < brt_tbl->sz_step_cnt; i++)
		brt_tbl->sz_brt_to_step += brt_tbl->step_cnt[i];

	brt_tbl->brt_to_step =
		(u32 *)kmalloc(brt_tbl->sz_brt_to_step * sizeof(u32), GFP_KERNEL);

	if (unlikely(!brt_tbl->brt_to_step)) {
		panel_err("alloc fail\n");
		return -EINVAL;
	}
	brt_tbl->brt_to_step[0] = brt_tbl->brt[0];
	i = 1;
	while (i < brt_tbl->sz_brt_to_step) {
		for (k = 1; k < brt_tbl->sz_brt; k++) {
			for (j = 1; j <= brt_tbl->step_cnt[k]; j++, i++) {
				brt_tbl->brt_to_step[i] = disp_interpolation64(brt_tbl->brt[k - 1] * disp_pow(10, 2),
					brt_tbl->brt[k] * disp_pow(10, 2), j, brt_tbl->step_cnt[k]);
				brt_tbl->brt_to_step[i] = disp_pow_round(brt_tbl->brt_to_step[i], 2);
				brt_tbl->brt_to_step[i] = disp_div64(brt_tbl->brt_to_step[i], disp_pow(10, 2));
				if (brt_tbl->brt[brt_tbl->sz_brt - 1] < brt_tbl->brt_to_step[i]) {

					brt_tbl->brt_to_step[i] = disp_pow_round(brt_tbl->brt_to_step[i], 2);
				}
				if (i >= brt_tbl->sz_brt_to_step) {
					panel_err("step cnt over %d %d\n", i, brt_tbl->sz_brt_to_step);
					break;
				}
			}
		}
	}
	return ret;
}

//#endif /* CONFIG_USDM_PANEL_DIMMING */

int lx8380x_maptbl_init_gamma_mode2_brt(struct maptbl *tbl)
{
	struct panel_info *panel_data;
	struct panel_device *panel;
	struct panel_dimming_info *panel_dim_info;
	//todo:remove
	panel_info("++\n");
	if (tbl == NULL) {
		panel_err("maptbl is null\n");
		return -EINVAL;
	}

	if (tbl->pdata == NULL) {
		panel_err("pdata is null\n");
		return -EINVAL;
	}

	panel = tbl->pdata;
	panel_data = &panel->panel_data;

	panel_dim_info = panel_data->panel_dim_info[PANEL_BL_SUBDEV_TYPE_DISP];

	if (panel_dim_info == NULL) {
		panel_err("panel_dim_info is null\n");
		return -EINVAL;
	}

	if (panel_dim_info->brt_tbl == NULL) {
		panel_err("panel_dim_info->brt_tbl is null\n");
		return -EINVAL;
	}

	generate_brt_step_table(panel_dim_info->brt_tbl);

	/* initialize brightness_table */
	memcpy(&panel->panel_bl.subdev[PANEL_BL_SUBDEV_TYPE_DISP].brt_tbl,
			panel_dim_info->brt_tbl, sizeof(struct brightness_table));

	return 0;
}

int lx8380x_maptbl_getidx_acl_control(struct maptbl *tbl)
{
	struct panel_device *panel = (struct panel_device *)tbl->pdata;
	struct panel_bl_device *panel_bl;
	struct panel_info *panel_data;
	int row;

	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_bl = &panel->panel_bl;
	panel_data = &panel->panel_data;

	if (is_hbm_brightness(panel_bl, panel_bl->props.brightness)) {
		row = panel_data->props.adaptive_control = LX8380X_ACL_RATIO_8;
	} else {
		row = panel_data->props.adaptive_control = LX8380X_ACL_RATIO_0;
	}

	panel_info("set acl %d\n", row);
	return maptbl_index(tbl, 0, row, 0);
}

/* required delay(ms) from display on cmd */
#define SMOOTH_DIM_DELAY_AFTER_DISPLAYON_MS 500
/* required delay(ms) from last brightnes change cmd */
#define SMOOTH_DIM_DELAY_PREV_BRIGHTNESS_MS 180
bool lx8380x_cond_is_smooth_dimming_available(struct panel_device *panel)
{
	struct panel_bl_device *panel_bl;
	struct panel_info *panel_data;
	static ktime_t last_time;
	ktime_t now;
	s64 diff;
	bool ret = false;
	
	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_bl = &panel->panel_bl;
	panel_data = &panel->panel_data;
	now = ktime_get();

	if (panel->state.disp_on != PANEL_DISPLAY_ON) {
		panel_info("panel is off state, false\n");
		goto exit;
	}
	diff = ktime_to_ms(ktime_sub(now, panel->ktime_panel_disp_on));
	if (diff < SMOOTH_DIM_DELAY_AFTER_DISPLAYON_MS) {
		panel_info("diff with displayon: %lld / %d, set 8frame\n", diff, SMOOTH_DIM_DELAY_AFTER_DISPLAYON_MS);
		goto exit;
	}
	diff = ktime_to_ms(ktime_sub(now, last_time));
	if (diff < SMOOTH_DIM_DELAY_PREV_BRIGHTNESS_MS) {
		panel_info("diff with prev brt: %lld / %d, set 8frame\n", diff, SMOOTH_DIM_DELAY_PREV_BRIGHTNESS_MS);
		goto exit;
	}

	panel_dbg("time %lld / %d last %lld / %d, set 32frame\n",
		ktime_to_ms(ktime_sub(now, panel->ktime_panel_disp_on)),
		SMOOTH_DIM_DELAY_AFTER_DISPLAYON_MS,
		ktime_to_ms(ktime_sub(now, last_time)),
		SMOOTH_DIM_DELAY_PREV_BRIGHTNESS_MS);

	ret = true;
exit:
	last_time = now;
	return ret;
}



int lx83805_get_cell_id(struct panel_device *panel, void *buf)
{
	u8 cell_id[11] = { 0, };
	struct panel_info *panel_data;

	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	panel_resource_copy(panel, cell_id, "cell_id");

	snprintf(buf, PAGE_SIZE, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
		cell_id[0], cell_id[1], cell_id[2], cell_id[3], cell_id[4],
		cell_id[5], cell_id[6], cell_id[7], cell_id[8], cell_id[9], cell_id[10]);
	return 0;
}

int lx83805_get_manufacture_date(struct panel_device *panel, void *buf)
{
	u8 date[11] = { 0, };
	u16 year;
	u8 month, day, hour, min, sec;
	u16 ms;

	struct panel_info *panel_data;

	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	panel_resource_copy(panel, date, "cell_id");

	year = ((date[0] & 0xF0) >> 4) + 2011;
	month = date[0] & 0xF;
	day = date[1] & 0x1F;
	hour = date[2] & 0x1F;
	min = date[3] & 0x3F;
	sec = date[4];
	ms = (date[5] << 8) | date[6];

	snprintf(buf, PAGE_SIZE, "%04d, %02d, %02d, %02d:%02d:%02d.%04d\n", year, month, day, hour, min, sec, ms);

	return 0;
}

int lx83805_get_octa_id(struct panel_device *panel, void *buf)
{
	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	snprintf(buf, PAGE_SIZE, "00000000000000000000000\n"); // to make total 23digit

	return 0;
}

int lx83806_get_manufacture_date(struct panel_device *panel, void *buf)
{
	u8 date[11] = { 0, };
	u16 year;
	u8 month, day, hour, min, sec;
	u16 ms;
	struct panel_info *panel_data;

	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_data = &panel->panel_data;
	panel_resource_copy(panel, date, "cell_id");

	year = ((date[4] & 0xF0) >> 4) + 2011;
	month = date[4] & 0xF;
	day = date[5] & 0x1F;
	hour = date[6] & 0x1F;
	min = date[7] & 0x3F;
	sec = date[8];
	ms = (date[9] << 8) | date[10];

	snprintf(buf, PAGE_SIZE, "%04d, %02d, %02d, %02d:%02d:%02d.%04d\n", year, month, day, hour, min, sec, ms);

	return 0;
}

int lx83806_get_cell_id(struct panel_device *panel, void *buf)
{
	u8 cell_id[11] = { 0, };
	struct panel_info *panel_data;

	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_data = &panel->panel_data;
	panel_resource_copy(panel, cell_id, "cell_id");

	snprintf(buf, PAGE_SIZE, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
				cell_id[4],	cell_id[5],	cell_id[6], cell_id[7], cell_id[8], cell_id[9], cell_id[10],
				cell_id[0], cell_id[1], cell_id[2], cell_id[3]);
	return 0;
}

int lx83807_get_cell_id(struct panel_device *panel, void *buf)
{
	u8 cell_id[20] = { 0, };
	struct panel_info *panel_data;

	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_data = &panel->panel_data;
	panel_resource_copy(panel, cell_id, "csot_cell_id");

	snprintf(buf, PAGE_SIZE, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
				cell_id[2],	cell_id[3],	cell_id[4], cell_id[5], cell_id[6], cell_id[7], cell_id[8],
				cell_id[9], cell_id[10], cell_id[11], cell_id[12]);
	return 0;
}

int lx83807_get_manufacture_date(struct panel_device *panel, void *buf)
{
	u8 date[20] = { 0, };
	u16 year;
	u8 month, day;
	struct panel_info *panel_data;

	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_data = &panel->panel_data;
	panel_resource_copy(panel, date, "csot_cell_id");

	year = ((date[14] & 0x0F) * 10) + (date[15] & 0x0F) + 2000;
	month = ((date[16] & 0x0F) * 10) + (date[17] & 0x0F);
	day = ((date[18] & 0x0F) * 10) + (date[19] & 0x0F);

	snprintf(buf, PAGE_SIZE, "%04d, %02d, %02d\n", year, month, day);

	return 0;
}

int lx83806_get_octa_id(struct panel_device *panel, void *buf)
{
	u8 octa_id_hex[16] = { 0, };
	u8 octa_id_ascii[16] = { 0, };
	struct panel_info *panel_data;
	int len = 0, i = 0;
	bool cell_id_exist = true;

	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_data = &panel->panel_data;
	panel_resource_copy(panel, octa_id_hex, "octa_id");

	for (i = 0; i < 16; i++) {
		octa_id_ascii[i] = isalnum(octa_id_hex[i]) ? octa_id_hex[i] : '\0';
		panel_dbg("%x -> %c\n", octa_id_hex[i], octa_id_ascii[i]);
		if (octa_id_ascii[i] == '\0') {
			cell_id_exist = false;
			break;
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "0000000"); // to make total 23digit
	if (cell_id_exist) {
		for (i = 0; i < 16; i++)
			len += snprintf(buf + len, PAGE_SIZE - len, "%c", octa_id_ascii[i]);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	return 0;
}

int lx8380x_get_manufacture_code(struct panel_device *panel, void *buf)
{

	u8 code[10] = { 0, };
	struct panel_info *panel_data;

	if (panel == NULL) {
		panel_err("panel is null\n");
		return -EINVAL;
	}
	panel_data = &panel->panel_data;

	panel_resource_copy(panel, code, "chip_id");
	snprintf(buf, PAGE_SIZE, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
		code[0], code[1], code[2], code[3], code[4],
		code[5], code[6], code[7], code[8], code[9]);

	return 0;
}

int lx8380x_maptbl_getidx_tset(struct maptbl *tbl)
{
	struct panel_device *panel;
	struct panel_info *panel_data;
	int row;

	if (!tbl || !tbl->pdata)
		return -EINVAL;

	panel = (struct panel_device *)tbl->pdata;
	if (unlikely(!panel))
		return -EINVAL;

	panel_data = &panel->panel_data;

	if (panel_data->props.temperature >= 1)
		row = GE_TEMP_1;
	else if (panel_data->props.temperature == 0)
		row = EQ_TEMP_0;
	else if ((panel_data->props.temperature <= -1) && (panel_data->props.temperature >= -14))
		row = LE_TEMP_MINUS1;
	else
		row = LE_TEMP_MINUS15;
	panel_info("tset %d\n", row);
	return maptbl_index(tbl, 0, row, 0);
}

int lx8380x_dump_show_error_flag(struct dumpinfo *dump)
{
	int ret;
	u8 error_flag[LX8380X_ERROR_FLAG_LEN] = { 0, };
	struct resinfo *res;

	if (!dump)
		return -EINVAL;

	res = dump->res;
	if (!is_valid_resource(res))
		return -EINVAL;

	if (!is_resource_initialized(res))
		return -EINVAL;

	if (!res || ARRAY_SIZE(error_flag) != res->dlen) {
		panel_err("invalid resource\n");
		return -EINVAL;
	}

	ret = copy_resource(error_flag, res);
	if (unlikely(ret < 0)) {
		panel_err("failed to copy err_fg resource\n");
		return -EINVAL;
	}
	return oled_dump_show_expects(dump);
}

bool lx8380x_cond_is_normal_to_hbm(struct panel_device *panel)
{
	struct panel_info *panel_data;
	struct panel_bl_device *panel_bl;
	bool ret = false;
	panel_data = &panel->panel_data;
	panel_bl = &panel->panel_bl;

	if (!is_hbm_brightness(panel_bl, panel_bl->props.prev_brightness) &&
		is_hbm_brightness(panel_bl, panel_bl->props.brightness))
		ret = true;
	return ret;
}


bool lx8380x_cond_is_mtp_valid(struct panel_device *panel)
{
	struct panel_info *panel_data;
	panel_data = &panel->panel_data;
	if (panel_data->props.is_valid_mtp == LX8380X_MTP_VALUE_IS_VALID)
		return true;
	return false;
}

#ifdef CONFIG_USDM_PANEL_MAFPC
int lx8380x_maptbl_getidx_mafpc_enable(struct maptbl *tbl)
{
	struct panel_device *panel;
	struct mafpc_device *mafpc;

	if (!tbl || !tbl->pdata)
		return -EINVAL;

	panel = tbl->pdata;

	mafpc = get_mafpc_device(panel);
	if (mafpc == NULL) {
		panel_err("failed to get mafpc device\n");
		return -EINVAL;
	}

	return maptbl_index(tbl, 0, !!mafpc->enable, 0);
}

void lx8380x_maptbl_copy_mafpc_enable(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	struct mafpc_device *mafpc;
	struct panel_info *panel_data;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;
	panel_data = &panel->panel_data;

	mafpc = get_mafpc_device(panel);
	if (mafpc == NULL) {
		panel_err("failed to get mafpc device\n");
		return;
	}

	/* update packet data(dst array buffer) if mafpc is disabled */
	if (!mafpc->enable) {
		oled_maptbl_copy_default(tbl, dst);
		return;
	}

	/*
	 * update packet data(dst array buffer) if mafpc is enabled and
	 * UPDATED_FROM_SVC and UPDATED_TO_DEV flag is set
	 */
	if ((mafpc->written & MAFPC_UPDATED_FROM_SVC) &&
		(mafpc->written & MAFPC_UPDATED_TO_DEV)) {
		oled_maptbl_copy_default(tbl, dst);
		return;
	}
}

void lx8380x_maptbl_copy_mafpc_ctrl(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	struct mafpc_device *mafpc;
	struct panel_info *panel_data;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;
	panel_data = &panel->panel_data;

	mafpc = get_mafpc_device(panel);
	if (mafpc == NULL) {
		panel_err("failed to get mafpc device\n");
		return;
	}

	panel_info("MCD:ABC:enabled: %x, written: %x, id[2]: %x\n",
		mafpc->enable, mafpc->written, panel_data->id[2]);

	if (mafpc->enable &&
		(mafpc->written & MAFPC_UPDATED_FROM_SVC) &&
		(mafpc->written & MAFPC_UPDATED_TO_DEV)) {
		memcpy(dst, mafpc->ctrl_cmd, mafpc->ctrl_cmd_len);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dst, mafpc->ctrl_cmd_len, false);
	}

	return;
}

static int get_mafpc_scale_index(struct mafpc_device *mafpc, struct panel_device *panel)
{
	struct panel_bl_device *panel_bl;
	int br_index, index = 0;

	if (!mafpc || !panel)
		return -EINVAL;

	panel_bl = &panel->panel_bl;
	if (!mafpc->scale_buf || !mafpc->scale_map_br_tbl)  {
		panel_err("mafpc img buf is null\n");
		return -EINVAL;
	}

	br_index = panel_bl->props.brightness;
	if (br_index >= mafpc->scale_map_br_tbl_len)
		br_index = mafpc->scale_map_br_tbl_len - 1;

	index = mafpc->scale_map_br_tbl[br_index];
	if (index < 0) {
		panel_err("mfapc invalid scale index : %d\n", br_index);
		return -EINVAL;
	}

	return index;
}

void lx8380x_maptbl_copy_mafpc_scale(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	struct mafpc_device *mafpc;
	int row, index;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;

	mafpc = get_mafpc_device(panel);
	if (mafpc == NULL) {
		panel_err("failed to get mafpc device\n");
		return;
	}

	if (!mafpc->scale_buf || !mafpc->scale_map_br_tbl)  {
		panel_err("mafpc img buf is null\n");
		return;
	}

	index = get_mafpc_scale_index(mafpc, panel);
	if (index < 0) {
		panel_err("mfapc invalid scale index : %d\n", index);
		return;
	}

	if (index >= LX8380X_MAFPC_SCALE_MAX)
		index = LX8380X_MAFPC_SCALE_MAX - 1;

	row = index * 3;
	memcpy(dst, mafpc->scale_buf + row, 3);

	panel_info("idx: %d, %x:%x:%x\n",
			index, dst[0], dst[1], dst[2]);

	return;
}

void lx8380x_maptbl_copy_mafpc_img(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	struct mafpc_device *mafpc;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;

	mafpc = get_mafpc_device(panel);
	if (mafpc == NULL) {
		panel_err("failed to get mafpc device\n");
		return;
	}

	if (!mafpc->comp_img_buf)  {
		panel_err("mafpc img buf is null\n");
		return;
	}

	memcpy(dst, mafpc->comp_img_buf, mafpc->comp_img_len);

	panel_info("len:%d, img:%x %x %x ~ %x %x %x\n", mafpc->comp_img_len,
		dst[0], dst[1], dst[2],
		dst[mafpc->comp_img_len - 3], dst[mafpc->comp_img_len - 2], dst[mafpc->comp_img_len - 1]);

	return;
}

#endif

struct pnobj_func lx8380x_function_table[MAX_LX8380X_FUNCTION] = {
	[LX8380X_MAPTBL_INIT_GAMMA_MODE2_BRT] = __PNOBJ_FUNC_INITIALIZER(LX8380X_MAPTBL_INIT_GAMMA_MODE2_BRT, lx8380x_maptbl_init_gamma_mode2_brt),
	[LX8380X_MAPTBL_GETIDX_ACL_CONTROL] = __PNOBJ_FUNC_INITIALIZER(LX8380X_MAPTBL_GETIDX_ACL_CONTROL, lx8380x_maptbl_getidx_acl_control),
	[LX8380X_MAPTBL_GETIDX_TSET] = __PNOBJ_FUNC_INITIALIZER(LX8380X_MAPTBL_GETIDX_TSET, lx8380x_maptbl_getidx_tset),
	[LX8380X_DUMP_SHOW_ERROR_FLAG] = __PNOBJ_FUNC_INITIALIZER(LX8380X_DUMP_SHOW_ERROR_FLAG, lx8380x_dump_show_error_flag),
#ifdef CONFIG_USDM_PANEL_MAFPC
	[LX8380X_MAPTBL_GETIDX_MAFPC_ENABLE] = __PNOBJ_FUNC_INITIALIZER(LX8380X_MAPTBL_GETIDX_MAFPC_ENABLE, lx8380x_maptbl_getidx_mafpc_enable),
	[LX8380X_MAPTBL_COPY_MAFPC_ENABLE] = __PNOBJ_FUNC_INITIALIZER(LX8380X_MAPTBL_COPY_MAFPC_ENABLE, lx8380x_maptbl_copy_mafpc_enable),
	[LX8380X_MAPTBL_COPY_MAFPC_CTRL] = __PNOBJ_FUNC_INITIALIZER(LX8380X_MAPTBL_COPY_MAFPC_CTRL, lx8380x_maptbl_copy_mafpc_ctrl),
	[LX8380X_MAPTBL_COPY_MAFPC_SCALE] = __PNOBJ_FUNC_INITIALIZER(LX8380X_MAPTBL_COPY_MAFPC_SCALE, lx8380x_maptbl_copy_mafpc_scale),
	[LX8380X_MAPTBL_COPY_MAFPC_IMG] = __PNOBJ_FUNC_INITIALIZER(LX8380X_MAPTBL_COPY_MAFPC_IMG, lx8380x_maptbl_copy_mafpc_img),
#endif
	[LX8380X_COND_IS_NORMAL_TO_HBM] = __PNOBJ_FUNC_INITIALIZER(LX8380X_COND_IS_NORMAL_TO_HBM, lx8380x_cond_is_normal_to_hbm),
	[LX8380X_COND_IS_MTP_VALID] = __PNOBJ_FUNC_INITIALIZER(LX8380X_COND_IS_MTP_VALID, lx8380x_cond_is_mtp_valid),
	[LX8380X_COND_IS_SMOOTH_DIMMING_AVAILABLE] = __PNOBJ_FUNC_INITIALIZER(LX8380X_COND_IS_SMOOTH_DIMMING_AVAILABLE, lx8380x_cond_is_smooth_dimming_available),
};

int lx8380x_init(void)
{
	static bool once;
	int ret;

	if (once)
		return 0;

	ret = panel_function_insert_array(lx8380x_function_table,
			ARRAY_SIZE(lx8380x_function_table));
	if (ret < 0)
		panel_err("failed to insert lx8380x_function_table\n");

	once = true;

	return 0;
}

MODULE_DESCRIPTION("Samsung Mobile Panel Driver");
MODULE_LICENSE("GPL");
