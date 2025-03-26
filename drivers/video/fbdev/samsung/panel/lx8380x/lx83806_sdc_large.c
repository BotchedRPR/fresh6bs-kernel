/*
 * linux/drivers/video/fbdev/exynos/panel/tft_common/tft_common.c
 *
 * TFT_COMMON Dimming Driver
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
#include "../panel_debug.h"
#include "lx8380x.h"
#include "lx83806_sdc_large.h"
#include "lx83806_sdc_large_panel.h"

bool is_readed_offset_same(u8* val1, u8* val2, u8* val3)
{
	if (!val1 || !val2 || !val3)
		return false;
	if ((val1[0] == val2[0]) && (val1[1] == val2[1])) {
		if ((val1[0] == val3[0]) && (val1[1] == val3[1])) {
			return true;
		}
	} else {
		panel_err("1:%x %x, 2:%x %x, 3:%x %x\n",
			val1[0], val1[1],
			val2[0], val2[1],
			val3[0], val3[1]);
	}
	return false;
}


int lx83806_sdc_large_maptbl_init_red_gamma_5nit(struct maptbl *tbl)
{
	struct panel_info *panel_data;
	struct panel_device *panel;
	u8 mtp_red_try1[2], mtp_red_try2[2], mtp_red_try3[2];
	int ret = 0;

	panel_info("++\n");
	if (tbl == NULL) {
		panel_err("maptbl is null\n");
		ret = -EINVAL;
		goto exit;
	}

	if (tbl->pdata == NULL) {
		panel_err("pdata is null\n");
		ret = -EINVAL;
		goto exit;
	}

	panel = tbl->pdata;
	panel_data = &panel->panel_data;

	if (panel_data->props.is_valid_mtp == LX8380X_MTP_VALUE_IS_MISMATCH) {
		panel_err("mtp value is already invalid(%d)\n", panel_data->props.is_valid_mtp);
		ret = 0;
		goto exit;
	}
	panel_resource_copy(panel, mtp_red_try1, "red_gamma_5nit_try1");
	panel_resource_copy(panel, mtp_red_try2, "red_gamma_5nit_try2");
	panel_resource_copy(panel, mtp_red_try3, "red_gamma_5nit_try3");

	if (is_readed_offset_same(mtp_red_try1, mtp_red_try2, mtp_red_try3))
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_VALID;
	else
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_MISMATCH;

exit:
	return ret;
}


void lx83806_sdc_large_maptbl_copy_red_gamma_5nit(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	u8 mtp_red[2] = { 0x00, };
	u16 red_default = 0;
	u16 red_offset[5] = { 0, };
	int offset_result = 0;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;
	panel_resource_copy(panel, mtp_red, "red_gamma_5nit_try1");

	red_default = mtp_red[0] & 0x00F0;
	red_default = (red_default << 4) | (0x00FF & mtp_red[1]);
	panel_info("0x%02X 0x%02X -> 0x%04X(%d)\n", mtp_red[0], mtp_red[1], red_default, red_default);

	offset_result = red_default - LX83806_SDC_LARGE_RED_GAMMA_5NIT_DEFAULT_1;
	if (offset_result < 0) {
		panel_warn("offset1 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	red_offset[0] = offset_result;

	offset_result = red_default - LX83806_SDC_LARGE_RED_GAMMA_5NIT_DEFAULT_2;
	if (offset_result < 0) {
		panel_warn("offset2 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	red_offset[1] = offset_result;

	offset_result = red_default - LX83806_SDC_LARGE_RED_GAMMA_5NIT_DEFAULT_3;
	if (offset_result < 0) {
		panel_warn("offset3 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	red_offset[2] = offset_result;

	offset_result = red_default - LX83806_SDC_LARGE_RED_GAMMA_5NIT_DEFAULT_4;
	if (offset_result < 0) {
		panel_warn("offset4 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	red_offset[3] = offset_result;

	offset_result = red_default - LX83806_SDC_LARGE_RED_GAMMA_5NIT_DEFAULT_5;
	if (offset_result < 0) {
		panel_warn("offset5 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	red_offset[4] = offset_result;

	panel_info("offsets 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n",
		red_offset[0], red_offset[1], red_offset[2], red_offset[3], red_offset[4]);

	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x00;
	dst[3] = (red_offset[0] >> 8) & 0x000F ;
	dst[4] = 0x00;
	dst[5] = (red_offset[0] & 0x00FF);
	dst[6] = ((red_offset[1] >> 4) & 0x00F0) | ((red_offset[2] >> 8) & 0x000F);
	dst[7] = (red_offset[1] & 0x00FF);
	dst[8] = (red_offset[2] & 0x00FF);
	dst[9] = ((red_offset[3] >> 4) & 0x00F0) | ((red_offset[4] >> 8) & 0x000F);
	dst[10] = (red_offset[3] & 0x00FF);
	dst[11] = (red_offset[4] & 0x00FF);

	return;

}

int lx83806_sdc_large_maptbl_init_green_gamma_5nit(struct maptbl *tbl)
{
	struct panel_info *panel_data;
	struct panel_device *panel;
	u8 mtp_green_try1[2], mtp_green_try2[2], mtp_green_try3[2];
	int ret = 0;

	panel_info("++\n");
	if (tbl == NULL) {
		panel_err("maptbl is null\n");
		ret = -EINVAL;
		goto exit;
	}

	if (tbl->pdata == NULL) {
		panel_err("pdata is null\n");
		ret = -EINVAL;
		goto exit;
	}

	panel = tbl->pdata;
	panel_data = &panel->panel_data;

	if (panel_data->props.is_valid_mtp == LX8380X_MTP_VALUE_IS_MISMATCH) {
		panel_err("mtp value is already invalid(%d)\n", panel_data->props.is_valid_mtp);
		ret = 0;
		goto exit;
	}
	panel_resource_copy(panel, mtp_green_try1, "green_gamma_5nit_try1");
	panel_resource_copy(panel, mtp_green_try2, "green_gamma_5nit_try2");
	panel_resource_copy(panel, mtp_green_try3, "green_gamma_5nit_try3");

	if (is_readed_offset_same(mtp_green_try1, mtp_green_try2, mtp_green_try3))
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_VALID;
	else
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_MISMATCH;

exit:
	return ret;
}


void lx83806_sdc_large_maptbl_copy_green_gamma_5nit(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	u8 mtp_green[2] = { 0x00, };
	u16 green_default = 0;
	u16 green_offset[5] = { 0, };
	int offset_result = 0;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;
	panel_resource_copy(panel, mtp_green, "green_gamma_5nit_try1");

	green_default = mtp_green[0] & 0x00F0;
	green_default = (green_default << 4) | (0x00FF & mtp_green[1]);
	panel_info("0x%02X 0x%02X -> 0x%04X(%d)\n", mtp_green[0], mtp_green[1], green_default, green_default);

	offset_result = green_default - LX83806_SDC_LARGE_GREEN_GAMMA_5NIT_DEFAULT_1;
	if (offset_result < 0) {
		panel_warn("offset1 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	green_offset[0] = offset_result;

	offset_result = green_default - LX83806_SDC_LARGE_GREEN_GAMMA_5NIT_DEFAULT_2;
	if (offset_result < 0) {
		panel_warn("offset2 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	green_offset[1] = offset_result;

	offset_result = green_default - LX83806_SDC_LARGE_GREEN_GAMMA_5NIT_DEFAULT_3;
	if (offset_result < 0) {
		panel_warn("offset3 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	green_offset[2] = offset_result;

	offset_result = green_default - LX83806_SDC_LARGE_GREEN_GAMMA_5NIT_DEFAULT_4;
	if (offset_result < 0) {
		panel_warn("offset4 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	green_offset[3] = offset_result;

	offset_result = green_default - LX83806_SDC_LARGE_GREEN_GAMMA_5NIT_DEFAULT_5;
	if (offset_result < 0) {
		panel_warn("offset5 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	green_offset[4] = offset_result;

	panel_info("offsets 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n",
		green_offset[0], green_offset[1], green_offset[2], green_offset[3], green_offset[4]);

	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x00;
	dst[3] = (green_offset[0] >> 8) & 0x000F ;
	dst[4] = 0x00;
	dst[5] = (green_offset[0] & 0x00FF);
	dst[6] = ((green_offset[1] >> 4) & 0x00F0) | ((green_offset[2] >> 8) & 0x000F);
	dst[7] = (green_offset[1] & 0x00FF);
	dst[8] = (green_offset[2] & 0x00FF);
	dst[9] = ((green_offset[3] >> 4) & 0x00F0) | ((green_offset[4] >> 8) & 0x000F);
	dst[10] = (green_offset[3] & 0x00FF);
	dst[11] = (green_offset[4] & 0x00FF);

	return;

}

int lx83806_sdc_large_maptbl_init_blue_gamma_5nit(struct maptbl *tbl)
{
	struct panel_info *panel_data;
	struct panel_device *panel;
	u8 mtp_blue_try1[2], mtp_blue_try2[2], mtp_blue_try3[2];
	int ret = 0;

	panel_info("++\n");
	if (tbl == NULL) {
		panel_err("maptbl is null\n");
		ret = -EINVAL;
		goto exit;
	}

	if (tbl->pdata == NULL) {
		panel_err("pdata is null\n");
		ret = -EINVAL;
		goto exit;
	}

	panel = tbl->pdata;
	panel_data = &panel->panel_data;

	if (panel_data->props.is_valid_mtp == LX8380X_MTP_VALUE_IS_MISMATCH) {
		panel_err("mtp value is already invalid(%d)\n", panel_data->props.is_valid_mtp);
		ret = 0;
		goto exit;
	}
	panel_resource_copy(panel, mtp_blue_try1, "blue_gamma_5nit_try1");
	panel_resource_copy(panel, mtp_blue_try2, "blue_gamma_5nit_try2");
	panel_resource_copy(panel, mtp_blue_try3, "blue_gamma_5nit_try3");

	if (is_readed_offset_same(mtp_blue_try1, mtp_blue_try2, mtp_blue_try3))
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_VALID;
	else
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_MISMATCH;

exit:
	return ret;
}


void lx83806_sdc_large_maptbl_copy_blue_gamma_5nit(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	u8 mtp_blue[2] = { 0x00, };
	u16 blue_default = 0;
	u16 blue_offset[5] = { 0, };
	int offset_result = 0;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;
	panel_resource_copy(panel, mtp_blue, "blue_gamma_5nit_try1");

	blue_default = mtp_blue[0] & 0x00F0;
	blue_default = (blue_default << 4) | (0x00FF & mtp_blue[1]);
	panel_info("0x%02X 0x%02X -> 0x%04X(%d)\n", mtp_blue[0], mtp_blue[1], blue_default, blue_default);

	offset_result = blue_default - LX83806_SDC_LARGE_BLUE_GAMMA_5NIT_DEFAULT_1;
	if (offset_result < 0) {
		panel_warn("offset1 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	blue_offset[0] = offset_result;

	offset_result = blue_default - LX83806_SDC_LARGE_BLUE_GAMMA_5NIT_DEFAULT_2;
	if (offset_result < 0) {
		panel_warn("offset2 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	blue_offset[1] = offset_result;

	offset_result = blue_default - LX83806_SDC_LARGE_BLUE_GAMMA_5NIT_DEFAULT_3;
	if (offset_result < 0) {
		panel_warn("offset3 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	blue_offset[2] = offset_result;

	offset_result = blue_default - LX83806_SDC_LARGE_BLUE_GAMMA_5NIT_DEFAULT_4;
	if (offset_result < 0) {
		panel_warn("offset4 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	blue_offset[3] = offset_result;

	offset_result = blue_default - LX83806_SDC_LARGE_BLUE_GAMMA_5NIT_DEFAULT_5;
	if (offset_result < 0) {
		panel_warn("offset5 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	blue_offset[4] = offset_result;

	panel_info("offsets 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n",
		blue_offset[0], blue_offset[1], blue_offset[2], blue_offset[3], blue_offset[4]);

	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x00;
	dst[3] = (blue_offset[0] >> 8) & 0x000F ;
	dst[4] = 0x00;
	dst[5] = (blue_offset[0] & 0x00FF);
	dst[6] = ((blue_offset[1] >> 4) & 0x00F0) | ((blue_offset[2] >> 8) & 0x000F);
	dst[7] = (blue_offset[1] & 0x00FF);
	dst[8] = (blue_offset[2] & 0x00FF);
	dst[9] = ((blue_offset[3] >> 4) & 0x00F0) | ((blue_offset[4] >> 8) & 0x000F);
	dst[10] = (blue_offset[3] & 0x00FF);
	dst[11] = (blue_offset[4] & 0x00FF);

	return;

}

int lx83806_sdc_large_maptbl_init_red_gamma_600nit(struct maptbl *tbl)
{
	struct panel_info *panel_data;
	struct panel_device *panel;
	u8 mtp_red_try1[2], mtp_red_try2[2], mtp_red_try3[2];
	int ret = 0;

	panel_info("++\n");
	if (tbl == NULL) {
		panel_err("maptbl is null\n");
		ret = -EINVAL;
		goto exit;
	}

	if (tbl->pdata == NULL) {
		panel_err("pdata is null\n");
		ret = -EINVAL;
		goto exit;
	}

	panel = tbl->pdata;
	panel_data = &panel->panel_data;

	if (panel_data->props.is_valid_mtp == LX8380X_MTP_VALUE_IS_MISMATCH) {
		panel_err("mtp value is already invalid(%d)\n", panel_data->props.is_valid_mtp);
		ret = 0;
		goto exit;
	}
	panel_resource_copy(panel, mtp_red_try1, "red_gamma_600nit_try1");
	panel_resource_copy(panel, mtp_red_try2, "red_gamma_600nit_try2");
	panel_resource_copy(panel, mtp_red_try3, "red_gamma_600nit_try3");

	if (is_readed_offset_same(mtp_red_try1, mtp_red_try2, mtp_red_try3))
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_VALID;
	else
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_MISMATCH;

exit:
	return ret;
}

void lx83806_sdc_large_maptbl_copy_red_gamma_600nit(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	u8 mtp_red[2] = { 0x00, };
	u16 red_default = 0;
	u16 red_offset = 0;
	int offset_result = 0;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;
	panel_resource_copy(panel, mtp_red, "red_gamma_600nit_try1");

	red_default = mtp_red[0] & 0x00F0;
	red_default = (red_default << 4) | (0x00FF & mtp_red[1]);
	panel_info("0x%02X 0x%02X -> 0x%04X(%d)\n", mtp_red[0], mtp_red[1], red_default, red_default);

	offset_result = red_default - LX83806_SDC_LARGE_RED_GAMMA_600NIT_DEFAULT;
	if (offset_result < 0) {
		panel_warn("offset1 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	red_offset = offset_result;
	panel_info("new reds 0x%04x\n", red_offset);

	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x00;
	dst[3] = (red_offset >> 8) & 0x000F ;
	dst[4] = 0x00;
	dst[5] = (red_offset & 0x00FF);

	return;

}

int lx83806_sdc_large_maptbl_init_green_gamma_600nit(struct maptbl *tbl)
{
	struct panel_info *panel_data;
	struct panel_device *panel;
	u8 mtp_green_try1[2], mtp_green_try2[2], mtp_green_try3[2];
	int ret = 0;

	panel_info("++\n");
	if (tbl == NULL) {
		panel_err("maptbl is null\n");
		ret = -EINVAL;
		goto exit;
	}

	if (tbl->pdata == NULL) {
		panel_err("pdata is null\n");
		ret = -EINVAL;
		goto exit;
	}

	panel = tbl->pdata;
	panel_data = &panel->panel_data;

	if (panel_data->props.is_valid_mtp == LX8380X_MTP_VALUE_IS_MISMATCH) {
		panel_err("mtp value is already invalid(%d)\n", panel_data->props.is_valid_mtp);
		ret = 0;
		goto exit;
	}
	panel_resource_copy(panel, mtp_green_try1, "green_gamma_600nit_try1");
	panel_resource_copy(panel, mtp_green_try2, "green_gamma_600nit_try2");
	panel_resource_copy(panel, mtp_green_try3, "green_gamma_600nit_try3");

	if (is_readed_offset_same(mtp_green_try1, mtp_green_try2, mtp_green_try3))
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_VALID;
	else
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_MISMATCH;

exit:
	return ret;
}

void lx83806_sdc_large_maptbl_copy_green_gamma_600nit(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	u8 mtp_green[2] = { 0x00, };
	u16 green_default = 0;
	u16 green_offset = 0;
	int offset_result = 0;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;
	panel_resource_copy(panel, mtp_green, "green_gamma_600nit_try1");

	green_default = mtp_green[0] & 0x00F0;
	green_default = (green_default << 4) | (0x00FF & mtp_green[1]);
	panel_info("0x%02X 0x%02X -> 0x%04X(%d)\n", mtp_green[0], mtp_green[1], green_default, green_default);

	offset_result = green_default - LX83806_SDC_LARGE_GREEN_GAMMA_600NIT_DEFAULT;
	if (offset_result < 0) {
		panel_warn("offset1 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	green_offset = offset_result;
	panel_info("new green 0x%04x\n", green_offset);

	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x00;
	dst[3] = (green_offset >> 8) & 0x000F ;
	dst[4] = 0x00;
	dst[5] = (green_offset & 0x00FF);

	return;

}

int lx83806_sdc_large_maptbl_init_blue_gamma_600nit(struct maptbl *tbl)
{
	struct panel_info *panel_data;
	struct panel_device *panel;
	u8 mtp_blue_try1[2], mtp_blue_try2[2], mtp_blue_try3[2];
	int ret = 0;

	panel_info("++\n");
	if (tbl == NULL) {
		panel_err("maptbl is null\n");
		ret = -EINVAL;
		goto exit;
	}

	if (tbl->pdata == NULL) {
		panel_err("pdata is null\n");
		ret = -EINVAL;
		goto exit;
	}

	panel = tbl->pdata;
	panel_data = &panel->panel_data;

	if (panel_data->props.is_valid_mtp == LX8380X_MTP_VALUE_IS_MISMATCH) {
		panel_err("mtp value is already invalid(%d)\n", panel_data->props.is_valid_mtp);
		ret = 0;
		goto exit;
	}
	panel_resource_copy(panel, mtp_blue_try1, "blue_gamma_600nit_try1");
	panel_resource_copy(panel, mtp_blue_try2, "blue_gamma_600nit_try2");
	panel_resource_copy(panel, mtp_blue_try3, "blue_gamma_600nit_try3");

	if (is_readed_offset_same(mtp_blue_try1, mtp_blue_try2, mtp_blue_try3))
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_VALID;
	else
		panel_data->props.is_valid_mtp = LX8380X_MTP_VALUE_IS_MISMATCH;

exit:
	return ret;
}

void lx83806_sdc_large_maptbl_copy_blue_gamma_600nit(struct maptbl *tbl, u8 *dst)
{
	struct panel_device *panel;
	u8 mtp_blue[2] = { 0x00, };
	u16 blue_default = 0;
	u16 blue_offset = 0;
	int offset_result = 0;

	if (!tbl || !tbl->pdata)
		return;

	panel = tbl->pdata;
	panel_resource_copy(panel, mtp_blue, "blue_gamma_600nit_try1");

	blue_default = mtp_blue[0] & 0x00F0;
	blue_default = (blue_default << 4) | (0x00FF & mtp_blue[1]);
	panel_info("0x%02X 0x%02X -> 0x%04X(%d)\n", mtp_blue[0], mtp_blue[1], blue_default, blue_default);

	offset_result = blue_default - LX83806_SDC_LARGE_BLUE_GAMMA_600NIT_DEFAULT;
	if (offset_result < 0) {
		panel_warn("offset1 is negative(%d -> 0)\n", offset_result);
		offset_result = 0;
	}
	blue_offset = offset_result;
	panel_info("new blues 0x%04x\n", blue_offset);

	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x00;
	dst[3] = (blue_offset >> 8) & 0x000F ;
	dst[4] = 0x00;
	dst[5] = (blue_offset & 0x00FF);

	return;

}

struct pnobj_func lx83806_sdc_large_function_table[MAX_LX83806_SDC_LARGE_FUNCTION] = {
	[LX83806_SDC_LARGE_MAPTBL_INIT_RED_GAMMA_5NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_INIT_RED_GAMMA_5NIT, lx83806_sdc_large_maptbl_init_red_gamma_5nit),
	[LX83806_SDC_LARGE_MAPTBL_INIT_GREEN_GAMMA_5NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_INIT_GREEN_GAMMA_5NIT, lx83806_sdc_large_maptbl_init_green_gamma_5nit),
	[LX83806_SDC_LARGE_MAPTBL_INIT_BLUE_GAMMA_5NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_INIT_BLUE_GAMMA_5NIT, lx83806_sdc_large_maptbl_init_blue_gamma_5nit),
	[LX83806_SDC_LARGE_MAPTBL_INIT_RED_GAMMA_600NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_INIT_RED_GAMMA_600NIT, lx83806_sdc_large_maptbl_init_red_gamma_600nit),
	[LX83806_SDC_LARGE_MAPTBL_INIT_GREEN_GAMMA_600NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_INIT_GREEN_GAMMA_600NIT, lx83806_sdc_large_maptbl_init_green_gamma_600nit),
	[LX83806_SDC_LARGE_MAPTBL_INIT_BLUE_GAMMA_600NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_INIT_BLUE_GAMMA_600NIT, lx83806_sdc_large_maptbl_init_blue_gamma_600nit),
	[LX83806_SDC_LARGE_MAPTBL_COPY_RED_GAMMA_5NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_COPY_RED_GAMMA_5NIT, lx83806_sdc_large_maptbl_copy_red_gamma_5nit),
	[LX83806_SDC_LARGE_MAPTBL_COPY_GREEN_GAMMA_5NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_COPY_GREEN_GAMMA_5NIT, lx83806_sdc_large_maptbl_copy_green_gamma_5nit),
	[LX83806_SDC_LARGE_MAPTBL_COPY_BLUE_GAMMA_5NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_COPY_BLUE_GAMMA_5NIT, lx83806_sdc_large_maptbl_copy_blue_gamma_5nit),
	[LX83806_SDC_LARGE_MAPTBL_COPY_RED_GAMMA_600NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_COPY_RED_GAMMA_600NIT, lx83806_sdc_large_maptbl_copy_red_gamma_600nit),
	[LX83806_SDC_LARGE_MAPTBL_COPY_GREEN_GAMMA_600NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_COPY_GREEN_GAMMA_600NIT, lx83806_sdc_large_maptbl_copy_green_gamma_600nit),
	[LX83806_SDC_LARGE_MAPTBL_COPY_BLUE_GAMMA_600NIT] = __PNOBJ_FUNC_INITIALIZER(LX83806_SDC_LARGE_MAPTBL_COPY_BLUE_GAMMA_600NIT, lx83806_sdc_large_maptbl_copy_blue_gamma_600nit),
};

static int __init lx83806_sdc_large_panel_init(void)
{
	lx8380x_init();
	panel_function_insert_array(lx83806_sdc_large_function_table, ARRAY_SIZE(lx83806_sdc_large_function_table));
	register_common_panel(&lx83806_sdc_large_panel_info);

	return 0;
}

static void __exit lx83806_sdc_large_panel_exit(void)
{
	deregister_common_panel(&lx83806_sdc_large_panel_info);
}

module_init(lx83806_sdc_large_panel_init)
module_exit(lx83806_sdc_large_panel_exit)

MODULE_DESCRIPTION("Samsung Mobile Panel Driver");
MODULE_LICENSE("GPL");

