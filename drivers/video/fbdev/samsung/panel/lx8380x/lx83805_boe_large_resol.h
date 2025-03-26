/*
 * linux/drivers/video/fbdev/exynos/panel/sw83109/sw83109_m54x_resol.h
 *
 * Header file for Panel Driver
 *
 * Copyright (c) 2019 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LX83805_BOE_LARGE_RESOL_H__
#define __LX83805_BOE_LARGE_RESOL_H__

#include <dt-bindings/display/panel-display.h>
#include "../panel.h"
#include "lx8380x.h"
#include "lx8380x_dimming.h"

struct panel_vrr lx83805_boe_large_default_panel_vrr[] = {
 	[LX8380X_VRR_60HS] = {
		.fps = 60,
		.te_sw_skip_count = 0,
		.te_hw_skip_count = 0,
		.mode = VRR_HS_MODE,
	},
 };

static struct panel_vrr *lx83805_boe_large_default_vrrtbl[] = {
 	&lx83805_boe_large_default_panel_vrr[LX8380X_VRR_60HS],
 };

static struct panel_resol lx83805_boe_large_default_resol[] = {
	[LX8380X_RESOL_480x480] = {
		.w = 480,
		.h = 480,
		.comp_type = PN_COMP_TYPE_NONE,
		.available_vrr = lx83805_boe_large_default_vrrtbl,
		.nr_available_vrr = ARRAY_SIZE(lx83805_boe_large_default_vrrtbl),
	},
};

#if defined(CONFIG_USDM_PANEL_DISPLAY_MODE)
static struct common_panel_display_mode lx83805_boe_large_display_mode[] = {
 	[LX8380X_LARGE_DISPLAY_MODE_480x480_60HS] = {
		.name = PANEL_DISPLAY_MODE_480x480_60HS,
		.resol = &lx83805_boe_large_default_resol[LX8380X_RESOL_480x480],
		.vrr = &lx83805_boe_large_default_panel_vrr[LX8380X_VRR_60HS],
	},
 };

static struct common_panel_display_mode *lx83805_boe_large_display_mode_array[] = {
 	[LX8380X_LARGE_DISPLAY_MODE_480x480_60HS] = &lx83805_boe_large_display_mode[LX8380X_LARGE_DISPLAY_MODE_480x480_60HS],
 };

static struct common_panel_display_modes lx83805_boe_large_display_modes = {
	.num_modes = ARRAY_SIZE(lx83805_boe_large_display_mode),
	.modes = (struct common_panel_display_mode **)&lx83805_boe_large_display_mode_array,
};
#endif /* CONFIG_USDM_PANEL_DISPLAY_MODE */
#endif /* __LX83805_BOE_LARGE_RESOL_H__ */
