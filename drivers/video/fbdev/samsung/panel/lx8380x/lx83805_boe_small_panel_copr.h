/*
 * linux/drivers/video/fbdev/exynos/panel/lx8380x/lx83805_boe_small_panel_copr.h
 *
 * Header file for COPR Driver
 *
 * Copyright (c) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LX83805_BOE_SMALL_PANEL_COPR_H__
#define __LX83805_BOE_SMALL_PANEL_COPR_H__

#include "../panel.h"
#include "../copr.h"
#include "lx83805_boe_small_panel.h"

#define LX83805_BOE_SMALL_COPR_EN			(1)
#define LX83805_BOE_SMALL_COPR_PWR			(1)
#define LX83805_BOE_SMALL_COPR_MASK			(0)
#define LX83805_BOE_SMALL_COPR_ROI_CTRL		(3)
#define LX83805_BOE_SMALL_COPR_GAMMA_CTRL	(3)

#ifdef CONFIG_USDM_PANEL_WISE
#define LX83805_BOE_SMALL_COPR_ROI1_ER		(256)
#define LX83805_BOE_SMALL_COPR_ROI1_EG		(256)
#define LX83805_BOE_SMALL_COPR_ROI1_EB		(256)

#define LX83805_BOE_SMALL_COPR_ROI2_ER		(256)
#define LX83805_BOE_SMALL_COPR_ROI2_EG		(256)
#define LX83805_BOE_SMALL_COPR_ROI2_EB		(256)

#define LX83805_BOE_SMALL_COPR_ROI1_X_S		(264)
#define LX83805_BOE_SMALL_COPR_ROI1_Y_S		(367)
#define LX83805_BOE_SMALL_COPR_ROI1_X_E		(291)
#define LX83805_BOE_SMALL_COPR_ROI1_Y_E		(393)

#define LX83805_BOE_SMALL_COPR_ROI2_X_S		(250)
#define LX83805_BOE_SMALL_COPR_ROI2_Y_S		(353)
#define LX83805_BOE_SMALL_COPR_ROI2_X_E		(305)
#define LX83805_BOE_SMALL_COPR_ROI2_Y_E		(407)
#else
#define LX83805_BOE_SMALL_COPR_ROI1_ER		(256)
#define LX83805_BOE_SMALL_COPR_ROI1_EG		(256)
#define LX83805_BOE_SMALL_COPR_ROI1_EB		(256)

#define LX83805_BOE_SMALL_COPR_ROI2_ER		(256)
#define LX83805_BOE_SMALL_COPR_ROI2_EG		(256)
#define LX83805_BOE_SMALL_COPR_ROI2_EB		(256)

#define LX83805_BOE_SMALL_COPR_ROI1_X_S		(282)
#define LX83805_BOE_SMALL_COPR_ROI1_Y_S		(352)
#define LX83805_BOE_SMALL_COPR_ROI1_X_E		(309)
#define LX83805_BOE_SMALL_COPR_ROI1_Y_E		(378)

#define LX83805_BOE_SMALL_COPR_ROI2_X_S		(268)
#define LX83805_BOE_SMALL_COPR_ROI2_Y_S		(338)
#define LX83805_BOE_SMALL_COPR_ROI2_X_E		(323)
#define LX83805_BOE_SMALL_COPR_ROI2_Y_E		(392)
#endif

static struct pktinfo PKTINFO(lx83805_boe_small_test_a4_on);
static struct pktinfo PKTINFO(lx83805_boe_small_test_off);

/* ===================================================================================== */
/* ============================== [S6E3FAC MAPPING TABLE] ============================== */
/* ===================================================================================== */
static struct maptbl lx83805_boe_small_copr_maptbl[] = {
	[COPR_MAPTBL] = DEFINE_0D_MAPTBL(lx83805_boe_small_copr_table, &OLED_FUNC(OLED_MAPTBL_INIT_DEFAULT), NULL, &OLED_FUNC(OLED_MAPTBL_COPY_COPR)),
};

/* ===================================================================================== */
/* ============================== [S6E3FAC COMMAND TABLE] ============================== */
/* ===================================================================================== */
static u8 BOE_SMALL_COPR[] = {
	0xE1,
	0x03, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,
	0x01, 0x00, 0x01, 0x00, 0x01, 0x20, 0x01, 0xA5, 0x01, 0x3A,
	0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xDF, 0x01, 0xDF
};

static DEFINE_PKTUI(lx83805_boe_small_copr, &lx83805_boe_small_copr_maptbl[COPR_MAPTBL], 1);
static DEFINE_VARIABLE_PACKET(lx83805_boe_small_copr, DSI_PKT_TYPE_WR, BOE_SMALL_COPR, 0);

static void *lx83805_boe_small_set_copr_cmdtbl[] = {
	&PKTINFO(lx83805_boe_small_test_a4_on),
	&PKTINFO(lx83805_boe_small_copr),
	&PKTINFO(lx83805_boe_small_test_off),
};
static DEFINE_SEQINFO(lx83805_boe_small_set_copr_seq, lx83805_boe_small_set_copr_cmdtbl);

static void *lx83805_boe_small_get_copr_spi_cmdtbl[] = {
	&lx8380x_restbl[RES_COPR_SPI],
};

static void *lx83805_boe_small_get_copr_dsi_cmdtbl[] = {
	&PKTINFO(lx83805_boe_small_test_a4_on),
	&lx8380x_restbl[RES_COPR_DSI],
	&PKTINFO(lx83805_boe_small_test_off),
};

static struct seqinfo lx83805_boe_small_copr_seqtbl[] = {
	SEQINFO_INIT(COPR_SET_SEQ, lx83805_boe_small_set_copr_cmdtbl),
	SEQINFO_INIT(COPR_SPI_GET_SEQ, lx83805_boe_small_get_copr_spi_cmdtbl),
	SEQINFO_INIT(COPR_DSI_GET_SEQ, lx83805_boe_small_get_copr_dsi_cmdtbl),
};

static struct panel_copr_data lx83805_boe_small_copr_data = {
	.seqtbl = lx83805_boe_small_copr_seqtbl,
	.nr_seqtbl = ARRAY_SIZE(lx83805_boe_small_copr_seqtbl),
	.maptbl = (struct maptbl *)lx83805_boe_small_copr_maptbl,
	.nr_maptbl = (sizeof(lx83805_boe_small_copr_maptbl) / sizeof(struct maptbl)),
	.version = COPR_VER_0_1,
	.options = {
		.thread_on = false,
		.check_avg = false,
	},
	.reg.v0_1 = {
		.copr_en = LX83805_BOE_SMALL_COPR_EN,
		.copr_pwr = LX83805_BOE_SMALL_COPR_PWR,
		.copr_mask = LX83805_BOE_SMALL_COPR_MASK,
		.copr_roi_ctrl = LX83805_BOE_SMALL_COPR_ROI_CTRL,
		.copr_gamma_ctrl = LX83805_BOE_SMALL_COPR_GAMMA_CTRL,
		.roi = {
			[0] = {
				.roi_er = LX83805_BOE_SMALL_COPR_ROI1_ER, .roi_eg = LX83805_BOE_SMALL_COPR_ROI1_EG,
				.roi_eb = LX83805_BOE_SMALL_COPR_ROI1_EB,
				.roi_xs = LX83805_BOE_SMALL_COPR_ROI1_X_S, .roi_ys = LX83805_BOE_SMALL_COPR_ROI1_Y_S,
				.roi_xe = LX83805_BOE_SMALL_COPR_ROI1_X_E, .roi_ye = LX83805_BOE_SMALL_COPR_ROI1_Y_E,
			},
			[1] = {
				.roi_er = LX83805_BOE_SMALL_COPR_ROI2_ER, .roi_eg = LX83805_BOE_SMALL_COPR_ROI2_EG,
				.roi_eb = LX83805_BOE_SMALL_COPR_ROI2_EB,
				.roi_xs = LX83805_BOE_SMALL_COPR_ROI2_X_S, .roi_ys = LX83805_BOE_SMALL_COPR_ROI2_Y_S,
				.roi_xe = LX83805_BOE_SMALL_COPR_ROI2_X_E, .roi_ye = LX83805_BOE_SMALL_COPR_ROI2_Y_E,
			},
		},
	},
	.nr_roi = 2,
};

#endif /* __LX83805_BOE_SMALL_PANEL_COPR_H__ */
