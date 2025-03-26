/*
 * linux/drivers/video/fbdev/exynos/panel/s6e3fc3/lx83807_csot_small_aod_panel.h
 *
 * Header file for AOD Driver
 *
 * Copyright (c) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LX83807_CSOT_SMALL_AOD_PANEL_H__
#define __LX83807_CSOT_SMALL_AOD_PANEL_H__

#include "oled_common_aod.h"
#include "lx8380x_aod.h"

static struct maptbl lx83807_csot_small_aod_maptbl[] = {
};

static u8 LX83807_CSOT_SMALL_AOD_TEST_A4_ON[] = { 0xB0, 0xA4 };
static u8 LX83807_CSOT_SMALL_AOD_TEST_OFF[] = { 0xB0, 0xCA };

static DEFINE_STATIC_PACKET(lx83807_csot_small_aod_test_a4_on, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_AOD_TEST_A4_ON, 0);
static DEFINE_STATIC_PACKET(lx83807_csot_small_aod_test_off, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_AOD_TEST_OFF, 0);

#if defined(CONFIG_USDM_FACTORY)
static u8 LX83807_CSOT_SMALL_SELF_MASK_ENABLE[] = {
	0x7A,
	0x01, 0x00, 0xD9, 0x00, 0xD7, 0x00, 0xD7, 0x00,
	0x00, 0x00, 0x01
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_self_mask_enable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_SELF_MASK_ENABLE, 0);
#else
static u8 LX83807_CSOT_SMALL_SELF_MASK_ENABLE[] = {
	0x7A,
	0x01, 0x00, 0xD8, 0x00, 0xD7, 0x00, 0xD7, 0x00,
	0x00, 0x00, 0x01
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_self_mask_enable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_SELF_MASK_ENABLE, 0);

#endif

static u8 LX83807_CSOT_SMALL_SELF_MASK_DISABLE[] = {
	0x7A,
	0x00
};
static DEFINE_STATIC_PACKET(lx83807_csot_small_self_mask_disable, DSI_PKT_TYPE_WR, LX83807_CSOT_SMALL_SELF_MASK_DISABLE, 0);

static void *lx83807_csot_small_aod_self_mask_ena_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_aod_test_a4_on),
	&PKTINFO(lx83807_csot_small_self_mask_enable),
	&PKTINFO(lx83807_csot_small_aod_test_off),
};

static void *lx83807_csot_small_aod_self_mask_dis_cmdtbl[] = {
	&PKTINFO(lx83807_csot_small_aod_test_a4_on),
	&PKTINFO(lx83807_csot_small_self_mask_disable),
	&PKTINFO(lx83807_csot_small_aod_test_off),
};

static struct seqinfo lx83807_csot_small_aod_seqtbl[] = {
	SEQINFO_INIT(SELF_MASK_ENA_SEQ, lx83807_csot_small_aod_self_mask_ena_cmdtbl),
	SEQINFO_INIT(SELF_MASK_DIS_SEQ, lx83807_csot_small_aod_self_mask_dis_cmdtbl),
};

static struct aod_tune lx83807_csot_small_aod = {
	.name = "lx83807_csot_small_aod",
	.nr_seqtbl = ARRAY_SIZE(lx83807_csot_small_aod_seqtbl),
	.seqtbl = lx83807_csot_small_aod_seqtbl,
	.nr_maptbl = ARRAY_SIZE(lx83807_csot_small_aod_maptbl),
	.maptbl = lx83807_csot_small_aod_maptbl,
	.self_mask_en = true,
};
#endif
