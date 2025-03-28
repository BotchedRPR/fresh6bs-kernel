// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos SoC series Pablo driver
 * Pablo v9.1 specific functions
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#if IS_ENABLED(CONFIG_EXYNOS_SCI)
#include <soc/samsung/exynos-sci.h>
#endif

#include "is-config.h"
#include "is-param.h"
#include "is-type.h"
#include "is-core.h"
#include "is-hw-chain.h"
#include "is-hw-settle-5nm-lpp.h"
#include "is-device-sensor.h"
#include "is-device-csi.h"
#include "is-device-ischain.h"
#include "../../interface/is-interface-ischain.h"
#include "is-hw.h"
#include "../../interface/is-interface-library.h"
#include "votf/pablo-votf.h"
#include "is-hw-dvfs.h"
#include "../../hardware/sfr/is-sfr-lme-v1_1.h"
#include "pablo-fpsimd.h"

/* #define USE_ORBMCH_WA */

/* SYSREG register description */
static const struct is_reg sysreg_csis_regs[SYSREG_CSIS_REG_CNT] = {
	{0x0108, "MEMCLK"},
	{0x0180, "POWER_FAIL_DETECT"},
	{0x031C, "CSIS_EMA_STATUS"},
	{0x0408, "CSIS_FRAME_ID_EN"},
	{0x040c, "CSISX6_SC_PDP_IN_EN"},
	{0x0410, "CSISX6_SC_CON0"},
	{0x0414, "CSISX6_SC_CON1"},
	{0x0418, "CSISX6_SC_CON2"},
	{0x041c, "CSISX6_SC_CON3"},
	{0x0420, "CSISX6_SC_CON4"},
	{0x0424, "CSISX6_SC_CON5"},
	{0x0428, "CSISX6_SC_CON6"},
	{0x042C, "CSISX6_SC_CON7"},
	{0x0430, "PDP_VC_CON0"},
	{0x0434, "PDP_VC_CON1"},
	{0x0438, "PDP_VC_CON2"},
	{0x043C, "PDP_VC_CON3"},
	{0x0440, "LH_GLUE_CON"},
	{0x0444, "LH_GLUE_INT_CON"},
	{0x04A0, "CSIS0_FRAME_ID_EN"},
	{0x04A4, "CSIS1_FRAME_ID_EN"},
	{0x04A8, "CSIS2_FRAME_ID_EN"},
	{0x04AC, "CSIS3_FRAME_ID_EN"},
	{0x04B0, "CSIS4_FRAME_ID_EN"},
	{0x04B4, "CSIS5_FRAME_ID_EN"},
	{0x0500, "MIPI_PHY_CON"},
};

static const struct is_reg sysreg_taa_regs[SYSREG_TAA_REG_CNT] = {
	{0X0108, "MEMCLK"},
	{0X0400, "TAA_USER_CON0"},
	{0X0404, "TAA_USER_CON1"},
	{0X0408, "LH_QACTIVE_CON"},
};

static const struct is_reg sysreg_mcfp0_regs[SYSREG_MCFP0_REG_CNT] = {
	{0x0108, "MEMCLK"},
};

static const struct is_reg sysreg_mcfp1_regs[SYSREG_MCFP1_REG_CNT] = {
	{0x0108, "MEMCLK"},
};

static const struct is_reg sysreg_dns_regs[SYSREG_DNS_REG_CNT] = {
	{0x0108, "MEMCLK"},
	{0x0400, "DNS_USER_CON0"},
	{0x0404, "DNS_USER_CON1"},
	{0x0428, "DNS_USER_CON5"},
};

static const struct is_reg sysreg_itp_regs[SYSREG_ITP_REG_CNT] = {
	{0x0108, "MEMCLK"},
	{0x0400, "ITP_USER_CON"},
};

static const struct is_reg sysreg_yuvpp_regs[SYSREG_YUVPP_REG_CNT] = {
	{0x0108, "MEMCLK"},
	{0x040C, "YUVPP_USER_CON3"},
};

static const struct is_reg sysreg_mcsc_regs[SYSREG_MCSC_REG_CNT] = {
	{0x0108, "MEMCLK"},
	{0x0408, "MCSC_USER_CON2"},
};

static const struct is_reg sysreg_lme_regs[SYSREG_LME_REG_CNT] = {
	{0x0108, "MEMCLK"},
	{0x0400, "LME_USER_CON0"},
};

static const struct is_field sysreg_csis_fields[SYSREG_CSIS_REG_FIELD_CNT] = {
	{"EN", 0, 1, RW, 0x1},
	{"PFD_AVDD085_MIPI_CPHY_STAT", 4, 1, RO, 0x0},
	{"PFD_AVDD085_MIPI_CPHY", 0, 1, RW, 0x0},
	{"SFR_ENABLE", 8, 1, RW, 0x0},
	{"WIDTH_DATA2REQ", 4, 2, RW, 0x3},
	{"EMA_BUSY", 0, 1, RO, 0x0},
	{"FRAME_ID_EN_CSIS0", 0, 1, RW, 0x0},
	{"PDP3_IN_CSIS5_EN", 23, 1, RW, 0x0},
	{"PDP3_IN_CSIS4_EN", 22, 1, RW, 0x0},
	{"PDP3_IN_CSIS3_EN", 21, 1, RW, 0x0},
	{"PDP3_IN_CSIS2_EN", 20, 1, RW, 0x0},
	{"PDP3_IN_CSIS1_EN", 19, 1, RW, 0x0},
	{"PDP3_IN_CSIS0_EN", 18, 1, RW, 0x0},
	{"PDP2_IN_CSIS5_EN", 17, 1, RW, 0x0},
	{"PDP2_IN_CSIS4_EN", 16, 1, RW, 0x0},
	{"PDP2_IN_CSIS3_EN", 15, 1, RW, 0x0},
	{"PDP2_IN_CSIS2_EN", 14, 1, RW, 0x0},
	{"PDP2_IN_CSIS1_EN", 13, 1, RW, 0x0},
	{"PDP2_IN_CSIS0_EN", 12, 1, RW, 0x0},
	{"PDP1_IN_CSIS5_EN", 11, 1, RW, 0x0},
	{"PDP1_IN_CSIS4_EN", 10, 1, RW, 0x0},
	{"PDP1_IN_CSIS3_EN", 9, 1, RW, 0x0},
	{"PDP1_IN_CSIS2_EN", 8, 1, RW, 0x0},
	{"PDP1_IN_CSIS1_EN", 7, 1, RW, 0x0},
	{"PDP1_IN_CSIS0_EN", 6, 1, RW, 0x0},
	{"PDP0_IN_CSIS5_EN", 5, 1, RW, 0x0},
	{"PDP0_IN_CSIS4_EN", 4, 1, RW, 0x0},
	{"PDP0_IN_CSIS3_EN", 3, 1, RW, 0x0},
	{"PDP0_IN_CSIS2_EN", 2, 1, RW, 0x0},
	{"PDP0_IN_CSIS1_EN", 1, 1, RW, 0x0},
	{"PDP0_IN_CSIS0_EN", 0, 1, RW, 0x0},
	{"GLUEMUX_PDP0_VAL", 0, 3, RW, 0x0},
	{"GLUEMUX_PDP1_VAL", 0, 3, RW, 0x0},
	{"GLUEMUX_PDP2_VAL", 0, 3, RW, 0x0},
	{"GLUEMUX_PDP3_VAL", 0, 3, RW, 0x0},
	{"GLUEMUX_CSIS_DMA0_OTF_SEL", 0, 4, RW, 0x0},
	{"GLUEMUX_CSIS_DMA1_OTF_SEL", 0, 4, RW, 0x0},
	{"GLUEMUX_CSIS_DMA2_OTF_SEL", 0, 4, RW, 0x0},
	{"GLUEMUX_CSIS_DMA3_OTF_SEL", 0, 4, RW, 0x0},
	{"MUX_IMG_VC_PDP0", 16, 2, RW, 0x0},
	{"MUX_AF_VC_PDP0", 0, 2, RW, 0x1},
	{"MUX_IMG_VC_PDP1", 16, 2, RW, 0x0},
	{"MUX_AF_VC_PDP1", 0, 2, RW, 0x1},
	{"MUX_IMG_VC_PDP2", 16, 2, RW, 0x0},
	{"MUX_AF_VC_PDP2", 0, 2, RW, 0x1},
	{"MUX_IMG_VC_PDP3", 16, 2, RW, 0x0},
	{"MUX_AF_VC_PDP3", 0, 2, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_ZOTF3_TAACSIS", 23, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_ZOTF2_TAACSIS", 22, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_ZOTF1_TAACSIS", 21, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_ZOTF0_TAACSIS", 20, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_SOTF3_TAACSIS", 19, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_SOTF2_TAACSIS", 18, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_SOTF1_TAACSIS", 17, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_SOTF0_TAACSIS", 16, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_OTF3_CSISTAA", 15, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_OTF2_CSISTAA", 14, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_OTF1_CSISTAA", 13, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_OTF0_CSISTAA", 12, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_INT_OTF3_PDPCSIS", 31, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_INT_OTF2_PDPCSIS", 30, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_INT_OTF1_PDPCSIS", 29, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_INT_OTF0_PDPCSIS", 28, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_INT_OTF3_PDPCSIS", 27, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_INT_OTF2_PDPCSIS", 26, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_INT_OTF1_PDPCSIS", 25, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_INT_OTF0_PDPCSIS", 24, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_INT_OTF3_CSISPDP", 23, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_INT_OTF2_CSISPDP", 22, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_INT_OTF1_CSISPDP", 21, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_INT_OTF0_CSISPDP", 20, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_INT_OTF3_CSISPDP", 19, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_INT_OTF2_CSISPDP", 18, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_INT_OTF1_CSISPDP", 17, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_INT_OTF0_CSISPDP", 16, 1, RW, 0x1},
	{"FID_LOC_BYTE", 16, 16, RW, 0x1b},
	{"FID_LOC_LINE", 8, 2, RW, 0x0},
	{"FRAME_ID_EN_CSIS", 0, 1, RW, 0x0},
	{"MIPI_RESETN_DPHY_S1", 5, 1, RW, 0x0},
	{"MIPI_RESETN_DPHY_S", 4, 1, RW, 0x0},
	{"MIPI_RESETN_DCPHY_S3", 3, 1, RW, 0x0},
	{"MIPI_RESETN_DCPHY_S2", 2, 1, RW, 0x0},
	{"MIPI_RESETN_DCPHY_S1", 1, 1, RW, 0x0},
	{"MIPI_RESETN_DCPHY_S", 0, 1, RW, 0x0},
};

static const struct is_field sysreg_taa_fields[SYSREG_TAA_REG_FIELD_CNT] = {
	{"EN", 0, 1, RW, 0x1},
	{"SW_RESETN_C2R_TAA1", 27, 1, RW, 0x1},
	{"SW_RESETN_C2R_TAA0", 26, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_OTF_TAADNS", 25, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_SOTF3_TAACSIS", 24, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_SOTF2_TAACSIS", 23, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_SOTF1_TAACSIS", 22, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_SOTF0_TAACSIS", 21, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_ZOTF3_TAACSIS", 20, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_ZOTF2_TAACSIS", 19, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_ZOTF1_TAACSIS", 18, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_ZOTF0_TAACSIS", 17, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_OTF3_CSISTAA", 16, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_OTF2_CSISTAA", 15, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_OTF1_CSISTAA", 14, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_OTF0_CSISTAA", 13, 1, RW, 0x1},
	{"TYPE_LHS_AST_GLUE_OTF_TAADNS", 12, 1, RW, 0x0},
	{"TYPE_LHS_AST_GLUE_SOTF3_TAACSIS", 11, 1, RW, 0x0},
	{"TYPE_LHS_AST_GLUE_SOTF2_TAACSIS", 10, 1, RW, 0x0},
	{"TYPE_LHS_AST_GLUE_SOTF1_TAACSIS", 9, 1, RW, 0x0},
	{"TYPE_LHS_AST_GLUE_SOTF0_TAACSIS", 8, 1, RW, 0x0},
	{"TYPE_LHS_AST_GLUE_ZOTF3_TAACSIS", 7, 1, RW, 0x0},
	{"TYPE_LHS_AST_GLUE_ZOTF2_TAACSIS", 6, 1, RW, 0x0},
	{"TYPE_LHS_AST_GLUE_ZOTF1_TAACSIS", 5, 1, RW, 0x0},
	{"TYPE_LHS_AST_GLUE_ZOTF0_TAACSIS", 4, 1, RW, 0x0},
	{"TYPE_LHM_AST_GLUE_OTF3_CSISTAA", 3, 1, RW, 0x0},
	{"TYPE_LHM_AST_GLUE_OTF2_CSISTAA", 2, 1, RW, 0x0},
	{"TYPE_LHM_AST_GLUE_OTF1_CSISTAA", 1, 1, RW, 0x0},
	{"TYPE_LHM_AST_GLUE_OTF0_CSISTAA", 0, 1, RW, 0x0},
	{"GLUEMUX_OTFOUT_SEL_0", 0, 2, RW, 0x0},
	{"LHS_AST_OTF_TAADNS", 12, 1, RW, 0x1},
	{"LHS_AST_ZOTF3_TAACSIS", 11, 1, RW, 0x1},
	{"LHS_AST_ZOTF2_TAACSIS", 10, 1, RW, 0x1},
	{"LHS_AST_ZOTF1_TAACSIS", 9, 1, RW, 0x1},
	{"LHS_AST_ZOTF0_TAACSIS", 8, 1, RW, 0x1},
	{"LHS_AST_SOTF3_TAACSIS", 7, 1, RW, 0x1},
	{"LHS_AST_SOTF2_TAACSIS", 6, 1, RW, 0x1},
	{"LHS_AST_SOTF1_TAACSIS", 5, 1, RW, 0x1},
	{"LHS_AST_SOTF0_TAACSIS", 4, 1, RW, 0x1},
	{"LHM_AST_OTF3_CSISTAA", 3, 1, RW, 0x1},
	{"LHM_AST_OTF2_CSISTAA", 2, 1, RW, 0x1},
	{"LHM_AST_OTF1_CSISTAA", 1, 1, RW, 0x1},
	{"LHM_AST_OTF0_CSISTAA", 0, 1, RW, 0x1},
};

static const struct is_field sysreg_mcfp0_fields[SYSREG_MCFP0_REG_FIELD_CNT] = {
	{"EN", 0, 1, RW, 0x1},
};

static const struct is_field sysreg_mcfp1_fields[SYSREG_MCFP1_REG_FIELD_CNT] = {
	{"EN", 0, 1, RW, 0x1},
};

static const struct is_field sysreg_dns_fields[SYSREG_DNS_REG_FIELD_CNT] = {
	{"EN", 0, 1, RW, 0x1},
#if IS_ENABLED(CONFIG_SOC_EXYNOS2100_EVT0)
	{"GLUEMUX_DNS0_VAL", 8, 1, RW, 0x0},
#else
	{"GLUEMUX_DNS0_VAL", 12, 1, RW, 0x0},
#endif
	{"AXCACHE_DNS1", 4, 4, RW, 0x2},
	{"AXCACHE_DNS0", 0, 4, RW, 0x2},
	{"C2R_DNS_SW_RESET", 23, 1, RW, 0x1},
	{"ENABLE_OTF4_IN_ITPDNS", 22, 1, RW, 0x1},
	{"ENABLE_OTF3_IN_ITPDNS", 21, 1, RW, 0x1},
	{"ENABLE_OTF2_IN_ITPDNS", 20, 1, RW, 0x1},
	{"ENABLE_OTF1_IN_ITPDNS", 19, 1, RW, 0x1},
	{"ENABLE_OTF0_IN_ITPDNS", 18, 1, RW, 0x1},
	{"ENABLE_OTF_OUT_CTL_DNSITP", 17, 1, RW, 0x1},
	{"ENABLE_OTF_IN_CTL_ITPDNS", 16, 1, RW, 0x1},
	{"ENABLE_OTF9_OUT_DNSITP", 15, 1, RW, 0x1},
	{"ENABLE_OTF8_OUT_DNSITP", 14, 1, RW, 0x1},
	{"ENABLE_OTF7_OUT_DNSITP", 13, 1, RW, 0x1},
	{"ENABLE_OTF6_OUT_DNSITP", 12, 1, RW, 0x1},
	{"ENABLE_OTF5_OUT_DNSITP", 11, 1, RW, 0x1},
	{"ENABLE_OTF4_OUT_DNSITP", 10, 1, RW, 0x1},
	{"ENABLE_OTF3_OUT_DNSITP", 9, 1, RW, 0x1},
	{"ENABLE_OTF2_OUT_DNSITP", 8, 1, RW, 0x1},
	{"ENABLE_OTF1_OUT_DNSITP", 7, 1, RW, 0x1},
	{"ENABLE_OTF0_OUT_DNSITP", 6, 1, RW, 0x1},
	{"ENABLE_OTF_IN_MCFP1DNS", 5, 1, RW, 0x1},
	{"ENABLE_OTF_IN_TAADNS", 4, 1, RW, 0x1},
	{"TYPE_LHM_AST_GLUE_OTF_MCFP1DNS", 3, 1, RW, 0x0},
	{"TYPE_LHM_AST_GLUE_OTF_TAADNS", 2, 1, RW, 0x0},
	{"SW_RESETN_LHM_AST_GLUE_OTF_MCFP1DNS", 1, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_OTF_TAADNS", 0, 1, RW, 0x1},
	{"C2R_DNS_TOPBYPASS_EN", 1, 1, RW, 0x1},
	{"C2R_DNS_TOPBYPASS_SEL", 0, 1, RW, 0x0},
};

static const struct is_field sysreg_itp_fields[SYSREG_ITP_REG_FIELD_CNT] = {
	{"EN", 0, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_OTF_MCFP1ITP", 19, 1, RW, 0x1},
	{"TYPE_LHM_AST_GLUE_OTF_MCFP1ITP", 18, 1, RW, 0x0},
	{"ENABLE_OTF_IN_MCFP1ITP", 17, 1, RW, 0x1},
	{"ENABLE_OTF4_OUT_ITPDNS", 16, 1, RW, 0x1},
	{"ENABLE_OTF3_OUT_ITPDNS", 15, 1, RW, 0x1},
	{"ENABLE_OTF2_OUT_ITPDNS", 14, 1, RW, 0x1},
	{"ENABLE_OTF1_OUT_ITPDNS", 13, 1, RW, 0x1},
	{"ENABLE_OTF0_OUT_ITPDNS", 12, 1, RW, 0x1},
	{"ENABLE_OTF_IN_CTL_DNSITP", 11, 1, RW, 0x1},
	{"ENABLE_OTF_OUT_CTL_ITPDNS", 10, 1, RW, 0x1},
	{"ENABLE_OTF9_IN_DNSITP", 9, 1, RW, 0x1},
	{"ENABLE_OTF8_IN_DNSITP", 8, 1, RW, 0x1},
	{"ENABLE_OTF7_IN_DNSITP", 7, 1, RW, 0x1},
	{"ENABLE_OTF6_IN_DNSITP", 6, 1, RW, 0x1},
	{"ENABLE_OTF5_IN_DNSITP", 5, 1, RW, 0x1},
	{"ENABLE_OTF4_IN_DNSITP", 4, 1, RW, 0x1},
	{"ENABLE_OTF3_IN_DNSITP", 3, 1, RW, 0x1},
	{"ENABLE_OTF2_IN_DNSITP", 2, 1, RW, 0x1},
	{"ENABLE_OTF1_IN_DNSITP", 1, 1, RW, 0x1},
	{"ENABLE_OTF0_IN_DNSITP", 0, 1, RW, 0x1},
};

static const struct is_field sysreg_yuvpp_fields[SYSREG_YUVPP_REG_FIELD_CNT] = {
	{"EN", 0, 1, RW, 0x1},
	{"SW_RESETN_LHS_AST_GLUE_OTF_YUVPPMCSC", 1, 1, RW, 0x1},
	{"TYPE_LHS_AST_GLUE_OTF_YUVPPMCSC", 0, 1, RW, 0x0},
};

static const struct is_field sysreg_mcsc_fields[SYSREG_MCSC_REG_FIELD_CNT] = {
	{"EN", 0, 1, RW, 0x1},
	{"SW_RESETN_LHM_AST_GLUE_OTF_YUVPPMCSC", 14, 1, RW, 0x1},
	{"TYPE_LHM_AST_GLUE_OTF_YUVPPMCSC", 13, 1, RW, 0x0},
	{"C2R_NPUS_MCSC_TOPBYPASS_SEL", 12, 1, RW, 0x0},
	{"C2R_VPC_MCSC_TOPBYPASS_SEL", 11, 1, RW, 0x0},
	{"C2R_DPUF_MCSC_TOPBYPASS_SEL", 10, 1, RW, 0x0},
	{"C2R_NPUS_MCSC_DISABLE_SEL", 9, 1, RW, 0x1},
	{"C2R_VPC_MCSC_DISABLE_SEL", 8, 1, RW, 0x1},
	{"C2R_DPUF_MCSC_DISABLE_SEL", 7, 1, RW, 0x1},
	{"C2R_NPUS_MCSC_SW_RESET", 6, 1, RW, 0x1},
	{"C2R_VPC_MCSC_SW_RESET", 5, 1, RW, 0x1},
	{"C2R_DPUF_MCSC_SW_RESET", 4, 1, RW, 0x1},
	{"C2AGENT_D2_MCSC_M6S4_C2AGENT_SW_RESET", 3, 1, RW, 0x1},
	{"C2AGENT_D1_MCSC_M6S4_C2AGENT_SW_RESET", 2, 1, RW, 0x1},
	{"C2AGENT_D0_MCSC_M6S4_C2AGENT_SW_RESET", 1, 1, RW, 0x1},
	{"ENABLE_OTF_IN_LHM_AST_OTF_YUVPPMCSC", 0, 1, RW, 0x1},
};

static const struct is_field sysreg_lme_fields[SYSREG_LME_REG_FIELD_CNT] = {
	{"EN", 0, 1, RW, 0x1},
	{"LME_SW_RESET", 4, 1, RW, 0x1},
	{"AXCACHE_LME", 0, 4, RW, 0x2},
};

static void __iomem *hwfc_rst;

static inline void __nocfi __is_isr_ddk(void *data, int handler_id)
{
	struct is_interface_hwip *itf_hw = NULL;
	struct hwip_intr_handler *intr_hw = NULL;

	itf_hw = (struct is_interface_hwip *)data;
	intr_hw = &itf_hw->handler[handler_id];

	if (intr_hw->valid) {
		is_fpsimd_get_isr();
		intr_hw->handler(intr_hw->id, intr_hw->ctx);
		is_fpsimd_put_isr();
	} else {
		err_itfc("[ID:%d](%d)- chain(%d) empty handler!!",
			itf_hw->id, handler_id, intr_hw->chain_id);
	}
}

static inline void __is_isr_host(void *data, int handler_id)
{
	struct is_interface_hwip *itf_hw = NULL;
	struct hwip_intr_handler *intr_hw = NULL;

	itf_hw = (struct is_interface_hwip *)data;
	intr_hw = &itf_hw->handler[handler_id];

	if (intr_hw->valid)
		intr_hw->handler(intr_hw->id, (void *)itf_hw->hw_ip);
	else
		err_itfc("[ID:%d](1) empty handler!!", itf_hw->id);
}

/*
 * Interrupt handler definitions
 */
static void __nocfi __is_isr4_3aax_common(int handler_id)
{
	struct is_lib_support *lib = is_get_lib_support();
	struct hwip_intr_handler intr_hw;

	intr_hw = *lib->intr_handler_taaisp[ID_3AA_0][handler_id];
	if (intr_hw.valid) {
		is_fpsimd_get_isr();
		intr_hw.handler(intr_hw.id, intr_hw.ctx);
		is_fpsimd_put_isr();
	}

	intr_hw = *lib->intr_handler_taaisp[ID_3AA_1][handler_id];
	if (intr_hw.valid) {
		is_fpsimd_get_isr();
		intr_hw.handler(intr_hw.id, intr_hw.ctx);
		is_fpsimd_put_isr();
	}

	intr_hw = *lib->intr_handler_taaisp[ID_3AA_2][handler_id];
	if (intr_hw.valid) {
		is_fpsimd_get_isr();
		intr_hw.handler(intr_hw.id, intr_hw.ctx);
		is_fpsimd_put_isr();
	}

	intr_hw = *lib->intr_handler_taaisp[ID_3AA_3][handler_id];
	if (intr_hw.valid) {
		is_fpsimd_get_isr();
		intr_hw.handler(intr_hw.id, intr_hw.ctx);
		is_fpsimd_put_isr();
	}
}

/* 3AA0 */
static irqreturn_t __is_isr1_3aa0(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP1);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr2_3aa0(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP2);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr3_3aa0(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP3);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr4_3aa0(int irq, void *data)
{
	__is_isr4_3aax_common(INTR_HWIP4);
	return IRQ_HANDLED;
}

/* 3AA1 */
static irqreturn_t __is_isr1_3aa1(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP1);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr2_3aa1(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP2);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr3_3aa1(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP3);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr4_3aa1(int irq, void *data)
{
	__is_isr4_3aax_common(INTR_HWIP5);
	return IRQ_HANDLED;
}

/* 3AA2 */
static irqreturn_t __is_isr1_3aa2(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP1);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr2_3aa2(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP2);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr3_3aa2(int irq, void *data)
{
	/* FOR DMA2/3 IRQ shared */
	struct is_interface_hwip *itf_hw = NULL;
	struct hwip_intr_handler *intr_hw = NULL;

	itf_hw = (struct is_interface_hwip *)data;
	intr_hw = &itf_hw->handler[INTR_HWIP3];

	if (intr_hw->chain_id != ID_3AA_2)
		return IRQ_NONE;

	__is_isr_ddk(data, INTR_HWIP3);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr4_3aa2(int irq, void *data)
{
	__is_isr4_3aax_common(INTR_HWIP6);
	return IRQ_HANDLED;
}

/* 3AA3 */
static irqreturn_t __is_isr1_3aa3(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP1);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr2_3aa3(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP2);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr3_3aa3(int irq, void *data)
{
	/* FOR DMA2/3 IRQ shared */
	struct is_interface_hwip *itf_hw = NULL;
	struct hwip_intr_handler *intr_hw = NULL;

	itf_hw = (struct is_interface_hwip *)data;
	intr_hw = &itf_hw->handler[INTR_HWIP3];

	if (intr_hw->chain_id != ID_3AA_3)
		return IRQ_NONE;

	__is_isr_ddk(data, INTR_HWIP3);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr4_3aa3(int irq, void *data)
{
	__is_isr4_3aax_common(INTR_HWIP7);
	return IRQ_HANDLED;
}

/* ITP0 */
static irqreturn_t __is_isr1_itp0(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP1);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr2_itp0(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP2);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr3_itp0(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP3);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr4_itp0(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP4);
	return IRQ_HANDLED;
}

/* LME */
static irqreturn_t __is_isr1_lme(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP1);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr2_lme(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP2);
	return IRQ_HANDLED;
}

#ifdef USE_ORBMCH_WA
static irqreturn_t __is_isr3_orbmch0(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP3);
	return IRQ_HANDLED;
}

static irqreturn_t __is_isr4_orbmch1(int irq, void *data)
{
	__is_isr_ddk(data, INTR_HWIP3);
	return IRQ_HANDLED;
}
#endif

/* YUVPP */
static irqreturn_t __is_isr1_yuvpp(int irq, void *data)
{
	/* To handle host and ddk both, host isr handler is registerd as slot 5 */
	__is_isr_host(data, INTR_HWIP5);
	return IRQ_HANDLED;
}

/* MCSC */
static irqreturn_t __is_isr1_mcs0(int irq, void *data)
{
	__is_isr_host(data, INTR_HWIP1);
	return IRQ_HANDLED;
}

/*
 * HW group related functions
 */
void __is_hw_group_init(struct is_group *group)
{
	int i;

	for (i = ENTRY_SENSOR; i < ENTRY_END; i++)
		group->subdev[i] = NULL;

	INIT_LIST_HEAD(&group->subdev_list);
}

int is_hw_group_cfg(void *group_data)
{
	int ret = 0;
	struct is_group *group;
	struct is_device_sensor *sensor;
	struct is_device_ischain *device;

	FIMC_BUG(!group_data);

	group = (struct is_group *)group_data;

#ifdef CONFIG_USE_SENSOR_GROUP
	if (group->slot == GROUP_SLOT_SENSOR) {
		u32 vc;

		sensor = group->sensor;
		if (!sensor) {
			err("device is NULL");
			BUG();
		}

		__is_hw_group_init(group);
		group->subdev[ENTRY_SENSOR] = &sensor->group_sensor.leader;
		list_add_tail(&sensor->group_sensor.leader.list, &group->subdev_list);

		for (vc = CSI_VIRTUAL_CH_0; vc < CSI_VIRTUAL_CH_MAX; vc++) {
			group->subdev[ENTRY_SSVC0 + vc] = &sensor->ssvc[vc];
			list_add_tail(&sensor->ssvc[vc].list, &group->subdev_list);
		}

		return ret;
	}
#endif

	device = group->device;
	if (!device) {
		err("device is NULL");
		BUG();
	}

	switch (group->slot) {
	case GROUP_SLOT_PAF:
		__is_hw_group_init(group);
		group->subdev[ENTRY_PAF] = &device->group_paf.leader;

		list_add_tail(&device->group_paf.leader.list, &group->subdev_list);
		break;
	case GROUP_SLOT_3AA:
		__is_hw_group_init(group);
		group->subdev[ENTRY_3AA] = &device->group_3aa.leader;

		list_add_tail(&device->group_3aa.leader.list, &group->subdev_list);
		break;
	case GROUP_SLOT_LME:
		__is_hw_group_init(group);
		group->subdev[ENTRY_LME] = &device->group_lme.leader;

		list_add_tail(&device->group_lme.leader.list, &group->subdev_list);
		break;
	case GROUP_SLOT_ISP:
		__is_hw_group_init(group);
		group->subdev[ENTRY_ISP] = &device->group_isp.leader;

		list_add_tail(&device->group_isp.leader.list, &group->subdev_list);
		break;
	case GROUP_SLOT_MCS:
		__is_hw_group_init(group);
		group->subdev[ENTRY_MCS] = &device->group_mcs.leader;

		list_add_tail(&device->group_mcs.leader.list, &group->subdev_list);
		break;
	case GROUP_SLOT_YPP:
		__is_hw_group_init(group);
		group->subdev[ENTRY_YPP] = &device->group_ypp.leader;

		list_add_tail(&device->group_ypp.leader.list, &group->subdev_list);
		break;
	case GROUP_SLOT_VRA:
	case GROUP_SLOT_CLH:
		break;
	default:
		probe_err("group slot(%d) is invalid", group->slot);
		BUG();
		break;
	}

	/* for hwfc: reset all REGION_IDX registers and outputs */
	hwfc_rst = ioremap(HWFC_INDEX_RESET_ADDR, SZ_4);

	return ret;
}

int is_hw_group_open(void *group_data)
{
	int ret = 0;
	u32 group_id;
	struct is_subdev *leader;
	struct is_group *group;
	struct is_device_ischain *device;

	FIMC_BUG(!group_data);

	group = group_data;
	leader = &group->leader;
	device = group->device;
	group_id = group->id;

	switch (group_id) {
#ifdef CONFIG_USE_SENSOR_GROUP
	case GROUP_ID_SS0:
	case GROUP_ID_SS1:
	case GROUP_ID_SS2:
	case GROUP_ID_SS3:
	case GROUP_ID_SS4:
	case GROUP_ID_SS5:
		leader->constraints_width = GROUP_SENSOR_MAX_WIDTH;
		leader->constraints_height = GROUP_SENSOR_MAX_HEIGHT;
		break;
#endif
	case GROUP_ID_PAF0:
	case GROUP_ID_PAF1:
	case GROUP_ID_PAF2:
	case GROUP_ID_PAF3:
		leader->constraints_width = GROUP_PDP_MAX_WIDTH;
		leader->constraints_height = GROUP_PDP_MAX_HEIGHT;
		break;
	case GROUP_ID_3AA0:
	case GROUP_ID_3AA1:
	case GROUP_ID_3AA2:
	case GROUP_ID_3AA3:
		leader->constraints_width = GROUP_3AA_MAX_WIDTH;
		leader->constraints_height = GROUP_3AA_MAX_HEIGHT;
		break;
	case GROUP_ID_ISP0:
	case GROUP_ID_YPP:
	case GROUP_ID_MCS0:
	case GROUP_ID_MCS1:
		leader->constraints_width = GROUP_ITP_MAX_WIDTH;
		leader->constraints_height = GROUP_ITP_MAX_HEIGHT;
		break;
	case GROUP_ID_LME0:
		leader->constraints_width = GROUP_LME_MAX_WIDTH;
		leader->constraints_height = GROUP_LME_MAX_HEIGHT;
		break;
	default:
		merr("(%s) is invalid", group, group_id_name[group_id]);
		break;
	}

	return ret;
}

int is_get_hw_list(int group_id, int *hw_list)
{
	int i;
	int hw_index = 0;

	/* initialization */
	for (i = 0; i < GROUP_HW_MAX; i++)
		hw_list[i] = -1;

	switch (group_id) {
	case GROUP_ID_PAF0:
		hw_list[hw_index] = DEV_HW_PAF0; hw_index++;
		break;
	case GROUP_ID_PAF1:
		hw_list[hw_index] = DEV_HW_PAF1; hw_index++;
		break;
	case GROUP_ID_PAF2:
		hw_list[hw_index] = DEV_HW_PAF2; hw_index++;
		break;
	case GROUP_ID_PAF3:
		hw_list[hw_index] = DEV_HW_PAF3; hw_index++;
		break;
	case GROUP_ID_3AA0:
		hw_list[hw_index] = DEV_HW_3AA0; hw_index++;
		break;
	case GROUP_ID_3AA1:
		hw_list[hw_index] = DEV_HW_3AA1; hw_index++;
		break;
	case GROUP_ID_3AA2:
		hw_list[hw_index] = DEV_HW_3AA2; hw_index++;
		break;
	case GROUP_ID_3AA3:
		hw_list[hw_index] = DEV_HW_3AA3; hw_index++;
		break;
	case GROUP_ID_LME0:
		hw_list[hw_index] = DEV_HW_LME0; hw_index++;
		break;
	case GROUP_ID_ISP0:
		hw_list[hw_index] = DEV_HW_ISP0; hw_index++;
		break;
	case GROUP_ID_YPP:
		hw_list[hw_index] = DEV_HW_YPP; hw_index++;
		break;
	case GROUP_ID_MCS0:
		hw_list[hw_index] = DEV_HW_MCSC0; hw_index++;
		break;
	case GROUP_ID_MAX:
		break;
	default:
		err("Invalid group%d(%s)", group_id, group_id_name[group_id]);
		break;
	}

	return hw_index;
}
/*
 * System registers configurations
 */
static inline int __is_hw_get_address(struct platform_device *pdev,
				struct is_interface_hwip *itf_hwip,
				int hw_id, char *hw_name,
				u32 resource_id, enum base_reg_index reg_index,
				bool alloc_memlog)
{
	struct resource *mem_res = NULL;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, resource_id);
	if (!mem_res) {
		dev_err(&pdev->dev, "Failed to get io memory region\n");
		return -EINVAL;
	}

	itf_hwip->hw_ip->regs_start[reg_index] = mem_res->start;
	itf_hwip->hw_ip->regs_end[reg_index] = mem_res->end;
	itf_hwip->hw_ip->regs[reg_index] =
		ioremap(mem_res->start, resource_size(mem_res));
	if (!itf_hwip->hw_ip->regs[reg_index]) {
		dev_err(&pdev->dev, "Failed to remap io region\n");
		return -EINVAL;
	}

	if (alloc_memlog)
		is_debug_memlog_alloc_dump(mem_res->start,
					resource_size(mem_res), hw_name);

	info_itfc("[ID:%2d] %s VA(0x%lx)\n", hw_id, hw_name,
		(ulong)itf_hwip->hw_ip->regs[reg_index]);

	return 0;
}

int is_hw_get_address(void *itfc_data, void *pdev_data, int hw_id)
{
	struct platform_device *pdev = NULL;
	struct is_interface_hwip *itf_hwip = NULL;
	int idx;

	FIMC_BUG(!itfc_data);
	FIMC_BUG(!pdev_data);

	itf_hwip = (struct is_interface_hwip *)itfc_data;
	pdev = (struct platform_device *)pdev_data;

	switch (hw_id) {
	case DEV_HW_3AA0:
		__is_hw_get_address(pdev, itf_hwip, hw_id, "3AA0", IORESOURCE_3AA0, REG_SETA, false);
		/* TODO: need check if exist dump_region */
		idx = 0;
		itf_hwip->hw_ip->dump_region[REG_SETA][idx].start = 0x0;
		itf_hwip->hw_ip->dump_region[REG_SETA][idx++].end = 0x1FAF;
		itf_hwip->hw_ip->dump_region[REG_SETA][idx].start = 0x1FC0;
		itf_hwip->hw_ip->dump_region[REG_SETA][idx++].end = 0x9FAF;
		itf_hwip->hw_ip->dump_region[REG_SETA][idx].start = 0x9FC0;
		itf_hwip->hw_ip->dump_region[REG_SETA][idx++].end = 0xFFFF;

		__is_hw_get_address(pdev, itf_hwip, hw_id, "3AA DMA", IORESOURCE_3AA_DMA_TOP, REG_EXT2, true);
		break;
	case DEV_HW_3AA1:
		__is_hw_get_address(pdev, itf_hwip, hw_id, "3AA1", IORESOURCE_3AA1, REG_SETA, false);
		itf_hwip->hw_ip->regs_start[REG_SETA] += LIC_CHAIN_OFFSET;
		itf_hwip->hw_ip->regs[REG_SETA] += LIC_CHAIN_OFFSET;

		__is_hw_get_address(pdev, itf_hwip, hw_id, "3AA DMA", IORESOURCE_3AA_DMA_TOP, REG_EXT2, false);
		break;
	case DEV_HW_3AA2:
		__is_hw_get_address(pdev, itf_hwip, hw_id, "3AA2", IORESOURCE_3AA2, REG_SETA, false);
		itf_hwip->hw_ip->regs_start[REG_SETA] += (2 * LIC_CHAIN_OFFSET);
		itf_hwip->hw_ip->regs[REG_SETA] += (2 * LIC_CHAIN_OFFSET);

		__is_hw_get_address(pdev, itf_hwip, hw_id, "3AA DMA", IORESOURCE_3AA_DMA_TOP, REG_EXT2, false);
		break;
	case DEV_HW_3AA3:
		__is_hw_get_address(pdev, itf_hwip, hw_id, "3AA3", IORESOURCE_3AA3, REG_SETA, false);
		itf_hwip->hw_ip->regs_start[REG_SETA] += (3 * LIC_CHAIN_OFFSET);
		itf_hwip->hw_ip->regs[REG_SETA] += (3 * LIC_CHAIN_OFFSET);

		__is_hw_get_address(pdev, itf_hwip, hw_id, "3AA DMA", IORESOURCE_3AA_DMA_TOP, REG_EXT2, false);
		break;
	case DEV_HW_LME0:
		__is_hw_get_address(pdev, itf_hwip, hw_id, "LME", IORESOURCE_LME, REG_SETA, false);

		idx = 0;
		itf_hwip->hw_ip->dump_region[REG_SETA][idx].start = 0x8000;
		itf_hwip->hw_ip->dump_region[REG_SETA][idx++].end = 0xFFFF;
		break;
	case DEV_HW_ISP0:
		__is_hw_get_address(pdev, itf_hwip, hw_id, "ITP", IORESOURCE_ITP, REG_SETA, true);

		__is_hw_get_address(pdev, itf_hwip, hw_id, "MCFP0", IORESOURCE_MCFP0, REG_EXT1, false);

		idx = 0;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx].start = 0x0;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx++].end = 0x03FF;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx].start = 0x0C00;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx++].end = 0x0CFF;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx].start = 0x0F00;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx++].end = 0x10FF;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx].start = 0x1800;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx++].end = 0x1AFF;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx].start = 0x2000;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx++].end = 0x26FF;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx].start = 0x2800;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx++].end = 0x2BFF;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx].start = 0x4000;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx++].end = 0x4DFF;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx].start = 0x5200;
		itf_hwip->hw_ip->dump_region[REG_EXT1][idx++].end = 0x55FF;

		__is_hw_get_address(pdev, itf_hwip, hw_id, "DNS", IORESOURCE_DNS, REG_EXT2, true);

		__is_hw_get_address(pdev, itf_hwip, hw_id, "MCFP1", IORESOURCE_MCFP1, REG_EXT3, false);

		idx = 0;
		itf_hwip->hw_ip->dump_region[REG_EXT3][idx].start = 0x0;
		itf_hwip->hw_ip->dump_region[REG_EXT3][idx++].end = 0x0FFF;
		itf_hwip->hw_ip->dump_region[REG_EXT3][idx].start = 0x3000;
		itf_hwip->hw_ip->dump_region[REG_EXT3][idx++].end = 0x3FFF;
		break;
	case DEV_HW_YPP:
		__is_hw_get_address(pdev, itf_hwip, hw_id, "YUVPP", IORESOURCE_YUVPP, REG_SETA, true);
		break;
	case DEV_HW_MCSC0:
		__is_hw_get_address(pdev, itf_hwip, hw_id, "MCSC0", IORESOURCE_MCSC, REG_SETA, true);
		break;
	default:
		probe_err("hw_id(%d) is invalid", hw_id);
		return -EINVAL;
	}

	return 0;
}

int is_hw_get_irq(void *itfc_data, void *pdev_data, int hw_id)
{
	struct is_interface_hwip *itf_hwip = NULL;
	struct platform_device *pdev = NULL;
	int ret = 0;

	FIMC_BUG(!itfc_data);

	itf_hwip = (struct is_interface_hwip *)itfc_data;
	pdev = (struct platform_device *)pdev_data;

	switch (hw_id) {
	case DEV_HW_3AA0:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 0);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq 3aa0-1\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 1);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq 3aa0-2\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP3] = platform_get_irq(pdev, 2);
		if (itf_hwip->irq[INTR_HWIP3] < 0) {
			err("Failed to get irq 3aa0 zsl dma\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP4] = platform_get_irq(pdev, 3);
		if (itf_hwip->irq[INTR_HWIP4] < 0) {
			err("Failed to get irq 3aa0 strp dma\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_3AA1:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 4);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq 3aa1-1\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 5);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq 3aa1-2\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP3] = platform_get_irq(pdev, 6);
		if (itf_hwip->irq[INTR_HWIP3] < 0) {
			err("Failed to get irq 3aa1 zsl dma\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP4] = platform_get_irq(pdev, 7);
		if (itf_hwip->irq[INTR_HWIP4] < 0) {
			err("Failed to get irq 3aa1 strp dma\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_3AA2:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 8);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq 3aa2-1\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 9);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq 3aa2-2\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP3] = platform_get_irq(pdev, 10);
		if (itf_hwip->irq[INTR_HWIP3] < 0) {
			err("Failed to get irq 3aa2 zsl dma\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP4] = platform_get_irq(pdev, 11);
		if (itf_hwip->irq[INTR_HWIP4] < 0) {
			err("Failed to get irq 3aa2 strp dma\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_3AA3:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 12);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq 3aa3-1\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 13);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq 3aa3-2\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP3] = platform_get_irq(pdev, 14);
		if (itf_hwip->irq[INTR_HWIP3] < 0) {
			err("Failed to get irq 3aa3 zsl dma\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP4] = platform_get_irq(pdev, 15);
		if (itf_hwip->irq[INTR_HWIP4] < 0) {
			err("Failed to get irq 3aa3 strp dma\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_LME0:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 16);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq lme0-0\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 17);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq lme0-1\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP3] = platform_get_irq(pdev, 18);
		if (itf_hwip->irq[INTR_HWIP3] < 0) {
			err("Failed to get irq orbmch0\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP4] = platform_get_irq(pdev, 19);
		if (itf_hwip->irq[INTR_HWIP4] < 0) {
			err("Failed to get irq orbmch1\n");
			return -EINVAL;
		}

		break;
	case DEV_HW_ISP0:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 20);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq isp0-1\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 21);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq isp0-2\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP3] = platform_get_irq(pdev, 22);
		if (itf_hwip->irq[INTR_HWIP3] < 0) {
			err("Failed to get irq mcfp0\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP4] = platform_get_irq(pdev, 23);
		if (itf_hwip->irq[INTR_HWIP4] < 0) {
			err("Failed to get irq mcfp1\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_YPP:
		itf_hwip->irq[INTR_HWIP5] = platform_get_irq(pdev, 24);
		if (itf_hwip->irq[INTR_HWIP5] < 0) {
			err("Failed to get irq yuvpp\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_MCSC0:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 25);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq mcsc0\n");
			return -EINVAL;
		}
		break;
	default:
		probe_err("hw_id(%d) is invalid", hw_id);
		return -EINVAL;
	}

	return ret;
}

static inline int __is_hw_request_irq(struct is_interface_hwip *itf_hwip,
	const char *name, int isr_num,
	unsigned int added_irq_flags,
	irqreturn_t (*func)(int, void *))
{
	size_t name_len = 0;
	int ret = 0;

	name_len = sizeof(itf_hwip->irq_name[isr_num]);
	snprintf(itf_hwip->irq_name[isr_num], name_len, "%s-%d", name, isr_num);

	ret = pablo_request_irq(itf_hwip->irq[isr_num], func,
		itf_hwip->irq_name[isr_num],
		added_irq_flags,
		itf_hwip);
	if (ret) {
		err_itfc("[HW:%s] request_irq [%d] fail", name, isr_num);
		return -EINVAL;
	}

	itf_hwip->handler[isr_num].id = isr_num;
	itf_hwip->handler[isr_num].valid = true;

	return ret;
}

static inline int __is_hw_free_irq(struct is_interface_hwip *itf_hwip, int isr_num)
{
	pablo_free_irq(itf_hwip->irq[isr_num], itf_hwip);

	return 0;
}

int is_hw_request_irq(void *itfc_data, int hw_id)
{
	struct is_interface_hwip *itf_hwip = NULL;
	int ret = 0;

	FIMC_BUG(!itfc_data);


	itf_hwip = (struct is_interface_hwip *)itfc_data;

	switch (hw_id) {
	case DEV_HW_3AA0:
		ret = __is_hw_request_irq(itf_hwip, "3a0-0", INTR_HWIP1, IRQF_TRIGGER_NONE, __is_isr1_3aa0);
		ret = __is_hw_request_irq(itf_hwip, "3a0-1", INTR_HWIP2, IRQF_TRIGGER_NONE, __is_isr2_3aa0);
		ret = __is_hw_request_irq(itf_hwip, "3a0-zsl", INTR_HWIP3, IRQF_TRIGGER_NONE, __is_isr3_3aa0);
		ret = __is_hw_request_irq(itf_hwip, "3a0-strp", INTR_HWIP4, IRQF_SHARED, __is_isr4_3aa0);
		break;
	case DEV_HW_3AA1:
		ret = __is_hw_request_irq(itf_hwip, "3a1-0", INTR_HWIP1, IRQF_TRIGGER_NONE, __is_isr1_3aa1);
		ret = __is_hw_request_irq(itf_hwip, "3a1-1", INTR_HWIP2, IRQF_TRIGGER_NONE, __is_isr2_3aa1);
		ret = __is_hw_request_irq(itf_hwip, "3a1-zsl", INTR_HWIP3, IRQF_TRIGGER_NONE, __is_isr3_3aa1);
		ret = __is_hw_request_irq(itf_hwip, "3a1-strp", INTR_HWIP4, IRQF_TRIGGER_NONE, __is_isr4_3aa1);
		break;
	case DEV_HW_3AA2:
		ret = __is_hw_request_irq(itf_hwip, "3a2-0", INTR_HWIP1, IRQF_TRIGGER_NONE, __is_isr1_3aa2);
		ret = __is_hw_request_irq(itf_hwip, "3a2-1", INTR_HWIP2, IRQF_TRIGGER_NONE, __is_isr2_3aa2);
		ret = __is_hw_request_irq(itf_hwip, "3a2-zsl", INTR_HWIP3, IRQF_SHARED, __is_isr3_3aa2);
		ret = __is_hw_request_irq(itf_hwip, "3a2-strp", INTR_HWIP4, IRQF_TRIGGER_NONE, __is_isr4_3aa2);
		break;
	case DEV_HW_3AA3:
		ret = __is_hw_request_irq(itf_hwip, "3a3-0", INTR_HWIP1, IRQF_TRIGGER_NONE, __is_isr1_3aa3);
		ret = __is_hw_request_irq(itf_hwip, "3a3-1", INTR_HWIP2, IRQF_TRIGGER_NONE, __is_isr2_3aa3);
		ret = __is_hw_request_irq(itf_hwip, "3a3-zsl", INTR_HWIP3, IRQF_SHARED, __is_isr3_3aa3);
		ret = __is_hw_request_irq(itf_hwip, "3a3-strp", INTR_HWIP4, IRQF_TRIGGER_NONE, __is_isr4_3aa3);
		break;
	case DEV_HW_LME0:
		ret = __is_hw_request_irq(itf_hwip, "lme-0", INTR_HWIP1, IRQF_TRIGGER_NONE, __is_isr1_lme);
		ret = __is_hw_request_irq(itf_hwip, "lme-1", INTR_HWIP2, IRQF_TRIGGER_NONE, __is_isr2_lme);
		/* To apply ORBMCH SW W/A, irq request for ORB was moved after power on */
		/*
		ret = __is_hw_request_irq(itf_hwip, "orbmch0", INTR_HWIP3, IRQF_TRIGGER_NONE, __is_isr3_orbmch0);
		ret = __is_hw_request_irq(itf_hwip, "orbmch1", INTR_HWIP4, IRQF_TRIGGER_NONE, __is_isr4_orbmch1);
		*/
		break;
	case DEV_HW_ISP0:
		ret = __is_hw_request_irq(itf_hwip, "itp-0", INTR_HWIP1, IRQF_TRIGGER_NONE, __is_isr1_itp0);
		ret = __is_hw_request_irq(itf_hwip, "itp-1", INTR_HWIP2, IRQF_TRIGGER_NONE, __is_isr2_itp0);
		ret = __is_hw_request_irq(itf_hwip, "mcfp0", INTR_HWIP3, IRQF_TRIGGER_NONE, __is_isr3_itp0);
		ret = __is_hw_request_irq(itf_hwip, "mcfp1", INTR_HWIP4, IRQF_TRIGGER_NONE, __is_isr4_itp0);
		break;
	case DEV_HW_YPP:
		ret = __is_hw_request_irq(itf_hwip, "yuvpp", INTR_HWIP5, IRQF_TRIGGER_NONE, __is_isr1_yuvpp);
		break;
	case DEV_HW_MCSC0:
		ret = __is_hw_request_irq(itf_hwip, "mcs0", INTR_HWIP1, IRQF_TRIGGER_NONE, __is_isr1_mcs0);
		break;
	default:
		probe_err("hw_id(%d) is invalid", hw_id);
		return -EINVAL;
	}

	return ret;
}

int is_hw_s_ctrl(void *itfc_data, int hw_id, enum hw_s_ctrl_id id, void *val)
{
	int ret = 0;

	switch (id) {
	case HW_S_CTRL_FULL_BYPASS:
		break;
	case HW_S_CTRL_CHAIN_IRQ:
		break;
	case HW_S_CTRL_HWFC_IDX_RESET:
		if (hw_id == IS_VIDEO_M3P_NUM) {
			struct is_video_ctx *vctx = (struct is_video_ctx *)itfc_data;
			struct is_device_ischain *device;
			unsigned long data = (unsigned long)val;

			FIMC_BUG(!vctx);
			FIMC_BUG(!GET_DEVICE(vctx));

			device = GET_DEVICE(vctx);

			/* reset if this instance is reprocessing */
			if (test_bit(IS_ISCHAIN_REPROCESSING, &device->state))
				writel(data, hwfc_rst);
		}
		break;
	case HW_S_CTRL_MCSC_SET_INPUT:
		{
			unsigned long mode = (unsigned long)val;

			info_itfc("%s: mode(%lu)\n", __func__, mode);
		}
		break;
	default:
		break;
	}

	return ret;
}

void __iomem *is_hw_get_sysreg(ulong core_regs)
{
	if (core_regs)
		err_itfc("%s: core_regs(%p)\n", __func__, (void *)core_regs);

	/* deprecated */

	return NULL;
}

u32 is_hw_find_settle(u32 mipi_speed, u32 use_cphy)
{
	u32 align_mipi_speed;
	u32 find_mipi_speed;
	const u32 *settle_table;
	size_t max;
	int s, e, m;

	if (use_cphy) {
		settle_table = is_csi_settle_table_cphy;
		max = sizeof(is_csi_settle_table_cphy) / sizeof(u32);
	} else {
		settle_table = is_csi_settle_table;
		max = sizeof(is_csi_settle_table) / sizeof(u32);
	}
	align_mipi_speed = ALIGN(mipi_speed, 10);

	s = 0;
	e = max - 2;

	if (settle_table[s] < align_mipi_speed)
		return settle_table[s + 1];

	if (settle_table[e] > align_mipi_speed)
		return settle_table[e + 1];

	/* Binary search */
	while (s <= e) {
		m = ALIGN((s + e) / 2, 2);
		find_mipi_speed = settle_table[m];

		if (find_mipi_speed == align_mipi_speed)
			break;
		else if (find_mipi_speed > align_mipi_speed)
			s = m + 2;
		else
			e = m - 2;
	}

	return settle_table[m + 1];
}

unsigned int get_dma(struct is_device_sensor *device, u32 *dma_ch)
{
	*dma_ch = 0;

	return 0;
}

void is_hw_camif_init(void)
{
	/* TODO */
}

#ifdef USE_CAMIF_FIX_UP
int is_hw_camif_fix_up(struct is_device_sensor *sensor)
{
	int ret = 0;

	/* TODO */

	return ret;
}
#endif

int is_hw_camif_cfg(void *sensor_data)
{
	struct is_device_sensor *sensor;
	struct is_device_csi *csi;
	struct is_fid_loc *fid_loc;
	void __iomem *sysreg_csis;
	u32 val = 0;

	FIMC_BUG(!sensor_data);

	sensor = sensor_data;
	csi = (struct is_device_csi *)v4l2_get_subdevdata(sensor->subdev_csi);
	fid_loc = &sensor->cfg->fid_loc;
	sysreg_csis = ioremap(SYSREG_CSIS_BASE_ADDR, 0x1000);

	if (csi->f_id_dec) {
		if (!fid_loc->valid) {
			fid_loc->byte = 27;
			fid_loc->line = 0;
			mwarn("fid_loc is NOT properly set.\n", sensor);
		}

		val = is_hw_get_reg(sysreg_csis,
				&sysreg_csis_regs[SYSREG_R_CSIS_CSIS0_FRAME_ID_EN + csi->ch]);
		val = is_hw_set_field_value(val,
				&sysreg_csis_fields[SYSREG_F_CSIS_FID_LOC_BYTE],
				fid_loc->byte);
		val = is_hw_set_field_value(val,
				&sysreg_csis_fields[SYSREG_F_CSIS_FID_LOC_LINE],
				fid_loc->line);
		val = is_hw_set_field_value(val,
				&sysreg_csis_fields[SYSREG_F_CSIS_FRAME_ID_EN_CSIS],
				1);

		info("SYSREG_R_CSIS%d_FRAME_ID_EN:byte %d line %d\n",
				csi->ch, fid_loc->byte, fid_loc->line);
	} else {
		val = 0;
	}

	is_hw_set_reg(sysreg_csis,
		&sysreg_csis_regs[SYSREG_R_CSIS_CSIS0_FRAME_ID_EN + csi->ch],
		val);

	iounmap(sysreg_csis);

	return 0;
}

void is_hw_ischain_qe_cfg(void)
{
	dbg_hw(2, "%s()\n", __func__);
}

int blk_dns_mux_control(struct is_device_ischain *device, u32 value)
{
	void __iomem *dns_sys_regs;
	u32 dns_val = 0;
	int ret = 0;

	FIMC_BUG(!device);
	dns_sys_regs = ioremap(SYSREG_DNS_BASE_ADDR, 0x1000);

	/* DNS input mux setting */
	/* DNS0 <- TNR */
	dns_val = is_hw_get_reg(dns_sys_regs,
				&sysreg_dns_regs[SYSREG_R_DNS_DNS_USER_CON0]);
	dns_val = is_hw_set_field_value(dns_val,
				&sysreg_dns_fields[SYSREG_F_DNS_GLUEMUX_DNS0_VAL], value);

	info("SYSREG_R_DNS_USER_CON0:(0x%08X)\n", dns_val);
	is_hw_set_reg(dns_sys_regs, &sysreg_dns_regs[SYSREG_R_DNS_DNS_USER_CON0], dns_val);

	iounmap(dns_sys_regs);
	return ret;
}

int is_hw_ischain_cfg(void *ischain_data)
{
	int ret = 0;
	struct is_device_ischain *device;

	FIMC_BUG(!ischain_data);

	device = (struct is_device_ischain *)ischain_data;
	if (test_bit(IS_ISCHAIN_REPROCESSING, &device->state))
		return ret;

	return ret;
}

int is_hw_ischain_enable(struct is_device_ischain *device)
{
	int ret = 0;
	struct is_interface_hwip *itf_hwip = NULL;
	struct is_core *core = is_get_is_core();
	struct is_interface_ischain *itfc;
	struct is_hardware *hw;
	int hw_slot;
#ifdef USE_ORBMCH_WA
	struct is_hw_ip *hw_ip = NULL;
#endif

	FIMC_BUG(!device);

	itfc = &core->interface_ischain;
	hw = &core->hardware;

	ret = blk_dns_mux_control(device, 1);
	if (!ret)
		merr("blk_dns_mux_control is failed (%d)\n", device, ret);

#ifdef USE_ORBMCH_WA
	info("%s: SW WA for ORBMCH\n", __func__);
	hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, DEV_HW_LME0);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	hw_ip = itf_hwip->hw_ip;
	writel(0x200, hw_ip->regs[REG_EXT1] + 0x64);
	writel(0x200, hw_ip->regs[REG_EXT2] + 0x64);
	ret = __is_hw_request_irq(itf_hwip, "orbmch0", INTR_HWIP3, IRQF_TRIGGER_NONE, __is_isr3_orbmch0);
	ret = __is_hw_request_irq(itf_hwip, "orbmch1", INTR_HWIP4, IRQF_TRIGGER_NONE, __is_isr4_orbmch1);
#endif

	/* irq affinity should be restored after S2R at gic600 */
	hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, DEV_HW_3AA0);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP1], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP2], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP3], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP4], true);

	hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, DEV_HW_3AA1);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP1], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP2], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP3], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP4], true);

	hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, DEV_HW_3AA2);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP1], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP2], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP3], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP4], true);

	hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, DEV_HW_3AA3);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP1], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP2], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP3], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP4], true);

	hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, DEV_HW_LME0);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP1], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP2], true);

	hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, DEV_HW_ISP0);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP1], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP2], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP3], true);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP4], true);

	hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, DEV_HW_YPP);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP5], true);

	hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, DEV_HW_MCSC0);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	pablo_set_affinity_irq(itf_hwip->irq[INTR_HWIP1], true);

	votfitf_disable_service();

	info("%s: complete\n", __func__);

	return ret;
}

int is_hw_ischain_disable(struct is_device_ischain *device)
{
	int ret = 0;
#ifdef USE_ORBMCH_WA
	struct is_interface_hwip *itf_hwip = NULL;
	int hw_slot = -1;
	struct is_core *core = is_get_is_core();
	struct is_interface_ischain *itfc = &core->interface_ischain;
#endif

	FIMC_BUG(!device);

#ifdef USE_ORBMCH_WA
	hw_slot = CALL_HW_CHAIN_INFO_OPS(&core->hardware, get_hw_slot_id, DEV_HW_LME0);
	itf_hwip = &(itfc->itf_ip[hw_slot]);
	ret = __is_hw_free_irq(itf_hwip, INTR_HWIP3);
	ret = __is_hw_free_irq(itf_hwip, INTR_HWIP4);
#endif

	info("%s: complete\n", __func__);

	return ret;
}

/* TODO: remove this, compile check only */
#ifdef ENABLE_HWACG_CONTROL
void is_hw_csi_qchannel_enable_all(bool enable)
{
	void __iomem *csi0_regs;
	void __iomem *csi1_regs;
	void __iomem *csi2_regs;
	void __iomem *csi3_regs;
	void __iomem *csi3_1_regs;

	u32 reg_val;

	csi0_regs = ioremap(CSIS0_QCH_EN_ADDR, SZ_4);
	csi1_regs = ioremap(CSIS1_QCH_EN_ADDR, SZ_4);
	csi2_regs = ioremap(CSIS2_QCH_EN_ADDR, SZ_4);
	csi3_regs = ioremap(CSIS3_QCH_EN_ADDR, SZ_4);
	csi3_1_regs = ioremap(CSIS3_1_QCH_EN_ADDR, SZ_4);

	reg_val = readl(csi0_regs);
	reg_val &= ~(1 << 20);
	writel(enable << 20 | reg_val, csi0_regs);

	reg_val = readl(csi1_regs);
	reg_val &= ~(1 << 20);
	writel(enable << 20 | reg_val, csi1_regs);

	reg_val = readl(csi2_regs);
	reg_val &= ~(1 << 20);
	writel(enable << 20 | reg_val, csi2_regs);

	reg_val = readl(csi3_regs);
	reg_val &= ~(1 << 20);
	writel(enable << 20 | reg_val, csi3_regs);

	reg_val = readl(csi3_1_regs);
	reg_val &= ~(1 << 20);
	writel(enable << 20 | reg_val, csi3_1_regs);

	iounmap(csi0_regs);
	iounmap(csi1_regs);
	iounmap(csi2_regs);
	iounmap(csi3_regs);
	iounmap(csi3_1_regs);
}
#endif

void __nocfi is_hw_interrupt_relay(struct is_group *group, void *hw_ip_data)
{
	struct is_group *child;
	struct is_hw_ip *hw_ip = (struct is_hw_ip *)hw_ip_data;
	struct is_hw_ip *hw_ip_child;
	struct hwip_intr_handler *intr_handler;
	int hw_list[GROUP_HW_MAX];

	child = group->child;
	if (child) {
		is_get_hw_list(child->id, hw_list);
		hw_ip_child = CALL_HW_CHAIN_INFO_OPS(hw_ip->hardware, get_hw_ip, hw_list[0]);
		if (!hw_ip_child) {
			serr_hw("invalid hw_id (%d)", hw_ip, hw_list[0]);
		} else {
			intr_handler = hw_ip_child->intr_handler[INTR_HWIP8];
			if (intr_handler && intr_handler->handler) {
				is_fpsimd_get_isr();
				intr_handler->handler(intr_handler->id, intr_handler->ctx);
				is_fpsimd_put_isr();
			}
		}
	}
}

static struct is_llc_way_num is_llc_way[IS_LLC_SN_END] = {
	/* default : VOTF 0MB, MCFP 0MB */
	[IS_LLC_SN_DEFAULT].votf = 0,
	[IS_LLC_SN_DEFAULT].mcfp = 0,
	/* FHD scenario : VOTF 5MB, MCFP 2MB */
	[IS_LLC_SN_FHD].votf = 10,
	[IS_LLC_SN_FHD].mcfp = 4,
	/* UHD scenario : VOTF 5MB, MCFP 2MB */
	[IS_LLC_SN_UHD].votf = 10,
	[IS_LLC_SN_UHD].mcfp = 4,
	/* 8K scenario : VOTF 2MB, MCFP 2MB */
	[IS_LLC_SN_8K].votf = 4,
	[IS_LLC_SN_8K].mcfp = 4,
	/* preview scenario : VOTF 5.5MB, MCFP 1.5MB */
	[IS_LLC_SN_PREVIEW].votf = 11,
	[IS_LLC_SN_PREVIEW].mcfp = 3,
};

void is_hw_configure_llc(bool on, struct is_device_ischain *device, ulong *llc_state)
{
#if IS_ENABLED(CONFIG_EXYNOS_SCI)
	struct is_dvfs_scenario_param param;
	int votf, mcfp;

	if (test_bit(LLC_DISABLE, llc_state)) {
		cam_llc_enable(on);
		minfo("[LLC] disable(%d)\n", device, on);
		return;
	}

	is_hw_dvfs_init_face_mask(device, &param);
	is_hw_dvfs_get_scenario_param(device, 0, &param);

	/* way 1 means alloc 512K LLC */
	if (on) {
		if (param.sensor == IS_DVFS_SENSOR_FRONT_VT) {
			/* Front VT not use LLC */
			votf = is_llc_way[IS_LLC_SN_DEFAULT].votf;
			mcfp = is_llc_way[IS_LLC_SN_DEFAULT].mcfp;
		} else if (param.mode == IS_DVFS_MODE_PHOTO) {
			votf = is_llc_way[IS_LLC_SN_PREVIEW].votf;
			mcfp = is_llc_way[IS_LLC_SN_PREVIEW].mcfp;
		} else if (param.mode == IS_DVFS_MODE_VIDEO) {
			switch (param.resol) {
			case IS_DVFS_RESOL_FHD:
				votf = is_llc_way[IS_LLC_SN_FHD].votf;
				mcfp = is_llc_way[IS_LLC_SN_FHD].mcfp;
				break;
			case IS_DVFS_RESOL_UHD:
				votf = is_llc_way[IS_LLC_SN_UHD].votf;
				mcfp = is_llc_way[IS_LLC_SN_UHD].mcfp;
				break;
			case IS_DVFS_RESOL_8K:
				votf = is_llc_way[IS_LLC_SN_8K].votf;
				mcfp = is_llc_way[IS_LLC_SN_8K].mcfp;
				break;
			default:
				votf = is_llc_way[IS_LLC_SN_DEFAULT].votf;
				mcfp = is_llc_way[IS_LLC_SN_DEFAULT].mcfp;
				break;
			}
		} else {
			votf = is_llc_way[IS_LLC_SN_DEFAULT].votf;
			mcfp = is_llc_way[IS_LLC_SN_DEFAULT].mcfp;
		}

		if (votf) {
			llc_region_alloc(LLC_REGION_CAM_CSIS, 1, votf);
			set_bit(LLC_REGION_CAM_CSIS, llc_state);
		}

		if (mcfp) {
			llc_region_alloc(LLC_REGION_CAM_MCFP, 1, mcfp);
			set_bit(LLC_REGION_CAM_MCFP, llc_state);
		}

		info("[LLC] alloc, VOTF:%d.%dMB, MCFP:%d.%dMB",
				votf / 2, (votf % 2) ? 5 : 0,
				mcfp / 2, (mcfp % 2) ? 5 : 0);
	} else {
		if (test_and_clear_bit(LLC_REGION_CAM_CSIS, llc_state))
			llc_region_alloc(LLC_REGION_CAM_CSIS, 0, 0);
		if (test_and_clear_bit(LLC_REGION_CAM_MCFP, llc_state))
			llc_region_alloc(LLC_REGION_CAM_MCFP, 0, 0);

		info("[LLC] release");
	}

	cam_llc_enable(on);
#endif
}

void is_hw_configure_bts_scen(struct is_resourcemgr *resourcemgr, int scenario_id)
{
	int bts_index = 0;

	switch (scenario_id) {
	case IS_DVFS_SN_REAR_SINGLE_TELE_VIDEO_8K24:
	case IS_DVFS_SN_REAR_SINGLE_TELE_VIDEO_8K30:
	case IS_DVFS_SN_REAR_SINGLE_WIDE_VIDEO_8K24:
	case IS_DVFS_SN_REAR_SINGLE_WIDE_VIDEO_8K30:
	case IS_DVFS_SN_TRIPLE_PHOTO:
	case IS_DVFS_SN_TRIPLE_VIDEO_FHD30:
	case IS_DVFS_SN_TRIPLE_VIDEO_UHD30:
	case IS_DVFS_SN_TRIPLE_VIDEO_FHD60:
	case IS_DVFS_SN_TRIPLE_VIDEO_UHD60:
	case IS_DVFS_SN_TRIPLE_CAPTURE:
		bts_index = 1;
		break;
	default:
		bts_index = 0;
		break;
	}

	/* If default scenario & specific scenario selected,
	 * off specific scenario first.
	 */
	if (resourcemgr->cur_bts_scen_idx && bts_index == 0)
		is_bts_scen(resourcemgr, resourcemgr->cur_bts_scen_idx, false);

	if (bts_index && bts_index != resourcemgr->cur_bts_scen_idx)
		is_bts_scen(resourcemgr, bts_index, true);
	resourcemgr->cur_bts_scen_idx = bts_index;
}

int is_hw_get_output_slot(u32 vid)
{
	int ret = -1;

	switch (vid) {
	case IS_VIDEO_SS0_NUM:
	case IS_VIDEO_SS1_NUM:
	case IS_VIDEO_SS2_NUM:
	case IS_VIDEO_SS3_NUM:
	case IS_VIDEO_SS4_NUM:
	case IS_VIDEO_SS5_NUM:
	case IS_VIDEO_PAF0S_NUM:
	case IS_VIDEO_PAF1S_NUM:
	case IS_VIDEO_PAF2S_NUM:
	case IS_VIDEO_PAF3S_NUM:
	case IS_VIDEO_30S_NUM:
	case IS_VIDEO_31S_NUM:
	case IS_VIDEO_32S_NUM:
	case IS_VIDEO_33S_NUM:
	case IS_VIDEO_I0S_NUM:
	case IS_VIDEO_YPP_NUM:
	case IS_VIDEO_LME0_NUM:
		ret = 0;
		break;
	default:
		ret = -1;
	}

	return ret;
}

int is_hw_get_capture_slot(struct is_frame *frame, dma_addr_t **taddr, u64 **taddr_k, u32 vid)
{
	int ret = 0;

	switch(vid) {
	/* TAA */
	case IS_VIDEO_30C_NUM:
	case IS_VIDEO_31C_NUM:
	case IS_VIDEO_32C_NUM:
	case IS_VIDEO_33C_NUM:
		*taddr = frame->txcTargetAddress;
		break;
	case IS_VIDEO_30P_NUM:
	case IS_VIDEO_31P_NUM:
	case IS_VIDEO_32P_NUM:
	case IS_VIDEO_33P_NUM:
		*taddr = frame->txpTargetAddress;
		break;
	case IS_VIDEO_30G_NUM:
	case IS_VIDEO_31G_NUM:
	case IS_VIDEO_32G_NUM:
	case IS_VIDEO_33G_NUM:
		*taddr = frame->mrgTargetAddress;
		break;
	case IS_VIDEO_30F_NUM:
	case IS_VIDEO_31F_NUM:
	case IS_VIDEO_32F_NUM:
	case IS_VIDEO_33F_NUM:
		*taddr = frame->efdTargetAddress;
		break;
	case IS_VIDEO_30D_NUM:
	case IS_VIDEO_31D_NUM:
	case IS_VIDEO_32D_NUM:
	case IS_VIDEO_33D_NUM:
		*taddr = frame->txdgrTargetAddress;
		break;
	case IS_VIDEO_30O_NUM:
	case IS_VIDEO_31O_NUM:
	case IS_VIDEO_32O_NUM:
	case IS_VIDEO_33O_NUM:
		*taddr = frame->txodsTargetAddress;
		break;
	case IS_VIDEO_30L_NUM:
	case IS_VIDEO_31L_NUM:
	case IS_VIDEO_32L_NUM:
	case IS_VIDEO_33L_NUM:
		*taddr = frame->txldsTargetAddress;
		break;
	case IS_VIDEO_30H_NUM:
	case IS_VIDEO_31H_NUM:
	case IS_VIDEO_32H_NUM:
	case IS_VIDEO_33H_NUM:
		*taddr = frame->txhfTargetAddress;
		break;
	/* ISP */
	case IS_VIDEO_I0C_NUM:
		*taddr = frame->ixcTargetAddress;
		break;
	case IS_VIDEO_I0P_NUM:
		*taddr = frame->ixpTargetAddress;
		break;
	case IS_VIDEO_I0V_NUM:
		*taddr = frame->ixvTargetAddress;
		break;
	case IS_VIDEO_I0W_NUM:
		*taddr = frame->ixwTargetAddress;
		break;
	case IS_VIDEO_I0T_NUM:
		*taddr = frame->ixtTargetAddress;
		break;
	case IS_VIDEO_I0G_NUM:
		*taddr = frame->ixgTargetAddress;
		break;
	case IS_VIDEO_IMM_NUM:
		*taddr = frame->ixmTargetAddress;
		if (taddr_k)
			*taddr_k = frame->ixmKTargetAddress;
		break;
	case IS_VIDEO_IRG_NUM:
		*taddr = frame->ixrrgbTargetAddress;
		break;
	case IS_VIDEO_ISC_NUM:
		*taddr = frame->ixscmapTargetAddress;
		break;
	case IS_VIDEO_IDR_NUM:
		*taddr = frame->ixdgrTargetAddress;
		break;
	case IS_VIDEO_INR_NUM:
		*taddr = frame->ixnoirTargetAddress;
		break;
	case IS_VIDEO_IND_NUM:
		*taddr = frame->ixnrdsTargetAddress;
		break;
	case IS_VIDEO_IDG_NUM:
		*taddr = frame->ixdgaTargetAddress;
		break;
	case IS_VIDEO_ISH_NUM:
		*taddr = frame->ixsvhistTargetAddress;
		break;
	case IS_VIDEO_IHF_NUM:
		*taddr = frame->ixhfTargetAddress;
		break;
	case IS_VIDEO_INW_NUM:
		*taddr = frame->ixnoiTargetAddress;
		break;
	case IS_VIDEO_INRW_NUM:
		*taddr = frame->ixnoirwTargetAddress;
		break;
	case IS_VIDEO_IRGW_NUM:
		*taddr = frame->ixwrgbTargetAddress;
		break;
	case IS_VIDEO_INB_NUM:
		*taddr = frame->ixbnrTargetAddress;
		break;
	/* YUVPP */
	case IS_VIDEO_YND_NUM:
		*taddr = frame->ypnrdsTargetAddress;
		break;
	case IS_VIDEO_YDG_NUM:
		*taddr = frame->ypdgaTargetAddress;
		break;
	case IS_VIDEO_YSH_NUM:
		*taddr = frame->ypsvhistTargetAddress;
		break;
	case IS_VIDEO_YNS_NUM:
		*taddr = frame->ypnoiTargetAddress;
		break;
	/* MCSC */
	case IS_VIDEO_M0P_NUM:
		*taddr = frame->sc0TargetAddress;
		break;
	case IS_VIDEO_M1P_NUM:
		*taddr = frame->sc1TargetAddress;
		break;
	case IS_VIDEO_M2P_NUM:
		*taddr = frame->sc2TargetAddress;
		break;
	case IS_VIDEO_M3P_NUM:
		*taddr = frame->sc3TargetAddress;
		break;
	case IS_VIDEO_M4P_NUM:
		*taddr = frame->sc4TargetAddress;
		break;
	case IS_VIDEO_M5P_NUM:
		*taddr = frame->sc5TargetAddress;
		break;
	/* LME */
	case IS_VIDEO_LME0S_NUM:
		*taddr = frame->lmesTargetAddress;
		if (taddr_k)
			*taddr_k = frame->lmesKTargetAddress;
		break;
	case IS_VIDEO_LME0C_NUM:
		*taddr = frame->lmecTargetAddress;
		if (taddr_k)
			*taddr_k = frame->lmecKTargetAddress;
		break;
	case IS_VIDEO_LME0M_NUM:
		/* No DMA out */
		if (taddr_k)
			*taddr_k = frame->lmemKTargetAddress;
		break;
	default:
		err_hw("Unsupported vid(%d)", vid);
		ret = -EINVAL;
	}

	/* Clear subdev frame's target address before set */
	if (taddr && *taddr)
		memset(*taddr, 0x0, sizeof(typeof(**taddr)) * IS_MAX_PLANES);
	if (taddr_k && *taddr_k)
		memset(*taddr_k, 0x0, sizeof(typeof(**taddr_k)) * IS_MAX_PLANES);

	return ret;
}

void * is_get_dma_blk(int type)
{
	struct is_lib_support *lib = is_get_lib_support();
	struct lib_mem_block * mblk = NULL;

	switch (type) {
	case ID_DMA_3AAISP:
		mblk = &lib->mb_dma_taaisp;
		break;
	case ID_DMA_MEDRC:
		mblk = &lib->mb_dma_medrc;
		break;
	case ID_DMA_ORBMCH:
		mblk = &lib->mb_dma_orbmch;
		break;
	case ID_DMA_TNR:
		mblk = &lib->mb_dma_tnr;
		break;
	case ID_DMA_CLAHE:
		mblk = &lib->mb_dma_clahe;
		break;
	default:
		err_hw("Invalid DMA type: %d\n", type);
		return NULL;
	}

	return (void *)mblk;
}

int is_get_dma_id(u32 vid)
{
	u32 dma_id;

	switch (vid) {
	case IS_VIDEO_LME0C_NUM:
		dma_id = ID_DBUF_LMEC;
		break;
	default:
		dma_id = -EINVAL;
		break;
	}

	return dma_id;
}

int is_hw_check_changed_chain_id(struct is_hardware *hardware, u32 hw_id, u32 instance)
{
	int hw_idx;
	int mapped_hw_id = 0;

	FIMC_BUG(!hardware);

	switch (hw_id) {
	case DEV_HW_PAF0:
	case DEV_HW_PAF1:
	case DEV_HW_PAF2:
	case DEV_HW_PAF3:
		for (hw_idx = DEV_HW_PAF0; hw_idx <= DEV_HW_PAF3; hw_idx++) {
			if (test_bit(hw_idx, &hardware->hw_map[instance]) && hw_idx != hw_id) {
				mapped_hw_id = hw_idx;
				break;
			}
		}
		break;
	case DEV_HW_3AA0:
	case DEV_HW_3AA1:
	case DEV_HW_3AA2:
	case DEV_HW_3AA3:
		for (hw_idx = DEV_HW_3AA0; hw_idx <= DEV_HW_3AA3; hw_idx++) {
			if (test_bit(hw_idx, &hardware->hw_map[instance]) && hw_idx != hw_id) {
				mapped_hw_id = hw_idx;
				break;
			}
		}
		break;
	default:
		break;
	}

	return mapped_hw_id;
}

void is_hw_fill_target_address(u32 hw_id, struct is_frame *dst, struct is_frame *src,
	bool reset)
{
	/* A previous address should not be cleared for sysmmu debugging. */
	reset = false;

	switch (hw_id) {
	case DEV_HW_PAF0:
	case DEV_HW_PAF1:
	case DEV_HW_PAF2:
		break;
	case DEV_HW_3AA0:
	case DEV_HW_3AA1:
	case DEV_HW_3AA2:
	case DEV_HW_3AA3:
		TADDR_COPY(dst, src, txcTargetAddress, reset);
		TADDR_COPY(dst, src, txpTargetAddress, reset);
		TADDR_COPY(dst, src, mrgTargetAddress, reset);
		TADDR_COPY(dst, src, efdTargetAddress, reset);
		TADDR_COPY(dst, src, txdgrTargetAddress, reset);
		TADDR_COPY(dst, src, txodsTargetAddress, reset);
		TADDR_COPY(dst, src, txldsTargetAddress, reset);
		TADDR_COPY(dst, src, txhfTargetAddress, reset);
		break;
	case DEV_HW_LME0:
		TADDR_COPY(dst, src, lmesTargetAddress, reset);
		TADDR_COPY(dst, src, lmesKTargetAddress, reset);
		TADDR_COPY(dst, src, lmecTargetAddress, reset);
		TADDR_COPY(dst, src, lmecKTargetAddress, reset);
		TADDR_COPY(dst, src, lmemKTargetAddress, reset);
		break;
	case DEV_HW_ISP0:
		TADDR_COPY(dst, src, ixcTargetAddress, reset);
		TADDR_COPY(dst, src, ixpTargetAddress, reset);
		TADDR_COPY(dst, src, ixtTargetAddress, reset);
		TADDR_COPY(dst, src, ixgTargetAddress, reset);
		TADDR_COPY(dst, src, ixvTargetAddress, reset);
		TADDR_COPY(dst, src, ixwTargetAddress, reset);
		TADDR_COPY(dst, src, ixmTargetAddress, reset);
		TADDR_COPY(dst, src, ixmKTargetAddress, reset);
		break;
	case DEV_HW_YPP:
		TADDR_COPY(dst, src, ixdgrTargetAddress, reset);
		TADDR_COPY(dst, src, ixrrgbTargetAddress, reset);
		TADDR_COPY(dst, src, ixnoirTargetAddress, reset);
		TADDR_COPY(dst, src, ixscmapTargetAddress, reset);
		TADDR_COPY(dst, src, ixnrdsTargetAddress, reset);
		TADDR_COPY(dst, src, ixdgaTargetAddress, reset);
		TADDR_COPY(dst, src, ixnrdsTargetAddress, reset);
		TADDR_COPY(dst, src, ixhfTargetAddress, reset);
		TADDR_COPY(dst, src, ixwrgbTargetAddress, reset);
		TADDR_COPY(dst, src, ixnoirwTargetAddress, reset);
		TADDR_COPY(dst, src, ixbnrTargetAddress, reset);
		TADDR_COPY(dst, src, ixnoiTargetAddress, reset);

		TADDR_COPY(dst, src, ypnrdsTargetAddress, reset);
		TADDR_COPY(dst, src, ypnoiTargetAddress, reset);
		TADDR_COPY(dst, src, ypdgaTargetAddress, reset);
		TADDR_COPY(dst, src, ypsvhistTargetAddress, reset);
		break;
	case DEV_HW_MCSC0:
		TADDR_COPY(dst, src, sc0TargetAddress, reset);
		TADDR_COPY(dst, src, sc1TargetAddress, reset);
		TADDR_COPY(dst, src, sc2TargetAddress, reset);
		TADDR_COPY(dst, src, sc3TargetAddress, reset);
		TADDR_COPY(dst, src, sc4TargetAddress, reset);
		TADDR_COPY(dst, src, sc5TargetAddress, reset);
		break;
	default:
		err("[%d] Invalid hw id(%d)", src->instance, hw_id);
		break;
	}
}

static struct is_video i_video[PABLO_VIDEO_PROBE_MAX];
void is_hw_chain_probe(void *core)
{
	/* PDP */
	is_pafs_video_probe(&i_video[PABLO_VIDEO_PROBE_PAF0S], core, IS_VIDEO_PAF0S_NUM, 0);
	is_pafs_video_probe(&i_video[PABLO_VIDEO_PROBE_PAF1S], core, IS_VIDEO_PAF1S_NUM, 1);
	is_pafs_video_probe(&i_video[PABLO_VIDEO_PROBE_PAF2S], core, IS_VIDEO_PAF2S_NUM, 2);
	is_pafs_video_probe(&i_video[PABLO_VIDEO_PROBE_PAF3S], core, IS_VIDEO_PAF3S_NUM, 3);

	/* 3AA */
	is_3as_video_probe(&i_video[PABLO_VIDEO_PROBE_30S], core, IS_VIDEO_30S_NUM, 0);
	is_3as_video_probe(&i_video[PABLO_VIDEO_PROBE_31S], core, IS_VIDEO_31S_NUM, 1);
	is_3as_video_probe(&i_video[PABLO_VIDEO_PROBE_32S], core, IS_VIDEO_32S_NUM, 2);
	is_3as_video_probe(&i_video[PABLO_VIDEO_PROBE_33S], core, IS_VIDEO_33S_NUM, 3);

	/* ISP */
	is_isps_video_probe(&i_video[PABLO_VIDEO_PROBE_I0S], core, IS_VIDEO_I0S_NUM, 0);

	/* YUVPP */
	is_ypp_video_probe(&i_video[PABLO_VIDEO_PROBE_YPP], core, IS_VIDEO_YUVP, 0);

	/* MCSC */
	is_mcs_video_probe(&i_video[PABLO_VIDEO_PROBE_M0S], core, IS_VIDEO_M0S_NUM, 0);

	/* LME */
	is_lme_video_probe(&i_video[PABLO_VIDEO_PROBE_LME0], core, IS_VIDEO_LME0_NUM, 0);
}

struct is_mem *is_hw_get_iommu_mem(u32 vid)
{
	struct is_core *core = is_get_is_core();

	return &core->resourcemgr.mem;
}

void is_hw_print_target_dva(struct is_frame *leader_frame, u32 instance)
{
	u32 i;
#if defined(CSTAT_CTX_NUM)
	u32 ctx;
#endif

	for (i = 0; i < IS_MAX_PLANES; i++) {

#if IS_ENABLED(SOC_30C)
		IS_PRINT_TARGET_DVA(txcTargetAddress);
#endif
#if defined(CSTAT_CTX_NUM)
		for (ctx = 0; ctx < CSTAT_CTX_NUM; ctx++)
			IS_PRINT_TARGET_DVA(txpTargetAddress[ctx]);
#else
		IS_PRINT_TARGET_DVA(txpTargetAddress);
#endif

#if IS_ENABLED(SOC_30G)
		IS_PRINT_TARGET_DVA(mrgTargetAddress);
#endif
#if defined(CSTAT_CTX_NUM)
		for (ctx = 0; ctx < CSTAT_CTX_NUM; ctx++)
			IS_PRINT_TARGET_DVA(efdTargetAddress[ctx]);
#else
		IS_PRINT_TARGET_DVA(efdTargetAddress);
#endif
#if IS_ENABLED(LOGICAL_VIDEO_NODE)
#if defined(CSTAT_CTX_NUM)
		for (ctx = 0; ctx < CSTAT_CTX_NUM; ctx++)
			IS_PRINT_TARGET_DVA(txdgrTargetAddress[ctx]);
#else
		IS_PRINT_TARGET_DVA(txdgrTargetAddress);
#endif
#endif
#if defined(ENABLE_ORBDS)
		IS_PRINT_TARGET_DVA(txodsTargetAddress);
#endif
#if defined(ENABLE_LMEDS)
#if defined(CSTAT_CTX_NUM)
		for (ctx = 0; ctx < CSTAT_CTX_NUM; ctx++)
			IS_PRINT_TARGET_DVA(txldsTargetAddress[ctx]);
#else
		IS_PRINT_TARGET_DVA(txldsTargetAddress);
#endif
#endif
#if defined(ENABLE_LMEDS1)
#if defined(CSTAT_CTX_NUM)
		for (ctx = 0; ctx < CSTAT_CTX_NUM; ctx++)
			IS_PRINT_TARGET_DVA(dva_cstat_lmeds1[ctx]);
#else
		IS_PRINT_TARGET_DVA(dva_cstat_lmeds1);
#endif
#endif
#if defined(ENABLE_HF) && IS_ENABLED(SOC_30S)
		IS_PRINT_TARGET_DVA(txhfTargetAddress);
#endif
#if IS_ENABLED(SOC_CSTAT_SVHIST)
#if defined(CSTAT_CTX_NUM)
		for (ctx = 0; ctx < CSTAT_CTX_NUM; ctx++)
			IS_PRINT_TARGET_DVA(dva_cstat_vhist[ctx]);
#else
		IS_PRINT_TARGET_DVA(dva_cstat_vhist);
#endif
#endif
#if IS_ENABLED(SOC_LME0)
		IS_PRINT_TARGET_DVA(lmesTargetAddress);
		IS_PRINT_TARGET_DVA(lmecTargetAddress);
#endif
#if IS_ENABLED(ENABLE_BYRP_HDR)
		IS_PRINT_TARGET_DVA(dva_byrp_hdr);
#endif
		IS_PRINT_TARGET_DVA(ixcTargetAddress);
		IS_PRINT_TARGET_DVA(ixpTargetAddress);
		IS_PRINT_TARGET_DVA(ixtTargetAddress);
		IS_PRINT_TARGET_DVA(ixgTargetAddress);
		IS_PRINT_TARGET_DVA(ixvTargetAddress);
		IS_PRINT_TARGET_DVA(ixwTargetAddress);
		IS_PRINT_TARGET_DVA(mexcTargetAddress);
		IS_PRINT_TARGET_DVA(orbxcKTargetAddress);
#if IS_ENABLED(SOC_ORBMCH)
		IS_PRINT_TARGET_DVA(mchxsTargetAddress);
#endif
#if IS_ENABLED(USE_MCFP_MOTION_INTERFACE)
		IS_PRINT_TARGET_DVA(ixmTargetAddress);
#endif
#if IS_ENABLED(SOC_YPP)
		IS_PRINT_TARGET_DVA(ixdgrTargetAddress);
		IS_PRINT_TARGET_DVA(ixrrgbTargetAddress);
		IS_PRINT_TARGET_DVA(ixnoirTargetAddress);
		IS_PRINT_TARGET_DVA(ixscmapTargetAddress);
		IS_PRINT_TARGET_DVA(ixnrdsTargetAddress);
		IS_PRINT_TARGET_DVA(ixnoiTargetAddress);
		IS_PRINT_TARGET_DVA(ixdgaTargetAddress);
		IS_PRINT_TARGET_DVA(ixsvhistTargetAddress);
		IS_PRINT_TARGET_DVA(ixhfTargetAddress);
		IS_PRINT_TARGET_DVA(ixwrgbTargetAddress);
		IS_PRINT_TARGET_DVA(ixnoirwTargetAddress);
		IS_PRINT_TARGET_DVA(ixbnrTargetAddress);
		IS_PRINT_TARGET_DVA(ypnrdsTargetAddress);
		IS_PRINT_TARGET_DVA(ypnoiTargetAddress);
		IS_PRINT_TARGET_DVA(ypdgaTargetAddress);
		IS_PRINT_TARGET_DVA(ypsvhistTargetAddress);
#endif
		IS_PRINT_TARGET_DVA(sc0TargetAddress);
		IS_PRINT_TARGET_DVA(sc1TargetAddress);
		IS_PRINT_TARGET_DVA(sc2TargetAddress);
		IS_PRINT_TARGET_DVA(sc3TargetAddress);
		IS_PRINT_TARGET_DVA(sc4TargetAddress);
		IS_PRINT_TARGET_DVA(sc5TargetAddress);
		IS_PRINT_TARGET_DVA(clxsTargetAddress);
		IS_PRINT_TARGET_DVA(clxcTargetAddress);
	}
}

bool is_hw_check_crta_group(int group_id)
{
	return false;
}

int is_hw_config(struct is_hw_ip *hw_ip, struct pablo_crta_buf_info *buf_info)
{
	return 0;
}

void is_hw_update_pcfi(struct is_hardware *hardware, struct is_group *group,
			struct is_frame *frame, struct pablo_crta_buf_info *pcfi_buf)
{
}
