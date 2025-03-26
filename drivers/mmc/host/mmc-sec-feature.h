// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Specific feature
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Storage Driver <storage.sec@samsung.com>
 */
#ifndef __MMC_SEC_FEATURE_H__
#define __MMC_SEC_FEATURE_H__

#include "dw_mmc.h"
#include <linux/platform_device.h>

#define HALT_UNHALT_ERR		0x00000001
#define CQ_EN_DIS_ERR		0x00000002
#define RPMB_SWITCH_ERR		0x00000004
#define HW_RST			0x00000008
#define MMC_ERR_MASK		(HALT_UNHALT_ERR | CQ_EN_DIS_ERR |\
				RPMB_SWITCH_ERR | HW_RST)
void mmc_sec_log_cq_err(u32 cmd_error, u32 data_error, u32 status);
void mmc_sec_set_features(struct mmc_host *host);
void mmc_sd_sec_log_req_err(struct mmc_host *host, struct mmc_request *mrq);
#endif

