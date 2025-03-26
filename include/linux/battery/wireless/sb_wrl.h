/*
 * sb_wrl.h
 * Samsung Mobile Battery Wrl Header
 *
 * Copyright (C) 2023 Samsung Electronics, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SB_WRL_H
#define __SB_WRL_H __FILE__

#include <linux/battery/sb_def.h>

struct sb_wrl_op {
	int (*set_vm_phm)(void*, int, int, sb_data);
	int (*get_vm_phm)(void*, sb_data*);
};

enum sb_wrl_error {
	SB_WRL_ERROR_TYPE_SSP_MISSING = 0,
	SB_WRL_ERROR_TYPE_PHM_FAIL,
	SB_WRL_ERROR_TYPE_MISALIGN,
	SB_WRL_ERROR_TYPE_AUTH,
	SB_WRL_ERROR_TYPE_INCOMPATIBLE,

	SB_WRL_ERROR_TYPE_MAX = 8
};

#define SB_WRL_DISABLE	(-3900)
#if IS_ENABLED(CONFIG_SB_WRL)
int sb_wrl_set_vm_phm(int event, int en, sb_data data);
int sb_wrl_get_vm_phm(sb_data *data);

int sb_wrl_register(const struct sb_wrl_op *op, void *pdata);

int sb_wrl_inc_error(enum sb_wrl_error err);
int sb_wrl_clr_error(enum sb_wrl_error err);
sb_data sb_wrl_error_raw_data(void);
#else
static inline int sb_wrl_set_vm_phm(int event, int en, sb_data data)
{ return SB_WRL_DISABLE; }
static inline int sb_wrl_get_vm_phm(sb_data *data)
{ return SB_WRL_DISABLE; }

static inline int sb_wrl_register(const struct sb_wrl_op *op, void *pdata)
{ return SB_WRL_DISABLE; }

static inline int sb_wrl_inc_error(enum sb_wrl_error err)
{ return SB_WRL_DISABLE; }
static inline int sb_wrl_clr_error(enum sb_wrl_error err)
{ return SB_WRL_DISABLE; }
static inline sb_data sb_wrl_error_raw_data(void)
{ return 0; }
#endif

#endif /* __SB_WRL_H */
