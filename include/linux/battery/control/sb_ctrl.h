/*
 * sb_ctrl.h
 * Samsung Mobile Battery Control Header
 *
 * Copyright (C) 2022 Samsung Electronics, Inc.
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

#ifndef __SB_CTRL_H
#define __SB_CTRL_H __FILE__

#include <linux/err.h>

#include <linux/battery/sb_def.h>

struct sb_ctrl;
struct device_node;

enum sb_ctrl_event {
	SB_CTRL_EVENT_NONE	= 0,

	SB_CTRL_EVENT_CABLE_ATTACH,
	SB_CTRL_EVENT_CABLE_DETACH,

	SB_CTRL_EVENT_MONITOR
};

#define SB_CONTROL_DISABLE		(-3700)
#if IS_ENABLED(CONFIG_SB_CONTROL)
struct sb_ctrl *sb_ctrl_create(struct device_node *np);
int sb_ctrl_set(struct sb_ctrl *ctrl, int event, sb_data data);
#else
static inline struct sb_ctrl *sb_ctrl_create(struct device_node *np)
{ return ERR_PTR(SB_CONTROL_DISABLE); }
static inline int sb_ctrl_set(struct sb_ctrl *ctrl, int event, sb_data data)
{ return SB_CONTROL_DISABLE; }
#endif

#endif /* __SB_CTRL_H */

