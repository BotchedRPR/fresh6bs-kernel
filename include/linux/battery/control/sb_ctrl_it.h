/*
 * sb_ctrl_it.h
 * Samsung Mobile Battery Control Interface Header
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

#ifndef __SB_CTRL_IT_H
#define __SB_CTRL_IT_H __FILE__

#include <linux/battery/sb_def.h>

#define ctrl_log(str, ...) pr_info("[sb-ctrl-%s]:%s: "str, SB_CTRL_NAME, __func__, ##__VA_ARGS__)

struct list_head;
struct device_node;

typedef void* (*ctrl_it_create)(struct device_node *np, unsigned int event);

struct sb_ctrl_it {
	struct list_head list;

	const char *name;
	ctrl_it_create	cb_create;
	sb_event_func	cb_event;
};

int add_sb_ctrl_it(struct sb_ctrl_it *it);

#endif /* __SB_CTRL_IT_H */

