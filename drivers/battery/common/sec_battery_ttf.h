/*
 * sec_battery.h
 * Samsung Mobile Battery Header
 *
 *
 * Copyright (C) 2012 Samsung Electronics, Inc.
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

#ifndef __SEC_BATTERY_TTF_H
#define __SEC_BATTERY_TTF_H __FILE__

#include <linux/errno.h>

struct device_node;
struct sec_battery_info;
struct sec_ttf_data;

void sec_bat_calc_time_to_full(struct sec_battery_info *battery);
int ttf_parse_fcc(struct device_node *np, unsigned int *fcc);
int ttf_init(struct sec_battery_info *battery);
void ttf_work_start(struct sec_battery_info *battery);
int ttf_display(struct sec_battery_info *battery);

#endif /* __SEC_BATTERY_H */
