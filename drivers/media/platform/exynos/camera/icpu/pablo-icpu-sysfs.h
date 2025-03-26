/* SPDX-License-Identifier: GPL */
/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Exynos Pablo image subsystem functions
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PABLO_ICPU_SYSFS_H
#define PABLO_ICPU_SYSFS_H

struct icpu_logger *get_icpu_core_log(void);
struct icpu_logger *get_icpu_debug_log(void);
struct icpu_logger *get_icpu_firmware_log(void);
struct icpu_logger *get_icpu_itf_log(void);
struct icpu_logger *get_icpu_msgqueue_log(void);
struct icpu_logger *get_icpu_selftest_log(void);
struct icpu_logger *get_icpu_hw_itf_log(void);
struct icpu_logger *get_icpu_hw_log(void);
struct icpu_logger *get_icpu_mbox_log(void);

int pablo_icpu_sysfs_probe(struct device *dev);
void pablo_icpu_sysfs_remove(struct device *dev);

#endif
