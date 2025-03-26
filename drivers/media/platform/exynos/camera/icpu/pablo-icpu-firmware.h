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

#ifndef PABLO_ICPU_FIRMWARE_H
#define PABLO_ICPU_FIRMWARE_H

int init_firmware_mem(struct icpu_core *core, size_t size);
void deinit_firmware_mem(struct icpu_core *core);
void config_firmware(bool fw_mem_alloc_boot);
int load_firmware(struct icpu_core *core);
void teardown_firmware(struct icpu_core *core);

#endif
