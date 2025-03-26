/*
 * linux/drivers/video/fbdev/exynos/panel/lx8380x/lx8380x_dimming.h
 *
 * Header file for LX8380X Dimming Driver
 *
 * Copyright (c) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LX8380X_DIMMING_H__
#define __LX8380X_DIMMING_H__
#include <linux/types.h>
#include <linux/kernel.h>
#include "../dimming.h"
#include "lx8380x.h"

#define LX8380X_NR_TP (11)

#define LX8380X_NR_LUMINANCE (256)
#define LX8380X_TARGET_LUMINANCE (600)

#define LX8380X_NR_HBM_LUMINANCE (344)
#define LX8380X_TARGET_HBM_LUMINANCE (2000)

#define LX8380X_NR_STEP (256)
#define LX8380X_HBM_STEP (344)
#define LX8380X_TOTAL_STEP (LX8380X_NR_STEP + LX8380X_HBM_STEP)

#ifdef CONFIG_USDM_PANEL_AOD_BL
#define LX8380X_AOD_NR_LUMINANCE (4)
#define LX8380X_AOD_TARGET_LUMINANCE (60)
#endif

#define LX8380X_TOTAL_NR_LUMINANCE (LX8380X_NR_LUMINANCE + LX8380X_NR_HBM_LUMINANCE)

#endif /* __LX8380X_DIMMING_H__ */
