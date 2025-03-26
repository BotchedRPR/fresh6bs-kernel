/* soc/samsung/exynos-lpd.h
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - Header file for exynos low power display support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_LPD_H
#define __EXYNOS_LPD_H

#if IS_ENABLED(CONFIG_EXYNOS_LPD)
#include <linux/notifier.h>

/*
 [LPD Notifier call chain]
 - LPD(BEGIN) -> PPMU(START) -> CHUB -> PPMU(STOP) -> LPD(END)
 - Assigned priotiry value for backward compatibility, will be removed later
 - DO NOT RETURN NOTIFY_STOP or NOTIFY_BAD in call chain fn()
*/
enum exynos_lpd_notifier_action {
	LPD_NOTIFIER_CONFIG, 	// To set panel info
	LPD_NOTIFIER_START_PRE, // For PM control
	LPD_NOTIFIER_START,
	LPD_NOTIFIER_STOP,
	LPD_NOTIFIER_STOP_POST, // For PM control
};

enum exynos_lpd_notifier_priority {
	LPD_NOTIFIER_PRIORITY_LPD_END = 0, 		// dedicated to LPD drv
	LPD_NOTIFIER_PRIORITY_PPMU_STOP = 3,	// PPMU Stop
	LPD_NOTIFIER_PRIORITY_CHUB = 5,
	LPD_NOTIFIER_PRIORITY_PPMU_START = 7,	// PPMU Start
	LPD_NOTIFIER_PRIORITY_LPD_BEGIN = 10,	// dedicated to LPD drv
	LPD_NOTIFIER_PRIORITY_MAX = LPD_NOTIFIER_PRIORITY_LPD_BEGIN,
};

#define DPU_STATE_OFF		0
#define	DPU_STATE_ON		1
#define DPU_STATE_DOZE		2
#define DPU_STATE_DOZE_SUSPEND	3

extern int lpd_notifier_register(struct notifier_block *nb);
extern int lpd_notifier_unregister(struct notifier_block *nb);
extern int lpd_notifier_call(u32 action, void *data);


/* reserved 4Mbytes for lpd oled compensation */
#define LPD_MAX_COMP_MEMORY		(4 * 1024 * 1024)

#define LPD_COMP_MAX_X_RES		500
#define LPD_COMP_MAX_Y_RES		500

#define LPD_COMP_MAX_BPP		4

/*max lut count: gray scale 0 ~ 255*/
#define LPD_COMP_MAX_LUT_COUNT		256


#define MINUTE_PER_HOUR			60
#define HOUR_PER_DAY			24

/*r, g, b*/
#define COLOR_PER_PIXEL			3

#define LPD_COMP_IMAGE_DATA_SZ		(sizeof(float))
#define LPD_COMP_LUT_DATA_SZ		(sizeof(float))

#define LPD_COMP_LUT_HEADER_SZ		(sizeof(unsigned int))
#define LPD_COMP_BR_HEADER_SZ		(sizeof(unsigned int))

#define LPD_COMP_BR_DATA_SZ		(sizeof(unsigned int))


#define LPD_COMP_BR_SIZE		(LPD_COMP_BR_HEADER_SZ + (MINUTE_PER_HOUR * HOUR_PER_DAY * LPD_COMP_BR_DATA_SZ))

#define LPD_COMP_IMAGE_SIZE		((LPD_COMP_MAX_X_RES * LPD_COMP_MAX_Y_RES * COLOR_PER_PIXEL * LPD_COMP_IMAGE_DATA_SZ) + LPD_COMP_BR_SIZE)
#define LPD_COMP_CANVAS_SIZE		(LPD_COMP_MAX_X_RES * LPD_COMP_MAX_Y_RES * LPD_COMP_MAX_BPP)
#define LPD_COMP_LUT_SIZE		(LPD_COMP_LUT_HEADER_SZ + (LPD_COMP_MAX_LUT_COUNT * LPD_COMP_LUT_DATA_SZ * COLOR_PER_PIXEL))

#define LPD_COMP_TOTAL_SIZE		(LPD_COMP_IMAGE_SIZE + LPD_COMP_CANVAS_SIZE + LPD_COMP_LUT_SIZE)


#if IS_ENABLED(CONFIG_LPD_AUTO_BR)

#define CMD_SEQ_MAGIC_CODE1		0xacc12345
#define CMD_SEQ_MAGIC_CODE2		0xacc23456

#define CMD_SEQ_MAGIC_OFFSET1		0
#define CMD_SEQ_MAGIC_OFFSET2		1


#define MAX_AUTO_BR_CNT			20

struct lpd_br_info {
	unsigned int br_cnt;
	unsigned int br_list[MAX_AUTO_BR_CNT];
	unsigned int nit_list[MAX_AUTO_BR_CNT];
};

struct lpd_cmd_buf {
	void *buf;
	unsigned int buf_size;
};

struct lpd_panel_cmd {
	struct lpd_br_info br_info;
	struct lpd_cmd_buf cmd_buf;
};

enum lpd_config_action {
	LPD_CONFIG_BR_CMD = 0,
	LPD_CONFIG_HBM_BR_CMD,
	LPD_CONFIG_INIT_CMD
};

extern int lpd_config_notifier_register(struct notifier_block *nb);
extern int lpd_config_notifier_unregister(struct notifier_block *nb);
extern int lpd_config_notifier_call(u32 action, void *data);
#endif

extern bool disp_is_on(void);
#else /* !CONFIG_EXYNOS_LPD */
static inline bool disp_is_on(void)
{
	return false;
}
#endif

#endif /* __EXYNOS_LPD_H */
