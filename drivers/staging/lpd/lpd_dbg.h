/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Debug interface between LPD fw and LPD driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ___SAMSUNG_LPD_DEBUG_H__
#define ___SAMSUNG_LPD_DEBUG_H__
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/dcache.h>


#define lpd_fmt(fmt) "LPD: "fmt

#define LPD_PR_PREFIX		"LPD: "
#define LPD_FW_PR_PREFIX	"LPD-FW: "


#define lpd_err(fmt, ...)									\
	do {											\
		if (lpd_log_level >= 3) {							\
			pr_err(pr_fmt(LPD_PR_PREFIX ":E:%s: " fmt), __func__, ##__VA_ARGS__);	\
		}										\
	} while (0)

#define lpd_warn(fmt, ...)									\
	do {											\
		if (lpd_log_level >= 4) {							\
			pr_warn(pr_fmt(LPD_PR_PREFIX ":W:%s: " fmt), __func__, ##__VA_ARGS__);	\
		}										\
	} while (0)

#define lpd_info(fmt, ...)									\
	do {											\
		if (lpd_log_level >= 6)								\
			pr_info(pr_fmt(LPD_PR_PREFIX ":I:%s: " fmt), __func__, ##__VA_ARGS__);	\
	} while (0)

#define lpd_dbg(fmt, ...)									\
	do {											\
		if (lpd_log_level >= 7)								\
			pr_info(pr_fmt(LPD_PR_PREFIX ":D:%s: " fmt), __func__, ##__VA_ARGS__);	\
	} while (0)

#define lpd_fw_log(fmt, ...)									\
	do {											\
		if (lpd_log_level >= 4)								\
			pr_info(pr_fmt(LPD_FW_PR_PREFIX ": " fmt), ##__VA_ARGS__);		\
	} while (0)



#if IS_ENABLED(CONFIG_EXYNOS_MEMORY_LOGGER)
#include <soc/samsung/exynos/memlogger.h>
struct memlogs {
	struct memlog *memlog_lpd;
	struct memlog_obj *memlog_sram_file;
	struct memlog_obj *memlog_sram;
	struct memlog_obj *memlog_printf_file;
	struct memlog_obj *memlog_printf;
};
#endif

struct lpd_debug_file {
	int id;
	struct dentry *file;
	void *private;
};

enum {
	LPD_DEBUGFILE_LEVEL,
	LPD_DEBUGFILE_ENABLE,
	MAX_LPD_DEBUFS,
};

struct lpd_debug_fs {
	struct dentry *root_dir;

	struct dentry *dbg_level;
	struct dentry *logbuf_enable;

	struct lpd_debug_file *debug_file[MAX_LPD_DEBUFS];
};

struct lpd_debug_info {
	struct task_struct *logbuf_thread;
	struct mutex logbuf_lock;		/* logbuf access lock */

	struct lpd_debug_fs debug_fs;
#if IS_ENABLED(CONFIG_EXYNOS_MEMORY_LOGGER)
	/* memlogger */
	struct memlogs mlog;
#endif
	int pm_refcnt;
};


extern int lpd_log_level;

#endif
