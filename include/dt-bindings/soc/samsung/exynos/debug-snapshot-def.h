/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#ifndef DEBUG_SNAPSHOT_TABLE_H
#define DEBUG_SNAPSHOT_TABLE_H

#define SZ_64K				0x00010000
#define SZ_256K				0x00040000
#define SZ_512K				0x00080000
#define SZ_1M				0x00100000

#define DSS_START_ADDR			0xA0000000
#define DSS_HEADER_SIZE		SZ_64K
#define DSS_KERNEL_SIZE         (2 * SZ_1M)
#define DSS_PLATFORM_SIZE	(2 * SZ_1M)
#define DSS_S2D_SIZE		(1 * SZ_1M)
#define DSS_FIRST_SIZE		(2 * SZ_1M)
#define DSS_INIT_TASK_SIZE	(0)
#define DSS_ARRAYRESET_SIZE	(SZ_256K)
#define DSS_ARRAYPANIC_SIZE	(SZ_256K)
#define DSS_KEVENTS_SIZE	(1 * SZ_1M)
#define DSS_FATAL_SIZE		(0 * SZ_1M)

#define DSS_HEADER_OFFSET	0

#define DSS_HEADER_ADDR		(DSS_START_ADDR + DSS_HEADER_OFFSET)
#define DSS_KERNEL_ADDR		(DSS_HEADER_ADDR + DSS_HEADER_SIZE)
#define DSS_PLATFORM_ADDR	(DSS_KERNEL_ADDR + DSS_KERNEL_SIZE)
#define DSS_KEVENTS_ADDR	(DSS_PLATFORM_ADDR + DSS_PLATFORM_SIZE)
#define DSS_S2D_ADDR		(DSS_KEVENTS_ADDR + DSS_KEVENTS_SIZE)
#define DSS_FIRST_ADDR		(DSS_S2D_ADDR + DSS_S2D_SIZE)
#define DSS_INIT_TASK_ADDR	(DSS_FIRST_ADDR + DSS_FIRST_SIZE)
#define DSS_ARRAYRESET_ADDR	(DSS_INIT_TASK_ADDR + DSS_INIT_TASK_SIZE)
#define DSS_ARRAYPANIC_ADDR	(DSS_ARRAYRESET_ADDR + DSS_ARRAYRESET_SIZE)
#define DSS_FATAL_ADDR		(DSS_ARRAYPANIC_ADDR + DSS_ARRAYPANIC_SIZE)

/* MEMLOGGER */
#define DSS_MEMLOG_BL_BASE		(0x290)

/* KEVENT ID */
#define DSS_ITEM_HEADER                 "header"
#define DSS_ITEM_KERNEL                 "log_kernel"
#define DSS_ITEM_PLATFORM               "log_platform"
#define DSS_ITEM_KEVENTS                "log_kevents"
#define DSS_ITEM_S2D                    "log_s2d"
#define DSS_ITEM_ARRDUMP_RESET          "log_arrdumpreset"
#define DSS_ITEM_ARRDUMP_PANIC          "log_arrdumppanic"
#define DSS_ITEM_FIRST		        "log_first"
#define DSS_ITEM_BACKTRACE	        "log_backtrace"

#define DSS_LOG_TASK                    "task_log"
#define DSS_LOG_WORK                    "work_log"
#define DSS_LOG_CPUIDLE                 "cpuidle_log"
#define DSS_LOG_SUSPEND                 "suspend_log"
#define DSS_LOG_IRQ                     "irq_log"
#define DSS_LOG_REG			"reg_log"
#define DSS_LOG_HRTIMER                 "hrtimer_log"
#define DSS_LOG_CLK                     "clk_log"
#define DSS_LOG_PMU                     "pmu_log"
#define DSS_LOG_FREQ                    "freq_log"
#define DSS_LOG_DM                      "dm_log"
#define DSS_LOG_REGULATOR               "regulator_log"
#define DSS_LOG_THERMAL                 "thermal_log"
#define DSS_LOG_ACPM                    "acpm_log"
#define DSS_LOG_PRINTK                  "printk_log"

/* MODE */
#define NONE_DUMP                       0
#define FULL_DUMP                       1
#define QUICK_DUMP                      2

/* ACTION */
#define GO_DEFAULT                      "default"
#define GO_DEFAULT_ID                   0
#define GO_PANIC                        "panic"
#define GO_PANIC_ID                     1
#define GO_WATCHDOG                     "watchdog"
#define GO_WATCHDOG_ID                  2
#define GO_S2D                          "s2d"
#define GO_S2D_ID                       3
#define GO_ARRAYDUMP                    "arraydump"
#define GO_ARRAYDUMP_ID                 4
#define GO_SCANDUMP                     "scandump"
#define GO_SCANDUMP_ID                  5
#define GO_HALT				"halt"
#define GO_HALT_ID			6

/* EXCEPTION POLICY */
#define DPM_F                           "feature"
#define DPM_P                           "policy"

#define DPM_P_EL1_DA                    "el1_da"
#define DPM_P_EL1_IA                    "el1_ia"
#define DPM_P_EL1_UNDEF                 "el1_undef"
#define DPM_P_EL1_SP_PC                 "el1_sp_pc"
#define DPM_P_EL1_INV                   "el1_inv"
#define DPM_P_EL1_SERROR                "el1_serror"
#endif
