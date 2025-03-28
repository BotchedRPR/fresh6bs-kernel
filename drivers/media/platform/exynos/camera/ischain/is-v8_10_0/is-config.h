/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IS_CONFIG_H
#define IS_CONFIG_H

#include "is-common-config.h"
#include "is-vendor-config.h"
#include <linux/ion_exynos.h>

/*
 * =================================================================================================
 * CONFIG - GLOBAL OPTIONS
 * =================================================================================================
 */
#define IS_SENSOR_COUNT		6
#define IS_STREAM_COUNT		9

/*
 * =================================================================================================
 * CONFIG - Chain IP configurations
 * =================================================================================================
 */

/* 3AA0 */
#define SOC_30S		1
#define SOC_30C		1
#define SOC_30P		1
#define SOC_30F		1
#define SOC_30G		1
#define SOC_ZSL_STRIP_DMA0

/* 3AA1 */
#define SOC_31S		1
#define SOC_31C		1
#define SOC_31P		1
#define SOC_31F		1
#define SOC_31G		1
#define SOC_ZSL_STRIP_DMA1

/* 3AA2 */
#define SOC_32S		1
#define SOC_32C		1
#define SOC_32P		1
#define SOC_32F		1
#define SOC_32G		1
#define SOC_ZSL_STRIP_DMA2

/* ITP0 */
#define SOC_I0S		1
#define SOC_DNS0S	1

/* MEIP/ORB-MCH */
#define SOC_ME0S	1
#define SOC_ME0C	1
#define SOC_ME1S	1
#define SOC_ME1C	1
#define SOC_ORB0C	1

#define SOC_MCS		1
#define SOC_SSVC0	1
#define SOC_SSVC1	1
#define SOC_SSVC2	1
#define SOC_SSVC3	1

#define SOC_TNR		1
#define SOC_TNR_MERGER	1

#define SOC_MCS0	1

#define SOC_CLH		1

/* #define SOC_ITSC */

/* #define SOC_MCS1 */
/* #define SOC_VRA */

/* Frame id decoder */
#define SYSREG_FRAME_ID_DEC		(0x0)

/*
 * =================================================================================================
 * CONFIG - SW configurations
 * =================================================================================================
 */
#define CHAIN_STRIPE_PROCESSING		1
#define STRIPE_MARGIN_WIDTH		512
#define STRIPE_MCSC_MARGIN_WIDTH	68
#define STRIPE_WIDTH_ALIGN		512
#define STRIPE_RATIO_PRECISION		1000

#define ENABLE_TNR
#define ENABLE_ORBMCH	1
#define ENABLE_10BIT_MCSC
#define ENABLE_DJAG_IN_MCSC
/* #define USE_MCSC_STRIP_OUT_CROP */	/* use for MCSC stripe */
/* #define ENABLE_VRA */
/* #define ENABLE_REPROCESSING_FD */
/* #define ENABLE_VRA_CHANGE_SETFILE_PARSING */
/* #define ENABLE_HYBRID_FD */

#define USE_ONE_BINARY
#define USE_RTA_BINARY
#define USE_BINARY_PADDING_DATA_ADDED	/* for DDK signature */
#define USE_DDK_SHUT_DOWN_FUNC
#define ENABLE_IRQ_MULTI_TARGET
#define ENABLE_3AA_LIC_OFFSET	1
#define IS_MAX_HW_3AA_SRAM		(13696)
#define PDP_RDMA_MO_LIMIT	1
#define PDP_RDMA_LINE_GAP	(0x1388)
#define USE_RECOVER_I2C_TRANS	2
#define ENABLE_MODECHANGE_CAPTURE
/*
 * PDP0: RDMA
 * PDP1: RDMA
 * PDP2: RDMA
 * 3AA0: 3AA0, ZSL/STRIP DMA0, MEIP0
 * 3AA1: 3AA1, ZSL/STRIP DMA1, MEIP1
 * 3AA2: 3AA2, ZSL/STRIP DMA2, MEIP2
 * ITP0: TNR, ITP0, DNS0
 * MCSC0:
 *
 */
#define HW_SLOT_MAX            (10)
#define valid_hw_slot_id(slot_id) \
	(slot_id >= 0 && slot_id < HW_SLOT_MAX)
#define HW_NUM_LIB_OFFSET	(2)
/* #define SKIP_SETFILE_LOAD */
/* #define SKIP_LIB_LOAD */

#define USE_SENSOR_IF_DPHY

/* FIMC-IS task priority setting */
#define TASK_SENSOR_WORK_PRIO		(IS_MAX_PRIO - 48) /* 52 */
#define TASK_GRP_OTF_INPUT_PRIO		(IS_MAX_PRIO - 49) /* 51 */
#define TASK_GRP_DMA_INPUT_PRIO		(IS_MAX_PRIO - 50) /* 50 */
#define TASK_MSHOT_WORK_PRIO		(IS_MAX_PRIO - 43) /* 57 */
#define TASK_LIB_OTF_PRIO		(IS_MAX_PRIO - 44) /* 56 */
#define TASK_LIB_AF_PRIO		(IS_MAX_PRIO - 45) /* 55 */
#define TASK_LIB_ISP_DMA_PRIO		(IS_MAX_PRIO - 46) /* 54 */
#define TASK_LIB_3AA_DMA_PRIO		(IS_MAX_PRIO - 47) /* 53 */
#define TASK_LIB_AA_PRIO		(IS_MAX_PRIO - 48) /* 52 */
#define TASK_LIB_RTA_PRIO		(IS_MAX_PRIO - 49) /* 51 */
#define TASK_LIB_VRA_PRIO		(IS_MAX_PRIO - 45) /* 55 */

/* EXTRA chain for 3AA and ITP
 * Each IP has 4 slot
 * 3AA: 0~3	ITP: 0~3
 * MEIP: 4~7	TNR: 4~7
 * DMA: 8~11	DNS: 8~11
 */
#define LIC_CHAIN_OFFSET	(0x10000)
#define LIC_CHAIN_NUM		(2)
#define LIC_CHAIN_OFFSET_NUM	(6)
/*
 * =================================================================================================
 * CONFIG - FEATURE ENABLE
 * =================================================================================================
 */

#define IS_MAX_TASK		(40)

#undef OVERFLOW_PANIC_ENABLE_ISCHAIN
#undef OVERFLOW_PANIC_ENABLE_CSIS
#if defined(CONFIG_ARM_EXYNOS_DEVFREQ)
#define CONFIG_IS_BUS_DEVFREQ
#endif
#if defined(OVERFLOW_PANIC_ENABLE_ISCHAIN)
#define DDK_OVERFLOW_RECOVERY		(0)	/* 0: do not execute recovery, 1: execute recovery */
#else
#define DDK_OVERFLOW_RECOVERY		(1)	/* 0: do not execute recovery, 1: execute recovery */
#endif
#define CAPTURE_NODE_MAX		12
#define OTF_YUV_FORMAT			(OTF_INPUT_FORMAT_YUV422)
#define MSB_OF_3AA_DMA_OUT		(11)
#define MSB_OF_DNG_DMA_OUT		(9)
/* #define USE_YUV_RANGE_BY_ISP */
/* #define ENABLE_3AA_DMA_CROP */

/* use bcrop1 to support DNG capture for pure bayer reporcessing */
#define USE_3AA_CROP_AFTER_BDS

/* #define ENABLE_ULTRA_FAST_SHOT */
/* #define ENABLE_HWFC */
#define TPU_COMPRESSOR
#define USE_I2C_LOCK
#define SENSOR_REQUEST_DELAY		2

#ifdef ENABLE_IRQ_MULTI_TARGET
#define IS_HW_IRQ_FLAG     IRQF_GIC_MULTI_TARGET
#else
#define IS_HW_IRQ_FLAG     0
#endif

/* #define MULTI_SHOT_KTHREAD */
/* #define MULTI_SHOT_TASKLET */
/* #define ENABLE_EARLY_SHOT */
#define ENABLE_HW_FAST_READ_OUT
#define FULL_OTF_TAIL_GROUP_ID		GROUP_ID_MCS0

#if defined(USE_I2C_LOCK)
#define I2C_MUTEX_LOCK(lock)	mutex_lock(lock)
#define I2C_MUTEX_UNLOCK(lock)	mutex_unlock(lock)
#else
#define I2C_MUTEX_LOCK(lock)
#define I2C_MUTEX_UNLOCK(lock)
#endif

#define ENABLE_DBG_EVENT_PRINT

#ifdef SECURE_CAMERA_IRIS
#undef SECURE_CAMERA_IRIS
#endif
/* #define SECURE_CAMERA_FACE */	/* For face Detection and face authentication */
#define SECURE_CAMERA_CH		((1 << CSI_ID_C) | (1 << CSI_ID_E))
#define SECURE_CAMERA_HEAP_ID		(11)
#define SECURE_CAMERA_MEM_ADDR		(0xE1900000)	/* secure_camera_heap */
#define SECURE_CAMERA_MEM_SIZE		(0x01EE0000)
#define NON_SECURE_CAMERA_MEM_ADDR	(0x0)	/* camera_heap */
#define NON_SECURE_CAMERA_MEM_SIZE	(0x0)
#define ION_EXYNOS_FLAG_PROTECTED	ION_FLAG_PROTECTED
#define ION_EXYNOS_FLAG_IOVA_EXTENSION	(0)

#define SECURE_CAMERA_FACE_SEQ_CHK	/* To check sequence before applying secure protection */

#define USE_NEW_PER_FRAME_CONTROL

#define ENABLE_HWACG_CONTROL

/* load calibation data */
#define ENABLE_CAL_LOAD

#define BDS_DVFS
/* #define ENABLE_VIRTUAL_OTF */
/* #define ENABLE_DCF */

/* #define ENABLE_PDP_STAT_DMA */

#define USE_PDP_LINE_INTR_FOR_PDAF	1
#define USE_DEBUG_PD_DUMP	0
#define DEBUG_PD_DUMP_CH	0  /* set a bit corresponding to each channel */

#define FAST_FDAE

/* init AWB */
#define INIT_AWB		1
#define WB_GAIN_COUNT		(4)
#define INIT_AWB_COUNT_REAR	(3)
#define INIT_AWB_COUNT_FRONT	(8)

/* use CIS global work for enhance launching time */
#define USE_CIS_GLOBAL_WORK	1

/* use OIS init thread option */
#define USE_OIS_INIT_WORK

/* #define USE_CAMIF_FIX_UP	1 */
#define CHAIN_TAG_SENSOR_IN_SOFTIRQ_CONTEXT	0
#define CHAIN_TAG_VC0_DMA_IN_HARDIRQ_CONTEXT	1

#define USE_SKIP_DUMP_LIC_OVERFLOW     1

/* default LIC value for 9630 */
/*
 * MAX: 16383
 * CTX0: 25M(5804), 25M(5804), 8M (3328)
 * CTX1: 25M(5804), 25M(5804), 8M (3328)
 */
#define DEFAULT_LIC_CTX0_OFFSET0	(5824)
#define DEFAULT_LIC_CTX0_OFFSET1	(5824)
#define DEFAULT_LIC_CTX1_OFFSET0	(5824)
#define DEFAULT_LIC_CTX1_OFFSET1	(5824)

#define DISABLE_CORE_IDLE_STATE

#define AP_PHY_LDO_ALL_SAME  	1

#define CLOGGING		1
#define USE_FASTEN_AE		0	/* This is a feature for fast launch to enter scenario */
/*
 * ======================================================
 * CONFIG - Interface version configs
 * ======================================================
 */

#define SETFILE_VERSION_INFO_HEADER1	(0xF85A20B4)
#define SETFILE_VERSION_INFO_HEADER2	(0xCA539ADF)

#define META_ITF_VER_20192003

#define ENABLE_FPSIMD_FOR_USER

#endif /* #ifndef IS_CONFIG_H */
