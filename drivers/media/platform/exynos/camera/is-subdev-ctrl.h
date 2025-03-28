// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Copyright (c) 2021 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef IS_SUBDEV_H
#define IS_SUBDEV_H

#include "is-video.h"
#include "pablo-work.h"

#define SUBDEV_INTERNAL_BUF_MAX		(8)

struct is_device_sensor;
struct is_device_ischain;
struct is_groupmgr;
struct is_group;

enum is_subdev_device_type {
	IS_SENSOR_SUBDEV,
	IS_ISCHAIN_SUBDEV,
};

enum is_subdev_state {
	IS_SUBDEV_OPEN,
	IS_SUBDEV_START,
	IS_SUBDEV_RUN,
	IS_SUBDEV_FORCE_SET,
	IS_SUBDEV_EXTERNAL_USE,
	IS_SUBDEV_INTERNAL_USE,
	IS_SUBDEV_INTERNAL_S_FMT,
	IS_SUBDEV_VOTF_USE,
};

enum pablo_subdev_get_type {
	PSGT_REGION_NUM,
};

struct is_subdev_path {
	u32		width;
	u32		height;
	struct is_crop	canv;
	struct is_crop	crop;
};

/* Caution: Do not exceed 64 */
enum is_subdev_id {
	ENTRY_SENSOR,
	ENTRY_SSVC0,
	ENTRY_SSVC1,
	ENTRY_SSVC2,
	ENTRY_SSVC3,
	ENTRY_3AA,
	ENTRY_3AC,
	ENTRY_3AP,
	ENTRY_3AF,
	ENTRY_3AG,
	ENTRY_3AO,
	ENTRY_3AL,
	ENTRY_ISP,
	ENTRY_IXC,
	ENTRY_IXP,
	ENTRY_IXT,
	ENTRY_IXG,
	ENTRY_IXV,
	ENTRY_IXW,
	ENTRY_MEXC,	/* MEIP or MCH*/
	ENTRY_MCS,
	ENTRY_M0P,
	ENTRY_M1P,
	ENTRY_M2P,
	ENTRY_M3P,
	ENTRY_M4P,
	ENTRY_M5P,
	ENTRY_VRA,
	ENTRY_PAF,	/* PDP(PATSTAT) Bayer RDMA */
	ENTRY_PDAF,	/* PDP(PATSTAT) AF RDMA */
	ENTRY_PDST,	/* PDP(PATSTAT) PD STAT WDMA */
	ENTRY_ORB,	/* ORBMCH */
	ENTRY_ORBXC,/* ORB */
	ENTRY_YPP,	/* YUVPP */
	ENTRY_LME,
	ENTRY_LMES,
	ENTRY_LMEC,

	ENTRY_BYRP,
	ENTRY_RGBP,
	ENTRY_RGBP_HF,
	ENTRY_MCFP,
	ENTRY_MCFP_VIDEO,
	ENTRY_MCFP_STILL,

	ENTRY_INTERNAL,

	ENTRY_END,
};

#define ENTRY_SS_VC0		ENTRY_SSVC0
#define ENTRY_SS_VC1		ENTRY_SSVC1
#define ENTRY_SS_VC2		ENTRY_SSVC2
#define ENTRY_SS_VC3		ENTRY_SSVC3
#define ENTRY_SS_VC4		ENTRY_3AA
#define ENTRY_SS_VC5		ENTRY_3AC
#define ENTRY_SS_VC6		ENTRY_3AP
#define ENTRY_SS_VC7		ENTRY_3AF
#define ENTRY_SS_VC8		ENTRY_3AG
#define ENTRY_SS_VC9		ENTRY_3AO
#define ENTRY_SS_MCB0		ENTRY_3AL
#define ENTRY_SS_MCB1		ENTRY_ISP
#define ENTRY_SS_MCB2		ENTRY_IXC
#define ENTRY_SS_MCB3		ENTRY_IXP
#define ENTRY_SS_BNS		ENTRY_IXT
#define ENTRY_CSTAT		ENTRY_IXG
#define ENTRY_CSTAT_LME_DS0	ENTRY_IXV
#define ENTRY_CSTAT_LME_DS1	ENTRY_IXW
#define ETNRY_CSTAT_FDPIG	ENTRY_MEXC
#define ENTRY_CSTAT_RGBHIST	ENTRY_CLH
#define ENTRY_CSTAT_SVHIST	ENTRY_CLHC
#define ENTRY_CSTAT_DRC		ENTRY_ORBXC
#define ENTRY_YUVP		ENTRY_YPP
#define ENTRY_MCSC		ENTRY_MCS
#define ENTRY_MCSC_P0		ENTRY_M0P
#define ENTRY_MCSC_P1		ENTRY_M1P
#define ENTRY_MCSC_P2		ENTRY_M2P
#define ENTRY_MCSC_P3		ENTRY_M3P
#define ENTRY_MCSC_P4		ENTRY_M4P
#define ENTRY_MCSC_P5		ENTRY_M5P
#define ENTRY_LME_PREV		ENTRY_LMES
#define ENTRY_LME_PURE		ENTRY_LMEC

static ulong is_subdev_wq_id[ENTRY_END] = {
	[ENTRY_SENSOR ... ENTRY_END-1] = WORK_MAX_MAP,
	[ENTRY_SENSOR] = WORK_SHOT_DONE,
	[ENTRY_SSVC0] = WORK_MAX_MAP,
	[ENTRY_SSVC1] = WORK_MAX_MAP,
	[ENTRY_SSVC2] = WORK_MAX_MAP,
	[ENTRY_SSVC3] = WORK_MAX_MAP,
	[ENTRY_3AA] = WORK_SHOT_DONE,
	[ENTRY_3AC] = WORK_30C_FDONE,
	[ENTRY_3AP] = WORK_30P_FDONE,
	[ENTRY_3AF] = WORK_30F_FDONE,
	[ENTRY_3AG] = WORK_30G_FDONE,
	[ENTRY_3AO] = WORK_30O_FDONE,
	[ENTRY_3AL] = WORK_30L_FDONE,
	[ENTRY_ISP] = WORK_SHOT_DONE,
	[ENTRY_IXC] = WORK_I0C_FDONE,
	[ENTRY_IXP] = WORK_I0P_FDONE,
	[ENTRY_IXT] = WORK_I0T_FDONE,
	[ENTRY_IXG] = WORK_I0G_FDONE,
	[ENTRY_IXV] = WORK_I0V_FDONE,
	[ENTRY_IXW] = WORK_I0W_FDONE,
	[ENTRY_MEXC] = WORK_ME0C_FDONE,
	[ENTRY_MCS] = WORK_SHOT_DONE,
	[ENTRY_M0P] = WORK_M0P_FDONE,
	[ENTRY_M1P] = WORK_M1P_FDONE,
	[ENTRY_M2P] = WORK_M2P_FDONE,
	[ENTRY_M3P] = WORK_M3P_FDONE,
	[ENTRY_M4P] = WORK_M4P_FDONE,
	[ENTRY_M5P] = WORK_M5P_FDONE,
	[ENTRY_VRA] = WORK_SHOT_DONE,
	[ENTRY_PAF] = WORK_SHOT_DONE,
	[ENTRY_PDAF] = WORK_MAX_MAP,
	[ENTRY_PDST] = WORK_MAX_MAP,
	[ENTRY_ORB] = WORK_SHOT_DONE,
	[ENTRY_ORBXC] = WORK_ORB0C_FDONE,
	[ENTRY_YPP] = WORK_SHOT_DONE,
	[ENTRY_LME] = WORK_SHOT_DONE,
	[ENTRY_LMES] = WORK_LME0S_FDONE,
	[ENTRY_LMEC] = WORK_LME0C_FDONE,
	[ENTRY_BYRP] = WORK_SHOT_DONE,
	[ENTRY_RGBP] = WORK_SHOT_DONE,
	[ENTRY_MCFP] = WORK_SHOT_DONE,
};

struct is_subdev_ops {
	int (*bypass)(struct is_subdev *subdev,
		void *device_data,
		struct is_frame *frame,
		bool bypass);
	int (*cfg)(struct is_subdev *subdev,
		void *device_data,
		struct is_frame *frame,
		struct is_crop *incrop,
		struct is_crop *otcrop,
		IS_DECLARE_PMAP(pmap));
	int (*tag)(struct is_subdev *subdev,
		void *device_data,
		struct is_frame *frame,
		struct camera2_node *node);
	int (*get)(struct is_subdev *subdev,
		   struct is_device_ischain *idi,
		   struct is_frame *frame,
		   enum pablo_subdev_get_type type,
		   void *result);
};

enum subdev_ch_mode {
	SCM_WO_PAF_HW,
	SCM_W_PAF_HW,
	SCM_MAX,
};

struct is_subdev {
	u32				id;
	u32				vid; /* video id */
	u32				cid; /* capture node id */
	enum chain_work_map		wq_id; /* workqueue id */
	char				name[4];
	u32				instance;
	unsigned long			state;

	u32				constraints_width; /* spec in width */
	u32				constraints_height; /* spec in height */

	u32				param_otf_in;
	u32				param_dma_in;
	u32				param_otf_ot;
	u32				param_dma_ot;

	struct is_subdev_path		input;
	struct is_subdev_path		output;

	struct list_head		list;

	/* for internal use */
	struct is_framemgr		internal_framemgr;
	u32				batch_num;
	u32				buffer_num;
	u32				bits_per_pixel;
	u32				memory_bitwidth;
	u32				sbwc_type;
	u32				lossy_byte32num;
	struct is_priv_buf		*pb_subdev[SUBDEV_INTERNAL_BUF_MAX];
	struct is_priv_buf		*pb_capture_subdev[SUBDEV_INTERNAL_BUF_MAX][CAPTURE_NODE_MAX];
	bool				use_shared_framemgr;
	char				data_type[15];

	struct is_video_ctx		*vctx;
	struct is_subdev		*leader;
	const struct is_subdev_ops	*ops;
};

int is_sensor_subdev_open(struct is_device_sensor *device,
	struct is_video_ctx *vctx);
int is_sensor_subdev_close(struct is_device_sensor *device,
	struct is_video_ctx *vctx);

int is_ischain_subdev_open(struct is_device_ischain *device,
	struct is_video_ctx *vctx);
int is_ischain_subdev_close(struct is_device_ischain *device,
	struct is_video_ctx *vctx);

/*common subdev*/
int is_subdev_probe(struct is_subdev *subdev,
	u32 instance,
	u32 id,
	const char *name,
	const struct is_subdev_ops *sops);
int is_subdev_open(struct is_subdev *subdev,
	struct is_video_ctx *vctx,
	void *ctl_data,
	u32 instance);
int is_subdev_close(struct is_subdev *subdev);
int pablo_subdev_buffer_init(struct is_subdev *is, struct vb2_buffer *vb);
int is_subdev_buffer_queue(struct is_subdev *subdev, struct vb2_buffer *vb);
int is_subdev_buffer_finish(struct is_subdev *subdev, struct vb2_buffer *vb);

struct is_queue_ops *is_get_sensor_subdev_qops(void);
struct is_queue_ops *is_get_ischain_subdev_qops(void);

bool is_subdev_internal_use_shared_framemgr(const struct is_subdev *subdev);
void is_subdev_internal_get_sbwc_type(const struct is_subdev *subdev,
					u32 *sbwc_type, u32 *lossy_byte32num);
int is_subdev_internal_get_buffer_size(const struct is_subdev *subdev,
					u32 *width, u32 *height,
					u32 *sbwc_block_width, u32 *sbwc_block_height);
void is_subdev_internal_lock_shared_framemgr(struct is_subdev *subdev);
void is_subdev_internal_unlock_shared_framemgr(struct is_subdev *subdev);
int is_subdev_internal_get_shared_framemgr(struct is_subdev *subdev,
	struct is_framemgr **framemgr, u32 width, u32 height);
int is_subdev_internal_put_shared_framemgr(struct is_subdev *subdev);
int is_subdev_internal_get_cap_node_num(const struct is_subdev *subdev);
int is_subdev_internal_get_out_node_info(const struct is_subdev *subdev, u32 *num_planes,
	u32 scenario, char *heapname);
int is_subdev_internal_get_cap_node_info(const struct is_subdev *subdev, u32 *vid,
	u32 *num_planes, u32 *buffer_size, u32 index, u32 scenario, char *heapname);
/* internal subdev use */
int is_subdev_internal_open(u32 instance, int vid,
				struct is_subdev *subdev);
int is_subdev_internal_close(struct is_subdev *subdev);
int is_subdev_internal_s_format(struct is_subdev *subdev,
				u32 width, u32 height, u32 bits_per_pixel,
				u32 sbwc_type, u32 lossy_byte32num,
				u32 buffer_num, const char *type_name);
struct is_sensor_cfg;
int is_subdev_internal_g_bpp(struct is_subdev *subdev,
	struct is_sensor_cfg *sensor_cfg);
int is_subdev_internal_start(struct is_subdev *subdev);
int is_subdev_internal_stop(struct is_subdev *subdev);

int __mcsc_dma_out_cfg(struct is_device_ischain *device,
	struct is_frame *ldr_frame,
	struct camera2_node *node,
	u32 pindex,
	IS_DECLARE_PMAP(pmap),
	int index);

#define GET_SUBDEV_FRAMEMGR(subdev) \
	({ struct is_framemgr *framemgr;						\
	if ((subdev) && (subdev)->vctx)							\
		framemgr = &(subdev)->vctx->queue.framemgr;				\
	else if ((subdev) && test_bit(IS_SUBDEV_INTERNAL_USE, &((subdev)->state)))	\
		framemgr = &(subdev)->internal_framemgr;				\
	else										\
		framemgr = NULL;							\
	framemgr;})

#define GET_SUBDEV_I_FRAMEMGR(subdev)				\
	({ struct is_framemgr *framemgr;			\
	if (subdev)						\
		framemgr = &(subdev)->internal_framemgr;	\
	else							\
		framemgr = NULL;				\
	framemgr; })

#define GET_SUBDEV_QUEUE(subdev) \
	(((subdev) && (subdev)->vctx) ? (&(subdev)->vctx->queue) : NULL)
#define CALL_SOPS(s, op, args...)	(((s) && (s)->ops && (s)->ops->op) ? ((s)->ops->op(s, args)) : 0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#endif
