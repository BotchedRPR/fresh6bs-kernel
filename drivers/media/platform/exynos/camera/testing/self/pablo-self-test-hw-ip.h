// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PABLO_SELF_TEST_HW_IP_H
#define PABLO_SELF_TEST_HW_IP_H

#include "pablo-framemgr.h"
#include "is-hw.h"

#define PST_WAIT_TIME_START_ISR 1000

enum pst_hw_ip_state_act {
	PST_HW_IP_STOP,
	PST_HW_IP_START,
	PST_HW_IP_PRESET,
	PST_HW_IP_UPDATE_PARAM,
	PST_HW_IP_UPDATE_VECTOR,
};

#define CALL_PST_CB(cb, op, args...)	\
	(((cb) && (cb)->op) ? ((cb)->op(args)) : 0)

struct pst_callback_ops {
	void (*init_param)(unsigned int index);
	void (*set_param)(struct is_frame *frame);
	void (*clr_param)(struct is_frame *frame);
	void (*set_rta_info)(struct is_frame *frame, struct size_cr_set *cr_set);
};

int pst_set_hw_ip(const char *val,
		enum is_hardware_id hw_id,
		struct is_hw_ip *hw_ip,
		struct is_frame *frame,
		u32 **param,
		struct size_cr_set *cr_set,
		size_t preset_size,
		unsigned long *result,
		const struct pst_callback_ops *pst_cb);
int pst_get_hw_ip(char *buffer,
		const struct is_hw_ip *hw_ip,
		const size_t preset_size,
		const unsigned long *result);

void *pst_get_param(struct is_frame *frame, u32 param_idx);
void pst_set_param(struct is_frame *frame, u32 *src_params, u32 param_idx);
int pst_cmp_param(struct is_frame *frame, void *param_dst, u32 param_idx);
struct is_priv_buf *pst_set_dva(struct is_frame *frame, dma_addr_t *dva, size_t *size, u32 vid);
void pst_clr_dva(struct is_priv_buf *pb);
void pst_get_size_of_dma_input(void *param, u32 align, size_t *size);
void pst_get_size_of_dma_output(void *param, u32 align, size_t *size);
void pst_get_size_of_mcs_output(void *param, u32 align, size_t *size);
#endif
