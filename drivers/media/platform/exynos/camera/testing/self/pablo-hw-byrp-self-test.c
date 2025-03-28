// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Exynos Pablo image subsystem functions
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include "pablo-self-test-hw-ip.h"
#include "pablo-framemgr.h"
#include "is-hw.h"
#include "is-core.h"
#include "is-device-ischain.h"

static int pst_set_hw_byrp(const char *val, const struct kernel_param *kp);
static int pst_get_hw_byrp(char *buffer, const struct kernel_param *kp);
static const struct kernel_param_ops pablo_param_ops_hw_byrp = {
	.set = pst_set_hw_byrp,
	.get = pst_get_hw_byrp,
};
module_param_cb(test_hw_byrp, &pablo_param_ops_hw_byrp, NULL, 0644);

#define NUM_OF_BYRP_PARAM (PARAM_BYRP_BYR_PROCESSED - PARAM_BYRP_CONTROL + 1)

static struct is_hw_ip *hw_ip_byrp;
static struct is_frame *frame_byrp;
static u32 byrp_param[NUM_OF_BYRP_PARAM][PARAMETER_MAX_MEMBER];
static struct is_priv_buf *pb[NUM_OF_BYRP_PARAM];
static struct size_cr_set byrp_cr_set;

static const struct byrp_param byrp_param_preset[] = {
	/* Param set[0]: STRGEN + BYR WDMA(SBWC OFF) */
	[0].control.cmd = CONTROL_COMMAND_START,
	[0].control.bypass = 0,
	[0].control.strgen = CONTROL_COMMAND_START,

	[0].stripe_input.total_count = 0,

	[0].dma_input.cmd = DMA_INPUT_COMMAND_DISABLE,
	[0].dma_input.format = DMA_INOUT_FORMAT_BAYER_PACKED,
	[0].dma_input.bitwidth = DMA_INOUT_BIT_WIDTH_10BIT,
	[0].dma_input.order = OTF_INPUT_ORDER_BAYER_GB_RG,
	[0].dma_input.plane = DMA_INOUT_PLANE_1,
	[0].dma_input.width = 4032,
	[0].dma_input.height = 3024,
	[0].dma_input.dma_crop_offset = 0,
	[0].dma_input.dma_crop_width = 4032,
	[0].dma_input.dma_crop_height = 3024,
	[0].dma_input.bayer_crop_offset_x = 0,
	[0].dma_input.bayer_crop_offset_y = 0,
	[0].dma_input.bayer_crop_width = 4032,
	[0].dma_input.bayer_crop_height = 3024,
	[0].dma_input.scene_mode = 0,
	[0].dma_input.msb = DMA_INOUT_BIT_WIDTH_10BIT - 1,
	[0].dma_input.stride_plane0 = 4032,
	[0].dma_input.stride_plane1 = 4032,
	[0].dma_input.stride_plane2 = 4032,
	[0].dma_input.v_otf_enable = OTF_INPUT_COMMAND_DISABLE,
	[0].dma_input.orientation = 0,
	[0].dma_input.strip_mode = 0,
	[0].dma_input.overlab_width = 0,
	[0].dma_input.strip_count = 0,
	[0].dma_input.strip_max_count = 0,
	[0].dma_input.sequence_id = 0,
	[0].dma_input.sbwc_type = NONE,

	[0].hdr.cmd = DMA_INPUT_COMMAND_DISABLE,

	[0].byr.cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[0].byr.format = DMA_INOUT_FORMAT_BAYER_PACKED,
	[0].byr.bitwidth = DMA_INOUT_BIT_WIDTH_10BIT,
	[0].byr.order = OTF_INPUT_ORDER_BAYER_GB_RG,
	[0].byr.plane = DMA_INOUT_PLANE_1,
	[0].byr.width = 4032,
	[0].byr.height = 3024,
	[0].byr.dma_crop_offset_x = 0,
	[0].byr.dma_crop_offset_y = 0,
	[0].byr.dma_crop_width = 4032,
	[0].byr.dma_crop_height = 3024,
	[0].byr.crop_enable = 0,
	[0].byr.msb = DMA_INOUT_BIT_WIDTH_10BIT - 1,
	[0].byr.stride_plane0 = 4032,
	[0].byr.stride_plane1 = 4032,
	[0].byr.stride_plane2 = 4032,
	[0].byr.v_otf_enable = OTF_OUTPUT_COMMAND_DISABLE,
	[0].byr.sbwc_type = NONE,

	[0].byr_processed.cmd = DMA_INPUT_COMMAND_DISABLE,

	/* Param set[1]: RDMA(SBWC OFF) + BYR WDMA(SBWC OFF) - DNG */
	[1].control.cmd = CONTROL_COMMAND_START,
	[1].control.bypass = 0,
	[1].control.strgen = CONTROL_COMMAND_STOP,

	[1].stripe_input.total_count = 0,

	[1].dma_input.cmd = DMA_INPUT_COMMAND_ENABLE,
	[1].dma_input.format = DMA_INOUT_FORMAT_BAYER_PACKED,
	[1].dma_input.bitwidth = DMA_INOUT_BIT_WIDTH_10BIT,
	[1].dma_input.order = OTF_INPUT_ORDER_BAYER_GB_RG,
	[1].dma_input.plane = DMA_INOUT_PLANE_1,
	[1].dma_input.width = 4032,
	[1].dma_input.height = 3024,
	[1].dma_input.dma_crop_offset = 0,
	[1].dma_input.dma_crop_width = 4032,
	[1].dma_input.dma_crop_height = 3024,
	[1].dma_input.bayer_crop_offset_x = 0,
	[1].dma_input.bayer_crop_offset_y = 0,
	[1].dma_input.bayer_crop_width = 4032,
	[1].dma_input.bayer_crop_height = 3024,
	[1].dma_input.scene_mode = 0,
	[1].dma_input.msb = DMA_INOUT_BIT_WIDTH_10BIT - 1,
	[1].dma_input.stride_plane0 = 4032,
	[1].dma_input.stride_plane1 = 4032,
	[1].dma_input.stride_plane2 = 4032,
	[1].dma_input.v_otf_enable = OTF_INPUT_COMMAND_DISABLE,
	[1].dma_input.orientation = 0,
	[1].dma_input.strip_mode = 0,
	[1].dma_input.overlab_width = 0,
	[1].dma_input.strip_count = 0,
	[1].dma_input.strip_max_count = 0,
	[1].dma_input.sequence_id = 0,
	[1].dma_input.sbwc_type = NONE,

	[1].hdr.cmd = DMA_INPUT_COMMAND_DISABLE,

	[1].byr.cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[1].byr.format = DMA_INOUT_FORMAT_BAYER,
	[1].byr.bitwidth = DMA_INOUT_BIT_WIDTH_16BIT,
	[1].byr.order = OTF_INPUT_ORDER_BAYER_GB_RG,
	[1].byr.plane = DMA_INOUT_PLANE_1,
	[1].byr.width = 4032,
	[1].byr.height = 3024,
	[1].byr.dma_crop_offset_x = 0,
	[1].byr.dma_crop_offset_y = 0,
	[1].byr.dma_crop_width = 4032,
	[1].byr.dma_crop_height = 3024,
	[1].byr.crop_enable = 0,
	[1].byr.msb = DMA_INOUT_BIT_WIDTH_10BIT - 1,
	[1].byr.stride_plane0 = 4032,
	[1].byr.stride_plane1 = 4032,
	[1].byr.stride_plane2 = 4032,
	[1].byr.v_otf_enable = OTF_OUTPUT_COMMAND_DISABLE,
	[1].byr.sbwc_type = NONE,

	[1].byr_processed.cmd = DMA_INPUT_COMMAND_DISABLE,

	/* Param set[2]: RDMA(SBWC OFF) + BYR PROC WDMA(SBWC OFF) - Super night */
	[2].control.cmd = CONTROL_COMMAND_START,
	[2].control.bypass = 0,
	[2].control.strgen = CONTROL_COMMAND_STOP,

	[2].stripe_input.total_count = 0,

	[2].dma_input.cmd = DMA_INPUT_COMMAND_ENABLE,
	[2].dma_input.format = DMA_INOUT_FORMAT_BAYER_PACKED,
	[2].dma_input.bitwidth = DMA_INOUT_BIT_WIDTH_10BIT,
	[2].dma_input.order = OTF_INPUT_ORDER_BAYER_GB_RG,
	[2].dma_input.plane = DMA_INOUT_PLANE_1,
	[2].dma_input.width = 4032,
	[2].dma_input.height = 3024,
	[2].dma_input.dma_crop_offset = 0,
	[2].dma_input.dma_crop_width = 4032,
	[2].dma_input.dma_crop_height = 3024,
	[2].dma_input.bayer_crop_offset_x = 0,
	[2].dma_input.bayer_crop_offset_y = 0,
	[2].dma_input.bayer_crop_width = 4032,
	[2].dma_input.bayer_crop_height = 3024,
	[2].dma_input.scene_mode = 0,
	[2].dma_input.msb = DMA_INOUT_BIT_WIDTH_10BIT - 1,
	[2].dma_input.stride_plane0 = 4032,
	[2].dma_input.stride_plane1 = 4032,
	[2].dma_input.stride_plane2 = 4032,
	[2].dma_input.v_otf_enable = OTF_INPUT_COMMAND_DISABLE,
	[2].dma_input.orientation = 0,
	[2].dma_input.strip_mode = 0,
	[2].dma_input.overlab_width = 0,
	[2].dma_input.strip_count = 0,
	[2].dma_input.strip_max_count = 0,
	[2].dma_input.sequence_id = 0,
	[2].dma_input.sbwc_type = NONE,

	[2].hdr.cmd = DMA_INPUT_COMMAND_DISABLE,

	[2].byr.cmd = DMA_INPUT_COMMAND_DISABLE,

	[2].byr_processed.cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[2].byr_processed.format = DMA_INOUT_FORMAT_BAYER_PACKED,
	[2].byr_processed.bitwidth = DMA_INOUT_BIT_WIDTH_12BIT,
	[2].byr_processed.order = OTF_INPUT_ORDER_BAYER_GB_RG,
	[2].byr_processed.plane = DMA_INOUT_PLANE_1,
	[2].byr_processed.width = 4032,
	[2].byr_processed.height = 3024,
	[2].byr_processed.dma_crop_offset_x = 0,
	[2].byr_processed.dma_crop_offset_y = 0,
	[2].byr_processed.dma_crop_width = 4032,
	[2].byr_processed.dma_crop_height = 3024,
	[2].byr_processed.crop_enable = 0,
	[2].byr_processed.msb = DMA_INOUT_BIT_WIDTH_12BIT - 1,
	[2].byr_processed.stride_plane0 = 4032,
	[2].byr_processed.stride_plane1 = 4032,
	[2].byr_processed.stride_plane2 = 4032,
	[2].byr_processed.v_otf_enable = OTF_OUTPUT_COMMAND_DISABLE,
	[2].byr_processed.sbwc_type = NONE,
};

static DECLARE_BITMAP(result, ARRAY_SIZE(byrp_param_preset));

static void pst_set_buf_byrp(struct is_frame *frame, u32 param_idx)
{
	size_t size[IS_MAX_PLANES];
	u32 align = 32;
	dma_addr_t *dva;

	memset(size, 0x0, sizeof(size));

	switch (PARAM_BYRP_CONTROL + param_idx) {
	case PARAM_BYRP_DMA_INPUT:
		dva = frame->dvaddr_buffer;
		pst_get_size_of_dma_input(&byrp_param[param_idx], align, size);
		break;
	case PARAM_BYRP_HDR:
		dva = frame->dva_byrp_hdr;
		pst_get_size_of_dma_output(&byrp_param[param_idx], align, size);
		break;
	case PARAM_BYRP_BYR:
		dva = frame->dva_byrp_byr;
		pst_get_size_of_dma_output(&byrp_param[param_idx], align, size);
		break;
	case PARAM_BYRP_BYR_PROCESSED:
		dva = frame->dva_byrp_byr_processed;
		pst_get_size_of_dma_output(&byrp_param[param_idx], align, size);
		break;
	default:
		break;
	}

	if (size[0])
		pb[param_idx] = pst_set_dva(frame, dva, size, GROUP_ID_BYRP);
}

static void pst_init_param_byrp(unsigned int index)
{
	int i = 0;

	memcpy(byrp_param[i++], (u32 *)&byrp_param_preset[index].control, PARAMETER_MAX_SIZE);
	memcpy(byrp_param[i++], (u32 *)&byrp_param_preset[index].otf_output, PARAMETER_MAX_SIZE);
	memcpy(byrp_param[i++], (u32 *)&byrp_param_preset[index].dma_input, PARAMETER_MAX_SIZE);
	memcpy(byrp_param[i++], (u32 *)&byrp_param_preset[index].stripe_input, PARAMETER_MAX_SIZE);
	memcpy(byrp_param[i++], (u32 *)&byrp_param_preset[index].hdr, PARAMETER_MAX_SIZE);
	memcpy(byrp_param[i++], (u32 *)&byrp_param_preset[index].byr, PARAMETER_MAX_SIZE);
	memcpy(byrp_param[i++], (u32 *)&byrp_param_preset[index].byr_processed, PARAMETER_MAX_SIZE);
}

static void pst_set_param_byrp(struct is_frame *frame)
{
	int i;

	frame->instance = 0;
	frame->fcount = 1234;
	frame->num_buffers = 1;

	for (i = 0; i < NUM_OF_BYRP_PARAM; i++) {
		pst_set_param(frame, byrp_param[i], PARAM_BYRP_CONTROL + i);
		pst_set_buf_byrp(frame, i);
	}
}

static void pst_clr_param_byrp(struct is_frame *frame)
{
	int i;

	for (i = 0; i < NUM_OF_BYRP_PARAM; i++) {
		if (!pb[i])
			continue;

		pst_clr_dva(pb[i]);
		pb[i] = NULL;
	}
}

static void pst_set_rta_info_byrp(struct is_frame *frame, struct size_cr_set *cr_set)
{
	frame->kva_byrp_rta_info[PLANE_INDEX_CR_SET] = (u64)cr_set;
}

static const struct pst_callback_ops pst_cb_byrp = {
	.init_param = pst_init_param_byrp,
	.set_param = pst_set_param_byrp,
	.clr_param = pst_clr_param_byrp,
	.set_rta_info = pst_set_rta_info_byrp,
};

static int pst_set_hw_byrp(const char *val, const struct kernel_param *kp)
{
	return pst_set_hw_ip(val,
			DEV_HW_BYRP,
			hw_ip_byrp,
			frame_byrp,
			(u32 **)byrp_param,
			&byrp_cr_set,
			ARRAY_SIZE(byrp_param_preset),
			result,
			&pst_cb_byrp);
}

static int pst_get_hw_byrp(char *buffer, const struct kernel_param *kp)
{
	return pst_get_hw_ip(buffer, hw_ip_byrp, ARRAY_SIZE(byrp_param_preset), result);
}
