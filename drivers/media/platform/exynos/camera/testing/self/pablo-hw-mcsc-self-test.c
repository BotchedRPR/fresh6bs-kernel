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
#include "../pablo-self-test-result.h"
#include "pablo-framemgr.h"
#include "is-hw.h"
#include "is-core.h"
#include "is-device-ischain.h"

static int pst_set_hw_mcsc(const char *val, const struct kernel_param *kp);
static int pst_get_hw_mcsc(char *buffer, const struct kernel_param *kp);
static const struct kernel_param_ops pablo_param_ops_hw_mcsc = {
	.set = pst_set_hw_mcsc,
	.get = pst_get_hw_mcsc,
};
module_param_cb(test_hw_mcsc, &pablo_param_ops_hw_mcsc, NULL, 0644);

#define NUM_OF_MCSC_PARAM (PARAM_MCS_STRIPE_INPUT - PARAM_MCS_CONTROL + 1)

static struct is_hw_ip *hw_ip_mcsc;
static struct is_frame *frame_mcsc;
static u32 mcsc_param[NUM_OF_MCSC_PARAM][PARAMETER_MAX_MEMBER];
static struct is_priv_buf *pb[NUM_OF_MCSC_PARAM];
static struct size_cr_set mcsc_cr_set;

static const struct mcs_param mcsc_param_preset[] = {
	/* Param set[0]: STRGEN */
	[0].control.cmd = CONTROL_COMMAND_START,
	[0].control.bypass = 0,
	[0].control.strgen = CONTROL_COMMAND_START,

	[0].input.otf_cmd = OTF_INPUT_COMMAND_ENABLE,
	[0].input.otf_format = OTF_INPUT_FORMAT_YUV422,
	[0].input.otf_bitwidth = OTF_INPUT_BIT_WIDTH_10BIT,
	[0].input.otf_order = 0,
	[0].input.dma_cmd = DMA_INPUT_COMMAND_DISABLE,
	[0].input.dma_format = 0,
	[0].input.dma_bitwidth = 0,
	[0].input.dma_order = 0,
	[0].input.plane = 0,
	[0].input.width = 1920,
	[0].input.height = 1440,
	[0].input.dma_stride_y = 0,
	[0].input.dma_stride_c = 0,
	[0].input.dma_crop_offset_x = 0,
	[0].input.dma_crop_offset_y = 0,
	[0].input.dma_crop_width = 0,
	[0].input.dma_crop_height = 0,
	[0].input.djag_out_width = 0,
	[0].input.djag_out_height = 0,
	[0].input.stripe_in_start_pos_x = 0,
	[0].input.stripe_in_end_pos_x = 0,
	[0].input.stripe_roi_start_pos_x = 0,
	[0].input.stripe_roi_end_pos_x = 0,
	[0].input.sbwc_type = 0,

	[0].output[0].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[0].output[0].dma_cmd = DMA_OUTPUT_COMMAND_DISABLE,

	[0].output[1].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[0].output[1].dma_cmd = DMA_OUTPUT_COMMAND_DISABLE,

	[0].output[2].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[0].output[2].dma_cmd = DMA_OUTPUT_COMMAND_DISABLE,

	[0].output[3].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[0].output[3].dma_cmd = DMA_OUTPUT_COMMAND_DISABLE,

	[0].output[4].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[0].output[4].dma_cmd = DMA_OUTPUT_COMMAND_DISABLE,

	[0].output[5].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[0].output[5].dma_cmd = DMA_OUTPUT_COMMAND_DISABLE,

	[0].stripe_input.total_count = 0,

	/* Param set[1]: STRGEN + WDMA[P0~4] YUV420 2P */
	[1].control.cmd = CONTROL_COMMAND_START,
	[1].control.bypass = 0,
	[1].control.strgen = CONTROL_COMMAND_START,

	[1].input.otf_cmd = OTF_INPUT_COMMAND_ENABLE,
	[1].input.otf_format = OTF_INPUT_FORMAT_YUV422,
	[1].input.otf_bitwidth = OTF_INPUT_BIT_WIDTH_10BIT,
	[1].input.otf_order = 0,
	[1].input.dma_cmd = DMA_INPUT_COMMAND_DISABLE,
	[1].input.dma_format = 0,
	[1].input.dma_bitwidth = 0,
	[1].input.dma_order = 0,
	[1].input.plane = 0,
	[1].input.width = 1920,
	[1].input.height = 1440,
	[1].input.dma_stride_y = 0,
	[1].input.dma_stride_c = 0,
	[1].input.dma_crop_offset_x = 0,
	[1].input.dma_crop_offset_y = 0,
	[1].input.dma_crop_width = 0,
	[1].input.dma_crop_height = 0,
	[1].input.djag_out_width = 0,
	[1].input.djag_out_height = 0,
	[1].input.stripe_in_start_pos_x = 0,
	[1].input.stripe_in_end_pos_x = 0,
	[1].input.stripe_roi_start_pos_x = 0,
	[1].input.stripe_roi_end_pos_x = 0,
	[1].input.sbwc_type = 0,

	[1].output[0].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[1].output[0].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[1].output[0].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[1].output[0].otf_order = 0,
	[1].output[0].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[1].output[0].dma_format = DMA_INOUT_FORMAT_YUV420,
	[1].output[0].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[1].output[0].dma_order = 1,
	[1].output[0].plane = 2,
	[1].output[0].crop_offset_x = 0,
	[1].output[0].crop_offset_y = 180,
	[1].output[0].crop_width = 1920,
	[1].output[0].crop_height = 1080,
	[1].output[0].width = 1920,
	[1].output[0].height = 1080,
	[1].output[0].dma_stride_y = 1920,
	[1].output[0].dma_stride_c = 1920,
	[1].output[0].yuv_range = 0,
	[1].output[0].flip = 0,
	[1].output[0].hwfc = 0,
	[1].output[0].offset_x = 0,
	[1].output[0].offset_y = 0,
	[1].output[0].cmd = 2,
	[1].output[0].stripe_in_start_pos_x = 0,
	[1].output[0].stripe_roi_start_pos_x = 0,
	[1].output[0].stripe_roi_end_pos_x = 0,
	[1].output[0].full_input_width = 1920,
	[1].output[0].full_output_width = 1920,
	[1].output[0].sbwc_type = 0,
	[1].output[0].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[1].output[1].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[1].output[1].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[1].output[1].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[1].output[1].otf_order = 0,
	[1].output[1].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[1].output[1].dma_format = DMA_INOUT_FORMAT_YUV420,
	[1].output[1].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[1].output[1].dma_order = 1,
	[1].output[1].plane = 2,
	[1].output[1].crop_offset_x = 0,
	[1].output[1].crop_offset_y = 180,
	[1].output[1].crop_width = 1920,
	[1].output[1].crop_height = 1080,
	[1].output[1].width = 1920,
	[1].output[1].height = 1080,
	[1].output[1].dma_stride_y = 1920,
	[1].output[1].dma_stride_c = 1920,
	[1].output[1].yuv_range = 0,
	[1].output[1].flip = 0,
	[1].output[1].hwfc = 0,
	[1].output[1].offset_x = 0,
	[1].output[1].offset_y = 0,
	[1].output[1].cmd = 2,
	[1].output[1].stripe_in_start_pos_x = 0,
	[1].output[1].stripe_roi_start_pos_x = 0,
	[1].output[1].stripe_roi_end_pos_x = 0,
	[1].output[1].full_input_width = 1920,
	[1].output[1].full_output_width = 1920,
	[1].output[1].sbwc_type = 0,
	[1].output[1].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[1].output[2].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[1].output[2].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[1].output[2].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[1].output[2].otf_order = 0,
	[1].output[2].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[1].output[2].dma_format = DMA_INOUT_FORMAT_YUV420,
	[1].output[2].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[1].output[2].dma_order = 1,
	[1].output[2].plane = 2,
	[1].output[2].crop_offset_x = 0,
	[1].output[2].crop_offset_y = 180,
	[1].output[2].crop_width = 1920,
	[1].output[2].crop_height = 1080,
	[1].output[2].width = 1920,
	[1].output[2].height = 1080,
	[1].output[2].dma_stride_y = 1920,
	[1].output[2].dma_stride_c = 1920,
	[1].output[2].yuv_range = 0,
	[1].output[2].flip = 0,
	[1].output[2].hwfc = 0,
	[1].output[2].offset_x = 0,
	[1].output[2].offset_y = 0,
	[1].output[2].cmd = 2,
	[1].output[2].stripe_in_start_pos_x = 0,
	[1].output[2].stripe_roi_start_pos_x = 0,
	[1].output[2].stripe_roi_end_pos_x = 0,
	[1].output[2].full_input_width = 1920,
	[1].output[2].full_output_width = 1920,
	[1].output[2].sbwc_type = 0,
	[1].output[2].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[1].output[3].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[1].output[3].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[1].output[3].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[1].output[3].otf_order = 0,
	[1].output[3].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[1].output[3].dma_format = DMA_INOUT_FORMAT_YUV420,
	[1].output[3].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[1].output[3].dma_order = 1,
	[1].output[3].plane = 2,
	[1].output[3].crop_offset_x = 0,
	[1].output[3].crop_offset_y = 180,
	[1].output[3].crop_width = 1920,
	[1].output[3].crop_height = 1080,
	[1].output[3].width = 1920,
	[1].output[3].height = 1080,
	[1].output[3].dma_stride_y = 1920,
	[1].output[3].dma_stride_c = 1920,
	[1].output[3].yuv_range = 0,
	[1].output[3].flip = 0,
	[1].output[3].hwfc = 0,
	[1].output[3].offset_x = 0,
	[1].output[3].offset_y = 0,
	[1].output[3].cmd = 2,
	[1].output[3].stripe_in_start_pos_x = 0,
	[1].output[3].stripe_roi_start_pos_x = 0,
	[1].output[3].stripe_roi_end_pos_x = 0,
	[1].output[3].full_input_width = 1920,
	[1].output[3].full_output_width = 1920,
	[1].output[3].sbwc_type = 0,
	[1].output[3].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[1].output[4].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[1].output[4].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[1].output[4].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[1].output[4].otf_order = 0,
	[1].output[4].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[1].output[4].dma_format = DMA_INOUT_FORMAT_YUV420,
	[1].output[4].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[1].output[4].dma_order = 1,
	[1].output[4].plane = 2,
	[1].output[4].crop_offset_x = 0,
	[1].output[4].crop_offset_y = 180,
	[1].output[4].crop_width = 1920,
	[1].output[4].crop_height = 1080,
	[1].output[4].width = 1920,
	[1].output[4].height = 1080,
	[1].output[4].dma_stride_y = 1920,
	[1].output[4].dma_stride_c = 1920,
	[1].output[4].yuv_range = 0,
	[1].output[4].flip = 0,
	[1].output[4].hwfc = 0,
	[1].output[4].offset_x = 0,
	[1].output[4].offset_y = 0,
	[1].output[4].cmd = 2,
	[1].output[4].stripe_in_start_pos_x = 0,
	[1].output[4].stripe_roi_start_pos_x = 0,
	[1].output[4].stripe_roi_end_pos_x = 0,
	[1].output[4].full_input_width = 1920,
	[1].output[4].full_output_width = 1920,
	[1].output[4].sbwc_type = 0,
	[1].output[4].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[1].stripe_input.total_count = 0,

	/* Param set[2]: STRGEN + WDMA[P0~4] YUV422 1P */
	[2].control.cmd = CONTROL_COMMAND_START,
	[2].control.bypass = 0,
	[2].control.strgen = CONTROL_COMMAND_START,

	[2].input.otf_cmd = OTF_INPUT_COMMAND_ENABLE,
	[2].input.otf_format = OTF_INPUT_FORMAT_YUV422,
	[2].input.otf_bitwidth = OTF_INPUT_BIT_WIDTH_10BIT,
	[2].input.otf_order = 0,
	[2].input.dma_cmd = DMA_INPUT_COMMAND_DISABLE,
	[2].input.dma_format = 0,
	[2].input.dma_bitwidth = 0,
	[2].input.dma_order = 0,
	[2].input.plane = 0,
	[2].input.width = 1920,
	[2].input.height = 1440,
	[2].input.dma_stride_y = 0,
	[2].input.dma_stride_c = 0,
	[2].input.dma_crop_offset_x = 0,
	[2].input.dma_crop_offset_y = 0,
	[2].input.dma_crop_width = 0,
	[2].input.dma_crop_height = 0,
	[2].input.djag_out_width = 0,
	[2].input.djag_out_height = 0,
	[2].input.stripe_in_start_pos_x = 0,
	[2].input.stripe_in_end_pos_x = 0,
	[2].input.stripe_roi_start_pos_x = 0,
	[2].input.stripe_roi_end_pos_x = 0,
	[2].input.sbwc_type = 0,

	[2].output[0].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[2].output[0].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[2].output[0].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[2].output[0].otf_order = 0,
	[2].output[0].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[2].output[0].dma_format = DMA_INOUT_FORMAT_YUV422,
	[2].output[0].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[2].output[0].dma_order = 1,
	[2].output[0].plane = 1,
	[2].output[0].crop_offset_x = 0,
	[2].output[0].crop_offset_y = 180,
	[2].output[0].crop_width = 1920,
	[2].output[0].crop_height = 1080,
	[2].output[0].width = 1920,
	[2].output[0].height = 1080,
	[2].output[0].dma_stride_y = 1920,
	[2].output[0].dma_stride_c = 1920,
	[2].output[0].yuv_range = 0,
	[2].output[0].flip = 0,
	[2].output[0].hwfc = 0,
	[2].output[0].offset_x = 0,
	[2].output[0].offset_y = 0,
	[2].output[0].cmd = 2,
	[2].output[0].stripe_in_start_pos_x = 0,
	[2].output[0].stripe_roi_start_pos_x = 0,
	[2].output[0].stripe_roi_end_pos_x = 0,
	[2].output[0].full_input_width = 1920,
	[2].output[0].full_output_width = 1920,
	[2].output[0].sbwc_type = 0,
	[2].output[0].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[2].output[1].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[2].output[1].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[2].output[1].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[2].output[1].otf_order = 0,
	[2].output[1].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[2].output[1].dma_format = DMA_INOUT_FORMAT_YUV422,
	[2].output[1].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[2].output[1].dma_order = 1,
	[2].output[1].plane = 1,
	[2].output[1].crop_offset_x = 0,
	[2].output[1].crop_offset_y = 180,
	[2].output[1].crop_width = 1920,
	[2].output[1].crop_height = 1080,
	[2].output[1].width = 1920,
	[2].output[1].height = 1080,
	[2].output[1].dma_stride_y = 1920,
	[2].output[1].dma_stride_c = 1920,
	[2].output[1].yuv_range = 0,
	[2].output[1].flip = 0,
	[2].output[1].hwfc = 0,
	[2].output[1].offset_x = 0,
	[2].output[1].offset_y = 0,
	[2].output[1].cmd = 2,
	[2].output[1].stripe_in_start_pos_x = 0,
	[2].output[1].stripe_roi_start_pos_x = 0,
	[2].output[1].stripe_roi_end_pos_x = 0,
	[2].output[1].full_input_width = 1920,
	[2].output[1].full_output_width = 1920,
	[2].output[1].sbwc_type = 0,
	[2].output[1].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[2].output[2].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[2].output[2].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[2].output[2].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[2].output[2].otf_order = 0,
	[2].output[2].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[2].output[2].dma_format = DMA_INOUT_FORMAT_YUV422,
	[2].output[2].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[2].output[2].dma_order = 1,
	[2].output[2].plane = 1,
	[2].output[2].crop_offset_x = 0,
	[2].output[2].crop_offset_y = 180,
	[2].output[2].crop_width = 1920,
	[2].output[2].crop_height = 1080,
	[2].output[2].width = 1920,
	[2].output[2].height = 1080,
	[2].output[2].dma_stride_y = 1920,
	[2].output[2].dma_stride_c = 1920,
	[2].output[2].yuv_range = 0,
	[2].output[2].flip = 0,
	[2].output[2].hwfc = 0,
	[2].output[2].offset_x = 0,
	[2].output[2].offset_y = 0,
	[2].output[2].cmd = 2,
	[2].output[2].stripe_in_start_pos_x = 0,
	[2].output[2].stripe_roi_start_pos_x = 0,
	[2].output[2].stripe_roi_end_pos_x = 0,
	[2].output[2].full_input_width = 1920,
	[2].output[2].full_output_width = 1920,
	[2].output[2].sbwc_type = 0,
	[2].output[2].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[2].output[3].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[2].output[3].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[2].output[3].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[2].output[3].otf_order = 0,
	[2].output[3].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[2].output[3].dma_format = DMA_INOUT_FORMAT_YUV422,
	[2].output[3].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[2].output[3].dma_order = 1,
	[2].output[3].plane = 1,
	[2].output[3].crop_offset_x = 0,
	[2].output[3].crop_offset_y = 180,
	[2].output[3].crop_width = 1920,
	[2].output[3].crop_height = 1080,
	[2].output[3].width = 1920,
	[2].output[3].height = 1080,
	[2].output[3].dma_stride_y = 1920,
	[2].output[3].dma_stride_c = 1920,
	[2].output[3].yuv_range = 0,
	[2].output[3].flip = 0,
	[2].output[3].hwfc = 0,
	[2].output[3].offset_x = 0,
	[2].output[3].offset_y = 0,
	[2].output[3].cmd = 2,
	[2].output[3].stripe_in_start_pos_x = 0,
	[2].output[3].stripe_roi_start_pos_x = 0,
	[2].output[3].stripe_roi_end_pos_x = 0,
	[2].output[3].full_input_width = 1920,
	[2].output[3].full_output_width = 1920,
	[2].output[3].sbwc_type = 0,
	[2].output[3].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[2].output[4].otf_cmd = OTF_OUTPUT_COMMAND_DISABLE,
	[2].output[4].otf_format = OTF_OUTPUT_FORMAT_YUV422,
	[2].output[4].otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT,
	[2].output[4].otf_order = 0,
	[2].output[4].dma_cmd = DMA_OUTPUT_COMMAND_ENABLE,
	[2].output[4].dma_format = DMA_INOUT_FORMAT_YUV422,
	[2].output[4].dma_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT,
	[2].output[4].dma_order = 1,
	[2].output[4].plane = 1,
	[2].output[4].crop_offset_x = 0,
	[2].output[4].crop_offset_y = 180,
	[2].output[4].crop_width = 1920,
	[2].output[4].crop_height = 1080,
	[2].output[4].width = 1920,
	[2].output[4].height = 1080,
	[2].output[4].dma_stride_y = 1920,
	[2].output[4].dma_stride_c = 1920,
	[2].output[4].yuv_range = 0,
	[2].output[4].flip = 0,
	[2].output[4].hwfc = 0,
	[2].output[4].offset_x = 0,
	[2].output[4].offset_y = 0,
	[2].output[4].cmd = 2,
	[2].output[4].stripe_in_start_pos_x = 0,
	[2].output[4].stripe_roi_start_pos_x = 0,
	[2].output[4].stripe_roi_end_pos_x = 0,
	[2].output[4].full_input_width = 1920,
	[2].output[4].full_output_width = 1920,
	[2].output[4].sbwc_type = 0,
	[2].output[4].bitsperpixel = (8 << 0) | (8 << 8) | (0 << 16),

	[2].stripe_input.total_count = 0,
};

static DECLARE_BITMAP(result, ARRAY_SIZE(mcsc_param_preset));

static void pst_set_buf_mcsc(struct is_frame *frame, u32 param_idx)
{
	size_t size[IS_MAX_PLANES];
	u32 align = 32;
	dma_addr_t *dva;

	memset(size, 0x0, sizeof(size));

	switch (PARAM_MCSC_CONTROL + param_idx) {
	case PARAM_MCS_OUTPUT0:
		dva = frame->sc0TargetAddress;
		break;
	case PARAM_MCS_OUTPUT1:
		dva = frame->sc1TargetAddress;
		break;
	case PARAM_MCS_OUTPUT2:
		dva = frame->sc2TargetAddress;
		break;
	case PARAM_MCS_OUTPUT3:
		dva = frame->sc3TargetAddress;
		break;
	case PARAM_MCS_OUTPUT4:
		dva = frame->sc4TargetAddress;
		break;
	case PARAM_MCS_OUTPUT5:
		dva = frame->sc5TargetAddress;
		break;
	default:
		break;
	}

	pst_get_size_of_mcs_output(&mcsc_param[param_idx], align, size);

	if (size[0])
		pb[param_idx] = pst_set_dva(frame, dva, size, GROUP_ID_MCS0);
}

static void pst_init_param_mcsc(unsigned int index)
{
	int i = 0;

	memcpy(mcsc_param[i++], (u32 *)&mcsc_param_preset[index].control, PARAMETER_MAX_SIZE);
	memcpy(mcsc_param[i++], (u32 *)&mcsc_param_preset[index].input, PARAMETER_MAX_SIZE);
	memcpy(mcsc_param[i++], (u32 *)&mcsc_param_preset[index].output[0], PARAMETER_MAX_SIZE);
	memcpy(mcsc_param[i++], (u32 *)&mcsc_param_preset[index].output[1], PARAMETER_MAX_SIZE);
	memcpy(mcsc_param[i++], (u32 *)&mcsc_param_preset[index].output[2], PARAMETER_MAX_SIZE);
	memcpy(mcsc_param[i++], (u32 *)&mcsc_param_preset[index].output[3], PARAMETER_MAX_SIZE);
	memcpy(mcsc_param[i++], (u32 *)&mcsc_param_preset[index].output[4], PARAMETER_MAX_SIZE);
	memcpy(mcsc_param[i++], (u32 *)&mcsc_param_preset[index].output[5], PARAMETER_MAX_SIZE);
	memcpy(mcsc_param[i++], (u32 *)&mcsc_param_preset[index].stripe_input, PARAMETER_MAX_SIZE);
}

static void pst_set_param_mcsc(struct is_frame *frame)
{
	int i;

	frame->instance = 0;
	frame->fcount = 1234;
	frame->num_buffers = 1;

	for (i = 0; i < NUM_OF_MCSC_PARAM; i++) {
		pst_set_param(frame, mcsc_param[i], PARAM_MCSC_CONTROL + i);
		pst_set_buf_mcsc(frame, i);
	}
}

static void pst_clr_param_mcsc(struct is_frame *frame)
{
	int i;

	for (i = 0; i < NUM_OF_MCSC_PARAM; i++) {
		if (!pb[i])
			continue;

		/* Ex) You can check vaidation of buffer data by using kva. */
		/* print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4, pb[i]->kva, SZ_4K, false); */

		pst_clr_dva(pb[i]);
		pb[i] = NULL;
	}
}

static const struct pst_callback_ops pst_cb_mcsc = {
	.init_param = pst_init_param_mcsc,
	.set_param = pst_set_param_mcsc,
	.clr_param = pst_clr_param_mcsc,
};

static int pst_set_hw_mcsc(const char *val, const struct kernel_param *kp)
{
	return pst_set_hw_ip(val,
			DEV_HW_MCSC0,
			hw_ip_mcsc,
			frame_mcsc,
			(u32 **)mcsc_param,
			&mcsc_cr_set,
			ARRAY_SIZE(mcsc_param_preset),
			result,
			&pst_cb_mcsc);
}

static int pst_get_hw_mcsc(char *buffer, const struct kernel_param *kp)
{
	return pst_get_hw_ip(buffer, hw_ip_mcsc, ARRAY_SIZE(mcsc_param_preset), result);
}
