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

#include "is-device-ischain.h"
#include "is-device-sensor.h"
#include "is-subdev-ctrl.h"
#include "is-config.h"
#include "is-param.h"
#include "is-video.h"
#include "is-type.h"

static int __orb_dma_in_cfg(struct is_device_ischain *device,
	struct is_subdev *leader,
	struct is_frame *ldr_frame,
	struct camera2_node *node,
	struct param_orbmch_dma *dma,
	u32 pindex,
	IS_DECLARE_PMAP(pmap))
{
	int ret = 0;
	struct is_fmt *fmt;
	struct param_control *control;
	struct is_group *group;
	struct is_crop *incrop;
	u32 hw_msb, hw_order, hw_plane;
	u32 hw_bitwidth;

	FIMC_BUG(!leader);
	FIMC_BUG(!device);
	FIMC_BUG(!ldr_frame);
	FIMC_BUG(!node);

	if (dma->cur_input_cmd != node->request)
		mlvinfo("[F%d][%d] RDMA enable: %d -> %d\n", device,
			node->vid, ldr_frame->fcount, pindex,
			dma->cur_input_cmd, node->request);

	if (!node->request) {
		dma->cur_input_cmd = DMA_INPUT_COMMAND_DISABLE;
		set_bit(PARAM_ORB_DMA, pmap);

		return 0;
	}

	incrop = (struct is_crop *)node->input.cropRegion;
	if (IS_NULL_CROP(incrop)) {
		merr("incrop is NULL", device);
		return -EINVAL;
	}
	group = &device->group_orb;
	fmt = is_find_format(node->pixelformat, node->flags);
	if (!fmt) {
		merr("pixel format(0x%x) is not found", device, node->pixelformat);
		return -EINVAL;
	}
	hw_bitwidth = fmt->hw_bitwidth;
	hw_msb = fmt->hw_bitwidth - 1;
	hw_order = fmt->hw_order;
	hw_plane = fmt->hw_plane;

	if ((dma->cur_input_width != incrop->w) ||
		(dma->cur_input_height != incrop->h))
		mlvinfo("[F%d]RDMA bcrop[%d, %d, %d, %d]\n",
			device, node->vid, ldr_frame->fcount,
			incrop->x, incrop->y, incrop->w, incrop->h);
	if (dma->input_bitwidth != fmt->hw_bitwidth)
		mlvinfo("[F%d]RDMA bitwidth: %d -> %d\n", device,
			node->vid, ldr_frame->fcount,
			dma->input_bitwidth, fmt->hw_bitwidth);

	control = is_itf_g_param(device, ldr_frame, PARAM_ORB_CONTROL);
	if (test_bit(IS_GROUP_START, &group->state)) {
		control->cmd = CONTROL_COMMAND_START;
		control->bypass = CONTROL_BYPASS_DISABLE;
	} else {
		control->cmd = CONTROL_COMMAND_STOP;
		control->bypass = CONTROL_BYPASS_DISABLE;
	}
	set_bit(PARAM_ORB_CONTROL, pmap);

	dma->cur_input_cmd = DMA_INPUT_COMMAND_ENABLE;
	dma->input_format = DMA_INOUT_FORMAT_Y;
	dma->input_bitwidth = hw_bitwidth;
	dma->input_order = hw_order;
	dma->input_msb = hw_msb;
	dma->input_plane = hw_plane;
	dma->cur_input_width = incrop->w;
	dma->cur_input_height = incrop->h;

	set_bit(pindex, pmap);

	return ret;
}

static int __orb_dma_out_cfg(struct is_device_ischain *device,
	struct is_subdev *leader,
	struct is_frame *ldr_frame,
	struct camera2_node *node,
	struct param_orbmch_dma *dma,
	u32 pindex,
	IS_DECLARE_PMAP(pmap))
{
	struct is_fmt *fmt;
	struct is_crop *otcrop;
	u32 hw_msb, hw_order, hw_plane;
	u32 hw_bitwidth;
	int ret = 0;
	u32 i;

	FIMC_BUG(!leader);
	FIMC_BUG(!device);
	FIMC_BUG(!ldr_frame);
	FIMC_BUG(!node);

	if (dma->output_cmd != node->request)
		mlvinfo("[F%d][%d] WDMA enable: %d -> %d\n", device,
			node->vid, ldr_frame->fcount, pindex,
			dma->output_cmd, node->request);

	if (!node->request) {
		switch (node->vid) {
		case IS_VIDEO_ORB0C_NUM:
		case IS_VIDEO_ORB1C_NUM:
			dma->output_cmd = DMA_OUTPUT_COMMAND_DISABLE;
			for (i = 0; i < IS_MAX_PLANES; i++)
				ldr_frame->orbxcKTargetAddress[i] = 0;
			ret = -EINVAL;
			break;
		case IS_VIDEO_ORB0M_NUM:
		case IS_VIDEO_ORB1M_NUM:
			dma->output_cmd = DMA_OUTPUT_COMMAND_DISABLE;
			ret = -EINVAL;
			break;
		default:
			break;
		}
		set_bit(PARAM_ORB_DMA, pmap);
		if (ret)
			mlvwarn("[F%d] Skip this frame by WDMA disable", device, node->vid, ldr_frame->fcount);

		return ret;
	}

	otcrop = (struct is_crop *)node->output.cropRegion;
	if (IS_NULL_CROP(otcrop)) {
		merr("otcrop is NULL", device);
		return -EINVAL;
	}

	fmt = is_find_format(node->pixelformat, node->flags);
	if (!fmt) {
		merr("pixel format(0x%x) is not found", device, node->pixelformat);
		return -EINVAL;
	}

	hw_bitwidth = fmt->hw_bitwidth;
	hw_msb = hw_bitwidth - 1;
	hw_order = fmt->hw_order;
	hw_plane = fmt->hw_plane;

	switch (node->vid) {
	case IS_VIDEO_ORB0C_NUM:
	case IS_VIDEO_ORB1C_NUM:
		if ((dma->output_width != otcrop->w) ||
			(dma->output_height != otcrop->h))
			mlvinfo("[F%d]WDMA bcrop[%d, %d, %d, %d]\n",
				device, node->vid, ldr_frame->fcount,
				otcrop->x, otcrop->y, otcrop->w, otcrop->h);
		if (dma->output_bitwidth != fmt->hw_bitwidth)
			mlvinfo("[F%d]WDMA bitwidth: %d -> %d\n", device,
				node->vid, ldr_frame->fcount,
				dma->output_bitwidth, fmt->hw_bitwidth);
		if (dma->output_plane != fmt->hw_plane)
			mlvinfo("[F%d]WDMA plane: %d -> %d\n", device,
				node->vid, ldr_frame->fcount,
				dma->output_plane, fmt->hw_plane);
		hw_plane = 2;
		dma->output_cmd = DMA_INPUT_COMMAND_ENABLE;
		dma->output_format = DMA_INOUT_FORMAT_Y;
		dma->output_order = hw_order;
		dma->output_bitwidth = hw_bitwidth;
		dma->output_plane = hw_plane;
		dma->output_msb = hw_msb;
		dma->output_width = otcrop->w;
		dma->output_height = otcrop->h;
		break;
	}

	return ret;
}

static int is_ischain_orb_cfg(struct is_subdev *leader,
	void *device_data,
	struct is_frame *frame,
	struct is_crop *incrop,
	struct is_crop *otcrop,
	IS_DECLARE_PMAP(pmap))
{
	int ret = 0;
	struct is_group *group;
	struct is_queue *queue;
	struct param_orbmch_dma *dma_input;
	struct param_control *control;
	struct is_device_ischain *device;
	u32 hw_format = DMA_INOUT_FORMAT_Y;
	u32 hw_bitwidth = DMA_INOUT_BIT_WIDTH_8BIT;
	u32 hw_plane = DMA_INOUT_PLANE_1;
	u32 hw_order = DMA_INOUT_ORDER_NO;
	u32 width, height;

	device = (struct is_device_ischain *)device_data;

	FIMC_BUG(!leader);
	FIMC_BUG(!device);
	FIMC_BUG(!incrop);

	width = incrop->w;
	height = incrop->h;
	group = &device->group_orb;
	queue = GET_SUBDEV_QUEUE(leader);
	if (!queue) {
		merr("queue is NULL", device);
		ret = -EINVAL;
		goto p_err;
	}

	if (!test_bit(IS_GROUP_OTF_INPUT, &group->state)) {
		if (!queue->framecfg.format) {
			merr("format is NULL", device);
			ret = -EINVAL;
			goto p_err;
		}

		hw_format = queue->framecfg.format->hw_format;
		hw_bitwidth = queue->framecfg.format->hw_bitwidth; /* memory width per pixel */
		hw_order = queue->framecfg.format->hw_order;
		hw_plane = queue->framecfg.format->hw_plane;
	}

	/* Configure Conrtol */
	if (!frame) {
		control = is_itf_g_param(device, NULL, PARAM_ORB_CONTROL);
		if (test_bit(IS_GROUP_START, &group->state)) {
			control->cmd = CONTROL_COMMAND_START;
			control->bypass = CONTROL_BYPASS_DISABLE;
		} else {
			control->cmd = CONTROL_COMMAND_STOP;
			control->bypass = CONTROL_BYPASS_DISABLE;
		}

		set_bit(PARAM_ORB_CONTROL, pmap);
	}

	dma_input = is_itf_g_param(device, frame, PARAM_ORB_DMA);
	if (test_bit(IS_GROUP_OTF_INPUT, &group->state))
		dma_input->cur_input_cmd = DMA_INPUT_COMMAND_DISABLE;
	else
		dma_input->cur_input_cmd = DMA_INPUT_COMMAND_ENABLE;

	dma_input->input_format = hw_format;
	dma_input->input_bitwidth = hw_bitwidth;
	dma_input->input_order = hw_order;
	dma_input->input_msb = hw_bitwidth - 1;
	dma_input->input_plane = hw_plane;
	dma_input->cur_input_width = width;
	dma_input->cur_input_height = height;

	set_bit(PARAM_ORB_DMA, pmap);

p_err:
	return ret;
}

static int is_ischain_orb_tag(struct is_subdev *subdev,
	void *device_data,
	struct is_frame *frame,
	struct camera2_node *node)
{
	int ret = 0;
	struct is_group *group;
	struct orbmch_param *orbmch_param;
	struct is_crop inparm;
	struct is_crop *incrop, *otcrop;
	struct is_subdev *leader;
	struct is_device_ischain *device;
	IS_DECLARE_PMAP(pmap);
	struct is_queue *queue;
	struct camera2_node *out_node = NULL;
	struct camera2_node *cap_node = NULL;
	struct is_sub_frame *sframe;
	struct param_orbmch_dma *dma = NULL;
	dma_addr_t *targetAddr;
	u32 num_planes;
	u64 *targetAddr_k;
	int j, i, p;

	device = (struct is_device_ischain *)device_data;

	FIMC_BUG(!subdev);
	FIMC_BUG(!device);
	FIMC_BUG(!device->is_region);
	FIMC_BUG(!frame);

	mdbgs_ischain(4, "ORBMCH TAG\n", device);

	incrop = (struct is_crop *)node->input.cropRegion;
	otcrop = (struct is_crop *)node->output.cropRegion;

	group = &device->group_orb;
	leader = subdev->leader;
	IS_INIT_PMAP(pmap);

	orbmch_param = &device->is_region->parameter.orbmch;

	if (IS_ENABLED(LOGICAL_VIDEO_NODE)) {
		queue = GET_SUBDEV_QUEUE(leader);
		if (!queue) {
			merr("queue is NULL", device);
			ret = -EINVAL;
			goto p_err;
		}

		if (queue->mode == CAMERA_NODE_NORMAL)
			goto p_skip;

		out_node = &frame->shot_ext->node_group.leader;
		dma = is_itf_g_param(device, frame, PARAM_ORB_DMA);
		ret = __orb_dma_in_cfg(device, subdev, frame, out_node, dma,
				PARAM_ORB_DMA, pmap);

		for (i = 0; i < CAPTURE_NODE_MAX; i++) {
			cap_node = &frame->shot_ext->node_group.capture[i];
			if (!cap_node->vid) {
				if (i == 0) {
					/* ORBC capture node is zero */
					dma->output_cmd = DMA_OUTPUT_COMMAND_DISABLE;

					set_bit(PARAM_ORB_DMA, pmap);
				}
				continue;
			}

			ret = __orb_dma_out_cfg(device, subdev, frame, cap_node,
					dma, PARAM_ORB_DMA, pmap);
			if (ret) {
				mrerr("__orb_dma_out_cfg is fail(%d)", device, frame, ret);
				goto p_err;
			}
		}

		/* buffer tagging */
#ifdef ENABLE_LVN_DUMMYOUTPUT
		for (i = 0; i < frame->out_node.sframe[0].num_planes; i++) {
			if (!frame->out_node.sframe[0].id) {
				mrerr("Invalid video id(%d)", device, frame,
						frame->out_node.sframe[0].id);
				return -EINVAL;
			}

			frame->dvaddr_buffer[i] = frame->out_node.sframe[0].dva[i];
			frame->kvaddr_buffer[i] = frame->out_node.sframe[0].kva[i];
		}
#endif
		for (i = 0; i < CAPTURE_NODE_MAX; i++) {
			sframe = &frame->cap_node.sframe[i];
			if (!sframe->id)
				continue;

			/* TODO: consider Sensor and PDP */
			if ((sframe->id >= IS_VIDEO_SS0VC0_NUM) &&
					(sframe->id <= IS_VIDEO_PAF3S_NUM))
				continue;

			targetAddr = NULL;
			targetAddr_k = NULL;
			ret = is_hw_get_capture_slot(frame, &targetAddr,
					&targetAddr_k, sframe->id);
			if (ret) {
				mrerr("Invalid capture slot(%d)", device, frame,
						sframe->id);
				return -EINVAL;
			}

			num_planes = sframe->num_planes / frame->num_shots;
			if (targetAddr) {
				for (j = 0, p = num_planes * frame->cur_shot_idx;
						j < num_planes; j++, p++)
					targetAddr[j] = sframe->dva[p];
			}

			if (targetAddr_k) {
				/*
				 * IS_VIDEO_ORB0C_NUM
				 * IS_VIDEO_ORB0M_NUM
				 */
				for (j = 0, p = num_planes * frame->cur_shot_idx;
						j < num_planes; j++, p++) {
					targetAddr_k[j] = sframe->kva[p];
				}
			}
		}

		ret = is_itf_s_param(device, frame, pmap);
		if (ret) {
			mrerr("is_itf_s_param is fail(%d)", device, frame, ret);
			goto p_err;
		}

		return ret;
	}

p_skip:
	inparm.x = 0;
	inparm.y = 0;
	inparm.w = orbmch_param->dma.cur_input_width;
	inparm.h = orbmch_param->dma.cur_input_height;

	if (IS_NULL_CROP(incrop))
		*incrop = inparm;

	if (!COMPARE_CROP(incrop, &inparm) ||
		test_bit(IS_SUBDEV_FORCE_SET, &leader->state)) {
		ret = is_ischain_orb_cfg(subdev,
			device,
			frame,
			incrop,
			otcrop,
			pmap);
		if (ret) {
			merr("is_ischain_orb_cfg is fail(%d)", device, ret);
			goto p_err;
		}
		if (!COMPARE_CROP(incrop, &subdev->input.crop) ||
			is_get_debug_param(IS_DEBUG_PARAM_STREAM)) {
			msrinfo("in_crop[%d, %d, %d, %d]\n", device, subdev, frame,
				incrop->x, incrop->y, incrop->w, incrop->h);

			subdev->input.crop = *incrop;
		}
	}

	ret = is_itf_s_param(device, frame, pmap);
	if (ret) {
		mrerr("is_itf_s_param is fail(%d)", device, frame, ret);
		goto p_err;
	}

p_err:
	return ret;
}

const struct is_subdev_ops is_subdev_orb_ops = {
	.bypass			= NULL,
	.cfg			= is_ischain_orb_cfg,
	.tag			= is_ischain_orb_tag,
};
