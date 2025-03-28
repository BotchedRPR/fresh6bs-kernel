/*
 * Samsung EXYNOS FIMC-IS (Imaging Subsystem) driver
 *
 * Copyright (C) 2014 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>

#include "videodev2_exynos_camera.h"
#include "pablo-hw-helper.h"
#include "is-err.h"
#include "is-core.h"
#include "is-hw-control.h"
#include "is-hw-dm.h"
#include "pablo-work.h"
#include "is-hw-ip.h"
#include "pablo-obte.h"
#include "pablo-fpsimd.h"
#include "is-interface-ddk.h"
#include "pablo-mmio.h"
#include "is-votfmgr.h"
#include "pablo-icpu-adapter.h"
#include "is-votf-id-table.h"

#define INTERNAL_SHOT_EXIST	(1)

static ulong debug_time_hw;
module_param(debug_time_hw, ulong, 0644);

static int is_hardware_frame_ndone(struct is_hw_ip *ldr_hw_ip,
	struct is_frame *frame, u32 instance,
	enum ShotErrorType done_type);

static inline void wq_func_schedule(struct is_interface *itf,
	struct work_struct *work_wq)
{
	if (itf->workqueue)
		queue_work(itf->workqueue, work_wq);
	else
		schedule_work(work_wq);
}

static void prepare_sfr_dump(struct is_hardware *hardware)
{
	int hw_slot = -1;
	int reg_size = 0;
	struct is_hw_ip *hw_ip = NULL;
	int i;

	if (!hardware) {
		err_hw("hardware is null\n");
		return;
	}

	for (hw_slot = 0; hw_slot < HW_SLOT_MAX; hw_slot++) {
		hw_ip = &hardware->hw_ip[hw_slot];

		if (hw_ip->id == DEV_HW_END || hw_ip->id == 0)
		       continue;

		hw_ip->sfr_dump_flag = false;

		for (i = 0; i < REG_SET_MAX; i++) {
			if (IS_ERR_OR_NULL(hw_ip->regs[i]) ||
				(hw_ip->regs_start[i] == 0) ||
				(hw_ip->regs_end[i] == 0))
				continue;

			reg_size = (hw_ip->regs_end[i] - hw_ip->regs_start[i] + 1);
			hw_ip->sfr_dump[i] = kzalloc(reg_size, GFP_KERNEL);
			if (IS_ERR_OR_NULL(hw_ip->sfr_dump[i]))
				serr_hw("sfr %d dump memory alloc fail", hw_ip, i);
			else
				sinfo_hw("sfr %d dump memory (V/P/S):(%lx/%lx/0x%X)[0x%llX~0x%llX]\n", hw_ip,
					i, (ulong)hw_ip->sfr_dump[i], (ulong)virt_to_phys(hw_ip->sfr_dump[i]),
					reg_size, hw_ip->regs_start[i], hw_ip->regs_end[i]);
		}
	}
}

static void _is_hardware_sfr_dump(struct is_hw_ip *hw_ip, bool flag_print_log)
{
	enum base_reg_index i;
	int j;
	void *sfr_dump;
	void *src, *dst;
	resource_size_t total_size, split_size;
	resource_size_t offset_start, offset_end;

	if (!test_bit(HW_OPEN, &hw_ip->state)) {
		swarn_hw("IP is not opend", hw_ip);
		return;
	}

	for (i = 0; i < REG_SET_MAX; i++) {
		sfr_dump = hw_ip->sfr_dump[i];

		if (IS_ERR_OR_NULL(sfr_dump))
			continue;

		total_size = (hw_ip->regs_end[i] - hw_ip->regs_start[i] + 1);

		if (hw_ip->sfr_dump_flag) {
			sinfo_hw("alreay done: SFR %d DUMP(V/P/S):(%lx/%lx/0x%llX)[0x%llX~0x%llX]\n", hw_ip,
					i, (ulong)sfr_dump, (ulong)virt_to_phys(sfr_dump),
					total_size, hw_ip->regs_start[i], hw_ip->regs_end[i]);
			if (flag_print_log) {
				print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4,
						sfr_dump, total_size, false);
			}
			continue;
		}

		/* dump reg */
		if (!hw_ip->dump_reg_list_size) {
			for (j = 0; j < DUMP_SPLIT_MAX ; j++) {
				offset_start = hw_ip->dump_region[i][j].start;
				offset_end = hw_ip->dump_region[i][j].end;

				if (j > 0 && (offset_start == 0 || offset_end == 0))
					break;

				src = hw_ip->regs[i] + offset_start;
				dst = sfr_dump + offset_start;
				split_size = offset_end ? offset_end - offset_start + 1 : total_size;

				memcpy(dst, src, split_size);
				sinfo_hw("##### SFR %d-%d DUMP(V/P/S):(%lx/%lx/0x%llX)[0x%llX~0x%llX]\n", hw_ip,
					i, j, (ulong)dst, (ulong)virt_to_phys(dst),
					split_size,
					hw_ip->regs_start[i] + offset_start,
					hw_ip->regs_start[i] + offset_start + split_size - 1);
#ifdef ENABLE_PANIC_SFR_PRINT
				print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4,
					dst, split_size, false);
#else
				if (flag_print_log)
					print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4,
						dst, split_size, false);
#endif

				if (offset_end == 0 || offset_end + 1 >= total_size)
					break;
			}
		} else {
			offset_start = hw_ip->dump_region[i][0].start;
			offset_end = hw_ip->dump_region[i][0].end;

			src = hw_ip->regs[i] + offset_start;
			dst = sfr_dump + offset_start;
			split_size = offset_end ? offset_end - offset_start + 1 : total_size;

			for (j = 0; j < hw_ip->dump_reg_list_size; j++)
				*(u32 *)(dst + hw_ip->dump_for_each_reg[j].sfr_offset) =
					readl(src + hw_ip->dump_for_each_reg[j].sfr_offset);

			sinfo_hw("##### SFR %d-0 DUMP(V/P/S):(%lx/%lx/0x%llX)[0x%llX~0x%llX]\n", hw_ip,
				i, (ulong)dst, (ulong)virt_to_phys(dst),
				split_size,
				hw_ip->regs_start[i] + offset_start,
				hw_ip->regs_start[i] + offset_start + split_size - 1);
#ifdef ENABLE_PANIC_SFR_PRINT
			print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4,
				dst, split_size, false);
#else
			if (flag_print_log)
				print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4,
					dst, split_size, false);
#endif
		}
		is_clean_dcache_area(sfr_dump, total_size);
	}

	hw_ip->sfr_dump_flag = true;
}

static inline void _is_hw_print_debug_trace(struct is_hw_ip *hw_ip, u32 index, u32 dbg_e, u32 dbg_x, const char *name)
{
	u32 instance = atomic_read(&hw_ip->instance);

	if (unlikely(test_bit(hw_ip->id, (unsigned long *)&debug_time_hw))) {
		msinfo_hw("TIME %s F%d: %05llu us\n", instance, hw_ip,
			   name, hw_ip->debug_info[index].fcount,
			   (hw_ip->debug_info[index].time[dbg_x] -
			   hw_ip->debug_info[index].time[dbg_e]) / 1000);
	}
}

void _is_hw_frame_dbg_trace(struct is_hw_ip *hw_ip, u32 fcount, u32 dbg_pts)
{
	u32 index, instance, debug_e;
	char name[20] = { 0, };

	FIMC_BUG_VOID(!hw_ip);

	debug_e = DEBUG_POINT_MAX;

	switch (dbg_pts) {
	case DEBUG_POINT_HW_SHOT_E:
		index = fcount % DEBUG_FRAME_COUNT;
		instance = atomic_read(&hw_ip->instance);
		hw_ip->debug_index[0] = fcount;
		hw_ip->debug_info[index].fcount = fcount;
		hw_ip->debug_info[index].instance = hw_ip->group[instance]->device->instance;
		break;
	case DEBUG_POINT_HW_SHOT_X:
		index = fcount % DEBUG_FRAME_COUNT;
		debug_e = DEBUG_POINT_HW_SHOT_E;
		snprintf(name, sizeof(name), "HW_SHOT");
		break;
	case DEBUG_POINT_FRAME_START:
		index = hw_ip->debug_index[1];
		hw_ip->debug_index[1] = hw_ip->debug_index[0] % DEBUG_FRAME_COUNT;
		break;
	case DEBUG_POINT_FRAME_END:
		index = hw_ip->debug_index[1];
		debug_e = DEBUG_POINT_FRAME_START;
		snprintf(name, sizeof(name), "HW");
		break;
	case DEBUG_POINT_LIB_SHOT_E:
	case DEBUG_POINT_RTA_REGS_E:
	case DEBUG_POINT_ADD_TO_CMDQ:
	case DEBUG_POINT_CONFIG_LOCK_E:
	case DEBUG_POINT_SHOT_MSG:
		index = fcount % DEBUG_FRAME_COUNT;
		break;
	case DEBUG_POINT_LIB_SHOT_X:
		index = fcount % DEBUG_FRAME_COUNT;
		debug_e = DEBUG_POINT_LIB_SHOT_E;
		snprintf(name, sizeof(name), "LIB_SHOT");
		break;
	case DEBUG_POINT_RTA_REGS_X:
		index = fcount % DEBUG_FRAME_COUNT;
		debug_e = DEBUG_POINT_RTA_REGS_E;
		snprintf(name, sizeof(name), "RTA_REGS");
		break;
	case DEBUG_POINT_SETTING_DONE:
		index = fcount % DEBUG_FRAME_COUNT;
		debug_e = DEBUG_POINT_ADD_TO_CMDQ;
		snprintf(name, sizeof(name), "PRE_CONFIG");
		break;
	case DEBUG_POINT_CONFIG_LOCK_X:
		index = fcount % DEBUG_FRAME_COUNT;
		debug_e = DEBUG_POINT_CONFIG_LOCK_E;
		snprintf(name, sizeof(name), "CONFIG_LOCK");
		break;
	case DEBUG_POINT_SHOT_CALLBACK:
		index = fcount % DEBUG_FRAME_COUNT;
		debug_e = DEBUG_POINT_SHOT_MSG;
		snprintf(name, sizeof(name), "SHOT~CALLBACK");
		break;
	default:
		merr_hw("Invalid event (%d)\n",
			atomic_read(&hw_ip->instance), dbg_pts);
		return;
	}

	hw_ip->debug_info[index].cpuid[dbg_pts] = raw_smp_processor_id();
	hw_ip->debug_info[index].time[dbg_pts] = local_clock();
	if (debug_e != DEBUG_POINT_MAX)
		_is_hw_print_debug_trace(hw_ip, index, debug_e, dbg_pts, name);
}

void _is_hw_frame_dbg_ext_trace(struct is_hw_ip *hw_ip, u32 fcount, u32 dbg_pts, u32 ext_id)
{
	FIMC_BUG_VOID(!hw_ip);

	switch (dbg_pts) {
	case DEBUG_POINT_FRAME_START:
		hw_ip->debug_ext_info[ext_id].fcount = fcount;
		hw_ip->debug_ext_info[ext_id].cpuid[dbg_pts] = raw_smp_processor_id();
		hw_ip->debug_ext_info[ext_id].time[dbg_pts] = cpu_clock(raw_smp_processor_id());
		break;
	case DEBUG_POINT_FRAME_END:
		hw_ip->debug_ext_info[ext_id].cpuid[dbg_pts] = raw_smp_processor_id();
		hw_ip->debug_ext_info[ext_id].time[dbg_pts] = cpu_clock(raw_smp_processor_id());
		break;
	default:
		merr_hw("Invalid event (%d)\n",
			atomic_read(&hw_ip->instance), dbg_pts);
		break;
	}

}

void print_hw_frame_count(struct is_hw_ip *hw_ip)
{
	int f_index, p_index, ext_index;
	struct hw_debug_info *debug_info;
	struct hw_debug_info *debug_ext_info;
	unsigned long long time[DEBUG_POINT_MAX];
	ulong usec[DEBUG_POINT_MAX];
	u32 instance;
	struct is_group *group;
	struct is_device_sensor *sensor;
	struct is_device_csi *csi;

	if (!hw_ip) {
		err_hw("hw_ip is null\n");
		return;
	}

	/* skip printing frame count, if hw_ip wasn't opened */
	if (!test_bit(HW_OPEN, &hw_ip->state))
		return;

	/* csis interrupt debug */
	instance = atomic_read(&hw_ip->instance);
	group = hw_ip->group[instance];

	if (!group)
		goto exit;

	if (group->prev && group->prev->device_type == IS_DEVICE_SENSOR) {
		if (!group->device) {
			err_hw("device is NULL");
			goto exit;
		}
		sensor = group->device->sensor;
		if (!sensor) {
			err_hw("sensor is NULL");
			goto exit;
		}

		csi = v4l2_get_subdevdata(sensor->subdev_csi);
		if (!csi) {
			err_hw("CSI is NULL");
			goto exit;
		}

		info("[HW:CSI%d]\n", csi->ch);
		for (f_index = 0; f_index < DEBUG_FRAME_COUNT; f_index++) {
			debug_info = &csi->debug_info[f_index];
			for (p_index = 0 ; p_index < DEBUG_POINT_MAX; p_index++) {
				time[p_index]  = debug_info->time[p_index];
				usec[p_index]  = do_div(time[p_index], NSEC_PER_SEC);
			}

			info("[%d][F:%d] shot[%5lu.%06lu], fs[c%d][%5lu.%06lu], fe[c%d][%5lu.%06lu], dma[c%d][%5lu.%06lu]\n",
				f_index, debug_info->fcount,
				(ulong)time[DEBUG_POINT_HW_SHOT_E], usec[DEBUG_POINT_HW_SHOT_E] / NSEC_PER_USEC,
				debug_info->cpuid[DEBUG_POINT_FRAME_START],
				(ulong)time[DEBUG_POINT_FRAME_START], usec[DEBUG_POINT_FRAME_START] / NSEC_PER_USEC,
				debug_info->cpuid[DEBUG_POINT_FRAME_END],
				(ulong)time[DEBUG_POINT_FRAME_END], usec[DEBUG_POINT_FRAME_END] / NSEC_PER_USEC,
				debug_info->cpuid[DEBUG_POINT_FRAME_DMA_END],
				(ulong)time[DEBUG_POINT_FRAME_DMA_END], usec[DEBUG_POINT_FRAME_DMA_END] / NSEC_PER_USEC);
		}
	}

exit:
	sinfo_hw("fs(%d), cl(%d), fe(%d), dma(%d)\n", hw_ip,
			atomic_read(&hw_ip->count.fs),
			atomic_read(&hw_ip->count.cl),
			atomic_read(&hw_ip->count.fe),
			atomic_read(&hw_ip->count.dma));

	for (f_index = 0; f_index < DEBUG_FRAME_COUNT; f_index++) {
		debug_info = &hw_ip->debug_info[f_index];
		for (p_index = 0 ; p_index < DEBUG_POINT_MAX; p_index++) {
			time[p_index]  = debug_info->time[p_index];
			usec[p_index]  = do_div(time[p_index], NSEC_PER_SEC);
		}

		info_hw("[%d][F:%d] shot[%5lu.%06lu], fs[c%d][%5lu.%06lu], fe[c%d][%5lu.%06lu], dma[c%d][%5lu.%06lu] (%d) \n",
				f_index, debug_info->fcount,
				(ulong)time[DEBUG_POINT_HW_SHOT_E], usec[DEBUG_POINT_HW_SHOT_E] / NSEC_PER_USEC,
				debug_info->cpuid[DEBUG_POINT_FRAME_START],
				(ulong)time[DEBUG_POINT_FRAME_START], usec[DEBUG_POINT_FRAME_START] / NSEC_PER_USEC,
				debug_info->cpuid[DEBUG_POINT_FRAME_END],
				(ulong)time[DEBUG_POINT_FRAME_END], usec[DEBUG_POINT_FRAME_END] / NSEC_PER_USEC,
				debug_info->cpuid[DEBUG_POINT_FRAME_DMA_END],
				(ulong)time[DEBUG_POINT_FRAME_DMA_END], usec[DEBUG_POINT_FRAME_DMA_END] / NSEC_PER_USEC,
				debug_info->instance);
	}

	for (ext_index = 0; ext_index < DEBUG_EXT_MAX; ext_index++) {
		debug_ext_info =  &hw_ip->debug_ext_info[ext_index];
		if (debug_ext_info->time[DEBUG_POINT_FRAME_START]) {
			for (p_index = DEBUG_POINT_FRAME_START; p_index <= DEBUG_POINT_FRAME_END; p_index++) {
				time[p_index]  = debug_ext_info->time[p_index];
				usec[p_index]  = do_div(time[p_index], NSEC_PER_SEC);
			}
			info_hw("[EXT%d][F:%d] fs[c%d][%5lu.%06lu], fe[c%d][%5lu.%06lu]\n",
				ext_index, debug_ext_info->fcount,
				debug_ext_info->cpuid[DEBUG_POINT_FRAME_START],
				(ulong)time[0], usec[DEBUG_POINT_FRAME_START] / NSEC_PER_USEC,
				debug_ext_info->cpuid[DEBUG_POINT_FRAME_END],
				(ulong)time[DEBUG_POINT_FRAME_END], usec[DEBUG_POINT_FRAME_END] / NSEC_PER_USEC);
		}
	}
}

void print_all_hw_frame_count(struct is_hardware *hardware)
{
	int hw_slot = -1;
	struct is_hw_ip *_hw_ip = NULL;

	if (!hardware) {
		err_hw("hardware is null\n");
		return;
	}

	for (hw_slot = 0; hw_slot < HW_SLOT_MAX; hw_slot++) {
		_hw_ip = &hardware->hw_ip[hw_slot];
		print_hw_frame_count(_hw_ip);
	}
}

static void is_hardware_flush_frame(struct is_hw_ip *hw_ip,
	enum is_frame_state state,
	enum ShotErrorType done_type)
{
	int ret = 0;
	struct is_framemgr *framemgr;
	struct is_frame *frame;
	ulong flags = 0;
	int retry;

	FIMC_BUG_VOID(!hw_ip);

	framemgr = hw_ip->framemgr;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	while (state <  FS_HW_WAIT_DONE) {
		frame = peek_frame(framemgr, state);
		while (frame) {
			trans_frame(framemgr, frame, state + 1);
			frame = peek_frame(framemgr, state);
		}
		state++;
	}
	frame = peek_frame(framemgr, FS_HW_WAIT_DONE);
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	retry = IS_MAX_HW_FRAME;
	while (frame && retry--) {
		if (done_type == IS_SHOT_TIMEOUT)
			mserr_hw("[F:%d]hardware is timeout", frame->instance, hw_ip, frame->fcount);

		ret = is_hardware_frame_ndone(hw_ip, frame, atomic_read(&hw_ip->instance), done_type);
		if (ret)
			mserr_hw("%s: hardware_frame_ndone fail",
				atomic_read(&hw_ip->instance), hw_ip, __func__);

		framemgr_e_barrier_irqs(framemgr, 0, flags);
		frame = peek_frame(framemgr, FS_HW_WAIT_DONE);
		framemgr_x_barrier_irqr(framemgr, 0, flags);
	}

	if (retry == 0)
		err_hw("frame flush is not completed");
}

static inline void is_hardware_fill_frame_info(u32 instance,
	struct is_frame *hw_frame,
	struct is_frame *frame,
	struct is_group *group,
	struct is_hardware *hardware,
	bool reset)
{
	struct is_group *child;
	struct is_hw_ip *hw_ip;

	hw_frame->groupmgr	= frame->groupmgr;
	hw_frame->group		= frame->group;
	hw_frame->shot_ext	= frame->shot_ext;
	hw_frame->shot		= frame->shot;
	hw_frame->shot_dva	= frame->shot_dva;
	hw_frame->shot_size	= frame->shot_size;
	hw_frame->cur_shot_idx	= frame->cur_shot_idx;
	hw_frame->fcount	= frame->fcount;
	hw_frame->rcount	= frame->rcount;
	hw_frame->bak_flag      = GET_OUT_FLAG_IN_DEVICE(IS_DEVICE_ISCHAIN, frame->bak_flag);
	hw_frame->out_flag      = GET_OUT_FLAG_IN_DEVICE(IS_DEVICE_ISCHAIN, frame->out_flag);
	hw_frame->core_flag	= 0;
	hw_frame->result	= 0;
	atomic_set(&hw_frame->shot_done_flag, 1);
	hw_frame->parameter	= frame->parameter;
	IS_COPY_PMAP(hw_frame->pmap, frame->pmap);
	memcpy(hw_frame->dvaddr_buffer, frame->dvaddr_buffer, sizeof(frame->dvaddr_buffer));
	memcpy(hw_frame->kvaddr_buffer, frame->kvaddr_buffer, sizeof(frame->kvaddr_buffer));
	memcpy(&hw_frame->stripe_info, &frame->stripe_info, sizeof(frame->stripe_info));

	child = group;
	while (child) {
		hw_ip = is_get_hw_ip(child->id, hardware);
		if (!hw_ip) {
			mgwarn("invalid hw_ip", child, child);
			continue;
		}

		is_hw_fill_target_address(hw_ip->id, hw_frame, frame, reset);
		child = child->child;
	}

	hw_frame->instance = instance;
}

static inline void mshot_schedule(struct is_hw_ip *hw_ip)
{
#if defined(MULTI_SHOT_TASKLET)
	tasklet_schedule(&hw_ip->tasklet_mshot);
#elif defined(MULTI_SHOT_KTHREAD)
	kthread_queue_work(&hw_ip->mshot_worker, &hw_ip->mshot_work);
#endif
}

#if defined(MULTI_SHOT_TASKLET)
static void tasklet_mshot(unsigned long data)
{
	u32 instance;
	int ret = 0;
	ulong hw_map;
	ulong flags = 0;
	struct is_hw_ip *hw_ip;
	struct is_hardware *hardware;
	struct is_frame *frame;
	struct is_framemgr *framemgr;
	struct is_group *group;

	hw_ip = (struct is_hw_ip *)data;
	if (!hw_ip) {
		err("hw_ip is NULL");
		BUG();
	}

	hardware = hw_ip->hardware;
	instance = atomic_read(&hw_ip->instance);
	group = hw_ip->group[instance];
	framemgr = hw_ip->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_HW_REQUEST);
	framemgr_x_barrier_irqr(framemgr, 0, flags);
	hw_map = hardware->hw_map[instance];

	if (!frame) {
		serr_hw("shot frame is empty", hw_ip);
		return;
	}

	msdbgs_hw(2, "[F:%d]%s\n", instance, hw_ip, frame->fcount, __func__);

	mod_timer(&hw_ip->shot_timer, jiffies + msecs_to_jiffies(is_get_shot_timeout()));

	is_fpsimd_get_func();
	ret = is_hardware_shot(hardware, instance, group, frame, framemgr,
			hw_map, frame->fcount);
	is_fpsimd_put_func();
	if (ret)
		mserr_hw("hardware_shot fail", instance, hw_ip);
}
#elif defined(MULTI_SHOT_KTHREAD)
void is_hardware_mshot_work_fn(struct kthread_work *work)
{
	u32 instance;
	int ret = 0;
	ulong hw_map;
	ulong flags = 0;
	struct is_hw_ip *hw_ip;
	struct is_hardware *hardware;
	struct is_frame *frame;
	struct is_framemgr *framemgr;
	struct is_group *group;

	hw_ip = container_of(work, struct is_hw_ip, mshot_work);

	hardware = hw_ip->hardware;
	instance = atomic_read(&hw_ip->instance);
	group = hw_ip->group[instance];
	framemgr = hw_ip->framemgr;
	framemgr_e_barrier_irqs(framemgr, 0, flags);
	frame = peek_frame(framemgr, FS_HW_REQUEST);
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	hw_map = hardware->hw_map[instance];

	if (!frame) {
		serr_hw("frame is empty", hw_ip);
	}

	msdbgs_hw(2, "[F:%d]%s\n", instance, hw_ip, frame->fcount, __func__);

	ret = is_hardware_shot(hardware, instance, group, frame, framemgr,
			hw_map, frame->fcount);
	if (ret)
		mserr_hw("hardware_shot fail", instance, hw_ip);
}

static int is_hardware_init_mshot_thread(struct is_hw_ip *hw_ip)
{
	int ret = 0;
	struct sched_param param = {.sched_priority = TASK_MSHOT_WORK_PRIO};

	if (hw_ip->mshot_task == NULL) {
		kthread_init_worker(&hw_ip->mshot_worker);
		hw_ip->mshot_task = kthread_run(kthread_worker_fn,
						&hw_ip->mshot_worker,
						"mshot_work");
		if (IS_ERR_OR_NULL(hw_ip->mshot_task)) {
			err("failed to create kthread");
			hw_ip->mshot_task = NULL;
			return -EFAULT;
		}

		ret = sched_setscheduler_nocheck(hw_ip->mshot_task, SCHED_FIFO, &param);
		if (ret) {
			err("sched_setscheduler_nocheck is fail(%d)", ret);
			return ret;
		}

		kthread_init_work(&hw_ip->mshot_work, is_hardware_mshot_work_fn);
	}

	return ret;
}

void is_hardware_deinit_mshot_thread(struct is_hw_ip *hw_ip)
{
	if (hw_ip->mshot_task != NULL) {
		if (kthread_stop(hw_ip->mshot_task))
			err("kthread_stop fail");

		hw_ip->mshot_task = NULL;
	}
}
#endif

int is_hardware_set_param(struct is_hardware *hardware, u32 instance,
	struct is_region *region, IS_DECLARE_PMAP(pmap), ulong hw_map)
{
	int ret = 0;
	int hw_slot = -1;
	struct is_hw_ip *hw_ip = NULL;

	FIMC_BUG(!hardware);
	FIMC_BUG(!region);

	for (hw_slot = 0; hw_slot < HW_SLOT_MAX; hw_slot++) {
		hw_ip = &hardware->hw_ip[hw_slot];

		if (!test_bit_variables(hw_ip->id, &hw_map))
			continue;

		ret = CALL_HWIP_OPS(hw_ip, set_param, region, pmap, instance, hw_map);
		if (ret) {
			mserr_hw("set_param fail (%d)", instance, hw_ip, hw_slot);
			return -EINVAL;
		}
	}

	msdbg_hw(1, "set_param hw_map[0x%lx]\n", instance, hw_ip, hw_map);

	return ret;
}

IS_TIMER_FUNC(is_hardware_shot_timer)
{
	struct is_hw_ip *hw_ip = from_timer(hw_ip, (struct timer_list *)data, shot_timer);
	u32 instance;
	struct is_group *group, *child;
	struct is_hw_ip *hw_ip_chd;
	struct is_hardware *hardware;
	int hw_list[GROUP_HW_MAX], hw_maxnum, hw_index;
	int f_index;
	struct hw_debug_info *debug_info;
	unsigned long long start, end, otf_end, dma_end, shot;
	bool timeout = false;

	instance = atomic_read(&hw_ip->instance);
	group = hw_ip->group[instance];
	child = group->tail;

	hardware = hw_ip->hardware;

	while (child && (child->device_type == IS_DEVICE_ISCHAIN)) {
		hw_maxnum = is_get_hw_list(child->id, hw_list);
		for (hw_index = hw_maxnum - 1; hw_index >= 0; hw_index--) {
			hw_ip_chd = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
			if (!hw_ip_chd) {
				merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
				timeout = true;
				goto flush_frame;
			}

			for (f_index = 0; f_index < DEBUG_FRAME_COUNT; f_index++) {
				debug_info = &hw_ip_chd->debug_info[f_index];

				shot = debug_info->time[DEBUG_POINT_HW_SHOT_E];
				start = debug_info->time[DEBUG_POINT_FRAME_START];
				otf_end = debug_info->time[DEBUG_POINT_FRAME_END];
				dma_end = debug_info->time[DEBUG_POINT_FRAME_DMA_END];
				end = max(otf_end, dma_end);

				if (time_after(jiffies, hw_ip->shot_timer.expires)) {
					if (start > end) {
						msinfo_hw("[F:%d] timeout: start_time > end_time", instance, hw_ip_chd,
								debug_info->fcount);
						timeout = true;
					} else if (shot > start) {
						msinfo_hw("[F:%d] timeout: shot_time > start_time", instance, hw_ip_chd,
								debug_info->fcount);
						timeout = true;
					}
				}
			}
		}

		child = child->parent;
	}

flush_frame:

	if (timeout) {
		print_all_hw_frame_count(hw_ip->hardware);
		is_hardware_flush_frame(hw_ip, FS_HW_REQUEST, IS_SHOT_TIMEOUT);
	} else {
		msinfo_hw(" false alarm timeout", instance, hw_ip);
		print_all_hw_frame_count(hw_ip->hardware);
	}
}

static void is_hardware_suspend(struct is_hardware *hardware,
		struct is_group *group)
{
	struct is_hw_ip *hw_ip;
	int hw_list[GROUP_HW_MAX], hw_index, hw_num;
	u32 instance;

	while (group) {
		hw_num = is_get_hw_list(group->id, hw_list);
		instance = group->instance;

		for (hw_index = 0; hw_index < hw_num; hw_index++) {
			hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
			if (!hw_ip) {
				merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
				continue;
			}

			CALL_HWIP_OPS(hw_ip, suspend, instance);
		}

		group = group->child;
	}
}

static int _is_hardware_shot(struct is_hardware *hardware,
	struct is_group *group, struct is_frame *frame,
	struct is_framemgr *framemgr, u32 framenum)
{
	int ret = 0;
	struct is_hw_ip *hw_ip, *ldr_hw_ip;
	struct is_group *child = NULL;
	struct is_group *head = NULL;
	ulong flags = 0;
	int hw_list[GROUP_HW_MAX], hw_index;
	int hw_maxnum = 0;
	ulong hw_map;
	u32 instance;
	struct pablo_crta_bufmgr *crta_bufmgr;
	struct pablo_crta_buf_info pcfi_buf = { 0, };

	head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);

	ldr_hw_ip = is_get_hw_ip(head->id, hardware);
	if (!ldr_hw_ip) {
		mgerr("invalid ldr_hw_ip", head, head);
		return -EINVAL;
	}

	instance = group->instance;

	if (is_hw_check_crta_group(head->id)) {
		crta_bufmgr = hardware->crta_bufmgr[instance];
		CALL_CRTA_BUFMGR_OPS(crta_bufmgr, get_process_buf,
			PABLO_CRTA_BUF_PCFI, framenum, &pcfi_buf);
	}

	child = group->tail;
	while (child && (child->device_type == IS_DEVICE_ISCHAIN)) {
		hw_maxnum = is_get_hw_list(child->id, hw_list);

		for (hw_index = hw_maxnum - 1; hw_index >= 0; hw_index--) {
			hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
			if (!hw_ip) {
				merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
				ret = -EINVAL;
				goto shot_err_cancel;
			}

			/* hw_ip->fcount : frame number of current frame in Vvalid  @ OTF *
			 * hw_ip->fcount is the frame number of next FRAME END interrupt  *
			 * In OTF scenario, hw_ip->fcount is not same as frame->fcount    */
			atomic_set(&hw_ip->fcount, framenum);
			atomic_set(&hw_ip->instance, instance);

			hw_ip->framemgr = &hardware->framemgr[ldr_hw_ip->id];

			hw_map = hardware->hw_map[instance];
			_is_hw_frame_dbg_trace(hw_ip, frame->fcount, DEBUG_POINT_HW_SHOT_E);
			if (pcfi_buf.kva)
				is_hw_config(hw_ip, &pcfi_buf);
			ret = CALL_HWIP_OPS(hw_ip, shot, frame, hw_map);
			_is_hw_frame_dbg_trace(hw_ip, frame->fcount, DEBUG_POINT_HW_SHOT_X);
			if (ret) {
				mserr_hw("shot fail [F:%d]", instance, hw_ip,
					frame->fcount);
				goto shot_err_cancel;
			}
		}
		child = child->parent;
	}

	if (pcfi_buf.kva)
		CALL_CRTA_BUFMGR_OPS(crta_bufmgr, put_buf, &pcfi_buf);

	return ret;

shot_err_cancel:
	mswarn_hw("[F:%d] Canceled by hardware shot err", instance, ldr_hw_ip, frame->fcount);

	framemgr_e_barrier_common(framemgr, 0, flags);
	trans_frame(framemgr, frame, FS_HW_FREE);
	framemgr_x_barrier_common(framemgr, 0, flags);

	if (child && child->tail) {
		struct is_group *restore_grp = child->tail;

		while (restore_grp && (restore_grp->id != child->id)) {
			is_hardware_restore_by_group(hardware, restore_grp, restore_grp->instance);
			restore_grp = restore_grp->parent;
		}
	}

	return ret;
}

static int _is_hardware_shot_prepare(struct is_hardware *hardware,
	struct is_group *group, struct is_frame *frame)
{
	int ret;
	u32 instance, fcount;
	struct pablo_icpu_adt *icpu_adt;
	struct camera2_shot *shot;
	struct pablo_crta_bufmgr *crta_bufmgr;
	struct pablo_crta_buf_info shot_buf = { 0, }, pcfi_buf = { 0, };
	struct is_group *head;
	struct is_hw_ip *head_hw_ip;

	icpu_adt = hardware->icpu_adt;
	shot = frame->shot;
	instance = group->instance;
	fcount = frame->fcount;

	crta_bufmgr = hardware->crta_bufmgr[instance];
	ret = CALL_CRTA_BUFMGR_OPS(crta_bufmgr, get_free_buf,
				PABLO_CRTA_BUF_PCFI, fcount, true, &pcfi_buf);

	if (!ret)
		is_hw_update_pcfi(hardware, group, frame, &pcfi_buf);
	else
		merr_adt("[F%d][PCFI]failed to get_free_buf", instance, fcount);

	/* get shot va */
	shot_buf.kva = shot;
	shot_buf.dva = frame->shot_dva;

	/* group is required in shot_callback even if it is a internal shot */
	if (!frame->group)
		frame->group = group;

	CALL_ADT_MSG_OPS(icpu_adt, send_msg_shot, instance,
			 (void *)hardware, (void *)frame,
			 fcount, &shot_buf, &pcfi_buf);

	head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);
	head_hw_ip = is_get_hw_ip(head->id, hardware);
	if (head_hw_ip)
		_is_hw_frame_dbg_trace(head_hw_ip, fcount, DEBUG_POINT_SHOT_MSG);

	return 0;
}

static int _is_hardware_shot_callback(void *user, void *ctx, void *rsp_msg)
{
	int ret;
	u32 framenum, instance;
	bool is_ss_grp;
	struct pablo_icpu_adt_rsp_msg *msg;
	struct is_hardware *hardware;
	struct is_hw_ip *head_hw_ip;
	struct is_frame *frame;
	struct is_group *group, *head;
	struct is_framemgr *framemgr;
	struct is_device_sensor *sensor;

	if (!user || !ctx || !rsp_msg) {
		err_hw("invalid callback: user(%p), ctx(%p), msg(%p)",
			user, ctx, rsp_msg);
		return -EINVAL;
	}

	msg = (struct pablo_icpu_adt_rsp_msg *)rsp_msg;
	hardware = (struct is_hardware *)user;
	frame = (struct is_frame *)ctx;

	if (msg->rsp)
		merr_hw("shot fail from icpu: msg_ret(%d)", msg->instance, msg->rsp);

	group = frame->group;
	if (!group) {
		merr_hw("[F%d]group is null", msg->instance, msg->fcount);
		return -EINVAL;
	}

	instance = group->instance;
	framenum = frame->fcount;
	framemgr = &hardware->framemgr[group->id];
	sensor = group->device->sensor;
	is_ss_grp = test_bit(IS_GROUP_OTF_INPUT, &group->state);

	if ((msg->fcount != framenum) || (msg->instance != instance)) {
		merr_hw("fcount, instance is not matched(S%d!=S%d, F%d!=F%d)", instance,
			msg->instance, instance, msg->fcount, framenum);
		return -EINVAL;
	}

	head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);
	head_hw_ip = is_get_hw_ip(head->id, hardware);
	if (!head_hw_ip || !test_bit(instance, &head_hw_ip->run_rsc_state)) {
		merr_hw("[F%d]ignore shot_callback after process stop", instance, framenum);
		return -EINVAL;
	}

	_is_hw_frame_dbg_trace(head_hw_ip, framenum, DEBUG_POINT_SHOT_CALLBACK);

	mdbg_adt(1, "[F%d]shot_callback\n", instance, framenum);

	ret = _is_hardware_shot(hardware, group, frame, framemgr, framenum);
	if (ret) {
		err_hw("_is_hardware_shot fail");
		return ret;
	}

	if (is_ss_grp && test_and_clear_bit(IS_DO_SHORT_CHAIN_SHOT, &sensor->seamless_state)) {
		ret = _is_hardware_shot(hardware, group->head->pnext, frame, framemgr, framenum);
		if (ret) {
			err_hw("_is_hardware_shot fail");
			return ret;
		}
	}

	if (!is_ss_grp || test_and_clear_bit(IS_SKIP_SENSOR_SHOT, &sensor->seamless_state))
		goto skip_sensor_shot;
	/*
	 * do the other device's group shot
	 * In case of VOTF, a user buffer should be set later than internal buffer.
	 * So, shot_callback of sensor group should be called after calling shot for PDP-3AA.
	 */
	ret = is_devicemgr_shot_callback(group, frame, frame->fcount, IS_DEVICE_ISCHAIN);
	if (ret) {
		err_hw("[F%d] is_devicemgr_shot_callback fail", frame->fcount);
		return -EINVAL;
	}

skip_sensor_shot:
	return ret;

}

int is_hardware_shot(struct is_hardware *hardware, u32 instance,
	struct is_group *group, struct is_frame *frame,
	struct is_framemgr *framemgr, ulong hw_map, u32 framenum)
{
	struct is_device_sensor *sensor;
	int ret = 0;
	ulong flags = 0;
	bool is_ss_grp;

	FIMC_BUG(!hardware);
	FIMC_BUG(!frame);

	sensor = group->device->sensor;
	is_ss_grp = test_bit(IS_GROUP_OTF_INPUT, &group->state);

	framemgr_e_barrier_common(framemgr, 0, flags);
	if (!test_bit(IS_GROUP_USE_MULTI_CH, &group->state))
		trans_frame(framemgr, frame, FS_HW_CONFIGURE);
	framemgr_x_barrier_common(framemgr, 0, flags);

	if (is_ss_grp && test_and_clear_bit(IS_SKIP_CHAIN_SHOT, &sensor->seamless_state)) {
		/* Trigger HW be in IDLE state because there will be no more shot */
		is_hardware_suspend(hardware, group);
		goto skip_chain_shot;
	}

	if (is_hw_check_crta_group(group->id)) {
		_is_hardware_shot_prepare(hardware, group, frame);

		if (test_and_clear_bit(IS_DO_SHORT_CHAIN_SHOT, &sensor->seamless_state))
			_is_hardware_shot_prepare(hardware, group->head->pnext, frame);

		return ret;
	}

	ret = _is_hardware_shot(hardware, group, frame, framemgr, framenum);
	if (ret) {
		err_hw("_is_hardware_shot fail");
		return ret;
	}

	if (is_ss_grp && test_and_clear_bit(IS_DO_SHORT_CHAIN_SHOT, &sensor->seamless_state)) {
		ret = _is_hardware_shot(hardware, group->head->pnext, frame, framemgr, framenum);
		if (ret) {
			err_hw("_is_hardware_shot fail");
			return ret;
		}
	}

skip_chain_shot:
	if (!is_ss_grp || test_and_clear_bit(IS_SKIP_SENSOR_SHOT, &sensor->seamless_state))
		goto skip_sensor_shot;
	/*
	 * do the other device's group shot
	 * In case of VOTF, a user buffer should be set later than internal buffer.
	 * So, shot_callback of sensor group should be called after calling shot for PDP-3AA.
	 */
	ret = is_devicemgr_shot_callback(group, frame, frame->fcount, IS_DEVICE_ISCHAIN);
	if (ret) {
		err_hw("[F%d] is_devicemgr_shot_callback fail", frame->fcount);
		return -EINVAL;
	}

skip_sensor_shot:
	return ret;
}

int is_hardware_get_meta(struct is_hw_ip *hw_ip, struct is_frame *frame,
	u32 instance, ulong hw_map, u32 output_id, enum ShotErrorType done_type)
{
	int ret = 0;

	FIMC_BUG(!hw_ip);

	if (((output_id != IS_HW_CORE_END) && (done_type == IS_SHOT_SUCCESS)
			&& (test_bit(hw_ip->id, &frame->core_flag)))
		|| (done_type != IS_SHOT_SUCCESS)) {
		/* FIMC-IS v3.x only
		 * There is a chance that the DMA done interrupt occurred before
		 * the core done interrupt. So we skip to call the get_meta function.
		 */
		/* There is no need to call get_meta function in case of NDONE */
		msdbg_hw(1, "%s: skip to get_meta [F:%d][B:0x%lx][C:0x%lx][O:0x%lx]\n",
			instance, hw_ip, __func__, frame->fcount,
			frame->bak_flag, frame->core_flag, frame->out_flag);
		return 0;
	}

	switch (hw_ip->id) {
	case DEV_HW_3AA0:
	case DEV_HW_3AA1:
	case DEV_HW_3AA2:
	case DEV_HW_3AA3:
		copy_ctrl_to_dm(frame->shot);
		fallthrough;
	case DEV_HW_ISP0:
	case DEV_HW_ISP1:
		ret = CALL_HWIP_OPS(hw_ip, get_meta, frame, hw_map);
		if (ret) {
			mserr_hw("[F:%d] get_meta fail", instance, hw_ip, frame->fcount);
			return 0;
		}
		if (hw_ip->id == DEV_HW_3AA0 ||
			hw_ip->id == DEV_HW_3AA1 ||
			hw_ip->id == DEV_HW_3AA2 ||
			hw_ip->id == DEV_HW_3AA3)
			is_hw_mcsc_set_ni(hw_ip->hardware, frame, instance);

		break;
	case DEV_HW_LME0:
	case DEV_HW_LME1:
	case DEV_HW_ORB0:
	case DEV_HW_ORB1:
	case DEV_HW_YPP:
	case DEV_HW_MCSC0:
	case DEV_HW_MCSC1:
	case DEV_HW_VRA:
	case DEV_HW_CLH0:
	case DEV_HW_BYRP:
	case DEV_HW_RGBP:
	case DEV_HW_MCFP:
		ret = CALL_HWIP_OPS(hw_ip, get_meta, frame, hw_map);
		if (ret) {
			mserr_hw("[F:%d] get_meta fail", instance, hw_ip, frame->fcount);
			return 0;
		}
		break;
	default:
		return 0;
		break;
	}

	msdbg_hw(1, "[F:%d]get_meta\n", instance, hw_ip, frame->fcount);

	return ret;
}

int check_shot_exist(struct is_framemgr *framemgr, u32 fcount, struct is_frame **frame)
{
	if (framemgr->queued_count[FS_HW_WAIT_DONE]) {
		*frame = find_frame(framemgr, FS_HW_WAIT_DONE, frame_fcount,
					(void *)(ulong)fcount);
		if (*frame) {
			if ((*frame)->type == SHOT_TYPE_INTERNAL)
				info_hw("[F:%d]is in complete_list\n", fcount);
			return INTERNAL_SHOT_EXIST;
		}
	}

	if (framemgr->queued_count[FS_HW_CONFIGURE]) {
		*frame = find_frame(framemgr, FS_HW_CONFIGURE, frame_fcount,
					(void *)(ulong)fcount);
		if (*frame) {
			if ((*frame)->type == SHOT_TYPE_INTERNAL)
				info_hw("[F:%d]is in process_list\n", fcount);
			return INTERNAL_SHOT_EXIST;
		}
	}

	return 0;
}

void is_set_hw_count(struct is_hardware *hardware, struct is_group *head,
	u32 instance, u32 fcount, u32 num_buffers, ulong hw_map)
{
	struct is_group *child;
	struct is_hw_ip *hw_ip = NULL;
	int hw_list[GROUP_HW_MAX], hw_index;
	int hw_maxnum = 0;
	u32 fs, cl, fe, dma;

	child = head->tail;
	while (child && (child->device_type == IS_DEVICE_ISCHAIN)) {
		hw_maxnum = is_get_hw_list(child->id, hw_list);
		for (hw_index = hw_maxnum - 1; hw_index >= 0; hw_index--) {
			hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
			if (!hw_ip) {
				merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
				continue;
			}

			if (test_bit_variables(hw_ip->id, &hw_map)) {
				fs = atomic_read(&hw_ip->count.fs);
				cl = atomic_read(&hw_ip->count.cl);
				fe = atomic_read(&hw_ip->count.fe);
				dma = atomic_read(&hw_ip->count.dma);
				atomic_set(&hw_ip->count.fs, (fcount - 1));
				atomic_set(&hw_ip->count.cl, (fcount - 1));
				atomic_set(&hw_ip->count.fe, (fcount - 1));
				atomic_set(&hw_ip->count.dma, (fcount - 1));
				msdbg_hw(1, "[F:%d]count clear, fs(%d->%d), fe(%d->%d), dma(%d->%d)\n",
						instance, hw_ip, fcount,
						fs, atomic_read(&hw_ip->count.fs),
						fe, atomic_read(&hw_ip->count.fe),
						dma, atomic_read(&hw_ip->count.dma));
			}
		}
		child = child->parent;
	}
}

int is_hardware_grp_shot(struct is_hardware *hardware, u32 instance,
	struct is_group *group, struct is_frame *frame, ulong hw_map)
{
	int ret = 0;
	struct is_device_sensor *sensor;
	struct is_hw_ip *hw_ip = NULL;
	struct is_frame *hw_frame;
	struct is_framemgr *framemgr;
	struct is_group *head, *ss_grp;
	ulong flags = 0;
	int num_buffers;
	bool reset;
	int i;

	FIMC_BUG(!hardware);
	FIMC_BUG(!frame);
	FIMC_BUG(instance >= IS_STREAM_COUNT);

	head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);

	hw_ip = is_get_hw_ip(head->id, hardware);
	if (!hw_ip) {
		mgerr("invalid hw_ip", head, head);
		return -EINVAL;
	}

	if (!atomic_read(&hardware->streaming[hardware->sensor_position[instance]]))
		msinfo_hw("grp_shot [F:%d][B:0x%lx][O:0x%lx][dva:%pad]\n",
			instance, hw_ip,
			frame->fcount,
			frame->bak_flag, frame->out_flag, &frame->dvaddr_buffer[0]);

	hw_ip->framemgr = &hardware->framemgr[hw_ip->id];
	framemgr = hw_ip->framemgr;

	framemgr_e_barrier_irqs(framemgr, 0, flags);

	hw_frame = get_frame(framemgr, FS_HW_FREE);
	if (hw_frame == NULL) {
		framemgr_x_barrier_irqr(framemgr, 0, flags);
		mserr_hw("free_head(NULL)", instance, hw_ip);
		return -EINVAL;
	}

	num_buffers = frame->num_buffers;
	reset = (num_buffers > 1) ? 0 : 1;
	is_hardware_fill_frame_info(instance, hw_frame, frame, head, hardware, reset);
	frame->type = SHOT_TYPE_EXTERNAL;
	hw_frame->type = frame->type;

	/* multi-buffer */
	hw_frame->planes	= frame->planes;
	hw_frame->num_buffers	= num_buffers;
	hw_frame->cur_buf_index	= 0;
	framemgr->batch_num = num_buffers;

	/* for NI (noise index) */
	hw_frame->noise_idx = frame->noise_idx;

	put_frame(framemgr, hw_frame, FS_HW_REQUEST);

	if (num_buffers > 1) {
		if (SUPPORT_HW_FRO(head->id)) {
			hw_ip->hw_fro_en = true;
		} else {
			hw_ip->hw_fro_en = false;
			hw_frame->type = SHOT_TYPE_MULTI;
			hw_frame->planes = 1;
			hw_frame->num_buffers = 1;
			hw_frame->cur_buf_index = 0;

			for (i = 1; i < num_buffers; i++) {
				hw_frame = get_frame(framemgr, FS_HW_FREE);
				if (hw_frame == NULL) {
					framemgr_x_barrier_irqr(framemgr, 0, flags);
					err_hw("[F%d]free_head(NULL)", frame->fcount);
					return -EINVAL;
				}

				reset = (i < (num_buffers - 1)) ? 0 : 1;
				is_hardware_fill_frame_info(instance, hw_frame, frame,
								head, hardware, reset);
				hw_frame->type = SHOT_TYPE_MULTI;
				hw_frame->planes = 1;
				hw_frame->num_buffers = 1;
				hw_frame->cur_buf_index = i;

				put_frame(framemgr, hw_frame, FS_HW_REQUEST);
			}
			hw_frame->type = frame->type; /* last buffer */
		}
	} else {
		hw_ip->hw_fro_en = false;
	}
	msdbg_hw(2, "ischain batch_num(%d), HW FRO(%d)\n", instance, hw_ip,
		num_buffers, hw_ip->hw_fro_en);

	if (test_bit(IS_GROUP_OTF_INPUT, &head->state)) {
		ss_grp = head->head;
		sensor = ss_grp->sensor;

		if (!atomic_read(&ss_grp->scount)) {
			hw_frame = peek_frame(framemgr, FS_HW_REQUEST);

			if (hw_frame)
				msinfo_hw("OTF start [F:%d][B:0x%lx][O:0x%lx]\n",
						instance, hw_ip,
						hw_frame->fcount,
						hw_frame->bak_flag, hw_frame->out_flag);

			if (is_sensor_g_aeb_mode(sensor))
				sensor->seamless_state = BIT(IS_SENSOR_SINGLE_MODE)
							| BIT(IS_DO_SHORT_CHAIN_SHOT);
		} else {
			atomic_set(&hw_ip->hardware->log_count, 0);
			framemgr_x_barrier_irqr(framemgr, 0, flags);

			return ret;
		}
	} else {
		/* set shot timer in DMA operation */
		mod_timer(&hw_ip->shot_timer, jiffies + msecs_to_jiffies(is_get_shot_timeout()));

		hw_frame = peek_frame(framemgr, FS_HW_REQUEST);
	}

	framemgr_x_barrier_irqr(framemgr, 0, flags);

	if (hw_frame == NULL) {
		mserr_hw("[F%d]req_head(NULL)", instance, hw_ip, frame->fcount);
		return -EINVAL;
	}

	is_set_hw_count(hardware, head, instance, frame->fcount, num_buffers, hw_map);
	ret = is_hardware_shot(hardware, instance, head, hw_frame, framemgr,
			hw_map, frame->fcount);
	if (ret) {
		mserr_hw("hardware_shot fail", instance, hw_ip);
		return -EINVAL;
	}

	return ret;
}

static int make_internal_shot(struct is_hw_ip *hw_ip, u32 instance, u32 fcount,
		struct is_framemgr *framemgr, u32 buf_index)
{
	int ret = 0;
	int i;
	struct is_frame *frame = NULL;

	if (framemgr->queued_count[FS_HW_FREE] < 3) {
		mswarn_hw("Free frame is less than 3", instance, hw_ip);
		frame_manager_print_info_queues(framemgr);
		check_hw_bug_count(hw_ip->hardware, 10);
	}

	ret = check_shot_exist(framemgr, fcount, &frame);
	if (ret == INTERNAL_SHOT_EXIST)
		return ret;

	frame = get_frame(framemgr, FS_HW_FREE);
	if (frame == NULL) {
		merr_hw("config_lock: frame(null)", instance);
		return -EINVAL;
	}

	frame->groupmgr		= NULL;
	frame->group		= NULL;
	frame->shot_ext		= NULL;
	frame->shot		= NULL;
	frame->shot_size	= 0;
	frame->shot_dva 	= 0;
	frame->fcount		= fcount;
	frame->rcount		= 0;
	frame->bak_flag		= 0;
	frame->out_flag		= 0;
	frame->core_flag	= 0;
	frame->result		= 0;
	atomic_set(&frame->shot_done_flag, 1);
	/* multi-buffer */
	frame->planes		= 0;
	if (hw_ip->hw_fro_en)
		frame->num_buffers = hw_ip->num_buffers;
	else
		frame->num_buffers = 1;

	for (i = 0; i < IS_MAX_PLANES; i++)
		frame->dvaddr_buffer[i]	= 0;

	frame->type = SHOT_TYPE_INTERNAL;
	frame->instance = instance;
	frame->cur_buf_index = buf_index;

	put_frame(framemgr, frame, FS_HW_REQUEST);

	return ret;
}

int is_hardware_config_lock(struct is_hw_ip *hw_ip, u32 instance, u32 framenum)
{
	int ret = 0;
	struct is_frame *frame;
	struct is_framemgr *framemgr;
	struct is_hardware *hardware;
	struct is_device_sensor *sensor;
	u32 sensor_fcount, hw_fcount;
	struct is_group *group, *head, *dev_head;
	struct is_hw_ip *hw_ip_ldr;

	FIMC_BUG(!hw_ip);

	group = hw_ip->group[instance];
	head = group->head;
	dev_head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);

	if (!test_bit(IS_GROUP_OTF_INPUT, &dev_head->state))
		return ret;

	msdbgs_hw(2, "[F:%d]C.L\n", instance, hw_ip, framenum);

	framemgr = hw_ip->framemgr;
	hardware = hw_ip->hardware;
	sensor = group->device->sensor;
	sensor_fcount = sensor->fcount;
	hw_fcount = atomic_read(&hw_ip->fcount);

	/* leader shot timer set in OTF used HW */
	hw_ip_ldr = is_get_hw_ip(dev_head->id, hw_ip->hardware);
	if (!hw_ip_ldr) {
		mserr_hw("[F%d] Failed to get leader hw_ip", instance, hw_ip, framenum);
		return -EINVAL;
	}

	mod_timer(&hw_ip_ldr->shot_timer, jiffies + msecs_to_jiffies(is_get_shot_timeout()));

retry_get_frame:
	framemgr_e_barrier(framemgr, 0);

	/* Sub stream context for 2EXP_MODE */
	if (hw_ip->frame_type == LIB_FRAME_HDR_SHORT) {
		dev_head = head->pnext;

		/* Re-use main stream frame that has been processed right before */
		frame = peek_frame_tail(framemgr, FS_HW_CONFIGURE);
		if (!frame) {
			framemgr_x_barrier(framemgr, 0);
			mserr_hw("[F%d] Failed to get main frame. seamless_state 0x%lx",
					instance, hw_ip, framenum,
					sensor->seamless_state);
			return -EINVAL;
		}

		goto shot_exist_frame;
	}

	if (!framemgr->queued_count[FS_HW_REQUEST]) {
		u32 num_buffers = hw_ip_ldr->hw_fro_en ? 1 : framemgr->batch_num;
		u32 buf_index;

		/* There is no request. Generate internal shot. */
		for (buf_index = 0; buf_index < num_buffers; buf_index++) {
			ret = make_internal_shot(hw_ip_ldr, instance, framenum + 1, framemgr,
						buf_index);
			if (ret == INTERNAL_SHOT_EXIST) {
				framemgr_x_barrier(framemgr, 0);

				return ret;
			} else if (ret) {
				framemgr_x_barrier(framemgr, 0);
				print_all_hw_frame_count(hardware);

				FIMC_BUG(1);
			}
		}
	}

	frame = peek_frame(framemgr, FS_HW_REQUEST);
	if (!frame) {
		framemgr_x_barrier(framemgr, 0);
		mserr_hw("frame is null", instance, hw_ip);

		return -EINVAL;
	} else if (frame->type == SHOT_TYPE_INTERNAL) {
		u32 log_count = atomic_read(&hardware->log_count);

		if ((log_count <= 20) || !(log_count % 100))
			msinfo_hw("config_lock: INTERNAL_SHOT [F:%d](%d) count(%d)\n",
					instance, hw_ip,
					frame->fcount, frame->index, log_count);
	} else if (((framemgr->queued_count[FS_HW_REQUEST] / framemgr->batch_num) > head->asyn_shots &&
		     frame->fcount < (sensor_fcount + 1)) ||
		    (frame->fcount <= hw_fcount)) {
		/* It's too old frame. Flush it */
		msinfo_hw("LATE_SHOT (%d)[F:%d][SF:%d][HF:%d][B:0x%lx][O:0x%lx][C:0x%lx]\n",
				instance, hw_ip,
				hw_ip->internal_fcount[instance], frame->fcount,
				sensor_fcount, hw_fcount,
				frame->bak_flag, frame->out_flag, frame->core_flag);

		frame->type = SHOT_TYPE_LATE;
		is_devicemgr_late_shot_handle(&sensor->group_sensor, frame, frame->type);
		trans_frame(framemgr, frame, FS_HW_WAIT_DONE);
		framemgr_x_barrier(framemgr, 0);

		goto retry_get_frame;
	}

	frame->frame_info[INFO_CONFIG_LOCK].cpu = raw_smp_processor_id();
	frame->frame_info[INFO_CONFIG_LOCK].pid = current->pid;
	frame->frame_info[INFO_CONFIG_LOCK].when = local_clock();

shot_exist_frame:
	framemgr_x_barrier(framemgr, 0);

	ret = is_hardware_shot(hardware, instance, dev_head,
			frame, framemgr, hardware->hw_map[instance], frame->fcount);
	if (ret) {
		mserr_hw("hardware_shot fail", instance, hw_ip);
		return -EINVAL;
	}

	return ret;
}

struct is_hw_ip *is_get_hw_ip(u32 group_id, struct is_hardware *hardware)
{
	int hw_list[GROUP_HW_MAX];
	int hw_maxnum = 0;

	hw_maxnum = is_get_hw_list(group_id, hw_list);

	return CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[0]);
}

static void is_hardware_seamless_change(struct is_group *group, u32 fcount)
{
	struct is_device_sensor *sensor;
	struct v4l2_control ctrl;
	u32 cur_hdr_mode;
	ulong prev_state;

	sensor = group->device->sensor;
	if (!is_sensor_g_aeb_mode(sensor))
		return;

	/* Read HDR mode from sensor module itf */
	ctrl.value = CAMERA_SENSOR_HDR_MODE_SINGLE;
	ctrl.id = V4L2_CID_SENSOR_GET_CUR_HDR_MODE;
	v4l2_subdev_call(sensor->subdev_module, core, ioctl,
			SENSOR_IOCTL_MOD_G_CTRL, &ctrl);
	cur_hdr_mode = ctrl.value;

	prev_state = sensor->seamless_state;

	/* IS-chain shot control for AEB mode */
	if (!test_bit(IS_GROUP_USE_MULTI_CH, &group->state)) {
		/* Long frame shot context */
		if (test_bit(IS_SENSOR_SINGLE_MODE, &sensor->seamless_state)) {
			if (test_and_clear_bit(IS_SENSOR_SWITCHING, &sensor->seamless_state))
				/* 2nd frame: Trigger short shot to turn on AEB */
				sensor->seamless_state = BIT(IS_SENSOR_2EXP_MODE);
			else if (cur_hdr_mode == CAMERA_SENSOR_HDR_MODE_2STHDR)
				/* 1st frame: Wait one more long shot */
				sensor->seamless_state |= BIT(IS_SENSOR_SWITCHING)
							| BIT(IS_DO_SHORT_CHAIN_SHOT);
		} else if (test_bit(IS_SENSOR_2EXP_MODE, &sensor->seamless_state)) {
			if (test_and_clear_bit(IS_SENSOR_SWITCHING, &sensor->seamless_state))
				/*
				 * Long frame shot after turning off AEB mode
				 * It's now on SINGLE mode
				 */
				sensor->seamless_state = BIT(IS_SENSOR_SINGLE_MODE);
			else if (cur_hdr_mode == CAMERA_SENSOR_HDR_MODE_SINGLE)
				/*
				 * Long frame shot to turn off AEB mode
				 * Wait next short shot
				 */
				set_bit(IS_SENSOR_SWITCHING, &sensor->seamless_state);
		}
	} else {
		/* Short frame shot context */
		if (test_bit(IS_SENSOR_2EXP_MODE, &sensor->seamless_state)) {
			if (test_bit(IS_SENSOR_SWITCHING, &sensor->seamless_state) ||
					cur_hdr_mode == CAMERA_SENSOR_HDR_MODE_SINGLE)
				/*
				 * Short frame to turn off AEB mode
				 * Skip next Short frame shot
				 */
				sensor->seamless_state |= BIT(IS_SENSOR_SWITCHING)
							| BIT(IS_SKIP_CHAIN_SHOT);
		} else {
			/* Short frame shot conext in SINGLE mode: Abnormal HW operation! */
			mgerr("[F%d] Invalid sensor output on SINGLE mode. seamless_state 0x%lx",
					group, group, fcount, sensor->seamless_state);

			sensor->seamless_state |= BIT(IS_SKIP_CHAIN_SHOT)
						| BIT(IS_SKIP_SENSOR_SHOT);
		}
	}

	/*
	 * Sensor shot control for AEB mode
	 * Long shot should not trigger sensor shot on AEB on
	 */
	if (!test_bit(IS_GROUP_USE_MULTI_CH, &group->state) &&
			test_bit(IS_SENSOR_2EXP_MODE, &sensor->seamless_state))
		set_bit(IS_SKIP_SENSOR_SHOT, &sensor->seamless_state);

	dbg_hw(2, "[%s][F%d]%s: %s_shot cur_hdr_mode %d seamless_state 0x%lx\n",
			group_id_name[group->id],
			fcount,
			__func__,
			test_bit(IS_GROUP_USE_MULTI_CH, &group->state) ? "SHORT" : "LONG",
			cur_hdr_mode,
			sensor->seamless_state);

	if ((prev_state ^ sensor->seamless_state) & IS_SENSOR_SEAMLESS_MODE_MASK)
		mginfo("[F%d] Change seamless_state 0x%lx -> 0x%lx\n",
				group, group, fcount,
				prev_state, sensor->seamless_state);
}

void is_hardware_frame_start(struct is_hw_ip *hw_ip, u32 instance)
{
	struct is_frame *frame = NULL, *check_frame;
	struct is_framemgr *framemgr;
	struct is_group *group, *head;
	u32 config_count;
	u32 hw_fcount;

	FIMC_BUG_VOID(!hw_ip);

	group = hw_ip->group[instance];
	head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);
	FIMC_BUG_VOID(!head);

	hw_fcount = atomic_read(&hw_ip->fcount);

	if (test_bit(IS_GROUP_OTF_INPUT, &head->state))
		is_hardware_seamless_change(group, hw_fcount);

	/* Frame state trasition is done by main stream */
	if (hw_ip->frame_type == LIB_FRAME_HDR_SHORT)
		goto skip_start;

	/*
	 * If there are hw_ips having framestart processing
	 * and they are bound by OTF, the problem that same action was duplicated
	 * maybe happened.
	 * ex. 1) 3A0* => ISP* -> MCSC0* : no problem
	 *     2) 3A0* -> ISP  -> MCSC0* : problem happened!!
	 *      (* : called is_hardware_frame_start)
	 * Only leader group in OTF groups can control frame.
	 */
	framemgr = hw_ip->framemgr;

	framemgr_e_barrier(framemgr, 0);
	frame = peek_frame(framemgr, FS_HW_CONFIGURE);

	if (test_bit(IS_GROUP_OTF_INPUT, &head->state)) {
		while (frame) {
			config_count = framemgr->queued_count[FS_HW_CONFIGURE];
			if (config_count > 1) {
				framemgr->proc_warn_cnt++;
				if (framemgr->proc_warn_cnt > 5) {
					msinfo_hw("[F%d][HWF%d][CF%d]flush configured frame\n",
						instance, hw_ip, frame->fcount, hw_fcount,
						config_count);

					frame_manager_print_info_queues(framemgr);

					trans_frame(framemgr, frame, FS_HW_WAIT_DONE);
					frame = peek_frame(framemgr, FS_HW_CONFIGURE);
				} else {
					break;
				}
			} else {
				framemgr->proc_warn_cnt = 0;
				break;
			}
		}
	}

	if (!frame) {
		check_frame = find_frame(framemgr, FS_HW_WAIT_DONE,
			frame_fcount, (void *)(ulong)atomic_read(&hw_ip->fcount));
		if (check_frame) {
			msdbgs_hw(2, "[F:%d] already processed to HW_WAIT_DONE state",
					instance, hw_ip, check_frame->fcount);

			framemgr_x_barrier(framemgr, 0);
		} else {
			/* error happened..print the frame info */
			frame_manager_print_info_queues(framemgr);
			print_all_hw_frame_count(hw_ip->hardware);
			framemgr_x_barrier(framemgr, 0);
			mserr_hw("FSTART frame null (%d) (%d != %d)", instance, hw_ip,
					hw_ip->internal_fcount[instance], group->id, head->id);
		}
		atomic_set(&hw_ip->status.Vvalid, V_VALID);
		return;
	}

	/* TODO: multi-instance */
	frame->frame_info[INFO_FRAME_START].cpu = raw_smp_processor_id();
	frame->frame_info[INFO_FRAME_START].pid = current->pid;
	frame->frame_info[INFO_FRAME_START].when = local_clock();

	if (test_bit(IS_GROUP_OTF_INPUT, &head->state)) {
		if (framemgr->batch_num == 1 && frame->fcount != atomic_read(&hw_ip->count.fs)) {
			/* error handling */
			info_hw("frame_start_isr (%d, %d)\n", frame->fcount,
					atomic_read(&hw_ip->count.fs));
			atomic_set(&hw_ip->count.fs, frame->fcount);
		}
	}

	trans_frame(framemgr, frame, FS_HW_WAIT_DONE);
	framemgr_x_barrier(framemgr, 0);

skip_start:
	if (atomic_read(&hw_ip->status.Vvalid) == V_VALID)
		msdbg_hw(1, "[F%d][HF%d] already in VValid", instance, hw_ip,
				frame ? frame->fcount : -1, hw_fcount);
	else
		atomic_set(&hw_ip->status.Vvalid, V_VALID);
}

int is_hardware_sensor_start(struct is_hardware *hardware, u32 instance,
	ulong hw_map, struct is_group *group)
{
	int ret = 0;
	struct is_hw_ip *hw_ip;
	struct is_group *head, *child;
	int hw_list[GROUP_HW_MAX], hw_maxnum, hw_index;
	struct is_device_sensor *sensor;
	ulong streaming_state = BIT(HW_SENSOR_STREAMING);

	FIMC_BUG(!hardware);

	head = group;

	if (!test_bit(IS_GROUP_OTF_INPUT, &head->state)) {
		mgwarn("group head is not OTF. So, there is nothing to do", head, head);
		return 0;
	}

	child = head->tail;
	while (child && (child->device_type == IS_DEVICE_ISCHAIN)) {
		hw_maxnum = is_get_hw_list(child->id, hw_list);
		for (hw_index = hw_maxnum - 1; hw_index >= 0; hw_index--) {
			hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
			if (!hw_ip) {
				merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
				return -EINVAL;
			}

			ret = CALL_HWIP_OPS(hw_ip, sensor_start, instance);
			if (ret) {
				mserr_hw("sensor_start fail", instance, hw_ip);
				return -EINVAL;
			}
			msdbg_hw(2, "hw_sensor_start [P:0x%lx]\n", instance, hw_ip, hw_map);
		}
		child = child->parent;
	}

	sensor = group->device->sensor;
	if (sensor && test_bit(IS_SENSOR_OTF_OUTPUT, &sensor->state))
		streaming_state |= BIT(HW_ISCHAIN_STREAMING);

	atomic_set(&hardware->streaming[hardware->sensor_position[instance]], streaming_state);
	atomic_set(&hardware->bug_count, 0);
	atomic_set(&hardware->log_count, 0);
	clear_bit(HW_OVERFLOW_RECOVERY, &hardware->hw_recovery_flag);

	return ret;
}

int is_hardware_sensor_stop(struct is_hardware *hardware, u32 instance,
	ulong hw_map, struct is_group *group)
{
	int ret = 0;
	int retry;
	struct is_frame *frame;
	struct is_framemgr *framemgr;
	struct is_group *head;
	struct is_hw_ip *hw_ip = NULL;
	ulong flags = 0;

	FIMC_BUG(!hardware);

	head = group;

	if (!test_bit(IS_GROUP_OTF_INPUT, &head->state)) {
		mgwarn("group head is not OTF. So, there is nothing to do", head, head);
		return 0;
	}

	hw_ip = is_get_hw_ip(head->id, hardware);
	if (!hw_ip) {
		mgerr("invalid hw_ip", head, head);
		return -EINVAL;
	}

	ret = CALL_HWIP_OPS(hw_ip, sensor_stop, instance);
	if (ret)
		mserr_hw("sensor_stop fail", instance, hw_ip);

	msdbg_hw(2, "hw_sensor_stop [P:0x%lx]\n", instance, hw_ip, hw_map);

	if (hw_ip->frame_type == LIB_FRAME_HDR_SHORT) {
		mginfo("Skip sensor_stop for multi_ch group", head, head);
		return 0;
	}

	atomic_set(&hardware->streaming[hardware->sensor_position[instance]], 0);
	atomic_set(&hardware->bug_count, 0);
	atomic_set(&hardware->log_count, 0);
	clear_bit(HW_OVERFLOW_RECOVERY, &hardware->hw_recovery_flag);

	/* decrease lic_update state if used */
	if (atomic_read(&hardware->lic_updated) > 0)
		atomic_dec(&hardware->lic_updated);

	framemgr = hw_ip->framemgr;
	retry = 99;
	while (--retry) {
		framemgr_e_barrier_irqs(framemgr, 0, flags);
		if (!framemgr->queued_count[FS_HW_WAIT_DONE]) {
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			break;
		}

		frame = peek_frame(framemgr, FS_HW_WAIT_DONE);
		if (frame == NULL) {
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			break;
		}

		if (frame->num_buffers > 1) {
			retry = 0;
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			break;
		}

		msinfo_hw("hw_sensor_stop: com_list: [F:%d][%d][O:0x%lx][C:0x%lx][(%d)",
			instance, hw_ip,
			frame->fcount, frame->type, frame->out_flag, frame->core_flag,
			framemgr->queued_count[FS_HW_WAIT_DONE]);
		mswarn_hw(" %d com waiting...", instance, hw_ip,
			framemgr->queued_count[FS_HW_WAIT_DONE]);

		framemgr_x_barrier_irqr(framemgr, 0, flags);
		usleep_range(1000, 1001);
	}

	if (!retry) {
		frame = NULL;
		framemgr_e_barrier_irqs(framemgr, 0, flags);

		frame = peek_frame(framemgr, FS_HW_WAIT_DONE);

		framemgr_x_barrier_irqr(framemgr, 0, flags);

		if (frame) {
			ret = is_hardware_frame_ndone(hw_ip, frame, instance, IS_SHOT_UNPROCESSED);
			if (ret)
				mserr_hw("hardware_frame_ndone fail", instance, hw_ip);
		}
	}

	/* for last fcount */
	print_all_hw_frame_count(hardware);

	msinfo_hw("hw_sensor_stop: done[P:0x%lx]\n", instance, hw_ip, hw_map);

	return ret;
}

int is_hardware_process_start(struct is_hardware *hardware, u32 instance,
	u32 group_id)
{
	int ret = 0;
	ulong hw_map;
	int hw_list[GROUP_HW_MAX];
	int hw_index, hw_maxnum;
	struct is_hw_ip *hw_ip = NULL;

	FIMC_BUG(!hardware);

	mdbg_hw(1, "process_start [%s]\n", instance, group_id_name[group_id]);

	hw_map = hardware->hw_map[instance];
	hw_maxnum = is_get_hw_list(group_id, hw_list);
	for (hw_index = 0; hw_index < hw_maxnum; hw_index++) {
		hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
		if (!hw_ip) {
			merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
			return -EINVAL;
		}

		if (test_and_set_bit(instance, &hw_ip->run_rsc_state))
			mswarn_hw("try to enable enabled instance", instance, hw_ip);

		ret = CALL_HWIP_OPS(hw_ip, enable, instance, hw_map);
		if (ret) {
			mserr_hw("enable fail", instance, hw_ip);
			return -EINVAL;
		}

		hw_ip->internal_fcount[instance] = 0;
	}

	return ret;
}

static int flush_frames_in_instance(struct is_hw_ip *hw_ip,
	struct is_framemgr *framemgr, u32 instance,
	enum is_frame_state state, enum ShotErrorType done_type)
{
	int retry = 150;
	struct is_frame *frame;
	int ret = 0;
	ulong flags = 0;
	u32 queued_count = 0;

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	queued_count = framemgr->queued_count[state];
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	while (--retry && queued_count) {
		framemgr_e_barrier_irqs(framemgr, 0, flags);
		frame = peek_frame(framemgr, state);
		if (!frame) {
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			break;
		}

		if (frame->instance != instance) {
			msinfo_hw("different instance's frame was detected\n",
				instance, hw_ip);
			info_hw("\t frame's instance: %d, queued count: %d\n",
				frame->instance, framemgr->queued_count[state]);

			/* FIXME: consider mixing frames among instances */
			framemgr_x_barrier_irqr(framemgr, 0, flags);
			break;
		}

		info_hw("frame info: %s(queued count: %d) [F:%d][T:%d][O:0x%lx][C:0x%lx]",
			hw_frame_state_name[frame->state], framemgr->queued_count[state],
			frame->fcount, frame->type, frame->out_flag, frame->core_flag);

		/* Core_flag need to be cleared in case,
		 * Other IPs are currently using framemgr of current IP.
		 */
		frame->core_flag = 0;
		set_bit(hw_ip->id, &frame->core_flag);

		framemgr_x_barrier_irqr(framemgr, 0, flags);
		ret = is_hardware_frame_ndone(hw_ip, frame, instance, done_type);
		if (ret) {
			mserr_hw("failed to process as NDONE", instance, hw_ip);
			break;
		}

		framemgr_e_barrier_irqs(framemgr, 0, flags);
		warn_hw("flushed a frame in %s, queued count: %d ",
			hw_frame_state_name[frame->state], framemgr->queued_count[state]);

		queued_count = framemgr->queued_count[state];
		framemgr_x_barrier_irqr(framemgr, 0, flags);

		if (queued_count > 0)
			usleep_range(1000, 1000);
	}

	return ret;
}

void is_hardware_force_stop(struct is_hardware *hardware,
	struct is_hw_ip *hw_ip, u32 instance)
{
	int ret = 0;
	struct is_framemgr *framemgr;
	enum is_frame_state state;

	FIMC_BUG_VOID(!hw_ip);

	framemgr = hw_ip->framemgr;
	msinfo_hw("frame manager queued count (%s: %d)(%s: %d)(%s: %d)\n",
		instance, hw_ip,
		hw_frame_state_name[FS_HW_WAIT_DONE],
		framemgr->queued_count[FS_HW_WAIT_DONE],
		hw_frame_state_name[FS_HW_CONFIGURE],
		framemgr->queued_count[FS_HW_CONFIGURE],
		hw_frame_state_name[FS_HW_REQUEST],
		framemgr->queued_count[FS_HW_REQUEST]);

	/* reverse order */
	for (state = FS_HW_WAIT_DONE; state > FS_HW_FREE; state--) {
		ret = flush_frames_in_instance(hw_ip, framemgr, instance, state, IS_SHOT_UNPROCESSED);
		if (ret) {
			mserr_hw("failed to flush frames in %s", instance, hw_ip,
				hw_frame_state_name[state]);
			return;
		}
	}
}

void is_hardware_process_stop(struct is_hardware *hardware, u32 instance,
	u32 group_id, u32 mode)
{
	int ret;
	int hw_list[GROUP_HW_MAX];
	int hw_index, hw_maxnum;
	ulong hw_map;
	struct is_hw_ip *hw_ip;
	struct is_hw_ip *hw_ip_chd;
	struct is_framemgr *framemgr;
	struct is_frame *frame;
	int retry;
	ulong flags = 0;
	u32 state;
	struct is_group *group;
	struct is_group *head;

	FIMC_BUG_VOID(!hardware);

	mdbg_hw(1, "process_stop [%s](%d)\n", instance, group_id_name[group_id], mode);

	hw_ip = is_get_hw_ip(group_id, hardware);
	FIMC_BUG_VOID(!hw_ip);

	framemgr = hw_ip->framemgr;
	FIMC_BUG_VOID(!framemgr);

	group = hw_ip->group[instance];
	if (!group) {
		merr_hw("failed to get group", instance);
		return;
	}

	head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);
	FIMC_BUG_VOID(!head);

	if (test_bit(IS_GROUP_OTF_INPUT, &head->state)) {
		state = FS_HW_WAIT_DONE;
	} else {
		/*
		 * If SW batch is used, all buffers in one batch shot must be retuend at once.
		 * So, all frames that are in FS_HW_REQUEST in one batch shot
		 * must be waited untill done.
		 * that's because all frames in one batch shot is changed to FS_HW_REQUEST
		 * at is_hardware_grp_shot() function,
		 */
		if (hw_ip->hw_fro_en == false && framemgr->batch_num > 1)
			state = FS_HW_REQUEST;
		else
			state = FS_HW_CONFIGURE;
	}

	for (; state <= FS_HW_WAIT_DONE; state++) {
		framemgr_e_barrier_common(framemgr, 0, flags);
		frame = peek_frame(framemgr, state);
		framemgr_x_barrier_common(framemgr, 0, flags);
		if (frame && frame->instance != instance) {
			msinfo_hw("frame->instance(%d), queued_count(%s(%d))\n",
					instance, hw_ip, frame->instance,
					hw_frame_state_name[state],
					framemgr->queued_count[state]);
		} else {
			retry = 10;
			while (--retry && framemgr->queued_count[state]) {
				mswarn_hw("%s(%d) com waiting...", instance, hw_ip,
						hw_frame_state_name[state],
						framemgr->queued_count[state]);
				usleep_range(5000, 5000);
			}
			if (!retry)
				mswarn_hw("waiting(until frame empty) is fail", instance, hw_ip);
		}
	}

	hw_map = hardware->hw_map[instance];
	hw_maxnum = is_get_hw_list(group_id, hw_list);
	for (hw_index = 0; hw_index < hw_maxnum; hw_index++) {
		hw_ip_chd = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
		if (!hw_ip) {
			merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
			return;
		}

		if (!test_and_clear_bit(instance, &hw_ip->run_rsc_state))
			mswarn_hw("try to disable disabled instance", instance, hw_ip);

		ret = CALL_HWIP_OPS(hw_ip_chd, disable, instance, hw_map);
		if (ret) {
			mserr_hw("disable fail", instance, hw_ip_chd);
		}
	}

	/* reset shot timer after process stop */
	if (test_bit(IS_GROUP_OTF_INPUT, &head->state)) {
		del_timer_sync(&hw_ip->shot_timer);
		msinfo_hw("del timer\n", instance, hw_ip);
	}

	if (mode == 0)
		return;

	is_hardware_force_stop(hardware, hw_ip, instance);
	hw_ip->internal_fcount[instance] = 0;

	return;
}

int is_hardware_open(struct is_hardware *hardware, u32 hw_id,
	struct is_group *group, u32 instance, bool rep_flag)
{
	int ret = 0;
	int ret_err = 0;
	struct is_hw_ip *hw_ip = NULL;
	int refcount;
	u32 f_type;
	enum exynos_sensor_position ss_pos;

	FIMC_BUG(!hardware);

	hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_id);
	if (!hw_ip) {
		merr_hw("invalid id (%d)", instance, hw_id);
		ret = -EINVAL;
		goto err_slot;
	}

	refcount = atomic_inc_return(&hw_ip->rsccount);
	if (refcount == 1) {
		hw_ip->hardware = hardware;
		hw_ip->framemgr = &hardware->framemgr[hw_ip->id];

		ret = CALL_HWIP_OPS(hw_ip, open, instance);
		if (ret) {
			mserr_hw("open fail", instance, hw_ip);
			goto err_open;
		}

		memset(hw_ip->debug_info, 0x00, sizeof(struct hw_debug_info) * DEBUG_FRAME_COUNT);
		memset(hw_ip->debug_ext_info, 0x00, sizeof(struct hw_debug_info) * DEBUG_EXT_MAX);
		memset(hw_ip->setfile, 0x00, sizeof(struct is_hw_ip_setfile) * SENSOR_POSITION_MAX);
		hw_ip->applied_scenario = -1;
		hw_ip->debug_index[0] = 0;
		hw_ip->debug_index[1] = 0;
		hw_ip->sfr_dump_flag = false;
		atomic_set(&hw_ip->count.fs, 0);
		atomic_set(&hw_ip->count.cl, 0);
		atomic_set(&hw_ip->count.fe, 0);
		atomic_set(&hw_ip->count.dma, 0);
		atomic_set(&hw_ip->status.Vvalid, V_BLANK);
		hw_ip->run_rsc_state = 0;
		atomic_inc(&hardware->slot_rsccount[group->slot]);
		timer_setup(&hw_ip->shot_timer, (void (*)(struct timer_list *))is_hardware_shot_timer, 0);

		if (!test_bit(IS_GROUP_OTF_INPUT, &group->state)) {
#if defined(MULTI_SHOT_TASKLET)
			tasklet_init(&hw_ip->tasklet_mshot, tasklet_mshot, (unsigned long)hw_ip);
#elif defined(MULTI_SHOT_KTHREAD)
			ret = is_hardware_init_mshot_thread(hw_ip);
			if (ret) {
				serr_hw("is_hardware_init_mshot_thread is fail(%d)", hw_ip, ret);
				goto err_kthread;
			}

			is_fpsimd_set_task_using(hw_ip->mshot_task, NULL);

			sinfo_hw("multi-shot kthread: %s[%d] started\n", hw_ip,
				hw_ip->mshot_task->comm, hw_ip->mshot_task->pid);
#endif
		}
	}

	ss_pos = hardware->sensor_position[instance];
	if (!refcount_read(&hw_ip->ref_setfile[ss_pos]))
		refcount_set(&hw_ip->ref_setfile[ss_pos], 1);
	else
		refcount_inc(&hw_ip->ref_setfile[ss_pos]);

	hw_ip->group[instance] = group;
	f_type = is_sensor_get_frame_type(instance);
	ret = CALL_HWIP_OPS(hw_ip, init, instance, rep_flag, f_type);
	if (ret) {
		mserr_hw("init fail", instance, hw_ip);
		goto err_init;
	}

	ret = CALL_ADT_MSG_OPS(hardware->icpu_adt, register_response_msg_cb, instance,
			PABLO_HIC_SHOT, _is_hardware_shot_callback);
	if (ret) {
		merr_adt("icpu_adt register_response_msg_cb fail", instance);
		goto err_adt;
	}

	set_bit(hw_id, &hardware->logical_hw_map[instance]);
	set_bit(hw_id, &hardware->hw_map[instance]);
	msinfo_hw("open. rsccnt %d ref_setfile %d\n", instance, hw_ip,
			refcount, refcount_read(&hw_ip->ref_setfile[ss_pos]));

	return 0;

err_adt:
	ret = CALL_HWIP_OPS(hw_ip, deinit, instance);
	if (ret)
		mserr_hw("deinit fail", instance, hw_ip);
err_init:
	if (refcount_dec_and_test(&hw_ip->ref_setfile[ss_pos]))
		hw_ip->setfile[ss_pos].using_count = 0;
#if defined(MULTI_SHOT_KTHREAD)
	if (refcount == 1)
		is_hardware_deinit_mshot_thread(hw_ip);
err_kthread:
#endif
	if (refcount == 1) {
		ret_err = CALL_HWIP_OPS(hw_ip, close, instance);
		if (ret_err)
			mserr_hw("close fail (%d)", instance, hw_ip, ret_err);
	}
err_open:
	atomic_dec(&hw_ip->rsccount);
err_slot:
	return ret;
}

static int is_hardware_close_by_rsccount(struct is_hardware *hardware, struct is_hw_ip *hw_ip,
	u32 hw_id, u32 instance)
{
	int ret = 0;
	int refcount;

	FIMC_BUG(!hardware);
	FIMC_BUG(!hw_ip);

	refcount = atomic_dec_return(&hw_ip->rsccount);
	if (refcount == 0) {
		hw_ip->framemgr = &hardware->framemgr[hw_ip->id];

		del_timer_sync(&hw_ip->shot_timer);

		/* Before close hw_ip, flush frames if remained */
		is_hardware_force_stop(hardware, hw_ip, instance);

		ret = CALL_HWIP_OPS(hw_ip, close, instance);
		if (ret)
			mserr_hw("close fail", instance, hw_ip);

		memset(hw_ip->debug_info, 0x00, sizeof(struct hw_debug_info) * DEBUG_FRAME_COUNT);
		memset(hw_ip->debug_ext_info, 0x00, sizeof(struct hw_debug_info) * DEBUG_EXT_MAX);
		hw_ip->debug_index[0] = 0;
		hw_ip->debug_index[1] = 0;
		clear_bit(HW_OPEN, &hw_ip->state);
		clear_bit(HW_INIT, &hw_ip->state);
		clear_bit(HW_CONFIG, &hw_ip->state);
		clear_bit(HW_RUN, &hw_ip->state);
		clear_bit(HW_TUNESET, &hw_ip->state);
		atomic_set(&hw_ip->fcount, 0);
		atomic_set(&hw_ip->instance, 0);
		hw_ip->run_rsc_state = 0;
		atomic_dec(&hardware->slot_rsccount[hw_ip->group[instance]->slot]);

#if defined(MULTI_SHOT_KTHREAD)
		is_hardware_deinit_mshot_thread(hw_ip);
#endif
	}

	return ret;
}

static int is_hardware_close_changed_chain(struct is_hardware *hardware, u32 hw_id, u32 instance)
{
	int ret = 0;
	struct is_hw_ip *hw_ip = NULL;
	int mapped_hw_id;

	FIMC_BUG(!hardware);

	mapped_hw_id = is_hw_check_changed_chain_id(hardware, hw_id, instance);
	if (mapped_hw_id) {
		hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, mapped_hw_id);
		if (!hw_ip) {
			merr_hw("invalid id (%d)", instance, mapped_hw_id);
			return -EINVAL;
		}

		info("[%d] %s: hw(%d) rsccount(%d)", instance, __func__, mapped_hw_id,
			atomic_read(&hw_ip->rsccount));
		is_hardware_close_by_rsccount(hardware, hw_ip, hw_id, instance);
	}

	return ret;
}

int is_hardware_close(struct is_hardware *hardware,u32 hw_id, u32 instance)
{
	int ret = 0;
	struct is_hw_ip *hw_ip;
	u32 phys_hw_mask;
	enum exynos_sensor_position ss_pos;

	FIMC_BUG(!hardware);

	/*
	 * The hw_ip have to be same between open and close to keep rsccount pair.
	 * So, logical_hw_ip have to keep initial hw_ip that are used open time.
	 */
	if (!test_bit(hw_id, &hardware->logical_hw_map[instance])) {
		merr_hw("[ID:%d]invalid hw_map state", instance, hw_id);
		return -EINVAL;
	}

	ret = CALL_ADT_MSG_OPS(hardware->icpu_adt, register_response_msg_cb, instance,
		PABLO_HIC_SHOT, NULL);
	if (ret)
		err_hw("icpu_adt register_response_msg_cb fail");

	hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_id);
	if (!hw_ip) {
		merr_hw("invalid id (%d)", instance, hw_id);
		return -EINVAL;
	}

	ret = CALL_HWIP_OPS(hw_ip, deinit, instance);
	if (ret)
		mserr_hw("deinit fail", instance, hw_ip);

	ret = is_hardware_close_changed_chain(hardware, hw_id, instance);
	if (ret)
		mserr_hw("close_changed_chain fail", instance, hw_ip);

	is_hardware_close_by_rsccount(hardware, hw_ip, hw_id, instance);

	clear_bit(hw_id, &hardware->logical_hw_map[instance]);
	clear_bit(hw_id, &hardware->hw_map[instance]);

	/* Reset using_count for the last sensor instance. */
	ss_pos = hardware->sensor_position[instance];
	if (refcount_dec_and_test(&hw_ip->ref_setfile[ss_pos]))
		hw_ip->setfile[ss_pos].using_count = 0;

	/* clear change hw_ip */
	if ((1 << hw_id) & DEV_HW_3AA_MASK)
		phys_hw_mask = DEV_HW_3AA_MASK;
	else if ((1 << hw_id) & DEV_HW_PAF_MASK)
		phys_hw_mask = DEV_HW_PAF_MASK;
	else
		phys_hw_mask = 0;
	hardware->hw_map[instance] &= ~(phys_hw_mask);

	msinfo_hw("close. rsccnt %d ref_setfile %d map [L(0x%lx) P(0x%lx)]\n", instance, hw_ip,
		atomic_read(&hw_ip->rsccount), refcount_read(&hw_ip->ref_setfile[ss_pos]),
		hardware->logical_hw_map[instance], hardware->hw_map[instance]);
	return ret;
}

int is_hardware_change_chain(struct is_hardware *hardware,
	struct is_group *group, u32 instance, u32 next_id)
{
	int ret = 0;
	struct is_group *child;
	struct is_hw_ip *hw_ip;

	FIMC_BUG(!hardware);

	child = group->tail;
	while (child && (child->device_type == IS_DEVICE_ISCHAIN)) {
		hw_ip = is_get_hw_ip(child->id, hardware);
		if (!hw_ip) {
			mgerr("invalid hw_ip", child, child);
			return -EINVAL;
		}

		ret = CALL_HWIP_OPS(hw_ip, change_chain, instance, next_id, hardware);
		if (ret) {
			mserr_hw("change_chain callback is fail", instance, hw_ip);
			return ret;
		}

		msinfo_hw("change_chain done (map: L(0x%lx) P(0x%lx))", instance, hw_ip,
			hardware->logical_hw_map[instance], hardware->hw_map[instance]);

		child = child->parent;
	}

	return ret;
}

int do_frame_done_work_func(struct is_interface *itf, int wq_id, u32 instance,
	u32 group_id, u32 fcount, u32 rcount, u32 status)
{
	int ret = 0;
	bool retry_flag = false;
	struct work_struct *work_wq;
	struct is_work_list *work_list;
	struct is_work *work;

	work_wq   = &itf->work_wq[wq_id];
	work_list = &itf->work_list[wq_id];
retry:
	get_free_work(work_list, &work);
	if (work) {
		work->msg.id		= 0;
		work->msg.command	= IS_FRAME_DONE;
		work->msg.instance	= instance;
		work->msg.group		= GROUP_ID(group_id);
		work->msg.param1	= fcount;
		work->msg.param2	= rcount;
		work->msg.param3	= status; /* status: enum ShotErrorType */
		work->msg.param4	= 0;

		work->fcount = work->msg.param1;
		set_req_work(work_list, work);

		if (!work_pending(work_wq))
			wq_func_schedule(itf, work_wq);
	} else {
		merr_hw("free work item is empty [retry_cnt:%d][WQ:%d][G:%d]",
			instance, (int)retry_flag, wq_id, group_id);
		if (retry_flag == false) {
			retry_flag = true;
			goto retry;
		}

		/* WQ debug info */
		print_fre_work_list(work_list);
		print_req_work_list(work_list);

		ret = -EINVAL;
	}

	return ret;
}

int is_hardware_shot_done(struct is_hw_ip *hw_ip, struct is_frame *frame,
	struct is_framemgr *framemgr, enum ShotErrorType done_type)
{
	int ret = 0;
	struct work_struct *work_wq;
	struct is_work_list *work_list;
	struct is_work *work;
	struct is_group *head, *group;
	u32  req_id, instance;
	ulong flags = 0;

	FIMC_BUG(!hw_ip);

	instance = frame->instance;
	group = hw_ip->group[instance];
	if (!group) {
		sdbg_hw(2, "hw_ip->group[%d] is NULL, use hw_ip->instance(%d)",
				hw_ip, instance, atomic_read(&hw_ip->instance));

		instance = atomic_read(&hw_ip->instance);
		group = hw_ip->group[instance];
		if (!group) {
			mserr_hw("group is null", instance, hw_ip);
			return -EINVAL;
		}
	}
	head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);

	msdbgs_hw(2, "shot_done [F:%d][B:0x%lx][C:0x%lx][O:0x%lx]\n",
		frame->instance, hw_ip, frame->fcount,
		frame->bak_flag, frame->core_flag, frame->out_flag);

	if (frame->type == SHOT_TYPE_INTERNAL || frame->type == SHOT_TYPE_MULTI)
		goto free_frame;

	switch (head->id) {
	case GROUP_ID_PAF0:
	case GROUP_ID_PAF1:
	case GROUP_ID_PAF2:
	case GROUP_ID_PAF3:
	case GROUP_ID_3AA0:
	case GROUP_ID_3AA1:
	case GROUP_ID_3AA2:
	case GROUP_ID_3AA3:
	case GROUP_ID_LME0:
	case GROUP_ID_LME1:
	case GROUP_ID_ORB0:
	case GROUP_ID_ORB1:
	case GROUP_ID_ISP0:
	case GROUP_ID_ISP1:
	case GROUP_ID_MCS0:
	case GROUP_ID_MCS1:
	case GROUP_ID_VRA0:
	case GROUP_ID_CLH0:
	case GROUP_ID_YPP:
	case GROUP_ID_YUVP:
	case GROUP_ID_BYRP:
	case GROUP_ID_RGBP:
	case GROUP_ID_MCFP:
		req_id = head->leader.id;
		break;
	default:
		err_hw("invalid group (G%d)", head->id);
		goto exit;
	}

	if (!test_bit_variables(req_id, &frame->out_flag)) {
		mserr_hw("invalid out_flag [F:%d][0x%x][B:0x%lx][O:0x%lx]",
			frame->instance, hw_ip, frame->fcount, req_id,
			frame->bak_flag, frame->out_flag);
		goto free_frame;
	}

	work_wq   = &hw_ip->itf->work_wq[WORK_SHOT_DONE];
	work_list = &hw_ip->itf->work_list[WORK_SHOT_DONE];

	get_free_work(work_list, &work);
	if (work) {
		work->msg.id		= 0;
		work->msg.command	= IS_FRAME_DONE;
		work->msg.instance	= frame->instance;
		work->msg.group		= GROUP_ID(head->id);
		work->msg.param1	= frame->fcount;
		work->msg.param2	= done_type; /* status: enum ShotErrorType */
		work->msg.param3	= frame->cur_shot_idx;
		work->msg.param4	= 0;

		work->fcount = work->msg.param1;
		set_req_work(work_list, work);

		if (!work_pending(work_wq))
			wq_func_schedule(hw_ip->itf, work_wq);
	} else {
		mserr_hw("free work item is empty\n", frame->instance, hw_ip);

		/* WQ debug info */
		print_fre_work_list(work_list);
		print_req_work_list(work_list);
	}
	clear_bit(req_id, &frame->out_flag);

free_frame:
	if (done_type) {
		msinfo_hw("SHOT_NDONE [E%d][F:%d]\n", frame->instance, hw_ip,
			done_type, frame->fcount);
		goto exit;
	}

	if (frame->type == SHOT_TYPE_INTERNAL) {
		msdbg_hw(1, "INTERNAL_SHOT_DONE [F:%d]\n",
			frame->instance, hw_ip, frame->fcount);
		atomic_inc(&hw_ip->hardware->log_count);
	} else if (frame->type == SHOT_TYPE_MULTI) {
		if (!test_bit(IS_GROUP_OTF_INPUT, &head->state)) {
			struct is_hw_ip *hw_ip_ldr = is_get_hw_ip(head->id, hw_ip->hardware);

			if (hw_ip_ldr)
				mshot_schedule(hw_ip_ldr);

			msdbg_hw(1, "SHOT_TYPE_MULTI [F:%d]\n",
				frame->instance, hw_ip, frame->fcount);
		}
	} else {
		msdbg_hw(1, "SHOT_DONE [F:%d]\n",
			frame->instance, hw_ip, frame->fcount);
		atomic_set(&hw_ip->hardware->log_count, 0);
	}
exit:
	framemgr_e_barrier_common(framemgr, 0, flags);
	trans_frame(framemgr, frame, FS_HW_FREE);
	framemgr_x_barrier_common(framemgr, 0, flags);
	atomic_set(&frame->shot_done_flag, 0);

	if (framemgr->queued_count[FS_HW_FREE] > 10)
		atomic_set(&hw_ip->hardware->bug_count, 0);

	return ret;
}

int is_hardware_frame_done(struct is_hw_ip *hw_ip, struct is_frame *frame,
	int wq_id, u32 output_id, enum ShotErrorType done_type, bool get_meta)
{
	int ret = 0;
	struct is_framemgr *framemgr;
	struct is_group *group, *head;
	ulong flags = 0;
	u32 hw_fe_cnt = 0;
	struct is_subdev *subdev;
	struct is_hw_ip *ldr_hw_ip;
	u32 instance;

	FIMC_BUG(!hw_ip);

	framemgr = hw_ip->framemgr;

	switch (done_type) {
	case IS_SHOT_SUCCESS:
		if (frame == NULL) {
			framemgr_e_barrier_common(framemgr, 0, flags);
			frame = peek_frame(framemgr, FS_HW_WAIT_DONE);
			framemgr_x_barrier_common(framemgr, 0, flags);
		} else {
			sdbg_hw(2, "frame NOT null!!(%d)", hw_ip, done_type);
		}

		if (IS_RUNNING_TUNING_SYSTEM() && frame)
			pablo_obte_regdump(frame->instance, hw_ip->id,
					frame->stripe_info.region_id,
					frame->stripe_info.region_num);

		break;
	case IS_SHOT_LATE_FRAME:
	case IS_SHOT_UNPROCESSED:
	case IS_SHOT_OVERFLOW:
	case IS_SHOT_INVALID_FRAMENUMBER:
	case IS_SHOT_TIMEOUT:
	case IS_SHOT_CONFIG_LOCK_DELAY:
	case IS_SHOT_DROP:
		break;
	default:
		serr_hw("invalid done_type(%d)", hw_ip, done_type);
		return -EINVAL;
	}

	hw_fe_cnt = atomic_read(&hw_ip->fcount);

	if (frame == NULL) {
		serr_hw("[F:%d]frame_done: frame(null)!!(%d)(0x%x)", hw_ip,
			hw_fe_cnt, done_type, output_id);
		framemgr_e_barrier_common(framemgr, 0, flags);
		frame_manager_print_info_queues(framemgr);
		print_all_hw_frame_count(hw_ip->hardware);
		framemgr_x_barrier_common(framemgr, 0, flags);
		return -EINVAL;
	}

	instance = frame->instance;
	group = hw_ip->group[instance];
	if (!group) {
		sdbg_hw(2, "hw_ip->group[%d] is NULL, use hw_ip->instance(%d)",
				hw_ip, instance, atomic_read(&hw_ip->instance));

		instance = atomic_read(&hw_ip->instance);
		group = hw_ip->group[instance];

		if (!group) {
			mserr_hw("group is null", instance, hw_ip);
			return -EINVAL;
		}
	}
	head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);

	msdbgs_hw(2, "[0x%x]frame_done [F:%d][HWF:%d][B:0x%lx][C:0x%lx][O:0x%lx]\n",
		frame->instance, hw_ip, output_id, frame->fcount, hw_fe_cnt,
		frame->bak_flag, frame->core_flag, frame->out_flag);

	/* check core_done */
	if (output_id == IS_HW_CORE_END) {
		if (!test_bit_variables(hw_ip->id, &frame->core_flag)) {
			msinfo_hw("invalid core_flag [F:%d][0x%x][C:0x%lx][O:0x%lx]",
				instance, hw_ip, frame->fcount,
				output_id, frame->core_flag, frame->out_flag);
			goto shot_done;
		}

		if (hw_ip->is_leader) {
			frame->frame_info[INFO_FRAME_END_PROC].cpu = raw_smp_processor_id();
			frame->frame_info[INFO_FRAME_END_PROC].pid = current->pid;
			frame->frame_info[INFO_FRAME_END_PROC].when = local_clock();
		}

		if (frame->shot && get_meta)
			is_hardware_get_meta(hw_ip, frame,
				instance, hw_ip->hardware->hw_map[instance],
				output_id, done_type);
	} else {
		u32 output_cnt = 0;

		list_for_each_entry(subdev, &group->subdev_list, list) {
			if (test_bit_variables(subdev->id, &frame->out_flag))
				output_cnt++;
		}

		if (!output_cnt) {
			msinfo_hw("invalid output_id [F:%d][C:0x%lx][O:0x%lx]",
				instance, hw_ip, frame->fcount,
				frame->core_flag, frame->out_flag);
			goto shot_done;
		}

		if (frame->shot && get_meta)
			is_hardware_get_meta(hw_ip, frame,
				instance, hw_ip->hardware->hw_map[instance],
				0, done_type);

		list_for_each_entry(subdev, &group->subdev_list, list) {
			if (test_bit_variables(subdev->id, &frame->out_flag)) {
				if (subdev->id == head->leader.id)
					continue;

				clear_bit(subdev->id, &frame->out_flag);

				if (frame->type == SHOT_TYPE_MULTI
					|| frame->type == SHOT_TYPE_INTERNAL)
					continue;

				ret = do_frame_done_work_func(hw_ip->itf,
						subdev->wq_id,
						instance,
						head->id,
						frame->fcount,
						frame->rcount,
						done_type);
				if (ret)
					FIMC_BUG(1);
			}
		}
	}

shot_done:
	if (output_id == IS_HW_CORE_END) {
		clear_bit(hw_ip->id, &frame->core_flag);

		if (atomic_read(&hw_ip->status.Vvalid) == V_BLANK)
			msdbg_hw(1, "[F%d][HF%d] already in VBlank", frame->instance, hw_ip,
					frame->fcount, hw_fe_cnt);
		else
			atomic_set(&hw_ip->status.Vvalid, V_BLANK);
	}

	framemgr_e_barrier_common(framemgr, 0, flags);
	if (!OUT_FLAG(frame->out_flag, head->leader.id)
		&& !frame->core_flag
		&& atomic_dec_and_test(&frame->shot_done_flag)) {
		framemgr_x_barrier_common(framemgr, 0, flags);
		ret = is_hardware_shot_done(hw_ip, frame, framemgr, done_type);

		if (!test_bit(IS_GROUP_OTF_INPUT, &head->state)) {
			ldr_hw_ip = is_get_hw_ip(head->id, hw_ip->hardware);
			if (ldr_hw_ip)
				del_timer(&ldr_hw_ip->shot_timer);
		}

		return ret;
	}
	framemgr_x_barrier_common(framemgr, 0, flags);

	return ret;
}

static int is_hardware_frame_ndone(struct is_hw_ip *ldr_hw_ip,
	struct is_frame *frame, u32 instance,
	enum ShotErrorType done_type)
{
	int ret = 0;
	struct is_hw_ip *hw_ip = NULL;
	struct is_group *group = NULL;
	struct is_group *chain_head = NULL;
	struct is_group *sensor_head = NULL;
	struct is_group *child = NULL;
	struct is_group *pnext = NULL;
	struct is_hardware *hardware;
	enum is_hardware_id hw_id = DEV_HW_END;
	int hw_list[GROUP_HW_MAX], hw_index;
	int hw_maxnum = 0;

	if (!frame) {
		mserr_hw("ndone frame is NULL(%d)", instance, ldr_hw_ip, done_type);
		return -EINVAL;
	} else {
		msinfo_hw("%s[F:%d][E%d][O:0x%lx][C:0x%lx]\n",
				instance, ldr_hw_ip, __func__,
				frame->fcount, done_type,
				frame->out_flag, frame->core_flag);
	}

	group = ldr_hw_ip->group[instance];
	if (!group) {
		mserr_hw("group is NULL", instance, ldr_hw_ip);
		return -EINVAL;
	}

	chain_head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);
	if (!chain_head) {
		mserr_hw("head is NULL(G%d)", instance, ldr_hw_ip, group->id);
		return -EINVAL;
	}
	sensor_head = chain_head->head;
	hardware = ldr_hw_ip->hardware;

	/* SFR dump */
	child = chain_head;
	pnext = sensor_head->pnext;
	while (child) {
		hw_maxnum = is_get_hw_list(child->id, hw_list);
		for (hw_index = 0; hw_index < hw_maxnum; hw_index++) {
			hw_id = hw_list[hw_index];
			hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_id);
			if (!hw_ip) {
				err_hw("hw_ip is NULL");
				continue;
			}

			if (done_type == IS_SHOT_TIMEOUT) {
				is_hardware_sfr_dump(hardware, hw_id, false);
				ret = CALL_HWIP_OPS(hw_ip, notify_timeout, child->instance);
				if (ret) {
					mserr_hw("notify_timeout fail", child->instance, hw_ip);
					break;
				}
			}
		}
		child = child->child;

		if (pnext && !child) {
			child = pnext;
			pnext = NULL;
		}
	}
#if defined(HW_TIMEOUT_PANIC_ENABLE)
	if (CHECK_TIMEOUT_PANIC_HW(ldr_hw_ip->id) && (done_type == IS_SHOT_TIMEOUT))
		is_debug_s2d(true, "IS_SHOT_TIMEOUT");
#endif

	/* if there is not any out_flag without leader, forcely set the core flag */
	if (!OUT_FLAG(frame->out_flag, chain_head->leader.id))
		set_bit(ldr_hw_ip->id, &frame->core_flag);

	/* Not done */
	child = chain_head;
	pnext = sensor_head->pnext;
	while (child) {
		hw_maxnum = is_get_hw_list(child->id, hw_list);
		for (hw_index = 0; hw_index < hw_maxnum; hw_index++) {
			hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
			if (!hw_ip) {
				merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
				return -EINVAL;
			}

			ret = CALL_HWIP_OPS(hw_ip, frame_ndone, frame, child->instance, done_type);
			if (ret) {
				mserr_hw("frame_ndone fail", child->instance, hw_ip);
				return -EINVAL;
			}
		}
		child = child->child;

		if (pnext && !child) {
			child = pnext;
			pnext = NULL;
		}
	}

	return ret;
}

static int parse_setfile_header(ulong addr, struct is_setfile_header *header)
{
	union __setfile_header *file_header;

	/* 1. check setfile version */
	/* 2. load version specific header information */
	file_header = (union __setfile_header *)addr;
	if (file_header->magic_number == (SET_FILE_MAGIC_NUMBER - 1)) {
		header->version = SETFILE_V2;

		header->num_ips = file_header->ver_2.subip_num;
		header->num_scenarios = file_header->ver_2.scenario_num;

		header->scenario_table_base = addr + sizeof(struct __setfile_header_ver_2);
		header->setfile_entries_base = addr + file_header->ver_2.setfile_offset;

		header->designed_bits = 0;
		memset(header->version_code, 0, 5);
		memset(header->revision_code, 0, 5);
	} else if (file_header->magic_number == SET_FILE_MAGIC_NUMBER) {
		header->version = SETFILE_V3;

		header->num_ips = file_header->ver_3.subip_num;
		header->num_scenarios = file_header->ver_3.scenario_num;

		header->scenario_table_base = addr + sizeof(struct __setfile_header_ver_3);
		header->setfile_entries_base = addr + file_header->ver_3.setfile_offset;

		header->designed_bits = file_header->ver_3.designed_bit;
		memcpy(header->version_code, file_header->ver_3.version_code, 4);
		header->version_code[4] = 0;
		memcpy(header->revision_code, file_header->ver_3.revision_code, 4);
		header->revision_code[4] = 0;
	} else {
		err_hw("invalid magic number[0x%08x]", file_header->magic_number);
		return -EINVAL;
	}

	/* 3. process more header information */
	header->num_setfile_base = header->scenario_table_base
		+ (header->num_ips * header->num_scenarios * sizeof(u32));
	header->setfile_table_base = header->num_setfile_base
		+ (header->num_ips * sizeof(u32));

	info_hw("%s: version(%d)(%s)\n", __func__, header->version, header->revision_code);

	dbg_hw(1, "%s: number of IPs: %d\n", __func__, header->num_ips);
	dbg_hw(1, "%s: number of scenario: %d\n", __func__, header->num_scenarios);
	dbg_hw(1, "%s: scenario table base: 0x%lx\n", __func__, header->scenario_table_base);
	dbg_hw(1, "%s: number of setfile base: 0x%lx\n", __func__, header->num_setfile_base);
	dbg_hw(1, "%s: setfile table base: 0x%lx\n", __func__, header->setfile_table_base);
	dbg_hw(1, "%s: setfile entries base: 0x%lx\n", __func__, header->setfile_entries_base);

	return 0;
}

static int parse_setfile_info(struct is_hw_ip *hw_ip,
	struct is_setfile_header header,
	u32 instance,
	u32 num_ips,
	struct __setfile_table_entry *setfile_table_entry)
{
	unsigned long base;
	size_t blk_size;
	u32 idx;
	struct is_hw_ip_setfile *setfile;
	enum exynos_sensor_position sensor_position;

	sensor_position = hw_ip->hardware->sensor_position[instance];
	setfile = &hw_ip->setfile[sensor_position];

	/* skip setfile parsing if it alreay parsed at each sensor position */
	if (setfile->using_count > 0)
		return 0;

	/* set version */
	setfile->version = header.version;

	/* set what setfile index is used at each scenario */
	base = header.scenario_table_base;
	blk_size = header.num_scenarios * sizeof(u32);
	memcpy(setfile->index, (void *)(base + (num_ips * blk_size)), blk_size);

	/* fill out-of-range index for each not-used scenario to check sanity */
	memset((u32 *)&setfile->index[header.num_scenarios],
		0xff, (IS_MAX_SCENARIO - header.num_scenarios) * sizeof(u32));
	for (idx = 0; idx < header.num_scenarios; idx++)
		msdbg_hw(1, "scenario table [%d:%d]\n", instance, hw_ip,
			idx, setfile->index[idx]);

	/* set the number of setfile at each sub IP */
	base = header.num_setfile_base;
	blk_size = sizeof(u32);
	setfile->using_count = (u32)*(ulong *)(base + (num_ips * blk_size));

	if (setfile->using_count > IS_MAX_SETFILE) {
		mserr_hw("too many setfile entries: %d", instance, hw_ip, setfile->using_count);
		return -EINVAL;
	}

	msdbg_hw(1, "number of setfile: %d\n", instance, hw_ip, setfile->using_count);

	/* set each setfile address and size */
	for (idx = 0; idx < setfile->using_count; idx++) {
		setfile->table[idx].addr =
			(ulong)(header.setfile_entries_base + setfile_table_entry[idx].offset),
		setfile->table[idx].size = setfile_table_entry[idx].size;

		msdbg_hw(1, "setfile[%d] addr: 0x%lx, size: %x\n",
			instance, hw_ip, idx,
			setfile->table[idx].addr,
			setfile->table[idx].size);
	}

	return 0;
}

static void set_hw_slots_bit(struct is_hardware *hw, unsigned long *slots, int nslots, int hw_id)
{
	int hw_slot;

	switch (hw_id) {
	/* setfile chain (3AA0, 3AA1, ISP0, ISP1) */
	case DEV_HW_3AA0:
		hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, hw_id);
		if (valid_hw_slot_id(hw_slot))
			set_bit(hw_slot, slots);
		hw_id = DEV_HW_3AA1;
		fallthrough;
	case DEV_HW_3AA1:
		hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, hw_id);
		if (valid_hw_slot_id(hw_slot))
			set_bit(hw_slot, slots);
		hw_id = DEV_HW_3AA2;
		fallthrough;
	case DEV_HW_3AA2:
		hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, hw_id);
		if (valid_hw_slot_id(hw_slot))
			set_bit(hw_slot, slots);
		hw_id = DEV_HW_3AA3;
		fallthrough;
	case DEV_HW_3AA3:
		hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, hw_id);
		if (valid_hw_slot_id(hw_slot))
			set_bit(hw_slot, slots);
		hw_id = DEV_HW_ISP0_ID;
		fallthrough;
	case DEV_HW_ISP0_ID:
		hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, hw_id);
		if (valid_hw_slot_id(hw_slot))
			set_bit(hw_slot, slots);
		hw_id = DEV_HW_ISP1;
		break;
	/* setfile chain (MCSC0, MCSC1) */
	case DEV_HW_MCSC0:
		hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, hw_id);
		if (valid_hw_slot_id(hw_slot))
			set_bit(hw_slot, slots);
		hw_id = DEV_HW_MCSC1;
		break;

	/* setfile chain (LME0, LME1) */
	case DEV_HW_LME0:
		hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, hw_id);
		if (valid_hw_slot_id(hw_slot))
			set_bit(hw_slot, slots);
		hw_id = DEV_HW_LME1;
		break;
	/* setfile chain (ORB0, ORB1) */
	case DEV_HW_ORB0:
		hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, hw_id);
		if (valid_hw_slot_id(hw_slot))
			set_bit(hw_slot, slots);
		hw_id = DEV_HW_ORB1;
		break;
	}

	switch (hw_id) {
	/* every leaf of each setfile chain */
	case DEV_HW_ISP1:
	case DEV_HW_VRA:
	case DEV_HW_MCSC1:
	case DEV_HW_CLH0:
	case DEV_HW_LME1:
	case DEV_HW_ORB1:
		hw_slot = CALL_HW_CHAIN_INFO_OPS(hw, get_hw_slot_id, hw_id);
		if (valid_hw_slot_id(hw_slot))
			set_bit(hw_slot, slots);
		break;
	}
}

static void get_setfile_hw_slots_no_hint(struct is_hardware *hw, unsigned long *slots, int ip, u32 num_ips)
{
	int hw_id = 0;

	bitmap_zero(slots, HW_SLOT_MAX);

	if (num_ips == 3) {
		/* ISP, DRC, VRA */
		switch (ip) {
		case 0:
			hw_id = DEV_HW_3AA0;
			break;
		case 2:
			hw_id = DEV_HW_VRA;
			break;
		}
	} else if (num_ips == 4) {
		/* ISP, DRC, TDNR, VRA */
		switch (ip) {
		case 0:
			hw_id = DEV_HW_3AA0;
			break;
		case 3:
			hw_id = DEV_HW_VRA;
			break;
		}
	} else if (num_ips == 5) {
		/* ISP, DRC, DIS, TDNR, VRA */
		switch (ip) {
		case 0:
			hw_id = DEV_HW_3AA0;
			break;
		case 4:
			hw_id = DEV_HW_VRA;
			break;
		}
	} else if (num_ips == 6) {
		/* ISP, DRC, DIS, TDNR, MCSC, VRA */
		switch (ip) {
		case 0:
			hw_id = DEV_HW_3AA0;
			break;
		case 4:
			hw_id = DEV_HW_MCSC0;
			break;
		case 5:
			hw_id = DEV_HW_VRA;
			break;
		}
	}

	dbg_hw(1, "%s: hw_id: %d, IP: %d, number of IPs: %d\n", __func__, hw_id, ip, num_ips);

	if (hw_id > 0)
		set_hw_slots_bit(hw, slots, HW_SLOT_MAX, hw_id);
}

static void get_setfile_hw_slots(struct is_hardware *hw, unsigned long *slots, unsigned long *hint)
{
	dbg_hw(1, "%s: designed bits(0x%lx) ", __func__, *hint);

	bitmap_zero(slots, HW_SLOT_MAX);

	if (test_and_clear_bit(SETFILE_DESIGN_BIT_3AA_ISP, hint)) {
		set_hw_slots_bit(hw, slots, HW_SLOT_MAX, DEV_HW_3AA0);

	} else if (test_and_clear_bit(SETFILE_DESIGN_BIT_DRC, hint)) {
		/* deprecated */
		/* set_hw_slots_bit(slots, HW_SLOT_MAX, DEV_HW_DRC); */

	} else if (test_and_clear_bit(SETFILE_DESIGN_BIT_SCC, hint)) {
		/* not supported yet */
		/* set_hw_slots_bit(slots, HW_SLOT_MAX, DEV_HW_SCC); */

	} else if (test_and_clear_bit(SETFILE_DESIGN_BIT_ODC, hint)) {
		/* not supported yet */
		/* set_hw_slots_bit(slots, HW_SLOT_MAX, DEV_HW_ODC); */

	} else if (test_and_clear_bit(SETFILE_DESIGN_BIT_VDIS, hint)) {
		/* deprecated */
		/* set_hw_slots_bit(slots, HW_SLOT_MAX, DEV_HW_DIS); */

	} else if (test_and_clear_bit(SETFILE_DESIGN_BIT_TDNR, hint)) {
		/* deprecated */
		/* set_hw_slots_bit(slots, HW_SLOT_MAX, DEV_HW_3DNR); */

	} else if (test_and_clear_bit(SETFILE_DESIGN_BIT_CLAHE, hint)) {
		/* deprecated */
		/* set_hw_slots_bit(slots, HW_SLOT_MAX, DEV_HW_CLH0); */

	} else if (test_and_clear_bit(SETFILE_DESIGN_BIT_SCX_MCSC, hint)) {
		set_hw_slots_bit(hw, slots, HW_SLOT_MAX, DEV_HW_MCSC0);

	} else if (test_and_clear_bit(SETFILE_DESIGN_BIT_FD_VRA, hint)) {
		set_hw_slots_bit(hw, slots, HW_SLOT_MAX, DEV_HW_VRA);

	}

	dbg_hw(1, "              -> (0x%lx)\n", *hint);
}

int is_hardware_load_setfile(struct is_hardware *hardware, ulong addr,
	u32 instance, ulong hw_map)
{
	struct is_setfile_header header;
	struct __setfile_table_entry *setfile_table_entry;
	unsigned long slots[DIV_ROUND_UP(HW_SLOT_MAX, BITS_PER_LONG)];
	struct is_hw_ip *hw_ip;
	unsigned long hw_slot;
	unsigned long hint;
	u32 ip;
	ulong setfile_table_idx = 0;
	int ret = 0;
	enum exynos_sensor_position sensor_position;

	ret = parse_setfile_header(addr, &header);
	if (ret) {
		merr_hw("failed to parse setfile header(%d)", instance, ret);
		return ret;
	}

	if (header.num_scenarios > IS_MAX_SCENARIO) {
		merr_hw("too many scenarios: %d", instance, header.num_scenarios);
		return -EINVAL;
	}

	hint = header.designed_bits;
	setfile_table_entry = (struct __setfile_table_entry *)header.setfile_table_base;
	sensor_position = hardware->sensor_position[instance];

	for (ip = 0; ip < header.num_ips; ip++) {
		if (header.version == SETFILE_V3)
			get_setfile_hw_slots(hardware, slots, &hint);
		else
			get_setfile_hw_slots_no_hint(hardware, slots, ip, header.num_ips);

		hw_ip = NULL;

		hw_slot = find_first_bit(slots, HW_SLOT_MAX);

		if (hw_slot == HW_SLOT_MAX) {
			unsigned long base = header.num_setfile_base;
			size_t blk_size = sizeof(u32);

			setfile_table_idx = (u32)*(ulong *)(base + (ip * blk_size));
			setfile_table_entry += setfile_table_idx;

			mdbg_hw(1, "skip parsing at not supported ip.\n", instance);

			continue;
		}

		while (hw_slot < HW_SLOT_MAX) {
			hw_ip = &hardware->hw_ip[hw_slot];

			clear_bit(hw_slot, slots);
			hw_slot = find_first_bit(slots, HW_SLOT_MAX);

			if (!test_bit(hw_ip->id, &hardware->hw_map[instance])) {
				msdbg_hw(1, "skip parsing at not mapped hw_ip", instance, hw_ip);
				if (hw_slot) {
					unsigned long base = header.num_setfile_base;
					size_t blk_size = sizeof(u32);

					setfile_table_idx = (u32)*(ulong *)(base + (ip * blk_size));
				}
				continue;
			}

			ret = parse_setfile_info(hw_ip, header, instance, ip, setfile_table_entry);
			if (ret) {
				mserr_hw("parse setfile info failed\n", instance, hw_ip);
				return ret;
			}

			ret = CALL_HWIP_OPS(hw_ip, load_setfile, instance, hw_map);
			if (ret) {
				mserr_hw("failed to load setfile(%d)", instance, hw_ip, ret);
				return ret;
			}

			/* set setfile table idx for next setfile_table base */
			setfile_table_idx = (ulong)hw_ip->setfile[sensor_position].using_count;
		}

		/* increase setfile table base even though there is no valid HW slot */
		if (hw_ip)
			setfile_table_entry += setfile_table_idx;
		else
			setfile_table_entry++;
	}

	return ret;
};

int is_hardware_apply_setfile(struct is_hardware *hardware, u32 instance,
	u32 scenario, ulong hw_map)
{
	struct is_hw_ip *hw_ip = NULL;
	int hw_id = 0;
	int ret = 0;
	int hw_slot = -1;
	enum exynos_sensor_position sensor_position;

	FIMC_BUG(!hardware);

	if (IS_MAX_SCENARIO <= scenario) {
		merr_hw("invalid scenario id: scenario(%d)", instance, scenario);
		return -EINVAL;
	}

	minfo_hw("apply_setfile: hw_map (0x%lx)\n", instance, hw_map);

	for (hw_slot = 0; hw_slot < HW_SLOT_MAX; hw_slot++) {
		hw_ip = &hardware->hw_ip[hw_slot];
		hw_id = hw_ip->id;
		if(!test_bit(hw_id, &hardware->hw_map[instance]))
			continue;

		ret = CALL_HWIP_OPS(hw_ip, apply_setfile, scenario, instance, hw_map);
		if (ret) {
			mserr_hw("apply_setfile fail (%d)", instance, hw_ip, ret);
			return -EINVAL;
		}

		sensor_position = hardware->sensor_position[instance];
		hw_ip->applied_scenario = scenario;
	}

	return ret;
}

int is_hardware_delete_setfile(struct is_hardware *hardware, u32 instance,
	ulong hw_map)
{
	int ret = 0;
	int hw_slot;
	struct is_hw_ip *hw_ip;

	FIMC_BUG(!hardware);

	minfo_hw("delete_setfile: hw_map (0x%lx)\n", instance, hw_map);
	for (hw_slot = 0; hw_slot < HW_SLOT_MAX; hw_slot++) {
		hw_ip = &hardware->hw_ip[hw_slot];

		if (!test_bit_variables(hw_ip->id, &hw_map))
			continue;

		ret = CALL_HWIP_OPS(hw_ip, delete_setfile, instance, hw_map);
		if (ret) {
			mserr_hw("delete_setfile fail", instance, hw_ip);
			return -EINVAL;
		}
	}

	return ret;
}

int is_hardware_runtime_resume(struct is_hardware *hardware)
{
	int ret = 0;

	return ret;
}

int is_hardware_runtime_suspend(struct is_hardware *hardware)
{
	return 0;
}

int is_hardware_capture_meta_request(struct is_hardware *hardware,
	struct is_group *group, u32 instance, u32 fcount, u32 size, ulong addr)
{
	int ret;
	struct is_hw_ip *hw_ip;
	int hw_list[GROUP_HW_MAX], hw_index;
	int hw_maxnum;
	ulong hw_map;

	hw_map = hardware->hw_map[instance];
	hw_maxnum = is_get_hw_list(group->id, hw_list);
	for (hw_index = hw_maxnum - 1; hw_index >= 0; hw_index--) {
		hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
		if (!hw_ip) {
			merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
			return -EINVAL;
		}

		ret = CALL_HWIP_OPS(hw_ip, get_cap_meta, hw_map, instance, fcount, size, addr);
		if (ret) {
			mserr_hw("get_cap_meta fail (%d)", instance, hw_ip, ret);
			return -EINVAL;
		}
	}

	return 0;
}

void is_hardware_sfr_dump(struct is_hardware *hardware, u32 hw_id, bool flag_print_log)
{
	int hw_slot = -1;
	struct is_hw_ip *hw_ip = NULL;

	if (!hardware) {
		err_hw("hardware is null\n");
		return;
	}

	if (hw_id == DEV_HW_END) {
		for (hw_slot = 0; hw_slot < HW_SLOT_MAX; hw_slot++) {
			hw_ip = &hardware->hw_ip[hw_slot];
			_is_hardware_sfr_dump(hw_ip, flag_print_log);
		}
	} else {
		hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_id);
		if (hw_ip)
			_is_hardware_sfr_dump(hw_ip, flag_print_log);
	}
}

int is_hardware_flush_frame_by_group(struct is_hardware *hardware,
	struct is_group *group, u32 instance)
{
	struct is_hw_ip *hw_ip = NULL;
	int hw_list[GROUP_HW_MAX];

	is_get_hw_list(group->id, hw_list);
	hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[0]);
	if (!hw_ip) {
		merr_hw("invalid id (%d)", instance, hw_list[0]);
		return -EINVAL;
	}

	msdbg_hw(1, "flush_frame by group(%d)\n", instance, hw_ip, group->id);
	is_hardware_flush_frame(hw_ip, FS_HW_REQUEST, IS_SHOT_UNPROCESSED);

	return 0;
}

int is_hardware_restore_by_group(struct is_hardware *hardware,
	struct is_group *group, u32 instance)
{
	int ret = 0;
	struct is_hw_ip *hw_ip = NULL;
	int hw_list[GROUP_HW_MAX], hw_index;
	int hw_maxnum = 0;

	hw_maxnum = is_get_hw_list(group->id, hw_list);
	for (hw_index = hw_maxnum - 1; hw_index >= 0; hw_index--) {
		hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[hw_index]);
		if (!hw_ip) {
			merr_hw("invalid id (%d)", instance, hw_list[hw_index]);
			return -EINVAL;
		}

		ret = CALL_HWIP_OPS(hw_ip, restore, instance);
		if (ret) {
			mserr_hw("reset & restore fail = %x", instance, hw_ip, ret);
			goto exit;
		}

		if (is_votf_check_link(group)) {
			if (CALL_HW_CHAIN_INFO_OPS(hw_ip->hardware, set_votf_reset))
				mserr_hw("votf reset fail", instance, hw_ip);

			is_votf_destroy_link(group);
			is_votf_create_link(group, 0, 0);
		}
	}

exit:
	return ret;
}

int is_hardware_recovery_shot(struct is_hardware *hardware, u32 instance,
	struct is_group *group, u32 recov_fcount, struct is_framemgr *framemgr)
{
	int ret = 0;
	struct is_frame *frame = NULL;
	struct is_hw_ip *hw_ip = NULL;
	int hw_list[GROUP_HW_MAX];
	ulong flags = 0;

	is_get_hw_list(group->id, hw_list);
	hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_list[0]);
	if (!hw_ip) {
		merr_hw("invalid id (%d)", instance, hw_list[0]);
		return -EINVAL;
	}

	framemgr_e_barrier_common(framemgr, 0, flags);
	if (!framemgr->queued_count[FS_HW_REQUEST]) {
		ret = make_internal_shot(hw_ip, instance, recov_fcount, framemgr, 0);
		if (ret) {
			framemgr_x_barrier_common(framemgr, 0, flags);
			goto exit;
		}
	}

	frame = peek_frame(framemgr, FS_HW_REQUEST);
	framemgr_x_barrier_common(framemgr, 0, flags);

	if (!frame) {
		mserr_hw("Failed to get REQUEST frame", instance, hw_ip);
		return -EINVAL;
	}

	ret = is_hardware_shot(hardware, instance, hw_ip->group[instance],
			frame, framemgr, hardware->hw_map[instance], frame->fcount);

exit:
	return ret;
}

void is_hardware_dma_cfg_kva64(char *name, struct is_hw_ip *hw_ip,
			struct is_frame *frame, int cur_idx,
			u32 *cmd, u32 plane,
			u64 *dst_kva, u64 *src_kva)
{
	int buf_i, plane_i, src_kva_i, dst_kva_i, src_p_i, dst_p_i;
	int level;
	u32 i;

	if (!(*cmd)) {
		for (i = 0; i < IS_MAX_PLANES; i++)
			dst_kva[i] = 0;
		return;
	}

	/* Iterator for single buffer with HW_FRO */
	for (buf_i = 0; buf_i < frame->num_buffers; buf_i++) {
		/* Single buffer DVA idx based on cur_idx in src_kva array */
		src_kva_i = (cur_idx + buf_i) * plane;
		dst_kva_i = buf_i * plane;

		if (!src_kva[src_kva_i]) {
			*cmd = 0; /* COMMAND_DISABLE */
			level = (src_kva_i == 0) ? 0 : 2;
			msdbg_hw(level, "[F:%d]%s_src_kva[%d] is zero\n",
				frame->instance, hw_ip,
				frame->fcount, name, src_kva_i);

			continue;
		}

		for (plane_i = 0; plane_i < plane; plane_i++) {
			src_p_i = src_kva_i + plane_i;
			dst_p_i = dst_kva_i + plane_i;
			dst_kva[dst_p_i] = src_kva[src_p_i];

			msdbg_hw(2, "[F:%d]%s_src_kva[%d] is 0x%lx\n",
				frame->instance, hw_ip,
				frame->fcount, name, plane_i, src_kva[src_p_i]);
		}
	}
}
PST_EXPORT_SYMBOL(is_hardware_dma_cfg_kva64);

u32 is_hardware_dma_cfg(char *name, struct is_hw_ip *hw_ip,
			struct is_frame *frame, int cur_idx, u32 num_buffers,
			u32 *cmd, u32 plane,
			pdma_addr_t *dst_dva, dma_addr_t *src_dva)
{
	int buf_i, plane_i, src_dva_i, dst_dva_i, src_p_i, dst_p_i;
	u32 org_cmd = *cmd;
	int level = 0;
	u32 i;

	if (!org_cmd) {
		for (i = 0; i < IS_MAX_PLANES; i++)
			dst_dva[i] = 0;
		goto exit;
	}

	msdbg_hw(3, "dma_cfg of %s, num plane %d\n", 0, hw_ip, name, plane);

	if (!plane) {
		mswarn_hw("plane is zero", frame->instance, hw_ip);
		*cmd = 0; /* COMMAND_DISABLE */
		goto exit;
	}

	/* Iterator for single buffer with HW_FRO */
	for (buf_i = 0; buf_i < frame->num_buffers; buf_i++) {
		/* Single buffer DVA idx based on cur_idx in src_dva array */
		src_dva_i = (cur_idx + (buf_i % num_buffers)) * plane;
		dst_dva_i = buf_i * plane;

		if (!src_dva[src_dva_i]) {
			*cmd = 0; /* COMMAND_DISABLE */
			level = (src_dva_i == 0) ? 0 : 2;
			msdbg_hw(level, "[F:%d]%s_src_dva[%d] is zero\n",
				frame->instance, hw_ip,
				frame->fcount, name, src_dva_i);

			continue;
		}

		for (plane_i = 0; plane_i < plane; plane_i++) {
			src_p_i = src_dva_i + plane_i;
			dst_p_i = dst_dva_i + plane_i;
			dst_dva[dst_p_i] = (pdma_addr_t) src_dva[src_p_i];
		}
	}

exit:
	return org_cmd;
}
PST_EXPORT_SYMBOL(is_hardware_dma_cfg);

int is_hardware_get_offline_data(struct is_hardware *hardware, u32 instance,
	struct is_group *group, void *data_desc, int fcount)
{
	struct is_hw_ip *hw_ip;
	struct is_group *head, *child;
	enum is_hardware_id hw_id;
	int hw_list[GROUP_HW_MAX], hw_maxnum, hw_index;
	int ret = 0;

	head = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);

	child = head->tail;
	while (child && (child->device_type == IS_DEVICE_ISCHAIN)) {
		hw_maxnum = is_get_hw_list(child->id, hw_list);
		for (hw_index = hw_maxnum - 1; hw_index >= 0; hw_index--) {
			hw_id = hw_list[hw_index];
			if (hw_id >= DEV_HW_3AA0 && hw_id <= DEV_HW_3AA3) {
				hw_ip =  CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_id);
				if (hw_ip)
					goto get_3AA_slot;
				else
					merr_hw("invalid hw_id (%d)", instance, hw_id);
			}
		}
		child = child->parent;
	}

	merr_hw("There is no 3AA slot", instance);

	return -EINVAL;

get_3AA_slot:
	ret = CALL_HWIP_OPS(hw_ip, get_offline_data, instance, data_desc, fcount);
	if (ret)
		merr_hw("fail to get offline_data (%d)", instance, hw_id);

	return ret;
}

void is_hardware_debug_otf(struct is_hardware *hardware, struct is_group *group)
{
	struct is_group *child;
	struct is_hw_ip *hw_ip_paf = NULL;
	struct is_hw_ip *hw_ip_3aa = NULL;

	child = GET_HEAD_GROUP_IN_DEVICE(IS_DEVICE_ISCHAIN, group);
	while (child) {
		if (child->slot == GROUP_SLOT_PAF)
			hw_ip_paf = is_get_hw_ip(child->id, hardware);
		else if (child->slot == GROUP_SLOT_3AA)
			hw_ip_3aa = is_get_hw_ip(child->id, hardware);

		child = child->child;
	}

	if ((hw_ip_paf && hw_ip_3aa)
			&& atomic_read(&hw_ip_3aa->count.fe) > atomic_read(&hw_ip_paf->count.fe)) {
		err_hw("hw_paf(%d) fe_cnt(%d) < hw_3aa(%d) fe_cnt(%d)",
				hw_ip_paf->id, atomic_read(&hw_ip_paf->count.fe),
				hw_ip_3aa->id, atomic_read(&hw_ip_3aa->count.fe));

		CALL_HWIP_OPS(hw_ip_paf, show_status, group->instance);
	}
}

static const struct is_hardware_ops is_hardware_ops = {
	.frame_start	= is_hardware_frame_start,
	.config_lock	= is_hardware_config_lock,
	.frame_done	= is_hardware_frame_done,
	.frame_ndone	= is_hardware_frame_ndone,
	.flush_frame	= is_hardware_flush_frame,
	.dbg_trace	= _is_hw_frame_dbg_trace,
	.dma_cfg	= is_hardware_dma_cfg,
	.dma_cfg_kva64	= is_hardware_dma_cfg_kva64,
};

static int is_hardware_ip_probe(struct is_hardware *hardware,
			struct is_interface *itf,
			struct is_interface_ischain *itfc,
			struct is_mem *mem,
			enum is_hardware_id ip_id,
			int ip_num, char *ip_name,
			hw_ip_probe_fn_t fn)
{
	int ret;
	struct is_hw_ip *hw_ip;
	int i;
	enum is_hardware_id hw_id;

	for (i = 0; i < ip_num; i++) {
		hw_id = ip_id + i;
		hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, hw_id);
		if (!hw_ip) {
			err_hw("invalid id (%d)", hw_id);
			return -EINVAL;
		}

		hw_ip->hardware = hardware;

		ret = fn(hw_ip, itf, itfc, hw_id, ip_name);
		if (ret) {
			err_hw("%s%d probe fail (%d)", ip_name, i, hw_id);
			return ret;
		}

		/* common */
		hw_ip->id = hw_id;
		snprintf(hw_ip->name, sizeof(hw_ip->name), "%s%d", ip_name, i);
		hw_ip->itf = itf;
		hw_ip->itfc = itfc;
		atomic_set(&hw_ip->fcount, 0);
		hw_ip->is_leader = true;
		atomic_set(&hw_ip->status.Vvalid, V_BLANK);
		atomic_set(&hw_ip->rsccount, 0);
		init_waitqueue_head(&hw_ip->status.wait_queue);
		hw_ip->state = 0;
		hw_ip->mem = mem;

		pablo_hw_helper_probe(hw_ip);

		sinfo_hw("probe done\n", hw_ip);
	}

	return 0;
}

int is_hardware_probe(struct is_hardware *hardware,
	struct is_interface *itf, struct is_interface_ischain *itfc,
	struct is_mem *mem, struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct device *dev;
	struct exynos_platform_is *pdata;
	struct is_core *core;

	FIMC_BUG(!hardware);
	FIMC_BUG(!itf);
	FIMC_BUG(!itfc);

	for (i = 0; i < HW_SLOT_MAX; i++) {
		hardware->hw_ip[i].id = DEV_HW_END;
		hardware->hw_ip[i].priv_info = NULL;
		hardware->hw_ip[i].dump_for_each_reg = NULL;
		hardware->hw_ip[i].dump_reg_list_size = 0;
	}

	dev = &pdev->dev;
	pdata = dev->platform_data;

	if (IS_ENABLED(SOC_CSTAT0))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				 DEV_HW_3AA0, pdata->num_of_ip.taa,
				"CSTAT", is_hw_cstat_probe);
	else
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_3AA0, pdata->num_of_ip.taa,
				"3AA", is_hw_3aa_probe);

	if (IS_ENABLED(SOC_I0S))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_ISP0, pdata->num_of_ip.isp,
				"ISP", is_hw_isp_probe);

	if (IS_ENABLED(SOC_BYRP))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_BYRP, pdata->num_of_ip.byrp,
				"BYRP", is_hw_byrp_probe);

	if (IS_ENABLED(SOC_RGBP))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_RGBP, pdata->num_of_ip.rgbp,
				"RGBP", is_hw_rgbp_probe);

	ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
			DEV_HW_MCSC0, pdata->num_of_ip.mcsc,
			"MCS", is_hw_mcsc_probe);

	if (IS_ENABLED(SOC_VRA))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_VRA, pdata->num_of_ip.vra,
				"VRA", is_hw_vra_probe);

	if (IS_ENABLED(SOC_YPP))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_YPP, pdata->num_of_ip.ypp,
				"YPP", is_hw_ypp_probe);

	if (IS_ENABLED(SOC_YUVP))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_YPP, pdata->num_of_ip.ypp,
				"YUVP", is_hw_yuvp_probe);

	if (IS_ENABLED(SOC_MCFP))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_MCFP, pdata->num_of_ip.mcfp,
				"MCFP", is_hw_mcfp_probe);

	if (IS_ENABLED(SOC_LME0))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_LME0, pdata->num_of_ip.lme,
				"LME", is_hw_lme_probe);

	if (IS_ENABLED(SOC_ORBMCH))
		ret |= is_hardware_ip_probe(hardware, itf, itfc, mem,
				DEV_HW_ORB0, pdata->num_of_ip.orbmch,
				"ORBMCH", is_hw_orbmch_probe);

	if (ret) {
		err_hw("failed to is_hardware_ip_probe()");
		return ret;
	}

	hardware->icpu_adt = pablo_get_icpu_adt();

	core = is_get_is_core();
	for (i = 0; i < IS_STREAM_COUNT; i++) {
		hardware->hw_map[i] = 0;
		hardware->sensor_position[i] = 0;
		hardware->crta_bufmgr[i] = &core->resourcemgr.crta_bufmgr[i];
	}

	for (i = 0; i < SENSOR_POSITION_MAX; i++)
		atomic_set(&hardware->streaming[i], 0);

	for (i = 0; i < GROUP_SLOT_MAX; i++)
		atomic_set(&hardware->slot_rsccount[i], 0);

	/* Initialize hardware operations */
	hardware->ops = &is_hardware_ops;

	mutex_init(&hardware->itf_lock);

	atomic_set(&hardware->rsccount, 0);
	atomic_set(&hardware->bug_count, 0);
	atomic_set(&hardware->log_count, 0);
	hardware->video_mode = false;
	clear_bit(HW_OVERFLOW_RECOVERY, &hardware->hw_recovery_flag);

	prepare_sfr_dump(hardware);

	return 0;
}

static int crc_flag;
int clear_gather_crc_status(u32 instance, struct is_hw_ip *hw_ip)
{
	struct is_device_ischain *device;
	struct is_device_sensor *sensor;
	struct is_device_csi *csi;

	device = is_get_ischain_device(instance);
	if (!device) {
		mserr_hw("failed to get devcie", instance, hw_ip);
		return -ENODEV;
	}

	sensor = device->sensor;
	if (!sensor) {
		mserr_hw("failed to get sensor", instance, hw_ip);
		return -ENODEV;
	}

	csi = (struct is_device_csi *)v4l2_get_subdevdata(sensor->subdev_csi);
	if (!csi) {
		mserr_hw("csi is null\n", instance, hw_ip);
		return -ENODEV;
	}

	if (csi->crc_flag)
		crc_flag |= 1 << instance;
	else
		crc_flag &= ~(1 << instance);

	csi->crc_flag = false;

	return crc_flag;
}
