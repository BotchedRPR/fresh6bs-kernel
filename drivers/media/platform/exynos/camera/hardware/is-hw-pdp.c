/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/mutex.h>

#include <media/v4l2-subdev.h>

#include "votf/pablo-votf.h"
#include "is-config.h"
#include "is-hw-pdp.h"
#include "is-device-sensor-peri.h"
#include "api/is-hw-api-pdp-v1.h"
#include "is-votfmgr.h"
#include "pablo-debug.h"
#include "is-resourcemgr.h"
#include "is-votf-id-table.h"
#include "pablo-irq.h"
#include "is-dvfs.h"
#include "pablo-fpsimd.h"
#include "is-interface-ddk.h"
#include "pablo-icpu-adapter.h"
#include "pablo-icpu-itf.h"

static DEFINE_MUTEX(cmn_reg_lock);

static void pdp_s_rdma_addr(void *data, unsigned long id, struct is_frame *frame)
{
	struct is_pdp *pdp;
	u32 direct;

	pdp = (struct is_pdp *)data;
	if (!pdp) {
		err("failed to get PDP");
		return;
	}

	direct = test_bit(IS_PDP_VOTF_FIRST_FRAME, &pdp->state) ? 1: 0;

	pdp_hw_s_rdma_addr(pdp->base, frame->dvaddr_buffer, frame->num_buffers, direct,
		pdp->sensor_cfg);
}

static void pdp_s_af_rdma_addr(void *data, unsigned long id, struct is_frame *frame)
{
	struct is_pdp *pdp;
	u32 direct;

	pdp = (struct is_pdp *)data;
	if (!pdp) {
		err("failed to get PDP");
		return;
	}

	direct = test_bit(IS_PDP_VOTF_FIRST_FRAME, &pdp->state) ? 1: 0;

	pdp_hw_s_af_rdma_addr(pdp->base, frame->dvaddr_buffer, frame->num_buffers, direct);
}

static void pdp_s_one_shot_enable(void *data, unsigned long id)
{
	int ret = 0;
	struct is_pdp *pdp;

	pdp = (struct is_pdp *)data;
	if (!pdp) {
		err("failed to get PDP");
		return;
	}

	if (test_and_clear_bit(IS_PDP_VOTF_FIRST_FRAME, &pdp->state)) {
		pdp_hw_s_path(pdp->base, DMA);
		pdp_hw_s_corex_enable(pdp->base, true);
		pdp->err_cnt_oneshot = 0;

		if (pdp_hw_g_idle_state(pdp->base))
			ret = pdp_hw_s_one_shot_enable(pdp);
	} else {
		ret = pdp_hw_s_one_shot_enable(pdp);
	}

	if (ret) {
		if (pdp->err_cnt_oneshot == 0) {
			pdp_hw_dump(pdp->base);
			votf_sfr_dump();
		}

		if (pdp->err_cnt_oneshot == PDP_STUCK_CNT) {
			err("PDP%d stuck", pdp->id);
			is_kernel_log_dump(false);
		}

		pdp->err_cnt_oneshot++;
	} else {
		pdp->err_cnt_oneshot = 0;
	}
}

static void pdp_rdma_wait_idle(void *data, unsigned long id)
{
	int ret = 0;
	struct is_pdp *pdp;

	pdp = (struct is_pdp *)data;
	if (!pdp) {
		err("failed to get PDP");
		return;
	}

	info("[PDP%d] %s\n", pdp->id, __func__);
	ret = pdp_hw_rdma_wait_idle(pdp->base);
	if (ret)
		err("[PDP%d] failed to pdp_hw_rdma_wait_idle", pdp->id);
}

static const struct votf_ops pdp_img_votf_ops = {
	.s_addr		= pdp_s_rdma_addr,
	.s_oneshot	= pdp_s_one_shot_enable,
	.wait_idle	= pdp_rdma_wait_idle,
};

static const struct votf_ops pdp_af_votf_ops = {
	.s_addr		= pdp_s_af_rdma_addr,
};

static int pdp_print_work_list(struct is_work_list *work_list)
{
	err("");
	print_fre_work_list(work_list);
	print_req_work_list(work_list);

	return 0;
}

static int get_free_set_req_work(struct is_work_list *work_list, unsigned int fcount)
{
	struct is_work *work;
	int ret;

	ret = get_free_work(work_list, &work);
	if (ret || !work) {
		err("failed to get FREE work from work_list(%d)", work_list->id);
		return ret;
	}

	work->fcount = fcount;
	set_req_work(work_list, work);

	if (IS_ENABLED(PDP_TRACE_WORK))
		pdp_print_work_list(work_list);

	return ret;
}

static inline void wq_func_schedule(struct is_pdp *pdp, struct work_struct *work_wq)
{
	if (pdp->wq_stat)
		queue_work(pdp->wq_stat, work_wq);
	else
		schedule_work(work_wq);
}

static int is_hw_pdp_set_pdstat_reg(struct is_pdp *pdp)
{
	unsigned long flag;
	struct pdp_stat_reg *regs;
	u32 regs_size;
	int i;
	int ret = 0;

	FIMC_BUG(!pdp);

	spin_lock_irqsave(&pdp->slock_paf_s_param, flag);

	if (pdp->stat_enable && test_bit(IS_PDP_SET_PARAM_COPY, &pdp->state)) {
		pdp_hw_s_corex_type(pdp->base, COREX_IGNORE);

		regs_size = pdp->regs_size;
		regs = pdp->regs;
		for (i = 0; i < regs_size; i++)
			writel(regs[i].reg_data, pdp->base + regs[i].reg_addr + COREX_OFFSET);

		clear_bit(IS_PDP_SET_PARAM_COPY, &pdp->state);

		pdp_hw_s_wdma_init(pdp->base, pdp->id);
		pdp_hw_s_line_row(pdp->base, pdp->stat_enable, pdp->vc_ext_sensor_mode, pdp->binning);

		pdp_hw_s_corex_type(pdp->base, COREX_COPY);

		dbg_pdp(1, " load ofs done", pdp);
	}

	spin_unlock_irqrestore(&pdp->slock_paf_s_param, flag);

	return ret;
}

static inline void is_pdp_frame_start_inline(struct is_hw_ip *hw_ip)
{
	struct is_pdp *pdp = (struct is_pdp *)hw_ip->priv_info;
	u32 instance = atomic_read(&hw_ip->instance);
	u32 fcount;
#if defined(CSIS_PDP_VOTF_GLOBAL_WA)
	struct paf_rdma_param *param;
	u32 en_votf;
#endif

	clear_bit(IS_PDP_VOTF_FIRST_FRAME, &pdp->state);

	/* PDP core setting */
	if (!test_bit(IS_PDP_DISABLE_REQ_IN_FS, &pdp->state))
		is_hw_pdp_set_pdstat_reg(pdp);

	fcount = atomic_add_return(1, &hw_ip->count.fs);
	_is_hw_frame_dbg_trace(hw_ip, fcount, DEBUG_POINT_FRAME_START);

	if (hw_ip->is_leader)
		CALL_HW_OPS(hw_ip->hardware, frame_start, hw_ip, instance);
	else
		atomic_set(&hw_ip->status.Vvalid, V_VALID);

#if defined(CSIS_PDP_VOTF_GLOBAL_WA)
	param = &hw_ip->region[instance]->parameter.paf;
	en_votf = param->dma_input.v_otf_enable;
	if (en_votf == OTF_INPUT_COMMAND_ENABLE) {
		struct is_group *group = hw_ip->group[instance];

		if (test_bit(IS_PDP_DISABLE_REQ_IN_FS, &pdp->state)) {
			struct is_device_ischain *device;
			struct is_device_sensor *sensor;
			u32 total_line, curr_line;

			device = is_get_ischain_device(instance);
			if (!device) {
				mserr_hw("failed to get devcie", instance, hw_ip);
				return;
			}

			sensor = device->sensor;
			if (!sensor) {
				mserr_hw("failed to get sensor", instance, hw_ip);
				return;
			}

			pdp_hw_get_line(pdp->base, &total_line, &curr_line);
			if (curr_line > 0 && ((curr_line * 100 / total_line) <= 70)) {
				pdp_hw_s_global_enable(pdp->base, false);
				/*
				 * During pdp is off in frame start ISR,
				 * CSIS DMA also need to be disabled,
				 * for prevent defect that caused by votf pair.
				 */
				v4l2_subdev_call(sensor->subdev_csi, core, ioctl,
						SENSOR_IOCTL_CSI_DMA_DISABLE, NULL);

				clear_bit(IS_PDP_DISABLE_REQ_IN_FS, &pdp->state);
				dbg_pdp(1, "Disable called in valid(total:%d, curr:%d)",
						pdp, total_line, curr_line);
			} else {
				mswarn_hw("No margin(total:%d, curr:%d) for disable, delay to next",
						instance, hw_ip, total_line, curr_line);
			}
		}
		/*
		 * Call interrupt handler function of rear IP connected by OTF.
		 * This is only used with col_row interrupt when v-blank is "0".
		 * TODO: next IP don't have to use this relay.
		 */
		is_hw_interrupt_relay(group, hw_ip);
	}
#endif
}

static inline void is_pdp_frame_end_inline(struct is_hw_ip *hw_ip)
{
	struct is_pdp *pdp = (struct is_pdp *)hw_ip->priv_info;
	u32 instance = atomic_read(&hw_ip->instance);

	atomic_add(1, &hw_ip->count.fe);

	_is_hw_frame_dbg_trace(hw_ip, atomic_read(&hw_ip->count.fs),
				DEBUG_POINT_FRAME_END);

	if (hw_ip->is_leader)
		CALL_HW_OPS(hw_ip->hardware, frame_done, hw_ip, NULL, -1, IS_HW_CORE_END,
				IS_SHOT_SUCCESS, false);
	else
		atomic_set(&hw_ip->status.Vvalid, V_BLANK);

	if (pdp->stat_enable)
		wq_func_schedule(pdp, &pdp->work_stat[WORK_PDP_STAT1]);

	/* clear crc flag */
	clear_gather_crc_status(instance, hw_ip);

	wake_up(&hw_ip->status.wait_queue);

	spin_lock(&pdp->slock_oneshot);
	if (test_and_clear_bit(IS_PDP_ONESHOT_PENDING, &pdp->state)) {
		spin_unlock(&pdp->slock_oneshot);

		pdp_hw_s_one_shot_enable(pdp);
		msinfo_hw("[F:%d] clear oneshot pending", instance, hw_ip,
				atomic_read(&hw_ip->count.fe));
	} else {
		spin_unlock(&pdp->slock_oneshot);
	}

}

static inline void is_pdp_stat_end_inline(struct is_hw_ip *hw_ip)
{
	struct is_pdp *pdp = (struct is_pdp *)hw_ip->priv_info;
	struct is_framemgr *stat_framemgr;
	struct is_frame *stat_frame;
	struct is_work_list *work_list;
	int work_id;
	u32 instance = atomic_read(&hw_ip->instance);

	if (pdp->stat_enable) {
		stat_framemgr = pdp->stat_framemgr;
		if (!stat_framemgr) {
			err("stat_framemgr is NULL\n");
			return;
		}

		framemgr_e_barrier(stat_framemgr, FMGR_IDX_30);

		/* select & flush */
		do {
			stat_frame = peek_frame(stat_framemgr, FS_REQUEST);
			if (!stat_frame) {
				dbg_pdp(0, " PE_PAF_STAT0 req frame is NULL\n", pdp);
				frame_manager_print_queues(stat_framemgr);
				break;
			}

			if (stat_frame->dvaddr_buffer[0] == pdp_hw_g_wdma_addr(pdp->base))
				break;

			if (stat_framemgr->queued_count[FS_REQUEST] == 1) {
				mswarn_hw("[F:%d] Not current stat_frame",
					instance, hw_ip, stat_frame->fcount);
				stat_frame = NULL;
				break;
			}

			trans_frame(stat_framemgr, stat_frame, FS_FREE);
			dbg_pdp(2, "flush R->F index %d, dva: %pad, kva: %lx\n",
					pdp, stat_frame->index,
					&stat_frame->dvaddr_buffer[0],
					stat_frame->kvaddr_buffer[0]);
		} while (stat_frame);

		if (stat_frame) {
			trans_frame(stat_framemgr, stat_frame, FS_FREE);

			dbg_pdp(2, "R->F index %d, dva: %pad, kva: %lx\n",
					pdp, stat_frame->index,
					&stat_frame->dvaddr_buffer[0],
					stat_frame->kvaddr_buffer[0]);

			framemgr_x_barrier(stat_framemgr, FMGR_IDX_30);

			for (work_id = WORK_PDP_STAT0; work_id < WORK_PDP_MAX; work_id++) {
				work_list = &pdp->work_list[work_id];
				get_free_set_req_work(work_list, stat_frame->fcount);
			}
		} else {
			framemgr_x_barrier(stat_framemgr, FMGR_IDX_30);
		}
		wq_func_schedule(pdp, &pdp->work_stat[WORK_PDP_STAT0]);
	}
}

static inline void is_pdp_overflow_inline(struct is_hw_ip *hw_ip, struct is_pdp *pdp,
					u32 instance)
{
#if IS_ENABLED(CONFIG_EXYNOS_SCI_DBG_AUTO)
	smc_ppc_enable(0);
#endif
	print_all_hw_frame_count(hw_ip->hardware);
	is_hardware_sfr_dump(hw_ip->hardware, DEV_HW_END, false);
#if defined(OVERFLOW_DTP_ENABLE_PDP)
	set_bit(IS_SENSOR_FRONT_DTP_STOP, &hw_ip->group[instance]->device->sensor->state);
	err("DTP forcely set by LIC overflow");
#else
	if (IS_ENABLED(USE_SKIP_DUMP_LIC_OVERFLOW)
	    && clear_gather_crc_status(instance, hw_ip) > 0) {
		set_bit(IS_SENSOR_ESD_RECOVERY,
			&hw_ip->group[instance]->device->sensor->state);
		warn("skip to s2d dump");
		pdp_hw_s_reset(pdp->cmn_base);
	} else {
		is_debug_s2d(true, "LIC overflow");
	}
#endif
}

static irqreturn_t is_isr_pdp_int1(int irq, void *data)
{
	struct is_hw_ip *hw_ip;
	struct is_pdp *pdp;
	unsigned int state;
	unsigned long err_state;
	u32 instance;
	int frame_start, frame_end;

	hw_ip = (struct is_hw_ip *)data;
	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		err("failed to get PDP");
		return IRQ_HANDLED;
	}

	instance = atomic_read(&hw_ip->instance);

	state = pdp_hw_g_int1_state(pdp->base, true, &pdp->irq_state[PDP_INT1])
		& pdp_hw_g_int1_mask(pdp->base);
	dbg_pdp(1, "INT1: 0x%x\n", pdp, state);

	if (!test_bit(HW_OPEN, &hw_ip->state)) {
		mserr_hw("INT1 invalid interrupt: 0x%x", instance, hw_ip, state);
		return IRQ_NONE;
	}

	if (test_bit(HW_OVERFLOW_RECOVERY, &hw_ip->hardware->hw_recovery_flag)) {
		mserr_hw("INT1 During recovery : invalid interrupt: 0x%x", instance, hw_ip, state);
		return IRQ_NONE;
	}

	frame_start = pdp_hw_is_occurred(state, PE_START);
	frame_end = pdp_hw_is_occurred(state, PE_END);

	/* Check VOTF wait connection */
	if (pdp_hw_is_occurred(state, PE_SPECIAL)) {
		struct is_group *group = hw_ip->group[instance];

		if (test_bit(IS_GROUP_VOTF_INPUT, &group->state))
			is_votf_check_wait_con(group);
	}

	if (frame_start && frame_end) {
		dbg_pdp(1, "start/end overlapped. 0x%x", pdp, state);

		if (atomic_read(&hw_ip->status.Vvalid) == V_BLANK) {
			is_pdp_frame_start_inline(hw_ip);
			is_pdp_frame_end_inline(hw_ip);
		} else {
			is_pdp_frame_end_inline(hw_ip);
			is_pdp_frame_start_inline(hw_ip);
		}
	} else if (frame_start) {
		is_pdp_frame_start_inline(hw_ip);
	} else if (frame_end) {
		is_pdp_frame_end_inline(hw_ip);
	}

	if (pdp_hw_is_occurred(state, PE_PAF_STAT0))
		is_pdp_stat_end_inline(hw_ip);

	err_state = (unsigned long)pdp_hw_is_occurred(state, PE_ERR_INT1);
	if (err_state) {
		unsigned long long time;
		int err;
		ulong usec;

		pdp->time_err = local_clock();
		time = pdp->time_err - pdp->time_rta_cfg;
		usec = do_div(time, NSEC_PER_SEC) / NSEC_PER_USEC;

		err = find_first_bit(&err_state, SZ_32);
		while (err < SZ_32) {
			if (usec < 100000)
				mserr_hw(" err INT1(%d):RTA wrong config", instance, hw_ip, err);
			else
				mserr_hw(" err INT1(%d):%s", instance, hw_ip, err, pdp->int1_str[err]);

			err = find_next_bit(&err_state, SZ_32, err + 1);
		}
		is_hardware_sfr_dump(hw_ip->hardware, hw_ip->id, false);

		if (pdp_hw_is_occurred(state, PE_PAF_OVERFLOW))
			is_pdp_overflow_inline(hw_ip, pdp, instance);
	}

	return IRQ_HANDLED;
}

static irqreturn_t is_isr_pdp_int2(int irq, void *data)
{
	struct is_hw_ip *hw_ip;
	struct is_pdp *pdp;
	unsigned int state;
	u32 instance;
	unsigned long err_state, err_rdma;
	int err;

	hw_ip = (struct is_hw_ip *)data;
	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		err("failed to get PDP");
		return IRQ_HANDLED;
	}

	instance = atomic_read(&hw_ip->instance);

	state = pdp_hw_g_int2_state(pdp->base, true, &pdp->irq_state[PDP_INT2])
		& pdp_hw_g_int2_mask(pdp->base);
	dbg_pdp(1, "INT2: 0x%x\n", pdp, state);

	if (pdp_hw_is_occurred(state, PE_PAF_STAT0))
		is_pdp_stat_end_inline(hw_ip);

	err_state = (unsigned long)pdp_hw_is_occurred(state, PE_ERR_INT2);
	if (err_state) {
		err = find_first_bit(&err_state, SZ_32);
		while (err < SZ_32) {
			mserr_hw(" err INT2(%d):%s", instance, hw_ip, err, pdp->int2_str[err]);
			err = find_next_bit(&err_state, SZ_32, err + 1);
		}

		if (pdp_hw_is_occurred(state, PE_ERR_RDMA_IRQ)) {
			err_rdma = pdp_hw_g_int2_rdma_state(pdp->base, true);
			err = find_first_bit(&err_rdma, SZ_32);
			while (err < SZ_32) {
				mserr_hw(" err RDMA(%d):%s", instance, hw_ip,
					err, pdp->int2_rdma_str[err]);
				err = find_next_bit(&err_rdma, SZ_32, err + 1);
			}
		}

		is_hardware_sfr_dump(hw_ip->hardware, hw_ip->id, false);

		if (pdp_hw_is_occurred(state, PE_PAF_OVERFLOW))
			is_pdp_overflow_inline(hw_ip, pdp, instance);
	}

	return IRQ_HANDLED;
}

static int is_hw_pdp_stat_end(struct is_pdp *pdp, u32 fcount, enum itf_vc_buf_data_type data_type)
{
	int ret;
	struct pablo_crta_buf_info stat_buf = { 0, };
	struct is_framemgr *stat_framemgr;
	struct is_frame *stat_frame;
	struct is_priv_buf *pb;
	unsigned long flags;

	/* replace get_vc_dma_buf() */
	stat_framemgr = pdp->stat_framemgr;
	if (!stat_framemgr) {
		merr_hw("failed to get stat_framemgr", pdp->prev_instance);
		return -ENXIO;
	}

	framemgr_e_barrier_irqs(stat_framemgr, FMGR_IDX_30, flags);

	stat_frame = find_frame(stat_framemgr, FS_FREE, frame_fcount, (void *)(ulong)fcount);
	if (stat_frame) {
		/* FREE -> PROCESS */
		trans_frame(stat_framemgr, stat_frame, FS_PROCESS);

		dbg_pdp(2, "F->P index %d, dva: %pad, kva: %lx, dva_icpu: %pad\n",
			pdp, stat_frame->index,
			&stat_frame->dvaddr_buffer[0],
			stat_frame->kvaddr_buffer[0],
			&stat_frame->pb_output->iova_ext);
	} else {
		/* PROCESS -> REPEAT_PROCESS */
		stat_frame = find_frame(stat_framemgr, FS_PROCESS, frame_fcount, (void *)(ulong)fcount);
		if (stat_frame) {
			trans_frame(stat_framemgr, stat_frame, FS_REPEAT_PROCESS);

			dbg_pdp(2, "P->RP index %d, dva: %pad, kva: %lx, dva_icpu: %pad\n",
				pdp, stat_frame->index,
				&stat_frame->dvaddr_buffer[0],
				stat_frame->kvaddr_buffer[0],
				&stat_frame->pb_output->iova_ext);
		} else {
			merr_hw("[F%d]failed to get a frame", pdp->prev_instance, fcount);
			ret = -EINVAL;
			goto err_invalid_frame;
		}
	}

	if (stat_frame) {
		pb = stat_frame->pb_output;
		stat_buf.kva = pb->kva;
		stat_buf.dva = pb->iova_ext;
	}

	framemgr_x_barrier_irqr(stat_framemgr, FMGR_IDX_30, flags);

	/* send message */
	if (data_type == VC_BUF_DATA_TYPE_GENERAL_STAT1) {
		ret = CALL_ADT_MSG_OPS(pdp->icpu_adt, send_msg_pdp_stat0_end, pdp->prev_instance,
				(void *)pdp, NULL, fcount, 1, &stat_buf);
		if (ret)
			merr_hw("icpu_adt send_msg_pdp_stat0_end fail", pdp->prev_instance);
	} else if (data_type == VC_BUF_DATA_TYPE_GENERAL_STAT2) {
		ret = CALL_ADT_MSG_OPS(pdp->icpu_adt, send_msg_pdp_stat1_end, pdp->prev_instance,
				(void *)pdp, NULL, fcount, 1, &stat_buf);
		if (ret)
			merr_hw("icpu_adt send_msg_pdp_stat1_end fail", pdp->prev_instance);
	} else {
		ret = -EINVAL;
	}

	return ret;

err_invalid_frame:
	framemgr_x_barrier_irqr(stat_framemgr, FMGR_IDX_30, flags);
	return ret;
}

static int is_hw_pdp_stat_end_callback(void *user, void *ctx, void *rsp_msg)
{
	u32 instance, fcount;
	struct is_pdp *pdp;
	struct pablo_icpu_adt_rsp_msg *msg;
	struct is_framemgr *stat_framemgr;
	struct is_frame *stat_frame;
	unsigned long flags;

	if (!user || !rsp_msg) {
		err_hw("invalid callback: user(%p), msg(%p)",
			user, rsp_msg);
		return -EINVAL;
	}

	pdp = (struct is_pdp *)user;
	msg = (struct pablo_icpu_adt_rsp_msg *)rsp_msg;
	instance = msg->instance;
	fcount = msg->fcount;

	mdbg_hw(1, "[F%d]pdp_stat_end_callback", instance, fcount);

	/* replace put_vc_dma_buf */
	stat_framemgr = pdp->stat_framemgr;
	if (!stat_framemgr) {
		merr_hw("failed to get stat_framemgr", instance);
		return -ENXIO;
	}

	framemgr_e_barrier_irqs(stat_framemgr, FMGR_IDX_30, flags);
	stat_frame = find_frame(stat_framemgr, FS_REPEAT_PROCESS, frame_fcount, (void *)(ulong)fcount);
	if (stat_frame) {
		/* REPEAT_PROCESS -> PROCESS */
		trans_frame(stat_framemgr, stat_frame, FS_PROCESS);

		dbg_pdp(2, "RP->P index %d, dva: %pad, kva: %lx, dva_icpu: %pad\n",
				pdp, stat_frame->index,
				&stat_frame->dvaddr_buffer[0],
				stat_frame->kvaddr_buffer[0],
				&stat_frame->pb_output->iova_ext);
	} else {
		/* PROCESS -> FREE */
		stat_frame = find_frame(stat_framemgr, FS_PROCESS, frame_fcount, (void *)(ulong)fcount);
		if (stat_frame) {
			trans_frame(stat_framemgr, stat_frame, FS_FREE);

			dbg_pdp(2, "P->F index %d, dva: %pad, kva: %lx, dva_icpu: %pad\n",
				pdp, stat_frame->index,
				&stat_frame->dvaddr_buffer[0],
				stat_frame->kvaddr_buffer[0],
				&stat_frame->pb_output->iova_ext);
		} else {
			merr_hw("failed to get a frame: fcount: %d", instance, fcount);
		}
	}
	framemgr_x_barrier_irqr(stat_framemgr, FMGR_IDX_30, flags);

	return 0;
}

static void __nocfi pdp_worker_stat0(struct work_struct *data)
{
	struct is_pdp *pdp;
	struct paf_action *pa, *temp;
	unsigned long flag;
	unsigned int fcount;
	struct is_work_list *work_list;
	struct is_work *work;

	FIMC_BUG_VOID(!data);

	pdp = container_of(data, struct is_pdp, work_stat[WORK_PDP_STAT0]);

	work_list = &pdp->work_list[WORK_PDP_STAT0];
	get_req_work(work_list, &work);
	while (work) {
		fcount = work->fcount;

		spin_lock_irqsave(&pdp->slock_paf_action, flag);
		list_for_each_entry_safe(pa, temp, &pdp->list_of_paf_action, list) {
			switch (pa->type) {
			case VC_STAT_TYPE_PDP_1_0_PDAF_STAT0:
			case VC_STAT_TYPE_PDP_1_1_PDAF_STAT0:
			case VC_STAT_TYPE_PDP_3_0_PDAF_STAT0:
			case VC_STAT_TYPE_PDP_3_1_PDAF_STAT0:
			case VC_STAT_TYPE_PDP_4_0_PDAF_STAT0:
			case VC_STAT_TYPE_PDP_4_1_PDAF_STAT0:
				is_fpsimd_get_func();
				if (IS_ENABLED(CRTA_CALL))
					is_hw_pdp_stat_end(pdp, fcount, VC_BUF_DATA_TYPE_GENERAL_STAT1);
				else
					pa->notifier(pa->type, fcount, pa->data);
				is_fpsimd_put_func();
				break;
			default:
				break;
			}
		}
		spin_unlock_irqrestore(&pdp->slock_paf_action, flag);

		dbg_pdp(3, "%s, fcount: %d\n", pdp, __func__, fcount);

		set_free_work(work_list, work);
		get_req_work(work_list, &work);

		if (IS_ENABLED(PDP_TRACE_WORK))
			pdp_print_work_list(work_list);
	}
}

static void __nocfi pdp_worker_stat1(struct work_struct *data)
{
	struct is_pdp *pdp;
	struct paf_action *pa, *temp;
	unsigned long flag;
	unsigned int fcount;
	struct is_work_list *work_list;
	struct is_work *work;

	FIMC_BUG_VOID(!data);

	pdp = container_of(data, struct is_pdp, work_stat[WORK_PDP_STAT1]);

	work_list = &pdp->work_list[WORK_PDP_STAT1];
	get_req_work(work_list, &work);
	while (work) {
		fcount = work->fcount;

		spin_lock_irqsave(&pdp->slock_paf_action, flag);
		list_for_each_entry_safe(pa, temp, &pdp->list_of_paf_action, list) {
			switch (pa->type) {
			case VC_STAT_TYPE_PDP_1_0_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_1_1_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_3_0_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_3_1_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_4_0_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_4_1_PDAF_STAT1:
				is_fpsimd_get_func();
				if (IS_ENABLED(CRTA_CALL))
					is_hw_pdp_stat_end(pdp, fcount, VC_BUF_DATA_TYPE_GENERAL_STAT2);
				else
					pa->notifier(pa->type, fcount, pa->data);
				is_fpsimd_put_func();
				break;
			default:
				break;
			}
		}
		spin_unlock_irqrestore(&pdp->slock_paf_action, flag);

		dbg_pdp(3, "%s, fcount: %d\n", pdp, __func__, fcount);

		set_free_work(work_list, work);
		get_req_work(work_list, &work);

		if (IS_ENABLED(PDP_TRACE_WORK))
			pdp_print_work_list(work_list);
	}
}

static int is_hw_pdp_set_regs(struct is_hw_ip *hw_ip, u32 chain_id,
	u32 instance, u32 fcount, struct cr_set *regs, u32 regs_size)
{
	int i;
	struct is_pdp *pdp;
	unsigned long flag;
	u32 corex_enable;
	ulong debug_iq = (unsigned long) is_get_debug_param(IS_DEBUG_PARAM_IQ);

	FIMC_BUG(!hw_ip->priv_info);
	FIMC_BUG(!regs);

	pdp = (struct is_pdp *)hw_ip->priv_info;

	if (!regs_size || unlikely(test_bit(hw_ip->id, &debug_iq)))
		return 0;

	if (test_bit(IS_PDP_WAITING_IDLE, &pdp->state)) {
		info("%s, skip PDP(%p) RTA setting during streaming off", __func__, pdp->base);
		return 0;
	}

	/* CAUTION: PD path must be on before algorithm block setting. */
	pdp_hw_s_pdstat_path(pdp->base, true);

	pdp_hw_g_corex_state(pdp->base, &corex_enable);
	dbg_pdp(1, "state[0x%lX], corex[%d]", pdp, pdp->state, corex_enable);

	if (test_bit(IS_PDP_SET_PARAM, &pdp->state) && corex_enable) {
		dbg_pdp(0, "PDP(%pa) store RTA set, size(%d)\n", pdp, &pdp->regs_start, regs_size);

		for (i = 0; i < regs_size; i++)
			info("[PDP%d][%d] store ofs: 0x%x, val: 0x%x\n", pdp->id,
					i, regs[i].reg_addr, regs[i].reg_data);

		spin_lock_irqsave(&pdp->slock_paf_s_param, flag);

		pdp->regs_size = regs_size;
		memcpy((void *)pdp->regs, (void *)regs, sizeof(struct pdp_stat_reg) * regs_size);
		set_bit(IS_PDP_SET_PARAM_COPY, &pdp->state);

		spin_unlock_irqrestore(&pdp->slock_paf_s_param, flag);
	} else {
		dbg_pdp(0, "PDP(%pa) RTA setting, size(%d)\n", pdp, &pdp->regs_start, regs_size);

		for (i = 0; i < regs_size; i++) {
			dbg_pdp(1, "[%d] ofs: 0x%x, val: 0x%x\n", pdp,
					i, regs[i].reg_addr, regs[i].reg_data);
			writel(regs[i].reg_data, pdp->base + regs[i].reg_addr + COREX_OFFSET);
		}

		/* CAUTION: WDMA size must be set after PDSTAT_ROI block setting. */
		pdp_hw_s_wdma_init(pdp->base, pdp->id);

		if (pdp->stat_enable)
			pdp_hw_s_line_row(pdp->base, pdp->stat_enable, pdp->vc_ext_sensor_mode, pdp->binning);

		set_bit(IS_PDP_SET_PARAM, &pdp->state);
	}

	pdp->time_rta_cfg = local_clock();

	return 0;
}

int pdp_set_param(struct v4l2_subdev *subdev, struct cr_set *regs, u32 regs_size)
{
	struct is_pdp *pdp;

	pdp = (struct is_pdp *)v4l2_get_subdevdata(subdev);
	if (!pdp) {
		err("failed to get PDP");
		return -ENODEV;
	}

	return is_hw_pdp_set_regs(pdp->hw_ip, 0, 0, 0, regs, regs_size);
}

int pdp_get_ready(struct v4l2_subdev *subdev, u32 *ready)
{
	struct is_pdp *pdp;

	pdp = (struct is_pdp *)v4l2_get_subdevdata(subdev);
	if (!pdp) {
		err("failed to get PDP");
		return -ENODEV;
	}

	if (test_bit(IS_PDP_SET_PARAM_COPY, &pdp->state))
		*ready = 0;
	else
		*ready = 1;

	return 0;
}

int pdp_register_notifier(struct v4l2_subdev *subdev, enum itf_vc_stat_type type,
		vc_dma_notifier_t notifier, void *data)
{
	struct is_pdp *pdp;
	struct paf_action *pa;
	unsigned long flag;

	pdp = (struct is_pdp *)v4l2_get_subdevdata(subdev);
	if (!pdp) {
		err("failed to get PDP");
		return -ENODEV;
	}

	switch (type) {
	case VC_STAT_TYPE_PDP_1_0_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_1_1_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_1_0_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_1_1_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_3_0_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_3_0_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_3_1_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_3_1_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_4_0_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_4_0_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_4_1_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_4_1_PDAF_STAT1:
		pa = kzalloc(sizeof(struct paf_action), GFP_ATOMIC);
		if (!pa) {
			err_lib("failed to allocate a PAF action");
			return -ENOMEM;
		}

		pa->type = type;
		pa->notifier = notifier;
		pa->data = data;

		spin_lock_irqsave(&pdp->slock_paf_action, flag);
		list_add(&pa->list, &pdp->list_of_paf_action);
		spin_unlock_irqrestore(&pdp->slock_paf_action, flag);

		break;
	default:
		return -EINVAL;
	}

	dbg_pdp(2, "%s, type: %d, notifier: %p\n", pdp, __func__, type, notifier);

	return 0;
}

int pdp_unregister_notifier(struct v4l2_subdev *subdev, enum itf_vc_stat_type type,
		vc_dma_notifier_t notifier)
{
	struct is_pdp *pdp;
	struct paf_action *pa, *temp;
	unsigned long flag;

	pdp = (struct is_pdp *)v4l2_get_subdevdata(subdev);
	if (!pdp) {
		err("failed to get PDP");
		return -ENODEV;
	}

	switch (type) {
	case VC_STAT_TYPE_PDP_1_0_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_1_1_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_1_0_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_1_1_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_3_0_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_3_0_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_3_1_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_3_1_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_4_0_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_4_0_PDAF_STAT1:
	case VC_STAT_TYPE_PDP_4_1_PDAF_STAT0:
	case VC_STAT_TYPE_PDP_4_1_PDAF_STAT1:
		spin_lock_irqsave(&pdp->slock_paf_action, flag);
		list_for_each_entry_safe(pa, temp,
				&pdp->list_of_paf_action, list) {
			if ((pa->notifier == notifier)
					&& (pa->type == type)) {
				list_del(&pa->list);
				kfree(pa);
			}
		}
		spin_unlock_irqrestore(&pdp->slock_paf_action, flag);

		break;
	default:
		return -EINVAL;
	}

	dbg_pdp(2, "%s, type: %d, notifier: %p\n", pdp, __func__, type, notifier);

	return 0;
}

void __nocfi pdp_notify(struct v4l2_subdev *subdev, unsigned int type, void *data)
{
	struct is_pdp *pdp;
	struct paf_action *pa, *temp;
	unsigned long flag;

	pdp = (struct is_pdp *)v4l2_get_subdevdata(subdev);
	if (!pdp) {
		err("failed to get PDP");
		return;
	}

	switch (type) {
	case CSIS_NOTIFY_DMA_END_VC_MIPISTAT:
		spin_lock_irqsave(&pdp->slock_paf_action, flag);
		list_for_each_entry_safe(pa, temp, &pdp->list_of_paf_action, list) {
			switch (pa->type) {
			case VC_STAT_TYPE_PDP_1_0_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_1_1_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_3_0_PDAF_STAT0:
			case VC_STAT_TYPE_PDP_3_0_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_3_1_PDAF_STAT0:
			case VC_STAT_TYPE_PDP_3_1_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_4_0_PDAF_STAT0:
			case VC_STAT_TYPE_PDP_4_0_PDAF_STAT1:
			case VC_STAT_TYPE_PDP_4_1_PDAF_STAT0:
			case VC_STAT_TYPE_PDP_4_1_PDAF_STAT1:
				is_fpsimd_get_func();
				pa->notifier(pa->type, *(unsigned int *)data, pa->data);
				is_fpsimd_put_func();
				break;
			default:
				break;
			}
		}
		spin_unlock_irqrestore(&pdp->slock_paf_action, flag);

	default:
		break;
	}

	dbg_pdp(2, "%s, sensor fcount: %d\n", pdp, __func__, *(unsigned int *)data);
}

static DEFINE_SPINLOCK(cmn_reg_slock);
static int is_hw_pdp_init_config(struct is_hw_ip *hw_ip, u32 instance, struct is_frame *frame)
{
	int ret = 0;
	int pd_mode;
	bool enable;
	unsigned int sensor_type;
	struct is_pdp *pdp;
	struct is_group *group;
	struct is_device_ischain *device;
	struct is_device_sensor *sensor;
	struct is_sensor_cfg *sensor_cfg;
	struct is_device_csi *csi;
	struct paf_rdma_param *param;
	struct is_crop img_full_size = {0, 0, 0, 0};
	struct is_crop img_crop_size = {0, 0, 0, 0};
	struct is_crop img_comp_size = {0, 0, 0, 0};
	u32 img_hwformat = DMA_INOUT_FORMAT_BAYER_PACKED;
	u32 img_pixelsize = OTF_INPUT_BIT_WIDTH_10BIT;
	u32 pd_width, pd_height, pd_hwformat;
	u32 path;
	u32 en_votf;
	u32 en_sdc;
	struct is_module_enum *module;
	u32 fps;
	ulong flags = 0;
	u32 extformat;
	u32 position;

	FIMC_BUG(!hw_ip);

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	group = hw_ip->group[instance];
	if (!group) {
		mserr_hw("failed to get group", instance, hw_ip);
		return -ENODEV;
	}

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

	pdp->sensor_cfg = sensor_cfg = sensor->cfg;
	if (!sensor_cfg) {
		mserr_hw("failed to get sensor_cfg", instance, hw_ip);
		return -EINVAL;
	}
	position = sensor->position;

	if (test_bit(IS_ISCHAIN_OFFLINE_REPROCESSING, &device->state))
		module = device->module_enum;
	else
		module = (struct is_module_enum *)v4l2_get_subdevdata(sensor->subdev_module);

	if (test_bit(IS_ISCHAIN_REPROCESSING, &device->state) ||
		!test_bit(IS_PDP_SET_PARAM, &pdp->state)) {
		pd_mode = PD_NONE; /* Reprocessing does not support PAF stat. */
		pdp->vc_ext_sensor_mode = 0;
	} else {
		pd_mode = sensor_cfg->pd_mode;
		pdp->vc_ext_sensor_mode =
			module ? module->vc_extra_info[VC_BUF_DATA_TYPE_GENERAL_STAT1].sensor_mode : 0;
		pdp->binning = sensor_cfg->binning;
	}

	enable = pdp_hw_to_sensor_type(pd_mode, &sensor_type);
	pdp->stat_enable = enable;

	param = &hw_ip->region[instance]->parameter.paf;
	en_votf = param->dma_input.v_otf_enable;
	if (param->dma_input.cmd == DMA_INPUT_COMMAND_ENABLE) {
		u32 min_fps = 0, max_fps = 0;

		if (frame->shot) {
			min_fps = frame->shot->ctl.aa.aeTargetFpsRange[0];
			max_fps = frame->shot->ctl.aa.aeTargetFpsRange[1];
		}

		path = DMA;
		pdp_hw_update_rdma_linegap(pdp->base, sensor_cfg, pdp, en_votf,
			min_fps, max_fps);
	} else {
		path = OTF;
	}

	/* WDMA */
	if (enable) {
		struct is_subdev *subdev = &device->pdst;
		struct is_framemgr *stat_framemgr = GET_SUBDEV_FRAMEMGR(subdev);
		struct is_frame *stat_frame;
		unsigned long flags;

		FIMC_BUG(!stat_framemgr);

		pdp->stat_framemgr = stat_framemgr;
		framemgr_e_barrier_irqs(stat_framemgr, FMGR_IDX_30, flags);

		stat_frame = get_frame(stat_framemgr, FS_FREE);
		if (stat_frame) {
			stat_frame->fcount = frame->fcount;
			put_frame(stat_framemgr, stat_frame, FS_REQUEST);
			framemgr_x_barrier_irqr(stat_framemgr, FMGR_IDX_30, flags);

			pdp_hw_s_wdma_enable(pdp->base, stat_frame->dvaddr_buffer[0]);

			dbg_pdp(2, "F->R index %d, hw_ip fcount: %d, dva: %pad, kva: %lx\n",
				pdp, stat_frame->index, atomic_read(&hw_ip->fcount),
				&stat_frame->dvaddr_buffer[0],
				stat_frame->kvaddr_buffer[0]);

		} else {
			mswarn_hw(" PDSTAT free frame is NULL", instance, hw_ip);
			frame_manager_print_queues(stat_framemgr);
			framemgr_x_barrier_irqr(stat_framemgr, FMGR_IDX_30, flags);
		}
	} else {
		pdp_hw_s_wdma_disable(pdp->base);
	}

	if (path == OTF || en_votf) {
		/* Get current frame time from sensor for use wait_idle time. */
		struct is_device_sensor_peri *sensor_peri;

		if (!IS_ENABLED(CONFIG_CAMERA_CIS_ZEBU_OBJ)
		    && module && module->private_data) {
			sensor_peri = module->private_data;
			pdp->cur_frm_time = sensor_peri->cis.cis_data->cur_frame_us_time;
		}

		/* A shot after stream on is prevented. */
		if (atomic_read(&group->head->scount))
			return 0;

		hw_ip->is_leader = false;
	} else {
		set_bit(hw_ip->id, &frame->core_flag);

		if (pdp->prev_instance == instance)
			return 0;

		hw_ip->is_leader = true;
	}

	pdp->prev_instance = instance;

	extformat = sensor_cfg->input[pdp->img_vc].extformat;
	if (extformat == HW_FORMAT_RAW10_SDC)
		pdp->en_sdc = en_sdc = 1;
	else
		pdp->en_sdc = en_sdc = 0;

	/* VOTF RDMA settings */
	if (en_votf == OTF_INPUT_COMMAND_ENABLE) {
		struct is_frame *votf_frame;
		struct is_framemgr *votf_fmgr;

		ret = is_hw_pdp_set_votf_config(group, sensor_cfg);
		if (ret)
			mserr_hw("votf config is fail(%d)", instance, hw_ip, ret);

		votf_frame = is_votf_get_frame(group, TRS, ENTRY_PAF, 0);
		if (votf_frame) {
			msinfo_hw(" VOTF Bayer 1st shot(index: %d, dva: 0x%pad)\n", instance, hw_ip,
				votf_frame->index, &votf_frame->dvaddr_buffer[0]);

			votf_fmgr = is_votf_get_framemgr(group, TRS, ENTRY_PAF);
			ret = votf_fmgr_call(votf_fmgr, master, s_addr, votf_frame);
			if (ret)
				mswarn_hw("votf_fmgr_call(master) is fail(%d)",
					instance, hw_ip, ret);

			ret = votf_fmgr_call(votf_fmgr, slave, s_addr, votf_frame);
			if (ret)
				mswarn_hw("votf_fmgr_call(slave) is fail(%d)",
					instance, hw_ip, ret);
		}

		votf_frame = is_votf_get_frame(group, TRS, ENTRY_PDAF, 0);
		if (votf_frame && enable) {
			msinfo_hw(" VOTF AF 1st shot(index: %d, dva: 0x%pad)\n", instance, hw_ip,
				votf_frame->index, &votf_frame->dvaddr_buffer[0]);

			votf_fmgr = is_votf_get_framemgr(group, TRS, ENTRY_PDAF);
			ret = votf_fmgr_call(votf_fmgr, master, s_addr, votf_frame);
			if (ret)
				mswarn_hw("votf_fmgr_call(master) is fail(%d)",
					instance, hw_ip, ret);

			ret = votf_fmgr_call(votf_fmgr, slave, s_addr, votf_frame);
			if (ret)
				mswarn_hw("votf_fmgr_call(slave) is fail(%d)",
					instance, hw_ip, ret);
		} else {
			v4l2_subdev_call(sensor->subdev_csi, core, ioctl,
					SENSOR_IOCTL_DISABLE_VC1_VOTF, NULL);
		}
	}

	if (test_bit(IS_ISCHAIN_REPROCESSING, &device->state))
		fps = 0; /* Reprocessing is not real-time procesing. So, a fps is not needed. */
	else
		fps = sensor_cfg->max_fps;

	if (path == OTF) {
		img_full_size.w = sensor_cfg->input[pdp->img_vc].width;
		img_full_size.h = sensor_cfg->input[pdp->img_vc].height;
		img_crop_size.w = img_full_size.w;
		img_crop_size.h = img_full_size.h;
		img_comp_size.w = img_crop_size.w;
		img_comp_size.h = img_crop_size.h;

		if (sensor_cfg->input[pdp->img_vc].hwformat == HW_FORMAT_RAW14)
			img_pixelsize = OTF_INPUT_BIT_WIDTH_14BIT;
		else if (sensor_cfg->input[pdp->img_vc].hwformat == HW_FORMAT_RAW12)
			img_pixelsize = OTF_INPUT_BIT_WIDTH_12BIT;
	} else if (path == DMA) {
		if (en_votf) {
			img_full_size.w = sensor_cfg->output[pdp->img_vc].width;
			/* output height does not mean DMA height, but VOTF line count. */
			img_full_size.h = sensor_cfg->input[pdp->img_vc].height;
			img_crop_size.w = img_full_size.w;
			img_crop_size.h = img_full_size.h;
			img_comp_size.w = img_crop_size.w;
			img_comp_size.h = img_crop_size.h;

			if (sensor_cfg->output[pdp->img_vc].hwformat == HW_FORMAT_RAW14)
				img_pixelsize = OTF_INPUT_BIT_WIDTH_14BIT;
			else if (sensor_cfg->output[pdp->img_vc].hwformat == HW_FORMAT_RAW12)
				img_pixelsize = OTF_INPUT_BIT_WIDTH_12BIT;
			else if (sensor_cfg->output[pdp->img_vc].hwformat == HW_FORMAT_RAW8)
				img_pixelsize = OTF_INPUT_BIT_WIDTH_8BIT;

			if (en_sdc) {
				img_crop_size.w = sensor_cfg->width;
				img_crop_size.h = sensor_cfg->height;

				/* PDP RDMA handles 8BIT RAW as 16bit unpacked format */
				if (img_pixelsize == OTF_INPUT_BIT_WIDTH_8BIT) {
					img_full_size.w /= 2;
					img_comp_size.w = img_full_size.w;
					img_hwformat = DMA_INOUT_FORMAT_BAYER;
				}
			}
		} else {
			img_full_size.w = param->dma_input.width;
			img_full_size.h = param->dma_input.height;
			img_crop_size.w = param->dma_input.dma_crop_width;
			img_crop_size.h = param->dma_input.dma_crop_height;
			img_comp_size.w = img_crop_size.w;
			img_comp_size.h = img_crop_size.h;
			img_hwformat = param->dma_input.format;
			img_pixelsize = param->dma_input.msb + 1;

			if (en_sdc) {
				img_comp_size.w = img_full_size.w;
				img_comp_size.h = img_full_size.h;
				img_pixelsize = param->dma_input.bitwidth;

				/* PDP RDMA handles 8BIT RAW as 16bit unpacked format */
				if (param->dma_input.bitwidth == DMA_INOUT_BIT_WIDTH_8BIT) {
					img_full_size.w /= 2;
					img_comp_size.w = img_full_size.w;
				}
			}
		}
	}

	pd_width = sensor_cfg->input[pdp->af_vc].width;
	pd_height = sensor_cfg->input[pdp->af_vc].height;
	pd_hwformat = sensor_cfg->input[pdp->af_vc].hwformat;

	/* PDP context setting */
	pdp_hw_s_sensor_type(pdp->base, sensor_type);
	pdp_hw_s_core(pdp, enable, sensor_cfg, img_full_size, img_crop_size, img_comp_size,
		img_hwformat, img_pixelsize,
		pd_width, pd_height, pd_hwformat, sensor_type, path, pdp->vc_ext_sensor_mode,
		fps, en_sdc, en_votf, frame->num_buffers, pdp->binning, position);

	spin_lock_irqsave(&cmn_reg_slock, flags);
	/* ch0 context */
	pdp_hw_s_context(pdp->cmn_base, pdp->id, path, pdp->lic_dma_only);
	spin_unlock_irqrestore(&cmn_reg_slock, flags);

	if (enable && is_get_debug_param(IS_DEBUG_PARAM_PDP) >= 5) {
		msinfo_hw(" is configured as default values\n", instance, hw_ip);
		pdp_hw_s_config_default(pdp->base);

		/* CAUTION: WDMA size must be set after PDSTAT_ROI block setting. */
		pdp_hw_s_wdma_init(pdp->base, pdp->id);
	}

	/* PDP OTF input mux & CSIS OTF output enable */
	if (path == OTF || en_votf) {
		csi = (struct is_device_csi *)v4l2_get_subdevdata(sensor->subdev_csi);
		if (!csi) {
			mserr_hw("csi is null\n", instance, hw_ip);
			return -ENODEV;
		}

		pdp->csi_ch = csi->ch;

		ret = pdp_hw_s_input_mux(pdp, true);
		if (ret)
			return ret;

		if (pdp->vc_mux_base) {
			u32 vc_mux_id;

			vc_mux_id = (pdp->vc_mux_elems * pdp->mux_offset) + pdp->csi_ch;
			writel(pdp->vc_mux_val[vc_mux_id], pdp->vc_mux_base);
		} else {
			mserr_hw("pdp_vc_mux_base is null: need to check DT define\n", instance, hw_ip);
		}

		spin_lock_irqsave(&cmn_reg_slock, flags);
		pdp_hw_s_input_enable(pdp, true);
		spin_unlock_irqrestore(&cmn_reg_slock, flags);

		msinfo_hw("[NS]CSI(%d) --> PDP(%d)\n", instance, hw_ip, pdp->csi_ch, pdp->id);
	}

	switch (path) {
	case OTF:
	case STRGEN:
		/* This path selection must be set at the end. */
		pdp_hw_s_path(pdp->base, path);
		pdp_hw_s_global_enable(pdp->base, true);
		break;
	case DMA:
		if (en_votf) {
			set_bit(IS_PDP_VOTF_FIRST_FRAME, &pdp->state);

			if (en_sdc || IS_ENABLED(CONFIG_VOTF_ONESHOT)) {
				/* One Shot */
			} else {
				/* This path selection must be set at the end. */
				pdp_hw_s_path(pdp->base, path);
				pdp_hw_s_global_enable(pdp->base, true);
			}
		}
		break;
	}

	if (is_get_debug_param(IS_DEBUG_PARAM_PDP) >= 2)
		pdp_hw_dump(pdp->base);

	msinfo_hw(" %s as PD mode %d IMG_VC %d AF_VC %d INT1 0x%x INT2 0x%x\n",
			instance, hw_ip,
			enable ? "enabled" : "disabled", pd_mode,
			pdp->img_vc, pdp->af_vc,
			pdp_hw_g_int1_state(pdp->base, false, &pdp->irq_state[PDP_INT1])
			& pdp_hw_g_int1_mask(pdp->base),
			pdp_hw_g_int2_state(pdp->base, false, &pdp->irq_state[PDP_INT2])
			& pdp_hw_g_int2_mask(pdp->base));

	return ret;
}

static int is_hw_pdp_open(struct is_hw_ip *hw_ip, u32 instance)
{
	int ret = 0;
	struct is_pdp *pdp;
	int work_id;

	FIMC_BUG(!hw_ip);

	if (test_bit(HW_OPEN, &hw_ip->state))
		return 0;

	frame_manager_probe(hw_ip->framemgr, hw_ip->id, "HWPDP");
	frame_manager_open(hw_ip->framemgr, IS_MAX_HW_FRAME);

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	spin_lock_init(&pdp->slock_paf_action);
	INIT_LIST_HEAD(&pdp->list_of_paf_action);
	spin_lock_init(&pdp->slock_paf_s_param);
	spin_lock_init(&pdp->slock_oneshot);

	pablo_set_affinity_irq(pdp->irq[PDP_INT1], true);
	pablo_set_affinity_irq(pdp->irq[PDP_INT2], true);

	for (work_id = WORK_PDP_STAT0; work_id < WORK_PDP_MAX; work_id++)
		init_work_list(&pdp->work_list[work_id], work_id, MAX_WORK_COUNT);

	if (pdp->icpu_adt) {
		pdp_register_notifier(pdp->subdev, VC_STAT_TYPE_PDP_4_1_PDAF_STAT0, NULL, NULL);
		pdp_register_notifier(pdp->subdev, VC_STAT_TYPE_PDP_4_1_PDAF_STAT1, NULL, NULL);
	}

	msinfo_hw(" is registered\n", instance, hw_ip);

	pdp->hw_fro_en = false;

	set_bit(HW_OPEN, &hw_ip->state);

	msdbg_hw(2, "open: framemgr[%s]", instance, hw_ip, hw_ip->framemgr->name);

	return ret;
}

static int is_hw_pdp_init(struct is_hw_ip *hw_ip, u32 instance,
			bool flag, u32 f_type)
{
	int ret = 0;
	u32 frame_idx;
	struct is_pdp *pdp;
	struct is_device_ischain *device;
	struct is_framemgr *framemgr;
	struct is_priv_buf *pb;

	FIMC_BUG(!hw_ip);
	FIMC_BUG(!hw_ip->priv_info);

	hw_ip->frame_type = f_type;

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	device = is_get_ischain_device(instance);
	if (!device) {
		mserr_hw("failed to get devcie", instance, hw_ip);
		return -ENODEV;
	}

	if (!test_bit(IS_ISCHAIN_REPROCESSING, &device->state)) {
		struct is_subdev *subdev = &device->pdst;
		u32 w, h, b;
		struct is_sensor_cfg *sensor_cfg;
		u32 sbwc_type, lossy_byte32num;

		if (!device->sensor) {
			mserr_hw("failed to get sensor", instance, hw_ip);
			return -EINVAL;
		}

		sensor_cfg = device->sensor->cfg;
		if (!sensor_cfg) {
			mserr_hw("failed to get senso_cfg", instance, hw_ip);
			return -EINVAL;
		}

		if (hw_ip->frame_type == LIB_FRAME_HDR_SHORT && pdp->vc_mux_arrs > 1)
			pdp->mux_offset = PDP_CTX_SUB;
		else
			pdp->mux_offset = PDP_CTX_MAIN;

		pdp_hw_g_input_vc(pdp->mux_offset, &pdp->img_vc, &pdp->af_vc);

		if (sensor_cfg->pd_mode == PD_MSPD || sensor_cfg->pd_mode == PD_MSPD_TAIL) {
			struct is_module_enum *module =
				v4l2_get_subdevdata(device->sensor->subdev_module);
			pdp->bpc_h_img_width[instance] =
				pdp_hw_get_bpc_reorder_sram_size(sensor_cfg->input[pdp->af_vc].width,
						module->vc_extra_info[VC_BUF_DATA_TYPE_GENERAL_STAT1].sensor_mode);
		} else {
			pdp->bpc_h_img_width[instance] = 0;
		}

		if (!test_bit(IS_SUBDEV_OPEN, &subdev->state)) {
			ret = is_subdev_internal_open(instance, IS_VIDEO_PAF0S_NUM, subdev);
			if (ret) {
				merr("is_subdev_internal_open is fail(%d)", subdev, ret);
				return ret;
			}

			if (IS_ENABLED(USE_DEBUG_PD_DUMP) && ((DEBUG_PD_DUMP_CH >> pdp->id) & 0x1)) {
				w = sensor_cfg->input[pdp->af_vc].width;
				h = sensor_cfg->input[pdp->af_vc].height;
				b = 16;

				msinfo_hw("[DBG] set PDSTAT Dump mode", instance, hw_ip);
			} else {
				pdp_hw_g_pdstat_size(&w, &h, &b);
			}

			is_subdev_internal_get_sbwc_type(subdev, &sbwc_type, &lossy_byte32num);

			ret = is_subdev_internal_s_format(subdev,
						w, h, b, sbwc_type, lossy_byte32num,
						SUBDEV_INTERNAL_BUF_MAX,
						"PDSTAT");
			if (ret) {
				merr("is_subdev_internal_s_format is fail(%d)", subdev, ret);
				return ret;
			}

			ret = is_subdev_internal_start(subdev);
			if (ret) {
				merr("subdev internal start is fail(%d)", subdev, ret);
				return ret;
			}

			if (pdp->icpu_dev) {
				framemgr = GET_SUBDEV_I_FRAMEMGR(subdev);
				for (frame_idx = 0; frame_idx < framemgr->num_frames; frame_idx++) {
					pb = framemgr->frames[frame_idx].pb_output;
					ret = CALL_BUFOP(pb, attach_ext, pb, pdp->icpu_dev);
					if (ret) {
						merr("failed to attach_ext(%d)", subdev, ret);
						return ret;
					}
				}
			}
		}

		subdev = &device->pdaf;
		if (!test_bit(IS_SUBDEV_OPEN, &subdev->state)) {
			ret = is_subdev_internal_open(instance, IS_VIDEO_PAF0S_NUM, subdev);
			if (ret) {
				merr("is_subdev_internal_open is fail(%d)", subdev, ret);
				return ret;
			}
		}

		ret = CALL_ADT_MSG_OPS(pdp->icpu_adt, register_response_msg_cb, instance,
			PABLO_HIC_PDP_STAT0_END, is_hw_pdp_stat_end_callback);
		if (ret)
			mserr_hw("icpu_adt register_response_msg_cb(pdp_stat0) fail", instance, hw_ip);

		ret = CALL_ADT_MSG_OPS(pdp->icpu_adt, register_response_msg_cb, instance,
			PABLO_HIC_PDP_STAT1_END, is_hw_pdp_stat_end_callback);
		if (ret)
			mserr_hw("icpu_adt register_response_msg_cb(pdp_stat1) fail", instance, hw_ip);
	}

	set_bit(HW_INIT, &hw_ip->state);

	return ret;
}

static int is_hw_pdp_deinit(struct is_hw_ip *hw_ip, u32 instance)
{
	int ret = 0;
	u32 frame_idx;
	struct is_pdp *pdp;
	struct is_device_ischain *device;
	struct is_framemgr *framemgr;
	struct is_priv_buf *pb;

	FIMC_BUG(!hw_ip);
	FIMC_BUG(!hw_ip->priv_info);

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	device = is_get_ischain_device(instance);
	if (!device) {
		mserr_hw("failed to get devcie", instance, hw_ip);
		return -ENODEV;
	}

	if (!test_bit(IS_ISCHAIN_REPROCESSING, &device->state)) {
		struct is_subdev *subdev = &device->pdst;

		if (test_bit(IS_SUBDEV_INTERNAL_USE, &subdev->state)) {
			if (pdp->icpu_dev) {
				framemgr = GET_SUBDEV_I_FRAMEMGR(subdev);
				for (frame_idx = 0; frame_idx < framemgr->num_frames; frame_idx++) {
					pb = framemgr->frames[frame_idx].pb_output;
					CALL_VOID_BUFOP(pb, detach_ext, pb);
				}
			}

			ret = is_subdev_internal_stop(subdev);
			if (ret)
				merr("subdev internal stop is fail(%d)", subdev, ret);

			ret = is_subdev_internal_close(subdev);
			if (ret)
				merr("is_subdev_internal_close is fail(%d)", subdev, ret);
		}

		subdev = &device->pdaf;
		if (test_bit(IS_SUBDEV_INTERNAL_USE, &subdev->state)) {
			ret = is_subdev_internal_close(subdev);
			if (ret)
				merr("is_subdev_internal_close is fail(%d)", subdev, ret);
		}
	}

	clear_bit(IS_PDP_SET_PARAM, &pdp->state);
	clear_bit(IS_PDP_SET_PARAM_COPY, &pdp->state);
	clear_bit(IS_PDP_DISABLE_REQ_IN_FS, &pdp->state);

	return ret;
}

static int is_hw_pdp_close(struct is_hw_ip *hw_ip, u32 instance)
{
	int ret = 0;
	struct is_pdp *pdp;
	struct paf_action *pa, *temp;
	unsigned long flag;
	int i;
	struct is_hardware *hardware;

	FIMC_BUG(!hw_ip);

	if (!test_bit(HW_OPEN, &hw_ip->state))
		return 0;

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	if (!list_empty(&pdp->list_of_paf_action)) {
		mserr_hw("flush remaining notifiers...", instance, hw_ip);
		spin_lock_irqsave(&pdp->slock_paf_action, flag);
		list_for_each_entry_safe(pa, temp,
				&pdp->list_of_paf_action, list) {
			list_del(&pa->list);
			kfree(pa);
		}
		spin_unlock_irqrestore(&pdp->slock_paf_action, flag);
	}

	frame_manager_close(hw_ip->framemgr);

	clear_bit(HW_OPEN, &hw_ip->state);

	clear_bit(IS_PDP_SET_PARAM, &pdp->state);
	clear_bit(IS_PDP_SET_PARAM_COPY, &pdp->state);
	clear_bit(IS_PDP_ONESHOT_PENDING, &pdp->state);
	clear_bit(IS_PDP_DISABLE_REQ_IN_FS, &pdp->state);

	pablo_set_affinity_irq(pdp->irq[PDP_INT1], false);
	pablo_set_affinity_irq(pdp->irq[PDP_INT2], false);

	hardware = hw_ip->hardware;
	if (!hardware) {
		mserr_hw("hardware is null", instance, hw_ip);
		return -EINVAL;
	}

	/*
	 * For safe power off
	 * This is common register for all PDP channel.
	 * So, it should be set one time in final instance.
	 */
	mutex_lock(&cmn_reg_lock);
	for (i = 0; i < pdp->max_num; i++) {
		struct is_hw_ip *hw_ip_phys = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip,
								DEV_HW_PAF0 + i);

		if (hw_ip_phys && test_bit(HW_OPEN, &hw_ip_phys->state))
			break;
	}

	if (i == pdp->max_num) {
		pdp_hw_s_reset(pdp->cmn_base);

		ret = pdp_hw_wait_idle(pdp->base, pdp->state, pdp->cur_frm_time);
		if (ret)
			mserr_hw("failed to pdp_hw_wait_idle", instance, hw_ip);

		msinfo_hw("final finished pdp ch%d\n", instance, hw_ip, pdp->id);
	}
	mutex_unlock(&cmn_reg_lock);

	msinfo_hw(" is unregistered\n", instance, hw_ip);

	return ret;
}

static int is_hw_pdp_set_subdev(struct is_hw_ip *hw_ip, u32 instance)
{
	int ret = 0;
	struct is_pdp *pdp;
	struct is_device_ischain *device;

	FIMC_BUG(!hw_ip);

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	/* subdev change for RTA */
	if (hw_ip->frame_type == LIB_FRAME_HDR_SHORT) {
		msinfo_hw("Skip set_subdev", instance, hw_ip);
		return 0;
	}

	device = is_get_ischain_device(instance);
	if (!device) {
		mserr_hw("failed to get devcie", instance, hw_ip);
		return -ENODEV;
	}

	if (!test_bit(IS_ISCHAIN_REPROCESSING, &device->state)) {
		struct is_device_sensor *sensor;
		struct is_module_enum *module;
		struct is_device_sensor_peri *sensor_peri;

		sensor = device->sensor;
		if (!sensor) {
			mserr_hw("failed to get sensor", instance, hw_ip);
			return -EINVAL;
		}

		module = (struct is_module_enum *)v4l2_get_subdevdata(sensor->subdev_module);

		sensor_peri = module->private_data;

		sensor_peri->pdp = pdp;
		sensor_peri->subdev_pdp = pdp->subdev;
	}

	return ret;
}

static int is_hw_pdp_enable(struct is_hw_ip *hw_ip, u32 instance, ulong hw_map)
{
	int ret = 0;
	struct is_pdp *pdp;
	struct is_hardware *hardware;
	struct pdp_lic_lut *lic_lut = NULL;
	struct is_group *group;
	int i;

	FIMC_BUG(!hw_ip);

	if (!test_bit_variables(hw_ip->id, &hw_map)) {
		mswarn_hw("Not mapped. hw_map 0x%lx", instance, hw_ip, hw_map);
		return 0;
	}

	if (!test_bit(HW_INIT, &hw_ip->state)) {
		mserr_hw("not initialized!!", instance, hw_ip);
		return -EINVAL;
	}

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	/* Change or set subdev for RTA */
	ret = is_hw_pdp_set_subdev(hw_ip, instance);
	if (ret) {
		mserr_hw("is_hw_pdp_set_subdev is fail", instance, hw_ip);
		return ret;
	}

	pdp->prev_instance = IS_STREAM_COUNT;
	pdp->r_line_gap = 0;
	group = hw_ip->group[instance];
	if (test_bit(IS_GROUP_VOTF_INPUT, &group->state)) {
		ret = is_votf_register_framemgr(group, TRS, pdp, &pdp_img_votf_ops, ENTRY_PAF);
		if (ret) {
			mserr_hw("(Bayer) is_votf_register_framemgr is failed", instance, hw_ip);
			return ret;
		}

		ret = is_votf_register_framemgr(group, TRS, pdp, &pdp_af_votf_ops, ENTRY_PDAF);
		if (ret) {
			mserr_hw("(AF) is_votf_register_framemgr is failed", instance, hw_ip);
			return ret;
		}
	}

	hardware = hw_ip->hardware;
	if (!hardware) {
		mserr_hw("hardware is null", instance, hw_ip);
		return -EINVAL;
	}

	/*
	 * This is common register for all PDP channel.
	 * So, it should be set one time in first instance.
	 */
	mutex_lock(&cmn_reg_lock);
	for (i = 0; i < pdp->max_num; i++) {
		struct is_hw_ip *hw_ip_phys = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip,
								DEV_HW_PAF0 + i);

		if (hw_ip_phys && test_bit(HW_RUN, &hw_ip_phys->state))
			break;
	}

	if (i == pdp->max_num) {
		int j;
		int prev_offset = 0;

		pdp_hw_s_reset(pdp->cmn_base);

		/* ch0 global */
		/* Update lic_lut here if it it needed. */
		pdp_hw_g_ip_version(pdp->cmn_base);
		pdp_hw_s_init(pdp->cmn_base);
		pdp_hw_s_global(pdp->cmn_base, pdp->id, pdp->lic_mode_def, pdp->lic_14bit,
			lic_lut);

		/*
		 * Due to limitation, mapping between LIC and core need to do sequencially.
		 * So, If other PDP uses first time, need to map LIC0 - Core0 first
		 */
		pdp_hw_s_pdstat_path(pdp->cmn_base, false);
		if (hw_ip->id != DEV_HW_PAF0) {
			msinfo_hw("Need to map LIC0 - Core0\n", instance, hw_ip);
		}

		msinfo_hw("first enterance pdp ch%d\n", instance, hw_ip, pdp->id);

		/* For decide BPC/Reorder SRAM offset when use MSPD/MSPD tail */
		for (j = 0; j < pdp->max_num; j++) {
			struct is_hw_ip *hw_ip_phys;
			struct is_pdp *pdp_tmp;
			int max_w = 0;
			int k;

			hw_ip_phys = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip,
							DEV_HW_PAF0 + j);
			pdp_tmp = (struct is_pdp *)hw_ip_phys->priv_info;

			if (!pdp_tmp) {
				mserr_hw("failed to get PDP", instance, hw_ip);
				return -ENODEV;
			}

			/* If use same pdp as multi instance,
			 * select biggest size.
			 */
			for (k = 0; k < IS_STREAM_COUNT; k++)
				max_w = (pdp_tmp->bpc_h_img_width[k] >= max_w) ? pdp_tmp->bpc_h_img_width[k] : max_w;

			pdp_tmp->bpc_rod_offset = 0;
			pdp_tmp->bpc_rod_size = 0;
			if (max_w) {
				pdp_tmp->bpc_rod_offset = prev_offset;
				pdp_tmp->bpc_rod_size = ALIGN(max_w, 16);
				prev_offset += pdp_tmp->bpc_rod_size;

				msinfo_hw("BPC/REORDER sram [CH%d] offset = %d, size = %d",
						instance, hw_ip, pdp_tmp->id,
						pdp_tmp->bpc_rod_offset, pdp_tmp->bpc_rod_size);
			}
		}
	}

	set_bit(HW_RUN, &hw_ip->state);
	mutex_unlock(&cmn_reg_lock);

	return ret;
}

static int is_hw_pdp_disable(struct is_hw_ip *hw_ip, u32 instance, ulong hw_map)
{
	int ret = 0;
	long timetowait;
	struct is_pdp *pdp;
	struct is_framemgr *stat_framemgr;
	struct is_frame *stat_frame;
	unsigned long flags;

	FIMC_BUG(!hw_ip);

	if (!test_bit_variables(hw_ip->id, &hw_map)) {
		mswarn_hw("Not mapped. hw_map 0x%lx", instance, hw_ip, hw_map);
		return 0;
	}

	if (!test_bit(HW_INIT, &hw_ip->state)) {
		mserr_hw("not initialized!!", instance, hw_ip);
		return -EINVAL;
	}

	msinfo_hw("disable: Vvalid(%d)\n", instance, hw_ip,
		atomic_read(&hw_ip->status.Vvalid));

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	/*
	 * Vvalid state is only needed to be checked when HW works at same instance.
	 * If HW works at different instance, skip Vvalid checking.
	 */
	if (atomic_read(&hw_ip->instance) == instance) {
		if (!pdp->hw_fro_en) {
			timetowait = wait_event_timeout(hw_ip->status.wait_queue,
					!atomic_read(&hw_ip->status.Vvalid),
					IS_HW_STOP_TIMEOUT);
			if (!timetowait) {
				mserr_hw("wait FRAME_END timeout (%ld)", instance,
						hw_ip, timetowait);
				ret = -ETIME;
			}
		}
	}

	stat_framemgr = pdp->stat_framemgr;
	if (stat_framemgr) {
		framemgr_e_barrier_irqs(stat_framemgr, FMGR_IDX_30, flags);
		stat_frame = get_frame(stat_framemgr, FS_REQUEST);
		while (stat_frame) {
			put_frame(stat_framemgr, stat_frame, FS_FREE);
			stat_frame = get_frame(stat_framemgr, FS_REQUEST);
		}
		framemgr_x_barrier_irqr(stat_framemgr, FMGR_IDX_30, flags);
	}

	if (hw_ip->run_rsc_state)
		return 0;

	if (flush_work(&pdp->work_stat[WORK_PDP_STAT0]))
		msinfo_hw("flush pdp wq for stat0\n", instance, hw_ip);
	if (flush_work(&pdp->work_stat[WORK_PDP_STAT1]))
		msinfo_hw("flush pdp wq for stat1\n", instance, hw_ip);

	clear_bit(HW_RUN, &hw_ip->state);

	return ret;
}

static int is_hw_pdp_shot(struct is_hw_ip *hw_ip, struct is_frame *frame,
	ulong hw_map)
{
	int ret = 0;
	int i, cur_idx;
	struct is_pdp *pdp;
	struct is_region *region;
	struct paf_rdma_param *param;
	u32 instance;
	u32 num_buffers;
	u32 en_votf;
	struct is_crop img_full_size = {0, 0, 0, 0};
	struct is_crop img_crop_size = {0, 0, 0, 0};
	struct is_crop img_comp_size = {0, 0, 0, 0};
	u32 img_hwformat, img_pixelsize;

	FIMC_BUG(!hw_ip);
	FIMC_BUG(!frame);

	instance = atomic_read(&hw_ip->instance);
	num_buffers = frame->num_buffers;
	msdbgs_hw(2, "[F:%d]shot\n", instance, hw_ip, frame->fcount);

	if (!test_bit_variables(hw_ip->id, &hw_map))
		return 0;

	if (!test_bit(HW_INIT, &hw_ip->state)) {
		mserr_hw("not initialized!!", instance, hw_ip);
		return -EINVAL;
	}

	FIMC_BUG(!hw_ip->priv_info);
	pdp = (struct is_pdp *)hw_ip->priv_info;
	region = hw_ip->region[instance];
	FIMC_BUG(!region);

	if (frame->type != SHOT_TYPE_INTERNAL)
		FIMC_BUG(!frame->shot);

	/* multi-buffer */
	hw_ip->num_buffers = num_buffers;
	pdp->hw_fro_en = num_buffers > 1 ? true : false;

	ret = is_hw_pdp_init_config(hw_ip, instance, frame);
	if (ret) {
		mserr_hw("is_hw_pdp_init_config is fail", instance, hw_ip);
		return -EINVAL;
	}

	/* RDMA settings */
	param = &hw_ip->region[instance]->parameter.paf;
	en_votf = param->dma_input.v_otf_enable;
	if ((param->dma_input.cmd == DMA_INPUT_COMMAND_ENABLE)
		&& (en_votf == OTF_INPUT_COMMAND_DISABLE)) {
		cur_idx = frame->cur_buf_index;

		for (i = 0; i < num_buffers; i++) {
			if (frame->dvaddr_buffer[i + cur_idx] == 0) {
				msinfo_hw("[F:%d]dvaddr_buffer[%d] is zero\n",
					instance, hw_ip, frame->fcount, i);
				FIMC_BUG(1);
			}
		}

		if (test_bit(PARAM_PAF_DMA_INPUT, frame->pmap)) {
			img_full_size.w = param->dma_input.width;
			img_full_size.h = param->dma_input.height;
			img_crop_size.w = param->dma_input.dma_crop_width;
			img_crop_size.h = param->dma_input.dma_crop_height;
			img_comp_size.w = img_crop_size.w;
			img_comp_size.h = img_crop_size.h;
			img_hwformat = param->dma_input.format;
			img_pixelsize = param->dma_input.msb + 1;

			if (pdp->en_sdc) {
				img_comp_size.w = img_full_size.w;
				img_comp_size.h = img_full_size.h;
				img_pixelsize = param->dma_input.bitwidth;

				/* PDP RDMA handles 8BIT RAW as 16bit unpacked format */
				if (param->dma_input.bitwidth == DMA_INOUT_BIT_WIDTH_8BIT) {
					img_full_size.w /= 2;
					img_comp_size.w = img_full_size.w;
				}
			}

			pdp_hw_s_img_size(pdp->base, img_full_size, img_crop_size, img_comp_size,
				img_hwformat, img_pixelsize, pdp->sensor_cfg);
			pdp_hw_s_lic_bit_mode(pdp->base, img_pixelsize);
		}

		pdp_hw_s_fro(pdp->base, num_buffers);
		pdp_hw_s_rdma_addr(pdp->base, &frame->dvaddr_buffer[cur_idx], num_buffers, 0,
			pdp->sensor_cfg);
		pdp_hw_s_path(pdp->base, DMA);
		if (pdp_hw_g_idle_state(pdp->base))
			pdp_hw_s_one_shot_enable(pdp);
	}

	return ret;
}

static int is_hw_pdp_set_param(struct is_hw_ip *hw_ip, struct is_region *region,
		IS_DECLARE_PMAP(pmap), u32 instance, ulong hw_map)
{
	FIMC_BUG(!hw_ip);
	FIMC_BUG(!hw_ip->priv_info);

	if (!test_bit_variables(hw_ip->id, &hw_map))
		return 0;

	if (!test_bit(HW_INIT, &hw_ip->state)) {
		mserr_hw("not initialized!!", instance, hw_ip);
		return -EINVAL;
	}

	hw_ip->region[instance] = region;

	return 0;
}

static int is_hw_pdp_frame_ndone(struct is_hw_ip *hw_ip, struct is_frame *frame,
	u32 instance, enum ShotErrorType done_type)
{
	int ret = 0;

	FIMC_BUG(!hw_ip);
	FIMC_BUG(!frame);

	if (test_bit_variables(hw_ip->id, &frame->core_flag))
		ret = CALL_HW_OPS(hw_ip->hardware, frame_done, hw_ip, frame, -1,
				IS_HW_CORE_END, done_type, false);

	return ret;
}

/*
 * Below function is not necessary currently,
 * because 3AA receive interrupt relay from PDP instead of 3AA start interrupt.
 * So, 3AA start and end interrutp is not overlapped anymore when using VOTF.
 */
static int is_hw_pdp_post_frame_gap(struct is_hw_ip *hw_ip, struct is_pdp *pdp, u32 instance)
{
#ifdef PDP_POST_FRAME_GAP
	int i;
	struct is_hardware *hardware;
	struct is_pdp *pdp_phys;
	u32 en_votf;
	struct is_hw_ip *hw_ip_phys;
	u32 instance_phys;
	struct is_region *region;
	struct is_sensor_cfg *sensor_cfg;
	ulong freq;
	ulong fps;
	ulong ratio = 10; /* Portion of V-blank is 10% */
	u32 val;

	freq = is_dvfs_get_freq(IS_DVFS_CAM) * MHZ;
	if (!freq) {
		mswarn_hw("failed to get PDP clock", instance, hw_ip);
		return -EINVAL;
	}

	hardware = hw_ip->hardware;

	for (i = 0; i < pdp->max_num; i++) {
		hw_ip_phys = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip,
						DEV_HW_PAF0 + i);

		if (!hw_ip_phys || !test_bit(HW_RUN, &hw_ip_phys->state))
			continue;

		instance_phys = atomic_read(&hw_ip_phys->instance);
		region = hw_ip_phys->region[instance_phys];
		if (!region)
			continue;

		en_votf = region->parameter.paf.dma_input.v_otf_enable;
		if (en_votf == OTF_INPUT_COMMAND_ENABLE) {
			pdp_phys = (struct is_pdp *)hw_ip_phys->priv_info;
			if (!pdp_phys) {
				mserr_hw("failed to get PDP", instance_phys, hw_ip_phys);
				return -ENODEV;
			}

			/*
			 * Calculate v-blank time
			 * 1 = 1 / freq (sec).
			 */
			sensor_cfg = pdp_phys->sensor_cfg;
			fps = sensor_cfg ? (ulong)sensor_cfg->framerate : 45;

			val = (PDP_POST_FRAME_GAP > 0) ? PDP_POST_FRAME_GAP : (u32)(freq / (fps * ratio));

			msinfo_hw("Force V-Blank (freq: %lu, fps: %lu, ratio: %lu, val: %x)\n",
				instance_phys, hw_ip_phys, freq, fps, ratio, val);

			/* V-blnak control in VOTF case */
			pdp_hw_s_post_frame_gap(pdp_phys->base, val);
		}
	}

#endif

	return 0;
}

static int is_hw_pdp_sensor_start(struct is_hw_ip *hw_ip, u32 instance)
{
	struct is_pdp *pdp;

	/* It's impossible to get clk info in ZEBU env. */
	if (IS_ENABLED(CONFIG_CAMERA_CIS_ZEBU_OBJ))
		return 0;

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	pdp_hw_s_before_sensor_start(pdp->base);

	return is_hw_pdp_post_frame_gap(hw_ip, pdp, instance);
}

static int is_hw_pdp_sensor_stop(struct is_hw_ip *hw_ip, u32 instance)
{
	int ret = 0;
	struct is_pdp *pdp;
	struct is_region *region;
	u32 en_votf;
	ulong flags = 0;

	pdp = (struct is_pdp *)hw_ip->priv_info;
	if (!pdp) {
		mserr_hw("failed to get PDP", instance, hw_ip);
		return -ENODEV;
	}

	region = hw_ip->region[instance];
	if (!region) {
		mserr_hw("region is NULL", instance, hw_ip);
		return -EINVAL;
	}

	en_votf = region->parameter.paf.dma_input.v_otf_enable;
	if (en_votf == OTF_INPUT_COMMAND_ENABLE) {
#if defined(CSIS_PDP_VOTF_GLOBAL_WA)
		if (!pdp->en_sdc)
			set_bit(IS_PDP_DISABLE_REQ_IN_FS, &pdp->state);
#endif
		set_bit(IS_PDP_WAITING_IDLE, &pdp->state);
		ret = pdp_hw_wait_idle(pdp->base, pdp->state, pdp->cur_frm_time);
		if (ret) {
			mserr_hw("failed to pdp_hw_wait_idle: last INT1(0x%X) INT2(0x%X)",
				instance, hw_ip,
				pdp->irq_state[PDP_INT1],
				pdp->irq_state[PDP_INT2]);
			print_all_hw_frame_count(hw_ip->hardware);
			is_debug_s2d(true, "failed to pdp_hw_wait_idle");
		}
	}

	/* This is for corex imediate setting after stream off. */
	pdp_hw_s_global_enable(pdp->base, false);

	pdp_hw_s_global_enable_clear(pdp->base);

	clear_bit(IS_PDP_SET_PARAM, &pdp->state);
	clear_bit(IS_PDP_SET_PARAM_COPY, &pdp->state);
	clear_bit(IS_PDP_ONESHOT_PENDING, &pdp->state);
	clear_bit(IS_PDP_DISABLE_REQ_IN_FS, &pdp->state);
	clear_bit(IS_PDP_WAITING_IDLE, &pdp->state);

	spin_lock_irqsave(&cmn_reg_slock, flags);
	pdp_hw_s_input_enable(pdp, false);
	spin_unlock_irqrestore(&cmn_reg_slock, flags);
	ret = pdp_hw_s_input_mux(pdp, false);

	return ret;
}

static int is_hw_pdp_change_chain(struct is_hw_ip *hw_ip, u32 instance,
	u32 next_id, struct is_hardware *hardware)
{
	int ret = 0;
	u32 curr_id;
	u32 next_hw_id = DEV_HW_PAF0 + next_id;
	struct is_hw_ip *next_hw_ip;

	curr_id = hw_ip->id - DEV_HW_PAF0;
	if (curr_id == next_id) {
		mswarn_hw("Same chain (curr:%d, next:%d)", instance, hw_ip,
			curr_id, next_id);
		goto p_err;
	}

	next_hw_ip = CALL_HW_CHAIN_INFO_OPS(hardware, get_hw_ip, next_hw_id);

	if (!test_and_clear_bit(instance, &hw_ip->run_rsc_state))
		mswarn_hw("try to disable disabled instance", instance, hw_ip);

	ret = is_hw_pdp_disable(hw_ip, instance, hardware->hw_map[instance]);
	if (ret) {
		msinfo_hw("is_hw_pdp_disable is fail", instance, hw_ip);
		return -EINVAL;
	}

	/*
	 * Copy instance infromation.
	 * But do not clear current hw_ip,
	 * because logical(initial) HW must be refered at close time.
	 */
	next_hw_ip->group[instance] = hw_ip->group[instance];
	next_hw_ip->region[instance] = hw_ip->region[instance];

	/* set & clear physical HW */
	set_bit(next_hw_id, &hardware->hw_map[instance]);
	clear_bit(hw_ip->id, &hardware->hw_map[instance]);

	if (test_and_set_bit(instance, &next_hw_ip->run_rsc_state))
		mswarn_hw("try to enable enabled instance", instance, next_hw_ip);

	/* This is for set LIC config */
	ret = is_hw_pdp_enable(next_hw_ip, instance, hardware->hw_map[instance]);
	if (ret) {
		msinfo_hw("is_hw_pdp_enable is fail", instance, next_hw_ip);
		return -EINVAL;
	}

	/*
	 * There is no change about rsccount when change_chain processed
	 * because there is no open/close operation.
	 * But if it isn't increased, abnormal situation can be occurred
         * according to hw close order among instances.
	 */
	if (!test_bit(hw_ip->id, &hardware->logical_hw_map[instance])) {
		atomic_dec(&hw_ip->rsccount);
		msinfo_hw("decrease hw_ip rsccount(%d)", instance, hw_ip, atomic_read(&hw_ip->rsccount));
	}

	if (!test_bit(next_hw_ip->id, &hardware->logical_hw_map[instance])) {
		atomic_inc(&next_hw_ip->rsccount);
		msinfo_hw("increase next_hw_ip rsccount(%d)", instance, next_hw_ip, atomic_read(&next_hw_ip->rsccount));
	}

	msinfo_hw("change_chain done (state: curr(0x%lx) next(0x%lx))", instance, hw_ip,
		hw_ip->state, next_hw_ip->state);
p_err:
	return ret;
}

static void is_hw_pdp_show_status(struct is_hw_ip *hw_ip, u32 instance)
{
	struct is_pdp *pdp = (struct is_pdp *)hw_ip->priv_info;
	u32 total_line, curr_line;

	pdp_hw_get_line(pdp->base, &total_line, &curr_line);
	msinfo_hw("total_line:%d, curr_line:%d", instance, hw_ip, total_line, curr_line);
}

static const struct v4l2_subdev_core_ops core_ops = {
	.init = NULL
};

static const struct v4l2_subdev_video_ops video_ops = {
	.s_stream = NULL,
};

static const struct v4l2_subdev_pad_ops pad_ops = {
	.set_fmt = NULL
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
	.video = &video_ops,
	.pad = &pad_ops
};

static struct is_pdp_ops pdp_ops = {
	.set_param = pdp_set_param,
	.get_ready = pdp_get_ready,
	.register_notifier = pdp_register_notifier,
	.unregister_notifier = pdp_unregister_notifier,
	.notify = pdp_notify,
};

const struct is_hw_ip_ops is_hw_pdp_ops = {
	.open			= is_hw_pdp_open,
	.init			= is_hw_pdp_init,
	.deinit			= is_hw_pdp_deinit,
	.close			= is_hw_pdp_close,
	.enable			= is_hw_pdp_enable,
	.disable		= is_hw_pdp_disable,
	.shot			= is_hw_pdp_shot,
	.set_param		= is_hw_pdp_set_param,
	.frame_ndone		= is_hw_pdp_frame_ndone,
	.sensor_start		= is_hw_pdp_sensor_start,
	.sensor_stop		= is_hw_pdp_sensor_stop,
	.change_chain		= is_hw_pdp_change_chain,
	.show_status		= is_hw_pdp_show_status,
	.set_regs		= is_hw_pdp_set_regs,
};

static int pdp_get_array_val(struct device *dev, struct device_node *dnode, char *name, u32 **val,
	u32 *elems)
{
	int ret = 0;

	ret = of_property_count_u32_elems(dnode, name);
	if (ret < 0) {
		dev_err(dev, "failed to get mux_val property\n");
		return ret;
	}

	*elems = ret;

	*val = devm_kcalloc(dev, *elems, sizeof(**val), GFP_KERNEL);
	if (!*val) {
		dev_err(dev, "out of memory for mux_val\n");
		ret = -ENOMEM;
		return ret;
	}

	ret = of_property_read_u32_array(dnode, name, *val, *elems);
	if (ret) {
		dev_err(dev, "failed to get mux_val resources\n");
		return ret;
	}

	return 0;
}

static int pdp_get_resource_mem_byname(struct platform_device *pdev, char *name, void __iomem **base, u32 **val,
	u32 *elems)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct device_node *dnode = dev->of_node;
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		dev_err(dev, "can't get memory resource for %s_base\n", name);
		return -ENODEV;
	}

	if (!base) {
		dev_err(dev, "invalid base for %s_base\n", name);
		return -EINVAL;
	}

	*base = devm_ioremap(dev, res->start, resource_size(res));
	if (!*base) {
		dev_err(dev, "ioremap failed for %s_base\n", name);
		return -ENOMEM;
	}

	if (of_find_property(dnode, name, NULL) && val && elems) {
		ret = pdp_get_array_val(dev, dnode, name, val, elems);
		if (ret) {
			dev_err(dev, "failed to pdp_get_array_val\n");
			return ret;
		}
	}

	return 0;
}

static int pdp_probe(struct platform_device *pdev)
{
	int ret = 0;
	int id;
	bool leader;
	struct device *is_dev;
	struct resource *res;
	struct is_pdp *pdp;
	struct device *dev = &pdev->dev;
	struct is_core *core;
	struct is_hw_ip *hw_ip;
	int hw_id;
	int max_num_of_pdp;
	int num_scen;
	struct device_node *lic_lut_np, *scen_np;
	int i, work_id;
	int reg_size;
	size_t name_len;

	is_dev = is_get_is_dev();
	if (is_dev == NULL) {
		warn("is_dev is not yet probed(pdp)");
		return -EPROBE_DEFER;
	}

	core = (struct is_core *)dev_get_drvdata(is_dev);
	if (!core) {
		err("core is NULL");
		return -EINVAL;
	}

	max_num_of_pdp = of_alias_get_highest_id("pdp");
	if (max_num_of_pdp < 0) {
		dev_err(dev, "invalid alias name\n");
		return max_num_of_pdp;
	}
	max_num_of_pdp += 1;

	id = of_alias_get_id(dev->of_node, "pdp");
	leader = of_property_read_bool(dev->of_node, "leader");

	hw_id = DEV_HW_PAF0 + id;
	pdp = devm_kzalloc(&pdev->dev, sizeof(struct is_pdp), GFP_KERNEL);
	if (!pdp) {
		dev_err(dev, "failed to alloc memory for pdp\n");
		return -ENOMEM;
	}

	/* LIC LUT */
	lic_lut_np = of_parse_phandle(dev->of_node, "lic_lut", 0);
	if (lic_lut_np) {
		ret = of_property_read_u32(lic_lut_np, "lic_mode_default", &pdp->lic_mode_def);
		if (ret) {
			dev_warn(dev, "lic_mode_default read is fail(%d)", ret);
			pdp->lic_mode_def = 0;
		}

		ret = of_property_read_u32(lic_lut_np, "lic_14bit", &pdp->lic_14bit);
		if (ret) {
			dev_warn(dev, "lic_14bit read is fail(%d)", ret);
			pdp->lic_14bit = 0;
		}

		ret = of_property_read_u32(lic_lut_np, "lic_dma_only", &pdp->lic_dma_only);
		if (ret) {
			dev_warn(dev, "lic_dma_only read is fail(%d)", ret);
			pdp->lic_dma_only = 0;
		}

		num_scen = of_get_child_count(lic_lut_np);

		pdp->lic_lut = devm_kcalloc(&pdev->dev, num_scen, sizeof(struct pdp_lic_lut), GFP_KERNEL);
		if (!pdp->lic_lut) {
			dev_err(dev, "failed to alloc memory for pdp->lic_lut\n");
			return -ENOMEM;
		}

		i = 0;
		for_each_child_of_node(lic_lut_np, scen_np) {
			ret = of_property_read_u32(scen_np, "mode", &pdp->lic_lut[i].mode);
			if (ret) {
				dev_err(dev, "failed to parse mode in LIC LUT\n");
				return -EINVAL;
			}

			ret = of_property_read_u32(scen_np, "param0", &pdp->lic_lut[i].param0);
			if (ret) {
				dev_err(dev, "failed to parse param0 in LIC LUT\n");
				return -EINVAL;
			}

			ret = of_property_read_u32(scen_np, "param1", &pdp->lic_lut[i].param1);
			if (ret) {
				dev_err(dev, "failed to parse param1 in LIC LUT\n");
				return -EINVAL;
			}

			ret = of_property_read_u32(scen_np, "param2", &pdp->lic_lut[i].param2);
			if (ret) {
				dev_err(dev, "failed to parse param2 in LIC LUT\n");
				return -EINVAL;
			}
			i++;
		}
	} else {
		dev_warn(dev, "LIC LUT is not defined in DT\n");
	}

	pdp->id = id;
	pdp->max_num = max_num_of_pdp;
	pdp->prev_instance = IS_STREAM_COUNT;
	pdp->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "can't get memory resource\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				dev_name(dev))) {
		dev_err(dev, "can't request region for resource %pR\n", res);
		return -EBUSY;
	}

	pdp->regs_start = res->start;
	pdp->regs_end = res->end;

	pdp->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!pdp->base) {
		dev_err(dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	hw_ip = CALL_HW_CHAIN_INFO_OPS(&core->hardware, get_hw_ip, hw_id);

	/* alloc sfr dump memory */
	hw_ip->regs_start[REG_SETA] = pdp->regs_start;
	hw_ip->regs_end[REG_SETA] = pdp->regs_end;
	hw_ip->regs[REG_SETA] = pdp->base;

	reg_size = (hw_ip->regs_end[REG_SETA] - hw_ip->regs_start[REG_SETA] + 1);
	hw_ip->sfr_dump[REG_SETA] = kzalloc(reg_size, GFP_KERNEL);
	if (IS_ERR_OR_NULL(hw_ip->sfr_dump[REG_SETA])) {
		serr_hw("sfr dump memory alloc fail", hw_ip);
		goto err_sfr_dump_alloc;
	} else {
		sinfo_hw("sfr dump memory (V/P/S):(%lx/%lx/0x%X)[0x%llX~0x%llX]\n", hw_ip,
			(ulong)hw_ip->sfr_dump[REG_SETA], (ulong)virt_to_phys(hw_ip->sfr_dump[REG_SETA]),
			reg_size, hw_ip->regs_start[REG_SETA], hw_ip->regs_end[REG_SETA]);
	}

	/* Common Register */
	ret = pdp_get_resource_mem_byname(pdev, "common", &pdp->cmn_base, NULL, NULL);
	if (ret && (ret != -ENODEV)) {
		dev_err(dev, "can't get memory resource for cmn_base\n");
		goto err_get_cmn_base;
	}

	/* SYSREG: PDP input mux */
	ret = pdp_get_resource_mem_byname(pdev, "mux", &pdp->mux_base, &pdp->mux_val,
		&pdp->mux_elems);
	if (ret && (ret != -ENODEV)) {
		dev_err(dev, "can't get memory resource for mux_base\n");
		goto err_get_mux_base;
	}

	/* SYSREG: PDP input vc mux */
	ret = pdp_get_resource_mem_byname(pdev, "vc_mux", &pdp->vc_mux_base, &pdp->vc_mux_val,
		&pdp->vc_mux_elems);
	if (ret && (ret != -ENODEV))
		dev_err(dev, "can't get memory resource for vc_mux_base\n");

	/* PDP could support multiple VC inputs */
	pdp->vc_mux_arrs = (pdp->vc_mux_elems / pdp->mux_elems);
	pdp->vc_mux_elems /= pdp->vc_mux_arrs;

	/* SYSREG: PDP input enable */
	ret = pdp_get_resource_mem_byname(pdev, "en", &pdp->en_base, &pdp->en_val,
		&pdp->en_elems);
	if (ret && (ret != -ENODEV))
		dev_warn(dev, "can't get memory resource for en_base\n");

	pdp->irq[PDP_INT1] = platform_get_irq(pdev, 0);
	if (pdp->irq[PDP_INT1] < 0) {
		dev_err(dev, "failed to get INT1 resource: %d\n", pdp->irq[PDP_INT1]);
		ret = pdp->irq[PDP_INT1];
		goto err_get_int1;
	}

	name_len = sizeof(pdp->irq_name[PDP_INT1]);
	snprintf(pdp->irq_name[PDP_INT1], name_len, "%s-%d", dev_name(dev), PDP_INT1);
	ret = pablo_request_irq(pdp->irq[PDP_INT1],
			is_isr_pdp_int1,
			pdp->irq_name[PDP_INT1],
			IRQF_SHARED, hw_ip);
	if (ret) {
		dev_err(dev, "failed to request INT1(%d): %d\n", pdp->irq[PDP_INT1], ret);
		goto err_req_int1;
	}

	pdp->irq[PDP_INT2] = platform_get_irq(pdev, 1);
	if (pdp->irq[PDP_INT2] < 0) {
		dev_err(dev, "failed to get INT2 resource: %d\n", pdp->irq[PDP_INT2]);
		ret = pdp->irq[PDP_INT2];
		goto err_get_int2;
	}

	name_len = sizeof(pdp->irq_name[PDP_INT2]);
	snprintf(pdp->irq_name[PDP_INT2], name_len, "%s-%d", dev_name(dev), PDP_INT2);
	ret = pablo_request_irq(pdp->irq[PDP_INT2],
			is_isr_pdp_int2,
			pdp->irq_name[PDP_INT2],
			IRQF_SHARED, hw_ip);
	if (ret) {
		dev_err(dev, "failed to request INT2(%d): %d\n", pdp->irq[PDP_INT2], ret);
		goto err_req_int2;
	}

	pdp->subdev = devm_kzalloc(&pdev->dev, sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!pdp->subdev) {
		dev_err(dev, "failed to alloc memory for pdp-subdev\n");
		ret = -ENOMEM;
		goto err_alloc_subdev;
	}

	v4l2_subdev_init(pdp->subdev, &subdev_ops);
	v4l2_set_subdevdata(pdp->subdev, pdp);
	snprintf(pdp->subdev->name, V4L2_SUBDEV_NAME_SIZE, "pdp-subdev.%d", pdp->id);

	pdp->pdp_ops = &pdp_ops;

	mutex_init(&pdp->control_lock);

	pdp->wq_stat = alloc_workqueue("pdp-stat0/[H/U]", WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!pdp->wq_stat)
		probe_warn("failed to alloc PDP own workqueue, will be use global one");

	INIT_WORK(&pdp->work_stat[WORK_PDP_STAT0], pdp_worker_stat0);
	INIT_WORK(&pdp->work_stat[WORK_PDP_STAT1], pdp_worker_stat1);

	pdp->icpu_adt = pablo_get_icpu_adt();
	pdp->icpu_dev = pablo_itf_get_icpu_dev();

	platform_set_drvdata(pdev, pdp);

	/* initialize device hardware */
	hw_ip->priv_info = pdp;
	pdp->hw_ip = hw_ip;
	hw_ip->id = hw_id;
	snprintf(hw_ip->name, sizeof(hw_ip->name), "%s%d", "PDP", id);
	hw_ip->ops = &is_hw_pdp_ops;
	hw_ip->itf  = &core->interface;
	hw_ip->itfc = NULL; /* interface_ischain is not neccessary */
	atomic_set(&hw_ip->fcount, 0);
	hw_ip->is_leader = leader;
	atomic_set(&hw_ip->status.Vvalid, V_BLANK);
	atomic_set(&hw_ip->rsccount, 0);
	hw_ip->run_rsc_state = 0;
	init_waitqueue_head(&hw_ip->status.wait_queue);

	for (work_id = WORK_PDP_STAT0; work_id < WORK_PDP_MAX; work_id++)
		init_work_list(&pdp->work_list[work_id], work_id, MAX_WORK_COUNT);

	clear_bit(HW_OPEN, &hw_ip->state);
	clear_bit(HW_INIT, &hw_ip->state);
	clear_bit(HW_RUN, &hw_ip->state);
	clear_bit(HW_TUNESET, &hw_ip->state);

	clear_bit(IS_PDP_SET_PARAM, &pdp->state);
	clear_bit(IS_PDP_SET_PARAM_COPY, &pdp->state);
	clear_bit(IS_PDP_ONESHOT_PENDING, &pdp->state);

	pdp_hw_g_int1_str(pdp->int1_str);
	pdp_hw_g_int2_str(pdp->int2_str);
	pdp_hw_g_int2_rdma_str(pdp->int2_rdma_str);

	is_debug_memlog_alloc_dump(res->start, resource_size(res), hw_ip->name);

	probe_info("%s device probe success\n", dev_name(dev));

	return 0;

err_alloc_subdev:
	pablo_free_irq(pdp->irq[PDP_INT2], hw_ip);
err_get_int2:
err_req_int2:
	pablo_free_irq(pdp->irq[PDP_INT1], hw_ip);
err_get_int1:
err_req_int1:
	if (pdp->en_base)
		devm_iounmap(dev, pdp->en_base);
	devm_iounmap(dev, pdp->mux_base);
err_get_mux_base:
	devm_iounmap(dev, pdp->cmn_base);
err_get_cmn_base:
	kfree(hw_ip->sfr_dump[REG_SETA]);
err_sfr_dump_alloc:
	devm_iounmap(dev, pdp->base);
err_ioremap:
	devm_release_mem_region(dev, res->start, resource_size(res));

	return ret;
}

static const struct of_device_id sensor_paf_pdp_match[] = {
	{
		.compatible = "samsung,sensor-paf-pdp",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sensor_paf_pdp_match);

static struct platform_driver pablo_paf_pdp_platform_driver = {
	.probe = pdp_probe,
	.driver = {
		.name   = "Sensor-PAF-PDP",
		.owner  = THIS_MODULE,
		.of_match_table = sensor_paf_pdp_match,
	}
};

struct platform_driver *pablo_paf_pdp_get_platform_driver(void)
{
	return &pablo_paf_pdp_platform_driver;
}

#ifndef MODULE
static int __init sensor_paf_pdp_init(void)
{
	int ret;

	ret = platform_driver_probe(&pablo_paf_pdp_platform_driver, pdp_probe);
	if (ret)
		err("failed to probe %s driver: %d\n",
			pablo_paf_pdp_platform_driver.driver.name, ret);

	return ret;
}
late_initcall_sync(sensor_paf_pdp_init);
#endif
