/*
 * tracepoint hook handling
 *
 * Copyright (C) 2018 Samsung Electronics Co., Ltd
 * Park Bumgyu <bumgyu.park@samsung.com>
 */

#include "../sched.h"

#include "ems.h"

#include <trace/events/sched.h>
#include <trace/events/ems_debug.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/cpuidle.h>
#include <trace/hooks/binder.h>
#include <trace/hooks/cgroup.h>
#include <trace/hooks/topology.h>

#include "../../../drivers/android/binder_trace.h"
#include "../../../drivers/android/binder_internal.h"

#define TF_ASYNC_BINDER 0x01

/******************************************************************************
 * tracepoint of Android vendor hook                                          *
 ******************************************************************************/
static void ems_hook_select_task_rq_rt(void *data,
			struct task_struct *p, int prev_cpu,
			int sd_flag, int wake_flags, int *new_cpu)
{
	int cpu;

	cpu = ems_select_task_rq_rt(p, prev_cpu, sd_flag, wake_flags);

	*new_cpu = cpu;
	TASK_AVD1_1(p) = cpu;
}

static void ems_hook_find_lowest_rq(void *data,
			struct task_struct *p, struct cpumask *local_cpu_mask,
			int ret, int *lowest_cpu)
{
	*lowest_cpu = frt_find_lowest_rq(p, local_cpu_mask);
}

static void ems_hook_tick_entry(void *data, struct rq *rq)
{
	ems_tick_entry(rq);
}

static void ems_hook_can_migrate_task(void *data,
			struct task_struct *p, int dst_cpu, int *can_migrate)
{
	ems_can_migrate_task(p, dst_cpu, can_migrate);
}

static void ems_hook_find_busiest_queue(void *data, int dst_cpu, struct sched_group *group,
		struct cpumask *env_cpus, struct rq **busiest, int *done)
{
	ems_find_busiest_queue(dst_cpu, group, env_cpus, busiest, done);
}

static void ems_hook_cpu_cgroup_can_attach(void *data,
			struct cgroup_taskset *tset, int *retval)
{
	ems_cpu_cgroup_can_attach(tset, *retval);
}

static void ems_hook_rebalance_domains(void *data,
			struct rq *rq, int *continue_balancing)
{
	*continue_balancing = !ems_load_balance(rq);
}

static void ems_hook_nohz_balancer_kick(void *data,
			struct rq *rq, unsigned int *flags, int *done)
{
	ems_nohz_balancer_kick(rq, flags, done);
}

static void ems_hook_newidle_balance(void *data,
			struct rq *this_rq, struct rq_flags *rf,
			int *pulled_task, int *done)
{
	ems_newidle_balance(this_rq, rf, pulled_task, done);
}

static void ems_hook_post_init_entity_util_avg(void *data, struct sched_entity *se)
{
	ems_post_init_entity_util_avg(se);
}

static void ems_hook_find_new_ilb(void *data, struct cpumask *nohz_idle_cpus_mask, int *ilb)
{
	*ilb = ems_find_new_ilb(nohz_idle_cpus_mask);
}

static void ems_hook_schedule(void *data, struct task_struct *prev,
				struct task_struct *next, struct rq *rq)
{
	ems_schedule(prev, next, rq);
}

static void ems_hook_set_task_cpu(void *data, struct task_struct *p,
				unsigned int new_cpu)
{
	ems_set_task_cpu(p, new_cpu);
}

static void ems_hook_alloc_oem_binder_struct(void *data, struct binder_transaction_data *tr,
				struct binder_transaction *t, struct binder_proc *target_proc)
{
	if ((t->flags & TF_ASYNC_BINDER) && ems_is_rt_policy(current->policy)) {
		t->priority.sched_policy = current->policy;
		t->priority.prio = current->normal_prio;
	}
}

static void ems_hook_update_misfit_status(void *data, struct task_struct *p,
					struct rq *rq, bool *need_update)
{
	ems_update_misfit_status(p, rq, need_update);
}

/******************************************************************************
 * built-in tracepoint                                                        *
 ******************************************************************************/
#if defined (CONFIG_SCHED_EMS_DEBUG)
static void ems_hook_pelt_cfs_tp(void *data, struct cfs_rq *cfs_rq)
{
	trace_sched_load_cfs_rq(cfs_rq);
}

static void ems_hook_pelt_rt_tp(void *data, struct rq *rq)
{
	trace_sched_load_rt_rq(rq);
}

static void ems_hook_pelt_dl_tp(void *data, struct rq *rq)
{
	trace_sched_load_dl_rq(rq);
}

static void ems_hook_pelt_irq_tp(void *data, struct rq *rq)
{
#ifdef CONFIG_HAVE_SCHED_AVG_IRQ
	trace_sched_load_irq(rq);
#endif
}

static void ems_hook_pelt_se_tp(void *data, struct sched_entity *se)
{
	trace_sched_load_se(se);
}

static void ems_hook_sched_overutilized_tp(void *data,
			struct root_domain *rd, bool overutilized)
{
	trace_sched_overutilized(overutilized);
}
#endif

static void ems_hook_arch_set_freq_scale(void *data, const struct cpumask *cpus,
			unsigned long freq,  unsigned long max, unsigned long *scale)
{
	ems_arch_set_freq_scale(cpus, freq, max, scale);
}

int hook_init(void)
{
	int ret;

	ret = register_trace_android_rvh_select_task_rq_rt(ems_hook_select_task_rq_rt, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_find_lowest_rq(ems_hook_find_lowest_rq, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_tick_entry(ems_hook_tick_entry, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_cpu_cgroup_can_attach(ems_hook_cpu_cgroup_can_attach, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_sched_rebalance_domains(ems_hook_rebalance_domains, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_sched_newidle_balance(ems_hook_newidle_balance, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_post_init_entity_util_avg(ems_hook_post_init_entity_util_avg, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_find_new_ilb(ems_hook_find_new_ilb, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_schedule(ems_hook_schedule, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_set_task_cpu(ems_hook_set_task_cpu, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_alloc_oem_binder_struct(ems_hook_alloc_oem_binder_struct, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_update_misfit_status(ems_hook_update_misfit_status, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_sched_nohz_balancer_kick(ems_hook_nohz_balancer_kick, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_can_migrate_task(ems_hook_can_migrate_task, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_rvh_find_busiest_queue(ems_hook_find_busiest_queue, NULL);
	if (ret)
		return ret;

	ret = register_trace_android_vh_arch_set_freq_scale(ems_hook_arch_set_freq_scale, NULL);
	if (ret)
		return ret;

#if defined (CONFIG_SCHED_EMS_DEBUG)
	WARN_ON(register_trace_pelt_cfs_tp(ems_hook_pelt_cfs_tp, NULL));
	WARN_ON(register_trace_pelt_rt_tp(ems_hook_pelt_rt_tp, NULL));
	WARN_ON(register_trace_pelt_dl_tp(ems_hook_pelt_dl_tp, NULL));
	WARN_ON(register_trace_pelt_irq_tp(ems_hook_pelt_irq_tp, NULL));
	WARN_ON(register_trace_pelt_se_tp(ems_hook_pelt_se_tp, NULL));
	WARN_ON(register_trace_sched_overutilized_tp(ems_hook_sched_overutilized_tp, NULL));
#endif

	return 0;
}
