/*
 * Samsung Exynos SoC series NPU driver
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _NPU_SCHEDULER_H_
#define _NPU_SCHEDULER_H_

#include <linux/version.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <soc/samsung/exynos_pm_qos.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/string.h>
#include <linux/thermal.h>

#include "npu-vs4l.h"
#include "npu-common.h"
#include "npu-session.h"
#include "npu-wakeup.h"
#include "npu-util-common.h"
#include "npu-util-autosleepthr.h"

#define NPU_MIN(a, b)		((a) < (b) ? (a) : (b))

static char *npu_scheduler_ip_name[] = {
	"CL0",
	"CL1",
	"CL2",
	"MIF",
	"INT",
	"NPU0",
	"NPU1",
	"DSP",
	"DNC",
};

static char *npu_scheduler_core_name[] = {
	"DSP",
	"DNC",
	"NPU0",
	"NPU1",
};

static int npu_scheduler_ip_pmqos_min[] = {
	PM_QOS_CLUSTER0_FREQ_MIN,
	PM_QOS_CLUSTER1_FREQ_MIN,
	PM_QOS_CLUSTER2_FREQ_MIN,
	PM_QOS_BUS_THROUGHPUT,
	PM_QOS_DEVICE_THROUGHPUT,
#ifdef CONFIG_SOC_S5E9935
	PM_QOS_NPU0_THROUGHPUT,
	PM_QOS_NPU1_THROUGHPUT,
#else
	PM_QOS_NPU_THROUGHPUT,
#endif
	PM_QOS_DSP_THROUGHPUT,
	PM_QOS_DNC_THROUGHPUT,
};

static int npu_scheduler_ip_pmqos_max[] = {
	PM_QOS_CLUSTER0_FREQ_MAX,
	PM_QOS_CLUSTER1_FREQ_MAX,
	PM_QOS_CLUSTER2_FREQ_MAX,
	PM_QOS_BUS_THROUGHPUT_MAX,
	PM_QOS_DEVICE_THROUGHPUT_MAX,
#ifdef CONFIG_SOC_S5E9935
	PM_QOS_NPU0_THROUGHPUT_MAX,
	PM_QOS_NPU1_THROUGHPUT_MAX,
#else
	PM_QOS_NPU_THROUGHPUT_MAX,
#endif
	PM_QOS_DSP_THROUGHPUT_MAX,
	PM_QOS_DNC_THROUGHPUT_MAX,
};

static inline int get_pm_qos_num(char *name, char *name_list[],
		int name_list_num, int pmqos_list[])
{
	int i;

	for (i = 0; i < name_list_num; i++) {
		if (!strcmp(name, name_list[i]))
			return pmqos_list[i];
	}
	return -1;
}

#define get_pm_qos_max(NAME)	\
	get_pm_qos_num(NAME, npu_scheduler_ip_name,	\
	ARRAY_SIZE(npu_scheduler_ip_name), npu_scheduler_ip_pmqos_max)
#define get_pm_qos_min(NAME)	\
	get_pm_qos_num(NAME, npu_scheduler_ip_name,	\
	ARRAY_SIZE(npu_scheduler_ip_name), npu_scheduler_ip_pmqos_min)

/*
 * time domain : us
 */

//#define CONFIG_NPU_SCHEDULER_OPEN_CLOSE
#define CONFIG_NPU_SCHEDULER_START_STOP

#define NPU_SCHEDULER_NAME	"npu-scheduler"
#define NPU_SCHEDULER_DEFAULT_PERIOD	17	/* msec */
#define NPU_SCHEDULER_DEFAULT_AFMLIMIT	800000	/* 800MHz */
#define NPU_SCHEDULER_DEFAULT_TPF	500000	/* maximum 500ms */
#define NPU_SCHEDULER_DEFAULT_REQUESTED_TPF	16667
#define NPU_SCHEDULER_FPS_LOAD_RESET_FRAME_NUM	3
#define NPU_SCHEDULER_BOOST_TIMEOUT	20 /* msec */
#define NPU_SCHEDULER_DEFAULT_IDLE_DELAY	10 /* msec */
#define NPU_SCHEDULER_MAX_IDLE_DELAY	65 /* msec */
#define NPU_SCHEDULER_MAX_DVFS_INFO	50

static char *npu_perf_mode_name[] = {
	"normal",
	"boostonexe",
	"boost",
	"cpu",
	"DN",
	"boostonexemo",
	"boostdlv3",
	"boostblocking",
};

enum {
	/* no minlock argument of NPU_PERF_MODE_NORMAL in dt */
	NPU_PERF_MODE_NORMAL = 0,	/* no boost */
	NPU_PERF_MODE_NPU_BOOST_ONEXE,	/* boost on execution but not KPI mode */
	NPU_PERF_MODE_NPU_BOOST,	/* KPI mode */
	NPU_PERF_MODE_CPU_BOOST,	/* don't care */
	NPU_PERF_MODE_NPU_DN,	/* DSP + NPU mode */
	NPU_PERF_MODE_NPU_BOOST_ONEXE_MO,
	NPU_PERF_MODE_NPU_BOOST_DLV3,
	NPU_PERF_MODE_NPU_BOOST_BLOCKING,	/* KPI mode blocking */
	NPU_PERF_MODE_NUM,
};

#if IS_ENABLED(CONFIG_NPU_WITH_CAM_NOTIFICATION)
#define NPU_SHARED_MIF_PLL_CLK	1066000
#endif

#define NPU_SCHEDULER_DVFS_ARG_NUM	10
/* total arg num should be less than arg limit 16 */
#define NPU_SCHEDULER_DVFS_TOTAL_ARG_NUM	\
	(NPU_SCHEDULER_DVFS_ARG_NUM + NPU_PERF_MODE_NUM - 2)
#define HWACG_ALWAYS_DISABLE	(0x01)
#define	HWACG_STATUS_ENABLE	(0x3A)
#define	HWACG_STATUS_DISABLE	(0x7F)
#define	HWACG_NPU				(0x01)
#define	HWACG_DSP				(0x02)
#define	HWACG_DNC				(0x04)
#define	MO_SCEN_NORMAL			(0x01)
#define	MO_SCEN_PERF			(0x02)
#define	MO_SCEN_G3D_HEAVY		(0x03)
#define	MO_SCEN_G3D_PERF		(0x04)
#define	LLC_MAX_WAYS			(16)

#define NPU_SCH_DEFAULT_VALUE   (INT_MAX)

static char *npu_scheduler_load_policy_name[] = {
	"idle",
	"fps",
	"rq",
	"fps-rq hybrid",	// use fps load for DVFS, use rq load for hi/low speed freq
};

enum {
	NPU_SCHEDULER_LOAD_IDLE = 0,
	NPU_SCHEDULER_LOAD_FPS,
	NPU_SCHEDULER_LOAD_RQ,
	NPU_SCHEDULER_LOAD_FPS_RQ,
};

static char *npu_scheduler_fps_policy_name[] = {
	"min",
	"max",
	"average",
	"average without min, max",
};

enum {
	NPU_SCHEDULER_FPS_MIN = 0,
	NPU_SCHEDULER_FPS_MAX,
	NPU_SCHEDULER_FPS_AVG,
	NPU_SCHEDULER_FPS_AVG2,		/* average without min, max */
};

struct npu_scheduler_control {
	wait_queue_head_t		wq;
	int				result_code;
	atomic_t			result_available;
};

#define NPU_SCHEDULER_CMD_POST_RETRY_INTERVAL	100
#define NPU_SCHEDULER_CMD_POST_RETRY_CNT	10
#define NPU_SCHEDULER_HW_RESP_TIMEOUT		1000

struct npu_scheduler_fps_frame {
	npu_uid_t	uid;
	u32		frame_id;
	s64		start_time;

	struct list_head	list;
};

#define NPU_SCHEDULER_PRIORITY_MIN	0
#define NPU_SCHEDULER_PRIORITY_MAX	255

struct npu_scheduler_fps_load {
	const struct npu_session *session;
	npu_uid_t	uid;
	u64		time_stamp;
	u32		priority;
	u32		bound_id;
	u32		frame_count;
	u32		tfc;		/* temperary frame count */
	s64		tpf;		/* time per frame */
	s64		requested_tpf;	/* tpf for fps */
	unsigned int	old_fps_load;	/* save previous fps load */
	unsigned int	fps_load;	/* 0.01 unit */
	s64		init_freq_ratio;
	u32		mode;
#if IS_ENABLED(CONFIG_NPU_USE_IFD)
	u32		dvfs_unset_time;
#endif
	struct list_head	list;
};

struct npu_scheduler_dvfs_sess_info {
	size_t buf_size;
	u64 	unified_op_id;
	u32 	hids;

	s64 	last_qbuf_usecs;
	s64 	last_dqbuf_usecs;
	s64 	last_run_usecs;
	u32 	last_idle_msecs;

	u32 	req_freq;
	u32 	prev_freq;

	u32 	tpf;
	u64 	load;	// fixed point value : fractional part = 10bit
	u8 dvfs_clk_dn_delay;
	u32 dvfs_unset_time; // msecs
	bool is_unified;
	bool	 is_linked;
	bool	 is_nm_mode;

	struct list_head list;
};

struct npu_scheduler_dvfs_info {
	char			*name;
	struct platform_device	*dvfs_dev;
	u32			activated;
	struct exynos_pm_qos_request	qos_req_min;
#if IS_ENABLED(CONFIG_NPU_USE_PI_DTM)
	struct exynos_pm_qos_request	qos_req_max;
#endif
	struct exynos_pm_qos_request	qos_req_min_dvfs_cmd;
	struct exynos_pm_qos_request	qos_req_max_dvfs_cmd;
	struct exynos_pm_qos_request	qos_req_min_nw_boost;
#if IS_ENABLED(CONFIG_NPU_WITH_CAM_NOTIFICATION)
	struct exynos_pm_qos_request	qos_req_max_cam_noti;
#endif

	s32			cur_freq;
	s32			min_freq;
	s32			max_freq;
	s32			delay;
	s32			limit_min;
	s32			limit_max;
	s32			is_init_freq;
	struct npu_scheduler_governor *gov;
	void			*gov_prop;
	u32			mode_min_freq[NPU_PERF_MODE_NUM];
	s32 curr_up_delay;
	s32 curr_down_delay;

	struct list_head	dev_list;	/* list to governor */
	struct list_head	ip_list;	/* list to scheduler */
};

#define NPU_SCHEDULER_LOAD_WIN_MAX	10
#define NPU_SCHEDULER_DEFAULT_LOAD_WIN_SIZE	1
#define NPU_SCHEDULER_FREQ_INTERVAL	4

#define NPU_DVFS_CMD_LEN	2
enum npu_dvfs_cmd {
	NPU_DVFS_CMD_MIN,
	NPU_DVFS_CMD_MAX
};

struct dvfs_cmd_map {
	char			*name;
	enum npu_dvfs_cmd	cmd;
	u32			freq;
};

struct dvfs_cmd_list {
	char			*name;
	struct dvfs_cmd_map	*list;
	int			count;
};
#define DTM_NM_LIMIT_TH (85000)
#define DTM_NM_LIMIT_FREQ (800000)
#define PID_TARGET_THERMAL (93000)
#define PID_P_GAIN (300)
#define PID_I_GAIN (1)
#define PID_INV_GAIN (300)
#define PID_I_BUF_SIZE (5)
#define PID_PERIOD (1)
#define PID_MIN_FREQ (160000)
#ifdef CONFIG_NPU_USE_PI_DTM_DEBUG
#define PID_DEBUG_CNT (3000)
#endif

#define PID_MAX_FREQ_MARGIN (50000)
#define NPU_MAX_DVFS_TABLE (16)

struct npu_scheduler_governor;
struct npu_scheduler_info {
	struct device	*dev;
	struct npu_device *device;

	u32		enable;
	u32		activated;
	u32		prev_mode;
	u32		mode;
	u32		mode_ref_cnt[NPU_PERF_MODE_NUM];
	u32		llc_mode;
	int		bts_scenindex;

	u64		time_stamp;
	u64		time_diff;
	u32		freq_interval;
	u32		tfi;
	u32		period;

/* gating information */
	u32		used_count;

/* load information */
	u32		load_policy;
	u32		load_window_index;
	u32		load_window_size;
	u32		load_window[NPU_SCHEDULER_LOAD_WIN_MAX];	/* 0.01 unit */
	unsigned int	load;		/* 0.01 unit */
	u64		load_idle_time;

/* idle-based load calculation */
	unsigned int	idle_load;	/* 0.01 unit */

/* FPS-based load calculation */
	struct mutex	fps_lock;
	u32		fps_policy;
	struct list_head fps_frame_list;
	struct list_head fps_load_list;
	unsigned int	fps_load;	/* 0.01 unit */
	u32		tpf_others;

/* RQ-based load calculation */
	struct mutex	rq_lock;
	s64		rq_start_time;
	s64		rq_idle_start_time;
	s64		rq_idle_time;
	unsigned int	rq_load;	/* 0.01 unit */

/* governer information */
	struct list_head gov_list;	/* governor list */
/* IP dvfs information */
	struct list_head ip_list;	/* device list */
/* DVFS command list */
	struct dvfs_cmd_list *dvfs_list;	/* DVFS cmd list */

	struct mutex	exec_lock;
	bool		is_dvfs_cmd;
#if IS_ENABLED(CONFIG_PM_SLEEP)
	struct wakeup_source		*sws;
#endif
	struct workqueue_struct		*sched_wq;
	struct delayed_work		sched_work;
	struct delayed_work		boost_off_work;
#if IS_ENABLED(CONFIG_NPU_ARBITRATION)
	struct wakeup_source		*aws;
	struct workqueue_struct		*arbitration_wq;
	struct delayed_work		arbitration_work;
	wait_queue_head_t		waitq;
#endif
	u32				fw_min_active_cores;
	u32				num_cores_active;
	npu_errno_t			result_code;
	u32				result_value;
	u32		llc_status;
	u32		llc_ways;
	u32		hwacg_status;

/* Frequency boost information only for open() and ioctl(S_FORMAT) */
	int		boost_count;

	struct npu_scheduler_control	sched_ctl;
	u32 saved_console_log_level;
	u32 dd_log_level_preset;
	u32 cpuidle_cnt;
	u32 DD_log_cnt;
	u32	dd_direct_path;
	u32	boost_flag;
#if IS_ENABLED(CONFIG_NPU_USE_LLC)
	u32	wait_hw_boot_flag;
#endif

/* NPU DTM */
#if IS_ENABLED(CONFIG_NPU_USE_PI_DTM)
	struct thermal_zone_device *npu_tzd;
	int	idx_cnt;
	int	th_err_db[PID_I_BUF_SIZE];
	int	curr_thermal;
	int	dtm_curr_freq;
	int	dtm_prev_freq;
	int	debug_log_en;
	u32	pid_en;
	u32	pid_target_thermal;
	int	pid_max_clk;
	int	pid_p_gain;
	int	pid_i_gain;
	int	pid_inv_gain;
	int	pid_period;

#ifdef CONFIG_NPU_USE_PI_DTM_DEBUG
	short	debug_log[PID_DEBUG_CNT][3];
	int	debug_cnt;
	int	debug_dump_cnt;
#endif
#endif

#if IS_ENABLED(CONFIG_NPU_USE_IFD)
//	u32	IFD_en;	//Inter Frame DVFS
	atomic_t	dvfs_queue_cnt;
	atomic_t dvfs_unlock_activated;
	struct list_head dsi_list;

	struct workqueue_struct		*dvfs_wq;
	struct delayed_work		dvfs_work;

	struct mutex	dvfs_info_lock;
#endif
#if IS_ENABLED(CONFIG_NPU_USE_PI_DTM) || IS_ENABLED(CONFIG_NPU_USE_IFD)
	u8	dvfs_NPU_freq_cnt[2];
	int dvfs_NPU_freq[2][NPU_MAX_DVFS_TABLE];
#endif
	struct mutex	param_handle_lock;
};

static inline struct dvfs_cmd_list *get_npu_dvfs_cmd_map(struct npu_scheduler_info *info, const char *cmd_name)
{
	int i;

	for (i = 0; ((info->dvfs_list) + i)->name != NULL; i++) {
		if (!strcmp(((info->dvfs_list) + i)->name, cmd_name))
			return (info->dvfs_list + i);
	}
	return (struct dvfs_cmd_list *)NULL;
}

static inline struct npu_scheduler_dvfs_info *get_npu_dvfs_info(struct npu_scheduler_info *info, const char *ip_name)
{
	struct npu_scheduler_dvfs_info *d;

	list_for_each_entry(d, &info->ip_list, ip_list) {
		if (!strcmp(ip_name, d->name))
			return d;
	}
	return (struct npu_scheduler_dvfs_info *)NULL;
}

#define NPU_SET_ATTR(_name, _category)					\
{									\
	.attr = {.name = #_name, .mode = 0664},				\
	.show = npu_show_attrs_##_category,				\
	.store = npu_store_attrs_##_category,				\
}

#define NPU_SCHEDULER_ATTR(_name)	NPU_SET_ATTR(_name, scheduler)

struct npu_device;

int npu_scheduler_probe(struct npu_device *device);
int npu_scheduler_release(struct npu_device *device);
int npu_scheduler_open(struct npu_device *device);
int npu_scheduler_close(struct npu_device *device);
void npu_scheduler_late_open(struct npu_device *device);
void npu_scheduler_early_close(struct npu_device *device);
int npu_scheduler_resume(struct npu_device *device);
int npu_scheduler_suspend(struct npu_device *device);
int npu_scheduler_start(struct npu_device *device);
int npu_scheduler_stop(struct npu_device *device);
void npu_scheduler_update_sched_param(struct npu_device *device, struct npu_session *session);
int npu_scheduler_set_freq(struct npu_scheduler_dvfs_info *d, void *req, s32 freq);
#if IS_ENABLED(CONFIG_NPU_USE_LLC)
void npu_scheduler_send_wait_info_to_hw(struct npu_session *session,	struct npu_scheduler_info *info);
#endif
npu_s_param_ret npu_scheduler_param_handler(struct npu_session *sess, struct vs4l_param *param);
void npu_scheduler_param_handler_dump(void);
int get_ip_max_freq(struct vs4l_freq_param *param);
void npu_scheduler_activate_peripheral_dvfs(unsigned long freq);
struct npu_scheduler_info *npu_scheduler_get_info(void);
int npu_scheduler_boost_on(struct npu_scheduler_info *info);
int npu_scheduler_boost_off(struct npu_scheduler_info *info);
int npu_scheduler_boost_off_timeout(struct npu_scheduler_info *info, s64 timeout);
int npu_scheduler_enable(struct npu_scheduler_info *info);
int npu_scheduler_disable(struct npu_scheduler_info *info);
void npu_scheduler_system_param_unset(void);
int npu_scheduler_register_session(const struct npu_session *session);
void npu_scheduler_unregister_session(const struct npu_session *session);
#if IS_ENABLED(CONFIG_NPU_USE_PI_DTM)
bool npu_scheduler_get_DTM_flag(void);
#endif
#if IS_ENABLED(CONFIG_NPU_USE_IFD)
void npu_scheduler_DVFS_session_info(struct npu_session *session);
void npu_scheduler_qbuf_DVFS(struct npu_session *session);
void npu_scheduler_dqbuf_DVFS(struct npu_session *session);
#else
void npu_scheduler_gate(struct npu_device *device, struct npu_frame *frame, bool idle);
void npu_scheduler_fps_update_idle(struct npu_device *device, struct npu_frame *frame, bool idle);
void npu_scheduler_rq_update_idle(struct npu_device *device, bool idle);
void npu_scheduler_set_init_freq(struct npu_device *device, npu_uid_t session_uid);
int npu_scheduler_load(struct npu_device *device, const struct npu_session *session);
void npu_scheduler_unload(struct npu_device *device, const struct npu_session *session);
#endif
#endif
