/*
 * Samsung Exynos SoC series Pablo driver
 *
 * Exynos Pablo DVFS v2 functions
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>

#include "pablo-kernel-variant.h"
#include "is-core.h"
#include "is-dvfs.h"
#include "is-hw-dvfs.h"
#include <videodev2_exynos_camera.h>

#ifdef CONFIG_PM_DEVFREQ
static int scenario_idx[IS_DVFS_SN_END];
static int is_hw_dvfs_init(void *dvfs_data)
{
	int i, ret = 0;
	struct is_dvfs_ctrl *dvfs_ctrl;

	dvfs_ctrl = (struct is_dvfs_ctrl *)dvfs_data;

	FIMC_BUG(!dvfs_ctrl);

	dvfs_ctrl->static_ctrl->cur_scenario_id = 0;
	dvfs_ctrl->dynamic_ctrl->cur_scenario_id = 0;
	dvfs_ctrl->dynamic_ctrl->cur_frame_tick = -1;

	for (i = 0; i < IS_DVFS_SN_END; i++)
		scenario_idx[is_dvfs_dt_arr[i].scenario_id] = i;

	return ret;
}

static char *is_hw_dvfs_get_scenario_name(int id)
{
	return (char *)is_dvfs_dt_arr[scenario_idx[id]].parse_scenario_nm;
}

static int is_hw_dvfs_get_frame_tick(int id)
{
	return is_dvfs_dt_arr[scenario_idx[id]].keep_frame_tick;
}

int is_dvfs_init(struct is_resourcemgr *resourcemgr)
{
	int ret = 0;
	struct is_dvfs_ctrl *dvfs_ctrl;
	u32 dvfs_t;

	FIMC_BUG(!resourcemgr);

	dvfs_ctrl = &resourcemgr->dvfs_ctrl;

	for (dvfs_t = 0; dvfs_t < IS_DVFS_END; dvfs_t++)
		dvfs_ctrl->cur_lv[dvfs_t] = IS_DVFS_LV_END;

	dvfs_ctrl->cur_hpg_qos = 0;
	dvfs_ctrl->cur_hmp_bst = 0;
	dvfs_ctrl->cur_cpus = NULL;

	/* init spin_lock for clock gating */
	mutex_init(&dvfs_ctrl->lock);

	if (!(dvfs_ctrl->static_ctrl))
		dvfs_ctrl->static_ctrl =
			kzalloc(sizeof(struct is_dvfs_scenario_ctrl), GFP_KERNEL);
	if (!(dvfs_ctrl->dynamic_ctrl))
		dvfs_ctrl->dynamic_ctrl =
			kzalloc(sizeof(struct is_dvfs_scenario_ctrl), GFP_KERNEL);

	if (!dvfs_ctrl->static_ctrl || !dvfs_ctrl->dynamic_ctrl) {
		err("dvfs_ctrl alloc is failed!!\n");
		return -ENOMEM;
	}

	/* assign static / dynamic scenario check logic data */
	ret = is_hw_dvfs_init((void *)dvfs_ctrl);
	if (ret) {
		err("is_hw_dvfs_init is failed(%d)\n", ret);
		return -EINVAL;
	}

	/* initialize bundle operating variable */
	dvfs_ctrl->bundle_operating = false;

	return 0;
}

int is_dvfs_sel_static(struct is_device_ischain *device)
{
	struct is_dvfs_ctrl *dvfs_ctrl;
	struct is_dvfs_scenario_ctrl *static_ctrl;
	struct is_resourcemgr *resourcemgr;
	int scenario_id;

	FIMC_BUG(!device);
	FIMC_BUG(!device->interface);

	resourcemgr = device->resourcemgr;
	dvfs_ctrl = &(resourcemgr->dvfs_ctrl);
	static_ctrl = dvfs_ctrl->static_ctrl;

	/* static scenario */
	if (!static_ctrl) {
		err("static_dvfs_ctrl is NULL");
		return -EINVAL;
	}

	dbg_dvfs(1, "[DVFS] %s setfile: %d, scen: %d, common: %d, vendor: %d\n", device, __func__,
		device->setfile & IS_SETFILE_MASK, (device->setfile & IS_SCENARIO_MASK) >> IS_SCENARIO_SHIFT,
		dvfs_ctrl->dvfs_scenario & IS_DVFS_SCENARIO_COMMON_MODE_MASK,
		(dvfs_ctrl->dvfs_scenario >> IS_DVFS_SCENARIO_VENDOR_SHIFT) & IS_DVFS_SCENARIO_VENDOR_MASK);

	scenario_id = is_hw_dvfs_get_scenario(device, IS_DVFS_STATIC);
	if (scenario_id < 0)
		scenario_id = IS_DVFS_SN_DEFAULT;

	static_ctrl->cur_scenario_id = scenario_id;
	static_ctrl->cur_frame_tick = -1;
	static_ctrl->scenario_nm = is_hw_dvfs_get_scenario_name(scenario_id);

	return scenario_id;
}

int is_dvfs_sel_dynamic(struct is_device_ischain *device, struct is_group *group, struct is_frame *frame)
{
	struct is_dvfs_ctrl *dvfs_ctrl;
	struct is_dvfs_scenario_ctrl *dynamic_ctrl;
	struct is_dvfs_scenario_ctrl *static_ctrl;
	struct is_resourcemgr *resourcemgr;
	int scenario_id;
	int vendor;

	FIMC_BUG(!device);

	/* Skip dynamic DVFS update when it's sensor group to minimize LATE_SHOT */
	if (group->device_type == IS_DEVICE_SENSOR && group->tail->next)
		return 0;

	resourcemgr = device->resourcemgr;
	dvfs_ctrl = &(resourcemgr->dvfs_ctrl);
	dynamic_ctrl = dvfs_ctrl->dynamic_ctrl;
	static_ctrl = dvfs_ctrl->static_ctrl;

	/* dynamic scenario */
	if (!dynamic_ctrl) {
		err("dynamic_dvfs_ctrl is NULL");
		return -EINVAL;
	}

	if (dynamic_ctrl->cur_frame_tick >= 0) {
		(dynamic_ctrl->cur_frame_tick)--;
		/*
		 * when cur_frame_tick is lower than 0, clear current scenario.
		 * This means that current frame tick to keep dynamic scenario
		 * was expired.
		 */
		if (dynamic_ctrl->cur_frame_tick < 0) {
			dynamic_ctrl->cur_scenario_id = -1;
		}
	}

	vendor = (dvfs_ctrl->dvfs_scenario >> IS_DVFS_SCENARIO_VENDOR_SHIFT) & IS_DVFS_SCENARIO_VENDOR_MASK;

	if (!test_bit(IS_ISCHAIN_REPROCESSING, &device->state) || group->id == GROUP_ID_VRA0
		|| vendor == IS_DVFS_SCENARIO_VENDOR_SSM)
		return -EAGAIN;

	scenario_id = is_hw_dvfs_get_scenario(device, IS_DVFS_DYNAMIC);
	if (scenario_id < 0)
		scenario_id = IS_DVFS_SN_DEFAULT;

	dynamic_ctrl->cur_scenario_id = scenario_id;
	dynamic_ctrl->cur_frame_tick = is_hw_dvfs_get_frame_tick(scenario_id);
	dynamic_ctrl->scenario_nm = is_hw_dvfs_get_scenario_name(scenario_id);

	return	scenario_id;
}

int is_dvfs_get_cur_lv(u32 dvfs_t)
{
	struct is_core *core;
	struct is_dvfs_ctrl *dvfs_ctrl;
	u32 cur_lv;

	core = is_get_is_core();
	dvfs_ctrl = &core->resourcemgr.dvfs_ctrl;
	cur_lv = dvfs_ctrl->cur_lv[dvfs_t];

	if (cur_lv >= IS_DVFS_LV_END) {
		warn("invalid level[%s(L%d)]", core->pdata->qos_name[dvfs_t], cur_lv);
		return -EINVAL;
	}

	return cur_lv;
}

int is_dvfs_get_freq(u32 dvfs_t)
{
	struct is_core *core;
	struct is_dvfs_ctrl *dvfs_ctrl;
	u32 cur_lv;

	core = is_get_is_core();
	dvfs_ctrl = &core->resourcemgr.dvfs_ctrl;
	cur_lv = dvfs_ctrl->cur_lv[dvfs_t];

	if (cur_lv >= IS_DVFS_LV_END) {
		warn("invalid level[%s(L%d)]", core->pdata->qos_name[dvfs_t], cur_lv);
		return 0;
	}

	return core->pdata->qos_tb[dvfs_t][cur_lv][IS_DVFS_VAL_FRQ];
}

static inline int is_get_qos_lv(struct is_core *core, u32 type, u32 scenario_id)
{
	struct exynos_platform_is *pdata = NULL;

	pdata = core->pdata;
	if (pdata == NULL) {
		err("pdata is NULL\n");
		return -EINVAL;
	}

	if (type >= IS_DVFS_END) {
		err("Cannot find DVFS value");
		return -EINVAL;
	}

	if (scenario_id >= IS_DVFS_SN_END) {
		err("invalid scenario_id(%d)", scenario_id);
		return -EINVAL;
	}

	return pdata->dvfs_data[scenario_id][type];
}

static inline u32 *is_get_qos_table(struct is_core *core, u32 type, u32 scenario_id)
{
	int lv;
	u32 *qos_tb;
	const char *name = core->pdata->qos_name[type];

	if (!name)
		return NULL;

	lv = is_get_qos_lv(core, type, scenario_id);
	if (lv < 0) {
		err("fail is_get_qos_lv()");
		return NULL;
	}

	if (lv >= IS_DVFS_LV_END)
		return NULL;

	qos_tb = core->pdata->qos_tb[type][lv];

	pr_cont("[%s(%d:%d)]", name, lv, qos_tb[IS_DVFS_VAL_FRQ]);

	return qos_tb;
}

static inline const char *is_get_cpus(struct is_core *core, u32 scenario_id)
{
	struct exynos_platform_is *pdata;

	pdata = core->pdata;
	if (pdata == NULL) {
		err("pdata is NULL\n");
		return NULL;
	}

	if (scenario_id >= IS_DVFS_SN_END) {
		err("invalid scenario_id(%d)", scenario_id);
		return NULL;
	}

	return pdata->dvfs_cpu[scenario_id];
}

static inline void _is_add_dvfs(struct is_core *core, int scenario_id,
				struct is_pm_qos_request *exynos_isp_pm_qos,
				u32 qos_thput,
				u32 dvfs_t)
{
	struct is_dvfs_ctrl *dvfs_ctrl;
	u32 *qos_tb;
	u32 qos, lv;

	qos_tb = is_get_qos_table(core, dvfs_t, scenario_id);
	if (!qos_tb)
		return;

	dvfs_ctrl = &core->resourcemgr.dvfs_ctrl;
	qos = qos_tb[IS_DVFS_VAL_QOS];
	lv = qos_tb[IS_DVFS_VAL_LEV];
	if (qos > 0 && !is_pm_qos_request_active(&exynos_isp_pm_qos[dvfs_t])) {
		is_pm_qos_add_request(&exynos_isp_pm_qos[dvfs_t], qos_thput, qos);
		dvfs_ctrl->cur_lv[dvfs_t] = lv;
	}
}

static inline void _is_remove_dvfs(struct is_core *core, int scenario_id,
				struct is_pm_qos_request *exynos_isp_pm_qos,
				u32 dvfs_t)
{
	u32 *qos_tb;
	u32 qos;

	qos_tb = is_get_qos_table(core, dvfs_t, scenario_id);
	if (!qos_tb)
		return;

	qos = qos_tb[IS_DVFS_VAL_QOS];
	if (qos > 0)
		is_pm_qos_remove_request(&exynos_isp_pm_qos[dvfs_t]);
}

static inline void _is_set_dvfs(struct is_dvfs_ctrl *dvfs_ctrl, u32 *qos_tb, u32 dvfs_t)
{
	struct is_pm_qos_request *exynos_isp_pm_qos;
	u32 qos, lv;

	if (!qos_tb)
		return;

	qos = qos_tb[IS_DVFS_VAL_QOS];
	lv = qos_tb[IS_DVFS_VAL_LEV];

	exynos_isp_pm_qos = is_get_pm_qos();

	if (qos && dvfs_ctrl->cur_lv[dvfs_t] != lv) {
		is_pm_qos_update_request(&exynos_isp_pm_qos[dvfs_t], qos);
		dvfs_ctrl->cur_lv[dvfs_t] = lv;
	}
}

int is_add_dvfs(struct is_core *core, int scenario_id)
{
	struct is_pm_qos_request *exynos_isp_pm_qos;
	u32 qos_thput[IS_DVFS_END];
	u32 dvfs_t;

	info("[RSC] %s", __func__);

	exynos_isp_pm_qos = is_get_pm_qos();

	if (IS_ENABLED(CONFIG_EXYNOS_PM_QOS))
		is_hw_dvfs_get_qos_throughput(qos_thput);

	for (dvfs_t = 0; dvfs_t < IS_DVFS_END; dvfs_t++)
		_is_add_dvfs(core, scenario_id, exynos_isp_pm_qos, qos_thput[dvfs_t], dvfs_t);

	return 0;
}

int is_remove_dvfs(struct is_core *core, int scenario_id)
{

	struct is_pm_qos_request *exynos_isp_pm_qos;
	u32 dvfs_t;

	exynos_isp_pm_qos = is_get_pm_qos();

	info("[RSC] %s", __func__);

	for (dvfs_t = 0; dvfs_t < IS_DVFS_END; dvfs_t++)
		_is_remove_dvfs(core, scenario_id, exynos_isp_pm_qos, dvfs_t);

	return 0;
}

static void is_pm_qos_update_bundle(struct is_core *core,
		struct is_device_ischain *device, u32 scenario_id)
{
	struct is_dvfs_ctrl *dvfs_ctrl;
	u32 nums = 0;
	u32 types[8] = {0, };
	u32 scns[8] = {0, };
	u32 *qos_tb;
	int i;

	dvfs_ctrl = &core->resourcemgr.dvfs_ctrl;

	if (is_hw_dvfs_get_bundle_update_seq(scenario_id, &nums, types,
				scns, &dvfs_ctrl->bundle_operating)) {
		for (i = 0; i < nums; i++) {
			minfo("DVFS bundle[%d]: ", device, i);
			qos_tb = is_get_qos_table(core, types[i], scns[i]);
			_is_set_dvfs(dvfs_ctrl, qos_tb, types[i]);
		}
	}
}

static bool _is_get_enabled_thrott_domain(struct is_dvfs_ctrl *dvfs_ctrl, u32 dvfs_t)
{
	if (test_bit(IS_DVFS_TICK_THROTT, &dvfs_ctrl->state)) {
		if (dvfs_t == IS_DVFS_INT_CAM &&
			IS_ENABLED(THROTTLING_INTCAM_ENABLE))
			return true;
		if (dvfs_t == IS_DVFS_MIF &&
			IS_ENABLED(THROTTLING_MIF_ENABLE))
			return true;
		if (dvfs_t == IS_DVFS_INT &&
			IS_ENABLED(THROTTLING_INT_ENABLE))
			return true;
	}

	return false;
}

int is_set_dvfs(struct is_core *core, struct is_device_ischain *device, int scenario_id)
{
	int ret = 0;
	u32 dvfs_t;
	u32 *qos_tb[IS_DVFS_END];
	int hpg_qos;
	const char *cpus;
	struct is_resourcemgr *resourcemgr;
	struct is_dvfs_ctrl *dvfs_ctrl;
	struct is_pm_qos_request *exynos_isp_pm_qos;
#if IS_ENABLED(CONFIG_SCHED_EMS_TUNE)
	struct emstune_mode_request *emstune_req;
#endif
	u32 lv;

	if (core == NULL) {
		err("core is NULL\n");
		return -EINVAL;
	}

	if (scenario_id < 0) {
		err("scenario is not valid\n");
		return -EINVAL;
	}

	if (device)
		is_pm_qos_update_bundle(core, device, scenario_id);

	resourcemgr = &core->resourcemgr;
	dvfs_ctrl = &(resourcemgr->dvfs_ctrl);
	exynos_isp_pm_qos = is_get_pm_qos();

	memset(qos_tb, 0, sizeof(qos_tb));

	info("[RSC:%d] %s", device ? device->instance : 0, __func__);

	/* General PM QOS */
	for (dvfs_t = 0; dvfs_t < IS_DVFS_END; dvfs_t++) {
		if (!core->pdata->qos_name[dvfs_t])
			continue;

		if (resourcemgr->limited_fps && _is_get_enabled_thrott_domain(dvfs_ctrl, dvfs_t))
			scenario_id = IS_DVFS_SN_THROTTLING;

		qos_tb[dvfs_t] = is_get_qos_table(core, dvfs_t, scenario_id);
		if (!qos_tb[dvfs_t]) {
			lv = is_hw_dvfs_get_lv(device, dvfs_t);
			if (lv < IS_DVFS_LV_END) {
				qos_tb[dvfs_t] = core->pdata->qos_tb[dvfs_t][lv];
				pr_cont("[%s(%d:%d)]", core->pdata->qos_name[dvfs_t],
					lv, qos_tb[dvfs_t][IS_DVFS_VAL_FRQ]);
			}
		}
	}

	for (dvfs_t = 0; dvfs_t < IS_DVFS_END; dvfs_t++)
		_is_set_dvfs(dvfs_ctrl, qos_tb[dvfs_t], dvfs_t);

	/* Others */
	hpg_qos = is_get_qos_lv(core, IS_DVFS_HPG, scenario_id);
#if IS_ENABLED(CONFIG_SCHED_EMS_TUNE)
	/* hpg_qos : number of minimum online CPU */
	if (hpg_qos > 0 && device && (dvfs_ctrl->cur_hpg_qos != hpg_qos)
		&& !test_bit(IS_ISCHAIN_REPROCESSING, &device->state)) {
		dvfs_ctrl->cur_hpg_qos = hpg_qos;

		emstune_req = is_get_emstune_req();
		/* for migration to big core */
		if (hpg_qos > 4) {
			if (!dvfs_ctrl->cur_hmp_bst) {
				minfo("BOOST on\n", device);
				emstune_boost(emstune_req, 1);
				dvfs_ctrl->cur_hmp_bst = 1;
			}
		} else {
			if (dvfs_ctrl->cur_hmp_bst) {
				minfo("BOOST off\n", device);
				emstune_boost(emstune_req, 0);
				dvfs_ctrl->cur_hmp_bst = 0;
			}
		}
	}
#endif

	cpus = is_get_cpus(core, dvfs_ctrl->static_ctrl->cur_scenario_id);
	if (cpus && (!dvfs_ctrl->cur_cpus || strcmp(dvfs_ctrl->cur_cpus, cpus)))
		dvfs_ctrl->cur_cpus = cpus;

	info("[HPG(%d, %d),CPU(%s),L_FPS(%d)]\n",
		hpg_qos, dvfs_ctrl->cur_hmp_bst,
		cpus ? cpus : "",
		resourcemgr->limited_fps);

	return ret;
}

int pablo_set_static_dvfs(struct is_device_ischain *device,
			  const char *suffix, int force_scn)
{
	struct is_sysfs_debug *sysfs_debug = is_get_sysfs_debug();
	struct is_dvfs_ctrl *dvfs_ctrl = &device->resourcemgr->dvfs_ctrl;
	int scn;

	if (!IS_ENABLED(ENABLE_DVFS) || !sysfs_debug->en_dvfs)
		return -EINVAL;

	if (is_pm_qos_request_active(&device->user_qos))
		return -EINVAL;

	mutex_lock(&dvfs_ctrl->lock);

	scn = is_dvfs_sel_static(device);
	if (scn >= IS_DVFS_SN_DEFAULT) {
		minfo("[ISC:D] static scenario(%d)-[%s%s]\n", device, scn,
				dvfs_ctrl->static_ctrl->scenario_nm, suffix);

		is_set_dvfs((struct is_core *)device->interface->core, device,
				force_scn < IS_DVFS_SN_END ? force_scn : scn);
	}

	mutex_unlock(&dvfs_ctrl->lock);

	return scn;
}

void is_set_static_dvfs(struct is_device_ischain *device, bool stream_on, bool use_fasten_ae)
{
	struct is_resourcemgr *rscmgr = device->resourcemgr;
	struct is_dvfs_ctrl *dvfs_ctrl = &rscmgr->dvfs_ctrl;
	int scn = IS_DVFS_SN_DEFAULT;

	if (stream_on) {
		if (use_fasten_ae)
			scn = pablo_set_static_dvfs(device, "FAST_LAUNCH", IS_DVFS_SN_DEFAULT);
		else
			scn = pablo_set_static_dvfs(device, "", IS_DVFS_SN_END);

		/* set bts_scen by scenario, even though it is error value */
		is_hw_configure_bts_scen(rscmgr, scn);
	}

	if (scn >= IS_DVFS_SN_DEFAULT) {
		mutex_lock(&dvfs_ctrl->lock);

		if (!stream_on)
			scn = is_dvfs_sel_static(device);

		if (scn >= IS_DVFS_SN_DEFAULT)
			dvfs_ctrl->dynamic_ctrl->cur_frame_tick = KEEP_FRAME_TICK_FAST_LAUNCH;

		mutex_unlock(&dvfs_ctrl->lock);
	}
}

unsigned int is_get_bit_count(unsigned long bits)
{
	unsigned int count = 0;

	while (bits) {
		bits &= (bits - 1);
		count++;
	}

	return count;
}
#endif
