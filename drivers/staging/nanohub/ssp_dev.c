/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */
#include "ssp.h"
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

//#include <linux/sec_debug.h>

#ifdef CONFIG_SSP_RTC
#include <linux/time.h>
#endif
#ifdef CONFIG_RTC_TEE_TIMESTAMP
#include <linux/smc.h>
#endif
#ifdef CONFIG_SENSORS_SSP_VIB_NOTIFY
#include "ssp_motor.h"
#endif
#include "ssp_iio.h"
#include "ssp_sensorlist.h"
#include "ssp_scontext.h"
#include "ssp_lpm.h"
#include "ssp_dump.h"
#include "ssp_bigdata.h"
#if IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
struct notifier_block panel_notif;
#endif

#define NORMAL_SENSOR_STATE_K	0x3FEFF
struct ssp_data *sensorhub_data;


static void init_sensorlist(struct ssp_data *data)
{
	struct sensor_info_ssp sensorinfo[SENSOR_TYPE_MAX] = {
		SENSOR_INFO_ACCELEROMETER,
		SENSOR_INFO_GYRO,
		SENSOR_INFO_ACCELEROMETER_32G,
		SENSOR_INFO_DIGITAL_HALL_RAW,
		SENSOR_INFO_DIGITAL_HALL_EVENT,
		SENSOR_INFO_PRESSURE,
		SENSOR_INFO_GESTURE,
		SENSOR_INFO_DIGITAL_HALL_SUB_RAW,
		SENSOR_INFO_DIGITAL_HALL_EVENT_15,
		SENSOR_INFO_LIGHT,
		SENSOR_INFO_AUTO_BRIGHTNESS,
		SENSOR_INFO_ORIENTATION,
		SENSOR_INFO_ROTATION_VECTOR,
		SENSOR_INFO_GAME_ROTATION_VECTOR,
		SENSOR_INFO_GYRO_UNCALIBRATED,
		SENSOR_INFO_LINEAR_ACCEL,
		SENSOR_INFO_AOD,
		SENSOR_INFO_HRM_ECG_LIB,
		SENSOR_INFO_HRM_RAW,
		SENSOR_INFO_HRM_RAW_FAC,
		SENSOR_INFO_HRM_LIB,
		SENSOR_INFO_HRM_RAW_FAC2,
		SENSOR_INFO_HRM_BIA_RAW,
		SENSOR_INFO_TEMPERATURE,
		SENSOR_INFO_GEOMAGNETIC_UNCALIB,
		SENSOR_INFO_GEOMAGNETIC_CALIB,
		SENSOR_INFO_GEOMAGNETIC_POWER,
		SENSOR_INFO_SENSOR27,
		SENSOR_INFO_SENSOR28,
		SENSOR_INFO_LIGHT_CCT,
		SENSOR_INFO_GRIP,
		SENSOR_INFO_GRIP_RAW,
		SENSOR_INFO_SCONTEXT,
	};

	memcpy(&data->info, sensorinfo, sizeof(data->info));
}

static void initialize_variable_iio(struct ssp_data *data)
{
	int type;
	pr_info("[SSP] %s start\n", __func__);

	init_sensorlist(data);

	for (type = 0; type < SENSOR_TYPE_MAX; type++) {
		data->delay[type].sampling_period = DEFAULT_POLLING_DELAY;
		data->delay[type].max_report_latency = 0;
	}
}

void ssp_enable(struct ssp_data *data, bool enable)
{
	pr_info("[SSP] %s, enable = %d, old enable = %d\n",
		__func__, enable, data->bSspShutdown);

	if (enable && data->bSspShutdown)
		data->bSspShutdown = false;
	else if (!enable && !data->bSspShutdown)
		data->bSspShutdown = true;
	else
		pr_err("[SSP] %s, enable error\n", __func__);
}

/*************************************************************************/
/* initialize sensor hub						 */
/*************************************************************************/

static void initialize_variable(struct ssp_data *data)
{
	int iSensorIndex;

	for (iSensorIndex = 0; iSensorIndex < SENSOR_MAX; iSensorIndex++) {
		data->adDelayBuf[iSensorIndex] = DEFAULT_POLLING_DELAY;
		data->batchLatencyBuf[iSensorIndex] = 0;
		data->batchOptBuf[iSensorIndex] = 0;
		data->aiCheckStatus[iSensorIndex] = INITIALIZATION_STATE;
		data->reportedData[iSensorIndex] = false;
	}
	data->adDelayBuf[HRM_LIB_SENSOR] = (100 * NSEC_PER_MSEC);

#ifdef CONFIG_SENSORS_SSP_HALLIC_SENSOR
	data->adDelayBuf[DIGITAL_HALL_EVENT_SENSOR] = (10 * NSEC_PER_MSEC);
	data->adDelayBuf[DIGITAL_HALL_EVENT_SENSOR_15] = (10 * NSEC_PER_MSEC);
#endif

	atomic64_set(&data->aSensorEnable, 0);
	data->uSensorState = NORMAL_SENSOR_STATE_K;

	data->is_reset_started = false;
	data->uResetCnt = UINT_MAX;
	data->uTimeOutCnt = 0;
	data->uComFailCnt = 0;
	data->uIrqCnt = 0;

	data->bFirstBoot = true;
	data->bSspShutdown = true;
	data->bSspReady = false;
	data->bGeomagneticRawEnabled = false;
	data->bAccelAlert = false;
	data->bLpModeEnabled = false;
	data->bTimeSyncing = true;

	data->accelcal.x = 0;
	data->accelcal.y = 0;
	data->accelcal.z = 0;

	data->gyrocal.x = 0;
	data->gyrocal.y = 0;
	data->gyrocal.z = 0;

	data->magoffset.x = 0;
	data->magoffset.y = 0;
	data->magoffset.z = 0;

	data->iPressureCal = 0;
	data->sealevelpressure = 0;

	data->uGyroDps = GYROSCOPE_DPS2000;

	memset(data->hrmcal, 0x00, sizeof(data->hrmcal));

	data->mcu_device = NULL;
	data->acc_device = NULL;
	data->gyro_device = NULL;
	data->mag_device = NULL;
	data->prs_device = NULL;
	data->light_device = NULL;
	data->hrm_device = NULL;
	data->hrm_ecg_device = NULL;
	data->front_hrm_device = NULL;
	data->uv_device = NULL;
	data->gsr_device = NULL;
	data->ecg_device = NULL;
	data->bia_device = NULL;

#ifdef CONFIG_SENSORS_SSP_VIB_NOTIFY
	data->motor_duration = 0;
	data->motor_flag = VIB_EVENT_STOP;
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
	data->uMagCntlRegData = 1;
	data->geomag_cntl_regdata = 1;
	data->bGeomagneticRawEnabled = false;
#endif

#ifdef CONFIG_SENSORS_SSP_BIA_SENSOR
	for (iSensorIndex = 0; iSensorIndex < BIA_CAL_DATA_SIZE; iSensorIndex++) {
		data->biacal[iSensorIndex] = -1;
	}
#endif

#if IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
	data->ub_con_state = SSP_UB_CON_UNKNOWN;
	memset(&data->panel_event_data, 0, sizeof(struct panel_event_bl_data));
#endif
	data->uLastResumeState = SSP_AP_STATUS_RESUME;
	data->uLogWIdx = 0xFFFFFFFF;
	data->hrm_vendor = 0xFF;
#ifdef CONFIG_SENSORS_SSP_HEART
	data->afe_sel_reading = 0xFF;
#endif

#if IS_ENABLED(CONFIG_SEC_FACTORY)
	data->bin_type = FACTORY_BIN;
#else
	data->bin_type = USER_BIN;
#endif

	memset(&data->crash_mini_dump, 0x00, sizeof(data->crash_mini_dump));
	memset(data->uSsrStuckInfo, 0x00, sizeof(char) * SENSOR_STUCK_INFO_SIZE);

	INIT_LIST_HEAD(&data->pending_list);

	/* file manager */
	init_completion(&data->fm_read_done);
	init_completion(&data->fm_write_done);
	data->fm_tx_buffer_length = 0;
	memset(data->fm_tx_buffer, 0, sizeof(data->fm_tx_buffer));
	memset(data->fm_rx_buffer, 0, sizeof(data->fm_rx_buffer));

	memset(data->msgBeforWakeup, 0x00, sizeof(data->msgBeforWakeup));
	data->msgBeforWakeupCnt = 0;
	data->wrongWakeupCnt = 0;
	data->prevTick = 0;
	if (data->pMsgBeforWakeup != NULL)
		memset(data->pMsgBeforWakeup, 0x00, 1024);

	data->panic_enable = 0;

	initialize_function_pointer(data);
}

int initialize_mcu(struct ssp_data *data)
{
	int iRet = 0;

	clean_pending_list(data);

	data->cnt_timeout = 0;
	data->cnt_com_fail = 0;

	iRet = get_chipid(data);
	pr_info("[SSP] MCU device ID = %d, reading ID = %d\n", DEVICE_ID, iRet);

	if (iRet != DEVICE_ID) {
		iRet = get_chipid(data);
		pr_info("[SSP] Retry MCU device ID = %d, reading ID = %d\n",
			DEVICE_ID, iRet);

		if (iRet != DEVICE_ID) {
			if (iRet < 0) {
				pr_err("[SSP]: %s - MCU is not working : 0x%x\n",
					__func__, iRet);
			} else {
				pr_err("[SSP]: %s - MCU identification failed\n",
					__func__);
				iRet = -ENODEV;
			}
			goto out;
		}
	}

	iRet = set_sensor_position(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - set_sensor_position failed\n", __func__);
//		goto out;
	}

	iRet = set_model_number(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - set_model_number failed\n", __func__);
	}

#ifdef CONFIG_SENSORS_SSP_LUCKY
	iRet = set_lcd_panel_type(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - sending lcd type data failed\n", __func__);
	}
#endif

	data->uSensorState = get_sensor_scanning_info(data);
	if (data->uSensorState == 0) {
		pr_err("[SSP]: %s - get_sensor_scanning_info failed\n",
			__func__);
		iRet = ERROR;
		goto out;
	}

#ifdef CONFIG_SENSORS_SSP_LUCKY
	data->uFeatureList = get_feature_list(data);
	if (data->uFeatureList == 0)
		pr_err("[SSP]: %s - get_feature_list failed\n", __func__);
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
	iRet = initialize_magnetic_sensor(data);
	if (iRet < 0)
		pr_err("[SSP]: %s - initialize magnetic sensor failed\n",
			__func__);
#endif

	data->uCurFirmRev = get_firmware_rev(data);
	pr_info("[SSP] MCU Firm Rev : New = %8u\n",
		data->uCurFirmRev);

	pr_err("[SSP] light-circle coord,x=%d,y=%d,z=%d", data->lightCircleBuf[0],
		data->lightCircleBuf[1], data->lightCircleBuf[2]);

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	set_lpm_mode(data);
#endif

out:
	return iRet;
}

int ssp_regulator_ctrl(struct regulator *reg,
		bool enable, int on_delay_ms)
{
	int ret;

	if (!reg) {
		pr_err("[SSP] can't find regulator for ctrl!\n");
		return -EINVAL;
	}

	if (enable) {
		ret = regulator_enable(reg);
		msleep(on_delay_ms);
	} else {
		ret = regulator_disable(reg);
	}

	if (ret)
		pr_err("[SSP] fail to regulator ctrl(%d) for %s\n",
			ret, enable ? "enable" : "disable");

	return ret;
}


#ifdef CONFIG_OF
static int ssp_parse_dt(struct device *dev,
	struct ssp_data *data)
{
	struct device_node *np = dev->of_node;
	int errorno = 0;
	const char *vreg_name;

	if (of_property_read_u32(np, "ssp,acc-position", &data->accel_position))
		data->accel_position = 0;

	if (of_property_read_u32(np, "ssp,mag-position", &data->mag_position))
		data->mag_position = 0;

	pr_info("[SSP] acc-posi[%d] mag-posi[%d]\n",
			data->accel_position, data->mag_position);

	if (of_property_read_u32(np, "ssp,ap-rev", &data->ap_rev))
		data->ap_rev = 0;
#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
	if (of_property_read_u32(np, "ssp,prox-hi_thresh",
			&data->uProxHiThresh_default))
		data->uProxHiThresh_default = DEFUALT_HIGH_THRESHOLD;

	if (of_property_read_u32(np, "ssp,prox-low_thresh",
			&data->uProxLoThresh_default))
		data->uProxLoThresh_default = DEFUALT_LOW_THRESHOLD;

	pr_info("[SSP] hi-thresh[%u] low-thresh[%u]\n",
		data->uProxHiThresh_default, data->uProxLoThresh_default);

	if (of_property_read_u32(np, "ssp,prox-cal_hi_thresh",
			&data->uProxHiThresh_cal))
		data->uProxHiThresh_cal = DEFUALT_CAL_HIGH_THRESHOLD;

	if (of_property_read_u32(np, "ssp,prox-cal_LOW_thresh",
			&data->uProxLoThresh_cal))
		data->uProxLoThresh_cal = DEFUALT_CAL_LOW_THRESHOLD;

	pr_info("[SSP] cal-hi[%u] cal-low[%u]\n",
		data->uProxHiThresh_cal, data->uProxLoThresh_cal);
#endif

	if (of_property_read_string(np, "ssp,model_name", &data->model_name))
		data->model_name = DEFAULT_MODEL_NAME;

	if (of_property_read_string(np, "ssp,aux_vreg", &vreg_name)) {
		pr_info("[SSP] %s does not use aux regulator\n",
			data->model_name);
		data->reg_aux = NULL;
	} else {
		data->reg_aux = regulator_get(NULL, vreg_name);
		if (IS_ERR(data->reg_aux)) {
			pr_err("[SSP] can't get regulator by %s\n", vreg_name);
			return -EINVAL;
		}
		pr_info("[SSP] Using %s for %s's aux-regulator\n",
			vreg_name, data->model_name);

		errorno = ssp_regulator_ctrl(data->reg_aux, true, 5);
		if (errorno)
			pr_err("[SSP] Can't contorl regulator(%d)\n", errorno);
	}

	if (of_property_read_u8(np, "ssp,cmd-ctl", &data->uCmdCtl))
		data->uCmdCtl = 0;
	else
		pr_info("[SSP] Using uCmdCtl : %u\n", data->uCmdCtl);

	if (of_property_read_u32(np, "ssp,chub-hw-rev", &data->chub_fw_rev))
		data->chub_fw_rev = -1;

	if (of_property_read_u32(np, "ssp,ap-hw-rev", &data->ap_hw_rev)) {
		data->ap_hw_rev = 0;
		pr_err("[SSP] Can't get ap-hw-rev, set as 0");
	} else {
		pr_info("[SSP] ssp,ap-hw-rev : %d", data->ap_hw_rev);
	}

	if (of_property_read_u16(np, "ssp,model-num", &data->model_number)) {
		data->model_number = 0;
		pr_err("[SSP] ssp,model-num, set as 0");
	} else {
		pr_info("[SSP] ssp,model-num : %d", data->model_number);
	}

#ifdef CONFIG_SENSORS_SSP_AP_WAKEUP
	data->irq_gpio = of_get_named_gpio(np, "ssp,irq_gpio", 0);
	if (!gpio_is_valid(data->irq_gpio)) {
		pr_err("%s: of_get_named_gpio failed: irq_gpio %d\n", __func__,
				data->irq_gpio);
		return -EINVAL;
	}
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
	if (of_property_read_u8_array(np, "ssp-pdc",
		data->pdc_matrix, ARRAY_SIZE(data->pdc_matrix)))
		pr_err("[SSP] pdc matrix table, set as 0");

	pr_info("[SSP] pdc : %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n",
			   data->pdc_matrix[0], data->pdc_matrix[1], data->pdc_matrix[2],
			   data->pdc_matrix[3], data->pdc_matrix[4],
			   data->pdc_matrix[5], data->pdc_matrix[6], data->pdc_matrix[7],
			   data->pdc_matrix[8], data->pdc_matrix[9],
			   data->pdc_matrix[10], data->pdc_matrix[11], data->pdc_matrix[12],
			   data->pdc_matrix[13], data->pdc_matrix[14],
			   data->pdc_matrix[15], data->pdc_matrix[16], data->pdc_matrix[17],
			   data->pdc_matrix[18], data->pdc_matrix[19],
			   data->pdc_matrix[20], data->pdc_matrix[21], data->pdc_matrix[22],
			   data->pdc_matrix[23], data->pdc_matrix[24],
			   data->pdc_matrix[25], data->pdc_matrix[26]);
#endif

#ifdef CONFIG_SENSORS_SSP_PRESSURE_SENSOR
#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
	if (of_property_read_u16_array(np, "pressure-sw-offset",
		data->sw_offset_matrix, ARRAY_SIZE(data->sw_offset_matrix))) {
		data->sw_offset = 0;
		pr_err("no pressure-sw-offset, set as 0");
	} else {
		data->sw_offset =
			data->sw_offset_matrix[0] ? data->sw_offset_matrix[1] * -1 : data->sw_offset_matrix[1];
	}
	pr_info("[SSP] pressure-sw-offset %d \n", data->sw_offset);
#endif
#endif

	if (of_property_read_u32_array(np, "ssp,light-circle",
		data->lightCircleBuf, ARRAY_SIZE(data->lightCircleBuf))){
		pr_err("[SSP] light-circle coord, set as 0");
		memset(data->lightCircleBuf, 0, sizeof(data->lightCircleBuf));
	}

	pr_err("[SSP] light-circle coord,x=%d,y=%d,z=%d", data->lightCircleBuf[0],
				data->lightCircleBuf[1], data->lightCircleBuf[2]);

	data->reg_hrm3p3v = devm_regulator_get(dev, "vdd_ldo21");
	if (IS_ERR(data->reg_hrm3p3v)) {
		pr_err("[SSP] could not get ldo21_reg, %ld\n", PTR_ERR(data->reg_hrm3p3v));
		data->reg_hrm3p3v = 0;
	} else {
		errorno = regulator_enable(data->reg_hrm3p3v);
		if (errorno) {
			dev_err(dev, "Failed to enable reg_hrm3p3v regulators: %d\n", errorno);
		}
	}

	data->reg_hrm1p8v = devm_regulator_get(dev, "vdd_ldo23");
	if (IS_ERR(data->reg_hrm1p8v)) {
		pr_err("[SSP] could not get ldo23_reg, %ld\n", PTR_ERR(data->reg_hrm1p8v));
		data->reg_hrm1p8v = 0;
	} else {
		errorno = regulator_enable(data->reg_hrm1p8v);
		if (errorno) {
			dev_err(dev, "Failed to enable reg_hrm1p8v regulators: %d\n", errorno);
		}
	}

#if !defined(SENSORS_SSP_HEART)
	data->hrm_5v_onoff = of_get_named_gpio(np, "ssp,hrm_5v_onoff", 0);
	if (!gpio_is_valid(data->hrm_5v_onoff)) {
		pr_err("%s: of_get_named_gpio failed: hrm_5v_onoff %d\n", __func__,
				data->hrm_5v_onoff);
	} else {
		gpio_direction_output(data->hrm_5v_onoff, 1);
	}
#endif

	return errorno;
}
#else
static int ssp_parse_dt(struct device *dev,
	struct ssp_data *data)
{
	return -1;
}
#endif

#if IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
int send_panel_information(struct ssp_data *data, struct panel_event_bl_data *evdata){
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	int iRet = 0;

	//TODO: send brightness + aor_ratio information to sensorhub
	if (msg == NULL) {
		iRet = -ENOMEM;
		pr_err("[SSP] %s, failed to allocate memory for ssp_msg\n", __func__);
		return iRet;
	}
	msg->cmd = MSG2SSP_PANEL_INFO;
	msg->length = sizeof(struct panel_event_bl_data);
	msg->options = AP2HUB_WRITE;
	msg->buffer = kzalloc(sizeof(struct panel_event_bl_data), GFP_KERNEL);
	msg->free_buffer = 1;
	memcpy(msg->buffer, (u8 *)evdata, sizeof(struct panel_event_bl_data));

	iRet = ssp_send_command(data, msg, 0);

	if (iRet < 0)
		pr_err("[SSP] %s, failed to send panel information", __func__);
	return iRet;
}

static int panel_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct panel_notifier_event_data *evdata = data;
	char cmd = 0;

	if (event == PANEL_EVENT_UB_CON_STATE_CHANGED) {
		if (evdata->state != PANEL_EVENT_UB_CON_STATE_CONNECTED && evdata->state != PANEL_EVENT_UB_CON_STATE_DISCONNECTED) {
			pr_info("[SSP] %s PANEL_EVENT_UB_CON_CHANGED, event errno(%d)\n", __func__, evdata->state);
		} else {
			pr_info("[SSP] %s PANEL_EVENT_UB_CON_STATE, event(%d,4==con,5==discon)\n", __func__, evdata->state);

			sensorhub_data->ub_con_state = evdata->state == PANEL_EVENT_UB_CON_STATE_CONNECTED ?
				SSP_UB_CON_CONNECTED : SSP_UB_CON_DISCONNECTED;
			cmd = evdata->state == PANEL_EVENT_UB_CON_STATE_CONNECTED ? MSG2SSP_UB_CON : MSG2SSP_UB_DISCON;

			ssp_send_cmd(sensorhub_data, cmd, 0);
		}
	}
	else if(event == PANEL_EVENT_COPR_STATE_CHANGED){
		pr_info("[SSP] %s PANEL_EVENT_COPR_STATE_CHANGED, event(%d,6==disable,7==enable)\n", __func__, evdata->state);
		if(evdata->state != PANEL_EVENT_COPR_STATE_ENABLED && evdata->state != PANEL_EVENT_COPR_STATE_DISABLED){
			pr_info("[SSP] %s PANEL_EVENT_COPR_STATE_CHANGED, event errno(%d)\n", __func__, evdata->state);
		} else {
			sensorhub_data->las_copr_status = evdata->state == PANEL_EVENT_COPR_STATE_DISABLED ?
				MSG2SSP_COPR_DISABLE : MSG2SSP_COPR_ENABLE;
			cmd = (evdata->state == PANEL_EVENT_COPR_STATE_DISABLED) ? MSG2SSP_COPR_DISABLE : MSG2SSP_COPR_ENABLE;

			ssp_send_cmd(sensorhub_data, cmd, 0);
		}

	}else if (event == PANEL_EVENT_BL_STATE_CHANGED) {
		pr_info("[SSP] %s PANEL_EVENT_BL_CHANGED br=%d,aor=%d,is_acl=%d,acl_val=%d\n", __func__, evdata->d.bl.level,
			evdata->d.bl.aor, evdata->d.bl.acl_status, evdata->d.bl.gradual_acl_val);
		// store these values for reset
		memcpy(&sensorhub_data->panel_event_data, evdata, sizeof(struct panel_event_bl_data));
		send_panel_information(sensorhub_data, &(evdata->d.bl));
	}

	return 0;
}
#endif

static void wristup_boost_release(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work,
						struct ssp_data, wristup_boost_release_work);

	freq_qos_update_request(&data->cpu_cluster_qos, WRISTUP_CPU_DEFAULT_FREQ);
}

static void initialize_wristup_booster(struct ssp_data *data)
{
	int ret;
	struct cpufreq_policy *policy;

	policy = cpufreq_cpu_get(DEFAULT_CLUSTER);
	if (!policy) {
		nanohub_info("%s: Failed to get cpufreq policy for cluster(%d)\n",
			__func__, DEFAULT_CLUSTER);
		ret = -EAGAIN;
		goto reset_qos;
	}

	ret = freq_qos_add_request(&policy->constraints,
		&data->cpu_cluster_qos, FREQ_QOS_MIN, WRISTUP_CPU_DEFAULT_FREQ);
	if (ret < 0) {
		nanohub_info("%s: Failed to add qos constraint (%d)\n",
			__func__, ret);
		goto reset_qos;
	}
	INIT_DELAYED_WORK(&data->wristup_boost_release_work, wristup_boost_release);
	data->wristup_boost_initialized = true;

	nanohub_info("%s: success! \n",__func__);

	return;

reset_qos:
	freq_qos_remove_request(&data->cpu_cluster_qos);
	data->wristup_boost_initialized = false;
	return;
}

static void remove_wristup_booster(struct ssp_data *data)
{
	if (data->wristup_boost_initialized) {
		freq_qos_remove_request(&data->cpu_cluster_qos);
		data->wristup_boost_initialized = false;
	}
}

void wristup_booster_turn_on(struct ssp_data *data)
{
	if (data->wristup_boost_initialized) {
		cancel_delayed_work(&data->wristup_boost_release_work);
		freq_qos_update_request(&data->cpu_cluster_qos, WRISTUP_CPU_MIN_FREQ);
		schedule_delayed_work(&data->wristup_boost_release_work, msecs_to_jiffies(WRISTUP_PERIOD_US));
	}
}

void power_on_task(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work,
						struct ssp_data, work_power_on);
	int ret;

	pr_err("[SSP]: %s power_on_task is called\n", __func__);
	ret = sensorhub_power_on(data);
	pr_err("[SSP]: %s 1 (ret : %d)\n", __func__, ret);

	if (ret == 0) {
		queue_refresh_task(data, 0);
		pr_err("[SSP]: %s 2\n", __func__);
	} else {
		disable_debug_timer(data);    // serene_tmp
	}
}

#ifdef CONFIG_SENSORS_SSP_AP_WAKEUP
static irqreturn_t handle_threaded_ssp_irq(int irq, void *handle)
{
	struct ssp_data *data = (struct ssp_data *) handle;

	data->irq_cnt++;
	pr_err("[SSP]: %s - %d (%d)(%d)\n", __func__, data->irq_gpio, data->irq_cnt, data->uLastResumeState);

	if (data->uLastResumeState == SSP_AP_STATUS_SUSPEND)
		ssp_wake_lock_timeout(data->ssp_wake_lock, 200);

	return IRQ_HANDLED;
}

void ssp_enable_irq(struct ssp_data *data, bool enable)
{
	if (enable && !data->irq_state) {
		data->irq_state = 1;
		enable_irq(data->irq);
		enable_irq_wake(data->irq);
		pr_info("%s : enable, irq=%d\n", __func__, data->irq);
	} else if (!enable && data->irq_state) {
		data->irq_state = 0;
		disable_irq_wake(data->irq);
		disable_irq(data->irq);
		data->irq_cnt = 0;
		pr_info("%s : disable, irq=%d\n", __func__, data->irq);
	} else {
		//pr_err("[SSP]: %s : no change - %d:%d\n", __func__, enable, data->irq_state);
	}
}

static int ssp_gpio_irq_init(struct ssp_data *data, struct device *dev)
{
	int iRet = 0;

	data->irq_init_done = false;

	device_init_wakeup(dev, true);

	iRet = gpio_request(data->irq_gpio, "ssp_irq");
	if (iRet) {
		pr_err("[SSP]: %s: unable to request gpio[%d]\n", __func__, data->irq_gpio);
		return FAIL;
	}
	iRet = gpio_direction_input(data->irq_gpio);
	if (iRet) {
		gpio_free(data->irq_gpio);
		pr_err("[SSP]: %s: unable to set_direction for ssp_irq [%d]\n", __func__, data->irq_gpio);
		return FAIL;
	}

	/* interrupt set */
	data->irq = gpio_to_irq(data->irq_gpio);
	if (data->irq  <= 0) {
		gpio_free(data->irq);
		return FAIL;
	}
	data->irq_init_done = true;

	pr_info("[SSP] %s: done, irq: %d\n", __func__, data->irq);
	return SUCCESS;
}

int ssp_enable_ap_wakeup(struct ssp_data *data, struct device *dev)
{
	int iRet = 0;

	iRet = request_threaded_irq(data->irq, NULL, handle_threaded_ssp_irq,
			(IRQF_TRIGGER_HIGH | IRQF_SHARED | IRQF_ONESHOT), "ssp_irq", data);

	if (iRet) {
		pr_err("[SSP] %s: fail to get irq(0x%08x)\n", __func__, iRet);
		free_irq(data->irq, data);
		return FAIL;
	}

	//enable_irq(data->irq);
	//enable_irq_wake(data->irq);
	enable_irq_wake(data->irq);
	data->irq_state = 1;
	ssp_enable_irq(data, 0);

	pr_info("[SSP] %s: done, irq: %d\n", __func__, data->irq);

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(data->pinctrl)) {
		pr_err("[SSP] %s: failed to get ssp pinctrl\n", __func__);
		data->pinctrl = NULL;
	} else {
		data->gpio_config = pinctrl_lookup_state(data->pinctrl, "default");
		if (IS_ERR(data->gpio_config)) {
			pr_err("[SSP] %s: failed to get default pin state\n", __func__);
			data->gpio_config = NULL;
		} else
			pr_err("[SSP] %s: success to get default pin state\n", __func__);
	}

	if (data->pinctrl && data->gpio_config) {
		if (pinctrl_select_state(data->pinctrl, data->gpio_config))
			pr_err("%s: failed to turn on gpio_config\n", __func__);
		else
			print_gpm8(data);
	} else
		pr_err("[SSP] %s: pinctrl or gpio_config is NULL\n", __func__);
	return SUCCESS;
}
#endif

int check_hrm_vendor(struct ssp_data *data, struct device *dev)
{
	if (data->hrm_vendor == 0xFF) {
		//ssp_regulator_ctrl(data->reg_hrm3p3v, false, 0);
		//ssp_regulator_ctrl(data->reg_hrm1p8v, false, 0);
		//gpio_direction_output(data->hrm_5v_onoff, 0);
		//msleep(100);

		pr_err("[SSP] %s: 3p3v(%d) 1p8v(%d)", __func__,
			regulator_is_enabled(data->reg_hrm3p3v), regulator_is_enabled(data->reg_hrm1p8v));

		if(!regulator_is_enabled(data->reg_hrm3p3v))
			ssp_regulator_ctrl(data->reg_hrm3p3v, true, 0);

		if(!regulator_is_enabled(data->reg_hrm1p8v))
			ssp_regulator_ctrl(data->reg_hrm1p8v, true, 0);

		gpio_direction_output(data->hrm_5v_onoff, 1);
		//msleep(100);
		data->hrm_vendor = 0x55;
	}
	return 0;
}

struct ssp_data *ssp_probe(struct device *dev)
{
	int iRet = 0;
	struct ssp_data *data;
#ifndef CONFIG_OF
	struct ssp_platform_data *pdata;
#endif
	pr_info("[SSP] %s\n", __func__);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		pr_err("[SSP]: %s - failed to allocate memory for data\n",
			__func__);
		iRet = -ENOMEM;
		goto exit;
	}

	data->is_probe_done = false;

	if (dev->of_node) {
		iRet = ssp_parse_dt(dev, data);
		if (iRet) {
			pr_err("[SSP]: %s - Failed to parse DT\n", __func__);
			goto err_setup;
		}
	} else {
#ifndef CONFIG_OF
		pdata = dev->platform_data;
		if (pdata == NULL) {
			pr_err("[SSP] %s, platform_data is null\n", __func__);
			iRet = -ENOMEM;
#endif
			goto err_setup;
		}

#ifndef CONFIG_OF
		/* AP system_rev */
		if (pdata->check_ap_rev)
			data->ap_rev = pdata->check_ap_rev();
		else
			data->ap_rev = 0;
		pr_info("[SSP] %s, system Rev = 0x%x\n", __func__,
			data->ap_rev);

		/* Get sensor positions */
		if (pdata->get_positions) {
			pdata->get_positions(&data->accel_position,
				&data->mag_position);
		} else {
			data->accel_position = 0;
			data->mag_position = 0;
		}
		if (pdata->mag_matrix) {
			data->mag_matrix_size = pdata->mag_matrix_size;
			data->mag_matrix = pdata->mag_matrix;
		}
	}
#endif

	data->bProbeIsDone = false;
	sensorhub_data = data;

	mutex_init(&data->comm_mutex);
	mutex_init(&data->pending_mutex);
	mutex_init(&data->sysfs_op_mtx);
	mutex_init(&data->chub_log_dump_mutex);
	mutex_init(&data->chub_dram_dump_mutex);
	mutex_init(&data->scontext_mutex);

	pr_info("\n#####################################################\n");

#ifdef CONFIG_SENSORS_SSP_CHUB
	INIT_DELAYED_WORK(&data->work_refresh, refresh_task);
	INIT_DELAYED_WORK(&data->work_power_on, power_on_task);
	INIT_WORK(&data->work_reset_noti, reset_noti_task);
	INIT_WORK(&data->work_chub_log_dump, chub_log_dump_task);
	INIT_WORK(&data->work_chub_dumpstate, chub_dumpstate_task);
#endif

	initialize_variable(data);

	data->ssp_wake_lock = ssp_wake_lock_init(dev, "ssp_wake_lock");
	data->ssp_comm_wake_lock = ssp_wake_lock_init(dev, "ssp_comm_wake_lock");

	initialize_wristup_booster(data);

	initialize_variable_iio(data);
	iRet = initialize_indio_dev(dev, data);
	if (iRet < 0) {
		ssp_errf("could not create input device");
		goto err_iio_register_device;
	}

#ifdef CONFIG_SENSORS_SSP_INPUT_DEV
	iRet = initialize_input_dev(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - could not create input device\n", __func__);
		goto err_input_register_device;
	}
#endif
	iRet = initialize_debug_timer(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - could not create workqueue\n", __func__);
		goto err_create_debug_workqueue;
	}

	/* big data wq */
	iRet = initialize_bigdata_work(data);
	if (iRet < 0) {
		ssp_errf("big_data_work_create_error");
		goto err_create_bigdata_workqueue;
	}

	/* chub log dump wq */
	data->chub_log_dump_wq = create_singlethread_workqueue("ssp_chub_log_dump_wq");
	if (!data->chub_log_dump_wq)
		goto err_create_chub_log_dump_workqueue;

	data->chub_log_dump_buf = vzalloc(CHUB_LOG_BUF_SIZE);
	if(data->chub_log_dump_buf == NULL)
		goto err_create_chub_log_dump_buffer;

	iRet = initialize_chub_log_dump_node(data);
	if (iRet < 0) {
		ssp_errf("chub_log_dump_node_create_error");
		goto  err_create_chub_log_dump_node;
	}

	iRet = initialize_timestamp_sync_timer(data);
	if (iRet < 0) {
		ssp_errf("could not create ts_sync workqueue");
		goto err_create_ts_sync_workqueue;
	}

	iRet = initialize_timestamp_diff_sync_work(data);
	if (iRet < 0) {
		ssp_errf("could not create ts_sync workqueue");
		goto err_create_ts_diff_sync_workqueue;
	}

	iRet = initialize_post_suspend_timestamp_sync_work(data);
	if (iRet < 0) {
		ssp_errf("could not init ap_resume_timestamp_sync");
	}

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	iRet = initialize_lpm_motion(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - could not create workqueue\n", __func__);
		goto err_create_lpm_motion;
	}
#endif

	sensors_class_init();
	iRet = initialize_sysfs(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - could not create sysfs\n", __func__);
		goto err_sysfs_create;
	}

	iRet = ssp_scontext_initialize(data);
	if (iRet < 0) {
		ssp_errf("ssp_scontext_initialize err(%d)", iRet);
		ssp_scontext_remove(data);
		goto err_init_scontext;
	}

#ifdef CONFIG_SENSORS_SSP_INPUT_DEV
	iRet = initialize_event_symlink(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - could not create symlink\n", __func__);
		goto err_symlink_create;
	}
#endif

#ifdef CONFIG_SENSORS_SSP_AP_WAKEUP
	iRet = ssp_gpio_irq_init(data, dev);
	if (iRet <= 0) {
		pr_err("[SSP]: %s - could not create ssp_gpio_init\n", __func__);
	} else {
		ssp_enable_ap_wakeup(data, dev);
	}
#endif

	ssp_enable(data, true);

#if IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
	//register notifier for panel related function
	panel_notif.notifier_call = panel_notifier_callback;
	iRet = panel_notifier_register(&panel_notif);
	if (iRet) {
		pr_err("[SSP]: %s - fail to register panel_notifier_callback\n", __func__);
	}
#endif

#ifdef CONFIG_SENSORS_SSP_VIB_NOTIFY
	initialize_motor_callback(data);
#endif

	pr_info("[SSP]: %s - probe success!\n", __func__);

	data->is_probe_done = true;

	enable_debug_timer(data);
	schedule_delayed_work(&data->work_power_on, msecs_to_jiffies(3000));

	data->bProbeIsDone = true;
	iRet = 0;
	mutex_init(&data->shutdown_lock);

	goto exit;

#ifdef CONFIG_SENSORS_SSP_INPUT_DEV
	remove_event_symlink(data);
err_symlink_create:
	ssp_scontext_remove(data);
#endif
err_init_scontext:
	remove_sysfs(data);
err_sysfs_create:
#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	destroy_workqueue(data->lpm_motion_wq);
err_create_lpm_motion:
#endif
	destroy_workqueue(data->ts_diff_sync_wq);
err_create_ts_diff_sync_workqueue:
	destroy_workqueue(data->ts_sync_wq);
err_create_ts_sync_workqueue:
	device_destroy(data->ssp_mmap_class, MKDEV(MAJOR(data->ssp_mmap_dev_num), 0));
	cdev_del(&(data->ssp_mmap_cdev));
	class_destroy(data->ssp_mmap_class);
	unregister_chrdev_region(data->ssp_mmap_dev_num, 1);
err_create_chub_log_dump_node:
	kvfree(data->chub_log_dump_buf);
err_create_chub_log_dump_buffer:
	destroy_workqueue(data->chub_log_dump_wq);
err_create_chub_log_dump_workqueue:
	destroy_workqueue(data->bigdata_wq);
err_create_bigdata_workqueue:
	destroy_workqueue(data->debug_wq);
err_create_debug_workqueue:
#ifdef CONFIG_SENSORS_SSP_INPUT_DEV
	remove_input_dev(data);
#endif
err_input_register_device:
	remove_indio_dev(data);
err_iio_register_device:
	ssp_wake_lock_destroy(data->ssp_comm_wake_lock);
	ssp_wake_lock_destroy(data->ssp_wake_lock);
	mutex_destroy(&data->comm_mutex);
	mutex_destroy(&data->pending_mutex);
	mutex_destroy(&data->sysfs_op_mtx);
	mutex_destroy(&data->chub_log_dump_mutex);
	mutex_destroy(&data->chub_dram_dump_mutex);
	mutex_destroy(&data->scontext_mutex);
err_setup:
	kfree(data);
	data = NULL;
	pr_err("[SSP] %s, probe failed!\n", __func__);
exit:
	pr_info("#####################################################\n\n");
	return data;
}

void ssp_shutdown(struct ssp_data *data)
{
	if (data->bProbeIsDone == false) {
		pr_err("[SSP] %s, ssp is not probed!\n", __func__);
		return;
	}

	func_dbg();

	data->bProbeIsDone = false;

	disable_debug_timer(data);
	remove_all_timestamp_sync_works(data);

	ssp_enable(data, false);
	clean_pending_list(data);

	mutex_lock(&data->shutdown_lock);
	cancel_work_sync(&data->work_reset_noti);
	cancel_work_sync(&data->work_debug);
	cancel_delayed_work_sync(&data->work_refresh);
	destroy_workqueue(data->debug_wq);
#ifdef CONFIG_SENSORS_SSP_VIB_NOTIFY
	remove_motor_callback(data);
#endif
#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	remove_lpm_motion(data);
#endif
	mutex_unlock(&data->shutdown_lock);

	ssp_scontext_remove(data);
	remove_sysfs(data);

#ifdef CONFIG_SENSORS_SSP_INPUT_DEV
	remove_event_symlink(data);
	remove_input_dev(data);
#endif
	remove_indio_dev(data);
	remove_chub_log_dump_node(data);

	remove_wristup_booster(data);

	ssp_wake_lock_destroy(data->ssp_comm_wake_lock);
	data->ssp_comm_wake_lock = NULL;
	ssp_wake_lock_destroy(data->ssp_wake_lock);
	data->ssp_wake_lock = NULL;

	mutex_destroy(&data->comm_mutex);
	mutex_destroy(&data->pending_mutex);
	mutex_destroy(&data->sysfs_op_mtx);
	mutex_destroy(&data->shutdown_lock);

#if IS_ENABLED(CONFIG_SEC_PANEL_NOTIFIER_V2)
	panel_notifier_unregister(&panel_notif);
#endif

	pr_info("[SSP] %s done\n", __func__);
}


int ssp_suspend(struct ssp_data *data, struct device *dev)
{
	ssp_infof();

	disable_debug_timer(data);
	disable_timestamp_sync_timer(data);

#ifdef CONFIG_SENSORS_SSP_AP_WAKEUP
	ssp_enable_irq(data, 1);  // if (device_may_wakeup(dev))
#endif

	data->uLastResumeState = SSP_AP_STATUS_SUSPEND;
	data->bSavePostSuspendTime = false;

	return 0;
}

int ssp_resume(struct ssp_data *data, struct device *dev)
{
	unsigned int wIdx;

	ssp_infof();

	enable_timestamp_sync_timer(data);
	wIdx = ssp_nanohub_log_buf_check(data);

	enable_debug_timer(data);

#ifdef CONFIG_SENSORS_SSP_AP_WAKEUP
	ssp_enable_irq(data, 0);  // if (device_may_wakeup(dev))
#endif

	data->uLastResumeState = SSP_AP_STATUS_RESUME;

	return 0;
}

int get_sensor_state(struct ssp_data *data, u16 sensor_type, const char *func_name)
{
	int sensor_state;

	sensor_state = data->uSensorState & (1ULL << sensor_type);

	if (!sensor_state)
		pr_info("[SSP]: %s - %d sensor is not connected(0x%llx)\n",
			func_name, sensor_type, data->uSensorState);

	return sensor_state;
}
