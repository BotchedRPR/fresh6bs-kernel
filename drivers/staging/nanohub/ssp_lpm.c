/*
 *  Copyright (C) 2021, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#include <linux/init.h>
#include "ssp_lpm.h"
#include "ssp_type_define.h"
#if IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
#include <linux/battery/common/sb_ext.h>
#endif

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
unsigned int fota_mode;

#if IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
static unsigned int is_lpcharge(void)
{
	return sb_get_lpcharge();
}

#else
static unsigned int is_lpcharge(void)
{
	return 0;
}
#endif

#if 0
static int __init ssp_check_fotamode(char *bootmode)
{
	if (strncmp(bootmode, "fota", 4) == 0)
		fota_mode = 1;

	return fota_mode;
}
__setup("bootmode=", ssp_check_fotamode);
#endif

static int ssp_charging_motion(struct ssp_data *data, int iEnable)
{
	u8 uBuf[2] = {0, 0};

	if (iEnable == 1) {
		pr_info("[SSP]%s, enable smart alert motion\n", __func__);
		send_instruction(data, ADD_LIBRARY,
			SH_LIBRARY_MOVE_DETECTION, uBuf, 2);
	} else {
		pr_info("[SSP]%s, disable smart alert motion\n", __func__);
		send_instruction(data, REMOVE_LIBRARY,
			SH_LIBRARY_MOVE_DETECTION, uBuf, 2);
	}

	return 0;
}

static int ssp_charging_rotation(struct ssp_data *data, int iEnable)
{
	u8 uBuf[2] = {0, 0};

	if (iEnable == 1) {
		pr_info("[SSP]%s, enable LPM rotation\n", __func__);
		send_instruction(data, ADD_LIBRARY,
			SH_LIBRARY_AUTOROTATION, uBuf, 2);
	} else {
		pr_info("[SSP]%s, disable LPM rotation\n", __func__);
		send_instruction(data, REMOVE_LIBRARY,
			SH_LIBRARY_AUTOROTATION, uBuf, 2);
	}

	return 0;
}

int ssp_parse_motion(struct ssp_data *data, char *dataframe, int start, int end)
{
	int length = end - start;
	char *buf = dataframe + start;

	if (length > 5) {
		pr_err("[SSP]%s, invalid data length %d\n", __func__, length);
		return FAIL;
	}

	if ((buf[0] == 1) && (buf[1] == 1)) {
		data->lpm_int_mode = buf[2];
		if (buf[2] == SH_LIBRARY_MOVE_DETECTION) // SMART_ALERT_MOTION
			pr_info("[SSP] %s - LP MODE WAKEUP\n", __func__);
		else if (buf[2] == SH_LIBRARY_AUTOROTATION)  // LPM_AUTO_ROTATION
			data->lpm_rotation_info = buf[3];
		queue_work(data->lpm_motion_wq, &data->work_lpm_motion);
		return SUCCESS;
	}

	return FAIL;
}

static void lpm_motion_work_func(struct work_struct *work)
{
	struct ssp_data *data =
		container_of(work, struct ssp_data, work_lpm_motion);

	if (data->lpm_int_mode == SH_LIBRARY_MOVE_DETECTION) {
		input_event(data->motion_input_dev, EV_KEY, KEY_HOMEPAGE, 1);
		input_sync(data->motion_input_dev);
		ssp_charging_motion(data, 0);

		msleep(20);

		input_event(data->motion_input_dev, EV_KEY, KEY_HOMEPAGE, 0);
		input_sync(data->motion_input_dev);
		ssp_charging_motion(data, 1);
	} else if (data->lpm_int_mode == SH_LIBRARY_AUTOROTATION) {
		if ((data->lpm_rotation_info >= -1)
				&& (data->lpm_rotation_info <= 3)) {
			pr_info("[SSP] %s - lpm_rotation_info : %d\n",
				__func__, data->lpm_rotation_info);
			input_event(data->motion_input_dev,
				EV_ABS, ABS_X, data->lpm_rotation_info);
			input_sync(data->motion_input_dev);
		}
	}

	ssp_wake_lock_timeout(data->ssp_wake_lock, 1 * HZ);
}

void set_charger_info(struct ssp_data *data)
{
	int iRet = 0;

	iRet = ssp_send_cmd(data, MSG2SSP_AP_STATUS_POW_CONNECTED, 0);

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s -fail to set_charger_info %d\n", __func__, iRet);
	}
}

void set_lpm_mode(struct ssp_data *data)
{
	//if (lpcharge == 1 || fota_mode == 1) {
	if (is_lpcharge()) {
		set_charger_info(data);
		ssp_charging_motion(data, 1);
		ssp_charging_rotation(data, 1);
		data->bLpModeEnabled = true;
		pr_info("[SSP] LPM Charging...\n");
	} else {
		data->bLpModeEnabled = false;
		pr_info("[SSP] Normal Booting OK\n");
	}
}

int initialize_lpm_motion(struct ssp_data *data)
{
	data->lpm_motion_wq =
		create_singlethread_workqueue("ssp_lpm_motion_wq");
	if (!data->lpm_motion_wq)
		return ERROR;

	INIT_WORK(&data->work_lpm_motion, lpm_motion_work_func);
	return SUCCESS;
}

void remove_lpm_motion(struct ssp_data *data)
{
	if (data->lpm_motion_wq) {
		cancel_work_sync(&data->work_lpm_motion);
		destroy_workqueue(data->lpm_motion_wq);
	}
}

#endif

