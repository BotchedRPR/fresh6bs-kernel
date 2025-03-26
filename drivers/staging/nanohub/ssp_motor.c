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
 
#if IS_ENABLED(CONFIG_SEC_VIB_NOTIFIER)
#include <linux/vibrator/sec_vibrator.h>
#include <linux/vibrator/sec_vibrator_notifier.h>
#endif
#include "ssp_motor.h"

#if IS_ENABLED(CONFIG_SEC_VIB_NOTIFIER)
static int vibrator_notifier(struct notifier_block *self, unsigned long action, void *data)
{
	struct vib_notifier_context *vib_data = (struct vib_notifier_context *)data;
	bool enable = (bool)action;
	int duration = vib_data->timeout;

	//ssp_err("%s: action:%ld, duration: %d, (0x%x)", __func__, action, duration, sensorhub_data->uLastResumeState);

	if (enable && duration >= 20 && sensorhub_data->uLastResumeState == SSP_AP_STATUS_RESUME) {
		ssp_info("%s[ON]: action:%ld, duration: %d, prev: %d", __func__, action, duration, sensorhub_data->motor_flag);

		sensorhub_data->motor_flag = VIB_EVENT_PLAY;
		sensorhub_data->motor_duration = duration;//effect->replay.length;
		queue_work(sensorhub_data->ssp_motor_wq, &sensorhub_data->work_ssp_motor);
	} else {
		//defence code for receiving stop event twice
		if (sensorhub_data->motor_flag != VIB_EVENT_STOP) {
			ssp_info("%s[OFF]: action:%ld, duration: %d", __func__, action, duration);

			sensorhub_data->motor_flag = VIB_EVENT_STOP;
			sensorhub_data->motor_duration = 0;
			queue_work(sensorhub_data->ssp_motor_wq, &sensorhub_data->work_ssp_motor);
		}
	}
	return 0;
}

static struct notifier_block vib_notifier = {
	.notifier_call = vibrator_notifier,
};
#endif

static int send_motor_state(struct ssp_data *data)
{
	int iRet = 0;
	struct ssp_msg *msg;

	pr_debug("[SSP] %s start\n", __func__);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		ssp_err("%s, failed to alloc memory for ssp_msg", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MCU_SET_MOTOR_STATUS;
	msg->length = 3;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(3, GFP_KERNEL);
	msg->free_buffer = 1;

	/*Set duration*/
	msg->buffer[0] = (char)(data->motor_duration & 0x00FF);
	msg->buffer[1] = (char)((data->motor_duration & 0xFF00)>>8);
	msg->buffer[2] = (unsigned char)(data->motor_flag);

	iRet = ssp_send_command(data, msg, 0);
	if (iRet != SUCCESS) {
		ssp_err("%s - fail %d", __func__, iRet);
		goto out;
	}
	pr_debug("[SSP] %s -> duration:%d flag:%d\n", __func__, data->motor_duration, data->motor_flag);
	iRet = (data->motor_flag << 16 | data->motor_duration);

out:
	return iRet;
}

static void ssp_motor_work_func(struct work_struct *work)
{
	int iRet = 0;
	struct ssp_data *data = container_of(work,
					struct ssp_data, work_ssp_motor);

	iRet = send_motor_state(data);
	pr_debug("[SSP] %s : iRet %d\n", __func__, iRet);
}

void initialize_motor_callback(struct ssp_data *data)
{
	int iRet = 0;

	//Get ssp_data pointer
	//ssp_data_info = data;

#if IS_ENABLED(CONFIG_SEC_VIB_NOTIFIER)
	//register notifier for motor related function
	iRet = sec_vib_notifier_register(&vib_notifier);
#endif
	ssp_info("sensor_motor_callback result : %d", iRet);

	//make wq for send motor duration
	data->ssp_motor_wq = create_singlethread_workqueue("ssp_motor_wq");
	if (!data->ssp_motor_wq) {
		iRet = -1;
		ssp_err("%s - could not create motor workqueue", __func__);
		destroy_workqueue(data->ssp_motor_wq);
		return;
	}

	INIT_WORK(&data->work_ssp_motor, ssp_motor_work_func);
}
void remove_motor_callback(struct ssp_data *data)
{
#if IS_ENABLED(CONFIG_SEC_VIB_NOTIFIER)
	sec_vib_notifier_unregister(&vib_notifier);
#endif
	if (data->ssp_motor_wq) {
		cancel_work_sync(&data->work_ssp_motor);
		destroy_workqueue(data->ssp_motor_wq);
	}
}

