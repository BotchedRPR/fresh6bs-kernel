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
#include "ssp_comm.h"
#include "ssp_data.h"


#define LIMIT_DELAY_CNT		200
#define RECEIVEBUFFERSIZE	12
#define DEBUG_SHOW_DATA	0

#define SSP_CMD_SIZE 256
#define SSP_MSG_HEADER_SIZE 9
#define SSP_MSG_CP2AP_HEADER_SIZE	4

#define MAKE_WORD(H, L) (((((u16)H) << 8) & 0xff00) | ((((u16)L)) & 0x00ff))

void handle_packet(struct ssp_data *data, char *packet, int packet_size)
{
#if 0
	ssp_sensorhub_log(__func__, packet, packet_size);
#else
	u16 msg_length = 0, msg_options = 0;
	char *buffer = NULL;

	if (packet_size < SSP_MSG_CP2AP_HEADER_SIZE) {
		nanohub_err("[SSP] %s nanohub packet size(%d) is small/(%s)", __func__, packet_size, packet);
		return;
	}

	msg_options = MAKE_WORD(packet[1], packet[0]);;
	msg_length = MAKE_WORD(packet[3], packet[2]);

	if (msg_length == 0) {
		nanohub_err("[SSP] %s lengh is zero %d %d", __func__, msg_length, msg_options);
		return;
	}

	if ((msg_options == AP2HUB_READ) || (msg_options == AP2HUB_RETURN)) {
		bool found = false;
		struct ssp_msg *msg, *n;

		pr_info("[SSP][Debug] %s %d %d %d", __func__, packet[4], msg_length, msg_options);

		mutex_lock(&data->pending_mutex);
		if (!list_empty(&data->pending_list)) {
			list_for_each_entry_safe(msg, n, &data->pending_list, list) {

				if (msg->options == (AP2HUB_WRITE | AP2HUB_RETURN)) {
					msg->length = 1;
					msg->options = AP2HUB_RETURN;
				}
				if ((msg->length == msg_length) && (msg->options == msg_options)) {
					list_del(&msg->list);
					found = true;
					break;
				}
			}

			if (!found) {
				nanohub_err("[SSP] %s %d %d - Not match error", __func__, msg_length, msg_options);
				goto exit;
			}

			if (msg_options == AP2HUB_READ || (msg_options == AP2HUB_RETURN)) {
				if (msg->buffer != NULL && msg->length != 0 && msg->length == msg_length) {
					memcpy(msg->buffer, packet + SSP_MSG_CP2AP_HEADER_SIZE, msg_length);
					pr_info("[SSP][Debug1] %s %d %d (%d)", __func__, msg->buffer[0], msg_length, msg->length);
				} else {
					if (msg->buffer == NULL)
						pr_err("[SSP] %s - NULL ptr!", __func__);
					nanohub_err("[SSP] %s rcv:%d(%d) %d - invalid packet", __func__, msg_length, msg->length,  msg_options);
				}
			}

			if (msg->done != NULL && !completion_done(msg->done)) {
				complete(msg->done);
			}
		} else {
			nanohub_err("[SSP] %s List empty error(%d %d)", __func__, msg_length, msg_options);
		}

exit:
		mutex_unlock(&data->pending_mutex);

	} else if (msg_options == HUB2AP_WRITE) {
		buffer = kzalloc(msg_length, GFP_KERNEL);
		memcpy(buffer, &packet[SSP_MSG_CP2AP_HEADER_SIZE], msg_length);

		data->timestamp = get_current_timestamp();
		parse_dataframe(data, buffer, msg_length);
		kfree(buffer);
	} else {
		nanohub_err("[SSP] %s msg_options(%d) does not define", __func__, msg_options);
	}

	return;
#endif
}

void clean_msg(struct ssp_msg *msg)
{
	if (msg->free_buffer) {
		kfree(msg->buffer);
		msg->buffer = NULL;
	}
	kfree(msg);
	msg = NULL;
}

static char ssp_cmd_data[SSP_CMD_SIZE];

static int do_transfer(struct ssp_data *data, struct ssp_msg *msg, int timeout)
{
	int status = SUCCESS;
	int ret = 0;
	bool is_ssp_shutdown;
	u16 ulength = SSP_MSG_HEADER_SIZE;

	//pr_info("[SSP][Debug] %s msg->option = (0x%x), msg->cmd = (0x%x), msg->data = (%d), msg->buffer = NULL, msg->length(%d)",
	//			__func__, msg->options, msg->cmd, (int)msg->data, msg->length);

	mutex_lock(&data->comm_mutex);

	is_ssp_shutdown = !is_sensorhub_working(data);
	if (is_ssp_shutdown || (msg->length > (SSP_CMD_SIZE - SSP_MSG_HEADER_SIZE))) {
		pr_err("[SSP] %s ssp shutdown = %d, (msg->length=%d) do not parse", is_ssp_shutdown, msg->length, __func__);
		mutex_unlock(&data->comm_mutex);
		return -EIO;
	}

	memcpy(ssp_cmd_data, msg, SSP_MSG_HEADER_SIZE);
	if (msg->options != AP2HUB_READ) {
		memcpy(&ssp_cmd_data[SSP_MSG_HEADER_SIZE], msg->buffer, msg->length);	
		ulength += msg->length;
	}
	if (msg->done != NULL) {
		mutex_lock(&data->pending_mutex);
		list_add_tail(&msg->list, &data->pending_list);
		mutex_unlock(&data->pending_mutex);
	}
	ret = sensorhub_comms_write(data, ssp_cmd_data, ulength, timeout);

	if (ret != ulength) {
		pr_err("[SSP] %s comm write fail!!", __func__);
		status = ERROR;
		if (msg->done != NULL){
			mutex_lock(&data->pending_mutex);
			list_del(&msg->list);
			mutex_unlock(&data->pending_mutex);
		}
	}

	mutex_unlock(&data->comm_mutex);

	if (status < 0) {
		is_ssp_shutdown = !is_sensorhub_working(data);
		data->cnt_com_fail += (is_ssp_shutdown) ? 0 : 1;
		pr_err("[SSP] %s cnt_com_fail %d , ssp_down %d ", __func__, data->cnt_com_fail, is_ssp_shutdown);

		return status;
	}

	if (status >= 0 && msg->done != NULL) {
		ret = wait_for_completion_timeout(msg->done,
				msecs_to_jiffies(timeout));

		if (msg->clean_pending_list_flag) {
			msg->clean_pending_list_flag = 0;
			pr_err("[SSP] %s communication fail so recovery_mcu func call", __func__);
			return -EINVAL;
		}

		/* when timeout is happened */
		if (!ret) {
			msg->done = NULL;
			mutex_lock(&data->pending_mutex);
			list_del(&msg->list);
			mutex_unlock(&data->pending_mutex);

			is_ssp_shutdown = !is_sensorhub_working(data);
			data->cnt_timeout += (is_ssp_shutdown) ? 0 : 1;

			pr_err("[SSP] %s cnt_timeout %d, ssp_down %d !!", __func__, data->cnt_timeout, is_ssp_shutdown);
			return -EINVAL;
		}
	}

	return status;
}

void clean_pending_list(struct ssp_data *data)
{
	struct ssp_msg *msg, *n;

	mutex_lock(&data->pending_mutex);

	list_for_each_entry_safe(msg, n, &data->pending_list, list) {
		list_del(&msg->list);
		if (msg->done != NULL && !completion_done(msg->done)) {
			msg->clean_pending_list_flag = 1;
			complete(msg->done);
		}
		if (msg->dead_hook != NULL)
			*(msg->dead_hook) = true;

		//clean_msg(msg);
	}
	mutex_unlock(&data->pending_mutex);
}

int ssp_send_command(struct ssp_data *data, struct ssp_msg *msg,  int timeout)
{
	int status = SUCCESS;
	DECLARE_COMPLETION_ONSTACK(done);

	if (data == NULL || msg == NULL) {
		pr_err("[SSP] %s():[SSP] data or msg is NULL\n", __func__);
		return ERROR;
	}	
	if ((msg->options == AP2HUB_READ || msg->options & AP2HUB_RETURN) && timeout <= 0) {
			pr_err("[SSP] %s AP2HUB_READ zero timeout", __func__);
			return -EINVAL;
	}

	if (timeout > 0) {
		msg->done = &done;
	} else {
		msg->done = NULL;
	}

	if (do_transfer(data, msg, timeout) < 0) {
		pr_err("[SSP] %s do_transfer error\n", __func__);
		status = ERROR;
	}

	if (status < 0) {
		if ((msg->cmd != SYNC_TIMESTAMP) && (msg->cmd != MSG2SSP_AP_MCU_SET_MOTOR_STATUS))
			data->uTimeOutCnt++;
	}
	clean_msg(msg);

	return status;
}

int ssp_send_cmd(struct ssp_data *data, char command, int arg)
{
	int iRet = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = command;
	msg->length = 1;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(1, GFP_KERNEL);
	msg->data = arg;
	msg->free_buffer = 1;

	iRet = ssp_send_command(data, msg, 0);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - command 0x%x failed %d\n",
				__func__, command, iRet);
		iRet = ERROR;
		goto out;
	}
	iRet = SUCCESS;
	ssp_dbg("[SSP]: %s - command 0x%x %d\n", __func__, command, arg);

out:
	return iRet;
}


int send_instruction(struct ssp_data *data, u8 uInst,
							u8 uSensorType, u8 *uSendBuf, u16 uLength)
{
	char command;
	int iRet = 0;
	u64 cur_time = 0;
	u16 buffer_len = uLength + 1;
	struct ssp_msg *msg;

	if ((!(data->uSensorState & (1ULL << uSensorType)))
		&& (uInst <= CHANGE_DELAY)) {
		pr_err("[SSP]: %s - Bypass Inst Skip! - %u\n",
			__func__, uSensorType);
		return FAIL;
	}

	switch (uInst) {
	case REMOVE_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_REMOVE;
		break;
	case ADD_SENSOR:
		buffer_len += sizeof(cur_time);
		data->bypass_logcnt = 0;
		command = MSG2SSP_INST_BYPASS_SENSOR_ADD;
		break;
	case CHANGE_DELAY:
		command = MSG2SSP_INST_CHANGE_DELAY;
		break;
	case GO_SLEEP:
		command = MSG2SSP_AP_STATUS_LCD_OFF;
		data->uLastAPState = MSG2SSP_AP_STATUS_LCD_OFF;
		break;
	case REMOVE_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_REMOVE;
		break;
	case ADD_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_ADD;
		break;
	default:
		pr_debug("[SSP] %s, Just passthrough Inst = 0x%x\n",
			__func__, uInst);
		command = uInst;
		break;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		iRet = -ENOMEM;
		return iRet;
	}
	msg->cmd = command;
	msg->length = buffer_len;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(buffer_len, GFP_KERNEL);
	msg->free_buffer = 1;

	msg->buffer[0] = uSensorType;
	memcpy(&msg->buffer[1], uSendBuf, uLength);

	if (uInst == ADD_SENSOR) {
		cur_time = get_current_timestamp();
		memcpy(&msg->buffer[uLength + 1], &cur_time, sizeof(cur_time));
	}

	ssp_dbg("[SSP]: %s - Inst = 0x%x, Sensor Type = 0x%x, data = %u, time = %llu\n",
			__func__, command, uSensorType, msg->buffer[1], cur_time);

	iRet = ssp_send_command(data, msg, 0);
	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - Instruction CMD Fail %d\n", __func__, iRet);
		iRet = ERROR;
	}

	return iRet;
}

int send_instruction_sync(struct ssp_data *data, u8 uInst,
	u8 uSensorType, u8 *uSendBuf, u16 uLength)
{
	char command;
	int iRet = 0, timeout = 1000;
	u64 cur_time = 0;
	char buffer[18] = { 0, };
	struct ssp_msg *msg;

	if ((!(data->uSensorState & (1ULL << uSensorType)))
		&& (uInst <= CHANGE_DELAY)) {
		pr_err("[SSP]: %s - Bypass Inst Skip! - %u\n",
			__func__, uSensorType);
		return FAIL;
	}

	switch (uInst) {
	case REMOVE_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_REMOVE;
		break;
	case ADD_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_ADD;
		break;
	case CHANGE_DELAY:
		command = MSG2SSP_INST_CHANGE_DELAY;
		break;
	case GO_SLEEP:
		command = MSG2SSP_AP_STATUS_LCD_OFF;
		data->uLastAPState = MSG2SSP_AP_STATUS_LCD_OFF;
		break;
	case REMOVE_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_REMOVE;
		break;
	case ADD_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_ADD;
		break;
	case EXT_CMD:
		if (uLength > 10) {
			pr_err("[SSP]: %s exceed size(%u) for cmd(0x%x)!!\n",
			__func__, uLength, uSensorType);
			return -EINVAL;
		}
		timeout = 3000;
		command = uSensorType;
		break;
	default:
		pr_info("[SSP] %s, Just passthrough Inst = 0x%x\n",
			__func__, uInst);
		command = uInst;
		break;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = command;

	msg->options = AP2HUB_WRITE | AP2HUB_RETURN;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	if (uInst == EXT_CMD) {
		msg->length = uLength;
		memcpy(&msg->buffer[0], uSendBuf, uLength);
	} else {
		msg->length = uLength + 1;
		msg->buffer[0] = uSensorType;
		memcpy(&msg->buffer[1], uSendBuf, uLength);
		if (uInst == ADD_SENSOR) {
			cur_time = get_current_timestamp();
			msg->length += sizeof(cur_time);
			memcpy(&msg->buffer[uLength + 1], &cur_time, sizeof(cur_time));
		}
	}

	ssp_dbg("[SSP]: %s - Inst Sync = 0x%x, Sensor Type = %u, data = %u, time = %llu\n",
			__func__, command, uSensorType, msg->buffer[0], cur_time);

	iRet = ssp_send_command(data, msg, timeout);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - Instruction CMD Fail %d\n", __func__, iRet);
		buffer[0] = ERROR;
		goto exit;
	}

exit:
	return buffer[0];
}

int get_chipid(struct ssp_data *data)
{
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	int ret;
	char send_buf[1] = {0,};
	char rc;

	rc = (data->uResetCnt > 0) ? 1 : 0;
	msg->cmd = MSG2SSP_AP_WHOAMI;
	msg->length = 1;
	msg->options = AP2HUB_READ;
	msg->buffer = send_buf;
	msg->free_buffer = 0;
	msg->data = rc;

	ret = ssp_send_command(data, msg, 2000);

	if (ret != SUCCESS) {
		pr_err("[SSP][Debug] get_firmware_rev error %d", ret);
	} else {
		pr_err("[SSP][Debug] get_firmware_rev %d", send_buf[0]);
	}


	return send_buf[0];
}

int set_sensor_position(struct ssp_data *data)
{
	int iRet = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_SENSOR_FORMATION;
	msg->length = 3;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(3, GFP_KERNEL);
	msg->free_buffer = 1;

	msg->buffer[0] = data->accel_position;
	msg->buffer[1] = data->accel_position;
	msg->buffer[2] = data->mag_position;

	iRet = ssp_send_command(data, msg, 0);

	pr_info("[SSP] Sensor Posision A : %u, G : %u, M: %u, P: %u\n",
			data->accel_position, data->accel_position, data->mag_position, 0);

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s -fail to set_sensor_position %d\n", __func__, iRet);
		iRet = ERROR;
	}
	return iRet;
}

u64 get_sensor_scanning_info(struct ssp_data *data)
{
	int iRet = 0, z = 0;
	u64 result = 0;
	char bin[SENSOR_MAX + 1];

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_SENSOR_SCANNING;
	msg->length = 4;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *) &result;//(char *) kzalloc(4, GFP_KERNEL);
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg, 3000);

	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - ipc fail %d\n", __func__, iRet);

	bin[SENSOR_MAX] = '\0';
	for (z = 0; z < SENSOR_MAX; z++)
		bin[SENSOR_MAX - 1 - z] = (result & (1ULL << z)) ? '1' : '0';
	pr_err("[SSP] %s state: %s\n", __func__, bin);

	return result;
}

unsigned int get_firmware_rev(struct ssp_data *data)
{
	int iRet;
	u32 result = SSP_INVALID_REVISION;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_FIRMWARE_REV;
	msg->length = 4;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *) &result;
	msg->free_buffer = 0;
	msg->data = data->ap_hw_rev;

	iRet = ssp_send_command(data, msg, 1000);

	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);

	return result;
}

unsigned int get_feature_list(struct ssp_data *data)
{
	int iRet = 0;
	u32 result = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}

	msg->cmd = MSG2SSP_AP_FEATURE_LIST_INFO;
	msg->length = 4;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *) &result;
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg, 3000);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);
	} else {
		pr_err("[SSP]: %s - 0x%x (%d)\n", __func__, result);
	}

	return result;
}

unsigned int get_ssr_stuck_info(struct ssp_data *data)
{
	int iRet = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_SSR_STUCK_INFO;
	msg->length = SENSOR_STUCK_INFO_SIZE;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *) &data->uSsrStuckInfo;
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg, 3000);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);
		memset(data->uSsrStuckInfo, 0x00, SENSOR_STUCK_INFO_SIZE);
	} else {
		pr_err("[SSP]: %s - 0x%x 0x%x 0x%x 0x%x\n", __func__, data->uSsrStuckInfo[0], data->uSsrStuckInfo[1], 
			data->uSsrStuckInfo[2], data->uSsrStuckInfo[3]);
	}
	return iRet;
}

int set_model_number(struct ssp_data *data)
{
	int iRet = 0;
#ifdef CONFIG_SENSORS_SSP_LUCKY
	int size = 2;
#else
	int size = 3;
#endif

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MODEL_NAME;
	msg->length = size;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(size, GFP_KERNEL);
	msg->free_buffer = 1;

	memcpy(msg->buffer, &data->model_number, 2);
	if (size == 3)
		msg->buffer[2] = data->bin_type;

	iRet = ssp_send_command(data, msg, 0);

	pr_info("[SSP] set_model_number: %u\n", data->model_number);
	pr_info("[SSP] set_bin_type: %u\n", data->bin_type);

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s -fail to set_model_number %d\n", __func__, iRet);
		iRet = ERROR;
	}
	return iRet;
}

// BOE_417000 or SDC_407004
#define LCD_PANEL_SVC_OCTA		"/sys/class/lcd/panel/lcd_type"// "/sys/class/lcd/panel/SVC_OCTA"

static int set_lcd_panel_type_to_ssp(struct ssp_data *data)
{
	int iRet = 0;
	int iLength = sizeof(int);
	int offset = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
#ifndef CONFIG_SENSORS_SSP_LUCKY
	iLength = iLength + sizeof(bool);
#endif

	msg->cmd = MSG2SSP_AP_SET_LCD_TYPE;
	msg->length = iLength;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(iLength, GFP_KERNEL);
	msg->free_buffer = 1;

	memcpy(&msg->buffer[offset], &data->lcd_type, sizeof(int));
#ifndef CONFIG_SENSORS_SSP_LUCKY
	offset += sizeof(int);
	memcpy(&msg->buffer[offset], &data->svc_octa_change, sizeof(bool));
#endif

	iRet = ssp_send_command(data, msg, 0);

	pr_info("[SSP] SET_LCD_TYPE : %x, svc_octa_change : %d\n", data->lcd_type, data->svc_octa_change);

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s -fail to set_model_number %d\n", __func__, iRet);
		iRet = ERROR;
	}
	return iRet;
}

extern int get_lcd_info(char *arg);

int set_lcd_panel_type(struct ssp_data *data)
{
	int iRet = 0;

	data->lcd_type = get_lcd_info("id");
	pr_err("[SSP] %s : %d %x\n", __func__, data->lcd_type, data->lcd_type);  

	if ((data->lcd_type & 0x00030000) == 0x00010000) {  // sdc : 0, boe : 1
		pr_err("[SSP]: lcd_type - BOE");
	} else {
		pr_err("[SSP]: lcd_type - SDC");	
	}

	iRet = set_lcd_panel_type_to_ssp(data);

	return iRet;
}

#ifdef CONFIG_SSP_RTC
unsigned int get_rtc_diff(struct ssp_data *data)
{
	int iRet;
	u64 result = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
						__func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_RTC_DIFF;
	msg->length = 4;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *) &result;
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg, 1000);

	if (iRet != SUCCESS)
		pr_err("[SSP] : %s - transfer fail %d\n", __func__, iRet);
	return result;
}
#endif

int sensor_reg_dump_read(struct ssp_data *data, u8 type)
{
	int iRet = 0;
	struct ssp_msg *msg;
	u8 reg_dump[SENSORHUB_REG_DUMP_SIZE + 1] = {0, };
	int i = 0;

	nanohub_info("[SSP]: %s - sensorhub dump start\n", __func__);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		nanohub_info("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_SENSOR_REG_DUMP;
	msg->length = sizeof(data->reg_dump);
	msg->options = AP2HUB_READ;
	msg->buffer = (char *)data->reg_dump;
	msg->free_buffer = 0;
	msg->data = type;

	iRet = ssp_send_command(data, msg, 3000);

	if (iRet != SUCCESS)
		nanohub_info("[SSP]: %s - fail %d\n", __func__, iRet);
	else
	{
		memcpy(reg_dump, data->reg_dump + 2, sizeof(data->reg_dump) - 2);
		
		nanohub_info("[SSP_dump] sensor_type = %u\n", type);
		for(i = 0 ; i < SENSORHUB_REG_DUMP_SIZE + 1; i += 16)
			nanohub_info("SSP:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,"
				"0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",
				reg_dump[i], reg_dump[i+1], reg_dump[i+2], reg_dump[i+3], reg_dump[i+4],
				reg_dump[i+5], reg_dump[i+6], reg_dump[i+7], reg_dump[i+8], reg_dump[i+9],
				reg_dump[i+10], reg_dump[i+11], reg_dump[i+12], reg_dump[i+13], reg_dump[i+14],
				reg_dump[i+15]);	
		}

	nanohub_info("[SSP]: %s - sensorhub dump end \n", __func__ );

	return iRet;
}

unsigned int get_library_protocol_version(struct ssp_data *data)
{
	int iRet = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_READ_LIB_PROTOCOL_VERSION;
	msg->length = LIB_PROTOCOL_VERSION_SIZE;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *) &data->lib_version;
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg, 3000);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);
		memset(data->lib_version, 0x00, LIB_PROTOCOL_VERSION_SIZE);
	} else {
		pr_err("[SSP]: %s - %d %d %d %d %d %d %d\n", __func__, data->lib_version[0],
			data->lib_version[1], data->lib_version[2], data->lib_version[3],
			data->lib_version[4], data->lib_version[5], data->lib_version[6]);
	}

	return iRet;
}

#ifndef CONFIG_SENSORS_SSP_LUCKY
unsigned int get_hrm_vendor(struct ssp_data *data)
{
	int iRet = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_GET_HRM_VENDOR;
	msg->length = 1;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *) &data->hrm_vendor;
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg, 3000);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);
	} else {
		pr_err("[SSP]: %s - %d\n", __func__, data->hrm_vendor); // ADI = 1, TI = 0
	}

	return iRet;
}
#endif

int ssp_send_write_cmd(struct ssp_data *data, u32 cmd, u32 subcmd,
		char *sendbuf, u16 length, const char *func_name)
{
	int iRet;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = cmd;
	msg->length = length;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(length, GFP_KERNEL);
	msg->free_buffer = 1;
	memcpy(msg->buffer, sendbuf, length);
	msg->data = subcmd;

	//pr_err("[SSP] %s 0x%x - %d %d\n", func_name, msg->cmd, msg->data, msg->buffer[0]);

	iRet = ssp_send_command(data, msg, 0);

	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);

	return iRet;
}

int ssp_send_read_cmd(struct ssp_data *data, u32 cmd, u32 subcmd,
		char *buf, u16 length, int timeout, int expectlen, const char *func_name)
{
	int iRet;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	memset(buf, 0x00, length);

	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}

	if (length != expectlen) {
		pr_err("[SSP] %s : length error (%d:%d)\n", func_name, length, expectlen);
		kfree(msg);
		return -ENOMEM;
	}

	msg->cmd = cmd;
	msg->length = length;
	msg->options = AP2HUB_READ;
	msg->buffer = buf;
	msg->free_buffer = 0;
	msg->data = subcmd;

	//pr_err("[SSP] %s(b) 0x%x - %d %d\n", func_name, msg->cmd, msg->data, msg->buffer[0]);

	iRet = ssp_send_command(data, msg, timeout);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);
	} else {
		ssp_dbg("[SSP] %s - %d", __func__, buf[0]);
	}

	return iRet;
}

int ssp_send_write_factory_cmd(struct ssp_data *data, u16 sensor_type, u32 subcmd,
		char *sendbuf, u16 length, const char *func_name)
{
	int iRet;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = SENSOR_FACTORY;
	msg->length = length;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(length, GFP_KERNEL);
	msg->free_buffer = 1;
	memcpy(msg->buffer, sendbuf, length);
	msg->data = (uint32_t)(subcmd << 16 | sensor_type);

	pr_err("[SSP] %s 0x%x - %d %d\n", func_name, msg->cmd, msg->data, msg->buffer[0]);

	iRet = ssp_send_command(data, msg, 0);

	if (iRet != SUCCESS)
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);

	return iRet;
}

int ssp_send_read_factory_cmd(struct ssp_data *data, u16 sensor_type, u32 subcmd,
		char *buf, u16 length, int timeout, int expectlen, const char *func_name)
{
	int iRet;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	memset(buf, 0x00, length);

	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}

	if (length != expectlen) {
		pr_err("[SSP] %s : length error (%d:%d)\n", func_name, length, expectlen);
		kfree(msg);
		return -ENOMEM;
	}

	msg->cmd = SENSOR_FACTORY;
	msg->length = length;
	msg->options = AP2HUB_READ;
	msg->buffer = buf;
	msg->free_buffer = 0;
	msg->data = (uint32_t)(subcmd << 16 | sensor_type);

	pr_err("[SSP] %s(f) 0x%x - %d %d\n", func_name, msg->cmd, msg->data, buf[0]);

	iRet = ssp_send_command(data, msg, timeout);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - transfer fail %d\n", __func__, iRet);
	} else {
		ssp_dbg("[SSP] %s - %d", __func__, buf[0]);
	}



	return iRet;
}
