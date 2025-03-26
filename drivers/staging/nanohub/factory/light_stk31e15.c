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
#include "../ssp.h"

#define VENDOR     "SITRONIX"
#define CHIP_ID     "STK31E15"

#define LIGHT_CAL_PARAM_FILE_PATH	"/efs/FactoryApp/light_cal_data"
#define SVC_OCTA_FILE_PATH			"/efs/FactoryApp/svc_octa_data"
#define LCD_PANEL_SVC_OCTA		"/sys/class/lcd/panel/SVC_OCTA"

#define ChCoef            720 // 50.0
#define ChCoef_BOE        300 // 30.0

#define SVC_OCTA_DATA_SIZE      22

static char *svc_octa_filp_name[2] = {SVC_OCTA_FILE_PATH, LCD_PANEL_SVC_OCTA};
static int svc_octa_filp_offset[2] = {0, 0};
static char svc_octa_data[2][SVC_OCTA_DATA_SIZE + 1] = { {0, }, };

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/
static s32 get_lsb_size(uint8_t e)
{
	s32 lsb_size = 0;

	switch (e) {
	case 0:
		lsb_size = 1;// 0.01f;
		break;
	case 1:
		lsb_size = 2;//0.02f;
		break;
	case 2:
		lsb_size = 4;//0.04f;
		break;
	case 3:
		lsb_size = 8;//0.08f;
		break;
	case 4:
		lsb_size = 16;//0.16f;
		break;
	case 5:
		lsb_size = 32;//0.32f;
		break;
	case 6:
		lsb_size = 64;//0.64f;
		break;
	case 7:
		lsb_size = 128;//1.28f;
		break;
	case 8:
		lsb_size = 256;//2.56f;
		break;
	case 9:
		lsb_size = 512;//5.12f;
		break;
	case 10:
		lsb_size = 1024;//10.24f;
		break;
	case 11:
		lsb_size = 2048;//20.48f;
		break;
	default:
		lsb_size = 0;
		break;
	}
	return lsb_size;
}

s32 light_get_lux(struct ssp_data *data, uint16_t m, uint8_t e)
{
	static s32 lux;
	static s32 lsb_size;

	lsb_size = (s32)get_lsb_size(e);

	if ((data->lcd_type & 0x00030000) == 0x00010000) {  // sdc : 0, boe : 1
		lux = (s32)((lsb_size * m * ChCoef_BOE) / 1000); /* lsb size 0.01, ChCoed 0.1 = 1000 */
		//pr_err("[SSP]: %s - BOE", __func__);
	}
	else {
		lux = (s32)((lsb_size * m * ChCoef) / 1000); /* lsb size 0.01, ChCoed 0.1 = 1000 */
		//pr_err("[SSP]: %s - SDC", __func__);
	}

	return lux;
}

static ssize_t light_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t light_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}

static ssize_t light_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u,%u,%u,%u,%u,%u\n",
		data->buf[SH_SENSOR_LIGHT_CCT].ch0_16ms, data->buf[SH_SENSOR_LIGHT_CCT].ch0_891us,
		data->buf[SH_SENSOR_LIGHT_CCT].lux_16ms, data->buf[SH_SENSOR_LIGHT_CCT].lux_891us,
		data->buf[SH_SENSOR_LIGHT_CCT].itime, data->buf[SH_SENSOR_LIGHT_CCT].gain_val);
}

static ssize_t light_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u,%u,%u,%u,%u,%u\n",
		data->buf[LIGHT_SENSOR].ch0_16ms, data->buf[LIGHT_SENSOR].ch0_891us,
		data->buf[LIGHT_SENSOR].lux_16ms, data->buf[LIGHT_SENSOR].lux_891us,
		data->buf[LIGHT_SENSOR].itime, data->buf[LIGHT_SENSOR].gain_val);
}

int set_light_cal_param_to_ssp(struct ssp_data *data)
{
	int iRet = 0;
	u32 light_cal_data[2] = {0, };

	struct ssp_msg *msg;

	file_manager_read(data, LIGHT_CAL_PARAM_FILE_PATH, (char *)light_cal_data, sizeof(light_cal_data) , 0);
	pr_info("[SSP]: %s - light_cal_data: %d %d", __func__, light_cal_data[0], light_cal_data[1]);

	if (strcmp(svc_octa_data[0], svc_octa_data[1]) != 0 && *svc_octa_data[0] != 0) {
		pr_err("[SSP] %s - svc_octa_data, previous = %s, current = %s", __func__, svc_octa_data[0], svc_octa_data[1]);
		data->svc_octa_change = true;
		return -EIO;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_AP_SET_LIGHT_CAL;
	msg->length = sizeof(light_cal_data);
	msg->options = AP2HUB_WRITE;
	msg->buffer = (u8 *)&light_cal_data;
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg, 0);
	if (iRet != SUCCESS) {
		pr_err("[SSP] %s -fail to set. %d\n", __func__, iRet);
		iRet = ERROR;
	}

	return iRet;
}

static ssize_t light_cal_show(struct device *dev,
					   struct device_attribute *attr, char *buf)
{
	u32 light_cal_data[2] = {0, };

	struct ssp_data *data = dev_get_drvdata(dev);
	int result = 0;

	file_manager_read(data, LIGHT_CAL_PARAM_FILE_PATH, (char *)light_cal_data, sizeof(light_cal_data), 0);
	result = light_cal_data[1] != 0 ? SUCCESS : FAIL;

	return sprintf(buf, "%d, %d,%d\n", result, light_cal_data[1], data->buf[LIGHT_SENSOR].raw_lux);
}

static ssize_t light_cal_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iRet = 0;
	int update = 0;
	u32 light_cal_data[2] = {0, };

	iRet = kstrtoint(buf, 10, &update);
	if (iRet < 0) {
		pr_err("[SSP]: %s - kstrtoint failed. %d\n", __func__, iRet);
		return iRet;
	}

	if (update) {
		struct ssp_msg *msg;

		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		msg->cmd = MSG2SSP_AP_GET_LIGHT_CAL;
		msg->length = sizeof(light_cal_data);
		msg->options = AP2HUB_READ;
		msg->buffer = (u8 *)&light_cal_data;
		msg->free_buffer = 0;

		iRet = ssp_send_command(data, msg, 1000);

		iRet = file_manager_write(data, SVC_OCTA_FILE_PATH, (char *)&svc_octa_data[1], SVC_OCTA_DATA_SIZE, 0);
		memcpy(svc_octa_data[0], svc_octa_data[1], SVC_OCTA_DATA_SIZE);
		if (iRet != SVC_OCTA_DATA_SIZE) {
			pr_err("[SSP]: %s - Can't write svc_octa_data to file\n", __func__);
			iRet = -EIO;
		} else {
			pr_err("[SSP]: %s - svc_octa_data[1]: %s", __func__, svc_octa_data[1]);
		}
	}

	pr_info("[SSP]: %s - light_cal_data: %d %d", __func__, light_cal_data[0], light_cal_data[1]);
	iRet = file_manager_write(data, LIGHT_CAL_PARAM_FILE_PATH, (char *)light_cal_data, sizeof(light_cal_data), 0);

	if (iRet < 0)
		pr_err("[SSP]: %s - light_do_calibrate() failed\n", __func__);

	return size;
}

static ssize_t light_copr_roix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iRet = 0;
	int iReties = 0;
	struct ssp_msg *msg;
	short copr_buf[12];

	memset(copr_buf, 0, sizeof(copr_buf));
retries:
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_GET_COPR_ROIX;
	msg->length = sizeof(copr_buf);
	msg->options = AP2HUB_READ;
	msg->buffer = (u8 *)copr_buf;
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg, 1000);

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s fail %d\n", __func__, iRet);

		if (iReties++ < 2) {
			pr_err("[SSP] %s fail, retry\n", __func__);
			mdelay(5);
			goto retries;
		}
		return FAIL;
	}

	pr_info("[SSP] %s - %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", __func__,
		copr_buf[0], copr_buf[1], copr_buf[2], copr_buf[3], 
		copr_buf[4], copr_buf[5], copr_buf[6], copr_buf[7], 
		copr_buf[8], copr_buf[9], copr_buf[10], copr_buf[11]);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
				copr_buf[0], copr_buf[1], copr_buf[2], copr_buf[3], 
				copr_buf[4], copr_buf[5], copr_buf[6], copr_buf[7], 
				copr_buf[8], copr_buf[9], copr_buf[10], copr_buf[11]);
}

static ssize_t light_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iRet = 0;
	struct ssp_msg *msg;
	short debug_buf[6];

	memset(debug_buf, 0, sizeof(debug_buf));

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_GET_LIGHT_DEBUG_INFO;
	msg->length = sizeof(debug_buf);
	msg->options = AP2HUB_READ;
	msg->buffer = (u8 *)debug_buf;
	msg->free_buffer = 0;

	iRet = ssp_send_command(data, msg, 1000);

	if (iRet != SUCCESS) {
		pr_err("[SSP] %s fail %d\n", __func__, iRet);
		return FAIL;
	}

	pr_info("[SSP] %s-%d,%d,%d,%d,%d,%d\n", __func__, debug_buf[0], debug_buf[1], 
		debug_buf[2], debug_buf[3], debug_buf[4], debug_buf[5]);
	
	return sprintf(buf, "%d, %d, %d, %d, %d, %d\n", debug_buf[0], debug_buf[1], 
		debug_buf[2], debug_buf[3], debug_buf[4], debug_buf[5]);
}

static ssize_t light_circle_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

       int circle0 = data->lightCircleBuf[0];
       int circle1 = data->lightCircleBuf[1];
       int circle2 = data->lightCircleBuf[2];

	return snprintf(buf, PAGE_SIZE, "%d.%d %d.%d %d.%d\n",
           circle0/100, circle0%100, circle1/100, circle1%100, circle2/100, (circle2%100)/10);
}

int initialize_light_sensor(struct ssp_data *data){
	int iRet = 0, i = 0;

	for (i = 0; i < 2; i++) {
		file_manager_read(data, svc_octa_filp_name[i], (char *)svc_octa_data[i], SVC_OCTA_DATA_SIZE, svc_octa_filp_offset[i]);
		pr_info("[SSP]: %s - svc_octa_filp[%d]: %s", __func__, i, svc_octa_data[i]);
	}

	iRet = set_light_cal_param_to_ssp(data);
	if (iRet < 0)
		pr_err("[SSP]: %s - sending light calibration data failed\n", __func__);

	iRet = set_lcd_panel_type(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - sending lcd type data failed\n", __func__);
	}

	return iRet;
}

static DEVICE_ATTR(vendor, 0444, light_vendor_show, NULL);
static DEVICE_ATTR(name, 0444, light_name_show, NULL);
static DEVICE_ATTR(lux, 0440, light_lux_show, NULL);
static DEVICE_ATTR(raw_data, 0440, light_data_show, NULL);
static DEVICE_ATTR(copr_roix, 0440, light_copr_roix_show, NULL);
static DEVICE_ATTR(light_cal, 0664, light_cal_show, light_cal_store);
static DEVICE_ATTR(light_circle, 0440, light_circle_show, NULL);
static DEVICE_ATTR(debug_info, 0440, light_debug_show, NULL);

static struct device_attribute *light_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_lux,
	&dev_attr_raw_data,
	&dev_attr_copr_roix,
	&dev_attr_light_cal,
    &dev_attr_light_circle,
	&dev_attr_debug_info,
	NULL,
};

void initialize_stk31e15_light_factorytest(struct ssp_data *data)
{
	sensors_register(data->light_device, data, light_attrs, "light_sensor");
}

void remove_stk31e15_light_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->light_device, light_attrs);
}
