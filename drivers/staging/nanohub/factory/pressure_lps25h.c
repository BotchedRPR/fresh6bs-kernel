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

#define VENDOR    "STM"
#define CHIP_ID_28_NAME    "LPS28DFW"
#define CHIP_ID_28		(0xB4)

#define CHIP_ID_27_NAME    "LPS27HHW"
#define CHIP_ID_27		(0xB3)



#define CALIBRATION_FILE_PATH        "/efs/FactoryApp/baro_delta" //"/csa/sensor/baro_cal_data"
#define SW_OFFSET_FILE_PATH            "/efs/FactoryApp/baro_sw_offset"

#define    PR_ABS_MAX    8388607        /* 24 bit 2'compl */
#define    PR_ABS_MIN    -8388608

#define PRESSURE_SELFTEST_COUNT 5
#define CAL_DATA_LEN            10
#define SW_OFFSET_DATA_LEN        10
/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

static ssize_t sea_level_pressure_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    int iRet;

    struct ssp_data *data = dev_get_drvdata(dev);

    iRet = kstrtoint(buf, 10, &data->sealevelpressure);
    if (iRet < 0)
        return iRet;

    if (data->sealevelpressure == 0) {
        pr_info("%s, our->temperature = 0\n", __func__);
        data->sealevelpressure = -1;
    }

    pr_info("[SSP] %s sea_level_pressure = %d\n",
        __func__, data->sealevelpressure);
    return size;
}

int lps25h_pressure_open_calibration(struct ssp_data *data)
{
    int iErr = 0;
    char chBuf[CAL_DATA_LEN] = {0,};

    file_manager_read(data, CALIBRATION_FILE_PATH, chBuf, CAL_DATA_LEN * sizeof(char), 0);

    iErr = kstrtoint(chBuf, 10, &data->iPressureCal);
    if (iErr < 0) {
        pr_err("[SSP]: %s - kstrtoint failed. %d", __func__, iErr);
        return iErr;
    }

    ssp_dbg("[SSP]: calibration: %d\n", data->iPressureCal);

    if (data->iPressureCal < PR_ABS_MIN || data->iPressureCal > PR_ABS_MAX)
        pr_err("[SSP]: %s - wrong offset value!!!\n", __func__);

    return iErr;
}

char *itoa_simple_helper(char *dest, int i) {
  if (i <= -10) {
    dest = itoa_simple_helper(dest, i/10);
  }
  *dest++ = '0' - i%10;
  return dest;
}

char *itoa_simple(char *dest, int i) {
  char *s = dest;
  if (i < 0) {
    *s++ = '-';
  } else {
    i = -i;
  }
  *itoa_simple_helper(s, i) = '\0';
  return dest;
}

int lps25h_set_pressure_cal(struct ssp_data *data)
{
    int iRet = 0;
    struct ssp_msg *msg;
    s32 pressure_cal;
#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
    char buf[CAL_DATA_LEN] = {0,};
    int sw_offset;
#endif

    if (!(data->uSensorState & (1 << PRESSURE_SENSOR))) {
        pr_info("[SSP]: %s - Skip this function!!!"\
            ", pressure sensor is not connected(0x%llx)\n",
            __func__, data->uSensorState);
        return iRet;
    }

    pressure_cal = data->iPressureCal;
#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
    sw_offset = data->sw_offset;
#endif

    msg = kzalloc(sizeof(*msg), GFP_KERNEL);
    if (msg == NULL) {
        pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
        return -ENOMEM;
    }
    msg->cmd = MSG2SSP_AP_MCU_SET_BARO_CAL;
#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
    msg->length = 8;
#else
    msg->length = 4;
#endif
    msg->options = AP2HUB_WRITE;
#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
    msg->buffer = (char *) kzalloc(8, GFP_KERNEL);
#else
    msg->buffer = (char *) kzalloc(4, GFP_KERNEL);
#endif
    msg->free_buffer = 1;
    memcpy(msg->buffer, &pressure_cal, 4);
#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
    memcpy(msg->buffer + 4, &sw_offset, 4);
#endif

    iRet = ssp_send_command(data, msg, 0);

    if (iRet != SUCCESS) {
        pr_err("[SSP]: %s - i2c fail %d\n", __func__, iRet);
        iRet = ERROR;
    }

#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
    itoa_simple(buf, pressure_cal);
    file_manager_write(data, CALIBRATION_FILE_PATH, buf, CAL_DATA_LEN * sizeof(char), 0);
    pr_info("[SSP] Set pressure cal data %d %d\n", pressure_cal, sw_offset);
#else
    pr_info("[SSP] Set pressure cal data %d\n", pressure_cal);
#endif

    return iRet;
}

#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
int open_pressure_sw_offset_file(struct ssp_data *data)
{
    int iErr = 0;

    char chBuf[SW_OFFSET_DATA_LEN] = {0,};
    file_manager_read(data, SW_OFFSET_FILE_PATH, chBuf, SW_OFFSET_DATA_LEN * sizeof(char), 0);
    iErr = kstrtoint(chBuf, 10, &data->sw_offset);

    if (iErr < 0) {
        pr_err("[SSP]: %s - kstrtoint failed. %d", __func__, iErr);
        return iErr;
    }
    ssp_dbg("[SSP]: sw_offset: %d\n", data->sw_offset);

    return iErr;
}

static ssize_t pressure_sw_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ssp_data *data = dev_get_drvdata(dev);

    return sprintf(buf, "%d\n", data->sw_offset);
}

static ssize_t pressure_sw_offset_store(struct device *dev, struct device_attribute *attr, const char *buf,
                     size_t size)
{
    struct ssp_data *data = dev_get_drvdata(dev);
    int sw_offset = 0;
    int iRet = 0;
    char str_sw_offset[8] = {0,};

    iRet = kstrtoint(buf, 10, &sw_offset);
    if (iRet < 0) {
        pr_err("[SSP]: %s - kstrtoint failed. %d\n", __func__, iRet);
        return iRet;
    }

    snprintf(str_sw_offset, sizeof(str_sw_offset), "%d", sw_offset);
    file_manager_write(data, SW_OFFSET_FILE_PATH, str_sw_offset, sizeof(str_sw_offset), 0);

    mutex_lock(&data->sysfs_op_mtx);
    data->sw_offset = sw_offset;
    set_pressure_cal(data);
    mutex_unlock(&data->sysfs_op_mtx);
    pr_info("[SSP] %s sw_offset %d", __func__, sw_offset);

    return size;
}
#endif

static ssize_t pressure_cabratioin_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    struct ssp_data *data = dev_get_drvdata(dev);
    int iPressureCal = 0, iErr = 0;

    iErr = kstrtoint(buf, 10, &iPressureCal);
    if (iErr < 0) {
        pr_err("[SSP]: %s - kstrtoint failed.(%d)", __func__, iErr);
        return iErr;
    }

    if (iPressureCal < PR_ABS_MIN || iPressureCal > PR_ABS_MAX)
        return -EINVAL;

    mutex_lock(&data->sysfs_op_mtx);
    data->iPressureCal = (s32)iPressureCal;
    set_pressure_cal(data);
    mutex_unlock(&data->sysfs_op_mtx);

    return size;
}

static ssize_t pressure_cabratioin_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct ssp_data *data = dev_get_drvdata(dev);

    mutex_lock(&data->sysfs_op_mtx);
    pressure_open_calibration(data);
    mutex_unlock(&data->sysfs_op_mtx);

    return snprintf(buf, PAGE_SIZE, "%d\n", data->iPressureCal);
}

/* sysfs for vendor & name */
static ssize_t pressure_vendor_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t pressure_name_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iRet = 0;
	int iReties = 0;
	struct ssp_msg *msg;
	u8 baro_id = 0;

retries:
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_GET_BARO_ID;
	msg->length = 1;
	msg->options = AP2HUB_READ;
	msg->buffer = &baro_id;
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

	pr_err("[SSP]baro_id=%d\n", baro_id);

	return snprintf(buf, PAGE_SIZE, "%s\n", baro_id == CHIP_ID_28 ? CHIP_ID_28_NAME :
		CHIP_ID_27_NAME);
}

static ssize_t raw_data_read(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct ssp_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n",
        data->buf[PRESSURE_SENSOR].pressure[0]);
}

static ssize_t pressure_temperature_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct ssp_data *data = dev_get_drvdata(dev);
    s32 temperature = 0;
    s32 float_temperature = 0;

    temperature = (s32) (data->buf[PRESSURE_SENSOR].pressure[1]);
    float_temperature = ((temperature%100) > 0 ? (temperature%100) : -(temperature%100));
    return sprintf(buf, "%d.%02d\n", (temperature/100), float_temperature);
}

static ssize_t pressure_selftest_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    char selftest_ret = 0;
    struct ssp_data *data = dev_get_drvdata(dev);

    ssp_send_read_factory_cmd(data, PRESSURE_SENSOR, 0, (char *)&selftest_ret, sizeof(selftest_ret), 1000, 1, __func__);

    return snprintf(buf, PAGE_SIZE, "%d\n", (int)selftest_ret);
}

static DEVICE_ATTR(raw_data, 0444, raw_data_read, NULL);
static DEVICE_ATTR(vendor, 0444, pressure_vendor_show, NULL);
static DEVICE_ATTR(name, 0444, pressure_name_show, NULL);
static DEVICE_ATTR(calibration, 0664,
    pressure_cabratioin_show, pressure_cabratioin_store);
static DEVICE_ATTR(sea_level_pressure, 0220,
    NULL, sea_level_pressure_store);
static DEVICE_ATTR(temperature, 0444, pressure_temperature_show, NULL);
static DEVICE_ATTR(selftest, 0444, pressure_selftest_show, NULL);
#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
static DEVICE_ATTR(sw_offset, 0664, pressure_sw_offset_show, pressure_sw_offset_store);
#endif

static struct device_attribute *pressure_attrs[] = {
    &dev_attr_raw_data,
    &dev_attr_vendor,
    &dev_attr_name,
    &dev_attr_calibration,
    &dev_attr_sea_level_pressure,
    &dev_attr_temperature,
    &dev_attr_selftest,
#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
    &dev_attr_sw_offset,
#endif
    NULL,
};

void initialize_lps25h_pressure_factorytest(struct ssp_data *data)
{
#if !defined(CONFIG_SENSORS_SSP_WISE) && !defined(CONFIG_SENSORS_SSP_FRESH) && !defined(CONFIG_SENSORS_SSP_LUCKY)
    data->convert_coef = 4096/100;
#endif
    sensors_register(data->prs_device, data, pressure_attrs,
        "barometer_sensor");
}

void remove_lps25h_pressure_factorytest(struct ssp_data *data)
{
    sensors_unregister(data->prs_device, pressure_attrs);
}
