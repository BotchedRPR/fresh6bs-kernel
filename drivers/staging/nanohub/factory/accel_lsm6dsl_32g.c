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

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define VENDOR        "STM"
#define CHIP_ID        "LSM6DSO"

#define CALIBRATION_FILE_PATH    "/efs/FactoryApp/accel_cal_data1"
#define CALIBRATION_DATA_AMOUNT    20

#define MAX_ACCEL_1G        4096
#define ACCEL_X_SPEC        0
#define ACCEL_Y_SPEC        0
#define ACCEL_Z_SPEC_MIN    -4096
#define ACCEL_Z_SPEC_MAX    4096

#define ACCEL32G_REACTIVE_LENGTH    1
#define ACCEL32G_SELFTEST_LENGTH    14

static ssize_t accel_vendor_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t accel_name_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}

int lsm6dsl_accel_open_calibration1(struct ssp_data *data)
{
    int iRet = 0;

    data->accelcal.x = 0;
    data->accelcal.y = 0;
    data->accelcal.z = 0;

    iRet = file_manager_read(data,CALIBRATION_FILE_PATH, (char *)&data->accelcal, sizeof(data->accelcal), 0);
    ssp_dbg("[SSP]: open accel calibration %d, %d, %d\n",
        data->accelcal.x, data->accelcal.y, data->accelcal.z);

    if ((data->accelcal.x == 0) && (data->accelcal.y == 0)
        && (data->accelcal.z == 0))
        return ERROR;

    return iRet;

}

int lsm6dsl_set_accel_cal1(struct ssp_data *data)
{
    int iRet = 0;
    struct ssp_msg *msg;
    s16 accel_cal[3];

    if (!(data->uSensorState & (1ULL << ACCELEROMETER_SENSOR_32G))) {
        pr_info("[SSP]: %s - Skip this function!!!"\
            ", accel sensor is not connected(0x%llx)\n",
            __func__, data->uSensorState);
        return iRet;
    }
    accel_cal[0] = data->accelcal.x;
    accel_cal[1] = data->accelcal.y;
    accel_cal[2] = data->accelcal.z;

    msg = kzalloc(sizeof(*msg), GFP_KERNEL);
    if (msg == NULL) {
        pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
        return -ENOMEM;
    }
    msg->cmd = MSG2SSP_AP_MCU_SET_ACCEL_CAL;
    msg->length = 6;
    msg->options = AP2HUB_WRITE;
    msg->buffer = (char *) kzalloc(6, GFP_KERNEL);

    msg->free_buffer = 1;
    memcpy(msg->buffer, accel_cal, 6);

	iRet = ssp_send_command(data, msg, 0);
    if (iRet != SUCCESS) {
        pr_err("[SSP]: %s - i2c fail %d\n", __func__, iRet);
        iRet = ERROR;
    }

    pr_info("[SSP] Set accel cal data1 %d, %d, %d\n", accel_cal[0], accel_cal[1], accel_cal[2]);
    return iRet;
}

static int enable_accel_for_cal(struct ssp_data *data)
{
    u8 uBuf[4] = { 0, };
    s32 dMsDelay = get_msdelay(data->adDelayBuf[ACCELEROMETER_SENSOR_32G]);

    memcpy(&uBuf[0], &dMsDelay, 4);

    if (atomic64_read(&data->aSensorEnable) & (1ULL << ACCELEROMETER_SENSOR_32G)) {
        if (get_msdelay(data->adDelayBuf[ACCELEROMETER_SENSOR_32G]) != 10) {
            send_instruction(data, CHANGE_DELAY,
                ACCELEROMETER_SENSOR_32G, uBuf, 4);
            return SUCCESS;
        }
    } else {
        send_instruction(data, ADD_SENSOR,
            ACCELEROMETER_SENSOR_32G, uBuf, 4);
    }

    return FAIL;
}

static void disable_accel_for_cal(struct ssp_data *data, int iDelayChanged)
{
    u8 uBuf[4] = { 0, };
    s32 dMsDelay = get_msdelay(data->adDelayBuf[ACCELEROMETER_SENSOR_32G]);

    memcpy(&uBuf[0], &dMsDelay, 4);

    if (atomic64_read(&data->aSensorEnable) & (1ULL << ACCELEROMETER_SENSOR_32G)) {
        if (iDelayChanged)
            send_instruction(data, CHANGE_DELAY,
                ACCELEROMETER_SENSOR_32G, uBuf, 4);
    } else {
        send_instruction(data, REMOVE_SENSOR,
            ACCELEROMETER_SENSOR_32G, uBuf, 4);
    }
}

static int accel_do_calibrate1(struct ssp_data *data, int iEnable)
{
    int iSum[3] = { 0, };
    int iRet = 0, iCount;

    if (iEnable) {
        data->accelcal.x = 0;
        data->accelcal.y = 0;
        data->accelcal.z = 0;
        set_accel_cal(data);
        iRet = enable_accel_for_cal(data);
        msleep(300);

        for (iCount = 0; iCount < CALIBRATION_DATA_AMOUNT; iCount++) {
            iSum[0] += data->buf[ACCELEROMETER_SENSOR_32G].x;
            iSum[1] += data->buf[ACCELEROMETER_SENSOR_32G].y;
            iSum[2] += data->buf[ACCELEROMETER_SENSOR_32G].z;
            mdelay(10);
        }
            disable_accel_for_cal(data, iRet);

            data->accelcal.x = (iSum[0] / CALIBRATION_DATA_AMOUNT);
            data->accelcal.y = (iSum[1] / CALIBRATION_DATA_AMOUNT);
            data->accelcal.z = (iSum[2] / CALIBRATION_DATA_AMOUNT);

            if (data->accelcal.z > 0)
                data->accelcal.z -= MAX_ACCEL_1G;
            else if (data->accelcal.z < 0)
                data->accelcal.z += MAX_ACCEL_1G;
        } else {
            data->accelcal.x = 0;
            data->accelcal.y = 0;
            data->accelcal.z = 0;
        }

        ssp_dbg("[SSP]: do accel calibrate %d, %d, %d\n",
            data->accelcal.x, data->accelcal.y, data->accelcal.z);

        file_manager_write(data, CALIBRATION_FILE_PATH, (char *)&data->accelcal, sizeof(data->accelcal), 0);

        lsm6dsl_set_accel_cal1(data);
        return iRet;
}

static ssize_t accel_calibration_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int iRet;
    int iCount = 0;
    struct ssp_data *data = dev_get_drvdata(dev);

    mutex_lock(&data->sysfs_op_mtx);
    iRet = lsm6dsl_accel_open_calibration1(data);
    if (iRet < 0)
        pr_err("[SSP]: %s - calibration1 open failed(%d)\n", __func__, iRet);

    ssp_dbg("[SSP] Cal1 data : %d %d %d - %d\n",
        data->accelcal.x, data->accelcal.y, data->accelcal.z, iRet);

    iCount = snprintf(buf, PAGE_SIZE, "%d %d %d %d\n", iRet, data->accelcal.x,
            data->accelcal.y, data->accelcal.z);
    mutex_unlock(&data->sysfs_op_mtx);

    return iCount;
}

static ssize_t accel_calibration_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    int iRet;
    int64_t dEnable;
    struct ssp_data *data = dev_get_drvdata(dev);

    iRet = kstrtoll(buf, 10, &dEnable);
    if (iRet < 0)
        return iRet;

    mutex_lock(&data->sysfs_op_mtx);
    iRet = accel_do_calibrate1(data, (int)dEnable);
    mutex_unlock(&data->sysfs_op_mtx);

    if (iRet < 0)
        pr_err("[SSP]: %s - accel_do_calibrate1() failed\n", __func__);

    return size;
}

static ssize_t raw_data_read(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct ssp_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
        data->buf[ACCELEROMETER_SENSOR_32G].x,
        data->buf[ACCELEROMETER_SENSOR_32G].y,
        data->buf[ACCELEROMETER_SENSOR_32G].z);
}

static ssize_t accel_reactive_alert_store1(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    char chTempBuf = 1;
    int iRet = 0;

    struct ssp_data *data = dev_get_drvdata(dev);

    if (sysfs_streq(buf, "1"))
        ssp_dbg("[SSP]: %s - on\n", __func__);
    else if (sysfs_streq(buf, "0"))
        ssp_dbg("[SSP]: %s - off\n", __func__);
    else if (sysfs_streq(buf, "2")) {
        ssp_dbg("[SSP]: %s - factory\n", __func__);

        mutex_lock(&data->sysfs_op_mtx);
        data->bAccelAlert1 = 0;

        iRet = ssp_send_read_factory_cmd(data, ACCELEROMETER_SENSOR_32G, chTempBuf, (char *)&chTempBuf,
                                sizeof(chTempBuf), 3000, ACCEL32G_REACTIVE_LENGTH, __func__);

        data->bAccelAlert1 = chTempBuf;
        mutex_unlock(&data->sysfs_op_mtx);

        if (iRet != SUCCESS) {
            pr_err("[SSP]: %s - accel1 Selftest Timeout!!\n", __func__);
            goto exit;
        }

        ssp_dbg("[SSP]: %s factory test success!\n", __func__);
    } else {
        pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
        return -EINVAL;
    }
exit:
    return size;
}

static ssize_t accel_reactive_alert_show1(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    bool bSuccess = false;
    struct ssp_data *data = dev_get_drvdata(dev);

    mutex_lock(&data->sysfs_op_mtx);
    if (data->bAccelAlert1 == true)
        bSuccess = true;
    else
        bSuccess = false;

    data->bAccelAlert1 = false;
    mutex_unlock(&data->sysfs_op_mtx);

    return snprintf(buf, PAGE_SIZE, "%u\n", bSuccess);
}

static ssize_t accel_hw_selftest_show1(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    char chTempBuf[14] = { 2, 0, };
    s8 init_status = 0, result = -1;
    s16 shift_ratio[6] = { 0, };
    int iRet;

    struct ssp_data *data = dev_get_drvdata(dev);

    iRet = ssp_send_read_factory_cmd(data, ACCELEROMETER_SENSOR_32G, chTempBuf[0], (char *)chTempBuf,
                                sizeof(chTempBuf), 7000, ACCEL32G_SELFTEST_LENGTH, __func__);

    if (iRet != SUCCESS) {
        pr_err("[SSP] %s - accel1 hw selftest Timeout!!\n", __func__);
        goto exit;
    }

    init_status = chTempBuf[0];
    shift_ratio[0] = (s16)((chTempBuf[2] << 8) + chTempBuf[1]);
    shift_ratio[1] = (s16)((chTempBuf[4] << 8) + chTempBuf[3]);
    shift_ratio[2] = (s16)((chTempBuf[6] << 8) + chTempBuf[5]);
    shift_ratio[3] = (s16)((chTempBuf[8] << 8) + chTempBuf[7]);
    shift_ratio[4] = (s16)((chTempBuf[10] << 8) + chTempBuf[9]);
    shift_ratio[5] = (s16)((chTempBuf[12] << 8) + chTempBuf[11]);
    result = chTempBuf[13];

    pr_info("[SSP] %s - %d, %d, %d, %d, %d, %d, %d, %d\n", __func__,
        init_status, result, shift_ratio[0], shift_ratio[1], shift_ratio[2], shift_ratio[3], shift_ratio[4], shift_ratio[5]);

    return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d\n",
                result, shift_ratio[0], shift_ratio[1], shift_ratio[2], shift_ratio[3], shift_ratio[4], shift_ratio[5]);
exit:
    return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d %d %d %d\n", -5, 0, 0, 0, 0, 0, 0);
}

static ssize_t accel_lowpassfilter_store1(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    int iRet = 0, new_enable = 1;
    struct ssp_data *data = dev_get_drvdata(dev);
    struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

    if (msg == NULL) {
        pr_err("[SSP] %s, failed to alloc memory\n", __func__);
        goto exit;
    }

    if (sysfs_streq(buf, "1"))
        new_enable = 1;
    else if (sysfs_streq(buf, "0"))
        new_enable = 0;
    else
        ssp_dbg("[SSP]: %s - invalid value!\n", __func__);

    msg->cmd = MSG2SSP_AP_SENSOR_LPF;
    msg->length = 1;
    msg->options = AP2HUB_WRITE;
    msg->buffer = (char *) kzalloc(1, GFP_KERNEL);
    if (msg->buffer == NULL) {
        pr_err("[SSP] %s, failed to alloc memory\n", __func__);
        goto exit;
    }

    *msg->buffer = new_enable;
    msg->free_buffer = 1;

	iRet = ssp_send_command(data, msg, 0);
    if (iRet != SUCCESS)
        pr_err("[SSP] %s - fail %d\n", __func__, iRet);
    else
        pr_info("[SSP] %s - %d\n", __func__, new_enable);

exit:
    return size;
}

static DEVICE_ATTR(name, S_IRUGO, accel_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, accel_vendor_show, NULL);
static DEVICE_ATTR(calibration, S_IRUGO | S_IWUSR | S_IWGRP,
    accel_calibration_show, accel_calibration_store);
static DEVICE_ATTR(raw_data, S_IRUGO, raw_data_read, NULL);
static DEVICE_ATTR(reactive_alert, S_IRUGO | S_IWUSR | S_IWGRP,
    accel_reactive_alert_show1, accel_reactive_alert_store1);
static DEVICE_ATTR(selftest, S_IRUGO, accel_hw_selftest_show1, NULL);
static DEVICE_ATTR(lowpassfilter, S_IWUSR | S_IWGRP, NULL, accel_lowpassfilter_store1);

static struct device_attribute *acc_attrs[] = {
    &dev_attr_name,
    &dev_attr_vendor,
    &dev_attr_calibration,
    &dev_attr_raw_data,
    &dev_attr_reactive_alert,
    &dev_attr_selftest,
    &dev_attr_lowpassfilter,
    NULL,
};

void initialize_lsm6dsl_accel1_factorytest(struct ssp_data *data)
{
    sensors_register(data->acc32g_device, data, acc_attrs,
        "accelerometer_sensor1");
}

void remove_lsm6dsl_accel1_factorytest(struct ssp_data *data)
{
    sensors_unregister(data->acc32g_device, acc_attrs);
}
