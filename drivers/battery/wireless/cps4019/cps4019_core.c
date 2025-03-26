/*
 *  cps4019_core.c
 *  Samsung Mobile CPS4019 Core Module
 *
 *  Copyright (C) 2023 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/device.h>

#include "cps4019_core.h"

static DEFINE_MUTEX(io_lock);

int cps4019_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 wbuf[2];
	u8 rbuf[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = wbuf;

	wbuf[0] = (reg & 0xFF00) >> 8;
	wbuf[1] = (reg & 0xFF);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = rbuf;

	mutex_lock(&io_lock);
	ret = i2c_transfer(client->adapter, msg, 2);
	mutex_unlock(&io_lock);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read err reg(0x%x) ret(%d)\n",
			__func__, reg, ret);
		return -1;
	}
	*val = rbuf[0];

	pr_debug("%s: reg = 0x%x, data = 0x%x\n", __func__, reg, *val);

	return ret;
}
EXPORT_SYMBOL(cps4019_reg_read);

int cps4019_reg_bulk_read(struct i2c_client *client, u16 reg, u8 *val, u32 size)
{
	struct i2c_msg msg[2];
	u8 wbuf[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = wbuf;

	wbuf[0] = (reg & 0xFF00) >> 8;
	wbuf[1] = (reg & 0xFF);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size;
	msg[1].buf = val;

	mutex_lock(&io_lock);
	ret = i2c_transfer(client->adapter, msg, 2);
	mutex_unlock(&io_lock);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read err reg(0x%x) ret(%d)\n",
			__func__, reg, ret);
		return -1;
	}

	return ret;
}
EXPORT_SYMBOL(cps4019_reg_bulk_read);

int cps4019_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };
	int ret;

	mutex_lock(&io_lock);
	ret = i2c_master_send(client, data, 3);
	mutex_unlock(&io_lock);
	if (ret < 3) {
		dev_err(&client->dev, "%s: write err reg(0x%x) ret(%d)\n",
			__func__, reg, ret);
		return ret < 0 ? ret : -EIO;
	}

	pr_debug("%s: reg = 0x%x, data = 0x%x\n", __func__, reg, val);
	return 0;
}
EXPORT_SYMBOL(cps4019_reg_write);

#define ADDR_SIZE	2
int cps4019_reg_bulk_write(struct i2c_client *client, u16 reg, u8 *val, u32 size)
{
	unsigned char *data = NULL;
	int ret = 0;

	data = kzalloc(size + ADDR_SIZE, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data[0] = (reg >> 8);
	data[1] = (reg & 0xff);
	memcpy(&data[ADDR_SIZE], val, size);

	mutex_lock(&io_lock);
	ret = i2c_master_send(client, data, size + ADDR_SIZE);
	mutex_unlock(&io_lock);

	kfree(data);
	if (ret <= ADDR_SIZE) {
		dev_err(&client->dev, "%s: write err reg(0x%x) ret(%d)\n",
			__func__, reg, ret);
		return ret < 0 ? ret : -EIO;
	}

	pr_debug("%s: reg = 0x%x, ret = %d\n", __func__, reg, ret);
	return 0;
}
EXPORT_SYMBOL(cps4019_reg_bulk_write);

int cps4019_reg_update(struct i2c_client *client, u16 reg, u8 val, u8 mask)
{
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &data[2];

	mutex_lock(&io_lock);
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		goto err_update;

	data[2] = (val & mask) | (data[2] & (~mask));
	ret = i2c_master_send(client, data, 3);
	if (ret < 0)
		goto err_update;

	mutex_unlock(&io_lock);

	pr_debug("%s: reg = 0x%x, data = 0x%x, mask = 0x%x\n", __func__, reg, val, mask);
	return ret;

err_update:
	mutex_unlock(&io_lock);
	dev_err(&client->dev, "%s: i2c write error, reg: 0x%x, ret: %d\n", __func__, reg, ret);
	return ret < 0 ? ret : -EIO;
}
EXPORT_SYMBOL(cps4019_reg_update);

#define CHECK_VRECT(val)	(val < 20475)
#define CHECK_VOUT(val)		(val < 20475)
int cps4019_get_adc(struct i2c_client *client, int adc_type)
{
	int ret = 0;
	u16 val = 0;

	switch (adc_type) {
	case CPS4019_ADC_VOUT:
		ret = cps4019_reg_bulk_read(client, CPS4019_ADC_VOUT_L_REG, (u8 *)&val, 2);
		ret = CHECK_VOUT(val) ? ret : (-1);
		break;
	case CPS4019_ADC_VRECT:
		ret = cps4019_reg_bulk_read(client, CPS4019_ADC_VRECT_L_REG, (u8 *)&val, 2);
		ret = CHECK_VRECT(val) ? ret : (-1);
		break;
	case CPS4019_ADC_IOUT:
		ret = cps4019_reg_bulk_read(client, CPS4019_ADC_IOUT_L_REG, (u8 *)&val, 2);
		break;
	case CPS4019_ADC_VSYS:
		ret = cps4019_reg_bulk_read(client, CPS4019_ADC_VSYS_L_REG, (u8 *)&val, 2);
		break;
	case CPS4019_ADC_DIE_TEMP:
		ret = cps4019_reg_bulk_read(client, CPS4019_ADC_DIE_TEMP_L_REG, (u8 *)&val, 2);
		break;
	case CPS4019_ADC_OP_FRQ:
		ret = cps4019_reg_bulk_read(client, CPS4019_ADC_OP_FREQ_L_REG, (u8 *)&val, 2);
		break;
	case CPS4019_ADC_PING_FRQ:
		ret = cps4019_reg_bulk_read(client, CPS4019_ADC_PING_FREQ_L_REG, (u8 *)&val, 2);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return (ret >= 0) ? val : 0;
}
EXPORT_SYMBOL(cps4019_get_adc);
