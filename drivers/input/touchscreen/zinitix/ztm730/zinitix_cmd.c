#include "zinitix_ztm730.h"

#ifdef ZINITIX_MISC_DEBUG
static int ts_misc_fops_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ts_misc_fops_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long ts_misc_fops_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct raw_ioctl raw_ioctl;
	struct reg_ioctl reg_ioctl;
	static int m_ts_debug_mode = ZINITIX_DEBUG;
	u8 *u8Data;
	int ret = 0;
	size_t sz = 0;
	u16 mode = 0;
	u16 val;
	int nval = 0;
	u32 size;
	u8 *buff = NULL;
#if IS_ENABLED(CONFIG_COMPAT)
	void __user *argp = compat_ptr(arg);
#else
	void __user *argp = (void __user *)arg;
#endif
	if (misc_info == NULL) {
		pr_err("%s %s:misc device NULL?\n", SECLOG, __func__);
		return -1;
	}

	switch (cmd) {
	case TOUCH_IOCTL_GET_DEBUGMSG_STATE:
		ret = m_ts_debug_mode;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_SET_DEBUGMSG_STATE:
		if (copy_from_user(&nval, argp, sizeof(nval))) {
			input_err(true, &misc_info->client->dev, "%s:copy_from_user\n", __func__);
			return -1;
		}
		if (nval)
			input_err(true, &misc_info->client->dev, "%s:on debug mode (%d)\n", __func__, nval);
		else
			input_err(true, &misc_info->client->dev, "%s:off debug mode (%d)\n", __func__, nval);
		m_ts_debug_mode = nval;
		break;

	case TOUCH_IOCTL_GET_CHIP_REVISION:
		ret = misc_info->cap_info.ic_revision;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_FW_VERSION:
		ret = misc_info->cap_info.fw_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_REG_DATA_VERSION:
		ret = misc_info->cap_info.reg_data_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_SIZE:
		if (copy_from_user(&sz, argp, sizeof(size_t)))
			return -1;

		input_info(true, &misc_info->client->dev, "%s:firmware size = %d\n", __func__, (int)sz);
		if (misc_info->cap_info.ic_fw_size != sz) {
			input_err(true, &misc_info->client->dev, "%s:firmware size error\n", __func__);
			return -1;
		}
		break;
/*
	case TOUCH_IOCTL_VARIFY_UPGRADE_DATA:
		if (copy_from_user(m_firmware_data,
			argp, info->cap_info.ic_fw_size))
			return -1;

		version = (u16) (m_firmware_data[52] | (m_firmware_data[53]<<8));

		input_err(true, &info->client->dev, "%s:firmware version = %x\n", __func__, version);

		if (copy_to_user(argp, &version, sizeof(version)))
			return -1;
		break;
*/
	case TOUCH_IOCTL_START_UPGRADE:
	{
		if (misc_info == NULL) {
			input_err(true, &misc_info->client->dev, "%s:misc device NULL?\n", __func__);
			return -1;
		}

		down(&misc_info->raw_data_lock);
		if (misc_info->update == 0) {
			up(&misc_info->raw_data_lock);
			return -2;
		}

		if (copy_from_user(&raw_ioctl,
			argp, sizeof(struct raw_ioctl))) {
			up(&misc_info->raw_data_lock);
			input_err(true, &misc_info->client->dev, "%s:copy_from_user(2)\n", __func__);
			return -1;
		}

		size = misc_info->cap_info.ic_fw_size;

		buff = kmalloc(size, GFP_KERNEL);

#if IS_ENABLED(CONFIG_COMPAT)
		if (copy_from_user((u8 *)buff, compat_ptr(raw_ioctl.buf), size)) {
#else
		if (copy_from_user((u8 *)buff, (void __user *)(raw_ioctl.buf), size)) {
#endif
			misc_info->work_state = NOTHING;
			if (buff != NULL)
				kfree(buff);
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "%s:copy_from_user(3)\n", __func__);
			return -1;
		}

		ret = ztm730_upgrade_sequence(misc_info, (u8 *)buff);

		if (buff != NULL)
			kfree(buff);

		up(&misc_info->raw_data_lock);
		return ret;
	}

	case TOUCH_IOCTL_GET_X_RESOLUTION:
		ret = misc_info->pdata->x_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_RESOLUTION:
		ret = misc_info->pdata->y_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_X_NODE_NUM:
		ret = misc_info->cap_info.x_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_NODE_NUM:
		ret = misc_info->cap_info.y_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_TOTAL_NODE_NUM:
		ret = misc_info->cap_info.total_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_HW_CALIBRAION:
		ret = -1;
		ztm730_enable_irq(misc_info, false);
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "%s:other process occupied.. (%d)\n",
				__func__, misc_info->work_state);
			ztm730_enable_irq(misc_info, true);
			up(&misc_info->work_lock);
			return -1;
		}
		misc_info->work_state = HW_CALIBRAION;
		sec_delay(100);

		/* h/w calibration */
		if (ztm730_hw_calibration(misc_info))
			ret = 0;

		mode = misc_info->touch_mode;
		ret = ztm730_write_reg(misc_info->client, ZTM730_TOUCH_MODE, mode);
		if (ret != I2C_SUCCESS) {
			input_err(true, &misc_info->client->dev, "%s:failed to set touch mode %d.\n", __func__, mode);
			goto fail_hw_cal;
		}

		ret = ztm730_soft_reset(misc_info);
		if (ret) {
			input_err(true, &misc_info->client->dev, "%s:Failed to write reset command\n", __func__);
			goto fail_hw_cal;
		}

		ztm730_enable_irq(misc_info, true);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;
fail_hw_cal:

		ztm730_enable_irq(misc_info, true);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -1;

	case TOUCH_IOCTL_SET_RAW_DATA_MODE:
		if (misc_info == NULL) {
			input_err(true, &misc_info->client->dev, "%s:misc device NULL?\n", __func__);
			return -1;
		}
		if (copy_from_user(&nval, argp, 4)) {
			input_err(true, &misc_info->client->dev, "%s:copy_from_user\n", __func__);
			misc_info->work_state = NOTHING;
			return -1;
		}

		ret = ztm730_set_touchmode(misc_info, (u16)nval);
		if (ret) {
			input_err(true, &misc_info->client->dev, "%s:Failed to set POINT_MODE\n", __func__);
			misc_info->work_state = NOTHING;
			return -1;
		}

		return 0;

#ifdef ZINITIX_FILE_DEBUG
	case TOUCH_IOCTL_ZI_FILE_DEBUG_DISABLE:

		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "%s:other process occupied.. (%d)\n",
				__func__, misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;
		if (copy_from_user(&reg_ioctl,
				argp, sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "%s:TOUCH_IOCTL_ZI_FILE_DEBUG_DISABLE(1)\n", __func__);
			return -1;
		}

#if IS_ENABLED(CONFIG_COMPAT)
		if (copy_from_user(&val, compat_ptr(reg_ioctl.val), sizeof(val))) {
#else
		if (copy_from_user(&val, (void __user *)(reg_ioctl.val), sizeof(val))) {
#endif
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "%s:TOUCH_IOCTL_ZI_FILE_DEBUG_DISABLE(2)\n", __func__);
			return -1;
		}

		misc_info->g_zini_file_debug_disable = val;

		input_info(true, &misc_info->client->dev, "%s:TOUCH_IOCTL_ZI_FILE_DEBUG_DISABLE = %d\n", __func__, val);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return 0;
#endif

	case TOUCH_IOCTL_GET_REG:
		if (misc_info == NULL) {
			input_err(true, &misc_info->client->dev, "%s:misc device NULL?\n", __func__);
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "%s:other process occupied.. (%d)\n",
				__func__, misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;

		if (copy_from_user(&reg_ioctl,
			argp, sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "%s:copy_from_user(1)\n", __func__);
			return -1;
		}

		if (ztm730_read_data(misc_info->client,
			(u16)reg_ioctl.addr, (u8 *)&val, 2) < 0)
			ret = -1;


		nval = (int)val;

#if IS_ENABLED(CONFIG_COMPAT)
		if (copy_to_user(compat_ptr(reg_ioctl.val), (u8 *)&nval, 4)) {
#else
		if (copy_to_user((void __user *)(reg_ioctl.val), (u8 *)&nval, 4)) {
#endif
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "%s:copy_to_user(2)\n", __func__);
			return -1;
		}

		input_err(true, &misc_info->client->dev, "%s:reg addr = 0x%x, val = 0x%x\n",
			__func__, reg_ioctl.addr, nval);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_SET_REG:

		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "%s:other process occupied.. (%d)\n",
				__func__, misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;
		if (copy_from_user(&reg_ioctl,
				argp, sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "%s:copy_from_user(1)\n", __func__);
			return -1;
		}

#if IS_ENABLED(CONFIG_COMPAT)
		if (copy_from_user(&val, compat_ptr(reg_ioctl.val), sizeof(val))) {
#else
		if (copy_from_user(&val, (void __user *)(reg_ioctl.val), sizeof(val))) {
#endif
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "%s:copy_from_user(2)\n", __func__);
			return -1;
		}

		ret = ztm730_write_reg(misc_info->client, (u16)reg_ioctl.addr, val);
		if (ret != I2C_SUCCESS) {
			input_err(true, &misc_info->client->dev, "%s:failed to set touch mode %d.\n", __func__, mode);
			ret = -1;
		}

		input_err(true, &misc_info->client->dev, "%s:write: reg addr = 0x%x, val = 0x%x\n",
			__func__, reg_ioctl.addr, val);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_DONOT_TOUCH_EVENT:
		if (misc_info == NULL) {
			input_err(true, &misc_info->client->dev, "%s:misc device NULL?\n", __func__);
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "%s:other process occupied.. (%d)\n",
				__func__, misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;
		ret = ztm730_write_reg(misc_info->client, ZTM730_INT_ENABLE_FLAG, 0);
		if (ret != I2C_SUCCESS) {
			input_err(true, &misc_info->client->dev,
				"%s:failed to set ZTM730_INT_ENABLE_FLAG.\n", __func__);
			ret = -1;
		}
		input_err(true, &misc_info->client->dev, "%s:write: reg addr = 0x%x, val = 0x0\n",
			__func__, ZTM730_INT_ENABLE_FLAG);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);

		return ret;

	case TOUCH_IOCTL_SEND_SAVE_STATUS:
		if (misc_info == NULL) {
			input_err(true, &misc_info->client->dev, "%s:misc device NULL?\n", __func__);
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "%s:other process occupied..(%d)\n",
				__func__, misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}
		misc_info->work_state = SET_MODE;
		ret = 0;

		ret = ztm730_write_cmd(misc_info->client, ZTM730_SAVE_STATUS_CMD);
		if (ret != I2C_SUCCESS) {
			input_err(true, &misc_info->client->dev,
				"%s:failed to write ZTM730_SAVE_STATUS_CMD\n", __func__);
			ret =  -1;
		}
		sec_delay(1000);	/* for fusing eeprom */

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);

		return ret;

	case TOUCH_IOCTL_GET_RAW_DATA:
		if (misc_info == NULL) {
			input_err(true, &misc_info->client->dev, "%s:TOUCH_IOCTL_GET_RAW_DATA (1)\n", __func__);
			return -1;
		}

		if (misc_info->touch_mode == TOUCH_POINT_MODE) {
			input_err(true, &misc_info->client->dev, "%s:TOUCH_IOCTL_GET_RAW_DATA (2)\n", __func__);
			return -1;
		}

		down(&misc_info->raw_data_lock);
		if (misc_info->update == 0) {
			up(&misc_info->raw_data_lock);
			input_err(true, &misc_info->client->dev, "%s:TOUCH_IOCTL_GET_RAW_DATA (3)\n", __func__);
			return -2;
		}

		if (copy_from_user(&raw_ioctl,
			argp, sizeof(struct raw_ioctl))) {
			up(&misc_info->raw_data_lock);
			input_err(true, &misc_info->client->dev, "%s:TOUCH_IOCTL_GET_RAW_DATA (4)\n", __func__);
			return -1;
		}

		misc_info->update = 0;

		u8Data = (u8 *)&misc_info->cur_data[0];
		if (raw_ioctl.sz > MAX_TRAW_DATA_SZ * 2)	//36x22 + 4x2 + 2 = 802
			raw_ioctl.sz = MAX_TRAW_DATA_SZ * 2;

#ifdef ZINITIX_FILE_DEBUG
		if (misc_info->g_zini_file_debug_disable) {
			misc_info->g_zini_raw_data_size = raw_ioctl.sz;
			input_info(true, &misc_info->client->dev, "%s:g_zini_file_debug_disable:%d, g_zini_raw_data_size:%d\n",
				__func__, misc_info->g_zini_file_debug_disable, misc_info->g_zini_raw_data_size);
		}
#endif

#if IS_ENABLED(CONFIG_COMPAT)
		if (copy_to_user(compat_ptr(raw_ioctl.buf), (u8 *)u8Data, raw_ioctl.sz)) {
#else
		if (copy_to_user((void __user *)(raw_ioctl.buf), (u8 *)u8Data, raw_ioctl.sz)) {
#endif
			up(&misc_info->raw_data_lock);
			input_err(true, &misc_info->client->dev,
				"%s:TOUCH_IOCTL_GET_RAW_DATA (5)\n", __func__);
			return -1;
		}

		up(&misc_info->raw_data_lock);

		return 0;

	default:
		break;
	}
	return 0;
}

const struct file_operations ts_misc_fops = {
	.owner = THIS_MODULE,
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_close,
	.compat_ioctl = ts_misc_fops_ioctl,
};

struct miscdevice touch_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zinitix_touch_misc",
	.fops = &ts_misc_fops,
};
#endif

int read_fw_verify_result(struct ztm730_info *info)
{
	struct i2c_client *client = info->client;
	s32 ret;
	u16 val;

	ret = ztm730_read_data(info->client,
		ZTM730_CHECKSUM_RESULT, (u8 *)&val, CHECKSUM_VAL_SIZE);
	if (ret < 0) {
		input_err(true, &client->dev,
			"%s:Failed to read CHECKSUM_RESULT register\n", __func__);
		goto out;
	}

	if (val != ZTM730_CHECK_SUM) {
		input_err(true, &client->dev,
			"%s:Failed to check ZTM730_CHECKSUM_RESULT[0x%04x]\n", __func__, val);
		goto out;
	}

	ret = ztm730_read_data(info->client,
		ZTM730_FW_CHECKSUM_REG, (u8 *)&val, CHECKSUM_VAL_SIZE);
	if (ret < 0) {
		input_err(true, &client->dev,
			"%s:Failed to read ZTM730_FW_CHECKSUM_VAL register\n", __func__);
		goto out;
	}

	info->checksum = val;
out:
	return ret;
}

int ztm730_upgrade_sequence(struct ztm730_info *info, const u8 *firmware_data)
{
	struct i2c_client *client = info->client;
	bool ret;

	ztm730_enable_irq(info, false);
	down(&info->work_lock);
	info->work_state = UPGRADE;

	ztm730_clear_report_data(info);

	input_info(true, &client->dev, "%s:start upgrade firmware\n", __func__);

	ret = ztm730_upgrade_firmware(info, firmware_data, info->cap_info.ic_fw_size);
	if (!ret)
		input_err(true, &client->dev, "%s:Failed update firmware\n", __func__);

	ztm730_init_touch(info);

	ztm730_enable_irq(info, true);
	info->work_state = NOTHING;
	up(&info->work_lock);

	return (ret) ? 0 : -1;
}

bool ztm730_set_touchmode(struct ztm730_info *info, u16 value)
{
	int i, ret = 0;

	ztm730_enable_irq(info, false);

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		input_err(true, &info->client->dev, "%s:other process occupied.\n", __func__);
		ztm730_enable_irq(info, true);
		up(&info->work_lock);
		return -1;
	}

	info->work_state = SET_MODE;

	if (value == TOUCH_SEC_MODE)
		info->touch_mode = TOUCH_POINT_MODE;
	else
		info->touch_mode = value;

	input_info(true, &info->client->dev, "%s:%d\n",
			__func__, info->touch_mode);

	if (info->touch_mode == TOUCH_CND_MODE) {
		ret = ztm730_write_reg(info->client, ZTM730_M_U_COUNT, SEC_M_U_COUNT);
		if (ret != I2C_SUCCESS) {
			input_err(true, &info->client->dev,
				"%s:Fail to set U Count [%d]\n",
				__func__, info->touch_mode);

				goto out;
		}

		ret = ztm730_write_reg(info->client, ZTM730_M_N_COUNT, SEC_M_N_COUNT);
		if (ret != I2C_SUCCESS) {
			input_err(true, &info->client->dev,
				"%s:Fail to set N Count [%d]\n",
				__func__, info->touch_mode);

				goto out;
		}

		ret = ztm730_write_reg(info->client, ZTM730_AFE_FREQUENCY, SEC_M_FREQUENCY);
		if (ret != I2C_SUCCESS) {
			input_err(true, &info->client->dev,
				"%s:Fail to set AFE Frequency [%d]\n",
				__func__, info->touch_mode);

				goto out;
		}

		ret = ztm730_write_reg(info->client, ZTM730_M_RST0_TIME, SEC_M_RST0_TIME);
		if (ret != I2C_SUCCESS) {
			input_err(true, &info->client->dev,
				"%s:Fail to set RST0 Time [%d]\n",
				__func__, info->touch_mode);

				goto out;
		}
	}

	ret = ztm730_write_reg(info->client, ZTM730_TOUCH_MODE, info->touch_mode);
	if (ret != I2C_SUCCESS) {
		input_err(true, &info->client->dev,
			"%s:Fail to set ZINITX_TOUCH_MODE [%d]\n",
			__func__, info->touch_mode);

			goto out;
	}

	ret = ztm730_soft_reset(info);
	if (ret) {
		input_err(true, &info->client->dev,
			"%s:Failed to write reset command\n", __func__);
		goto out;
	}
	sec_delay(400);

	/* clear garbage data */
	for (i = 0; i <= INT_CLEAR_RETRY; i++) {
		sec_delay(20);
		ret = ztm730_write_cmd(info->client, ZTM730_CLEAR_INT_STATUS_CMD);
		if (ret != I2C_SUCCESS) {
			input_err(true, &info->client->dev,
				"%s:Failed to clear garbage data[%d/INT_CLEAR_RETRY]\n", __func__, i);

		}
		usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	}

out:
	info->work_state = NOTHING;
	ztm730_enable_irq(info, true);
	up(&info->work_lock);

	return ret;
}

static ssize_t support_feature_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	u32 features = 0;

	if (info->pdata->enable_settings_aot)
		features |= INPUT_FEATURE_ENABLE_SETTINGS_AOT;

	if (info->pdata->enable_sysinput_enabled)
		features |= INPUT_FEATURE_ENABLE_SYSINPUT_ENABLED;

	snprintf(buff, sizeof(buff), "%d", features);

	input_info(true, &info->client->dev, "%s: %d%s%s\n",
			__func__, features,
			features & INPUT_FEATURE_ENABLE_SETTINGS_AOT ? " aot" : "",
			features & INPUT_FEATURE_ENABLE_SYSINPUT_ENABLED ? " SE" : "");

	return snprintf(buf, SEC_CMD_BUF_SIZE, "%s\n", buff);
}

ssize_t enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);

	input_info(true, &info->client->dev, "%s: %d\n", __func__, info->enabled);
	return scnprintf(buf, PAGE_SIZE, "%d", info->enabled);
}

ssize_t enabled_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	int buff[2];
	int err;

	if (!info->pdata->enable_sysinput_enabled)
		return -EINVAL;

	err = sscanf(buf, "%d,%d", &buff[0], &buff[1]);
	if (err != 2) {
		input_err(true, &info->client->dev,
				"%s: failed read params [%d]\n", __func__, err);
		return -EINVAL;

	}

#if 0
	if (!info->info_work_done) {
		input_info(true, &info->client->dev, "%s: not finished info work\n", __func__);
		return count;
	}
#endif

	if (buff[0] == DISPLAY_STATE_ON && buff[1] == DISPLAY_EVENT_LATE) {
		if (info->enabled) {
			input_err(true, &info->client->dev, "%s: device already enabled\n", __func__);
			goto out;
		}
		input_info(true, &info->client->dev, "%s: [%s] enable\n", __func__, current->comm);
		ztm730_input_open(info->input_dev);
		if (info->pdata->support_bezel_detent)
			ztm730_bezel_open(info->input_dev);
	} else if (buff[0] == DISPLAY_STATE_OFF && buff[1] == DISPLAY_EVENT_EARLY) {
		if (!info->enabled) {
			input_info(true, &info->client->dev, "%s: [%s] device already disabled\n", __func__, current->comm);
			goto out;
		}
		input_info(true, &info->client->dev, "%s: [%s] disable\n", __func__, current->comm);
		if (info->pdata->support_bezel_detent)
			ztm730_bezel_close(info->input_dev);
		ztm730_input_close(info->input_dev);
	} else if (buff[0] == DISPLAY_STATE_FORCE_ON) {
		if (info->enabled) {
			input_err(true, &info->client->dev, "%s: device already enabled\n", __func__);
			goto out;
		}
		input_info(true, &info->client->dev, "%s: [%s] DISPLAY_STATE_FORCE_ON\n", __func__, current->comm);

		ztm730_input_open(info->input_dev);
		if (info->pdata->support_bezel_detent)
			ztm730_bezel_open(info->input_dev);

	} else if (buff[0] == DISPLAY_STATE_FORCE_OFF || buff[0] == DISPLAY_STATE_LPM_OFF) {
		if (!info->enabled) {
			input_err(true, &info->client->dev, "%s: device already disabled\n", __func__);
			goto out;
		}
		input_info(true, &info->client->dev, "%s: [%s] DISPLAY_STATE_FORCE_OFF\n", __func__, current->comm);

		if (info->pdata->support_bezel_detent)
			ztm730_bezel_close(info->input_dev);
		ztm730_input_close(info->input_dev);
	} else if (buff[0] == DISPLAY_STATE_DOZE || buff[0] == DISPLAY_STATE_DOZE_SUSPEND) {
		if (!info->enabled) {
			input_err(true, &info->client->dev, "%s: device already disabled\n", __func__);
			goto out;
		}
		input_info(true, &info->client->dev, "%s: [%s] DISPLAY_STATE_DOZE. AOD\n", __func__, current->comm);

		if (info->pdata->support_bezel_detent)
			ztm730_bezel_close(info->input_dev);
		ztm730_input_close(info->input_dev);
	}

out:
	return count;
}

#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
static u16 ts_get_touch_reg(u16 addr)
{
	int ret = 1;
	u16 reg_value;

	disable_irq(misc_info->irq);

	down(&misc_info->work_lock);
	if (misc_info->work_state != NOTHING) {
		input_info(true, &misc_info->client->dev, "other process occupied.. (%d)\n",
				misc_info->work_state);
		enable_irq(misc_info->irq);
		up(&misc_info->work_lock);
		return -1;
	}
	misc_info->work_state = SET_MODE;

//	ztm730_write_reg(misc_info->client, 0x0A, 0x0A);
	usleep_range(20 * 1000, 20 * 1000);
//	ztm730_write_reg(misc_info->client, 0x0A, 0x0A);

	ret = ztm730_read_data(misc_info->client, addr, (u8 *)&reg_value, 2);
	if (ret < 0)
		input_err(true, &misc_info->client->dev, "%s: fail read touch reg\n", __func__);

	misc_info->work_state = NOTHING;
	enable_irq(misc_info->irq);
	up(&misc_info->work_lock);

	return reg_value;
}

static void ts_set_touch_reg(u16 addr, u16 value)
{
	disable_irq(misc_info->irq);

	down(&misc_info->work_lock);
	if (misc_info->work_state != NOTHING) {
		input_info(true, &misc_info->client->dev, "other process occupied.. (%d)\n",
				misc_info->work_state);
		enable_irq(misc_info->irq);
		up(&misc_info->work_lock);
		return;
	}
	misc_info->work_state = SET_MODE;

//	ztm730_write_reg(misc_info->client, 0x0A, 0x0A);
	usleep_range(20 * 1000, 20 * 1000);
//	ztm730_write_reg(misc_info->client, 0x0A, 0x0A);

	if (ztm730_write_reg(misc_info->client, addr, value) != I2C_SUCCESS)
		input_err(true, &misc_info->client->dev, "%s: fail write touch reg\n", __func__);

	misc_info->work_state = NOTHING;
	enable_irq(misc_info->irq);
	up(&misc_info->work_lock);
}

static ssize_t read_reg_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);

	input_info(true, &info->client->dev, "%s: 0x%x\n", __func__,
			info->store_reg_data);

	return snprintf(buf, SEC_CMD_BUF_SIZE, "0x%x", info->store_reg_data);
}

static ssize_t read_reg_data_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	u32 buff[2] = {0, }; //addr, size
	int ret;

	ret = sscanf(buf, "0x%x,0x%x", &buff[0], &buff[1]);
	if (ret != 2) {
		input_err(true, &info->client->dev,
				"%s: failed read params[0x%x]\n", __func__, ret);
		return -EINVAL;
	}

	if (buff[1] != 1 && buff[1] != 2) {
		input_err(true, &info->client->dev,
				"%s: incorrect byte length [0x%x]\n", __func__, buff[1]);
		return -EINVAL;
	}

	info->store_reg_data = ts_get_touch_reg((u16)buff[0]);

	if (buff[1] == 1)
		info->store_reg_data = info->store_reg_data & 0x00FF;

	input_info(true, &info->client->dev,
			"%s: read touch reg [addr:0x%x][data:0x%x]\n",
			__func__, buff[0], info->store_reg_data);

	return size;
}

static ssize_t write_reg_data_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	int buff[3];
	int ret;

	ret = sscanf(buf, "0x%x,0x%x,0x%x", &buff[0], &buff[1], &buff[2]);
	if (ret != 3) {
		input_err(true, &info->client->dev,
				"%s: failed read params[0x%x]\n", __func__, ret);
		return -EINVAL;
	}

	if (buff[1] != 1 && buff[1] != 2) {
		input_err(true, &info->client->dev,
				"%s: incorrect byte length [0x%x]\n", __func__, buff[1]);
		return -EINVAL;
	}

	if (buff[1] == 1)
		buff[2] = buff[2] & 0x00FF;

	ts_set_touch_reg((u16)buff[0], (u16)buff[2]);
	input_info(true, &info->client->dev,
			"%s: write touch reg [addr:0x%x][byte:0x%x][data:0x%x]\n",
			__func__, buff[0], buff[1], buff[2]);

	return size;
}
#endif

static DEVICE_ATTR(support_feature, 0444, support_feature_show, NULL);
static DEVICE_ATTR(enabled, 0664, enabled_show, enabled_store);
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
static DEVICE_ATTR_RW(read_reg_data);
static DEVICE_ATTR_WO(write_reg_data);
#endif

static struct attribute *sysfs_attributes[] = {
	&dev_attr_support_feature.attr,
	&dev_attr_enabled.attr,
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
	&dev_attr_read_reg_data.attr,
	&dev_attr_write_reg_data.attr,
#endif
	NULL,
};

static struct attribute_group sysfs_attr_group = {
	.attrs = sysfs_attributes,
};

static void fw_update(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct zxt_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	char fw_path[SEC_TS_MAX_FW_PATH+1];
	char result[16] = {0};
	const struct firmware *tsp_fw = NULL;
	int ret;
#if 0
#if IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP) && IS_ENABLED(CONFIG_SPU_VERIFY)
	int ori_size;
	int spu_ret;
#endif
#endif

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(result, sizeof(result), "NG");
		sec_cmd_set_cmd_result(sec, result, strnlen(result, sizeof(result)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	switch (sec->cmd_param[0]) {
	case BUILT_IN:
		if (!pdata->fw_name) {
			input_err(true, &client->dev, "%s: firmware_name is NULL\n", __func__);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			snprintf(result, sizeof(result), "%s", "NG");
			goto err;
		}

		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", pdata->fw_name);

		ret = request_firmware(&tsp_fw, fw_path, &(client->dev));
		if (ret) {
			input_err(true, &client->dev,
				"%s: Firmware image %s not available\n", __func__, fw_path);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			snprintf(result, sizeof(result), "%s", "NG");
			goto err;
 		}

		ret = ztm730_upgrade_sequence(info, tsp_fw->data);
		release_firmware(tsp_fw);
		if (ret < 0) {
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			snprintf(result, sizeof(result), "%s", "NG");
			goto err;
		}
		break;
	case UMS:
#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", TSP_EXTERNAL_FW);
#else
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", TSP_EXTERNAL_FW_SIGNED);
#endif
		ret = request_firmware(&tsp_fw, fw_path, &(client->dev));
		if (ret) {
			input_err(true, &client->dev,
				"%s: Firmware image %s not available\n", __func__, fw_path);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			snprintf(result, sizeof(result), "%s", "NG");
			goto err;
		}

#if 0
#if IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP) && IS_ENABLED(CONFIG_SPU_VERIFY)
		ori_size = tsp_fw->size - SPU_METADATA_SIZE(TSP);

		spu_ret = spu_firmware_signature_verify("TSP", tsp_fw->data, tsp_fw->size);
		if (ori_size != spu_ret) {
			input_err(true, &client->dev, "%s: signature verify failed, ori:%d, fsize:%ld\n",
					__func__, ori_size, tsp_fw->size);
			release_firmware(tsp_fw);
			goto err;
		}
#endif
#endif
		ret = ztm730_upgrade_sequence(info, tsp_fw->data);
		release_firmware(tsp_fw);
		if (ret < 0) {
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			snprintf(result, sizeof(result), "%s", "NG");
			goto err;
		}
		break;
	default:
		input_err(true, &client->dev, "%s: invalid fw file type!!\n", __func__);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(result, sizeof(result), "%s", "NG");
		goto err;
	}

	sec->cmd_state = SEC_CMD_STATUS_OK;
	snprintf(result, sizeof(result), "%s", "OK");

err:
	sec_cmd_set_cmd_result(sec, result, strnlen(result, sizeof(result)));
	input_info(true, &client->dev, "%s: \"%s\"\n", __func__, sec->cmd_result);
}

int read_fw_ver_bin(struct ztm730_info *info)
{
	const struct firmware *tsp_fw = NULL;
	char fw_path[SEC_TS_MAX_FW_PATH];
	int ret;

	if (info->fw_data == NULL) {
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", info->pdata->fw_name);
		ret = request_firmware(&tsp_fw, fw_path, &info->client->dev);
		if (ret) {
			input_err(true, &info->client->dev, "%s:failed to request_firmware %s\n",
						__func__, fw_path);
			return -1;
		}
		info->fw_data = (unsigned char *)tsp_fw->data;
	}

	info->img_version_of_bin[0] = (u16)(info->fw_data[68] | (info->fw_data[69] << 8));
	info->img_version_of_bin[1] = (u16)(info->fw_data[56] | (info->fw_data[57]<<8));
	info->img_version_of_bin[2] = (u16)(info->fw_data[48] | (info->fw_data[49] << 8));
	info->img_version_of_bin[3] = (u16)(info->fw_data[60] | (info->fw_data[61] << 8));

	release_firmware(tsp_fw);
	info->fw_data = NULL;

	return 0;
}

static void get_fw_ver_bin(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[16] = { 0 };
	u32 version;
	int ret;

	sec_cmd_set_default_result(sec);

	ret = read_fw_ver_bin(info);
	if (ret) {
		input_err(true, &info->client->dev, "%s: binary fw info is not read\n",
				__func__);
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto NG;
	}

	version = (u32)(info->img_version_of_bin[0] << 24) | (info->img_version_of_bin[1] << 16) |
			(info->img_version_of_bin[2] << 8) | info->img_version_of_bin[3];

	snprintf(buff, sizeof(buff), "ZI%08X", version);
	sec->cmd_state = SEC_CMD_STATUS_OK;

NG:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "FW_VER_BIN");

	input_info(true, &info->client->dev, "%s: \"%s\"\n", __func__, sec->cmd_result);
}

static void get_fw_ver_ic(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[16] = { 0 };
	char model[16] = { 0 };
	u16 vendor_id;
	u32 version, length;
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	ret = ztm730_ic_version_check(info);
	if (ret < 0) {
		input_err(true, &info->client->dev, "%s: firmware version read error\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	vendor_id = ntohs(info->cap_info.vendor_id);
	version = (u32)((u32)(info->cap_info.ic_revision & 0xff) << 24)
		| ((info->cap_info.fw_minor_version & 0xff) << 16)
		| ((info->cap_info.hw_id & 0xff) << 8) | (info->cap_info.reg_data_version & 0xff);

	length = sizeof(vendor_id);
	snprintf(buff, length + 1, "%s", (u8 *)&vendor_id);
	snprintf(buff + length, sizeof(buff) - length, "%08X", version);
	snprintf(model, length + 1, "%s", (u8 *)&vendor_id);
	snprintf(model + length, sizeof(model) - length, "%04X", version >> 16);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING) {
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "FW_VER_IC");
		sec_cmd_set_cmd_result_all(sec, model, strnlen(model, sizeof(model)), "FW_MODEL");
	}
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &info->client->dev, "%s: \"%s\"\n", __func__, sec->cmd_result);
}

static void get_chip_vendor(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[16] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buff, sizeof(buff), "%s", ZTM730_VENDOR_NAME);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "IC_VENDOR");
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
	return;
}

static void get_chip_name(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[16] = { 0 };

	sec_cmd_set_default_result(sec);

	strncpy(buff, "ZTM730", sizeof(buff));
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "IC_NAME");
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
	return;
}

static void check_connection(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[16] = { 0 };
	u16 threshold = 0;
	int ret;


	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: Touch is stopped!\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	ztm730_enable_irq(info, false);
	ret = ztm730_read_data(info->client, ZTM730_CONNECTED_REG, (char *)&threshold, 2);
	if (ret < 0) {
		input_err(true, &info->client->dev, "%s:Failed to read ZTM730_CONNECTED_REG\n", __func__);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buff, sizeof(buff), "NG");
		goto out;
	}

	if (threshold >= TSP_CONNECTED_VALID) {
		sec->cmd_state = SEC_CMD_STATUS_OK;
		snprintf(buff, sizeof(buff), "OK");
	} else {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		snprintf(buff, sizeof(buff), "NG");
	}

	input_info(true, &info->client->dev, "%s:trehshold = %d\n", __func__, threshold);

out:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	ztm730_enable_irq(info, true);

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
}

static void get_x_num(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[16] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buff, sizeof(buff), "%u", info->cap_info.x_node_num);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
	return;
}

static void get_y_num(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[16] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buff, sizeof(buff), "%u", info->cap_info.y_node_num);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
	return;
}

static void ztm730_print_frame(struct ztm730_info *info, s16 *buff, int x_node, int y_node)
{
	unsigned char *pStr = NULL;
	unsigned char pTmp[16] = { 0 };
	int offset;
	int i, j;
	int lsize = 7 * (x_node + 1);

	pStr = kzalloc(lsize, GFP_KERNEL);
	if (!pStr)
		return;

	snprintf(pTmp, sizeof(pTmp), "  Y   ");
	strlcat(pStr, pTmp, lsize);

	for (i = 0; i < y_node; i++) {
		snprintf(pTmp, sizeof(pTmp), "  %02d  ", i);
		strlcat(pStr, pTmp, lsize);
	}

	input_raw_info(true, &info->client->dev, "%s\n", pStr);
	memset(pStr, 0x0, lsize);
	snprintf(pTmp, sizeof(pTmp), " +");
	strlcat(pStr, pTmp, lsize);

	for (i = 0; i < y_node; i++) {
		snprintf(pTmp, sizeof(pTmp), "------");
		strlcat(pStr, pTmp, lsize);
	}

	input_raw_info(true, &info->client->dev, "%s\n", pStr);

	for (i = 0; i < x_node; i++) {
		memset(pStr, 0x0, lsize);
		snprintf(pTmp, sizeof(pTmp), "X%02d |", i);
		strlcat(pStr, pTmp, lsize);

		for (j = 0; j < y_node; j++) {
			offset = i * y_node + j;

			snprintf(pTmp, sizeof(pTmp), " %5d", buff[offset]);

			strlcat(pStr, pTmp, lsize);
		}

		input_raw_info(true, &info->client->dev, "%s\n", pStr);
	}

	kfree(pStr);
}

static void ztm730_print_channel(struct ztm730_info *info, s16 *buff)
{
	unsigned char *pStr = NULL;
	unsigned char pTmp[16] = { 0 };
	int offset;
	int i, j;
	int lsize = 7 * (info->cap_info.x_node_num + 1);

	pStr = kzalloc(lsize, GFP_KERNEL);
	if (!pStr)
		return;

	snprintf(pTmp, sizeof(pTmp), "      ");
	strlcat(pStr, pTmp, lsize);

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		snprintf(pTmp, sizeof(pTmp), "  %02d  ", i);
		strlcat(pStr, pTmp, lsize);
	}

	input_raw_info(true, &info->client->dev, "%s\n", pStr);
	memset(pStr, 0x0, lsize);
	snprintf(pTmp, sizeof(pTmp), " +");
	strlcat(pStr, pTmp, lsize);

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		snprintf(pTmp, sizeof(pTmp), "------");
		strlcat(pStr, pTmp, lsize);
	}

	input_raw_info(true, &info->client->dev, "%s\n", pStr);

	for (i = 0; i < 2; i++) {
		memset(pStr, 0x0, lsize);
		snprintf(pTmp, sizeof(pTmp), "%d |", i);
		strlcat(pStr, pTmp, lsize);

		for (j = 0; j < info->cap_info.x_node_num; j++) {
			offset = i * info->cap_info.x_node_num + j;

			snprintf(pTmp, sizeof(pTmp), " %5d", buff[offset]);

			strlcat(pStr, pTmp, lsize);
		}

		input_raw_info(true, &info->client->dev, "%s\n", pStr);
	}

	kfree(pStr);
}

static int get_raw_data(struct ztm730_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct zxt_ts_platform_data *pdata = info->pdata;
	u32 total_node = info->cap_info.total_node_num;
	u32 sz;
	int i, j = 0;
	int retry_cnt;

	ztm730_enable_irq(info, false);

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		input_info(true, &client->dev, "%s: other process occupied(%d)\n",
			__func__, info->work_state);
		ztm730_enable_irq(info, true);
		up(&info->work_lock);
		return -1;
	}

	info->work_state = RAW_DATA;

	if (info->touch_mode == TOUCH_POINT_MODE)
		retry_cnt = 30;
	else
		retry_cnt = 150;

	for (i = 0; i < skip_cnt; i++) {
		j = 0;
		while (gpio_get_value(pdata->gpio_int)) {
			sec_delay(7);
			if (++j > retry_cnt) {
				input_err(true, &info->client->dev, "%s: (skip_cnt) wait int timeout\n", __func__);
				goto error_out;
			}
		}

		ztm730_write_cmd(client, ZTM730_CLEAR_INT_STATUS_CMD);
		sec_delay(1);
	}

	input_info(true, &client->dev, "%s: read raw data\n", __func__);
	sz = total_node * 2;

	j = 0;
	while (gpio_get_value(pdata->gpio_int)) {
		sec_delay(7);
		if (++j > retry_cnt) {
			input_err(true, &info->client->dev, "%s: wait int timeout\n", __func__);
			goto error_out;
		}
	}

	if (ztm730_read_raw_data(client, ZTM730_RAWDATA_REG, (char *)buff, sz) < 0) {
		input_err(true, &info->client->dev, "%s: error read zinitix tc raw data\n", __func__);
		goto error_out;
	}

	ztm730_write_cmd(client, ZTM730_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	ztm730_enable_irq(info, true);
	up(&info->work_lock);

	return 0;

error_out:
	ztm730_write_cmd(client, ZTM730_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	ztm730_enable_irq(info, true);
	up(&info->work_lock);

	return -1;
}

static int ts_set_touchmode(struct ztm730_info *info, u16 value)
{
	int i, ret = 1;
	int retry_cnt = 0;
	struct capa_info *cap = &info->cap_info;

	ztm730_enable_irq(info, false);

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		input_err(true, &info->client->dev, "%s: other process occupied.. (%d)\n",
			__func__, info->work_state);
		ztm730_enable_irq(info, true);
		up(&info->work_lock);
		return -1;
	}

retry_ts_set_touchmode:

	info->work_state = SET_MODE;

	if (value == TOUCH_SEC_MODE)
		info->touch_mode = TOUCH_POINT_MODE;
	else
		info->touch_mode = value;

	input_info(true, &info->client->dev, "%s: %d\n",
		__func__, info->touch_mode);

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (ztm730_write_reg(info->client, ZTM730_DELAY_RAW_FOR_HOST, RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS)
			input_err(true, &info->client->dev,
				"%s: Fail to set DELAY_RAW_FOR_HOST\n", __func__);
	}

	if (ztm730_write_reg(info->client, ZTM730_TOUCH_MODE, info->touch_mode) != I2C_SUCCESS)
		input_err(true, &info->client->dev,
			"%s: Fail to set TOUCH_MODE %d\n",
			__func__, info->touch_mode);

	input_dbg(false, &info->client->dev, "%s: write regiter end\n", __func__);

	ret = ztm730_read_data(info->client, ZTM730_TOUCH_MODE, (u8 *)&cap->current_touch_mode, 2);
	if (ret < 0) {
		input_err(true, &info->client->dev, "%s: fail touch mode read\n", __func__);
		goto out;
	}

	if (cap->current_touch_mode != info->touch_mode) {
		if (retry_cnt < 1) {
			retry_cnt++;
			goto retry_ts_set_touchmode;
		}
		input_err(true, &info->client->dev, "%s: fail to set touch_mode %d (current_touch_mode %d)\n",
				__func__, info->touch_mode, cap->current_touch_mode);
		ret = -1;
		goto out;
	}

	/* clear garbage data */
	for (i = 0; i < 10; i++) {
		sec_delay(20);
		ztm730_write_cmd(info->client, ZTM730_CLEAR_INT_STATUS_CMD);
	}
	input_dbg(false, &info->client->dev, "%s: garbage data end\n", __func__);

out:
	info->work_state = NOTHING;
	ztm730_enable_irq(info, true);
	up(&info->work_lock);

	return ret;
}

static void run_dnd_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	u16 min, max;
	s32 i, j;
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	ret = ts_set_touchmode(info, TOUCH_DND_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->dnd_data, 2);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	min = 0xFFFF;
	max = 0x0000;

	input_info(true, &client->dev, "%s: start\n", __func__);
	for (i = 0; i < info->cap_info.x_node_num; i++) {
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			if (raw_data->dnd_data[i * info->cap_info.y_node_num + j] < min &&
				raw_data->dnd_data[i * info->cap_info.y_node_num + j] != 0)
				min = raw_data->dnd_data[i * info->cap_info.y_node_num + j];
			if (raw_data->dnd_data[i * info->cap_info.y_node_num + j] > max)
				max = raw_data->dnd_data[i * info->cap_info.y_node_num + j];
		}
	}

	ztm730_print_frame(info, raw_data->dnd_data, info->cap_info.x_node_num, info->cap_info.y_node_num);

	snprintf(buff, sizeof(buff), "%d,%d", min, max);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "DND");
	sec->cmd_state = SEC_CMD_STATUS_OK;

out:
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
			sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "DND");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
	input_info(true, &client->dev, "%s: \"%s\"\n", __func__, sec->cmd_result);

}

static void run_dnd_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[16] = { 0 };
	char *all_cmdbuff;
	s32 i, j;
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	if (ztm730_fix_active_mode(info, true) != I2C_SUCCESS) {
		ret = -1;
		goto out;
	}

	input_info(true, &info->client->dev, "%s: start\n", __func__);
	ret = ts_set_touchmode(info, TOUCH_DND_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->dnd_data, 1);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	all_cmdbuff = kzalloc(info->cap_info.x_node_num*info->cap_info.y_node_num * 6, GFP_KERNEL);
	if (!all_cmdbuff) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			snprintf(buff, sizeof(buff), "%d,", raw_data->dnd_data[i * info->cap_info.y_node_num + j]);
			strcat(all_cmdbuff, buff);
		}
	}

	ztm730_print_frame(info, raw_data->dnd_data, info->cap_info.x_node_num, info->cap_info.y_node_num);

	sec_cmd_set_cmd_result(sec, all_cmdbuff,
			strnlen(all_cmdbuff, sizeof(all_cmdbuff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	kfree(all_cmdbuff);
out:
	ts_set_touchmode(info, false);
	ztm730_fix_active_mode(info, false);

	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
}

static void run_dnd_v_gap_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	char buff_onecmd_1[SEC_CMD_STR_LEN] = { 0 };
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val;
	u16 screen_max = 0x0000;

	sec_cmd_set_default_result(sec);

	memset(raw_data->vgap_data, 0x00, TSP_CMD_NODE_NUM);

	input_info(true, &client->dev, "%s: start\n", __func__);
	for (i = 0; i < x_num - 1; i++) {
		for (j = 0; j < y_num; j++) {
			offset = (i * y_num) + j;
			cur_val = raw_data->dnd_data[offset];
			next_val = raw_data->dnd_data[offset + y_num];
			if (!next_val) {
				raw_data->vgap_data[offset] = next_val;
				continue;
			}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
			else
				val = 100 - ((next_val * 100) / cur_val);

			raw_data->vgap_data[offset] = val;

			if (raw_data->vgap_data[i * y_num + j] > screen_max)
				screen_max = raw_data->vgap_data[i * y_num + j];
		}
	}

	ztm730_print_frame(info, raw_data->vgap_data, info->cap_info.x_node_num, info->cap_info.y_node_num);

	input_info(true, &client->dev, "%s: screen_max %d\n", __func__, screen_max);
	snprintf(buff, sizeof(buff), "%d", screen_max);
	snprintf(buff_onecmd_1, sizeof(buff_onecmd_1), "%d,%d", 0, screen_max);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff_onecmd_1, strnlen(buff_onecmd_1, sizeof(buff_onecmd_1)), "DND_V_GAP");
	sec->cmd_state = SEC_CMD_STATUS_OK;
}

static void run_dnd_h_gap_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	char buff_onecmd_1[SEC_CMD_STR_LEN] = { 0 };
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val;
	u16 screen_max = 0x0000;

	sec_cmd_set_default_result(sec);

	memset(raw_data->hgap_data, 0x00, TSP_CMD_NODE_NUM);

	input_info(true, &client->dev, "%s: start\n", __func__);
	for (i = 0; i < x_num ; i++) {
		for (j = 0; j < y_num - 1; j++) {
			offset = (i * y_num) + j;
			cur_val = raw_data->dnd_data[offset];
			if (!cur_val) {
				raw_data->hgap_data[offset] = cur_val;
				continue;
			}
			next_val = raw_data->dnd_data[offset + 1];
			if (!next_val) {
				raw_data->hgap_data[offset] = next_val;
				for (++j; j < y_num - 1; j++) {
					offset = (i * y_num) + j;

					next_val = raw_data->dnd_data[offset];
					if (!next_val) {
						raw_data->hgap_data[offset] = next_val;
						continue;
					}
					break;
				}
				continue;
			}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
			else
				val = 100 - ((next_val * 100) / cur_val);

			raw_data->hgap_data[offset] = val;
			if (raw_data->hgap_data[i * y_num + j] > screen_max)
				screen_max = raw_data->hgap_data[i * y_num + j];
		}
	}

	ztm730_print_frame(info, raw_data->hgap_data, info->cap_info.x_node_num, info->cap_info.y_node_num);

	input_info(true, &client->dev, "%s: screen_max %d\n", __func__, screen_max);
	snprintf(buff, sizeof(buff), "%d", screen_max);
	snprintf(buff_onecmd_1, sizeof(buff_onecmd_1), "%d,%d", 0, screen_max);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff_onecmd_1, strnlen(buff_onecmd_1, sizeof(buff_onecmd_1)), "DND_H_GAP");
	sec->cmd_state = SEC_CMD_STATUS_OK;
}

#define CMD_RESULT_WORD_LEN	10
static void run_dnd_v_gap_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct tsp_raw_data *raw_data = info->raw_data;
	char temp[SEC_CMD_STR_LEN] = { 0 };
	char *buff = NULL;
	int total_node = info->cap_info.x_node_num * info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val;

	sec_cmd_set_default_result(sec);

	buff = kzalloc(total_node * CMD_RESULT_WORD_LEN, GFP_KERNEL);
	if (!buff) {
		snprintf(temp, SEC_CMD_STR_LEN, "NG");
		sec_cmd_set_cmd_result(sec, temp, SEC_CMD_STR_LEN);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	input_info(true, &info->client->dev, "%s: start\n", __func__);
	for (i = 0; i < info->cap_info.x_node_num - 1; i++) {
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			offset = (i * info->cap_info.y_node_num) + j;
			cur_val = raw_data->dnd_data[offset];
			next_val = raw_data->dnd_data[offset + info->cap_info.y_node_num];
			if (!next_val || !cur_val) {
				raw_data->vgap_data[offset] = 100;
				if (!next_val && !cur_val)
					raw_data->vgap_data[offset] = 0;
				snprintf(temp, CMD_RESULT_WORD_LEN, "%d,", raw_data->vgap_data[offset]);
				strncat(buff, temp, CMD_RESULT_WORD_LEN);
				memset(temp, 0x00, SEC_CMD_STR_LEN);
				continue;
			}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
			else
				val = 100 - ((next_val * 100) / cur_val);
			raw_data->vgap_data[offset] = val;

			snprintf(temp, CMD_RESULT_WORD_LEN, "%d,", raw_data->vgap_data[offset]);
			strncat(buff, temp, CMD_RESULT_WORD_LEN);
			memset(temp, 0x00, SEC_CMD_STR_LEN);
		}
	}

	ztm730_print_frame(info, raw_data->vgap_data, info->cap_info.x_node_num - 1, info->cap_info.y_node_num);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, total_node * CMD_RESULT_WORD_LEN));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	kfree(buff);
}

static void run_dnd_h_gap_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct tsp_raw_data *raw_data = info->raw_data;
	char temp[SEC_CMD_STR_LEN] = { 0 };
	char *buff = NULL;
	int total_node = info->cap_info.x_node_num * info->cap_info.y_node_num;
	int i, j, dnd_offset, hgap_offset, val, cur_val, next_val;

	sec_cmd_set_default_result(sec);

	buff = kzalloc(total_node * CMD_RESULT_WORD_LEN, GFP_KERNEL);
	if (!buff) {
		snprintf(temp, SEC_CMD_STR_LEN, "NG");
		sec_cmd_set_cmd_result(sec, temp, SEC_CMD_STR_LEN);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	input_info(true, &info->client->dev, "%s: start\n", __func__);
	for (i = 0; i < info->cap_info.x_node_num; i++) {
		for (j = 0; j < info->cap_info.y_node_num - 1; j++) {
			hgap_offset = (i * (info->cap_info.y_node_num - 1)) + j;
			dnd_offset = (i * info->cap_info.y_node_num) + j;
			cur_val = raw_data->dnd_data[dnd_offset];
			next_val = raw_data->dnd_data[dnd_offset + 1];

			if (!cur_val || !next_val) {
				raw_data->hgap_data[hgap_offset] = 100;
				if (!cur_val && !next_val)
					raw_data->hgap_data[hgap_offset] = 0;
				snprintf(temp, CMD_RESULT_WORD_LEN, "%d,", raw_data->hgap_data[hgap_offset]);
				strncat(buff, temp, CMD_RESULT_WORD_LEN);
				memset(temp, 0x00, SEC_CMD_STR_LEN);
				continue;
			}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
			else
				val = 100 - ((next_val * 100) / cur_val);
			raw_data->hgap_data[hgap_offset] = val;

			snprintf(temp, CMD_RESULT_WORD_LEN, "%d,", raw_data->hgap_data[hgap_offset]);
			strncat(buff, temp, CMD_RESULT_WORD_LEN);
			memset(temp, 0x00, SEC_CMD_STR_LEN);
		}
	}

	ztm730_print_frame(info, raw_data->hgap_data, info->cap_info.x_node_num, info->cap_info.y_node_num - 1);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, total_node * CMD_RESULT_WORD_LEN));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	kfree(buff);
}

#define TRX_SHORT_RX_MAX	100
#define TRX_SHORT_TX_MAX	200
static void run_trx_short_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	s32 j;
	u16 min, max;
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	ret = ts_set_touchmode(info, TOUCH_TRXSHORT_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->trxshort_data, 2);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	min = 0xFFFF;
	max = 0x0000;

	for (j = 0; j < info->cap_info.y_node_num; j++) {
		if (raw_data->trxshort_data[j] != TRX_SHORT_RX_MAX) {
			input_err(true, &info->client->dev, "%s:TX Short [%d][%d]\n",
				__func__, j, raw_data->trxshort_data[j]);
		}
		if (raw_data->trxshort_data[j] < min)
			min = raw_data->trxshort_data[j];
		if (raw_data->trxshort_data[j] > max)
			max = raw_data->trxshort_data[j];
	}

	for (j = info->cap_info.y_node_num; j < info->cap_info.y_node_num*2; j++) {
		if (raw_data->trxshort_data[j] != TRX_SHORT_TX_MAX) {
			input_err(true, &info->client->dev, "%s:RX Short [%d][%d]\n",
				__func__, j, raw_data->trxshort_data[j]);
		}
		if (raw_data->trxshort_data[j] < min)
			min = raw_data->trxshort_data[j];
		if (raw_data->trxshort_data[j] > max)
			max = raw_data->trxshort_data[j];
	}

	ztm730_print_channel(info, raw_data->trxshort_data);

	snprintf(buff, sizeof(buff), "%d,%d", min, max);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

out:
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "TRX_SHORT");

	input_info(true, &client->dev, "%s: \"%s\"\n", __func__, sec->cmd_result);
}

static void run_trx_short_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	char *all_cmdbuff;
	s32 j;
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	ret = ts_set_touchmode(info, TOUCH_TRXSHORT_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}

	ret = get_raw_data(info, (u8 *)raw_data->trxshort_data, 2);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	all_cmdbuff = kzalloc(info->cap_info.y_node_num * 12, GFP_KERNEL);
	if (!all_cmdbuff) {
		ret = -ENOMEM;
		goto out;
	}

	for (j = 0; j < info->cap_info.y_node_num * 2; j++) {
		snprintf(buff, sizeof(buff), "%u,", raw_data->trxshort_data[j]);
		strcat(all_cmdbuff, buff);	
	}

	ztm730_print_channel(info, raw_data->trxshort_data);

	sec_cmd_set_cmd_result(sec, all_cmdbuff,
			strnlen(all_cmdbuff, sizeof(all_cmdbuff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	kfree(all_cmdbuff);

out:
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
}

static void run_ssr_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;
	struct tsp_raw_data *raw_data = info->raw_data;
	int total_node = info->cap_info.x_node_num + info->cap_info.y_node_num;
	int y_num = info->cap_info.y_node_num;
	char tx_buff[SEC_CMD_STR_LEN] = { 0 };
	char rx_buff[SEC_CMD_STR_LEN] = { 0 };
	u16 tx_min, tx_max, rx_min, rx_max;
	s32 j;
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(tx_buff, sizeof(tx_buff), "NG");
		sec_cmd_set_cmd_result(sec, tx_buff, strnlen(tx_buff, sizeof(tx_buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	ret = ts_set_touchmode(info, TOUCH_SSR_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->ssr_data, 1);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	tx_min = 0xFFFF;
	tx_max = 0x0000;
	rx_min = 0xFFFF;
	rx_max = 0x0000;

	input_info(true, &client->dev, "%s: SELF SATURATION start\n", __func__);
	for (j = y_num; j < total_node; j++) {
		input_info(true, &client->dev, "Tx%d: %d\n", j - y_num, raw_data->ssr_data[j]);

		if (raw_data->ssr_data[j] < tx_min && raw_data->ssr_data[j] != 0)
			tx_min = raw_data->ssr_data[j];

		if (raw_data->ssr_data[j] > tx_max)
			tx_max = raw_data->ssr_data[j];
	}

	for (j = 0; j < y_num; j++) {
		input_info(true, &client->dev, "Rx%d: %d\n", j, raw_data->ssr_data[j]);

		if (raw_data->ssr_data[j] < rx_min && raw_data->ssr_data[j] != 0)
			rx_min = raw_data->ssr_data[j];

		if (raw_data->ssr_data[j] > rx_max)
			rx_max = raw_data->ssr_data[j];
	}

	ztm730_print_channel(info, raw_data->ssr_data);

	snprintf(tx_buff, sizeof(tx_buff), "%d,%d", tx_min, tx_max);
	snprintf(rx_buff, sizeof(rx_buff), "%d,%d", rx_min, rx_max);
	sec_cmd_set_cmd_result(sec, rx_buff, strnlen(rx_buff, sizeof(rx_buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING) {
		sec_cmd_set_cmd_result_all(sec, tx_buff, strnlen(tx_buff, sizeof(tx_buff)), "SELF_SATURATION_TX");
		sec_cmd_set_cmd_result_all(sec, rx_buff, strnlen(rx_buff, sizeof(rx_buff)), "SELF_SATURATION_RX");
	}
	sec->cmd_state = SEC_CMD_STATUS_OK;

out:
	if (ret < 0) {
		snprintf(rx_buff, sizeof(rx_buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, rx_buff, strnlen(rx_buff, sizeof(rx_buff)));
		if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING) {
			sec_cmd_set_cmd_result_all(sec, rx_buff, strnlen(rx_buff, sizeof(rx_buff)), "SELF_SATURATION_TX");
			sec_cmd_set_cmd_result_all(sec, rx_buff, strnlen(rx_buff, sizeof(rx_buff)), "SELF_SATURATION_RX");
		}
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
	input_info(true, &client->dev, "%s: \"%s\"\n", __func__, sec->cmd_result);
}

static void run_self_saturation_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[16] = { 0 };
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	if (ztm730_fix_active_mode(info, true) != I2C_SUCCESS) {
		ret = -1;
		goto out;
	}

	input_info(true, &info->client->dev, "%s: start\n", __func__);

	ret = ts_set_touchmode(info, TOUCH_SSR_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->ssr_data, 1);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	ztm730_print_channel(info, raw_data->ssr_data);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

out:
	ztm730_fix_active_mode(info, false);

	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
}

static void run_self_saturation_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[16] = { 0 };
	int total_node = info->cap_info.x_node_num + info->cap_info.y_node_num;
	char *all_cmdbuff;
	s32 j;
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	if (ztm730_fix_active_mode(info, true) != I2C_SUCCESS) {
		ret = -1;
		goto out;
	}

	input_info(true, &info->client->dev, "%s: start\n", __func__);

	ret = ts_set_touchmode(info, TOUCH_SSR_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->ssr_data, 1);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	all_cmdbuff = kzalloc(total_node * 6, GFP_KERNEL);
	if (!all_cmdbuff) {
		ret = -ENOMEM;
		goto out;
	}

	for (j = 0; j < total_node; j++) {
		sprintf(buff, "%d,", raw_data->ssr_data[j]);
		strlcat(all_cmdbuff, buff, total_node * 6);
	}

	ztm730_print_channel(info, raw_data->ssr_data);

	sec_cmd_set_cmd_result(sec, all_cmdbuff,
			strnlen(all_cmdbuff, sizeof(all_cmdbuff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	kfree(all_cmdbuff);
out:
	ztm730_fix_active_mode(info, false);

	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
}

static void run_high_voltage_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;

	sec_cmd_set_default_result(sec);

	ret = ts_set_touchmode(info, TOUCH_CHARGE_PUMP_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->charge_pump_data, 1);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	snprintf(buff, sizeof(buff), "%d,%d", raw_data->charge_pump_data[0], raw_data->charge_pump_data[0]);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "HIGH_VOLTAGE");
	sec->cmd_state = SEC_CMD_STATUS_OK;

out:
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
			sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "HIGH_VOLTAGE");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
	input_raw_info(true, &client->dev, "%s: \"%s\"\n", __func__, sec->cmd_result);
}

static void run_high_voltage_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;

	sec_cmd_set_default_result(sec);

	ret = ts_set_touchmode(info, TOUCH_CHARGE_PUMP_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->charge_pump_data, 1);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	snprintf(buff, sizeof(buff), "%d", raw_data->charge_pump_data[0]);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

out:
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	input_info(true, &client->dev, "%s: \"%s\"\n", __func__, sec->cmd_result);
}

static void run_acc_jitter_test_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;

	sec_cmd_set_default_result(sec);

	input_info(true, &info->client->dev, "%s: start\n", __func__);

	ret = ts_set_touchmode(info, TOUCH_JITTER_ACCELERATION_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->acc_test_data, 2);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	ztm730_print_channel(info, raw_data->acc_test_data);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

out:
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
}

static void run_acc_jitter_test_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int total_node = info->cap_info.y_node_num * 2;
	char *all_cmdbuff;
	s32 j;
	int ret;

	sec_cmd_set_default_result(sec);

	input_info(true, &info->client->dev, "%s: start\n", __func__);

	ret = ts_set_touchmode(info, TOUCH_JITTER_ACCELERATION_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->acc_test_data, 2);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	all_cmdbuff = kzalloc(total_node * 6, GFP_KERNEL);
	if (!all_cmdbuff) {
		ret = -ENOMEM;
		goto out;
	}

	for (j = 0; j < info->cap_info.y_node_num * 2; j++) {
		sprintf(buff, "%d,", raw_data->acc_test_data[j]);
		strlcat(all_cmdbuff, buff, total_node * 6);
	}

	ztm730_print_channel(info, raw_data->acc_test_data);

	sec_cmd_set_cmd_result(sec, all_cmdbuff,
			strnlen(all_cmdbuff, sizeof(all_cmdbuff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	kfree(all_cmdbuff);

out:
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
}

static void run_interrupt_gpio_test(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int drive_value = 1;
	int irq_value = 0;
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	ret = ts_set_touchmode(info, TOUCH_NO_OPERATION_MODE);
	if (ret < 0)
		goto out;

	ztm730_enable_irq(info, false);

	ztm730_write_cmd(client, ZTM730_DRIVE_INT_STATUS_CMD);
	usleep_range(20 * 1000, 21 * 1000);

	drive_value = gpio_get_value(info->pdata->gpio_int);
	input_info(true, &client->dev, "%s: ZTM730_DRIVE_INT_STATUS_CMD: interrupt gpio: %d\n", __func__, drive_value);

	ztm730_write_cmd(client, ZTM730_CLEAR_INT_STATUS_CMD);
	usleep_range(20 * 1000, 21 * 1000);

	irq_value = gpio_get_value(info->pdata->gpio_int);
	input_info(true, &client->dev, "%s: ZTM730_CLEAR_INT_STATUS_CMD: interrupt gpio : %d\n", __func__, irq_value);

	ztm730_enable_irq(info, true);

	ret = ts_set_touchmode(info, TOUCH_POINT_MODE);
	if (ret < 0)
		goto out;

out:
	if (drive_value == 0 && irq_value == 1) {
		snprintf(buff, sizeof(buff), "%s", "0");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		if (drive_value != 0)
			snprintf(buff, sizeof(buff), "%s", "1:HIGH");
		else if (irq_value != 1)
			snprintf(buff, sizeof(buff), "%s", "1:LOW");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "INT_GPIO");

	input_info(true, &client->dev, "%s: \"%s\"(%d)\n", __func__, sec->cmd_result,
				(int)strlen(sec->cmd_result));
}

static void run_jitter_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;
	struct tsp_raw_data *raw_data = info->raw_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	u16 min, max;
	s32 i, j;
	int ret;

	sec_cmd_set_default_result(sec);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	if (ztm730_write_reg(info->client, ZTM730_JITTER_SAMPLING_CNT, 100) != I2C_SUCCESS)
		input_info(true, &client->dev, "%s: Fail to set JITTER_CNT.\n", __func__);

	ret = ts_set_touchmode(info, TOUCH_JITTER_MODE);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ret = get_raw_data(info, (u8 *)raw_data->jitter_data, 1);
	if (ret < 0) {
		ts_set_touchmode(info, TOUCH_POINT_MODE);
		goto out;
	}
	ts_set_touchmode(info, TOUCH_POINT_MODE);

	min = 0xFFFF;
	max = 0x0000;

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			if (raw_data->jitter_data[i * info->cap_info.y_node_num + j] < min &&
				raw_data->jitter_data[i * info->cap_info.y_node_num + j] != 0)
				min = raw_data->jitter_data[i * info->cap_info.y_node_num + j];

			if (raw_data->jitter_data[i * info->cap_info.y_node_num + j] > max)
				max = raw_data->jitter_data[i * info->cap_info.y_node_num + j];
		}
	}

	ztm730_print_frame(info, raw_data->jitter_data, info->cap_info.x_node_num, info->cap_info.y_node_num);

	snprintf(buff, sizeof(buff), "%d,%d", min, max);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "JITTER");
	sec->cmd_state = SEC_CMD_STATUS_OK;

out:
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
			sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "JITTER");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
	input_info(true, &client->dev, "%s: \"%s\"(%d)\n", __func__, sec->cmd_result,
				(int)strlen(sec->cmd_result));
}

int ztm730_fix_active_mode(struct ztm730_info *info, bool active)
{
	u16 temp_power_state;
	u8 ret_cnt = 0;

	input_info(true, &info->client->dev, "%s: %s\n", __func__, active ? "fix" : "release");

retry:
	ztm730_write_reg(info->client, ZTM730_POWER_STATE_FLAG, active);

	temp_power_state = 0xFFFF;
	ztm730_read_data(info->client, ZTM730_POWER_STATE_FLAG, (u8 *)&temp_power_state, 2);

	if (temp_power_state != active) {
		input_err(true, &info->client->dev, "%s: fail write 0x%x register = %d\n",
			__func__, ZTM730_POWER_STATE_FLAG, temp_power_state);
		if (ret_cnt < 3) {
			ret_cnt++;
			goto retry;
		} else {
			input_err(true, &info->client->dev, "%s: fail write 0x%x register\n",
				__func__, ZTM730_POWER_STATE_FLAG);
			return I2C_FAIL;
		}
	}

	sec_delay(5);

	return I2C_SUCCESS;
}

static void fix_active_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[16] = { 0 };
	int ret;

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		info->fix_active_mode = !!sec->cmd_param[0];
		ret = ztm730_fix_active_mode(info, info->fix_active_mode);
		if (ret != I2C_SUCCESS) {
			input_err(true, &info->client->dev, "%s: failed, retval = %d\n", __func__, ret);
			snprintf(buff, sizeof(buff), "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		} else {
			snprintf(buff, sizeof(buff), "OK");
			sec->cmd_state = SEC_CMD_STATUS_OK;
		}
	}

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %s cmd_param: %d\n", __func__, buff, sec->cmd_param[0]);
}

#ifndef CONFIG_SEC_FACTORY
static void aot_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec_cmd_set_cmd_exit(sec);
		return;
	}

	info->aot_enable = !!sec->cmd_param[0];

	mutex_lock(&info->modechange);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF && info->aot_enable == 1) {
		ztm730_ts_start(info);
		ztm730_ts_set_lowpowermode(info, TO_LOWPOWER_MODE);
	} else if (info->power_state == SEC_INPUT_STATE_LPM && info->aot_enable == 0) {
		ztm730_ts_stop(info);
	}

	mutex_unlock(&info->modechange);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %d\n", __func__, sec->cmd_param[0]);
}
#endif

static void glove_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		info->glove_mode = sec->cmd_param[0];
		if (info->glove_mode)
			zinitix_bit_set(info->optional_mode, DEF_OPTIONAL_MODE_SENSITIVE_BIT);
		else
			zinitix_bit_clr(info->optional_mode, DEF_OPTIONAL_MODE_SENSITIVE_BIT);

		if (info->work_state != PROBE)
			ztm730_set_optional_mode(info, false);

		snprintf(buff, sizeof(buff), "%s", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	}
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %d\n", __func__, sec->cmd_param[0]);
}

static void bezel_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;

	sec_cmd_set_default_result(sec);

	if (!info->pdata->support_bezel_detent ||
			sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		info->bezel_enable = sec->cmd_param[0];
		if (info->bezel_enable)
			zinitix_bit_set(info->optional_mode, DEF_OPTIONAL_MODE_WHEEL_ON_BIT);
		else
			zinitix_bit_clr(info->optional_mode, DEF_OPTIONAL_MODE_WHEEL_ON_BIT);

		ret = ztm730_set_optional_mode(info, true);
		if (ret) {
			input_err(true, &info->client->dev,
				"%s:failed ztm730_set_optional_mode\n", __func__);
			snprintf(buff, sizeof(buff), "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		} else {
			snprintf(buff, sizeof(buff), "OK");
			sec->cmd_state = SEC_CMD_STATUS_OK;
		}
	}
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %d\n", __func__, sec->cmd_param[0]);
}

static void dead_zone_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int val = sec->cmd_param[0];

	sec_cmd_set_default_result(sec);

	if (val)
		zinitix_bit_clr(info->optional_mode, DEF_OPTIONAL_MODE_EDGE_SELECT);
	else
		zinitix_bit_set(info->optional_mode, DEF_OPTIONAL_MODE_EDGE_SELECT);

	ztm730_set_optional_mode(info, false);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s cmd_param: %d\n", __func__, buff, sec->cmd_param[0]);
}

static void set_body_detection(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		if (sec->cmd_param[0])
			zinitix_bit_set(info->optional_mode, DEF_OPTIONAL_MODE_BODY_DETECTION);
		else
			zinitix_bit_clr(info->optional_mode, DEF_OPTIONAL_MODE_BODY_DETECTION);

		if (info->work_state != PROBE) {
			if (info->power_state != SEC_INPUT_STATE_POWER_OFF)
				ztm730_set_optional_mode(info, true);
		}

		snprintf(buff, sizeof(buff), "%s", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	}
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %d\n", __func__, sec->cmd_param[0]);
}

static void factory_cmd_result_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	struct i2c_client *client = info->client;

	sec->item_count = 0;
	memset(sec->cmd_result_all, 0x00, SEC_CMD_RESULT_STR_LEN);

	if (info->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &info->client->dev, "%s: [ERROR] Touch is stopped\n",
				__func__);
		sec->cmd_all_factory_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	sec->cmd_all_factory_state = SEC_CMD_STATUS_RUNNING;

	get_chip_vendor(sec);
	get_chip_name(sec);
	get_fw_ver_bin(sec);
	get_fw_ver_ic(sec);

	run_trx_short_read(sec);
	run_high_voltage_read(sec);
	run_interrupt_gpio_test(sec);
	run_jitter_read(sec);

	sec->cmd_all_factory_state = SEC_CMD_STATUS_OK;

out:
	input_info(true, &client->dev, "%s: %d%s\n", __func__, sec->item_count,
			sec->cmd_result_all);
}

static void charger_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		info->ta_connected = sec->cmd_param[0];

		if (info->ta_connected)
			zinitix_bit_set(info->optional_mode,
				DEF_OPTIONAL_MODE_USB_DETECT_BIT);
		else
			zinitix_bit_clr(info->optional_mode,
				DEF_OPTIONAL_MODE_USB_DETECT_BIT);

		ret = ztm730_set_optional_mode(info, false);
		if (ret < 0) {
			input_err(true, &info->client->dev, "%s: failed, retval = %d\n", __func__, ret);
			snprintf(buff, sizeof(buff), "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		} else {
			snprintf(buff, sizeof(buff), "OK");
			sec->cmd_state = SEC_CMD_STATUS_OK;
		}
	}

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %s cmd_param: %d\n", __func__, buff, sec->cmd_param[0]);
}

static void wireless_charger_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		info->wireless_charger_mode = sec->cmd_param[0];

		ret = ztm730_write_reg(info->client, ZTM730_WIRELESS_CHARGER_MODE, info->wireless_charger_mode);
		if (ret != I2C_SUCCESS) {
			input_err(true, &info->client->dev, "%s: failed, retval = %d\n", __func__, ret);
			snprintf(buff, sizeof(buff), "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		} else {
			snprintf(buff, sizeof(buff), "OK");
			sec->cmd_state = SEC_CMD_STATUS_OK;
		}
	}

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %s cmd_param: %d\n", __func__, buff, sec->cmd_param[0]);
}

static void rawdata_init(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret = 0;

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] == 0) {
		info->raw_mode = sec->cmd_param[0];
	} else {
		// param : 1 -> strength
		// param : 2 -> raw
		info->raw_mode = sec->cmd_param[0];
		info->raw_len = PAGE_SIZE;

		sec_input_rawdata_buffer_alloc();
	}

	if (info->power_state != SEC_INPUT_STATE_POWER_OFF) {
		if (info->raw_mode)
			ret = ztm730_write_reg(info->client, ZTM730_REPORT_RAWDATA, 0x0001);
		else
			ret = ztm730_write_reg(info->client, ZTM730_REPORT_RAWDATA, 0x0000);
		input_info(true, &info->client->dev, "%s: ZTM730_REPORT_RAWDATA[%02X]: ret: %d\n", __func__, ZTM730_REPORT_RAWDATA, ret);
	}

	snprintf(buff, sizeof(buff), "OK:%d", 3);
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
}

static void debug(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	info->debug_flag = sec->cmd_param[0];

	snprintf(buff, sizeof(buff), "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
}

static void not_support_cmd(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);
	snprintf(buff, sizeof(buff), "NA");

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
	sec_cmd_set_cmd_exit(sec);
}

static struct sec_cmd sec_cmds[] = {
	{SEC_CMD("fw_update", fw_update),},
	{SEC_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{SEC_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{SEC_CMD("get_chip_vendor", get_chip_vendor),},
	{SEC_CMD("get_chip_name", get_chip_name),},
	{SEC_CMD("check_connection", check_connection),},
	{SEC_CMD("get_x_num", get_x_num),},
	{SEC_CMD("get_y_num", get_y_num),},
	{SEC_CMD("run_dnd_read", run_dnd_read),},
	{SEC_CMD("run_dnd_read_all", run_dnd_read_all),},
	{SEC_CMD("run_dnd_v_gap_read", run_dnd_v_gap_read),},
	{SEC_CMD("run_dnd_v_gap_read_all", run_dnd_v_gap_read_all),},
	{SEC_CMD("run_dnd_h_gap_read", run_dnd_h_gap_read),},
	{SEC_CMD("run_dnd_h_gap_read_all", run_dnd_h_gap_read_all),},
	{SEC_CMD("run_trx_short_read", run_trx_short_read),},
	{SEC_CMD("run_trx_short_read_all", run_trx_short_read_all),},
	{SEC_CMD("run_ssr_read", run_ssr_read),},
	{SEC_CMD("run_self_saturation_read", run_self_saturation_read),},
	{SEC_CMD("run_self_saturation_read_all", run_self_saturation_read_all),},
	{SEC_CMD("run_high_voltage_read", run_high_voltage_read),},
	{SEC_CMD("run_high_voltage_read_all", run_high_voltage_read_all),},
	{SEC_CMD("run_acc_jitter_test_read", run_acc_jitter_test_read),},
	{SEC_CMD("run_acc_jitter_test_read_all", run_acc_jitter_test_read_all),},
	{SEC_CMD("run_interrupt_gpio_test", run_interrupt_gpio_test),},
	{SEC_CMD("run_jitter_read", run_jitter_read),},
	{SEC_CMD_H("fix_active_mode", fix_active_mode),},
#ifndef CONFIG_SEC_FACTORY
	{SEC_CMD_H("aot_enable", aot_enable),},
#endif
	{SEC_CMD_H("glove_mode", glove_mode),},
	{SEC_CMD_H("bezel_enable", bezel_enable),},
	{SEC_CMD("dead_zone_enable", dead_zone_enable),},
	{SEC_CMD("set_body_detection", set_body_detection),},
	{SEC_CMD("factory_cmd_result_all", factory_cmd_result_all),},
	{SEC_CMD_H("charger_mode", charger_mode),},
	{SEC_CMD_H("wireless_charger_mode", wireless_charger_mode),},
	{SEC_CMD_H("rawdata_init", rawdata_init),},
	{SEC_CMD("debug", debug),},
	{SEC_CMD("not_support_cmd", not_support_cmd),},
};

static void run_rawdata_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct ztm730_info *info = container_of(sec, struct ztm730_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	input_raw_info(true, &info->client->dev, "TRX short\n");
	run_trx_short_read(sec);
	run_high_voltage_read(sec);

	input_raw_info(true, &info->client->dev, "DND\n");
	run_dnd_read(sec);

	input_raw_info(true, &info->client->dev, "Self saturation TRX\n");
	run_self_saturation_read(sec);

	input_raw_info(true, &info->client->dev, "ACC Jitter\n");
	run_acc_jitter_test_read(sec);

	snprintf(buff, sizeof(buff), "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_raw_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
}

void ztm730_run_rawdata_all(struct ztm730_info *info)
{
	struct sec_cmd_data *sec = &info->sec;

	run_rawdata_read_all(sec);
}

int init_sec_factory(struct ztm730_info *info)
{
	struct tsp_raw_data *raw_data;
	int retval = 0;

	raw_data = kzalloc(sizeof(struct tsp_raw_data), GFP_KERNEL);
	if (unlikely(!raw_data)) {
		input_err(true, &info->client->dev, "%s: Failed to allocate memory\n",
				__func__);
		retval = -ENOMEM;
		goto err_alloc;
	}

	retval = sec_cmd_init_without_platdata(&info->sec, sec_cmds,
			ARRAY_SIZE(sec_cmds), SEC_CLASS_DEVT_TSP, &sysfs_attr_group);
	if (retval < 0) {
		input_err(true, &info->client->dev,
				"%s: Failed to sec_cmd_init\n", __func__);
		goto exit;
	}

	retval = sysfs_create_link(&info->sec.fac_dev->kobj,
			&info->input_dev->dev.kobj, "input");
	if (retval < 0) {
		input_err(true, &info->client->dev,
				"%s: Failed to create input symbolic link\n",
				__func__);
		goto exit;
	}

	info->raw_data = raw_data;

#ifdef ZINITIX_MISC_DEBUG
	misc_info = info;

	retval = misc_register(&touch_misc_device);
	if (retval) {
		input_err(true, &info->client->dev, "%s:Failed to register touch misc device\n", __func__);
		goto exit;
	}
#endif

	return 0;

exit:
	kfree(raw_data);
err_alloc:
	return retval;
}

void remove_sec_factory(struct ztm730_info *info)
{
	input_err(true, &info->client->dev, "%s\n", __func__);

	if (info->raw_data)
		kfree(info->raw_data);

	sysfs_remove_link(&info->sec.fac_dev->kobj, "input");

#ifdef ZINITIX_MISC_DEBUG
	misc_deregister(&touch_misc_device);
#endif
	sec_cmd_exit(&info->sec, SEC_CLASS_DEVT_TSP);
}
