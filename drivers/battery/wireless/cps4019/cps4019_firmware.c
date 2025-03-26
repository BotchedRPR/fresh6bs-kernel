/*
 *  cps4019_firmware.c
 *  Samsung CPS4019 Firmware Driver
 *
 *  Copyright (C) 2021 Samsung Electronics
 * Yeongmi Ha <yeongmi86.ha@samsung.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <linux/workqueue.h>

#include <linux/battery/sb_def.h>
#include <linux/battery/sb_notify.h>
#include <linux/battery/common/sb_event.h>
#include <linux/battery/wireless/sb_wrl_fw.h>

#include "cps4019_core.h"
#include "cps4019_firmware.h"

#define fw_log(str, ...) pr_info("[CPS4019-FW]:%s: "str, __func__, ##__VA_ARGS__)

#define FW_MODULE_NAME		"cps4019-fw"

struct sb_wrl_fw {
	struct i2c_client		*parent;
	struct device_node		*of_node;
	struct regmap			*rm;
	struct wakeup_source	*ws;
	struct delayed_work		work;
	struct mutex			mlock;
	cb_fw_result			func;

	/* status */
	int state;
	int mode;
};

struct cps_fw_data {
	struct sb_wrl_fw_data	app_bl;
	struct sb_wrl_fw_data	app;

	/* app1 wa */
	struct sb_wrl_fw_data	app1_wa;
};

#define FW_FLAG_APP_TYPE	(1 << 1)	/* 0 : APP1, 1 : APP2 */

static int regmap_write_u32(struct regmap *rm, unsigned int reg, unsigned int value)
{
	return regmap_bulk_write(rm, reg, &value, 4);
}

static int regmap_read_u32(struct regmap *rm, unsigned int reg, unsigned int *value)
{
	unsigned char buf[4];
	int ret = 0;

	ret = regmap_bulk_read(rm, reg, buf, 4);
	if (!ret)
		*value = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

	return ret;
}

/* big & little endian convert */
static int ble_convert(int data)
{
	char  *p;
	char  tmp[4];

	p = (char *)(&data);
	tmp[0] = p[3];
	tmp[1] = p[2];
	tmp[2] = p[1];
	tmp[3] = p[0];
	return *(int *)(tmp);
}

#define CMD_RUNNING		0x66
#define CMD_PASS		0x55
#define CMD_FAIL		0xAA
#define CMD_ILLEGAL		0x40
static int cmd_wait(struct regmap *rm)
{
	int ret = 0, cnt = 0, value = CMD_RUNNING;

	while ((value == CMD_RUNNING) &&
			(cnt <= 3000)) {
		usleep_range(5000, 6000);

		ret = regmap_read_u32(rm, 0x200005F8, &value);
		if (ret < 0) {
			cnt += 1000;
		} else {
			cnt++;
			value = ble_convert(value);
		}
	}

	fw_log("ret = %d, cnt = %d, value = 0x%x\n", ret, cnt, value);
	return value;
}

#if IS_ENABLED(CONFIG_ENG_BATTERY_CONCEPT)
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
static int get_fw_data(const char *path, struct sb_wrl_fw_data *fw_data)
{
	mm_segment_t old_fs;
	struct file *fp;
	long nread;
	int ret = 0;

	old_fs = force_uaccess_begin();
	fp = filp_open(path, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		ret = PTR_ERR(fp);
		goto err_open;
	}

	fw_data->size = fp->f_path.dentry->d_inode->i_size;
	if (fw_data->size <= 0) {
		ret = -ENODEV;
		goto err_open;
	}

	fw_data->data = kmalloc(fw_data->size, GFP_KERNEL);
	if (!fw_data->data) {
		ret = -ENOMEM;
		goto err_open;
	}

	nread = kernel_read(fp, fw_data->data, fw_data->size, &fp->f_pos);
	if (nread != fw_data->size) {
		ret = -EIO;
		goto err_read;
	}
	fw_log("path = %s, size = %ld\n", path, fw_data->size);

	filp_close(fp, current->files);
	force_uaccess_end(old_fs);

	return 0;

err_read:
	kfree(fw_data->data);
	fw_data->data = NULL;
err_open:
	fw_data->size = 0;
	force_uaccess_end(old_fs);
	return ret;
}
#else
static int get_fw_data(const char *path, struct sb_wrl_fw_data *fw_data)
{
	return -ESRCH;
}
#endif

static void init_fw_data(struct sb_wrl_fw_data *fw_data)
{
	kfree(fw_data->data);

	fw_data->data = NULL;
	fw_data->size = 0;
}

static void init_cps_fw_data(struct cps_fw_data *cps_data)
{
	init_fw_data(&cps_data->app_bl);
	init_fw_data(&cps_data->app);
	init_fw_data(&cps_data->app1_wa);
}

static int app_program(struct sb_wrl_fw *fw, struct sb_wrl_fw_data *app, int fw_flag)
{
	int i, cfg_buf_size = 64, ret;
	int *p;

	fw_log("load firmware to MTP\n");
	p = (int *)app->data;

	for (i = 0; i < (app->size / 4); i++)
		p[i] = ble_convert(p[i]);

	ret = regmap_write_u32(fw->rm, 0x200005F4, ble_convert(cfg_buf_size));
	fw_log("write buf size (%d)\n", ret);

	if (!(fw_flag & FW_FLAG_APP_TYPE)) {
		ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x000000C0));
		fw_log("write app1 program start addr (%d)\n", ret);
	} else {
		ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x000000D0));
		fw_log("write app2 program start addr (%d)\n", ret);
	}
	ret = cmd_wait(fw->rm);
	if (ret != CMD_PASS) {
		fw_log("write app program start addr fail\n");
		return SB_WRL_FW_RESULT_FAIL;
	}

	ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x00000060));
	fw_log("cmd eraser_0 (%d)\n", ret);
	ret = cmd_wait(fw->rm);
	if (ret != CMD_PASS) {
		fw_log("erase fail\n");
		return SB_WRL_FW_RESULT_FAIL;
	}

	for (i = 0; i < (app->size / 4 / cfg_buf_size); i++) {
		unsigned int reg = 0x20000600, cmd_value = 0x00000010;

		if (i & 0x1) {
			reg = 0x20000700;
			cmd_value = 0x00000020;
		}

		ret = regmap_bulk_write(fw->rm, reg, p, cfg_buf_size * 4);
		fw_log("bulk write - idx = %d, reg = 0x%x, reg = %d\n", i, reg, ret);
		p = p + cfg_buf_size;

		ret = cmd_wait(fw->rm);
		if (ret != CMD_PASS) {
			fw_log("write %s data to MTP fail\n", (i & 0x1) ? "buffer 1" : "buffer 0");
			return SB_WRL_FW_RESULT_FAIL;
		}

		ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(cmd_value));
		fw_log("cmd buffer (%d)\n", ret);
	}

	ret = cmd_wait(fw->rm);
	if (ret != CMD_PASS) {
		fw_log("write %s data to MTP fail\n", (i & 0x1) ? "buffer 1" : "buffer 0");
		return SB_WRL_FW_RESULT_FAIL;
	}

	return SB_WRL_FW_RESULT_PASS;
}

static int app1_crc_check(struct sb_wrl_fw *fw, struct sb_wrl_fw_data *app1_wa, int fw_flag)
{
	int ret = 0, retry_cnt = 0;

	if (!(fw_flag & FW_FLAG_APP_TYPE)) {
		fw_log("app1 crc check skip\n");
		return SB_WRL_FW_RESULT_PASS;
	}

	fw_log("app1 crc check\n");

	ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x000000A0));
	fw_log("cmd crc_app1 (%d)\n", ret);
	ret = cmd_wait(fw->rm);
	if (ret != CMD_PASS) {
		fw_log("app1 crc fail\n");
		goto program_app1;
	}

	ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x000000E0));
	fw_log("refresh app1 (%d)\n", ret);
	ret = cmd_wait(fw->rm);
	if (ret != CMD_PASS) {
		fw_log("refresh app1 fail\n");
		//goto program_app1;
	}

	ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x000000A0));
	fw_log("cmd crc_app1 again (%d)\n", ret);
	ret = cmd_wait(fw->rm);
	if (ret != CMD_PASS) {
		fw_log("app1 crc fail again\n");
		goto program_app1;
	}

	return SB_WRL_FW_RESULT_PASS;

program_app1:
	for (; retry_cnt < 10; retry_cnt++) {
		int program_cnt = 0;

		do {
			ret = app_program(fw, app1_wa, 0);
			fw_log("program app1 - ret(%d), cnt(%d)\n", ret, program_cnt);
			if ((++program_cnt) > 3)
				break;
		} while (ret != SB_WRL_FW_RESULT_PASS);

		ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x000000A0));
		fw_log("cmd crc_app1 (%d)\n", ret);
		ret = cmd_wait(fw->rm);
		if (ret == CMD_PASS)
			break;

		fw_log("app1 crc fail - retry_cnt(%d)\n", retry_cnt);
	}

	return (ret == CMD_PASS) ?
		SB_WRL_FW_RESULT_PASS : SB_WRL_FW_RESULT_FAIL;
}

static int run_fw(struct sb_wrl_fw *fw, struct cps_fw_data *cps_data, unsigned int fw_flag)
{
	int ret;

	fw_log("load to sram\n");
	ret = regmap_write_u32(fw->rm, 0xFFFFFF00, ble_convert(0x0E000000));
	fw_log("enable 32bit i2c (%d)\n", ret);
	ret = regmap_write_u32(fw->rm, 0x40040070, ble_convert(0x0000A061));
	fw_log("write password (%d)\n", ret);
	ret = regmap_write_u32(fw->rm, 0x40040004, ble_convert(0x00000008));
	fw_log("reset and halt mcu (%d)\n", ret);

	ret = regmap_bulk_write(fw->rm, 0x20000000, cps_data->app_bl.data, cps_data->app_bl.size);
	fw_log("loaded sram hex (%d)\n", ret);

	ret = regmap_write_u32(fw->rm, 0x40040010, ble_convert(0x00000001));
	fw_log("triming load function is disabled (%d)\n", ret);
	ret = regmap_write_u32(fw->rm, 0x40040004, ble_convert(0x00000066));
	fw_log("enable remap function and reset the mcu (%d)\n", ret);

	usleep_range(10000, 11000);

	ret = regmap_write_u32(fw->rm, 0xFFFFFF00, ble_convert(0x0E000000));
	fw_log("enable 32bit i2c (%d)\n", ret);

	usleep_range(10000, 11000);

	fw_log("bootloader crc check\n");
	ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x000000B0));
	fw_log("cmd crc_test (%d)\n", ret);
	ret = cmd_wait(fw->rm);
	if (ret != CMD_PASS) {
		fw_log("bootloader crc fail\n");
		return SB_WRL_FW_RESULT_FAIL;
	}

	ret = app1_crc_check(fw, &cps_data->app1_wa, fw_flag);
	if (ret != SB_WRL_FW_RESULT_PASS)
		return ret;

	ret = app_program(fw, &cps_data->app, fw_flag);
	if (ret != SB_WRL_FW_RESULT_PASS)
		return ret;

	fw_log("check app CRC\n");
	ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x00000090));
	fw_log("cmd crc_app (%d)\n", ret);
	ret = cmd_wait(fw->rm);
	if (ret != CMD_PASS) {
		fw_log("app crc fail\n");
		return SB_WRL_FW_RESULT_FAIL;
	}

	fw_log("write mcu start flag\n");
	ret = regmap_write_u32(fw->rm, 0x200005FC, ble_convert(0x00000080));
	fw_log("cmd wr_flag (%d)\n", ret);
	ret = cmd_wait(fw->rm);
	if (ret != CMD_PASS) {
		fw_log("write mcu start flag fail\n");
		return SB_WRL_FW_RESULT_FAIL;
	}

	ret = regmap_write_u32(fw->rm, 0x40040004, ble_convert(0x00000001));
	fw_log("reset all system (%d)\n", ret);

	msleep(1000);

{
	u16 chip_id = 0;

	fw_log("check chip id\n");
	ret = cps4019_reg_bulk_read(fw->parent, 0x0000, (u8 *)&chip_id, 2);
	fw_log("read chip_id (ret = %d, id = 0x%x)\n", ret, chip_id);
	if (chip_id != 0x4019) {
		fw_log("check chip_id fail\n");
		return SB_WRL_FW_RESULT_FAIL;
	}
}
	return SB_WRL_FW_RESULT_PASS;
}

static int copy_fw_data_by_dt(struct device_node *np, const char *name, struct sb_wrl_fw_data *app)
{
	int len, ret;

	len = of_property_count_u8_elems(np, name);
	if (len <= 0)
		return -ENODEV;

	fw_log("%s get len = %d\n", name, len);
	app->data = kmalloc(len, GFP_KERNEL);
	if (!app->data)
		return -ENOMEM;
	app->size = len;

	ret = of_property_read_u8_array(np, name, app->data, app->size);
	if (ret < 0) {
		kfree(app->data);
		app->size = 0;
		return ret;
	}
	fw_log("%s finish\n", name);

	return 0;
}

static int copy_fw_data(struct sb_wrl_fw_data *app, unsigned char *data, unsigned int size)
{
	app->data = kmalloc(size, GFP_KERNEL);
	if (!app->data)
		return -ENOMEM;

	app->size = size;
	memcpy(app->data, data, size);

	return 0;
}

static int load_fw_by_dt(struct device_node *np, struct cps_fw_data *cps_data)
{
	int ret = 0;

	ret = copy_fw_data_by_dt(np, "app_bl", &cps_data->app_bl);
	if (ret) {
		fw_log("failed to get app_bl - dt(ret = %d)\n", ret);
		goto err_copy;
	}

	ret = copy_fw_data_by_dt(np, "app", &cps_data->app);
	if (ret) {
		fw_log("failed to get app - dt(ret = %d)\n", ret);
		goto err_copy;
	}

	ret = copy_fw_data_by_dt(np, "app1_wa", &cps_data->app1_wa);
	if (ret) {
		fw_log("failed to get app1_wa - dt(ret = %d)\n", ret);
		goto err_copy;
	}

	return 0;

err_copy:
	init_cps_fw_data(cps_data);
	return ret;
}

static int load_fw_by_bin(struct cps_fw_data *cps_data)
{
	int ret = 0;

	ret = copy_fw_data(&cps_data->app_bl, CPS4019_APP2_BL, CPS4019_APP2_BL_SIZE);
	if (ret) {
		fw_log("failed to copy app_bl - ret(%d)\n", ret);
		goto err_copy;
	}

	ret = copy_fw_data(&cps_data->app, CPS4019_APP2, CPS4019_APP2_SIZE);
	if (ret) {
		fw_log("failed to copy app - ret(%d)\n", ret);
		goto err_copy;
	}

	ret = copy_fw_data(&cps_data->app1_wa, CPS4019_APP1, CPS4019_APP1_SIZE);
	if (ret) {
		fw_log("failed to copy app1_wa - ret(%d)\n", ret);
		goto err_copy;
	}

	return 0;

err_copy:
	init_cps_fw_data(cps_data);
	return ret;
}

static int load_fw_by_sdcard(const char *app_bl_path, const char *app_path, const char *app1_wa_path,
								struct cps_fw_data *cps_data)
{
	int ret = 0;

	ret = get_fw_data(app_bl_path, &cps_data->app_bl);
	if (ret < 0)
		goto err_get;

	ret = get_fw_data(app_path, &cps_data->app);
	if (ret < 0)
		goto err_get;

	if (app1_wa_path) {
		ret = get_fw_data(app1_wa_path, &cps_data->app1_wa);
		if (ret < 0)
			goto err_get;
	}

	return 0;

err_get:
	init_cps_fw_data(cps_data);
	return ret;
}

static void cb_work(struct work_struct *work)
{
	struct sb_wrl_fw *fw = container_of(work, struct sb_wrl_fw, work.work);
	struct cps_fw_data cps_data;
	int state = SB_WRL_FW_RESULT_FAIL;

	__pm_stay_awake(fw->ws);
	fw_log("start\n");

	mutex_lock(&fw->mlock);
	fw->state = SB_WRL_FW_RESULT_RUNNING;
	sb_notify_call_bit(SB_NOTIFY_EVENT_MISC,
		BATT_MISC_EVENT_WIRELESS_FW_UPDATE, BATT_MISC_EVENT_WIRELESS_FW_UPDATE);
	mutex_unlock(&fw->mlock);

	memset(&cps_data, 0, sizeof(struct cps_fw_data));
	fw_log("run fw - mode = %d\n", fw->mode);
	switch (fw->mode) {
	case SB_WRL_RX_BUILT_IN_MODE:
		/* try to update app2 only */
		if (fw->of_node) {
			if (load_fw_by_dt(fw->of_node, &cps_data))
				break;
		} else {
			if (load_fw_by_bin(&cps_data))
				break;
		}
		state = run_fw(fw, &cps_data, FW_FLAG_APP_TYPE);
		break;
	case SB_WRL_RX_SDCARD_MODE:
		/* try to update app1 */
		if (!load_fw_by_sdcard(CPS4019_APP1_BL_PATH, CPS4019_APP1_PATH, NULL, &cps_data))
			state = run_fw(fw, &cps_data, 0);
		init_cps_fw_data(&cps_data);

		/* try to update app2 */
		if (!load_fw_by_sdcard(CPS4019_APP2_BL_PATH, CPS4019_APP2_PATH, CPS4019_APP1_PATH, &cps_data))
			state = run_fw(fw, &cps_data, FW_FLAG_APP_TYPE);
		break;
	default:
		state = SB_WRL_FW_RESULT_PASS;
		break;
	}
	init_cps_fw_data(&cps_data);

	fw_log("%s firmware update\n",
		(state == SB_WRL_FW_RESULT_PASS) ? "success" : "fail");

	if (fw->func)
		fw->func(fw->parent, state);

	mutex_lock(&fw->mlock);
	fw->state = state;
	sb_notify_call_bit(SB_NOTIFY_EVENT_MISC, 0, BATT_MISC_EVENT_WIRELESS_FW_UPDATE);
	mutex_unlock(&fw->mlock);

	__pm_relax(fw->ws);
}

static const struct regmap_config rm_cfg = {
	.reg_bits = 32,
	.val_bits = 8,
};

struct sb_wrl_fw *sb_wrl_fw_register(struct i2c_client *i2c, cb_fw_result func)
{
	struct sb_wrl_fw *fw = NULL;
	struct regmap *rm = NULL;

	if (IS_ERR_OR_NULL(i2c))
		return ERR_PTR(-EINVAL);

	rm = devm_regmap_init_i2c(i2c, &rm_cfg);
	if (IS_ERR(rm))
		return ERR_PTR(PTR_ERR(rm));

	fw = kzalloc(sizeof(struct sb_wrl_fw), GFP_KERNEL);
	if (!fw)
		return ERR_PTR(-ENOMEM);

	fw->parent = i2c;
	fw->func = func;
	fw->rm = rm;
	fw->ws = wakeup_source_register(&i2c->dev, FW_MODULE_NAME);
	INIT_DELAYED_WORK(&fw->work, cb_work);
	mutex_init(&fw->mlock);
	fw->of_node = of_find_node_by_name(NULL, SB_WRL_FW_NAME);

	fw->state = SB_WRL_FW_RESULT_INIT;
	fw->mode = SB_WRL_RX_BUILT_IN_MODE;

	fw_log("finished!!\n");
	return fw;
}

int sb_wrl_fw_get_version(struct sb_wrl_fw *fw, int *version)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(fw) || (version == NULL))
		return -EINVAL;

	/* check dt version */
	if (fw->of_node)
		ret = of_property_read_u32(fw->of_node, SB_WRL_FW_VER, (unsigned int *)version);
	else
		*version = CPS4019_APP2_VERSION;

	return ret;
}

int sb_wrl_fw_get_state(struct sb_wrl_fw *fw, int *state)
{
	if (IS_ERR_OR_NULL(fw) || (state == NULL))
		return -EINVAL;

	mutex_lock(&fw->mlock);
	*state = fw->state;
	mutex_unlock(&fw->mlock);
	fw_log("state = %s\n", get_fw_result_str(fw->state));
	return 0;
}

int sb_wrl_fw_update(struct sb_wrl_fw *fw, int mode)
{
	unsigned int work_state;

	if (IS_ERR_OR_NULL(fw))
		return -EINVAL;

	if ((mode != SB_WRL_RX_SDCARD_MODE) &&
		(mode != SB_WRL_RX_BUILT_IN_MODE))
		return -EINVAL;

	work_state = work_busy(&fw->work.work);
	fw_log("work_state = 0x%x, fw_state = %d\n", work_state, fw->state);
	if (work_state & (WORK_BUSY_PENDING | WORK_BUSY_RUNNING))
		return -EBUSY;

	mutex_lock(&fw->mlock);
	fw->state = SB_WRL_FW_RESULT_INIT;
	fw->mode = mode;
	schedule_delayed_work(&fw->work, 0);
	mutex_unlock(&fw->mlock);

	return 0;
}
