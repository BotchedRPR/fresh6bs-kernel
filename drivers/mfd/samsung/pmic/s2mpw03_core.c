/*
 * s2mpw03.c - mfd core driver for the s2mpw03
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/samsung/pmic/s2mpw03.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#if IS_ENABLED(CONFIG_EXYNOS_ACPM)
#include <soc/samsung/acpm_mfd.h>
#define MAIN_CHANNEL	0
#endif
#include <soc/samsung/exynos-pmu-if.h>
#include <linux/wakeup_reason.h>
#include <linux/debugfs.h>

#if IS_ENABLED(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#define I2C_ADDR_TOP	0x00
#define I2C_ADDR_PMIC	0x01
#define I2C_ADDR_RTC	0x02
#define I2C_ADDR_CODEC 	0x03
#define I2C_ADDR_CHG	0x04
#define I2C_ADDR_FG	0x05
#define I2C_ADDR_CLOSE	0x0F

#define I2C_RETRY_CNT	3

#if IS_ENABLED(CONFIG_DEBUG_FS)
static u8 i2caddr;
static u8 i2cdata;
static u8 i2cslave;
static struct i2c_client *dbgi2c;
static struct dentry *s2mpw03_root;
static struct dentry *s2mpw03_i2caddr;
static struct dentry *s2mpw03_i2cdata;
static struct s2mpw03_dev *dbg_s2mpw03;
#endif

struct device_node *acpm_mfd_node;

static struct mfd_cell s2mpw03_devs[] = {
	{ .name = "s2mpw03-regulator", },
	{ .name = "s2mpw03-rtc", },
#if IS_ENABLED(CONFIG_KEYBOARD_S2MPW03)
	{ .name = "s2mpw03-power-keys", },
#endif
#if IS_ENABLED(CONFIG_CHARGER_S2MPW03)
	{ .name = "s2mpw03-charger", },
#endif
#if IS_ENABLED(CONFIG_FUELGAUGE_S2MPW03)
	{ .name = "s2mpw03-fuelgauge", },
#endif
#if IS_ENABLED(CONFIG_BATTERY_S2MU00X)
	{ .name = "s2mu00x-battery", },
#endif
};

#if IS_ENABLED(CONFIG_EXYNOS_ACPM)
static DEFINE_MUTEX(ext_bus_lock);

void s2mpw03_ext_bus_set(bool flag)
{
	if (flag)
		mutex_lock(&ext_bus_lock);
	else
		mutex_unlock(&ext_bus_lock);
}
EXPORT_SYMBOL_GPL(s2mpw03_ext_bus_set);

int s2mpw03_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = exynos_acpm_read_reg(acpm_mfd_node, MAIN_CHANNEL, i2c->addr, reg, dest);
		if (!ret)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(s2mpw03_read_reg);

int s2mpw03_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = exynos_acpm_bulk_read(acpm_mfd_node, MAIN_CHANNEL, i2c->addr, reg, count, buf);
		if (!ret)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(s2mpw03_bulk_read);

int s2mpw03_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = exynos_acpm_write_reg(acpm_mfd_node, MAIN_CHANNEL, i2c->addr, reg, value);
		if (!ret)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpw03_write_reg);

int s2mpw03_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = exynos_acpm_bulk_write(acpm_mfd_node, MAIN_CHANNEL, i2c->addr, reg, count, buf);
		if (!ret)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpw03_bulk_write);

int s2mpw03_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = exynos_acpm_update_reg(acpm_mfd_node, MAIN_CHANNEL, i2c->addr, reg, val, mask);
		if (!ret)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpw03_update_reg);
#else
int s2mpw03_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret >= 0)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret < 0) {
		pr_info("%s:%s reg(0x%x), ret(%d)\n",
			 MFD_DEV_NAME, __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}
EXPORT_SYMBOL_GPL(s2mpw03_read_reg);

int s2mpw03_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
		if (ret >= 0)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpw03_bulk_read);

int s2mpw03_read_word(struct i2c_client *i2c, u8 reg)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = i2c_smbus_read_word_data(i2c, reg);
		if (ret >= 0)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}	
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret < 0)
		return ret;

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpw03_read_word);

int s2mpw03_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = i2c_smbus_write_byte_data(i2c, reg, value);
		if (ret >= 0)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}	
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret < 0)
		pr_info("%s:%s reg(0x%x), ret(%d)\n",
				MFD_DEV_NAME, __func__, reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(s2mpw03_write_reg);

int s2mpw03_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
		if (ret >= 0)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}	
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(s2mpw03_bulk_write);

int s2mpw03_write_word(struct i2c_client *i2c, u8 reg, u16 value)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = i2c_smbus_write_word_data(i2c, reg, value);
		if (ret >= 0)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}	
	mutex_unlock(&s2mpw03->i2c_lock);
	if (ret < 0)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(s2mpw03_write_word);

int s2mpw03_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
	int ret, i;
	u8 old_val, new_val;

	mutex_lock(&s2mpw03->i2c_lock);
	for (i = 0; i < I2C_RETRY_CNT; ++i) {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret >= 0)
			break;
		pr_info("%s reg(0x%x), ret(%d), i2c_retry_cnt(%d/%d)\n",
			__func__, reg, ret, i + 1, I2C_RETRY_CNT);
	}
	if (ret >= 0) {
		old_val = ret & 0xff;
		new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&s2mpw03->i2c_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mpw03_update_reg);

#endif

#if IS_ENABLED(CONFIG_OF)
static int of_s2mpw03_dt(struct device *dev,
			 struct s2mpw03_platform_data *pdata,
			 struct s2mpw03_dev *s2mpw03)
{
	struct device_node *np = dev->of_node;
	struct device_node *codec_np;
	struct i2c_client *i2c_codec;
	int ret, strlen;
	const char *status;
	u32 val;

	if (!np)
		return -EINVAL;

	acpm_mfd_node = np;

	status = of_get_property(np, "s2mpw03,wakeup", &strlen);
	if (status == NULL)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wakeup = true;
		else
			pdata->wakeup = false;
	}

	codec_np = of_parse_phandle(np, "samsung,codec-interrupt", 0);
	if(codec_np) {
		i2c_codec = of_find_i2c_device_by_node(codec_np);
		s2mpw03->codec = i2c_codec;
		pr_err("[DEBUG] %s: codec node exist, name: %s\n", __func__, i2c_codec->name);
	}

	/* WTSR, SMPL */
	pdata->wtsr_smpl = devm_kzalloc(dev, sizeof(*pdata->wtsr_smpl),
			GFP_KERNEL);
	if (!pdata->wtsr_smpl)
		return -ENOMEM;

	status = of_get_property(np, "wtsr_en", &strlen);
	if (status == NULL)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wtsr_smpl->wtsr_en = true;
		else
			pdata->wtsr_smpl->wtsr_en = false;
	}

	status = of_get_property(np, "smpl_en", &strlen);
	if (status == NULL)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wtsr_smpl->smpl_en = true;
		else
			pdata->wtsr_smpl->smpl_en = false;
	}

	ret = of_property_read_u32(np, "wtsr_timer_val",
			&pdata->wtsr_smpl->wtsr_timer_val);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "smpl_timer_val",
			&pdata->wtsr_smpl->smpl_timer_val);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "check_jigon", &val);
	if (ret)
		return -EINVAL;
	pdata->wtsr_smpl->check_jigon = !!val;

	/* init time */
	pdata->init_time = devm_kzalloc(dev, sizeof(*pdata->init_time),
			GFP_KERNEL);
	if (!pdata->init_time)
		return -ENOMEM;

/*	ret = of_property_read_u32(np, "init_time,msec",
			&pdata->init_time->tm_msec);
	if (ret)
		return -EINVAL;
*/
	ret = of_property_read_u32(np, "init_time,sec",
			&pdata->init_time->tm_sec);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,min",
			&pdata->init_time->tm_min);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,hour",
			&pdata->init_time->tm_hour);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,mday",
			&pdata->init_time->tm_mday);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,mon",
			&pdata->init_time->tm_mon);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,year",
			&pdata->init_time->tm_year);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,wday",
			&pdata->init_time->tm_wday);
	if (ret)
		return -EINVAL;

	/* rtc optimize */
	ret = of_property_read_u32(np, "osc-bias-up", &val);
	if (!ret)
		pdata->osc_bias_up = val;
	else
		pdata->osc_bias_up = -1;

	ret = of_property_read_u32(np, "rtc_cap_sel", &val);
	if (!ret)
		pdata->cap_sel = val;
	else
		pdata->cap_sel = -1;

	ret = of_property_read_u32(np, "rtc_osc_xin", &val);
	if (!ret)
		pdata->osc_xin = val;
	else
		pdata->osc_xin = -1;

	ret = of_property_read_u32(np, "rtc_osc_xout", &val);
	if (!ret)
		pdata->osc_xout = val;
	else
		pdata->osc_xout = -1;

	ret = of_property_read_u32(np, "ldo6_registance", &val);
	if (!ret)
		pdata->ldo6_reg = val;
	else
		pdata->ldo6_reg = -1;

	ret = of_property_read_u32(np, "phase", &val);
	if (!ret)
		pdata->phase = val;
	else
		pdata->phase = -1;

	return 0;
}
#else
static int of_s2mpw03_dt(struct device *dev,
			 struct max77834_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_DEBUG_FS)
static ssize_t s2mpw03_i2caddr_read(struct file *file, char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[10];
	ssize_t ret;

	ret = snprintf(buf, sizeof(buf), "0x%x 0x%x\n",i2cslave, i2caddr);
	if (ret < 0)
		return ret;

	return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

static ssize_t s2mpw03_i2caddr_write(struct file *file, const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[20];
	ssize_t len, argc;
	char addr1[10], addr2[10];
	u8 val;

	len = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (len < 0)
		return len;

	buf[len] = '\0';
	argc = sscanf(buf, "%4s %4s", addr1, addr2);
	if (argc < 2)
		return -EINVAL;

	if (!kstrtou8(addr1, 0, &val))
		i2cslave = val;

	if (!kstrtou8(addr2, 0, &val))
		i2caddr = val;

	switch(i2cslave) {
	case I2C_ADDR_TOP:
		dbgi2c = dbg_s2mpw03->i2c;
		break;
	case I2C_ADDR_PMIC:
		dbgi2c = dbg_s2mpw03->pmic;
		break;
	case I2C_ADDR_RTC:
		dbgi2c = dbg_s2mpw03->rtc;
		break;
	case I2C_ADDR_CODEC:
		break;
	case I2C_ADDR_CHG:
		dbgi2c = dbg_s2mpw03->charger;
		break;
	case I2C_ADDR_FG:
		dbgi2c = dbg_s2mpw03->fuelgauge;
		break;
	case I2C_ADDR_CLOSE:
		dbgi2c = dbg_s2mpw03->close;
		break;
	default:
		break;
	}

	return len;
}

static ssize_t s2mpw03_i2cdata_read(struct file *file, char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[10];
	ssize_t ret;

	ret = s2mpw03_read_reg(dbgi2c, i2caddr, &i2cdata);
	if (ret)
		return ret;

	ret = snprintf(buf, sizeof(buf), "0x%x\n", i2cdata);
	if (ret < 0)
		return ret;

	return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

static ssize_t s2mpw03_i2cdata_write(struct file *file, const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[30];
	ssize_t len, ret, argc;
	char addr1[10], addr2[10], data[10];
	u8 val;

	len = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (len < 0)
		return len;

	buf[len] = '\0';

	argc = sscanf(buf, "%4s %4s %4s", addr1, addr2, data);
	if (argc < 3)
		return -EINVAL;

	if (!kstrtou8(addr1, 0, &val))
		i2cslave = val;

	switch(i2cslave) {
	case I2C_ADDR_TOP:
		dbgi2c = dbg_s2mpw03->i2c;
		break;
	case I2C_ADDR_PMIC:
		dbgi2c = dbg_s2mpw03->pmic;
		break;
	case I2C_ADDR_RTC:
		dbgi2c = dbg_s2mpw03->rtc;
		break;
	case I2C_ADDR_CODEC:
		break;
	case I2C_ADDR_CHG:
		dbgi2c = dbg_s2mpw03->charger;
		break;
	case I2C_ADDR_FG:
		dbgi2c = dbg_s2mpw03->fuelgauge;
		break;
	case I2C_ADDR_CLOSE:
		dbgi2c = dbg_s2mpw03->close;
		break;
	default:
		break;
	}

	if (!kstrtou8(addr2, 0, &val))
		i2caddr = val;

	if (!kstrtou8(data, 0, &val)) {
		ret = s2mpw03_write_reg(dbgi2c, i2caddr, val);
		if (ret < 0)
			return ret;
	}
	return len;
}

static const struct file_operations s2mpw03_i2caddr_fops = {
	.open = simple_open,
	.read = s2mpw03_i2caddr_read,
	.write = s2mpw03_i2caddr_write,
	.llseek = default_llseek,
};
static const struct file_operations s2mpw03_i2cdata_fops = {
	.open = simple_open,
	.read = s2mpw03_i2cdata_read,
	.write = s2mpw03_i2cdata_write,
	.llseek = default_llseek,
};
#endif

static u8 s2mpw03_get_evt_version(u8 id)
{
	u8 evt_ver = (id & PMIC_HW_REV_ID_MASK) >> PMIC_HW_REV_ID_SHIFT;
	return evt_ver;
}

static u8 s2mpw03_get_otp_version(u8 id)
{
	u8 otp_ver = id & PMIC_OTP_VER_MASK;
	return otp_ver;
}

static int s2mpw03_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *dev_id)
{
	struct s2mpw03_dev *s2mpw03;
	struct s2mpw03_platform_data *pdata = i2c->dev.platform_data;

	u8 reg_data;
	int ret = 0;

	pr_info("[PMIC] %s: start\n", __func__);

	s2mpw03 = devm_kzalloc(&i2c->dev, sizeof(struct s2mpw03_dev), GFP_KERNEL);
	if (!s2mpw03)
		return -ENOMEM;

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev,
			 sizeof(struct s2mpw03_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c->dev, "Failed to allocate memory\n");
			ret = -ENOMEM;
			goto err;
		}

		ret = of_s2mpw03_dt(&i2c->dev, pdata, s2mpw03);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to get device of_node\n");
			goto err;
		}

		i2c->dev.platform_data = pdata;
	} else
		pdata = i2c->dev.platform_data;


	s2mpw03->dev = &i2c->dev;
	i2c->addr = I2C_ADDR_TOP;	/* forced COMMON address For GKI-R */
	s2mpw03->i2c = i2c;
	s2mpw03->irq = i2c->irq;
	s2mpw03->device_type = S2MPW03X;

	if (pdata) {
		s2mpw03->pdata = pdata;

		pdata->irq_base = devm_irq_alloc_descs(s2mpw03->dev, -1, 0, S2MPW03_IRQ_NR, -1);
		if (pdata->irq_base < 0) {
			pr_err("%s:%s irq_alloc_descs Fail! ret(%d)\n",
				MFD_DEV_NAME, __func__, pdata->irq_base);
			ret = -EINVAL;
			goto err;
		} else
			s2mpw03->irq_base = pdata->irq_base;

		s2mpw03->wakeup = pdata->wakeup;
		s2mpw03->ldo6_reg = pdata->ldo6_reg;
		s2mpw03->phase = pdata->phase;
	} else {
		ret = -EINVAL;
		goto err;
	}
	mutex_init(&s2mpw03->i2c_lock);

	i2c_set_clientdata(i2c, s2mpw03);

	/* TODO */
	if (s2mpw03_read_reg(i2c, S2MPW03_PMIC_REG_PMICID, &reg_data) < 0) {
		dev_err(s2mpw03->dev,
			"device not found on this channel (this is not an error)\n");
		ret = -ENODEV;
		goto err_w_lock;
	} else {
		/* print hw_rev and otp_ver */
		s2mpw03->pmic_rev = s2mpw03_get_evt_version(reg_data);
		s2mpw03->pmic_ver = s2mpw03_get_otp_version(reg_data);
		pr_info("%s: pmic_hw_rev: 0x%02hhx, pmic_otp_ver: 0x%02hhx\n", __func__,
			s2mpw03->pmic_rev, s2mpw03->pmic_ver);
	}

	s2mpw03->pmic = devm_i2c_new_dummy_device(s2mpw03->dev, s2mpw03->i2c->adapter, I2C_ADDR_PMIC);
	s2mpw03->rtc = devm_i2c_new_dummy_device(s2mpw03->dev, s2mpw03->i2c->adapter, I2C_ADDR_RTC);
	s2mpw03->charger = devm_i2c_new_dummy_device(s2mpw03->dev, s2mpw03->i2c->adapter, I2C_ADDR_CHG);
	s2mpw03->fuelgauge = devm_i2c_new_dummy_device(s2mpw03->dev, s2mpw03->i2c->adapter, I2C_ADDR_FG);
	s2mpw03->close = devm_i2c_new_dummy_device(s2mpw03->dev, s2mpw03->i2c->adapter, I2C_ADDR_CLOSE);

	i2c_set_clientdata(s2mpw03->pmic, s2mpw03);
	i2c_set_clientdata(s2mpw03->rtc, s2mpw03);
	i2c_set_clientdata(s2mpw03->charger, s2mpw03);
	i2c_set_clientdata(s2mpw03->fuelgauge, s2mpw03);
	i2c_set_clientdata(s2mpw03->close, s2mpw03);

	ret = s2mpw03_irq_init(s2mpw03);
	if (ret < 0)
		goto err_irq_init;

	ret = mfd_add_devices(s2mpw03->dev, -1, s2mpw03_devs,
			ARRAY_SIZE(s2mpw03_devs), NULL, 0, NULL);
	if (ret < 0)
		goto err_mfd;

	device_init_wakeup(s2mpw03->dev, pdata->wakeup);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	dbgi2c = s2mpw03->i2c;
	dbg_s2mpw03 = s2mpw03;
	s2mpw03_root = debugfs_create_dir("s2mpw03-regs", NULL);
	s2mpw03_i2caddr = debugfs_create_file("i2caddr", 0644, s2mpw03_root, NULL, &s2mpw03_i2caddr_fops);
	s2mpw03_i2cdata = debugfs_create_file("i2cdata", 0644, s2mpw03_root, NULL, &s2mpw03_i2cdata_fops);
#endif

	pr_info("[PMIC] %s: end\n", __func__);

	return ret;

err_mfd:
	mfd_remove_devices(s2mpw03->dev);
err_irq_init:
	i2c_unregister_device(s2mpw03->i2c);
err_w_lock:
	mutex_destroy(&s2mpw03->i2c_lock);
err:
	return ret;
}

static int s2mpw03_i2c_remove(struct i2c_client *i2c)
{
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	debugfs_remove_recursive(s2mpw03_i2cdata);
	debugfs_remove_recursive(s2mpw03_i2caddr);
	debugfs_remove_recursive(s2mpw03_root);
#endif

	mfd_remove_devices(s2mpw03->dev);
	i2c_unregister_device(s2mpw03->i2c);
	mutex_destroy(&s2mpw03->i2c_lock);

	return 0;
}

static const struct i2c_device_id s2mpw03_i2c_id[] = {
	{ MFD_DEV_NAME, TYPE_S2MPW03 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mpw03_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static struct of_device_id s2mpw03_i2c_dt_ids[] = {
	{ .compatible = "samsung,s2mpw03mfd" },
	{ },
};
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_PM)
static int s2mpw03_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		enable_irq_wake(s2mpw03->irq);

	disable_irq(s2mpw03->irq);

	return 0;
}

#define WAKEUP_STAT2 0x3c54
#define VGPIO2PMU_EINT0 (17)
#define VGPIO2PMU_EINT1 (18)

static int s2mpw03_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct s2mpw03_dev *s2mpw03 = i2c_get_clientdata(i2c);
//	unsigned int wakeup_stat;

#if !IS_ENABLED(CONFIG_SAMSUNG_PRODUCT_SHIP)
	pr_info("%s:%s\n", MFD_DEV_NAME, __func__);
#endif /* CONFIG_SAMSUNG_PRODUCT_SHIP */
/*
	exynos_pmu_read(WAKEUP_STAT2, &wakeup_stat);

	if ((wakeup_stat & 1 << VGPIO2PMU_EINT0)
			|| (wakeup_stat & 1 << VGPIO2PMU_EINT1))
		log_irq_wakeup_reason(s2mpw03->irq);
*/
	if (device_may_wakeup(dev))
		disable_irq_wake(s2mpw03->irq);

	enable_irq(s2mpw03->irq);

	return 0;
}
#else
#define s2mpw03_suspend	NULL
#define s2mpw03_resume NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops s2mpw03_pm = {
	.suspend_late = s2mpw03_suspend,
	.resume_early = s2mpw03_resume,
};

static struct i2c_driver s2mpw03_i2c_driver = {
	.driver		= {
		.name	= MFD_DEV_NAME,
		.owner	= THIS_MODULE,
#if IS_ENABLED(CONFIG_PM)
		.pm	= &s2mpw03_pm,
#endif /* CONFIG_PM */
#if IS_ENABLED(CONFIG_OF)
		.of_match_table	= s2mpw03_i2c_dt_ids,
#endif /* CONFIG_OF */
	},
	.probe		= s2mpw03_i2c_probe,
	.remove		= s2mpw03_i2c_remove,
	.id_table	= s2mpw03_i2c_id,
};

static int __init s2mpw03_i2c_init(void)
{
	pr_info("%s:%s\n", MFD_DEV_NAME, __func__);
	return i2c_add_driver(&s2mpw03_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(s2mpw03_i2c_init);

static void __exit s2mpw03_i2c_exit(void)
{
	i2c_del_driver(&s2mpw03_i2c_driver);
}
module_exit(s2mpw03_i2c_exit);

MODULE_DESCRIPTION("s2mpw03 multi-function core driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
