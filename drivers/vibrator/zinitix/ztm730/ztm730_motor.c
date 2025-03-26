/*
** =============================================================================
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** File:
**     ztm730_motor.c
**
** Description:
**     ztm730 vibration motor ic driver
**
** =============================================================================
*/
#define pr_fmt(fmt) "[VIB] " fmt
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/vibrator/sec_vibrator.h>
#include "ztm730_motor.h"
#include <linux/debugfs.h>
#include <linux/time.h>
#include <linux/suspend.h>
#ifdef CONFIG_DRV_SAMSUNG
#include <linux/sec_class.h>
#if defined(CONFIG_SEC_DEBUG)
#include <linux/sec_debug.h>
#endif
#endif
#define MAX_FIFO_SIZE	0xff

static struct ztm730_motor_data *g_ztm730MotorData;

static const struct regmap_config ztm730_motor_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static int ztm730_motor_set_frequency(struct device *dev, int frequency)
{
	struct ztm730_motor_data *pMotorData = dev_get_drvdata(dev);

	// freq_alert enum --> 0 from sec_vibrator
	if (frequency != FREQ_ALERT)
		pMotorData->frequency = frequency;
	pr_info("%s frequency=%d\n", __func__, frequency);
	return 0;
}

static int ztm730_motor_set_intensity(struct device *dev, int intensity)
{
	struct ztm730_motor_data *pMotorData = dev_get_drvdata(dev);
	int ret = 0;

	if (intensity < -(MAX_INTENSITY) || MAX_INTENSITY < intensity) {
		pr_err("%s out of range %d\n", __func__, intensity);
		return -EINVAL;
	}

	pr_info("%s intensity=%d\n", __func__, intensity);

	pMotorData->intensity = intensity;
	return ret;
}

static int ztm730_motor_reg_read(struct ztm730_motor_data *pMotorData, unsigned char reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(pMotorData->mpRegmap, reg, &val);

	if (ret < 0) {
		pr_err("%s:0x%x error(%d)\n", __func__, reg, ret);
		return ret;
	} else
		return val;
}

static int ztm730_motor_reg_write(struct ztm730_motor_data *pMotorData,
	unsigned char reg, unsigned char val)
{
	int ret;

	ret = regmap_write(pMotorData->mpRegmap, reg, val);
	if (ret < 0)
		pr_err("%s:0x%x=0x%x error(%d)\n", __func__, reg, val, ret);

	return ret;
}

static int ztm730_motor_raw_write(struct ztm730_motor_data *pMotorData,
	unsigned char reg, const unsigned char *val, int val_len)
{
	int ret;

	ret = regmap_raw_write(pMotorData->mpRegmap, reg, val, val_len);
	if (ret < 0)
		pr_err("%s:0x%x error(%d)\n", __func__, reg, ret);

	return ret;
}

static int ztm730_motor_set_bits(struct ztm730_motor_data *pMotorData,
	unsigned char reg, unsigned char mask, unsigned char val)
{
	int ret;

	ret = regmap_update_bits(pMotorData->mpRegmap, reg, mask, val);
	if (ret < 0)
		pr_err("%s:reg=%x, mask=0x%x, value=0x%x error (%d)\n",
			__func__, reg, mask, val, ret);

	return ret;
}

static int ztm730_motor_update_intensity(struct ztm730_motor_data *pMotorData)
{
	// only for scenarios in which motor is already running.
	struct ztm730_motor_pdata *pMotorPdata;
	int ret, strength;

	if (!pMotorData) {
		pr_err("%s:ztm730_motor_data NULL error\n", __func__);
		return -EINVAL;
	}
	pMotorPdata = pMotorData->msPlatData;
	strength = pMotorPdata->strength
			* pMotorData->intensity / 10000;

	pr_info("%s strength=0x%x intensity=%d\n", __func__,
			strength, pMotorData->intensity);

	ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_STRENGTH, strength);
	if (ret < 0) {
		pr_err("%s:STRENGTH 0x%02x write %d fail (%d)\n",
			__func__, MOTOR_REG_STRENGTH, strength, ret);
		return -EINVAL;
	}
	return ret;
}

static int ztm730_motor_update_frequency(struct ztm730_motor_data *pMotorData)
{
	// only for scenarios in which motor is already running.
	int freq_reg, freq_hz, ret = -EINVAL;

	if (!pMotorData) {
		pr_err("%s:ztm730_motor_data NULL error\n", __func__);
		return -EINVAL;
	}

	/* Driving Period = DRV_FREQ * 16 Clocks
	 *  -> DRV_FREQ = Driving Period / (16 Clocks * Clock Period)
	 *		 Clock Frequency / (16 Clocks * Driving Frequency)
	 *  DRV_FREQ_H[7:0] = DRV_FREQ[15:8]
	 * DRV_FREQ_L[7:0] = DRV_FREQ[7:0]
	 */

	pr_info("%s frequency=%d\n", __func__, pMotorData->frequency);

	freq_hz = pMotorData->frequency;
	freq_reg = (MOTOR_CLK * 10) / (freq_hz << 4);
	ret = ztm730_motor_reg_write(pMotorData,
		MOTOR_REG_DRV_FREQ_H, (freq_reg >> 8));
	if (ret < 0) {
		pr_err("%s:DRV_FREQ_H 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_DRV_FREQ_H, ret);
		goto out;
	}

	ret = ztm730_motor_reg_write(pMotorData,
		MOTOR_REG_DRV_FREQ_L, (freq_reg & 0xff));
	if (ret < 0) {
		pr_err("%sDRV_FREQ_L 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_DRV_FREQ_L, ret);
		goto out;
	}

	ret = ztm730_motor_reg_write(pMotorData,
		MOTOR_REG_RESO_FREQ_H, (freq_reg >> 8));
	if (ret < 0) {
		pr_err("%s:RESO_FREQ_H 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_RESO_FREQ_H, ret);
		goto out;
	}

	ret = ztm730_motor_reg_write(pMotorData,
		MOTOR_REG_RESO_FREQ_L, (freq_reg & 0xff));
	if (ret < 0) {
		pr_err("%s:RESO_FREQ_L 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_RESO_FREQ_L, ret);
		goto out;
	}

out:
	return ret;
}

static int ztm730_motor_set_pattern_params(struct device *dev)
{
	struct ztm730_motor_data *pMotorData = dev_get_drvdata(dev);
	int ret = -EINVAL;

	if (!pMotorData) {
		pr_err("%s:ztm730_motor_data NULL error\n", __func__);
		goto out;
	}

	ret = ztm730_motor_update_intensity(pMotorData);
	if (ret < 0)
		goto out;
	ret = ztm730_motor_update_frequency(pMotorData);

out:
	return ret;
}

static int ztm730_motor_run(struct ztm730_motor_data *pMotorData)
{
	struct ztm730_motor_pdata *pMotorPdata;
	int freq_reg, freq_hz, ret = -EINVAL;
	unsigned char val, strength;

	if (!pMotorData) {
		pr_err("%s:ztm730_motor_data NULL error\n", __func__);
		return -EINVAL;
	}

	pMotorPdata = pMotorData->msPlatData;

	/* strength is calculated with the level-mapped value in dt
	 *  multiplying intensity value out of 10000 */
	strength = pMotorPdata->strength
			* pMotorData->intensity / 10000;

	pr_info("%s strength=0x%x intensity=%d\n", __func__,
			strength, pMotorData->intensity);

	/* frequency data from platform is used unless it's 0
	 * which means to use frequency defined in device tree for each model
	 */
	if (pMotorData->frequency)
		freq_hz = pMotorData->frequency;

	ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_ENABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN write fail (%d)\n", __func__, ret);
		goto out;
	}

	if (pMotorPdata->soft_en_delay)
		msleep(pMotorPdata->soft_en_delay);

	if (pMotorPdata->overdrive_num) {
		val = (pMotorData->overdrive & MOTOR_REG_OVER_DRV_EN_MASK)
			<< OVER_DRV_SHIFT_EN;
		val |= (pMotorPdata->overdrive_num & MOTOR_REG_OVER_DRV_CNT_MASK)
			<< OVER_DRV_SHIFT_CNT;
		ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_OVER_DRV, val);
		if (ret < 0) {
			pr_err("%s:OVER_DRV 0x%02x write fail(%d)\n",
				__func__, MOTOR_REG_OVER_DRV, ret);
			goto out;
		}
	}
	ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_STRENGTH, strength);
	if (ret < 0) {
		pr_err("%s:STRENGTH 0x%02x write %d fail (%d)\n",
			__func__, MOTOR_REG_STRENGTH, strength, ret);
		goto out;
	}

	ret = ztm730_motor_set_bits(pMotorData, MOTOR_REG_MODE_01,
		1 << MODE_01_SHIFT_DRV_MODE,
		pMotorPdata->meLoop << MODE_01_SHIFT_DRV_MODE);
	if (ret < 0) {
		pr_err("%s:MODE_01 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_MODE_01, ret);
		goto out;
	}

	/* Driving Period = DRV_FREQ * 16 Clocks
	 *  -> DRV_FREQ = Driving Period / (16 Clocks * Clock Period)
	 *		 Clock Frequency / (16 Clocks * Driving Frequency)
	 *  DRV_FREQ_H[7:0] = DRV_FREQ[15:8]
	 * DRV_FREQ_L[7:0] = DRV_FREQ[7:0]	
	 */
	freq_reg = (MOTOR_CLK * 10) / (freq_hz << 4);
	ret = ztm730_motor_reg_write(pMotorData,
		MOTOR_REG_DRV_FREQ_H, (freq_reg >> 8));
	if (ret < 0) {
		pr_err("%s:DRV_FREQ_H 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_DRV_FREQ_H, ret);
		goto out;
	}

	ret = ztm730_motor_reg_write(pMotorData,
		MOTOR_REG_DRV_FREQ_L, (freq_reg & 0xff));
	if (ret < 0) {
		pr_err("%sDRV_FREQ_L 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_DRV_FREQ_L, ret);
		goto out;
	}

	ret = ztm730_motor_reg_write(pMotorData,
		MOTOR_REG_RESO_FREQ_H, (freq_reg >> 8));
	if (ret < 0) {
		pr_err("%s:RESO_FREQ_H 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_RESO_FREQ_H, ret);
		goto out;
	}

	ret = ztm730_motor_reg_write(pMotorData,
		MOTOR_REG_RESO_FREQ_L, (freq_reg & 0xff));
	if (ret < 0) {
		pr_err("%s:RESO_FREQ_L 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_RESO_FREQ_L, ret);
		goto out;
	}

	ret = ztm730_motor_reg_write(pMotorData,
		MOTOR_REG_ADC_SAMPLING_TIME, pMotorPdata->adc_sampling_time);
	if (ret < 0) {
		pr_err("%s:ADC_SAMPLING_TIME 0x%02x write fail(%d)\n",
			__func__, MOTOR_REG_ADC_SAMPLING_TIME, ret);
		goto out;
	}

	if (!pMotorData->running) {
		if (pMotorData->en_gpio > 0)
			gpio_set_value(pMotorData->en_gpio, 1);

		ret = ztm730_motor_reg_write(pMotorData,
			MOTOR_REG_MODE_00, pMotorPdata->motor_start_data);
		if (ret < 0) {
			pr_err("%s:MODE_00 write fail(%d)\n", __func__, ret);
			goto out;
		}
		pMotorData->running = true;
	}
out:
	return ret;
}

static int ztm730_motor_set_fifo_intensity(struct device *dev, int intensity)
{
	struct ztm730_motor_data *pMotorData = dev_get_drvdata(dev);
	int ret = 0;

	if (intensity < -(MAX_INTENSITY) || MAX_INTENSITY < intensity) {
		pr_err("%s out of range %d\n", __func__, intensity);
		ret = -EINVAL;
		goto out;
	}

	pr_info("%s fifo_intensity=%d\n", __func__, intensity);

	pMotorData->fifo_intensity = intensity;
out:
	return ret;
}

static int ztm730_motor_fifo_play(struct ztm730_motor_data *pMotorData, const u8 *data, const int data_size)
{
	struct ztm730_motor_pdata *pMotorPdata;
	int index, write_size, ret = -EINVAL;
	unsigned char strength;

	if (data_size == 0) {
		pr_err("%s:data_size is ZERO\n", __func__);
		return -EINVAL;
	}

	if (!pMotorData) {
		pr_err("%s:ztm730_motor_data NULL error\n", __func__);
		return -EINVAL;
	}

	pMotorPdata = pMotorData->msPlatData;
	ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_ENABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN write fail(%d)\n", __func__, ret);
		return ret;
	}

	if (pMotorPdata->soft_en_delay)
		msleep(pMotorPdata->soft_en_delay);

	/* strength is calculated with the level-mapped value in dt
	 * multiplying intensity value out of 10000
	 * pMotorPdata->fifo_strength = 0x80(max)  pMotorData->fifo_intensity (1250~10000)
	 */

	if (pMotorData->fifo_intensity <= 3500) {
		strength = pMotorPdata->fifo_strength
			* pMotorData->fifo_intensity / 7000;
	}

	else
		strength = (pMotorPdata->fifo_strength * pMotorData->fifo_intensity / 13000)
		+ ((3 * pMotorPdata->fifo_strength)/13);

	pr_info("%s strength=0x%x intensity=%d\n", __func__,
	strength, pMotorData->fifo_intensity);

	ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_FIFO_STRENGTH, strength);
	if (ret < 0) {
		pr_err("%s:STRENGTH 0x%02x write %d fail (%d)\n",
			__func__, MOTOR_REG_FIFO_STRENGTH, strength, ret);
		return ret;
	}

	ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_FIFO_INTERVAL, (unsigned char)data[0]);
	if (ret < 0) {
		pr_err("%s:FIFO_INTERVAL write fail(%d)\n", __func__, ret);
		return ret;
	}

	for (index = 1; index < data_size; index += write_size) {
		if ((data_size - index) > MAX_FIFO_SIZE)
			write_size = MAX_FIFO_SIZE;
		else
			write_size = data_size - index;

		ret = ztm730_motor_raw_write(pMotorData, MOTOR_REG_FIFO_DATA,
					(const unsigned char *)&data[index], write_size);
		if (ret < 0) {
			pr_err("%s:FIFO_DATA write fail(%d)\n", __func__, ret);
			return ret;
		}
	}

	ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN write fail(%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int ztm730_motor_haptic_play(struct device *dev, bool en)
{
	struct ztm730_motor_data *pMotorData = dev_get_drvdata(dev);
	struct ztm730_motor_pdata *pMotorPdata;
	int err;

	pr_info("%s en:%d\n", __func__, en);

	if (!pMotorData) {
		pr_err("%s:ztm730_motor_data NULL error\n", __func__);
		return -EINVAL;
	}

	pMotorPdata = pMotorData->msPlatData;
	mutex_lock(&pMotorData->lock);
	if (pMotorData->is_suspend)
		goto out;

	if (en) {
		err = ztm730_motor_run(pMotorData);
		if (err < 0) {
			pr_err("%s:motor run fail(%d)\n", __func__, err);
			goto out;
		}
	} else if (pMotorData->running) {
		err = ztm730_motor_reg_write(pMotorData, MOTOR_REG_MODE_00, pMotorPdata->motor_stop_data);
		if (err < 0) {
			pr_err("%s:MODE_00 write fail(%d)\n", __func__, err);
			goto out;
		}

		if (pMotorData->en_gpio > 0)
			gpio_set_value(pMotorData->en_gpio, 0);

		err = ztm730_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
		if (err < 0) {
			pr_err("%s:SOFT_EN write fail(%d)\n",
						__func__, err);
			goto out;
		}

		pMotorData->running = false;
		ktime_get_real_ts64(&pMotorData->last_motor_off);
		pr_info("%s:Stop\n", __func__);
	}

out:
	mutex_unlock(&pMotorData->lock);

	return 0;
}

static int ztm730_get_motor_type(struct device *dev, char *buf)
{
	struct ztm730_motor_data *ddata = dev_get_drvdata(dev);
	int ret = snprintf(buf, VIB_BUFSIZE, "%s\n", ddata->msPlatData->vib_type);

	return ret;
}

static int ztm730_get_fifo_filepath(struct device *dev, char *buf)
{
	struct ztm730_motor_data *ddata = dev_get_drvdata(dev);
	int ret = snprintf(buf, VIB_BUFSIZE, "%s\n", ddata->msPlatData->file_path);

	return ret;
}

static int ztm730_motor_store_fifo_vib(struct device *dev, int file_num)
{
	struct ztm730_motor_data *pMotorData = dev_get_drvdata(dev);

	const struct firmware *vib_fw;
	int ret;
	char *file_path;
	s64 start = 0, duration = 0;

	pr_info("%s file=%d +\n", __func__, file_num);

	if (file_num == 1)
		file_path = "01_TAP_LIGHT.bin";
	else if (file_num == 2)
		file_path = "02_TAP_NORMAL.bin";
	else if (file_num == 3)
		file_path = "03_TAP_HEAVY.bin";
	else if (file_num == 4)
		file_path = "04_TAP_HEAVY_2.bin";
	else if (file_num == 5)
		file_path = "05_CLICK_FINE.bin";
	else if (file_num == 6)
		file_path = "06_CLICK_FINE_AUDIBLE.bin";
	else if (file_num == 7)
		file_path = "07_CLICK_NORMAL.bin";
	else if (file_num == 8)
		file_path = "08_CLICK_NORMAL_AUDIBLE.bin";
	else if (file_num == 9)
		file_path = "09_CLICK_HEAVY.bin";
	else if (file_num == 10)
		file_path = "10_CLICK_HEAVY_AUDIBLE.bin";
	else if (file_num == 11)
		file_path = "11_FADE_IN_SHORT.bin";
	else if (file_num == 12)
		file_path = "12_FADE_IN_MEDIUM.bin";
	else if (file_num == 13)
		file_path = "13_FADE_IN_LONG.bin";
	else if (file_num == 14)
		file_path = "14_FADE_IN_LONGER.bin";
	else if (file_num == 15)
		file_path = "15_FADE_OUT_SHORT.bin";
	else if (file_num == 16)
		file_path = "16_FADE_OUT_MEDIUM.bin";
	else if (file_num == 17)
		file_path = "17_FADE_OUT_LONG.bin";
	else if (file_num == 18)
		file_path = "18_FADE_OUT_LONGER.bin";
	else if (file_num == 19)
		file_path = "19_EFFECT_SPRING.bin";
	else if (file_num == 20)
		file_path = "20_TOUCH_BEZEL_FINE.bin";
	else if (file_num == 21)
		file_path = "21_TOUCH_BEZEL_NORMAL.bin";
	else if (file_num == 22)
		file_path = "22_CLICK_SMOOTH.bin";
	else if (file_num == 23)
		file_path = "23_CLICK_BUMP.bin";
	else if (file_num == 24)
		file_path = "24_CLICK_BUMP_AUDIBLE.bin";
	else if (file_num == 25)
		file_path = "25_END_EFFECT.bin";
	else if (file_num == 26)
		file_path = "26_LONG_VIB.bin";
	else if (file_num == 27)
		file_path = "27_EFFECT_WATCH_FACE_ANALOG.bin";
	else if (file_num == 28)
		file_path = "28_EFFECT_WATCH_FACE_DIGITAL.bin";
	else {
		pr_err("%s error. -\n", __func__);
		return -EINVAL;
	}

	start = ktime_to_ms(ktime_get());

	mutex_lock(&pMotorData->lock);

	/* fw loaded */
	ret = request_firmware(&vib_fw, file_path, pMotorData->dev);

	if (ret < 0) {
		pr_err("failed to read %s", file_path);
		goto fw_request_fail;
	}

	pMotorData->fifo_data_size = vib_fw->size;

	duration = ktime_to_ms(ktime_get()) - start;
	pr_info("%s file read duration=%dms\n", __func__, duration);

	ret = ztm730_motor_fifo_play(pMotorData, (unsigned char *)vib_fw->data, vib_fw->size);
	if (ret < 0) {
		pr_err("%s:failed fifo_play(%d)\n", __func__, ret);
		goto fifo_play_fail;
	}
	pMotorData->msPlatData->file_path = file_path;

fifo_play_fail:
	release_firmware(vib_fw);
fw_request_fail:
	vib_fw = NULL;
	mutex_unlock(&pMotorData->lock);
	duration = ktime_to_ms(ktime_get()) - start;
	pr_info("%s total duration=%dms -\n", __func__, duration);

	return 0;
}

static const struct sec_vibrator_ops ztm730_ops = {
	.enable = ztm730_motor_haptic_play,
	.set_intensity = ztm730_motor_set_intensity,
	.set_fifo_intensity = ztm730_motor_set_fifo_intensity,
	.set_frequency = ztm730_motor_set_frequency,
	.get_motor_type = ztm730_get_motor_type,
	.get_fifo_filepath = ztm730_get_fifo_filepath,
	.enable_fifo = ztm730_motor_store_fifo_vib,
	.update_packet_params = ztm730_motor_set_pattern_params,
};

static int ztm730_motor_platform_init(struct ztm730_motor_data *pMotorData)
{
	struct ztm730_motor_pdata *pMotorPdata = pMotorData->msPlatData;
	int ret = 0, i;
	unsigned int value_tmp = 0;

	pr_info("%s\n", __func__);

	ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_ENABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN 0x%02x write fail(%d)\n",
					__func__, MOTOR_REG_SOFT_EN, ret);
		goto out;
	}

	if (pMotorPdata->soft_en_delay)
		msleep(pMotorPdata->soft_en_delay);

	value_tmp = 0;
	value_tmp |= (pMotorPdata->meLoop << MODE_01_SHIFT_DRV_MODE);
	value_tmp |= ((pMotorPdata->break_mode ? 0x00 : 0x01) << MODE_01_SHIFT_NBREAK_EN);
	value_tmp |= (MODE_01_DEFAULT_PGA_BEMP_GAIN << MODE_01_SHIFT_PGA_BEMP_GAIN);
	ret = ztm730_motor_set_bits(pMotorData,
		MOTOR_REG_MODE_01, MOTOR_REG_MODE_01_MASK, value_tmp);
	if (ret < 0) {
		pr_err("%s:MODE_01 0x%02x write fail(%d)\n",
					__func__, MOTOR_REG_MODE_01, ret);
		goto out;
	}

	value_tmp = 0;
	value_tmp |= (pMotorPdata->motor_type << MODE_13_SHIFT_ERM_NLRA);
	value_tmp |= (1 << MODE_13_SHIFT_SIN_OUT_EN); //(will enable Sine wave on HW request)
	ret = ztm730_motor_set_bits(pMotorData,
		MOTOR_REG_MODE_13, MOTOR_REG_MODE_13_MASK, value_tmp);
	if (ret < 0) {
		pr_err("%s:MODE_13 0x%02x write fail(%d)\n",
					__func__, MOTOR_REG_MODE_13, ret);
		goto out;
	}

	if (pMotorPdata->count_init_regs > 0) {
		for (i = 0; i < pMotorPdata->count_init_regs; i++) {
			ret = ztm730_motor_reg_write(pMotorData,
						pMotorPdata->init_regs[i].addr,
						pMotorPdata->init_regs[i].data);
			if (ret < 0) {
				pr_err("%s:init_regs 0x%02x write fail(%d)\n",
					__func__,
					pMotorPdata->init_regs[i].addr, ret);
				goto out;
			}
		}
	}

	ret = ztm730_motor_reg_write(pMotorData, MOTOR_REG_SOFT_EN, SOFT_DISABLE);
	if (ret < 0) {
		pr_err("%s:SOFT_EN 0x%02x write fail(%d)\n",
					__func__, MOTOR_REG_SOFT_EN, ret);
		goto out;
	}

out:
	return ret;
}

static void ztm730_motor_trigger_init(struct work_struct *work)
{
	struct ztm730_motor_data *pMotorData = container_of(work,
						       struct ztm730_motor_data,
						       trigger_init);
	mutex_lock(&pMotorData->lock);
	ztm730_motor_platform_init(pMotorData);
	mutex_unlock(&pMotorData->lock);
}

static int ztm730_motor_haptics_init(struct ztm730_motor_data *pMotorData)
{
	pMotorData->sec_vib_ddata.time_compensation = 10;
	pMotorData->sec_vib_ddata.max_delay_ms = 20;
	pMotorData->sec_vib_ddata.frequency = pMotorData->frequency;
	pMotorData->sec_vib_ddata.dev = pMotorData->dev;
	pMotorData->sec_vib_ddata.vib_ops = &ztm730_ops;
	sec_vibrator_register(&pMotorData->sec_vib_ddata);

	INIT_WORK(&pMotorData->trigger_init, ztm730_motor_trigger_init);

	//pMotorData->ws = wakeup_source_register(NULL, "vibrator_wlock");
	mutex_init(&pMotorData->lock);

	return 0;
}

int ztm730_motor_reset_handler(void)
{
	if (g_ztm730MotorData == NULL) {
		pr_err("%s:g_ztm730MotorData is NULL\n", __func__);
		return -ENODEV;
	}

	queue_work(system_highpri_wq, &g_ztm730MotorData->trigger_init);

	return 0;
}

bool ztm730_motor_is_running(void)
{
	if (g_ztm730MotorData == NULL)
		return false;

	return g_ztm730MotorData->running;
}

static int ztm730_motor_parse_dt(struct i2c_client *client, struct ztm730_motor_pdata *pdata)
{
	const char *motor_type;
	const char *loop_type;
	int err;

	pr_info("%s\n", __func__);

	err = of_property_read_string(client->dev.of_node,"ztm730_motor,motor-type", &motor_type);
	if (err < 0) {
		pr_err("%s:motor-type read fail(%d)\n", __func__, err);
		return -ENODEV;
	}

	if (!strcmp(motor_type, "LRA")) {
		pdata->motor_type = ACTUATOR_LRA;
		pdata->motor_start_data = MODE_00_I2C_LRA;
		pdata->motor_stop_data = MODE_00_STOP_LRA;
	} else if (!strcmp(motor_type, "ERM")) {
		pdata->motor_type = ACTUATOR_ERM;
		pdata->motor_start_data = MODE_00_I2C_ERM;
		pdata->motor_stop_data = MODE_00_STOP_ERM;
	} else {
		pr_err("%s:Wrong motor type[%s]\n", __func__, motor_type);
		return -ENODEV;
	}
	pr_info("%s:motor-type = %s\n", __func__, motor_type);

	err = of_property_read_string(client->dev.of_node,
			"ztm730_motor,loop-type", &loop_type);
	if (err < 0) {
		pr_err("%s:loop-type read fail(%d)\n", __func__, err);
		return -ENODEV;
	}
	if (!strcmp(loop_type, "open"))
		pdata->meLoop = OPEN_LOOP;
	else if (!strcmp(loop_type, "closed"))
		pdata->meLoop = CLOSED_LOOP;
	else {
		pr_err("%s: Wrong loop type: %s\n", __func__, loop_type);
		return -ENODEV;
	}
	pr_info("%s:loop-type[%s]\n", __func__, loop_type);

	pdata->break_mode = of_property_read_bool(client->dev.of_node,
			"ztm730_motor,break-on");
	pr_info("%s:break-on[%d]\n", __func__, (int)pdata->break_mode);

	err = of_property_read_u32(client->dev.of_node,
			"ztm730_motor,brake-delay-ms", &pdata->break_delay);
	if (err < 0) {
		pdata->break_delay = DEFAULT_BRAKE_DELAY;
		dev_warn(&client->dev, "%s: brake-delay-ms read fail(%d) :%d\n",
				__func__, err, pdata->break_delay);
	} else
		pr_info("%s:brake-delay-ms[%d]\n", __func__, pdata->break_delay);

	pdata->count_init_regs = of_property_count_u32_elems(client->dev.of_node,
			"ztm730_motor,regs-init");
	if (pdata->count_init_regs > 0) {
		pdata->init_regs = devm_kzalloc(&client->dev,
				sizeof(u32) * pdata->count_init_regs, GFP_KERNEL);
		err = of_property_read_u32_array(client->dev.of_node,
				"ztm730_motor,regs-init",
				(u32 *)pdata->init_regs, pdata->count_init_regs);
		if (err < 0) {
			pr_err("%s:regs-init read fail(%d)\n", __func__, err);
			return -ENODEV;
		}
		pdata->count_init_regs /= 2;
	}
	pr_info("%s:regs-init count[%d]\n", __func__, pdata->count_init_regs);

	pdata->en_gpio = of_get_named_gpio(client->dev.of_node,
			"ztm730_motor,motor_en", 0);
	if (pdata->en_gpio < 0)
		pr_debug("%s: motor_en read fail(%d)\n",
				__func__, pdata->en_gpio);
	else
		pr_info("%s:motor_en[%d]\n", __func__, pdata->en_gpio);

	err = of_property_read_string(client->dev.of_node,
			"ztm730_motor,regulator-name", &pdata->regulator_name);
	if (err < 0)
		pr_warn("%s: regulator-name read fail(%d)\n",
				__func__, err);
	else
		pr_info("%s:regulator-name[%s]\n", __func__, pdata->regulator_name);

	err = of_property_read_u32(client->dev.of_node,
			"ztm730_motor,adc-sampling-time", &pdata->adc_sampling_time);
	if (err < 0) {
		pdata->adc_sampling_time = DEFAULT_ADC_SAMPLING_TIME;
		pr_warn("%s: adc-sampling-time read fail(%d) :%d\n",
				__func__, err, pdata->adc_sampling_time);
	} else
		pr_info("%s:adc-sampling-time[%d]\n", __func__, pdata->adc_sampling_time);

	err = of_property_read_u32(client->dev.of_node,
			"ztm730_motor,soft-en-delay-ms", &pdata->soft_en_delay);
	if (err < 0) {
		pdata->soft_en_delay = DEFAULT_SOFT_EN_DELAY;
		pr_warn("%s: soft-en-delay-ms read fail(%d) :%d\n",
				__func__, err, pdata->soft_en_delay);
	} else
		pr_info("%s:soft-en-delay-ms[%d]\n", __func__, pdata->soft_en_delay);

	err = of_property_read_u32(client->dev.of_node,
			"ztm730_motor,strength", &pdata->strength);
	if (err < 0)
		pr_err("%s:strength read fail(%d)\n", __func__, err);
	else
		pr_info("%s:strength[%d]\n", __func__, pdata->strength);

	err = of_property_read_u32(client->dev.of_node,
			"ztm730_motor,fifo_strength", &pdata->fifo_strength);
	if (err < 0) {
		pdata->fifo_strength = 0x80;
		pr_err("%s:fifo_strength read fail(%d)\n", __func__, err);
	} else
		pr_info("%s:fifo_strength[%x]\n", __func__, pdata->fifo_strength);

	err = of_property_read_u32(client->dev.of_node,
			"ztm730_motor,frequency", &pdata->frequency);
	if (err < 0) {
		pdata->frequency = 2100; // in fail case
		pr_err("%s:frequency read fail(%d)\n", __func__, err);
	} else
		pr_info("%s:frequency[%d]\n", __func__, pdata->frequency);

	err = of_property_read_u32(client->dev.of_node,
			"ztm730_motor,overdrive-num", &pdata->overdrive_num);
	if (err < 0) {
		pr_debug("%s: overdrive-num read fail(%d) :%d\n",
				__func__, err, pdata->overdrive_num);
	} else
		pr_info("%s:overdrive-num[%d]\n", __func__, pdata->overdrive_num);

	err = of_property_read_string(client->dev.of_node,
			"samsung,vib_type", &pdata->vib_type);
	if (err < 0)
		pr_info("%s: vib type not specified\n", __func__);
	else
		pr_info("vib_type = %s\n", pdata->vib_type);

	return 0;
}

static int ztm730_motor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ztm730_motor_data *pMotorData;
	struct ztm730_motor_pdata *pMotorPdata;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s:I2C check failed\n", __func__);
		return -ENODEV;
	}

	/* platform_data init */
	if (client->dev.of_node) {
		pMotorPdata = devm_kzalloc(&client->dev,
				sizeof(struct ztm730_motor_pdata), GFP_KERNEL);
		if (!pMotorPdata) {
			pr_err("%s:unable to allocate pdata memory\n", __func__);
			return -ENOMEM;
		}
		err = ztm730_motor_parse_dt(client, pMotorPdata);
		if (err < 0) {
			pr_err("%s:fail to read DT %d\n", __func__, err);
			return -ENODEV;
		}
	} else {
		pr_err("%s:fail to platform data init\n", __func__);
		return -ENODEV;
	}
	pMotorData = devm_kzalloc(&client->dev, sizeof(struct ztm730_motor_data), GFP_KERNEL);
	if (pMotorData == NULL) {
		pr_err("%s:no memory\n", __func__);
		return -ENOMEM;
	}
	pMotorData->client = client;
	pMotorData->dev = &client->dev;
	pMotorData->msPlatData = pMotorPdata;
	g_ztm730MotorData = pMotorData;
	i2c_set_clientdata(client, pMotorData);
	pMotorData->mpRegmap = devm_regmap_init_i2c(client, &ztm730_motor_i2c_regmap);
	if (IS_ERR(pMotorData->mpRegmap)) {
		err = PTR_ERR(pMotorData->mpRegmap);
		pr_err("%s:Failed to allocate register map(%d)\n", __func__, err);
		return err;
	}
	if (pMotorPdata->regulator_name == NULL) {
		pMotorData->regulator
			= devm_regulator_get(pMotorData->dev, "ztm730_motor");
		if (IS_ERR(pMotorData->regulator)) {
			pr_err("%s:Failed to get moter power supply.\n", __func__);
			return -EPROBE_DEFER;
		}
		err = regulator_set_voltage(pMotorData->regulator, MOTOR_VCC, MOTOR_VCC);
		if (err < 0)
			pr_err("%s:Failed to set moter power %duV(%d)\n",
			__func__, MOTOR_VCC, err);
		err = regulator_enable(pMotorData->regulator);
		if (err < 0) {
			pr_err("%s:enable fail(%d)\n", __func__, err);
			return -EFAULT;
		} else
			pr_info("%s regulator enable\n",
					__func__);
	}

	pMotorData->frequency = pMotorPdata->frequency;
	if (pMotorPdata->en_gpio > 0) {
		err = devm_gpio_request_one(pMotorData->dev, pMotorPdata->en_gpio,
			GPIOF_DIR_OUT | GPIOF_INIT_LOW, "motor");
		if (err < 0) {
			pr_err("%s:en_gpio request fail %d\n", __func__, err);
			return -EFAULT;
		}
	}
	pMotorData->en_gpio = pMotorPdata->en_gpio;
	err = ztm730_motor_reg_read(pMotorData, MOTOR_REG_ADC_SAMPLING_TIME);
	if (err < 0) {
		pr_err("%s:i2c bus fail(%d)\n", __func__, err);
		return -EFAULT;
	}
	pr_info("%s:ADC_SAMPLING_TIME(0x%x)\n", __func__, err);
	pMotorData->mnDeviceID = err;

	err = ztm730_motor_platform_init(pMotorData);
	if (err < 0) {
		pr_err("%s:dev_init_platform failed(%d)\n", __func__, err);
		return -EFAULT;
	}
	err = ztm730_motor_haptics_init(pMotorData);
	if (err < 0) {
		pr_err("%s:ztm730_motor_haptics_init failed(%d)\n", __func__, err);
		return -EFAULT;
	}

	pr_info("%s:succeeded\n", __func__);

	return 0;
}

void ztm730_motor_shutdown(struct i2c_client *client)
{
	struct ztm730_motor_data *pMotorData = i2c_get_clientdata(client);
	struct ztm730_motor_pdata *pMotorPdata = pMotorData->msPlatData;
	int err;

	pr_info("%s", __func__);

	if (pMotorData->regulator) {
		err = regulator_disable(pMotorData->regulator);
		if (err < 0)
			pr_err("%s:%s:disable fail(%d)", __func__,
				pMotorPdata->regulator_name, err);
		else
			pr_info("%s:%s disable\n", __func__,
				pMotorPdata->regulator_name);
	}
}

static int ztm730_motor_suspend(struct device *dev)
{
	struct ztm730_motor_data *pMotorData = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	pMotorData->is_suspend = true;
	return 0;
}

static int ztm730_motor_resume(struct device *dev)
{
	struct ztm730_motor_data *pMotorData = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	pMotorData->is_suspend = false;
	return 0;
}

static SIMPLE_DEV_PM_OPS(ztm730_motor_pm_ops, ztm730_motor_suspend,
		ztm730_motor_resume);

#if defined(CONFIG_OF)
static struct of_device_id haptic_dt_ids[] = {
	{ .compatible = HAPTICS_DEVICE_NAME },
	{ },
};
MODULE_DEVICE_TABLE(of, haptic_dt_ids);
#endif /* CONFIG_OF */

static struct i2c_device_id ztm730_motor_id_table[] = {
	{ HAPTICS_DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ztm730_motor_id_table);

static struct i2c_driver ztm730_motor_driver = {
	.driver = {
		.name = HAPTICS_DEVICE_NAME,
		.owner = THIS_MODULE,
		.pm = &ztm730_motor_pm_ops,
#if defined(CONFIG_OF)
		.of_match_table = haptic_dt_ids,
#endif /* CONFIG_OF */
	},
	.id_table = ztm730_motor_id_table,
	.probe = ztm730_motor_probe,
	.shutdown = ztm730_motor_shutdown,
};

static int __init ztm730_motor_init(void)
{
	return i2c_add_driver(&ztm730_motor_driver);
}

static void __exit ztm730_motor_exit(void)
{
	i2c_del_driver(&ztm730_motor_driver);
}

module_init(ztm730_motor_init);
module_exit(ztm730_motor_exit);

MODULE_AUTHOR("samsung");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);
