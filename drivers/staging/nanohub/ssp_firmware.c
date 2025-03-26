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

#undef SSP_FIRMWARE_REVISION_CHUB

#if defined(CONFIG_SENSORS_SSP_FRESH6) || defined(CONFIG_SENSORS_SSP_WISE6)
#include "firmware/os_one_image_watch6.h"
#define SSP_FIRMWARE_REVISION_CHUB   	24101400
#elif defined(CONFIG_SENSORS_SSP_LUCKY)
#include "firmware/os_one_image_lucky.h"
#define SSP_FIRMWARE_REVISION_CHUB   	24100800
#else
#include "firmware/os_one_image.h"
#define SSP_FIRMWARE_REVISION_CHUB   20121600
#error "need to define mode config !!!"
#endif

//#define CONFIG_SENSORHUB_SPU


unsigned int get_module_rev(struct ssp_data *data)
{
	return SSP_FIRMWARE_REVISION_CHUB;
}

void toggle_mcu_reset(struct ssp_data *data)
{
	gpio_set_value_cansleep(data->rst, 0);

	usleep_range(1000, 1200);

	gpio_set_value_cansleep(data->rst, 1);

	msleep(50);

	if (!gpio_get_value(data->mcu_int2))
		pr_err("[SSP]: SH has entered bootloader in %s !!!!!",
			__func__);
}

int download_sensorhub_firmware(struct ssp_data *data, struct device *dev, void *addr)
{
#ifdef CONFIG_SENSORHUB_SPU
	int fw_size;
	char *fw_buf = NULL;

	if (addr == NULL)
		return -EINVAL;

	fw_size = request_spu_firmware(data, (u8 **)&fw_buf);
	if (fw_size > 0) {
		ssp_infof("dowload spu firmware");
		ssp_infof("fw type %d bin(size:%d) on %lx", data->fw_type, (int)fw_size, (unsigned long)addr);
		memcpy(addr, fw_buf, fw_size);

		release_spu_firmware(data, fw_buf);
		return 1;
	}
#endif

	ssp_infof("chub_fw_rev:%d,uResetCnt:%d,addr:0x%x,len:0x%x,%d)", data->chub_fw_rev, data->uResetCnt, 
		addr, os_chub_bin_len, os_chub_bin_len);

	if(os_chub_bin_len > CHUB_SRAM_IMAGE_SIZE) {
		memcpy_toio(addr, os_chub_bin, CHUB_SRAM_IMAGE_SIZE);
		memcpy_toio(addr + LPD_CHUB_IMAGE_START_OFFSET,
			os_chub_bin + LPD_CHUB_IMAGE_START_OFFSET, os_chub_bin_len - LPD_CHUB_IMAGE_START_OFFSET);
	} else {
		memcpy_toio(addr, os_chub_bin, os_chub_bin_len);
	}
	return 2;
}


