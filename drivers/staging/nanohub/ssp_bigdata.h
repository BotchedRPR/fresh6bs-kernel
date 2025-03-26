/*
 *  Copyright (C) 2024, Samsung Electronics Co. Ltd. All Rights Reserved.
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

#ifndef __SSP_BIGDATA_H__
#define __SSP_BIGDATA_H__

#define CONFIG_SENSORS_SSP_BIGDATA_V2	(1)

#define SSP_BIG_VERSION					(0)

/* Big Data Event */
#define SSP_BIG_SENSOR_STUCK_INFO		0x01
#define SSP_BIG_SENSOR_STUCK_REGDUMP	0x02
#define SSP_BIG_LIB_REG_STATUS			0x03

#define SSP_BIG_RESET_INFO				0x04
#define SSP_BIG_CRASH_MINIDUMP			0x05
#define SSP_BIG_WAKEUP_INFO				0x06
#define SSP_BIG_SENSOR_INIT_STATE		0x07

#define SSP_BIG_ESN						0x50
#define SSP_BIG_TEST					0x55

#ifdef CONFIG_SENSORS_SSP_BIGDATA_V2 
int ssp_big_queue_work(struct ssp_data *data, u8 event, bool isNofity);
void report_wakeup_info(struct ssp_data *data, char lib_num);
void report_wakeup_info_async(struct ssp_data *data, char lib_num, char count);
void report_wakeup_info_err(struct ssp_data *data, char inst, int sensor);
int initialize_bigdata_work(struct ssp_data *data);
#else
static inline int ssp_big_queue_work(struct ssp_data *data, u8 event, bool isNofity) {return 0; }
static inline void report_wakeup_info(struct ssp_data *data, char lib_num) {return; }
static inline void report_wakeup_info_async(struct ssp_data *data, char lib_num, char count) {return; }
static inline void report_wakeup_info_err(struct ssp_data *data, char inst, int sensor) {return; }
static inline int initialize_bigdata_work(struct ssp_data *data) {return 0; }
#endif
#endif // __SSP_BIGDATA_H__