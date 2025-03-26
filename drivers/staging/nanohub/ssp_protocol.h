/*
 *  Copyright (C) 2017, Samsung Electronics Co. Ltd. All Rights Reserved.
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

#ifndef __SSP_PROTOCOL_H__
#define __SSP_PROTOCOL_H__

#include <linux/bitops.h>

/*
 *AP2HUB_READ (0x00)
 */
#define SYNC_TIMESTAMP_OVER					0x00

#define MSG2SSP_AP_WHOAMI					0x01
#define MSG2SSP_AP_FIRMWARE_REV				0x02
#define MSG2SSP_AP_SENSOR_SCANNING			0x03
#define MSG2SSP_AP_MODEL_NAME				0x04

#define MSG2SSP_INST_BYPASS_SENSOR_ADD		0x05
#define MSG2SSP_INST_BYPASS_SENSOR_REMOVE	0x06
#define MSG2SSP_INST_CHANGE_DELAY			0x07

#define SYNC_TIMESTAMP						0x0C
#define BEZELMAG_CALIBRATION				0x0D
#define BEZELMAG_SET_CAL_DATA				0x0E
#define MSG2SSP_AP_GET_BIG_DATA				0x0F

#define SENSOR_FACTORY						0x10
#define GYROSCOPE_DPS_FACTORY				0x11
#define MCU_FACTORY							0x12
#define GEOMAGNETIC_FACTORY					0x13
#define HRM_LIB_VERSION_INFO				0x14
#define HRM_IR_LEVEL_THRESHOLD				0x15
#define MSG2SSP_SENSOR_REG_DUMP				0x16
#define MSG2SSP_AP_FEATURE_LIST_INFO		0x17
#define MSG2SSP_SSR_STUCK_INFO				0x18
#define HRM_SENSOR_INFO   					0x19
#define HRM_IOCTL       					0x1A
#define HRM_DQA								0x1B
#define MSG2SSP_AP_SET_LCD_TYPE				0x1D

/*
 * AP2HUB_READ (0x00)
 */
#define MSG2SSP_AP_STATUS_LCD_ON			0x1E
#define MSG2SSP_AP_STATUS_SIOP				0x1F
#define MSG2SSP_AP_STATUS_LCD_OFF			0x20

#define MSG2SSP_AP_SENSOR_FLUSH				0x21

#define MSG2SSP_AP_SENSOR_LPF				0x23
#define MSG2SSP_AP_STATUS_POW_CONNECTED		0x24
#define MSG2SSP_AP_STATUS_POW_DISCONNECTED	0x25

#define MSG2SSP_AP_SENSOR_FORMATION			0x26
#ifdef CONFIG_SENSORS_SSP_VIB_NOTIFY
#define MSG2SSP_AP_MCU_SET_MOTOR_STATUS		0x27
#endif
#define MSG2SSP_AP_MCU_SET_GYRO_CAL			0x28
#define MSG2SSP_AP_MCU_SET_ACCEL_CAL		0x29
#define MSG2SSP_AP_MCU_SET_HRM_OSC_REG		0x2A
#define MSG2SSP_AP_MCU_SET_BARO_CAL			0x2B

#define MSG2SSP_AP_READ_HRM_CON_DET_PIN		0x2D

#define MSG2SSP_AP_READ_LIB_PROTOCOL_VERSION 0x2E

#define MSG2SSP_PANEL_INFO					0x2F
#define MSG2SSP_COPR_ENABLE					0x30
#define MSG2SSP_COPR_DISABLE				0x31
#define MSG2SSP_UB_CON						0x32
#define MSG2SSP_UB_DISCON					0x33
#define MSG2SSP_AP_GET_LIGHT_CAL			0x34
#define MSG2SSP_AP_SET_LIGHT_CAL			0x39

#define MSG2SSP_INST_LIBRARY_GETVALUE_8		0x35
#define MSG2SSP_INST_LIBRARY_PUTVALUE_9		0x36


#define MSG2SSP_INST_LIBRARY_ADD			0x37
#define MSG2SSP_INST_LIBRARY_REMOVE			0x38

/* MAX_INST_VALUE highly depends on SensorhubFW */
#define MSG2SSP_MAX_INST_VALUE				0x3F

/* PANEL data */
#define MSG2SSP_GET_COPR_ROIX				0x3A
#define MSG2SSP_GET_LIGHT_DEBUG_INFO		0x3B
#define MSG2SSP_COPR_NUM_INFO				0x3C
#define MSG2SSP_GET_BARO_ID					0x3D
#define MSG2SSP_GET_BEZEL_TEST_READY        0x3E
#define MSG2SSP_SET_PMS_HYSTERESIS        	0x3F

#define MSG2SSP_AP_STATUS_HRM_SHUTDOWN      0x40
#define MSG2SSP_GET_HRM_VENDOR      		0x41

#define MSG2SSP_SET_BEZEL_TEST_ONOFF      	0x42

/* SSP_INSTRUCTION_CMD */
enum {
	REMOVE_SENSOR = 0,
	ADD_SENSOR,
	CHANGE_DELAY,
	GO_SLEEP,
	REMOVE_LIBRARY,
	ADD_LIBRARY,
	EXT_CMD = 10,
};


#define MSG2SSP_AP_STATUS_RESET			0xD5
#define MSG2SSP_HUB_STATUS_NOTI			0xD6
#define MSG2SSP_INST_LIB_NOTI			0xB4


/* SSP -> AP Instruction */
#define MSG2AP_INST_BYPASS_DATA			0x37
#define MSG2AP_INST_LIBRARY_DATA		0x01
#define MSG2AP_INST_DEBUG_DATA			0x03
#define MSG2AP_INST_BIG_DATA			0x04
#define MSG2AP_INST_META_DATA			0x05
#define MSG2AP_INST_TIME_SYNC			0x06
#define MSG2AP_INST_RESET				0x07
#define MSG2AP_INST_GYRO_CAL			0x08
#define MSG2AP_INST_MAG_CAL				0x09
#define MSG2AP_INST_ACC_ESN				0x0A
#define MSG2AP_INST_DUMP_DATA			0xDD

#define MSG2AP_INST_RESERVED			0xEE  // for hrm shutdown big data
#define MSG2AP_INST_RESERVED2			0xEF  // for wrong lib type

#endif /* __SSP_PROTOCOL_H__ */
