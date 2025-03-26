/*
 * cps4019_core.h
 * Samsung CPS4019 Core Header
 *
 * Copyright (C) 2023 Samsung Electronics, Inc.
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

#ifndef __CPS4019_CORE_H
#define __CPS4019_CORE_H __FILE__

#include <linux/i2c.h>
#include <linux/types.h>

#define CPS4019_FW_MAJOR_REV_L_REG		0x0004
#define CPS4019_FW_MAJOR_REV_H_REG		0x0005
#define CPS4019_FW_MINOR_REV_L_REG		0x0006
#define CPS4019_FW_MINOR_REV_H_REG		0x0007

/* INT & STATUS L */
#define CPS4019_STAT_VOUT_MASK			(1 << 7)
#define CPS4019_STAT_VRECT_MASK			(1 << 6)
#define CPS4019_OP_MODE_MASK			(1 << 5)
#define CPS4019_OVER_VOLTAGE_MASK		(1 << 4)
#define CPS4019_OVER_CURRENT_MASK		(1 << 3)
#define CPS4019_OVER_TEMP_MASK			(1 << 2)
#define CPS4019_UNDER_VOLTAGE_MASK		(1 << 1)
#define CPS4019_TX_DATA_RECV_MASK		(1)
/* INT & STATUS H */
#define CPS4019_AC_MISSING_DET_MASK		(1 << 7)
#define CPS4019_DATA_STORE_MASK			(1 << 6)
#define CPS4019_GP2_SAMPLE_DONE_MASK	(1 << 5)
#define CPS4019_ADT_SENT_MASK			(1 << 3)
#define CPS4019_ADT_RECV_MASK			(1 << 2)
#define CPS4019_ADT_ERROR_MASK			(1 << 1)
#define CPS4019_ADT_DATA_MASK			(0x7 << 1)
#define CPS4019_APP2_REQUEST_MASK		(1)

#define CPS4019_STATUS_L_REG			0x0020
#define CPS4019_STATUS_H_REG			0x0021
#define CPS4019_INT_L_REG				0x0022
#define CPS4019_INT_H_REG				0x0023
#define CPS4019_INT_ENABLE_L_REG		0x0024
#define CPS4019_INT_ENABLE_H_REG		0x0025
#define CPS4019_INT_CLEAR_L_REG			0x0026
#define CPS4019_INT_CLEAR_H_REG			0x0027

#define CPS4019_SYS_OP_MODE_REG			0x0028
#define CPS4019_RX_MODE_SHIFT			5
#define CPS4019_RX_MODE_MASK			(0x7 << CPS4019_RX_MODE_SHIFT)
#define CPS4019_RX_MODE_AC_MISSING		0
#define CPS4019_RX_MODE_WPC_BPP			1

#define CPS4019_CHG_STATUS_REG			0x002C
#define CPS4019_EPT_REG					0x002D
/* End of Power Transfer Register, EPT (0x3B) (RX only) */
#define CPS4019_EPT_UNKNOWN				0
#define CPS4019_EPT_END_OF_CHG			1
#define CPS4019_EPT_INT_FAULT			2
#define CPS4019_EPT_OVER_TEMP			3
#define CPS4019_EPT_OVER_VOL			4
#define CPS4019_EPT_OVER_CURR			5
#define CPS4019_EPT_BATT_FAIL			6
#define CPS4019_EPT_RECONFIG			7

#define CPS4019_VOUT_SET_L_REG			0x0030
#define CPS4019_VOUT_SET_H_REG			0x0031
#define CPS4019_VRECT_ADJ_REG			0x0032
#define CPS4019_ILIM_SET_REG			0x0033
#define CPS4019_ADC_VOUT_L_REG			0x0034
#define CPS4019_ADC_VOUT_H_REG			0x0035
#define CPS4019_ADC_VRECT_L_REG			0x0036
#define CPS4019_ADC_VRECT_H_REG			0x0037
#define CPS4019_ADC_IOUT_L_REG			0x0038
#define CPS4019_ADC_IOUT_H_REG			0x0039
#define CPS4019_ADC_VSYS_L_REG			0x003A
#define CPS4019_ADC_VSYS_H_REG			0x003B
#define CPS4019_ADC_DIE_TEMP_L_REG		0x003C
#define CPS4019_ADC_DIE_TEMP_H_REG		0x003D
#define CPS4019_ADC_OP_FREQ_L_REG		0x003E
#define CPS4019_ADC_OP_FREQ_H_REG		0x003F
#define CPS4019_ADC_PING_FREQ_L_REG		0x0040
#define CPS4019_ADC_PING_FREQ_H_REG		0x0041

#define CPS4019_DC_VOUT_HDR_REG			0x0044
#define CPS4019_FX_VOUT_HDR_REG			0x0045
#define CPS4019_DC_VOUT_HIGHEST_L_REG	0x0046
#define CPS4019_DC_VOUT_HIGHEST_H_REG	0x0047

#define CPS4019_COMMAND_L_REG			0x0050
#define CPS4019_CMD_WATCHDOG_EN_MASK	(1 << 7)
#define CPS4019_CMD_WATCHDOG_RST_MASK	(1 << 6)
#define CPS4019_CMD_CLEAR_INT_MASK		(1 << 5)
#define CPS4019_CMD_SEND_CHG_STS_MASK	(1 << 4)
#define CPS4019_CMD_SEND_EOP_MASK		(1 << 3)
#define CPS4019_CMD_MCU_RST_MASK		(1 << 2)
#define CPS4019_CMD_TOGGLE_LDO_MASK		(1 << 1)
#define CPS4019_CMD_SEND_RX_DATA_MASK	(1)

#define CPS4019_COMMAND_H_REG				0x0051
#define CPS4019_CMD_SEND_ADT_MASK			(1 << 2)
#define CPS4019_CMD_SAMPLE_CALL_VOLT_MASK	(1 << 1)
#define CPS4019_CMD_SAMPLE_LOAD_VOLT_MASK	(1)

#define CPS4019_PPP_HEADER_REG			0x0052
#define CPS4019_HEADER_ESS				0x01	/* END SIGNAL STRENGTH */
#define CPS4019_HEADER_EPT				0x02	/* END POWER TRANSFER */
#define CPS4019_HEADER_ECE				0x03	/* END CONTROL ERROR */
#define CPS4019_HEADER_ERP				0x04	/* END RECEIVED POWER */
#define CPS4019_HEADER_ECS				0x05	/* POWER CONTROL HOLD */
#define CPS4019_HEADER_PCH_OFF			0x06
#define CPS4019_HEADER_AFC_CONF			0x28
#define CPS4019_HEADER_CONFIGURATION	0x51
#define CPS4019_HEADER_IDENTIFICATION	0x71
#define CPS4019_HEADER_EXTENDED_IDENT	0x81

#define CPS4019_RX_DATA_COM_REG			0x0053
#define CPS4019_RX_DATA_VALUE_REG		0x0054
#define CPS4019_RX_DATA_MAX				7

#define CPS4019_TX_DATA_COM_REG			0x005B
#define CPS4019_TX_DATA_VALUE_REG		0x005C
#define CPS4019_TX_DATA_MAX				3

#define CPS4019_RECT_MODE_REG			0x006A

#define CPS4019_VOUT_MODE_REG			0x006B
#define CPS4019_VOUT_MODE_SHIFT			0
#define CPS4019_VOUT_MODE_MASK			(0x7 << CPS4019_VOUT_MODE_SHIFT)

#define CPS4019_VBAT_HEADROOM_REG		0x006D

#define CPS4019_VOUT_LOWEST_L_REG		0x006F
#define CPS4019_VOUT_LOWEST_H_REG		0x0070

#define CPS4019_DATA_MODE_REG			0x0076
#define CPS4019_DATA_STORE_EN_MASK		(1 << 7)

#define CPS4019_BT_VOUT_HDR_REG			0x007E
#define CPS4019_I_CHG_SET_REG			0x007F

#define CPS4019_ADT_TYPE_REG			0x0200
#define CPS4019_ADT_TYPE_SHIFT			3
enum {
	CPS4019_ADT_TYPE_END		= 0,
	CPS4019_ADT_TYPE_GENERAL,
	CPS4019_ADT_TYPE_AUTH,

	CPS4019_ADT_TYPE_RESET		= 0x12
};

#define CPS4019_ADT_DATA_SIZE_REG		0x0201
#define CPS4019_ADT_DATA_L_REG			0x0202
#define CPS4019_ADT_DATA_H_REG			0x0265
#define CPS4019_ADT_MAX_SIZE			(CPS4019_ADT_DATA_H_REG - CPS4019_ADT_DATA_L_REG)

/* i2c interface */
int cps4019_reg_read(struct i2c_client *client, u16 reg, u8 *val);
int cps4019_reg_bulk_read(struct i2c_client *client, u16 reg, u8 *val, u32 size);
int cps4019_reg_write(struct i2c_client *client, u16 reg, u8 val);
int cps4019_reg_bulk_write(struct i2c_client *client, u16 reg, u8 *val, u32 size);
int cps4019_reg_update(struct i2c_client *client, u16 reg, u8 val, u8 mask);

#define cps4019_set_byte(i2c, reg, val)		\
	cps4019_reg_write(i2c, reg, val)

#define cps4019_set_short(i2c, reg, val)	\
	cps4019_reg_bulk_write(i2c, reg, (u8 *)&val, 2)

#define cps4019_set_irq_en_l(i2c, mask, state) \
	cps4019_reg_update(i2c, CPS4019_INT_ENABLE_L_REG, ((state) ? mask : 0), mask)

#define cps4019_set_irq_en_h(i2c, mask, state) \
	cps4019_reg_update(i2c, CPS4019_INT_ENABLE_H_REG, ((state) ? mask : 0), mask)

/* adc interface */
enum {
	CPS4019_ADC_VOUT = 0,
	CPS4019_ADC_VRECT,
	CPS4019_ADC_IOUT,
	CPS4019_ADC_VSYS,	/* VBAT */
	CPS4019_ADC_DIE_TEMP,
	CPS4019_ADC_OP_FRQ,
	CPS4019_ADC_PING_FRQ,
};

int cps4019_get_adc(struct i2c_client *client, int adc_type);

#endif /* __CPS4019_CORE_H */
