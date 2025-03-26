/*
 * sb_vote_event.h
 * Samsung Mobile Battery Vote Event Header
 *
 * Copyright (C) 2022 Samsung Electronics, Inc.
 *
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

#ifndef __SB_VOTE_EVENT_H
#define __SB_VOTE_EVENT_H __FILE__

/* VOTER NAME LIST */
#define VN_FCC		"FCC"
#define VN_ICL		"ICL"
#define VN_TOPOFF	"TOPOFF"
#define VN_FV		"FV"
#define VN_CHG_EN	"CHGEN"
#define VN_IN2BAT	"IN2BAT"
#define VN_WPC_EN	"WPC-EN"	/* EN Pin */
#define VN_WPC_PHM	"WPC-PHM"	/* PHM */
#define VN_WPC_LDO	"WPC-LDO"	/* LDO */
#define VN_WPC_APM	"WPC-APM"	/* AP Mode */
#define VN_WPC_VM	"WPC-VM"	/* VOUT MODE */
#define VN_WPC_DC_HDR		"WPC-DC-HDR"		/* DC VOUT HEADROOM */
#define VN_WPC_ICHG			"WPC-ICHG"			/* DC ICHG */
#define VN_WPC_DC_HIGH_VOUT	"WPC-DC-HIGH-VOUT"	/* DC HIGH VOUT */
#define VN_WPC_LOW_VOUT		"WPC-LOW-VOUT"		/* BT LOW VOUT */
#define VN_WPC_BT_VBAT_HDR	"WPC-BT-VBAT-HDR"	/* BT VBAT HEADROOM */
#define VN_WPC_BT_HDR		"WPC-BT-HDR"		/* BT VOUT HEADROOM */
#define VN_WPC_VOUT			"WPC-VOUT"			/* FX VOUT */
#define VN_WPC_FX_HDR		"WPC-FX-HDR"		/* FX VOUT HEADROOM */
#define VN_WPC_VOUT_PHM		"WPC-VOUT-PHM"		/* VOUT with PHM */
#define VN_WPC_VOUT_OFF		"WPC-VOUT-OFF"		/* VOUT OFF */

#define FOREACH_VOTER(GENERATE)	{ \
	GENERATE(VOTER_CABLE), \
	GENERATE(VOTER_TEST), \
	GENERATE(VOTER_WPC_CONTROL), \
	GENERATE(VOTER_S2MPW03_CONTROL), \
	GENERATE(VOTER_CTRL_0), \
	GENERATE(VOTER_CTRL_1), \
	GENERATE(VOTER_CTRL_2), \
	GENERATE(VOTER_CTRL_3), \
	GENERATE(VOTER_CTRL_4), \
	GENERATE(VOTER_CTRL_5), \
	GENERATE(VOTER_CTRL_6), \
	GENERATE(VOTER_CTRL_7), \
	GENERATE(VOTER_CTRL_8), \
	GENERATE(VOTER_CTRL_9), \
	GENERATE(VOTER_STARTUP), \
	GENERATE(VOTER_AGING_STEP), \
	GENERATE(VOTER_STORE_MODE), \
	GENERATE(VOTER_SWELLING), \
	GENERATE(VOTER_SYSOVLO), \
	GENERATE(VOTER_VBUS_OVP), \
	GENERATE(VOTER_FULL_CHARGE), \
	GENERATE(VOTER_TIME_EXPIRED), \
	GENERATE(VOTER_TOPOFF_CHANGE), \
	GENERATE(VOTER_SLATE), \
	GENERATE(VOTER_FOTA_DOWNLOAD), \
	GENERATE(VOTER_VBAT_OVP), \
	GENERATE(VOTER_WPC_DET), \
	GENERATE(VOTER_WPC_ID_CHK), \
	GENERATE(VOTER_WPC_ALIGN_CHK), \
	GENERATE(VOTER_WPC_TX_PHM), \
	GENERATE(VOTER_WPC_CS100), \
	GENERATE(VOTER_WPC_FW), \
	GENERATE(VOTER_WPC_HALF_BRIDGE), \
	GENERATE(VOTER_WPC_MODE), \
	GENERATE(VOTER_WPC_AUTH), \
	GENERATE(VOTER_WPC_RX_PWR_CHK), \
	GENERATE(VOTER_WPC_VM_PHM), \
	GENERATE(VOTER_WPC_ERROR), \
	GENERATE(VOTER_MAX) \
}

#define FOREACH_CHARGE_MODE(GENERATE)	{ \
	GENERATE(SB_CHG_MODE_BUCK_OFF), \
	GENERATE(SB_CHG_MODE_CHARGING_OFF), \
	GENERATE(SB_CHG_MODE_CHARGING), \
	GENERATE(SB_CHG_MODE_MAX) \
}

#define GENERATE_ENUM(ENUM) ENUM
#define GENERATE_STRING(STRING)	#STRING

enum VOTER_ENUM
FOREACH_VOTER(GENERATE_ENUM);

extern const char *sb_voter_name[];

enum CHARGE_MODE_ENUM
FOREACH_CHARGE_MODE(GENERATE_ENUM);

extern const char *sb_chg_mode_name[];

enum WPC_PHM_SET {
	WPC_PHM_OFF	= 0,
	WPC_PHM_ON,
	WPC_PHM_EPT_ON,
	WPC_PHM_FEPT_ON,
};

enum WPC_LDO_SET {
	WPC_LDO_ON = 0,
	WPC_LDO_OFF,
};

enum WPC_APM_SET {
	WPC_APM_NONE = 0,
	WPC_APM_NORMAL,
	WPC_APM_STORE,
	WPC_APM_BOOT,
	WPC_APM_WAKE_UP,
	WPC_APM_VOUT_ON,
	WPC_APM_VOUT_OFF,
	WPC_APM_TEST,
};

/* AP MODE	VOUT		VRECT
 * 0x00	:	5V			5.5V
 * 0x01	:	5V			5.35V
 * 0x02	:	VBAT + 0.3V	VOUT + 0.1V
 * 0x03	:	VBAT + 0.3V	VOUT + 0.1V
 * 0x04	:	VBAT + 0.3V	VOUT + 0.1V
 * 0x05	:	VBAT + 0.3V	VOUT + 0.1V
 * 0x06	:	4.8V		5.16V
 * 0x07	:	4.8V		5.28V
 * 0x08	:	4.9V		5.26V
 * 0x09	:	4.9V		5.38V
 * 0x0A	:	4.9V		5.46V
 * 0x0B	:	4.9V		5.55V
 * 0x0C	:	5.1V		5.5V
 * 0x0D	:	5.1V		5.6V
 * 0x0E	:	5.1V		5.7V
 * 0x0F	:	5.1V		5.8V
 * 0x10	:	4.6V		4.7V
 * 0x11	:	4.7V		4.8V
 * 0x12	:	WAKE UP
 * 0x13	:	VOUT OFF
 * 0xF0	:	5V
 */
enum sb_wrl_ap_mode {
	SB_WRL_AP_MODE_HOLD		= 0,
	SB_WRL_AP_MODE_BOOT,

	SB_WRL_AP_MODE_BATTERY,

	SB_WRL_AP_MODE_CC_CV	= 0x6,
	SB_WRL_AP_MODE_INCOMP_CC_CV,

	SB_WRL_AP_MODE_CC		= 0x10,
	SB_WRL_AP_MODE_INCOMP_CC,
	SB_WRL_AP_MODE_WAKEUP,
	SB_WRL_AP_MODE_INCOMP_PHP,

	SB_WRL_AP_MODE_MEASURE	= 0xF0
};

/*
 * Vout Mode Table
 * Value(Bit)	|		Vout			|		Vrect
 * b000					5V					5.5V
 * b001			Vsys + ICHG * Ron_chg		Vout + DC Vout Headroom
 * b010			Vsys + VBAT Headroom		Vout + BT Vout Headroom
 * b011					vout				vout + FV Vout Headroom
 * b100				Vout Off(LDO off)		-
 * b101				5V -> Prev				5.5V -> Prev
 * b110					5V					5.5V
 * b111						AP Mode Table
 */
enum sb_wrl_vm_type {
	SB_WRL_VM_INIT	= 0,
	SB_WRL_VM_DC,		/* Direct Charging */
	SB_WRL_VM_BT,		/* Battery Tracking */
	SB_WRL_VM_FX,		/* Fixed Vout */
	SB_WRL_VM_OFF,		/* Vout Off */
	SB_WRL_VM_WAKE,		/* Wake Up */
	SB_WRL_VM_CM,		/* Current Measure */
	SB_WRL_VM_APM,		/* AP Mode */
	SB_WRL_VM_MAX
};

static inline const char *get_vout_mode_str(int mode)
{
	switch (mode) {
	case SB_WRL_VM_INIT:
		return "Init";
	case SB_WRL_VM_DC:
		return "DC";
	case SB_WRL_VM_BT:
		return "BT";
	case SB_WRL_VM_FX:
		return "FX";
	case SB_WRL_VM_OFF:
		return "Off";
	case SB_WRL_VM_WAKE:
		return "Wake";
	}

	return "Reserved";
}

#endif /* __SB_VOTE_EVENT_H */
