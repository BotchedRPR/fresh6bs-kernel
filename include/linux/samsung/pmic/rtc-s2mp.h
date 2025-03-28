/*  rtc-s2mp.h
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_MFD_SEC_RTC_H
#define __LINUX_MFD_SEC_RTC_H

/* Slave addr = 0x0C */
/* RTC Registers */
#if IS_ENABLED(CONFIG_RTC_DRV_S2MP)
#define S2MP_RTC_REG_CTRL	0x00
#define S2MP_RTC_REG_WTSR_SMPL	0x01
#define S2MP_RTC_REG_UPDATE	0x02
#define S2MP_RTC_REG_CAPSEL	0x03
#define S2MP_RTC_REG_SEC	0x04
#define S2MP_RTC_REG_MIN	0x05
#define S2MP_RTC_REG_HOUR	0x06
#define S2MP_RTC_REG_WEEK	0x07
#define S2MP_RTC_REG_DAY	0x08
#define S2MP_RTC_REG_MON	0x09
#define S2MP_RTC_REG_YEAR	0x0A
#define S2MP_RTC_REG_A0SEC	0x0B
#define S2MP_RTC_REG_A0MIN	0x0C
#define S2MP_RTC_REG_A0HOUR	0x0D
#define S2MP_RTC_REG_A0WEEK	0x0E
#define S2MP_RTC_REG_A0DAY	0x0F
#define S2MP_RTC_REG_A0MON	0x10
#define S2MP_RTC_REG_A0YEAR	0x11
#define S2MP_RTC_REG_A1SEC	0x12
#define S2MP_RTC_REG_A1MIN	0x13
#define S2MP_RTC_REG_A1HOUR	0x14
#define S2MP_RTC_REG_A1WEEK	0x15
#define S2MP_RTC_REG_A1DAY	0x16
#define S2MP_RTC_REG_A1MON	0x17
#define S2MP_RTC_REG_A1YEAR	0x18
#else
#define S2MP_RTC_REG_CTRL	0x00
#define S2MP_RTC_REG_WTSR_SMPL	0x01
#define S2MP_RTC_REG_UPDATE	0x02
#define S2MP_RTC_REG_CAPSEL	0x03
#define S2MP_RTC_REG_MSEC	0x04
#define S2MP_RTC_REG_SEC	0x05
#define S2MP_RTC_REG_MIN	0x06
#define S2MP_RTC_REG_HOUR	0x07
#define S2MP_RTC_REG_WEEK	0x08
#define S2MP_RTC_REG_DAY	0x09
#define S2MP_RTC_REG_MON	0x0A
#define S2MP_RTC_REG_YEAR	0x0B
#define S2MP_RTC_REG_A0SEC	0x0C
#define S2MP_RTC_REG_A0MIN	0x0D
#define S2MP_RTC_REG_A0HOUR	0x0E
#define S2MP_RTC_REG_A0WEEK	0x0F
#define S2MP_RTC_REG_A0DAY	0x10
#define S2MP_RTC_REG_A0MON	0x11
#define S2MP_RTC_REG_A0YEAR	0x12
#define S2MP_RTC_REG_A1SEC	0x13
#define S2MP_RTC_REG_A1MIN	0x14
#define S2MP_RTC_REG_A1HOUR	0x15
#define S2MP_RTC_REG_A1WEEK	0x16
#define S2MP_RTC_REG_A1DAY	0x17
#define S2MP_RTC_REG_A1MON	0x18
#define S2MP_RTC_REG_A1YEAR	0x19
#define S2MP_RTC_REG_OSCCTRL	0x1A
#endif
#define S2MP_RTC_REG_ETC1	0x1E

#define RTC_REG_SECURE1		(0x20)
#define RTC_REG_SECURE2		(0x21)
#define RTC_REG_SECURE3		(0x22)
#define RTC_REG_SECURE4		(0x23)
#define RTC_REG_WDT		(0x24)

/* S2MP_RTC_REG_ETC1 [3:0] fg parameter version  */
#define DATA_VER_MASK		(0xF)
#define DATA_VER_SHIFT		0
/* S2MP_RTC_REG_ETC1 [7:4] fg reset case  */
#define FG_RESET_MASK		(0xF << 4)
#define FG_RESET_SHIFT		4
#define FG_RESET_BY_JIG		(0x1 << FG_RESET_SHIFT)
#define FG_RESET_BY_TEMP	(0x2 << FG_RESET_SHIFT)

/* RTC Control Register */
#define BCD_EN_SHIFT			0
#define BCD_EN_MASK			(1 << BCD_EN_SHIFT)
#define MODEL24_SHIFT			1
#define MODEL24_MASK			(1 << MODEL24_SHIFT)
/* WTSR and SMPL Register */
#define WTSRT_SHIFT			0
#define SMPLT_SHIFT			3
#define WTSRT_MASK			(7 << WTSRT_SHIFT)
#define SMPLT_MASK			(7 << SMPLT_SHIFT)
#define WTSR_EN_SHIFT			6
#define SMPL_EN_SHIFT			7
#define WTSR_EN_MASK			(1 << WTSR_EN_SHIFT)
#define SMPL_EN_MASK			(1 << SMPL_EN_SHIFT)
#define SUB_SMPL_EN_SHIFT		3
#define SUB_SMPL_EN_MASK		(1 << SUB_SMPL_EN_SHIFT)
/* RTC Update Register */
#define RTC_RUDR_SHIFT			0
#define RTC_RUDR_MASK			(1 << RTC_RUDR_SHIFT)
#define RTC_AUDR_SHIFT_REV		4
#define RTC_AUDR_MASK_REV		(1 << RTC_AUDR_SHIFT_REV)
#if IS_ENABLED(CONFIG_RTC_BOOT_ALARM)
#define RTC_WAKE_SHIFT			3
#define RTC_WAKE_MASK			(1 << RTC_WAKE_SHIFT)
#endif
#define RTC_FREEZE_SHIFT		2
#define RTC_FREEZE_MASK			(1 << RTC_FREEZE_SHIFT)
#define RTC_WUDR_SHIFT_REV		1
#define RTC_WUDR_MASK_REV		(1 << RTC_WUDR_SHIFT_REV)
/* RTC HOUR Register */
#define HOUR_PM_SHIFT			6
#define HOUR_PM_MASK			(1 << HOUR_PM_SHIFT)
/* RTC Alarm Enable */
#define ALARM_ENABLE_SHIFT		7
#define ALARM_ENABLE_MASK		(1 << ALARM_ENABLE_SHIFT)
/* PMIC STATUS2 Register */
#define RTCA0E				(1<<2)
#define RTCA1E				(1<<1)

#define WTSR_TIMER_BITS(v)		(((v) << WTSRT_SHIFT) & WTSRT_MASK)
#define SMPL_TIMER_BITS(v)		(((v) << SMPLT_SHIFT) & SMPLT_MASK)

/* RTC Optimize */
#define OSC_BIAS_UP_SHIFT		2
#define OSC_BIAS_UP_MASK		(1 << OSC_BIAS_UP_SHIFT)
#define CAP_SEL_SHIFT			0
#define CAP_SEL_MASK			(0x03 << CAP_SEL_SHIFT)
#define OSC_XIN_SHIFT			5
#define OSC_XIN_MASK			(0x07 << OSC_XIN_SHIFT)
#define OSC_XOUT_SHIFT			2
#define OSC_XOUT_MASK			(0x07 << OSC_XOUT_SHIFT)

/* RTC SECURE */
#define RTC_BOOTING             (3)
#define RTC_SUSPEND             (4)
#define RTC_RESUME              (5)
#define RTC_SHUTDOWN            (6)
#define RTC_POWEROFF            (7)

/* RTC_WDT */
#define WDT_EN			(1 << 0)
#define WDT_TIME_SHIFT		1
#define WDT_TIME_MASK		(3 << WDT_TIME_SHIFT)
#define WDT_TIME_BITS(v)	(((v) << WDT_TIME_SHIFT) & WDT_TIME_MASK)

extern int secure_debug_set_config(u8 addr, u8 config);
extern int secure_debug_read(u8 addr, u8 bit);
extern void secure_debug_write(u8 addr, u8 bit, u8 val);
extern void secure_debug_clear(void);

/* RTC Counter Register offsets */
#if IS_ENABLED(CONFIG_RTC_DRV_S2MP)
enum {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	NR_RTC_CNT_REGS,
};
#else
enum {
/*	RTC_MSEC = 0, */
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	NR_RTC_CNT_REGS,
};

#endif
#endif /*  __LINUX_MFD_SEC_RTC_H */
