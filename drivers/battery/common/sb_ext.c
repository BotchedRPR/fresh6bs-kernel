/*
 *  sb_ext.c
 *  Samsung Mobile Battery Extern Module
 *
 *  Copyright (C) 2022 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/battery/common/sb_psy.h>
#include <linux/battery/common/sb_ext.h>
#include <linux/battery/common/sb_type.h>
#include <linux/battery/common/sb_event.h>
#include <linux/battery/common/sb_vote_event.h>

const char *sb_voter_name[] = FOREACH_VOTER(GENERATE_STRING);
EXPORT_SYMBOL(sb_voter_name);
const char *sb_chg_mode_name[] = FOREACH_CHARGE_MODE(GENERATE_STRING);
EXPORT_SYMBOL(sb_chg_mode_name);

static int __read_mostly misc_test;
module_param(misc_test, int, 0444);
int sb_get_misc_test(void)
{
	return misc_test;
}
EXPORT_SYMBOL(sb_get_misc_test);

static int __read_mostly factory_mode;
module_param(factory_mode, int, 0444);
int sb_get_factory_mode(void)
{
	return factory_mode;
}
EXPORT_SYMBOL(sb_get_factory_mode);

static unsigned int __read_mostly boot_mode;
module_param(boot_mode, uint, 0444);
int sb_get_boot_mode(void)
{
	return boot_mode;
}
EXPORT_SYMBOL(sb_get_boot_mode);

static unsigned int __read_mostly lpcharge;
module_param(lpcharge, uint, 0444);
int sb_get_lpcharge(void)
{
	return lpcharge;
}
EXPORT_SYMBOL(sb_get_lpcharge);

static unsigned int __read_mostly wirelessd;
module_param(wirelessd, uint, 0444);
int sb_get_wirelessd(void)
{
	return wirelessd;
}
EXPORT_SYMBOL(sb_get_wirelessd);

static int __read_mostly fg_reset;
module_param(fg_reset, int, 0444);
int sb_get_fg_reset(void)
{
	return fg_reset;
}
EXPORT_SYMBOL(sb_get_fg_reset);

static char * __read_mostly sales_code;
module_param(sales_code, charp, 0444);
const char *sb_get_sales_code(void)
{
	return sales_code;
}
EXPORT_SYMBOL(sb_get_sales_code);

const char *sb_get_ct_str(int cable_type)
{
	switch (cable_type) {
	case SB_CBL_NONE:
		return "NONE";
	case SB_CBL_TA:
		return "TA";
	case SB_CBL_USB:
		return "USB";
	case SB_CBL_UARTOFF:
		return "UARTOFF";
	case SB_CBL_WRL:
		return "WRL";
	case SB_CBL_INBOX_WRL:
		return "INBOX_WRL";
	case SB_CBL_D2D_WRL:
		return "D2D_WRL";
	case SB_CBL_INCOMP_WRL:
		return "INCOMP_WRL";
	case SB_CBL_ERR_WRL:
		return "ERR_WRL";
	case SB_CBL_FM_WRL:
		return "FM_WRL";
	case SB_CBL_BPACK_WRL:
		return "BPACK_WRL";
	case SB_CBL_AUTH_WRL_2W:
		return "AUTH_WRL_2W";
	case SB_CBL_AUTH_WRL_5W:
		return "AUTH_WRL_5W";
	case SB_CBL_ERR_AUTH_WRL:
		return "ERR_AUTH_WRL";
	}
	return "UNKNOWN";
}
EXPORT_SYMBOL(sb_get_ct_str);

const char *sb_get_cm_str(unsigned int charging_mode)
{
	switch (charging_mode) {
	case SEC_BATTERY_CHARGING_NONE:
		return "None";
	case SEC_BATTERY_CHARGING_1ST:
		return "Normal";
	case SEC_BATTERY_CHARGING_2ND:
		return "Additional";
	case SEC_BATTERY_CHARGING_RECHARGING:
		return "Re-Charging";
	}
	return "UNKNOWN";
}
EXPORT_SYMBOL(sb_get_cm_str);

const char *sb_get_bst_str(int status)
{
	switch (status) {
	case POWER_SUPPLY_STATUS_UNKNOWN:
		return "Unknown";
	case POWER_SUPPLY_STATUS_CHARGING:
		return "Chg";
	case POWER_SUPPLY_STATUS_DISCHARGING:
		return "DisChg";
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		return "NotChg";
	case POWER_SUPPLY_STATUS_FULL:
		return "Full";
	}
	return "UNKNOWN";
}
EXPORT_SYMBOL(sb_get_bst_str);

const char *sb_get_hl_str(int health)
{
	switch (health) {
	case POWER_SUPPLY_HEALTH_GOOD:
		return "Good";
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		return "Overheat";
	case POWER_SUPPLY_HEALTH_WARM:
		return "Warm";
	case POWER_SUPPLY_HEALTH_DEAD:
		return "Dead";
	case POWER_SUPPLY_HEALTH_OVERVOLTAGE:
		return "OverVoltage";
	case POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
		return "UnspecFailure";
	case POWER_SUPPLY_HEALTH_COLD:
		return "Cold";
	case POWER_SUPPLY_HEALTH_COOL:
		return "Cool";
	case POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE:
		return "WatchdogTimerExpire";
	case POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE:
		return "SafetyTimerExpire";
	case POWER_SUPPLY_EXT_HEALTH_UNDERVOLTAGE:
		return "UnderVoltage";
	case POWER_SUPPLY_EXT_HEALTH_OVERHEATLIMIT:
		return "OverheatLimit";
	case POWER_SUPPLY_EXT_HEALTH_VSYS_OVP:
		return "VsysOVP";
	case POWER_SUPPLY_EXT_HEALTH_VBAT_OVP:
		return "VbatOVP";
	}
	return "UNKNOWN";
}
EXPORT_SYMBOL(sb_get_hl_str);

const char *sb_get_tz_str(int thermal_zone)
{
	switch (thermal_zone) {
	case BAT_THERMAL_NORMAL:
		return "Normal";
	case BAT_THERMAL_COLD:
		return "COLD";
	case BAT_THERMAL_COOL3:
		return "COOL3";
	case BAT_THERMAL_COOL2:
		return "COOL2";
	case BAT_THERMAL_COOL1:
		return "COOL1";
	case BAT_THERMAL_WARM:
		return "WARM";
	case BAT_THERMAL_OVERHEAT:
		return "OVERHEAT";
	case BAT_THERMAL_OVERHEATLIMIT:
		return "OVERHEATLIM";
	}
	return "UNKNOWN";
}
EXPORT_SYMBOL(sb_get_tz_str);

static int __init sb_ext_init(void)
{
	pr_info("%s:\n", __func__);
	return 0;
}
module_init(sb_ext_init);

static void __exit sb_ext_exit(void)
{
}
module_exit(sb_ext_exit);

MODULE_DESCRIPTION("Samsung Battery Ext");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
