/*
 * sec_battery_thermal.c
 * Samsung Mobile Battery Driver
 *
 * Copyright (C) 2021 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/power_supply.h>

#include <linux/battery/sb_vote.h>

#include <linux/battery/common/sb_ext.h>
#include <linux/battery/common/sb_type.h>
#include <linux/battery/common/sb_event.h>
#include <linux/battery/common/sb_vote_event.h>
#include <linux/battery/wireless/sb_wrl.h>

#include <linux/battery/module/sb_adc.h>
#include <linux/battery/module/sb_cisd.h>

#include "sec_battery.h"
#include "sec_battery_type.h"

#define THERMAL_HYSTERESIS_2	19
#define THERMAL_HYSTERESIS_3	29
#define THERMAL_HYSTERESIS_5	49

void sec_bat_set_threshold(struct sec_battery_info *battery)
{
	if (is_wired_type(battery->cable_type)) {
		battery->cold_cool3_thresh = battery->pdata->wire_cold_cool3_thresh;
		battery->cool3_cool2_thresh = battery->pdata->wire_cool3_cool2_thresh;
		battery->cool2_cool1_thresh = battery->pdata->wire_cool2_cool1_thresh;
		battery->cool1_normal_thresh = battery->pdata->wire_cool1_normal_thresh;
		battery->normal_warm_thresh = battery->pdata->wire_normal_warm_thresh;
		battery->warm_overheat_thresh = battery->pdata->wire_warm_overheat_thresh;
	} else {
		battery->cold_cool3_thresh = battery->pdata->wireless_cold_cool3_thresh;
		battery->cool3_cool2_thresh = battery->pdata->wireless_cool3_cool2_thresh;
		battery->cool2_cool1_thresh = battery->pdata->wireless_cool2_cool1_thresh;
		battery->cool1_normal_thresh = battery->pdata->wireless_cool1_normal_thresh;
		battery->normal_warm_thresh = battery->pdata->wireless_normal_warm_thresh;
		battery->warm_overheat_thresh = battery->pdata->wireless_warm_overheat_thresh;
	}

	switch (battery->thermal_zone) {
	case BAT_THERMAL_OVERHEAT:
		battery->warm_overheat_thresh = battery->pdata->warm_recov_thresh;
		battery->normal_warm_thresh = battery->pdata->warm_recov_thresh;
		break;
	case BAT_THERMAL_WARM:
		if (battery->cable_type == SB_CBL_D2D_WRL)
			battery->normal_warm_thresh = battery->pdata->warm_recov_thresh - 10;
		else
			battery->normal_warm_thresh = battery->pdata->warm_recov_thresh;
		break;
	case BAT_THERMAL_COOL1:
		battery->cool1_normal_thresh += THERMAL_HYSTERESIS_2;
		break;
	case BAT_THERMAL_COOL2:
		battery->cool2_cool1_thresh += THERMAL_HYSTERESIS_2;
		battery->cool1_normal_thresh += THERMAL_HYSTERESIS_2;
		break;
	case BAT_THERMAL_COOL3:
		battery->cool3_cool2_thresh += THERMAL_HYSTERESIS_2;
		battery->cool2_cool1_thresh += THERMAL_HYSTERESIS_2;
		battery->cool1_normal_thresh += THERMAL_HYSTERESIS_2;
		break;
	case BAT_THERMAL_COLD:
		battery->cold_cool3_thresh += THERMAL_HYSTERESIS_2;
		battery->cool3_cool2_thresh += THERMAL_HYSTERESIS_2;
		battery->cool2_cool1_thresh += THERMAL_HYSTERESIS_2;
		battery->cool1_normal_thresh += THERMAL_HYSTERESIS_2;
		break;
	case BAT_THERMAL_NORMAL:
	default:
		break;
	}
}
EXPORT_SYMBOL(sec_bat_set_threshold);

void sec_bat_thermal_check(struct sec_battery_info *battery)
{
	int pre_thermal_zone = battery->thermal_zone;
	int bat_thm;

	sb_adc_get_valuef(BATT_TEMP, &bat_thm);
	pr_err("%s: co_c3: %d, c3_c2: %d, c2_c1: %d, c1_no: %d, no_wa: %d, wa_ov: %d, tz(%s)\n", __func__,
			battery->cold_cool3_thresh, battery->cool3_cool2_thresh, battery->cool2_cool1_thresh,
			battery->cool1_normal_thresh, battery->normal_warm_thresh, battery->warm_overheat_thresh,
			sb_get_tz_str(battery->thermal_zone));

	if (battery->status == POWER_SUPPLY_STATUS_DISCHARGING ||
		battery->skip_swelling) {
		battery->health_change = false;
		pr_debug("%s: DISCHARGING or 15 test mode. stop thermal check\n", __func__);
		battery->thermal_zone = BAT_THERMAL_NORMAL;
		sb_vote_set(battery->topoff_vote, VOTER_SWELLING, false, 0);
		sb_vote_set(battery->fcc_vote, VOTER_SWELLING, false, 0);
		sb_vote_set(battery->fv_vote, VOTER_SWELLING, false, 0);
		sb_vote_set(battery->chgen_vote, VOTER_SWELLING, false, 0);
		sb_wrl_set_vm_phm(VOTER_SWELLING, false, WPC_PHM_OFF);
		sec_bat_set_current_event(battery, 0, SEC_BAT_CURRENT_EVENT_SWELLING_MODE);
		sec_bat_set_threshold(battery);
		return;
	}

	if (battery->pdata->temp_check_type == SEC_BATTERY_TEMP_CHECK_NONE) {
		pr_err("%s: BAT_THM, Invalid Temp Check Type\n", __func__);
		return;
	}

	/* COLD - COOL3 - COOL2 - COOL1 - NORMAL - WARM - OVERHEAT - OVERHEATLIMIT*/
	/* temporary setting for usb cable case*/
	if (bat_thm >= battery->normal_warm_thresh) {
		if (bat_thm >= battery->warm_overheat_thresh)
			battery->thermal_zone = BAT_THERMAL_OVERHEAT;
		else
			battery->thermal_zone = BAT_THERMAL_WARM;
	} else if (bat_thm <= battery->cool1_normal_thresh) {
		if (bat_thm <= battery->cold_cool3_thresh)
			battery->thermal_zone = BAT_THERMAL_COLD;
		else if (bat_thm <= battery->cool3_cool2_thresh)
			battery->thermal_zone = BAT_THERMAL_COOL3;
		else if (bat_thm <= battery->cool2_cool1_thresh)
			battery->thermal_zone = BAT_THERMAL_COOL2;
		else
			battery->thermal_zone = BAT_THERMAL_COOL1;
	} else {
		battery->thermal_zone = BAT_THERMAL_NORMAL;
	}

	if (pre_thermal_zone != battery->thermal_zone) {
		battery->bat_thm_count++;

		if (battery->bat_thm_count < battery->pdata->temp_check_count) {
			pr_info("%s : bat_thm_count %d/%d\n", __func__,
					battery->bat_thm_count, battery->pdata->temp_check_count);
			battery->thermal_zone = pre_thermal_zone;
			return;
		}

		pr_info("%s: thermal zone update (%s -> %s), bat_thm(%d)\n", __func__,
				sb_get_tz_str(pre_thermal_zone),
				sb_get_tz_str(battery->thermal_zone), bat_thm);
		battery->health_change = true;
		battery->bat_thm_count = 0;

		sec_bat_set_threshold(battery);
		sec_bat_set_current_event(battery, 0, SEC_BAT_CURRENT_EVENT_SWELLING_MODE);

		switch (battery->thermal_zone) {
		case BAT_THERMAL_OVERHEAT:
			sb_wrl_set_vm_phm(VOTER_SWELLING, true, WPC_PHM_EPT_ON);
			sb_vote_set(battery->chgen_vote, VOTER_SWELLING, true, SB_CHG_MODE_CHARGING_OFF);

			battery->health = POWER_SUPPLY_HEALTH_OVERHEAT;
			sec_bat_set_charging_status(battery, POWER_SUPPLY_STATUS_NOT_CHARGING);
			sb_cisd_count(2, UNSAFETY_TEMP, UNSAFETY_TEMP_D);
			sec_bat_set_current_event(battery, SEC_BAT_CURRENT_EVENT_HIGH_TEMP_SWELLING,
				SEC_BAT_CURRENT_EVENT_SWELLING_MODE);
			break;
		case BAT_THERMAL_WARM:
			sb_wrl_set_vm_phm(VOTER_SWELLING, true, WPC_PHM_ON);
			sb_vote_set(battery->chgen_vote, VOTER_SWELLING, true, SB_CHG_MODE_CHARGING_OFF);

			sb_vote_set(battery->fv_vote, VOTER_SWELLING, true, battery->pdata->high_temp_float);
			sb_vote_set(battery->topoff_vote, VOTER_SWELLING, true, battery->pdata->high_temp_topoff);

			sb_cisd_count(2, HIGH_SWELLING_CNT, HIGH_SWELLING_CNT_D);
			battery->health = POWER_SUPPLY_HEALTH_GOOD;
			sec_bat_set_current_event(battery, SEC_BAT_CURRENT_EVENT_HIGH_TEMP_SWELLING,
				SEC_BAT_CURRENT_EVENT_SWELLING_MODE);
			break;
		case BAT_THERMAL_COOL1:
			if (is_wireless_type(battery->cable_type))
				sb_vote_set(battery->fcc_vote, VOTER_SWELLING, true, battery->pdata->wireless_cool1_current);
			else
				sb_vote_set(battery->fcc_vote, VOTER_SWELLING, true, battery->pdata->wire_cool1_current);

			sb_vote_set(battery->fv_vote, VOTER_SWELLING, true, battery->pdata->low_temp_float);
			sb_vote_set(battery->topoff_vote, VOTER_SWELLING, false, 0);
			sb_vote_set(battery->chgen_vote, VOTER_SWELLING, true, SB_CHG_MODE_CHARGING);
			sb_wrl_set_vm_phm(VOTER_SWELLING, false, WPC_PHM_OFF);
			battery->health = POWER_SUPPLY_HEALTH_GOOD;
			sec_bat_set_current_event(battery, SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL1,
				SEC_BAT_CURRENT_EVENT_SWELLING_MODE);
			break;
		case BAT_THERMAL_COOL2:
			if (is_wireless_type(battery->cable_type))
				sb_vote_set(battery->fcc_vote, VOTER_SWELLING, true, battery->pdata->wireless_cool2_current);
			else
				sb_vote_set(battery->fcc_vote, VOTER_SWELLING, true, battery->pdata->wire_cool2_current);

			sb_vote_set(battery->fv_vote, VOTER_SWELLING, true, battery->pdata->low_temp_float);
			sb_vote_set(battery->topoff_vote, VOTER_SWELLING, false, 0);
			sb_vote_set(battery->chgen_vote, VOTER_SWELLING, true, SB_CHG_MODE_CHARGING);
			sb_wrl_set_vm_phm(VOTER_SWELLING, false, WPC_PHM_OFF);
			battery->health = POWER_SUPPLY_HEALTH_GOOD;
			sec_bat_set_current_event(battery, SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL2,
				SEC_BAT_CURRENT_EVENT_SWELLING_MODE);
			break;
		case BAT_THERMAL_COOL3:
			if (is_wireless_type(battery->cable_type))
				sb_vote_set(battery->fcc_vote, VOTER_SWELLING, true, battery->pdata->wireless_cool3_current);
			else
				sb_vote_set(battery->fcc_vote, VOTER_SWELLING, true, battery->pdata->wire_cool3_current);

			sb_vote_set(battery->fv_vote, VOTER_SWELLING, true, battery->pdata->low_temp_float);
			sb_vote_set(battery->topoff_vote, VOTER_SWELLING, false, 0);
			sb_vote_set(battery->chgen_vote, VOTER_SWELLING, true, SB_CHG_MODE_CHARGING);
			sb_wrl_set_vm_phm(VOTER_SWELLING, false, WPC_PHM_OFF);
			battery->health = POWER_SUPPLY_HEALTH_GOOD;
			sec_bat_set_current_event(battery, SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL3,
				SEC_BAT_CURRENT_EVENT_SWELLING_MODE);
			break;
		case BAT_THERMAL_COLD:
			sb_wrl_set_vm_phm(VOTER_SWELLING, true, WPC_PHM_EPT_ON);
			sb_vote_set(battery->chgen_vote, VOTER_SWELLING, true, SB_CHG_MODE_CHARGING_OFF);
			battery->health = POWER_SUPPLY_HEALTH_COLD;
			sec_bat_set_charging_status(battery, POWER_SUPPLY_STATUS_NOT_CHARGING);
			sb_cisd_count(2, UNSAFETY_TEMP, UNSAFETY_TEMP_D);
			sec_bat_set_current_event(battery, SEC_BAT_CURRENT_EVENT_LOW_TEMP_SWELLING_COOL3,
				SEC_BAT_CURRENT_EVENT_SWELLING_MODE);
			break;
		case BAT_THERMAL_NORMAL:
		default:
			sb_vote_set(battery->fcc_vote, VOTER_SWELLING, false, 0);
			sb_vote_set(battery->fv_vote, VOTER_SWELLING, false, 0);
			sb_vote_set(battery->topoff_vote, VOTER_SWELLING, false, 0);
			sb_vote_set(battery->chgen_vote, VOTER_SWELLING, false, 0);
			sb_wrl_set_vm_phm(VOTER_SWELLING, false, WPC_PHM_OFF);
			sb_cisd_count(2, SWELLING_RECOVERY_CNT, SWELLING_RECOVERY_CNT_D);
			battery->health = POWER_SUPPLY_HEALTH_GOOD;
			break;
		}
		if ((battery->thermal_zone >= BAT_THERMAL_COOL3) && (battery->thermal_zone <= BAT_THERMAL_WARM)) {
			if ((battery->capacity >= 100) || (battery->status == POWER_SUPPLY_STATUS_FULL))
				sec_bat_set_charging_status(battery, POWER_SUPPLY_STATUS_FULL);
			else
				sec_bat_set_charging_status(battery, POWER_SUPPLY_STATUS_CHARGING);
		}
	} else /* pre_thermal_zone == battery->thermal_zone */
		battery->health_change = false;
}
EXPORT_SYMBOL(sec_bat_thermal_check);
