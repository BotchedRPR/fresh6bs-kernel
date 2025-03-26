#include <linux/delay.h>

#include <linux/battery/common/sb_psy.h>
#include <linux/battery/common/sb_type.h>
#include <linux/battery/common/sb_vote_event.h>
#include <linux/battery/module/sb_startup.h>

#include "p9222_charger.h"

static int sup_event(void *pdata, unsigned int event, sb_data time)
{
	struct p9222_charger_data *charger = pdata;
	sb_data phm_result = WPC_PHM_OFF;
	int vout, vrect, i;
	bool reset_wireless_chg = false;

	if (!is_wireless_type(charger->cable_type))
		goto end_event;

	sb_vote_get_result(charger->phm_vote, &phm_result);
	if (phm_result != WPC_PHM_OFF)
		goto end_event;

	/* WA code. Boot and put it on the charging pad 2 seconds later
	 * Vout can be abnormal value
	 */
	for (i = 0; i < 3; i++) {
		if (charger->pdata->ic_on_mode || (charger->wc_w_state)) {
			vout = p9222_get_adc(charger, P9222_ADC_VOUT);
			vrect = p9222_get_adc(charger, P9222_ADC_VRECT);
		} else {
			vrect = 0;
			vrect = 0;
		}
		if (vout >= 2000) {
			reset_wireless_chg = false;
			break;
		}
		reset_wireless_chg = (vrect >= 3500);
		pr_info("%s vout(%d) vrect(%d) reset_wireless_chg(%d)\n",
			__func__, vout, vrect, reset_wireless_chg);
		msleep(100);
	}

	/* if vout is abnormal, reset charging using PHM */
	if (reset_wireless_chg) {
		pr_info("%s vout & chgen off!!\n", __func__);
		sb_vote_set_f(VN_CHG_EN, VOTER_STARTUP, true, SB_CHG_MODE_CHARGING_OFF);
		sb_vote_set(charger->phm_vote, VOTER_STARTUP, true, WPC_PHM_ON);

		pr_info("%s vout & chgen on!!\n", __func__);
		sb_vote_set(charger->phm_vote, VOTER_STARTUP, false, WPC_PHM_OFF);
		sb_vote_set_f(VN_CHG_EN, VOTER_STARTUP, false, SB_CHG_MODE_CHARGING_OFF);
	}

end_event:
	sb_vote_set(charger->apm_vote, VOTER_STARTUP, false, WPC_APM_NONE);
	return 0;
}

int p9222_startup_init(struct p9222_charger_data *charger)
{
	unsigned int sup_time = 60; /* default 1min */
	int ret = 0;

	if (!sb_startup_is_activated()) {
		pr_info("%s: startup is not activated!\n", __func__);
		return 0;
	}

	/* 0x06: vout 4.8V, vret 5.16V */
	sb_vote_set(charger->apm_vote, VOTER_STARTUP, true, WPC_APM_BOOT);

	/* set startup */
	ret = sb_startup_register(charger, sup_event, 0, sup_time);
	return ret;
}
EXPORT_SYMBOL(p9222_startup_init);
