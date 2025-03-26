// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Specific feature
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Storage Driver <storage.sec@samsung.com>
 */

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/ctype.h>

#include "dw_mmc-exynos.h"
#include "../core/host.h"
#include "../core/mmc_ops.h"
#include "../core/core.h"
#include "../core/card.h"
#include "mmc-sec-feature.h"
#include "mmc-sec-sysfs.h"

struct device *sec_mmc_cmd_dev;

#define UNSTUFF_BITS(resp, start, size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})
#define MMC_SD_SEC_CALC_STATUS_ERR(member) ({	\
			cur_status_err->member = status_err->member - saved_status_err->member; })
static inline void mmc_sd_sec_get_curr_err_info(struct mmc_sd_sec_device_info *cdi,
		struct mmc_card *card, unsigned long long *crc_cnt,
		unsigned long long *tmo_cnt, struct mmc_sd_sec_status_err_info *cur_status_err)
{
	struct mmc_sd_sec_err_info *err_log = &cdi->err_info[0];
	struct mmc_sd_sec_err_info *saved_err_log = &cdi->saved_err_info[0];
	struct mmc_sd_sec_status_err_info *status_err = &cdi->status_err;
	struct mmc_sd_sec_status_err_info *saved_status_err = &cdi->saved_status_err;

	int i;

	/* Only sbc(0,1)/cmd(2,3)/data(4,5) is checked. */
	for (i = 0; i < 6; i++) {
		if (err_log[i].err_type == -EILSEQ && *crc_cnt < U64_MAX)
			*crc_cnt += (err_log[i].count - saved_err_log[i].count);
		if (err_log[i].err_type == -ETIMEDOUT && *tmo_cnt < U64_MAX)
			*tmo_cnt += (err_log[i].count - saved_err_log[i].count);
	}

	MMC_SD_SEC_CALC_STATUS_ERR(ge_cnt);
	MMC_SD_SEC_CALC_STATUS_ERR(cc_cnt);
	MMC_SD_SEC_CALC_STATUS_ERR(ecc_cnt);
	MMC_SD_SEC_CALC_STATUS_ERR(wp_cnt);
	MMC_SD_SEC_CALC_STATUS_ERR(oor_cnt);
	MMC_SD_SEC_CALC_STATUS_ERR(halt_cnt);
	MMC_SD_SEC_CALC_STATUS_ERR(cq_cnt);
	MMC_SD_SEC_CALC_STATUS_ERR(rpmb_cnt);
	MMC_SD_SEC_CALC_STATUS_ERR(hw_rst_cnt);
	MMC_SD_SEC_CALC_STATUS_ERR(noti_cnt);
}
#define MMC_SD_SEC_SAVE_STATUS_ERR(member) ({		\
		saved_status_err->member = status_err->member;	})

static inline void mmc_sd_sec_save_err_info(struct mmc_sd_sec_device_info *cdi)
{
	struct mmc_sd_sec_err_info *err_log = &cdi->err_info[0];
	struct mmc_sd_sec_err_info *saved_err_log = &cdi->saved_err_info[0];
	struct mmc_sd_sec_status_err_info *status_err = &cdi->status_err;
	struct mmc_sd_sec_status_err_info *saved_status_err = &cdi->saved_status_err;
	int i;

	/* Save current error count */
	for (i = 0; i < MAX_LOG_INDEX; i++)
		saved_err_log[i].count = err_log[i].count;

	MMC_SD_SEC_SAVE_STATUS_ERR(ge_cnt);
	MMC_SD_SEC_SAVE_STATUS_ERR(cc_cnt);
	MMC_SD_SEC_SAVE_STATUS_ERR(ecc_cnt);
	MMC_SD_SEC_SAVE_STATUS_ERR(wp_cnt);
	MMC_SD_SEC_SAVE_STATUS_ERR(oor_cnt);
	MMC_SD_SEC_SAVE_STATUS_ERR(halt_cnt);
	MMC_SD_SEC_SAVE_STATUS_ERR(cq_cnt);
	MMC_SD_SEC_SAVE_STATUS_ERR(rpmb_cnt);
	MMC_SD_SEC_SAVE_STATUS_ERR(hw_rst_cnt);
	MMC_SD_SEC_SAVE_STATUS_ERR(noti_cnt);
}

static ssize_t mmc_sec_unique_number_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_card *card = host->card;
	char gen_pnm[3];
	int i;

	if (!card) {
		dev_info(dev, "%s : card is not exist!\n", __func__);
		return -EINVAL;
	}

	switch (card->cid.manfid) {
	case 0x02:	/* Sandisk	-> [3][4] */
	case 0x45:
		sprintf(gen_pnm, "%.*s", 2, card->cid.prod_name + 3);
		break;
	case 0x11:	/* Toshiba	-> [1][2] */
	case 0x90:	/* Hynix */
		sprintf(gen_pnm, "%.*s", 2, card->cid.prod_name + 1);
		break;
	case 0x13:
	case 0xFE:	/* Micron	-> [4][5] */
		sprintf(gen_pnm, "%.*s", 2, card->cid.prod_name + 4);
		break;
	case 0x15:	/* Samsung	-> [0][1] */
	default:
		sprintf(gen_pnm, "%.*s", 2, card->cid.prod_name + 0);
		break;
	}

	/* Convert to Capital */
	for (i = 0 ; i < 2 ; i++)
		gen_pnm[i] = toupper(gen_pnm[i]);

	return sprintf(buf, "C%s%02X%08X%02X\n",
			gen_pnm, card->cid.prv, card->cid.serial,
			UNSTUFF_BITS(card->raw_cid, 8, 8));
}

static ssize_t mmc_sec_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_card *card = host->card;
	struct mmc_sd_sec_status_err_info status_err;
	u64 crc_cnt = 0;
	u64 tmo_cnt = 0;
	int len = 0;

	if (!card) {
		len = snprintf(buf, PAGE_SIZE,
				"\"GE\":\"0\",\"CC\":\"0\",\"ECC\":\"0\",\"WP\":\"0\","
				"\"OOR\":\"0\",\"CRC\":\"0\",\"TMO\":\"0\","
				"\"HALT\":\"0\",\"CQED\":\"0\",\"RPMB\":\"0\"\n");
		goto out;
	}

	memset(&status_err, 0, sizeof(struct mmc_sd_sec_status_err_info));

	mmc_sd_sec_get_curr_err_info(&mdi, card, &crc_cnt, &tmo_cnt, &status_err);

	len = snprintf(buf, PAGE_SIZE,
			"\"GE\":\"%d\",\"CC\":\"%d\",\"ECC\":\"%d\",\"WP\":\"%d\","
			"\"OOR\":\"%d\",\"CRC\":\"%lld\",\"TMO\":\"%lld\","
			"\"HALT\":\"%d\",\"CQED\":\"%d\",\"RPMB\":\"%d\"\n",
			status_err.ge_cnt, status_err.cc_cnt,
			status_err.ecc_cnt, status_err.wp_cnt,
			status_err.oor_cnt, crc_cnt, tmo_cnt,
			status_err.halt_cnt, status_err.cq_cnt,
			status_err.rpmb_cnt);
out:
	return len;
}

static ssize_t mmc_sec_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_card *card = host->card;

	if (!card)
		return -ENODEV;

	if ((buf[0] != 'C' && buf[0] != 'c') || (count != 1))
		return -EINVAL;

	mmc_sd_sec_save_err_info(&mdi);

	return count;
}

static ssize_t mmc_sec_summary_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_card *card = host->card;
	char *bus_speed_mode = "";
	static const char *const unit[] = {"B", "KB", "MB", "GB", "TB"};
	uint64_t size;
	int digit = 0, pre_size = 1;
	int len = 0;
	char ret_size[6];

	if (card) {
		/* SIZE */
		size = (uint64_t)card->ext_csd.sectors *
				card->ext_csd.data_sector_size;

		/* SIZE - unit */
		while (size > 1024) {
			size /= 1024;
			digit++;
			if (digit == 4)
				break;
		}

		/* SIZE - capacity */
		while (size > pre_size) {
			if (pre_size > 1024)
				break;
			pre_size = pre_size << 1;
		}

		sprintf(ret_size, "%d%s", pre_size, unit[digit]);

		/* SPEED MODE */
		if (mmc_card_hs400(card) || mmc_card_hs400es(card))
			bus_speed_mode = "HS400";
		else if (mmc_card_hs200(card))
			bus_speed_mode = "HS200";
		else if (mmc_card_ddr52(card))
			bus_speed_mode = "DDR50";
		else if (mmc_card_hs(card))
			bus_speed_mode = "HS";
		else
			bus_speed_mode = "LEGACY";

		/* SUMMARY */
		len = sprintf(buf, "\"MANID\":\"0x%02X\",\"PNM\":\"%s\","
			"\"REV\":\"%#x%x%x%x\",\"CQ\":\"%d\","
			"\"SIZE\":\"%s\",\"SPEEDMODE\":\"%s\","
			"\"LIFE\":\"%u\"\n",
			card->cid.manfid, card->cid.prod_name,
			(char)card->ext_csd.fwrev[4],
			(char)card->ext_csd.fwrev[5],
			(char)card->ext_csd.fwrev[6],
			(char)card->ext_csd.fwrev[7],
			((card->host->cqe_on) ? true : false),
			ret_size, bus_speed_mode,
			(card->ext_csd.device_life_time_est_typ_a >
			 card->ext_csd.device_life_time_est_typ_b ?
			 card->ext_csd.device_life_time_est_typ_a :
			 card->ext_csd.device_life_time_est_typ_b));
		dev_info(dev, "%s", buf);
		return len;
	} else {
		/* SUMMARY : No MMC Case */
		dev_info(dev, "%s : No eMMC Card\n", __func__);
		return sprintf(buf, "\"MANID\":\"NoCard\",\"PNM\":\"NoCard\""
				",\"REV\":\"NoCard\",\"CQ\":\"NoCard\",\"SIZE\":\"NoCard\""
				",\"SPEEDMODE\":\"NoCard\",\"LIFE\":\"NoCard\"\n");
	}
}

#ifdef CONFIG_SEC_FACTORY
static ssize_t mmc_sec_hwrst_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_card *card = host->card;

	if (card)
		return sprintf(buf, "%d\n", card->ext_csd.rst_n_function);
	else
		return sprintf(buf, "no card\n");

}
static DEVICE_ATTR(hwrst, 0444, mmc_sec_hwrst_show, NULL);
#endif

static inline void sd_sec_calc_error_count(struct mmc_sd_sec_err_info *err_log,
		unsigned long long *crc_cnt, unsigned long long *tmo_cnt)
{
	int i = 0;

	/* Only sbc(0,1)/cmd(2,3)/data(4,5) is checked. */
	for (i = 0; i < 6; i++) {
		if (err_log[i].err_type == -EILSEQ && *crc_cnt < U64_MAX)
			*crc_cnt += err_log[i].count;
		if (err_log[i].err_type == -ETIMEDOUT && *tmo_cnt < U64_MAX)
			*tmo_cnt += err_log[i].count;
	}
}
static ssize_t mmc_sd_sec_error_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_card *card = host->card;
	struct mmc_sd_sec_device_info *cdi;
	struct mmc_sd_sec_err_info *err_log;
	struct mmc_sd_sec_status_err_info *status_err;
	u64 crc_cnt = 0;
	u64 tmo_cnt = 0;
	int len = 0;
	int i;

	if (!card) {
		len = snprintf(buf, PAGE_SIZE, "No card\n");
		goto out;
	}

	cdi = get_device_info(host);
	err_log = &cdi->err_info[0];
	status_err = &cdi->status_err;

	len += snprintf(buf, PAGE_SIZE,
				"type : err    status: first_issue_time:  last_issue_time:      count\n");

	for (i = 0; i < MAX_LOG_INDEX; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"%5s:%4d 0x%08x %16llu, %16llu, %10d\n",
				err_log[i].type, err_log[i].err_type,
				err_log[i].status,
				err_log[i].first_issue_time,
				err_log[i].last_issue_time,
				err_log[i].count);
	}

	sd_sec_calc_error_count(err_log, &crc_cnt, &tmo_cnt);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"GE:%d,CC:%d,ECC:%d,WP:%d,OOR:%d,CRC:%lld,TMO:%lld,HALT:%d,CQED:%d,RPMB:%d,HWRST:%d\n",
			status_err->ge_cnt, status_err->cc_cnt,
			status_err->ecc_cnt, status_err->wp_cnt,
			status_err->oor_cnt, crc_cnt, tmo_cnt,
			status_err->halt_cnt, status_err->cq_cnt,
			status_err->rpmb_cnt, status_err->hw_rst_cnt);

out:
	return len;
}

static ssize_t mmc_sec_ic_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_sd_sec_device_info *cdi;

	cdi = get_device_info(host);

	return sprintf(buf, "%u\n", cdi->ic);
}

static ssize_t mmc_sec_ic_info_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_sd_sec_device_info *cdi;
	unsigned int value;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	cdi = get_device_info(host);
	cdi->ic = value;

	return count;
}

static ssize_t mmc_sec_lc_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_sd_sec_device_info *cdi;

	cdi = get_device_info(host);

	return sprintf(buf, "%u\n", cdi->lc);
}

static ssize_t mmc_sec_lc_info_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_sd_sec_device_info *cdi;
	unsigned int value;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	cdi = get_device_info(host);
	cdi->lc = value;

	return count;
}

static ssize_t mmc_sec_flt_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = dev_get_drvdata(dev);
	struct mmc_card *card = host->card;
	struct mmc_sd_sec_device_info *cdi;
	u8 *ext_csd;
	int err;
	u8 flt;

	cdi = get_device_info(host);
	flt = cdi->flt;

	mmc_claim_host(host);

	/*
	 * If eMMC is in suspend state, eMMC can't respond to host command.
	 * So return previous flt value.
	 */
	if (mmc_card_suspended(card)) {
		pr_info("%s : mmc is suspended state.", __func__);
		goto release_host;
	}

	if (card->ext_csd.cmdq_en) {
		err = mmc_cmdq_disable(card);
		if (err) {
			pr_err("%s : mmc cmdq disable failed. err : %d\n", __func__, err);
			goto release_host;
		}
	}

	err = mmc_get_ext_csd(card, &ext_csd);
	if (!err) {
		pr_info("%s : FLT : 0x%02x\n", __func__, ext_csd[EXT_CSD_VENDOR_HEALTH_REPORT]);
		flt = ext_csd[EXT_CSD_VENDOR_HEALTH_REPORT];
		cdi->flt = flt;
		kfree(ext_csd);
	} else
		pr_err("%s : failed to get ext_csd. err : %d\n", __func__, err);

	if (card->reenable_cmdq && !card->ext_csd.cmdq_en) {
		err = mmc_cmdq_enable(card);
		if (err)
			pr_err("%s : mmc cmdq enable failed. err : %d\n", __func__, err);
	}

release_host:
	mmc_release_host(host);
	return sprintf(buf, "%u\n", flt);
}

static DEVICE_ATTR(ic, 0664, mmc_sec_ic_info_show, mmc_sec_ic_info_store);
static DEVICE_ATTR(lc, 0664, mmc_sec_lc_info_show, mmc_sec_lc_info_store);
static DEVICE_ATTR(flt, 0444, mmc_sec_flt_info_show, NULL);
static DEVICE_ATTR(un, 0440, mmc_sec_unique_number_show, NULL);
static DEVICE_ATTR(mmc_summary, 0444, mmc_sec_summary_show, NULL);
static DEVICE_ATTR(mmc_data, 0664, mmc_sec_data_show, mmc_sec_data_store);
static DEVICE_ATTR(err_count, 0444, mmc_sd_sec_error_count_show, NULL);

static struct attribute *mmc_attributes[] = {
	&dev_attr_un.attr,
	&dev_attr_mmc_data.attr,
	&dev_attr_mmc_summary.attr,
	&dev_attr_err_count.attr,
#ifdef CONFIG_SEC_FACTORY
	&dev_attr_hwrst.attr,
#endif
	&dev_attr_ic.attr,
	&dev_attr_lc.attr,
	&dev_attr_flt.attr,
	NULL,
};

static struct attribute_group mmc_attr_group = {
	.attrs = mmc_attributes,
};

void sd_sec_create_sysfs_group(struct mmc_host *host, struct device **dev,
		const struct attribute_group *dev_attr_group, const char *group_name)
{
	*dev = sec_device_create(host, group_name);
	if (IS_ERR(*dev)) {
		pr_err("%s: Failed to create device for %s!\n", __func__, group_name);
		return;
	}
	if (sysfs_create_group(&(*dev)->kobj, dev_attr_group))
		pr_err("%s: Failed to create %s sysfs group\n", __func__, group_name);
}

void mmc_sec_init_sysfs(struct mmc_host *host)
{
	sd_sec_create_sysfs_group(host, &sec_mmc_cmd_dev,
			&mmc_attr_group, "mmc");
}
