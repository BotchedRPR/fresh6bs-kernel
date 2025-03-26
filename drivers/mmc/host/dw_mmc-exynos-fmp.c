/*
 * Exynos FMP MMC crypto interface
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <asm/unaligned.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <soc/samsung/exynos-smc.h>
#include <linux/of.h>

#include <linux/keyslot-manager.h>
#include <linux/mmc/core.h>
#include "../core/queue.h"
#include "dw_mmc-exynos-fmp.h"
#ifdef CONFIG_EXYNOS_FMP_FIPS
#include <crypto/fmp_fips.h>
#endif

#define WORD_SIZE	4
#define FMP_IV_SIZE_16	16
#define FMP_IV_MAX_IDX	(FMP_IV_SIZE_16 / WORD_SIZE)

#define byte2word(b0, b1, b2, b3)       \
			(((unsigned int)(b0) << 24) | \
			((unsigned int)(b1) << 16) | \
			((unsigned int)(b2) << 8) | (b3))
#define get_word(x, c)  byte2word(((unsigned char *)(x) + 4 * (c))[0], \
				((unsigned char *)(x) + 4 * (c))[1], \
				((unsigned char *)(x) + 4 * (c))[2], \
				((unsigned char *)(x) + 4 * (c))[3])

int fmp_mmc_crypt_cfg(struct bio *bio, void *desc,
					struct mmc_data *data, int page_index,
					bool cmdq_enabled)
{
#ifdef CONFIG_EXYNOS_FMP_FIPS
	struct exynos_fmp_crypt_info fmp_ci;
	int ret = 0;
#else
	int idx, max;
	u8 iv[FMP_IV_SIZE_16];
#endif
	struct mmc_blk_request *brq = container_of(data, struct mmc_blk_request, data);
	struct mmc_queue_req *mq_rq = container_of(brq, struct mmc_queue_req, brq);
	struct request *rq = mmc_queue_req_to_req(mq_rq);
	struct request_queue *q = rq->q;
	struct fmp_sg_entry *table = desc;
	char *key = NULL;
	const u8 *enckey, *twkey;
	u64 dun = 0;

	if (!bio || !q)
		return 0;

#ifdef CONFIG_EXYNOS_FMP_FIPS
	if (!q->ksm || !bio_has_crypt_ctx(bio)) {
		ret = is_fmp_fips_op(bio);
		if (ret) {
			pr_debug("%s: find fips\n", __func__);
			fmp_ci.fips = true;
			goto encrypt;
		}
		return 0;
	}

	key = (u8 *)bio->bi_crypt_context->bc_key->raw;
	enckey = key;
	twkey = enckey + AES_KEYSIZE_256;
	dun = bio->bi_crypt_context->bc_dun[0] + page_index;
	fmp_ci.fips = false;

#else /* CONFIG_EXYNOS_FMP_FIPS */
	if (!q->ksm || !bio_has_crypt_ctx(bio))
		return 0;
#endif /* CONFIG_EXYNOS_FMP_FIPS */

#ifdef CONFIG_EXYNOS_FMP_FIPS

	/* Configure FMP on each segment of the request. */
encrypt:
	fmp_ci.key = key;
	fmp_ci.iv = dun;
	fmp_ci.cmdq_enabled = cmdq_enabled;

	ret = exynos_fmp_crypt(&fmp_ci, (void *)table);
	if (ret) {
		pr_err("%s: fails to crypt fmp. ret:%d\n", __func__, ret);
		return ret;
	}
#else
	key = (u8 *)bio->bi_crypt_context->bc_key->raw;
	enckey = key;
	twkey = enckey + AES_KEYSIZE_256;

	/* Reject weak AES-XTS keys. */
	if (!crypto_memneq(enckey, twkey, AES_KEYSIZE_256)) {
		pr_err("%s: Can't use weak AES-XTS key\n", __func__);
		return -EINVAL;
	}

	/* Set the algorithm and key length. */
	if (cmdq_enabled) {
		SET_CMDQ_FAS(table, FMP_ALGO_MODE_AES_XTS);
		SET_CMDQ_KEYLEN(table, FKL_CMDQ);
	}
	else {
		SET_FAS(table, FMP_ALGO_MODE_AES_XTS);
		SET_KEYLEN(table, FKL);
	}

	/* Set the key. */
	max = bio->bi_crypt_context->bc_key->size / WORD_SIZE;
	for (idx = 0; idx < (max / 2); idx++)
		*(&table->file_enckey0 + idx) =
			get_word(key, (max / 2) - (idx + 1));

	for (idx = 0; idx < (max / 2); idx++)
		*(&table->file_twkey0 + idx) =
			get_word(key, max - (idx + 1));

	/* Set the IV. */
	dun = bio->bi_crypt_context->bc_dun[0] + page_index;
	memset(iv, 0, FMP_IV_SIZE_16);
	memcpy(iv, &dun, sizeof(dun));

	for (idx = 0; idx < FMP_IV_MAX_IDX; idx++)
		*(&table->file_iv0 + idx) =
			get_word(iv, FMP_IV_MAX_IDX - (idx + 1));
#endif /* CONFIG_EXYNOS_FMP_FIPS */

	return 0;
}
EXPORT_SYMBOL(fmp_mmc_crypt_cfg);

int fmp_mmc_crypt_clear(struct bio *bio, void *desc,
					struct mmc_data *data, bool cmdq_enabled)
{
	struct mmc_blk_request *brq = container_of(data, struct mmc_blk_request, data);
	struct mmc_queue_req *mq_rq = container_of(brq, struct mmc_queue_req, brq);
	struct request *rq = mmc_queue_req_to_req(mq_rq);
	struct request_queue *q = rq->q;
	struct fmp_sg_entry *table = desc;
#ifdef CONFIG_EXYNOS_FMP_FIPS
	struct exynos_fmp_crypt_info fmp_ci;
	int ret = 0;
	fmp_ci.fips = false;
#endif

	if (!bio || !q)
		return 0;

#ifdef CONFIG_EXYNOS_FMP_FIPS
	if (!q->ksm || !bio_has_crypt_ctx(bio)) {
		ret = is_fmp_fips_clean(bio);
		if (ret) {
			fmp_ci.fips = true;
			pr_debug("%s: find fips\n", __func__);
		} else {
			return 0;
		}
	}
#else /* CONFIG_EXYNOS_FMP_FIPS */
	if (!q->ksm || !bio_has_crypt_ctx(bio))
		return 0;
#endif /* CONFIG_EXYNOS_FMP_FIPS */

#ifdef CONFIG_EXYNOS_FMP_FIPS
	table = desc;
	ret = exynos_fmp_clear(&fmp_ci, (void *)table);
	if (ret) {
		pr_warn("%s: fails to clear fips\n", __func__);
	}
#else /* CONFIG_EXYNOS_FMP_FIPS */
	memset(&table->file_iv0, 0, sizeof(__le32) * 24);
#endif /* CONFIG_EXYNOS_FMP_FIPS */

	return 0;
}
EXPORT_SYMBOL(fmp_mmc_crypt_clear);

int fmp_mmc_init_crypt(struct mmc_host *mmc)
{
	struct device *dev = mmc_dev(mmc);

#ifndef CONFIG_EXYNOS_FMP_FIPS
	pr_info("%s Exynos FMP Version: %s\n", __func__, FMP_DRV_VERSION);
#endif
	/* Advertise crypto capabilities to the block layer. */
	blk_ksm_init_passthrough(&mmc->ksm);
	mmc->ksm.max_dun_bytes_supported = AES_BLOCK_SIZE;
	mmc->ksm.features = BLK_CRYPTO_FEATURE_STANDARD_KEYS;
	mmc->ksm.dev = dev; // NULL?
	mmc->ksm.crypto_modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
		FMP_DATA_UNIT_SIZE;
	return 0;
}
EXPORT_SYMBOL(fmp_mmc_init_crypt);

int fmp_mmc_sec_cfg(bool init)
{
	u64 ret = 0;
	pr_info("%s init = %d\n", __func__, init);

	/* configure fmp */
	ret = exynos_smc(SMC_CMD_FMP_SECURITY, 0, FMP_EMBEDDED, CFG_DESCTYPE_3);
	if (ret)
		pr_err("%s: Fail smc call for FMP_SECURITY. ret(%d)\n",
				__func__, ret);

	/* configure smu */
	if (init)
		ret = exynos_smc(SMC_CMD_SMU, SMU_INIT, FMP_EMBEDDED, 0);
	else
		ret = exynos_smc(SMC_CMD_FMP_SMU_RESUME, 0, FMP_EMBEDDED, 0);

	if (ret)
		pr_err("%s: Fail smc call for SMU_INIT/RESUME. ret(%d)\n",
				__func__, ret);
	return ret;
}
EXPORT_SYMBOL(fmp_mmc_sec_cfg);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:dwmmc_exynos_fmp");
