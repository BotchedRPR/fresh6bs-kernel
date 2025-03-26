/*
 * Copyright (C) 2016 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _EXYNOS_FMP_FIPS_H_
#define _EXYNOS_FMP_FIPS_H_

#include <linux/bio.h>

struct fmp_table_setting {
	__le32 des0;		/* des0 */
#define GET_CMDQ_LENGTH(d) \
	(((d)->des0 & 0xffff0000) >> 16)
	__le32 des1;		/* des1 */
	__le32 des2;		/* des2 */
#define FKL BIT(26)
#define DKL BIT(27)
#define SET_KEYLEN(d, v) ((d)->des2 |= (uint32_t)v)
#define SET_FAS(d, v) \
			((d)->des2 = ((d)->des2 & 0xcfffffff) | v << 28)
#define SET_DAS(d, v) \
			((d)->des2 = ((d)->des2 & 0x3fffffff) | v << 30)
#define GET_FAS(d)      ((d)->des2 & 0x30000000)
#define GET_DAS(d)      ((d)->des2 & 0xc0000000)
#define GET_LENGTH(d) \
			((d)->des2 & 0x3ffffff)
	__le32 des3;		/* des3 */
	/* CMDQ Operation */
#define FKL_CMDQ BIT(0)
#define DKL_CMDQ BIT(1)
#define SET_CMDQ_KEYLEN(d, v) ((d)->des3 |= (uint32_t)v)
#define SET_CMDQ_FAS(d, v) \
			((d)->des3 = ((d)->des3 & 0xfffffff3) | v << 2)
#define SET_CMDQ_DAS(d, v) \
			((d)->des3 = ((d)->des3 & 0xffffffcf) | v << 4)
#define GET_CMDQ_FAS(d) ((d)->des3 & 0x0000000c)
#define GET_CMDQ_DAS(d) ((d)->des3 & 0x00000030)
	__le32 reserved0;	/* des4 */
	__le32 reserved1;
	__le32 reserved2;
	__le32 reserved3;
	__le32 file_iv0;	/* des8 */
	__le32 file_iv1;
	__le32 file_iv2;
	__le32 file_iv3;
	__le32 file_enckey0;	/* des12 */
	__le32 file_enckey1;
	__le32 file_enckey2;
	__le32 file_enckey3;
	__le32 file_enckey4;
	__le32 file_enckey5;
	__le32 file_enckey6;
	__le32 file_enckey7;
	__le32 file_twkey0;	/* des20 */
	__le32 file_twkey1;
	__le32 file_twkey2;
	__le32 file_twkey3;
	__le32 file_twkey4;
	__le32 file_twkey5;
	__le32 file_twkey6;
	__le32 file_twkey7;
	__le32 disk_iv0;	/* des28 */
	__le32 disk_iv1;
	__le32 disk_iv2;
	__le32 disk_iv3;
};


struct exynos_fmp_crypt_info {
	bool fips;
	u64 iv;
	const u8 *key;
	bool cmdq_enabled;
};

struct exynos_fmp_key_info {
	const u8 *raw;
	unsigned int size;
	bool cmdq_enabled;
};

bool is_fmp_fips_op(struct bio *bio);
bool is_fmp_fips_clean(struct bio *bio);
int exynos_fmp_crypt(struct exynos_fmp_crypt_info *fmp_ci, struct fmp_table_setting *table);
int exynos_fmp_clear(struct exynos_fmp_crypt_info *fmp_ci, struct fmp_table_setting *table);
#endif /* _EXYNOS_FMP_FIPS_H_ */
