/*
 * sb_vote.h
 * Samsung Mobile Battery Vote Header
 *
 * Copyright (C) 2021 Samsung Electronics, Inc.
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

#ifndef __SB_VOTE_H
#define __SB_VOTE_H __FILE__

#include <linux/err.h>

#include <linux/battery/sb_def.h>

enum {
	SB_VOTE_TYPE_MIN,
	SB_VOTE_TYPE_MAX,
	SB_VOTE_TYPE_EN,
	SB_VOTE_TYPE_DATA,
};

enum {
	VOTE_PRI_0 = 0,
	VOTE_PRI_1,
	VOTE_PRI_2,
	VOTE_PRI_3,
	VOTE_PRI_4,
	VOTE_PRI_5,
	VOTE_PRI_6,
	VOTE_PRI_7,
	VOTE_PRI_8,
	VOTE_PRI_9,
	VOTE_PRI_10,
};
#define VOTE_PRI_MIN	VOTE_PRI_0
#define VOTE_PRI_MAX	VOTE_PRI_10

enum {
	SB_VOTER_DISABLE = 0,
	SB_VOTER_ENABLE,
	SB_VOTER_FORCE_SET
};

struct sb_vote;

struct sb_vote_cfg {
	const char	*name;
	int			type;

	const char	**voter_list;
	int			voter_num;

	sb_func		cb;
	sb_cmp_func	cmp;
};

#define SB_VOTE_DISABLE		(-3663)
#define SB_VOTE_DISABLE_STR	"voteoff"

#if IS_ENABLED(CONFIG_SB_VOTE)
struct sb_vote *sb_vote_create(const struct sb_vote_cfg *vote_cfg, void *pdata, sb_data init_data);
void sb_vote_destroy(struct sb_vote *vote);

struct sb_vote *sb_vote_find(const char *name);

int sb_vote_get(struct sb_vote *vote, int event, sb_data *data);
int sb_vote_get_result(struct sb_vote *vote, sb_data *data);
int _sb_vote_set(struct sb_vote *vote, int event, int en, sb_data data, const char *fname, int line);
int sb_vote_set_pri(struct sb_vote *vote, int event, int pri);
int sb_vote_refresh(struct sb_vote *vote);

int sb_vote_show(char *buf, unsigned int p_size);
#else
static inline struct sb_vote *sb_vote_create(const struct sb_vote_cfg *vote_cfg, void *pdata, sb_data init_data)
{ return ERR_PTR(SB_VOTE_DISABLE); }
static inline void sb_vote_destroy(struct sb_vote *vote) {}

static inline struct sb_vote *sb_vote_find(const char *name)
{ return ERR_PTR(SB_VOTE_DISABLE); }

static inline int sb_vote_get(struct sb_vote *vote, int event, sb_data *data)
{ return SB_VOTE_DISABLE; }
static inline int sb_vote_get_result(struct sb_vote *vote, sb_data *data)
{ return SB_VOTE_DISABLE; }
static inline int _sb_vote_set(struct sb_vote *vote, int event, int en, sb_data data, const char *fname, int linee)
{ return SB_VOTE_DISABLE; }
static inline int sb_vote_set_pri(struct sb_vote *vote, int event, int pri)
{ return SB_VOTE_DISABLE; }
static inline int sb_vote_refresh(struct sb_vote *vote)
{ return SB_VOTE_DISABLE; }

static inline int sb_vote_show(char *buf, unsigned int p_size)
{ return SB_VOTE_DISABLE; }
#endif

#define sb_vote_init(_name, _type, _voter_num, _init_data, _voter_list, _cb_vote, _cb_cmp, _pdata) \
({ \
	const struct sb_vote_cfg cfg = { \
		.name = _name, .type = _type, .voter_list = _voter_list, .voter_num = _voter_num, .cb = _cb_vote, .cmp = _cb_cmp \
	}; \
	sb_vote_create(&cfg, _pdata, _init_data); \
})

#define sb_vote_set(_vote, _event, _en, _value)	_sb_vote_set(_vote, _event, _en, _value, __func__, __LINE__)
#define sb_vote_set_f(_name, _event, _en, _value) \
do { \
	struct sb_vote *vote = sb_vote_find(_name); \
\
	if (!vote) { \
		pr_err("%s: failed to find vote(%s)\n", __func__, (_name)); \
		break; \
	} \
	sb_vote_set(vote, _event, _en, _value); \
} while (0)

#define sb_votef_refresh(_name) \
do { \
	struct sb_vote *vote = sb_vote_find(_name); \
\
	if (!vote) { \
		pr_err("%s: failed to find vote(%s)\n", __func__, (_name)); \
		break; \
	} \
	sb_vote_refresh(vote); \
} while (0)

#define sb_vote_set_pri_f(_name, _event, _pri) \
do { \
	struct sb_vote *vote = sb_vote_find(_name); \
\
	if (!vote) { \
		pr_err("%s: failed to find vote(%s)\n", __func__, (_name)); \
		break; \
	} \
	sb_vote_set_pri(vote, _event, _pri); \
} while (0)

#endif /* __SB_VOTE_H */
