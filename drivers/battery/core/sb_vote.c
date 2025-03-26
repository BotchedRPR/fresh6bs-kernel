/*
 *  sb_vote.c
 *  Samsung Mobile Battery Vote
 *
 *  Copyright (C) 2022 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>

#include <linux/battery/sb_vote.h>
#include <linux/battery/sb_pqueue.h>

enum {
	VOTER_INIT = 0,
	VOTER_FORCE,
	VOTER_MAX,
};

struct sb_voter {
	const char *name;
	sb_data data;

	bool enable;
};

struct sb_vote {
	const char *name;
	int			type;

	struct sb_voter *vlist;
	int			vnum;

	sb_func		cb;
	sb_cmp_func	cmp;

	void		*pdata;

	struct sb_pqueue *pq;

	struct mutex lock;
	struct list_head list;

	struct dentry *root;
	struct dentry *status_ent;
	struct dentry *force_set_ent;
};

static struct dentry *debug_root;
static struct dentry *status_all;
static LIST_HEAD(vote_list);
static DEFINE_MUTEX(vote_lock);

static void set_voter(struct sb_voter *voter, const char *name, bool en, sb_data data)
{
	voter->name = name;
	voter->enable = en;
	voter->data = data;
}

static bool cmp_min(sb_data data1, sb_data data2)
{
	return (data1 < data2);
}

static bool cmp_max(sb_data data1, sb_data data2)
{
	return (data1 > data2);
}

static bool check_vote_cfg(const struct sb_vote_cfg *cfg)
{
	return (cfg != NULL) &&
		(cfg->voter_list != NULL) &&
		(cfg->voter_num > 0) &&
		(cfg->cb != NULL) &&
		(cfg->type >= SB_VOTE_TYPE_MIN) &&
		(cfg->type <= SB_VOTE_TYPE_DATA) &&
		((cfg->type != SB_VOTE_TYPE_DATA) || (cfg->cmp != NULL));
}

struct sb_vote *find_vote(const char *name)
{
	struct sb_vote *vote = NULL;

	if (name == NULL)
		return NULL;

	list_for_each_entry(vote, &vote_list, list) {
		if (strcmp(vote->name, name) == 0) {
			return vote;
		}
	}

	return NULL;
}

static void sb_vote_init_debug(struct sb_vote *vote);
struct sb_vote *sb_vote_create(const struct sb_vote_cfg *vote_cfg, void *pdata, sb_data init_data)
{
	struct sb_vote *vote = NULL;
	sb_data data = 0;
	int ret, i;

	if (!check_vote_cfg(vote_cfg) || (pdata == NULL))
		return ERR_PTR(-EINVAL);

	mutex_lock(&vote_lock);
	ret = -EEXIST;
	vote = find_vote(vote_cfg->name);
	if (!IS_ERR_OR_NULL(vote))
		goto err_vote;

	ret = -ENOMEM;
	vote = kzalloc(sizeof(struct sb_vote), GFP_KERNEL);
	if (!vote)
		goto err_vote;

	vote->name = vote_cfg->name;
	vote->cb = vote_cfg->cb;
	vote->pdata = pdata;
	vote->type = vote_cfg->type;

	vote->vnum = vote_cfg->voter_num + VOTER_MAX;
	vote->vlist = kcalloc(vote->vnum, sizeof(struct sb_voter), GFP_KERNEL);
	if (!vote->vlist)
		goto err_vlist;

	switch (vote->type) {
	case SB_VOTE_TYPE_MIN:
		vote->cmp = cmp_min;
		break;
	case SB_VOTE_TYPE_MAX:
		vote->cmp = cmp_max;
		break;
	case SB_VOTE_TYPE_EN:
		vote->cmp = NULL;
		break;
	case SB_VOTE_TYPE_DATA:
		vote->cmp = vote_cfg->cmp;
		break;
	}

	vote->pq = sb_pq_create((PQF_REMOVE | PQF_PRIORITY), vote->vnum, vote->cmp);
	if (IS_ERR_OR_NULL(vote->pq)) {
		ret = (int)PTR_ERR(vote->pq);
		goto err_pq;
	}

	/* init voter */
	for (i = 0; i < vote_cfg->voter_num; i++) {
		set_voter(&vote->vlist[i], vote_cfg->voter_list[i], false, init_data);
		sb_pq_set_pri(vote->pq, i, VOTE_PRI_MIN);
	}

	/* i = vote->vnum - VOTER_MAX; */
	i = vote_cfg->voter_num + VOTER_INIT;
	set_voter(&vote->vlist[i], "Init", true, init_data);
	sb_pq_set_pri(vote->pq, i, VOTE_PRI_MIN - 1);
	sb_pq_push(vote->pq, i, vote->vlist[i].data);

	i = vote_cfg->voter_num + VOTER_FORCE;
	set_voter(&vote->vlist[i], "Force", false, init_data);
	sb_pq_set_pri(vote->pq, i, VOTE_PRI_MAX + 1);

	mutex_init(&vote->lock);
	sb_vote_init_debug(vote);

	pr_info("%s: %s\n", __func__, vote->name);
	list_add(&vote->list, &vote_list);
	mutex_unlock(&vote_lock);

	mutex_lock(&vote->lock);
	/* call cb */
	if (sb_pq_top(vote->pq, &data) >= 0)
		vote->cb(vote->pdata, data);
	mutex_unlock(&vote->lock);

	return vote;

err_pq:
	kfree(vote->vlist);
err_vlist:
	kfree(vote);
err_vote:
	mutex_unlock(&vote_lock);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(sb_vote_create);

void sb_vote_destroy(struct sb_vote *vote)
{
	if (IS_ERR_OR_NULL(vote))
		return;

	mutex_lock(&vote_lock);

	pr_info("%s: %s\n", __func__, vote->name);
	list_del(&vote->list);
	sb_pq_destroy(vote->pq);
	kfree(vote->vlist);
	debugfs_remove_recursive(vote->root);
	mutex_destroy(&vote->lock);
	kfree(vote);

	mutex_unlock(&vote_lock);
}
EXPORT_SYMBOL(sb_vote_destroy);

struct sb_vote *sb_vote_find(const char *name)
{
	struct sb_vote *vote = NULL;

	if (name == NULL)
		return NULL;

	mutex_lock(&vote_lock);
	vote = find_vote(name);
	mutex_unlock(&vote_lock);

	return vote;
}
EXPORT_SYMBOL(sb_vote_find);

int sb_vote_get(struct sb_vote *vote, int event, sb_data *data)
{
	struct sb_voter *voter;

	if (IS_ERR_OR_NULL(vote) ||
		(event < 0) || (event >= vote->vnum - VOTER_MAX) ||
		(data == NULL))
		return -EINVAL;

	mutex_lock(&vote->lock);

	voter = &vote->vlist[event];
	*data = voter->data;

	mutex_unlock(&vote->lock);

	return 0;
}
EXPORT_SYMBOL(sb_vote_get);

int sb_vote_get_result(struct sb_vote *vote, sb_data *data)
{
	sb_data ret = 0;

	if (IS_ERR_OR_NULL(vote) || (data == NULL))
		return -EINVAL;

	mutex_lock(&vote->lock);

	ret = sb_pq_top(vote->pq, data);

	mutex_unlock(&vote->lock);

	return ret;
}
EXPORT_SYMBOL(sb_vote_get_result);

static bool check_vote_changed(sb_data old_data, sb_data new_data)
{
	return (old_data != new_data);
}

int _sb_vote_set(struct sb_vote *vote, int event, int en, sb_data data, const char *fname, int line)
{
	struct sb_voter *voter;
	sb_data old_data = 0, new_data = 0;
	int ret = 0;

	if (IS_ERR_OR_NULL(vote) ||
		(event < 0) || (event >= vote->vnum - VOTER_MAX))
		return -EINVAL;

	mutex_lock(&vote->lock);

	voter = &vote->vlist[event];

	switch (en) {
	case SB_VOTER_FORCE_SET:
		break;
	default:
	case SB_VOTER_ENABLE:
	case SB_VOTER_DISABLE:
		if ((voter->enable == !!(en)) &&
			(voter->data == data))
			goto skip_set;
		break;
	}

	sb_pq_top(vote->pq, &old_data);

	if (voter->enable)
		sb_pq_remove(vote->pq, event);

	voter->enable = !!(en);
	voter->data = data;
	if (voter->enable)
		sb_pq_push(vote->pq, event, voter->data);

	sb_pq_top(vote->pq, &new_data);

	if (check_vote_changed(old_data, new_data))
		ret = vote->cb(vote->pdata, new_data);

skip_set:
	mutex_unlock(&vote->lock);

	return ret;
}
EXPORT_SYMBOL(_sb_vote_set);

int sb_vote_set_pri(struct sb_vote *vote, int event, int pri)
{
	sb_data old_data = 0, new_data = 0;
	int ret = 0;

	if (IS_ERR_OR_NULL(vote) ||
		(event < 0) || (event >= vote->vnum - VOTER_MAX) ||
		(pri < VOTE_PRI_MIN) || (pri > VOTE_PRI_MAX))
		return -EINVAL;
	else if (vote->type == SB_VOTE_TYPE_DATA)
		return -EPERM;

	mutex_lock(&vote->lock);

	if (sb_pq_get_pri(vote->pq, event) == pri)
		goto skip_pri;

	sb_pq_top(vote->pq, &old_data);

	sb_pq_set_pri(vote->pq, event, pri);

	sb_pq_top(vote->pq, &new_data);

	if (check_vote_changed(old_data, new_data))
		ret = vote->cb(vote->pdata, new_data);

skip_pri:
	mutex_unlock(&vote->lock);

	return ret;
}
EXPORT_SYMBOL(sb_vote_set_pri);

int sb_vote_refresh(struct sb_vote *vote)
{
	sb_data data = 0;
	int ret = 0;

	if (IS_ERR_OR_NULL(vote))
		return -EINVAL;

	mutex_lock(&vote->lock);

	if (sb_pq_top(vote->pq, &data) >= 0)
		ret = vote->cb(vote->pdata, data);

	mutex_unlock(&vote->lock);

	return ret;
}
EXPORT_SYMBOL(sb_vote_refresh);

int sb_vote_show(char *buf, unsigned int p_size)
{
	char *type_str = "Unkonwn";
	struct sb_voter *voter;
	struct sb_vote *vote;
	bool val_type = true;
	sb_data data = 0;
	int i, j = 0;

	if (list_empty(&vote_list)) {
		j += scnprintf(buf + j, p_size - j, "No vote\n");
		return j;
	}

	mutex_lock(&vote_lock);
	list_for_each_entry(vote, &vote_list, list) {
		mutex_lock(&vote->lock);
		switch (vote->type) {
		case SB_VOTE_TYPE_MIN:
			type_str = "Min";
			break;
		case SB_VOTE_TYPE_MAX:
			type_str = "Max";
			break;
		case SB_VOTE_TYPE_EN:
			type_str = "En";
			break;
		case SB_VOTE_TYPE_DATA:
		default:
			type_str = "Data";
			val_type = false;
			break;
		}

		for (i = 0; i < vote->vnum; i++) {
			voter = &vote->vlist[i];

			if (voter->enable)
				j += scnprintf(buf + j, p_size - j, "%s: %s:\t\t\ten=%d v=%lld p=%d\n",
						vote->name,
						voter->name,
						voter->enable,
						((val_type) ? voter->data : 0),
						sb_pq_get_pri(vote->pq, i));
		}

		i = sb_pq_top(vote->pq, &data);
		if (i >= 0 && i < vote->vnum) {
			voter = &vote->vlist[i];

			j += scnprintf(buf + j, p_size - j, "%s: voter=%s type=%s v=%lld\n",
				vote->name, voter->name, type_str, ((val_type) ? voter->data : 0));
		}
		mutex_unlock(&vote->lock);
	}
	mutex_unlock(&vote_lock);

	return j;
}
EXPORT_SYMBOL(sb_vote_show);

static void show_vote(struct seq_file *m, struct sb_vote *vote)
{
	char *type_str = "Unkonwn";
	struct sb_voter *voter;
	bool val_type = true;
	sb_data data = 0;
	int i;

	mutex_lock(&vote->lock);

	switch (vote->type) {
	case SB_VOTE_TYPE_MIN:
		type_str = "Min";
		break;
	case SB_VOTE_TYPE_MAX:
		type_str = "Max";
		break;
	case SB_VOTE_TYPE_EN:
		type_str = "En";
		break;
	case SB_VOTE_TYPE_DATA:
	default:
		type_str = "Data";
		val_type = false;
		break;
	}

	for (i = 0; i < vote->vnum; i++) {
		voter = &vote->vlist[i];

		if (voter->enable)
			seq_printf(m, "%s: %s:\t\t\ten=%d v=%lld p=%d\n",
					vote->name,
					voter->name,
					voter->enable,
					((val_type) ? voter->data : 0),
					sb_pq_get_pri(vote->pq, i));
	}

	i = sb_pq_top(vote->pq, &data);
	if (i >= 0 && i < vote->vnum) {
		voter = &vote->vlist[i];

		seq_printf(m, "%s: voter=%s type=%s v=%lld\n",
			vote->name, voter->name, type_str, ((val_type) ? voter->data : 0));
	}

	mutex_unlock(&vote->lock);
}

static int show_all_clients(struct seq_file *m, void *data)
{
	struct sb_vote *vote;

	if (list_empty(&vote_list)) {
		seq_puts(m, "No vote\n");
		return 0;
	}

	mutex_lock(&vote_lock);

	list_for_each_entry(vote, &vote_list, list) {
		show_vote(m, vote);
	}

	mutex_unlock(&vote_lock);

	return 0;
}

static int vote_status_all_open(struct inode *inode, struct file *file)
{

	return single_open(file, show_all_clients, NULL);
}

static const struct file_operations vote_status_all_ops = {
	.owner		= THIS_MODULE,
	.open		= vote_status_all_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_vote_clients(struct seq_file *m, void *data)
{
	struct sb_vote *vote = m->private;

	show_vote(m, vote);
	return 0;
}

static int vote_status_open(struct inode *inode, struct file *file)
{
	struct sb_vote *vote = inode->i_private;

	return single_open(file, show_vote_clients, vote);
}

static const struct file_operations vote_status_ops = {
	.owner		= THIS_MODULE,
	.open		= vote_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int force_get(void *data, u64 *val)
{
	struct sb_vote *vote = data;
	struct sb_voter *voter;

	voter = &vote->vlist[vote->vnum - VOTER_MAX + VOTER_FORCE];
	*val = voter->enable;

	return 0;
}

static int force_set(void *data, u64 val)
{
	struct sb_vote *vote = data;
	struct sb_voter *voter;
	bool en = !!(val);
	sb_data vote_data = 0;
	int idx = 0;

	mutex_lock(&vote->lock);

	idx = vote->vnum - VOTER_MAX + VOTER_FORCE;
	voter = &vote->vlist[idx];
	if (voter->enable != en) {
		if (en)
			sb_pq_push(vote->pq, idx, voter->data);
		else
			sb_pq_pop(vote->pq, &vote_data);
	}
	voter->enable = en;
	if (sb_pq_top(vote->pq, &vote_data) >= 0)
		vote->cb(vote->pdata, vote_data);

	mutex_unlock(&vote->lock);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vote_force_ops, force_get, force_set, "%lld\n");

static void sb_vote_init_debug(struct sb_vote *vote)
{
	if (debug_root == NULL) {
		debug_root = debugfs_create_dir("sb-vote", NULL);
		if (!debug_root) {
			pr_err("Couldn't create debug dir\n");
		} else {
			status_all = debugfs_create_file("status_all",
					S_IFREG | 0444,
					debug_root, NULL,
					&vote_status_all_ops);
			if (!status_all)
				pr_err("Couldn't create status_all dbg file\n");
		}
	}
	if (debug_root)
		vote->root = debugfs_create_dir(vote->name, debug_root);
	if (!vote->root) {
		pr_err("Couldn't create debug dir %s\n", vote->name);
	} else {
		struct sb_voter *voter;

		vote->status_ent = debugfs_create_file("status", S_IFREG | 0444,
				vote->root, vote,
				&vote_status_ops);
		if (!vote->status_ent)
			pr_err("Couldn't create status dbg file for %s\n", vote->name);

		voter = &vote->vlist[vote->vnum - VOTER_MAX + VOTER_FORCE];
		debugfs_create_u32("force_val", S_IFREG | 0644, vote->root, (u32 *)&voter->data);

		vote->force_set_ent = debugfs_create_file("force_set",
				S_IFREG | 0444,
				vote->root, vote,
				&vote_force_ops);
		if (!vote->force_set_ent)
			pr_err("Couldn't create force_set dbg file for %s\n", vote->name);
	}
}
