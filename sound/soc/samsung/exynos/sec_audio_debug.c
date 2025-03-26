/*
 *  sec_audio_debug.c
 *
 *  Copyright (c) 2023 Samsung Electronics
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/sched/clock.h>

#include <sound/soc.h>
#include <sound/samsung/sec_audio_debug.h>

#include <linux/sec_debug.h>

#define DBG_STR_BUFF_SZ 256
#define LOG_MSG_BUFF_SZ 512

#define SEC_AUDIO_DEBUG_STRING_WQ_NAME "sec_audio_dbg_str_wq"

#define MAX_DBG_PARAM 4
#define MAX_ERR_STR_LEN 20

char abox_err_s[MAX_ERR_STR_LEN][TYPE_ABOX_DEBUG_MAX + 1] = {
	[TYPE_ABOX_NOERROR] = "NOERR",
	[TYPE_ABOX_DATAABORT] = "DABRT",
	[TYPE_ABOX_PREFETCHABORT] = "PABRT",
	[TYPE_ABOX_OSERROR] = "OSERR",
	[TYPE_ABOX_VSSERROR] = "VSERR",
	[TYPE_ABOX_UNDEFEXCEPTION] = "UNDEF",
	[TYPE_ABOX_DEBUGASSERT] = "ASSRT",
	[TYPE_ABOX_DEBUG_MAX] = "UNKWN",
};

static size_t rmem_size_min[TYPE_SIZE_MAX] = {0xab0cab0c, 0xab0cab0c};

struct device *debug_dev;

struct modem_status {
	int event;
	unsigned long long time;
	unsigned long nanosec_t;
};

struct audio_mode_status {
	int audio_mode;
	unsigned long long time;
	unsigned long nanosec_t;
};

struct sec_audio_debug_data {
	char *dbg_str_buf;
	struct mutex dbg_lock;
	struct workqueue_struct *debug_string_wq;
	struct work_struct debug_string_work;
	enum abox_debug_err_type debug_err_type;
	int param[MAX_DBG_PARAM];
	struct audio_mode_status audio_mode_state;
	struct modem_status modem_state;
};

static struct sec_audio_debug_data *p_debug_data;

void set_modem_event(int event)
{
	if (!p_debug_data)
		return;

	p_debug_data->modem_state.event = event;
	p_debug_data->modem_state.time = local_clock();
	p_debug_data->modem_state.nanosec_t = do_div(p_debug_data->modem_state.time, 1000000000);
}
EXPORT_SYMBOL_GPL(set_modem_event);

static void abox_debug_string_update_workfunc(struct work_struct *wk)
{
	unsigned int len = 0;
	int i = 0;
	unsigned long long time = local_clock();
	unsigned long nanosec_t = do_div(time, 1000000000);

	if (!p_debug_data)
		return;

	mutex_lock(&p_debug_data->dbg_lock);

	p_debug_data->dbg_str_buf = kzalloc(sizeof(char) * DBG_STR_BUFF_SZ, GFP_KERNEL);
	if (!p_debug_data->dbg_str_buf) {
		pr_err("%s: no memory\n", __func__);
		mutex_unlock(&p_debug_data->dbg_lock);
		return;
	}

	len += snprintf(p_debug_data->dbg_str_buf + len, DBG_STR_BUFF_SZ - len,
			"[%lu.%06lu] ", (unsigned long) time, nanosec_t / 1000);
	if (len >= (DBG_STR_BUFF_SZ - 1))
		goto buff_done;

	len += snprintf(p_debug_data->dbg_str_buf + len, DBG_STR_BUFF_SZ - len,
			"%s ", abox_err_s[p_debug_data->debug_err_type]);
	if (len >= (DBG_STR_BUFF_SZ - 1))
		goto buff_done;

	for (i = 0; i < MAX_DBG_PARAM; i++) {
		len += snprintf(p_debug_data->dbg_str_buf + len, DBG_STR_BUFF_SZ - len,
				"%#x ", p_debug_data->param[i]);
		if (len >= (DBG_STR_BUFF_SZ - 1))
			goto buff_done;
	}

	len += snprintf(p_debug_data->dbg_str_buf + len, DBG_STR_BUFF_SZ - len,
					"mode %d:%lu.%06lu ", p_debug_data->audio_mode_state.audio_mode,
					(unsigned long) p_debug_data->audio_mode_state.time,
					p_debug_data->audio_mode_state.nanosec_t / 1000);
	if (len >= (DBG_STR_BUFF_SZ - 1))
		goto buff_done;

	len += snprintf(p_debug_data->dbg_str_buf + len, DBG_STR_BUFF_SZ - len,
					"modem ev %d:%lu.%06lu", p_debug_data->modem_state.event,
					(unsigned long) p_debug_data->modem_state.time,
					p_debug_data->modem_state.nanosec_t / 1000);
	if (len >= (DBG_STR_BUFF_SZ - 1))
		goto buff_done;

buff_done:
	if (len >= (DBG_STR_BUFF_SZ - 1))
		len = (DBG_STR_BUFF_SZ - 1);
	p_debug_data->dbg_str_buf[len] = 0;
	pr_info("%s: %s\n", __func__, p_debug_data->dbg_str_buf);
	secdbg_exin_set_aud(p_debug_data->dbg_str_buf);

	kfree(p_debug_data->dbg_str_buf);
	p_debug_data->dbg_str_buf = NULL;
	mutex_unlock(&p_debug_data->dbg_lock);
}

void abox_debug_string_update(enum abox_debug_err_type type, int p1, int p2, int p3,
				int audio_mode, unsigned long long audio_mode_time)
{
	if (!p_debug_data)
		return;

	if (type > TYPE_ABOX_DEBUG_MAX)
		type = TYPE_ABOX_DEBUG_MAX;

	p_debug_data->debug_err_type = type;
	p_debug_data->param[0] = type;
	p_debug_data->param[1] = p1;
	p_debug_data->param[2] = p2;
	p_debug_data->param[3] = p3;

	p_debug_data->audio_mode_state.audio_mode = audio_mode;
	p_debug_data->audio_mode_state.time = audio_mode_time;
	p_debug_data->audio_mode_state.nanosec_t = do_div(p_debug_data->audio_mode_state.time, 1000000000);

	queue_work(p_debug_data->debug_string_wq,
			&p_debug_data->debug_string_work);
}
EXPORT_SYMBOL_GPL(abox_debug_string_update);

int check_upload_mode_disabled(void)
{
	int val = secdbg_mode_enter_upload();

	if (val == 0x5)
		return 0;
	else
		return 1;
}
EXPORT_SYMBOL_GPL(check_upload_mode_disabled);

int __read_mostly debug_level;
module_param(debug_level, int, 0440);

int check_debug_level_low(void)
{
	int debug_level_low = 0;

	pr_info("%s: 0x%x\n", __func__, debug_level);

	if (debug_level == 0x4f4c)
		debug_level_low = 1;
	else
		debug_level_low = 0;

	return debug_level_low;
}
EXPORT_SYMBOL_GPL(check_debug_level_low);

size_t get_rmem_size_min(enum rmem_size_type id)
{
	return rmem_size_min[id];
}
EXPORT_SYMBOL_GPL(get_rmem_size_min);

static int sec_audio_debug_probe(struct platform_device *pdev)
{
	struct sec_audio_debug_data *data;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	unsigned int val = 0;

	ret = of_property_read_u32(np, "abox_dbg_size_min", &val);
	if (ret < 0)
		dev_err(dev, "%s: failed to get abox_dbg_size_min %d\n", __func__, ret);
	else
		rmem_size_min[TYPE_ABOX_DBG_SIZE] = (size_t) val;

	ret = of_property_read_u32(np, "abox_slog_size_min", &val);
	if (ret < 0)
		dev_err(dev, "%s: failed to get abox_slog_size_min %d\n", __func__, ret);
	else
		rmem_size_min[TYPE_ABOX_SLOG_SIZE] = (size_t) val;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	p_debug_data = data;
	mutex_init(&p_debug_data->dbg_lock);

	p_debug_data->debug_string_wq = create_singlethread_workqueue(
						SEC_AUDIO_DEBUG_STRING_WQ_NAME);
	if (p_debug_data->debug_string_wq == NULL) {
		pr_err("%s: failed to creat debug_string_wq\n", __func__);
		ret = -ENOENT;
		goto err_data;
	}

	INIT_WORK(&p_debug_data->debug_string_work,
			abox_debug_string_update_workfunc);

	debug_dev = &pdev->dev;

err_data:
	mutex_destroy(&p_debug_data->dbg_lock);
	kfree(p_debug_data);
	p_debug_data = NULL;

	return ret;
}

static int sec_audio_debug_remove(struct platform_device *pdev)
{
	destroy_workqueue(p_debug_data->debug_string_wq);
	p_debug_data->debug_string_wq = NULL;

	mutex_destroy(&p_debug_data->dbg_lock);
	kfree(p_debug_data);
	p_debug_data = NULL;

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sec_audio_debug_of_match[] = {
	{ .compatible = "samsung,audio-debug", },
	{},
};
MODULE_DEVICE_TABLE(of, sec_audio_debug_of_match);
#endif /* CONFIG_OF */

static struct platform_driver sec_audio_debug_driver = {
	.driver		= {
		.name	= "sec-audio-debug",
		.owner	= THIS_MODULE,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = of_match_ptr(sec_audio_debug_of_match),
#endif /* CONFIG_OF */
	},

	.probe = sec_audio_debug_probe,
	.remove = sec_audio_debug_remove,
};

module_platform_driver(sec_audio_debug_driver);

MODULE_DESCRIPTION("Samsung Electronics Audio Debug driver");
MODULE_LICENSE("GPL");
