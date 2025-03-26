#include <linux/input/input_booster.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
#include <linux/module.h>

static struct freq_qos_request cpu_cluster_qos;
static bool cluster_qos_init_success;
#define DEFAULT_CLUSTER 0

static DEFINE_MUTEX(input_lock);

int freq_qos_init(void);

void ib_set_booster(long *qos_values)
{
	int res_type = 0;
	int cur_res_idx;
	long value = -1;

	for (res_type = 0; res_type < allowed_res_count; res_type++) {
		cur_res_idx = allowed_resources[res_type];
		value = qos_values[cur_res_idx];

		if (value <= 0)
			continue;

		switch (cur_res_idx) {
		case CLUSTER2:
		case CLUSTER1:
		case CLUSTER0:
			mutex_lock(&input_lock);
			if (cluster_qos_init_success) {
				pr_booster("[Input Booster2] ******      qos value : %d\n", value);
				freq_qos_update_request(&cpu_cluster_qos, value);
			} else {
				freq_qos_init();
			}
			mutex_unlock(&input_lock);
			break;
		case MIF:
		case INT:
		case HMPBOOST:
		case UCC:
		default:
			break;
		}
	}
}

void ib_release_booster(long *rel_flags)
{
	int res_type = 0;
	int cur_res_idx;
	long flag = -1;

	for (res_type = 0; res_type < allowed_res_count; res_type++) {
		cur_res_idx = allowed_resources[res_type];
		flag = rel_flags[cur_res_idx];

		if (flag <= 0)
			continue;

		switch (cur_res_idx) {
		case CLUSTER2:
		case CLUSTER1:
		case CLUSTER0:
			pr_booster("[Input Booster2] ******      qos release value : %d\n", release_val[cur_res_idx]);
			freq_qos_update_request(&cpu_cluster_qos, release_val[cur_res_idx]);
			break;
		case MIF:
		case INT:
		case HMPBOOST:
		case UCC:
		default:
			break;
		}
	}
}

int freq_qos_init(void)
{
	int ret;
	struct cpufreq_policy *policy;

	policy = cpufreq_cpu_get(DEFAULT_CLUSTER);
	if (!policy) {
		pr_err("%s: Failed to get cpufreq policy for cluster(%d)\n",
			__func__, DEFAULT_CLUSTER);
		ret = -EAGAIN;
		goto reset_qos;
	}

	ret = freq_qos_add_request(&policy->constraints,
		&cpu_cluster_qos, FREQ_QOS_MIN, policy->min);
	if (ret < 0) {
		pr_err("%s: Failed to add qos constraint (%d)\n",
			__func__, ret);
		goto reset_qos;
	}

	cluster_qos_init_success = true;
	return 0;

reset_qos:
	freq_qos_remove_request(&cpu_cluster_qos);
	cluster_qos_init_success = false;
	return ret;
}

int input_booster_init_vendor(void)
{
	memset(&cpu_cluster_qos, 0, sizeof(cpu_cluster_qos));
	return 1;
}

void input_booster_exit_vendor(void)
{
	freq_qos_remove_request(&cpu_cluster_qos);
}