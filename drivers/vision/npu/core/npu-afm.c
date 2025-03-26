/*
 * Samsung Exynos SoC series NPU driver
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/of_irq.h>
#include <linux/printk.h>
#include <linux/delay.h>

#include "npu-device.h"
#include "npu-system.h"
#include "npu-util-regs.h"
#include "npu-afm.h"
#include "npu-hw-device.h"
#include "npu-scheduler.h"

static unsigned int work_cnt;

static struct npu_system *npu_afm_system;

static void __npu_afm_work(int freq);

static void npu_afm_control_grobal(struct npu_system *system,
					int location, int enable)
{
	if (location == HTU_DNC) {
		if (enable)
			npu_cmd_map(system, "afmdncen");
		else
			npu_cmd_map(system, "afmdncdis");
	} else { /* location == HTU_GNPU1 */
		if (enable)
			npu_cmd_map(system, "afmgnpu1en");
		else
			npu_cmd_map(system, "afmgnpu1dis");
	}
}

static int npu_afm_check_dnc_interrupt(void)
{
	return npu_cmd_map(npu_afm_system, "chkdncitr");
}

static void npu_afm_clear_dnc_interrupt(void)
{
	int val = 0;

	val = npu_cmd_map(npu_afm_system, "chkdncitr");
	if (val)
		npu_cmd_map(npu_afm_system, "clrdncitr");
}

static void npu_afm_clear_dnc_tdc(void)
{
	npu_cmd_map(npu_afm_system, "clrdnctdc");
}

static int npu_afm_check_gnpu1_interrupt(void)
{
	return npu_cmd_map(npu_afm_system, "chkgnpu1itr");
}

static void npu_afm_clear_gnpu1_interrupt(void)
{
	int val = 0;

	val = npu_cmd_map(npu_afm_system, "chkgnpu1itr");
	if (val)
		npu_cmd_map(npu_afm_system, "clrgnpu1itr");
}

static void npu_afm_clear_gnpu1_tdc(void)
{
	npu_cmd_map(npu_afm_system, "clrgnpu1tdc");
}


static irqreturn_t npu_afm_isr0(int irq, void *data)
{
	/* TODO : implement. */
//	npu_cmd_map(npu_afm_system, "afmthren");
//
//	mdelay(100);
//
//	npu_cmd_map(npu_afm_system, "afmthrdis");

	npu_afm_clear_dnc_interrupt();
	npu_afm_clear_dnc_tdc();

//	npu_info("[JONGWOO] Test %s\n", __func__);
//
//	npu_cmd_map(npu_afm_system, "printafmst");

	npu_afm_system->ocp_warn_status = 1;

	queue_delayed_work(npu_afm_system->afm_dnc_wq,
			&npu_afm_system->afm_dnc_work,
			msecs_to_jiffies(1));

	pr_err("Test %s\n", __func__);
	return IRQ_HANDLED;
	if (!npu_afm_check_dnc_interrupt())
		return IRQ_NONE;

/*
	npu_afm_control_interrupt(npu_afm_system, NPU_AFM_DISABLE);

	npu_afm_system->ocp_warn_status = 1;

	queue_delayed_work(npu_afm_system->afm_dnc_wq,
			&npu_afm_system->afm_dnc_work,
			msecs_to_jiffies(0));

	npu_afm_clear_tdc();
	npu_afm_clear_interrupt();

	return IRQ_HANDLED;
*/
}

static irqreturn_t npu_afm_isr1(int irq, void *data)
{
	/* TODO : implement. */
	pr_err("Test %s\n", __func__);
	npu_afm_clear_gnpu1_interrupt();
	npu_afm_check_gnpu1_interrupt();
	npu_afm_clear_gnpu1_tdc();
	return IRQ_HANDLED;
}

static irqreturn_t (*afm_isr_list[])(int, void *) = {
	npu_afm_isr0,
	npu_afm_isr1,
};


static void __npu_afm_work(int freq)
{
	struct npu_scheduler_dvfs_info *d;
	struct npu_scheduler_info *info;

	info = npu_scheduler_get_info();

	mutex_lock(&info->exec_lock);
	list_for_each_entry(d, &info->ip_list, ip_list) {
		if (!strcmp("NPU0", d->name) || !strcmp("NPU1", d->name)) {
			npu_scheduler_set_freq(d, &d->qos_req_max, freq);
		}
	}
	mutex_unlock(&info->exec_lock);

	/* TODO : Implement control freq. */
	return;
}

static void npu_afm_dnc_work(struct work_struct *work)
{
//	struct npu_system *system;
//
//	system = container_of(work, struct npu_system, afm_dnc_work.work);
//
//	system->ocp_warn_status = 0;

	if ((work_cnt % 2))
		__npu_afm_work(1500000);
	else
		__npu_afm_work(533000);

	work_cnt++;
/*
	npu_dbg("afm work start(ocp_status : %d)\n", system->ocp_warn_status);
	if (!system->ocp_warn_status) {
		npu_dbg("Trying release afm_limit_freq\n");
		__npu_afm_work();

		npu_afm_clear_dnc_tdc();
		npu_afm_clear_dnc_interrupt();

		npu_dbg("End release afm_limit_freq\n");
		return;
	}
	__npu_afm_work();
*/
	queue_delayed_work(npu_afm_system->afm_dnc_wq,
			&npu_afm_system->afm_dnc_work,
			msecs_to_jiffies(1000));
	npu_dbg("afm work end\n");
}

void npu_afm_open(struct npu_system *system, int hid)
{
	npu_afm_control_grobal(system, HTU_DNC, NPU_AFM_ENABLE);
	//npu_afm_control_throttling(system, NPU_AFM_ENABLE);
	//npu_cmd_map(npu_afm_system, "afmthren");
	npu_cmd_map(npu_afm_system, "clrdncdiv");

	if (hid & NPU_HWDEV_ID_NPU) {
		npu_afm_control_grobal(system, HTU_GNPU1, NPU_AFM_ENABLE);
		npu_cmd_map(npu_afm_system, "clrgnpu1div");
	} else if (hid & NPU_HWDEV_ID_DSP) {
		npu_cmd_map(npu_afm_system, "clrdspdiv");
	}

	system->ocp_warn_status = 0;

	work_cnt = 0;

	npu_info("open success, hid : %d\n", hid);
}

void npu_afm_close(struct npu_system *system, int hid)
{
	//npu_cmd_map(npu_afm_system, "afmthrdis");
	npu_afm_control_grobal(system, HTU_DNC, NPU_AFM_DISABLE);
	//npu_afm_control_throttling(system, NPU_AFM_DISABLE);

	if (hid & NPU_HWDEV_ID_NPU) {
		npu_afm_control_grobal(system, HTU_GNPU1, NPU_AFM_DISABLE);
	}

	cancel_delayed_work_sync(&npu_afm_system->afm_dnc_work);

	npu_info("close success, hid : %d\n", hid);
}

int npu_afm_probe(struct npu_device *device)
{
	int i, ret = 0, afm_irq_idx = 0;
	const char *buf;
	struct npu_system *system = &device->system;
	struct device *dev = &system->pdev->dev;
	struct cpumask cpu_mask;

	system->ocp_warn_status = 0;

	for (i = system->irq_num; i < (system->irq_num + system->afm_irq_num); i++, afm_irq_idx++) {
		ret = devm_request_irq(dev, system->irq[i], afm_isr_list[afm_irq_idx],
					IRQF_TRIGGER_HIGH, "exynos-npu", NULL);
		if (ret)
			probe_err("fail(%d) in devm_request_irq(%d)\n", ret, i);
	}

	ret = of_property_read_string(dev->of_node, "samsung,npuinter-isr-cpu-affinity", &buf);
	if (ret) {
		probe_info("AFM set the CPU affinity of ISR to 5\n");
		cpumask_set_cpu(5, &cpu_mask);
	}	else {
		probe_info("AFM set the CPU affinity of ISR to %s\n", buf);
		cpulist_parse(buf, &cpu_mask);
	}

	for (i = system->irq_num; i < (system->irq_num + system->afm_irq_num); i++) {
		ret = irq_set_affinity_hint(system->irq[i], &cpu_mask);
		if (ret) {
			probe_err("fail(%d) in irq_set_affinity_hint(%d)\n", ret, i);
			goto err_probe_irq;
		}
	}

	INIT_DELAYED_WORK(&system->afm_dnc_work, npu_afm_dnc_work);

	system->afm_dnc_wq = create_singlethread_workqueue(dev_name(device->dev));
	if (!system->afm_dnc_wq) {
		probe_err("fail to create workqueue -> system->afm_wq\n");
		ret = -EFAULT;
		goto err_probe_irq;
	}

	npu_afm_system = system;
	probe_info("NPU AFM probe success\n");
	return ret;
err_probe_irq:
	for (i = system->irq_num; i < (system->irq_num + system->afm_irq_num); i++) {
		irq_set_affinity_hint(system->irq[i], NULL);
		devm_free_irq(dev, system->irq[i], NULL);
	}

	probe_err("NPU AFM probe failed(%d)\n", ret);
	return ret;
}

int npu_afm_release(struct npu_device *device)
{
	int i, ret = 0;
	struct npu_system *system = &device->system;
	struct device *dev = &system->pdev->dev;

	for (i = system->irq_num; i < (system->irq_num + system->afm_irq_num); i++) {
		irq_set_affinity_hint(system->irq[i], NULL);
		devm_free_irq(dev, system->irq[i], NULL);
	}

	npu_afm_system = NULL;
	probe_info("NPU AFM release success\n");
	return ret;
}
