/*
 * Platform Dependent file for usage of Preallocted Memory
 *
 * Copyright (C) 2024 Synaptics Incorporated. All rights reserved.
 *
 * This software is licensed to you under the terms of the
 * GNU General Public License version 2 (the "GPL") with Broadcom special exception.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION
 * DOES NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES,
 * SYNAPTICS' TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT
 * EXCEED ONE HUNDRED U.S. DOLLARS
 *
 * Copyright (C) 2024, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/unistd.h>
#include <linux/bug.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#ifndef CONFIG_ARCH_EXYNOS
#include <osl.h>
#endif /* CONFIG_ARCH_EXYNOS */

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM

#ifdef DHD_FW_COREDUMP
#if defined(CONFIG_DHD_USE_STATIC_BUF) && defined(DHD_USE_STATIC_MEMDUMP)
#ifndef CONFIG_BCMDHD_PREALLOC_MEMDUMP
// force pre-malloc memmory for trap dump for dhd_get_fwdump_buf()
#define CONFIG_BCMDHD_PREALLOC_MEMDUMP
#endif // CONFIG_BCMDHD_PREALLOC_MEMDUMP
#endif // defined(CONFIG_DHD_USE_STATIC_BUF) && defined(DHD_USE_STATIC_MEMDUMP)
#endif // DHD_FW_COREDUMP

#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define WLAN_STATIC_DHD_INFO_BUF	7
#define WLAN_STATIC_DHD_WLFC_BUF	8
#define WLAN_STATIC_DHD_IF_FLOW_LKUP	9
#define WLAN_STATIC_DHD_MEMDUMP_RAM	11
#define WLAN_STATIC_DHD_WLFC_HANGER	12
#define WLAN_STATIC_DHD_PKTID_MAP	13	/* Deprecated */
#define WLAN_STATIC_DHD_PKTID_IOCTL_MAP	14	/* Deprecated */
#define WLAN_STATIC_DHD_LOG_DUMP_BUF	15
#define WLAN_STATIC_DHD_LOG_DUMP_BUF_EX	16
#define WLAN_STATIC_DHD_PKTLOG_DUMP_BUF	17	/* Deprecated */

#define WLAN_SCAN_BUF_SIZE		(64 * 1024)

#define WLAN_DHD_INFO_BUF_SIZE		(64 * 1024)
#define WLAN_DHD_WLFC_BUF_SIZE		(64 * 1024)
#define WLAN_DHD_IF_FLOW_LKUP_SIZE	(64 * 1024)
/* Have 2MB ramsize to accomodate future chips */
#define WLAN_DHD_MEMDUMP_SIZE		(3 * 1024 * 1024)

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#ifdef BCMPCIE
#define DHD_SKB_1PAGE_BUFSIZE	(PAGE_SIZE*1)
#define DHD_SKB_2PAGE_BUFSIZE	(PAGE_SIZE*2)
#define DHD_SKB_4PAGE_BUFSIZE	(PAGE_SIZE*4)

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	0
#define WLAN_SECTION_SIZE_2	0
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_1PAGE_BUF_NUM	0
#define DHD_SKB_2PAGE_BUF_NUM	192
#define DHD_SKB_4PAGE_BUF_NUM	0

#else
#define DHD_SKB_HDRSIZE		336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_1PAGE_BUF_NUM	8
#define DHD_SKB_2PAGE_BUF_NUM	8
#define DHD_SKB_4PAGE_BUF_NUM	1
#endif /* BCMPCIE */

#define WLAN_SKB_1_2PAGE_BUF_NUM	((DHD_SKB_1PAGE_BUF_NUM) + \
		(DHD_SKB_2PAGE_BUF_NUM))
#define WLAN_SKB_BUF_NUM	((WLAN_SKB_1_2PAGE_BUF_NUM) + \
		(DHD_SKB_4PAGE_BUF_NUM))

#define WLAN_MAX_PKTID_ITEMS		(8192)
#define WLAN_DHD_PKTID_MAP_HDR_SIZE	(20 + 4*(WLAN_MAX_PKTID_ITEMS + 1))
#define WLAN_DHD_PKTID_MAP_ITEM_SIZE	(32)
#define WLAN_DHD_PKTID_MAP_SIZE		((WLAN_DHD_PKTID_MAP_HDR_SIZE) + \
		((WLAN_MAX_PKTID_ITEMS+1) * WLAN_DHD_PKTID_MAP_ITEM_SIZE))

#define WLAN_MAX_PKTID_IOCTL_ITEMS	(32)
#define WLAN_DHD_PKTID_IOCTL_MAP_HDR_SIZE	(20 + 4*(WLAN_MAX_PKTID_IOCTL_ITEMS + 1))
#define WLAN_DHD_PKTID_IOCTL_MAP_ITEM_SIZE	(32)
#define WLAN_DHD_PKTID_IOCTL_MAP_SIZE		((WLAN_DHD_PKTID_IOCTL_MAP_HDR_SIZE) + \
		((WLAN_MAX_PKTID_IOCTL_ITEMS+1) * WLAN_DHD_PKTID_IOCTL_MAP_ITEM_SIZE))

#define DHD_LOG_DUMP_BUF_SIZE	(1024 * 1024 * 4)
#define DHD_LOG_DUMP_BUF_EX_SIZE	(1024 * 1024 * 2)

#define DHD_PKTLOG_DUMP_BUF_SIZE	(64 * 1024)

#define WLAN_DHD_WLFC_HANGER_MAXITEMS		3072
#define WLAN_DHD_WLFC_HANGER_ITEM_SIZE		32
#define WLAN_DHD_WLFC_HANGER_SIZE	((WLAN_DHD_WLFC_HANGER_ITEM_SIZE) + \
	((WLAN_DHD_WLFC_HANGER_MAXITEMS) * (WLAN_DHD_WLFC_HANGER_ITEM_SIZE)))

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

static void *wlan_static_scan_buf0 = NULL;
static void *wlan_static_scan_buf1 = NULL;
static void *wlan_static_dhd_info_buf = NULL;
static void *wlan_static_dhd_wlfc_buf = NULL;
static void *wlan_static_if_flow_lkup = NULL;
static void *wlan_static_dhd_memdump_ram = NULL;
static void *wlan_static_dhd_wlfc_hanger = NULL;
static void *wlan_static_dhd_log_dump_buf = NULL;
static void *wlan_static_dhd_log_dump_buf_ex = NULL;

void dhd_exit_wlan_mem(void);

void
*dhd_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM) {
		return wlan_static_skb;
	}

	if (section == WLAN_STATIC_SCAN_BUF0) {
		return wlan_static_scan_buf0;
	}

	if (section == WLAN_STATIC_SCAN_BUF1) {
		return wlan_static_scan_buf1;
	}

	if (section == WLAN_STATIC_DHD_INFO_BUF) {
		if (size > WLAN_DHD_INFO_BUF_SIZE) {
			pr_err("request DHD_INFO size(%lu) is bigger than"
				" static size(%d).\n", size,
				WLAN_DHD_INFO_BUF_SIZE);
			return NULL;
		}
		return wlan_static_dhd_info_buf;
	}

	if (section == WLAN_STATIC_DHD_WLFC_BUF)  {
		if (size > WLAN_DHD_WLFC_BUF_SIZE) {
			pr_err("request DHD_WLFC size(%lu) is bigger than"
				" static size(%d).\n",
				size, WLAN_DHD_WLFC_BUF_SIZE);
			return NULL;
		}
		return wlan_static_dhd_wlfc_buf;
	}

	if (section == WLAN_STATIC_DHD_WLFC_HANGER) {
		if (size > WLAN_DHD_WLFC_HANGER_SIZE) {
			pr_err("request DHD_WLFC_HANGER size(%lu) is bigger than"
				" static size(%d).\n",
				size, WLAN_DHD_WLFC_HANGER_SIZE);
			return NULL;
		}
		return wlan_static_dhd_wlfc_hanger;
	}

	if (section == WLAN_STATIC_DHD_IF_FLOW_LKUP)  {
		if (size > WLAN_DHD_IF_FLOW_LKUP_SIZE) {
			pr_err("request DHD_WLFC size(%lu) is bigger than"
				" static size(%d).\n",
				size, WLAN_DHD_WLFC_BUF_SIZE);
			return NULL;
		}
		return wlan_static_if_flow_lkup;
	}

	if (section == WLAN_STATIC_DHD_MEMDUMP_RAM) {
		if (size > WLAN_DHD_MEMDUMP_SIZE) {
			pr_err("request DHD_MEMDUMP_RAM size(%lu) is bigger"
				" than static size(%d).\n",
				size, WLAN_DHD_MEMDUMP_SIZE);
			return NULL;
		}
		return wlan_static_dhd_memdump_ram;
	}

	if (section == WLAN_STATIC_DHD_LOG_DUMP_BUF) {
		if (size > DHD_LOG_DUMP_BUF_SIZE) {
			pr_err("request DHD_LOG_DUMP_BUF size(%lu) is bigger then"
				" static size(%d).\n",
				size, DHD_LOG_DUMP_BUF_SIZE);
			return NULL;
		}
		return wlan_static_dhd_log_dump_buf;
	}

	if (section == WLAN_STATIC_DHD_LOG_DUMP_BUF_EX) {
		if (size > DHD_LOG_DUMP_BUF_EX_SIZE) {
			pr_err("request DHD_LOG_DUMP_BUF_EX size(%lu) is bigger then"
				" static size(%d).\n",
				size, DHD_LOG_DUMP_BUF_EX_SIZE);
			return NULL;
		}
		return wlan_static_dhd_log_dump_buf_ex;
	}

	if ((section < 0) || (section >= PREALLOC_WLAN_SEC_NUM)) {
		return NULL;
	}

	if (wlan_mem_array[section].size < size) {
		return NULL;
	}

	return wlan_mem_array[section].mem_ptr;
}

#ifdef DHD_DUMP_BUF_KVMALLOC
#ifdef CONFIG_ARCH_EXYNOS
#define DUMP_BUF_MALLOC(size)   kvmalloc(size, GFP_KERNEL)
#define DUMP_BUF_MFREE(addr)    kvfree(addr)
#else
#define DUMP_BUF_MALLOC(size)   KVMALLOC(size, GFP_KERNEL)
#define DUMP_BUF_MFREE(addr)    KVFREE(addr)
#endif /* CONFIG_ARCH_EXYNOS */
#else
#define DUMP_BUF_MALLOC(size)   kmalloc(size, GFP_KERNEL)
#define DUMP_BUF_MFREE(addr)    kfree(addr)
#endif /* DHD_DUMP_BUF_KVMALLOC */

int
dhd_init_wlan_mem(void)
{
	int i;
	int j;

#if !defined(BCMPCIE)
	for (i = 0; i < DHD_SKB_1PAGE_BUF_NUM; i++) {
		if (!wlan_static_skb[i]) {
			wlan_static_skb[i] = __dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE, GFP_KERNEL);
			if (!wlan_static_skb[i]) {
				pr_err("Failed to alloc 1PAGE SKB BUF\n");
				goto err_skb_alloc;
			}
		}
	}
#endif /* !BCMPCIE */

	for (i = DHD_SKB_1PAGE_BUF_NUM; i < WLAN_SKB_1_2PAGE_BUF_NUM; i++) {
		if (!wlan_static_skb[i]) {
			wlan_static_skb[i] = __dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE, GFP_KERNEL);
			if (!wlan_static_skb[i]) {
				pr_err("Failed to alloc 2PAGE SKB BUF\n");
				goto err_skb_alloc;
			}
		}
	}

#if !defined(BCMPCIE)
	for (i = WLAN_SKB_1_2PAGE_BUF_NUM; i < WLAN_SKB_BUF_NUM; i++) {
		if (!wlan_static_skb[i]) {
			wlan_static_skb[i] = __dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE, GFP_KERNEL);
			if (!wlan_static_skb[i]) {
				pr_err("Failed to alloc 4PAGE SKB BUF\n");
				goto err_skb_alloc;
			}
		}
	}
#endif /* !BCMPCIE */

	for (i = 0; i < PREALLOC_WLAN_SEC_NUM; i++) {
		if ((wlan_mem_array[i].size > 0) && (!wlan_mem_array[i].mem_ptr)) {
			wlan_mem_array[i].mem_ptr =
				DUMP_BUF_MALLOC(wlan_mem_array[i].size);

			if (!wlan_mem_array[i].mem_ptr) {
				pr_err("Failed to mem_alloc for WLAN\n");
				goto err_mem_alloc;
			}
		}
	}

	if (!wlan_static_scan_buf0) {
		wlan_static_scan_buf0 = DUMP_BUF_MALLOC(WLAN_SCAN_BUF_SIZE);
		if (!wlan_static_scan_buf0) {
			pr_err("Failed to alloc wlan_static_scan_buf0\n");
			goto err_mem_alloc;
		}
	}

	if (!wlan_static_scan_buf1) {
		wlan_static_scan_buf1 = DUMP_BUF_MALLOC(WLAN_SCAN_BUF_SIZE);
		if (!wlan_static_scan_buf1) {
			pr_err("Failed to alloc wlan_static_scan_buf1\n");
			goto err_mem_alloc;
		}
	}

	if (!wlan_static_dhd_log_dump_buf) {
		wlan_static_dhd_log_dump_buf = DUMP_BUF_MALLOC(DHD_LOG_DUMP_BUF_SIZE);
		if (!wlan_static_dhd_log_dump_buf) {
			pr_err("Failed to alloc wlan_static_dhd_log_dump_buf\n");
			goto err_mem_alloc;
		}
	}

	if (!wlan_static_dhd_log_dump_buf_ex) {
		wlan_static_dhd_log_dump_buf_ex = DUMP_BUF_MALLOC(DHD_LOG_DUMP_BUF_EX_SIZE);
		if (!wlan_static_dhd_log_dump_buf_ex) {
			pr_err("Failed to alloc wlan_static_dhd_log_dump_buf_ex\n");
			goto err_mem_alloc;
		}
	}

	if (!wlan_static_dhd_info_buf) {
		wlan_static_dhd_info_buf = DUMP_BUF_MALLOC(WLAN_DHD_INFO_BUF_SIZE);
		if (!wlan_static_dhd_info_buf) {
			pr_err("Failed to alloc wlan_static_dhd_info_buf\n");
			goto err_mem_alloc;
		}
	}

#ifdef BCMPCIE
	if (!wlan_static_if_flow_lkup) {
		wlan_static_if_flow_lkup = DUMP_BUF_MALLOC(WLAN_DHD_IF_FLOW_LKUP_SIZE);
		if (!wlan_static_if_flow_lkup) {
			pr_err("Failed to alloc wlan_static_if_flow_lkup\n");
			goto err_mem_alloc;
		}
	}
#else
	if (!wlan_static_dhd_wlfc_buf) {
		wlan_static_dhd_wlfc_buf = DUMP_BUF_MALLOC(WLAN_DHD_WLFC_BUF_SIZE);
		if (!wlan_static_dhd_wlfc_buf) {
			pr_err("Failed to alloc wlan_static_dhd_wlfc_buf\n");
			goto err_mem_alloc;
		}
	}

	if (!wlan_static_dhd_wlfc_hanger) {
		wlan_static_dhd_wlfc_hanger = DUMP_BUF_MALLOC(WLAN_DHD_WLFC_HANGER_SIZE);
		if (!wlan_static_dhd_wlfc_hanger) {
			pr_err("Failed to alloc wlan_static_dhd_wlfc_hanger\n");
			goto err_mem_alloc;
		}
	}
#endif /* BCMPCIE */

#ifdef CONFIG_BCMDHD_PREALLOC_MEMDUMP
	if (!wlan_static_dhd_memdump_ram) {
		wlan_static_dhd_memdump_ram = DUMP_BUF_MALLOC(WLAN_DHD_MEMDUMP_SIZE);
		if (!wlan_static_dhd_memdump_ram) {
			pr_err("Failed to alloc wlan_static_dhd_memdump_ram\n");
			goto err_mem_alloc;
		}
	}
#endif /* CONFIG_BCMDHD_PREALLOC_MEMDUMP */

	pr_err("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

err_mem_alloc:
	dhd_exit_wlan_mem();
	return -ENOMEM;

err_skb_alloc:
	/*
	 * When all the skb alloc buf couldn't alloced, free these buf with alloced size
	 * dhd_exit_wlan_mem will free with total size (don't know alloced size)
	 */
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0; j < i; j++) {
		if (wlan_static_skb[j]) {
			dev_kfree_skb(wlan_static_skb[j]);
		}
	}
	return -ENOMEM;
}

void
dhd_exit_wlan_mem(void)
{
	int i = 0;

#ifdef CONFIG_BCMDHD_PREALLOC_MEMDUMP
	if (wlan_static_dhd_memdump_ram) {
		DUMP_BUF_MFREE(wlan_static_dhd_memdump_ram);
		wlan_static_dhd_memdump_ram = NULL;
	}

#endif /* CONFIG_BCMDHD_PREALLOC_MEMDUMP */

#ifdef BCMPCIE
	if (wlan_static_if_flow_lkup) {
		DUMP_BUF_MFREE(wlan_static_if_flow_lkup);
		wlan_static_if_flow_lkup = NULL;
	}
#else
	if (wlan_static_dhd_wlfc_buf) {
		DUMP_BUF_MFREE(wlan_static_dhd_wlfc_buf);
		wlan_static_dhd_wlfc_buf = NULL;
	}

	if (wlan_static_dhd_wlfc_hanger) {
		DUMP_BUF_MFREE(wlan_static_dhd_wlfc_hanger);
		wlan_static_dhd_wlfc_hanger = NULL;
	}
#endif /* BCMPCIE */
	if (wlan_static_dhd_info_buf) {
		DUMP_BUF_MFREE(wlan_static_dhd_info_buf);
		wlan_static_dhd_info_buf = NULL;
	}

	if (wlan_static_dhd_log_dump_buf) {
		DUMP_BUF_MFREE(wlan_static_dhd_log_dump_buf);
		wlan_static_dhd_log_dump_buf = NULL;
	}

	if (wlan_static_dhd_log_dump_buf_ex) {
		DUMP_BUF_MFREE(wlan_static_dhd_log_dump_buf_ex);
		wlan_static_dhd_log_dump_buf_ex = NULL;
	}

	if (wlan_static_scan_buf1) {
		DUMP_BUF_MFREE(wlan_static_scan_buf1);
		wlan_static_scan_buf1 = NULL;
	}

	if (wlan_static_scan_buf0) {
		DUMP_BUF_MFREE(wlan_static_scan_buf0);
		wlan_static_scan_buf0 = NULL;
	}

	for (i = 0; i < PREALLOC_WLAN_SEC_NUM; i++) {
		if (wlan_mem_array[i].mem_ptr) {
			DUMP_BUF_MFREE(wlan_mem_array[i].mem_ptr);
			wlan_mem_array[i].mem_ptr = NULL;
		}
	}

	for (i = 0; i < WLAN_SKB_BUF_NUM; i++) {
		if (wlan_static_skb[i]) {
			dev_kfree_skb(wlan_static_skb[i]);
			wlan_static_skb[i] = NULL;
		}
	}

	return;
}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
