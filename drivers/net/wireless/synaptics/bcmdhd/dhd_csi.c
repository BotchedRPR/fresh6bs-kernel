/*
 * Broadcom Dongle Host Driver (DHD), Linux-specific network interface.
 * Basically selected code segments from usb-cdc.c and usb-rndis.c
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

#include <osl.h>

#include <bcmutils.h>

#include <bcmendian.h>
#include <linuxver.h>
#include <linux/list.h>
#include <linux/sort.h>
#include <dngl_stats.h>
#include <wlioctl.h>

#include <bcmevent.h>
#include <dhd.h>
#include <dhd_dbg.h>
#include <dhd_csi.h>

#define NULL_CHECK(p, s, err)  \
	do { \
		if (!(p)) { \
			printf("NULL POINTER (%s) : %s\n", __FUNCTION__, (s)); \
			err = BCME_ERROR; \
			return err; \
		} \
	} while (0)

#define TIMESPEC_TO_US(ts)  (((uint64)(ts).tv_sec * USEC_PER_SEC) + \
						(ts).tv_nsec / NSEC_PER_USEC)

#define NULL_ADDR	"\x00\x00\x00\x00\x00\x00"

int
dhd_csi_event_handler(dhd_pub_t *dhd, wl_event_msg_t *event, void *event_data)
{
	int ret = BCME_OK;
	bool is_new = TRUE;
	struct timespec64 ts;
	cfr_dump_data_fw_t *p_event;
	cfr_dump_list_t *ptr, *next, *new;

	NULL_CHECK(dhd, "dhd is NULL", ret);

	DHD_TRACE(("Enter %s\n", __FUNCTION__));
	ktime_get_boottime_ts64(&ts);

	if (!event_data) {
		DHD_ERROR(("%s: event_data is NULL\n", __FUNCTION__));
		return -EINVAL;
	}
	p_event = (cfr_dump_data_fw_t *)event_data;

	/* check if this addr exist */
	if (!list_empty(&dhd->csi_list)) {
		list_for_each_entry_safe(ptr, next, &dhd->csi_list, list) {
			if (bcmp(&ptr->entry.header.fw_header.peer_macaddr,
				&p_event->header.peer_macaddr, ETHER_ADDR_LEN) == 0) {
				int pos = 0, dump_len = 0, remain = 0;
				is_new = FALSE;
				DHD_TRACE(("CSI data exist\n"));
				dump_len = p_event->header.cfr_dump_length;
				if (p_event->header.status == 0) {
					if (dump_len < MAX_EVENT_SIZE) {
						bcopy(&p_event->data, &ptr->entry.data, dump_len);
					}
					else {
						/* for big csi data */
						uint8 *p = (uint8 *)&ptr->entry.data;
						remain = p_event->header.remain_length;
						if (remain) {
							pos = dump_len - remain - MAX_EVENT_SIZE;
							p += pos;
							bcopy(&p_event->data, p, MAX_EVENT_SIZE);
						}
						/* copy rest of csi data */
						else {
							pos = dump_len -
								(dump_len % MAX_EVENT_SIZE);
							p += pos;
							bcopy(&p_event->data, p,
								(dump_len % MAX_EVENT_SIZE));
						}
					}
				}
				ptr->entry.header.fw_header.remain_length = p_event->header.remain_length;
				ptr->entry.header.fw_header.rssi = p_event->header.rssi;
				ptr->entry.header.fc = 0;
				ptr->entry.header.cfr_timestamp = (uint64)TIMESPEC_TO_US(ts);
				return BCME_OK;
			}
		}
	}
	if (is_new) {
		if (dhd->csi_count < MAX_CSI_NUM) {
			new = (cfr_dump_list_t *)MALLOCZ(dhd->osh, sizeof(cfr_dump_list_t));
			if (!new) {
				DHD_ERROR(("Malloc cfr dump list error\n"));
				return BCME_NOMEM;
			}
			bcopy(&p_event->header, &new->entry.header, sizeof(cfr_dump_header_t));
			/* put the timestamp here */
			new->entry.header.cfr_timestamp = (uint64)TIMESPEC_TO_US(ts);
			/* Fill Chip ID */
			new->entry.header.chip_id = dhd_bus_chip_id(dhd);
			/* FC Field */
			new->entry.header.fc = 0;

			DHD_TRACE(("Entry data size %d @%llu\n",
				new->entry.header.fw_header.cfr_dump_length,
				new->entry.header.cfr_timestamp));
			/* for big csi data */
			if (p_event->header.remain_length) {
				DHD_TRACE(("remain %d\n", p_event->header.remain_length));
				bcopy(&p_event->data, &new->entry.data, MAX_EVENT_SIZE);
			}
			else
				bcopy(&p_event->data, &new->entry.data,
					p_event->header.cfr_dump_length);
			INIT_LIST_HEAD(&(new->list));
			list_add_tail(&(new->list), &dhd->csi_list);
			dhd->csi_count++;
		} else {
			DHD_TRACE(("Over maximum CSI Number 8. SKIP it.\n"));
		}
	}
	return ret;
}

int
dhd_csi_init(dhd_pub_t *dhd)
{
	int err = BCME_OK;

	NULL_CHECK(dhd, "dhd is NULL", err);
	INIT_LIST_HEAD(&dhd->csi_list);
	dhd->csi_count = 0;

	return err;
}

int
dhd_csi_deinit(dhd_pub_t *dhd)
{
	int err = BCME_OK;
	cfr_dump_list_t *ptr, *next;

	NULL_CHECK(dhd, "dhd is NULL", err);

	if (0 != dhd->csi_count) {
		if (!list_empty(&dhd->csi_list)) {
			list_for_each_entry_safe(ptr, next, &dhd->csi_list, list) {
				list_del(&ptr->list);
				MFREE(dhd->osh, ptr, sizeof(cfr_dump_list_t));
			}
		}
	}

	return err;
}

void
dhd_csi_clean_list(dhd_pub_t *dhd)
{
	cfr_dump_list_t *ptr, *next;
	int num = 0;

	if (!dhd) {
		DHD_ERROR(("NULL POINTER: %s\n", __FUNCTION__));
		return;
	}

	if (!list_empty(&dhd->csi_list)) {
		list_for_each_entry_safe(ptr, next, &dhd->csi_list, list) {
			list_del(&ptr->list);
			num++;
			MFREE(dhd->osh, ptr, sizeof(cfr_dump_list_t));
		}
	}
	dhd->csi_count = 0;
	DHD_TRACE(("Clean up %d record\n", num));
}

int
dhd_csi_dump_list(dhd_pub_t *dhd, char *buf)
{
	int ret = BCME_OK;
	cfr_dump_list_t *ptr, *next;
	uint8 * pbuf = buf;
	int num = 0;
	int length = 0;

	NULL_CHECK(dhd, "dhd is NULL", ret);

	/* check if this addr exist */
	if (!list_empty(&dhd->csi_list)) {
		list_for_each_entry_safe(ptr, next, &dhd->csi_list, list) {
			if (ptr->entry.header.fw_header.remain_length) {
				DHD_ERROR(("data not ready %d\n",
					ptr->entry.header.fw_header.remain_length));
				continue;
			}
			bcopy(&ptr->entry.header, pbuf, sizeof(cfr_dump_header_t));
			length += sizeof(cfr_dump_header_t);
			pbuf += sizeof(cfr_dump_header_t);
			DHD_TRACE(("Copy data size %d\n",
				ptr->entry.header.fw_header.cfr_dump_length));
			bcopy(&ptr->entry.data, pbuf, ptr->entry.header.fw_header.cfr_dump_length);
			length += ptr->entry.header.fw_header.cfr_dump_length;
			pbuf += ptr->entry.header.fw_header.cfr_dump_length;
			num++;
		}
	}
	DHD_TRACE(("dump %d record %d bytes\n", num, length));

	return length;
}
