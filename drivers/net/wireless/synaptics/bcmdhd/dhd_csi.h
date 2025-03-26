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

#ifndef __DHD_CSI_H__
#define __DHD_CSI_H__

/* Maxinum csi file dump size */
#define MAX_CSI_FILESZ		(32 * 1024)
/* Maxinum subcarrier number */
#define MAXIMUM_CFR_DATA	256 * 4
#define CSI_DUMP_PATH		"/sys/bcm-dhd/csi"
#define MAX_EVENT_SIZE		1400
/* maximun csi number stored at dhd */
#define MAX_CSI_NUM		8

/* Header structure to store header contents from firmware */
typedef struct cfr_dump_header_fw {
	uint8 status;
	/* Peer MAC address */
	uint8 peer_macaddr[6];
	/* Number of Space Time Streams */
	uint8 sts;
	/* Number of RX chain */
	uint8 num_rx;
	/* Number of subcarrier */
	uint16 num_carrier;
	/* Length of the CSI dump */
	uint32 cfr_dump_length;
	/* remain unsend CSI data length */
	uint32 remain_length;
	/* RSSI */
	int8 rssi;
} __attribute__ ((packed)) cfr_dump_header_fw_t;

typedef struct cfr_dump_header {
	cfr_dump_header_fw_t fw_header;
	/* Chip id. 1 for BCM43456/8 */
	uint16 chip_id;
	/* Frame control field */
	uint8 fc;
	/* Time stamp when CFR capture is taken, in microseconds since the epoch */
	uint64 cfr_timestamp;
} __attribute__ ((packed)) cfr_dump_header_t;

typedef struct cfr_dump_data {
	cfr_dump_header_t header;
	uint32 data[MAXIMUM_CFR_DATA];
} cfr_dump_data_t;

typedef struct {
	struct list_head list;
	cfr_dump_data_t entry;
} cfr_dump_list_t;

typedef struct cfr_dump_data_fw {
	cfr_dump_header_fw_t header;
	uint32 data[MAXIMUM_CFR_DATA];
} cfr_dump_data_fw_t;

int dhd_csi_event_handler(dhd_pub_t *dhd, wl_event_msg_t *event, void *event_data);

int dhd_csi_init(dhd_pub_t *dhd);

int dhd_csi_deinit(dhd_pub_t *dhd);

void dhd_csi_clean_list(dhd_pub_t *dhd);

int dhd_csi_dump_list(dhd_pub_t *dhd, char *buf);
#endif /* __DHD_CSI_H__ */
