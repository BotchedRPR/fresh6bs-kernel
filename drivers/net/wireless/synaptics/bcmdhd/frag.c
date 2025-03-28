/*
 * IE/TLV fragmentation/defragmentation support for
 * Broadcom 802.11bang Networking Device Driver
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
 * <<Broadcom-WL-IPTag/Dual:>>
 */

#include <bcmutils.h>
#include <frag.h>
#include <802.11.h>

/* defrag a fragmented dot11 ie/tlv. if space does not permit, return the needed
 * ie length to contain all the fragments with status BCME_BUFTOOSHORT.
 * out_len is in/out parameter, max length on input, used/required length on output
 */
int
bcm_tlv_dot11_defrag(const void *buf, uint buf_len, uint8 id, bool id_ext,
	uint8 *out, uint *out_len)
{
	int err = BCME_OK;
	const bcm_tlv_t *ie;
	uint tot_len = 0;
	uint out_left;

	/* find the ie; includes validation */
	ie = bcm_parse_tlvs_dot11(buf, buf_len, id, id_ext);
	if (!ie) {
		err = BCME_IE_NOTFOUND;
		goto done;
	}

	out_left =  (out && out_len) ? *out_len : 0;

	/* first fragment */
	tot_len = id_ext ? ie->len - 1 : ie->len;

	/* copy out if output space permits */
	if ((out == NULL) || (out_left < tot_len)) {
		err = BCME_BUFTOOSHORT;
		out_left = 0; /* prevent further copy */
	} else {
		memcpy(out, &ie->data[id_ext ? 1 : 0], tot_len);
		out += tot_len;
		out_left -= tot_len;
	}

	/* if not fragmened or not fragmentable per 802.11 table  9-77 11md0.1 bail
	 * we can introduce the latter check later
	 */
	if (ie->len != BCM_TLV_MAX_DATA_SIZE) {
		goto done;
	}

	/* adjust buf_len to length after ie including it */
	buf_len -= (uint)(((const uint8 *)ie - (const uint8 *)buf));

	/* update length from fragments, okay if no next ie */
	while ((ie = bcm_next_tlv(ie, &buf_len)) &&
			(ie->id == DOT11_MNG_FRAGMENT_ID)) {
		/* note: buf_len starts at next ie and last frag may be partial */
		if ((out == NULL) || (out_left < ie->len)) {
			err = BCME_BUFTOOSHORT;
			out_left = 0;
		} else {
			memcpy(out, &ie->data[0], ie->len);
			out += ie->len;
			out_left -= ie->len;
		}

		tot_len += ie->len + BCM_TLV_HDR_SIZE;

		/* all but last should be of max size */
		if (ie->len < BCM_TLV_MAX_DATA_SIZE) {
			break;
		}
	}

done:
	if (out_len) {
		*out_len = tot_len;
	}

	return err;
}

int
bcm_tlv_dot11_frag_tot_len(const void *buf, uint buf_len,
	uint8 id, bool id_ext, uint *ie_len)
{
	return  bcm_tlv_dot11_defrag(buf, buf_len, id, id_ext, NULL, ie_len);
}
