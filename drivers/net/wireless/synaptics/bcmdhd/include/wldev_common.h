/*
 * Common function shared by Linux WEXT, cfg80211 and p2p drivers
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
#ifndef __WLDEV_COMMON_H__
#define __WLDEV_COMMON_H__

#include <wlioctl.h>

/* wl_dev_ioctl - get/set IOCTLs, will call net_device's do_ioctl (or
 *  netdev_ops->ndo_do_ioctl in new kernels)
 *  @dev: the net_device handle
 */

s32 wldev_ioctl_get(
	struct net_device *dev, u32 cmd, void *arg, u32 len);

s32 wldev_ioctl_set(
	struct net_device *dev, u32 cmd, const void *arg, u32 len);

/** Retrieve named IOVARs, this function calls wl_dev_ioctl with
 *  WLC_GET_VAR IOCTL code
 */
s32 wldev_iovar_getbuf(
	struct net_device *dev, s8 *iovar_name,
	const void *param, u32 paramlen, void *buf, u32 buflen, struct mutex* buf_sync);

/** Set named IOVARs, this function calls wl_dev_ioctl with
 *  WLC_SET_VAR IOCTL code
 */
s32 wldev_iovar_setbuf(
	struct net_device *dev, s8 *iovar_name,
	const void *param, s32 paramlen, void *buf, s32 buflen, struct mutex* buf_sync);

s32 wldev_iovar_setint(
	struct net_device *dev, s8 *iovar, s32 val);

s32 wldev_iovar_getint(
	struct net_device *dev, s8 *iovar, s32 *pval);

/** The following function can be implemented if there is a need for bsscfg
 *  indexed IOVARs
 */

s32 wldev_mkiovar_bsscfg(
	const s8 *iovar_name, const s8 *param, s32 paramlen,
	s8 *iovar_buf, s32 buflen, s32 bssidx);

/** Retrieve named and bsscfg indexed IOVARs, this function calls wl_dev_ioctl with
 *  WLC_GET_VAR IOCTL code
 */
s32 wldev_iovar_getbuf_bsscfg(
	struct net_device *dev, s8 *iovar_name, void *param, s32 paramlen,
	void *buf, s32 buflen, s32 bsscfg_idx, struct mutex* buf_sync);

/** Set named and bsscfg indexed IOVARs, this function calls wl_dev_ioctl with
 *  WLC_SET_VAR IOCTL code
 */
s32 wldev_iovar_setbuf_bsscfg(
	struct net_device *dev, const s8 *iovar_name, const void *param, s32 paramlen,
	void *buf, s32 buflen, s32 bsscfg_idx, struct mutex* buf_sync);

s32 wldev_iovar_getint_bsscfg(
	struct net_device *dev, s8 *iovar, s32 *pval, s32 bssidx);

s32 wldev_iovar_setint_bsscfg(
	struct net_device *dev, s8 *iovar, s32 val, s32 bssidx);

extern int dhd_net_set_fw_path(struct net_device *dev, char *fw);
extern int dhd_net_bus_suspend(struct net_device *dev);
extern int dhd_net_bus_resume(struct net_device *dev, uint8 stage);
extern int dhd_net_wifi_platform_set_power(struct net_device *dev, bool on,
	unsigned long delay_msec);
extern void dhd_get_customized_country_code(struct net_device *dev, char *country_iso_code,
	wl_country_t *cspec);
extern void dhd_bus_country_set(struct net_device *dev, wl_country_t *cspec, bool notify);

#ifdef OEM_ANDROID
extern bool dhd_force_country_change(struct net_device *dev);
#endif

extern void dhd_bus_band_set(struct net_device *dev, uint band);
extern int wldev_set_country(struct net_device *dev, char *country_code, bool notify,
	int revinfo);
extern int net_os_wake_lock(struct net_device *dev);
extern int net_os_wake_unlock(struct net_device *dev);
extern int net_os_wake_lock_timeout(struct net_device *dev);
extern int net_os_wake_lock_timeout_enable(struct net_device *dev, int val);
extern int net_os_set_dtim_skip(struct net_device *dev, int val);
extern int net_os_set_suspend_disable(struct net_device *dev, int val);
extern int net_os_set_suspend(struct net_device *dev, int val, int force);
extern int net_os_set_suspend_bcn_li_dtim(struct net_device *dev, int val);
extern int net_os_set_max_dtim_enable(struct net_device *dev, int val);
#ifdef DISABLE_DTIM_IN_SUSPEND
extern int net_os_set_disable_dtim_in_suspend(struct net_device *dev, int val);
#endif /* DISABLE_DTIM_IN_SUSPEND */

#if defined(OEM_ANDROID)
extern int wl_parse_ssid_list_tlv(char** list_str, wlc_ssid_ext_t* ssid,
	int max, int *bytes_left);
#endif /* defined(OEM_ANDROID) */

/* Get the link speed from dongle, speed is in kpbs */
int wldev_get_link_speed(struct net_device *dev, int *plink_speed);

int wldev_get_rssi(struct net_device *dev, scb_val_t *prssi);

int wldev_get_ssid(struct net_device *dev, wlc_ssid_t *pssid);

int wldev_get_band(struct net_device *dev, uint *pband);
int wldev_get_mode(struct net_device *dev, uint8 *pband, uint8 caplen);
int wldev_get_datarate(struct net_device *dev, int *datarate);
int wldev_set_band(struct net_device *dev, uint band);

#endif /* __WLDEV_COMMON_H__ */
