/*
 * Wifi Virtual Interface implementaion
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
/* */

#include <typedefs.h>
#include <linuxver.h>
#include <linux/kernel.h>

#include <bcmutils.h>
#include <bcmstdlib_s.h>
#include <bcmwifi_channels.h>
#include <bcmendian.h>
#include <ethernet.h>
#ifdef WL_WPS_SYNC
#include <eapol.h>
#endif /* WL_WPS_SYNC */
#include <802.11.h>
#include <bcmiov.h>
#include <linux/if_arp.h>
#include <asm/uaccess.h>

#include <ethernet.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <linux/wait.h>
#include <net/cfg80211.h>
#include <net/rtnetlink.h>

#include <wlioctl.h>
#include <bcmevent.h>
#include <wldev_common.h>
#include <wl_cfg80211.h>
#include <wl_cfgp2p.h>
#include <wl_cfgscan.h>
#include <wl_cfgvif.h>
#include <bcmdevs.h>
#include <bcmdevs_legacy.h>
#ifdef WL_FILS
#include <fils.h>
#include <frag.h>
#endif /* WL_FILS */

#ifdef OEM_ANDROID
#include <wl_android.h>
#endif

#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_linux.h>
#include <dhd_linux_pktdump.h>
#include <dhd_debug.h>
#include <dhdioctl.h>
#include <wlioctl.h>
#include <dhd_cfg80211.h>
#include <dhd_bus.h>
#include <wl_cfgvendor.h>

#ifdef WL_NAN
#include <wl_cfgnan.h>
#endif /* WL_NAN */

#ifdef BCMPCIE
#include <dhd_flowring.h>
#endif
#if defined(BIGDATA_SOFTAP) || defined(DHD_ENABLE_BIGDATA_LOGGING)
#include <wl_bigdata.h>
#endif /* BIGDATA_SOFTAP || DHD_ENABLE_BIGDATA_LOGGING */

#define MAX_VIF_OFFSET	15
#define MAX_WAIT_TIME 1500

#define DNGL_FUNC(func, parameters) func parameters
#define COEX_DHCP

/* SoftAP related parameters */
#define DEFAULT_2G_SOFTAP_CHANNEL	1
#define DEFAULT_2G_SOFTAP_CHANSPEC	0x1006
#define DEFAULT_5G_SOFTAP_CHANNEL	149

#define MAX_VNDR_OUI_STR_LEN	256u
#define VNDR_OUI_STR_LEN	10u
#define DOT11_DISCONNECT_RC     2u

bool
is_mode_different(dhd_pub_t *dhdp, int ifidx, int nmode, int vhtmode,
		int hemode, chanspec_t chanspec, bool force_all_phy_mode_cmp);
#if defined(WL_FW_OCE_AP_SELECT)
static bool
wl_cfgoce_has_ie(const u8 *ie, const u8 **tlvs, u32 *tlvs_len, const u8 *oui, u32 oui_len, u8 type);

/* Check whether the given IE looks like WFA OCE IE. */
#define wl_cfgoce_is_oce_ie(ie, tlvs, len)      wl_cfgoce_has_ie(ie, tlvs, len, \
	(const uint8 *)WFA_OUI, WFA_OUI_LEN, WFA_OUI_TYPE_MBO_OCE)

/* Is any of the tlvs the expected entry? If
 * not update the tlvs buffer pointer/length.
 */
static bool
wl_cfgoce_has_ie(const u8 *ie, const u8 **tlvs, u32 *tlvs_len, const u8 *oui, u32 oui_len, u8 type)
{
	/* If the contents match the OUI and the type */
	if (ie[TLV_LEN_OFF] >= oui_len + 1 &&
			!bcmp(&ie[TLV_BODY_OFF], oui, oui_len) &&
			type == ie[TLV_BODY_OFF + oui_len]) {
		return TRUE;
	}

	return FALSE;
}
#endif /* WL_FW_OCE_AP_SELECT */

static bool check_dev_role_integrity(struct bcm_cfg80211 *cfg, u32 dev_role);

#define IS_WPA_AKM(akm) ((akm) == RSN_AKM_NONE ||			\
				 (akm) == RSN_AKM_UNSPECIFIED ||	\
				 (akm) == RSN_AKM_PSK)

#ifdef SUPPORT_AP_BWCTRL
static void
wl_update_apchan_bwcap(struct bcm_cfg80211 *cfg, struct net_device *ndev, chanspec_t chanspec);
#endif /* SUPPORT_AP_BWCTRL */

#ifdef SUPPORT_2G_VHT
extern int dhd_2g_vht;
#endif /* SUPPORT_2G_VHT */

#ifdef WL_STATIC_IF
#define WL_BSSIDX_MAX	16
#endif /* WL_STATIC_IF */

#if defined(WL_FW_OCE_AP_SELECT)
bool wl_cfg80211_is_oce_ap(struct wiphy *wiphy, const u8 *bssid_hint)
{
	const u8 *parse = NULL;
	bcm_tlv_t *ie;
	const struct cfg80211_bss_ies *ies;
	u32 len;
	struct cfg80211_bss *bss;

	bss = CFG80211_GET_BSS(wiphy, NULL, bssid_hint, 0, 0);
	if (!bss) {
		WL_ERR(("Unable to find AP in the cache"));
		return false;
	}

	if (rcu_access_pointer(bss->ies)) {
		ies = rcu_access_pointer(bss->ies);
		parse = ies->data;
		len = ies->len;
	} else {
		WL_ERR(("ies is NULL"));
		return false;
	}

	while ((ie = bcm_parse_tlvs(parse, len, DOT11_MNG_VS_ID))) {
		if (wl_cfgoce_is_oce_ie((const uint8*)ie, (u8 const **)&parse, &len) == TRUE) {
			return true;
		} else {
			ie = bcm_next_tlv((const bcm_tlv_t*) ie, &len);
			if (!ie) {
				return false;
			}
			parse = (uint8 *)ie;
			WL_DBG(("NON OCE IE. next ie ptr:%p", parse));
		}
	}
	WL_DBG(("OCE IE NOT found"));
	return false;
}
#endif /* WL_FW_OCE_AP_SELECT */

/* Dump the contents of the encoded wps ie buffer and get pbc value */
void
wl_validate_wps_ie(const char *wps_ie, s32 wps_ie_len, bool *pbc)
{
	#define WPS_IE_FIXED_LEN 6
	s16 len;
	const u8 *subel = NULL;
	u16 subelt_id;
	u16 subelt_len;
	u16 val;
	u8 *valptr = (uint8*) &val;
	if (wps_ie == NULL || wps_ie_len < WPS_IE_FIXED_LEN) {
		WL_ERR(("invalid argument : NULL\n"));
		return;
	}
	len = (s16)wps_ie[TLV_LEN_OFF];

	if (len > wps_ie_len) {
		WL_ERR(("invalid length len %d, wps ie len %d\n", len, wps_ie_len));
		return;
	}
	WL_DBG(("wps_ie len=%d\n", len));
	len -= 4;	/* for the WPS IE's OUI, oui_type fields */
	subel = wps_ie + WPS_IE_FIXED_LEN;
	while (len >= 4) {		/* must have attr id, attr len fields */
		valptr[0] = *subel++;
		valptr[1] = *subel++;
		subelt_id = HTON16(val);

		valptr[0] = *subel++;
		valptr[1] = *subel++;
		subelt_len = HTON16(val);

		len -= 4;			/* for the attr id, attr len fields */
		len -= (s16)subelt_len;	/* for the remaining fields in this attribute */
		if (len < 0) {
			break;
		}
		WL_DBG((" subel=%p, subelt_id=0x%x subelt_len=%u\n",
			subel, subelt_id, subelt_len));

		if (subelt_id == WPS_ID_VERSION) {
			WL_DBG(("  attr WPS_ID_VERSION: %u\n", *subel));
		} else if (subelt_id == WPS_ID_REQ_TYPE) {
			WL_DBG(("  attr WPS_ID_REQ_TYPE: %u\n", *subel));
		} else if (subelt_id == WPS_ID_CONFIG_METHODS) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_CONFIG_METHODS: %x\n", HTON16(val)));
		} else if (subelt_id == WPS_ID_DEVICE_NAME) {
			char devname[33];
			int namelen = MIN(subelt_len, (sizeof(devname) - 1));

			if (namelen) {
				memcpy(devname, subel, namelen);
				devname[namelen] = '\0';
				/* Printing len as rx'ed in the IE */
				WL_DBG(("  attr WPS_ID_DEVICE_NAME: %s (len %u)\n",
					devname, subelt_len));
			}
		} else if (subelt_id == WPS_ID_DEVICE_PWD_ID) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_DEVICE_PWD_ID: %u\n", HTON16(val)));
			*pbc = (HTON16(val) == DEV_PW_PUSHBUTTON) ? true : false;
		} else if (subelt_id == WPS_ID_PRIM_DEV_TYPE) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_PRIM_DEV_TYPE: cat=%u \n", HTON16(val)));
			valptr[0] = *(subel + 6);
			valptr[1] = *(subel + 7);
			WL_DBG(("  attr WPS_ID_PRIM_DEV_TYPE: subcat=%u\n", HTON16(val)));
		} else if (subelt_id == WPS_ID_REQ_DEV_TYPE) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_REQ_DEV_TYPE: cat=%u\n", HTON16(val)));
			valptr[0] = *(subel + 6);
			valptr[1] = *(subel + 7);
			WL_DBG(("  attr WPS_ID_REQ_DEV_TYPE: subcat=%u\n", HTON16(val)));
		} else if (subelt_id == WPS_ID_SELECTED_REGISTRAR_CONFIG_METHODS) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_SELECTED_REGISTRAR_CONFIG_METHODS"
				": cat=%u\n", HTON16(val)));
		} else {
			WL_DBG(("  unknown attr 0x%x\n", subelt_id));
		}

		subel += subelt_len;
	}
}

bool
wl_cfg80211_check_vif_in_use(struct net_device *ndev)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(ndev);
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);
	bool nan_enabled = FALSE;

#ifdef WL_NAN
	nan_enabled = wl_cfgnan_is_enabled(cfg);
#endif /* WL_NAN */

	if (nan_enabled || (wl_cfgp2p_vif_created(cfg)) ||
		(dhd->op_mode & DHD_FLAG_HOSTAP_MODE)) {
		WL_MEM(("%s: Virtual interfaces in use. NAN %d P2P %d softAP %d\n",
			__FUNCTION__, nan_enabled, wl_cfgp2p_vif_created(cfg),
			(dhd->op_mode & DHD_FLAG_HOSTAP_MODE)));
		return TRUE;
	}

	return FALSE;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
void
wl_cfgvif_delayed_remove_iface_work(struct work_struct *work)
{
	struct bcm_cfg80211 *cfg = NULL;
	struct net_device *ndev;

	BCM_SET_CONTAINER_OF(cfg, work, struct bcm_cfg80211, remove_iface_work.work);

	if (cfg->if_event_info.ifidx) {
		ndev = bcmcfg_to_prmry_ndev(cfg);

		dhd_net_if_lock(ndev);
		rtnl_lock();
		/* Remove interface except for primary ifidx */
		wl_cfg80211_remove_if(cfg, cfg->if_event_info.ifidx, ndev, FALSE);
		rtnl_unlock();

		dhd_net_if_unlock(ndev);
	}
	return;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0) */

#ifdef WL_IFACE_MGMT
/* Get active secondary data iface type */
wl_iftype_t
wl_cfg80211_get_sec_iface(struct bcm_cfg80211 *cfg)
{
#ifndef WL_STATIC_IF
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);
#endif /* !WL_STATIC_IF */
	struct net_device *p2p_ndev = NULL;

	p2p_ndev = wl_to_p2p_bss_ndev(cfg,
		P2PAPI_BSSCFG_CONNECTION1);

#ifdef WL_STATIC_IF
	if (IS_CFG80211_STATIC_IF_ACTIVE(cfg)) {
		if (IS_AP_IFACE(cfg->static_ndev->ieee80211_ptr)) {
			return WL_IF_TYPE_AP;
		}
	}
#else
	if (dhd->op_mode & DHD_FLAG_HOSTAP_MODE) {
		return WL_IF_TYPE_AP;
	}
#endif /* WL_STATIC_IF */

	if (p2p_ndev && p2p_ndev->ieee80211_ptr) {
		if (p2p_ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_GO) {
			return WL_IF_TYPE_P2P_GO;
		}

		/* Set role to GC when cfg80211 layer downgrades P2P
		 * role to station type while bringing down the interface
		 */
		if (p2p_ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_STATION) {
			WL_DBG_MEM(("%s, Change to GC base role\n", __FUNCTION__));
			return WL_IF_TYPE_P2P_GC;
		}

		if (p2p_ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_CLIENT) {
			return WL_IF_TYPE_P2P_GC;
		}
	}

#ifdef WL_NAN
	if (wl_cfgnan_is_dp_active(bcmcfg_to_prmry_ndev(cfg))) {
		return WL_IF_TYPE_NAN;
	}
#endif /* WL_NAN */
	return WL_IFACE_NOT_PRESENT;
}

/*
* Handle incoming data interface request based on policy.
* If there is any conflicting interface, that will be
* deleted.
*/
static s32
wl_cfg80211_data_if_mgmt(struct bcm_cfg80211 *cfg,
	wl_iftype_t new_wl_iftype)
{
	s32 ret = BCME_OK;
	bool del_iface = false;
	wl_iftype_t sec_wl_if_type = wl_cfg80211_get_sec_iface(cfg);

	if (sec_wl_if_type == WL_IF_TYPE_NAN &&
		new_wl_iftype == WL_IF_TYPE_NAN) {
		/* Multi NDP is allowed irrespective of Policy */
		return BCME_OK;
	}

	if (sec_wl_if_type == WL_IFACE_NOT_PRESENT) {
		/*
		* If there is no active secondary I/F, there
		* is no interface conflict. Do nothing.
		*/
		return BCME_OK;
	}

	/* Handle secondary data link case */
	switch (cfg->iface_data.policy) {
		case WL_IF_POLICY_CUSTOM:
		case WL_IF_POLICY_DEFAULT: {
			WL_INFORM_MEM(("%s, Delete any existing iface\n", __FUNCTION__));
			del_iface = true;
			break;
		}
		case WL_IF_POLICY_FCFS: {
			WL_INFORM_MEM(("Found active iface = %s, can't support new iface = %s\n",
				wl_iftype_to_str(sec_wl_if_type), wl_iftype_to_str(new_wl_iftype)));
			ret = BCME_ERROR;
			break;
		}
		case WL_IF_POLICY_LP: {
			WL_INFORM_MEM(("Remove active sec data interface, allow incoming iface\n"));
			/* Delete existing data iface and allow incoming sec iface */
			del_iface = true;
			break;
		}
		case WL_IF_POLICY_ROLE_PRIORITY: {
			WL_INFORM_MEM(("Existing iface = %s (%d) and new iface = %s (%d)\n",
				wl_iftype_to_str(sec_wl_if_type),
				cfg->iface_data.priority[sec_wl_if_type],
				wl_iftype_to_str(new_wl_iftype),
				cfg->iface_data.priority[new_wl_iftype]));
			if (cfg->iface_data.priority[new_wl_iftype] >
				cfg->iface_data.priority[sec_wl_if_type]) {
				del_iface = true;
			} else {
				WL_ERR(("Can't support new iface = %s\n",
					wl_iftype_to_str(new_wl_iftype)));
					ret = BCME_ERROR;
			}
			break;
		}
		default: {
			WL_ERR(("Unsupported interface policy = %d\n",
				cfg->iface_data.policy));
			return BCME_ERROR;
		}
	}
	if (del_iface) {
		ret = wl_cfg80211_delete_iface(cfg, sec_wl_if_type);
	}
	return ret;
}

/* Handle discovery ifaces based on policy */
static s32
wl_cfg80211_disc_if_mgmt(struct bcm_cfg80211 *cfg,
	wl_iftype_t new_wl_iftype, bool *disable_nan, bool *disable_p2p)
{
	s32 ret = BCME_OK;
	wl_iftype_t sec_wl_if_type =
		wl_cfg80211_get_sec_iface(cfg);
	*disable_p2p = false;
	*disable_nan = false;

	if (sec_wl_if_type == WL_IF_TYPE_NAN &&
			new_wl_iftype == WL_IF_TYPE_NAN) {
		/* Multi NDP is allowed irrespective of Policy */
		return BCME_OK;
	}

	/*
	* Check for any policy conflicts with active secondary
	* interface for incoming discovery iface
	*/
	if ((sec_wl_if_type != WL_IFACE_NOT_PRESENT) &&
		(is_discovery_iface(new_wl_iftype))) {
		switch (cfg->iface_data.policy) {
			case WL_IF_POLICY_CUSTOM: {
				if (sec_wl_if_type == WL_IF_TYPE_NAN &&
					new_wl_iftype == WL_IF_TYPE_P2P_DISC) {
					WL_INFORM_MEM(("Allow P2P Discovery with active NDP\n"));
					/* No further checks are required. */
					return BCME_OK;
				}
				/*
				* Intentional fall through to default policy
				* as for AP and associated ifaces, both are same
				*/
			}
				/* fall through */
				fallthrough;
			case WL_IF_POLICY_DEFAULT: {
				 if (sec_wl_if_type == WL_IF_TYPE_AP) {
					WL_INFORM_MEM(("AP is active, cant support new iface\n"));
					ret = BCME_ERROR;
				} else if (sec_wl_if_type == WL_IF_TYPE_P2P_GC ||
					sec_wl_if_type == WL_IF_TYPE_P2P_GO) {
					if (new_wl_iftype == WL_IF_TYPE_P2P_DISC) {
						/*
						* Associated discovery case,
						* Fall through
						*/
					} else {
						/* clear associated group interfaces */
						WL_INFORM_MEM(("remove P2P group,"
							" to support new iface\n"));
						ret = wl_cfg80211_delete_iface(cfg,
							sec_wl_if_type);
					}
				} else if (sec_wl_if_type == WL_IF_TYPE_NAN) {
					ret = wl_cfg80211_delete_iface(cfg, sec_wl_if_type);
				}
				break;
			}
			case WL_IF_POLICY_FCFS: {
				WL_INFORM_MEM(("Can't support new iface = %s\n",
						wl_iftype_to_str(new_wl_iftype)));
				ret = BCME_ERROR;
				break;
			}
			case WL_IF_POLICY_LP: {
				/* Delete existing data iface n allow incoming sec iface */
				WL_INFORM_MEM(("Remove active sec data interface = %s\n",
					wl_iftype_to_str(sec_wl_if_type)));
				ret = wl_cfg80211_delete_iface(cfg,
						sec_wl_if_type);
				break;
			}
			case WL_IF_POLICY_ROLE_PRIORITY: {
				WL_INFORM_MEM(("Existing iface = %s (%d) and new iface = %s (%d)\n",
					wl_iftype_to_str(sec_wl_if_type),
					cfg->iface_data.priority[sec_wl_if_type],
					wl_iftype_to_str(new_wl_iftype),
					cfg->iface_data.priority[new_wl_iftype]));
				if (cfg->iface_data.priority[new_wl_iftype] >
					cfg->iface_data.priority[sec_wl_if_type]) {
					WL_INFORM_MEM(("Remove active sec data iface\n"));
					ret = wl_cfg80211_delete_iface(cfg,
						sec_wl_if_type);
				} else {
					WL_ERR(("Can't support new iface = %s"
						" due to low priority\n",
						wl_iftype_to_str(new_wl_iftype)));
						ret = BCME_ERROR;
				}
				break;
			}
			default: {
				WL_ERR(("Unsupported policy\n"));
				return BCME_ERROR;
			}
		}
	} else {
		/*
		* Handle incoming new secondary iface request,
		* irrespective of existing discovery ifaces
		*/
		if ((cfg->iface_data.policy == WL_IF_POLICY_CUSTOM) &&
			(new_wl_iftype == WL_IF_TYPE_NAN)) {
			WL_INFORM_MEM(("Allow NAN Data Path\n"));
			/* No further checks are required. */
			return BCME_OK;
		}
	}

	/* Check for any conflicting discovery iface */
	switch (new_wl_iftype) {
		case WL_IF_TYPE_P2P_DISC:
		case WL_IF_TYPE_P2P_GO:
		case WL_IF_TYPE_P2P_GC: {
			*disable_nan = true;
			break;
		}
		case WL_IF_TYPE_NAN_NMI:
		case WL_IF_TYPE_NAN: {
			*disable_p2p = true;
			break;
		}
		case WL_IF_TYPE_STA:
		case WL_IF_TYPE_AP: {
			*disable_nan = true;
			*disable_p2p = true;
			break;
		}
		default: {
			WL_ERR(("Unsupported\n"));
			return BCME_ERROR;
		}
	}
	return ret;
}

static bool
wl_cfg80211_is_associated_discovery(struct bcm_cfg80211 *cfg,
	wl_iftype_t new_wl_iftype)
{
	struct net_device *p2p_ndev = NULL;
	p2p_ndev = wl_to_p2p_bss_ndev(cfg, P2PAPI_BSSCFG_CONNECTION1);

	if (new_wl_iftype == WL_IF_TYPE_P2P_DISC && p2p_ndev &&
		p2p_ndev->ieee80211_ptr &&
		is_p2p_group_iface(p2p_ndev->ieee80211_ptr)) {
			return true;
	}
#ifdef WL_NAN
	else if ((new_wl_iftype == WL_IF_TYPE_NAN_NMI) &&
		(wl_cfgnan_is_dp_active(bcmcfg_to_prmry_ndev(cfg)))) {
			return true;
		}
#endif /* WL_NAN */
	return false;
}

/* Handle incoming discovery iface request */
static s32
wl_cfg80211_handle_discovery_config(struct bcm_cfg80211 *cfg,
	wl_iftype_t new_wl_iftype)
{
	s32 ret = BCME_OK;
	bool disable_p2p = false;
	bool disable_nan = false;

	wl_iftype_t active_sec_iface =
		wl_cfg80211_get_sec_iface(cfg);

	if (is_discovery_iface(new_wl_iftype) &&
		(active_sec_iface != WL_IFACE_NOT_PRESENT)) {
		if (wl_cfg80211_is_associated_discovery(cfg,
			new_wl_iftype) == TRUE) {
			WL_DBG(("Associate iface request is allowed= %s\n",
				wl_iftype_to_str(new_wl_iftype)));
			return ret;
		}
	}

	ret = wl_cfg80211_disc_if_mgmt(cfg, new_wl_iftype,
			&disable_nan, &disable_p2p);
	if (ret != BCME_OK) {
		WL_ERR(("Failed at disc iface mgmt, ret = %d\n", ret));
		return ret;
	}

	if (disable_nan) {
#ifdef WL_NAN
		/* Disable nan to avoid conflict with p2p */
		ret = wl_cfgnan_check_nan_disable_pending(cfg, true, true);
		if (ret != BCME_OK) {
			WL_ERR(("failed to disable nan, error[%d]\n", ret));
			return ret;
		}
#endif /* WL_NAN */
	}

	if (disable_p2p) {
		/* Disable p2p discovery */
		ret = wl_cfg80211_deinit_p2p_discovery(cfg);
		if (ret != BCME_OK) {
			/* Should we fail nan enab here */
			WL_ERR(("Failed to disable p2p_disc for allowing nan\n"));
			return ret;
		}
	}
	return ret;
}

/*
* Check for any conflicting iface before adding iface.
* Based on policy, either conflicting iface is removed
* or new iface add request is blocked.
*/
s32
wl_cfg80211_handle_if_role_conflict(struct bcm_cfg80211 *cfg,
	wl_iftype_t new_wl_iftype)
{
	s32 ret = BCME_OK;

	WL_INFORM_MEM(("Incoming iface = %s\n", wl_iftype_to_str(new_wl_iftype)));

	if (!is_discovery_iface(new_wl_iftype)) {
		/* Incoming data interface request */
		if (wl_cfg80211_get_sec_iface(cfg) != WL_IFACE_NOT_PRESENT) {
			/* active interface present - Apply interface data policy */
			ret = wl_cfg80211_data_if_mgmt(cfg, new_wl_iftype);
			if (ret != BCME_OK) {
				WL_ERR(("if_mgmt fail:%d\n", ret));
				return ret;
			}
		}
	}
	/* Apply discovery config */
	ret = wl_cfg80211_handle_discovery_config(cfg, new_wl_iftype);
	return ret;
}
#endif /* WL_IFACE_MGMT */

s32
wl_release_vif_macaddr(struct bcm_cfg80211 *cfg, const u8 *mac_addr, u16 wl_iftype)
{
	struct net_device *ndev =  bcmcfg_to_prmry_ndev(cfg);
	u16 org_toggle_bytes;
	u16 cur_toggle_bytes;
	u16 toggled_bit;

	WL_DBG(("%s: ndev:%p type:%d mac:%p cfg:%p dev_addr:%p STATIC:%d\n",
		__func__, ndev, wl_iftype, mac_addr, cfg,
		(ndev?ndev->dev_addr:NULL), IS_CFG80211_STATIC_IF(cfg, ndev)));

	if (!ndev || !mac_addr || ETHER_ISNULLADDR(mac_addr)) {
		return -EINVAL;
	}
	WL_DBG(("%s:Mac addr" MACDBG "\n",
			__FUNCTION__, MAC2STRDBG(mac_addr)));

#if defined(SPECIFIC_MAC_GEN_SCHEME)
	if ((wl_iftype == WL_IF_TYPE_P2P_DISC) || (wl_iftype == WL_IF_TYPE_AP) ||
		(wl_iftype == WL_IF_TYPE_P2P_GO) || (wl_iftype == WL_IF_TYPE_P2P_GC)) {
		/* Avoid invoking release mac addr code for interfaces using
		 * fixed mac addr.
		 */
		return BCME_OK;
	}
#else
	if (wl_iftype == WL_IF_TYPE_P2P_DISC) {
		return BCME_OK;
	}
#endif /* SPECIFIC_MAC_GEN_SCHEME */

	if (
#ifdef WL_NAN
	!((cfg->nancfg->mac_rand) && (wl_iftype == WL_IF_TYPE_NAN)) &&
#endif /* WL_NAN */
#if defined(WL_STATIC_IF)
		/* interface delete is invalid for static interface */
		(!IS_CFG80211_STATIC_IF(cfg, ndev)) &&
#endif /* WL_STATIC_IF */
		ndev->dev_addr)
	{
		/* Fetch last two bytes of mac address */
		org_toggle_bytes = (*((const u16 *)&ndev->dev_addr[4]));
		org_toggle_bytes = ntoh16(org_toggle_bytes);
		cur_toggle_bytes = *((const u16 *)&mac_addr[4]);
		cur_toggle_bytes = ntoh16(cur_toggle_bytes);

		toggled_bit = (org_toggle_bytes ^ cur_toggle_bytes);
		WL_DBG(("org_toggle_bytes:%04X cur_toggle_bytes:%04X\n",
			org_toggle_bytes, cur_toggle_bytes));
		if (toggled_bit & cfg->vif_macaddr_mask) {
			/* This toggled_bit is marked in the used mac addr
			 * mask. Clear it.
			 */
			cfg->vif_macaddr_mask &= ~toggled_bit;
			WL_INFORM(("MAC address - " MACDBG " released. toggled_bit:%04X"
				" vif_mask:%04X\n",
				MAC2STRDBG(mac_addr), toggled_bit, cfg->vif_macaddr_mask));
		} else {
			WL_ERR(("MAC address - " MACDBG " not found in the used list."
				" toggled_bit:%04x vif_mask:%04x\n", MAC2STRDBG(mac_addr),
				toggled_bit, cfg->vif_macaddr_mask));
			return -EINVAL;
		}
	}

	return BCME_OK;
}

s32
wl_get_vif_macaddr(struct bcm_cfg80211 *cfg, u16 wl_iftype, u8 *mac_addr)
{
	struct ether_addr *p2p_dev_addr = NULL;
	struct net_device *ndev =  bcmcfg_to_prmry_ndev(cfg);
	u16 toggle_mask;
	u16 toggle_bit;
	u16 toggle_bytes;
	u16 used;
	u32 offset = 0;
	/* Toggle mask starts from MSB of second last byte */
	u16 mask = 0x8000;
	int i = 0;
	bool rand_mac = false;

	BCM_REFERENCE(i);
	BCM_REFERENCE(rand_mac);
	if (!mac_addr) {
		return -EINVAL;
	}

#ifdef WL_NAN
	rand_mac = cfg->nancfg->mac_rand;
	if (wl_iftype == WL_IF_TYPE_NAN && rand_mac) {
		/* ensure nmi != ndi */
		do {
			RANDOM_BYTES(mac_addr, ETHER_ADDR_LEN);
			/* restore mcast and local admin bits to 0 and 1 */
			ETHER_SET_UNICAST(mac_addr);
			ETHER_SET_LOCALADDR(mac_addr);
			i++;
			if (i == NAN_RAND_MAC_RETRIES) {
				break;
			}
		} while (eacmp(cfg->nancfg->nan_nmi_mac, mac_addr) == 0);

		if (i == NAN_RAND_MAC_RETRIES) {
			if (eacmp(cfg->nancfg->nan_nmi_mac, mac_addr) == 0) {
				WL_ERR(("\nCouldn't generate rand NDI which != NMI\n"));
				return BCME_NORESOURCE;
			}
		}
		return BCME_OK;
	}
#endif /* WL_NAN */

	if (cfg && cfg->p2p) {
		p2p_dev_addr = wl_to_p2p_bss_macaddr(cfg, P2PAPI_BSSCFG_DEVICE);
	}

	if ((wl_iftype == WL_IF_TYPE_P2P_DISC) && p2p_dev_addr &&
		ETHER_IS_LOCALADDR(p2p_dev_addr)) {
		/* If mac address is already generated return the mac */
		(void)memcpy_s(mac_addr, ETH_ALEN, p2p_dev_addr->octet, ETH_ALEN);
		return BCME_OK;
	}
	(void)memcpy_s(mac_addr, ETH_ALEN, ndev->perm_addr, ETH_ALEN);
/*
 * VIF MAC address managment
 * P2P Device addres: Primary MAC with locally admin. bit set
 * P2P Group address/NAN NMI/Softap/NAN DPI: Primary MAC addr
 *    with local admin bit set and one additional bit toggled.
 * cfg->vif_macaddr_mask will hold the info regarding the mac address
 * released. Ensure to call wl_release_vif_macaddress to free up
 * the mac address.
 */
#if defined(SPECIFIC_MAC_GEN_SCHEME)
	if (wl_iftype == WL_IF_TYPE_P2P_DISC ||	wl_iftype == WL_IF_TYPE_AP) {
		mac_addr[0] |= 0x02;
	} else if ((wl_iftype == WL_IF_TYPE_P2P_GO) || (wl_iftype == WL_IF_TYPE_P2P_GC)) {
		mac_addr[0] |= 0x02;
		mac_addr[4] ^= 0x80;
	}
#else
	if (wl_iftype == WL_IF_TYPE_P2P_DISC) {
		mac_addr[0] |= 0x02;
	}
#endif /* SPECIFIC_MAC_GEN_SCHEME */
	else {
		/* For locally administered mac addresses, we keep the
		 * OUI part constant and just work on the last two bytes.
		 */
		mac_addr[0] |= 0x02;
		toggle_mask = cfg->vif_macaddr_mask;
		toggle_bytes = ntoh16(*((u16 *)&mac_addr[4]));
		do {
			used = toggle_mask & mask;
			if (!used) {
				/* Use this bit position */
				toggle_bit = mask >> offset;
				toggle_bytes ^= toggle_bit;
				cfg->vif_macaddr_mask |= toggle_bit;
				WL_DBG(("toggle_bit:%04X toggle_bytes:%04X toggle_mask:%04X\n",
					toggle_bit, toggle_bytes, cfg->vif_macaddr_mask));
				/* Macaddress are stored in network order */
				mac_addr[5] = *((u8 *)&toggle_bytes);
				mac_addr[4] = *(((u8 *)&toggle_bytes + 1));
				break;
			}

			/* Shift by one */
			toggle_mask = toggle_mask << 0x1;
			offset++;
			if (offset > MAX_VIF_OFFSET) {
				/* We have used up all macaddresses. Something wrong! */
				WL_ERR(("Entire range of macaddress used up.\n"));
				ASSERT(0);
				break;
			}
		} while (true);
	}
	WL_INFORM_MEM(("Get virtual I/F mac addr: "MACDBG"\n", MAC2STRDBG(mac_addr)));
	return BCME_OK;
}

bcm_struct_cfgdev *
wl_cfg80211_add_virtual_iface(struct wiphy *wiphy,
#if defined(WL_CFG80211_P2P_DEV_IF)
	const char *name,
#else
	char *name,
#endif /* WL_CFG80211_P2P_DEV_IF */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
	unsigned char name_assign_type,
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)) */
	enum nl80211_iftype type,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0))
	u32 *flags,
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0) */
	struct vif_params *params)
{
	u16 wl_iftype;
	u16 wl_mode;
	struct net_device *primary_ndev;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	struct wireless_dev *wdev;
#if !defined(WL_STATIC_IF) && defined(APSTA_NON_CONCURRENT)
	int ret = BCME_OK;
#endif /* !WL_STATIC_IF && APSTA_NON_CONCURRENT */

	WL_DBG(("Enter iftype: %d\n", type));
	if (!cfg) {
		return ERR_PTR(-EINVAL);
	}

	/* Use primary I/F for sending cmds down to firmware */
	primary_ndev = bcmcfg_to_prmry_ndev(cfg);
	if (unlikely(!wl_get_drv_status(cfg, READY, primary_ndev))) {
#if !defined(WL_STATIC_IF) && defined(APSTA_NON_CONCURRENT)
		if (type == NL80211_IFTYPE_AP) {
			ret = dhd_open(primary_ndev);
			if (ret != BCME_OK) {
				WL_ERR(("%s, dhd_open failed\n", __FUNCTION__));
				return ERR_PTR(-ENODEV);
			}
		}
		else
#endif /* !WL_STATIC_IF && APSTA_NON_CONCURRENT */
		{
			WL_ERR(("device is not ready\n"));
			return ERR_PTR(-ENODEV);
		}
	}

	if (!name) {
		WL_ERR(("Interface name not provided \n"));
		return ERR_PTR(-EINVAL);
	}

	if (cfg80211_to_wl_iftype(type, &wl_iftype, &wl_mode) < 0) {
		return ERR_PTR(-EINVAL);
	}

	wdev = wl_cfg80211_add_if(cfg, primary_ndev, wl_iftype, name, NULL);
	if (unlikely(!wdev)) {
		return ERR_PTR(-ENODEV);
	}
	return wdev_to_cfgdev(wdev);
}

s32
wl_cfg80211_del_virtual_iface(struct wiphy *wiphy, bcm_struct_cfgdev *cfgdev)
{
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	struct wireless_dev *wdev = cfgdev_to_wdev(cfgdev);
	int ret = BCME_OK;
	u16 wl_iftype;
	u16 wl_mode;
	struct net_device *primary_ndev;

	if (!cfg) {
		return -EINVAL;
	}

	primary_ndev = bcmcfg_to_prmry_ndev(cfg);
	wdev = cfgdev_to_wdev(cfgdev);
	if (!wdev) {
		WL_ERR(("wdev null"));
		return -ENODEV;
	}

	WL_DBG(("Enter  wdev:%p iftype: %d\n", wdev, wdev->iftype));
	if (cfg80211_to_wl_iftype(wdev->iftype, &wl_iftype, &wl_mode) < 0) {
		WL_ERR(("Wrong iftype: %d\n", wdev->iftype));
		return -ENODEV;
	}

	if ((ret = wl_cfg80211_del_if(cfg, primary_ndev,
			wdev, NULL)) < 0) {
		WL_ERR(("IF del failed\n"));
	}

	return ret;
}

static s32
wl_cfg80211_change_p2prole(struct wiphy *wiphy, struct net_device *ndev, enum nl80211_iftype type)
{
	s32 wlif_type;
	s32 mode = 0;
	s32 index;
	s32 err;
	s32 conn_idx = -1;
	chanspec_t chspec;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	struct ether_addr p2p_dev_addr = {{0, 0, 0, 0, 0, 0}};
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);

	WL_INFORM_MEM(("Enter. current_role:%d new_role:%d \n", ndev->ieee80211_ptr->iftype, type));

	DEV_ADDR_GET(ndev, p2p_dev_addr.octet);

	if (!cfg->p2p || !wl_cfgp2p_vif_created(cfg)) {
		WL_ERR(("P2P not initialized \n"));
		return -EINVAL;
	}

	if (!is_p2p_group_iface(ndev->ieee80211_ptr)) {
		WL_ERR(("Wrong if type \n"));
		return -EINVAL;
	}

	/* Abort any on-going scans to avoid race condition issues */
	wl_cfgscan_cancel_scan(cfg);

	index = wl_get_bssidx_by_wdev(cfg, ndev->ieee80211_ptr);
	if (index < 0) {
		WL_ERR(("Find bsscfg index from ndev(%p) failed\n", ndev));
		return BCME_ERROR;
	}
	if (wl_cfgp2p_find_type(cfg, index, &conn_idx) != BCME_OK) {
		return BCME_ERROR;
	}

	/* In concurrency case, STA may be already associated in a particular
	 * channel. so retrieve the current channel of primary interface and
	 * then start the virtual interface on that.
	 */
	chspec = wl_cfg80211_get_shared_freq(wiphy);
	if (type == NL80211_IFTYPE_P2P_GO) {
		/* Dual p2p doesn't support multiple P2PGO interfaces,
		 * p2p_go_count is the counter for GO creation
		 * requests.
		 */
		if ((cfg->p2p->p2p_go_count > 0) && (type == NL80211_IFTYPE_P2P_GO)) {
			WL_ERR(("FW does not support multiple GO\n"));
			return BCME_ERROR;
		}
		mode = WL_MODE_AP;
		wlif_type = WL_P2P_IF_GO;
		dhd->op_mode &= ~DHD_FLAG_P2P_GC_MODE;
		dhd->op_mode |= DHD_FLAG_P2P_GO_MODE;
	} else {
		wlif_type = WL_P2P_IF_CLIENT;
		/* for GO */
		if (wl_get_mode_by_netdev(cfg, ndev) == WL_MODE_AP) {
			WL_INFORM_MEM(("Downgrading P2P GO to cfg_iftype:%d \n", type));
			wl_add_remove_eventmsg(ndev, WLC_E_PROBREQ_MSG, false);
			cfg->p2p->p2p_go_count--;
			/* disable interface before bsscfg free */
			err = wl_cfgp2p_ifdisable(cfg, &p2p_dev_addr);
			/* if fw doesn't support "ifdis",
			 * do not wait for link down of ap mode
			 */
			if (err == 0) {
				WL_DBG(("Wait for Link Down event for GO !!!\n"));
				wait_for_completion_timeout(&cfg->iface_disable,
					msecs_to_jiffies(500));
			} else if (err != BCME_UNSUPPORTED) {
				msleep(300);
			}
		}
	}

	wl_set_p2p_status(cfg, IF_CHANGING);
	wl_clr_p2p_status(cfg, IF_CHANGED);
	wl_cfgp2p_ifchange(cfg, &p2p_dev_addr,
		htod32(wlif_type), chspec, conn_idx);
	wait_event_interruptible_timeout(cfg->netif_change_event,
		(wl_get_p2p_status(cfg, IF_CHANGED) == true),
		msecs_to_jiffies(MAX_WAIT_TIME));

	wl_clr_p2p_status(cfg, IF_CHANGING);
	wl_clr_p2p_status(cfg, IF_CHANGED);

	if (mode == WL_MODE_AP) {
		wl_set_drv_status(cfg, CONNECTED, ndev);
	}

	return BCME_OK;
}

s32
wl_cfg80211_change_virtual_iface(struct wiphy *wiphy, struct net_device *ndev,
	enum nl80211_iftype type,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0))
	u32 *flags,
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0) */
	struct vif_params *params)
{
	s32 infra = 1;
	s32 err = BCME_OK;
	u16 wl_iftype;
	u16 wl_mode;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	struct net_info *netinfo = NULL;
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);
	struct net_device *primary_ndev;

	if (!dhd)
		return -EINVAL;

	WL_INFORM_MEM(("[%s] Enter. current cfg_iftype:%d new cfg_iftype:%d \n",
		ndev->name, ndev->ieee80211_ptr->iftype, type));
	primary_ndev = bcmcfg_to_prmry_ndev(cfg);

	if (cfg80211_to_wl_iftype(type, &wl_iftype, &wl_mode) < 0) {
		WL_ERR(("Unknown role \n"));
		return -EINVAL;
	}

	mutex_lock(&cfg->if_sync);
	netinfo = wl_get_netinfo_by_wdev(cfg, ndev->ieee80211_ptr);
	if (unlikely(!netinfo)) {
#ifdef WL_STATIC_IF
		if (IS_CFG80211_STATIC_IF(cfg, ndev)) {
			/* Incase of static interfaces, the netinfo will be
			 * allocated only when FW interface is initialized. So
			 * store the value and use it during initialization.
			 */
			WL_INFORM_MEM(("skip change vif for static if\n"));
			ndev->ieee80211_ptr->iftype = type;
			err = BCME_OK;
		} else
#endif /* WL_STATIC_IF */
		{
			WL_ERR(("netinfo not found \n"));
			err = -ENODEV;
		}
		goto fail;
	}

	if ((primary_ndev == ndev) && !(ndev->flags & IFF_UP)) {
		/*
		* If interface is not initialized, store the role and
		* return. The role will be initilized after interface
		* up
		*/
		WL_INFORM_MEM(("skip change role before dev up\n"));
		ndev->ieee80211_ptr->iftype = type;
		err = BCME_OK;
		goto fail;
	}

	/* perform pre-if-change tasks */
	wl_cfg80211_iface_state_ops(ndev->ieee80211_ptr,
		WL_IF_CHANGE_REQ, wl_iftype, wl_mode);

	if (dhd_query_bus_erros(dhd)) {
		WL_ERR(("bus error before changing role!\n"));
		err = -EINVAL;
		goto fail;
	}

	switch (type) {
	case NL80211_IFTYPE_ADHOC:
		infra = 0;
		break;
	case NL80211_IFTYPE_STATION:
		/* Supplicant sets iftype to STATION while removing p2p GO */
		if (ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_GO) {
			/* Downgrading P2P GO */
			err = wl_cfg80211_change_p2prole(wiphy, ndev, type);
			if (unlikely(err)) {
				WL_ERR(("P2P downgrade failed \n"));
			}
		} else if (ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {
			/* Downgrade role from AP to STA */
			if ((err = wl_cfg80211_add_del_bss(cfg, ndev,
				netinfo->bssidx, wl_iftype, 0, NULL)) < 0) {
				WL_ERR(("AP-STA Downgrade failed \n"));
				goto fail;
			}
		}
		break;
	case NL80211_IFTYPE_AP:
		/* intentional fall through */
	case NL80211_IFTYPE_AP_VLAN:
		{
			if (!wl_get_drv_status(cfg, AP_CREATED, ndev)) {
#if !defined(OEM_ANDROID)
				dhd->op_mode = DHD_FLAG_HOSTAP_MODE;
#endif /* BCMDONGLEHOST */
				err = wl_cfg80211_set_ap_role(cfg, ndev);
				if (unlikely(err)) {
					WL_ERR(("set ap role failed!\n"));
					goto fail;
				}
			} else {
				WL_INFORM_MEM(("AP_CREATED bit set. Skip role change\n"));
			}
			break;
		}
	case NL80211_IFTYPE_P2P_GO:
		/* Intentional fall through */
	case NL80211_IFTYPE_P2P_CLIENT:
		infra = 1;
		err = wl_cfg80211_change_p2prole(wiphy, ndev, type);
		break;
	case NL80211_IFTYPE_MONITOR:
#ifdef WL_CFG80211_MONITOR
		if (ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_STATION) {
			break;
		} else {
			WL_ERR(("Unsupported type:%d \n", type));
			err = -EINVAL;
			goto fail;
		}
#endif /* WL_CFG80211_MONITOR */
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MESH_POINT:
		/* Intentional fall through */
	default:
		WL_ERR(("Unsupported type:%d \n", type));
		err = -EINVAL;
		goto fail;
	}

	err = wldev_ioctl_set(ndev, WLC_SET_INFRA, &infra, sizeof(s32));
	if (err < 0) {
		WL_ERR(("SET INFRA/IBSS  error %d\n", err));
		goto fail;
	}

	wl_cfg80211_iface_state_ops(primary_ndev->ieee80211_ptr,
		WL_IF_CHANGE_DONE, wl_iftype, wl_mode);

	/* Update new iftype in relevant structures */
	if (is_p2p_group_iface(ndev->ieee80211_ptr) && (type == NL80211_IFTYPE_STATION)) {
		/* For role downgrade cases, we keep interface role as GC */
		netinfo->iftype = WL_IF_TYPE_P2P_GC;
		WL_DBG_MEM(("[%s] Set  base role to GC, current role"
			"ndev->ieee80211_ptr->iftype = %d\n",
			__FUNCTION__, ndev->ieee80211_ptr->iftype));
	} else {
		netinfo->iftype = wl_iftype;
	}

	ndev->ieee80211_ptr->iftype = type;

	WL_INFORM_MEM(("[%s] cfg_iftype changed to %d\n", ndev->name, type));

fail:
	if (err) {
		wl_flush_fw_log_buffer(ndev, FW_LOGSET_MASK_ALL);
	}
	mutex_unlock(&cfg->if_sync);
	return err;
}

/* This function will check the given chanspec with, FW supported
 * chanspec list, If the the chanspec is available will return
 * chanspec otherwise return zero.
 */
chanspec_t
wl_channel_to_chanspec(struct wiphy *wiphy, struct net_device *dev,
			chanspec_t chanspec, u32 bw_cap, u8 *_buf)
{
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	u8 *buf = NULL;
	uint buf_len = 1024 * 4;
	wl_uint32_list_t *list;
	u32 channel = 0;
	int err = BCME_OK;
	chanspec_t c = 0, ret_c = 0;
	int bw = 0, tmp_bw = 0;
	int i;
	u32 tmp_c;

	if (_buf) {
		/* accept if external caller provide the chanspecs */
		buf = _buf;
	} else {
		buf = (u8 *)MALLOC(cfg->osh, buf_len);
		if (!buf) {
			WL_ERR(("buf memory alloc failed\n"));
			goto exit;
		}

		err = wldev_iovar_getbuf_bsscfg(dev, "chanspecs", NULL,
			0, buf, buf_len, 0, &cfg->ioctl_buf_sync);
		if (err != BCME_OK) {
			WL_ERR(("get chanspecs failed with %d\n", err));
			goto exit;
		}
	}

	/* Retreive the channel from the chanspec */
	channel = CHSPEC_CHANNEL(chanspec);

	list = (wl_uint32_list_t *)(void *)buf;
	for (i = 0; i < dtoh32(list->count); i++) {
		c = dtoh32(list->element[i]);
		if ((CHSPEC_IS2G(chanspec)) &&
				(channel <= CH_MAX_2G_CHANNEL)) {
			if (!CHSPEC_IS20(c))
				continue;
			if (channel == CHSPEC_CHANNEL(c) &&
				(CHSPEC_BAND(c) == CHSPEC_BAND(chanspec))) {
				ret_c = c;
				bw = 20;
				goto exit;
			}
		}
		tmp_c = wf_chspec_ctlchan(c);
		tmp_bw = CHSPEC_BW(c);
		if (tmp_c != channel)
			continue;

		if ((tmp_bw > bw) && (tmp_bw <= bw_cap) &&
				(CHSPEC_BAND(c) == CHSPEC_BAND(chanspec))) {
			bw = tmp_bw;
			ret_c = c;
			if (bw == bw_cap) {
				goto exit;
			}
		}
	}

exit:
	if (!_buf) {
		if (buf) {
			MFREE(cfg->osh, buf, buf_len);
		}
	}

	WL_DBG(("return chanspec %x %d\n", ret_c, bw));
	return ret_c;
}

void
wl_cfg80211_cleanup_virtual_ifaces(struct bcm_cfg80211 *cfg, bool rtnl_lock_reqd)
{
	struct net_info *iter, *next;
	struct net_device *primary_ndev;

	/* Note: This function will clean up only the network interface and host
	 * data structures. The firmware interface clean up will happen in the
	 * during chip reset (ifconfig wlan0 down for built-in drivers/rmmod
	 * context for the module case).
	 */
	primary_ndev = bcmcfg_to_prmry_ndev(cfg);
	WL_DBG(("Enter\n"));
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	for_each_ndev(cfg, iter, next) {
		GCC_DIAGNOSTIC_POP();
		if (iter->ndev && (iter->ndev != primary_ndev)) {
			/* Ensure interfaces are down before deleting */
#ifdef WL_STATIC_IF
			/* Avoiding cleaning static ifaces */
			if (!IS_CFG80211_STATIC_IF(cfg, iter->ndev))
#endif /* WL_STATIC_IF */
			{
				dev_close(iter->ndev);
				WL_DBG(("Cleaning up iface:%s \n", iter->ndev->name));
				wl_cfg80211_post_ifdel(iter->ndev, rtnl_lock_reqd, 0);
			}
		}
	}
}

int
wl_get_bandwidth_cap(struct net_device *ndev, uint32 band, uint32 *bandwidth)
{
	u32 bw = WL_CHANSPEC_BW_20;
	s32 err = BCME_OK;
	s32 bw_cap = 0;
	struct {
		u32 band;
		u32 bw_cap;
	} param = {0, 0};
	u8 ioctl_buf[WLC_IOCTL_SMLEN];

	if (band == WL_CHANSPEC_BAND_2G) {
		param.band = WLC_BAND_2G;
	} else if (band == WL_CHANSPEC_BAND_5G) {
		param.band = WLC_BAND_5G;
	}
#ifdef WL_6G_BAND
	else if (band == WL_CHANSPEC_BAND_6G) {
		param.band = WLC_BAND_6G;
	}
#endif
	if (param.band) {
		/* bw_cap is newly defined iovar for checking bandwidth
		  * capability of the band in Aardvark_branch_tob
		  */
		err = wldev_iovar_getbuf(ndev, "bw_cap", &param, sizeof(param),
			ioctl_buf, sizeof(ioctl_buf), NULL);
		if (err) {
			if (err != BCME_UNSUPPORTED) {
				WL_ERR(("bw_cap failed, %d\n", err));
				return err;
			} else {
				/* if firmware doesn't support bw_cap iovar,
				 * we have to use mimo_bw_cap
				 */
				err = wldev_iovar_getint(ndev, "mimo_bw_cap", &bw_cap);
				if (err) {
					WL_ERR(("error get mimo_bw_cap (%d)\n", err));
				}
				if (bw_cap != WLC_N_BW_20ALL) {
					bw = WL_CHANSPEC_BW_40;
				}
			}
		} else {
			if (WL_BW_CAP_160MHZ(ioctl_buf[0])) {
				bw = WL_CHANSPEC_BW_160;
			} else if (WL_BW_CAP_80MHZ(ioctl_buf[0])) {
				bw = WL_CHANSPEC_BW_80;
			} else if (WL_BW_CAP_40MHZ(ioctl_buf[0])) {
				bw = WL_CHANSPEC_BW_40;
			} else {
				bw = WL_CHANSPEC_BW_20;
			}
		}
	} else if (band == WL_CHANSPEC_BAND_2G) {
		bw = WL_CHANSPEC_BW_20;
	}

	*bandwidth = bw;

	return err;
}

s32
wl_get_nl80211_band(u32 wl_band)
{
	s32 err = BCME_ERROR;

	switch (wl_band) {
		case WL_CHANSPEC_BAND_2G:
			return IEEE80211_BAND_2GHZ;
		case WL_CHANSPEC_BAND_5G:
			return IEEE80211_BAND_5GHZ;
#ifdef WL_BAND_6G
		case WL_CHANSPEC_BAND_6G:
			/* current kernels doesn't support seperate
			 * band for 6GHz. so till patch is available
			 * map it under 5GHz
			 */
			return IEEE80211_BAND_5GHZ;
#endif /* WL_BAND_6G */
		default:
			WL_ERR(("unsupported Band. %d\n", wl_band));
	}

	return err;
}

static struct ieee80211_channel *
wl_cfg80211_get_ieee80211_chan(struct net_device *dev,
	struct cfg80211_ap_settings *info)
{
	struct ieee80211_channel * ptr = NULL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
	if (info) {
		ptr = info->chandef.chan;
	}
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
	if (info) {
		ptr = info->channel;
	}
#else /* (LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)) */
	if ((dev) && (dev->ieee80211_ptr)) {
		ptr = dev->ieee80211_ptr->channel;
	}
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)) */

	return ptr;
}

static chanspec_t
wl_chspec_from_chandef(struct net_device *dev,
	struct cfg80211_ap_settings *info)
{
	chanspec_t  chspec = 0x0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
	/* kernel version > 3.8.0 supports 'struct cfg80211_chan_def'
	 * and the 'struct cfg80211_ap_settings' contains parameters
	 */

	struct cfg80211_chan_def * chandef;
	int                        band_width = 0;
	uint32                     primary_chanspec = 0x0;
	uint32                     center_chanspec = 0x0;

	if ((!info) || (!(info->chandef.chan))) {
		WL_ERR(("invalid parameters, info=0x%p!\n", info));
		return 0x0;
	} else {
		chandef = &info->chandef;
	}

	/* use the 'chan->center_freq' to get the the primary channel */
	if (chandef->chan->center_freq) {
		primary_chanspec = wl_freq_to_chanspec(chandef->chan->center_freq);
	}
	/* use the 'center_freq1' to get the the center channel, and refere to
	   kernel cfg80211 API function cfg80211_chandef_create()
	 */
	if (chandef->center_freq1) {
		center_chanspec  = wl_freq_to_chanspec(chandef->center_freq1);
	}
	/* unify/verify the bandwidth */
	band_width = chandef->width;
	if (WL_CHANSPEC_BAND_5G == CHSPEC_BAND(primary_chanspec)) {
		/* 5G BW160 should downgrade to 80MHz as channels falls in DFS */
		if (NL80211_CHAN_WIDTH_80 < band_width) {
			band_width = NL80211_CHAN_WIDTH_80;
			WL_TRACE(("%s-%d: force to BW80 for 5G, channel=%d!\n",
			          __FUNCTION__, __LINE__, CHSPEC_CHANNEL(chspec)));
		}
	}

	/* adjust according to kernel CFG80211 cfg80211_chandef_valid() */
	switch (band_width)
	{
		case NL80211_CHAN_WIDTH_160:
			chspec = wf_create_160MHz_chspec(
			               CHSPEC_CHANNEL(primary_chanspec),
			               CHSPEC_CHANNEL(center_chanspec),
			               CHSPEC_BAND(primary_chanspec));
			WL_TRACE(("%s-%d: BW160, pri=0x%X, center=0x%X -> chspec=0x%X\n",
			          __FUNCTION__, __LINE__,
			          primary_chanspec, center_chanspec, chspec));
			break;
		case NL80211_CHAN_WIDTH_80P80:
			/* Not-supported 8080 degrade to 80 */
		case NL80211_CHAN_WIDTH_80:
			chspec = wf_create_80MHz_chspec(
			              CHSPEC_CHANNEL(primary_chanspec),
			               CHSPEC_CHANNEL(center_chanspec),
			               CHSPEC_BAND(primary_chanspec));
			WL_TRACE(("%s-%d: BW80, pri=0x%X, center=0x%X -> chspec=0x%X\n",
			          __FUNCTION__, __LINE__,
			          primary_chanspec, center_chanspec, chspec));
			break;
		case NL80211_CHAN_WIDTH_40:
			chspec = wf_create_40MHz_chspec(
			               CHSPEC_CHANNEL(primary_chanspec),
			               CHSPEC_CHANNEL(center_chanspec),
			               CHSPEC_BAND(primary_chanspec));
			WL_TRACE(("%s-%d: BW40, pri=0x%X, center=0x%X -> chspec=0x%X\n",
			          __FUNCTION__, __LINE__,
			          primary_chanspec, center_chanspec, chspec));
			break;
		default:
			chspec = wf_create_20MHz_chspec(
			               CHSPEC_CHANNEL(primary_chanspec),
			               CHSPEC_BAND(primary_chanspec));
			WL_TRACE(("%s-%d: BW20, pri=0x%X, center=0x%X -> chspec=0x%X\n",
			          __FUNCTION__, __LINE__,
			          primary_chanspec, center_chanspec, chspec));
			break;
	}
	WL_TRACE(("%s-%d: from CFG80211: freq=%d, channel=%d, "
	          "width=%d(BW40/80/160=%d/%d/%d), "
	          "chspec_bw=0x%X(BW20/40/80/160=0x%X/0x%X/0x%X/0x%X)\n",
	          __FUNCTION__, __LINE__,
	          chandef->chan->center_freq, chandef->chan->hw_value,
	          chandef->width, NL80211_CHAN_WIDTH_40, NL80211_CHAN_WIDTH_80,
	          NL80211_CHAN_WIDTH_160,
	          CHSPEC_SB_LOWER(chspec), WL_CHANSPEC_BW_20, WL_CHANSPEC_BW_40,
	          WL_CHANSPEC_BW_80, WL_CHANSPEC_BW_160));

#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
	/* kernel version >= 3.6.0 only supports 'struct ieee80211_channel'
	 * and the 'struct cfg80211_ap_settings' only contains the
	 * 'channel' and 'channel_type' parameters
	 */

	struct ieee80211_channel * chan = NULL;
	uint32                     primary_chanspec = 0x0;
	uint32                     center_chanspec = 0x0;

	if (!info) {
		WL_ERR(("invalid parameters!\n"));
		return 0x0;
	} else {
		chan = info->channel;
	}

	/* 'channel_type' means secondary channel type */
	switch (info->channel_type) {
		case NL80211_CHAN_HT40MINUS:
			primary_chanspec = wl_freq_to_chanspec(chan->center_freq);
			center_chanspec  = wl_freq_to_chanspec(chan->center_freq - 20);
			chspec = wf_create_40MHz_chspec(
			               CHSPEC_CHANNEL(primary_chanspec),
			               CHSPEC_CHANNEL(center_chanspec),
			               CHSPEC_BAND(primary_chanspec));
			WL_TRACE(("%s-%d: BW40-, pri=0x%X, center=0x%X -> chspec=0x%X\n",
			          __FUNCTION__, __LINE__,
			          primary_chanspec, center_chanspec, chspec));
			break;
		case NL80211_CHAN_HT40PLUS:
			primary_chanspec = wl_freq_to_chanspec(chan->center_freq);
			center_chanspec  = wl_freq_to_chanspec(chan->center_freq + 20);
			chspec = wf_create_40MHz_chspec(
			               CHSPEC_CHANNEL(primary_chanspec),
			               CHSPEC_CHANNEL(center_chanspec),
			               CHSPEC_BAND(primary_chanspec));
			WL_TRACE(("%s-%d: BW40+, pri=0x%X, center=0x%X -> chspec=0x%X\n",
			          __FUNCTION__, __LINE__,
			          primary_chanspec, center_chanspec, chspec));
			break;
		default:
			chspec = wl_freq_to_chanspec(chan->center_freq);
			break;
	}
	WL_TRACE(("%s-%d: ieee80211_channel, center_freq=%d, new chspec=0x%X\n",
	          __FUNCTION__, __LINE__,
	          chan->center_freq, chspec));

#else /* (LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)) */
	/* kernel version < 3.6.0 only supports 'struct ieee80211_channel',
	 * and the 'struct cfg80211_ap_settings' doesn't contain the
	 * channel related information, and we try to get from 'wireless_dev'
	 */

	struct ieee80211_channel * chan = NULL;

	/* only 'ieee80211_channel' in 'dev->ieee80211_ptr' */
	if ((dev) && (dev->ieee80211_ptr) && (dev->ieee80211_ptr->channel)) {
		chan = dev->ieee80211_ptr->channel;
		chspec = wl_freq_to_chanspec(chan->center_freq);
		WL_TRACE(("%s-%d: dev->ieee80211_channel, center_freq=%d "
		          "-> chspec=0x%X\n",
		          __FUNCTION__, __LINE__, chan->center_freq, chspec));
	}

#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)) */

	return chspec;
}

#if defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE)
extern uint8 dhd_force_bw;
#ifdef DISABLE_MODE_CHANGE
extern int dhd_force_max_mode;
#endif /* DISABLE_MODE_CHANGE */
#endif /* defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE) */

s32
wl_cfg80211_set_chanspec(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_ap_settings *info)
{
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	struct ieee80211_channel *chan = NULL;
	chanspec_t  chspec = INVCHANSPEC;
	u32 band_width = WL_CHANSPEC_BW_20, bw = WL_CHANSPEC_BW_20;
	s32 err = BCME_OK;
#if defined(CUSTOM_SET_CPUCORE) || defined(APSTA_RESTRICTED_CHANNEL) || \
	defined(DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P)
	dhd_pub_t *dhd =  (dhd_pub_t *)(cfg->pub);
#endif /* CUSTOM_SET_CPUCORE || APSTA_RESTRICTED_CHANNEL */

	dev = ndev_to_wlc_ndev(dev, cfg);

	/* 1. decode the chanspec from upper layer */
	chspec = wl_chspec_from_chandef(dev, info);
	chan = wl_cfg80211_get_ieee80211_chan(dev, info);
	band_width = CHSPEC_BW(chspec);

#if defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE)
	bw = 0x0FFFF;
	/* 2.1 check if customer specify bandwidth */
	if (dhd_force_bw >= CH_BANDWIDTH_160MHZ) {
		bw = WL_CHANSPEC_BW_160;
	} else if (dhd_force_bw >= CH_BANDWIDTH_80MHZ) {
		bw = WL_CHANSPEC_BW_80;
	} else if (dhd_force_bw >= CH_BANDWIDTH_40MHZ) {
		bw = WL_CHANSPEC_BW_40;
	} else if (dhd_force_bw >= CH_BANDWIDTH_20MHZ) {
		bw = WL_CHANSPEC_BW_20;
	}
	if ((0x0FFFF != bw) && (0 < bw)) {
		WL_ERR(("force_bw=%d set from 0x%X to 0x%X, CHSPEC_BW=0x%X\n",
		        dhd_force_bw, band_width, bw, CHSPEC_BW(chspec)));
		band_width = bw;
	}
#endif /* defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE) */

	/* 2.2 finally verify the chipset capability */
	err = wl_get_bandwidth_cap(dev, CHSPEC_BAND(chspec), &bw);
	if (err < 0) {
		WL_ERR(("Failed to get bandwidth information, err=%d\n", err));
		return err;
	} else if (bw < band_width) {
		WL_ERR(("capability force band_width=0x%X to be 0x%X\n", band_width, bw));
		band_width = bw;
	}

	/* 3. get the new chanspec */
	if (CHSPEC_BW(chspec) != band_width) {
		chspec = wf_create_chspec_from_primary(
		                   wf_chspec_primary20_chan(chspec),
		                   band_width, CHSPEC_BAND(chspec));
		WL_ERR(("switch bw to 0x%X, chspec=0x%X\n", band_width, chspec));
	}

	/* 4. verify the chanspec since sometime the CLM may remove some chanpsec */
	if (wf_chspec_valid(chspec)) {
		wl_uint32_list_t  *pCH_supported = NULL;
		int                chanspec_total = 0;
		int                n = 0;

		// get FW supported chanspecs for comparison
		chanspec_total = sizeof(uint32) * (1 + MAXCHANNEL);
		pCH_supported = MALLOC(cfg->osh, chanspec_total);
		if (!pCH_supported) {
			WL_ERR(("*Warning, no memory for %d bytes chanspecs, skip\n",
			        chanspec_total));
		} else {
			n = wldev_iovar_getbuf(dev, "chanspecs", NULL, 0,
			           pCH_supported, chanspec_total, NULL);
			if (BCME_OK != n) {
				WL_ERR(("*Warning, get chanspecs failed with %d, skip\n", n));
			} else {
				pCH_supported->count = dtoh32(pCH_supported->count);
				for (n = 0;n < pCH_supported->count; n++) {
					if (pCH_supported->element[n] == chspec) {
						break;
					}
				}
				if (pCH_supported->count <= n) {
					WL_ERR(("*warning, chipset not supported chspec=0x%X\n",
					        chspec));
					chspec = 0x0;
				}
			}
		}
		if (pCH_supported) {
			MFREE(cfg->osh, pCH_supported, chanspec_total);
		}
	}
	if ((0x0 == chspec) || (!wf_chspec_valid(chspec))) {
		if (chan) {
			chspec = wl_freq_to_chanspec(chan->center_freq);
			WL_ERR(("*warning, try to recover to chspec=0x%X\n", chspec));
		}
	}

#ifdef WL_P2P_6G
	if (!(cfg->p2p_6g_enabled))
#endif /* WL_P2P_6G */
	{
		if (IS_P2P_GO(dev->ieee80211_ptr) && (CHSPEC_IS6G(chspec))) {
			WL_ERR(("P2P GO not allowed on 6G\n"));
			return -ENOTSUPP;
		}
	}

	/* 5. set chanspec to FW */
	if (wf_chspec_valid(chspec)) {
		/* convert 802.11 ac chanspec to current fw chanspec type */
		chspec = wl_chspec_host_to_driver(chspec);
		if (BCME_OK != (err = wldev_iovar_setint(dev, "chanspec", chspec))) {
			WL_ERR(("WLC_SET_CHANNEL error %d(BCME_BADCHAN=%d), "
			        "chip may not be supporting this channel\n",
			        err, BCME_BADCHAN));
		} else {
#ifdef DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P
			/* Disable Frameburst only for stand-alone 2GHz SoftAP/P2P GO*/
			if ((wl_get_mode_by_netdev(cfg, dev) == WL_MODE_AP) &&
				(dhd->disable_2g20_fb) &&
			    (DHD_OPMODE_SUPPORTED(cfg->pub, DHD_FLAG_HOSTAP_MODE) ||
			     DHD_OPMODE_SUPPORTED(cfg->pub, DHD_FLAG_P2P_MODE)) &&
			    (CHSPEC_IS2G(chspec)) && (band_width == WL_CHANSPEC_BW_20) &&
			    (!wl_get_drv_status(cfg, CONNECTED, bcmcfg_to_prmry_ndev(cfg)))
			   ) {
				WL_ERR(("Disabling frameburst on "
					"stand-alone 2GHz SoftAP/P2P GO\n"));
				wl_cfg80211_set_frameburst(cfg, FALSE);
			}
#endif /* DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P */

#ifdef CUSTOM_SET_CPUCORE
			if (dhd->op_mode == DHD_FLAG_HOSTAP_MODE) {
				WL_DBG(("SoftAP mode do not need to set cpucore\n"));
			} else if (chspec & WL_CHANSPEC_BW_80) {
				/* SoftAp only mode do not need to set cpucore */
				if ((dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) &&
					dev != bcmcfg_to_prmry_ndev(cfg)) {
					/* Soft AP on virtual Iface (AP+STA case) */
					dhd->chan_isvht80 |= DHD_FLAG_HOSTAP_MODE;
					dhd_set_cpucore(dhd, TRUE);
				} else if (is_p2p_group_iface(dev->ieee80211_ptr)) {
					/* If P2P IF is vht80 */
					dhd->chan_isvht80 |= DHD_FLAG_P2P_MODE;
					dhd_set_cpucore(dhd, TRUE);
				}
			}
#endif /* CUSTOM_SET_CPUCORE */
		}

		if (wl_get_mode_by_netdev(cfg, dev) == WL_MODE_AP) {
			/* Update AP/GO operating chanspec */
			cfg->ap_oper_channel = chspec;
		}
	} else {
		WL_ERR(("*Error, invalid chspec=0x%x\n", chspec));
		err = BCME_BADCHAN;
	}

	if (err) {
		wl_flush_fw_log_buffer(bcmcfg_to_prmry_ndev(cfg),
			FW_LOGSET_MASK_ALL);
	} else {
		WL_ERR(("Setting chanspec 0x%x for GO/AP \n", chspec));
	}
	return err;
}

s32
wl_cfg80211_set_channel(struct wiphy *wiphy, struct net_device *dev,
	struct ieee80211_channel *chan,
	enum nl80211_channel_type channel_type)
{
	chanspec_t chspec = INVCHANSPEC;
	chanspec_t cur_chspec = INVCHANSPEC;
	u32 bw = WL_CHANSPEC_BW_20;
	s32 err = BCME_OK;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
#if defined(CUSTOM_SET_CPUCORE) || defined(APSTA_RESTRICTED_CHANNEL) || \
	defined(DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P)
	dhd_pub_t *dhd =  (dhd_pub_t *)(cfg->pub);
#endif /* CUSTOM_SET_CPUCORE || APSTA_RESTRICTED_CHANNEL */

	dev = ndev_to_wlc_ndev(dev, cfg);
	chspec = wl_freq_to_chanspec(chan->center_freq);
	WL_ERR(("netdev_ifidx(%d), chan_type(%d) target channel(%d) \n",
		dev->ifindex, channel_type, CHSPEC_CHANNEL(chspec)));

#ifdef WL_P2P_6G
	if (!(cfg->p2p_6g_enabled))
#endif /* WL_P2P_6G */
	{
		if (IS_P2P_GO(dev->ieee80211_ptr) && (CHSPEC_IS6G(chspec))) {
			WL_ERR(("P2P GO not allowed on 6G\n"));
			return -ENOTSUPP;
		}
	}

#if defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE)
	switch (dhd_force_bw) {
		case CH_BANDWIDTH_20MHZ:
			WL_ERR(("Force BW to 20\n"));
			bw = WL_CHANSPEC_BW_20;
			break;
		case CH_BANDWIDTH_40MHZ:
			WL_ERR(("Force BW to 40\n"));
			bw = WL_CHANSPEC_BW_40;
			break;
		case CH_BANDWIDTH_80MHZ:
			WL_ERR(("Force BW to 80\n"));
			bw = WL_CHANSPEC_BW_80;
			break;
	}

	goto set_channel;
#endif /* defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE) */

#if defined(APSTA_RESTRICTED_CHANNEL)
	/* Some customer platform used limited number of channels
	 * for SoftAP interface on STA/SoftAP concurrent mode.
	 * - 2.4GHz Channel: CH1 - CH13
	 * - 5GHz Channel: CH149 (it depends on the country code)
	 * If the Android framework sent invaild channel configuration
	 * to DHD, driver should change the channel which is sutible for
	 * STA/SoftAP concurrent mode.
	 * - Set operating channel to CH1 (default 2.4GHz channel for
	 *   restricted APSTA mode) if STA interface was associated to
	 *   5GHz APs except for CH149.
	 * - Otherwise, set the channel to the same channel as existing AP.
	 */
	if (wl_get_mode_by_netdev(cfg, dev) == WL_MODE_AP &&
		DHD_OPMODE_STA_SOFTAP_CONCURR(dhd) &&
		wl_get_drv_status(cfg, CONNECTED, bcmcfg_to_prmry_ndev(cfg))) {
		u32 *sta_chanspec = (u32 *)wl_read_prof(cfg,
			bcmcfg_to_prmry_ndev(cfg), WL_PROF_CHAN);
		if (chan->band == wl_get_nl80211_band(CHSPEC_BAND(*sta_chanspec))) {
			/* Do not try SCC in 5GHz if channel is not CH149 */
			chspec = (
#ifdef WL_6G_BAND
				CHSPEC_IS6G(*sta_chanspec) ||
#endif /* WL_6G_BAND */
				(CHSPEC_IS5G(*sta_chanspec) &&
				wf_chspec_primary20_chan(*sta_chanspec) !=
				DEFAULT_5G_SOFTAP_CHANNEL)) ?
				DEFAULT_2G_SOFTAP_CHANSPEC: *sta_chanspec;
			WL_ERR(("target chanspec will be changed to %x\n", chspec));
			if (CHSPEC_IS2G(chspec)) {
				bw = WL_CHANSPEC_BW_20;
				goto set_channel;
			}
		}
	}
#endif /* APSTA_RESTRICTED_CHANNEL */

	err = wl_get_bandwidth_cap(dev, CHSPEC_BAND(chspec), &bw);
	if (err < 0) {
		WL_ERR(("Failed to get bandwidth information, err=%d\n", err));
		return err;
	}

	/* In case of 5G downgrade BW to 80MHz as 160MHz channels falls in DFS */
	if (CHSPEC_IS5G(chspec) && (bw == WL_CHANSPEC_BW_160)) {
		bw = WL_CHANSPEC_BW_80;
	}
set_channel:
	cur_chspec = wf_create_chspec_from_primary(wf_chspec_primary20_chan(chspec),
		bw, CHSPEC_BAND(chspec));
#ifdef WL_6G_BAND
	if (cfg->acs_chspec &&
		CHSPEC_IS6G(cfg->acs_chspec) &&
		(wf_chspec_ctlchspec(cfg->acs_chspec) == wf_chspec_ctlchspec(cur_chspec))) {
		WL_DBG(("using acs_chanspec %x\n", cfg->acs_chspec));
		cur_chspec = cfg->acs_chspec;
		cfg->acs_chspec = 0;
	}
#endif /* WL_6G_BAND */
	if (wf_chspec_valid(cur_chspec)) {
		/* convert 802.11 ac chanspec to current fw chanspec type */
		cur_chspec = wl_chspec_host_to_driver(cur_chspec);
		if (cur_chspec != INVCHANSPEC) {
			if ((err = wldev_iovar_setint(dev, "chanspec",
				cur_chspec)) == BCME_BADCHAN) {
				u32 local_channel = CHSPEC_CHANNEL(chspec);
				if ((bw == WL_CHANSPEC_BW_80) || (bw == WL_CHANSPEC_BW_160))
					goto change_bw;
				err = wldev_ioctl_set(dev, WLC_SET_CHANNEL,
					&local_channel, sizeof(local_channel));
				if (err < 0) {
					WL_ERR(("WLC_SET_CHANNEL error %d"
					"chip may not be supporting this channel\n", err));
				}
			} else if (err) {
				WL_ERR(("failed to set chanspec error %d\n", err));
			}
#ifdef DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P
			else {
				/* Disable Frameburst only for stand-alone 2GHz SoftAP */
				if (dhd->disable_2g20_fb &&
					wl_get_mode_by_netdev(cfg, dev) == WL_MODE_AP &&
					DHD_OPMODE_SUPPORTED(cfg->pub, DHD_FLAG_HOSTAP_MODE) &&
					(CHSPEC_IS2G(chspec)) &&
					!wl_get_drv_status(cfg, CONNECTED,
						bcmcfg_to_prmry_ndev(cfg))) {
					WL_DBG(("Disabling frameburst on "
						"stand-alone 2GHz SoftAP\n"));
					wl_cfg80211_set_frameburst(cfg, FALSE);
				}
			}
#endif /* DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P */
		} else {
			WL_ERR(("failed to convert host chanspec to fw chanspec\n"));
			err = BCME_ERROR;
		}
	} else {
change_bw:
		if (bw == WL_CHANSPEC_BW_160) {
			bw = WL_CHANSPEC_BW_80;
		} else if (bw == WL_CHANSPEC_BW_80) {
			bw = WL_CHANSPEC_BW_40;
		} else if (bw == WL_CHANSPEC_BW_40) {
			bw = WL_CHANSPEC_BW_20;
		} else {
			bw = 0;
		}
		if (bw)
			goto set_channel;
		WL_ERR(("Invalid chanspec 0x%x\n", chspec));
		err = BCME_ERROR;
	}
#ifdef CUSTOM_SET_CPUCORE
	if (dhd->op_mode == DHD_FLAG_HOSTAP_MODE) {
		WL_DBG(("SoftAP mode do not need to set cpucore\n"));
	} else if (chspec & WL_CHANSPEC_BW_80) {
		/* SoftAp only mode do not need to set cpucore */
		if ((dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) &&
			dev != bcmcfg_to_prmry_ndev(cfg)) {
			/* Soft AP on virtual Iface (AP+STA case) */
			dhd->chan_isvht80 |= DHD_FLAG_HOSTAP_MODE;
			dhd_set_cpucore(dhd, TRUE);
		} else if (is_p2p_group_iface(dev->ieee80211_ptr)) {
			/* If P2P IF is vht80 */
			dhd->chan_isvht80 |= DHD_FLAG_P2P_MODE;
			dhd_set_cpucore(dhd, TRUE);
		}
	}
#endif /* CUSTOM_SET_CPUCORE */
	if (!err && (wl_get_mode_by_netdev(cfg, dev) == WL_MODE_AP)) {
		/* Update AP/GO operating chanspec */
		cfg->ap_oper_channel = wl_freq_to_chanspec(chan->center_freq);
	}
	if (err) {
		wl_flush_fw_log_buffer(bcmcfg_to_prmry_ndev(cfg),
			FW_LOGSET_MASK_ALL);
	} else {
		WL_DBG(("Setting chanspec %x for GO/AP \n", chspec));
	}
	return err;
}

static s32
wl_validate_opensecurity(struct net_device *dev, s32 bssidx, bool privacy)
{
	s32 err = BCME_OK;
	u32 wpa_val;
	s32 wsec = 0;

	/* set auth */
	err = wldev_iovar_setint_bsscfg(dev, "auth", 0, bssidx);
	if (err < 0) {
		WL_ERR(("auth error %d\n", err));
		return BCME_ERROR;
	}

	if (privacy) {
		/* If privacy bit is set in open mode, then WEP would be enabled */
		wsec = WEP_ENABLED;
		WL_DBG(("Setting wsec to %d for WEP \n", wsec));
	}

	/* set wsec */
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (err < 0) {
		WL_ERR(("wsec error %d\n", err));
		return BCME_ERROR;
	}

	/* set upper-layer auth */
	if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_ADHOC)
		wpa_val = WPA_AUTH_NONE;
	else
		wpa_val = WPA_AUTH_DISABLED;
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", wpa_val, bssidx);
	if (err < 0) {
		WL_ERR(("wpa_auth error %d\n", err));
		return BCME_ERROR;
	}

	return 0;
}

#define MAX_FILS_IND_IE_LEN 1024u
static s32
wl_validate_fils_ind_ie(struct net_device *dev, const bcm_tlv_t *filsindie, s32 bssidx)
{
	s32 err = BCME_OK;
	struct bcm_cfg80211 *cfg = NULL;
	bcm_iov_buf_t *iov_buf = NULL;
	bcm_xtlv_t* pxtlv;
	int iov_buf_size = 0;

	if (!dev || !filsindie) {
		WL_ERR(("%s: dev/filsidie is null\n", __FUNCTION__));
		goto exit;
	}

	cfg = wl_get_cfg(dev);
	if (!cfg) {
		WL_ERR(("%s: cfg is null\n", __FUNCTION__));
		goto exit;
	}

	iov_buf_size = sizeof(bcm_iov_buf_t) + sizeof(bcm_xtlv_t) + filsindie->len - 1;
	iov_buf = MALLOCZ(cfg->osh, iov_buf_size);
	if (!iov_buf) {
		WL_ERR(("%s: iov_buf alloc failed! %d bytes\n", __FUNCTION__, iov_buf_size));
		err = BCME_NOMEM;
		goto exit;
	}
	iov_buf->version = WL_FILS_IOV_VERSION;
	iov_buf->id = WL_FILS_CMD_ADD_IND_IE;
	iov_buf->len = sizeof(bcm_xtlv_t) + filsindie->len - 1;
	pxtlv = (bcm_xtlv_t*)&iov_buf->data[0];
	pxtlv->id = WL_FILS_XTLV_IND_IE;
	pxtlv->len = filsindie->len;
	/* memcpy_s return check not required as buffer is allocated based on ie
	 * len
	 */
	(void)memcpy_s(pxtlv->data, filsindie->len, filsindie->data, filsindie->len);

	err = wldev_iovar_setbuf(dev, "fils", iov_buf, iov_buf_size,
		cfg->ioctl_buf, WLC_IOCTL_SMLEN, &cfg->ioctl_buf_sync);
	if (unlikely(err)) {
		WL_ERR(("fils indication ioctl error (%d)\n", err));
		 goto exit;
	}

exit:
	if (err < 0) {
		WL_ERR(("FILS Ind setting error %d\n", err));
	}

	if (iov_buf) {
		MFREE(cfg->osh, iov_buf, iov_buf_size);
	}
	return err;
}

#ifdef MFP
static int
wl_get_mfp_capability(u8 rsn_cap, u32 *wpa_auth, u32 *mfp_val)
{
	u32 mfp = 0;
	if (rsn_cap & RSN_CAP_MFPR) {
		WL_DBG(("MFP Required \n"));
		mfp = WL_MFP_REQUIRED;
		/* Our firmware has requirement that WPA2_AUTH_PSK/WPA2_AUTH_UNSPECIFIED
		 * be set, if SHA256 OUI is to be included in the rsn ie.
		 */
		if (*wpa_auth & WPA2_AUTH_PSK_SHA256) {
			*wpa_auth |= WPA2_AUTH_PSK;
		} else if (*wpa_auth & WPA2_AUTH_1X_SHA256) {
			*wpa_auth |= WPA2_AUTH_UNSPECIFIED;
		}
	} else if (rsn_cap & RSN_CAP_MFPC) {
		WL_DBG(("MFP Capable \n"));
		mfp = WL_MFP_CAPABLE;
	}

	/* Validate MFP */
	if ((*wpa_auth == WPA3_AUTH_SAE_PSK) && (mfp != WL_MFP_REQUIRED)) {
		WL_ERR(("MFPR should be set for SAE PSK. mfp:%d\n", mfp));
		return BCME_ERROR;
	} else if ((*wpa_auth == (WPA3_AUTH_SAE_PSK | WPA2_AUTH_PSK)) &&
		(mfp != WL_MFP_CAPABLE)) {
		WL_ERR(("mfp(%d) should be set to capable(%d) for SAE transition mode\n",
				mfp, WL_MFP_CAPABLE));
		return BCME_ERROR;
	}

	*mfp_val = mfp;
	return BCME_OK;
}
#endif /* MFP */

static s32
wl_validate_wpa2ie(struct net_device *dev, const bcm_tlv_t *wpa2ie, s32 bssidx)
{
	s32 len = 0;
	s32 err = BCME_OK;
	u16 auth = 0; /* d11 open authentication */
	u32 wsec;
	u32 pval = 0;
	u32 gval = 0;
	u32 wpa_auth = 0;
	const wpa_suite_mcast_t *mcast;
	const wpa_suite_ucast_t *ucast;
	const wpa_suite_auth_key_mgmt_t *mgmt;
	const wpa_pmkid_list_t *pmkid;
	int cnt = 0;
#ifdef MFP
	u32 mfp = 0;
#endif /* MFP */
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	struct wl_security *sec = wl_read_prof(cfg, dev, WL_PROF_SEC);

	u16 suite_count;
	u8 rsn_cap[2];
	u32 rsn_cap_value = 0x0;
	u32 wme_bss_disable;

	if (wpa2ie == NULL)
		goto exit;

	WL_DBG(("Enter \n"));
	len =  wpa2ie->len - WPA2_VERSION_LEN;
	/* check the mcast cipher */
	mcast = (const wpa_suite_mcast_t *)&wpa2ie->data[WPA2_VERSION_LEN];
	switch (mcast->type) {
		case WPA_CIPHER_NONE:
			gval = 0;
			break;
		case WPA_CIPHER_WEP_40:
		case WPA_CIPHER_WEP_104:
			gval = WEP_ENABLED;
			break;
		case WPA_CIPHER_TKIP:
			gval = TKIP_ENABLED;
			break;
		case WPA_CIPHER_AES_CCM:
			gval = AES_ENABLED;
			break;

#ifdef BCMWAPI_WPI
		case WAPI_CIPHER_SMS4:
			gval = SMS4_ENABLED;
			break;
#endif

		default:
			WL_ERR(("No Security Info\n"));
			break;
	}
	if ((len -= WPA_SUITE_LEN) <= 0)
		return BCME_BADLEN;

	/* check the unicast cipher */
	ucast = (const wpa_suite_ucast_t *)&mcast[1];
	suite_count = ltoh16_ua(&ucast->count);
	switch (ucast->list[0].type) {
		case WPA_CIPHER_NONE:
			pval = 0;
			break;
		case WPA_CIPHER_WEP_40:
		case WPA_CIPHER_WEP_104:
			pval = WEP_ENABLED;
			break;
		case WPA_CIPHER_TKIP:
			pval = TKIP_ENABLED;
			break;
		case WPA_CIPHER_AES_CCM:
			pval = AES_ENABLED;
			break;

#ifdef BCMWAPI_WPI
		case WAPI_CIPHER_SMS4:
			pval = SMS4_ENABLED;
			break;
#endif

		default:
			WL_ERR(("No Security Info\n"));
	}
	if ((len -= (WPA_IE_SUITE_COUNT_LEN + (WPA_SUITE_LEN * suite_count))) <= 0)
		return BCME_BADLEN;

	/* FOR WPS , set SEC_OW_ENABLED */
	wsec = (pval | gval | SES_OW_ENABLED);
	/* check the AKM */
	mgmt = (const wpa_suite_auth_key_mgmt_t *)&ucast->list[suite_count];
	suite_count = cnt = ltoh16_ua(&mgmt->count);
	while (cnt--) {
		if (bcmp(mgmt->list[cnt].oui, WFA_OUI, WFA_OUI_LEN) == 0) {
			switch (mgmt->list[cnt].type) {
			case RSN_AKM_DPP:
				wpa_auth |= WPA3_AUTH_DPP_AKM;
				break;
			default:
				WL_ERR(("No Key Mgmt Info in WFA_OUI\n"));
			}
		} else {
			switch (mgmt->list[cnt].type) {
			case RSN_AKM_NONE:
				wpa_auth |= WPA_AUTH_NONE;
				break;
			case RSN_AKM_UNSPECIFIED:
				wpa_auth |= WPA2_AUTH_UNSPECIFIED;
				break;
			case RSN_AKM_PSK:
				wpa_auth |= WPA2_AUTH_PSK;
				break;
#ifdef MFP
			case RSN_AKM_MFP_PSK:
				wpa_auth |= WPA2_AUTH_PSK_SHA256;
				break;
			case RSN_AKM_MFP_1X:
				wpa_auth |= WPA2_AUTH_1X_SHA256;
				break;
			case RSN_AKM_FILS_SHA256:
				wpa_auth |= WPA2_AUTH_FILS_SHA256;
				break;
			case RSN_AKM_FILS_SHA384:
				wpa_auth |= WPA2_AUTH_FILS_SHA384;
				break;
#if defined(WL_SAE) || defined(WL_CLIENT_SAE)
			case RSN_AKM_SAE_PSK:
				wpa_auth |= WPA3_AUTH_SAE_PSK;
				break;
#endif /* WL_SAE || WL_CLIENT_SAE */
#endif /* MFP */
#if defined(WL_OWE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0))
			case RSN_AKM_OWE:
				wpa_auth |= WPA3_AUTH_OWE;
				break;
#endif /* WL_OWE && LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0) */
			default:
				WL_ERR(("No Key Mgmt Info\n"));
			}
		}
	}
	if ((len -= (WPA_IE_SUITE_COUNT_LEN + (WPA_SUITE_LEN * suite_count))) >= RSN_CAP_LEN) {
		rsn_cap[0] = *(const u8 *)&mgmt->list[suite_count];
		rsn_cap[1] = *((const u8 *)&mgmt->list[suite_count] + 1);

		if (rsn_cap[0] & (RSN_CAP_16_REPLAY_CNTRS << RSN_CAP_PTK_REPLAY_CNTR_SHIFT)) {
			wme_bss_disable = 0;
		} else {
			wme_bss_disable = 1;
		}

#ifdef MFP
		if (wl_get_mfp_capability(rsn_cap[0], &wpa_auth, &mfp) != BCME_OK) {
			WL_ERR(("mfp configuration invalid. rsn_cap:0x%x\n", rsn_cap[0]));
			return BCME_ERROR;
		}
#endif /* MFP */

		/* set wme_bss_disable to sync RSN Capabilities */
		err = wldev_iovar_setint_bsscfg(dev, "wme_bss_disable", wme_bss_disable, bssidx);
		if (err < 0) {
			WL_ERR(("wme_bss_disable error %d\n", err));
			return BCME_ERROR;
		}

		/* sync up the RSN Capabilities */
		rsn_cap_value = (0x0ff & rsn_cap[0]) | (0x0ff00 & (rsn_cap[1] << 8));
		err = wldev_iovar_setint_bsscfg(dev, "wpa_cap", rsn_cap_value, bssidx);
		if (err < 0) {
			WL_ERR(("rsn_cap_value error %d\n", err));
			return BCME_ERROR;
		}
	} else {
		WL_DBG(("There is no RSN Capabilities. remained len %d\n", len));
	}

	len -= RSN_CAP_LEN;
	if (len >= WPA2_PMKID_COUNT_LEN) {
		pmkid = (const wpa_pmkid_list_t *)
		        ((const u8 *)&mgmt->list[suite_count] + RSN_CAP_LEN);
		cnt = ltoh16_ua(&pmkid->count);
		if (cnt != 0) {
			WL_ERR(("AP has non-zero PMKID count. Wrong!\n"));
			return BCME_ERROR;
		}
		/* since PMKID cnt is known to be 0 for AP, */
		/* so don't bother to send down this info to firmware */
	}

#ifdef MFP
	len -= WPA2_PMKID_COUNT_LEN;
	if (len >= WPA_SUITE_LEN) {
		cfg->bip_pos =
		        (const u8 *)&mgmt->list[suite_count] + RSN_CAP_LEN + WPA2_PMKID_COUNT_LEN;
	} else {
		cfg->bip_pos = NULL;
	}
#endif

	/* set auth */
	err = wldev_iovar_setint_bsscfg(dev, "auth", auth, bssidx);
	if (err < 0) {
		WL_ERR(("auth error %d\n", err));
		return BCME_ERROR;
	}

	/* set wsec */
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (err < 0) {
		WL_ERR(("wsec error %d\n", err));
		return BCME_ERROR;
	}

#ifdef MFP
	cfg->mfp_mode = mfp;
#endif /* MFP */

	/* set upper-layer auth */
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", wpa_auth, bssidx);
	if (err < 0) {
		WL_ERR(("wpa_auth error %d\n", err));
		return BCME_ERROR;
	}

	if (sec) {
		/* store applied sec settings */
		sec->fw_wpa_auth = wpa_auth;
		sec->fw_wsec = wsec;
		sec->fw_auth = auth;
#ifdef MFP
		sec->fw_mfp = mfp;
#endif /* mfp */
	}
exit:
	return 0;
}

static s32
wl_validate_wpaie(struct net_device *dev, const wpa_ie_fixed_t *wpaie, s32 bssidx)
{
	const wpa_suite_mcast_t *mcast;
	const wpa_suite_ucast_t *ucast;
	const wpa_suite_auth_key_mgmt_t *mgmt;
	u16 auth = 0; /* d11 open authentication */
	u16 count;
	s32 err = BCME_OK;
	s32 len = 0;
	u32 i;
	u32 wsec;
	u32 pval = 0;
	u32 gval = 0;
	u32 wpa_auth = 0;
	u32 tmp = 0;
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	struct wl_security *sec = wl_read_prof(cfg, dev, WL_PROF_SEC);

	if (wpaie == NULL)
		goto exit;
	WL_DBG(("Enter \n"));
	len = wpaie->length;    /* value length */
	len -= WPA_IE_TAG_FIXED_LEN;
	/* check for multicast cipher suite */
	if (len < WPA_SUITE_LEN) {
		WL_INFORM_MEM(("no multicast cipher suite\n"));
		goto exit;
	}

	/* pick up multicast cipher */
	mcast = (const wpa_suite_mcast_t *)&wpaie[1];
	len -= WPA_SUITE_LEN;
	if (!bcmp(mcast->oui, WPA_OUI, WPA_OUI_LEN)) {
		if (IS_WPA_CIPHER(mcast->type)) {
			tmp = 0;
			switch (mcast->type) {
				case WPA_CIPHER_NONE:
					tmp = 0;
					break;
				case WPA_CIPHER_WEP_40:
				case WPA_CIPHER_WEP_104:
					tmp = WEP_ENABLED;
					break;
				case WPA_CIPHER_TKIP:
					tmp = TKIP_ENABLED;
					break;
				case WPA_CIPHER_AES_CCM:
					tmp = AES_ENABLED;
					break;
				default:
					WL_ERR(("No Security Info\n"));
			}
			gval |= tmp;
		}
	}
	/* Check for unicast suite(s) */
	if (len < WPA_IE_SUITE_COUNT_LEN) {
		WL_INFORM_MEM(("no unicast suite\n"));
		goto exit;
	}
	/* walk thru unicast cipher list and pick up what we recognize */
	ucast = (const wpa_suite_ucast_t *)&mcast[1];
	count = ltoh16_ua(&ucast->count);
	len -= WPA_IE_SUITE_COUNT_LEN;
	for (i = 0; i < count && len >= WPA_SUITE_LEN;
		i++, len -= WPA_SUITE_LEN) {
		if (!bcmp(ucast->list[i].oui, WPA_OUI, WPA_OUI_LEN)) {
			if (IS_WPA_CIPHER(ucast->list[i].type)) {
				tmp = 0;
				switch (ucast->list[i].type) {
					case WPA_CIPHER_NONE:
						tmp = 0;
						break;
					case WPA_CIPHER_WEP_40:
					case WPA_CIPHER_WEP_104:
						tmp = WEP_ENABLED;
						break;
					case WPA_CIPHER_TKIP:
						tmp = TKIP_ENABLED;
						break;
					case WPA_CIPHER_AES_CCM:
						tmp = AES_ENABLED;
						break;
					default:
						WL_ERR(("No Security Info\n"));
				}
				pval |= tmp;
			}
		}
	}
	len -= (count - i) * WPA_SUITE_LEN;
	/* Check for auth key management suite(s) */
	if (len < WPA_IE_SUITE_COUNT_LEN) {
		WL_INFORM_MEM((" no auth key mgmt suite\n"));
		goto exit;
	}
	/* walk thru auth management suite list and pick up what we recognize */
	mgmt = (const wpa_suite_auth_key_mgmt_t *)&ucast->list[count];
	count = ltoh16_ua(&mgmt->count);
	len -= WPA_IE_SUITE_COUNT_LEN;
	for (i = 0; i < count && len >= WPA_SUITE_LEN;
		i++, len -= WPA_SUITE_LEN) {
		if (!bcmp(mgmt->list[i].oui, WPA_OUI, WPA_OUI_LEN)) {
			if (IS_WPA_AKM(mgmt->list[i].type)) {
				tmp = 0;
				switch (mgmt->list[i].type) {
					case RSN_AKM_NONE:
						tmp = WPA_AUTH_NONE;
						break;
					case RSN_AKM_UNSPECIFIED:
						tmp = WPA_AUTH_UNSPECIFIED;
						break;
					case RSN_AKM_PSK:
						tmp = WPA_AUTH_PSK;
						break;
					default:
						WL_ERR(("No Key Mgmt Info\n"));
				}
				wpa_auth |= tmp;
			}
		}

	}
	/* FOR WPS , set SEC_OW_ENABLED */
	wsec = (pval | gval | SES_OW_ENABLED);
	/* set auth */
	err = wldev_iovar_setint_bsscfg(dev, "auth", auth, bssidx);
	if (err < 0) {
		WL_ERR(("auth error %d\n", err));
		return BCME_ERROR;
	}
	/* set wsec */
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (err < 0) {
		WL_ERR(("wsec error %d\n", err));
		return BCME_ERROR;
	}
	/* set upper-layer auth */
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", wpa_auth, bssidx);
	if (err < 0) {
		WL_ERR(("wpa_auth error %d\n", err));
		return BCME_ERROR;
	}

	if (sec) {
		/* store applied sec settings */
		sec->fw_wpa_auth = wpa_auth;
		sec->fw_wsec = wsec;
		sec->fw_auth = auth;
	}

exit:
	return 0;
}

#if defined(SUPPORT_SOFTAP_WPAWPA2_MIXED)
static u32 wl_get_cipher_type(uint8 type)
{
	u32 ret = 0;
	switch (type) {
		case WPA_CIPHER_NONE:
			ret = 0;
			break;
		case WPA_CIPHER_WEP_40:
		case WPA_CIPHER_WEP_104:
			ret = WEP_ENABLED;
			break;
		case WPA_CIPHER_TKIP:
			ret = TKIP_ENABLED;
			break;
		case WPA_CIPHER_AES_CCM:
			ret = AES_ENABLED;
			break;

#ifdef BCMWAPI_WPI
		case WAPI_CIPHER_SMS4:
			ret = SMS4_ENABLED;
			break;
#endif

		default:
			WL_ERR(("No Security Info\n"));
	}
	return ret;
}

static u32 wl_get_suite_auth_key_mgmt_type(uint8 type, const wpa_suite_mcast_t *mcast)
{
	u32 ret = 0;
	u32 is_wpa2 = 0;

	if (!bcmp(mcast->oui, WPA2_OUI, WPA2_OUI_LEN)) {
		is_wpa2 = 1;
	}

	WL_INFORM_MEM(("%s, type = %d\n", is_wpa2 ? "WPA2":"WPA", type));
	if (bcmp(mcast->oui, WFA_OUI, WFA_OUI_LEN) == 0) {
		switch (type) {
			case RSN_AKM_DPP:
				ret = WPA3_AUTH_DPP_AKM;
				break;
			default:
				WL_ERR(("No Key Mgmt Info in WFA_OUI\n"));
		}
	} else {
		switch (type) {
		case RSN_AKM_NONE:
			/* For WPA and WPA2, AUTH_NONE is common */
			ret = WPA_AUTH_NONE;
			break;
		case RSN_AKM_UNSPECIFIED:
			if (is_wpa2) {
				ret = WPA2_AUTH_UNSPECIFIED;
			} else {
				ret = WPA_AUTH_UNSPECIFIED;
			}
			break;
		case RSN_AKM_PSK:
			if (is_wpa2) {
				ret = WPA2_AUTH_PSK;
			} else {
				ret = WPA_AUTH_PSK;
			}
			break;
#ifdef WL_SAE
		case RSN_AKM_SAE_PSK:
			ret = WPA3_AUTH_SAE_PSK;
			break;
#endif /* WL_SAE */
		default:
			WL_ERR(("No Key Mgmt Info\n"));
		}
	}
	return ret;
}

static s32
wl_validate_wpaie_wpa2ie(struct net_device *dev, const wpa_ie_fixed_t *wpaie,
	const bcm_tlv_t *wpa2ie, s32 bssidx)
{
	const wpa_suite_mcast_t *mcast;
	const wpa_suite_ucast_t *ucast;
	const wpa_suite_auth_key_mgmt_t *mgmt;
	u16 auth = 0; /* d11 open authentication */
	u16 count;
	s32 err = BCME_OK;
	u32 wme_bss_disable;
	u16 suite_count;
	u8 rsn_cap[2];
	s32 len = 0;
	u32 i;
	u32 wsec1, wsec2, wsec;
	u32 pval = 0;
	u32 gval = 0;
	u32 wpa_auth = 0;
	u32 wpa_auth1 = 0;
	u32 wpa_auth2 = 0;
#ifdef MFP
	u32 mfp = 0;
#endif /* MFP */

	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	struct wl_security *sec = wl_read_prof(cfg, dev, WL_PROF_SEC);

	if (wpaie == NULL || wpa2ie == NULL)
		goto exit;

	WL_DBG(("Enter \n"));
	len = wpaie->length;    /* value length */
	len -= WPA_IE_TAG_FIXED_LEN;
	/* check for multicast cipher suite */
	if (len < WPA_SUITE_LEN) {
		WL_INFORM_MEM(("no multicast cipher suite\n"));
		goto exit;
	}

	/* pick up multicast cipher */
	mcast = (const wpa_suite_mcast_t *)&wpaie[1];
	len -= WPA_SUITE_LEN;
	if (!bcmp(mcast->oui, WPA_OUI, WPA_OUI_LEN)) {
		if (IS_WPA_CIPHER(mcast->type)) {
			gval |= wl_get_cipher_type(mcast->type);
		}
	}
	WL_DBG(("\nwpa ie validate\n"));
	WL_DBG(("wpa ie mcast cipher = 0x%X\n", gval));

	/* Check for unicast suite(s) */
	if (len < WPA_IE_SUITE_COUNT_LEN) {
		WL_INFORM_MEM(("no unicast suite\n"));
		goto exit;
	}

	/* walk thru unicast cipher list and pick up what we recognize */
	ucast = (const wpa_suite_ucast_t *)&mcast[1];
	count = ltoh16_ua(&ucast->count);
	len -= WPA_IE_SUITE_COUNT_LEN;
	for (i = 0; i < count && len >= WPA_SUITE_LEN;
		i++, len -= WPA_SUITE_LEN) {
		if (!bcmp(ucast->list[i].oui, WPA_OUI, WPA_OUI_LEN)) {
			if (IS_WPA_CIPHER(ucast->list[i].type)) {
				pval |= wl_get_cipher_type(ucast->list[i].type);
			}
		}
	}
	WL_ERR(("wpa ie ucast count =%d, cipher = 0x%X\n", count, pval));

	/* FOR WPS , set SEC_OW_ENABLED */
	wsec1 = (pval | gval | SES_OW_ENABLED);
	WL_ERR(("wpa ie wsec = 0x%X\n", wsec1));

	len -= (count - i) * WPA_SUITE_LEN;
	/* Check for auth key management suite(s) */
	if (len < WPA_IE_SUITE_COUNT_LEN) {
		WL_INFORM_MEM((" no auth key mgmt suite\n"));
		goto exit;
	}
	/* walk thru auth management suite list and pick up what we recognize */
	mgmt = (const wpa_suite_auth_key_mgmt_t *)&ucast->list[count];
	count = ltoh16_ua(&mgmt->count);
	len -= WPA_IE_SUITE_COUNT_LEN;
	for (i = 0; i < count && len >= WPA_SUITE_LEN;
		i++, len -= WPA_SUITE_LEN) {
		if (!bcmp(mgmt->list[i].oui, WPA_OUI, WPA_OUI_LEN)) {
			if (IS_WPA_AKM(mgmt->list[i].type)) {
				wpa_auth1 |=
					wl_get_suite_auth_key_mgmt_type(mgmt->list[i].type, mcast);
			}
		}

	}
	WL_ERR(("wpa ie wpa_suite_auth_key_mgmt count=%d, key_mgmt = 0x%X\n", count, wpa_auth1));
	WL_ERR(("\nwpa2 ie validate\n"));

	pval = 0;
	gval = 0;
	len =  wpa2ie->len;
	/* check the mcast cipher */
	mcast = (const wpa_suite_mcast_t *)&wpa2ie->data[WPA2_VERSION_LEN];
	gval = wl_get_cipher_type(mcast->type);

	WL_ERR(("wpa2 ie mcast cipher = 0x%X\n", gval));
	if ((len -= WPA_SUITE_LEN) <= 0)
	{
		WL_ERR(("P:wpa2 ie len[%d]", len));
		return BCME_BADLEN;
	}

	/* check the unicast cipher */
	ucast = (const wpa_suite_ucast_t *)&mcast[1];
	suite_count = ltoh16_ua(&ucast->count);
	WL_ERR((" WPA2 ucast cipher count=%d\n", suite_count));
	pval |= wl_get_cipher_type(ucast->list[0].type);

	if ((len -= (WPA_IE_SUITE_COUNT_LEN + (WPA_SUITE_LEN * suite_count))) <= 0)
		return BCME_BADLEN;

	WL_ERR(("wpa2 ie ucast cipher = 0x%X\n", pval));

	/* FOR WPS , set SEC_OW_ENABLED */
	wsec2 = (pval | gval | SES_OW_ENABLED);
	WL_ERR(("wpa2 ie wsec = 0x%X\n", wsec2));

	/* check the AKM */
	mgmt = (const wpa_suite_auth_key_mgmt_t *)&ucast->list[suite_count];
	suite_count = ltoh16_ua(&mgmt->count);
	wpa_auth2 = wl_get_suite_auth_key_mgmt_type(mgmt->list[0].type, mcast);
	WL_ERR(("wpa ie wpa_suite_auth_key_mgmt count=%d, key_mgmt = 0x%X\n", count, wpa_auth2));

	wsec = (wsec1 | wsec2);
	wpa_auth = (wpa_auth1 | wpa_auth2);
	WL_ERR(("wpa_wpa2 wsec=0x%X wpa_auth=0x%X\n", wsec, wpa_auth));

	if ((len -= (WPA_IE_SUITE_COUNT_LEN + (WPA_SUITE_LEN * suite_count))) >= RSN_CAP_LEN) {
		rsn_cap[0] = *(const u8 *)&mgmt->list[suite_count];
		rsn_cap[1] = *((const u8 *)&mgmt->list[suite_count] + 1);
		if (rsn_cap[0] & (RSN_CAP_16_REPLAY_CNTRS << RSN_CAP_PTK_REPLAY_CNTR_SHIFT)) {
			wme_bss_disable = 0;
		} else {
			wme_bss_disable = 1;
		}
		WL_DBG(("P:rsn_cap[0]=[0x%X]:wme_bss_disabled[%d]\n", rsn_cap[0], wme_bss_disable));

#ifdef MFP
		if (wl_get_mfp_capability(rsn_cap[0], &wpa_auth, &mfp) != BCME_OK) {
			WL_ERR(("mfp configuration invalid. rsn_cap:0x%x\n", rsn_cap[0]));
			return BCME_ERROR;
		}
		cfg->mfp_mode = mfp;
#endif /* MFP */

		/* set wme_bss_disable to sync RSN Capabilities */
		err = wldev_iovar_setint_bsscfg(dev, "wme_bss_disable", wme_bss_disable, bssidx);
		if (err < 0) {
			WL_ERR(("wme_bss_disable error %d\n", err));
			return BCME_ERROR;
		}
	} else {
		WL_DBG(("There is no RSN Capabilities. remained len %d\n", len));
	}

	/* set auth */
	err = wldev_iovar_setint_bsscfg(dev, "auth", auth, bssidx);
	if (err < 0) {
		WL_ERR(("auth error %d\n", err));
		return BCME_ERROR;
	}
	/* set wsec */
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (err < 0) {
		WL_ERR(("wsec error %d\n", err));
		return BCME_ERROR;
	}
	/* set upper-layer auth */
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", wpa_auth, bssidx);
	if (err < 0) {
		WL_ERR(("wpa_auth error %d\n", err));
		return BCME_ERROR;
	}

	if (sec) {
		sec->fw_wpa_auth = wpa_auth;
		sec->fw_auth = auth;
		sec->fw_wsec = wsec;
	}

exit:
	return 0;
}
#endif /* SUPPORT_SOFTAP_WPAWPA2_MIXED */

static s32
wl_cfg80211_bcn_validate_sec(
	struct net_device *dev,
	struct parsed_ies *ies,
	u32 dev_role,
	s32 bssidx,
	bool privacy)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	wl_cfgbss_t *bss = wl_get_cfgbss_by_wdev(cfg, dev->ieee80211_ptr);
	struct wl_security *sec = wl_read_prof(cfg, dev, WL_PROF_SEC);

	if (!bss) {
		WL_ERR(("cfgbss is NULL \n"));
		return BCME_ERROR;
	}

	if (dev_role == NL80211_IFTYPE_P2P_GO && (ies->wpa2_ie)) {
		/* For P2P GO, the sec type is WPA2-PSK */
		WL_DBG(("P2P GO: validating wpa2_ie"));
		if (wl_validate_wpa2ie(dev, ies->wpa2_ie, bssidx)  < 0)
			return BCME_ERROR;

	} else if (dev_role == NL80211_IFTYPE_AP) {

		WL_DBG(("SoftAP: validating security"));
		/* If wpa2_ie or wpa_ie is present validate it */

#if defined(SUPPORT_SOFTAP_WPAWPA2_MIXED)
		if ((ies->wpa_ie != NULL && ies->wpa2_ie != NULL)) {
			if (wl_validate_wpaie_wpa2ie(dev, ies->wpa_ie, ies->wpa2_ie, bssidx)  < 0) {
				bss->security_mode = false;
				return BCME_ERROR;
			}
		}
		else {
#endif /* SUPPORT_SOFTAP_WPAWPA2_MIXED */
		if ((ies->wpa2_ie || ies->wpa_ie) &&
			((wl_validate_wpa2ie(dev, ies->wpa2_ie, bssidx)  < 0 ||
			wl_validate_wpaie(dev, ies->wpa_ie, bssidx) < 0))) {
			bss->security_mode = false;
			return BCME_ERROR;
		}

		if (ies->fils_ind_ie &&
			(wl_validate_fils_ind_ie(dev, ies->fils_ind_ie, bssidx)  < 0)) {
			bss->security_mode = false;
			return BCME_ERROR;
		}

		bss->security_mode = true;
		if (bss->rsn_ie) {
			MFREE(cfg->osh, bss->rsn_ie, bss->rsn_ie[1]
				+ WPA_RSN_IE_TAG_FIXED_LEN);
		}
		if (bss->wpa_ie) {
			MFREE(cfg->osh, bss->wpa_ie, bss->wpa_ie[1]
				+ WPA_RSN_IE_TAG_FIXED_LEN);
		}
		if (bss->wps_ie) {
			MFREE(cfg->osh, bss->wps_ie, bss->wps_ie[1] + 2);
		}
		if (bss->fils_ind_ie) {
			MFREE(cfg->osh, bss->fils_ind_ie, bss->fils_ind_ie[1]
				+ FILS_INDICATION_IE_TAG_FIXED_LEN);
		}
		if (ies->wpa_ie != NULL) {
			/* WPAIE */
			bss->rsn_ie = NULL;
			bss->wpa_ie = MALLOCZ(cfg->osh,
					ies->wpa_ie->length
					+ WPA_RSN_IE_TAG_FIXED_LEN);
			if (bss->wpa_ie) {
				memcpy(bss->wpa_ie, ies->wpa_ie,
					ies->wpa_ie->length
					+ WPA_RSN_IE_TAG_FIXED_LEN);
			}
		} else if (ies->wpa2_ie != NULL) {
			/* RSNIE */
			bss->wpa_ie = NULL;
			bss->rsn_ie = MALLOCZ(cfg->osh,
					ies->wpa2_ie->len
					+ WPA_RSN_IE_TAG_FIXED_LEN);
			if (bss->rsn_ie) {
				memcpy(bss->rsn_ie, ies->wpa2_ie,
					ies->wpa2_ie->len
					+ WPA_RSN_IE_TAG_FIXED_LEN);
			}
		}
#ifdef WL_FILS
		if (ies->fils_ind_ie) {
			bss->fils_ind_ie = MALLOCZ(cfg->osh,
					ies->fils_ind_ie->len
					+ FILS_INDICATION_IE_TAG_FIXED_LEN);
			if (bss->fils_ind_ie) {
				memcpy(bss->fils_ind_ie, ies->fils_ind_ie,
					ies->fils_ind_ie->len
					+ FILS_INDICATION_IE_TAG_FIXED_LEN);
			}
		}
#endif /* WL_FILS */
#if defined(SUPPORT_SOFTAP_WPAWPA2_MIXED)
		}
#endif /* SUPPORT_SOFTAP_WPAWPA2_MIXED */
		if (!ies->wpa2_ie && !ies->wpa_ie) {
			wl_validate_opensecurity(dev, bssidx, privacy);
			bss->security_mode = false;
		}

		if (ies->wps_ie) {
			bss->wps_ie = MALLOCZ(cfg->osh, ies->wps_ie_len);
			if (bss->wps_ie) {
				memcpy(bss->wps_ie, ies->wps_ie, ies->wps_ie_len);
			}
		}
	}

	WL_INFORM_MEM(("[%s] wpa_auth:0x%x auth:0x%x wsec:0x%x mfp:0x%x\n",
		dev->name, sec->fw_wpa_auth, sec->fw_auth, sec->fw_wsec, sec->fw_mfp));
	return 0;

}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)) || defined(WL_COMPAT_WIRELESS)
static s32 wl_cfg80211_bcn_set_params(
	struct cfg80211_ap_settings *info,
	struct net_device *dev,
	u32 dev_role, s32 bssidx)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	s32 err = BCME_OK;

	WL_DBG(("interval (%d) \ndtim_period (%d) \n",
		info->beacon_interval, info->dtim_period));

	if (info->beacon_interval) {
		if ((err = wldev_ioctl_set(dev, WLC_SET_BCNPRD,
			&info->beacon_interval, sizeof(s32))) < 0) {
			WL_ERR(("Beacon Interval Set Error, %d\n", err));
			return err;
		}
	}

	if (info->dtim_period) {
		if ((err = wldev_ioctl_set(dev, WLC_SET_DTIMPRD,
			&info->dtim_period, sizeof(s32))) < 0) {
			WL_ERR(("DTIM Interval Set Error, %d\n", err));
			return err;
		}
	}

	if ((info->ssid) && (info->ssid_len > 0) &&
		(info->ssid_len <= DOT11_MAX_SSID_LEN)) {
		WL_DBG(("SSID (%s) len:%zd \n", info->ssid, info->ssid_len));
		if (dev_role == NL80211_IFTYPE_AP) {
			/* Store the hostapd SSID */
			bzero(cfg->hostapd_ssid.SSID, DOT11_MAX_SSID_LEN);
			memcpy(cfg->hostapd_ssid.SSID, info->ssid, info->ssid_len);
			cfg->hostapd_ssid.SSID_len = (uint32)info->ssid_len;
		} else {
				/* P2P GO */
			bzero(cfg->p2p->ssid.SSID, DOT11_MAX_SSID_LEN);
			memcpy(cfg->p2p->ssid.SSID, info->ssid, info->ssid_len);
			cfg->p2p->ssid.SSID_len = (uint32)info->ssid_len;
		}
	}

	return err;
}
#endif /* LINUX_VERSION >= VERSION(3,4,0) || WL_COMPAT_WIRELESS */

s32
wl_cfg80211_parse_ies(const u8 *ptr, u32 len, struct parsed_ies *ies)
{
	s32 err = BCME_OK;

	bzero(ies, sizeof(struct parsed_ies));

	/* find the WPSIE */
	if ((ies->wps_ie = wl_cfgp2p_find_wpsie(ptr, len)) != NULL) {
		WL_DBG(("WPSIE in beacon \n"));
		ies->wps_ie_len = ies->wps_ie->length + WPA_RSN_IE_TAG_FIXED_LEN;
	} else {
		WL_ERR(("No WPSIE in beacon \n"));
	}

	/* find the RSN_IE */
	if ((ies->wpa2_ie = bcm_parse_tlvs(ptr, len,
		DOT11_MNG_RSN_ID)) != NULL) {
		WL_DBG((" WPA2 IE found\n"));
		ies->wpa2_ie_len = ies->wpa2_ie->len;
	}

	/* find the FILS_IND_IE */
	if ((ies->fils_ind_ie = bcm_parse_tlvs(ptr, len,
		DOT11_MNG_FILS_IND_ID)) != NULL) {
		WL_DBG((" FILS IND IE found\n"));
		ies->fils_ind_ie_len = ies->fils_ind_ie->len;
	}

	/* find the WPA_IE */
	if ((ies->wpa_ie = wl_cfgp2p_find_wpaie(ptr, len)) != NULL) {
		WL_DBG((" WPA found\n"));
		ies->wpa_ie_len = ies->wpa_ie->length;
	}

	return err;

}

s32
wl_cfg80211_set_ap_role(
	struct bcm_cfg80211 *cfg,
	struct net_device *dev)
{
	s32 err = BCME_OK;
	s32 infra = 1;
	s32 ap = 0;
	s32 pm;
	s32 bssidx;
	s32 apsta = 0;

	if ((bssidx = wl_get_bssidx_by_wdev(cfg, dev->ieee80211_ptr)) < 0) {
		WL_ERR(("Find p2p index from wdev(%p) failed\n", dev->ieee80211_ptr));
		return -EINVAL;
	}

	WL_INFORM_MEM(("[%s] Bringup SoftAP on bssidx:%d \n", dev->name, bssidx));

	if ((err = wl_cfg80211_add_del_bss(cfg, dev, bssidx,
			WL_IF_TYPE_AP, 0, NULL)) < 0) {
		WL_ERR(("wl add_del_bss returned error:%d\n", err));
		return err;
	}

	/*
	 * For older chips, "bss" iovar does not support
	 * bsscfg role change/upgradation, and still
	 * return BCME_OK on attempt
	 * Hence, below traditional way to handle the same
	 */

	if ((err = wldev_ioctl_get(dev,
			WLC_GET_AP, &ap, sizeof(s32))) < 0) {
		WL_ERR(("Getting AP mode failed %d \n", err));
		return err;
	}

	if (!ap) {
		/* AP mode switch not supported. Try setting up AP explicitly */
		err = wldev_iovar_getint(dev, "apsta", (s32 *)&apsta);
		if (unlikely(err)) {
			WL_ERR(("Could not get apsta %d\n", err));
			return err;
		}
		if (apsta == 0) {
			/* If apsta is not set, set it */

			/* Check for any connected interfaces before wl down */
			if (wl_get_drv_status_all(cfg, CONNECTED) > 0) {
				WL_ERR(("Concurrent i/f operational. can't do wl down"));
				return BCME_ERROR;
			}
			err = wldev_ioctl_set(dev, WLC_DOWN, &ap, sizeof(s32));
			if (err < 0) {
				WL_ERR(("WLC_DOWN error %d\n", err));
				return err;
			}
			err = wldev_iovar_setint(dev, "apsta", 0);
			if (err < 0) {
				WL_ERR(("wl apsta 0 error %d\n", err));
				return err;
			}
			ap = 1;
			if ((err = wldev_ioctl_set(dev,
					WLC_SET_AP, &ap, sizeof(s32))) < 0) {
				WL_ERR(("setting AP mode failed %d \n", err));
				return err;
			}
		}
	}

	if (bssidx == 0) {
		pm = 0;
		if ((err = wldev_ioctl_set(dev, WLC_SET_PM, &pm, sizeof(pm))) != 0) {
			WL_ERR(("wl PM 0 returned error:%d\n", err));
			/* Ignore error, if any */
			err = BCME_OK;
		}
		err = wldev_ioctl_set(dev, WLC_SET_INFRA, &infra, sizeof(s32));
		if (err < 0) {
			WL_ERR(("SET INFRA error %d\n", err));
			return err;
		}
	}

	/* On success, mark AP creation in progress. */
	wl_set_drv_status(cfg, AP_CREATING, dev);
	return 0;
}

void
wl_cfg80211_ap_timeout_work(struct work_struct *work)
{
	struct bcm_cfg80211 *cfg = NULL;
	dhd_pub_t *dhdp = NULL;
	BCM_SET_CONTAINER_OF(cfg, work, struct bcm_cfg80211, ap_work.work);

	WL_ERR(("** AP LINK UP TIMEOUT **\n"));
	dhdp = (dhd_pub_t *)(cfg->pub);
	if (dhd_query_bus_erros(dhdp)) {
		return;
	}
#ifdef DHD_PCIE_RUNTIMEPM
	dhdpcie_runtime_bus_wake(dhdp, CAN_SLEEP(), __builtin_return_address(0));
#endif /* DHD_PCIE_RUNTIMEPM */
	dhdp->iface_op_failed = TRUE;

#if defined(DHD_DEBUG) && defined(DHD_FW_COREDUMP)
	if (dhdp->memdump_enabled) {
		dhdp->memdump_type = DUMP_TYPE_AP_LINKUP_FAILURE;
		dhd_bus_mem_dump(dhdp);
	}
#endif /* DHD_DEBUG && DHD_FW_COREDUMP */

#if defined(OEM_ANDROID)
	WL_ERR(("Notify hang event to upper layer \n"));
	dhdp->hang_reason = HANG_REASON_IFACE_ADD_FAILURE;
	net_os_send_hang_message(bcmcfg_to_prmry_ndev(cfg));
#endif /* OEM_ANDROID */
}

/* In RSDB downgrade cases, the link up event can get delayed upto 7-8 secs */
#define MAX_AP_LINK_WAIT_TIME   10000
static s32
wl_cfg80211_bcn_bringup_ap(
	struct net_device *dev,
	struct parsed_ies *ies,
	u32 dev_role, s32 bssidx)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	struct wl_join_params join_params;
	bool is_bssup = false;
	s32 infra = 1;
	s32 join_params_size = 0;
	s32 ap = 1;
	s32 wsec;
#ifdef DISABLE_11H_SOFTAP
	s32 spect = 0;
#endif /* DISABLE_11H_SOFTAP */
#ifdef SOFTAP_UAPSD_OFF
	uint32 wme_apsd = 0;
#endif /* SOFTAP_UAPSD_OFF */
	s32 err = BCME_OK;
	s32 is_rsdb_supported = BCME_ERROR;

	is_rsdb_supported = DHD_OPMODE_SUPPORTED(cfg->pub, DHD_FLAG_RSDB_MODE);
	if (is_rsdb_supported < 0)
		return (-ENODEV);

	WL_DBG(("Enter dev_role:%d bssidx:%d ifname:%s\n", dev_role, bssidx, dev->name));

	/* Common code for SoftAP and P2P GO */
	wl_clr_drv_status(cfg, AP_CREATED, dev);

	/* Make sure INFRA is set for AP/GO */
	err = wldev_ioctl_set(dev, WLC_SET_INFRA, &infra, sizeof(s32));
	if (err < 0) {
		WL_ERR(("SET INFRA error %d\n", err));
		goto exit;
	}

	/* Do abort scan before creating GO */
	wl_cfgscan_cancel_scan(cfg);

	/* Schedule delayed work to handle link time out. schedule
	 * before ssid iovar. Sometimes before iovar context should
	 * resume, the event may come and get processed.
	 */
	if (schedule_delayed_work(&cfg->ap_work,
			msecs_to_jiffies((const unsigned int)MAX_AP_LINK_WAIT_TIME))) {
		WL_DBG(("ap timeout work scheduled\n"));
	}

	if (dev_role == NL80211_IFTYPE_P2P_GO) {
		is_bssup = wl_cfg80211_bss_isup(dev, bssidx);
		if (!is_bssup && (ies->wpa2_ie != NULL)) {

			err = wldev_iovar_setbuf_bsscfg(dev, "ssid", &cfg->p2p->ssid,
				sizeof(cfg->p2p->ssid), cfg->ioctl_buf, WLC_IOCTL_MAXLEN,
				bssidx, &cfg->ioctl_buf_sync);
			if (err < 0) {
				WL_ERR(("GO SSID setting error %d\n", err));
				goto exit;
			}

#ifdef MFP
			err = wldev_iovar_setint_bsscfg(dev, "mfp", cfg->mfp_mode, bssidx);
			if (err < 0) {
				WL_ERR(("MFP Setting failed. ret = %d \n", err));
				/* If fw doesn't support mfp, Ignore the error */
				if (err != BCME_UNSUPPORTED) {
					goto exit;
				}
			}
#endif /* MFP */

			if ((err = wl_cfg80211_bss_up(cfg, dev, bssidx, 1)) < 0) {
				WL_ERR(("GO Bring up error %d\n", err));
				goto exit;
			}
			wait_for_completion_timeout(&cfg->iface_up,
				msecs_to_jiffies(500));
			wl_clr_drv_status(cfg, AP_CREATING, dev);
		} else
			WL_DBG(("Bss is already up\n"));
	} else if (dev_role == NL80211_IFTYPE_AP) {

		if (!wl_get_drv_status(cfg, AP_CREATING, dev)) {
			/* Make sure fw is in proper state */
			err = wl_cfg80211_set_ap_role(cfg, dev);
			if (unlikely(err)) {
				WL_ERR(("set ap role failed!\n"));
				goto exit;
			}
		}

		/* Device role SoftAP */
		WL_DBG(("Creating AP bssidx:%d dev_role:%d\n", bssidx, dev_role));
		/* Clear the status bit after use */
		wl_clr_drv_status(cfg, AP_CREATING, dev);

#ifdef DISABLE_11H_SOFTAP
		/* Some old WLAN card (e.g. Intel PRO/Wireless 2200BG)
		 * does not try to connect SoftAP because they cannot detect
		 * 11h IEs. For this reason, we disable 11h feature in case
		 * of SoftAP mode. (Related CSP case number: 661635)
		 */
		if (is_rsdb_supported == 0) {
			err = wldev_ioctl_set(dev, WLC_DOWN, &ap, sizeof(s32));
			if (err < 0) {
				WL_ERR(("WLC_DOWN error %d\n", err));
				goto exit;
			}
		}
		err = wldev_ioctl_set(dev, WLC_SET_SPECT_MANAGMENT,
			&spect, sizeof(s32));
		if (err < 0) {
			WL_ERR(("SET SPECT_MANAGMENT error %d\n", err));
			goto exit;
		}
#endif /* DISABLE_11H_SOFTAP */

#ifdef WL_DISABLE_HE_SOFTAP
		err = wl_cfg80211_change_he_features(dev, cfg, bssidx, WL_HE_FEATURES_HE_AP, FALSE);
		if (err < 0) {
			WL_ERR(("failed to set he features, error=%d\n", err));
		}
#endif /* WL_DISABLE_HE_SOFTAP */

#ifdef SOFTAP_UAPSD_OFF
		err = wldev_iovar_setbuf_bsscfg(dev, "wme_apsd", &wme_apsd, sizeof(wme_apsd),
			cfg->ioctl_buf, WLC_IOCTL_SMLEN, bssidx, &cfg->ioctl_buf_sync);
		if (err < 0) {
			WL_ERR(("failed to disable uapsd, error=%d\n", err));
		}
#endif /* SOFTAP_UAPSD_OFF */

		err = wldev_ioctl_set(dev, WLC_UP, &ap, sizeof(s32));
		if (unlikely(err)) {
			WL_ERR(("WLC_UP error (%d)\n", err));
			goto exit;
		}

#ifdef MFP
		if (cfg->bip_pos) {
			err = wldev_iovar_setbuf_bsscfg(dev, "bip",
				(const void *)(cfg->bip_pos), WPA_SUITE_LEN, cfg->ioctl_buf,
				WLC_IOCTL_SMLEN, bssidx, &cfg->ioctl_buf_sync);
			if (err < 0) {
				WL_ERR(("bip set error %d\n", err));

				{
					goto exit;
				}
			}
		}
#endif /* MFP */

		err = wldev_iovar_getint(dev, "wsec", (s32 *)&wsec);
		if (unlikely(err)) {
			WL_ERR(("Could not get wsec %d\n", err));
			goto exit;
		}
		if ((wsec == WEP_ENABLED) && cfg->wep_key.len) {
			WL_DBG(("Applying buffered WEP KEY \n"));
			err = wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &cfg->wep_key,
				sizeof(struct wl_wsec_key), cfg->ioctl_buf,
				WLC_IOCTL_MAXLEN, bssidx, &cfg->ioctl_buf_sync);
			/* clear the key after use */
			bzero(&cfg->wep_key, sizeof(struct wl_wsec_key));
			if (unlikely(err)) {
				WL_ERR(("WLC_SET_KEY error (%d)\n", err));
				goto exit;
			}
		}

#ifdef MFP
		/* This needs to go after wsec otherwise the wsec command will
		 * overwrite the values set by MFP
		 */
		err = wldev_iovar_setint_bsscfg(dev, "mfp", cfg->mfp_mode, bssidx);
		if (err < 0) {
			WL_ERR(("MFP Setting failed. ret = %d \n", err));
			/* If fw doesn't support mfp, Ignore the error */
			if (err != BCME_UNSUPPORTED) {
				goto exit;
			}
		}
#endif /* MFP */

		bzero(&join_params, sizeof(join_params));
		/* join parameters starts with ssid */
		join_params_size = sizeof(join_params.ssid);
		join_params.ssid.SSID_len = MIN(cfg->hostapd_ssid.SSID_len,
			(uint32)DOT11_MAX_SSID_LEN);
		memcpy(join_params.ssid.SSID, cfg->hostapd_ssid.SSID,
			join_params.ssid.SSID_len);
		join_params.ssid.SSID_len = htod32(join_params.ssid.SSID_len);

		/* create softap */
		if ((err = wldev_ioctl_set(dev, WLC_SET_SSID, &join_params,
			join_params_size)) != 0) {
			WL_ERR(("SoftAP/GO set ssid failed! \n"));
			goto exit;
		} else {
			WL_DBG((" SoftAP SSID \"%s\" \n", join_params.ssid.SSID));
		}

		if (bssidx != 0) {
			/* AP on Virtual Interface */
			if ((err = wl_cfg80211_bss_up(cfg, dev, bssidx, 1)) < 0) {
				WL_ERR(("AP Bring up error %d\n", err));
				goto exit;
			}
		}

	} else {
		WL_ERR(("Wrong interface type %d\n", dev_role));
		goto exit;
	}

	SUPP_LOG(("AP/GO UP\n"));

exit:
	if (cfg->wep_key.len) {
		bzero(&cfg->wep_key, sizeof(struct wl_wsec_key));
	}

#ifdef MFP
	if (cfg->mfp_mode) {
		cfg->mfp_mode = 0;
	}

	if (cfg->bip_pos) {
		cfg->bip_pos = NULL;
	}
#endif /* MFP */

	if (err) {
		SUPP_LOG(("AP/GO bring up fail. err:%d\n", err));
		/* Cancel work if scheduled */
		if (delayed_work_pending(&cfg->ap_work)) {
			cancel_delayed_work_sync(&cfg->ap_work);
			WL_DBG(("cancelled ap_work\n"));
		}
	}
	return err;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)) || defined(WL_COMPAT_WIRELESS)
s32
wl_cfg80211_parse_ap_ies(
	struct net_device *dev,
	struct cfg80211_beacon_data *info,
	struct parsed_ies *ies)
{
	struct parsed_ies prb_ies;
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);
	const u8 *vndr = NULL;
	u32 vndr_ie_len = 0;
	s32 err = BCME_OK;

	/* Parse Beacon IEs */
	if (wl_cfg80211_parse_ies((const u8 *)info->tail,
		info->tail_len, ies) < 0) {
		WL_ERR(("Beacon get IEs failed \n"));
		err = -EINVAL;
		goto fail;
	}

	if ((err = wl_cfg80211_config_rsnxe_ie(cfg, dev,
		(const u8 *)info->tail, info->tail_len)) < 0) {
		WL_ERR(("Failed to configure rsnxe ie: %d\n", err));
		err = -EINVAL;
		goto fail;
	}

	vndr = (const u8 *)info->proberesp_ies;
	vndr_ie_len = (uint32)info->proberesp_ies_len;

	if (dhd->op_mode & DHD_FLAG_HOSTAP_MODE) {
		/* SoftAP mode */
		const struct ieee80211_mgmt *mgmt;
		mgmt = (const struct ieee80211_mgmt *)info->probe_resp;
		if (mgmt != NULL) {
			vndr = (const u8 *)&mgmt->u.probe_resp.variable;
			vndr_ie_len = (uint32)(info->probe_resp_len -
				offsetof(const struct ieee80211_mgmt, u.probe_resp.variable));
		}
	}
	/* Parse Probe Response IEs */
	if (wl_cfg80211_parse_ies((const u8 *)vndr, vndr_ie_len, &prb_ies) < 0) {
		WL_ERR(("PROBE RESP get IEs failed \n"));
		err = -EINVAL;
	}
fail:

	return err;
}

s32
wl_cfg80211_set_ies(
	struct net_device *dev,
	struct cfg80211_beacon_data *info,
	s32 bssidx)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);
	const u8 *vndr = NULL;
	u32 vndr_ie_len = 0;
	s32 err = BCME_OK;

	/* Set Beacon IEs to FW */
	if ((err = wl_cfg80211_set_mgmt_vndr_ies(cfg, ndev_to_cfgdev(dev), bssidx,
		VNDR_IE_BEACON_FLAG, (const u8 *)info->tail,
		info->tail_len)) < 0) {
		WL_ERR(("Set Beacon IE Failed \n"));
	} else {
		WL_DBG(("Applied Vndr IEs for Beacon \n"));
	}

	vndr = (const u8 *)info->proberesp_ies;
	vndr_ie_len = (uint32)info->proberesp_ies_len;

	if (dhd->op_mode & DHD_FLAG_HOSTAP_MODE) {
		/* SoftAP mode */
		const struct ieee80211_mgmt *mgmt;
		mgmt = (const struct ieee80211_mgmt *)info->probe_resp;
		if (mgmt != NULL) {
			vndr = (const u8 *)&mgmt->u.probe_resp.variable;
			vndr_ie_len = (uint32)(info->probe_resp_len -
				offsetof(struct ieee80211_mgmt, u.probe_resp.variable));
		}
	}

	/* Set Probe Response IEs to FW */
	if ((err = wl_cfg80211_set_mgmt_vndr_ies(cfg, ndev_to_cfgdev(dev), bssidx,
		VNDR_IE_PRBRSP_FLAG, vndr, vndr_ie_len)) < 0) {
		WL_ERR(("Set Probe Resp IE Failed \n"));
	} else {
		WL_DBG(("Applied Vndr IEs for Probe Resp \n"));
	}

#ifdef WL11U
	if (dhd->op_mode & DHD_FLAG_HOSTAP_MODE) {
		bcm_tlv_t *interworking_ie;
		/* find the INTERWORKING IE */
		if ((interworking_ie =
			wl_cfg80211_find_interworking_ie(info->tail, info->tail_len)) != NULL) {
			WL_DBG(("INTERWORKING found\n"));
			if ((err = wl_cfg80211_add_iw_ie(cfg, dev, bssidx,
					VNDR_IE_CUSTOM_FLAG, interworking_ie->id,
					interworking_ie->data,
					interworking_ie->len)) != BCME_OK) {
				WL_ERR(("Failed to add interworking IE"));
			}
		}
	}
#endif /* WL11U */
	return err;
}
#endif /* LINUX_VERSION >= VERSION(3,4,0) || WL_COMPAT_WIRELESS */

static s32 wl_cfg80211_hostapd_sec(
	struct net_device *dev,
	struct parsed_ies *ies,
	s32 bssidx)
{
	bool update_bss = 0;
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	wl_cfgbss_t *bss = wl_get_cfgbss_by_wdev(cfg, dev->ieee80211_ptr);

	if (!bss) {
		WL_ERR(("cfgbss is NULL \n"));
		return -EINVAL;
	}

	if (ies->wps_ie) {
		/* Remove after verification.
		 * Setting IE part moved to set_ies func
		 */
		if (bss->wps_ie &&
			memcmp(bss->wps_ie, ies->wps_ie, ies->wps_ie_len)) {
			WL_DBG((" WPS IE is changed\n"));
			MFREE(cfg->osh, bss->wps_ie, bss->wps_ie[1] + 2);
			bss->wps_ie = MALLOCZ(cfg->osh, ies->wps_ie_len);
			if (bss->wps_ie) {
				memcpy(bss->wps_ie, ies->wps_ie, ies->wps_ie_len);
			}
		} else if (bss->wps_ie == NULL) {
			WL_DBG((" WPS IE is added\n"));
			bss->wps_ie = MALLOCZ(cfg->osh, ies->wps_ie_len);
			if (bss->wps_ie) {
				memcpy(bss->wps_ie, ies->wps_ie, ies->wps_ie_len);
			}
		}

#if defined(SUPPORT_SOFTAP_WPAWPA2_MIXED)
		if (ies->wpa_ie != NULL && ies->wpa2_ie != NULL) {
			WL_ERR(("update bss - wpa_ie and  wpa2_ie is not null\n"));
			if (!bss->security_mode) {
				/* change from open mode to security mode */
				update_bss = true;
				bss->wpa_ie = MALLOCZ(cfg->osh,
					ies->wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN);
				if (bss->wpa_ie) {
					memcpy(bss->wpa_ie, ies->wpa_ie,
						ies->wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN);
				}
				bss->rsn_ie = MALLOCZ(cfg->osh,
						ies->wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN);
				if (bss->rsn_ie) {
					memcpy(bss->rsn_ie, ies->wpa2_ie,
						ies->wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN);
				}
			} else {
				/* change from (WPA or WPA2 or WPA/WPA2) to WPA/WPA2 mixed mode */
				if (bss->wpa_ie) {
					if (memcmp(bss->wpa_ie,
					ies->wpa_ie, ies->wpa_ie->length +
					WPA_RSN_IE_TAG_FIXED_LEN)) {
						MFREE(cfg->osh, bss->wpa_ie,
							bss->wpa_ie[1] + WPA_RSN_IE_TAG_FIXED_LEN);
						update_bss = true;
						bss->wpa_ie = MALLOCZ(cfg->osh,
							ies->wpa_ie->length
							+ WPA_RSN_IE_TAG_FIXED_LEN);
						if (bss->wpa_ie) {
							memcpy(bss->wpa_ie, ies->wpa_ie,
								ies->wpa_ie->length
								+ WPA_RSN_IE_TAG_FIXED_LEN);
						}
					}
				}
				else {
					update_bss = true;
					bss->wpa_ie = MALLOCZ(cfg->osh,
						ies->wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN);
					if (bss->wpa_ie) {
						memcpy(bss->wpa_ie, ies->wpa_ie,
							ies->wpa_ie->length
							+ WPA_RSN_IE_TAG_FIXED_LEN);
					}
				}
				if (bss->rsn_ie) {
					if (memcmp(bss->rsn_ie,
					ies->wpa2_ie,
					ies->wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN)) {
						update_bss = true;
						MFREE(cfg->osh, bss->rsn_ie,
							bss->rsn_ie[1] + WPA_RSN_IE_TAG_FIXED_LEN);
						bss->rsn_ie = MALLOCZ(cfg->osh,
							ies->wpa2_ie->len
							+ WPA_RSN_IE_TAG_FIXED_LEN);
						if (bss->rsn_ie) {
							memcpy(bss->rsn_ie, ies->wpa2_ie,
								ies->wpa2_ie->len
								+ WPA_RSN_IE_TAG_FIXED_LEN);
						}
					}
				}
				else {
					update_bss = true;
					bss->rsn_ie = MALLOCZ(cfg->osh,
						ies->wpa2_ie->len
						+ WPA_RSN_IE_TAG_FIXED_LEN);
					if (bss->rsn_ie) {
						memcpy(bss->rsn_ie, ies->wpa2_ie,
							ies->wpa2_ie->len
							+ WPA_RSN_IE_TAG_FIXED_LEN);
					}
				}
			}
			WL_ERR(("update_bss=%d\n", update_bss));
			if (update_bss) {
				bss->security_mode = true;
				wl_cfg80211_bss_up(cfg, dev, bssidx, 0);
				if (wl_validate_wpaie_wpa2ie(dev, ies->wpa_ie,
					ies->wpa2_ie, bssidx)  < 0) {
					return BCME_ERROR;
				}
				wl_cfg80211_bss_up(cfg, dev, bssidx, 1);
			}

		}
		else
#endif /* SUPPORT_SOFTAP_WPAWPA2_MIXED */
		if ((ies->wpa_ie != NULL || ies->wpa2_ie != NULL)) {
			if (!bss->security_mode) {
				/* change from open mode to security mode */
				update_bss = true;
				if (ies->wpa_ie != NULL) {
					bss->wpa_ie = MALLOCZ(cfg->osh,
						ies->wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN);
					if (bss->wpa_ie) {
						memcpy(bss->wpa_ie,
							ies->wpa_ie,
							ies->wpa_ie->length
							+ WPA_RSN_IE_TAG_FIXED_LEN);
					}
				} else {
					bss->rsn_ie = MALLOCZ(cfg->osh,
						ies->wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN);
					if (bss->rsn_ie) {
						memcpy(bss->rsn_ie,
							ies->wpa2_ie,
							ies->wpa2_ie->len
							+ WPA_RSN_IE_TAG_FIXED_LEN);
					}
				}
			} else if (bss->wpa_ie) {
				/* change from WPA2 mode to WPA mode */
				if (ies->wpa_ie != NULL) {
					update_bss = true;
					MFREE(cfg->osh, bss->rsn_ie,
						bss->rsn_ie[1] + WPA_RSN_IE_TAG_FIXED_LEN);
					bss->wpa_ie = MALLOCZ(cfg->osh,
						ies->wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN);
					if (bss->wpa_ie) {
						memcpy(bss->wpa_ie,
							ies->wpa_ie,
							ies->wpa_ie->length
							+ WPA_RSN_IE_TAG_FIXED_LEN);
					}
				} else if (memcmp(bss->rsn_ie,
					ies->wpa2_ie, ies->wpa2_ie->len
					+ WPA_RSN_IE_TAG_FIXED_LEN)) {
					update_bss = true;
					MFREE(cfg->osh, bss->rsn_ie,
						bss->rsn_ie[1] + WPA_RSN_IE_TAG_FIXED_LEN);
					bss->rsn_ie = MALLOCZ(cfg->osh,
						ies->wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN);
					if (bss->rsn_ie) {
						memcpy(bss->rsn_ie,
							ies->wpa2_ie,
							ies->wpa2_ie->len
							+ WPA_RSN_IE_TAG_FIXED_LEN);
					}
					bss->wpa_ie = NULL;
				}
			}
			if (update_bss) {
				bss->security_mode = true;
				wl_cfg80211_bss_up(cfg, dev, bssidx, 0);
				if (wl_validate_wpa2ie(dev, ies->wpa2_ie, bssidx)  < 0 ||
					wl_validate_wpaie(dev, ies->wpa_ie, bssidx) < 0) {
					return BCME_ERROR;
				}
				wl_cfg80211_bss_up(cfg, dev, bssidx, 1);
			}
		}
	} else {
		WL_ERR(("No WPSIE in beacon \n"));
	}
	return 0;
}

static s32
wl_cfg80211_set_scb_timings(
	struct bcm_cfg80211 *cfg,
	struct net_device *dev)
{
	int err;
	u32 ps_pretend;
	wl_scb_probe_t scb_probe;
	u32 ps_pretend_retries;

	bzero(&scb_probe, sizeof(wl_scb_probe_t));
	scb_probe.scb_timeout = WL_SCB_TIMEOUT;
	scb_probe.scb_activity_time = WL_SCB_ACTIVITY_TIME;
	scb_probe.scb_max_probe = WL_SCB_MAX_PROBE;
	err = wldev_iovar_setbuf(dev, "scb_probe", (void *)&scb_probe,
		sizeof(wl_scb_probe_t), cfg->ioctl_buf, WLC_IOCTL_SMLEN,
		&cfg->ioctl_buf_sync);
	if (unlikely(err)) {
		WL_ERR(("set 'scb_probe' failed, error = %d\n", err));
		return err;
	}

	ps_pretend_retries = WL_PSPRETEND_RETRY_LIMIT;
	err = wldev_iovar_setint(dev, "pspretend_retry_limit", ps_pretend_retries);
	if (unlikely(err)) {
		if (err == BCME_UNSUPPORTED) {
			/* Ignore error if fw doesn't support the iovar */
			WL_DBG(("set 'pspretend_retry_limit %d' failed, error = %d\n",
				ps_pretend_retries, err));
		} else {
			WL_ERR(("set 'pspretend_retry_limit %d' failed, error = %d\n",
				ps_pretend_retries, err));
			return err;
		}
	}

	ps_pretend = MAX(WL_SCB_MAX_PROBE / 2, WL_MIN_PSPRETEND_THRESHOLD);
	err = wldev_iovar_setint(dev, "pspretend_threshold", ps_pretend);
	if (unlikely(err)) {
		if (err == BCME_UNSUPPORTED) {
			/* Ignore error if fw doesn't support the iovar */
			WL_DBG(("wl pspretend_threshold %d set error %d\n",
				ps_pretend, err));
		} else {
			WL_ERR(("wl pspretend_threshold %d set error %d\n",
				ps_pretend, err));
			return err;
		}
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)) || defined(WL_COMPAT_WIRELESS)
s32
wl_cfg80211_start_ap(
	struct wiphy *wiphy,
	struct net_device *dev,
	struct cfg80211_ap_settings *info)
{
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	s32 err = BCME_OK;
	struct parsed_ies ies;
	s32 bssidx = 0;
	u32 dev_role = 0;
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);
	struct net_device  *primary_ndev = bcmcfg_to_prmry_ndev(cfg);

	WL_DBG(("Enter \n"));
	BCM_REFERENCE(dhd);
	BCM_REFERENCE(primary_ndev);

	if ((bssidx = wl_get_bssidx_by_wdev(cfg, dev->ieee80211_ptr)) < 0) {
		WL_ERR(("Find p2p index from wdev(%p) failed\n", dev->ieee80211_ptr));
		return BCME_ERROR;
	}

	if (p2p_is_on(cfg) && (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_GO)) {
		dev_role = NL80211_IFTYPE_P2P_GO;
	} else if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {

		if (!wl_get_drv_status(cfg, AP_CREATING, dev)) {
			/* Make sure fw is in proper state */
			err = wl_cfg80211_set_ap_role(cfg, dev);
			if (unlikely(0 > err)) {
				WL_ERR(("set ap role failed!\n"));
				return BCME_ERROR;
			}
		}
		dev_role = NL80211_IFTYPE_AP;
		dhd->op_mode |= DHD_FLAG_HOSTAP_MODE;
		err = dhd_ndo_enable(dhd, FALSE);
		WL_DBG(("Disabling NDO on Hostapd mode %d\n", err));
		if (err) {
			WL_ERR(("Disabling NDO Failed %d\n", err));
		}
#if defined(ARP_OFFLOAD_SUPPORT) && defined(DISABLE_ARP_OFFLOAD_ON_SOFTAP)
		WL_ERR(("Disabling ARP Offload on SoftAP mode\n"));
		/* IF SoftAP is enabled, disable ARP Offload */
		dhd_arp_offload_enable(dhd, FALSE);
#endif /* ARP_OFFLOAD_SUPPORT && DISABLE_ARP_OFFLOAD_ON_SOFTAP */
		/* Disable packet filter */
		if (dhd->early_suspended) {
			WL_ERR(("Disable pkt_filter\n"));
#ifdef PKT_FILTER_SUPPORT
			dhd_enable_packet_filter(FALSE, dhd);
#endif /* PKT_FILTER_SUPPORT */
#ifdef APF
			dhd_dev_apf_disable_filter(dhd_linux_get_primary_netdev(dhd));
#endif /* APF */
		}
	} else {
		/* only AP or GO role need to be handled here. */
		err = -EINVAL;
		goto fail;
	}

	/* disable TDLS */
#ifdef WLTDLS
	if (bssidx == 0) {
		/* Disable TDLS for primary Iface. For virtual interface,
		 * tdls disable will happen from interface create context
		 */
		wl_cfg80211_tdls_config(cfg, TDLS_STATE_AP_CREATE, false);
	}
#endif /*  WLTDLS */

	if (!check_dev_role_integrity(cfg, dev_role)) {
		err = -EINVAL;
		goto fail;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
	if (primary_ndev) {
		int  need_down = FALSE, need_up = TRUE;
		int  ifidx = dhd_net2idx(dhd->info, dev);
		int  gmode = -1;
		int  nmode = 0;
		int  vhtmode = 0;
		int  hemode = 0;
		int  only_mode = 0;
		int  primary_idx = 0;
#if defined(WL_HOSTAPD_CFG)
		chanspec_t chanspec = 0;
#endif /* WL_HOSTAPD_CFG */
#if defined(WL_HOSTAPD_CFG) || defined(SUPPORT_2G_VHT)
		struct ieee80211_channel *chan = NULL;
#endif /* defined(WL_HOSTAPD_CFG) || defined(SUPPORT_2G_VHT */
		BCM_REFERENCE(primary_idx);

		// HT
		if (info->ht_cap) {
			nmode = 1;
		}
		if (info->ht_required) {
			only_mode = OMC_HT;
		}
		WL_ERR(("need nmode=%d, only_mode=%d\n", nmode, only_mode));

		// VHT
		if (info->vht_cap) {
			vhtmode = 1;
		}
#ifdef SUPPORT_2G_VHT
		if (dhd_2g_vht) {
			chan = wl_cfg80211_get_ieee80211_chan(dev, info);
			if (chan->band == IEEE80211_BAND_2GHZ) {
				WL_ERR(("Force 2G VHT\n"));
				vhtmode = 1;
			}
		}
#endif /* SUPPORT_2G_VHT */
		if (info->vht_required) {
			only_mode = OMC_VHT;
		}
		WL_ERR(("need vhtmode=%d, only_mode=%d\n", vhtmode, only_mode));

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0))
		// HE
		if (info->he_cap) {
			hemode = 1;
		}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0))
		if (info->he_required) {
			only_mode = OMC_HE;
		}
#endif /* ((LINUX_VERSION_CODE >= KERNEL_VERSION(5, 8, 0)) */
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)) */
		WL_ERR(("need hemode=%d, only_mode=%d\n", hemode, only_mode));

#if defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE)
#if defined(WL_HOSTAPD_CFG)
		chan = wl_cfg80211_get_ieee80211_chan(dev, info);
		chanspec = wl_freq_to_chanspec(chan->center_freq);
		/*
		 * This function compares the required phymodes of softAP/p2p with current phymodes
		 * and return TRUE if they are different
		*/
#ifdef DISABLE_MODE_CHANGE
		if (!dhd_force_max_mode) {
#endif /* DISABLE_MODE_CHANGE */
			if (is_mode_different(dhd, ifidx, nmode, vhtmode, hemode, chanspec, FALSE)) {
				WL_ERR(("Modes are different so down/up the radio\n"));
				need_down = TRUE;
				need_up = TRUE;
			} else {
				WL_ERR(("Modes are the same, pass through\n"));
			}
#endif /* defined(WL_HOSTAPD_CFG) */
			err = dhd_80211_mode_apply_by_value(dhd, ifidx, need_down, need_up,
				gmode, nmode, vhtmode, hemode, only_mode);
			if (err < 0) {
				WL_ERR(("fail to set mode, error %d\n", err));
			}
#ifdef DISABLE_MODE_CHANGE
		} else {
			WL_ERR(("Follow current wireless modes\n"));
		}
#endif /* DISABLE_MODE_CHANGE */
#else /* defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE) */
		UNUSED_PARAMETER(dhd);
		UNUSED_PARAMETER(ifidx);
		UNUSED_PARAMETER(need_down);
		UNUSED_PARAMETER(need_up);
		UNUSED_PARAMETER(gmode);
		UNUSED_PARAMETER(nmode);
		UNUSED_PARAMETER(vhtmode);
		UNUSED_PARAMETER(hemode);
		UNUSED_PARAMETER(only_mode);
		WL_ERR(("skip adaptation since not allow!\n"));
#endif /* defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE) */
	}
#endif /* ((LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)) */

#ifndef WL_COMPAT_WIRELESS
	if ((err = wl_cfg80211_set_chanspec(wiphy, dev, info)) < 0 ) {
		WL_ERR(("Set channel failed \n"));
		goto fail;
	}
#endif /* WL_COMPAT_WIRELESS */

	if ((err = wl_cfg80211_bcn_set_params(info, dev,
		dev_role, bssidx)) < 0) {
		WL_ERR(("Beacon params set failed \n"));
		goto fail;
	}

	/* Parse IEs */
	if ((err = wl_cfg80211_parse_ap_ies(dev, &info->beacon, &ies)) < 0) {
		WL_ERR(("Set IEs failed \n"));
		goto fail;
	}

	if ((err = wl_cfg80211_bcn_validate_sec(dev, &ies,
		dev_role, bssidx, info->privacy)) < 0)
	{
		WL_ERR(("Beacon set security failed \n"));
		goto fail;
	}

	if ((err = wl_cfg80211_bcn_bringup_ap(dev, &ies,
		dev_role, bssidx)) < 0) {
		WL_ERR(("Beacon bring up AP/GO failed \n"));
		goto fail;
	}

	/* Set GC/STA SCB expiry timings. */
	if ((err = wl_cfg80211_set_scb_timings(cfg, dev))) {
		WL_ERR(("scb setting failed \n"));
		goto fail;
	}

	wl_set_drv_status(cfg, CONNECTED, dev);
	WL_DBG(("** AP/GO Created **\n"));

#ifdef WL_CFG80211_ACL
	/* Enfoce Admission Control. */
	if ((err = wl_cfg80211_set_mac_acl(wiphy, dev, info->acl)) < 0) {
		WL_ERR(("Set ACL failed\n"));
	}
#endif /* WL_CFG80211_ACL */

	/* Set IEs to FW */
	if ((err = wl_cfg80211_set_ies(dev, &info->beacon, bssidx)) < 0)
		WL_ERR(("Set IEs failed \n"));

#ifdef WLDWDS
	if (dev->ieee80211_ptr->use_4addr) {
		if ((err = wl_cfg80211_set_mgmt_vndr_ies(cfg, ndev_to_cfgdev(dev), bssidx,
				VNDR_IE_ASSOCRSP_FLAG, (const u8 *)info->beacon.assocresp_ies,
				info->beacon.assocresp_ies_len)) < 0) {
			WL_ERR(("Set ASSOC RESP IE Failed\n"));
		}
	}
#endif /* WLDWDS */

	/* Enable Probe Req filter, WPS-AP certification 4.2.13 */
	if ((dev_role == NL80211_IFTYPE_AP) && (ies.wps_ie != NULL)) {
		bool pbc = 0;
		wl_validate_wps_ie((const char *) ies.wps_ie, ies.wps_ie_len, &pbc);
		if (pbc) {
			WL_DBG(("set WLC_E_PROBREQ_MSG\n"));
			wl_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, true);
		}
	}

	/* Configure hidden SSID */
	if (info->hidden_ssid != NL80211_HIDDEN_SSID_NOT_IN_USE) {
		if ((err = wldev_iovar_setint(dev, "closednet", 1)) < 0)
			WL_ERR(("failed to set hidden : %d\n", err));
		WL_DBG(("hidden_ssid_enum_val: %d \n", info->hidden_ssid));
	}

#ifdef SUPPORT_AP_RADIO_PWRSAVE
	if (dev_role == NL80211_IFTYPE_AP) {
		if (!wl_set_ap_rps(dev, FALSE, dev->name)) {
			wl_cfg80211_init_ap_rps(cfg);
		} else {
			WL_ERR(("Set rpsnoa failed \n"));
		}
	}
#endif /* SUPPORT_AP_RADIO_PWRSAVE */
fail:
	if (err) {
		WL_ERR(("ADD/SET beacon failed\n"));
		wl_flush_fw_log_buffer(dev, FW_LOGSET_MASK_ALL);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0) || defined(CFG80211_BKPORT_MLO)
		wl_cfg80211_stop_ap(wiphy, dev, 0);
#else
		wl_cfg80211_stop_ap(wiphy, dev);
#endif /* KERNEL_VERSION(5, 19, 0) || defined(CFG80211_BKPORT_MLO) */
		if (dev_role == NL80211_IFTYPE_AP) {
			dhd->op_mode &= ~DHD_FLAG_HOSTAP_MODE;
			/* Enable packet filter */
			if (dhd->early_suspended) {
				WL_ERR(("Enable pkt_filter\n"));
#ifdef PKT_FILTER_SUPPORT
				dhd_enable_packet_filter(TRUE, dhd);
#endif /* PKT_FILTER_SUPPORT */
#ifdef APF
				dhd_dev_apf_enable_filter(dhd_linux_get_primary_netdev(dhd));
#endif /* APF */
			}
#ifdef DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P
			/* If softap creation fails restore Frameburst */
			wl_cfg80211_set_frameburst(cfg, dhd->frameburst);
#endif /* DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P */
		}

#ifdef DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P
		else if (dev_role == NL80211_IFTYPE_P2P_GO) {
			/* If p2p creation fails restore Frameburst */
			dhd->op_mode &= ~DHD_FLAG_P2P_GO_MODE;
			wl_cfg80211_set_frameburst(cfg, dhd->frameburst);
		}
#endif /* DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P */
#ifdef WLTDLS
		if (bssidx == 0) {
			/* Since AP creation failed, re-enable TDLS */
			wl_cfg80211_tdls_config(cfg, TDLS_STATE_AP_DELETE, false);
		}
#endif /*  WLTDLS */

	}

	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0) || defined(CFG80211_BKPORT_MLO)
s32
wl_cfg80211_stop_ap(
	struct wiphy *wiphy,
	struct net_device *dev,
	unsigned int link_id)
#else
s32
wl_cfg80211_stop_ap(
	struct wiphy *wiphy,
	struct net_device *dev)
#endif /* LINUX_VER >= 5.19 || defined(CFG80211_BKPORT_MLO) */
{
	int err = 0;
	u32 dev_role = 0;
	int ap = 0;
	s32 bssidx = 0;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	s32 is_rsdb_supported = BCME_ERROR;
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);

	WL_DBG(("Enter \n"));

	if (wl_cfg80211_get_bus_state(cfg)) {
		/* since bus is down, iovar will fail. recovery path will bringup the bus. */
		WL_ERR(("bus is not ready\n"));
		return BCME_OK;
	}
	is_rsdb_supported = DHD_OPMODE_SUPPORTED(cfg->pub, DHD_FLAG_RSDB_MODE);
	if (is_rsdb_supported < 0)
		return (-ENODEV);

	wl_clr_drv_status(cfg, AP_CREATING, dev);
	wl_clr_drv_status(cfg, AP_CREATED, dev);
	cfg->ap_oper_channel = INVCHANSPEC;

	if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {
		dev_role = NL80211_IFTYPE_AP;
		WL_DBG(("stopping AP operation\n"));
	} else if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_GO) {
		dev_role = NL80211_IFTYPE_P2P_GO;
		WL_DBG(("stopping P2P GO operation\n"));
	} else {
		WL_ERR(("no AP/P2P GO interface is operational.\n"));
		return -EINVAL;
	}

	if ((bssidx = wl_get_bssidx_by_wdev(cfg, dev->ieee80211_ptr)) < 0) {
		WL_ERR(("find p2p index from wdev(%p) failed\n", dev->ieee80211_ptr));
		return BCME_ERROR;
	}

	if (!check_dev_role_integrity(cfg, dev_role)) {
		WL_ERR(("role integrity check failed \n"));
		err = -EINVAL;
		goto exit;
	}

	/* Free up resources */
	wl_cfg80211_cleanup_if(dev);

	/* Clear AP/GO connected status */
	wl_clr_drv_status(cfg, CONNECTED, dev);
	if ((err = wl_cfg80211_bss_up(cfg, dev, bssidx, 0)) < 0) {
		WL_ERR(("bss down error %d\n", err));
	}

	if (dev_role == NL80211_IFTYPE_AP) {
		/* Restore FB for SoftAP removal without specific chanspec check, as it's safe. */
#ifdef DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P
		wl_cfg80211_set_frameburst(cfg, dhd->frameburst);
#endif /* DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P */
		/* Enable packet filter */
		if (dhd->early_suspended) {
			WL_ERR(("Enable pkt_filter\n"));
#ifdef PKT_FILTER_SUPPORT
			dhd_enable_packet_filter(TRUE, dhd);
#endif /* PKT_FILTER_SUPPORT */
#ifdef APF
			dhd_dev_apf_enable_filter(dhd_linux_get_primary_netdev(dhd));
#endif /* APF */
		}

		if (is_rsdb_supported == 0) {
			/* For non-rsdb chips, we use stand alone AP. Do wl down on stop AP */
			err = wldev_ioctl_set(dev, WLC_UP, &ap, sizeof(s32));
			if (unlikely(err)) {
				WL_ERR(("WLC_UP error (%d)\n", err));
				err = -EINVAL;
				goto exit;
			}
		}

#ifdef WL_DISABLE_HE_SOFTAP
		if (wl_cfg80211_change_he_features(dev, cfg, bssidx, WL_HE_FEATURES_HE_AP,
			TRUE) != BCME_OK) {
			WL_ERR(("failed to set he features\n"));
		}
#endif /* WL_DISABLE_HE_SOFTAP */

		wl_cfg80211_clear_per_bss_ies(cfg, dev->ieee80211_ptr);
#ifdef SUPPORT_AP_RADIO_PWRSAVE
		if (!wl_set_ap_rps(dev, FALSE, dev->name)) {
			wl_cfg80211_init_ap_rps(cfg);
		} else {
			WL_ERR(("Set rpsnoa failed \n"));
		}
#endif /* SUPPORT_AP_RADIO_PWRSAVE */
	} else {
		/* Do we need to do something here */
		WL_DBG(("Stopping P2P GO \n"));
		/* Restore FB for P2P GO removal without specific chanspec check, as it's safe. */
#ifdef DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P
		wl_cfg80211_set_frameburst(cfg, dhd->frameburst);
#endif /* DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P */

#if defined(OEM_ANDROID)
		DHD_OS_WAKE_LOCK_CTRL_TIMEOUT_ENABLE((dhd_pub_t *)(cfg->pub),
			DHD_EVENT_TIMEOUT_MS*3);
		DHD_OS_WAKE_LOCK_TIMEOUT((dhd_pub_t *)(cfg->pub));
#endif

	}

	SUPP_LOG(("AP/GO Link down\n"));
exit:
	if (err) {
		/* In case of failure, flush fw logs */
		wl_flush_fw_log_buffer(dev, FW_LOGSET_MASK_ALL);
		SUPP_LOG(("AP/GO Link down fail. err:%d\n", err));
	}
#ifdef WLTDLS
	if (bssidx == 0) {
		/* re-enable TDLS if the number of connected interfaces is less than 2 */
		wl_cfg80211_tdls_config(cfg, TDLS_STATE_AP_DELETE, false);
	}
#endif /* WLTDLS */

	if (dev_role == NL80211_IFTYPE_AP) {
		/* clear the AP mode */
		dhd->op_mode &= ~DHD_FLAG_HOSTAP_MODE;
	}

#if defined(ARP_OFFLOAD_SUPPORT) && defined(DISABLE_ARP_OFFLOAD_ON_SOFTAP)
	/* Enable ARP Offload back for STA mode */
	if ((dhd->op_mode & DHD_FLAG_STA_MODE) && dhd->early_suspended) {
		WL_ERR(("Enable ARP Offload back for STA mode\n"));
		dhd_arp_offload_enable(dhd, TRUE);
	}
#endif /* ARP_OFFLOAD_SUPPORT && DISABLE_ARP_OFFLOAD_ON_SOFTAP */

#if defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE)
	if (cfg &&
#ifdef DISABLE_MODE_CHANGE
		!dhd_force_max_mode &&
#endif /* DISABLE_MODE_CHANGE */
		1) {
		int  ifidx     = dhd_net2idx(dhd->info, dev);

		/* recover 802.11 mode if possible */
		if (DHD_BAD_IF != ifidx) {
			int  need_down = FALSE;
			int  need_up   = TRUE;

#if defined(WL_HOSTAPD_CFG)
			dhd_if_t    *ifp = NULL;
			chanspec_t chanspec = 0;
			ifp = dhd_get_ifp(dhd, ifidx);
			/*
			 * This function compares the required phymodes of softAP/p2p
			 * with current phymodes and return TRUE if they are different
			*/
			if (is_mode_different(dhd, ifidx, ifp->nmode, ifp->vhtmode,
					ifp->hemode, chanspec, TRUE)) {
				WL_ERR(("Modes are different so down/up the radio\n"));
				need_down = TRUE;
				need_up = TRUE;
			}
#endif /* defined(WL_HOSTAPD_CFG) */
			dhd_80211_mode_apply(dhd, ifidx, need_down, need_up);
		}
	}
#endif /* defined(WL_HOSTAPD_CFG) || defined(SYNA_80211_MODE) */

	return err;
}

s32
wl_cfg80211_change_beacon(
	struct wiphy *wiphy,
	struct net_device *dev,
	struct cfg80211_beacon_data *info)
{
	s32 err = BCME_OK;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	struct parsed_ies ies;
	u32 dev_role = 0;
	s32 bssidx = 0;
	bool pbc = 0;

	WL_DBG(("Enter \n"));

	if ((bssidx = wl_get_bssidx_by_wdev(cfg, dev->ieee80211_ptr)) < 0) {
		WL_ERR(("Find p2p index from wdev(%p) failed\n", dev->ieee80211_ptr));
		return BCME_ERROR;
	}

	if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_GO) {
		dev_role = NL80211_IFTYPE_P2P_GO;
	} else if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {
		dev_role = NL80211_IFTYPE_AP;
	} else {
		err = -EINVAL;
		goto fail;
	}

	if (!check_dev_role_integrity(cfg, dev_role)) {
		err = -EINVAL;
		goto fail;
	}

	if ((dev_role == NL80211_IFTYPE_P2P_GO) && (cfg->p2p_wdev == NULL)) {
		WL_ERR(("P2P already down status!\n"));
		err = BCME_ERROR;
		goto fail;
	}

	/* Parse IEs */
	if ((err = wl_cfg80211_parse_ap_ies(dev, info, &ies)) < 0) {
		WL_ERR(("Parse IEs failed \n"));
		goto fail;
	}

	/* Set IEs to FW */
	if ((err = wl_cfg80211_set_ies(dev, info, bssidx)) < 0) {
		WL_ERR(("Set IEs failed \n"));
		goto fail;
	}

	if (dev_role == NL80211_IFTYPE_AP) {
		if (wl_cfg80211_hostapd_sec(dev, &ies, bssidx) < 0) {
			WL_ERR(("Hostapd update sec failed \n"));
			err = -EINVAL;
			goto fail;
		}
		/* Enable Probe Req filter, WPS-AP certification 4.2.13 */
		if ((dev_role == NL80211_IFTYPE_AP) && (ies.wps_ie != NULL)) {
			wl_validate_wps_ie((const char *) ies.wps_ie, ies.wps_ie_len, &pbc);
			WL_DBG((" WPS AP, wps_ie is exists pbc=%d\n", pbc));
			if (pbc)
				wl_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, true);
			else
				wl_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, false);
		}
	}

fail:
	if (err) {
		wl_flush_fw_log_buffer(dev, FW_LOGSET_MASK_ALL);
	}
	return err;
}
#else
s32
wl_cfg80211_add_set_beacon(struct wiphy *wiphy, struct net_device *dev,
	struct beacon_parameters *info)
{
	s32 err = BCME_OK;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	s32 ie_offset = 0;
	s32 bssidx = 0;
	u32 dev_role = NL80211_IFTYPE_AP;
	struct parsed_ies ies;
	bcm_tlv_t *ssid_ie;
	bool pbc = 0;
	bool privacy;
	bool is_bss_up = 0;
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);

	WL_DBG(("interval (%d) dtim_period (%d) head_len (%d) tail_len (%d)\n",
		info->interval, info->dtim_period, info->head_len, info->tail_len));

	if (dev == bcmcfg_to_prmry_ndev(cfg)) {
		dev_role = NL80211_IFTYPE_AP;
	}
#if defined(WL_ENABLE_P2P_IF)
	else if (dev == cfg->p2p_net) {
		/* Group Add request on p2p0 */
		dev = bcmcfg_to_prmry_ndev(cfg);
		dev_role = NL80211_IFTYPE_P2P_GO;
	}
#endif /* WL_ENABLE_P2P_IF */

	if ((bssidx = wl_get_bssidx_by_wdev(cfg, dev->ieee80211_ptr)) < 0) {
		WL_ERR(("Find p2p index from wdev(%p) failed\n", dev->ieee80211_ptr));
		return BCME_ERROR;
	}

	if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_GO) {
		dev_role = NL80211_IFTYPE_P2P_GO;
	} else if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {
		dhd->op_mode |= DHD_FLAG_HOSTAP_MODE;
	}

	if (!check_dev_role_integrity(cfg, dev_role)) {
		err = -ENODEV;
		goto fail;
	}

	if ((dev_role == NL80211_IFTYPE_P2P_GO) && (cfg->p2p_wdev == NULL)) {
		WL_ERR(("P2P already down status!\n"));
		err = BCME_ERROR;
		goto fail;
	}

	ie_offset = DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
	/* find the SSID */
	if ((ssid_ie = bcm_parse_tlvs((u8 *)&info->head[ie_offset],
		info->head_len - ie_offset,
		DOT11_MNG_SSID_ID)) != NULL) {
		if (dev_role == NL80211_IFTYPE_AP) {
			/* Store the hostapd SSID */
			bzero(&cfg->hostapd_ssid.SSID[0], DOT11_MAX_SSID_LEN);
			cfg->hostapd_ssid.SSID_len = MIN(ssid_ie->len, DOT11_MAX_SSID_LEN);
			memcpy(&cfg->hostapd_ssid.SSID[0], ssid_ie->data,
				cfg->hostapd_ssid.SSID_len);
		} else {
				/* P2P GO */
			bzero(&cfg->p2p->ssid.SSID[0], DOT11_MAX_SSID_LEN);
			cfg->p2p->ssid.SSID_len = MIN(ssid_ie->len, DOT11_MAX_SSID_LEN);
			memcpy(cfg->p2p->ssid.SSID, ssid_ie->data,
				cfg->p2p->ssid.SSID_len);
		}
	}

	if (wl_cfg80211_parse_ies((u8 *)info->tail,
		info->tail_len, &ies) < 0) {
		WL_ERR(("Beacon get IEs failed \n"));
		err = -EINVAL;
		goto fail;
	}

	if ((err = wl_cfg80211_set_mgmt_vndr_ies(cfg, ndev_to_cfgdev(dev), bssidx,
		VNDR_IE_BEACON_FLAG, (u8 *)info->tail,
		info->tail_len)) < 0) {
		WL_ERR(("Beacon set IEs failed \n"));
		goto fail;
	} else {
		WL_DBG(("Applied Vndr IEs for Beacon \n"));
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
	if ((err = wl_cfg80211_set_mgmt_vndr_ies(cfg, ndev_to_cfgdev(dev), bssidx,
		VNDR_IE_PRBRSP_FLAG, (u8 *)info->proberesp_ies,
		info->proberesp_ies_len)) < 0) {
		WL_ERR(("ProbeRsp set IEs failed \n"));
		goto fail;
	} else {
		WL_DBG(("Applied Vndr IEs for ProbeRsp \n"));
	}
#endif

	is_bss_up = wl_cfg80211_bss_isup(dev, bssidx);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
	privacy = info->privacy;
#else
	privacy = 0;
#endif
	if (!is_bss_up &&
		(wl_cfg80211_bcn_validate_sec(dev, &ies, dev_role, bssidx, privacy) < 0))
	{
		WL_ERR(("Beacon set security failed \n"));
		err = -EINVAL;
		goto fail;
	}

	/* Set BI and DTIM period */
	if (info->interval) {
		if ((err = wldev_ioctl_set(dev, WLC_SET_BCNPRD,
			&info->interval, sizeof(s32))) < 0) {
			WL_ERR(("Beacon Interval Set Error, %d\n", err));
			return err;
		}
	}
	if (info->dtim_period) {
		if ((err = wldev_ioctl_set(dev, WLC_SET_DTIMPRD,
			&info->dtim_period, sizeof(s32))) < 0) {
			WL_ERR(("DTIM Interval Set Error, %d\n", err));
			return err;
		}
	}

	/* If bss is already up, skip bring up */
	if (!is_bss_up &&
		(err = wl_cfg80211_bcn_bringup_ap(dev, &ies, dev_role, bssidx)) < 0)
	{
		WL_ERR(("Beacon bring up AP/GO failed \n"));
		goto fail;
	}

	/* Set GC/STA SCB expiry timings. */
	if ((err = wl_cfg80211_set_scb_timings(cfg, dev))) {
		WL_ERR(("scb setting failed \n"));
		goto fail;
	}

	if (wl_get_drv_status(cfg, AP_CREATED, dev)) {
		/* Soft AP already running. Update changed params */
		if (wl_cfg80211_hostapd_sec(dev, &ies, bssidx) < 0) {
			WL_ERR(("Hostapd update sec failed \n"));
			err = -EINVAL;
			goto fail;
		}
	}

	/* Enable Probe Req filter */
	if (((dev_role == NL80211_IFTYPE_P2P_GO) ||
		(dev_role == NL80211_IFTYPE_AP)) && (ies.wps_ie != NULL)) {
		wl_validate_wps_ie((char *) ies.wps_ie, ies.wps_ie_len, &pbc);
		if (pbc)
			wl_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, true);
	}

	WL_DBG(("** ADD/SET beacon done **\n"));
	wl_set_drv_status(cfg, CONNECTED, dev);

fail:
	if (err) {
		WL_ERR(("ADD/SET beacon failed\n"));
		if (dev_role == NL80211_IFTYPE_AP) {
			/* clear the AP mode */
			dhd->op_mode &= ~DHD_FLAG_HOSTAP_MODE;
		}
	}
	return err;

}

s32
wl_cfg80211_del_beacon(struct wiphy *wiphy, struct net_device *dev)
{
	int err = 0;
	s32 bssidx = 0;
	int infra = 0;
	struct wireless_dev *wdev = dev->ieee80211_ptr;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);

	WL_DBG(("Enter. \n"));

	if (!wdev) {
		WL_ERR(("wdev null \n"));
		return -EINVAL;
	}

	if ((wdev->iftype != NL80211_IFTYPE_P2P_GO) && (wdev->iftype != NL80211_IFTYPE_AP)) {
		WL_ERR(("Unspported iface type iftype:%d \n", wdev->iftype));
	}

	wl_clr_drv_status(cfg, AP_CREATING, dev);
	wl_clr_drv_status(cfg, AP_CREATED, dev);

	/* Clear AP/GO connected status */
	wl_clr_drv_status(cfg, CONNECTED, dev);

	cfg->ap_oper_channel = INVCHANSPEC;

	if ((bssidx = wl_get_bssidx_by_wdev(cfg, dev->ieee80211_ptr)) < 0) {
		WL_ERR(("find p2p index from wdev(%p) failed\n", dev->ieee80211_ptr));
		return BCME_ERROR;
	}

	/* Do bss down */
	if ((err = wl_cfg80211_bss_up(cfg, dev, bssidx, 0)) < 0) {
		WL_ERR(("bss down error %d\n", err));
	}

	/* fall through is intentional */
	err = wldev_ioctl_set(dev, WLC_SET_INFRA, &infra, sizeof(s32));
	if (err < 0) {
		WL_ERR(("SET INFRA error %d\n", err));
	}
	 wl_cfg80211_clear_per_bss_ies(cfg, dev->ieee80211_ptr);

	if (wdev->iftype == NL80211_IFTYPE_AP) {
		/* clear the AP mode */
		dhd->op_mode &= ~DHD_FLAG_HOSTAP_MODE;
	}

	return 0;
}
#endif /* LINUX_VERSION < VERSION(3,4,0) || WL_COMPAT_WIRELESS */

s32
wl_get_auth_assoc_status(struct bcm_cfg80211 *cfg, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	u32 reason = ntoh32(e->reason);
	u32 event = ntoh32(e->event_type);
	struct wl_security *sec = wl_read_prof(cfg, ndev, WL_PROF_SEC);

#if defined(DHD_ENABLE_BIGDATA_LOGGING)
	(void)memcpy_s(&cfg->event_auth_assoc, sizeof(wl_event_msg_t),
		e, sizeof(wl_event_msg_t));
	WL_DBG(("event=%d status %d reason %d \n",
		ntoh32(cfg->event_auth_assoc.event_type),
		ntoh32(cfg->event_auth_assoc.status),
		ntoh32(cfg->event_auth_assoc.reason)));
#endif /* DHD_ENABLE_BIGDATA_LOGGING */
	if (sec) {
		switch (event) {
		case WLC_E_ASSOC:
		case WLC_E_AUTH:
		case WLC_E_AUTH_IND:
			sec->auth_assoc_res_status = reason;
			break;
		default:
			break;
		}
	} else {
		WL_ERR(("sec is NULL\n"));
	}
	return 0;
}

/* The mainline kernel >= 3.2.0 has support for indicating new/del station
 * to AP/P2P GO via events. If this change is backported to kernel for which
 * this driver is being built, then define WL_CFG80211_STA_EVENT. You
 * should use this new/del sta event mechanism for BRCM supplicant >= 22.
 */
#if !defined(WL_CFG80211_STA_EVENT) && (LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0))
static s32
wl_notify_connect_status_ap_legacy(struct bcm_cfg80211 *cfg, struct net_device *ndev
	const wl_event_msg_t *e, void *data)
{
	s32 err = 0;
	u32 event = ntoh32(e->event_type);
	u32 reason = ntoh32(e->reason);
	u32 len = ntoh32(e->datalen);
	u32 status = ntoh32(e->status);

	bool isfree = false;
	u8 *mgmt_frame;
	u8 bsscfgidx = e->bsscfgidx;
	s32 freq;
	s32 channel;
	u8 *body = NULL;
	u16 fc = 0;
	u32 body_len = 0;

	struct ieee80211_supported_band *band;
	struct ether_addr da;
	struct ether_addr bssid;
	struct wiphy *wiphy = bcmcfg_to_wiphy(cfg);
	channel_info_t ci;
	u8 ioctl_buf[WLC_IOCTL_SMLEN];

	WL_DBG(("Enter \n"));
	if (!len && (event == WLC_E_DEAUTH)) {
		len = 2; /* reason code field */
		data = &reason;
	}
	if (len) {
		body = (u8 *)MALLOCZ(cfg->osh, len);
		if (body == NULL) {
			WL_ERR(("wl_notify_connect_status: Failed to allocate body\n"));
			return WL_INVALID;
		}
	}
	bzero(&bssid, ETHER_ADDR_LEN);
	WL_DBG(("Enter event %d ndev %p\n", event, ndev));
	if (wl_get_mode_by_netdev(cfg, ndev) == WL_INVALID) {
		MFREE(cfg->osh, body, len);
		return WL_INVALID;
	}
	if (len)
		memcpy(body, data, len);

	wldev_iovar_getbuf_bsscfg(ndev, "cur_etheraddr",
		NULL, 0, ioctl_buf, sizeof(ioctl_buf), bsscfgidx, NULL);
	memcpy(da.octet, ioctl_buf, ETHER_ADDR_LEN);
	bzero(&bssid, sizeof(bssid));
	err = wldev_ioctl_get(ndev, WLC_GET_BSSID, &bssid, ETHER_ADDR_LEN);
	switch (event) {
		case WLC_E_ASSOC_IND:
			fc = FC_ASSOC_REQ;
			break;
		case WLC_E_REASSOC_IND:
			fc = FC_REASSOC_REQ;
			break;
		case WLC_E_DISASSOC_IND:
			fc = FC_DISASSOC;
			break;
		case WLC_E_DEAUTH_IND:
			fc = FC_DISASSOC;
			break;
		case WLC_E_DEAUTH:
			fc = FC_DISASSOC;
			break;
		default:
			fc = 0;
			goto exit;
	}
	err = wldev_iovar_getint(ndev, "chanspec", (s32 *)&chanspec);
	if (unlikely(err)) {
		MFREE(cfg->osh, body, len);
		WL_ERR(("%s: Could not get chanspec %d\n", __FUNCTION__, err));
		return err;
	}
	chanspec = wl_chspec_driver_to_host(chanspec);
	freq = wl_channel_to_frequency(wf_chspec_ctlchan(chanspec), CHSPEC_BAND(chanspec));
	body_len = len;
	err = wl_frame_get_mgmt(cfg, fc, &da, &e->addr, &bssid,
		&mgmt_frame, &len, body);
	if (err < 0)
		goto exit;
	isfree = true;

	if ((event == WLC_E_ASSOC_IND && reason == DOT11_SC_SUCCESS) ||
			(event == WLC_E_DISASSOC_IND) ||
			((event == WLC_E_DEAUTH_IND) || (event == WLC_E_DEAUTH))) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
		cfg80211_rx_mgmt(ndev, freq, 0, mgmt_frame, len, 0);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0))
		cfg80211_rx_mgmt(ndev, freq, 0, mgmt_frame, len, 0, GFP_ATOMIC);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)) || \
		defined(WL_COMPAT_WIRELESS)
		cfg80211_rx_mgmt(ndev, freq, 0, mgmt_frame, len, GFP_ATOMIC);
#else
		cfg80211_rx_mgmt(ndev, freq, mgmt_frame, len, GFP_ATOMIC);
#endif /* LINUX_VERSION >= VERSION(3, 18,0) || WL_COMPAT_WIRELESS */
	}

exit:
	if (isfree) {
		MFREE(cfg->osh, mgmt_frame, len);
	}
	if (body) {
		MFREE(cfg->osh, body, body_len);
	}

}
#endif /* WL_CFG80211_STA_EVENT || KERNEL_VER < 3.2 */

s32
wl_notify_connect_status_ap(struct bcm_cfg80211 *cfg, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	s32 err = 0;
	u32 event = ntoh32(e->event_type);
	u32 reason = ntoh32(e->reason);
	u32 len = ntoh32(e->datalen);
	u32 status = ntoh32(e->status);
#if defined(WL_CFG80211_STA_EVENT) || (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
	struct station_info sinfo;
#endif /* (LINUX_VERSION >= VERSION(3,2,0)) || !WL_CFG80211_STA_EVENT */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0))
	struct cfg80211_update_owe_info owe_info;
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0) */
#ifdef BIGDATA_SOFTAP
	dhd_pub_t *dhdp;
#endif /* BIGDATA_SOFTAP */

	WL_INFORM_MEM(("[%s] Mode AP/GO. Event:%d status:%d reason:%d\n",
		ndev->name, event, ntoh32(e->status), reason));

#ifdef WL_CLIENT_SAE
	if (event == WLC_E_AUTH && ntoh32(e->auth_type) == DOT11_SAE) {
		err = wl_handle_auth_event(cfg, ndev, e, data);
		if (err != BCME_OK) {
			return err;
		}
	}
#endif /* WL_CLIENT_SAE */

	if (event == WLC_E_AUTH_IND) {
#ifdef WL_SAE
		if (ntoh32(e->auth_type) == DOT11_SAE) {
			wl_bss_handle_sae_auth(cfg, ndev, e, data);
		}
#endif /* WL_SAE */
		wl_get_auth_assoc_status(cfg, ndev, e, data);
		return 0;
	}
	/* if link down, bsscfg is disabled. */
	if (event == WLC_E_LINK && reason == WLC_E_LINK_BSSCFG_DIS &&
		wl_get_p2p_status(cfg, IF_DELETING) && (ndev != bcmcfg_to_prmry_ndev(cfg))) {
		wl_add_remove_eventmsg(ndev, WLC_E_PROBREQ_MSG, false);
		WL_INFORM_MEM(("AP mode link down !! \n"));
		complete(&cfg->iface_disable);
		return 0;
	}

	/* if link up, bsscfg is enabled */
	if (event == WLC_E_LINK && status == WLC_E_STATUS_SUCCESS &&
		(reason == WLC_E_REASON_INITIAL_ASSOC) &&
		(ndev != bcmcfg_to_prmry_ndev(cfg)) && (cfg->p2p_supported) &&
		(wl_get_mode_by_netdev(cfg, ndev) == WL_MODE_AP)) {
		WL_INFORM_MEM(("AP mode link up !! \n"));
		complete(&cfg->iface_up);
	}

	if ((event == WLC_E_LINK) && (status == WLC_E_STATUS_SUCCESS) &&
		(reason == WLC_E_REASON_INITIAL_ASSOC) &&
		(wl_get_mode_by_netdev(cfg, ndev) == WL_MODE_AP)) {
		if (!wl_get_drv_status(cfg, AP_CREATED, ndev)) {
			/* AP/GO brought up successfull in firmware */
			WL_INFORM_MEM(("AP/GO Link up\n"));
			wl_set_drv_status(cfg, AP_CREATED, ndev);
			if (delayed_work_pending(&cfg->ap_work)) {
				cancel_delayed_work_sync(&cfg->ap_work);
				WL_DBG(("cancelled ap_work\n"));
			}
#ifdef BIGDATA_SOFTAP
			wl_ap_stainfo_init(cfg);
#endif /* BIGDATA_SOFTAP */
			return 0;
		}
	}

	if (event == WLC_E_DISASSOC_IND || event == WLC_E_DEAUTH_IND || event == WLC_E_DEAUTH) {
		WL_DBG(("event %s(%d) status %d reason %d\n",
		bcmevent_get_name(event), event, ntoh32(e->status), reason));
	}

#ifdef BIGDATA_SOFTAP
	if (event == WLC_E_LINK && reason == WLC_E_LINK_BSSCFG_DIS) {
		WL_ERR(("AP link down - skip get sta data\n"));
	} else {
		dhdp = (dhd_pub_t *)(cfg->pub);
		if (dhdp && dhdp->op_mode & DHD_FLAG_HOSTAP_MODE) {
			dhd_schedule_gather_ap_stadata(cfg, ndev, e);
		}
	}
#endif /* BIGDATA_SOFTAP */

#if !defined(WL_CFG80211_STA_EVENT) && !defined(WL_COMPAT_WIRELESS) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0))
	err = wl_notify_connect_status_ap_legacy(cfg, ndev, e, data);
#else /* LINUX_VERSION < VERSION(3,2,0) && !WL_CFG80211_STA_EVENT && !WL_COMPAT_WIRELESS */
	memset_s(&sinfo, sizeof(sinfo), 0, sizeof(sinfo));
	if ((event == WLC_E_ASSOC_IND) ||
		((event == WLC_E_REASSOC_IND) && (reason == DOT11_SC_SUCCESS)) ||
		((event == WLC_E_REASSOC_IND) &&
			(reason == DOT11_SC_ASSOC_TRY_LATER) &&
			(bcm_parse_tlvs(data, len, DOT11_MNG_RSN_ID)) != NULL)) {
		/* Linux ver >= 4.0 assoc_req_ies_len is used instead of
		 * STATION_INFO_ASSOC_REQ_IES flag
		 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0))
		sinfo.filled = STA_INFO_BIT(INFO_ASSOC_REQ_IES);
#endif /*  (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) */
		if (!data) {
			WL_ERR(("No IEs present in ASSOC/REASSOC_IND"));
			return -EINVAL;
		}
		sinfo.assoc_req_ies = data;
		sinfo.assoc_req_ies_len = len;
		WL_INFORM_MEM(("[%s] new sta event for "MACDBG "\n",
			ndev->name, MAC2STRDBG(e->addr.octet)));
		cfg80211_new_sta(ndev, e->addr.octet, &sinfo, GFP_ATOMIC);
#ifdef WL_WPS_SYNC
		wl_wps_session_update(ndev, WPS_STATE_LINKUP, e->addr.octet);
#endif /* WL_WPS_SYNC */
	} else if ((event == WLC_E_DEAUTH_IND) ||
		((event == WLC_E_DEAUTH) && (reason != DOT11_RC_RESERVED)) ||
		(event == WLC_E_DISASSOC_IND)) {
		/*
		 * WAR: Dongle sends WLC_E_DEAUTH event with DOT11_RC_RESERVED
		 * to delete flowring in case of PCIE Full dongle.
		 * By deleting flowring on SoftAP interface we can avoid any issues
		 * due to stale/bad state of flowring.
		 * Therefore, we don't need to notify the client dissaociation to Hostapd
		 * in this case.
		 * Please refer to the RB:115182 to understand the case more clearly.
		 */
		WL_INFORM_MEM(("[%s] del sta event for "MACDBG "\n",
			ndev->name, MAC2STRDBG(e->addr.octet)));
		cfg80211_del_sta(ndev, e->addr.octet, GFP_ATOMIC);
#ifdef WL_WPS_SYNC
		wl_wps_session_update(ndev, WPS_STATE_LINKDOWN, e->addr.octet);
#endif /* WL_WPS_SYNC */
	}
#endif /* LINUX_VERSION < VERSION(3,2,0) && !WL_CFG80211_STA_EVENT && !WL_COMPAT_WIRELESS */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0))
	else if (event == WLC_E_OWE_INFO) {
		if (!data) {
			WL_ERR(("No DH-IEs present in ASSOC/REASSOC_IND"));
			return -EINVAL;
		}
		if (wl_dbg_level & WL_DBG_DBG) {
			prhex("FW-OWEIE ", (uint8 *)data, len);
		}
		eacopy(e->addr.octet, owe_info.peer);
		owe_info.ie = data;
		owe_info.ie_len = len;
		WL_INFORM_MEM(("Recieved owe_info. Mac addr " MACDBG"\n",
			MAC2STRDBG((const u8*)(&e->addr))));
		cfg80211_update_owe_info_event(ndev, &owe_info, GFP_ATOMIC);
	}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0) */
	return err;
}

s32
wl_frame_get_mgmt(struct bcm_cfg80211 *cfg, u16 fc,
	const struct ether_addr *da, const struct ether_addr *sa,
	const struct ether_addr *bssid, u8 **pheader, u32 *body_len, u8 *pbody)
{
	struct dot11_management_header *hdr;
	u32 totlen = 0;
	s32 err = 0;
	u8 *offset;
	u32 prebody_len = *body_len;
	switch (fc) {
		case FC_ASSOC_REQ:
			/* capability , listen interval */
			totlen = DOT11_ASSOC_REQ_FIXED_LEN;
			*body_len += DOT11_ASSOC_REQ_FIXED_LEN;
			break;

		case FC_REASSOC_REQ:
			/* capability, listen inteval, ap address */
			totlen = DOT11_REASSOC_REQ_FIXED_LEN;
			*body_len += DOT11_REASSOC_REQ_FIXED_LEN;
			break;
	}
	totlen += DOT11_MGMT_HDR_LEN + prebody_len;
	*pheader = (u8 *)MALLOCZ(cfg->osh, totlen);
	if (*pheader == NULL) {
		WL_ERR(("memory alloc failed \n"));
		return -ENOMEM;
	}
	hdr = (struct dot11_management_header *) (*pheader);
	hdr->fc = htol16(fc);
	hdr->durid = 0;
	hdr->seq = 0;
	offset = (u8*)(hdr + 1) + (totlen - DOT11_MGMT_HDR_LEN - prebody_len);
	bcopy((const char*)da, (u8*)&hdr->da, ETHER_ADDR_LEN);
	bcopy((const char*)sa, (u8*)&hdr->sa, ETHER_ADDR_LEN);
	bcopy((const char*)bssid, (u8*)&hdr->bssid, ETHER_ADDR_LEN);
	if ((pbody != NULL) && prebody_len)
		bcopy((const char*)pbody, offset, prebody_len);
	*body_len = totlen;
	return err;
}

#if defined(WLTDLS)
bool wl_cfg80211_is_tdls_tunneled_frame(void *frame, u32 frame_len)
{
	unsigned char *data;

	if (frame == NULL) {
		WL_ERR(("Invalid frame \n"));
		return false;
	}

	if (frame_len < 5) {
		WL_ERR(("Invalid frame length [%d] \n", frame_len));
		return false;
	}

	data = frame;

	if (!memcmp(data, TDLS_TUNNELED_PRB_REQ, 5) ||
		!memcmp(data, TDLS_TUNNELED_PRB_RESP, 5)) {
		WL_DBG(("TDLS Vendor Specific Received type\n"));
		return true;
	}

	return false;
}
#endif /* WLTDLS */

#ifdef WLTDLS
s32
wl_tdls_event_handler(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
	const wl_event_msg_t *e, void *data) {

	struct net_device *ndev = NULL;
	u32 reason = ntoh32(e->reason);
	s8 *msg = NULL;
#ifdef DISABLE_WL_FRAMEBURST_2G20_TDLS
	dhd_pub_t *dhdp = (dhd_pub_t *)(cfg->pub);
#endif /* DISABLE_WL_FRAMEBURST_2G20_TDLS */
	ndev = cfgdev_to_wlc_ndev(cfgdev, cfg);

	switch (reason) {
	case WLC_E_TDLS_PEER_DISCOVERED :
		msg = " TDLS PEER DISCOVERD ";
		break;
	case WLC_E_TDLS_PEER_CONNECTED :
		if (cfg->tdls_mgmt_frame) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
			cfg80211_rx_mgmt(cfgdev, cfg->tdls_mgmt_freq, 0,
					cfg->tdls_mgmt_frame, cfg->tdls_mgmt_frame_len, 0);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0))
			cfg80211_rx_mgmt(cfgdev, cfg->tdls_mgmt_freq, 0,
					cfg->tdls_mgmt_frame, cfg->tdls_mgmt_frame_len,	0,
					GFP_ATOMIC);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)) || \
			defined(WL_COMPAT_WIRELESS)
			cfg80211_rx_mgmt(cfgdev, cfg->tdls_mgmt_freq, 0,
					cfg->tdls_mgmt_frame, cfg->tdls_mgmt_frame_len,
					GFP_ATOMIC);
#else
			cfg80211_rx_mgmt(cfgdev, cfg->tdls_mgmt_freq,
					cfg->tdls_mgmt_frame, cfg->tdls_mgmt_frame_len, GFP_ATOMIC);

#endif /* LINUX_VERSION >= VERSION(3, 18,0) || WL_COMPAT_WIRELESS */
		}
		msg = " TDLS PEER CONNECTED ";
#ifdef SUPPORT_SET_CAC
		/* TDLS connect reset CAC */
		wl_cfg80211_set_cac(cfg, 0);
#endif /* SUPPORT_SET_CAC */
#ifdef DISABLE_WL_FRAMEBURST_2G20_TDLS
		if (dhdp && dhdp->disable_2g20_fb) {
			if(wl_cfg80211_check_is2G20(ndev)) {
				/* Disable Frameburst only for stand-alone 2GHz TDLS */
				WL_INFORM(("Disabling frameburst for 2GHz-20MHz TDLS\n"));
				wl_cfg80211_set_frameburst(cfg, FALSE);
			}
		}
#endif /* DISABLE_WL_FRAMEBURST_2G20_TDLS */
		break;
	case WLC_E_TDLS_PEER_DISCONNECTED :
		if (cfg->tdls_mgmt_frame) {
			MFREE(cfg->osh, cfg->tdls_mgmt_frame, cfg->tdls_mgmt_frame_len);
			cfg->tdls_mgmt_frame_len = 0;
			cfg->tdls_mgmt_freq = 0;
		}
		msg = "TDLS PEER DISCONNECTED ";
#ifdef SUPPORT_SET_CAC
		/* TDLS disconnec, set CAC */
		wl_cfg80211_set_cac(cfg, 1);
#endif /* SUPPORT_SET_CAC */
#ifdef DISABLE_WL_FRAMEBURST_2G20_TDLS
		/* Restore the Frameburst back to its previous value after 2G20 operation */
		if (dhdp && dhdp->disable_2g20_fb) {
			if (wl_cfg80211_check_is2G20(ndev)) {
				/* Enable Frameburst back for 2GHz TDLS once link is teardown */
				WL_INFORM(("Enabling frameburst for 2GHz-20MHz TDLS after teardown\n"));
				wl_cfg80211_set_frameburst(cfg, dhdp->frameburst);
			}
		}
#endif /* DISABLE_WL_FRAMEBURST_2G20_TDLS */
		break;
	}
	if (msg) {
		WL_ERR(("%s: " MACDBG " on %s ndev\n", msg, MAC2STRDBG((const u8*)(&e->addr)),
			(bcmcfg_to_prmry_ndev(cfg) == ndev) ? "primary" : "secondary"));
	}
	return 0;

}

#endif /* WLTDLS */

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 2, 0)) || defined(WL_COMPAT_WIRELESS)
#if (defined(CONFIG_ARCH_MSM) && defined(TDLS_MGMT_VERSION2)) || \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3, 16, 0) && \
	LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
s32
wl_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
	u8 *peer, u8 action_code, u8 dialog_token, u16 status_code,
	u32 peer_capability, const u8 *buf, size_t len)
#elif ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)) && \
		(LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)))
s32 wl_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
	const u8 *peer, u8 action_code, u8 dialog_token, u16 status_code,
	u32 peer_capability, const u8 *buf, size_t len)
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
s32 wl_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
       const u8 *peer, u8 action_code, u8 dialog_token, u16 status_code,
       u32 peer_capability, bool initiator, const u8 *buf, size_t len)
#else /* CONFIG_ARCH_MSM && TDLS_MGMT_VERSION2 */
s32
wl_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
	u8 *peer, u8 action_code, u8 dialog_token, u16 status_code,
	const u8 *buf, size_t len)
#endif /* CONFIG_ARCH_MSM && TDLS_MGMT_VERSION2 */
{
	s32 ret = 0;
#if defined(TDLS_MSG_ONLY_WFD) && defined(WLTDLS)
	struct bcm_cfg80211 *cfg;
	tdls_wfd_ie_iovar_t info;
	bzero(&info, sizeof(info));
	cfg = wl_get_cfg(dev);

#if defined(CONFIG_ARCH_MSM) && defined(TDLS_MGMT_VERSION2)
	/* Some customer platform back ported this feature from kernel 3.15 to kernel 3.10
	 * and that cuases build error
	 */
	BCM_REFERENCE(peer_capability);
#endif /* CONFIG_ARCH_MSM && TDLS_MGMT_VERSION2 */

	switch (action_code) {
		/* We need to set TDLS Wifi Display IE to firmware
		 * using tdls_wfd_ie iovar
		 */
		case WLAN_TDLS_SET_PROBE_WFD_IE:
			WL_ERR(("wl_cfg80211_tdls_mgmt: WLAN_TDLS_SET_PROBE_WFD_IE\n"));
			info.mode = TDLS_WFD_PROBE_IE_TX;

			if (len > sizeof(info.data)) {
				return -EINVAL;
			}
			memcpy(&info.data, buf, len);
			info.length = len;
			break;
		case WLAN_TDLS_SET_SETUP_WFD_IE:
			WL_ERR(("wl_cfg80211_tdls_mgmt: WLAN_TDLS_SET_SETUP_WFD_IE\n"));
			info.mode = TDLS_WFD_IE_TX;

			if (len > sizeof(info.data)) {
				return -EINVAL;
			}
			memcpy(&info.data, buf, len);
			info.length = len;
			break;
		case WLAN_TDLS_SET_WFD_ENABLED:
			WL_ERR(("wl_cfg80211_tdls_mgmt: WLAN_TDLS_SET_MODE_WFD_ENABLED\n"));
			dhd_tdls_set_mode((dhd_pub_t *)(cfg->pub), true);
			goto out;
		case WLAN_TDLS_SET_WFD_DISABLED:
			WL_ERR(("wl_cfg80211_tdls_mgmt: WLAN_TDLS_SET_MODE_WFD_DISABLED\n"));
			dhd_tdls_set_mode((dhd_pub_t *)(cfg->pub), false);
			goto out;
		default:
			WL_ERR(("Unsupported action code : %d\n", action_code));
			goto out;
	}
	ret = wldev_iovar_setbuf(dev, "tdls_wfd_ie", &info, sizeof(info),
			cfg->ioctl_buf, WLC_IOCTL_MAXLEN, &cfg->ioctl_buf_sync);

	if (ret) {
		WL_ERR(("tdls_wfd_ie error %d\n", ret));
	}

out:
#endif /* TDLS_MSG_ONLY_WFD && WLTDLS */
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0))
s32
wl_cfg80211_tdls_oper(struct wiphy *wiphy, struct net_device *dev,
	const u8 *peer, enum nl80211_tdls_operation oper)
#else
s32
wl_cfg80211_tdls_oper(struct wiphy *wiphy, struct net_device *dev,
	u8 *peer, enum nl80211_tdls_operation oper)
#endif
{
	s32 ret = 0;
#ifdef WLTDLS
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	tdls_iovar_t info;
	dhd_pub_t *dhdp;
	bool tdls_auto_mode = false;
	dhdp = (dhd_pub_t *)(cfg->pub);
	bzero(&info, sizeof(tdls_iovar_t));
	if (peer) {
		memcpy(&info.ea, peer, ETHER_ADDR_LEN);
	} else {
		return -1;
	}
	switch (oper) {
	case NL80211_TDLS_DISCOVERY_REQ:
		/* If the discovery request is broadcast then we need to set
		 * info.mode to Tunneled Probe Request
		 */
		if (memcmp(peer, (const uint8 *)BSSID_BROADCAST, ETHER_ADDR_LEN) == 0) {
			info.mode = TDLS_MANUAL_EP_WFD_TPQ;
			WL_ERR(("wl_cfg80211_tdls_oper: TDLS TUNNELED PRBOBE REQUEST\n"));
		} else {
			info.mode = TDLS_MANUAL_EP_DISCOVERY;
		}
		break;
	case NL80211_TDLS_SETUP:
		if (dhdp->tdls_mode == true) {
			info.mode = TDLS_MANUAL_EP_CREATE;
			tdls_auto_mode = false;
			/* Do tear down and create a fresh one */
			ret = wl_cfg80211_tdls_config(cfg, TDLS_STATE_TEARDOWN, tdls_auto_mode);
			if (ret < 0) {
				return ret;
			}
		} else {
			tdls_auto_mode = true;
		}
		break;
	case NL80211_TDLS_TEARDOWN:
		info.mode = TDLS_MANUAL_EP_DELETE;
		break;
	default:
		WL_ERR(("Unsupported operation : %d\n", oper));
		goto out;
	}
	/* turn on TDLS */
	ret = wl_cfg80211_tdls_config(cfg, TDLS_STATE_SETUP, tdls_auto_mode);
	if (ret < 0) {
		return ret;
	}
	if (info.mode) {
		ret = wldev_iovar_setbuf(dev, "tdls_endpoint", &info, sizeof(info),
			cfg->ioctl_buf, WLC_IOCTL_MAXLEN, &cfg->ioctl_buf_sync);
		if (ret) {
			WL_ERR(("tdls_endpoint error %d\n", ret));
		}
	}
out:
	/* use linux generic error code instead of firmware error code */
	if (ret) {
		wl_flush_fw_log_buffer(dev, FW_LOGSET_MASK_ALL);
		return -ENOTSUPP;
	}
#endif /* WLTDLS */
	return ret;
}
#endif /* LINUX_VERSION > VERSION(3,2,0) || WL_COMPAT_WIRELESS */

static bool check_dev_role_integrity(struct bcm_cfg80211 *cfg, u32 dev_role)
{
	dhd_pub_t *dhd = (dhd_pub_t *)(cfg->pub);
	if (((dev_role == NL80211_IFTYPE_AP) &&
		!(dhd->op_mode & DHD_FLAG_HOSTAP_MODE)) ||
		((dev_role == NL80211_IFTYPE_P2P_GO) &&
		!(dhd->op_mode & DHD_FLAG_P2P_GO_MODE)))
	{
		WL_ERR(("device role select failed role:%d op_mode:%d \n", dev_role, dhd->op_mode));
		return false;
	}
	return true;
}

s32
wl_cfg80211_dfs_ap_move(struct net_device *ndev, char *data, char *command, int total_len)
{
	char ioctl_buf[WLC_IOCTL_SMLEN];
	int err = 0;
	uint32 val = 0;
	chanspec_t chanspec = 0;
	int abort;
	int bytes_written = 0;
	struct wl_dfs_ap_move_status_v2 *status;
	char chanbuf[CHANSPEC_STR_LEN];
	const char *dfs_state_str[DFS_SCAN_S_MAX] = {
		"Radar Free On Channel",
		"Radar Found On Channel",
		"Radar Scan In Progress",
		"Radar Scan Aborted",
		"RSDB Mode switch in Progress For Scan"
	};
	if (ndev->ieee80211_ptr->iftype != NL80211_IFTYPE_AP) {
		bytes_written = snprintf(command, total_len, "AP is not up\n");
		return bytes_written;
	}
	if (!*data) {
		if ((err = wldev_iovar_getbuf(ndev, "dfs_ap_move", NULL, 0,
				ioctl_buf, sizeof(ioctl_buf), NULL))) {
			WL_ERR(("setting dfs_ap_move failed with err=%d \n", err));
			return err;
		}
		status = (struct wl_dfs_ap_move_status_v2 *)ioctl_buf;

		if (status->version != WL_DFS_AP_MOVE_VERSION) {
			err = BCME_UNSUPPORTED;
			WL_ERR(("err=%d version=%d\n", err, status->version));
			return err;
		}

		if (status->move_status != (int8) DFS_SCAN_S_IDLE) {
			chanspec = wl_chspec_driver_to_host(status->chanspec);
			if (chanspec != 0 && chanspec != INVCHANSPEC) {
				wf_chspec_ntoa(chanspec, chanbuf);
				bytes_written = snprintf(command, total_len,
					"AP Target Chanspec %s (0x%x)\n", chanbuf, chanspec);
			}
			bytes_written += snprintf(command + bytes_written,
					total_len - bytes_written,
					"%s\n", dfs_state_str[status->move_status]);
			return bytes_written;
		} else {
			bytes_written = snprintf(command, total_len, "dfs AP move in IDLE state\n");
			return bytes_written;
		}
	}

	abort = bcm_atoi(data);
	if (abort == -1) {
		if ((err = wldev_iovar_setbuf(ndev, "dfs_ap_move", &abort,
				sizeof(int), ioctl_buf, sizeof(ioctl_buf), NULL)) < 0) {
			WL_ERR(("seting dfs_ap_move failed with err %d\n", err));
			return err;
		}
	} else {
		chanspec = wf_chspec_aton(data);
		if (chanspec != 0) {
			val = wl_chspec_host_to_driver(chanspec);
			if (val != INVCHANSPEC) {
				if ((err = wldev_iovar_setbuf(ndev, "dfs_ap_move", &val,
					sizeof(int), ioctl_buf, sizeof(ioctl_buf), NULL)) < 0) {
					WL_ERR(("seting dfs_ap_move failed with err %d\n", err));
					return err;
				}
				WL_DBG((" set dfs_ap_move successfull"));
			} else {
				err = BCME_USAGE_ERROR;
			}
		}
	}
	return err;
}

#ifdef WL_CFG80211_ACL
static int
wl_cfg80211_set_mac_acl(struct wiphy *wiphy, struct net_device *cfgdev,
	const struct cfg80211_acl_data *acl)
{
	int i;
	int ret = 0;
	int macnum = 0;
	int macmode = MACLIST_MODE_DISABLED;
	struct maclist *list;
	struct bcm_cfg80211 *cfg = wl_get_cfg(cfgdev);

	/* get the MAC filter mode */
	if (acl && acl->acl_policy == NL80211_ACL_POLICY_DENY_UNLESS_LISTED) {
		macmode = MACLIST_MODE_ALLOW;
	} else if (acl && acl->acl_policy == NL80211_ACL_POLICY_ACCEPT_UNLESS_LISTED &&
	acl->n_acl_entries) {
		macmode = MACLIST_MODE_DENY;
	}

	/* if acl == NULL, macmode is still disabled.. */
	if (macmode == MACLIST_MODE_DISABLED) {
		if ((ret = wl_android_set_ap_mac_list(cfgdev, macmode, NULL)) != 0)
			WL_ERR(("wl_cfg80211_set_mac_acl: Setting MAC list"
				" failed error=%d\n", ret));

		return ret;
	}

	macnum = acl->n_acl_entries;
	if (macnum < 0 || macnum > MAX_NUM_MAC_FILT) {
		WL_ERR(("wl_cfg80211_set_mac_acl: invalid number of MAC address entries %d\n",
			macnum));
		return -1;
	}

	/* allocate memory for the MAC list */
	list = (struct maclist *)MALLOC(cfg->osh, sizeof(int) +
		sizeof(struct ether_addr) * macnum);
	if (!list) {
		WL_ERR(("wl_cfg80211_set_mac_acl: failed to allocate memory\n"));
		return -1;
	}

	/* prepare the MAC list */
	list->count = htod32(macnum);
	for (i = 0; i < macnum; i++) {
		memcpy(&list->ea[i], &acl->mac_addrs[i], ETHER_ADDR_LEN);
	}
	/* set the list */
	if ((ret = wl_android_set_ap_mac_list(cfgdev, macmode, list)) != 0)
		WL_ERR(("wl_cfg80211_set_mac_acl: Setting MAC list failed error=%d\n", ret));

	MFREE(cfg->osh, list, sizeof(int) +
		sizeof(struct ether_addr) * macnum);

	return ret;
}
#endif /* WL_CFG80211_ACL */

#ifdef WL_CFG80211_MONITOR
int
wl_cfg80211_set_monitor_channel(struct wiphy *wiphy, struct cfg80211_chan_def *chandef)
{
	int err = BCME_OK;
	struct bcm_cfg80211 *cfg;
	struct net_device *ndev;
	dhd_pub_t *dhdp;
	u32 bw = WL_CHANSPEC_BW_20;
	chanspec_t chspec = 0;

	WL_TRACE(("%s: Enter\n", __FUNCTION__));

	if (!wiphy) {
		WL_ERR(("%s: wiphy is null\n", __FUNCTION__));
		return BCME_ERROR;
	}

	cfg = wiphy_priv(wiphy);
	if (!cfg) {
		WL_ERR(("%s: cfg is null\n", __FUNCTION__));
		return BCME_ERROR;
	}

	dhdp = (dhd_pub_t *)(cfg->pub);
	if (!dhdp->monitor_enable) {
		WL_ERR(("%s: Monitor mode is not enabled\n", __FUNCTION__));
		return BCME_UNSUPPORTED;
	}

	ndev = bcmcfg_to_prmry_ndev(cfg);
	/* Convert from struct cfg80211_chan_def to chanspec_t */
	chspec = wl_freq_to_chanspec(chandef->chan->center_freq);
	WL_ERR(("%s: target_channel(%d), target bandwidth(%d)\n",
		__FUNCTION__, CHSPEC_CHANNEL(chspec), chandef->width));

	if (
#ifdef WL_6G_BAND
		CHSPEC_IS6G(chspec) ||
#endif /* WL_6G_BAND */
		CHSPEC_IS5G(chspec)) {
		switch (chandef->width) {
			case NL80211_CHAN_WIDTH_20:
				bw = WL_CHANSPEC_BW_20;
				break;
			case NL80211_CHAN_WIDTH_40:
				bw = WL_CHANSPEC_BW_40;
				break;
			case NL80211_CHAN_WIDTH_80:
				bw = WL_CHANSPEC_BW_80;
				break;
			case NL80211_CHAN_WIDTH_160:
				bw = WL_CHANSPEC_BW_160;
				break;
			default:
				err = wl_get_bandwidth_cap(ndev, CHSPEC_BAND(chspec), &bw);
				if (err < 0) {
					WL_ERR(("Failed to get bandwidth information, err=%d\n",
						err));
					return err;
				}

				/* In case of 5G downgrade BW to 80MHz
				 * as 160MHz channels falls in DFS
				 */
				if (CHSPEC_IS5G(chspec) && (bw == WL_CHANSPEC_BW_160)) {
					bw = WL_CHANSPEC_BW_80;
				}
				break;
		}
	} else if (CHSPEC_IS2G(chspec)) {
		bw = WL_CHANSPEC_BW_20;
	} else {
		WL_ERR(("invalid band (%d)\n", CHSPEC_BAND(chspec)));
		return -EINVAL;
	}

	WL_DBG(("%s: chandef->width(%u), bw(%u)\n",
		__FUNCTION__, chandef->width, bw));
	chspec = wf_create_chspec_from_primary(wf_chspec_primary20_chan(chspec),
		bw, CHSPEC_BAND(chspec));

	if (!wf_chspec_valid(chspec)) {
		WL_ERR(("Invalid chanspec 0x%x\n", chspec));
		return -EINVAL;
	}

	/* Set chanspec for monitor channel */
	err = wldev_iovar_setint(ndev, "chanspec", chspec);
	if (unlikely(err)) {
		WL_ERR(("%s: Could not set chanspec %d\n", __FUNCTION__, err));
	}

	return err;
}
#endif /* WL_CFG80211_MONITOR */

int wl_chspec_to_chandef(chanspec_t chanspec,
	struct cfg80211_chan_def *chandef, struct wiphy *wiphy)
{
	uint16 freq = 0;
	struct ieee80211_channel *chan;

	if (!chandef) {
		return -1;
	} else {
		memset(chandef, 0, sizeof(*chandef));
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION (3, 8, 0))
	chandef->center_freq1 =  wl_channel_to_frequency(CHSPEC_CHANNEL(chanspec), CHSPEC_BAND(chanspec));
	freq = wl_channel_to_frequency(wf_chspec_primary20_chan(chanspec), CHSPEC_BAND(chanspec));
	chandef->chan = ieee80211_get_channel(wiphy, freq);
	chandef->center_freq2 = 0;

	switch (CHSPEC_BW(chanspec)) {
		case WL_CHANSPEC_BW_20:
			chandef->width = NL80211_CHAN_WIDTH_20;
			break;

		case WL_CHANSPEC_BW_40:
			chandef->width = NL80211_CHAN_WIDTH_40;
			break;

		case WL_CHANSPEC_BW_80:
			chandef->width = NL80211_CHAN_WIDTH_80;
			break;

		case WL_CHANSPEC_BW_8080:
		{
			/* XXX Left as is but need proper calculation for center_freq2 is used */
			int chan_type = 0;
			int channel = 0;
			uint16 sb = CHSPEC_CTL_SB(chanspec);

			if (sb == WL_CHANSPEC_CTL_SB_LL) {
				channel -= (CH_10MHZ_APART + CH_20MHZ_APART);
			} else if (sb == WL_CHANSPEC_CTL_SB_LU) {
				channel -= CH_10MHZ_APART;
			} else if (sb == WL_CHANSPEC_CTL_SB_UL) {
				channel += CH_10MHZ_APART;
			} else {
				/* WL_CHANSPEC_CTL_SB_UU */
				channel += (CH_10MHZ_APART + CH_20MHZ_APART);
			}

			if (sb == WL_CHANSPEC_CTL_SB_LL || sb == WL_CHANSPEC_CTL_SB_LU)
				chan_type = NL80211_CHAN_HT40MINUS;
			else if (sb == WL_CHANSPEC_CTL_SB_UL || sb == WL_CHANSPEC_CTL_SB_UU)
				chan_type = NL80211_CHAN_HT40PLUS;
			freq = wl_channel_to_frequency(channel, CHSPEC_BAND(chanspec));
			chan = ieee80211_get_channel(wiphy, freq);
			cfg80211_chandef_create(chandef, chan, chan_type);
			return 0;
			break;
		}

		case WL_CHANSPEC_BW_160:
			chandef->width = NL80211_CHAN_WIDTH_160;
			break;
		default:
			chandef->width = NL80211_CHAN_WIDTH_20;
			break;
	}

#elif (LINUX_VERSION_CODE >= KERNEL_VERSION (3, 5, 0))

	int chan_type = 0;
	int channel = 0;
	channel = CHSPEC_CHANNEL(chanspec);
	switch (CHSPEC_BW(chanspec)) {
		case WL_CHANSPEC_BW_20:
			case WL_CHANSPEC_BW_20:
			chan_type = NL80211_CHAN_HT20;
			break;
		case WL_CHANSPEC_BW_40:
			/* 'chan_type' is marking 2nd channel */
			if (CHSPEC_SB_UPPER(chanspec)) {
				channel += CH_10MHZ_APART;
				chan_type = NL80211_CHAN_HT40MINUS;
			} else {
				channel -= CH_10MHZ_APART;
				chan_type = NL80211_CHAN_HT40PLUS;
			}
			break;
		default:
			chan_type = NL80211_CHAN_HT20;
			break;
	}

	freq = wl_channel_to_frequency(channel, CHSPEC_BAND(chanspec));
	chan = ieee80211_get_channel(wiphy, freq);
	WL_DBG(("channel:%d freq:%d chan_type: %d chan_ptr:%p \n",
		channel, freq, chan_type, chan));
	if (unlikely(!chan)) {
		/* fw and cfg80211 channel lists are not in sync */
		WL_ERR(("Couldn't find matching channel in wiphy channel list \n"));
		ASSERT(0);
		return -EINVAL;
	}

	chandef->freq = freq;
	chandef->chan_type = chan_type;
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION (3, 8, 0)) */

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
void
wl_cfg80211_ch_switch_notify(struct net_device *dev, uint16 chanspec, struct wiphy *wiphy)
{
	u32 freq;
	struct cfg80211_chan_def chandef;

	if (!wiphy) {
		WL_ERR(("wiphy is null\n"));
		return;
	}
#if (LINUX_VERSION_CODE <= KERNEL_VERSION (3, 18, 0))
	/* Channel switch support is only for AP/GO/ADHOC/MESH */
	if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_STATION ||
		dev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_CLIENT) {
		WL_ERR(("No channel switch notify support for STA/GC\n"));
		return;
	}
#endif /* (LINUX_VERSION_CODE <= KERNEL_VERSION (3, 18, 0)) */

	if (wl_chspec_to_chandef(chanspec, &chandef, wiphy)) {
		WL_ERR(("chspec_chandef failed\n"));
		return;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION (3, 8, 0))
	freq = chandef.chan ? chandef.chan->center_freq : chandef.center_freq1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0) || (defined(CUSTOMER_HW4) && \
	LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	 cfg80211_ch_switch_notify(dev, &chandef, 0, 0);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0) || defined(CFG80211_BKPORT_MLO)
	cfg80211_ch_switch_notify(dev, &chandef, 0);
#else
	cfg80211_ch_switch_notify(dev, &chandef);
#endif /* KERNEL_VERSION(5, 19, 0) || defined(CFG80211_BKPORT_MLO) */
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION (3, 5, 0))
	freq = chandef.freq;
	cfg80211_ch_switch_notify(dev, freq, chandef.chan_type);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION (3, 8, 0)) */

	WL_ERR(("Channel switch notification for freq: %d chanspec: 0x%x\n", freq, chanspec));
	return;
}
#endif /* LINUX_VERSION_CODE >= (3, 5, 0) */

static void
wl_ap_channel_ind(struct bcm_cfg80211 *cfg,
	struct net_device *ndev,
	chanspec_t chanspec)
{
	u32 channel = LCHSPEC_CHANNEL(chanspec);

	WL_INFORM_MEM(("(%s) AP channel:%d chspec:0x%x \n",
		ndev->name, channel, chanspec));

#ifdef SUPPORT_AP_BWCTRL
	wl_update_apchan_bwcap(cfg, ndev, chanspec);
#endif /* SUPPORT_AP_BWCTRL */

	if (!(cfg->ap_oper_channel == INVCHANSPEC) && (cfg->ap_oper_channel != chanspec)) {
		/*
		 * If cached channel is different from the channel indicated
		 * by the event, notify user space about the channel switch.
		 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
		wl_cfg80211_ch_switch_notify(ndev, chanspec, bcmcfg_to_wiphy(cfg));
#endif /* LINUX_VERSION_CODE >= (3, 5, 0) */
		cfg->ap_oper_channel = chanspec;
	}
}

s32
wl_ap_start_ind(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
const wl_event_msg_t *e, void *data)
{
	struct net_device *ndev = NULL;
	chanspec_t chanspec;

	WL_DBG(("Enter\n"));
	if (unlikely(e->status)) {
		WL_ERR(("status:0x%x \n", e->status));
		return -1;
	}

	if (!data) {
		return -EINVAL;
	}

	if (likely(cfgdev)) {
		ndev = cfgdev_to_wlc_ndev(cfgdev, cfg);
		chanspec = *((chanspec_t *)data);

		if (wl_get_mode_by_netdev(cfg, ndev) == WL_MODE_AP) {
			/* For AP/GO role */
			wl_ap_channel_ind(cfg, ndev, chanspec);
		}
	}

	return 0;
}

s32
wl_csa_complete_ind(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
const wl_event_msg_t *e, void *data)
{
	int error = 0;
	u32 chanspec = 0;
	struct net_device *ndev = NULL;
	struct ether_addr bssid;
#ifdef DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P
	dhd_pub_t *dhdp = (dhd_pub_t *)(cfg->pub);
#endif /* DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P */
	WL_DBG(("Enter\n"));
	if (unlikely(e->status)) {
		WL_ERR(("status:0x%x \n", e->status));
		return -1;
	}

	if (likely(cfgdev)) {
		ndev = cfgdev_to_wlc_ndev(cfgdev, cfg);
		/* Get association state if not AP and then query chanspec */
		if (!((wl_get_mode_by_netdev(cfg, ndev)) == WL_MODE_AP)) {
			error = wldev_ioctl_get(ndev, WLC_GET_BSSID, &bssid, ETHER_ADDR_LEN);
			if (error) {
				WL_ERR(("CSA on %s. Not associated. error=%d\n",
					ndev->name, error));
				return BCME_ERROR;
			}
		}

		error = wldev_iovar_getint(ndev, "chanspec", &chanspec);
		if (unlikely(error)) {
			WL_ERR(("Get chanspec error: %d \n", error));
			return -1;
		}

		WL_INFORM_MEM(("[%s] CSA ind. ch:0x%x\n", ndev->name, chanspec));
		if (wl_get_mode_by_netdev(cfg, ndev) == WL_MODE_AP) {
			/* For AP/GO role */
			wl_ap_channel_ind(cfg, ndev, chanspec);
#ifdef DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P
		/* If latest chanspec after csa operation is 2g20 disable FB else enable FB */
		if (dhdp && dhdp->disable_2g20_fb) {
			/* Set Frameburst to true if not 2G20 else set it to false */
			wl_cfg80211_set_frameburst(cfg, !wl_cfg80211_check_is2G20(ndev));
		}
#endif /* DISABLE_WL_FRAMEBURST_2G20_SOFTAP_P2P */
		} else {
			/* STA/GC roles */
			if (!wl_get_drv_status(cfg, CONNECTED, ndev)) {
				WL_ERR(("CSA on %s. Not associated.\n", ndev->name));
				return BCME_ERROR;
			}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
			wl_cfg80211_ch_switch_notify(ndev, chanspec, bcmcfg_to_wiphy(cfg));
#endif /* LINUX_VERSION_CODE >= (3, 5, 0) */
		}

	}

	return 0;
}

#ifdef WLTDLS
s32
wl_cfg80211_tdls_config(struct bcm_cfg80211 *cfg, enum wl_tdls_config state, bool auto_mode)
{
	struct net_device *ndev = bcmcfg_to_prmry_ndev(cfg);
	int err = 0;
	struct net_info *iter, *next;
	int update_reqd = 0;
	int enable = 0;
	dhd_pub_t *dhdp;
	dhdp = (dhd_pub_t *)(cfg->pub);

	/*
	 * TDLS need to be enabled only if we have a single STA/GC
	 * connection.
	 */

	WL_DBG(("Enter state:%d\n", state));
	if (!cfg->tdls_supported) {
		/* FW doesn't support tdls. Do nothing */
		return -ENODEV;
	}

	/* Protect tdls config session */
	mutex_lock(&cfg->tdls_sync);

	if (state == TDLS_STATE_TEARDOWN) {
		/* Host initiated TDLS tear down */
		err = dhd_tdls_enable(ndev, false, auto_mode, NULL);
		goto exit;
	} else if ((state == TDLS_STATE_AP_CREATE) ||
		(state == TDLS_STATE_NMI_CREATE)) {
		/* We don't support tdls while AP/GO/NAN is operational */
		update_reqd = true;
		enable = false;
	} else if ((state == TDLS_STATE_CONNECT) || (state == TDLS_STATE_IF_CREATE)) {
		if (wl_get_drv_status_all(cfg,
			CONNECTED) >= TDLS_MAX_IFACE_FOR_ENABLE) {
			/* For STA/GC connect command request, disable
			 * tdls if we have any concurrent interfaces
			 * operational.
			 */
			WL_DBG(("Interface limit restriction. disable tdls.\n"));
			update_reqd = true;
			enable = false;
		}
	} else if ((state == TDLS_STATE_DISCONNECT) ||
		(state == TDLS_STATE_AP_DELETE) ||
		(state == TDLS_STATE_SETUP) ||
		(state == TDLS_STATE_IF_DELETE)) {
		/* Enable back the tdls connection only if we have less than
		 * or equal to a single STA/GC connection.
		 */
		if (wl_get_drv_status_all(cfg,
			CONNECTED) == 0) {
			/* If there are no interfaces connected, enable tdls */
			update_reqd = true;
			enable = true;
		} else if (wl_get_drv_status_all(cfg,
			CONNECTED) == TDLS_MAX_IFACE_FOR_ENABLE) {
			/* We have one interface in CONNECTED state.
			 * Verify whether its a STA interface before
			 * we enable back tdls.
			 */
			GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
			for_each_ndev(cfg, iter, next) {
				GCC_DIAGNOSTIC_POP();
				if ((iter->ndev) && (wl_get_drv_status(cfg, CONNECTED, ndev)) &&
					(ndev->ieee80211_ptr->iftype != NL80211_IFTYPE_STATION)) {
					WL_DBG(("Non STA iface operational. cfg_iftype:%d"
						" Can't enable tdls.\n",
						ndev->ieee80211_ptr->iftype));
					err = -ENOTSUPP;
					goto exit;
				}
			}
			/* No AP/GO found. Enable back tdls */
			update_reqd = true;
			enable = true;
		} else {
			WL_DBG(("Concurrent connection mode. Can't enable tdls. \n"));
			err = -ENOTSUPP;
			goto exit;
		}
	} else {
		WL_ERR(("Unknown tdls state:%d \n", state));
		err = -EINVAL;
		goto exit;
	}

	if (update_reqd == true) {
		if (dhdp->tdls_enable == enable) {
			WL_DBG(("No change in tdls state. Do nothing."
				" tdls_enable:%d\n", enable));
			goto exit;
		}
		err = wldev_iovar_setint(ndev, "tdls_enable", enable);
		if (unlikely(err)) {
			WL_ERR(("tdls_enable setting failed. err:%d\n", err));
			goto exit;
		} else {
			WL_INFORM_MEM(("tdls_enable %d state:%d\n", enable, state));
			/* Update the dhd state variable to be in sync */
			dhdp->tdls_enable = enable;
			if (state == TDLS_STATE_SETUP) {
				/* For host initiated setup, apply TDLS params
				 * Don't propagate errors up for param config
				 * failures
				 */
				dhd_tdls_enable(ndev, true, auto_mode, NULL);

			}
		}
	} else {
		WL_DBG(("Skip tdls config. state:%d update_reqd:%d "
			"current_status:%d \n",
			state, update_reqd, dhdp->tdls_enable));
	}

exit:
	if (err) {
		wl_flush_fw_log_buffer(ndev, FW_LOGSET_MASK_ALL);
	}
	mutex_unlock(&cfg->tdls_sync);
	return err;
}
#endif /* WLTDLS */

struct net_device* wl_get_ap_netdev(struct bcm_cfg80211 *cfg, char *ifname)
{
	struct net_info *iter, *next;
	struct net_device *ndev = NULL;

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	for_each_ndev(cfg, iter, next) {
		GCC_DIAGNOSTIC_POP();
		if (iter->ndev) {
			if (strncmp(iter->ndev->name, ifname, IFNAMSIZ) == 0) {
				if (iter->ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {
					ndev = iter->ndev;
					break;
				}
			}
		}
	}

	return ndev;
}

#ifdef SUPPORT_AP_HIGHER_BEACONRATE
#define WLC_RATE_FLAG	0x80
#define RATE_MASK		0x7f

int wl_set_ap_beacon_rate(struct net_device *dev, int val, char *ifname)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhdp;
	wl_rateset_args_t rs;
	int error = BCME_ERROR, i;
	struct net_device *ndev = NULL;

	dhdp = (dhd_pub_t *)(cfg->pub);

	if (dhdp && !(dhdp->op_mode & DHD_FLAG_HOSTAP_MODE)) {
		WL_ERR(("Not Hostapd mode\n"));
		return BCME_NOTAP;
	}

	ndev = wl_get_ap_netdev(cfg, ifname);

	if (ndev == NULL) {
		WL_ERR(("No softAP interface named %s\n", ifname));
		return BCME_NOTAP;
	}

	bzero(&rs, sizeof(wl_rateset_args_t));
	error = wldev_iovar_getbuf(ndev, "rateset", NULL, 0,
		&rs, sizeof(wl_rateset_args_t), NULL);
	if (error < 0) {
		WL_ERR(("get rateset failed = %d\n", error));
		return error;
	}

	if (rs.count < 1) {
		WL_ERR(("Failed to get rate count\n"));
		return BCME_ERROR;
	}

	/* Host delivers target rate in the unit of 500kbps */
	/* To make it to 1mbps unit, atof should be implemented for 5.5mbps basic rate */
	for (i = 0; i < rs.count && i < WL_NUMRATES; i++)
		if (rs.rates[i] & WLC_RATE_FLAG)
			if ((rs.rates[i] & RATE_MASK) == val)
				break;

	/* Valid rate has been delivered as an argument */
	if (i < rs.count && i < WL_NUMRATES) {
		error = wldev_iovar_setint(ndev, "force_bcn_rspec", val);
		if (error < 0) {
			WL_ERR(("set beacon rate failed = %d\n", error));
			return BCME_ERROR;
		}
	} else {
		WL_ERR(("Rate is invalid"));
		return BCME_BADARG;
	}

	return BCME_OK;
}

int
wl_get_ap_basic_rate(struct net_device *dev, char* command, char *ifname, int total_len)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhdp;
	wl_rateset_args_t rs;
	int error = BCME_ERROR;
	int i, bytes_written = 0;
	struct net_device *ndev = NULL;

	dhdp = (dhd_pub_t *)(cfg->pub);

	if (!(dhdp->op_mode & DHD_FLAG_HOSTAP_MODE)) {
		WL_ERR(("Not Hostapd mode\n"));
		return BCME_NOTAP;
	}

	ndev = wl_get_ap_netdev(cfg, ifname);

	if (ndev == NULL) {
		WL_ERR(("No softAP interface named %s\n", ifname));
		return BCME_NOTAP;
	}

	bzero(&rs, sizeof(wl_rateset_args_t));
	error = wldev_iovar_getbuf(ndev, "rateset", NULL, 0,
		&rs, sizeof(wl_rateset_args_t), NULL);
	if (error < 0) {
		WL_ERR(("get rateset failed = %d\n", error));
		return error;
	}

	if (rs.count < 1) {
		WL_ERR(("Failed to get rate count\n"));
		return BCME_ERROR;
	}

	/* Delivers basic rate in the unit of 500kbps to host */
	for (i = 0; i < rs.count && i < WL_NUMRATES; i++)
		if (rs.rates[i] & WLC_RATE_FLAG)
			bytes_written += snprintf(command + bytes_written, total_len,
							"%d ", rs.rates[i] & RATE_MASK);

	/* Remove last space in the command buffer */
	if (bytes_written && (bytes_written < total_len)) {
		command[bytes_written - 1] = '\0';
		bytes_written--;
	}

	return bytes_written;

}
#endif /* SUPPORT_AP_HIGHER_BEACONRATE */

#ifdef SUPPORT_AP_RADIO_PWRSAVE
#define MSEC_PER_MIN	(60000L)

static int
_wl_update_ap_rps_params(struct net_device *dev)
{
	struct bcm_cfg80211 *cfg = NULL;
	rpsnoa_iovar_params_t iovar;
	u8 smbuf[WLC_IOCTL_SMLEN];

	if (!dev)
		return BCME_BADARG;

	cfg = wl_get_cfg(dev);

	bzero(&iovar, sizeof(iovar));
	bzero(smbuf, sizeof(smbuf));

	iovar.hdr.ver = RADIO_PWRSAVE_VERSION;
	iovar.hdr.subcmd = WL_RPSNOA_CMD_PARAMS;
	iovar.hdr.len = sizeof(iovar);
	iovar.param->band = WLC_BAND_ALL;
	iovar.param->level = cfg->ap_rps_info.level;
	iovar.param->stas_assoc_check = cfg->ap_rps_info.sta_assoc_check;
	iovar.param->pps = cfg->ap_rps_info.pps;
	iovar.param->quiet_time = cfg->ap_rps_info.quiet_time;

	if (wldev_iovar_setbuf(dev, "rpsnoa", &iovar, sizeof(iovar),
		smbuf, sizeof(smbuf), NULL)) {
		WL_ERR(("Failed to set rpsnoa params"));
		return BCME_ERROR;
	}

	return BCME_OK;
}

int
wl_get_ap_rps(struct net_device *dev, char* command, char *ifname, int total_len)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhdp;
	int error = BCME_ERROR;
	int bytes_written = 0;
	struct net_device *ndev = NULL;
	rpsnoa_iovar_status_t iovar;
	u8 smbuf[WLC_IOCTL_SMLEN];
	u32 chanspec = 0;
	u8 idx = 0;
	u16 state;
	u32 sleep;
	u32 time_since_enable;

	dhdp = (dhd_pub_t *)(cfg->pub);

	if (!dhdp) {
		error = BCME_NOTUP;
		goto fail;
	}

	if (!(dhdp->op_mode & DHD_FLAG_HOSTAP_MODE)) {
		WL_ERR(("Not Hostapd mode\n"));
		error = BCME_NOTAP;
		goto fail;
	}

	ndev = wl_get_ap_netdev(cfg, ifname);

	if (ndev == NULL) {
		WL_ERR(("No softAP interface named %s\n", ifname));
		error = BCME_NOTAP;
		goto fail;
	}

	bzero(&iovar, sizeof(iovar));
	bzero(smbuf, sizeof(smbuf));

	iovar.hdr.ver = RADIO_PWRSAVE_VERSION;
	iovar.hdr.subcmd = WL_RPSNOA_CMD_STATUS;
	iovar.hdr.len = sizeof(iovar);
	iovar.stats->band = WLC_BAND_ALL;

	error = wldev_iovar_getbuf(ndev, "rpsnoa", &iovar, sizeof(iovar),
		smbuf, sizeof(smbuf), NULL);
	if (error < 0) {
		WL_ERR(("get ap radio pwrsave failed = %d\n", error));
		goto fail;
	}

	/* RSDB event doesn't seem to be handled correctly.
	 * So check chanspec of AP directly from the firmware
	 */
	error = wldev_iovar_getint(ndev, "chanspec", (s32 *)&chanspec);
	if (error < 0) {
		WL_ERR(("get chanspec from AP failed = %d\n", error));
		goto fail;
	}

	chanspec = wl_chspec_driver_to_host(chanspec);
	if (CHSPEC_IS2G(chanspec))
		idx = 0;
	else if (
#ifdef WL_6G_BAND
		CHSPEC_IS6G(chanspec) ||
#endif /* WL_6G_BAND */
		CHSPEC_IS5G(chanspec))
		idx = 1;
	else {
		error = BCME_BADCHAN;
		goto fail;
	}

	state = ((rpsnoa_iovar_status_t *)smbuf)->stats[idx].state;
	sleep = ((rpsnoa_iovar_status_t *)smbuf)->stats[idx].sleep_dur;
	time_since_enable = ((rpsnoa_iovar_status_t *)smbuf)->stats[idx].sleep_avail_dur;

	/* Conver ms to minute, round down only */
	sleep = DIV_U64_BY_U32(sleep, MSEC_PER_MIN);
	time_since_enable = DIV_U64_BY_U32(time_since_enable, MSEC_PER_MIN);

	bytes_written += snprintf(command + bytes_written, total_len,
		"state=%d sleep=%d time_since_enable=%d", state, sleep, time_since_enable);
	error = bytes_written;

fail:
	return error;
}

int
wl_set_ap_rps(struct net_device *dev, bool enable, char *ifname)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhdp;
	struct net_device *ndev = NULL;
	rpsnoa_iovar_t iovar;
	u8 smbuf[WLC_IOCTL_SMLEN];
	int ret = BCME_OK;

	dhdp = (dhd_pub_t *)(cfg->pub);

	if (!dhdp) {
		ret = BCME_NOTUP;
		goto exit;
	}

	if (!(dhdp->op_mode & DHD_FLAG_HOSTAP_MODE)) {
		WL_ERR(("Not Hostapd mode\n"));
		ret = BCME_NOTAP;
		goto exit;
	}

	ndev = wl_get_ap_netdev(cfg, ifname);

	if (ndev == NULL) {
		WL_ERR(("No softAP interface named %s\n", ifname));
		ret = BCME_NOTAP;
		goto exit;
	}

	if (cfg->ap_rps_info.enable != enable) {
		cfg->ap_rps_info.enable = enable;
		if (enable) {
			ret = _wl_update_ap_rps_params(ndev);
			if (ret) {
				WL_ERR(("Filed to update rpsnoa params\n"));
				goto exit;
			}
		}
		bzero(&iovar, sizeof(iovar));
		bzero(smbuf, sizeof(smbuf));

		iovar.hdr.ver = RADIO_PWRSAVE_VERSION;
		iovar.hdr.subcmd = WL_RPSNOA_CMD_ENABLE;
		iovar.hdr.len = sizeof(iovar);
		iovar.data->band = WLC_BAND_ALL;
		iovar.data->value = (int16)enable;

		ret = wldev_iovar_setbuf(ndev, "rpsnoa", &iovar, sizeof(iovar),
			smbuf, sizeof(smbuf), NULL);
		if (ret) {
			WL_ERR(("Failed to enable AP radio power save"));
			goto exit;
		}
		cfg->ap_rps_info.enable = enable;
	}
exit:
	return ret;
}

int
wl_update_ap_rps_params(struct net_device *dev, ap_rps_info_t* rps, char *ifname)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhdp;
	struct net_device *ndev = NULL;

	dhdp = (dhd_pub_t *)(cfg->pub);

	if (!dhdp)
		return BCME_NOTUP;

	if (!(dhdp->op_mode & DHD_FLAG_HOSTAP_MODE)) {
		WL_ERR(("Not Hostapd mode\n"));
		return BCME_NOTAP;
	}

	ndev = wl_get_ap_netdev(cfg, ifname);

	if (ndev == NULL) {
		WL_ERR(("No softAP interface named %s\n", ifname));
		return BCME_NOTAP;
	}

	if (!rps)
		return BCME_BADARG;

	if (rps->pps < RADIO_PWRSAVE_PPS_MIN)
		return BCME_BADARG;

	if (rps->level < RADIO_PWRSAVE_LEVEL_MIN ||
		rps->level > RADIO_PWRSAVE_LEVEL_MAX)
		return BCME_BADARG;

	if (rps->quiet_time < RADIO_PWRSAVE_QUIETTIME_MIN)
		return BCME_BADARG;

	if (rps->sta_assoc_check > RADIO_PWRSAVE_ASSOCCHECK_MAX ||
		rps->sta_assoc_check < RADIO_PWRSAVE_ASSOCCHECK_MIN)
		return BCME_BADARG;

	cfg->ap_rps_info.pps = rps->pps;
	cfg->ap_rps_info.level = rps->level;
	cfg->ap_rps_info.quiet_time = rps->quiet_time;
	cfg->ap_rps_info.sta_assoc_check = rps->sta_assoc_check;

	if (cfg->ap_rps_info.enable) {
		if (_wl_update_ap_rps_params(ndev)) {
			WL_ERR(("Failed to update rpsnoa params"));
			return BCME_ERROR;
		}
	}

	return BCME_OK;
}

void
wl_cfg80211_init_ap_rps(struct bcm_cfg80211 *cfg)
{
	cfg->ap_rps_info.enable = FALSE;
	cfg->ap_rps_info.sta_assoc_check = RADIO_PWRSAVE_STAS_ASSOC_CHECK;
	cfg->ap_rps_info.pps = RADIO_PWRSAVE_PPS;
	cfg->ap_rps_info.quiet_time = RADIO_PWRSAVE_QUIET_TIME;
	cfg->ap_rps_info.level = RADIO_PWRSAVE_LEVEL;
}
#endif /* SUPPORT_AP_RADIO_PWRSAVE */

int
wl_cfg80211_iface_count(struct net_device *dev)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	struct net_info *iter, *next;
	int iface_count = 0;

	/* Return the count of network interfaces (skip netless p2p discovery
	 * interface)
	 */
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	for_each_ndev(cfg, iter, next) {
		GCC_DIAGNOSTIC_POP();
		if (iter->ndev) {
			iface_count++;
		}
	}
	return iface_count;
}

typedef struct {
	uint16 id;
	uint16 len;
	uint32 val;
} he_xtlv_v32;

static bool
wl_he_get_uint_cb(void *ctx, uint16 *id, uint16 *len)
{
	he_xtlv_v32 *v32 = ctx;

	*id = v32->id;
	*len = v32->len;

	return FALSE;
}

	static void
wl_he_pack_uint_cb(void *ctx, uint16 id, uint16 len, uint8 *buf)
{
	he_xtlv_v32 *v32 = ctx;

	BCM_REFERENCE(id);
	BCM_REFERENCE(len);

	v32->val = htod32(v32->val);

	switch (v32->len) {
		case sizeof(uint8):
			*buf = (uint8)v32->val;
			break;
		case sizeof(uint16):
			store16_ua(buf, (uint16)v32->val);
			break;
		case sizeof(uint32):
			store32_ua(buf, v32->val);
			break;
		default:
			/* ASSERT(0); */
			break;
	}
}

int wl_cfg80211_change_he_features(struct net_device *dev, struct bcm_cfg80211 *cfg,
		s32 bssidx, u32 he_flag, bool set)
{
	bcm_xtlv_t read_he_xtlv;
	uint8 se_he_xtlv[32];
	int se_he_xtlv_len = sizeof(se_he_xtlv);
	he_xtlv_v32 v32;
	u32 he_feature = 0;
	s32 err = 0;

	read_he_xtlv.id = WL_HE_CMD_FEATURES;
	read_he_xtlv.len = 0;
	err = wldev_iovar_getbuf_bsscfg(dev, "he", &read_he_xtlv, sizeof(read_he_xtlv),
			cfg->ioctl_buf, WLC_IOCTL_SMLEN, bssidx, NULL);
	if (err < 0) {
		WL_ERR(("HE get failed. error=%d\n", err));
		return err;
	} else {
		he_feature =  *(int*)cfg->ioctl_buf;
		he_feature = dtoh32(he_feature);
	}

	v32.id = WL_HE_CMD_FEATURES;
	v32.len = sizeof(s32);

	if (set) {
		v32.val = (he_feature | he_flag);
	} else {
		v32.val = (he_feature & ~he_flag);
	}

	err = bcm_pack_xtlv_buf((void *)&v32, se_he_xtlv, sizeof(se_he_xtlv),
			BCM_XTLV_OPTION_ALIGN32, wl_he_get_uint_cb, wl_he_pack_uint_cb,
			&se_he_xtlv_len);
	if (err != BCME_OK) {
		WL_ERR(("failed to pack he settvl=%d\n", err));
	}

	err = wldev_iovar_setbuf_bsscfg(dev, "he", &se_he_xtlv, sizeof(se_he_xtlv),
			cfg->ioctl_buf, WLC_IOCTL_SMLEN, bssidx, &cfg->ioctl_buf_sync);
	if (err < 0) {
		WL_ERR(("failed to set he features, error=%d\n", err));
	}
	WL_INFORM(("Set HE[%d] done\n", set));

	return err;
}

int wl_cfg80211_set_he_features(struct net_device *dev, struct bcm_cfg80211 *cfg,
		s32 bssidx, u32 interface_type, uint features)
{
	uint8 se_he_xtlv[32];
	int se_he_xtlv_len = sizeof(se_he_xtlv);
	he_xtlv_v32 v32;
	s32 err = 0;

	v32.id = WL_HE_CMD_FEATURES;
	v32.len = sizeof(s32);
	v32.val = features;

	err = bcm_pack_xtlv_buf((void *)&v32, se_he_xtlv, sizeof(se_he_xtlv),
			BCM_XTLV_OPTION_ALIGN32, wl_he_get_uint_cb, wl_he_pack_uint_cb,
			&se_he_xtlv_len);
	if (err != BCME_OK) {
		WL_ERR(("failed to pack he settvl=%d\n", err));
	}

	err = wldev_iovar_setbuf_bsscfg(dev, "he", &se_he_xtlv, sizeof(se_he_xtlv),
			cfg->ioctl_buf, WLC_IOCTL_SMLEN, bssidx, &cfg->ioctl_buf_sync);
	if (err < 0) {
		WL_ERR(("failed to set he features, error=%d\n", err));
	}
	WL_INFORM(("Set he features[%d] done\n", features));

	return err;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0))
int
wl_cfg80211_channel_switch(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_csa_settings *params)
{
	s32 err = BCME_OK;
	u32 bw = WL_CHANSPEC_BW_20;
	chanspec_t chspec = 0;
	wl_chan_switch_t csa_arg;
	struct cfg80211_chan_def *chandef = &params->chandef;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);
	struct net_device *primary_dev = bcmcfg_to_prmry_ndev(cfg);

	dev = ndev_to_wlc_ndev(dev, cfg);
	chspec = wl_freq_to_chanspec(chandef->chan->center_freq);

	WL_ERR(("netdev_ifidx(%d), target channel(%d) target bandwidth(%d),"
		" mode(%d), count(%d)\n", dev->ifindex, CHSPEC_CHANNEL(chspec), chandef->width,
		params->block_tx, params->count));

	if (wl_get_mode_by_netdev(cfg, dev) != WL_MODE_AP) {
		WL_ERR(("Channel Switch doesn't support on "
			"the non-SoftAP mode\n"));
		return -EINVAL;
	}

	/* Check if STA is trying to associate with an AP */
	if (wl_get_drv_status(cfg, CONNECTING, primary_dev)) {
		WL_ERR(("Connecting is in progress\n"));
		return BCME_BUSY;
	}

	if (chspec == cfg->ap_oper_channel) {
		WL_ERR(("Channel %d is same as current operating channel,"
			" so skip\n", CHSPEC_CHANNEL(chspec)));
		return BCME_OK;
	}

	if (
#ifdef WL_6G_BAND
		CHSPEC_IS6G(chspec) ||
#endif
		CHSPEC_IS5G(chspec)) {
#ifdef APSTA_RESTRICTED_CHANNEL
		if (CHSPEC_CHANNEL(chspec) != DEFAULT_5G_SOFTAP_CHANNEL) {
			WL_ERR(("Invalid 5G Channel, chan=%d\n", CHSPEC_CHANNEL(chspec)));
			return -EINVAL;
		}
#endif /* APSTA_RESTRICTED_CHANNEL */
		err = wl_get_bandwidth_cap(primary_dev, CHSPEC_BAND(chspec), &bw);
		if (err < 0) {
			WL_ERR(("Failed to get bandwidth information,"
				" err=%d\n", err));
			return err;
		}
	} else if (CHSPEC_IS2G(chspec)) {
#ifdef APSTA_RESTRICTED_CHANNEL
		dhd_pub_t *dhdp = (dhd_pub_t *)(cfg->pub);
		chanspec_t *sta_chanspec = (chanspec_t *)wl_read_prof(cfg,
			primary_dev, WL_PROF_CHAN);

		/* In 2GHz STA/SoftAP concurrent mode, the operating channel
		 * of STA and SoftAP should be confgiured to the same 2GHz
		 * channel. Otherwise, it is an invalid configuration.
		 */
		if (DHD_OPMODE_STA_SOFTAP_CONCURR(dhdp) &&
			wl_get_drv_status(cfg, CONNECTED, primary_dev) &&
			sta_chanspec && (CHSPEC_CHANNEL(*sta_chanspec) != CHSPEC_CHANNEL(chspec))) {
			WL_ERR(("Invalid 2G Channel in case of STA/SoftAP"
				" concurrent mode, sta_chan=%d, chan=%d\n",
				CHSPEC_CHANNEL(*sta_chanspec), CHSPEC_CHANNEL(chspec)));
			return -EINVAL;
		}
#endif /* APSTA_RESTRICTED_CHANNEL */
		bw = WL_CHANSPEC_BW_20;
	} else {
		WL_ERR(("invalid band (%d)\n", CHSPEC_BAND(chspec)));
		return -EINVAL;
	}

#ifdef WL_6G_BAND
	/* Avoid in case of 6G as for each center frequency bw is unique and is
	* detected based on centre frequency.
	*/
	if (!CHSPEC_IS6G(chspec))
#endif /* WL_6G_BAND */
	{
		chspec = wf_channel2chspec(CHSPEC_CHANNEL(chspec), bw);
	}
	if (!wf_chspec_valid(chspec)) {
		WL_ERR(("Invalid chanspec 0x%x\n", chspec));
		return -EINVAL;
	}

	/* Send CSA to associated STAs */
	memset(&csa_arg, 0, sizeof(wl_chan_switch_t));
	csa_arg.mode = params->block_tx;
	csa_arg.count = params->count;
	csa_arg.chspec = chspec;
	csa_arg.frame_type = CSA_BROADCAST_ACTION_FRAME;
	csa_arg.reg = 0;

	err = wldev_iovar_setbuf(dev, "csa", &csa_arg, sizeof(wl_chan_switch_t),
		cfg->ioctl_buf, WLC_IOCTL_SMLEN, &cfg->ioctl_buf_sync);
	if (err < 0) {
		WL_ERR(("Failed to switch channel, err=%d\n", err));
	}

	return err;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0) */

#ifdef SUPPORT_AP_SUSPEND
void
wl_set_ap_suspend_error_handler(struct net_device *ndev, bool suspend)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(ndev);
	dhd_pub_t *dhdp = (dhd_pub_t *)(cfg->pub);

	if (wl_get_drv_status(cfg, READY, ndev)) {
		/* IF dongle is down due to previous hang or other conditions, sending
		* one more hang notification is not needed.
		*/
		if (dhd_query_bus_erros(dhdp)) {
			return;
		}
		dhdp->iface_op_failed = TRUE;
#if defined(DHD_FW_COREDUMP)
		if (dhdp->memdump_enabled) {
			dhdp->memdump_type = DUMP_TYPE_IFACE_OP_FAILURE;
			dhd_bus_mem_dump(dhdp);
		}
#endif /* DHD_FW_COREDUMP */

#if defined(OEM_ANDROID)
		WL_ERR(("Notify hang event to upper layer \n"));
		dhdp->hang_reason = suspend ?
			HANG_REASON_BSS_DOWN_FAILURE : HANG_REASON_BSS_UP_FAILURE;
		net_os_send_hang_message(ndev);
#endif /* BCMDONGLEHOST && OEM_ANDROID */

	}
}

#define MAX_AP_RESUME_TIME   5000
int
wl_set_ap_suspend(struct net_device *dev, bool suspend, char *ifname)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhdp;
	struct net_device *ndev = NULL;
	int ret = BCME_OK;
	bool is_bssup = FALSE;
	int bssidx;
	unsigned long start_j;
	int time_to_sleep = MAX_AP_RESUME_TIME;

	dhdp = (dhd_pub_t *)(cfg->pub);

	if (!dhdp) {
		return BCME_NOTUP;
	}

	if (!(dhdp->op_mode & DHD_FLAG_HOSTAP_MODE)) {
		WL_ERR(("Not Hostapd mode\n"));
		return BCME_NOTAP;
	}

	ndev = wl_get_ap_netdev(cfg, ifname);

	if (ndev == NULL) {
		WL_ERR(("No softAP interface named %s\n", ifname));
		return BCME_NOTAP;
	}

	if ((bssidx = wl_get_bssidx_by_wdev(cfg, ndev->ieee80211_ptr)) < 0) {
		WL_ERR(("Find p2p index from wdev(%p) failed\n", ndev->ieee80211_ptr));
		return BCME_NOTFOUND;
	}

	is_bssup = wl_cfg80211_bss_isup(ndev, bssidx);
	if (is_bssup && suspend) {
		wl_clr_drv_status(cfg, AP_CREATED, ndev);
		wl_clr_drv_status(cfg, CONNECTED, ndev);

		if ((ret = wl_cfg80211_bss_up(cfg, ndev, bssidx, 0)) < 0) {
			WL_ERR(("AP suspend error %d, suspend %d\n", ret, suspend));
			ret = BCME_NOTDOWN;
			goto exit;
		}
	} else if (!is_bssup && !suspend) {
		/* Abort scan before starting AP again */
		wl_cfgscan_cancel_scan(cfg);

		if ((ret = wl_cfg80211_bss_up(cfg, ndev, bssidx, 1)) < 0) {
			WL_ERR(("AP resume error %d, suspend %d\n", ret, suspend));
			ret = BCME_NOTUP;
			goto exit;
		}

		while (TRUE) {
			start_j = get_jiffies_64();
			/* Wait for Linkup event to mark successful AP bring up */
			ret = wait_event_interruptible_timeout(cfg->netif_change_event,
				wl_get_drv_status(cfg, AP_CREATED, ndev),
				msecs_to_jiffies(time_to_sleep));
			if (ret == -ERESTARTSYS) {
				WL_ERR(("waitqueue was interrupted by a signal\n"));
				time_to_sleep -= jiffies_to_msecs(get_jiffies_64() - start_j);
				if (time_to_sleep <= 0) {
					WL_ERR(("time to sleep hits 0\n"));
					ret = BCME_NOTUP;
					goto exit;
				}
			} else if (ret == 0 || !wl_get_drv_status(cfg, AP_CREATED, ndev)) {
				WL_ERR(("AP resume failed!\n"));
				ret = BCME_NOTUP;
				goto exit;
			} else {
				wl_set_drv_status(cfg, CONNECTED, ndev);
				wl_clr_drv_status(cfg, AP_CREATING, ndev);
				ret = BCME_OK;
				break;
			}
		}
	} else {
		/* bssup + resume or bssdown + suspend,
		 * So, returns OK
		 */
		ret = BCME_OK;
	}
exit:
	if (ret != BCME_OK)
		wl_set_ap_suspend_error_handler(bcmcfg_to_prmry_ndev(cfg), suspend);

	return ret;
}
#endif /* SUPPORT_AP_SUSPEND */

#ifdef SUPPORT_SOFTAP_ELNA_BYPASS
int wl_set_softap_elna_bypass(struct net_device *dev, char *ifname, int enable)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	struct net_device *ifdev = NULL;
	char iobuf[WLC_IOCTL_SMLEN];
	int err = BCME_OK;
	int iftype = 0;

	memset(iobuf, 0, WLC_IOCTL_SMLEN);

	/* Check the interface type */
	ifdev = wl_get_netdev_by_name(cfg, ifname);
	if (ifdev == NULL) {
		WL_ERR(("%s: Could not find net_device for ifname:%s\n", __FUNCTION__, ifname));
		err = BCME_BADARG;
		goto fail;
	}

	iftype = ifdev->ieee80211_ptr->iftype;
	if (iftype == NL80211_IFTYPE_AP) {
		err = wldev_iovar_setint(ifdev, "softap_elnabypass", enable);
		if (unlikely(err)) {
			WL_ERR(("%s: Failed to set softap_elnabypass, err=%d\n",
				__FUNCTION__, err));
		}
	} else {
		WL_ERR(("%s: softap_elnabypass should control in SoftAP mode only\n",
			__FUNCTION__));
		err = BCME_BADARG;
	}
fail:
	return err;
}
int wl_get_softap_elna_bypass(struct net_device *dev, char *ifname, void *param)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	int *enable = (int*)param;
	struct net_device *ifdev = NULL;
	char iobuf[WLC_IOCTL_SMLEN];
	int err = BCME_OK;
	int iftype = 0;

	memset(iobuf, 0, WLC_IOCTL_SMLEN);

	/* Check the interface type */
	ifdev = wl_get_netdev_by_name(cfg, ifname);
	if (ifdev == NULL) {
		WL_ERR(("%s: Could not find net_device for ifname:%s\n", __FUNCTION__, ifname));
		err = BCME_BADARG;
		goto fail;
	}

	iftype = ifdev->ieee80211_ptr->iftype;
	if (iftype == NL80211_IFTYPE_AP) {
		err = wldev_iovar_getint(ifdev, "softap_elnabypass", enable);
		if (unlikely(err)) {
			WL_ERR(("%s: Failed to get softap_elnabypass, err=%d\n",
				__FUNCTION__, err));
		}
	} else {
		WL_ERR(("%s: softap_elnabypass should control in SoftAP mode only\n",
			__FUNCTION__));
		err = BCME_BADARG;
	}
fail:
	return err;

}
#endif /* SUPPORT_SOFTAP_ELNA_BYPASS */

#ifdef SUPPORT_AP_BWCTRL
#define OPER_MODE_ENABLE	(1 << 8)
static int op2bw[] = {20, 40, 80, 160};
static int bw2cap[] = { 0, 0, WLC_BW_CAP_20MHZ, WLC_BW_CAP_40MHZ,
	WLC_BW_CAP_80MHZ, WLC_BW_CAP_160MHZ, WLC_BW_CAP_160MHZ };

static int
wl_get_ap_he_mode(struct net_device *ndev, struct bcm_cfg80211 *cfg, bool *he)
{
	bcm_xtlv_t read_he_xtlv;
	int ret = 0;
	u8  he_enab = 0;
	u32 he_feature = 0;
	*he = FALSE;

	/* Check he enab first */
	read_he_xtlv.id = WL_HE_CMD_ENAB;
	read_he_xtlv.len = 0;

	ret = wldev_iovar_getbuf(ndev, "he", &read_he_xtlv, sizeof(read_he_xtlv),
			cfg->ioctl_buf, WLC_IOCTL_SMLEN, NULL);
	if (ret < 0) {
		if (ret == BCME_UNSUPPORTED) {
			/* HE not supported */
			ret = BCME_OK;
		} else {
			WL_ERR(("HE ENAB get failed. ret=%d\n", ret));
		}
		goto exit;
	} else {
		he_enab =  *(u8*)cfg->ioctl_buf;
	}

	if (!he_enab) {
		goto exit;
	}

	/* Then check BIT3 of he features */
	read_he_xtlv.id = WL_HE_CMD_FEATURES;
	read_he_xtlv.len = 0;

	ret = wldev_iovar_getbuf(ndev, "he", &read_he_xtlv, sizeof(read_he_xtlv),
			cfg->ioctl_buf, WLC_IOCTL_SMLEN, NULL);
	if (ret < 0) {
		WL_ERR(("HE FEATURE get failed. error=%d\n", ret));
		goto exit;
	} else {
		he_feature =  *(int*)cfg->ioctl_buf;
		he_feature = dtoh32(he_feature);
	}

	if (he_feature & WL_HE_FEATURES_HE_AP) {
		WL_DBG(("HE is enabled in AP\n"));
		*he = TRUE;
	}
exit:
	return ret;
}

static void
wl_update_apchan_bwcap(struct bcm_cfg80211 *cfg, struct net_device *ndev, chanspec_t chanspec)
{
	struct net_device *dev = bcmcfg_to_prmry_ndev(cfg);
	struct wireless_dev *wdev = ndev_to_wdev(dev);
	struct wiphy *wiphy = wdev->wiphy;
	int ret = BCME_OK;
	u32 bw_cap;
	u32 ctl_chan;
	chanspec_t chanbw = WL_CHANSPEC_BW_20;

	/* Update channel in profile */
	ctl_chan = wf_chspec_ctlchan(chanspec);
	wl_update_prof(cfg, ndev, NULL, &chanspec, WL_PROF_CHAN);

	/* BW cap is only updated in 5GHz */
	if (ctl_chan <= CH_MAX_2G_CHANNEL)
		return;

	/* Get WL BW CAP */
	ret = wl_get_bandwidth_cap(bcmcfg_to_prmry_ndev(cfg),
		CHSPEC_BAND(chanspec), &bw_cap);
	if (ret < 0) {
		WL_ERR(("get bw_cap failed = %d\n", ret));
		goto exit;
	}

	chanbw = CHSPEC_BW(wl_channel_to_chanspec(wiphy,
		ndev, chanspec, bw_cap, NULL));

exit:
	cfg->bw_cap_5g = bw2cap[chanbw >> WL_CHANSPEC_BW_SHIFT];
	WL_INFORM_MEM(("supported bw cap is:0x%x\n", cfg->bw_cap_5g));

}

int
wl_rxchain_to_opmode_nss(int rxchain)
{
	/*
	 * Nss 1 -> 0, Nss 2 -> 1
	 * This is from operating mode field
	 * in 8.4.1.50 of 802.11ac-2013
	 */
	/* TODO : Nss 3 ? */
	if (rxchain == 3)
		return (1 << 4);
	else
		return 0;
}

int
wl_update_opmode(struct net_device *ndev, u32 bw)
{
	int ret = BCME_OK;
	int oper_mode;
	int rxchain;

	ret = wldev_iovar_getint(ndev, "rxchain", (s32 *)&rxchain);
	if (ret < 0) {
		WL_ERR(("get rxchain failed = %d\n", ret));
		goto exit;
	}

	oper_mode = bw;
	oper_mode |= wl_rxchain_to_opmode_nss(rxchain);
	/* Enable flag */
	oper_mode |= OPER_MODE_ENABLE;

	ret = wldev_iovar_setint(ndev, "oper_mode", oper_mode);
	if (ret < 0) {
		WL_ERR(("set oper_mode failed = %d\n", ret));
		goto exit;
	}

exit:
	return ret;
}

int
wl_set_ap_bw(struct net_device *dev, u32 bw, char *ifname)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhdp;
	struct net_device *ndev = NULL;
	int ret = BCME_OK;
	chanspec_t *chanspec;
	bool he;

	dhdp = (dhd_pub_t *)(cfg->pub);

	if (!dhdp) {
		return BCME_NOTUP;
	}

	if (!(dhdp->op_mode & DHD_FLAG_HOSTAP_MODE)) {
		WL_ERR(("Not Hostapd mode\n"));
		return BCME_NOTAP;
	}

	ndev = wl_get_ap_netdev(cfg, ifname);

	if (ndev == NULL) {
		WL_ERR(("No softAP interface named %s\n", ifname));
		return BCME_NOTAP;
	}

	if (bw > DOT11_OPER_MODE_160MHZ) {
		WL_ERR(("BW is too big %d\n", bw));
		return BCME_BADARG;
	}

	chanspec = (chanspec_t *)wl_read_prof(cfg, ndev, WL_PROF_CHAN);
	if (CHSPEC_IS2G(*chanspec)) {
		WL_ERR(("current chanspec is %d, not supported\n", *chanspec));
		ret = BCME_BADCHAN;
		goto exit;
	}

	if ((DHD_OPMODE_STA_SOFTAP_CONCURR(dhdp) &&
		wl_get_drv_status(cfg, CONNECTED, bcmcfg_to_prmry_ndev(cfg))) ||
		wl_cfgnan_is_enabled(cfg)) {
		WL_ERR(("BW control in concurrent mode is not supported\n"));
		return BCME_BUSY;
	}

	/* When SCAN is on going either in STA or in AP, return BUSY */
	if (wl_get_drv_status_all(cfg, SCANNING)) {
		WL_ERR(("STA is SCANNING, not support BW control\n"));
		return BCME_BUSY;
	}

	/* When SCANABORT is on going either in STA or in AP, return BUSY */
	if (wl_get_drv_status_all(cfg, SCAN_ABORTING)) {
		WL_ERR(("STA is SCAN_ABORTING, not support BW control\n"));
		return BCME_BUSY;
	}

	/* When CONNECTION is on going in STA, return BUSY */
	if (wl_get_drv_status(cfg, CONNECTING, bcmcfg_to_prmry_ndev(cfg))) {
		WL_ERR(("STA is CONNECTING, not support BW control\n"));
		return BCME_BUSY;
	}

	/* BW control in AX mode needs more verification */
	ret = wl_get_ap_he_mode(ndev, cfg, &he);
	if (ret == BCME_OK && he) {
		WL_ERR(("BW control in HE mode is not supported\n"));
		return BCME_UNSUPPORTED;
	}
	if (ret < 0) {
		WL_ERR(("Check AX mode is failed\n"));
		goto exit;
	}

	if ((!WL_BW_CAP_160MHZ(cfg->bw_cap_5g) && (bw == DOT11_OPER_MODE_160MHZ)) ||
		(!WL_BW_CAP_80MHZ(cfg->bw_cap_5g) && (bw >= DOT11_OPER_MODE_80MHZ)) ||
		(!WL_BW_CAP_40MHZ(cfg->bw_cap_5g) && (bw >= DOT11_OPER_MODE_40MHZ)) ||
		(!WL_BW_CAP_20MHZ(cfg->bw_cap_5g))) {
		WL_ERR(("bw_cap %x does not support bw = %d\n", cfg->bw_cap_5g, bw));
		ret = BCME_BADARG;
		goto exit;
	}

	WL_DBG(("Updating AP BW to %d\n", op2bw[bw]));

	ret = wl_update_opmode(ndev, bw);
	if (ret < 0) {
		WL_ERR(("opmode set failed = %d\n", ret));
		goto exit;
	}

exit:
	return ret;
}

int
wl_get_ap_bw(struct net_device *dev, char* command, char *ifname, int total_len)
{
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	dhd_pub_t *dhdp;
	struct net_device *ndev = NULL;
	int ret = BCME_OK;
	u32 chanspec = 0;
	u32 bw = DOT11_OPER_MODE_20MHZ;
	int bytes_written = 0;

	dhdp = (dhd_pub_t *)(cfg->pub);

	if (!dhdp) {
		return BCME_NOTUP;
	}

	if (!(dhdp->op_mode & DHD_FLAG_HOSTAP_MODE)) {
		WL_ERR(("Not Hostapd mode\n"));
		return BCME_NOTAP;
	}

	ndev = wl_get_ap_netdev(cfg, ifname);

	if (ndev == NULL) {
		WL_ERR(("No softAP interface named %s\n", ifname));
		return BCME_NOTAP;
	}

	ret = wldev_iovar_getint(ndev, "chanspec", (s32 *)&chanspec);
	if (ret < 0) {
		WL_ERR(("get chanspec from AP failed = %d\n", ret));
		goto exit;
	}

	chanspec = wl_chspec_driver_to_host(chanspec);

	if (CHSPEC_IS20(chanspec)) {
		bw = DOT11_OPER_MODE_20MHZ;
	} else if (CHSPEC_IS40(chanspec)) {
		bw = DOT11_OPER_MODE_40MHZ;
	} else if (CHSPEC_IS80(chanspec)) {
		bw = DOT11_OPER_MODE_80MHZ;
	} else if (CHSPEC_IS_BW_160_WIDE(chanspec)) {
		bw = DOT11_OPER_MODE_160MHZ;
	} else {
		WL_ERR(("chanspec error %x\n", chanspec));
		ret = BCME_BADCHAN;
		goto exit;
	}

	bytes_written += snprintf(command + bytes_written, total_len,
		"bw=%d", bw);
	ret = bytes_written;
exit:
	return ret;
}

void
wl_restore_ap_bw(struct bcm_cfg80211 *cfg)
{
	int ret = BCME_OK;
	u32 bw;
	bool he = FALSE;
	struct net_info *iter, *next;
	struct net_device *ndev = NULL;
	chanspec_t *chanspec;

	if (!cfg) {
		return;
	}

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	for_each_ndev(cfg, iter, next) {
		GCC_DIAGNOSTIC_POP();
		if (iter->ndev) {
			if (iter->ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {
				chanspec = (chanspec_t *)wl_read_prof(cfg, iter->ndev,
					WL_PROF_CHAN);
				if (CHSPEC_IS2G(*chanspec)) {
					ndev = iter->ndev;
					break;
				}
			}
		}
	}

	if (!ndev) {
		return;
	}

	/* BW control in AX mode not allowed */
	ret = wl_get_ap_he_mode(bcmcfg_to_prmry_ndev(cfg), cfg, &he);
	if (ret == BCME_OK && he) {
		return;
	}
	if (ret < 0) {
		WL_ERR(("Check AX mode is failed\n"));
		return;
	}

	if (WL_BW_CAP_160MHZ(cfg->bw_cap_5g)) {
		bw = DOT11_OPER_MODE_160MHZ;
	} else if (WL_BW_CAP_80MHZ(cfg->bw_cap_5g)) {
		bw = DOT11_OPER_MODE_80MHZ;
	} else if (WL_BW_CAP_40MHZ(cfg->bw_cap_5g)) {
		bw = DOT11_OPER_MODE_40MHZ;
	} else {
		return;
	}

	WL_DBG(("Restoring AP BW to %d\n", op2bw[bw]));

	ret = wl_update_opmode(ndev, bw);
	if (ret < 0) {
		WL_ERR(("bw restore failed = %d\n", ret));
		return;
	}
}
#endif /* SUPPORT_AP_BWCTRL */

uint32
wl_cfgvif_get_iftype_count(struct bcm_cfg80211 *cfg, wl_iftype_t iftype)
{
	struct net_info *iter, *next;
	u32 count = 0;

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	for_each_ndev(cfg, iter, next) {
		GCC_DIAGNOSTIC_POP();
		/* Check against wl_iftype_t type */
		if ((iter->wdev) && (iter->iftype == iftype)) {
			struct net_device *ndev = iter->wdev->netdev;

			switch (iftype) {
				case WL_IF_TYPE_STA:
				case WL_IF_TYPE_AP:
					if (ndev && wl_get_drv_status(cfg, CONNECTED, ndev)) {
						/* If interface is in connected state, mark it */
						count++;
					}
					break;
				/* P2P, NAN interfaces are dynamically created when required
				 * and removed after use. so presence of interface indicates
				 * that role is active.
				 */
				case WL_IF_TYPE_P2P_GO:
				case WL_IF_TYPE_P2P_GC:
				case WL_IF_TYPE_P2P_DISC:
				case WL_IF_TYPE_NAN:
				case WL_IF_TYPE_NAN_NMI:
				case WL_IF_TYPE_MONITOR:
				case WL_IF_TYPE_IBSS:
					count++;
					break;
				default:
					WL_DBG(("Ignore iftype:%d\n", iftype));
			}
		}
	}
	return count;
}

#ifdef WL_STATIC_IF
struct net_device *
wl_cfg80211_register_static_if(struct bcm_cfg80211 *cfg, u16 iftype, char *ifname)
{
	struct net_device *ndev;
	struct wireless_dev *wdev = NULL;
	int ifidx = WL_STATIC_IFIDX; /* Register ndev with a reserved ifidx */
	u8 mac_addr[ETH_ALEN];
	struct net_device *primary_ndev;
#ifdef DHD_USE_RANDMAC
	struct ether_addr ea_addr;
#endif /* DHD_USE_RANDMAC */

	WL_INFORM_MEM(("[STATIC_IF] Enter (%s) iftype:%d\n", ifname, iftype));

	if (!cfg) {
		WL_ERR(("cfg null\n"));
		return NULL;
	}
	primary_ndev = bcmcfg_to_prmry_ndev(cfg);
	UNUSED_PARAMETER(primary_ndev);

#ifdef DHD_USE_RANDMAC
	wl_cfg80211_generate_mac_addr(&ea_addr);
	(void)memcpy_s(mac_addr, ETH_ALEN, ea_addr.octet, ETH_ALEN);
#else
	/* Use primary mac with locally admin bit set */
	DEV_ADDR_GET(primary_ndev, mac_addr);
	mac_addr[0] |= 0x02;
#endif /* DHD_USE_RANDMAC */

	ndev = wl_cfg80211_allocate_if(cfg, ifidx, ifname, mac_addr,
		WL_BSSIDX_MAX, NULL);
	if (unlikely(!ndev)) {
		WL_ERR(("Failed to allocate static_if\n"));
		goto fail;
	}
	wdev = (struct wireless_dev *)MALLOCZ(cfg->osh, sizeof(*wdev));
	if (unlikely(!wdev)) {
		WL_ERR(("Failed to allocate wdev for static_if\n"));
		goto fail;
	}

	wdev->wiphy = cfg->wdev->wiphy;
	wdev->iftype = iftype;

	ndev->ieee80211_ptr = wdev;
	SET_NETDEV_DEV(ndev, wiphy_dev(wdev->wiphy));
	wdev->netdev = ndev;

	if (wl_cfg80211_register_if(cfg, ifidx,
		ndev, TRUE) != BCME_OK) {
		WL_ERR(("ndev registration failed!\n"));
		goto fail;
	}

	cfg->static_ndev = ndev;
	cfg->static_ndev_state = NDEV_STATE_OS_IF_CREATED;
	wl_cfg80211_update_iflist_info(cfg, ndev, ifidx, NULL, WL_BSSIDX_MAX,
		ifname, NDEV_STATE_OS_IF_CREATED);
	WL_INFORM_MEM(("Static I/F (%s) Registered\n", ndev->name));
	return ndev;

fail:
	wl_cfg80211_remove_if(cfg, ifidx, ndev, false);
	return NULL;
}

void
wl_cfg80211_unregister_static_if(struct bcm_cfg80211 *cfg)
{
	WL_INFORM_MEM(("[STATIC_IF] Enter\n"));
	if (!cfg || !cfg->static_ndev) {
		WL_ERR(("invalid input\n"));
		return;
	}

	/* wdev free will happen from notifier context */
	/* free_netdev(cfg->static_ndev);
	*/
	dhd_unregister_net(cfg->static_ndev, true);
}

s32
wl_cfg80211_static_if_open(struct net_device *net)
{
	struct wireless_dev *wdev = NULL;
	struct bcm_cfg80211 *cfg = wl_get_cfg(net);
	struct net_device *primary_ndev = bcmcfg_to_prmry_ndev(cfg);
	u16 iftype = net->ieee80211_ptr ? net->ieee80211_ptr->iftype : 0;
	u16 wl_iftype, wl_mode;

	WL_INFORM_MEM(("[STATIC_IF] dev_open ndev %p and wdev %p\n", net, net->ieee80211_ptr));
	ASSERT(cfg->static_ndev == net);

	if (cfg80211_to_wl_iftype(iftype, &wl_iftype, &wl_mode) <  0) {
		return BCME_ERROR;
	}
	if (cfg->static_ndev_state != NDEV_STATE_FW_IF_CREATED) {
#if defined(DHD_USE_RANDMAC) || defined(WL_SOFTAP_RAND)
		wdev = wl_cfg80211_add_if(cfg, primary_ndev, wl_iftype, net->name, net->dev_addr);
#else
		wdev = wl_cfg80211_add_if(cfg, primary_ndev, wl_iftype, net->name, NULL);
#endif /* DHD_USE_RANDMAC || WL_SOFTAP_RAND */
		if (!wdev) {
			WL_ERR(("[STATIC_IF] wdev is NULL, can't proceed"));
			return BCME_ERROR;
		}
	} else {
		WL_INFORM_MEM(("Fw IF for static netdev already created\n"));
	}

	return BCME_OK;
}

s32
wl_cfg80211_static_if_close(struct net_device *net)
{
	int ret = BCME_OK;
	struct bcm_cfg80211 *cfg = wl_get_cfg(net);
	struct net_device *primary_ndev = bcmcfg_to_prmry_ndev(cfg);

	/* clear flags */
	wl_clr_drv_status(cfg, AP_ROLE_UPGRADED, net);

	if (cfg->static_ndev_state == NDEV_STATE_FW_IF_CREATED) {
		if (mutex_is_locked(&cfg->if_sync) == TRUE) {
			ret = _wl_cfg80211_del_if(cfg, primary_ndev, net->ieee80211_ptr, net->name);
		} else {
			ret = wl_cfg80211_del_if(cfg, primary_ndev, net->ieee80211_ptr, net->name);
		}

		if (unlikely(ret)) {
			WL_ERR(("Del iface failed for static_if %d\n", ret));
			/* on critical errors the cfg80211_del API would trigger hang event
			 * for WiFi to reset. However, the driver need to return okay to allow the
			 * kernel to clear the IFF_UP flags.
			 */
			ret = BCME_OK;
		}
	}

	return ret;
}
struct net_device *
wl_cfg80211_post_static_ifcreate(struct bcm_cfg80211 *cfg,
	wl_if_event_info *event, u8 *addr, s32 iface_type)
{
	struct net_device *new_ndev = NULL;
	struct wireless_dev *wdev = NULL;

	WL_INFORM_MEM(("Updating static iface after Fw IF create \n"));
	new_ndev = cfg->static_ndev;

	if (new_ndev) {
		wdev = new_ndev->ieee80211_ptr;
		ASSERT(wdev);
		wdev->iftype = iface_type;
		DEV_ADDR_SET(new_ndev, addr);
	}

	cfg->static_ndev_state = NDEV_STATE_FW_IF_CREATED;
	wl_cfg80211_update_iflist_info(cfg, new_ndev, event->ifidx, addr, event->bssidx,
		event->name, NDEV_STATE_FW_IF_CREATED);
	return new_ndev;
}
s32
wl_cfg80211_post_static_ifdel(struct bcm_cfg80211 *cfg, struct net_device *ndev)
{
	cfg->static_ndev_state = NDEV_STATE_FW_IF_DELETED;
	wl_cfg80211_update_iflist_info(cfg, ndev, WL_STATIC_IFIDX, NULL,
		WL_BSSIDX_MAX, NULL, NDEV_STATE_FW_IF_DELETED);
	wl_cfg80211_clear_per_bss_ies(cfg, ndev->ieee80211_ptr);
	wl_dealloc_netinfo_by_wdev(cfg, ndev->ieee80211_ptr);
	return BCME_OK;
}
#endif /* WL_STATIC_IF */
