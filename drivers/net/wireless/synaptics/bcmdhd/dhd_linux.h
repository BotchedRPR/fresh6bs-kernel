/*
 * DHD Linux header file (dhd_linux exports for cfg80211 and other components)
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

/* wifi platform functions for power, interrupt and pre-alloc, either
 * from Android-like platform device data, or Broadcom wifi platform
 * device data.
 *
 */
#ifndef __DHD_LINUX_H__
#define __DHD_LINUX_H__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <dngl_stats.h>
#include <dhd.h>
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(DHD_USE_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif /* defined(CONFIG_HAS_EARLYSUSPEND) && defined(DHD_USE_EARLYSUSPEND) */
#if defined(CONFIG_WIFI_CONTROL_FUNC)
#include <linux/wlan_plat.h>
#else
#include <dhd_plat.h>
#endif /* CONFIG_WIFI_CONTROL_FUNC */

#ifdef BCMPCIE
#include <bcmmsgbuf.h>
#endif /* BCMPCIE */

#ifdef PCIE_FULL_DONGLE
#include <etd.h>
#endif /* PCIE_FULL_DONGLE */

#ifdef WL_MONITOR
#define MAX_RADIOTAP_SIZE      256 /* Maximum size to hold HE Radiotap header format */
#define MAX_MON_PKT_SIZE       (4096 + MAX_RADIOTAP_SIZE)
#endif /* WL_MONITOR */

#ifdef DHD_COREDUMP
#define PC_FOUND_BIT 0x01
#define LR_FOUND_BIT 0x02
#define ALL_ADDR_VAL (PC_FOUND_BIT | LR_FOUND_BIT)
#define READ_NUM_BYTES 1000
#define DHD_FUNC_STR_LEN 80

#define DHD_COREDUMP_MAGIC 0xDDCEDACF
#define TLV_TYPE_LENGTH_SIZE	(8u)
/* coredump is composed as following TLV format.
 * Type(32bit) | Length(32bit) | Value(x bit)
 * e.g) socram type | length | socram dump
 *      sssr core1 type | length | sssr core1 dump
 *      ...
 */
enum coredump_types {
	DHD_COREDUMP_TYPE_SSSRDUMP_CORE0_BEFORE = 0,
	DHD_COREDUMP_TYPE_SSSRDUMP_CORE0_AFTER,
	DHD_COREDUMP_TYPE_SSSRDUMP_CORE1_BEFORE,
	DHD_COREDUMP_TYPE_SSSRDUMP_CORE1_AFTER,
	DHD_COREDUMP_TYPE_SSSRDUMP_CORE2_BEFORE,
	DHD_COREDUMP_TYPE_SSSRDUMP_CORE2_AFTER,
	DHD_COREDUMP_TYPE_SSSRDUMP_DIG_BEFORE,
	DHD_COREDUMP_TYPE_SSSRDUMP_DIG_AFTER,
	DHD_COREDUMP_TYPE_SOCRAMDUMP
};

#ifdef DHD_SSSR_DUMP
typedef struct dhd_coredump {
	uint32 type;
	uint32 length;
	void *bufptr;
} dhd_coredump_t;
#endif /* DHD_SSSR_DUMP */
#endif /* DHD_COREDUMP */

#ifdef DHD_WAKE_RX_STATUS
#define ETHER_ICMP6_HEADER	20
#define ETHER_IPV6_SADDR (ETHER_ICMP6_HEADER + 2)
#define ETHER_IPV6_DAADR (ETHER_IPV6_SADDR + IPV6_ADDR_LEN)
#define ETHER_ICMPV6_TYPE (ETHER_IPV6_DAADR + IPV6_ADDR_LEN)
#endif /* DHD_WAKE_RX_STATUS */
#define DHD_REGISTRATION_TIMEOUT  12000  /* msec : allowed time to finished dhd registration */

typedef struct wifi_adapter_info {
	const char	*name;
	uint		irq_num;
	uint		intr_flags;
	const char	*fw_path;
	const char	*nv_path;
	const char	*clm_path;
	const char	*sig_path;
	const char	*boot_path;
	const char 	*otp_path;
	uint32		otp_addr;
	void		*wifi_plat_data;	/* wifi ctrl func, for backward compatibility */
	uint		bus_type;
	uint		bus_num;
	uint		slot_num;
#if defined(BT_OVER_SDIO)
	const char	*btfw_path;
#endif /* defined (BT_OVER_SDIO) */
} wifi_adapter_info_t;

typedef struct bcmdhd_wifi_platdata {
	uint				num_adapters;
	wifi_adapter_info_t	*adapters;
} bcmdhd_wifi_platdata_t;

/** Per STA params. A list of dhd_sta objects are managed in dhd_if */
typedef struct dhd_sta {
	cumm_ctr_t cumm_ctr;    /* cummulative queue length of child flowrings */
	uint16 flowid[NUMPRIO]; /* allocated flow ring ids (by priority) */
	void * ifp;             /* associated dhd_if */
	struct ether_addr ea;   /* stations ethernet mac address */
	struct list_head list;  /* link into dhd_if::sta_list */
	int idx;                /* index of self in dhd_pub::sta_pool[] */
	int ifidx;              /* index of interface in dhd */
} dhd_sta_t;
typedef dhd_sta_t dhd_sta_pool_t;

#ifdef DHD_4WAYM4_FAIL_DISCONNECT
typedef enum {
	NONE_4WAY,
	M1_4WAY,
	M2_4WAY,
	M3_4WAY,
	M4_4WAY
} msg_4way_t;
typedef enum {
	M3_RXED,
	M4_TXFAILED
} msg_4way_state_t;
#define MAX_4WAY_TIMEOUT_MS 2000
#endif /* DHD_4WAYM4_FAIL_DISCONNECT */

#if defined(DHD_LB)
/* Dynamic CPU selection for load balancing. */
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <asm/atomic.h>

/* FIXME: Make this a module param or a sysfs. */
#if !defined(DHD_LB_PRIMARY_CPUS)
#define DHD_LB_PRIMARY_CPUS     0x0 /* Big CPU coreids mask */
#endif
#if !defined(DHD_LB_SECONDARY_CPUS)
#define DHD_LB_SECONDARY_CPUS   0xFE /* Little CPU coreids mask */
#endif

#define HIST_BIN_SIZE	9

#if defined(DHD_LB_TXP)
/* Pkttag not compatible with PROP_TXSTATUS or WLFC */
typedef struct dhd_tx_lb_pkttag_fr {
	struct net_device *net;
	int ifidx;
} dhd_tx_lb_pkttag_fr_t;

#define DHD_LB_TX_PKTTAG_SET_NETDEV(tag, netdevp)	((tag)->net = netdevp)
#define DHD_LB_TX_PKTTAG_NETDEV(tag)			((tag)->net)

#define DHD_LB_TX_PKTTAG_SET_IFIDX(tag, ifidx)	((tag)->ifidx = ifidx)
#define DHD_LB_TX_PKTTAG_IFIDX(tag)		((tag)->ifidx)
#endif /* DHD_LB_TXP */
#endif /* DHD_LB */

#define FILE_DUMP_MAX_WAIT_TIME 4000

#define htod32(i) (i)
#define htod16(i) (i)
#define dtoh32(i) (i)
#define dtoh16(i) (i)
#define htodchanspec(i) (i)
#define dtohchanspec(i) (i)

#ifdef BLOCK_IPV6_PACKET
#define HEX_PREF_STR	"0x"
#define UNI_FILTER_STR	"010000000000"
#define ZERO_ADDR_STR	"000000000000"
#define ETHER_TYPE_STR	"0000"
#define IPV6_FILTER_STR	"20"
#define ZERO_TYPE_STR	"00"
#endif /* BLOCK_IPV6_PACKET */

#if defined(OEM_ANDROID) && defined(SOFTAP)
extern bool ap_cfg_running;
extern bool ap_fw_loaded;
#endif

#if defined(OEM_ANDROID) && defined(BCMPCIE)
extern int dhd_get_suspend_bcn_li_dtim(dhd_pub_t *dhd, int *dtim_period, int *bcn_interval);
#else
extern int dhd_get_suspend_bcn_li_dtim(dhd_pub_t *dhd);
#endif /* OEM_ANDROID && BCMPCIE */

#ifdef CUSTOMER_HW4
#ifdef MIMO_ANT_SETTING
#ifdef DHD_EXPORT_CNTL_FILE
extern unsigned long antsel;
#endif /* DHD_EXPORT_CNTL_FILE */
extern int dhd_sel_ant_from_file(dhd_pub_t *dhd);
#endif /* MIMO_ANT_SETTING */
#ifdef WRITE_WLANINFO
#define MAX_VERSION_LEN		512
#ifdef DHD_EXPORT_CNTL_FILE
extern char version_info[MAX_VERSION_LEN];
#endif /* DHD_EXPORT_CNTL_FILE */
extern uint32 sec_save_wlinfo(char *firm_ver, char *dhd_ver, char *nvram_p, char *clm_ver);
#endif /* WRITE_WLANINFO */
#ifdef LOGTRACE_FROM_FILE
extern int dhd_logtrace_from_file(dhd_pub_t *dhd);
#ifdef DHD_EXPORT_CNTL_FILE
extern unsigned long logtrace_val;
#endif /* DHD_EXPORT_CNTL_FILE */
#endif /* LOGTRACE_FROM_FILE */
#ifdef GEN_SOFTAP_INFO_FILE
#define SOFTAP_INFO_BUF_SZ 512
#ifdef DHD_EXPORT_CNTL_FILE
extern char softapinfostr[SOFTAP_INFO_BUF_SZ];
#endif /* DHD_EXPORT_CNTL_FILE */
extern uint32 sec_save_softap_info(void);
#endif /* GEN_SOFTAP_INFO_FILE */
#endif /* CUSTOMER_HW4 */

#ifdef DHD_SEND_HANG_PRIVCMD_ERRORS
extern uint32 report_hang_privcmd_err;
#endif /* DHD_SEND_HANG_PRIVCMD_ERRORS */

#ifdef DHD_LOG_DUMP
/* 0: DLD_BUF_TYPE_GENERAL, 1: DLD_BUF_TYPE_PRESERVE
* 2: DLD_BUF_TYPE_SPECIAL
*/
#define DLD_BUFFER_NUM 3

#ifndef CUSTOM_LOG_DUMP_BUFSIZE_MB
#define CUSTOM_LOG_DUMP_BUFSIZE_MB	4 /* DHD_LOG_DUMP_BUF_SIZE 4 MB static memory in kernel */
#endif /* CUSTOM_LOG_DUMP_BUFSIZE_MB */

#define LOG_DUMP_TOTAL_BUFSIZE (1024 * 1024 * CUSTOM_LOG_DUMP_BUFSIZE_MB)

/*
 * Below are different sections that use the prealloced buffer
 * and sum of the sizes of these should not cross LOG_DUMP_TOTAL_BUFSIZE
 */
#ifdef EWP_BCM_TRACE
#define LOG_DUMP_GENERAL_MAX_BUFSIZE (192 * 1024 * CUSTOM_LOG_DUMP_BUFSIZE_MB)
#define LOG_DUMP_BCM_TRACE_MAX_BUFSIZE (64 * 1024 * CUSTOM_LOG_DUMP_BUFSIZE_MB)
#else
#define LOG_DUMP_GENERAL_MAX_BUFSIZE (256 * 1024 * CUSTOM_LOG_DUMP_BUFSIZE_MB)
#define LOG_DUMP_BCM_TRACE_MAX_BUFSIZE 0
#endif /* EWP_BCM_TRACE */
#define LOG_DUMP_PRESERVE_MAX_BUFSIZE (128 * 1024 * CUSTOM_LOG_DUMP_BUFSIZE_MB)
#define LOG_DUMP_ECNTRS_MAX_BUFSIZE (256 * 1024 * CUSTOM_LOG_DUMP_BUFSIZE_MB)
#define LOG_DUMP_RTT_MAX_BUFSIZE (256 * 1024 * CUSTOM_LOG_DUMP_BUFSIZE_MB)
#define LOG_DUMP_FILTER_MAX_BUFSIZE (128 * 1024 * CUSTOM_LOG_DUMP_BUFSIZE_MB)

#if LOG_DUMP_TOTAL_BUFSIZE < (LOG_DUMP_GENERAL_MAX_BUFSIZE + \
	LOG_DUMP_PRESERVE_MAX_BUFSIZE + LOG_DUMP_ECNTRS_MAX_BUFSIZE + LOG_DUMP_RTT_MAX_BUFSIZE \
	+ LOG_DUMP_BCM_TRACE_MAX_BUFSIZE + LOG_DUMP_FILTER_MAX_BUFSIZE)
#error "LOG_DUMP_TOTAL_BUFSIZE is lesser than sum of all rings"
#endif

/* Special buffer is allocated as separately in prealloc */
#define LOG_DUMP_SPECIAL_MAX_BUFSIZE (8 * 1024)

#define LOG_DUMP_MAX_FILESIZE (8 *1024 * 1024) /* 8 MB default */

#ifdef CONFIG_LOG_BUF_SHIFT
/* 15% of kernel log buf size, if for example klog buf size is 512KB
* 15% of 512KB ~= 80KB
*/
#define LOG_DUMP_KERNEL_TAIL_FLUSH_SIZE \
	(15 * ((1 << CONFIG_LOG_BUF_SHIFT)/100))
#endif /* CONFIG_LOG_BUF_SHIFT */

#define LOG_DUMP_COOKIE_BUFSIZE	1024u
typedef struct {
	char *hdr_str;
	log_dump_section_type_t sec_type;
} dld_hdr_t;

#define DHD_PRINT_BUF_NAME_LEN 30
void dhd_get_debug_dump_len(void *handle, struct sk_buff *skb, void *event_info, u8 event);
void cfgvendor_log_dump_len(dhd_pub_t *dhdp, log_dump_type_t *type, struct sk_buff *skb);
#endif /* DHD_LOG_DUMP */

typedef struct dhd_if_event {
	struct list_head	list;
	wl_event_data_if_t	event;
	char			name[IFNAMSIZ+1];
	uint8			mac[ETHER_ADDR_LEN];
} dhd_if_event_t;

/* Interface control information */
typedef struct dhd_if {
	struct dhd_info *info;			/* back pointer to dhd_info */
	/* OS/stack specifics */
	struct net_device *net;
	int				idx;			/* iface idx in dongle */
	uint			subunit;		/* subunit */
	uint8			mac_addr[ETHER_ADDR_LEN];	/* assigned MAC address */
	bool			set_macaddress;
	bool			set_multicast;
	uint8			bssidx;			/* bsscfg index for the interface */
	bool			attached;		/* Delayed attachment when unset */
	bool			txflowcontrol;	/* Per interface flow control indicator */
	char			name[IFNAMSIZ+1]; /* linux interface name */
	char			dngl_name[IFNAMSIZ+1]; /* corresponding dongle interface name */
	struct net_device_stats stats;

	// upper layer suppose different interface has different
	// 802.11 mode especially the multiply AP scenario and
	// bottom layer depends on its implementation
	int8            gmode;
	int8            nmode;
	int8            vhtmode;
	int8            hemode;
	int8            onlymode;

#ifdef PCIE_FULL_DONGLE
	struct list_head sta_list;		/* sll of associated stations */
	spinlock_t	sta_list_lock;		/* lock for manipulating sll */
#endif /* PCIE_FULL_DONGLE */
	uint32  ap_isolate;			/* ap-isolation settings */
#ifdef DHD_L2_FILTER
	bool parp_enable;
	bool parp_discard;
	bool parp_allnode;
	arp_table_t *phnd_arp_table;
	/* for Per BSS modification */
	bool dhcp_unicast;
	bool block_ping;
	bool grat_arp;
	bool block_tdls;
#endif /* DHD_L2_FILTER */
#ifdef DHD_MCAST_REGEN
	bool mcast_regen_bss_enable;
#endif
	bool rx_pkt_chainable;		/* set all rx packet to chainable config by default */
	cumm_ctr_t cumm_ctr;		/* cummulative queue length of child flowrings */
	uint8 tx_paths_active;
	bool del_in_progress;
	bool static_if;			/* used to avoid some operations on static_if */
#ifdef DHD_4WAYM4_FAIL_DISCONNECT
	struct delayed_work m4state_work;
	atomic_t m4state;
#endif /* DHD_4WAYM4_FAIL_DISCONNECT */
#ifdef DHDTCPSYNC_FLOOD_BLK
	uint32 tsync_rcvd;
	uint32 tsyncack_txed;
	u64 last_sync;
	struct work_struct  blk_tsfl_work;
	uint32 tsync_per_sec;
	bool disconnect_tsync_flood;
#endif /* DHDTCPSYNC_FLOOD_BLK */
#ifdef DHD_POST_EAPOL_M1_AFTER_ROAM_EVT
	bool recv_reassoc_evt;
	bool post_roam_evt;
#endif /* DHD_POST_EAPOL_M1_AFTER_ROAM_EVT */
} dhd_if_t;

struct ipv6_work_info_t {
	uint8			if_idx;
	char			ipv6_addr[IPV6_ADDR_LEN];
	unsigned long		event;
};

typedef struct dhd_dump {
	uint8 *buf;
	int bufsize;
	uint8 *hscb_buf;
	int hscb_bufsize;
} dhd_dump_t;
#ifdef DNGL_AXI_ERROR_LOGGING
typedef struct dhd_axi_error_dump {
	ulong fault_address;
	uint32 axid;
	struct hnd_ext_trap_axi_error_v1 etd_axi_error_v1;
} dhd_axi_error_dump_t;
#endif /* DNGL_AXI_ERROR_LOGGING */

#ifdef DHD_PCIE_NATIVE_RUNTIMEPM
struct dhd_rx_tx_work {
	struct work_struct work;
	struct sk_buff *skb;
	struct net_device *net;
	struct dhd_pub *pub;
};
#endif /* DHD_PCIE_NATIVE_RUNTIMEPM */

#ifdef FILTER_IE
#define FILTER_IE_PATH "/vendor/etc/wifi/filter_ie"
#define FILTER_IE_BUFSZ 1024 /* ioc buffsize for FILTER_IE */
#define FILE_BLOCK_READ_SIZE 256
#define WL_FILTER_IE_IOV_HDR_SIZE OFFSETOF(wl_filter_ie_iov_v1_t, tlvs)
#endif /* FILTER_IE */

#define NULL_CHECK(p, s, err)  \
			do { \
				if (!(p)) { \
					printk("NULL POINTER (%s) : %s\n", __FUNCTION__, (s)); \
					err = BCME_ERROR; \
					return err; \
				} \
			} while (0)

int dhd_wifi_platform_register_drv(void);
void dhd_wifi_platform_unregister_drv(void);
wifi_adapter_info_t* dhd_wifi_platform_get_adapter(uint32 bus_type, uint32 bus_num,
	uint32 slot_num);
int wifi_platform_set_power(wifi_adapter_info_t *adapter, bool on, unsigned long msec);
int wifi_platform_bus_enumerate(wifi_adapter_info_t *adapter, bool device_present);
int wifi_platform_get_irq_number(wifi_adapter_info_t *adapter, unsigned long *irq_flags_ptr);
extern int wifi_platform_get_irq_level(wifi_adapter_info_t *adapter);
int wifi_platform_get_mac_addr(wifi_adapter_info_t *adapter, unsigned char *buf);
#ifdef DHD_COREDUMP
int wifi_platform_set_coredump(wifi_adapter_info_t *adapter, const char *buf, int buf_len,
	const char *info);
#endif /* DHD_COREDUMP */
#ifdef CUSTOM_COUNTRY_CODE
void *wifi_platform_get_country_code(wifi_adapter_info_t *adapter, char *ccode,
	u32 flags);
#else
void *wifi_platform_get_country_code(wifi_adapter_info_t *adapter, char *ccode);
#endif /* CUSTOM_COUNTRY_CODE */
void* wifi_platform_prealloc(wifi_adapter_info_t *adapter, int section, unsigned long size);
void* wifi_platform_get_prealloc_func_ptr(wifi_adapter_info_t *adapter);

int dhd_get_fw_mode(struct dhd_info *dhdinfo);
bool dhd_update_fw_nv_path(struct dhd_info *dhdinfo);

#if defined(BT_OVER_SDIO)
int dhd_net_bus_get(struct net_device *dev);
int dhd_net_bus_put(struct net_device *dev);
#endif /* BT_OVER_SDIO */
#if defined(WLADPS) || defined(WLADPS_PRIVATE_CMD)
#define ADPS_ENABLE	1
#define ADPS_DISABLE	0

int dhd_enable_adps(dhd_pub_t *dhd, uint8 on);
#endif /* WLADPS || WLADPS_PRIVATE_CMD */
#ifdef DHDTCPSYNC_FLOOD_BLK
extern void dhd_reset_tcpsync_info_by_ifp(dhd_if_t *ifp);
extern void dhd_reset_tcpsync_info_by_dev(struct net_device *dev);
#endif /* DHDTCPSYNC_FLOOD_BLK */
#ifdef PCIE_FULL_DONGLE
extern void dhd_net_del_flowrings_sta(dhd_pub_t * dhd, struct net_device * ndev);
#endif /* PCIE_FULL_DONGLE */
int dhd_get_fw_capabilities(dhd_pub_t * dhd);
#ifdef WL_CFGVENDOR_SEND_ALERT_EVENT
void dhd_alert_process(struct work_struct *work_data);
#endif /* WL_CFGVENDOR_SEND_ALERT_EVENT */
#endif /* __DHD_LINUX_H__ */
