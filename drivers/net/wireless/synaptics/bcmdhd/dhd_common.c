/*
 * Broadcom Dongle Host Driver (DHD), common DHD core.
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
#include <typedefs.h>
#include <osl.h>

#include <epivers.h>
#include <bcmutils.h>
#include <bcmstdlib_s.h>

#include <bcmendian.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_ip.h>
#include <bcmevent.h>
#include <dhdioctl.h>
#include <wlioctl.h>
#include <bcmwifi_channels.h>
#ifdef DHD_SDTC_ETB_DUMP
#include <bcmiov.h>
#endif /* DHD_SDTC_ETB_DUMP */

#ifdef PCIE_FULL_DONGLE
#include <bcmmsgbuf.h>
#endif /* PCIE_FULL_DONGLE */

#ifdef SHOW_LOGTRACE
#include <event_log.h>
#endif /* SHOW_LOGTRACE */

#ifdef BCMPCIE
#include <dhd_flowring.h>
#endif

#include <dhd_bus.h>
#include <dhd_proto.h>
#include <dhd_dbg.h>
#include <802.1d.h>
#include <dhd_debug.h>
#include <dhd_dbg_ring.h>
#include <dhd_mschdbg.h>
#include <msgtrace.h>

#ifdef WL_CFG80211
#include <wl_cfg80211.h>
#include <wl_cfgvif.h>
#endif
#if defined(OEM_ANDROID) && defined(PNO_SUPPORT)
#include <dhd_pno.h>
#endif /* (OEM_ANDROID) && (PNO_SUPPORT) */
#ifdef RTT_SUPPORT
#include <dhd_rtt.h>
#endif

#ifdef DNGL_EVENT_SUPPORT
#include <dnglevent.h>
#endif

#define htod32(i) (i)
#define htod16(i) (i)
#define dtoh32(i) (i)
#define dtoh16(i) (i)
#define htodchanspec(i) (i)
#define dtohchanspec(i) (i)

#ifdef PROP_TXSTATUS
#include <wlfc_proto.h>
#include <dhd_wlfc.h>
#endif

#if defined(__linux__)
#include <dhd_linux.h>
#endif /* __linux__ */

#ifdef DHD_L2_FILTER
#include <dhd_l2_filter.h>
#endif /* DHD_L2_FILTER */

#ifdef DHD_PSTA
#include <dhd_psta.h>
#endif /* DHD_PSTA */

#ifdef DHD_WET
#include <dhd_wet.h>
#endif /* DHD_WET */

#ifdef DHD_LOG_DUMP
#include <dhd_dbg.h>
#ifdef DHD_PKT_LOGGING
#include <dhd_pktlog.h>
#endif
#endif /* DHD_LOG_DUMP */

#ifdef CSI_SUPPORT
#include <dhd_csi.h>
#endif /* CSI_SUPPORT */

#ifdef DHD_LOG_PRINT_RATE_LIMIT
int log_print_threshold = 0;
#endif /* DHD_LOG_PRINT_RATE_LIMIT */

#ifdef DHD_DEBUGABILITY_LOG_DUMP_RING
int dhd_msg_level = DHD_ERROR_VAL | DHD_EVENT_VAL
#if defined(BOARD_HIKEY) || defined(BOARD_VIM)
		| DHD_FWLOG_VAL
#endif /* BOARD_HIKEY || BOARD_VIM */
	        | DHD_PKT_MON_VAL;
int dhd_log_level = DHD_ERROR_VAL | DHD_EVENT_VAL
		| DHD_PKT_MON_VAL | DHD_FWLOG_VAL | DHD_IOVAR_MEM_VAL;
#else
int dbgring_msg_level = 0;
/* For CUSTOMER_HW4/Hikey do not enable DHD_ERROR_MEM_VAL by default */
int dhd_msg_level = DHD_ERROR_VAL | DHD_FWLOG_VAL | DHD_EVENT_VAL
	/* For CUSTOMER_HW4 do not enable DHD_IOVAR_MEM_VAL by default */
#if !defined(CUSTOMER_HW4) && !defined(BOARD_HIKEY)
	| DHD_IOVAR_MEM_VAL
#endif
#ifndef OEM_ANDROID
	| DHD_MSGTRACE_VAL
#endif /* OEM_ANDROID */
	| DHD_PKT_MON_VAL;

int dhd_log_level = DHD_ERROR_VAL | DHD_FWLOG_VAL | DHD_EVENT_VAL
	/* For CUSTOMER_HW4 do not enable DHD_IOVAR_MEM_VAL by default */
#if !defined(CUSTOMER_HW4) && !defined(BOARD_HIKEY)
	| DHD_IOVAR_MEM_VAL
#endif
#ifndef OEM_ANDROID
	| DHD_MSGTRACE_VAL
#endif /* OEM_ANDROID */
	| DHD_PKT_MON_VAL;

#endif /* DHD_DEBUGABILITY_LOG_DUMP_RING */

#ifdef DHD_DEBUG
#include <sdiovar.h>
#endif /* DHD_DEBUG */

#ifdef DHD_PCIE_NATIVE_RUNTIMEPM
#include <linux/pm_runtime.h>
#endif /* DHD_PCIE_NATIVE_RUNTIMEPM */

#if defined(DHD_LOGLEVEL) && !defined(DHD_DEBUG)
#error "Log level control is supported only with DHD_DEBUG"
#endif  /* defined(DHD_LOGLEVEL) && !defined(DHD_DEBUG) */

#ifdef SOFTAP
char fw_path2[MOD_PARAM_PATHLEN];
extern bool softap_enabled;
#endif

#ifdef SHOW_LOGTRACE
#define BYTES_AHEAD_NUM		10	/* address in map file is before these many bytes */
#define READ_NUM_BYTES		1000 /* read map file each time this No. of bytes */
#define GO_BACK_FILE_POS_NUM_BYTES	100 /* set file pos back to cur pos */
static char *ramstart_str = " text_start"; /* string in mapfile has addr ramstart */
static char *rodata_start_str = " rodata_start"; /* string in mapfile has addr rodata start */
static char *rodata_end_str = " rodata_end"; /* string in mapfile has addr rodata end */
#define RAMSTART_BIT	0x01
#define RDSTART_BIT		0x02
#define RDEND_BIT		0x04
#define ALL_MAP_VAL		(RAMSTART_BIT | RDSTART_BIT | RDEND_BIT)
#endif /* SHOW_LOGTRACE */

#ifdef SHOW_LOGTRACE
/* the fw file path is taken from either the module parameter at
 * insmod time or is defined as a constant of different values
 * for different platforms
 */
extern char *st_str_file_path;
#endif /* SHOW_LOGTRACE */

#ifdef EWP_EDL
#include "bcmmsgbuf.h"  /* add for compatibility */
typedef struct msg_hdr_edl {
	uint32 infobuf_ver;
	info_buf_payload_hdr_t pyld_hdr;
	msgtrace_hdr_t trace_hdr;
} msg_hdr_edl_t;
#endif /* EWP_EDL */

#define DHD_TPUT_MAX_TX_PKTS_BATCH	1000

/* Last connection success/failure status */
uint32 dhd_conn_event = 0xffffffff;
uint32 dhd_conn_status = 0xffffffff;
uint32 dhd_conn_reason = 0xffffffff;

extern int dhd_iscan_request(void * dhdp, uint16 action);
extern void dhd_ind_scan_confirm(void *h, bool status);
extern int dhd_iscan_in_progress(void *h);
void dhd_iscan_lock(void);
void dhd_iscan_unlock(void);
extern int dhd_change_mtu(dhd_pub_t *dhd, int new_mtu, int ifidx);
#if defined(OEM_ANDROID) && !defined(AP) && defined(WLP2P)
extern int dhd_get_concurrent_capabilites(dhd_pub_t *dhd);
#endif

extern int dhd_socram_dump(struct dhd_bus *bus);
extern void dhd_set_packet_filter(dhd_pub_t *dhd);

#ifdef DNGL_EVENT_SUPPORT
static void dngl_host_event_process(dhd_pub_t *dhdp, bcm_dngl_event_t *event,
	bcm_dngl_event_msg_t *dngl_event, size_t pktlen);
static int dngl_host_event(dhd_pub_t *dhdp, void *pktdata, bcm_dngl_event_msg_t *dngl_event,
	size_t pktlen);
#endif /* DNGL_EVENT_SUPPORT */

#ifdef WL_CFGVENDOR_SEND_HANG_EVENT
static void copy_hang_info_ioctl_timeout(dhd_pub_t *dhd, int ifidx, wl_ioctl_t *ioc);
#endif /* WL_CFGVENDOR_SEND_HANG_EVENT */

#ifdef DHD_SEND_HANG_IOCTL_SUSPEND_ERROR
#define MAX_IOCTL_SUSPEND_ERROR	10
static int ioctl_suspend_error = 0;
#endif /* DHD_SEND_HANG_IOCTL_SUSPEND_ERROR */

/* Should ideally read this from target(taken from wlu) */
#define MAX_CHUNK_LEN 1408 /* 8 * 8 * 22 */

#if defined(OEM_ANDROID)
/* note these variables will be used with wext */
bool ap_cfg_running = FALSE;
bool ap_fw_loaded = FALSE;
#endif /* defined(OEM_ANDROID) && defined(SOFTAP) */

#define CHIPID_MISMATCH	8

#if defined(PCIE_FULL_DONGLE)
#define DHD_BUS "PCIE"
#else
/* SDIO */
#if defined(RXFRAME_THREAD)
#define DHD_BUS "SDIO-RXF"
#elif defined(DHD_LB_RXP)
#define DHD_BUS "SDIO-NAPI"
#else
#define DHD_BUS "SDIO"
#endif
#endif

#define DHD_VERSION "\nDongle Host Driver, version " EPI_VERSION_STR " " EPI_COMMIT_ID " " DHD_BUS "\n"

#if defined(DHD_DEBUG) && defined(DHD_COMPILED)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdate-time"
const char dhd_version[] = DHD_VERSION DHD_COMPILED " compiled on "
			__DATE__ " at " __TIME__ "\n\0<TIMESTAMP>";
#pragma GCC diagnostic pop
#else
const char dhd_version[] = DHD_VERSION;
#endif /* DHD_DEBUG && DHD_COMPILED */

char fw_version[FW_VER_STR_LEN] = "\0";
char clm_version[CLM_VER_STR_LEN] = "\0";

char bus_api_revision[BUS_API_REV_STR_LEN] = "\0";

void dhd_set_timer(void *bus, uint wdtick);

static char* ioctl2str(uint32 ioctl);

#ifdef DHD_LOGLEVEL
void dhd_loglevel_set(dhd_loglevel_data_t *);
void dhd_loglevel_get(dhd_loglevel_data_t *);
#endif /* DHD_LOGLEVEL */

/* IOVar table */
enum {
	IOV_VERSION = 1,
	IOV_MSGLEVEL,
	IOV_BCMERRORSTR,
	IOV_BCMERROR,
	IOV_WDTICK,
	IOV_DUMP,
	IOV_CLEARCOUNTS,
	IOV_LOGDUMP,
	IOV_LOGCAL,
	IOV_LOGSTAMP,
	IOV_GPIOOB,
	IOV_IOCTLTIMEOUT,
	IOV_CONS,
	IOV_DCONSOLE_POLL,
#if defined(DHD_DEBUG)
	IOV_DHD_JOIN_TIMEOUT_DBG,
	IOV_SCAN_TIMEOUT,
	IOV_MEM_DEBUG,
#ifdef BCMPCIE
	IOV_FLOW_RING_DEBUG,
#endif /* BCMPCIE */
#endif /* defined(DHD_DEBUG) */
#ifdef PROP_TXSTATUS
	IOV_PROPTXSTATUS_ENABLE,
	IOV_PROPTXSTATUS_MODE,
	IOV_PROPTXSTATUS_OPT,
#ifdef QMONITOR
	IOV_QMON_TIME_THRES,
	IOV_QMON_TIME_PERCENT,
#endif /* QMONITOR */
	IOV_PROPTXSTATUS_MODULE_IGNORE,
	IOV_PROPTXSTATUS_CREDIT_IGNORE,
	IOV_PROPTXSTATUS_TXSTATUS_IGNORE,
	IOV_PROPTXSTATUS_RXPKT_CHK,
#endif /* PROP_TXSTATUS */
	IOV_BUS_TYPE,
	IOV_CHANGEMTU,
	IOV_HOSTREORDER_FLOWS,
#ifdef DHDTCPACK_SUPPRESS
	IOV_TCPACK_SUPPRESS,
#endif /* DHDTCPACK_SUPPRESS */
	IOV_AP_ISOLATE,
#ifdef DHD_L2_FILTER
	IOV_DHCP_UNICAST,
	IOV_BLOCK_PING,
	IOV_PROXY_ARP,
	IOV_GRAT_ARP,
	IOV_BLOCK_TDLS,
#endif /* DHD_L2_FILTER */
	IOV_DHD_IE,
#ifdef DHD_PSTA
	IOV_PSTA,
#endif /* DHD_PSTA */
#ifdef DHD_WET
	IOV_WET,
	IOV_WET_HOST_IPV4,
	IOV_WET_HOST_MAC,
#endif /* DHD_WET */
	IOV_CFG80211_OPMODE,
	IOV_ASSERT_TYPE,
	IOV_LMTEST,
#ifdef DHD_MCAST_REGEN
	IOV_MCAST_REGEN_BSS_ENABLE,
#endif
#ifdef SHOW_LOGTRACE
	IOV_DUMP_TRACE_LOG,
#endif /* SHOW_LOGTRACE */
	IOV_DONGLE_TRAP_TYPE,
	IOV_DONGLE_TRAP_INFO,
	IOV_BPADDR,
	IOV_DUMP_DONGLE, /**< dumps core registers and d11 memories */
#if defined(DHD_LOG_DUMP)
	IOV_LOG_DUMP,
#endif /* DHD_LOG_DUMP */
	IOV_TPUT_TEST,
	IOV_DEBUG_BUF_DEST_STAT,
#ifdef DHD_DEBUG
	IOV_INDUCE_ERROR,
#endif /* DHD_DEBUG */
	IOV_FIS_TRIGGER,
#ifdef RTT_GEOFENCE_CONT
#if defined(RTT_SUPPORT) && defined(WL_NAN)
	IOV_RTT_GEOFENCE_TYPE_OVRD,
#endif /* RTT_SUPPORT && WL_NAN */
#endif /* RTT_GEOFENCE_CONT */
	IOV_FW_VBS,
#ifdef DHD_TX_PROFILE
	IOV_TX_PROFILE_TAG,
	IOV_TX_PROFILE_ENABLE,
	IOV_TX_PROFILE_DUMP,
#endif /* defined(DHD_TX_PROFILE) */
	IOV_CHECK_TRAP_ROT,
#if defined(WL_MON_OWN_PKT)
	IOV_ROAM_EVENTS,
#endif
#ifdef DHD_LOGLEVEL
	IOV_LOGLEVEL,
#endif /* DHD_LOGLEVEL */
#ifdef BCMPCIE
	IOV_L1SS_INIT_ENAB,
	IOV_ASPM_INIT_ENAB,
#endif /* BCMPCIE */
#ifdef SYNA_SAR_CUSTOMER_PARAMETER
	IOV_SAR_MODE,
#endif /* SYNA_SAR_CUSTOMER_PARAMETER */
	IOV_LAST
};

const bcm_iovar_t dhd_iovars[] = {
	/* name         varid                   flags   flags2 type     minlen */
	{"version",	IOV_VERSION,		0,	0, IOVT_BUFFER,	0},
#if defined(DHD_DEBUG) || defined(DHD_MSG_LEVEL_SUPPORT)
	{"msglevel",	IOV_MSGLEVEL,		0,	0, IOVT_UINT32,	0},
#endif /* DHD_DEBUG || DHD_MSG_LEVEL_SUPPORT */
#ifdef DHD_DEBUG
	{"mem_debug",   IOV_MEM_DEBUG,  0,      0,      IOVT_BUFFER,    0 },
#ifdef BCMPCIE
	{"flow_ring_debug", IOV_FLOW_RING_DEBUG, 0, 0, IOVT_BUFFER, 0 },
#endif /* BCMPCIE */
#endif /* DHD_DEBUG */
	{"bcmerrorstr", IOV_BCMERRORSTR,	0,	0, IOVT_BUFFER,	BCME_STRLEN},
	{"bcmerror",	IOV_BCMERROR,		0,	0, IOVT_INT8,	0},
	{"wdtick",	IOV_WDTICK,		0,	0, IOVT_UINT32,	0},
	{"dump",	IOV_DUMP,		0,	0, IOVT_BUFFER,	DHD_IOCTL_MAXLEN_32K},
	{"cons",	IOV_CONS,		0,	0, IOVT_BUFFER,	0},
	{"dconpoll",	IOV_DCONSOLE_POLL,	0,	0, IOVT_UINT32,	0},
	{"clearcounts", IOV_CLEARCOUNTS,	0,	0, IOVT_VOID,	0},
	{"gpioob",	IOV_GPIOOB,		0,	0, IOVT_UINT32,	0},
	{"ioctl_timeout", IOV_IOCTLTIMEOUT,	0,	0, IOVT_UINT32,	0},
#ifdef PROP_TXSTATUS
	{"proptx",	IOV_PROPTXSTATUS_ENABLE,	0,	0, IOVT_BOOL,	0 },
	/*
	set the proptxtstatus operation mode:
	0 - Do not do any proptxtstatus flow control
	1 - Use implied credit from a packet status
	2 - Use explicit credit
	*/
	{"ptxmode",	IOV_PROPTXSTATUS_MODE,	0,	0, IOVT_UINT32,	0 },
	{"proptx_opt", IOV_PROPTXSTATUS_OPT,	0,	0, IOVT_UINT32,	0 },
#ifdef QMONITOR
	{"qtime_thres",	IOV_QMON_TIME_THRES,	0,	0, IOVT_UINT32,	0 },
	{"qtime_percent", IOV_QMON_TIME_PERCENT, 0,	0, IOVT_UINT32,	0 },
#endif /* QMONITOR */
	{"pmodule_ignore", IOV_PROPTXSTATUS_MODULE_IGNORE, 0, 0, IOVT_BOOL, 0 },
	{"pcredit_ignore", IOV_PROPTXSTATUS_CREDIT_IGNORE, 0, 0, IOVT_BOOL, 0 },
	{"ptxstatus_ignore", IOV_PROPTXSTATUS_TXSTATUS_IGNORE, 0, 0,  IOVT_BOOL, 0 },
	{"rxpkt_chk", IOV_PROPTXSTATUS_RXPKT_CHK, 0, 0, IOVT_BOOL, 0 },
#endif /* PROP_TXSTATUS */
	{"bustype", IOV_BUS_TYPE, 0, 0, IOVT_UINT32, 0},
	{"changemtu", IOV_CHANGEMTU, 0, 0, IOVT_UINT32, 0 },
	{"host_reorder_flows", IOV_HOSTREORDER_FLOWS, 0, 0, IOVT_BUFFER,
	(WLHOST_REORDERDATA_MAXFLOWS + 1) },
#ifdef DHDTCPACK_SUPPRESS
	{"tcpack_suppress",	IOV_TCPACK_SUPPRESS,	0,	0, IOVT_UINT8,	0 },
#endif /* DHDTCPACK_SUPPRESS */
#ifdef DHD_L2_FILTER
	{"dhcp_unicast", IOV_DHCP_UNICAST, (0), 0, IOVT_BOOL, 0 },
#endif /* DHD_L2_FILTER */
	{"ap_isolate", IOV_AP_ISOLATE, (0), 0, IOVT_BOOL, 0},
#ifdef DHD_L2_FILTER
	{"block_ping", IOV_BLOCK_PING, (0), 0, IOVT_BOOL, 0},
	{"proxy_arp", IOV_PROXY_ARP, (0), 0, IOVT_BOOL, 0},
	{"grat_arp", IOV_GRAT_ARP, (0), 0, IOVT_BOOL, 0},
	{"block_tdls", IOV_BLOCK_TDLS, (0), IOVT_BOOL, 0},
#endif /* DHD_L2_FILTER */
	{"dhd_ie", IOV_DHD_IE, (0), 0, IOVT_BUFFER, 0},
#ifdef DHD_PSTA
	/* PSTA/PSR Mode configuration. 0: DIABLED 1: PSTA 2: PSR */
	{"psta", IOV_PSTA, 0, 0, IOVT_UINT32, 0},
#endif /* DHD PSTA */
#ifdef DHD_WET
	/* WET Mode configuration. 0: DIABLED 1: WET */
	{"wet", IOV_WET, 0, 0, IOVT_UINT32, 0},
	{"wet_host_ipv4", IOV_WET_HOST_IPV4, 0, 0, IOVT_UINT32, 0},
	{"wet_host_mac", IOV_WET_HOST_MAC, 0, 0, IOVT_BUFFER, 0},
#endif /* DHD WET */
	{"op_mode",	IOV_CFG80211_OPMODE,	0,	0, IOVT_UINT32,	0 },
	{"assert_type", IOV_ASSERT_TYPE, (0), 0, IOVT_UINT32, 0},
	{"lmtest", IOV_LMTEST,	0,	0, IOVT_UINT32,	0 },
#ifdef DHD_MCAST_REGEN
	{"mcast_regen_bss_enable", IOV_MCAST_REGEN_BSS_ENABLE, 0, 0, IOVT_BOOL, 0},
#endif
#ifdef SHOW_LOGTRACE
	{"dump_trace_buf", IOV_DUMP_TRACE_LOG,	0, 0, IOVT_BUFFER,	sizeof(trace_buf_info_t) },
#endif /* SHOW_LOGTRACE */
	{"trap_type", IOV_DONGLE_TRAP_TYPE, 0, 0, IOVT_UINT32, 0 },
	{"trap_info", IOV_DONGLE_TRAP_INFO, 0, 0, IOVT_BUFFER, sizeof(trap_t) },
#ifdef DHD_DEBUG
	{"bpaddr", IOV_BPADDR,	0, 0, IOVT_BUFFER,	sizeof(sdreg_t) },
#endif /* DHD_DEBUG */
	{"dump_dongle", IOV_DUMP_DONGLE, 0, 0, IOVT_BUFFER,
	MAX(sizeof(dump_dongle_in_t), sizeof(dump_dongle_out_t)) },
#if defined(DHD_LOG_DUMP)
	{"log_dump", IOV_LOG_DUMP,	0, 0, IOVT_UINT8, 0},
#endif /* DHD_LOG_DUMP */
	{"tput_test", IOV_TPUT_TEST, 0, 0, IOVT_BUFFER, sizeof(tput_test_t)},
	{"debug_buf_dest_stat", IOV_DEBUG_BUF_DEST_STAT, 0, 0, IOVT_UINT32, 0 },
#if defined(DHD_SSSR_DUMP)
	{"fis_trigger", IOV_FIS_TRIGGER, 0, 0, IOVT_UINT32, 0},
#endif
#ifdef DHD_DEBUG
	{"induce_error", IOV_INDUCE_ERROR, (0), 0, IOVT_UINT16, 0 },
#endif /* DHD_DEBUG */
#ifdef RTT_GEOFENCE_CONT
#if defined(RTT_SUPPORT) && defined(WL_NAN)
	{"rtt_geofence_type_ovrd", IOV_RTT_GEOFENCE_TYPE_OVRD, (0), 0, IOVT_BOOL, 0},
#endif /* RTT_SUPPORT && WL_NAN */
#endif /* RTT_GEOFENCE_CONT */
	{"fw_verbose", IOV_FW_VBS, 0, 0, IOVT_UINT32, 0},
#ifdef DHD_TX_PROFILE
	{"tx_profile_tag", IOV_TX_PROFILE_TAG, 0, 0, IOVT_BUFFER,
	sizeof(dhd_tx_profile_protocol_t)},
	{"tx_profile_enable",	IOV_TX_PROFILE_ENABLE,	0,	0,	IOVT_BOOL,	0},
	{"tx_profile_dump",	IOV_TX_PROFILE_DUMP,	0,	0,	IOVT_UINT32,	0},
#endif /* defined(DHD_TX_PROFILE) */
	{"check_trap_rot", IOV_CHECK_TRAP_ROT, (0), 0, IOVT_BOOL, 0},
#if defined(WL_MON_OWN_PKT)
	{"roam_events", IOV_ROAM_EVENTS, 0, 0, IOVT_UINT32, 0},
#endif
#ifdef DHD_LOGLEVEL
	{"loglevel", IOV_LOGLEVEL, (0), 0, IOVT_BUFFER, sizeof(dhd_loglevel_data_t)},
#endif /* DHD_LOGLEVEL */
#ifdef BCMPCIE
	{"l1ss_init_enab", IOV_L1SS_INIT_ENAB, (0), 0, IOVT_BOOL, 0},
	{"aspm_init_enab", IOV_ASPM_INIT_ENAB, (0), 0, IOVT_BOOL, 0},
#endif /* BCMPCIE */
#ifdef SYNA_SAR_CUSTOMER_PARAMETER
	{"sarmode", IOV_SAR_MODE, (0), 0, IOVT_UINT32, 0},
#endif /* SYNA_SAR_CUSTOMER_PARAMETER */
	/* --- add new iovars *ABOVE* this line --- */
	{NULL, 0, 0, 0, 0, 0 }
};

#define DHD_IOVAR_BUF_SIZE	128

fw_download_status_t
dhd_fw_download_status(dhd_pub_t * dhd_pub)
{
	return dhd_pub->fw_download_status;
}

bool
dhd_query_bus_erros(dhd_pub_t *dhdp)
{
	bool ret = FALSE;

	if (dhdp->dongle_reset) {
		DHD_ERROR_RLMT(("%s: Dongle Reset occurred, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}

	if (dhdp->dongle_trap_occured) {
		DHD_ERROR_RLMT(("%s: FW TRAP has occurred, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
#ifdef OEM_ANDROID
		dhdp->hang_reason = HANG_REASON_DONGLE_TRAP;
		dhd_os_send_hang_message(dhdp);
#endif /* OEM_ANDROID */
	}

	if (dhdp->iovar_timeout_occured) {
		DHD_ERROR_RLMT(("%s: Resumed on timeout for previous IOVAR, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}

#ifdef PCIE_FULL_DONGLE
	if (dhdp->d3ack_timeout_occured) {
		DHD_ERROR_RLMT(("%s: Resumed on timeout for previous D3ACK, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}
	if (dhdp->livelock_occured) {
		DHD_ERROR_RLMT(("%s: LIVELOCK occurred for previous msg, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}

	if (dhdp->pktid_audit_failed) {
		DHD_ERROR_RLMT(("%s: pktid_audit_failed, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}
#endif /* PCIE_FULL_DONGLE */

	if (dhdp->iface_op_failed) {
		DHD_ERROR_RLMT(("%s: iface_op_failed, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}

	if (dhdp->scan_timeout_occurred) {
		DHD_ERROR_RLMT(("%s: scan_timeout_occurred, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}

	if (dhdp->scan_busy_occurred) {
		DHD_ERROR_RLMT(("%s: scan_busy_occurred, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}

#ifdef DNGL_AXI_ERROR_LOGGING
	if (dhdp->axi_error) {
		DHD_ERROR_RLMT(("%s: AXI error occurred, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}
#endif /* DNGL_AXI_ERROR_LOGGING */

	if (dhd_bus_get_linkdown(dhdp)) {
		DHD_ERROR_RLMT(("%s : PCIE Link down occurred, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}

	if (dhd_bus_get_cto(dhdp)) {
		DHD_ERROR_RLMT(("%s : CTO Recovery reported, cannot proceed\n",
			__FUNCTION__));
		ret = TRUE;
	}

	return ret;
}

void
dhd_clear_bus_errors(dhd_pub_t *dhdp)
{
	if (!dhdp)
		return;

	dhdp->dongle_reset = FALSE;
	dhdp->dongle_trap_occured = FALSE;
	dhdp->dsack_hc_due_to_isr_delay = FALSE;
	dhdp->dsack_hc_due_to_dpc_delay = FALSE;
	dhdp->iovar_timeout_occured = FALSE;
#ifdef PCIE_FULL_DONGLE
	dhdp->d3ack_timeout_occured = FALSE;
	dhdp->livelock_occured = FALSE;
	dhdp->pktid_audit_failed = FALSE;
#endif
	dhdp->iface_op_failed = FALSE;
	dhdp->scan_timeout_occurred = FALSE;
	dhdp->scan_busy_occurred = FALSE;
}

#ifdef DHD_SSSR_DUMP

/* This can be overwritten by module parameter defined in dhd_linux.c */
uint sssr_enab = TRUE;

#ifdef DHD_FIS_DUMP
uint fis_enab = TRUE;
#else
uint fis_enab = FALSE;
#endif /* DHD_FIS_DUMP */

int
dhd_sssr_mempool_init(dhd_pub_t *dhd)
{
	dhd->sssr_mempool = (uint8 *) VMALLOCZ(dhd->osh, DHD_SSSR_MEMPOOL_SIZE);
	if (dhd->sssr_mempool == NULL) {
		DHD_ERROR(("%s: VMALLOC of sssr_mempool failed\n",
			__FUNCTION__));
		return BCME_ERROR;
	}
	return BCME_OK;
}

void
dhd_sssr_mempool_deinit(dhd_pub_t *dhd)
{
	if (dhd->sssr_mempool) {
		VMFREE(dhd->osh, dhd->sssr_mempool, DHD_SSSR_MEMPOOL_SIZE);
		dhd->sssr_mempool = NULL;
	}
}

int
dhd_sssr_reg_info_init(dhd_pub_t *dhd)
{
	dhd->sssr_reg_info = (sssr_reg_info_cmn_t *) MALLOCZ(dhd->osh, sizeof(sssr_reg_info_cmn_t));
	if (dhd->sssr_reg_info == NULL) {
		DHD_ERROR(("%s: MALLOC of sssr_reg_info failed\n",
			__FUNCTION__));
		return BCME_ERROR;
	}
	return BCME_OK;
}

void
dhd_sssr_reg_info_deinit(dhd_pub_t *dhd)
{
	if (dhd->sssr_reg_info) {
		MFREE(dhd->osh, dhd->sssr_reg_info, sizeof(sssr_reg_info_cmn_t));
		dhd->sssr_reg_info = NULL;
	}
}

void
dhd_dump_sssr_reg_info(dhd_pub_t *dhd)
{
}

void dhdpcie_fill_sssr_reg_info(dhd_pub_t *dhd);

int
dhd_get_sssr_reg_info(dhd_pub_t *dhd)
{
	int ret;

	if (dhd->force_sssr_init) {
		dhdpcie_fill_sssr_reg_info(dhd);
		dhd->force_sssr_init = FALSE;
		goto done;
	}

	/* get sssr_reg_info from firmware */
	ret = dhd_iovar(dhd, 0, "sssr_reg_info", NULL, 0,  (char *)dhd->sssr_reg_info,
		sizeof(sssr_reg_info_cmn_t), FALSE);
	if (ret < 0) {
		DHD_ERROR(("%s: sssr_reg_info failed (error=%d)\n",
			__FUNCTION__, ret));
		return BCME_ERROR;
	}

done:
	dhd_dump_sssr_reg_info(dhd);
	return BCME_OK;
}

uint32
dhd_get_sssr_bufsize(dhd_pub_t *dhd)
{
	int i;
	uint32 sssr_bufsize = 0;
	uint8 num_d11cores;

	num_d11cores = dhd_d11_slices_num_get(dhd);

	switch (dhd->sssr_reg_info->rev2.version) {
		case SSSR_REG_INFO_VER_3 :
			/* intentional fall through */
		case SSSR_REG_INFO_VER_2 :
			for (i = 0; i < num_d11cores; i++) {
				sssr_bufsize += dhd->sssr_reg_info->rev2.mac_regs[i].sr_size;
			}
			if ((dhd->sssr_reg_info->rev2.length >
			 OFFSETOF(sssr_reg_info_v2_t, dig_mem_info)) &&
			 dhd->sssr_reg_info->rev2.dig_mem_info.dig_sr_addr) {
				sssr_bufsize += 0; /* TBD */
			}
			break;
		case SSSR_REG_INFO_VER_1 :
			for (i = 0; i < num_d11cores; i++) {
				sssr_bufsize += dhd->sssr_reg_info->rev1.mac_regs[i].sr_size;
			}
			if (dhd->sssr_reg_info->rev1.vasip_regs.vasip_sr_size) {
				sssr_bufsize += dhd->sssr_reg_info->rev1.vasip_regs.vasip_sr_size;
			} else if ((dhd->sssr_reg_info->rev1.length > OFFSETOF(sssr_reg_info_v1_t,
				dig_mem_info)) && dhd->sssr_reg_info->rev1.
				dig_mem_info.dig_sr_addr) {
				sssr_bufsize += dhd->sssr_reg_info->rev1.dig_mem_info.dig_sr_size;
			}
			break;
		case SSSR_REG_INFO_VER_0 :
			for (i = 0; i < num_d11cores; i++) {
				sssr_bufsize += dhd->sssr_reg_info->rev0.mac_regs[i].sr_size;
			}
			if (dhd->sssr_reg_info->rev0.vasip_regs.vasip_sr_size) {
				sssr_bufsize += dhd->sssr_reg_info->rev0.vasip_regs.vasip_sr_size;
			}
			break;
		default :
			DHD_ERROR(("invalid sssr_reg_ver"));
			return BCME_UNSUPPORTED;
	}

#ifdef DHD_SSSR_DUMP_BEFORE_SR
	/* Double the size as different dumps will be saved before and after SR */
	sssr_bufsize = 2 * sssr_bufsize;
#endif /* DHD_SSSR_DUMP_BEFORE_SR */

	return sssr_bufsize;
}

int
dhd_sssr_dump_init(dhd_pub_t *dhd)
{
	int i;
	uint32 sssr_bufsize;
	uint32 mempool_used = 0;
	uint8 num_d11cores = 0;
	bool alloc_sssr = FALSE;
	uint32 sr_size = 0;

	dhd->sssr_inited = FALSE;
	if (!sssr_enab) {
		DHD_ERROR(("%s: sssr dump not inited as instructed by mod param\n", __FUNCTION__));
		return BCME_OK;
	}

	/* check if sssr mempool is allocated */
	if (dhd->sssr_mempool == NULL) {
		DHD_ERROR(("%s: sssr_mempool is not allocated\n",
			__FUNCTION__));
		return BCME_ERROR;
	}

	/* check if sssr mempool is allocated */
	if (dhd->sssr_reg_info == NULL) {
		DHD_ERROR(("%s: sssr_reg_info is not allocated\n",
			__FUNCTION__));
		return BCME_ERROR;
	}

	/* Get SSSR reg info */
	if (dhd_get_sssr_reg_info(dhd) != BCME_OK) {
		DHD_ERROR(("%s: dhd_get_sssr_reg_info failed\n", __FUNCTION__));
		printf("DEBUG_SSSr: %s: dhd_get_sssr_reg_info failed\n", __FUNCTION__);
		return BCME_ERROR;
	}

	num_d11cores = dhd_d11_slices_num_get(dhd);
	/* Validate structure version and length */
	switch (dhd->sssr_reg_info->rev2.version) {
		case SSSR_REG_INFO_VER_3 :
			if (dhd->sssr_reg_info->rev3.length != sizeof(sssr_reg_info_v3_t)) {
				DHD_ERROR(("%s: dhd->sssr_reg_info->rev2.length (%d : %d)"
					 "mismatch on rev2\n", __FUNCTION__,
					 (int)dhd->sssr_reg_info->rev3.length,
					 (int)sizeof(sssr_reg_info_v3_t)));
				return BCME_ERROR;
			}
			break;
		case SSSR_REG_INFO_VER_2 :
			if (dhd->sssr_reg_info->rev2.length != sizeof(sssr_reg_info_v2_t)) {
				DHD_ERROR(("%s: dhd->sssr_reg_info->rev2.length (%d : %d)"
					 "mismatch on rev2\n", __FUNCTION__,
					 (int)dhd->sssr_reg_info->rev2.length,
					 (int)sizeof(sssr_reg_info_v2_t)));
				return BCME_ERROR;
			}
			break;
		case SSSR_REG_INFO_VER_1 :
			if (dhd->sssr_reg_info->rev1.length != sizeof(sssr_reg_info_v1_t)) {
				DHD_ERROR(("%s: dhd->sssr_reg_info->rev1.length (%d : %d)"
					 "mismatch on rev1\n", __FUNCTION__,
					 (int)dhd->sssr_reg_info->rev1.length,
					 (int)sizeof(sssr_reg_info_v1_t)));
				return BCME_ERROR;
			}
			break;
		case SSSR_REG_INFO_VER_0 :
			if (dhd->sssr_reg_info->rev0.length != sizeof(sssr_reg_info_v0_t)) {
				DHD_ERROR(("%s: dhd->sssr_reg_info->rev0.length (%d : %d)"
					 "mismatch on rev0\n", __FUNCTION__,
					 (int)dhd->sssr_reg_info->rev0.length,
					 (int)sizeof(sssr_reg_info_v0_t)));
				return BCME_ERROR;
			}
			break;
		default :
			DHD_ERROR(("invalid sssr_reg_ver"));
			return BCME_UNSUPPORTED;
	}

	/* validate fifo size */
	sssr_bufsize = dhd_get_sssr_bufsize(dhd);
	if (sssr_bufsize > DHD_SSSR_MEMPOOL_SIZE) {
		DHD_ERROR(("%s: sssr_bufsize(%d) is greater than sssr_mempool(%d)\n",
			__FUNCTION__, (int)sssr_bufsize, DHD_SSSR_MEMPOOL_SIZE));
		return BCME_ERROR;
	}

	/* init all pointers to NULL */
	for (i = 0; i < num_d11cores; i++) {
#ifdef DHD_SSSR_DUMP_BEFORE_SR
		dhd->sssr_d11_before[i] = NULL;
#endif /* DHD_SSSR_DUMP_BEFORE_SR */
		dhd->sssr_d11_after[i] = NULL;
	}

#ifdef DHD_SSSR_DUMP_BEFORE_SR
	dhd->sssr_dig_buf_before = NULL;
#endif /* DHD_SSSR_DUMP_BEFORE_SR */
	dhd->sssr_dig_buf_after = NULL;

	/* Allocate memory */
	for (i = 0; i < num_d11cores; i++) {
		alloc_sssr = FALSE;
		sr_size = 0;

		switch (dhd->sssr_reg_info->rev2.version) {
			case SSSR_REG_INFO_VER_3 :
				/* intentional fall through */
			case SSSR_REG_INFO_VER_2 :
				if (dhd->sssr_reg_info->rev2.mac_regs[i].sr_size) {
					alloc_sssr = TRUE;
					sr_size = dhd->sssr_reg_info->rev2.mac_regs[i].sr_size;
				}
				break;
			case SSSR_REG_INFO_VER_1 :
				if (dhd->sssr_reg_info->rev1.mac_regs[i].sr_size) {
					alloc_sssr = TRUE;
					sr_size = dhd->sssr_reg_info->rev1.mac_regs[i].sr_size;
				}
				break;
			case SSSR_REG_INFO_VER_0 :
				if (dhd->sssr_reg_info->rev0.mac_regs[i].sr_size) {
					alloc_sssr = TRUE;
					sr_size = dhd->sssr_reg_info->rev0.mac_regs[i].sr_size;
				}
				break;
			default :
				DHD_ERROR(("invalid sssr_reg_ver"));
				return BCME_UNSUPPORTED;
		}

		if (alloc_sssr) {
#ifdef DHD_SSSR_DUMP_BEFORE_SR
			dhd->sssr_d11_before[i] = (uint32 *)(dhd->sssr_mempool + mempool_used);
			mempool_used += sr_size;
#endif /* DHD_SSSR_DUMP_BEFORE_SR */

			dhd->sssr_d11_after[i] = (uint32 *)(dhd->sssr_mempool + mempool_used);
			mempool_used += sr_size;
		}
	}

	/* Allocate dump memory for VASIP (version 0 or 1) or digital core (version 0, 1, or 2) */
	alloc_sssr = FALSE;
	sr_size = 0;
	switch (dhd->sssr_reg_info->rev2.version) {
		case SSSR_REG_INFO_VER_3 :
			/* intentional fall through */
		case SSSR_REG_INFO_VER_2 :
			if ((dhd->sssr_reg_info->rev2.length >
			 OFFSETOF(sssr_reg_info_v2_t, dig_mem_info)) &&
			 dhd->sssr_reg_info->rev2.dig_mem_info.dig_sr_addr) {
				alloc_sssr = TRUE;
				sr_size = dhd->sssr_reg_info->rev2.dig_mem_info.dig_sr_size;
			}
			break;
		case SSSR_REG_INFO_VER_1 :
			if (dhd->sssr_reg_info->rev1.vasip_regs.vasip_sr_size) {
				alloc_sssr = TRUE;
				sr_size = dhd->sssr_reg_info->rev1.vasip_regs.vasip_sr_size;
			} else if ((dhd->sssr_reg_info->rev1.length > OFFSETOF(sssr_reg_info_v1_t,
				dig_mem_info)) && dhd->sssr_reg_info->rev1.
				dig_mem_info.dig_sr_addr) {
				alloc_sssr = TRUE;
				sr_size = dhd->sssr_reg_info->rev1.dig_mem_info.dig_sr_size;
			}
			break;
		case SSSR_REG_INFO_VER_0 :
			if (dhd->sssr_reg_info->rev0.vasip_regs.vasip_sr_size) {
				alloc_sssr = TRUE;
				sr_size = dhd->sssr_reg_info->rev0.vasip_regs.vasip_sr_size;
			}
			break;
		default :
			DHD_ERROR(("invalid sssr_reg_ver"));
			return BCME_UNSUPPORTED;
	}

	if (alloc_sssr) {
		dhd->sssr_dig_buf_after = (uint32 *)(dhd->sssr_mempool + mempool_used);
		mempool_used += sr_size;

#ifdef DHD_SSSR_DUMP_BEFORE_SR
		/* DIG dump before suspend is not applicable. */
		dhd->sssr_dig_buf_before = NULL;
#endif /* DHD_SSSR_DUMP_BEFORE_SR */
	}

	dhd->sssr_inited = TRUE;

	return BCME_OK;

}

void
dhd_sssr_dump_deinit(dhd_pub_t *dhd)
{
	int i;

	dhd->sssr_inited = FALSE;
	/* init all pointers to NULL */
	for (i = 0; i < MAX_NUM_D11_CORES_WITH_SCAN; i++) {
#ifdef DHD_SSSR_DUMP_BEFORE_SR
		dhd->sssr_d11_before[i] = NULL;
#endif /* DHD_SSSR_DUMP_BEFORE_SR */
		dhd->sssr_d11_after[i] = NULL;
	}
#ifdef DHD_SSSR_DUMP_BEFORE_SR
	dhd->sssr_dig_buf_before = NULL;
#endif /* DHD_SSSR_DUMP_BEFORE_SR */
	dhd->sssr_dig_buf_after = NULL;

	return;
}

void
dhd_sssr_print_filepath(dhd_pub_t *dhd, char *path)
{
	bool print_info = FALSE;
	int dump_mode;

	if (!dhd || !path) {
		DHD_ERROR(("%s: dhd or memdump_path is NULL\n",
			__FUNCTION__));
		return;
	}

	if (!dhd->sssr_dump_collected) {
		/* SSSR dump is not collected */
		return;
	}

	dump_mode = dhd->sssr_dump_mode;

	if (bcmstrstr(path, "core_0_before")) {
		if (dhd->sssr_d11_outofreset[0] &&
			dump_mode == SSSR_DUMP_MODE_SSSR) {
			print_info = TRUE;
		}
	} else if (bcmstrstr(path, "core_0_after")) {
		if (dhd->sssr_d11_outofreset[0]) {
			print_info = TRUE;
		}
	} else if (bcmstrstr(path, "core_1_before")) {
		if (dhd->sssr_d11_outofreset[1] &&
			dump_mode == SSSR_DUMP_MODE_SSSR) {
			print_info = TRUE;
		}
	} else if (bcmstrstr(path, "core_1_after")) {
		if (dhd->sssr_d11_outofreset[1]) {
			print_info = TRUE;
		}
	} else if (bcmstrstr(path, "core_2_before")) {
		if (dhd->sssr_d11_outofreset[2] &&
			dump_mode == SSSR_DUMP_MODE_SSSR) {
			print_info = TRUE;
		}
	} else if (bcmstrstr(path, "core_2_after")) {
		if (dhd->sssr_d11_outofreset[2]) {
			print_info = TRUE;
		}
	} else {
		print_info = TRUE;
	}

	if (print_info) {
		DHD_ERROR(("%s: file_path = %s%s\n", __FUNCTION__,
			path, FILE_NAME_HAL_TAG));
	}
}
#endif /* DHD_SSSR_DUMP */

#ifdef DHD_COREDUMP
int
dhd_coredump_mempool_init(dhd_pub_t *dhd)
{
	dhd->coredump_mem = (uint8*) VMALLOCZ(dhd->osh, DHD_MEMDUMP_BUFFER_SIZE);
	if (dhd->coredump_mem == NULL) {
		DHD_ERROR(("%s: VMALLOCZ of coredump_mem failed\n", __FUNCTION__));
		return BCME_ERROR;
	}

	dhd->coredump_len = DHD_MEMDUMP_BUFFER_SIZE;
	return BCME_OK;
}

void
dhd_coredump_mempool_deinit(dhd_pub_t *dhd)
{
	if (dhd->coredump_mem) {
		VMFREE(dhd->osh, dhd->coredump_mem, DHD_MEMDUMP_BUFFER_SIZE);
		dhd->coredump_mem = NULL;
		dhd->coredump_len = 0;
	}
}

#ifdef DHD_SSSR_DUMP
dhd_coredump_t dhd_coredump_types[] = {
	{DHD_COREDUMP_TYPE_SSSRDUMP_CORE0_BEFORE, 0, NULL},
	{DHD_COREDUMP_TYPE_SSSRDUMP_CORE0_AFTER, 0, NULL},
	{DHD_COREDUMP_TYPE_SSSRDUMP_CORE1_BEFORE, 0, NULL},
	{DHD_COREDUMP_TYPE_SSSRDUMP_CORE1_AFTER, 0, NULL},
	{DHD_COREDUMP_TYPE_SSSRDUMP_CORE2_BEFORE, 0, NULL},
	{DHD_COREDUMP_TYPE_SSSRDUMP_CORE2_AFTER, 0, NULL},
	{DHD_COREDUMP_TYPE_SSSRDUMP_DIG_BEFORE, 0, NULL},
	{DHD_COREDUMP_TYPE_SSSRDUMP_DIG_AFTER, 0, NULL},
	{DHD_COREDUMP_TYPE_SOCRAMDUMP, 0, NULL}
};

static int dhd_append_sssr_tlv(uint8 *buf_dst, int type_idx, int buf_remain)
{
	uint32 type_val, length_val;
	uint32 *type, *length;
	void *buf_src;
	int total_size = 0, ret = 0;

	/* DHD_COREDUMP_TYPE_SSSRDUMP_[CORE[0|1|2]|DIG]_[BEFORE|AFTER] */
	type_val = dhd_coredump_types[type_idx].type;
	length_val = dhd_coredump_types[type_idx].length;

	if (length_val == 0) {
		return 0;
	}

	type = (uint32*)buf_dst;
	*type = type_val;
	length = (uint32*)(buf_dst + sizeof(*type));
	*length = length_val;

	buf_dst += TLV_TYPE_LENGTH_SIZE;
	total_size += TLV_TYPE_LENGTH_SIZE;

	buf_src = dhd_coredump_types[type_idx].bufptr;
	ret = memcpy_s(buf_dst, buf_remain, buf_src, *length);

	if (ret) {
		DHD_ERROR(("Failed to memcpy_s() for coredump.\n"));
		return BCME_ERROR;
	}

	DHD_INFO(("%s: type: %u, length: %u\n",	__FUNCTION__, *type, *length));

	total_size += *length;
	return total_size;
}

/* reconstruct dump memory with socram and sssr dump as TLV format */
int dhd_collect_coredump(dhd_pub_t *dhdp, dhd_dump_t *dump)
{
	uint8 *buf_ptr = dhdp->coredump_mem;
	int buf_len = dhdp->coredump_len;
	uint8 *socram_mem = dump->buf;
	int socram_len = dump->bufsize;
	int ret = 0, i = 0;
	int total_size = 0;
	uint8 num_d11cores = 0;

	ret = memcpy_s(buf_ptr, buf_len,
			socram_mem, socram_len);

	if (ret) {
		DHD_ERROR(("Failed to memmove_s() for coredump.\n"));
		return BCME_ERROR;
	}

	total_size = socram_len;
	buf_ptr += total_size;

	/* SSSR dump */
	if (!sssr_enab || !dhdp->collect_sssr) {
		DHD_ERROR(("SSSR is not enabled or not collected yet "
				"sssr_enab = %d, dhdp->collect_sssr = %d\n",
				sssr_enab, dhdp->collect_sssr));
		return BCME_UNSUPPORTED;
	}

	num_d11cores = dhd_d11_slices_num_get(dhdp);
	for (i = 0; i < num_d11cores + 1; i++) {
		int written_bytes = 0;
		int type_idx = i * 2;
		if ((i < num_d11cores) && (!dhdp->sssr_d11_outofreset[i])) {
			continue;
		}

#ifdef DHD_SSSR_DUMP_BEFORE_SR
		/* DHD_COREDUMP_TYPE_SSSRDUMP_[CORE[0|1|2]|DIG]_BEFORE */
		written_bytes = dhd_append_sssr_tlv(buf_ptr, type_idx,
			buf_len - total_size);
		if (written_bytes == BCME_ERROR) {
			return BCME_ERROR;
		}
		buf_ptr += written_bytes;
		total_size += written_bytes;
#endif /* DHD_SSSR_DUMP_BEFORE_SR */

		/* DHD_COREDUMP_TYPE_SSSRDUMP_[CORE[0|1|2]|DIG]_AFTER */
		written_bytes = dhd_append_sssr_tlv(buf_ptr, type_idx + 1,
			buf_len - total_size);
		if (written_bytes == BCME_ERROR) {
			return BCME_ERROR;
		}
		buf_ptr += written_bytes;
		total_size += written_bytes;
	}

	dump->buf = dhdp->coredump_mem;
	dump->bufsize = total_size;

	return BCME_OK;
}
#endif /* DHD_SSSR_DUMP */
#endif /* DHD_COREDUMP */

#ifdef DHD_SDTC_ETB_DUMP
/*
 * sdtc: system debug trace controller
 * etb: embedded trace buf
 */
void
dhd_sdtc_etb_init(dhd_pub_t *dhd)
{
	bcm_iov_buf_t *iov_req = NULL;
	etb_addr_info_t *p_etb_addr_info = NULL;
	bcm_iov_buf_t *iov_resp = NULL;
	uint8 *buf = NULL;
	int ret = 0;
	uint16 iovlen = 0;
	uint16 version = 0;

	BCM_REFERENCE(p_etb_addr_info);
	dhd->sdtc_etb_inited = FALSE;

	iov_req = MALLOCZ(dhd->osh, WLC_IOCTL_SMLEN);
	if (iov_req == NULL) {
		DHD_ERROR(("%s: Failed to alloc buffer for iovar request\n", __FUNCTION__));
		goto exit;
	}

	buf = MALLOCZ(dhd->osh, WLC_IOCTL_MAXLEN);
	if (buf == NULL) {
		DHD_ERROR(("%s: Failed to alloc buffer for iovar response\n", __FUNCTION__));
		goto exit;
	}

	/* fill header */
	iov_req->version = WL_SDTC_IOV_VERSION;
	iov_req->id = WL_SDTC_CMD_ETB_INFO;
	iov_req->len = sizeof(etb_addr_info_t);
	iovlen = OFFSETOF(bcm_iov_buf_t, data) + iov_req->len;

	ret = dhd_iovar(dhd, 0, "sdtc", (char *)iov_req, iovlen,
		(char *)buf, WLC_IOCTL_MAXLEN, FALSE);
	if (ret < 0) {
		DHD_ERROR(("%s failed to get sdtc etb_info %d\n", __FUNCTION__, ret));
		goto exit;
	}

	version = dtoh16(*(uint16 *)buf);
	/* Check for version */
	if (version != WL_SDTC_IOV_VERSION) {
		DHD_ERROR(("%s WL_SDTC_IOV_VERSION mis match\n", __FUNCTION__));
		goto exit;
	}
	iov_resp = (bcm_iov_buf_t *)buf;
	if (iov_resp->id == iov_req->id) {
		p_etb_addr_info = (etb_addr_info_t*)iov_resp->data;
		dhd->etb_addr_info.version = p_etb_addr_info->version;
		dhd->etb_addr_info.len = p_etb_addr_info->len;
		dhd->etb_addr_info.etbinfo_addr = p_etb_addr_info->etbinfo_addr;

		DHD_ERROR(("%s etb_addr_info: ver:%d, len:%d, addr:0x%x\n", __FUNCTION__,
			dhd->etb_addr_info.version, dhd->etb_addr_info.len,
			dhd->etb_addr_info.etbinfo_addr));
	} else {
		DHD_ERROR(("%s Unknown CMD-ID (%d) as  response for request ID %d\n",
			__FUNCTION__, iov_resp->id, iov_req->id));
		goto exit;
	}

	/* since all the requirements for SDTC and ETB are met mark the capability as TRUE */
	dhd->sdtc_etb_inited = TRUE;
	DHD_ERROR(("%s sdtc_etb_inited: %d\n", __FUNCTION__, dhd->sdtc_etb_inited));
exit:
	if (iov_req) {
		MFREE(dhd->osh, iov_req, WLC_IOCTL_SMLEN);
	}
	if (buf) {
		MFREE(dhd->osh, buf, WLC_IOCTL_MAXLEN);
	}
	return;
}

void
dhd_sdtc_etb_deinit(dhd_pub_t *dhd)
{
	dhd->sdtc_etb_inited = FALSE;
	dhd->sdtc_etb_dump_len = 0;
}

int
dhd_sdtc_etb_mempool_init(dhd_pub_t *dhd)
{
	dhd->sdtc_etb_dump_len = 0;
	dhd->sdtc_etb_mempool = (uint8 *) MALLOCZ(dhd->osh, DHD_SDTC_ETB_MEMPOOL_SIZE);
	if (dhd->sdtc_etb_mempool == NULL) {
		DHD_ERROR(("%s: MALLOC of sdtc_etb_mempool failed\n",
			__FUNCTION__));
		return BCME_ERROR;
	}
	return BCME_OK;
}

void
dhd_sdtc_etb_mempool_deinit(dhd_pub_t *dhd)
{
	if (dhd->sdtc_etb_mempool) {
		MFREE(dhd->osh, dhd->sdtc_etb_mempool, DHD_SDTC_ETB_MEMPOOL_SIZE);
		dhd->sdtc_etb_mempool = NULL;
	}
}
#endif /* DHD_SDTC_ETB_DUMP */

#ifdef DHD_FW_COREDUMP
void* dhd_get_fwdump_buf(dhd_pub_t *dhd_pub, uint32 length)
{
	if (!dhd_pub->soc_ram) {
#if defined(CONFIG_DHD_USE_STATIC_BUF) && defined(DHD_USE_STATIC_MEMDUMP)
		dhd_pub->soc_ram = (uint8*)DHD_OS_PREALLOC(dhd_pub,
			DHD_PREALLOC_MEMDUMP_RAM, length);
#else
		dhd_pub->soc_ram = (uint8*) MALLOC(dhd_pub->osh, length);

		if ((dhd_pub->soc_ram == NULL) && CAN_SLEEP()) {
			DHD_ERROR(("%s: Try to allocate virtual memory for fw crash snap shot.\n",
				__FUNCTION__));
			dhd_pub->soc_ram = (uint8*) VMALLOC(dhd_pub->osh, length);
		}
#endif /* CONFIG_DHD_USE_STATIC_BUF && DHD_USE_STATIC_MEMDUMP */
	}

	if (dhd_pub->soc_ram == NULL) {
		DHD_ERROR(("%s: *Failed to allocate memory for fw crash snap shot.\n",
			__FUNCTION__));
		dhd_pub->soc_ram_length = 0;
	} else {
		memset(dhd_pub->soc_ram, 0, length);
		dhd_pub->soc_ram_length = length;
	}

	/* soc_ram free handled in dhd_{free,clear} */
	return dhd_pub->soc_ram;
}
#endif /* DHD_FW_COREDUMP */

/* to NDIS developer, the structure dhd_common is redundant,
 * please do NOT merge it back from other branches !!!
 */

int
dhd_common_socram_dump(dhd_pub_t *dhdp)
{
	return dhd_socram_dump(dhdp->bus);
}

int
dhd_dump(dhd_pub_t *dhdp, char *buf, int buflen)
{
	struct bcmstrbuf b;
	struct bcmstrbuf *strbuf = &b;
#ifdef DHD_MEM_STATS
	uint64 malloc_mem = 0;
	uint64 total_dhd_mem = 0;
#ifdef BCMPCIE
	uint64 txpath_bkpq_len = 0;
	uint64 txpath_bkpq_mem = 0;
	uint64 total_txpath_mem = 0;
#endif /* BCMPCIE */
#endif /* DHD_MEM_STATS */

	if (!dhdp || !dhdp->prot || !buf) {
		return BCME_ERROR;
	}

	bcm_binit(strbuf, buf, buflen);

	/* Base DHD info */
	bcm_bprintf(strbuf, "%s\n", dhd_version);
	bcm_bprintf(strbuf, "\n");
	bcm_bprintf(strbuf, "pub.up %d pub.txoff %d pub.busstate %d\n",
	            dhdp->up, dhdp->txoff, dhdp->busstate);
	bcm_bprintf(strbuf, "pub.hdrlen %u pub.maxctl %u pub.rxsz %u\n",
	            dhdp->hdrlen, dhdp->maxctl, dhdp->rxsz);
	bcm_bprintf(strbuf, "pub.iswl %d pub.drv_version %ld pub.mac "MACDBG"\n",
	            dhdp->iswl, dhdp->drv_version, MAC2STRDBG(&dhdp->mac));
	bcm_bprintf(strbuf, "pub.bcmerror %d tickcnt %u\n", dhdp->bcmerror, dhdp->tickcnt);

	bcm_bprintf(strbuf, "dongle stats:\n");
	bcm_bprintf(strbuf, "tx_packets %lu tx_bytes %lu tx_errors %lu tx_dropped %lu\n",
	            dhdp->dstats.tx_packets, dhdp->dstats.tx_bytes,
	            dhdp->dstats.tx_errors, dhdp->dstats.tx_dropped);
	bcm_bprintf(strbuf, "rx_packets %lu rx_bytes %lu rx_errors %lu rx_dropped %lu\n",
	            dhdp->dstats.rx_packets, dhdp->dstats.rx_bytes,
	            dhdp->dstats.rx_errors, dhdp->dstats.rx_dropped);
	bcm_bprintf(strbuf, "multicast %lu\n", dhdp->dstats.multicast);

	bcm_bprintf(strbuf, "bus stats:\n");
	bcm_bprintf(strbuf, "tx_packets %lu  tx_dropped %lu tx_multicast %lu tx_errors %lu\n",
	            dhdp->tx_packets, dhdp->tx_dropped, dhdp->tx_multicast, dhdp->tx_errors);
	bcm_bprintf(strbuf, "tx_ctlpkts %lu tx_ctlerrs %lu\n",
	            dhdp->tx_ctlpkts, dhdp->tx_ctlerrs);
	bcm_bprintf(strbuf, "rx_packets %lu rx_multicast %lu rx_errors %lu \n",
	            dhdp->rx_packets, dhdp->rx_multicast, dhdp->rx_errors);
	bcm_bprintf(strbuf, "rx_ctlpkts %lu rx_ctlerrs %lu rx_dropped %lu\n",
	            dhdp->rx_ctlpkts, dhdp->rx_ctlerrs, dhdp->rx_dropped);
	bcm_bprintf(strbuf, "rx_readahead_cnt %lu tx_realloc %lu\n",
	            dhdp->rx_readahead_cnt, dhdp->tx_realloc);
	bcm_bprintf(strbuf, "tx_pktgetfail %lu rx_pktgetfail %lu\n",
	            dhdp->tx_pktgetfail, dhdp->rx_pktgetfail);
	bcm_bprintf(strbuf, "tx_big_packets %lu\n",
	            dhdp->tx_big_packets);
	bcm_bprintf(strbuf, "\n");
#ifdef DMAMAP_STATS
	/* Add DMA MAP info */
	bcm_bprintf(strbuf, "DMA MAP stats: \n");
	bcm_bprintf(strbuf, "txdata: %lu size: %luK, rxdata: %lu size: %luK\n",
			dhdp->dma_stats.txdata, KB(dhdp->dma_stats.txdata_sz),
			dhdp->dma_stats.rxdata, KB(dhdp->dma_stats.rxdata_sz));
#ifndef IOCTLRESP_USE_CONSTMEM
	bcm_bprintf(strbuf, "IOCTL RX: %lu size: %luK ,",
			dhdp->dma_stats.ioctl_rx, KB(dhdp->dma_stats.ioctl_rx_sz));
#endif /* !IOCTLRESP_USE_CONSTMEM */
	bcm_bprintf(strbuf, "EVENT RX: %lu size: %luK, INFO RX: %lu size: %luK, "
			"TSBUF RX: %lu size %luK\n",
			dhdp->dma_stats.event_rx, KB(dhdp->dma_stats.event_rx_sz),
			dhdp->dma_stats.info_rx, KB(dhdp->dma_stats.info_rx_sz),
			dhdp->dma_stats.tsbuf_rx, KB(dhdp->dma_stats.tsbuf_rx_sz));
	bcm_bprintf(strbuf, "Total : %luK \n",
			KB(dhdp->dma_stats.txdata_sz + dhdp->dma_stats.rxdata_sz +
			dhdp->dma_stats.ioctl_rx_sz + dhdp->dma_stats.event_rx_sz +
			dhdp->dma_stats.tsbuf_rx_sz));
#endif /* DMAMAP_STATS */
	bcm_bprintf(strbuf, "dhd_induce_error : %u\n", dhdp->dhd_induce_error);
	/* Add any prot info */
	dhd_prot_dump(dhdp, strbuf);
	bcm_bprintf(strbuf, "\n");

	/* Add any bus info */
	dhd_bus_dump(dhdp, strbuf);

#if defined(DHD_LB_STATS)
	dhd_lb_stats_dump(dhdp, strbuf);
#endif /* DHD_LB_STATS */

#ifdef DHD_MEM_STATS

	malloc_mem = MALLOCED(dhdp->osh);

	bcm_bprintf(strbuf, "\nDHD malloc memory_usage: %llubytes %lluKB\n",
		malloc_mem, (malloc_mem / 1024));

	total_dhd_mem = malloc_mem;
#ifdef BCMPCIE
	txpath_bkpq_len = dhd_active_tx_flowring_bkpq_len(dhdp);
	/*
	 * Instead of traversing the entire queue to find the skbs length,
	 * considering MAX_MTU_SZ as lenth of each skb.
	 */
	txpath_bkpq_mem = (txpath_bkpq_len* MAX_MTU_SZ);
	total_txpath_mem = dhdp->txpath_mem + txpath_bkpq_mem;

	bcm_bprintf(strbuf, "\nDHD tx-bkpq len: %llu memory_usage: %llubytes %lluKB\n",
		txpath_bkpq_len, txpath_bkpq_mem, (txpath_bkpq_mem / 1024));
	bcm_bprintf(strbuf, "DHD tx-path memory_usage: %llubytes %lluKB\n",
		total_txpath_mem, (total_txpath_mem / 1024));

	total_dhd_mem += total_txpath_mem;
#endif /* BCMPCIE */
#if defined(DHD_LB_STATS)
	total_dhd_mem += dhd_lb_mem_usage(dhdp, strbuf);
#endif /* DHD_LB_STATS */
	bcm_bprintf(strbuf, "\nDHD Totoal memory_usage: %llubytes %lluKB \n",
		total_dhd_mem, (total_dhd_mem / 1024));
#endif /* DHD_MEM_STATS */
#if defined(DHD_LB_STATS)
	bcm_bprintf(strbuf, "\nlb_rxp_stop_thr_hitcnt: %llu lb_rxp_strt_thr_hitcnt: %llu\n",
		dhdp->lb_rxp_stop_thr_hitcnt, dhdp->lb_rxp_strt_thr_hitcnt);
	bcm_bprintf(strbuf, "\nlb_rxp_napi_sched_cnt: %llu lb_rxp_napi_complete_cnt: %llu\n",
		dhdp->lb_rxp_napi_sched_cnt, dhdp->lb_rxp_napi_complete_cnt);
#endif /* DHD_LB_STATS */

#if defined(DHD_MQ) && defined(DHD_MQ_STATS)
	dhd_mqstats_dump(dhdp, strbuf);
#endif

#ifdef DHD_WET
	if (dhd_get_wet_mode(dhdp)) {
		bcm_bprintf(strbuf, "Wet Dump:\n");
		dhd_wet_dump(dhdp, strbuf);
		}
#endif /* DHD_WET */

	DHD_ERROR(("%s bufsize: %d free: %d", __FUNCTION__, buflen, strbuf->size));
	/* return remaining buffer length */
	return (!strbuf->size ? BCME_BUFTOOSHORT : strbuf->size);
}

void
dhd_dump_to_kernelog(dhd_pub_t *dhdp)
{
	char buf[512];

	DHD_ERROR(("F/W version: %s\n", fw_version));
	bcm_bprintf_bypass = TRUE;
	dhd_dump(dhdp, buf, sizeof(buf));
	bcm_bprintf_bypass = FALSE;
}

int
dhd_wl_ioctl_cmd(dhd_pub_t *dhd_pub, int cmd, void *arg, int len, uint8 set, int ifidx)
{
	wl_ioctl_t ioc;

	ioc.cmd = cmd;
	ioc.buf = arg;
	ioc.len = len;
	ioc.set = set;

	return dhd_wl_ioctl(dhd_pub, ifidx, &ioc, arg, len);
}

int
dhd_wl_ioctl_get_intiovar(dhd_pub_t *dhd_pub, char *name, uint *pval,
	int cmd, uint8 set, int ifidx)
{
	char iovbuf[WLC_IOCTL_SMLEN];
	int ret = -1;

	memset(iovbuf, 0, sizeof(iovbuf));
	if (bcm_mkiovar(name, NULL, 0, iovbuf, sizeof(iovbuf))) {
		ret = dhd_wl_ioctl_cmd(dhd_pub, cmd, iovbuf, sizeof(iovbuf), set, ifidx);
		if (0 <= ret) {
			*pval = ltoh32(*((uint*)iovbuf));
		} else {
			DHD_ERROR(("%s: get int iovar %s failed, ERR %d\n",
				__FUNCTION__, name, ret));
		}
	} else {
		DHD_ERROR(("%s: mkiovar %s failed\n",
			__FUNCTION__, name));
	}

	return ret;
}

int
dhd_wl_ioctl_set_intiovar(dhd_pub_t *dhd_pub, char *name, uint val,
	int cmd, uint8 set, int ifidx)
{
	char iovbuf[WLC_IOCTL_SMLEN];
	int ret = -1;
	int lval = htol32(val);
	uint len;

	len = bcm_mkiovar(name, (char*)&lval, sizeof(lval), iovbuf, sizeof(iovbuf));

	if (len) {
		ret = dhd_wl_ioctl_cmd(dhd_pub, cmd, iovbuf, len, set, ifidx);
		if (ret) {
			DHD_ERROR(("%s: set int iovar %s failed, ERR %d\n",
				__FUNCTION__, name, ret));
		}
	} else {
		DHD_ERROR(("%s: mkiovar %s failed\n",
			__FUNCTION__, name));
	}

	return ret;
}

static struct ioctl2str_s {
	uint32 ioctl;
	char *name;
} ioctl2str_array[] = {
	{WLC_UP, "UP"},
	{WLC_DOWN, "DOWN"},
	{WLC_SET_PROMISC, "SET_PROMISC"},
	{WLC_SET_INFRA, "SET_INFRA"},
	{WLC_SET_AUTH, "SET_AUTH"},
	{WLC_SET_SSID, "SET_SSID"},
	{WLC_RESTART, "RESTART"},
	{WLC_SET_CHANNEL, "SET_CHANNEL"},
	{WLC_SET_RATE_PARAMS, "SET_RATE_PARAMS"},
	{WLC_SET_KEY, "SET_KEY"},
	{WLC_SCAN, "SCAN"},
	{WLC_DISASSOC, "DISASSOC"},
	{WLC_REASSOC, "REASSOC"},
	{WLC_SET_COUNTRY, "SET_COUNTRY"},
	{WLC_SET_WAKE, "SET_WAKE"},
	{WLC_SET_SCANSUPPRESS, "SET_SCANSUPPRESS"},
	{WLC_SCB_DEAUTHORIZE, "SCB_DEAUTHORIZE"},
	{WLC_SET_WSEC, "SET_WSEC"},
	{WLC_SET_INTERFERENCE_MODE, "SET_INTERFERENCE_MODE"},
	{WLC_SET_RADAR, "SET_RADAR"},
	{0, NULL}
};

static char *
ioctl2str(uint32 ioctl)
{
	struct ioctl2str_s *p = ioctl2str_array;

	while (p->name != NULL) {
		if (p->ioctl == ioctl) {
			return p->name;
		}
		p++;
	}

	return "";
}

/**
 * @param ioc          IO control struct, members are partially used by this function.
 * @param buf [inout]  Contains parameters to send to dongle, contains dongle response on return.
 * @param len          Maximum number of bytes that dongle is allowed to write into 'buf'.
 */
int
dhd_wl_ioctl(dhd_pub_t *dhd_pub, int ifidx, wl_ioctl_t *ioc, void *buf, int len)
{
	int ret = BCME_ERROR;
	unsigned long flags;
#ifdef DUMP_IOCTL_IOV_LIST
	dhd_iov_li_t *iov_li;
#endif /* DUMP_IOCTL_IOV_LIST */

	if (dhd_query_bus_erros(dhd_pub)) {
		return -ENODEV;
	}

#ifdef DHD_PCIE_NATIVE_RUNTIMEPM
	DHD_OS_WAKE_LOCK(dhd_pub);
	if (pm_runtime_get_sync(dhd_bus_to_dev(dhd_pub->bus)) < 0) {
		DHD_RPM(("%s: pm_runtime_get_sync error. \n", __FUNCTION__));
		DHD_OS_WAKE_UNLOCK(dhd_pub);
		return BCME_ERROR;
	}
#endif /* DHD_PCIE_NATIVE_RUNTIMEPM */

#ifdef KEEPIF_ON_DEVICE_RESET
	if (ioc->cmd == WLC_GET_VAR) {
		dbus_config_t config;
		config.general_param = 0;
		if (buf) {
			if (!strcmp(buf, "wowl_activate")) {
				 /* 1 (TRUE) after decreased by 1 */
				config.general_param = 2;
			} else if (!strcmp(buf, "wowl_clear")) {
				 /* 0 (FALSE) after decreased by 1 */
				config.general_param = 1;
			}
		}
		if (config.general_param) {
			config.config_id = DBUS_CONFIG_ID_KEEPIF_ON_DEVRESET;
			config.general_param--;
			dbus_set_config(dhd_pub->dbus, &config);
		}
	}
#endif /* KEEPIF_ON_DEVICE_RESET */

	if (dhd_os_proto_block(dhd_pub)) {
#ifdef DHD_LOG_DUMP
		int slen, val, lval, min_len;
		char *msg, tmp[64];

		/* WLC_GET_VAR */
		if (ioc->cmd == WLC_GET_VAR && buf) {
			min_len = MIN(sizeof(tmp) - 1, strlen(buf));
			memset(tmp, 0, sizeof(tmp));
			bcopy(buf, tmp, min_len);
			tmp[min_len] = '\0';
		}
#endif /* DHD_LOG_DUMP */

#ifdef DHD_DISCONNECT_TRACE
		if (WLC_DISASSOC == ioc->cmd || WLC_DOWN == ioc->cmd ||
			WLC_DISASSOC_MYAP == ioc->cmd) {
			DHD_ERROR(("IOCTL Disconnect WiFi: %d\n", ioc->cmd));
		}
#endif /* HW_DISCONNECT_TRACE */
		/* logging of iovars that are send to the dongle, ./dhd msglevel +iovar */
		if (ioc->set == TRUE) {
			char *pars = (char *)buf; /* points at user buffer */
			if (ioc->cmd == WLC_SET_VAR && buf) {
				DHD_DNGL_IOVAR_SET(("iovar:%d: set %s", ifidx, pars));
				if (ioc->len > 1 + sizeof(uint32)) {
					/* skip iovar name */
					pars += strnlen(pars, ioc->len - 1 - sizeof(uint32));
					/* skip NULL character */
					pars++;
				}
			} else {
				DHD_DNGL_IOVAR_SET(("ioctl:%d: set %d %s",
					ifidx, ioc->cmd, ioctl2str(ioc->cmd)));
			}
			if (pars != NULL) {
				DHD_DNGL_IOVAR_SET((" 0x%x\n", *(uint32*)pars));
			} else {
				DHD_DNGL_IOVAR_SET((" NULL\n"));
			}
		}

		DHD_LINUX_GENERAL_LOCK(dhd_pub, flags);
		if (DHD_BUS_CHECK_DOWN_OR_DOWN_IN_PROGRESS(dhd_pub)) {
			DHD_INFO(("%s: returning as busstate=%d\n",
				__FUNCTION__, dhd_pub->busstate));
			DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);
			dhd_os_proto_unblock(dhd_pub);
			return -ENODEV;
		}
		DHD_BUS_BUSY_SET_IN_IOVAR(dhd_pub);
		DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);

#ifdef DHD_PCIE_RUNTIMEPM
		dhdpcie_runtime_bus_wake(dhd_pub, TRUE, dhd_wl_ioctl);
#endif /* DHD_PCIE_RUNTIMEPM */

		DHD_LINUX_GENERAL_LOCK(dhd_pub, flags);
		if (DHD_BUS_CHECK_SUSPEND_OR_ANY_SUSPEND_IN_PROGRESS(dhd_pub) ||
			dhd_pub->dhd_induce_error == DHD_INDUCE_IOCTL_SUSPEND_ERROR) {
			DHD_ERROR(("%s: bus is in suspend(%d) or suspending(0x%x) state!!\n",
				__FUNCTION__, dhd_pub->busstate, dhd_pub->dhd_bus_busy_state));
#ifdef DHD_SEND_HANG_IOCTL_SUSPEND_ERROR
			ioctl_suspend_error++;
			if (ioctl_suspend_error > MAX_IOCTL_SUSPEND_ERROR) {
				dhd_pub->hang_reason = HANG_REASON_IOCTL_SUSPEND_ERROR;
				dhd_os_send_hang_message(dhd_pub);
				ioctl_suspend_error = 0;
			}
#endif /* DHD_SEND_HANG_IOCTL_SUSPEND_ERROR */
			DHD_BUS_BUSY_CLEAR_IN_IOVAR(dhd_pub);
			dhd_os_busbusy_wake(dhd_pub);
			DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);
			dhd_os_proto_unblock(dhd_pub);
			return -ENODEV;
		}
#ifdef DHD_SEND_HANG_IOCTL_SUSPEND_ERROR
		ioctl_suspend_error = 0;
#endif /* DHD_SEND_HANG_IOCTL_SUSPEND_ERROR */
		DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);

#ifdef DUMP_IOCTL_IOV_LIST
		if (ioc->cmd != WLC_GET_MAGIC && ioc->cmd != WLC_GET_VERSION && buf) {
			if (!(iov_li = MALLOC(dhd_pub->osh, sizeof(*iov_li)))) {
				DHD_ERROR(("iovar dump list item allocation Failed\n"));
			} else {
				iov_li->cmd = ioc->cmd;
				if (buf)
					bcopy((char *)buf, iov_li->buff, strlen((char *)buf)+1);
				dhd_iov_li_append(dhd_pub, &dhd_pub->dump_iovlist_head,
						&iov_li->list);
			}
		}
#endif /* DUMP_IOCTL_IOV_LIST */

		ret = dhd_prot_ioctl(dhd_pub, ifidx, ioc, buf, len);

#ifdef DUMP_IOCTL_IOV_LIST
		if (ret == -ETIMEDOUT) {
			DHD_ERROR(("Last %d issued commands: Latest one is at bottom.\n",
				IOV_LIST_MAX_LEN));
			dhd_iov_li_print(&dhd_pub->dump_iovlist_head);
		}
#endif /* DUMP_IOCTL_IOV_LIST */
#ifdef WL_CFGVENDOR_SEND_HANG_EVENT
		if (ret == -ETIMEDOUT) {
			copy_hang_info_ioctl_timeout(dhd_pub, ifidx, ioc);
		}
#endif /* WL_CFGVENDOR_SEND_HANG_EVENT */
#ifdef DHD_LOG_DUMP
		if ((ioc->cmd == WLC_GET_VAR || ioc->cmd == WLC_SET_VAR) &&
				buf != NULL) {
			if (buf) {
				lval = 0;
				slen = strlen(buf) + 1;
				msg = (char*)buf;
				if (len >= slen + sizeof(lval)) {
					if (ioc->cmd == WLC_GET_VAR) {
						msg = tmp;
						lval = *(int*)buf;
					} else {
						min_len = MIN(ioc->len - slen, sizeof(int));
						bcopy((msg + slen), &lval, min_len);
					}
					if (!strncmp(msg, "cur_etheraddr",
						strlen("cur_etheraddr"))) {
						lval = 0;
					}
				}
				DHD_IOVAR_MEM((
					"%s: cmd: %d, msg: %s val: 0x%x,"
					" len: %d, set: %d, txn-id: %d\n",
					ioc->cmd == WLC_GET_VAR ?
					"WLC_GET_VAR" : "WLC_SET_VAR",
					ioc->cmd, msg, lval, ioc->len, ioc->set,
					dhd_prot_get_ioctl_trans_id(dhd_pub)));
			} else {
				DHD_IOVAR_MEM(("%s: cmd: %d, len: %d, set: %d, txn-id: %d\n",
					ioc->cmd == WLC_GET_VAR ? "WLC_GET_VAR" : "WLC_SET_VAR",
					ioc->cmd, ioc->len, ioc->set,
					dhd_prot_get_ioctl_trans_id(dhd_pub)));
			}
		} else {
			slen = ioc->len;
			if (buf != NULL && slen != 0) {
				if (slen >= 4) {
					val = *(int*)buf;
				} else if (slen >= 2) {
					val = *(short*)buf;
				} else {
					val = *(char*)buf;
				}
				/* Do not dump for WLC_GET_MAGIC and WLC_GET_VERSION */
				if (ioc->cmd != WLC_GET_MAGIC && ioc->cmd != WLC_GET_VERSION) {
					DHD_IOVAR_MEM(("WLC_IOCTL: cmd: %d, val: %d, len: %d, "
						"set: %d\n", ioc->cmd, val, ioc->len, ioc->set));
				}
			} else {
				DHD_IOVAR_MEM(("WLC_IOCTL: cmd: %d, buf is NULL\n", ioc->cmd));
			}
		}
#endif /* DHD_LOG_DUMP */
#if defined(OEM_ANDROID)
		if ((0 > ret) && dhd_pub->up) {
			/* Send hang event only if dhd_open() was success */
			dhd_os_check_hang(dhd_pub, ifidx, ret);
		}

		if (ret == -ETIMEDOUT && !dhd_pub->up) {
			DHD_ERROR(("%s: 'resumed on timeout' error is "
				"occurred before the interface does not"
				" bring up\n", __FUNCTION__));
		}
#endif /* defined(OEM_ANDROID) */

		DHD_LINUX_GENERAL_LOCK(dhd_pub, flags);
		DHD_BUS_BUSY_CLEAR_IN_IOVAR(dhd_pub);
		dhd_os_busbusy_wake(dhd_pub);
		DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);

		dhd_os_proto_unblock(dhd_pub);

		if (ret < 0) {
			if ((ioc->cmd == WLC_GET_VAR || ioc->cmd == WLC_SET_VAR) &&
					buf != NULL) {
				if (ret == BCME_UNSUPPORTED || ret == BCME_NOTASSOCIATED) {
					DHD_ERROR_MEM(("%s: %s: %s, %s\n",
						__FUNCTION__, ioc->cmd == WLC_GET_VAR ?
						"WLC_GET_VAR" : "WLC_SET_VAR",
						buf? (char *)buf:"NO MESSAGE",
						ret == BCME_UNSUPPORTED ? "UNSUPPORTED"
						: "NOT ASSOCIATED"));
				} else {
					DHD_ERROR_MEM(("%s: %s: %s, ret = %d\n",
						__FUNCTION__, ioc->cmd == WLC_GET_VAR ?
						"WLC_GET_VAR" : "WLC_SET_VAR",
						(char *)buf, ret));
				}
			} else {
				if (ret == BCME_UNSUPPORTED || ret == BCME_NOTASSOCIATED) {
					DHD_ERROR_MEM(("%s: WLC_IOCTL: cmd: %d, %s\n",
						__FUNCTION__, ioc->cmd,
						ret == BCME_UNSUPPORTED ? "UNSUPPORTED" :
						"NOT ASSOCIATED"));
				} else {
					DHD_ERROR_MEM(("%s: WLC_IOCTL: cmd: %d, ret = %d\n",
						__FUNCTION__, ioc->cmd, ret));
				}
			}
		}
	}

#ifdef DHD_PCIE_NATIVE_RUNTIMEPM
	pm_runtime_mark_last_busy(dhd_bus_to_dev(dhd_pub->bus));
	pm_runtime_put_autosuspend(dhd_bus_to_dev(dhd_pub->bus));

	DHD_OS_WAKE_UNLOCK(dhd_pub);
#endif /* DHD_PCIE_NATIVE_RUNTIMEPM */

	return ret;
}

uint wl_get_port_num(wl_io_pport_t *io_pport)
{
	return 0;
}

/* Get bssidx from iovar params
 * Input:   dhd_pub - pointer to dhd_pub_t
 *	    params  - IOVAR params
 * Output:  idx	    - BSS index
 *	    val	    - ponter to the IOVAR arguments
 */
static int
dhd_iovar_parse_bssidx(dhd_pub_t *dhd_pub, const char *params, uint32 *idx, const char **val)
{
	char *prefix = "bsscfg:";
	uint32	bssidx;

	if (!(strncmp(params, prefix, strlen(prefix)))) {
		/* per bss setting should be prefixed with 'bsscfg:' */
		const char *p = params + strlen(prefix);

		/* Skip Name */
		while (*p != '\0')
			p++;
		/* consider null */
		p = p + 1;
		bcopy(p, &bssidx, sizeof(uint32));
		/* Get corresponding dhd index */
		bssidx = dhd_bssidx2idx(dhd_pub, htod32(bssidx));

		if (bssidx >= DHD_MAX_IFS) {
			DHD_ERROR(("%s Wrong bssidx provided\n", __FUNCTION__));
			return BCME_ERROR;
		}

		/* skip bss idx */
		p += sizeof(uint32);
		*val = p;
		*idx = bssidx;
	} else {
		DHD_ERROR(("%s: bad parameter for per bss iovar\n", __FUNCTION__));
		return BCME_ERROR;
	}

	return BCME_OK;
}

#if defined(DHD_DEBUG) && defined(BCMDHDUSB)
/* USB Device console input function */
int dhd_bus_console_in(dhd_pub_t *dhd, uchar *msg, uint msglen)
{
	DHD_TRACE(("%s \n", __FUNCTION__));

	return dhd_iovar(dhd, 0, "cons", msg, msglen, NULL, 0, TRUE);

}
#endif /* DHD_DEBUG && BCMDHDUSB  */

#ifdef DHD_DEBUG
int
dhd_mem_debug(dhd_pub_t *dhd, uchar *msg, uint msglen)
{
	unsigned long int_arg = 0;
	char *p;
	char *end_ptr = NULL;
	dhd_dbg_mwli_t *mw_li;
	dll_t *item, *next;
	/* check if mwalloc, mwquery or mwfree was supplied arguement with space */
	p = bcmstrstr((char *)msg, " ");
	if (p != NULL) {
		/* space should be converted to null as separation flag for firmware */
		*p = '\0';
		/* store the argument in int_arg */
		int_arg = bcm_strtoul(p+1, &end_ptr, 10);
	}

	if (!p && !strcmp(msg, "query")) {
		/* lets query the list inetrnally */
		if (dll_empty(dll_head_p(&dhd->mw_list_head))) {
			DHD_ERROR(("memwaste list is empty, call mwalloc < size > to allocate\n"));
		} else {
			for (item = dll_head_p(&dhd->mw_list_head);
					!dll_end(&dhd->mw_list_head, item); item = next) {
				next = dll_next_p(item);
				mw_li = (dhd_dbg_mwli_t *)CONTAINEROF(item, dhd_dbg_mwli_t, list);
				DHD_ERROR(("item: <id=%d, size=%d>\n", mw_li->id, mw_li->size));
			}
		}
	} else if (p && end_ptr && (*end_ptr == '\0') && !strcmp(msg, "alloc")) {
		int32 alloc_handle;
		/* convert size into KB and append as integer */
		*((int32 *)(p+1)) = int_arg*1024;
		*(p+1+sizeof(int32)) = '\0';

		/* recalculated length -> 5 bytes for "alloc" + 4 bytes for size +
		 * 1 bytes for null caracter
		 */
		msglen = strlen(msg) + sizeof(int32) + 1;
		if (dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, msg, msglen+1, FALSE, 0) < 0) {
			DHD_ERROR(("IOCTL failed for memdebug alloc\n"));
		}

		/* returned allocated handle from dongle, basically address of the allocated unit */
		alloc_handle = *((int32 *)msg);

		/* add a node in the list with tuple <id, handle, size> */
		if (alloc_handle == 0) {
			DHD_ERROR(("Reuqested size could not be allocated\n"));
		} else if (!(mw_li = MALLOC(dhd->osh, sizeof(*mw_li)))) {
			DHD_ERROR(("mw list item allocation Failed\n"));
		} else {
			mw_li->id = dhd->mw_id++;
			mw_li->handle = alloc_handle;
			mw_li->size = int_arg;
			/* append the node in the list */
			dll_append(&dhd->mw_list_head, &mw_li->list);
		}
	} else if (p && end_ptr && (*end_ptr == '\0') && !strcmp(msg, "free")) {
		/* inform dongle to free wasted chunk */
		int handle = 0;
		int size = 0;
		for (item = dll_head_p(&dhd->mw_list_head);
				!dll_end(&dhd->mw_list_head, item); item = next) {
			next = dll_next_p(item);
			mw_li = (dhd_dbg_mwli_t *)CONTAINEROF(item, dhd_dbg_mwli_t, list);

			if (mw_li->id == (int)int_arg) {
				handle = mw_li->handle;
				size = mw_li->size;
				dll_delete(item);
				MFREE(dhd->osh, mw_li, sizeof(*mw_li));
				if (dll_empty(dll_head_p(&dhd->mw_list_head))) {
					/* reset the id */
					dhd->mw_id = 0;
				}
			}
		}
		if (handle) {
			int len;
			/* append the free handle and the chunk size in first 8 bytes
			 * after the command and null character
			 */
			*((int32 *)(p+1)) = handle;
			*((int32 *)((p+1)+sizeof(int32))) = size;
			/* append null as terminator */
			*(p+1+2*sizeof(int32)) = '\0';
			/* recalculated length -> 4 bytes for "free" + 8 bytes for hadnle and size
			 * + 1 bytes for null caracter
			 */
			len = strlen(msg) + 2*sizeof(int32) + 1;
			/* send iovar to free the chunk */
			if (dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, msg, len, FALSE, 0) < 0) {
				DHD_ERROR(("IOCTL failed for memdebug free\n"));
			}
		} else {
			DHD_ERROR(("specified id does not exist\n"));
		}
	} else {
		/* for all the wrong argument formats */
		return BCME_BADARG;
	}
	return 0;
}
extern void
dhd_mw_list_delete(dhd_pub_t *dhd, dll_t *list_head)
{
	dll_t *item;
	dhd_dbg_mwli_t *mw_li;
	while (!(dll_empty(list_head))) {
		item = dll_head_p(list_head);
		mw_li = (dhd_dbg_mwli_t *)CONTAINEROF(item, dhd_dbg_mwli_t, list);
		dll_delete(item);
		MFREE(dhd->osh, mw_li, sizeof(*mw_li));
	}
}
#ifdef BCMPCIE
int
dhd_flow_ring_debug(dhd_pub_t *dhd, char *msg, uint msglen)
{
	flow_ring_table_t *flow_ring_table;
	char *cmd;
	char *end_ptr = NULL;
	uint8 prio;
	uint16 flowid;
	int i;
	int ret = 0;
	cmd = bcmstrstr(msg, " ");
	BCM_REFERENCE(prio);
	if (cmd != NULL) {
		/* in order to use string operations append null */
		*cmd = '\0';
	} else {
		DHD_ERROR(("missing: create/delete args\n"));
		return BCME_ERROR;
	}
	if (cmd && !strcmp(msg, "create")) {
		/* extract <"source address", "destination address", "priority"> */
		uint8 sa[ETHER_ADDR_LEN], da[ETHER_ADDR_LEN];
		BCM_REFERENCE(sa);
		BCM_REFERENCE(da);
		msg = msg + strlen("create") + 1;
		/* fill ethernet source address */
		for (i = 0; i < ETHER_ADDR_LEN; i++) {
			sa[i] = (uint8)bcm_strtoul(msg, &end_ptr, 16);
			if (*end_ptr == ':') {
				msg = (end_ptr + 1);
			} else if (i != 5) {
				DHD_ERROR(("not a valid source mac addr\n"));
				return BCME_ERROR;
			}
		}
		if (*end_ptr != ' ') {
			DHD_ERROR(("missing: destiantion mac id\n"));
			return BCME_ERROR;
		} else {
			/* skip space */
			msg = end_ptr + 1;
		}
		/* fill ethernet destination address */
		for (i = 0; i < ETHER_ADDR_LEN; i++) {
			da[i] = (uint8)bcm_strtoul(msg, &end_ptr, 16);
			if (*end_ptr == ':') {
				msg = (end_ptr + 1);
			} else if (i != 5) {
				DHD_ERROR(("not a valid destination  mac addr\n"));
				return BCME_ERROR;
			}
		}
		if (*end_ptr != ' ') {
			DHD_ERROR(("missing: priority\n"));
			return BCME_ERROR;
		} else {
			msg = end_ptr + 1;
		}
		/* parse priority */
		prio = (uint8)bcm_strtoul(msg, &end_ptr, 10);
		if (prio > MAXPRIO) {
			DHD_ERROR(("%s: invalid priority. Must be between 0-7 inclusive\n",
				__FUNCTION__));
			return BCME_ERROR;
		}

		if (*end_ptr != '\0') {
			DHD_ERROR(("msg not truncated with NULL character\n"));
			return BCME_ERROR;
		}
		ret = dhd_flowid_debug_create(dhd, 0, prio, (char *)sa, (char *)da, &flowid);
		if (ret != BCME_OK) {
			DHD_ERROR(("%s: flowring creation failed ret: %d\n", __FUNCTION__, ret));
			return BCME_ERROR;
		}
		return BCME_OK;

	} else if (cmd && !strcmp(msg, "delete")) {
		msg = msg + strlen("delete") + 1;
		/* parse flowid */
		flowid = (uint16)bcm_strtoul(msg, &end_ptr, 10);
		if (*end_ptr != '\0') {
			DHD_ERROR(("msg not truncated with NULL character\n"));
			return BCME_ERROR;
		}

		/* Find flowid from ifidx 0 since this IOVAR creating flowring with ifidx 0 */
		if (dhd_flowid_find_by_ifidx(dhd, 0, flowid) != BCME_OK)
		{
			DHD_ERROR(("%s : Deleting not created flowid: %u\n", __FUNCTION__, flowid));
			return BCME_ERROR;
		}

		flow_ring_table = (flow_ring_table_t *)dhd->flow_ring_table;
		ret = dhd_bus_flow_ring_delete_request(dhd->bus, (void *)&flow_ring_table[flowid]);
		if (ret != BCME_OK) {
			DHD_ERROR(("%s: flowring deletion failed ret: %d\n", __FUNCTION__, ret));
			return BCME_ERROR;
		}
		return BCME_OK;
	}
	DHD_ERROR(("%s: neither create nor delete\n", __FUNCTION__));
	return BCME_ERROR;
}
#endif /* BCMPCIE */
#endif /* DHD_DEBUG */

#ifdef SYNA_SAR_CUSTOMER_PARAMETER

typedef struct _dhd_sar_parameters {
	dhd_sar_parameter  other;
	dhd_sar_parameter  fcc;
	dhd_sar_parameter  eu;
	dhd_sar_parameter  latam;
} dhd_sar_parameters;

static dhd_sar_parameters  gSAR_params;

/* pick up usable parameters */
static sarctrl_set *dhd_sar_ctrlset_get(
		dhd_sar_parameter *dhd_sar_param, int advance_mode)
{
	sarctrl_set *sarctrl = NULL;

	if (NULL == dhd_sar_param) {
		DHD_ERROR(("%s: *invalid argument, sar_param=0x%p\n",
		           __FUNCTION__, dhd_sar_param));
	} else switch (advance_mode) {
		case SAR_HEAD:
			sarctrl = &dhd_sar_param->dynamic;
			DHD_TRACE(("%s: dynamic=0x%p, sarctrl=0x%p, ver=%d,"
			           " sarctrl_2g=0x%X, sarctrl_2g_2=0x%X\n",
			           __FUNCTION__, dhd_sar_param, sarctrl, sarctrl->ver,
			           sarctrl->basic.sarctrl_2g, sarctrl->basic.sarctrl_2g_2));
			break;
		case SAR_GRIP:
			sarctrl = &dhd_sar_param->grip;
			DHD_TRACE(("%s: grip=0x%p, sarctrl=0x%p, ver=%d,"
			           " sarctrl_2g=0x%X, sarctrl_2g_2=0x%X\n",
			           __FUNCTION__, dhd_sar_param, sarctrl, sarctrl->ver,
			           sarctrl->basic.sarctrl_2g, sarctrl->basic.sarctrl_2g_2));
			break;
		case SAR_BT:
			sarctrl = &dhd_sar_param->bt;
			DHD_TRACE(("%s: bt=0x%p, sarctrl=0x%p, ver=%d,"
			           " sarctrl_2g=0x%X, sarctrl_2g_2=0x%X\n",
			           __FUNCTION__, dhd_sar_param, sarctrl, sarctrl->ver,
			           sarctrl->basic.sarctrl_2g, sarctrl->basic.sarctrl_2g_2));
			break;
		case SAR_HOTSPOT:
			sarctrl = &dhd_sar_param->hotspot;
			DHD_TRACE(("%s: hotspot=0x%p, sarctrl=0x%p, ver=%d,"
			           " sarctrl_2g=0x%X, sarctrl_2g_2=0x%X\n",
			           __FUNCTION__, dhd_sar_param, sarctrl, sarctrl->ver,
			           sarctrl->basic.sarctrl_2g, sarctrl->basic.sarctrl_2g_2));
			break;
		default:
			DHD_ERROR(("%s: *Error, invalid advance_mode=%d\n",
			           __FUNCTION__, advance_mode));
	}

	return sarctrl;
}

int dhd_sar_init_parameter(dhd_pub_t *dhd, eCountry_flag_type type, int advance_mode,
	char *list_str)
{
	char          temp_str[16];
	sarctrl_set  *sarctrl = NULL;
	uint32       *txpwr_list = NULL;
	char         *next = NULL;
	int          len = 0, idx = 0;

	if (NULL == list_str) {
		DHD_ERROR(("%s: *Error, invalid parameber\n", __FUNCTION__));
		return BCME_BADARG;
	} else {
		next = list_str;
	}

	switch (type) {
		case SYNA_COUNTRY_TYPE_FCC:
			sarctrl = dhd_sar_ctrlset_get(&gSAR_params.fcc, advance_mode);
			break;
		case SYNA_COUNTRY_TYPE_EU:
			sarctrl = dhd_sar_ctrlset_get(&gSAR_params.eu, advance_mode);
			break;
		case SYNA_COUNTRY_TYPE_LATAM:
			sarctrl = dhd_sar_ctrlset_get(&gSAR_params.latam, advance_mode);
			break;
		case SYNA_COUNTRY_TYPE_DEFAULT:
			sarctrl = dhd_sar_ctrlset_get(&gSAR_params.other, advance_mode);
			break;
		default:
			DHD_ERROR(("%s: *Error, invalid type=%d\n", __FUNCTION__, type));
			return BCME_UNSUPPORTED;
			break;
	}
	if (!sarctrl) {
		DHD_ERROR(("%s: *Error, invalid advance_mode=%d\n",
		           __FUNCTION__, advance_mode));
		return BCME_UNSUPPORTED;
	}

	/* change dhd_sar_parameter structure to uint32 array.
	 * use txpwr_list pointer to fill the data
	 * one by one in following loop code
	 */
	txpwr_list = (uint32 *)(&sarctrl->basic);
	idx = 0;
	while ((len = strcspn(next, " ,")) > 0) {
		if (len >= sizeof(temp_str)) {
			DHD_ERROR(("%s: init '%s' before ','/' ' is too long\n",
			           __FUNCTION__, next));
			return BCME_ERROR;
		}
		strlcpy(temp_str, next, sizeof(temp_str));
		temp_str[len] = 0;
		txpwr_list[idx] = (uint32)simple_strtoul(temp_str, NULL, 0);
		DHD_TRACE(("%s: get txpwr[%d]=0x%x\n",
		           __FUNCTION__, idx, txpwr_list[idx]));

		if (idx == CONST_SARCTRL_SET_QTY) {
			DHD_ERROR(("%s: too many parameters(more than %d): '%s'\n",
			           __FUNCTION__, idx, list_str));
			break;
		}

		idx++;
		next += len;
		next += strspn(next, " ,");
	}

#ifdef VSDB
	if (FW_SUPPORTED(dhd, 6g)) {
		sarctrl->ver = SAR_PARAM_6G_V3_VSDB | (idx << SAR_PARAM_NUM_OFFSET);
	} else {
		sarctrl->ver = SAR_PARAM_V1_VSDB | (idx << SAR_PARAM_NUM_OFFSET);
	}
	if (CONST_SARCTRL_SINGLE_SET_QTY != idx) {
		DHD_ERROR(("%s: *Waring, idx=%d mismatch VSDB expect=%d\n",
		           __FUNCTION__, idx, (int)CONST_SARCTRL_SINGLE_SET_QTY));
	}
#else /* VSDB */
	if (FW_SUPPORTED(dhd, 6g)) {
		sarctrl->ver = SAR_PARAM_6G_V4_RSDB | (idx << SAR_PARAM_NUM_OFFSET);
	} else {
		sarctrl->ver = SAR_PARAM_V2_RSDB | (idx << SAR_PARAM_NUM_OFFSET);
	}
	if (CONST_SARCTRL_SET_QTY != idx) {
		DHD_ERROR(("%s: *Waring, idx=%d mismatch RSDB expect=%d\n",
		           __FUNCTION__, idx, (int)CONST_SARCTRL_SET_QTY));
	}
#endif /* VSDB */

	return 0;
}

int dhd_sar_reset_parameter(void)
{
	bzero(&gSAR_params, sizeof(gSAR_params));

	return BCME_OK;
}

int dhd_sar_set_parameter(dhd_pub_t *dhd_pub, int advance_mode)
{
	int                 err = BCME_OK;
	char                iovbuf[WLC_IOCTL_SMLEN];
	wl_country_t       *cspec = NULL;
	sarctrl_set         sarctrl_iov = { 0 }, *sarctrl = NULL;
	eCountry_flag_type  type = SYNA_COUNTRY_TYPE_INVALID;
	int sar_ver = 0, sar_param_num = 0;

	if (NULL == dhd_pub) {
		DHD_ERROR(("%s: bcm_mkiovar failed.", __FUNCTION__));
		return BCME_BADARG;
	} else if (SAR_DISABLE == advance_mode) {
		DHD_ERROR(("%s: SKIP since disabled.", __FUNCTION__));
		return BCME_OK;
	}

	memset(iovbuf, 0, sizeof(iovbuf));
	if (0 >= bcm_mkiovar("country", NULL, 0, iovbuf, sizeof(iovbuf))) {
		err = BCME_BUFTOOSHORT;
		DHD_ERROR(("%s: sar country mkiovar failed.", __FUNCTION__));
		return err;
	}
	err = dhd_wl_ioctl_cmd(dhd_pub, WLC_GET_VAR, iovbuf, sizeof(iovbuf), FALSE, 0);
	if (err) {
		DHD_ERROR(("%s: country code get failed\n", __FUNCTION__));
		return err;
	}
	cspec = (wl_country_t *)iovbuf;
	type = syna_country_check_type(cspec->ccode);
	switch (type) {
		case SYNA_COUNTRY_TYPE_FCC:
			sarctrl = dhd_sar_ctrlset_get(&gSAR_params.fcc, advance_mode);
			break;
		case SYNA_COUNTRY_TYPE_EU:
			sarctrl = dhd_sar_ctrlset_get(&gSAR_params.eu, advance_mode);
			break;
		case SYNA_COUNTRY_TYPE_LATAM:
			sarctrl = dhd_sar_ctrlset_get(&gSAR_params.latam, advance_mode);
			break;
		default:
			sarctrl = dhd_sar_ctrlset_get(&gSAR_params.other, advance_mode);
			break;
	}
	/* check if parameters valid */
	if ((!sarctrl) || (!sarctrl->ver)) {
		DHD_ERROR(("%s: *warning, try default for country='%s'(type=%d)\n",
		           __FUNCTION__, cspec->ccode, type));
		sarctrl = dhd_sar_ctrlset_get(&gSAR_params.other, advance_mode);
		if ((!sarctrl) || (!sarctrl->ver)) {
			/* stop/block setting if customer not set parameters */
			DHD_ERROR(("%s: *skip since parameter not set\n", __FUNCTION__));
			return BCME_UNSUPPORTED;
		}
	}
	DHD_TRACE(("%s: country='%s', type=%d, advance_mode=%d\n",
	           __FUNCTION__, cspec->ccode, type, advance_mode));
	/* Try to get sar version from FW */
	err = dhd_iovar(dhd_pub, 0, "sar_params", NULL, 0,
		(char *)&sarctrl_iov, sizeof(sarctrl_iov), FALSE);
	if (err) {
		/* Not support sar_params read */
	} else {
		sar_ver = sarctrl_iov.ver & SAR_PARAM_VER_MASK;
		sar_param_num = (sarctrl_iov.ver & SAR_PARAM_NUM_MASK) >> SAR_PARAM_NUM_OFFSET;
	}

	memset(&sarctrl_iov, 0, sizeof(sarctrl_iov));
	if (sar_ver == SAR_PARAM_DYN) {
		int file_sar_num = (sarctrl->ver & SAR_PARAM_NUM_MASK) >> SAR_PARAM_NUM_OFFSET;
		if (file_sar_num  != sar_param_num) {
			DHD_ERROR(("%s: Need confirm SAR PARAMETERS configuration based on FW\n",
				__FUNCTION__));
			return BCME_BADARG;
		}
		sarctrl_iov.ver = sarctrl->ver;
		err = memcpy_s(sarctrl_iov.sarctrl, MAX_SAR_PARAMS_NUM * sizeof(u32),
			sarctrl->sarctrl, file_sar_num * sizeof(u32));
		if (err) {
			DHD_ERROR(("%s: memcpy error num = %d\n",
				__FUNCTION__, file_sar_num));
			return BCME_BADARG;
		}
	} else {
		/* prepare IOVAR for old version*/
		sarctrl_iov.ver   = sarctrl->ver & SAR_PARAM_VER_MASK;
		sarctrl_iov.basic = sarctrl->basic;
		sarctrl_iov.rsdb  = sarctrl->rsdb;
	}
	DHD_TRACE(("%s: sarctrl_iov=0x%p, ver=%x, sarctrl_2g=0x%X, sarctrl_2g_2=0x%X\n",
	           __FUNCTION__, &sarctrl_iov, sarctrl_iov.ver,
	           sarctrl_iov.basic.sarctrl_2g,
	           sarctrl_iov.basic.sarctrl_2g_2));
	err = dhd_iovar(dhd_pub, 0, "sar_params", (char *)&sarctrl_iov,
		sizeof(sarctrl_iov), NULL, 0, TRUE);
	if (unlikely(err)) {
		DHD_ERROR(("%s: Failed to set sar_params - error (%d)\n",
		           __FUNCTION__, err));
	}

	return err;
}

int dhd_sar_set(dhd_pub_t *dhd_pub, int sar_val)
{
	sar_advance_modes sar_tx_power_val = SAR_DISABLE;
	int airplane_mode = 0;
	int err = 0;

	/* Map Android TX power modes to power mode */
	switch (sar_val) {
		case SAR_OFF:
			/* SAR disabled */
			sar_tx_power_val = SAR_DISABLE;
			airplane_mode = 0;
			break;
		case HEAD_NORMAL:
			/* HEAD mode, Normal */
			sar_tx_power_val = SAR_HEAD;
			airplane_mode = 0;
			break;
		case HEAD_AIRPLANE:
			/* HEAD mode, Airplane */
			sar_tx_power_val = SAR_HEAD;
			airplane_mode = 1;
			break;
		case GRIP_NORMAL:
			/* GRIP mode, Normal */
			sar_tx_power_val = SAR_GRIP;
			airplane_mode = 0;
			break;
		case GRIP_AIRPLANE:
			/* GRIP mode, Airplane */
			sar_tx_power_val = SAR_GRIP;
			airplane_mode = 1;
			break;
		case BT_NORMAL:
			/* BT mode, Normal */
			sar_tx_power_val = SAR_BT;
			airplane_mode = 0;
			break;
		case BT_AIRPLANE:
			/* BT mode, Airplane */
			sar_tx_power_val = SAR_BT;
			airplane_mode = 1;
			break;
		case HOTSPOT_NORMAL:
			/* HOTSPOT mode, Normal */
			sar_tx_power_val = SAR_HOTSPOT;
			airplane_mode = 0;
			break;
		case HOTSPOT_AIRPLANE:
			/* HOTSPOT mode, AIRPLANE */
			sar_tx_power_val = SAR_HOTSPOT;
			airplane_mode = 1;
			break;
		default:
			DHD_ERROR(("%s: invalid wifi tx power scenario = %d\n",
			        __FUNCTION__, sar_tx_power_val));
			err = -EINVAL;
			goto exit;
	}

	if (sar_tx_power_val) {
		err = dhd_sar_set_parameter(dhd_pub, sar_tx_power_val);
		if (!err) {
			/* If dhd and FW all support this feature, it will return 0
			 * DHD already set sar parameter to FW.
			 * so only need to set sar_enable 1 to enable SAR
			 */
			sar_tx_power_val = 1;
		} else if (err == BCME_UNSUPPORTED) {
			/* If dhd or FW not support/set these parameters
			 * use traditional way
			 */
			DHD_TRACE(("DHD/FW not support sar_param,"
					"try to use SAR parameters in nvram"));
		} else {
			DHD_ERROR(("%s: Failed to set sar_params - error (%d)\n",
			           __FUNCTION__, err));
			goto exit;
		}
	}

	DHD_TRACE(("%s: sar_mode %d airplane_mode %d\n",
	           __FUNCTION__, sar_tx_power_val, airplane_mode));
	err = dhd_iovar(dhd_pub, 0, "fccpwrlimit2g", (char *)&airplane_mode,
	                sizeof(airplane_mode), NULL, 0, TRUE);
	if (unlikely(err)) {
		DHD_ERROR(("%s: Failed to set airplane_mode - error (%d)\n",
		           __FUNCTION__, err));
		goto exit;
	}
	err = dhd_iovar(dhd_pub, 0, "sar_enable", (char *)&sar_tx_power_val,
	                sizeof(sar_tx_power_val), NULL, 0, TRUE);
	if (unlikely(err)) {
		DHD_ERROR(("%s: Failed to set sar_enable - error (%d)\n",
		           __FUNCTION__, err));
		goto exit;
	}

	/* Cache the tx power mode sent by the hal */
	dhd_pub->dhd_sar_mode = sar_val;
	DHD_TRACE(("%s: tx_power_mode %d SUCCESS\n", __FUNCTION__, sar_val));

exit:
	return err;

}

#endif /* SYNA_SAR_CUSTOMER_PARAMETER */

static int
dhd_doiovar(dhd_pub_t *dhd_pub, const bcm_iovar_t *vi, uint32 actionid, const char *name,
            void *params, int plen, void *arg, uint len, int val_size)
{
	int bcmerror = 0;
	int32 int_val = 0;
	uint32 dhd_ver_len, bus_api_rev_len;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));
	DHD_TRACE(("%s: actionid = %d; name %s\n", __FUNCTION__, actionid, name));

	if ((bcmerror = bcm_iovar_lencheck(vi, arg, len, IOV_ISSET(actionid))) != 0)
		goto exit;

	if (plen >= (int)sizeof(int_val))
		bcopy(params, &int_val, sizeof(int_val));

	switch (actionid) {
	case IOV_GVAL(IOV_VERSION):
		/* Need to have checked buffer length */
		dhd_ver_len = sizeof(dhd_version) - 1;
		bus_api_rev_len = strlen(bus_api_revision);
		if (len > dhd_ver_len + bus_api_rev_len) {
			bcmerror = memcpy_s((char *)arg, len, dhd_version, dhd_ver_len);
			if (bcmerror != BCME_OK) {
				break;
			}
			bcmerror = memcpy_s((char *)arg + dhd_ver_len, len - dhd_ver_len,
				bus_api_revision, bus_api_rev_len);
			if (bcmerror != BCME_OK) {
				break;
			}
			*((char *)arg + dhd_ver_len + bus_api_rev_len) = '\0';
		}
		break;

	case IOV_GVAL(IOV_MSGLEVEL):
		int_val = (int32)dhd_msg_level;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_MSGLEVEL):
#ifdef WL_CFG80211
		/* XXX Usage of this enhanced IOV
		 * dhd -i wlan0 msglevel 0x8001
		 * change dhd msg level to DHD_REORDER_VAL & DHD_ERROR_VAL
		 * dhd -i wlan0 msglevel 0x18001
		 * change dhd msg level to DHD_REORDER_VAL & DHD_ERROR_VAL
		 * and enable WL_DBG_DBG
		 * dhd -i wlan0 msglevel 0x200ff
		 * change wl debug msg level only ,
		 * do not change dhd msg level enable whole WL msg level
		 */
		/* Enable DHD and WL logs in oneshot */
		if (int_val & DHD_WL_VAL2) {
			wl_cfg80211_enable_trace(TRUE, int_val & (~DHD_WL_VAL2));
			wl_cfg80211_enable_log_trace(TRUE, int_val & (~DHD_WL_VAL2));
		} else if (int_val & DHD_WL_VAL) {
			wl_cfg80211_enable_trace(FALSE, WL_DBG_DBG);
			wl_cfg80211_enable_log_trace(FALSE, WL_DBG_DBG);
		}
		if (!(int_val & DHD_WL_VAL2))
#endif /* WL_CFG80211 */
		{
			dhd_msg_level = int_val;
			dhd_log_level = int_val;
		}
		break;
#ifdef DHD_LOGLEVEL
	case IOV_GVAL(IOV_LOGLEVEL):
		dhd_loglevel_get((dhd_loglevel_data_t *)arg);
		break;

	case IOV_SVAL(IOV_LOGLEVEL):
		dhd_loglevel_set((dhd_loglevel_data_t *)arg);
		break;
#endif /* DHD_LOGLEVEL */

	case IOV_GVAL(IOV_BCMERRORSTR):
		bcm_strncpy_s((char *)arg, len, bcmerrorstr(dhd_pub->bcmerror), BCME_STRLEN);
		((char *)arg)[BCME_STRLEN - 1] = 0x00;
		break;

	case IOV_GVAL(IOV_BCMERROR):
		int_val = (int32)dhd_pub->bcmerror;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_GVAL(IOV_WDTICK):
		int_val = (int32)dhd_watchdog_ms;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_WDTICK):
		if (!dhd_pub->up) {
			bcmerror = BCME_NOTUP;
			break;
		}

		dhd_wd_interval_set(dhd_pub, (uint)int_val);

		dhd_os_wd_timer(dhd_pub, (uint)int_val);
		break;

	case IOV_GVAL(IOV_DUMP):
		if (dhd_dump(dhd_pub, arg, len) <= 0)
			bcmerror = BCME_ERROR;
		else
			bcmerror = BCME_OK;
		break;

	case IOV_GVAL(IOV_DCONSOLE_POLL):
		int_val = (int32)dhd_pub->dhd_console_ms;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_DCONSOLE_POLL):
		dhd_pub->dhd_console_ms = (uint)int_val;
		break;

/* open FW cons and log by default */
#if defined(DHD_DEBUG) || defined(DHD_FW_CONS_SUPPORT)
	case IOV_SVAL(IOV_CONS):
		if (len > 0)
			bcmerror = dhd_bus_console_in(dhd_pub, arg, len - 1);
		break;
#endif /* DHD_DEBUG || DHD_FW_CONS_SUPPORT */

	case IOV_SVAL(IOV_CLEARCOUNTS):
		dhd_pub->tx_packets = dhd_pub->rx_packets = 0;
		dhd_pub->tx_errors = dhd_pub->rx_errors = 0;
		dhd_pub->tx_ctlpkts = dhd_pub->rx_ctlpkts = 0;
		dhd_pub->tx_ctlerrs = dhd_pub->rx_ctlerrs = 0;
		dhd_pub->tx_dropped = 0;
		dhd_pub->rx_dropped = 0;
		dhd_pub->tx_pktgetfail = 0;
		dhd_pub->rx_pktgetfail = 0;
		dhd_pub->rx_readahead_cnt = 0;
		dhd_pub->tx_realloc = 0;
		dhd_pub->wd_dpc_sched = 0;
		dhd_pub->tx_big_packets = 0;
		memset(&dhd_pub->dstats, 0, sizeof(dhd_pub->dstats));
		dhd_bus_clearcounts(dhd_pub);
#ifdef PROP_TXSTATUS
		/* clear proptxstatus related counters */
		dhd_wlfc_clear_counts(dhd_pub);
#endif /* PROP_TXSTATUS */
#if defined(DHD_LB_STATS)
		DHD_LB_STATS_RESET(dhd_pub);
#endif /* DHD_LB_STATS */
		break;

	case IOV_GVAL(IOV_IOCTLTIMEOUT): {
		int_val = (int32)dhd_os_get_ioctl_resp_timeout();
		bcopy(&int_val, arg, sizeof(int_val));
		break;
	}

	case IOV_SVAL(IOV_IOCTLTIMEOUT): {
		if (int_val <= 0)
			bcmerror = BCME_BADARG;
		else
			dhd_os_set_ioctl_resp_timeout((unsigned int)int_val);
		break;
	}

#ifdef PROP_TXSTATUS
	case IOV_GVAL(IOV_PROPTXSTATUS_ENABLE): {
		bool wlfc_enab = FALSE;
		bcmerror = dhd_wlfc_get_enable(dhd_pub, &wlfc_enab);
		if (bcmerror != BCME_OK)
			goto exit;
		int_val = wlfc_enab ? 1 : 0;
		bcopy(&int_val, arg, val_size);
		break;
	}
	case IOV_SVAL(IOV_PROPTXSTATUS_ENABLE): {
		bool wlfc_enab = FALSE;
		bcmerror = dhd_wlfc_get_enable(dhd_pub, &wlfc_enab);
		if (bcmerror != BCME_OK)
			goto exit;

		/* wlfc is already set as desired */
		if (wlfc_enab == (int_val == 0 ? FALSE : TRUE))
			goto exit;

		if (int_val == TRUE) {
			uint32 up;
			bcmerror = dhd_wlfc_init(dhd_pub);
			/* set WLC_UP to update wlfc credit */
			if (dhd_wl_ioctl_cmd(dhd_pub, WLC_UP, &up, sizeof(uint32), TRUE, 0) < 0) {
				DHD_ERROR(("%s: WLC_UP return err\n", __FUNCTION__));
			}
		} else
			bcmerror = dhd_wlfc_deinit(dhd_pub);

		break;
	}
	case IOV_GVAL(IOV_PROPTXSTATUS_MODE):
		bcmerror = dhd_wlfc_get_mode(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_MODE):
		dhd_wlfc_set_mode(dhd_pub, int_val);
		break;
#ifdef QMONITOR
	case IOV_GVAL(IOV_QMON_TIME_THRES): {
		int_val = dhd_qmon_thres(dhd_pub, FALSE, 0);
		bcopy(&int_val, arg, val_size);
		break;
	}

	case IOV_SVAL(IOV_QMON_TIME_THRES): {
		dhd_qmon_thres(dhd_pub, TRUE, int_val);
		break;
	}

	case IOV_GVAL(IOV_QMON_TIME_PERCENT): {
		int_val = dhd_qmon_getpercent(dhd_pub);
		bcopy(&int_val, arg, val_size);
		break;
	}
#endif /* QMONITOR */

	case IOV_GVAL(IOV_PROPTXSTATUS_MODULE_IGNORE):
		bcmerror = dhd_wlfc_get_module_ignore(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_MODULE_IGNORE):
		dhd_wlfc_set_module_ignore(dhd_pub, int_val);
		break;

	case IOV_GVAL(IOV_PROPTXSTATUS_CREDIT_IGNORE):
		bcmerror = dhd_wlfc_get_credit_ignore(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_CREDIT_IGNORE):
		dhd_wlfc_set_credit_ignore(dhd_pub, int_val);
		break;

	case IOV_GVAL(IOV_PROPTXSTATUS_TXSTATUS_IGNORE):
		bcmerror = dhd_wlfc_get_txstatus_ignore(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_TXSTATUS_IGNORE):
		dhd_wlfc_set_txstatus_ignore(dhd_pub, int_val);
		break;

	case IOV_GVAL(IOV_PROPTXSTATUS_RXPKT_CHK):
		bcmerror = dhd_wlfc_get_rxpkt_chk(dhd_pub, &int_val);
		if (bcmerror != BCME_OK)
			goto exit;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_PROPTXSTATUS_RXPKT_CHK):
		dhd_wlfc_set_rxpkt_chk(dhd_pub, int_val);
		break;

#endif /* PROP_TXSTATUS */

	case IOV_GVAL(IOV_BUS_TYPE):
		/* The dhd application queries the driver to check if its usb or sdio.  */
#ifdef BCMDHDUSB
		int_val = BUS_TYPE_USB;
#endif
#ifdef BCMSDIO
		int_val = BUS_TYPE_SDIO;
#endif
#ifdef PCIE_FULL_DONGLE
		int_val = BUS_TYPE_PCIE;
#endif
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_CHANGEMTU):
		int_val &= 0xffff;
		bcmerror = dhd_change_mtu(dhd_pub, int_val, 0);
		break;

	case IOV_GVAL(IOV_HOSTREORDER_FLOWS):
	{
		uint i = 0;
		uint8 *ptr = (uint8 *)arg;
		uint8 count = 0;

		ptr++;
		for (i = 0; i < WLHOST_REORDERDATA_MAXFLOWS; i++) {
			if (dhd_pub->reorder_bufs[i] != NULL) {
				*ptr = dhd_pub->reorder_bufs[i]->flow_id;
				ptr++;
				count++;
			}
		}
		ptr = (uint8 *)arg;
		*ptr = count;
		break;
	}
#ifdef DHDTCPACK_SUPPRESS
	case IOV_GVAL(IOV_TCPACK_SUPPRESS): {
		int_val = (uint32)dhd_pub->tcpack_sup_mode;
		bcopy(&int_val, arg, val_size);
		break;
	}
	case IOV_SVAL(IOV_TCPACK_SUPPRESS): {
		bcmerror = dhd_tcpack_suppress_set(dhd_pub, (uint8)int_val);
		break;
	}
#endif /* DHDTCPACK_SUPPRESS */

#ifdef DHD_L2_FILTER
	case IOV_GVAL(IOV_DHCP_UNICAST): {
		uint32 bssidx;
		const char *val;
		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_DHCP_UNICAST: bad parameterand name = %s\n",
				__FUNCTION__, name));
			bcmerror = BCME_BADARG;
			break;
		}
		int_val = dhd_get_dhcp_unicast_status(dhd_pub, bssidx);
		memcpy(arg, &int_val, val_size);
		break;
	}
	case IOV_SVAL(IOV_DHCP_UNICAST): {
		uint32	bssidx;
		const char *val;
		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_DHCP_UNICAST: bad parameterand name = %s\n",
				__FUNCTION__, name));
			bcmerror = BCME_BADARG;
			break;
		}
		memcpy(&int_val, val, sizeof(int_val));
		bcmerror = dhd_set_dhcp_unicast_status(dhd_pub, bssidx, int_val ? 1 : 0);
		break;
	}
	case IOV_GVAL(IOV_BLOCK_PING): {
		uint32 bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_BLOCK_PING: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}
		int_val = dhd_get_block_ping_status(dhd_pub, bssidx);
		memcpy(arg, &int_val, val_size);
		break;
	}
	case IOV_SVAL(IOV_BLOCK_PING): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_BLOCK_PING: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}
		memcpy(&int_val, val, sizeof(int_val));
		bcmerror = dhd_set_block_ping_status(dhd_pub, bssidx, int_val ? 1 : 0);
		break;
	}
	case IOV_GVAL(IOV_PROXY_ARP): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_PROXY_ARP: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}
		int_val = dhd_get_parp_status(dhd_pub, bssidx);
		bcopy(&int_val, arg, val_size);
		break;
	}
	case IOV_SVAL(IOV_PROXY_ARP): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_PROXY_ARP: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}
		bcopy(val, &int_val, sizeof(int_val));

		/* Issue a iovar request to WL to update the proxy arp capability bit
		 * in the Extended Capability IE of beacons/probe responses.
		 */
		bcmerror = dhd_iovar(dhd_pub, bssidx, "proxy_arp_advertise", val, sizeof(int_val),
				NULL, 0, TRUE);
		if (bcmerror == BCME_OK) {
			dhd_set_parp_status(dhd_pub, bssidx, int_val ? 1 : 0);
		}
		break;
	}
	case IOV_GVAL(IOV_GRAT_ARP): {
		uint32 bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_GRAT_ARP: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}
		int_val = dhd_get_grat_arp_status(dhd_pub, bssidx);
		memcpy(arg, &int_val, val_size);
		break;
	}
	case IOV_SVAL(IOV_GRAT_ARP): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_GRAT_ARP: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}
		memcpy(&int_val, val, sizeof(int_val));
		bcmerror = dhd_set_grat_arp_status(dhd_pub, bssidx, int_val ? 1 : 0);
		break;
	}
	case IOV_GVAL(IOV_BLOCK_TDLS): {
		uint32 bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_BLOCK_TDLS: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}
		int_val = dhd_get_block_tdls_status(dhd_pub, bssidx);
		memcpy(arg, &int_val, val_size);
		break;
	}
	case IOV_SVAL(IOV_BLOCK_TDLS): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: IOV_BLOCK_TDLS: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}
		memcpy(&int_val, val, sizeof(int_val));
		bcmerror = dhd_set_block_tdls_status(dhd_pub, bssidx, int_val ? 1 : 0);
		break;
	}
#endif /* DHD_L2_FILTER */
	case IOV_SVAL(IOV_DHD_IE): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: dhd ie: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}

		break;
	}
	case IOV_GVAL(IOV_AP_ISOLATE): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: ap isoalate: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}

		int_val = dhd_get_ap_isolate(dhd_pub, bssidx);
		bcopy(&int_val, arg, val_size);
		break;
	}
	case IOV_SVAL(IOV_AP_ISOLATE): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: ap isolate: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}

		ASSERT(val);
		bcopy(val, &int_val, sizeof(uint32));
		dhd_set_ap_isolate(dhd_pub, bssidx, int_val);
		break;
	}
#ifdef DHD_PSTA
	case IOV_GVAL(IOV_PSTA): {
		int_val = dhd_get_psta_mode(dhd_pub);
		bcopy(&int_val, arg, val_size);
		break;
		}
	case IOV_SVAL(IOV_PSTA): {
		if (int_val >= DHD_MODE_PSTA_DISABLED && int_val <= DHD_MODE_PSR) {
			dhd_set_psta_mode(dhd_pub, int_val);
		} else {
			bcmerror = BCME_RANGE;
		}
		break;
		}
#endif /* DHD_PSTA */
#ifdef DHD_WET
	case IOV_GVAL(IOV_WET):
		 int_val = dhd_get_wet_mode(dhd_pub);
		 bcopy(&int_val, arg, val_size);
		 break;

	case IOV_SVAL(IOV_WET):
		 if (int_val == 0 || int_val == 1) {
			 dhd_set_wet_mode(dhd_pub, int_val);
			 /* Delete the WET DB when disabled */
			 if (!int_val) {
				 dhd_wet_sta_delete_list(dhd_pub);
			 }
		 } else {
			 bcmerror = BCME_RANGE;
		 }
				 break;
	case IOV_SVAL(IOV_WET_HOST_IPV4):
			dhd_set_wet_host_ipv4(dhd_pub, params, plen);
			break;
	case IOV_SVAL(IOV_WET_HOST_MAC):
			dhd_set_wet_host_mac(dhd_pub, params, plen);
		break;
#endif /* DHD_WET */
#ifdef DHD_MCAST_REGEN
	case IOV_GVAL(IOV_MCAST_REGEN_BSS_ENABLE): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, (char *)name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: mcast_regen_bss_enable: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}

		int_val = dhd_get_mcast_regen_bss_enable(dhd_pub, bssidx);
		bcopy(&int_val, arg, val_size);
		break;
	}

	case IOV_SVAL(IOV_MCAST_REGEN_BSS_ENABLE): {
		uint32	bssidx;
		const char *val;

		if (dhd_iovar_parse_bssidx(dhd_pub, (char *)name, &bssidx, &val) != BCME_OK) {
			DHD_ERROR(("%s: mcast_regen_bss_enable: bad parameter\n", __FUNCTION__));
			bcmerror = BCME_BADARG;
			break;
		}

		ASSERT(val);
		bcopy(val, &int_val, sizeof(uint32));
		dhd_set_mcast_regen_bss_enable(dhd_pub, bssidx, int_val);
		break;
	}
#endif /* DHD_MCAST_REGEN */

	case IOV_GVAL(IOV_CFG80211_OPMODE): {
		int_val = (int32)dhd_pub->op_mode;
		bcopy(&int_val, arg, sizeof(int_val));
		break;
		}
	case IOV_SVAL(IOV_CFG80211_OPMODE): {
		if (int_val <= 0)
			bcmerror = BCME_BADARG;
		else
			dhd_pub->op_mode = int_val;
		break;
	}

	case IOV_GVAL(IOV_ASSERT_TYPE):
		int_val = g_assert_type;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_ASSERT_TYPE):
		g_assert_type = (uint32)int_val;
		break;

	case IOV_GVAL(IOV_LMTEST): {
		*(uint32 *)arg = (uint32)lmtest;
		break;
	}

	case IOV_SVAL(IOV_LMTEST): {
		uint32 val = *(uint32 *)arg;
		if (val > 50)
			bcmerror = BCME_BADARG;
		else {
			lmtest = (uint)val;
			DHD_ERROR(("%s: lmtest %s\n",
				__FUNCTION__, (lmtest == FALSE)? "OFF" : "ON"));
		}
		break;
	}

#ifdef SHOW_LOGTRACE
	case IOV_GVAL(IOV_DUMP_TRACE_LOG): {
		trace_buf_info_t *trace_buf_info = (trace_buf_info_t *)arg;
		dhd_dbg_ring_t *dbg_verbose_ring = NULL;

		dbg_verbose_ring = dhd_dbg_get_ring_from_ring_id(dhd_pub, FW_VERBOSE_RING_ID);
		if (dbg_verbose_ring == NULL) {
			DHD_ERROR(("dbg_verbose_ring is NULL\n"));
			bcmerror = BCME_UNSUPPORTED;
			break;
		}

		if (trace_buf_info != NULL) {
			bzero(trace_buf_info, sizeof(trace_buf_info_t));
			dhd_dbg_read_ring_into_trace_buf(dbg_verbose_ring, trace_buf_info);
		} else {
			DHD_ERROR(("%s: arg is NULL\n", __FUNCTION__));
			bcmerror = BCME_NOMEM;
		}
		break;
	}
#endif /* SHOW_LOGTRACE */
	case IOV_GVAL(IOV_DONGLE_TRAP_TYPE):
		if (dhd_pub->dongle_trap_occured)
			int_val = ltoh32(dhd_pub->last_trap_info.type);
		else
			int_val = 0;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_GVAL(IOV_DONGLE_TRAP_INFO):
	{
		struct bcmstrbuf strbuf;
		bcm_binit(&strbuf, arg, len);
		if (dhd_pub->dongle_trap_occured == FALSE) {
			bcm_bprintf(&strbuf, "no trap recorded\n");
			break;
		}
		dhd_bus_dump_trap_info(dhd_pub->bus, &strbuf);
		break;
	}
#ifdef DHD_DEBUG
#if defined(BCMSDIO) || defined(BCMPCIE)

	case IOV_GVAL(IOV_BPADDR):
		{
			sdreg_t sdreg;
			uint32 addr, size;

			memcpy(&sdreg, params, sizeof(sdreg));

			addr = sdreg.offset;
			size = sdreg.func;

			bcmerror = dhd_bus_readwrite_bp_addr(dhd_pub, addr, size,
				(uint *)&int_val, TRUE);

			memcpy(arg, &int_val, sizeof(int32));

			break;
		}

	case IOV_SVAL(IOV_BPADDR):
		{
			sdreg_t sdreg;
			uint32 addr, size;

			memcpy(&sdreg, params, sizeof(sdreg));

			addr = sdreg.offset;
			size = sdreg.func;

			bcmerror = dhd_bus_readwrite_bp_addr(dhd_pub, addr, size,
				(uint *)&sdreg.value,
				FALSE);

			break;
		}
#endif /* BCMSDIO || BCMPCIE */
#ifdef BCMPCIE
	case IOV_SVAL(IOV_FLOW_RING_DEBUG):
		{
			bcmerror = dhd_flow_ring_debug(dhd_pub, arg, len);
			break;
		}
#endif /* BCMPCIE */
	case IOV_SVAL(IOV_MEM_DEBUG):
		if (len > 0) {
			bcmerror = dhd_mem_debug(dhd_pub, arg, len - 1);
		}
		break;
#endif /* DHD_DEBUG */
#if defined(DHD_LOG_DUMP)
	case IOV_GVAL(IOV_LOG_DUMP):
		{
			dhd_prot_debug_info_print(dhd_pub);
			dhd_log_dump_trigger(dhd_pub, CMD_DEFAULT);
			break;
		}
#endif /* DHD_LOG_DUMP */

	case IOV_GVAL(IOV_TPUT_TEST):
		{
			tput_test_t *tput_data = NULL;
			if (params && plen >= sizeof(tput_test_t)) {
				tput_data = (tput_test_t *)params;
				bcmerror = dhd_tput_test(dhd_pub, tput_data);
			} else {
				DHD_ERROR(("%s: tput test - no input params ! \n", __FUNCTION__));
				bcmerror = BCME_BADARG;
			}
			break;
		}
	case IOV_GVAL(IOV_DEBUG_BUF_DEST_STAT):
		{
			if (dhd_pub->debug_buf_dest_support) {
				debug_buf_dest_stat_t *debug_buf_dest_stat =
					(debug_buf_dest_stat_t *)arg;
				memcpy(debug_buf_dest_stat, dhd_pub->debug_buf_dest_stat,
					sizeof(dhd_pub->debug_buf_dest_stat));
			} else {
				bcmerror = BCME_DISABLED;
			}
			break;
		}

#if defined(DHD_SSSR_DUMP)
	case IOV_GVAL(IOV_FIS_TRIGGER):
		bcmerror = dhd_bus_fis_trigger(dhd_pub);

		if (bcmerror == BCME_OK) {
			bcmerror = dhd_bus_fis_dump(dhd_pub);
		}

		int_val = bcmerror;
		bcopy(&int_val, arg, val_size);
		break;
#endif /* defined(DHD_SSSR_DUMP) */

#ifdef DHD_DEBUG
	case IOV_SVAL(IOV_INDUCE_ERROR): {
		if (int_val >= DHD_INDUCE_ERROR_MAX) {
			DHD_ERROR(("%s: Invalid command : %u\n", __FUNCTION__, (uint16)int_val));
		} else {
			dhd_pub->dhd_induce_error = (uint16)int_val;
#ifdef BCMPCIE
			if (dhd_pub->dhd_induce_error == DHD_INDUCE_BH_CBP_HANG) {
				dhdpcie_induce_cbp_hang(dhd_pub);
			}
#endif /* BCMPCIE */
		}
		break;
	}
#endif /* DHD_DEBUG */
#ifdef RTT_GEOFENCE_CONT
#if defined(RTT_SUPPORT) && defined(WL_NAN)
	case IOV_GVAL(IOV_RTT_GEOFENCE_TYPE_OVRD): {
		bool enable = 0;
		dhd_rtt_get_geofence_cont_ind(dhd_pub, &enable);
		int_val = enable ? 1 : 0;
		bcopy(&int_val, arg, val_size);
		break;
	}
	case IOV_SVAL(IOV_RTT_GEOFENCE_TYPE_OVRD): {
		bool enable = *(bool *)arg;
		dhd_rtt_set_geofence_cont_ind(dhd_pub, enable);
		break;
	}
#endif /* RTT_SUPPORT && WL_NAN */
#endif /* RTT_GEOFENCE_CONT */
	case IOV_GVAL(IOV_FW_VBS): {
		*(uint32 *)arg = (uint32)dhd_dbg_get_fwverbose(dhd_pub);
		break;
	}

	case IOV_SVAL(IOV_FW_VBS): {
		if (int_val < 0) {
			int_val = 0;
		}
		dhd_dbg_set_fwverbose(dhd_pub, (uint32)int_val);
		break;
	}

#ifdef DHD_TX_PROFILE
	case IOV_SVAL(IOV_TX_PROFILE_TAG):
	{
		/* note: under the current implementation only one type of packet may be
		 * tagged per profile
		 */
		const dhd_tx_profile_protocol_t *protocol = NULL;
		/* for example, we might have a profile of profile_index 6, but at
		 * offset 2 from dhd_pub->protocol_filters.
		 */
		uint8 offset;

		if (params == NULL) {
			bcmerror = BCME_ERROR;
			break;
		}

		protocol = (dhd_tx_profile_protocol_t *)params;

		/* validate */
		if (protocol->version != DHD_TX_PROFILE_VERSION) {
			bcmerror = BCME_VERSION;
			break;
		}
		if (protocol->profile_index > DHD_MAX_PROFILE_INDEX) {
			DHD_ERROR(("%s:\tprofile index must be between 0 and %d\n",
					__FUNCTION__, DHD_MAX_PROFILE_INDEX));
			bcmerror = BCME_RANGE;
			break;
		}
		if (protocol->layer != DHD_TX_PROFILE_DATA_LINK_LAYER && protocol->layer
				!= DHD_TX_PROFILE_NETWORK_LAYER) {
			DHD_ERROR(("%s:\tlayer must be %d or %d\n", __FUNCTION__,
					DHD_TX_PROFILE_DATA_LINK_LAYER,
					DHD_TX_PROFILE_NETWORK_LAYER));
			bcmerror = BCME_BADARG;
			break;
		}
		if (protocol->protocol_number > __UINT16_MAX__) {
			DHD_ERROR(("%s:\tprotocol number must be <= %d\n", __FUNCTION__,
					__UINT16_MAX__));
			bcmerror = BCME_BADLEN;
			break;
		}

		/* find the dhd_tx_profile_protocol_t */
		for (offset = 0; offset < dhd_pub->num_profiles; offset++) {
			if (dhd_pub->protocol_filters[offset].profile_index ==
					protocol->profile_index) {
				break;
			}
		}

		if (offset >= DHD_MAX_PROFILES) {
#if DHD_MAX_PROFILES > 1
			DHD_ERROR(("%s:\tonly %d profiles supported at present\n",
					__FUNCTION__, DHD_MAX_PROFILES));
#else /* DHD_MAX_PROFILES > 1 */
			DHD_ERROR(("%s:\tonly %d profile supported at present\n",
					__FUNCTION__, DHD_MAX_PROFILES));
			DHD_ERROR(("%s:\tthere is a profile of index %d\n", __FUNCTION__,
					dhd_pub->protocol_filters->profile_index));
#endif /* DHD_MAX_PROFILES > 1 */
			bcmerror = BCME_NOMEM;
			break;
		}

		/* memory already allocated in dhd_attach; just assign the value */
		dhd_pub->protocol_filters[offset] = *protocol;

		if (offset >= dhd_pub->num_profiles) {
			dhd_pub->num_profiles = offset + 1;
		}

		break;
	}

	case IOV_SVAL(IOV_TX_PROFILE_ENABLE):
		dhd_pub->tx_profile_enab = int_val ? TRUE : FALSE;
		break;

	case IOV_GVAL(IOV_TX_PROFILE_ENABLE):
		int_val = dhd_pub->tx_profile_enab;
		bcmerror = memcpy_s(arg, val_size, &int_val, sizeof(int_val));
		break;

	case IOV_SVAL(IOV_TX_PROFILE_DUMP):
	{
		const dhd_tx_profile_protocol_t *protocol = NULL;
		uint8 offset;
		char *format = "%s:\ttx_profile %s: %d\n";

		for (offset = 0; offset < dhd_pub->num_profiles; offset++) {
			if (dhd_pub->protocol_filters[offset].profile_index == int_val) {
				protocol = &(dhd_pub->protocol_filters[offset]);
				break;
			}
		}

		if (protocol == NULL) {
			DHD_ERROR(("%s:\tno profile with index %d\n", __FUNCTION__,
					int_val));
			bcmerror = BCME_ERROR;
			break;
		}

		printf(format, __FUNCTION__, "profile_index", protocol->profile_index);
		printf(format, __FUNCTION__, "layer", protocol->layer);
		printf(format, __FUNCTION__, "protocol_number", protocol->protocol_number);
		printf(format, __FUNCTION__, "src_port", protocol->src_port);
		printf(format, __FUNCTION__, "dest_port", protocol->dest_port);

		break;
	}
#endif /* defined(DHD_TX_PROFILE) */

	case IOV_GVAL(IOV_CHECK_TRAP_ROT): {
		int_val = dhd_pub->check_trap_rot? 1 : 0;
		(void)memcpy_s(arg, val_size, &int_val, sizeof(int_val));
		break;
	}
	case IOV_SVAL(IOV_CHECK_TRAP_ROT): {
		dhd_pub->check_trap_rot = *(bool *)arg;
		break;
	}

#if defined(WL_MON_OWN_PKT)
	case IOV_SVAL(IOV_ROAM_EVENTS): {
		bool bval = *(bool *)arg;
		if (bval != 0 && bval != 1)
			bcmerror = BCME_ERROR;
		else
			dhd_pub->roam_event_rx = bval;
		break;
	}
	case IOV_GVAL(IOV_ROAM_EVENTS):
		int_val = dhd_pub->roam_event_rx;
		(void)memcpy_s(arg, val_size, &int_val, sizeof(int_val));
		break;
#endif
#ifdef SYNA_SAR_CUSTOMER_PARAMETER
	case IOV_GVAL(IOV_SAR_MODE):
		int_val = dhd_pub->dhd_sar_mode;
		(void)memcpy_s(arg, val_size, &int_val, sizeof(int_val));
		break;
	case IOV_SVAL(IOV_SAR_MODE):
		dhd_sar_set(dhd_pub, int_val);
		break;
#endif /* SYNA_SAR_CUSTOMER_PARAMETER */

#ifdef BCMPCIE
	case IOV_SVAL(IOV_L1SS_INIT_ENAB): {
		bool bval = *(bool *)arg;
		DHD_ERROR(("%s: set the L1SS value to be %d\n", __FUNCTION__, bval));
		dhd_bus_l1ss_enable_rc_ep(dhd_pub->bus, bval);
		break;
	}
	case IOV_SVAL(IOV_ASPM_INIT_ENAB): {
		bool bval = *(bool *)arg;
		DHD_ERROR(("%s: set the ASPM value to be %d\n", __FUNCTION__, bval));
		dhd_bus_aspm_enable_rc_ep(dhd_pub->bus, bval);
		break;
	}
#endif /* BCMPCIE */

	default:
		bcmerror = BCME_UNSUPPORTED;
		break;
	}

exit:
	DHD_TRACE(("%s: actionid %d, bcmerror %d\n", __FUNCTION__, actionid, bcmerror));
	return bcmerror;
}

/* Store the status of a connection attempt for later retrieval by an iovar */
void
dhd_store_conn_status(uint32 event, uint32 status, uint32 reason)
{
	/* Do not overwrite a WLC_E_PRUNE with a WLC_E_SET_SSID
	 * because an encryption/rsn mismatch results in both events, and
	 * the important information is in the WLC_E_PRUNE.
	 */
	if (!(event == WLC_E_SET_SSID && status == WLC_E_STATUS_FAIL &&
	      dhd_conn_event == WLC_E_PRUNE)) {
		dhd_conn_event = event;
		dhd_conn_status = status;
		dhd_conn_reason = reason;
	}
}

bool
dhd_prec_enq(dhd_pub_t *dhdp, struct pktq *q, void *pkt, int prec, bool enqhead)
{
	void *p;
	int eprec = -1;		/* precedence to evict from */
	bool discard_oldest;

	/* Fast case, precedence queue is not full and we are also not
	 * exceeding total queue length
	 */
	if (!pktqprec_full(q, prec) && !pktq_full(q)) {
		if (enqhead)
			pktq_penq_head(q, prec, pkt);
		else
			pktq_penq(q, prec, pkt);
		return TRUE;
	}

	/* Determine precedence from which to evict packet, if any */
	if (pktqprec_full(q, prec))
		eprec = prec;
	else if (pktq_full(q)) {
		p = pktq_peek_tail(q, &eprec);
		ASSERT(p);
		if (eprec > prec || eprec < 0)
			return FALSE;
	}

	/* Evict if needed */
	if (eprec >= 0) {
		/* Detect queueing to unconfigured precedence */
		ASSERT(!pktqprec_empty(q, eprec));
		discard_oldest = AC_BITMAP_TST(dhdp->wme_dp, eprec);
		if (eprec == prec && !discard_oldest)
			return FALSE;		/* refuse newer (incoming) packet */
		/* Evict packet according to discard policy */
		p = discard_oldest ? pktq_pdeq(q, eprec) : pktq_pdeq_tail(q, eprec);
		ASSERT(p);
#ifdef DHDTCPACK_SUPPRESS
		if (dhd_tcpack_check_xmit(dhdp, p) == BCME_ERROR) {
			DHD_ERROR(("%s %d: tcpack_suppress ERROR!!! Stop using it\n",
				__FUNCTION__, __LINE__));
			dhd_tcpack_suppress_set(dhdp, TCPACK_SUP_OFF);
		}
#endif /* DHDTCPACK_SUPPRESS */
		PKTFREE(dhdp->osh, p, TRUE);
	}

	/* Enqueue */
	if (enqhead)
		p = pktq_penq_head(q, prec, pkt);
	else
		p = pktq_penq(q, prec, pkt);
	ASSERT(p);

	return TRUE;
}

/*
 * Functions to drop proper pkts from queue:
 *	If one pkt in queue is non-fragmented, drop first non-fragmented pkt only
 *	If all pkts in queue are all fragmented, find and drop one whole set fragmented pkts
 *	If can't find pkts matching upper 2 cases, drop first pkt anyway
 */
bool
dhd_prec_drop_pkts(dhd_pub_t *dhdp, struct pktq *pq, int prec, f_droppkt_t fn)
{
	struct pktq_prec *q = NULL;
	void *p, *prev = NULL, *next = NULL, *first = NULL, *last = NULL, *prev_first = NULL;
	pkt_frag_t frag_info;

	ASSERT(dhdp && pq);
	ASSERT(prec >= 0 && prec < pq->num_prec);

	q = &pq->q[prec];
	p = q->head;

	if (p == NULL)
		return FALSE;

	while (p) {
		frag_info = pkt_frag_info(dhdp->osh, p);
		if (frag_info == DHD_PKT_FRAG_NONE) {
			break;
		} else if (frag_info == DHD_PKT_FRAG_FIRST) {
			if (first) {
				/* No last frag pkt, use prev as last */
				last = prev;
				break;
			} else {
				first = p;
				prev_first = prev;
			}
		} else if (frag_info == DHD_PKT_FRAG_LAST) {
			if (first) {
				last = p;
				break;
			}
		}

		prev = p;
		p = PKTLINK(p);
	}

	if ((p == NULL) || ((frag_info != DHD_PKT_FRAG_NONE) && !(first && last))) {
		/* Not found matching pkts, use oldest */
		prev = NULL;
		p = q->head;
		frag_info = 0;
	}

	if (frag_info == DHD_PKT_FRAG_NONE) {
		first = last = p;
		prev_first = prev;
	}

	p = first;
	while (p) {
		next = PKTLINK(p);
		q->n_pkts--;
		pq->n_pkts_tot--;

#ifdef WL_TXQ_STALL
		q->dequeue_count++;
#endif

		PKTSETLINK(p, NULL);

		if (fn)
			fn(dhdp, prec, p, TRUE);

		if (p == last)
			break;

		p = next;
	}

	if (prev_first == NULL) {
		if ((q->head = next) == NULL)
			q->tail = NULL;
	} else {
		PKTSETLINK(prev_first, next);
		if (!next)
			q->tail = prev_first;
	}

	return TRUE;
}

static int
dhd_iovar_op(dhd_pub_t *dhd_pub, const char *name,
	void *params, int plen, void *arg, uint len, bool set)
{
	int bcmerror = 0;
	uint val_size;
	const bcm_iovar_t *vi = NULL;
	uint32 actionid;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	ASSERT(name);

	/* Get MUST have return space */
	ASSERT(set || (arg && len));

	/* Set does NOT take qualifiers */
	ASSERT(!set || (!params && !plen));

	if ((vi = bcm_iovar_lookup(dhd_iovars, name)) == NULL) {
		bcmerror = BCME_UNSUPPORTED;
		goto exit;
	}

	DHD_CTL(("%s: %s %s, len %d plen %d\n", __FUNCTION__,
		name, (set ? "set" : "get"), len, plen));

	/* set up 'params' pointer in case this is a set command so that
	 * the convenience int and bool code can be common to set and get
	 */
	if (params == NULL) {
		params = arg;
		plen = len;
	}

	if (vi->type == IOVT_VOID)
		val_size = 0;
	else if (vi->type == IOVT_BUFFER)
		val_size = len;
	else
		/* all other types are integer sized */
		val_size = sizeof(int);

	actionid = set ? IOV_SVAL(vi->varid) : IOV_GVAL(vi->varid);

	bcmerror = dhd_doiovar(dhd_pub, vi, actionid, name, params, plen, arg, len, val_size);

exit:
	return bcmerror;
}

int
dhd_ioctl(dhd_pub_t * dhd_pub, dhd_ioctl_t *ioc, void *buf, uint buflen)
{
	int bcmerror = 0;
	unsigned long flags;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	if (!buf) {
		return BCME_BADARG;
	}

	dhd_os_dhdiovar_lock(dhd_pub);
	switch (ioc->cmd) {
		case DHD_GET_MAGIC:
			if (buflen < sizeof(int))
				bcmerror = BCME_BUFTOOSHORT;
			else
				*(int*)buf = DHD_IOCTL_MAGIC;
			break;

		case DHD_GET_VERSION:
			if (buflen < sizeof(int))
				bcmerror = BCME_BUFTOOSHORT;
			else
				*(int*)buf = DHD_IOCTL_VERSION;
			break;

		case DHD_GET_VAR:
		case DHD_SET_VAR:
			{
				char *arg;
				uint arglen;

				DHD_LINUX_GENERAL_LOCK(dhd_pub, flags);
				if (DHD_BUS_CHECK_DOWN_OR_DOWN_IN_PROGRESS(dhd_pub) &&
					bcmstricmp((char *)buf, "devreset")) {
					/* In platforms like FC19, the FW download is done via IOCTL
					 * and should not return error for IOCTLs fired before FW
					 * Download is done
					 */
					if (dhd_fw_download_status(dhd_pub) == FW_DOWNLOAD_DONE) {
						DHD_ERROR(("%s: return as fw_download_status=%d\n",
							__FUNCTION__,
							dhd_fw_download_status(dhd_pub)));
						DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);
						dhd_os_dhdiovar_unlock(dhd_pub);
						return -ENODEV;
					}
				}
				DHD_BUS_BUSY_SET_IN_DHD_IOVAR(dhd_pub);
				DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);

#ifdef DHD_PCIE_RUNTIMEPM
				dhdpcie_runtime_bus_wake(dhd_pub, TRUE, dhd_ioctl);
#endif /* DHD_PCIE_RUNTIMEPM */

				DHD_LINUX_GENERAL_LOCK(dhd_pub, flags);
				if (DHD_BUS_CHECK_SUSPEND_OR_SUSPEND_IN_PROGRESS(dhd_pub)) {
					/* If Suspend/Resume is tested via pcie_suspend IOVAR
					 * then continue to execute the IOVAR, return from here for
					 * other IOVARs, also include pciecfgreg and devreset to go
					 * through.
					 */
					if (bcmstricmp((char *)buf, "pcie_suspend") &&
					    bcmstricmp((char *)buf, "pciecfgreg") &&
					    bcmstricmp((char *)buf, "devreset") &&
					    bcmstricmp((char *)buf, "sdio_suspend")) {
						DHD_ERROR(("%s: bus is in suspend(%d)"
							"or suspending(0x%x) state\n",
							__FUNCTION__, dhd_pub->busstate,
							dhd_pub->dhd_bus_busy_state));
						DHD_BUS_BUSY_CLEAR_IN_DHD_IOVAR(dhd_pub);
						dhd_os_busbusy_wake(dhd_pub);
						DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);
						dhd_os_dhdiovar_unlock(dhd_pub);
						return -ENODEV;
					}
				}
				/* During devreset ioctl, we call dhdpcie_advertise_bus_cleanup,
				 * which will wait for all the busy contexts to get over for
				 * particular time and call ASSERT if timeout happens. As during
				 * devreset ioctal, we made DHD_BUS_BUSY_SET_IN_DHD_IOVAR,
				 * to avoid ASSERT, clear the IOCTL busy state. "devreset" ioctl is
				 * not used in Production platforms but only used in FC19 setups.
				 */
				if (!bcmstricmp((char *)buf, "devreset") ||
#ifdef BCMPCIE
				    (dhd_bus_is_multibp_capable(dhd_pub->bus) &&
				    !bcmstricmp((char *)buf, "dwnldstate")) ||
#endif /* BCMPCIE */
				    FALSE)
				{
					DHD_BUS_BUSY_CLEAR_IN_DHD_IOVAR(dhd_pub);
				}
				DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);

				/* scan past the name to any arguments */
				for (arg = buf, arglen = buflen; *arg && arglen; arg++, arglen--)
					;

				if (arglen == 0 || *arg) {
					bcmerror = BCME_BUFTOOSHORT;
					goto unlock_exit;
				}

				/* account for the NUL terminator */
				arg++, arglen--;
				/* call with the appropriate arguments */
				if (ioc->cmd == DHD_GET_VAR) {
					bcmerror = dhd_iovar_op(dhd_pub, buf, arg, arglen,
							buf, buflen, IOV_GET);
				} else {
					bcmerror = dhd_iovar_op(dhd_pub, buf, NULL, 0,
							arg, arglen, IOV_SET);
				}
				if (bcmerror != BCME_UNSUPPORTED) {
					goto unlock_exit;
				}

				/* not in generic table, try protocol module */
				if (ioc->cmd == DHD_GET_VAR) {
					bcmerror = dhd_prot_iovar_op(dhd_pub, buf, arg,
							arglen, buf, buflen, IOV_GET);
				} else {
					bcmerror = dhd_prot_iovar_op(dhd_pub, buf,
							NULL, 0, arg, arglen, IOV_SET);
				}
				if (bcmerror != BCME_UNSUPPORTED) {
					goto unlock_exit;
				}

				/* if still not found, try bus module */
				if (ioc->cmd == DHD_GET_VAR) {
					bcmerror = dhd_bus_iovar_op(dhd_pub, buf,
							arg, arglen, buf, buflen, IOV_GET);
				} else {
					bcmerror = dhd_bus_iovar_op(dhd_pub, buf,
							NULL, 0, arg, arglen, IOV_SET);
				}
				if (bcmerror != BCME_UNSUPPORTED) {
					goto unlock_exit;
				}

			}
			goto unlock_exit;

		default:
			bcmerror = BCME_UNSUPPORTED;
	}
	dhd_os_dhdiovar_unlock(dhd_pub);
	return bcmerror;

unlock_exit:
	DHD_LINUX_GENERAL_LOCK(dhd_pub, flags);
	DHD_BUS_BUSY_CLEAR_IN_DHD_IOVAR(dhd_pub);
	dhd_os_busbusy_wake(dhd_pub);
	DHD_LINUX_GENERAL_UNLOCK(dhd_pub, flags);
	dhd_os_dhdiovar_unlock(dhd_pub);
	return bcmerror;
}

#ifdef SHOW_EVENTS

static void
wl_show_roam_event(dhd_pub_t *dhd_pub, uint status, uint datalen,
	uint event_type, const char *event_name, char *eabuf, void *event_data)
{
	if (status == WLC_E_STATUS_SUCCESS) {
		DHD_EVENT(("MACEVENT: %s(%d), MAC %s\n",
		           event_name, event_type, eabuf));
	} else {
		if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s(%d), failed status %d\n",
			           event_name, event_type, status));
		} else if (status == WLC_E_STATUS_NO_NETWORKS) {
			if (datalen) {
				uint8 id = *((uint8 *)event_data);
				if (id != DOT11_MNG_PROPR_ID) {
					wl_roam_event_t *roam_data =
						(wl_roam_event_t *)event_data;
					bcm_xtlv_t *tlv = (bcm_xtlv_t *)roam_data->xtlvs;
					if (tlv->id == WLC_ROAM_NO_NETWORKS_TLV_ID) {
						uint32 *fail_reason = (uint32 *)tlv->data;
						switch (*fail_reason) {
							case WLC_E_REASON_NO_NETWORKS:
								DHD_EVENT(("MACEVENT: %s(%d),"
								           " no networks found\n",
								           event_name, event_type));
								break;
							case WLC_E_REASON_NO_NETWORKS_BY_SCORE:
								DHD_EVENT(("MACEVENT: %s(%d),"
								           " no networks found by score\n",
								           event_name, event_type));
								break;
							default:
								DHD_ERROR(("MACEVENT: %s(%d),"
								           " unknown fail reason 0x%x\n",
								           event_name, event_type,
								           *fail_reason));
								ASSERT(0);
						}
					} else {
						DHD_EVENT(("MACEVENT: %s(%d),"
						           " no networks found\n",
						           event_name, event_type));
					}
				} else {
					DHD_EVENT(("MACEVENT: %s(%d),"
					           " no networks found\n",
					           event_name, event_type));
				}
			} else {
				DHD_EVENT(("MACEVENT: %s(%d), no networks found\n",
				           event_name, event_type));
			}
		} else {
			DHD_EVENT(("MACEVENT: %s(%d), unexpected status %d\n",
			           event_name, event_type, (int)status));
		}
	}
}

static void
wl_show_roam_cache_update_event(const char *name, uint status,
	uint reason, uint datalen, void *event_data)
{
	wlc_roam_cache_update_event_t *cache_update;
	uint16 len_of_tlvs;
	void *val_tlv_ptr;
	bcm_xtlv_t *val_xtlv;
	char ntoa_buf[ETHER_ADDR_STR_LEN];
	uint idx;
	const char* reason_name = NULL;
	const char* status_name = NULL;
	static struct {
		uint event;
		const char *event_name;
	} reason_names[] = {
		{WLC_E_REASON_INITIAL_ASSOC, "INITIAL ASSOCIATION"},
		{WLC_E_REASON_LOW_RSSI, "LOW_RSSI"},
		{WLC_E_REASON_DEAUTH, "RECEIVED DEAUTHENTICATION"},
		{WLC_E_REASON_DISASSOC, "RECEIVED DISASSOCATION"},
		{WLC_E_REASON_BCNS_LOST, "BEACONS LOST"},
		{WLC_E_REASON_BETTER_AP, "BETTER AP FOUND"},
		{WLC_E_REASON_MINTXRATE, "STUCK AT MIN TX RATE"},
		{WLC_E_REASON_BSSTRANS_REQ, "REQUESTED ROAM"},
		{WLC_E_REASON_TXFAIL, "TOO MANY TXFAILURES"},
		{WLC_E_REASON_SIGNAL_DROP_STEP_POINT, "SIGNAL_DROP_STEP_POINT"},
		{WLC_E_REASON_HIGH_QBSS_LOAD, "HIGH_QBSS_LOAD"},
		{WLC_E_REASON_CUSTOMER_INITIALIZE, "CUSTOMER_INITIALIZE"},
		{WLC_E_REASON_PARTIAL_SWITCH_FULL, "PARTIAL_TO_FULL"},
		{WLC_E_REASON_LOW_RSSI_PARTIAL2FULL, "LOW_RSSI_PARTIAL2FULL"},
		{WLC_E_REASON_BEACON_LOST_PARTIAL2FULL, "BEACON_LOST_PARTIAL2FULL"},
		{WLC_E_REASON_SIGNAL_DROP_STEP_POINT_PARTIAL2FULL, "SIGNAL_DROP_STEP_POINT_PARTIAL2FULL"}
	};

	static struct {
		uint event;
		const char *event_name;
	} status_names[] = {
		{WLC_E_STATUS_SUCCESS, "operation was successful"},
		{WLC_E_STATUS_FAIL, "operation failed"},
		{WLC_E_STATUS_TIMEOUT, "operation timed out"},
		{WLC_E_STATUS_NO_NETWORKS, "failed due to no matching network found"},
		{WLC_E_STATUS_ABORT, "operation was aborted"},
		{WLC_E_STATUS_NO_ACK, "protocol failure: packet not ack'd"},
		{WLC_E_STATUS_UNSOLICITED, "AUTH or ASSOC packet was unsolicited"},
		{WLC_E_STATUS_ATTEMPT, "attempt to assoc to an auto auth configuration"},
		{WLC_E_STATUS_PARTIAL, "scan results are incomplete"},
		{WLC_E_STATUS_NEWSCAN, "scan aborted by another scan"},
		{WLC_E_STATUS_NEWASSOC, "scan aborted due to assoc in progress"},
		{WLC_E_STATUS_11HQUIET, "802.11h quiet period started"},
		{WLC_E_STATUS_SUPPRESS, "user disabled scanning"},
		{WLC_E_STATUS_NOCHANS, "no allowable channels to scan"},
		{WLC_E_STATUS_CS_ABORT, "abort channel select"},
		{WLC_E_STATUS_ERROR, "request failed due to error"},
		{WLC_E_STATUS_INVALID, "Invalid status code"}
	};

	UNUSED_PARAMETER(ntoa_buf);
	UNUSED_PARAMETER(reason_name);
	UNUSED_PARAMETER(status_name);

	switch (reason) {
	case WLC_ROAM_CACHE_UPDATE_NEW_ROAM_CACHE:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is new roam cache\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_JOIN:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is start of join\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_RSSI_DELTA:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is delta in rssi\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_MOTION_RSSI_DELTA:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is motion delta in rssi\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_CHANNEL_MISS:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is missed channel\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_START_SPLIT_SCAN:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is start of split scan\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_START_FULL_SCAN:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is start of full scan\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_INIT_ASSOC:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is init association\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_FULL_SCAN_FAILED:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is failure in full scan\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_NO_AP_FOUND:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is empty scan result\n", status));
		break;
	case WLC_ROAM_CACHE_UPDATE_MISSING_AP:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is missed ap\n", status));
		break;
	default:
		DHD_EVENT(("Current roam cache status %d, "
			"reason for cache update is unknown %d\n", status, reason));
		break;
	}

	if (datalen < sizeof(wlc_roam_cache_update_event_t)) {
		DHD_ERROR(("MACEVENT: %s, missing event data\n", name));
		return;
	}

	cache_update = (wlc_roam_cache_update_event_t *)event_data;
	val_tlv_ptr = (void *)cache_update->xtlvs;
	len_of_tlvs = datalen - sizeof(wlc_roam_cache_update_event_t);
	val_xtlv = (bcm_xtlv_t *)val_tlv_ptr;
	if (val_xtlv->id != WL_RMC_RPT_CMD_DATA) {
		DHD_ERROR(("MACEVENT: %s, unexpected xtlv id %d\n",
			name, val_xtlv->id));
		return;
	}
	val_tlv_ptr = (uint8 *)val_tlv_ptr + BCM_XTLV_HDR_SIZE;
	len_of_tlvs = val_xtlv->len;

	while (len_of_tlvs && len_of_tlvs > BCM_XTLV_HDR_SIZE) {
		val_xtlv = (bcm_xtlv_t *)val_tlv_ptr;
		switch (val_xtlv->id) {
			case WL_RMC_RPT_XTLV_BSS_INFO:
			{
				rmc_bss_info_v1_t *bss_info = (rmc_bss_info_v1_t *)(val_xtlv->data);
				DHD_EVENT(("\t Current BSS INFO:\n"));
				DHD_EVENT(("\t\tRSSI: %d\n", bss_info->rssi));
				DHD_EVENT(("\t\tNumber of full scans performed "
					"on current BSS: %d\n", bss_info->fullscan_count));
				for (idx = 0; idx < ARRAYSIZE(reason_names); idx++) {
					if (reason_names[idx].event == bss_info->reason) {
						reason_name = reason_names[idx].event_name;
					}
				}
				DHD_EVENT(("\t\tReason code for last full scan: %s(%d)\n",
					reason_name, bss_info->reason));
				DHD_EVENT(("\t\tDelta between current time and "
					"last full scan: %d\n", bss_info->time_full_scan));
				for (idx = 0; idx < ARRAYSIZE(status_names); idx++) {
					if (status_names[idx].event == bss_info->status)
						status_name = status_names[idx].event_name;
				}
				DHD_EVENT(("\t\tLast status code for not roaming: %s(%d)\n",
					status_name, bss_info->status));

			}
				break;
			case WL_RMC_RPT_XTLV_CANDIDATE_INFO:
			case WL_RMC_RPT_XTLV_USER_CACHE_INFO:
			{
				rmc_candidate_info_v1_t *candidate_info =
					(rmc_candidate_info_v1_t *)(val_xtlv->data);
				UNUSED_PARAMETER(candidate_info);
				if (val_xtlv->id == WL_RMC_RPT_XTLV_CANDIDATE_INFO) {
					DHD_EVENT(("\t Candidate INFO:\n"));
				} else {
					DHD_EVENT(("\t User Candidate INFO:\n"));
				}
				DHD_EVENT(("\t\tBSSID: %s\n",
					bcm_ether_ntoa((const struct ether_addr *)
					&candidate_info->bssid, ntoa_buf)));
				DHD_EVENT(("\t\tRSSI: %d\n", candidate_info->rssi));
				DHD_EVENT(("\t\tChannel: %d\n", candidate_info->ctl_channel));
				DHD_EVENT(("\t\tDelta between current time and last "
					"seen time: %d\n", candidate_info->time_last_seen));
				DHD_EVENT(("\t\tBSS load: %d\n", candidate_info->bss_load));
			}
				break;
			default:
				DHD_ERROR(("MACEVENT: %s, unexpected xtlv id %d\n",
					name, val_xtlv->id));
				return;
		}
		val_tlv_ptr = (uint8 *)val_tlv_ptr + bcm_xtlv_size(val_xtlv,
			BCM_XTLV_OPTION_NONE);
		len_of_tlvs -= (uint16)bcm_xtlv_size(val_xtlv, BCM_XTLV_OPTION_NONE);
	}
}

static void
wl_show_host_event(dhd_pub_t *dhd_pub, wl_event_msg_t *event, void *event_data,
	void *raw_event_ptr, char *eventmask)
{
	uint i, status, reason;
	bool group = FALSE, flush_txq = FALSE, link = FALSE;
	bool host_data = FALSE; /* prints  event data after the case  when set */
	const char *auth_str;
	const char *event_name;
	const uchar *buf;
	char err_msg[256], eabuf[ETHER_ADDR_STR_LEN];
	uint event_type, flags, auth_type, datalen;

	event_type = ntoh32(event->event_type);
	flags = ntoh16(event->flags);
	status = ntoh32(event->status);
	reason = ntoh32(event->reason);
	BCM_REFERENCE(reason);
	auth_type = ntoh32(event->auth_type);
	datalen = (event_data != NULL) ? ntoh32(event->datalen) : 0;

	/* debug dump of event messages */
	snprintf(eabuf, sizeof(eabuf), MACDBG, MAC2STRDBG(event->addr.octet));

	event_name = bcmevent_get_name(event_type);
	BCM_REFERENCE(event_name);

	if (flags & WLC_EVENT_MSG_LINK)
		link = TRUE;
	if (flags & WLC_EVENT_MSG_GROUP)
		group = TRUE;
	if (flags & WLC_EVENT_MSG_FLUSHTXQ)
		flush_txq = TRUE;

	switch (event_type) {
	case WLC_E_START:
		DHD_EVENT(("MACEVENT: %s(%d), MAC %s\n",
		           event_name, event_type, eabuf));
		break;
	case WLC_E_DEAUTH:
	case WLC_E_DISASSOC:
		DHD_EVENT(("MACEVENT: %s(%d), MAC %s\n",
		           event_name, event_type, eabuf));
		break;

	case WLC_E_ASSOC_IND:
	case WLC_E_REASSOC_IND:

		DHD_EVENT(("MACEVENT: %s(%d), MAC %s\n",
		           event_name, event_type, eabuf));

		break;

	case WLC_E_ASSOC:
	case WLC_E_REASSOC:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, SUCCESS\n",
			           event_name, event_type, eabuf));
		} else if (status == WLC_E_STATUS_TIMEOUT) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, TIMEOUT\n",
			           event_name, event_type, eabuf));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, FAILURE, "
			           "status %d reason %d\n",
			           event_name, event_type,
			           eabuf, (int)status, (int)reason));
		} else if (status == WLC_E_STATUS_SUPPRESS) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, SUPPRESS\n",
			           event_name, event_type, eabuf));
		} else if (status == WLC_E_STATUS_NO_ACK) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, NOACK\n",
			           event_name, event_type, eabuf));
		} else {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, unexpected status %d\n",
			           event_name, event_type, eabuf, (int)status));
		}

		break;

	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC_IND:
		DHD_EVENT(("MACEVENT: %s(%d), MAC %s, reason %d\n",
		           event_name, event_type, eabuf, (int)reason));
		break;

	case WLC_E_AUTH:
	case WLC_E_AUTH_IND:
		if (auth_type == DOT11_OPEN_SYSTEM)
			auth_str = "Open System";
		else if (auth_type == DOT11_SHARED_KEY)
			auth_str = "Shared Key";
		else if (auth_type == DOT11_SAE)
			auth_str = "SAE";
		else {
			snprintf(err_msg, sizeof(err_msg), "AUTH unknown: %d", (int)auth_type);
			auth_str = err_msg;
		}

		if (event_type == WLC_E_AUTH_IND) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, %s\n",
			           event_name, event_type, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, %s, SUCCESS\n",
			           event_name, event_type, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_TIMEOUT) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, %s, TIMEOUT\n",
			           event_name, event_type, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, %s, FAILURE, "
			           "status %d reason %d\n",
			           event_name, event_type,
			           eabuf, auth_str, (int)status, (int)reason));
		} else if (status == WLC_E_STATUS_SUPPRESS) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, %s, SUPPRESS\n",
			           event_name, event_type, eabuf, auth_str));
		} else if (status == WLC_E_STATUS_NO_ACK) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, %s, NOACK\n",
			           event_name, event_type, eabuf, auth_str));
		} else {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, %s, status %d reason %d\n",
			           event_name, event_type,
			           eabuf, auth_str, (int)status, (int)reason));
		}
		BCM_REFERENCE(auth_str);

		break;

	case WLC_E_ROAM:
		wl_show_roam_event(dhd_pub, status, datalen,
		                   event_type, event_name, eabuf, event_data);
		break;
	case WLC_E_ROAM_START:
		if (datalen >= sizeof(wlc_roam_start_event_t)) {
			const wlc_roam_start_event_t *roam_start =
				(wlc_roam_start_event_t *)event_data;
			UNUSED_PARAMETER(roam_start);
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, status %d,"
			           " reason %d, auth %d, current bss rssi %d\n",
			           event_name, event_type,
			           eabuf, (int)status, (int)reason,
			           (int)auth_type, (int)roam_start->rssi));
		} else {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, status %d, "
			           "reason %d, auth %d\n",
			           event_name, event_type,
			           eabuf, (int)status, (int)reason,
			           (int)auth_type));
		}
		break;
	case WLC_E_ROAM_PREP:
		if (datalen >= sizeof(wlc_roam_prep_event_t)) {
			const wlc_roam_prep_event_t *roam_prep =
				(wlc_roam_prep_event_t *)event_data;
			UNUSED_PARAMETER(roam_prep);
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, status %d,"
			           " reason %d, auth %d, target bss rssi %d\n",
			           event_name, event_type,
			           eabuf, (int)status, (int)reason,
			           (int)auth_type, (int)roam_prep->rssi));
		} else {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s, "
			           "status %d, reason %d, auth %d\n",
			           event_name, event_type,
			           eabuf, (int)status, (int)reason,
			           (int)auth_type));
		}
		break;
	case WLC_E_ROAM_CACHE_UPDATE:
		DHD_EVENT(("MACEVENT: %s(%d)\n", event_name, event_type));
		wl_show_roam_cache_update_event(event_name, status,
			reason, datalen, event_data);
		break;
	case WLC_E_JOIN:
	case WLC_E_SET_SSID:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s(%d), MAC %s\n",
			           event_name, event_type, eabuf));
		} else {
			if (status == WLC_E_STATUS_FAIL) {
				DHD_EVENT(("MACEVENT: %s(%d), failed status %d\n",
				           event_name, event_type, status));
			} else if (status == WLC_E_STATUS_NO_NETWORKS) {
				DHD_EVENT(("MACEVENT: %s(%d), no networks found\n",
				           event_name, event_type));
			} else {
				DHD_EVENT(("MACEVENT: %s(%d), unexpected status %d\n",
				           event_name, event_type, (int)status));
			}
		}
		break;

	case WLC_E_BEACON_RX:
		if (status == WLC_E_STATUS_SUCCESS) {
			DHD_EVENT(("MACEVENT: %s(%d), SUCCESS\n",
			           event_name, event_type));
		} else if (status == WLC_E_STATUS_FAIL) {
			DHD_EVENT(("MACEVENT: %s(%d), FAIL\n",
			           event_name, event_type));
		} else {
			DHD_EVENT(("MACEVENT: %s(%d), status %d\n",
			           event_name, event_type, status));
		}
		break;

	case WLC_E_LINK:
		DHD_EVENT(("MACEVENT: %s(%d) %s flags:0x%x status:%d reason:%d\n",
		           event_name, event_type,
		           link?"UP":"DOWN", flags, status, reason));
#ifdef PCIE_FULL_DONGLE
#endif
		BCM_REFERENCE(link);
		break;

	case WLC_E_MIC_ERROR:
		DHD_EVENT(("MACEVENT: %s(%d), MAC %s, Group %d, Flush %d\n",
		           event_name, event_type, eabuf, group, flush_txq));
		BCM_REFERENCE(group);
		BCM_REFERENCE(flush_txq);
		break;

	case WLC_E_ICV_ERROR:
	case WLC_E_UNICAST_DECODE_ERROR:
	case WLC_E_MULTICAST_DECODE_ERROR:
		DHD_EVENT(("MACEVENT: %s(%d), MAC %s\n",
		           event_name, event_type, eabuf));
		break;

	case WLC_E_TXFAIL:
		DHD_EVENT(("MACEVENT: %s(%d), RA %s status %d\n",
		           event_name, event_type, eabuf, status));
		break;

	case WLC_E_ASSOC_REQ_IE:
	case WLC_E_ASSOC_RESP_IE:
	case WLC_E_PMKID_CACHE:
		DHD_EVENT(("MACEVENT: %s(%d)\n", event_name, event_type));
		break;

	case WLC_E_SCAN_COMPLETE:
		DHD_EVENT(("MACEVENT: %s(%d)\n", event_name, event_type));
		break;
	case WLC_E_RSSI_LQM:
	case WLC_E_PFN_NET_FOUND:
	case WLC_E_PFN_NET_LOST:
	case WLC_E_PFN_SCAN_COMPLETE:
	case WLC_E_PFN_SCAN_NONE:
	case WLC_E_PFN_SCAN_ALLGONE:
	case WLC_E_PFN_GSCAN_FULL_RESULT:
	case WLC_E_PFN_SSID_EXT:
		DHD_EVENT(("PNOEVENT: %s(%d)\n", event_name, event_type));
		break;

	case WLC_E_PFN_SCAN_BACKOFF:
	case WLC_E_PFN_BSSID_SCAN_BACKOFF:
		DHD_EVENT(("PNOEVENT: %s(%d), status %d, reason %d\n",
		           event_name, event_type, (int)status, (int)reason));
		break;

	case WLC_E_PSK_SUP:
	case WLC_E_PRUNE:
		DHD_EVENT(("MACEVENT: %s(%d), status %d, reason %d\n",
		           event_name, event_type, (int)status, (int)reason));
		break;

#ifdef WIFI_ACT_FRAME
	case WLC_E_ACTION_FRAME:
		DHD_TRACE(("MACEVENT: %s(%d) Bssid %s\n",
		           event_name, event_type, eabuf));
		break;
	case WLC_E_ACTION_FRAME_COMPLETE:
		if (datalen >= sizeof(uint32)) {
			const uint32 *pktid = event_data;
			BCM_REFERENCE(pktid);
			DHD_EVENT(("MACEVENT: %s(%d) status %d, reason %d, pktid 0x%x\n",
			           event_name, event_type,
			           (int)status, (int)reason, *pktid));
		}
		break;
#endif /* WIFI_ACT_FRAME */

#ifdef SHOW_LOGTRACE
	case WLC_E_TRACE:
	{
		dhd_dbg_trace_evnt_handler(dhd_pub, event_data, raw_event_ptr, datalen);
		break;
	}
#endif /* SHOW_LOGTRACE */

	case WLC_E_RSSI:
		if (datalen >= sizeof(int)) {
			DHD_EVENT(("MACEVENT:%s(%d), val=%d\n",
			           event_name, event_type, ntoh32(*((int *)event_data))));
		}
		break;

	case WLC_E_SERVICE_FOUND:
	case WLC_E_P2PO_ADD_DEVICE:
	case WLC_E_P2PO_DEL_DEVICE:
		DHD_EVENT(("MACEVENT: %s(%d), MAC %s\n",
		           event_name, event_type, eabuf));
		break;

#ifdef BT_WIFI_HANDOBER
	case WLC_E_BT_WIFI_HANDOVER_REQ:
		DHD_EVENT(("MACEVENT: %s(%d), MAC %s\n",
		           event_name, event_type, eabuf));
		break;
#endif

	case WLC_E_CCA_CHAN_QUAL:
		/* I would like to check here that datalen >= sizeof(cca_chan_qual_event_t)
		 * but since definition of cca_chan_qual_event_t is different
		 * between blazar and legacy firmware, I will
		 * check only that datalen is bigger than 0.
		 */
		if (datalen > 0) {
			const cca_chan_qual_event_t *cca_event =
				(cca_chan_qual_event_t *)event_data;
			if ((cca_event->id == WL_CHAN_QUAL_FULLPM_CCA) ||
			    (cca_event->id == WL_CHAN_QUAL_FULLPM_CCA_OFDM_DESENSE)) {
				const cca_only_chan_qual_event_t *cca_only_event =
					(const cca_only_chan_qual_event_t *)cca_event;
				BCM_REFERENCE(cca_only_event);
				DHD_EVENT(("MACEVENT:%s(%d), MAC %s, status %d, "
				           "reason %d, auth %d, channel 0x%02x\n",
				           event_name, event_type, eabuf,
				           (int)status, (int)reason,
				           (int)auth_type, cca_event->chanspec));
				DHD_EVENT(("\tTOTAL (dur %dms me %dms notme %dms "
				           "interf %dms ts 0x%08x)\n",
				           cca_only_event->cca_busy_ext.duration,
				           cca_only_event->cca_busy_ext.congest_ibss,
				           cca_only_event->cca_busy_ext.congest_obss,
				           cca_only_event->cca_busy_ext.interference,
				           cca_only_event->cca_busy_ext.timestamp));
				DHD_EVENT(("\t  !PM (dur %dms me %dms notme %dms "
				           "interf %dms)\n",
				           cca_only_event->cca_busy_nopm.duration,
				           cca_only_event->cca_busy_nopm.congest_ibss,
				           cca_only_event->cca_busy_nopm.congest_obss,
				           cca_only_event->cca_busy_nopm.interference));
				DHD_EVENT(("\t   PM (dur %dms me %dms notme %dms "
				           "interf %dms)\n",
				           cca_only_event->cca_busy_pm.duration,
				           cca_only_event->cca_busy_pm.congest_ibss,
				           cca_only_event->cca_busy_pm.congest_obss,
				           cca_only_event->cca_busy_pm.interference));
				if (cca_event->id == WL_CHAN_QUAL_FULLPM_CCA_OFDM_DESENSE) {
					DHD_EVENT(("\t OFDM desense %d\n",
					           ((const cca_only_chan_qual_event_v2_t *)
					           cca_only_event)->ofdm_desense));
				}
			} else if (cca_event->id == WL_CHAN_QUAL_FULL_CCA) {
				DHD_EVENT(("MACEVENT:%s(%d), MAC %s, status %d, reason %d,"
				           " auth %d, channel 0x%02x (dur %dms ibss %dms "
				           "obss %dms interf %dms ts 0x%08x)\n",
				           event_name, event_type, eabuf, (int)status,
				           (int)reason, (int)auth_type, cca_event->chanspec,
				           cca_event->cca_busy_ext.duration,
				           cca_event->cca_busy_ext.congest_ibss,
				           cca_event->cca_busy_ext.congest_obss,
				           cca_event->cca_busy_ext.interference,
				           cca_event->cca_busy_ext.timestamp));
			} else if (cca_event->id == WL_CHAN_QUAL_CCA) {
				DHD_EVENT(("MACEVENT:%s(%d), MAC %s, status %d, reason %d,"
				           " auth %d, channel 0x%02x "
				           "(dur %dms busy %dms ts 0x%08x)\n",
				           event_name, event_type, eabuf, (int)status,
				           (int)reason, (int)auth_type, cca_event->chanspec,
				           cca_event->cca_busy.duration,
				           cca_event->cca_busy.congest,
				           cca_event->cca_busy.timestamp));
			} else if ((cca_event->id == WL_CHAN_QUAL_NF) ||
			           (cca_event->id == WL_CHAN_QUAL_NF_LTE)) {
				DHD_EVENT(("MACEVENT:%s(%d), MAC %s, status %d, reason %d,"
				           " auth %d, channel 0x%02x (NF[%d] %ddB)\n",
				           event_name, event_type, eabuf, (int)status,
				           (int)reason, (int)auth_type, cca_event->chanspec,
				           cca_event->id, cca_event->noise));
			} else {
				DHD_EVENT(("MACEVENT:%s(%d), MAC %s, status %d, reason %d,"
				           " auth %d, channel 0x%02x (unknown ID %d)\n",
				           event_name, event_type, eabuf, (int)status,
				           (int)reason, (int)auth_type, cca_event->chanspec,
				           cca_event->id));
			}
		}
		break;
	case WLC_E_ESCAN_RESULT:
		if (datalen >= sizeof(wl_escan_result_v2_t)) {
			const wl_escan_result_v2_t *escan_result =
				(wl_escan_result_v2_t *)event_data;
			BCM_REFERENCE(escan_result);
#ifdef OEM_ANDROID
			/* Because WLC_E_ESCAN_RESULT event log are being print too many.
			* So, DHD_EVENT() changes to be used DHD_TRACE() in HW4 platform.
			*/
			DHD_TRACE(("MACEVENT:%s(%d), MAC %s, status %d \n",
			           event_name, event_type, eabuf, (int)status));
#else
			DHD_EVENT(("MACEVENT:%s(%d), MAC %s, status %d sync-id %u\n",
			           event_name, event_type, eabuf,
			           (int)status, dtoh16(escan_result->sync_id)));
#endif /* CUSTOMER_HW4 */
		}
		break;
	case WLC_E_IF:
		if (datalen >= sizeof(struct wl_event_data_if)) {
			const struct wl_event_data_if *ifevent =
				(struct wl_event_data_if *)event_data;
			BCM_REFERENCE(ifevent);

			DHD_EVENT(("MACEVENT: %s(%d), opcode:0x%d  ifidx:%d role:%d\n",
			           event_name, event_type, ifevent->opcode,
			           ifevent->ifidx, ifevent->role));
		}
		break;
#ifdef SHOW_LOGTRACE
	case WLC_E_MSCH:
	{
		wl_mschdbg_event_handler(dhd_pub, raw_event_ptr, reason, event_data, datalen);
		break;
	}
#endif /* SHOW_LOGTRACE */

	case WLC_E_PSK_AUTH:
		DHD_EVENT(("MACEVENT: %s(%d), RA %s status %d Reason:%d\n",
		           event_name, event_type, eabuf, status, reason));
		break;
	case WLC_E_AGGR_EVENT:
		if (datalen >= sizeof(event_aggr_data_t)) {
			const event_aggr_data_t *aggrbuf = event_data;
			int j = 0, len = 0;
			const uint8 *data = aggrbuf->data;
			DHD_EVENT(("MACEVENT: %s(%d), num of events %d "
			           "total len %d sub events: ",
			           event_name, event_type,
			           aggrbuf->num_events, aggrbuf->len));
			for (j = 0; j < aggrbuf->num_events; j++)
			{
				const wl_event_msg_t * sub_event = (const wl_event_msg_t *)data;
				if (len > aggrbuf->len) {
					DHD_ERROR(("%s: Aggr events corrupted!", __FUNCTION__));
					break;
				}
				DHD_EVENT(("\n Event type: %d ", ntoh32(sub_event->event_type)));
				len += ALIGN_SIZE((ntoh32(sub_event->datalen) +
					sizeof(wl_event_msg_t)), sizeof(uint64));
				buf = (const uchar *)(data + sizeof(wl_event_msg_t));
				BCM_REFERENCE(buf);
				DHD_EVENT((" data (%d) : ", ntoh32(sub_event->datalen)));
				for (i = 0; i < ntoh32(sub_event->datalen); i++) {
					DHD_EVENT((" 0x%02x ", buf[i]));
				}
				data = aggrbuf->data + len;
			}
			DHD_EVENT(("\n"));
		}
		break;
	case WLC_E_PHY_CAL:
		{
			DHD_EVENT(("MACEVENT: %s(%d), reason:%d\n",
			           event_name, event_type, reason));
			break;
		}
	case WLC_E_NAN_CRITICAL:
		{
			DHD_EVENT(("MACEVENT: %s(%d), type:%d\n",
			           event_name, event_type, reason));
			break;
		}
	case WLC_E_NAN_NON_CRITICAL:
		{
			DHD_TRACE(("MACEVENT: %s(%d), type:%d\n",
			           event_name, event_type, reason));
			break;
		}
	case WLC_E_PROXD:
		if (datalen >= sizeof(wl_proxd_event_t)) {
			const wl_proxd_event_t *proxd =
				(wl_proxd_event_t*)event_data;
			DHD_LOG_MEM(("MACEVENT: %s(%d), event:%d, status:%d\n",
			             event_name, event_type, proxd->type, reason));
		}
		break;
	case WLC_E_RPSNOA:
		if (datalen >= sizeof(rpsnoa_stats_t)) {
			const rpsnoa_stats_t *stat = event_data;
			if (datalen == sizeof(*stat)) {
				DHD_EVENT(("MACEVENT: %s(%d), band %s, status %d, pps %d\n",
				           event_name, event_type,
				           (stat->band == WLC_BAND_2G) ? "2G":"5G",
				           stat->state, stat->last_pps));
			}
		}
		break;
	case WLC_E_WA_LQM:
		if (datalen >= sizeof(wl_event_wa_lqm_t)) {
			const wl_event_wa_lqm_t *event_wa_lqm =
				(wl_event_wa_lqm_t *)event_data;
			const bcm_xtlv_t *subevent;
			const wl_event_wa_lqm_basic_t *elqm_basic;

			if ((event_wa_lqm->ver != WL_EVENT_WA_LQM_VER) ||
			    (event_wa_lqm->len < sizeof(wl_event_wa_lqm_t) + BCM_XTLV_HDR_SIZE)) {
				DHD_ERROR(("MACEVENT: %s(%d) invalid (ver=%d len=%d)\n",
				           event_name, event_type,
				           event_wa_lqm->ver, event_wa_lqm->len));
				break;
			}

			subevent = (const bcm_xtlv_t *)event_wa_lqm->subevent;
			 if ((subevent->id != WL_EVENT_WA_LQM_BASIC) ||
			     (subevent->len < sizeof(wl_event_wa_lqm_basic_t))) {
				DHD_ERROR(("MACEVENT: %s(%d) invalid sub-type "
				           "(id=%d len=%d)\n",
				           event_name, event_type,
				           subevent->id, subevent->len));
				break;
			}

			elqm_basic = (const wl_event_wa_lqm_basic_t *)subevent->data;
			BCM_REFERENCE(elqm_basic);
			DHD_EVENT(("MACEVENT: %s(%d) (RSSI=%d SNR=%d "
			           "TxRate=%d RxRate=%d)\n",
			           event_name, event_type,
			           elqm_basic->rssi, elqm_basic->snr,
			           elqm_basic->tx_rate, elqm_basic->rx_rate));
		}
		break;

	case WLC_E_OBSS_DETECTION:
		{
			DHD_EVENT(("MACEVENT: %s(%d), type:%d\n",
			           event_name, event_type, reason));
			break;
		}

	case WLC_E_AP_BCN_MUTE:
		if (datalen >= sizeof(wlc_bcn_mute_miti_event_data_v1_t)) {
			const wlc_bcn_mute_miti_event_data_v1_t
				*bcn_mute_miti_evnt_data = event_data;
			UNUSED_PARAMETER(bcn_mute_miti_evnt_data);
			DHD_EVENT(("MACEVENT: %s(%d), reason :%d uatbtt_count: %d\n",
			           event_name, event_type,
			           reason, bcn_mute_miti_evnt_data->uatbtt_count));
		}
		break;
#ifdef WL_TWT
	case WLC_E_TWT:
		DHD_EVENT(("MACEVENT: %s(%d), type:%d\n",
		           event_name, event_type, reason));
		break;
#endif /* WL_TWT */
#ifdef WL_CFG80211
	case WLC_E_COUNTRY_CODE_CHANGED:
		DHD_EVENT(("MACEVENT: %s(%d): Country code changed to %s\n",
		           event_name, event_type, (char*)event_data));
		break;
#endif /* WL_CFG80211 */
#if defined(CSI_SUPPORT)
	case WLC_E_CSI:
		dhd_csi_event_handler(dhd_pub, event, (void *)event_data);
		break;
#endif /* CSI_SUPPORT */
        case WLC_E_OWE_INFO:
                DHD_EVENT(("MACEVENT: %s, MAC %s type:%d\n", event_name, eabuf, reason));
                break;
	default:
		DHD_INFO(("MACEVENT:%s(%d), MAC %s, status %d, reason %d, auth %d\n",
		          event_name, event_type,
		          eabuf, (int)status, (int)reason,
		          (int)auth_type));
		break;
	}

	/* show any appended data if message level is set to bytes or host_data is set */
	if ((DHD_BYTES_ON() || (host_data == TRUE)) && DHD_EVENT_ON() && datalen) {
		buf = (uchar *) event_data;
		BCM_REFERENCE(buf);
		DHD_EVENT((" data (%d) : ", datalen));
		for (i = 0; i < datalen; i++) {
			DHD_EVENT((" 0x%02x ", buf[i]));
		}
		DHD_EVENT(("\n"));
	}
} /* wl_show_host_event */
#endif /* SHOW_EVENTS */

#ifdef DNGL_EVENT_SUPPORT
/* Check whether packet is a BRCM dngl event pkt. If it is, process event data. */
int
dngl_host_event(dhd_pub_t *dhdp, void *pktdata, bcm_dngl_event_msg_t *dngl_event, size_t pktlen)
{
	bcm_dngl_event_t *pvt_data = (bcm_dngl_event_t *)pktdata;

	dngl_host_event_process(dhdp, pvt_data, dngl_event, pktlen);
	return BCME_OK;
}

#ifdef PARSE_DONGLE_HOST_EVENT
typedef struct hck_id_to_str_s {
	uint32 id;
	char *name;
} hck_id_to_str_t;

hck_id_to_str_t hck_sw_id_to_str[] = {
	{WL_HC_DD_PCIE, "WL_HC_DD_PCIE"},
	{WL_HC_DD_RX_DMA_STALL, "WL_HC_DD_RX_DMA_STALL"},
	{WL_HC_DD_RX_STALL, "WL_HC_DD_RX_STALL"},
	{WL_HC_DD_TX_STALL, "WL_HC_DD_TX_STALL"},
	{WL_HC_DD_SCAN_STALL, "WL_HC_DD_SCAN_STALL"},
	{WL_HC_DD_PHY, "WL_HC_DD_PHY"},
	{WL_HC_DD_REINIT, "WL_HC_DD_REINIT"},
	{WL_HC_DD_TXQ_STALL, "WL_HC_DD_TXQ_STALL"},
	{0, NULL}
};

hck_id_to_str_t hck_pcie_module_to_str[] = {
	{HEALTH_CHECK_PCIEDEV_INDUCED_IND, "PCIEDEV_INDUCED_IND"},
	{HEALTH_CHECK_PCIEDEV_H2D_DMA_IND, "PCIEDEV_H2D_DMA_IND"},
	{HEALTH_CHECK_PCIEDEV_D2H_DMA_IND, "PCIEDEV_D2H_DMA_IND"},
	{HEALTH_CHECK_PCIEDEV_IOCTL_STALL_IND, "PCIEDEV_IOCTL_STALL_IND"},
	{HEALTH_CHECK_PCIEDEV_D3ACK_STALL_IND, "PCIEDEV_D3ACK_STALL_IND"},
	{HEALTH_CHECK_PCIEDEV_NODS_IND, "PCIEDEV_NODS_IND"},
	{HEALTH_CHECK_PCIEDEV_LINKSPEED_FALLBACK_IND, "PCIEDEV_LINKSPEED_FALLBACK_IND"},
	{HEALTH_CHECK_PCIEDEV_DSACK_STALL_IND, "PCIEDEV_DSACK_STALL_IND"},
	{0, NULL}
};

hck_id_to_str_t hck_rx_stall_v2_to_str[] = {
	{BCM_RX_HC_RESERVED, "BCM_RX_HC_RESERVED"},
	{BCM_RX_HC_UNSPECIFIED, "BCM_RX_HC_UNSPECIFIED"},
	{BCM_RX_HC_UNICAST_DECRYPT_FAIL, "BCM_RX_HC_UNICAST_DECRYPT_FAIL"},
	{BCM_RX_HC_BCMC_DECRYPT_FAIL, "BCM_RX_HC_BCMC_DECRYPT_FAIL"},
	{BCM_RX_HC_UNICAST_REPLAY, "BCM_RX_HC_UNICAST_REPLAY"},
	{BCM_RX_HC_BCMC_REPLAY, "BCM_RX_HC_BCMC_REPLAY"},
	{BCM_RX_HC_AMPDU_DUP, "BCM_RX_HC_AMPDU_DUP"},
	{0, NULL}
};

static void
dhd_print_dongle_hck_id(uint32 id, hck_id_to_str_t *hck)
{
	while (hck->name != NULL) {
		if (hck->id == id) {
			DHD_ERROR(("DONGLE_HCK_EVENT: %s\n", hck->name));
			return;
		}
		hck++;
	}
}

void
dhd_parse_hck_common_sw_event(bcm_xtlv_t *wl_hc)
{

	wl_rx_hc_info_v2_t *hck_rx_stall_v2;
	uint16 id;

	id = ltoh16(wl_hc->id);

	if (id == WL_HC_DD_RX_STALL_V2) {
		/*  map the hck_rx_stall_v2 structure to the value of the XTLV */
		hck_rx_stall_v2 =
			(wl_rx_hc_info_v2_t*)wl_hc;
		DHD_ERROR(("type:%d len:%d if_idx:%d ac:%d pkts:%d"
			" drop:%d alert_th:%d reason:%d peer_ea:"MACF"\n",
			hck_rx_stall_v2->type,
			hck_rx_stall_v2->length,
			hck_rx_stall_v2->if_idx,
			hck_rx_stall_v2->ac,
			hck_rx_stall_v2->rx_hc_pkts,
			hck_rx_stall_v2->rx_hc_dropped_all,
			hck_rx_stall_v2->rx_hc_alert_th,
			hck_rx_stall_v2->reason,
			ETHER_TO_MACF(hck_rx_stall_v2->peer_ea)));
		dhd_print_dongle_hck_id(
				ltoh32(hck_rx_stall_v2->reason),
				hck_rx_stall_v2_to_str);
	} else {
		dhd_print_dongle_hck_id(ltoh16(wl_hc->id),
				hck_sw_id_to_str);
	}

}

#endif /* PARSE_DONGLE_HOST_EVENT */
#if defined(WL_CFGVENDOR_SEND_ALERT_EVENT) && defined(HCHK_COMMON_SW_EVENT)
static void
dhd_send_error_alert_event(dhd_pub_t *dhdp, bcm_xtlv_t *wl_hc)
{
	uint16 id;

	id = ltoh16(wl_hc->id);

	switch (id) {
		case WL_HC_DD_RX_STALL:
			dhdp->alert_reason = ALERT_RX_STALL;
			break;
		case WL_HC_DD_TX_STALL:
			dhdp->alert_reason = ALERT_TX_STALL;
			break;
		case WL_HC_DD_SCAN_STALL:
			dhdp->alert_reason = ALERT_SCAN_STALL;
			break;
		case WL_HC_DD_TXQ_STALL:
			dhdp->alert_reason = ALERT_FW_QUEUE_STALL;
			break;
		default:
			return;
	}
	dhd_os_send_alert_message(dhdp);
}
#endif /* defined(WL_CFGVENDOR_SEND_ALERT_EVENT) && defined(HCHK_COMMON_SW_EVENT)  */
void
dngl_host_event_process(dhd_pub_t *dhdp, bcm_dngl_event_t *event,
	bcm_dngl_event_msg_t *dngl_event, size_t pktlen)
{
	uint8 *p = (uint8 *)(event + 1);
	uint16 type = ntoh16_ua((void *)&dngl_event->event_type);
	uint16 datalen = ntoh16_ua((void *)&dngl_event->datalen);
	uint16 version = ntoh16_ua((void *)&dngl_event->version);

	DHD_EVENT(("VERSION:%d, EVENT TYPE:%d, DATALEN:%d\n", version, type, datalen));
	if (datalen > (pktlen - sizeof(bcm_dngl_event_t) + ETHER_TYPE_LEN)) {
		return;
	}
	if (version != BCM_DNGL_EVENT_MSG_VERSION) {
		DHD_ERROR(("%s:version mismatch:%d:%d\n", __FUNCTION__,
			version, BCM_DNGL_EVENT_MSG_VERSION));
		return;
	}
	switch (type) {
	   case DNGL_E_SOCRAM_IND:
		{
		   bcm_dngl_socramind_t *socramind_ptr = (bcm_dngl_socramind_t *)p;
		   uint16 tag = ltoh32(socramind_ptr->tag);
		   uint16 taglen = ltoh32(socramind_ptr->length);
		   p = (uint8 *)socramind_ptr->value;
		   DHD_EVENT(("Tag:%d Len:%d Datalen:%d\n", tag, taglen, datalen));
		   switch (tag) {
			case SOCRAM_IND_ASSERT_TAG:
			    {
				/*
				* The payload consists of -
				* null terminated function name padded till 32 bit boundary +
				* Line number - (32 bits)
				* Caller address (32 bits)
				*/
				char *fnname = (char *)p;
				if (datalen < (ROUNDUP(strlen(fnname) + 1, sizeof(uint32)) +
					sizeof(uint32) * 2)) {
					DHD_ERROR(("Wrong length:%d\n", datalen));
					return;
				}
				DHD_EVENT(("ASSRT Function:%s ", p));
				p += ROUNDUP(strlen(p) + 1, sizeof(uint32));
				DHD_EVENT(("Line:%d ", *(uint32 *)p));
				p += sizeof(uint32);
				DHD_EVENT(("Caller Addr:0x%x\n", *(uint32 *)p));
#ifdef PARSE_DONGLE_HOST_EVENT
				DHD_ERROR(("DONGLE_HCK_EVENT: SOCRAM_IND_ASSERT_TAG\n"));
#endif /* PARSE_DONGLE_HOST_EVENT */
				break;
			    }
			case SOCRAM_IND_TAG_HEALTH_CHECK:
			   {
				bcm_dngl_healthcheck_t *dngl_hc = (bcm_dngl_healthcheck_t *)p;
				DHD_EVENT(("SOCRAM_IND_HEALTHCHECK_TAG:%d Len:%d datalen:%d\n",
					ltoh32(dngl_hc->top_module_tag),
					ltoh32(dngl_hc->top_module_len),
					datalen));
				if (DHD_EVENT_ON()) {
					prhex("HEALTHCHECK", p, MIN(ltoh32(dngl_hc->top_module_len)
						+ BCM_XTLV_HDR_SIZE, datalen));
				}
#ifdef DHD_LOG_DUMP
				memset(dhdp->health_chk_event_data, 0, HEALTH_CHK_BUF_SIZE);
				memcpy(dhdp->health_chk_event_data, p,
						MIN(ltoh32(dngl_hc->top_module_len),
						HEALTH_CHK_BUF_SIZE));
#endif /* DHD_LOG_DUMP */
				p = (uint8 *)dngl_hc->value;

				switch (ltoh32(dngl_hc->top_module_tag)) {
					case HEALTH_CHECK_TOP_LEVEL_MODULE_PCIEDEV_RTE:
					   {
						bcm_dngl_pcie_hc_t *pcie_hc;
						pcie_hc = (bcm_dngl_pcie_hc_t *)p;
						BCM_REFERENCE(pcie_hc);
						if (ltoh32(dngl_hc->top_module_len) <
								sizeof(bcm_dngl_pcie_hc_t)) {
							DHD_ERROR(("Wrong length:%d\n",
								ltoh32(dngl_hc->top_module_len)));
							return;
						}
						DHD_EVENT(("%d:PCIE HC error:%d flag:0x%x,"
							" control:0x%x\n",
							ltoh32(pcie_hc->version),
							ltoh32(pcie_hc->pcie_err_ind_type),
							ltoh32(pcie_hc->pcie_flag),
							ltoh32(pcie_hc->pcie_control_reg)));
#ifdef PARSE_DONGLE_HOST_EVENT
						dhd_print_dongle_hck_id(
							ltoh32(pcie_hc->pcie_err_ind_type),
								hck_pcie_module_to_str);
#endif /* PARSE_DONGLE_HOST_EVENT */
						break;
					   }
#ifdef HCHK_COMMON_SW_EVENT
					case HCHK_SW_ENTITY_WL_PRIMARY:
					case HCHK_SW_ENTITY_WL_SECONDARY:
					{
						bcm_xtlv_t *wl_hc = (bcm_xtlv_t*)p;

						if (ltoh32(dngl_hc->top_module_len) <
								sizeof(bcm_xtlv_t)) {
							DHD_ERROR(("WL SW HC Wrong length:%d\n",
								ltoh32(dngl_hc->top_module_len)));
							return;
						}
						BCM_REFERENCE(wl_hc);
						DHD_EVENT(("WL SW HC type %d len %d\n",
						ltoh16(wl_hc->id), ltoh16(wl_hc->len)));

#ifdef PARSE_DONGLE_HOST_EVENT
						dhd_parse_hck_common_sw_event(wl_hc);
#endif /* PARSE_DONGLE_HOST_EVENT */
#ifdef WL_CFGVENDOR_SEND_ALERT_EVENT
						dhd_send_error_alert_event(dhdp, wl_hc);
#endif /* WL_CFGVENDOR_SEND_ALERT_EVENT */
						break;

					}
#endif /* HCHK_COMMON_SW_EVENT */
					default:
					{
						DHD_ERROR(("%s:Unknown module TAG:%d\n",
						  __FUNCTION__,
						  ltoh32(dngl_hc->top_module_tag)));
						break;
					}
				}
				break;
			   }
			default:
			   DHD_ERROR(("%s:Unknown TAG\n", __FUNCTION__));
			   if (p && DHD_EVENT_ON()) {
				   prhex("SOCRAMIND", p, taglen);
			   }
			   break;
		   }
		   break;
		}
	   default:
		DHD_ERROR(("%s:Unknown DNGL Event Type:%d\n", __FUNCTION__, type));
		if (p && DHD_EVENT_ON()) {
			prhex("SOCRAMIND", p, datalen);
		}
		break;
	}
#ifdef DHD_FW_COREDUMP
	if (dhdp->memdump_enabled) {
		dhdp->memdump_type = DUMP_TYPE_DONGLE_HOST_EVENT;
		if (
#ifdef GDB_PROXY
			!dhdp->gdb_proxy_active &&
#endif /* GDB_PROXY */
			dhd_schedule_socram_dump(dhdp)) {
				DHD_ERROR(("%s: socram dump failed\n", __FUNCTION__));
		}
	}
#else
	dhd_dbg_send_urgent_evt(dhdp, p, datalen);
#endif /* DHD_FW_COREDUMP */

}

#endif /* DNGL_EVENT_SUPPORT */

/* Stub for now. Will become real function as soon as shim
 * is being integrated to Android, Linux etc.
 */
int
wl_event_process_default(wl_event_msg_t *event, struct wl_evt_pport *evt_pport)
{
	return BCME_OK;
}

int
wl_event_process(dhd_pub_t *dhd_pub, int *ifidx, void *pktdata,
	uint pktlen, void **data_ptr, void *raw_event)
{
	wl_evt_pport_t evt_pport;
	wl_event_msg_t event;
	bcm_event_msg_u_t evu;
	int ret;

	/* make sure it is a BRCM event pkt and record event data */
	ret = wl_host_event_get_data(pktdata, pktlen, &evu);
	if (ret != BCME_OK) {
		return ret;
	}

	memcpy(&event, &evu.event, sizeof(wl_event_msg_t));

	/* convert event from network order to host order */
	wl_event_to_host_order(&event);

	/* record event params to evt_pport */
	evt_pport.dhd_pub = dhd_pub;
	evt_pport.ifidx = ifidx;
	evt_pport.pktdata = pktdata;
	evt_pport.data_ptr = data_ptr;
	evt_pport.raw_event = raw_event;
	evt_pport.data_len = pktlen;

	ret = wl_event_process_default(&event, &evt_pport);

	return ret;
} /* wl_event_process */

/* Check whether packet is a BRCM event pkt. If it is, record event data. */
int
wl_host_event_get_data(void *pktdata, uint pktlen, bcm_event_msg_u_t *evu)
{
	int ret;

	ret = is_wlc_event_frame(pktdata, pktlen, 0, evu);
#ifndef WL_MON_OWN_PKT
	if (ret != BCME_OK) {
		DHD_ERROR(("%s: Invalid event frame, err = %d\n",
			__FUNCTION__, ret));
	}
#endif

	return ret;
}

int
wl_process_host_event(dhd_pub_t *dhd_pub, int *ifidx, void *pktdata, uint pktlen,
	wl_event_msg_t *event, void **data_ptr, void *raw_event)
{
	bcm_event_t *pvt_data = (bcm_event_t *)pktdata;
	bcm_event_msg_u_t evu;
	uint8 *event_data;
	uint32 type, status, datalen, reason;
	uint16 flags;
	uint evlen;
	int ret;
	uint16 usr_subtype;
#if defined(__linux__)
	dhd_if_t *ifp = NULL;
	BCM_REFERENCE(ifp);
#endif /* DHD_POST_EAPOL_M1_AFTER_ROAM_EVT */

	ret = wl_host_event_get_data(pktdata, pktlen, &evu);
	if (ret != BCME_OK) {
		return ret;
	}

	usr_subtype = ntoh16_ua((void *)&pvt_data->bcm_hdr.usr_subtype);
	switch (usr_subtype) {
	case BCMILCP_BCM_SUBTYPE_EVENT:
		memcpy(event, &evu.event, sizeof(wl_event_msg_t));
		*data_ptr = &pvt_data[1];
		break;
	case BCMILCP_BCM_SUBTYPE_DNGLEVENT:
#ifdef DNGL_EVENT_SUPPORT
		/* If it is a DNGL event process it first */
		if (dngl_host_event(dhd_pub, pktdata, &evu.dngl_event, pktlen) == BCME_OK) {
			/*
			 * Return error purposely to prevent DNGL event being processed
			 * as BRCM event
			 */
			return BCME_ERROR;
		}
#endif /* DNGL_EVENT_SUPPORT */
		return BCME_NOTFOUND;
	default:
		return BCME_NOTFOUND;
	}

	/* start wl_event_msg process */
	event_data = *data_ptr;
	type = ntoh32_ua((void *)&event->event_type);
	flags = ntoh16_ua((void *)&event->flags);
	status = ntoh32_ua((void *)&event->status);
	reason = ntoh32_ua((void *)&event->reason);
	datalen = ntoh32_ua((void *)&event->datalen);
	evlen = datalen + sizeof(bcm_event_t);

	DHD_TRACE(("%s: event_type=%d(%s), flags=%d, datalen=%d, status=%d,"
	           " reason=%d, auth_type=%d(0x%X), "
	           "scb=%02X:%02X:%02X:%02X:%02X:%02X, ifidx=%d\n",
	           __FUNCTION__, type, bcmevent_get_name(type),
	           flags, datalen, status, reason, 0, 0,
	           event->addr.octet[0], event->addr.octet[1], event->addr.octet[2],
	           event->addr.octet[3], event->addr.octet[4], event->addr.octet[5],
	           event->ifidx));

	switch (type) {
#ifdef PROP_TXSTATUS
	case WLC_E_FIFO_CREDIT_MAP:
		dhd_wlfc_enable(dhd_pub);
		dhd_wlfc_FIFOcreditmap_event(dhd_pub, event_data);
		WLFC_DBGMESG(("WLC_E_FIFO_CREDIT_MAP:(AC0,AC1,AC2,AC3),(BC_MC),(OTHER): "
			"(%d,%d,%d,%d),(%d),(%d)\n", event_data[0], event_data[1],
			event_data[2],
			event_data[3], event_data[4], event_data[5]));
		break;

	case WLC_E_BCMC_CREDIT_SUPPORT:
		dhd_wlfc_BCMCCredit_support_event(dhd_pub);
		break;
#ifdef LIMIT_BORROW
	case WLC_E_ALLOW_CREDIT_BORROW:
		dhd_wlfc_disable_credit_borrow_event(dhd_pub, event_data);
		break;
#endif /* LIMIT_BORROW */
#endif /* PROP_TXSTATUS */

	case WLC_E_ULP:
		break;
	case WLC_E_TDLS_PEER_EVENT:
#if defined(WLTDLS) && defined(PCIE_FULL_DONGLE)
		{
			dhd_tdls_event_handler(dhd_pub, event);
		}
#endif
		break;

	case WLC_E_IF:
		{
		struct wl_event_data_if *ifevent = (struct wl_event_data_if *)event_data;

		/* Ignore the event if NOIF is set */
		if (ifevent->reserved & WLC_E_IF_FLAGS_BSSCFG_NOIF) {
			DHD_ERROR(("WLC_E_IF: NO_IF set, event Ignored\r\n"));
			return (BCME_UNSUPPORTED);
		}
#ifdef PCIE_FULL_DONGLE
		dhd_update_interface_flow_info(dhd_pub, ifevent->ifidx,
			ifevent->opcode, ifevent->role);
#endif
#ifdef PROP_TXSTATUS
		{
			uint8* ea = pvt_data->eth.ether_dhost;
			WLFC_DBGMESG(("WLC_E_IF: idx:%d, action:%s, "
			              "iftype:%s, ["MACDBG"]\n",
			              ifevent->ifidx,
			              ((ifevent->opcode == WLC_E_IF_ADD) ? "ADD":"DEL"),
			              ((ifevent->role == 0) ? "STA":"AP "),
			              MAC2STRDBG(ea)));
			(void)ea;

			if (ifevent->opcode == WLC_E_IF_CHANGE)
				dhd_wlfc_interface_event(dhd_pub,
					eWLFC_MAC_ENTRY_ACTION_UPDATE,
					ifevent->ifidx, ifevent->role, ea);
			else
				dhd_wlfc_interface_event(dhd_pub,
					((ifevent->opcode == WLC_E_IF_ADD) ?
					eWLFC_MAC_ENTRY_ACTION_ADD : eWLFC_MAC_ENTRY_ACTION_DEL),
					ifevent->ifidx, ifevent->role, ea);

			/* dhd already has created an interface by default, for 0 */
			if (ifevent->ifidx == 0)
				break;
		}
#endif /* PROP_TXSTATUS */

		if (ifevent->ifidx > 0 && ifevent->ifidx < DHD_MAX_IFS) {
			if (ifevent->opcode == WLC_E_IF_ADD) {
				if (dhd_event_ifadd(dhd_pub->info, ifevent, event->ifname,
					event->addr.octet)) {

					DHD_ERROR(("%s: dhd_event_ifadd failed ifidx: %d  %s\n",
						__FUNCTION__, ifevent->ifidx, event->ifname));
					return (BCME_ERROR);
				}
			} else if (ifevent->opcode == WLC_E_IF_DEL) {
#ifdef PCIE_FULL_DONGLE
				dhd_flow_rings_delete(dhd_pub,
					(uint8)dhd_ifname2idx(dhd_pub->info, event->ifname));
#endif /* PCIE_FULL_DONGLE */
				dhd_event_ifdel(dhd_pub->info, ifevent, event->ifname,
					event->addr.octet);
			} else if (ifevent->opcode == WLC_E_IF_CHANGE) {
#ifdef WL_CFG80211
				dhd_event_ifchange(dhd_pub->info, ifevent, event->ifname,
					event->addr.octet);
#endif /* WL_CFG80211 */
			}
		} else {
#if !defined(PROP_TXSTATUS) && !defined(PCIE_FULL_DONGLE) && defined(WL_CFG80211)
			DHD_INFO(("%s: Invalid ifidx %d for %s\n",
			   __FUNCTION__, ifevent->ifidx, event->ifname));
#endif /* !PROP_TXSTATUS && !PCIE_FULL_DONGLE && WL_CFG80211 */
		}
		/* send up the if event: btamp user needs it */
		*ifidx = dhd_ifname2idx(dhd_pub->info, event->ifname);
		/* push up to external supp/auth */
		dhd_event(dhd_pub->info, (char *)pvt_data, evlen, *ifidx);
		break;
	}

	case WLC_E_NDIS_LINK:
		break;
	case WLC_E_PFN_NET_FOUND:
	case WLC_E_PFN_SCAN_ALLGONE: /* share with WLC_E_PFN_BSSID_NET_LOST */
	case WLC_E_PFN_NET_LOST:
		break;
#if defined(OEM_ANDROID) && defined(PNO_SUPPORT)
	case WLC_E_PFN_BSSID_NET_FOUND:
	case WLC_E_PFN_BEST_BATCHING:
		dhd_pno_event_handler(dhd_pub, event, (void *)event_data);
		break;
#endif /* #if defined(OEM_ANDROID) && defined(PNO_SUPPORT) */
#if defined(RTT_SUPPORT)
	case WLC_E_PROXD:
#ifndef WL_CFG80211
		dhd_rtt_event_handler(dhd_pub, event, (void *)event_data);
#endif /* WL_CFG80211 */
		break;
#endif /* RTT_SUPPORT */
		/* These are what external supplicant/authenticator wants */
	case WLC_E_ASSOC_IND:
	case WLC_E_AUTH_IND:
	case WLC_E_REASSOC_IND:
		dhd_findadd_sta(dhd_pub,
			dhd_ifname2idx(dhd_pub->info, event->ifname),
			&event->addr.octet);
		break;
#if defined(DHD_FW_COREDUMP)
	case WLC_E_PSM_WATCHDOG:
		DHD_ERROR(("%s: WLC_E_PSM_WATCHDOG event received : \n", __FUNCTION__));
		if (dhd_socram_dump(dhd_pub->bus) != BCME_OK) {
			DHD_ERROR(("%s: socram dump ERROR : \n", __FUNCTION__));
		}
	break;
#endif
	case WLC_E_NATOE_NFCT:
#ifdef WL_NATOE
		DHD_EVENT(("%s: WLC_E_NATOE_NFCT event received \n", __FUNCTION__));
		dhd_natoe_ct_event(dhd_pub, event_data);
#endif /* WL_NATOE */
	break;
	case WLC_E_SLOTTED_BSS_PEER_OP:
		DHD_EVENT(("%s: WLC_E_SLOTTED_BSS_PEER_OP event received for peer: "
			"" MACDBG ", status = %d\n",
			__FUNCTION__, MAC2STRDBG(event->addr.octet), status));
		if (status == WLC_E_STATUS_SLOTTED_PEER_ADD) {
			dhd_findadd_sta(dhd_pub, dhd_ifname2idx(dhd_pub->info,
				event->ifname), &event->addr.octet);
		} else if (status == WLC_E_STATUS_SLOTTED_PEER_DEL) {
			uint8 ifindex = (uint8)dhd_ifname2idx(dhd_pub->info, event->ifname);
			BCM_REFERENCE(ifindex);
			dhd_del_sta(dhd_pub, dhd_ifname2idx(dhd_pub->info,
				event->ifname), &event->addr.octet);
#ifdef PCIE_FULL_DONGLE
			dhd_flow_rings_delete_for_peer(dhd_pub, ifindex,
				(char *)&event->addr.octet[0]);
#endif
		} else {
			DHD_ERROR(("%s: WLC_E_SLOTTED_BSS_PEER_OP: Status is not expected = %d\n",
				__FUNCTION__, status));
		}
		break;
#ifdef DHD_POST_EAPOL_M1_AFTER_ROAM_EVT
	case WLC_E_REASSOC:
		ifp = dhd_get_ifp(dhd_pub, event->ifidx);

		if (!ifp)
			break;

		/* Consider both STA and GC due to CSA.
		 * Drop EAPOL M1 frame only if roam is done to the same BSSID.
		 * wpa_supplicant only handles different BSSID case.
		 */
		if ((status == WLC_E_STATUS_SUCCESS) &&
			(IS_STA_IFACE(ndev_to_wdev(ifp->net)) || IS_P2P_GC(ndev_to_wdev(ifp->net))) &&
			wl_cfg80211_is_event_from_connected_bssid(ifp->net, event, event->ifidx)) {
			ifp->recv_reassoc_evt = TRUE;
		}
		break;
#endif /* DHD_POST_EAPOL_M1_AFTER_ROAM_EVT */
	case WLC_E_LINK:
#if defined(DHD_UPDATE_INTF_MAC)
		if ((WLC_E_LINK == type) && (WLC_EVENT_MSG_LINK & flags)) {
			dhd_event_iflink(dhd_pub->info, event,
			                 (struct wl_event_data_if *)event_data,
			                 event->ifname, event->addr.octet);
		}
#endif /* defined(DHD_UPDATE_INTF_MAC) */
#ifdef PCIE_FULL_DONGLE
		if (dhd_update_interface_link_status(dhd_pub, (uint8)dhd_ifname2idx(dhd_pub->info,
			event->ifname), (uint8)flags) != BCME_OK) {
			DHD_ERROR(("%s: dhd_update_interface_link_status Failed.\n",
				__FUNCTION__));
			break;
		}
		if (!flags) {
			DHD_ERROR(("%s: Deleting all STA from assoc list and flowrings.\n",
				__FUNCTION__));
			/* Delete all sta and flowrings */
			dhd_del_all_sta(dhd_pub, dhd_ifname2idx(dhd_pub->info, event->ifname));
			dhd_flow_rings_delete(dhd_pub, (uint8)dhd_ifname2idx(dhd_pub->info,
				event->ifname));
		}
#endif /* PCIE_FULL_DONGLE */
		/* fall through */
		fallthrough;
	case WLC_E_DEAUTH:
	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC:
	case WLC_E_DISASSOC_IND:
#ifdef PCIE_FULL_DONGLE
		if (type != WLC_E_LINK) {
			uint8 ifindex = (uint8)dhd_ifname2idx(dhd_pub->info, event->ifname);
			uint8 role = dhd_flow_rings_ifindex2role(dhd_pub, ifindex);
			uint8 del_sta = TRUE;
#ifdef WL_CFG80211
			if (role == WLC_E_IF_ROLE_STA &&
				!wl_cfg80211_is_roam_offload(dhd_idx2net(dhd_pub, ifindex)) &&
					!wl_cfg80211_is_event_from_connected_bssid(
						dhd_idx2net(dhd_pub, ifindex), event, *ifidx)) {
				del_sta = FALSE;
			}
#endif /* WL_CFG80211 */
			DHD_EVENT(("%s: Link event %d, flags %x, status %x, role %d, del_sta %d\n",
				__FUNCTION__, type, flags, status, role, del_sta));

			if (del_sta) {
				DHD_EVENT(("%s: Deleting STA " MACDBG "\n",
					__FUNCTION__, MAC2STRDBG(event->addr.octet)));

				dhd_del_sta(dhd_pub, dhd_ifname2idx(dhd_pub->info,
					event->ifname), &event->addr.octet);
				/* Delete all flowrings for STA and P2P Client */
				if (role == WLC_E_IF_ROLE_STA || role == WLC_E_IF_ROLE_P2P_CLIENT) {
					dhd_flow_rings_delete(dhd_pub, ifindex);
				} else {
					dhd_flow_rings_delete_for_peer(dhd_pub, ifindex,
						(char *)&event->addr.octet[0]);
				}
			}
		}
#endif /* PCIE_FULL_DONGLE */
#ifdef DHD_POST_EAPOL_M1_AFTER_ROAM_EVT
		ifp = dhd_get_ifp(dhd_pub, event->ifidx);
		if (ifp) {
			ifp->recv_reassoc_evt = FALSE;
			ifp->post_roam_evt = FALSE;
		}
#endif /* DHD_POST_EAPOL_M1_AFTER_ROAM_EVT */
		/* fall through */
		fallthrough;
	default:
		*ifidx = dhd_ifname2idx(dhd_pub->info, event->ifname);
		/* push up to external supp/auth */
		dhd_event(dhd_pub->info, (char *)pvt_data, evlen, *ifidx);
		DHD_TRACE(("%s: MAC event %d, flags %x, status %x\n",
			__FUNCTION__, type, flags, status));
		BCM_REFERENCE(flags);
		BCM_REFERENCE(status);
		BCM_REFERENCE(reason);

		break;
	}
#if defined(STBAP)
	/* For routers, EAPD will be working on these events.
	 * Overwrite interface name to that event is pushed
	 * to host with its registered interface name
	 */
	memcpy(pvt_data->event.ifname, dhd_ifname(dhd_pub, *ifidx), IFNAMSIZ);
#endif

#ifdef DHD_STATUS_LOGGING
	if (dhd_pub->statlog) {
		dhd_statlog_process_event(dhd_pub, type, *ifidx,
			status, reason, flags);
	}
#endif /* DHD_STATUS_LOGGING */

#ifdef SHOW_EVENTS
	/* force to call for some logical processing like timeout timer */
	wl_show_host_event(dhd_pub, event,
		(void *)event_data, raw_event, dhd_pub->enable_log);
#endif /* SHOW_EVENTS */

	return (BCME_OK);
} /* wl_process_host_event */

int
wl_host_event(dhd_pub_t *dhd_pub, int *ifidx, void *pktdata, uint pktlen,
	wl_event_msg_t *event, void **data_ptr, void *raw_event)
{
	return wl_process_host_event(dhd_pub, ifidx, pktdata, pktlen, event, data_ptr,
			raw_event);
}

void
dhd_print_buf(void *pbuf, int len, int bytes_per_line)
{
#ifdef DHD_DEBUG
	int i, j = 0;
	unsigned char *buf = pbuf;

	if (bytes_per_line == 0) {
		bytes_per_line = len;
	}

	for (i = 0; i < len; i++) {
		printf("%2.2x", *buf++);
		j++;
		if (j == bytes_per_line) {
			printf("\n");
			j = 0;
		} else {
			printf(":");
		}
	}
	printf("\n");
#endif /* DHD_DEBUG */
}
#ifndef strtoul
#define strtoul(nptr, endptr, base) bcm_strtoul((nptr), (endptr), (base))
#endif

#if defined(PKT_FILTER_SUPPORT) || defined(DHD_PKT_LOGGING)
/* Convert user's input in hex pattern to byte-size mask */
int
wl_pattern_atoh(char *src, char *dst)
{
	int i;
	if (strncmp(src, "0x", 2) != 0 &&
	    strncmp(src, "0X", 2) != 0) {
		DHD_ERROR(("Mask invalid format. Needs to start with 0x\n"));
		return -1;
	}
	src = src + 2; /* Skip past 0x */
	if (strlen(src) % 2 != 0) {
		DHD_ERROR(("Mask invalid format. Needs to be of even length\n"));
		return -1;
	}
	for (i = 0; *src != '\0'; i++) {
		char num[3];
		bcm_strncpy_s(num, sizeof(num), src, 2);
		num[2] = '\0';
		dst[i] = (uint8)strtoul(num, NULL, 16);
		src += 2;
	}
	return i;
}

int
pattern_atoh_len(char *src, char *dst, int len)
{
	int i;
	if (strncmp(src, "0x", HD_PREFIX_SIZE) != 0 &&
			strncmp(src, "0X", HD_PREFIX_SIZE) != 0) {
		DHD_ERROR(("Mask invalid format. Needs to start with 0x\n"));
		return -1;
	}
	src = src + HD_PREFIX_SIZE; /* Skip past 0x */
	if (strlen(src) % HD_BYTE_SIZE != 0) {
		DHD_ERROR(("Mask invalid format. Needs to be of even length\n"));
		return -1;
	}
	for (i = 0; *src != '\0'; i++) {
		char num[HD_BYTE_SIZE + 1];

		if (i > len - 1) {
			DHD_ERROR(("pattern not in range, idx: %d len: %d\n", i, len));
			return -1;
		}
		bcm_strncpy_s(num, sizeof(num), src, HD_BYTE_SIZE);
		num[HD_BYTE_SIZE] = '\0';
		dst[i] = (uint8)strtoul(num, NULL, 16);
		src += HD_BYTE_SIZE;
	}
	return i;
}
#endif /* PKT_FILTER_SUPPORT || DHD_PKT_LOGGING */

#ifdef PKT_FILTER_SUPPORT
int
dhd_pktfilter_mode_change(dhd_pub_t * dhd, int enable)
{
	int   rc = 0;
	uint  operation_mode = PKT_FILTER_MODE_DISABLE;

	operation_mode =  dhd_master_mode
	                | ((enable)?(0):(PKT_FILTER_MODE_DISABLE));

	/* Contorl the master mode */
	rc = dhd_wl_ioctl_set_intiovar(dhd, "pkt_filter_mode", operation_mode, WLC_SET_VAR, TRUE, 0);

	return rc;
}

void
dhd_pktfilter_offload_enable(dhd_pub_t * dhd, char *arg, int enable, int change_filter_mode)
{
	char				*argv[8];
	int					i = 0;
	const char			*str;
	int					buf_len;
	int					str_len;
	char				*arg_save = 0, *arg_org = 0;
	int					rc;
	char				buf[32] = {0};
	wl_pkt_filter_enable_t	enable_parm;
	wl_pkt_filter_enable_t	* pkt_filterp;

	if (!arg)
		return;

	if (!(arg_save = MALLOC(dhd->osh, strlen(arg) + 1))) {
		DHD_ERROR(("%s: malloc failed\n", __FUNCTION__));
		goto fail;
	}
	arg_org = arg_save;
	memcpy(arg_save, arg, strlen(arg) + 1);

	argv[i] = bcmstrtok(&arg_save, " ", 0);

	i = 0;
	if (argv[i] == NULL) {
		DHD_ERROR(("No args provided\n"));
		goto fail;
	}

	str = "pkt_filter_enable";
	str_len = strlen(str);
	bcm_strncpy_s(buf, sizeof(buf) - 1, str, sizeof(buf) - 1);
	buf[ sizeof(buf) - 1 ] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_enable_t *)(buf + str_len + 1);

	/* Parse packet filter id. */
	enable_parm.id = htod32(strtoul(argv[i], NULL, 0));

	/* Parse enable/disable value. */
	enable_parm.enable = htod32(enable);

	buf_len += sizeof(enable_parm);
	memcpy((char *)pkt_filterp,
	       &enable_parm,
	       sizeof(enable_parm));

	/* Enable/disable the specified filter. */
	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);
	rc = rc >= 0 ? 0 : rc;
	if (rc) {
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		           __FUNCTION__, arg, rc));
		dhd_set_packet_filter(dhd);
		rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);
		rc = rc >= 0 ? 0 : rc;
		if (rc) {
			DHD_TRACE_HW4(("%s: 2nd retry failed to add pktfilter %s, retcode = %d\n",
			               __FUNCTION__, arg, rc));
		} else {
			DHD_TRACE_HW4(("%s: 2nd retry successfully added pktfilter %s\n",
			               __FUNCTION__, arg));
		}
	}
	else
		DHD_TRACE(("%s: successfully added pktfilter %s\n",
		           __FUNCTION__, arg));

		/* Contorl the master mode if needs */
	if (change_filter_mode) {
		rc = dhd_pktfilter_mode_change(dhd, enable);
		rc = rc >= 0 ? 0 : rc;
		if (rc) {
			DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
			           __FUNCTION__, arg, rc));
		}
			}

fail:
	if (arg_org)
		MFREE(dhd->osh, arg_org, strlen(arg) + 1);
}

/* Packet filter section: extended filters have named offsets, add table here */
typedef struct {
	char *name;
	uint16 base;
} wl_pfbase_t;

static wl_pfbase_t basenames[] = { WL_PKT_FILTER_BASE_NAMES };

static int
wl_pkt_filter_base_parse(char *name)
{
	uint i;
	char *bname, *uname;

	for (i = 0; i < ARRAYSIZE(basenames); i++) {
		bname = basenames[i].name;
		for (uname = name; *uname; bname++, uname++) {
			if (*bname != bcm_toupper(*uname)) {
				break;
			}
		}
		if (!*uname && !*bname) {
			break;
		}
	}

	if (i < ARRAYSIZE(basenames)) {
		return basenames[i].base;
	} else {
		return -1;
	}
}

void
dhd_pktfilter_offload_set(dhd_pub_t * dhd, char *arg)
{
	const char			*str;
	wl_pkt_filter_t			pkt_filter;
	wl_pkt_filter_t			*pkt_filterp;
	int				buf_len;
	int				str_len;
	int				rc = -1;
	uint32				mask_size;
	uint32				pattern_size;
	char				*argv[MAXPKT_ARG] = {0}, * buf = 0;
	int				i = 0;
	char				*arg_save = 0, *arg_org = 0;

	if (!arg)
		return;

	if (!(arg_save = MALLOC(dhd->osh, strlen(arg) + 1))) {
		DHD_ERROR(("%s: malloc failed\n", __FUNCTION__));
		goto fail;
	}

	arg_org = arg_save;

	if (!(buf = MALLOC(dhd->osh, MAX_PKTFLT_BUF_SIZE))) {
		DHD_ERROR(("%s: malloc failed\n", __FUNCTION__));
		goto fail;
	}

	memset(buf, 0, MAX_PKTFLT_BUF_SIZE);
	memcpy(arg_save, arg, strlen(arg) + 1);

	if (strlen(arg) > MAX_PKTFLT_BUF_SIZE) {
		DHD_ERROR(("Not enough buffer %d < %d\n", (int)strlen(arg), (int)sizeof(buf)));
		goto fail;
	}

	argv[i] = bcmstrtok(&arg_save, " ", 0);
	while (argv[i++]) {
		if (i >= MAXPKT_ARG) {
			DHD_ERROR(("Invalid args provided\n"));
			goto fail;
		}
		argv[i] = bcmstrtok(&arg_save, " ", 0);
	}

	i = 0;
	if (argv[i] == NULL) {
		DHD_ERROR(("No args provided\n"));
		goto fail;
	}

	str = "pkt_filter_add";
	str_len = strlen(str);
	bcm_strncpy_s(buf, MAX_PKTFLT_BUF_SIZE, str, 1 + str_len);
	buf[ str_len ] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (wl_pkt_filter_t *) (buf + str_len + 1);

	/* Parse packet filter id. */
	pkt_filter.id = htod32(strtoul(argv[i], NULL, 0));

	if (argv[++i] == NULL) {
		DHD_ERROR(("Polarity not provided\n"));
		goto fail;
	}

	/* Parse filter polarity. */
	pkt_filter.negate_match = htod32(strtoul(argv[i], NULL, 0));

	if (argv[++i] == NULL) {
		DHD_ERROR(("Filter type not provided\n"));
		goto fail;
	}

	/* Parse filter type. */
	pkt_filter.type = htod32(strtoul(argv[i], NULL, 0));
	if ((pkt_filter.type == 0) || (pkt_filter.type == 1)) {
		if (argv[++i] == NULL) {
			DHD_ERROR(("Offset not provided\n"));
			goto fail;
		}

		/* Parse pattern filter offset. */
		pkt_filter.u.pattern.offset = htod32(strtoul(argv[i], NULL, 0));

		if (argv[++i] == NULL) {
			DHD_ERROR(("Bitmask not provided\n"));
			goto fail;
		}

		/* Parse pattern filter mask. */
		rc  = wl_pattern_atoh(argv[i],
			(char *) pkt_filterp->u.pattern.mask_and_pattern);

		if (rc == -1) {
			DHD_ERROR(("Rejecting: %s\n", argv[i]));
			goto fail;
		}
		mask_size = htod32(rc);
		if (argv[++i] == NULL) {
			DHD_ERROR(("Pattern not provided\n"));
			goto fail;
		}

		/* Parse pattern filter pattern. */
		rc = wl_pattern_atoh(argv[i],
			(char *) &pkt_filterp->u.pattern.mask_and_pattern[rc]);

		if (rc == -1) {
			DHD_ERROR(("Rejecting: %s\n", argv[i]));
			goto fail;
		}
		pattern_size = htod32(rc);
		if (mask_size != pattern_size) {
			DHD_ERROR(("Mask and pattern not the same size\n"));
			goto fail;
		}

		pkt_filter.u.pattern.size_bytes = mask_size;
		buf_len += WL_PKT_FILTER_FIXED_LEN;
		buf_len += (WL_PKT_FILTER_PATTERN_FIXED_LEN + 2 * rc);

		/* Keep-alive attributes are set in local	variable (keep_alive_pkt), and
		 * then memcpy'ed into buffer (keep_alive_pktp) since there is no
		 * guarantee that the buffer is properly aligned.
		 */
		memcpy((char *)pkt_filterp,
			&pkt_filter,
			WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_FIXED_LEN);
	} else if ((pkt_filter.type == 2) || (pkt_filter.type == 6)) {
		int list_cnt = 0;
		char *endptr = NULL;
		wl_pkt_filter_pattern_listel_t *pf_el =
			(wl_pkt_filter_pattern_listel_t *)&pkt_filterp->u.patlist.patterns[0];

		while (argv[++i] != NULL) {
			/* Check valid buffer size. */
			if ((buf_len + MAX_PKTFLT_FIXED_BUF_SIZE) > MAX_PKTFLT_BUF_SIZE) {
				DHD_ERROR(("buffer over length MAX_PKTFLT_FIXED_BUF_SIZE\n"));
				goto fail;
			}

			/* Parse pattern filter base and offset. */
			if (bcm_isdigit(*argv[i])) {
				/* Numeric base */
				rc = strtoul(argv[i], &endptr, 0);
			} else {
				endptr = strchr(argv[i], ':');
				if (endptr) {
					*endptr = '\0';
					rc = wl_pkt_filter_base_parse(argv[i]);
					if (rc == -1) {
						printf("Invalid base %s\n", argv[i]);
						goto fail;
					}
					*endptr = ':';
				}
			}

			if (endptr == NULL) {
				printf("Invalid [base:]offset format: %s\n", argv[i]);
				goto fail;
			}

			if (*endptr == ':') {
				pf_el->base_offs = htod16(rc);
				rc = strtoul(endptr + 1, &endptr, 0);
			} else {
				/* Must have had a numeric offset only */
				pf_el->base_offs = htod16(0);
			}

			if (*endptr) {
				printf("Invalid [base:]offset format: %s\n", argv[i]);
				goto fail;
			}
			if (rc > 0x0000FFFF) {
				printf("Offset too large\n");
				goto fail;
			}
			pf_el->rel_offs = htod16(rc);

			/* Clear match_flag (may be set in parsing which follows) */
			pf_el->match_flags = htod16(0);

			/* Parse pattern filter mask and pattern directly into ioctl buffer */
			if (argv[++i] == NULL) {
				printf("Bitmask not provided\n");
				goto fail;
			}
			rc = wl_pattern_atoh(argv[i], (char*)pf_el->mask_and_data);
			if ((rc == -1) || (rc > MAX_PKTFLT_FIXED_PATTERN_SIZE)) {
				printf("Rejecting: %s\n", argv[i]);
				goto fail;
			}
			mask_size = htod16(rc);

			if (argv[++i] == NULL) {
				printf("Pattern not provided\n");
				goto fail;
			}

			endptr = argv[i];
			if (*endptr == '!') {
				pf_el->match_flags =
					htod16(WL_PKT_FILTER_MFLAG_NEG);
				if (*(++endptr) == '\0') {
					printf("Pattern not provided\n");
					goto fail;
				}
			}
			rc = wl_pattern_atoh(endptr, (char*)&pf_el->mask_and_data[rc]);
			if ((rc == -1) || (rc > MAX_PKTFLT_FIXED_PATTERN_SIZE)) {
				printf("Rejecting: %s\n", argv[i]);
				goto fail;
			}
			pattern_size = htod16(rc);

			if (mask_size != pattern_size) {
				printf("Mask and pattern not the same size\n");
				goto fail;
			}

			pf_el->size_bytes = mask_size;

			/* Account for the size of this pattern element */
			buf_len += WL_PKT_FILTER_PATTERN_LISTEL_FIXED_LEN + 2 * rc;

			/* Move to next element location in ioctl buffer */
			pf_el = (wl_pkt_filter_pattern_listel_t*)
				((uint8*)pf_el + WL_PKT_FILTER_PATTERN_LISTEL_FIXED_LEN + 2 * rc);

			/* Count list element */
			list_cnt++;
		}

		/* Account for initial fixed size, and copy initial fixed fields */
		buf_len += WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_LIST_FIXED_LEN;

		if (buf_len > MAX_PKTFLT_BUF_SIZE) {
			DHD_ERROR(("buffer over length MAX_PKTFLT_BUF_SIZE\n"));
			goto fail;
		}

		/* Update list count and total size */
		pkt_filter.u.patlist.list_cnt = list_cnt;
		pkt_filter.u.patlist.PAD1[0] = 0;
		pkt_filter.u.patlist.totsize = buf + buf_len - (char*)pkt_filterp;
		pkt_filter.u.patlist.totsize -= WL_PKT_FILTER_FIXED_LEN;

		memcpy((char *)pkt_filterp, &pkt_filter,
			WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_LIST_FIXED_LEN);
	} else {
		DHD_ERROR(("Invalid filter type %d\n", pkt_filter.type));
		goto fail;
	}

	rc = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);
	rc = rc >= 0 ? 0 : rc;

	if (rc)
		DHD_TRACE(("%s: failed to add pktfilter %s, retcode = %d\n",
		           __FUNCTION__, arg, rc));
	else
		DHD_TRACE(("%s: successfully added pktfilter %s\n",
		           __FUNCTION__, arg));

fail:
	if (arg_org)
		MFREE(dhd->osh, arg_org, strlen(arg) + 1);

	if (buf)
		MFREE(dhd->osh, buf, MAX_PKTFLT_BUF_SIZE);
}

void
dhd_pktfilter_offload_delete(dhd_pub_t *dhd, int id)
{
	int ret;

	ret = dhd_wl_ioctl_set_intiovar(dhd, "pkt_filter_delete",
		id, WLC_SET_VAR, TRUE, 0);
	if (ret < 0) {
		DHD_ERROR(("%s: Failed to delete filter ID:%d, ret=%d\n",
		           __FUNCTION__, id, ret));
	}
}
#endif /* PKT_FILTER_SUPPORT */

/* ========================== */
/* ==== ARP OFFLOAD SUPPORT = */
/* ========================== */
#ifdef ARP_OFFLOAD_SUPPORT
void
dhd_arp_offload_set(dhd_pub_t * dhd, int arp_mode)
{
	int retcode;

	retcode = dhd_wl_ioctl_set_intiovar(dhd, "arp_ol",
		arp_mode, WLC_SET_VAR, TRUE, 0);

	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode) {
		DHD_ERROR(("%s: failed to set ARP offload mode to 0x%x, retcode = %d\n",
			__FUNCTION__, arp_mode, retcode));
	} else {
		DHD_TRACE(("%s: successfully set ARP offload mode to 0x%x\n",
			__FUNCTION__, arp_mode));
		dhd->arpol_configured = TRUE;
	}
}

void
dhd_arp_offload_enable(dhd_pub_t * dhd, int arp_enable)
{
	int retcode;

#ifdef DISABLE_ARP_OFFLOAD_ON_SOFTAP
	/* IF SoftAP is enabled, do NOT enable ARP Offload */
	if (dhd->pfaoe_enab && (dhd->op_mode & DHD_FLAG_HOSTAP_MODE) && arp_enable) {
		DHD_ERROR(("%s: Skip enabling ARP offload when SoftAP is enabled\n", __FUNCTION__));
		return;
	}
#endif /* DISABLE_ARP_OFFLOAD_ON_SOFTAP */

	if (!dhd->arpol_configured) {
		/* If arpol is not applied, apply it */
#ifdef DISABLE_UPDATE_ARP_CACHE_IN_SUSPEND
		/* ARP request packet will be forwarded to update host ARP cache even when
		 * ARP offload is enabled, it will have failures on Packet Filter Validation.
		 */
		if (dhd->pfaoe_enab && dhd->early_suspended)
			dhd_arp_offload_set(dhd, dhd_arp_mode & ~ARP_OL_UPDATE_HOST_CACHE);
		else
#endif /* DISABLE_UPDATE_ARP_CACHE_IN_SUSPEND */
			dhd_arp_offload_set(dhd, dhd_arp_mode);
	}

#ifdef DHD_GARP_KA_PERIOD
	if (DHD_GARP_KA_PERIOD) {
		/* Keep engine on */
		arp_enable = 1;
	}
#endif /* DHD_GARP_KA_PERIOD */

	retcode = dhd_wl_ioctl_set_intiovar(dhd, "arpoe",
		arp_enable, WLC_SET_VAR, TRUE, 0);
	retcode = retcode >= 0 ? 0 : retcode;
	if (retcode)
		DHD_ERROR(("%s: failed to enabe ARP offload to %d, retcode = %d\n",
			__FUNCTION__, arp_enable, retcode));
	else
#ifdef DHD_LOG_DUMP
		DHD_LOG_MEM(("%s: successfully enabed ARP offload to %d\n",
			__FUNCTION__, arp_enable));
#else
		DHD_TRACE(("%s: successfully enabed ARP offload to %d\n",
			__FUNCTION__, arp_enable));
#endif /* DHD_LOG_DUMP */
	if (arp_enable) {
		uint32 version;
		retcode = dhd_wl_ioctl_get_intiovar(dhd, "arp_version",
			&version, WLC_GET_VAR, FALSE, 0);
		if (retcode) {
			DHD_INFO(("%s: fail to get version (maybe version 1:retcode = %d\n",
				__FUNCTION__, retcode));
			dhd->arp_version = 1;
		}
		else {
			DHD_INFO(("%s: ARP Version= %x\n", __FUNCTION__, version));
			dhd->arp_version = version;
		}
	}
}

/* XXX ANDREY: clear AOE arp_table  */
void
dhd_aoe_arp_clr(dhd_pub_t *dhd, int idx)
{
	int ret = 0;

	if (dhd == NULL) return;
	if (dhd->arp_version == 1)
		idx = 0;

	ret = dhd_iovar(dhd, idx, "arp_table_clear", NULL, 0, NULL, 0, TRUE);
	if (ret < 0)
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
	else {
#ifdef DHD_LOG_DUMP
		DHD_LOG_MEM(("%s: ARP table clear\n", __FUNCTION__));
#else
		DHD_TRACE(("%s: ARP table clear\n", __FUNCTION__));
#endif /* DHD_LOG_DUMP */
	}
	/* mac address isn't cleared here but it will be cleared after dongle off */
	dhd->hmac_updated = 0;
}

/* XXX ANDREY: clear hostip table  */
void
dhd_aoe_hostip_clr(dhd_pub_t *dhd, int idx)
{
	int ret = 0;

	if (dhd == NULL) return;
	if (dhd->arp_version == 1)
		idx = 0;

	ret = dhd_iovar(dhd, idx, "arp_hostip_clear", NULL, 0, NULL, 0, TRUE);
	if (ret < 0)
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
	else {
#ifdef DHD_LOG_DUMP
		DHD_LOG_MEM(("%s: ARP host ip clear\n", __FUNCTION__));
#else
		DHD_TRACE(("%s: ARP host ip clear\n", __FUNCTION__));
#endif /* DHD_LOG_DUMP */
	}
}

void
dhd_arp_offload_add_ip(dhd_pub_t *dhd, uint32 ipaddr, int idx)
{
	int ret;

	if (dhd == NULL) return;
	if (dhd->arp_version == 1)
		idx = 0;

	ret = dhd_iovar(dhd, idx, "arp_hostip", (char *)&ipaddr, sizeof(ipaddr),
			NULL, 0, TRUE);
	if (ret < 0)
		DHD_ERROR(("%s: ARP ip addr add failed, ret = %d\n", __FUNCTION__, ret));
	else {
		/* mac address is updated in the dongle */
		dhd->hmac_updated = 1;
#ifdef DHD_LOG_DUMP
		DHD_LOG_MEM(("%s: ARP ip addr entry added \n", __FUNCTION__));
#else
		DHD_TRACE(("%s: ARP ip addr entry added \n", __FUNCTION__));
#endif /* DHD_LOG_DUMP */
	}
}

int
dhd_arp_get_arp_hostip_table(dhd_pub_t *dhd, void *buf, int buflen, int idx)
{
	int ret, i;
	uint32 *ptr32 = buf;
	bool clr_bottom = FALSE;

	if (!buf)
		return -1;
	if (dhd == NULL) return -1;
	if (dhd->arp_version == 1)
		idx = 0;

	ret = dhd_iovar(dhd, idx, "arp_hostip", NULL, 0, (char *)buf, buflen,
			FALSE);
	if (ret) {
		DHD_TRACE(("%s: ioctl WLC_GET_VAR error %d\n",
		__FUNCTION__, ret));

		return -1;
	}

	/* clean up the buf, ascii reminder */
	for (i = 0; i < MAX_IPV4_ENTRIES; i++) {
		if (!clr_bottom) {
			if (*ptr32 == 0)
				clr_bottom = TRUE;
		} else {
			*ptr32 = 0;
		}
		ptr32++;
	}

	return 0;
}
#endif /* ARP_OFFLOAD_SUPPORT  */

/*
 * Neighbor Discovery Offload: enable NDO feature
 * Called  by ipv6 event handler when interface comes up/goes down
 */
int
dhd_ndo_enable(dhd_pub_t * dhd, int ndo_enable)
{
	int retcode;

	if (dhd == NULL)
		return -1;

#if defined(WL_CFG80211) && defined(WL_NAN)
	if (wl_cfgnan_is_dp_active(dhd_linux_get_primary_netdev(dhd))) {
		/* If nan dp is active, skip NDO */
		DHD_INFO(("Active NAN DP, skip NDO\n"));
		return 0;
	}
#endif /* WL_CFG80211 && WL_NAN */
#ifdef WL_CFG80211
	if (dhd->op_mode & DHD_FLAG_HOSTAP_MODE) {
		/* NDO disable on STA+SOFTAP mode */
		ndo_enable = FALSE;
	}
#endif /* WL_CFG80211 */
	retcode = dhd_wl_ioctl_set_intiovar(dhd, "ndoe",
		ndo_enable, WLC_SET_VAR, TRUE, 0);
	if (retcode)
		DHD_ERROR(("%s: failed to enabe ndo to %d, retcode = %d\n",
			__FUNCTION__, ndo_enable, retcode));
	else
		DHD_TRACE(("%s: successfully enabed ndo offload to %d\n",
			__FUNCTION__, ndo_enable));

	return retcode;
}

/*
 * Neighbor Discover Offload: enable NDO feature
 * Called  by ipv6 event handler when interface comes up
 */
int
dhd_ndo_add_ip(dhd_pub_t *dhd, char* ipv6addr, int idx)
{
	int iov_len = 0;
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	int retcode;

	if (dhd == NULL)
		return -1;

	iov_len = bcm_mkiovar("nd_hostip", (char *)ipv6addr,
		IPV6_ADDR_LEN, iovbuf, sizeof(iovbuf));
	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return -1;
	}
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx);

	if (0 > retcode)
		DHD_ERROR(("%s: ndo ip addr add failed, retcode = %d\n",
		__FUNCTION__, retcode));
	else
		DHD_TRACE(("%s: ndo ipaddr entry added \n",
		__FUNCTION__));

	return retcode;
}
/*
 * Neighbor Discover Offload: enable NDO feature
 * Called  by ipv6 event handler when interface goes down
 */
int
dhd_ndo_remove_ip(dhd_pub_t *dhd, int idx)
{
	int iov_len = 0;
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	int retcode;

	if (dhd == NULL)
		return -1;

	iov_len = bcm_mkiovar("nd_hostip_clear", NULL,
		0, iovbuf, sizeof(iovbuf));
	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return -1;
	}
	retcode = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx);

	if (0 > retcode)
		DHD_ERROR(("%s: ndo ip addr remove failed, retcode = %d\n",
		__FUNCTION__, retcode));
	else
		DHD_TRACE(("%s: ndo ipaddr entry removed \n",
		__FUNCTION__));

	return retcode;
}
/* Enhanced ND offload */
uint16
dhd_ndo_get_version(dhd_pub_t *dhdp)
{
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	wl_nd_hostip_t ndo_get_ver;
	int iov_len;
	int retcode;
	uint16 ver = 0;

	if (dhdp == NULL) {
		return BCME_ERROR;
	}

	memset(&iovbuf, 0, sizeof(iovbuf));
	ndo_get_ver.version = htod16(WL_ND_HOSTIP_IOV_VER);
	ndo_get_ver.op_type = htod16(WL_ND_HOSTIP_OP_VER);
	ndo_get_ver.length = htod32(WL_ND_HOSTIP_FIXED_LEN + sizeof(uint16));
	ndo_get_ver.u.version = 0;
	iov_len = bcm_mkiovar("nd_hostip", (char *)&ndo_get_ver,
		WL_ND_HOSTIP_FIXED_LEN + sizeof(uint16), iovbuf, sizeof(iovbuf));

	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return BCME_ERROR;
	}

	retcode = dhd_wl_ioctl_cmd(dhdp, WLC_GET_VAR, iovbuf, iov_len, FALSE, 0);

	if (0 > retcode) {
		DHD_ERROR(("%s: failed, retcode = %d\n", __FUNCTION__, retcode));
		/* ver iovar not supported. NDO version is 0 */
		ver = 0;
	} else {
		wl_nd_hostip_t *ndo_ver_ret = (wl_nd_hostip_t *)iovbuf;

		if ((dtoh16(ndo_ver_ret->version) == WL_ND_HOSTIP_IOV_VER) &&
				(dtoh16(ndo_ver_ret->op_type) == WL_ND_HOSTIP_OP_VER) &&
				(dtoh32(ndo_ver_ret->length) == WL_ND_HOSTIP_FIXED_LEN
					+ sizeof(uint16))) {
			/* nd_hostip iovar version */
			ver = dtoh16(ndo_ver_ret->u.version);
		}

		DHD_TRACE(("%s: successfully get version: %d\n", __FUNCTION__, ver));
	}

	return ver;
}

int
dhd_ndo_add_ip_with_type(dhd_pub_t *dhdp, char *ipv6addr, uint8 type, int idx)
{
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	wl_nd_hostip_t ndo_add_addr;
	int iov_len;
	int retcode;

	if (dhdp == NULL || ipv6addr == 0) {
		return BCME_ERROR;
	}

	/* wl_nd_hostip_t fixed param */
	ndo_add_addr.version = htod16(WL_ND_HOSTIP_IOV_VER);
	ndo_add_addr.op_type = htod16(WL_ND_HOSTIP_OP_ADD);
	ndo_add_addr.length = htod32(WL_ND_HOSTIP_WITH_ADDR_LEN);
	/* wl_nd_host_ip_addr_t param for add */
	memcpy(&ndo_add_addr.u.host_ip.ip_addr, ipv6addr, IPV6_ADDR_LEN);
	ndo_add_addr.u.host_ip.type = type;

	iov_len = bcm_mkiovar("nd_hostip", (char *)&ndo_add_addr,
		WL_ND_HOSTIP_WITH_ADDR_LEN, iovbuf, sizeof(iovbuf));
	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return BCME_ERROR;
	}

	retcode = dhd_wl_ioctl_cmd(dhdp, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx);
	if (0 > retcode) {
		DHD_ERROR(("%s: failed, retcode = %d\n", __FUNCTION__, retcode));
#ifdef NDO_CONFIG_SUPPORT
		if (retcode == BCME_NORESOURCE) {
			/* number of host ip addr exceeds FW capacity, Deactivate ND offload */
			DHD_INFO(("%s: Host IP count exceed device capacity,"
				"ND offload deactivated\n", __FUNCTION__));
			dhdp->ndo_host_ip_overflow = TRUE;
			dhd_ndo_enable(dhdp, FALSE);
		}
#endif /* NDO_CONFIG_SUPPORT */
	} else {
		DHD_TRACE(("%s: successfully added: %d\n", __FUNCTION__, retcode));
	}

	return retcode;
}

int
dhd_ndo_remove_ip_by_addr(dhd_pub_t *dhdp, char *ipv6addr, int idx)
{
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	wl_nd_hostip_t ndo_del_addr;
	int iov_len;
	int retcode;

	if (dhdp == NULL || ipv6addr == 0) {
		return BCME_ERROR;
	}

	/* wl_nd_hostip_t fixed param */
	ndo_del_addr.version = htod16(WL_ND_HOSTIP_IOV_VER);
	ndo_del_addr.op_type = htod16(WL_ND_HOSTIP_OP_DEL);
	ndo_del_addr.length = htod32(WL_ND_HOSTIP_WITH_ADDR_LEN);
	/* wl_nd_host_ip_addr_t param for del */
	memcpy(&ndo_del_addr.u.host_ip.ip_addr, ipv6addr, IPV6_ADDR_LEN);
	ndo_del_addr.u.host_ip.type = 0;	/* don't care */

	iov_len = bcm_mkiovar("nd_hostip", (char *)&ndo_del_addr,
		WL_ND_HOSTIP_WITH_ADDR_LEN, iovbuf, sizeof(iovbuf));

	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return BCME_ERROR;
	}

	retcode = dhd_wl_ioctl_cmd(dhdp, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx);
	if (0 > retcode) {
		DHD_ERROR(("%s: failed, retcode = %d\n", __FUNCTION__, retcode));
	} else {
		DHD_TRACE(("%s: successfully removed: %d\n", __FUNCTION__, retcode));
	}

	return retcode;
}

int
dhd_ndo_remove_ip_by_type(dhd_pub_t *dhdp, uint8 type, int idx)
{
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	wl_nd_hostip_t ndo_del_addr;
	int iov_len;
	int retcode;

	if (dhdp == NULL) {
		return BCME_ERROR;
	}

	/* wl_nd_hostip_t fixed param */
	ndo_del_addr.version = htod16(WL_ND_HOSTIP_IOV_VER);
	if (type == WL_ND_IPV6_ADDR_TYPE_UNICAST) {
		ndo_del_addr.op_type = htod16(WL_ND_HOSTIP_OP_DEL_UC);
	} else if (type == WL_ND_IPV6_ADDR_TYPE_ANYCAST) {
		ndo_del_addr.op_type = htod16(WL_ND_HOSTIP_OP_DEL_AC);
	} else {
		return BCME_BADARG;
	}
	ndo_del_addr.length = htod32(WL_ND_HOSTIP_FIXED_LEN);

	iov_len = bcm_mkiovar("nd_hostip", (char *)&ndo_del_addr, WL_ND_HOSTIP_FIXED_LEN,
			iovbuf, sizeof(iovbuf));

	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return BCME_ERROR;
	}

	retcode = dhd_wl_ioctl_cmd(dhdp, WLC_SET_VAR, iovbuf, iov_len, TRUE, idx);
	if (0 > retcode) {
		DHD_ERROR(("%s: failed, retcode = %d\n", __FUNCTION__, retcode));
	} else {
		DHD_TRACE(("%s: successfully removed: %d\n", __FUNCTION__, retcode));
	}

	return retcode;
}

int
dhd_ndo_unsolicited_na_filter_enable(dhd_pub_t *dhdp, int enable)
{
	char iovbuf[DHD_IOVAR_BUF_SIZE];
	int iov_len;
	int retcode;

	if (dhdp == NULL) {
		return BCME_ERROR;
	}

	iov_len = bcm_mkiovar("nd_unsolicited_na_filter", (char *)&enable, sizeof(int),
			iovbuf, sizeof(iovbuf));

	if (!iov_len) {
		DHD_ERROR(("%s: Insufficient iovar buffer size %zu \n",
			__FUNCTION__, sizeof(iovbuf)));
		return BCME_ERROR;
	}

	retcode = dhd_wl_ioctl_cmd(dhdp, WLC_SET_VAR, iovbuf, iov_len, TRUE, 0);
	if (0 > retcode)
		DHD_ERROR(("%s: failed to enable Unsolicited NA filter to %d, retcode = %d\n",
			__FUNCTION__, enable, retcode));
	else {
		DHD_TRACE(("%s: successfully enabled Unsolicited NA filter to %d\n",
			__FUNCTION__, enable));
	}

	return retcode;
}
#ifdef SIMPLE_ISCAN

uint iscan_thread_id = 0;
iscan_buf_t * iscan_chain = 0;

iscan_buf_t *
dhd_iscan_allocate_buf(dhd_pub_t *dhd, iscan_buf_t **iscanbuf)
{
	iscan_buf_t *iscanbuf_alloc = 0;
	iscan_buf_t *iscanbuf_head;

	DHD_ISCAN(("%s: Entered\n", __FUNCTION__));
	dhd_iscan_lock();

	iscanbuf_alloc = (iscan_buf_t*)MALLOC(dhd->osh, sizeof(iscan_buf_t));
	if (iscanbuf_alloc == NULL)
		goto fail;

	iscanbuf_alloc->next = NULL;
	iscanbuf_head = *iscanbuf;

	DHD_ISCAN(("%s: addr of allocated node = 0x%X"
		   "addr of iscanbuf_head = 0x%X dhd = 0x%X\n",
		   __FUNCTION__, iscanbuf_alloc, iscanbuf_head, dhd));

	if (iscanbuf_head == NULL) {
		*iscanbuf = iscanbuf_alloc;
		DHD_ISCAN(("%s: Head is allocated\n", __FUNCTION__));
		goto fail;
	}

	while (iscanbuf_head->next)
		iscanbuf_head = iscanbuf_head->next;

	iscanbuf_head->next = iscanbuf_alloc;

fail:
	dhd_iscan_unlock();
	return iscanbuf_alloc;
}

void
dhd_iscan_free_buf(void *dhdp, iscan_buf_t *iscan_delete)
{
	iscan_buf_t *iscanbuf_free = 0;
	iscan_buf_t *iscanbuf_prv = 0;
	iscan_buf_t *iscanbuf_cur;
	dhd_pub_t *dhd = dhd_bus_pub(dhdp);
	DHD_ISCAN(("%s: Entered\n", __FUNCTION__));

	dhd_iscan_lock();

	iscanbuf_cur = iscan_chain;

	/* If iscan_delete is null then delete the entire
	 * chain or else delete specific one provided
	 */
	if (!iscan_delete) {
		while (iscanbuf_cur) {
			iscanbuf_free = iscanbuf_cur;
			iscanbuf_cur = iscanbuf_cur->next;
			iscanbuf_free->next = 0;
			MFREE(dhd->osh, iscanbuf_free, sizeof(iscan_buf_t));
		}
		iscan_chain = 0;
	} else {
		while (iscanbuf_cur) {
			if (iscanbuf_cur == iscan_delete)
				break;
			iscanbuf_prv = iscanbuf_cur;
			iscanbuf_cur = iscanbuf_cur->next;
		}
		if (iscanbuf_prv)
			iscanbuf_prv->next = iscan_delete->next;

		iscan_delete->next = 0;
		MFREE(dhd->osh, iscan_delete, sizeof(iscan_buf_t));

		if (!iscanbuf_prv)
			iscan_chain = 0;
	}
	dhd_iscan_unlock();
}

iscan_buf_t *
dhd_iscan_result_buf(void)
{
	return iscan_chain;
}

int
dhd_iscan_issue_request(void * dhdp, wl_iscan_params_t *pParams, uint32 size)
{
	int rc = -1;
	dhd_pub_t *dhd = dhd_bus_pub(dhdp);
	char *buf;
	char iovar[] = "iscan";
	uint32 allocSize = 0;
	wl_ioctl_t ioctl;
	int len;

	if (pParams) {
		allocSize = (size + strlen(iovar) + 1);
		if ((allocSize < size) || (allocSize < strlen(iovar)))
		{
			DHD_ERROR(("%s: overflow - allocation size too large %d < %d + %d!\n",
				__FUNCTION__, allocSize, size, strlen(iovar)));
			goto cleanUp;
		}
		buf = MALLOC(dhd->osh, allocSize);

		if (buf == NULL)
			{
			DHD_ERROR(("%s: malloc of size %d failed!\n", __FUNCTION__, allocSize));
			goto cleanUp;
			}
		ioctl.cmd = WLC_SET_VAR;
		len = bcm_mkiovar(iovar, (char *)pParams, size, buf, allocSize);
		if (len == 0) {
			rc = BCME_BUFTOOSHORT;
			goto cleanUp;
		}
		rc = dhd_wl_ioctl(dhd, 0, &ioctl, buf, len);
	}

cleanUp:
	if (buf) {
		MFREE(dhd->osh, buf, allocSize);
	}

	return rc;
}

static int
dhd_iscan_get_partial_result(void *dhdp, uint *scan_count)
{
	wl_iscan_results_t *list_buf;
	wl_iscan_results_t list;
	wl_scan_results_t *results;
	iscan_buf_t *iscan_cur;
	int status = -1;
	dhd_pub_t *dhd = dhd_bus_pub(dhdp);
	int rc;
	wl_ioctl_t ioctl;
	int len;

	DHD_ISCAN(("%s: Enter\n", __FUNCTION__));

	iscan_cur = dhd_iscan_allocate_buf(dhd, &iscan_chain);
	if (!iscan_cur) {
		DHD_ERROR(("%s: Failed to allocate node\n", __FUNCTION__));
		dhd_iscan_free_buf(dhdp, 0);
		dhd_iscan_request(dhdp, WL_SCAN_ACTION_ABORT);
		dhd_ind_scan_confirm(dhdp, FALSE);
		goto fail;
	}

	dhd_iscan_lock();

	memset(iscan_cur->iscan_buf, 0, WLC_IW_ISCAN_MAXLEN);
	list_buf = (wl_iscan_results_t*)iscan_cur->iscan_buf;
	results = &list_buf->results;
	results->buflen = WL_ISCAN_RESULTS_FIXED_SIZE;
	results->version = 0;
	results->count = 0;

	memset(&list, 0, sizeof(list));
	list.results.buflen = htod32(WLC_IW_ISCAN_MAXLEN);
	len = bcm_mkiovar("iscanresults", (char *)&list, WL_ISCAN_RESULTS_FIXED_SIZE,
		iscan_cur->iscan_buf, WLC_IW_ISCAN_MAXLEN);
	if (len == 0) {
		dhd_iscan_free_buf(dhdp, 0);
		dhd_iscan_request(dhdp, WL_SCAN_ACTION_ABORT);
		dhd_ind_scan_confirm(dhdp, FALSE);
		status = BCME_BUFTOOSHORT;
		goto fail;
	}
	ioctl.cmd = WLC_GET_VAR;
	ioctl.set = FALSE;
	rc = dhd_wl_ioctl(dhd, 0, &ioctl, iscan_cur->iscan_buf, WLC_IW_ISCAN_MAXLEN);

	results->buflen = dtoh32(results->buflen);
	results->version = dtoh32(results->version);
	*scan_count = results->count = dtoh32(results->count);
	status = dtoh32(list_buf->status);
	DHD_ISCAN(("%s: Got %d resuls status = (%x)\n", __FUNCTION__, results->count, status));

	dhd_iscan_unlock();

	if (!(*scan_count)) {
		 /* TODO: race condition when FLUSH already called */
		dhd_iscan_free_buf(dhdp, 0);
	}
fail:
	return status;
}

#endif /* SIMPLE_ISCAN */

/*
 * returns = TRUE if associated, FALSE if not associated
 */
bool dhd_is_associated(dhd_pub_t *dhd, uint8 ifidx, int *retval)
{
	char bssid[6], zbuf[6];
	int ret = -1;

	bzero(bssid, 6);
	bzero(zbuf, 6);

	ret  = dhd_wl_ioctl_cmd(dhd, WLC_GET_BSSID, (char *)&bssid,
		ETHER_ADDR_LEN, FALSE, ifidx);
	/* XXX:AS!!! res can be: -17(BCME_NOTASSOCIATED),-22(BCME_NORESOURCE), and 0(OK)
	  OK - doesn't mean associated yet, the returned bssid
	  still needs to be checked for non zero array
	*/
	DHD_TRACE((" %s WLC_GET_BSSID ioctl res = %d\n", __FUNCTION__, ret));

	if (ret == BCME_NOTASSOCIATED) {
		DHD_ERROR(("%s: WLC_GET_BSSID, NOT ASSOCIATED\n", __FUNCTION__));
	}

	if (retval)
		*retval = ret;

	if (ret < 0)
		return FALSE;

	if ((memcmp(bssid, zbuf, ETHER_ADDR_LEN) == 0)) {
		DHD_TRACE(("%s: WLC_GET_BSSID ioctl returned zero bssid\n", __FUNCTION__));
		return FALSE;
	}
	return TRUE;
}

/* Function to estimate possible DTIM_SKIP value */
#if defined(OEM_ANDROID) && defined(BCMPCIE)
int
dhd_get_suspend_bcn_li_dtim(dhd_pub_t *dhd, int *dtim_period, int *bcn_interval)
{
	int bcn_li_dtim = 1; /* deafult no dtim skip setting */
	int ret = -1;
	int allowed_skip_dtim_cnt = 0;

	if (dhd->disable_dtim_in_suspend) {
		DHD_ERROR(("%s Disable bcn_li_dtim in suspend\n", __FUNCTION__));
		bcn_li_dtim = 0;
		return bcn_li_dtim;
	}

	/* Check if associated */
	if (dhd_is_associated(dhd, 0, NULL) == FALSE) {
		DHD_TRACE(("%s NOT assoc ret %d\n", __FUNCTION__, ret));
		return bcn_li_dtim;
	}

	if (dtim_period == NULL || bcn_interval == NULL)
		return bcn_li_dtim;

	/* read associated AP beacon interval */
	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_BCNPRD,
		bcn_interval, sizeof(*bcn_interval), FALSE, 0)) < 0) {
		DHD_ERROR(("%s get beacon failed code %d\n", __FUNCTION__, ret));
		return bcn_li_dtim;
	}

	/* read associated AP dtim setup */
	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_DTIMPRD,
		dtim_period, sizeof(*dtim_period), FALSE, 0)) < 0) {
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
		return bcn_li_dtim;
	}

	/* if not assocated just return */
	if (*dtim_period == 0) {
		return bcn_li_dtim;
	}

	if (dhd->max_dtim_enable) {
		bcn_li_dtim =
			(int) (MAX_DTIM_ALLOWED_INTERVAL / ((*dtim_period) * (*bcn_interval)));
		if (bcn_li_dtim == 0) {
			bcn_li_dtim = 1;
		}
	} else {
		/* attemp to use platform defined dtim skip interval */
		bcn_li_dtim = dhd->suspend_bcn_li_dtim;

		/* check if sta listen interval fits into AP dtim */
		if (*dtim_period > CUSTOM_LISTEN_INTERVAL) {
			/* AP DTIM to big for our Listen Interval : no dtim skiping */
			bcn_li_dtim = NO_DTIM_SKIP;
			DHD_ERROR(("%s DTIM=%d > Listen=%d : too big ...\n",
				__FUNCTION__, *dtim_period, CUSTOM_LISTEN_INTERVAL));
			return bcn_li_dtim;
		}

		if (((*dtim_period) * (*bcn_interval) * bcn_li_dtim) > MAX_DTIM_ALLOWED_INTERVAL) {
			allowed_skip_dtim_cnt =
				MAX_DTIM_ALLOWED_INTERVAL / ((*dtim_period) * (*bcn_interval));
			bcn_li_dtim =
				(allowed_skip_dtim_cnt != 0) ? allowed_skip_dtim_cnt : NO_DTIM_SKIP;
		}

		if ((bcn_li_dtim * (*dtim_period)) > CUSTOM_LISTEN_INTERVAL) {
			/* Round up dtim_skip to fit into STAs Listen Interval */
			bcn_li_dtim = (int)(CUSTOM_LISTEN_INTERVAL / *dtim_period);
			DHD_TRACE(("%s agjust dtim_skip as %d\n", __FUNCTION__, bcn_li_dtim));
		}
	}

	DHD_ERROR(("%s beacon=%d bcn_li_dtim=%d DTIM=%d Listen=%d\n",
		__FUNCTION__, *bcn_interval, bcn_li_dtim, *dtim_period, CUSTOM_LISTEN_INTERVAL));

	return bcn_li_dtim;
}
#else /* OEM_ANDROID && BCMPCIE */
int
dhd_get_suspend_bcn_li_dtim(dhd_pub_t *dhd)
{
	int bcn_li_dtim = 1; /* deafult no dtim skip setting */
	int ret = -1;
	int dtim_period = 0;
	int ap_beacon = 0;
	int allowed_skip_dtim_cnt = 0;

	if (dhd->disable_dtim_in_suspend) {
		DHD_ERROR(("%s Disable bcn_li_dtim in suspend\n", __FUNCTION__));
		bcn_li_dtim = 0;
		goto exit;
	}

	/* Check if associated */
	if (dhd_is_associated(dhd, 0, NULL) == FALSE) {
		DHD_TRACE(("%s NOT assoc ret %d\n", __FUNCTION__, ret));
		goto exit;
	}

	/* read associated AP beacon interval */
	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_BCNPRD,
		&ap_beacon, sizeof(ap_beacon), FALSE, 0)) < 0) {
		DHD_ERROR(("%s get beacon failed code %d\n", __FUNCTION__, ret));
		goto exit;
	}

	/* read associated ap's dtim setup */
	if ((ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_DTIMPRD,
		&dtim_period, sizeof(dtim_period), FALSE, 0)) < 0) {
		DHD_ERROR(("%s failed code %d\n", __FUNCTION__, ret));
		goto exit;
	}

	/* if not assocated just exit */
	if (dtim_period == 0) {
		goto exit;
	}

	if (dhd->max_dtim_enable) {
		bcn_li_dtim =
			(int) (MAX_DTIM_ALLOWED_INTERVAL / (ap_beacon * dtim_period));
		if (bcn_li_dtim == 0) {
			bcn_li_dtim = 1;
		}
	} else {
		/* attemp to use platform defined dtim skip interval */
		bcn_li_dtim = dhd->suspend_bcn_li_dtim;

		/* check if sta listen interval fits into AP dtim */
		if (dtim_period > CUSTOM_LISTEN_INTERVAL) {
			/* AP DTIM to big for our Listen Interval : no dtim skiping */
			bcn_li_dtim = NO_DTIM_SKIP;
			DHD_ERROR(("%s DTIM=%d > Listen=%d : too big ...\n",
				__FUNCTION__, dtim_period, CUSTOM_LISTEN_INTERVAL));
			goto exit;
		}

		if ((dtim_period * ap_beacon * bcn_li_dtim) > MAX_DTIM_ALLOWED_INTERVAL) {
			allowed_skip_dtim_cnt =
				MAX_DTIM_ALLOWED_INTERVAL / (dtim_period * ap_beacon);
			bcn_li_dtim =
				(allowed_skip_dtim_cnt != 0) ? allowed_skip_dtim_cnt : NO_DTIM_SKIP;
		}

		if ((bcn_li_dtim * dtim_period) > CUSTOM_LISTEN_INTERVAL) {
			/* Round up dtim_skip to fit into STAs Listen Interval */
			bcn_li_dtim = (int)(CUSTOM_LISTEN_INTERVAL / dtim_period);
			DHD_TRACE(("%s agjust dtim_skip as %d\n", __FUNCTION__, bcn_li_dtim));
		}
	}

	DHD_ERROR(("%s beacon=%d bcn_li_dtim=%d DTIM=%d Listen=%d\n",
		__FUNCTION__, ap_beacon, bcn_li_dtim, dtim_period, CUSTOM_LISTEN_INTERVAL));

exit:
	return bcn_li_dtim;
}
#endif /* OEM_ANDROID && BCMPCIE */

#ifdef CONFIG_SILENT_ROAM
#ifdef HIGHBAND_ROAM
int
dhd_highband_roam_init(dhd_pub_t *dhd)
{
	int ret = BCME_OK;
	wlc_sroam_t *psroam;
	wlc_sroam_info_t *sroam;
	uint sroamlen = sizeof(*sroam) + SROAM_HDRLEN;

	if (dhd->op_mode &
		(DHD_FLAG_HOSTAP_MODE | DHD_FLAG_P2P_GC_MODE | DHD_FLAG_P2P_GO_MODE)) {
		DHD_INFO((" Failed to init sroam, op_mode 0x%04x\n", dhd->op_mode));
		return ret;
	}

	psroam = (wlc_sroam_t *)MALLOCZ(dhd->osh, sroamlen);
	if (!psroam) {
		DHD_ERROR(("%s Fail to malloc buffer\n", __FUNCTION__));
		return BCME_NOMEM;
	}

	ret = dhd_iovar(dhd, 0, "sroam", NULL, 0, (char *)psroam, sroamlen, FALSE);
	if (ret < 0) {
		DHD_ERROR(("%s Failed to Get sroam %d\n", __FUNCTION__, ret));
		goto done;
	}

	if (psroam->ver != WLC_SILENT_ROAM_CUR_VER) {
		ret = BCME_VERSION;
		goto done;
	}

	sroam = (wlc_sroam_info_t *)psroam->data;
	sroam->sroam_on = TRUE;
	sroam->sroam_period_time = HIGHBAND_PERIOD_TIME;
	sroam->sroam_inact_cnt = HIGHBAND_INACT_CNT;
	sroam->sroam_inact_time = HIGHBAND_INACT_TIME;
	sroam->sroam_min_rssi = HIGHBAND_RSSI;

	DHD_ERROR(("Silent roam monitor init, sroam_on = %d "
		"sroam_period_time = %d, sroam_inact_cnt = %d, sroam_inact_time = %d "
		"sroam_min_rssi = %d\n"
		,sroam->sroam_on, sroam->sroam_period_time,
		sroam->sroam_inact_cnt, sroam->sroam_inact_time, sroam->sroam_min_rssi));

	ret = dhd_iovar(dhd, 0, "sroam", (char *)psroam, sroamlen, NULL, 0, TRUE);
	if (ret < 0) {
		DHD_ERROR(("%s Failed to Set sroam %d\n", __FUNCTION__, ret));
	}

done:
	if (psroam) {
		MFREE(dhd->osh, psroam, sroamlen);
	}

	return ret;
}
#endif /* HIGHBAND_ROAM */

int
dhd_sroam_set_mon(dhd_pub_t *dhd, bool set)
{
	int ret = BCME_OK;
	wlc_sroam_t *psroam;
	wlc_sroam_info_t *sroam;
	uint sroamlen = sizeof(*sroam) + SROAM_HDRLEN;

	/* Check if associated */
	if (dhd_is_associated(dhd, 0, NULL) == FALSE) {
		DHD_TRACE(("%s NOT assoc\n", __FUNCTION__));
		return ret;
	}

	if (set && (dhd->op_mode &
		(DHD_FLAG_HOSTAP_MODE | DHD_FLAG_P2P_GC_MODE | DHD_FLAG_P2P_GO_MODE))) {
		DHD_INFO((" Failed to set sroam %d, op_mode 0x%04x\n", set, dhd->op_mode));
		return ret;
	}

	if (!dhd->sroam_turn_on) {
		DHD_INFO((" Failed to set sroam %d, sroam turn %d\n", set, dhd->sroam_turn_on));
		return ret;
	}
	psroam = (wlc_sroam_t *)MALLOCZ(dhd->osh, sroamlen);
	if (!psroam) {
		DHD_ERROR(("%s Fail to malloc buffer\n", __FUNCTION__));
		return BCME_NOMEM;
	}

	ret = dhd_iovar(dhd, 0, "sroam", NULL, 0, (char *)psroam, sroamlen, FALSE);
	if (ret < 0) {
		DHD_ERROR(("%s Failed to Get sroam %d\n", __FUNCTION__, ret));
		goto done;
	}

	if (psroam->ver != WLC_SILENT_ROAM_CUR_VER) {
		ret = BCME_VERSION;
		goto done;
	}

	sroam = (wlc_sroam_info_t *)psroam->data;
	sroam->sroam_on = set;
	DHD_INFO((" Silent roam monitor mode %s\n", set ? "On" : "Off"));

	ret = dhd_iovar(dhd, 0, "sroam", (char *)psroam, sroamlen, NULL, 0, TRUE);
	if (ret < 0) {
		DHD_ERROR(("%s Failed to Set sroam %d\n", __FUNCTION__, ret));
	}

done:
	if (psroam) {
		MFREE(dhd->osh, psroam, sroamlen);
	}

	return ret;
}
#endif /* CONFIG_SILENT_ROAM */

/* Check if the mode supports STA MODE */
bool dhd_support_sta_mode(dhd_pub_t *dhd)
{

#ifdef  WL_CFG80211
	if (!(dhd->op_mode & DHD_FLAG_STA_MODE))
		return FALSE;
	else
#endif /* WL_CFG80211 */
		return TRUE;
}

#if defined(KEEP_ALIVE)
int dhd_keep_alive_onoff(dhd_pub_t *dhd)
{
	char				buf[32] = {0};
	const char			*str;
	wl_mkeep_alive_pkt_t	mkeep_alive_pkt = {0, 0, 0, 0, 0, {0}};
	wl_mkeep_alive_pkt_t	*mkeep_alive_pktp;
	int					buf_len;
	int					str_len;
	int res					= -1;

	if (!dhd_support_sta_mode(dhd))
		return res;

	DHD_TRACE(("%s execution\n", __FUNCTION__));

	str = "mkeep_alive";
	str_len = strlen(str);
	strlcpy(buf, str, sizeof(buf));
	mkeep_alive_pktp = (wl_mkeep_alive_pkt_t *) (buf + str_len + 1);
	mkeep_alive_pkt.period_msec = CUSTOM_KEEP_ALIVE_SETTING;
	buf_len = str_len + 1;
	mkeep_alive_pkt.version = htod16(WL_MKEEP_ALIVE_VERSION);
	mkeep_alive_pkt.length = htod16(WL_MKEEP_ALIVE_FIXED_LEN);
	/* Setup keep alive zero for null packet generation */
	mkeep_alive_pkt.keep_alive_id = 0;
	mkeep_alive_pkt.len_bytes = 0;
	buf_len += WL_MKEEP_ALIVE_FIXED_LEN;
	bzero(mkeep_alive_pkt.data, sizeof(mkeep_alive_pkt.data));
	/* Keep-alive attributes are set in local	variable (mkeep_alive_pkt), and
	 * then memcpy'ed into buffer (mkeep_alive_pktp) since there is no
	 * guarantee that the buffer is properly aligned.
	 */
	memcpy((char *)mkeep_alive_pktp, &mkeep_alive_pkt, WL_MKEEP_ALIVE_FIXED_LEN);

	res = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, buf, buf_len, TRUE, 0);

	return res;
}
#endif /* defined(KEEP_ALIVE) */

/* modify for compatibility */
#if defined(OEM_ANDROID) && defined(PNO_SUPPORT)
#define CSCAN_TLV_TYPE_SSID_IE	'S'
/*
 *  SSIDs list parsing from cscan tlv list
 */
int
wl_parse_ssid_list_tlv(char** list_str, wlc_ssid_ext_t* ssid, int max, int *bytes_left)
{
	char* str;
	int idx = 0;
	uint8 len;

	if ((list_str == NULL) || (*list_str == NULL) || (*bytes_left < 0)) {
		DHD_ERROR(("%s error paramters\n", __FUNCTION__));
		return BCME_BADARG;
	}
	str = *list_str;
	while (*bytes_left > 0) {
		if (str[0] != CSCAN_TLV_TYPE_SSID_IE) {
			*list_str = str;
			DHD_TRACE(("nssid=%d left_parse=%d %d\n", idx, *bytes_left, str[0]));
			return idx;
		}

		if (idx >= max) {
			DHD_ERROR(("%s number of SSIDs more than %d\n", __FUNCTION__, idx));
			return BCME_BADARG;
		}

		/* Get proper CSCAN_TLV_TYPE_SSID_IE */
		*bytes_left -= 1;
		if (*bytes_left == 0) {
			DHD_ERROR(("%s no length field.\n", __FUNCTION__));
			return BCME_BADARG;
		}
		str += 1;
		ssid[idx].rssi_thresh = 0;
		ssid[idx].flags = 0;
		len = str[0];
		if (len == 0) {
			/* Broadcast SSID */
			ssid[idx].SSID_len = 0;
			memset((char*)ssid[idx].SSID, 0x0, DOT11_MAX_SSID_LEN);
			*bytes_left -= 1;
			str += 1;

			DHD_TRACE(("BROADCAST SCAN  left=%d\n", *bytes_left));
		} else if (len <= DOT11_MAX_SSID_LEN) {
			/* Get proper SSID size */
			ssid[idx].SSID_len = len;
			*bytes_left -= 1;
			/* Get SSID */
			if (ssid[idx].SSID_len > *bytes_left) {
				DHD_ERROR(("%s out of memory range len=%d but left=%d\n",
				__FUNCTION__, ssid[idx].SSID_len, *bytes_left));
				return BCME_BADARG;
			}
			str += 1;
			memcpy((char*)ssid[idx].SSID, str, ssid[idx].SSID_len);

			*bytes_left -= ssid[idx].SSID_len;
			str += ssid[idx].SSID_len;
			ssid[idx].hidden = TRUE;

			DHD_TRACE(("%s :size=%d left=%d\n",
				(char*)ssid[idx].SSID, ssid[idx].SSID_len, *bytes_left));
		} else {
			DHD_ERROR(("### SSID size more than %d\n", str[0]));
			return BCME_BADARG;
		}
		idx++;
	}

	*list_str = str;
	return idx;
}
/* Android ComboSCAN support */

/*
 *  data parsing from ComboScan tlv list
*/
int
wl_iw_parse_data_tlv(char** list_str, void *dst, int dst_size, const char token,
                     int input_size, int *bytes_left)
{
	char* str;
	uint16 short_temp;
	uint32 int_temp;

	if ((list_str == NULL) || (*list_str == NULL) ||(bytes_left == NULL) || (*bytes_left < 0)) {
		DHD_ERROR(("%s error paramters\n", __FUNCTION__));
		return -1;
	}
	str = *list_str;

	/* Clean all dest bytes */
	memset(dst, 0, dst_size);
	if (*bytes_left > 0) {

		if (str[0] != token) {
			DHD_TRACE(("%s NOT Type=%d get=%d left_parse=%d \n",
				__FUNCTION__, token, str[0], *bytes_left));
			return -1;
		}

		*bytes_left -= 1;
		str += 1;

		if (input_size == 1) {
			memcpy(dst, str, input_size);
		}
		else if (input_size == 2) {
			memcpy(dst, (char *)htod16(memcpy(&short_temp, str, input_size)),
				input_size);
		}
		else if (input_size == 4) {
			memcpy(dst, (char *)htod32(memcpy(&int_temp, str, input_size)),
				input_size);
		}

		*bytes_left -= input_size;
		str += input_size;
		*list_str = str;
		return 1;
	}
	return 1;
}

/*
 *  channel list parsing from cscan tlv list
*/
int
wl_iw_parse_channel_list_tlv(char** list_str, uint16* channel_list,
                             int channel_num, int *bytes_left)
{
	char* str;
	int idx = 0;

	if ((list_str == NULL) || (*list_str == NULL) ||(bytes_left == NULL) || (*bytes_left < 0)) {
		DHD_ERROR(("%s error paramters\n", __FUNCTION__));
		return -1;
	}
	str = *list_str;

	while (*bytes_left > 0) {

		if (str[0] != CSCAN_TLV_TYPE_CHANNEL_IE) {
			*list_str = str;
			DHD_TRACE(("End channel=%d left_parse=%d %d\n", idx, *bytes_left, str[0]));
			return idx;
		}
		/* Get proper CSCAN_TLV_TYPE_CHANNEL_IE */
		*bytes_left -= 1;
		str += 1;

		if (str[0] == 0) {
			/* All channels */
			channel_list[idx] = 0x0;
		}
		else {
			channel_list[idx] = (uint16)str[0];
			DHD_TRACE(("%s channel=%d \n", __FUNCTION__,  channel_list[idx]));
		}
		*bytes_left -= 1;
		str += 1;

		if (idx++ > 255) {
			DHD_ERROR(("%s Too many channels \n", __FUNCTION__));
			return -1;
		}
	}

	*list_str = str;
	return idx;
}

/* Parse a comma-separated list from list_str into ssid array, starting
 * at index idx.  Max specifies size of the ssid array.  Parses ssids
 * and returns updated idx; if idx >= max not all fit, the excess have
 * not been copied.  Returns -1 on empty string, or on ssid too long.
 */
int
wl_iw_parse_ssid_list(char** list_str, wlc_ssid_t* ssid, int idx, int max)
{
	char* str, *ptr;

	if ((list_str == NULL) || (*list_str == NULL))
		return -1;

	for (str = *list_str; str != NULL; str = ptr) {

		/* check for next TAG */
		if (!strncmp(str, GET_CHANNEL, strlen(GET_CHANNEL))) {
			*list_str	 = str + strlen(GET_CHANNEL);
			return idx;
		}

		if ((ptr = strchr(str, ',')) != NULL) {
			*ptr++ = '\0';
		}

		if (strlen(str) > DOT11_MAX_SSID_LEN) {
			DHD_ERROR(("ssid <%s> exceeds %d\n", str, DOT11_MAX_SSID_LEN));
			return -1;
		}

		if (strlen(str) == 0)
			ssid[idx].SSID_len = 0;

		if (idx < max) {
			bzero(ssid[idx].SSID, sizeof(ssid[idx].SSID));
			strlcpy((char*)ssid[idx].SSID, str, sizeof(ssid[idx].SSID));
			ssid[idx].SSID_len = sizeof(ssid[idx].SSID);
		}
		idx++;
	}
	return idx;
}

/*
 * Parse channel list from iwpriv CSCAN
 */
int
wl_iw_parse_channel_list(char** list_str, uint16* channel_list, int channel_num)
{
	int num;
	int val;
	char* str;
	char* endptr = NULL;

	if ((list_str == NULL)||(*list_str == NULL))
		return -1;

	str = *list_str;
	num = 0;
	while (strncmp(str, GET_NPROBE, strlen(GET_NPROBE))) {
		val = (int)strtoul(str, &endptr, 0);
		if (endptr == str) {
			printf("could not parse channel number starting at"
				" substring \"%s\" in list:\n%s\n",
				str, *list_str);
			return -1;
		}
		str = endptr + strspn(endptr, " ,");

		if (num == channel_num) {
			DHD_ERROR(("too many channels (more than %d) in channel list:\n%s\n",
				channel_num, *list_str));
			return -1;
		}

		channel_list[num++] = (uint16)val;
	}
	*list_str = str;
	return num;
}
#endif /* defined(OEM_ANDROID) && defined(PNO_SUPPORT) */

#ifdef DHD_LINUX_STD_FW_API
int dhd_get_download_buffer(dhd_pub_t	*dhd, char *file_path, download_type_t component,
	char ** buffer, int *length)
{
	int ret = BCME_ERROR;
	const struct firmware *fw = NULL;
#ifdef SUPPORT_OTA_UPDATE
	uint8 *buf = NULL;
	int len = 0;
	ota_update_info_t *ota_info = &dhd->ota_update_info;

	if (component == CLM_BLOB) {
		if (ota_info->clm_len) {
			DHD_ERROR(("Using OTA CLM_BLOB\n"));
			buf = ota_info->clm_buf;
			len = ota_info->clm_len;
		}
	}
	else if (component == NVRAM) {
		if (ota_info->nvram_len) {
			DHD_ERROR(("Using OTA NVRAM.\n"));
			buf = ota_info->nvram_buf;
			len = ota_info->nvram_len;
		}
	}

	if (len) {
		*buffer = (char *)buf;
		*length = len;
	}
	else
#endif /* SUPPORT_OTA_UPDATE */
	{
		if (file_path) {
			ret = dhd_os_get_img_fwreq(&fw, file_path);
			if (ret < 0) {
				DHD_ERROR(("dhd_os_get_img(Request Firmware API) error : %d\n",
					ret));
				goto err;
			} else {
				DHD_ERROR(("dhd_os_get_img(Request Firmware API) success\n"));
				if ((fw->size <= 0 || fw->size > *length)) {
					*length = fw->size;
					goto err;
				}
				*buffer = VMALLOCZ(dhd->osh, fw->size);
				if (*buffer == NULL) {
					DHD_ERROR(("%s: Failed to allocate memory %d bytes\n",
						__FUNCTION__, (int)fw->size));
					ret = BCME_NOMEM;
					goto err;
				}
				*length = fw->size;
				ret = memcpy_s(*buffer, fw->size, fw->data, fw->size);
				if (ret != BCME_OK) {
					DHD_ERROR(("%s: memcpy_s failed, err : %d\n",
							__FUNCTION__, ret));
					goto err;
				}
				ret = BCME_OK;
			}
		}
	}
err:
	if (fw) {
		dhd_os_close_img_fwreq(fw);
	}
	return ret;
}

#else

/* Given filename and download type,  returns a buffer pointer and length
* for download to f/w. Type can be FW or NVRAM.
*
*/
int dhd_get_download_buffer(dhd_pub_t	*dhd, char *file_path, download_type_t component,
	char ** buffer, int *length)
{
	int ret = BCME_ERROR;
	int len = 0;
	int file_len;
	void *image = NULL;
	uint8 *buf = NULL;

	/* Point to cache if available. */
	/* No Valid cache found on this call */
	if (!len) {
		file_len = *length;
		*length = 0;

		if (file_path) {
			image = dhd_os_open_image1(dhd, file_path);
			if (image == NULL) {
				goto err;
			}
		}

		buf = MALLOCZ(dhd->osh, file_len);
		if (buf == NULL) {
			DHD_ERROR(("%s: Failed to allocate memory %d bytes\n",
				__FUNCTION__, file_len));
			ret = BCME_NOMEM;
			goto err;
		}

		/* Download image */
		len = dhd_os_get_image_block((char *)buf, file_len, image);
		if ((len <= 0 || len > file_len)) {
			MFREE(dhd->osh, buf, file_len);
			goto err;
		}
	}

	ret = BCME_OK;
	*length = len;
	*buffer = (char *)buf;

	/* Cache if first call. */

err:
	if (image)
		dhd_os_close_image1(dhd, image);

	return ret;
}
#endif /* DHD_LINUX_STD_FW_API */

int
dhd_download_2_dongle(dhd_pub_t	*dhd, char *iovar, uint16 flag, uint16 dload_type,
	unsigned char *dload_buf, int len)
{
	struct wl_dload_data *dload_ptr = (struct wl_dload_data *)dload_buf;
	int err = 0;
	int dload_data_offset;
	static char iovar_buf[WLC_IOCTL_MEDLEN];
	int iovar_len;

	memset(iovar_buf, 0, sizeof(iovar_buf));

	dload_data_offset = OFFSETOF(wl_dload_data_t, data);
	dload_ptr->flag = (DLOAD_HANDLER_VER << DLOAD_FLAG_VER_SHIFT) | flag;
	dload_ptr->dload_type = dload_type;
	dload_ptr->len = htod32(len - dload_data_offset);
	dload_ptr->crc = 0;
	len = ROUNDUP(len, 8);

	iovar_len = bcm_mkiovar(iovar, (char *)dload_buf,
		(uint)len, iovar_buf, sizeof(iovar_buf));
	if (iovar_len == 0) {
		DHD_ERROR(("%s: insufficient buffer space passed to bcm_mkiovar for '%s' \n",
		           __FUNCTION__, iovar));
		return BCME_BUFTOOSHORT;
	}

	err = dhd_wl_ioctl_cmd(dhd, WLC_SET_VAR, iovar_buf,
			iovar_len, IOV_SET, 0);

	return err;
}

int
dhd_download_blob(dhd_pub_t *dhd, unsigned char *buf,
		uint32 len, char *iovar)

{
	int chunk_len;
#if defined(DHD_LINUX_STD_FW_API)
	int cumulative_len = 0;
#endif /* !LINUX && !linux || DHD_LINUX_STD_FW_API */
	int size2alloc;
	unsigned char *new_buf;
	int err = 0, data_offset;
	uint16 dl_flag = DL_BEGIN;

	data_offset = OFFSETOF(wl_dload_data_t, data);
	size2alloc = data_offset + MAX_CHUNK_LEN;
	size2alloc = ROUNDUP(size2alloc, 8);

	if ((new_buf = (unsigned char *)MALLOCZ(dhd->osh, size2alloc)) != NULL) {
		do {
#if defined(DHD_LINUX_STD_FW_API)
			if (len >= MAX_CHUNK_LEN)
				chunk_len = MAX_CHUNK_LEN;
			else
				chunk_len = len;

			memcpy(new_buf + data_offset, buf + cumulative_len, chunk_len);
			cumulative_len += chunk_len;
#else
			chunk_len = dhd_os_get_image_block((char *)(new_buf + data_offset),
				MAX_CHUNK_LEN, buf);
			if (chunk_len < 0) {
				DHD_ERROR(("%s: dhd_os_get_image_block failed (%d)\n",
					__FUNCTION__, chunk_len));
				err = BCME_ERROR;
				goto exit;
			}
#endif /* !LINUX && !linux || DHD_LINUX_STD_FW_API */
			if (len - chunk_len == 0)
				dl_flag |= DL_END;

			err = dhd_download_2_dongle(dhd, iovar, dl_flag, DL_TYPE_CLM,
				new_buf, data_offset + chunk_len);

			dl_flag &= ~DL_BEGIN;

			len = len - chunk_len;
		} while ((len > 0) && (err == 0));
#if defined(DHD_LINUX_STD_FW_API)
		MFREE(dhd->osh, new_buf, size2alloc);
#endif /* !LINUX && !linux || DHD_LINUX_STD_FW_API */
	} else {
		err = BCME_NOMEM;
	}
#if !defined(DHD_LINUX_STD_FW_API)
exit:
	if (new_buf) {
		MFREE(dhd->osh, new_buf, size2alloc);
	}
#endif /* LINUX || linux && !DHD_LINUX_STD_FW_API */
	return err;
}

int
dhd_apply_default_txcap(dhd_pub_t  *dhd, char *path)
{
	return 0;
}

int
dhd_check_current_clm_data(dhd_pub_t *dhd)
{
	char iovbuf[WLC_IOCTL_SMLEN];
	wl_country_t *cspec;
	int err = BCME_OK;

	memset(iovbuf, 0, sizeof(iovbuf));
	err = bcm_mkiovar("country", NULL, 0, iovbuf, sizeof(iovbuf));
	if (err == 0) {
		err = BCME_BUFTOOSHORT;
		DHD_ERROR(("%s: bcm_mkiovar failed.", __FUNCTION__));
		return err;
	}
	err = dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, iovbuf, sizeof(iovbuf), FALSE, 0);
	if (err) {
		DHD_ERROR(("%s: country code get failed\n", __FUNCTION__));
		return err;
	}
	cspec = (wl_country_t *)iovbuf;
	if ((strncmp(cspec->ccode, WL_CCODE_NULL_COUNTRY, WLC_CNTRY_BUF_SZ)) == 0) {
		DHD_ERROR(("%s: ----- This FW is not included CLM data -----\n",
			__FUNCTION__));
		return FALSE;
	}
	DHD_ERROR(("%s: ----- This FW is included CLM data -----\n",
		__FUNCTION__));
	return TRUE;
}

int
dhd_apply_default_clm(dhd_pub_t *dhd, char *clm_path)
{
	int len = 0, memblock_len = 0;
	char *memblock = NULL;
	int err = BCME_OK;
	char iovbuf[WLC_IOCTL_SMLEN];
	int status = FALSE;
#ifdef SUPPORT_OTA_UPDATE
	ota_update_info_t *ota_info = &dhd->ota_update_info;
#endif /* SUPPORT_OTA_UPDATE */

	/* If CLM blob file is found on the filesystem, download the file.
	 * After CLM file download or If the blob file is not present,
	 * validate the country code before proceeding with the initialization.
	 * If country code is not valid, fail the initialization.
	 */
#if defined(DHD_LINUX_STD_FW_API)
	DHD_ERROR(("Clmblob file : %s\n", clm_path));
	len = MAX_CLM_BUF_SIZE;
	err = dhd_get_download_buffer(dhd, clm_path, CLM_BLOB, &memblock, &len);
#ifdef DHD_LINUX_STD_FW_API
	memblock_len = len;
#else
	memblock_len = MAX_CLM_BUF_SIZE;
#endif /* DHD_LINUX_STD_FW_API */
#else
	memblock = dhd_os_open_image1(dhd, (char *)clm_path);
	len = dhd_os_get_image_size(memblock);
	BCM_REFERENCE(memblock_len);
#endif /* !LINUX && !linux || DHD_LINUX_STD_FW_API */

	if (memblock == NULL) {
#if defined(DHD_BLOB_EXISTENCE_CHECK)
		if (!dhd->is_blob) {
			status = dhd_check_current_clm_data(dhd);
			if (status == TRUE) {
				err = BCME_OK;
			} else {
				err = status;
			}
		}
#endif /* DHD_BLOB_EXISTENCE_CHECK */
		goto exit;
	}

	if ((len > 0) && (len < MAX_CLM_BUF_SIZE) && memblock) {
		status = dhd_check_current_clm_data(dhd);
		if (status == TRUE) {
#if defined(DHD_BLOB_EXISTENCE_CHECK)
			if (dhd->op_mode != DHD_FLAG_MFG_MODE) {
				if (dhd->is_blob) {
					err = BCME_ERROR;
				}
				goto exit;
			}
#else
			DHD_ERROR(("%s: CLM already exist in F/W, "
				"new CLM data will be added to the end of existing CLM data!\n",
				__FUNCTION__));
#endif /* DHD_BLOB_EXISTENCE_CHECK */
		} else if (status != FALSE) {
			err = status;
			goto exit;
		}

		/* Found blob file. Download the file */
		DHD_TRACE(("clm file download from %s \n", clm_path));
		err = dhd_download_blob(dhd, (unsigned char*)memblock, len, "clmload");
		if (err) {
			DHD_ERROR(("%s: CLM download failed err=%d\n", __FUNCTION__, err));
			/* Retrieve clmload_status and print */
			memset(iovbuf, 0, sizeof(iovbuf));
			len = bcm_mkiovar("clmload_status", NULL, 0, iovbuf, sizeof(iovbuf));
			if (len == 0) {
				err = BCME_BUFTOOSHORT;
				goto exit;
			}
			err = dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, iovbuf, sizeof(iovbuf), FALSE, 0);
			if (err) {
				DHD_ERROR(("%s: clmload_status get failed err=%d \n",
					__FUNCTION__, err));
			} else {
				DHD_ERROR(("%s: clmload_status: %d \n",
					__FUNCTION__, *((int *)iovbuf)));
				if (*((int *)iovbuf) == CHIPID_MISMATCH) {
					DHD_ERROR(("Chip ID mismatch error \n"));
				}
			}
			err = BCME_ERROR;
			goto exit;
		} else {
			DHD_INFO(("%s: CLM download succeeded \n", __FUNCTION__));
		}
	} else {
		DHD_INFO(("Skipping the clm download. len:%d memblk:%p \n", len, memblock));
	}

	/* Verify country code */
	status = dhd_check_current_clm_data(dhd);

	if (status != TRUE) {
		/* Country code not initialized or CLM download not proper */
		DHD_ERROR(("country code not initialized\n"));
		err = status;
	}
exit:

	if (memblock) {
#ifdef SUPPORT_OTA_UPDATE
		if (ota_info->clm_len) {
			MFREE(dhd->osh, ota_info->clm_buf, ota_info->clm_len);
			ota_info->clm_len = 0;
		}
		else
#endif /* SUPPORT_OTA_UPDATE */
		{
#if !defined(DHD_LINUX_STD_FW_API)
			dhd_os_close_image1(dhd, memblock);
#else
			dhd_free_download_buffer(dhd, memblock, memblock_len);
#endif /* LINUX || linux */
		}
	}

	return err;
}

void dhd_free_download_buffer(dhd_pub_t	*dhd, void *buffer, int length)
{
#if defined(DHD_LINUX_STD_FW_API)
	VMFREE(dhd->osh, buffer, length);
#else
	MFREE(dhd->osh, buffer, length);
#endif /* DHD_LINUX_STD_FW_API */
}

#ifdef SHOW_LOGTRACE
int
dhd_parse_logstrs_file(osl_t *osh, char *raw_fmts, int logstrs_size,
		dhd_event_log_t *event_log)
{
	uint32 *lognums = NULL;
	char *logstrs = NULL;
	logstr_trailer_t *trailer = NULL;
	int ram_index = 0;
	char **fmts = NULL;
	int num_fmts = 0;
	bool match_fail = TRUE;
	int32 i = 0;
	uint8 *pfw_id = NULL;
	uint32 fwid = 0;
	char fwid_str[FWID_STR_LEN];
	uint32 hdr_logstrs_size = 0;
#ifdef DHD_LINUX_STD_FW_API
	int err = 0;
	const struct firmware *fw = NULL;
#else
	void *file = NULL;
	int file_len = 0;
#endif /* DHD_LINUX_STD_FW_API */

	/* Read last three words in the logstrs.bin file */
	trailer = (logstr_trailer_t *) (raw_fmts + logstrs_size -
		sizeof(logstr_trailer_t));

	if (trailer->log_magic == LOGSTRS_MAGIC) {
		/*
		* logstrs.bin has a header.
		*/
		if (trailer->version == 1) {
			logstr_header_v1_t *hdr_v1 = (logstr_header_v1_t *) (raw_fmts +
					logstrs_size - sizeof(logstr_header_v1_t));
			DHD_INFO(("%s: logstr header version = %u\n",
					__FUNCTION__, hdr_v1->version));
			num_fmts =	hdr_v1->rom_logstrs_offset / sizeof(uint32);
			ram_index = (hdr_v1->ram_lognums_offset -
				hdr_v1->rom_lognums_offset) / sizeof(uint32);
			lognums = (uint32 *) &raw_fmts[hdr_v1->rom_lognums_offset];
			logstrs = (char *)	 &raw_fmts[hdr_v1->rom_logstrs_offset];
			hdr_logstrs_size = hdr_v1->logstrs_size;
		} else if (trailer->version == 2) {
			logstr_header_t *hdr = (logstr_header_t *) (raw_fmts + logstrs_size -
					sizeof(logstr_header_t));
			DHD_INFO(("%s: logstr header version = %u; flags = %x\n",
					__FUNCTION__, hdr->version, hdr->flags));

			/* For ver. 2 of the header, need to match fwid of
			 *  both logstrs.bin and fw bin
			 */
#ifdef DHD_LINUX_STD_FW_API
			err = dhd_os_get_img_fwreq(&fw, st_str_file_path);
			if (err < 0) {
				DHD_ERROR(("%s: (Request Firmware API) '%s' error=%d\n",
				           __FUNCTION__, st_str_file_path, err));
				goto error;
			}
			memset(fwid_str, 0, sizeof(fwid_str));
			err = memcpy_s(fwid_str, (sizeof(fwid_str) - 1),
				&(fw->data[fw->size - (sizeof(fwid_str) - 1)]),
				(sizeof(fwid_str) - 1));
			if (err) {
				DHD_ERROR(("%s: failed to copy raw_fmts, err=%d\n",
					__FUNCTION__, err));
				goto error;
			}
#else
			/* read the FWID from fw bin */
			file = dhd_os_open_image1(NULL, st_str_file_path);
			if (!file) {
				DHD_ERROR(("%s: cannot open fw file '%s'!\n",
				           __FUNCTION__, st_str_file_path));
				goto error;
			}
			file_len = dhd_os_get_image_size(file);
			if (file_len <= 0) {
				DHD_ERROR(("%s: bad fw file length !\n", __FUNCTION__));
				goto error;
			}
			/* fwid is at the end of fw bin in string format */
			if (dhd_os_seek_file(file, file_len - (sizeof(fwid_str) - 1)) < 0) {
				DHD_ERROR(("%s: can't seek file \n", __FUNCTION__));
				goto error;
			}

			memset(fwid_str, 0, sizeof(fwid_str));
			if (dhd_os_get_image_block(fwid_str, sizeof(fwid_str) - 1, file) <= 0) {
				DHD_ERROR(("%s: read fw file failed !\n", __FUNCTION__));
				goto error;
			}
#endif /* DHD_LINUX_STD_FW_API */
			pfw_id = (uint8 *)bcmstrnstr(fwid_str, sizeof(fwid_str) - 1,
					FWID_STR_1, strlen(FWID_STR_1));
			if (!pfw_id) {
				pfw_id = (uint8 *)bcmstrnstr(fwid_str, sizeof(fwid_str) - 1,
					FWID_STR_2, strlen(FWID_STR_2));
				if (!pfw_id) {
					DHD_ERROR(("%s: could not find id in FW bin!\n",
							__FUNCTION__));
					goto error;
				}
			}
			/* search for the '-' in the fw id str, after which the
			 * actual 4 byte fw id is present
			 */
			while (pfw_id && *pfw_id != '-') {
				++pfw_id;
			}
			++pfw_id;
			fwid = bcm_strtoul((char *)pfw_id, NULL, 16);

			/* check if fw id in logstrs.bin matches the fw one */
			if (hdr->fw_id != fwid) {
				DHD_ERROR(("%s: logstr id does not match FW!"
					"logstrs_fwid:0x%x, rtecdc_fwid:0x%x\n",
					__FUNCTION__, hdr->fw_id, fwid));
				goto error;
			}

			match_fail = FALSE;
			num_fmts = hdr->rom_logstrs_offset / sizeof(uint32);
			ram_index = (hdr->ram_lognums_offset -
				hdr->rom_lognums_offset) / sizeof(uint32);
			lognums = (uint32 *) &raw_fmts[hdr->rom_lognums_offset];
			logstrs = (char *)	 &raw_fmts[hdr->rom_logstrs_offset];
			hdr_logstrs_size = hdr->logstrs_size;

error:
#ifdef DHD_LINUX_STD_FW_API
			if (fw) {
				dhd_os_close_img_fwreq(fw);
			}
#else
			if (file) {
				dhd_os_close_image1(NULL, file);
			}
#endif /* DHD_LINUX_STD_FW_API */
			if (match_fail) {
				return BCME_DECERR;
			}
		} else {
			DHD_ERROR(("%s: Invalid logstr version %u\n", __FUNCTION__,
					trailer->version));
			return BCME_ERROR;
		}
		if (logstrs_size != hdr_logstrs_size) {
			DHD_ERROR(("%s: bad logstrs_size %d\n", __FUNCTION__, hdr_logstrs_size));
			return BCME_ERROR;
		}
	} else {
		/*
		 * Legacy logstrs.bin format without header.
		 */
		num_fmts = *((uint32 *) (raw_fmts)) / sizeof(uint32);

		/* Legacy RAM-only logstrs.bin format:
		 *	  - RAM 'lognums' section
		 *	  - RAM 'logstrs' section.
		 *
		 * 'lognums' is an array of indexes for the strings in the
		 * 'logstrs' section. The first uint32 is an index to the
		 * start of 'logstrs'. Therefore, if this index is divided
		 * by 'sizeof(uint32)' it provides the number of logstr
		 *	entries.
		 */
		ram_index = 0;
		lognums = (uint32 *) raw_fmts;
		logstrs = (char *) &raw_fmts[num_fmts << 2];
	}
	if (num_fmts) {
		if (event_log->fmts != NULL) {
			fmts = event_log->fmts;	/* reuse existing malloced fmts */
		} else {
			fmts = MALLOC(osh, num_fmts  * sizeof(char *));
		}
	}
	if (fmts == NULL) {
		DHD_ERROR(("%s: Failed to allocate fmts memory\n", __FUNCTION__));
		return BCME_ERROR;
	}
	event_log->fmts_size = num_fmts  * sizeof(char *);

	for (i = 0; i < num_fmts; i++) {
		/* ROM lognums index into logstrs using 'rom_logstrs_offset' as a base
		* (they are 0-indexed relative to 'rom_logstrs_offset').
		*
		* RAM lognums are already indexed to point to the correct RAM logstrs (they
		* are 0-indexed relative to the start of the logstrs.bin file).
		*/
		if (i == ram_index) {
			logstrs = raw_fmts;
		}
		fmts[i] = &logstrs[lognums[i]];
	}
	event_log->fmts = fmts;
	event_log->raw_fmts_size = logstrs_size;
	event_log->raw_fmts = raw_fmts;
	event_log->num_fmts = num_fmts;
	return BCME_OK;
} /* dhd_parse_logstrs_file */

#ifdef DHD_LINUX_STD_FW_API
int dhd_parse_map_file(osl_t *osh, const void *ptr, uint32 *ramstart, uint32 *rodata_start,
		uint32 *rodata_end)
{
	char *raw_fmts =  NULL, *raw_fmts_loc = NULL;
	uint32 read_size = READ_NUM_BYTES, offset = 0;
	int error = 0;
	char * cptr = NULL;
	char c;
	uint8 count = 0;
	uint32 size = 0;

	*ramstart = 0;
	*rodata_start = 0;
	*rodata_end = 0;
	size = (uint32)(((const struct firmware *)ptr)->size);

	/* Allocate 1 byte more than read_size to terminate it with NULL */
	raw_fmts = MALLOCZ(osh, read_size + 1);
	if (raw_fmts == NULL) {
		DHD_ERROR(("%s: Failed to allocate raw_fmts memory \n", __FUNCTION__));
		goto fail;
	}

	/* read ram start, rodata_start and rodata_end values from map  file */
	while (count != ALL_MAP_VAL)
	{
		/* Bound check for size before doing memcpy() */
		if ((offset + read_size) > size) {
			read_size = size - offset;
		}

		error = memcpy_s(raw_fmts, read_size,
			(((const char *)((const struct firmware *)ptr)->data) + offset), read_size);
		if (error) {
			DHD_ERROR(("%s: failed to copy raw_fmts, err=%d\n",
				__FUNCTION__, error));
			goto fail;
		}
		/* End raw_fmts with NULL as strstr expects NULL terminated strings */
		raw_fmts[read_size] = '\0';

		/* Get ramstart address */
		raw_fmts_loc = raw_fmts;
		if (!(count & RAMSTART_BIT) &&
			(cptr = bcmstrnstr(raw_fmts_loc, read_size, ramstart_str,
			strlen(ramstart_str)))) {
			cptr = cptr - BYTES_AHEAD_NUM;
			sscanf(cptr, "%x %c text_start", ramstart, &c);
			count |= RAMSTART_BIT;
		}

		/* Get ram rodata start address */
		raw_fmts_loc = raw_fmts;
		if (!(count & RDSTART_BIT) &&
			(cptr = bcmstrnstr(raw_fmts_loc, read_size, rodata_start_str,
			strlen(rodata_start_str)))) {
			cptr = cptr - BYTES_AHEAD_NUM;
			sscanf(cptr, "%x %c rodata_start", rodata_start, &c);
			count |= RDSTART_BIT;
		}

		/* Get ram rodata end address */
		raw_fmts_loc = raw_fmts;
		if (!(count & RDEND_BIT) &&
			(cptr = bcmstrnstr(raw_fmts_loc, read_size, rodata_end_str,
			strlen(rodata_end_str)))) {
			cptr = cptr - BYTES_AHEAD_NUM;
			sscanf(cptr, "%x %c rodata_end", rodata_end, &c);
			count |= RDEND_BIT;
		}

		if ((offset + read_size) >= size) {
			break;
		}

		memset(raw_fmts, 0, read_size);
		offset += (read_size - GO_BACK_FILE_POS_NUM_BYTES);
	}

fail:
	if (raw_fmts) {
		MFREE(osh, raw_fmts, read_size + 1);
		raw_fmts = NULL;
	}
	if (count == ALL_MAP_VAL) {
		return BCME_OK;
	}
	else {
		DHD_ERROR(("%s: readmap error 0X%x \n", __FUNCTION__,
				count));
		return BCME_ERROR;
	}
} /* dhd_parse_map_file */
#else
int dhd_parse_map_file(osl_t *osh, void *file, uint32 *ramstart, uint32 *rodata_start,
		uint32 *rodata_end)
{
	char *raw_fmts =  NULL, *raw_fmts_loc = NULL;
	uint32 read_size = READ_NUM_BYTES;
	int error = 0;
	char * cptr = NULL;
	char c;
	uint8 count = 0;

	*ramstart = 0;
	*rodata_start = 0;
	*rodata_end = 0;

	/* Allocate 1 byte more than read_size to terminate it with NULL */
	raw_fmts = MALLOCZ(osh, read_size + 1);
	if (raw_fmts == NULL) {
		DHD_ERROR(("%s: Failed to allocate raw_fmts memory \n", __FUNCTION__));
		goto fail;
	}

	/* read ram start, rodata_start and rodata_end values from map  file */
	while (count != ALL_MAP_VAL)
	{
		error = dhd_os_read_file(file, raw_fmts, read_size);
		if (error < 0) {
			DHD_ERROR(("%s: map file read failed err:%d \n", __FUNCTION__,
					error));
			goto fail;
		}

		/* End raw_fmts with NULL as strstr expects NULL terminated strings */
		raw_fmts[read_size] = '\0';

		/* Get ramstart address */
		raw_fmts_loc = raw_fmts;
		if (!(count & RAMSTART_BIT) &&
			(cptr = bcmstrnstr(raw_fmts_loc, read_size, ramstart_str,
			strlen(ramstart_str)))) {
			cptr = cptr - BYTES_AHEAD_NUM;
			sscanf(cptr, "%x %c text_start", ramstart, &c);
			count |= RAMSTART_BIT;
		}

		/* Get ram rodata start address */
		raw_fmts_loc = raw_fmts;
		if (!(count & RDSTART_BIT) &&
			(cptr = bcmstrnstr(raw_fmts_loc, read_size, rodata_start_str,
			strlen(rodata_start_str)))) {
			cptr = cptr - BYTES_AHEAD_NUM;
			sscanf(cptr, "%x %c rodata_start", rodata_start, &c);
			count |= RDSTART_BIT;
		}

		/* Get ram rodata end address */
		raw_fmts_loc = raw_fmts;
		if (!(count & RDEND_BIT) &&
			(cptr = bcmstrnstr(raw_fmts_loc, read_size, rodata_end_str,
			strlen(rodata_end_str)))) {
			cptr = cptr - BYTES_AHEAD_NUM;
			sscanf(cptr, "%x %c rodata_end", rodata_end, &c);
			count |= RDEND_BIT;
		}

		if (error < (int)read_size) {
			/*
			* since we reset file pos back to earlier pos by
			* GO_BACK_FILE_POS_NUM_BYTES bytes we won't reach EOF.
			* The reason for this is if string is spreaded across
			* bytes, the read function should not miss it.
			* So if ret value is less than read_size, reached EOF don't read further
			*/
			break;
		}
		memset(raw_fmts, 0, read_size);
		/*
		* go back to predefined NUM of bytes so that we won't miss
		* the string and  addr even if it comes as splited in next read.
		*/
		dhd_os_seek_file(file, -GO_BACK_FILE_POS_NUM_BYTES);
	}

fail:
	if (raw_fmts) {
		MFREE(osh, raw_fmts, read_size + 1);
		raw_fmts = NULL;
	}
	if (count == ALL_MAP_VAL) {
		return BCME_OK;
	}
	else {
		DHD_ERROR(("%s: readmap error 0X%x \n", __FUNCTION__,
				count));
		return BCME_ERROR;
	}

} /* dhd_parse_map_file */
#endif /* DHD_LINUX_STD_FW_API */

#ifdef PCIE_FULL_DONGLE
int
dhd_event_logtrace_infobuf_pkt_process(dhd_pub_t *dhdp, void *pktbuf,
		dhd_event_log_t *event_data)
{
	uint32 infobuf_version;
	info_buf_payload_hdr_t *payload_hdr_ptr;
	uint16 payload_hdr_type;
	uint16 payload_hdr_length;

	DHD_TRACE(("%s:Enter\n", __FUNCTION__));

	if (PKTLEN(dhdp->osh, pktbuf) < sizeof(uint32)) {
		DHD_ERROR(("%s: infobuf too small for version field\n",
			__FUNCTION__));
		goto exit;
	}
	infobuf_version = *((uint32 *)PKTDATA(dhdp->osh, pktbuf));
	PKTPULL(dhdp->osh, pktbuf, sizeof(uint32));
	if (infobuf_version != PCIE_INFOBUF_V1) {
		DHD_ERROR(("%s: infobuf version %d is not PCIE_INFOBUF_V1\n",
			__FUNCTION__, infobuf_version));
		goto exit;
	}

	/* Version 1 infobuf has a single type/length (and then value) field */
	if (PKTLEN(dhdp->osh, pktbuf) < sizeof(info_buf_payload_hdr_t)) {
		DHD_ERROR(("%s: infobuf too small for v1 type/length  fields\n",
			__FUNCTION__));
		goto exit;
	}
	/* Process/parse the common info payload header (type/length) */
	payload_hdr_ptr = (info_buf_payload_hdr_t *)PKTDATA(dhdp->osh, pktbuf);
	payload_hdr_type = ltoh16(payload_hdr_ptr->type);
	payload_hdr_length = ltoh16(payload_hdr_ptr->length);
	if (payload_hdr_type != PCIE_INFOBUF_V1_TYPE_LOGTRACE) {
		DHD_ERROR(("%s: payload_hdr_type %d is not V1_TYPE_LOGTRACE\n",
			__FUNCTION__, payload_hdr_type));
		goto exit;
	}
	PKTPULL(dhdp->osh, pktbuf, sizeof(info_buf_payload_hdr_t));

	/* Validate that the specified length isn't bigger than the
	 * provided data.
	 */
	if (payload_hdr_length > PKTLEN(dhdp->osh, pktbuf)) {
		DHD_ERROR(("%s: infobuf logtrace length is bigger"
			" than actual buffer data\n", __FUNCTION__));
		goto exit;
	}
	dhd_dbg_trace_evnt_handler(dhdp, PKTDATA(dhdp->osh, pktbuf),
		event_data, payload_hdr_length);

	return BCME_OK;

exit:
	return BCME_ERROR;
} /* dhd_event_logtrace_infobuf_pkt_process */
#endif /* PCIE_FULL_DONGLE */
#endif /* SHOW_LOGTRACE */

#if defined(WLTDLS) && defined(PCIE_FULL_DONGLE)

/* To handle the TDLS event in the dhd_common.c
 */
int dhd_tdls_event_handler(dhd_pub_t *dhd_pub, wl_event_msg_t *event)
{
	int ret = BCME_OK;

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST()
	ret = dhd_tdls_update_peer_info(dhd_pub, event);
	GCC_DIAGNOSTIC_POP()

	return ret;
}

int dhd_free_tdls_peer_list(dhd_pub_t *dhd_pub)
{
	tdls_peer_node_t *cur = NULL, *prev = NULL;
	if (!dhd_pub)
		return BCME_ERROR;
	cur = dhd_pub->peer_tbl.node;

	if ((dhd_pub->peer_tbl.node == NULL) && !dhd_pub->peer_tbl.tdls_peer_count)
		return BCME_ERROR;

	while (cur != NULL) {
		prev = cur;
		cur = cur->next;
		MFREE(dhd_pub->osh, prev, sizeof(tdls_peer_node_t));
	}
	dhd_pub->peer_tbl.tdls_peer_count = 0;
	dhd_pub->peer_tbl.node = NULL;
	return BCME_OK;
}
#endif /* #if defined(WLTDLS) && defined(PCIE_FULL_DONGLE) */

/* pretty hex print a contiguous buffer
* based on the debug level specified
*/
void
dhd_prhex(const char *msg, volatile uchar *buf, uint nbytes, uint8 dbg_level)
{
	char line[128], *p;
	int len = sizeof(line);
	int nchar;
	uint i;

	if (msg && (msg[0] != '\0')) {
		if (dbg_level == DHD_ERROR_VAL)
			DHD_ERROR(("%s:\n", msg));
		else if (dbg_level == DHD_INFO_VAL)
			DHD_INFO(("%s:\n", msg));
		else if (dbg_level == DHD_TRACE_VAL)
			DHD_TRACE(("%s:\n", msg));
	}

	p = line;
	for (i = 0; i < nbytes; i++) {
		if (i % 16 == 0) {
			nchar = snprintf(p, len, "  %04x: ", i);	/* line prefix */
			p += nchar;
			len -= nchar;
		}
		if (len > 0) {
			nchar = snprintf(p, len, "%02x ", buf[i]);
			p += nchar;
			len -= nchar;
		}

		if (i % 16 == 15) {
			/* flush line */
			if (dbg_level == DHD_ERROR_VAL)
				DHD_ERROR(("%s:\n", line));
			else if (dbg_level == DHD_INFO_VAL)
				DHD_INFO(("%s:\n", line));
			else if (dbg_level == DHD_TRACE_VAL)
				DHD_TRACE(("%s:\n", line));
			p = line;
			len = sizeof(line);
		}
	}

	/* flush last partial line */
	if (p != line) {
		if (dbg_level == DHD_ERROR_VAL)
			DHD_ERROR(("%s:\n", line));
		else if (dbg_level == DHD_INFO_VAL)
			DHD_INFO(("%s:\n", line));
		else if (dbg_level == DHD_TRACE_VAL)
			DHD_TRACE(("%s:\n", line));
	}
}

int
dhd_tput_test(dhd_pub_t *dhd, tput_test_t *tput_data)
{
	struct ether_header ether_hdr;
	tput_pkt_t tput_pkt;
	void *pkt = NULL;
	uint8 *pktdata = NULL;
	uint32 pktsize = 0;
	uint64 total_size = 0;
	uint32 *crc = 0;
	uint32 pktid = 0;
	uint32 total_num_tx_pkts = 0;
	int err = 0, err_exit = 0;
	uint32 i = 0;
	uint64 time_taken = 0;
	int max_txbufs = 0;
	uint32 n_batches = 0;
	uint32 n_remain = 0;
	uint8 tput_pkt_hdr_size = 0;
	bool batch_cnt = FALSE;
	bool tx_stop_pkt = FALSE;

	if (tput_data->version != TPUT_TEST_T_VER ||
		tput_data->length != TPUT_TEST_T_LEN) {
		DHD_ERROR(("%s: wrong structure ver/len! \n", __FUNCTION__));
		err_exit = BCME_BADARG;
		goto exit_error;
	}

	if (dhd->tput_data.tput_test_running) {
		DHD_ERROR(("%s: tput test already running ! \n", __FUNCTION__));
		err_exit = BCME_BUSY;
		goto exit_error;
	}
#ifdef PCIE_FULL_DONGLE
	/*
	 * 100 bytes to accommodate ether header and tput header. As of today
	 * both occupy 30 bytes. Rest is reserved.
	 */
	if ((tput_data->payload_size > TPUT_TEST_MAX_PAYLOAD) ||
		(tput_data->payload_size > (DHD_FLOWRING_RX_BUFPOST_PKTSZ - 100))) {
		DHD_ERROR(("%s: payload size is too large! max_payload=%u rx_bufpost_size=%u\n",
			__FUNCTION__, TPUT_TEST_MAX_PAYLOAD,
			(DHD_FLOWRING_RX_BUFPOST_PKTSZ - 100)));
		err_exit = BCME_BUFTOOLONG;
		goto exit_error;
	}
#endif
	max_txbufs = dhd_get_max_txbufs(dhd);
	max_txbufs = MIN(max_txbufs, DHD_TPUT_MAX_TX_PKTS_BATCH);

	if (!(tput_data->num_pkts > 0)) {
		DHD_ERROR(("%s: invalid num_pkts: %d to tx\n",
			__FUNCTION__, tput_data->num_pkts));
		err_exit = BCME_ERROR;
		goto exit_error;
	}

	memset(&dhd->tput_data, 0, sizeof(dhd->tput_data));
	memcpy(&dhd->tput_data, tput_data, sizeof(*tput_data));
	dhd->tput_data.pkts_bad = dhd->tput_data.pkts_good = 0;
	dhd->tput_data.pkts_cmpl = 0;
	dhd->tput_start_ts = dhd->tput_stop_ts = 0;

	if (tput_data->flags & TPUT_TEST_USE_ETHERNET_HDR) {
		pktsize = sizeof(ether_hdr) + sizeof(tput_pkt_t) +
				(tput_data->payload_size - 12);
	} else {
		pktsize = sizeof(tput_pkt_t) +
				(tput_data->payload_size - 12);
	}

	tput_pkt_hdr_size = (uint8)((uint8 *)&tput_pkt.crc32 -
			(uint8 *)&tput_pkt.mac_sta);

	/* mark the tput test as started */
	dhd->tput_data.tput_test_running = TRUE;

	if (tput_data->direction == TPUT_DIR_TX) {
		/* for ethernet header */
		memcpy(ether_hdr.ether_shost, tput_data->mac_sta, ETHER_ADDR_LEN);
		memcpy(ether_hdr.ether_dhost, tput_data->mac_ap, ETHER_ADDR_LEN);
		ether_hdr.ether_type = hton16(ETHER_TYPE_IP);

		/* fill in the tput pkt */
		memset(&tput_pkt, 0, sizeof(tput_pkt));
		memcpy(tput_pkt.mac_ap, tput_data->mac_ap, ETHER_ADDR_LEN);
		memcpy(tput_pkt.mac_sta, tput_data->mac_sta, ETHER_ADDR_LEN);
		tput_pkt.pkt_type = hton16(TPUT_PKT_TYPE_NORMAL);
		tput_pkt.num_pkts = hton32(tput_data->num_pkts);

		if (tput_data->num_pkts > (uint32)max_txbufs) {
			n_batches = tput_data->num_pkts / max_txbufs;
			n_remain = tput_data->num_pkts % max_txbufs;
		} else {
			n_batches = 0;
			n_remain = tput_data->num_pkts;
		}
		DHD_ERROR(("%s: num_pkts: %u n_batches: %u n_remain: %u\n",
			__FUNCTION__, tput_data->num_pkts, n_batches, n_remain));

		do {
			/* reset before every batch */
			dhd->batch_tx_pkts_cmpl = 0;
			if (n_batches) {
				dhd->batch_tx_num_pkts = max_txbufs;
				--n_batches;
			} else if (n_remain) {
				dhd->batch_tx_num_pkts = n_remain;
				n_remain = 0;
			} else {
				DHD_ERROR(("Invalid. This should not hit\n"));
			}

			dhd->tput_start_ts = OSL_SYSUPTIME_US();
			for (i = 0; (i < dhd->batch_tx_num_pkts) || (tx_stop_pkt); ++i) {
				pkt = PKTGET(dhd->osh, pktsize, TRUE);
				if (!pkt) {
					dhd->tput_data.tput_test_running = FALSE;
					DHD_ERROR(("%s: PKTGET fails ! Not enough Tx buffers\n",
						__FUNCTION__));
					DHD_ERROR(("%s: pkts_good:%u; pkts_bad:%u; pkts_cmpl:%u\n",
						__FUNCTION__, dhd->tput_data.pkts_good,
						dhd->tput_data.pkts_bad, dhd->tput_data.pkts_cmpl));
					err_exit = BCME_NOMEM;
					goto exit_error;
				}
				pktdata = PKTDATA(dhd->osh, pkt);
				PKTSETLEN(dhd->osh, pkt, pktsize);
				memset(pktdata, 0, pktsize);
				if (tput_data->flags & TPUT_TEST_USE_ETHERNET_HDR) {
					memcpy(pktdata, &ether_hdr, sizeof(ether_hdr));
					pktdata += sizeof(ether_hdr);
				}
				/* send stop pkt as last pkt */
				if (tx_stop_pkt) {
					tput_pkt.pkt_type = hton16(TPUT_PKT_TYPE_STOP);
					tx_stop_pkt = FALSE;
				} else
					tput_pkt.pkt_type = hton16(TPUT_PKT_TYPE_NORMAL);
				tput_pkt.pkt_id = hton32(pktid++);
				tput_pkt.crc32 = 0;
				memcpy(pktdata, &tput_pkt, sizeof(tput_pkt));
				/* compute crc32 over the pkt-id, num-pkts and data fields */
				crc = (uint32 *)(pktdata + tput_pkt_hdr_size);
				*crc = hton32(hndcrc32(pktdata + tput_pkt_hdr_size + 4,
						8 + (tput_data->payload_size - 12),
						CRC32_INIT_VALUE));

				err = dhd_sendpkt(dhd, 0, pkt);
				if (err != BCME_OK) {
					DHD_INFO(("%s: send pkt (id = %u) fails (err = %d) ! \n",
						__FUNCTION__, pktid, err));
					dhd->tput_data.pkts_bad++;
				}
				total_num_tx_pkts++;
				if ((total_num_tx_pkts == tput_data->num_pkts) && (!tx_stop_pkt)) {
					tx_stop_pkt = TRUE;
				}
			}
			DHD_INFO(("%s: TX done, wait for completion...\n", __FUNCTION__));
			if (!dhd_os_tput_test_wait(dhd, NULL,
					TPUT_TEST_WAIT_TIMEOUT_DEFAULT)) {
				dhd->tput_stop_ts = OSL_SYSUPTIME_US();
				dhd->tput_data.tput_test_running = FALSE;
				DHD_ERROR(("%s: TX completion timeout !"
					" Total Tx pkts (including STOP) = %u; pkts cmpl = %u; \n",
					__FUNCTION__, total_num_tx_pkts, dhd->batch_tx_pkts_cmpl));
				err_exit = BCME_ERROR;
				goto exit_error;
			}
			if ((dhd->tput_start_ts && dhd->tput_stop_ts &&
				(dhd->tput_stop_ts > dhd->tput_start_ts)) || (time_taken)) {
				if (!time_taken) {
					time_taken = dhd->tput_stop_ts - dhd->tput_start_ts;
				}
			} else {
				dhd->tput_data.tput_test_running = FALSE;
				DHD_ERROR(("%s: bad timestamp while cal tx batch time\n",
					__FUNCTION__));
				err_exit = BCME_ERROR;
				goto exit_error;
			}
			if (n_batches || n_remain) {
				batch_cnt = TRUE;
			} else {
				batch_cnt = FALSE;
			}
		} while (batch_cnt);
	} else {
		/* TPUT_DIR_RX */
		DHD_INFO(("%s: waiting for RX completion... \n", __FUNCTION__));
		if (!dhd_os_tput_test_wait(dhd, NULL, tput_data->timeout_ms)) {
			DHD_ERROR(("%s: RX completion timeout ! \n", __FUNCTION__));
			dhd->tput_stop_ts = OSL_SYSUPTIME_US();
		}
	}

	/* calculate the throughput in bits per sec */
	if (dhd->tput_start_ts && dhd->tput_stop_ts &&
		(dhd->tput_stop_ts > dhd->tput_start_ts)) {
		time_taken = dhd->tput_stop_ts - dhd->tput_start_ts;
		time_taken = DIV_U64_BY_U32(time_taken, MSEC_PER_SEC); /* convert to ms */
		dhd->tput_data.time_ms = time_taken;
		if (time_taken) {
			total_size = pktsize * dhd->tput_data.pkts_cmpl * 8;
			dhd->tput_data.tput_bps = DIV_U64_BY_U64(total_size, time_taken);
			/* convert from ms to seconds */
			dhd->tput_data.tput_bps = dhd->tput_data.tput_bps * 1000;
		}
	} else {
		DHD_ERROR(("%s: bad timestamp !\n", __FUNCTION__));
	}
	DHD_INFO(("%s: DONE. tput = %llu bps, time = %llu ms\n", __FUNCTION__,
		dhd->tput_data.tput_bps, dhd->tput_data.time_ms));

	memcpy(tput_data, &dhd->tput_data, sizeof(dhd->tput_data));

	dhd->tput_data.tput_test_running = FALSE;

	err_exit = BCME_OK;

exit_error:
	DHD_ERROR(("%s: pkts_good = %u; pkts_bad = %u; pkts_cmpl = %u\n",
		__FUNCTION__, dhd->tput_data.pkts_good,
		dhd->tput_data.pkts_bad, dhd->tput_data.pkts_cmpl));

	return err_exit;
}

void
dhd_tput_test_rx(dhd_pub_t *dhd, void *pkt)
{
	uint8 *pktdata = NULL;
	tput_pkt_t *tput_pkt = NULL;
	uint32 crc = 0;
	uint8 tput_pkt_hdr_size = 0;

	pktdata = PKTDATA(dhd->osh, pkt);
	if (dhd->tput_data.flags & TPUT_TEST_USE_ETHERNET_HDR)
		pktdata += sizeof(struct ether_header);
	tput_pkt = (tput_pkt_t *)pktdata;

	/* record the timestamp of the first packet received */
	if (dhd->tput_data.pkts_cmpl == 0) {
		dhd->tput_start_ts = OSL_SYSUPTIME_US();
	}

	if (ntoh16(tput_pkt->pkt_type) != TPUT_PKT_TYPE_STOP &&
			dhd->tput_data.pkts_cmpl <= dhd->tput_data.num_pkts) {
		dhd->tput_data.pkts_cmpl++;
	}
	/* drop rx packets received beyond the specified # */
	if (dhd->tput_data.pkts_cmpl > dhd->tput_data.num_pkts)
		return;

	DHD_TRACE(("%s: Rx tput test pkt, id = %u ; type = %u\n", __FUNCTION__,
		ntoh32(tput_pkt->pkt_id), ntoh16(tput_pkt->pkt_type)));

	/* discard if mac addr of AP/STA does not match the specified ones */
	if ((memcmp(tput_pkt->mac_ap, dhd->tput_data.mac_ap,
			ETHER_ADDR_LEN) != 0) ||
		(memcmp(tput_pkt->mac_sta, dhd->tput_data.mac_sta,
			ETHER_ADDR_LEN) != 0)) {
		dhd->tput_data.pkts_bad++;
		DHD_INFO(("%s: dropping tput pkt with id %u due to bad AP/STA mac !\n",
			__FUNCTION__, ntoh32(tput_pkt->pkt_id)));
		return;
	}

	tput_pkt_hdr_size = (uint8)((uint8 *)&tput_pkt->crc32 -
			(uint8 *)&tput_pkt->mac_sta);
	pktdata += tput_pkt_hdr_size + 4;
	crc = hndcrc32(pktdata, 8 + (dhd->tput_data.payload_size - 12),
			CRC32_INIT_VALUE);
	if (crc != ntoh32(tput_pkt->crc32)) {
		DHD_INFO(("%s: dropping tput pkt with id %u due to bad CRC !\n",
			__FUNCTION__, ntoh32(tput_pkt->pkt_id)));
		dhd->tput_data.pkts_bad++;
		return;
	}

	if (ntoh16(tput_pkt->pkt_type) != TPUT_PKT_TYPE_STOP)
		dhd->tput_data.pkts_good++;

	/* if we have received the stop packet or all the # of pkts, we're done */
	if (ntoh16(tput_pkt->pkt_type) == TPUT_PKT_TYPE_STOP ||
			dhd->tput_data.pkts_cmpl == dhd->tput_data.num_pkts) {
		dhd->tput_stop_ts = OSL_SYSUPTIME_US();
		dhd_os_tput_test_wake(dhd);
	}
}

#ifdef DUMP_IOCTL_IOV_LIST
void
dhd_iov_li_append(dhd_pub_t *dhd, dll_t *list_head, dll_t *node)
{
	dll_t *item;
	dhd_iov_li_t *iov_li;
	dhd->dump_iovlist_len++;

	if (dhd->dump_iovlist_len == IOV_LIST_MAX_LEN+1) {
		item = dll_head_p(list_head);
		iov_li = (dhd_iov_li_t *)CONTAINEROF(item, dhd_iov_li_t, list);
		dll_delete(item);
		MFREE(dhd->osh, iov_li, sizeof(*iov_li));
		dhd->dump_iovlist_len--;
	}
	dll_append(list_head, node);
}

void
dhd_iov_li_print(dll_t *list_head)
{
	dhd_iov_li_t *iov_li;
	dll_t *item, *next;
	uint8 index = 0;
	for (item = dll_head_p(list_head); !dll_end(list_head, item); item = next) {
		next = dll_next_p(item);
		iov_li = (dhd_iov_li_t *)CONTAINEROF(item, dhd_iov_li_t, list);
		DHD_ERROR(("%d:cmd_name = %s, cmd = %d.\n", ++index, iov_li->buff, iov_li->cmd));
	}
}

void
dhd_iov_li_delete(dhd_pub_t *dhd, dll_t *list_head)
{
	dll_t *item;
	dhd_iov_li_t *iov_li;
	while (!(dll_empty(list_head))) {
		item = dll_head_p(list_head);
		iov_li = (dhd_iov_li_t *)CONTAINEROF(item, dhd_iov_li_t, list);
		dll_delete(item);
		MFREE(dhd->osh, iov_li, sizeof(*iov_li));
	}
}
#endif /* DUMP_IOCTL_IOV_LIST */

#ifdef EWP_EDL
/* For now we are allocating memory for EDL ring using DMA_ALLOC_CONSISTENT
* The reason being that, in hikey, if we try to DMA_MAP prealloced memory
* it is failing with an 'out of space in SWIOTLB' error
*/
int
dhd_edl_mem_init(dhd_pub_t *dhd)
{
	int ret = 0;

	memset(&dhd->edl_ring_mem, 0, sizeof(dhd->edl_ring_mem));
	ret = dhd_dma_buf_alloc(dhd, &dhd->edl_ring_mem, DHD_EDL_RING_SIZE);
	if (ret != BCME_OK) {
		DHD_ERROR(("%s: alloc of edl_ring_mem failed\n",
			__FUNCTION__));
		return BCME_ERROR;
	}
	return BCME_OK;
}

/*
 * NOTE:- that dhd_edl_mem_deinit need NOT be called explicitly, because the dma_buf
 * for EDL is freed during 'dhd_prot_detach_edl_rings' which is called during de-init.
 */
void
dhd_edl_mem_deinit(dhd_pub_t *dhd)
{
	if (dhd->edl_ring_mem.va != NULL)
		dhd_dma_buf_free(dhd, &dhd->edl_ring_mem);
}

int
dhd_event_logtrace_process_edl(dhd_pub_t *dhdp, uint8 *data,
		void *evt_decode_data)
{
	msg_hdr_edl_t *msg = NULL;
	cmn_msg_hdr_t *cmn_msg_hdr = NULL;
	uint8 *buf = NULL;

	if (!data || !dhdp || !evt_decode_data) {
		DHD_ERROR(("%s: invalid args ! \n", __FUNCTION__));
		return BCME_ERROR;
	}

	/* format of data in each work item in the EDL ring:
	* |cmn_msg_hdr_t |payload (var len)|cmn_msg_hdr_t|
	* payload = |infobuf_ver(u32)|info_buf_payload_hdr_t|msgtrace_hdr_t|<var len data>|
	*/
	cmn_msg_hdr = (cmn_msg_hdr_t *)data;
	msg = (msg_hdr_edl_t *)(data + sizeof(cmn_msg_hdr_t));
	buf = (uint8 *)msg;
	/* validate the fields */
	if (ltoh32(msg->infobuf_ver) != PCIE_INFOBUF_V1) {
		DHD_ERROR(("%s: Skipping msg with invalid infobuf ver (0x%x)"
			" expected (0x%x)\n", __FUNCTION__,
			msg->infobuf_ver, PCIE_INFOBUF_V1));
		return BCME_VERSION;
	}

	/* in EDL, the request_id field of cmn_msg_hdr is overloaded to carry payload length */
	if (sizeof(info_buf_payload_hdr_t) > cmn_msg_hdr->request_id) {
		DHD_ERROR(("%s: infobuf too small for v1 type/length fields\n",
			__FUNCTION__));
		return BCME_BUFTOOLONG;
	}

	if (ltoh16(msg->pyld_hdr.type) != PCIE_INFOBUF_V1_TYPE_LOGTRACE) {
		DHD_ERROR(("%s: payload_hdr_type %d is not V1_TYPE_LOGTRACE\n",
			__FUNCTION__, ltoh16(msg->pyld_hdr.type)));
		return BCME_BADOPTION;
	}

	if (ltoh16(msg->pyld_hdr.length) > cmn_msg_hdr->request_id) {
		DHD_ERROR(("%s: infobuf logtrace length %u is bigger"
			" than available buffer size %u\n", __FUNCTION__,
			ltoh16(msg->pyld_hdr.length), cmn_msg_hdr->request_id));
		return BCME_BADLEN;
	}

	/* dhd_dbg_trace_evnt_handler expects the data to start from msgtrace_hdr_t */
	buf += sizeof(msg->infobuf_ver) + sizeof(msg->pyld_hdr);
	dhd_dbg_trace_evnt_handler(dhdp, buf, evt_decode_data,
		ltoh16(msg->pyld_hdr.length));

#ifdef SHOW_LOGTRACE
	/*
	 * check 'dhdp->logtrace_pkt_sendup' and if true alloc an skb
	 * copy the event data to the skb and send it up the stack
	 */
	if (dhdp->logtrace_pkt_sendup) {
		DHD_INFO(("%s: send up event log, len %u bytes\n", __FUNCTION__,
				(uint32)(ltoh16(msg->pyld_hdr.length) +
				sizeof(info_buf_payload_hdr_t) + 4)));
		dhd_sendup_info_buf(dhdp, (uint8 *)msg);
	}
#endif /* SHOW_LOGTRACE */

	return BCME_OK;
}
#endif /* EWP_EDL */

#if defined(DHD_LOG_DUMP) || defined(DHD_FW_COREDUMP)
#define DEBUG_DUMP_TRIGGER_INTERVAL_SEC	4
void
dhd_log_dump_trigger(dhd_pub_t *dhdp, int subcmd)
{
#if defined(DHD_DUMP_FILE_WRITE_FROM_KERNEL)
	log_dump_type_t *flush_type;
#endif /* DHD_DUMP_FILE_WRITE_FROM_KERNEL */
	uint64 current_time_sec;

	if (!dhdp) {
		DHD_ERROR(("dhdp is NULL !\n"));
		return;
	}

	if (subcmd >= CMD_MAX || subcmd < CMD_DEFAULT) {
		DHD_ERROR(("%s : Invalid subcmd \n", __FUNCTION__));
		return;
	}

	current_time_sec = DIV_U64_BY_U32(OSL_LOCALTIME_NS(), NSEC_PER_SEC);

	DHD_ERROR(("%s: current_time_sec=%lld debug_dump_time_sec=%lld interval=%d\n",
		__FUNCTION__, current_time_sec, dhdp->debug_dump_time_sec,
		DEBUG_DUMP_TRIGGER_INTERVAL_SEC));

	if ((current_time_sec - dhdp->debug_dump_time_sec) < DEBUG_DUMP_TRIGGER_INTERVAL_SEC) {
		DHD_ERROR(("%s : Last debug dump triggered(%lld) within %d seconds, so SKIP\n",
			__FUNCTION__, dhdp->debug_dump_time_sec, DEBUG_DUMP_TRIGGER_INTERVAL_SEC));
		return;
	}

	clear_debug_dump_time(dhdp->debug_dump_time_str);
#ifdef DHD_PCIE_RUNTIMEPM
	/* wake up RPM if SYSDUMP is triggered */
	dhdpcie_runtime_bus_wake(dhdp, TRUE, __builtin_return_address(0));
#endif /* DHD_PCIE_RUNTIMEPM */
	/*  */

	dhdp->debug_dump_subcmd = subcmd;

	dhdp->debug_dump_time_sec = DIV_U64_BY_U32(OSL_LOCALTIME_NS(), NSEC_PER_SEC);

#if defined(DHD_DUMP_FILE_WRITE_FROM_KERNEL)
	/* flush_type is freed at do_dhd_log_dump function */
	flush_type = MALLOCZ(dhdp->osh, sizeof(log_dump_type_t));
	if (flush_type) {
		*flush_type = DLD_BUF_TYPE_ALL;
		dhd_schedule_log_dump(dhdp, flush_type);
	} else {
		DHD_ERROR(("%s Fail to malloc flush_type\n", __FUNCTION__));
		return;
	}
#endif /* DHD_DUMP_FILE_WRITE_FROM_KERNEL */

	/* Inside dhd_mem_dump, event notification will be sent to HAL and
	 * from other context DHD pushes memdump, debug_dump and pktlog dump
	 * to HAL and HAL will write into file
	 */
#if (defined(BCMPCIE) || defined(BCMSDIO)) && defined(DHD_FW_COREDUMP)
	dhdp->memdump_type = DUMP_TYPE_BY_SYSDUMP;
	dhd_bus_mem_dump(dhdp);
#endif /* BCMPCIE && DHD_FW_COREDUMP */

#if defined(DHD_PKT_LOGGING) && defined(DHD_DUMP_FILE_WRITE_FROM_KERNEL)
	dhd_schedule_pktlog_dump(dhdp);
#endif /* DHD_PKT_LOGGING && DHD_DUMP_FILE_WRITE_FROM_KERNEL */
}
#endif /* DHD_LOG_DUMP || DHD_FW_COREDUMP */

#if defined(SHOW_LOGTRACE)
int
dhd_print_fw_ver_from_file(dhd_pub_t *dhdp, char *fwpath)
{
	void *file = NULL;
	int size = 0;
	char buf[FW_VER_STR_LEN];
	char *str = NULL;
	int ret = BCME_OK;

	if (!fwpath)
		return BCME_BADARG;

	file = dhd_os_open_image1(dhdp, fwpath);
	if (!file) {
		ret = BCME_ERROR;
		goto exit;
	}
	size = dhd_os_get_image_size(file);
	if (!size) {
		ret = BCME_ERROR;
		goto exit;
	}

	/* seek to the last 'X' bytes in the file */
	if (dhd_os_seek_file(file, size - FW_VER_STR_LEN) != BCME_OK) {
		ret = BCME_ERROR;
		goto exit;
	}

	/* read the last 'X' bytes of the file to a buffer */
	memset(buf, 0, FW_VER_STR_LEN);
	if (dhd_os_get_image_block(buf, FW_VER_STR_LEN - 1, file) < 0) {
		ret = BCME_ERROR;
		goto exit;
	}
	/* search for 'Version' in the buffer */
	str = bcmstrnstr(buf, FW_VER_STR_LEN, FW_VER_STR, strlen(FW_VER_STR));
	if (!str) {
		ret = BCME_ERROR;
		goto exit;
	}
	/* go back in the buffer to the last ascii character */
	while (str != buf &&
		(*str >= ' ' && *str <= '~')) {
		--str;
	}
	/* reverse the final decrement, so that str is pointing
	* to the first ascii character in the buffer
	*/
	++str;

	if (strlen(str) > (FW_VER_STR_LEN - 1)) {
		ret = BCME_BADLEN;
		goto exit;
	}

	DHD_ERROR(("FW version in file '%s': %s\n", fwpath, str));
	/* copy to global variable, so that in case FW load fails, the
	* core capture logs will contain FW version read from the file
	*/
	memset(fw_version, 0, FW_VER_STR_LEN);
	strlcpy(fw_version, str, FW_VER_STR_LEN);

exit:
	if (file)
		dhd_os_close_image1(dhdp, file);

	return ret;
}
#endif

#ifdef WL_CFGVENDOR_SEND_HANG_EVENT

static void
copy_hang_info_ioctl_timeout(dhd_pub_t *dhd, int ifidx, wl_ioctl_t *ioc)
{
	int remain_len;
	int i;
	int *cnt;
	char *dest;
	int bytes_written;
	uint32 ioc_dwlen = 0;

	if (!dhd || !dhd->hang_info) {
		DHD_ERROR(("%s dhd=%p hang_info=%p\n",
			__FUNCTION__, dhd, (dhd ? dhd->hang_info : NULL)));
		return;
	}

	cnt = &dhd->hang_info_cnt;
	dest = dhd->hang_info;

	memset(dest, 0, VENDOR_SEND_HANG_EXT_INFO_LEN);
	(*cnt) = 0;

	bytes_written = 0;
	remain_len = VENDOR_SEND_HANG_EXT_INFO_LEN - bytes_written;

	get_debug_dump_time(dhd->debug_dump_time_hang_str);
	copy_debug_dump_time(dhd->debug_dump_time_str, dhd->debug_dump_time_hang_str);

	bytes_written += scnprintf(&dest[bytes_written], remain_len, "%d %d %s %d %d %d %d %d %d ",
			HANG_REASON_IOCTL_RESP_TIMEOUT, VENDOR_SEND_HANG_EXT_INFO_VER,
			dhd->debug_dump_time_hang_str,
			ifidx, ioc->cmd, ioc->len, ioc->set, ioc->used, ioc->needed);
	(*cnt) = HANG_FIELD_IOCTL_RESP_TIMEOUT_CNT;

	clear_debug_dump_time(dhd->debug_dump_time_hang_str);

	/* Access ioc->buf only if the ioc->len is more than 4 bytes */
	ioc_dwlen = (uint32)(ioc->len / sizeof(uint32));
	if (ioc_dwlen > 0) {
		const uint32 *ioc_buf = (const uint32 *)ioc->buf;

		remain_len = VENDOR_SEND_HANG_EXT_INFO_LEN - bytes_written;
		GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
		bytes_written += scnprintf(&dest[bytes_written], remain_len,
			"%08x", *(uint32 *)(ioc_buf++));
		GCC_DIAGNOSTIC_POP();
		(*cnt)++;
		if ((*cnt) >= HANG_FIELD_CNT_MAX) {
			return;
		}

		for (i = 1; i < ioc_dwlen && *cnt <= HANG_FIELD_CNT_MAX;
			i++, (*cnt)++) {
			remain_len = VENDOR_SEND_HANG_EXT_INFO_LEN - bytes_written;
			GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
			bytes_written += scnprintf(&dest[bytes_written], remain_len, "%c%08x",
				HANG_RAW_DEL, *(uint32 *)(ioc_buf++));
			GCC_DIAGNOSTIC_POP();
		}
	}

	DHD_INFO(("%s hang info len: %d data: %s\n",
		__FUNCTION__, (int)strlen(dhd->hang_info), dhd->hang_info));
}

#endif /* WL_CFGVENDOR_SEND_HANG_EVENT */

#if defined(DHD_H2D_LOG_TIME_SYNC)
/*
 * Helper function:
 * Used for Dongle console message time syncing with Host printk
 */
void dhd_h2d_log_time_sync(dhd_pub_t *dhd)
{
	uint64 ts;

	/*
	 * local_clock() returns time in nano seconds.
	 * Dongle understand only milli seconds time.
	 */
	ts = local_clock();
	/* Nano seconds to milli seconds */
	do_div(ts, 1000000);
	if (dhd_wl_ioctl_set_intiovar(dhd,  "rte_timesync", ts, WLC_SET_VAR, TRUE, 0)) {
		DHD_ERROR(("%s rte_timesync **** FAILED ****\n", __FUNCTION__));
		/* Stopping HOST Dongle console time syncing */
		dhd->dhd_rte_time_sync_ms = 0;
	}
}
#endif /* DHD_H2D_LOG_TIME_SYNC */

/* configuations of ecounters to be enabled by default in FW */
static ecounters_cfg_t ecounters_cfg_tbl[] = {
	/* Global ecounters */
	{ECOUNTERS_STATS_TYPES_FLAG_GLOBAL, 0x0, WL_IFSTATS_XTLV_BUS_PCIE},
	// {ECOUNTERS_STATS_TYPES_FLAG_GLOBAL, 0x0, WL_IFSTATS_XTLV_TX_AMPDU_STATS},
	// {ECOUNTERS_STATS_TYPES_FLAG_GLOBAL, 0x0, WL_IFSTATS_XTLV_RX_AMPDU_STATS},

	/* Slice specific ecounters */
	{ECOUNTERS_STATS_TYPES_FLAG_SLICE, 0x0, WL_SLICESTATS_XTLV_PERIODIC_STATE},
	{ECOUNTERS_STATS_TYPES_FLAG_SLICE, 0x1, WL_SLICESTATS_XTLV_PERIODIC_STATE},
	{ECOUNTERS_STATS_TYPES_FLAG_SLICE, 0x1, WL_IFSTATS_XTLV_WL_SLICE_BTCOEX},

	/* Interface specific ecounters */
	{ECOUNTERS_STATS_TYPES_FLAG_IFACE, 0x0, WL_IFSTATS_XTLV_IF_PERIODIC_STATE},
	{ECOUNTERS_STATS_TYPES_FLAG_IFACE, 0x0, WL_IFSTATS_XTLV_GENERIC},
	{ECOUNTERS_STATS_TYPES_FLAG_IFACE, 0x0, WL_IFSTATS_XTLV_INFRA_SPECIFIC},
	{ECOUNTERS_STATS_TYPES_FLAG_IFACE, 0x0, WL_IFSTATS_XTLV_MGT_CNT},

	/* secondary interface */
	/* XXX REMOVE for temporal, will be enabled after decision
	{ECOUNTERS_STATS_TYPES_FLAG_IFACE, 0x1, WL_IFSTATS_XTLV_IF_PERIODIC_STATE},
	{ECOUNTERS_STATS_TYPES_FLAG_IFACE, 0x1, WL_IFSTATS_XTLV_GENERIC},
	{ECOUNTERS_STATS_TYPES_FLAG_IFACE, 0x1, WL_IFSTATS_XTLV_INFRA_SPECIFIC},
	{ECOUNTERS_STATS_TYPES_FLAG_IFACE, 0x1, WL_IFSTATS_XTLV_MGT_CNT},
	*/
};

/* XXX: Same event id shall be defined in consecutive order in the below table */
static event_ecounters_cfg_t event_ecounters_cfg_tbl[] = {
	/* Interface specific event ecounters */
	{WLC_E_DEAUTH_IND, ECOUNTERS_STATS_TYPES_FLAG_IFACE, 0x0, WL_IFSTATS_XTLV_IF_EVENT_STATS},
};

/* Accepts an argument to -s, -g or -f and creates an XTLV */
int
dhd_create_ecounters_params(dhd_pub_t *dhd, uint16 type, uint16 if_slice_idx,
	uint16 stats_rep, uint8 **xtlv)
{
	uint8 *req_xtlv = NULL;
	ecounters_stats_types_report_req_t *req;
	bcm_xtlvbuf_t xtlvbuf, container_xtlvbuf;
	ecountersv2_xtlv_list_elt_t temp;
	uint16 xtlv_len = 0, total_len = 0;
	int rc = BCME_OK;

	/* fill in the stat type XTLV. For now there is no explicit TLV for the stat type. */
	temp.id = stats_rep;
	temp.len = 0;

	/* Hence len/data = 0/NULL */
	xtlv_len += temp.len + BCM_XTLV_HDR_SIZE;

	/* Total length of the container */
	total_len = BCM_XTLV_HDR_SIZE +
		OFFSETOF(ecounters_stats_types_report_req_t, stats_types_req) + xtlv_len;

	/* Now allocate a structure for the entire request */
	if ((req_xtlv = (uint8 *)MALLOCZ(dhd->osh, total_len)) == NULL) {
		rc = BCME_NOMEM;
		goto fail;
	}

	/* container XTLV context */
	bcm_xtlv_buf_init(&container_xtlvbuf, (uint8 *)req_xtlv, total_len,
		BCM_XTLV_OPTION_ALIGN32);

	/* Fill other XTLVs in the container. Leave space for XTLV headers */
	req = (ecounters_stats_types_report_req_t *)(req_xtlv + BCM_XTLV_HDR_SIZE);
	req->flags = type;
	if (type == ECOUNTERS_STATS_TYPES_FLAG_SLICE) {
		req->slice_mask = 0x1 << if_slice_idx;
	} else if (type == ECOUNTERS_STATS_TYPES_FLAG_IFACE) {
		req->if_index = if_slice_idx;
	}

	/* Fill remaining XTLVs */
	bcm_xtlv_buf_init(&xtlvbuf, (uint8*) req->stats_types_req, xtlv_len,
		BCM_XTLV_OPTION_ALIGN32);
	if (bcm_xtlv_put_data(&xtlvbuf, temp.id, NULL, temp.len)) {
		DHD_ERROR(("Error creating XTLV for requested stats type = %d\n", temp.id));
		rc = BCME_ERROR;
		goto fail;
	}

	/* fill the top level container and get done with the XTLV container */
	rc = bcm_xtlv_put_data(&container_xtlvbuf, WL_ECOUNTERS_XTLV_REPORT_REQ, NULL,
		bcm_xtlv_buf_len(&xtlvbuf) + OFFSETOF(ecounters_stats_types_report_req_t,
		stats_types_req));

	if (rc) {
		DHD_ERROR(("Error creating parent XTLV for type = %d\n", req->flags));
		goto fail;
	}

fail:
	if (rc && req_xtlv) {
		MFREE(dhd->osh, req_xtlv, total_len);
		req_xtlv = NULL;
	}

	/* update the xtlv pointer */
	*xtlv = req_xtlv;
	return rc;
}

static int
dhd_ecounter_autoconfig(dhd_pub_t *dhd)
{
	int rc = BCME_OK;
	uint32 buf;
	rc = dhd_iovar(dhd, 0, "ecounters_autoconfig", NULL, 0, (char *)&buf, sizeof(buf), FALSE);

	if (rc != BCME_OK) {

		if (rc != BCME_UNSUPPORTED) {
			rc = BCME_OK;
			DHD_ERROR(("%s Ecounter autoconfig in fw failed : %d\n", __FUNCTION__, rc));
		} else {
			DHD_ERROR(("%s Ecounter autoconfig in FW not supported(ignore)\n", __FUNCTION__));
		}
	}

	return rc;
}

int
dhd_ecounter_configure(dhd_pub_t *dhd, bool enable)
{
	int rc = BCME_OK;
	if (enable) {
		if (dhd_ecounter_autoconfig(dhd) != BCME_OK) {
			if ((rc = dhd_start_ecounters(dhd)) != BCME_OK) {
				DHD_ERROR(("%s Ecounters start failed\n", __FUNCTION__));
			} else if ((rc = dhd_start_event_ecounters(dhd)) != BCME_OK) {
				DHD_ERROR(("%s Event_Ecounters start failed\n", __FUNCTION__));
			}
		}
	} else {
		if ((rc = dhd_stop_ecounters(dhd)) != BCME_OK) {
			DHD_ERROR(("%s Ecounters stop failed\n", __FUNCTION__));
		} else if ((rc = dhd_stop_event_ecounters(dhd)) != BCME_OK) {
			DHD_ERROR(("%s Event_Ecounters stop failed\n", __FUNCTION__));
		}
	}
	return rc;
}

int
dhd_start_ecounters(dhd_pub_t *dhd)
{
	uint8 i = 0;
	uint8 *start_ptr;
	int rc = BCME_OK;
	bcm_xtlv_t *elt;
	ecounters_config_request_v2_t *req = NULL;
	ecountersv2_processed_xtlv_list_elt *list_elt, *tail = NULL;
	ecountersv2_processed_xtlv_list_elt *processed_containers_list = NULL;
	uint16 total_processed_containers_len = 0;

	for (i = 0; i < ARRAYSIZE(ecounters_cfg_tbl); i++) {
		ecounters_cfg_t *ecounter_stat = &ecounters_cfg_tbl[i];

		if ((list_elt = (ecountersv2_processed_xtlv_list_elt *)
			MALLOCZ(dhd->osh, sizeof(*list_elt))) == NULL) {
			DHD_ERROR(("Ecounters v2: No memory to process\n"));
			goto fail;
		}

		rc = dhd_create_ecounters_params(dhd, ecounter_stat->type,
			ecounter_stat->if_slice_idx, ecounter_stat->stats_rep, &list_elt->data);

		if (rc) {
			DHD_ERROR(("Ecounters v2: Could not process: stat: %d return code: %d\n",
				ecounter_stat->stats_rep, rc));

			/* Free allocated memory and go to fail to release any memories allocated
			 * in previous iterations. Note that list_elt->data gets populated in
			 * dhd_create_ecounters_params() and gets freed there itself.
			 */
			MFREE(dhd->osh, list_elt, sizeof(*list_elt));
			list_elt = NULL;
			goto fail;
		}
		elt = (bcm_xtlv_t *) list_elt->data;

		/* Put the elements in the order they are processed */
		if (processed_containers_list == NULL) {
			processed_containers_list = list_elt;
		} else {
			tail->next = list_elt;
		}
		tail = list_elt;
		/* Size of the XTLV returned */
		total_processed_containers_len += BCM_XTLV_LEN(elt) + BCM_XTLV_HDR_SIZE;
	}

	/* Now create ecounters config request with totallength */
	req = (ecounters_config_request_v2_t *)MALLOCZ(dhd->osh, sizeof(*req) +
		total_processed_containers_len);

	if (req == NULL) {
		rc = BCME_NOMEM;
		goto fail;
	}

	req->version = ECOUNTERS_VERSION_2;
	req->logset = EVENT_LOG_SET_ECOUNTERS;
	req->reporting_period = ECOUNTERS_DEFAULT_PERIOD;
	req->num_reports = ECOUNTERS_NUM_REPORTS;
	req->len = total_processed_containers_len +
		OFFSETOF(ecounters_config_request_v2_t, ecounters_xtlvs);

	/* Copy config */
	start_ptr = req->ecounters_xtlvs;

	/* Now go element by element in the list */
	while (processed_containers_list) {
		list_elt = processed_containers_list;

		elt = (bcm_xtlv_t *)list_elt->data;

		memcpy(start_ptr, list_elt->data, BCM_XTLV_LEN(elt) + BCM_XTLV_HDR_SIZE);
		start_ptr += (size_t)(BCM_XTLV_LEN(elt) + BCM_XTLV_HDR_SIZE);
		processed_containers_list = processed_containers_list->next;

		/* Free allocated memories */
		MFREE(dhd->osh, elt, elt->len + BCM_XTLV_HDR_SIZE);
		MFREE(dhd->osh, list_elt, sizeof(*list_elt));
	}

	if ((rc = dhd_iovar(dhd, 0, "ecounters", (char *)req, req->len, NULL, 0, TRUE)) < 0) {
		DHD_ERROR(("failed to start ecounters, rc=%d\n", rc));
	}

fail:
	if (req) {
		MFREE(dhd->osh, req, sizeof(*req) + total_processed_containers_len);
	}

	/* Now go element by element in the list */
	while (processed_containers_list) {
		list_elt = processed_containers_list;
		elt = (bcm_xtlv_t *)list_elt->data;
		processed_containers_list = processed_containers_list->next;

		/* Free allocated memories */
		MFREE(dhd->osh, elt, elt->len + BCM_XTLV_HDR_SIZE);
		MFREE(dhd->osh, list_elt, sizeof(*list_elt));
	}
	return rc;
}

int
dhd_stop_ecounters(dhd_pub_t *dhd)
{
	int rc = BCME_OK;
	ecounters_config_request_v2_t *req;

	/* Now create ecounters config request with totallength */
	req = (ecounters_config_request_v2_t *)MALLOCZ(dhd->osh, sizeof(*req));

	if (req == NULL) {
		rc = BCME_NOMEM;
		goto fail;
	}

	req->version = ECOUNTERS_VERSION_2;
	req->len = OFFSETOF(ecounters_config_request_v2_t, ecounters_xtlvs);

	if ((rc = dhd_iovar(dhd, 0, "ecounters", (char *)req, req->len, NULL, 0, TRUE)) < 0) {
		DHD_ERROR(("failed to stop ecounters\n"));
	}

fail:
	if (req) {
		MFREE(dhd->osh, req, sizeof(*req));
	}
	return rc;
}

/* configured event_id_array for event ecounters */
typedef struct event_id_array {
	uint8	event_id;
	uint8	str_idx;
} event_id_array_t;

/* get event id array only from event_ecounters_cfg_tbl[] */
static inline int __dhd_event_ecounters_get_event_id_array(event_id_array_t *event_array)
{
	uint8 i;
	uint8 idx = 0;
	int32 prev_evt_id = -1;

	for (i = 0; i < (uint8)ARRAYSIZE(event_ecounters_cfg_tbl); i++) {
		if (prev_evt_id != event_ecounters_cfg_tbl[i].event_id) {
			if (prev_evt_id >= 0)
				idx++;
			event_array[idx].event_id = event_ecounters_cfg_tbl[i].event_id;
			event_array[idx].str_idx = i;
		}
		prev_evt_id = event_ecounters_cfg_tbl[i].event_id;
	}
	return idx;
}

/* One event id has limit xtlv num to request based on wl_ifstats_xtlv_id * 2 interface */
#define ECNTRS_MAX_XTLV_NUM (31 * 2)

int
dhd_start_event_ecounters(dhd_pub_t *dhd)
{
	uint8 i, j = 0;
	uint8 event_id_cnt = 0;
	uint16 processed_containers_len = 0;
	uint16 max_xtlv_len = 0;
	int rc = BCME_OK;
	uint8 *ptr;
	uint8 *data;
	event_id_array_t *id_array;
	bcm_xtlv_t *elt = NULL;
	event_ecounters_config_request_v2_t *req = NULL;

	/* XXX: the size of id_array is limited by the size of event_ecounters_cfg_tbl */
	id_array = (event_id_array_t *)MALLOCZ(dhd->osh, sizeof(event_id_array_t) *
		ARRAYSIZE(event_ecounters_cfg_tbl));

	if (id_array == NULL) {
		rc = BCME_NOMEM;
		goto fail;
	}
	event_id_cnt = __dhd_event_ecounters_get_event_id_array(id_array);

	max_xtlv_len = ((BCM_XTLV_HDR_SIZE +
		OFFSETOF(event_ecounters_config_request_v2_t, ecounters_xtlvs)) *
		ECNTRS_MAX_XTLV_NUM);

	/* Now create ecounters config request with max allowed length */
	req = (event_ecounters_config_request_v2_t *)MALLOCZ(dhd->osh,
		sizeof(event_ecounters_config_request_v2_t *) + max_xtlv_len);

	if (req == NULL) {
		rc = BCME_NOMEM;
		goto fail;
	}

	for (i = 0; i <= event_id_cnt; i++) {
		/* req initialization by event id */
		req->version = ECOUNTERS_VERSION_2;
		req->logset = EVENT_LOG_SET_ECOUNTERS;
		req->event_id = id_array[i].event_id;
		req->flags = EVENT_ECOUNTERS_FLAGS_ADD;
		req->len = 0;
		processed_containers_len = 0;

		/* Copy config */
		ptr = req->ecounters_xtlvs;

		for (j = id_array[i].str_idx; j < (uint8)ARRAYSIZE(event_ecounters_cfg_tbl); j++) {
			event_ecounters_cfg_t *event_ecounter_stat = &event_ecounters_cfg_tbl[j];
			if (id_array[i].event_id != event_ecounter_stat->event_id)
				break;

			rc = dhd_create_ecounters_params(dhd, event_ecounter_stat->type,
				event_ecounter_stat->if_slice_idx, event_ecounter_stat->stats_rep,
				&data);

			if (rc) {
				DHD_ERROR(("%s: Could not process: stat: %d return code: %d\n",
					__FUNCTION__, event_ecounter_stat->stats_rep, rc));
				goto fail;
			}

			elt = (bcm_xtlv_t *)data;

			memcpy(ptr, elt, BCM_XTLV_LEN(elt) + BCM_XTLV_HDR_SIZE);
			ptr += (size_t)(BCM_XTLV_LEN(elt) + BCM_XTLV_HDR_SIZE);
			processed_containers_len += BCM_XTLV_LEN(elt) + BCM_XTLV_HDR_SIZE;

			/* Free allocated memories alloced by dhd_create_ecounters_params */
			MFREE(dhd->osh, elt, elt->len + BCM_XTLV_HDR_SIZE);

			if (processed_containers_len > max_xtlv_len) {
				DHD_ERROR(("%s XTLV NUM IS OVERFLOWED THAN ALLOWED!!\n",
					__FUNCTION__));
				rc = BCME_BADLEN;
				goto fail;
			}
		}

		req->len = processed_containers_len +
			OFFSETOF(event_ecounters_config_request_v2_t, ecounters_xtlvs);

		DHD_INFO(("%s req version %d logset %d event_id %d flags %d len %d\n",
			__FUNCTION__, req->version, req->logset, req->event_id,
			req->flags, req->len));

		rc = dhd_iovar(dhd, 0, "event_ecounters", (char *)req, req->len, NULL, 0, TRUE);

		if (rc < 0) {
			DHD_ERROR(("failed to start event_ecounters(event id %d) with rc %d\n",
				req->event_id, rc));
			goto fail;
		}
	}

fail:
	/* Free allocated memories */
	if (req) {
		MFREE(dhd->osh, req, sizeof(event_ecounters_config_request_v2_t *) + max_xtlv_len);
	}
	if (id_array) {
		MFREE(dhd->osh, id_array, sizeof(event_id_array_t) *
			ARRAYSIZE(event_ecounters_cfg_tbl));
	}

	return rc;
}

int
dhd_stop_event_ecounters(dhd_pub_t *dhd)
{
	int rc = BCME_OK;
	event_ecounters_config_request_v2_t *req;

	/* Now create ecounters config request with totallength */
	req = (event_ecounters_config_request_v2_t *)MALLOCZ(dhd->osh, sizeof(*req));

	if (req == NULL) {
		rc = BCME_NOMEM;
		goto fail;
	}

	req->version = ECOUNTERS_VERSION_2;
	req->flags = EVENT_ECOUNTERS_FLAGS_DEL_ALL;
	req->len = OFFSETOF(event_ecounters_config_request_v2_t, ecounters_xtlvs);

	if ((rc = dhd_iovar(dhd, 0, "event_ecounters", (char *)req, req->len, NULL, 0, TRUE)) < 0) {
		DHD_ERROR(("failed to stop event_ecounters\n"));
	}

fail:
	if (req) {
		MFREE(dhd->osh, req, sizeof(*req));
	}
	return rc;
}
#ifdef DHD_LOG_DUMP

#ifdef DHD_DEBUGABILITY_DEBUG_DUMP
static dhd_debug_dump_ring_entry_t dhd_debug_dump_ring_map[] = {
	{LOG_DUMP_SECTION_TIMESTAMP, DEBUG_DUMP_RING1_ID},
	{LOG_DUMP_SECTION_ECNTRS, DEBUG_DUMP_RING2_ID},
	{LOG_DUMP_SECTION_STATUS, DEBUG_DUMP_RING1_ID},
	{LOG_DUMP_SECTION_RTT, DEBUG_DUMP_RING2_ID},
	{LOG_DUMP_SECTION_DHD_DUMP, DEBUG_DUMP_RING1_ID},
	{LOG_DUMP_SECTION_EXT_TRAP, DEBUG_DUMP_RING1_ID},
	{LOG_DUMP_SECTION_HEALTH_CHK, DEBUG_DUMP_RING1_ID},
	{LOG_DUMP_SECTION_COOKIE, DEBUG_DUMP_RING1_ID},
	{LOG_DUMP_SECTION_RING, DEBUG_DUMP_RING1_ID},
};

int dhd_debug_dump_get_ring_num(int sec_type)
{
	int idx = 0;
	for (idx = 0; idx < ARRAYSIZE(dhd_debug_dump_ring_map); idx++) {
		if (dhd_debug_dump_ring_map[idx].type == sec_type) {
			idx = dhd_debug_dump_ring_map[idx].debug_dump_ring;
			return idx;
		}
	}
	return idx;
}
#endif /* DHD_DEBUGABILITY_DEBUG_DUMP */

int
dhd_dump_debug_ring(dhd_pub_t *dhdp, void *ring_ptr, const void *user_buf,
		log_dump_section_hdr_t *sec_hdr,
		char *text_hdr, int buflen, uint32 sec_type)
{
	uint32 rlen = 0;
	uint32 data_len = 0;
	void *data = NULL;
	unsigned long flags = 0;
	int ret = 0;
	dhd_dbg_ring_t *ring = (dhd_dbg_ring_t *)ring_ptr;
#ifndef DHD_DEBUGABILITY_DEBUG_DUMP
	int pos = 0;
#endif /* !DHD_DEBUGABILITY_DEBUG_DUMP */
	int fpos_sechdr = 0;
	int tot_len = 0;
	char *tmp_buf = NULL;
	int idx;
	int ring_num = 0;

	BCM_REFERENCE(idx);
#ifndef DHD_DEBUGABILITY_DEBUG_DUMP
	BCM_REFERENCE(pos);
#endif /* !DHD_DEBUGABILITY_DEBUG_DUMP */
	BCM_REFERENCE(tot_len);
	BCM_REFERENCE(fpos_sechdr);
	BCM_REFERENCE(data_len);
	BCM_REFERENCE(tmp_buf);
	BCM_REFERENCE(ring_num);

#ifdef DHD_DEBUGABILITY_DEBUG_DUMP
	if (!dhdp || !ring || !sec_hdr || !text_hdr) {
		return BCME_BADARG;
	}

	tmp_buf = (char *)VMALLOCZ(dhdp->osh, ring->ring_size);
	if (!tmp_buf) {
		DHD_ERROR(("%s: VMALLOC Fail id:%d size:%u\n",
			__func__, ring->id, ring->ring_size));
		return BCME_NOMEM;
	}
#else
	if (!dhdp || !ring || !user_buf || !sec_hdr || !text_hdr) {
		return BCME_BADARG;
	}
#endif /* DHD_DEBUGABILITY_DEBUG_DUMP */
	/* do not allow further writes to the ring
	 * till we flush it
	 */
	DHD_DBG_RING_LOCK(ring->lock, flags);
	ring->state = RING_SUSPEND;
	DHD_DBG_RING_UNLOCK(ring->lock, flags);

#ifdef DHD_DEBUGABILITY_DEBUG_DUMP
	ring_num = dhd_debug_dump_get_ring_num(sec_type);
	dhd_export_debug_data(text_hdr, NULL, NULL, strlen(text_hdr), &ring_num);

	data = tmp_buf;
	do {
		rlen = dhd_dbg_ring_pull_single(ring, data, ring->ring_size, TRUE);
		if (rlen > 0) {
			tot_len += rlen;
			data += rlen;
		}
	} while ((rlen > 0));

	sec_hdr->type = sec_type;
	sec_hdr->length = tot_len;
	DHD_ERROR(("%s: DUMP id:%d type:%u tot_len:%d\n", __func__, ring->id, sec_type, tot_len));

	dhd_export_debug_data((char *)sec_hdr, NULL, NULL, sizeof(*sec_hdr), &ring_num);
	dhd_export_debug_data(tmp_buf, NULL, NULL, tot_len, &ring_num);

	VMFREE(dhdp->osh, tmp_buf, ring->ring_size);
#else
	if (dhdp->concise_dbg_buf) {
		/* re-use concise debug buffer temporarily
		 * to pull ring data, to write
		 * record by record to file
		 */
		data_len = CONCISE_DUMP_BUFLEN;
		data = dhdp->concise_dbg_buf;
		ret = dhd_export_debug_data(text_hdr, NULL, user_buf, strlen(text_hdr), &pos);
		/* write the section header now with zero length,
		 * once the correct length is found out, update
		 * it later
		 */
		fpos_sechdr = pos;
		sec_hdr->type = sec_type;
		sec_hdr->length = 0;
		ret = dhd_export_debug_data((char *)sec_hdr, NULL, user_buf,
			sizeof(*sec_hdr), &pos);
		do {
			rlen = dhd_dbg_ring_pull_single(ring, data, data_len, TRUE);
			if (rlen > 0) {
				/* write the log */
				ret = dhd_export_debug_data(data, NULL, user_buf, rlen, &pos);
			}
			DHD_DBGIF(("%s: rlen : %d\n", __FUNCTION__, rlen));
		} while ((rlen > 0));
		/* now update the section header length in the file */
		/* Complete ring size is dumped by HAL, hence updating length to ring size */
		sec_hdr->length = ring->ring_size;
		ret = dhd_export_debug_data((char *)sec_hdr, NULL, user_buf,
			sizeof(*sec_hdr), &fpos_sechdr);
	} else {
		DHD_ERROR(("%s: No concise buffer available !\n", __FUNCTION__));
	}
#endif /* DHD_DEBUGABILITY_DEBUG_DUMP */

	DHD_DBG_RING_LOCK(ring->lock, flags);
	ring->state = RING_ACTIVE;
	/* Resetting both read and write pointer,
	 * since all items are read.
	 */
	ring->rp = ring->wp = 0;
	DHD_DBG_RING_UNLOCK(ring->lock, flags);

	return ret;
}

int
dhd_log_dump_ring_to_file(dhd_pub_t *dhdp, void *ring_ptr, void *file,
		unsigned long *file_posn, log_dump_section_hdr_t *sec_hdr,
		char *text_hdr, uint32 sec_type)
{
	uint32 rlen = 0;
	uint32 data_len = 0, total_len = 0;
	void *data = NULL;
	unsigned long fpos_sechdr = 0;
	unsigned long flags = 0;
	int ret = 0;
	dhd_dbg_ring_t *ring = (dhd_dbg_ring_t *)ring_ptr;

	if (!dhdp || !ring || !file || !sec_hdr ||
		!file_posn || !text_hdr)
		return BCME_BADARG;

	/* do not allow further writes to the ring
	 * till we flush it
	 */
	DHD_DBG_RING_LOCK(ring->lock, flags);
	ring->state = RING_SUSPEND;
	DHD_DBG_RING_UNLOCK(ring->lock, flags);

	if (dhdp->concise_dbg_buf) {
		/* re-use concise debug buffer temporarily
		 * to pull ring data, to write
		 * record by record to file
		 */
		data_len = CONCISE_DUMP_BUFLEN;
		data = dhdp->concise_dbg_buf;
		dhd_os_write_file_posn(file, file_posn, text_hdr,
				strlen(text_hdr));
		/* write the section header now with zero length,
		 * once the correct length is found out, update
		 * it later
		 */
		dhd_init_sec_hdr(sec_hdr);
		fpos_sechdr = *file_posn;
		sec_hdr->type = sec_type;
		sec_hdr->length = 0;
		dhd_os_write_file_posn(file, file_posn, (char *)sec_hdr,
				sizeof(*sec_hdr));
		do {
			rlen = dhd_dbg_ring_pull_single(ring, data, data_len, TRUE);
			if (rlen > 0) {
				/* write the log */
				ret = dhd_os_write_file_posn(file, file_posn, data, rlen);
				if (ret < 0) {
					DHD_ERROR(("%s: write file error !\n", __FUNCTION__));
					DHD_DBG_RING_LOCK(ring->lock, flags);
					ring->state = RING_ACTIVE;
					DHD_DBG_RING_UNLOCK(ring->lock, flags);
					return BCME_ERROR;
				}
			}
			total_len += rlen;
		} while (rlen > 0);
		/* now update the section header length in the file */
		sec_hdr->length = total_len;
		dhd_os_write_file_posn(file, &fpos_sechdr, (char *)sec_hdr, sizeof(*sec_hdr));
	} else {
		DHD_ERROR(("%s: No concise buffer available !\n", __FUNCTION__));
	}

	DHD_DBG_RING_LOCK(ring->lock, flags);
	ring->state = RING_ACTIVE;
	/* Resetting both read and write pointer,
	 * since all items are read.
	 */
	ring->rp = ring->wp = 0;
	DHD_DBG_RING_UNLOCK(ring->lock, flags);
	return BCME_OK;
}

/* logdump cookie */
#define MAX_LOGUDMP_COOKIE_CNT	10u
#define LOGDUMP_COOKIE_STR_LEN	50u
int
dhd_logdump_cookie_init(dhd_pub_t *dhdp, uint8 *buf, uint32 buf_size)
{
	uint32 ring_size;

	if (!dhdp || !buf) {
		DHD_ERROR(("INVALID PTR: dhdp:%p buf:%p\n", dhdp, buf));
		return BCME_ERROR;
	}

	ring_size = dhd_ring_get_hdr_size() + LOGDUMP_COOKIE_STR_LEN * MAX_LOGUDMP_COOKIE_CNT;
	if (buf_size < ring_size) {
		DHD_ERROR(("BUF SIZE IS TO SHORT: req:%d buf_size:%d\n",
			ring_size, buf_size));
		return BCME_ERROR;
	}

	dhdp->logdump_cookie = dhd_ring_init(dhdp, buf, buf_size,
		LOGDUMP_COOKIE_STR_LEN, MAX_LOGUDMP_COOKIE_CNT,
		DHD_RING_TYPE_FIXED);
	if (!dhdp->logdump_cookie) {
		DHD_ERROR(("FAIL TO INIT COOKIE RING\n"));
		return BCME_ERROR;
	}

	return BCME_OK;
}

void
dhd_logdump_cookie_deinit(dhd_pub_t *dhdp)
{
	if (!dhdp) {
		return;
	}
	if (dhdp->logdump_cookie) {
		dhd_ring_deinit(dhdp, dhdp->logdump_cookie);
	}

	return;
}

#ifdef DHD_TX_PROFILE
int
dhd_tx_profile_detach(dhd_pub_t *dhdp)
{
	int result = BCME_ERROR;

	if (dhdp != NULL && dhdp->protocol_filters != NULL) {
		MFREE(dhdp->osh, dhdp->protocol_filters, DHD_MAX_PROFILES *
				sizeof(*(dhdp->protocol_filters)));
		dhdp->protocol_filters = NULL;

		result = BCME_OK;
	}

	return result;
}

int
dhd_tx_profile_attach(dhd_pub_t *dhdp)
{
	int result = BCME_ERROR;

	if (dhdp != NULL) {
		dhdp->protocol_filters = (dhd_tx_profile_protocol_t*)MALLOCZ(dhdp->osh,
				DHD_MAX_PROFILES * sizeof(*(dhdp->protocol_filters)));

		if (dhdp->protocol_filters != NULL) {
			result = BCME_OK;
		}
	}

	if (result != BCME_OK) {
		DHD_ERROR(("%s:\tMALLOC of tx profile protocol filters failed\n",
			__FUNCTION__));
	}

	return result;
}
#endif /* defined(DHD_TX_PROFILE) */

void
dhd_logdump_cookie_save(dhd_pub_t *dhdp, char *cookie, char *type)
{
	char *ptr;

	if (!dhdp || !cookie || !type || !dhdp->logdump_cookie) {
		DHD_ERROR(("%s: At least one buffer ptr is NULL dhdp=%p cookie=%p"
			" type = %p, cookie_cfg:%p\n", __FUNCTION__,
			dhdp, cookie, type, dhdp?dhdp->logdump_cookie: NULL));
		return;
	}
	ptr = (char *)dhd_ring_get_empty(dhdp->logdump_cookie);
	if (ptr == NULL) {
		DHD_ERROR(("%s : Skip to save due to locking\n", __FUNCTION__));
		return;
	}
	scnprintf(ptr, LOGDUMP_COOKIE_STR_LEN, "%s: %s\n", type, cookie);
	return;
}

int
dhd_logdump_cookie_get(dhd_pub_t *dhdp, char *ret_cookie, uint32 buf_size)
{
	char *ptr;

	if (!dhdp || !ret_cookie || !dhdp->logdump_cookie) {
		DHD_ERROR(("%s: At least one buffer ptr is NULL dhdp=%p"
			"cookie=%p cookie_cfg:%p\n", __FUNCTION__,
			dhdp, ret_cookie, dhdp?dhdp->logdump_cookie: NULL));
		return BCME_ERROR;
	}
	ptr = (char *)dhd_ring_get_first(dhdp->logdump_cookie);
	if (ptr == NULL) {
		DHD_ERROR(("%s : Skip to save due to locking\n", __FUNCTION__));
		return BCME_ERROR;
	}
	memcpy(ret_cookie, ptr, MIN(buf_size, strlen(ptr)));
	dhd_ring_free_first(dhdp->logdump_cookie);
	return BCME_OK;
}

int
dhd_logdump_cookie_count(dhd_pub_t *dhdp)
{
	if (!dhdp || !dhdp->logdump_cookie) {
		DHD_ERROR(("%s: At least one buffer ptr is NULL dhdp=%p cookie=%p\n",
			__FUNCTION__, dhdp, dhdp?dhdp->logdump_cookie: NULL));
		return 0;
	}
	return dhd_ring_get_cur_size(dhdp->logdump_cookie);
}

static inline int
__dhd_log_dump_cookie_to_file(
	dhd_pub_t *dhdp, void *fp, const void *user_buf, unsigned long *f_pos,
	char *buf, uint32 buf_size)
{

	uint32 remain = buf_size;
	int ret = BCME_ERROR;
	char tmp_buf[LOGDUMP_COOKIE_STR_LEN];
	log_dump_section_hdr_t sec_hdr;
	uint32 read_idx;
	uint32 write_idx;

	read_idx = dhd_ring_get_read_idx(dhdp->logdump_cookie);
	write_idx = dhd_ring_get_write_idx(dhdp->logdump_cookie);
	while (dhd_logdump_cookie_count(dhdp) > 0) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		ret = dhd_logdump_cookie_get(dhdp, tmp_buf, LOGDUMP_COOKIE_STR_LEN);
		if (ret != BCME_OK) {
			return ret;
		}
		remain -= scnprintf(&buf[buf_size - remain], remain, "%s", tmp_buf);
	}
	dhd_ring_set_read_idx(dhdp->logdump_cookie, read_idx);
	dhd_ring_set_write_idx(dhdp->logdump_cookie, write_idx);

	ret = dhd_export_debug_data(COOKIE_LOG_HDR, fp, user_buf, strlen(COOKIE_LOG_HDR), f_pos);
	if (ret < 0) {
		DHD_ERROR(("%s : Write file Error for cookie hdr\n", __FUNCTION__));
		return ret;
	}
	sec_hdr.magic = LOG_DUMP_MAGIC;
	sec_hdr.timestamp = local_clock();
	sec_hdr.type = LOG_DUMP_SECTION_COOKIE;
	sec_hdr.length = buf_size - remain;

	ret = dhd_export_debug_data((char *)&sec_hdr, fp, user_buf, sizeof(sec_hdr), f_pos);
	if (ret < 0) {
		DHD_ERROR(("%s : Write file Error for section hdr\n", __FUNCTION__));
		return ret;
	}

	ret = dhd_export_debug_data(buf, fp, user_buf, sec_hdr.length, f_pos);
	if (ret < 0) {
		DHD_ERROR(("%s : Write file Error for cookie data\n", __FUNCTION__));
	}

	return ret;
}

uint32
dhd_log_dump_cookie_len(dhd_pub_t *dhdp)
{
	int len = 0;
	char tmp_buf[LOGDUMP_COOKIE_STR_LEN];
	log_dump_section_hdr_t sec_hdr;
	char *buf = NULL;
	int ret = BCME_ERROR;
	uint32 buf_size = MAX_LOGUDMP_COOKIE_CNT * LOGDUMP_COOKIE_STR_LEN;
	uint32 read_idx;
	uint32 write_idx;
	uint32 remain;

	remain = buf_size;

	if (!dhdp || !dhdp->logdump_cookie) {
		DHD_ERROR(("%s At least one ptr is NULL "
			"dhdp = %p cookie %p\n",
			__FUNCTION__, dhdp, dhdp?dhdp->logdump_cookie:NULL));
		goto exit;
	}

	buf = (char *)MALLOCZ(dhdp->osh, buf_size);
	if (!buf) {
		DHD_ERROR(("%s Fail to malloc buffer\n", __FUNCTION__));
		goto exit;
	}

	read_idx = dhd_ring_get_read_idx(dhdp->logdump_cookie);
	write_idx = dhd_ring_get_write_idx(dhdp->logdump_cookie);
	while (dhd_logdump_cookie_count(dhdp) > 0) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		ret = dhd_logdump_cookie_get(dhdp, tmp_buf, LOGDUMP_COOKIE_STR_LEN);
		if (ret != BCME_OK) {
			goto exit;
		}
		remain -= (uint32)strlen(tmp_buf);
	}
	dhd_ring_set_read_idx(dhdp->logdump_cookie, read_idx);
	dhd_ring_set_write_idx(dhdp->logdump_cookie, write_idx);
	len += strlen(COOKIE_LOG_HDR);
	len += sizeof(sec_hdr);
	len += (buf_size - remain);
exit:
	if (buf)
		MFREE(dhdp->osh, buf, buf_size);
	return len;
}

int
dhd_log_dump_cookie(dhd_pub_t *dhdp, const void *user_buf)
{
	int ret = BCME_ERROR;
	char tmp_buf[LOGDUMP_COOKIE_STR_LEN];
	log_dump_section_hdr_t sec_hdr;
	char *buf = NULL;
	uint32 buf_size = MAX_LOGUDMP_COOKIE_CNT * LOGDUMP_COOKIE_STR_LEN;
	int pos = 0;
	uint32 read_idx;
	uint32 write_idx;
	uint32 remain;

	remain = buf_size;

	if (!dhdp || !dhdp->logdump_cookie) {
		DHD_ERROR(("%s At least one ptr is NULL "
			"dhdp = %p cookie %p\n",
			__FUNCTION__, dhdp, dhdp?dhdp->logdump_cookie:NULL));
		goto exit;
	}

	buf = (char *)MALLOCZ(dhdp->osh, buf_size);
	if (!buf) {
		DHD_ERROR(("%s Fail to malloc buffer\n", __FUNCTION__));
		goto exit;
	}

	read_idx = dhd_ring_get_read_idx(dhdp->logdump_cookie);
	write_idx = dhd_ring_get_write_idx(dhdp->logdump_cookie);
	while (dhd_logdump_cookie_count(dhdp) > 0) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		ret = dhd_logdump_cookie_get(dhdp, tmp_buf, LOGDUMP_COOKIE_STR_LEN);
		if (ret != BCME_OK) {
			goto exit;
		}
		remain -= scnprintf(&buf[buf_size - remain], remain, "%s", tmp_buf);
	}
	dhd_ring_set_read_idx(dhdp->logdump_cookie, read_idx);
	dhd_ring_set_write_idx(dhdp->logdump_cookie, write_idx);
	ret = dhd_export_debug_data(COOKIE_LOG_HDR, NULL, user_buf, strlen(COOKIE_LOG_HDR), &pos);
	sec_hdr.magic = LOG_DUMP_MAGIC;
	sec_hdr.timestamp = local_clock();
	sec_hdr.type = LOG_DUMP_SECTION_COOKIE;
	sec_hdr.length = buf_size - remain;
	ret = dhd_export_debug_data((char *)&sec_hdr, NULL, user_buf, sizeof(sec_hdr), &pos);
	ret = dhd_export_debug_data(buf, NULL, user_buf, sec_hdr.length, &pos);
exit:
	if (buf)
		MFREE(dhdp->osh, buf, buf_size);
	return ret;
}

int
dhd_log_dump_cookie_to_file(dhd_pub_t *dhdp, void *fp, const void *user_buf, unsigned long *f_pos)
{
	char *buf;
	int ret = BCME_ERROR;
	uint32 buf_size = MAX_LOGUDMP_COOKIE_CNT * LOGDUMP_COOKIE_STR_LEN;

#ifdef DHD_DEBUGABILITY_DEBUG_DUMP
	if (!dhdp || !dhdp->logdump_cookie) {
#else
	if (!dhdp || !dhdp->logdump_cookie || (!fp && !user_buf) || !f_pos) {
#endif /* DHD_DEBUGABILITY_DEBUG_DUMP */
		DHD_ERROR(("%s At least one ptr is NULL "
			"dhdp = %p cookie %p fp = %p f_pos = %p\n",
			__FUNCTION__, dhdp, dhdp?dhdp->logdump_cookie:NULL, fp, f_pos));
		return ret;
	}

	buf = (char *)MALLOCZ(dhdp->osh, buf_size);
	if (!buf) {
		DHD_ERROR(("%s Fail to malloc buffer\n", __FUNCTION__));
		return ret;
	}
	ret = __dhd_log_dump_cookie_to_file(dhdp, fp, user_buf, f_pos, buf, buf_size);
	MFREE(dhdp->osh, buf, buf_size);

	return ret;
}
#endif /* DHD_LOG_DUMP */

#ifdef CONFIG_ROAM_RSSI_LIMIT
int
dhd_roam_rssi_limit_get(dhd_pub_t *dhd, int *lmt2g, int *lmt5g)
{
	wlc_roam_rssi_limit_t *plmt;
	wlc_roam_rssi_lmt_info_v1_t *pinfo;
	int ret = BCME_OK;
	int plmt_len = sizeof(*pinfo) + ROAMRSSI_HDRLEN;

	plmt = (wlc_roam_rssi_limit_t *)MALLOCZ(dhd->osh, plmt_len);
	if (!plmt) {
		DHD_ERROR(("%s Fail to malloc buffer\n", __FUNCTION__));
		return BCME_NOMEM;
	}

	/* Get roam rssi limit */
	ret = dhd_iovar(dhd, 0, "roam_rssi_limit", NULL, 0, (char *)plmt, plmt_len, FALSE);
	if (ret < 0) {
		DHD_ERROR(("%s Failed to Get roam_rssi_limit %d\n", __FUNCTION__, ret));
		goto done;
	}

	if (plmt->ver != WLC_ROAM_RSSI_LMT_VER_1) {
	    ret = BCME_VERSION;
	    goto done;
	}

	pinfo = (wlc_roam_rssi_lmt_info_v1_t *)plmt->data;
	*lmt2g = (int)pinfo->rssi_limit_2g;
	*lmt5g = (int)pinfo->rssi_limit_5g;

done:
	if (plmt) {
		MFREE(dhd->osh, plmt, plmt_len);
	}
	return ret;
}

int
dhd_roam_rssi_limit_set(dhd_pub_t *dhd, int lmt2g, int lmt5g)
{
	wlc_roam_rssi_limit_t *plmt;
	wlc_roam_rssi_lmt_info_v1_t *pinfo;
	int ret = BCME_OK;
	int plmt_len = sizeof(*pinfo) + ROAMRSSI_HDRLEN;

	/* Sanity check RSSI limit Value */
	if ((lmt2g < ROAMRSSI_2G_MIN) || (lmt2g > ROAMRSSI_2G_MAX)) {
		DHD_ERROR(("%s Not In Range 2G ROAM RSSI Limit\n", __FUNCTION__));
		return BCME_RANGE;
	}
	if ((lmt2g < ROAMRSSI_5G_MIN) || (lmt2g > ROAMRSSI_5G_MAX)) {
		DHD_ERROR(("%s Not In Range 5G ROAM RSSI Limit\n", __FUNCTION__));
		return BCME_RANGE;
	}

	plmt = (wlc_roam_rssi_limit_t *)MALLOCZ(dhd->osh, plmt_len);
	if (!plmt) {
		DHD_ERROR(("%s Fail to malloc buffer\n", __FUNCTION__));
		return BCME_NOMEM;
	}
	plmt->ver = WLC_ROAM_RSSI_LMT_VER_1;
	plmt->len = sizeof(*pinfo);
	pinfo = (wlc_roam_rssi_lmt_info_v1_t *)plmt->data;
	pinfo->rssi_limit_2g = (int16)lmt2g;
	pinfo->rssi_limit_5g = (int16)lmt5g;

	/* Set roam rssi limit */
	ret = dhd_iovar(dhd, 0, "roam_rssi_limit", (char *)plmt, plmt_len, NULL, 0, TRUE);
	if (ret < 0) {
		DHD_ERROR(("%s Failed to Get roam_rssi_limit %d\n", __FUNCTION__, ret));
		goto done;
	}
done:
	if (plmt) {
		MFREE(dhd->osh, plmt, plmt_len);
	}
	return ret;
}
#endif /* CONFIG_ROAM_RSSI_LIMIT */

#ifdef CONFIG_ROAM_MIN_DELTA
int
dhd_roam_min_delta_get(dhd_pub_t *dhd, uint32 *dt2g, uint32 *dt5g)
{
	wlc_roam_min_delta_t *pmin_delta;
	wlc_roam_min_delta_info_v1_t *pmin_delta_info;
	int ret = BCME_OK;
	int plen = sizeof(*pmin_delta_info) + ROAM_MIN_DELTA_HDRLEN;

	pmin_delta = (wlc_roam_min_delta_t *)MALLOCZ(dhd->osh, plen);
	if (!pmin_delta) {
		DHD_ERROR(("%s Fail to malloc buffer\n", __FUNCTION__));
		return BCME_NOMEM;
	}

	/* Get Minimum ROAM score delta */
	ret = dhd_iovar(dhd, 0, "roam_min_delta", NULL, 0, (char *)pmin_delta, plen, FALSE);
	if (ret < 0) {
		DHD_ERROR(("%s Failed to Get roam_min_delta %d\n", __FUNCTION__, ret));
		goto done;
	}

	if (pmin_delta->ver != WLC_ROAM_MIN_DELTA_VER_1) {
		ret = BCME_VERSION;
		goto done;
	}

	pmin_delta_info = (wlc_roam_min_delta_info_v1_t *)pmin_delta->data;
	*dt2g = (uint32)pmin_delta_info->roam_min_delta_2g;
	*dt5g = (uint32)pmin_delta_info->roam_min_delta_5g;

done:
	if (pmin_delta) {
		MFREE(dhd->osh, pmin_delta, plen);
	}
	return ret;
}

int
dhd_roam_min_delta_set(dhd_pub_t *dhd, uint32 dt2g, uint32 dt5g)
{
	wlc_roam_min_delta_t *pmin_delta;
	wlc_roam_min_delta_info_v1_t *pmin_delta_info;
	int ret = BCME_OK;
	int plen = sizeof(*pmin_delta_info) + ROAM_MIN_DELTA_HDRLEN;

	/* Sanity check Minimum ROAM score delta */
	if ((dt2g > ROAM_MIN_DELTA_MAX) || (dt5g > ROAM_MIN_DELTA_MAX)) {
		DHD_ERROR(("%s Not In Range Minimum ROAM score delta, 2G: %d, 5G: %d\n",
			__FUNCTION__, dt2g, dt5g));
		return BCME_RANGE;
	}

	pmin_delta = (wlc_roam_min_delta_t *)MALLOCZ(dhd->osh, plen);
	if (!pmin_delta) {
		DHD_ERROR(("%s Fail to malloc buffer\n", __FUNCTION__));
		return BCME_NOMEM;
	}
	pmin_delta->ver = WLC_ROAM_MIN_DELTA_VER_1;
	pmin_delta->len = sizeof(*pmin_delta_info);
	pmin_delta_info = (wlc_roam_min_delta_info_v1_t *)pmin_delta->data;
	pmin_delta_info->roam_min_delta_2g = (uint32)dt2g;
	pmin_delta_info->roam_min_delta_5g = (uint32)dt5g;

	/* Set Minimum ROAM score delta */
	ret = dhd_iovar(dhd, 0, "roam_min_delta", (char *)pmin_delta, plen, NULL, 0, TRUE);
	if (ret < 0) {
		DHD_ERROR(("%s Failed to Set roam_min_delta %d\n", __FUNCTION__, ret));
		goto done;
	}
done:
	if (pmin_delta) {
		MFREE(dhd->osh, pmin_delta, plen);
	}
	return ret;
}
#endif /* CONFIG_ROAM_MIN_DELTA */

#ifdef HOST_SFH_LLC
#define SSTLOOKUP(proto) (((proto) == 0x80f3) || ((proto) == 0x8137))
/** Convert Ethernet to 802.3 per 802.1H (use bridge-tunnel if type in SST)
 * Note:- This function will overwrite the ethernet header in the pkt
 * with a 802.3 ethernet + LLC/SNAP header by utilising the headroom
 * in the packet. The pkt data pointer should be pointing to the
 * start of the packet (at the ethernet header) when the function is called.
 * The pkt data pointer will be pointing to the
 * start of the new 802.3 header if the function returns successfully
 *
 *
 * Original Ethernet (header length = 14):
 * ----------------------------------------------------------------------------------------
 * |                                                     |   DA   |   SA   | T |  Data... |
 * ----------------------------------------------------------------------------------------
 *                                                            6        6     2
 *
 * Conversion to 802.3 (header length = 22):
 *                     (LLC includes ether_type in last 2 bytes):
 * ----------------------------------------------------------------------------------------
 * |                                      |   DA   |   SA   | L | LLC/SNAP | T |  Data... |
 * ----------------------------------------------------------------------------------------
 *                                             6        6     2       6      2
 */
int
BCMFASTPATH(dhd_ether_to_8023_hdr)(osl_t *osh, struct ether_header *eh, void *p)
{
	struct ether_header *neh;
	struct dot11_llc_snap_header *lsh;
	uint16 plen, ether_type;

	if (PKTHEADROOM(osh, p) < DOT11_LLC_SNAP_HDR_LEN) {
		DHD_ERROR(("%s: FATAL! not enough pkt headroom !\n", __FUNCTION__));
		ASSERT(0);
		return BCME_BUFTOOSHORT;
	}

	ether_type = ntoh16(eh->ether_type);
	neh = (struct ether_header *)PKTPUSH(osh, p, DOT11_LLC_SNAP_HDR_LEN);

	/* 802.3 MAC header */
	eacopy((char*)eh->ether_dhost, (char*)neh->ether_dhost);
	eacopy((char*)eh->ether_shost, (char*)neh->ether_shost);
	plen = (uint16)PKTLEN(osh, p) - ETHER_HDR_LEN;
	neh->ether_type = hton16(plen);

	/* 802.2 LLC header */
	lsh = (struct dot11_llc_snap_header *)&neh[1];
	lsh->dsap = 0xaa;
	lsh->ssap = 0xaa;
	lsh->ctl = 0x03;

	/* 802.2 SNAP header Use RFC1042 or bridge-tunnel if type in SST per 802.1H */
	lsh->oui[0] = 0x00;
	lsh->oui[1] = 0x00;
	if (SSTLOOKUP(ether_type))
		lsh->oui[2] = 0xf8;
	else
		lsh->oui[2] = 0x00;
	lsh->type = hton16(ether_type);

	return BCME_OK;
}

/** Convert 802.3+LLC to ethernet
 * Note:- This function will overwrite the 802.3+LLC hdr in the pkt
 * with an ethernet header. The pkt data pointer should be pointing to the
 * start of the packet (at the 802.3 header) when the function is called.
 * The pkt data pointer will be pointing to the
 * start of the ethernet header if the function returns successfully
 */
int
BCMFASTPATH(dhd_8023_llc_to_ether_hdr)(osl_t *osh, struct ether_header *eh8023, void *p)
{
	struct dot11_llc_snap_header *lsh = NULL;
	uint16 ether_type = 0;
	uint8 *pdata = NULL;

	if (!p || !eh8023)
		return BCME_BADARG;

	pdata = PKTDATA(osh, p);
	ether_type = ntoh16(eh8023->ether_type);
	/* ether type in 802.3 hdr for sfh llc host insertion case
	 * contains length, replace it with actual ether type at the
	 * end of the LLC hdr
	 */
	if (ether_type < ETHER_TYPE_MIN) {
		/* 802.2 LLC header */
		lsh = (struct dot11_llc_snap_header *)(pdata + sizeof(*eh8023));
		eh8023->ether_type = lsh->type;
		pdata = PKTPULL(osh, p, DOT11_LLC_SNAP_HDR_LEN);
		memcpy_s(pdata, sizeof(*eh8023), eh8023, sizeof(*eh8023));
	 } else {
		DHD_ERROR_RLMT(("ethertype 0x%x is not a length !\n", ether_type));
		return BCME_BADARG;
	 }

	return BCME_OK;
}
#endif /* HOST_SFH_LLC */

int
dhd_iovar(dhd_pub_t *pub, int ifidx, char *name, char *param_buf, uint param_len, char *res_buf,
		uint res_len, bool set)
{
	char *buf = NULL;
	uint input_len;
	wl_ioctl_t ioc;
	int ret;

	if (res_len > WLC_IOCTL_MAXLEN || param_len > WLC_IOCTL_MAXLEN)
		return BCME_BADARG;

	input_len = strlen(name) + 1 + param_len;

	/* WAR to fix GET iovar returning buf too short error
	 * If param len is 0 for get iovar, increment input_len by sizeof(int)
	 * to avoid the length check error in fw
	 */
	if (!set && !param_len) {
		input_len += sizeof(int);
	}
	if (input_len > WLC_IOCTL_MAXLEN)
		return BCME_BADARG;

	buf = NULL;
	if (set) {
		if (res_buf || res_len != 0) {
			DHD_ERROR(("%s: SET wrong arguemnet\n", __FUNCTION__));
			ret = BCME_BADARG;
			goto exit;
		}
		buf = MALLOCZ(pub->osh, input_len);
		if (!buf) {
			DHD_ERROR(("%s: mem alloc failed\n", __FUNCTION__));
			ret = BCME_NOMEM;
			goto exit;
		}
		ret = bcm_mkiovar(name, param_buf, param_len, buf, input_len);
		if (!ret) {
			ret = BCME_NOMEM;
			goto exit;
		}

		ioc.cmd = WLC_SET_VAR;
		ioc.buf = buf;
		ioc.len = input_len;
		ioc.set = set;

		ret = dhd_wl_ioctl(pub, ifidx, &ioc, ioc.buf, ioc.len);
	} else {
		if (!res_buf || !res_len) {
			DHD_ERROR(("%s: GET failed. resp_buf NULL or length 0.\n", __FUNCTION__));
			ret = BCME_BADARG;
			goto exit;
		}

		if (res_len < input_len) {
			DHD_INFO(("%s: res_len(%d) < input_len(%d)\n", __FUNCTION__,
					res_len, input_len));
			buf = MALLOCZ(pub->osh, input_len);
			if (!buf) {
				DHD_ERROR(("%s: mem alloc failed\n", __FUNCTION__));
				ret = BCME_NOMEM;
				goto exit;
			}
			ret = bcm_mkiovar(name, param_buf, param_len, buf, input_len);
			if (!ret) {
				ret = BCME_NOMEM;
				goto exit;
			}

			ioc.cmd = WLC_GET_VAR;
			ioc.buf = buf;
			ioc.len = input_len;
			ioc.set = set;

			ret = dhd_wl_ioctl(pub, ifidx, &ioc, ioc.buf, ioc.len);

			if (ret == BCME_OK) {
				memcpy(res_buf, buf, res_len);
			}
		} else {
			memset(res_buf, 0, res_len);
			ret = bcm_mkiovar(name, param_buf, param_len, res_buf, res_len);
			if (!ret) {
				ret = BCME_NOMEM;
				goto exit;
			}

			ioc.cmd = WLC_GET_VAR;
			ioc.buf = res_buf;
			ioc.len = res_len;
			ioc.set = set;

			ret = dhd_wl_ioctl(pub, ifidx, &ioc, ioc.buf, ioc.len);
		}
	}
exit:
	if (buf) {
		MFREE(pub->osh, buf, input_len);
	}
	return ret;
}
#ifdef DHD_LOGLEVEL
void dhd_loglevel_get(dhd_loglevel_data_t *level_data)
{
	level_data->dhd_print_lv = dhd_msg_level;
	level_data->dhd_log_lv = dhd_log_level;
#ifdef WL_CFG80211
	level_data->wl_print_lv = wl_cfg80211_get_print_level();
	level_data->wl_log_lv = wl_cfg80211_get_log_level();
#endif /* WL_CFG80211 */
}

void dhd_loglevel_set(dhd_loglevel_data_t *level_data)
{
	if (level_data->type == DHD_LOGLEVEL_TYPE_ALL) {
#ifdef WL_CFG80211
		if (level_data->component == DHD_LOGLEVEL_COMP_WL) {
			wl_cfg80211_enable_trace(TRUE, level_data->wl_print_lv);
			wl_cfg80211_enable_log_trace(TRUE, level_data->wl_log_lv);
		} else
#endif /* WL_CFG80211 */
		{
			dhd_msg_level = level_data->dhd_print_lv;
			dhd_log_level = level_data->dhd_log_lv;
		}
	} else if (level_data->type == DHD_LOGLEVEL_TYPE_PRINT) {
#ifdef WL_CFG80211
		if (level_data->component == DHD_LOGLEVEL_COMP_WL) {
			wl_cfg80211_enable_trace(TRUE, level_data->wl_print_lv);
		} else
#endif /* WL_CFG80211 */
		{
			dhd_msg_level = level_data->dhd_print_lv;
		}
	} else if (level_data->type == DHD_LOGLEVEL_TYPE_LOG) {
#ifdef WL_CFG80211
		if (level_data->component == DHD_LOGLEVEL_COMP_WL) {
			wl_cfg80211_enable_log_trace(TRUE, level_data->wl_log_lv);
		} else
#endif /* WL_CFG80211 */
		{
			dhd_log_level = level_data->dhd_log_lv;
		}
	}
	DHD_ERROR(("%s: type:%u dhd_msg_level:0x%x dhd_log_level:0x%x\n",
		__func__, level_data->type, dhd_msg_level, dhd_log_level));
#ifdef WL_CFG80211
	DHD_ERROR(("%s: type:%u wl_dbg_level:0x%x wl_log_level:0x%x\n",
		__func__, level_data->type, wl_dbg_level, wl_log_level));
#endif /* WL_CFG80211 */
}
#endif /* DHD_LOGLEVEL */
#if defined(DHD_WAKE_STATUS_PRINT) && defined(DHD_WAKE_RX_STATUS) && \
	defined(DHD_WAKE_STATUS)
#ifdef BCMPCIE
extern int bcmpcie_get_total_wake(struct dhd_bus *bus);
#endif /* BCMPCIE */
void
dhd_dump_wake_status(dhd_pub_t *dhdp, wake_counts_t *wcp, struct ether_header *eh)
{
	uint wake = 0;

	BCM_REFERENCE(wake);
#ifdef BCMPCIE
	wake = bcmpcie_get_total_wake(dhdp->bus);
#endif /* BCMPCIE */

	DHD_ERROR(("wake %u rxwake %u readctrlwake %u\n",
		wake, wcp->rxwake, wcp->rcwake));
	DHD_ERROR(("unicast %u muticast %u broadcast %u icmp %u\n",
		wcp->rx_ucast, wcp->rx_mcast, wcp->rx_bcast, wcp->rx_icmp));
	DHD_ERROR(("multi4 %u multi6 %u icmp6 %u multiother %u\n",
		wcp->rx_multi_ipv4, wcp->rx_multi_ipv6, wcp->rx_icmpv6, wcp->rx_multi_other));
	DHD_ERROR(("icmp6_ra %u, icmp6_na %u, icmp6_ns %u\n",
		wcp->rx_icmpv6_ra, wcp->rx_icmpv6_na, wcp->rx_icmpv6_ns));
	DHD_ERROR(("Src_mac: "MACDBG", Dst_mac: "MACDBG"\n",
		MAC2STRDBG(eh->ether_shost), MAC2STRDBG(eh->ether_dhost)));
	return;
}
#endif /* DHD_WAKE_STATUS_PRINT && DHD_WAKE_RX_STATUS && DHD_WAKE_STATUS */

#ifdef SUPPORT_OTA_UPDATE
void
dhd_ota_buf_clean(dhd_pub_t *dhdp)
{
	ota_update_info_t *ota_info = &dhdp->ota_update_info;

	if (ota_info->clm_buf) {
		MFREE(dhdp->osh, ota_info->clm_buf, ota_info->clm_len);
	}
	if (ota_info->nvram_buf) {
		MFREE(dhdp->osh, ota_info->nvram_buf, ota_info->nvram_len);
	}
	ota_info->nvram_len = 0;
	ota_info->clm_len = 0;
	return;
}
#endif /* SUPPORT_OTA_UPDATE */

int dhd_80211_mode_update(dhd_pub_t *dhdp, int ifidx,
		int gmode, int nmode, int vhtmode, int hemode, int only_mode)
{
	dhd_if_t    *ifp = NULL;
	int          ret = -1;

	ifp = dhd_get_ifp(dhdp, ifidx);
	if (NULL != ifp) {
		/* update each tag first */
		if (-1 != gmode) {
			ifp->gmode = gmode;
		}
		if (-1 != nmode) {
			ifp->nmode = nmode;
		}
		if (-1 != vhtmode) {
			ifp->vhtmode = vhtmode;
		}
		if (-1 != hemode) {
			ifp->hemode = hemode;
		}
		if (-1 != only_mode) {
			ifp->onlymode = only_mode;
		}
	}

	return ret;
}

/*
 * By default the device phymode will be HE. This function compares the device phymode
 * with the required phymode of softAP/p2p. If the function called from wl_cfg80211_stop_ap,
 * force_all_phy_mode_cmp will be set to true to compare all modes.
*/
bool
is_mode_different(dhd_pub_t *dhdp, int ifidx, int nmode, int vhtmode,
		int hemode, chanspec_t chanspec, bool force_all_phy_mode_cmp)
{
	uint8 iovar_buf[DHD_IOVAR_BUF_SIZE];
	uint16 iovar_buf_len = sizeof(iovar_buf);
	int ret = BCME_OK, err = 0;
	int prev_nmode = 0, prev_vhtmode = 0, prev_hemode = 0;
	bcm_xtlv_t *pxtlv = (bcm_xtlv_t *)iovar_buf;

	if (force_all_phy_mode_cmp || (CHSPEC_IS2G(chanspec) || CHSPEC_IS5G(chanspec))) {
		ret = dhd_wl_ioctl_get_intiovar(dhdp, "nmode", &prev_nmode, WLC_GET_VAR, FALSE, 0);
		if (ret == BCME_OK) {
			DHD_ERROR(("%s : Prev nmode:%d set nmode:%d\n",
					__FUNCTION__, prev_nmode, nmode));

			if (prev_nmode != nmode) {
				return TRUE;
			}
		} else {
			DHD_ERROR(("%s : Failed to get nmode:%d\n", __FUNCTION__, ret));
			return TRUE;
		}
	}

	if (force_all_phy_mode_cmp || CHSPEC_IS5G(chanspec) ||
#ifdef SUPPORT_2G_VHT
		CHSPEC_IS2G(chanspec) ||
#endif /* SUPPORT_2G_VHT */
		FALSE) {
		ret = dhd_wl_ioctl_get_intiovar(dhdp, "vhtmode", &prev_vhtmode,
				WLC_GET_VAR, FALSE, 0);
		if (ret == BCME_OK) {
			DHD_ERROR(("%s : Prev vhtmode:%d set vhtmode:%d\n",
					__FUNCTION__, prev_vhtmode, vhtmode));

			if (prev_vhtmode != vhtmode) {
				return TRUE;
			}
		} else {
			DHD_ERROR(("%s : Failed to get vhtmode:%d\n", __FUNCTION__, ret));
			return TRUE;
		}
	}

	ret = bcm_pack_xtlv_entry((uint8**)&pxtlv, &iovar_buf_len,
			WL_HE_CMD_ENAB, 0, NULL, BCM_XTLV_OPTION_ALIGN32);

	if (0 > ret) {
		DHD_ERROR(("%s failed to pack he enab, err: %s\n",
				__FUNCTION__, bcmerrorstr(err)));
		return TRUE;
	} else {
		ret = dhd_iovar(dhdp, ifidx, "he", NULL, 0,
				(char *)&iovar_buf, iovar_buf_len, FALSE);
		if (ret == BCME_OK) {
			memcpy(&prev_hemode, iovar_buf, sizeof(uint32));
			DHD_ERROR(("%s : Prev hemode:%d set hemode:%d\n",
					__FUNCTION__, prev_hemode, hemode));

			if (prev_hemode != hemode) {
				return TRUE;
			}
		}
	}
	return FALSE;
}

/*
   need_down: set to 1 if needs WLC_DOWN before setting, and note another
               interface will be disconnect under the cucurrent case
   need_up:   set to 1 if needs WLC_UP after setting, and note some
              scenario like up too early during no-CLM-blob FW start up
              stage will encounter exception since PHY layer doesn't
              have valid blob.
*/
int dhd_80211_mode_apply_by_value(dhd_pub_t *dhdp, int ifidx,
		int need_down, int need_up,
		int gmode, int nmode, int vhtmode, int hemode, int mode_reqd)
{
	int    err = 0, ret = 0;
	uint8  iovar_buf[DHD_IOVAR_BUF_SIZE];
	uint16 iovar_buf_len = sizeof(iovar_buf);

	DHD_ERROR(("%s: ifidx=%d, need_down=%d, gmode=%d, nmode=%d,"
	           " vhtmode=%d, hemode=%d, mode_reqd=%d\n",
	           __FUNCTION__, ifidx, need_down,
	           gmode, nmode, vhtmode, hemode, mode_reqd));

	if (   (-1 == gmode)
	    && (-1 == nmode)
	    && (-1 == vhtmode)
	    && (-1 == hemode)
	    && (-1 == mode_reqd)
	   ) {
		DHD_ERROR(("%s: skip since no action required\n", __FUNCTION__));
		return 1;
	}

	/* check if need down */
	if (need_down) {
		err = dhd_wl_ioctl_cmd(dhdp, WLC_DOWN, NULL, 0, TRUE, ifidx);
		if (0 > err) {
			DHD_ERROR(("%s: wl down failed, err=%s\n",
			           __FUNCTION__, bcmerrorstr(err)));
			ret += -0x01;
			/* we can still go on to change as much as possible */
		}
	}

	if (-1 != gmode) {
		err = dhd_wl_ioctl_cmd(dhdp, WLC_SET_GMODE, (char *)&gmode,
			sizeof(gmode), TRUE, ifidx);
		if (0 > err) {
			DHD_ERROR(("%s: set g_mode=%d fail, err=%s\n",
			           __FUNCTION__, gmode, bcmerrorstr(err)));
			ret += -0x02;
		}
	}

	if (-1 != nmode) {
		err = dhd_iovar(dhdp, ifidx, "nmode", (char *)&nmode,
			sizeof(nmode), NULL, 0, TRUE);
		if (0 > err) {
			DHD_ERROR(("%s: set ht_mode=%d fail, err=%s\n",
			           __FUNCTION__, nmode, bcmerrorstr(err)));
			ret += -0x04;
		}
	}

	if (-1 != vhtmode) {
		err = dhd_iovar(dhdp, ifidx, "vhtmode", (char *)&vhtmode,
			sizeof(vhtmode), NULL, 0, TRUE);
		if (0 > err) {
			DHD_ERROR(("%s: set vht_mode=%d fail, err=%s\n",
			           __FUNCTION__, vhtmode, bcmerrorstr(err)));
			ret += -0x08;
		}
	}

	if (-1 != hemode) {
		bcm_xtlv_t *pxtlv = (bcm_xtlv_t *)iovar_buf;
		int8        he = hemode;

		err = bcm_pack_xtlv_entry((uint8**)&pxtlv, &iovar_buf_len,
		            WL_HE_CMD_ENAB, sizeof(he),
		            (const uint8 *)(&he), BCM_XTLV_OPTION_ALIGN32);

		if (0 > err) {
			DHD_ERROR(("%s failed to pack he enab, err: %s\n",
			           __FUNCTION__, bcmerrorstr(err)));
			ret += -0x10;
		} else {
			err = dhd_iovar(dhdp, ifidx, "he", (char *)&iovar_buf,
			                sizeof(iovar_buf), NULL, 0, TRUE);
			if (0 > err) {
				DHD_ERROR(("%s: set he_mode=%d fail, err=%s\n",
				           __FUNCTION__, hemode, bcmerrorstr(err)));
				ret += -0x20;
			}
		}
	}

	/* request only mode */
	if (-1 != mode_reqd) {
		err = dhd_iovar(dhdp, ifidx, "mode_reqd", (char *)&mode_reqd,
			sizeof(mode_reqd), NULL, 0, TRUE);
		if (0 > err) {
			DHD_ERROR(("%s: set only_mode=%d fail, err=%s"
			           "(ignore for STA role)\n",
			           __FUNCTION__, mode_reqd, bcmerrorstr(err)));
			ret += -0x40;
		}
	}

	/* check if needs up */
	if (need_up) {
		err = dhd_wl_ioctl_cmd(dhdp, WLC_UP, NULL, 0, TRUE, ifidx);
		if (0 > err) {
			DHD_ERROR(("%s: wl up failed\n", __FUNCTION__));
		}
	}

	if (0 != ret) {
		DHD_ERROR(("%s: final result is 0x%X(0x%X)\n",
		           __FUNCTION__, ret, -ret));
	}
	return ret;
}

/*
   need_down: set to 1 if needs WLC_DOWN before setting, and note another
               interface will be disconnect under the cucurrent case
   need_up:   set to 1 if needs WLC_UP after setting, and note some
              scenario like up too early during no-CLM-blob FW start up
              stage will encounter exception since PHY layer doesn't
              have valid blob.
*/
int dhd_80211_mode_apply(dhd_pub_t *dhdp, uint ifidx, int need_down, int need_up)
{
	dhd_if_t    *ifp = NULL;
	int          ret = -1;

	ifp = dhd_get_ifp(dhdp, ifidx);
	if (NULL != ifp) {
		/* checking dependency */
		if (0 == ifp->hemode) {
			if (OMC_HE == ifp->onlymode) {
				ifp->onlymode = 0;
			}
		} else if (0 == ifp->vhtmode) {
			ifp->hemode = 0;
			if (   (OMC_VHT == ifp->onlymode)
			    || (OMC_HE  == ifp->onlymode)
			   ) {
				ifp->onlymode = 0;
			}
		} else if (0 == ifp->nmode) {
			ifp->vhtmode = 0;
			ifp->hemode  = 0;
			if (   (OMC_VHT == ifp->onlymode)
			    || (OMC_HE  == ifp->onlymode)
			    || (OMC_HT  == ifp->onlymode)
			   ) {
				ifp->onlymode = 0;
			}
		} else if (0 == ifp->gmode) {
			ifp->nmode   = 0;
			ifp->vhtmode = 0;
			ifp->hemode  = 0;
			if (   (OMC_VHT == ifp->onlymode)
			    || (OMC_HE	== ifp->onlymode)
			    || (OMC_HT	== ifp->onlymode)
			   ) {
				ifp->onlymode = 0;
			}
		}

		/* set each mode value */
		ret = dhd_80211_mode_apply_by_value(dhdp, ifidx,
		         need_down, need_up, ifp->gmode, ifp->nmode,
		         ifp->vhtmode, ifp->hemode, ifp->onlymode);
	}

	return ret;
}

uint32 wl_set_bandwidth_capability(dhd_pub_t *dhdp,
	uint32 band, uint32 value,
	int need_down, int need_up)
{
	struct {
		u32 band;
		u32 bw_cap;
	} param = {0, 0};
	s32  err = BCME_OK;

	if (need_down) {
		if (dhd_wl_ioctl_cmd(dhdp, WLC_DOWN, NULL, 0, TRUE, 0) < 0) {
			DHD_ERROR(("%s: wl down failed\n", __FUNCTION__));
		}
	}

	/* reset to maximum capability */
	param.band = htod32(band);
	param.bw_cap = htod32(value);
	err = dhd_iovar(dhdp, 0, "bw_cap", (char *)&param,
	                sizeof(param), NULL, 0, TRUE);
	if (unlikely(err)) {
		DHD_ERROR(("error reset bw_cap %d (%d)\n", band, err));
	}
	DHD_TRACE(("%s-%d: reset bw_cap[%d]=%d\n",\
	           __FUNCTION__, __LINE__, band, value));

	if (need_up) {
		if (dhd_wl_ioctl_cmd(dhdp, WLC_UP, NULL, 0, TRUE, 0) < 0) {
			DHD_ERROR(("%s: wl up failed\n", __FUNCTION__));
		}
	}

	return err;
}
