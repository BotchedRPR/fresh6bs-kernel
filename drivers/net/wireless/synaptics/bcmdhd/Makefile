# bcmdhd
#
# Copyright (C) 2024 Synaptics Incorporated. All rights reserved.
#
# This software is licensed to you under the terms of the
# GNU General Public License version 2 (the "GPL") with Broadcom special exception.
#
# INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
# EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
# AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
# IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
# WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
# AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION
# DOES NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES,
# SYNAPTICS' TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT
# EXCEED ONE HUNDRED U.S. DOLLARS
#
# Copyright (C) 2024, Broadcom.
#
#      Unless you and Broadcom execute a separate written software license
# agreement governing use of this software, this software is licensed to you
# under the terms of the GNU General Public License version 2 (the "GPL"),
# available at http://www.broadcom.com/licenses/GPLv2.php, with the
# following added to such license:
#
#      As a special exception, the copyright holders of this software give you
# permission to link this software with independent modules, and to copy and
# distribute the resulting executable under terms of your choice, provided that
# you also meet, for each linked independent module, the terms and conditions of
# the license of that module.  An independent module is a module which is not
# derived from this software.  The special exception does not apply to any
# modifications of the software.
#
#
# <<Broadcom-WL-IPTag/Open:>>
#

##### customer features pick up
# use '='(equal) rathern than '?=' for below items if needs to force overwrite external predefined value
CONFIG_CHIPSET ?= BCM43012
CONFIG_BCMDHD_NAME ?= dhd
BCMDHD_MODULE_NAME := $(CONFIG_BCMDHD_NAME)
CONFIG_BCMDHD ?= m
CONFIG_BCMDHD_PCIE ?= n
CONFIG_BCMDHD_SDIO ?= y
CONFIG_BCMDHD_SDIO_SDMMC ?= y
# 0:SDPCM_TXGLOM_CPY(copy)
# 1:SDPCM_TXGLOM_MDESC(ADMA)
CONFIG_BCMDHD_SDIO_TXGLOM_MANNER ?= 1
CONFIG_BCMDHD_OOB ?= y
CONFIG_BUILD_CFG80211 ?= y
CONFIG_WPA3_SUPPORT ?= y
CONFIG_BCMDHD_DEBUG ?= n
CONFIG_BCMDHD_LOG ?= n
CONFIG_BROADCOM_WIFI_RESERVED_MEM ?= y
CONFIG_DHD_USE_STATIC_BUF=y
CONFIG_ANDROID_PLATFORM ?= y
CONFIG_BCMDHD_STD_FW_API ?= n
CONFIG_BCMDHD_PKTMON ?= y
CONFIG_BCMDHD_GKI ?= n
CONFIG_FUNCTION_ALIGNMENT ?= 16

### bus type checking
ifeq ($(CONFIG_BCMDHD_PCIE),)
  ifeq ($(CONFIG_BCMDHD_SDIO),)
    CONFIG_BCMDHD_PCIE = y
    CONFIG_BCMDHD_SDIO = n
  else # CONFIG_BCMDHD_SDIO
    ifeq ($(CONFIG_BCMDHD_SDIO),y)
      CONFIG_BCMDHD_PCIE = n
      CONFIG_BCMDHD_SDIO = y
    else # CONFIG_BCMDHD_SDIO
      CONFIG_BCMDHD_PCIE = y
      CONFIG_BCMDHD_SDIO = n
    endif # CONFIG_BCMDHD_SDIO
  endif # CONFIG_BCMDHD_SDIO
else # CONFIG_BCMDHD_PCIE
  ifeq ($(CONFIG_BCMDHD_PCIE),y)
    CONFIG_BCMDHD_PCIE = y
    CONFIG_BCMDHD_SDIO = n
  else # CONFIG_BCMDHD_PCIE
    ifeq ($(CONFIG_BCMDHD_SDIO),)
      CONFIG_BCMDHD_PCIE = y
      CONFIG_BCMDHD_SDIO = n
    else # CONFIG_BCMDHD_SDIO
      ifeq ($(CONFIG_BCMDHD_SDIO),y)
        CONFIG_BCMDHD_PCIE = n
        CONFIG_BCMDHD_SDIO = y
      else # CONFIG_BCMDHD_SDIO
        CONFIG_BCMDHD_PCIE = y
        CONFIG_BCMDHD_SDIO = n
      endif # CONFIG_BCMDHD_SDIO
    endif # CONFIG_BCMDHD_SDIO
  endif # CONFIG_BCMDHD_SDIO
endif # CONFIG_BCMDHD_PCIE
ifeq ($(CONFIG_BCMDHD_SDIO),y)
  ifneq ($(CONFIG_BCMDHD_SDIO_SDMMC),y)
    CONFIG_BCMDHD_SDIO_SDSTD = y
  else # CONFIG_BCMDHD_SDIO_SDMMC
    CONFIG_BCMDHD_SDIO_SDSTD = n
  endif # CONFIG_BCMDHD_SDIO_SDMMC
endif # CONFIG_BCMDHD_SDIO

### Path to the module source
M ?= $(shell pwd)

# For inbuilt module, below configs will be provided via defconfig
# But for out-of-tree module, explicitly define them here and add
# them as cflags
ifeq ($(KERNEL_SRC),)
  LINUXDIR ?= $(KDIR)
  ifneq ($(LINUXDIR),)
    KERNEL_SRC ?= $(LINUXDIR)
  endif
  MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
  MAKEFILE_DIR := $(dir ${MAKEFILE_PATH})
  BCMDHD_ROOT=${MAKEFILE_DIR}
  OUT_OF_TREE_BUILD := y
else # KERNEL_SRC
  KBUILD_OPTIONS += BCMDHD_ROOT=$(shell cd $(KERNEL_SRC); readlink -e $(M))
endif # KERNEL_SRC

# Account for compiler with full path
IS_CLANG=$(shell echo $(CC) | sed -e 's/.*clang.*/clang/')

ifneq ($(IS_CLANG),clang)
  ifneq ($(LLVM),1)
    # Except for in-kernel clang build
    CC ?= $(CROSS_COMPILE)gcc
    LD ?= $(CROSS_COMPILE)ld
    NM ?= $(CROSS_COMPILE)nm
    OBJCOPY ?= $(CROSS_COMPILE)objcopy
    OBJDUMP ?= $(CROSS_COMPILE)objdump
    STRIP ?= $(CROSS_COMPILE)strip
  endif # LLVM
endif # CC

### Derive Linux version
LINUXSRC := $(LINUXDIR)
### Try to fix for not local build
ifneq ($(wildcard $(LINUXSRC)/source), )
  LINUXSRC := $(LINUXDIR)/source
endif
ifneq ($(LINUXSRC),)
  VERSION = $(shell grep "^VERSION =" $(LINUXSRC)/Makefile | cut -d " " -f 3)
  PATCHLEVEL = $(shell grep "^PATCHLEVEL =" $(LINUXSRC)/Makefile | cut -d " " -f 3)
  SUBLEVEL = $(shell grep "^SUBLEVEL =" $(LINUXSRC)/Makefile | cut -d " " -f 3)
endif # LINUXSRC
BCM_KVERSIONSTRING := $(VERSION)$(if $(PATCHLEVEL),.$(PATCHLEVEL)$(if $(SUBLEVEL),.$(SUBLEVEL)))

### Compares two dotted numeric strings (e.g 2.3.16.1) for $1 >= $2
define wlan_version_ge
$(findstring TRUE,$(shell bash -c 'sort -cu -t. -k1,1nr -k2,2nr -k3,3nr -k4,4nr <(echo -e "$2\n$1") 2>&1 || echo TRUE'))
endef # wlan_version_ge
ifeq ($(GCCVER),)
  GCCVER = $(shell $(CC) --version | grep -Eo '[0-9x]+(\.[0-9x]+)+' | tail -1)
endif # GCCVER



ifeq ($(ARCH),x86)
  # Usually x86 platform doens't need the OOB related feature
  CONFIG_BCMDHD_OOB=n
endif # ARCH = x86

##### SDIO/PCIE Basic feature
WFLAGS := -Wall \
    -Wstrict-prototypes \
    -Werror \
    -Wno-main \
    -Wno-format-zero-length \
    -Wswitch \
    -Wno-sign-compare \
    -Wno-pointer-sign \
    -Wno-attributes \
    -Wno-unused-parameter \
    -fno-strict-aliasing \
    -Woverride-init \
    -Wuninitialized \
    -Wunused-variable \
    -DSTRICT_GCC_WARNINGS

ifeq ($(call wlan_version_ge,$(GCCVER),8.0),TRUE)
WFLAGS += \
    -Werror=sizeof-pointer-memaccess \
    -Werror=format
endif # ($(call wlan_version_ge,$(GCCVER),8.0),TRUE)

WFLAGS_EXTRA_A := \
    -Wclobbered \
    -Wmissing-parameter-type \
    -Wold-style-declaration \
    -Wunused-but-set-variable \
    -Wmaybe-uninitialized

WFLAGS_EXTRA_B := \
    -Wtype-limits \
    -Wmissing-field-initializers

WFLAGS_EXTRA_C := -Werror=stringop-overflow

# more stric syntax checking accordingly
ifneq ($(IS_CLANG),clang)
  WFLAGS += ${WFLAGS_EXTRA_A}
  ifeq ($(call wlan_version_ge,$(GCCVER),8.0),TRUE)
    WFLAGS += ${WFLAGS_EXTRA_C}
  endif # ($(call wlan_version_ge,$(GCCVER),8.0),TRUE)
endif # CC != clang

ifeq ($(CONFIG_ARCH_HISI),y)
  WFLAGS += ${WFLAGS_EXTRA_B}
endif # CONFIG_ARCH_HISI = y


##### common feature
DHDCFLAGS := ${WFLAGS}

DHDCFLAGS += -Dlinux -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DBCMDMA32        \
	-DBCMFILEIMAGE -DDHDTHREAD -DSHOW_EVENTS -DWLP2P -DWIFI_ACT_FRAME  \
	-DARP_OFFLOAD_SUPPORT -DKEEP_ALIVE -DPKT_FILTER_SUPPORT            \
	-DEMBEDDED_PLATFORM -DSEC_ENHANCEMENT -DDHD_USE_RANDMAC            \
	-DUSE_NEW_RSPEC_DEFS -DWL_P2P_USE_RANDMAC -DBCMUTILS_ERR_CODES

##### optional feature
# FW download manner should be either:
#  DHD_LINUX_STD_FW_API: standard kernel FW API
#  DHD_SUPPORT_VFS_CALL: legacy file operation API
ifneq ($(CONFIG_BCMDHD_STD_FW_API),)
DHDCFLAGS += -DDHD_LINUX_STD_FW_API
else # CONFIG_BCMDHD_STD_FW_API
DHDCFLAGS += -DDHD_SUPPORT_VFS_CALL
endif # CONFIG_BCMDHD_STD_FW_API

DHDCFLAGS += -DBCMDHD_MODULE_NAME=${BCMDHD_MODULE_NAME}
DHDCFLAGS += -DWL_VIRTUAL_APSTA
DHDCFLAGS += -DPOWERUP_MAX_RETRY=3
ifeq ($(CONFIG_ARCH_HISI),y)
  DHDCFLAGS += -DBOARD_HIKEY  # get memory dump
  DHDCFLAGS += -DSHOW_LOGTRACE -DDHD_FW_COREDUMP -DDHD_COREDUMP
  ### send up the event as skb packet and some WL command like 'escanresults' may need this
  DHDCFLAGS += -DWL_EVENT_ENAB
endif # CONFIG_ARCH_HISI = y

# Debugability
ifeq ($(CONFIG_BCMDHD_LOG),y)
  CONFIG_BCMDHD_DEBUG = y
endif # CONFIG_BCMDHD_LOG
ifeq ($(CONFIG_BCMDHD_DEBUG),y)
  DHDCFLAGS += -DDHD_DEBUG
  DHDCFLAGS += -DBCMDBG
  # Dongle init fail
  DHDCFLAGS += -DDEBUG_DNGL_INIT_FAIL
  DHDCFLAGS += -DDHD_FW_COREDUMP
  DHDCFLAGS += -DBCMDBG_MEM
  DHDCFLAGS += -DDHD_RND_DEBUG
  DHDCFLAGS += -DBCMPERFSTATS
  DHDCFLAGS += -DDNGL_EVENT_SUPPORT
  DHDCFLAGS += -DREPORT_FATAL_TIMEOUTS
  DHDCFLAGS += -DDEVICE_TX_STUCK_DETECT
  DHDCFLAGS += -DTX_STATUS_LATENCY_STATS
  DHDCFLAGS += -DAWDL_SLOT_STATS
  DHDCFLAGS += -DDBG_RING_LOG_INIT_DEFAULT
  DHDCFLAGS += -DBCMDBG_ASSERT
  DHDCFLAGS += -DDBG_PKT_MON
  DHDCFLAGS += -DRSSI_MONITOR_SUPPORT
  DHDCFLAGS += -DSUPPORT_RSSI_SUM_REPORT
  DHDCFLAGS += -DSET_SSID_FAIL_CUSTOM_RC=100
  ifeq ($(CONFIG_SEC_FACTORY),y)
    # issue TRAP related
    DHDCFLAGS += -DDHD_DETECT_CONSECUTIVE_MFG_HANG
    DHDCFLAGS += -DBCM_DETECT_TURN_ON_FAILURE
  endif # CONFIG_SEC_FACTORY
  ifeq ($(CONFIG_BCMDHD_PCIE),y)
    DHDCFLAGS += -DDHD_TIMESYNC
  endif # CONFIG_BCMDHD_PCIE
  ifeq ($(CONFIG_BUILD_CFG80211),y)
    DHDCFLAGS += -DDEBUGFS_CFG80211
  endif # CONFIG_BUILD_CFG80211

  # control the log level
  DHDCFLAGS += -DDHD_LOGLEVEL
endif # CONFIG_BCMDHD_DEBUG

ifeq ($(CONFIG_BCMDHD_LOG),y)
  # Enable Log Dump
  DHDCFLAGS += -DDHD_LOG_DUMP
  DHDCFLAGS += -DSHOW_LOGTRACE
  # coredump
  DHDCFLAGS += -DDHD_FW_COREDUMP
  #  depends on DHD_LOG_DUMP
  # Enable log print rate limit
  DHDCFLAGS += -DDHD_LOG_PRINT_RATE_LIMIT
  # Logging
  DHDCFLAGS += -DDHD_8021X_DUMP
  DHDCFLAGS += -DDHD_DHCP_DUMP
  DHDCFLAGS += -DDHD_ICMP_DUMP
  DHDCFLAGS += -DDHD_ARP_DUMP
  DHDCFLAGS += -DDHD_DNS_DUMP
  DHDCFLAGS += -DDHD_PKTDUMP_ROAM
  DHDCFLAGS += -DDHD_IFDEBUG
  DHDCFLAGS += -DCUSTOM_LOG_DUMP_BUFSIZE_MB=1
  DHDCFLAGS += -DDHD_DBG_DUMP
  DHDCFLAGS += -DDHD_DUMP_BUF_KVMALLOC
  # Print out kernel panic point of file and line info when assertion happened
  DHDCFLAGS += -DBCMASSERT_LOG
  # Enable SI utility error log
  DHDCFLAGS += -DSI_ERROR_ENFORCE
  ifeq ($(CONFIG_BCMDHD_PCIE),y)
    # Enable SSSR Dump
    DHDCFLAGS += -DDHD_SSSR_DUMP
    # Enable SMD/Minidump collection
    DHDCFLAGS += -DD2H_MINIDUMP
    # ROT and Scan timeout debugging due to Kernel scheduling problem
    DHDCFLAGS += -DDHD_KERNEL_SCHED_DEBUG
    # EDL
    DHDCFLAGS += -DEWP_EDL
    DHDCFLAGS += -DEWP_ETD_PRSRV_LOGS
    # EWP
    DHDCFLAGS += -DDHD_PCIE_REG_ACCESS
    DHDCFLAGS += -DEWP_ECNTRS_LOGGING
    DHDCFLAGS += -DDHD_HMAPTEST
    DHDCFLAGS += -DEVENT_LOG_RATE_HC
    DHDCFLAGS += -DCONFIG_BCMDHD_PREALLOC_MEMDUMP

    DHDCFLAGS += -DDHD_PKT_LOGGING
    DHDCFLAGS += -DDHD_PKT_LOGGING_DBGRING
  endif # CONFIG_BCMDHD_PCIE
endif # CONFIG_BCMDHD_LOG

# turn on below key features supported
DHDCFLAGS += -DDHD_MSG_LEVEL_SUPPORT
DHDCFLAGS += -DDHD_FW_LOG_SUPPORT
DHDCFLAGS += -DDHD_FW_CONS_SUPPORT
DHDCFLAGS += -DDHD_8021X_DUMP
DHDCFLAGS += -DWL_CFGVENDOR_SEND_ALERT_EVENT
DHDCFLAGS += -DDHD_UPDATE_INTF_MAC

# for debug trap
DHDCFLAGS += -DWL_CFGVENDOR_SEND_HANG_EVENT

# customer configuration file
DHDCFLAGS += -DREAD_CONFIG_FROM_FILE

#customer config WME_AC STA parameters
DHDCFLAGS += -DSYNA_CUSTOMER_WME_AC

# Memory consumed by DHD
DHDCFLAGS += -DDHD_MEM_STATS

# Enable SBN feature
DHDCFLAGS += -DDHD_SBN

# MBO/OCE
DHDCFLAGS += -DWL_OCE
DHDCFLAGS += -DWL_MBO
# Advertise OCE_STA capability
DHDCFLAGS += -DWL_CAP_OCE_STA

# Wapi
DHDCFLAGS += -DBCMWAPI_WPI -DBCMWAPI_WAI

# CCX
DHDCFLAGS += -DBCMCCX

# FBT
DHDCFLAGS += -DWLFBT

# OKC
DHDCFLAGS += -DOKC_SUPPORT
DHDCFLAGS += -DWES_SUPPORT
DHDCFLAGS += -DSUPPORT_RESTORE_SCAN_PARAMS

# monitor feature
ifeq ($(CONFIG_BCMDHD_PKTMON),y)
  # For smartmu feature , both of the following MACRO have to be
  # enabled, and may conflict with other generic monitor feature
  DHDCFLAGS += -DWL_MON_OWN_PKT
  DHDCFLAGS += -DWL_MONITOR
else # CONFIG_BCMDHD_PKTMON
  # Manually open this if needs the monitor feature
  ifneq (y, y) # user-choice
    DHDCFLAGS += -DDBG_PKT_MON
    DHDCFLAGS += -DHOST_RADIOTAP_CONV
    # Enable monitor chanspec setting from cfg80211 ops
    DHDCFLAGS += -DWL_CFG80211_MONITOR
    ifeq ($(CONFIG_BCMDHD_PCIE),y)
      # Support Monitor Mode only under PCIE
      DHDCFLAGS += -DWL_MONITOR
    endif # CONFIG_BCMDHD_PCIE
  endif # user-choice
endif # CONFIG_BCMDHD_PKTMON

# CFG80211
ifeq ($(CONFIG_BUILD_CFG80211),y)
  DHDCFLAGS += -DWL_CFG80211
  ifeq ($(CONFIG_CFG80211_FILS_BKPORT),y)
    DHDCFLAGS += -DWL_FILS
  endif # CONFIG_CFG80211_FILS_BKPORT
  ifeq ($(CONFIG_CFG80211_FILS_ROAM_BKPORT),y)
    DHDCFLAGS += -DWL_FILS_ROAM_OFFLD
    DHDCFLAGS += -DCFG80211_ROAM_API_GE_4_12
  endif # CONFIG_CFG80211_FILS_ROAM_BKPORT

  DHDCFLAGS += -DWL_GCMP_SUPPORT
  DHDCFLAGS += -DDISABLE_CCMP_GCMP256

  # Android iface management
  DHDCFLAGS += -DWL_IFACE_MGMT

  # for WPA3
  ifeq ($(CONFIG_WPA3_SUPPORT),y)
    # WPA3 external authenticator
    ifeq ($(call wlan_version_gt,$(BCM_KVERSIONSTRING),4.16),TRUE)
      # for external supplicant (only kernel > 4.16 support external authenticator)
      DHDCFLAGS += -DWL_CLIENT_SAE
    else # ($(call wlan_version_gt,$(BCM_KVERSIONSTRING),4.16),TRUE)
      ### old kernel needs back port support, so add the kernel patch first,
      ### and then change below 'CONFIG_CFG80211_SAE_BKPORT' item to be 'y'
      CONFIG_CFG80211_SAE_BKPORT ?= n
      ifeq ($(CONFIG_CFG80211_SAE_BKPORT),y)
        DHDCFLAGS += -DWL_CLIENT_SAE
      endif # CONFIG_CFG80211_SAE_BKPORT
    endif # ($(call wlan_version_gt,$(BCM_KVERSIONSTRING),4.16),TRUE)

    ### for dongle in-driver-supplicant
    # DHDCFLAGS += -DWL_SAE
  endif # CONFIG_WPA3_SUPPORT

  ifeq ($(CONFIG_CFG80211_SCANTYPE_BKPORT),y)
    DHDCFLAGS += -DWL_SCAN_TYPE
  endif # CONFIG_CFG80211_SCANTYPE_BKPORT
endif # CONFIG_BUILD_CFG80211

# Block ARP during DHCP on STA/SoftAP concurrent mode
DHDCFLAGS += -DAPSTA_BLOCK_ARP_DURING_DHCP

ifeq ($(CONFIG_BCMDHD_PCIE),y)
  # Enable Load Balancing support by default.
  # DHD_LB_RXP - Perform RX Packet processing in parallel, default enabled
  # DHD_LB_RXP_DEACTIVATED - set lb_rxp_active to 0 (disable).
  # DHD_LB_TXP - Perform TX Packet processing in parallel, default disabled, enabled using DHD_LB_TXP_DEFAULT_ENAB
  # DHD_LB_STATS - To display the Load Blancing statistics
  DHDCFLAGS += -DDHD_LB -DDHD_LB_RXP -DDHD_LB_TXP -DDHD_LB_STATS
  DHDCFLAGS += -DDHD_LB_PRIMARY_CPUS=0xF0 -DDHD_LB_SECONDARY_CPUS=0x0E
  DHDCFLAGS += -DDHD_LB_RXP_DEACTIVATED
ifneq ($(CONFIG_BCMDHD_SINGLE_CORE),y)
  # Use spin_lock_bh locks
  DHDCFLAGS += -DDHD_USE_SPIN_LOCK_BH
endif # CONFIG_BCMDHD_SINGLE_CORE
  # Enable CTO Recovery
  DHDCFLAGS += -DBCMPCIE_CTO_PREVENTION
  # Enable inband device wake feature
  DHDCFLAGS += -DPCIE_INB_DW
  # GRO (Generic Receive Offload) feature
  DHDCFLAGS += -DENABLE_DHD_GRO
  DHDCFLAGS += -DDHD_GRO_ENABLE_HOST_CTRL

  # IRQSET in DHD
  ifeq ($(CONFIG_BCMDHD),y) # only turn on when built-in since it requires kernel non-export '__irq_set_affinity'
    DHDCFLAGS += -DDHD_LB_IRQSET
  endif # CONFIG_BCMDHD
  DHDCFLAGS += -DDHD_MSI_SUPPORT
  # Enable PKTID AUDIT
  DHDCFLAGS += -DDHD_PKTID_AUDIT_ENABLED
  DHDCFLAGS += -DIDLE_TX_FLOW_MGMT

  # Dongle reset during Wifi ON to keep in same state
  DHDCFLAGS += -DFORCE_DONGLE_RESET_IN_DEVRESET_ON

  # Enable Host SFH LLC insertion in Tx pkts
  DHDCFLAGS += -DHOST_SFH_LLC

  # throughput tuning
  ifneq ($(CONFIG_BCMDHD_HTPUT),)
   $(warning : ** HTPUT build)
   DHDCFLAGS += -DDHD_HTPUT_TUNABLES
  endif

  # Runtime PM feature

  ifeq ($(CONFIG_ARCH_MESON),)  # disable runtime PM for vim3
    DHDCFLAGS += -DDHD_PCIE_RUNTIMEPM -DMAX_IDLE_COUNT=11

  endif # CONFIG_ARCH_MESON
endif # CONFIG_BCMDHD_PCIE

ifeq ($(CONFIG_FIB_RULES),y)
  # HAL File dump is supported only for iptable builds(brcm_wlan_iptables_defconfig)
  DHDCFLAGS += -DDHD_FILE_DUMP_EVENT
  # HAL File dump is supported only for iptable builds(brcm_wlan_iptables_defconfig)
  DHDCFLAGS := $(filter-out -DDHD_DUMP_FILE_WRITE_FROM_KERNEL ,$(DHDCFLAGS))
endif

# CUSTOMER flags
ifneq ($(CONFIG_BCM43456)$(CONFIG_43012),)
DHDCFLAGS += -DNUM_EVENT_LOG_SETS=8
# Random scan
DHDCFLAGS += -DSUPPORT_RANDOM_MAC_SCAN
DHDCFLAGS += -DWL_USE_RANDOMIZED_SCAN
else
# 43456 is Beamformee only
DHDCFLAGS += -DUSE_WL_TXBF
# Disabled in FW
DHDCFLAGS += -DDISABLE_PM_BCNRX
# TODO Add later
DHDCFLAGS += -DENABLE_IPMCAST_FILTER
endif

# Basic / Common Feature
DHDCFLAGS += -DDISABLE_FRAMEBURST_VSDB
DHDCFLAGS += -DUSE_WL_FRAMEBURST
DHDCFLAGS += -DUSE_WL_TXBF
DHDCFLAGS += -DSOFTAP_UAPSD_OFF
DHDCFLAGS += -DVSDB
DHDCFLAGS += -DWL_CFG80211_STA_EVENT
DHDCFLAGS += -DWL11U
DHDCFLAGS += -DMFP
DHDCFLAGS += -DDHD_EXTRA_EVENT

# Enable auto country to support country 00
DHDCFLAGS += -DWL_AUTO_COUNTRY
# for regdomain dynamically update with country
DHDCFLAGS += -DWIPHY_DYNAMIC_UPDATE
# use customer country code mapping table
DHDCFLAGS += -DCUSTOM_COUNTRY_CODE
# support kernel call to change the country
DHDCFLAGS += -DCONFIG_CFG80211_INTERNAL_REGDB
# for supporting SAR related feature
DHDCFLAGS += -DSYNA_SAR_CUSTOMER_PARAMETER

# MAC
DHDCFLAGS += -DENABLE_ARP_SNOOP_MODE
# Skip supplicant bssid and channel hints
DHDCFLAGS += -DWL_SKIP_CONNECT_HINTS
# Phy / System
DHDCFLAGS += -DDHD_ENABLE_LPC
DHDCFLAGS += -DDISABLE_PM_BCNRX
DHDCFLAGS += -DFCC_PWR_LIMIT_2G
DHDCFLAGS += -DSUPPORT_2G_VHT
DHDCFLAGS += -DSUPPORT_5G_1024QAM_VHT
DHDCFLAGS += -DSUPPORT_LTECX
DHDCFLAGS += -DSUPPORT_LQCM
DHDCFLAGS += -DSUPPORT_SET_CAC
DHDCFLAGS += -DSUPPORT_WL_TXPOWER
DHDCFLAGS += -DWL_THERMAL_MITIGATION
DHDCFLAGS += -DDHD_DFS_MASTER
# Support to update clm/nvram through downloading OTA
DHDCFLAGS += -DSUPPORT_OTA_UPDATE
# Roaming feature
DHDCFLAGS += -DDHD_LOSSLESS_ROAMING
DHDCFLAGS += -DWL_ROAM_TIMEOUT_MS=2000 # ms
DHDCFLAGS += -DENABLE_FW_ROAM_SUSPEND
DHDCFLAGS += -DROAM_API
# Removed for 43013  DHDCFLAGS += -DROAM_AP_ENV_DETECTION
DHDCFLAGS += -DROAM_CHANNEL_CACHE
DHDCFLAGS += -DROAM_ENABLE
DHDCFLAGS += -DSKIP_ROAM_TRIGGER_RESET
DHDCFLAGS += -DWBTEXT
DHDCFLAGS += -DWBTEXT_BTMDELTA=0
DHDCFLAGS += -DWBTEXT_SCORE_V2
DHDCFLAGS += -DRRM_BCNREQ_MAX_CHAN_TIME=12
DHDCFLAGS += -DWL_LASTEVT
DHDCFLAGS += -DROAM_EVT_DISABLE

# Wake related
DHDCFLAGS += -DCONFIG_HAS_WAKELOCK
DHDCFLAGS += -DWAKELOCK_BACKPORT

# for non-android platform, we could disable below MACRO
ifneq ($(CONFIG_ARCH_ROCKCHIP),y)
DHDCFLAGS += -DDHD_WAKE_STATUS
DHDCFLAGS += -DDHD_WAKE_RX_STATUS
DHDCFLAGS += -DDHD_WAKE_EVENT_STATUS
DHDCFLAGS += -DCUSTOM_WAKE_REASON_STATS
DHDCFLAGS += -DDHD_WAKEPKT_SET_MARK
endif
# DHDCFLAGS += -DDHD_USE_SCAN_WAKELOCK

DHDCFLAGS += -DPNO_SUPPORT
# Scheduled scan (PNO)
DHDCFLAGS += -DWL_SCHED_SCAN
DHDCFLAGS += -DDISABLE_ADAPTIVE_PNO
# Gscan needs PNO
DHDCFLAGS += -DGSCAN_SUPPORT

# FW ROAM control
DHDCFLAGS += -DROAMEXP_SUPPORT

# Android Feature
ifeq ($(CONFIG_ANDROID_PLATFORM),y)
  DHDCFLAGS += -DOEM_ANDROID
  DHDCFLAGS += -DGET_CUSTOM_MAC_ENABLE
  DHDCFLAGS += -DDHD_SET_MAC_AFTER_DOWN
  CONFIG_DHD_SET_RANDOM_MAC_VAL ?= 0x001A11
  DHDCFLAGS += -DAPF
  DHDCFLAGS += -DDHD_GET_VALID_CHANNELS
  DHDCFLAGS += -DLINKSTAT_SUPPORT
  DHDCFLAGS += -DLINKSTAT_EXT_SUPPORT
  DHDCFLAGS += -DPFN_SCANRESULT_2
  DHDCFLAGS += -DWL_IFACE_COMB_NUM_CHANNELS
  DHDCFLAGS += -DASSOC_PREF_5G
  DHDCFLAGS += -DASSOC_PREFER_RSSI_THRESH=-65
  DHDCFLAGS += -DQOS_MAP_SET
  # Custom Mapping of DSCP to User Priority
  DHDCFLAGS += -DWL_CUSTOM_MAPPING_OF_DSCP
  # FAKE AP
  DHDCFLAGS += -DWL_BCNRECV
  # ADDTS
  DHDCFLAGS += -DWL_CAC_TS
  # ADPS
  DHDCFLAGS += -DWLADPS

  # Android Q
  DHDCFLAGS += -DWL_USE_RANDOMIZED_SCAN
  DHDCFLAGS += -DSTA_RANDMAC_ENFORCED
  # Connected MAC randomization
  DHDCFLAGS += -DWL_STA_ASSOC_RAND
  # Soft AP MAC randomization
  DHDCFLAGS += -DWL_SOFTAP_RAND
  # p2p MAC randomization
  DHDCFLAGS += -DWL_P2P_RAND
  # Enable below define for production
  # DHDCFLAGS += -DMACADDR_PROVISION_ENFORCED
  # Android iface management
  DHDCFLAGS += -DWL_IFACE_MGMT
  DHDCFLAGS += -DWL_IFACE_MGMT_CONF
  # TWT HAL
  DHDCFLAGS += -DWL_TWT_HAL_IF
  # Enable usable_channel support for HAL layer
  DHDCFLAGS += -DWL_USABLE_CHAN
  # Thermal mitigation threshold *Mbps
  # DHDCFLAGS += -DTHERMAL_MITIGATION_THRESHOLD=1

  # Silent roam
  DHDCFLAGS += -DCONFIG_SILENT_ROAM
  # Highband roam
  DHDCFLAGS += -DHIGHBAND_ROAM
  DHDCFLAGS += -DHIGHBAND_PERIOD_TIME=120
  DHDCFLAGS += -DHIGHBAND_INACT_CNT=16
  DHDCFLAGS += -DHIGHBAND_INACT_TIME=10
  DHDCFLAGS += -DHIGHBAND_RSSI=-60
else # CONFIG_ANDROID_PLATFORM
  DHDCFLAGS += -DDISABLE_ANDROID_PNO
endif # CONFIG_ANDROID_PLATFORM


DHDCFLAGS += -DENABLE_BCN_LI_BCN_WAKEUP
DHDCFLAGS += -DSET_SSID_FAIL_CUSTOM_RC=100
# Packet
#DHDCFLAGS += -DBLOCK_IPV6_PACKET
#DHDCFLAGS += -DDHD_DONOT_FORWARD_BCMEVENT_AS_NETWORK_PKT # NAN test failure
DHDCFLAGS += -DPASS_ALL_MCAST_PKTS
DHDCFLAGS += -DPKTPRIO_OVERRIDE

ifeq ($(CONFIG_BCMDHD_PCIE),y)
  # Kernel/Platform Related Feature
  #DHDCFLAGS += -DDHD_LB_TXP_DEFAULT_ENAB # Not needed for Brix
  #DHDCFLAGS += -DDHD_RECOVER_TIMEOUT # Not needed for Brix
  #DHDCFLAGS += -DDHD_USE_ATOMIC_PKTGET # Not needed for Brix
  #DHDCFLAGS += -DDHD_USE_COHERENT_MEM_FOR_RING # Not needed for Brix
  #DHDCFLAGS += -DDONGLE_ENABLE_ISOLATION # Not compatible with Brix platform
  #DHDCFLAGS += -DKEEP_WIFION_OPTION # Not compatible with Brix platform
endif # CONFIG_BCMDHD_PCIE

DHDCFLAGS += -DWL_SUPPORT_BACKPORTED_KPATCHES
# SoftAP
DHDCFLAGS += -DSUPPORT_AP_HIGHER_BEACONRATE
DHDCFLAGS += -DSUPPORT_AP_RADIO_PWRSAVE
DHDCFLAGS += -DSUPPORT_HIDDEN_AP
DHDCFLAGS += -DSUPPORT_SOFTAP_SINGL_DISASSOC
DHDCFLAGS += -DWL_SUPPORT_AUTO_CHANNEL
DHDCFLAGS += -DSUPPORT_SOFTAP_WPAWPA2_MIXED
DHDCFLAGS += -DSOFTAP_TPUT_ENHANCE
# P2P
DHDCFLAGS += -DP2P_LISTEN_OFFLOADING
DHDCFLAGS += -DP2P_SKIP_DFS
# P2P GO Negotiation
# DHDCFLAGS += -DWL_CFG80211_GON_COLLISION
DHDCFLAGS += -DDIS_COEX_FOR_P2P_DISC
# for 802.11 mode setting and force bandwidth
DHDCFLAGS += -DWL_NEWCFG_PRIVCMD_SUPPORT

# SCAN
DHDCFLAGS += -DCUSTOMER_SCAN_TIMEOUT_SETTING
DHDCFLAGS += -DDISABLE_PRUNED_SCAN
DHDCFLAGS += -DESCAN_BUF_OVERFLOW_MGMT
DHDCFLAGS += -DUSE_INITIAL_SHORT_DWELL_TIME
DHDCFLAGS += -DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
DHDCFLAGS += -DSUPPORT_RANDOM_MAC_SCAN
DHDCFLAGS += -DCUSTOM_SCAN_PASSIVE_TIME=110
DHDCFLAGS += -DCUSTOM_FORCE_NODFS_FLAG    # no-DFS during SCAN time

# Suspend/Resume
DHDCFLAGS += -DENABLE_MAX_DTIM_IN_SUSPEND
DHDCFLAGS += -DSUPPORT_DEEP_SLEEP

# Misc Features
DHDCFLAGS += -DDHD_BLOB_EXISTENCE_CHECK
DHDCFLAGS += -DWL_RELMCAST
DHDCFLAGS += -DWL_SUPP_EVENT
DHDCFLAGS += -DDISABLE_WL_FRAMEBURST_SOFTAP
DHDCFLAGS += -DFILTER_IE
DHDCFLAGS += -DCUSTOM_LONG_RETRY_LIMIT=12
# usually the upper layer supplicant can buffer the M1 before associated
#DHDCFLAGS += -DDHD_POST_EAPOL_M1_AFTER_ROAM_EVT
# Kind of WAR
DHDCFLAGS += -DENABLE_TDLS_AUTO_MODE
DHDCFLAGS += -DEXPLICIT_DISCIF_CLEANUP
DHDCFLAGS += -DSKIP_WLFC_ON_CONCURRENT
DHDCFLAGS += -DCUSTOM_BLOCK_DEAUTH_AT_EAP_FAILURE
DHDCFLAGS += -DTDLS_MSG_ONLY_WFD
# Custom tuning value
DHDCFLAGS += -DCUSTOM_ROAM_TIME_THRESH_IN_SUSPEND=6000
DHDCFLAGS += -DCUSTOM_EVENT_PM_WAKE=30
DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=30000
# For special PNO Event keep wake lock for 10sec
DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=10
DHDCFLAGS += -DSPECIFIC_MAC_GEN_SCHEME
DHDCFLAGS += -DSET_RANDOM_MAC_SOFTAP
ifeq ($(CONFIG_DHD_SET_RANDOM_MAC_VAL),y)
  DHDCFLAGS += -DCONFIG_DHD_SET_RANDOM_MAC_VAL=$(CONFIG_DHD_SET_RANDOM_MAC_VAL)
endif # CONFIG_DHD_SET_RANDOM_MAC_VAL
DHDCFLAGS += -DCUSTOM_TDLS_IDLE_MODE_SETTING=10000
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_HIGH=-80
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_LOW=-85
DHDCFLAGS += -DCUSTOM_TCPACK_SUPP_RATIO=15
DHDCFLAGS += -DCUSTOM_TCPACK_DELAY_TIME=10
DHDCFLAGS += -DCUSTOM_AMPDU_BA_WSIZE=32
DHDCFLAGS += -DCUSTOM_AMPDU_MPDU=32
DHDCFLAGS += -DD3_ACK_RESP_TIMEOUT=4000
DHDCFLAGS += -DIOCTL_RESP_TIMEOUT=5000
DHDCFLAGS += -DMAX_DTIM_ALLOWED_INTERVAL=925
DHDCFLAGS += -DNUM_SCB_MAX_PROBE=3
DHDCFLAGS += -DWL_SCB_TIMEOUT=10
# need to tune according if needs
DHDCFLAGS += -DWIFI_TURNON_DELAY=200
DHDCFLAGS += -DWIFI_TURNOFF_DELAY=10
# AMPDU
DHDCFLAGS += -DSUPPORT_AMPDU_MPDU_CMD
DHDCFLAGS += -DMIRACAST_AMPDU_SIZE=8

# Static preallocated buffers
DHDCFLAGS += -DDHD_USE_STATIC_MEMDUMP

ifeq ($(CONFIG_BCMDHD_PCIE),y)
  DHDCFLAGS += -DDHD_USE_STATIC_CTRLBUF
  # Use coherent pool
  DHDCFLAGS += -DDHD_USE_COHERENT_MEM_FOR_RING

  # 4way handshake disconnection feature
  DHDCFLAGS += -DDHD_4WAYM4_FAIL_DISCONNECT

  # Enabling this flag will allocate the coherent memory
  # from Atomic pool. For 64 bit ARM arch the default
  # coherent memory pool size is 256K. This size has to be
  # increased to 4M if the DHD uses this feature. So if the
  # BSP cannot be modified DHD should not be allocating from
  # coherent pool.
  #DHDCFLAGS += -DDHD_ALLOC_COHERENT_MEM_FROM_ATOMIC_POOL
endif # CONFIG_BCMDHD_PCIE

# DMA64 suppports on 64bit Architecture
ifeq ($(CONFIG_64BIT),y)
  DHDCFLAGS := $(filter-out -DBCMDMA32,$(DHDCFLAGS))
  DHDCFLAGS += -DBCMDMA64OSL
endif # CONFIG_64BIT

# TDLS enable
DHDCFLAGS += -DWLTDLS -DWLTDLS_AUTO_ENABLE
# For TDLS tear down inactive time 40 sec
DHDCFLAGS += -DCUSTOM_TDLS_IDLE_MODE_SETTING=10000
# for TDLS RSSI HIGH for establishing TDLS link
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_HIGH=-80
# for TDLS RSSI HIGH for tearing down TDLS link
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_LOW=-85

# Roaming trigger
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-75
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=10
# Set PM 2 always regardless suspend/resume
DHDCFLAGS += -DSUPPORT_PM2_ONLY
# remove this if allow DHD driver manage PM automatically
DHDCFLAGS += -DDISABLE_AUTO_CONTROL_PM

# Vendor Extension
ifeq ($(call wlan_version_ge,$(BCM_KVERSIONSTRING),3.14.0),TRUE)
  DHDCFLAGS += -DWL_VENDOR_EXT_SUPPORT
  ifeq ($(CONFIG_BUILD_CFG80211),y)
  DHDCFLAGS += -DBCM_PRIV_CMD_SUPPORT
  DHDCFLAGS += -DWL_PRIV_CMD_OVER_NL80211
  endif # CONFIG_BUILD_CFG80211
endif # wlan_version_ge

# RSSI Monitor
DHDCFLAGS += -DRSSI_MONITOR_SUPPORT
# NDOffload
DHDCFLAGS += -DNDO_CONFIG_SUPPORT
DHDCFLAGS += -DIPV6_NDO_SUPPORT

# Early suspend
DHDCFLAGS += -DDHD_USE_EARLYSUSPEND

# For Scan result patch
DHDCFLAGS += -DESCAN_RESULT_PATCH
DHDCFLAGS += -DDUAL_ESCAN_RESULT_BUFFER

# CSI
DHDCFLAGS += -DCSI_SUPPORT

# SAR Tx power scenario
DHDCFLAGS += -DWL_SAR_TX_POWER
DHDCFLAGS += -DWL_SAR_TX_POWER_CONFIG

# OWE
DHDCFLAGS += -DWL_OWE

# For Static Buffer
ifeq ($(CONFIG_DHD_USE_STATIC_BUF),y)
  DHDCFLAGS += -DCONFIG_DHD_USE_STATIC_BUF
  DHDCFLAGS += -DENHANCED_STATIC_BUF
  DHDCFLAGS += -DSTATIC_WL_PRIV_STRUCT
#  DHDCFLAGS += -DWL_STATIC_IF
endif # CONFIG_DHD_USE_STATIC_BUF

# Ioctl timeout 5000ms
DHDCFLAGS += -DIOCTL_RESP_TIMEOUT=5000

# Prevent rx thread monopolize
DHDCFLAGS += -DWAIT_DEQUEUE

# idle count
DHDCFLAGS += -DDHD_USE_IDLECOUNT

# SKB TAILPAD to avoid out of boundary memory access
DHDCFLAGS += -DDHDENABLE_TAILPAD

# Disable FRAMEBURST on VSDB
DHDCFLAGS += -DDISABLE_FRAMEBURST_VSDB

# WPS
DHDCFLAGS += -DWL_WPS_SYNC -DBCMCRYPTO_COMPONENT

# Path name to store the FW Debug symbol files
DHDCFLAGS += -DPLATFORM_PATH="\"/data/vendor/conn/\""

##########################
# driver type
# m: module type driver
# y: built-in type driver
##########################
DRIVER_TYPE ?= $(CONFIG_BCMDHD)

##### Chip dependent feature
ifneq ($(filter $(CONFIG_CHIPSET), BCM43456 BCM43012),$(CONFIG_CHIPSET))
DHDCFLAGS += -DWL11AX -DWL_CAP_HE
DHDCFLAGS += -DWL_6G_BAND -DWL_SOFTAP_6G -DWL_NAN_6G -DWL_P2P_6G
endif
DHDCFLAGS += -DWL11AC

ifneq ($(filter BCM43, $(CONFIG_CHIPSET)),)
  # New Features
  DHDCFLAGS += -D$(CONFIG_CHIPSET)
endif # CONFIG_BCM43xx

ifneq ($(CONFIG_BCMDHD_SINGLE_CORE),y)
# Enable set rxf and dpc to different CPUCORE
DHDCFLAGS += -DCUSTOM_SET_CPUCORE
DHDCFLAGS += -DPRIMARY_CPUCORE=1
DHDCFLAGS += -DDPC_CPUCORE=5
DHDCFLAGS += -DRXF_CPUCORE=4
DHDCFLAGS += -DMAX_RETRY_SET_CPUCORE=10
ifneq ($(CONFIG_ARCH_MESON),)
  DHDCFLAGS += -DCUSTOM_DPC_CPUCORE=4
else
  DHDCFLAGS += -DCUSTOM_DPC_CPUCORE=0
endif
endif # CONFIG_BCMDHD_SINGLE_CORE

ifeq ($(CONFIG_BCMDHD_SDIO),y)
  # DHDCFLAGS += -DTOE
  DHDCFLAGS += -DBDC -DDHD_BCMEVENTS -DMMC_SDIO_ABORT
  DHDCFLAGS += -DBCMSDIO -DUSE_SDIOFIFO_IOVAR
  ifeq ($(CONFIG_BCMDHD_SDIO_SDSTD),y)
    DHDCFLAGS += -DBCMSDIOH_STD
    DHDCFLAGS += -DBCMSDIOH_STD_TUNING_WAR
  endif # CONFIG_BCMDHD_SDIO_SDSTD
  ifeq ($(CONFIG_BCMDHD_SDIO_SDMMC),y)
    DHDCFLAGS += -DBCMLXSDMMC
  endif # CONFIG_BCMDHD_SDIO_SDMMC
  DHDCFLAGS += -DPROP_TXSTATUS
  DHDCFLAGS += -DLIMIT_BORROW
  ifeq ($(CONFIG_BCMDHD_OOB),y)
    DHDCFLAGS += -DDHD_USE_HOST_WAKE
    DHDCFLAGS += -DOOB_INTR_ONLY
    DHDCFLAGS += -DHW_OOB
    ifeq ($(CONFIG_BCMDHD_DISABLE_WOWLAN),y)
      DHDCFLAGS += -DDISABLE_WOWLAN
    endif # CONFIG_BCMDHD_DISABLE_WOWLAN
  else
    ifeq ($(CONFIG_BCMDHD_SDIO_SDMMC),y)
      DHDCFLAGS += -DSDIO_ISR_THREAD
    endif # CONFIG_BCMDHD_SDIO_SDMMC
  endif # CONFIG_BCMDHD_OOB
  ifeq ($(CONFIG_BCMDHD_GKI),y)
    DHDCFLAGS += -DSKIP_REGON_GPIO
    DHDCFLAGS += -DGKI_NO_KERNEL_BUS_PATCH
  endif # CONFIG_BCMDHD_GKI
  DHDCFLAGS += -DSDTEST
  # tput enhancement
  DHDCFLAGS += -DCUSTOM_GLOM_SETTING=8 -DCUSTOM_RXCHAIN=1
  DHDCFLAGS += -DCUSTOM_SDIO_F2_BLKSIZE=256
  DHDCFLAGS += -DUSE_DYNAMIC_F2_BLKSIZE -DDYNAMIC_F2_BLKSIZE_FOR_NONLEGACY=128
  DHDCFLAGS += -DBCMSDIOH_TXGLOM
  DHDCFLAGS += -DCUSTOM_TXGLOM=$(CONFIG_BCMDHD_SDIO_TXGLOM_MANNER)
  DHDCFLAGS += -DRXFRAME_THREAD
  DHDCFLAGS += -DREPEAT_READFRAME
  DHDCFLAGS += -DCUSTOM_MAX_TXGLOM_SIZE=40

  DHDCFLAGS += -DWLFC_STATE_PREALLOC

  DHDCFLAGS += -DPROP_TXSTATUS_VSDB

  DHDCFLAGS += -DCUSTOM_DPC_PRIO_SETTING=98
  DHDCFLAGS += -DDHD_SDALIGN=4

  DHDCFLAGS += -DCUSTOM_SDIO_CREDTRIG=32
  DHDCFLAGS += -DCUSTOM_TXINRX_THRES=128
  DHDCFLAGS += -DDHD_TXBOUND=75
  DHDCFLAGS += -DDHD_TXMINMAX=75
  DHDCFLAGS += -DDHD_RXBOUND=512
  DHDCFLAGS += -DMAX_HDR_READ=256
  DHDCFLAGS += -DDHD_FIRSTREAD=256
  DHDCFLAGS += -DDISABLE_FLOW_CONTROL
  DHDCFLAGS += -DCUSTOM_DHD_WATCHDOG_MS=100
  DHDCFLAGS += -DFREE_PKT_IN_RXF
  DHDCFLAGS += -DDEBUG_SDIO_TRANSFER_TIME
endif # CONFIG_BCMDHD_SDIO

ifeq ($(CONFIG_BCMDHD_PCIE),y)
  DHDCFLAGS += -DPCIE_FULL_DONGLE -DBCMPCIE -DCUSTOM_DPC_PRIO_SETTING=-1
  # HEAP ASLR
  DHDCFLAGS += -DBCM_ASLR_HEAP

  DHDCFLAGS += -DMAX_CNTL_TX_TIMEOUT=1
  ifneq ($(filter y, $(CONFIG_ARCH_MSM) $(CONFIG_ARCH_QCOM)),)
    DHDCFLAGS += -DMSM_PCIE_LINKDOWN_RECOVERY
  endif # CONFIG_ARCH_MSM || CONFIG_ARCH_QCOM
  ifeq ($(CONFIG_DHD_USE_STATIC_BUF),y)
    DHDCFLAGS += -DDHD_USE_STATIC_IOCTLBUF
  endif

  ## OOB
  ifeq ($(CONFIG_BCMDHD_OOB),y)
    DHDCFLAGS += -DDHD_USE_HOST_WAKE
    DHDCFLAGS += -DBCMPCIE_OOB_HOST_WAKE
    DHDCFLAGS += -DDHD_USE_PCIE_OOB_THREADED_IRQ
  endif # CONFIG_BCMDHD_OOB

  # Enable health check event handling
  DHDCFLAGS += -DDNGL_EVENT_SUPPORT
  DHDCFLAGS += -DHCHK_COMMON_SW_EVENT
endif # CONFIG_BCMDHD_PCIE

# TCP ACK suppress
DHDCFLAGS += -DDHDTCPACK_SUPPRESS
# Expand TCP tx queue
DHDCFLAGS += -DDHD_TCP_PACING_SHIFT
# TCP auto adjust TCP win-size
DHDCFLAGS += -DDHD_TCP_WINSIZE_ADJUST
# Expand TCP tx queue to 10 times of default size
DHDCFLAGS += -DTSQ_MULTIPLIER=10

# prioritize 802.1x packet
DHDCFLAGS += -DEAPOL_PKT_PRIO

# Configuration of default GPIOs when device tree is not used
ifneq ($(CONFIG_WLAN_REG_ON_GPIO),)
DHDCFLAGS += -DWLAN_REG_ON_GPIO_DEFAULT=$(CONFIG_WLAN_REG_ON_GPIO)
endif
ifneq ($(CONFIG_WLAN_HOST_WAKE_GPIO),)
DHDCFLAGS += -DWLAN_HOST_WAKE_GPIO_DEFAULT=$(CONFIG_WLAN_HOST_WAKE_GPIO)
endif


ifeq ($(CONFIG_BCMDHD),y) # for builtin DHD driver
  DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD
  DHDCFLAGS += -DWAKEUP_KSOFTIRQD_POST_NAPI_SCHEDULE
  DHDCFLAGS += -DUSE_LATE_INITCALL_SYNC
  # Use kernel strlcpy() implementation instead of one, defined in bcmstdlib_s.c
  DHDCFLAGS += -DBCM_USE_PLATFORM_STRLCPY
else # for module DHD driver
  DHDCFLAGS += -DBCMDHD_MODULAR
  DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD
endif # for builtin DHD driver

DHDOFILES := dhd_pno.o dhd_common.o dhd_ip.o dhd_custom_gpio.o \
    dhd_linux.o dhd_linux_sched.o dhd_linux_wq.o aiutils.o \
    bcmevent.o bcmutils.o bcmwifi_channels.o hndpmu.o linux_osl.o linux_pkt.o \
    sbutils.o siutils.o  dhd_linux_exportfs.o wl_roam.o \
    wldev_common.o wl_linux_mon.o dhd_linux_platdev.o \
    dhd_linux_pktdump.o wl_cfg_btcoex.o hnd_pktq.o \
    hnd_pktpool.o bcmxtlv.o bcm_app_utils.o dhd_debug.o dhd_csi.o \
    dhd_debug_linux.o dhd_mschdbg.o bcmbloom.o dhd_dbg_ring.o bcmstdlib_s.o frag.o \
    wl_cfgscan.o dhd_pktlog.o

DHDOFILES += $(BCMINTERNAL_DHDOFILES)

ifeq ($(CONFIG_BUILD_CFG80211),y)
  DHDOFILES += dhd_cfg80211.o wl_cfg80211.o wl_cfgvendor.o wl_cfgp2p.o wl_cfgnan.o wl_android.o
endif # CONFIG_BUILD_CFG80211

# extra Source files
DHDOFILES += wl_cfgvif.o wl_cfg_btcoex.o dhd_timesync.o

ifeq ($(CONFIG_ARCH_EXYNOS),y)
  DHDCFLAGS += -DBOARD_MODULAR_INIT
  DHDOFILES += dhd_custom_exynos.o
else ifeq ($(CONFIG_SOC_GOOGLE),y)
  DHDCFLAGS += -DBOARD_MODULAR_INIT
  DHDOFILES += dhd_custom_google.o
else ifeq ($(CONFIG_ARCH_HISI),y)
  DHDCFLAGS += -DDHD_DT_COMPAT_ENTRY=\"android,bcmdhd_wlan\"
  DHDCFLAGS += -DBOARD_MODULAR_INIT
  # Hikey sched is not so optimized and hence need a higher timeout
  DHDCFLAGS += -DWAIT_FOR_DISCONNECT_MAX=20
  DHDOFILES += dhd_custom_hikey.o
else ifneq ($(filter y, $(CONFIG_ARCH_MSM) $(CONFIG_ARCH_QCOM)),)
  DHDCFLAGS += -DBOARD_MODULAR_INIT
  DHDOFILES += dhd_custom_msm.o
  ifeq ($(CONFIG_ARCH_SDM660),y)
    DHDCFLAGS += -DCONFIG_ARCH_SDM660
  endif # CONFIG_ARCH_SDM660
else ifneq ($(CONFIG_ARCH_MESON),)
  DHDCFLAGS += -DBOARD_MODULAR_INIT
  DHDCFLAGS += -DEMPTY_CARD_DETECT -DRMMOD_CARD_DETECT
  DHDOFILES += dhd_custom_meson.o
  ### send up the event as skb packet and some WL command like 'escanresults' may need this
  DHDCFLAGS += -DWL_EVENT_ENAB
else ifneq ($(CONFIG_ARCH_DUMMY),)
  DHDCFLAGS += -DBOARD_MODULAR_INIT
  DHDOFILES += dhd_custom_dummy.o
else ifneq ($(filter y, $(CONFIG_ARCH_SYNA) $(CONFIG_ARCH_BERLIN) $(CONFIG_SYNA_CLK_BASE)),)
  DHDCFLAGS += -DBOARD_MODULAR_INIT
  ifeq ($(CONFIG_BCMDHD_PCIE),y)
    # VS640/VS680 doesn't provide PICE bus enumeration
    DHDCFLAGS += -DSKIP_REGON_GPIO
  else # CONFIG_BCMDHD_SDIO
  endif # CONFIG_BCMDHD_PCIE
  DHDOFILES += dhd_custom_default.o
else # Default customer platform
  DHDCFLAGS += -DBOARD_MODULAR_INIT
  DHDOFILES += dhd_custom_default.o
endif # CONFIG_ARCH_HISI
ifeq ($(CONFIG_BROADCOM_WIFI_RESERVED_MEM),y)
  DHDCFLAGS += -DCONFIG_BROADCOM_WIFI_RESERVED_MEM
  DHDOFILES += dhd_custom_memprealloc.o
endif # CONFIG_BROADCOM_WIFI_RESERVED_MEM

ifeq ($(CONFIG_BCMDHD_SDIO),y)
  DHDOFILES += bcmsdh.o bcmsdh_linux.o
  ifeq ($(CONFIG_BCMDHD_SDIO_SDMMC),y)
  DHDOFILES += bcmsdh_sdmmc.o bcmsdh_sdmmc_linux.o
  else ifeq ($(CONFIG_BCMDHD_SDIO_SDSTD),y)
  DHDOFILES += bcmsdstd.o bcmsdstd_linux.o
  endif # CONFIG_BCMDHD_SDIO_SDMMC
  DHDOFILES += dhd_cdc.o dhd_wlfc.o dhd_sdio.o
endif # CONFIG_BCMDHD_SDIO

ifeq ($(CONFIG_BCMDHD_PCIE),y)
  DHDOFILES += dhd_pcie.o dhd_pcie_linux.o dhd_msgbuf.o dhd_flowring.o
  DHDOFILES += pcie_core.o
endif # CONFIG_BCMDHD_PCIE

ifneq ($(filter -DDHD_LB, $(DHDCFLAGS)),)
DHDOFILES += dhd_linux_lb.o
endif # DHD_LB

ifneq ($(filter -DDHD_EVENT_LOG_FILTER, $(DHDCFLAGS)),)
DHDOFILES += dhd_event_log_filter.o
endif # DHD_EVENT_LOG_FILTER

ifneq ($(filter -DREAD_CONFIG_FROM_FILE, $(DHDCFLAGS)),)
  DHDOFILES += dhd_config.o
endif # READ_CONFIG_FROM_FILE

################################################################################
# Customization for Samsung
CONFIG_BCM43012 ?= y
CONFIG_CHIPSET := BCM43012
#DHDCFLAGS += -D$(CONFIG_CHIPSET)_CHIP

DHDCFLAGS += -DCUSTOMER_HW4 -DCUSTOMER_HW4_DEBUG -DCUSTOMER_HW4_PRIVATE_CMD
# Use default (/vendor/firmware/) prefix of file paths
# DHDCFLAGS += -DVENDOR_PATH="\"\""
# Control PM related value for MFG process(Read from file)
DHDCFLAGS += -DDHD_PM_CONTROL_FROM_FILE
# Export control file
DHDCFLAGS += -DDHD_EXPORT_CNTL_FILE
DHDCFLAGS += -DREAD_MACADDR
DHDCFLAGS += -DWRITE_WLANINFO
# Logging
# Debug
DHDCFLAGS += -DSIMPLE_MAC_PRINT
# Enable wakelock for legacy scan
DHDCFLAGS += -DDHD_USE_SCAN_WAKELOCK
# Enable wakelock debug function
DHDCFLAGS += -DDHD_TRACE_WAKE_LOCK
# Print out kernel panic point of file and line info when assertion happened
DHDCFLAGS += -DBCMASSERT_LOG
# Collect Current Assoc/Reject Status for Big Data logging
DHDCFLAGS += -DDHD_ENABLE_BIGDATA_LOGGING
# Collect roaming related information for Big Data logging
DHDCFLAGS += -DDHD_PUB_ROAM_EVT

# Enable SI utility error log
DHDCFLAGS += -DSI_ERROR_ENFORCE

# Print 8021X
DHDCFLAGS += -DDHD_8021X_DUMP

# Print DHCP
DHDCFLAGS += -DDHD_DHCP_DUMP

# Print ICMP
DHDCFLAGS += -DDHD_ICMP_DUMP

# Print ARP
DHDCFLAGS += -DDHD_ARP_DUMP

# Print DNS
DHDCFLAGS += -DDHD_DNS_DUMP

# Enable Firmware Coredump
DHDCFLAGS += -DDHD_FW_COREDUMP
DHDCFLAGS += -DDHD_FILE_DUMP_EVENT
ifeq ($(CONFIG_BCMDHD_SDIO),y)
  DHDCFLAGS += -DDEBUGABILITY
  DHDCFLAGS += -DDHD_DEBUGABILITY_LOG_DUMP_RING
endif # CONFIG_BCMDHD_SDIO

# Enable Log Dump
DHDCFLAGS += -DDHD_LOG_DUMP
DHDCFLAGS += -DDHD_PKT_LOGGING
ifeq ($(CONFIG_BCMDHD_LOG),y)
# CONTROL_LOGTRACE=1 to print as EL
  DHDCFLAGS += -DCUSTOM_CONTROL_LOGTRACE=1
endif
DHDCFLAGS += -DBCMASSERT_LOG

DHDCFLAGS += -DRSSI_MONITOR_SUPPORT
DHDCFLAGS += -DSUPPORT_RSSI_SUM_REPORT

ifneq ($(filter -DDHD_PKT_LOGGING,$(DHDCFLAGS)),)
	DHDOFILES += dhd_pktlog.o
ifneq ($(filter -DDHD_COMPACT_PKT_LOG,$(DHDCFLAGS)),)
DHDOFILES += bcm_l2_filter.o
endif
endif

DHDOFILES += dhd_custom_cis.o dhd_custom_sec.o

DHDCFLAGS += -DDHD_MAC_ADDR_EXPORT

# Prune scan results lower than -90 dB
DHDCFLAGS += -DDHD_PRUNE_SCAN_RSSI_LOWER=-90

# Limit 2.4GHz support to 20Mhz for coex
DHDCFLAGS += -DSUPPORT_2G_20M_ONLY
################################################################################

##### Finalize
ifeq ($(CONFIG_BCMDHD_SINGLE_CORE),y)
EXTRA_CFLAGS := $(DHDCFLAGS) -O3
else
EXTRA_CFLAGS := $(DHDCFLAGS) -g
endif # CONFIG_BCMDHD_SINGLE_CORE
EXTRA_CFLAGS += -DSRCBASE=\"$(BCMDHD_ROOT)\" -DDHD_COMPILED=\"$(BCMDHD_ROOT)\"
EXTRA_CFLAGS += -I$(BCMDHD_ROOT)/include -I$(BCMDHD_ROOT)
KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(CURDIR)
ifeq ($(call wlan_version_ge,$(GCCVER),4.9),TRUE)
KBUILD_CFLAGS += -Wno-date-time
endif # ($(call wlan_version_ge,$(GCCVER),4.9),TRUE)
ifeq ($(CONFIG_BCMDHD_SINGLE_CORE),y)
EXTRA_LDFLAGS := $(DHDLDFLAGS) -O3
else
EXTRA_LDFLAGS := $(DHDLDFLAGS) -g
endif # CONFIG_BCMDHD_SINGLE_CORE

${BCMDHD_MODULE_NAME}-objs := $(DHDOFILES)
obj-$(DRIVER_TYPE)   += ${BCMDHD_MODULE_NAME}.o
ccflags-y := $(KBUILD_CFLAGS)
ccflags-y += $(EXTRA_CFLAGS)

ifeq ($(call wlan_version_ge,$(BCM_KVERSIONSTRING),5.3.0),TRUE)
build_command=$(MAKE) -j8 --no-print-directory -C ${KERNEL_SRC} M=$(CURDIR) $(KBUILD_OPTIONS)
else # ($(call wlan_version_ge,$(BCM_KVERSIONSTRING),5.3.0),TRUE)
build_command=$(MAKE) -j8 --no-print-directory -C ${KERNEL_SRC} SUBDIRS=$(CURDIR) $(KBUILD_OPTIONS)
endif # ($(call wlan_version_ge,$(BCM_KVERSIONSTRING),5.3.0),TRUE)

all:
	@echo ARCH='$(ARCH)'
	@echo CROSS_COMPILE='$(CROSS_COMPILE)'
	@echo CC='$(CC)'
	@echo LINUXDIR='$(LINUXDIR)'
	@echo ${build_command} modules
	@${build_command} modules
ifeq ($(OUT_OF_TREE_BUILD),y)
	$(OBJDUMP) -d -S ${BCMDHD_MODULE_NAME}.ko > ${BCMDHD_MODULE_NAME}.debug.dis.src
	$(NM) -n --defined-only ${BCMDHD_MODULE_NAME}.ko > ${BCMDHD_MODULE_NAME}.ko.map
	$(STRIP) -g ${BCMDHD_MODULE_NAME}.ko -o ${BCMDHD_MODULE_NAME}.ko.stripped
	cp -rf ${BCMDHD_MODULE_NAME}.ko  ${BCMDHD_MODULE_NAME}.debug.ko
	cp -rf ${BCMDHD_MODULE_NAME}.ko.stripped  ${BCMDHD_MODULE_NAME}.ko
endif # OUT_OF_TREE_BUILD

install modules_install:
	@${build_command} modules_install

clean:
	@rm -rf *.o *.ko *.ko.* *.a *.mod *.mod.c *.dis* *~ .*.cmd *.o.cmd .*.o.cmd \
	Module.symvers modules.order .tmp_versions modules.builtin *.o.ur-safe .*.o.d *.o.ur-detected *.map
	@echo clean done!
