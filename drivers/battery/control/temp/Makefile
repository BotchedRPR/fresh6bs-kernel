# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel bc temp modules.
#

# Object files in subdirectories

obj-$(CONFIG_SB_TEMP)			+= sb-temp-base.o
sb-temp-base-$(CONFIG_SB_TEMP)	+= sb_temp_base.o

obj-$(CONFIG_SB_FORCED_EPT) 			+= sb-forced-ept.o
sb-forced-ept-$(CONFIG_SB_FORCED_EPT)	+= sb_forced_ept.o

obj-$(CONFIG_SB_TEMP_W6)			+= sb-temp-watch6.o
sb-temp-watch6-$(CONFIG_SB_TEMP_W6)	+= sb_temp_watch6.o

subdir-ccflags-y := -Wformat
