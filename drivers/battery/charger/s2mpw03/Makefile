# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the kernel S2MPW03 charger driver.
#

# Object files in subdirectories

obj-$(CONFIG_CHARGER_S2MPW03)				+= s2mpw03-charger.o
s2mpw03-charger-$(CONFIG_CHARGER_S2MPW03)	+= s2mpw03_charger.o

obj-$(CONFIG_MUIC_S2MPW03)			+= s2mpw03-muic.o
s2mpw03-muic-$(CONFIG_MUIC_S2MPW03)	+= s2mpw03_muic.o

subdir-ccflags-y := -Wformat
