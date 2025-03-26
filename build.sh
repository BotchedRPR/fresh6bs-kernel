#!/bin/sh

export PLATFORM_VERSION=14
export ANDROID_MAJOR_VERSION=u
export TARGET_SOC=s5e5515
export ARCH=arm64

# igor - use gcc
export CROSS_COMPILE=aarch64-linux-gnu-

make s5e5515-fresh6bsue_defconfig
make -j$(nproc --all)
