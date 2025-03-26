pablo-icpu-objs := pablo-icpu-core.o
pablo-icpu-objs += pablo-icpu-itf.o
pablo-icpu-objs += mbox/pablo-icpu-mbox.o
pablo-icpu-objs += pablo-icpu-selftest.o
pablo-icpu-objs += pablo-icpu-debug.o
pablo-icpu-objs += pablo-icpu-msg-queue.o
pablo-icpu-objs += pablo-icpu-firmware.o
pablo-icpu-objs += pablo-icpu-sysfs.o
pablo-icpu-objs += hardware/pablo-icpu-hw-itf.o

pablo-icpu-$(CONFIG_PABLO_ICPU_HW_V1_0) += hardware/pablo-icpu-v1_0.o

ifeq ($(CONFIG_PABLO_ICPU),y)
obj-m += pablo-icpu.o
endif

subdir-ccflags-$(CONFIG_PABLO_ICPU) := -I$(srctree)/drivers/media/platform/exynos/camera/icpu
subdir-ccflags-$(CONFIG_PABLO_ICPU) := -I$(srctree)/drivers/media/platform/exynos/camera/icpu/hardware
