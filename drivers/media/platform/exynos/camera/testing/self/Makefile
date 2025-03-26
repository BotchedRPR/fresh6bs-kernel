pablo-self-tests-objs := pablo-self-test.o
pablo-self-tests-objs += pablo-self-test-hw-ip.o
pablo-self-tests-objs += pablo-power-self-test.o

pablo-self-tests-$(CONFIG_PABLO_V11_0_0)+= pablo-hw-byrp-self-test.o \
					pablo-hw-rgbp-self-test.o \
					pablo-hw-yuvp-self-test.o \
					pablo-hw-mcsc-self-test.o \
					pablo-hw-mcfp-self-test.o

obj-$(CONFIG_PABLO_SELF_TEST) += pablo-self-tests.o
