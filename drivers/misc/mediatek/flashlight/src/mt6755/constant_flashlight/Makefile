#
# Makefile for misc devices that really don't fit anywhere else.
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y				+= leds_strobe.o

ifeq ($(CONFIG_HQ_FLASHLIGHT_IC_LM3642), y)
ccflags-y += -DCONFIG_HQ_FLASHLIGHT_IC_LM3642
endif
