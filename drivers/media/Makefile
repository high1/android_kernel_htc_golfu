#
# Makefile for the kernel multimedia device drivers.
#
NOSTDINC_FLAGS := -I$(srctree)/include/compat/ \
        -include $(srctree)/include/compat/linux/compat-2.6.h \
        $(CFLAGS)

media-objs	:= media-device.o media-devnode.o media-entity.o

ifeq ($(CONFIG_MEDIA_CONTROLLER),y)
  obj-$(CONFIG_MEDIA_SUPPORT) += media.o
endif

obj-y += common/ rc/ video/

obj-$(CONFIG_VIDEO_DEV) += radio/
obj-$(CONFIG_DVB_CORE)  += dvb/
