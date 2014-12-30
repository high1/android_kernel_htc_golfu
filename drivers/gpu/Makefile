NOSTDINC_FLAGS := -I$(srctree)/include/compat/ \
        -include $(srctree)/include/compat/linux/compat-2.6.h \
        $(CFLAGS)

obj-y			+= drm/ vga/ stub/ ion/
obj-$(CONFIG_MSM_KGSL)	+= msm/
