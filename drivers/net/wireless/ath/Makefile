NOSTDINC_FLAGS := -I$(srctree)/include/compat/ \
        -include $(srctree)/include/compat/linux/compat-2.6.h \
        $(CFLAGS)

obj-$(CONFIG_ATH5K)		+= ath5k/
obj-$(CONFIG_ATH6KL)		+= ath6kl/
obj-$(CONFIG_ATH9K_HW)		+= ath9k/
obj-$(CONFIG_CARL9170)		+= carl9170/

ifneq ($(CONFIG_MACH_GOLFU),y)
obj-$(CONFIG_ATH_COMMON)	+= ath.o
endif

ath-objs :=	main.o \
		regd.o \
		hw.o \
		key.o

ath-$(CONFIG_ATH_DEBUG) += debug.o
