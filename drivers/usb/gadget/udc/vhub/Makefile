# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_USB_LINKEDSEMI_VHUB) += linkedsemi-vhub.o

linkedsemi-vhub-y := vhub.o dev.o ep_pool.o ep0.o epn.o debugfs.o trace.o

CFLAGS_trace.o := -I$(src)
