# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_USB_LINKEDSEMI_PCIE_VUSB) += pcie-vusb.o

pcie-vusb-y := linkedsemi-pcie-vusb.o trace.o

CFLAGS_linkedsemi-pcie-vusb.o := -I$(srctree)/drivers/pci/controller/dwc -I$(srctree)/drivers/dma/dw-edma
CFLAGS_trace.o := -I$(src)
