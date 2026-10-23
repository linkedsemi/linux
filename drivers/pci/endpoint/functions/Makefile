# SPDX-License-Identifier: GPL-2.0
#
# Makefile for PCI Endpoint Functions
#

obj-$(CONFIG_PCI_EPF_TEST)		+= pci-epf-test.o
obj-$(CONFIG_PCI_EPF_NTB)		+= pci-epf-ntb.o
obj-$(CONFIG_PCI_EPF_VNTB) 		+= pci-epf-vntb.o
obj-$(CONFIG_PCI_EPF_MHI)		+= pci-epf-mhi.o
obj-$(CONFIG_PCI_EPF_VGA)		+= pci-epf-vga.o
obj-$(CONFIG_PCI_EPF_XHCI)		+= pci-epf-xhci.o

CFLAGS_pci-epf-vga.o	:= -I$(srctree)/drivers/pci/controller/dwc -I$(srctree)/drivers/dma/dw-edma
CFLAGS_pci-epf-xhci.o	:= -I$(srctree)/drivers/pci/controller/dwc -I$(srctree)/drivers/dma/dw-edma
