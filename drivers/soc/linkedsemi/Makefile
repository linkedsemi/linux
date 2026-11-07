# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_MCTP_PCIE_LINKEDSEMI) += linkedsemi-pcie-mctp.o
obj-$(CONFIG_ESPI_VW_LINKEDSEMI) += linkedsemi-espi-vw.o
obj-$(CONFIG_PCC_LINKEDSEMI) += linkedsemi-pcc.o
obj-$(CONFIG_MMBI_LINKEDSEMI) += linkedsemi-mmbi.o
obj-$(CONFIG_ESPI_MMBI_LINKEDSEMI) += linkedsemi-espi-mmbi.o
obj-$(CONFIG_PCIE_MMBI_LINKEDSEMI) += linkedsemi-pcie-mmbi.o

CFLAGS_linkedsemi-pcie-mctp.o := -I$(srctree)/drivers/pci/controller/dwc -I$(srctree)/drivers/dma/dw-edma
