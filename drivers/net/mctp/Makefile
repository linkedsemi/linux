obj-$(CONFIG_MCTP_SERIAL) += mctp-serial.o
obj-$(CONFIG_MCTP_TRANSPORT_I2C) += mctp-i2c.o
obj-$(CONFIG_MCTP_TRANSPORT_I3C) += mctp-i3c.o
obj-$(CONFIG_MCTP_TRANSPORT_USB) += mctp-usb.o
obj-$(CONFIG_MCTP_PCIE_LINKEDSEMI) += linkedsemi-pcie-mctp.o

CFLAGS_linkedsemi-pcie-mctp.o := -I$(srctree)/drivers/pci/controller/dwc -I$(srctree)/drivers/dma/dw-edma
