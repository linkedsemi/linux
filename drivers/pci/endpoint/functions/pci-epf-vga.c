#include <linux/module.h>
#include <linux/io.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/pci_ids.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include "pcie-linkedsemi.h"
#define VGA_VENDOR_ID 0x2222
#define VGA_DEVICE_ID 0x1111
#define FRAME_BUFFER_BAR BAR_0
#define VPN_BAR			BAR_2
#define DMA_BAR			BAR_3
#define DMA_BAR_SIZE 	0x1000
#define VBIOS_FIRMWARE_FILE_NAME "vbios_fw.bin"
struct pci_epf_vga{
	struct pci_epf		*epf;
	const struct firmware *vbios_fw;
	phys_addr_t vbios_dma_addr;
	phys_addr_t vbios_buf_size;
	void *vbios_buf;
};

static struct pci_epf_header vga_header = {
	.vendorid	= VGA_VENDOR_ID,
	.deviceid	= VGA_DEVICE_ID,
	.baseclass_code = PCI_BASE_CLASS_DISPLAY,
	.subclass_code = PCI_CLASS_DISPLAY_VGA & 0xff,
	.interrupt_pin	= PCI_INTERRUPT_UNKNOWN,
};

static int pci_epf_vga_epc_init(struct pci_epf *epf)
{
	dev_info(&epf->dev, "pci_epf_vga_epc_init called\n");
    return 0;
}

static void pci_epf_vga_epc_deinit(struct pci_epf *epf)
{
	dev_info(&epf->dev, "pci_epf_vga_epc_deinit called\n");
}

static int pci_epf_vga_link_up(struct pci_epf *epf)
{
	dev_info(&epf->dev, "pci_epf_vga_link_up called\n");
	return 0;
}

static int pci_epf_vga_link_down(struct pci_epf *epf)
{
	dev_info(&epf->dev, "pci_epf_vga_link_down called\n");
	return 0;
}

static const struct pci_epc_event_ops pci_epf_vga_event_ops = {
	.epc_init = pci_epf_vga_epc_init,
	.epc_deinit = pci_epf_vga_epc_deinit,
	.link_up = pci_epf_vga_link_up,
	.link_down = pci_epf_vga_link_down,
};

static int pci_epf_vga_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	struct pci_epf_vga *epf_vga;
	struct device *dev = &epf->dev;

	epf_vga = devm_kzalloc(dev, sizeof(*epf_vga), GFP_KERNEL);
	if (!epf_vga)
		return -ENOMEM;
    epf->header = &vga_header;
    epf->event_ops = &pci_epf_vga_event_ops;
    epf_vga->epf = epf;
    epf_set_drvdata(epf,epf_vga);
    return 0;
}

static void pci_epf_vga_unbind(struct pci_epf *epf)
{
	struct pci_epf_vga *epf_vga = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct ls_pcie *ls_pcie = dev_get_drvdata(epc->dev.parent);
	dev_info(&epf->dev, "pci_epf_vga_unbind called\n");
	ls_pcie_ep_clear_bar_raw(ls_pcie,epf->func_no,DMA_BAR);
	pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,&epf->bar[VPN_BAR]);
	pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,&epf->bar[FRAME_BUFFER_BAR]);
	release_firmware(epf_vga->vbios_fw);
	ls_pcie_ep_clear_rom_bar(ls_pcie,epf->func_no);
	memunmap(epf_vga->vbios_buf);
}

static int pci_epf_vga_bind(struct pci_epf *epf)
{
	struct pci_epf_vga *epf_vga = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	const struct pci_epc_features *epc_features;
	struct device *dev = &epf->dev;
	struct pci_epf_header *header = epf->header;
	struct ls_pcie *ls_pcie = dev_get_drvdata(epc->dev.parent);
	struct reserved_mem *rmem_vbios0 = ls_get_reserved_mem_by_name(ls_pcie->pci.dev,"vbios0");
	struct reserved_mem *rmem_vga0 = ls_get_reserved_mem_by_name(ls_pcie->pci.dev,"vga0");
	struct platform_device *pdev = to_platform_device(ls_pcie->pci.dev);
	struct resource *vpn_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vpn");
	int ret;
	dev_info(&epf->dev, "pci_epf_vga_bind called, func_no: %d\n", epf->func_no);
	if(epf->func_no)
	{
		dev_err(dev, "FAIL: allocated pci vga func_no is %d which must be 0!\n",epf->func_no);
		return -EINVAL;
	}
	if(!rmem_vbios0)
	{
		dev_err(dev, "Failed to get rmem_vbios0\n");
		return -EINVAL;
	}
	if(!rmem_vga0)
	{
		dev_err(dev, "Failed to get rmem_vga0\n");
		return -EINVAL;
	}
	ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no, header);
	if (ret) {
		dev_err(dev, "Configuration header write failed\n");
		return ret;
	}
	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	epf_vga->vbios_dma_addr = rmem_vbios0->base;
	epf_vga->vbios_buf_size = rmem_vbios0->size;
	epf_vga->vbios_buf = memremap(epf_vga->vbios_dma_addr, epf_vga->vbios_buf_size,MEMREMAP_WC);
	if(!epf_vga->vbios_buf)
	{
		dev_err(dev, "Failed to ioremap vbios buffer\n");
		return -ENOMEM;
	}
	dev_info(&epf->dev, "vbios buffer: 0x%px, dma addr: 0x%llx,size: 0x%llx\n", epf_vga->vbios_buf,epf_vga->vbios_dma_addr,epf_vga->vbios_buf_size);

	ret = ls_pcie_ep_set_rom_bar(ls_pcie,epf->func_no,epf_vga->vbios_dma_addr,epf_vga->vbios_buf_size);
	if(ret)
	{
		dev_err(dev, "Failed to set rom bar,%d\n", ret);
		goto mem_unmap;
	}

	ret = request_firmware_into_buf(&epf_vga->vbios_fw,VBIOS_FIRMWARE_FILE_NAME,ls_pcie->pci.dev,epf_vga->vbios_buf,epf_vga->vbios_buf_size);
	if(ret)
	{
        dev_err(dev, "Failed to load vbios firmware,%d\n", ret);
		goto clear_rom_bar;
	}

	epf->bar[FRAME_BUFFER_BAR].phys_addr = rmem_vga0->base;
	epf->bar[FRAME_BUFFER_BAR].size = rmem_vga0->size;
	epf->bar[FRAME_BUFFER_BAR].addr = NULL;
	epf->bar[FRAME_BUFFER_BAR].barno = FRAME_BUFFER_BAR;
	epf->bar[FRAME_BUFFER_BAR].flags = PCI_BASE_ADDRESS_MEM_TYPE_64;
	dev_info(dev, "fb_base phys addr: 0x%llx,size: 0x%lx\n", epf->bar[FRAME_BUFFER_BAR].phys_addr,epf->bar[FRAME_BUFFER_BAR].size);

	ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no,&epf->bar[FRAME_BUFFER_BAR]);
    if (ret) {
        dev_err(dev, "Failed to set BAR%d\n", FRAME_BUFFER_BAR);
		goto release_fw;
    }

	epf->bar[VPN_BAR].phys_addr = vpn_res->start;
	epf->bar[VPN_BAR].size = resource_size(vpn_res);
	epf->bar[VPN_BAR].addr = NULL;
	epf->bar[VPN_BAR].barno = VPN_BAR;
	epf->bar[VPN_BAR].flags = PCI_BASE_ADDRESS_MEM_TYPE_32;
	dev_info(dev, "vpn phys addr: 0x%llx,size: 0x%lx\n", epf->bar[VPN_BAR].phys_addr,epf->bar[VPN_BAR].size);
	ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no,&epf->bar[VPN_BAR]);
    if (ret) {
        dev_err(dev, "Failed to set BAR%d\n", VPN_BAR);
		goto claer_fb_bar;
    }
	ls_pcie_ep_set_bar_raw(ls_pcie,epf->func_no,DMA_BAR,DMA_BAR_SIZE,PCI_BASE_ADDRESS_MEM_TYPE_32);
    return ret;
claer_fb_bar:
	pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,&epf->bar[FRAME_BUFFER_BAR]);
release_fw:
	release_firmware(epf_vga->vbios_fw);
clear_rom_bar:
	ls_pcie_ep_clear_rom_bar(ls_pcie,epf->func_no);
mem_unmap:
	memunmap(epf_vga->vbios_buf);
	return ret;
}

static const struct pci_epf_ops ops = {
	.unbind	= pci_epf_vga_unbind,
	.bind	= pci_epf_vga_bind,
};

static const struct pci_epf_device_id pci_epf_vga_ids[] = {
	{
		.name = "pci_epf_vga",
	},
	{},
};

static struct pci_epf_driver pci_epf_vga_driver = {
	.driver.name	= "pci_epf_vga",
	.probe		= pci_epf_vga_probe,
	.id_table	= pci_epf_vga_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_vga_init(void)
{
	int ret;


	ret = pci_epf_register_driver(&pci_epf_vga_driver);
	if (ret) {
		pr_err("Failed to register pci epf test driver --> %d\n", ret);
		return ret;
	}

	return 0;
}

module_init(pci_epf_vga_init);

static void __exit pci_epf_vga_exit(void)
{
	pci_epf_unregister_driver(&pci_epf_vga_driver);
}
module_exit(pci_epf_vga_exit);

MODULE_DESCRIPTION("PCI EPF VGA DRIVER");
MODULE_LICENSE("GPL v2");
