#include <linux/module.h>
#include <linux/io.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/pci_ids.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include "pcie-linkedsemi.h"
#define XHCI_VENDOR_ID 0x2222
#define XHCI_DEVICE_ID 0x3333

struct pci_epf_xhci {
	struct pci_epf		*epf;
	u64 hc_addr_range;
	u64 *gdma_desc;
	phys_addr_t msi_mem_phy_addr;
	void *msi_mem;
};

static struct pci_epf_header xhci_header = {
	.vendorid	= XHCI_VENDOR_ID,
	.deviceid	= XHCI_DEVICE_ID,
	.baseclass_code = PCI_BASE_CLASS_SERIAL,
	.subclass_code = PCI_CLASS_SERIAL_USB & 0xff,
	.progif_code = PCI_CLASS_SERIAL_USB_XHCI & 0xff,
    .interrupt_pin	= PCI_INTERRUPT_UNKNOWN,
};

static void xhci_msi_en_callback(struct ls_pcie *pcie,u8 func_no,u8 en,void *param)
{
	struct pci_epf_xhci *epf_xhci = param;
	struct pci_epc *epc = pcie->pci.ep.epc;
	if(en)
	{
		u32 msi_addr_low = dw_pcie_readl_dbi(&pcie->pci,MSI_ADDR_LOW_OFFSET);
		u32 msi_addr_high = dw_pcie_readl_dbi(&pcie->pci,MSI_ADDR_HIGH_OFFSET);
		u64 msi_addr = (u64)msi_addr_high<<32 | msi_addr_low;
		u32 aligned_offset = msi_addr & (epc->mem->window.page_size - 1);
		msi_addr = ALIGN_DOWN(msi_addr, epc->mem->window.page_size);
		pci_epc_map_addr(epc,func_no,0,epf_xhci->msi_mem_phy_addr,msi_addr,epc->mem->window.page_size);
		u16 msi_data = dw_pcie_readw_dbi(&pcie->pci,MSI_DATA_OFFSET);
		*epf_xhci->gdma_desc = (epf_xhci->msi_mem_phy_addr + aligned_offset)<<32|msi_data;
		dev_info(pcie->pci.dev,"xhci msi addr %llx, data %x\n",(u64)msi_addr_high<<32 | msi_addr_low,msi_data);
	}else
	{
		dev_info(pcie->pci.dev,"xhci msi disable\n");
		pci_epc_unmap_addr(epc,func_no,0,epf_xhci->msi_mem_phy_addr);
	}
}

static int pci_epf_xhci_bind(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;
	struct pci_epf_header *header = epf->header;
	struct device *dev = &epf->dev;
	struct resource *res;
	// const struct pci_epc_features *epc_features;
	struct ls_pcie *ls_pcie = dev_get_drvdata(epc->dev.parent);
	struct platform_device *pdev = to_platform_device(ls_pcie->pci.dev);
	struct pci_epf_xhci *epf_xhci = epf_get_drvdata(epf);

	int ret;
    dev_info(dev, "pci_epf_xhci_bind called\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, epf->id->name);
	if(!res){
		dev_err(dev, "Failed to get \"%s\" resource\n", epf->id->name);
		return -ENODEV;
	}
	dev_info(dev, "\"%s\" resource start: 0x%llx, size: %llu,func_no: %d\n", 
		epf->id->name, res->start, resource_size(res), epf->func_no);
	
	ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no, header);
	if (ret) {
		dev_err(dev, "Configuration header write failed\n");
		return ret;
	}  

	ret = pci_epc_set_msi(epc, epf->func_no, epf->vfunc_no, 1);
	if (ret) {
		dev_err(dev, "Failed to set MSI\n");
		return ret;
	}

	epf->bar[BAR_0].size = resource_size(res);
	epf->bar[BAR_0].barno = BAR_0;
	epf->bar[BAR_0].flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	epf->bar[BAR_0].phys_addr = res->start;
	epf->bar[BAR_0].addr = 0;//ioremap(res->start, resource_size(res));

	// if (IS_ERR(epf->bar[BAR_0].addr)) {
	// 	dev_err(dev, "Failed to ioremap xhci memory\n");
	// 	return PTR_ERR(epf->bar[BAR_0].addr);
	// }

	ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no,&epf->bar[BAR_0]);
	if (ret) {
		dev_err(dev, "Failed to set xhci xhci BAR0\n");
		// iounmap(epf->bar[BAR_0].addr);
		return ret;
	}
	
	if(strcmp(epf->id->name, "xhci_hc") == 0){
		struct resource *xhci_hc_range = platform_get_resource_byname(pdev,IORESOURCE_MEM,"xhci_hc_range");
		epf_xhci->hc_addr_range = xhci_hc_range->start;
		struct dw_pcie_ob_atu_cfg atu_cfg = {
			.func_no = epf->func_no,
			.type = PCIE_ATU_TYPE_MEM,
			.parent_bus_addr = epf_xhci->hc_addr_range,
			.pci_addr = 0x0,
			.size = ls_pcie->pci.region_limit,
		};
		ret = ls_pcie_set_outbound_atu(ls_pcie, &atu_cfg);
		if(ret<0) {
			dev_err(dev, "Failed to set outbound ATU for xhci_hc\n");
			goto clear_bar;
		}
		ls_pcie_set_msi_en_callback(ls_pcie,epf->func_no,xhci_msi_en_callback,epf_xhci);
		epf_xhci->msi_mem = pci_epc_mem_alloc_addr(epc, &epf_xhci->msi_mem_phy_addr,
							epc->mem->window.page_size);
		dev_info(dev,"xhci msi mem base %llx\n",epf_xhci->msi_mem_phy_addr);
		if(IS_ERR(epf_xhci->msi_mem))
		{
			dev_err(dev, "Failed to alloc msi_mem\n");
			ret = PTR_ERR(epf_xhci->msi_mem);
			goto clear_outbound;
		}
		struct resource *xhci_msi = platform_get_resource_byname(pdev,IORESOURCE_MEM,"xhci_msi");
		epf_xhci->gdma_desc = devm_memremap(dev,xhci_msi->start,sizeof(*epf_xhci->gdma_desc),MEMREMAP_WT);
		BUG_ON(!epf_xhci->gdma_desc);
	}
	return 0;
clear_outbound:
	ls_pcie_clear_outbound_atu(ls_pcie,epf_xhci->hc_addr_range);
clear_bar:
	pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,&epf->bar[BAR_0]);
	return ret;
}

static void pci_epf_xhci_unbind(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;
	struct pci_epf_xhci *epf_xhci = epf_get_drvdata(epf);
	struct ls_pcie *ls_pcie = dev_get_drvdata(epc->dev.parent);
    dev_info(&epf->dev, "pci_epf_xhci_unbind called\n");
	if(strcmp(epf->id->name, "xhci_hc") == 0){
		pci_epc_mem_free_addr(epc,epf_xhci->msi_mem_phy_addr,epf_xhci->msi_mem,epc->mem->window.page_size);
		ls_pcie_clear_outbound_atu(ls_pcie,epf_xhci->hc_addr_range);
	}
	pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,&epf->bar[BAR_0]);
}


static const struct pci_epf_ops ops = {
	.unbind	= pci_epf_xhci_unbind,
	.bind	= pci_epf_xhci_bind,
};

static const struct pci_epf_device_id pci_epf_xhci_ids[] = {
	{
		.name = "xhci_vhub",
	},
	{
		.name = "xhci_hc",
	},
	{},
};

static int pci_epf_xhci_epc_init(struct pci_epf *epf)
{
	dev_info(&epf->dev, "pci_epf_xhci_epc_init called\n");
    return 0;
}

static void pci_epf_xhci_epc_deinit(struct pci_epf *epf)
{
	dev_info(&epf->dev, "pci_epf_xhci_epc_deinit called\n");
}

static int pci_epf_xhci_link_up(struct pci_epf *epf)
{
	dev_info(&epf->dev, "pci_epf_xhci_link_up called\n");
	return 0;
}

static int pci_epf_xhci_link_down(struct pci_epf *epf)
{
	dev_info(&epf->dev, "pci_epf_xhci_link_down called\n");
	return 0;
}

static const struct pci_epc_event_ops pci_epf_xhci_event_ops = {
	.epc_init = pci_epf_xhci_epc_init,
	.epc_deinit = pci_epf_xhci_epc_deinit,
	.link_up = pci_epf_xhci_link_up,
	.link_down = pci_epf_xhci_link_down,
};

static int pci_epf_xhci_probe(struct pci_epf *epf,
	const struct pci_epf_device_id *id)
{
	struct pci_epf_xhci *epf_xhci;
	struct device *dev = &epf->dev;
    dev_info(&epf->dev, "pci_epf_xhci_probe called\n");
	// dump_stack();
	epf_xhci = devm_kzalloc(dev, sizeof(*epf_xhci), GFP_KERNEL);
	if (!epf_xhci)
		return -ENOMEM;
    epf->header = &xhci_header;
    epf->event_ops = &pci_epf_xhci_event_ops;
	epf->id = id;
    epf_xhci->epf = epf;
    epf_set_drvdata(epf,epf_xhci);
	return 0;
}

static struct pci_epf_driver pci_epf_xhci_driver = {
	.driver.name	= "pci_epf_xhci",
	.probe		= pci_epf_xhci_probe,
	.id_table	= pci_epf_xhci_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_xhci_init(void)
{
	int ret;


	ret = pci_epf_register_driver(&pci_epf_xhci_driver);
	if (ret) {
		pr_err("Failed to register pci epf xhci driver --> %d\n", ret);
		return ret;
	}

	return 0;
}

module_init(pci_epf_xhci_init);

static void __exit pci_epf_xhci_exit(void)
{
	pci_epf_unregister_driver(&pci_epf_xhci_driver);
}
module_exit(pci_epf_xhci_exit);

MODULE_DESCRIPTION("PCI EPF VHUB DRIVER");
MODULE_LICENSE("GPL v2");
