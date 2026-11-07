// SPDX-License-Identifier: GPL-2.0
/*
 * Linkedsemi LS2010 PCIe MMBI endpoint function driver
 *
 * Implements the MMBI BMC side as a PCI endpoint function, modeled
 * after pci-epf-xhci.c.  The EPF is instantiated through configfs:
 *
 *   # mount -t configfs none /sys/kernel/config
 *   # cd /sys/kernel/config/pci_ep
 *   # mkdir functions/ls_pcie_mmbi
 *   # ln -s functions/ls_pcie_mmbi controllers/<ls pcie ep>/
 *
 * It exposes the MMBI region as BAR0 and the doorbell page as BAR1.
 * The BAR0 space is a no-map reserved-memory region referenced by a
 * child node of the endpoint controller's device tree node matched by
 * compatible ("linkedsemi,ls-pcie-mmbi", no dynamic allocation):
 *
 *   - BMC -> host interrupts are reported with pci_epc_raise_irq()
 *     (MSI) after the BMC advances its B2H write pointer.
 *   - host -> BMC interrupts are delivered as a local doorbell
 *     interrupt: the host writes mmbi-intr-value to the mmbi-intr-addr
 *     doorbell address and the SoC raises the local interrupt declared
 *     in the child node.  BAR1 maps the EPC page containing
 *     mmbi-intr-addr (aligned down to the EPC page size); the driver
 *     publishes the doorbell offset within that page (bmc_int_location
 *     = mmbi-intr-addr - BAR1 base) and the write value
 *     (bmc_int_value) in the MMBI descriptor.  The doorbell behavior
 *     itself is a SoC property:
 *
 *       reserved-memory {
 *           mmbi_bar: mmbi-bar@d2000000 {
 *               reg = <0x0 0xd2000000 0x0 0x10000>;
 *               no-map;
 *           };
 *       };
 *
 *       ls_mmbi: mmbi {
 *           compatible = "linkedsemi,ls-pcie-mmbi";
 *           memory-region = <&mmbi_bar>;  // BAR0 space, no dynamic alloc
 *           interrupts = <NN 4>;          // local doorbell interrupt
 *           mmbi-intr-addr = <0xd2000010>; // doorbell address; BAR1
 *                                        // maps its EPC page
 *           mmbi-intr-value = <0x1>;     // value the host writes to the
 *                                        // doorbell (bmc_int_value)
 *           status = "okay";
 *       };
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_ids.h>
#include <linux/platform_device.h>

#include "linkedsemi-mmbi.h"

#define LS_MMBI_PCIE_DRIVER_NAME	"linkedsemi-pcie-mmbi"
#define LS_MMBI_PCIE_EPF_NAME		"ls_pcie_mmbi"
#define LS_MMBI_PCIE_DT_COMPAT		"linkedsemi,ls-pcie-mmbi"
#define LS_MMBI_PCIE_VENDOR_ID		0x2222
#define LS_MMBI_PCIE_DEVICE_ID		0x4444

/* MMBI class code, same family as the Aspeed MMBI function (0x0C0C00) */
#define LS_MMBI_PCIE_SUBCLASS		0x0c

struct ls_mmbi_pcie {
	struct pci_epf *epf;
	struct ls_mmbi_ins ins;
	int irq;
};

static struct pci_epf_header ls_mmbi_pcie_header = {
	.vendorid = LS_MMBI_PCIE_VENDOR_ID,
	.deviceid = LS_MMBI_PCIE_DEVICE_ID,
	.baseclass_code = PCI_BASE_CLASS_SERIAL,
	.subclass_code = LS_MMBI_PCIE_SUBCLASS,
	.progif_code = 0x00,
	.interrupt_pin = PCI_INTERRUPT_UNKNOWN,
};

static int ls_mmbi_pcie_raise_host_int(struct ls_mmbi_ins *ins)
{
	struct ls_mmbi_pcie *priv = container_of(ins, struct ls_mmbi_pcie, ins);

	return pci_epc_raise_irq(priv->epf->epc, priv->epf->func_no,
				 priv->epf->vfunc_no, PCI_IRQ_MSI,1);
}

/* Host wrote the BAR: refresh the channel state and wake pollers */
static irqreturn_t ls_mmbi_pcie_intr_handler(int irq, void *arg)
{
	struct ls_mmbi_pcie *priv = arg;

	ls_mmbi_host_write_notify(&priv->ins);
	return IRQ_HANDLED;
}

/*
 * Set up the BARs from the "ls_mmbi" child node.  Nothing is allocated
 * at runtime.
 *
 *   - BAR0: the no-map reserved-memory region referenced by
 *     "memory-region", mapped for BMC-side MMBI access (ins->virt).
 *   - BAR1: the EPC page containing the mmbi-intr-addr doorbell
 *     address (aligned down to the EPC page size).  The host
 *     interrupts the BMC by writing mmbi-intr-value to that address;
 *     the doorbell behavior itself is a SoC property.  The doorbell
 *     offset within the BAR1 page and the write value are published
 *     as bmc_int_location / bmc_int_value in the MMBI descriptor.
 */
static int ls_mmbi_pcie_bar_setup(struct ls_mmbi_pcie *priv,
				  struct device_node *child)
{
	struct pci_epf *epf = priv->epf;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct ls_mmbi_ins *ins = &priv->ins;
	struct pci_epf_bar *bar0 = &epf->bar[BAR_0];
	struct pci_epf_bar *bar1 = &epf->bar[BAR_1];
	u64 page_size = epc->mem->window.page_size;
	u64 bar1_phys, intr_off;
	struct device_node *np;
	struct resource res;
	u32 intr_addr, intr_value;
	int ret;

	/* BAR0: MMBI region */
	np = of_parse_phandle(child, "memory-region", 0);
	if (!np) {
		dev_err(dev, "missing 'memory-region' in %pOF\n", child);
		return -EINVAL;
	}

	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret) {
		dev_err(dev, "failed to get MMBI BAR resource: %d\n", ret);
		return ret;
	}

	if (resource_size(&res) < LS_MMBI_MIN_SIZE ||
	    (resource_size(&res) & 7)) {
		dev_err(dev, "invalid MMBI BAR size %pa\n", &res);
		return -EINVAL;
	}

	bar0->addr = ioremap(res.start, resource_size(&res));
	if (!bar0->addr)
		return -ENOMEM;

	bar0->phys_addr = res.start;
	bar0->size = resource_size(&res);
	bar0->barno = BAR_0;

	ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no, bar0);
	if (ret) {
		dev_err(dev, "failed to set BAR0: %d\n", ret);
		iounmap(bar0->addr);
		bar0->addr = NULL;
		return ret;
	}

	ins->virt = bar0->addr;
	ins->size = bar0->size;

	/* BAR1: EPC page containing the doorbell address */
	ret = of_property_read_u32(child, "mmbi-intr-addr", &intr_addr);
	if (ret) {
		dev_err(dev, "missing 'mmbi-intr-addr'\n");
		goto err_clear_bar0;
	}

	ret = of_property_read_u32(child, "mmbi-intr-value", &intr_value);
	if (ret) {
		dev_err(dev, "missing 'mmbi-intr-value'\n");
		goto err_clear_bar0;
	}

	if (intr_addr & 3) {
		dev_err(dev, "invalid 'mmbi-intr-addr' %#x\n", intr_addr);
		ret = -EINVAL;
		goto err_clear_bar0;
	}

	bar1_phys = ALIGN_DOWN(intr_addr, page_size);
	intr_off = intr_addr - bar1_phys;
	if (intr_off > U32_MAX) {
		dev_err(dev, "doorbell offset %#llx too large\n", intr_off);
		ret = -EINVAL;
		goto err_clear_bar0;
	}

	bar1->phys_addr = bar1_phys;
	bar1->size = page_size;
	bar1->barno = BAR_1;

	ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no, bar1);
	if (ret) {
		dev_err(dev, "failed to set BAR1: %d\n", ret);
		goto err_clear_bar0;
	}

	ins->bmc_int_location = intr_off;
	ins->bmc_int_value = intr_value;

	dev_info(dev,
		 "doorbell: addr %#x, BAR1 %#llx+%#llx, offset %#x, value %#x\n",
		 intr_addr, bar1_phys, bar1->size, ins->bmc_int_location,
		 ins->bmc_int_value);

	return 0;

err_clear_bar0:
	pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no, bar0);
	iounmap(bar0->addr);
	bar0->addr = NULL;
	ins->virt = NULL;
	return ret;
}

/*
 * Request the local doorbell interrupt declared by the child node's
 * "interrupts" property; it fires when the host writes the doorbell.
 */
static int ls_mmbi_pcie_intr_setup(struct ls_mmbi_pcie *priv,
				   struct device_node *child)
{
	struct device *dev = &priv->epf->dev;
	int ret, irq;

	irq = of_irq_get(child, 0);
	if (irq < 0) {
		dev_err(dev, "failed to get MMBI doorbell interrupt: %d\n",
			irq);
		return irq;
	}

	ret = devm_request_irq(dev, irq, ls_mmbi_pcie_intr_handler, 0,
			       LS_MMBI_PCIE_DRIVER_NAME, priv);
	if (ret) {
		dev_err(dev, "failed to request MMBI doorbell interrupt: %d\n",
			ret);
		return ret;
	}
	priv->irq = irq;

	dev_info(dev, "doorbell interrupt %d requested\n", irq);

	return 0;
}

static int ls_mmbi_pcie_bind(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct ls_mmbi_pcie *priv = epf_get_drvdata(epf);
	struct pci_epf_bar *bar0 = &epf->bar[BAR_0];
	struct pci_epf_bar *bar1 = &epf->bar[BAR_1];
	struct platform_device *pdev;
	struct device_node *child;
	int ret;

	/* The EPF is DT-configured through a child of the EPC node,
	 * matched by compatible.  The node defaults to disabled; boards
	 * must set status = "okay" to enable the function.
	 */
	pdev = to_platform_device(epc->dev.parent);
	child = of_get_compatible_child(pdev->dev.of_node,
					LS_MMBI_PCIE_DT_COMPAT);
	if (!child) {
		dev_err(dev, "no '%s' child node in %pOF\n",
			LS_MMBI_PCIE_DT_COMPAT, pdev->dev.of_node);
		return -ENODEV;
	}

	if (!of_device_is_available(child)) {
		of_node_put(child);
		return -ENODEV;
	}

	ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no,
				   epf->header);
	if (ret) {
		dev_err(dev, "failed to write configuration header: %d\n",
			ret);
		goto out_put;
	}

	ret = pci_epc_set_msi(epc, epf->func_no, epf->vfunc_no, 1);
	if (ret) {
		dev_err(dev, "failed to set MSI: %d\n", ret);
		goto out_put;
	}

	/* BAR0/BAR1 plus the doorbell descriptor fields */
	ret = ls_mmbi_pcie_bar_setup(priv, child);
	if (ret)
		goto out_put;

	/* Local doorbell interrupt */
	ret = ls_mmbi_pcie_intr_setup(priv, child);
	if (ret)
		goto err_clear_bars;

	priv->ins.dev = dev;
	priv->ins.num_of_channels = 1;
	priv->ins.host_int_type = LS_MMBI_HOST_INT_PCIE;
	priv->ins.host_int_location = 1;	/* MSI number */
	priv->ins.host_int_value = 0;
	/* bmc_int_type = doorbell in the BAR space; bmc_int_location and
	 * bmc_int_value were filled from the DT by bar_setup above.
	 */
	priv->ins.bmc_int_type = 1;
	priv->ins.host_int_raise = ls_mmbi_pcie_raise_host_int;

	ret = ls_mmbi_init(&priv->ins);
	if (ret) {
		dev_err(dev, "failed to initialize MMBI instance: %d\n", ret);
		goto err_intr_teardown;
	}

	of_node_put(child);
	dev_info(dev, "MMBI endpoint function bound (BAR0 %zu bytes)\n",
		 bar0->size);

	return 0;

err_intr_teardown:
	devm_free_irq(dev, priv->irq, priv);
err_clear_bars:
	if (bar1->phys_addr) {
		pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no, bar1);
		bar1->phys_addr = 0;
	}
	pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no, bar0);
	iounmap(bar0->addr);
	bar0->addr = NULL;
	priv->ins.virt = NULL;
out_put:
	of_node_put(child);
	return ret;
}

static void ls_mmbi_pcie_unbind(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;
	struct ls_mmbi_pcie *priv = epf_get_drvdata(epf);
	struct pci_epf_bar *bar0 = &epf->bar[BAR_0];
	struct pci_epf_bar *bar1 = &epf->bar[BAR_1];

	ls_mmbi_remove(&priv->ins);

	/* The epf device may outlive this binding; drop the devm resource
	 * so a re-bind does not double-request the doorbell interrupt.
	 */
	devm_free_irq(&epf->dev, priv->irq, priv);

	if (bar1->phys_addr) {
		pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no, bar1);
		bar1->phys_addr = 0;
	}

	pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no, bar0);
	if (bar0->addr) {
		iounmap(bar0->addr);
		bar0->addr = NULL;
	}
}

static int ls_mmbi_pcie_link_up(struct pci_epf *epf)
{
	dev_info(&epf->dev, "MMBI endpoint link up\n");
	return 0;
}

static int ls_mmbi_pcie_link_down(struct pci_epf *epf)
{
	dev_info(&epf->dev, "MMBI endpoint link down\n");
	return 0;
}

static const struct pci_epc_event_ops ls_mmbi_pcie_event_ops = {
	.link_up = ls_mmbi_pcie_link_up,
	.link_down = ls_mmbi_pcie_link_down,
};

static int ls_mmbi_pcie_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	struct ls_mmbi_pcie *priv;
	struct device *dev = &epf->dev;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->epf = epf;
	epf->header = &ls_mmbi_pcie_header;
	epf->event_ops = &ls_mmbi_pcie_event_ops;
	epf_set_drvdata(epf, priv);

	return 0;
}

static const struct pci_epf_ops ls_mmbi_pcie_ops = {
	.bind = ls_mmbi_pcie_bind,
	.unbind = ls_mmbi_pcie_unbind,
};

static const struct pci_epf_device_id ls_mmbi_pcie_ids[] = {
	{
		.name = LS_MMBI_PCIE_EPF_NAME,
	},
	{},
};

static struct pci_epf_driver ls_mmbi_pcie_driver = {
	.driver.name = LS_MMBI_PCIE_DRIVER_NAME,
	.probe = ls_mmbi_pcie_probe,
	.id_table = ls_mmbi_pcie_ids,
	.ops = &ls_mmbi_pcie_ops,
	.owner = THIS_MODULE,
};

static int __init ls_mmbi_pcie_init(void)
{
	int ret;

	ret = pci_epf_register_driver(&ls_mmbi_pcie_driver);
	if (ret)
		pr_err("failed to register LS2010 PCIe MMBI EPF driver: %d\n",
		       ret);

	return ret;
}
module_init(ls_mmbi_pcie_init);

static void __exit ls_mmbi_pcie_exit(void)
{
	pci_epf_unregister_driver(&ls_mmbi_pcie_driver);
}
module_exit(ls_mmbi_pcie_exit);

MODULE_DESCRIPTION("Linkedsemi LS2010 PCIe MMBI endpoint function");
MODULE_LICENSE("GPL");
