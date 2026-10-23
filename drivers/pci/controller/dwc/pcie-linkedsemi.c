#include "pcie-linkedsemi.h"

struct ls_pcie_of_data {
    enum dw_pcie_device_mode mode;
};
#define PIPE_CFG0_CFG_MSI_EN_OFFSET     0x18
#define     CFG_MSI_EN_POS              5
#define     CFG_MSI_EN_MASK             GENMASK(12,5)

#define PIPE_CFG1_INT_CLR1_OFFSET       0x0         //0xba0
#define     RADM_CORRECTABLE_ERR_CLR    BIT(16)
#define     RADM_NONFATAL_ERR_CLR       BIT(17)
#define     RADM_FATAL_ERR_CLR          BIT(18)

#define PIPE_CFG1_INT_CLR2_OFFSET       0xc         //0xbac
#define     ASSERT_INTA_GRT_CLR     BIT(2)
#define     ASSERT_INTB_GRT_CLR     BIT(3)
#define     ASSERT_INTC_GRT_CLR     BIT(4)
#define     ASSERT_INTD_GRT_CLR     BIT(5)

#define PIPE_CFG1_IP_INT_MASK_OFFSET    0x1c        //0xbbc
#define     SMLH_LINK_UP            BIT(0)

#define PIPE_CFG1_IP_INT_STAT_OFFSET    0x2c
#define     RADM_CORRECTABLE_ERR    BIT(15)
#define     RADM_NONFATAL_ERR       BIT(16)
#define     RADM_FATAL_ERR          BIT(17)
#define     ASSERT_INTA_GRT         BIT(18) 
#define     ASSERT_INTB_GRT         BIT(19) 
#define     ASSERT_INTC_GRT         BIT(20) 
#define     ASSERT_INTD_GRT         BIT(21) 
#define     RADM_INTA_ASSERTED      BIT(22)
#define     RADM_INTB_ASSERTED      BIT(23)
#define     RADM_INTC_ASSERTED      BIT(24)
#define     RADM_INTD_ASSERTED      BIT(25)

#define PIPE_CFG2_RADM_TRGT1_ATU_CBUF_ERR_INT_STAT_OFFSET 0xc   //0xc9c
#define PIPE_CFG2_CFG_SYS_ERR_RC_INT_STAT_OFFSET 0x10           //0xca0
#define PIPE_CFG2_CFG_MSI_EN_INT_OFFSET 0x18                    //0xca8
#define     CFG_MSI_EN_INT_MASK_MASK GENMASK(7,0)
#define     CFG_MSI_EN_INT_RAW_MASK  GENMASK(15,8)
#define     CFG_MSI_EN_INT_STAT_POS  16
#define     CFG_MSI_EN_INT_STAT_MASK GENMASK(23,16)
#define PIPE_CFG2_CFG_MSI_EN_INT_CLR_OFFSET 0x1c                //0xcac
#define PIPE_CFG2_RADM_PM_INT_OFFSET            0x20            //0xcb0
#define     LINK_REQ_RST_NOT_INT_MASK   BIT(0)
#define     RADM_PM_PME_INT_MASK        BIT(1)
#define     RADM_PM_TO_ACK_MASK         BIT(2)
#define     RADM_PM_TURNOFF_MASK        BIT(3)
#define     LINK_REQ_RST_NOT_INT_RAW    BIT(4)
#define     RADM_PM_PME_INT_RAW         BIT(5)
#define     RADM_PM_TO_ACK_RAW          BIT(6)
#define     RADM_PM_TURNOFF_RAW         BIT(7)
#define     LINK_REQ_RST_NOT_INT_STAT    BIT(8)
#define     RADM_PM_PME_INT_STAT         BIT(9)
#define     RADM_PM_TO_ACK_STAT          BIT(10)
#define     RADM_PM_TURNOFF_STAT         BIT(11)
#define PIPE_CFG2_RADM_PM_INT_CLR_OFFSET        0x24
#define     LINK_REQ_RST_NOT_INT_CLR   BIT(0)
#define     RADM_PM_PME_INT_CLR        BIT(1)
#define     RADM_PM_TO_ACK_CLR         BIT(2)
#define     RADM_PM_TURNOFF_CLR        BIT(3)
#define PIPE_CFG2_MISC_INT_OFFSET               0x2c

#define EP_FUNC_REG_OFFSET 0x10000
#define NUM_DMA_WR_CHAN 8
#define NUM_DMA_RD_CHAN 8
#define ATU_ROM_BAR_IDX 6

static irqreturn_t ls_pcie_misc_irq_thread(int irq,void *arg)
{
    struct ls_pcie *ls_pcie = arg;
    int err;
    u32 cfg_msi_en;
    err = regmap_read(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg0 + PIPE_CFG0_CFG_MSI_EN_OFFSET,&cfg_msi_en);
    BUG_ON(err);
    dev_info(ls_pcie->pci.dev,"cfg_msi_en %x,int status %x\n",cfg_msi_en,ls_pcie->cfg_msi_en_int_status);
    for(u8 i=0;i<LS_PCIE_MAX_FUNC;++i)
    {
        if((ls_pcie->cfg_msi_en_int_status & (1<<i))&& ls_pcie->cfg_msi_en_callback[i])
        {
            ls_pcie->cfg_msi_en_callback[i](ls_pcie,i,cfg_msi_en>>CFG_MSI_EN_POS>>i,ls_pcie->cfg_msi_en_param[i]);
        }
    }
    ls_pcie->cfg_msi_en_int_status = 0;
    if(ls_pcie->link_req_rst_not_status)
    {
        dev_info(ls_pcie->pci.dev,"link_req_rst_not_status\n");
        dw_pcie_ep_linkdown(&ls_pcie->pci.ep);
    }
    ls_pcie->link_req_rst_not_status = 0;
    return IRQ_HANDLED;
}

static irqreturn_t ls_pcie_misc_hard_irq(int irq, void *arg)
{
    struct ls_pcie *ls_pcie = arg;
    int err;
    u32 msi_en_int_stat,radm_pm_int_stat;
    err = regmap_read(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg2 + PIPE_CFG2_CFG_MSI_EN_INT_OFFSET,&msi_en_int_stat);
    BUG_ON(err);
    dev_info(ls_pcie->pci.dev,"msi_en_int_stat %x\n",msi_en_int_stat);
    if(msi_en_int_stat & CFG_MSI_EN_INT_STAT_MASK)
    {
        err = regmap_write(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg2 + PIPE_CFG2_CFG_MSI_EN_INT_CLR_OFFSET, 
            (msi_en_int_stat & CFG_MSI_EN_INT_STAT_MASK)>>CFG_MSI_EN_INT_STAT_POS);
        BUG_ON(err);
        ls_pcie->cfg_msi_en_int_status = (msi_en_int_stat & CFG_MSI_EN_INT_STAT_MASK)>>CFG_MSI_EN_INT_STAT_POS;
    }
    err = regmap_read(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg2 + 
        PIPE_CFG2_RADM_PM_INT_OFFSET,&radm_pm_int_stat);
    BUG_ON(err);
    dev_info(ls_pcie->pci.dev,"radm_pm_int_stat %x\n",radm_pm_int_stat);
    if(radm_pm_int_stat & LINK_REQ_RST_NOT_INT_STAT)
    {
        err = regmap_write(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg2 + PIPE_CFG2_RADM_PM_INT_CLR_OFFSET,LINK_REQ_RST_NOT_INT_CLR);
        BUG_ON(err);
        ls_pcie->link_req_rst_not_status = 1;
    }
    //u32 radm_trgt1_atu_cbuf_err_int_stat,cfg_sys_err_rc_int_stat,misc_int_stat;
    // err = regmap_read(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg2 + 
    //     PIPE_CFG2_RADM_TRGT1_ATU_CBUF_ERR_INT_STAT_OFFSET,&radm_trgt1_atu_cbuf_err_int_stat);
    // BUG_ON(err);
    // err = regmap_read(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg2 + 
    //     PIPE_CFG2_CFG_SYS_ERR_RC_INT_STAT_OFFSET,&cfg_sys_err_rc_int_stat);
    // BUG_ON(err);
    // err = regmap_read(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg2 + 
    //     PIPE_CFG2_MISC_INT_OFFSET,&misc_int_stat);
    // BUG_ON(err);
    return IRQ_WAKE_THREAD;
}

static irqreturn_t ls_pcie_smlh_link_irq_thread(int irq, void *arg)
{
    struct ls_pcie *ls_pcie = arg;
    dw_pcie_ep_linkup(&ls_pcie->pci.ep);
    dev_info(ls_pcie->pci.dev,"ls_pcie_smlh_link_irq_thread\n");
    return IRQ_HANDLED;
}

static irqreturn_t ls_pcie_ip_irq_thread(int irq, void *arg)
{
    struct ls_pcie *ls_pcie = arg;
    u32 int_stat;
    dev_info(ls_pcie->pci.dev,"ls_pcie_ip_irq_thread\n");
    int err = regmap_read(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg1 + PIPE_CFG1_IP_INT_STAT_OFFSET,&int_stat);
    BUG_ON(err);
    if(int_stat & RADM_CORRECTABLE_ERR)
    {
        dev_err(ls_pcie->pci.dev,"radm correctable err\n");
        regmap_write(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg1 + PIPE_CFG1_INT_CLR1_OFFSET, RADM_CORRECTABLE_ERR_CLR);
    }
    if(int_stat & RADM_NONFATAL_ERR)
    {
        dev_err(ls_pcie->pci.dev,"radm nonfatal err\n");
        regmap_write(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg1 + PIPE_CFG1_INT_CLR1_OFFSET, RADM_NONFATAL_ERR_CLR);
    }
    if(int_stat & RADM_FATAL_ERR)
    {
        dev_err(ls_pcie->pci.dev,"radm nonfatal err\n");
        regmap_write(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg1 + PIPE_CFG1_INT_CLR1_OFFSET, RADM_FATAL_ERR_CLR);
    }
    return IRQ_HANDLED;
}

static irqreturn_t ls_pcie_radm_to_irq_thread(int irq, void *arg)
{
    struct ls_pcie *ls_pcie = arg;
    dev_info(ls_pcie->pci.dev,"ls_pcie_radm_to_irq_thread\n");
    return IRQ_HANDLED;
}

static irqreturn_t ls_pcie_aux_irq_thread(int irq, void *arg)
{
    struct ls_pcie *ls_pcie = arg;
    dev_info(ls_pcie->pci.dev,"ls_pcie_aux_irq_thread\n");
    return IRQ_HANDLED;
}

static irqreturn_t ls_pcie_msg_irq_thread(int irq, void *arg)
{
    struct ls_pcie *ls_pcie = arg;
    dev_info(ls_pcie->pci.dev,"ls_pcie_msg_irq_thread\n");
    return IRQ_HANDLED;
}

void ls_pcie_set_msi_en_callback(struct ls_pcie *pcie,u8 func_no,msi_en_cb_t cb,void *param)
{
    pcie->cfg_msi_en_callback[func_no] = cb;
    pcie->cfg_msi_en_param[func_no] = param;
}
EXPORT_SYMBOL(ls_pcie_set_msi_en_callback);

struct reserved_mem *ls_get_reserved_mem_by_name(struct device *dev,const char *name)
{
	struct device_node *np = dev->of_node;
	BUG_ON(!np);
	int idx = of_property_match_string(np, "memory-region-names", name);
	struct device_node *target = of_parse_phandle(np, "memory-region", idx);
	struct reserved_mem *rmem = of_reserved_mem_lookup(target);
	of_node_put(target);
	return rmem;
}
EXPORT_SYMBOL(ls_get_reserved_mem_by_name);


static bool pcie_dma_write_chan_filter(struct dma_chan *chan, void *data)
{
    struct dma_slave_caps caps;
    dma_get_slave_caps(chan, &caps);
    return caps.directions == BIT(DMA_MEM_TO_DEV);
}

static bool pcie_dma_read_chan_filter(struct dma_chan *chan, void *data)
{
    struct dma_slave_caps caps;
    dma_get_slave_caps(chan, &caps);
    return caps.directions == BIT(DMA_DEV_TO_MEM);
}

struct dma_chan *ls_pcie_request_dma_chan(struct ls_pcie *priv, bool write)
{
    struct dma_device *dma = &priv->pci.edma.dw->dma;
    return __dma_request_channel(&dma->cap_mask, write ? pcie_dma_write_chan_filter : pcie_dma_read_chan_filter,
        priv, dma->dev->of_node);
}
EXPORT_SYMBOL(ls_pcie_request_dma_chan);

void ls_pcie_dma_release_chan(struct dma_chan *chan)
{
    dmaengine_terminate_sync(chan);
    dma_release_channel(chan);
}
EXPORT_SYMBOL(ls_pcie_dma_release_chan);

static void pcie_dma_complete(void *param)
{
    complete(param);
}

int ls_pcie_dma_read_write(struct dma_chan *chan, phys_addr_t dst, phys_addr_t src, size_t len,
        dma_async_tx_callback callback,void *callback_param,struct completion *completion_ptr,bool read)
{
    struct dma_device *dma = chan->device;
    struct dma_slave_config config = {
        .dst_addr = dst,
        .src_addr = src,
    };
    int ret = dmaengine_slave_config(chan, &config);
    if (ret) {
        dev_err(dma->dev, "Failed to configure DMA channel\n");
        return ret;
    }
    struct dma_async_tx_descriptor *tx = dmaengine_prep_slave_single(chan,read ? dst : src, len, read ? DMA_DEV_TO_MEM : DMA_MEM_TO_DEV, 0);
    if (!tx) {
        dev_err(dma->dev, "Failed to prepare DMA transaction\n");
        return -EIO;
    }
    if(completion_ptr)
    {
        init_completion(completion_ptr);
        tx->callback = pcie_dma_complete;
        tx->callback_param = completion_ptr;
    }else
    {
        tx->callback = callback;
        tx->callback_param = callback_param;
    }
    dma_cookie_t cookie = dmaengine_submit(tx);
    if (dma_submit_error(cookie)) {
        dev_err(dma->dev, "Failed to submit DMA transaction\n");
        return -EIO;
    }
    dma_async_issue_pending(chan);
    if(completion_ptr)
    {
        wait_for_completion(completion_ptr);
    }
    return 0;
}
EXPORT_SYMBOL(ls_pcie_dma_read_write);

#define PCIE_IN_ATU_FUZZY_TYPE_MATCH_EN	BIT(27)
#define PCIE_IN_ATU_SINGLE_ADDR_LOC_TRANS_EN BIT(23)
#define PCIE_IN_ATU_MSG_TYPE_MATCH_EN	BIT(13)
int ls_pcie_set_inbound_atu(struct dw_pcie *pci, int index, u32 ctrl1,u32 ctrl2,u64 base_addr,
				u64 limit_addr, u64 target_addr,u8 cbuf_incr);
int ls_pcie_set_msgd_inbound_atu(struct ls_pcie *pcie,u8 cbuf_incr,u32 cbuf_size,u64 target_addr)
{
    struct dw_pcie *pci = &pcie->pci;
    pcie->msgd_ib_atu = find_first_zero_bit(pci->ep.ib_window_map, pci->num_ib_windows);
    u64 limit_addr = target_addr + cbuf_size - 1;
    dev_info(pci->dev, "Set inbound MSGD ATU: atu_win=%d, target_addr=0x%llx, limit_addr=0x%llx\n", pcie->msgd_ib_atu, target_addr, limit_addr);
    int ret = ls_pcie_set_inbound_atu(pci,pcie->msgd_ib_atu,PCIE_ATU_TYPE_MSG,
        PCIE_IN_ATU_FUZZY_TYPE_MATCH_EN |PCIE_IN_ATU_SINGLE_ADDR_LOC_TRANS_EN | PCIE_IN_ATU_MSG_TYPE_MATCH_EN,0,
        limit_addr,target_addr,cbuf_incr);
    if (ret) {
        dev_err(pci->dev, "Failed to program inbound ATU\n");
        return ret;
    }
    set_bit(pcie->msgd_ib_atu, pci->ep.ib_window_map);
    return 0;
}
EXPORT_SYMBOL(ls_pcie_set_msgd_inbound_atu);

void ls_pcie_clear_msgd_inbound_atu(struct ls_pcie *pcie)
{
    struct dw_pcie *pci = &pcie->pci;
    dw_pcie_disable_atu(pci, PCIE_ATU_REGION_DIR_IB, pcie->msgd_ib_atu);
    clear_bit(pcie->msgd_ib_atu, pci->ep.ib_window_map);
}
EXPORT_SYMBOL(ls_pcie_clear_msgd_inbound_atu);


int ls_pcie_set_outbound_atu(struct ls_pcie *pcie,struct dw_pcie_ob_atu_cfg *atu_cfg)
{
    struct dw_pcie *pci = &pcie->pci;
	u32 free_win = find_first_zero_bit(pci->ep.ob_window_map, pci->num_ob_windows);
	int ret;
    if (free_win >= pci->num_ob_windows) {
		dev_err(pci->dev, "No free outbound window\n");
		return -EINVAL;
	}
    dev_info(pci->dev, "Set outbound ATU: atu_win=%d\n", free_win);
    atu_cfg->index = free_win;
    ret = dw_pcie_prog_outbound_atu(pci,atu_cfg);
    if(ret)
    {
        dev_err(pci->dev, "Failed to program outbound ATU\n");
        return ret;
    }
    set_bit(free_win, pci->ep.ob_window_map);
	pci->ep.outbound_addr[free_win] = atu_cfg->parent_bus_addr;
    return 0;
}
EXPORT_SYMBOL(ls_pcie_set_outbound_atu);

void ls_pcie_clear_outbound_atu(struct ls_pcie *pcie,u64 cpu_addr)
{
    pci_epc_unmap_addr(pcie->pci.ep.epc, 0, 0, cpu_addr);
}
EXPORT_SYMBOL(ls_pcie_clear_outbound_atu);


int ls_pcie_ep_set_rom_bar(struct ls_pcie *pcie,u8 func_no,dma_addr_t vbios_addr,u32 size)
{
	struct dw_pcie *pci = &pcie->pci;
    struct dw_pcie_ep *ep = &pci->ep;
	u32 free_win = find_first_zero_bit(ep->ib_window_map, pci->num_ib_windows);
    dev_info(pci->dev, "Set ROM BAR: func_no=%d, atu_win=%d, vbios_addr=0x%llx, size=0x%x\n", func_no, free_win, vbios_addr, size);

	int ret = dw_pcie_prog_ep_inbound_atu(pci,func_no,free_win,PCIE_ATU_TYPE_MEM,vbios_addr,ATU_ROM_BAR_IDX,size);
    
    if (ret) {
        dev_err(pci->dev, "Failed to program inbound ATU\n");
        return ret;
    }
    pcie->rom_bar_atu[func_no] = free_win + 1;
	set_bit(free_win, ep->ib_window_map);

    dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_ep_writel_dbi2(&pci->ep, func_no, PCI_ROM_ADDRESS, PCI_ROM_ADDRESS_ENABLE);
	dw_pcie_ep_writel_dbi2(&pci->ep, func_no, PCI_ROM_ADDRESS, lower_32_bits(size - 1));
	dw_pcie_ep_writel_dbi(&pci->ep, func_no, PCI_ROM_ADDRESS, PCI_ROM_ADDRESS_ENABLE);

	dw_pcie_dbi_ro_wr_dis(pci);
    return 0;
}
EXPORT_SYMBOL(ls_pcie_ep_set_rom_bar);

void ls_pcie_ep_clear_rom_bar(struct ls_pcie *pcie,u8 func_no)
{
    struct dw_pcie *pci = &pcie->pci;
    struct dw_pcie_ep *ep = &pci->ep;

    dw_pcie_dbi_ro_wr_en(pci);

    dw_pcie_ep_writel_dbi2(&pci->ep, func_no, PCI_ROM_ADDRESS, 0x0);
    dw_pcie_ep_writel_dbi(&pci->ep, func_no, PCI_ROM_ADDRESS, 0x0);

    dw_pcie_dbi_ro_wr_dis(pci);

    if(pcie->rom_bar_atu[func_no])
    {
        dev_info(pci->dev, "Clear ROM BAR: func_no=%d, atu_win=%d\n", func_no, pcie->rom_bar_atu[func_no] - 1);
    	dw_pcie_disable_atu(pci, PCIE_ATU_REGION_DIR_IB, pcie->rom_bar_atu[func_no] - 1);
        clear_bit(pcie->rom_bar_atu[func_no] - 1, ep->ib_window_map);
        pcie->rom_bar_atu[func_no] = 0;
    }
}
EXPORT_SYMBOL(ls_pcie_ep_clear_rom_bar);

void ls_pcie_ep_set_bar_raw(struct ls_pcie *pcie,u8 func_no,enum pci_barno bar,size_t size,int flags)
{
    struct dw_pcie *pci = &pcie->pci;
    struct dw_pcie_ep *ep = &pci->ep;
	u32 reg = PCI_BASE_ADDRESS_0 + (4 * bar);
	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_ep_writel_dbi2(ep, func_no, reg, lower_32_bits(size - 1));
	dw_pcie_ep_writel_dbi(ep, func_no, reg, flags);
	dw_pcie_dbi_ro_wr_dis(pci);
}
EXPORT_SYMBOL(ls_pcie_ep_set_bar_raw);

void ls_pcie_ep_clear_bar_raw(struct ls_pcie *pcie,u8 func_no,enum pci_barno bar)
{
    struct dw_pcie *pci = &pcie->pci;
	struct dw_pcie_ep *ep = &pci->ep;
	u32 reg = PCI_BASE_ADDRESS_0 + (4 * bar);
	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_ep_writel_dbi2(ep, func_no, reg, 0x0);
	dw_pcie_ep_writel_dbi(ep, func_no, reg, 0x0);
	dw_pcie_dbi_ro_wr_dis(pci);
}
EXPORT_SYMBOL(ls_pcie_ep_clear_bar_raw);

static void ls_pcie_remove(struct platform_device *pdev)
{
	struct ls_pcie *ls_pcie = platform_get_drvdata(pdev);
    pci_epc_deinit_notify(ls_pcie->pci.ep.epc);
    dw_pcie_ep_deinit(&ls_pcie->pci.ep);
    clk_disable_unprepare(ls_pcie->pcie_clk);
}

static void ls_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
    struct ls_pcie *ls_pcie = container_of(pci,struct ls_pcie, pci);
	enum pci_barno bar;
    dev_info(pci->dev, "ls_pcie_ep_init\n");
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++)
		dw_pcie_ep_reset_bar(pci, bar);
    for(u8 i=0;i<LS_PCIE_MAX_FUNC;++i)
        ls_pcie_ep_clear_rom_bar(ls_pcie,i);
}

static int ls_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				unsigned int type, u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_IRQ_INTX:
		return dw_pcie_ep_raise_intx_irq(ep, func_no);
	case PCI_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
	}

	return 0;
}

static const struct pci_epc_features ls_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = false,
};

static const struct pci_epc_features*
ls_pcie_ep_get_features(struct dw_pcie_ep *ep)
{
    return &ls_pcie_epc_features;
}

static unsigned int ls_pcie_ep_get_dbi_offset(struct dw_pcie_ep *ep, u8 func_no)
{
	return EP_FUNC_REG_OFFSET * func_no;
}

static const struct dw_pcie_ep_ops pcie_ep_ops = {
	.init = ls_pcie_ep_init,
	.raise_irq = ls_pcie_ep_raise_irq,
	.get_features = ls_pcie_ep_get_features,
	.get_dbi_offset = ls_pcie_ep_get_dbi_offset,
};

static int irq_setup(struct platform_device *pdev,struct ls_pcie *ls_pcie)
{
	struct device *dev = &pdev->dev;
    int irq,ret;
    irq = platform_get_irq_byname(pdev,"smlh_link");
    if(irq < 0)
    {
		dev_err(dev, "cannot get smlh link irq\n");
        return irq;
    }
    ret = devm_request_threaded_irq(dev,irq,NULL,ls_pcie_smlh_link_irq_thread, IRQF_ONESHOT,dev_name(dev),ls_pcie);
    if(ret<0) {
		dev_err(dev, "cannot request smlh link irq\n");
        return ret;
    }

    irq = platform_get_irq_byname(pdev,"ip");
    if(irq < 0)
    {
		dev_err(dev, "cannot get ip irq\n");
        return irq;
    }
    ret = devm_request_threaded_irq(dev,irq,NULL,ls_pcie_ip_irq_thread, IRQF_ONESHOT,dev_name(dev),ls_pcie);
    if(ret<0) {
		dev_err(dev, "cannot request ip irq\n");
        return ret;
    }

    irq = platform_get_irq_byname(pdev,"radm_to");
    if(irq < 0)
    {
		dev_err(dev, "cannot get radm_to irq\n");
        return irq;
    }
    ret = devm_request_threaded_irq(dev,irq,NULL,ls_pcie_radm_to_irq_thread, IRQF_ONESHOT,dev_name(dev),ls_pcie);
    if(ret<0) {
		dev_err(dev, "cannot request radm_to irq\n");
        return ret;
    }

    irq = platform_get_irq_byname(pdev,"aux");
    if(irq < 0)
    {
		dev_err(dev, "cannot get aux irq\n");
        return irq;
    }
    ret = devm_request_threaded_irq(dev,irq,NULL,ls_pcie_aux_irq_thread, IRQF_ONESHOT,dev_name(dev),ls_pcie);
    if(ret<0) {
		dev_err(dev, "cannot request aux irq\n");
        return ret;
    }

    irq = platform_get_irq_byname(pdev,"msg");
    if(irq < 0)
    {
		dev_err(dev, "cannot get msg irq\n");
        return irq;
    }
    ret = devm_request_threaded_irq(dev,irq,NULL,ls_pcie_msg_irq_thread, IRQF_ONESHOT,dev_name(dev),ls_pcie);
    if(ret<0) {
		dev_err(dev, "cannot request msg irq\n");
        return ret;
    }

    irq = platform_get_irq_byname(pdev,"misc");
    if(irq < 0)
    {
		dev_err(dev, "cannot get misc irq\n");
        return irq;
    }
    ret = devm_request_threaded_irq(dev,irq,ls_pcie_misc_hard_irq,ls_pcie_misc_irq_thread, IRQF_ONESHOT,dev_name(dev),ls_pcie);
    if(ret<0) {
		dev_err(dev, "cannot request misc irq\n");
        return ret;
    }
    return 0;
}

static int ls_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
    const struct ls_pcie_of_data *of_data = of_device_get_match_data(dev);
    struct resource *cfg[3];
    dev_info(dev, "ls pcie probe entry,platform_device %px\n",pdev);
    int ret = 0;
    if(!of_data){
		dev_err(dev, "of_device_get_match_data fail\n");
        return -EINVAL;
    }

    struct ls_pcie *ls_pcie = devm_kzalloc(dev,sizeof(*ls_pcie),GFP_KERNEL);
    if(!ls_pcie){
		dev_err(dev, "ls pcie devm_kzalloc fail\n");
        return -ENOMEM;
    }
    ls_pcie->pipesys_cfg = syscon_regmap_lookup_by_phandle(dev->of_node, "pipesys-cfg");
    if (IS_ERR(ls_pcie->pipesys_cfg)) {
        dev_err(dev, "Failed to get pipesys syscon regmap\n");
        return PTR_ERR(ls_pcie->pipesys_cfg);
    }

    cfg[0] = platform_get_resource_byname(pdev,IORESOURCE_MEM,"pipe_cfg0");
    if (!cfg[0]) {
        dev_err(&pdev->dev, "Failed to get pipe_cfg0 resource\n");
        return -ENODEV;
    }

    cfg[1] = platform_get_resource_byname(pdev,IORESOURCE_MEM,"pipe_cfg1");
    if (!cfg[1]) {
        dev_err(&pdev->dev, "Failed to get pipe_cfg1 resource\n");
        return -ENODEV;
    }

    cfg[2] = platform_get_resource_byname(pdev,IORESOURCE_MEM,"pipe_cfg2");
    if (!cfg[2]) {
        dev_err(&pdev->dev, "Failed to get pipe_cfg2 resource\n");
        return -ENODEV;
    }

    ls_pcie->pipe_cfg0 = cfg[0]->start;
    ls_pcie->pipe_cfg1 = cfg[1]->start;
    ls_pcie->pipe_cfg2 = cfg[2]->start;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
    if(ret)
    {
        dev_err(dev, "dma_set_mask_and_coherent fail,%d\n",ret);
        return ret;
    }
    ls_pcie->pci.dev = dev;
    platform_set_drvdata(pdev,ls_pcie);

    ret = irq_setup(pdev,ls_pcie);
    if(ret < 0)
    {
        dev_err(dev, "irq_setup fail,%d\n",ret);
        return ret;
    }

    ls_pcie->pcie_clk = devm_clk_get_optional(dev, "pcie-clk");
    if (IS_ERR(ls_pcie->pcie_clk)) {
        dev_err(dev, "Failed to get pcie-clk\n");
        return PTR_ERR(ls_pcie->pcie_clk);
    }

    ret = clk_prepare_enable(ls_pcie->pcie_clk);
    if (ret) {
        dev_err(dev, "Failed to enable pcie-clk, %d\n", ret);
        return ret;
    }
    switch(of_data->mode)
    {
    case DW_PCIE_RC_TYPE:
		dev_err(dev, "ls pcie rc not supported\n");
        ret = -EINVAL;
        goto unprepare_clk;
    break;
    case DW_PCIE_EP_TYPE:
        ls_pcie->pci.ep.ops = &pcie_ep_ops;
        ls_pcie->pci.ep.page_size = SZ_64K;
        ls_pcie->pci.edma.nr_irqs = 1;
        ls_pcie->pci.edma.ll_wr_cnt = NUM_DMA_WR_CHAN;
        ls_pcie->pci.edma.ll_rd_cnt = NUM_DMA_RD_CHAN;
        ls_pcie->pci.edma.mf = EDMA_MF_HDMA_NATIVE;
        ret = dw_pcie_ep_init(&ls_pcie->pci.ep);
        if(ret){
			dev_err(dev, "dw_pcie_ep_init fail,%d\n",ret);
            goto unprepare_clk;
        }

        dev_info(dev,"pci->dbi_base=0x%px\n",ls_pcie->pci.dbi_base);
        dev_info(dev,"pci->dbi_base2=0x%px\n",ls_pcie->pci.dbi_base2);
        dev_info(dev,"pci->atu_base=0x%px\n",ls_pcie->pci.atu_base);
        dev_info(dev,"pci->atu_size=0x%lx\n",ls_pcie->pci.atu_size);
        dev_info(dev,"pci->atu_phys_addr=0x%llx\n",ls_pcie->pci.atu_phys_addr);
        dev_info(dev,"pci->dbi_phys_addr=0x%llx\n",ls_pcie->pci.dbi_phys_addr);
        dev_info(dev,"pci->edma.reg_base=0x%px\n",ls_pcie->pci.edma.reg_base);
        
        dev_info(dev, "dw_pcie_ep_init_registers\n");

        ret = dw_pcie_ep_init_registers(&ls_pcie->pci.ep);
        if(ret) {
			dev_err(dev, "dw_pcie_ep_init_registers fail,%d\n",ret);
            dw_pcie_ep_deinit(&ls_pcie->pci.ep);
            goto unprepare_clk;
        }
        dev_info(dev, "pci_epc_init_notify\n");
        
		pci_epc_init_notify(ls_pcie->pci.ep.epc);
    break;
    default:
		dev_err(dev, "INVALID device type %d\n", of_data->mode);

        ret = -EINVAL;
        goto unprepare_clk;
    break;
    }
    ret = regmap_update_bits(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg1 + PIPE_CFG1_IP_INT_MASK_OFFSET,
        SMLH_LINK_UP,SMLH_LINK_UP);
    BUG_ON(ret);
    ret = regmap_update_bits(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg2 + PIPE_CFG2_CFG_MSI_EN_INT_OFFSET,
        CFG_MSI_EN_INT_MASK_MASK,CFG_MSI_EN_INT_MASK_MASK);
    BUG_ON(ret);
    ret = regmap_update_bits(ls_pcie->pipesys_cfg,ls_pcie->pipe_cfg2 + PIPE_CFG2_RADM_PM_INT_OFFSET,
        LINK_REQ_RST_NOT_INT_MASK,LINK_REQ_RST_NOT_INT_MASK);
    BUG_ON(ret);
    return 0;

unprepare_clk:
    clk_disable_unprepare(ls_pcie->pcie_clk);
    return ret;
}

static const struct ls_pcie_of_data ls_pcie_rc_of_data = {
    .mode = DW_PCIE_RC_TYPE,
};

static const struct ls_pcie_of_data ls_pcie_ep_of_data = {
    .mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id ls_pcie_of_match[] = {
	{ .compatible = "linkedsemi,ls-pcie-rc",.data = &ls_pcie_rc_of_data },
	{ .compatible = "linkedsemi,ls-pcie-ep",.data = &ls_pcie_ep_of_data },
	{},
};
MODULE_DEVICE_TABLE(of, ls_pcie_of_match);

static struct platform_driver ls_pcie_driver = {
    .probe = ls_pcie_probe,
    .remove = ls_pcie_remove,
    .driver = {
        .name = "ls-pcie",
        .of_match_table = ls_pcie_of_match,
    },
};
module_platform_driver(ls_pcie_driver);
MODULE_DESCRIPTION("PCIe controller driver for Linkedsemi SOCs");
MODULE_LICENSE("GPL v2");


