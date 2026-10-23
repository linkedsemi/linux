#ifndef PCIE_LS_H_
#define PCIE_LS_H_

#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/phy/phy.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/of_reserved_mem.h>
#include <linux/regmap.h>
#include "pcie-designware.h"
#include "dw-edma-core.h"

#define LS_PCIE_MAX_FUNC 8
#define MSI_ADDR_LOW_OFFSET 0x74
#define MSI_ADDR_HIGH_OFFSET 0x78
#define MSI_DATA_OFFSET 0x7c
enum cbuf_incr_type {
    CBUF_INCR_0 = 0,
    CBUF_INCR_4 = 1,
    CBUF_INCR_8 = 2,
    CBUF_INCR_16 = 3,
    CBUF_INCR_32 = 4,
    CBUF_INCR_64 = 5,
    CBUF_INCR_128 = 6,
    CBUF_INCR_256 = 7,
    CBUF_INCR_512 = 8,
    CBUF_INCR_1024 = 9,
    CBUF_INCR_2048 = 10,
    CBUF_INCR_4096 = 11,
    CBUF_INCR_8192 = 12,
};

#define CBUF_INCR_SIZE(incr) (2U << (incr))

struct ls_pcie;
typedef void (*msi_en_cb_t)(struct ls_pcie *,u8 func_no,u8 en,void *param);
struct ls_pcie {
    struct dw_pcie pci;
    struct regmap *pipesys_cfg;
    struct clk *pcie_clk;
    msi_en_cb_t cfg_msi_en_callback[LS_PCIE_MAX_FUNC];
    void *cfg_msi_en_param[LS_PCIE_MAX_FUNC];
    u16 pipe_cfg0,pipe_cfg1,pipe_cfg2;
    u8 rom_bar_atu[LS_PCIE_MAX_FUNC];
    u8 msgd_ib_atu;
    u8 cfg_msi_en_int_status;
    u8 link_req_rst_not_status;
};
void ls_pcie_set_msi_en_callback(struct ls_pcie *pcie,u8 func_no,msi_en_cb_t cb,void *param);
struct reserved_mem *ls_get_reserved_mem_by_name(struct device *dev,const char *name);
struct dma_chan *ls_pcie_request_dma_chan(struct ls_pcie *priv, bool write);
void ls_pcie_dma_release_chan(struct dma_chan *chan);
int ls_pcie_dma_read_write(struct dma_chan *chan, phys_addr_t dst, phys_addr_t src, size_t len,
        dma_async_tx_callback callback,void *callback_param,struct completion *completion_ptr,bool read);
int ls_pcie_set_msgd_inbound_atu(struct ls_pcie *pcie,u8 cbuf_incr,u32 cbuf_size,u64 target_addr);
void ls_pcie_clear_msgd_inbound_atu(struct ls_pcie *pcie);
int ls_pcie_set_outbound_atu(struct ls_pcie *pcie,struct dw_pcie_ob_atu_cfg *atu_cfg);
void ls_pcie_clear_outbound_atu(struct ls_pcie *pcie,u64 cpu_addr);
int ls_pcie_ep_set_rom_bar(struct ls_pcie *pcie,u8 func_no,dma_addr_t vbios_addr,u32 size);
void ls_pcie_ep_clear_rom_bar(struct ls_pcie *pcie,u8 func_no);
void ls_pcie_ep_set_bar_raw(struct ls_pcie *pcie,u8 func_no,enum pci_barno bar,size_t size,int flags);
void ls_pcie_ep_clear_bar_raw(struct ls_pcie *pcie,u8 func_no,enum pci_barno bar);

#endif

