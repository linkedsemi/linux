#ifndef __LS_PCIE_VUSB_H__
#define __LS_PCIE_VUSB_H__

#include <linux/list.h>

struct ls_pcie_vusb_dev;
struct ls_pcie_vusb;
struct device;

struct ls_pcie_vusb_ops
{
    int (*set_address)(struct ls_pcie_vusb_dev *vusb_dev, u8 addr);
    int (*setup_packet)(struct ls_pcie_vusb_dev *vusb_dev, u8 *setup, u8 addr);
    int (*reset)(struct ls_pcie_vusb_dev *vusb_dev);
};

struct ls_pcie_vusb_dev
{
    struct ls_pcie_vusb_ops *ops;
    struct ls_pcie_vusb *vusb;
    struct device *dma_dev;
    u16 port_idx;
    u8 addr;
};

struct ls_pcie_vusb_ep_req
{
    void *buf;
    u32 len;
    u32 actual;
    dma_addr_t dma;
    struct ls_pcie_vusb *vusb;
    struct list_head node;
    int (*complete)(struct ls_pcie_vusb_ep_req *req);
    int status;
    u8 ep;
    u8 addr;
    u8 stall;
};

/* bind port */
int ls_pcie_vusb_add_dev(struct ls_pcie_vusb_dev *dev, struct ls_pcie_vusb_ops *ops);
/* unbind port */
int ls_pcie_vusb_delete_dev(struct ls_pcie_vusb_dev *dev);
/* attach port */
int ls_pcie_vusb_connect_dev(struct ls_pcie_vusb_dev *dev);
/* detach port */
int ls_pcie_vusb_disconnect_dev(struct ls_pcie_vusb_dev *dev);
/* submit req */
int ls_pcie_vusb_req_submit(struct ls_pcie_vusb_ep_req *req);
/* cancel req */
int ls_pcie_vusb_req_cancel(struct ls_pcie_vusb_ep_req *req);

#endif //__LS_PCIE_VUSB_H__
