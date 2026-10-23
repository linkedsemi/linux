#ifndef __LS_PCIE_VHUB__
#define __LS_PCIE_VHUB__

#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include "linkedsemi-pcie-vusb.h"

#define LS_PCIE_VHUB_PORT_MAX           (5)

struct ls_pcie_vhub_dev;
struct ls_pcie_vhub;
struct ls_pcie_vhub_ep;

typedef enum
{
    VHUB_REQ_IDLE,
    VHUB_REQ_BUSY,
    VHUB_REQ_DEQUEUE,
    VHUB_REQ_DISABLE,
    VHUB_REQ_FREED
} vhub_req_state_t;

struct ls_pcie_vhub_req
{
    struct usb_request req;
    struct list_head node;
    struct ls_pcie_vusb_ep_req ep_req;
    struct ls_pcie_vhub_ep *vhub_ep;
    vhub_req_state_t state;
};

struct ls_pcie_vhub_ep
{
    char name[16];
    struct usb_ep ep;
    struct ls_pcie_vhub_dev *dev;
    struct ls_pcie_vhub *vhub;
    void *buf;
    dma_addr_t buf_dma;
    size_t buf_len;
    struct list_head req_list;
    struct list_head dev_node;
    struct ls_pcie_vusb_ep_req ep_req;
    uint8_t enabled : 1;
    uint8_t halted : 1;
    uint8_t wedged : 1;
    uint8_t delay_start : 1;
    uint8_t dir : 1;
};

struct ls_pcie_vhub_dev
{
    struct usb_gadget_driver *driver;
    struct usb_gadget *gadget;
    struct device *udc_dev;
    struct ls_pcie_vhub_ep ep0;
    struct list_head ep_allocated;
    struct ls_pcie_vhub *vhub;
    spinlock_t lock;
    uint8_t ep_count;
    uint8_t addr;
    uint8_t port_index;
    uint8_t enabled : 1;
    uint8_t delayed_status : 1;
};

struct ls_pcie_vhub_port
{
    union
    {
        struct
        {
            uint16_t status;
            uint16_t change;
        };
        uint32_t raw;
    } portsc;
    struct ls_pcie_vhub_dev dev;
};

struct ls_pcie_vhub
{
    struct ls_pcie_vusb_dev dev;
    struct platform_device *pdev;
    struct ls_pcie_vhub_port *ports;
    struct ls_pcie_vhub_dev *setup_dev;
    struct ls_pcie_vhub_ep ep0;
    struct ls_pcie_vhub_ep ep1;
    uint8_t ports_num;
    spinlock_t lock;
};

int ls_pcie_vhub_attach(struct ls_pcie_vhub *vhub, uint8_t port_idx);
int ls_pcie_vhub_detach(struct ls_pcie_vhub *vhub, uint8_t port_idx);
int ls_pcie_vhub_dev_init(struct ls_pcie_vhub *vhub, uint8_t port_idx);
int ls_pcie_vhub_dev_deinit(struct ls_pcie_vhub *vhub, uint8_t port_idx);
int ls_pcie_vhub_dev_setup(struct ls_pcie_vhub_dev *dev, struct usb_ctrlrequest *ctrl);
int ls_pcie_vhub_dev_reset(struct ls_pcie_vhub_dev *dev);

#endif //__LS_PCIE_VHUB__
