#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/usb/gadget.h>
#include "linkedsemi-pcie-vusb.h"
#include "vhub.h"
#include "ep.h"

static inline struct ls_pcie_vhub_req *next_request(struct list_head *list)
{
    return list_first_entry_or_null(list, struct ls_pcie_vhub_req, node);
}

static void ls_pcie_vhub_unmap_request(struct ls_pcie_vhub_ep *ep, struct ls_pcie_vhub_req *vhub_req)
{
    usb_gadget_unmap_request_by_dev(ep->vhub->dev.dma_dev, &vhub_req->req, ep->dir);
}

static int vhub_epn_kick_transfer(struct ls_pcie_vhub_ep *epn)
{
    struct ls_pcie_vhub_req *vhub_req = next_request(&epn->req_list);
    struct usb_request *req = &vhub_req->req;
    struct ls_pcie_vusb_ep_req *ep_req = &vhub_req->ep_req;
    int ret;

    ret = usb_gadget_map_request_by_dev(epn->vhub->dev.dma_dev, req, epn->dir);
    if (ret)
        return ret;

    ep_req->addr = epn->dev->addr;
    ep_req->ep = epn->ep.address;
    ep_req->actual = 0;
    ep_req->len = req->length;
    ep_req->vusb = epn->vhub->dev.vusb;
    ep_req->complete = ls_pcie_vhub_ep_req_complete;
    ep_req->stall = 0;

    if (req->length) {
        ep_req->buf = req->buf;
        ep_req->dma = req->dma;
    } else {
        ep_req->buf = NULL;
        ep_req->dma = 0;
    }

    ret = ls_pcie_vusb_req_submit(ep_req);
    if (ret)
        ls_pcie_vhub_unmap_request(epn, vhub_req);

    return ret;
}

static int ls_pcie_vhub_epn_disable(struct usb_ep *ep)
{
    struct ls_pcie_vhub_ep *vhub_ep = container_of(ep, struct ls_pcie_vhub_ep, ep);
    struct ls_pcie_vhub_req *vhub_req, *tmp;
    struct ls_pcie_vhub_dev *dev = vhub_ep->dev;

    spin_lock(&dev->lock);
    list_for_each_entry_safe(vhub_req, tmp, &vhub_ep->req_list, node)
    {
        bool req_busy = (vhub_req->state == VHUB_REQ_BUSY);

        list_del_init(&vhub_req->node);
        if (req_busy && ls_pcie_vusb_req_cancel(&vhub_req->ep_req) != 0) {
            vhub_req->state = VHUB_REQ_DISABLE;
            continue;
        }
        vhub_req->state = VHUB_REQ_IDLE;
        spin_unlock(&dev->lock);

        if (req_busy)
            ls_pcie_vhub_unmap_request(vhub_ep, vhub_req);

        vhub_req->req.status = -ESHUTDOWN;
        usb_gadget_giveback_request(ep, &vhub_req->req);
        spin_lock(&dev->lock);
    }

    ep->desc = NULL;
    vhub_ep->enabled = false;
    vhub_ep->halted = false;
    vhub_ep->wedged = false;
    vhub_ep->delay_start = false;
    spin_unlock(&dev->lock);

    return 0;
}

static int ls_pcie_vhub_ep_gadget_giveback(struct ls_pcie_vhub_ep *ep,struct ls_pcie_vhub_req *req, int status)
{
    list_del_init(&req->node);
    req->req.status = status;

    spin_unlock(&ep->dev->lock);

    ls_pcie_vhub_unmap_request(ep, req);
    usb_gadget_giveback_request(&ep->ep, &req->req);

    spin_lock(&ep->dev->lock);

    return 0;
}

static int ls_pcie_vhub_epn_enable(struct usb_ep *ep, const struct usb_endpoint_descriptor *desc)
{
    struct ls_pcie_vhub_ep *vhub_ep = container_of(ep, struct ls_pcie_vhub_ep, ep);
    printk("ls_pcie_vhub_epn_enable: ep = %p, desc->bEndpointAddress = 0x%02x\n",
           ep, desc->bEndpointAddress);

    vhub_ep->enabled = true;
    vhub_ep->dir = !!(desc->bEndpointAddress & USB_DIR_IN);
    vhub_ep->halted = false;
    vhub_ep->wedged = false;
    vhub_ep->ep.address = desc->bEndpointAddress;
    vhub_ep->ep.desc = desc;
    return 0;
}

static int ls_pcie_vhub_ep_stall(struct ls_pcie_vhub_ep *vhub_ep)
{
    struct ls_pcie_vusb_ep_req *ep_req = &vhub_ep->ep_req;

    ep_req->buf = NULL;
    ep_req->dma = 0;
    ep_req->addr = vhub_ep->dev->addr;
    ep_req->ep = vhub_ep->ep.address;
    ep_req->actual = 0;
    ep_req->len = 0;
    ep_req->vusb = vhub_ep->vhub->dev.vusb;
    ep_req->complete = NULL;
    ep_req->stall = 1;

    return ls_pcie_vusb_req_submit(ep_req);
}

static int ls_pcie_vhub_epn_set_halt_and_wedge(struct usb_ep *ep, bool halt, bool wedge)
{
    struct ls_pcie_vhub_ep *vhub_ep = container_of(ep, struct ls_pcie_vhub_ep, ep);
    struct ls_pcie_vhub_dev *dev = vhub_ep->dev;
    int ret = 0;

    if (!dev)
        return -ESHUTDOWN;
    if (halt && !ep->desc)
        return -EINVAL;

    spin_lock(&dev->lock);
    if (halt) {

        if (vhub_ep->halted) {
            if (wedge)
                vhub_ep->wedged = true;
            spin_unlock(&dev->lock);
            return 0;
        }

        struct ls_pcie_vhub_req *head = next_request(&vhub_ep->req_list);

        if (head && head->state == VHUB_REQ_BUSY) {
            spin_unlock(&dev->lock);
            return -EAGAIN;
        }

        ret = ls_pcie_vhub_ep_stall(vhub_ep);
        if (ret) {
            spin_unlock(&dev->lock);
            return ret;
        }
        vhub_ep->halted = true;
        vhub_ep->wedged = wedge;
    } else {
        vhub_ep->halted = false;
        vhub_ep->wedged = false;

        if (vhub_ep->delay_start) {
            struct ls_pcie_vhub_req *vhub_req;

            vhub_ep->delay_start = false;
            vhub_req = next_request(&vhub_ep->req_list);
            if (!vhub_req) {
                spin_unlock(&dev->lock);
                return 0;
            }

            vhub_req->state = VHUB_REQ_BUSY;
            vhub_req->req.status = -EINPROGRESS;
            vhub_req->req.actual = 0;
            ret = vhub_epn_kick_transfer(vhub_ep);
            if (ret) {
                list_del_init(&vhub_req->node);
                vhub_req->state = VHUB_REQ_IDLE;
                vhub_req->req.status = 0;
            }
        }
    }
    spin_unlock(&dev->lock);

    return ret;
}

static int ls_pcie_vhub_epn_set_halt(struct usb_ep *ep, int value)
{
    return ls_pcie_vhub_epn_set_halt_and_wedge(ep, !!value, false);
}

static int ls_pcie_vhub_epn_set_wedge(struct usb_ep *ep)
{
    return ls_pcie_vhub_epn_set_halt_and_wedge(ep, true, true);
}

int ls_pcie_vhub_ep_req_complete(struct ls_pcie_vusb_ep_req *req)
{
    struct ls_pcie_vhub_req *vhub_req = container_of(req, struct ls_pcie_vhub_req, ep_req);
    struct ls_pcie_vhub_ep *vhub_ep = vhub_req->vhub_ep;
    struct ls_pcie_vhub_dev *vhub_dev = vhub_ep->dev;
    int ret = 0;

    spin_lock(&vhub_dev->lock);

    if (vhub_req->state == VHUB_REQ_FREED) {
        spin_unlock(&vhub_dev->lock);
        kfree(vhub_req);
        return 0;
    }

    if (vhub_req->state != VHUB_REQ_BUSY) {
        int status = (vhub_req->state == VHUB_REQ_DEQUEUE) ? -ECONNRESET : -ESHUTDOWN;
        vhub_req->state = VHUB_REQ_IDLE;
        ls_pcie_vhub_ep_gadget_giveback(vhub_ep, vhub_req, status);
        spin_unlock(&vhub_dev->lock);
        return 0;
    }

    vhub_req->state = VHUB_REQ_IDLE;

    if (vhub_dev->enabled == false) {
        ls_pcie_vhub_ep_gadget_giveback(vhub_ep, vhub_req, -ESHUTDOWN);
        spin_unlock(&vhub_dev->lock);
        return 0;
    }

    bool more_queued = !list_is_singular(&vhub_ep->req_list);

    vhub_req->req.actual = req->actual;

    ls_pcie_vhub_ep_gadget_giveback(vhub_ep, vhub_req, req->status);

    if (more_queued) {
        vhub_req = next_request(&vhub_ep->req_list);
        if (vhub_req) {
            if (vhub_ep->halted || vhub_ep->wedged) {
                vhub_ep->delay_start = true;
            } else {
                vhub_req->state = VHUB_REQ_BUSY;
                ret = vhub_epn_kick_transfer(vhub_ep);
            }
        }
    }

    spin_unlock(&vhub_dev->lock);

    return ret;
}

static int ls_pcie_vhub_epn_queue(struct usb_ep *ep, struct usb_request *req, gfp_t gfp_flags)
{
    struct ls_pcie_vhub_req *vhub_req = container_of(req, struct ls_pcie_vhub_req, req);
    struct ls_pcie_vhub_ep *vhub_ep = container_of(ep, struct ls_pcie_vhub_ep, ep);
    struct ls_pcie_vhub_dev *vhub_dev = vhub_ep->dev;
    int ret = 0;

    spin_lock(&vhub_dev->lock);

    if (vhub_dev->enabled == false) {
        spin_unlock(&vhub_dev->lock);
        return -ESHUTDOWN;
    }

    req->status = -EINPROGRESS;
    req->actual = 0;

    list_add_tail(&vhub_req->node, &vhub_ep->req_list);
    if (next_request(&vhub_ep->req_list) != vhub_req) {
        vhub_req->state = VHUB_REQ_IDLE;
        spin_unlock(&vhub_dev->lock);
        return 0;
    }

    if (vhub_ep->halted || vhub_ep->wedged) {
        vhub_req->state = VHUB_REQ_IDLE;
        vhub_ep->delay_start = true;
        spin_unlock(&vhub_dev->lock);
        return 0;
    }

    vhub_req->state = VHUB_REQ_BUSY;

    ret = vhub_epn_kick_transfer(vhub_ep);
    if (ret) {
        list_del_init(&vhub_req->node);
        vhub_req->state = VHUB_REQ_IDLE;
        req->status = 0;
    }

    spin_unlock(&vhub_dev->lock);

    return ret;
}

static int ls_pcie_vhub_epn_dequeue(struct usb_ep *ep, struct usb_request *req)
{
    struct ls_pcie_vhub_ep *vhub_ep = container_of(ep, struct ls_pcie_vhub_ep, ep);
    struct ls_pcie_vhub_dev *vhub_dev = vhub_ep->dev;
    struct ls_pcie_vhub_req *dq_req = container_of(req, struct ls_pcie_vhub_req, req);
    struct ls_pcie_vhub_req *vhub_req, *tmp;

    printk("epn_dequeue: ep->address = 0x%x, req->length = %u\n", ep->address, req->length);

    spin_lock(&vhub_dev->lock);
    list_for_each_entry_safe(vhub_req, tmp, &vhub_ep->req_list, node) {
        if (vhub_req == dq_req) {
            bool req_busy = (dq_req->state == VHUB_REQ_BUSY);

            list_del_init(&dq_req->node);
            if (req_busy && ls_pcie_vusb_req_cancel(&dq_req->ep_req) != 0) {
                dq_req->state = VHUB_REQ_DEQUEUE;
                spin_unlock(&vhub_dev->lock);
                return 0;
            }
            dq_req->state = VHUB_REQ_IDLE;
            spin_unlock(&vhub_dev->lock);

            if (req_busy)
                ls_pcie_vhub_unmap_request(vhub_ep, dq_req);
            req->status = -ECONNRESET;
            usb_gadget_giveback_request(ep, req);
            return 0;
        }
    }
    spin_unlock(&vhub_dev->lock);

    return -EINVAL;
}

static const struct usb_ep_ops ls_pcie_vhub_epn_ops = {
    .enable = ls_pcie_vhub_epn_enable,
    .disable = ls_pcie_vhub_epn_disable,
    .queue = ls_pcie_vhub_epn_queue,
    .dequeue = ls_pcie_vhub_epn_dequeue,
    .set_halt = ls_pcie_vhub_epn_set_halt,
    .set_wedge = ls_pcie_vhub_epn_set_wedge,
    .alloc_request = ls_pcie_vhub_alloc_request,
    .free_request = ls_pcie_vhub_free_request,
};

static struct ls_pcie_vhub_ep *ls_pcie_vhub_ep_alloc(struct ls_pcie_vhub_dev *dev)
{
    struct ls_pcie_vhub_ep *ep;

    if (!dev || !dev->vhub)
        return NULL;

    ep = kzalloc(sizeof(*ep), GFP_KERNEL);
    if (!ep)
        return NULL;

    ep->dev = dev;
    ep->vhub = dev->vhub;
    INIT_LIST_HEAD(&ep->req_list);
    INIT_LIST_HEAD(&ep->ep.ep_list);
    INIT_LIST_HEAD(&ep->dev_node);

    list_add_tail(&ep->dev_node, &dev->ep_allocated);
    dev->ep_count++;
    return ep;
}

void ls_pcie_vhub_epn_free(struct ls_pcie_vhub_ep *ep)
{
    struct ls_pcie_vhub_dev *dev;

    if (!ep || !ep->vhub)
        return;

    dev = ep->dev;

    if (dev) {
        if (!list_empty(&ep->dev_node))
            list_del_init(&ep->dev_node);
        if (dev->ep_count)
            dev->ep_count--;
    }

    kfree(ep);
}

struct ls_pcie_vhub_ep *ls_pcie_vhub_epn_alloc(struct ls_pcie_vhub_dev *dev)
{
    struct ls_pcie_vhub_ep *ep = NULL;
    uint8_t idx;

    idx = dev->ep_count;

    ep = ls_pcie_vhub_ep_alloc(dev);
    if (!ep)
        return NULL;

    snprintf(ep->name, sizeof(ep->name), "p%d-ep%d", dev->port_index, idx);
    ep->ep.name = ep->name;
    ep->ep.maxpacket = 512;
    ep->ep.ops = &ls_pcie_vhub_epn_ops;
    ep->ep.caps.type_bulk = true;
    ep->ep.caps.type_int = true;
    ep->ep.caps.dir_in = true;
    ep->ep.caps.dir_out = true;
    usb_ep_set_maxpacket_limit(&ep->ep, 512);
    list_add_tail(&ep->ep.ep_list, &dev->gadget->ep_list);

    return ep;
}

struct usb_request *ls_pcie_vhub_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
    struct ls_pcie_vhub_req *req;
    struct ls_pcie_vhub_ep *vep = container_of(ep, struct ls_pcie_vhub_ep, ep);

    req = kzalloc(sizeof(*req), gfp_flags);
    if (!req)
        return NULL;

    req->vhub_ep = vep;
    req->state = VHUB_REQ_IDLE;
    req->ep_req.vusb = vep->vhub->dev.vusb;

    return &req->req;
}

void ls_pcie_vhub_free_request(struct usb_ep *ep, struct usb_request *req)
{
    struct ls_pcie_vhub_req *vhub_req = container_of(req, struct ls_pcie_vhub_req, req);
    struct ls_pcie_vhub_ep *vhub_ep = container_of(ep, struct ls_pcie_vhub_ep, ep);
    struct ls_pcie_vhub_dev *dev = vhub_ep->dev;

    spin_lock(&dev->lock);
    if (vhub_req->state != VHUB_REQ_IDLE &&
        ls_pcie_vusb_req_cancel(&vhub_req->ep_req) != 0) {
        vhub_req->state = VHUB_REQ_FREED;
        spin_unlock(&dev->lock);
        return;
    }
    spin_unlock(&dev->lock);

    kfree(vhub_req);
}
