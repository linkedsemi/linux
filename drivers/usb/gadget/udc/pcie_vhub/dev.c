#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/usb/composite.h>
#include "vhub.h"
#include "ep.h"

#define gadget_to_vdev(g)           (dev_get_platdata(&g->dev))

static int ls_pcie_vhub_ep0_map_req(struct ls_pcie_vhub *vhub, struct ls_pcie_vhub_req *req)
{
    return usb_gadget_map_request_by_dev(vhub->dev.dma_dev, &req->req, req->vhub_ep->dir);
}

static void ls_pcie_vhub_ep0_unmap_req(struct ls_pcie_vhub *vhub, struct ls_pcie_vhub_req *req)
{
    usb_gadget_unmap_request_by_dev(vhub->dev.dma_dev, &req->req, req->vhub_ep->dir);
}

static int ep0_delegate_req(struct ls_pcie_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    if (!dev->driver || !dev->driver->setup)
        return -ENODEV;
    return dev->driver->setup(dev->gadget, ctrl);
}

static int ep0_send_status(struct ls_pcie_vhub_dev *dev, uint16_t usb_status)
{
    struct ls_pcie_vhub_ep *ep = &dev->ep0;
    struct ls_pcie_vusb_ep_req *ep_req = &ep->ep_req;

    memcpy(ep->buf, &usb_status, sizeof(usb_status));
    ep_req->buf = ep->buf;
    ep_req->dma = ep->buf_dma;
    ep_req->addr = dev->addr;
    ep_req->ep = ep->ep.address;
    ep_req->actual = 0;
    ep_req->len = sizeof(usb_status);
    ep_req->vusb = dev->vhub->dev.vusb;
    ep_req->complete = NULL;
    ep_req->stall = 0;

    return ls_pcie_vusb_req_submit(ep_req);
}

static int ep0_send_zlp_status(struct ls_pcie_vhub_dev *dev)
{
    struct ls_pcie_vhub_ep *ep = &dev->ep0;
    struct ls_pcie_vusb_ep_req *ep_req = &ep->ep_req;

    ep_req->buf = NULL;
    ep_req->dma = 0;
    ep_req->addr = dev->addr;
    ep_req->ep = ep->ep.address;
    ep_req->actual = 0;
    ep_req->len = 0;
    ep_req->vusb = dev->vhub->dev.vusb;
    ep_req->complete = NULL;
    ep_req->stall = 0;

    return ls_pcie_vusb_req_submit(ep_req);
}

static int ep0_send_stall(struct ls_pcie_vhub_dev *dev)
{
    struct ls_pcie_vhub_ep *ep = &dev->ep0;
    struct ls_pcie_vusb_ep_req *ep_req = &ep->ep_req;

    ep_req->buf = NULL;
    ep_req->dma = 0;
    ep_req->addr = dev->addr;
    ep_req->ep = ep->ep.address;
    ep_req->actual = 0;
    ep_req->len = 0;
    ep_req->vusb = dev->vhub->dev.vusb;
    ep_req->complete = NULL;
    ep_req->stall = 1;

    return ls_pcie_vusb_req_submit(ep_req);
}


static struct ls_pcie_vhub_ep *ls_pcie_vhub_find_ep_by_addr(struct ls_pcie_vhub_dev *dev, uint8_t ep_addr)
{
    struct ls_pcie_vhub_ep *ep;

    if ((ep_addr & USB_ENDPOINT_NUMBER_MASK) == 0)
        return &dev->ep0;

    list_for_each_entry(ep, &dev->ep_allocated, dev_node) {
        if (ep == &dev->ep0)
            continue;
        if (ep->ep.address == ep_addr)
            return ep;
    }

    return NULL;
}

static int ep0_handle_status(struct ls_pcie_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    uint32_t recip;
    uint32_t value;
    uint16_t usb_status = 0;

    value = le16_to_cpu(ctrl->wValue);
    if (value != 0)
        return -EINVAL;

    recip = ctrl->bRequestType & USB_RECIP_MASK;
    switch (recip) {
    case USB_RECIP_DEVICE:
        usb_status = dev->gadget->is_selfpowered << USB_DEVICE_SELF_POWERED;
        return ep0_send_status(dev, usb_status);
    case USB_RECIP_INTERFACE:
        return ep0_delegate_req(dev, ctrl);
    case USB_RECIP_ENDPOINT:
    {
        uint8_t ep_addr = le16_to_cpu(ctrl->wIndex) & 0xff;
        struct ls_pcie_vhub_ep *ep = ls_pcie_vhub_find_ep_by_addr(dev, ep_addr);
        if (!ep)
            return -EINVAL;
        usb_status = ep->halted ? 1 : 0;
        return ep0_send_status(dev, usb_status);
    }
    default:
        return -EINVAL;
    }
}

static int ep0_handle_feature(struct ls_pcie_vhub_dev *dev, struct usb_ctrlrequest *ctrl, int set)
{
    uint8_t recip = ctrl->bRequestType & USB_RECIP_MASK;
    uint16_t wValue = le16_to_cpu(ctrl->wValue);
    uint16_t wIndex = le16_to_cpu(ctrl->wIndex);
    int ret = 0;

    if (recip == USB_RECIP_ENDPOINT) {
        uint8_t ep_addr = wIndex & 0xff;
        struct ls_pcie_vhub_ep *ep;

        if (wValue != USB_ENDPOINT_HALT)
            return -EOPNOTSUPP;

        ep = ls_pcie_vhub_find_ep_by_addr(dev, ep_addr);
        if (!ep)
            return -EINVAL;

        if (!set && ep->wedged)
            return ep0_send_zlp_status(dev);

        ret = ep->ep.ops->set_halt(&ep->ep, !!set);
        if (ret)
            return ep0_send_stall(dev);

        return ep0_send_zlp_status(dev);
    }

    if (recip == USB_RECIP_DEVICE) {
        if (wValue == USB_DEVICE_REMOTE_WAKEUP)
            return ep0_send_zlp_status(dev);
        return -EOPNOTSUPP;
    }

    return -EOPNOTSUPP;
}

static int ep0_set_address(struct ls_pcie_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    uint16_t wValue = le16_to_cpu(ctrl->wValue);
    uint16_t wIndex = le16_to_cpu(ctrl->wIndex);
    uint16_t wLength = le16_to_cpu(ctrl->wLength);

    if (wIndex || wLength || wValue > 127)
        return -EINVAL;

    dev->addr = wValue & 0x7f;
    printk("port %d set address %d\n", dev->port_index, dev->addr);
    usb_gadget_set_state(dev->gadget, USB_STATE_ADDRESS);
    return 0;
}

static int ep0_set_config(struct ls_pcie_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    int ret;
    enum usb_device_state state = dev->gadget->state;
    uint32_t cfg = le16_to_cpu(ctrl->wValue);

    switch (state) {
    case USB_STATE_DEFAULT:
        return -EINVAL;
    case USB_STATE_ADDRESS:
        ret = ep0_delegate_req(dev, ctrl);
        if (cfg && (!ret || ret == USB_GADGET_DELAYED_STATUS)) {
            if (ret == 0)
                usb_gadget_set_state(dev->gadget, USB_STATE_CONFIGURED);
        }
        break;
    case USB_STATE_CONFIGURED:
        ret = ep0_delegate_req(dev, ctrl);
        if (!cfg && !ret)
            usb_gadget_set_state(dev->gadget, USB_STATE_ADDRESS);
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static int ep0_std_req(struct ls_pcie_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    int ret;

    switch (ctrl->bRequest) {
    case USB_REQ_GET_STATUS:
        ret = ep0_handle_status(dev, ctrl);
        break;
    case USB_REQ_CLEAR_FEATURE:
        ret = ep0_handle_feature(dev, ctrl, 0);
        break;
    case USB_REQ_SET_FEATURE:
        ret = ep0_handle_feature(dev, ctrl, 1);
        break;
    case USB_REQ_SET_ADDRESS:
        ret = ep0_set_address(dev, ctrl);
        break;
    case USB_REQ_SET_CONFIGURATION:
        ret = ep0_set_config(dev, ctrl);
        break;
    case USB_REQ_SET_SEL:
        printk("USB_REQ_SET_SEL\n");
        ret = -EOPNOTSUPP;
        break;
    case USB_REQ_SET_ISOCH_DELAY:
        printk("USB_REQ_SET_ISOCH_DELAY\n");
        ret = -EOPNOTSUPP;
        break;
    default:
        if (ctrl->bRequest == USB_REQ_GET_DESCRIPTOR && (ctrl->wValue >> 8) == USB_DT_DEBUG) {
            return ep0_send_stall(dev);
        }
        ret = ep0_delegate_req(dev, ctrl);
        break;
    }

    return ret;
}

int ls_pcie_vhub_dev_setup(struct ls_pcie_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    int ret;

    printk("dev_setup: port=%d, type=0x%x, request=0x%x, value=0x%x, index=0x%x, length=%d\n",
           dev->port_index, ctrl->bRequestType, ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);

    dev->ep0.dir = !!(ctrl->bRequestType & USB_DIR_IN);

    if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
        ret = ep0_std_req(dev, ctrl);
    else
        ret = ep0_delegate_req(dev, ctrl);

    if (ret == USB_GADGET_DELAYED_STATUS) {
        dev->delayed_status = 1;
    } else if (ret < 0) {
        ret = ep0_send_stall(dev);
    }

    return ret;
}

static void ls_pcie_vhub_flush_ep_requests(struct ls_pcie_vhub_ep *vhub_ep, int status)
{
    struct ls_pcie_vhub_dev *dev = vhub_ep->dev;
    struct ls_pcie_vhub_req *vhub_req, *tmp;

    spin_lock(&dev->lock);
    list_for_each_entry_safe(vhub_req, tmp, &vhub_ep->req_list, node) {
        int req_busy = (vhub_req->state == VHUB_REQ_BUSY);

        list_del_init(&vhub_req->node);
        vhub_req->state = VHUB_REQ_DISABLE;
        spin_unlock(&dev->lock);

        if (req_busy) {
            if (ls_pcie_vusb_req_cancel(&vhub_req->ep_req) != 0) {
                spin_lock(&dev->lock);
                continue;
            }
            ls_pcie_vhub_ep0_unmap_req(vhub_ep->vhub, vhub_req);
        }

        vhub_req->state = VHUB_REQ_IDLE;
        vhub_req->req.status = status;
        usb_gadget_giveback_request(&vhub_ep->ep, &vhub_req->req);

        spin_lock(&dev->lock);
    }
    spin_unlock(&dev->lock);
}

int ls_pcie_vhub_dev_reset(struct ls_pcie_vhub_dev *dev)
{
    struct ls_pcie_vhub_ep *ep, *tmp;

    dev->addr = 0;
    dev->ep0.dir = 0;
    dev->delayed_status = 0;
    usb_gadget_set_state(dev->gadget, USB_STATE_DEFAULT);

    list_for_each_entry_safe(ep, tmp, &dev->ep_allocated, dev_node) {
        ls_pcie_vhub_flush_ep_requests(ep, -ESHUTDOWN);
        spin_lock(&dev->lock);
        ep->halted = false;
        ep->wedged = false;
        ep->delay_start = false;
        if (ep != &dev->ep0) {
            ep->ep.desc = NULL;
            ep->dir = 0;
        }
        spin_unlock(&dev->lock);
    }

    spin_lock(&dev->lock);
    if (dev->driver)
        dev->enabled = true;
    else
        dev->enabled = false;
    spin_unlock(&dev->lock);

    if (dev->enabled)
        usb_gadget_udc_reset(dev->gadget, dev->driver);

    return 0;
}

static int ls_pcie_vhub_ep0_queue(struct usb_ep *ep, struct usb_request *req, gfp_t gfp_flags)
{
    struct ls_pcie_vhub_req *vhub_req = container_of(req, struct ls_pcie_vhub_req, req);
    struct ls_pcie_vhub_ep *vhub_ep = container_of(ep, struct ls_pcie_vhub_ep, ep);
    struct ls_pcie_vhub_dev *dev = vhub_ep->dev;
    struct ls_pcie_vusb_ep_req *ep_req = &vhub_req->ep_req;
    int ret;

    spin_lock(&dev->lock);

    if (dev->enabled == false) {
        spin_unlock(&dev->lock);
        return -ESHUTDOWN;
    }

    if (dev->delayed_status) {
        dev->delayed_status = 0;
        usb_gadget_set_state(dev->gadget, USB_STATE_CONFIGURED);
    }

    list_add_tail(&vhub_req->node, &vhub_ep->req_list);

    vhub_req->state = VHUB_REQ_BUSY;
    req->status = -EINPROGRESS;
    req->actual = 0;

    ep_req->addr = dev->addr;
    ep_req->ep = vhub_ep->dir ? 0x80 : 0x0;
    ep_req->actual = 0;
    ep_req->vusb = vhub_ep->vhub->dev.vusb;
    ep_req->complete = ls_pcie_vhub_ep_req_complete;
    ep_req->len = req->length;

    if (req->length)
    {
        ret = ls_pcie_vhub_ep0_map_req(vhub_ep->vhub, vhub_req);
        if (ret) {
            list_del_init(&vhub_req->node);
            vhub_req->state = VHUB_REQ_IDLE;
            req->status = 0;
            spin_unlock(&dev->lock);
            return ret;
        }

        ep_req->buf = vhub_req->req.buf;
        ep_req->dma = vhub_req->req.dma;
    }
    else
    {
        ep_req->buf = NULL;
        ep_req->dma = 0;
    }

    ret = ls_pcie_vusb_req_submit(ep_req);
    if (ret) {
        if (req->length)
            ls_pcie_vhub_ep0_unmap_req(vhub_ep->vhub, vhub_req);
        list_del_init(&vhub_req->node);
        vhub_req->state = VHUB_REQ_IDLE;
        req->status = 0;
    }

    spin_unlock(&dev->lock);

    return ret;
}

static int ls_pcie_vhub_ep0_dequeue(struct usb_ep *ep, struct usb_request *req)
{
    struct ls_pcie_vhub_ep *vhub_ep = container_of(ep, struct ls_pcie_vhub_ep, ep);
    struct ls_pcie_vhub_dev *dev = vhub_ep->dev;
    struct ls_pcie_vhub_req *dq_req = container_of(req, struct ls_pcie_vhub_req, req);
    struct ls_pcie_vhub_req *vhub_req, *tmp;

    spin_lock(&dev->lock);
    list_for_each_entry_safe(vhub_req, tmp, &vhub_ep->req_list, node) {
        if (vhub_req == dq_req) {
            int req_busy = (dq_req->state == VHUB_REQ_BUSY);

            list_del_init(&dq_req->node);
            if (req_busy && ls_pcie_vusb_req_cancel(&dq_req->ep_req) != 0) {
                dq_req->state = VHUB_REQ_DEQUEUE;
                spin_unlock(&dev->lock);
                return 0;
            }
            dq_req->state = VHUB_REQ_IDLE;
            spin_unlock(&dev->lock);

            if (req_busy)
                ls_pcie_vhub_ep0_unmap_req(vhub_ep->vhub, dq_req);

            req->status = -ECONNRESET;
            usb_gadget_giveback_request(ep, req);
            return 0;
        }
    }
    spin_unlock(&dev->lock);

    return -EINVAL;
}

static int ls_pcie_vhub_ep0_set_halt(struct usb_ep *ep, int value)
{
    struct ls_pcie_vhub_ep *vhub_ep = container_of(ep, struct ls_pcie_vhub_ep, ep);
    vhub_ep->halted = !!value;
    if (!value)
        vhub_ep->wedged = false;
    return 0;
}

static const struct usb_ep_ops ls_pcie_vhub_ep0_ops = {
    .queue = ls_pcie_vhub_ep0_queue,
    .dequeue = ls_pcie_vhub_ep0_dequeue,
    .set_halt = ls_pcie_vhub_ep0_set_halt,
    .alloc_request = ls_pcie_vhub_alloc_request,
    .free_request = ls_pcie_vhub_free_request,
};

static void ls_pcie_vhub_dev_ep0_init(struct ls_pcie_vhub_dev *dev)
{
    struct ls_pcie_vhub_ep *ep0 = &dev->ep0;
    struct usb_ep *uep = &ep0->ep;

    ep0->dev = dev;
    ep0->vhub = dev->vhub;
    INIT_LIST_HEAD(&ep0->req_list);
    INIT_LIST_HEAD(&ep0->ep.ep_list);
    INIT_LIST_HEAD(&ep0->dev_node);
    list_add_tail(&ep0->dev_node, &dev->ep_allocated);
    dev->ep_count++;

    /* ep0 gadget init */
    uep->ops = &ls_pcie_vhub_ep0_ops;
    snprintf(ep0->name, sizeof(ep0->name), "ep0");
    uep->name = ep0->name;
    uep->address = 0;
    uep->desc = NULL;
    uep->caps.dir_in = true;
    uep->caps.dir_out = true;
    uep->caps.type_control = true;
    usb_ep_set_maxpacket_limit(uep, 64);
}

static void ls_pcie_vhub_dev_release(struct device *dev)
{
    kfree(dev);
}

static void ls_pcie_vhub_gadget_release(struct device *dev)
{
    struct usb_gadget *gadget = container_of(dev, struct usb_gadget, dev);
    kfree(gadget);
}

static int udc_pullup(struct usb_gadget* gadget, int on)
{
    struct ls_pcie_vhub_dev *dev = gadget_to_vdev(gadget);

    if (on) {
        ls_pcie_vhub_attach(dev->vhub, dev->port_index);
        return 0;
    }

    spin_lock(&dev->lock);
    dev->enabled = false;
    dev->addr = 0;
    spin_unlock(&dev->lock);
    ls_pcie_vhub_detach(dev->vhub, dev->port_index);

    return 0;
}

static struct usb_ep *udc_match_ep(struct usb_gadget *gadget,
                        struct usb_endpoint_descriptor *desc,
                        struct usb_ss_ep_comp_descriptor *ss)
{
    struct usb_ep *ep;
    struct ls_pcie_vhub_dev *dev = gadget_to_vdev(gadget);

    list_for_each_entry(ep, &gadget->ep_list, ep_list) {
        if (usb_gadget_ep_match_desc(gadget, ep, desc, ss)) {
            printk("list_for_each_entry: found ep = %p\n", ep);
            return ep;
        }
    }
    /* check desc */
    struct ls_pcie_vhub_ep *vhub_ep = ls_pcie_vhub_epn_alloc(dev);
    if (!vhub_ep)
        return NULL;
    vhub_ep->dir = !!(desc->bEndpointAddress & USB_DIR_IN);
    printk("ls_pcie_vhub_match_ep: found ep = %p, dir = %d, name = %s\n", &vhub_ep->ep, vhub_ep->dir, vhub_ep->ep.name);

    return &vhub_ep->ep;
}

static int udc_start(struct usb_gadget *gadget, struct usb_gadget_driver *driver)
{
    struct ls_pcie_vhub_dev *dev = gadget_to_vdev(gadget);

    printk("udc_start called, driver name = %s\n", driver->driver.name);

    spin_lock(&dev->lock);
    dev->driver = driver;
    spin_unlock(&dev->lock);

    return 0;
}

static int udc_stop(struct usb_gadget *gadget)
{
    struct ls_pcie_vhub_dev *dev = gadget_to_vdev(gadget);
    struct ls_pcie_vhub_ep *ep, *tmp;

    printk("udc_stop called\n");

    list_for_each_entry_safe(ep, tmp, &dev->ep_allocated, dev_node) {
        ls_pcie_vhub_flush_ep_requests(ep, -ESHUTDOWN);
        if (ep != &dev->ep0) {
            ep->ep.desc = NULL;
            ep->dir = 0;
        }
        ep->halted = false;
        ep->wedged = false;
        ep->delay_start = false;
    }

    spin_lock(&dev->lock);
    dev->driver = NULL;
    dev->enabled = false;
    spin_unlock(&dev->lock);

    dev->delayed_status = 0;
    usb_gadget_set_state(dev->gadget, USB_STATE_NOTATTACHED);

    return 0;
}

static int udc_set_selfpowered(struct usb_gadget *gadget, int is_selfpowered)
{
    gadget->is_selfpowered = is_selfpowered;
    return 0;
}

static const struct usb_gadget_ops ls_pcie_vhub_dev_udc_ops = {
    .set_selfpowered = udc_set_selfpowered,
    .pullup = udc_pullup,
    .match_ep = udc_match_ep,
    .udc_start	= udc_start,
    .udc_stop	= udc_stop,
};

int ls_pcie_vhub_dev_init(struct ls_pcie_vhub *vhub, uint8_t port_idx)
{
    struct ls_pcie_vhub_port *port = &vhub->ports[port_idx];
    struct ls_pcie_vhub_dev *dev = &port->dev;
    int ret;

    port->portsc.status = USB_PORT_STAT_POWER;
    port->portsc.change = 0;

    dev->port_index = port_idx;
    dev->vhub = vhub;
    dev->ep_count = 0;
    dev->addr = 0;
    dev->driver = NULL;
    dev->enabled = false;
    dev->delayed_status = 0;
    spin_lock_init(&dev->lock);

    INIT_LIST_HEAD(&dev->ep_allocated);

    dev->udc_dev = kzalloc(sizeof(struct device), GFP_KERNEL);
    if (!dev->udc_dev) {
        return -ENOMEM;
    }
    device_initialize(dev->udc_dev);
    dev->udc_dev->release = ls_pcie_vhub_dev_release;
    dev->udc_dev->parent = &dev->vhub->pdev->dev;
    dev_set_name(dev->udc_dev, "pcie-vhub:p%d", port_idx);

    ret = device_add(dev->udc_dev);
    if (ret) {
        dev->udc_dev = NULL;
        printk("failed to add udc device\n");
        return ret;
    }

    dev->gadget = kzalloc(sizeof(*dev->gadget), GFP_KERNEL);
    if (!dev->gadget) {
        ret = -ENOMEM;
        goto err_udc;
    }

    ls_pcie_vhub_dev_ep0_init(dev);

    dev->ep0.buf = dma_alloc_coherent(dev->vhub->dev.dma_dev, 64, &dev->ep0.buf_dma, GFP_KERNEL);
    if (!dev->ep0.buf) {
        ret = -ENOMEM;
        goto err_gadget;
    }
    dev->ep0.buf_len = 64;

    INIT_LIST_HEAD(&dev->gadget->ep_list);
    /* gadget link vdev */
    dev->gadget->dev.platform_data = dev;
    dev->gadget->ops = &ls_pcie_vhub_dev_udc_ops;
    dev->gadget->ep0 = &dev->ep0.ep;
    dev->gadget->name = KBUILD_MODNAME;
    dev->gadget->max_speed = USB_SPEED_HIGH;
    dev->gadget->speed = USB_SPEED_HIGH;
    dev->gadget->dev.of_node = vhub->pdev->dev.of_node;
    dev->gadget->dev.of_node_reused = true;
    ret = usb_add_gadget_udc_release(dev->udc_dev, dev->gadget, ls_pcie_vhub_gadget_release);
    if (ret) {
        dev->gadget = NULL;
        goto err_udc;
    }
    return 0;

err_gadget:
    kfree(dev->gadget);
    dev->gadget = NULL;
err_udc:
    device_unregister(dev->udc_dev);
    dev->udc_dev = NULL;
    return ret;
}

int ls_pcie_vhub_dev_deinit(struct ls_pcie_vhub *vhub, uint8_t port_idx)
{
    struct ls_pcie_vhub_port *port = &vhub->ports[port_idx];
    struct ls_pcie_vhub_dev *dev = &port->dev;
    struct ls_pcie_vhub_ep *ep, *tmp;

    list_for_each_entry_safe(ep, tmp, &dev->ep_allocated, dev_node) {
        ls_pcie_vhub_flush_ep_requests(ep, -ESHUTDOWN);
    }

    if (dev->gadget) {
        usb_del_gadget_udc(dev->gadget);
        dev->gadget = NULL;
    }
    if (dev->udc_dev)
        device_unregister(dev->udc_dev);

    if (dev->ep0.buf) {
        dma_free_coherent(dev->vhub->dev.dma_dev, dev->ep0.buf_len, dev->ep0.buf, dev->ep0.buf_dma);
        dev->ep0.buf = NULL;
    }

    if (dev->ep0.vhub)
        list_del_init(&dev->ep0.dev_node);

    while (!list_empty(&dev->ep_allocated)) {
        struct ls_pcie_vhub_ep *ep = list_first_entry(&dev->ep_allocated, struct ls_pcie_vhub_ep, dev_node);
        ls_pcie_vhub_epn_free(ep);
    }
    dev->driver = NULL;
    dev->enabled = false;
    dev->addr = 0;

    return 0;
}
