#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/usb/composite.h>
#include "debugfs.h"
#include "vhub.h"
#include "trace.h"

#define gadget_to_vdev(g)           (dev_get_platdata(&g->dev))

static void ep0_internal_complete(struct usb_ep *ep, struct usb_request *req)
{
    /* internal control replies don't need upper-layer notification */
}

struct usb_request *ls_vhub_ep_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
    /* alloc request */
    struct ls_vhub_ep *epn = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub_req *req = kzalloc(sizeof(struct ls_vhub_req), gfp_flags);
    if (!req)
        return NULL;

    req->internel = false;
    req->dir = epn->number & 0x1;

    trace_ls_vhub_ep_alloc_request(epn, req);

    return &req->request;
}

void ls_vhub_ep_free_request(struct usb_ep *ep, struct usb_request *request)
{
    struct ls_vhub_req *req = container_of(request, struct ls_vhub_req, request);

    trace_ls_vhub_ep_free_request(container_of(ep, struct ls_vhub_ep, endpoint), req);

    kfree(req);
}

static int ep0_delegate_req(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    int ret;

    if (!dev->driver || !dev->driver->setup)
        return -ENODEV;

    spin_unlock(&dev->vhub->lock);
    ret = dev->driver->setup(dev->gadget, ctrl);
    spin_lock(&dev->vhub->lock);

    return ret;
}

static int ep0_reply(struct ls_vhub_dev *dev, const void *data, u16 len)
{
    struct ls_vhub_req *req = &dev->ep0.req;

    if (len > dev->ep0.data_len)
        return -EINVAL;

    memcpy(dev->ep0.data, data, len);
    req->request.buf = dev->ep0.data;
    req->request.dma = dev->ep0.data_dma;
    req->request.length = len;
    req->request.complete = ep0_internal_complete;
    req->dma_mapped = false;
    req->internel = true;

    return ls_vhub_ep0_queue_req(dev->ep0.out, req);
}

static int ep0_set_address(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    u16 wValue = le16_to_cpu(ctrl->wValue);
    u16 wIndex = le16_to_cpu(ctrl->wIndex);
    u16 wLength = le16_to_cpu(ctrl->wLength);

    if (wIndex || wLength || wValue > 127)
        return -EINVAL;

    dev->address = wValue & 0x7f;
    trace_ls_vhub_ep0_set_address(dev, dev->address);

    dev_dbg(dev->udev, "port %d set address %d\n", dev->port_index, dev->address);

    /* enable ep0 */
    ls_vhub_ep_enable(dev->ep0.out);
    ls_vhub_ep_enable(dev->ep0.in);
    /* map ep0 */
    ls_vhub_ep_map(dev->ep0.out);
    ls_vhub_ep_map(dev->ep0.in);

    /* ep0 start transfer, prepare receive setup buffer */
    ls_vhub_ep_prepare_trb(dev->ep0.out->trb->trb, dev->ep0.out->trb_dma, 8, DWC3_TRBCTL_CONTROL_SETUP, false);
    ls_vhub_ep_start_transfer(dev->ep0.out, dev->ep0.out->trb_dma, 0);

    usb_gadget_set_state(dev->gadget, USB_STATE_ADDRESS);
    return 0;
}

static struct ls_vhub_ep *_wIndex_to_dep(struct ls_vhub_dev *dev, __le16 wIndex_le)
{
    struct ls_vhub_ep *ep, *tmp;
    u8 address = le16_to_cpu(wIndex_le);

    list_for_each_entry_safe(ep, tmp, &dev->ep_list, dev_node) {
        if (ep->endpoint.address == address)
            return ep;
    }
    return NULL;
}

static int ep0_get_status(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    u16 status = 0;
    u16 wLength = le16_to_cpu(ctrl->wLength);
    struct ls_vhub_ep *ep;

    switch (ctrl->bRequestType & USB_RECIP_MASK) {
    case USB_RECIP_DEVICE:
        if (dev->gadget->is_selfpowered)
            status |= BIT(0);
        break;
    case USB_RECIP_INTERFACE:
        break;
    case USB_RECIP_ENDPOINT:
        ep = _wIndex_to_dep(dev, ctrl->wIndex);
        if (!ep)
            return -EINVAL;
        if (ep->stall)
            status |= 1;
        break;
    default:
        return -EINVAL;
    }

    status = cpu_to_le16(status);
    return ep0_reply(dev, &status, min_t(u16, wLength, 2));
}

static int ep0_set_config(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    int ret;
    enum usb_device_state state = dev->gadget->state;
    u32 cfg = le16_to_cpu(ctrl->wValue);

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

static int dwc3_ep0_handle_device(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl, int set)
{
    u32 wValue;
    u32 wIndex;
    int ret = 0;

    wValue = le16_to_cpu(ctrl->wValue);
    wIndex = le16_to_cpu(ctrl->wIndex);

    switch (wValue) {
    case USB_DEVICE_REMOTE_WAKEUP:
        break;
    case USB_DEVICE_U1_ENABLE:
        break;
    case USB_DEVICE_U2_ENABLE:
        break;
    case USB_DEVICE_LTM_ENABLE:
        ret = -EINVAL;
        break;
    case USB_DEVICE_TEST_MODE:
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static int dwc3_ep0_handle_intf(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl, int set)
{
    u32 wValue = le16_to_cpu(ctrl->wValue);

    switch (wValue) {
    case USB_INTRF_FUNC_SUSPEND:
        return ep0_delegate_req(dev, ctrl);
    }

    return -EINVAL;
}

static int ls_vhub_ep_remove_req(struct ls_vhub_ep *ep, int status)
{
    while (!list_empty(&ep->req_list)) {
        struct ls_vhub_req *req = next_request(&ep->req_list);
        ls_vhub_ep_gadget_giveback(ep, req, status);
    }
    return 0;
}

static void ls_vhub_ep_shutdown(struct ls_vhub_ep *ep)
{
    ls_vhub_ep_stop_transfer(ep);
    if (ep->stall && !ep->ctrl)
        ls_vhub_ep_clear_stall(ep);
    ls_vhub_ep_disable(ep);
    ep->stall = 0;
    ep->wedge = 0;
    ep->delay_start = 0;
    ls_vhub_ep_unmap(ep);
    ls_vhub_ep_remove_req(ep, -ESHUTDOWN);
}

static void ls_vhub_dev_reset_ep0(struct ls_vhub_dev *dev)
{
    dev->address = 0;
    dev->delayed_status = 0;
    dev->ep0.state = EP0_SETUP_PHASE;
    dev->ep0.three_stage_setup = false;
    dev->ep0.expect_in = false;

    ls_vhub_ep_shutdown(dev->ep0.in);
    ls_vhub_ep_shutdown(dev->ep0.out);
}

static void ls_vhub_dev_notify_reset(struct ls_vhub_dev *dev)
{
    struct ls_vhub *vhub = dev->vhub;

    trace_ls_vhub_dev_notify_reset(dev);

    if (!dev->driver || !dev->enabled)
        return;

    spin_unlock(&vhub->lock);
    usb_gadget_udc_reset(dev->gadget, dev->driver);
    spin_lock(&vhub->lock);
}

static bool ls_vhub_dev_port_connected(struct ls_vhub_dev *dev)
{
    struct ls_pcie_vhub_port *port = &dev->vhub->ports[dev->port_index];

    return !!(port->portsc.status & USB_PORT_STAT_CONNECTION);
}

static void ls_vhub_dev_handle_reset(struct ls_vhub_dev *dev, bool keep_enabled)
{
    ls_vhub_dev_notify_reset(dev);
    ls_vhub_dev_reset_ep0(dev);

    if (!ls_vhub_dev_port_connected(dev))
        dev->enabled = 0;
    else
        dev->enabled = keep_enabled ? (dev->driver != NULL) : 0;

    usb_gadget_set_state(dev->gadget, USB_STATE_DEFAULT);
}

static void ls_vhub_dev_ep0_dma_free(struct ls_vhub_dev *dev)
{
    struct device *dma_dev = &dev->vhub->pdev->dev;

    if (dev->ep0.data) {
        dma_free_coherent(dma_dev, dev->ep0.data_len, dev->ep0.data, dev->ep0.data_dma);
        dev->ep0.data = NULL;
        dev->ep0.data_dma = 0;
    }
}

static int ls_vhub_dev_ep0_dma_alloc(struct ls_vhub_dev *dev)
{
    struct device *dma_dev = &dev->vhub->pdev->dev;

    dev->ep0.data_len = 64;
    dev->ep0.data = dma_alloc_coherent(dma_dev, dev->ep0.data_len, &dev->ep0.data_dma, GFP_KERNEL);

    if (!dev->ep0.data) {
        ls_vhub_dev_ep0_dma_free(dev);
        return -ENOMEM;
    }

    return 0;
}

static int dwc3_ep0_handle_endpoint(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl, int set)
{
    u32 wValue;
    wValue = le16_to_cpu(ctrl->wValue);
    struct ls_vhub_ep *epn;

    switch (wValue) {
    case USB_ENDPOINT_HALT:
        epn = _wIndex_to_dep(dev, ctrl->wIndex);
        if (epn == NULL)
            return -EINVAL;

        if (set)
            return ls_vhub_ep_set_halt(epn, set);
        if (epn->wedge)
            return 0;

        return ls_vhub_ep_set_halt(epn, set);
    default:
        return -EINVAL;
    }
}

static int ep0_handle_feature(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl, int set)
{
    u32 recip;
    int ret;

    recip = ctrl->bRequestType & USB_RECIP_MASK;

    switch (recip) {
    case USB_RECIP_DEVICE:
        ret = dwc3_ep0_handle_device(dev, ctrl, set);
        break;
    case USB_RECIP_INTERFACE:
        ret = dwc3_ep0_handle_intf(dev, ctrl, set);
        break;
    case USB_RECIP_ENDPOINT:
        ret = dwc3_ep0_handle_endpoint(dev, ctrl, set);
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static int ep0_std_req(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    int ret = 0;
    switch (ctrl->bRequest) {
    case USB_REQ_GET_STATUS:
        ret = ep0_get_status(dev, ctrl);
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
        ret = -EOPNOTSUPP;
        break;
    case USB_REQ_SET_ISOCH_DELAY:
        ret = -EOPNOTSUPP;
        break;
    default:
        if (ctrl->bRequest == USB_REQ_GET_DESCRIPTOR && (ctrl->wValue >> 8) == USB_DT_DEBUG) {
            return -EOPNOTSUPP;
        }
        ret = ep0_delegate_req(dev, ctrl);
    }

    return ret;
}

int ls_vhub_dev_setup(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl)
{
    int ret;

    if (!dev->enabled)
        return -ENODEV;

    if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
        ret = ep0_std_req(dev, ctrl);
    else
        ret = ep0_delegate_req(dev, ctrl);

    if (ret == USB_GADGET_DELAYED_STATUS) {
        vhub_dbg_dev(dev, "ep0 setup USB_GADGET_DELAYED_STATUS\n");
        dev->delayed_status = 1;
    }

    if (ret < 0) {
        vhub_dbg_dev(dev, "ep0 setup ret %d\n", ret);
    }

    return ret;
}

static void ls_vhub_dev_release(struct device *dev)
{
    kfree(dev);
}

static int ls_vhub_epn_shutdown(struct ls_vhub_dev *dev)
{
    struct ls_vhub_ep *ep, *tmp;
    list_for_each_entry_safe(ep, tmp, &dev->ep_list, dev_node) {
        if (!ep->ctrl)
            ls_vhub_ep_shutdown(ep);
    }

    return 0;
}

static void ls_vhub_dev_cleanup_epn(struct ls_vhub_dev *dev)
{
    struct ls_vhub_ep *ep, *tmp;
    unsigned long flags;

    ls_vhub_lock(dev->vhub, &flags);
    list_for_each_entry_safe(ep, tmp, &dev->ep_list, dev_node) {
        if (ep->ctrl)
            continue;
        list_del_init(&ep->dev_node);
        list_del_init(&ep->endpoint.ep_list);
        ls_vhub_ep_free(&dev->vhub->ep_pool, ep);
    }
    ls_vhub_unlock(dev->vhub, &flags);
}

static int udc_pullup(struct usb_gadget *gadget, int is_on)
{
    struct ls_vhub_dev *dev = gadget_to_vdev(gadget);
    struct ls_vhub *vhub = dev->vhub;
    unsigned long flags;

    trace_ls_vhub_udc_pullup(dev, is_on);

    if (is_on) {
        ls_vhub_lock(vhub, &flags);
        ls_vhub_attach(vhub, dev->port_index);
        ls_vhub_unlock(vhub, &flags);
        return 0;
    }

    /* dev disable */
    WRITE_ONCE(dev->enabled, 0);
    /* wait irq thread exit */
    synchronize_irq(vhub->irq);

    ls_vhub_lock(vhub, &flags);
    /* detach host */
    ls_vhub_detach(vhub, dev->port_index);
    /* epn shutdown */
    ls_vhub_epn_shutdown(dev);
    /* ep0 reset */
    ls_vhub_dev_reset_ep0(dev);
    /* reset dev ep0 */
    usb_gadget_set_state(dev->gadget, USB_STATE_NOTATTACHED);
    ls_vhub_unlock(vhub, &flags);

    return 0;
}

static int udc_start(struct usb_gadget *gadget, struct usb_gadget_driver *driver)
{
    struct ls_vhub_dev *dev = gadget_to_vdev(gadget);
    struct ls_vhub *vhub = dev->vhub;
    unsigned long flags;

    trace_ls_vhub_udc_start(dev, driver);

    ls_vhub_lock(vhub, &flags);
    dev->driver = driver;
    dev->gadget->is_selfpowered = 1;
    ls_vhub_unlock(vhub, &flags);

    return 0;
}

static int udc_stop(struct usb_gadget *gadget)
{
    struct ls_vhub_dev *dev = gadget_to_vdev(gadget);
    struct ls_vhub *vhub = dev->vhub;
    unsigned long flags;

    trace_ls_vhub_udc_stop(dev);

    ls_vhub_lock(vhub, &flags);
    dev->enabled = 0;
    dev->delayed_status = 0;
    dev->driver = NULL;
    ls_vhub_unlock(vhub, &flags);
    /* free epn trb dma buffer */
    ls_vhub_dev_cleanup_epn(dev);

    return 0;
}

static struct usb_ep *udc_match_ep(struct usb_gadget *gadget, struct usb_endpoint_descriptor *desc, struct usb_ss_ep_comp_descriptor *ss_desc)
{
    struct ls_vhub_dev *dev = gadget_to_vdev(gadget);
    struct ls_vhub *vhub = dev->vhub;
    struct ls_vhub_ep *epn = NULL;
    struct usb_ep *ep;
    unsigned long flags;

    trace_ls_vhub_udc_match_ep(dev, desc);

    ls_vhub_lock(vhub, &flags);
    list_for_each_entry(ep, &gadget->ep_list, ep_list) {
        if (usb_gadget_ep_match_desc(gadget, ep, desc, ss_desc)) {
            ls_vhub_unlock(vhub, &flags);
            return ep;
        }
    }

    epn = ls_vhub_ep_alloc(&vhub->ep_pool, usb_endpoint_dir_in(desc));
    ls_vhub_unlock(vhub, &flags);
    if (!epn)
        return NULL;

    trace_ls_vhub_udc_ep_alloc(dev, epn->number, desc);

    if (ls_vhub_epn_init(vhub, dev, epn))
        goto err_free;

    ls_vhub_lock(vhub, &flags);
    list_add_tail(&epn->dev_node, &dev->ep_list);
    list_add_tail(&epn->endpoint.ep_list, &gadget->ep_list);
    ls_vhub_unlock(vhub, &flags);

    dev_dbg(dev->udev, "allocated new endpoint [%d]\n", epn->number);
    return &epn->endpoint;

err_free:
    ls_vhub_lock(vhub, &flags);
    ls_vhub_ep_free(&vhub->ep_pool, epn);
    ls_vhub_unlock(vhub, &flags);

    return NULL;
}

int ls_vhub_ep_gadget_giveback(struct ls_vhub_ep *ep, struct ls_vhub_req *req, int status)
{
    req->request.status = status;
    list_del_init(&req->node);

    trace_ls_vhub_ep_gadget_giveback(ep, req, status);

    if (ep->dev && req->dma_mapped) {
        /* unmap request dma buffer */
        usb_gadget_unmap_request_by_dev(&ep->vhub->pdev->dev, &req->request, req->dir);
        req->dma_mapped = false;
    }

    /* notify upper layer */
    spin_unlock(&ep->vhub->lock);
    usb_gadget_giveback_request(&ep->endpoint, &req->request);
    spin_lock(&ep->vhub->lock);

    return 0;
}

void ls_vhub_dev_bus_reset(struct ls_vhub_dev *dev)
{
    trace_ls_vhub_dev_bus_reset(dev);

    ls_vhub_dev_handle_reset(dev, false);
}

void ls_vhub_dev_disconnect(struct ls_vhub_dev *dev)
{
    struct ls_vhub *vhub = dev->vhub;

    trace_ls_vhub_dev_disconnect(dev);

    if (dev->driver && dev->driver->disconnect) {
        spin_unlock(&vhub->lock);
        dev->driver->disconnect(dev->gadget);
        spin_lock(&vhub->lock);
    }
}

void ls_vhub_dev_suspend(struct ls_vhub_dev *dev)
{
    struct ls_vhub *vhub = dev->vhub;

    trace_ls_vhub_dev_suspend(dev);

    if (!dev->enabled || !dev->driver || !dev->driver->suspend)
        return;

    spin_unlock(&vhub->lock);
    dev->driver->suspend(dev->gadget);
    spin_lock(&vhub->lock);
}

void ls_vhub_dev_resume(struct ls_vhub_dev *dev)
{
    struct ls_vhub *vhub = dev->vhub;

    trace_ls_vhub_dev_resume(dev);

    if (!dev->enabled || !dev->driver || !dev->driver->resume)
        return;

    spin_unlock(&vhub->lock);
    dev->driver->resume(dev->gadget);
    spin_lock(&vhub->lock);
}

int ls_vhub_dev_reset(struct ls_vhub *vhub, u8 port_idx)
{
    struct ls_pcie_vhub_port *port = &vhub->ports[port_idx];
    struct ls_vhub_dev *dev = &port->dev;

    trace_ls_vhub_dev_reset(vhub, port_idx);

    ls_vhub_dev_handle_reset(dev, true);

    return 0;
}

static void ls_vhub_gadget_release(struct device *dev)
{
    struct usb_gadget *gadget = container_of(dev, struct usb_gadget, dev);
    kfree(gadget);
}

static const struct usb_gadget_ops _gadget_ops = {
    .pullup = udc_pullup,
    .udc_start	= udc_start,
    .udc_stop	= udc_stop,
    .match_ep = udc_match_ep
};

int ls_vhub_dev_init(struct ls_vhub *vhub, u8 port_idx)
{
    struct ls_pcie_vhub_port *port = &vhub->ports[port_idx];
    struct ls_vhub_dev *dev = &port->dev;
    int ret = 0;

    trace_ls_vhub_dev_init(vhub, port_idx);

    port->portsc.status = USB_PORT_STAT_POWER;
    port->portsc.change = 0;

    dev->vhub = vhub;
    dev->port_index = port_idx;
    dev->address = 0;
    dev->common_ep0 = &vhub->common_ep0;

    INIT_LIST_HEAD(&dev->ep_list);

    dev->ep0.out = ls_vhub_ep_alloc(&vhub->ep_pool, 0);
    dev->ep0.in = ls_vhub_ep_alloc(&vhub->ep_pool, 1);
    dev->ep0.state = EP0_SETUP_PHASE;
    dev->ep0.req.internel = true;
    dev->ep0.req.dma_mapped = false;
    INIT_LIST_HEAD(&dev->ep0.req.node);

    ls_vhub_ep0_init(vhub, dev, dev->ep0.out);
    ls_vhub_ep0_init(vhub, dev, dev->ep0.in);

    ret = ls_vhub_dev_ep0_dma_alloc(dev);
    if (ret)
        goto err_dma;

    /* new udev instance */
    dev->udev = kzalloc(sizeof(struct device), GFP_KERNEL);
    if (!dev->udev) {
        ret = -ENOMEM;
        goto err_dma;
    }
    device_initialize(dev->udev);
    dev->udev->release = ls_vhub_dev_release;
    dev->udev->parent = &vhub->pdev->dev;
    dev_set_name(dev->udev, "ls-vhub:p%d", port_idx);

    ret = device_add(dev->udev);
    if (ret) {
        put_device(dev->udev);
        dev_err(&vhub->pdev->dev, "failed to add udev\n");
        goto err_dma;
    }

    /* new gadget instance */
    dev->gadget = kzalloc(sizeof(struct usb_gadget), GFP_KERNEL);
    if (!dev->gadget) {
        ret = -ENOMEM;
        goto err_udev;
    }
    INIT_LIST_HEAD(&dev->gadget->ep_list);
    /* gadget link vdev */
    dev->gadget->dev.platform_data = dev;
    dev->gadget->ops = &_gadget_ops;
    dev->gadget->ep0 = &dev->ep0.out->endpoint;
    dev->gadget->name = "ls-vhub-gadget";
    dev->gadget->max_speed = USB_SPEED_HIGH;
    dev->gadget->speed = USB_SPEED_HIGH;
    dev->gadget->dev.of_node = vhub->pdev->dev.of_node;
    dev->gadget->dev.of_node_reused = true;
    ret = usb_add_gadget_udc_release(dev->udev, dev->gadget, ls_vhub_gadget_release);
    if (ret)
        goto err_gadget;

    ls_vhub_dev_debugfs_init(dev);

    return 0;

err_gadget:
    kfree(dev->gadget);
    dev->gadget = NULL;
err_udev:
    device_unregister(dev->udev);
err_dma:
    ls_vhub_dev_ep0_dma_free(dev);
    ls_vhub_ep_free(&vhub->ep_pool, dev->ep0.in);
    ls_vhub_ep_free(&vhub->ep_pool, dev->ep0.out);

    return ret;
}

int ls_vhub_dev_deinit(struct ls_vhub *vhub, u8 port_idx)
{
    struct ls_pcie_vhub_port *port = &vhub->ports[port_idx];
    struct ls_vhub_dev *dev = &port->dev;

    usb_del_gadget_udc(dev->gadget);
    device_unregister(dev->udev);

    ls_vhub_dev_ep0_dma_free(dev);
    ls_vhub_ep_free(&vhub->ep_pool, dev->ep0.out);
    ls_vhub_ep_free(&vhub->ep_pool, dev->ep0.in);

    ls_vhub_dev_debugfs_exit(dev);

    return 0;
}
