#include <linux/dma-mapping.h>
#include "trace.h"
#include "vhub.h"

#define DEPEVT_STATUS_CONTROL_DATA              1
#define DEPEVT_STATUS_CONTROL_STATUS            2

static struct usb_endpoint_descriptor ep0_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bmAttributes = USB_ENDPOINT_XFER_CONTROL,
    .wMaxPacketSize  = cpu_to_le16(64),
    .bEndpointAddress = 0
};

static inline struct ls_vhub_ctrl_ep *ep0_ctrl(struct ls_vhub_ep *ep)
{
    if (ep->common)
        return &ep->vhub->common_ep0;
    if (ep->dev)
        return &ep->dev->ep0;
    return &ep->vhub->ep0;
}

static inline struct ls_vhub_ep *ep0_xfer_out(struct ls_vhub_ep *ep)
{
    return (ep->number & 1) ? ep - 1 : ep;
}

static inline struct ls_vhub_ep *ep0_req_ep(struct ls_vhub_ep *ep)
{
    struct ls_vhub_ep *out = ep0_xfer_out(ep);
    struct ls_vhub_dev *dev = out->vhub->addr0_dev;

    if (out->common && dev)
        return dev->ep0.out;
    return out;
}

static inline struct ls_vhub_dev *ep0_owner_dev(struct ls_vhub_ep *ep)
{
    return ep->dev ? ep->dev : ep->vhub->addr0_dev;
}

static inline struct ls_vhub_ctrl_ep *ep0_dev_ctrl(struct ls_vhub_dev *dev)
{
    return (dev->vhub->addr0_dev == dev) ? dev->common_ep0 : &dev->ep0;
}

static int ls_vhub_ep0_out_start(struct ls_vhub_ep *ep)
{
    ls_vhub_ep_prepare_trb(&ep->trb->trb[0], ep->trb_dma, 8, DWC3_TRBCTL_CONTROL_SETUP, false);
    return ls_vhub_ep_start_transfer(ep, ep->trb_dma, 0);
}

int ls_vhub_ep0_stall_and_restart(struct ls_vhub_ep *ep)
{
    struct ls_vhub_ctrl_ep *ctrl_ep = ep0_ctrl(ep);

    if (!ep->enabled)
        return 0;

    /* stall ep0 */
    ls_vhub_ep_set_halt(ep, 1);
    /* switch setup state, prepare receive setup packet */
    ctrl_ep->state = EP0_SETUP_PHASE;

    trace_ls_vhub_ep0_stall_and_restart(ep);

    return ls_vhub_ep0_out_start(ep);
}

static void ls_vhub_ep0_xfer_status(struct ls_vhub *vhub, struct ls_vhub_ep *ep)
{
    struct ls_vhub_ctrl_ep *ctrl_ep = ep0_ctrl(ep);

    if (ctrl_ep->three_stage_setup == true) {
        /* three stage, send or receive zlp */
        ls_vhub_ep_prepare_trb(&ep->trb->trb[0], ep->trb_dma, 0, DWC3_TRBCTL_CONTROL_STATUS3, false);
    } else {
        ls_vhub_ep_prepare_trb(&ep->trb->trb[0], ep->trb_dma, 0, DWC3_TRBCTL_CONTROL_STATUS2, false);
    }
    ls_vhub_ep_start_transfer(ep, ep->trb_dma, 0);
}

static int ls_vhub_ep0_map_req(struct ls_vhub *vhub, struct ls_vhub_req *req)
{
    int ret;

    if (req->internel)
        return 0;

    ret = usb_gadget_map_request_by_dev(&vhub->pdev->dev, &req->request, req->dir);
    if (ret)
        return ret;

    req->dma_mapped = true;
    return 0;
}

static int ls_vhub_ep0_xfer_data(struct ls_vhub *vhub, struct ls_vhub_ep *ep, struct ls_vhub_req *req)
{
    /* start transfer */
    u32 trb_len = 0;
    int ret;

    vhub_dbg_ep(ep, "ep0 xfer data: length = %d, dir = %d, maxpacket = %d\n",
                req->request.length, req->dir, ep->endpoint.maxpacket);

    if (req->request.length == 0) {
        if (!req->dir)
            trb_len = ep->endpoint.maxpacket;
        ls_vhub_ep_prepare_trb(&ep->trb->trb[0], vhub->bounce_dma, trb_len, DWC3_TRBCTL_CONTROL_DATA, false);
    } else if (!IS_ALIGNED(req->request.length, ep->endpoint.maxpacket) && (req->dir == 0)) {
        u32 maxpacket = ep->endpoint.maxpacket;
        u32 rem = req->request.length % maxpacket;
        ret = ls_vhub_ep0_map_req(vhub, req);
        if (ret)
            return ret;
        ls_vhub_ep_prepare_trb(&ep->trb->trb[0], req->request.dma, req->request.length, DWC3_TRBCTL_CONTROL_DATA, true);
        /* align to mps */
        ls_vhub_ep_prepare_trb(&ep->trb->trb[1], vhub->bounce_dma, maxpacket - rem, DWC3_TRBCTL_CONTROL_DATA, false);
    } else if (IS_ALIGNED(req->request.length, ep->endpoint.maxpacket) && req->request.length && req->request.zero) {
        ret = ls_vhub_ep0_map_req(vhub, req);
        if (ret)
            return ret;
        ls_vhub_ep_prepare_trb(&ep->trb->trb[0], req->request.dma, req->request.length, DWC3_TRBCTL_CONTROL_DATA, true);
        if (!req->dir)
            trb_len = ep->endpoint.maxpacket;
        /* prepare zlp trb */
        ls_vhub_ep_prepare_trb(&ep->trb->trb[1], vhub->bounce_dma, trb_len, DWC3_TRBCTL_CONTROL_DATA, false);
    } else {
        /* trb length aligned mps or in direction */
        ret = ls_vhub_ep0_map_req(vhub, req);
        if (ret)
            return ret;
        ls_vhub_ep_prepare_trb(&ep->trb->trb[0], req->request.dma, req->request.length, DWC3_TRBCTL_CONTROL_DATA, false);
    }

    /* start transfer */
    ret = ls_vhub_ep_start_transfer(ep, ep->trb_dma, 0);
    if (ret) {
        dev_err(vhub_ep_dev(ep), "EP0 start transfer fail, %d!!\n", ret);
    }
    return ret;
}

static int ls_vhub_ep0_dispatch_setup(struct ls_vhub *vhub, struct ls_vhub_ep *ep,
                                      struct usb_ctrlrequest *setup_packet)
{
    if (ep->common) {
        if (vhub->address == 0)
            return ls_vhub_setup(vhub, setup_packet);

        if (!vhub->addr0_dev)
            return -ENODEV;

        return ls_vhub_dev_setup(vhub->addr0_dev, setup_packet);
    }

    if (ep->dev)
        return ls_vhub_dev_setup(ep->dev, setup_packet);

    return ls_vhub_setup(vhub, setup_packet);
}

static int ls_vhub_ep0_setup(struct ls_vhub *vhub, const union evt_buf *evt)
{
    int ret = -EINVAL;
    struct ls_vhub_ep *ep = &vhub->ep_pool.eps[evt->depevt.phy_ep_num];
    struct ls_vhub_ctrl_ep *ctrl_ep = ep0_ctrl(ep);
    struct usb_ctrlrequest *setup_packet = (struct usb_ctrlrequest *)ep->trb;

    trace_ls_vhub_ep0_setup(vhub, ctrl_ep, setup_packet);

    if (ep->dev && !ep->dev->enabled) {
        goto out;
    }

    ctrl_ep->three_stage_setup = setup_packet->wLength != 0;
    ctrl_ep->expect_in = ctrl_ep->three_stage_setup && !!(setup_packet->bRequestType & USB_DIR_IN);
    ret = ls_vhub_ep0_dispatch_setup(vhub, ep, setup_packet);

out:
    trace_ls_vhub_ep0_setup_result(vhub, evt->depevt.phy_ep_num, ret);

    if (ret < 0) {
        /* stall ep0 */
        ls_vhub_ep0_stall_and_restart(ep);
    }

    return ret;
}

static void ls_vhub_ep0_data_complete(struct ls_vhub *vhub, const union evt_buf *evt)
{
    struct ls_vhub_ep *ep0_xfer = &vhub->ep_pool.eps[evt->depevt.phy_ep_num];
    struct ls_vhub_ep *ep0_out = ep0_xfer_out(ep0_xfer);
    struct ls_vhub_ep *ep0_req = ep0_req_ep(ep0_xfer);
    struct dwc3_trb *trb = ep0_xfer->trb->trb;
    struct ls_vhub_req *req;
    u32 status, remain, xfer_len;

    req = next_request(&ep0_req->req_list);
    if (!req)
        return;

    status = DWC3_TRB_SIZE_TRBSTS(trb->size);
    if (status == DWC3_TRBSTS_SETUP_PENDING) {
        ls_vhub_ep_gadget_giveback(ep0_req, req, -ECONNRESET);
        return;
    }

    remain = trb->size & DWC3_TRB_SIZE_MASK;
    xfer_len = req->request.length - remain;
    req->request.actual += xfer_len;

    trace_ls_vhub_ep0_data_complete(ep0_req, req, xfer_len, status);

    vhub_dbg_ep(ep0_req, "ep0 data complete: actual = %d\n", req->request.actual);

    if (req->dir && xfer_len < req->request.length) {
        if (!list_empty(&ep0_req->req_list))
            ls_vhub_ep_gadget_giveback(ep0_req, next_request(&ep0_req->req_list), -ECONNRESET);
        ls_vhub_ep0_stall_and_restart(ep0_out);
        return;
    }

    ls_vhub_ep_gadget_giveback(ep0_req, req, 0);
}

static void ls_vhub_ep0_status(struct ls_vhub *vhub, const union evt_buf *evt)
{
    struct ls_vhub_ep *ep = &vhub->ep_pool.eps[evt->depevt.phy_ep_num];
    struct ls_vhub_ep *ep0_out = ep0_xfer_out(ep);
    struct ls_vhub_ep *ep0_req = ep0_req_ep(ep);
    struct ls_vhub_ctrl_ep *ctrl_ep = ep0_ctrl(ep0_out);
    struct ls_vhub_req *req;

    /* update ep0 state */
    ctrl_ep->state = EP0_SETUP_PHASE;
    /* a detaching udc_pullup(0) may be waiting for this transfer to end */

    req = next_request(&ep0_req->req_list);
    if (req != NULL) {
        /* two stage transfer complete */
        ls_vhub_ep_gadget_giveback(ep0_req, req, 0);
    }
    /* prepare receive next setup packet */
    ls_vhub_ep0_out_start(ep0_out);
    /* device have a new address, so we clear addr0_dev */
    if (ep0_out->common && vhub->addr0_dev && vhub->addr0_dev->address)
        vhub->addr0_dev = NULL;
}

static void ls_vhub_ep0_xfer_notready(struct ls_vhub *vhub, const union evt_buf *evt)
{
    u8 ep_num = evt->depevt.phy_ep_num;
    struct ls_vhub_ep *ep = &vhub->ep_pool.eps[ep_num];
    struct ls_vhub_ctrl_ep *ctrl_ep = ep0_ctrl(ep);
    struct ls_vhub_dev *dev = ep0_owner_dev(ep);

    switch (evt->depevt.evt_status) {
    case DEPEVT_STATUS_CONTROL_DATA:
        vhub_dbg_ep(ep, "ep0 xfer notready ep %d: CONTROL_DATA\n", ep_num);
        break;
    case DEPEVT_STATUS_CONTROL_STATUS:
        /* update next control transfer state */
        ctrl_ep->state = EP0_STATUS_PHASE;
        vhub_dbg_ep(ep, "ep0 xfer notready ep %d: CONTROL_STATUS\n", ep_num);
        /* upper layer delayed status */
        if (dev && dev->delayed_status) {
            if (!list_empty(&ep0_req_ep(ep)->req_list)) {
                dev->delayed_status = 0;
                usb_gadget_set_state(dev->gadget, USB_STATE_CONFIGURED);
                ls_vhub_ep0_xfer_status(vhub, ep);
            }
            return;
        }
        ls_vhub_ep0_xfer_status(vhub, ep);
        break;
    default:
        break;
    }
}

static void ls_vhub_ep0_xfer_complete(struct ls_vhub *vhub, const union evt_buf *evt)
{
    struct ls_vhub_ep *ep = &vhub->ep_pool.eps[evt->depevt.phy_ep_num];
    struct ls_vhub_ctrl_ep *ctrl_ep = ep0_ctrl(ep);

    if (!ep->active)
        return;

    ep->active = 0;
    ep->res_idx = 0;

    switch (ctrl_ep->state) {
    case EP0_SETUP_PHASE:
        ls_vhub_ep0_setup(vhub, evt);
        break;
    case EP0_DATA_PHASE:
        ls_vhub_ep0_data_complete(vhub, evt);
        break;
    case EP0_STATUS_PHASE:
        ls_vhub_ep0_status(vhub, evt);
        break;
    default:
        break;
    }
}

int ls_vhub_ep0_queue_req(struct ls_vhub_ep *ep0, struct ls_vhub_req *req)
{
    int ret = 0;
    struct ls_vhub *vhub = ep0->vhub;
    struct ls_vhub_ctrl_ep *ctrl_ep;

    struct ls_vhub_dev *dev = ep0->dev;

    if (dev && dev->enabled == false) {
        dev_err(vhub_ep_dev(ep0), "device not enabled, queue fail!!\n");
        return -ENOTCONN;
    }

    if (dev && dev->address == 0 && dev != vhub->addr0_dev)
        return -ENODEV;

    req->request.actual	= 0;
    req->request.status	= -EINPROGRESS;

    list_add_tail(&req->node, &ep0->req_list);

    if (dev && dev->address == 0)
        ep0 = vhub->common_ep0.out;

    ctrl_ep = ep0_ctrl(ep0);
    /* current dir */
    req->dir = ctrl_ep->expect_in;

    if (dev && dev->delayed_status) {
        dev->delayed_status = 0;
        usb_gadget_set_state(dev->gadget, USB_STATE_CONFIGURED);
        if (ctrl_ep->state == EP0_STATUS_PHASE) {
            ls_vhub_ep0_xfer_status(vhub, ep0 + !ctrl_ep->expect_in);
        }
        return 0;
    }

    if (ctrl_ep->three_stage_setup) {
        ctrl_ep->state = EP0_DATA_PHASE;
        ls_vhub_ep0_xfer_data(vhub, ep0 + ctrl_ep->expect_in, req);
    }

    return ret;
}

static int ls_vhub_ep0_queue(struct usb_ep *ep, struct usb_request *request, gfp_t gfp_flags)
{
    struct ls_vhub_ep *ep0 = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub_req *req = container_of(request, struct ls_vhub_req, request);
    struct ls_vhub *vhub = ep0->vhub;
    unsigned long flags;
    int ret;

    ls_vhub_lock(vhub, &flags);
    ret = ls_vhub_ep0_queue_req(ep0, req);
    ls_vhub_unlock(vhub, &flags);

    return ret;
}

static int ls_vhub_ep_dequeue(struct usb_ep *ep, struct usb_request *request)
{
    struct ls_vhub_ep *ep0 = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub_req *req = container_of(request, struct ls_vhub_req, request);
    struct ls_vhub *vhub = ep0->vhub;
    struct ls_vhub_req *iter, *tmp;
    unsigned long flags;
    int ret = -EINVAL;

    ls_vhub_lock(vhub, &flags);
    list_for_each_entry_safe(iter, tmp, &ep0->req_list, node) {
        if (iter != req)
            continue;

        if (next_request(&ep0->req_list) == req)
            ls_vhub_ep_stop_transfer(ep0);

        ls_vhub_ep_gadget_giveback(ep0, req, -ECONNRESET);
        ret = 0;
        break;
    }
    ls_vhub_unlock(vhub, &flags);

    return ret;
}

static int ls_vhub_ep0_set_halt(struct usb_ep *ep, int value)
{
    struct ls_vhub_ep *ep0 = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub *vhub = ep0->vhub;
    unsigned long flags;

    ls_vhub_lock(vhub, &flags);
    if (value) {
        ls_vhub_ep0_stall_and_restart(ep0);
    } else {
        ep0->stall = 0;
    }
    ls_vhub_unlock(vhub, &flags);

    return 0;
}

static int ls_vhub_ep_set_wedge(struct usb_ep *ep)
{
    struct ls_vhub_ep *ep0 = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub *vhub = ep0->vhub;
    unsigned long flags;
    dev_dbg(vhub_ep_dev(ep0), "ls_vhub_ep_set_wedge %d\n", ep0->number);

    ls_vhub_lock(vhub, &flags);
    ls_vhub_ep0_stall_and_restart(ep0);
    ls_vhub_unlock(vhub, &flags);

    return 0;
}

static const struct usb_ep_ops _ep0_ops = {
    .alloc_request = ls_vhub_ep_alloc_request,
    .free_request = ls_vhub_ep_free_request,
    .queue = ls_vhub_ep0_queue,
    .dequeue = ls_vhub_ep_dequeue,
    .set_halt = ls_vhub_ep0_set_halt,
    .set_wedge = ls_vhub_ep_set_wedge,
};

int ls_vhub_ep0_handle(struct ls_vhub *vhub, const union evt_buf *evt)
{
    enum ep_evt evt_type = evt->depevt.ep_evt_type;
    struct ls_vhub_ep *ep = &vhub->ep_pool.eps[evt->depevt.phy_ep_num];

    if (!ep->enabled) {
        vhub_dbg_ep(ep, "drop ep0 event %d: endpoint disabled\n", evt_type);
        return 0;
    }

    if (ep->dev && !ep->dev->enabled) {
        vhub_dbg_ep(ep, "drop ep0 event %d: device disabled\n", evt_type);
        return 0;
    }

    if (ep->common && vhub->addr0_dev && vhub->addr0_dev->enabled == false) {
        vhub_dbg_ep(ep, "drop ep0 event %d: addr0 device disabled\n", evt_type);
        ls_vhub_ep0_stall_and_restart(ep);
        vhub->addr0_dev = NULL;
        return 0;
    }

    switch (evt_type) {
    case DEPEVT_XferComplete:
        ls_vhub_ep0_xfer_complete(vhub, evt);
        break;
    case DEPEVT_XferNotReady:
        ls_vhub_ep0_xfer_notready(vhub, evt);
    break;
    case DEPEVT_EPCmdCmplt:
        break;
    default:
        break;
    }

    return 0;
}

int ls_vhub_ep0_init(struct ls_vhub *vhub, struct ls_vhub_dev *dev, struct ls_vhub_ep *ep)
{
    ep->endpoint.desc = &ep0_desc;
    INIT_LIST_HEAD(&ep->endpoint.ep_list);
    INIT_LIST_HEAD(&ep->req_list);

    ep->endpoint.ops = &_ep0_ops;
    ep->endpoint.name = "ep0";
    ep->endpoint.caps.type_control = true;
    usb_ep_set_maxpacket_limit(&ep->endpoint, 64);
    ep->ctrl = true;
    ep->dev = dev;
    ep->vhub = vhub;
    snprintf(ep->name, sizeof(ep->name), "ep%u%s",
             ep->number >> 1, (ep->number & 1) ? "in" : "out");

    if (dev) {
        /* insert dev ep list */
        list_add_tail(&ep->dev_node, &dev->ep_list);
    }

    return 0;
}
