#include <linux/dma-mapping.h>
#include "vhub.h"
#include "trace.h"

static int ls_vhub_epn_xfer_complete(struct ls_vhub_ep *epn)
{
    int ret = 0;
    struct ls_vhub_req *req;
    u32 remain;
    struct dwc3_trb *trb;

    if (!epn->active)
        return 0;

    req = next_request(&epn->req_list);
    epn->active = 0;
    epn->res_idx = 0;

    if (!req || !epn->enabled)
        return 0;

    trb = epn->trb->trb;
    remain = trb->size & DWC3_TRB_SIZE_MASK;
    req->request.actual = req->request.length - remain;
    req->request.status = 0;

    trace_ls_vhub_epn_complete(epn, req, 0);

    vhub_dbg_ep(epn, "xfer complete: epn=%d actual=%d length=%d dir=%s\n",
                epn->number, req->request.actual, req->request.length, req->dir ? "IN" : "OUT");

    if (req->chain) {
        trb++;
        trb->ctrl &= ~DWC3_TRB_CTRL_HWO;
        req->chain = false;
    }

    bool more_queued = !list_is_singular(&epn->req_list);
    ls_vhub_ep_gadget_giveback(epn, req, 0);

    if (more_queued) {
        req = next_request(&epn->req_list);
        if (req && epn->enabled)
            ret = ls_vhub_epn_kick_transfer(epn);
    }

    return ret;
}

int ls_vhub_epn_handle(struct ls_vhub *vhub, const union evt_buf *evt)
{
    enum ep_evt evt_type = evt->depevt.ep_evt_type;
    u8 ep_num = evt->depevt.phy_ep_num;
    struct ls_vhub_ep *ep = &vhub->ep_pool.eps[ep_num];

    if (!ep->enabled) {
        vhub_dbg_ep(ep, "drop epn event %d: endpoint disabled\n", evt_type);
        return 0;
    }

    switch (evt_type) {
    case DEPEVT_XferComplete:
        return ls_vhub_epn_xfer_complete(ep);
    case DEPEVT_XferNotReady:
        break;
    case DEPEVT_EPCmdCmplt:
        break;
    default:
        break;
    }

    return 0;
}

static int ls_vhub_epn_enable(struct usb_ep *ep, const struct usb_endpoint_descriptor *desc)
{
    struct ls_vhub_ep *epn = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub *vhub = epn->vhub;
    unsigned long flags;
    int ret;

    trace_ls_vhub_epn_enable(epn);

    ls_vhub_lock(vhub, &flags);
    /* enable ep */
    epn->endpoint.address = desc->bEndpointAddress;
    epn->endpoint.desc = desc;

    ret = ls_vhub_ep_enable(epn);
    if (ret == 0) {
        ls_vhub_ep_map(epn);
    }
    ls_vhub_unlock(vhub, &flags);

    return ret;
}

static int ls_vhub_epn_disable(struct usb_ep *ep)
{
    struct ls_vhub_ep *epn = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub *vhub = epn->vhub;
    unsigned long flags;

    trace_ls_vhub_epn_disable(epn);

    ls_vhub_lock(vhub, &flags);

    ls_vhub_ep_stop_transfer(epn);
    if (epn->stall)
        ls_vhub_ep_clear_stall(epn);
    epn->stall = 0;
    ls_vhub_ep_disable(epn);
    epn->wedge = 0;
    epn->delay_start = 0;
    ls_vhub_ep_unmap(epn);

    while (!list_empty(&epn->req_list)) {
        struct ls_vhub_req *req = next_request(&epn->req_list);
        ls_vhub_ep_gadget_giveback(epn, req, -ESHUTDOWN);
    }

    ls_vhub_unlock(vhub, &flags);

    return 0;
}

static bool ls_vhub_needs_extra_trb(struct ls_vhub_ep *epn, struct ls_vhub_req *req)
{
    unsigned int maxp = usb_endpoint_maxp(epn->endpoint.desc);
    unsigned int rem = req->request.length % maxp;

    if ((req->request.length && req->request.zero && !rem &&
         !usb_endpoint_xfer_isoc(epn->endpoint.desc)) || 
         (!req->dir && rem)) {
            req->chain = true;
            return true;
        }

    return false;
}

static void ls_vhub_prepare_one_trb(struct ls_vhub_ep *epn, struct dwc3_trb *trb, 
                                        dma_addr_t dma_buf, u32 size, bool chain)
{
    trb->bptrl = lower_32_bits(dma_buf);
    trb->bptrh = upper_32_bits(dma_buf);
    trb->size = DWC3_TRB_SIZE_LENGTH(size);

    switch (usb_endpoint_type(epn->endpoint.desc)) {
    case USB_ENDPOINT_XFER_CONTROL:
        trb->ctrl = DWC3_TRBCTL_CONTROL_SETUP;
        break;
    case USB_ENDPOINT_XFER_ISOC:
        break;
    case USB_ENDPOINT_XFER_BULK:
    case USB_ENDPOINT_XFER_INT:
        trb->ctrl = DWC3_TRBCTL_NORMAL;
        break;
    default:
        break;
    }

    if (chain) {
        trb->ctrl |= DWC3_TRB_CTRL_CHN;
    } else {
        trb->ctrl |= DWC3_TRB_CTRL_IOC | DWC3_TRB_CTRL_LST;
    }

    trb->ctrl |= DWC3_TRB_CTRL_HWO;
}

int ls_vhub_epn_kick_transfer(struct ls_vhub_ep *epn)
{
    struct usb_ep *ep = &epn->endpoint;
    struct ls_vhub *vhub = epn->vhub;
    struct ls_vhub_req *req = next_request(&epn->req_list);
    int ret;

    trace_ls_vhub_epn_kick(epn, req);

    u32 maxp = usb_endpoint_maxp(ep->desc);
    u32 rem = req->request.length % maxp;
    bool extra_trb = ls_vhub_needs_extra_trb(epn, req);

    if (req->internel == false) {
        ret = usb_gadget_map_request_by_dev(&vhub->pdev->dev, &req->request, req->dir);
        if (ret)
            return ret;
        req->dma_mapped = true;
    }

    vhub_dbg_ep(epn, "kick transfer: length=%d dir=%d extra_trb=%d sgs=%d\n",
                req->request.length, req->dir, extra_trb, req->request.num_mapped_sgs);

    /* Prepare a normal TRB */
    if (req->dir || req->request.length) {
        ls_vhub_prepare_one_trb(epn, &epn->trb->trb[0], req->request.dma, req->request.length, extra_trb);
    }

    /* Prepare extra TRBs for ZLP and MPS OUT transfer alignment */
    if ((!req->dir && !req->request.length) || extra_trb) {
        ls_vhub_prepare_one_trb(epn, &epn->trb->trb[1], epn->vhub->bounce_dma, req->dir ? 0 : maxp - rem, false);
    }

    /* start transfer */
    ret = ls_vhub_ep_start_transfer(epn, epn->trb_dma, 0);
    if (ret) {
        /* stop transfer */
        vhub_dbg_ep(epn, "kick transfer: start failed\n");
        ls_vhub_ep_stop_transfer(epn);
        ls_vhub_ep_gadget_giveback(epn, req, -ECONNRESET);
    }

    return ret;
}

int ls_vhub_epn_queue_req(struct ls_vhub_ep *epn, struct ls_vhub_req *req)
{
    int ret = 0;

    if (!epn->enabled) {
        ret = -ESHUTDOWN;
        goto out;
    }

    vhub_dbg_ep(epn, "queue req: length=%d dir=%d\n", req->request.length, req->dir);

    req->request.actual = 0;
    req->request.status = -EINPROGRESS;
    list_add_tail(&req->node, &epn->req_list);
    if (next_request(&epn->req_list) != req) {
        vhub_dbg_ep(epn, "queue req: req list not empty\n");
        goto out;
    }

    if (epn->stall || epn->wedge) {
        vhub_dbg_ep(epn, "queue req: has stall\n");
        epn->delay_start = true;
        return 0;
    }

    ret = ls_vhub_epn_kick_transfer(epn);

out:
    return ret;
}

static int ls_vhub_epn_queue(struct usb_ep *ep, struct usb_request *request, gfp_t gfp_flags)
{
    struct ls_vhub_ep *epn = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub_req *req = container_of(request, struct ls_vhub_req, request);
    struct ls_vhub *vhub = epn->vhub;
    unsigned long flags;
    int ret;

    trace_ls_vhub_epn_queue(epn, req);

    ls_vhub_lock(vhub, &flags);
    ret = ls_vhub_epn_queue_req(epn, req);
    ls_vhub_unlock(vhub, &flags);

    return ret;
}

static int ls_vhub_epn_dequeue(struct usb_ep *ep, struct usb_request *request)
{
    struct ls_vhub_ep *epn = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub_req *req = container_of(request, struct ls_vhub_req, request);
    struct ls_vhub *vhub = epn->vhub;
    struct ls_vhub_req *iter, *tmp;
    unsigned long flags;
    bool active = false;
    int ret = -EINVAL;

    trace_ls_vhub_epn_dequeue(epn, req);

    ls_vhub_lock(vhub, &flags);
    list_for_each_entry_safe(iter, tmp, &epn->req_list, node) {
        if (iter != req)
            continue;

        /* 当前 req 可能已经被提交给了控制器 */
        active = next_request(&epn->req_list) == req;
        if (active)
            ls_vhub_ep_stop_transfer(epn);

        ls_vhub_ep_gadget_giveback(epn, req, -ECONNRESET);
        ret = 0;
        break;
    }

    if (!ret && active && epn->enabled) {
        req = next_request(&epn->req_list);
        if (req) {
            ret = ls_vhub_epn_kick_transfer(epn);
            if (ret)
                ls_vhub_ep_gadget_giveback(epn, req, ret);
        }
    }
    ls_vhub_unlock(vhub, &flags);

    return ret;
}

static int ls_vhub_epn_set_halt(struct usb_ep *ep, int value)
{
    struct ls_vhub_ep *epn = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub *vhub = epn->vhub;
    int ret;
    unsigned long flags;

    trace_ls_vhub_epn_set_halt(epn, value);

    vhub_dbg_ep(epn, "set halt: value=%d\n", value);

    ls_vhub_lock(vhub, &flags);
    ret = ls_vhub_ep_set_halt(epn, value);
    ls_vhub_unlock(vhub, &flags);
    return ret;
}

static int ls_vhub_epn_set_wedge(struct usb_ep *ep)
{
    struct ls_vhub_ep *epn = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub *vhub = epn->vhub;
    int ret;
    unsigned long flags;

    trace_ls_vhub_epn_set_wedge(epn);

    vhub_dbg_ep(epn, "set wedge\n");

    ls_vhub_lock(vhub, &flags);
    epn->wedge = 1;
    ret = ls_vhub_ep_set_halt(epn, 1);
    ls_vhub_unlock(vhub, &flags);

    return ret;
}

static const struct usb_ep_ops _epn_ops = {
    .enable = ls_vhub_epn_enable,
    .disable = ls_vhub_epn_disable,
    .alloc_request = ls_vhub_ep_alloc_request,
    .free_request = ls_vhub_ep_free_request,
    .queue = ls_vhub_epn_queue,
    .dequeue = ls_vhub_epn_dequeue,
    .set_halt = ls_vhub_epn_set_halt,
    .set_wedge = ls_vhub_epn_set_wedge
};

int ls_vhub_epn_init(struct ls_vhub *vhub, struct ls_vhub_dev *dev, struct ls_vhub_ep *ep)
{
    u8 dir_in = ep->number & 0x1;

    INIT_LIST_HEAD(&ep->req_list);
    INIT_LIST_HEAD(&ep->endpoint.ep_list);
    INIT_LIST_HEAD(&ep->dev_node);
    ep->endpoint.ops = &_epn_ops;
    snprintf(ep->name, sizeof(ep->name), "ep%u%s", ep->number >> 1,dir_in ? "in" : "out");

    if (dev) {
        ep->endpoint.name = ep->name;
    }

    ep->endpoint.caps.type_control = false;
    ep->endpoint.caps.type_bulk = true;
    ep->endpoint.caps.type_int = true;
    ep->endpoint.caps.type_iso = false;
    ep->endpoint.caps.dir_in = dir_in;
    ep->endpoint.caps.dir_out = !dir_in;

    ep->common = false;
    ep->ctrl = false;
    ep->resource = false;
    ep->delay_start = false;
    ep->active = false;
    ep->dev = dev;
    ep->vhub = vhub;

    return 0;
}
