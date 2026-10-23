#ifndef __LS_PCIE_VHUB_EP_H__
#define __LS_PCIE_VHUB_EP_H__

#include <linux/usb/gadget.h>
#include "vhub.h"

struct usb_request *ls_pcie_vhub_alloc_request(struct usb_ep *ep, gfp_t gfp_flags);
void ls_pcie_vhub_free_request(struct usb_ep *ep, struct usb_request *req);

struct ls_pcie_vhub_ep *ls_pcie_vhub_epn_alloc(struct ls_pcie_vhub_dev *dev);
void ls_pcie_vhub_epn_free(struct ls_pcie_vhub_ep *ep);
int ls_pcie_vhub_ep_req_complete(struct ls_pcie_vusb_ep_req *req);

#endif /* __LS_PCIE_VHUB_EP_H__ */

