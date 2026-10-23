#ifndef __VHUB_H__
#define __VHUB_H__

#include "vhub_def.h"

int ls_vhub_attach(struct ls_vhub *vhub, int port_idx);
int ls_vhub_detach(struct ls_vhub *vhub, int port_idx);
int ls_vhub_setup(struct ls_vhub *vhub, struct usb_ctrlrequest *ctrl);

int ls_vhub_dev_setup(struct ls_vhub_dev *dev, struct usb_ctrlrequest *ctrl);
int ls_vhub_dev_init(struct ls_vhub *vhub, u8 port_idx);
int ls_vhub_dev_deinit(struct ls_vhub *vhub, u8 port_idx);
int ls_vhub_dev_reset(struct ls_vhub *vhub, u8 port_idx);
void ls_vhub_dev_bus_reset(struct ls_vhub_dev *dev);
void ls_vhub_dev_disconnect(struct ls_vhub_dev *dev);
void ls_vhub_dev_suspend(struct ls_vhub_dev *dev);
void ls_vhub_dev_resume(struct ls_vhub_dev *dev);

int ls_vhub_ep_pool_init(struct ls_vhub *vhub, u8 ep_num);
struct ls_vhub_ep *ls_vhub_ep_alloc(struct ls_vhub_ep_pool *ep_pool, u8 dir);
void ls_vhub_ep_free(struct ls_vhub_ep_pool *ep_pool, struct ls_vhub_ep *ep);

int ls_vhub_ep0_init(struct ls_vhub *vhub, struct ls_vhub_dev *dev, struct ls_vhub_ep *ep);
int ls_vhub_epn_init(struct ls_vhub *vhub, struct ls_vhub_dev *dev, struct ls_vhub_ep *ep);
int ls_vhub_ep0_queue_req(struct ls_vhub_ep *ep0, struct ls_vhub_req *req);
int ls_vhub_epn_queue_req(struct ls_vhub_ep *epn, struct ls_vhub_req *req);
int ls_vhub_ep0_handle(struct ls_vhub *vhub, const union evt_buf *evt);
int ls_vhub_epn_handle(struct ls_vhub *vhub, const union evt_buf *evt);

int ls_vhub_ep0_stall_and_restart(struct ls_vhub_ep *ep);
int ls_vhub_epn_kick_transfer(struct ls_vhub_ep *epn);
int ls_vhub_ep_gadget_giveback(struct ls_vhub_ep *ep, struct ls_vhub_req *req, int status);

int ls_vhub_ep_enable(struct ls_vhub_ep *ep);
int ls_vhub_ep_disable(struct ls_vhub_ep *ep);
int ls_vhub_ep_clear_stall(struct ls_vhub_ep *ep);
int ls_vhub_ep_stop_transfer(struct ls_vhub_ep *ep);
int ls_vhub_ep_start_transfer(struct ls_vhub_ep *ep, dma_addr_t trb, u16 stream_id);
int ls_vhub_ep_set_halt(struct ls_vhub_ep *ep, u8 set);

void ls_vhub_ep_map(struct ls_vhub_ep *ep);
void ls_vhub_ep_unmap(struct ls_vhub_ep *ep);

struct usb_request *ls_vhub_ep_alloc_request(struct usb_ep *ep, gfp_t gfp_flags);
void ls_vhub_ep_free_request(struct usb_ep *ep, struct usb_request *request);
void ls_vhub_ep_prepare_trb(struct dwc3_trb *trb, dma_addr_t buf_ptr, u32 size, u32 type, bool chain);

#endif //__VHUB_H__
