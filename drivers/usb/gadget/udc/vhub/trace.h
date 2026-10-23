#undef TRACE_SYSTEM
#define TRACE_SYSTEM ls_vhub

#if !defined(_TRACE_VHUB_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_VHUB_H

#include <linux/tracepoint.h>
#include "vhub.h"

#define VHUB_MSG_MAX 500

TRACE_EVENT(ls_vhub_notify_update,

	TP_PROTO(struct ls_vhub *vhub, u8 bitmap),

	TP_ARGS(vhub, bitmap),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
		__field(u8, bitmap)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->bitmap = bitmap;
	),

	TP_printk("%s bitmap=0x%02x",
		  __get_str(dev), __entry->bitmap)
);

TRACE_EVENT(ls_vhub_dep_cmd,

	TP_PROTO(struct ls_vhub *vhub, u8 ep, u32 cmd,
		 const struct dep_command_param *param,
		 u32 result, int ret),

	TP_ARGS(vhub, ep, cmd, param, result, ret),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
		__field(u8, ep)
		__field(u32, cmd)
		__field(u32, param0)
		__field(u32, param1)
		__field(u32, param2)
		__field(u32, result)
		__field(int, ret)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep = ep;
		__entry->cmd = cmd;
		__entry->param0 = param->param0;
		__entry->param1 = param->param1;
		__entry->param2 = param->param2;
		__entry->result = result;
		__entry->ret = ret;
	),

	TP_printk("%s ep%u cmd=%s param0=0x%08x param1=0x%08x param2=0x%08x result=0x%08x ret=%d",
		  __get_str(dev), __entry->ep,
		  __print_symbolic(__entry->cmd & 0xf,
			{ DWC3_DEPCMD_DEPSTARTCFG,         "DEPSTARTCFG" },
			{ DWC3_DEPCMD_ENDTRANSFER,         "ENDTRANSFER" },
			{ DWC3_DEPCMD_UPDATETRANSFER,      "UPDATETRANSFER" },
			{ DWC3_DEPCMD_STARTTRANSFER,       "STARTTRANSFER" },
			{ DWC3_DEPCMD_CLEARSTALL,          "CLEARSTALL" },
			{ DWC3_DEPCMD_SETSTALL,            "SETSTALL" },
			{ DWC3_DEPCMD_GETSEQNUMBER,        "GETSEQNUMBER/GETEPSTATE" },
			{ DWC3_DEPCMD_SETTRANSFRESOURCE,   "SETTRANSFRESOURCE" },
			{ DWC3_DEPCMD_SETEPCONFIG,         "SETEPCONFIG" }),
		  __entry->param0, __entry->param1, __entry->param2,
		  __entry->result, __entry->ret)
);

TRACE_EVENT(ls_vhub_ep_config,

	TP_PROTO(struct ls_vhub *vhub, u8 ep_number,
		 const struct usb_endpoint_descriptor *desc,
		 u8 config_action),

	TP_ARGS(vhub, ep_number, desc, config_action),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
		__field(u8, ep_number)
		__field(u8, ep_dir)
		__field(u8, ep_type)
		__field(u16, maxpacket)
		__field(u8, bInterval)
		__field(u8, config_action)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep_number = ep_number;
		__entry->ep_dir = ep_number & 1;
		__entry->ep_type = usb_endpoint_type(desc);
		__entry->maxpacket = le16_to_cpu(desc->wMaxPacketSize);
		__entry->bInterval = desc->bInterval;
		__entry->config_action = config_action;
	),

	TP_printk("%s ep%u %s dir=%s mps=%u bInterval=%u action=%u",
		  __get_str(dev), __entry->ep_number,
		  __print_symbolic(__entry->ep_type,
			{ USB_ENDPOINT_XFER_CONTROL, "CONTROL" },
			{ USB_ENDPOINT_XFER_ISOC,     "ISOC" },
			{ USB_ENDPOINT_XFER_BULK,     "BULK" },
			{ USB_ENDPOINT_XFER_INT,      "INT" }),
		  __entry->ep_dir ? "IN" : "OUT",
		  __entry->maxpacket, __entry->bInterval, __entry->config_action)
);

TRACE_EVENT(ls_vhub_udc_pullup,

	TP_PROTO(struct ls_vhub_dev *dev, int is_on),

	TP_ARGS(dev, is_on),

	TP_STRUCT__entry(
		__string(name, dev_name(dev->udev))
		__field(u8, port)
		__field(u8, is_on)
	),

	TP_fast_assign(
		__assign_str(name);
		__entry->port = dev->port_index;
		__entry->is_on = !!is_on;
	),

	TP_printk("%s port=%u pullup=%s",
		  __get_str(name), __entry->port,
		  __entry->is_on ? "on" : "off")
);

TRACE_EVENT(ls_vhub_udc_start,

	TP_PROTO(struct ls_vhub_dev *dev, struct usb_gadget_driver *driver),

	TP_ARGS(dev, driver),

	TP_STRUCT__entry(
		__string(name, dev_name(dev->udev))
		__string(driver, driver->driver.name)
		__field(u8, port)
	),

	TP_fast_assign(
		__assign_str(name);
		__assign_str(driver);
		__entry->port = dev->port_index;
	),

	TP_printk("%s port=%u driver=%s",
		  __get_str(name), __entry->port, __get_str(driver))
);

TRACE_EVENT(ls_vhub_udc_stop,

	TP_PROTO(struct ls_vhub_dev *dev),

	TP_ARGS(dev),

	TP_STRUCT__entry(
		__string(name, dev_name(dev->udev))
		__field(u8, port)
	),

	TP_fast_assign(
		__assign_str(name);
		__entry->port = dev->port_index;
	),

	TP_printk("%s port=%u stop", __get_str(name), __entry->port)
);

TRACE_EVENT(ls_vhub_udc_match_ep,

	TP_PROTO(struct ls_vhub_dev *dev,
		 const struct usb_endpoint_descriptor *desc),

	TP_ARGS(dev, desc),

	TP_STRUCT__entry(
		__string(name, dev_name(dev->udev))
		__field(u8, port)
		__field(u8, ep_type)
		__field(u8, dir)
		__field(u16, maxpacket)
	),

	TP_fast_assign(
		__assign_str(name);
		__entry->port = dev->port_index;
		__entry->ep_type = usb_endpoint_type(desc);
		__entry->dir = usb_endpoint_dir_in(desc);
		__entry->maxpacket = le16_to_cpu(desc->wMaxPacketSize);
	),

	TP_printk("%s port=%u match_ep type=%s dir=%s mps=%u",
		  __get_str(name), __entry->port,
		  __print_symbolic(__entry->ep_type,
			{ USB_ENDPOINT_XFER_CONTROL, "CONTROL" },
			{ USB_ENDPOINT_XFER_ISOC,     "ISOC" },
			{ USB_ENDPOINT_XFER_BULK,     "BULK" },
			{ USB_ENDPOINT_XFER_INT,      "INT" }),
		  __entry->dir ? "IN" : "OUT",
		  __entry->maxpacket)
);

TRACE_EVENT(ls_vhub_udc_ep_alloc,

	TP_PROTO(struct ls_vhub_dev *dev, u8 ep_num,
		 const struct usb_endpoint_descriptor *desc),

	TP_ARGS(dev, ep_num, desc),

	TP_STRUCT__entry(
		__string(name, dev_name(dev->udev))
		__field(u8, port)
		__field(u8, ep_num)
		__field(u8, ep_type)
		__field(u8, dir)
		__field(u16, maxpacket)
	),

	TP_fast_assign(
		__assign_str(name);
		__entry->port = dev->port_index;
		__entry->ep_num = ep_num;
		__entry->ep_type = usb_endpoint_type(desc);
		__entry->dir = usb_endpoint_dir_in(desc);
		__entry->maxpacket = le16_to_cpu(desc->wMaxPacketSize);
	),

	TP_printk("%s port=%u alloc_ep ep=%u type=%s dir=%s mps=%u",
		  __get_str(name), __entry->port, __entry->ep_num,
		  __print_symbolic(__entry->ep_type,
			{ USB_ENDPOINT_XFER_CONTROL, "CONTROL" },
			{ USB_ENDPOINT_XFER_ISOC,     "ISOC" },
			{ USB_ENDPOINT_XFER_BULK,     "BULK" },
			{ USB_ENDPOINT_XFER_INT,      "INT" }),
		  __entry->dir ? "IN" : "OUT",
		  __entry->maxpacket)
);

TRACE_EVENT(ls_vhub_dev_init,

	TP_PROTO(struct ls_vhub *vhub, u8 port_idx),

	TP_ARGS(vhub, port_idx),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
		__field(u8, port)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->port = port_idx;
	),

	TP_printk("%s dev_init port=%u", __get_str(dev), __entry->port)
);

DECLARE_EVENT_CLASS(ls_vhub_dev_event,

	TP_PROTO(struct ls_vhub_dev *dev),

	TP_ARGS(dev),

	TP_STRUCT__entry(
		__string(name, dev_name(dev->udev))
		__field(u8, port)
	),

	TP_fast_assign(
		__assign_str(name);
		__entry->port = dev->port_index;
	),

	TP_printk("%s port=%u", __get_str(name), __entry->port)
);

DEFINE_EVENT(ls_vhub_dev_event, ls_vhub_dev_bus_reset,
	TP_PROTO(struct ls_vhub_dev *dev),
	TP_ARGS(dev)
);

DEFINE_EVENT(ls_vhub_dev_event, ls_vhub_dev_disconnect,
	TP_PROTO(struct ls_vhub_dev *dev),
	TP_ARGS(dev)
);

DEFINE_EVENT(ls_vhub_dev_event, ls_vhub_dev_suspend,
	TP_PROTO(struct ls_vhub_dev *dev),
	TP_ARGS(dev)
);

DEFINE_EVENT(ls_vhub_dev_event, ls_vhub_dev_resume,
	TP_PROTO(struct ls_vhub_dev *dev),
	TP_ARGS(dev)
);

DEFINE_EVENT(ls_vhub_dev_event, ls_vhub_dev_notify_reset,
	TP_PROTO(struct ls_vhub_dev *dev),
	TP_ARGS(dev)
);

DECLARE_EVENT_CLASS(ls_vhub_port_event,

	TP_PROTO(struct ls_vhub *vhub, u8 port_idx),

	TP_ARGS(vhub, port_idx),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
		__field(u8, port)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->port = port_idx;
	),

	TP_printk("%s port=%u", __get_str(dev), __entry->port)
);

DEFINE_EVENT(ls_vhub_port_event, ls_vhub_dev_reset,
	TP_PROTO(struct ls_vhub *vhub, u8 port_idx),
	TP_ARGS(vhub, port_idx)
);

DECLARE_EVENT_CLASS(ls_vhub_epn_event,

	TP_PROTO(struct ls_vhub_ep *ep),

	TP_ARGS(ep),

	TP_STRUCT__entry(
		__string(dev, dev_name(vhub_ep_dev(ep)))
		__field(u8, ep_num)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep_num = ep->number;
	),

	TP_printk("%s ep%u", __get_str(dev), __entry->ep_num)
);

DEFINE_EVENT(ls_vhub_epn_event, ls_vhub_epn_enable,
	TP_PROTO(struct ls_vhub_ep *ep),
	TP_ARGS(ep)
);

DEFINE_EVENT(ls_vhub_epn_event, ls_vhub_epn_disable,
	TP_PROTO(struct ls_vhub_ep *ep),
	TP_ARGS(ep)
);

DEFINE_EVENT(ls_vhub_epn_event, ls_vhub_epn_set_wedge,
	TP_PROTO(struct ls_vhub_ep *ep),
	TP_ARGS(ep)
);

TRACE_EVENT(ls_vhub_epn_set_halt,

	TP_PROTO(struct ls_vhub_ep *ep, int value),

	TP_ARGS(ep, value),

	TP_STRUCT__entry(
		__string(dev, dev_name(vhub_ep_dev(ep)))
		__field(u8, ep_num)
		__field(int, value)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep_num = ep->number;
		__entry->value = value;
	),

	TP_printk("%s ep%u set_halt value=%d",
		  __get_str(dev), __entry->ep_num, __entry->value)
);

DECLARE_EVENT_CLASS(ls_vhub_epn_req_event,

	TP_PROTO(struct ls_vhub_ep *ep, struct ls_vhub_req *req),

	TP_ARGS(ep, req),

	TP_STRUCT__entry(
		__string(dev, dev_name(vhub_ep_dev(ep)))
		__field(u8, ep_num)
		__field(void *, req)
		__field(u32, length)
		__field(u8, dir)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep_num = ep->number;
		__entry->req = req;
		__entry->length = req->request.length;
		__entry->dir = req->dir;
	),

	TP_printk("%s ep%u req=%p length=%u dir=%s",
		  __get_str(dev), __entry->ep_num, __entry->req,
		  __entry->length, __entry->dir ? "IN" : "OUT")
);

DEFINE_EVENT(ls_vhub_epn_req_event, ls_vhub_epn_queue,
	TP_PROTO(struct ls_vhub_ep *ep, struct ls_vhub_req *req),
	TP_ARGS(ep, req)
);

DEFINE_EVENT(ls_vhub_epn_req_event, ls_vhub_epn_dequeue,
	TP_PROTO(struct ls_vhub_ep *ep, struct ls_vhub_req *req),
	TP_ARGS(ep, req)
);

DEFINE_EVENT(ls_vhub_epn_req_event, ls_vhub_ep_alloc_request,
	TP_PROTO(struct ls_vhub_ep *ep, struct ls_vhub_req *req),
	TP_ARGS(ep, req)
);

DEFINE_EVENT(ls_vhub_epn_req_event, ls_vhub_ep_free_request,
	TP_PROTO(struct ls_vhub_ep *ep, struct ls_vhub_req *req),
	TP_ARGS(ep, req)
);

DEFINE_EVENT(ls_vhub_epn_req_event, ls_vhub_epn_kick,
	TP_PROTO(struct ls_vhub_ep *ep, struct ls_vhub_req *req),
	TP_ARGS(ep, req)
);

TRACE_EVENT(ls_vhub_ep_gadget_giveback,

	TP_PROTO(struct ls_vhub_ep *ep, struct ls_vhub_req *req,
		 int status),

	TP_ARGS(ep, req, status),

	TP_STRUCT__entry(
		__string(dev, dev_name(vhub_ep_dev(ep)))
		__field(u8, ep_num)
		__field(void *, req)
		__field(int, status)
		__field(u8, internal)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep_num = ep->number;
		__entry->req = req;
		__entry->status = status;
		__entry->internal = req->internel;
	),

	TP_printk("%s ep%u req=%p status=%d internal=%u",
		  __get_str(dev), __entry->ep_num, __entry->req,
		  __entry->status, __entry->internal)
);

TRACE_EVENT(ls_vhub_ep1_complete,

	TP_PROTO(struct ls_vhub *vhub, int status, u32 actual, bool pending),

	TP_ARGS(vhub, status, actual, pending),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
		__field(int, status)
		__field(u32, actual)
		__field(u8, pending)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->status = status;
		__entry->actual = actual;
		__entry->pending = pending;
	),

	TP_printk("%s ep1_complete status=%d actual=%u pending=%u",
		  __get_str(dev), __entry->status, __entry->actual,
		  __entry->pending)
);

TRACE_EVENT(ls_vhub_notify_busy,

	TP_PROTO(struct ls_vhub *vhub),

	TP_ARGS(vhub),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
	),

	TP_fast_assign(
		__assign_str(dev);
	),

	TP_printk("%s notify_busy", __get_str(dev))
);

TRACE_EVENT(ls_vhub_ep0_setup_result,

	TP_PROTO(struct ls_vhub *vhub, u8 ep_num, int ret),

	TP_ARGS(vhub, ep_num, ret),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
		__field(u8, ep_num)
		__field(int, ret)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep_num = ep_num;
		__entry->ret = ret;
	),

	TP_printk("%s ep%u setup_result ret=%d",
		  __get_str(dev), __entry->ep_num, __entry->ret)
);

TRACE_EVENT(ls_vhub_epn_complete,

	TP_PROTO(struct ls_vhub_ep *ep, struct ls_vhub_req *req,
		 int status),

	TP_ARGS(ep, req, status),

	TP_STRUCT__entry(
		__string(dev, dev_name(vhub_ep_dev(ep)))
		__field(u8, ep_num)
		__field(void *, req)
		__field(u32, actual)
		__field(u32, length)
		__field(int, status)
		__field(u8, dir)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep_num = ep->number;
		__entry->req = req;
		__entry->actual = req->request.actual;
		__entry->length = req->request.length;
		__entry->status = status;
		__entry->dir = req->dir;
	),

	TP_printk("%s ep%u req=%p actual=%u length=%u status=%d dir=%s",
		  __get_str(dev), __entry->ep_num, __entry->req,
		  __entry->actual, __entry->length, __entry->status,
		  __entry->dir ? "IN" : "OUT")
);

TRACE_EVENT(ls_vhub_ep0_data_complete,

	TP_PROTO(struct ls_vhub_ep *ep, struct ls_vhub_req *req,
		 u32 xfer_len, u32 status),

	TP_ARGS(ep, req, xfer_len, status),

	TP_STRUCT__entry(
		__string(dev, dev_name(vhub_ep_dev(ep)))
		__field(u8, ep_num)
		__field(void *, req)
		__field(u32, actual)
		__field(u32, length)
		__field(u32, xfer_len)
		__field(u32, status)
		__field(u8, dir)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep_num = ep->number;
		__entry->req = req;
		__entry->actual = req->request.actual;
		__entry->length = req->request.length;
		__entry->xfer_len = xfer_len;
		__entry->status = status;
		__entry->dir = req->dir;
	),

	TP_printk("%s ep%u req=%p actual=%u length=%u xfer=%u status=%u dir=%s",
		  __get_str(dev), __entry->ep_num, __entry->req,
		  __entry->actual, __entry->length, __entry->xfer_len,
		  __entry->status, __entry->dir ? "IN" : "OUT")
);

TRACE_EVENT(ls_vhub_ep0_stall_and_restart,

	TP_PROTO(struct ls_vhub_ep *ep),

	TP_ARGS(ep),

	TP_STRUCT__entry(
		__string(dev, dev_name(vhub_ep_dev(ep)))
		__field(u8, ep_num)
		__field(u8, enabled)
	),

	TP_fast_assign(
		__assign_str(dev);
		__entry->ep_num = ep->number;
		__entry->enabled = ep->enabled;
	),

	TP_printk("%s ep%u stall_and_restart enabled=%u",
		  __get_str(dev), __entry->ep_num, __entry->enabled)
);

TRACE_EVENT(ls_vhub_ep0_set_address,

	TP_PROTO(struct ls_vhub_dev *dev, u16 address),

	TP_ARGS(dev, address),

	TP_STRUCT__entry(
		__string(name, dev_name(dev->udev))
		__field(u8, port)
		__field(u16, address)
		__field(u8, ep0_out)
		__field(u8, ep0_in)
	),

	TP_fast_assign(
		__assign_str(name);
		__entry->port = dev->port_index;
		__entry->address = address;
		__entry->ep0_out = dev->ep0.out->number;
		__entry->ep0_in = dev->ep0.in->number;
	),

	TP_printk("%s port=%u set_address=%u ep0_out=%u ep0_in=%u",
		  __get_str(name), __entry->port, __entry->address,
		  __entry->ep0_out, __entry->ep0_in)
);

TRACE_EVENT(ls_vhub_port_set,

	TP_PROTO(struct ls_vhub *vhub, u8 port, const char *action_str,
		 u16 status, u16 change),

	TP_ARGS(vhub, port, action_str, status, change),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
		__string(action, action_str)
		__field(u8, port)
		__field(u16, status)
		__field(u16, change)
	),

	TP_fast_assign(
		__assign_str(dev);
		__assign_str(action);
		__entry->port = port;
		__entry->status = status;
		__entry->change = change;
	),

	TP_printk("%s port=%u action=%s status=0x%04x change=0x%04x",
		  __get_str(dev), __entry->port, __get_str(action),
		  __entry->status, __entry->change)
);

TRACE_EVENT(ls_vhub_ep0_setup,
	TP_PROTO(struct ls_vhub *vhub,
		 struct ls_vhub_ctrl_ep *ep0,
		 struct usb_ctrlrequest *setup),
	TP_ARGS(vhub, ep0, setup),

	TP_STRUCT__entry(
		__string(where,
			 ep0->out->dev ?
			 dev_name(ep0->out->dev->udev) :
			 (ep0->out->common && vhub->addr0_dev) ?
			 dev_name(vhub->addr0_dev->udev) :
			 (const char *)"hub")
		__field(u8, ep)
		__field(u8, bRequestType)
		__field(u8, bRequest)
		__field(u16, wValue)
		__field(u16, wIndex)
		__field(u16, wLength)
	),
	TP_fast_assign(
		__assign_str(where);
		__entry->ep = ep0->out->number;
		__entry->bRequestType = setup->bRequestType;
		__entry->bRequest = setup->bRequest;
		__entry->wValue = le16_to_cpu(setup->wValue);
		__entry->wIndex = le16_to_cpu(setup->wIndex);
		__entry->wLength = le16_to_cpu(setup->wLength);
	),

	TP_printk("%s ep%u: %s",
		  __get_str(where),
		  __entry->ep,
		  usb_decode_ctrl(__get_buf(VHUB_MSG_MAX), VHUB_MSG_MAX,
				  __entry->bRequestType,
				  __entry->bRequest,
				  __entry->wValue,
				  __entry->wIndex,
				  __entry->wLength))
);

TRACE_EVENT(ls_vhub_event,

	TP_PROTO(struct ls_vhub *vhub, u32 event),

	TP_ARGS(vhub, event),

	TP_STRUCT__entry(
		__string(dev, dev_name(&vhub->pdev->dev))
		__field(u32, event)
		__field(u8, is_dep)
		__field(u8, phy_ep_num)
		__field(u8, ep_evt_type)
		__field(u8, evt_status)
		__field(u16, evt_param)
		__field(u8, dev_evt_type)
		__field(u16, evt_info)
	),

	TP_fast_assign(
		const union evt_buf *__evt = (const union evt_buf *)&event;

		__assign_str(dev);
		__entry->event = event;
		__entry->is_dep = EVT_IS_DEPEVT(__evt);

		if (__entry->is_dep) {
			__entry->phy_ep_num = __evt->depevt.phy_ep_num;
			__entry->ep_evt_type = __evt->depevt.ep_evt_type;
			__entry->evt_status = __evt->depevt.evt_status;
			__entry->evt_param = __evt->depevt.evt_param;
			__entry->dev_evt_type = 0;
			__entry->evt_info = 0;
		} else {
			__entry->phy_ep_num = 0;
			__entry->ep_evt_type = 0;
			__entry->evt_status = 0;
			__entry->evt_param = 0;
			__entry->dev_evt_type = __evt->devt.dev_evt_type;
			__entry->evt_info = __evt->devt.evt_info;
		}
	),

	TP_printk("%s event=0x%08x %s %s ep=%u status=%u param=%u dev_info=%u",
		  __get_str(dev), __entry->event,
		  __entry->is_dep ? "DEP" : "DEV",
		  __entry->is_dep ?
			__print_symbolic(__entry->ep_evt_type,
				{ DEPEVT_XferComplete,   "XferComplete" },
				{ DEPEVT_XferInProgress, "XferInProgress" },
				{ DEPEVT_XferNotReady,   "XferNotReady" },
				{ DEPEVT_StreamEvt,      "StreamEvt" },
				{ DEPEVT_EPCmdCmplt,     "EPCmdCmplt" }) :
			__print_symbolic(__entry->dev_evt_type,
				{ DWC3_DEVICE_EVENT_DISCONNECT,         "DISCONNECT" },
				{ DWC3_DEVICE_EVENT_RESET,              "RESET" },
				{ DWC3_DEVICE_EVENT_CONNECT_DONE,       "CONNECT_DONE" },
				{ DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE, "LINK_STATUS_CHANGE" },
				{ DWC3_DEVICE_EVENT_WAKEUP,             "WAKEUP" },
				{ DWC3_DEVICE_EVENT_HIBER_REQ,          "HIBER_REQ" },
				{ DWC3_DEVICE_EVENT_SUSPEND,            "SUSPEND" },
				{ DWC3_DEVICE_EVENT_SOF,                "SOF" },
				{ DWC3_DEVICE_EVENT_ERRATIC_ERROR,      "ERRATIC_ERROR" },
				{ DWC3_DEVICE_EVENT_CMD_CMPL,           "CMD_CMPL" },
				{ DWC3_DEVICE_EVENT_OVERFLOW,           "OVERFLOW" }),
		  __entry->phy_ep_num,
		  __entry->evt_status,
		  __entry->evt_param,
		  __entry->evt_info)
);

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

#include <trace/define_trace.h>
