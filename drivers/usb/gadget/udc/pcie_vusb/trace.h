#undef TRACE_SYSTEM
#define TRACE_SYSTEM ls_pcie_vusb

#if !defined(_TRACE_LS_PCIE_VUSB_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_LS_PCIE_VUSB_H

#include <linux/tracepoint.h>
#include <linux/usb/ch9.h>

#define VUSB_MSG_MAX 500

TRACE_EVENT(ep0_match,

    TP_PROTO(u32 slot_id, u32 stall, u32 has_data,
         u32 dir, u32 len, u32 setup_wlen),

    TP_ARGS(slot_id, stall, has_data, dir, len, setup_wlen),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, stall)
        __field(u32, has_data)
        __field(u32, dir)
        __field(u32, len)
        __field(u32, setup_wlen)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->stall = stall;
        __entry->has_data = has_data;
        __entry->dir = dir;
        __entry->len = len;
        __entry->setup_wlen = setup_wlen;
    ),

    TP_printk("slot=%u stall=%u has_data=%u dir=%s len=%u wlen=%u",
          __entry->slot_id, __entry->stall, __entry->has_data,
          __entry->dir ? "IN" : "OUT", __entry->len, __entry->setup_wlen)
);

TRACE_EVENT(epn_in_match,

    TP_PROTO(u32 slot_id, u32 ep, u32 trb_remain, u32 req_remain,
         u32 xfer_len, u32 stall),

    TP_ARGS(slot_id, ep, trb_remain, req_remain, xfer_len, stall),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, ep)
        __field(u32, trb_remain)
        __field(u32, req_remain)
        __field(u32, xfer_len)
        __field(u32, stall)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->ep = ep;
        __entry->trb_remain = trb_remain;
        __entry->req_remain = req_remain;
        __entry->xfer_len = xfer_len;
        __entry->stall = stall;
    ),

    TP_printk("slot=%u ep=%u trb_remain=%u req_remain=%u xfer_len=%u stall=%u",
          __entry->slot_id, __entry->ep, __entry->trb_remain,
          __entry->req_remain, __entry->xfer_len, __entry->stall)
);

TRACE_EVENT(epn_out_match,

    TP_PROTO(u32 slot_id, u32 ep, u32 trb_remain, u32 req_remain,
         u32 xfer_len, u32 stall),

    TP_ARGS(slot_id, ep, trb_remain, req_remain, xfer_len, stall),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, ep)
        __field(u32, trb_remain)
        __field(u32, req_remain)
        __field(u32, xfer_len)
        __field(u32, stall)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->ep = ep;
        __entry->trb_remain = trb_remain;
        __entry->req_remain = req_remain;
        __entry->xfer_len = xfer_len;
        __entry->stall = stall;
    ),

    TP_printk("slot=%u ep=%u trb_remain=%u req_remain=%u xfer_len=%u stall=%u",
          __entry->slot_id, __entry->ep, __entry->trb_remain,
          __entry->req_remain, __entry->xfer_len, __entry->stall)
);

TRACE_EVENT(req_submit,

    TP_PROTO(u32 slot_id, u32 ep,
         u32 len, u32 dir),

    TP_ARGS(slot_id, ep, len, dir),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, ep)
        __field(u32, len)
        __field(u32, dir)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->ep = ep;
        __entry->len = len;
        __entry->dir = dir;
    ),

    TP_printk("slot=%u ep=%u dir=%s len=%u",
          __entry->slot_id, __entry->ep,
          __entry->dir ? "IN" : "OUT", __entry->len)
);

TRACE_EVENT(reg_event,

    TP_PROTO(u32 reg, u32 data),

    TP_ARGS(reg, data),

    TP_STRUCT__entry(
        __field(u32, reg)
        __field(u32, data)
    ),

    TP_fast_assign(
        __entry->reg = reg;
        __entry->data = data;
    ),

    TP_printk("reg=%s data=0x%x",
          __print_symbolic(__entry->reg,
            { 0x80,  "USBCMD" },
            { 0x84,  "USBSTS" },
            { 0x88,  "PAGESIZE" },
            { 0x94,  "DNCTRL" },
            { 0x98,  "CRCR_LO" },
            { 0x9c,  "CRCR_HI" },
            { 0xb0,  "DCBAAP_LO" },
            { 0xb4,  "DCBAAP_HI" },
            { 0xb8,  "CONFIG" },
            { 0x180, "PORTSC" },
            { 0x184, "PORTSC1" },
            { 0x188, "PORTSC2" },
            { 0x18c, "PORTSC3" },
            { 0x1c0, "IMAN" },
            { 0x1c4, "IMOD" },
            { 0x1c8, "ERSTSZ" },
            { 0x1d0, "ERSTBA_LO" },
            { 0x1d4, "ERSTBA_HI" },
            { 0x1d8, "ERDP_LO" },
            { 0x1dc, "ERDP_HI" }),
          __entry->data)
);

TRACE_EVENT(doorbell,

    TP_PROTO(u32 slot_id, u32 data),

    TP_ARGS(slot_id, data),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, data)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->data = data;
    ),

    TP_printk("%s handle slot=%u data=0x%x",
          __entry->slot_id ? "slot" : "cmd",
          __entry->slot_id, __entry->data)
);

TRACE_EVENT(xfer_submit,

    TP_PROTO(u32 slot_id, u32 slot_state, u32 ep, u32 ep_state),

    TP_ARGS(slot_id, slot_state, ep, ep_state),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
        __field(u32, ep)
        __field(u32, ep_state)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
        __entry->ep = ep;
        __entry->ep_state = ep_state;
    ),

    TP_printk("slot=%u slot_state=%u ep=%u ep_state=%u",
          __entry->slot_id, __entry->slot_state,
          __entry->ep, __entry->ep_state)
);

TRACE_EVENT(port_change,

    TP_PROTO(u32 attach, u32 port),

    TP_ARGS(attach, port),

    TP_STRUCT__entry(
        __field(u32, attach)
        __field(u32, port)
    ),

    TP_fast_assign(
        __entry->attach = attach;
        __entry->port = port;
    ),

    TP_printk("attach=%u port=%u",
          __entry->attach, __entry->port)
);

TRACE_EVENT(enable_slot,

    TP_PROTO(int slot_id),

    TP_ARGS(slot_id),

    TP_STRUCT__entry(
        __field(int, slot_id)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
    ),

    TP_printk("slot=%d", __entry->slot_id)
);

TRACE_EVENT(disable_slot,

    TP_PROTO(u32 slot_id, u32 slot_state),

    TP_ARGS(slot_id, slot_state),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
    ),

    TP_printk("slot=%u state=%s",
          __entry->slot_id,
          __print_symbolic(__entry->slot_state,
            { 0, "DISABLE" },
            { 1, "ENABLE" },
            { 2, "DEFAULT" },
            { 3, "ADDRESSED" },
            { 4, "CONFIGURED" }))
);

TRACE_EVENT(addr_dev,

    TP_PROTO(u32 slot_id, u32 slot_state, u32 bsr, u32 addr),

    TP_ARGS(slot_id, slot_state, bsr, addr),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
        __field(u32, bsr)
        __field(u32, addr)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
        __entry->bsr = bsr;
        __entry->addr = addr;
    ),

    TP_printk("slot=%u state=%s bsr=%u addr=%u",
          __entry->slot_id,
          __print_symbolic(__entry->slot_state,
            { 0, "DISABLE" },
            { 1, "ENABLE" },
            { 2, "DEFAULT" },
            { 3, "ADDRESSED" },
            { 4, "CONFIGURED" }),
          __entry->bsr, __entry->addr)
);

TRACE_EVENT(reset_dev,

    TP_PROTO(u32 slot_id, u32 slot_state),

    TP_ARGS(slot_id, slot_state),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
    ),

    TP_printk("slot=%u state=%s",
          __entry->slot_id,
          __print_symbolic(__entry->slot_state,
            { 0, "DISABLE" },
            { 1, "ENABLE" },
            { 2, "DEFAULT" },
            { 3, "ADDRESSED" },
            { 4, "CONFIGURED" }))
);

TRACE_EVENT(config_ep,

    TP_PROTO(u32 slot_id, u32 slot_state, u32 add_flags, u32 drop_flags),

    TP_ARGS(slot_id, slot_state, add_flags, drop_flags),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
        __field(u32, add_flags)
        __field(u32, drop_flags)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
        __entry->add_flags = add_flags;
        __entry->drop_flags = drop_flags;
    ),

    TP_printk("slot=%u state=%s add=0x%x drop=0x%x",
          __entry->slot_id,
          __print_symbolic(__entry->slot_state,
            { 0, "DISABLE" },
            { 1, "ENABLE" },
            { 2, "DEFAULT" },
            { 3, "ADDRESSED" },
            { 4, "CONFIGURED" }),
          __entry->add_flags, __entry->drop_flags)
);

TRACE_EVENT(deconfig_ep,

    TP_PROTO(u32 slot_id, u32 slot_state),

    TP_ARGS(slot_id, slot_state),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
    ),

    TP_printk("slot=%u state=%s",
          __entry->slot_id,
          __print_symbolic(__entry->slot_state,
            { 0, "DISABLE" },
            { 1, "ENABLE" },
            { 2, "DEFAULT" },
            { 3, "ADDRESSED" },
            { 4, "CONFIGURED" }))
);

TRACE_EVENT(eval_context,

    TP_PROTO(u32 slot_id, u32 slot_state),

    TP_ARGS(slot_id, slot_state),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
    ),

    TP_printk("slot=%u state=%s",
          __entry->slot_id,
          __print_symbolic(__entry->slot_state,
            { 0, "DISABLE" },
            { 1, "ENABLE" },
            { 2, "DEFAULT" },
            { 3, "ADDRESSED" },
            { 4, "CONFIGURED" }))
);

TRACE_EVENT(stop_ring,

    TP_PROTO(u32 slot_id, u32 slot_state, u32 ep, u32 ep_state, u64 deq),

    TP_ARGS(slot_id, slot_state, ep, ep_state, deq),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
        __field(u32, ep)
        __field(u32, ep_state)
        __field(u64, deq)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
        __entry->ep = ep;
        __entry->ep_state = ep_state;
        __entry->deq = deq;
    ),

    TP_printk("slot=%u state=%s ep=%u ep_state=%s deq=0x%llx",
          __entry->slot_id,
          __print_symbolic(__entry->slot_state,
            { 0, "DISABLE" },
            { 1, "ENABLE" },
            { 2, "DEFAULT" },
            { 3, "ADDRESSED" },
            { 4, "CONFIGURED" }),
          __entry->ep,
          __print_symbolic(__entry->ep_state,
            { 0, "DISABLE" },
            { 1, "RUNNING" },
            { 2, "HALTED" },
            { 3, "STOPPED" },
            { 4, "ERROR" }),
          __entry->deq)
);

TRACE_EVENT(reset_ep,

    TP_PROTO(u32 slot_id, u32 slot_state, u32 ep, u32 ep_state, u32 tsp),

    TP_ARGS(slot_id, slot_state, ep, ep_state, tsp),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
        __field(u32, ep)
        __field(u32, ep_state)
        __field(u32, tsp)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
        __entry->ep = ep;
        __entry->ep_state = ep_state;
        __entry->tsp = tsp;
    ),

    TP_printk("slot=%u state=%s ep=%u ep_state=%s tsp=%u",
          __entry->slot_id,
          __print_symbolic(__entry->slot_state,
            { 0, "DISABLE" },
            { 1, "ENABLE" },
            { 2, "DEFAULT" },
            { 3, "ADDRESSED" },
            { 4, "CONFIGURED" }),
          __entry->ep,
          __print_symbolic(__entry->ep_state,
            { 0, "DISABLE" },
            { 1, "RUNNING" },
            { 2, "HALTED" },
            { 3, "STOPPED" },
            { 4, "ERROR" }),
          __entry->tsp)
);

TRACE_EVENT(set_deq,

    TP_PROTO(u32 slot_id, u32 slot_state, u32 ep, u32 ep_state,
         u32 stream_id, u64 deq),

    TP_ARGS(slot_id, slot_state, ep, ep_state, stream_id, deq),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, slot_state)
        __field(u32, ep)
        __field(u32, ep_state)
        __field(u32, stream_id)
        __field(u64, deq)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->slot_state = slot_state;
        __entry->ep = ep;
        __entry->ep_state = ep_state;
        __entry->stream_id = stream_id;
        __entry->deq = deq;
    ),

    TP_printk("slot=%u state=%s ep=%u ep_state=%s stream=%u deq=0x%llx",
          __entry->slot_id,
          __print_symbolic(__entry->slot_state,
            { 0, "DISABLE" },
            { 1, "ENABLE" },
            { 2, "DEFAULT" },
            { 3, "ADDRESSED" },
            { 4, "CONFIGURED" }),
          __entry->ep,
          __print_symbolic(__entry->ep_state,
            { 0, "DISABLE" },
            { 1, "RUNNING" },
            { 2, "HALTED" },
            { 3, "STOPPED" },
            { 4, "ERROR" }),
          __entry->stream_id, __entry->deq)
);

TRACE_EVENT(cmd_noop,

    TP_PROTO(u64 deq),

    TP_ARGS(deq),

    TP_STRUCT__entry(
        __field(u64, deq)
    ),

    TP_fast_assign(
        __entry->deq = deq;
    ),

    TP_printk("deq=0x%llx", __entry->deq)
);

TRACE_EVENT(cmd_failed,

    TP_PROTO(u32 type, int ret, u32 handled),

    TP_ARGS(type, ret, handled),

    TP_STRUCT__entry(
        __field(u32, type)
        __field(int, ret)
        __field(u32, handled)
    ),

    TP_fast_assign(
        __entry->type = type;
        __entry->ret = ret;
        __entry->handled = handled;
    ),

    TP_printk("cmd=%s ret=%d handled=%u",
          __print_symbolic(__entry->type,
            { 6,  "LINK" },
            { 9,  "ENABLE_SLOT" },
            { 10, "DISABLE_SLOT" },
            { 11, "ADDR_DEV" },
            { 12, "CONFIG_EP" },
            { 13, "EVAL_CONTEXT" },
            { 14, "RESET_EP" },
            { 15, "STOP_RING" },
            { 16, "SET_DEQ" },
            { 17, "RESET_DEV" },
            { 18, "FORCE_EVENT" },
            { 19, "NEG_BANDWIDTH" },
            { 20, "SET_LT" },
            { 21, "GET_BW" },
            { 22, "FORCE_HEADER" },
            { 23, "CMD_NOOP" }),
          __entry->ret, __entry->handled)
);

TRACE_EVENT(normal_trb,

    TP_PROTO(u32 slot_id, u32 ep, u32 dir, u32 trb_len,
         u32 ch, u32 isp, u32 ioc),

    TP_ARGS(slot_id, ep, dir, trb_len, ch, isp, ioc),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, ep)
        __field(u32, dir)
        __field(u32, trb_len)
        __field(u32, ch)
        __field(u32, isp)
        __field(u32, ioc)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->ep = ep;
        __entry->dir = dir;
        __entry->trb_len = trb_len;
        __entry->ch = ch;
        __entry->isp = isp;
        __entry->ioc = ioc;
    ),

    TP_printk("slot=%u ep=%u dir=%s trb_len=%u ch=%u isp=%u ioc=%u",
          __entry->slot_id, __entry->ep,
          __entry->dir ? "OUT" : "IN",
          __entry->trb_len, __entry->ch, __entry->isp, __entry->ioc)
);

TRACE_EVENT(setup_stage,

    TP_PROTO(u32 slot_id, u32 ep, u32 trt, u32 bmreqtype, u32 breq,
         u32 wvalue, u32 windex, u32 wlen),

    TP_ARGS(slot_id, ep, trt, bmreqtype, breq, wvalue, windex, wlen),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, ep)
        __field(u32, trt)
        __field(u32, bmreqtype)
        __field(u32, breq)
        __field(u32, wvalue)
        __field(u32, windex)
        __field(u32, wlen)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->ep = ep;
        __entry->trt = trt;
        __entry->bmreqtype = bmreqtype;
        __entry->breq = breq;
        __entry->wvalue = wvalue;
        __entry->windex = windex;
        __entry->wlen = wlen;
    ),

    TP_printk("slot=%u ep=%u trt=%s %s",
          __entry->slot_id, __entry->ep,
          __print_symbolic(__entry->trt,
            { 0, "NO_DATA" },
            { 1, "RSVD" },
            { 2, "OUT_DATA" },
            { 3, "IN_DATA" }),
          usb_decode_ctrl(__get_buf(VUSB_MSG_MAX), VUSB_MSG_MAX,
                  __entry->bmreqtype, __entry->breq,
                  __entry->wvalue, __entry->windex, __entry->wlen))
);

TRACE_EVENT(data_stage,

    TP_PROTO(u32 slot_id, u32 ep, u32 len, u32 dir,
         u32 ch, u32 isp, u32 ioc),

    TP_ARGS(slot_id, ep, len, dir, ch, isp, ioc),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, ep)
        __field(u32, len)
        __field(u32, dir)
        __field(u32, ch)
        __field(u32, isp)
        __field(u32, ioc)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->ep = ep;
        __entry->len = len;
        __entry->dir = dir;
        __entry->ch = ch;
        __entry->isp = isp;
        __entry->ioc = ioc;
    ),

    TP_printk("slot=%u ep=%u dir=%s len=%u ch=%u isp=%u ioc=%u",
          __entry->slot_id, __entry->ep,
          __entry->dir ? "IN" : "OUT",
          __entry->len, __entry->ch, __entry->isp, __entry->ioc)
);

TRACE_EVENT(status_stage,

    TP_PROTO(u32 slot_id, u32 ep, u32 dir, u32 ch, u32 ioc),

    TP_ARGS(slot_id, ep, dir, ch, ioc),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, ep)
        __field(u32, dir)
        __field(u32, ch)
        __field(u32, ioc)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->ep = ep;
        __entry->dir = dir;
        __entry->ch = ch;
        __entry->ioc = ioc;
    ),

    TP_printk("slot=%u ep=%u dir=%s ch=%u ioc=%u",
          __entry->slot_id, __entry->ep,
          __entry->dir ? "IN" : "OUT",
          __entry->ch, __entry->ioc)
);

TRACE_EVENT(post_event,

    TP_PROTO(u32 type, u32 ep_id, u32 code),

    TP_ARGS(type, ep_id, code),

    TP_STRUCT__entry(
        __field(u32, type)
        __field(u32, ep_id)
        __field(u32, code)
    ),

    TP_fast_assign(
        __entry->type = type;
        __entry->ep_id = ep_id;
        __entry->code = code;
    ),

    TP_printk("type=%s ep_id=%u code=%s",
          __print_symbolic(__entry->type,
            { 32, "TRANSFER" },
            { 33, "COMPLETION" },
            { 34, "PORT_STATUS" }),
          __entry->ep_id,
          __print_symbolic(__entry->code,
            { 1,  "SUCCESS" },
            { 4,  "USB_XACT_ERR" },
            { 5,  "TRB_ERR" },
            { 6,  "STALL" },
            { 7,  "RESOURCE" },
            { 9,  "NO_SLOTS" },
            { 11, "SLOT_NOT_ENABLED" },
            { 13, "SHORT_PACKET" },
            { 17, "PARAMETER" },
            { 19, "CONTEXT_STATE" }))
);

TRACE_EVENT(ep_failed,

    TP_PROTO(u32 slot_id, u32 ep, u32 type, int ret, u32 handled),

    TP_ARGS(slot_id, ep, type, ret, handled),

    TP_STRUCT__entry(
        __field(u32, slot_id)
        __field(u32, ep)
        __field(u32, type)
        __field(int, ret)
        __field(u32, handled)
    ),

    TP_fast_assign(
        __entry->slot_id = slot_id;
        __entry->ep = ep;
        __entry->type = type;
        __entry->ret = ret;
        __entry->handled = handled;
    ),

    TP_printk("slot=%u ep=%u trb=%s ret=%d handled=%u",
          __entry->slot_id, __entry->ep,
          __print_symbolic(__entry->type,
            { 1, "NORMAL" },
            { 2, "SETUP" },
            { 3, "DATA" },
            { 4, "STATUS" },
            { 5, "ISOC" },
            { 6, "LINK" },
            { 7, "EVENT_DATA" },
            { 8, "TR_NOOP" }),
          __entry->ret, __entry->handled)
);

#endif /* _TRACE_LS_PCIE_VUSB_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

#include <trace/define_trace.h>
