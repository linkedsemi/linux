// SPDX-License-Identifier: GPL-2.0
/*
 * ls-vhub debugfs interface
 *
 *   /sys/kernel/debug/usb/<vhub_name>/
 *   ├── regdump          — register dump (debugfs_regset32, incl. 32 DEPs)
 *   ├── events           — raw event buffer
 *   ├── fifo             — TX/RX FIFO space
 *   ├── link_state       — current USB link state (DSTS)
 *   ├── ep_map           — EP map table (disabled: hw regs write-only)
 *   ├── ep_pool          — phys EP pool (32 entries, used/free per EP)
 *   ├── ports            — all downstream ports at a glance
 *   ├── ep0              — hub own EP0 (common + addressed) + TRBs
 *   ├── ep1              — hub interrupt endpoint + TRBs
 *   ├── dev-0/           — downstream device 0
 *   │   ├── state        — address, enabled, driver, gadget_state
 *   │   ├── ep0          — device EP0 state + TRBs
 *   │   ├── eps          — allocated endpoint list (name/num/stall/dir/type)
 *   │   └── trbs         — TRB dump for all endpoints of this device
 *   ...
 */

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include "debugfs.h"
#include "vhub.h"

/* ===================================================================
 * Register dump — static table, used by debugfs_regset32
 * =================================================================== */

#define dump_register(nm) \
    { .name = __stringify(nm), .offset = DWC3_ ##nm, }

#define DEPREG(n, reg, off) \
    { .name = "DEP" #reg "(" #n ")", .offset = (0xc800 + (n) * 0x10 + (off)), }

#define dump_ep_register_set(n) \
    DEPREG(n, CMDPAR2, 0x00), \
    DEPREG(n, CMDPAR1, 0x04), \
    DEPREG(n, CMDPAR0, 0x08), \
    DEPREG(n, CMD,     0x0c),

static const struct debugfs_reg32 dwc3_regs[] = {
    /* Global */
    dump_register(GSBUSCFG0),
    dump_register(GSBUSCFG1),
    dump_register(GTXTHRCFG),
    dump_register(GRXTHRCFG),
    dump_register(GCTL),
    dump_register(GPMSTS),
    dump_register(GSTS),
    dump_register(GUCTL1),
    dump_register(GSNPSID),
    dump_register(GGPIO),
    dump_register(GUID),
    dump_register(GUCTL),
    dump_register(GBUSERRADDRLO),
    dump_register(GBUSERRADDRHI),
    dump_register(GPRTBIMAP0),
    dump_register(GPRTBIMAP1),
    dump_register(GHWPARAMS0),
    dump_register(GHWPARAMS1),
    dump_register(GHWPARAMS2),
    dump_register(GHWPARAMS3),
    dump_register(GHWPARAMS4),
    dump_register(GHWPARAMS5),
    dump_register(GHWPARAMS6),
    dump_register(GHWPARAMS7),
    dump_register(GDBGFIFOSPACE),
    dump_register(GDBGLTSSM),
    dump_register(GDBGBMU),
    dump_register(GPRTBIMAP_HS0),
    dump_register(GPRTBIMAP_HS1),
    dump_register(GPRTBIMAP_FS0),
    dump_register(GPRTBIMAP_FS1),
    dump_register(GUCTL2),
    dump_register(VER_NUMBER),
    dump_register(VER_TYPE),

    /* USB2 PHY */
    dump_register(GUSB2PHYCFG(0)),
    dump_register(GUSB2PHYCFG(1)),
    dump_register(GUSB2PHYCFG(2)),
    dump_register(GUSB2PHYCFG(3)),
    dump_register(GUSB2PHYCFG(4)),
    dump_register(GUSB2PHYCFG(5)),
    dump_register(GUSB2PHYCFG(6)),
    dump_register(GUSB2PHYCFG(7)),
    dump_register(GUSB2PHYCFG(8)),
    dump_register(GUSB2PHYCFG(9)),
    dump_register(GUSB2PHYCFG(10)),
    dump_register(GUSB2PHYCFG(11)),
    dump_register(GUSB2PHYCFG(12)),
    dump_register(GUSB2PHYCFG(13)),
    dump_register(GUSB2PHYCFG(14)),
    dump_register(GUSB2PHYCFG(15)),

    dump_register(GUSB2I2CCTL(0)),
    dump_register(GUSB2I2CCTL(1)),
    dump_register(GUSB2I2CCTL(2)),
    dump_register(GUSB2I2CCTL(3)),
    dump_register(GUSB2I2CCTL(4)),
    dump_register(GUSB2I2CCTL(5)),
    dump_register(GUSB2I2CCTL(6)),
    dump_register(GUSB2I2CCTL(7)),
    dump_register(GUSB2I2CCTL(8)),
    dump_register(GUSB2I2CCTL(9)),
    dump_register(GUSB2I2CCTL(10)),
    dump_register(GUSB2I2CCTL(11)),
    dump_register(GUSB2I2CCTL(12)),
    dump_register(GUSB2I2CCTL(13)),
    dump_register(GUSB2I2CCTL(14)),
    dump_register(GUSB2I2CCTL(15)),

    dump_register(GUSB2PHYACC(0)),
    dump_register(GUSB2PHYACC(1)),
    dump_register(GUSB2PHYACC(2)),
    dump_register(GUSB2PHYACC(3)),
    dump_register(GUSB2PHYACC(4)),
    dump_register(GUSB2PHYACC(5)),
    dump_register(GUSB2PHYACC(6)),
    dump_register(GUSB2PHYACC(7)),
    dump_register(GUSB2PHYACC(8)),
    dump_register(GUSB2PHYACC(9)),
    dump_register(GUSB2PHYACC(10)),
    dump_register(GUSB2PHYACC(11)),
    dump_register(GUSB2PHYACC(12)),
    dump_register(GUSB2PHYACC(13)),
    dump_register(GUSB2PHYACC(14)),
    dump_register(GUSB2PHYACC(15)),

    /* TX FIFO sizing registers */
    dump_register(GTXFIFOSIZ(0)),
    dump_register(GTXFIFOSIZ(1)),
    dump_register(GTXFIFOSIZ(2)),
    dump_register(GTXFIFOSIZ(3)),
    dump_register(GTXFIFOSIZ(4)),
    dump_register(GTXFIFOSIZ(5)),
    dump_register(GTXFIFOSIZ(6)),
    dump_register(GTXFIFOSIZ(7)),
    dump_register(GTXFIFOSIZ(8)),
    dump_register(GTXFIFOSIZ(9)),
    dump_register(GTXFIFOSIZ(10)),
    dump_register(GTXFIFOSIZ(11)),
    dump_register(GTXFIFOSIZ(12)),
    dump_register(GTXFIFOSIZ(13)),
    dump_register(GTXFIFOSIZ(14)),
    dump_register(GTXFIFOSIZ(15)),
    dump_register(GTXFIFOSIZ(16)),
    dump_register(GTXFIFOSIZ(17)),
    dump_register(GTXFIFOSIZ(18)),
    dump_register(GTXFIFOSIZ(19)),
    dump_register(GTXFIFOSIZ(20)),
    dump_register(GTXFIFOSIZ(21)),
    dump_register(GTXFIFOSIZ(22)),
    dump_register(GTXFIFOSIZ(23)),
    dump_register(GTXFIFOSIZ(24)),
    dump_register(GTXFIFOSIZ(25)),
    dump_register(GTXFIFOSIZ(26)),
    dump_register(GTXFIFOSIZ(27)),
    dump_register(GTXFIFOSIZ(28)),
    dump_register(GTXFIFOSIZ(29)),
    dump_register(GTXFIFOSIZ(30)),
    dump_register(GTXFIFOSIZ(31)),

    /* RX FIFO sizing registers */
    dump_register(GRXFIFOSIZ(0)),
    dump_register(GRXFIFOSIZ(1)),
    dump_register(GRXFIFOSIZ(2)),
    dump_register(GRXFIFOSIZ(3)),
    dump_register(GRXFIFOSIZ(4)),
    dump_register(GRXFIFOSIZ(5)),
    dump_register(GRXFIFOSIZ(6)),
    dump_register(GRXFIFOSIZ(7)),
    dump_register(GRXFIFOSIZ(8)),
    dump_register(GRXFIFOSIZ(9)),
    dump_register(GRXFIFOSIZ(10)),
    dump_register(GRXFIFOSIZ(11)),
    dump_register(GRXFIFOSIZ(12)),
    dump_register(GRXFIFOSIZ(13)),
    dump_register(GRXFIFOSIZ(14)),
    dump_register(GRXFIFOSIZ(15)),
    dump_register(GRXFIFOSIZ(16)),
    dump_register(GRXFIFOSIZ(17)),
    dump_register(GRXFIFOSIZ(18)),
    dump_register(GRXFIFOSIZ(19)),
    dump_register(GRXFIFOSIZ(20)),
    dump_register(GRXFIFOSIZ(21)),
    dump_register(GRXFIFOSIZ(22)),
    dump_register(GRXFIFOSIZ(23)),
    dump_register(GRXFIFOSIZ(24)),
    dump_register(GRXFIFOSIZ(25)),
    dump_register(GRXFIFOSIZ(26)),
    dump_register(GRXFIFOSIZ(27)),
    dump_register(GRXFIFOSIZ(28)),
    dump_register(GRXFIFOSIZ(29)),
    dump_register(GRXFIFOSIZ(30)),
    dump_register(GRXFIFOSIZ(31)),

    /* Event buffer & device */
    dump_register(GEVNTADRLO(0)),
    dump_register(GEVNTADRHI(0)),
    dump_register(GEVNTSIZ(0)),
    dump_register(GEVNTCOUNT(0)),

    dump_register(GHWPARAMS8),
    dump_register(GUCTL3),
    dump_register(GFLADJ),
    dump_register(DCFG),
    dump_register(DCTL),
    dump_register(DEVTEN),
    dump_register(DSTS),
    dump_register(DGCMDPAR),
    dump_register(DGCMD),
    dump_register(DALEPENA),

    /* Per-endpoint DEP registers (device-mode physical endpoints) */
    dump_ep_register_set(0)
    dump_ep_register_set(1)
    dump_ep_register_set(2)
    dump_ep_register_set(3)
    dump_ep_register_set(4)
    dump_ep_register_set(5)
    dump_ep_register_set(6)
    dump_ep_register_set(7)
    dump_ep_register_set(8)
    dump_ep_register_set(9)
    dump_ep_register_set(10)
    dump_ep_register_set(11)
    dump_ep_register_set(12)
    dump_ep_register_set(13)
    dump_ep_register_set(14)
    dump_ep_register_set(15)
    dump_ep_register_set(16)
    dump_ep_register_set(17)
    dump_ep_register_set(18)
    dump_ep_register_set(19)
    dump_ep_register_set(20)
    dump_ep_register_set(21)
    dump_ep_register_set(22)
    dump_ep_register_set(23)
    dump_ep_register_set(24)
    dump_ep_register_set(25)
    dump_ep_register_set(26)
    dump_ep_register_set(27)
    dump_ep_register_set(28)
    dump_ep_register_set(29)
    dump_ep_register_set(30)
    dump_ep_register_set(31)
};

/* ===================================================================
 * Helpers — shared formatting routines
 * =================================================================== */

static const char *ep0_state_str(enum ls_vhub_ep0_state s)
{
    switch (s) {
    case EP0_UNCONNECTED:  return "UNCONNECTED";
    case EP0_SETUP_PHASE:  return "SETUP";
    case EP0_DATA_PHASE:   return "DATA";
    case EP0_STATUS_PHASE: return "STATUS";
    default:               return "?";
    }
}

static void ep_show_one(struct seq_file *s, const char *label, struct ls_vhub_ep *ep)
{
    if (!ep) {
        seq_printf(s, "  %-4s: (null)\n", label);
        return;
    }
    seq_printf(s, "  %-4s: num=%-2d en=%-1d stall=%-1d wedge=%-1d active=%-1d res=%-1d  req_q=%s\n",
               label, ep->number, ep->enabled, ep->stall, ep->wedge,
               ep->active, ep->resource,
               list_empty(&ep->req_list) ? "empty" : "pending");
}

static __printf(3, 4) void ep0_ctrl_show(struct seq_file *s,
                        struct ls_vhub_ctrl_ep *ep0, const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    seq_vprintf(s, fmt, args);
    va_end(args);
    seq_printf(s, ": state=%-8s three_stage=%d expect_in=%d data_len=%u\n",
               ep0_state_str(ep0->state),
               ep0->three_stage_setup, ep0->expect_in, ep0->data_len);
    ep_show_one(s, "OUT", ep0->out);
    ep_show_one(s, "IN",  ep0->in);
    seq_putc(s, '\n');
}

static const char *trb_type_str(u32 ctrl)
{
    switch (DWC3_TRBCTL_TYPE(ctrl)) {
    case DWC3_TRBCTL_NORMAL:          return "NORMAL";
    case DWC3_TRBCTL_CONTROL_SETUP:   return "SETUP";
    case DWC3_TRBCTL_CONTROL_STATUS2: return "STATUS2";
    case DWC3_TRBCTL_CONTROL_STATUS3: return "STATUS3";
    case DWC3_TRBCTL_CONTROL_DATA:    return "DATA";
    default:                          return "?";
    }
}

static void trb_show_one(struct seq_file *s, int idx, struct dwc3_trb *trb)
{
    u32 ctrl = trb->ctrl;

    seq_printf(s, "  TRB[%d]: buf=0x%08x%08x size=%u type=%-7s %s%s%s%s\n",
               idx, trb->bptrh, trb->bptrl, trb->size & DWC3_TRB_SIZE_MASK,
               trb_type_str(ctrl),
               (ctrl & DWC3_TRB_CTRL_HWO) ? "HWO " : "",
               (ctrl & DWC3_TRB_CTRL_IOC) ? "IOC " : "",
               (ctrl & DWC3_TRB_CTRL_CHN) ? "CHN " : "",
               (ctrl & DWC3_TRB_CTRL_LST) ? "LST" : "");
}

static void ep0_trb_dump(struct seq_file *s, struct ls_vhub_ctrl_ep *ep0)
{
    if (ep0->out && ep0->out->trb) {
        seq_puts(s, "  OUT TRBs:\n");
        trb_show_one(s, 0, &ep0->out->trb->trb[0]);
        trb_show_one(s, 1, &ep0->out->trb->trb[1]);
    }
    if (ep0->in && ep0->in->trb) {
        seq_puts(s, "  IN TRBs:\n");
        trb_show_one(s, 0, &ep0->in->trb->trb[0]);
        trb_show_one(s, 1, &ep0->in->trb->trb[1]);
    }
}

/* ===================================================================
 * vhub-level: events — raw event buffer dump
 * =================================================================== */

static int events_show(struct seq_file *s, void *unused)
{
    struct ls_vhub *vhub = s->private;
    struct event_buffer *evt = &vhub->evt_buf;
    int count = evt->length / sizeof(u32);

    seq_printf(s, "length=%u  dequeue=%u/%d  dma=%pad\n\n",
               evt->length, evt->dequeue, count, &evt->dma);

    for (int i = 0; i < count; i++) {
        const union evt_buf *e = &((union evt_buf *)evt->buf)[i];

        seq_printf(s, "  [%02d] 0x%08x  ", i, ((u32 *)evt->buf)[i]);

        if (EVT_IS_DEPEVT(e))
            seq_printf(s, "DEP: ep=%u type=%u status=%u param=%u\n",
                       e->depevt.phy_ep_num, e->depevt.ep_evt_type,
                       e->depevt.evt_status, e->depevt.evt_param);
        else
            seq_printf(s, "DEV: type=%u info=%u\n",
                       e->devt.dev_evt_type, e->devt.evt_info);
    }
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(events);

/* ===================================================================
 * vhub-level: fifo — TX/RX FIFO space usage
 * =================================================================== */

static int fifo_show(struct seq_file *s, void *unused)
{
    struct ls_vhub *vhub = s->private;
    u32 mdwidth = DWC3_HWPARAMS0_MDWIDTH(vhub->hwparams.hwparams0);
    u32 word_bytes = mdwidth / 8;
    int num_eps = DWC3_NUM_EPS(vhub->hwparams.hwparams3);

    seq_printf(s, "MDWIDTH=%u bits (%u B/word)  NUM_EPS=%d\n\n",
               mdwidth, word_bytes, num_eps);

    seq_puts(s, "--- TX FIFO ---\n");
    seq_puts(s,
        "EP  FIFO#   Space(words)   Depth(words)   Space(bytes)   Depth(bytes)\n");

    for (int ep = 1; ep < num_eps; ep += 2) {
        u8 fifo_nr = ep >> 1;
        u32 txfifosiz = vhub_readl(vhub->regs, DWC3_GTXFIFOSIZ(fifo_nr));
        u32 depth = txfifosiz & 0xffff;

        if (depth == 0)
            continue;

        vhub_writel(vhub->regs, DWC3_GDBGFIFOSPACE,
                    DWC3_GDBGFIFOSPACE_NUM(fifo_nr) |
                    DWC3_GDBGFIFOSPACE_TYPE(0));
        u32 space = DWC3_GDBGFIFOSPACE_SPACE(
                    vhub_readl(vhub->regs, DWC3_GDBGFIFOSPACE));

        seq_printf(s, "%-4d %-6u %-14u %-14u %-14u %-14u%s\n",
                   ep, fifo_nr, space, depth,
                   space * word_bytes, depth * word_bytes,
                   space == depth ? " [empty]" : "");
    }

    /* RX FIFO (shared) */
    seq_puts(s, "\n--- RX FIFO (shared) ---\n");
    {
        u32 rxfifosiz = vhub_readl(vhub->regs, DWC3_GRXFIFOSIZ(0));
        u32 rx_depth = rxfifosiz & 0xffff;

        vhub_writel(vhub->regs, DWC3_GDBGFIFOSPACE,
                    DWC3_GDBGFIFOSPACE_NUM(0) |
                    DWC3_GDBGFIFOSPACE_TYPE(1));
        u32 rx_space = DWC3_GDBGFIFOSPACE_SPACE(
                       vhub_readl(vhub->regs, DWC3_GDBGFIFOSPACE));

        seq_printf(s, "  Space=%u/%u words  (%u/%u bytes)%s\n",
                   rx_space, rx_depth,
                   rx_space * word_bytes, rx_depth * word_bytes,
                   rx_space == rx_depth ? " [empty]" : "");
    }

    return 0;
}
DEFINE_SHOW_ATTRIBUTE(fifo);

/* ===================================================================
 * vhub-level: ports — all ports status at a glance
 * =================================================================== */

static int ports_show(struct seq_file *s, void *unused)
{
    struct ls_vhub *vhub = s->private;

    seq_puts(s, "Port  Status    Change    Addr  Enabled  GadgetState  Driver\n");
    seq_puts(s, "----  ------    ------    ----  -------  -----------  ------\n");

    for (int i = 0; i < vhub->ports_num; i++) {
        struct ls_pcie_vhub_port *p = &vhub->ports[i];
        struct ls_vhub_dev *dev = &p->dev;

        seq_printf(s, "  %-2d  0x%04x    0x%04x    %-4d  %-7d  %-11d  %s\n",
                   i, p->portsc.status, p->portsc.change,
                   dev->address, dev->enabled,
                   dev->gadget ? dev->gadget->state : -1,
                   dev->driver ? dev->driver->driver.name : "(none)");
    }
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(ports);

/* ===================================================================
 * vhub-level: hub ep0 — both common_ep0 (addr=0) and addressed ep0
 * =================================================================== */

static int hub_ep0_show(struct seq_file *s, void *unused)
{
    struct ls_vhub *vhub = s->private;

    ep0_ctrl_show(s, &vhub->common_ep0, "common_ep0 (addr=0)");
    ep0_trb_dump(s, &vhub->common_ep0);

    ep0_ctrl_show(s, &vhub->ep0, "hub_ep0    (addr=%d)", vhub->address);
    ep0_trb_dump(s, &vhub->ep0);
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(hub_ep0);

/* ===================================================================
 * vhub-level: hub ep1 — interrupt endpoint
 * =================================================================== */

static int hub_ep1_show(struct seq_file *s, void *unused)
{
    struct ls_vhub *vhub = s->private;

    if (!vhub->epn.ep) {
        seq_puts(s, "ep1: not allocated\n");
        return 0;
    }

    ep_show_one(s, "EP1", vhub->epn.ep);
    seq_printf(s, "  buf=%px  buf_dma=%pad  buf_len=%u\n",
               vhub->epn.buf, &vhub->epn.buf_dma, vhub->epn.buf_len);
    if (vhub->epn.ep->trb) {
        seq_puts(s, "  TRBs:\n");
        trb_show_one(s, 0, &vhub->epn.ep->trb->trb[0]);
        trb_show_one(s, 1, &vhub->epn.ep->trb->trb[1]);
    }
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(hub_ep1);

/* ===================================================================
 * dev-<N>/state — per-device state
 * =================================================================== */

static int dev_state_show(struct seq_file *s, void *unused)
{
    struct ls_vhub_dev *dev = s->private;

    seq_printf(s, "Port:        %d\n", dev->port_index);
    seq_printf(s, "Address:     %d\n", dev->address);
    seq_printf(s, "Enabled:     %d\n", dev->enabled);
    seq_printf(s, "Delayed:     %d\n", dev->delayed_status);
    seq_printf(s, "Driver:      %s\n",
               dev->driver ? dev->driver->driver.name : "(none)");
    seq_printf(s, "GadgetState: %d\n",
               dev->gadget ? dev->gadget->state : -1);
    seq_printf(s, "common_ep0:  %s\n",
               dev->common_ep0 == &dev->vhub->common_ep0 ? "shared" : "own");
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(dev_state);

/* ===================================================================
 * dev-<N>/ep0 — per-device EP0
 * =================================================================== */

static int dev_ep0_show(struct seq_file *s, void *unused)
{
    struct ls_vhub_dev *dev = s->private;
    ep0_ctrl_show(s, &dev->ep0, "dev_ep0");
    ep0_trb_dump(s, &dev->ep0);
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(dev_ep0);

/* ===================================================================
 * dev-<N>/eps — per-device endpoint list
 * =================================================================== */

static int dev_eps_show(struct seq_file *s, void *unused)
{
    struct ls_vhub_dev *dev = s->private;
    struct ls_vhub_ep *ep;

    seq_printf(s, "%-8s %-4s %-3s %-5s %-5s %-6s %-3s  %-6s  %s\n",
               "name", "num", "en", "stall", "wedge", "active", "res",
               "dir", "type");

    list_for_each_entry(ep, &dev->ep_list, dev_node) {
        const char *type;

        if (!ep->endpoint.desc) {
            type = "-";
        } else {
            switch (usb_endpoint_type(ep->endpoint.desc)) {
            case USB_ENDPOINT_XFER_CONTROL: type = "CTRL"; break;
            case USB_ENDPOINT_XFER_ISOC:    type = "ISOC"; break;
            case USB_ENDPOINT_XFER_BULK:    type = "BULK"; break;
            case USB_ENDPOINT_XFER_INT:     type = "INT";  break;
            default:                        type = "?";    break;
            }
        }

        seq_printf(s, "%-8s %-4d %-3d %-5d %-5d %-6d %-3d  %-6s  %s\n",
                   ep->name, ep->number, ep->enabled, ep->stall, ep->wedge,
                   ep->active, ep->resource,
                   ep->endpoint.caps.dir_in ? "IN" : "OUT", type);
    }

    if (list_empty(&dev->ep_list))
        seq_puts(s, "  (no endpoints allocated)\n");

    return 0;
}
DEFINE_SHOW_ATTRIBUTE(dev_eps);

/* ===================================================================
 * vhub-level: link_state — current USB link state (DSTS)
 * =================================================================== */

static const char *link_state_str(u8 s)
{
    switch (s) {
    case DWC3_LINK_STATE_U0:     return "U0 (On)";
    case DWC3_LINK_STATE_U1:     return "U1";
    case DWC3_LINK_STATE_U2:     return "U2 (Sleep)";
    case DWC3_LINK_STATE_U3:     return "U3 (Suspend)";
    case DWC3_LINK_STATE_RESET:  return "RESET";
    case DWC3_LINK_STATE_RESUME: return "RESUME";
    default:                     return "UNKNOWN";
    }
}

static int link_state_show(struct seq_file *s, void *unused)
{
    struct ls_vhub *vhub = s->private;
    u32 dsts = vhub_readl(vhub->regs, DWC3_DSTS);
    u8 ls = (dsts & DWC3_DSTS_USBLNKST_MASK) >> DWC3_DSTS_USBLNKST_SHIFT;

    seq_printf(s, "DSTS=0x%08x  link_state=%s\n", dsts, link_state_str(ls));
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(link_state);

/* ===================================================================
 * dev-<N>/trbs — dump TRB pairs for all endpoints of a device
 * =================================================================== */

static int dev_trbs_show(struct seq_file *s, void *unused)
{
    struct ls_vhub_dev *dev = s->private;
    struct ls_vhub_ep *ep;

    list_for_each_entry(ep, &dev->ep_list, dev_node) {
        seq_printf(s, "%s (num=%d):\n", ep->name, ep->number);
        if (!ep->trb) {
            seq_puts(s, "  (no TRB buffer)\n\n");
            continue;
        }
        trb_show_one(s, 0, &ep->trb->trb[0]);
        trb_show_one(s, 1, &ep->trb->trb[1]);
        seq_putc(s, '\n');
    }

    if (list_empty(&dev->ep_list))
        seq_puts(s, "(no endpoints allocated)\n");

    return 0;
}
DEFINE_SHOW_ATTRIBUTE(dev_trbs);

/* ===================================================================
 * vhub-level: ep_map — EP map table (dev_addr:ep_num → phys_ep mapping)
 *
 * Disabled: map registers are write-only in current hardware revision.
 * =================================================================== */
#if 0
static int ep_map_show(struct seq_file *s, void *unused)
{
    struct ls_vhub *vhub = s->private;
    int mirror = VHUB_EP_MAP_REG_COUNT >> 1;

    seq_puts(s, "phys_ep  dir  map_idx  dev_addr:ep_num  raw\n");
    seq_puts(s, "-------  ---  -------  ---------------  --------\n");

    for (int i = 0; i < VHUB_EP_MAP_REG_COUNT; i++) {
        u32 val = vhub->ep_pool.map_table[i].value;
        int dir = i >= mirror;
        int phys_ep = dir ? ((i - mirror) * 2 + 3) : (i * 2 + 2);

        seq_printf(s, "  %-4d    %-3s    %-4d     %d:%-13d  0x%08x\n",
                   phys_ep, dir ? "IN" : "OUT", i,
                   val & 0x7f, (val >> 7) & 0xf, val);
    }
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(ep_map);
#endif

/* ===================================================================
 * vhub-level: ep_pool — physical endpoint pool usage
 * =================================================================== */

static int ep_pool_show(struct seq_file *s, void *unused)
{
    struct ls_vhub *vhub = s->private;

    seq_puts(s, "phys  used  ctrl  common  name         dev\n");
    seq_puts(s, "----  ----  ----  ------  ----         ---\n");

    for (int i = 0; i < vhub->ep_pool.ep_num; i++) {
        struct ls_vhub_ep *ep = &vhub->ep_pool.eps[i];
        char owner[32];

        if (!ep->used)
            snprintf(owner, sizeof(owner), "-");
        else if (!ep->dev)
            snprintf(owner, sizeof(owner), "hub");
        else if (ep->dev->driver)
            snprintf(owner, sizeof(owner), "%s", ep->dev->driver->driver.name);
        else
            snprintf(owner, sizeof(owner), "dev-%d", ep->dev->port_index);

        seq_printf(s, "  %-3d  %-4d  %-4d  %-6d  %-12s  %s\n",
                   ep->number, ep->used, ep->ctrl, ep->common,
                   ep->used ? ep->name : "(free)", owner);
    }
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(ep_pool);

/* ===================================================================
 * Public API — called from vhub.c / dev.c
 * =================================================================== */

int ls_vhub_debugfs_init(struct ls_vhub *vhub)
{
    struct dentry *root;

    root = debugfs_create_dir(dev_name(&vhub->pdev->dev), usb_debug_root);
    vhub->debugfs_root = root;

    /* regdump — uses debugfs_regset32 */
    vhub->regset = devm_kzalloc(&vhub->pdev->dev, sizeof(*vhub->regset),
                                GFP_KERNEL);
    if (vhub->regset) {
        vhub->regset->regs  = dwc3_regs;
        vhub->regset->nregs = ARRAY_SIZE(dwc3_regs);
        vhub->regset->base  = vhub->regs - DWC3_GLOBALS_REGS_START;
        vhub->regset->dev   = &vhub->pdev->dev;
        debugfs_create_regset32("regdump", 0444, root, vhub->regset);
    }

    /* vhub-level info files */
    debugfs_create_file("events",     0444, root, vhub, &events_fops);
    debugfs_create_file("fifo",       0444, root, vhub, &fifo_fops);
    debugfs_create_file("link_state", 0444, root, vhub, &link_state_fops);
    /* ep_map disabled — map registers are write-only in current silicon */
    /* debugfs_create_file("ep_map",     0444, root, vhub, &ep_map_fops); */
    debugfs_create_file("ep_pool",    0444, root, vhub, &ep_pool_fops);
    debugfs_create_file("ports",      0444, root, vhub, &ports_fops);

    /* hub's own endpoints */
    debugfs_create_file("ep0", 0444, root, vhub, &hub_ep0_fops);
    debugfs_create_file("ep1", 0444, root, vhub, &hub_ep1_fops);

    return 0;
}

void ls_vhub_debugfs_exit(struct ls_vhub *vhub)
{
    if (!IS_ERR_OR_NULL(vhub->debugfs_root)) {
        debugfs_remove_recursive(vhub->debugfs_root);
        vhub->debugfs_root = NULL;
    }
}

int ls_vhub_dev_debugfs_init(struct ls_vhub_dev *dev)
{
    struct dentry *d;
    char name[16];

    snprintf(name, sizeof(name), "dev-%d", dev->port_index);
    d = debugfs_create_dir(name, dev->vhub->debugfs_root);
    if (IS_ERR(d))
        return PTR_ERR(d);
    dev->debugfs_dir = d;

    debugfs_create_file("state", 0444, d, dev, &dev_state_fops);
    debugfs_create_file("ep0",   0444, d, dev, &dev_ep0_fops);
    debugfs_create_file("eps",   0444, d, dev, &dev_eps_fops);
    debugfs_create_file("trbs",  0444, d, dev, &dev_trbs_fops);

    return 0;
}

void ls_vhub_dev_debugfs_exit(struct ls_vhub_dev *dev)
{
    if (!IS_ERR_OR_NULL(dev->debugfs_dir)) {
        debugfs_remove_recursive(dev->debugfs_dir);
        dev->debugfs_dir = NULL;
    }
}

#endif // CONFIG_DEBUG_FS
