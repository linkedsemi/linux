#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include "vhub.h"
#include "debugfs.h"
#include "trace.h"

static struct usb_device_descriptor device_desc = {
    .bLength = sizeof(device_desc),
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB =  cpu_to_le16(0x0200),
    .bDeviceClass = 0x9, // hub class
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 1, // Hi-speed hub with single TT
    .bMaxPacketSize0 = 0x40, // ep0 max size
    .idVendor = 0x1234,
    .idProduct = 0x66, 
    .bcdDevice = 0x0100,
    .iManufacturer = 0, // string desc index
    .iProduct = 1, // string desc index
    .iSerialNumber = 0, // string desc index
    .bNumConfigurations = 0x01
};

struct usb_qualifier_descriptor vhub_dev_qualifier = {
    .bLength = 10,
    .bDescriptorType = 6,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .bNumConfigurations = 1,
    .bRESERVED = 0
};

static struct usb_hub_cfg_desc
{
    struct usb_config_descriptor  cfg;
    struct usb_interface_descriptor interface;
    struct usb_endpoint_descriptor endpoint;
} __packed hub_cfg_desc = {
    .cfg = {
        .bLength = 9,
        .bDescriptorType = USB_DT_CONFIG,
        .wTotalLength = 9 + 9 + 7,
        .bNumInterfaces = 1,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
        .bMaxPower = 0,
    },
    .interface = {
        .bLength = 9,
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 1,
        .bInterfaceClass = 0x9,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 0,
    },
    .endpoint = {
        .bLength = 7,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x81,
        .bmAttributes = 0x3,
        .wMaxPacketSize = 1,
        .bInterval = 0x5 // 2^4 * 125us = 2ms
    }
};

static struct usb_hub_descriptor hub_desc = {
    .bDescLength = 9,
    .bDescriptorType = USB_DT_HUB,
    .bNbrPorts = 5,
    .wHubCharacteristics = cpu_to_le16(HUB_CHAR_NO_LPSM),
    .bPwrOn2PwrGood = 0x0a,
    .bHubContrCurrent = 0,
    .u.hs.DeviceRemovable[0] = 0xE0,
    .u.hs.DeviceRemovable[1] = 0xff
};

static u8 vhub_str0[] = { 4, USB_DT_STRING, 0x09, 0x04 };
static u8 vhub_str1[] = { 24, USB_DT_STRING,
    0x76, 0x00, 0x69, 0x00,
    0x72, 0x00, 0x74, 0x00,
    0x75, 0x00, 0x61, 0x00,
    0x6C, 0x00,
    0x20, 0x00,
    0x68, 0x00, 0x75, 0x00,
    0x62, 0x00
};

static int dwc3_dep_command(struct ls_vhub *vhub, u8 phy_ep_idx, u32 *cmd,
                            const struct dep_command_param *param)
{
    u32 retries = 5000;
    u32 value = 0;
    u32 cmd_val = *cmd;

    vhub_writel(vhub->regs, DWC3_DEPCMDPAR0(phy_ep_idx), param->param0);
    vhub_writel(vhub->regs, DWC3_DEPCMDPAR1(phy_ep_idx), param->param1);
    vhub_writel(vhub->regs, DWC3_DEPCMDPAR2(phy_ep_idx), param->param2);
    vhub_writel(vhub->regs, DWC3_DEPCMD(phy_ep_idx), cmd_val);

    do {
        value = vhub_readl(vhub->regs, DWC3_DEPCMD(phy_ep_idx));
        if (!(value & DWC3_DEPCMD_CMDACT)) {
            switch (DWC3_DEPCMD_STATUS(value))
            {
            case 0:
                break;
            case DEPEVT_TRANSFER_NO_RESOURCE:
                dev_err(&vhub->pdev->dev, "DEPEVT_TRANSFER_NO_RESOURCE\n");
                break;
            case DEPEVT_TRANSFER_BUS_EXPIRY:
                dev_err(&vhub->pdev->dev, "DEPEVT_TRANSFER_BUS_EXPIRY\n");
                break;
            default:
                dev_err(&vhub->pdev->dev, "UNKNOWN cmd status\n");
            }
            break;
        }
        udelay(1);
    } while (--retries);
    *cmd = value;

    trace_ls_vhub_dep_cmd(vhub, phy_ep_idx, cmd_val, param, value, retries ? 0 : -ETIMEDOUT);

    return retries ? 0 : -ETIMEDOUT;
}

static int dwc3_dep_config(struct ls_vhub *vhub, u8 phy_ep_idx, const struct dep_command_param *param)
{
    u32 cmd = DWC3_DEPCMD_SETEPCONFIG | DWC3_DEPCMD_CMDACT;

    return dwc3_dep_command(vhub, phy_ep_idx, &cmd, param);
}

static int dwc3_ep_config(struct ls_vhub *vhub, u8 ep_number, const struct usb_endpoint_descriptor *desc, u8 config_action)
{
    u8 ep_dir = ep_number & 0x1;
    int ret;
    struct dep_command_param cmd_param;

    u16 mps = desc->wMaxPacketSize;
    u8 ep_type = usb_endpoint_type(desc);

    if (ep_type != USB_ENDPOINT_XFER_CONTROL)
    {
        cmd_param.param0 = DWC3_DEPCFG_EP_TYPE(ep_type) |
                           DWC3_DEPCFG_MAX_PACKET_SIZE(mps) |
                           DWC3_DEPCFG_ACTION(config_action);
        if (ep_dir)
            cmd_param.param0 |= DWC3_DEPCFG_FIFO_NUMBER(ep_number >> 1);

        cmd_param.param1 = DWC3_DEPCFG_XFER_COMPLETE_EN |
                           DWC3_DEPCFG_XFER_IN_PROGRESS_EN |
                           DWC3_DEPCFG_STREAM_EVENT_EN |
                           DWC3_DEPCFG_EP_NUMBER(ep_number) |
                           DWC3_DEPCFG_BINTERVAL_M1(min_t(u8, desc->bInterval - 1, 13));
        cmd_param.param2 = 0;
    }
    else
    {
        // 控制端点 IN/OUT 的 FIFONum 需一致，这里统一设为 ep_number >> 1
        cmd_param.param0 = DWC3_DEPCFG_EP_TYPE(USB_ENDPOINT_XFER_CONTROL) |
                           DWC3_DEPCFG_MAX_PACKET_SIZE(mps) |
                           DWC3_DEPCFG_ACTION(config_action) |
                           DWC3_DEPCFG_FIFO_NUMBER(ep_number >> 1);

        cmd_param.param1 = DWC3_DEPCFG_XFER_COMPLETE_EN |
                           DWC3_DEPCFG_XFER_IN_PROGRESS_EN |
                           DWC3_DEPCFG_XFER_NOT_READY_EN |
                           DWC3_DEPCFG_STREAM_EVENT_EN |
                           DWC3_DEPCFG_EP_NUMBER(ep_number);
        cmd_param.param2 = 0;
    }

    ret = dwc3_dep_config(vhub, ep_number, &cmd_param);
    vhub_writel(vhub->regs, DWC3_DALEPENA, vhub_readl(vhub->regs, DWC3_DALEPENA) | (1 << ep_number));

    return ret;
}

static int dwc3_ep_set_xfer_resource(struct ls_vhub *vhub, u8 phy_ep_idx)
{
    u32 cmd = DWC3_DEPCMD_SETTRANSFRESOURCE | DWC3_DEPCMD_CMDACT;
    struct dep_command_param param = {
        .param0 = DWC3_DEPXFERCFG_NUM_XFER_RES(1),
    };

    return dwc3_dep_command(vhub, phy_ep_idx, &cmd, &param);
}

static int dwc3_dep_start_config(struct ls_vhub *vhub, u8 XferRscIdx)
{
    int ret;
    u32 cmd = DWC3_DEPCMD_DEPSTARTCFG | DWC3_DEPCMD_CMDACT | DWC3_DEPCMD_PARAM(XferRscIdx);
    struct dep_command_param param = {0};

    ret = dwc3_dep_command(vhub, 0, &cmd, &param);
    if (ret)
        return ret;

    for (int i = XferRscIdx; i < vhub->ep_pool.ep_num; i++) {
        vhub->ep_pool.eps[i].resource = 0;
    }

    return 0;
}

void ls_vhub_ep_prepare_trb(struct dwc3_trb *trb, dma_addr_t buf_ptr, u32 size,
                         u32 type, bool chain)
{
    trb->bptrl = lower_32_bits(buf_ptr);
    trb->bptrh = upper_32_bits(buf_ptr);
    trb->size = size;
    trb->ctrl = type;
    trb->ctrl |= (DWC3_TRB_CTRL_HWO | DWC3_TRB_CTRL_ISP_IMI);
    if (chain)
        trb->ctrl |= DWC3_TRB_CTRL_CHN;
    else
        trb->ctrl |= (DWC3_TRB_CTRL_IOC | DWC3_TRB_CTRL_LST);
}

static int ls_vhub_ep_set_stall(struct ls_vhub_ep *ep)
{
    u32 cmd = DWC3_DEPCMD_SETSTALL | DWC3_DEPCMD_CMDACT;
    struct dep_command_param param = {0};

    return dwc3_dep_command(ep->vhub, ep->number, &cmd, &param);
}

int ls_vhub_ep_clear_stall(struct ls_vhub_ep *ep)
{
    u32 cmd = DWC3_DEPCMD_CLEARSTALL | DWC3_DEPCMD_CMDACT;
    struct dep_command_param param = {0};

    return dwc3_dep_command(ep->vhub, ep->number, &cmd, &param);
}

/*
             8字节宽度
start   ->  |--------|
            |--------|
            |--------|
            |--------|
            |--------|
            |--------|
            |--------|
            |--------|
            |--------|
            |--------|
            |--------|
            |--------|
    什么策略去分配？
*/
/*
static int ls_vhub_txfifo_ram_depth(struct ls_vhub *vhub)
{
    int ram_depth;
    bool single_port;

    single_port = DWC3_HWPARAMS1_SPRAM_TYPE(vhub->hwparams.hwparams1);

    ram_depth = single_port ?
        DWC3_HWPARAMS6_RAM0_DEPTH(vhub->hwparams.hwparams6) :
        DWC3_HWPARAMS7_RAM1_DEPTH(vhub->hwparams.hwparams7);

    if (single_port) {
        u32 reg = vhub_readl(vhub->regs, DWC3_GTXFIFOSIZ(0));
        u32 fifo_base = DWC3_GTXFIFOSIZ_TXFSTADDR(reg);

        ram_depth -= (fifo_base >> 16);
    }

    return ram_depth;
}
*/
/*
static int ls_vhub_ep_txfifo_resize(struct ls_vhub_ep *ep)
{
    if (ep->ctrl) {
        return 0;
    }

    struct ls_vhub *vhub = ep->vhub;
    u32 fifo_info = vhub_readl(vhub->regs, DWC3_GTXFIFOSIZ(ep->number >> 1));
    u16 depth = DWC3_GTXFIFOSIZ_DEPTH(fifo_info);
    u32 start_addr = DWC3_GTXFIFOSIZ_TXFSTADDR(fifo_info);

    u32 mdwidth = DWC3_HWPARAMS0_MDWIDTH(vhub->hwparams.hwparams0);
    u32 word_bytes = mdwidth / 8;
    u32 words_per_512 = 512 / word_bytes;

    u16 new_depth = ALIGN_DOWN(depth, words_per_512);

    if (WARN_ON(new_depth < words_per_512))
        return -EINVAL;

    vhub_writel(vhub->regs, DWC3_GTXFIFOSIZ(ep->number >> 1), start_addr | new_depth);

    return 0;
}
*/

int ls_vhub_ep_start_transfer(struct ls_vhub_ep *ep, dma_addr_t trb, u16 stream_id)
{
    int ret;
    u32 cmd = DWC3_DEPCMD_STARTTRANSFER | DWC3_DEPCMD_CMDACT |
                   DWC3_DEPCMD_PARAM(stream_id);
    struct dep_command_param param = {
        .param0 = upper_32_bits(trb),
        .param1 = lower_32_bits(trb),
    };

    ret = dwc3_dep_command(ep->vhub, ep->number, &cmd, &param);
    if (ret == 0) {
        ep->res_idx = DWC3_DEPCMD_GET_RSC_IDX(cmd);
        ep->active = 1;
    }

    return ret;
}

static int __ls_vhub_ep_enable(struct ls_vhub *vhub, struct ls_vhub_ep *ep, u8 action)
{
    int ret;
    const struct usb_endpoint_descriptor *desc = ep->endpoint.desc;

    dev_dbg(&vhub->pdev->dev, "Enabling endpoint %d, max packet size: %d, type = %d, bInterval = %d\n", ep->number, le16_to_cpu(desc->wMaxPacketSize), usb_endpoint_type(desc), desc->bInterval);
    /* TODO: fifo resize */
    ret = dwc3_ep_config(vhub, ep->number, desc, action);

    if (!ret && !ep->resource) {
        ret = dwc3_ep_set_xfer_resource(vhub, ep->number);
        ep->resource = 1;
    }

    return ret;
}

static void ls_vhub_reset_ep0_pair(struct ls_vhub_ctrl_ep *ep0)
{
    struct ls_vhub_ep *out = ep0->out;
    struct ls_vhub_ep *in = ep0->in;

    ls_vhub_ep_stop_transfer(out);
    ls_vhub_ep_stop_transfer(in);

    out->stall = 0;
    in->stall = 0;
    out->wedge = 0;
    in->wedge = 0;
    ep0->state = EP0_SETUP_PHASE;
    ep0->three_stage_setup = false;
    ep0->expect_in = false;
}

static void ls_vhub_reset_common_ep0(struct ls_vhub *vhub)
{
    struct ls_vhub_ep *out = vhub->common_ep0.out;

    if (vhub->common_ep0.state != EP0_SETUP_PHASE) {
        ls_vhub_reset_ep0_pair(&vhub->common_ep0);
        /* restart common ep0 */
        ls_vhub_ep0_stall_and_restart(out);
        /* remove internal requests */
        list_del_init(&vhub->common_ep0.req.node);
    }
}

static void ls_vhub_reset_addressed_eps(struct ls_vhub *vhub)
{
    if (vhub->ep0.out) {
        ls_vhub_reset_ep0_pair(&vhub->ep0);
        ls_vhub_ep_disable(vhub->ep0.out);
        ls_vhub_ep_disable(vhub->ep0.in);
        ls_vhub_ep_unmap(vhub->ep0.out);
        ls_vhub_ep_unmap(vhub->ep0.in);
        /* remove internal requests */
        list_del_init(&vhub->ep0.req.node);
    }

    if (vhub->epn.ep) {
        ls_vhub_ep_stop_transfer(vhub->epn.ep);
        ls_vhub_ep_disable(vhub->epn.ep);
        ls_vhub_ep_unmap(vhub->epn.ep);
        /* remove internal requests */
        list_del_init(&vhub->epn.req.node);
        vhub->epn.notify_pending = false;
    }
}

static void ls_vhub_reset_port_state(struct ls_pcie_vhub_port *port)
{
    /* remain connect and power status */
    port->portsc.status &= USB_PORT_STAT_CONNECTION | USB_PORT_STAT_POWER;
    port->portsc.change = 0;
}

static void ls_vhub_bus_reset_ports(struct ls_vhub *vhub)
{
    for (int i = 0; i < vhub->ports_num; i++) {
        ls_vhub_reset_port_state(&vhub->ports[i]);
        ls_vhub_dev_bus_reset(&vhub->ports[i].dev);
    }
}

static void ls_vhub_disconnect_ports(struct ls_vhub *vhub)
{
    for (int i = 0; i < vhub->ports_num; i++) {
        ls_vhub_reset_port_state(&vhub->ports[i]);
        ls_vhub_dev_disconnect(&vhub->ports[i].dev);
    }
}

static void ls_vhub_suspend_ports(struct ls_vhub *vhub)
{
    if (vhub->suspended)
        return;

    vhub->suspended = true;

    for (int i = 0; i < vhub->ports_num; i++) {
        if (vhub->ports[i].portsc.status & USB_PORT_STAT_SUSPEND)
            continue;
        ls_vhub_dev_suspend(&vhub->ports[i].dev);
    }
}

static void ls_vhub_resume_ports(struct ls_vhub *vhub)
{
    if (!vhub->suspended)
        return;

    vhub->suspended = false;

    for (int i = 0; i < vhub->ports_num; i++) {
        if (vhub->ports[i].portsc.status & USB_PORT_STAT_SUSPEND)
            continue;
        ls_vhub_dev_resume(&vhub->ports[i].dev);
    }
}

static void dwc3_dev_event_reset(struct ls_vhub *vhub)
{
    u32 reg;

    reg = vhub_readl(vhub->regs, DWC3_DCTL);
    reg &= ~DWC3_DCTL_TSTCTRL_MASK;
    reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;
    vhub_writel(vhub->regs, DWC3_DCTL, reg);

    reg = vhub_readl(vhub->regs, DWC3_DCFG);
    reg &= ~DWC3_DCFG_DEVADDR_MASK;
    vhub_writel(vhub->regs, DWC3_DCFG, reg);

    vhub->address = 0;
    vhub->addr0_dev = NULL;
    vhub->suspended = false;

    /* reset ports */
    ls_vhub_bus_reset_ports(vhub);
    /* disable vhub endpoints */
    ls_vhub_reset_addressed_eps(vhub);
    /* reset common ep0 */
    ls_vhub_reset_common_ep0(vhub);
}

static void dwc3_dev_event_connect_done(struct ls_vhub *vhub)
{
    u32 reg = vhub_readl(vhub->regs, DWC3_DSTS);
    u8 speed = reg & DWC3_DSTS_CONNECTSPD;

    if (speed != DWC3_DSTS_HIGHSPEED)
        vhub_dbg_vhub(vhub, "unexpected USB2 speed: 0x%x\n", speed);
    else
        vhub_dbg_vhub(vhub, "connected at high-speed\n");

    vhub->common_ep0.out->endpoint.maxpacket = 64;
    vhub->common_ep0.in->endpoint.maxpacket = 64;
}

static void dwc3_dev_event_link_status(struct ls_vhub *vhub, const union evt_buf *evt)
{
    u8 link_state = evt->devt.evt_info & DWC3_LINK_STATE_MASK;

    switch (link_state) {
    case DWC3_LINK_STATE_U0:
        vhub_dbg_vhub(vhub, "link state: U0/ON\n");
        ls_vhub_resume_ports(vhub);
        break;
    case DWC3_LINK_STATE_U3:
        vhub_dbg_vhub(vhub, "link state: U3/SUSPEND\n");
        ls_vhub_suspend_ports(vhub);
        break;
    case DWC3_LINK_STATE_RESUME:
        vhub_dbg_vhub(vhub, "link state: RESUME\n");
        ls_vhub_resume_ports(vhub);
        break;
    case DWC3_LINK_STATE_RESET:
        vhub_dbg_vhub(vhub, "link state: RESET\n");
        break;
    case DWC3_LINK_STATE_U1:    /* USB3 only, for USB2-only controller this won't appear */
        vhub_dbg_vhub(vhub, "link state: U1\n");
        break;
    case DWC3_LINK_STATE_U2:    /* USB3 only, for USB2-only controller this won't appear */
        vhub_dbg_vhub(vhub, "link state: U2/SLEEP\n");
        break;
    default:
        vhub_dbg_vhub(vhub, "link state: 0x%x\n", link_state);
        break;
    }
}

static void dwc3_dev_event_disconnect(struct ls_vhub *vhub)
{
    vhub->address = 0;
    vhub->addr0_dev = NULL;
    vhub->suspended = false;

    ls_vhub_disconnect_ports(vhub);
    ls_vhub_reset_addressed_eps(vhub);
    ls_vhub_reset_common_ep0(vhub);

    vhub_dbg_vhub(vhub, "device event: DISCONNECT\n");
}

static void dwc3_dev_event(struct ls_vhub *vhub, const union evt_buf *evt)
{
    switch (evt->devt.dev_evt_type) {
    case DWC3_DEVICE_EVENT_DISCONNECT:
        dwc3_dev_event_disconnect(vhub);
        break;
    case DWC3_DEVICE_EVENT_RESET:
        dwc3_dev_event_reset(vhub);
        break;
    case DWC3_DEVICE_EVENT_CONNECT_DONE:
        dwc3_dev_event_connect_done(vhub);
        break;
    case DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE:
        dwc3_dev_event_link_status(vhub, evt);
        break;
    case DWC3_DEVICE_EVENT_WAKEUP:
        ls_vhub_resume_ports(vhub);
        break;
    case DWC3_DEVICE_EVENT_HIBER_REQ:
        vhub_dbg_vhub(vhub, "device event: hibernation request\n");
        break;
    case DWC3_DEVICE_EVENT_SUSPEND:
        vhub_dbg_vhub(vhub, "device event: suspend\n");
        if ((evt->devt.evt_info & DWC3_LINK_STATE_MASK) == DWC3_LINK_STATE_U3)
            ls_vhub_suspend_ports(vhub);
        break;
    case DWC3_DEVICE_EVENT_SOF:
        break;
    case DWC3_DEVICE_EVENT_ERRATIC_ERROR:
        vhub_dbg_vhub(vhub, "device event: erratic error\n");
        break;
    case DWC3_DEVICE_EVENT_CMD_CMPL:
        vhub_dbg_vhub(vhub, "device event: command complete\n");
        break;
    case DWC3_DEVICE_EVENT_OVERFLOW:
        vhub_dbg_vhub(vhub, "device event: overflow\n");
        break;
    default:
        vhub_dbg_vhub(vhub, "device event: unknown type %u info 0x%x\n",
                       evt->devt.dev_evt_type, evt->devt.evt_info);
        break;
    }
}

static void ls_vhub_ep0_req_complete(struct usb_ep *ep, struct usb_request *req)
{
    /* internel req, do nothing */
}

static void ls_vhub_notify_update(struct ls_vhub *vhub);

static void ls_vhub_ep1_req_complete(struct usb_ep *ep, struct usb_request *req)
{
    struct ls_vhub_ep *ep1 = container_of(ep, struct ls_vhub_ep, endpoint);
    struct ls_vhub *vhub = ep1->vhub;
    unsigned long flags;

    vhub_dbg_ep(ep1, "ep1 req complete: status = %d, actual = %d\n", req->status, req->actual);
    trace_ls_vhub_ep1_complete(vhub, req->status, req->actual,
                               vhub->epn.notify_pending);

    if (!vhub->epn.notify_pending)
        return;

    ls_vhub_lock(vhub, &flags);
    ls_vhub_notify_update(vhub);
    ls_vhub_unlock(vhub, &flags);
}

static int ls_vhub_ep0_replay(struct ls_vhub *vhub, void *data, size_t len)
{
    struct ls_vhub_ctrl_ep *ep0;

    /* copy ep data buffer */
    if (vhub->address)
    {
        memcpy(vhub->ep0.data, data, len);
        vhub->ep0.req.request.buf = vhub->ep0.data;
        vhub->ep0.req.request.dma = vhub->ep0.data_dma;
        vhub->ep0.req.request.length = len;
        vhub->ep0.req.request.complete = ls_vhub_ep0_req_complete;
        ep0 = &vhub->ep0;
    }
    else
    {
        memcpy(vhub->common_ep0.data, data, len);
        vhub->common_ep0.req.request.buf = vhub->common_ep0.data;
        vhub->common_ep0.req.request.dma = vhub->common_ep0.data_dma;
        vhub->common_ep0.req.request.length = len;
        vhub->common_ep0.req.request.complete = ls_vhub_ep0_req_complete;
        ep0 = &vhub->common_ep0;
    }
    /* internal ep0 */
    ls_vhub_ep0_queue_req(ep0->out, &ep0->req);

    return 0;
}

static int ls_vhub_dev_status(struct ls_vhub *vhub, u16 wLength)
{
    u16 len = MIN(wLength, (u16)2);
    u16 status = cpu_to_le16(1 << USB_DEVICE_SELF_POWERED);

    ls_vhub_ep0_replay(vhub, &status, len);

    return 0;
}

static int ls_vhub_feature(struct ls_vhub *vhub, u16 wIndex, u16 wValue, bool is_set)
{
    vhub_dbg_vhub(vhub, "%s hub feature\n", is_set ? "set" : "clr");
    return 0;
}

static int ls_vhub_get_desc(struct ls_vhub *vhub, u8 type, u16 wLength)
{
    void *desc = NULL;
    u32 desc_len = 0;

    switch (type) {
    case USB_DT_DEVICE:
        vhub_dbg_vhub(vhub, "GET_DESCRIPTOR: device\n");
        desc = &device_desc;
        desc_len = sizeof(device_desc);
        break;
    case USB_DT_CONFIG:
        vhub_dbg_vhub(vhub, "GET_DESCRIPTOR: config\n");
        desc = &hub_cfg_desc;
        desc_len = sizeof(hub_cfg_desc);
        break;
    case USB_DT_OTHER_SPEED_CONFIG:
        return -1;
        break;
    case USB_DT_HUB:
        vhub_dbg_vhub(vhub, "GET_DESCRIPTOR: hub\n");
        desc = &hub_desc;
        desc_len = hub_desc.bDescLength;
        break;
    case USB_DT_DEVICE_QUALIFIER:
        vhub_dbg_vhub(vhub, "GET_DESCRIPTOR: device qualifier\n");
        desc = &vhub_dev_qualifier;
        desc_len = sizeof (vhub_dev_qualifier);
        break;
    default:
        vhub_dbg_vhub(vhub, "GET_DESCRIPTOR: unknown type %u\n", type);
        desc = NULL;
        desc_len = 0;
        break;
    }

    desc_len = MIN(wLength, desc_len);

    /* Start transfer on correct EP0 IN endpoint. */
    ls_vhub_ep0_replay(vhub, desc, desc_len);

    return 0;
}

static int ls_vhub_get_string(struct ls_vhub *vhub, u8 index, u16 wLength)
{
    u8 *str_desc = NULL;
    u16 len = 0;

    if (index == 0 || index == 1) {
        str_desc = index ? vhub_str1 : vhub_str0;
        len = MIN(wLength, str_desc[0]);
        return ls_vhub_ep0_replay(vhub, str_desc, len);
    }

    return -EINVAL;
}

static int ls_vhub_port_status(struct ls_vhub *vhub, u8 port_index)
{
    u32 raw;
    if (!port_index || port_index > vhub->ports_num)
        return -EINVAL;

    struct ls_pcie_vhub_port *port = &vhub->ports[port_index - 1];
    raw = port->portsc.raw;

    return ls_vhub_ep0_replay(vhub, &raw, 4);
}

static int ls_vhub_set_address(struct ls_vhub *vhub, u16 address)
{
    vhub->address = address & 0x7f;

    dwc3_dep_start_config(vhub, 2);
    /* enable vhub self ep0 */
    ls_vhub_ep_enable(vhub->ep0.out);
    ls_vhub_ep_enable(vhub->ep0.in);
    ls_vhub_ep_map(vhub->ep0.out);
    ls_vhub_ep_map(vhub->ep0.in);

    dev_dbg(&vhub->pdev->dev, "ls_vhub_set_address %d, out %d, in %d\n", address, vhub->ep0.out->number, vhub->ep0.in->number);
    ls_vhub_ep_prepare_trb(&vhub->ep0.out->trb->trb[0], vhub->ep0.out->trb_dma, 8, DWC3_TRBCTL_CONTROL_SETUP, false);
    ls_vhub_ep_start_transfer(vhub->ep0.out, vhub->ep0.out->trb_dma, 0);

    return 0;
}

static int ls_vhub_set_config(struct ls_vhub *vhub, u16 config)
{
    dev_dbg(&vhub->pdev->dev, "set configuration = %u\n", config);

    if (config > 1)
        return -EINVAL;

    if (!config) {
        /* stop epn transfer */
        ls_vhub_ep_stop_transfer(vhub->epn.ep);
        ls_vhub_ep_disable(vhub->epn.ep);
        ls_vhub_ep_unmap(vhub->epn.ep);
        list_del_init(&vhub->epn.req.node);
        vhub->epn.notify_pending = false;
        return 0;
    }

    /* enable vhub self ep1 */
    ls_vhub_ep_enable(vhub->epn.ep);
    ls_vhub_ep_map(vhub->epn.ep);

    if (vhub->epn.notify_pending)
        ls_vhub_notify_update(vhub);

    return 0;
}

static int ls_vhub_std_req(struct ls_vhub *vhub, struct usb_ctrlrequest *setup)
{
    u8 type = setup->bRequestType;
    u8 req = setup->bRequest;
    u16 wValue  = le16_to_cpu(setup->wValue);
    u16 wIndex  = le16_to_cpu(setup->wIndex);
    u16 wLength = le16_to_cpu(setup->wLength);

    if ((type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
        switch (req) {
        case USB_REQ_SET_ADDRESS:
            return ls_vhub_set_address(vhub, wValue);
        case USB_REQ_GET_STATUS:
            return ls_vhub_dev_status(vhub, wLength);
        case USB_REQ_SET_FEATURE:
            return ls_vhub_feature(vhub, wIndex, wValue, 1);
        case USB_REQ_CLEAR_FEATURE:
            return ls_vhub_feature(vhub, wIndex, wValue, 0);
        case USB_REQ_GET_CONFIGURATION:
            {
                u8 cfg = vhub->epn.ep->enabled ? 1 : 0;
                u16 len = MIN(wLength, (u16)1);
                return ls_vhub_ep0_replay(vhub, &cfg, len);
            }
        case USB_REQ_SET_CONFIGURATION:
            return ls_vhub_set_config(vhub, wValue);
        case USB_REQ_GET_DESCRIPTOR:
            switch (wValue >> 8) {
            case USB_DT_DEVICE:
            case USB_DT_CONFIG:
            case USB_DT_DEVICE_QUALIFIER:
            case USB_DT_OTHER_SPEED_CONFIG:
                return ls_vhub_get_desc(vhub, wValue >> 8, wLength);
            case USB_DT_STRING:
                return ls_vhub_get_string(vhub, wValue & 0xff, wLength);
            }
            dev_dbg(&vhub->pdev->dev, "USB_REQ_GET_DESCRIPTOR type = %d\n", wValue >> 8);
            return -ENOTSUPP; // stall
        case USB_REQ_GET_INTERFACE:
            return -ENOTSUPP;
        case USB_REQ_SET_INTERFACE:
            return -ENOTSUPP;
        }
    }

    if ((type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
        switch (req) {
        case USB_REQ_GET_STATUS:
            return 0;
        }
    }

    if ((type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
        switch (req) {
        case USB_REQ_GET_STATUS:
            return 0;
        case USB_REQ_SET_FEATURE:
            vhub_dbg_vhub(vhub, "endpoint SET_FEATURE\n");
            return 0;
        case USB_REQ_CLEAR_FEATURE:
            vhub_dbg_vhub(vhub, "endpoint CLEAR_FEATURE\n");
            return 0;
        }
    }

    return 0;
}

enum ls_vhub_port_action {
    LS_VHUB_PORT_RESET_BEGIN,
    LS_VHUB_PORT_RESET_DONE_HS,
    LS_VHUB_PORT_ENABLE,
    LS_VHUB_PORT_DISABLE,
    LS_VHUB_PORT_SUSPEND,
    LS_VHUB_PORT_RESUME,
    LS_VHUB_PORT_ATTACH,
    LS_VHUB_PORT_DETACH,
};

static const char *ls_vhub_port_action_name(enum ls_vhub_port_action action)
{
    switch (action) {
    case LS_VHUB_PORT_RESET_BEGIN:  return "RESET_BEGIN";
    case LS_VHUB_PORT_RESET_DONE_HS: return "RESET_DONE_HS";
    case LS_VHUB_PORT_ENABLE:       return "ENABLE";
    case LS_VHUB_PORT_DISABLE:      return "DISABLE";
    case LS_VHUB_PORT_SUSPEND:      return "SUSPEND";
    case LS_VHUB_PORT_RESUME:       return "RESUME";
    case LS_VHUB_PORT_ATTACH:       return "ATTACH";
    case LS_VHUB_PORT_DETACH:       return "DETACH";
    default:                        return "UNKNOWN";
    }
}

static void ls_vhub_notify_update(struct ls_vhub *vhub)
{
    struct ls_vhub_ep *ep = vhub->epn.ep;
    struct ls_vhub_req *req = &vhub->epn.req;
    u8 bitmap = 0;

    if (!ep || !vhub->epn.buf || !ep->enabled) {
        vhub->epn.notify_pending = true;
        return;
    }

    if (!list_empty(&req->node)) {
        vhub->epn.notify_pending = true;
        trace_ls_vhub_notify_busy(vhub);
        vhub_dbg_vhub(vhub, "ep1 busy, keep pending\n");
        return;
    }

    vhub->epn.notify_pending = false;

    for (int i = 0; i < vhub->ports_num; i++) {
        if (vhub->ports[i].portsc.change)
            bitmap |= 1 << (i + 1);
    }

    if (!bitmap)
        return;

    vhub_dbg_vhub(vhub, "notify port change: bitmap = 0x%x\n", bitmap);
    trace_ls_vhub_notify_update(vhub, bitmap);

    vhub->epn.buf[0] = bitmap;
    req->request.buf = vhub->epn.buf;
    req->request.length = 1;
    req->request.dma = vhub->epn.buf_dma;
    req->request.actual = 0;
    req->request.complete = ls_vhub_ep1_req_complete;
    req->dma_mapped = false;
    req->internel = true;
    /* queue */
    ls_vhub_epn_queue_req(ep, req);
}

static void ls_vhub_port_set(struct ls_vhub *vhub, u8 port_idx,
                             enum ls_vhub_port_action action)
{
    struct ls_pcie_vhub_port *p = &vhub->ports[port_idx];
    u16 old_status = p->portsc.status;
    u16 clr = 0, set = 0;
    bool change = false;

    switch (action) {
    case LS_VHUB_PORT_RESET_BEGIN:
        clr = USB_PORT_STAT_ENABLE | USB_PORT_STAT_SUSPEND;
        set = USB_PORT_STAT_RESET;
        change = false;
        break;
    case LS_VHUB_PORT_RESET_DONE_HS:
        clr = USB_PORT_STAT_RESET;
        set = USB_PORT_STAT_ENABLE | USB_PORT_STAT_HIGH_SPEED;
        change = true;
        break;
    case LS_VHUB_PORT_ENABLE:
        set = USB_PORT_STAT_ENABLE;
        change = true;
        break;
    case LS_VHUB_PORT_DISABLE:
        clr = USB_PORT_STAT_ENABLE;
        change = true;
        break;
    case LS_VHUB_PORT_SUSPEND:
        set = USB_PORT_STAT_SUSPEND;
        change = true;
        break;
    case LS_VHUB_PORT_RESUME:
        clr = USB_PORT_STAT_SUSPEND;
        change = true;
        break;
    case LS_VHUB_PORT_ATTACH:
        set = USB_PORT_STAT_CONNECTION;
        change = true;
        break;
    case LS_VHUB_PORT_DETACH:
        clr = USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE |
              USB_PORT_STAT_SUSPEND | USB_PORT_STAT_HIGH_SPEED;
        change = true;
        break;
    }

    p->portsc.status = (old_status & ~clr) | set;
    if (!change)
        return;

    u16 chg = p->portsc.status ^ old_status;
    chg &= USB_PORT_STAT_C_CONNECTION |
           USB_PORT_STAT_C_ENABLE |
           USB_PORT_STAT_C_SUSPEND |
           USB_PORT_STAT_C_OVERCURRENT |
           USB_PORT_STAT_C_RESET |
           USB_PORT_STAT_C_L1;

    if (p->portsc.status & USB_PORT_STAT_ENABLE)
        chg &= ~USB_PORT_STAT_C_ENABLE;

    p->portsc.change = chg;

    trace_ls_vhub_port_set(vhub, port_idx,
                           ls_vhub_port_action_name(action),
                           p->portsc.status, p->portsc.change);

    if (chg) {
        /* notify host */
        ls_vhub_notify_update(vhub);
    }
}

static int ls_vhub_hub_status(struct ls_vhub *vhub)
{
    dev_dbg(&vhub->pdev->dev, "hub status!!\n");
    return ls_vhub_ep0_replay(vhub, "\0\0\0\0", 4);
}

static int ls_vhub_port_reset(struct ls_vhub *vhub, u8 port)
{
    struct ls_pcie_vhub_port *p = &vhub->ports[port - 1];

    /* soft-disconnected ports must not grab addr0 or revive the UDC */
    if (!(p->portsc.status & USB_PORT_STAT_CONNECTION)) {
        vhub_dbg_vhub(vhub, "reset port[%d] ignored (not connected)\n", port);
        return 0;
    }

    dev_dbg(&vhub->pdev->dev, "reset port[%d]\n", port);
    ls_vhub_port_set(vhub, port-1, LS_VHUB_PORT_RESET_BEGIN);
    vhub->addr0_dev = &p->dev;
    ls_vhub_dev_reset(vhub, port-1);
    ls_vhub_port_set(vhub, port-1, LS_VHUB_PORT_RESET_DONE_HS);

    return 0;
}

static int ls_vhub_class_req(struct ls_vhub *vhub, struct usb_ctrlrequest *setup)
{
    u16 wValue, wIndex, wLength;

    wValue = le16_to_cpu(setup->wValue);
    wIndex = le16_to_cpu(setup->wIndex);
    wLength = le16_to_cpu(setup->wLength);

    switch ((setup->bRequestType << 8) | setup->bRequest) {
    case GetHubStatus:
        return ls_vhub_hub_status(vhub);
    case GetPortStatus:
        return ls_vhub_port_status(vhub, wIndex & 0xf);
    case GetHubDescriptor:
        {
            if (wValue != (USB_DT_HUB << 8))
                return -ENOTSUPP;
            u16 len = MIN(wLength, (u16)hub_desc.bDescLength);
            return ls_vhub_ep0_replay(vhub, &hub_desc, len);
        }
    case SetHubFeature:
    case ClearHubFeature:
        /* No feature, just complete the requests */
        if (wValue == C_HUB_LOCAL_POWER ||
            wValue == C_HUB_OVER_CURRENT)
            return 0;
        /* ep stall, xhci 4.10.2.1 stall error */
        return -ENOTSUPP;
    case SetPortFeature:
        {
            u8 port = wIndex & 0xf;
            if (!port || port > vhub->ports_num)
                return -EINVAL;
            switch (wValue) {
            case USB_PORT_FEAT_POWER:
                vhub->ports[port-1].portsc.status |= USB_PORT_STAT_POWER;
                break;
            case USB_PORT_FEAT_ENABLE:
                ls_vhub_port_set(vhub, port-1, LS_VHUB_PORT_ENABLE);
                break;
            case USB_PORT_FEAT_RESET:
                ls_vhub_port_reset(vhub, port);
                break;
            case USB_PORT_FEAT_SUSPEND:
                ls_vhub_port_set(vhub, port-1, LS_VHUB_PORT_SUSPEND);
                break;
            case USB_PORT_FEAT_TEST:
            case USB_PORT_FEAT_INDICATOR:
                /* ACK only */
                break;
            default:
                return -EOPNOTSUPP;
            }
            return 0;
        }
    case ClearPortFeature:
        {
            u8 port = wIndex & 0xf;
            if (!port || port > vhub->ports_num)
                return -EINVAL;
            struct ls_pcie_vhub_port *p = &vhub->ports[port - 1];
            switch (wValue) {
            case USB_PORT_FEAT_ENABLE:
                ls_vhub_port_set(vhub, port - 1, LS_VHUB_PORT_DISABLE);
                break;
            case USB_PORT_FEAT_SUSPEND:
                ls_vhub_port_set(vhub, port - 1, LS_VHUB_PORT_RESUME);
                break;
            case USB_PORT_FEAT_POWER:
                p->portsc.status &= ~USB_PORT_STAT_POWER;
                break;
            case USB_PORT_FEAT_INDICATOR:
                break;
            case USB_PORT_FEAT_C_CONNECTION:
            case USB_PORT_FEAT_C_ENABLE:
            case USB_PORT_FEAT_C_SUSPEND:
            case USB_PORT_FEAT_C_OVER_CURRENT:
            case USB_PORT_FEAT_C_RESET:
                p->portsc.change &= ~(1u << (wValue - 16));
                break;
            default:
                return -EOPNOTSUPP;
            }
            return 0;
        }
    case ClearTTBuffer:
    case ResetTT:
    case StopTT:
        return 0;
    case GetTTState:
        return 0;
    default:
        dev_err(&vhub->pdev->dev, "Unknown class request\n");
    }
    return -EOPNOTSUPP;
}

int ls_vhub_attach(struct ls_vhub *vhub, int port_idx)
{
    /* update port state */
    ls_vhub_port_set(vhub, port_idx, LS_VHUB_PORT_ATTACH);

    return 0;
}

int ls_vhub_detach(struct ls_vhub *vhub, int port_idx)
{
    /* update port state */
    ls_vhub_port_set(vhub, port_idx, LS_VHUB_PORT_DETACH);
    return 0;
}

int ls_vhub_setup(struct ls_vhub *vhub, struct usb_ctrlrequest *ctrl)
{
    switch (ctrl->bRequestType & USB_TYPE_MASK) {
    case USB_TYPE_STANDARD:
        return ls_vhub_std_req(vhub, ctrl);
    case USB_TYPE_CLASS:
        return ls_vhub_class_req(vhub, ctrl);
    case USB_TYPE_VENDOR:
    default:
        dev_err(&vhub->pdev->dev, "Not supported setup request\n");
    }

    return -ENOTSUPP;
}

static void dwc3_endpoint_event(struct ls_vhub *vhub, const union evt_buf *evt)
{
    u8 ep_num = evt->depevt.phy_ep_num;
    struct ls_vhub_ep *ep = &vhub->ep_pool.eps[ep_num];

    if (ep->ctrl) {
        ls_vhub_ep0_handle(vhub, evt);
    } else {
        ls_vhub_epn_handle(vhub, evt);
    }
}

static irqreturn_t ls_vhub_irq_thread(int irq, void *arg)
{
    struct ls_vhub *vhub = arg;
    const union evt_buf *evt;
    u32 event;
    unsigned long flags;

    local_bh_disable();
    ls_vhub_lock(vhub, &flags);
    while (kfifo_get(&vhub->fifo, &event)) {
        evt = (union evt_buf *)&event;
        trace_ls_vhub_event(vhub, event);

        if (EVT_IS_DEPEVT(evt)) {
            dwc3_endpoint_event(vhub, evt);
        } else {
            dwc3_dev_event(vhub, evt);
        }
    }
    ls_vhub_unlock(vhub, &flags);
    /* 与上面的 local_bh_disable() 配对，恢复 softirq, 若当前 CPU 有 pending softirq，会在这里被处理 */
    local_bh_enable(); 

    /* unmask interrupt */
    size_t size = vhub_readl(vhub->regs, DWC3_GEVNTSIZ(0));
    size &= ~DWC3_GEVNTSIZ_INTMASK;
    vhub_writel(vhub->regs, DWC3_GEVNTSIZ(0), size);

    return IRQ_HANDLED;
}

static irqreturn_t ls_vhub_irq(int irq, void *arg)
{
    struct ls_vhub *vhub = arg;
    struct event_buffer *evt_buf = &vhub->evt_buf;
    u32 len_words = evt_buf->length / sizeof(u32);

    u32 count = vhub_readl(vhub->regs, DWC3_GEVNTCOUNT(0));
    count &= DWC3_GEVNTCOUNT_MASK;
    if (!count)
        return IRQ_NONE;

    u32 size = vhub_readl(vhub->regs, DWC3_GEVNTSIZ(0));
    size |= DWC3_GEVNTSIZ_INTMASK;
    vhub_writel(vhub->regs, DWC3_GEVNTSIZ(0), size);

    u32 processed = 0;

    while (processed < count) {
        u32 evt = ((u32 *)evt_buf->buf)[evt_buf->dequeue];
        kfifo_put(&vhub->fifo, evt);
        evt_buf->dequeue = (evt_buf->dequeue + 1) % len_words;
        processed += sizeof(u32);
    }

    vhub_writel(vhub->regs, DWC3_GEVNTCOUNT(0), processed);
    return IRQ_WAKE_THREAD; // wakeup thread
}


static inline void dwc3_gadget_dctl_write_safe(struct ls_vhub *vhub, u32 value)
{
    value &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;
    vhub_writel(vhub->regs, DWC3_DCTL, value);
}

static void ls_vhub_common_ep0_init(struct ls_vhub *vhub)
{
    struct ls_vhub_ep *common_ep0 = &vhub->ep_pool.eps[0];
    struct ls_vhub_ep *common_ep1 = &vhub->ep_pool.eps[1];

    vhub->common_ep0.out = common_ep0;
    vhub->common_ep0.in = common_ep1;
    vhub->common_ep0.req.dma_mapped = false;
    vhub->common_ep0.req.internel = true;
    vhub->common_ep0.state = EP0_SETUP_PHASE;
    INIT_LIST_HEAD(&vhub->common_ep0.req.node);

    /* init default ep0 */
    ls_vhub_ep0_init(vhub, NULL, common_ep0);
    ls_vhub_ep0_init(vhub, NULL, common_ep1);
    /* enable default ep0 */
    ls_vhub_ep_enable(common_ep0);
    ls_vhub_ep_enable(common_ep1);

    ls_vhub_ep_prepare_trb(&common_ep0->trb->trb[0], common_ep0->trb_dma, 8, DWC3_TRBCTL_CONTROL_SETUP, false);
    ls_vhub_ep_start_transfer(common_ep0, common_ep0->trb_dma, 0);
    /* alloc data dma buffer */
    vhub->common_ep0.data_len = 512;
    vhub->common_ep0.data = dma_alloc_coherent(&vhub->pdev->dev, vhub->common_ep0.data_len, &vhub->common_ep0.data_dma, GFP_KERNEL);
    /* bounce buffer */
    vhub->bounce = dma_alloc_coherent(&vhub->pdev->dev, 512, &vhub->bounce_dma, GFP_KERNEL);
}

static void ls_vhub_self_ep0_init(struct ls_vhub *vhub)
{
    /* alloc vhub control endpoint */
    vhub->ep0.out = ls_vhub_ep_alloc(&vhub->ep_pool, 0);
    vhub->ep0.in = ls_vhub_ep_alloc(&vhub->ep_pool, 1);
    vhub->ep0.out->ctrl = 1;
    vhub->ep0.in->ctrl = 1;
    vhub->ep0.req.dma_mapped = false;
    vhub->ep0.req.internel = true;
    vhub->ep0.state = EP0_SETUP_PHASE;
    INIT_LIST_HEAD(&vhub->ep0.req.node);

    /* vhub ep0 init */
    ls_vhub_ep0_init(vhub, NULL, vhub->ep0.out);
    ls_vhub_ep0_init(vhub, NULL, vhub->ep0.in);

    /* alloc data dma buffer */
    vhub->ep0.data_len = 512;
    vhub->ep0.data = dma_alloc_coherent(&vhub->pdev->dev, vhub->ep0.data_len, &vhub->ep0.data_dma, GFP_KERNEL);
}

static int ls_vhub_endpoint1_init(struct ls_vhub *vhub)
{
    u8 *buf;
    dma_addr_t buf_dma;
    struct ls_vhub_ep *ep;

    ep = ls_vhub_ep_alloc(&vhub->ep_pool, 1);
    if (!ep)
        return -ENOMEM;
    ep->endpoint.desc = &hub_cfg_desc.endpoint;

    if (ls_vhub_epn_init(vhub, NULL, ep)) {
        ls_vhub_ep_free(&vhub->ep_pool, ep);
        return -ENOMEM;
    }

    buf = dma_alloc_coherent(&vhub->pdev->dev, 512, &buf_dma, GFP_KERNEL);
    if (!buf) {
        ls_vhub_ep_free(&vhub->ep_pool, ep);
        return -ENOMEM;
    }

    vhub->epn.ep = ep;
    vhub->epn.buf = buf;
    vhub->epn.buf_dma = buf_dma;
    vhub->epn.buf_len = 512;
    vhub->epn.req.internel = true;
    vhub->epn.req.dir = true;
    vhub->epn.notify_pending = false;
    INIT_LIST_HEAD(&vhub->epn.req.node);

    return 0;
}

void ls_vhub_ep_map(struct ls_vhub_ep *ep)
{
    union ls_vhub_ep_map ep_map = {0};

    if (!ep->map)
        return;

    ep_map.dev_addr = ep->dev ? ep->dev->address : ep->vhub->address;
    ep_map.ep_num = usb_endpoint_num(ep->endpoint.desc);
    vhub_dbg_ep(ep, "ep map: dev_addr = %d, ep_num = %d\n", ep_map.dev_addr, ep_map.ep_num);
    ep->map->value = ep_map.value;
}

void ls_vhub_ep_unmap(struct ls_vhub_ep *ep)
{
    if (!ep->map)
        return;
    ep->map->value = 0;
}

int ls_vhub_ep_enable(struct ls_vhub_ep *ep)
{
    int ret = __ls_vhub_ep_enable(ep->vhub, ep, Initialize_EP_State);

    if (!ret)
        ep->enabled = 1;

    return ret;
}

int ls_vhub_ep_disable(struct ls_vhub_ep *ep)
{
    struct ls_vhub *vhub = ep->vhub;

    if (!ep->enabled)
        return 0;

    u32 reg = vhub_readl(vhub->regs, DWC3_DALEPENA);
    reg &= ~DWC3_DALEPENA_EP(ep->number);
    vhub_writel(vhub->regs, DWC3_DALEPENA, reg);
    ep->enabled = 0;

    return 0;
}

int ls_vhub_ep_stop_transfer(struct ls_vhub_ep *ep)
{
    int ret;
    u32 cmd;
    struct dep_command_param param = {0};

    if (!ep || !ep->active) {
        return 0;
    }

    cmd = DWC3_DEPCMD_ENDTRANSFER | DWC3_DEPCMD_CMDACT |
          DWC3_DEPCMD_HIPRI_FORCERM | DWC3_DEPCMD_PARAM(ep->res_idx);

    ret = dwc3_dep_command(ep->vhub, ep->number, &cmd, &param);
    if (ret)
        dev_err(&ep->vhub->pdev->dev, "end transfer failed on ep %d: %d\n", ep->number, ret);

    ep->active = 0;
    ep->res_idx = 0;
    ep->trb->trb[0].ctrl &= ~DWC3_TRB_CTRL_HWO;
    ep->trb->trb[1].ctrl &= ~DWC3_TRB_CTRL_HWO;

    return ret;
}

int ls_vhub_ep_set_halt(struct ls_vhub_ep *ep, u8 set)
{
    int ret = 0;

    if (set) {
        if (!list_empty(&ep->req_list))
            return -EAGAIN;

        vhub_dbg_ep(ep, "set stall ep %d\n", ep->number);

        ret = ls_vhub_ep_set_stall(ep);
        if (ret) {
            dev_err(&ep->vhub->pdev->dev, "failed to set STALL on %s\n", ep->name);
            return ret;
        }
        ep->stall = 1;
    } else {
        if (ep->ctrl) {
            ep->stall = 0;
            ep->wedge = 0;
            return 0;
        }

        ls_vhub_ep_stop_transfer(ep);
        ret = ls_vhub_ep_clear_stall(ep);
        if (!ret) {
            ep->stall = 0;
            ep->wedge = 0;
        }

        if (ep->delay_start) {
            ls_vhub_epn_kick_transfer(ep);
            ep->delay_start = 0;
        }
    }

    return ret;
}

static void dwc3_event_buffer_init(struct ls_vhub *vhub)
{
    /* alloc event dma buffer */
    struct event_buffer *evt = &vhub->evt_buf;
    evt->length = DWC3_EVENT_BUFFERS_SIZE;
    evt->dequeue = 0;
    evt->buf = dma_alloc_coherent(&vhub->pdev->dev, DWC3_EVENT_BUFFERS_SIZE, &evt->dma, GFP_KERNEL);
    /* config event regs */
    vhub_writel(vhub->regs, DWC3_GEVNTADRLO(0), lower_32_bits(evt->dma));
    vhub_writel(vhub->regs, DWC3_GEVNTADRHI(0), upper_32_bits(evt->dma));
    vhub_writel(vhub->regs, DWC3_GEVNTSIZ(0), evt->length);
    vhub_writel(vhub->regs, DWC3_GEVNTCOUNT(0), 0);
}

static void dwc3_event_buffer_deinit(struct ls_vhub *vhub)
{
    struct event_buffer *evt = &vhub->evt_buf;
    if (!evt->buf)
        return;

    u32 reg = vhub_readl(vhub->regs, DWC3_DSTS);
    if (!(reg & DWC3_DSTS_DEVCTRLHLT))
        return;

    evt->dequeue = 0;

    vhub_writel(vhub->regs, DWC3_GEVNTADRLO(0), 0);
    vhub_writel(vhub->regs, DWC3_GEVNTADRHI(0), 0);
    vhub_writel(vhub->regs, DWC3_GEVNTSIZ(0), DWC3_GEVNTSIZ_INTMASK | DWC3_GEVNTSIZ_SIZE(0));
    reg = vhub_readl(vhub->regs, DWC3_GEVNTCOUNT(0));
    vhub_writel(vhub->regs, DWC3_GEVNTCOUNT(0), reg);
    dma_free_coherent(&vhub->pdev->dev, evt->length, evt->buf, evt->dma);
}

static void dwc3_hwparams_cache(struct ls_vhub *vhub)
{
    struct dwc3_hwparams *parms = &vhub->hwparams;

    /* alloc event dma buffer */
    parms->hwparams0 = vhub_readl(vhub->regs, DWC3_GHWPARAMS0);
    parms->hwparams1 = vhub_readl(vhub->regs, DWC3_GHWPARAMS1);
    parms->hwparams2 = vhub_readl(vhub->regs, DWC3_GHWPARAMS2);
    parms->hwparams3 = vhub_readl(vhub->regs, DWC3_GHWPARAMS3);
    parms->hwparams4 = vhub_readl(vhub->regs, DWC3_GHWPARAMS4);
    parms->hwparams5 = vhub_readl(vhub->regs, DWC3_GHWPARAMS5);
    parms->hwparams6 = vhub_readl(vhub->regs, DWC3_GHWPARAMS6);
    parms->hwparams7 = vhub_readl(vhub->regs, DWC3_GHWPARAMS7);
    parms->hwparams8 = vhub_readl(vhub->regs, DWC3_GHWPARAMS8);

    dev_dbg(&vhub->pdev->dev, "DWC3 HWPARAMS:\n");
    dev_dbg(&vhub->pdev->dev, "  HWPARAMS0 = 0x%08x\n", parms->hwparams0);
    dev_dbg(&vhub->pdev->dev, "  HWPARAMS1 = 0x%08x\n", parms->hwparams1);
    dev_dbg(&vhub->pdev->dev, "  HWPARAMS2 = 0x%08x\n", parms->hwparams2);
    dev_dbg(&vhub->pdev->dev, "  HWPARAMS3 = 0x%08x\n", parms->hwparams3);
    dev_dbg(&vhub->pdev->dev, "  HWPARAMS4 = 0x%08x\n", parms->hwparams4);
    dev_dbg(&vhub->pdev->dev, "  HWPARAMS5 = 0x%08x\n", parms->hwparams5);
    dev_dbg(&vhub->pdev->dev, "  HWPARAMS6 = 0x%08x\n", parms->hwparams6);
    dev_dbg(&vhub->pdev->dev, "  HWPARAMS7 = 0x%08x\n", parms->hwparams7);
    dev_dbg(&vhub->pdev->dev, "  HWPARAMS8 = 0x%08x\n", parms->hwparams8);
}

static int dwc3_core_soft_reset(struct ls_vhub *vhub)
{
    int retries = 1000;

    u32 reg = vhub_readl(vhub->regs, DWC3_DCTL);
    reg |= DWC3_DCTL_CSFTRST;
    reg &= ~DWC3_DCTL_RUN_STOP;
    dwc3_gadget_dctl_write_safe(vhub, reg);

    do {
        reg = vhub_readl(vhub->regs, DWC3_DCTL);
        if (!(reg & DWC3_DCTL_CSFTRST))
            break;
        udelay(1);
    } while (--retries);

    if (retries == 0) {
        dev_err(&vhub->pdev->dev, "DWC3 controller soft reset failed!!!\n");
        return -1;
    }

    return 0;
}

static int dwc3_core_run(struct ls_vhub *vhub)
{
    /* controller run */
    u32 reg = vhub_readl(vhub->regs, DWC3_DCTL);
    reg |= DWC3_DCTL_RUN_STOP;
    vhub_writel(vhub->regs, DWC3_DCTL, reg);
    return 0;
}

static int dwc3_core_stop(struct ls_vhub *vhub)
{
    int retries = 1000;

    /* controller stop */
    u32 reg = vhub_readl(vhub->regs, DWC3_DCTL);
    reg &= ~DWC3_DCTL_RUN_STOP;
    vhub_writel(vhub->regs, DWC3_DCTL, reg);

    /* wait for controller to halt before tearing down event buffer */
    do {
        reg = vhub_readl(vhub->regs, DWC3_DSTS);
        if (reg & DWC3_DSTS_DEVCTRLHLT)
            break;
        udelay(1);
    } while (--retries);

    return 0;
}

static int dwc3_phy_setup(struct ls_vhub *vhub)
{
    /* setup vendor specific phy */
#define REG_CTRL   0x00
#define REG_CFG    0x04
#define REG_STATUS 0x0c

    /* TODO: dts */
    void __iomem *hw_base = devm_ioremap(&vhub->pdev->dev, 0x96000000, 0x100);
    if (!hw_base)
        return -ENOMEM;

    u32 val = readl(hw_base + REG_STATUS);
    val |= BIT(2);
    writel(val, hw_base + REG_STATUS);
    writel(0xf5,  hw_base + REG_CTRL);
    writel(0x301, hw_base + REG_CFG);

    return 0;
}

static int dwc3_phy_init(struct ls_vhub *vhub)
{
    /* high-speed, slelect ulpi */
    vhub_writel(vhub->regs, DWC3_GUSB2PHYCFG(0), 0x40003817);
    // vhub_writel(vhub->regs, DWC3_GUSB2PHYCFG(0), 0x40002407); // utmi

    return 0;
}

static int dwc3_clk_setup(struct ls_vhub *vhub)
{
    int ret = 0;
    /* zynq */
    vhub->clk = devm_clk_get_optional(&vhub->pdev->dev, "vhub-clk");
    ret = clk_prepare_enable(vhub->clk);
    if (ret) {
        dev_err(&vhub->pdev->dev, "Failed to enable vhub-clk, %d\n", ret);
        return ret;
    }

    return 0;
}

static int ls_vhub_controller_init(struct ls_vhub *vhub)
{
    /* setup usb clk */
    dwc3_clk_setup(vhub);
    /* setup usb phy */
    dwc3_phy_setup(vhub);
    /* cache hardware params */
    dwc3_hwparams_cache(vhub);
    /* soft reset */
    dwc3_core_soft_reset(vhub);
    /* init dwc3 phy reg */
    dwc3_phy_init(vhub);

    /*
     * DCFG: Device Configuration
     *   DevSpd[2:0]   = 0    (HS)
     *   DevAddr[9:3]  = 0
     *   BIT(11)       = 1    (LPM_CAP, pre-2.90a)
     *   NUMP[21:17]   = 4    (receive buffers)
     *   BIT(22)       = 1    (LPM_CAP, >= 2.90a)
     */
    vhub_writel(vhub->regs, DWC3_DCFG, 0x480800);

    /*
     * GUCTL: Global User Control
     *   REFCLKPER[31:22] = 41    (24MHz refclk period in ns)
     *   DTFT[8:0]        = 16    (device timeout: 16*256*8ns = 32us)
     */
    vhub_writel(vhub->regs, DWC3_GUCTL, 0xa400010);

    vhub_writel(vhub->regs, DWC3_GUCTL2, vhub_readl(vhub->regs, DWC3_GUCTL2) | DWC3_GUCTL2_RST_ACTBITLATER);

    /*
     * GCTL: Global Control
     *   PWRDNSCALE[31:19] = 0x618
     *   BIT(16)           = U2RSTECN
     *   PRTCAPDIR[13:12]  = 2    (DEVICE)
     *   BIT(9)
     *   BIT(2)            = U2EXIT_LFPS
     */
    vhub_writel(vhub->regs, DWC3_GCTL, 0x30C12204);

    /*
     * DEVTEN: Device Event Enable
     *   BIT(16) = 1
     *   BIT(15) = 1
     *   BIT(14) = 1
     *   BIT(13) = 1
     *   BIT(12) = 1  (VNDRDEVTSTRCVEDEN)
     *   BIT(9)  = 1  (ERRTICERREN)
     *   BIT(8)  = 1
     *   BIT(6)  = 1  (U3L2L1SUSPEN)
     *   BIT(5)  = 1  (HIBERNATIONREQEVTEN)
     *   BIT(4)  = 1  (WKUPEVTEN)
     *   BIT(3)  = 1  (ULSTCNGEN)
     *   BIT(2)  = 1  (CONNECTDONEEN)
     *   BIT(1)  = 1  (USBRSTEN)
     *   BIT(0)  = 1  (DISCONNEVTEN)
     */
    vhub_writel(vhub->regs, DWC3_DEVTEN, 0x1F37F);

    dwc3_event_buffer_init(vhub);
    /* reset all endpoints */
    dwc3_dep_start_config(vhub, 0);

    return 0;
}

static int ls_vhub_port_init(struct ls_vhub *vhub, u8 port_num)
{
    vhub->ports = devm_kzalloc(&vhub->pdev->dev, sizeof(*vhub->ports) * port_num, GFP_KERNEL);
    vhub->ports_num = port_num;

    for (int i = 0; i < port_num; i++) {
        ls_vhub_dev_init(vhub, i);
    }

    return 0;
}

static int ls_vhub_port_deinit(struct ls_vhub *vhub, u8 port_num)
{
    for (int i = 0; i < port_num; i++) {
        ls_vhub_dev_deinit(vhub, i);
    }

    return 0;
}

static int ls_vhub_endpoint0_init(struct ls_vhub *vhub)
{
    ls_vhub_common_ep0_init(vhub);
    ls_vhub_self_ep0_init(vhub);

    return 0;
}

static int ls_vhub_probe(struct platform_device *pdev)
{
    struct ls_vhub *vhub;
    struct resource *res;
    int ret;
    struct resource dwc_res;

    vhub = devm_kzalloc(&pdev->dev, sizeof(*vhub), GFP_KERNEL);
    if (!vhub)
        return -ENOMEM;

    vhub->pdev = pdev;
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "no MEM resource\n");
        return -ENODEV;
    }

    dwc_res = *res;
    dwc_res.start += DWC3_GLOBALS_REGS_START;
    vhub->regs = devm_ioremap_resource(&pdev->dev, &dwc_res);
    if (IS_ERR(vhub->regs))
        return PTR_ERR(vhub->regs);
    dev_dbg(&pdev->dev, "vhub regs: phys=%pa size=0x%llx mapped=%px\n", &dwc_res.start, resource_size(&dwc_res), vhub->regs);

    platform_set_drvdata(pdev, vhub);
    /* init ep pool */
    vhub->ep_pool.eps = devm_kzalloc(&pdev->dev, VHUB_EP_NUM * sizeof(struct ls_vhub_ep), GFP_KERNEL);
    vhub->ep_pool.trb_pool = dma_alloc_coherent(&pdev->dev, VHUB_EP_NUM * sizeof (struct ls_vhub_ep_trb), &vhub->ep_pool.trb_dma, GFP_KERNEL);
    if (!vhub->ep_pool.trb_pool)
        return -ENOMEM;
    ls_vhub_ep_pool_init(vhub, VHUB_EP_NUM);

    vhub->irq = platform_get_irq(pdev, 0);
    if (vhub->irq < 0) {
        dev_err(&pdev->dev, "failed to get irq\n");
        ret = vhub->irq;
        goto err_trb_pool;
    }

    INIT_KFIFO(vhub->fifo);
    spin_lock_init(&vhub->lock);
    /* irq install */
    ret = devm_request_threaded_irq(&pdev->dev, vhub->irq, ls_vhub_irq, ls_vhub_irq_thread,
                                    IRQF_ONESHOT, "ls_hw_vhub", vhub);
    if (ret) {
        dev_err(&pdev->dev, "failed to request irq\n");
        goto err_trb_pool;
    }

    /* controller init */
    ls_vhub_controller_init(vhub);
    /* vhub endpoint0 init */
    ls_vhub_endpoint0_init(vhub);
    /* debugfs init */
    ls_vhub_debugfs_init(vhub);
    /* init vhub port and alloc pair ep0 */
    ls_vhub_port_init(vhub, 5);
    /* vhub endpoint1 init */
    ls_vhub_endpoint1_init(vhub);
    /* controller run */
    dwc3_core_run(vhub);

    return ret;

err_trb_pool:
    dma_free_coherent(&pdev->dev, VHUB_EP_NUM * sizeof (struct ls_vhub_ep_trb),
                      vhub->ep_pool.trb_pool, vhub->ep_pool.trb_dma);
    return ret;
}

static void ls_vhub_remove(struct platform_device *pdev)
{
    struct ls_vhub *vhub = platform_get_drvdata(pdev);
    vhub_dbg_vhub(vhub, "remove start\n");

    // 1. deinit port device
    ls_vhub_port_deinit(vhub, 5);
    // 2. remove debugfs root
    ls_vhub_debugfs_exit(vhub);
    // 3. stop dwc3 controller
    dwc3_core_stop(vhub);
    // 4. disable irq
    vhub_writel(vhub->regs, DWC3_DEVTEN, 0x00);
    // 5. release event buffer
    dwc3_event_buffer_deinit(vhub);
    // 6. release common ep0 dma buffer
    dma_free_coherent(&vhub->pdev->dev, vhub->common_ep0.data_len, vhub->common_ep0.data, vhub->common_ep0.data_dma);
    // 7. release vhub ep0 dma buffer
    dma_free_coherent(&vhub->pdev->dev, vhub->ep0.data_len, vhub->ep0.data, vhub->ep0.data_dma);
    // 8. release vhub bounce dma buffer
    dma_free_coherent(&vhub->pdev->dev, 512, vhub->bounce, vhub->bounce_dma);
    // 9. release vhub ep1 dma buffer
    dma_free_coherent(&vhub->pdev->dev, vhub->epn.buf_len, vhub->epn.buf, vhub->epn.buf_dma);
    // 10. release vhub trb pool dma buffer
    dma_free_coherent(&vhub->pdev->dev, VHUB_EP_NUM * sizeof (struct ls_vhub_ep_trb), vhub->ep_pool.trb_pool, vhub->ep_pool.trb_dma);

    vhub_dbg_vhub(vhub, "remove end\n");
}

static const struct of_device_id ls_vhub_match_table[] = {
    { .compatible = "linkedsemi,ls-vhub", .data = NULL},
    { }
};

static struct platform_driver ls_vhub_driver = {
    .driver = {
        .name = "ls-vhub",
        .of_match_table = of_match_ptr(ls_vhub_match_table),
    },
    .probe = ls_vhub_probe,
    .remove = ls_vhub_remove,
};

static int __init ls_vhub_init(void)
{
    return platform_driver_register(&ls_vhub_driver);
}

static void __exit ls_vhub_exit(void)
{
    platform_driver_unregister(&ls_vhub_driver);
}

module_init(ls_vhub_init)
module_exit(ls_vhub_exit)

int ls_vhub_debug;
module_param_named(vhub_debug, ls_vhub_debug, int, 0644);
MODULE_PARM_DESC(vhub_debug, "Enable verbose ls-vhub logs (0=off, 1=on)");

MODULE_DEVICE_TABLE(of, ls_vhub_match_table);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zytang");
MODULE_DESCRIPTION("Linkedsemi vhub driver");
