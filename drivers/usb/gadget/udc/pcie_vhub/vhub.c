#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/dma-mapping.h>
#include "linkedsemi-pcie-vusb.h"
#include "vhub.h"

static struct usb_device_descriptor device_desc = {
    .bLength = sizeof(device_desc),
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB =  cpu_to_le16(0x0200),
    .bDeviceClass = 0x9, // hub class
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 1, // Hi-speed hub with single TT
    .bMaxPacketSize0 = 0x40, // ep0 max size
    .idVendor = 0x8888,
    .idProduct = 0x66, 
    .bcdDevice = 0x0100,
    .iManufacturer = 0, // string desc index, 0 表示不存在，host 就不会请求了
    .iProduct = 1, // string desc index
    .iSerialNumber = 0, // string desc index
    .bNumConfigurations = 0x01
};

struct usb_qualifier_descriptor vhub_dev_qualifier = {
    .bLength = 10,                 /* 描述符长度 */
    .bDescriptorType = 6,          /* DEVICE_QUALIFIER */
    .bcdUSB = 0x0200,              /* USB 2.0 */
    .bDeviceClass = 0x00,          /* Device class same as device descriptor */
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,         /* 默认 EP0 最大包大小 */
    .bNumConfigurations = 1,       /* 配置数量 */
    .bRESERVED = 0                 /* 保留 */
};

static struct usb_hub_cfg_desc
{
    struct usb_config_descriptor  cfg;
    struct usb_interface_descriptor interface;
    struct usb_endpoint_descriptor endpoint;
} __packed hub_cfg_desc = {
    .cfg = {
        .bLength	= 9,
        .bDescriptorType	= USB_DT_CONFIG,
        .wTotalLength		= 9 + 9 + 7,
        .bNumInterfaces		= 1,	 // 一个 interface
        .bConfigurationValue	= 1,
        .iConfiguration		= 0, // 0 表示不存在字符串描述符
        .bmAttributes		= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER, // 如果支持 wakeup，host 会休眠，controll 按道理要上中断，但是没上
        .bMaxPower		= 0,
    },
    .interface = {
        .bLength = 9,
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 0,	// 表示第 0 个 interface 描述符
        .bAlternateSetting = 0, // 一个 interface 下可以有不同的配置
        .bNumEndpoints = 1,
        .bInterfaceClass = 0x9, // hub
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 0,
    },
    .endpoint = {
        .bLength = 7,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x81,
        .bmAttributes = 0x3, // Interrupt endpoint, notify host port status is change
        .wMaxPacketSize = 1,
        .bInterval = 0x5 // 2^4 * 125us = 2ms
    }
};

static struct usb_hub_descriptor hub_desc = {
    .bDescLength = 9,
    .bDescriptorType = USB_DT_HUB,
    .bNbrPorts = LS_PCIE_VHUB_PORT_MAX,
    .wHubCharacteristics = cpu_to_le16(HUB_CHAR_NO_LPSM),
    .bPwrOn2PwrGood = 0x0a,
    .bHubContrCurrent = 0,
    .u.hs.DeviceRemovable[0] = 0xC0,
    .u.hs.DeviceRemovable[1] = 0xff
};

static uint8_t vhub_str0[] = { 4, USB_DT_STRING, 0x09, 0x04 };
static uint8_t vhub_str1[] = { 24, USB_DT_STRING,
    0x76, 0x00, 0x69, 0x00,
    0x72, 0x00, 0x74, 0x00,
    0x75, 0x00, 0x61, 0x00,
    0x6C, 0x00,
    0x20, 0x00,
    0x68, 0x00, 0x75, 0x00,
    0x62, 0x00
};

static int ls_pcie_vhub_ep_send(struct ls_pcie_vhub_ep *ep, uint8_t ep_num, void *data, size_t length, bool stall)
{
    struct ls_pcie_vusb_ep_req *req;

    if (!ep)
        return -EINVAL;

    req = &ep->ep_req;

    if (length > ep->buf_len)
        return -EMSGSIZE;

    if (data) {
        memcpy(ep->buf, data, length);
        req->buf = ep->buf;
        req->dma = ep->buf_dma;
    } else {
        req->buf = NULL;
        req->dma = 0;
    }

    req->len = length;
    req->ep = ep_num;
    req->addr = ep->vhub->dev.addr;
    req->vusb = ep->vhub->dev.vusb;
    req->stall = stall;
    req->complete = NULL;

    return ls_pcie_vusb_req_submit(req);
}

static int ls_pcie_vhub_ep0_submit(struct ls_pcie_vhub *vhub, void *data, size_t len, bool stall)
{
    return ls_pcie_vhub_ep_send(&vhub->ep0, 0x80, data, len, stall);
}

static int ls_pcie_vhub_notify_port_change(struct ls_pcie_vhub *vhub, uint8_t port)
{
    struct ls_pcie_vhub_ep *ep1 = &vhub->ep1;
    uint8_t notify_value = 1 << (port + 1); // bit map, bit0 is reserved, bit1 for port1, bit2 for port2, ...
    ls_pcie_vhub_ep_send(ep1, 0x81, &notify_value, 1, 0);

    return 0;
}

enum vhub_port_action {
    VHUB_PORT_RESET_BEGIN,
    VHUB_PORT_RESET_DONE_HS,
    VHUB_PORT_ENABLE,
    VHUB_PORT_DISABLE,
    VHUB_PORT_SUSPEND,
    VHUB_PORT_RESUME,
    VHUB_PORT_ATTACH,
    VHUB_PORT_DETACH,
};

static void ls_pcie_vhub_port_set( struct ls_pcie_vhub *vhub, uint8_t port, enum vhub_port_action action)
{
    struct ls_pcie_vhub_port *p = &vhub->ports[port];
    uint16_t old_status;
    uint16_t clr = 0, set = 0;
    bool change = false;
    uint16_t chg;

    spin_lock(&vhub->lock);
    old_status = p->portsc.status;

    switch (action) {
    case VHUB_PORT_RESET_BEGIN:
        clr = USB_PORT_STAT_ENABLE | USB_PORT_STAT_SUSPEND;
        set = USB_PORT_STAT_RESET;
        change = false;
        break;
    case VHUB_PORT_RESET_DONE_HS:
        clr = USB_PORT_STAT_RESET;
        set = USB_PORT_STAT_ENABLE | USB_PORT_STAT_HIGH_SPEED;
        change = true;
        break;
    case VHUB_PORT_ENABLE:
        set = USB_PORT_STAT_ENABLE;
        change = true;
        break;
    case VHUB_PORT_DISABLE:
        clr = USB_PORT_STAT_ENABLE;
        change = true;
        break;
    case VHUB_PORT_SUSPEND:
        set = USB_PORT_STAT_SUSPEND;
        change = true;
        break;
    case VHUB_PORT_RESUME:
        clr = USB_PORT_STAT_SUSPEND;
        change = true;
        break;
    case VHUB_PORT_ATTACH:
        set = USB_PORT_STAT_CONNECTION;
        change = true;
        break;
    case VHUB_PORT_DETACH:
        clr = USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE;
        change = true;
        break;
    }

    p->portsc.status = (old_status & ~clr) | set;

    if (!change) {
        spin_unlock(&vhub->lock);
        return;
    }

    chg = p->portsc.status ^ old_status;
    chg &= USB_PORT_STAT_C_CONNECTION |
           USB_PORT_STAT_C_ENABLE |
           USB_PORT_STAT_C_SUSPEND |
           USB_PORT_STAT_C_OVERCURRENT |
           USB_PORT_STAT_C_RESET |
           USB_PORT_STAT_C_L1;

    if (p->portsc.status & USB_PORT_STAT_ENABLE)
        chg &= ~USB_PORT_STAT_C_ENABLE;

    p->portsc.change = chg;
    if (chg)
        ls_pcie_vhub_notify_port_change(vhub, port);
    spin_unlock(&vhub->lock);
}

int ls_pcie_vhub_detach(struct ls_pcie_vhub *vhub, uint8_t port_idx)
{
    if (!vhub || port_idx >= vhub->ports_num)
        return -EINVAL;

    spin_lock(&vhub->lock);
    if (vhub->setup_dev == &vhub->ports[port_idx].dev)
        vhub->setup_dev = NULL;
    spin_unlock(&vhub->lock);

    ls_pcie_vhub_port_set(vhub, port_idx, VHUB_PORT_DETACH);

    return 0;
}

int ls_pcie_vhub_attach(struct ls_pcie_vhub *vhub, uint8_t port_idx)
{
    if (!vhub || port_idx >= vhub->ports_num)
        return -EINVAL;

    ls_pcie_vhub_port_set(vhub, port_idx, VHUB_PORT_ATTACH);

    return 0;
}

static int ls_pcie_vhub_dev_status(struct ls_pcie_vhub *vhub, uint16_t wLength)
{
    u16 status = cpu_to_le16(1 << USB_DEVICE_SELF_POWERED);

    return ls_pcie_vhub_ep0_submit(vhub, &status, MIN_T(uint16_t, wLength, 2), false);
}

static int ls_pcie_vhub_stall(struct ls_pcie_vhub *vhub)
{
    return ls_pcie_vhub_ep0_submit(vhub, NULL, 0, true);
}

static int ls_pcie_vhub_feature(struct ls_pcie_vhub *vhub, uint16_t wIndex, uint16_t wValue, bool is_set)
{
    if (wValue != USB_DEVICE_REMOTE_WAKEUP && wValue != USB_DEVICE_TEST_MODE)
        return ls_pcie_vhub_stall(vhub);

    return ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
}

static int ls_pcie_vhub_get_desc(struct ls_pcie_vhub *vhub,
                                    uint8_t type,
                                    uint16_t wLength)
{
    void *desc = NULL;
    uint32_t desc_len = 0;
    bool stall = 0;

    switch (type) {
    case USB_DT_DEVICE:
        desc = &device_desc;
        desc_len = sizeof(device_desc);
        break;

    case USB_DT_CONFIG:
    case USB_DT_OTHER_SPEED_CONFIG:
        desc = &hub_cfg_desc;
        desc_len = sizeof(hub_cfg_desc);
        break;

    case USB_DT_HUB:
        desc = &hub_desc;
        desc_len = sizeof(hub_desc);
        break;

    case USB_DT_DEVICE_QUALIFIER:
        desc = &vhub_dev_qualifier;
        desc_len = sizeof (vhub_dev_qualifier);
        break;
    default:
        desc = NULL;
        desc_len = 0;
        stall = 1;
        break;
    }

    return ls_pcie_vhub_ep_send(&vhub->ep0, 0x80, desc, MIN(wLength, desc_len), stall);
}

static int ls_pcie_vhub_get_string(struct ls_pcie_vhub *vhub, uint8_t index, uint16_t wLength)
{
    uint8_t *str_desc = NULL;
    uint16_t len = 0;
    bool stall = false;

    if (index == 0 || index == 1) {
        str_desc = index ? vhub_str1 : vhub_str0;
        len = MIN(wLength, str_desc[0]);
    } else {
        stall = true;
    }

    return ls_pcie_vhub_ep_send(&vhub->ep0, 0x80, str_desc, len, stall);
}

static int ls_pcie_vhub_std_req(struct ls_pcie_vhub *vhub, struct usb_ctrlrequest *setup)
{
    uint8_t type = setup->bRequestType;
    uint8_t req = setup->bRequest;
    uint16_t wValue  = le16_to_cpu(setup->wValue);
    uint16_t wIndex  = le16_to_cpu(setup->wIndex);
    uint16_t wLength = le16_to_cpu(setup->wLength);

    if ((type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
        switch (req) {
        case USB_REQ_SET_ADDRESS:
            if (wIndex || wLength || wValue > 127)
                return ls_pcie_vhub_stall(vhub);
            vhub->dev.addr = wValue & 0x7f;
            return ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
        case USB_REQ_GET_STATUS:
            return ls_pcie_vhub_dev_status(vhub, wLength);
        case USB_REQ_SET_FEATURE:
            return ls_pcie_vhub_feature(vhub, wIndex, wValue, 1);
        case USB_REQ_CLEAR_FEATURE:
            return ls_pcie_vhub_feature(vhub, wIndex, wValue, 0);
        case USB_REQ_GET_CONFIGURATION:
        {
            uint8_t config = vhub->ep1.enabled ? 1 : 0;

            return ls_pcie_vhub_ep0_submit(vhub, &config, MIN_T(uint16_t, wLength, 1), false);
        }
        case USB_REQ_SET_CONFIGURATION:
            if (wValue > 1)
                return ls_pcie_vhub_stall(vhub);
            vhub->ep1.enabled = (wValue != 0);
            return ls_pcie_vhub_ep_send(&vhub->ep0, 0x80, NULL, 0, 0);
        case USB_REQ_GET_DESCRIPTOR:
            switch (wValue >> 8) {
            case USB_DT_DEVICE:
            case USB_DT_CONFIG:
            case USB_DT_DEVICE_QUALIFIER:
            case USB_DT_OTHER_SPEED_CONFIG:
                return ls_pcie_vhub_get_desc(vhub, wValue >> 8, wLength);
            case USB_DT_STRING:
                return ls_pcie_vhub_get_string(vhub, wValue & 0xff, wLength);
            }
            return ls_pcie_vhub_stall(vhub);
        case USB_REQ_GET_INTERFACE:
        {
            uint8_t alt = 0;

            if (wIndex)
                return ls_pcie_vhub_stall(vhub);
            return ls_pcie_vhub_ep0_submit(vhub, &alt, MIN_T(uint16_t, wLength, 1), false);
        }
        case USB_REQ_SET_INTERFACE:
            if (wValue || wIndex)
                return ls_pcie_vhub_stall(vhub);
            return ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
        }
    }

    if ((type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
        switch (req) {
        case USB_REQ_GET_STATUS:
            return ls_pcie_vhub_ep0_submit(vhub, "\0\0", MIN_T(uint16_t, wLength, 2), false);
        }
    }

    if ((type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {

        switch (req) {
        case USB_REQ_GET_STATUS:
            return ls_pcie_vhub_ep0_submit(vhub, "\0\0", MIN_T(uint16_t, wLength, 2), false);
        case USB_REQ_SET_FEATURE:
        case USB_REQ_CLEAR_FEATURE:
            if (wValue != USB_ENDPOINT_HALT)
                return ls_pcie_vhub_stall(vhub);
            return ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
        }
    }

    return ls_pcie_vhub_stall(vhub);
}

static int ls_pcie_vhub_hub_status(struct ls_pcie_vhub *vhub)
{
    return ls_pcie_vhub_ep0_submit(vhub, "\0\0\0\0", 4, false);
}

static int ls_pcie_vhub_port_status(struct ls_pcie_vhub *vhub, uint8_t port_index)
{
    uint32_t raw;
    if (!port_index || port_index > vhub->ports_num)
        return ls_pcie_vhub_stall(vhub);

    struct ls_pcie_vhub_port *port = &vhub->ports[port_index - 1];
    raw = port->portsc.raw;

    return ls_pcie_vhub_ep0_submit(vhub, &raw, 4, false);
}

static int ls_pcie_vhub_port_reset(struct ls_pcie_vhub *vhub, uint8_t port_index)
{
    int ret;
    if (port_index >= vhub->ports_num)
        return ls_pcie_vhub_stall(vhub);

    /* finish setup stage */
    ret = ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
    if (ret)
        return ret;

    /* update port status */
    ls_pcie_vhub_port_set(vhub, port_index, VHUB_PORT_RESET_BEGIN);
    /* dev reset */
    ls_pcie_vhub_dev_reset(&vhub->ports[port_index].dev);
    /* port change */
    ls_pcie_vhub_port_set(vhub, port_index, VHUB_PORT_RESET_DONE_HS);
    /* vhub mark current setup_dev */
    spin_lock(&vhub->lock);
    vhub->setup_dev = &vhub->ports[port_index].dev;
    spin_unlock(&vhub->lock);

    return ret;
}

static int ls_pcie_vhub_port_suspend(struct ls_pcie_vhub *vhub, uint8_t port_index)
{
    ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
    return 0;
}

static int ls_pcie_vhub_set_port_feature(struct ls_pcie_vhub *vhub, uint8_t port_idx, uint16_t feat)
{
    struct ls_pcie_vhub_port *port;

    if (port_idx == 0 || port_idx > vhub->ports_num)
        return ls_pcie_vhub_stall(vhub);
    port_idx--;

    port = &vhub->ports[port_idx];

    switch (feat) {
    case USB_PORT_FEAT_POWER:
        spin_lock(&vhub->lock);
        port->portsc.status |= USB_PORT_STAT_POWER;
        spin_unlock(&vhub->lock);
        return ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
    case USB_PORT_FEAT_RESET:
        return ls_pcie_vhub_port_reset(vhub, port_idx);
    case USB_PORT_FEAT_SUSPEND:
        spin_lock(&vhub->lock);
        port->portsc.status |= USB_PORT_STAT_SUSPEND;
        spin_unlock(&vhub->lock);
        return ls_pcie_vhub_port_suspend(vhub, port_idx);
    case USB_PORT_FEAT_TEST:
    case USB_PORT_FEAT_INDICATOR:
        return ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
    default:
        /* stall*/
        return ls_pcie_vhub_stall(vhub);
    }
}

static int ls_pcie_vhub_clr_port_feature(struct ls_pcie_vhub *vhub, uint8_t port, uint16_t feat)
{
    struct ls_pcie_vhub_port *p;

    if (port == 0 || port > vhub->ports_num)
        return ls_pcie_vhub_stall(vhub);

    port--;
    p = &vhub->ports[port];

    spin_lock(&vhub->lock);
    switch(feat) {
    case USB_PORT_FEAT_ENABLE:
        p->portsc.status &= ~(USB_PORT_STAT_ENABLE | USB_PORT_STAT_SUSPEND);
        break;
    case USB_PORT_FEAT_SUSPEND:
        p->portsc.status &= ~USB_PORT_STAT_SUSPEND;
        break;
    case USB_PORT_FEAT_POWER:
        break;
    case USB_PORT_FEAT_INDICATOR:
        break;
    case USB_PORT_FEAT_C_CONNECTION:
    case USB_PORT_FEAT_C_ENABLE:
    case USB_PORT_FEAT_C_SUSPEND:
    case USB_PORT_FEAT_C_OVER_CURRENT:
    case USB_PORT_FEAT_C_RESET:
        p->portsc.change &= ~(1u << (feat - 16));
        break;
    }
    spin_unlock(&vhub->lock);

    /* ep transfer */
    return ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
}

static int ls_pcie_vhub_class_req(struct ls_pcie_vhub *vhub, struct usb_ctrlrequest *setup)
{
    uint16_t wValue, wIndex, wLength;

    wValue = le16_to_cpu(setup->wValue);
    wIndex = le16_to_cpu(setup->wIndex);
    wLength = le16_to_cpu(setup->wLength);

    switch ((setup->bRequestType << 8) | setup->bRequest) {
    case GetHubStatus:
        ls_pcie_vhub_hub_status(vhub);
        break;
    case GetPortStatus:
        ls_pcie_vhub_port_status(vhub, wIndex & 0xf);
        break;
    case GetHubDescriptor:
        {
            if (wValue != (USB_DT_HUB << 8))
                return ls_pcie_vhub_stall(vhub);
            size_t len = MIN_T(size_t, sizeof(hub_desc), wLength);
            ls_pcie_vhub_ep0_submit(vhub, &hub_desc, len, false);
        }
        break;
    case SetHubFeature:
    case ClearHubFeature:
        return ls_pcie_vhub_stall(vhub);
    case SetPortFeature:
        return ls_pcie_vhub_set_port_feature(vhub, wIndex & 0xf, wValue);
    case ClearPortFeature:
        return ls_pcie_vhub_clr_port_feature(vhub, wIndex & 0xf, wValue);
    case ClearTTBuffer:
    case ResetTT:
    case StopTT:
        return ls_pcie_vhub_ep0_submit(vhub, NULL, 0, false);
    case GetTTState:
        return ls_pcie_vhub_ep0_submit(vhub, "\0\0", MIN_T(uint16_t, wLength, 2), false);
    default:
        printk("Unknown class request\n");
        return ls_pcie_vhub_stall(vhub);
    }
    return 0;
}

static struct ls_pcie_vhub_dev *ls_pcie_vhub_find_dev(struct ls_pcie_vhub *vhub, uint8_t addr)
{
    /* 0 is USB default address: resolve via setup_dev path only. */
    if (addr == 0)
        return NULL;

    for (int i = 0; i < vhub->ports_num; i++) {
        struct ls_pcie_vhub_port *port = &vhub->ports[i];
        if (port->dev.enabled &&
            port->dev.addr == addr) {
            return &port->dev;
        }
    }

    return NULL;
}

static struct ls_pcie_vhub_dev *ls_pcie_vhub_get_setup_dev(struct ls_pcie_vhub *vhub)
{
    struct ls_pcie_vhub_dev *dev;

    spin_lock(&vhub->lock);
    dev = vhub->setup_dev;
    if (dev && !dev->enabled)
        dev = NULL;
    spin_unlock(&vhub->lock);

    return dev;
}

static int vhub_handle_root_setup(struct ls_pcie_vhub *vhub, struct usb_ctrlrequest *ctrl)
{
    printk("vhub_setup: type=0x%x, request=0x%x, value=0x%x, index=0x%x, length=%d\n",
            ctrl->bRequestType, ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);

    switch (ctrl->bRequestType & USB_TYPE_MASK) {
    case USB_TYPE_STANDARD:
        return ls_pcie_vhub_std_req(vhub, ctrl);
    case USB_TYPE_CLASS:
        return ls_pcie_vhub_class_req(vhub, ctrl);
    case USB_TYPE_VENDOR:
        return ls_pcie_vhub_stall(vhub);
    default:
        return ls_pcie_vhub_stall(vhub);
    }
}

static int vhub_handle_dev_setup(struct ls_pcie_vhub *vhub, struct usb_ctrlrequest *ctrl, uint8_t addr)
{
    struct ls_pcie_vhub_dev *dev;

    /* USB default-address traffic must target the current setup device. */
    if (addr == 0)
        dev = ls_pcie_vhub_get_setup_dev(vhub);
    else
        dev = ls_pcie_vhub_find_dev(vhub, addr);

    if (!dev) {
        printk("vhub: no device for addr %u\n", addr);
        return -ENODEV;
    }

    return ls_pcie_vhub_dev_setup(dev, ctrl);
}

static int vhub_setup_packet(struct ls_pcie_vusb_dev *vdev, uint8_t *setup, uint8_t addr)
{
    struct usb_ctrlrequest *ctrl = (struct usb_ctrlrequest *)setup;
    struct ls_pcie_vhub *vhub = container_of(vdev, struct ls_pcie_vhub, dev);

    if (addr == vdev->addr)
        return vhub_handle_root_setup(vhub, ctrl);

    return vhub_handle_dev_setup(vhub, ctrl, addr);
}

static int vhub_set_address(struct ls_pcie_vusb_dev *vdev, uint8_t addr)
{
    /* set address */
    struct ls_pcie_vhub *vhub = container_of(vdev, struct ls_pcie_vhub, dev);
    struct ls_pcie_vhub_dev *dev;
    int ret;

    if (addr == 0 || addr >= 128)
        return -EINVAL;

    if (vdev->addr == 0) {
        vdev->addr = addr;
        return 0;
    }

    dev = ls_pcie_vhub_get_setup_dev(vhub);
    if (!dev) {
        printk("can't find setup dev for addr %d\n", addr);
        return -ENODEV;
    }

    struct usb_ctrlrequest ctrl = {
        .bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
        .bRequest = USB_REQ_SET_ADDRESS,
        .wValue = cpu_to_le16(addr),
        .wIndex = 0,
        .wLength = 0
    };

    ret = ls_pcie_vhub_dev_setup(dev, &ctrl);
    if (ret)
        return ret;

    spin_lock(&vhub->lock);
    if (vhub->setup_dev == dev)
        vhub->setup_dev = NULL;
    spin_unlock(&vhub->lock);

    return 0;
}

static int vhub_reset(struct ls_pcie_vusb_dev *vdev)
{
    struct ls_pcie_vhub *vhub = container_of(vdev, struct ls_pcie_vhub, dev);

    vdev->addr = 0;
    vhub->ep1.enabled = false;
    spin_lock(&vhub->lock);
    vhub->setup_dev = NULL;
    spin_unlock(&vhub->lock);

    return 0;
}

static struct ls_pcie_vusb_ops vhub_ops = {
    .set_address = vhub_set_address,
    .setup_packet = vhub_setup_packet,
    .reset = vhub_reset
};

static ssize_t debug_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t debug_cmd_show(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR_RW(debug_cmd);

static int ls_pcie_vhub_ep_init(struct ls_pcie_vhub *vhub)
{
    vhub->ep0.buf = dma_alloc_coherent(vhub->dev.dma_dev, 64, &vhub->ep0.buf_dma, GFP_KERNEL);
    vhub->ep1.buf = dma_alloc_coherent(vhub->dev.dma_dev, 512, &vhub->ep1.buf_dma, GFP_KERNEL);
    if (!vhub->ep0.buf || !vhub->ep1.buf)
        return -ENOMEM;

    vhub->ep0.buf_len = 64;
    vhub->ep1.buf_len = 512;

    vhub->ep0.vhub = vhub;
    vhub->ep1.vhub = vhub;

    return 0;
}

static int ls_pcie_vhub_probe(struct platform_device *pdev)
{
    struct ls_pcie_vusb *vusb = dev_get_drvdata(pdev->dev.parent);
    if (!vusb) {
        dev_err(&pdev->dev, "parent is null??\n");
        return -EPROBE_DEFER;
    }

    struct ls_pcie_vhub *vhub = devm_kzalloc(&pdev->dev, sizeof(*vhub), GFP_KERNEL);
    if (!vhub) {
        dev_err(&pdev->dev, "pcie vhub can't alloc\n");
        return -ENOMEM;
    }

    spin_lock_init(&vhub->lock);
    vhub->dev.vusb = vusb;
    vhub->pdev = pdev;
    vhub->ports_num = LS_PCIE_VHUB_PORT_MAX;
    vhub->ports = devm_kzalloc(&pdev->dev, sizeof(*vhub->ports) * vhub->ports_num, GFP_KERNEL);
    platform_set_drvdata(pdev, vhub);
    device_create_file(&pdev->dev, &dev_attr_debug_cmd);
    ls_pcie_vusb_add_dev(&vhub->dev, &vhub_ops);

    ls_pcie_vhub_ep_init(vhub);
    for (int i = 0; i < vhub->ports_num; i++) {
        ls_pcie_vhub_dev_init(vhub, i);
    }
    printk("ls_pcie_vhub_probe\n");

    return 0;
}

static void ls_pcie_vhub_remove(struct platform_device *pdev)
{
    struct ls_pcie_vhub *vhub =  platform_get_drvdata(pdev);

    printk("ls_pcie_vhub_remove!!\n");

    device_remove_file(&pdev->dev, &dev_attr_debug_cmd);

    /* Stop host-side traffic before teardown. */
    ls_pcie_vusb_disconnect_dev(&vhub->dev);

    for (int i = 0; i < vhub->ports_num; i++) {
        ls_pcie_vhub_dev_deinit(vhub, i);
    }

    if (vhub->ep0.buf)
        dma_free_coherent(vhub->dev.dma_dev, 64, vhub->ep0.buf, vhub->ep0.buf_dma);
    if (vhub->ep1.buf)
        dma_free_coherent(vhub->dev.dma_dev, 512, vhub->ep1.buf, vhub->ep1.buf_dma);

    ls_pcie_vusb_delete_dev(&vhub->dev);
}

static const struct of_device_id ls_pcie_vhub_match_table[] = {
    { .compatible = "linkedsemi,ls-pcie-vhub", .data = NULL},
    { }
};

static struct platform_driver ls_pcie_vhub_driver = {
    .driver = {
        .name = "ls-pcie-vhub",
        .of_match_table = of_match_ptr(ls_pcie_vhub_match_table),
    },
    .probe = ls_pcie_vhub_probe,
    .remove = ls_pcie_vhub_remove,
};

static int __init ls_pcie_vhub_init(void)
{
    return platform_driver_register(&ls_pcie_vhub_driver);
}

static void __exit ls_pcie_vhub_exit(void)
{
    platform_driver_unregister(&ls_pcie_vhub_driver);
}

module_init(ls_pcie_vhub_init)
module_exit(ls_pcie_vhub_exit)

MODULE_DEVICE_TABLE(of, ls_pcie_vhub_match_table);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zytang");
MODULE_DESCRIPTION("Linkedsemi PCIe vhub driver");


/* debug commands */
struct vusb_debug_cmd
{
    const char *name;
    int (*handler)(struct ls_pcie_vhub *hub, int argc, char **argv);
    const char *help;
};

static int cmd_vhub_attach(struct ls_pcie_vhub *hub, int argc, char **argv)
{
    ls_pcie_vusb_connect_dev(&hub->dev);
    return 0;
}

static struct vusb_debug_cmd debug_cmds[] = {
    {"vhub_attach", cmd_vhub_attach, "vhub_attach" },
};

static int run_debug_cmd(struct ls_pcie_vhub *vhub, char *buf)
{
    char *argv[8];
    int argc = 0;
    char *token;
    int i;

    while ((token = strsep(&buf, " \n")) != NULL) {
        if (*token == '\0')
            continue;

        argv[argc++] = token;

        if (argc >= ARRAY_SIZE(argv))
            break;
    }

    if (argc == 0)
        return -EINVAL;

    for (i = 0; i < ARRAY_SIZE(debug_cmds); i++) {
        if (!strcmp(argv[0], debug_cmds[i].name))
            return debug_cmds[i].handler(vhub, argc, argv);
    }

    printk("unknown cmd: %s\n", argv[0]);
    return -EINVAL;
}

static ssize_t debug_cmd_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    struct ls_pcie_vhub *vhub = dev_get_drvdata(dev);
    char kbuf[128];

    strncpy(kbuf, buf, sizeof(kbuf));
    kbuf[sizeof(kbuf) - 1] = '\0';

    run_debug_cmd(vhub, kbuf);

    return count;
}

static ssize_t debug_cmd_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    int i;
    int len = 0;

    for (i = 0; i < ARRAY_SIZE(debug_cmds); i++)
        len += sprintf(buf + len, "%s : %s\n",
                       debug_cmds[i].name,
                       debug_cmds[i].help);

    return len;
}
