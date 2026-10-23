#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include "linkedsemi-pcie-vusb.h"
#include "pcie-linkedsemi.h"
#include "trace.h"

#define LS_PCEI_VUSB_CTRL_OFFSET           (0x200) // PCIe vusb control reg
#define LS_PCOE_VUSB_XHCI_CAP_OFFSET       (0x0)   // Capability Reg
#define LS_PCOE_VUSB_XHCI_OP_OFFSET        (0x80)  // Operational Reg
#define LS_PCOE_VUSB_XHCI_PT_OFFSET        (0x180)  // Port Reg
#define LS_PCOE_VUSB_XHCI_RT_OFFSET        (0x1a0) // Runtime Reg
#define LS_PCOE_VUSB_XHCI_DB_OFFSET        (0x130) // Doorbell Reg
#define LS_PCOE_VUSB_XHCI_ET_OFFSET        (0x100) // Extended Capability Pointer Reg
#define LS_PCOE_VUSB_TRANSFER              (0x666) // usr transfer
#define LS_PCOE_VUSB_PORT_CHANGE           (0x667) // usr attach/detach

#define LS_PCIE_VUSB_CPU_WR_OFFSET         (0x800)

/* bit definitions */
#define USBCMD_RS       (1<<0) // run/stop 控制器
#define USBCMD_HCRST    (1<<1) // 复位控制器
#define USBCMD_INTE     (1<<2) // 中断总开关
#define USBCMD_HSEE     (1<<3)
#define USBCMD_LHCRST   (1<<7)
#define USBCMD_CSS      (1<<8)
#define USBCMD_CRS      (1<<9)
#define USBCMD_EWE      (1<<10)
#define USBCMD_EU3S     (1<<11)

#define USBSTS_HCH      (1<<0) // 控制器状态，value 与 USBCMD_RS 相反，由控制器维护
#define USBSTS_HSE      (1<<2) // 
#define USBSTS_EINT     (1<<3) // pending bit 从 0 到 1 跳变，该位置位。要先清该位，然后清 irq pending bit
#define USBSTS_PCD      (1<<4) // 多种 portsc change bit（如 Connect Status Change、Port Reset Change 等），都会置位
#define USBSTS_SSS      (1<<8) // USBCMD_CSS 置为 1，这个会被控制器设置为 1，控制器完成后，清 0 该 bit, 表示 usbcmd 的 css 完成
#define USBSTS_RSS      (1<<9) // 同上
#define USBSTS_SRE      (1<<10) 
#define USBSTS_CNR      (1<<11) // 为 1 表示控制器没准备好
#define USBSTS_HCE      (1<<12) // 控制器 error

#define PORTSC_CCS          (1<<0)  // CCS 是“此刻是否有设备连接”的真实状态
#define PORTSC_PED          (1<<1) // USB2 端口的 PED 启用依赖 端口复位
#define PORTSC_OCA          (1<<3)
#define PORTSC_PR           (1<<4)
#define PORTSC_PLS_SHIFT        5
#define PORTSC_PLS_MASK     0xf
#define PORTSC_PP           (1<<9)
#define PORTSC_SPEED_SHIFT      10
#define PORTSC_SPEED_MASK   0xf
#define PORTSC_SPEED_FULL   (1<<10)
#define PORTSC_SPEED_LOW    (2<<10)
#define PORTSC_SPEED_HIGH   (3<<10)
#define PORTSC_SPEED_SUPER  (4<<10)
#define PORTSC_PIC_SHIFT        14
#define PORTSC_PIC_MASK     0x3
#define PORTSC_LWS          (1<<16)
#define PORTSC_CSC          (1<<17) // CSC 是“状态发生过变化”的一次性事件
#define PORTSC_PEC          (1<<18)
#define PORTSC_WRC          (1<<19)
#define PORTSC_OCC          (1<<20)
#define PORTSC_PRC          (1<<21)
#define PORTSC_PLC          (1<<22)
#define PORTSC_CEC          (1<<23)
#define PORTSC_CAS          (1<<24)
#define PORTSC_WCE          (1<<25)
#define PORTSC_WDE          (1<<26)
#define PORTSC_WOE          (1<<27)
#define PORTSC_DR           (1<<30)
#define PORTSC_WPR          (1<<31)

#define IMAN_IP         (1<<0)
#define IMAN_IE         (1<<1)
#define ERDP_EHB        (1<<3)

#define MAX_EXIT_LATENCY_MASK   (0xffff)        /* slot_ctx.dev_info2[15:0] */
#define INT_TARGET_MASK         (0x7ff)         /* slot_ctx.dev_state[10:0] */
#define MAX_PACKET_MASK         (0xffff << 16)  /* ep_ctx.ep_info2[31:16] */

/* Completion Code - only applicable for some types of TRBs */
#define COMP_CODE_MASK        (0xff)
#define COMP_CODE(p)          (((p) & 0xff) << 24)

enum xhci_comp_code
{
    COMP_INVALID,
    COMP_SUCCESS,
    COMP_DATA_BUFFER_ERROR,
    COMP_BABBLE_DETECTED_ERROR,
    COMP_USB_TRANSACTION_ERROR,
    COMP_TRB_ERROR,
    COMP_STALL_ERROR,
    COMP_RESOURCE_ERROR,
    COMP_BANDWIDTH_ERROR,
    COMP_NO_SLOTS_AVAILABLE_ERROR,
    COMP_INVALID_STREAM_TYPE_ERROR,
    COMP_SLOT_NOT_ENABLED_ERROR,
    COMP_ENDPOINT_NOT_ENABLED_ERROR,
    COMP_SHORT_PACKET,
    COMP_RING_UNDERRUN,
    COMP_RING_OVERRUN,
    COMP_VF_EVENT_RING_FULL_ERROR,
    COMP_PARAMETER_ERROR,
    COMP_BANDWIDTH_OVERRUN_ERROR,
    COMP_CONTEXT_STATE_ERROR,
    COMP_NO_PING_RESPONSE_ERROR,
    COMP_EVENT_RING_FULL_ERROR,
    COMP_INCOMPATIBLE_DEVICE_ERROR,
    COMP_MISSED_SERVICE_ERROR,
    COMP_COMMAND_RING_STOPPED,
    COMP_COMMAND_ABORTED,
    COMP_STOPPED,
    COMP_STOPPED_LENGTH_INVALID,
    COMP_STOPPED_SHORT_PACKET,
    COMP_MAX_EXIT_LATENCY_TOO_LARGE_ERROR,
    COMP_ISOCH_BUFFER_OVERRUN = 31,
    COMP_EVENT_LOST_ERROR,
    COMP_UNDEFINED_ERROR,
    COMP_INVALID_STREAM_ID_ERROR,
    COMP_SECONDARY_BANDWIDTH_ERROR,
    COMP_SPLIT_TRANSACTION_ERROR
};

enum
{
    PLS_U0              =  0,
    PLS_U1              =  1,
    PLS_U2              =  2,
    PLS_U3              =  3,
    PLS_DISABLED        =  4,
    PLS_RX_DETECT       =  5,
    PLS_INACTIVE        =  6,
    PLS_POLLING         =  7,
    PLS_RECOVERY        =  8,
    PLS_HOT_RESET       =  9,
    PLS_COMPILANCE_MODE = 10,
    PLS_TEST_MODE       = 11,
    PLS_RESUME          = 15,
};

#define XHCI_TRB_TYPE_MASK        (0x3f)
#define XHCI_TRB_EP_ID_MASK        (0x3f)


#define XHCI_TRB_TYPE_GET(p)      (((p) >> 10) & XHCI_TRB_TYPE_MASK)
#define XHCI_TRB_CS(p)            ((p) & BIT(0))
#define XHCI_TRB_TYPE(p)          ((((p) & XHCI_TRB_TYPE_MASK) << 10))
#define XHCI_TRB_EP_ID(p)          ((((p) & XHCI_TRB_EP_ID_MASK) << 16))
#define XHCI_TRB_TD_SIZE_GET(p)   (((p) >> 17) & 0x1f)

#define XHCI_SLOT_GET(p)          (((p) >> 24) & 0xff)
#define XHCI_SLOT(p)              ((((p) & 0xff) << 24))
#define XHCI_SLOT_CTX_HUB(p)      (((p) >> 26) & 0x1)
#define XHCI_SLOT_CTX_PORTS(p)    (((p) >> 24) & 0xff)
#define XHCI_SLOT_CTX_PORT_IDX(p) (((p) >> 16) & 0xff)
#define XHCI_SLOT_CTX_ROOT_STRING(p) ((p) & 0xfffff)

#define XHCI_BSR_GET(p)          (((p) >> 9) & 0x1)
/* Reset Endpoint TRB:Transfer State Preserve(软复位标志,field[3] bit9) */
#define XHCI_TRB_TSP(p)          (((p) >> 9) & 0x1)

#define XHCI_TRB_ISP(p)      (((p) >> 2) & 0x1)
#define XHCI_TRB_CH(p)       (((p) >> 4) & 0x1)
#define XHCI_TRB_IOC(p)      (((p) >> 5) & 0x1)
#define XHCI_TRB_DIR(p)      (((p) >> 16) & 0x1)
#define XHCI_TRB_DC(p)       (((p) >> 9) & 0x1)
#define XHCI_TRB_EP(p)       (((p) >> 16) & 0x1f)
#define XHCI_TRB_SP(p)       (((p) >> 23) & 0x1)
#define XHCI_TRB_IDT(p)      (((p) >> 6) & 0x1)

#define TRB_CYCLE       (1<<0)
#define TRB_TC          (1<<1)

enum xhci_trb_type
{
    TRB_NORMAL = 1,
    TRB_SETUP,
    TRB_DATA,
    TRB_STATUS,
    TRB_ISOC,
    TRB_LINK,
    TRB_EVENT_DATA,
    TRB_TR_NOOP,
    TRB_ENABLE_SLOT,
    TRB_DISABLE_SLOT,
    TRB_ADDR_DEV,
    TRB_CONFIG_EP,
    TRB_EVAL_CONTEXT,
    TRB_RESET_EP,
    TRB_STOP_RING,
    TRB_SET_DEQ,
    TRB_RESET_DEV,
    TRB_FORCE_EVENT,
    TRB_NEG_BANDWIDTH,
    TRB_SET_LT,
    TRB_GET_BW,
    TRB_FORCE_HEADER,
    TRB_CMD_NOOP,
    TRB_TRANSFER = 32,
    TRB_COMPLETION,
    TRB_PORT_STATUS,
    TRB_BANDWIDTH_EVENT,
    TRB_DOORBELL,
    TRB_HC_EVENT,
    TRB_DEV_NOTE,
    TRB_MFINDEX_WRAP,
    TRB_TYPE_MAX
};

#define EP_STATE_MASK   (0x7)

enum xhci_ep_state
{
    EP_STATE_DISABLE,
    EP_STATE_RUNNING,
    EP_STATE_HALTED,
    EP_STATE_STOPPED,
    EP_STATE_ERROR
};

#define SLOT_STATE_MASK     ((0x1f) << 27)
#define SLOT_CTX_ENTR       ((0x1f) << 27)

#define SET_SLOT_STATE(p)   ((p & 0x1f) << 27)
#define SET_SLOT_CTX_ENTR(p)   ((p & 0x1f) << 27)

enum xhci_slot_state
{
    SLOT_STATE_DISABLE,
    SLOT_STATE_ENABLE,
    SLOT_STATE_DEFAULT,
    SLOT_STATE_ADDRESSED,
    SLOT_STATE_CONFIGURED,
};

enum ep_cfg_op
{
    EP_NO_CHANGE,
    EP_ADD_ONLY,
    EP_DROP_ONLY,
    EP_DROP_AND_ADD
};

struct pcie_vhub_reg
{
    union
    {
        struct
        {
            volatile uint32_t read_event_en : 1;
            volatile uint32_t data_int_en : 1;
            volatile uint32_t data_int_clr : 1;
            volatile uint32_t data_int_raw : 1;
            volatile uint32_t data_overrun_int_en : 1;
            volatile uint32_t data_overrun_int_clr : 1;
            volatile uint32_t data_overrun_int_raw : 1;
            volatile uint32_t time_count_en : 1;
            volatile uint32_t time_count_clr : 1;
            volatile uint32_t event_num : 6;
            volatile uint32_t event_thres : 6;
            volatile uint32_t reserved : 10;
        };
        volatile uint32_t raw;
    } ctrl;
    volatile uint32_t clkdiv;
    union
    {
        struct
        {
            volatile uint32_t addr;
            volatile uint32_t data;
        };
        volatile uint64_t raw;
    } rxfifo;
};

struct xhci_cap_regs
{
    volatile uint32_t caplength;
    volatile uint32_t hcsparams1;
    volatile uint32_t hcsparams2;
    volatile uint32_t hcsparams3;
    volatile uint32_t hccparams1;
    volatile uint32_t dboff;
    volatile uint32_t rtsoff;
    volatile uint32_t hccparams2;
};

struct xhci_port_regs
{
    volatile uint32_t portsc;
    volatile uint32_t portpmsc;
    volatile uint32_t portli;
    volatile uint32_t porthlpmc; // usb2.0：If LPM is not supported (HLC = '0') then this register is reserved
}; 

struct xhci_op_regs
{
    volatile uint32_t command;
    volatile uint32_t status;
    volatile uint32_t page_size;
    volatile uint32_t reserved1;
    volatile uint32_t reserved2;
    volatile uint32_t dev_notification;
    volatile uint64_t cmd_ring;
    /* rsvd: offset 0x20-2F */
    volatile uint32_t reserved3[4];
    volatile uint64_t dcbaa_ptr;
    volatile uint32_t config_reg;
    /* rsvd: offset 0x3C-3FF */
    volatile uint32_t reserved4[241];
    volatile struct xhci_port_regs port[1]; // 400-13FFh
};

struct interrupter_regs
{
    volatile uint32_t iman;
    volatile uint32_t imod;
    volatile uint32_t erstsz;
    volatile uint32_t rsvd;
    volatile uint64_t erstba;
    volatile uint64_t erdp;
};

struct xhci_runtime_regs
{
    volatile uint32_t micro_frame;
    volatile uint32_t rsvd[7];
    volatile struct interrupter_regs ir_set[1];
};

struct ext_cap
{
    uint8_t cap_id;
    uint8_t next_cap_point;
};

struct ext_cap_legacy
{
    struct ext_cap cap;
    union
    {
        struct
        {
            uint16_t bios_owned_sem : 1;
            uint16_t rsvd : 7;
            uint16_t os_owned_sem : 1;
            uint16_t rsvd1 : 7;
        };
        uint16_t raw;
    } sync;
    uint32_t leg_ctl_sts;
};

struct ext_cap_sup_proto
{
    struct ext_cap cap;
    uint8_t minor_revision;
    uint8_t major_revision;
    uint32_t name_string;
    union
    {
        struct
        {
            uint32_t port_offect : 8;
            uint32_t port_count : 8;
            uint32_t protocol_defined : 12;
            uint32_t psic : 4;
        };
        uint32_t raw;
    } port;
    uint32_t protocol_slot_type;
};

struct erst_entry
{
    uint64_t seg_addr;
    uint32_t seg_size;
    uint32_t rsvd;
};

struct ls_pcie_vusb_event
{
    size_t data;
    union
    {
        struct
        {
            uint32_t is_write : 1;
            uint32_t reg : 12;
        };
        uint32_t raw;
    } addr; 
};

struct xhci_trb
{
    uint32_t field[4];
};

struct xhci_slot_ctx
{
    uint32_t dev_info;
    uint32_t dev_info2;
    uint32_t tt_info;
    uint32_t dev_state;
    /* offset 0x10 to 0x1f reserved for HC internal use */
    uint32_t reserved[4];
};

struct xhci_ep_ctx
{
    uint32_t ep_info;
    uint32_t ep_info2;
    uint64_t deq;
    uint32_t tx_info;
    /* offset 0x14 - 0x1f reserved for HC internal use */
    uint32_t reserved[3];
};

struct xhci_input_control_ctx
{
    uint32_t drop_flags;
    uint32_t add_flags;
    uint32_t rsvd2[6];
};

struct xhci_device_ctx
{
    struct xhci_slot_ctx slot_ctx;
    struct xhci_ep_ctx ep_ctx[31];
};

struct xhci_input_ctx
{
    struct xhci_input_control_ctx control_ctx;
    struct xhci_slot_ctx slot_ctx;
    struct xhci_ep_ctx ep_ctx[31];
};

struct ls_pcie_vusb_ep_ring
{
    uint64_t first_trb;
    uint64_t deq_addr;
    uint8_t ccs;
};

struct ls_pcie_vusb_trb
{
    struct xhci_trb trb;
    uint64_t deq_addr;
    u32 buf_len;
    u32 filled;
    u8 td_size;
    u8 ccs;
    struct list_head node;
};

struct ls_pcie_vusb_td
{
    struct list_head trb_list;
    u32 trb_count;
    u32 total_len;
    u32 xfer_len;
    struct list_head node;
};

struct ls_pcie_vusb_ep
{
    struct ls_pcie_vusb_ep_ring ep_ring;
    enum xhci_ep_state state;
    struct list_head ep_req;
    struct ls_pcie_vusb_td *build_td;
    struct list_head td_list;
    u32 mps;
    u8 in;
};

struct ls_pcie_vusb_slot
{
    struct ls_pcie_vusb_ep ep[31];
    enum xhci_slot_state state;
    struct ls_pcie_vusb *vusb;
    spinlock_t lock;
    struct ls_pcie_vusb_port *port;
    uint8_t address;
    int16_t id;

    struct
    {
        uint64_t cpu_ctx;
        struct xhci_device_ctx *ctx;
        dma_addr_t dma;
    } dev;

    struct
    {
        uint64_t cpu_ctx;
        struct xhci_input_ctx *ctx;
        dma_addr_t dma;
    } input;

    struct
    {
        struct xhci_trb trb;
        uint64_t ep_deq;
        uint8_t ccs;
    } setup, data, status;
};

#define MAX_HC_SLOTS        256

struct xhci_dev_ctx_array
{
    uint64_t dev_ctx_ptrs[MAX_HC_SLOTS];
    dma_addr_t dma;
};

struct ls_pcie_vusb;

struct ls_pcie_vusb_port
{
    uint32_t portsc;
    struct ls_pcie_vusb_dev *vdev;
    spinlock_t lock;
};

enum ls_pcie_vusb_ops_type
{
    LS_PCIE_VUSB_OPS_SET_ADDRESS,
    LS_PCIE_VUSB_OPS_SETUP_PACKET,
    LS_PCIE_VUSB_OPS_RESET,
};

struct ls_pcie_vusb_ops_work
{
    struct work_struct work;
    struct ls_pcie_vusb *vusb;
    struct ls_pcie_vusb_dev *vdev;
    enum ls_pcie_vusb_ops_type type;
    uint8_t addr;
    uint8_t setup[8];
};

#define LS_PCIE_VUSB_PORT_NUM           (1)

struct ls_pcie_vusb
{
    struct platform_device *pdev;
    struct ls_pcie *pcie;
    struct pci_epf *epf;
    void __iomem *base;
    int irq;
    struct list_head reg_list;
    bool is_run;
    DECLARE_KFIFO(fifo, struct ls_pcie_vusb_event, 256);
    spinlock_t fifo_lock;
    struct ls_pcie_vusb_port port[LS_PCIE_VUSB_PORT_NUM];
    struct ls_pcie_vusb_slot slot[8];
    struct xhci_dev_ctx_array *dcbaa;
    uint64_t cpu_dcbaa;
    uint8_t addr_pool[128];
    struct workqueue_struct *slot_wq;
    wait_queue_head_t thread_wait;
    struct task_struct *thread;
    uint32_t command;
    uint32_t status;
    uint32_t iman;

    struct
    {
        void *trb;
        dma_addr_t trb_dma;
    } ep_cmd;

    struct
    {
        struct dma_chan *write;
        struct dma_chan *read;
        struct completion read_comp;
        struct completion write_comp;
    } dma_chan;

    struct
    {
        void *trb;
        dma_addr_t trb_dma;
        uint64_t addr;
        uint64_t deq_addr;
        uint8_t ccs;
    } crcr;

    struct
    {
        void *trb;
        dma_addr_t trb_dma;
        struct erst_entry entry;
        uint64_t seg_end;
        uint64_t deq_addr;
        uint64_t enq_addr;
        uint32_t erdp_low;
        uint8_t pcs;
    } event_ring;
};

static int ls_pcie_vusb_dev_attach(struct ls_pcie_vusb *vusb, uint8_t port);
static int ls_pcie_vusb_dev_detach(struct ls_pcie_vusb *vusb, uint8_t port);
static struct ls_pcie_vusb_slot *ls_pcie_vusb_slot_find(struct ls_pcie_vusb *vusb, uint8_t addr);

static inline uint8_t ep_addr_to_index(uint8_t ep_addr)
{
    uint8_t ep_num = ep_addr & USB_ENDPOINT_NUMBER_MASK;

    if (ep_num == 0)
        return 0;

    return (ep_num << 1) - 1 + !!(ep_addr & USB_ENDPOINT_DIR_MASK);
}

static int ls_pcie_vusb_event_put(struct ls_pcie_vusb *vusb, struct ls_pcie_vusb_event *event)
{
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&vusb->fifo_lock, flags);
    ret = kfifo_put(&vusb->fifo, *event);
    spin_unlock_irqrestore(&vusb->fifo_lock, flags);

    return ret;
}

int ls_pcie_vusb_connect_dev(struct ls_pcie_vusb_dev *dev)
{
    if (!dev || !dev->vusb)
        return -EINVAL;

    struct ls_pcie_vusb *vusb = dev->vusb;

    if (vusb->is_run == false)
        return -EBUSY;

    struct ls_pcie_vusb_event event;
    event.addr.is_write = 1;
    event.addr.reg = LS_PCOE_VUSB_PORT_CHANGE;
    event.data = dev->port_idx << 1 | 1;
    ls_pcie_vusb_event_put(vusb, &event);
    wake_up(&vusb->thread_wait);

    return 0;
}
EXPORT_SYMBOL(ls_pcie_vusb_connect_dev);

int ls_pcie_vusb_disconnect_dev(struct ls_pcie_vusb_dev *dev)
{
    if (!dev || !dev->vusb)
        return -EINVAL;

    struct ls_pcie_vusb *vusb = dev->vusb;

    if (vusb->is_run == false)
        return 0;

    struct ls_pcie_vusb_event event;
    event.addr.is_write = 1;
    event.addr.reg = LS_PCOE_VUSB_PORT_CHANGE;
    event.data = dev->port_idx << 1 | 0;
    ls_pcie_vusb_event_put(vusb, &event);
    wake_up(&vusb->thread_wait);

    return 0;
}
EXPORT_SYMBOL(ls_pcie_vusb_disconnect_dev);

int ls_pcie_vusb_add_dev(struct ls_pcie_vusb_dev *dev, struct ls_pcie_vusb_ops *ops)
{
    struct ls_pcie_vusb *vusb = dev->vusb;

    for (int i = 0; i < ARRAY_SIZE(vusb->port); i++) {
        spin_lock(&vusb->port[i].lock);
        if (vusb->port[i].vdev == NULL) {
            vusb->port[i].vdev = dev;
            dev->ops = ops;
            dev->port_idx = i;
            dev->dma_dev = vusb->pcie->pci.dev;
            spin_unlock(&vusb->port[i].lock);
            return 0;
        }
        spin_unlock(&vusb->port[i].lock);
    }

    return -1;
}
EXPORT_SYMBOL(ls_pcie_vusb_add_dev);

int ls_pcie_vusb_delete_dev(struct ls_pcie_vusb_dev *dev)
{
    struct ls_pcie_vusb *vusb = dev->vusb;

    if (!vusb)
        return -EINVAL;

    for (int i = 0; i < ARRAY_SIZE(vusb->port); i++) {
        spin_lock(&vusb->port[i].lock);
        if (vusb->port[i].vdev == dev) {
            vusb->port[i].vdev = NULL;
            dev->ops = NULL;
            spin_unlock(&vusb->port[i].lock);
            return 0;
        }
        spin_unlock(&vusb->port[i].lock);
    }

    return -ENOENT;
}
EXPORT_SYMBOL(ls_pcie_vusb_delete_dev);

int ls_pcie_vusb_req_cancel(struct ls_pcie_vusb_ep_req *req)
{
    struct ls_pcie_vusb_slot *slot = ls_pcie_vusb_slot_find(req->vusb, req->addr);
    uint8_t ep_index;
    struct ls_pcie_vusb_ep *ep;
    struct ls_pcie_vusb_ep_req *cur, *tmp;

    if (!slot)
        return -EINVAL;

    ep_index = ep_addr_to_index(req->ep);
    if (ep_index >= ARRAY_SIZE(slot->ep))
        return -EINVAL;
    ep = &slot->ep[ep_index];

    spin_lock(&slot->lock);
    list_for_each_entry_safe(cur, tmp, &ep->ep_req, node) {
        if (cur == req) {
            list_del_init(&req->node);
            req->status = -ECANCELED;
            spin_unlock(&slot->lock);
            return 0;
        }
    }
    spin_unlock(&slot->lock);

    return -ENOENT;
}
EXPORT_SYMBOL(ls_pcie_vusb_req_cancel);

int ls_pcie_vusb_req_submit(struct ls_pcie_vusb_ep_req *req)
{
    struct ls_pcie_vusb_slot *slot = ls_pcie_vusb_slot_find(req->vusb, req->addr);
    uint8_t ep_index;
    struct ls_pcie_vusb_ep *ep;

    if (!slot)
        return -EINVAL;

    if (req->status == -EINPROGRESS)
        return -EBUSY;

    ep_index = ep_addr_to_index(req->ep);
    if (ep_index >= ARRAY_SIZE(slot->ep))
        return -EINVAL;
    ep = &slot->ep[ep_index];

    req->actual = 0;
    req->status = -EINPROGRESS;
    spin_lock(&slot->lock);

    /* check slot state */
    if (slot->state == SLOT_STATE_DISABLE) {
        spin_unlock(&slot->lock);
        req->status = 0;
        return -ESHUTDOWN;
    }

    list_add_tail(&req->node, &ep->ep_req);
    spin_unlock(&slot->lock);

    trace_req_submit(slot->id, ep_index, req->len, !!(req->ep & USB_ENDPOINT_DIR_MASK));

    struct ls_pcie_vusb_event event;
    event.addr.is_write = 1;
    event.addr.reg = LS_PCOE_VUSB_TRANSFER;
    event.data = ((u32)slot->id << 8) | ep_index;
    ls_pcie_vusb_event_put(slot->vusb, &event);
    wake_up(&slot->vusb->thread_wait);

    return 0;
}
EXPORT_SYMBOL(ls_pcie_vusb_req_submit);

static void ls_pcie_vusb_raise_irq(struct ls_pcie_vusb *vusb)
{
    pci_epc_raise_irq(vusb->epf->epc, vusb->epf->func_no, vusb->epf->vfunc_no, PCI_IRQ_MSI, 1);
}

static inline int ls_pcie_vusb_write_remote_mem(struct ls_pcie_vusb *vusb, phys_addr_t remote, phys_addr_t data, size_t len);

static int ls_pcie_vusb_post_event(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    struct xhci_op_regs *op = vusb->base + LS_PCOE_VUSB_XHCI_OP_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
    struct xhci_runtime_regs *rt = vusb->base + LS_PCOE_VUSB_XHCI_RT_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
    struct xhci_trb *vtrb = vusb->event_ring.trb;
    uint64_t next_enq;

    trace_post_event(XHCI_TRB_TYPE_GET(trb->field[3]), (trb->field[3] >> 16) & 0x1f,
                     (trb->field[2] >> 24) & COMP_CODE_MASK);

    next_enq = vusb->event_ring.enq_addr + sizeof(struct xhci_trb);
    if (next_enq >= vusb->event_ring.seg_end) {
        next_enq = vusb->event_ring.entry.seg_addr;
    }

    if (next_enq == (vusb->event_ring.deq_addr & ~0xF))
        return -ENOSPC;

    *vtrb = *trb;
    vtrb->field[3] &= cpu_to_le32(~0x1); 
    vtrb->field[3] |= vusb->event_ring.pcs;

    ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)vusb->event_ring.enq_addr, 
                                 (phys_addr_t)vusb->event_ring.trb_dma, sizeof(*trb));

    vusb->status |= USBSTS_EINT;
    op->status = vusb->status;
    vusb->iman |= IMAN_IP;
    rt->ir_set[0].iman = vusb->iman;
    rt->ir_set[0].erdp |= ERDP_EHB;

    vusb->event_ring.enq_addr = next_enq;
    if (vusb->event_ring.enq_addr == vusb->event_ring.entry.seg_addr) {
        vusb->event_ring.pcs ^= 1;
    }

    ls_pcie_vusb_raise_irq(vusb);
    vusb->iman &= ~IMAN_IP;
    rt->ir_set[0].iman = vusb->iman;

    return 0;
}

static inline int ls_pcie_vusb_read_remote_mem(struct ls_pcie_vusb *vusb, phys_addr_t remote, phys_addr_t data, size_t len)
{
    if (remote == 0 || len == 0) {
        dev_err(&vusb->pdev->dev, "vusb: DMA read from host addr %lld, len %zu\n", remote, len);
        dump_stack();
        return 0;
    }

    return ls_pcie_dma_read_write(vusb->dma_chan.read, data, remote, len, NULL, NULL, &vusb->dma_chan.read_comp, true);
}

static inline int ls_pcie_vusb_write_remote_mem(struct ls_pcie_vusb *vusb, phys_addr_t remote, phys_addr_t data, size_t len)
{
    if (remote == 0 || len == 0) {
        dev_err(&vusb->pdev->dev, "vusb: DMA write to host addr %lld, len %zu\n", remote, len);
        dump_stack();
        return 0;
    }

    return ls_pcie_dma_read_write(vusb->dma_chan.write, remote, data, len, NULL, NULL, &vusb->dma_chan.write_comp, false);
}

static void ls_pcie_vusb_halt_ep_and_sync_ctx(struct ls_pcie_vusb_slot *slot, uint8_t ep_index, 
                                                uint64_t deq_addr, uint8_t ccs)
{
    struct ls_pcie_vusb *vusb = slot->vusb;

    if (!slot->dev.ctx || ep_index >= ARRAY_SIZE(slot->ep))
        return;

    slot->dev.ctx->ep_ctx[ep_index].deq = deq_addr | ccs;
    slot->dev.ctx->ep_ctx[ep_index].ep_info &= ~EP_STATE_MASK;
    slot->dev.ctx->ep_ctx[ep_index].ep_info |= EP_STATE_HALTED;
    slot->ep[ep_index].state = EP_STATE_HALTED;
    ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)slot->dev.cpu_ctx, (phys_addr_t)slot->dev.dma, sizeof(struct xhci_device_ctx));
}

static void ls_pcie_vusb_ep_finish_req(struct ls_pcie_vusb_ep_req *req)
{
    req->status = 0;
    if (req->complete)
        req->complete(req);
}

static int ls_pcie_vusb_ctrl_ep_handle(struct ls_pcie_vusb_slot *slot, struct ls_pcie_vusb_ep_req *ep_req)
{
    struct ls_pcie_vusb *vusb = slot->vusb;
    uint8_t ep_index = ep_addr_to_index(ep_req->ep);
    struct usb_ctrlrequest *setup = (struct usb_ctrlrequest *)&slot->setup.trb;
    uint16_t setup_wlen = le16_to_cpu(setup->wLength);
    bool has_data_stage = setup_wlen != 0;
    bool stall = !!ep_req->stall;
    bool status_ioc = XHCI_TRB_IOC(slot->status.trb.field[3]);
    bool setup_in = !!(setup->bRequestType & USB_DIR_IN);

    ep_req->actual = 0;

    if (ep_index >= ARRAY_SIZE(slot->ep))
        return -EINVAL;

    trace_ep0_match(slot->id, stall, has_data_stage, setup_in, ep_req->len, setup_wlen);

    if (stall) {
        uint64_t stall_deq;
        uint8_t stall_ccs;
        struct xhci_trb event_trb = {0};

        if (has_data_stage) {
            stall_deq = slot->data.ep_deq;
            stall_ccs = slot->data.ccs;
        } else {
            stall_deq = slot->status.ep_deq;
            stall_ccs = slot->status.ccs;
        }
        ls_pcie_vusb_halt_ep_and_sync_ctx(slot, ep_index, stall_deq, stall_ccs);

        event_trb.field[0] = stall_deq;
        event_trb.field[2] = COMP_CODE(COMP_STALL_ERROR) | (has_data_stage ? (slot->data.trb.field[2] & 0x1ffff) : 0);
        event_trb.field[3] = XHCI_SLOT(slot->id) | XHCI_TRB_TYPE(TRB_TRANSFER) | XHCI_TRB_EP_ID(ep_index + 1);

        ls_pcie_vusb_post_event(slot->vusb, &event_trb);
        ls_pcie_vusb_ep_finish_req(ep_req);
        return 0;
    }

    if (has_data_stage) {
        struct xhci_trb event_trb = {0};
        uint32_t cpu_buffer_len = slot->data.trb.field[2] & 0x1ffff;
        uint32_t actual_len = MIN(ep_req->len, setup_wlen);
        uint64_t cpu_ddr = slot->data.trb.field[0] | ((uint64_t)slot->data.trb.field[1] << 32);
        bool isp = XHCI_TRB_ISP(slot->data.trb.field[3]);
        bool ioc = XHCI_TRB_IOC(slot->data.trb.field[3]);

        if (ep_req->buf) {
            if (XHCI_TRB_IDT(slot->data.trb.field[3])) {
                memcpy(ep_req->buf, &slot->data.trb.field[0], actual_len);
            } else if (setup_in) {
                ls_pcie_vusb_write_remote_mem(vusb, cpu_ddr, (phys_addr_t)ep_req->dma, actual_len);
            } else {
                ls_pcie_vusb_read_remote_mem(vusb, cpu_ddr, (phys_addr_t)ep_req->dma, actual_len);
            }
        }

        event_trb.field[0] = slot->data.ep_deq;
        event_trb.field[1] = 0;
        event_trb.field[2] = COMP_CODE(COMP_SUCCESS);

        if (actual_len < cpu_buffer_len && isp)
            event_trb.field[2] = COMP_CODE(COMP_SHORT_PACKET);
        else
            isp = false;

        event_trb.field[2] |= (cpu_buffer_len - actual_len);
        event_trb.field[3] = XHCI_SLOT(slot->id) | XHCI_TRB_TYPE(TRB_TRANSFER) | XHCI_TRB_EP_ID(ep_index + 1);

        ep_req->actual = actual_len;
        if (ioc || isp)
            ls_pcie_vusb_post_event(vusb, &event_trb);
    }

    if (status_ioc) {
        struct xhci_trb status_event = {0};
        status_event.field[0] = slot->status.ep_deq;
        status_event.field[1] = 0;
        status_event.field[2] = COMP_CODE(COMP_SUCCESS);
        status_event.field[3] = XHCI_SLOT(slot->id) | XHCI_TRB_TYPE(TRB_TRANSFER) | XHCI_TRB_EP_ID(ep_index + 1);
        ls_pcie_vusb_post_event(vusb, &status_event);
    }

    ls_pcie_vusb_ep_finish_req(ep_req);

    return 0;
}

static void ls_pcie_vusb_td_free(struct ls_pcie_vusb_td *td)
{
    struct ls_pcie_vusb_trb *trb, *tmp;

    if (!td)
        return;

    list_for_each_entry_safe(trb, tmp, &td->trb_list, node) {
        list_del_init(&trb->node);
        kfree(trb);
    }

    kfree(td);
}

static void ls_pcie_vusb_ep0_match(struct ls_pcie_vusb_slot *slot)
{
    struct ls_pcie_vusb_ep *ep = &slot->ep[0];

    spin_lock(&slot->lock);
    if (!list_empty(&ep->ep_req))
    {
        struct ls_pcie_vusb_ep_req *req = list_first_entry(&ep->ep_req, struct ls_pcie_vusb_ep_req, node);
        list_del_init(&req->node);
        spin_unlock(&slot->lock);
        ep->in = !!(req->ep & USB_DIR_IN);
        /* control endpoint handle */
        ls_pcie_vusb_ctrl_ep_handle(slot, req);
        spin_lock(&slot->lock);
    }
    spin_unlock(&slot->lock);
}

static void ls_pcie_vusb_epn_in_match(struct ls_pcie_vusb_slot *slot, struct ls_pcie_vusb_ep *ep)
{
    spin_lock(&slot->lock);
    if (list_empty(&ep->ep_req) || list_empty(&ep->td_list)) {
        spin_unlock(&slot->lock);
        return;
    }

    struct ls_pcie_vusb_ep_req *req = list_first_entry(&ep->ep_req, struct ls_pcie_vusb_ep_req, node);
    struct ls_pcie_vusb_td *td = list_first_entry(&ep->td_list, struct ls_pcie_vusb_td, node);
    /* remove req */
    list_del_init(&req->node);
    /* spin unlock */
    spin_unlock(&slot->lock);

    struct ls_pcie_vusb_trb *trb = list_first_entry(&td->trb_list, struct ls_pcie_vusb_trb, node);
    while (trb) {
        bool short_packet = false;
        bool trb_ok = false;

        phys_addr_t remote_buf_addr = trb->trb.field[0] | ((uint64_t)trb->trb.field[1] << 32);
        bool isp = XHCI_TRB_ISP(trb->trb.field[3]);
        bool ioc = XHCI_TRB_IOC(trb->trb.field[3]);
        size_t trb_remain = trb->buf_len - trb->filled;
        size_t req_remain = req->len - req->actual;
        size_t xfer_len = MIN(trb_remain, req_remain);
        u8 ep_index = ep_addr_to_index(req->ep);

        trace_epn_in_match(slot->id, ep_index, (u32)trb_remain, (u32)req_remain, (u32)xfer_len, req->stall);

        if (req->stall) {
            struct xhci_trb event_trb = {0};
            ls_pcie_vusb_halt_ep_and_sync_ctx(slot, ep_index, trb->deq_addr, trb->ccs);
            event_trb.field[0] = trb->deq_addr;
            event_trb.field[2] = COMP_CODE(COMP_STALL_ERROR) | (trb->buf_len - trb->filled);
            event_trb.field[3] = XHCI_SLOT(slot->id) | XHCI_TRB_TYPE(TRB_TRANSFER) | XHCI_TRB_EP_ID(ep_index + 1);
            ls_pcie_vusb_post_event(slot->vusb, &event_trb);

            list_del_init(&td->node);
            ls_pcie_vusb_td_free(td);
            ls_pcie_vusb_ep_finish_req(req);
            break;
        }

        if (req->len == 0) {
            struct xhci_trb event_trb = {0};

            event_trb.field[0] = trb->deq_addr;
            event_trb.field[2] = COMP_CODE(COMP_SHORT_PACKET) |
                                 (trb->buf_len - trb->filled);
            event_trb.field[3] = XHCI_SLOT(slot->id) |
                                 XHCI_TRB_TYPE(TRB_TRANSFER) |
                                 XHCI_TRB_EP_ID(ep_index + 1);

            if (isp || ioc)
                ls_pcie_vusb_post_event(slot->vusb, &event_trb);

            list_del_init(&td->node);
            ls_pcie_vusb_td_free(td);
            ls_pcie_vusb_ep_finish_req(req);
            break;
        }

        if (trb->td_size == 0) {
            if (xfer_len < trb_remain)
                short_packet = xfer_len % ep->mps;
        } else {
            if (xfer_len < trb_remain)
                short_packet = req->len % ep->mps;
        }

        ls_pcie_vusb_write_remote_mem(slot->vusb, remote_buf_addr + trb->filled, req->dma + req->actual, xfer_len);

        trb->filled += xfer_len;
        req->actual += xfer_len;

        trb_ok = (trb->filled == trb->buf_len);

        struct xhci_trb event_trb = {0};
        event_trb.field[0] = trb->deq_addr;
        event_trb.field[3] = XHCI_SLOT(slot->id) | XHCI_TRB_TYPE(TRB_TRANSFER) | XHCI_TRB_EP_ID(ep_index + 1);

        if (short_packet) {
            if (isp || ioc) {
                event_trb.field[2] = COMP_CODE(COMP_SHORT_PACKET) | (trb->buf_len - trb->filled);
                ls_pcie_vusb_post_event(slot->vusb, &event_trb);
            }
            list_del_init(&td->node);
            ls_pcie_vusb_td_free(td);
            ls_pcie_vusb_ep_finish_req(req);
            break;
        } else if (trb_ok) {
            if (ioc) {
                event_trb.field[2] = COMP_CODE(COMP_SUCCESS);
                ls_pcie_vusb_post_event(slot->vusb, &event_trb);
            }
            list_del_init(&trb->node);
            kfree(trb);

            if (list_empty(&td->trb_list)) {
                list_del_init(&td->node);
                ls_pcie_vusb_td_free(td);
                ls_pcie_vusb_ep_finish_req(req);
                break;
            }
        }

        if (req->actual == req->len) {
            ls_pcie_vusb_ep_finish_req(req);
            break;
        }

        trb = list_first_entry(&td->trb_list, struct ls_pcie_vusb_trb, node);
    }
}

static void ls_pcie_vusb_epn_out_match(struct ls_pcie_vusb_slot *slot, struct ls_pcie_vusb_ep *ep)
{
    spin_lock(&slot->lock);
    if (list_empty(&ep->ep_req) || list_empty(&ep->td_list)) {
        spin_unlock(&slot->lock);
        return;
    }

    struct ls_pcie_vusb_ep_req *req = list_first_entry(&ep->ep_req, struct ls_pcie_vusb_ep_req, node);
    struct ls_pcie_vusb_td *td = list_first_entry(&ep->td_list, struct ls_pcie_vusb_td, node);
    /* remove req */
    list_del_init(&req->node);
    /* spin unlock */
    spin_unlock(&slot->lock);

    struct ls_pcie_vusb_trb *trb = list_first_entry(&td->trb_list, struct ls_pcie_vusb_trb, node);
    while (trb) {
        bool trb_ok = false;

        phys_addr_t remote_buf_addr = trb->trb.field[0] | ((uint64_t)trb->trb.field[1] << 32);
        bool ioc = XHCI_TRB_IOC(trb->trb.field[3]);
        size_t trb_remain = trb->buf_len - trb->filled;
        size_t req_remain = req->len - req->actual;
        size_t xfer_len = MIN(trb_remain, req_remain);

        u8 ep_index = ep_addr_to_index(req->ep);

        trace_epn_out_match(slot->id, ep_index, (u32)trb_remain, (u32)req_remain, (u32)xfer_len, req->stall);

        if (req->stall) {
            struct xhci_trb event_trb = {0};

            ls_pcie_vusb_halt_ep_and_sync_ctx(slot, ep_index,
                                              trb->deq_addr, trb->ccs);

            event_trb.field[0] = trb->deq_addr;
            event_trb.field[2] = COMP_CODE(COMP_STALL_ERROR) |
                                 (trb->buf_len - trb->filled);
            event_trb.field[3] = XHCI_SLOT(slot->id) |
                                 XHCI_TRB_TYPE(TRB_TRANSFER) |
                                 XHCI_TRB_EP_ID(ep_index + 1);
            ls_pcie_vusb_post_event(slot->vusb, &event_trb);

            list_del_init(&td->node);
            ls_pcie_vusb_td_free(td);
            ls_pcie_vusb_ep_finish_req(req);
            break;
        }

        if (XHCI_TRB_IDT(trb->trb.field[3]))
            memcpy(req->buf + req->actual, (u8 *)&trb->trb.field[0] + trb->filled, xfer_len);
        else
            ls_pcie_vusb_read_remote_mem(slot->vusb, remote_buf_addr + trb->filled, req->dma + req->actual, xfer_len);
        trb->filled += xfer_len;
        req->actual += xfer_len;

        trb_ok = (trb->filled == trb->buf_len);

        struct xhci_trb event_trb = {0};
        event_trb.field[0] = trb->deq_addr;
        event_trb.field[3] = XHCI_SLOT(slot->id) |
                             XHCI_TRB_TYPE(TRB_TRANSFER) |
                             XHCI_TRB_EP_ID(ep_index + 1);

        if (trb_ok) {
            if (ioc) {
                event_trb.field[2] = COMP_CODE(COMP_SUCCESS);
                ls_pcie_vusb_post_event(slot->vusb, &event_trb);
            }

            list_del_init(&trb->node);
            kfree(trb);

            if (list_empty(&td->trb_list)) {
                bool out_short = ep->mps && (td->total_len % ep->mps);
                list_del_init(&td->node);
                ls_pcie_vusb_td_free(td);
                if (out_short || req->actual == req->len) {
                    ls_pcie_vusb_ep_finish_req(req);
                } else {
                    ls_pcie_vusb_ep_finish_req(req);
                }
                break;
            }
        }

        if (req->actual == req->len) {
            ls_pcie_vusb_ep_finish_req(req);
            break;
        }
        trb = list_first_entry(&td->trb_list, struct ls_pcie_vusb_trb, node);
    }
}

static void ls_pcie_vusb_epn_match(struct ls_pcie_vusb_slot *slot, struct ls_pcie_vusb_ep *ep)
{
    if (ep->in) {
        ls_pcie_vusb_epn_in_match(slot, ep);
    } else {
        ls_pcie_vusb_epn_out_match(slot, ep);
    }
}

static void ls_pcie_vusb_ops_work_handler(struct work_struct *work)
{
    struct ls_pcie_vusb_ops_work *ops_work = container_of(work, struct ls_pcie_vusb_ops_work, work);
    struct ls_pcie_vusb *vusb = ops_work->vusb;
    struct ls_pcie_vusb_dev *vdev = ops_work->vdev;
    int port_idx;

    if (!vusb || !vdev || !vdev->ops)
        goto out;

    port_idx = vdev->port_idx;
    if (port_idx < 0 || port_idx >= ARRAY_SIZE(vusb->port))
        goto out;

    if (vusb->port[port_idx].vdev != vdev)
        goto out;

    switch (ops_work->type) {
    case LS_PCIE_VUSB_OPS_SET_ADDRESS:
        if (vdev->ops->set_address)
            vdev->ops->set_address(vdev, ops_work->addr);
        break;
    case LS_PCIE_VUSB_OPS_SETUP_PACKET:
        if (vdev->ops->setup_packet)
            vdev->ops->setup_packet(vdev, ops_work->setup, ops_work->addr);
        break;
    case LS_PCIE_VUSB_OPS_RESET:
        if (vdev->ops->reset)
            vdev->ops->reset(vdev);
        break;
    default:
        break;
    }

out:
    kfree(ops_work);
}

static int ls_pcie_vusb_queue_ops(struct ls_pcie_vusb *vusb, struct ls_pcie_vusb_ops_work *ops_work)
{
    INIT_WORK(&ops_work->work, ls_pcie_vusb_ops_work_handler);
    ops_work->vusb = vusb;
    queue_work(vusb->slot_wq, &ops_work->work);

    return 0;
}

static int ls_pcie_vusb_queue_set_address(struct ls_pcie_vusb *vusb, struct ls_pcie_vusb_dev *vdev, uint8_t addr)
{
    struct ls_pcie_vusb_ops_work *ops_work;

    ops_work = kzalloc(sizeof(*ops_work), GFP_KERNEL);
    if (!ops_work) {
        dev_err(&vusb->pdev->dev, "kzalloc fail!!");
        return -ENOMEM;
    }

    ops_work->vdev = vdev;
    ops_work->type = LS_PCIE_VUSB_OPS_SET_ADDRESS;
    ops_work->addr = addr;

    return ls_pcie_vusb_queue_ops(vusb, ops_work);
}

static int ls_pcie_vusb_queue_setup_packet(struct ls_pcie_vusb *vusb, struct ls_pcie_vusb_dev *vdev,
                                            uint8_t addr, const uint8_t *setup)
{
    struct ls_pcie_vusb_ops_work *ops_work;

    ops_work = kzalloc(sizeof(*ops_work), GFP_KERNEL);
    if (!ops_work)
        return -ENOMEM;

    ops_work->vdev = vdev;
    ops_work->type = LS_PCIE_VUSB_OPS_SETUP_PACKET;
    ops_work->addr = addr;
    if (setup)
        memcpy(ops_work->setup, setup, sizeof(ops_work->setup));

    return ls_pcie_vusb_queue_ops(vusb, ops_work);
}

static int ls_pcie_vusb_queue_reset(struct ls_pcie_vusb *vusb, struct ls_pcie_vusb_dev *vdev)
{
    struct ls_pcie_vusb_ops_work *ops_work;

    ops_work = kzalloc(sizeof(*ops_work), GFP_KERNEL);
    if (!ops_work)
        return -ENOMEM;

    ops_work->vdev = vdev;
    ops_work->type = LS_PCIE_VUSB_OPS_RESET;

    return ls_pcie_vusb_queue_ops(vusb, ops_work);
}

static struct ls_pcie_vusb_slot *ls_pcie_vusb_slot_find(struct ls_pcie_vusb *vusb, uint8_t addr)
{
    if (addr == 0xff)
        return NULL;

    for (int i = 1; i < ARRAY_SIZE(vusb->slot); i++) {
        if (vusb->slot[i].vusb == vusb &&
            vusb->slot[i].state != SLOT_STATE_DISABLE &&
            vusb->slot[i].address == addr)
            return &vusb->slot[i];
    }

    return NULL;
}

struct pcie_vusb_reg_info
{
    const char *name;
    void __iomem *base;
    uint32_t addr;
    uint32_t size;
    int (*write_reg)(struct pcie_vusb_reg_info *reg_info, uint32_t reg, size_t data);
    int (*read_reg)(struct pcie_vusb_reg_info *reg_info, uint32_t reg);
    struct list_head node;
    struct ls_pcie_vusb *vusb;
};

static int pcie_vusb_reg_ops_register(struct ls_pcie_vusb *vusb, struct pcie_vusb_reg_info *reg_info)
{
    reg_info->vusb = vusb;
    list_add_tail(&reg_info->node, &vusb->reg_list);
    return 0;
}

static int pcie_vusb_cap_reg_init(void __iomem *base)
{
    struct xhci_cap_regs *reg = base;

    reg->caplength = LS_PCOE_VUSB_XHCI_OP_OFFSET | 0x110 << 16; // interface version 1.1.0
    reg->hcsparams1 = 0x1000108; // 1 ports, 8 slot,  1 Interrupter Register set
    reg->hcsparams2 = 0x0000000f; // Max Scratchpad Buffers set 0
    reg->hcsparams3 = 0x00000000; // U1 Device Exit Latency、U2 Device Exit Latency
    reg->hccparams1 = 0x400000; // Extended Capabilities Pointer 0x40 << 2 = 0x100, support 32-bit Addressing Capability, 32bytes Context Size
    reg->hccparams2 = 0; // 0
    reg->dboff = LS_PCOE_VUSB_XHCI_DB_OFFSET; // doorbell 寄存器的偏移
    reg->rtsoff = LS_PCOE_VUSB_XHCI_RT_OFFSET; // run time 寄存器偏移

    return 0;
}

static int pcie_vusb_ext_reg_init(void __iomem *base)
{
    struct ext_cap_legacy *legacy = base;
    struct ext_cap_sup_proto *sup_proto = (struct ext_cap_sup_proto *)(base + 0x10);

    legacy->cap.cap_id = 1;
    legacy->cap.next_cap_point = 4; // 0x10
    legacy->sync.raw = 0;
    legacy->leg_ctl_sts = 0;

    sup_proto->cap.cap_id = 2;
    sup_proto->cap.next_cap_point = 0;

    // usb 2.0
    sup_proto->major_revision = 2;
    sup_proto->minor_revision = 0;
    sup_proto->name_string = 0x20425355; // usb 

    sup_proto->port.port_offect = 1;
    sup_proto->port.port_count = 1;
    sup_proto->port.psic = 0;
    sup_proto->port.protocol_defined = 0x0; // not support hardware lpm
    sup_proto->protocol_slot_type = 0; // USB3 or USB2 xHCI Supported Protocol Capability shall be set to 0

    return 0;
}

static int pcie_vusb_op_reg_init(void __iomem *base)
{
    struct xhci_op_regs *reg = base;
    reg->page_size = 1;
    reg->command = 0;
    reg->status = USBSTS_HCH;
    reg->config_reg = 0;
    reg->dcbaa_ptr = 0;
    reg->port[0].portsc = PLS_RX_DETECT << PORTSC_PLS_SHIFT | PORTSC_PP;

    return 0;
}

static int pcie_vusb_rt_reg_init(void __iomem *base)
{
    struct xhci_runtime_regs *reg = base;

    reg->ir_set[0].iman = 0;
    reg->ir_set[0].imod = 0;

    reg->ir_set[0].erstsz = 0;
    reg->ir_set[0].erstba = 0;
    reg->ir_set[0].erdp = 0;

    return 0;
}

static int cap_reg_write(struct pcie_vusb_reg_info *reg_info, uint32_t reg, size_t data)
{
    trace_reg_event(reg, (u32)data);
    return 0;
}

static int cap_reg_read(struct pcie_vusb_reg_info *reg_info, uint32_t reg)
{
    return 0;
}

static struct pcie_vusb_reg_info cap_reg = {
    .name = "cap reg",
    .addr = LS_PCOE_VUSB_XHCI_CAP_OFFSET,
    .size = 0x20,
    .write_reg = cap_reg_write,
    .read_reg = cap_reg_read
};

static void ls_pcie_vusb_addr_free(struct ls_pcie_vusb *vusb, uint8_t addr);
static void ls_pcie_vusb_flush_slot(struct ls_pcie_vusb_slot *slot, int status);

static int pcie_vusb_reset(struct ls_pcie_vusb *vusb, struct xhci_op_regs *regs)
{
    struct xhci_runtime_regs *rt = vusb->base + LS_PCOE_VUSB_XHCI_RT_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;

    for (int i = 0; i < ARRAY_SIZE(vusb->slot); i++)
        ls_pcie_vusb_flush_slot(&vusb->slot[i], -ESHUTDOWN);

    dev_info(&vusb->pdev->dev,"pcie_vusb_reset\n");
    vusb->is_run = true;
    vusb->command = 0;
    vusb->status = USBSTS_HCH;

    vusb->crcr.addr = 0;
    vusb->crcr.deq_addr = 0;
    vusb->crcr.ccs = 1;

    vusb->event_ring.enq_addr = 0;
    vusb->event_ring.deq_addr = 0;
    vusb->event_ring.seg_end = 0;
    vusb->event_ring.pcs = 1;

    regs->status = USBSTS_HCH;
    regs->dcbaa_ptr = 0;
    regs->config_reg = 0;
    regs->dev_notification = 0;
    regs->cmd_ring = 0;
    regs->port[0].portsc = PLS_RX_DETECT << PORTSC_PLS_SHIFT | PORTSC_PP;

    /* reset runtime registers */
    rt->micro_frame = 0;
    vusb->iman = 0;
    rt->ir_set[0].iman = 0;
    rt->ir_set[0].imod = 0;
    rt->ir_set[0].erstsz = 0;
    rt->ir_set[0].erstba = 0;
    rt->ir_set[0].erdp = 0;

    /* clear command reg, wakeup xhci host driver */
    regs->command = 0;

    return 0;
}

static int pcie_vusb_run(struct ls_pcie_vusb *vusb, struct xhci_op_regs *regs)
{
    dev_info(&vusb->pdev->dev,"pcie_vusb_run\n");
    /* update reg */
    vusb->status &= ~USBSTS_HCH;
    regs->status = vusb->status;
    vusb->is_run = true;

    return 0;
}

static int pcie_vusb_stop(struct ls_pcie_vusb *vusb, struct xhci_op_regs *regs)
{
    dev_info(&vusb->pdev->dev,"pcie_vusb_stop\n");
    vusb->is_run = false;
    vusb->status |= USBSTS_HCH;
    regs->status = vusb->status;
    for (int i = 0; i < ARRAY_SIZE(vusb->slot); i++)
        ls_pcie_vusb_flush_slot(&vusb->slot[i], -ESHUTDOWN);
    return 0;
}

static int op_cmd_reg_handle(struct pcie_vusb_reg_info *reg_info, uint32_t data)
{
    struct xhci_op_regs *regs = reg_info->base;
    struct ls_pcie_vusb *vusb = reg_info->vusb;
    uint32_t old_cmd = vusb->command;

    if (data & USBCMD_HCRST) {
        pcie_vusb_reset(reg_info->vusb, regs);
        return 0;
    }

    vusb->command = data & (USBCMD_RS | USBCMD_INTE | USBCMD_HSEE | USBCMD_EWE);

    if ((data & USBCMD_RS) && !(old_cmd & USBCMD_RS)) {
        pcie_vusb_run(vusb, regs);
    } else if (!(data & USBCMD_RS) && (old_cmd & USBCMD_RS)) {
        pcie_vusb_stop(vusb, regs);
    }

    return 0;
}

static int op_reg_write(struct pcie_vusb_reg_info *reg_info, uint32_t reg, size_t data)
{
    struct xhci_op_regs *regs = reg_info->base;
    struct ls_pcie_vusb *vusb = reg_info->vusb;

    trace_reg_event(reg, (u32)data);

    switch (reg)
    {
    case 0x80:
        op_cmd_reg_handle(reg_info, data);
        break;
    case 0x84:
        vusb->status &= ~(data & (USBSTS_HSE | USBSTS_PCD | USBSTS_SRE));
        if ((data & USBSTS_EINT) && vusb->event_ring.enq_addr == vusb->event_ring.deq_addr)
            vusb->status &= ~USBSTS_EINT;
        regs->status = vusb->status;
        break;
    case 0x88:
        break;
    case 0x94:
        break;
    case 0x98:
        break;
    case 0x9c:
        vusb->crcr.ccs = regs->cmd_ring & 0x1;
        vusb->crcr.addr = regs->cmd_ring & ~0x3fULL;
        vusb->crcr.deq_addr = vusb->crcr.addr;
        regs->cmd_ring = vusb->crcr.addr | vusb->crcr.ccs;
        break;
    case 0xb0:
        break;
    case 0xb4:
        vusb->cpu_dcbaa = regs->dcbaa_ptr & ~0x3fULL;
        dev_info(&vusb->pdev->dev, "dcbaap %llx, reg %llx\n", (uint64_t)vusb->cpu_dcbaa, regs->dcbaa_ptr);
        break;
    case 0xb8:
        break;
    }
    return 0;
}

static int op_reg_read(struct pcie_vusb_reg_info *reg_info, uint32_t reg)
{
    return 0;
}

static struct pcie_vusb_reg_info op_reg = {
    .name = "op reg",
    .addr = LS_PCOE_VUSB_XHCI_OP_OFFSET,
    .size = 0x3c,
    .write_reg = op_reg_write,
    .read_reg = op_reg_read
};

static int ls_pcie_vusb_port_reset(struct pcie_vusb_reg_info *reg_info, uint8_t port_idx)
{
    struct xhci_op_regs *regs = reg_info->base;
    struct ls_pcie_vusb *vusb = reg_info->vusb;
    uint32_t portsc;

    if (vusb->port[port_idx].vdev &&
        vusb->port[port_idx].vdev->ops &&
        vusb->port[port_idx].vdev->ops->reset) {
        ls_pcie_vusb_queue_reset(vusb, vusb->port[port_idx].vdev);
    }

    portsc = vusb->port[port_idx].portsc;

    portsc &= ~PORTSC_PR;
    if (portsc & PORTSC_CCS) {
        portsc &= ~(PORTSC_PLS_MASK << PORTSC_PLS_SHIFT);
        portsc |= PLS_U0 << PORTSC_PLS_SHIFT;
        portsc |= PORTSC_PED;
        portsc |= PORTSC_SPEED_HIGH;
    }

    if (!(portsc & PORTSC_PRC)) {
        portsc |= PORTSC_PRC;
        vusb->status |= USBSTS_PCD;
        regs->status = vusb->status;
    }

    vusb->port[port_idx].portsc = portsc;
    regs->port[port_idx].portsc = portsc;

    /* post event */
    struct xhci_trb trb = {0};
    trb.field[0] = (port_idx + 1) << 24;
    trb.field[1] = 0;
    trb.field[2] = COMP_CODE(COMP_SUCCESS);
    trb.field[3] = XHCI_TRB_TYPE(TRB_PORT_STATUS);

    ls_pcie_vusb_post_event(vusb, &trb);

    return 0;
}

static int port_reg_write(struct pcie_vusb_reg_info *reg_info, uint32_t reg, size_t data)
{
    struct xhci_op_regs *regs = reg_info->base;
    struct ls_pcie_vusb *vusb = reg_info->vusb;
    uint8_t port_idx;
    uint32_t portsc;
    uint32_t w1c_mask = PORTSC_PED | PORTSC_CSC | PORTSC_PEC | PORTSC_WRC |
                        PORTSC_OCC | PORTSC_PRC | PORTSC_PLC | PORTSC_CEC;
    uint32_t rw_mask = PORTSC_PP | (PORTSC_PIC_MASK << PORTSC_PIC_SHIFT) |
                       PORTSC_WCE | PORTSC_WDE | PORTSC_WOE;

    trace_reg_event(reg, (u32)data);

    switch (reg)
    {
    case 0x180:
        port_idx = 0;
        portsc = vusb->port[port_idx].portsc;

        /* PED is RW1CS: a 1->0 transition sets PEC. */
        if ((data & PORTSC_PED) && (portsc & PORTSC_PED))
            portsc |= PORTSC_PEC;

        /* W1C bits */
        portsc &= ~(data & w1c_mask);

        /* RW bits */
        portsc &= ~rw_mask;
        portsc |= (data & rw_mask);

        /* PLS write via LWS */
        if (data & PORTSC_LWS) {
            portsc &= ~(PORTSC_PLS_MASK << PORTSC_PLS_SHIFT);
            portsc |= data & (PORTSC_PLS_MASK << PORTSC_PLS_SHIFT);
        }

        vusb->port[port_idx].portsc = portsc;
        /* update reg value */
        regs->port[port_idx].portsc = portsc;

        /* PR triggers reset handling */
        if (data & PORTSC_PR)
            ls_pcie_vusb_port_reset(reg_info, port_idx);
        break;
    case 0x184:
        break;
    case 0x188:
        break;
    case 0x18c:
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int port_reg_read(struct pcie_vusb_reg_info *reg_info, uint32_t reg)
{
    return 0;
}

static struct pcie_vusb_reg_info pt_reg = {
    .name = "port reg",
    .addr = LS_PCOE_VUSB_XHCI_PT_OFFSET,
    .size = 0x10, // one set
    .write_reg = port_reg_write,
    .read_reg = port_reg_read
};

static int rt_reg_write(struct pcie_vusb_reg_info *reg_info, uint32_t reg, size_t data)
{
    struct xhci_runtime_regs *regs = reg_info->base;
    struct ls_pcie_vusb *vusb = reg_info->vusb;

    trace_reg_event(reg, (u32)data);

    switch (reg)
    {
    case 0x1c0:
        vusb->iman &= ~(data & (IMAN_IP | IMAN_IE));
        vusb->iman |= data & IMAN_IE;
        regs->ir_set[0].iman = vusb->iman;
        break;
    case 0x1c4:
        /* 中断聚合，时间+水位线，不实现此功能 */
        break;
    case 0x1c8:
        break;
    case 0x1d0:
        break;
    case 0x1d4:
        /* Read Event Ring Segment Table */
        ls_pcie_vusb_read_remote_mem(vusb, regs->ir_set[0].erstba, (phys_addr_t)vusb->event_ring.trb_dma, sizeof(struct erst_entry));
        vusb->event_ring.entry = *(struct erst_entry *)vusb->event_ring.trb;
        vusb->event_ring.seg_end = vusb->event_ring.entry.seg_addr + vusb->event_ring.entry.seg_size * sizeof(struct xhci_trb);
        if (vusb->event_ring.enq_addr == 0) {
            vusb->event_ring.enq_addr = vusb->event_ring.entry.seg_addr;
            vusb->event_ring.deq_addr = vusb->event_ring.entry.seg_addr;
            regs->ir_set[0].erdp = vusb->event_ring.entry.seg_addr;
        }
        break;
    case 0x1d8:
        vusb->event_ring.erdp_low = data;
        break;
    case 0x1dc:
    {
        uint64_t erdp = ((uint64_t)data << 32) | vusb->event_ring.erdp_low;
        if (vusb->event_ring.erdp_low & ERDP_EHB)
            erdp &= ~(uint64_t)ERDP_EHB;

        regs->ir_set[0].erdp = erdp;
        vusb->event_ring.deq_addr = erdp & ~0xfULL;

        if (vusb->event_ring.enq_addr == 0)
            vusb->event_ring.enq_addr = vusb->event_ring.deq_addr;
        else if (vusb->event_ring.enq_addr == vusb->event_ring.deq_addr && vusb->status & USBSTS_EINT) {
            /* 事件都被 xhci 驱动消费掉了，清 EINT */
            struct xhci_op_regs *op = vusb->base + LS_PCOE_VUSB_XHCI_OP_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
            vusb->status &= ~USBSTS_EINT;
            op->status = vusb->status;
        }
        break;
    }
    }

    return 0;
}

static int rt_reg_read(struct pcie_vusb_reg_info *reg_info, uint32_t reg)
{
    return 0;
}

static struct pcie_vusb_reg_info rt_reg = {
    .name = "rt reg",
    .addr = LS_PCOE_VUSB_XHCI_RT_OFFSET,
    .size = 0x40,
    .write_reg = rt_reg_write,
    .read_reg = rt_reg_read
};

static int ls_pcie_vusb_alloc_slot(struct ls_pcie_vusb *vusb)
{
    for (int i = 1; i < ARRAY_SIZE(vusb->slot); i++) {
        if (vusb->slot[i].vusb == NULL) {
            vusb->slot[i].vusb = vusb;
            vusb->slot[i].id = i;
            vusb->slot[i].address = 0;
            return i;
        }
    }

    return -1;
}

static int ls_pcie_vusb_free_slot(struct ls_pcie_vusb *vusb, int slot_id)
{
    if (slot_id < 1 || slot_id >= (int)ARRAY_SIZE(vusb->slot))
        return -1;

    vusb->slot[slot_id].address = 0xff;
    vusb->slot[slot_id].vusb = NULL;
    vusb->slot[slot_id].id = -1;

    return 0;
}

struct ls_pcie_cmd_handler
{
    enum xhci_trb_type type;
    int (*handler)(struct ls_pcie_vusb *vusb, struct xhci_trb *trb);
};

static int enable_slot_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    int slot_idx = ls_pcie_vusb_alloc_slot(vusb);
    struct xhci_trb event_trb = {0};

    event_trb.field[0] = vusb->crcr.deq_addr;
    event_trb.field[1] = 0;

    if (slot_idx < 0) {
        event_trb.field[2] = COMP_CODE(COMP_NO_SLOTS_AVAILABLE_ERROR);
        event_trb.field[3] = XHCI_TRB_TYPE(TRB_COMPLETION);
    } else {
        vusb->slot[slot_idx].state = SLOT_STATE_ENABLE;
        event_trb.field[2] = COMP_CODE(COMP_SUCCESS);
        event_trb.field[3] = XHCI_SLOT(slot_idx) | XHCI_TRB_TYPE(TRB_COMPLETION);
    }
    trace_enable_slot(slot_idx);

    ls_pcie_vusb_post_event(vusb, &event_trb);

    return 0;
}

static int disable_slot_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    int slot_id = XHCI_SLOT_GET(trb->field[3]);
    struct ls_pcie_vusb_slot *slot;
    struct xhci_trb event_trb = {0};
    int i;

    event_trb.field[0] = vusb->crcr.deq_addr;
    event_trb.field[1] = 0;
    event_trb.field[3] = XHCI_SLOT(slot_id) | XHCI_TRB_TYPE(TRB_COMPLETION);

    if (slot_id < 1 || slot_id >= ARRAY_SIZE(vusb->slot)) {
        event_trb.field[2] = COMP_CODE(COMP_SLOT_NOT_ENABLED_ERROR);
        goto out;
    }

    slot = &vusb->slot[slot_id];

    trace_disable_slot(slot_id, slot->state);

    if (!slot->vusb || slot->state == SLOT_STATE_DISABLE) {
        event_trb.field[2] = COMP_CODE(COMP_SLOT_NOT_ENABLED_ERROR);
        goto out;
    }

    spin_lock(&slot->lock);
    slot->state = SLOT_STATE_DISABLE;
    spin_unlock(&slot->lock);

    ls_pcie_vusb_flush_slot(slot, -ECONNRESET);

    for (i = 0; i < ARRAY_SIZE(slot->ep); i++) {
        struct ls_pcie_vusb_ep *ep = &slot->ep[i];
        ep->ep_ring.deq_addr = 0;
        ep->ep_ring.ccs = 0;
        ep->state = EP_STATE_DISABLE;
    }

    ls_pcie_vusb_addr_free(vusb, slot->address);

    vusb->dcbaa->dev_ctx_ptrs[slot_id] = 0;

    if (slot->input.ctx) {
        dma_free_coherent(vusb->pcie->pci.dev, sizeof(struct xhci_input_ctx),
                          slot->input.ctx, slot->input.dma);
        slot->input.ctx = NULL;
    }

    if (slot->dev.ctx) {
        dma_free_coherent(vusb->pcie->pci.dev, sizeof(struct xhci_device_ctx),
                          slot->dev.ctx, slot->dev.dma);
        slot->dev.ctx = NULL;
    }

    ls_pcie_vusb_free_slot(vusb, slot_id);
    slot->address = 0xff;
    slot->port = NULL;

    event_trb.field[2] = COMP_CODE(COMP_SUCCESS);

out:
    ls_pcie_vusb_post_event(vusb, &event_trb);

    return 0;
}

static uint8_t ls_pcie_vusb_addr_alloc(struct ls_pcie_vusb *vusb)
{
    for (int i = 1; i < ARRAY_SIZE(vusb->addr_pool); i++) {
        if (vusb->addr_pool[i] == 0) {
            vusb->addr_pool[i] = i;
            return i;
        }
    }

    return 0;
}

static void ls_pcie_vusb_addr_free(struct ls_pcie_vusb *vusb, uint8_t addr)
{
    if (!addr || addr >= ARRAY_SIZE(vusb->addr_pool))
        return;
    vusb->addr_pool[addr] = 0;
}

static int addr_dev_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    int slot_id = XHCI_SLOT_GET(trb->field[3]);
    bool bsr = XHCI_BSR_GET(trb->field[3]);
    struct ls_pcie_vusb_slot *slot;
    struct ls_pcie_vusb_port *port;
    struct xhci_input_ctx *input_ctx;
    struct xhci_trb event_trb = {0};
    enum xhci_comp_code code = COMP_SUCCESS;
    uint8_t port_index;
    uint8_t addr = 0;
    int ret;

    event_trb.field[0] = vusb->crcr.deq_addr;
    event_trb.field[1] = 0;
    event_trb.field[3] = XHCI_SLOT(slot_id) | XHCI_TRB_TYPE(TRB_COMPLETION);

    if (slot_id < 1 || slot_id >= ARRAY_SIZE(vusb->slot)) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    slot = &vusb->slot[slot_id];

    if (!slot->vusb) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    /*
     * 4.6.5:
     *   BSR = 1 requires the slot to be in the Enabled state.
     *   BSR = 0 requires the slot to be in the Enabled or Default state.
     */
    if (bsr) {
        if (slot->state != SLOT_STATE_ENABLE)
            code = COMP_CONTEXT_STATE_ERROR;
    } else {
        if (slot->state != SLOT_STATE_ENABLE &&
            slot->state != SLOT_STATE_DEFAULT)
            code = COMP_CONTEXT_STATE_ERROR;
    }

    if (code != COMP_SUCCESS)
        goto out;

    /* Fetch DCBAA to obtain the Output Device Context pointer. */
    ret = ls_pcie_vusb_read_remote_mem(vusb, (phys_addr_t)vusb->cpu_dcbaa,
                                       (phys_addr_t)vusb->dcbaa->dma,
                                       sizeof(vusb->dcbaa->dev_ctx_ptrs));
    if (ret) {
        code = COMP_USB_TRANSACTION_ERROR;
        goto out;
    }

    slot->dev.cpu_ctx = vusb->dcbaa->dev_ctx_ptrs[slot_id];
    if (!slot->dev.cpu_ctx) {
        code = COMP_TRB_ERROR;
        goto out;
    }

    if (!slot->input.ctx) {
        slot->input.ctx = dma_alloc_coherent(vusb->pcie->pci.dev,
                                             sizeof(struct xhci_input_ctx),
                                             &slot->input.dma, GFP_KERNEL);
        if (!slot->input.ctx) {
            code = COMP_RESOURCE_ERROR;
            goto out;
        }
    }

    if (!slot->dev.ctx) {
        slot->dev.ctx = dma_alloc_coherent(vusb->pcie->pci.dev,
                                           sizeof(struct xhci_device_ctx),
                                           &slot->dev.dma, GFP_KERNEL);
        if (!slot->dev.ctx) {
            code = COMP_RESOURCE_ERROR;
            goto out;
        }
    }

    slot->input.cpu_ctx = trb->field[0] | ((uint64_t)trb->field[1] << 32);
    if (!slot->input.cpu_ctx) {
        code = COMP_PARAMETER_ERROR;
        goto out;
    }

    ret = ls_pcie_vusb_read_remote_mem(vusb, (phys_addr_t)slot->input.cpu_ctx,
                                       (phys_addr_t)slot->input.dma,
                                       sizeof(struct xhci_input_ctx));
    if (ret) {
        code = COMP_USB_TRANSACTION_ERROR;
        goto out;
    }

    input_ctx = slot->input.ctx;

    if ((input_ctx->control_ctx.add_flags & (BIT(0) | BIT(1))) !=
            (BIT(0) | BIT(1)) ||
        (input_ctx->control_ctx.add_flags & ~(BIT(0) | BIT(1))) ||
        input_ctx->control_ctx.drop_flags) {
        code = COMP_PARAMETER_ERROR;
        goto out;
    }

    port_index = XHCI_SLOT_CTX_PORT_IDX(input_ctx->slot_ctx.dev_info2);
    if (port_index < 1 || port_index > LS_PCIE_VUSB_PORT_NUM) {
        code = COMP_PARAMETER_ERROR;
        goto out;
    }

    port = &vusb->port[port_index - 1];
    if (!slot->port)
        slot->port = port;

    if (bsr) {
        slot->dev.ctx->slot_ctx = input_ctx->slot_ctx;
        slot->dev.ctx->ep_ctx[0] = input_ctx->ep_ctx[0];
        slot->dev.ctx->ep_ctx[0].ep_info &= ~EP_STATE_MASK;
        slot->dev.ctx->ep_ctx[0].ep_info |= EP_STATE_RUNNING;
        slot->ep[0].state = EP_STATE_RUNNING;
        slot->ep[0].ep_ring.deq_addr = input_ctx->ep_ctx[0].deq & ~0xf;
        slot->ep[0].ep_ring.ccs = input_ctx->ep_ctx[0].deq & 0x1;

        slot->dev.ctx->slot_ctx.dev_state &= ~(SLOT_STATE_MASK | 0xff);
        slot->dev.ctx->slot_ctx.dev_state |= SET_SLOT_STATE(SLOT_STATE_DEFAULT);
        slot->address = 0;
        slot->state = SLOT_STATE_DEFAULT;
    } else {
        addr = ls_pcie_vusb_addr_alloc(vusb);
        if (!addr) {
            code = COMP_RESOURCE_ERROR;
            goto out;
        }

        if (port->vdev && port->vdev->ops && port->vdev->ops->set_address) {
            ls_pcie_vusb_queue_set_address(vusb, port->vdev, addr);
        }

        slot->dev.ctx->slot_ctx = input_ctx->slot_ctx;
        slot->dev.ctx->ep_ctx[0] = input_ctx->ep_ctx[0];
        slot->dev.ctx->ep_ctx[0].ep_info &= ~EP_STATE_MASK;
        slot->dev.ctx->ep_ctx[0].ep_info |= EP_STATE_RUNNING;
        slot->ep[0].state = EP_STATE_RUNNING;
        slot->ep[0].ep_ring.deq_addr = input_ctx->ep_ctx[0].deq & ~0xf;
        slot->ep[0].ep_ring.ccs = input_ctx->ep_ctx[0].deq & 0x1;

        slot->dev.ctx->slot_ctx.dev_state &= ~(SLOT_STATE_MASK | 0xff);
        slot->dev.ctx->slot_ctx.dev_state |=
            SET_SLOT_STATE(SLOT_STATE_ADDRESSED) | addr;
        slot->address = addr;
        slot->state = SLOT_STATE_ADDRESSED;
    }
    trace_addr_dev(slot_id, slot->state, bsr, addr);

    ret = ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)slot->dev.cpu_ctx,
                                        (phys_addr_t)slot->dev.dma,
                                        sizeof(struct xhci_device_ctx));
    if (ret)
        code = COMP_USB_TRANSACTION_ERROR;

out:
    event_trb.field[2] = COMP_CODE(code);
    ls_pcie_vusb_post_event(vusb, &event_trb);

    return 0;
}

static void ls_pcie_vusb_ep_clear_tokens(struct ls_pcie_vusb_slot *slot, int ep_index);

static int reset_dev_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    int slot_id = XHCI_SLOT_GET(trb->field[3]);
    struct ls_pcie_vusb_slot *slot;
    struct xhci_trb event_trb = {0};
    enum xhci_comp_code code = COMP_SUCCESS;
    int ep_index;
    int ret;

    event_trb.field[0] = vusb->crcr.deq_addr & ~0xf;
    event_trb.field[1] = 0;
    event_trb.field[3] = XHCI_SLOT(slot_id) | XHCI_TRB_TYPE(TRB_COMPLETION);

    if (slot_id < 1 || slot_id >= ARRAY_SIZE(vusb->slot)) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    slot = &vusb->slot[slot_id];

    trace_reset_dev(slot_id, slot->state);

    if (!slot->vusb) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    if (!slot->dev.ctx) {
        code = COMP_TRB_ERROR;
        goto out;
    }

    if (!slot->dev.cpu_ctx) {
        code = COMP_TRB_ERROR;
        goto out;
    }

    if (slot->state != SLOT_STATE_ADDRESSED &&
        slot->state != SLOT_STATE_CONFIGURED) {
        code = COMP_CONTEXT_STATE_ERROR;
        goto out;
    }

    ls_pcie_vusb_flush_slot(slot, -ESHUTDOWN);

    ls_pcie_vusb_addr_free(vusb, slot->address);
    slot->address = 0;

    slot->dev.ctx->ep_ctx[0].ep_info &= ~EP_STATE_MASK;
    slot->dev.ctx->ep_ctx[0].ep_info |= EP_STATE_RUNNING;
    slot->ep[0].state = EP_STATE_RUNNING;

    for (ep_index = 1; ep_index < ARRAY_SIZE(slot->ep); ep_index++) {
        slot->dev.ctx->ep_ctx[ep_index].ep_info &= ~EP_STATE_MASK;
        slot->dev.ctx->ep_ctx[ep_index].ep_info |= EP_STATE_DISABLE;
        slot->dev.ctx->ep_ctx[ep_index].deq = 0;
        slot->ep[ep_index].state = EP_STATE_DISABLE;
    }

    slot->dev.ctx->slot_ctx.dev_state &= ~(SLOT_STATE_MASK | 0xff);
    slot->dev.ctx->slot_ctx.dev_state |= SET_SLOT_STATE(SLOT_STATE_DEFAULT);
    slot->dev.ctx->slot_ctx.dev_info &= ~SLOT_CTX_ENTR;
    slot->dev.ctx->slot_ctx.dev_info |= SET_SLOT_CTX_ENTR(1);
    slot->state = SLOT_STATE_DEFAULT;

    ret = ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)slot->dev.cpu_ctx,
                                        (phys_addr_t)slot->dev.dma,
                                        sizeof(struct xhci_device_ctx));
    if (ret)
        code = COMP_USB_TRANSACTION_ERROR;

out:
    event_trb.field[2] = COMP_CODE(code);
    ls_pcie_vusb_post_event(vusb, &event_trb);

    return 0;
}

static void ls_pcie_vusb_ep_clear_tokens(struct ls_pcie_vusb_slot *slot, int ep_index)
{
    struct ls_pcie_vusb_ep *ep = &slot->ep[ep_index];
    struct ls_pcie_vusb_td *td, *tmp;

    if (ep->build_td) {
        ls_pcie_vusb_td_free(ep->build_td);
        ep->build_td = NULL;
    }

    list_for_each_entry_safe(td, tmp, &ep->td_list, node) {
        list_del_init(&td->node);
        ls_pcie_vusb_td_free(td);
    }
}

static void ls_pcie_vusb_ep_apply_add(struct ls_pcie_vusb_slot *slot, int ep_index)
{
    slot->dev.ctx->ep_ctx[ep_index] = slot->input.ctx->ep_ctx[ep_index];
    slot->dev.ctx->ep_ctx[ep_index].ep_info &= ~EP_STATE_MASK;
    slot->dev.ctx->ep_ctx[ep_index].ep_info |= EP_STATE_RUNNING;
    slot->ep[ep_index].state = EP_STATE_RUNNING;
    slot->ep[ep_index].ep_ring.deq_addr = slot->input.ctx->ep_ctx[ep_index].deq & ~0xf;
    slot->ep[ep_index].ep_ring.ccs = slot->input.ctx->ep_ctx[ep_index].deq & 0x1;
    slot->ep[ep_index].mps = (slot->input.ctx->ep_ctx[ep_index].ep_info2 >> 16) & 0xffff;
    slot->ep[ep_index].in = ep_index ? !(ep_index & 1) : 0;
}

static void ls_pcie_vusb_ep_apply_drop(struct ls_pcie_vusb_slot *slot, int ep_index)
{
    slot->dev.ctx->ep_ctx[ep_index].ep_info &= ~EP_STATE_MASK;
    slot->dev.ctx->ep_ctx[ep_index].ep_info |= EP_STATE_DISABLE;
    slot->dev.ctx->ep_ctx[ep_index].deq = 0;
    slot->ep[ep_index].state = EP_STATE_DISABLE;
    slot->ep[ep_index].mps = 0;
    slot->ep[ep_index].in = 0;
    ls_pcie_vusb_ep_clear_tokens(slot, ep_index);
}

static enum xhci_comp_code ls_pcie_vusb_config_ep(struct ls_pcie_vusb_slot *slot)
{
    struct xhci_input_control_ctx *ctrl;
    uint32_t add_flags;
    uint32_t drop_flags;
    int ep_index;
    int last_ctx_entry = 1;
    bool ep_running = false;

    if (!slot || !slot->input.ctx || !slot->dev.ctx)
        return COMP_TRB_ERROR;

    ctrl = &slot->input.ctx->control_ctx;
    add_flags = ctrl->add_flags;
    drop_flags = ctrl->drop_flags;

    trace_config_ep(slot->id, slot->state, add_flags, drop_flags);

    /* check, A0 = 1, A1 = 0, D0 = 0, D1 = 0 */
    if (!(add_flags & BIT(0)) ||
        (drop_flags & BIT(0)) ||
        (drop_flags & BIT(1)) ||
        (add_flags & BIT(1)))
        return COMP_PARAMETER_ERROR;

    /* endpoint is run, notify trb error  */
    for (ep_index = 1; ep_index < 31; ep_index++) {
        uint32_t flag = BIT(ep_index + 1);
        bool add = !!(add_flags & flag);
        bool drop = !!(drop_flags & flag);

        if (add && !drop && slot->ep[ep_index].state != EP_STATE_DISABLE)
            return COMP_TRB_ERROR;
    }

    for (ep_index = 1; ep_index < 31; ep_index++) {
        uint32_t flag = BIT(ep_index + 1);
        bool add = !!(add_flags & flag);
        bool drop = !!(drop_flags & flag);

        if (drop && !add && slot->ep[ep_index].state != EP_STATE_DISABLE)
            ls_pcie_vusb_ep_apply_drop(slot, ep_index);
    }

    for (ep_index = 1; ep_index < 31; ep_index++) {
        uint32_t flag = BIT(ep_index + 1);
        bool add = !!(add_flags & flag);
        bool drop = !!(drop_flags & flag);

        if (add) {
            if (drop)
                ls_pcie_vusb_ep_clear_tokens(slot, ep_index);
            ls_pcie_vusb_ep_apply_add(slot, ep_index);
        }
    }

    /* A0 = 1: copy the Slot Context fields from the Input Context. */
    slot->dev.ctx->slot_ctx.dev_info = slot->input.ctx->slot_ctx.dev_info;
    slot->dev.ctx->slot_ctx.dev_info2 = slot->input.ctx->slot_ctx.dev_info2;
    slot->dev.ctx->slot_ctx.tt_info = slot->input.ctx->slot_ctx.tt_info;

    for (ep_index = 1; ep_index < 31; ep_index++) {
        if (slot->ep[ep_index].state == EP_STATE_RUNNING) {
            ep_running = true;
            last_ctx_entry = ep_index + 1; /* DCI */
        }
    }

    slot->dev.ctx->slot_ctx.dev_state &= ~SLOT_STATE_MASK;
    slot->dev.ctx->slot_ctx.dev_info &= ~SLOT_CTX_ENTR;

    if (ep_running) {
        slot->dev.ctx->slot_ctx.dev_state |= SET_SLOT_STATE(SLOT_STATE_CONFIGURED);
        slot->dev.ctx->slot_ctx.dev_info |= SET_SLOT_CTX_ENTR(last_ctx_entry);
        slot->state = SLOT_STATE_CONFIGURED;
    } else {
        slot->dev.ctx->slot_ctx.dev_state |= SET_SLOT_STATE(SLOT_STATE_ADDRESSED);
        slot->dev.ctx->slot_ctx.dev_info |= SET_SLOT_CTX_ENTR(1);
        slot->state = SLOT_STATE_ADDRESSED;
    }

    return COMP_SUCCESS;
}

static enum xhci_comp_code ls_pcie_vusb_deconfig_ep(struct ls_pcie_vusb_slot *slot)
{
    int ep_index;

    if (!slot || !slot->dev.ctx)
        return COMP_TRB_ERROR;

    trace_deconfig_ep(slot->id, slot->state);

    for (ep_index = 1; ep_index < 31; ep_index++) {
        struct ls_pcie_vusb_ep *ep = &slot->ep[ep_index];

        ls_pcie_vusb_ep_clear_tokens(slot, ep_index);

        slot->dev.ctx->ep_ctx[ep_index].ep_info &= ~EP_STATE_MASK;
        slot->dev.ctx->ep_ctx[ep_index].ep_info |= EP_STATE_DISABLE;
        slot->dev.ctx->ep_ctx[ep_index].deq = 0;
        ep->state = EP_STATE_DISABLE;
    }

    slot->dev.ctx->slot_ctx.dev_state &= ~SLOT_STATE_MASK;
    slot->dev.ctx->slot_ctx.dev_state |= SET_SLOT_STATE(SLOT_STATE_ADDRESSED);
    slot->dev.ctx->slot_ctx.dev_info &= ~SLOT_CTX_ENTR;
    slot->dev.ctx->slot_ctx.dev_info |= SET_SLOT_CTX_ENTR(1);
    slot->state = SLOT_STATE_ADDRESSED;

    return COMP_SUCCESS;
}

static int config_ep_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    int slot_id = XHCI_SLOT_GET(trb->field[3]);
    struct ls_pcie_vusb_slot *slot;
    uint8_t dc = XHCI_TRB_DC(trb->field[3]);
    struct xhci_trb event_trb = {0};
    enum xhci_comp_code code;
    int ret;

    if (slot_id <= 0 || slot_id >= ARRAY_SIZE(vusb->slot)) {
        event_trb.field[2] = COMP_CODE(COMP_SLOT_NOT_ENABLED_ERROR);
        goto out;
    }

    slot = &vusb->slot[slot_id];

    if (!slot->vusb) {
        event_trb.field[2] = COMP_CODE(COMP_SLOT_NOT_ENABLED_ERROR);
        goto out;
    }

    /* Fetch DCBAA to obtain the Output Device Context pointer. */
    ret = ls_pcie_vusb_read_remote_mem(vusb, (phys_addr_t)vusb->cpu_dcbaa,
                                       (phys_addr_t)vusb->dcbaa->dma,
                                       sizeof(vusb->dcbaa->dev_ctx_ptrs));
    if (ret) {
        event_trb.field[2] = COMP_CODE(COMP_USB_TRANSACTION_ERROR);
        goto out;
    }

    slot->dev.cpu_ctx = vusb->dcbaa->dev_ctx_ptrs[slot_id];
    if (!slot->dev.cpu_ctx || !slot->dev.ctx) {
        event_trb.field[2] = COMP_CODE(COMP_TRB_ERROR);
        goto out;
    }

    /*
     * DC = 0 references an Input Context. DC = 1 ignores the Input Context
     * Pointer entirely (4.6.6).
     */
    if (!dc) {
        slot->input.cpu_ctx = trb->field[0] | ((uint64_t)trb->field[1] << 32);
        if (!slot->input.cpu_ctx) {
            event_trb.field[2] = COMP_CODE(COMP_PARAMETER_ERROR);
            goto out;
        }

        if (!slot->input.ctx) {
            event_trb.field[2] = COMP_CODE(COMP_TRB_ERROR);
            goto out;
        }

        ret = ls_pcie_vusb_read_remote_mem(vusb,
                                           (phys_addr_t)slot->input.cpu_ctx,
                                           (phys_addr_t)slot->input.dma,
                                           sizeof(struct xhci_input_ctx));
        if (ret) {
            event_trb.field[2] = COMP_CODE(COMP_USB_TRANSACTION_ERROR);
            goto out;
        }
    }

    if (slot->state != SLOT_STATE_ADDRESSED &&
        slot->state != SLOT_STATE_CONFIGURED) {
        event_trb.field[2] = COMP_CODE(COMP_CONTEXT_STATE_ERROR);
        goto out;
    }

    if (dc) {
        code = ls_pcie_vusb_deconfig_ep(slot);
    } else {
        code = ls_pcie_vusb_config_ep(slot);
    }

    if (code == COMP_SUCCESS) {
        ret = ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)slot->dev.cpu_ctx,
                                            (phys_addr_t)slot->dev.dma,
                                            sizeof(struct xhci_device_ctx));
        if (ret)
            code = COMP_USB_TRANSACTION_ERROR;
    }

    event_trb.field[2] = COMP_CODE(code);

out:
    event_trb.field[0] = vusb->crcr.deq_addr & ~(0xf);
    event_trb.field[1] = 0;
    event_trb.field[3] = XHCI_SLOT(slot_id) | XHCI_TRB_TYPE(TRB_COMPLETION);
    ls_pcie_vusb_post_event(vusb, &event_trb);

    return 0;
}

static int stop_ring_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    int slot_id = XHCI_SLOT_GET(trb->field[3]);
    uint8_t ep_index = XHCI_TRB_EP(trb->field[3]) - 1;
    struct ls_pcie_vusb_slot *slot;
    struct xhci_trb event_trb = {0};
    enum xhci_comp_code code = COMP_SUCCESS;
    int ret;

    event_trb.field[0] = vusb->crcr.deq_addr & ~0xf;
    event_trb.field[1] = 0;
    event_trb.field[3] = XHCI_SLOT(slot_id) | XHCI_TRB_TYPE(TRB_COMPLETION) | XHCI_TRB_EP_ID(ep_index + 1);

    if (slot_id < 1 || slot_id >= ARRAY_SIZE(vusb->slot)) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    slot = &vusb->slot[slot_id];

    if (!slot->vusb) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    if (ep_index >= ARRAY_SIZE(slot->ep)) {
        code = COMP_CONTEXT_STATE_ERROR;
        goto out;
    }

    trace_stop_ring(slot->id, slot->state, ep_index, slot->ep[ep_index].state,
                    slot->ep[ep_index].ep_ring.deq_addr | slot->ep[ep_index].ep_ring.ccs);

    if (slot->state != SLOT_STATE_DEFAULT &&
        slot->state != SLOT_STATE_ADDRESSED &&
        slot->state != SLOT_STATE_CONFIGURED) {
        code = COMP_CONTEXT_STATE_ERROR;
        goto out;
    }

    if (!slot->dev.ctx || !slot->dev.cpu_ctx) {
        code = COMP_TRB_ERROR;
        goto out;
    }

    if (slot->ep[ep_index].state != EP_STATE_RUNNING) {
        code = COMP_CONTEXT_STATE_ERROR;
        goto out;
    }

    ls_pcie_vusb_ep_clear_tokens(slot, ep_index);

    /* Write final Dequeue Pointer + DCS, and set EP State to Stopped. */
    slot->dev.ctx->ep_ctx[ep_index].deq =
        slot->ep[ep_index].ep_ring.deq_addr | slot->ep[ep_index].ep_ring.ccs;
    slot->dev.ctx->ep_ctx[ep_index].ep_info &= ~EP_STATE_MASK;
    slot->dev.ctx->ep_ctx[ep_index].ep_info |= EP_STATE_STOPPED;
    slot->ep[ep_index].state = EP_STATE_STOPPED;

    ret = ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)slot->dev.cpu_ctx,
                                        (phys_addr_t)slot->dev.dma,
                                        sizeof(struct xhci_device_ctx));
    if (ret) {
        code = COMP_USB_TRANSACTION_ERROR;
    }

out:
    event_trb.field[2] = COMP_CODE(code);
    ls_pcie_vusb_post_event(vusb, &event_trb);

    return 0;
}

static int reset_ep_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    int slot_id = XHCI_SLOT_GET(trb->field[3]);
    uint8_t ep_index = XHCI_TRB_EP(trb->field[3]) - 1;
    struct xhci_trb event_trb = {0};
    struct ls_pcie_vusb_slot *slot;
    enum xhci_comp_code code = COMP_SUCCESS;
    int ret;

    event_trb.field[0] = vusb->crcr.deq_addr & ~0xf;
    event_trb.field[1] = 0;
    event_trb.field[3] = XHCI_SLOT(slot_id) | XHCI_TRB_TYPE(TRB_COMPLETION) | XHCI_TRB_EP_ID(ep_index + 1);

    if (slot_id < 1 || slot_id >= ARRAY_SIZE(vusb->slot)) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    slot = &vusb->slot[slot_id];

    if (!slot->vusb) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    if (ep_index >= ARRAY_SIZE(slot->ep)) {
        code = COMP_CONTEXT_STATE_ERROR;
        goto out;
    }

    trace_reset_ep(slot->id, slot->state, ep_index, slot->ep[ep_index].state,
                   XHCI_TRB_TSP(trb->field[3]));

    if (slot->state != SLOT_STATE_DEFAULT &&
        slot->state != SLOT_STATE_ADDRESSED &&
        slot->state != SLOT_STATE_CONFIGURED) {
        code = COMP_CONTEXT_STATE_ERROR;
        goto out;
    }

    if (slot->ep[ep_index].state != EP_STATE_HALTED) {
        code = COMP_CONTEXT_STATE_ERROR;
        goto out;
    }

    if (!slot->dev.ctx || !slot->dev.cpu_ctx) {
        code = COMP_TRB_ERROR;
        goto out;
    }

    slot->dev.ctx->ep_ctx[ep_index].ep_info &= ~EP_STATE_MASK;
    slot->dev.ctx->ep_ctx[ep_index].ep_info |= EP_STATE_STOPPED;
    slot->ep[ep_index].state = EP_STATE_STOPPED;

    if (XHCI_TRB_TSP(trb->field[3])) {
        ls_pcie_vusb_ep_clear_tokens(slot, ep_index);
        slot->ep[ep_index].ep_ring.deq_addr =
            slot->dev.ctx->ep_ctx[ep_index].deq & ~0xfULL;
        slot->ep[ep_index].ep_ring.ccs =
            slot->dev.ctx->ep_ctx[ep_index].deq & 0x1;
    }

    ret = ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)slot->dev.cpu_ctx,
                                        (phys_addr_t)slot->dev.dma,
                                        sizeof(struct xhci_device_ctx));
    if (ret)
        code = COMP_USB_TRANSACTION_ERROR;

out:
    event_trb.field[2] = COMP_CODE(code);
    ls_pcie_vusb_post_event(vusb, &event_trb);
    return 0;
}

static int set_deq_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    int slot_id = XHCI_SLOT_GET(trb->field[3]);
    uint8_t ep_index = XHCI_TRB_EP(trb->field[3]) - 1;
    uint16_t stream_id = (trb->field[2] >> 16) & 0xffff;
    struct xhci_trb event_trb = {0};
    struct ls_pcie_vusb_slot *slot;
    uint64_t deq;
    uint64_t deq_addr;
    uint8_t dcs;
    int ret = 0;

    /* command completion event */
    event_trb.field[0] = vusb->crcr.deq_addr & ~(0xf);
    event_trb.field[1] = 0;
    event_trb.field[3] = XHCI_SLOT(slot_id) | XHCI_TRB_TYPE(TRB_COMPLETION) | XHCI_TRB_EP_ID(ep_index + 1);

    if (slot_id <= 0 || slot_id >= ARRAY_SIZE(vusb->slot)) {
        event_trb.field[2] = COMP_CODE(COMP_SLOT_NOT_ENABLED_ERROR);
        goto out;
    }
    slot = &vusb->slot[slot_id];

    if (!slot->vusb) {
        event_trb.field[2] = COMP_CODE(COMP_SLOT_NOT_ENABLED_ERROR);
        goto out;
    }

    if (ep_index >= ARRAY_SIZE(slot->ep)) {
        event_trb.field[2] = COMP_CODE(COMP_CONTEXT_STATE_ERROR);
        goto out;
    }

    trace_set_deq(slot->id, slot->state, ep_index, slot->ep[ep_index].state, stream_id,
                  ((uint64_t)trb->field[1] << 32) | trb->field[0]);

    if (slot->state != SLOT_STATE_DEFAULT &&
        slot->state != SLOT_STATE_ADDRESSED &&
        slot->state != SLOT_STATE_CONFIGURED) {
        event_trb.field[2] = COMP_CODE(COMP_CONTEXT_STATE_ERROR);
        goto out;
    }

    if (slot->ep[ep_index].state != EP_STATE_STOPPED && slot->ep[ep_index].state != EP_STATE_ERROR) {
        event_trb.field[2] = COMP_CODE(COMP_CONTEXT_STATE_ERROR);
        goto out;
    }

    if (!slot->dev.ctx || !slot->dev.cpu_ctx) {
        event_trb.field[2] = COMP_CODE(COMP_TRB_ERROR);
        goto out;
    }

    /* Streams are not supported in this controller model yet. */
    if (stream_id != 0) {
        event_trb.field[2] = COMP_CODE(COMP_TRB_ERROR);
        goto out;
    }

    deq = ((uint64_t)trb->field[1] << 32) | trb->field[0];
    dcs = deq & 0x1;
    deq_addr = deq & ~(0xfULL);
    if (!deq_addr) {
        event_trb.field[2] = COMP_CODE(COMP_PARAMETER_ERROR);
        goto out;
    }

    /* Drop cached tokens referencing old ring state */
    ls_pcie_vusb_ep_clear_tokens(slot, ep_index);

    slot->ep[ep_index].ep_ring.deq_addr = deq_addr;
    slot->ep[ep_index].ep_ring.ccs = dcs;
    slot->dev.ctx->ep_ctx[ep_index].deq = deq_addr | dcs;

    ret = ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)slot->dev.cpu_ctx,
                                        (phys_addr_t)slot->dev.dma,
                                        sizeof(struct xhci_device_ctx));
    if (ret) {
        event_trb.field[2] = COMP_CODE(COMP_USB_TRANSACTION_ERROR);
        goto out;
    }

    event_trb.field[2] = COMP_CODE(COMP_SUCCESS);

out:
    ls_pcie_vusb_post_event(vusb, &event_trb);
    return 0;
}

static int noop_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    struct xhci_trb event_trb = {0};

    trace_cmd_noop(vusb->crcr.deq_addr);

    event_trb.field[0] = vusb->crcr.deq_addr & ~0xf;
    event_trb.field[1] = 0;
    event_trb.field[2] = COMP_CODE(COMP_SUCCESS);
    event_trb.field[3] = XHCI_TRB_TYPE(TRB_COMPLETION);

    ls_pcie_vusb_post_event(vusb, &event_trb);
    return 0;
}

static int eval_context_handler(struct ls_pcie_vusb *vusb, struct xhci_trb *trb)
{
    int slot_id = XHCI_SLOT_GET(trb->field[3]);
    struct ls_pcie_vusb_slot *slot;
    struct xhci_trb event_trb = {0};
    enum xhci_comp_code code = COMP_SUCCESS;
    uint32_t add_flags;
    uint32_t drop_flags;
    int ret;

    event_trb.field[0] = vusb->crcr.deq_addr & ~0xf;
    event_trb.field[1] = 0;
    event_trb.field[3] = XHCI_SLOT(slot_id) | XHCI_TRB_TYPE(TRB_COMPLETION);

    if (slot_id < 1 || slot_id >= ARRAY_SIZE(vusb->slot)) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    slot = &vusb->slot[slot_id];

    trace_eval_context(slot->id, slot->state);

    if (!slot->vusb) {
        code = COMP_SLOT_NOT_ENABLED_ERROR;
        goto out;
    }

    if (slot->state != SLOT_STATE_DEFAULT &&
        slot->state != SLOT_STATE_ADDRESSED &&
        slot->state != SLOT_STATE_CONFIGURED) {
        code = COMP_CONTEXT_STATE_ERROR;
        goto out;
    }

    if (!slot->dev.ctx || !slot->dev.cpu_ctx || !slot->input.ctx) {
        code = COMP_TRB_ERROR;
        goto out;
    }

    slot->input.cpu_ctx = trb->field[0] | ((uint64_t)trb->field[1] << 32);
    if (!slot->input.cpu_ctx) {
        code = COMP_PARAMETER_ERROR;
        goto out;
    }

    ret = ls_pcie_vusb_read_remote_mem(vusb, (phys_addr_t)slot->input.cpu_ctx,
                                       (phys_addr_t)slot->input.dma,
                                       sizeof(struct xhci_input_ctx));
    if (ret) {
        code = COMP_USB_TRANSACTION_ERROR;
        goto out;
    }

    add_flags = slot->input.ctx->control_ctx.add_flags;
    drop_flags = slot->input.ctx->control_ctx.drop_flags;

    /* Evaluate Context: all Drop Context flags shall be cleared to 0. */
    if (drop_flags) {
        code = COMP_PARAMETER_ERROR;
        goto out;
    }

    if (add_flags & BIT(0)) {
        slot->dev.ctx->slot_ctx.dev_info2 &= ~MAX_EXIT_LATENCY_MASK;
        slot->dev.ctx->slot_ctx.dev_info2 |= slot->input.ctx->slot_ctx.dev_info2 & MAX_EXIT_LATENCY_MASK;

        slot->dev.ctx->slot_ctx.dev_state &= ~INT_TARGET_MASK;
        slot->dev.ctx->slot_ctx.dev_state |= slot->input.ctx->slot_ctx.dev_state & INT_TARGET_MASK;
    }

    if (add_flags & BIT(1)) {
        slot->dev.ctx->ep_ctx[0].ep_info2 &= ~MAX_PACKET_MASK;
        slot->dev.ctx->ep_ctx[0].ep_info2 |= slot->input.ctx->ep_ctx[0].ep_info2 & MAX_PACKET_MASK;
        slot->ep[0].mps = (slot->input.ctx->ep_ctx[0].ep_info2 >> 16) & 0xffff;
    }

    ret = ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)slot->dev.cpu_ctx,
                                        (phys_addr_t)slot->dev.dma,
                                        sizeof(struct xhci_device_ctx));
    if (ret)
        code = COMP_USB_TRANSACTION_ERROR;

out:
    event_trb.field[2] = COMP_CODE(code);
    ls_pcie_vusb_post_event(vusb, &event_trb);
    return 0;
}

static const struct ls_pcie_cmd_handler cmd_handler[] = {
    {TRB_ENABLE_SLOT, enable_slot_handler},
    {TRB_DISABLE_SLOT, disable_slot_handler},
    {TRB_ADDR_DEV, addr_dev_handler},
    {TRB_RESET_DEV, reset_dev_handler},
    {TRB_CONFIG_EP, config_ep_handler},
    {TRB_EVAL_CONTEXT, eval_context_handler},
    {TRB_STOP_RING, stop_ring_handler},
    {TRB_RESET_EP, reset_ep_handler},
    {TRB_SET_DEQ, set_deq_handler},
    {TRB_CMD_NOOP, noop_handler},
};

static int ls_pcie_vusb_cmd_handle(struct ls_pcie_vusb *vusb)
{
    struct xhci_trb *trb = vusb->crcr.trb;

    while (1)
    {
        ls_pcie_vusb_read_remote_mem(vusb, vusb->crcr.deq_addr, 
                                    (phys_addr_t)vusb->crcr.trb_dma, 
                                    sizeof(struct xhci_trb));

        uint32_t control = le32_to_cpu(trb->field[3]);
        uint32_t type = XHCI_TRB_TYPE_GET(control);

        if (vusb->crcr.ccs != (control & TRB_CYCLE))
            break;

        /* link trb */
        if (type == TRB_LINK)
        {
            if (control & TRB_TC)
                vusb->crcr.ccs ^= 1;
            vusb->crcr.deq_addr = (uint64_t)trb->field[1] << 32 | (trb->field[0] & ~(0xf));
            continue;
        }

        /* process command */
        int ret = -1;
        bool handled = false;

        for (int i = 0; i < ARRAY_SIZE(cmd_handler); i++)
        {
            if (cmd_handler[i].type == type)
            {
                if (cmd_handler[i].handler)
                    ret = cmd_handler[i].handler(vusb, trb);
                handled = true;
                break;
            }
        }

        if (!handled || ret != 0)
        {
            trace_cmd_failed(type, ret, handled);

            struct xhci_trb event_trb = {0};
            event_trb.field[0] = vusb->crcr.deq_addr & ~0xf;
            event_trb.field[1] = 0;
            event_trb.field[2] = COMP_CODE(COMP_TRB_ERROR);
            event_trb.field[3] = XHCI_TRB_TYPE(TRB_COMPLETION);
            ls_pcie_vusb_post_event(vusb, &event_trb);
        }

        /* update deq point */
        vusb->crcr.deq_addr += sizeof(struct xhci_trb);
        /* Process only one command at a time to avoid command buildup in the FIFO, which could cause synchronization issues */
        break;
    }

    return 0;
}

struct ls_pcie_ep_handler
{
    enum xhci_trb_type type;
    int (*handler)(struct ls_pcie_vusb_slot *slot, struct xhci_trb *trb, uint8_t slot_id, uint8_t ep_num);
};

static int normal_trb_ep_handler(struct ls_pcie_vusb_slot *slot, struct xhci_trb *trb, uint8_t slot_id, uint8_t ep_num)
{
    struct ls_pcie_vusb_ep *ep = &slot->ep[ep_num];
    struct ls_pcie_vusb_td *td;
    struct ls_pcie_vusb_trb *single_trb;

    single_trb = kzalloc(sizeof(*single_trb), GFP_KERNEL);
    if (!single_trb)
        return -ENOMEM;

    td = ep->build_td;
    if (!td) {
        td = kzalloc(sizeof(*td), GFP_KERNEL);
        if (!td) {
            kfree(single_trb);
            return -ENOMEM;
        }

        INIT_LIST_HEAD(&td->node);
        INIT_LIST_HEAD(&td->trb_list);
        td->trb_count = 0;
        td->total_len = 0;
        td->xfer_len = 0;
        ep->build_td = td;
    }

    single_trb->trb = *trb;
    single_trb->deq_addr = ep->ep_ring.deq_addr;
    single_trb->buf_len = trb->field[2] & 0x1ffff;
    single_trb->filled = 0;
    single_trb->td_size = XHCI_TRB_TD_SIZE_GET(trb->field[2]);
    single_trb->ccs = ep->ep_ring.ccs;

    list_add_tail(&single_trb->node, &td->trb_list);
    td->trb_count++;
    td->total_len += single_trb->buf_len;

    if (!XHCI_TRB_CH(trb->field[3])) {
        td->xfer_len = td->total_len;
        ep->build_td = NULL;
        list_add_tail(&td->node, &ep->td_list);
    }

    trace_normal_trb(slot_id, ep_num, ep_num % 2, single_trb->buf_len,
            XHCI_TRB_CH(trb->field[3]), XHCI_TRB_ISP(trb->field[3]), XHCI_TRB_IOC(trb->field[3]));

    if (!XHCI_TRB_CH(trb->field[3]))
        ls_pcie_vusb_epn_match(slot, ep);

    return 0;
}

static int setup_stage_trb_ep_handler(struct ls_pcie_vusb_slot *slot, struct xhci_trb *trb, uint8_t slot_id, uint8_t ep_num)
{
    /* push gadget framework */
    slot->setup.trb = *trb;
    slot->setup.ep_deq = slot->ep[ep_num].ep_ring.deq_addr & ~(0xf);
    slot->setup.ccs = slot->ep[ep_num].ep_ring.ccs;
    uint32_t f0 = le32_to_cpu(trb->field[0]);
    uint32_t f1 = le32_to_cpu(trb->field[1]);
    trace_setup_stage(slot_id, ep_num, (le32_to_cpu(trb->field[3]) >> 16) & 0x3,
            f0 & 0xff, (f0 >> 8) & 0xff, f0 >> 16, f1 & 0xffff, f1 >> 16);

    return 0;
}

static int data_stage_trb_ep_handler(struct ls_pcie_vusb_slot *slot, struct xhci_trb *trb, uint8_t slot_id, uint8_t ep_num)
{
    slot->data.trb = *trb;
    slot->data.ep_deq = slot->ep[ep_num].ep_ring.deq_addr & ~(0xf);
    slot->data.ccs = slot->ep[ep_num].ep_ring.ccs;
    trace_data_stage(slot_id, ep_num, trb->field[2] & 0x1ffff, XHCI_TRB_DIR(trb->field[3]),
            XHCI_TRB_CH(trb->field[3]), XHCI_TRB_ISP(trb->field[3]), XHCI_TRB_IOC(trb->field[3]));

    return 0;
}

static int status_stage_trb_ep_handler(struct ls_pcie_vusb_slot *slot, struct xhci_trb *trb, uint8_t slot_id, uint8_t ep_num)
{
    slot->status.trb = *trb;
    slot->status.ep_deq = slot->ep[ep_num].ep_ring.deq_addr & ~(0xf);
    slot->status.ccs = slot->ep[ep_num].ep_ring.ccs;

    trace_status_stage(slot_id, ep_num, XHCI_TRB_DIR(trb->field[3]),
            XHCI_TRB_CH(trb->field[3]), XHCI_TRB_IOC(trb->field[3]));

    if (slot->port && slot->port->vdev && slot->port->vdev->ops &&
        slot->port->vdev->ops->setup_packet) {
        ls_pcie_vusb_queue_setup_packet(slot->vusb, slot->port->vdev, slot->address, (uint8_t *)&slot->setup.trb);
    }

    return 0;
}

static int isoch_trb_ep_handler(struct ls_pcie_vusb_slot *slot, struct xhci_trb *trb, uint8_t slot_id, uint8_t ep_num)
{
    return 0;
}

static int noop_trb_ep_handler(struct ls_pcie_vusb_slot *slot, struct xhci_trb *trb, uint8_t slot_id, uint8_t ep_num)
{
    return 0;
}

static const struct ls_pcie_ep_handler ep_handler[] = {
    {TRB_NORMAL, normal_trb_ep_handler},
    {TRB_SETUP, setup_stage_trb_ep_handler},
    {TRB_DATA, data_stage_trb_ep_handler},
    {TRB_STATUS, status_stage_trb_ep_handler},
    {TRB_ISOC, isoch_trb_ep_handler},
    {TRB_TR_NOOP, noop_trb_ep_handler},
};

static int ls_pcie_vusb_ep_handle(struct ls_pcie_vusb *vusb, uint8_t slot_id, uint32_t data)
{
    struct ls_pcie_vusb_slot *slot = &vusb->slot[slot_id];
    uint8_t ep_index = (data & 0xff) - 1;
    struct ls_pcie_vusb_ep *ep = &slot->ep[ep_index];
    struct xhci_trb *trb = vusb->ep_cmd.trb;

    if (ep_index >= 31 || !slot->dev.ctx)
        return -EINVAL;

    if (slot->state == SLOT_STATE_DISABLE)
        return -EINVAL;

    if (ep->state == EP_STATE_HALTED || ep->state == EP_STATE_DISABLE)
        return 0;

    if (ep->state == EP_STATE_STOPPED) {
        slot->dev.ctx->ep_ctx[ep_index].ep_info &= ~EP_STATE_MASK;
        slot->dev.ctx->ep_ctx[ep_index].ep_info |= EP_STATE_RUNNING;
        ep->state = EP_STATE_RUNNING;
    }

    while (1)
    {
        ls_pcie_vusb_read_remote_mem(vusb, ep->ep_ring.deq_addr,
                                    (phys_addr_t)vusb->ep_cmd.trb_dma,
                                    sizeof(struct xhci_trb));

        uint32_t control = le32_to_cpu(trb->field[3]);

        if (ep->ep_ring.ccs != (control & TRB_CYCLE))
            break;

        uint32_t type = XHCI_TRB_TYPE_GET(control);

        if (type == TRB_LINK)
        {
            if (control & TRB_TC)
                ep->ep_ring.ccs ^= 1;
            ep->ep_ring.deq_addr = (uint64_t)trb->field[1] << 32 | (trb->field[0] & ~(0xf));
            continue;
        }

        int ret = -1;
        bool handled = false;

        for (int i = 0; i < ARRAY_SIZE(ep_handler); i++)
        {
            if (ep_handler[i].type == type)
            {
                if (ep_handler[i].handler)
                    ret = ep_handler[i].handler(slot, trb, slot_id, ep_index);
                handled = true;
                break;
            }
        }

        if (!handled || ret != 0) {
            trace_ep_failed(slot_id, ep_index, type, ret, handled);

            struct xhci_trb event_trb = {0};
            ls_pcie_vusb_halt_ep_and_sync_ctx(slot, ep_index, ep->ep_ring.deq_addr, ep->ep_ring.ccs);
            event_trb.field[0] = ep->ep_ring.deq_addr;
            event_trb.field[2] = COMP_CODE(COMP_TRB_ERROR);
            event_trb.field[3] = XHCI_SLOT(slot_id) | XHCI_TRB_TYPE(TRB_TRANSFER) | XHCI_TRB_EP_ID(ep_index + 1);
            ls_pcie_vusb_post_event(vusb, &event_trb);
            break;
        }

        if (ep->state != EP_STATE_RUNNING)
            break;

        ep->ep_ring.deq_addr += sizeof(struct xhci_trb);

        /* process one doorbell transfer ring */
        if (ep_index && !XHCI_TRB_CH(control))
            break;
    }

    if (ep->state == EP_STATE_RUNNING) {
        slot->dev.ctx->ep_ctx[ep_index].deq = ep->ep_ring.deq_addr | ep->ep_ring.ccs;
        ls_pcie_vusb_write_remote_mem(vusb, (phys_addr_t)slot->dev.cpu_ctx, (phys_addr_t)slot->dev.dma,
                                      sizeof(struct xhci_device_ctx));
    }

    return 0;
}

static int db_reg_write(struct pcie_vusb_reg_info *reg_info, uint32_t reg, size_t data)
{
    unsigned int idx;

    if (reg < reg_info->addr ||
        reg >= (reg_info->addr + reg_info->size))
        return -EINVAL;

    idx = (reg - reg_info->addr) / sizeof(uint32_t);

    trace_doorbell(idx, (u32)data);

    if (idx == 0)
        ls_pcie_vusb_cmd_handle(reg_info->vusb);
    else if (idx < ARRAY_SIZE(reg_info->vusb->slot))
        ls_pcie_vusb_ep_handle(reg_info->vusb, idx, data);

    return 0;
}

static int db_reg_read(struct pcie_vusb_reg_info *reg_info, uint32_t reg)
{
    return 0;
}

static struct pcie_vusb_reg_info db_reg = {
    .name = "db reg",
    .addr = LS_PCOE_VUSB_XHCI_DB_OFFSET,
    .size = 0x20,
    .write_reg = db_reg_write,
    .read_reg = db_reg_read
};

static int usb_xfer_submit(struct pcie_vusb_reg_info *reg_info, uint32_t reg, size_t data)
{
    struct ls_pcie_vusb *vusb = reg_info->vusb;
    int slot_id = (data >> 8) & 0xff;
    int ep_index = data & 0xff;
    struct ls_pcie_vusb_slot *slot;

    if (slot_id < 1 || slot_id >= ARRAY_SIZE(vusb->slot))
        return -EINVAL;

    slot = &vusb->slot[slot_id];

    trace_xfer_submit(slot_id, (u32)slot->state, ep_index, (u32)slot->ep[ep_index].state);

    if (slot->vusb != vusb || slot->state == SLOT_STATE_DISABLE)
        return -EINVAL;

    if (ep_index < 0 || ep_index >= ARRAY_SIZE(slot->ep))
        return -EINVAL;

    if (slot->ep[ep_index].state != EP_STATE_RUNNING)
        return 0;

    if (ep_index == 0)
        ls_pcie_vusb_ep0_match(slot);
    else
        ls_pcie_vusb_epn_match(slot, &slot->ep[ep_index]);

    return 0;
}

static struct pcie_vusb_reg_info submit_reg = {
    .name = "usb xfer submit",
    .addr = LS_PCOE_VUSB_TRANSFER,
    .size = 0x1,
    .write_reg = usb_xfer_submit,
};

static int usb_port_change(struct pcie_vusb_reg_info *reg_info, uint32_t reg, size_t data)
{
    struct ls_pcie_vusb *vusb = reg_info->vusb;
    u16 port = data >> 1;
    bool attach = data & 1;

    trace_port_change(attach, port);

    if (attach) {
        ls_pcie_vusb_dev_attach(vusb, port);
    } else {
        ls_pcie_vusb_dev_detach(vusb, port);
    }

    return 0;
}

static struct pcie_vusb_reg_info port_update_reg = {
    .name = "usb port change",
    .addr = LS_PCOE_VUSB_PORT_CHANGE,
    .size = 0x1,
    .write_reg = usb_port_change,
};

static int et_reg_write(struct pcie_vusb_reg_info *reg_info, uint32_t reg, size_t data)
{
    struct ext_cap_legacy *legacy = reg_info->base;

    trace_reg_event(reg, (u32)data);

    if (reg == reg_info->addr) {
        /* cap_id 与 next 为 ro, 恢复为初始值  */
        legacy->cap.cap_id = 1;
        legacy->cap.next_cap_point = 4;
    } else if (reg == reg_info->addr + 0x4) {
        /* bit29-31 为 rw1c */
        legacy->leg_ctl_sts &= ~((uint32_t)data & 0xe0000000);
    }

    return 0;
}

static int et_reg_read(struct pcie_vusb_reg_info *reg_info, uint32_t reg)
{
    return 0;
}

static struct pcie_vusb_reg_info et_reg = {
    .name = "et reg",
    .addr = LS_PCOE_VUSB_XHCI_ET_OFFSET,
    .size = 0x20,
    .write_reg = et_reg_write,
    .read_reg = et_reg_read
};

static int ls_pcie_vusb_port_update(struct ls_pcie_vusb *vusb, uint8_t port, int attach)
{
    struct xhci_op_regs *op = vusb->base + LS_PCOE_VUSB_XHCI_OP_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
    uint32_t old_portsc;

    if (!vusb || port >= ARRAY_SIZE(vusb->port))
        return -EINVAL;

    old_portsc = vusb->port[port].portsc;
    vusb->port[port].portsc &= ~(PORTSC_CCS | PORTSC_PED |
                                 (PORTSC_PLS_MASK << PORTSC_PLS_SHIFT));
    vusb->port[port].portsc |= PORTSC_PP;

    if (attach) {
        vusb->port[port].portsc |= PORTSC_CCS;
        vusb->port[port].portsc |= (PLS_POLLING << PORTSC_PLS_SHIFT);
    } else {
        vusb->port[port].portsc |= (PLS_RX_DETECT << PORTSC_PLS_SHIFT);
    }

    if (!!(old_portsc & PORTSC_CCS) != !!(vusb->port[port].portsc & PORTSC_CCS))
        vusb->port[port].portsc |= PORTSC_CSC;
    if (!!(old_portsc & PORTSC_PED) != !!(vusb->port[port].portsc & PORTSC_PED))
        vusb->port[port].portsc |= PORTSC_PEC;

    op->port[port].portsc = vusb->port[port].portsc;

    if ((vusb->port[port].portsc & ~old_portsc) & (PORTSC_CSC | PORTSC_PEC)) {
        vusb->status |= USBSTS_PCD;
        op->status = vusb->status;
    }

    return 0;
}

static void ls_pcie_vusb_flush_slot(struct ls_pcie_vusb_slot *slot, int status)
{
    for (int i = 0; i < ARRAY_SIZE(slot->ep); i++) {
        struct ls_pcie_vusb_ep *ep = &slot->ep[i];
        struct ls_pcie_vusb_ep_req *req;

        ls_pcie_vusb_ep_clear_tokens(slot, i);

        spin_lock(&slot->lock);
        while (!list_empty(&ep->ep_req)) {
            req = list_first_entry(&ep->ep_req, struct ls_pcie_vusb_ep_req, node);
            list_del_init(&req->node);
            req->actual = 0;
            req->status = status;

            spin_unlock(&slot->lock);
            if (req->complete)
                req->complete(req);
            spin_lock(&slot->lock);
        }
        spin_unlock(&slot->lock);
    }
}

static int ls_pcie_vusb_dev_attach(struct ls_pcie_vusb *vusb, uint8_t port)
{
    struct xhci_trb trb = {0};
    int ret;

    if (!vusb || port >= ARRAY_SIZE(vusb->port))
        return -EINVAL;
    if (!vusb->port[port].vdev)
        return -ENODEV;
    if (vusb->port[port].portsc & PORTSC_CCS)
        return 0;

    trb.field[0] = (port + 1) << 24;
    trb.field[1] = 0;
    trb.field[2] = COMP_CODE(COMP_SUCCESS);
    trb.field[3] = TRB_PORT_STATUS << 10;

    ret = ls_pcie_vusb_port_update(vusb, port, 1);
    if (ret)
        return ret;
    ret = ls_pcie_vusb_post_event(vusb, &trb);
    if (ret)
        return ret;

    return 0;
}

static int ls_pcie_vusb_dev_detach(struct ls_pcie_vusb *vusb, uint8_t port)
{
    struct xhci_trb trb = {0};
    int ret;

    if (!vusb || port >= ARRAY_SIZE(vusb->port))
        return -EINVAL;
    if (!(vusb->port[port].portsc & PORTSC_CCS))
        return 0;

    trb.field[0] = (port + 1) << 24;
    trb.field[1] = 0;
    trb.field[2] = COMP_CODE(COMP_SUCCESS);
    trb.field[3] = TRB_PORT_STATUS << 10;

    ret = ls_pcie_vusb_port_update(vusb, port, 0);
    if (ret)
        return ret;
    ret = ls_pcie_vusb_post_event(vusb, &trb);
    if (ret)
        return ret;

    return 0;
}

static struct pci_epf *find_epf_by_name(struct pci_epc *epc, const char *name)
{
    struct pci_epf *epf;

    mutex_lock(&epc->list_lock);
    list_for_each_entry(epf, &epc->pci_epf, list) {
        if (strcmp(epf->name, name) == 0) {
            mutex_unlock(&epc->list_lock);
            return epf;
        }
    }
    mutex_unlock(&epc->list_lock);
    return NULL;
}

static irqreturn_t ls_pcie_vusb_irq(int irq, void *arg)
{
    struct ls_pcie_vusb *vusb = arg;
    struct pcie_vhub_reg __iomem *reg = (vusb->base + LS_PCEI_VUSB_CTRL_OFFSET);
    int event_num = reg->ctrl.event_num;

    if (reg->ctrl.data_overrun_int_raw) {
        dev_err(&vusb->pdev->dev,"fifo overrun!!, event_num %d\n", reg->ctrl.event_num);
        reg->ctrl.data_overrun_int_clr = 1;
    }

    for (int i = 0; i < event_num; i++) {
        struct ls_pcie_vusb_event event;
        event.addr.raw = reg->rxfifo.addr;
        event.data = reg->rxfifo.data;
        if (!ls_pcie_vusb_event_put(vusb, &event))
            dev_err(&vusb->pdev->dev, "kfifo full, drop event addr=0x%x data=0x%lx\n", event.addr.raw, event.data);
    }

    reg->ctrl.data_int_clr = 1;

    wake_up(&vusb->thread_wait);

    return IRQ_HANDLED;
}

static int pcie_vusb_event_dispatch(struct ls_pcie_vusb *vusb, struct ls_pcie_vusb_event *event)
{
    struct pcie_vusb_reg_info *reg_info;

    list_for_each_entry(reg_info, &vusb->reg_list, node) {
        if (event->addr.reg >= reg_info->addr && 
            event->addr.reg < (reg_info->addr + reg_info->size)) {
            if (event->addr.is_write)
                reg_info->write_reg(reg_info, event->addr.reg, event->data);
            else
                reg_info->read_reg(reg_info, event->addr.reg);
        }
    }

    return 0;
}

static int ls_pcie_vusb_thread(void *arg)
{
    struct ls_pcie_vusb *vusb = arg;
    struct ls_pcie_vusb_event event;

    while (!kthread_should_stop()) {
        wait_event_interruptible(vusb->thread_wait,
                                 !kfifo_is_empty(&vusb->fifo) ||
                                 kthread_should_stop());

        if (kthread_should_stop())
            break;

        while (kfifo_get(&vusb->fifo, &event))
            pcie_vusb_event_dispatch(vusb, &event);
    }

    return 0;
}

static int pcie_vusb_reg_init(void __iomem *base)
{
    pcie_vusb_cap_reg_init(base + LS_PCOE_VUSB_XHCI_CAP_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET);
    pcie_vusb_ext_reg_init(base + LS_PCOE_VUSB_XHCI_ET_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET);
    pcie_vusb_op_reg_init(base + LS_PCOE_VUSB_XHCI_OP_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET);
    pcie_vusb_rt_reg_init(base + LS_PCOE_VUSB_XHCI_RT_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET);

    return 0;
}

static int pcie_vusb_ops_init(struct ls_pcie_vusb *vusb)
{
    cap_reg.base = (char *)vusb->base + LS_PCOE_VUSB_XHCI_CAP_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
    op_reg.base = (char *)vusb->base + LS_PCOE_VUSB_XHCI_OP_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
    rt_reg.base = (char *)vusb->base + LS_PCOE_VUSB_XHCI_RT_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
    db_reg.base = (char *)vusb->base + LS_PCOE_VUSB_XHCI_DB_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
    et_reg.base = (char *)vusb->base + LS_PCOE_VUSB_XHCI_ET_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
    pt_reg.base = (char *)vusb->base + LS_PCOE_VUSB_XHCI_OP_OFFSET + LS_PCIE_VUSB_CPU_WR_OFFSET;
    submit_reg.base = (char *)vusb->base + LS_PCOE_VUSB_TRANSFER + LS_PCIE_VUSB_CPU_WR_OFFSET;
    port_update_reg.base = (char *)vusb->base + LS_PCOE_VUSB_TRANSFER + LS_PCIE_VUSB_CPU_WR_OFFSET;

    pcie_vusb_reg_ops_register(vusb, &rt_reg);
    pcie_vusb_reg_ops_register(vusb, &db_reg);
    pcie_vusb_reg_ops_register(vusb, &pt_reg);
    pcie_vusb_reg_ops_register(vusb, &op_reg);
    pcie_vusb_reg_ops_register(vusb, &et_reg);
    pcie_vusb_reg_ops_register(vusb, &cap_reg);
    pcie_vusb_reg_ops_register(vusb, &submit_reg);
    pcie_vusb_reg_ops_register(vusb, &port_update_reg);

    return 0;
}

static int pcie_vusb_init(struct ls_pcie_vusb *vusb)
{
    struct pcie_vhub_reg __iomem *reg = (vusb->base + LS_PCEI_VUSB_CTRL_OFFSET);

    /* flfo flush */
    reg->ctrl.data_overrun_int_clr = 1;
    /* init ctrl reg */
    reg->ctrl.raw = 0;
    reg->ctrl.data_int_en = 1;
    reg->ctrl.data_overrun_int_en = 1;
    reg->ctrl.event_thres = 0;
    reg->ctrl.time_count_en = 1;
    vusb->status = USBSTS_HCH;
    vusb->iman = 0;

    INIT_LIST_HEAD(&vusb->reg_list);
    pcie_vusb_reg_init(vusb->base);
    pcie_vusb_ops_init(vusb);
    INIT_KFIFO(vusb->fifo);
    spin_lock_init(&vusb->fifo_lock);

    return 0;
}

static int ls_pcie_vusb_probe(struct platform_device *pdev)
{
    struct device_node *parent = of_get_parent(pdev->dev.of_node);
    int irq, ret;
    struct ls_pcie_vusb *vusb;
    dma_addr_t dma;

    if (!parent) {
        dev_err(&pdev->dev, "Failed to get parent device node\n");
        return -ENODEV;
    }

    struct platform_device *parent_pdev = of_find_device_by_node(parent);
    if (!parent_pdev) {
        dev_err(&pdev->dev, "Failed to get parent platform device\n");
        return -ENODEV;
    }

    struct resource *res = platform_get_resource_byname(parent_pdev, IORESOURCE_MEM, "xhci_vhub");
    if (!res) {
        dev_err(&pdev->dev, "no resource\n");
        return -ENODEV;
    }

    vusb = devm_kzalloc(&pdev->dev, sizeof(*vusb), GFP_KERNEL);
    if (!vusb) {
        dev_err(&pdev->dev, "Failed to allocate memory for pcie vusb\n");
        return -ENOMEM;
    }

    vusb->base = devm_ioremap_resource(&pdev->dev, res);
    vusb->pcie = platform_get_drvdata(parent_pdev);
    vusb->pdev = pdev;

    vusb->dma_chan.write = ls_pcie_request_dma_chan(vusb->pcie, true);
    vusb->dma_chan.read = ls_pcie_request_dma_chan(vusb->pcie, false);

    vusb->epf = find_epf_by_name(vusb->pcie->pci.ep.epc, "xhci_vhub");

    vusb->dcbaa = dma_alloc_coherent(vusb->pcie->pci.dev, sizeof (struct xhci_dev_ctx_array), &dma, GFP_KERNEL);
    vusb->dcbaa->dma = dma;

    vusb->ep_cmd.trb = dma_alloc_coherent(vusb->pcie->pci.dev, sizeof (struct xhci_trb) * 3, &vusb->ep_cmd.trb_dma, GFP_KERNEL);

    vusb->crcr.trb = (u8 *)vusb->ep_cmd.trb + sizeof (struct xhci_trb);
    vusb->crcr.trb_dma = vusb->ep_cmd.trb_dma + sizeof (struct xhci_trb);
    vusb->event_ring.trb = (u8 *)vusb->ep_cmd.trb + 2 * sizeof (struct xhci_trb);
    vusb->event_ring.trb_dma = vusb->ep_cmd.trb_dma + 2 * sizeof (struct xhci_trb);

    platform_set_drvdata(pdev, vusb);

    init_waitqueue_head(&vusb->thread_wait);

    /* init ep req list and workqueue */
    for (int i = 0; i < ARRAY_SIZE(vusb->slot); i++) {
        spin_lock_init(&vusb->slot[i].lock);
        vusb->slot[i].address = 0xff;
        /* ep req/td list init */
        for (int j = 0; j < ARRAY_SIZE(vusb->slot[i].ep); j++) {
            INIT_LIST_HEAD(&vusb->slot[i].ep[j].ep_req);
            INIT_LIST_HEAD(&vusb->slot[i].ep[j].td_list);
        }
    }

    for (int i = 0; i < ARRAY_SIZE(vusb->port); i++) {
        spin_lock_init(&vusb->port[i].lock);
        vusb->port[i].portsc = (PLS_RX_DETECT << PORTSC_PLS_SHIFT) | PORTSC_PP;
    }

    vusb->slot_wq = alloc_ordered_workqueue("vusb-slot", WQ_HIGHPRI);
    if (!vusb->slot_wq) {
        dev_err(&pdev->dev, "alloc workqueue fail\n");
        return -ENOMEM;
    }

    irq = platform_get_irq(pdev, 0);
    if (irq < 0) {
        dev_err(&pdev->dev, "get irq fail\n");
        return irq;
    }
    vusb->irq = irq;

    pcie_vusb_init(vusb);

    vusb->thread = kthread_create(ls_pcie_vusb_thread, vusb, "vusb-service");
    if (IS_ERR(vusb->thread)) {
        ret = PTR_ERR(vusb->thread);
        dev_err(&pdev->dev, "create vusb irq thread fail\n");
        return ret;
    }
    /* top priority among normal tasks so events are handled promptly */
    set_user_nice(vusb->thread, MIN_NICE);
    wake_up_process(vusb->thread);

    ret = devm_request_irq(&pdev->dev, irq, ls_pcie_vusb_irq, 0,
                           dev_name(&pdev->dev), vusb);

    if (ret < 0) {
        dev_err(&pdev->dev, "cannot request irq\n");
        return ret;
    }

    ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
    if (ret < 0) {
        dev_err(&pdev->dev, "of_platform_populate fail\n");
        return ret;
    }

    return 0;
}

static int pcie_vusb_deinit(struct ls_pcie_vusb *vusb)
{
    struct pcie_vhub_reg __iomem *reg = (vusb->base + LS_PCEI_VUSB_CTRL_OFFSET);

    /* clear fifo */
    reg->ctrl.data_overrun_int_clr = 1;
    /* disable vusb */
    reg->ctrl.raw = 0;

    return 0;
}

static void ls_pcie_vusb_remove(struct platform_device *pdev)
{
    struct ls_pcie_vusb *vusb =  platform_get_drvdata(pdev);

    if (vusb->irq >= 0)
        devm_free_irq(&pdev->dev, vusb->irq, vusb);

    if (vusb->thread)
        kthread_stop(vusb->thread);

    pcie_vusb_deinit(vusb);

    for (int i = 0; i < ARRAY_SIZE(vusb->slot); i++) {
        vusb->slot[i].address = 0xff;
        vusb->slot[i].state = SLOT_STATE_DISABLE;
    }
    destroy_workqueue(vusb->slot_wq);

    ls_pcie_dma_release_chan(vusb->dma_chan.read);
    ls_pcie_dma_release_chan(vusb->dma_chan.write);

    dma_free_coherent(vusb->pcie->pci.dev, sizeof (*vusb->dcbaa), vusb->dcbaa, vusb->dcbaa->dma);
    dma_free_coherent(vusb->pcie->pci.dev, sizeof(struct xhci_trb) * 3, vusb->ep_cmd.trb, vusb->ep_cmd.trb_dma);

    for (int i = 0; i < ARRAY_SIZE(vusb->slot); i++) {
        if (vusb->slot[i].dev.ctx != NULL) {
            dma_free_coherent(vusb->pcie->pci.dev, sizeof(struct xhci_device_ctx), vusb->slot[i].dev.ctx, vusb->slot[i].dev.dma);
            vusb->slot[i].dev.ctx = NULL;
        }
        if (vusb->slot[i].input.ctx != NULL) {
            dma_free_coherent(vusb->pcie->pci.dev, sizeof (struct xhci_input_ctx), vusb->slot[i].input.ctx, vusb->slot[i].input.dma);
            vusb->slot[i].input.ctx = NULL;
        }
    }
}

static const struct of_device_id ls_pcie_vusb_match_table[] = {
    { .compatible = "linkedsemi,ls-pcie-vusb", .data = NULL},
    { }
};

static struct platform_driver ls_pcie_vusb_driver = {
    .driver = {
        .name = "ls-pcie-vusb",
        .of_match_table = of_match_ptr(ls_pcie_vusb_match_table),
    },
    .probe = ls_pcie_vusb_probe,
    .remove = ls_pcie_vusb_remove,
};

static int __init ls_pcie_vusb_init(void)
{
    return platform_driver_register(&ls_pcie_vusb_driver);
}

static void __exit ls_pcie_vusb_exit(void)
{
    platform_driver_unregister(&ls_pcie_vusb_driver);
}

module_init(ls_pcie_vusb_init)
module_exit(ls_pcie_vusb_exit)

MODULE_DEVICE_TABLE(of, ls_pcie_vusb_match_table);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zytang");
MODULE_DESCRIPTION("Linkedsemi PCIe vusb driver");
