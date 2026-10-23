#ifndef __VHUB_DEF_H__
#define __VHUB_DEF_H__

#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/usb/gadget.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/printk.h>

#define DWC3_GLOBALS_REGS_START     0xc100
#define DWC3_GLOBALS_REGS_END       0xc6ff
#define DWC3_DEVICE_REGS_START      0xc700
#define DWC3_DEVICE_REGS_END        0xcbff
#define DWC3_OTG_REGS_START         0xcc00
#define DWC3_OTG_REGS_END           0xccff

#define DWC3_XHCI_REGS_START        0x0
#define DWC3_XHCI_REGS_END          0x7fff

#define VHUB_EP_MAP_REGS_START      0xcd00
#define VHUB_EP_MAP_REG_COUNT       30
#define VHUB_EP_NUM                 (VHUB_EP_MAP_REG_COUNT + 2)

#define DWC3_GSBUSCFG0              0xc100
#define DWC3_GSBUSCFG1              0xc104
#define DWC3_GTXTHRCFG              0xc108 // Tx Threshold, is not applicable in USB 2.0-only mode
#define DWC3_GRXTHRCFG              0xc10c // Rx Threshold, SS only
#define DWC3_GCTL                   0xc110
#define DWC3_GPMSTS                 0xc114 // Power Mgmt Status, hibernation only
#define DWC3_GSTS                   0xc118
#define DWC3_GUCTL1                 0xc11c
#define DWC3_GSNPSID                0xc120
#define DWC3_GGPIO                  0xc124
#define DWC3_GUID                   0xc128
#define DWC3_GUCTL                  0xc12c
#define DWC3_GBUSERRADDRLO          0xc130
#define DWC3_GBUSERRADDRHI          0xc134

#define DWC3_GPRTBIMAP0             0xc138 // Port-to-Bus mapping, SS only
#define DWC3_GPRTBIMAP1             0xc13c // Port-to-Bus mapping, SS only
#define DWC3_GHWPARAMS0             0xc140
#define DWC3_GHWPARAMS1             0xc144
#define DWC3_GHWPARAMS2             0xc148
#define DWC3_GHWPARAMS3             0xc14c
#define DWC3_GHWPARAMS4             0xc150
#define DWC3_GHWPARAMS5             0xc154
#define DWC3_GHWPARAMS6             0xc158
#define DWC3_GHWPARAMS7             0xc15c
#define DWC3_GDBGFIFOSPACE          0xc160 // FIFO space debug: write SEL then read SPACE_AVAILABLE
#define DWC3_GDBGLTSSM              0xc164 // LTSSM debug
#define DWC3_GDBGBMU                0xc16c // Bus Monitor Unit debug
#define DWC3_GPRTBIMAP_HS0          0xc180 // Per-port HS mapping for ports 0-3
#define DWC3_GPRTBIMAP_HS1          0xc184 // Per-port HS mapping for ports 4-7
#define DWC3_GPRTBIMAP_FS0          0xc188 // Per-port FS/LS mapping for ports 0-3
#define DWC3_GPRTBIMAP_FS1          0xc18c // Per-port FS/LS mapping for ports 4-7
#define DWC3_GUCTL2                 0xc19c // Global User Control 2
#define DWC3_VER_NUMBER             0xc1a0 // Version number
#define DWC3_VER_TYPE               0xc1a4 // Version type
#define DWC3_GTXFIFOSIZ(n)           (0xc300 + ((n) * 0x04)) // TX FIFO RAM start addr + depth
#define DWC3_GTXFIFOSIZ_DEPTH(n)     ((n) & 0xffff)
#define DWC3_GTXFIFOSIZ_TXFSTADDR(n) ((n) & 0xffff0000)
#define DWC3_GRXFIFOSIZ(n)           (0xc380 + ((n) * 0x04)) // RX FIFO RAM start addr + depth
#define DWC3_GHWPARAMS8             0xc600
#define DWC3_GUCTL3                 0xc60c
#define DWC3_GFLADJ                 0xc630 // Frame Length Adj, REFCLK_* fields LPM/Hibernation only

/* HWPARAMS0 field decode */
#define DWC3_HWPARAMS0_MDWIDTH(n)   (((n) >> 8) & 0xff)
/* HWPARAMS1 field decode */
#define DWC3_HWPARAMS1_SPRAM_TYPE(n) (((n) >> 23) & 1)  /* 1 = single-port RAM */
/* HWPARAMS6 field decode */
#define DWC3_HWPARAMS6_RAM0_DEPTH(n)   (((n) >> 16) & 0xffff)
/* HWPARAMS7 field decode */
#define DWC3_HWPARAMS7_RAM1_DEPTH(n)   ((n) & 0xffff)

#define DWC3_GUSB2PHYCFG(n)         (0xc200 + ((n) * 0x04))
#define DWC3_GUSB2I2CCTL(n)         (0xc240 + ((n) * 0x04))
#define DWC3_GUSB2PHYACC(n)         (0xc280 + ((n) * 0x04))
#define DWC3_GUSB2PHYACC_ULPI(n)   (0xc280 + ((n) * 0x04))

#define DWC3_ULPI_PHYACC_NEWREGREQ  BIT(25)
#define DWC3_ULPI_PHYACC_VSTSDONE   BIT(24)
#define DWC3_ULPI_PHYACC_VSTSBSY    BIT(23)
#define DWC3_ULPI_PHYACC_REGWR      BIT(22)
#define DWC3_ULPI_PHYACC_REGADDR(n) (((n) & 0x3f) << 16)
#define DWC3_ULPI_PHYACC_EXTREGADDR(n) (((n) & 0xff) << 8)
#define DWC3_ULPI_PHYACC_REGDATA(n) ((n) & 0xff)

#define DWC3_DCFG                   0xc700 // Device Config, DevSpd=HS/FS in USB 2.0-only mode
#define DWC3_DCTL                   0xc704 // Device Control, L1HibernationEn/KeepConnect always 0 (EN_PWROPT=0)
#define DWC3_DEVTEN                 0xc708
#define DWC3_DSTS                   0xc70c
#define DWC3_DGCMDPAR               0xc710
#define DWC3_DGCMD                  0xc714
#define DWC3_DALEPENA               0xc720
#define DWC3_DCFG1                  0xc740 /* DWC_usb32 only */

#define DWC3_DEP_BASE(n)            (0xc800 + ((n) * 0x10))
#define DWC3_DEPCMDPAR2(n)          (DWC3_DEP_BASE(n) + 0x00)
#define DWC3_DEPCMDPAR1(n)          (DWC3_DEP_BASE(n) + 0x04)
#define DWC3_DEPCMDPAR0(n)          (DWC3_DEP_BASE(n) + 0x08)
#define DWC3_DEPCMD(n)              (DWC3_DEP_BASE(n) + 0x0c)

#define DWC3_GEVNTADRLO(n)          (0xc400 + ((n) * 0x10))
#define DWC3_GEVNTADRHI(n)          (0xc404 + ((n) * 0x10))
#define DWC3_GEVNTSIZ(n)            (0xc408 + ((n) * 0x10))
#define DWC3_GEVNTCOUNT(n)          (0xc40c + ((n) * 0x10))

/* GEVNTSIZ bits */
#define DWC3_GEVNTSIZ_INTMASK       BIT(31)
#define DWC3_GEVNTSIZ_SIZE(n)       ((n) & 0xffff)
#define DWC3_GEVNTCOUNT_MASK        0xfffc
#define DWC3_EVENT_BUFFERS_SIZE     4096

#define DWC3_DCTL_RUN_STOP          BIT(31)
#define DWC3_DCTL_CSFTRST           BIT(30)
#define DWC3_DCTL_LSFTRST           BIT(29)
#define DWC3_DCTL_ULSTCHNGREQ_MASK  (0x0f << 5)
#define DWC3_DCTL_ULSTCHNGREQ(n)    (((n) << 5) & DWC3_DCTL_ULSTCHNGREQ_MASK)
#define DWC3_DCTL_TSTCTRL_MASK      (0xf << 1)

#define DWC3_DCFG_DEVSPD(n)         ((n) & 0x7)        /* [2:0] */
#define DWC3_DCFG_DEVSPD_HS         0
#define DWC3_DCFG_DEVSPD_FS         1
#define DWC3_DCFG_DEVSPD_SS         4
#define DWC3_DCFG_DEVADDR(addr)     ((addr) << 3)      /* [9:3] */
#define DWC3_DCFG_DEVADDR_MASK      DWC3_DCFG_DEVADDR(0x7f)
#define DWC3_DCFG_INTRNUM(n)        (((n) & 0x3f) << 12) /* [17:12] */
#define DWC3_DCFG_NUMP_SET(n)       (((n) & 0x1f) << 17) /* [21:17] */
#define DWC3_DCFG_LPM_CAP           BIT(22)

/* GUCTL fields */
#define DWC3_GUCTL_REFCLKPER(n)     (((n) & 0x3ff) << 22) /* [31:22] */

/* GUCTL2 fields */
#define DWC3_GUCTL2_RST_ACTBITLATER  BIT(14)
#define DWC3_GUCTL2_LC_TIMER         BIT(19)

/* GCTL fields (partial, add as needed) */
#define DWC3_GCTL_PWRDNSCALE(n)     ((n) << 19)
#define DWC3_GCTL_U2RSTECN          BIT(16)
#define DWC3_GCTL_PRTCAPDIR(n)      ((n) << 12)
#define DWC3_GCTL_PRTCAP_HOST       1
#define DWC3_GCTL_PRTCAP_DEVICE     2
#define DWC3_GCTL_PRTCAP_OTG        3
#define DWC3_GCTL_CORESOFTRESET     BIT(11)
#define DWC3_GCTL_SOFITPSYNC        BIT(10)
#define DWC3_GCTL_SCALEDOWN(n)      ((n) << 4)
#define DWC3_GCTL_DISSCRAMBLE       BIT(3)
#define DWC3_GCTL_U2EXIT_LFPS       BIT(2)
#define DWC3_GCTL_GBLHIBERNATIONEN  BIT(1)
#define DWC3_GCTL_DSBLCLKGTNG       BIT(0)

#define DWC3_DEVTEN_L1L2L3EVTEN(n)  (((n) & 0x1f) << 12) /* [16:12] */
#define DWC3_DEVTEN_VNDRDEVTSTRCVEDEN   BIT(12)
#define DWC3_DEVTEN_EVNTOVERFLOWEN      BIT(11)
#define DWC3_DEVTEN_CMDCMPLTEN          BIT(10)
#define DWC3_DEVTEN_ERRTICERREN         BIT(9)
#define DWC3_DEVTEN_SOFEN               BIT(7)
#define DWC3_DEVTEN_U3L2L1SUSPEN        BIT(6)
#define DWC3_DEVTEN_HIBERNATIONREQEVTEN BIT(5)
#define DWC3_DEVTEN_WKUPEVTEN           BIT(4)
#define DWC3_DEVTEN_ULSTCNGEN           BIT(3)
#define DWC3_DEVTEN_CONNECTDONEEN       BIT(2)
#define DWC3_DEVTEN_USBRSTEN            BIT(1)
#define DWC3_DEVTEN_DISCONNEVTEN        BIT(0)

#define DWC3_GSTS_BUSERRADDRVLD     BIT(4)
#define DWC3_GSTS_CSRTRANS          BIT(5)
#define DWC3_GSTS_DEVICEINT         BIT(6)

#define DWC3_DSTS_DEVCTRLHLT        BIT(22)
#define DWC3_DSTS_USBLNKST_SHIFT    18
#define DWC3_DSTS_USBLNKST_MASK     (0xf << 18)
#define DWC3_DSTS_CONNECTSPD        (7 << 0)
#define DWC3_DSTS_SUPERSPEED_PLUS   (5 << 0) /* DWC_usb31 only */
#define DWC3_DSTS_SUPERSPEED        (4 << 0)
#define DWC3_DSTS_HIGHSPEED         (0 << 0)
#define DWC3_DSTS_FULLSPEED         BIT(0)

/* GDBGFIFOSPACE fields */
#define DWC3_GDBGFIFOSPACE_SPACE(n)     (((n) >> 16) & 0xffff)  /* [31:16] */
#define DWC3_GDBGFIFOSPACE_NUM(n)       ((n) & 0x1f)            /* [4:0] 物理端点号 */
#define DWC3_GDBGFIFOSPACE_TYPE(n)      (((n) << 5) & 0x1e0)    /* [8:5] FIFO类型：0=TXFIFO, 1=RXFIFO, 2=TXREQQ */

#define DWC3_DALEPENA_EP(n)         BIT(n)

#define DWC3_DEPCMD_DEPSTARTCFG     (0x09 << 0)
#define DWC3_DEPCMD_ENDTRANSFER     (0x08 << 0)
#define DWC3_DEPCMD_UPDATETRANSFER  (0x07 << 0)
#define DWC3_DEPCMD_STARTTRANSFER   (0x06 << 0)
#define DWC3_DEPCMD_CLEARSTALL      (0x05 << 0)
#define DWC3_DEPCMD_SETSTALL        (0x04 << 0)
/* Applies for core versions 1.90a and earlier */
#define DWC3_DEPCMD_GETSEQNUMBER    (0x03 << 0)
/* Applies for core versions 1.94a and later */
#define DWC3_DEPCMD_GETEPSTATE      (0x03 << 0)
#define DWC3_DEPCMD_SETTRANSFRESOURCE (0x02 << 0)
#define DWC3_DEPCMD_SETEPCONFIG     (0x01 << 0)

#define DWC3_DEPCMD_CMD(x)          ((x) & 0xf)
#define DWC3_DEPCMD_PARAM_SHIFT     16
#define DWC3_DEPCMD_PARAM(x)        ((x) << DWC3_DEPCMD_PARAM_SHIFT)
#define DWC3_DEPCMD_GET_RSC_IDX(x)  (((x) >> DWC3_DEPCMD_PARAM_SHIFT) & 0x7f)
#define DWC3_DEPCMD_STATUS(x)       (((x) >> 12) & 0x0F)
#define DWC3_DEPCMD_HIPRI_FORCERM   BIT(11)
#define DWC3_DEPCMD_CLEARPENDIN     BIT(11)
#define DWC3_DEPCMD_CMDACT          BIT(10)
#define DWC3_DEPCMD_CMDIOC          BIT(8)

/* DEPCFG parameter 1 */
#define DWC3_DEPCFG_INT_NUM(n)          (((n) & 0x1f) << 0)
#define DWC3_DEPCFG_XFER_COMPLETE_EN    BIT(8)
#define DWC3_DEPCFG_XFER_IN_PROGRESS_EN BIT(9)
#define DWC3_DEPCFG_XFER_NOT_READY_EN   BIT(10)
#define DWC3_DEPCFG_STREAM_EVENT_EN     BIT(13)
#define DWC3_DEPCFG_BINTERVAL_M1(n)     (((n) & 0xff) << 16)
#define DWC3_DEPCFG_STREAM_CAPABLE      BIT(24)
#define DWC3_DEPCFG_EP_NUMBER(n)        (((n) & 0x1f) << 25)
#define DWC3_DEPCFG_FIFO_BASED          BIT(31)

/* DEPCFG parameter 0 */
#define DWC3_DEPCFG_EP_TYPE(n)          (((n) & 0x3) << 1)
#define DWC3_DEPCFG_MAX_PACKET_SIZE(n)  (((n) & 0x7ff) << 3)
#define DWC3_DEPCFG_FIFO_NUMBER(n)      (((n) & 0x1f) << 17)
#define DWC3_DEPCFG_BURST_SIZE(n)       (((n) & 0xf) << 22)
#define DWC3_DEPCFG_ACTION(n)           (((n) & 0x3) << 30)

#define DWC3_DEPXFERCFG_NUM_XFER_RES(n) ((n) & 0xffff)

/* TRB Control bits */
#define DWC3_TRB_CTRL_HWO               BIT(0)
#define DWC3_TRB_CTRL_LST               BIT(1)
#define DWC3_TRB_CTRL_CHN               BIT(2)
#define DWC3_TRB_CTRL_CSP               BIT(3)
#define DWC3_TRB_CTRL_TRBCTL(n)         (((n) & 0x3f) << 4)
#define DWC3_TRB_CTRL_ISP_IMI           BIT(10)
#define DWC3_TRB_CTRL_IOC               BIT(11)
#define DWC3_TRB_CTRL_SID_SOFN(n)       (((n) & 0xffff) << 14)
#define DWC3_TRB_CTRL_GET_SID_SOFN(n)   (((n) & (0xffff << 14)) >> 14)

/* TRB types (TRBCTL field encodings) */
#define DWC3_TRBCTL_TYPE(n)             ((n) & (0x3f << 4))
#define DWC3_TRBCTL_NORMAL              DWC3_TRB_CTRL_TRBCTL(1)
#define DWC3_TRBCTL_CONTROL_SETUP       DWC3_TRB_CTRL_TRBCTL(2)
#define DWC3_TRBCTL_CONTROL_STATUS2     DWC3_TRB_CTRL_TRBCTL(3)
#define DWC3_TRBCTL_CONTROL_STATUS3     DWC3_TRB_CTRL_TRBCTL(4)
#define DWC3_TRBCTL_CONTROL_DATA        DWC3_TRB_CTRL_TRBCTL(5)
#define DWC3_TRBCTL_ISOCHRONOUS_FIRST   DWC3_TRB_CTRL_TRBCTL(6)
#define DWC3_TRBCTL_ISOCHRONOUS         DWC3_TRB_CTRL_TRBCTL(7)
#define DWC3_TRBCTL_LINK_TRB            DWC3_TRB_CTRL_TRBCTL(8)

/* TRB Size bits */
#define DWC3_TRB_SIZE_MASK              (0x00ffffff)
#define DWC3_TRB_SIZE_LENGTH(n)         ((n) & DWC3_TRB_SIZE_MASK)
#define DWC3_TRB_SIZE_PCM1(n)           (((n) & 0x03) << 24)
#define DWC3_TRB_SIZE_TRBSTS(n)         (((n) & (0x0f << 28)) >> 28)

/* TRB Status codes */
#define DWC3_TRBSTS_OK                  0
#define DWC3_TRBSTS_MISSED_ISOC         1
#define DWC3_TRBSTS_SETUP_PENDING       2
#define DWC3_TRB_STS_XFER_IN_PROG       4

#define DWC3_NUM_EPS_MASK               (0x3f << 12)
#define DWC3_NUM_EPS(hwparams3)         ((hwparams3 & (DWC3_NUM_EPS_MASK)) >> 12)

/* Device events */
#define DWC3_DEVICE_EVENT_DISCONNECT            0
#define DWC3_DEVICE_EVENT_RESET                 1
#define DWC3_DEVICE_EVENT_CONNECT_DONE          2
#define DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE    3
#define DWC3_DEVICE_EVENT_WAKEUP                4
#define DWC3_DEVICE_EVENT_HIBER_REQ             5
#define DWC3_DEVICE_EVENT_SUSPEND               6
#define DWC3_DEVICE_EVENT_SOF                   7
#define DWC3_DEVICE_EVENT_ERRATIC_ERROR         9
#define DWC3_DEVICE_EVENT_CMD_CMPL              10
#define DWC3_DEVICE_EVENT_OVERFLOW              11

/* Endpoint events */
enum ep_evt {
    DEPEVT_XferComplete   = 1,
    DEPEVT_XferInProgress = 2,
    DEPEVT_XferNotReady   = 3,
    DEPEVT_StreamEvt      = 6,
    DEPEVT_EPCmdCmplt     = 7,
};

/* Endpoint event status codes */
#define DEPEVT_TRANSFER_NO_RESOURCE     1
#define DEPEVT_TRANSFER_BUS_EXPIRY      2

/* Link states */
#define DWC3_LINK_STATE_MASK            0x0f
#define DWC3_LINK_STATE_U0              0x00
#define DWC3_LINK_STATE_U1              0x01
#define DWC3_LINK_STATE_U2              0x02
#define DWC3_LINK_STATE_U3              0x03
#define DWC3_LINK_STATE_RESET           0x0e
#define DWC3_LINK_STATE_RESUME          0x0f

struct dwc3_trb {
    u32 bptrl;
    u32 bptrh;
    u32 size;
    u32 ctrl;
};

struct ls_vhub_ep_trb {
    struct dwc3_trb trb[2];
};

union evt_buf {
    /* Endpoint event view (bit 0 == 0) */
    struct {
        u32 is_dep      : 1;  /* [0]           0 for endpoint event       */
        u32 phy_ep_num  : 5;  /* [5:1]         physical endpoint number    */
        u32 ep_evt_type : 4;  /* [9:6]         endpoint event type         */
        u32             : 2;  /* [11:10]       reserved                    */
        u32 evt_status  : 4;  /* [15:12]       event status                */
        u32 evt_param   : 16; /* [31:16]       event parameter             */
    } depevt;

    /* Device event view (bit 0 == 1) */
    struct {
        u32 is_devt     : 1;  /* [0]           1 for device event          */
        u32             : 7;  /* [7:1]         reserved                    */
        u32 dev_evt_type: 5;  /* [12:8]        device event type           */
        u32             : 3;  /* [15:13]       reserved                    */
        u32 evt_info    : 9;  /* [24:16]       event info                  */
        u32             : 7;  /* [31:25]       reserved                    */
    } devt;
};

#define EVT_IS_DEPEVT(evt)  (!(evt)->depevt.is_dep)

struct event_buffer {
    void *buf;
    dma_addr_t dma;
    u32 length;
    u32 dequeue;
};

union ls_vhub_ep_map {
    u32 value;
    struct {
        u32 dev_addr : 7;
        u32 ep_num   : 4;
        u32 rsvd     : 21;
    };
};

struct dwc3_hwparams {
    u32 hwparams0;
    u32 hwparams1;
    u32 hwparams2;
    u32 hwparams3;
    u32 hwparams4;
    u32 hwparams5;
    u32 hwparams6;
    u32 hwparams7;
    u32 hwparams8;
    u32 hwparams9;
};

/* EP0 state machine */
enum ls_vhub_ep0_state {
    EP0_UNCONNECTED = 0,
    EP0_SETUP_PHASE,
    EP0_DATA_PHASE,
    EP0_STATUS_PHASE
};

/* Endpoint configuration action */
enum ep_config_action {
    Initialize_EP_State,
    Restore_EP_State,
    Modify_EP_State
};

/* Forward declarations */
struct ls_vhub_dev;
struct ls_vhub;

struct ls_vhub_req {
    struct usb_request request;
    struct list_head node;
    bool dma_mapped : 1;
    bool dir : 1;
    bool chain : 1;
    bool internel : 1;
};

struct ls_vhub_ep {
    struct usb_ep endpoint;
    struct ls_vhub_dev *dev;
    struct ls_vhub *vhub;
    volatile union ls_vhub_ep_map *map;
    struct ls_vhub_ep_trb *trb;
    dma_addr_t trb_dma;
    struct list_head dev_node;
    struct list_head req_list;
    char name[8];
    u8 number;
    u8 res_idx;
    bool used : 1;
    bool ctrl : 1;
    bool common : 1;
    bool resource : 1;
    bool stall : 1;
    bool wedge : 1;
    bool enabled : 1;
    bool active : 1;
    bool delay_start : 1;
};

struct ls_vhub_ep_pool {
    struct ls_vhub_ep *eps;
    volatile union ls_vhub_ep_map *map_table;
    struct ls_vhub_ep_trb *trb_pool;
    dma_addr_t trb_dma;
    u16 ep_num;
};

struct ls_vhub_ctrl_ep {
    struct ls_vhub_ep *out;
    struct ls_vhub_ep *in;
    struct ls_vhub_req req;
    void *data;
    dma_addr_t data_dma;
    u16 data_len;
    enum ls_vhub_ep0_state state;
    bool three_stage_setup : 1;
    bool expect_in : 1;
};

struct ls_vhub_dev {
    struct usb_gadget_driver *driver;
    struct usb_gadget *gadget;  /* gadget struct with descriptors etc. */
    struct device *udev;        /* registered with kernel since not auto-created */
    struct ls_vhub_ctrl_ep ep0;
    struct ls_vhub_ctrl_ep *common_ep0;
    struct list_head ep_list;   /* ep0 + matched endpoints from pool */
    struct ls_vhub *vhub;
    struct dentry *debugfs_dir;
    u8 delayed_status;
    u8 address;
    u8 port_index;
    u8 enabled;
};

struct ls_pcie_vhub_port {
    union {
        struct {
            u16 status;
            u16 change;
        };
        u32 raw;
    } portsc;
    struct ls_vhub_dev dev;
};

struct dep_command_param {
    u32 param2;
    u32 param1;
    u32 param0;
};

struct ls_vhub {
    struct platform_device *pdev;
    void __iomem *regs;
    int irq;
    struct event_buffer evt_buf;
    DECLARE_KFIFO(fifo, u32, 64);
    spinlock_t lock;
    struct dwc3_hwparams hwparams;
    struct ls_vhub_ep_pool ep_pool;
    u8 address;
    bool suspended;
    struct ls_vhub_ctrl_ep common_ep0;
    struct ls_vhub_ctrl_ep ep0;
    struct ls_vhub_dev *addr0_dev;
    u8 ports_num;
    struct ls_pcie_vhub_port *ports;
    void *bounce;
    dma_addr_t bounce_dma;
    struct clk *clk;
    struct dentry *debugfs_root;
    struct debugfs_regset32 *regset;
    struct {
        struct ls_vhub_ep *ep;
        struct ls_vhub_req req;
        u8 *buf;
        dma_addr_t buf_dma;
        u16 buf_len;
        bool notify_pending;
    } epn;
};

static inline struct device *vhub_ep_dev(struct ls_vhub_ep *ep)
{
    return ep->dev ? ep->dev->udev : &ep->vhub->pdev->dev;
}

extern int ls_vhub_debug;

#define vhub_dbg(dev, fmt, ...) \
    do { \
        if (ls_vhub_debug) \
            dev_info(dev, fmt, ##__VA_ARGS__); \
        else \
            dev_dbg(dev, fmt, ##__VA_ARGS__); \
    } while (0)

#define vhub_error(dev, fmt, ...) \
    do { \
            dev_err(dev, fmt, ##__VA_ARGS__); \
    } while (0)

#define vhub_dbg_vhub(vhub, fmt, ...) \
    vhub_dbg(&(vhub)->pdev->dev, "[hub] " fmt, ##__VA_ARGS__)

#define vhub_error_vhub(vhub, fmt, ...) \
    vhub_error(&(vhub)->pdev->dev, "[hub] " fmt, ##__VA_ARGS__)

#define vhub_dbg_dev(dev, fmt, ...) \
    vhub_dbg((dev)->udev, "[dev] " fmt, ##__VA_ARGS__)

#define vhub_dbg_ep(ep, fmt, ...) \
    vhub_dbg(vhub_ep_dev(ep), "[ep%u] " fmt, (ep)->number, ##__VA_ARGS__)

#define vhub_dbg_ep_hex_dump(ep, prefix_str, buf, len) \
    do { \
        char __prefix[32]; \
        scnprintf(__prefix, sizeof(__prefix), "[ep%u] %s", (ep)->number, prefix_str); \
        vhub_dbg_hex_dump(vhub_ep_dev(ep), __prefix, buf, len); \
    } while (0)

#define vhub_dbg_hex_dump(dev, prefix_str, buf, len) \
    do { \
        if (ls_vhub_debug) \
            print_hex_dump(KERN_INFO, prefix_str, DUMP_PREFIX_OFFSET, 16, 1, \
                           buf, len, true); \
        else \
            print_hex_dump_debug(prefix_str, DUMP_PREFIX_OFFSET, 16, 1, \
                                 buf, len, true); \
    } while (0)

static inline struct ls_vhub_req *next_request(struct list_head *list)
{
    return list_first_entry_or_null(list, struct ls_vhub_req, node);
}

static inline void ls_vhub_lock(struct ls_vhub *vhub, unsigned long *flags)
{
    spin_lock_irqsave(&vhub->lock, *flags);
}

static inline void ls_vhub_unlock(struct ls_vhub *vhub, unsigned long *flags)
{
    spin_unlock_irqrestore(&vhub->lock, *flags);
}

static inline u32 vhub_readl(void __iomem *base, u32 offset)
{
    return readl(base + offset - DWC3_GLOBALS_REGS_START);
}

static inline void vhub_writel(void __iomem *base, u32 offset, u32 value)
{
    writel(value, base + offset - DWC3_GLOBALS_REGS_START);
}

#endif
