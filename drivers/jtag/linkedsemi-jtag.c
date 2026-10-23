#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/jtag.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define DEVICE_NAME    "ls-jtag"

/*
 * MJTAG Register map (from mjtag说明.xlsx)
 *
 *  Offset  Name         Access  FIFO Depth     Description
 *  ------  ----         ------  ----------     -----------
 *  0x00    INTR_MSK     R/W     —              Interrupt mask (写 1 使能对应中断)
 *  0x04    INTR_CLR     W       —              Interrupt clear (写 1 清除对应中断)
 *  0x08    INTR_STT     R       —              Interrupt status (1=中断已产生)
 *  0x0c    INTR_RAW     R       —              Interrupt raw (未 mask 的原始中断状态)
 *  0x10    TMS          R/W     16 条 (64B)    TMS 帧写数据 (32-bit/条，LSB 先发)
 *  0x14    TDI          R/W     16 条 (64B)    TDI 帧写数据 (32-bit/条，LSB 先发)
 *  0x18    DW           R/W     16 条 (64B)    DW 帧写数据 [5:0]，一帧 TCK 周期数 (max 63)
 *  0x1c    TDO          R       16 条 (64B)    TDO 帧读数据 (32-bit/条，LSB 先收)
 *  0x20    FIFO_PUSH    W       —             BIT(0)=TMS, BIT(1)=TDI, BIT(2)=DW 写使能脉冲
 *                                              (仅 mjtag_fifo_wren=1 时有效)
 *  0x24    CTRL         R/W     —             [11:0]=TCK 分频, [20:16]=TXD 延时, [28:24]=RXD 延时
 *  0x28    TRST         R/W     —             BIT(0): 1=TRST 高电平(释放), 0=TRST 低电平(复位)
 *  0x2c    FIFO_WREN    R/W     —             1=软件写 FIFO 模式, 0=启动发送
 *  0x30    TDO_FT       R/W     —             [3:0] TDO FIFO 快满阈值 (0~15, default 8)
 */

#define MJTAG_INTR_MSK      0x00
#define MJTAG_INTR_CLR      0x04
#define MJTAG_INTR_STT      0x08
#define MJTAG_INTR_RAW      0x0c
#define MJTAG_TMS           0x10
#define MJTAG_TDI           0x14
#define MJTAG_DW            0x18
#define MJTAG_TDO           0x1c
#define MJTAG_FIFO_PUSH     0x20
#define MJTAG_CTRL          0x24
#define MJTAG_TRST          0x28
#define MJTAG_FIFO_WREN     0x2c
#define MJTAG_TDO_FT        0x30

/* Interrupt bits */
#define MJTAG_INTR_ALL          0x1f
#define MJTAG_INTR_TX_FIFO_FULL     BIT(0)
#define MJTAG_INTR_TX_FIFO_EMPTY    BIT(1)
#define MJTAG_INTR_RX_FIFO_FULL     BIT(2)
#define MJTAG_INTR_RX_FIFO_EMPTY    BIT(3)
#define MJTAG_INTR_RX_FIFO_ALMOST_FULL  BIT(4)

/* FIFO_PUSH bits */
#define MJTAG_FIFO_PUSH_TMS BIT(0)
#define MJTAG_FIFO_PUSH_TDI BIT(1)
#define MJTAG_FIFO_PUSH_DW  BIT(2)
#define MJTAG_FIFO_PUSH_ALL (MJTAG_FIFO_PUSH_TMS | \
                 MJTAG_FIFO_PUSH_TDI | \
                 MJTAG_FIFO_PUSH_DW)

/* CTRL fields */
#define MJTAG_CTRL_TCK_DIV_MASK     0x00000fff
#define MJTAG_CTRL_TXD_CAP_DLY_MASK 0x001f0000
#define MJTAG_CTRL_TXD_CAP_DLY_SHIFT    16
#define MJTAG_CTRL_RXD_CAP_DLY_MASK 0x1f000000
#define MJTAG_CTRL_RXD_CAP_DLY_SHIFT    24

/* TRST */
#define MJTAG_TRST_CTL      BIT(0)

/* FIFO_WREN */
#define MJTAG_FIFO_WREN_START   0
#define MJTAG_FIFO_WREN_WRITE   BIT(0)

/*
 * FIFO parameters:
 *   TMS / TDI / DW each have their own 64-byte (16-entry) TX FIFO.
 *   TDO has a 64-byte (16-entry) RX FIFO.
 *   Up to 16 frames can be queued before draining.
 *   TMS/TDI registers are 32-bit, so effective max per frame is 32 bits.
 */
#define MJTAG_FIFO_DEPTH    16
#define MJTAG_FRAME_MAX     32      /* limited by 32-bit TMS/TDI registers */
#define MJTAG_XFER_TIMEOUT_MS    1000

struct ls_jtag_xfer
{
    struct jtag_xfer *xfer;
    u8 *xfer_data;
};

struct ls_jtag {
    struct device *dev;
    void __iomem *base;
    struct clk *clk;
    int irq;
    enum jtag_tapstate current_state;
    struct ls_jtag_xfer *cur_xfer;
    struct completion done;
    struct mutex xfer_lock;
};

static inline u32 ls_jtag_readl(struct ls_jtag *ls_jtag, u32 reg)
{
    return readl(ls_jtag->base + reg);
}

static inline void ls_jtag_writel(struct ls_jtag *ls_jtag, u32 val, u32 reg)
{
    writel(val, ls_jtag->base + reg);
}

static irqreturn_t ls_jtag_isr(int irq, void *dev_id)
{
    struct ls_jtag *ls_jtag = dev_id;
    u32 stt;

    stt = ls_jtag_readl(ls_jtag, MJTAG_INTR_STT);
    if (!stt)
        return IRQ_NONE;

    if (stt & (MJTAG_INTR_RX_FIFO_ALMOST_FULL | MJTAG_INTR_RX_FIFO_FULL)) {
        u32 msk = ls_jtag_readl(ls_jtag, MJTAG_INTR_MSK);

        msk &= ~(MJTAG_INTR_RX_FIFO_ALMOST_FULL | MJTAG_INTR_RX_FIFO_FULL);
        ls_jtag_writel(ls_jtag, msk, MJTAG_INTR_MSK);
    }

    ls_jtag_writel(ls_jtag, stt, MJTAG_INTR_CLR);

    if (stt & (MJTAG_INTR_RX_FIFO_ALMOST_FULL | MJTAG_INTR_RX_FIFO_FULL))
        complete(&ls_jtag->done);

    return IRQ_HANDLED;
}

static int ls_jtag_out_disable(struct ls_jtag *ls_jtag)
{
    /* TODO: 设置为高阻态 */
    return 0;
}

static int ls_jtag_out_enable(struct ls_jtag *ls_jtag)
{
    /* TODO: 设置为 jtag 模式 */
    return 0;
}

static int ls_jtag_freq_get(struct jtag *jtag, u32 *freq)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    unsigned long pclk;
    u32 div;

    pclk = clk_get_rate(ls_jtag->clk);
    if (!pclk)
        return -EOPNOTSUPP;

    div = ls_jtag_readl(ls_jtag, MJTAG_CTRL) & MJTAG_CTRL_TCK_DIV_MASK;
    *freq = pclk / div;
    return 0;
}

static int ls_jtag_freq_set(struct jtag *jtag, u32 freq)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    unsigned long pclk;
    u32 div, ctrl, txd_dly;

    if (!freq)
        return -EINVAL;

    pclk = clk_get_rate(ls_jtag->clk);
    if (!pclk)
        return -EOPNOTSUPP;

    /*
     * TCK = pclk / div. div is written directly to CTRL[11:0].
     * Minimum divider is 2. Use ceiling division to ensure
     * actual TCK <= requested freq.
     */
    div = (pclk + freq - 1) / freq;
    if (div < 2)
        div = 2;
    if (div > MJTAG_CTRL_TCK_DIV_MASK)
        div = MJTAG_CTRL_TCK_DIV_MASK;

    /* TXD cap delay: div / 2 - 1, clamped to the 5-bit field max */
    txd_dly = div / 2 - 1;
    if (txd_dly > 0x1f)
        txd_dly = 0x1f;

    ctrl = ls_jtag_readl(ls_jtag, MJTAG_CTRL);
    ctrl &= ~(MJTAG_CTRL_TCK_DIV_MASK | MJTAG_CTRL_TXD_CAP_DLY_MASK);
    ctrl |= div;
    ctrl |= txd_dly << MJTAG_CTRL_TXD_CAP_DLY_SHIFT;
    ls_jtag_writel(ls_jtag, ctrl, MJTAG_CTRL);

    return 0;
}

static int ls_jtag_status_get(struct jtag *jtag, u32 *state)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    *state = ls_jtag->current_state;
    return 0;
}

static int ls_jtag_wait_done(struct ls_jtag *ls_jtag)
{
    if (!wait_for_completion_timeout(&ls_jtag->done,
                                     msecs_to_jiffies(MJTAG_XFER_TIMEOUT_MS))) {
        dev_err(ls_jtag->dev, "transfer timed out\n");
        return -ETIMEDOUT;
    }
    return 0;
}

static void ls_jtag_flush_rx(struct ls_jtag *ls_jtag)
{
    int i;

    for (i = 0; i < MJTAG_FIFO_DEPTH; i++) {
        if (ls_jtag_readl(ls_jtag, MJTAG_INTR_RAW) & MJTAG_INTR_RX_FIFO_EMPTY)
            break;
        ls_jtag_readl(ls_jtag, MJTAG_TDO);
    }
}

static int __ls_jtag_status_set(struct ls_jtag *ls_jtag, const struct tms_cycle *tms)
{
    int ret;

    /* start jtag transfer */
    ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_WRITE, MJTAG_FIFO_WREN);
    ls_jtag_writel(ls_jtag, 0, MJTAG_TDO_FT);
    ls_jtag_writel(ls_jtag, tms->tmsbits, MJTAG_TMS);
    ls_jtag_writel(ls_jtag, 0, MJTAG_TDI);
    ls_jtag_writel(ls_jtag, tms->count, MJTAG_DW);
    ls_jtag_writel(ls_jtag, MJTAG_FIFO_PUSH_ALL, MJTAG_FIFO_PUSH);

    ls_jtag_flush_rx(ls_jtag);
    reinit_completion(&ls_jtag->done);
    ls_jtag_writel(ls_jtag, MJTAG_INTR_ALL, MJTAG_INTR_CLR);
    ls_jtag_writel(ls_jtag, MJTAG_INTR_RX_FIFO_ALMOST_FULL | MJTAG_INTR_RX_FIFO_FULL, MJTAG_INTR_MSK);
    ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_START, MJTAG_FIFO_WREN);

    ret = ls_jtag_wait_done(ls_jtag);
    if (ret)
        return ret;

    ls_jtag_readl(ls_jtag, MJTAG_TDO);
    ls_jtag_writel(ls_jtag, MJTAG_INTR_ALL, MJTAG_INTR_CLR);

    return 0;
}

static int ls_jtag_status_set(struct jtag *jtag, struct jtag_tap_state *tapstate)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    enum jtag_tapstate from, to;
    const struct tms_cycle *tms;
    int ret;

    mutex_lock(&ls_jtag->xfer_lock);

    from = tapstate->from;
    to = tapstate->endstate;

    if (from == JTAG_STATE_CURRENT)
        from = ls_jtag->current_state;

    /* check state */
    if (to == ls_jtag->current_state) {
        dev_dbg(ls_jtag->dev, "status_set: to == current_state (%d)\n", to);
        ret = 0;
        goto out;
    }

    if (tapstate->reset || to == JTAG_STATE_TLRESET) {
        /* reset tap */
        tms = &_tms_cycle_lookup[ls_jtag->current_state][JTAG_STATE_TLRESET];
        to = JTAG_STATE_TLRESET;
    } else {
        if (from != ls_jtag->current_state) {
            dev_err(ls_jtag->dev, "status_set: from (%d) != current_state (%d)\n", from, ls_jtag->current_state);
            ret = -EINVAL;
            goto out;
        }
        tms = &_tms_cycle_lookup[from][to];
    }

    if (tms->count == 0) {
        dev_err(ls_jtag->dev, "status_set: no path from %d to %d\n", from, to);
        ret = -EINVAL;
        goto out;
    }

    ret = __ls_jtag_status_set(ls_jtag, tms);
    if (!ret)
        /* update current state */
        ls_jtag->current_state = to;

out:
    mutex_unlock(&ls_jtag->xfer_lock);
    return ret;
}

static int ls_jtag_shift_padding(struct ls_jtag *ls_jtag, u8 data, u32 bit_len, bool last_bit)
{
    u32 remain = bit_len;
    int ret;

    while (remain > 0) {
        u32 nframes = min_t(u32, (remain + 31) / 32, MJTAG_FIFO_DEPTH);
        u32 i;

        ls_jtag_writel(ls_jtag, nframes - 1, MJTAG_TDO_FT);
        ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_WRITE, MJTAG_FIFO_WREN);

        for (i = 0; i < nframes; i++) {
            u32 dw  = min_t(u32, remain, 32);
            u32 tms = 0;
            u32 tdi = data ? (1u << dw) - 1 : 0;

            if (remain == dw && last_bit)
                tms = BIT(dw - 1);

            ls_jtag_writel(ls_jtag, tms, MJTAG_TMS);
            ls_jtag_writel(ls_jtag, tdi, MJTAG_TDI);
            ls_jtag_writel(ls_jtag, dw, MJTAG_DW);
            ls_jtag_writel(ls_jtag, MJTAG_FIFO_PUSH_ALL, MJTAG_FIFO_PUSH);
            remain -= dw;
        }
        ls_jtag_flush_rx(ls_jtag);
        reinit_completion(&ls_jtag->done);
        ls_jtag_writel(ls_jtag, MJTAG_INTR_ALL, MJTAG_INTR_CLR);
        ls_jtag_writel(ls_jtag, MJTAG_INTR_RX_FIFO_ALMOST_FULL | MJTAG_INTR_RX_FIFO_FULL, MJTAG_INTR_MSK);
        ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_START, MJTAG_FIFO_WREN);

        ret = ls_jtag_wait_done(ls_jtag);
        if (ret)
            return ret;

        for (i = 0; i < nframes; i++)
            ls_jtag_readl(ls_jtag, MJTAG_TDO);

        ls_jtag_writel(ls_jtag, MJTAG_INTR_ALL, MJTAG_INTR_CLR);
    }
    return 0;
}

static int ls_jtag_shift_data(struct ls_jtag *ls_jtag, u8 *buf, u32 bit_len, u8 dir, bool last_bit)
{
    u32 remain = bit_len;
    u32 boff = 0;
    int ret;

    while (remain > 0) {
        u32 nframes   = min_t(u32, (remain + 31) / 32, MJTAG_FIFO_DEPTH);
        u8 frame_nb[16];
        u32 batch_off = boff;
        u32 i;

        ls_jtag_writel(ls_jtag, nframes - 1, MJTAG_TDO_FT);
        ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_WRITE, MJTAG_FIFO_WREN);

        for (i = 0; i < nframes; i++) {
            u32 dw = min_t(u32, remain, 32);
            u32 nbytes = DIV_ROUND_UP(dw, 8);
            u32 tms = 0;
            u32 tdi = 0;

            frame_nb[i] = nbytes;

            if (remain == dw && last_bit)
                tms = BIT(dw - 1);

            if (dir & JTAG_WRITE_XFER)
                memcpy(&tdi, buf + boff, nbytes);

            ls_jtag_writel(ls_jtag, tms, MJTAG_TMS);
            ls_jtag_writel(ls_jtag, tdi, MJTAG_TDI);
            ls_jtag_writel(ls_jtag, dw, MJTAG_DW);
            ls_jtag_writel(ls_jtag, MJTAG_FIFO_PUSH_ALL, MJTAG_FIFO_PUSH);

            boff += nbytes;
            remain -= dw;
        }
        ls_jtag_flush_rx(ls_jtag);
        reinit_completion(&ls_jtag->done);
        ls_jtag_writel(ls_jtag, MJTAG_INTR_ALL, MJTAG_INTR_CLR);
        ls_jtag_writel(ls_jtag, MJTAG_INTR_RX_FIFO_ALMOST_FULL | MJTAG_INTR_RX_FIFO_FULL, MJTAG_INTR_MSK);
        ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_START, MJTAG_FIFO_WREN);

        ret = ls_jtag_wait_done(ls_jtag);
        if (ret)
            return ret;

        for (i = 0; i < nframes; i++) {
            u32 tdo = ls_jtag_readl(ls_jtag, MJTAG_TDO);
            if (dir & JTAG_READ_XFER)
                memcpy(buf + batch_off, &tdo, frame_nb[i]);
            batch_off += frame_nb[i];
        }

        ls_jtag_writel(ls_jtag, MJTAG_INTR_ALL, MJTAG_INTR_CLR);
    }
    return 0;
}

static int ls_jtag_ir_scan(struct ls_jtag *ls_jtag, struct jtag_xfer *xfer, u8 *xfer_data)
{
    union pad_config pad = { .int_value = xfer->padding };
    int ret;

    ret = ls_jtag_shift_padding(ls_jtag, pad.pad_data, pad.pre_pad_number, false);
    if (ret)
        return ret;
    ret = ls_jtag_shift_data(ls_jtag, xfer_data, xfer->length, xfer->direction, pad.post_pad_number == 0 ? true : false);
    if (ret)
        return ret;
    ret = ls_jtag_shift_padding(ls_jtag, pad.pad_data, pad.post_pad_number, true);
    if (ret)
        return ret;
    ls_jtag->current_state = JTAG_STATE_EXIT1IR;

    return 0;
}

static int ls_jtag_dr_scan(struct ls_jtag *ls_jtag, struct jtag_xfer *xfer, u8 *xfer_data)
{
    union pad_config pad = { .int_value = xfer->padding };
    int ret;

    ret = ls_jtag_shift_padding(ls_jtag, pad.pad_data, pad.pre_pad_number, false);
    if (ret)
        return ret;
    ret = ls_jtag_shift_data(ls_jtag, xfer_data, xfer->length, xfer->direction, pad.post_pad_number == 0 ? true : false);
    if (ret)
        return ret;
    ret = ls_jtag_shift_padding(ls_jtag, pad.pad_data, pad.post_pad_number, true);
    if (ret)
        return ret;
    ls_jtag->current_state = JTAG_STATE_EXIT1DR;

    return 0;
}

static int ls_jtag_xfer(struct jtag *jtag, struct jtag_xfer *xfer, u8 *xfer_data)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    enum jtag_tapstate from = xfer->from;
    enum jtag_tapstate to = xfer->type == JTAG_SIR_XFER ? JTAG_STATE_SHIFTIR : JTAG_STATE_SHIFTDR;
    int ret;

    mutex_lock(&ls_jtag->xfer_lock);

    if (from == JTAG_STATE_CURRENT)
        from = ls_jtag->current_state;

    if (from != ls_jtag->current_state) {
        dev_err(ls_jtag->dev, "xfer: from (%d) != current (%d)\n", from, ls_jtag->current_state);
        ret = -EINVAL;
        goto out;
    }

    if (to != ls_jtag->current_state) {
        const struct tms_cycle *t;
        t = &_tms_cycle_lookup[ls_jtag->current_state][to];
        if (t->count == 0) {
            dev_err(ls_jtag->dev, "xfer: no path from %d to %d\n", ls_jtag->current_state, to);
            ret = -EINVAL;
            goto out;
        }
        ret = __ls_jtag_status_set(ls_jtag, t);
        if (ret)
            goto out;
        ls_jtag->current_state = to;
    }

    if (xfer->type == JTAG_SIR_XFER)
        ret = ls_jtag_ir_scan(ls_jtag, xfer, xfer_data);
    else
        ret = ls_jtag_dr_scan(ls_jtag, xfer, xfer_data);
    if (ret)
        goto out;

    /* goto endstate */
    if (xfer->endstate != ls_jtag->current_state) {
        const struct tms_cycle *t;
        t = &_tms_cycle_lookup[ls_jtag->current_state][xfer->endstate];
        if (t->count == 0) {
            dev_err(ls_jtag->dev, "xfer: no path from %d to %d\n", ls_jtag->current_state, xfer->endstate);
            ret = -EINVAL;
            goto out;
        }
        ret = __ls_jtag_status_set(ls_jtag, t);
        if (ret)
            goto out;
        ls_jtag->current_state = xfer->endstate;
    }

    ret = 0;
out:
    mutex_unlock(&ls_jtag->xfer_lock);
    return ret;
}

static int ls_jtag_mode_set(struct jtag *jtag, struct jtag_mode *jtag_mode)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);

    if (jtag_mode->feature == 1)
    {
        if (jtag_mode->mode == 0) {
            ls_jtag_out_enable(ls_jtag);
        } else if (jtag_mode->mode == 1) {
            ls_jtag_out_disable(ls_jtag);
        }
    }

    return 0;
}

static int ls_jtag_trst_set(struct jtag *jtag, u32 active)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);

    /*
     * Register: BIT(0)=1 → TRST high (released)
     *           BIT(0)=0 → TRST low  (asserted, target in reset)
     * active != 0: assert reset (pull TRST low)
     * active == 0: release reset (drive TRST high)
     */
    ls_jtag_writel(ls_jtag, active ? 0 : MJTAG_TRST_CTL, MJTAG_TRST);

    if (active)
        ls_jtag->current_state = JTAG_STATE_TLRESET;

    return 0;
}

static int ls_jtag_bitbang(struct jtag *jtag, struct bitbang_packet *bitbang,
                           struct tck_bitbang *bitbang_data)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    u32 remain = bitbang->length;
    u32 idx = 0;
    int ret = 0;

    mutex_lock(&ls_jtag->xfer_lock);

    while (remain > 0) {
        u32 batch = min_t(u32, remain, MJTAG_FIFO_DEPTH);
        u32 i;

        ls_jtag_writel(ls_jtag, batch - 1, MJTAG_TDO_FT);
        ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_WRITE, MJTAG_FIFO_WREN);

        for (i = 0; i < batch; i++) {
            ls_jtag_writel(ls_jtag, bitbang_data[idx + i].tms, MJTAG_TMS);
            ls_jtag_writel(ls_jtag, bitbang_data[idx + i].tdi, MJTAG_TDI);
            ls_jtag_writel(ls_jtag, 1, MJTAG_DW);
            ls_jtag_writel(ls_jtag, MJTAG_FIFO_PUSH_ALL, MJTAG_FIFO_PUSH);
        }
        ls_jtag_flush_rx(ls_jtag);
        reinit_completion(&ls_jtag->done);
        ls_jtag_writel(ls_jtag, MJTAG_INTR_ALL, MJTAG_INTR_CLR);
        ls_jtag_writel(ls_jtag, MJTAG_INTR_RX_FIFO_ALMOST_FULL | MJTAG_INTR_RX_FIFO_FULL, MJTAG_INTR_MSK);
        ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_START, MJTAG_FIFO_WREN);

        ret = ls_jtag_wait_done(ls_jtag);
        if (ret)
            break;

        for (i = 0; i < batch; i++)
            bitbang_data[idx + i].tdo = ls_jtag_readl(ls_jtag, MJTAG_TDO) & 1;

        ls_jtag_writel(ls_jtag, MJTAG_INTR_ALL, MJTAG_INTR_CLR);

        idx += batch;
        remain -= batch;
    }

    mutex_unlock(&ls_jtag->xfer_lock);
    return ret;
}

static int ls_jtag_enable(struct jtag *jtag)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);

    ls_jtag_writel(ls_jtag, MJTAG_INTR_RX_FIFO_ALMOST_FULL | MJTAG_INTR_RX_FIFO_FULL, MJTAG_INTR_MSK);
    /* clear interrupts */
    ls_jtag_writel(ls_jtag, MJTAG_INTR_ALL, MJTAG_INTR_CLR);
    /* assert trst */
    ls_jtag_writel(ls_jtag, 1, MJTAG_TRST);
    /* set frequency */
    ls_jtag_freq_set(jtag, 1000000);
    /* jtag mode */
    ls_jtag_out_enable(ls_jtag);

    ls_jtag->current_state = JTAG_STATE_TLRESET;

    return 0;
}

static int ls_jtag_disable(struct jtag *jtag)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    /* disable jtag mode，set h-z */
    ls_jtag_out_disable(ls_jtag);

    return 0;
}

static const struct jtag_ops ls_jtag_ops = {
    .freq_get   = ls_jtag_freq_get,
    .freq_set   = ls_jtag_freq_set,
    .status_get = ls_jtag_status_get,
    .status_set = ls_jtag_status_set,
    .xfer       = ls_jtag_xfer,
    .mode_set   = ls_jtag_mode_set,
    .trst_set   = ls_jtag_trst_set,
    .bitbang    = ls_jtag_bitbang,
    .enable     = ls_jtag_enable,
    .disable    = ls_jtag_disable,
};

static int ls_jtag_hw_init(struct platform_device *pdev, struct ls_jtag *ls_jtag)
{
    struct device *dev = &pdev->dev;
    struct resource *res;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(dev, "no memory resource\n");
        return -ENODEV;
    }
    ls_jtag->base = devm_ioremap_resource(dev, res);
    if (IS_ERR(ls_jtag->base))
        return PTR_ERR(ls_jtag->base);

    ls_jtag->clk = devm_clk_get(dev, "jtag-master-clk");
    if (IS_ERR(ls_jtag->clk)) {
        dev_err(ls_jtag->dev, "jtag-master-clk get failed\n");
        return PTR_ERR(ls_jtag->clk);
    }

    clk_prepare_enable(ls_jtag->clk);

    ls_jtag->irq = platform_get_irq(pdev, 0);
    if (ls_jtag->irq < 0) {
        dev_err(dev, "no IRQ available\n");
        return ls_jtag->irq;
    }

    ls_jtag->irq = devm_request_irq(dev, ls_jtag->irq, ls_jtag_isr, 0, DEVICE_NAME, ls_jtag);
    if (ls_jtag->irq < 0) {
        dev_err(dev, "request_irq failed: %d\n", ls_jtag->irq);
        return ls_jtag->irq;
    }

    ls_jtag_out_disable(ls_jtag);

    dev_info(dev, "JTAG master at 0x%pa initialized\n", &res->start);
    return 0;
}

static void ls_jtag_hw_deinit(struct ls_jtag *ls_jtag)
{
    /* TODO: mask interrupts, assert TRST, disable controller */
    clk_disable_unprepare(ls_jtag->clk);
}

static ssize_t debug_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t debug_cmd_show(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR_RW(debug_cmd);

static int ls_jtag_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct ls_jtag *ls_jtag;
    struct jtag *jtag;
    int ret;

    jtag = jtag_alloc(dev, sizeof(*ls_jtag), &ls_jtag_ops);
    if (!jtag)
        return -ENOMEM;

    ls_jtag = jtag_priv(jtag);
    ls_jtag->dev = dev;

    init_completion(&ls_jtag->done);
    mutex_init(&ls_jtag->xfer_lock);

    ret = ls_jtag_hw_init(pdev, ls_jtag);
    if (ret)
        goto err_free;

    ret = devm_jtag_register(dev, jtag);
    if (ret) {
        dev_err(dev, "jtag_register failed: %d\n", ret);
        goto err_deinit;
    }

    ls_jtag_enable(jtag);

    platform_set_drvdata(pdev, jtag);
    /* sysfs init */
    device_create_file(&pdev->dev, &dev_attr_debug_cmd);

    dev_info(dev, "LS JTAG master probed\n");
    return 0;

err_deinit:
    ls_jtag_hw_deinit(ls_jtag);
err_free:
    jtag_free(jtag);
    return ret;
}

static void ls_jtag_remove(struct platform_device *pdev)
{
    struct jtag *jtag = platform_get_drvdata(pdev);
    struct ls_jtag *ls_jtag = jtag_priv(jtag);

    device_remove_file(&pdev->dev, &dev_attr_debug_cmd);

    ls_jtag_hw_deinit(ls_jtag);
    jtag_free(jtag);
}

static const struct of_device_id ls_jtag_match[] = {
    { .compatible = "linkedsemi,ls-jtag-master" },
    {}
};
MODULE_DEVICE_TABLE(of, ls_jtag_match);

static struct platform_driver ls_jtag_driver = {
    .driver = {
        .name       = DEVICE_NAME,
        .of_match_table = ls_jtag_match,
    },
    .probe  = ls_jtag_probe,
    .remove = ls_jtag_remove,
};
module_platform_driver(ls_jtag_driver);

/* debug commands */
struct jtag_debug_cmd
{
    const char *name;
    int (*handler)(struct jtag *jtag, int argc, char **argv);
    const char *help;
};

/*
 * cmd_clk — generate N TCK cycles with TMS=0 (stay in current state).
 * Usage: echo "clk <N>" > debug_cmd
 *   N: number of TCK cycles (1 ~ 1008, limited by 16 frames × 63 max each)
 */
/*
 * cmd_clk — generate N TCK cycles with configurable TMS/TDI.
 * Usage: echo "clk <N> [tms] [tdi]" > debug_cmd
 *   N:   cycles (1~512)
 *   tms: TMS pattern, hex, default 0
 *   tdi: TDI pattern, hex, default 0
 */
static int cmd_clk(struct jtag *jtag, int argc, char **argv)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    u32 total, remain, chunk;
    u32 tms = 0, tdi = 0;
    int ret;

    if (argc < 2 || argc > 4)
        return -EINVAL;

    ret = kstrtou32(argv[1], 0, &total);
    if (ret || total == 0)
        return -EINVAL;
    if (argc >= 3) {
        ret = kstrtou32(argv[2], 0, &tms);
        if (ret)
            return -EINVAL;
    }
    if (argc >= 4) {
        ret = kstrtou32(argv[3], 0, &tdi);
        if (ret)
            return -EINVAL;
    }

    dev_info(ls_jtag->dev, "clk: %u cycles, TMS=0x%08x, TDI=0x%08x\n",
             total, tms, tdi);

    ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_WRITE, MJTAG_FIFO_WREN);

    ls_jtag_writel(ls_jtag, MJTAG_INTR_RX_FIFO_ALMOST_FULL | MJTAG_INTR_RX_FIFO_FULL, MJTAG_INTR_MSK);
    ls_jtag_writel(ls_jtag, 1, MJTAG_TDO_FT);

    /* First frame: use caller's TMS/TDI. Remaining frames: TMS=0, TDI hold. */
    remain = total;
    while (remain > 0) {
        chunk = min_t(u32, remain, MJTAG_FRAME_MAX);
        ls_jtag_writel(ls_jtag, tms, MJTAG_TMS);
        ls_jtag_writel(ls_jtag, tdi, MJTAG_TDI);
        ls_jtag_writel(ls_jtag, chunk, MJTAG_DW);
        ls_jtag_writel(ls_jtag, MJTAG_FIFO_PUSH_ALL, MJTAG_FIFO_PUSH);
        remain -= chunk;
        tms = 0;  /* subsequent frames keep TMS=0 */
    }

    ls_jtag_writel(ls_jtag, MJTAG_FIFO_WREN_START, MJTAG_FIFO_WREN);
    return 0;
}

/*
 * cmd_trst — read or write TRST_CTL register.
 * Usage: echo "trst"       > debug_cmd  → read current value
 *        echo "trst <val>" > debug_cmd  → write val (0=assert reset, 1=release)
 */
static int cmd_trst(struct jtag *jtag, int argc, char **argv)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    u32 val;

    if (argc == 1) {
        val = ls_jtag_readl(ls_jtag, MJTAG_TRST);
        dev_info(ls_jtag->dev, "TRST = %u (%s)\n", val,
                 val & MJTAG_TRST_CTL ? "HIGH / released" : "LOW / asserted");
    } else if (argc == 2) {
        if (kstrtou32(argv[1], 0, &val))
            return -EINVAL;
        val = val ? MJTAG_TRST_CTL : 0;
        ls_jtag_writel(ls_jtag, val, MJTAG_TRST);
        dev_info(ls_jtag->dev, "TRST set to %u (%s)\n", val,
                 val ? "HIGH / released" : "LOW / asserted");
    } else {
        return -EINVAL;
    }
    return 0;
}

/*
 * cmd_state — read current TAP state, or navigate to a target state.
 * Usage: echo "state"           > debug_cmd  → read current state
 *        echo "state <target>"  > debug_cmd  → navigate to target
 * Target: tlreset idle seldr capdr sdr ex1dr pdr ex2dr upddr
 *         selir capir sir ex1ir pir ex2ir updir
 */
static int cmd_state(struct jtag *jtag, int argc, char **argv)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    static const char * const names[] = {
        [JTAG_STATE_TLRESET]   = "reset",
        [JTAG_STATE_IDLE]      = "idle",
        [JTAG_STATE_SELECTDR]  = "seldr",
        [JTAG_STATE_CAPTUREDR] = "cdr",
        [JTAG_STATE_SHIFTDR]   = "sdr",
        [JTAG_STATE_EXIT1DR]   = "ex1dr",
        [JTAG_STATE_PAUSEDR]   = "pdr",
        [JTAG_STATE_EXIT2DR]   = "ex2dr",
        [JTAG_STATE_UPDATEDR]  = "udr",
        [JTAG_STATE_SELECTIR]  = "selir",
        [JTAG_STATE_CAPTUREIR] = "cir",
        [JTAG_STATE_SHIFTIR]   = "sir",
        [JTAG_STATE_EXIT1IR]   = "ex1ir",
        [JTAG_STATE_PAUSEIR]   = "pir",
        [JTAG_STATE_EXIT2IR]   = "ex2ir",
        [JTAG_STATE_UPDATEIR]  = "uir",
    };
    enum jtag_tapstate target;
    int i, matched;

    if (argc == 1) {
        dev_info(ls_jtag->dev, "current state: %s (%d)\n",
                 names[ls_jtag->current_state], ls_jtag->current_state);
        return 0;
    }

    if (argc != 2)
        return -EINVAL;

    /* Match target state by name */
    matched = -1;
    for (i = 0; i < JTAG_STATE_CURRENT; i++) {
        if (names[i] && !strcmp(argv[1], names[i])) {
            matched = i;
            break;
        }
    }

    if (matched < 0) {
        dev_info(ls_jtag->dev,
                 "unknown state '%s'. Available: reset idle seldr cdr sdr ex1dr pdr ex2dr udr selir cir sir ex1ir pir ex2ir uir\n",
                 argv[1]);
        return -EINVAL;
    }
    target = matched;

    ls_jtag_status_set(jtag, &(struct jtag_tap_state){ .from = ls_jtag->current_state, .endstate = target });

    return 0;
}

/*
 * cmd_irq — dump all interrupt-related registers.
 * Usage: echo "irq" > debug_cmd
 */
static int cmd_irq(struct jtag *jtag, int argc, char **argv)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    u32 msk, stt, raw;

    msk = ls_jtag_readl(ls_jtag, MJTAG_INTR_MSK);
    stt = ls_jtag_readl(ls_jtag, MJTAG_INTR_STT);
    raw = ls_jtag_readl(ls_jtag, MJTAG_INTR_RAW);

    dev_info(ls_jtag->dev,
             "IRQ:  msk=0x%02x  stt=0x%02x  raw=0x%02x\n"
             "  TX_FIFO_FULL      msk=%d stt=%d raw=%d\n"
             "  TX_FIFO_EMPTY     msk=%d stt=%d raw=%d\n"
             "  RX_FIFO_FULL      msk=%d stt=%d raw=%d\n"
             "  RX_FIFO_EMPTY     msk=%d stt=%d raw=%d\n"
             "  RX_ALMOST_FULL    msk=%d stt=%d raw=%d\n",
             msk, stt, raw,
             !!(msk & BIT(0)), !!(stt & BIT(0)), !!(raw & BIT(0)),
             !!(msk & BIT(1)), !!(stt & BIT(1)), !!(raw & BIT(1)),
             !!(msk & BIT(2)), !!(stt & BIT(2)), !!(raw & BIT(2)),
             !!(msk & BIT(3)), !!(stt & BIT(3)), !!(raw & BIT(3)),
             !!(msk & BIT(4)), !!(stt & BIT(4)), !!(raw & BIT(4)));
    return 0;
}

/*
 * cmd_xfer — test IR/DR scan via ls_jtag_xfer().
 * Usage: echo "xfer <ir|dr> <r|w|rw> <len> [value] [pre] [post] [fill]" > debug_cmd
 *   pre:  pre-pad bits  (default 0)
 *   post: post-pad bits (default 0)
 *   fill: pad fill data, 0 or 1 (default 0)
 */
static int cmd_xfer(struct jtag *jtag, int argc, char **argv)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    struct jtag_xfer xfer = {0};
    union pad_config *pad = (union pad_config *)&xfer.padding;
    u8 *buf;
    u32 len, bufsz, val = 0;
    int ret;

    {
        int expect = 4 + (xfer.direction & JTAG_WRITE_XFER ? 4 : 3);
        if (argc < 4 || argc > expect)
            return -EINVAL;
    }

    /* type */
    if (!strcmp(argv[1], "ir"))
        xfer.type = JTAG_SIR_XFER;
    else if (!strcmp(argv[1], "dr"))
        xfer.type = JTAG_SDR_XFER;
    else
        return -EINVAL;

    /* direction */
    if (!strcmp(argv[2], "r"))
        xfer.direction = JTAG_READ_XFER;
    else if (!strcmp(argv[2], "w"))
        xfer.direction = JTAG_WRITE_XFER;
    else if (!strcmp(argv[2], "rw"))
        xfer.direction = JTAG_READ_WRITE_XFER;
    else
        return -EINVAL;

    /* length */
    ret = kstrtou32(argv[3], 0, &len);
    if (ret || len == 0)
        return -EINVAL;
    xfer.length = len;

    bufsz = DIV_ROUND_UP(len, 8);
    buf = kzalloc(bufsz, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    /*
     * Optional args: [val] [pre] [post] [pad_data]
     *   READ:         skip val, so 4 args total
     *   WRITE / RW:   val first, then 5 args total
     */
    {
        int ai = 4;
        bool is_write = (xfer.direction & JTAG_WRITE_XFER) != 0;

        if (is_write && argc > ai) {
            ret = kstrtou32(argv[ai++], 0, &val);
            if (ret) goto out_free;
            *(u32 *)buf = val;
        }
        if (argc > ai) {
            u32 tmp;
            ret = kstrtou32(argv[ai++], 0, &tmp);
            if (ret) goto out_free;
            pad->pre_pad_number = tmp;
        }
        if (argc > ai) {
            u32 tmp;
            ret = kstrtou32(argv[ai++], 0, &tmp);
            if (ret) goto out_free;
            pad->post_pad_number = tmp;
        }
        if (argc > ai) {
            u32 tmp;
            ret = kstrtou32(argv[ai++], 0, &tmp);
            if (ret) goto out_free;
            pad->pad_data = tmp;
        }
    }

    dev_info(ls_jtag->dev, "xfer: %s %s len=%u val=0x%x pre=%u post=%u pad_data=%u\n",
             argv[1], argv[2], len, val,
             pad->pre_pad_number, pad->post_pad_number, pad->pad_data);

    xfer.from     = JTAG_STATE_CURRENT;
    xfer.endstate = JTAG_STATE_IDLE;
    xfer.tdio     = (u64)(uintptr_t)buf;

    ret = ls_jtag_xfer(jtag, &xfer, buf);
    if (ret) {
        dev_err(ls_jtag->dev, "xfer failed: %d\n", ret);
        goto out_free;
    }

    if (xfer.direction & JTAG_READ_XFER) {
        u32 words = DIV_ROUND_UP(len, 32);
        u32 i;
        for (i = 0; i < words; i++)
            dev_info(ls_jtag->dev, "xfer done: TDO[%u] = 0x%08x\n",
                     i, ((u32 *)buf)[i]);
    } else {
        dev_info(ls_jtag->dev, "xfer done\n");
    }

out_free:
    kfree(buf);
    return ret;
}

/*
 * cmd_bitbang — test bitbang with specified TMS/TDI pattern.
 * Usage: echo "bitbang <tms> <tdi> <N>" > debug_cmd
 *   tms: TMS pattern, hex (LSB first, e.g. 0x7 = 111)
 *   tdi: TDI pattern, hex (LSB first)
 *   N:   number of cycles (1~16)
 */
static int cmd_bitbang(struct jtag *jtag, int argc, char **argv)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);
    struct bitbang_packet bitbang;
    struct tck_bitbang *bb;
    u32 tms_pat, tdi_pat, n, i;
    int ret;

    if (argc != 4)
        return -EINVAL;

    ret = kstrtou32(argv[1], 0, &tms_pat);
    if (ret) return -EINVAL;
    ret = kstrtou32(argv[2], 0, &tdi_pat);
    if (ret) return -EINVAL;
    ret = kstrtou32(argv[3], 0, &n);
    if (ret || n == 0 || n > 512)
        return -EINVAL;

    bb = kcalloc(n, sizeof(*bb), GFP_KERNEL);
    if (!bb)
        return -ENOMEM;

    for (i = 0; i < n; i++) {
        bb[i].tms = (tms_pat >> i) & 1;
        bb[i].tdi = (tdi_pat >> i) & 1;
    }
    bitbang.length = n;
    bitbang.data   = bb;

    ret = ls_jtag_bitbang(jtag, &bitbang, bb);
    if (ret) {
        dev_err(ls_jtag->dev, "bitbang failed: %d\n", ret);
        kfree(bb);
        return ret;
    }

    for (i = 0; i < n; i++)
        dev_info(ls_jtag->dev, "bitbang[%u]: tms=%u tdi=%u → tdo=%u\n",
                 i, bb[i].tms, bb[i].tdi, bb[i].tdo);

    kfree(bb);
    return 0;
}

static struct jtag_debug_cmd debug_cmds[] = {
    {"clk",     cmd_clk,     "clk <N> [tms] [tdi]  — TCK cycles with configurable TMS/TDI"},
    {"tck_run", cmd_clk,     "tck_run <N>            — generate N TCK cycles (TMS=0)"},
    {"trst",    cmd_trst,    "trst [0|1]              — read/write TRST"},
    {"state",   cmd_state,   "state [reset|idle|seldr|cdr|sdr|ex1dr|pdr|ex2dr|udr|selir|cir|sir|ex1ir|pir|ex2ir|uir]"},
    {"irq",     cmd_irq,     "irq                     — dump interrupt status"},
    {"xfer",    cmd_xfer,    "xfer <ir|dr> <r|w|rw> <len> [val] [pre] [post] [fill]  — pre/post=pad bits, fill=0/1"},
    {"bitbang", cmd_bitbang, "bitbang <tms> <tdi> <N>  — bitbang N cycles (1~512)"},
};

static int run_debug_cmd(struct jtag *jtag, char *buf)
{
    struct ls_jtag *ls_jtag = jtag_priv(jtag);

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
            return debug_cmds[i].handler(jtag, argc, argv);
    }

    dev_warn(ls_jtag->dev, "unknown cmd: %s\n", argv[0]);
    return -EINVAL;
}

static ssize_t debug_cmd_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    struct jtag *jtag = dev_get_drvdata(dev);
    char kbuf[128];

    strncpy(kbuf, buf, sizeof(kbuf));
    kbuf[sizeof(kbuf) - 1] = '\0';

    run_debug_cmd(jtag, kbuf);

    return count;
}

static ssize_t debug_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int i;
    int len = 0;

    for (i = 0; i < ARRAY_SIZE(debug_cmds); i++)
        len += sprintf(buf + len, "%s : %s\n",
                       debug_cmds[i].name,
                       debug_cmds[i].help);

    return len;
}

MODULE_DESCRIPTION("LinkedSemi JTAG Master Driver");
MODULE_AUTHOR("Zhenye Tang");
MODULE_LICENSE("GPL");
