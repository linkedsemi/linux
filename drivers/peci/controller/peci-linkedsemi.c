// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Linkedsemi Corporation.

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/peci.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* PECI registers */
#define PECI_INTR_MSK			0x000
#define PECI_INTR_CLR			0x004
#define PECI_INTR_STT			0x008
#define PECI_INTR_RAW			0x00c
/* Each source uses the same bit in MSK, CLR, STT and RAW. */
#define   PECI_INTR_FSM_END		BIT(0)
#define   PECI_INTR_32BYTES_END		BIT(1)

#define PECI_CTRL0			0x010
#define   PECI_PRE_DIV			GENMASK(7, 0)
#define   PECI_FLT_NUM			GENMASK(11, 8)
#define   PECI_DAT_LEN_M2		GENMASK(21, 12)
#define   PECI_DAT_WLEN_M2		GENMASK(30, 22)
#define   PECI_DAT_WT_MODE		BIT(31)

#define PECI_TXRX_REQ			0x014
/* RW: write one to start, zero to abort; hardware clears it at frame end. */
#define   PECI_TXRX_START		BIT(0)

/* 0x018 and 0x01c are reserved. */
#define PECI_A_TIM0			0x020
#define   PECI_A_BIT_CYC		GENMASK(9, 0)
#define   PECI_A_SMP_IDX		GENMASK(25, 16)
#define PECI_A_TIM1			0x024
#define   PECI_A_TGL0_IDX		GENMASK(9, 0)
#define   PECI_A_TGL1_IDX		GENMASK(25, 16)
#define PECI_M_TIM0			0x028
#define   PECI_M_BIT_CYC		GENMASK(9, 0)
#define   PECI_M_SMP_IDX		GENMASK(25, 16)
#define PECI_M_TIM1			0x02c
#define   PECI_M_TGL0_IDX		GENMASK(9, 0)
#define   PECI_M_TGL1_IDX		GENMASK(25, 16)

/* Each data bank contains eight consecutive 32-bit words. */
#define PECI_TX_DAT0			0x030
#define PECI_RX_DAT0			0x050
#define PECI_TX_H_DAT0			0x070
#define PECI_RX_H_DAT0			0x090
#define PECI_DATA_WORDS			8
#define PECI_MMIO_SIZE			(PECI_RX_H_DAT0 + PECI_DATA_WORDS * sizeof(u32))

/* PECI protocol */
#define PECI_ADDR_LEN 1
#define PECI_FCS_LEN 1
#define PECI_WRLEN_LEN 1
#define PECI_RDLEN_LEN 1

#define PECI_A_BIT_CYC_VAL 11
#define PECI_A_TGT_IDX0_VAL 3
#define PECI_M_TGT_IDX0_VAL 3
#define PECI_A_SMP_IDX_VAL 6
#define PECI_A_TGT_IDX1_VAL 8
#define PECI_M_TGT_IDX1_VAL 8

#define PECI_LINKEDSEMI_INTR_MASK (PECI_INTR_FSM_END | PECI_INTR_32BYTES_END)
#define PECI_LINKEDSEMI_WIRE_MAX 32
#define PECI_LINKEDSEMI_BITRATE_MIN 2000
#define PECI_LINKEDSEMI_BITRATE_MAX 1000000
#define PECI_LINKEDSEMI_BITRATE_DEFAULT 200000
#define PECI_LINKEDSEMI_TIMEOUT_MS_DEFAULT 1000
#define PECI_LINKEDSEMI_TIMEOUT_MS_MAX 60000

struct peci_linkedsemi_config {
	void __iomem *base;
	int irq_num;
	u32 clock_source;
	struct reset_control *reset;
	u32 cmd_timeout_ms;
};

struct peci_linkedsemi_data {
	struct completion xfer_sync_sem;
	struct peci_linkedsemi_config config;
	struct device *dev;
	/* Protects the active transfer state shared with the IRQ handler. */
	spinlock_t irq_lock;
	bool active;
	bool faulted;
	int xfer_error;
	unsigned int wire_len;
	unsigned int wire_received;
	u8 tx_wire[PECI_LINKEDSEMI_WIRE_MAX];
	u8 rx_wire[PECI_LINKEDSEMI_WIRE_MAX];
};

static u8 peci_linkedsemi_crc8(const u8 *buf, size_t len)
{
	u8 crc_result = 0;
	unsigned int bit;

	while (len--) {
		crc_result ^= *buf++;
		for (bit = 0; bit < 8; bit++)
			crc_result = (crc_result << 1) ^
				     ((crc_result & 0x80) ? 0x07 : 0);
	}
	return crc_result;
}

static void peci_wr_buf(struct peci_linkedsemi_data *dev_data, const u8 *txbuf)
{
	void __iomem *base = dev_data->config.base + PECI_TX_DAT0;
	unsigned int i;

	for (i = 0; i < PECI_DATA_WORDS; i++)
		writel(get_unaligned_le32(txbuf + i * sizeof(u32)),
		       base + i * sizeof(u32));
}

static void peci_rd_buf(struct peci_linkedsemi_data *dev_data, u8 *rxbuf)
{
	void __iomem *base = dev_data->config.base + PECI_RX_DAT0;
	unsigned int i;

	for (i = 0; i < PECI_DATA_WORDS; i++)
		put_unaligned_le32(readl(base + i * sizeof(u32)),
				   rxbuf + i * sizeof(u32));
}

static void peci_linkedsemi_receive_block(struct peci_linkedsemi_data *dev_data)
{
	u8 buf[32];
	unsigned int remaining = dev_data->wire_len - dev_data->wire_received;
	unsigned int len = min(remaining, 32U);

	if (!len) {
		dev_data->xfer_error = -EIO;
		return;
	}
	peci_rd_buf(dev_data, buf);
	memcpy(dev_data->rx_wire + dev_data->wire_received, buf, len);
	dev_data->wire_received += len;
}

static irqreturn_t peci_linkedsemi_isr(int irq, void *arg)
{
	struct peci_linkedsemi_data *dev_data = arg;
	void __iomem *base = dev_data->config.base;
	unsigned long flags;
	bool done = false;
	u32 status;

	spin_lock_irqsave(&dev_data->irq_lock, flags);
	/* STT is already qualified by MSK, where one enables the source. */
	status = readl(base + PECI_INTR_STT) & PECI_LINKEDSEMI_INTR_MASK;
	if (!status) {
		spin_unlock_irqrestore(&dev_data->irq_lock, flags);
		return IRQ_NONE;
	}
	if (dev_data->active) {
		/* A supported transaction fits entirely in the low data bank. */
		if (status == PECI_INTR_FSM_END)
			peci_linkedsemi_receive_block(dev_data);
		else
			dev_data->xfer_error = -EIO;
		done = true;
		dev_data->active = false;
		writel(0, base + PECI_INTR_MSK);
	}
	/* WO/W1C: never read-modify-write INTR_CLR. */
	writel(status, base + PECI_INTR_CLR);
	if (done)
		complete(&dev_data->xfer_sync_sem);
	spin_unlock_irqrestore(&dev_data->irq_lock, flags);
	return IRQ_HANDLED;
}

static int peci_linkedsemi_configure(struct peci_linkedsemi_data *dev_data,
				     u32 bitrate)
{
	const struct peci_linkedsemi_config *dev_config = &dev_data->config;
	void __iomem *base = dev_config->base;
	u64 divisor, bit_cycles;
	u32 div;

	if (!bitrate || !dev_config->clock_source)
		return -EINVAL;
	/*
	 * bit period = input clock period * (pre_div + 1) * (bit_cyc + 1).
	 * Sample after (smp_idx + 1) divided clocks from the bit's rising edge.
	 */
	divisor = DIV_ROUND_UP_ULL((u64)dev_config->clock_source,
				   (u64)bitrate * (PECI_A_BIT_CYC_VAL + 1));
	if (!divisor || divisor > 256)
		return -ERANGE;
	bit_cycles = divisor * (PECI_A_BIT_CYC_VAL + 1);
	/* Compare without truncating the actual rate at either boundary. */
	if (dev_config->clock_source < bit_cycles * PECI_LINKEDSEMI_BITRATE_MIN ||
	    dev_config->clock_source > bit_cycles * PECI_LINKEDSEMI_BITRATE_MAX)
		return -ERANGE;
	div = divisor - 1;

	writel(FIELD_PREP(PECI_A_BIT_CYC, PECI_A_BIT_CYC_VAL) |
		       FIELD_PREP(PECI_A_SMP_IDX, PECI_A_SMP_IDX_VAL),
	       base + PECI_A_TIM0);
	writel(FIELD_PREP(PECI_A_TGL0_IDX, PECI_A_TGT_IDX0_VAL) |
		       FIELD_PREP(PECI_A_TGL1_IDX, PECI_A_TGT_IDX1_VAL),
	       base + PECI_A_TIM1);
	writel(FIELD_PREP(PECI_M_BIT_CYC, PECI_A_BIT_CYC_VAL) |
		       FIELD_PREP(PECI_M_SMP_IDX, PECI_A_SMP_IDX_VAL),
	       base + PECI_M_TIM0);
	writel(FIELD_PREP(PECI_M_TGL0_IDX, PECI_M_TGT_IDX0_VAL) |
		       FIELD_PREP(PECI_M_TGL1_IDX, PECI_M_TGT_IDX1_VAL),
	       base + PECI_M_TIM1);
	writel(FIELD_PREP(PECI_PRE_DIV, div), base + PECI_CTRL0);
	if (FIELD_GET(PECI_PRE_DIV, readl(base + PECI_CTRL0)) != div)
		return -EIO;
	return 0;
}

static void peci_linkedsemi_quiesce(struct peci_linkedsemi_data *dev_data)
{
	unsigned long flags;

	spin_lock_irqsave(&dev_data->irq_lock, flags);
	dev_data->active = false;
	writel(0, dev_data->config.base + PECI_INTR_MSK);
	/* Flush the mask before waiting for an in-flight handler. */
	readl(dev_data->config.base + PECI_INTR_MSK);
	spin_unlock_irqrestore(&dev_data->irq_lock, flags);
	synchronize_irq(dev_data->config.irq_num);
	writel(PECI_LINKEDSEMI_INTR_MASK,
	       dev_data->config.base + PECI_INTR_CLR);
}

/* Call with the interrupt source masked and no active transfer handler. */
static int peci_linkedsemi_abort(struct peci_linkedsemi_data *dev_data)
{
	void __iomem *base = dev_data->config.base;
	u32 busy;
	int ret;

	/* Writing zero terminates the transaction and returns the FSM to idle. */
	writel(0, base + PECI_TXRX_REQ);
	ret = readl_poll_timeout(base + PECI_TXRX_REQ, busy,
				 !(busy & PECI_TXRX_START), 10, 100000);
	/* Discard status latched while the old transaction was stopping. */
	writel(PECI_LINKEDSEMI_INTR_MASK, base + PECI_INTR_CLR);
	dev_data->faulted = !!ret;
	if (ret)
		dev_err(dev_data->dev, "failed to stop PECI transaction\n");
	return ret;
}

static void peci_linkedsemi_disable(void *arg)
{
	struct peci_linkedsemi_data *dev_data = arg;

	peci_linkedsemi_quiesce(dev_data);
	peci_linkedsemi_abort(dev_data);
}

static int peci_linkedsemi_transfer(struct peci_controller *controller, u8 addr,
				    struct peci_request *req)
{
	struct peci_linkedsemi_data *dev_data =
		dev_get_drvdata(controller->dev.parent);
	const struct peci_linkedsemi_config *dev_config = &dev_data->config;
	void __iomem *base = dev_config->base;
	unsigned int tx_data_len, xfer_tx_len, xfer_rx_len, xfer_len;
	unsigned long flags;
	unsigned long timeout;
	u8 crc_result;
	u32 ctrl, busy;
	long waited;
	int ret;

	if (!req || req->tx.len > PECI_REQUEST_MAX_BUF_SIZE ||
	    req->rx.len > PECI_REQUEST_MAX_BUF_SIZE)
		return -EINVAL;
	if (!req->tx.len && req->rx.len)
		return -EINVAL;

	tx_data_len = PECI_WRLEN_LEN + PECI_RDLEN_LEN + req->tx.len;
	xfer_tx_len = PECI_ADDR_LEN + tx_data_len + PECI_FCS_LEN;
	xfer_rx_len = req->rx.len ? req->rx.len + PECI_FCS_LEN : 0;
	xfer_len = xfer_tx_len + xfer_rx_len;
	if (xfer_len > PECI_LINKEDSEMI_WIRE_MAX)
		return -EOPNOTSUPP;

	/* The PECI core holds the controller bus lock throughout the transfer. */
	if (dev_data->faulted)
		return -EIO;
	/* Wait for the previous request to self-clear before touching its banks. */
	ret = readl_poll_timeout(base + PECI_TXRX_REQ, busy,
				 !(busy & PECI_TXRX_START), 10,
				 dev_config->cmd_timeout_ms * 1000);
	if (ret) {
		peci_linkedsemi_quiesce(dev_data);
		peci_linkedsemi_abort(dev_data);
		return ret;
	}

	peci_linkedsemi_quiesce(dev_data);
	reinit_completion(&dev_data->xfer_sync_sem);
	memset(dev_data->rx_wire, 0, sizeof(dev_data->rx_wire));
	dev_data->wire_len = xfer_len;
	dev_data->wire_received = 0;
	dev_data->xfer_error = 0;

	memset(dev_data->tx_wire, 0, sizeof(dev_data->tx_wire));
	dev_data->tx_wire[0] = addr;
	dev_data->tx_wire[1] = req->tx.len;
	dev_data->tx_wire[2] = req->rx.len;
	memcpy(dev_data->tx_wire + PECI_ADDR_LEN + PECI_WRLEN_LEN + PECI_RDLEN_LEN,
	       req->tx.buf, req->tx.len);
	crc_result = peci_linkedsemi_crc8(dev_data->tx_wire,
				       PECI_ADDR_LEN + tx_data_len);
	peci_wr_buf(dev_data, dev_data->tx_wire);

	ctrl = readl(base + PECI_CTRL0);
	ctrl &= ~(PECI_DAT_LEN_M2 | PECI_DAT_WLEN_M2 | PECI_DAT_WT_MODE);
	/*
	 * The data phase excludes the address. It includes both FCS slots
	 * when a read is requested. Switch to read at the Write FCS slot,
	 * including for Ping and commands with no read payload.
	 */
	ctrl |= FIELD_PREP(PECI_DAT_LEN_M2, xfer_len - PECI_ADDR_LEN - 1) |
		FIELD_PREP(PECI_DAT_WLEN_M2, tx_data_len) | PECI_DAT_WT_MODE;
	writel(ctrl, base + PECI_CTRL0);

	spin_lock_irqsave(&dev_data->irq_lock, flags);
	dev_data->active = true;
	writel(PECI_INTR_FSM_END, base + PECI_INTR_MSK);
	writel(PECI_TXRX_START, base + PECI_TXRX_REQ);
	spin_unlock_irqrestore(&dev_data->irq_lock, flags);

	timeout = msecs_to_jiffies(dev_config->cmd_timeout_ms);
	waited = wait_for_completion_interruptible_timeout(&dev_data->xfer_sync_sem,
							   timeout);
	peci_linkedsemi_quiesce(dev_data);
	if (waited <= 0) {
		peci_linkedsemi_abort(dev_data);
		return waited ? (int)waited : -ETIMEDOUT;
	}
	ret = dev_data->xfer_error;
	if (ret || dev_data->wire_received != xfer_len) {
		peci_linkedsemi_abort(dev_data);
		return -EIO;
	}
	if (dev_data->rx_wire[xfer_tx_len - 1] != crc_result)
		return req->tx.len ? -EBADMSG : -ENODEV;
	if (req->rx.len) {
		if (peci_linkedsemi_crc8(dev_data->rx_wire + xfer_tx_len,
					 req->rx.len) !=
		    dev_data->rx_wire[xfer_len - 1])
			return -EBADMSG;
		/* Linux owns exactly rx.len bytes; never copy the trailing Read FCS. */
		memcpy(req->rx.buf, dev_data->rx_wire + xfer_tx_len,
		       req->rx.len);
	}
	return 0;
}

static const struct peci_controller_ops peci_linkedsemi_driver_api = {
	.xfer = peci_linkedsemi_transfer,
};

static int peci_linkedsemi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct peci_linkedsemi_data *dev_data;
	struct peci_linkedsemi_config *dev_config;
	struct peci_controller *controller;
	struct resource *res;
	struct clk *clk;
	unsigned long rate;
	u32 bitrate = PECI_LINKEDSEMI_BITRATE_DEFAULT;
	int ret;

	dev_data = devm_kzalloc(dev, sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data)
		return -ENOMEM;
	dev_config = &dev_data->config;
	dev_data->dev = dev;
	init_completion(&dev_data->xfer_sync_sem);
	spin_lock_init(&dev_data->irq_lock);
	platform_set_drvdata(pdev, dev_data);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < PECI_MMIO_SIZE)
		return -EINVAL;
	dev_config->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dev_config->base))
		return PTR_ERR(dev_config->base);
	dev_config->irq_num = platform_get_irq(pdev, 0);
	if (dev_config->irq_num < 0)
		return dev_config->irq_num;
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "cannot enable clock\n");
	rate = clk_get_rate(clk);
	if (!rate || rate > U32_MAX)
		return dev_err_probe(dev, -ERANGE, "invalid input clock rate\n");
	dev_config->clock_source = rate;

	device_property_read_u32(dev, "clock-frequency", &bitrate);
	dev_config->cmd_timeout_ms = PECI_LINKEDSEMI_TIMEOUT_MS_DEFAULT;
	device_property_read_u32(dev, "cmd-timeout-ms",
				 &dev_config->cmd_timeout_ms);
	if (!dev_config->cmd_timeout_ms ||
	    dev_config->cmd_timeout_ms > PECI_LINKEDSEMI_TIMEOUT_MS_MAX)
		return dev_err_probe(dev, -EINVAL, "invalid cmd-timeout-ms\n");
	dev_config->reset =
		devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(dev_config->reset))
		return dev_err_probe(dev, PTR_ERR(dev_config->reset),
				     "cannot get reset\n");
	if (dev_config->reset) {
		ret = reset_control_reset(dev_config->reset);
		if (ret)
			return dev_err_probe(dev, ret, "cannot reset PECI\n");
	}
	/* Stop any previous transaction, including on FPGA without a reset. */
	writel(0, dev_config->base + PECI_INTR_MSK);
	ret = peci_linkedsemi_abort(dev_data);
	if (ret)
		return dev_err_probe(dev, ret,
				     "controller still busy after abort\n");
	ret = peci_linkedsemi_configure(dev_data, bitrate);
	if (ret)
		return dev_err_probe(dev, ret,
				     "cannot configure PECI timing\n");
	ret = devm_request_irq(dev, dev_config->irq_num, peci_linkedsemi_isr, 0,
			       dev_name(dev), dev_data);
	if (ret)
		return dev_err_probe(dev, ret,
				     "cannot request PECI interrupt\n");
	ret = devm_add_action_or_reset(dev, peci_linkedsemi_disable, dev_data);
	if (ret)
		return ret;
	controller = devm_peci_controller_add(dev, &peci_linkedsemi_driver_api);
	if (IS_ERR(controller))
		return dev_err_probe(dev, PTR_ERR(controller),
				     "cannot add PECI controller\n");
	return 0;
}

static const struct of_device_id peci_linkedsemi_of_table[] = {
	{ .compatible = "linkedsemi,ls2010-peci" },
	{ .compatible = "linkedsemi,peci" },
	{}
};
MODULE_DEVICE_TABLE(of, peci_linkedsemi_of_table);

static struct platform_driver peci_linkedsemi_driver = {
	.probe = peci_linkedsemi_probe,
	.driver = {
		.name = "peci-linkedsemi",
		.of_match_table = peci_linkedsemi_of_table,
	},
};
module_platform_driver(peci_linkedsemi_driver);

MODULE_DESCRIPTION("Linkedsemi PECI controller");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("PECI");
