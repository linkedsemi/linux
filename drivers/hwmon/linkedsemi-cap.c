// SPDX-License-Identifier: GPL-2.0
/*
 * Linkedsemi LS2010 CAP (frequency capture) driver.
 *
 * Exposed through hwmon as fan channels: fanN_input reports RPM computed from
 * the raw period count in a single division.
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define DRV_NAME		"linkedsemi-cap"

#define LS_CAP_V1_CHANNELS	8
#define LS_CAP_MAX_CHANNELS	16
#define LS_CAP_TIMEOUT_MS	1000

/* Tach pulses per revolution. */
#define LS_CAP_FAN_PULSES	2

/* V1 register offsets */
#define CAP_PRE_DIV		0x04
#define CAP_CNT_EN		0x08
#define CAP_INTR_MSK		0x10
#define CAP_INTR_CLR		0x14
#define CAP_INTR_STT		0x18
#define CAP_CTRL(channel)	(0x20 + 4 * (channel))
#define CAP_COUNT(channel)	(0x40 + 8 * (channel))

/* V2 register offsets */
#define CAP2_INTR1_CLR		0x14
#define CAP2_INTR1_STT		0x18
#define CAP2_INTR2_MSK		0x20
#define CAP2_INTR2_CLR		0x24
#define CAP2_INTR2_STT		0x28
#define CAP2_CTRL(channel)	(0x30 + 4 * (channel))
#define CAP2_COUNT(channel)	(0x70 + 8 * (channel))

struct ls_cap_channel {
	struct completion	capture_done;
	struct mutex		access_lock;	/* serializes reads of this channel */
	u16			high_cycles;
	u16			low_cycles;
	u8			pulse_error;
};

struct ls_cap {
	void __iomem		*reg_base;
	u32			input_clock_hz;
	u32			prescaler;
	u32			channel_count;
	u32			channel_cfg[LS_CAP_MAX_CHANNELS];
	bool			is_v2;
	struct ls_cap_channel	channel_data[LS_CAP_MAX_CHANNELS];
	spinlock_t		intr_lock;	/* protects INTR*_MSK RMW */
};

static void cap_intr_mask_atomic(struct ls_cap *cap, u32 channel, bool enable)
{
	u32 bank_channel = channel % 8;
	u32 channel_intr_mask = BIT(bank_channel) | BIT(bank_channel + 8) | BIT(bank_channel + 16);
	u32 intr_reg_offset = (cap->is_v2 && channel >= 8) ? CAP2_INTR2_MSK : CAP_INTR_MSK;
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&cap->intr_lock, flags);

	value = readl(cap->reg_base + intr_reg_offset);
	if (enable) {
		value |= channel_intr_mask;
	} else {
		value &= ~channel_intr_mask;
	}
	writel(value, cap->reg_base + intr_reg_offset);

	spin_unlock_irqrestore(&cap->intr_lock, flags);
}

static irqreturn_t ls_cap_isr(int irq, void *data)
{
	struct ls_cap *cap = data;
	u32 intr_status_bank1, intr_status_bank2 = 0;
	u32 channel;

	intr_status_bank1 = readl(cap->reg_base + (cap->is_v2 ? CAP2_INTR1_STT : CAP_INTR_STT));
	if (cap->is_v2) {
		intr_status_bank2 = readl(cap->reg_base + CAP2_INTR2_STT);
	}

	for (channel = 0; channel < cap->channel_count; channel++) {
		struct ls_cap_channel *channel_state = &cap->channel_data[channel];
		u32 intr_status = (cap->is_v2 && channel >= 8) ? intr_status_bank2 : intr_status_bank1;
		u32 bank_channel = channel % 8;
		u32 intr_clr = (cap->is_v2 && channel >= 8) ? CAP2_INTR2_CLR : CAP_INTR_CLR;
		u32 count_reg = cap->is_v2 ? CAP2_COUNT(channel) : CAP_COUNT(channel);

		if (intr_status & BIT(bank_channel)) {
			writel(BIT(bank_channel), cap->reg_base + intr_clr);
			cap_intr_mask_atomic(cap, channel, false);
			channel_state->high_cycles = readl(cap->reg_base + count_reg);
			channel_state->low_cycles = readl(cap->reg_base + count_reg + 4);
			channel_state->pulse_error = 0;
		} else if (intr_status & BIT(bank_channel + 8)) {
			writel(BIT(bank_channel + 8), cap->reg_base + intr_clr);
			cap_intr_mask_atomic(cap, channel, false);
			channel_state->pulse_error = 1;
		} else if (intr_status & BIT(bank_channel + 16)) {
			writel(BIT(bank_channel + 16), cap->reg_base + intr_clr);
			cap_intr_mask_atomic(cap, channel, false);
			channel_state->pulse_error = 2;
		} else {
			continue;
		}
		complete(&channel_state->capture_done);
	}

	return IRQ_HANDLED;
}

/* One read starts a capture and returns the measured period in ticks. */
static int ls_cap_measure(struct ls_cap *cap, u32 channel, u32 *cycles)
{
	struct ls_cap_channel *channel_state;
	u32 ctrl_reg = cap->is_v2 ? CAP2_CTRL(channel) : CAP_CTRL(channel);
	u32 total_cycles;
	unsigned long timeout;
	int ret;

	if (channel >= cap->channel_count) {
		return -ENODEV;
	}

	if (!readl(cap->reg_base + ctrl_reg)) {
		return -ENODEV;
	}

	channel_state = &cap->channel_data[channel];
	mutex_lock(&channel_state->access_lock);

	cap_intr_mask_atomic(cap, channel, true);

	timeout = wait_for_completion_timeout(&channel_state->capture_done, msecs_to_jiffies(LS_CAP_TIMEOUT_MS));
	if (!timeout) {
		cap_intr_mask_atomic(cap, channel, false);
		reinit_completion(&channel_state->capture_done);
		ret = -EAGAIN;
		goto out_unlock;
	}

	if (channel_state->pulse_error) {
		ret = -EIO;
		goto out_unlock;
	}

	total_cycles = (u32)channel_state->high_cycles + channel_state->low_cycles;
	if (!total_cycles) {
		ret = -ERANGE;
		goto out_unlock;
	}

	*cycles = total_cycles;
	ret = 0;

out_unlock:
	mutex_unlock(&channel_state->access_lock);
	return ret;
}

/* Convert the measured period in ticks to RPM. */
static u32 ls_cap_cycles_to_rpm(struct ls_cap *cap, u32 cycles)
{
	u32 divisor;
	u64 rpm;

	divisor = cycles * (cap->prescaler + 1) * LS_CAP_FAN_PULSES;
	if (!divisor) {
		return 0;
	}

	rpm = (u64)cap->input_clock_hz * 60;
	do_div(rpm, divisor);

	return rpm;
}

static int ls_cap_hwmon_read(struct device *dev, enum hwmon_sensor_types type, u32 attr, int channel, long *val)
{
	struct ls_cap *cap = dev_get_drvdata(dev);
	u32 cycles;
	int ret;

	switch (attr) {
	case hwmon_fan_input:
		ret = ls_cap_measure(cap, channel, &cycles);
		if (ret) {
			return ret;
		}
		*val = ls_cap_cycles_to_rpm(cap, cycles);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static umode_t ls_cap_hwmon_is_visible(const void *drvdata, enum hwmon_sensor_types type, u32 attr, int channel)
{
	const struct ls_cap *cap = drvdata;

	if (channel >= cap->channel_count || !cap->channel_cfg[channel]) {
		return 0;
	}

	return attr == hwmon_fan_input ? 0444 : 0;
}

static const struct hwmon_ops ls_cap_hwmon_ops = {
	.is_visible = ls_cap_hwmon_is_visible,
	.read = ls_cap_hwmon_read,
};

static const struct hwmon_channel_info *ls_cap_hwmon_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT),
	NULL
};

static const struct hwmon_chip_info ls_cap_chip_info = {
	.ops = &ls_cap_hwmon_ops,
	.info = ls_cap_hwmon_info,
};

static int ls_cap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ls_cap *cap;
	struct device *hwmon;
	u32 hardware_channels, enabled = 0;
	int dt_channels;
	u32 prescaler = 0, input_clock_hz = 0;
	u32 channel;
	int ret, irq;

	cap = devm_kzalloc(dev, sizeof(*cap), GFP_KERNEL);
	if (!cap) {
		return -ENOMEM;
	}

	cap->is_v2 = of_property_read_bool(dev->of_node, "linkedsemi,cap-v2");
	hardware_channels = cap->is_v2 ? LS_CAP_MAX_CHANNELS : LS_CAP_V1_CHANNELS;

	dt_channels = of_property_count_u32_elems(dev->of_node, "channels");
	if (dt_channels <= 0 || dt_channels > hardware_channels) {
		return dev_err_probe(dev, -EINVAL, "invalid channels property\n");
	}

	ret = of_property_read_u32_array(dev->of_node, "channels", cap->channel_cfg, dt_channels);
	if (ret) {
		return dev_err_probe(dev, ret, "failed to read channels\n");
	}

	ret = of_property_read_u32(dev->of_node, "prescaler", &prescaler);
	if (ret) {
		return dev_err_probe(dev, ret, "missing prescaler\n");
	}
	if (prescaler > U8_MAX) {
		return dev_err_probe(dev, -EINVAL, "invalid prescaler\n");
	}
	cap->prescaler = prescaler;
	cap->channel_count = hardware_channels;

	ret = of_property_read_u32(dev->of_node, "clock-frequency", &input_clock_hz);
	if (ret) {
		return dev_err_probe(dev, ret, "missing clock rate\n");
	}
	if (!input_clock_hz) {
		return dev_err_probe(dev, -EINVAL, "invalid clock rate\n");
	}
	cap->input_clock_hz = input_clock_hz;

	cap->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cap->reg_base)) {
		return PTR_ERR(cap->reg_base);
	}

	spin_lock_init(&cap->intr_lock);

	/* Counter enable, prescaler, clear status, channel ctrl. */
	writel(1, cap->reg_base + CAP_CNT_EN);
	writel(cap->prescaler, cap->reg_base + CAP_PRE_DIV);
	if (cap->is_v2) {
		writel(0xffffff, cap->reg_base + CAP2_INTR1_CLR);
		writel(0xffffff, cap->reg_base + CAP2_INTR2_CLR);
	} else {
		writel(0xffffff, cap->reg_base + CAP_INTR_CLR);
	}
	for (channel = 0; channel < cap->channel_count; channel++) {
		u32 ctrl_reg = cap->is_v2 ? CAP2_CTRL(channel) : CAP_CTRL(channel);

		writel(cap->channel_cfg[channel], cap->reg_base + ctrl_reg);
		init_completion(&cap->channel_data[channel].capture_done);
		mutex_init(&cap->channel_data[channel].access_lock);
		if (cap->channel_cfg[channel]) {
			enabled++;
		}
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		return dev_err_probe(dev, irq, "failed to get IRQ\n");
	}

	ret = devm_request_irq(dev, irq, ls_cap_isr, 0, dev_name(dev), cap);
	if (ret) {
		return dev_err_probe(dev, ret, "failed to request IRQ\n");
	}

	hwmon = devm_hwmon_device_register_with_info(dev, "ls_cap", cap, &ls_cap_chip_info, NULL);
	if (IS_ERR(hwmon)) {
		return dev_err_probe(dev, PTR_ERR(hwmon), "failed to register hwmon device\n");
	}

	dev_info(dev, "Linkedsemi CAP: %u channels (%u enabled), %u Hz input, %u pulses/rev\n",
		 hardware_channels, enabled, cap->input_clock_hz, LS_CAP_FAN_PULSES);

	return 0;
}

static const struct of_device_id ls_cap_of_match[] = {
	{ .compatible = "linkedsemi,ls-cap" },
	{ }
};
MODULE_DEVICE_TABLE(of, ls_cap_of_match);

static struct platform_driver ls_cap_driver = {
	.probe = ls_cap_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ls_cap_of_match,
	},
};
module_platform_driver(ls_cap_driver);

MODULE_DESCRIPTION("Linkedsemi LS2010 frequency capture (hwmon fan channels)");
MODULE_LICENSE("GPL");
