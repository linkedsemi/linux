// SPDX-License-Identifier: GPL-2.0
/*
 * Linkedsemi LS2010 PWM controller driver.
 *
 * Period and high time are 16-bit prescaled clock cycles.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/spinlock.h>

#define DRV_NAME		"linkedsemi-pwm"
#define LS_PWM_MAX_CHANNELS	8

/* Register offsets */
#define LS_PWM_EN		0x00
#define LS_PWM_PRE_DIV		0x04
#define LS_PWM_INTR_MSK		0x10
#define LS_PWM_INTR_CLR		0x14
#define LS_PWM_INTR_STT		0x18
#define LS_PWM_INTR_RAW		0x1c
#define LS_PWM_CTRL(channel)		(0x20 + 4 * (channel))
#define LS_PWM_PARAM(channel)		(0x60 + 16 * (channel))
#define LS_PWM_PARAM_CYC		0x00
#define LS_PWM_PARAM_HIGH		0x04

/* PWM_CTRL bits */
#define LS_PWM_CTRL_POL			BIT(3)

/* PWM_EN: bit n enables, bit 16 + n disables */
#define LS_PWM_ENABLE(channel)		BIT(channel)
#define LS_PWM_DISABLE(channel)		BIT(16 + (channel))

#define LS_PWM_MAX_CYCLES	0xffff

struct ls_pwm {
	struct pwm_chip		chip;
	void __iomem		*base;
	struct clk		*clk;
	u32			input_clock_hz;
	u32			num_channels;
	spinlock_t		lock;	/* protects CTRL/PARAM/EN RMW */
};

static int ls_pwm_set_cycles(struct ls_pwm *priv, u32 channel, u32 period_cycles, u32 pulse_cycles, bool inverted)
{
	unsigned long flags;
	u32 ctrl, enable_value;

	if (channel >= priv->num_channels) {
		return -EINVAL;
	}
	if (pulse_cycles > period_cycles) {
		return -EINVAL;
	}
	if (!pulse_cycles || !period_cycles) {
		return -EINVAL;
	}
	if (period_cycles > LS_PWM_MAX_CYCLES || pulse_cycles > LS_PWM_MAX_CYCLES) {
		return -EINVAL;
	}

	spin_lock_irqsave(&priv->lock, flags);

	ctrl = readl(priv->base + LS_PWM_CTRL(channel));
	if (inverted) {
		ctrl |= LS_PWM_CTRL_POL;
	} else {
		ctrl &= ~LS_PWM_CTRL_POL;
	}
	writel(ctrl, priv->base + LS_PWM_CTRL(channel));

	writel(period_cycles, priv->base + LS_PWM_PARAM(channel) + LS_PWM_PARAM_CYC);
	writel(pulse_cycles, priv->base + LS_PWM_PARAM(channel) + LS_PWM_PARAM_HIGH);

	enable_value = readl(priv->base + LS_PWM_EN);
	enable_value |= LS_PWM_ENABLE(channel);
	writel(enable_value, priv->base + LS_PWM_EN);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int ls_pwm_get_cycles_per_sec(struct ls_pwm *priv, u32 channel, u64 *cycle)
{
	u32 prescaler;

	if (channel >= priv->num_channels) {
		return -EINVAL;
	}

	prescaler = readl(priv->base + LS_PWM_PRE_DIV);
	*cycle = priv->input_clock_hz / (prescaler + 1);
	return 0;
}

static int ls_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm, const struct pwm_state *state)
{
	struct ls_pwm *priv = pwmchip_get_drvdata(chip);
	unsigned long flags;
	u64 cycles_per_sec;
	u32 period_cycles, duty_cycles, enable_value;
	unsigned int channel = pwm->hwpwm;
	int ret;

	if (!state->enabled) {
		spin_lock_irqsave(&priv->lock, flags);
		enable_value = readl(priv->base + LS_PWM_EN);
		enable_value |= LS_PWM_DISABLE(channel);
		writel(enable_value, priv->base + LS_PWM_EN);
		spin_unlock_irqrestore(&priv->lock, flags);
		return 0;
	}

	if (!state->period) {
		return -EINVAL;
	}

	ret = ls_pwm_get_cycles_per_sec(priv, channel, &cycles_per_sec);
	if (ret) {
		return ret;
	}

	period_cycles = div_u64((u64)state->period * cycles_per_sec, NSEC_PER_SEC);
	duty_cycles = div_u64((u64)state->duty_cycle * cycles_per_sec, NSEC_PER_SEC);

	return ls_pwm_set_cycles(priv, channel, period_cycles, duty_cycles, state->polarity == PWM_POLARITY_INVERSED);
}

static int ls_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm, struct pwm_state *state)
{
	struct ls_pwm *priv = pwmchip_get_drvdata(chip);
	u64 cycles_per_sec;
	u32 period_cycles, duty_cycles, ctrl, enable_value;
	unsigned int channel = pwm->hwpwm;
	int ret;

	ret = ls_pwm_get_cycles_per_sec(priv, channel, &cycles_per_sec);
	if (ret) {
		return ret;
	}
	if (!cycles_per_sec) {
		return -EINVAL;
	}

	period_cycles = readl(priv->base + LS_PWM_PARAM(channel) + LS_PWM_PARAM_CYC);
	duty_cycles = readl(priv->base + LS_PWM_PARAM(channel) + LS_PWM_PARAM_HIGH);
	ctrl = readl(priv->base + LS_PWM_CTRL(channel));
	enable_value = readl(priv->base + LS_PWM_EN);

	state->period = DIV_ROUND_UP_ULL((u64)period_cycles * NSEC_PER_SEC, cycles_per_sec);
	state->duty_cycle = DIV_ROUND_UP_ULL((u64)duty_cycles * NSEC_PER_SEC, cycles_per_sec);
	state->polarity = (ctrl & LS_PWM_CTRL_POL) ? PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;
	state->enabled = !!(enable_value & LS_PWM_ENABLE(channel)) && period_cycles;

	return 0;
}

static const struct pwm_ops ls_pwm_ops = {
	.apply		= ls_pwm_apply,
	.get_state	= ls_pwm_get_state,
};

static int ls_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_chip *chip;
	struct ls_pwm *priv;
	u32 num_channels = 8, prescaler = 0, input_clock_hz = 0;
	u32 channel;
	int ret;

	of_property_read_u32(dev->of_node, "linkedsemi,num-channels", &num_channels);
	if (!num_channels || num_channels > LS_PWM_MAX_CHANNELS) {
		return dev_err_probe(dev, -EINVAL, "invalid channel count %u\n", num_channels);
	}

	of_property_read_u32(dev->of_node, "prescaler", &prescaler);
	if (prescaler > U8_MAX) {
		return dev_err_probe(dev, -EINVAL, "invalid prescaler %u\n", prescaler);
	}

	chip = devm_pwmchip_alloc(dev, num_channels, sizeof(*priv));
	if (IS_ERR(chip)) {
		return PTR_ERR(chip);
	}

	priv = pwmchip_get_drvdata(chip);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base)) {
		return PTR_ERR(priv->base);
	}

	priv->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(priv->clk)) {
		return dev_err_probe(dev, PTR_ERR(priv->clk), "failed to get clock\n");
	}

	if (priv->clk) {
		input_clock_hz = clk_get_rate(priv->clk);
	}

	if (!input_clock_hz) {
		ret = of_property_read_u32(dev->of_node, "clock-frequency", &input_clock_hz);
		if (ret) {
			return dev_err_probe(dev, ret, "missing clock rate\n");
		}
	}

	if (!input_clock_hz) {
		return dev_err_probe(dev, -EINVAL, "invalid clock rate\n");
	}

	priv->input_clock_hz = input_clock_hz;
	priv->num_channels = num_channels;

	spin_lock_init(&priv->lock);

	/* Channel ctrl, prescaler, clear status. */
	for (channel = 0; channel < priv->num_channels; channel++) {
		writel(0, priv->base + LS_PWM_CTRL(channel));
	}
	writel(prescaler, priv->base + LS_PWM_PRE_DIV);
	writel(0xffffff, priv->base + LS_PWM_INTR_CLR);

	chip->ops = &ls_pwm_ops;
	chip->atomic = true;

	ret = devm_pwmchip_add(dev, chip);
	if (ret) {
		return dev_err_probe(dev, ret, "failed to add PWM chip\n");
	}

	dev_info(dev, "Linkedsemi PWM: %u channels at %u Hz\n", num_channels, input_clock_hz);

	return 0;
}

static const struct of_device_id ls_pwm_of_match[] = {
	{ .compatible = "linkedsemi,ls-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, ls_pwm_of_match);

static struct platform_driver ls_pwm_driver = {
	.probe = ls_pwm_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ls_pwm_of_match,
	},
};
module_platform_driver(ls_pwm_driver);

MODULE_DESCRIPTION("Linkedsemi LS2010 PWM controller");
MODULE_LICENSE("GPL");
