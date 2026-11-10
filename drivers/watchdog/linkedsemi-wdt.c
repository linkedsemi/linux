// SPDX-License-Identifier: GPL-2.0
/*
 * Linkedsemi LS2010 IWDT watchdog driver.
 *
 * The watchdog clock is expected to be enabled by the clock subsystem
 * or the bootloader; the driver only handles the IWDT register block.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/watchdog.h>

#define DRV_NAME		"linkedsemi-wdt"
#define LS_WDT_DEFAULT_TIMEOUT	60

static bool nowayout = WATCHDOG_NOWAYOUT;

/* Register offsets (IWDT v2) */
#define LS_IWDT_LOAD		0x00
#define LS_IWDT_CTRL		0x04
#define LS_IWDT_CNT		0x08
#define LS_IWDT_INTR		0x0c

/* IWDT_CTRL bits; INTR_CLR and CNT_CLR are write-1 strobes */
#define LS_IWDT_CTRL_EN		BIT(0)
#define LS_IWDT_CTRL_RST_EN	BIT(1)
#define LS_IWDT_CTRL_INTR_CLR	BIT(2)
#define LS_IWDT_CTRL_CNT_CLR	BIT(3)

struct ls_wdt {
	struct watchdog_device	wdd;
	void __iomem		*base;
	struct clk		*clk;
	u32			rate;
	u32			load;
	spinlock_t		lock;	/* protects CTRL/LOAD register sequences */
};

/* ticks = timeout_ms * rate / 1000 */
static u32 ls_wdt_load_from_timeout(struct ls_wdt *wdt, u32 timeout_ms)
{
	u64 ticks = div_u64((u64)timeout_ms * wdt->rate, 1000);

	return (ticks > U32_MAX) ? U32_MAX : (u32)ticks;
}

static int ls_wdt_start(struct watchdog_device *wdd)
{
	struct ls_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 ctrl, ctrl_value;

	if (!wdt->load) {
		return -EOPNOTSUPP;
	}

	/* Strobe bits latch; only EN/RST_EN mean "already configured". */
	ctrl = readl(wdt->base + LS_IWDT_CTRL);
	if (ctrl & (LS_IWDT_CTRL_EN | LS_IWDT_CTRL_RST_EN)) {
		return -EBUSY;
	}

	spin_lock_irqsave(&wdt->lock, flags);
	ctrl_value = LS_IWDT_CTRL_EN | LS_IWDT_CTRL_RST_EN;
	writel(ctrl_value, wdt->base + LS_IWDT_CTRL);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return 0;
}

static int ls_wdt_stop(struct watchdog_device *wdd)
{
	struct ls_wdt *wdt = watchdog_get_drvdata(wdd);

	writel(0, wdt->base + LS_IWDT_CTRL);

	return 0;
}

static int ls_wdt_ping(struct watchdog_device *wdd)
{
	struct ls_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 ctrl, ctrl_value;

	spin_lock_irqsave(&wdt->lock, flags);
	ctrl = readl(wdt->base + LS_IWDT_CTRL);
	ctrl_value = ctrl | LS_IWDT_CTRL_CNT_CLR | LS_IWDT_CTRL_INTR_CLR;
	writel(ctrl_value, wdt->base + LS_IWDT_CTRL);
	ctrl_value = ctrl & ~(LS_IWDT_CTRL_CNT_CLR | LS_IWDT_CTRL_INTR_CLR);
	writel(ctrl_value, wdt->base + LS_IWDT_CTRL);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return 0;
}

static int ls_wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout_s)
{
	struct ls_wdt *wdt = watchdog_get_drvdata(wdd);
	u64 timeout_ms = (u64)timeout_s * 1000;

	if (timeout_ms > U32_MAX) {
		timeout_ms = U32_MAX;
	}

	if (!watchdog_active(wdd)) {
		writel(0, wdt->base + LS_IWDT_CTRL);
	}

	wdt->load = ls_wdt_load_from_timeout(wdt, (u32)timeout_ms);
	writel(wdt->load, wdt->base + LS_IWDT_LOAD);

	return 0;
}

static const struct watchdog_info ls_wdt_info = {
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity	= "Linkedsemi IWDT",
};

static const struct watchdog_ops ls_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ls_wdt_start,
	.stop		= ls_wdt_stop,
	.ping		= ls_wdt_ping,
	.set_timeout	= ls_wdt_set_timeout,
};

static int ls_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ls_wdt *wdt;
	u64 timeout_ms;
	u32 ctrl;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt) {
		return -ENOMEM;
	}

	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base)) {
		return PTR_ERR(wdt->base);
	}

	wdt->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(wdt->clk)) {
		return dev_err_probe(dev, PTR_ERR(wdt->clk), "failed to get clock\n");
	}

	if (wdt->clk) {
		wdt->rate = clk_get_rate(wdt->clk);
	}

	if (!wdt->rate) {
		ret = of_property_read_u32(dev->of_node, "clock-frequency", &wdt->rate);
		if (ret) {
			return dev_err_probe(dev, ret, "missing clock rate\n");
		}
	}

	if (!wdt->rate) {
		return dev_err_probe(dev, -EINVAL, "invalid clock rate\n");
	}

	spin_lock_init(&wdt->lock);

	wdt->wdd.info		= &ls_wdt_info;
	wdt->wdd.ops		= &ls_wdt_ops;
	wdt->wdd.parent		= dev;
	wdt->wdd.min_timeout	= 1;
	wdt->wdd.max_timeout	= U32_MAX / wdt->rate;
	wdt->wdd.timeout	= LS_WDT_DEFAULT_TIMEOUT;
	if (!wdt->wdd.max_timeout) {
		return dev_err_probe(dev, -EINVAL, "clock rate too high\n");
	}

	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_stop_on_reboot(&wdt->wdd);
	watchdog_stop_on_unregister(&wdt->wdd);

	/* timeout-sec comes from DT; keep the driver default if absent. */
	ret = watchdog_init_timeout(&wdt->wdd, 0, dev);
	if (ret) {
		dev_warn(dev, "invalid timeout-sec, using %us\n", wdt->wdd.timeout);
	}

	ctrl = readl(wdt->base + LS_IWDT_CTRL);
	if (ctrl & (LS_IWDT_CTRL_EN | LS_IWDT_CTRL_RST_EN)) {
		u32 load;

		load = readl(wdt->base + LS_IWDT_LOAD);
		if (load) {
			wdt->load = load;
			wdt->wdd.timeout = clamp(DIV_ROUND_UP_ULL((u64)load, wdt->rate), wdt->wdd.min_timeout, wdt->wdd.max_timeout);
		}
		/* Adopt a bootloader-started watchdog instead of hitting
		 * ls_wdt_start()'s -EBUSY guard.
		 */
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
		dev_info(dev, "watchdog already running, timeout %us\n", wdt->wdd.timeout);
	} else {
		timeout_ms = (u64)wdt->wdd.timeout * 1000;
		if (timeout_ms > U32_MAX) {
			timeout_ms = U32_MAX;
		}

		writel(0, wdt->base + LS_IWDT_CTRL);
		wdt->load = ls_wdt_load_from_timeout(wdt, (u32)timeout_ms);
		writel(wdt->load, wdt->base + LS_IWDT_LOAD);
	}

	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret) {
		return dev_err_probe(dev, ret, "failed to register watchdog\n");
	}

	dev_info(dev, "Linkedsemi IWDT at %pR, %u Hz, timeout %us\n", &pdev->resource[0], wdt->rate, wdt->wdd.timeout);

	return 0;
}

static const struct of_device_id ls_wdt_of_match[] = {
	{ .compatible = "linkedsemi,ls-watchdog" },
	{ }
};
MODULE_DEVICE_TABLE(of, ls_wdt_of_match);

static struct platform_driver ls_wdt_driver = {
	.probe = ls_wdt_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ls_wdt_of_match,
	},
};
module_platform_driver(ls_wdt_driver);

module_param(nowayout, bool, 0444);

MODULE_DESCRIPTION("Linkedsemi LS2010 IWDT watchdog");
MODULE_LICENSE("GPL");
