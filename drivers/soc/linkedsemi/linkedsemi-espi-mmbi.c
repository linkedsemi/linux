// SPDX-License-Identifier: GPL-2.0
/*
 * Linkedsemi LS2010 eSPI MMBI transport driver
 *
 * The MMBI region is a no-map reserved-memory region referenced by the
 * device tree ("memory-region"); the host accesses it over eSPI
 * through the E906 firmware
 * (soc/linkedsemi/common/espi_lpc/espi_lpc_host_mmbi.c).  The firmware
 * and this driver exchange struct ls_mmbi_msg records over a mailbox
 * channel:
 *
 *   - LS_MMBI_INFO:      sent by this driver once after initialization;
 *                        carries the physical base and size of the MMBI
 *                        region so the firmware can serve host MMBI
 *                        accesses from it.
 *   - LS_MMBI_HOST_INT:  sent by this driver to report an interrupt to
 *                        the host after writing B2H data.  The virtual
 *                        wire index/data carried in the message are
 *                        configured in the device tree ("vw-idx" /
 *                        "vw-data"); the firmware maps vw_idx 0/1 to
 *                        eSPI edge interrupts and vw_idx 6 with
 *                        vw_data 0x11 to SCI.
 *   - LS_MMBI_HOST_RWS_WR: sent by the firmware when the host wrote the
 *                        host_rws region (H2B write pointer or B2H read
 *                        pointer advanced); this driver refreshes the
 *                        channel state and wakes pollers.
 *
 * Device tree:
 *
 *   reserved-memory {
 *       mmbi_mem: mmbi-mem@d1010000 {
 *           reg = <0x0 0xd1010000 0x0 0x1000>;
 *           no-map;
 *       };
 *   };
 *
 *   mmbi0: mmbi {
 *       compatible = "linkedsemi,ls-espi-mmbi";
 *       memory-region = <&mmbi_mem>;    // MMBI region, no dynamic alloc
 *       mboxes = <&mailbox 4 12 3 12 3>; // chan4, 12B slots (sizeof
 *                                        // struct ls_mmbi_msg); must
 *                                        // match the E906 firmware's
 *                                        // mbox node exactly
 *       vw-idx = <6>;                   // VW index in MMBI_HOST_INT
 *       vw-data = <0x11>;               // VW data in MMBI_HOST_INT (SCI)
 *       status = "okay";
 *   };
 */

#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "linkedsemi-mmbi.h"

#define DEVICE_NAME			"linkedsemi-mmbi"

/*
 * Mailbox record exchanged with the E906 firmware.  Keep in sync with
 * struct mmbi_msg / enum mmbi_msg_type in
 * soc/linkedsemi/common/espi_lpc/espi_lpc_common.h.  The enum is int
 * sized on both sides.
 */
enum ls_mmbi_msg_type {
	LS_MMBI_INFO = 0,
	LS_MMBI_HOST_INT,
	LS_MMBI_HOST_RWS_WR,
};

struct ls_mmbi_msg {
	enum ls_mmbi_msg_type type;
	union {
		struct {
			u32 base;	/* C908 physical base of the MMBI region */
			u32 size;
		} info;
		struct {
			u8 vw_idx;
			u8 vw_data;
		} host_int;
		struct {
			u8 ch;
		} host_rws_wr;
	} u;
};

/* The enum is int sized on the E906 (Zephyr) side as well */
static_assert(sizeof(struct ls_mmbi_msg) == 12);

struct ls_espi_mmbi {
	struct ls_mmbi_ins ins;
	struct mbox_client mbox_cl;
	struct mbox_chan *mbox_ch;
	phys_addr_t phys;
	u8 vw_idx;
	u8 vw_data;
};

static int ls_espi_mmbi_raise_host_int(struct ls_mmbi_ins *ins)
{
	struct ls_espi_mmbi *priv = container_of(ins, struct ls_espi_mmbi, ins);
	struct ls_mmbi_msg msg = {
		.type = LS_MMBI_HOST_INT,
		.u.host_int = {
			.vw_idx = priv->vw_idx,
			.vw_data = priv->vw_data,
		},
	};
	int ret;

	ret = mbox_send_message(priv->mbox_ch, &msg);
	if (ret < 0)
		dev_err(ins->dev, "failed to send MMBI_HOST_INT: %d\n", ret);

	return ret;
}

static void ls_espi_mmbi_mbox_rx(struct mbox_client *cl, void *mssg)
{
	struct ls_espi_mmbi *priv =
		container_of(cl, struct ls_espi_mmbi, mbox_cl);
	struct ls_mmbi_msg *msg = mssg;

	switch (msg->type) {
	case LS_MMBI_HOST_RWS_WR:
		/* The host advanced a pointer in channel ch's host_rws */
		ls_mmbi_host_write_notify(&priv->ins, msg->u.host_rws_wr.ch);
		break;
	case LS_MMBI_INFO:
	case LS_MMBI_HOST_INT:
	default:
		dev_warn_ratelimited(cl->dev,
				     "unexpected MMBI message type %d\n",
				     msg->type);
		break;
	}
}

static int ls_espi_mmbi_send_info(struct ls_espi_mmbi *priv)
{
	struct ls_mmbi_msg msg = {
		.type = LS_MMBI_INFO,
		.u.info = {
			.base = (u32)priv->phys,
			.size = (u32)priv->ins.size,
		},
	};
	int ret;

	if (priv->phys > U32_MAX) {
		dev_err(priv->ins.dev,
			"MMBI region %pa above 4G, not usable by the E906\n",
			&priv->phys);
		return -ERANGE;
	}

	ret = mbox_send_message(priv->mbox_ch, &msg);
	if (ret < 0) {
		dev_err(priv->ins.dev, "failed to send MMBI_INFO: %d\n", ret);
		return ret;
	}

	dev_info(priv->ins.dev, "sent MMBI_INFO base %#x size %#x\n",
		 msg.u.info.base, msg.u.info.size);
	return 0;
}

static int ls_espi_mmbi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ls_espi_mmbi *priv;
	struct device_node *np;
	struct resource res;
	u32 vw_idx, vw_data;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* The MMBI region is a no-map reserved-memory region */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "missing 'memory-region'\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret) {
		dev_err(dev, "failed to get MMBI region resource: %d\n", ret);
		return ret;
	}

	priv->phys = res.start;
	priv->ins.size = resource_size(&res);
	priv->ins.virt = devm_ioremap(dev, res.start, priv->ins.size);
	if (!priv->ins.virt) {
		dev_err(dev, "failed to map MMBI region\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(dev->of_node, "vw-idx", &vw_idx);
	if (ret) {
		dev_err(dev, "missing 'vw-idx'\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "vw-data", &vw_data);
	if (ret) {
		dev_err(dev, "missing 'vw-data'\n");
		return ret;
	}

	/* The firmware handles VW index 0/1 (edge) and 6/0x11 (SCI) */
	if (vw_idx > 6 || (vw_idx > 1 && vw_idx != 6)) {
		dev_err(dev, "invalid 'vw-idx' %u\n", vw_idx);
		return -EINVAL;
	}

	/* Per-channel space sizes, 0 = channel not used */
	ret = of_property_read_variable_u32_array(dev->of_node, "channel-size",
						  priv->ins.channel_size,
						  1, LS_MMBI_MAX_CHANNELS);
	if (ret < 0) {
		dev_err(dev, "missing 'channel-size'\n");
		return ret;
	}

	priv->ins.dev = dev;
	priv->ins.host_int_type = LS_MMBI_HOST_INT_ESPI_VW;
	priv->ins.host_int_location = vw_idx;
	priv->ins.host_int_value = vw_data;
	priv->ins.bmc_int_type = 0;
	priv->ins.host_int_raise = ls_espi_mmbi_raise_host_int;
	priv->vw_idx = vw_idx;
	priv->vw_data = vw_data;

	ret = ls_mmbi_init(&priv->ins);
	if (ret)
		return ret;

	priv->mbox_cl.dev = dev;
	priv->mbox_cl.rx_callback = ls_espi_mmbi_mbox_rx;
	priv->mbox_cl.tx_block = false;
	priv->mbox_ch = mbox_request_channel(&priv->mbox_cl, 0);
	if (IS_ERR(priv->mbox_ch)) {
		ret = PTR_ERR(priv->mbox_ch);
		dev_err(dev, "failed to request MMBI mailbox channel: %d\n",
			ret);
		goto err_remove_mmbi;
	}

	platform_set_drvdata(pdev, priv);

	/* Announce the region only after the descriptor is initialized */
	ret = ls_espi_mmbi_send_info(priv);
	if (ret)
		goto err_free_channel;

	dev_info(dev, "eSPI MMBI ready: region %pa size %#zx, vw %u/%#x\n",
		 &priv->phys, priv->ins.size, vw_idx, vw_data);

	return 0;

err_free_channel:
	mbox_free_channel(priv->mbox_ch);
err_remove_mmbi:
	ls_mmbi_remove(&priv->ins);
	return ret;
}

static void ls_espi_mmbi_remove(struct platform_device *pdev)
{
	struct ls_espi_mmbi *priv = platform_get_drvdata(pdev);

	mbox_free_channel(priv->mbox_ch);
	ls_mmbi_remove(&priv->ins);
}

static const struct of_device_id ls_espi_mmbi_of_match[] = {
	{ .compatible = "linkedsemi,ls-espi-mmbi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ls_espi_mmbi_of_match);

static struct platform_driver ls_espi_mmbi_driver = {
	.probe = ls_espi_mmbi_probe,
	.remove = ls_espi_mmbi_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = ls_espi_mmbi_of_match,
	},
};
module_platform_driver(ls_espi_mmbi_driver);

MODULE_DESCRIPTION("Linkedsemi LS2010 eSPI MMBI transport driver");
MODULE_LICENSE("GPL");
