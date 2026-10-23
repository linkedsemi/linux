#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/ptr_ring.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/poll.h>
#include <uapi/linux/poll.h>
#include "pcie-linkedsemi.h"
#include "aspeed-mctp.h"
#define MCTP_RX_FIFO_CTRL_OFFSET 0x0
#define MCTP_RX_FIFO_POP_BIT BIT(6)
#define MCTP_RX_FIFO_WCLR_BIT BIT(7)
#define MCTP_RX_FIFO_RCLR_BIT BIT(8)
#define MCTP_RX_FIFO_WARN_LEVEL_OFFSET 0x3c
#define MCTP_RX_ATU_MATCH_MASK_OFFSET 0x40
#define MCTP_RX_FIFO_DATA0_OFFSET 0x44
#define MCTP_RX_DATA0_ROUTE_TYPE_MASK GENMASK(2,0)
#define MCTP_RX_DATA0_LENGTH_HIGH_MASK GENMASK(1,0)
#define MCTP_RX_FIFO_DATA1_OFFSET 0x48
#define MCTP_RX_FIFO_INT_OFFSET 0x4c
#define MCTP_RX_FIFO_INT_STT_MASK GENMASK(20,18)
#define MCTP_RX_FIFO_FULL_INT_STT_BIT BIT(18)
#define MCTP_RX_FIFO_EMPTY_INT_STT_BIT BIT(19)
#define MCTP_RX_FIFO_WARN_LEVEL_INT_STT_BIT BIT(20)
#define MCTP_RX_FIFO_FULL_INT_MASK BIT(12)
#define MCTP_RX_FIFO_EMPTY_INT_MASK BIT(13)
#define MCTP_RX_FIFO_WARN_LEVEL_INT_MASK BIT(14)
#define MCTP_RX_FIFO_LEVEL_MASK GENMASK(5,0)
#define RX_CBUF_INCR CBUF_INCR_512

#define PCIE_VDM_HDR_SIZE 16
#define LS_PCIE_MCTP_MTU 256
#define PCIE_VDM_DATA_SIZE_DW (LS_PCIE_MCTP_MTU / 4)
#define PCIE_VDM_HDR_SIZE_DW (PCIE_VDM_HDR_SIZE / 4)
#define PCIE_MCTP_MIN_PACKET_SIZE (PCIE_VDM_HDR_SIZE + 4)

#define MCTP_MSG_TYPE_CONTROL 0x00
#define MCTP_MSG_TYPE_MASK GENMASK(6, 0)
#define MCTP_TAG_OWNER BIT(3)
#define MCTP_CTRL_HDR_RQ BIT(7)
#define MCTP_CTRL_CMD_SET_ENDPOINT_ID 0x01
#define MCTP_CTRL_CMD_GET_ENDPOINT_ID 0x02
#define MCTP_CTRL_CMD_ENDPOINT_DISCOVERY 0x0c
#define MCTP_CTRL_CC_SUCCESS 0x00
#define PCIE_VDM_ROUTE_TYPE_MASK GENMASK(2, 0)

#define MCTP_EID_INFO_MAX 256
#define MCTP_PAYLOAD_SIZE(hdr) (4*(((((uint8_t *)hdr)[2] & MCTP_RX_DATA0_LENGTH_HIGH_MASK) << 8) | ((uint8_t *)hdr)[3]))
#define IATU_REGION_CTRL2_OFF_OUTBOUND_0_TAGMASK 0xff00
#define IATU_REGION_CTRL2_OFF_OUTBOUND_0_TAGPOS 8

const phys_addr_t msgd_tx_addr = 0xFFFFFFFFFFFF0000ULL; // Use the last 64KB of the 64-bit address space for MSGD

enum mctp_pcie_vdm_route_type {
	MCTP_PCIE_VDM_ROUTE_TO_RC = 0,
	MCTP_PCIE_VDM_ROUTE_BY_ID = 2,
	MCTP_PCIE_VDM_BROADCAST_FROM_RC = 3,
};

enum mctp_address_type {
	MCTP_GENERIC_ADDR_FORMAT = 0,
	MCTP_EXTENDED_ADDR_FORMAT = 1
};

union mctp_eid_data_info {
	struct aspeed_mctp_eid_info eid_info;
	struct aspeed_mctp_eid_ext_info eid_ext_info;
};

struct ls_mctp_endpoint {
	struct list_head link;
	union mctp_eid_data_info data;
};

struct mctp_pcie_payload {
	u32 payload[PCIE_VDM_DATA_SIZE_DW];
};

struct mctp_pcie_tx_packet {
    struct list_head head;
	struct mctp_pcie_payload *payload;
    dma_addr_t payload_dma_addr;
    u32 hdr[PCIE_VDM_HDR_SIZE_DW];
    u32 size;
};

struct mctp_pcie_rx_packet {
    struct list_head head;
    u32 hdr[PCIE_VDM_HDR_SIZE_DW];
    u32 payload[PCIE_VDM_DATA_SIZE_DW];
};

struct ls_pcie_mctp_client {
    struct list_head head;
    struct list_head rx_q;
    struct ls_pcie_mctp *priv;
  	wait_queue_head_t wait_queue;
    struct aspeed_mctp_type_handler_ioctl type;
    bool type_valid;
};

struct ls_pcie_mctp {
    struct device *dev;
    struct ls_pcie *pcie;
    struct dma_chan *dma_write_chan;
    struct task_struct *tx_thread;
    void *rx_circular_buffer;
    phys_addr_t rx_dma_handle;
	struct ls_pcie_mctp_client *default_client;
	wait_queue_head_t tx_thread_wait_queue;
	struct miscdevice mctp_miscdev;
    struct completion tx_completion;
	struct list_head clients;
    struct list_head tx_q;
    struct list_head rx_packet_free_list;
	spinlock_t clients_lock;
    u32 bdf_reg_offset;
    u32 tx_reg_offset;
    u32 rx_buffer_size;
    u32 rx_reg_offset;
    u16 rx_ring_count;
    u16 rx_idx;
    u8 own_eid;
    u8 outbound_atu_idx;
};

static int ls_pcie_mctp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct platform_device *pdev = to_platform_device(misc->parent);
	struct ls_pcie_mctp *priv = platform_get_drvdata(pdev);
    struct ls_pcie_mctp_client *client = kzalloc(sizeof(*client), GFP_KERNEL);
    unsigned long flags;
    if (!client) {
        dev_err(priv->dev, "Failed to allocate memory for client\n");
        return -ENOMEM;
    }
    client->priv = priv;
    INIT_LIST_HEAD(&client->head);
    INIT_LIST_HEAD(&client->rx_q);
    init_waitqueue_head(&client->wait_queue);

   	spin_lock_irqsave(&priv->clients_lock, flags);
    list_add_tail(&client->head, &priv->clients);
	spin_unlock_irqrestore(&priv->clients_lock, flags);

    file->private_data = client;
	return 0;
}

static int ls_pcie_mctp_release(struct inode *inode, struct file *file)
{
	struct ls_pcie_mctp_client *client = file->private_data;
    struct ls_pcie_mctp *priv = client->priv;
    unsigned long flags;
    spin_lock_irqsave(&priv->clients_lock, flags);
    list_del(&client->head);
    spin_unlock_irqrestore(&priv->clients_lock, flags);

	kfree(client);
	return 0;
}

static ssize_t ls_pcie_mctp_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct ls_pcie_mctp_client *client = file->private_data;
	struct ls_pcie_mctp *priv = client->priv;
    unsigned long flags;
	if (count < PCIE_MCTP_MIN_PACKET_SIZE)
		return -EINVAL;

	if (count > sizeof(u32)*(PCIE_VDM_HDR_SIZE_DW + PCIE_VDM_DATA_SIZE_DW))
		count = sizeof(u32)*(PCIE_VDM_HDR_SIZE_DW + PCIE_VDM_DATA_SIZE_DW);

    struct mctp_pcie_rx_packet *rx_pkt = list_first_entry(&client->rx_q, struct mctp_pcie_rx_packet, head);
    if(!rx_pkt)
    {
        return -EAGAIN;
    }
    if (copy_to_user(buf, rx_pkt->hdr, count)){
        dev_err(priv->dev, "Failed to copy data to user\n");
        return -EFAULT;
    }
    spin_lock_irqsave(&priv->clients_lock,flags);
    list_move_tail(&rx_pkt->head, &priv->rx_packet_free_list);
    spin_unlock_irqrestore(&priv->clients_lock, flags);
    return count;
}

static ssize_t ls_pcie_mctp_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{   
    struct ls_pcie_mctp_client *client = file->private_data;
    struct ls_pcie_mctp *priv = client->priv;
    struct ls_pcie *pcie = priv->pcie;
    struct device *pcie_dev = pcie->pci.dev;
	struct mctp_pcie_tx_packet *tx_packet;
	unsigned long flags;
    int ret;
	if (count < PCIE_MCTP_MIN_PACKET_SIZE)
		return -EINVAL;

	if (count > sizeof(*tx_packet->payload)+sizeof(tx_packet->hdr))
		return -ENOSPC;
    tx_packet = kzalloc(sizeof(*tx_packet), GFP_KERNEL);
    if (!tx_packet) {
        dev_err(priv->dev, "Failed to allocate memory for TX packet\n");
        return -ENOMEM;
    }
    tx_packet->payload = dma_alloc_coherent(pcie_dev, sizeof(*tx_packet->payload), &tx_packet->payload_dma_addr, GFP_KERNEL);
    if (!tx_packet->payload) {
        dev_err(priv->dev, "Failed to allocate DMA memory for TX packet payload\n");
        ret = -ENOMEM;
        goto free_tx_packet;
    }
    tx_packet->size = count;
    //TODO update packet header if needed

    if (copy_from_user(&tx_packet->hdr, buf, PCIE_VDM_HDR_SIZE)) {
        ret = -EFAULT;
        goto free_dma_buffer;
    }
    if (copy_from_user(tx_packet->payload->payload, buf + PCIE_VDM_HDR_SIZE, count - PCIE_VDM_HDR_SIZE)) {
        ret = -EFAULT;
        goto free_dma_buffer;
    }
	spin_lock_irqsave(&priv->clients_lock, flags);
	list_add_tail(&tx_packet->head, &priv->tx_q);
	spin_unlock_irqrestore(&priv->clients_lock, flags);
    wake_up_interruptible(&priv->tx_thread_wait_queue);
    return count;


free_dma_buffer:        
    dma_free_coherent(pcie_dev, sizeof(*tx_packet->payload), tx_packet->payload, tx_packet->payload_dma_addr);
free_tx_packet:
    kfree(tx_packet);
    return ret;
}

static int ls_mctp_filter_eid(struct ls_pcie_mctp *priv, void __user *userbuf)
{
    return -ENOTSUPP;
}

static u16 _get_bdf(struct ls_pcie_mctp *priv)
{
    struct ls_pcie *pcie = priv->pcie;
    u32 val[2];
    int err = regmap_read(pcie->pipesys_cfg,priv->bdf_reg_offset,&val[0]);
    BUG_ON(err);
    err = regmap_read(pcie->pipesys_cfg,priv->bdf_reg_offset + 8,&val[1]);
    BUG_ON(err);
    return ((val[0] & 0xff)<<8)|((val[1] & 0x1f)<<3);
}

static int ls_mctp_get_bdf(struct ls_pcie_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_get_bdf bdf = { _get_bdf(priv) };

	if (copy_to_user(userbuf, &bdf, sizeof(bdf))) {
		dev_err(priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int ls_mctp_get_medium_id(struct ls_pcie_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_get_medium_id id = { 0x09 }; /* PCIe revision 2.0 */

	if (copy_to_user(userbuf, &id, sizeof(id))) {
		dev_err(priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int ls_mctp_get_mtu(struct ls_pcie_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_get_mtu id = { LS_PCIE_MCTP_MTU };

	if (copy_to_user(userbuf, &id, sizeof(id))) {
		dev_err(priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int ls_mctp_register_default_handler(struct ls_pcie_mctp_client *client)
{
	struct ls_pcie_mctp *priv = client->priv;
	int ret = 0;
    unsigned long flags;
	spin_lock_irqsave(&priv->clients_lock, flags);

	if (!priv->default_client)
		priv->default_client = client;
	else if (priv->default_client != client)
		ret = -EBUSY;

	spin_unlock_irqrestore(&priv->clients_lock, flags);

	return ret;
}

static int ls_mctp_register_type_handler(struct ls_pcie_mctp_client *client,
				  void __user *userbuf)
{
	struct ls_pcie_mctp *priv = client->priv;
	struct aspeed_mctp_type_handler_ioctl handler;

	if (copy_from_user(&handler, userbuf, sizeof(handler))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

    if(client->type_valid)
    {
        return -EBUSY;
    }

    client->type = handler;
    client->type_valid = true;
	return 0;
}

static int ls_mctp_unregister_type_handler(struct ls_pcie_mctp_client *client,
				    void __user *userbuf)
{
	struct ls_pcie_mctp *priv = client->priv;
	struct aspeed_mctp_type_handler_ioctl handler;

	if (copy_from_user(&handler, userbuf, sizeof(handler))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}
    if(!client->type_valid)
    {
        return -EINVAL;
    }
    if (client->type.mctp_type != handler.mctp_type ||
        client->type.pci_vendor_id != handler.pci_vendor_id ||
        client->type.vendor_type != handler.vendor_type ||
        client->type.vendor_type_mask != handler.vendor_type_mask) {
        return -EINVAL;
    }
    client->type_valid = false;
	return 0;
}

static int ls_mctp_get_eid_info(struct ls_pcie_mctp *priv, void __user *userbuf,
			 enum mctp_address_type addr_format)
{
    return -ENOTSUPP;
}

static int ls_mctp_set_eid_info(struct ls_pcie_mctp *priv, void __user *userbuf,
			 enum mctp_address_type addr_format)
{
    return -ENOTSUPP;
}

static int ls_mctp_set_own_eid(struct ls_pcie_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_set_own_eid data;

	if (copy_from_user(&data, userbuf, sizeof(data))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}
	priv->own_eid = data.eid;
	return 0;
}

static long ls_pcie_mctp_ioctl(struct file *file,unsigned int cmd,unsigned long arg)
{
    struct ls_pcie_mctp_client *client = file->private_data;
    struct ls_pcie_mctp *priv = client->priv;
    void __user *userbuf = (void __user *)arg;
    int ret = 0;
	switch (cmd) {
	case ASPEED_MCTP_IOCTL_FILTER_EID:
		ret = ls_mctp_filter_eid(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_BDF:
		ret = ls_mctp_get_bdf(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_MEDIUM_ID:
		ret = ls_mctp_get_medium_id(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_MTU:
		ret = ls_mctp_get_mtu(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_REGISTER_DEFAULT_HANDLER:
		ret = ls_mctp_register_default_handler(client);
	break;

	case ASPEED_MCTP_IOCTL_REGISTER_TYPE_HANDLER:
		ret = ls_mctp_register_type_handler(client, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_UNREGISTER_TYPE_HANDLER:
		ret = ls_mctp_unregister_type_handler(client, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_EID_INFO:
		ret = ls_mctp_get_eid_info(priv, userbuf, MCTP_GENERIC_ADDR_FORMAT);
	break;

	case ASPEED_MCTP_IOCTL_GET_EID_EXT_INFO:
		ret = ls_mctp_get_eid_info(priv, userbuf, MCTP_EXTENDED_ADDR_FORMAT);
	break;

	case ASPEED_MCTP_IOCTL_SET_EID_INFO:
		ret = ls_mctp_set_eid_info(priv, userbuf, MCTP_GENERIC_ADDR_FORMAT);
	break;

	case ASPEED_MCTP_IOCTL_SET_EID_EXT_INFO:
		ret = ls_mctp_set_eid_info(priv, userbuf, MCTP_EXTENDED_ADDR_FORMAT);
	break;

	case ASPEED_MCTP_IOCTL_SET_OWN_EID:
		ret = ls_mctp_set_own_eid(priv, userbuf);
	break;

	default:
		dev_err(priv->dev, "Command not found\n");
		ret = -ENOTTY;
	}
    return ret;
}

static __poll_t ls_pcie_mctp_poll(struct file *file,struct poll_table_struct *pt)
{
    struct ls_pcie_mctp_client *client = file->private_data;
    __poll_t ret = 0;
    poll_wait(file,&client->wait_queue,pt);
    if(!list_empty(&client->rx_q))
    {
        ret |= POLLIN;
    }
    return ret | POLLOUT;
}

static void ls_pcie_mctp_remove(struct platform_device *pdev)
{
    struct ls_pcie_mctp *priv = platform_get_drvdata(pdev);
    struct ls_pcie *pcie = priv->pcie;
    regmap_write(pcie->pipesys_cfg, priv->rx_reg_offset + MCTP_RX_FIFO_INT_OFFSET, 0);
    kthread_stop(priv->tx_thread);
    misc_deregister(&priv->mctp_miscdev);
    ls_pcie_dma_release_chan(priv->dma_write_chan);
    ls_pcie_clear_outbound_atu(priv->pcie, msgd_tx_addr);
    ls_pcie_clear_msgd_inbound_atu(priv->pcie);
    dma_unmap_single(pcie->pci.dev, priv->rx_dma_handle, priv->rx_buffer_size, DMA_FROM_DEVICE);
}

static const struct file_operations ls_pcie_mctp_fops = {
	.owner = THIS_MODULE,
	.open = ls_pcie_mctp_open,
	.release = ls_pcie_mctp_release,
	.read = ls_pcie_mctp_read,
	.write = ls_pcie_mctp_write,
	.unlocked_ioctl = ls_pcie_mctp_ioctl,
	.poll = ls_pcie_mctp_poll,
};

static struct ls_pcie_mctp_client *find_rx_client(struct ls_pcie_mctp *priv, struct mctp_pcie_rx_packet *rx_pkt)
{
    struct ls_pcie_mctp_client *client;
    list_for_each_entry(client, &priv->clients, head) {
        if(client->type_valid)
        {

        }
    }
    return priv->default_client;
}

static void print_rx_pkt(struct device *dev, struct mctp_pcie_rx_packet *rx_pkt)
{
    uint8_t *hdr = (uint8_t *)rx_pkt->hdr;
    uint8_t *payload = (uint8_t *)rx_pkt->payload;

    printk(KERN_INFO "mctp rx"
        "  %2.2x %2.2x %2.2x %2.2x"
        "  %2.2x %2.2x %2.2x %2.2x"
        "  %2.2x %2.2x %2.2x %2.2x"
        "  %2.2x %2.2x %2.2x %2.2x",
        hdr[0], hdr[1], hdr[2], hdr[3],
        hdr[4], hdr[5], hdr[6], hdr[7],
        hdr[8], hdr[9], hdr[10], hdr[11],
        hdr[12], hdr[13], hdr[14], hdr[15]);
    for (int i = 0; i < MCTP_PAYLOAD_SIZE(hdr); i++) {
        printk(KERN_CONT " %2.2x", payload[i]);
    }
    printk(KERN_CONT "\n");
}

static void print_tx_pkt(struct device *dev, struct mctp_pcie_tx_packet *tx_pkt)
{
    uint8_t *hdr = (uint8_t *)tx_pkt->hdr;
    uint8_t *payload = (uint8_t *)tx_pkt->payload;

    printk(KERN_INFO "mctp tx"
        "  %2.2x %2.2x %2.2x %2.2x"
        "  %2.2x %2.2x %2.2x %2.2x"
        "  %2.2x %2.2x %2.2x %2.2x"
        "  %2.2x %2.2x %2.2x %2.2x",
        hdr[0], hdr[1], hdr[2], hdr[3],
        hdr[4], hdr[5], hdr[6], hdr[7],
        hdr[8], hdr[9], hdr[10], hdr[11],
        hdr[12], hdr[13], hdr[14], hdr[15]);
    for (int i = 0; i < tx_pkt->size; i++) {
        printk(KERN_CONT " %2.2x", payload[i]);
    }
    printk(KERN_CONT "\n");
}

#if defined(DEBUG_MCTP_DISCOVERY)
static int ls_mctp_queue_ctrl_response(struct ls_pcie_mctp *priv,
                                       struct mctp_pcie_rx_packet *rx_pkt,
                                       u16 rx_data_len)
{
    struct ls_pcie *pcie = priv->pcie;
    struct device *pcie_dev = pcie->pci.dev;
    struct mctp_pcie_tx_packet *tx_pkt;
    unsigned long flags;
    u8 *req_hdr;
    u8 *req_data;
    u8 *resp_hdr;
    u8 *resp_data;
    u8 cmd;
    u32 resp_data_len;

    req_hdr = (u8 *)rx_pkt->hdr;
    req_data = (u8 *)rx_pkt->payload;
    if ((req_data[0] & MCTP_MSG_TYPE_MASK) != MCTP_MSG_TYPE_CONTROL) {
        dev_err(priv->dev, "Invalid message type\n");
        return 0;
    }

    if (!(req_data[1] & MCTP_CTRL_HDR_RQ)) {
        dev_err(priv->dev, "Invalid control header request: 0x%2.2x\n", req_data[1]);
        return 0;
    }

    cmd = req_data[2];
    if (cmd != MCTP_CTRL_CMD_ENDPOINT_DISCOVERY
        && cmd != MCTP_CTRL_CMD_SET_ENDPOINT_ID
        && cmd != MCTP_CTRL_CMD_GET_ENDPOINT_ID) {
        dev_err(priv->dev, "Invalid command\n");
        return 0;
    }

    tx_pkt = kzalloc(sizeof(*tx_pkt), GFP_ATOMIC);
    if (!tx_pkt) {
        dev_err(priv->dev, "Failed to allocate memory for tx packet\n");
        return -ENOMEM;
    }

    tx_pkt->payload = dma_alloc_coherent(pcie_dev, rx_data_len,
                                         &tx_pkt->payload_dma_addr,
                                         GFP_ATOMIC);
    if (!tx_pkt->payload) {
        dev_err(priv->dev, "Failed to allocate memory for tx payload\n");
        kfree(tx_pkt);
        return -ENOMEM;
    }

    memcpy(tx_pkt->hdr, rx_pkt->hdr, sizeof(tx_pkt->hdr));
    resp_hdr = (u8 *)tx_pkt->hdr;
    resp_data = (u8 *)tx_pkt->payload->payload;
    memset(resp_data, 0, sizeof(*tx_pkt->payload));

    /* target bdf */
    resp_hdr[4] = req_hdr[8];
    resp_hdr[5] = req_hdr[9];
    resp_hdr[8] = req_hdr[4];
    resp_hdr[9] = req_hdr[5];

    /* MCTP transport header */
    resp_hdr[13] = req_hdr[14]; /* dst eid */
    resp_hdr[14] = 0;
    resp_hdr[15] &= ~MCTP_TAG_OWNER;

    /* MCTP control header */
    resp_data[0] = req_data[0];
    resp_data[1] = req_data[1] & ~MCTP_CTRL_HDR_RQ;
    resp_data[2] = req_data[2];
    resp_data[3] = MCTP_CTRL_CC_SUCCESS;

    resp_hdr[0] &= ~MCTP_RX_DATA0_ROUTE_TYPE_MASK;
    if (cmd == MCTP_CTRL_CMD_SET_ENDPOINT_ID) {
        if (rx_data_len >= MCTP_TRANSPORT_HDR_SIZE + 4 && req_data[4] != 0) {
            priv->own_eid = req_data[4];
        }
        resp_hdr[0] |= MCTP_PCIE_VDM_ROUTE_BY_ID;
        resp_hdr[6] = 0x10;
        resp_data[4] = 0x00;
        resp_data[5] = priv->own_eid;
        resp_data[6] = 0x00;
        resp_data[7] = 0x00;
        resp_data_len = 8;
        dev_info(priv->dev, "MCTP_CTRL_CMD_SET_ENDPOINT_ID: own_eid=%d\n", priv->own_eid);
    } else if (cmd == MCTP_CTRL_CMD_GET_ENDPOINT_ID) {
        resp_hdr[0] |= MCTP_PCIE_VDM_ROUTE_BY_ID;
        resp_hdr[6] = 0x10;
        resp_data[4] = 0x00;
        resp_data[5] = 0x00;
        resp_data[6] = 0x00;
        resp_data[7] = 0x00;
        resp_data_len = 8;
    } else if (cmd == MCTP_CTRL_CMD_ENDPOINT_DISCOVERY) {
        /* Route discovery response to Root Complex (R_RC = 0). */
        resp_hdr[0] |= MCTP_PCIE_VDM_ROUTE_TO_RC;
        resp_data_len = 4;
    }

    tx_pkt->size = resp_data_len;
    resp_hdr[3] = resp_data_len / 4;

    print_tx_pkt(priv->dev, tx_pkt);
    list_add_tail(&tx_pkt->head, &priv->tx_q);
    wake_up_interruptible(&priv->tx_thread_wait_queue);

    return 0;
}
#endif /* DEBUG_MCTP_DISCOVERY */

static irqreturn_t ls_pcie_mctp_irq(int irq, void *arg)
{
    struct ls_pcie_mctp *priv = arg;
    struct ls_pcie *pcie = priv->pcie;
    u32 int_reg;
    unsigned long flags;
    irqreturn_t ret = IRQ_NONE;
    while(1)
    {
        int err = regmap_read(pcie->pipesys_cfg,priv->rx_reg_offset+MCTP_RX_FIFO_INT_OFFSET,&int_reg);
        BUG_ON(err);
        if((int_reg & MCTP_RX_FIFO_INT_STT_MASK) == 0)
        {
            break;
        }
        dev_info(priv->dev, "MCTP IRQ: int_reg=0x%x\n", int_reg);
        ret = IRQ_HANDLED;
        if(int_reg & MCTP_RX_FIFO_WARN_LEVEL_INT_STT_BIT){
            u8 fifo_lvl = int_reg & MCTP_RX_FIFO_LEVEL_MASK;
            while(fifo_lvl--)
            {
                u16 length;
                struct mctp_pcie_rx_packet *rx_pkt = list_first_entry(&priv->rx_packet_free_list, struct mctp_pcie_rx_packet, head);
                BUG_ON(!rx_pkt);
                err = regmap_read(pcie->pipesys_cfg, priv->rx_reg_offset + MCTP_RX_FIFO_DATA0_OFFSET, &rx_pkt->hdr[0]);
                BUG_ON(err);
                rx_pkt->hdr[0] = be32_to_cpu(rx_pkt->hdr[0]);
                uint8_t *hdr = (uint8_t *)&rx_pkt->hdr[0];
                length = MCTP_PAYLOAD_SIZE(hdr) + 8;
                err = regmap_read(pcie->pipesys_cfg, priv->rx_reg_offset + MCTP_RX_FIFO_DATA1_OFFSET, &rx_pkt->hdr[1]);
                BUG_ON(err);
                rx_pkt->hdr[1] = be32_to_cpu(rx_pkt->hdr[1]);
                err = regmap_write(pcie->pipesys_cfg, priv->rx_reg_offset + MCTP_RX_FIFO_CTRL_OFFSET, MCTP_RX_FIFO_POP_BIT);
                BUG_ON(err);
                dma_sync_single_for_cpu(pcie->pci.dev, priv->rx_dma_handle + priv->rx_idx*CBUF_INCR_SIZE(RX_CBUF_INCR),
                        CBUF_INCR_SIZE(RX_CBUF_INCR), DMA_FROM_DEVICE);
                // TODO: check mtu
                memcpy(&rx_pkt->hdr[2], priv->rx_circular_buffer + priv->rx_idx*CBUF_INCR_SIZE(RX_CBUF_INCR), length);
                dma_sync_single_for_device(pcie->pci.dev, priv->rx_dma_handle + priv->rx_idx*CBUF_INCR_SIZE(RX_CBUF_INCR),
                        CBUF_INCR_SIZE(RX_CBUF_INCR), DMA_FROM_DEVICE);
                priv->rx_idx = (priv->rx_idx + 1) % priv->rx_ring_count;
                struct ls_pcie_mctp_client *client;
                print_rx_pkt(priv->dev, rx_pkt);
#if defined(DEBUG_MCTP_DISCOVERY)
                if (length >= 8) {
                    ls_mctp_queue_ctrl_response(priv, rx_pkt, length - 8);
                }
#endif
                client = find_rx_client(priv, rx_pkt);
                if(client)
                {
                    spin_lock_irqsave(&priv->clients_lock, flags);
                    list_move_tail(&rx_pkt->head, &client->rx_q);
                    spin_unlock_irqrestore(&priv->clients_lock, flags);
                    wake_up_interruptible(&client->wait_queue);
                }
            }
        }
    }

    return ret;
}

void dw_pcie_outbound_atu_type_set(struct dw_pcie *pci, int index, int type);
void dw_pcie_outbound_atu_target_set(struct dw_pcie *pci, int index, u32 lower_target, u32 upper_target);
void dw_pcie_outbound_atu_tag_set(struct dw_pcie *pci, int index, u32 tag);
static void mctp_pkt_tx(struct ls_pcie_mctp *priv, struct mctp_pcie_tx_packet *tx_pkt)
{
#if 0
    tx_pkt->hdr[2] = 0x5aa50000;
    tx_pkt->hdr[3] = 0x00005aa5;
    tx_pkt->size = 256;
    for (int i = 0; i < tx_pkt->size / 4; i++) {
        tx_pkt->payload->payload[i] = 0x10000000 + i;
    }
#endif
    uint8_t type = PCIE_ATU_TYPE_MSG | (tx_pkt->hdr[0] & MCTP_RX_DATA0_ROUTE_TYPE_MASK);
    dw_pcie_outbound_atu_type_set(&priv->pcie->pci, priv->outbound_atu_idx, type); //set message routing subfield
    dw_pcie_outbound_atu_target_set(&priv->pcie->pci, priv->outbound_atu_idx, cpu_to_be32(tx_pkt->hdr[3]), cpu_to_be32(tx_pkt->hdr[2]));
    dw_pcie_outbound_atu_tag_set(&priv->pcie->pci, priv->outbound_atu_idx, ((uint8_t *)tx_pkt->hdr)[6]);
    // regmap_write(priv->pcie->pipesys_cfg, priv->tx_reg_offset + 4, tx_pkt->hdr[2]);
    // regmap_write(priv->pcie->pipesys_cfg, priv->tx_reg_offset, tx_pkt->hdr[3]);
    ls_pcie_dma_read_write(priv->dma_write_chan, msgd_tx_addr, tx_pkt->payload_dma_addr, 
        tx_pkt->size, NULL, NULL, &priv->tx_completion, false);
}

static inline bool tx_trigger_condition(struct ls_pcie_mctp *priv)
{
    return !list_empty(&priv->tx_q);
}

static int tx_thread_func(void *param)
{
    struct ls_pcie_mctp *priv = (struct ls_pcie_mctp *)param;
    struct ls_pcie *pcie = priv->pcie;
    struct device *pcie_dev = pcie->pci.dev;
    unsigned long flags;
    init_waitqueue_head(&priv->tx_thread_wait_queue);
    dev_info(priv->dev, "MCTP TX thread started\n");
    while(!kthread_should_stop()){
        wait_event_interruptible(priv->tx_thread_wait_queue, tx_trigger_condition(priv));
        dev_info(priv->dev, "MCTP TX thread woke up\n");
        if(tx_trigger_condition(priv)) {
            struct mctp_pcie_tx_packet *cur_tx = list_first_entry(&priv->tx_q, struct mctp_pcie_tx_packet, head);
        	spin_lock_irqsave(&priv->clients_lock, flags);
            list_del(&cur_tx->head);
            spin_unlock_irqrestore(&priv->clients_lock, flags);
            mctp_pkt_tx(priv, cur_tx);
            dma_free_coherent(pcie_dev, sizeof(*cur_tx->payload), cur_tx->payload, cur_tx->payload_dma_addr);
            kfree(cur_tx);
        }
    }
    return 0;
}

static int ls_pcie_mctp_probe(struct platform_device *pdev)
{
	struct ls_pcie_mctp *priv;
    struct resource *rx_res;
    struct resource *tx_res;
    struct resource *bdf_res;
	int ret,irq;
	struct device_node *parent = of_get_parent(pdev->dev.of_node);
    if (!parent) {
        dev_err(&pdev->dev, "Failed to get parent device node\n");
        return -ENODEV;
    }
    struct platform_device *parent_pdev = of_find_device_by_node(parent);
    if (!parent_pdev) {
        dev_err(&pdev->dev, "Failed to get parent platform device\n");
        return -ENODEV;
    }
    struct ls_pcie *pcie = platform_get_drvdata(parent_pdev);
    if (!pcie) {
        dev_err(&pdev->dev, "ls pcie not inited\n");
        return -EPROBE_DEFER;
    }    
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
        dev_err(&pdev->dev, "Failed to allocate memory for ls_pcie_mctp\n");
		return -ENOMEM;
	}
	
	rx_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rx");
    if (!rx_res) {
        dev_err(&pdev->dev, "Failed to get rx memory resource\n");
        return -ENODEV;
    }

    tx_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tx");
    if (!tx_res) {
        dev_err(&pdev->dev, "Failed to get tx memory resource\n");
        return -ENODEV;
    }

    bdf_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bdf");
    if (!bdf_res) {
        dev_err(&pdev->dev, "Failed to get BDF resource\n");
        return -ENODEV;
    }
    priv->bdf_reg_offset = bdf_res->start;
	irq = platform_get_irq(pdev, 0);
    if(irq<0) {
		dev_err(&pdev->dev, "cannot get irq\n");
        return irq;
    }
    ret = devm_request_irq(&pdev->dev, irq, ls_pcie_mctp_irq, IRQF_SHARED,
				dev_name(&pdev->dev), priv);
    if(ret<0) {
		dev_err(&pdev->dev, "cannot request irq\n");
        return ret;
    }

    priv->pcie = pcie;
    priv->dev = &pdev->dev;
    priv->rx_reg_offset = rx_res->start;
    priv->tx_reg_offset = tx_res->start;
    platform_set_drvdata(pdev, priv);

    priv->rx_buffer_size = pcie->pci.region_align;
    int order = get_order(priv->rx_buffer_size);
    priv->rx_circular_buffer = (void *)devm_get_free_pages(&pdev->dev, GFP_KERNEL,order);
    if (!priv->rx_circular_buffer) {
        dev_err(&pdev->dev, "Failed to remap MCTP RX buffer\n");
        return -ENOMEM;
    }

    priv->rx_idx = 0; 
    priv->rx_ring_count = priv->rx_buffer_size / CBUF_INCR_SIZE(RX_CBUF_INCR);

    struct mctp_pcie_rx_packet *rx_pkt = devm_kzalloc(&pdev->dev, priv->rx_ring_count*sizeof(*rx_pkt), GFP_KERNEL);
    if (!rx_pkt) {
        dev_err(&pdev->dev, "Failed to allocate rx packet buffer\n");
        return -ENOMEM;
    }
    INIT_LIST_HEAD(&priv->rx_packet_free_list);
    for (int i = 0; i < priv->rx_ring_count; i++) {
        list_add_tail(&rx_pkt[i].head, &priv->rx_packet_free_list);
    }

    priv->rx_dma_handle = dma_map_single(pcie->pci.dev,priv->rx_circular_buffer,priv->rx_buffer_size,DMA_FROM_DEVICE);
    if(dma_mapping_error(pcie->pci.dev, priv->rx_dma_handle)) {
        dev_err(&pdev->dev, "Failed to map MCTP RX buffer DMA\n");
        return -ENOMEM;
    }

    ret = ls_pcie_set_msgd_inbound_atu(pcie,RX_CBUF_INCR,priv->rx_buffer_size,priv->rx_dma_handle);
    if (ret) {
        dev_err(&pdev->dev, "Failed to set MSGD inbound ATU\n");
        goto dma_ummap;
    }

    struct dw_pcie_ob_atu_cfg atu = {
        .type = PCIE_ATU_TYPE_MSG,
        .code = 0x7f, // Use 0x7f as the message code for MSGD
        .parent_bus_addr = msgd_tx_addr,
        .size = pcie->pci.region_align, // Use region_align as the size for the MSGD ATU
    };

    ret = ls_pcie_set_outbound_atu(pcie, &atu);
    if (ret<0) {
        dev_err(&pdev->dev, "Failed to set MSGD outbound ATU\n");
        goto clear_ib_atu;
    }
    priv->outbound_atu_idx = ret;
    priv->dma_write_chan = ls_pcie_request_dma_chan(pcie, true);
    if (!priv->dma_write_chan) {
        dev_err(&pdev->dev, "Failed to request DMA write channel\n");
        ret = -ENODEV;
        goto clear_ob_atu;
    }

	priv->mctp_miscdev.parent = priv->dev;
	priv->mctp_miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->mctp_miscdev.name = kasprintf(GFP_KERNEL, "linkedsemi-mctp@%llx", rx_res->start);
	priv->mctp_miscdev.fops = &ls_pcie_mctp_fops;
	ret = misc_register(&priv->mctp_miscdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register miscdev\n");
		goto release_dma_chan;
	}

	INIT_LIST_HEAD(&priv->clients);
    INIT_LIST_HEAD(&priv->tx_q);
    spin_lock_init(&priv->clients_lock);

    regmap_write(pcie->pipesys_cfg, priv->rx_reg_offset + MCTP_RX_ATU_MATCH_MASK_OFFSET, 0xffffffff);
    regmap_write(pcie->pipesys_cfg, priv->rx_reg_offset + MCTP_RX_FIFO_CTRL_OFFSET, MCTP_RX_FIFO_WCLR_BIT | MCTP_RX_FIFO_RCLR_BIT);
    regmap_write(pcie->pipesys_cfg, priv->rx_reg_offset + MCTP_RX_FIFO_WARN_LEVEL_OFFSET, 0x1);
    regmap_write(pcie->pipesys_cfg, priv->rx_reg_offset + MCTP_RX_FIFO_INT_OFFSET, MCTP_RX_FIFO_WARN_LEVEL_INT_MASK);
    priv->tx_thread = kthread_run(tx_thread_func, priv, "ls_pcie_mctp_tx_thread@%llx", rx_res->start);
    if (IS_ERR(priv->tx_thread)) {
        dev_err(&pdev->dev, "Failed to create tx thread\n");
        ret = PTR_ERR(priv->tx_thread);
        goto unregister_miscdev;
    }
    return 0;

unregister_miscdev:
    misc_deregister(&priv->mctp_miscdev);
release_dma_chan:
    ls_pcie_dma_release_chan(priv->dma_write_chan);
clear_ob_atu:
    ls_pcie_clear_outbound_atu(pcie, msgd_tx_addr);
clear_ib_atu:
    ls_pcie_clear_msgd_inbound_atu(pcie);
dma_ummap:
    dma_unmap_single(pcie->pci.dev, priv->rx_dma_handle, priv->rx_buffer_size, DMA_FROM_DEVICE);
    return ret;
}

static const struct of_device_id ls_pcie_mctp_match_table[] = {
	{ .compatible = "linkedsemi,ls-pcie-mctp", .data = NULL},
	{ }
};

static struct platform_driver ls_pcie_mctp_driver = {
	.driver	= {
		.name		= "ls-pcie-mctp",
		.of_match_table	= of_match_ptr(ls_pcie_mctp_match_table),
	},
	.probe	= ls_pcie_mctp_probe,
	.remove	= ls_pcie_mctp_remove,
};

static int __init ls_pcie_mctp_init(void)
{
	return platform_driver_register(&ls_pcie_mctp_driver);
}

static void __exit ls_pcie_mctp_exit(void)
{
	platform_driver_unregister(&ls_pcie_mctp_driver);
}

module_init(ls_pcie_mctp_init)
module_exit(ls_pcie_mctp_exit)

MODULE_DEVICE_TABLE(of, ls_pcie_mctp_match_table);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Linkedsemi PCIE MCTP driver");
