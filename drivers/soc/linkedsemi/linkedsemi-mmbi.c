// SPDX-License-Identifier: GPL-2.0
/*
 * Linkedsemi LS2010 MMBI (Memory Mapped Bus Interface) common driver
 *
 * Implements the BMC-side MMBI protocol shared with the LS2010 host:
 * descriptor / host_ros / host_rws initialization and B2H/H2B buffer
 * space allocation, the channel state machine and the user space
 * interface.  The per-channel user space interface (miscdev
 * /dev/mmbi%d-ch%d with read/write/poll) keeps the same semantics as
 * the Aspeed MMBI driver (drivers/soc/aspeed/mmbi/aspeed-mmbi.c).
 *
 * Transport specific parts (mailbox exchange with the E906 firmware for
 * eSPI, BAR mapping and doorbell interrupt for PCIe) live in
 * linkedsemi-espi-mmbi.c and linkedsemi-pcie-mmbi.c.  Those drivers
 * allocate the shared region (dma_alloc_coherent) and hand it to
 * ls_mmbi_init() through struct ls_mmbi_ins.
 *
 * Layout of the shared region (offsets are bytes, descriptor fields
 * marked /8 are stored in units of 8 bytes):
 *
 *   +---------------------------------------+ 0x00
 *   | struct ls_mmbi_desc (64B)             | descriptor
 *   +---------------------------------------+ 0x40
 *   | struct ls_mmbi_host_ros (8B)          | b2h_wp, h2b_rp (BMC owned)
 *   +---------------------------------------+ 0x48
 *   | struct ls_mmbi_host_rws (8B)          | h2b_wp, b2h_rp (host owned)
 *   +---------------------------------------+ 0x50
 *   | B2H buffer (BMC -> host)              | b2h_l bytes
 *   +---------------------------------------+
 *   | H2B buffer (host -> BMC)              | h2b_l bytes
 *   +---------------------------------------+
 */

#include <asm/barrier.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "linkedsemi-mmbi.h"

static DEFINE_IDA(ls_mmbi_ida);

struct ls_mmbi_chan_priv {
	struct miscdevice miscdev;
	wait_queue_head_t rx_wait;	/* host -> BMC data available */
	wait_queue_head_t tx_wait;	/* BMC -> host buffer space available */
	bool rx_ready;
	bool tx_ready;
	spinlock_t rx_lock;
	spinlock_t tx_lock;
	struct ls_mmbi_chan_desc *chan;
};

static inline struct ls_mmbi_desc *ls_mmbi_desc(struct ls_mmbi_ins *mmbi)
{
	return mmbi->virt + LS_MMBI_DESC_OFF;
}

static inline struct ls_mmbi_host_ros *ls_mmbi_ros(struct ls_mmbi_ins *mmbi)
{
	return mmbi->virt + LS_MMBI_ROS_OFF;
}

static inline struct ls_mmbi_host_rws *ls_mmbi_rws(struct ls_mmbi_ins *mmbi)
{
	return mmbi->virt + LS_MMBI_RWS_OFF;
}

/* Free space in a ring, wp/rp are byte offsets within [0, size) */
static u32 ls_mmbi_avail(u32 wp, u32 rp, u32 size)
{
	return (wp >= rp) ? (size - wp + rp) : (rp - wp);
}

/* Unread bytes in a ring */
static u32 ls_mmbi_unread(u32 wp, u32 rp, u32 size)
{
	return (wp >= rp) ? (wp - rp) : (size - rp + wp);
}

/* Pointer register value without the state bits */
static u32 ls_mmbi_ptr(u32 reg)
{
	return reg & LS_MMBI_PTR_MASK;
}

static void ls_mmbi_chan_refresh(struct ls_mmbi_chan_desc *chan)
{
	struct ls_mmbi_ins *mmbi = chan->mmbi;
	struct ls_mmbi_host_ros *ros = ls_mmbi_ros(mmbi);
	struct ls_mmbi_host_rws *rws = ls_mmbi_rws(mmbi);
	struct ls_mmbi_chan_priv *priv = chan->priv;
	u32 wp, rp;

	/* Host -> BMC */
	dma_rmb();
	wp = ls_mmbi_ptr(READ_ONCE(rws->h2b_wp));
	rp = ls_mmbi_ptr(READ_ONCE(ros->h2b_rp));
	priv->rx_ready = ls_mmbi_unread(wp, rp, chan->h2b_l) > 0;

	/* BMC -> host */
	wp = ls_mmbi_ptr(READ_ONCE(ros->b2h_wp));
	rp = ls_mmbi_ptr(READ_ONCE(rws->b2h_rp));
	priv->tx_ready = ls_mmbi_avail(wp, rp, chan->b2h_l) > 0;
}

void ls_mmbi_host_write_notify(struct ls_mmbi_ins *mmbi)
{
	int i;

	for (i = 0; i < mmbi->num_of_channels; i++) {
		struct ls_mmbi_chan_desc *chan = &mmbi->chan_desc[i];
		struct ls_mmbi_chan_priv *priv = chan->priv;

		ls_mmbi_chan_refresh(chan);
		if (priv->rx_ready)
			wake_up_interruptible(&priv->rx_wait);
		if (priv->tx_ready)
			wake_up_interruptible(&priv->tx_wait);
	}
}
EXPORT_SYMBOL_GPL(ls_mmbi_host_write_notify);

/*
 * BMC -> host: write payload into the B2H ring and advance the B2H
 * write pointer (keeping the BMC state bits).  All-or-nothing like the
 * Aspeed driver: -EAGAIN when the data does not fit.  The transport
 * hook that reports an interrupt to the host is invoked by the caller
 * after releasing the channel lock, because it may sleep (mbox_send,
 * pci_epc_raise_irq).
 */
static ssize_t ls_mmbi_chan_tx(struct ls_mmbi_chan_desc *chan,
			       const u8 *data, size_t len)
{
	struct ls_mmbi_ins *mmbi = chan->mmbi;
	struct ls_mmbi_host_ros *ros = ls_mmbi_ros(mmbi);
	struct ls_mmbi_host_rws *rws = ls_mmbi_rws(mmbi);
	u8 *buf = mmbi->virt + chan->b2h_ba;
	u32 wp, rp, first;
	size_t avail;

	if (len == 0)
		return -EINVAL;

	wp = ls_mmbi_ptr(READ_ONCE(ros->b2h_wp));
	rp = ls_mmbi_ptr(READ_ONCE(rws->b2h_rp));
	avail = ls_mmbi_avail(wp, rp, chan->b2h_l);
	if (len > avail)
		return -EAGAIN;

	first = min_t(u32, len, chan->b2h_l - wp);
	memcpy(buf + wp, data, first);
	memcpy(buf, data + first, len - first);

	wp += len;
	if (wp >= chan->b2h_l)
		wp -= chan->b2h_l;

	/* Publish the payload before advancing the pointer */
	dma_wmb();
	WRITE_ONCE(ros->b2h_wp, wp | (READ_ONCE(ros->b2h_wp) & ~LS_MMBI_PTR_MASK));

	return len;
}

/*
 * host -> BMC: read payload from the H2B ring and advance the H2B read
 * pointer.  Returns 0 when no data is pending (non-blocking, like the
 * Aspeed driver; user space waits with poll()).
 */
static ssize_t ls_mmbi_chan_rx(struct ls_mmbi_chan_desc *chan,
			       u8 *data, size_t max_len)
{
	struct ls_mmbi_ins *mmbi = chan->mmbi;
	struct ls_mmbi_host_ros *ros = ls_mmbi_ros(mmbi);
	struct ls_mmbi_host_rws *rws = ls_mmbi_rws(mmbi);
	u8 *buf = mmbi->virt + chan->h2b_ba;
	u32 wp, rp, first, len;

	if (max_len == 0)
		return 0;

	dma_rmb();
	wp = ls_mmbi_ptr(READ_ONCE(rws->h2b_wp));
	rp = ls_mmbi_ptr(READ_ONCE(ros->h2b_rp));
	len = ls_mmbi_unread(wp, rp, chan->h2b_l);
	if (len == 0)
		return 0;

	len = min(len, (u32)max_len);
	first = min(len, chan->h2b_l - rp);
	memcpy(data, buf + rp, first);
	memcpy(data + first, buf, len - first);

	rp += len;
	if (rp >= chan->h2b_l)
		rp -= chan->h2b_l;
	WRITE_ONCE(ros->h2b_rp, rp);

	return len;
}

static struct ls_mmbi_chan_desc *ls_mmbi_file_chan(struct file *file)
{
	return file->private_data;
}

static ssize_t ls_mmbi_misc_read(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct ls_mmbi_chan_desc *chan = ls_mmbi_file_chan(file);
	struct ls_mmbi_chan_priv *priv = chan->priv;
	u8 *kbuf;
	ssize_t ret;

	if (count == 0)
		return 0;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	spin_lock(&priv->rx_lock);
	ret = ls_mmbi_chan_rx(chan, kbuf, count);
	spin_unlock(&priv->rx_lock);

	if (ret > 0 && copy_to_user(buf, kbuf, ret))
		ret = -EFAULT;

	kfree(kbuf);
	return ret;
}

static ssize_t ls_mmbi_misc_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct ls_mmbi_chan_desc *chan = ls_mmbi_file_chan(file);
	struct ls_mmbi_chan_priv *priv = chan->priv;
	u8 *kbuf;
	ssize_t ret;

	if (count == 0)
		return 0;

	kbuf = memdup_user(buf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	spin_lock(&priv->tx_lock);
	ret = ls_mmbi_chan_tx(chan, kbuf, count);
	spin_unlock(&priv->tx_lock);

	if (ret > 0) {
		/* The write may have consumed the last free space; refresh
		 * and let blocked writers know if more is available.
		 */
		ls_mmbi_chan_refresh(chan);
		if (priv->tx_ready)
			wake_up_interruptible(&priv->tx_wait);

		/* Report the new data to the host (may sleep) */
		if (chan->mmbi->host_int_raise)
			chan->mmbi->host_int_raise(chan->mmbi);
	}

	kfree(kbuf);
	return ret;
}

static __poll_t ls_mmbi_misc_poll(struct file *file,
				  struct poll_table_struct *pt)
{
	struct ls_mmbi_chan_desc *chan = ls_mmbi_file_chan(file);
	struct ls_mmbi_chan_priv *priv = chan->priv;
	__poll_t ret = 0, requested;

	requested = poll_requested_events(pt);

	if (requested & POLL_IN)
		poll_wait(file, &priv->rx_wait, pt);
	if (requested & POLL_OUT)
		poll_wait(file, &priv->tx_wait, pt);

	ls_mmbi_chan_refresh(chan);

	if (priv->rx_ready)
		ret |= POLL_IN | POLLRDNORM;
	if (priv->tx_ready)
		ret |= POLL_OUT | POLLWRNORM;

	return ret;
}

static int ls_mmbi_misc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct ls_mmbi_chan_priv *priv =
		container_of(miscdev, struct ls_mmbi_chan_priv, miscdev);

	file->private_data = priv->chan;
	return 0;
}

static const struct file_operations ls_mmbi_fops = {
	.owner = THIS_MODULE,
	.open = ls_mmbi_misc_open,
	.read = ls_mmbi_misc_read,
	.write = ls_mmbi_misc_write,
	.poll = ls_mmbi_misc_poll,
};

static int ls_mmbi_chan_init_miscdev(struct ls_mmbi_chan_desc *chan)
{
	struct ls_mmbi_ins *mmbi = chan->mmbi;
	struct ls_mmbi_chan_priv *priv = chan->priv;
	int ret;

	priv->miscdev.name = devm_kasprintf(mmbi->dev, GFP_KERNEL,
					    "mmbi%d-ch%d",
					    mmbi->ins_id, chan->index);
	if (!priv->miscdev.name)
		return -ENOMEM;

	priv->miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->miscdev.fops = &ls_mmbi_fops;
	priv->miscdev.parent = mmbi->dev;

	ret = misc_register(&priv->miscdev);
	if (ret) {
		dev_err(mmbi->dev, "failed to register %s: %d\n",
			priv->miscdev.name, ret);
		return ret;
	}

	dev_info(mmbi->dev, "registered %s\n", priv->miscdev.name);
	return 0;
}

/*
 * Initialize the shared region: write the descriptor, clear host_ros /
 * host_rws and set the BMC state bits.  The B2H and H2B buffers split
 * the space left after the descriptor and pointer regions in half.
 */
static int ls_mmbi_init_layout(struct ls_mmbi_ins *mmbi)
{
	struct ls_mmbi_desc *desc;
	struct ls_mmbi_host_ros *ros;
	struct ls_mmbi_host_rws *rws;
	u32 buf_total, buf_len;
	u8 i;

	if (mmbi->size < LS_MMBI_MIN_SIZE ||
	    mmbi->size > U32_MAX || (mmbi->size & 7)) {
		dev_err(mmbi->dev, "invalid MMBI region size %zu\n",
			mmbi->size);
		return -EINVAL;
	}

	buf_total = mmbi->size - LS_MMBI_BUF_OFF;
	buf_len = (buf_total / 2 / mmbi->num_of_channels) & ~7U;
	if (buf_len == 0) {
		dev_err(mmbi->dev, "MMBI region too small for %u channels\n",
			mmbi->num_of_channels);
		return -EINVAL;
	}

	memset(mmbi->virt, 0, mmbi->size);

	desc = ls_mmbi_desc(mmbi);
	ros = ls_mmbi_ros(mmbi);
	rws = ls_mmbi_rws(mmbi);

	memcpy(desc->signature, LS_MMBI_SIGNATURE, sizeof(LS_MMBI_SIGNATURE) - 1);
	desc->version = LS_MMBI_VERSION_1_0;
	desc->os_use = 0;
	desc->buft = LS_MMBI_BUF_TYPE_VPSCB;
	desc->h_ros_p = LS_MMBI_UNITS(LS_MMBI_ROS_OFF);
	desc->h_rws_p = LS_MMBI_UNITS(LS_MMBI_RWS_OFF);
	desc->h_int_t = mmbi->host_int_type;
	desc->h_int_l = mmbi->host_int_location;
	desc->h_int_v = mmbi->host_int_value;
	desc->bmc_int_t = mmbi->bmc_int_type;
	desc->bmc_int_l = mmbi->bmc_int_location;
	desc->bmc_int_v = mmbi->bmc_int_value;

	/* BMC is up, not in reset */
	ros->b2h_wp = LS_MMBI_STATE_B_U;
	ros->h2b_rp = 0;
	rws->h2b_wp = 0;
	rws->b2h_rp = 0;

	for (i = 0; i < mmbi->num_of_channels; i++) {
		struct ls_mmbi_chan_desc *chan = &mmbi->chan_desc[i];

		chan->b2h_ba = LS_MMBI_BUF_OFF + i * buf_len * 2;
		chan->h2b_ba = chan->b2h_ba + buf_len;
		chan->b2h_l = buf_len;
		chan->h2b_l = buf_len;
	}

	/* Only channel 0 layout is described in the single descriptor */
	desc->b2h_ba = LS_MMBI_UNITS(mmbi->chan_desc[0].b2h_ba);
	desc->h2b_ba = LS_MMBI_UNITS(mmbi->chan_desc[0].h2b_ba);
	desc->b2h_l = mmbi->chan_desc[0].b2h_l;
	desc->h2b_l = mmbi->chan_desc[0].h2b_l;

	return 0;
}

int ls_mmbi_init(struct ls_mmbi_ins *mmbi)
{
	int i, ret;

	if (!mmbi || !mmbi->dev || !mmbi->virt)
		return -EINVAL;

	if (mmbi->num_of_channels == 0 ||
	    mmbi->num_of_channels > LS_MMBI_MAX_CHANNELS) {
		dev_err(mmbi->dev, "invalid number of channels %u\n",
			mmbi->num_of_channels);
		return -EINVAL;
	}

	if (mmbi->num_of_channels > 1)
		dev_warn(mmbi->dev,
			 "%u channels requested; the LS2010 shared descriptor "
			 "layout only describes channel 0\n",
			 mmbi->num_of_channels);

	mmbi->ins_id = ida_alloc(&ls_mmbi_ida, GFP_KERNEL);
	if (mmbi->ins_id < 0)
		return mmbi->ins_id;

	ret = ls_mmbi_init_layout(mmbi);
	if (ret)
		goto err_ida;

	for (i = 0; i < mmbi->num_of_channels; i++) {
		struct ls_mmbi_chan_desc *chan = &mmbi->chan_desc[i];
		struct ls_mmbi_chan_priv *priv;

		priv = devm_kzalloc(mmbi->dev, sizeof(*priv), GFP_KERNEL);
		if (!priv) {
			ret = -ENOMEM;
			goto err_remove;
		}

		chan->index = i;
		chan->mmbi = mmbi;
		chan->priv = priv;
		priv->chan = chan;
		priv->tx_ready = true;

		spin_lock_init(&priv->rx_lock);
		spin_lock_init(&priv->tx_lock);
		init_waitqueue_head(&priv->rx_wait);
		init_waitqueue_head(&priv->tx_wait);

		ret = ls_mmbi_chan_init_miscdev(chan);
		if (ret)
			goto err_remove;
	}

	dev_info(mmbi->dev,
		 "MMBI instance %d initialized: %zu bytes, %u channels, "
		 "b2h/h2b buffers %u bytes each\n",
		 mmbi->ins_id, mmbi->size, mmbi->num_of_channels,
		 mmbi->chan_desc[0].b2h_l);

	return 0;

err_remove:
	while (i-- > 0) {
		struct ls_mmbi_chan_priv *priv = mmbi->chan_desc[i].priv;

		if (priv)
			misc_deregister(&priv->miscdev);
	}
err_ida:
	ida_free(&ls_mmbi_ida, mmbi->ins_id);
	return ret;
}
EXPORT_SYMBOL_GPL(ls_mmbi_init);

void ls_mmbi_remove(struct ls_mmbi_ins *mmbi)
{
	int i;

	if (!mmbi)
		return;

	for (i = 0; i < mmbi->num_of_channels; i++) {
		struct ls_mmbi_chan_priv *priv = mmbi->chan_desc[i].priv;

		if (priv)
			misc_deregister(&priv->miscdev);
	}

	ida_free(&ls_mmbi_ida, mmbi->ins_id);
}
EXPORT_SYMBOL_GPL(ls_mmbi_remove);

MODULE_DESCRIPTION("Linkedsemi LS2010 MMBI common driver");
MODULE_LICENSE("GPL");
