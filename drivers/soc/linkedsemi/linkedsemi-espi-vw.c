// SPDX-License-Identifier: GPL-2.0
/*
 * Linkedsemi LS2010 eSPI virtual wire (VW) driver
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <uapi/linux/linkedsemi-espi-vw.h>

#define DEVICE_NAME			"linkedsemi-espi"

/*
 * Downstream VW status shared with the E906 firmware.  Keep in sync with
 * struct espi_h2b_vw_base in
 * soc/linkedsemi/common/espi_lpc/espi_lpc_common.h.
 */
struct espi_h2b_vw_base {
	u8 s02_ms;
	u8 s03_ms;
	u8 s07_ms;
	u8 ms_vw_val[LS_ESPI_VW_MS_NUM];
};

struct ls_espi_vw {
	struct mbox_client mbox_cl;
	struct mbox_chan *mbox_ch;

	/* Downstream VW levels shared with the E906 firmware. */
	struct espi_h2b_vw_base __iomem *h2b;

	struct miscdevice vw_mdev;
	struct miscdevice pltrst_mdev;

	/* PLTRST# monitor, follows ast2700-espi.c semantics. */
	spinlock_t pltrst_lock;
	wait_queue_head_t pltrst_wq;
	char pltrst_status;
	bool pltrst_avail;
};

static struct ls_espi_vw *to_ls_espi_vw_from_vw(struct file *filp)
{
	return container_of(filp->private_data, struct ls_espi_vw, vw_mdev);
}

static struct ls_espi_vw *to_ls_espi_vw_from_pltrst(struct file *filp)
{
	return container_of(filp->private_data, struct ls_espi_vw,
			    pltrst_mdev);
}

static void ls_espi_vw_mbox_rx(struct mbox_client *cl, void *mssg)
{
	struct ls_espi_vw *vw = container_of(cl, struct ls_espi_vw, mbox_cl);
	struct espi_vwire_msg *msg = mssg;
	unsigned long flags;

	if (msg->vw_idx == LS_ESPI_VW_SYS_EVT_3) {
		char status = (msg->vw_val & LS_ESPI_SYS_EVT_3_PLTRST_N) ? '1' : '0';
		bool changed = false;

		spin_lock_irqsave(&vw->pltrst_lock, flags);
		if (status != vw->pltrst_status) {
			vw->pltrst_status = status;
			vw->pltrst_avail = true;
			changed = true;
		}
		spin_unlock_irqrestore(&vw->pltrst_lock, flags);

		if (changed)
			wake_up_interruptible(&vw->pltrst_wq);
	}
}

static int ls_espi_vw_send(struct ls_espi_vw *vw, u8 idx, u8 val)
{
	struct espi_vwire_msg vw_msg = {.vw_idx = idx, .vw_val = val};
	int ret;

	ret = mbox_send_message(vw->mbox_ch, &vw_msg);
	if (ret < 0)
		dev_err(vw->mbox_cl.dev,
			"failed to send VW %#x/%#x: %d\n", idx, val, ret);

	return ret;
}

static long ls_espi_vw_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct ls_espi_vw *vw = to_ls_espi_vw_from_vw(filp);
	void __iomem *reg;
	u8 val;

	switch (cmd) {
	case LS_ESPI_VW_GET_S02_MS:
		reg = &vw->h2b->s02_ms;
		break;
	case LS_ESPI_VW_GET_S03_MS:
		reg = &vw->h2b->s03_ms;
		break;
	case LS_ESPI_VW_GET_S07_MS:
		reg = &vw->h2b->s07_ms;
		break;
	case LS_ESPI_VW_GET_MS_VW_VAL: {
		struct ls_espi_vw_ms_vw_arg vw_arg;

		if (copy_from_user(&vw_arg, (void __user *)arg,
				   sizeof(vw_arg)))
			return -EFAULT;
		if (vw_arg.idx >= LS_ESPI_VW_MS_NUM)
			return -EINVAL;
		vw_arg.val = readb(&vw->h2b->ms_vw_val[vw_arg.idx]);
		if (copy_to_user((void __user *)arg, &vw_arg, sizeof(vw_arg)))
			return -EFAULT;
		return 0;
	}
	case LS_ESPI_VW_PUT_VW: {
		struct espi_vwire_msg msg;

		if (copy_from_user(&msg, (void __user *)arg, sizeof(msg)))
			return -EFAULT;
		return ls_espi_vw_send(vw, msg.vw_idx, msg.vw_val);
	}
	default:
		return -EINVAL;
	}

	val = readb(reg);
	if (put_user(val, (u8 __user *)arg))
		return -EFAULT;

	return 0;
}

static const struct file_operations ls_espi_vw_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ls_espi_vw_ioctl,
};

static int ls_espi_vw_pltrst_open(struct inode *inode, struct file *filp)
{
	struct ls_espi_vw *vw = to_ls_espi_vw_from_pltrst(filp);
	unsigned long flags;

	if ((filp->f_flags & O_ACCMODE) != O_RDONLY)
		return -EACCES;

	/* Same as ast2700-espi.c: report the current sample after open. */
	spin_lock_irqsave(&vw->pltrst_lock, flags);
	vw->pltrst_avail = true;
	spin_unlock_irqrestore(&vw->pltrst_lock, flags);

	return 0;
}

static ssize_t ls_espi_vw_pltrst_read(struct file *filp, char __user *buf,
				      size_t count, loff_t *offset)
{
	struct ls_espi_vw *vw = to_ls_espi_vw_from_pltrst(filp);
	DECLARE_WAITQUEUE(wait, current);
	char data, old_sample;
	int ret = 0;

	spin_lock_irq(&vw->pltrst_lock);

	if (filp->f_flags & O_NONBLOCK) {
		if (!vw->pltrst_avail) {
			ret = -EAGAIN;
			goto out_unlock;
		}

		data = vw->pltrst_status;
		vw->pltrst_avail = false;
	} else {
		add_wait_queue(&vw->pltrst_wq, &wait);
		set_current_state(TASK_INTERRUPTIBLE);

		old_sample = vw->pltrst_status;

		do {
			if (old_sample != vw->pltrst_status) {
				data = vw->pltrst_status;
				vw->pltrst_avail = false;
				break;
			}

			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
			} else {
				spin_unlock_irq(&vw->pltrst_lock);
				schedule();
				spin_lock_irq(&vw->pltrst_lock);
			}
		} while (!ret);

		remove_wait_queue(&vw->pltrst_wq, &wait);
		set_current_state(TASK_RUNNING);
	}

out_unlock:
	spin_unlock_irq(&vw->pltrst_lock);
	if (ret)
		return ret;

	if (put_user(data, buf))
		return -EFAULT;

	return sizeof(data);
}

static unsigned int ls_espi_vw_pltrst_poll(struct file *filp,
					   poll_table *wait)
{
	struct ls_espi_vw *vw = to_ls_espi_vw_from_pltrst(filp);
	unsigned long flags;
	unsigned int mask = 0;

	poll_wait(filp, &vw->pltrst_wq, wait);

	spin_lock_irqsave(&vw->pltrst_lock, flags);
	if (vw->pltrst_avail)
		mask |= POLLIN;
	spin_unlock_irqrestore(&vw->pltrst_lock, flags);

	return mask;
}

static const struct file_operations ls_espi_vw_pltrst_fops = {
	.owner = THIS_MODULE,
	.open = ls_espi_vw_pltrst_open,
	.read = ls_espi_vw_pltrst_read,
	.poll = ls_espi_vw_pltrst_poll,
};

static int ls_espi_vw_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ls_espi_vw *vw;
	u32 h2b_base;
	int id, ret;

	vw = devm_kzalloc(dev, sizeof(*vw), GFP_KERNEL);
	if (!vw)
		return -ENOMEM;

	spin_lock_init(&vw->pltrst_lock);
	init_waitqueue_head(&vw->pltrst_wq);
	vw->pltrst_status = 'U';
	vw->pltrst_avail = false;

	/*
	 * The sysevent mirror sits at the tail of the E906's eSPI0 controller
	 * window and is maintained by the E906 firmware (espi_ls.c).  Map it
	 * without requesting the region exclusively: it belongs to the
	 * firmware, not to any Linux device.
	 */
	ret = of_property_read_u32(dev->of_node, "h2b-vw-base", &h2b_base);
	if (ret) {
		dev_err(dev, "missing 'h2b-vw-base'\n");
		return ret;
	}
	vw->h2b = devm_ioremap(dev, h2b_base, sizeof(*vw->h2b));
	if (!vw->h2b) {
		dev_err(dev, "failed to map shared VW status region\n");
		return -ENOMEM;
	}

	vw->mbox_cl.dev = dev;
	vw->mbox_cl.rx_callback = ls_espi_vw_mbox_rx;
	vw->mbox_cl.tx_block = false;
	vw->mbox_ch = mbox_request_channel(&vw->mbox_cl, 0);
	if (IS_ERR(vw->mbox_ch)) {
		dev_err(dev, "failed to request VW mailbox channel\n");
		return PTR_ERR(vw->mbox_ch);
	}

	id = of_alias_get_id(dev->of_node, "espi");
	if (id < 0)
		id = 0;

	vw->vw_mdev.parent = dev;
	vw->vw_mdev.minor = MISC_DYNAMIC_MINOR;
	vw->vw_mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s-vw%d",
					  DEVICE_NAME, id);
	vw->vw_mdev.fops = &ls_espi_vw_fops;
	if (!vw->vw_mdev.name) {
		ret = -ENOMEM;
		goto err_free_channel;
	}

	vw->pltrst_mdev.parent = dev;
	vw->pltrst_mdev.minor = MISC_DYNAMIC_MINOR;
	vw->pltrst_mdev.name = devm_kasprintf(dev, GFP_KERNEL,
					      "%s-pltrstn%d", DEVICE_NAME, id);
	vw->pltrst_mdev.fops = &ls_espi_vw_pltrst_fops;
	if (!vw->pltrst_mdev.name) {
		ret = -ENOMEM;
		goto err_free_channel;
	}

	ret = misc_register(&vw->vw_mdev);
	if (ret) {
		dev_err(dev, "cannot register %s: %d\n", vw->vw_mdev.name, ret);
		goto err_free_channel;
	}

	ret = misc_register(&vw->pltrst_mdev);
	if (ret) {
		dev_err(dev, "cannot register %s: %d\n",
			vw->pltrst_mdev.name, ret);
		goto err_dereg_vw;
	}

	platform_set_drvdata(pdev, vw);

	dev_info(dev, "%s and %s registered\n",
		 vw->vw_mdev.name, vw->pltrst_mdev.name);

	return 0;

err_dereg_vw:
	misc_deregister(&vw->vw_mdev);
err_free_channel:
	mbox_free_channel(vw->mbox_ch);
	return ret;
}

static void ls_espi_vw_remove(struct platform_device *pdev)
{
	struct ls_espi_vw *vw = platform_get_drvdata(pdev);

	misc_deregister(&vw->pltrst_mdev);
	misc_deregister(&vw->vw_mdev);
	mbox_free_channel(vw->mbox_ch);
}

static const struct of_device_id ls_espi_vw_of_match[] = {
	{ .compatible = "linkedsemi,ls-espi-vw" },
	{ }
};
MODULE_DEVICE_TABLE(of, ls_espi_vw_of_match);

static struct platform_driver ls_espi_vw_driver = {
	.probe = ls_espi_vw_probe,
	.remove = ls_espi_vw_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = ls_espi_vw_of_match,
	},
};
module_platform_driver(ls_espi_vw_driver);

MODULE_DESCRIPTION("Linkedsemi LS2010 eSPI virtual wire driver");
MODULE_LICENSE("GPL");
