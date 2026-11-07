// SPDX-License-Identifier: GPL-2.0
/*
 * Linkedsemi LS2010 Post Code Capture (PCC) driver
 *
 * The E906 firmware (soc/linkedsemi/common/espi_lpc/espi_lpc_host_pcc.c)
 * snoops host I/O writes to the configured post-code ports and pushes
 * each captured code into the shared host_pcc_env FIFO, then raises a
 * mailbox interrupt to announce it.  On the mailbox event this driver
 * drains the shared FIFO into a kfifo and exposes the codes to user
 * space through a misc device, with the same read(2)/poll(2) semantics
 * as the Aspeed LPC PCC driver (drivers/soc/aspeed/aspeed-lpc-pcc.c).
 */

#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/lock_free_fifo.h>
#include <linux/mailbox_client.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wait.h>

#define DEVICE_NAME "linkedsemi-pcc"

/*
 * One post code captured by the E906 host PCC.  Keep in sync with
 * struct pcc_data in zephyr/include/zephyr/drivers/pcc.h.
 */
struct pcc_data {
	u8 post_code;
	u8 port;
};

/*
 * Post code FIFO shared with the E906 firmware.  Keep in sync with
 * struct host_pcc_env in soc/linkedsemi/common/espi_lpc/espi_lpc_common.h.
 * The E906 is a 32-bit core, so the embedded fifo buffer pointer is u32
 * wide; the layout matches struct fifo_env in the firmware's fifo.h.
 */
struct host_pcc_env {
	struct lf_fifo_env fifo;
	struct pcc_data data[];
};

struct ls_pcc {
	struct mbox_client mbox_cl;
	struct mbox_chan *mbox_ch;
	struct host_pcc_env *env;
	struct kfifo fifo;
	wait_queue_head_t wq;
	struct miscdevice mdev;
};

/*
 * The mailbox message only announces that post codes are pending; the
 * codes themselves are drained from the shared host_pcc_env FIFO owned
 * by the E906 firmware.
 */
static void ls_pcc_mbox_rx(struct mbox_client *cl, void *mssg)
{
	struct ls_pcc *pcc = container_of(cl, struct ls_pcc, mbox_cl);
	struct pcc_data data;

	while (lf_fifo_data_get(&pcc->env->fifo, pcc->env->data, &data)) {
		/* If the kfifo has enough space, insert the new data;
		 * otherwise, discard it.
		 */
		if (kfifo_in(&pcc->fifo, &data, sizeof(data)) != sizeof(data))
			dev_err_ratelimited(pcc->mbox_cl.dev,
					    "post code fifo full, dropping %#x\n",
					    data.post_code);
	}

	wake_up_interruptible(&pcc->wq);
}

static ssize_t ls_pcc_read(struct file *file, char __user *buffer,
			   size_t count, loff_t *ppos)
{
	int rc;
	unsigned int copied;
	struct ls_pcc *pcc = container_of(file->private_data,
					  struct ls_pcc, mdev);

	if (kfifo_is_empty(&pcc->fifo)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		rc = wait_event_interruptible(pcc->wq,
					      !kfifo_is_empty(&pcc->fifo));
		if (rc == -ERESTARTSYS)
			return -EINTR;
	}

	rc = kfifo_to_user(&pcc->fifo, buffer, count, &copied);

	return rc ? rc : copied;
}

static __poll_t ls_pcc_poll(struct file *file, struct poll_table_struct *pt)
{
	struct ls_pcc *pcc = container_of(file->private_data,
					  struct ls_pcc, mdev);

	poll_wait(file, &pcc->wq, pt);

	return !kfifo_is_empty(&pcc->fifo) ? POLLIN : 0;
}

static const struct file_operations ls_pcc_fops = {
	.owner = THIS_MODULE,
	.read = ls_pcc_read,
	.poll = ls_pcc_poll,
};

static int ls_pcc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ls_pcc *pcc;
	int id, rc;

	pcc = devm_kzalloc(dev, sizeof(*pcc), GFP_KERNEL);
	if (!pcc)
		return -ENOMEM;

	pcc->env = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pcc->env)) {
		dev_err(dev, "failed to remap host_pcc_env\n");
		return PTR_ERR(pcc->env);
	}

	rc = kfifo_alloc(&pcc->fifo, PAGE_SIZE, GFP_KERNEL);
	if (rc)
		return rc;

	init_waitqueue_head(&pcc->wq);

	id = of_alias_get_id(dev->of_node, "pcc");
	if (id < 0)
		id = 0;

	pcc->mbox_cl.dev = dev;
	pcc->mbox_cl.rx_callback = ls_pcc_mbox_rx;
	pcc->mbox_ch = mbox_request_channel(&pcc->mbox_cl, 0);
	if (IS_ERR(pcc->mbox_ch)) {
		dev_err(dev, "failed to request post code mailbox channel\n");
		rc = PTR_ERR(pcc->mbox_ch);
		goto err_free_fifo;
	}

	pcc->mdev.parent = dev;
	pcc->mdev.minor = MISC_DYNAMIC_MINOR;
	pcc->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s%d", DEVICE_NAME, id);
	if (!pcc->mdev.name) {
		rc = -ENOMEM;
		goto err_free_channel;
	}
	pcc->mdev.fops = &ls_pcc_fops;
	rc = misc_register(&pcc->mdev);
	if (rc) {
		dev_err(dev, "failed to register misc device\n");
		goto err_free_channel;
	}

	platform_set_drvdata(pdev, pcc);

	dev_info(dev, "%s registered\n", pcc->mdev.name);

	return 0;

err_free_channel:
	mbox_free_channel(pcc->mbox_ch);
err_free_fifo:
	kfifo_free(&pcc->fifo);

	return rc;
}

static void ls_pcc_remove(struct platform_device *pdev)
{
	struct ls_pcc *pcc = platform_get_drvdata(pdev);

	misc_deregister(&pcc->mdev);
	mbox_free_channel(pcc->mbox_ch);
	kfifo_free(&pcc->fifo);
}

static const struct of_device_id ls_pcc_of_match[] = {
	{ .compatible = "linkedsemi,ls-pcc" },
	{ }
};
MODULE_DEVICE_TABLE(of, ls_pcc_of_match);

static struct platform_driver ls_pcc_driver = {
	.probe = ls_pcc_probe,
	.remove = ls_pcc_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = ls_pcc_of_match,
	},
};
module_platform_driver(ls_pcc_driver);

MODULE_DESCRIPTION("Linkedsemi LS2010 Post Code Capture driver");
MODULE_LICENSE("GPL");
