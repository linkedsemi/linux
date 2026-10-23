#include <linux/init.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/atomic.h>
#include <linux/of_platform.h>
#include <linux/mailbox_client.h>
#include "kcs_bmc_device.h"
#define DEVICE_NAME	"ls-kcs-bmc"

struct peterson_lock {
    bool flag[2];
    uint8_t turn;
} __attribute__((aligned(4)));

typedef struct peterson_lock kcs_lock_t;

struct host_kcs_env {
    kcs_lock_t lock;
    uint8_t status;
    uint8_t data_out;
    uint8_t data_in;
};

#define IDR offsetof(struct host_kcs_env,data_in)
#define ODR offsetof(struct host_kcs_env,data_out)
#define STR offsetof(struct host_kcs_env,status)

struct ls_kcs_bmc {
	struct kcs_bmc_device kcs_bmc;
    struct mbox_client mbox_cl;
    struct mbox_chan *mbox_ch;
	struct host_kcs_env *reg;
	spinlock_t spin_lock;
	u8 event_mask;
};

enum kcs_hb_msg_type {
    KCS_IBF_EVENT,
    KCS_OBF_EVENT,
};

static inline void amp_lock(struct peterson_lock *lock,uint8_t id)
{
	lock->flag[id] = true;
	lock->turn = !id;
	while(lock->flag[!id] && lock->turn == !id);
}

static inline void amp_unlock(struct peterson_lock *lock,uint8_t id)
{
	lock->flag[id] = false;
}

static inline struct ls_kcs_bmc *to_ls_kcs_bmc(struct kcs_bmc_device *kcs_bmc)
{
	return container_of(kcs_bmc, struct ls_kcs_bmc, kcs_bmc);
}

static void kcs_b2h_send_obf(struct ls_kcs_bmc *ls_kcs)
{
    enum kcs_hb_msg_type msg = KCS_OBF_EVENT;
    mbox_send_message(ls_kcs->mbox_ch,&msg);
}

static u8 ls_kcs_inb(struct kcs_bmc_device *kcs_bmc, u32 reg)
{
	struct ls_kcs_bmc *ls_kcs = to_ls_kcs_bmc(kcs_bmc);
	unsigned long flags;
	u8 ret = 0;
	if(reg == IDR)
	{
		spin_lock_irqsave(&ls_kcs->spin_lock,flags);
		amp_lock(&ls_kcs->reg->lock,1);
		ls_kcs->reg->status &= ~KCS_BMC_STR_IBF;
		ret = ls_kcs->reg->data_in;
		amp_unlock(&ls_kcs->reg->lock,1);
		spin_unlock_irqrestore(&ls_kcs->spin_lock,flags);
	}else if(reg == STR)
	{
		ret = ls_kcs->reg->status;
	}else{
		dev_err(kcs_bmc->dev, "ls_kcs_inb error\n");
		while(1);
	}
	return ret;
	
}

static void ls_kcs_outb(struct kcs_bmc_device *kcs_bmc, u32 reg, u8 data)
{
	struct ls_kcs_bmc *ls_kcs = to_ls_kcs_bmc(kcs_bmc);
	unsigned long flags;
	if(reg == ODR)
	{
		spin_lock_irqsave(&ls_kcs->spin_lock,flags);
		amp_lock(&ls_kcs->reg->lock,1);
		ls_kcs->reg->status |= KCS_BMC_STR_OBF;
		ls_kcs->reg->data_out = data;
		amp_unlock(&ls_kcs->reg->lock,1);
		spin_unlock_irqrestore(&ls_kcs->spin_lock,flags);
		kcs_b2h_send_obf(ls_kcs);
	}else
	{
		dev_err(kcs_bmc->dev, "ls_kcs_outb error\n");
		while(1);
	}

}

static void ls_kcs_updateb(struct kcs_bmc_device *kcs_bmc, u32 reg, u8 mask, u8 data)
{
	struct ls_kcs_bmc *ls_kcs = to_ls_kcs_bmc(kcs_bmc);
	unsigned long flags;
	if(reg == STR)
	{
		spin_lock_irqsave(&ls_kcs->spin_lock,flags);
		amp_lock(&ls_kcs->reg->lock,1);
		ls_kcs->reg->status = (ls_kcs->reg->status & ~mask) | data;
		amp_unlock(&ls_kcs->reg->lock,1);
		spin_unlock_irqrestore(&ls_kcs->spin_lock,flags);
	}else{
		dev_err(kcs_bmc->dev, "ls_kcs_updateb error\n");
		while(1);
	}
}

static void ls_kcs_irq_mask_update(struct kcs_bmc_device *kcs_bmc, u8 mask, u8 state)
{
	struct ls_kcs_bmc *ls_kcs = to_ls_kcs_bmc(kcs_bmc);
	ls_kcs->event_mask = (ls_kcs->event_mask & ~mask) | state;
}

static const struct kcs_bmc_device_ops ls_kcs_ops = {
	.irq_mask_update = ls_kcs_irq_mask_update,
	.io_inputb = ls_kcs_inb,
	.io_outputb = ls_kcs_outb,
	.io_updateb = ls_kcs_updateb,
};

static int ls_kcs_of_get_channel(struct platform_device *pdev)
{
	struct device_node *np;
	u32 channel;
	int rc;

	np = pdev->dev.of_node;

	rc = of_property_read_u32(np, "kcs_chan", &channel);
	if ((rc != 0) || channel == 0) {
		dev_err(&pdev->dev, "no valid 'kcs_chan' configured\n");
		return -EINVAL;
	}

	return channel;
}

static void ls_kcs_mbox_rx(struct mbox_client *cl, void *mssg)
{
    struct ls_kcs_bmc *ls_kcs = container_of(cl,struct ls_kcs_bmc,mbox_cl);
	kcs_bmc_handle_event(&ls_kcs->kcs_bmc);
}

static int ls_kcs_probe(struct platform_device *pdev)
{
	struct ls_kcs_bmc *priv;
	struct kcs_bmc_device *kcs_bmc;
	u32 chan;
	int rc;


	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

    struct resource *res = platform_get_resource(pdev,IORESOURCE_MEM,0);
    if(!res) {
        dev_err(&pdev->dev,"Fail to get ls-kcs share memory\n");
        return -EINVAL;
    }
    priv->reg = devm_ioremap_resource(&pdev->dev,res);
 	if (IS_ERR(priv->reg)) {
        dev_err(&pdev->dev,"Fail to remap kcs reg\n");
 		return PTR_ERR(priv->reg);
    }

    priv->mbox_cl.dev = &pdev->dev;
    priv->mbox_cl.rx_callback = ls_kcs_mbox_rx;
    priv->mbox_ch = mbox_request_channel(&priv->mbox_cl,0);
    if(IS_ERR(priv->mbox_ch)) {
		dev_err(&pdev->dev, "mbox request fail\n");
        return -ENODEV;
    }

	chan = ls_kcs_of_get_channel(pdev);
	if(chan < 0)
		return chan;

	spin_lock_init(&priv->spin_lock);

	kcs_bmc = &priv->kcs_bmc;
	kcs_bmc->dev = &pdev->dev;
	kcs_bmc->channel = chan;
	kcs_bmc->ioreg.idr = IDR;
	kcs_bmc->ioreg.odr = ODR;
	kcs_bmc->ioreg.str = STR;
	kcs_bmc->ops = &ls_kcs_ops;

	platform_set_drvdata(pdev, priv);

	rc = kcs_bmc_add_device(kcs_bmc);
	if (rc) {
		dev_err(&pdev->dev, "Failed to register channel %d: %d\n", kcs_bmc->channel, rc);
		return rc;
	}
	
	dev_info(&pdev->dev,"channel=%u idr=0x%x odr=0x%x str=0x%x\n",
		chan,kcs_bmc->ioreg.idr, kcs_bmc->ioreg.odr, kcs_bmc->ioreg.str);

	return 0;
}

static void ls_kcs_remove(struct platform_device *pdev)
{
	struct ls_kcs_bmc *priv = platform_get_drvdata(pdev);
	struct kcs_bmc_device *kcs_bmc = &priv->kcs_bmc;
	kcs_bmc_remove_device(kcs_bmc);

}

static const struct of_device_id ls_kcs_bmc_match[] = {
	{ .compatible = "linkedsemi,ls-kcs-bmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, ls_kcs_bmc_match);

static struct platform_driver ls_kcs_bmc_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.of_match_table	= ls_kcs_bmc_match,
	},
	.probe	= ls_kcs_probe,
	.remove	= ls_kcs_remove,
};
module_platform_driver(ls_kcs_bmc_driver);
MODULE_DESCRIPTION("LINKEDSEMI KCS BMC DRIVER");
MODULE_LICENSE("GPL");
