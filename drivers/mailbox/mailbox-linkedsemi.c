
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/lock_free_fifo.h>

/* mboxes cell layout: <&mbox channel tx_width tx_depth rx_width rx_depth> */
#define LS_MBOX_DT_CELLS	5

struct chan_info {
    struct lf_fifo_env *env;
    u16 width;
    u16 depth;
};

struct mbox_chan_info {
    struct chan_info tx;
    struct chan_info rx;
    bool started;
};

struct ls_mbox_priv { 
	struct mbox_controller mbox;
    struct mbox_chan_info *ch_info;
    spinlock_t rx_lock;
    int irq;
    struct regmap *tx_reg_base;
    struct regmap *rx_reg_base;
    u16 tx_intr_set_offset;
    u16 rx_intr_clr_offset;
};

#define GET_WR_IDX(fifo) ((fifo)->wr_idx>=(fifo)->length?(fifo)->wr_idx - (fifo)->length:(fifo)->wr_idx)
#define GET_RD_IDX(fifo) ((fifo)->rd_idx>=(fifo)->length?(fifo)->rd_idx - (fifo)->length:(fifo)->rd_idx)

static inline void *chan_buf(struct chan_info *info)
{
    return (void *)info->env + sizeof(struct lf_fifo_env);
}

static bool tx_data_put(struct chan_info *info,void *data)
{
    return lf_fifo_data_put(info->env,chan_buf(info),data);
}

static int ls_mbox_send_data(struct mbox_chan *chan, void *data)
{
    struct mbox_controller *mbox = chan->mbox;
    struct ls_mbox_priv *priv = container_of(mbox,struct ls_mbox_priv,mbox);
    int ch_idx = chan - mbox->chans;
    if(tx_data_put(&priv->ch_info[ch_idx].tx,data)==false)
    {
        dev_err(mbox->dev,"mbox ch %d tx fifo full", ch_idx);
        return -EBUSY;
    }
    regmap_write(priv->tx_reg_base,priv->tx_intr_set_offset,1);
    return 0;

}

static bool ls_mbox_last_tx_done(struct mbox_chan *chan)
{
    return true;
}

static int ls_mbox_startup(struct mbox_chan *chan);
static void ls_mbox_shutdown(struct mbox_chan *chan);

static const struct mbox_chan_ops ls_mbox_ops = {
	.send_data = ls_mbox_send_data,
    .flush = NULL,
	.startup = ls_mbox_startup,
	.shutdown = ls_mbox_shutdown,
    .last_tx_done = ls_mbox_last_tx_done,
    .peek_data = NULL,
};

static bool rx_data_peek(struct chan_info *info,void **data)
{
    return lf_fifo_data_peek(info->env,chan_buf(info),data);
}

static void mbox_rx(struct ls_mbox_priv *priv,int ch_idx)
{
    struct mbox_chan_info *info = &priv->ch_info[ch_idx];
    void *data;
    while (info->started && priv->mbox.chans[ch_idx].cl 
        && rx_data_peek(&info->rx, &data)) {
        mbox_chan_received_data(&priv->mbox.chans[ch_idx],data);
        lf_fifo_data_drop(info->rx.env);
    }
}

static irqreturn_t ls_mbox_irq(int irq, void *p)
{
    struct ls_mbox_priv *priv = p;
    regmap_write(priv->rx_reg_base,priv->rx_intr_clr_offset,0);
    spin_lock(&priv->rx_lock);
    for(int i=0; i < priv->mbox.num_chans; ++i)
    {
        mbox_rx(priv,i);
    }
    spin_unlock(&priv->rx_lock);
    return IRQ_HANDLED;
}

static int ls_mbox_startup(struct mbox_chan *chan)
{
    struct ls_mbox_priv *priv = container_of(chan->mbox,struct ls_mbox_priv,mbox);
    struct mbox_chan_info *info = chan->con_priv;
    unsigned long flags;
    int ch_idx = chan - chan->mbox->chans;

    if(!info) {
        return -EINVAL;
    }

    spin_lock_irqsave(&priv->rx_lock, flags);
    info->started = true;
    mbox_rx(priv, ch_idx);
    spin_unlock_irqrestore(&priv->rx_lock, flags);

    return 0;
}

static void ls_mbox_shutdown(struct mbox_chan *chan)
{
    struct ls_mbox_priv *priv = container_of(chan->mbox,struct ls_mbox_priv,mbox);
    struct mbox_chan_info *info = chan->con_priv;
    unsigned long flags;

    if(!info) {
        return;
    }

    spin_lock_irqsave(&priv->rx_lock, flags);
    info->started = false;
    spin_unlock_irqrestore(&priv->rx_lock, flags);

    synchronize_irq(priv->irq);
}

static int count_clients_in_dt(struct device_node *controller_dn)
{
    struct device_node *client_dn;
    int count = 0;
    for_each_node_with_property(client_dn,"mboxes") {
		if (!of_device_is_available(client_dn))
			continue;
        struct of_phandle_args args;
        if (of_parse_phandle_with_args(client_dn, "mboxes", "#mbox-cells", 0, &args))
            continue;
        if(args.np == controller_dn && args.args_count == LS_MBOX_DT_CELLS) {
            count++;
        }
        of_node_put(args.np);
    }
    return count;
}

static void *mbox_chan_info_assign(struct chan_info *ch,void *mem)
{
    ch->env = mem;
    return mem + ALIGN(sizeof(struct lf_fifo_env) + ch->width * ch->depth, 4);
}

static void ls_mbox_fifo_init(struct lf_fifo_env *env, struct chan_info *ch, phys_addr_t env_phys)
{
    env->buf = env_phys + sizeof(struct lf_fifo_env);
    env->rd_idx = 0;
    env->wr_idx = 0;
    env->length = ch->depth;
    env->item_size = ch->width;
}

static int collect_mbox_channels_info(struct ls_mbox_priv *priv, struct device_node *controller_dn, struct resource *res)
{
    struct device_node *client_dn;
    int err = 0;

    for_each_node_with_property(client_dn,"mboxes") {
        struct of_phandle_args args;
        unsigned int ch_idx;

		if (!of_device_is_available(client_dn))
			continue;
        if(of_parse_phandle_with_args(client_dn,"mboxes","#mbox-cells",0,&args))
            continue;
        if(args.np != controller_dn) {
            of_node_put(args.np);
            continue;
        }
        of_node_put(args.np);

        if(args.args_count != LS_MBOX_DT_CELLS) {
            dev_err(priv->mbox.dev, "client %pOFn: expected %d mboxes cells, got %d\n",
                    client_dn, LS_MBOX_DT_CELLS, args.args_count);
            err = -EINVAL;
            continue;
        }

        ch_idx = args.args[0];
        if(ch_idx >= priv->mbox.num_chans) {
            dev_err(priv->mbox.dev, "client %pOFn: channel %u out of range\n",
                    client_dn, ch_idx);
            err = -EINVAL;
            continue;
        }
        if(priv->ch_info[ch_idx].tx.width || priv->ch_info[ch_idx].rx.width) {
            dev_err(priv->mbox.dev, "client %pOFn: duplicate channel %u\n",
                    client_dn, ch_idx);
            err = -EINVAL;
            continue;
        }

        priv->ch_info[ch_idx].tx.width = args.args[1];
        priv->ch_info[ch_idx].tx.depth = args.args[2];
        priv->ch_info[ch_idx].rx.width = args.args[3];
        priv->ch_info[ch_idx].rx.depth = args.args[4];
        dev_info(priv->mbox.dev,"channel %d, tx width %d, tx depth %d, rx width %d, rx depth %d\n",
            ch_idx,args.args[1],args.args[2],args.args[3],args.args[4]);
    }

    if(err) {
        return err;
    }

    for(int i=0; i<priv->mbox.num_chans; ++i) {
        if(!priv->ch_info[i].tx.width || !priv->ch_info[i].tx.depth ||
           !priv->ch_info[i].rx.width || !priv->ch_info[i].rx.depth) {
            dev_err(priv->mbox.dev, "mbox channel %d is not configured\n", i);
            return -EINVAL;
        }
    }

    size_t need_size = 0;
    for(int i=0; i<priv->mbox.num_chans; ++i) {
        need_size += ALIGN(sizeof(struct lf_fifo_env) +
                           (size_t)priv->ch_info[i].tx.width *
                           priv->ch_info[i].tx.depth, 4);
        need_size += ALIGN(sizeof(struct lf_fifo_env) +
                           (size_t)priv->ch_info[i].rx.width *
                           priv->ch_info[i].rx.depth, 4);
    }
    if(resource_size(res) < need_size) {
        dev_err(priv->mbox.dev, "mbox shared memory too small: need %zu bytes, have %llu\n",
                need_size, (unsigned long long)resource_size(res));
        return -EINVAL;
    }

    void *mem = devm_ioremap_resource(priv->mbox.dev,res);
    if(IS_ERR(mem)) {
        dev_err(priv->mbox.dev, "Fail to remap ls-mbox share memory\n");
        return PTR_ERR(mem);
    }
    void *mbox_base = mem;
    for(int i=0; i<priv->mbox.num_chans; ++i) {
        struct chan_info *tx = &priv->ch_info[i].tx;

        ls_mbox_fifo_init(mem, tx,mem - mbox_base + res->start);
        mem = mbox_chan_info_assign(tx, mem);
        mem = mbox_chan_info_assign(&priv->ch_info[i].rx, mem);
    }
    wmb();
    return 0;
}

static struct mbox_chan *ls_mbox_xlate(struct mbox_controller *mbox,const struct of_phandle_args *sp)
{
    if(!sp || sp->args_count != LS_MBOX_DT_CELLS ||
       sp->args[0] >= mbox->num_chans) {
        return ERR_PTR(-EINVAL);
    }
    return &mbox->chans[sp->args[0]];
}

static int ls_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
    struct ls_mbox_priv *priv = devm_kzalloc(dev,sizeof(*priv),GFP_KERNEL);
    if(!priv) {
        dev_err(dev, "Failed to allocate memory for ls-mbox\n");
        return -ENOMEM;
    }
    priv->mbox.dev = dev;
    priv->mbox.ops = &ls_mbox_ops;
    priv->mbox.num_chans = count_clients_in_dt(dev->of_node);
    priv->mbox.chans = devm_kcalloc(dev,priv->mbox.num_chans,sizeof(*priv->mbox.chans),GFP_KERNEL);
    if(!priv->mbox.chans) {
        dev_err(dev, "Failed to allocate memory for ls-mbox channels\n");
        return -ENOMEM;
    }
    priv->mbox.of_xlate = ls_mbox_xlate;
    priv->mbox.txdone_irq = false;
    priv->mbox.txdone_poll = true;
    priv->mbox.txpoll_period = 0;

    priv->ch_info = devm_kcalloc(dev,priv->mbox.num_chans,sizeof(*priv->ch_info),GFP_KERNEL);
    if(!priv->ch_info) {
        dev_err(dev, "Failed to allocate memory for ls-mbox channels info\n");
        return -ENOMEM;
    }
    spin_lock_init(&priv->rx_lock);

    for(int i=0;i<priv->mbox.num_chans;++i)
    {
        priv->mbox.chans[i].con_priv = &priv->ch_info[i];
    }

    priv->tx_reg_base = syscon_regmap_lookup_by_phandle(dev->of_node, "tx-syscon");
    if(IS_ERR(priv->tx_reg_base)) {
        dev_err(dev, "Failed to get ls-mbox tx-syscon\n");
        return PTR_ERR(priv->tx_reg_base);
    }

    priv->rx_reg_base = syscon_regmap_lookup_by_phandle(dev->of_node, "rx-syscon");
    if(IS_ERR(priv->rx_reg_base)) {
        dev_err(dev, "Failed to get ls-mbox rx-syscon\n");
        return PTR_ERR(priv->rx_reg_base);
    }

    struct resource *res = platform_get_resource(pdev,IORESOURCE_MEM,0);
    if(!res) {
        dev_err(dev,"Fail to get ls-mbox share memory\n");
        return -EINVAL;
    }

    int rc = collect_mbox_channels_info(priv,dev->of_node,res);
    if(rc < 0) {
        return rc;
    }

    if(of_property_read_u16(dev->of_node,"tx-intr-set-offset",&priv->tx_intr_set_offset))
    {
        dev_err(dev,"Failed to get ls-mbox tx-intr-set-offset");
        return -EINVAL;
    }

    if(of_property_read_u16(dev->of_node,"rx-intr-clr-offset",&priv->rx_intr_clr_offset))
    {
        dev_err(dev,"Failed to get ls-mbox rx-intr-clr-offset");
        return -EINVAL;
    }

	platform_set_drvdata(pdev, priv);

    int irq = platform_get_irq(pdev, 0);
    if(irq < 0) {
		dev_err(dev, "cannot get ls-mbox irq\n");
        return irq;
    }
    priv->irq = irq;

    int ret = devm_request_irq(dev,irq,ls_mbox_irq,IRQF_SHARED,dev_name(dev),priv);
    if(ret < 0) {
        dev_err(dev,"Fail to request ls-mbox irq\n");
        return ret;
    }

    ret = devm_mbox_controller_register(dev,&priv->mbox);
    if(ret) {
        dev_err(dev,"mbox controller register fail ,%d\n",ret);
        return ret;
    }

    return 0;
}

static const struct of_device_id ls_mbox_dt_ids[] = {
	{ .compatible = "linkedsemi,ls-mbox" },
	{}
};
MODULE_DEVICE_TABLE(of, ls_mbox_dt_ids);

static struct platform_driver ls_mbox_driver = {
	.probe		= ls_mbox_probe,
	.driver = {
		.name	= "linkedsemi-mbox",
		.of_match_table = ls_mbox_dt_ids,
	},
};
module_platform_driver(ls_mbox_driver);

MODULE_DESCRIPTION("LINKEDSEMI MAILBOX DRIVER");
MODULE_LICENSE("GPL");
