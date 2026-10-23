#include <linux/init.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/platform_device.h>
#include <linux/mailbox_client.h>
#include <linux/workqueue.h>
#include <linux/lock_free_fifo.h>
#define DEVICE_NAME	"ls-vuart"
#define LS_VUART_NAME DEVICE_NAME
#define LS_VUART_NR_UARTS 2

#define VUART_FIFO_SIZE 16

struct host_vuart_fifo {
    uint32_t init_magic;
    struct lf_fifo_env b2h;
    struct lf_fifo_env h2b;
    uint8_t b2h_buf[VUART_FIFO_SIZE];
    uint8_t h2b_buf[VUART_FIFO_SIZE];
    bool host_rx_from_vuart;
    bool host_tx_to_vuart;
};

enum vuart_hb_msg_type {
    B_RX_AVAIL,
    B_TX_EMPTY,
    H_RX_AVAIL,
    H_TX_EMPTY,
    VUART_MODE_SET,
    VUART_SEND_BREAK,
};

struct vuart_hb_msg {
    enum vuart_hb_msg_type type;
    bool host_rx_from_vuart;
    bool host_tx_to_vuart;
};

struct ls_vuart
{
    struct uart_port port;
    struct work_struct tx_work;
    struct mbox_client mbox_cl;
    struct mbox_chan *mbox_ch;
    struct host_vuart_fifo *mem;
    bool tx_enabled;
    bool rx_enabled;
};

static struct uart_driver ls_vuart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DEVICE_NAME,
	.dev_name	= "ttyVUART",
	.major		= 0,
	.minor		= 0,
	.nr		= LS_VUART_NR_UARTS,
};

static unsigned int ls_vuart_tx_empty(struct uart_port *port)
{
    struct ls_vuart *priv = container_of(port,struct ls_vuart,port);
    return lf_fifo_empty(&priv->mem->b2h) ? TIOCSER_TEMT : 0;
}

static unsigned int ls_vuart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void ls_vuart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{

}

static void ls_vuart_stop_tx(struct uart_port *port)
{
    struct ls_vuart *priv = container_of(port,struct ls_vuart,port);
    priv->tx_enabled = false;
}

static void ls_vuart_stop_rx(struct uart_port *port)
{
    struct ls_vuart *priv = container_of(port,struct ls_vuart,port);
    priv->rx_enabled = false;
}

static void ls_vuart_start_tx(struct uart_port *port)
{
    struct ls_vuart *priv = container_of(port,struct ls_vuart,port);
    priv->tx_enabled = true;
    schedule_work(&priv->tx_work);
}

static void ls_vuart_set_termios(struct uart_port *port,
				 struct ktermios *termios,
				 const struct ktermios *old)
{

}

static int ls_vuart_startup(struct uart_port *port)
{
    struct ls_vuart *priv = container_of(port,struct ls_vuart,port);
    priv->rx_enabled = true;
    return 0;
}

static void ls_vuart_shutdown(struct uart_port *port)
{
    struct ls_vuart *priv = container_of(port,struct ls_vuart,port);
    priv->rx_enabled = false;
}

static const char *ls_vuart_type(struct uart_port *port)
{
	return DEVICE_NAME;
}

static void ls_vuart_config_port(struct uart_port *port, int flags)
{

}

static const struct uart_ops ls_vuart_ops = {
	.tx_empty       = ls_vuart_tx_empty,
	.get_mctrl      = ls_vuart_get_mctrl,
	.set_mctrl      = ls_vuart_set_mctrl,
	.start_tx       = ls_vuart_start_tx,
	.stop_tx        = ls_vuart_stop_tx,
	.stop_rx        = ls_vuart_stop_rx,
	.startup        = ls_vuart_startup,
	.shutdown       = ls_vuart_shutdown,
	.set_termios    = ls_vuart_set_termios,
	.type           = ls_vuart_type,
	.config_port	= ls_vuart_config_port,
};

static void vuart_tx(struct ls_vuart *vuart) {
    u8 ch;
    struct uart_port *port = &vuart->port;
    while(!lf_fifo_full(&vuart->mem->b2h)) {
        if(uart_fifo_get(port,&ch)) {
            lf_fifo_data_put(&vuart->mem->b2h,vuart->mem->b2h_buf,&ch);
        }else {
            break;
        }
    }
    struct vuart_hb_msg msg = {
        .type = H_RX_AVAIL,
    };
    mbox_send_message(vuart->mbox_ch,&msg);
}

static void tx_work_handler(struct work_struct *work)
{
    struct ls_vuart *vuart = container_of(work,struct ls_vuart,tx_work);
    vuart_tx(vuart);
}

static void vuart_rx(struct ls_vuart *vuart) {
    u8 ch;
    struct uart_port *port = &vuart->port;
    while(lf_fifo_data_get(&vuart->mem->h2b,vuart->mem->h2b_buf,&ch))
    {
        if(vuart->rx_enabled)
        {
            port->icount.rx++;
            tty_insert_flip_char(&port->state->port, ch, TTY_NORMAL);
        }
    }
    tty_flip_buffer_push(&port->state->port);
    struct vuart_hb_msg msg = {
        .type = H_TX_EMPTY,
    };
    mbox_send_message(vuart->mbox_ch,&msg);
}

static void ls_vuart_mbox_rx(struct mbox_client *cl, void *mssg)
{
    struct vuart_hb_msg *msg = mssg;
    struct ls_vuart *vuart = container_of(cl,struct ls_vuart,mbox_cl);
    switch(msg->type) {
    case B_TX_EMPTY:
        schedule_work(&vuart->tx_work);
    break;
    case B_RX_AVAIL: 
        vuart_rx(vuart);
    break;
    default:

    break;
    }
}

static int ls_vuart_probe(struct platform_device *pdev)
{
    struct ls_vuart *vuart = devm_kzalloc(&pdev->dev,sizeof(*vuart),GFP_KERNEL);
    if(!vuart) {
        return -ENOMEM;
    }

    struct resource *res = platform_get_resource(pdev,IORESOURCE_MEM,0);
    if(!res) {
        dev_err(&pdev->dev,"Fail to get ls-mbox share memory\n");
        return -EINVAL;
    }
    vuart->mem = devm_ioremap_resource(&pdev->dev,res);
 	if (IS_ERR(vuart->mem)) {
        dev_err(&pdev->dev,"Fail to remap vuart mem\n");
 		return PTR_ERR(vuart->mem);
    }

    int id = of_alias_get_id(pdev->dev.of_node, "vuart");
	if (id < 0 || id >= LS_VUART_NR_UARTS) {
		dev_err(&pdev->dev, "missing aliases entry\n");
		return -ENODEV;
	}
    platform_set_drvdata(pdev,vuart);

    vuart->mbox_cl.dev = &pdev->dev;
    vuart->mbox_cl.rx_callback = ls_vuart_mbox_rx;
    vuart->mbox_ch = mbox_request_channel(&vuart->mbox_cl,0);
    if(IS_ERR(vuart->mbox_ch)) {
		dev_err(&pdev->dev, "mbox request fail, %ld\n",PTR_ERR(vuart->mbox_ch));
        return -ENODEV;
    }

    vuart->port.dev = &pdev->dev;
    vuart->port.line = id;
    vuart->port.iotype = UPIO_MEM;
    vuart->port.ops = &ls_vuart_ops;
    vuart->port.flags = UPF_BOOT_AUTOCONF;
    vuart->port.type = PORT_UNKNOWN;
    int ret = uart_add_one_port(&ls_vuart_driver,&vuart->port);
    if(ret) {
        dev_err(&pdev->dev,"ls vuart add port fail\n");
        mbox_free_channel(vuart->mbox_ch);
        return ret;
    }

    INIT_WORK(&vuart->tx_work,tx_work_handler);
    return 0;
}

static void ls_vuart_remove(struct platform_device *pdev)
{
    struct ls_vuart *vuart = platform_get_drvdata(pdev);
    cancel_work_sync(&vuart->tx_work);
    mbox_free_channel(vuart->mbox_ch);
    uart_remove_one_port(&ls_vuart_driver, &vuart->port);
}

static const struct of_device_id ls_vuart_of_match[] = {
	{ .compatible = "linkedsemi,ls-vuart" },
};

static struct platform_driver ls_vuart_platform_driver = {
	.probe		= ls_vuart_probe,
	.remove		= ls_vuart_remove,
	.driver		= {
		.name	= LS_VUART_NAME,
		.of_match_table = ls_vuart_of_match,
	},
};

static int __init ls_vuart_init(void)
{
    int ret;
    ret = uart_register_driver(&ls_vuart_driver);
    if(ret) {
        return ret;
    }
	ret = platform_driver_register(&ls_vuart_platform_driver);
    if(ret)
        uart_unregister_driver(&ls_vuart_driver);
    return ret;
}

static void __exit ls_vuart_exit(void)
{
	platform_driver_unregister(&ls_vuart_platform_driver);
	uart_unregister_driver(&ls_vuart_driver);
}

module_init(ls_vuart_init);
module_exit(ls_vuart_exit);
MODULE_DESCRIPTION("LS VUART DRIVER");
MODULE_LICENSE("GPL");