// SPDX-License-Identifier: GPL-2.0-or-later
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/math64.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/property.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#define LINKEDSEMI_SGPIO_CTRL        0x140

#define LINKEDSEMI_SGPIO_CLK_DIV_MASK    GENMASK(31, 16)
#define LINKEDSEMI_SGPIO_ENABLE        BIT(0)
#define LINKEDSEMI_SGPIO_PINS_SHIFT        5
#define LINKEDSEMI_SGPIO_PINS_MASK         GENMASK(10, 5)
#define LINKEDSEMI_SGPIO_PUSH_PULL         BIT(1)
#define LINKEDSEMI_SGPIO_CONFIG_MASK       (LINKEDSEMI_SGPIO_CLK_DIV_MASK | \
                                      LINKEDSEMI_SGPIO_PINS_MASK | \
                                      LINKEDSEMI_SGPIO_PUSH_PULL | \
                                      LINKEDSEMI_SGPIO_ENABLE)
#define LINKEDSEMI_SGPIO_MAX_PINS          256
#define LINKEDSEMI_SGPIO_MAX_FREQUENCY     25000000U
#define LINKEDSEMI_SGPIO_RESOURCE_SIZE     0x184
#define LINKEDSEMI_SGPIO_CONFIG_TIMEOUT_US 10000

struct linkedsemi_sgpio_pdata {
    const u32 pin_mask;
};

struct linkedsemi_sgpio {
    struct gpio_chip chip;
    struct device *dev;
    struct clk *pclk;
    raw_spinlock_t lock;
    void __iomem *base;
    unsigned int irq;
    unsigned int nr_gpios;
    unsigned int nr_banks;
    u32 config;
    u32 input_hz;
    u32 output_shadow[8];
};

struct linkedsemi_sgpio_bank {
    u16    val_regs;
    u16    rdata_reg;
    u16    irq_regs;
    u16    status_regs;
    u16    tolerance_regs;
    // const char  names[4][3];
};

/*
 * Note: The "value" register returns the input value when the GPIO is
 *     configured as an input.
 *
 *     The "rdata" register returns the output value when the GPIO is
 *     configured as an output.
 */
static const struct linkedsemi_sgpio_bank linkedsemi_sgpio_banks[] = {
    {
        .val_regs = 0x0000,
        .rdata_reg = 0x00164,
        .irq_regs = 0x0004,
        .status_regs = 0x0014,
        .tolerance_regs = 0x0018,
        // .names = { "A", "B", "C", "D" },
    },
    {
        .val_regs = 0x001c,
        .rdata_reg = 0x00168,
        .irq_regs = 0x0020,
        .status_regs = 0x0030,
        .tolerance_regs = 0x0034,
        // .names = { "E", "F", "G", "H" },
    },
    {
        .val_regs = 0x0038,
        .rdata_reg = 0x0016c,
        .irq_regs = 0x003c,
        .status_regs = 0x004c,
        .tolerance_regs = 0x0050,
        // .names = { "I", "J", "K", "L" },
    },
    {
        .val_regs = 0x0054,
        .rdata_reg = 0x00170,
        .irq_regs = 0x0058,
        .status_regs = 0x0068,
        .tolerance_regs = 0x006c,
        // .names = { "M", "N", "O", "P" },
    },
    {
        .val_regs = 0x0070,
        .rdata_reg = 0x00174,
        .irq_regs = 0x0074,
        .status_regs = 0x0084,
        .tolerance_regs = 0x0088,
    },
    {
        .val_regs = 0x008c,
        .rdata_reg = 0x00178,
        .irq_regs = 0x0090,
        .status_regs = 0x0100,
        .tolerance_regs = 0x0104,
    },
    {
        .val_regs = 0x0108,
        .rdata_reg = 0x0017c,
        .irq_regs = 0x010c,
        .status_regs = 0x011c,
        .tolerance_regs = 0x0120,
    },
    {
        .val_regs = 0x0124,
        .rdata_reg = 0x00180,
        .irq_regs = 0x0128,
        .status_regs = 0x0138,
        .tolerance_regs = 0x013c,
    },
};

enum linkedsemi_sgpio_reg {
    reg_val,
    reg_rdata,
    reg_irq_enable,
    reg_irq_type0,
    reg_irq_type1,
    reg_irq_type2,
    reg_irq_status,
    reg_tolerance,
};

#define GPIO_VAL_VALUE      0x00
#define GPIO_IRQ_ENABLE     0x00
#define GPIO_IRQ_TYPE0      0x04
#define GPIO_IRQ_TYPE1      0x08
#define GPIO_IRQ_TYPE2      0x0C

static void __iomem *bank_reg(struct linkedsemi_sgpio *gpio,
                     const struct linkedsemi_sgpio_bank *bank,
                     const enum linkedsemi_sgpio_reg reg)
{
    switch (reg) {
    case reg_val:
        return gpio->base + bank->val_regs + GPIO_VAL_VALUE;
    case reg_rdata:
        return gpio->base + bank->rdata_reg;
    case reg_irq_enable:
        return gpio->base + bank->irq_regs + GPIO_IRQ_ENABLE;
    case reg_irq_type0:
        return gpio->base + bank->irq_regs + GPIO_IRQ_TYPE0;
    case reg_irq_type1:
        return gpio->base + bank->irq_regs + GPIO_IRQ_TYPE1;
    case reg_irq_type2:
        return gpio->base + bank->irq_regs + GPIO_IRQ_TYPE2;
    case reg_irq_status:
        /* SGPIO6 STATUS crosses the 0xa0..0xff register hole. */
        return gpio->base + bank->status_regs;
    case reg_tolerance:
        return gpio->base + bank->tolerance_regs;
    default:
        /* acturally if code runs to here, it's an error case */
        BUG();
    }
}

#define GPIO_BANK(x)    ((x) >> 6)
#define GPIO_OFFSET(x)  ((x) & GENMASK(5, 0))
#define GPIO_BIT(x)     BIT(GPIO_OFFSET(x) >> 1)

static const struct linkedsemi_sgpio_bank *to_bank(unsigned int offset)
{
    unsigned int bank;

    bank = GPIO_BANK(offset);

    WARN_ON(bank >= ARRAY_SIZE(linkedsemi_sgpio_banks));
    return &linkedsemi_sgpio_banks[bank];
}

static int linkedsemi_sgpio_init_valid_mask(struct gpio_chip *gc,
        unsigned long *valid_mask, unsigned int ngpios)
{
    bitmap_set(valid_mask, 0, ngpios);
    return 0;
}

static void linkedsemi_sgpio_irq_init_valid_mask(struct gpio_chip *gc,
        unsigned long *valid_mask, unsigned int ngpios)
{
    unsigned int i;

    /* input GPIOs are even bits */
    for (i = 0; i < ngpios; i++) {
        if (i % 2)
            clear_bit(i, valid_mask);
    }
}

static bool linkedsemi_sgpio_is_input(unsigned int offset)
{
    return !(offset % 2);
}

static int linkedsemi_sgpio_get(struct gpio_chip *gc, unsigned int offset)
{
    struct linkedsemi_sgpio *gpio = gpiochip_get_data(gc);
    const struct linkedsemi_sgpio_bank *bank = to_bank(offset);
    unsigned long flags;
    enum linkedsemi_sgpio_reg reg;
    int rc = 0;

    raw_spin_lock_irqsave(&gpio->lock, flags);

    reg = linkedsemi_sgpio_is_input(offset) ? reg_val : reg_rdata;
    rc = !!(ioread32(bank_reg(gpio, bank, reg)) & GPIO_BIT(offset));

    raw_spin_unlock_irqrestore(&gpio->lock, flags);

    return rc;
}

static int sgpio_set_value(struct gpio_chip *gc, unsigned int offset, int val)
{
    struct linkedsemi_sgpio *gpio = gpiochip_get_data(gc);
    const struct linkedsemi_sgpio_bank *bank = to_bank(offset);
    void __iomem *addr_w;
    u32 reg;

    if (linkedsemi_sgpio_is_input(offset))
        return -EINVAL;

    /* READ_DATA can lag by a frame; preserve pending writes in a shadow. */
    addr_w = bank_reg(gpio, bank, reg_val);
    reg = gpio->output_shadow[GPIO_BANK(offset)];

    if (val)
        reg |= GPIO_BIT(offset);
    else
        reg &= ~GPIO_BIT(offset);

    iowrite32(reg, addr_w);
    gpio->output_shadow[GPIO_BANK(offset)] = reg;

    return 0;
}

static int linkedsemi_sgpio_set(struct gpio_chip *gc, unsigned int offset, int val)
{
    struct linkedsemi_sgpio *gpio = gpiochip_get_data(gc);
    unsigned long flags;
    int rc;

    raw_spin_lock_irqsave(&gpio->lock, flags);
    rc = sgpio_set_value(gc, offset, val);
    raw_spin_unlock_irqrestore(&gpio->lock, flags);

    return rc;
}


static int linkedsemi_sgpio_set_multiple(struct gpio_chip *gc, unsigned long *mask,
                                   unsigned long *bits)
{
    struct linkedsemi_sgpio *gpio = gpiochip_get_data(gc);
    u32 bank_mask[ARRAY_SIZE(linkedsemi_sgpio_banks)] = { 0 };
    u32 bank_bits[ARRAY_SIZE(linkedsemi_sgpio_banks)] = { 0 };
    unsigned int offset, i;
    unsigned long flags;
    u32 value;

    /* Validate the complete request before writing any bank. */
    for_each_set_bit(offset, mask, gc->ngpio) {
        if (linkedsemi_sgpio_is_input(offset))
            return -EINVAL;
        bank_mask[GPIO_BANK(offset)] |= GPIO_BIT(offset);
        if (test_bit(offset, bits))
            bank_bits[GPIO_BANK(offset)] |= GPIO_BIT(offset);
    }

    raw_spin_lock_irqsave(&gpio->lock, flags);
    for (i = 0; i < gpio->nr_banks; i++) {
        if (!bank_mask[i])
            continue;
        value = (gpio->output_shadow[i] & ~bank_mask[i]) | bank_bits[i];
        iowrite32(value, bank_reg(gpio, &linkedsemi_sgpio_banks[i], reg_val));
        gpio->output_shadow[i] = value;
    }
    raw_spin_unlock_irqrestore(&gpio->lock, flags);

    return 0;
}

static int linkedsemi_sgpio_get_multiple(struct gpio_chip *gc, unsigned long *mask,
                                   unsigned long *bits)
{
    struct linkedsemi_sgpio *gpio = gpiochip_get_data(gc);
    u32 inputs[ARRAY_SIZE(linkedsemi_sgpio_banks)];
    u32 outputs[ARRAY_SIZE(linkedsemi_sgpio_banks)];
    unsigned long have_inputs = 0, have_outputs = 0;
    unsigned long flags;
    unsigned int offset, i;
    u32 value;

    raw_spin_lock_irqsave(&gpio->lock, flags);
    for_each_set_bit(offset, mask, gc->ngpio) {
        i = GPIO_BANK(offset);
        if (linkedsemi_sgpio_is_input(offset)) {
            if (!(have_inputs & BIT(i))) {
                inputs[i] = ioread32(bank_reg(gpio,
                                      &linkedsemi_sgpio_banks[i], reg_val));
                have_inputs |= BIT(i);
            }
            value = inputs[i];
        } else {
            if (!(have_outputs & BIT(i))) {
                outputs[i] = ioread32(bank_reg(gpio,
                                       &linkedsemi_sgpio_banks[i], reg_rdata));
                have_outputs |= BIT(i);
            }
            value = outputs[i];
        }
        __assign_bit(offset, bits, value & GPIO_BIT(offset));
    }
    raw_spin_unlock_irqrestore(&gpio->lock, flags);

    return 0;
}

static int linkedsemi_sgpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
    return linkedsemi_sgpio_is_input(offset) ? 0 : -EINVAL;
}

static int linkedsemi_sgpio_dir_out(struct gpio_chip *gc, unsigned int offset, int val)
{
    struct linkedsemi_sgpio *gpio = gpiochip_get_data(gc);
    unsigned long flags;
    int rc;

    /* No special action is required for setting the direction; we'll
     * error-out in sgpio_set_value if this isn't an output GPIO */

    raw_spin_lock_irqsave(&gpio->lock, flags);
    rc = sgpio_set_value(gc, offset, val);
    raw_spin_unlock_irqrestore(&gpio->lock, flags);

    return rc;
}

static int linkedsemi_sgpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
    return !!linkedsemi_sgpio_is_input(offset);
}

static void irqd_to_linkedsemi_sgpio_data(struct irq_data *d,
                    struct linkedsemi_sgpio **gpio,
                    const struct linkedsemi_sgpio_bank **bank,
                    u32 *bit, int *offset)
{
    struct linkedsemi_sgpio *internal;

    *offset = irqd_to_hwirq(d);
    internal = gpiochip_get_data(irq_data_get_irq_chip_data(d));
    WARN_ON(!internal);

    *gpio = internal;
    *bank = to_bank(*offset);
    *bit = GPIO_BIT(*offset);
}

static void linkedsemi_sgpio_irq_ack(struct irq_data *d)
{
    const struct linkedsemi_sgpio_bank *bank;
    struct linkedsemi_sgpio *gpio;
    unsigned long flags;
    void __iomem *status_addr;
    int offset;
    u32 bit;

    irqd_to_linkedsemi_sgpio_data(d, &gpio, &bank, &bit, &offset);

    status_addr = bank_reg(gpio, bank, reg_irq_status);

    raw_spin_lock_irqsave(&gpio->lock, flags);

    iowrite32(bit, status_addr);

    raw_spin_unlock_irqrestore(&gpio->lock, flags);
}

static void linkedsemi_sgpio_irq_set_mask(struct irq_data *d, bool set)
{
    const struct linkedsemi_sgpio_bank *bank;
    struct linkedsemi_sgpio *gpio;
    unsigned long flags;
    u32 reg, bit;
    void __iomem *addr;
    int offset;

    irqd_to_linkedsemi_sgpio_data(d, &gpio, &bank, &bit, &offset);
    addr = bank_reg(gpio, bank, reg_irq_enable);

    /* Unmasking the IRQ */
    if (set)
        gpiochip_enable_irq(&gpio->chip, irqd_to_hwirq(d));

    raw_spin_lock_irqsave(&gpio->lock, flags);

    reg = ioread32(addr);
    if (set)
        reg |= bit;
    else
        reg &= ~bit;

    iowrite32(reg, addr);

    raw_spin_unlock_irqrestore(&gpio->lock, flags);

    /* Masking the IRQ */
    if (!set)
        gpiochip_disable_irq(&gpio->chip, irqd_to_hwirq(d));


}

static void linkedsemi_sgpio_irq_mask(struct irq_data *d)
{
    linkedsemi_sgpio_irq_set_mask(d, false);
}

static void linkedsemi_sgpio_irq_unmask(struct irq_data *d)
{
    linkedsemi_sgpio_irq_set_mask(d, true);
}

static int linkedsemi_sgpio_set_type(struct irq_data *d, unsigned int type)
{
    u32 type0 = 0;
    u32 type1 = 0;
    u32 type2 = 0;
    u32 bit, reg, enabled;
    const struct linkedsemi_sgpio_bank *bank;
    irq_flow_handler_t handler;
    struct linkedsemi_sgpio *gpio;
    unsigned long flags;
    void __iomem *addr;
    int offset;

    irqd_to_linkedsemi_sgpio_data(d, &gpio, &bank, &bit, &offset);
    if (!linkedsemi_sgpio_is_input(offset))
        return -EINVAL;

    switch (type & IRQ_TYPE_SENSE_MASK) {
    case IRQ_TYPE_EDGE_BOTH:
        type2 |= bit;
        handler = handle_edge_irq;
        break;
    case IRQ_TYPE_EDGE_RISING:
        type0 |= bit;
        fallthrough;
    case IRQ_TYPE_EDGE_FALLING:
        handler = handle_edge_irq;
        break;
    case IRQ_TYPE_LEVEL_HIGH:
        type0 |= bit;
        fallthrough;
    case IRQ_TYPE_LEVEL_LOW:
        type1 |= bit;
        handler = handle_level_irq;
        break;
    default:
        return -EINVAL;
    }

    raw_spin_lock_irqsave(&gpio->lock, flags);

    addr = bank_reg(gpio, bank, reg_irq_enable);
    enabled = ioread32(addr);
    iowrite32(enabled & ~bit, addr);

    addr = bank_reg(gpio, bank, reg_irq_type0);
    reg = ioread32(addr);
    reg = (reg & ~bit) | type0;
    iowrite32(reg, addr);

    addr = bank_reg(gpio, bank, reg_irq_type1);
    reg = ioread32(addr);
    reg = (reg & ~bit) | type1;
    iowrite32(reg, addr);

    addr = bank_reg(gpio, bank, reg_irq_type2);
    reg = ioread32(addr);
    reg = (reg & ~bit) | type2;
    iowrite32(reg, addr);

    /* W1C clears only this pin's stale event before restoring its mask. */
    iowrite32(bit, bank_reg(gpio, bank, reg_irq_status));
    iowrite32(enabled, bank_reg(gpio, bank, reg_irq_enable));

    raw_spin_unlock_irqrestore(&gpio->lock, flags);

    irq_set_handler_locked(d, handler);

    return 0;
}

static void linkedsemi_sgpio_irq_handler(struct irq_desc *desc)
{
    struct gpio_chip *gc = irq_desc_get_handler_data(desc);
    struct irq_chip *ic = irq_desc_get_chip(desc);
    struct linkedsemi_sgpio *data = gpiochip_get_data(gc);
    unsigned int i, p, pins;
    unsigned long reg;

    chained_irq_enter(ic, desc);

    for (i = 0; i < data->nr_banks; i++) {
        const struct linkedsemi_sgpio_bank *bank = &linkedsemi_sgpio_banks[i];

        pins = min_t(unsigned int, 32, data->nr_gpios - i * 32);
        reg = ioread32(bank_reg(data, bank, reg_irq_status)) &
              ioread32(bank_reg(data, bank, reg_irq_enable)) &
              GENMASK(pins - 1, 0);

        for_each_set_bit(p, &reg, pins)
            generic_handle_domain_irq(gc->irq.domain, (i * 32 + p) * 2);
    }

    chained_irq_exit(ic, desc);
}

static void linkedsemi_sgpio_irq_print_chip(struct irq_data *d, struct seq_file *p)
{
    const struct linkedsemi_sgpio_bank *bank;
    struct linkedsemi_sgpio *gpio;
    u32 bit;
    int offset;

    irqd_to_linkedsemi_sgpio_data(d, &gpio, &bank, &bit, &offset);
    seq_puts(p, dev_name(gpio->dev));
}

static const struct irq_chip linkedsemi_sgpio_irq_chip = {
    .name = "linkedsemi-sgpio",
    .irq_ack = linkedsemi_sgpio_irq_ack,
    .irq_mask = linkedsemi_sgpio_irq_mask,
    .irq_unmask = linkedsemi_sgpio_irq_unmask,
    .irq_set_type = linkedsemi_sgpio_set_type,
    .irq_print_chip = linkedsemi_sgpio_irq_print_chip,
    .flags = IRQCHIP_IMMUTABLE,
    GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int linkedsemi_sgpio_setup_irqs(struct linkedsemi_sgpio *gpio,
                   struct platform_device *pdev)
{
    int rc, i;
    const struct linkedsemi_sgpio_bank *bank;
    struct gpio_irq_chip *irq;
    rc = platform_get_irq(pdev, 0);
    if (rc < 0)
        return rc;

    gpio->irq = rc;

    /* Disable IRQ and clear Interrupt status registers for all SGPIO Pins. */
    for (i = 0; i < ARRAY_SIZE(linkedsemi_sgpio_banks); i++) {
        bank =  &linkedsemi_sgpio_banks[i];
        /* disable irq enable bits */
        iowrite32(0x00000000, bank_reg(gpio, bank, reg_irq_enable));
        /* clear status bits */
        iowrite32(0xffffffff, bank_reg(gpio, bank, reg_irq_status));
    }
    irq = &gpio->chip.irq;
    gpio_irq_chip_set_chip(irq, &linkedsemi_sgpio_irq_chip);
    irq->init_valid_mask = linkedsemi_sgpio_irq_init_valid_mask;
    irq->handler = handle_bad_irq;
    irq->default_type = IRQ_TYPE_NONE;
    irq->parent_handler = linkedsemi_sgpio_irq_handler;
    irq->parent_handler_data = &gpio->chip;
    irq->parents = &gpio->irq;
    irq->num_parents = 1;
    /* Apply default IRQ settings */
    for (i = 0; i < ARRAY_SIZE(linkedsemi_sgpio_banks); i++) {
        bank = &linkedsemi_sgpio_banks[i];
        /* set falling or level-low irq */
        iowrite32(0x00000000, bank_reg(gpio, bank, reg_irq_type0));
        /* trigger type is edge */
        iowrite32(0x00000000, bank_reg(gpio, bank, reg_irq_type1));
        /* single edge trigger */
        iowrite32(0x00000000, bank_reg(gpio, bank, reg_irq_type2));
        iowrite32(0x00000000, bank_reg(gpio, bank, reg_val));
        gpio->output_shadow[i] = 0;
    }
    return 0;
}

static int linkedsemi_sgpio_reset_tolerance(struct gpio_chip *chip,
                    unsigned int offset, bool enable)
{
    struct linkedsemi_sgpio *gpio = gpiochip_get_data(chip);
    unsigned long flags;
    void __iomem *reg;
    u32 val;

    reg = bank_reg(gpio, to_bank(offset), reg_tolerance);

    raw_spin_lock_irqsave(&gpio->lock, flags);

    val = readl(reg);

    if (enable)
        val |= GPIO_BIT(offset);
    else
        val &= ~GPIO_BIT(offset);

    writel(val, reg);

    raw_spin_unlock_irqrestore(&gpio->lock, flags);

    return 0;
}

static int linkedsemi_sgpio_set_config(struct gpio_chip *chip, unsigned int offset,
                   unsigned long config)
{
    unsigned long param = pinconf_to_config_param(config);
    u32 arg = pinconf_to_config_argument(config);

    /* Input requests also ask gpiolib for this optional output setting. */
    if (param == PIN_CONFIG_PERSIST_STATE && !linkedsemi_sgpio_is_input(offset))
        return linkedsemi_sgpio_reset_tolerance(chip, offset, arg);

    return -ENOTSUPP;
}

static const struct linkedsemi_sgpio_pdata raptor_sgpiom_pdata = {
    .pin_mask = GENMASK(10, 5),
};

static const struct of_device_id linkedsemi_sgpio_of_table[] = {
    { .compatible = "linkedsemi,sgpiom", .data = &raptor_sgpiom_pdata, },
    {}
};

MODULE_DEVICE_TABLE(of, linkedsemi_sgpio_of_table);


static int linkedsemi_sgpio_validate_config(u32 config, u32 nr_gpios, u32 input_hz)
{
    u32 divisor = 2 * (FIELD_GET(LINKEDSEMI_SGPIO_CLK_DIV_MASK, config) + 1);

    if (!nr_gpios || nr_gpios > LINKEDSEMI_SGPIO_MAX_PINS || nr_gpios % 8 ||
        !(config & LINKEDSEMI_SGPIO_ENABLE) || (config & ~LINKEDSEMI_SGPIO_CONFIG_MASK) ||
        FIELD_GET(LINKEDSEMI_SGPIO_PINS_MASK, config) != nr_gpios / 8)
        return -EINVAL;
    if (input_hz && (u64)input_hz >
        (u64)LINKEDSEMI_SGPIO_MAX_FREQUENCY * divisor)
        return -ERANGE;
    return 0;
}

static int linkedsemi_sgpio_calc_div(u32 input_hz, u32 frequency, u32 *divider)
{
    u64 divisor;

    if (!input_hz || !frequency || frequency > LINKEDSEMI_SGPIO_MAX_FREQUENCY)
        return -EINVAL;
    /* Round up the divisor so the actual rate never exceeds the request. */
    divisor = DIV_ROUND_UP_ULL((u64)input_hz, 2ULL * frequency);
    if (!divisor || divisor > 65536)
        return -ERANGE;
    *divider = divisor - 1;
    return 0;
}

static int linkedsemi_sgpio_write_config(struct linkedsemi_sgpio *gpio, u32 config,
                                   u32 *readback)
{
    void __iomem *addr = gpio->base + LINKEDSEMI_SGPIO_CTRL;

    iowrite32(config, addr);
    return readl_poll_timeout(addr, *readback,
                             (*readback & LINKEDSEMI_SGPIO_CONFIG_MASK) == config,
                             10, LINKEDSEMI_SGPIO_CONFIG_TIMEOUT_US);
}

static void linkedsemi_sgpio_stop(void *data)
{
    struct linkedsemi_sgpio *gpio = data;
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(linkedsemi_sgpio_banks); i++)
        iowrite32(0, bank_reg(gpio, &linkedsemi_sgpio_banks[i], reg_irq_enable));
    iowrite32(gpio->config & ~LINKEDSEMI_SGPIO_ENABLE,
              gpio->base + LINKEDSEMI_SGPIO_CTRL);
}

static ssize_t linkedsemi_sgpio_registers_show(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
    struct linkedsemi_sgpio *gpio = dev_get_drvdata(dev);
    unsigned long flags;
    unsigned int i;
    ssize_t len;

    raw_spin_lock_irqsave(&gpio->lock, flags);
    len = sysfs_emit(buf,
                    "CONFG=%#010x serial_bits=%u input_hz=%u layout=linkedsemi-fpga config_offset=%#x\n",
                    ioread32(gpio->base + LINKEDSEMI_SGPIO_CTRL), gpio->nr_gpios,
                    gpio->input_hz, LINKEDSEMI_SGPIO_CTRL);
    for (i = 0; i < gpio->nr_banks; i++) {
        const struct linkedsemi_sgpio_bank *bank = &linkedsemi_sgpio_banks[i];

        len += sysfs_emit_at(buf, len,
                            "bank%u IN=%#010x READ_DATA=%#010x SHADOW=%#010x EN=%#010x TYPE=%#010x/%#010x/%#010x STATUS=%#010x\n",
                            i, ioread32(bank_reg(gpio, bank, reg_val)),
                            ioread32(bank_reg(gpio, bank, reg_rdata)),
                            gpio->output_shadow[i],
                            ioread32(bank_reg(gpio, bank, reg_irq_enable)),
                            ioread32(bank_reg(gpio, bank, reg_irq_type0)),
                            ioread32(bank_reg(gpio, bank, reg_irq_type1)),
                            ioread32(bank_reg(gpio, bank, reg_irq_type2)),
                            ioread32(bank_reg(gpio, bank, reg_irq_status)));
    }
    raw_spin_unlock_irqrestore(&gpio->lock, flags);
    return len;
}
static DEVICE_ATTR(registers, 0444, linkedsemi_sgpio_registers_show, NULL);

static struct attribute *linkedsemi_sgpio_attrs[] = {
    &dev_attr_registers.attr,
    NULL,
};
ATTRIBUTE_GROUPS(linkedsemi_sgpio);

static int linkedsemi_sgpio_probe(struct platform_device *pdev)
{
    u32 nr_gpios, sgpio_freq, sgpio_clk_div, gpio_cnt_regval, pin_mask;
    const struct linkedsemi_sgpio_pdata *pdata;
    struct linkedsemi_sgpio *gpio;
    struct resource *resource;
    unsigned long apb_freq;
    u32 config, readback;
    u32 input_hz;
    int rc;
    gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
    if (!gpio)
        return -ENOMEM;
    resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!resource || resource_size(resource) < LINKEDSEMI_SGPIO_RESOURCE_SIZE)
        return dev_err_probe(&pdev->dev, -EINVAL, "Register window too small\n");
    gpio->base = devm_ioremap_resource(&pdev->dev, resource);
    if (IS_ERR(gpio->base))
        return PTR_ERR(gpio->base);

    gpio->dev = &pdev->dev;
    pdata = device_get_match_data(&pdev->dev);
    if (!pdata)
        return -EINVAL;
    pin_mask = pdata->pin_mask;

    rc = device_property_read_u32(&pdev->dev, "ngpios", &nr_gpios);
    if (rc < 0) {
        dev_err(&pdev->dev, "Could not read ngpios property\n");
        return -EINVAL;
    } else if (!nr_gpios || nr_gpios > LINKEDSEMI_SGPIO_MAX_PINS || nr_gpios % 8) {
        dev_err(&pdev->dev, "ngpios must be 8..256 in multiples of 8: %u\n",
            nr_gpios);
        return -EINVAL;
    }
    gpio->pclk = devm_clk_get_enabled(&pdev->dev, NULL);
    if (IS_ERR(gpio->pclk)) {
        dev_err(&pdev->dev, "devm_clk_get failed\n");
        return PTR_ERR(gpio->pclk);
    }
    apb_freq = clk_get_rate(gpio->pclk);
    if (device_property_present(&pdev->dev, "clock-frequency")) {
        rc = device_property_read_u32(&pdev->dev, "clock-frequency", &input_hz);
        if (rc || !input_hz)
            return dev_err_probe(&pdev->dev, -EINVAL, "Invalid core clock\n");
    } else {
        if (!apb_freq || apb_freq > U32_MAX)
            return dev_err_probe(&pdev->dev, -EINVAL, "Invalid input clock\n");
        input_hz = apb_freq;
    }

    /*
     * From the datasheet,
     *    SGPIO period = 1/PCLK * 2 * (GPIO254[31:16] + 1)
     *    period = 2 * (GPIO254[31:16] + 1) / PCLK
     *    frequency = 1 / (2 * (GPIO254[31:16] + 1) / PCLK)
     *    frequency = PCLK / (2 * (GPIO254[31:16] + 1))
     *    frequency * 2 * (GPIO254[31:16] + 1) = PCLK
     *    GPIO254[31:16] = PCLK / (frequency * 2) - 1
     */
    rc = device_property_read_u32(&pdev->dev, "bus-frequency", &sgpio_freq);
    if (rc)
        return dev_err_probe(&pdev->dev, rc, "Missing bus-frequency\n");
    rc = linkedsemi_sgpio_calc_div(input_hz, sgpio_freq, &sgpio_clk_div);
    if (rc)
        return dev_err_probe(&pdev->dev, rc, "Invalid bus-frequency\n");
    gpio_cnt_regval = ((nr_gpios / 8) << LINKEDSEMI_SGPIO_PINS_SHIFT) & pin_mask;
    config = FIELD_PREP(LINKEDSEMI_SGPIO_CLK_DIV_MASK, sgpio_clk_div) |
             gpio_cnt_regval | LINKEDSEMI_SGPIO_ENABLE;
    /* Preserve the original open-drain default; opt into DO push-pull. */
    if (device_property_read_bool(&pdev->dev, "drive-push-pull"))
        config |= LINKEDSEMI_SGPIO_PUSH_PULL;
    rc = linkedsemi_sgpio_validate_config(config, nr_gpios, input_hz);
    if (rc)
        return dev_err_probe(&pdev->dev, rc, "Invalid CONFG or frame length\n");

    gpio->nr_gpios = nr_gpios;
    gpio->nr_banks = DIV_ROUND_UP(nr_gpios, 32);
    gpio->config = config;
    gpio->input_hz = input_hz;

    raw_spin_lock_init(&gpio->lock);
    platform_set_drvdata(pdev, gpio);
    rc = linkedsemi_sgpio_write_config(gpio, config & ~LINKEDSEMI_SGPIO_ENABLE, &readback);
    if (rc) {
        iowrite32(0, gpio->base + LINKEDSEMI_SGPIO_CTRL);
        return dev_err_probe(&pdev->dev, rc, "CONFG readback %#010x\n", readback);
    }
    rc = devm_add_action_or_reset(&pdev->dev, linkedsemi_sgpio_stop, gpio);
    if (rc)
        return rc;

    gpio->chip.owner = THIS_MODULE;
    gpio->chip.parent = &pdev->dev;
    gpio->chip.ngpio = nr_gpios * 2;
    gpio->chip.init_valid_mask = linkedsemi_sgpio_init_valid_mask;
    gpio->chip.direction_input = linkedsemi_sgpio_dir_in;
    gpio->chip.direction_output = linkedsemi_sgpio_dir_out;
    gpio->chip.get_direction = linkedsemi_sgpio_get_direction;
    gpio->chip.request = NULL;
    gpio->chip.free = NULL;
    gpio->chip.get = linkedsemi_sgpio_get;
    gpio->chip.get_multiple = linkedsemi_sgpio_get_multiple;
    gpio->chip.set = linkedsemi_sgpio_set;
    gpio->chip.set_multiple = linkedsemi_sgpio_set_multiple;
    gpio->chip.set_config = linkedsemi_sgpio_set_config;
    gpio->chip.label = dev_name(&pdev->dev);
    gpio->chip.base = -1;
    rc = linkedsemi_sgpio_setup_irqs(gpio, pdev);
    if (rc)
        return dev_err_probe(&pdev->dev, rc, "Cannot set up IRQ\n");
    rc = linkedsemi_sgpio_write_config(gpio, config, &readback);
    if (rc)
        return dev_err_probe(&pdev->dev, rc, "CONFG readback %#010x\n", readback);

    rc = devm_gpiochip_add_data(&pdev->dev, &gpio->chip, gpio);
    if (rc < 0)
        return rc;
    dev_info(&pdev->dev,
             "%u inputs + %u outputs, input=%u Hz, calculated SCLK=%u Hz, CONFG=%#010x\n",
             nr_gpios, nr_gpios, input_hz,
             input_hz / (u32)(2 * (FIELD_GET(LINKEDSEMI_SGPIO_CLK_DIV_MASK, config) + 1)),
             config);
    return 0;
}

static struct platform_driver linkedsemi_sgpio_driver = {
    .probe = linkedsemi_sgpio_probe,
    .driver = {
        .name = KBUILD_MODNAME,
        .of_match_table = linkedsemi_sgpio_of_table,
        .dev_groups = linkedsemi_sgpio_groups,
    },
};

module_platform_driver(linkedsemi_sgpio_driver);
MODULE_DESCRIPTION("LinkedSemi Serial GPIO Master Driver");
MODULE_LICENSE("GPL");
