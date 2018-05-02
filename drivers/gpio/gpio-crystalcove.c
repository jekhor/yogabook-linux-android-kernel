/*
 * gpio-crystalcove.c - Intel Crystal Cove GPIO Driver
 *
 * Copyright (C) 2012, 2014 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Yang, Bin <bin.yang@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/gpio.h>

static struct pmic_gpio gpio_info;

static void __crystalcove_irq_mask(int gpio, int mask)
{
	u8 mirqs0 = gpio < 8 ? MGPIO0IRQS0 : MGPIO1IRQS0;
	int offset = gpio < 8 ? gpio : gpio - 8;

	if (mask)
		intel_soc_pmic_setb(mirqs0, 1 << offset);
	else
		intel_soc_pmic_clearb(mirqs0, 1 << offset);
}

static void __crystalcove_irq_type(int gpio, int type)
{
	u8 ctli = gpio < 8 ? GPIO0P0CTLI + gpio : GPIO1P0CTLI + (gpio - 8);

	type &= IRQ_TYPE_EDGE_BOTH;
	intel_soc_pmic_clearb(ctli, CTLI_INTCNT_BE);

	if (type == IRQ_TYPE_EDGE_BOTH)
		intel_soc_pmic_setb(ctli, CTLI_INTCNT_BE);
	else if (type == IRQ_TYPE_EDGE_RISING)
		intel_soc_pmic_setb(ctli, CTLI_INTCNT_PE);
	else if (type & IRQ_TYPE_EDGE_FALLING)
		intel_soc_pmic_setb(ctli, CTLI_INTCNT_NE);
}

static int crystalcove_gpio_direction_input(struct gpio_chip *chip,
					    unsigned gpio)
{
	u8 ctlo;

	if (gpio > gpio_info.gpio_data->num_gpio)
		return 0;

	ctlo = gpio < 8 ? GPIO0P0CTLO + gpio : GPIO1P0CTLO + (gpio - 8);

	intel_soc_pmic_writeb(ctlo, CTLO_INPUT_DEF);
	return 0;
}

static int crystalcove_gpio_direction_output(struct gpio_chip *chip,
					     unsigned gpio, int value)
{
	u8 ctlo;

	if (gpio > gpio_info.gpio_data->num_gpio)
		return 0;

	ctlo = gpio < 8 ? GPIO0P0CTLO + gpio : GPIO1P0CTLO + (gpio - 8);

	intel_soc_pmic_writeb(ctlo, CTLO_OUTPUT_DEF | value);
	return 0;
}

static int crystalcove_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	u8 ctli;

	if (gpio > gpio_info.gpio_data->num_gpio)
		return 0;

	ctli = gpio < 8 ? GPIO0P0CTLI + gpio : GPIO1P0CTLI + (gpio - 8);

	return intel_soc_pmic_readb(ctli) & 0x1;
}

static void crystalcove_gpio_set(struct gpio_chip *chip,
				 unsigned gpio, int value)
{
	u8 ctlo;

	if (gpio > gpio_info.gpio_data->num_gpio)
		return;

	ctlo = gpio < 8 ? GPIO0P0CTLO + gpio : GPIO1P0CTLO + (gpio - 8);

	if (value)
		intel_soc_pmic_setb(ctlo, 1);
	else
		intel_soc_pmic_clearb(ctlo, 1);
}

static int crystalcove_irq_type(struct irq_data *data, unsigned type)
{
	struct pmic_gpio *cg = irq_data_get_irq_chip_data(data);

	cg->trigger_type = type;
	cg->update |= UPDATE_TYPE;

	return 0;
}

static int crystalcove_gpio_to_irq(struct gpio_chip *chip, unsigned gpio)
{
	struct pmic_gpio *cg =
		container_of(chip, struct pmic_gpio, chip);

	if (gpio > cg->gpio_data->num_gpio)
		return -1;

	return cg->irq_base + gpio;
}

static void crystalcove_bus_lock(struct irq_data *data)
{
	struct pmic_gpio *cg = irq_data_get_irq_chip_data(data);

	mutex_lock(&cg->buslock);
}

static void crystalcove_bus_sync_unlock(struct irq_data *data)
{
	struct pmic_gpio *cg = irq_data_get_irq_chip_data(data);
	int gpio = data->irq - cg->irq_base;

	if (cg->update & UPDATE_TYPE)
		__crystalcove_irq_type(gpio, cg->trigger_type);
	if (cg->update & UPDATE_MASK)
		__crystalcove_irq_mask(gpio, cg->irq_mask);
	cg->update = 0;

	mutex_unlock(&cg->buslock);
}

static void crystalcove_irq_unmask(struct irq_data *data)
{
	struct pmic_gpio *cg = irq_data_get_irq_chip_data(data);

	cg->irq_mask = 0;
	cg->update |= UPDATE_MASK;
}

static void crystalcove_irq_mask(struct irq_data *data)
{
	struct pmic_gpio *cg = irq_data_get_irq_chip_data(data);

	cg->irq_mask = 1;
	cg->update |= UPDATE_MASK;
}

static struct irq_chip crystalcove_irqchip = {
	.name			= "PMIC-GPIO",
	.irq_mask		= crystalcove_irq_mask,
	.irq_unmask		= crystalcove_irq_unmask,
	.irq_set_type		= crystalcove_irq_type,
	.irq_bus_lock		= crystalcove_bus_lock,
	.irq_bus_sync_unlock	= crystalcove_bus_sync_unlock,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

#ifdef CONFIG_PM_SLEEP
static int crystalcove_suspend_noirq(struct device *dev)
{
	struct pmic_gpio *cg = dev_get_drvdata(dev);
	cg->print_wakeup = 1;
	return 0;
}

static int crystalcove_resume_early(struct device *dev)
{
	struct pmic_gpio *cg = dev_get_drvdata(dev);
	cg->print_wakeup = 0;
	return 0;
}
#endif

static irqreturn_t crystalcove_gpio_irq_handler(int irq, void *data)
{
	struct pmic_gpio *cg = data;
	int pending;
	int gpio;

	pending = intel_soc_pmic_readb(GPIO0IRQ) & 0xff;
	pending |= (intel_soc_pmic_readb(GPIO1IRQ) & 0xff) << 8;
	intel_soc_pmic_writeb(GPIO0IRQ, pending & 0xff);
	intel_soc_pmic_writeb(GPIO1IRQ, (pending >> 8) & 0xff);

	local_irq_disable();
	for (gpio = 0; gpio < cg->gpio_data->num_gpio; gpio++) {
		if (pending & (1 << gpio)) {
			if (cg->print_wakeup) {
				pr_info("crystalcove_gpio: irq[%d]"
					" might wake up system!\n",
					cg->irq_base + gpio);
			}
			generic_handle_irq(cg->irq_base + gpio);
		}
	}
	local_irq_enable();

	return IRQ_HANDLED;
}

static void crystalcove_gpio_dbg_show(struct seq_file *s,
				      struct gpio_chip *chip)
{
	int gpio, offset;
	u8 ctlo, ctli, mirqs0, mirqsx, irq;

	for (gpio = 0; gpio < gpio_info.gpio_data->num_gpio; gpio++) {
		offset = gpio < 8 ? gpio : gpio - 8;
		ctlo = intel_soc_pmic_readb(
			(gpio < 8 ? GPIO0P0CTLO : GPIO1P0CTLO) + offset);
		ctli = intel_soc_pmic_readb(
			(gpio < 8 ? GPIO0P0CTLI : GPIO1P0CTLI) + offset);
		mirqs0 = intel_soc_pmic_readb(
			gpio < 8 ? MGPIO0IRQS0 : MGPIO1IRQS0);
		irq = intel_soc_pmic_readb(
			gpio < 8 ? GPIO0IRQ : GPIO1IRQ);
		seq_printf(s, " gpio-%-2d %s %s %s %s ctlo=%2x,%s %s",
			   gpio, ctlo & CTLO_DIR_OUT ? "out" : "in ",
			   ctli & 0x1 ? "hi" : "lo",
			   ctli & CTLI_INTCNT_NE ? "fall" : "    ",
			   ctli & CTLI_INTCNT_PE ? "rise" : "    ",
			   ctlo,
			   mirqs0 & (1 << offset) ? "s0 mask  " : "s0 unmask",
			   irq & (1 << offset) ? "pending" : "       ");
		switch (gpio_info.gpio_data->type) {
		case CRYSTAL_COVE:
			mirqsx = intel_soc_pmic_readb(
				gpio < 8 ? MGPIO0IRQSX : MGPIO1IRQSX);
			seq_printf(s, " %s\n", mirqsx & (1 << offset) ?
				"sx mask  " : "sx unmask");
			break;
		default:
			seq_printf(s, "\n");
			break;
		}
	}
}

static int crystalcove_gpio_probe(struct platform_device *pdev)
{
	int irq = platform_get_irq(pdev, 0);
	struct pmic_gpio *cg = &gpio_info;
	int retval;
	int i;
	struct device *dev = intel_soc_pmic_dev();
	struct pmic_gpio_data *gpio_data;

	gpio_data = (struct pmic_gpio_data *)pdev->dev.platform_data;

	mutex_init(&cg->buslock);
	cg->chip.label = "intel_crystalcove";
	cg->chip.direction_input = crystalcove_gpio_direction_input;
	cg->chip.direction_output = crystalcove_gpio_direction_output;
	cg->chip.get = crystalcove_gpio_get;
	cg->chip.set = crystalcove_gpio_set;
	cg->chip.to_irq = crystalcove_gpio_to_irq;
	cg->chip.base = -1;
	cg->gpio_data = gpio_data;
	cg->chip.ngpio = cg->gpio_data->num_vgpio;
	cg->chip.can_sleep = 1;
	cg->chip.dev = dev;
	cg->chip.dbg_show = crystalcove_gpio_dbg_show;

	retval = gpiochip_add(&cg->chip);
	if (retval) {
		dev_warn(&pdev->dev, "add gpio chip error: %d\n", retval);
		return retval;
	}

	cg->irq_base = irq_alloc_descs(-1, INTEL_PMIC_IRQBASE,
					cg->gpio_data->num_gpio, 0);

	for (i = 0; i < cg->gpio_data->num_gpio; i++) {
		irq_set_chip_data(i + cg->irq_base, cg);
		irq_set_chip_and_handler_name(i + cg->irq_base,
					      &crystalcove_irqchip,
					      handle_simple_irq,
					      "demux");
	}

	retval = request_threaded_irq(irq, NULL, crystalcove_gpio_irq_handler,
				      IRQF_ONESHOT, "crystalcove_gpio", cg);

	if (retval) {
		dev_warn(&pdev->dev, "request irq failed: %d\n", retval);
		return retval;
	}

	dev_set_drvdata(&pdev->dev, cg);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops crystalcove_pm = {
	.suspend_noirq = crystalcove_suspend_noirq,
	.resume_early = crystalcove_resume_early,
};
#else
static const struct dev_pm_ops crystalcove_pm;
#endif

static struct platform_device_id crystalcove_gpio_id_table[] = {
	{ .name = "crystal_cove_gpio" },
	{ .name = "whiskey_cove_gpio" },
	{},
};
static struct platform_driver crystalcove_gpio_driver = {
	.probe = crystalcove_gpio_probe,
	.driver = {
		.name = "crystal_cove_gpio",
		.pm = &crystalcove_pm,
	},
	.id_table = crystalcove_gpio_id_table,
};

MODULE_DEVICE_TABLE(platform, crystalcove_gpio_id_table);
module_platform_driver(crystalcove_gpio_driver);

MODULE_AUTHOR("Yang, Bin <bin.yang@intel.com>");
MODULE_DESCRIPTION("Intel Crystal Cove GPIO Driver");
MODULE_LICENSE("GPL");
