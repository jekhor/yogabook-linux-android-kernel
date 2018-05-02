/*
 * drivers/usb/otg/nop-usb-xceiv.c
 *
 * NOP USB transceiver for all USB transceiver which are either built-in
 * into USB IP or which are mostly autonomous.
 *
 * Copyright (C) 2009 Texas Instruments Inc
 * Author: Ajay Kumar Gupta <ajay.gupta@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Current status:
 *	This provides a "nop" transceiver for PHYs which are
 *	autonomous such as isp1504, isp1707, etc.
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>
#include <linux/usb/usb_phy_gen_xceiv.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "phy-generic.h"

static struct platform_device *pd;

void usb_nop_xceiv_register(void)
{
	if (pd)
		return;
	pd = platform_device_register_simple("usb_phy_gen_xceiv", -1, NULL, 0);
	if (IS_ERR(pd)) {
		pr_err("Unable to register generic usb transceiver\n");
		pd = NULL;
		return;
	}
}
EXPORT_SYMBOL(usb_nop_xceiv_register);

void usb_nop_xceiv_unregister(void)
{
	platform_device_unregister(pd);
	pd = NULL;
}
EXPORT_SYMBOL(usb_nop_xceiv_unregister);

static int nop_set_suspend(struct usb_phy *x, int suspend)
{
	return 0;
}

static void nop_func_set(struct usb_phy_gen_xceiv *nop, struct gpio_desc *gpio,
			 int asserted, int wait)
{
	if (IS_ERR(gpio))
		return;

	gpiod_set_value_cansleep(gpio, asserted);

	if (wait)
		usleep_range(10000, 20000);
}

static inline void nop_reset_set(struct usb_phy_gen_xceiv *nop, int asserted)
{
	nop_func_set(nop, nop->gpio_reset, !asserted, !asserted);
}

static inline void nop_cs_set(struct usb_phy_gen_xceiv *nop, int asserted)
{
	nop_func_set(nop, nop->gpio_cs, asserted, asserted);
}

int usb_gen_phy_init(struct usb_phy *phy)
{
	struct usb_phy_gen_xceiv *nop = dev_get_drvdata(phy->dev);

	/* Assert CS */
	nop_cs_set(nop, 1);

	if (!IS_ERR(nop->vcc)) {
		if (regulator_enable(nop->vcc))
			dev_err(phy->dev, "Failed to enable power\n");
	}

	if (!IS_ERR(nop->clk))
		clk_prepare_enable(nop->clk);

	/* De-assert RESET */
	nop_reset_set(nop, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_gen_phy_init);

void usb_gen_phy_shutdown(struct usb_phy *phy)
{
	struct usb_phy_gen_xceiv *nop = dev_get_drvdata(phy->dev);

	/* Assert RESET */
	nop_reset_set(nop, 1);

	if (!IS_ERR(nop->clk))
		clk_disable_unprepare(nop->clk);

	if (!IS_ERR(nop->vcc)) {
		if (regulator_disable(nop->vcc))
			dev_err(phy->dev, "Failed to disable power\n");
	}

	/* De-assert CS */
	nop_cs_set(nop, 0);
}
EXPORT_SYMBOL_GPL(usb_gen_phy_shutdown);

static int nop_set_peripheral(struct usb_otg *otg, struct usb_gadget *gadget)
{
	if (!otg)
		return -ENODEV;

	if (!gadget) {
		otg->gadget = NULL;
		return -ENODEV;
	}

	otg->gadget = gadget;
	otg->phy->state = OTG_STATE_B_IDLE;
	return 0;
}

static int nop_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	if (!otg)
		return -ENODEV;

	if (!host) {
		otg->host = NULL;
		return -ENODEV;
	}

	otg->host = host;
	return 0;
}

static int nop_set_power(struct usb_phy *phy, unsigned mA)
{
	/* Notify other drivers that device enumerated or not.
	 * e.g It is needed by some charger driver, to set
	 * charging current for SDP case */
	atomic_notifier_call_chain(&phy->notifier,
				   USB_EVENT_ENUMERATED, &mA);

	dev_info(phy->dev, "Draw %d mA\n", mA);

	return 0;
}

int usb_phy_gen_create_phy(struct device *dev, struct usb_phy_gen_xceiv *nop,
		struct usb_phy_gen_xceiv_platform_data *pdata)
{
	enum usb_phy_type type = USB_PHY_TYPE_USB2;
	struct device *gpio_dev = dev;
	int err;

	u32 clk_rate = 0;
	bool needs_vcc = false;

	if (dev->of_node) {
		struct device_node *node = dev->of_node;

		if (of_property_read_u32(node, "clock-frequency", &clk_rate))
			clk_rate = 0;

		needs_vcc = of_property_read_bool(node, "vcc-supply");

	} else if (pdata) {
		type = pdata->type;
		clk_rate = pdata->clk_rate;
		needs_vcc = pdata->needs_vcc;
	}

	nop->phy.otg = devm_kzalloc(dev, sizeof(*nop->phy.otg),
			GFP_KERNEL);
	if (!nop->phy.otg)
		return -ENOMEM;

	nop->clk = devm_clk_get(dev, "main_clk");
	if (IS_ERR(nop->clk)) {
		dev_dbg(dev, "Can't get phy clock: %ld\n",
					PTR_ERR(nop->clk));
	}

	if (!IS_ERR(nop->clk) && clk_rate) {
		err = clk_set_rate(nop->clk, clk_rate);
		if (err) {
			dev_err(dev, "Error setting clock rate\n");
			return err;
		}
	}

	nop->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(nop->vcc)) {
		dev_dbg(dev, "Error getting vcc regulator: %ld\n",
					PTR_ERR(nop->vcc));
		if (needs_vcc)
			return -EPROBE_DEFER;
	}

	/* HACK: use parent's ACPI companion if only available */
	if (!ACPI_HANDLE(dev) && ACPI_HANDLE(dev->parent))
		gpio_dev = dev->parent;

	nop->gpio_reset = devm_gpiod_get_index(gpio_dev, "reset", 0);
	if (IS_ERR(nop->gpio_reset)) {
		if (PTR_ERR(nop->gpio_reset) == -EPROBE_DEFER) {
			dev_err(dev, "Error requesting RESET GPIO\n");
			return -EPROBE_DEFER;
		}
		dev_info(dev, "No RESET GPIO is available\n");
	}

	nop->gpio_cs = devm_gpiod_get_index(gpio_dev, "cs", 1);
	if (IS_ERR(nop->gpio_cs)) {
		if (PTR_ERR(nop->gpio_cs) == -EPROBE_DEFER) {
			dev_err(dev, "Error requesting CS GPIO\n");
			return -EPROBE_DEFER;
		}
		dev_info(dev, "No CS GPIO is available\n");
	}

	nop->dev		= dev;
	nop->phy.dev		= nop->dev;
	nop->phy.label		= "nop-xceiv";
	nop->phy.set_suspend	= nop_set_suspend;
	nop->phy.state		= OTG_STATE_UNDEFINED;
	nop->phy.type		= type;
	nop->phy.set_power	= nop_set_power;

	nop->phy.otg->phy		= &nop->phy;
	nop->phy.otg->set_host		= nop_set_host;
	nop->phy.otg->set_peripheral	= nop_set_peripheral;

	return 0;
}
EXPORT_SYMBOL_GPL(usb_phy_gen_create_phy);

static int usb_phy_gen_xceiv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct usb_phy_gen_xceiv	*nop;
	int err;

	nop = devm_kzalloc(dev, sizeof(*nop), GFP_KERNEL);
	if (!nop)
		return -ENOMEM;

	err = usb_phy_gen_create_phy(dev, nop, dev_get_platdata(&pdev->dev));
	if (err)
		return err;

	nop->phy.init		= usb_gen_phy_init;
	nop->phy.shutdown	= usb_gen_phy_shutdown;

	err = usb_add_phy_dev(&nop->phy);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n",
			err);
		return err;
	}

	platform_set_drvdata(pdev, nop);

	return 0;
}

static int usb_phy_gen_xceiv_remove(struct platform_device *pdev)
{
	struct usb_phy_gen_xceiv *nop = platform_get_drvdata(pdev);

	usb_remove_phy(&nop->phy);

	return 0;
}

static const struct of_device_id nop_xceiv_dt_ids[] = {
	{ .compatible = "usb-nop-xceiv" },
	{ }
};

MODULE_DEVICE_TABLE(of, nop_xceiv_dt_ids);

static struct platform_driver usb_phy_gen_xceiv_driver = {
	.probe		= usb_phy_gen_xceiv_probe,
	.remove		= usb_phy_gen_xceiv_remove,
	.driver		= {
		.name	= "usb_phy_gen_xceiv",
		.owner	= THIS_MODULE,
		.of_match_table = nop_xceiv_dt_ids,
	},
};

static int __init usb_phy_gen_xceiv_init(void)
{
	return platform_driver_register(&usb_phy_gen_xceiv_driver);
}
subsys_initcall(usb_phy_gen_xceiv_init);

static void __exit usb_phy_gen_xceiv_exit(void)
{
	platform_driver_unregister(&usb_phy_gen_xceiv_driver);
}
module_exit(usb_phy_gen_xceiv_exit);

MODULE_ALIAS("platform:usb_phy_gen_xceiv");
MODULE_AUTHOR("Texas Instruments Inc");
MODULE_DESCRIPTION("NOP USB Transceiver driver");
MODULE_LICENSE("GPL");
