/*
 * platform_iwlwifi_pcie.c: iwlwifi pcie platform device.
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/leds_pwm.h>

#define gpio_southwest_NUM	98
#define gpio_north_NUM	73
#define gpio_east_NUM	27
#define	gpio_southeast_NUM	86

#define	gpio_southwest_base	(ARCH_NR_GPIOS-gpio_southwest_NUM)
#define gpio_north_base		(gpio_southwest_base - gpio_north_NUM)
#define	gpio_east_base		(gpio_north_base - gpio_east_NUM)
#define gpio_southeast_base		(gpio_east_base - gpio_southeast_NUM)
#define MF_LPC_AD1 52

#define TP_LED_EN	(MF_LPC_AD1 + gpio_southeast_base)

static struct led_pwm pwm_leds[] = {
	{
		.name		= "keyboard-led",
		.max_brightness	= 255,
		.pwm_period_ns	= 40000,
		.en_gpio		= TP_LED_EN,
	},
};

static struct led_pwm_platform_data pwm_data = {
	.num_leds	= ARRAY_SIZE(pwm_leds),
	.leds		= pwm_leds,
};

static struct platform_device leds_tps61163 = {
	.name	= "leds_tps61163",
	.id	= -1,
	.dev	= {
		.platform_data = &pwm_data,
	},
};

int __init tps61163a_platform_init(void)
{
	int ret = 0;

	ret = platform_device_register(&leds_tps61163);
	pr_info("leds_tps61163 platform init\n");

	return ret;
}
late_initcall(tps61163a_platform_init);
