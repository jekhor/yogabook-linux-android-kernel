/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>

#define gpio_southwest_NUM 98
#define gpio_north_NUM 73
#define gpio_east_NUM 27
#define gpio_southeast_NUM 86

#define gpio_southwest_base (ARCH_NR_GPIOS-gpio_southwest_NUM) //414
#define gpio_north_base (gpio_southwest_base - gpio_north_NUM) //341
#define gpio_east_base (gpio_north_base - gpio_east_NUM) //314
#define gpio_southeast_base (gpio_east_base - gpio_southeast_NUM)//228

#define SD3_CD_N 81 //se81

#define GPIO_SLOT (gpio_southeast_base + SD3_CD_N)
#define SLOT_GPIO_WAKElOCK_TIMEOUT   (1 * HZ)

struct slot_gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
};

struct attribute private_attr = {
        .name = "tlmm",
        .mode = S_IRWXUGO,
};

static struct attribute *slot_gpio_kobj_attrs[] = {
        &private_attr,
        NULL,
};

ssize_t slot_gpio_kobj_show(struct kobject *kobject, struct attribute *attr,char *buf)
{
	int state = 0;
	state = gpio_get_value(GPIO_SLOT);
	return sprintf(buf, "1UI2 = %d\n", state);
}

struct sysfs_ops slot_gpio_kobj_sysops =
{
         .show = slot_gpio_kobj_show,
};

struct kobj_type slot_gpio_ktype =
{
       .sysfs_ops = &slot_gpio_kobj_sysops,
       .default_attrs = slot_gpio_kobj_attrs,
};

struct kobject slot_gpio_kobj;

static ssize_t switch_slot_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct slot_gpio_switch_data *switch_data = container_of(sdev, struct slot_gpio_switch_data, sdev);
        int state = 0;

        state = gpio_get_value(switch_data->gpio);
        printk("slot_gpio_switch_work() %d\n", state);
        switch_set_state(&switch_data->sdev, state);
	if (state)
		return sprintf(buf, "%d\n", state);
	return -1;
}

/* Register two kobject:
 * 1)/sys/class/switch2/slot_gpio for android HAL,
 *   switch will send uevent to HAL,android can handle this uevent;
 * 2)/sys/private/tlmm for lenove SIM tray test;
*/
static int slot_gpio_switch_probe(struct platform_device *pdev)
{
	struct slot_gpio_switch_data *switch_data;
	int ret = 0;

        printk("slot_gpio_switch_probe()\n");
        switch_data = kzalloc(sizeof(struct slot_gpio_switch_data), GFP_KERNEL);
        if (!switch_data)
               return -ENOMEM;

        switch_data->sdev.name = "slot_gpio";
        switch_data->gpio = GPIO_SLOT;//pdata->gpio;
	switch_data->sdev.print_state = switch_slot_gpio_print_state;
	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
            goto err_switch_dev_register;

	ret = kobject_init_and_add(&slot_gpio_kobj,&slot_gpio_ktype,NULL,"private");
	if(ret)
	    goto err_kobject_init_and_add;
	return 0;

err_kobject_init_and_add:
	switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static struct platform_driver slot_gpio_driver = {
	.probe = slot_gpio_switch_probe,
	.driver = {
		.name = "slot_gpio_switch",
		.owner = THIS_MODULE,
	},
};

static struct platform_device slot_gpio_switch = {
	       .name = "slot_gpio_switch",
	};


static struct platform_device *slot_gpio_devices_evb[] __initdata = {
	        &slot_gpio_switch,
	};


static int __init slot_gpio_init(void)
{
    platform_add_devices(slot_gpio_devices_evb, ARRAY_SIZE(slot_gpio_devices_evb));
    return platform_driver_register(&slot_gpio_driver);
}

fs_initcall(slot_gpio_init);
MODULE_DESCRIPTION("SIM card slot gpio switch driver");
MODULE_LICENSE("GPL");
