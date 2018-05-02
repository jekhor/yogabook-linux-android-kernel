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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>

#define SEC_HALL_DEBUG

#define gpio_southwest_NUM 98
#define gpio_north_NUM 73
#define gpio_east_NUM 27
#define gpio_southeast_NUM 86

#define gpio_southwest_base (ARCH_NR_GPIOS-gpio_southwest_NUM) //414
#define gpio_north_base (gpio_southwest_base - gpio_north_NUM) //341
#define gpio_east_base (gpio_north_base - gpio_east_NUM) //314
#define gpio_southeast_base (gpio_east_base - gpio_southeast_NUM)//228

#define MF_ISH_GPIO_1 18//e18

#define GPIO_SEC_HALL_NIRQ (gpio_east_base + MF_ISH_GPIO_1)
#define HALL_WAKElOCK_TIMEOUT   (1 * HZ)

static DEFINE_MUTEX(mutex);
struct sec_hall_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	int wake_flag;
	struct work_struct work;
	struct wake_lock wakelock;
};

static void sec_hall_switch_work(struct work_struct *work)
{
	int state;
	struct sec_hall_switch_data	*data =
		container_of(work, struct sec_hall_switch_data, work);

	state = gpio_get_value(data->gpio);
#ifdef SEC_HALL_DEBUG
    printk("sec_hall_switch_work() %d\n", state);
#endif
	switch_set_state(&data->sdev, state);
}

static irqreturn_t sec_hall_irq_handler(int irq, void *dev_id)
{
	struct sec_hall_switch_data *switch_data =
	    (struct sec_hall_switch_data *)dev_id;

	schedule_work(&switch_data->work);
	wake_lock_timeout(&switch_data->wakelock, HALL_WAKElOCK_TIMEOUT);

	return IRQ_HANDLED;
}

static ssize_t switch_sec_hall_print_state(struct switch_dev *sdev, char *buf)
{
	struct sec_hall_switch_data	*switch_data =
		container_of(sdev, struct sec_hall_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static ssize_t suspend_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    int flag = 0;
    struct sec_hall_switch_data *switch_data;
    switch_data = dev_get_drvdata(dev);

    mutex_lock(&mutex);
    flag = switch_data->wake_flag;
    mutex_unlock(&mutex);
    return sprintf(buf,"%s\n",flag ?
	    "suspend" : "resume");
}

static ssize_t suspend_store(struct device *dev,
			    struct device_attribute *attr,
			    char *buf,size_t count)
{
    printk(KERN_ERR"%s:suspend_store working...\n",__func__);
    int ret;
    struct sec_hall_switch_data *switch_data;
    switch_data = dev_get_drvdata(dev);

    mutex_lock(&mutex);
    if(!strncmp(buf,"1",1)){
        switch_data->wake_flag = 1;
        disable_irq(switch_data->irq);
	disable_irq_wake(switch_data->irq);
	free_irq(switch_data->irq,switch_data);
    } else {
	switch_data->wake_flag = 0;
	ret = request_irq(switch_data->irq, sec_hall_irq_handler,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, switch_data->sdev.name, switch_data);
	if (ret < 0)
	    printk(KERN_ERR"%s:request irq [%d] failed!\n",__func__,switch_data->irq);

        enable_irq(switch_data->irq);
	enable_irq_wake(switch_data->irq);
    }
    mutex_unlock(&mutex);

    return count;
}

static DEVICE_ATTR(power_HAL_suspend,0777,suspend_show,suspend_store);

static int sec_hall_switch_probe(struct platform_device *pdev)
{
	//struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct sec_hall_switch_data *switch_data;
	int ret = 0;

	//if (!pdata)
	//	return -EBUSY;
#ifdef SEC_HALL_DEBUG
    printk("sec_hall_switch_probe()\n");
#endif

	switch_data = kzalloc(sizeof(struct sec_hall_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = "sec_hall";
	switch_data->gpio = GPIO_SEC_HALL_NIRQ;//pdata->gpio;
	//switch_data->name_on = pdata->name_on;
	//switch_data->name_off = pdata->name_off;
	//switch_data->state_on = pdata->state_on;
	//switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_sec_hall_print_state;
	switch_data->wake_flag = 0;

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->gpio, switch_data->sdev.name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, sec_hall_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(switch_data->irq, sec_hall_irq_handler,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, switch_data->sdev.name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	enable_irq_wake(switch_data->irq);
	/* Perform initial detection */
	sec_hall_switch_work(&switch_data->work);
	wake_lock_init(&switch_data->wakelock, WAKE_LOCK_SUSPEND, "sec_hall");


	dev_set_drvdata(&pdev->dev,switch_data);
	ret = device_create_file(&pdev->dev,&dev_attr_power_HAL_suspend);
	if(ret < 0){
	    printk(KERN_ERR"Create file error!\n");
	    goto err_request_irq;
	}

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}
/*
static int sec_hall_switch_remove(struct platform_device *pdev)
{
    struct sec_hall_switch_data *switch_data = platform_get_drvdata(pdev);

    cancel_work_sync(&switch_data->work);
    gpio_free(switch_data->gpio);
    switch_dev_unregister(&switch_data->sdev);
    kfree(switch_data);

    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id sec_hall_match_table[] = {
	{ .compatible = "qcom,sec_hall",},
	{ },
};
#else
#define sec_hall_match_table NULL
#endif
MODULE_DEVICE_TABLE(of, sec_hall_match_table);
static struct platform_driver sec_hall_switch_driver = {
	.probe		= sec_hall_switch_probe,
	.remove		= sec_hall_switch_remove,
	.driver		= {
		.name	= "sec_hall",
		.owner	= THIS_MODULE,
                .of_match_table = sec_hall_match_table,
	},
};

static int __init sec_hall_switch_init(void)
{
	return platform_driver_register(&sec_hall_switch_driver);
}

static void __exit sec_hall_switch_exit(void)
{
	platform_driver_unregister(&sec_hall_switch_driver);
}

module_init(sec_hall_switch_init);
module_exit(sec_hall_switch_exit);

MODULE_AUTHOR("Lenovo");
MODULE_DESCRIPTION("CAP Sensor driver");
MODULE_LICENSE("GPL");
*/
static struct platform_driver cht_sec_hall_driver = {
	.probe = sec_hall_switch_probe,
	.driver = {
		.name = "cht_sec_hall_switch",
		.owner = THIS_MODULE,
	},
};

static struct platform_device cht_sec_hall_switch = {
	       .name      = "cht_sec_hall_switch",
	};


static struct platform_device *sec_hall_devices_evb[] __initdata = {
	        &cht_sec_hall_switch,
	};


static int __init sec_hall_init(void)
{
 /*       struct sysdev_class_attribute **attr;
        int res;

        res = sysdev_class_register(&module_sec_hall_class);
        if (unlikely(res)) {
                return res;
        }

        for (attr = msec_hall_int_attributes; *attr; attr++) {
                res = sysdev_class_create_file(&module_sec_hall_class, *attr);
                if (res)
                        goto out_unreg;
        }
*/
	
    platform_add_devices(sec_hall_devices_evb, ARRAY_SIZE(sec_hall_devices_evb));
    return platform_driver_register(&cht_sec_hall_driver);
/*
out_unreg:
        for (; attr >= msec_hall_int_attributes; attr--)
                sysdev_class_remove_file(&module_sec_hall_class, *attr);
        sysdev_class_unregister(&module_sec_hall_class);

        return res;
*/

}

fs_initcall(sec_hall_init);
MODULE_DESCRIPTION("Sec_Hall switch sensor driver");
MODULE_LICENSE("GPL");

