#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/input/mt.h>

#include "pad_main.h"


//#define DEBUG
#ifdef DEBUG
#undef pr_debug
#define pr_debug pr_err
#endif

#define DRIVER_NAME "touch_pad"



struct pad_controller 
{
	struct platform_device *pdev;
	struct input_dev *pointer_dev;
    int line;
};


static struct pad_controller* p_controller;


void create_input(void )
{
	int ret;
    struct input_dev *input_dev;

    input_dev = input_allocate_device();

    if (input_dev == NULL)
    {
        return ;
    }

    input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    //input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    set_bit(INPUT_PROP_POINTER, input_dev->propbit);
    set_bit(INPUT_PROP_SEMI_MT, input_dev->propbit);
    //set_bit(INPUT_PROP_DIRECT, input_dev->propbit);


    input_mt_init_slots(input_dev, 16, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 0x190, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0x280, 0x500, 0, 0);
    //input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 0x12C, 0, 0);
    //input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0x2d0, 0x4b0, 0, 0);
    //input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 0x123, 0, 0);
    //input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0x244, 0x4a7, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    input_dev->name = "lenovo-smartpad_touch";
    input_dev->phys = "input/ts";
    input_dev->id.bustype = BUS_I2C;
    input_dev->id.vendor = 0x0;
    input_dev->id.product = 0x0;
    input_dev->id.version = 0x0;

    ret = input_register_device(input_dev);

	p_controller->pointer_dev = input_dev;
}

void set_tp_resolution(int w, int h)
{
}

int notify_touch_event(int finger, int x, int y, int w)
{
    if (x > p_controller->line )
        return 0;
    if ( x >=  0 ) {
    pr_err("%s id %d  x %x y %d\n", __func__,finger, x, y);
    input_mt_slot(p_controller->pointer_dev, finger);
    input_report_abs(p_controller->pointer_dev, ABS_MT_TRACKING_ID, finger);
    input_report_abs(p_controller->pointer_dev, ABS_MT_POSITION_X, x);
    input_report_abs(p_controller->pointer_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(p_controller->pointer_dev, ABS_MT_WIDTH_MAJOR, w);
    input_report_abs(p_controller->pointer_dev, ABS_MT_TOUCH_MAJOR, w);
    }else
    {
         input_mt_slot(p_controller->pointer_dev, finger);
         input_report_abs(p_controller->pointer_dev, ABS_MT_TRACKING_ID, -1);
    }
    input_sync(p_controller->pointer_dev);
	return 1;
}
EXPORT_SYMBOL(notify_touch_event);



static ssize_t line_store(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
	int v1;
	sscanf(buf, "%d", &v1);
	p_controller->line  = msecs_to_jiffies(v1);
    return count;
}
static ssize_t line_show(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
        pr_debug("%s\n", __func__);
        return snprintf(buf, PAGE_SIZE, "%d\n", jiffies_to_msecs(p_controller->line));
}



static DEVICE_ATTR(line, 0664, line_show, line_store);

static struct attribute *pad__attrs[] = {
	//&dev_attr_mode.attr,
	&dev_attr_line.attr,
	NULL,
};


static struct attribute_group pad__attr_group = {
	.attrs = pad__attrs,
};

static int pad__input_probe(struct platform_device *pdev)
{
	int ret;

	pr_info("%s enter\n", __func__);
	p_controller = kmalloc(sizeof(struct pad_controller), GFP_KERNEL);
	if (!p_controller)
	{
		pr_info("%s error allocate controller \n", __func__);
		return -1;
	}
	p_controller->pdev = pdev;
	p_controller->line = 0x123;
	ret = sysfs_create_group(&pdev->dev.kobj, &pad__attr_group);
	if (ret<0)
	{
		pr_err("attr create error\n");
	}
	create_input();
	pr_info("%s\n", __func__);
	return 0;
}

static int pad__input_remove(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	input_unregister_device(p_controller->pointer_dev);
	kfree(p_controller);
        return 0;
}
#ifdef CONFIG_PM_SLEEP
static int pad_main_suspend(struct device *dev)
{
	//pr_debug("%s\n", __func__);

        return 0;
}

static int pad_main_resume(struct device *dev)
{
	//pr_debug("%s\n", __func__);
        return 0;
}
static SIMPLE_DEV_PM_OPS(pad_main_pm, pad_main_suspend, pad_main_resume);
#endif

static struct platform_driver pad__input_driver = {
	.driver         = {
                .name   = DRIVER_NAME,
                .owner  = THIS_MODULE,
                .pm     = &pad_main_pm,
        },

        .probe  = pad__input_probe,
        .remove = pad__input_remove,
};


static struct platform_device pad__input_pdev = {
        /* should be the same name as driver_name */ 
        .name = DRIVER_NAME,
        .id = -1,
};


static int __init pad__init(void)
{
	int ret;


        ret = platform_driver_register(&pad__input_driver);
        if (ret < 0) {
                goto err_driver_register;
	}

        ret = platform_device_register(&pad__input_pdev);
        if (ret < 0) {
		pr_err("%s %d\n", __func__, ret);
                goto err_platform_device_register;
	}
	return 0;

err_platform_device_register:
        platform_driver_unregister(&pad__input_driver);
err_driver_register:
        return ret;
}

static void __exit pad__exit(void)
{
	platform_device_unregister(&pad__input_pdev);
        platform_driver_unregister(&pad__input_driver);
}

module_init(pad__init);
module_exit(pad__exit);

MODULE_AUTHOR("lenovo");
MODULE_DESCRIPTION("pad input device");
MODULE_LICENSE("GPL");
