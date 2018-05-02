#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "pad_main.h"


//#define DEBUG
#ifdef DEBUG
#undef pr_debug
#define pr_debug pr_err
#endif

#define DRIVER_NAME "touchpad"
#define P_WIDTH     2230
#define P_HEIGHT    1250

enum WORK_MODE {
	OFF_MODE = 0x00,
	SLEEP_MODE,
        WORK_MODE,
	MAX_MODE,	
};


struct rect {
	int topx;
	int topy;
	int endx;
	int endy;
};

enum pad_state{
        IDLE = 0x00,
        BUTTON,
        MOVING,
        NUM_STATE,
};      
        
        
#define PAD_SLEEP_MASK        0x10
#define BUTTON_MARGIN  0

struct pad_controller 
{
	struct platform_device *pdev;
	struct rect pad_area;
	struct input_dev *mse_dev;
	int current_mode;
	int current_state;
	struct pad_pointer pt;
	//time stamp
	unsigned long timestamp;
	unsigned long config_timeout;
	unsigned char swap_axis;
	//struct work_struct work;
	//int (*handle_event)(int id, int x, int y);
	unsigned char support_right;
};


static struct pad_controller* p_controller;

static int is_in_rect(struct rect area, struct pad_pointer pt)
{
	return ( pt.x > area.topx) && (pt.x < area.endx )
		&& (pt.y > area.topy) && (pt.y < area.endy );  
}

static int idle_process(struct pad_pointer pt);
static int button_process(struct pad_pointer pt);
static int moving_process(struct pad_pointer pt);
static int (*state_process[])(struct pad_pointer pt) = {
        idle_process,
        button_process,
        moving_process
};


static int idle_process(struct pad_pointer pt)
{
	if (( pt.finger >= 0 ) && (pt.x >= 0 ) /*&& is_in_rect(p_controller->pad_area, pt)*/)
	{
		p_controller->pt.x = pt.x;
		p_controller->pt.y = pt.y;
		p_controller->pt.finger = pt.finger; 
		p_controller->current_state = BUTTON;
		p_controller->timestamp = jiffies;
		pr_info("padtouch start \n");	
	}
	return 1;
}
static int button_process(struct pad_pointer pt)
{
	unsigned long cj = jiffies;
	unsigned short btn = BTN_LEFT;
	pr_debug("%s \n", __func__);
	if (pt.finger == p_controller->pt.finger) 
	{
		//pr_debug("here\n");
		if ((pt.x < 0 ) && ((p_controller->timestamp + p_controller->config_timeout ) > cj ))
		{
			/* button click */
			if (p_controller->support_right) {
				if (p_controller->pt.y > 
			((p_controller->pad_area.endy + p_controller->pad_area.topy) /2 + BUTTON_MARGIN))

				{
					btn = BTN_RIGHT;
				}
			}
			pr_info("padtouch click %d\n", btn);	
			input_report_key(p_controller->mse_dev, btn, 1);
        		input_sync(p_controller->mse_dev);
			input_report_key(p_controller->mse_dev, btn, 0);
        		input_sync(p_controller->mse_dev);
                	p_controller->pt.x = -1;
                	p_controller->pt.y = -1;
			p_controller->pt.finger = -1;
			p_controller->current_state = IDLE;
		}else if ((p_controller->timestamp + p_controller->config_timeout ) <  cj )
		{
			pr_info("padtouch moving \n");
			p_controller->current_state = MOVING;
		}
	}
	return 1;
}
static int moving_process(struct pad_pointer pt)
{
	int dx, dy;
	pr_debug("%s \n", __func__);
	if ((pt.finger == p_controller->pt.finger ) && ( pt.x >= 0)){
		/* moving */
		dx = pt.x - p_controller->pt.x;
		dy = pt.y - p_controller->pt.y;
		if (p_controller->swap_axis)
		{
                	input_report_rel(p_controller->mse_dev, REL_X, dy);
                	input_report_rel(p_controller->mse_dev, REL_Y, dx*(-1));
		}
		else{
                	input_report_rel(p_controller->mse_dev, REL_X, dy);
                	input_report_rel(p_controller->mse_dev, REL_Y, dx);
		}
                input_sync(p_controller->mse_dev);
                p_controller->pt.x = pt.x;
                p_controller->pt.y = pt.y;
	}else if (( pt.finger == -1 ) || (( pt.finger == p_controller->pt.finger) && ( pt.x < 0 )))
	{
                p_controller->pt.x = -1;
                p_controller->pt.y = -1;
		p_controller->pt.finger = -1;
		p_controller->current_state = IDLE;
	}
	return 0;
}

static int handle_mouse(int finger, int x,int y)
{
	struct pad_pointer pt;
	pt.finger = finger;
	pt.x = x;
	pt.y = y;
	//pr_debug("%s %d\n", __func__, p_controller->current_state);
	return state_process[p_controller->current_state](pt);
#if 0 
	if (finger < 0 ){
                p_controller->pt.x = p_controller->pt.y = -1;
        }
        else if (p_controller->pt.x < 0 )
        {
                p_controller->pt.x = x;
                p_controller->pt.y = y;
		p_controller->pt.finger = -1;
        }else {
                input_report_rel(p_controller->mse_dev, REL_X, x-p_controller->pt.x);
                input_report_rel(p_controller->mse_dev, REL_Y, y-p_controller->pt.y);
                input_sync(p_controller->mse_dev);
                p_controller->pt.x = x;
                p_controller->pt.y = y;
		if ( -1 == p_controller->finger) {
			p_controller->finger = finger;
			p_controller->timestamp = jiffies;
		}
	}
	return 1;
#endif
}

void create_mouse(void )
{
	int ret;
	struct input_dev *input_dev;
	input_dev = input_allocate_device();
	input_dev->name = "lenovo-smartpad-mouse";

        input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);
        input_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT); 
        input_dev->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);

	ret = input_register_device(input_dev);
	p_controller->mse_dev = input_dev;
}

void set_tp_resolution(int w, int h)
{
}

int notify_touch_event(int finger, int x, int y)
{
	static int type, code;
	struct pad_pointer pt;
	pt.x = x;
	pt.y = y;
	if (!p_controller)
		return 0;
	if ( p_controller->current_mode == OFF_MODE )
	{
		return 0;
	}
	pr_debug("%s, %d ,%d \n", __func__, x, y);
	if (( x < 0 ) || is_in_rect(p_controller->pad_area, pt))
	{

		if ((( p_controller->pt.finger == -1) && (x >= 0 )) 
			|| (p_controller->pt.finger == finger )) {
			pr_debug("%s %d %d %d, %d\n", __func__, finger, x , y, p_controller->pt.finger);
			handle_mouse(finger,x,y);
		}
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(notify_touch_event);

static void pad__work(struct work_struct *work)
{
	pr_debug("%s\n", __func__);
}
void switch_mode(int mode)
{
	pr_info("%s %d\n", __func__, mode);
	p_controller->current_mode = mode;
}

#if 0
static ssize_t hwver_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	pr_debug("%s\n", __func__);
	return 0;
}
#endif
static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value;
	int ret;
	pr_debug("%s\n", __func__);
	ret = kstrtoint(buf,10,&value);
	if ( value < MAX_MODE ){
		switch_mode(value);
	}
	return count;
}
static ssize_t mode_show(struct device *dev, struct device_attribute *attr, 
				char *buf)
{
	
	pr_debug("%s\n", __func__);
	return snprintf(buf, PAGE_SIZE, "%d\n",p_controller->current_mode);
}
static ssize_t pad_area_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int x, y, xx, yy;
	int cnt;
	cnt = sscanf(buf, "%d %d %d %d", &x, &y, &xx, &yy);
	if ( cnt < 4)  {
		printk("invalid input %s\n", buf);
		return count;
	}
	p_controller->pad_area.topx = x;
	p_controller->pad_area.topy= y;
	p_controller->pad_area.endx = xx;
	p_controller->pad_area.endy = yy;
	printk(" pad_area %d %d-- %d %d\n", x, y, xx, yy);
	return count;
}
static ssize_t pad_area_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	pr_debug("%s\n", __func__);
	return snprintf(buf, PAGE_SIZE, "%d %d\n", p_controller->pad_area.topx, p_controller->pad_area.topy);
}

static ssize_t button_time_store(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
	int v1;
        pr_debug("%s\n", __func__);
	sscanf(buf, "%d", &v1);
	if ( v1 > 0 )
	{
		p_controller->config_timeout = msecs_to_jiffies(v1);
	}
        return count;
}
static ssize_t button_time_show(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
        pr_debug("%s\n", __func__);
        return snprintf(buf, PAGE_SIZE, "%d\n", jiffies_to_msecs(p_controller->config_timeout));
}

static ssize_t swap_axis_store(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
        int v1;
        pr_debug("%s\n", __func__);
        sscanf(buf, "%d", &v1);
	if (v1)
		v1 = 1;
        p_controller->swap_axis = v1;
        return count;
}
static ssize_t swap_axis_show(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
        pr_debug("%s\n", __func__);
        return snprintf(buf, PAGE_SIZE, "%d\n", p_controller->swap_axis);
}

static ssize_t right_enable_store(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
        int v1;
        //pr_debug("%s\n", __func__);
        sscanf(buf, "%d", &v1);
        if (v1)
                v1 = 1;
        p_controller->support_right= v1;
        return count;
}
static ssize_t right_enable_show(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
        //pr_debug("%s\n", __func__);
        return snprintf(buf, PAGE_SIZE, "%d\n", p_controller->support_right);
}



static DEVICE_ATTR(mode,0664, mode_show, mode_store);
//static DEVICE_ATTR(hwver,0777,hwver_show,NULL);
static DEVICE_ATTR(pad_area,0664, pad_area_show, pad_area_store);
static DEVICE_ATTR(button_time, 0664, button_time_show, button_time_store);
static DEVICE_ATTR(swap_axis, 0664, swap_axis_show, swap_axis_store);
static DEVICE_ATTR(right_enable, 0664, right_enable_show, right_enable_store);

static struct attribute *pad__attrs[] = {
	//&dev_attr_mode.attr,
//	&dev_attr_hwver.attr,
	&dev_attr_pad_area.attr,
	&dev_attr_button_time.attr,
	&dev_attr_right_enable.attr,
	&dev_attr_swap_axis.attr,
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
	p_controller->pad_area.topx = 0x0;
	p_controller->pad_area.topy = 0x244;
	p_controller->pad_area.endx = 0x123;
	p_controller->pad_area.endy = 0x4a7;
	p_controller->current_state = IDLE; // should be sleep mode 
	p_controller->config_timeout = msecs_to_jiffies(100);
	p_controller->current_mode = WORK_MODE;
	p_controller->support_right = 1;
	p_controller->pt.finger = -1;
	p_controller->swap_axis = 1;
	ret = sysfs_create_group(&pdev->dev.kobj, &pad__attr_group);
	if (ret<0)
	{
		pr_err("attr create error\n");
	}
	//INIT_WORK(&p_controller->work, pad__work);
	create_mouse();
	pr_info("%s\n", __func__);
	return 0;
}

static int pad__input_remove(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	input_unregister_device(p_controller->mse_dev);
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
