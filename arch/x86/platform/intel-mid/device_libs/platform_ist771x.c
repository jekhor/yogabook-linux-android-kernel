/*
 * platform_hideep.c: hideep platform data initilization file
 *
 * (C) Copyright 2014 Hideep
 * Author: Kim Liao
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_ist771x.h"
#include <linux/input/hideep_ts.h>



/* Hideep changes for intel*/
int tp_connected;

static int hideep_gpio_setup(int gpio, bool configure, int dir, int state);

static struct hideep_platform_data ist7xx_board_data = {
	.version = HIDEEP_DRIVER_VERSION,
	.gpio_int = HIDEEP_INT_GPIO,
	.gpio_reset = HIDEEP_RESET_GPIO,
	.gpio_scl = HIDEEP_I2C_SCL_GPIO,
	.gpio_sda = HIDEEP_I2C_SDA_GPIO,
	.gpio_power = HIDEEP_POWER_GPIO,
 	.gpio_config = hideep_gpio_setup,
 	.power_ldo_gpio = -1,
	.max_x = 1920,
	.max_y = 1200,
	.max_z = 255,
	.max_w = 255,
	.power = NULL,
};


void *ist7xx_platform_data(void *info)
{
	return &ist7xx_board_data;

}


//setting gpio.
static int hideep_gpio_setup(int gpio, bool configure, int dir, int state)
{
	int ret = 0;
	unsigned char buf[16];

	printk("%s, gpio=%d, configure=%d, dir=%d, state=%d\n", __func__, gpio, configure, dir, state);

	if (configure) {
		snprintf(buf, PAGE_SIZE, "dsx_gpio_%u\n", gpio);

		ret = gpio_request(gpio, buf);
		
	        if (ret) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, ret);
			return ret;
		}

		if (dir == 0)
			ret = gpio_direction_input(gpio);
		else
			ret = gpio_direction_output(gpio, state);
		if (ret) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return ret;
		}
	} else {
		gpio_free(gpio);
	}

	return ret;
}

static int __init hideep_ist771x_platform_init(void)
{
    int i2c_busnum = 1;
    struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
    void *pdata = NULL;
	printk(KERN_ERR "hideep_ist771x_platform_init\n");
	
    tp_connected = 0;
    if (yeti_hw_ver == 0 )
	i2c_busnum = 4;

    memset(&i2c_info, 0, sizeof(i2c_info));
    strncpy(i2c_info.type, I2C_DRIVER_NAME, strlen(I2C_DRIVER_NAME));

    i2c_info.addr = 0x6C;

    printk(KERN_ERR "hideep_platform_init I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);

    pdata = &ist7xx_board_data;

    if(pdata != NULL)
        i2c_info.platform_data = pdata;
    else
        printk(KERN_ERR "%s, pdata is NULL\n", __func__);
#if 1
	adapter = i2c_get_adapter(i2c_busnum);
	if(adapter){
		if(i2c_new_device(adapter,&i2c_info)){
			printk(KERN_ERR "add new i2c device %s , addr 0x%x\n", I2C_DRIVER_NAME,i2c_info.addr);
			return 0;
		}else{
			printk(KERN_ERR "add new i2c device %s , addr 0x%x fail !!!\n", I2C_DRIVER_NAME,i2c_info.addr);
		}
	}else{
		printk(KERN_ERR "[%s]get adapter %d fail\n",__func__,i2c_busnum);
		return -EINVAL;
	}
#else
    return i2c_register_board_info(i2c_busnum, &i2c_info, 1);
#endif
}

device_initcall(hideep_ist771x_platform_init);


