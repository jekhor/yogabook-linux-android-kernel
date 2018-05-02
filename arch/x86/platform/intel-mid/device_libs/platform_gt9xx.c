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
#include <linux/i2c.h>


#define I2C_DRIVER_NAME				"gt9xx_ts"

static int __init gt9xx_platform_init(void)
{
	int i2c_busnum = 1;
	struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
	void *pdata = NULL;

	if (yeti_hw_ver == 0 )
		i2c_busnum = 4;

	memset(&i2c_info, 0, sizeof(i2c_info));
	strncpy(i2c_info.type, I2C_DRIVER_NAME, strlen(I2C_DRIVER_NAME));

	i2c_info.addr = 0x14;

	printk(KERN_ERR "gt9xx_platform_init I2C bus = %d, name = %s, irq = 0x%2x, addr = 0x%x\n",
	        i2c_busnum,
	        i2c_info.type,
	        i2c_info.irq,
	        i2c_info.addr);

	//pdata = &ist7xx_board_data;

	if(pdata != NULL)
		i2c_info.platform_data = pdata;
	else
		printk(KERN_ERR "%s, pdata is NULL\n", __func__);

	adapter = i2c_get_adapter(i2c_busnum);
	if(adapter){
	if(i2c_new_device(adapter,&i2c_info)){
		//printk(KERN_ERR "add new i2c device %s , addr 0x%x\n", I2C_DRIVER_NAME,i2c_info.addr);
		return 0;
	}else{
		printk(KERN_ERR "add new i2c device %s , addr 0x%x fail !!!\n", I2C_DRIVER_NAME,i2c_info.addr);
	}
	}else{
	printk(KERN_ERR "[%s]get adapter %d fail\n",__func__,i2c_busnum);
	return -EINVAL;
	}
}

device_initcall(gt9xx_platform_init);


