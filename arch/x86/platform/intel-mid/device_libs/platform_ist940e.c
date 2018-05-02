/*
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>

#define ISTCORE_I2C_NAME            "ist940e"
#define I2C_DRIVER_NAME				"ist940e"


int hideep_gpio_setup(int gpio, bool configure, int dir, int state)
{
	int retval = 0;
	static const char *name[] = {"hideep_gpio_305", "hideep_gpio_421"};

	//printk("%s, gpio=%d, configure=%d, dir=%d, state=%d\n", __func__, gpio, configure, dir, state);

	if (configure) {
		retval = gpio_request(gpio, gpio % 305 ? name[1] : name[0]);

		if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);

		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}
EXPORT_SYMBOL_GPL(hideep_gpio_setup);


#define VPROG3B         (0x9B)
#define VPROG3B_VSEL    (0xCB)   //3V0  /*VOUT =  0.25 + 61 * 0.05 = 3.3V*/
#define VPROG4D         (0x9F)
#define VPROG4D_VSEL    (0xCF)   //3V0  /*VOUT =  0.25 + 61 * 0.05 = 3.3V*/
#define VPROG5A         (0xA0)
#define VPROG5A_VSEL    (0xD0)   //1V8  /*VOUT =  0.25 + 31 * 0.05 = 1.8V*/

#define VPROG_ENABLE 0x2
#define VPROG_DISABLE 0x0

/*add begin by lenovo xucm1 20160506 YETIM-849 for control VPROG5A voltage */
void hideep_iovdd_switch(struct device *dev, int on)
{
	static struct regulator *iovdd_i2c = NULL;
	int ret;

	if (!iovdd_i2c) {
		iovdd_i2c = regulator_get(dev, "VPROG5A");
		if (IS_ERR(iovdd_i2c)) {
			printk("regulator get of vcc_i2c failed");
			ret = PTR_ERR(iovdd_i2c);
			iovdd_i2c = NULL;
			return;
		}
	}

	ret= regulator_set_voltage(iovdd_i2c, 1800000,1800000);
	if (ret) {
		printk( "Regulator set_vtg failed vcc_i2c rc=%d\n", ret);
		goto ERR_GET_VCC;
	}

	if (on) {
		ret = regulator_enable(iovdd_i2c);
	} else {
		ret = regulator_disable(iovdd_i2c);
	}
	if (ret<0){
		printk("Regulator enable/disable failed vcc_i2c ret=%d\n", ret);
		goto ERR_GET_VCC;
	}

	return;
ERR_GET_VCC:
	regulator_put(iovdd_i2c);
	return ;
}
/*add end by lenovo xucm1 20160506*/

void hideep_set_pmic_power(struct device *dev, bool flag)
{
   u8 reg_value[2] = {VPROG_DISABLE, VPROG_ENABLE};

   if (yeti_hw_ver >= 2 ) {
	   intel_soc_pmic_writeb(VPROG4D_VSEL, 61);
	   intel_soc_pmic_writeb(VPROG4D, reg_value[flag]);
   }else{
	   intel_soc_pmic_writeb(VPROG3B_VSEL, 61);
	   intel_soc_pmic_writeb(VPROG3B, reg_value[flag]);
   }
/*modify begin by lenovo xucm1 20160506 YETIM-849 for change VPROG5A control mode */
#if 0
   intel_soc_pmic_writeb(VPROG5A_VSEL, 31);


   intel_soc_pmic_writeb(VPROG5A, reg_value[flag]);
#else
	  hideep_iovdd_switch(dev,flag);
#endif
/*modify end by lenovo xucm1 20160506*/

  // printk("%s:ist940 enble V3P3 and V1P8 for touch,yeti_hw_ver=%d\n", __func__,yeti_hw_ver);

}
EXPORT_SYMBOL_GPL(hideep_set_pmic_power);

struct hideep_platform_data
{
    u32                         version;
    u32                         gpio_int;
    u32                         max_x;
    u32                         max_y;
    u32                         max_z;
    u32                         max_w;
    void                        (*power)(bool on);
};

static struct hideep_platform_data hideep_pdata = {
	.max_x = 1200,
	.max_y = 1920,
	.max_z = 255,
	.max_w = 255, // ice cream "max_area"
	.power = hideep_set_pmic_power,
};

static int __init hideep_platform_init(void)
{
    int i2c_busnum = 5;
    struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
    void *pdata = NULL;
	printk(KERN_ERR "hideep_platform_init\n");

    memset(&i2c_info, 0, sizeof(i2c_info));
    strncpy(i2c_info.type, I2C_DRIVER_NAME, strlen(I2C_DRIVER_NAME));

    i2c_info.addr = 0x6C;

    printk(KERN_ERR "hideep_platform_init I2C bus = %d, name = %s, irq = 0x%2x, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);

    pdata = &hideep_pdata;

    if(pdata != NULL)
        i2c_info.platform_data = pdata;
    else
        printk(KERN_ERR "%s, pdata is NULL\n", __func__);
#if 1
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
#else
    return i2c_register_board_info(i2c_busnum, &i2c_info, 1);
#endif
}

device_initcall(hideep_platform_init);

