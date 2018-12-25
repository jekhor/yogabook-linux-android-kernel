/*
 * platform_bq27541.c: bq27541 initilization file
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <asm/platform_sst_audio.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/platform_device.h>
#include <linux/sfi.h>
#include <asm/intel-mid.h>
#include <linux/power/bq27x00_battery.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/power/bq2589x_reg.h>

#define BQ27541_MAIN_SLAVE_ADDR 0x55
//#define BQ27541_SEC_SLAVE_ADDR  0x55
#define BQ27541_MAIN_I2C_MASTER 1
#define BQ27541_SEC_I2C_MASTER  0

/*
 * Translate temperatures to compensate for thermistor circuit problem (EVT2).
 * Precision is limited but should provide accurate values.
 *
 * FIXME: reference code, need change
 */
static int bq27541_translate_temp(int temperature)
{
	if (temperature <= 980)
		/*
		 * This indicates that the thermistor is disconnected on EVT2. Return the same
		 * value as would be measured on EVT3 in case of missing thermistor so that the
		 * gas gauge driver can recognize the disconnected state.
		 */
		temperature = -408;
	else if (temperature <= 986)
		temperature = 0;
	else if (temperature <= 989)
		temperature = 50;
	else if (temperature <= 993)
		temperature = 100;
	else if (temperature <= 998)
		temperature = 150;
	else if (temperature <= 1003)
		temperature = 200;
	else if (temperature <= 1009)
		temperature = 250;
	else if (temperature <= 1016)
		temperature = 300;
	else if (temperature <= 1024)
		temperature = 350;
	else if (temperature <= 1044)
		temperature = 400;
	else if (temperature <= 1055)
		temperature = 450;
	else if (temperature <= 1067)
		temperature = 500;
	else if (temperature <= 1080)
		temperature = 550;
	else if (temperature <= 1095)
		temperature = 600;
	else if (temperature <= 1116)
		temperature = 650;
	else
		temperature = 700;

	return temperature;
}

static struct bq27x00_platform_data bq27541_main_platform_data = {
	.soc_int_irq = -1,
	.bat_low_irq = -1,
};

static struct i2c_board_info __initdata bq27541_main_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("bq27500", BQ27541_MAIN_SLAVE_ADDR),
		.platform_data = &bq27541_main_platform_data,
	},
};

static int __init bq27541_platform_init(void)
{
	//int soc_int_gpio, soc_int_irq=0;
	struct i2c_adapter *adapter;
	struct i2c_adapter *adapter_slave;
	
	//soc_int_gpio = get_gpio_by_name("max_fg_alert");
	//soc_int_gpio = 148; // GPIOS_18,102 + 28 + 18
	//soc_int_irq = gpio_to_irq(soc_int_gpio);

	//printk("%s: gpio = %d, irq = %d, i2C = %d\n",__func__,soc_int_gpio, soc_int_irq, BQ27541_MAIN_I2C_MASTER);

	//res = irq_set_irq_wake(soc_int_irq, 0);
	//if (res) {
	//	pr_err("%s: Failed to set irq wake for soc_int: %d\n", __func__, res);
	//	return 0;
	//}

	//bq27541_main_platform_data.soc_int_irq = soc_int_irq;
	//bq27541_platform_data.translate_temp = bq27541_translate_temp;
#if 1
	adapter = i2c_get_adapter(BQ27541_MAIN_I2C_MASTER);
	if(adapter){
		if(i2c_new_device(adapter, &bq27541_main_i2c_boardinfo[0])){
			printk(KERN_ERR "add i2c device: main FG bq27500, I2C%d, addr 0x%x\n", BQ27541_MAIN_I2C_MASTER, BQ27541_MAIN_SLAVE_ADDR);
			return 0;
		}else{
			printk(KERN_ERR "add i2c device %s, addr 0x%x fail !!!\n", "bq27500", BQ27541_MAIN_SLAVE_ADDR);
		}
	}else{
		printk(KERN_ERR "[%s]get adapter %d fail ! ! !\n",__func__, BQ27541_MAIN_I2C_MASTER);
		return -EINVAL;
	}
/*
	adapter_slave = i2c_get_adapter(BQ27541_SEC_I2C_MASTER);
	if(adapter_slave){
		if(i2c_new_device(adapter_slave, &bq27541_main_i2c_boardinfo[0])){
			printk(KERN_ERR "add i2c device: slave FG bq27500, I2C%d, addr 0x%x\n", BQ27541_SEC_I2C_MASTER, BQ27541_MAIN_SLAVE_ADDR);
			return 0;
		}else{
			printk(KERN_ERR "add i2c device %s, addr 0x%x fail !!!\n", "bq27500", BQ27541_MAIN_SLAVE_ADDR);
		}
	}else{
		printk(KERN_ERR "[%s]get adapter %d fail ! ! !\n",__func__, BQ27541_SEC_I2C_MASTER);
		return -EINVAL;
	}
*/	
#else
	res = i2c_register_board_info(BQ27541_MAIN_I2C_MASTER, &bq27541_main_i2c_boardinfo, ARRAY_SIZE(bq27541_main_i2c_boardinfo));
	if(res < 0){
		pr_err("bq27541_platform_init: fail register bq27541 i2c device\n");
	}
#endif	
	return 0;
}

/* charger bq25892 device init */
#define BQ25892_SEC_SLAVE_ADDR  0x6B
#define BQ25892_SEC_I2C_MASTER  0

static struct bq2589x_platform_data bq25892_data_sec = {
	.gpio_irq = 346,   /* GPIO_DFX4 */
	.gpio_ce  = 388,   /* GPIO_CAMERASB08 */
	.gpio_otg = 247,     /* null: conect to VSYS2 */  //liulc3

	.enable_watchdog = 1,
	.watchdog_timeout = 40,   /* only valid if watchdog timer is enabled */
//	.enable_charge_timer =
	.charge_timeout = 12,     /* only valid if fast charge timer is enabled */
	.boost_frequency = 1500,  /* 1.5MHz */
	.boost_voltage = 4998,
	.boost_ilimit = 1300,     /* 1300mA */
	.ir_comp_resistance = 0,
	.ir_comp_vclamp = 32,     /* default */
	.thermal_regulation_threshold = 120,    /* default */
	.jeita_vset = 150,        /* default */
	.jeita_iset = 20,         /* default */
	.sys_min_voltage = 3500,
//	.enable_termination =
	.termination_current = 256,   /* 256mA default */
	.precharge_voltage = 3000,
	.recharge_threshold = 100,
//	.enable_ilimit_pin =
	.vindpm_offset = 600,
	.force_vindpm = 1,

/* add bq25892_charge_param init data here */
};

static struct i2c_board_info __initdata bq25892_sec_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("bq25892-second", BQ25892_SEC_SLAVE_ADDR),
		.platform_data = &bq25892_data_sec,
	},
};

static int __init bq25892_platform_init(void)
{
	struct i2c_adapter *adapter;
	adapter = i2c_get_adapter(BQ25892_SEC_I2C_MASTER);
	if(adapter){
		if(i2c_new_device(adapter, &bq25892_sec_i2c_boardinfo[0])){
			printk(KERN_ERR "add i2c device: second charger bq25892, I2C%d, addr 0x%x\n", BQ25892_SEC_I2C_MASTER, BQ25892_SEC_SLAVE_ADDR);
			return 0;
		}else{
			printk(KERN_ERR "add i2c device %s, addr 0x%x fail !!!\n", "bq25892", BQ25892_SEC_SLAVE_ADDR);
		}
	}else{
		printk(KERN_ERR "[%s]get adapter %d fail ! ! !\n",__func__, BQ25892_SEC_I2C_MASTER);
		return -EINVAL;
	}
}

static int __init charger_fuel_gauge_init(void)
{
	bq27541_platform_init();
	bq25892_platform_init();
}

//fs_initcall(bq27541_platform_init);
device_initcall(charger_fuel_gauge_init);

