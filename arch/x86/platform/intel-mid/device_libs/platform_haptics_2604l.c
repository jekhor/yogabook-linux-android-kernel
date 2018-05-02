/*
** =============================================================================
** Copyright (c) 2014  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** File:
**     board-mako-haptics.c
**
** Description:
**     platform data for Haptics devices
**
** =============================================================================
*/

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#define CONFIG_HAPTICS_DRV2604L

#include <linux/gpio.h>
#include <linux/haptic/drv2604l.h>

#define APQ8064_GSBI3_QUP_I2C_BUS_ID            3

#if defined(CONFIG_HAPTICS_DRV2604L)

#define gpio_southwest_NUM	98
#define gpio_north_NUM	73
#define gpio_east_NUM	27
#define gpio_southeast_NUM	86

#define gpio_southwest_base	(ARCH_NR_GPIOS-gpio_southwest_NUM)
#define gpio_north_base		(gpio_southwest_base - gpio_north_NUM)
#define gpio_east_base			(gpio_north_base - gpio_east_NUM)
#define gpio_southeast_base	(gpio_east_base - gpio_southeast_NUM)

#define VB1_EN	79//SW79 bl37 i2c0
#define VB3_EN	47//N47 k28 i2c3

#define GPIO_VB1_EN	(gpio_southwest_base + VB1_EN)
#define GPIO_VB3_EN	(gpio_north_base + VB3_EN)

static struct DRV2604L_platform_data  drv2604l_plat_data_right = {
	.gpio_enable = GPIO_VB1_EN,			//enable the chip
	.gpio_trigger = 0,			//external trigger pin, (0: internal trigger)
#if defined(CONFIG_HAPTICS_LRA_SEMCO1030)
	//rated = 1.5Vrms, ov=2.1Vrms, f=204hz
	.loop = CLOSE_LOOP,
	.RTPFormat = Signed,
	.BIDIRInput = BiDirectional,
	.actuator = {
		.device_type = LRA,
		.rated_vol = 0x3d,
		.over_drive_vol = 0x87,
		.LRAFreq = 204,
	},
#elif defined(CONFIG_HAPTICS_ERM_EVOWAVE_Z4TJGB1512658)
	//rated vol = 3.0 v, ov = 3.6 v, risetime = 150 ms
	.loop = CLOSE_LOOP,
	.RTPFormat = Signed,
	.BIDIRInput = BiDirectional,	
	.actuator = {
		.device_type = ERM,		
		.rated_vol = 0x8d,
		.over_drive_vol = 0xa9,
	},
#else
#error "please define actuator type"
#endif	
};

static struct DRV2604L_platform_data  drv2604l_plat_data_left = {
	.gpio_enable = GPIO_VB3_EN,			//enable the chip
	.gpio_trigger = 0,			//external trigger pin, (0: internal trigger)
#if defined(CONFIG_HAPTICS_LRA_SEMCO1030)
	//rated = 1.5Vrms, ov=2.1Vrms, f=204hz
	.loop = CLOSE_LOOP,
	.RTPFormat = Signed,
	.BIDIRInput = BiDirectional,
	.actuator = {
		.device_type = LRA,
		.rated_vol = 0x3d,
		.over_drive_vol = 0x87,
		.LRAFreq = 204,
	},
#elif defined(CONFIG_HAPTICS_ERM_EVOWAVE_Z4TJGB1512658)
	//rated vol = 3.0 v, ov = 3.6 v, risetime = 150 ms
	.loop = CLOSE_LOOP,
	.RTPFormat = Signed,
	.BIDIRInput = BiDirectional,	
	.actuator = {
		.device_type = ERM,		
		.rated_vol = 0x8d,
		.over_drive_vol = 0xa9,
	},
#else
#error "please define actuator type"
#endif	
};
#endif

static int __init platform_add_i2c_haptics_device(void)
{
	int i2c_busnum = 0;
	struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;

	memset(&i2c_info, 0, sizeof(i2c_info));
	strncpy(i2c_info.type, HAPTICS_DEVICE_NAME, strlen(HAPTICS_DEVICE_NAME));
	i2c_info.addr = 0x5a;

	//vb1 en/right/bus0
	i2c_info.platform_data = &drv2604l_plat_data_right;
	i2c_busnum = 0;
	adapter = i2c_get_adapter(i2c_busnum);
	if(adapter){	
		if(i2c_new_device(adapter, &i2c_info)){
			pr_info("%s i2c-%d add device, addr: 0x%x, name: %s [OK]\n", __func__, i2c_busnum , i2c_info.addr, HAPTICS_DEVICE_NAME);
		}
		else{
			pr_err("%s i2c-%d add device, addr: 0x%x, name: %s [FAILED]\n", __func__, i2c_busnum , i2c_info.addr, HAPTICS_DEVICE_NAME);
			return -ENODEV;
		}
	}
	else{
		pr_err("[%s]get adapter %d fail\n",__func__, i2c_busnum);
		return -EBUSY;
	}

	//vb3 en/left/bus3
	i2c_info.platform_data = &drv2604l_plat_data_left;
	i2c_busnum = 3;
	adapter = i2c_get_adapter(i2c_busnum);
	if(adapter){
		if(i2c_new_device(adapter, &i2c_info)){
			pr_info("%s i2c-%d add device, addr: 0x%x, name: %s [OK]\n", __func__, i2c_busnum , i2c_info.addr, HAPTICS_DEVICE_NAME);
		}
		else{
			pr_err("%s i2c-%d add device, addr: 0x%x, name: %s [FAILED]\n", __func__, i2c_busnum , i2c_info.addr, HAPTICS_DEVICE_NAME);
			return -ENODEV;
		}
	}
	else{
		pr_err("[%s]get adapter %d fail\n",__func__, i2c_busnum);
		return -EBUSY;
	}
	return 0;
}

static int __init platform_add_haptics_device(void)
{
	return platform_add_i2c_haptics_device();
}

device_initcall(platform_add_haptics_device);
