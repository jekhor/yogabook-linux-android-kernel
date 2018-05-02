/*
 * platform_ov2740.c: ov2740 platform data initilization file
 *
 * (C) Copyright 2015 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>

#include <linux/vlv2_plat_clock.h>


#include <linux/mfd/intel_soc_pmic.h>
#include <linux/atomisp_gmin_platform.h>

#include "atomisp_gmin_pmic_regs.h"
#include "platform_ov2740.h"


/* workround - pin defined for byt */
/* GP_CAMERASB09 is NO.15, the number is 102+15 = 117
 * So, for GP_CAMERASB09 = 117 + 9 = 126
 */
#define CAMERA_1_RESET	(341+148-98) /* CAMERASB10 */
#define CAMERA_1_PWDN	(341+152-98) /* CAMERASB07 */


/* Should be defined in vlv2_plat_clock API, isn't: */
#define VLV2_CLK_19P2MHZ 0
#define VLV2_CLK_ON      1
#define VLV2_CLK_OFF     2
#define OSC_CAM0_CLK 0x0
#define OSC_CAM1_CLK 0x1


static int camera_vprog1_on;
static int camera_reset = -1;
static int camera_reset2 = 361;
static int camera_power_down = -1;
//static int camera_vcm_en = -1;





/*
 * MRFLD VV secondary camera sensor - OV2740 platform data
 */

static void ov2740_verify_gpio_power(void)
{
	printk("CAMERA: start check ov2740 gpio\n");
	printk("CAMERA_1_RESET: %d %x\n", camera_reset, gpio_get_value(camera_reset));
	printk("CAMERA_1_PWDN: %d %x\n", camera_power_down, gpio_get_value(camera_power_down));
	printk("VPROG2P8 reg:%x %x\r\n", VPROG_2P8V, intel_soc_pmic_readb(VPROG_2P8V));

	
}
static int ov2740_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{

if (flag)
		gpio_direction_output(camera_power_down, 1);
	else
		gpio_direction_output(camera_power_down, 0);

	usleep_range(1000, 1500);

	return 0;
}

static int ov2740_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	int status =0;

	printk("%s %d new clock flag=%d VLV2_CLK_19P2MHZ:%d\r\n", __func__, __LINE__, flag, VLV2_CLK_19P2MHZ);

	 if (flag)
                ret = vlv2_plat_set_clock_freq(OSC_CAM1_CLK, VLV2_CLK_19P2MHZ);
     	if(ret){
		printk("bingo...%s(): set clk freq ret is %d, .\n", __func__, ret);
		return ret;
	}
		/*Disable CAMERA 0 clock as bug in ACPI table*/
		vlv2_plat_configure_clock(OSC_CAM0_CLK, VLV2_CLK_OFF); 
        ret = vlv2_plat_configure_clock(OSC_CAM1_CLK,
                                         flag ? VLV2_CLK_ON : VLV2_CLK_OFF);
	udelay(10);

	status = vlv2_plat_get_clock_status(OSC_CAM1_CLK);
       // STATUS: 0: undefined; 1: on; 2: off;
  	printk("bingo...%s(): ret is %d, OSC_CAM1_CLK's status is %d.\n", __func__, ret, status);
	return ret;

}

static int ov2740_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	printk("%s %d flag=%d\r\n", __func__, __LINE__,flag);

	if (flag) {
		if (!camera_vprog1_on) {
			/*1. AVDD on First  V2P8SX*/
			ret = camera_set_pmic_power(CAMERA_2P8V, true); /* AVDD rising first*/
			usleep_range(2000, 2100);
			/*2. Xshutdown up*/
			gpio_direction_output(camera_reset, 1);
			gpio_direction_output(camera_reset2, 1);
			usleep_range(1000, 1500);
			/* 2. DOVDD power up*/
			ret = camera_set_pmic_power(CAMERA_1P8V, true);
			usleep_range(1000, 1500);
			/* 3. DVDD On*/

			ret = camera_set_pmic_power(CAMERA_1P2SX, true);
			usleep_range(1000, 1500);

			printk("-------verify power ON start-------\r\n");
			ov2740_verify_gpio_power();
			printk("-------verify power ON end-------\r\n");
			if (!ret) {
				/* ov2740 VDIG rise to XCLR release */
				usleep_range(1000, 1200);
				camera_vprog1_on = 1;
			}
			udelay(500);
			gpio_direction_output(391, 0);
			udelay(500);
			gpio_direction_output(391, 1);
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
			/* 1. DVDD Off*/
			ret = camera_set_pmic_power(CAMERA_1P2SX, false);
			usleep_range(1000, 1500);
			/* 2. DOVDD power off*/
			ret = camera_set_pmic_power(CAMERA_1P8V, false); 
			usleep_range(1000, 1500);
			/* 3. Xshutdown down*/
			gpio_direction_output(camera_reset, 0);
			gpio_direction_output(camera_reset2, 0);
			usleep_range(1000, 1500);			
			/*1. AVDD on First  V2P8SX*/
			ret = camera_set_pmic_power(CAMERA_2P8V, false); /* AVDD rising first*/
			usleep_range(2000, 2100);
			/*workaround to turnoff v2p8sx*/
			ret = camera_set_pmic_power(CAMERA_2P8V, false);
			ret = camera_set_pmic_power(CAMERA_1P8V, false);

			printk("-------verify power OFF start-------\r\n");
			ov2740_verify_gpio_power();
			printk("-------verify power OFF end-------\r\n");

			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}
	return ret;
}

static int ov2740_csi_configure(struct v4l2_subdev *sd, int flag)
{
	printk("%s %d flag=%d\r\n", __func__, __LINE__, flag);
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 2,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}


static int ov2740_platform_init(struct i2c_client *client)
{
	int ret;
	
	printk("yangsy %s %d\r\n", __func__, __LINE__);	
	ret = gpio_request(CAMERA_1_PWDN, "camera_1_pwdn");
	if (ret) {
		pr_err("platform ov2740 %s: failed to request PWDN gpio(pin %d) keep going\n",
			__func__, CAMERA_1_PWDN);
	}
	camera_power_down = CAMERA_1_PWDN;
	ret = gpio_direction_output(camera_power_down, 0);
	if (ret) {
			pr_err("%s shut down camera camera_power_down failed", __func__);
	}

	printk("%s %d\r\n", __func__, __LINE__);	
	ret = gpio_request(CAMERA_1_RESET, "camera_1_reset");
	if (ret) {
		pr_err("platform ov2740 %s: failed to request RST gpio(pin %d) keep going\n",
			__func__, CAMERA_1_RESET);
	}
	camera_reset = CAMERA_1_RESET;
	ret = gpio_direction_output(camera_reset, 0);
	if (ret) {
			pr_err("%s shut down camera camera_reset failed", __func__);
	}

	ret = gpio_request(camera_reset2, "camera_2_reset");
	if (ret) {
		pr_err("platform ov2740 %s: failed to request RST gpio(pin %d) keep going\n",
			__func__, camera_reset2);
	}
	ret = gpio_direction_output(camera_reset2, 0);
	printk("%s %d\r\n", __func__, __LINE__);
	/*workaround to turnoff v2p8sx*/
	ret = camera_set_pmic_power(CAMERA_2P8V, false);
	ret = camera_set_pmic_power(CAMERA_1P8V, false);
	vlv2_plat_configure_clock(OSC_CAM0_CLK, VLV2_CLK_OFF); 
	vlv2_plat_configure_clock(OSC_CAM1_CLK, VLV2_CLK_OFF); 

	return 0;
}

static int ov2740_platform_deinit(void)
{
	printk("%s %d\r\n", __func__, __LINE__);

	return 0;
}


static struct camera_sensor_platform_data ov2740_sensor_platform_data = {
	.gpio_ctrl      = ov2740_gpio_ctrl,
	.flisclk_ctrl   = ov2740_flisclk_ctrl,
	.power_ctrl     = ov2740_power_ctrl,
	.csi_cfg        = ov2740_csi_configure,
	.platform_init = ov2740_platform_init,
	.platform_deinit = ov2740_platform_deinit,
};

void *ov2740_platform_data(void *info)
{
	printk("%s %d\r\n", __func__, __LINE__);
	camera_reset = -1;
	camera_power_down = -1;

	return &ov2740_sensor_platform_data;
}

EXPORT_SYMBOL_GPL(ov2740_platform_data);

