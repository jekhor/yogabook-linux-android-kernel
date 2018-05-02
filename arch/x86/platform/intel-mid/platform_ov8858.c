/*
 * platform_ov8858.c: ov8858 platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
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
#include "platform_ov8858.h"


/* workround - pin defined for byt */
/* GP_CAMERASB09 is NO.15, the number is 102+15 = 117
 * So, for GP_CAMERASB09 = 117 + 9 = 126
 */
#define CAMERA_0_RESET (341+52)	/* GP_CAMERASB09, Low to reset */
#define CAMERA_0_PWDN (341+45)	/* GP_CAMERASB05, Low to power down*/


/* Should be defined in vlv2_plat_clock API, isn't: */
#define VLV2_CLK_19P2MHZ 0
#define VLV2_CLK_ON      1
#define VLV2_CLK_OFF     2
#define OSC_CAM0_CLK 0x0
#define OSC_CAM1_CLK 0x1


static int camera_vprog1_on;
static int camera_reset = -1;
static int camera_power_down = -1;
//static int camera_vcm_en = -1;





/*
 * MRFLD VV primary camera sensor - OV8858 platform data
 */

static void ov8858_verify_gpio_power(void)
{
	printk("CAMERA: start check ov8858 gpio\n");
	printk("CAMERA_0_RESET: %d %x\n", camera_reset, gpio_get_value(camera_reset));
	printk("CAMERA_0_PWDN: %d %x\n", camera_power_down, gpio_get_value(camera_power_down));
	printk("VPROG1P2SX:%x %x\r\n", VPROG5B,intel_soc_pmic_readb(VPROG_1P2SX));
	printk("VPROG1P2SX_SEL reg:%x %x\r\n", VPROG5B_VSEL, intel_soc_pmic_readb(VPROG_1P2SX_VSEL));
	printk("VPROG5B reg:%x %x\r\n", VPROG5B,intel_soc_pmic_readb(VPROG5B));
	printk("VPROG5B_SEL reg:%x %x\r\n", VPROG5B_VSEL, intel_soc_pmic_readb(VPROG5B_VSEL));
	printk("VPROG2P8 reg:%x %x\r\n", VPROG_2P8V, intel_soc_pmic_readb(VPROG_2P8V));

	
}
static int ov8858_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{

	if (flag) {
		printk("%s %d flag:%d\n", __func__, __LINE__, flag);
		gpio_direction_output(camera_reset,1);
		/* ov8858 reset pulse should be more than 2ms
		 */
		usleep_range(3500, 4000);

	} else {
		printk("%s %d flag:%d\n", __func__, __LINE__, flag);
		gpio_direction_output(camera_reset,0);
		/* 1us - Falling time of REGEN after XCLR H -> L */
		usleep_range(2000, 2100);
	}

	return 0;
}

static int ov8858_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	int status = 0;


	printk("%s %d new clock flag=%d VLV2_CLK_19P2MHZ:%d\r\n", __func__, __LINE__, flag, VLV2_CLK_19P2MHZ);

	if (flag)
		ret = vlv2_plat_set_clock_freq(OSC_CAM0_CLK, VLV2_CLK_19P2MHZ);
	if(ret){
		printk("bingo...%s(): set clk freq ret is %d, .\n", __func__, ret);
		return ret;
	}

	ret=vlv2_plat_configure_clock(OSC_CAM0_CLK,
									 flag ? VLV2_CLK_ON : VLV2_CLK_OFF);
	udelay(10);

	status = vlv2_plat_get_clock_status(OSC_CAM0_CLK);
       // STATUS: 0: undefined; 1: on; 2: off;
    	printk("bingo...%s(): ret is %d, OSC_CAM0_CLK's status is %d.\n", __func__, ret, status);

    	return ret;


}

static int ov8858_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	printk("%s %d flag=%d\r\n", __func__, __LINE__,flag);

	if (flag) {
		if (!camera_vprog1_on) {
			/*1. AVDD on First  V2P8SX*/
			ret = camera_set_pmic_power(CAMERA_2P8V, true); /* AVDD rising first*/
			usleep_range(2000, 2100);
			/*2. Xshutdown up*/
			gpio_direction_output(camera_power_down,1);
			usleep_range(1000, 1500);
			/* 2. DOVDD power up  V1P8SX*/
			ret = camera_set_pmic_power(CAMERA_1P8V, true);
			usleep_range(1000, 1500);
			/* 3. DVDD On*/

			ret = camera_set_pmic_power(CAMERA_1P2SX, true);
			usleep_range(1000, 1500);

			ret = camera_set_pmic_power(CAMERA_3P3A, true);
			usleep_range(1000, 1500);

			printk("-------verify power ON start-------\r\n");
			ov8858_verify_gpio_power();
			printk("-------verify power ON end-------\r\n");
			if (!ret) {
				/* ov8858 VDIG rise to XCLR release */
				usleep_range(1000, 1200);
				camera_vprog1_on = 1;
			}
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
			/* 1. DVDD Off*/
			ret = camera_set_pmic_power(CAMERA_1P2SX, false);
			usleep_range(1000, 1500);
			/* 2. DOVDD power off V1P8SX*/
			ret = camera_set_pmic_power(CAMERA_1P8V, false);
			usleep_range(1000, 1500);
			/* 3. Xshutdown down*/
			gpio_direction_output(camera_power_down,0);
			usleep_range(1000, 1500);			
			/*1. AVDD on First  V2P8SX*/
			ret = camera_set_pmic_power(CAMERA_2P8V, false); /* AVDD rising first*/
			usleep_range(2000, 2100);
			/*workaround to turnoff v2p8sx*/
			ret = camera_set_pmic_power(CAMERA_2P8V, false);
			ret = camera_set_pmic_power(CAMERA_1P8V, false);

			ret = camera_set_pmic_power(CAMERA_3P3A, false);
			usleep_range(1000, 1500);

			printk("-------verify power OFF start-------\r\n");
			ov8858_verify_gpio_power();
			printk("-------verify power OFF end-------\r\n");

			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}
	return ret;
}

static int ov8858_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;

	printk("%s %d flag= %d\r\n", __func__, __LINE__, flag);
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}


static int ov8858_platform_init(struct i2c_client *client)
{
	int ret;
	
	printk("yangsy %s %d\r\n", __func__, __LINE__);	
	ret = gpio_request(CAMERA_0_PWDN, "camera_1_pwdn");
	if (ret) {
		pr_err("platform ov8858 %s: failed to request PWDN gpio(pin %d) keep going\n",
			__func__, CAMERA_0_PWDN);
	}
	camera_power_down = CAMERA_0_PWDN;
	ret = gpio_direction_output(camera_power_down, 0);
	if (ret) {
			pr_err("%s shut down camera camera_power_down failed", __func__);
	}

	printk("%s %d\r\n", __func__, __LINE__);	
	ret = gpio_request(CAMERA_0_RESET, "camera_1_reset");
	if (ret) {
		pr_err("platform ov8858 %s: failed to request RST gpio(pin %d) keep going\n",
			__func__, CAMERA_0_RESET);
	}
	camera_reset = CAMERA_0_RESET;
	ret = gpio_direction_output(camera_reset, 0);
	if (ret) {
			pr_err("%s shut down camera camera_reset failed", __func__);
	}

	printk("%s %d\r\n", __func__, __LINE__);
	/*workaround to turnoff v2p8sx*/
	ret = camera_set_pmic_power(CAMERA_2P8V, false);
	ret = camera_set_pmic_power(CAMERA_1P8V, false);
	vlv2_plat_configure_clock(OSC_CAM0_CLK, VLV2_CLK_OFF); 
	vlv2_plat_configure_clock(OSC_CAM1_CLK, VLV2_CLK_OFF); 

	return 0;
}

static int ov8858_platform_deinit(void)
{
	printk("%s %d\r\n", __func__, __LINE__);

	return 0;
}


static struct camera_sensor_platform_data ov8858_sensor_platform_data = {
	.gpio_ctrl      = ov8858_gpio_ctrl,
	.flisclk_ctrl   = ov8858_flisclk_ctrl,
	.power_ctrl     = ov8858_power_ctrl,
	.csi_cfg        = ov8858_csi_configure,
	.platform_init = ov8858_platform_init,
	.platform_deinit = ov8858_platform_deinit,
};

void *ov8858_platform_data(void *info)
{
	printk("%s %d\r\n", __func__, __LINE__);
	camera_reset = -1;
	camera_power_down = -1;

	return &ov8858_sensor_platform_data;
}

EXPORT_SYMBOL_GPL(ov8858_platform_data);

