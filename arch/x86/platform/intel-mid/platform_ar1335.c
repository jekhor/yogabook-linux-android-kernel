/*
 * platform_ar1335.c: ar1335 platform data initilization file
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
#include "platform_ar1335.h"


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
#define OSC_CAM1_CLK 0x0


static int camera_vprog1_on;
static int camera_reset = -1;
static int camera_power_down = -1;
//static int camera_vcm_en = -1;

static int log_enable = 0;



#define AR1335_PLAT_LOG(a, ...) \
       do { \
               if (log_enable) \
                       printk(a,## __VA_ARGS__); \
       } while (0)

static void ar1335_verify_gpio_power(void)
{
#if 0
       AR1335_PLAT_LOG("CAMERA: start check ar1335 gpio\n");
       AR1335_PLAT_LOG("CAMERA_0_RESET: %d %x\n", camera_reset, gpio_get_value(camera_reset));
       AR1335_PLAT_LOG("CAMERA_0_PWDN: %d %x\n", camera_power_down, gpio_get_value(camera_power_down));
       AR1335_PLAT_LOG("VPROG1P2SX:%x %x\r\n", VPROG5B,intel_soc_pmic_readb(VPROG_1P2SX));
       AR1335_PLAT_LOG("VPROG1P2SX_SEL reg:%x %x\r\n", VPROG5B_VSEL, intel_soc_pmic_readb(VPROG_1P2SX_VSEL));
       AR1335_PLAT_LOG("VPROG5B reg:%x %x\r\n", VPROG5B,intel_soc_pmic_readb(VPROG5B));
       AR1335_PLAT_LOG("VPROG5B_SEL reg:%x %x\r\n", VPROG5B_VSEL, intel_soc_pmic_readb(VPROG5B_VSEL));
       AR1335_PLAT_LOG("VPROG2P8 reg:%x %x\r\n", VPROG_2P8V, intel_soc_pmic_readb(VPROG_2P8V));
       AR1335_PLAT_LOG("CAMERA: start check ar1335 gpio\n");
       AR1335_PLAT_LOG("CAMERA_0_RESET: %d %x\n", camera_reset, gpio_get_value(camera_reset));
       AR1335_PLAT_LOG("CAMERA_0_PWDN: %d %x\n", camera_power_down, gpio_get_value(camera_power_down));
       AR1335_PLAT_LOG("VPROG1P2SX:%x %x\r\n", VPROG5B,intel_soc_pmic_readb(VPROG_1P2SX));
       AR1335_PLAT_LOG("VPROG1P2SX_SEL reg:%x %x\r\n", VPROG5B_VSEL, intel_soc_pmic_readb(VPROG_1P2SX_VSEL));
       AR1335_PLAT_LOG("VPROG5B reg:%x %x\r\n", VPROG5B,intel_soc_pmic_readb(VPROG5B));
       AR1335_PLAT_LOG("VPROG5B_SEL reg:%x %x\r\n", VPROG5B_VSEL, intel_soc_pmic_readb(VPROG5B_VSEL));
       AR1335_PLAT_LOG("VPROG2P8 reg:%x %x\r\n", VPROG_2P8V, intel_soc_pmic_readb(VPROG_2P8V));

#endif
      return;
	
}
static int ar1335_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{

	if (flag) {
	        AR1335_PLAT_LOG("%s %d flag:%d\n", __func__, __LINE__, flag);	
		gpio_direction_output(camera_power_down, 1); /* reset is active low */
		/* ar1335 reset pulse should be more than 2ms
		 */
		usleep_range(3500, 4000);

	} else {
                AR1335_PLAT_LOG("%s %d flag:%d\n", __func__, __LINE__, flag);
		gpio_direction_output(camera_power_down, 0);
		/* 1us - Falling time of REGEN after XCLR H -> L */
		//usleep_range(2000, 2100);
		usleep_range(10000, 10100);
	}

	return 0;
}

static int ar1335_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
        AR1335_PLAT_LOG("%s %d new clock flag=%d VLV2_CLK_19P2MHZ:%d\r\n", __func__, __LINE__, flag, VLV2_CLK_19P2MHZ);

	if (flag)
			ret = vlv2_plat_set_clock_freq(OSC_CAM0_CLK, VLV2_CLK_19P2MHZ);

	return vlv2_plat_configure_clock(OSC_CAM0_CLK,
									 flag ? VLV2_CLK_ON : VLV2_CLK_OFF);

}

static int ar1335_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
        
        AR1335_PLAT_LOG("%s %d flag=%d\r\n", __func__, __LINE__,flag);

	if (flag) {
		if (!camera_vprog1_on) {
			/* 1. DOVDD 1.8V  power up*/
			ret = camera_set_pmic_power(CAMERA_VPROG5B, true);
                        AR1335_PLAT_LOG("%s %d,IOVDD 1.8V  Rising\n",__func__,__LINE__);
			usleep_range(1000, 1500);

			/* 2. DVDD On*/
			ret = camera_set_pmic_power(CAMERA_1P2SX, true);
                        AR1335_PLAT_LOG("%s %d,DVDD 1.2V  Rising\n",__func__,__LINE__);
			usleep_range(1000, 1500);

			/*2. Xshutdown up,not connected*/
			//gpio_direction_output(camera_reset, 1);
			//usleep_range(1000, 1500);

			/*3. AVDD on V2P8SX*/
			ret = camera_set_pmic_power(CAMERA_2P8V, true); /* AVDD rising first*/
                        AR1335_PLAT_LOG("%s %d,AVDD 2.8V  Rising\n",__func__,__LINE__);
			usleep_range(2000, 2100);

			ret = camera_set_pmic_power(CAMERA_3P3A, true);
			usleep_range(1000, 1500);

		        AR1335_PLAT_LOG("-------verify power ON start-------\r\n");	
			ar1335_verify_gpio_power();
                        AR1335_PLAT_LOG("-------verify power ON end-------\r\n");
			if (!ret) {
				/* ar1335 VDIG rise to XCLR release */
				usleep_range(1000, 1200);
				camera_vprog1_on = 1;
			}
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
			/*1. AVDD on First  V2P8SX*/
			ret = camera_set_pmic_power(CAMERA_2P8V, false); /* AVDD rising first*/
                        AR1335_PLAT_LOG("%s %d,AVDD 2.8V OFF\n",__func__,__LINE__);
			usleep_range(2000, 2100);

			/* 2. DVDD Off*/
			ret = camera_set_pmic_power(CAMERA_1P2SX, false);
			//usleep_range(1000, 1500);
                        AR1335_PLAT_LOG("%s %d,DVDD 1.2V OFF\n",__func__,__LINE__);

			usleep_range(10000, 10100);

			/* 3. DOVDD power off*/
			ret = camera_set_pmic_power(CAMERA_VPROG5B, false); 
                        AR1335_PLAT_LOG("%s %d,IOVDD 1.8V OFF\n",__func__,__LINE__);
			usleep_range(1000, 1500);

			/* 3. Xshutdown down*/
			//gpio_direction_output(camera_reset, 0);
			//usleep_range(1000, 1500);

			/*workaround to turnoff v2p8sx*/
			ret = camera_set_pmic_power(CAMERA_2P8V, false);
			ret = camera_set_pmic_power(CAMERA_1P8V, false);

			ret = camera_set_pmic_power(CAMERA_3P3A, false);
			usleep_range(1000, 1500);
                        AR1335_PLAT_LOG("-------verify power OFF start-------\r\n");
			ar1335_verify_gpio_power();
                        AR1335_PLAT_LOG("-------verify power OFF end-------\r\n");

			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}
	return ret;
}

static int ar1335_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;

        AR1335_PLAT_LOG("%s %d flag= %d\r\n", __func__, __LINE__, flag);
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_gbrg, flag);
}


static int ar1335_platform_init(struct i2c_client *client)
{
	int ret;

        AR1335_PLAT_LOG("yangsy %s %d\r\n", __func__, __LINE__); 	
	ret = gpio_request(CAMERA_0_PWDN, "camera_1_pwdn");
	if (ret) {
		pr_err("platform ar1335 %s: failed to request PWDN gpio(pin %d) keep going\n",
			__func__, CAMERA_0_PWDN);
	}
	camera_power_down = CAMERA_0_PWDN;
	ret = gpio_direction_output(camera_power_down, 0);
	if (ret) {
			pr_err("%s shut down camera camera_power_down failed", __func__);
	}

        AR1335_PLAT_LOG("%s %d\r\n", __func__, __LINE__);
	ret = gpio_request(CAMERA_0_RESET, "camera_1_reset");
	if (ret) {
		pr_err("platform ar1335 %s: failed to request RST gpio(pin %d) keep going\n",
			__func__, CAMERA_0_RESET);
	}
	camera_reset = CAMERA_0_RESET;
	ret = gpio_direction_output(camera_reset, 0);
	if (ret) {
			pr_err("%s shut down camera camera_reset failed", __func__);
	}

        AR1335_PLAT_LOG("%s %d\r\n", __func__, __LINE__);
	/*workaround to turnoff v2p8sx*/
	ret = camera_set_pmic_power(CAMERA_2P8V, false);
	ret = camera_set_pmic_power(CAMERA_1P8V, false);
	vlv2_plat_configure_clock(OSC_CAM0_CLK, VLV2_CLK_OFF); 
	vlv2_plat_configure_clock(OSC_CAM1_CLK, VLV2_CLK_OFF); 

	return 0;
}

static int ar1335_platform_deinit(void)
{
        AR1335_PLAT_LOG("%s %d\r\n", __func__, __LINE__);
	return 0;
}


static struct camera_sensor_platform_data ar1335_sensor_platform_data = {
	.gpio_ctrl      = ar1335_gpio_ctrl,
	.flisclk_ctrl   = ar1335_flisclk_ctrl,
	.power_ctrl     = ar1335_power_ctrl,
	.csi_cfg        = ar1335_csi_configure,
	.platform_init = ar1335_platform_init,
	.platform_deinit = ar1335_platform_deinit,
};

void *ar1335_platform_data(void *info)
{
        AR1335_PLAT_LOG("%s %d\r\n", __func__, __LINE__);
	camera_reset = -1;
	camera_power_down = -1;

	return &ar1335_sensor_platform_data;
}

EXPORT_SYMBOL_GPL(ar1335_platform_data);

