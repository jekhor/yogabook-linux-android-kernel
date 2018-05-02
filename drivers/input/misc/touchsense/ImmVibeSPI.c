/*
** =============================================================================
**
** File: ImmVibeSPI.c
**
** Description:
**	   Device-dependent functions called by Immersion TSP API
**	   to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
**
** Copyright (c) 2012-2013 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
**
** =============================================================================
*/
//#warning ********* Compiling SPI for DRV2604 using LRA actuator ************

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/slab.h>
#include <linux/types.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>

#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/pwm.h>

#include <linux/workqueue.h>

struct gpio_desc *gpiod_enable;
struct gpio_desc *gpiod_enable1;

/*
** Copy ImmVibeSPI.c, autocal.seq, and init.seq from
** actuator subdirectory into the same directory as tspdrv.c
*/

/*
** Enable workqueue to allow DRV2604 time to brake
*/
#define GUARANTEE_AUTOTUNE_BRAKE_TIME  0

/*
** Enable to use DRV2604 EN pin to enter standby mode
*/
#define USE_DRV2604_EN_PIN	0

/*
** gpio connected to DRV2604 EN pin
*/
#define GPIO_AMP_EN  0x00

/*
** Enable to use DRV2604 i2c command to enter standby mode
*/
#define USE_DRV2604_STANDBY 1

/*
** Address of our device
*/
#define DEVICE_ADDR 0x5A

/*
** i2c bus that it sits on
*/
#define DEVICE_BUS	0
#define DEVICE_BUS1  2

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS 2

/*
** Name of the DRV2604 board
*/
#define DRV2604_BOARD_NAME	 "DRV2604"
#define DRV2605_BOARD_NAME	 "DRV2605"

#define ERM_INDEX 0
#define LRA_INDEX 1

/*
** Go
*/
#define GO_REG 0x0C
#define GO	   0x01
#define STOP   0x00

/*
** Status
*/
#define STATUS_REG			0x00
#define STATUS_DEFAULT		0x00

#define DIAG_RESULT_MASK	(1 << 3)
#define AUTO_CAL_PASSED		(0 << 3)
#define AUTO_CAL_FAILED		(1 << 3)
#define DIAG_GOOD			(0 << 3)
#define DIAG_BAD			(1 << 3)

#define DEV_ID_MASK (7 << 5)
#define DRV2605 (3 << 5) /* per DRV2604 datasheet */
#define DRV2604L (6 << 5)
#define DRV2604 (4 << 5)

/*
** Mode
*/
#define MODE_REG			0x01
#define MODE_STANDBY		0x40
#define MODE_DEVICE_READY	0x00

#define DRV2604_MODE_MASK			0x07
#define MODE_INTERNAL_TRIGGER		0
#define MODE_REAL_TIME_PLAYBACK		5
#define MODE_DIAGNOSTICS			6
#define AUTO_CALIBRATION			7

#define MODE_RESET					0x80

/*
** Real Time Playback
*/
#define REAL_TIME_PLAYBACK_REG		0x02

/*
** Library Selection
*/
#define LIBRARY_SELECTION_REG		0x03
#define LIBRARY_SELECTION_DEFAULT	0x00

/*
** Waveform Sequencer
*/
#define WAVEFORM_SEQUENCER_REG		0x04
#define WAVEFORM_SEQUENCER_REG2		0x05
#define WAVEFORM_SEQUENCER_REG3		0x06
#define WAVEFORM_SEQUENCER_REG4		0x07
#define WAVEFORM_SEQUENCER_REG5		0x08
#define WAVEFORM_SEQUENCER_REG6		0x09
#define WAVEFORM_SEQUENCER_REG7		0x0A
#define WAVEFORM_SEQUENCER_REG8		0x0B
#define WAVEFORM_SEQUENCER_MAX		8
#define WAVEFORM_SEQUENCER_DEFAULT	0x00

/*
** OverDrive Time Offset
*/
#define OVERDRIVE_TIME_OFFSET_REG  0x0D

/*
** Sustain Time Offset, postive
*/
#define SUSTAIN_TIME_OFFSET_POS_REG 0x0E

/*
** Sustain Time Offset, negative
*/
#define SUSTAIN_TIME_OFFSET_NEG_REG 0x0F

/*
** Brake Time Offset
*/
#define BRAKE_TIME_OFFSET_REG		0x10

/*
** Rated Voltage
*/
#define RATED_VOLTAGE_REG			0x16

/*
** Overdrive Clamp Voltage
*/
#define OVERDRIVE_CLAMP_VOLTAGE_REG 0x17

/*
** Auto Calibrationi Compensation Result
*/
#define AUTO_CALI_RESULT_REG		0x18

/*
** Auto Calibration Back-EMF Result
*/
#define AUTO_CALI_BACK_EMF_RESULT_REG 0x19

/*
** Feedback Control
*/
#define FEEDBACK_CONTROL_REG		0x1A

#define FEEDBACK_CONTROL_BEMF_LRA_GAIN0 0 /* 5x */
#define FEEDBACK_CONTROL_BEMF_LRA_GAIN1 1 /* 10x */
#define FEEDBACK_CONTROL_BEMF_LRA_GAIN2 2 /* 20x */
#define FEEDBACK_CONTROL_BEMF_LRA_GAIN3 3 /* 30x */

#define LOOP_RESPONSE_SLOW		(0 << 2)
#define LOOP_RESPONSE_MEDIUM	(1 << 2) /* default */
#define LOOP_RESPONSE_FAST		(2 << 2)
#define LOOP_RESPONSE_VERY_FAST (3 << 2)

#define FB_BRAKE_FACTOR_1X	 (0 << 4) /* 1x */
#define FB_BRAKE_FACTOR_2X	 (1 << 4) /* 2x */
#define FB_BRAKE_FACTOR_3X	 (2 << 4) /* 3x (default) */
#define FB_BRAKE_FACTOR_4X	 (3 << 4) /* 4x */
#define FB_BRAKE_FACTOR_6X	 (4 << 4) /* 6x */
#define FB_BRAKE_FACTOR_8X	 (5 << 4) /* 8x */
#define FB_BRAKE_FACTOR_16X  (6 << 4) /* 16x */
#define FB_BRAKE_DISABLED	 (7 << 4)

#define FEEDBACK_CONTROL_MODE_ERM 0 /* default */
#define FEEDBACK_CONTROL_MODE_LRA (1 << 7)

/*
** Control1
*/
#define Control1_REG			0x1B

#define STARTUP_BOOST_ENABLED	(1 << 7)
#define STARTUP_BOOST_DISABLED	(0 << 7) /* default */

/*
** Control2
*/
#define Control2_REG			0x1C

#define IDISS_TIME_MASK			0x03
#define IDISS_TIME_VERY_SHORT	0
#define IDISS_TIME_SHORT		1
#define IDISS_TIME_MEDIUM		2 /* default */
#define IDISS_TIME_LONG			3

#define BLANKING_TIME_MASK			0x0C
#define BLANKING_TIME_VERY_SHORT	(0 << 2)
#define BLANKING_TIME_SHORT			(1 << 2)
#define BLANKING_TIME_MEDIUM		(2 << 2) /* default */
#define BLANKING_TIME_VERY_LONG		(3 << 2)

#define AUTO_RES_GAIN_MASK		   0x30
#define AUTO_RES_GAIN_VERY_LOW	   (0 << 4)
#define AUTO_RES_GAIN_LOW		   (1 << 4)
#define AUTO_RES_GAIN_MEDIUM	   (2 << 4) /* default */
#define AUTO_RES_GAIN_HIGH		   (3 << 4)

#define SOFT_BRAKE_MASK			   0x40
#define SOFT_BRAKE				   (1 << 6)

#define BIDIR_INPUT_MASK		   0x80
#define UNIDIRECT_INPUT			   (0 << 7)
#define BIDIRECT_INPUT			   (1 << 7) /* default */

/*
** Control3
*/
#define Control3_REG 0x1D

#define ERM_OpenLoop_Enabled (1 << 5)
#define NG_Thresh_1 (1 << 6)
#define NG_Thresh_2 (2 << 6)
#define NG_Thresh_3 (3 << 6)

/*
** Auto Calibration Memory Interface
*/
#define AUTOCAL_MEM_INTERFACE_REG	0x1E

#define AUTOCAL_TIME_150MS			(0 << 4)
#define AUTOCAL_TIME_250MS			(1 << 4)
#define AUTOCAL_TIME_500MS			(2 << 4)
#define AUTOCAL_TIME_1000MS			(3 << 4)

#define SILICON_REVISION_REG		0x3B
#define SILICON_REVISION_MASK		0x07

#define DEFAULT_DRIVE_TIME		0x17

#define MAX_AUTOCALIBRATION_ATTEMPT 2
#define SKIP_AUTOCAL		1
#define GO_BIT_POLL_INTERVAL	15

#define MAX_REVISION_STRING_SIZE 20

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH 1
#endif
#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW 0
#endif

static int g_nDeviceID = -1,tempindex = 0;
static struct i2c_client* g_pTheClient = NULL;
static struct i2c_client* g_pTheClient2 = NULL;
static bool g_bAmpEnabled[2] = {false,false};

static const unsigned char init_sequenceerm[] = {
MODE_REG, MODE_INTERNAL_TRIGGER,
REAL_TIME_PLAYBACK_REG, 0x00,
LIBRARY_SELECTION_REG, 0x00,
WAVEFORM_SEQUENCER_REG, 0x01,
WAVEFORM_SEQUENCER_REG2, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG3, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG4, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG5, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG6, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG7, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG8, WAVEFORM_SEQUENCER_DEFAULT,
GO_REG, STOP,
OVERDRIVE_TIME_OFFSET_REG, 0x00,
SUSTAIN_TIME_OFFSET_POS_REG, 0x00,
SUSTAIN_TIME_OFFSET_NEG_REG, 0x00,
BRAKE_TIME_OFFSET_REG, 0x00,
RATED_VOLTAGE_REG, 0x52,
OVERDRIVE_CLAMP_VOLTAGE_REG, 0x99,
AUTO_CALI_RESULT_REG, 0xC,
AUTO_CALI_BACK_EMF_RESULT_REG, 0x6F,
FEEDBACK_CONTROL_REG, 0xB6,
Control1_REG, 0x93,
Control2_REG, 0xF5,
Control3_REG, 0x80,
AUTOCAL_MEM_INTERFACE_REG, 0x30,
};

static const unsigned char init_sequencelra[] = {
MODE_REG, MODE_INTERNAL_TRIGGER,
REAL_TIME_PLAYBACK_REG, 0x00,
LIBRARY_SELECTION_REG, 0x00,
WAVEFORM_SEQUENCER_REG, 0x01,
WAVEFORM_SEQUENCER_REG2, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG3, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG4, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG5, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG6, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG7, WAVEFORM_SEQUENCER_DEFAULT,
WAVEFORM_SEQUENCER_REG8, WAVEFORM_SEQUENCER_DEFAULT,
GO_REG, STOP,
OVERDRIVE_TIME_OFFSET_REG, 0x00,
SUSTAIN_TIME_OFFSET_POS_REG, 0x00,
SUSTAIN_TIME_OFFSET_NEG_REG, 0x00,
BRAKE_TIME_OFFSET_REG, 0x00,
RATED_VOLTAGE_REG, 0x52,
OVERDRIVE_CLAMP_VOLTAGE_REG, 0x99,
AUTO_CALI_RESULT_REG, 0xC,
AUTO_CALI_BACK_EMF_RESULT_REG, 0x6F,
FEEDBACK_CONTROL_REG, 0xB6,
Control1_REG, 0x93,
Control2_REG, 0xF5,
Control3_REG, 0x80,
AUTOCAL_MEM_INTERFACE_REG, 0x30
};

#if SKIP_AUTOCAL == 0
static const unsigned char autocal_sequence[] = {
#include <autocal.seq>
};
#endif

#ifndef ACTUATOR_NAME
#define ACTUATOR_NAME "AWA"
#endif


extern struct DRV2604L_data *Imm_pDrv2604ldata[2];

extern int drv2604l_reg_read(struct DRV2604L_data *pDrv2604ldata, unsigned int reg);
/*{
	unsigned int val;
	int ret;
	
	ret = regmap_read(pDrv2604ldata->regmap, reg, &val);
	
	if (ret < 0)
		return ret;
	else
		return val;
}*/

extern int drv2604l_reg_write(struct DRV2604L_data *pDrv2604ldata, unsigned char reg, char val);
/*{
	return regmap_write(pDrv2604ldata->regmap, reg, val);
}*/


static void drv2604_write_reg_val(const unsigned char* data, unsigned int size, int index)
{
	int i = 0;

	if (size % 2 != 0)
		return;

	/*if (index == 0){
		while (i < size)
		{
			//i2c_smbus_write_byte_data(g_pTheClient, data[i], data[i+1]);
			i2c_master_send(g_pTheClient2, &data[i], 2);
			i+=2;
		}
	} else if (index == 1) {
		while (i < size)
		{
			//i2c_smbus_write_byte_data(g_pTheClient2, data[i], data[i+1]);
			i2c_master_send(g_pTheClient, &data[i], 2);
			i+=2;
		}
	}*/

	if(index>1 ||Imm_pDrv2604ldata[index] == NULL )
	   return;

	while (i < size)
	{
	   // i2c_smbus_write_byte_data(g_pTheClient, data[i], data[i+1]);

		//printk("current actuator index = %d \n", index);
	drv2604l_reg_write(Imm_pDrv2604ldata[index], data[i],  data[i+1]);
		  i+=2;
	}
}

static unsigned char drv2604_read_reg(unsigned char reg, int index)
{
	/*unsigned char buffer = '0';
	if (index == 0) {
		if( 1!= i2c_master_send(g_pTheClient2,&reg,1) ) {
			printk( KERN_ERR "drv2605_i2c_read 0 fail! \n" );
			return buffer;
		}
		if( 1!= i2c_master_recv(g_pTheClient2,&buffer,1) ) {
			printk( KERN_ERR "drv2605_i2c_read 0 fail! \n" );
			return buffer;
		}
	} else if (index == 1) {
		if( 1!= i2c_master_send(g_pTheClient,&reg,1) ) {
			printk( KERN_ERR "drv2605_i2c_read 1 fail! \n" );
			return buffer;
		}
		if( 1!= i2c_master_recv(g_pTheClient,&buffer,1) ) {
			printk( KERN_ERR "drv2605_i2c_read 1 fail! \n" );
			return buffer;
		}
	}
	return buffer;*/

	 if(index>1 ||Imm_pDrv2604ldata[index] == NULL )
		return 0;

	return	drv2604l_reg_read(Imm_pDrv2604ldata[index], reg);
}

#if SKIP_AUTOCAL == 0
static void drv2604_poll_go_bit(void)
{
	while (drv2604_read_reg(GO_REG) == GO)
	  schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
}
#endif

static void drv2604_set_rtp_val(char value, int index)
{
	char rtp_val[] =
	{
		REAL_TIME_PLAYBACK_REG, value
	};
	drv2604_write_reg_val(rtp_val, sizeof(rtp_val), index);
}

static void drv2604_change_mode(char mode, int index)
{
	unsigned char tmp[] =
	{
		MODE_REG, mode
	};
	drv2604_write_reg_val(tmp, sizeof(tmp), index);
}

static ssize_t vibr_enableV1_store(struct device *dev,
								struct device_attribute *attr, const char *buf,
								size_t size)
{
	int err = 0;
	int enable = 0;

	err = sscanf(buf, "%d", &enable);
	if (err < 0) {
		printk("Enable V1 failed: %d\n", err);
		goto exit;
	}

	if (enable == 1) {
		drv2604_change_mode(MODE_DEVICE_READY, 0);
		usleep_range(1000, 1000);

		/* Workaround for power issue in the DRV2604 */
		/* Restore the register settings if they have reset to the defaults */
		if((drv2604_read_reg(RATED_VOLTAGE_REG, 0) != init_sequencelra[5]))
		{
			drv2604_write_reg_val(init_sequencelra, sizeof(init_sequencelra), 0);
		}
		drv2604_change_mode(MODE_REAL_TIME_PLAYBACK, 0);
		drv2604_set_rtp_val(127, 0);
	}
	else if (enable == 0) {
		//ImmVibeSPI_ForceOut_AmpDisable(0);
		drv2604_set_rtp_val(0, 0);
		drv2604_change_mode(MODE_STANDBY, 0);
	}
	//printk("%s, vib_enableV1_store enable = %d \n", __FUNCTION__, enable);

exit:
		return size;
}

static ssize_t vibr_enableV2_store(struct device *dev,
								struct device_attribute *attr, const char *buf,
								size_t size)
{
	int err = 0;
	int enable = 0;

	err = sscanf(buf, "%d", &enable);
	if (err < 0) {
		printk("Enable V2 failed: %d\n", err);
		goto exit;
	}

	if (enable == 1) {
		drv2604_change_mode(MODE_DEVICE_READY, 1);
		usleep_range(1000, 1000);

		/* Workaround for power issue in the DRV2604 */
		/* Restore the register settings if they have reset to the defaults */
		if((drv2604_read_reg(RATED_VOLTAGE_REG, 1) != init_sequencelra[5]))
		{
			drv2604_write_reg_val(init_sequencelra, sizeof(init_sequencelra), 1);
		}
		drv2604_change_mode(MODE_REAL_TIME_PLAYBACK, 1);
		drv2604_set_rtp_val(127, 1);
		//drv2604_set_rtp_val(0xff, 1);
	} else if (enable == 0) {
		drv2604_set_rtp_val(0, 1);
		drv2604_change_mode(MODE_STANDBY, 1);
		//drv2604_set_rtp_val(0, 1);
	}

	//printk("%s, vib_enableV2_store enable = %d \n", __FUNCTION__, enable);

exit:
		return size;
}

static DEVICE_ATTR(enableV1, S_IRUGO | S_IWUSR, NULL, vibr_enableV1_store);
static DEVICE_ATTR(enableV2, S_IRUGO | S_IWUSR, NULL, vibr_enableV2_store);

static struct attribute *vibr_attrs[] = {
		&dev_attr_enableV1.attr,
		&dev_attr_enableV2.attr,
		NULL,
};

static const struct attribute_group vibr_attr_group = {
		.attrs = vibr_attrs,
};

int drv2604_vibra_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	acpi_handle handle = ACPI_HANDLE(dev);
	struct acpi_device *device;
	const struct acpi_device_id *acpi_id;
	struct gpio_desc *gpiod;
	int ret;

	ret = acpi_bus_get_device(handle, &device);
	if (ret) {
		printk("could not get acpi device - %d\n", ret);
	}

	if (!ACPI_HANDLE(dev))
		printk("drv2604 ACPI HANDLE fail \n");
	acpi_id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!acpi_id)
		printk("drv2604 failed to get ACPI info \n");

	gpiod = devm_gpiod_get_index(dev, "drv2604_gpio_enable", 0);
	if (IS_ERR(gpiod)) {
		int errrr = PTR_ERR(gpiod);
		printk("get drv2604_gpio_enable failed: %d \n", errrr);
	}
	gpiod_enable = gpiod;

	gpiod = devm_gpiod_get_index(dev, "drv2604_gpio_enable1", 1);
	if (IS_ERR(gpiod)) {
		int errrr = PTR_ERR(gpiod);
		printk("get drv2604_gpio_enable1 failed: %d \n", errrr);
	}
	gpiod_enable1 = gpiod;

	gpiod_direction_output(gpiod_enable1, 1);

	ret = sysfs_create_group(&pdev->dev.kobj, &vibr_attr_group);
	if (ret) {
		printk("drv2604 create sysfs failed: %d \n", ret);
	}

	//printk("drv2604_vibra_probe success \n");
	return 0;
}

#if USE_DRV2604_EN_PIN
static void drv2604_set_en(bool enabled)
{
	gpio_direction_output(GPIO_AMP_EN, enabled ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
}
#endif
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex);
static void drv2604_shutdown(struct i2c_client* client);
static int drv2604_probe(struct i2c_client* client, const struct i2c_device_id* id);
static int drv2604_remove(struct i2c_client* client);
static const struct i2c_device_id drv2604_id[] =
{
	{DRV2604_BOARD_NAME, 0},
	{},
};

#if 0
static struct i2c_board_info info = {
	I2C_BOARD_INFO(DRV2604_BOARD_NAME, DEVICE_ADDR),
};
#endif

static struct i2c_driver drv2604_driver =
{
	.probe = drv2604_probe,
	.remove = drv2604_remove,
	.id_table = drv2604_id,
	.shutdown = drv2604_shutdown,
	.driver =
	{
		.name = DRV2604_BOARD_NAME,
	},
};

static void drv2605_shutdown(struct i2c_client* client);
static int drv2605_probe(struct i2c_client* client, const struct i2c_device_id* id);
static int drv2605_remove(struct i2c_client* client);
static const struct i2c_device_id drv2605_id[] =
{
	{DRV2605_BOARD_NAME, 0},
	{},
};

#if 0
static struct i2c_board_info info1 = {
	I2C_BOARD_INFO(DRV2605_BOARD_NAME, DEVICE_ADDR),
};
#endif

static struct i2c_driver drv2605_driver =
{
	.probe = drv2605_probe,
	.remove = drv2605_remove,
	.id_table = drv2605_id,
	.shutdown = drv2605_shutdown,
	.driver =
	{
		.name = DRV2605_BOARD_NAME,
	},
};
#if 0
static struct acpi_device_id drv2604_acpi_match[] = {
	{ "DRV2604", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, drv2604_acpi_match);

static struct platform_driver drv2604_vibra_driver = {
	.driver = {
		.name = "drv2604_vibra",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(drv2604_acpi_match),
	},
	.probe = drv2604_vibra_probe,
};
#endif

static int drv2604_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	char status;
#if SKIP_AUTOCAL == 0
	int nCalibrationCount = 0;
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		DbgOut((DBL_ERROR, "drv2604 probe failed"));
		return -ENODEV;
	}
	if (g_pTheClient2 == NULL) {
		g_pTheClient2 = client;
		tempindex = 0;
	} else {
		g_pTheClient = client;
		tempindex = 1;
	}

	gpiod_direction_output(gpiod_enable, 1);
#if USE_DRV2604_EN_PIN
	gpio_request(GPIO_AMP_EN, "vibrator-en");
	drv2604_set_en(true);
#endif
#if USE_DRV2604_STANDBY
	drv2604_change_mode(MODE_DEVICE_READY, ERM_INDEX);
#endif
	/* Wait 1000 us for chip power to stabilize */
	usleep_range(1000, 1000);

#if SKIP_AUTOCAL
	drv2604_write_reg_val(init_sequencelra, sizeof(init_sequencelra), ERM_INDEX);
	status = drv2604_read_reg(STATUS_REG, ERM_INDEX);
#else
	/* Run auto-calibration */
	do{
		drv2604_write_reg_val(autocal_sequence, sizeof(autocal_sequence), ERM_INDEX);

		/* Wait until the procedure is done */
		drv2604_poll_go_bit();

		/* Read status */
		status = drv2604_read_reg(STATUS_REG, ERM_INDEX);

		nCalibrationCount++;

	} while (((status & DIAG_RESULT_MASK) == AUTO_CAL_FAILED) && (nCalibrationCount < MAX_AUTOCALIBRATION_ATTEMPT));

	/* Check result */
	if ((status & DIAG_RESULT_MASK) == AUTO_CAL_FAILED)
	{
	  DbgOut((DBL_ERROR, "drv2604 auto-calibration failed after %d attempts.\n", nCalibrationCount));
	}
	else
	{
		/* Read calibration results */
		drv2604_read_reg(AUTO_CALI_RESULT_REG, ERM_INDEX);
		drv2604_read_reg(AUTO_CALI_BACK_EMF_RESULT_REG, ERM_INDEX);
		drv2604_read_reg(FEEDBACK_CONTROL_REG, ERM_INDEX);
	}
#endif

	/* Read device ID */
	g_nDeviceID = (status & DEV_ID_MASK);
	switch (g_nDeviceID)
	{
		case DRV2605:
			DbgOut((DBL_INFO, "drv2604 driver found: drv2605.\n"));
			break;
		case DRV2604L:
			DbgOut((DBL_INFO, "drv2604 driver found: drv2604L.\n"));
			break;
		case DRV2604:
			DbgOut((DBL_INFO, "drv2604 driver found: drv2604.\n"));
			break;
		default:
			DbgOut((DBL_INFO, "drv2604 driver found: unknown.\n"));
			break;
	}

#if USE_DRV2604_STANDBY
	/* Put hardware in standby */
	drv2604_change_mode(MODE_STANDBY, ERM_INDEX);
#elif USE_DRV2604_EN_PIN
	/* enable RTP mode that will be toggled on/off with EN pin */
#endif

#if USE_DRV2604_EN_PIN
	/* turn off chip */
	drv2604_set_en(false);
#endif

	DbgOut((DBL_INFO, "drv2604 probe succeeded"));
	return 0;
}

static void drv2604_shutdown(struct i2c_client* client)
{

	ImmVibeSPI_ForceOut_AmpDisable(0);

	DbgOut((DBL_ERROR, "drv2604_shutdown.\n"));
	return;
}


static int drv2604_remove(struct i2c_client* client)
{
	DbgOut((DBL_VERBOSE, "drv2604_remove.\n"));
	return 0;
}

static int drv2605_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	char status;
#if SKIP_AUTOCAL == 0
	int nCalibrationCount = 0;
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		DbgOut((DBL_ERROR, "drv2605 probe failed"));
		return -ENODEV;
	}
	if (g_pTheClient2 == NULL) {
		g_pTheClient2 = client;
		tempindex = 0;
	}
	else {
		g_pTheClient = client;
		tempindex = 1;
	}

#if USE_DRV2604_STANDBY
	drv2604_change_mode(MODE_DEVICE_READY, LRA_INDEX);
#endif
	usleep_range(1000, 1000);

#if SKIP_AUTOCAL
	drv2604_write_reg_val(init_sequencelra, sizeof(init_sequencelra), LRA_INDEX);
	status = drv2604_read_reg(STATUS_REG, LRA_INDEX);
#endif
	g_nDeviceID = (status & DEV_ID_MASK);
	switch (g_nDeviceID)
	{
		case DRV2605:
			DbgOut((DBL_INFO, "drv2604 driver found: drv2605.\n"));
			break;
		case DRV2604:
			DbgOut((DBL_INFO, "drv2604 driver found: drv2604.\n"));
			break;
		default:
			DbgOut((DBL_INFO, "drv2604 driver found: unknown.\n"));
			break;
	}

#if USE_DRV2604_STANDBY
	drv2604_change_mode(MODE_STANDBY, LRA_INDEX);
#elif USE_DRV2604_EN_PIN
#endif

#if USE_DRV2604_EN_PIN
	drv2604_set_en(false);
#endif

	DbgOut((DBL_INFO, "drv2604 probe succeeded"));

  return 0;
}

static void drv2605_shutdown(struct i2c_client* client)
{

	ImmVibeSPI_ForceOut_AmpDisable(1);

	DbgOut((DBL_ERROR, "drv2605_shutdown\n"));
	return;
}


static int drv2605_remove(struct i2c_client* client)
{
	DbgOut((DBL_VERBOSE, "drv2605_remove.\n"));
	return 0;
}

#if GUARANTEE_AUTOTUNE_BRAKE_TIME

#define AUTOTUNE_BRAKE_TIME 25

static VibeInt8 g_lastForce = 0;
static bool g_brake = false;

static void autotune_brake_complete(struct work_struct *work)
{
	/* new nForce value came in before workqueue terminated */
	if (g_lastForce > 0)
		return;

#if USE_DRV2604_STANDBY
	/* Put hardware in standby */
	//drv2604_change_mode(MODE_STANDBY, ERM_INDEX);
#endif

#if USE_DRV2604_EN_PIN
	drv2604_set_en(false);
#endif
}

DECLARE_DELAYED_WORK(g_brake_complete, autotune_brake_complete);

static struct workqueue_struct *g_workqueue;

#endif

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
	int tindex = nActuatorIndex;
	//printk("ImmVibeSPI_ForceOut_AmpDisable.\n");

	if (g_bAmpEnabled[nActuatorIndex])
	{
		DbgOut((DBL_VERBOSE, "ImmVibeSPI_ForceOut_AmpDisable.\n"));
		

		/* Set the force to 0 */
		drv2604_set_rtp_val(0, tindex);

#if GUARANTEE_AUTOTUNE_BRAKE_TIME
		/* if a brake signal arrived from daemon, let the chip stay on
		 * extra time to allow it to brake */
		if (g_brake && g_workqueue)
		{
			queue_delayed_work(g_workqueue,
							   &g_brake_complete,
							   msecs_to_jiffies(AUTOTUNE_BRAKE_TIME));
		}
		else /* disable immediately (smooth effect style) */
#endif
		{
#if USE_DRV2604_STANDBY
			/* Put hardware in standby via i2c */
			drv2604_change_mode(MODE_STANDBY, tindex);
#endif

#if USE_DRV2604_EN_PIN
			/* Disable hardware via pin */
			drv2604_set_en(false);//????????????????????????
#endif
		}
		//g_bAmpEnabled = false;
		g_bAmpEnabled[nActuatorIndex] = false;
	}
	return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
	int tindex = nActuatorIndex;
			//printk("ImmVibeSPI_ForceOut_AmpEnable.\n");

	if (!g_bAmpEnabled[nActuatorIndex])
	{
		DbgOut((DBL_VERBOSE, "ImmVibeSPI_ForceOut_AmpEnable.\n"));

#if GUARANTEE_AUTOTUNE_BRAKE_TIME
		cancel_delayed_work_sync(&g_brake_complete);
#endif

#if USE_DRV2604_EN_PIN
		drv2604_set_en(true);
#endif

#if USE_DRV2604_STANDBY
		drv2604_change_mode(MODE_DEVICE_READY, tindex);
#endif
		// Chip requires minimum 250us power-up delay before RTP value can be set
		// If the chip is powered on <10ms after powering off, it needs 1000us
		// for the internal LDO voltage to stabilize
		usleep_range(1000, 1000);

		/* Workaround for power issue in the DRV2604 */
		/* Restore the register settings if they have reset to the defaults */
		if(tindex == ERM_INDEX && (drv2604_read_reg(RATED_VOLTAGE_REG, ERM_INDEX) != init_sequencelra[5]))
		{
			drv2604_write_reg_val(init_sequencelra, sizeof(init_sequencelra), ERM_INDEX);
		} else if (tindex == LRA_INDEX && (drv2604_read_reg(RATED_VOLTAGE_REG, LRA_INDEX) != init_sequencelra[5])) {
			drv2604_write_reg_val(init_sequencelra, sizeof(init_sequencelra), LRA_INDEX);
		}

		drv2604_change_mode(MODE_REAL_TIME_PLAYBACK, tindex);
		g_bAmpEnabled[nActuatorIndex] = true;
	}
	return VIBE_S_SUCCESS;
}

/*
** Called at initialization time.
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
//	struct i2c_adapter* adapter;
//	struct i2c_client* client;

	//DbgOut((DBL_VERBOSE, "ImmVibeSPI_ForceOut_Initialize.\n"));
	//printk("ImmVibeSPI_ForceOut_Initialize.\n");

#if 0
	int ret = 0;
	ret = platform_driver_register(&drv2604_vibra_driver);
	if (ret)
		printk("drv2604 Platform register failed\n");

	adapter = i2c_get_adapter(DEVICE_BUS1);
	//i2c_register_board_info(DEVICE_BUS1, &info, 1);

	if (adapter) {
		client = i2c_new_device(adapter, &info);

		if (client) {
			int retVal = i2c_add_driver(&drv2604_driver);

			if (retVal) {
				printk("drv2604 ImmVibeSPI_ForceOut_Initialize val error\n");
				return VIBE_E_FAIL;
			}

		} else {
			DbgOut((DBL_VERBOSE, "drv2604: Cannot create new device.\n"));
			return VIBE_E_FAIL;
		}

	} else {
		DbgOut((DBL_VERBOSE, "ImmVibeSPI_ForceOut_Initialize. i2c fail adapter\n"));
		return VIBE_E_FAIL;
	}

	adapter = i2c_get_adapter(DEVICE_BUS);

	if (adapter) {
		client = i2c_new_device(adapter, &info1);

		if (client) {
			int retVal = i2c_add_driver(&drv2605_driver);

			if (retVal) {
				printk("2605 ImmVibeSPI_ForceOut_Initialize val error\n");
				return VIBE_E_FAIL;
			}

		} else {
			DbgOut((DBL_VERBOSE, "drv2605: Cannot create new device.\n"));
			return VIBE_E_FAIL;
		}

	} else {
		DbgOut((DBL_VERBOSE, "2605 ImmVibeSPI_ForceOut_Initialize. i2c fail adapter\n"));
		return VIBE_E_FAIL;
	}
	#endif

#if GUARANTEE_AUTOTUNE_BRAKE_TIME
	g_workqueue = create_workqueue("tspdrv_workqueue");
#endif
	//printk("ImmVibeSPI_ForceOut_Initialize success \n");
	return VIBE_S_SUCCESS;
}

/*
** Called at termination time to disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
	DbgOut((DBL_VERBOSE, "ImmVibeSPI_ForceOut_Terminate.\n"));
	//printk("ImmVibeSPI_ForceOut_Terminate.\n");

#if GUARANTEE_AUTOTUNE_BRAKE_TIME
	if (g_workqueue)
	{
		destroy_workqueue(g_workqueue);
		g_workqueue = 0;
	}
#endif

	ImmVibeSPI_ForceOut_AmpDisable(0);
	ImmVibeSPI_ForceOut_AmpDisable(1);

	/* Remove TS5000 driver */
	i2c_del_driver(&drv2604_driver);
	i2c_del_driver(&drv2605_driver);

	return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set the force
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
	int tindex = nActuatorIndex;
	 //printk("set sample");
#if GUARANTEE_AUTOTUNE_BRAKE_TIME
	VibeInt8 force = pForceOutputBuffer[0];
	if (force > 0 && g_lastForce <= 0)
	{
		g_brake = false;

		ImmVibeSPI_ForceOut_AmpEnable(nActuatorIndex);
	}
	else if (force <= 0 && g_lastForce > 0)
	{
		g_brake = force < 0;

		ImmVibeSPI_ForceOut_AmpDisable(nActuatorIndex);
	}

	if (g_lastForce != force)
	{
		/* AmpDisable sets force to zero, so need to here */
		if (force > 0)
			drv2604_set_rtp_val(pForceOutputBuffer[0], tindex);

		g_lastForce = force;
	}
#else
	drv2604_set_rtp_val(pForceOutputBuffer[0], tindex);
#endif

	return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
	if (nActuatorIndex != 0) return VIBE_S_SUCCESS;

	switch (nFrequencyParameterID)
	{
		case VIBE_KP_CFG_FREQUENCY_PARAM1:
			/* Update frequency parameter 1 */
			break;

		case VIBE_KP_CFG_FREQUENCY_PARAM2:
			/* Update frequency parameter 2 */
			break;

		case VIBE_KP_CFG_FREQUENCY_PARAM3:
			/* Update frequency parameter 3 */
			break;

		case VIBE_KP_CFG_FREQUENCY_PARAM4:
			/* Update frequency parameter 4 */
			break;

		case VIBE_KP_CFG_FREQUENCY_PARAM5:
			/* Update frequency parameter 5 */
			break;

		case VIBE_KP_CFG_FREQUENCY_PARAM6:
			/* Update frequency parameter 6 */
			break;
	}
	return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
//	char szRevision[MAX_REVISION_STRING_SIZE];

	if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

	DbgOut((DBL_VERBOSE, "ImmVibeSPI_Device_GetName.\n"));
	   //printk( "ImmVibeSPI_Device_GetName.\n");

	/*switch (g_nDeviceID)
	{
		case DRV2605:*/
			strncpy(szDevName, "DRV2604X2", nSize-1);
			/*break;
		case DRV2604:
			strncpy(szDevName, "DRV2604", nSize-1);
			break;
		default:
			strncpy(szDevName, "Unknown", nSize-1);
			break;
	}*/

	/* Append revision number to the device name */
	//sprintf(szRevision, "r%d %s", (drv2604_read_reg(SILICON_REVISION_REG) & SILICON_REVISION_MASK), ACTUATOR_NAME);
	//if ((strlen(szRevision) + strlen(szDevName)) < nSize-1)
	//	  strcat(szDevName, szRevision);

	szDevName[nSize - 1] = '\0'; /* make sure the string is NULL terminated */
	return VIBE_S_SUCCESS;
}
