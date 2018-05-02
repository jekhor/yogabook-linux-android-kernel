/*! \file sx9310.c
 * \brief  SX9310 Driver
 *
 * Driver for the SX9310 
 * Copyright (c) 2011 Semtech Corp
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */
//#define DEBUG
#define DRIVER_NAME "sx9310"

#define MAX_WRITE_ARRAY_SIZE 32
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>

#include <linux/input/smtc/misc/sx86xx.h> /* main struct, interrupt,init,pointers */
#include <linux/input/smtc/misc/sx9310_i2c_reg.h>
#include <linux/input/smtc/misc/sx9310_platform_data.h>  /* platform data */

#define IDLE 0
#define ACTIVE 1

#if 0
/*GPIO define*/
#define	gpio_southwest_NUM 98
#define	gpio_north_NUM 73
#define	gpio_east_NUM 27
#define	gpio_southeast_NUM 86

#define	gpio_southwest_base (ARCH_NR_GPIOS-gpio_southwest_NUM) //414
#define	gpio_north_base (gpio_southwest_base - gpio_north_NUM) //341
#define	gpio_east_base (gpio_north_base - gpio_east_NUM) //314
#define	gpio_southeast_base (gpio_east_base - gpio_southeast_NUM)//228

#define	MF_ISH_GPIO_7 16
#define	MF_ISH_GPIO_2 24
#define	MF_ISH_GPIO_8 23


/* IO Used for NIRQ */
#define	GPIO_SX9310_NIRQ (gpio_east_base + MF_ISH_GPIO_7)
#define	GPIO_SX9310_SARSTATE (gpio_east_base + MF_ISH_GPIO_2)//tp3110
#define	GPIO_SX9310_RST (gpio_east_base + MF_ISH_GPIO_8) //337
#endif

#define LOOP_COUNT		5
#define INTERVAL		20 * 1000
#define INTERVAL_FAR	5  * 1000
#define CALI_LOOP
//#define USE_KERNEL_SUSPEND

//psx86XX_t this = 0;
//static bool sar_enable = true;

#ifdef CALI_LOOP
//static int cali_count = 0;
//struct timer_list cali_timer;
struct workqueue_struct *cali_workqueue = NULL;
//struct work_struct cali_work;
#endif

static u8 get_default_value_of_reg(psx86XX_t this, u8 reg);
static void set_default_value_of_reg(psx86XX_t this, u8 reg, u8 val);

/*! \struct sx9310
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx9310
{
	pbuttonInformation_t pbuttonInformation;
	psx9310_platform_data_t hw; /* specific platform data settings */
} sx9310_t, *psx9310_t;


/*! \fn static int write_register(psx86XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx86XX_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;
	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
	if (this && this->bus) {
		i2c = this->bus;

		returnValue = i2c_master_send(i2c,buffer,2);
		dev_dbg(&i2c->dev,"write_register Address: 0x%x Value: 0x%x Return: %d\n",
		address,value,returnValue);
	}
	return returnValue;
}

/*! \fn static int read_register(psx86XX_t this, u8 address, u8 *value) 
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct 
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to 
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx86XX_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;
	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c,address);
		dev_dbg(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} else {
			return returnValue;
		}
	}
	return -ENOMEM;
}
/*! \brief Sends a write register range to the device
 * \param this Pointer to main parent struct 
 * \param reg 8-bit register address (base address)
 * \param data pointer to 8-bit register values
 * \param size size of the data pointer
 * \return Value from i2c_master_send
 */
static int __attribute__((unused)) write_registerEx(psx86XX_t this, unsigned char reg,
				unsigned char *data, int size)
{
  struct i2c_client *i2c = 0;
	u8 tx[MAX_WRITE_ARRAY_SIZE];
	int ret = 0;

  if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
  {
	dev_dbg(this->pdev, "inside write_registerEx()\n");
	tx[0] = reg;
	dev_dbg(this->pdev, "going to call i2c_master_send(0x%p, 0x%x ",
			(void *)i2c,tx[0]);
	for (ret = 0; ret < size; ret++)
	{
	  tx[ret+1] = data[ret];
	  dev_dbg(this->pdev, "0x%x, ",tx[ret+1]);
	}
	dev_dbg(this->pdev, "\n");

	ret = i2c_master_send(i2c, tx, size+1 );
	  if (ret < 0)
		dev_err(this->pdev, "I2C write error\n");
  }
  dev_dbg(this->pdev, "leaving write_registerEx()\n");


	return ret;
}
/*! \brief Reads a group of registers from the device
* \param this Pointer to main parent struct 
* \param reg 8-Bit address to read from (base address)
* \param data Pointer to 8-bit value array to save registers to 
* \param size size of array
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int __attribute__((unused)) read_registerEx(psx86XX_t this, unsigned char reg,
				unsigned char *data, int size)
{
  struct i2c_client *i2c = 0;
	int ret = 0;
	u8 tx[] = {
		reg
	};
  if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
  {
	dev_dbg(this->pdev, "inside read_registerEx()\n");
	dev_dbg(this->pdev,
		"going to call i2c_master_send(0x%p,0x%p,1) Reg: 0x%x\n",
															   (void *)i2c,(void *)tx,tx[0]);
	ret = i2c_master_send(i2c,tx,1);
	if (ret >= 0) {
	  dev_dbg(this->pdev, "going to call i2c_master_recv(0x%p,0x%p,%x)\n",
															  (void *)i2c,(void *)data,size);
		ret = i2c_master_recv(i2c, data, size);
	}
  }
	if (unlikely(ret < 0))
		dev_err(this->pdev, "I2C read error\n");
  dev_dbg(this->pdev, "leaving read_registerEx()\n");
	return ret;
}
/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct 
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx86XX_t this)
{
	s32 returnValue = 0;
	dev_info( this->pdev, "Performing manual_offset_calibration()\n");
	returnValue = write_register(this,SX9310_IRQSTAT_REG,0xFF);
	return returnValue;
}
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx86XX_t this = dev_get_drvdata(dev);

	dev_dbg(this->pdev, "Reading IRQSTAT_REG\n");
	read_register(this,SX9310_IRQSTAT_REG,&reg_value);
	return sprintf(buf, "%d\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	psx86XX_t this = dev_get_drvdata(dev);
	unsigned long val;
	s32 returnValue = 0;

	if (strict_strtoul(buf, 0, &val))
		return -EINVAL;

	if (val) {
		// if the previous state is touched, then need to keep touched state
		if (this->pre_state == ACTIVE) {
			this->ignore_once_info_modem= true;
			dev_info( this->pdev, "we should ignore this change state\n");
		}
		returnValue = manual_offset_calibration(this);
		if (returnValue >0) {
			count = returnValue;
		}
		else
			dev_err( this->pdev, "Performing manual_offset_calibration() FAIL!!!\n");
	}
	return count;
}

static ssize_t reg_dump_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx86XX_t this = dev_get_drvdata(dev);
	char * p = buf;

	if (this->read_flag) {
		this->read_flag = 0;
		read_register(this, this->read_reg, &reg_value);
		p += sprintf(p, "%02x\n", reg_value);
		return (p-buf);
	}

	read_register(this, SX9310_IRQ_ENABLE_REG, &reg_value);
	p += sprintf(p, "ENABLE(0x%02x)=0x%02x\n", SX9310_IRQ_ENABLE_REG, reg_value);

	read_register(this, SX9310_IRQFUNC_REG, &reg_value);
	p += sprintf(p, "IRQFUNC(0x%02x)=0x%02x\n", SX9310_IRQFUNC_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL0_REG, &reg_value);
	p += sprintf(p, "CTRL0(0x%02x)=0x%02x\n", SX9310_CPS_CTRL0_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL1_REG, &reg_value);
	p += sprintf(p, "CTRL1(0x%02x)=0x%02x\n", SX9310_CPS_CTRL1_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL2_REG, &reg_value);
	p += sprintf(p, "CTRL2(0x%02x)=0x%02x\n", SX9310_CPS_CTRL2_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL3_REG, &reg_value);
	p += sprintf(p, "CTRL3(0x%02x)=0x%02x\n", SX9310_CPS_CTRL3_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL4_REG, &reg_value);
	p += sprintf(p, "CTRL4(0x%02x)=0x%02x\n", SX9310_CPS_CTRL4_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL5_REG, &reg_value);
	p += sprintf(p, "CTRL5(0x%02x)=0x%02x\n", SX9310_CPS_CTRL5_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL6_REG, &reg_value);
	p += sprintf(p, "CTRL6(0x%02x)=0x%02x\n", SX9310_CPS_CTRL6_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL7_REG, &reg_value);
	p += sprintf(p, "CTRL7(0x%02x)=0x%02x\n", SX9310_CPS_CTRL7_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL8_REG, &reg_value);
	p += sprintf(p, "CTRL8(0x%02x)=0x%02x\n", SX9310_CPS_CTRL8_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL9_REG, &reg_value);
	p += sprintf(p, "CTRL9(0x%02x)=0x%02x\n", SX9310_CPS_CTRL9_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL10_REG, &reg_value);
	p += sprintf(p, "CTRL10(0x%02x)=0x%02x\n", SX9310_CPS_CTRL10_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL11_REG, &reg_value);
	p += sprintf(p, "CTRL11(0x%02x)=0x%02x\n", SX9310_CPS_CTRL11_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL12_REG, &reg_value);
	p += sprintf(p, "CTRL12(0x%02x)=0x%02x\n", SX9310_CPS_CTRL12_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL13_REG, &reg_value);
	p += sprintf(p, "CTRL13(0x%02x)=0x%02x\n", SX9310_CPS_CTRL13_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL14_REG, &reg_value);
	p += sprintf(p, "CTRL14(0x%02x)=0x%02x\n", SX9310_CPS_CTRL14_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL15_REG, &reg_value);
	p += sprintf(p, "CTRL15(0x%02x)=0x%02x\n", SX9310_CPS_CTRL15_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL16_REG, &reg_value);
	p += sprintf(p, "CTRL16(0x%02x)=0x%02x\n", SX9310_CPS_CTRL16_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL17_REG, &reg_value);
	p += sprintf(p, "CTRL17(0x%02x)=0x%02x\n", SX9310_CPS_CTRL17_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL18_REG, &reg_value);
	p += sprintf(p, "CTRL18(0x%02x)=0x%02x\n", SX9310_CPS_CTRL18_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL19_REG, &reg_value);
	p += sprintf(p, "CTRL19(0x%02x)=0x%02x\n", SX9310_CPS_CTRL19_REG, reg_value);

	read_register(this, SX9310_SAR_CTRL0_REG, &reg_value);
	p += sprintf(p, "SCTRL0(0x%02x)=0x%02x\n", SX9310_SAR_CTRL0_REG, reg_value);

	read_register(this, SX9310_SAR_CTRL1_REG, &reg_value);
	p += sprintf(p, "SCTRL1(0x%02x)=0x%02x\n", SX9310_SAR_CTRL1_REG, reg_value);

	read_register(this, SX9310_SAR_CTRL2_REG, &reg_value);
	p += sprintf(p, "SCTRL2(0x%02x)=0x%02x\n", SX9310_SAR_CTRL2_REG, reg_value);

	reg_value = gpio_get_value(this->nirq_gpio);
	p += sprintf(p, "NIRQ=%d\n", reg_value);

	return (p-buf);
}

static ssize_t reg_dump_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	psx86XX_t this = dev_get_drvdata(dev);
	unsigned int val,reg,opt;

	if (sscanf(buf, "%x,%x,%x", &reg, &val, &opt) == 3) {
		this->read_reg = *((u8*)&reg);
		this->read_flag = 1;
	} else if (sscanf(buf, "%x,%x", &reg, &val) == 2) {
		dev_info( this->pdev, "%s,reg = 0x%02x, val = 0x%02x\n", __FUNCTION__, *(u8*)&reg, *(u8*)&val);
		write_register(this, *((u8*)&reg), *((u8*)&val));
		set_default_value_of_reg(this, *((u8*)&reg), *((u8*)&val));
	}

	return count;
}

static ssize_t touch_status_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx86XX_t this = dev_get_drvdata(dev);

	read_register(this, SX9310_STAT0_REG, &reg_value);
	dev_dbg(this->pdev, "Reading SX9310_STAT0_REG = %d\n", reg_value);

	reg_value &= 0x0f;

	return sprintf(buf, "0x%02x\n", reg_value);
}

static ssize_t sx9310_enable_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	psx86XX_t this = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", this->sar_enable ? "enable" : "disable" );
}

static ssize_t sx9310_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	unsigned long val;
	psx86XX_t this = dev_get_drvdata(dev);
	u8 reg_val = 0x50;

	if (strict_strtoul(buf, 0, &val))
		return -EINVAL;

	if (val<0 || val>7)
		return -EINVAL;

	if (val == 0) {
		dev_info( this->pdev, "sx9310 manual disable\n");
		if (timer_pending(&this->cali_timer))
			del_timer_sync(&this->cali_timer);
		write_register(this, SX9310_CPS_CTRL0_REG, 0x50);
		this->sar_enable = false;
	} else {
		dev_info( this->pdev, "sx9310 manual enable\n");
		reg_val = get_default_value_of_reg(this, SX9310_CPS_CTRL0_REG);
		//reg_val = (reg_val&(~0x7)) + val;
		set_default_value_of_reg(this, SX9310_CPS_CTRL0_REG, reg_val);
		write_register(this, SX9310_CPS_CTRL0_REG, reg_val);
		manual_offset_calibration(this);
		this->sar_enable = true;
	}

	this->set_sar_state(1);

	return count;
}

static DEVICE_ATTR(calibrate, 0644, manual_offset_calibration_show,
								manual_offset_calibration_store);
static DEVICE_ATTR(reg, 0644, reg_dump_show, reg_dump_store);
static DEVICE_ATTR(status, 0644, touch_status_show, NULL);
static DEVICE_ATTR(enable, 0666, sx9310_enable_show, sx9310_enable_store);

static struct attribute *sx9310_attributes[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_reg.attr,
	&dev_attr_status.attr,
	&dev_attr_enable.attr,
	NULL,
};
static struct attribute_group sx9310_attr_group = {
	.attrs = sx9310_attributes,
};
/*********************************************************************/





/*! \fn static int read_regStat(psx86XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx86XX_t this)
{
	u8 data = 0;
	if (this) {
		if (read_register(this,SX9310_IRQSTAT_REG,&data) == 0)
			return (data & 0x00FF);
	}
	return 0;
}

static void read_rawData(psx86XX_t this)
{
	u8 msb=0, lsb=0;
	if(this){
		write_register(this,SX9310_CPSRD,1);//here to check the CS1, also can read other channel
		msleep(100);
		read_register(this,SX9310_USEMSB,&msb);
		read_register(this,SX9310_USELSB,&lsb);
		dev_info(this->pdev, "sx9310 cs1 raw data USEFUL msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_AVGMSB,&msb);
		read_register(this,SX9310_AVGLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs1 raw data AVERAGE msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_DIFFMSB,&msb);
		read_register(this,SX9310_DIFFLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs1 raw data DIFF msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_OFFSETMSB,&msb);
		read_register(this,SX9310_OFFSETLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs1 raw data OFFSET msb = 0x%x, lsb = 0x%x\n",msb,lsb);

		write_register(this,SX9310_CPSRD,2);//here to check the CS1, also can read other channel
		msleep(100);
		read_register(this,SX9310_USEMSB,&msb);
		read_register(this,SX9310_USELSB,&lsb);
		dev_info(this->pdev, "sx9310 cs2 raw data USEFUL msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_AVGMSB,&msb);
		read_register(this,SX9310_AVGLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs2 raw data AVERAGE msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_DIFFMSB,&msb);
		read_register(this,SX9310_DIFFLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs2 raw data DIFF msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_OFFSETMSB,&msb);
		read_register(this,SX9310_OFFSETLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs2 raw data OFFSET msb = 0x%x, lsb = 0x%x\n",msb,lsb);
	}
}

/*! \brief	Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
 */
static void hw_init(psx86XX_t this)
{
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pdata = 0;
	int i = 0;
	/* configure device */
	dev_dbg(this->pdev, "Going to Setup I2C Registers\n");
	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
	{
		while ( i < pdata->i2c_reg_num) {
			/* Write all registers/values contained in i2c_reg */
			dev_dbg(this->pdev, "Going to Write Reg: 0x%x Value: 0x%x\n",
				pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
//		msleep(3);		  
			write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
			i++;
		}
	} else {
		dev_err(this->pdev, "ERROR! platform data 0x%p\n",pDevice->hw);
	}

	this->set_sar_state(1);//dli set sar state as 1, no touch when init.
}
/*********************************************************************/
/*! \fn static int initialize(psx86XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct 
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx86XX_t this)
{
	if (this) {
		/* prepare reset by disabling any irq handling */
		this->irq_disabled = 1;
		disable_irq(this->irq);
		/* perform a reset */
		write_register(this,SX9310_SOFTRESET_REG,SX9310_SOFTRESET);
		/* wait until the reset has finished by monitoring NIRQ */
		dev_dbg(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");
		/* just sleep for awhile instead of using a loop with reading irq status */
		msleep(300);
//	  while(this->get_nirq_low && this->get_nirq_low()) { read_regStat(this); }
		dev_dbg(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n",this->get_nirq_low());
		hw_init(this);
		msleep(100); /* make sure everything is running */
		manual_offset_calibration(this);

		/* re-enable interrupt handling */
		enable_irq(this->irq);
		this->irq_disabled = 0;

		/* make sure no interrupts are pending since enabling irq will only
		 * work on next falling edge */
		read_regStat(this);
		dev_dbg(this->pdev, "Exiting initialize(). NIRQ = %d\n",this->get_nirq_low());
		return 0;
	}
  	return -ENOMEM;
}

#ifdef CALI_LOOP

static u8 get_default_value_of_reg(psx86XX_t this, u8 reg)
{
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pdata = 0;
	int i = 0;
	u8 val = 0;

	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
		while ( i < pdata->i2c_reg_num) {
			if (reg == pdata->pi2c_reg[i].reg)
				return pdata->pi2c_reg[i].val;
			write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
			i++;
		}
	} else {
		dev_err(this->pdev, "ERROR! platform data 0x%p\n",pDevice->hw);
	}

	read_register(this, reg, &val);
	return val;
}

static void set_default_value_of_reg(psx86XX_t this, u8 reg, u8 val)
{
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pdata = 0;
	int i = 0;

	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
		while ( i < pdata->i2c_reg_num) {
			if (reg == pdata->pi2c_reg[i].reg) {
				pdata->pi2c_reg[i].val = val;
				break;
			}
			i++;
		}
	} else {
		dev_err(this->pdev, "ERROR! platform data 0x%p\n",pDevice->hw);
	}
}

static void calibration_work(struct work_struct *work)
{
	psx86XX_t this = container_of(work, struct sx86XX , cali_work);

	dev_info(this->pdev, "calibration work\n");

	manual_offset_calibration(this);

	if (this->sar_enable && (this->cali_count == LOOP_COUNT))
		write_register(this, SX9310_CPS_CTRL0_REG, get_default_value_of_reg(this, SX9310_CPS_CTRL0_REG));
}

static void calibration_loop(unsigned long p)
{
	psx86XX_t this = (psx86XX_t)p;

	dev_info(this->pdev, "calibration loop %d \n", this->cali_count);

	queue_work(cali_workqueue, &this->cali_work);

	if (this->cali_count++ < LOOP_COUNT) {
		if (timer_pending(&this->cali_timer))
			del_timer_sync(&this->cali_timer);

		this->cali_timer.expires = jiffies + INTERVAL;
		add_timer(&this->cali_timer);
	}
}
#endif

/*! 
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct 
 */
static void touchProcess(psx86XX_t this)
{
	int counter = 0;
	u8 i = 0;
	u8 stat1 = 0;
	int numberOfButtons = 0;
	psx9310_t pDevice = NULL;
	struct _buttonInfo *buttons = NULL;
//	struct input_dev *input = NULL;
	int state = IDLE;

	struct _buttonInfo *pCurrentButton  = NULL;


	if (this && (pDevice = this->pDevice))
	{
		dev_dbg(this->pdev, "Inside touchProcess()\n");
		read_register(this, SX9310_STAT0_REG, &i);
		read_register(this, SX9310_STAT1_REG, &stat1);
		dev_info(this->pdev, "Read Reg[0x%02x] = 0x%02x\n", SX9310_STAT0_REG, i);
		dev_info(this->pdev, "Read Reg[0x%02x] = 0x%02x\n", SX9310_STAT1_REG, stat1);

		buttons = pDevice->pbuttonInformation->buttons;
//	  input = pDevice->pbuttonInformation->input;
		numberOfButtons = pDevice->pbuttonInformation->buttonSize;
	
//	  if (unlikely( (buttons==NULL) || (input==NULL) )) {
//		dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
//		return;
//	  }

		for (counter = 0; counter < numberOfButtons; counter++) {
			pCurrentButton = &buttons[counter];
			if (pCurrentButton==NULL) {
			  dev_err(this->pdev,"ERROR!! current button at index: %d NULL!!!\n",
																		  counter);
			  return; // ERRORR!!!!
			}
			switch (pCurrentButton->state) {
				case IDLE: /* Button is not being touched! */
		  if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {
			/* User pressed button */
			dev_info(this->pdev, "cap button %d touched\n", counter);
//			  input_report_key(input, pCurrentButton->keycode, 1);
			pCurrentButton->state = ACTIVE;
		  } else {
			dev_dbg(this->pdev, "Button %d already released.\n",counter);
		  }
		  break;
		case ACTIVE: /* Button is being touched! */ 
		  if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {
			/* User released button */
			dev_info(this->pdev, "cap button %d released\n",counter);
//			  input_report_key(input, pCurrentButton->keycode, 0);
			pCurrentButton->state = IDLE;
		  } else {
			dev_dbg(this->pdev, "Button %d still touched.\n",counter);
		  }
		  break;
		default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
		  break;
	  };
	}

	for (counter = 0; counter < numberOfButtons; counter++) {
		pCurrentButton = &buttons[counter];
		if (pCurrentButton && pCurrentButton->state == ACTIVE) {
			state = ACTIVE;
			break;
		}
	}

	// if still touched after calibration triggered by band-switch,
	// no need to keep state because it means the last calibration
	// can't compensate the variation
	//if (this->ignore_once_info_modem && state == ACTIVE)
	//	  this->ignore_once_info_modem = false;

	// touched to released, need to calibration
	if (this->pre_state == ACTIVE && state == IDLE) {
	if (this->ignore_once_info_modem == false) {//no need to report state if we have just conducted a calibration
			this->set_sar_state(1);

			/* Loop calibration start */
			if (timer_pending(&this->cali_timer))
				del_timer_sync(&this->cali_timer);

			this->cali_timer.expires = jiffies + INTERVAL_FAR;
			add_timer(&this->cali_timer);
		}
	} else if (this->pre_state == IDLE && state == ACTIVE) {// released to touched
		this->set_sar_state(0);
		del_timer_sync(&this->cali_timer);
	}

	this->ignore_once_info_modem = false;
	this->pre_state = state;

//	  input_sync(input);
	  dev_dbg(this->pdev, "Leaving touchProcess()\n");
  }
}

static int sx9310_check_id(psx86XX_t this)
{
	u8 id;
	if(read_register(this, SX9310_WHOAMI, &id)) {
		dev_err( this->pdev, "sx9310 check id fail\n");
		return -1;
	}

	if(id !=0x01) {
		dev_err( this->pdev, "sx9310 id not match:0x%02x\n", id);
		return -2;
	}

	dev_info( this->pdev, "sx9310 check id ok\n");
	return 0;
}

/*! \fn static int sx9310_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9310_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i = 0;
	int ret;
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pplatData = 0;
//	struct input_dev *input = NULL;
	psx86XX_t this;

	dev_info(&client->dev, "sx9310_probe()\n");

	pplatData = client->dev.platform_data;
	if (!pplatData) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
					 I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	this = kzalloc(sizeof(sx86XX_t), GFP_KERNEL); /* create memory for main struct */
	dev_dbg(&client->dev, "\t Initialized Main Memory: 0x%p\n",this);

	/* SARSTATE gpio config */
//	gpio_request(GPIO_SX9310_SARSTATE, "SX9310_SARSTATE");
//	gpio_direction_output(GPIO_SX9310_SARSTATE, 1);

	if (this)
	{
		/* In case we need to reinitialize data 
		 * (e.q. if suspend reset device) */
		this->init = initialize;
		/* shortcut to read status of interrupt */
		this->refreshStatus = read_regStat;
		/* pointer to function from platform data to get pendown 
		 * (1->NIRQ=0, 0->NIRQ=1) */
		this->get_nirq_low = pplatData->get_is_nirq_low;
		this->set_sar_state = pplatData->set_sar_state;//dli
		this->get_sar_state = pplatData->get_sar_state;
		this->nirq_gpio = pplatData->nirq_gpio;
		this->nstate_gpio = pplatData->nstate_gpio;

		/* save irq in case we need to reference it */
		this->irq = client->irq;
		//this->irq = gpio_to_irq(GPIO_SX9310_NIRQ);
		dev_info(&client->dev, "this  irq: %d \n", this->irq);
		/* do we need to create an irq timer after interrupt ? */
		this->useIrqTimer = 0;
		this->ignore_once_info_modem= false;
		this->pre_state = IDLE;
		/* Setup function to call on corresponding reg irq source bit */
		if (MAX_NUM_STATUS_BITS>= 8)
		{
			this->statusFunc[0] = 0; /* TXEN_STAT */
			this->statusFunc[1] = 0; /* UNUSED */
			this->statusFunc[2] = 0; /* UNUSED */
			//this->statusFunc[3] = 0; /* CONV_STAT */
			this->statusFunc[3] = read_rawData; /* CONV_STAT */
			this->statusFunc[4] = 0; /* COMP_STAT */
			this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
			this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
			this->statusFunc[7] = 0; /* RESET_STAT */
		}

		/* setup i2c communication */
		this->bus = client;
		i2c_set_clientdata(client, this);

		/* record device struct */
		this->pdev = &client->dev;

		if(sx9310_check_id(this)) {
			dev_err(this->pdev, "sx9310 probe failed\n");
			kfree(this);
			return -EINVAL;
		}

		/* create memory for device specific struct */
		this->pDevice = pDevice = kzalloc(sizeof(sx9310_t), GFP_KERNEL);
		dev_dbg(&client->dev, "\t Initialized Device Specific Memory: 0x%p\n",pDevice);

		if (pDevice)
		{
			/* for accessing items in user data (e.g. calibrate) */
			ret = sysfs_create_group(&client->dev.kobj, &sx9310_attr_group);
			if(ret < 0){
				pr_err("%s:failed to create sx9310 attr group\n", __func__);
				return ret;
			}

			/* Check if we hava a platform initialization function to call*/
			if (pplatData->init_platform_hw)
			  pplatData->init_platform_hw();

			/* Add Pointer to main platform data struct */
			pDevice->hw = pplatData;
			
			/* Initialize the button information initialized with keycodes */
			pDevice->pbuttonInformation = pplatData->pbuttonInformation;

			/* Create the input device */
	//		input = input_allocate_device();
	//		if (!input) {
	//		  return -ENOMEM;
	//		}
	//
		  /* Set all the keycodes */
	//		__set_bit(EV_KEY, input->evbit);
			for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
	//			__set_bit(pDevice->pbuttonInformation->buttons[i].keycode, 
	//														  input->keybit);
				pDevice->pbuttonInformation->buttons[i].state = IDLE;
			}
		  /* save the input pointer and finish initialization */
	//		pDevice->pbuttonInformation->input = input;
	//		input->name = "SX9310 Cap Touch";
	//		input->id.bustype = BUS_I2C;
	//		input->id.product = sx863x->product;
	//		input->id.version = sx863x->version;
	//		if(input_register_device(input))
	//		  return -ENOMEM;
		}
		sx86XX_init(this);

	// add calibration loop.
#ifdef CALI_LOOP
		if (!cali_workqueue)
			cali_workqueue = create_workqueue("cali_workqueue");
		INIT_WORK(&this->cali_work, calibration_work);

		init_timer(&this->cali_timer);
		this->cali_count = 0;
		this->cali_timer.function = calibration_loop;
		this->cali_timer.data = (unsigned long)this;
		this->cali_timer.expires = jiffies + INTERVAL;
		add_timer(&this->cali_timer);
#endif

		this->read_flag = 0;
		this->read_reg = SX9310_CPS_CTRL0_REG;
		this->sar_enable = true;
		return	0;
	}
  	return -1;
}

/*! \fn static int sx9310_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx86XX_remove()
 */
static int sx9310_remove(struct i2c_client *client)
{
	psx9310_platform_data_t pplatData =0;
  	psx9310_t pDevice = 0;
	psx86XX_t this = i2c_get_clientdata(client);
	if (this && (pDevice = this->pDevice))
	{
//		input_unregister_device(pDevice->pbuttonInformation->input);

		sysfs_remove_group(&client->dev.kobj, &sx9310_attr_group);
		pplatData = client->dev.platform_data;
		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw();
		kfree(this->pDevice);
	}

#ifdef CALI_LOOP
	del_timer(&this->cali_timer);
	cancel_work_sync(&this->cali_work);
	flush_workqueue(cali_workqueue);
	destroy_workqueue(cali_workqueue);
#endif

	return sx86XX_remove(this);
}

#if 0
/*====================================================*/
/***** Kernel Suspend *****/
static int sx9310_suspend(struct i2c_client *client)
{
  psx86XX_t this = i2c_get_clientdata(client);
  sx86XX_suspend(this);
  read_regStat(this);
 return 0;
}
/***** Kernel Resume *****/
static int sx9310_resume(struct i2c_client *client)
{
  psx86XX_t this = i2c_get_clientdata(client);
  sx86XX_resume(this);

	//resume calibration.
	manual_offset_calibration(this);
	write_register(this, SX9310_CPS_CTRL0_REG, get_default_value_of_reg(this, SX9310_CPS_CTRL0_REG));
	dev_info( this->pdev, "sx9310 resume calibration.\n");
  return 0;
}
#endif

#if CONFIG_PM
static int sx9310_suspend(struct device *dev)
{
	struct i2c_client *client = (struct i2c_client *)to_i2c_client(dev);
	psx86XX_t this = (psx86XX_t)i2c_get_clientdata(client);

	if (this->cali_count++ < LOOP_COUNT) {
		if (timer_pending(&this->cali_timer))
			del_timer_sync(&this->cali_timer);
	}

	dev_info(this->pdev, "sx9310 prepare suspend.\n");
	write_register(this, SX9310_CPS_CTRL0_REG, 0x50);
	sx86XX_suspend(this);
	dev_info(this->pdev, "sx9310 disable IRQ.\n");
	read_regStat(this);
	return 0;
}

static int sx9310_resume(struct device *dev)
{
	struct i2c_client *client = (struct i2c_client *)to_i2c_client(dev);
	psx86XX_t this = (psx86XX_t)i2c_get_clientdata(client);

	sx86XX_resume(this);

	//resume calibration.
	if (this->sar_enable == true) {
		manual_offset_calibration(this);
		write_register(this, SX9310_CPS_CTRL0_REG, get_default_value_of_reg(this, SX9310_CPS_CTRL0_REG));
		dev_info( this->pdev, "sx9310 resume calibration.\n");
	}

	if (this->cali_count++ < LOOP_COUNT) {
		if (timer_pending(&this->cali_timer))
			del_timer_sync(&this->cali_timer);

		this->cali_timer.expires = jiffies + INTERVAL;
		add_timer(&this->cali_timer);
	}

	return 0;
}
static void sx9310_complete(struct device *dev)
{
	pr_err("%s\n",__func__);
	sx9310_resume(dev);
}
static const struct dev_pm_ops sx9310_pm = {
	.prepare = sx9310_suspend,
	.complete = sx9310_complete,
};
#endif

/*====================================================*/
static struct i2c_device_id sx9310_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx9310_idtable);
static struct i2c_driver sx9310_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
#if CONFIG_PM
		.pm	= &sx9310_pm,
#endif
	},
	.id_table = sx9310_idtable,
	.probe	  = sx9310_probe,
	.remove	  = /*__devexit_p*/(sx9310_remove),
#if defined(USE_KERNEL_SUSPEND)
	.suspend	= sx9310_suspend,
	.resume		= sx9310_resume,
#endif
};
static int __init sx9310_init(void)
{
	return i2c_add_driver(&sx9310_driver);
}
static void __exit sx9310_exit(void)
{
	i2c_del_driver(&sx9310_driver);
}

module_init(sx9310_init);
module_exit(sx9310_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9310 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
