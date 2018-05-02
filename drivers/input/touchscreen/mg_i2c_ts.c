/* Morgan digitizer device driver.
 *
 * Copyright(c) 2013 MorganTouch Inc.
 *
 * Author: leo 20130908
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/regulator/consumer.h>

#include <linux/gpio.h>
#include "mg_i2c_ts.h"

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>

#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/delay.h>

#include <linux/errno.h>
#include <linux/ctype.h>

#include <linux/input/wacom.h>

//#include "update.h"


#define MG_DRIVER_NAME 	"mg-i2c-mtouch"
#define CAP_BUF_SIZE 				30
#define DIG_BUF_SIZE 				8
#define BTN_TOOL_PEN_HOVER  238
#define UPDATEFW	0

static struct workqueue_struct *Mg_wq;
static int command_flag= 0;
static struct mg_data *private_ts;

#define COORD_INTERPRET(MSB_BYTE, LSB_BYTE)  (MSB_BYTE << 8 | LSB_BYTE)

struct mg_data
{
    __u16 	x, y, w, p, id,k, pen, key;
    struct i2c_client *client;
    /* digitizer */
    struct input_dev *dig_dev;
    struct mutex lock;
    struct work_struct work;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    struct workqueue_struct *mg_wq;
    struct wacom_platform_data *pdata;
    struct regulator *vdd;
    struct regulator *vcc_i2c;
    int irq;
    int (*power)(int on);
    int intr_gpio;
    struct miscdevice firmware;
    __u16 suspended;
    __u16 tp_mode;
};

static u8 read_buf[CAP_BUF_SIZE]={0};
static u8 ver_buf[RD_COMMAND_BIT]={0};
int ver_flag = 0;
int id_flag = 0;

static u16 mg_keycode[] = {KEY_PREVIOUS, KEY_NEXT, KEY_SAVE}; 

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mg_early_suspend(struct early_suspend *h);
static void mg_late_resume(struct early_suspend *h);
#endif
static struct i2c_driver mg_driver;
#ifdef CONFIG_PM_SLEEP
static int mg_i2c_suspend(struct device *dev);
static int mg_i2c_resume(struct device *dev);
#endif

#if UPDATEFW
static int flag_8899 = 0;
static int flag_8898 = 0;
int  ReadFirmwareVersion(void)
{
    int err,i = 5;

    memset(ver_buf, 0, sizeof(ver_buf));	
    ver_flag = 1;
    command_flag = 1;
    err = i2c_smbus_write_i2c_block_data(private_ts->client, 0, COMMAND_BYTE, command_list[3]);
    mdelay(800);
    if(err < 0)
    {
        ver_flag = 0;
        command_flag = 1;
        printk("[Driver ]ReadFirmwareVersion error!!!!!\n");
        return 0;
    }
    //printk(" ReadFirmwareVersion: %4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x\n", ver_buf[0], ver_buf[1], ver_buf[2], ver_buf[3], ver_buf[4], ver_buf[5], ver_buf[6], ver_buf[7]);

    while( (ver_buf[5] == 0) && (ver_buf[6] == 0) && i--){
        printk("[Driver ]ReadFirmwareVersion error,  read again!!!!!\n");

        i2c_smbus_write_i2c_block_data(private_ts->client, 0, COMMAND_BYTE, command_list[3]);
        mdelay(10);
        i2c_smbus_read_i2c_block_data(private_ts->client, 0, 8, ver_buf);	
        mdelay(10);
        //printk(" ReadFirmwareVersion: %4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x\n", ver_buf[0], ver_buf[1], ver_buf[2], ver_buf[3], ver_buf[4], ver_buf[5], ver_buf[6], ver_buf[7]);
    }

    if( (ver_buf[5] == 0) && (ver_buf[6] == 0))
    {
        printk("[Driver ]ReadFirmwareVersion ==0.0==\n");		
        return 0;
    }
    else if(ver_buf[5] == 0x01)
    {
        flag_8899 = 1;
        printk("IC IS 8899!!!\n");	
        return 1;
    }
    else if(ver_buf[5] == 0x03)
    {
        flag_8898 = 1;
        printk("IC IS 8898!!!\n");	
        return 1;
    }
    return 0;	
}

int CheckIfFirmwareNeedUpdate(void)
{	
    //int DataLen = sizeof(mg8398_firmware);
    unsigned char *p_data = NULL;
    int fv1,fv2;
    int ret;
    ret = ReadFirmwareVersion();
    if(flag_8899 == 1 && flag_8898 == 0)
    {
        p_data = mg8398_firmware_8899;
    }
    else if(flag_8899 == 0 && flag_8898 == 1)
    {
        p_data = mg8398_firmware_8898;
    }

    /*
       if (!DataLen)
       {
       printk("No firmware for update\n");
       return -1;
       }
       */
    mdelay(200);
    if(!ret)
    {
        return 1;
    } 
    else
    {
        fv1 = ver_buf[5];
        fv2 = ver_buf[6];
        printk("=========================\n");
        printk("===MG8398 FW Version = %x.%x===\n", fv1, fv2);	
        printk("=========================\n");	
        if( (p_data[0] != fv1) || (p_data[1] != fv2))
            return 0;
        else
            return 1;
    }					
}  

int UpdateFirmware(void)
{
    int i, ret = 0;
    int kk= 10;
    int PkgCnt = 0;
    u8 test[5] = {0};
    int DataLen = 0;
    unsigned char *p_data = NULL;

    if(flag_8899 == 1 && flag_8898 == 0)
    {
        DataLen = sizeof(mg8398_firmware_8899);
        p_data = mg8398_firmware_8899;
    }
    else if(flag_8899 == 0 && flag_8898 == 1)
    {
        DataLen = sizeof(mg8398_firmware_8898);
        p_data = mg8398_firmware_8898;
    }

    printk("FWLen = %d\n", DataLen);

    // Jump to Bootloader
    printk("[Driver]Turn to Bootloader !!!!\n");
    ret = i2c_smbus_write_i2c_block_data(private_ts->client, 0, COMMAND_BYTE, command_list[0]);
    if(ret < 0)
    {
        printk("[Driver]Turn to Bootloader Fail!!!!!\n");
        return -1;
    }
    mdelay(5);

    printk("[Driver]Read IC PID !!!!\n");
    ret = i2c_smbus_write_i2c_block_data(private_ts->client, 0, COMMAND_BYTE, command_list[4]);
    if(ret < 0)
    {
        printk("[Driver]Turn to Bootloader Fail!!!!!\n");
        return -1;
    }
    mdelay(100);

    ret = i2c_smbus_read_i2c_block_data(private_ts->client, 0, COMMAND_BYTE, test);	
    if(ret < 0){
        printk("Cannot Read Ack, \n");}

    //printk(" Read PID Ack: %4x,%4x,%4x,%4x,%4x\n", test[0], test[1], test[2], test[3], test[4]);
    while((test[3] != 0xab) && (test[4] != 0xcd) && kk--){
        printk("[Driver]Turn to Bootloader Fail, read again!!!!!\n");

        //i2c_smbus_write_i2c_block_data(private_ts->client, 0, COMMAND_BYTE, command_list[0]);
        //mdelay(500);
        //i2c_smbus_read_i2c_block_data(private_ts->client, 0, 5, test);

        //printk(" Read BOOT Ack: %4x,%4x,%4x,%4x,%4x\n", test[0], test[1], test[2], test[3], test[4]);

        i2c_smbus_write_i2c_block_data(private_ts->client, 0, COMMAND_BYTE, command_list[4]);	
        mdelay(100);
        i2c_smbus_read_i2c_block_data(private_ts->client, 0, 5, test);
        mdelay(10);
        //printk(" Read PID Ack: %4x,%4x,%4x,%4x,%4x\n", test[0], test[1], test[2], test[3], test[4]);
    }

    if((test[3] == 0xab) && (test[4] == 0xcd))
    {
        printk("[Driver]Turn to Bootloader sucess!!!!\n");
        printk("================start update=================\n");
        p_data += 13;
        PkgCnt = (DataLen-13)/13;

        for(i = 0; i < PkgCnt; i++)
        {
            // ret = i2c_master_send(private_ts->client, p_data, 13);
            ret = i2c_smbus_write_i2c_block_data(private_ts->client, 0, FIRMWARE_UPDATE_LENGTH, p_data);			
            if(ret < 0)
            {
                printk("[Driver ]i2c_smbus_write_i2c_block_data fail, Update Failed!!!!!\n");
                return -1;
            }
            mdelay(20);

            ret = i2c_smbus_read_i2c_block_data(private_ts->client, 0, COMMAND_BYTE, test);	
            if(ret < 0)
                printk("Cannot Read Ack, Update Failed\n");

            printk(" Read Ack: %4x,%4x,%4x,%4x,%4x\n", test[0], test[1], test[2], test[3], test[4]);
            p_data += 13;
        }
        mdelay(400);
        printk("================end update=================\n");
        ret = i2c_smbus_write_i2c_block_data(private_ts->client, 0, COMMAND_BYTE, command_list[1]);
        if(ret < 0){
            printk("[Driver]Turn to App Fail!!!!!\n");
            return -1;
        }
        else {
            printk("[Driver]Turn to App sucess!!!!\n");
        }
        mdelay(400);
    }

    return 0;
}
/*
   static DEVICE_ATTR(firmware_version, S_IRUGO|S_IWUSR|S_IWGRP, ReadFirmwareVersion, NULL);
   static struct attribute *mg_attributes[] = {
   &dev_attr_firmware_version.attr,
   NULL
   };

   static const struct attribute_group mg_attr_group = {
   .attrs = mg_attributes,
   };
   */
#endif
#if 1
    static ssize_t reset_emr_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{

        unsigned int val;
        int error;

        error = kstrtouint(buf, 10, &val);
        if (error)
                return error;

        printk("%s %d\n", __func__, val);
        if (val)
                error = gpio_direction_output(private_ts->pdata->gpio_reset, 1);
        else
                error = gpio_direction_output(private_ts->pdata->gpio_reset, 0);
        printk("%s %d, %d\n", __func__, val, error);
        return count;

}

static ssize_t enable_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{

        unsigned int val;
        int error;

        error = kstrtouint(buf, 10, &val);
        if (error)
                return error;

        printk("%s %d\n", __func__, val);
        return count;

}

static ssize_t tp_mode_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int tp_mode;
    struct i2c_client *client = to_i2c_client(dev);
    struct mg_data *ts = i2c_get_clientdata(client);

    tp_mode = simple_strtoul(buf, NULL, 0);
     if ( tp_mode == 2 ){
	ts->tp_mode = tp_mode;
	mg_i2c_suspend(dev);
     }else if (tp_mode == 3){
	ts->tp_mode = tp_mode;
	mg_i2c_resume(dev);
     }
    return count;
}

static ssize_t tp_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct mg_data *ts = i2c_get_clientdata(client);
    return scnprintf(buf, PAGE_SIZE, "%d\n",ts->suspended?2:3);
}

   static DEVICE_ATTR(reset_emr, S_IRUGO|S_IWUSR|S_IWGRP, NULL, reset_emr_store);
   static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, NULL, enable_store);
   static DEVICE_ATTR(tp_mode, S_IRUGO|S_IWUSR|S_IWGRP, tp_mode_show, tp_mode_write);

    static struct attribute *mg_attributes[] = {
   /* &dev_attr_reset_emr.attr, */
   /* &dev_attr_enable.attr, */
   &dev_attr_tp_mode.attr,
   NULL
   };

   static const struct attribute_group mg_attr_group = {
   .attrs = mg_attributes,
   };


#endif


static int mg_query_device(struct i2c_client *client)
{
	int err;
	printk("%s\n", __func__);
        err = i2c_smbus_write_i2c_block_data(client, 0, COMMAND_BYTE, command_list[4]);	
	if ( err < 0 ) 
	{
		printk("%s error \n", __func__);
		return -1;
	}
        mdelay(100);
        i2c_smbus_read_i2c_block_data(client, 0, 5, ver_buf);
        printk(" ReadFirmwareVersion: %4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x\n", ver_buf[0], ver_buf[1], ver_buf[2], ver_buf[3], ver_buf[4], ver_buf[5], ver_buf[6], ver_buf[7]);
	return 0;
}

static int wacom_power_on(struct mg_data *data, bool on)
{
        int rc=0;
        if (!on)
                goto power_off;

	printk("morgan %s %d\n", __func__, on);
        rc = regulator_enable(data->vdd);
        if (rc) {
                dev_err(&data->client->dev,
                        "Regulator vdd enable failed rc=%d\n", rc);
        return rc;
        }

        msleep(50);

        rc = regulator_enable(data->vcc_i2c);
        if (rc) {
                dev_err(&data->client->dev,
                        "Regulator vcc_i2c enable failed rc=%d\n", rc);
        return rc;
        }

        msleep(50);
    return 0;


power_off:


        rc = regulator_disable(data->vcc_i2c);
        if (rc) {
                dev_err(&data->client->dev,
                        "Regulator vcc_i2c disable failed rc=%d\n", rc);
        return rc;
        }

        if(data->vdd){
                rc = regulator_disable(data->vdd);
                if (rc) {
                        dev_err(&data->client->dev,
                                "Regulator vdd disable failed rc=%d\n", rc);
                return rc;
                }
        }
    return 0;
}

static int wacom_power_init(struct mg_data *data, bool on)
{
        int rc;
        if (!on)
                goto pwr_deinit;

	printk("%s %d\n", __func__, on);
        //data->vdd = regulator_get(&data->client->dev, "VPROG6A");
	if (yeti_hw_ver >= 2 )
	       data->vdd = regulator_get(&data->client->dev, "VPROG3B");
	else
	       data->vdd = regulator_get(&data->client->dev, "VPROG4D");
        if (IS_ERR(data->vdd)) {
                rc = PTR_ERR(data->vdd);
                dev_err(&data->client->dev,
                        "Regulator VPROG6A 3.3 v get failed vdd rc=%d\n", rc);
                goto err;
        }

        if (regulator_count_voltages(data->vdd) > 0) {
                rc = regulator_set_voltage(data->vdd, WACOM_VTG_MIN_UV,
                                           WACOM_VTG_MAX_UV);
                if (rc) {
                        dev_err(&data->client->dev,
                        "Regulator set_vtg VPROG6A failed vdd rc=%d\n", rc);
                        goto reg_vdd_put;
                }
        }



        data->vcc_i2c = regulator_get(&data->client->dev, "VPROG5B");
        if (IS_ERR(data->vcc_i2c)) {
                rc = PTR_ERR(data->vcc_i2c);
                dev_err(&data->client->dev,
                        "Regulator get failed vcc_i2c rc=%d\n", rc);
                goto reg_vdd_put;
        }

        if (regulator_count_voltages(data->vcc_i2c) > 0) {
                rc = regulator_set_voltage(data->vcc_i2c, WACOM_I2C_VTG_MIN_UV,
                                           WACOM_I2C_VTG_MAX_UV);
                if (rc) {
                        dev_err(&data->client->dev,
                        "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
                        goto reg_vcc_i2c_put;
                }
        }

        return 0;


reg_vcc_i2c_put:
        regulator_put(data->vcc_i2c);
reg_vdd_put:
        regulator_put(data->vdd);
err:
        return rc;
pwr_deinit:
        if (regulator_count_voltages(data->vcc_i2c) > 0)
                regulator_set_voltage(data->vcc_i2c, 0, WACOM_I2C_VTG_MAX_UV);
                regulator_put(data->vcc_i2c);

        if (regulator_count_voltages(data->vdd) > 0)
                regulator_set_voltage(data->vdd, 0, WACOM_VTG_MAX_UV);
                regulator_put(data->vdd);
        return 0;
}


static irqreturn_t mg_irq(int irq, void *_mg)
{
    struct mg_data *mg = _mg;

    queue_work(Mg_wq, &mg->work);

    return IRQ_HANDLED;
}

static void key_msg_process(u8 *data_in,struct mg_data *mg)
{
  int i, mask;
   if (read_buf[MG_DIG_MODE] !=4 )
	return;
   __u16 key_state = read_buf[MG_DIG_STATUS];   
  if (( key_state == 0 ) && (mg->k == 0 ))
	return;
  //printk("%s %d %d\n",__func__, key_state, mg->k); 
  for(i = 0;i < 3;i++){
      mask = 1<<(i+1);
      if (( mg->k & mask ) && (( key_state & mask ) == 0 )) {
	input_report_key(mg->dig_dev, mg_keycode[i], 0);
	printk(" key released %d\n", mg_keycode[i]);
      }
      if ((( mg->k & mask) == 0) && ( key_state & mask )) {
	input_report_key(mg->dig_dev, mg_keycode[i], 1);
	printk(" key pressed %d\n", mg_keycode[i]);
       }
  }   
  input_sync(mg->dig_dev);
  mg->k = key_state;
}
static inline void mg_report(struct mg_data *mg)
{	
   // input_report_key(mg->dig_dev, BTN_TOOL_PEN, ABS_MT_TOOL_TYPE);
    input_event(mg->dig_dev,EV_KEY,BTN_TOOL_PEN,mg->pen);
    input_report_abs(mg->dig_dev,ABS_MT_POSITION_X,(REPORT_MAX_X - mg->x));
    input_report_abs(mg->dig_dev,ABS_MT_POSITION_Y,mg->y);
    input_report_abs(mg->dig_dev,ABS_MT_PRESSURE,mg->p);
    input_sync(mg->dig_dev);
}

static void mg_msg_process(u8 *data_in,struct mg_data *mg)
{

    u8 keyCode = read_buf[MG_DIG_STATUS];
    __u16 pressure = (COORD_INTERPRET(read_buf[MG_DIG_Z_HI], read_buf[MG_DIG_Z_LOW]));
    bool hover_enter = 0;
//	static int  flag_back = 0;
    if (read_buf[MG_DIG_MODE] != 2) 
        return;
    //last pressure is saved in mg->p,and last key code is saved in mg->w
   // printk("%s k %x->%x, p %x->%x\n", __func__, mg->w,keyCode, mg->p, pressure);
    if(pressure==0)
    {
        if(0 != mg->p)
        	input_event(mg->dig_dev,EV_KEY,BTN_TOUCH,0);
        switch(keyCode)
        {
            case MG_OUT_RANG:
                //if(MG_IN_RANG == mg->w){
                input_event(mg->dig_dev,EV_KEY,BTN_TOOL_PEN,0);
		mg->pen = 0;
                    //printk("input report EV_KEY,code is pen hover out rang\n");
		if (mg->key)
			input_event(mg->dig_dev, EV_KEY, mg->key, 0);
		mg->key = 0;
                //}
                break;
            case MG_IN_RANG://0x10
                if(MG_OUT_RANG == mg->w){
                    input_event(mg->dig_dev,EV_KEY,BTN_TOOL_PEN_HOVER,1);
                    hover_enter = 1;
                    //printk("input report EV_KEY,code is pen hover\n");
                }
                else if(MG_CUSTOM_BTN == mg->w||MG_RIGHT_BTN == mg->w) {
                    input_event(mg->dig_dev,EV_KEY,mg->w == MG_CUSTOM_BTN?BTN_STYLUS:BTN_STYLUS2,0);
		    mg->key = 0;
		}
			/*
				if(flag_back != 1){
					//printk("flag_back:\t%d\n",flag_back);
                	flag_back = 1;
				//	printk("flag_back:\t%d\n",flag_back);
				}
			*/
                break;
            case MG_CUSTOM_BTN:
                if(MG_IN_RANG == mg->w){
                    input_event(mg->dig_dev,EV_KEY,BTN_STYLUS,1);
	            mg->key = BTN_STYLUS;
		}
                break;
            case MG_RIGHT_BTN://0x13
                if(MG_IN_RANG == mg->w) {
                    input_event(mg->dig_dev,EV_KEY,BTN_STYLUS2,1);
	            mg->key = BTN_STYLUS2;
		}
				//printk("flag_back2222:\t%d\n",flag_back);
			/*
				if (flag_back == 1) {
		            flag_back = 0;
					//printk("MG_RIGHT_BTN:%xflag_back::%d\n",MG_RIGHT_BTN,flag_back);
					input_event(mg->dig_dev,EV_KEY,KEY_MENU,1);
		            input_event(mg->dig_dev,EV_KEY,KEY_MENU,0);
		            input_sync(mg->dig_dev);
				}
			*/
                break;
            default:
                break;
        }
    }
    else if(0 == mg->p)
    {
        input_event(mg->dig_dev,EV_KEY,BTN_TOUCH,1);       	
    }
    input_sync(mg->dig_dev);
    mg->x =  COORD_INTERPRET(read_buf[MG_DIG_Y_HI], read_buf[MG_DIG_Y_LOW]);
    mg->y =  COORD_INTERPRET(read_buf[MG_DIG_X_HI], read_buf[MG_DIG_X_LOW]);
    mg->w = read_buf[MG_DIG_STATUS];
    mg->p = pressure;
    mg_report(mg);
    if(hover_enter){
        input_event(mg->dig_dev,EV_KEY,BTN_TOOL_PEN,1);
        input_event(mg->dig_dev,EV_KEY,BTN_TOOL_PEN_HOVER,0);
	mg->pen = 1;
    }
}

static void mg_i2c_work(struct work_struct *work)
{
    int i = 0;
    struct mg_data *mg = container_of(work, struct mg_data, work);
    u_int8_t ret = 0;

    memset( read_buf, 0, sizeof(read_buf) );
    /*	Check if I/O control command 	*/
    if(command_flag == 1)
    {
        ret = i2c_smbus_read_i2c_block_data(mg->client, 0x0, RD_COMMAND_BIT, read_buf);
        if(ret < 0)
        {
            for(i = 1; i< 11; i++)
            {
                if(i == 11)
                {
                    printk("Read 10 Times error!!!!!\n");
                    command_flag = 0;
                    return;
                }
                printk("Read %4d Times Error!!!!!\n", i);
                ret = i2c_smbus_read_i2c_block_data(mg->client, 0x0, RD_COMMAND_BIT, read_buf);
                if(ret >= 0)
                {
                    printk("Read OK!!!!!\n");
                    break;
                }
            }
        }

        if(ver_flag == 1 || id_flag == 1)
        {
            for(i = 0; i < RD_COMMAND_BIT; i ++)
                ver_buf[i] = read_buf[i];
            ver_flag = 0;
            id_flag = 0;
        }

        command_flag = 0;
        return;
    }

    ret = i2c_smbus_read_i2c_block_data(mg->client, 0x0, CAP_BUF_SIZE, read_buf);
    if(ret < 0)
    {
        printk("Read error!!!!!\n");
        return;
    }
#if 0
    printk("\n");
    printk("READ BUF: ");
    for(i = 0; i < 8; i++)
    {

        printk("%4x", read_buf[i]);
    }
    printk("\n");
#endif

    if (read_buf[MG_DIG_MODE] == 2) 
    {
        mg_msg_process(read_buf, mg);
    }else if (read_buf[MG_DIG_MODE] == 4)
    {
       key_msg_process(read_buf, mg);
    }	
}



static int mg_probe(struct i2c_client *client, const struct i2c_device_id *ids)	
{

    struct mg_data *mg;
    struct input_dev *input_dig;
    int err = 0;
    int i;
    printk("\n=========%s=======\n", __func__);
    //allocate mg data 
    /*	err = i2c_smbus_write_i2c_block_data(client, 0, COMMAND_BYTE, command_list[4]);
        if(err < 0)
        {
        printk("-----read ID failed\n");
        return err;
        }
        */
    mg = kzalloc(sizeof(struct mg_data), GFP_KERNEL);
    if (!mg)
        return -ENOMEM;

    mg->client = client;
    mg->k = 0;
    mg->suspended = 0;
    mg->key = 0;
    dev_info(&mg->client->dev, "device probing\n");
    i2c_set_clientdata(client, mg);
    mutex_init(&mg->lock);
    mg->pdata = client->dev.platform_data; 
    /* allocate input device for digitizer */
    input_dig = input_allocate_device();
    input_dig->name = "mg-digitizer";
    input_dig->id.bustype = BUS_I2C;
    mg->dig_dev = input_dig;

    //__set_bit(INPUT_PROP_DIRECT, input_dig->propbit);
    //__set_bit(INPUT_PROP_POINTER, input_dig->propbit);
    __set_bit(EV_ABS, input_dig->evbit);
    __set_bit(EV_KEY, input_dig->evbit);
	__set_bit(KEY_BACK, input_dig->keybit);//
	__set_bit(KEY_MENU, input_dig->keybit);
     for( i = 0; i<3;i++)
	{
	 __set_bit(mg_keycode[i], input_dig->keybit);
	}

    __set_bit(BTN_TOUCH, input_dig->keybit);
    __set_bit(BTN_TOOL_PEN, input_dig->keybit);
    __set_bit(BTN_TOOL_PEN_HOVER, input_dig->keybit);
    __set_bit(BTN_STYLUS, input_dig->keybit);
    __set_bit(BTN_STYLUS2, input_dig->keybit);

    input_set_abs_params(input_dig, ABS_MT_POSITION_X, 0, REPORT_MAX_X, 0, 0);
    input_set_abs_params(input_dig, ABS_MT_POSITION_Y, 0, REPORT_MAX_Y, 0, 0);
    input_set_abs_params(input_dig, ABS_MT_TOUCH_MAJOR, 0, DIG_MAX_P, 0, 0);
    input_set_abs_params(input_dig, ABS_MT_PRESSURE, 0, DIG_MAX_P, 0, 0);

    err = input_register_device(input_dig);
    if (err) {
		kfree(mg);
		return -1;
	}

    err = wacom_power_init(mg, true);
        if (err) {
                dev_err(&client->dev, "power init failed");
             goto exit_input;
        }
    err = wacom_power_on(mg, true);
        if (err) {
                dev_err(&client->dev, "power on failed");
             goto exit_input;
        }

        printk("power on morgan device\n");
	if ( mg_query_device(client) < 0 )
	{
	      wacom_power_init(mg,false);
	      printk(" query mg device \n");
             goto exit_input;
	}

    INIT_WORK(&mg->work, mg_i2c_work);
	
   #if  1
        if(gpio_is_valid(mg->pdata->gpio_reset)){
                err = gpio_request(mg->pdata->gpio_reset, "wacom_reset");
                if (err) {
                        pr_err("%s: Failed to get gpio %d (code: %d)",
                                __func__, mg->pdata->gpio_reset, err);
                        goto exit_input;
                }
#if 0
                err = gpio_direction_output(mg->pdata->gpio_reset, 0);
                if (err) {
                        pr_err("%s: Failed to set gpio %d direction",
                                        __func__, mg->pdata->gpio_reset);
                        return err;
                }
                msleep(1000);
#endif
                err = gpio_direction_output(mg->pdata->gpio_reset, 1);
                if (err) {
                        pr_err("%s: Failed to set gpio %d direction",
                                        __func__, mg->pdata->gpio_reset);
			gpio_free(mg->pdata->gpio_reset);
                        goto exit_input;
                }
                msleep(1000);

        }
#endif

        if(gpio_is_valid(mg->pdata->gpio_int)){
                err= gpio_request(mg->pdata->gpio_int, "mg_interrupt");
                if (err) {
                        pr_err("%s: Failed to get gpio %d (code: %d)",
                                __func__, mg->pdata->gpio_int, err);
			gpio_free(mg->pdata->gpio_reset);
                        return err;
                }
                err= gpio_direction_input(mg->pdata->gpio_int);
                if (err) {
                        pr_err("%s: Failed to set gpio %d direction",
                                        __func__, mg->pdata->gpio_int);
			gpio_free(mg->pdata->gpio_reset);
			gpio_free(mg->pdata->gpio_int);
                        goto exit_input;
                }
                mg->client->irq = gpio_to_irq (mg->pdata->gpio_int);
		printk("%s irq %d\n", __func__, mg->client->irq);
        }



    mg->irq = client->irq;

    //err = request_irq(mg->irq, mg_irq,IRQF_TRIGGER_LOW|IRQF_ONESHOT, MG_DRIVER_NAME, mg);
    err = request_irq(mg->irq, mg_irq,IRQF_TRIGGER_FALLING, MG_DRIVER_NAME, mg);
    private_ts = mg;

#ifdef CONFIG_HAS_EARLYSUSPEND
    mg->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    mg->early_suspend.suspend = mg_early_suspend;
    mg->early_suspend.resume = mg_late_resume;
    register_early_suspend(&mg->early_suspend);
#endif


#if UPDATEFW
    //ReadFirmwareVersion();

    if(CheckIfFirmwareNeedUpdate())
        printk( "====MG8398 no need to be updated!!!!\n" );
    else
    {
        printk("====MG8398 firmware need to be update!\n");
        mdelay(1000);
        UpdateFirmware();
        mdelay(500);
    }
#endif
     err = sysfs_create_group(&client->dev.kobj, &mg_attr_group);
        if (err) {
		printk("%s create attr fail \n", __func__);
	}

    return 0;

exit_input:
    input_unregister_device(mg->dig_dev);
    kfree(mg);

    return -1;

}

static int mg_remove(struct i2c_client *client)
{
    struct mg_data *mg = i2c_get_clientdata(client);

    free_irq(mg->irq, mg);
    input_unregister_device(mg->dig_dev);
    kfree(mg);
    return 0;
}


#ifdef CONFIG_PM_SLEEP
static int mg_i2c_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);

    int ret;
    struct mg_data *ts = i2c_get_clientdata(client);

    if (ts->suspended)  {
        printk("%s suspended yet\n", __func__);
	return 0;
     }

    if (ts->irq)
        disable_irq(client->irq);

    ret = cancel_work_sync(&ts->work);
    if (ret && ts->irq) // if work was pending disable-count is now 2 
        enable_irq(client->irq);
    mdelay(50);


    ret = i2c_smbus_write_i2c_block_data(ts->client, 0, COMMAND_BYTE, command_list[5]);
    if (ret < 0){
        printk(KERN_ERR "mg_i2c_suspend: i2c_smbus_write_i2c_block_data failed\n");}
    else {
        printk("mg_suspend success\n");}

	ts->suspended = 1;
        return 0;
}

static int mg_i2c_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);

        struct mg_data *ts = i2c_get_clientdata(client);
	if ( ts->tp_mode  == 2 ){
		printk("%s tp_mode %d\n", __func__, ts->tp_mode);
		return;
	}
    if (ts->suspended == 0 )
    {
        printk("%s resume yet\n", __func__);
	return 0;
    }
    if (ts->irq)
        enable_irq(client->irq);

     //gpio_direction_output(->pdata->gpio_reset, 1);
     mdelay(10);
     gpio_set_value(ts->pdata->gpio_reset, 0);
     mdelay(50);
     gpio_set_value(ts->pdata->gpio_reset, 1);
     mdelay(10);

      printk("mg_resume success\n");
	ts->suspended = 0;

        return 0;
}
static SIMPLE_DEV_PM_OPS(mg_i2c_pm, mg_i2c_suspend, mg_i2c_resume);

#endif 

#ifdef CONFIG_HAS_EARLYSUSPEND
static int mg_suspend(struct i2c_client *client, pm_message_t state)
{
#if 1
    //printk("\nmg_suspend11111111\n"); 
    int ret;

    struct mg_data *ts = i2c_get_clientdata(client);

    if (ts->irq)
        disable_irq(client->irq);

    ret = cancel_work_sync(&ts->work);
    if (ret && ts->irq) // if work was pending disable-count is now 2 
        enable_irq(client->irq);
    mdelay(50);


    ret = i2c_smbus_write_i2c_block_data(ts->client, 0, COMMAND_BYTE, command_list[5]);
    if (ret < 0){
        printk(KERN_ERR "mg_suspend: i2c_smbus_write_i2c_block_data failed\n");}
    else {
        printk("mg8398_suspend success\n");}
#endif
    return 0;
}

static int mg_resume(struct i2c_client *client)
{

    struct mg_data *ts = i2c_get_clientdata(client);
    if (ts->irq)
        enable_irq(client->irq);

    gpio_direction_output(MG_RST, 1);
    mdelay(10);
    gpio_set_value(MG_RST, GPIO_LOW);
    mdelay(10);
    gpio_set_value(MG_RST, GPIO_HIGH);
    mdelay(10);

    printk("mg8398_resume success\n");
    return 0;
}

static void mg_early_suspend(struct early_suspend *h)
{
    struct mg_data *ts;
    ts = container_of(h, struct mg_data, early_suspend);
    mg_suspend(ts->client, PMSG_SUSPEND);
}

static void mg_late_resume(struct early_suspend *h)
{
    struct mg_data *ts;
    ts = container_of(h, struct mg_data, early_suspend);
    mg_resume(ts->client);
}
#endif

static struct i2c_device_id mg_id_table[] =
{
    /* the slave address is passed by i2c_boardinfo */
    {MGT_NAME},
    {/* end of list */}
};

static struct i2c_driver mg_driver = {
    .driver = 
    {
        .name	 = MG_DRIVER_NAME,
	.pm     = &mg_i2c_pm,
    },
    .id_table 	= mg_id_table,
    .probe 		= mg_probe,
    .remove 	= mg_remove,
};

static int __init mg_init(void)
{
    Mg_wq = create_singlethread_workqueue("mg_wq");	
    if (!Mg_wq) 
    {
        printk(KERN_ALERT "creat workqueue faiked\n");
        return -ENOMEM;
    }
    return i2c_add_driver(&mg_driver);
}

static void mg_exit(void)
{
    i2c_del_driver(&mg_driver);
}
module_init(mg_init);
module_exit(mg_exit);


