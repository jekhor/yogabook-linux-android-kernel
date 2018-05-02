/*******************************************************************************
 * Copyright (C) 2014 HiDeep, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *******************************************************************************/

#include <linux/input/ist520e.h>

/*******************************************************************************
 * sysfs subsystem
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define     IST520E_UC      "crimson.bin"
extern void istcore_init_mode(struct ist510e *ts);

/*------------------------------------------------------------------------------
 * download ucode
 *-----------------------------------------------------------------------------*/
static ssize_t hideep_self_test_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    int i;
    int count;

    ts_log_info();

    hideep_irq_enable(false);
    istxxxx_test_mode(ts);
    hideep_irq_enable(true);
    ret = 0;
    //TX short
    for(count = 0, i = 0; i < TX_NUM; i++){
        if(ts->TXshortResult[i]){
            if(count ==0)
              ts_log_err("TX[%d]\n",i);
            else
              ts_log_err(",TX[%d]\n",i);
            count++;
            ts_log_info("tx short");
            ret = -1;
        }
    }
    if(count)
        ts_log_err( "TX(s) are short!\n");
    //TX open
    for(count = 0, i = 0; i < TX_NUM; i++){
        if(ts->TXopenResult[i]){
            if(count ==0)
              ts_log_err( "TX[%d]\n",i);
            else
              ts_log_err( ",TX[%d]\n",i);
            count++;
            ts_log_info("tx open");
            ret = -1;
        }
    }
    if(count)
        ts_log_err( "TX(s) are open!\n");


    //RX short
    for(count = 0, i = 0; i <RX_NUM; i++){
        if(ts->RXshortResult[i]){
            if(count ==0)
              ts_log_err( "RX[%d]\n",i);
            else
              ts_log_err( ",RX[%d]\n",i);
            count++;
            ts_log_info("rx short");
            ret = -1;
        }
    }
    if(count)
        ts_log_err( "RX(s) are short!\n");
    //RX open
    for(count = 0, i = 0; i < RX_NUM; i++){
        if(ts->RXopenResult[i]){
            if(count ==0)
              ts_log_err( "RX[%d]\n",i);
            else
              ts_log_err( ",RX[%d]\n",i);
            count++;
            ts_log_info("rs open");
            ret = -1;
        }
    }
    if(count)
        ts_log_err( "RX(s) are open!\n");
    if(ret == 0)
        return scnprintf(buf, PAGE_SIZE, "PASS\n");
    return scnprintf(buf, PAGE_SIZE, "FAIL\n");
}


static ssize_t
fuse_ucode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct ist510e *ts_drv = dev_get_drvdata(dev);
    int ret;

    hideep_irq_enable(false);
    ts->manually_update = true;
    ret = ist_load_ucode(dev, IST520E_UC, MANUAL_FIRMWARE_UPDATE);
    if (ret)
    {
        dev_err(dev, "The firmware update failed(%d)\n", ret);
        count = ret;
    }
    ts->manually_update = false;
    hideep_irq_enable(true);

    return count;

}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static ssize_t
read_ucode(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t len = 0;
    struct ist510e *ts = dev_get_drvdata(dev);

    //---------------------------------
    // TODO : make sentence about ucode.
    len = scnprintf(buf, PAGE_SIZE, "%d\n", ts->dwz_info.ver_c);
    return len;
}

/*------------------------------------------------------------------------------
 * vr interface : data read
 *-----------------------------------------------------------------------------*/
static ssize_t
r_vr_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    u32 len = 0;
    u32 vr_data = 0;
    struct ist510e *ts = dev_get_drvdata(dev);

    istcore_i2c_read(ts, ts->vr_addr, ts->vr_size, (u8*)&vr_data);

    //---------------------------------
    len = scnprintf(buf, PAGE_SIZE,"vr : %d %d(%02x)\n", ts->vr_addr, vr_data, vr_data);
    return len;
}

/*------------------------------------------------------------------------------
 * vr interface : data write
 *-----------------------------------------------------------------------------*/
static ssize_t
w_vr_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct ist510e *ts = dev_get_drvdata(dev);
    u32 vr_addr = 0;
    u32 vr_data = 0;
    u32 vr_size = 0;
    u32 vr_flag = 0;

    sscanf (buf,"%d %d %d %d", &vr_addr, &vr_data, &vr_size, &vr_flag);
    if(vr_addr >= TOUCH_COUNT_ADDR)
      return 0;
    if(vr_size >  sizeof(vr_data))
      return 0;

    ts->vr_addr = vr_addr;
    ts->vr_size = vr_size;

    //---------------------------------
    if(vr_flag != 0)
        istcore_i2c_write(ts, vr_addr, vr_size, (u8*)&vr_data);

    return count;
}

/*Add Begin  by xucm1 20160623 feature: add anypen API  control pen function*/
#define IST940_ANYPEN_ADDR   (0x081D)
 int anypen_switch_operate(struct ist510e *ts, bool en)
{
	int ret;
	u8 cmd = 0;

	switch(en){
		case 0:
			cmd=0x00;
			ret=istcore_i2c_write(ts, IST940_ANYPEN_ADDR, 1 , &cmd);
			if(ret <0)
				printk("##IST940 disable anypen i2c error=%d\n",ret);

			ts->anypen_switch=0;
			break;
		case 1:
			cmd=0x01;
			ret=istcore_i2c_write(ts, IST940_ANYPEN_ADDR, 1 , &cmd);
			if(ret <0)
				printk("##IST940 enable anypen i2c error=%d\n",ret);

			ts->anypen_switch=1;
			break;
	}

	return ret;
}

static ssize_t anypen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int en;
	int ret;
	struct i2c_client *client =to_i2c_client(dev);
	struct ist510e *ts = i2c_get_clientdata(client);

	en  = simple_strtoul(buf, NULL, 10);
	if(en==0 || en==1)
		ret =anypen_switch_operate(ts,en);
	else
		printk("%s:set anypen enable/disable format error en=%d,buf=%s\n",__FUNCTION__,en,buf);

    return count;
}

static ssize_t
anypen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 len = 0;
	struct i2c_client *client =to_i2c_client(dev);
	struct ist510e *ts = i2c_get_clientdata(client);

    len = scnprintf(buf, PAGE_SIZE,"%d\n",ts->anypen_switch);

    return len;
}
/*Add end by xucm1 20160623*/

static ssize_t
version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    s32 ret = 0;
    u32 len = 0;
    struct ist510e *ts = dev_get_drvdata(dev);

    ret = ist_load_dwz(ts);
	if(!ts->suspended)
    {
      istcore_reset_ic();
	  istcore_init_mode(ts);
    }
    len = scnprintf(buf, PAGE_SIZE,"version: %d.%02x\nvendor: %02x\nDD version: %d.%02d\n",\
                    (ts->dwz_info.ver_v>>8)&0xff, (ts->dwz_info.ver_v)&0xff, ts->dwz_info.factory_id,\
                    HIDEEP_DD_VERSION_MAJOR,HIDEEP_DD_VERSION_MINOR);
    return len;
}

static ssize_t loglevel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE,"loglevel = %d\n", g_loglevel);
}

static ssize_t loglevel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int temp;

    ISTCORE_INFO("enter\n");
    if(sscanf (buf,"%d", &temp)!=1){
        return -EINVAL;
    }
    g_loglevel = temp;
    return count;
}

static ssize_t gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE,"gesture = %d(1: enable, 0: disable)\n", ts->gesture_enable);
}

static ssize_t gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int temp;

    ISTCORE_INFO("enter\n");
    if(ts->suspended == 1){
        return -EINVAL;
    }

    if(sscanf (buf,"%d", &temp)!=1){
        return -EINVAL;
    }

    ts->gesture_enable = temp ? 1 : 0;

    return count;
}

static ssize_t hideep_power_hal_suspend_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ist510e *ts = i2c_get_clientdata(client);

    return sprintf(buf, "%s\n",
        ts->power_hal_want_suspend ? "suspend" : "resume");
}

static ssize_t hideep_power_hal_suspend_store(struct device *dev,
                         struct device_attribute *attr,
                         const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ist510e *ts = i2c_get_clientdata(client);

    ISTCORE_INFO(": %s suspended %d \n", buf, ts->suspended);

    if (!strncmp(buf, POWER_HAL_SUSPEND_ON,
             POWER_HAL_SUSPEND_STATUS_LEN)) {
        hideep_power_hal_suspend(dev);
    } else {
        hideep_power_hal_resume(dev);
    }

    return count;
}

/*******************************************************************************
 * attribute group
 *******************************************************************************/
/*------------------------------------------------------------------------------
 * load_firmware : ucode fuse start
 *-----------------------------------------------------------------------------*/
static DEVICE_ATTR(selftest, S_IRWXUGO, hideep_self_test_read, NULL);
static DEVICE_ATTR(load_firmware  , S_IRWXUGO, read_ucode   , fuse_ucode);
static DEVICE_ATTR(update  , S_IRWXUGO, read_ucode   , fuse_ucode);
static DEVICE_ATTR(vr_data        , S_IRWXUGO, r_vr_data    , w_vr_data );
static DEVICE_ATTR(version        , S_IRWXUGO, version_show    , NULL );
static DEVICE_ATTR(loglevel       , S_IRWXUGO, loglevel_show    , loglevel_store);
static DEVICE_ATTR(gesture        , S_IRWXUGO, gesture_show    , gesture_store);
static DEVICE_ATTR(anypen        , S_IRWXUGO, anypen_show  , anypen_store);

#ifdef CONFIG_PM_SLEEP
static DEVICE_ATTR(power_HAL_suspend, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
                hideep_power_hal_suspend_show,
                hideep_power_hal_suspend_store);
#endif


/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static struct attribute *ist510e_ts_sysfs_entries[] =
{
    &dev_attr_load_firmware.attr,
    &dev_attr_selftest.attr       ,
    &dev_attr_update.attr       ,
    &dev_attr_vr_data.attr      ,
    &dev_attr_version.attr      ,
    &dev_attr_loglevel.attr     ,
    &dev_attr_gesture.attr      ,
    &dev_attr_anypen.attr,
#ifdef CONFIG_PM_SLEEP
    &dev_attr_power_HAL_suspend.attr,
#endif
    NULL
};

static struct attribute_group ist510e_ts_attr_group =
{
    .attrs  = ist510e_ts_sysfs_entries,
};


/*******************************************************************************
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
istcore_sysfs_init(struct ist510e *ts)
{
    int ret;
    struct  i2c_client *client = ts->client;

    /* Create the files associated with this kobject */
    ret = sysfs_create_group(&client->dev.kobj, &ist510e_ts_attr_group);
    ISTCORE_INFO("device : %s \n", client->dev.kobj.name);
    sysfs_create_link(NULL,&client->dev.kobj,"hideep");
    return ret;

}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
istcore_sysfs_exit(struct ist510e *ts)
{
    struct  i2c_client *client = ts->client;

    sysfs_remove_group(&client->dev.kobj, &ist510e_ts_attr_group);

    return 0;

}
