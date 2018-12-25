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
#include "hideep.h"
/*******************************************************************************
 * sysfs subsystem 
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
extern struct hideep_data *private_ts;

#ifdef HIDEEP_SELF_TEST
static ssize_t hideep_self_test_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    int i;
    int count;

    dbg_fun();
    
	mutex_lock(&private_ts->io_lock); // Mutex lock - front of function
    hideep_test_mode(private_ts);
	mutex_unlock(&private_ts->io_lock); // Mutex lock - front of function
    ret = 0;
    //TX short
    for(count = 0, i = 0; i < TX_NUM; i++){
    	if(private_ts->TXshortResult[i]){
    		if(count ==0)
    		  dbg_err("TX[%d]\n",i);
    		else
    		  dbg_err(",TX[%d]\n",i);
    		count++;
            dbg_fun("tx short");
    		ret = -1;
    	}
    }
    if(count)
    	dbg_err( "TX(s) are short!\n");
    //TX open
    for(count = 0, i = 0; i < TX_NUM; i++){
    	if(private_ts->TXopenResult[i]){
    		if(count ==0)
    		  dbg_err( "TX[%d]\n",i);
    		else
    		  dbg_err( ",TX[%d]\n",i);
    		count++;
            dbg_fun("tx open");
    		ret = -1;
    	}
    }
    if(count)
    	dbg_err( "TX(s) are open!\n");
    	
   
    //RX short
    for(count = 0, i = 0; i < TX_NUM; i++){
    	if(private_ts->RXshortResult[i]){
    		if(count ==0)
    		  dbg_err( "RX[%d]\n",i);
    		else
    		  dbg_err( ",RX[%d]\n",i);
    		count++;
            dbg_fun("rx short");
    		ret = -1;
    	}
    }
    if(count)
    	dbg_err( "RX(s) are short!\n");
    //RX open
    for(count = 0, i = 0; i < TX_NUM; i++){
    	if(private_ts->RXopenResult[i]){
    		if(count ==0)
    		  dbg_err( "RX[%d]\n",i);
    		else
    		  dbg_err( ",RX[%d]\n",i);
    		count++;
            dbg_fun("rs open");
    		ret = -1;
    	}
    }
    if(count)
    	dbg_err( "RX(s) are open!\n");
    if(ret == 0)
        return scnprintf(buf, PAGE_SIZE, "PASS\n");
    return scnprintf(buf, PAGE_SIZE, "FAIL\n");
}
static DEVICE_ATTR(selftest, S_IRWXUGO, hideep_self_test_read, NULL);

#endif
static ssize_t hideep_update_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;

    dbg_fun();    
    ret = hideep_update(dev, HIDEEP_MANUAL_FW);
    if (ret<0)
    {
        dev_err(dev, "The firmware update failed(%d)\n", ret);
        count = ret;
    }
    return count;
}
static DEVICE_ATTR(update, S_IRWXUGO, NULL, hideep_update_write);

static ssize_t hideep_set_gpio_scl(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret,n;

    dbg_fun();
	n = simple_strtoul(buf, NULL, 0);

	switch (n) {
	case 1:
		printk("%s set scl 1\n");
		HIDEEP_SWD_CLK_SET(n);
		break;
	case 0:
		printk("%s set scl 0\n");
		HIDEEP_SWD_CLK_SET(n);
		break;
	default:
		printk("%s not support\n",__func__);
		break;
	}

    return count;
}
static DEVICE_ATTR(scl_gpio, S_IRWXUGO, NULL, hideep_set_gpio_scl);

static ssize_t hideep_set_gpio_sda(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret,n;

    dbg_fun();
	n = simple_strtoul(buf, NULL, 0);

	switch (n) {
	case 1:
		printk("%s set sda 1\n");
		HIDEEP_SWD_DAT_SET(n);
		break;
	case 0:
		printk("%s set sda 0\n");
		HIDEEP_SWD_DAT_SET(n);
		break;
	default:
		printk("%s not support\n",__func__);
		break;
	}

    return count;
}
static DEVICE_ATTR(sda_gpio, S_IRWXUGO, NULL, hideep_set_gpio_sda);

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct hideep_data *ts_drv = dev_get_drvdata(dev);
    unsigned char vs[10];

    if(check_version(ts_drv->client,vs)>=0){
        if((vs[8] & 0xf0) ==0)
            return scnprintf(buf, PAGE_SIZE, "[Hideep] O-film \nFW:v%d.%02d\n", 
                         vs[__HIDEEP_MAJOR_VERSION__+6], vs[__HIDEEP_MINOR_VERSION__+6]);
        else
            return scnprintf(buf, PAGE_SIZE, "[Hideep] Mutto \nFW:v%d.%02d\n", 
                         vs[__HIDEEP_MAJOR_VERSION__+6], vs[__HIDEEP_MINOR_VERSION__+6]);
    }
    else
        return scnprintf(buf, PAGE_SIZE, "can't read out version no.\n");
}
static DEVICE_ATTR(version, S_IRWXUGO, version_show, NULL);


static ssize_t driver_ver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "driver version = v%d.%02d.%02d\n",DD_VERSION_MAJOR,DD_VERSION_MIDDLE,DD_VERSION_MINOR);
}
static DEVICE_ATTR(driver_ver, S_IRWXUGO, driver_ver_show, NULL);

static ssize_t line_gap_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int n;
    struct hideep_data *ts_drv = dev_get_drvdata(dev);

    ts_drv->lines = simple_strtoul(buf, NULL, 0);
    return count;
}

static ssize_t line_gap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct hideep_data *ts_drv = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "line=%d\n",ts_drv->lines);
}
static DEVICE_ATTR(line_gap, S_IRWXUGO, line_gap_show, line_gap_write);



/*******************************************************************************
 * attribute group
 *******************************************************************************/
static struct attribute *hideep_ts_sysfs_entries[] =
{
    &dev_attr_update.attr,
    &dev_attr_version.attr,
    &dev_attr_driver_ver.attr,
    &dev_attr_scl_gpio.attr,    
	&dev_attr_sda_gpio.attr, 
#ifdef HIDEEP_SELF_TEST
    &dev_attr_selftest.attr,
#endif
    &dev_attr_line_gap.attr,
    NULL
};
static struct attribute_group hideep_ts_attr_group =
{
    .attrs  = hideep_ts_sysfs_entries,
};
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
hideep_sysfs_init(struct hideep_data *ts)
{
    int ret;
    struct  i2c_client *client = ts->client;
    /* Create the files associated with this kobject */
    ret = sysfs_create_group(&client->dev.kobj, &hideep_ts_attr_group);
    printk("device : %s \n", client->dev.kobj.name);
    return ret;

}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
hideep_sysfs_exit(struct hideep_data *ts)
{
    struct  i2c_client *client = ts->client;
    sysfs_remove_group(&client->dev.kobj, &hideep_ts_attr_group);
    return 0;
}
