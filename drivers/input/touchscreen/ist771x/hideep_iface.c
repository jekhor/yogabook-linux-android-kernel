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
#include <linux/input/hideep_ts.h>
#include "hideep.h"
#include <linux/poll.h>
#ifdef HIDEEP_IF_DEVICE
/*******************************************************************************
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define ADDR_VR     (0x0000)
#define ADDR_VR_END (0x1000)
#define ADDR_IMG    (0x1000)
#define ADDR_UC     (0x10000000)
#define MAX_VR_BUFF  1024
#define IST510E_IF_DEVICE_NAME  "ist771x_debug"
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define IST510E_IOC_MAGIC  'k'
#define IST510E_CFG     _IOW(IST510E_IOC_MAGIC,  1, struct hideep_debug_cfg)
#define IST510E_RDIM    _IOW(IST510E_IOC_MAGIC,  2, int)
//#define IST510E_RDVR    _IO(IST510E_IOC_MAGIC,  3)
//#define IST510E_ENABLE  _IOR(SCULL_IOC_MAGIC,  2, int)
#define IST510E_IOC_MAXNR 4

#define OPM_MOD_CAP   0x81
#define OPM_TOUCH_A   0x00
 
/*******************************************************************************
 * basic operations..
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int32_t
hideep_get_vreg(struct hideep_data *ts,u32 addr, u32 len)
{
    int32_t ret = 0;
    struct i2c_client *client = ts->client;
    struct hideep_debug_dev  *debug_dev = &ts->debug_dev;
    ret = hideep_i2c_read(client, addr, len, debug_dev->vr_buff  );
    if(ret < 0)
        goto i2c_err;
    dbg_fun("hideep_get_vreg(0x%02x:%d)", addr, len);
    return ret;
    //---------------------------------
    // error recovery.
i2c_err:
    dbg_err("%s(%d) : i2c_err", __FUNCTION__, __LINE__);
    return ret;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int32_t
hideep_set_vreg(struct hideep_data *ts, u32 addr, u32 len)
{
    int32_t ret = 0;
    struct i2c_client *client = ts->client;
    struct hideep_debug_dev  *debug_dev = &ts->debug_dev;
    int32_t wr_remain = len;
    u32 vr_addr  = addr;
    int32_t wr_len = len;
    uint8_t* buff  = debug_dev->vr_buff;
    do
    {
        if(wr_remain >=  MAX_VR_BUFF)
            wr_len = MAX_VR_BUFF;
        else
            wr_len = wr_remain  ;
        //-----------------------------
        ret = hideep_i2c_write(client, vr_addr, wr_len, buff);
        if(ret < 0)
            goto i2c_err;
        wr_remain -= MAX_VR_BUFF;
        vr_addr   += MAX_VR_BUFF;
        buff      += MAX_VR_BUFF;
    }while(wr_remain>0);
    dbg_fun("hideep_set_vreg(0x%02x:%d)",  addr, len);
    return ret;
    //---------------------------------
    // error recovery.
i2c_err:
    dbg_err("%s(%d) : i2c_err", __FUNCTION__, __LINE__);
    return ret;
}
/*------------------------------------------------------------------------------
 * TODO : howto handle interrupt..
 *-----------------------------------------------------------------------------*/
static size_t
hideep_download_uc(struct hideep_data *ts, const char __user *uc, size_t count)
{
    int ret;
    uint8_t * ucode;
    ucode = kmalloc(count, GFP_KERNEL);
    ret = copy_from_user(ucode, uc, count);
    if (ret < 0)
    {
        dbg_err("ADDR_UC : copy_to_user");
        kfree(ucode);
        return 0;
    }
    //---------------------------------
    //ist_fuse_ucode(ts->client, uc, count);
    //---------------------------------
    kfree(ucode);
    dbg_fun("hideep_data_download_uc(%d)", count);
    return count;
}
/*******************************************************************************
 * file operations
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
hideep_iface_open(struct inode *inode, struct file *file)
{
    struct hideep_debug_dev *dev_info;
    //---------------------------------
    dbg_fun();
    dev_info = container_of(inode->i_cdev, struct hideep_debug_dev, acdev);
    if (dev_info == NULL)
    {
        dbg_err("No such char device node");
        return -ENODEV;
    }
    file->private_data = dev_info;
    dbg_fun("end");
    return 0;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
hideep_iface_release(struct inode *inode, struct file *file)
{
    dbg_fun();
    file->private_data = NULL;
    return 0;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static unsigned int
hideep_iface_poll(struct file *file, struct poll_table_struct *wait)
{
    u32 mask = 0;
    struct hideep_debug_dev *dev_info = file->private_data;
    
    dbg_fun();
    if(file->private_data == NULL)
    	return 0;
    poll_wait(file, &dev_info->i_packet, wait);
    if(dev_info->i_rdy)
    {
        mask |= POLLIN | POLLRDNORM;
        dev_info->i_rdy = 0;
    }
    return mask;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static ssize_t
hideep_iface_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    int ret;
    struct hideep_debug_dev *drv_info = file->private_data;
    ssize_t rd_len = 0;
    uint8_t* rd_buffer = NULL;
    //---------------------------------
    // vr read address
    dbg_fun("count = %d",count);
    if(file->private_data == NULL)
    	return 0;
    	
    if( (*offset) <ADDR_VR_END )
    {
        drv_info->vr_size = count;
        rd_buffer         = drv_info->vr_buff;
        rd_len            = count;
        ret = hideep_get_vreg(drv_info->ts, *offset, rd_len);
        if(ret < 0)
            rd_len = 0;
    }
    else
    //---------------------------------
    // im read address
    if( (*offset) == ADDR_IMG  )
    {
        drv_info->im_size = count;
        rd_buffer         = drv_info->im_buff;
        rd_len            = count;
    }
    else
    {
        dbg_err("hideep_data_read : undefined address");
        return 0;
    }

    //---------------------------------
    ret = copy_to_user(buf, rd_buffer, rd_len);
    if (ret < 0)
    {
        dbg_err("error : copy_to_user");
        return -EFAULT;
    }
    return rd_len;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static ssize_t
hideep_iface_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    int ret;
    struct hideep_debug_dev *drv_info = file->private_data;
    int wr_len = 0;
    //---------------------------------
    // vr write address
    dbg_fun("count = %d",count);
    if(file->private_data == NULL)
    	return 0;
    if( (*offset) <ADDR_VR_END )
    {
        wr_len = count;
        ret = copy_from_user(drv_info->vr_buff, buf, wr_len);
        if (ret < 0)
        {
            dbg_err("error : copy_to_user");
            return -EFAULT;
        }
        ret = hideep_set_vreg(drv_info->ts, *offset, wr_len);
        if(ret < 0)
            wr_len = 0;
    }
    else
    //---------------------------------
    if( (*offset) == ADDR_UC   )
    {
        wr_len = hideep_download_uc(drv_info->ts, buf, count);
    }
    else
    {
        dbg_err("hideep_data_write : undefined address");
        return 0;
    }
    //---------------------------------

    return wr_len;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static loff_t
hideep_iface_llseek(struct file *file, loff_t off, int whence)
{
    loff_t newpos;
    struct hideep_debug_dev *drv_info = file->private_data;
    
    
    dbg_fun("off = 0x%08x, whence = %d",(unsigned int)off,whence);
    if(file->private_data == NULL)
    	return -EFAULT;
    	
    switch(whence)
    {
        /* SEEK_SET */
        case 0  :
            newpos = off;
            break;
        /* SEEK_CUR */
        case 1  :
        	  dbg_fun("set mode off = 0x%08x",(unsigned int)off);
        	  if(off == 0x1000)
        	  {
		        	  drv_info->im_r_en = 1;
		        	  drv_info->vr_buff[0] = OPM_MOD_CAP;  // select frame mode...
		            hideep_set_vreg(drv_info->ts, 0x00, 1);
        	  }
        	  else if(off == 0x240)
        	  {
		        	  drv_info->im_r_en = 0;
		        	  drv_info->vr_buff[0] = OPM_TOUCH_A;
		            hideep_set_vreg(drv_info->ts, 0x00, 1);
        	  }
            newpos = file->f_pos;
        	  break;
        /* SEEK_END */
        case 2  :
            drv_info->im_size = off&0xffff;
            drv_info->vr_size = (off>>16)&0xffff;
            dbg_fun("hideep_IFACE_CFG : %d", drv_info->im_size);
            dbg_fun("hideep_IFACE_CFG : %d", drv_info->vr_size);
            newpos = file->f_pos;
            break;
        default:
            return -EINVAL;
    }
    if (newpos < 0)
        return -EINVAL;
    file->f_pos = newpos;
    return newpos;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static long
hideep_iface_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct hideep_debug_dev *drv_info = file->private_data;
    int err = 0;
    int ret = 0;
    struct hideep_debug_cfg config;
    /*
    * extract the type and number bitfields, and don't decode
    * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
    */
    dbg_fun();
    if(file->private_data == NULL)
    	return 0;
    	
    if (_IOC_TYPE(cmd) != IST510E_IOC_MAGIC) return -ENOTTY;
    if (_IOC_NR(cmd)    > IST510E_IOC_MAXNR) return -ENOTTY;
   /*
    * the direction is a bitmask, and VERIFY_WRITE catches R/W
    * transfers. `Type' is user-oriented, while
    * access_ok is kernel-oriented, so the concept of "read" and
    * "write" is reversed
    */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if(err)
        return -EFAULT;
    switch(cmd)
    {
        case IST510E_CFG:
        {
            ret = copy_from_user(&config , (void*)arg, sizeof(struct hideep_debug_cfg));
            drv_info->im_size = config.im_size;
            drv_info->vr_size = config.vr_size;
            dbg_fun("hideep_IFACE_CFG : %d", drv_info->im_size);
            dbg_fun("hideep_IFACE_CFG : %d", drv_info->vr_size);
            break;
        }
        case IST510E_RDIM:
        {
            ret = __get_user(drv_info->im_r_en , (uint8_t __user *)arg);
            //-----------------------------------
            // config vr to control ic
            if(drv_info->im_r_en != 0)
            {
                drv_info->vr_buff[0] = OPM_MOD_CAP;  // select frame mode...
            }
            else
            {
                drv_info->vr_buff[0] = OPM_TOUCH_A;
            }
            hideep_set_vreg(drv_info->ts, 0x00, 1);
            break;
        }
        default:  /* redundant, as cmd was checked against MAXNR */
            return -ENOTTY;
    }
    return ret;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static const struct file_operations hideep_data_debug_fops =
{
    .owner          = THIS_MODULE    ,
    .open           = hideep_iface_open   ,
    .poll           = hideep_iface_poll   ,
    .release        = hideep_iface_release,
    .read           = hideep_iface_read   ,
    .write          = hideep_iface_write  ,
    .llseek         = hideep_iface_llseek ,
    .unlocked_ioctl = hideep_iface_ioctl ,
};
/*******************************************************************************
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
void
hideep_iface_unregister(struct hideep_data *ts)
{
    struct hideep_debug_dev  *dev  = &ts->debug_dev;
    device_destroy(ts->debug_class, ts->debug_dev_no);
    cdev_del(&dev->acdev);
    if(dev->im_buff)
        kfree(dev->im_buff);
    if(dev->vr_buff)
        kfree(dev->vr_buff);
    return;
}
/*------------------------------------------------------------------------------
 * register character device
 *-----------------------------------------------------------------------------*/
int
hideep_iface_register(struct hideep_data *ts, u32 minor)
{
    int err = 0;
    struct device *device = NULL;
    struct hideep_debug_dev  *dev  = &ts->debug_dev;
    dev_t devno = ts->debug_dev_no;

    //-------------------------------------------
    // register driver
    cdev_init(&dev->acdev, &hideep_data_debug_fops);
    dev->acdev.owner = THIS_MODULE;
    err = cdev_add(&dev->acdev, ts->debug_dev_no, 1);
    if (err)
    {
        goto err;
    }
    //-------------------------------------------
    // add  device
    device = device_create(ts->debug_class,NULL,devno,NULL, IST510E_IF_DEVICE_NAME);
    if (IS_ERR(device))
    {
        err = PTR_ERR(device);
        cdev_del(&dev->acdev);
        goto err;
    }
    return 0;
err:
    dbg_err("hideep_data_debug_register failed");
    return err;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
void
hideep_iface_uninit(struct hideep_data *ts)
{
    /* Get rid of character devices (if any exist) */
    hideep_iface_unregister(ts);
    if(ts->debug_class)
        class_destroy(ts->debug_class);
    unregister_chrdev_region(ts->debug_dev_no, 1);
    return;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
hideep_iface_init(struct hideep_data *ts)
{
    int ret = 0;
    dev_t dev_id = 0;
    struct hideep_debug_dev *debug_drv = &ts->debug_dev;
    //---------------------------------
    // load device & driver
    ret = alloc_chrdev_region(&dev_id, 0, 1, IST510E_IF_DEVICE_NAME);
    if (ret < 0)
    {
        return ret;
    }
    ts->debug_dev_no = dev_id;
    ts->debug_class  = class_create(THIS_MODULE, IST510E_IF_DEVICE_NAME);
    if (IS_ERR(ts->debug_class))
    {
        ret = PTR_ERR(ts->debug_class);
        goto fail;
    }
    ret = hideep_iface_register(ts, 0);
    if (ret)
    {
        goto fail;
    }
    //---------------------------------
    // init debug driver
    init_waitqueue_head(&debug_drv->i_packet);
    debug_drv->ts         = ts;
    debug_drv->im_r_en    = 0;                                                  // disable
    debug_drv->im_size    = 4096;
    debug_drv->vr_size    = 4096;
    debug_drv->im_buff    = kmalloc(debug_drv->im_size, GFP_KERNEL);
    debug_drv->vr_buff    = kmalloc(debug_drv->vr_size, GFP_KERNEL);
    if(!debug_drv->im_buff || !debug_drv->vr_buff)
    {
        goto fail;
    }
    //---------------------------------
    dbg_fun("istxxxx_iface_init done");
    return 0;
    //---------------------------------
fail:
    hideep_iface_uninit(ts);
    pr_err("%s failed", __FUNCTION__);
    return ret;
}

#endif
