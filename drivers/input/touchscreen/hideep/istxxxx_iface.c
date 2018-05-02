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
#include <linux/poll.h>
#ifdef ISTCORE_IF_DEVICE

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

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define IST510E_IOC_MAGIC  'k'

#define IST510E_CFG     _IOW(IST510E_IOC_MAGIC,  1, struct ist510e_debug_cfg)
#define IST510E_RDIM    _IOW(IST510E_IOC_MAGIC,  2, int)
//#define IST510E_RDVR    _IO(IST510E_IOC_MAGIC,  3)
//#define IST510E_ENABLE  _IOR(SCULL_IOC_MAGIC,  2, int)

#define IST510E_IOC_MAXNR 4



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
istcore_get_vreg(struct ist510e *ts,u32 addr, u32 len)
{
    int32_t ret = 0;
    struct i2c_client *client = ts->client;
    struct ist510e_debug_dev  *debug_dev = &ts->debug_dev;

    ret = istcore_i2c_read(ts, addr, len, debug_dev->vr_buff  );
    if(ret < 0)
        goto i2c_err;

    ISTCORE_INFO("istcore_get_vreg(0x%02x:%d)\n", addr, len);
    return ret;

    //---------------------------------
    // error recovery.
i2c_err:
    dev_err(&client->dev, "%s(%d) : i2c_err\n", __FUNCTION__, __LINE__);

    return ret;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int32_t
istcore_set_vreg(struct ist510e *ts, u32 addr, u32 len)
{
    int32_t ret = 0;
    struct i2c_client *client = ts->client;
    struct ist510e_debug_dev  *debug_dev = &ts->debug_dev;
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
        ret = istcore_i2c_write(ts, vr_addr, wr_len, buff);
        if(ret < 0)
            goto i2c_err;

        wr_remain -= MAX_VR_BUFF;
        vr_addr   += MAX_VR_BUFF;
        buff      += MAX_VR_BUFF;

    }while(wr_remain>0);

    ISTCORE_INFO("istcore_set_vreg(0x%02x:%d)\n",  addr, len);
    return ret;

    //---------------------------------
    // error recovery.
i2c_err:
    dev_err(&client->dev, "%s(%d) : i2c_err\n", __FUNCTION__, __LINE__);

    ISTCORE_DBG("istcore_set_vreg\n");

    return ret;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static size_t
istcore_download_uc(struct ist510e *ts, const char __user *uc, size_t count)
{
    int ret;
    uint8_t * ucode;

    ucode = kmalloc(count, GFP_KERNEL);

    ret = copy_from_user(ucode, uc, count);
    if (ret < 0)
    {
        ISTCORE_ERR("ADDR_UC : copy_to_user\n");
        kfree(ucode);
        return 0;
    }
    //---------------------------------
#ifdef MTK_SLOUTION
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#else
    hideep_irq_enable(false);
#endif

    ist_fuse_ucode(ts->client, uc, count);

#ifdef MTK_SLOUTION
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#else
    
    hideep_irq_enable(true);
#endif
    //---------------------------------
    kfree(ucode);

    //ISTCORE_INFO("ist510e_download_uc(%d)\n", count);

    return count;
}

/*******************************************************************************
 * file operations
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
istcore_iface_open(struct inode *inode, struct file *file)
{
    struct ist510e_debug_dev *dev_info;

    //---------------------------------
    dev_info = container_of(inode->i_cdev, struct ist510e_debug_dev, cdev);
    if (dev_info == NULL)
    {
        ISTCORE_ERR("No such char device node\n");
        return -ENODEV;
    }

    file->private_data = dev_info;

    ISTCORE_INFO("istcore_iface_open\n");

    return 0;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
istcore_iface_release(struct inode *inode, struct file *file)
{
    ISTCORE_INFO("%s",__FUNCTION__);
    file->private_data = NULL;
    istcore_reset_ic();
    return 0;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static unsigned int
istcore_iface_poll(struct file *file, struct poll_table_struct *wait)
{
    u32 mask = 0;
    struct ist510e_debug_dev *dev_info = file->private_data;
    //ISTCORE_INFO("%s",__FUNCTION__);
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
istcore_iface_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    int ret;
    struct ist510e_debug_dev *drv_info = file->private_data;
    ssize_t rd_len = 0;
    uint8_t* rd_buffer = NULL;

    //---------------------------------
    // vr read address
    //ISTCORE_INFO("%s count = %d",__FUNCTION__,(int)count);
    if(file->private_data == NULL)
        return 0;

    if( (*offset) <ADDR_VR_END )
    {
        drv_info->vr_size = count;
        rd_buffer         = drv_info->vr_buff;
        rd_len            = count;

        ret = istcore_get_vreg(drv_info->ts, *offset, rd_len);
        if(ret < 0)
            rd_len = 0;
    }
    else
    //---------------------------------
    // im read address
    if( (*offset) == ADDR_IMG  )
    {
        ISTCORE_INFO("reading image\n");
        drv_info->im_size = count;
        rd_buffer         = drv_info->im_buff;
        rd_len            = count;
    }
    else
    {
        ISTCORE_ERR("ist510e_read : undefined address\n");
        return 0;
    }
    //---------------------------------
    ret = copy_to_user(buf, rd_buffer, rd_len);
    if (ret < 0)
    {
        ISTCORE_ERR("error : copy_to_user\n");
        return -EFAULT;
    }
    return rd_len;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static ssize_t
istcore_iface_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    int ret;
    struct ist510e_debug_dev *drv_info = file->private_data;
    int wr_len = 0;

    //---------------------------------
    // vr write address
    ISTCORE_INFO("%s count = %d\n",__FUNCTION__,(int)count);
    if(file->private_data == NULL)
        return 0;
    if( (*offset) <ADDR_VR_END )
    {
        wr_len = count;

        ret = copy_from_user(drv_info->vr_buff, buf, wr_len);
        if (ret < 0)
        {
            ISTCORE_ERR("error : copy_to_user\n");
            return -EFAULT;
        }

        ret = istcore_set_vreg(drv_info->ts, *offset, wr_len);
        if(ret < 0)
            wr_len = 0;

    }
    else
    //---------------------------------
    if( (*offset) == ADDR_UC   )
    {
        wr_len = istcore_download_uc(drv_info->ts, buf, count);
    }
    else
    {
        ISTCORE_ERR("ist510e_write : undefined address\n");
        return 0;
    }

    //---------------------------------


    return wr_len;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static loff_t
istcore_iface_llseek(struct file *file, loff_t off, int whence)
{
    loff_t newpos;
    struct ist510e_debug_dev *drv_info = file->private_data;

    ISTCORE_INFO("%s off = 0x%08x, whence = %d\n",__FUNCTION__,(unsigned int)off,whence);
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
              ISTCORE_INFO("%s set mode off = 0x%08x\n",__FUNCTION__,(unsigned int)off);
              if(off == 0x1000)
              {
                      drv_info->im_r_en = 1;
                      drv_info->vr_buff[0] = OPM_MOD_CAP;  // select frame mode...
                    istcore_set_vreg(drv_info->ts, 0x00, 1);
              }
              else if(off == 0x240)
              {
                      drv_info->im_r_en = 0;
                      drv_info->vr_buff[0] = OPM_TOUCH_A;
                    istcore_set_vreg(drv_info->ts, 0x00, 1);
              }
            newpos = file->f_pos;
              break;
        /* SEEK_END */
        case 2  :
            drv_info->im_size = off&0xffff;
            drv_info->vr_size = (off>>16)&0xffff;
            ISTCORE_INFO("%s ISTCORE_IFACE_CFG : %d\n", __FUNCTION__,drv_info->im_size);
            ISTCORE_INFO("%s ISTCORE_IFACE_CFG : %d\n", __FUNCTION__,drv_info->vr_size);
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
istcore_iface_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct ist510e_debug_dev *drv_info = file->private_data;
    int err = 0;
    int ret = 0;

    /*
    * extract the type and number bitfields, and don't decode
    * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
    */
    ISTCORE_INFO("%s",__FUNCTION__);
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
            struct ist510e_debug_cfg config;
            ret = copy_from_user(&config , (void*)arg, sizeof(struct ist510e_debug_cfg));

            drv_info->im_size = config.im_size;
            drv_info->vr_size = config.vr_size;

            ISTCORE_INFO("ISTCORE_IFACE_CFG : %d\n", drv_info->im_size);
            ISTCORE_INFO("ISTCORE_IFACE_CFG : %d\n", drv_info->vr_size);
            break;
        }
        case IST510E_RDIM:
        {
            ret = __get_user(drv_info->im_r_en , (uint8_t __user *)arg);

            //-----------------------------------
            // config vr to control ic
            if(drv_info->im_r_en != 0)
            {
                drv_info->vr_buff[0] = OPM_MOD_CAP;        // select frame mode...
            }
            else
            {
                drv_info->vr_buff[0] = OPM_TOUCH_A;
            }

            istcore_set_vreg(drv_info->ts, 0x00, 1);
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
static const struct file_operations ist510e_debug_fops =
{
    .owner          = THIS_MODULE    ,

    .open           = istcore_iface_open   ,
    .poll           = istcore_iface_poll   ,
    .release        = istcore_iface_release,
    .read           = istcore_iface_read   ,
    .write          = istcore_iface_write  ,
    .llseek         = istcore_iface_llseek ,
    .unlocked_ioctl = istcore_iface_ioctl ,
};

/*******************************************************************************
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static void
istcore_iface_unregister(struct ist510e *ts)
{
    struct ist510e_debug_dev  *dev  = &ts->debug_dev;

    device_destroy(ts->debug_class, ts->debug_dev_no);
    cdev_del(&dev->cdev);

    if(dev->im_buff)
        kfree(dev->im_buff);
    if(dev->vr_buff)
        kfree(dev->vr_buff);

    return;
}

/*------------------------------------------------------------------------------
 * register character device
 *-----------------------------------------------------------------------------*/
static int
istcore_iface_register(struct ist510e *ts, u32 minor)
{
    int err = 0;
    struct device *device = NULL;
    struct ist510e_debug_dev  *dev  = &ts->debug_dev;
    dev_t devno = ts->debug_dev_no;


    //-------------------------------------------
    // register driver
    cdev_init(&dev->cdev, &ist510e_debug_fops);
    dev->cdev.owner = THIS_MODULE;

    err = cdev_add(&dev->cdev, ts->debug_dev_no, 1);
    if (err)
    {
        goto err;
    }

    //-------------------------------------------
    // add  device
    device = device_create(ts->debug_class,NULL,devno,NULL, ISTCORE_IF_DEVICE_NAME);
    if (IS_ERR(device))
    {
        err = PTR_ERR(device);
        cdev_del(&dev->cdev);
        goto err;
    }
    ISTCORE_INFO("success!\n");
    return 0;

err:
    ISTCORE_ERR("ist510e_debug_register failed\n");
    return err;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
void
istcore_iface_uninit(struct ist510e *ts)
{

    /* Get rid of character devices (if any exist) */
    istcore_iface_unregister(ts);

    if(ts->debug_class)
        class_destroy(ts->debug_class);

    unregister_chrdev_region(ts->debug_dev_no, 1);
    return;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
istcore_iface_init(struct ist510e *ts)
{
    int ret = 0;
    dev_t dev_id = 0;
    struct ist510e_debug_dev *debug_drv = &ts->debug_dev;

    //---------------------------------
    // load device & driver
    ret = alloc_chrdev_region(&dev_id, 0, 1, ISTCORE_IF_DEVICE_NAME);
    if (ret < 0)
    {
        return ret;
    }
    ts->debug_dev_no = dev_id;
    ts->debug_class  = class_create(THIS_MODULE, ISTCORE_IF_DEVICE_NAME);
    if (IS_ERR(ts->debug_class))
    {
        ret = PTR_ERR(ts->debug_class);
        goto fail;
    }
    ret = istcore_iface_register(ts, 0);
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
    ISTCORE_INFO("istxxxx_iface_init....\n");
    return 0;

    //---------------------------------
fail:
    istcore_iface_uninit(ts);

    pr_err("%s failed\n", __FUNCTION__);

    return ret;
}


#endif


