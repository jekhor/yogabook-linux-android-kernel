/* drivers/input/touchscreen/HiDeep_ts.c
 *
 * Copyright (C) 2012 HiDeep, Inc.
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
 */


#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
//#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>  
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/gfp.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
//#include <mach/msm_iomap.h>
#include <linux/input/hideep_ts.h>
#include "hideep.h"
#include <uapi/linux/i2c.h>
//#include "Hideep_download.h"
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
//#include "../lct_tp_fm_info.h"

#define SLOT_TYPE					1
#define I2C_RETRY_CNT 				5

#define TOUCH_MAX_COUNT 			10 //Model Dependent

#define TOUCH_COUNT_ADDR 			0x0240
#define TOUCH_READ_START_ADDR 		0x0242 //Fixed value
#define TOUCH_READ_SIZE				10
#define TOUCH_KEY_READ_START_ADDR	(TOUCH_READ_START_ADDR + TOUCH_READ_SIZE * TOUCH_MAX_COUNT) //Fixed value
#define TOUCH_KEY_READ_SIZE			2

#define TOUCH_FLAG_MASK				0x10
#define TOUCH_DEEP_SLEEP_MASK		0x10

#define TOUCH_READ_REGS_LEN 		100 //Fixed value
#define TOUCH_WRITE_REGS_LEN 		16 //Fixed value

#define COMMAND0					0x080F
#if 0
#define TOUCH_READ_SW_VER_ADDR 		0x0080 //Model Dependent
#define TOUCH_READ_VR_VER_ADDR 		0x0098 //Model Dependent

#define HIDEEP_HW_REVISON 			0x01 //Model Dependent
#define HIDEEP_FW_MAJOR_VER			0x0001 //Model Dependent
#define HIDEEP_FW_MINOR_VER			0x01 //Model Dependent

#define HIDEEP_VR_MAJOR_VER			0x01 //Model Dependent
#define HIDEEP_VR_MINOR_VER			0x00 //Model Dependent

#define TX_CH_NUM					16 //Model Dependent
#define RX_CH_NUM					24 //Model Dependent
#define KEY_CH_NUM					2  //Model Dependent
#endif
#define DEBUG_PRINT 				0 //Model Dependent

#define HIDEEP_VTG_MIN_UV		3300000
#define HIDEEP_VTG_MAX_UV		3300000
#define HIDEEP_I2C_VTG_MIN_UV	1800000
#define HIDEEP_I2C_VTG_MAX_UV	1800000

#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"
#define PINCTRL_STATE_GPIO	     "pmx_ts_gpio"
#define PINCTRL_STATE_IIC	    "i2c_active"

#if SLOT_TYPE
#define REPORT_MT(touch_number, x, y, w, z) \
do {     \
	input_mt_slot(ts->input_dev, touch_number);	\
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);	\
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);             \
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);             \
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);         \
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, z); \
	input_report_key(ts->input_dev, BTN_TOUCH, 1); \
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1); \
} while (0)
#else
#define REPORT_MT(touch_number, x, y, w, z) \
do {     \
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, touch_number);\
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);             \
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);             \
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);         \
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, z); \
	input_mt_sync(ts->input_dev);                                      \
} while (0)
#endif

struct touch_event_reg{
	unsigned short		x;
	unsigned short		y;
	unsigned short		z;
	unsigned char		w;
	unsigned char		flag;
	unsigned char		type;
	unsigned char		index;
};

struct hideep_data *private_ts;

static struct finger_info gFinger_info[TOUCH_MAX_COUNT];
#ifdef SUPPORT_READ_TP_VERSION
	char tp_ver[50] = {0};
#endif


extern int tp_connected;
#ifdef CONFIG_LENOVO_SMART_PAD_SUPPORT
extern int notify_touch_event(int finger, int x, int y);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static void hideep_early_suspend(struct early_suspend *h);
static void hideep_late_resume(struct early_suspend *h);
#endif
static irqreturn_t hideep_irq_handler(int irq, void *handle);
static void hideep_int_enable(void *pdata, unsigned char enable)
{
	struct hideep_data *pts = pdata;
	if(enable){
		dbg_fun("Enable irq=%d \n",pts->client->irq);
		enable_irq(pts->client->irq);
	}
	else{
		dbg_fun("disable irq=%d \n",pts->client->irq);
		disable_irq(pts->client->irq);
	}
	return;
}

void hideep_reset(void)
{
	dbg_fun();
	gpio_direction_output(private_ts->pdata->gpio_reset,0);
	msleep(5);
	gpio_direction_output(private_ts->pdata->gpio_reset,1);
	return;
}
static void hideep_swd_dat(unsigned char send)
{
	gpio_direction_output(private_ts->pdata->gpio_int,send);
}
static unsigned char hideep_swd_dat_read(void)
{
	unsigned char ret;
	gpio_direction_input(private_ts->pdata->gpio_int);
	udelay(5);
	ret = gpio_get_value(private_ts->pdata->gpio_int);
	return (ret?1:0);
}

static void hideep_i2c_sda(unsigned char send)
{
	gpio_direction_output(private_ts->pdata->gpio_sda,send);
}
static unsigned char hideep_i2c_sda_read(void)
{
	unsigned char ret;
	gpio_direction_input(private_ts->pdata->gpio_sda);
	ret = gpio_get_value(private_ts->pdata->gpio_sda);
	return (ret?1:0);
}
static void hideep_swd_clk(unsigned char level)
{
	gpio_direction_output(private_ts->pdata->gpio_scl,level);
}

#define HIDEEP_CHANGE_I2C_GPIO_RETRY_TIME   5
#ifdef	HIDEEP_TP_VENDOR
int hideep_tp_vendor(void)
{
	unsigned char ret;
	int retry;
	int count;
	int old_status;
	dbg_fun();
	gpio_direction_input(private_ts->pdata->gpio_tp_vendor);
	old_status = gpio_get_value(private_ts->pdata->gpio_tp_vendor);
	for(retry = 0, count = 0;retry<5;retry++)
	{
		for(count = 0;count <10;count ++){
			mdelay(2);
			ret = gpio_get_value(private_ts->pdata->gpio_tp_vendor);
			if(ret == old_status)
				continue;
			old_status = ret;
			break;
		}
		if(count>=10)
			break;
		else
			ret = 1;
	}
	dbg_log("ret = %d\n",ret);
	return (ret?1:0);
}
#endif
static void hideep_swd_uninit(void)
{
	dbg_fun();
#if 1
	gpio_free(private_ts->pdata->gpio_sda);
    gpio_free(private_ts->pdata->gpio_scl);

/*restore gpio setting*/
	lnw_gpio_write_reg(private_ts->pdata->gpio_scl,&(private_ts->pdata->gpio_scl_value0),
	&(private_ts->pdata->gpio_scl_value1));
	lnw_gpio_write_reg(private_ts->pdata->gpio_sda,&(private_ts->pdata->gpio_sda_value0),
	&(private_ts->pdata->gpio_sda_value1));
	//lnw_gpio_lock(private_ts->pdata->gpio_scl,1);
#endif
	return;  
}

static int hideep_swd_init(void)
{
	int ret = 0;
    struct i2c_client *client = private_ts->client;
    struct device *dev = &client->dev;
    struct device_node *np = dev->of_node;

	u32 gpio_int_value0,gpio_int_value1;
	u32 ouput1=0x8003;
	u32 pad2=0x4c00001;

    dbg_fun();
	/*get orignal gpio setting*/
	lnw_gpio_read_reg(private_ts->pdata->gpio_scl,&(private_ts->pdata->gpio_scl_value0),
	&(private_ts->pdata->gpio_scl_value1));

	lnw_gpio_read_reg(private_ts->pdata->gpio_sda,&(private_ts->pdata->gpio_sda_value0),
	&(private_ts->pdata->gpio_sda_value1));

	lnw_gpio_read_reg(private_ts->pdata->gpio_int,&(gpio_int_value0),
	&(gpio_int_value1));
    /* scl, sda to gpio switching */
	//lnw_gpio_lock(private_ts->pdata->gpio_scl,0);

	ret = gpio_request(private_ts->pdata->gpio_scl, hideep_dd"_swd_clk_pin");
	if(ret <0){
		dbg_err("cannot control swd clk pin, ret = 0x%x", ret);
		goto err_swd_clk;
	}

	lnw_gpio_set_alt(private_ts->pdata->gpio_scl, HIDEEP_I2C_SCL_GPIO_MODE);	
	ret = gpio_direction_output(private_ts->pdata->gpio_scl,1);
	if(ret < 0)
	{
		dbg_err("gpio_direction_output, ret = 0x%x", ret);
		goto err_swd_clk_dir_output;
	}

	lnw_gpio_write_reg(private_ts->pdata->gpio_scl,&(ouput1),
	&(pad2));

	ret = gpio_request(private_ts->pdata->gpio_sda, hideep_dd"_i2c_sda_pin");
	if(ret <0){
		dbg_err("cannot control i2c dat pin, ret = 0x%x", ret);
		goto err_swd_clk;
	}
	
	lnw_gpio_set_alt(private_ts->pdata->gpio_sda, HIDEEP_I2C_SDA_GPIO_MODE);
	ret = gpio_direction_output(private_ts->pdata->gpio_sda,1);
	if(ret < 0) 
	{
		dbg_err("gpio_direction_output, ret = 0x%x", ret);
		goto err_swd_clk_dir_output;
	}
	lnw_gpio_write_reg(private_ts->pdata->gpio_sda,&(ouput1),
	&(pad2));
	dbg_fun("swd init done");
	return ret;
err_swd_clk_dir_output:
err_swd_clk:
	hideep_swd_uninit();

	return ret;
}

#ifdef HIDEEP_SELF_TEST
static int inline hideep_i2c_read_fifo(struct i2c_client *client, u16 addr, u16 len, u8 *buf)
{
    int ret = -1;
    u16 s_addr   = addr;
    u8 *s_buf    = buf ;
    s32  s_remain = len;
    u16 s_len    = len;
    
    //dbg_fun();
    ret = i2c_master_send(client, (char *) &s_addr, 2 );
    if (ret < 0)
    {
        goto err;
    }
    do
    {
        if(s_len >= MAX_TRANSACTION_LENGTH)
            s_len = MAX_TRANSACTION_LENGTH;

        ret = i2c_master_recv(client, (char *) s_buf,s_len);
        if (ret < 0)
        {
            goto err;
        }

        s_remain -= MAX_TRANSACTION_LENGTH;
        s_buf    += MAX_TRANSACTION_LENGTH;
        //s_addr   += MAX_TRANSACTION_LENGTH;
        s_len     = s_remain;
    }while(s_remain > 0);

 err:
    return ret;
}
#else
static int inline hideep_i2c_read_fifo(struct i2c_client *client, u16 addr, u16 len, u8 *buf)
{
    int ret = -1;
    u16 s_addr   = addr;
    u8 *s_buf    = buf ;
    s32  s_remain = len;
    u16 s_len    = len;
    
    //dbg_fun();
    do
    {
        if(s_len >= MAX_TRANSACTION_LENGTH)
            s_len = MAX_TRANSACTION_LENGTH;

        ret = i2c_master_send(client, (char *) &s_addr, 2 );
        if (ret < 0)
        {
            goto err;
        }
        ret = i2c_master_recv(client, (char *) s_buf,s_len);
        if (ret < 0)
        {
            goto err;
        }

        s_remain -= MAX_TRANSACTION_LENGTH;
        s_buf    += MAX_TRANSACTION_LENGTH;
        s_addr   += MAX_TRANSACTION_LENGTH;
        s_len     = s_remain;
    }while(s_remain > 0);

 err:
    return ret;
}
#endif
/*------------------------------------------------------------------------------
 * i2c write primitive
 *-----------------------------------------------------------------------------*/
static int inline hideep_i2c_write_fifo(struct i2c_client *client, u16 addr, u16 len, u8 *buf)
{
    s32 ret       = -1;
    u16 s_addr   = addr;
    u8 *s_buf    = buf ;
    s32  s_remain = len;
    u16 s_len    = len;
    
    //dbg_fun();
    do
    {
        if(s_len >= MAX_TRANSACTION_LENGTH)
            s_len = MAX_TRANSACTION_LENGTH-2;
        //-----------------------------
        // data mangling..
        private_ts->seg_buff[0] = (s_addr >> 0) & 0xFF;
        private_ts->seg_buff[1] = (s_addr >> 8) & 0xFF;
        memcpy( &private_ts->seg_buff[2], s_buf, s_len);

        ret = i2c_master_send(client, (char *) private_ts->seg_buff, s_len+2);
        if (ret < 0)
        {
            goto err;
        }
        dbg_log("ret = %d",ret);
        s_remain -= s_len;//MAX_TRANSACTION_LENGTH;
        s_buf    += s_len;//MAX_TRANSACTION_LENGTH;
        s_addr   += s_len;//MAX_TRANSACTION_LENGTH;
        s_len     = s_remain;

    }while(s_remain > 0);
    return 0;

err:
    dev_err(&client->dev, "%s(%d) : i2c_err\n", __FUNCTION__, __LINE__);
    return -1;

}

#ifdef I2C_DMA
/*------------------------------------------------------------------------------
 * i2c read primitive
 *-----------------------------------------------------------------------------*/
int hideep_i2c_read_dma(struct i2c_client *client, u16 addr, u16 len, u8 *buf)
{
    u16 s_addr   = addr;
    u8 *s_buf    = (u8*)&private_ts->dma_pa ;
    int s_remain = len;
    int s_len    = len;
    //int iread;
    struct i2c_msg msg[2] =
    {
        {
            .addr     = client->addr,
            .flags    = 0,
            .buf      = (u8*)&addr,
            .len      = 2
        },
        {
            .addr     = client->addr,
            .flags    = I2C_M_RD,
            .buf      = s_buf,
            .len      = s_len
        },
    };

    //---------------------------------
    // transmit data using DMA
    //dbg_fun("addr = 0x%x, len = %d", addr, len);
    if (i2c_transfer(client->adapter, &(msg[0]), 1) != 1)
    {  
         dbg_err("i2c transfer error!");
         goto err;
    }
    do
    {
        if(s_len >= MAX_I2C_DMA_LENGTH)
            s_len = MAX_I2C_DMA_LENGTH;

        msg[1].buf   = s_buf;
        msg[1].len   = s_len;


        //dbg_fun("len = %d", s_len);
        if (i2c_transfer(client->adapter, &(msg[1]), 1) != 1)
        {  
             dbg_err("i2c transfer error!");
             goto err;
         }
        s_remain -= MAX_I2C_DMA_LENGTH;
        s_buf    += MAX_I2C_DMA_LENGTH;
        s_addr   += MAX_I2C_DMA_LENGTH;
        s_len     = s_remain;
        //dbg_fun("len = %d, s_remain = %d", s_len , s_remain);


    }while(s_remain > 0);
    //dbg_fun();
    //---------------------------------
    memcpy(buf, private_ts->dma_va, len);

        //for(iread = 0;iread<MAX_I2C_DMA_LENGTH && iread<len;iread++)
           //dbg_log("buf[%d]=0x%x",iread,buf[iread]);
    //dbg_fun();
    return  0;

err:
    dbg_err();
    return -1;
}

/*------------------------------------------------------------------------------
 * i2c write primitive
 *-----------------------------------------------------------------------------*/
int hideep_i2c_write_dma(struct i2c_client *client, u16 addr, u16 len, u8 *buf)
{
    u16  s_addr   = addr;
    u8  *s_buf    = (u8*)&private_ts->dma_pa ;
    u32  s_remain = len;
    u16  s_len    = len;
    struct i2c_msg msg[1] =
    {
        {
            .addr     = client->addr,
            .flags    = 0,
            .buf      = s_buf,
            .len      = s_len
        }
    };

    //dbg_fun("addr = 0x%x, len = %d", addr, len);
    //---------------------------------
    memcpy(private_ts->dma_va, buf, len);
    //dbg_fun();

    //---------------------------------
    // transmit data using DMA
    do
    {
        if(s_len >= MAX_I2C_DMA_LENGTH)
            s_len = MAX_I2C_DMA_LENGTH;

        msg[0].buf   = s_buf;
        msg[0].len   = s_len;


        if (i2c_transfer(client->adapter, &(msg[0]), 1) != 1)
        {
             dbg_err("i2c transfer error!");
             goto err;
         }

        s_remain -= MAX_I2C_DMA_LENGTH;
        s_buf    += MAX_I2C_DMA_LENGTH;
        s_addr   += MAX_I2C_DMA_LENGTH;
        s_len     = s_remain;


    }while(s_remain > 0);
    //dbg_fun();
    return  0;
    
err:
    return -1;
}

#endif

/*------------------------------------------------------------------------------
 * i2c read
 *-----------------------------------------------------------------------------*/
int hideep_i2c_read(struct i2c_client *client, unsigned short addr, unsigned short length, unsigned char *buf)
{
    s32 ret       = -1;

#ifdef	I2C_MUTEX
    mutex_lock(&private_ts->i2c_mutex);
#endif
#ifdef I2C_DMA
    if(length > I2C_DMA_BUF_SIZE){
    		dbg_err("I2C dma over size.");
        goto err;
    }
    if(length <= MAX_TRANSACTION_LENGTH)
        ret = hideep_i2c_read_fifo(client, addr, length, buf);
    else
        ret = hideep_i2c_read_dma (client, addr, length, buf);
    if(ret < 0){
        goto err;
    }
#else
    ret = hideep_i2c_read_fifo(client, addr, length, buf);
    if (ret < 0){
        goto err;
    }
#endif
#ifdef	I2C_MUTEX
    mutex_unlock(&private_ts->i2c_mutex);
#endif
    return 0;
err:
#ifdef	I2C_MUTEX
    mutex_unlock(&private_ts->i2c_mutex);
#endif
    dbg_err("I2C read error.");
    return -1;
}

/*------------------------------------------------------------------------------
 *i2c write
 *-----------------------------------------------------------------------------*/
int hideep_i2c_write(struct i2c_client *client, unsigned short addr, unsigned short length, unsigned char *buf)
{
    s32 ret       = -1;

#ifdef	I2C_MUTEX
    mutex_lock(&private_ts->i2c_mutex);
#endif
#ifdef I2C_DMA
    if(length > I2C_DMA_BUF_SIZE){
        goto err;
		}
    if(length <= MAX_TRANSACTION_LENGTH)
        ret = hideep_i2c_write_fifo(client, addr, length, buf);
    else
        ret = hideep_i2c_write_dma (client, addr, length, buf);
    if(ret < 0){
        goto err;
		}
#else
    ret = hideep_i2c_write_fifo(client, addr, length, buf);
    if (ret < 0){
        goto err;
    }
#endif
#ifdef	I2C_MUTEX
    mutex_unlock(&private_ts->i2c_mutex);
#endif
    return 0;
    
err:
#ifdef	I2C_MUTEX
    mutex_unlock(&private_ts->i2c_mutex);
#endif
    dbg_err("I2C read error.");
    return -1;
}

/*
int hideep_i2c_read(struct i2c_client *client, unsigned short addr, unsigned short length, unsigned char *buf)
{
//    struct i2c_adapter *adapter = client->adapter;
//    struct i2c_msg msg;
    int ret = -1;

#if 0
    msg.addr = client->addr;
    msg.flags = 0x00;
    msg.len = 2;
    msg.buf = (unsigned char *) & addr;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = (unsigned char *) buf;

printk ("[ist771x] %s, %x\n", __FUNCTION__, client->addr);
    ret = i2c_transfer(adapter, &msg, 2);
#else
	ret = i2c_master_send(client, (char *) &addr, 2);
	if (ret < 0)
	{
		printk("[ist771x] : send error : [%d]", ret);
		return ret;
	}

	ret = i2c_master_recv(client, (char *) buf, length);
#endif
    if (ret < 0)
    {
        printk("[ist771x] : read error : [%d]", ret);
    }

    return ret;
}

int hideep_i2c_write(struct i2c_client *client, unsigned short addr, unsigned short length, unsigned char *buf) {
    int i;
    int ret = 0;
    char data[TOUCH_WRITE_REGS_LEN];

    data[0] = addr & 0xff;
    data[1] = (addr & 0xff00)>>8;    
    if (length > TOUCH_WRITE_REGS_LEN-2)
    {
        pr_err("[ist771x] %s :size error \n", __FUNCTION__);
        return -EINVAL;
    }
    for(i = 0; i < length ; i++)
        data[2+i] =*buf++;

    ret = i2c_master_send(client, (char *) &data, length+2);

    if (ret < 0)
    {
        printk("[ist771x] : send error : [%d]", ret);
        return ret;
    }

    if (ret == (length+2))
        return length;
    else
    {
        pr_err("[ist771x] :write error : [%d]", ret);
        return -EIO;
    }
}
*/
#if SLOT_TYPE
static void hideep_release_all_finger(struct hideep_data *ts)
{
    int i;
#if DEBUG_PRINT
    pr_info("[ist771x] %s\n", __func__);
#endif
	
	for (i = 0; i < TOUCH_MAX_COUNT; i++)
	{
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
#ifdef CONFIG_LENOVO_SMART_PAD_SUPPORT
		notify_touch_event(i, -1, -1);
#endif
		gFinger_info[i].w = 0;
		gFinger_info[i].z = 0;
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0); 
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0); 

	input_sync(ts->input_dev);
}
#else	
static void hideep_release_all_finger(struct hideep_data *ts)
{
#if DEBUG_PRINT
    pr_info("[ist771x] %s\n", __func__);
#endif

	int i;
	for(i=0; i<TOUCH_MAX_COUNT; i++)
	{
		if(-1 == gFinger_info[i].z)
			continue;

		if(gFinger_info[i].z == 0)
			input_mt_sync(ts->input_dev);

		if(0 == gFinger_info[i].z)
			gFinger_info[i].w = -1;
	}
	input_sync(ts->input_dev);
}
#endif

#if 0
static int check_vr(struct hideep_data *ts, unsigned char *val)
{
    int ret = 0;
    unsigned char i = 0;
	unsigned short vr_version;

    for (i = 0; i < I2C_RETRY_CNT; i++)
    {
        ret = hideep_i2c_read(ts->client, TOUCH_READ_VR_VER_ADDR, 2, &val[0]);

        if (ret >= 0)
        {
            pr_info("[ist771x] : VR Version[%03d.%d] \n", val[0], val[1]);
            break; // i2c success
        }
    }

    if (ret < 0)
    {
        pr_info("[ist771x] %s,%d: i2c read fail[%d] \n", __FUNCTION__, __LINE__, ret);
        return ret;
    }
}

static int vr_update(struct hideep_data *ts)
{
    int ret = 0;
    unsigned char vr_ver[2];

    struct hideep_data *info = ts;
    struct i2c_client *client = info->client;
    //struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

    ret = check_vr(ts, vr_ver);
    if (ret < 0)
        pr_err("[ist771x] check_vr fail! [%d]", ret);
    else
    {
    	if (vr_ver[0] < HIDEEP_VR_MAJOR_VER || vr_ver[0] == 0xFF)
    	{
    		//ret = update_from_vr_file (client);
    		ret = update_from_vr_binary (client);
    	}
    	else if (vr_ver[0] == HIDEEP_VR_MAJOR_VER)
    	{
    		if (vr_ver[1] < HIDEEP_VR_MINOR_VER)
    		{
    			//ret = update_from_vr_file (client);
    			ret = update_from_vr_binary (client);
    		}
    	}
		//Firmware update
		//TBD
    }

    return ret;
}

static int check_firmware(struct hideep_data *ts, unsigned char *val)
{
    int ret = 0;
    unsigned char i = 0;
	unsigned short fw_version;

    for (i = 0; i < I2C_RETRY_CNT; i++)
    {
        ret = hideep_i2c_read(ts->client, TOUCH_READ_SW_VER_ADDR, 3, &val[0]);
		
        if (ret >= 0)
        {
			memcpy((unsigned char *)&fw_version, &val[0], 2);
            pr_info("[ist771x] : FW Version[%03d.%d] \n", fw_version, val[2]);
            break; // i2c success
        }
    }

    if (ret < 0)
    {
        pr_info("[ist771x] %s,%d: i2c read fail[%d] \n", __FUNCTION__, __LINE__, ret);
        return ret;
    }
}

static int firmware_update(struct hideep_data *ts)
{
    int ret = 0;
    unsigned char fw_ver[3];
    unsigned short fw_major;
    unsigned char fw_minor;

    struct hideep_data *info = ts;
    struct i2c_client *client = info->client;
    //struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

    ret = check_firmware(ts, fw_ver);

    if (ret < 0)
        pr_err("[ist771x] check_firmware fail! [%d]", ret);
    else
    {
        memcpy((unsigned char *)&fw_major, &fw_ver[0], 2);
        fw_minor = fw_ver[2];

    	if (fw_major < HIDEEP_FW_MAJOR_VER || fw_major == 0xFFFF)
    	{
    		//ret = update_from_fw_file (client);
    		ret = update_from_fw_binary (client);
    	}
    	else if (fw_major == HIDEEP_FW_MAJOR_VER)
    	{
    		if (fw_minor < HIDEEP_FW_MINOR_VER)
    		{
    			//ret = update_from_fw_file (client);
    			ret = update_from_fw_binary (client);
    		}
    	}
    }

    return ret;
}
#endif
static int tsp_keycodes[4] = {KEY_MENU, KEY_HOME, KEY_SEARCH, KEY_BACK};

static void hideep_get_data(struct hideep_data *ts)
{
    int ret = 0, i;
    unsigned char buf[TOUCH_READ_REGS_LEN] = { 0, };
    int touch_count;
	unsigned char	index, type, flag;
	int key_count;
	unsigned char keycode, key_z;
	struct touch_event_reg tFinger;
    unsigned char total_count = 0;

#if DEBUG_PRINT
    pr_info("[ist771x] %s : \n", __FUNCTION__);
#endif
    mutex_lock(&ts->io_lock);
    if (ts == NULL)
        pr_info("[ist771x] %s: ts data is NULL \n", __FUNCTION__);

    for (i = 0; i < I2C_RETRY_CNT; i++)
    {
        ret = hideep_i2c_read(ts->client, TOUCH_COUNT_ADDR, 2, buf);

        if (ret >= 0)
        {
#if DEBUG_PRINT
            pr_info("[ist771x] : TOUCH_READ_REGS_LEN [%d] \n", ret);
#endif
            break; // i2c success
        }
    }

    if (ret < 0)
    {
        pr_info("[ist771x] %s,%d: i2c read fail[%u] \n", __FUNCTION__, __LINE__, ret);
        goto hideep_get_data_out;
    }
    else
    {
        touch_count = buf[0];
		key_count = buf[1] & 0x0F;
		//memcpy((unsigned char *)&touch_count, buf, 2);
#if DEBUG_PRINT
        pr_info("[ist771x] %s,%d: touch_count[%d] \n", __FUNCTION__, __LINE__, touch_count);
#endif
    }

//screen
    if (touch_count > 0 && touch_count < TOUCH_MAX_COUNT+1)
    {
        for (i = 0; i < I2C_RETRY_CNT; i++)
        {
            ret = hideep_i2c_read(ts->client, TOUCH_READ_START_ADDR, touch_count*TOUCH_READ_SIZE, buf);
            if (ret >= 0)
            {
#if DEBUG_PRINT
                pr_info("[ist771x] hideep_get_data : TS_READ_START_ADDR [%d] \n", ret);
#endif
                break; // i2c success
            }
        }

        if (ret < 0)
        {
            pr_info("[ist771x] %s,%d: i2c read fail[%u] \n", __FUNCTION__, __LINE__, ret);
            goto hideep_get_data_out;
        }
        else
        {
            for (i = 0; i < touch_count*TOUCH_READ_SIZE; i = i + TOUCH_READ_SIZE)
            {
				type = flag = 0;
				memset((unsigned char *)&tFinger, 0x0, sizeof(struct touch_event_reg));
				memcpy((unsigned char *)&tFinger, &buf[i], TOUCH_READ_SIZE);

                //dbg_log("x=%d,y=%d,z=%d,w=%d,flag=%d,type=%d,index=%d",tFinger.x,tFinger.y,                         tFinger.z,tFinger.w,tFinger.flag,tFinger.type,tFinger.index);
				index = tFinger.index;
				type = tFinger.type;
				flag = tFinger.flag;
				
				switch (type)					
				{
				case 0x00:
					break;
				case 0x01:
					break;
				case 0x02:
					break;
				}
				gFinger_info[index].x = tFinger.x;
				gFinger_info[index].y = tFinger.y;	
				gFinger_info[index].flag = tFinger.flag;			
					
				if(tFinger.flag & TOUCH_FLAG_MASK)
				{
					gFinger_info[index].w = 0;
					gFinger_info[index].z = 0;
				}
				else
				{
					gFinger_info[index].w = tFinger.w;
					gFinger_info[index].z = tFinger.z;
					total_count++;
				}
            }

          if (total_count == 0)
          {
	      hideep_release_all_finger(ts);
              //input_report_key(ts->input_dev, BTN_TOUCH, 0);
              //input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
#if !SLOT_TYPE
              input_mt_sync(ts->input_dev);
#endif
          }

          else
	  {
	     for (i = 0; i < TOUCH_MAX_COUNT; i++)
             {
//                if (gFinger_info[i].w == -1)
//                    continue;

#if SLOT_TYPE
				if(gFinger_info[i].w == 0 || (gFinger_info[i].flag & TOUCH_FLAG_MASK))
				{
				  // release event
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false); 
#ifdef CONFIG_LENOVO_SMART_PAD_SUPPORT
					notify_touch_event(i, -1,-1);
#endif
				}
				else
				{
					if (gFinger_info[i].y < ts->lines ) 
					REPORT_MT(i, gFinger_info[i].x, gFinger_info[i].y, gFinger_info[i].w, gFinger_info[i].z);
					else {
#ifdef CONFIG_LENOVO_SMART_PAD_SUPPORT
					notify_touch_event(i, gFinger_info[i].x,gFinger_info[i].y);
#endif
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false); 
					}
				}
#else
				if(gFinger_info[i].w == 0)
				{
					// release event    
					input_mt_sync(ts->input_dev);  
				}
				else
				{
					REPORT_MT(i, gFinger_info[i].x, gFinger_info[i].y, gFinger_info[i].w, gFinger_info[i].z);
				}
#endif
#if DEBUG_PRINT
                pr_info("[ist771x] %s: Touch ID: %d, State : %d, x: %d, y: %d, z: %d w: %d\n", __FUNCTION__,
                        i, (gFinger_info[i].z > 0), gFinger_info[i].x, gFinger_info[i].y, gFinger_info[i].z, gFinger_info[i].w);

#endif
//                if (gFinger_info[i].w == 0)
//                    gFinger_info[i].w = -1;
             }
          }
       }
    }
    else
    {
	if (touch_count == 0)
	    hideep_release_all_finger(ts);
    }
//key
	if (key_count > 0)
	{
		for (i = 0; i < I2C_RETRY_CNT; i++)
        {
            ret = hideep_i2c_read(ts->client, TOUCH_KEY_READ_START_ADDR, key_count*TOUCH_KEY_READ_SIZE, buf);
            if (ret >= 0)
            {
#if DEBUG_PRINT
                pr_info("[ist771x] hideep_get_data : TS_KEY_READ_START_ADDR [%d] \n", ret);
#endif
   	            break; // i2c success
       	    }
       	}

        if (ret < 0)
        {
            pr_info("[ist771x] %s,%d: i2c read fail[%u] \n", __FUNCTION__, __LINE__, ret);
            goto hideep_get_data_out;
        }
        else
        {
        	for (i = 0; i < key_count*TOUCH_KEY_READ_SIZE; i = i + TOUCH_KEY_READ_SIZE)
            {
        		keycode = buf[i];
				key_z = buf[i+1];
				
				input_report_key(ts->input_dev, tsp_keycodes[keycode], ((key_z > 0) ? 1 : 0));
        	}
        }
	}
	input_sync(ts->input_dev);
hideep_get_data_out:
    mutex_unlock(&ts->io_lock);
    return;
}

#ifdef HIDEEP_IF_DEVICE
static s32
hideep_get_image(struct hideep_data *ts)
{
    s32 ret = 0;
    struct i2c_client *client = ts->client;
    struct hideep_debug_dev  *debug_dev = &ts->debug_dev;
    ret = hideep_i2c_read(client, VR_ADDR_IMAGE, debug_dev->im_size, debug_dev->im_buff);
    if(ret < 0)
        goto i2c_err;
    dbg_fun( "load image from sensor(%d)", debug_dev->im_size);
    return ret;
i2c_err:
    dbg_err("i2c error");
    return ret;
}
#endif
static irqreturn_t hideep_irq_handler(int irq, void *handle)
{
    struct hideep_data *ts = (struct hideep_data *) handle;
    //dbg_fun();
    if(private_ts->suspend)
    	goto hideep_irq_handler_exit;
#ifdef HIDEEP_IF_DEVICE
    if(ts->debug_dev.im_r_en == 1)
    {
        dbg_fun("read image");
        hideep_get_image(ts);
        ts->debug_dev.i_rdy   = 1;                                                  // TODO : need semaphore..
        wake_up_interruptible(&ts->debug_dev.i_packet);
    }
    else
#endif
    {
        hideep_get_data(ts);
    }
hideep_irq_handler_exit:
    return IRQ_HANDLED;
}
#ifdef _HIDEEP_TEST_MODE
struct hideep_ts_test test;
#endif
#ifdef CONFIG_OF
static int hideep_ts_parse_dt(struct device *dev, struct hideep_platform_data *hideep_pdata){
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

    rc = of_property_read_u32(np, "hideep,version", &temp_val);
    if (!rc){
	  hideep_pdata->version = temp_val;
    }else if (rc != -EINVAL) {
		dev_err(dev, "Unable to read hideep,version\n");
		return rc;
	}
    
    rc = of_property_read_u32(np, "hideep,max-x", &temp_val);
    if (!rc){
	  hideep_pdata->max_x = temp_val;
    }else if (rc != -EINVAL) {
		dev_err(dev, "Unable to read hideep,max-x\n");
		return rc;
	}
    
    rc = of_property_read_u32(np, "hideep,max-y", &temp_val);
    if (!rc){
	  hideep_pdata->max_y = temp_val;
    }else if (rc != -EINVAL) {
		dev_err(dev, "Unable to read hideep,max-y\n");
		return rc;
	}
    
    rc = of_property_read_u32(np, "hideep,max-z", &temp_val);
    if (!rc){
      hideep_pdata->max_z = temp_val;
    }else if (rc != -EINVAL) {
		dev_err(dev, "Unable to read hideep,max-z\n");
		return rc;
	}

    rc = of_property_read_u32(np, "hideep,max-w", &temp_val);
    if (!rc){
	  hideep_pdata->max_w = temp_val;
    }else if (rc != -EINVAL) {
		dev_err(dev, "Unable to read hideep,max-w\n");
		return rc;
	}
	/* power ldo gpio info*/
	hideep_pdata->power_ldo_gpio = of_get_named_gpio_flags(np, "hideep,power_ldo-gpio",
				0, NULL);
	/* reset, irq gpio info */
	hideep_pdata->gpio_reset= of_get_named_gpio_flags(np,
			"hideep,reset-gpio", 0, NULL);
	hideep_pdata->gpio_int= of_get_named_gpio_flags(np,
			"hideep,irq-gpio", 0, NULL);

			#ifdef	HIDEEP_TP_VENDOR
	hideep_pdata->gpio_tp_vendor = of_get_named_gpio_flags(np,
			"hideep,tp-vendor-gpio", 0, NULL);
			#endif
    hideep_pdata->int_enable = hideep_int_enable;
	hideep_pdata->reset = hideep_reset;
	hideep_pdata->swd_init = hideep_swd_init;
	hideep_pdata->swd_uninit = hideep_swd_uninit;
	hideep_pdata->swd_dat = hideep_swd_dat;
	hideep_pdata->swd_dat_read = hideep_swd_dat_read;
	hideep_pdata->swd_clk = hideep_swd_clk;
	hideep_pdata->i2c_sda = hideep_i2c_sda;
	hideep_pdata->i2c_sda_read = hideep_i2c_sda_read;
	hideep_pdata->i2c_scl = hideep_swd_clk;
	#ifdef HIDEEP_TP_VENDOR
	hideep_pdata->tp_vendor = hideep_tp_vendor;
	#endif
	return 0;
}
#else
static inline int hideep_ts_parse_dt(struct device *dev,
				struct hideep_platform_data *hideep_pdata)
{
	hideep_pdata->int_enable = hideep_int_enable;
	hideep_pdata->reset = hideep_reset;
	hideep_pdata->swd_init = hideep_swd_init;
	hideep_pdata->swd_uninit = hideep_swd_uninit;
	hideep_pdata->swd_dat = hideep_swd_dat;
	hideep_pdata->swd_dat_read = hideep_swd_dat_read;
	hideep_pdata->swd_clk = hideep_swd_clk;
	hideep_pdata->i2c_sda = hideep_i2c_sda;
	hideep_pdata->i2c_sda_read = hideep_i2c_sda_read;
	hideep_pdata->i2c_scl = hideep_swd_clk;

	return 0;
}
#endif

static int hideep_power_on(struct hideep_data *data, bool on)
{
	int rc=0;

	if (!on)
		goto power_off;

    if (gpio_is_valid(data->pdata->power_ldo_gpio))
	{
		dbg_fun("power_ldo_gpio\n");
		gpio_set_value(data->pdata->power_ldo_gpio, 1);
	}
    
  	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
        return rc;
	}
	if(data->vdd2){
		rc = regulator_enable(data->vdd2);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd2 enable failed rc=%d\n", rc);
	        return rc;
		}
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
    
    if (gpio_is_valid(data->pdata->power_ldo_gpio))
	{
		gpio_set_value(data->pdata->power_ldo_gpio, 0);
	}
    
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
	if(data->vdd2){
		rc = regulator_disable(data->vdd2);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd2 disable failed rc=%d\n", rc);
	        return rc;
		}
	}
    return 0;
    
	
}

static int hideep_power_init(struct hideep_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;
#if 0
    if (gpio_is_valid(data->pdata->power_ldo_gpio))
	{
		printk("%s, power_ldo_gpio\n", __func__);
		rc = gpio_request(data->pdata->power_ldo_gpio, "hideep_ldo_gpio");
		if (rc)
		{
			printk("power gpio request failed\n");
			return rc;
		}
		
		rc = gpio_direction_output(data->pdata->power_ldo_gpio, 1);
		if (rc)
		{
			printk("set_direction for irq gpio failed\n");
			goto free_ldo_gpio;
		}
	}
#endif

#ifndef EIGHT_PIN_SOLUTION

	data->vdd = regulator_get(&data->client->dev, "VPROG6A");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator VPROG6A 3.3 v get failed vdd rc=%d\n", rc);
		goto err;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, HIDEEP_VTG_MIN_UV,
					   HIDEEP_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg VPROG6A failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	if (yeti_hw_ver >= 2)
		data->vdd2 = regulator_get(&data->client->dev, "VPROG3B");
	else
		data->vdd2 = regulator_get(&data->client->dev, "VPROG4D");
	if (IS_ERR(data->vdd2)) {
		rc = PTR_ERR(data->vdd2);
		dev_err(&data->client->dev,
			"Regulator vdd get failed vdd rc=%d\n", rc);
		goto reg_vdd_put;
	}

	if (regulator_count_voltages(data->vdd2) > 0) {
		rc = regulator_set_voltage(data->vdd2, HIDEEP_VTG_MIN_UV,
					   HIDEEP_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd2_put;
		}
	}
	data->vcc_i2c = regulator_get(&data->client->dev, "VPROG5B");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd2_put;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, HIDEEP_I2C_VTG_MIN_UV,
					   HIDEEP_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
#else
	data->vdd = regulator_get(&data->client->dev, "VPROG3B");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator VPROG6A 3.3 v get failed vdd rc=%d\n", rc);
		goto err;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, HIDEEP_VTG_MIN_UV,
					   HIDEEP_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg VPROG6A failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}
	data->vdd2=NULL;
	data->vcc_i2c = regulator_get(&data->client->dev, "VPROG5A");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd2_put;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, HIDEEP_I2C_VTG_MIN_UV,
					   HIDEEP_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

#endif

	return 0;
    

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
	
#if 0
free_ldo_gpio:	
	if (gpio_is_valid(data->pdata->power_ldo_gpio))
		gpio_free(data->pdata->power_ldo_gpio); 
#endif
reg_vdd2_put:
	if(data->vdd2){
		regulator_put(data->vdd2);
	}
reg_vdd_put:
	regulator_put(data->vdd);
err:
	return rc;

pwr_deinit:
    if (gpio_is_valid(data->pdata->power_ldo_gpio))
		gpio_free(data->pdata->power_ldo_gpio);    
    
	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, HIDEEP_I2C_VTG_MAX_UV);
		regulator_put(data->vcc_i2c);
		
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, HIDEEP_VTG_MAX_UV);
		regulator_put(data->vdd);
		
	if (data->vdd2){
		regulator_set_voltage(data->vdd2, 0, HIDEEP_VTG_MAX_UV);
		regulator_put(data->vdd2);
	}


	return 0;
}

static int hideep_pinctrl_init(struct hideep_data *hideep_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	hideep_data->ts_pinctrl = devm_pinctrl_get(&(hideep_data->client->dev));
	if (IS_ERR_OR_NULL(hideep_data->ts_pinctrl)) {
		retval = PTR_ERR(hideep_data->ts_pinctrl);
		dev_dbg(&hideep_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	hideep_data->pinctrl_state_active
		= pinctrl_lookup_state(hideep_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(hideep_data->pinctrl_state_active)) {
		retval = PTR_ERR(hideep_data->pinctrl_state_active);
		dev_err(&hideep_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	hideep_data->pinctrl_state_suspend
		= pinctrl_lookup_state(hideep_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(hideep_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(hideep_data->pinctrl_state_suspend);
		dev_err(&hideep_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

    hideep_data->pinctrl_state_gpio
		= pinctrl_lookup_state(hideep_data->ts_pinctrl,
			PINCTRL_STATE_GPIO);
	if (IS_ERR_OR_NULL(hideep_data->pinctrl_state_gpio)) {
		retval = PTR_ERR(hideep_data->pinctrl_state_gpio);
		dev_err(&hideep_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_GPIO, retval);
		goto err_pinctrl_lookup;
	}

    hideep_data->pinctrl_state_iic
		= pinctrl_lookup_state(hideep_data->ts_pinctrl,
			PINCTRL_STATE_IIC);
	if (IS_ERR_OR_NULL(hideep_data->pinctrl_state_iic)) {
		retval = PTR_ERR(hideep_data->pinctrl_state_iic);
		dev_err(&hideep_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_IIC, retval);
		goto err_pinctrl_lookup;
	}
    
	hideep_data->pinctrl_state_release
		= pinctrl_lookup_state(hideep_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(hideep_data->pinctrl_state_release)) {
		retval = PTR_ERR(hideep_data->pinctrl_state_release);
		dev_dbg(&hideep_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(hideep_data->ts_pinctrl);
err_pinctrl_get:
	hideep_data->ts_pinctrl = NULL;
	return retval;
}

#if 0
static int hideep_pinctrl_select(struct hideep_data *hideep_data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? hideep_data->pinctrl_state_active
		: hideep_data->pinctrl_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(hideep_data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&hideep_data->client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(&hideep_data->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}
#endif
static int hideep_gpio_setup(int gpio, bool configure, int dir, int state)
{
	int ret = 0;
	unsigned char buf[16];

	printk("%s, gpio=%d, configure=%d, dir=%d, state=%d\n", __func__, gpio, configure, dir, state);

	if (configure) {
		snprintf(buf, PAGE_SIZE, "dsx_gpio_%u\n", gpio);

		ret = gpio_request(gpio, buf);
		
	        if (ret) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, ret);
			return ret;
		}

		if (dir == 0)
			ret = gpio_direction_input(gpio);
		else
			ret = gpio_direction_output(gpio, state);
		if (ret) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return ret;
		}
	} else {
		gpio_free(gpio);
	}

	return ret;
}

static int hideep_suspend(struct i2c_client *client, pm_message_t mesg)
{
	
    unsigned char buf[3];
    unsigned short addr = COMMAND0;
    int ret = 0;

    dbg_fun();
    if(private_ts->updating == 1)
        goto hideep_suspend_end_updating;
    if(private_ts->suspend)
        goto hideep_suspend_end;
    private_ts->suspend = 1;
    private_ts->resume = 0;
    mutex_lock(&private_ts->io_lock); // Mutex lock - front of function
    hideep_release_all_finger(private_ts);
    private_ts->pdata->int_enable(private_ts, 0);
    //   cancel_work_sync(&private_ts->work); // cancel proceeding work thread as soon as  disable irq

	//private_ts->pdata->reset();
	msleep(10);
    buf[0] = TOUCH_DEEP_SLEEP_MASK;
    ret = hideep_i2c_write(private_ts->client,addr, 1, buf);
    //msleep(5);
    //hideep_power_on(private_ts, false);
    dbg_log("ret = %d", ret);
hideep_suspend_end:
    mutex_unlock(&private_ts->io_lock); // Mutex lock - end of function
hideep_suspend_end_updating:
    dbg_fun();
    return 0;
}

static int hideep_resume(struct i2c_client *client)
{
    unsigned char buf[2] = {0,};
    unsigned short addr = 0;
    int ret = 0;

    dbg_fun();
    if(private_ts->updating == 1)
        goto hideep_resume_end_updating;
	mutex_lock(&private_ts->io_lock); // Mutex lock - front of function
    if(private_ts->resume)
        goto hideep_resume_end;
    private_ts->suspend = 0;
    private_ts->resume = 1;
   // hideep_power_on(private_ts, true);
   // msleep(5);
	private_ts->pdata->reset();
	msleep(10);
    ret = hideep_i2c_write(private_ts->client, addr, 1, buf);
    dbg_log("ret = %d", ret);
    private_ts->pdata->int_enable(private_ts, 1);
hideep_resume_end:
    mutex_unlock(&private_ts->io_lock); // Mutex lock - end of function
hideep_resume_end_updating:
    dbg_fun();
    return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct hideep_data *hideep_data =
		container_of(self, struct hideep_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			hideep_data && hideep_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			hideep_resume(hideep_data->client);
		else if (*blank == FB_BLANK_POWERDOWN)
			hideep_suspend(hideep_data->client, PMSG_SUSPEND);
	}


	return 0;
}
#endif
static int hideep_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct hideep_data *ts;
	struct hideep_platform_data *platform_data;
	
    int ret = 0, i;
    unsigned char vs[10];

    printk("[ist771x] %s\n", __func__);
    if (tp_connected)
		return -ENODEV;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        pr_info("hideep_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

    ts = kmalloc(sizeof(struct hideep_data), GFP_KERNEL);

    if (ts == NULL)
    {
        pr_info("[ist771x] %s: failed to create a state of hideep-ts\n", __FUNCTION__);
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }
    if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(struct hideep_platform_data),
			GFP_KERNEL);
		if (!platform_data) {
			ret = -ENOMEM;
            goto err_mem_alloc_failed;
		}

		ret = hideep_ts_parse_dt(&client->dev, platform_data);
		if (ret)
			goto err_ts_parse_dt_failed;
	} else {
	   platform_data = client->dev.platform_data;
	   ret = hideep_ts_parse_dt(&client->dev, platform_data);
		if (ret)
			goto err_ts_parse_dt_failed;
	}
	
	ts->client = client;
    ts->pdata = platform_data;
#ifdef I2C_DMA
	dbg_fun();
    ts->dma_va = dma_alloc_coherent(NULL, I2C_DMA_BUF_SIZE, &ts->dma_pa, GFP_KERNEL);
    if(!ts->dma_va)
    {
         dbg_err("Allocate DMA I2C Buffer failed!");
         goto err_input_dev_mem;
    }
	dbg_fun();
#endif
	//if (ts->pdata->power)
//		ts->power = ts->pdata->power;
	
    //ts->client = client;
    i2c_set_clientdata(client, ts);
    ts->resume = 0;
    ts->suspend = 0;

    private_ts = ts;
    private_ts->wait_int = 0;
    private_ts->updating = 0;
    private_ts->lines = 941;
    mutex_init(&ts->io_lock);
#ifdef	I2C_MUTEX
    mutex_init(&ts->i2c_mutex);
#endif
    ret = hideep_power_init(ts, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto err_power_init_failed;
	}
    ret = hideep_power_on(ts, true);
		if (ret) {
			dev_err(&client->dev, "power on failed");
			goto err_power_on_failed;
		}
    ts->ts_pinctrl = NULL;
#if 0
    ret = hideep_pinctrl_init(ts);
    if (!ret && ts->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		ret = pinctrl_select_state(ts->ts_pinctrl,
					ts->pinctrl_state_active);
		if (ret < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
		}
	}
	#endif
	
       //setting gpio
       
       if(gpio_is_valid(ts->pdata->gpio_reset)){
           ret = hideep_gpio_setup(ts->pdata->gpio_reset,true,1,0);
           if (ret) {
			dev_err(&client->dev,
				"first setup for reset gpio failed\n");
			goto free_reset_gpio;
		   }
           msleep(10);
           ret = hideep_gpio_setup(ts->pdata->gpio_reset,false,1,0);
           if (ret) {
			dev_err(&client->dev,
				"second setup for reset gpio failed\n");
			goto free_reset_gpio;
		   }
           msleep(10);
           ret = hideep_gpio_setup(ts->pdata->gpio_reset,true,1,1);
           if (ret) {
			dev_err(&client->dev,
				"third setup for reset gpio failed\n");
			goto free_reset_gpio;
		   }
       }
       if(gpio_is_valid(ts->pdata->gpio_int)){
           ret = hideep_gpio_setup(ts->pdata->gpio_int,true,0,0);
           if (ret) {
			dev_err(&client->dev,
				"setup for irq gpio failed\n");
			goto free_irq_gpio;
		   }
           msleep(20);
	       ts->client->irq = gpio_to_irq (ts->pdata->gpio_int);
	}
      #ifdef	HIDEEP_TP_VENDOR
      if(gpio_is_valid(ts->pdata->gpio_tp_vendor)){
       		ret = hideep_gpio_setup(ts->pdata->gpio_tp_vendor,true,0,0);
          if (ret) {
						dev_err(&client->dev,"setup for tp vendor gpio failed\n");
						goto free_tp_vendor_gpio;
		  		}
			}
			#endif	

    if(check_version(ts->client,vs) < 0){
	printk("no touch connected ist\n");
	ret = -ENODEV;
	goto free_irq_gpio;
    }
    ts->input_dev = input_allocate_device();
	
    if (!ts->input_dev)
    {
        pr_info("[ist771x] %s: Not enough memory\n", __FUNCTION__);
        ret = -ENOMEM;
        goto err_input_dev_alloc_failed;
    }

	ts->input_dev->name = "hideep-ist771x";

    __set_bit(EV_ABS,  ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
	__set_bit(BTN_TOOL_FINGER, ts->input_dev->keybit);

	input_mt_init_slots(ts->input_dev, TOUCH_MAX_COUNT, 0);
	
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->pdata->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->pdata->max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, ts->pdata->max_w, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, ts->pdata->max_z, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, TOUCH_MAX_COUNT-1, 0, 0);

	__set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	
    ret = input_register_device(ts->input_dev);
	
    if (ret)
    {
        pr_info("[ist771x] %s: Failed to register device\n", __FUNCTION__);
        ret = -ENOMEM;
        goto err_input_register_device_failed;
    }
 
    printk("%s name = %s\n",__func__, ts->client->name);
    if (ts->client->irq)
    {
        ret = request_threaded_irq(ts->client->irq, NULL, hideep_irq_handler, /*IRQF_ONESHOT | IRQ_TYPE_EDGE_FALLING */(IRQF_TRIGGER_FALLING | IRQF_ONESHOT), ts->client->name, ts);
        printk("%s, request_threaded_irq, ret = 0x%x\n",__func__, ret);
        if (ret > 0)
        {
            pr_info("[ist771x] %s: Can't allocate irq %d, ret %d\n", __FUNCTION__, ts->client->irq, ret);
            ret = -EBUSY;
            goto err_request_irq;
        }
    }

    for (i = 0; i < TOUCH_MAX_COUNT; i++) /* _SUPPORT_MULTITOUCH_ */
    {
	gFinger_info[i].z = 0;
	gFinger_info[i].w = 0;
    }
#ifdef SUPPORT_READ_TP_VERSION
	memset(tp_ver, 0, sizeof(tp_ver));
    if(check_version(ts->client,vs)>=0){
        if((vs[8] & 0xf0) ==0)
            sprintf(tp_ver, "[Hideep] O-film \nFW:v%d.%02d", 
                         vs[__HIDEEP_MAJOR_VERSION__+6], vs[__HIDEEP_MINOR_VERSION__+6]);
        else
            sprintf(tp_ver, "[Hideep] Mutto \nFW:v%d.%02d", 
                         vs[__HIDEEP_MAJOR_VERSION__+6], vs[__HIDEEP_MINOR_VERSION__+6]);
    }else{
        sprintf(tp_ver, "can't read out version no.");
    }
	init_tp_fm_info(0,tp_ver,"IST700");
#endif
#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;

	ret = fb_register_client(&ts->fb_notif);

	if (ret)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			ret);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = hideep_early_suspend;
    ts->early_suspend.resume = hideep_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

#ifdef _HIDEEP_TEST_MODE
    test.get_data = kzalloc(4096, GFP_KERNEL);
	test.client = client;
	test.pdata = ts->pdata;
	test.tx_num = TX_CH_NUM;
	test.rx_num = RX_CH_NUM;
	test.key_num = KEY_CH_NUM;
	test.irq = ts->irq;
#endif

    	//manually update.
     ret = hideep_sysfs_init(ts);                      // sysfs export..
     if(ret){
     		dbg_err("CANNOT create control file nood"); 
        goto err_sysfs_failed;
     }
#ifdef HIDEEP_IF_DEVICE
    ret = hideep_iface_init(ts);
    if(ret < 0)
    {
        dbg_fun("debug interface device error");    
        goto err_create_fw_wq_failed_device;
    }
#endif
#if 0
		//automatically update.
    //private_ts->updating = 1;
    ts->hideep_fw_wq = create_singlethread_workqueue("hideep_fw_wq");
    if (!ts->hideep_fw_wq) {
        dbg_err("create workqueue failed");
        ret  = -ENOMEM;
        goto err_create_fw_wq_failed;
    }
    INIT_DELAYED_WORK(&ts->fw_work, hideep_update_fw_thread);
    queue_delayed_work(ts->hideep_fw_wq, &ts->fw_work,20);
#endif
    printk("[ist771x] %s: Start touchscreen. name: %s, irq: %d\n", __FUNCTION__, ts->client->name, ts->client->irq);

	printk("test \n");

    tp_connected = 1;	
    return 0;
err_create_fw_wq_failed:
#ifdef HIDEEP_IF_DEVICE
	  hideep_iface_uninit(ts);
err_create_fw_wq_failed_device:
#endif
		hideep_sysfs_exit(ts);
err_sysfs_failed:
		input_unregister_device(ts->input_dev);

err_request_irq:
    pr_info("[ist771x] %s: err_request_irq failed\n", __func__);
    free_irq(client->irq, ts);
err_input_register_device_failed:
    pr_info("[ist771x] %s: err_input_register_device failed\n", __func__);
    input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
    pr_info("[ist771x] %s: err_input_dev_alloc failed\n", __func__);
    #ifdef	HIDEEP_TP_VENDOR
free_tp_vendor_gpio:
	if (gpio_is_valid(platform_data->gpio_tp_vendor))
		gpio_free(platform_data->gpio_tp_vendor);
		#endif
free_irq_gpio:
	if (gpio_is_valid(platform_data->gpio_int))
		gpio_free(platform_data->gpio_int);
free_reset_gpio:
    pr_info("[ist771x] %s: free_reset_gpio\n", __func__);
	if (gpio_is_valid(platform_data->gpio_reset))
		gpio_free(platform_data->gpio_reset);
	if (ts->ts_pinctrl) {
		if (IS_ERR_OR_NULL(ts->pinctrl_state_release)) {
			devm_pinctrl_put(ts->ts_pinctrl);
			ts->ts_pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(ts->ts_pinctrl,
					ts->pinctrl_state_release);
			if (ret)
				pr_err("failed to select relase pinctrl state\n");
		}
	}
    hideep_power_on(ts, false);
err_power_on_failed:
    pr_info("[ist771x] %s: failed to power on\n", __func__);
    hideep_power_init(ts, false);
#ifdef I2C_DMA
	dbg_fun();
    dma_free_coherent(NULL, I2C_DMA_BUF_SIZE, ts->dma_va, ts->dma_pa);
	dbg_fun();
#endif
err_power_init_failed:
    pr_info("[ist771x] %s: failed to power init\n", __func__);
    mutex_destroy(&ts->io_lock);
#ifdef	I2C_MUTEX
    mutex_destroy(&ts->i2c_mutex);
#endif
#ifdef I2C_DMA
err_input_dev_mem:
#endif
err_ts_parse_dt_failed:
#ifdef CONFIG_OF
    pr_info("[ist771x] %s: failed to ts parse dt\n", __func__);
#endif
err_mem_alloc_failed:
    pr_info("[ist771x] %s: failed to allocate memory\n", __func__);
err_alloc_data_failed:
    pr_info("[ist771x] %s: err_after_get_regulator failed_\n", __func__);

    kfree(ts);

err_check_functionality_failed:
    pr_info("[ist771x] %s: err_check_functionality failed_\n", __func__);

    return ret;
}

static int hideep_remove(struct i2c_client *client)
{
    struct hideep_data *ts = i2c_get_clientdata(client);

//    unregister_early_suspend(&ts->early_suspend);
    mutex_destroy(&ts->io_lock);
#ifdef	I2C_MUTEX
    mutex_destroy(&ts->i2c_mutex);
#endif
    free_irq(client->irq, ts);
    ts->power(0);
#ifdef I2C_DMA
		dbg_fun();
    dma_free_coherent(NULL, I2C_DMA_BUF_SIZE, ts->dma_va, ts->dma_pa);
		dbg_fun();
#endif
    input_unregister_device(ts->input_dev);
    kfree(ts);
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hideep_early_suspend(struct early_suspend *h)
{
    struct hideep_data *ts;
    
#if DEBUG_PRINT
    pr_info("[ist771x] %s\n", __func__);
#endif

    ts = container_of(h, struct hideep_data, early_suspend);
    hideep_suspend(ts->client, PMSG_SUSPEND);
}

static void hideep_late_resume(struct early_suspend *h)
{
    struct hideep_data *ts;
    
#if DEBUG_PRINT
    pr_info("[ist771x] %s\n", __func__);
#endif

    ts = container_of(h, struct hideep_data, early_suspend);
    hideep_resume(ts->client);
}
#endif

static const struct i2c_device_id hideep_id[] =
{
    { I2C_DRIVER_NAME, 0 },
    { }
};

#ifdef CONFIG_OF
static struct of_device_id hideep_match_table[] = {
	{ .compatible = "hideep,ts",},
	{ },
};
#else
#define hideep_match_table NULL
#endif

static struct i2c_driver hideep_driver =
{
    .driver =
    {
        .name = I2C_DRIVER_NAME,
        .owner = THIS_MODULE,
		.of_match_table = hideep_match_table,
    }, 
    .id_table = hideep_id, 
    .probe = hideep_probe, 
    .remove = hideep_remove,
    //.remove = __devexit_p(hideep_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = hideep_suspend, 
    .resume = hideep_resume,
#endif
};

/*
static struct kobject *knock_on_kobj;
struct attribute {
    char            *name;
    struct module   *owner;
    mode_t          mode;
};
*/

static int __init hideep_init(void)
{
/*
#ifdef _USE_SYSFS
    knock_on_kobj = kobject_create_and_add("kobject_knock_on", kernel_kobj);
    if (!example_kobj)
        return -ENOMEM;

    retval = sysfs_create_file( knock_on_kobj, &attr_knockon);
#endif
*/
    int ret = 0;

    //ret = hideep_i2c_bus_init();
	printk ("[ist771x] %d\n", ret);
    if(ret<0)
        return ret;
    return i2c_add_driver(&hideep_driver);
}

static void __exit hideep_exit(void)
{
    i2c_del_driver(&hideep_driver);
    //hideep_i2c_bus_exit();
}

MODULE_DESCRIPTION("Driver for HiDeep Touchscreen Controller");
MODULE_AUTHOR("HiDeep Inc.");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

module_init(hideep_init);
module_exit(hideep_exit);

