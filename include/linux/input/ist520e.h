/*
 * include/linux/HiDeep_ts.h - platform data structure for iST Series sensor
 *
 * Copyright (C) 2012 Hideep, Inc.
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

#ifndef _LINUX_HIDEEP_TS_H
#define _LINUX_HIDEEP_TS_H

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h> // slot
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/gfp.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/power_supply.h>

#ifdef CONFIG_PM_SLEEP
#include <linux/power_hal_sysfs.h>
#endif


#define ISTCORE_IF_DEVICE
#define IST510E_KEYBUTTON
#define PROTOCOL_HIDEEP_20
//#define IST510E_I2C_DEV_REGISTER
//#define HIDEEP_REPEAT_START
/*------------------------------------------------------------------------------
 * board porting :
 *-----------------------------------------------------------------------------*/
#define ISTCORE_DEV_NAME            "IST940E : "
#define ISTCORE_I2C_NAME            "ist940e"
#define ISTCORE_I2C_ADDR            0x6C
#define ISTCORE_I2C_BUS             5
#define HIDEEP_TS_NAME              "Hideep"
#define ISTCORE_IF_DEVICE_NAME      "ist940e_debug"
#define MAX_TRANSACTION_LENGTH      8
#define MAX_I2C_DMA_LENGTH          252
#define MAX_I2C_SPEED               400
//#define MAX_VR_BUFF                 0x80

/* Intel platform gpio base */
#define gpio_southwest_NUM	98
#define gpio_north_NUM	73
#define gpio_east_NUM	27
#define	gpio_southeast_NUM	86

#define	gpio_southwest_base	(ARCH_NR_GPIOS-gpio_southwest_NUM)
#define gpio_north_base		(gpio_southwest_base - gpio_north_NUM)
#define	gpio_east_base		(gpio_north_base - gpio_east_NUM)
#define gpio_southeast_base		(gpio_east_base - gpio_southeast_NUM)

#define	GPIO_ALERT	77
#define FST_SPI_CS2_B  7

#define HIDEEP_ATTN_GPIO (gpio_southeast_base + GPIO_ALERT)
#define HIDEEP_RESET_GPIO (FST_SPI_CS2_B + gpio_southwest_base)

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define ISTCORE_PWR_NORMAL         1
#define ISTCORE_PWR_SLEEP          2
#define ISTCORE_PWR_PGM            3

/*------------------------------------------------------------------------------
 * touch
 *-----------------------------------------------------------------------------*/
#define TOUCH_MAX_COUNT             10
#define KEYS_MAX_COUNT              3
#define TOUCH_COUNT_ADDR            0x0240
#define TOUCH_READ_START_ADDR       0x0242
#define KEY_READ_START_ADDR         (0x0242 + (sizeof(struct ist510e_touch_evt)) * TOUCH_MAX_COUNT)
#define VR_ADDR_IMAGE               0x1000
#define VR_ADDR_BASE                0x0000
#define DWZ_ADDR                    0x02C0
#define BOOT_OFFS                   0x0400

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define IST_TOUCH
//#define IST_KEY

#define EV_ALWAYS_REPORT            0   //0x01
#define EV_TOUCHED                  1   //0x02
#define EV_FIRST_CONTACT            2   //0x04
#define EV_DRAG_MOVE                3   //0x08
#define EV_RELEASED                 4   //0x10
#define EV_PINCH                    5   //0x20
#define EV_PRESSURE                 6   //0x40

#define KY_RELEASED                 0x20
#define KY_PRESSED                  0x40
#define KY_FIRST_PRESSED            0x80
#define KY_PRESSED_MASK             0xC0

/*------------------------------------------------------------------------------
 * VR.OPMode
 *-----------------------------------------------------------------------------*/
#define OPM_RAW_FLAG            (0x80)
#define OPM_TOUCH_A             (0x00)
#define OPM_MOD                 (0x80)
#define OPM_MOD_CAP             (0x81)
#define OPM_MOD_PRS             (0x82)
#define OPM_FRAME_PRS           (0x84)
#define OPM_DEMOD               (0x85)
#define OPM_DEMOD_CAP           (0x86)
#define OPM_DEMOD_PRS           (0x87)
#define OPM_FRAME               (0x88)
#define OPM_FRAME_CAP           (0x89)
#define OPM_DIFF                (0x8A)
#define OPM_DIFF_CAP            (0x8B)
#define OPM_DIFF_PRS            (0x8C)
#define OPM_BASELINE_CAP        (0x8D)
#define OPM_BASELINE_PRS        (0x8E)
#define OPM_SCDIFF              (0x8F)
#define OPM_STATUS              (0x90)
#define OPM_LASTDIFF            (0x91)
#define OPM_PARM0               (0x92)
#define OPM_PARM1               (0x93)
#define OPM_PARM2               (0x94)
#define OPM_PARM3               (0x95)

#define START_FIRMWARE_UPDATE	(0)
#define MANUAL_FIRMWARE_UPDATE	(1)

/*------------------------------------------------------------------------*//**
 * command list
 *//*--------------------------------------------------------------------------*/
#define IST_EMPTY               (0x0000)
#define IST_ENTER_RIP           (0x080F)                                    // enter sleep mode
#define IST_ENTER_ZOMBIE        (0x0815)                                    // enter sleep mode
#define IST_RESET               (0x0803)                                    // software reset
#define IST_TEST_MODE           (0x0804)                                    //
#define IST_WORK_MODE 		(0x081E)
#define IST_CMD_CHARGE          (0x0811)                                    //charge detection 


#define DO_STARTUP_FW_UPDATE
#define FWUPDATE_WORK_DELAY_MS 10000
#define IST940E_FW      "hideep/YetiM_B_OFILM_MP_0100F121_20160623.bin"
#define GIS_IST940E_FW      "hideep/YetiM_B_GIS_MP_0600F121_20160623.bin"
#define KEY_LPM_WAKEUP	KEY_WAKEUP
#define HIDEEP_LPM
#define HIDEEP_DD_VERSION_MAJOR	0
#define HIDEEP_DD_VERSION_MINOR 1
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#ifdef  PROTOCOL_HIDEEP_20
#define TX_NUM 35
#define RX_NUM 58
struct ist510e_touch_evt
{
     u16                       x;
     u16                       y;
     u16                       z;
     u8                        w;
     u8                        flag;
     u8                        type;                       // not included version 1.0
     u8                        index;
};

#else

struct ist510e_touch_evt
{
     u8                         flag;
     u8                         index;
     u16                        x;
     u16                        y;
     u8                         z;
     u8                         w;
};

#endif

struct ist510e_touch_key
{
    u8                         flag;
    u8                         z;
};

enum
{
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

/*-----------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
typedef struct pannel_info_t
{
    u16 vendor;
    u16 product;
    u16 version;

    u16 dp_w;            // display width
    u16 dp_h;            // display height
    u8  tx  ;
    u8  rx  ;
    u8  tx_stride;
    u8  key_nr;
    u16 key[10];

} PANNEL_INFO_T;

typedef struct dwz_info_t
{
    //-------------------------------------------
    u32 c_begin;         // code start address
    u16 c_crc[6];        // code crc
    
    u32 d_begin;         // custom code
    u16 d_len  ;         // custom code length
    u16 rsv0   ;

    u32 v_begin;         // vreg   code
    u16 v_len  ;         // vreg   code length
    u16 rsv1   ;

    u32 f_begin;         // vreg   code
    u16 f_len  ;         // vreg   code length
    u16 rsv2   ;

    u16 ver_b  ;         // version information
    u16 ver_c  ;
    u16 ver_d  ;
    u16 ver_v  ;

    u8 factory_id   ;
    u8 panel_type   ;
    u8 model_name[6];         // model name
    u16 product_code;         // product code
    u16 extra_option;         // extra option

    u16 ver_ft_major;
    u16 ver_ft_minor;
#if 0
    //-------------------------------------------
    //ext
    PANNEL_INFO_T  pannel;
#endif
} DWZ_INFO_T;
/*------------------------------------------------------------------------------
 * driver information for iST510e
 *-----------------------------------------------------------------------------*/
struct istcore_platform_data
{
    u32                         version;
    u32                         gpio_int;
    u32                         max_x;
    u32                         max_y;
    u32                         max_z;
    u32                         max_w;
    u32                         (*power)(int on);
};

typedef struct ist510e_debug_dev
{
    u8                         *data;
    struct cdev                     cdev;
    wait_queue_head_t               i_packet;
    u32                        i_rdy;
    u8                        *vr_buff;
    u8                        *im_buff;
    u16                        im_size;
    u16                        vr_size;
    u8                         im_r_en;

    struct ist510e                 *ts    ;

} ist510e_debug_dev_t;

typedef struct ist510e_debug_cfg
{
    u16                        im_size;
    u16                        vr_size;
} ist510e_debug_cfg_t;

struct ist510e
{
    DWZ_INFO_T                      dwz_info;
    long                            tch_bit;
    u16                             addr;
    struct  i2c_client              *client;
    struct  input_dev              *input_dev;
    struct  istcore_platform_data   *pdata;
    struct  workqueue_struct        *fwu_workqueue;
    struct  delayed_work            fwu_work;
    struct  work_struct             work;
    u32                             flags;
    bool                            irq_enable;
#ifdef MTK_SLOUTION
    struct task_struct             *thread;
    u8                              *dma_va;
    u32                              dma_pa;
#endif
    bool                            manually_update;
    s32                             irq;
    s32                             dev_state;
    u32                             (*power)(int on);
    struct mutex                    i2c_mutex;
    struct mutex                    dev_mutex;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend            early_suspend;
#endif
#ifdef ISTCORE_IF_DEVICE
    u32                         debug_dev_no;
    struct ist510e_debug_dev        debug_dev  ;
    struct class                   *debug_class;
#endif
    bool                        gesture_enable;
    u32                         input_lpm_nr;
    u32                         input_evt_nr;
    u32                         input_key_nr;
    struct ist510e_touch_evt        input_evt[TOUCH_MAX_COUNT];
    struct ist510e_touch_key        input_key[KEYS_MAX_COUNT ];
    u8                         seg_buff[256];
    u32 vr_addr;
    u32 vr_data;
    u32 vr_size;
    unsigned char TXshortResult[TX_NUM];
    unsigned char RXshortResult[RX_NUM];
    unsigned char TXopenResult[TX_NUM];
    unsigned char RXopenResult[RX_NUM];

#ifdef CONFIG_PM_SLEEP
	bool suspended;
	bool power_hal_want_suspend;
	struct mutex power_hal_lock;
#endif
    struct  workqueue_struct        *notifier_wq;
    struct  work_struct             notifier_work;
   struct notifier_block power_supply_notifier;
   int is_usb_plug_in;
   bool power_status;
   bool anypen_switch;
};
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
extern struct ist510e *ts;
extern unsigned int g_loglevel;
void istxxxx_test_mode(struct ist510e *ts);
int istcore_sysfs_init(struct ist510e *ts);
int istcore_sysfs_exit(struct ist510e *ts);
int ist_load_ucode(struct device *dev, const char *fn, int mode);
int ist_load_dwz(struct ist510e *ts);
int istcore_i2c_read(struct ist510e *ts, u16 addr, u16 len, u8 *buf);
int istcore_i2c_write(struct ist510e *ts, u16 addr, u16 len, u8 *buf);
int istcore_iface_init(struct ist510e *ts);
void istcore_iface_uninit(struct ist510e *ts);
int ist_fuse_ucode(struct i2c_client *client, u8 *code, unsigned int len);
void istcore_reset_ic(void);
void istcore_lcd_power_on(void);
void istcore_lcd_power_off(void);
void hideep_irq_enable(bool en);
int anypen_switch_operate(struct ist510e *ts, bool en);
void istcore_init_mode(struct ist510e *ts);
#ifdef CONFIG_PM_SLEEP
void hideep_power_hal_suspend(struct device *dev);
void hideep_power_hal_resume(struct device *dev);
static int hideep_power_hal_suspend_init(struct device *dev);
static void hideep_power_hal_suspend_destroy(struct device *dev);
static ssize_t hideep_power_hal_suspend_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t hideep_power_hal_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#endif

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define HIDEEP_LOGLEVEL_DEBUG 2
#define HIDEEP_LOGLEVEL_XY 3
#define HIDEEP_LOGLEVEL_I2C 4
/*
#define ISTCORE_INFO(a,arg...) printk("hideep:%s@%d " a, __FUNCTION__, __LINE__, ##arg)
#define ISTCORE_WARN(a,arg...) printk("hideep:%s@%d WARNING " a, __FUNCTION__, __LINE__, ##arg)
#define ISTCORE_ERR(a,arg...) printk("hideep:%s@%d ERR " a, __FUNCTION__, __LINE__, ##arg)
*/

#define DEBUG    0

#if  DEBUG
#define ISTCORE_INFO(a,arg...)     printk(ISTCORE_DEV_NAME a,##arg)
#define ISTCORE_DBG(a,arg...)      printk(ISTCORE_DEV_NAME a,##arg)
#else
#define ISTCORE_INFO(a,arg...)     do {} while (0)
#define ISTCORE_DBG(a,arg...)     do {} while (0)
#endif

#define ISTCORE_WARN(a,arg...) printk(ISTCORE_DEV_NAME a,##arg)
#define ISTCORE_ERR(a,arg...) printk(ISTCORE_DEV_NAME a,##arg)


#define ts_log_err(a,arg...) printk("hideep:%s@%d ERR " a, __FUNCTION__, __LINE__, ##arg)
#define ISTCORE_DBG(a,arg...) \
do{\
	if(g_loglevel>HIDEEP_LOGLEVEL_DEBUG)\
		printk("hideep:%s@%d "a,__FUNCTION__, __LINE__, ##arg);\
}while(0)
#define ISTCORE_XY(a,arg...) \
do{\
	if(g_loglevel>HIDEEP_LOGLEVEL_XY)\
		printk("hideep:%s@%d "a,__FUNCTION__, __LINE__, ##arg);\
}while(0)
#define ISTCORE_I2C(a,arg...) \
do{\
	if(g_loglevel>HIDEEP_LOGLEVEL_I2C)\
		printk("hideep:%s@%d "a,__FUNCTION__, __LINE__, ##arg);\
}while(0)
#define ts_log_info(a,arg...) \
do{\
	if(g_loglevel>1)\
		printk("hideep:%s@%d "a,__FUNCTION__, __LINE__, ##arg);\
}while(0)
#define ts_log_debug(a,arg...) \
do{\
	if(g_loglevel>HIDEEP_LOGLEVEL_DEBUG)\
		printk("hideep:%s@%d "a,__FUNCTION__, __LINE__, ##arg);\
}while(0)
#endif /* _LINUX_HIDEEP_TS_H */
