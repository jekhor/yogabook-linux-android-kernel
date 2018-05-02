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

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#define HIDEEP_TS_NAME "hideep-ist771x"
#define PLATFORM_DRIVER_NAME HIDEEP_TS_NAME
#define I2C_DRIVER_NAME "hideep-ist771x-i2c"
//#define	HIDEEP_TP_VENDOR
#define HIDEEP_FW      "ist771_auto.bin"
#define HIDEEP_MT_FW      "ist771_mutto_auto.bin"
#define HIDEEP_MANUAL_FW      "ist771.bin"

//#define EIGHT_PIN_SOLUTION

/* Intel platform gpio base */
#define gpio_southwest_NUM	98
#define gpio_north_NUM	73
#define gpio_east_NUM	27
#define	gpio_southeast_NUM	86

#define	gpio_southwest_base	(ARCH_NR_GPIOS-gpio_southwest_NUM)
#define gpio_north_base		(gpio_southwest_base - gpio_north_NUM)
#define	gpio_east_base		(gpio_north_base - gpio_east_NUM)
#define gpio_southeast_base		(gpio_east_base - gpio_southeast_NUM)

#ifdef EIGHT_PIN_SOLUTION
#define	GPIO_ALERT	77
#define FST_SPI_CS2_B  7

#define HIDEEP_INT_GPIO (gpio_southeast_base + GPIO_ALERT)
#define HIDEEP_RESET_GPIO (FST_SPI_CS2_B + gpio_southwest_base)

#else

#define	GPIO_CAMERASB04	56
#define GPIO_CAMERASB01  53
#define I2C4_SCL		50
#define I2C4_SDA		46

#define HIDEEP_INT_GPIO (gpio_north_base + GPIO_CAMERASB04) //GPIO_CAMERASB04
#define HIDEEP_RESET_GPIO (gpio_north_base + GPIO_CAMERASB01) //GPIO_CAMERASB01
#define HIDEEP_I2C_SCL_GPIO (gpio_southwest_base + I2C4_SCL)
#define HIDEEP_I2C_SDA_GPIO (gpio_southwest_base + I2C4_SDA)

#define HIDEEP_I2C_SCL_GPIO_MODE	0
#define HIDEEP_I2C_SCL_I2C_MODE		1
#define HIDEEP_I2C_SDA_GPIO_MODE	0
#define HIDEEP_I2C_SDA_I2C_MODE		1
#endif
#define HIDEEP_DRIVER_VERSION 1
#define HIDEEP_POWER_GPIO -1


#define HIDEEP_IF_DEVICE

#define TX_NUM 28
#define RX_NUM 42
#define hideep_dd_version_major 1
#define hideep_dd_version_minor 1
#define hideep_dd		"hideep_tsp"
#define TOUCH_MAX_COUNT             10 //Model Dependent
//#define I2C_DMA
#define HIDEEP_SELF_TEST
//#define I2C_MUTEX

#define HIDEEP_DEBUG
#ifdef HIDEEP_DEBUG
#define hideep_tag               "[ist771x] "
#define dbg_fun(fmt, args...)     printk(hideep_tag"%s @ %d "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define dbg_err(fmt, args...)     printk(hideep_tag"error %s @ %d : "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define dbg_log(fmt, args...)     printk(hideep_tag"%s @ %d : "fmt"\n", __FUNCTION__, __LINE__, ##args)
#else
#define hideep_tag               "[ist771x] "
#define dbg_fun(fmt, args...)     /*printk(hideep_tag"%s @ %d "fmt"\n", __FUNCTION__, __LINE__, ##args)*/
#define dbg_err(fmt, args...)     printk(hideep_tag"error %s @ %d : "fmt"\n", __FUNCTION__, __LINE__, ##args)
#define dbg_log(fmt, args...)
#endif

#define HIDEEP_SWD_CLK_SET(a) private_ts->pdata->swd_clk(a)
#define HIDEEP_SWD_DAT_SET(a) private_ts->pdata->swd_dat(a)
#define HIDEEP_SWD_DAT_GET() private_ts->pdata->swd_dat_read()
#define hideep_gpio_i2c_SCL(a) private_ts->pdata->i2c_scl(a)
#define hideep_gpio_i2c_SDA(a) private_ts->pdata->i2c_sda(a)
#define hideep_gpio_i2c_SDA_GET() private_ts->pdata->i2c_sda_read()

enum
{
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

struct finger_info
{
	int x;
	int y;	
	int z;
	int w;
    int flag;
};

struct hideep_platform_data {
	int version;
	int gpio_int;
    u32 irq_flags;
	int gpio_reset;
    int  power_ldo_gpio;
    u32 reset_flags;
    int gpio_scl;
    int gpio_sda;
	u32 gpio_scl_value0;
	u32 gpio_scl_value1;
	u32 gpio_sda_value0;
	u32 gpio_sda_value1;
	int gpio_power;
	#ifdef	HIDEEP_TP_VENDOR
  int gpio_tp_vendor;
  #endif
	int (*gpio_config)(int gpio, bool configure, int dir, int state);
	int max_x;
	int max_y;
	int max_z;
	int max_w;
	int (*gpio_init)(void *handle, void *pdata);
	void (*gpio_uninit)(void *pdata);
    void (*int_enable)(void *pdata, unsigned char enable);
	void (*power)(void *pdata, unsigned char enable);
	void (*reset)(void);
	int (*swd_init)(void);
	void (*swd_uninit)(void);
	void (*swd_dat)(unsigned char send);
	unsigned char (*swd_dat_read)(void);
	void (*swd_clk)(unsigned char level);
	void (*i2c_sda)(unsigned char send);
	unsigned char (*i2c_sda_read)(void);
	void (*i2c_scl)(unsigned char level);
	#ifdef	HIDEEP_TP_VENDOR
	int (*tp_vendor)(void);
	#endif
};

struct hideep_touch_evt
{
    uint16_t        x;
    uint16_t        y;
    uint16_t         z;
    uint8_t         flag;
    uint8_t         index;
    uint8_t         w;
};
struct hideep_input_evt
{
    int32_t         x;
    int32_t         y;
    int32_t         z;
    int32_t         w;
    int32_t	    flag;
};

#define finger_info hideep_input_evt

typedef struct hideep_debug_dev
{
    u8                         *data;
    struct cdev                acdev;
    wait_queue_head_t          i_packet;           // read raw
    u32                        i_rdy;
    u8                         *vr_buff;
    u8                         *im_buff;
    u16                        im_size;
    u16                        vr_size;
    u8                         im_r_en;
    struct hideep_data         *ts    ;
} hideep_debug_dev_t;
typedef struct hideep_debug_cfg
{
    u16                        im_size;
    u16                        vr_size;
} hideep_debug_cfg_t;
struct hideep_data
{
	u16   addr;
    unsigned int auto_update_check;
	struct 	i2c_client *client; 
	struct 	input_dev *input_dev;
	struct 	hideep_platform_data *pdata;
	struct 	work_struct work;
    int	updating;
    int	update_flag;
    int	wait_int;
    int	waiting_suspend;
	u32   flags;
    struct  workqueue_struct *hideep_wq;
    struct  workqueue_struct *hideep_fw_wq;
    struct   delayed_work fw_work;
	int 				irq;
	int 	(*power)(int on);
    struct   mutex io_lock;
#ifdef	I2C_MUTEX
    struct   mutex i2c_mutex;
#endif
	struct completion 		init_done;
    uint32_t                         input_nr;
    struct hideep_touch_evt        input[TOUCH_MAX_COUNT];
    struct hideep_input_evt        touch[TOUCH_MAX_COUNT];
    #if defined(CONFIG_FB)
	struct notifier_block fb_notif;
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct 	early_suspend early_suspend;	
    #endif
    struct regulator *vdd;
	struct regulator *vdd2;
	struct regulator *vcc_i2c;
    struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
    struct pinctrl_state *pinctrl_state_gpio;
	struct pinctrl_state *pinctrl_state_iic;
    unsigned int suspend;
    unsigned int resume;
    
#ifdef HIDEEP_IF_DEVICE
    u32                         debug_dev_no;
    struct hideep_debug_dev        debug_dev  ;
    struct class                   *debug_class;
#endif
    u8                         seg_buff[256];
#ifdef I2C_DMA
    void  *dma_va;
    dma_addr_t dma_pa;
#endif
#ifdef HIDEEP_SELF_TEST
	unsigned char TXshortResult[TX_NUM];
	unsigned char RXshortResult[RX_NUM];
	unsigned char TXopenResult[TX_NUM];
	unsigned char RXopenResult[RX_NUM];
#endif 
   int lines;
};

extern void lnw_gpio_set_alt(int gpio, int alt);
extern void lnw_gpio_lock(int gpio,int on);
void lnw_gpio_read_reg(int gpio, u32 *value0, u32* value1);
void lnw_gpio_write_reg(int gpio, u32 *value0, u32* value1);


#endif /* _LINUX_HIDEEP_TS_H */
