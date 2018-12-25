

#ifndef __HIDEEP__
#define __HIDEEP__

#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>

#include <linux/module.h>

#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
//#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h> // slot
#include <linux/input/hideep_ts.h> // slot
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/gfp.h>
//#include "tpd.h"
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
//#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <uapi/linux/i2c.h>

#include "hideep_fw.h"

#define DD_VERSION_MAJOR  0
#define DD_VERSION_MIDDLE 1
#define DD_VERSION_MINOR  1

#define TOUCH_COUNT_ADDR            0x0240
#define TOUCH_READ_START_ADDR       0x0242 //Fixed value
#define TOUCH_FLAG_MASK             0x10
#define TOUCH_DEEP_SLEEP_MASK       0x10
#define MAX_TRANSACTION_LENGTH      16
#define MAX_I2C_DMA_LENGTH          252
#define I2C_DMA_BUF_SIZE						(4*1024)


#define VR_ADDR_IMAGE               0x1000
#define VR_ADDR_BASE                0x0000
#define DWZ_ADDR                    0x0400
#define __HIDEEP_MAJOR_VERSION__ 1
#define __HIDEEP_MINOR_VERSION__ 0
#define __HIDEEP_UPDATE_BOOTLOADER__ 1
#define __HIDEEP_VERIFY_METHOD__ 1
#define __HIDEEP_FLASH_VERIFY_TOGETHER__  0

#define TOUCH_READ_SW_VER_ADDR     0x0132 //Model Dependent

#define TOUCH_READ_BL_VER_ADDR     0X8014
#define TOUCH_READ_CODE_VER_ADDR     0X8016
#define TOUCH_READ_CUST_VER_ADDR     0X8018
#define TOUCH_READ_VR_VER_ADDR     0X801A
#define TOUCH_READ_FACTORY_ID_ADDR	0X8022



extern struct hideep_data *private_ts;



#ifdef HIDEEP_IF_DEVICE
int hideep_iface_init    (struct hideep_data *ts);
void hideep_iface_uninit (struct hideep_data *ts);
#endif

int hideep_sysfs_init   (struct hideep_data *ts);
int hideep_sysfs_exit   (struct hideep_data *ts);
void hideep_reset(void);
void hideep_update_fw_thread(struct work_struct *work);
int check_version(struct i2c_client *client, u8 *val);
int hideep_update(struct device *dev, const char *buf);
int hideep_i2c_read(struct i2c_client *client, uint16_t addr, uint16_t len, uint8_t *buf);
int hideep_i2c_write(struct i2c_client *client, uint16_t addr, uint16_t len, uint8_t *buf);
#ifdef HIDEEP_SELF_TEST
void hideep_test_mode(struct hideep_data *ts);
#endif
#ifdef	HIDEEP_TP_VENDOR
int hideep_tp_vendor(void);
#endif

#endif //__HIDEEP__
