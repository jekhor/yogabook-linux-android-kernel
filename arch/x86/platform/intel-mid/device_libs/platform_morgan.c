#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <linux/i2c.h>
#include <linux/input/wacom.h>

static struct wacom_platform_data w9002_board_data = {
	.gpio_int = WACOM_MAGTOUCH_INT_GPIO,
	.gpio_reset = WACOM_MAGTOUCH_RESET_GPIO,
};

static int __init wacom_platform_init(void)
{
    int i2c_busnum = 3;
    struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
    void *pdata = NULL;
	printk(KERN_ERR "morgan %s\n",__func__);

    //intel_scu_ipc_msic_vprog3(1);
if (yeti_hw_ver == 0 )
        i2c_busnum = 4;

    memset(&i2c_info, 0, sizeof(i2c_info));
    strncpy(i2c_info.type, MGT_NAME, strlen(MGT_NAME));

    i2c_info.addr = 0x44;
	i2c_info.platform_data = &w9002_board_data;

    printk(KERN_ERR "%s I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",__func__,
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);
#if 1
	adapter = i2c_get_adapter(i2c_busnum);
	if(adapter){
		if(i2c_new_device(adapter,&i2c_info)){
			printk(KERN_ERR "add new i2c device %s , addr 0x%x\n", MGT_NAME,i2c_info.addr);
			return 0;
		}else{
			printk(KERN_ERR "add new i2c device %s , addr 0x%x fail !!!\n", MGT_NAME,i2c_info.addr);
		}
	}else{
		printk(KERN_ERR "[%s]get adapter %d fail\n",__func__,i2c_busnum);
		return -EINVAL;
	}
#else
    return i2c_register_board_info(i2c_busnum, &i2c_info, 1);
#endif
}

device_initcall(wacom_platform_init);

