
#ifndef _LINUX_WACOM_H
#define _LINUX_WACOM_H


#define gpio_southwest_NUM	98
#define gpio_north_NUM	73
#define gpio_east_NUM	27
#define	gpio_southeast_NUM	86

#define	gpio_southwest_base	(ARCH_NR_GPIOS-gpio_southwest_NUM)
#define gpio_north_base		(gpio_southwest_base - gpio_north_NUM)
#define	gpio_east_base		(gpio_north_base - gpio_east_NUM)
#define gpio_southeast_base		(gpio_east_base - gpio_southeast_NUM)

#define	MF_SMB_DATA			82 //GPIO_SW82
#define GPIO_CAMERASB06  	49

#define WACOM_MAGTOUCH_INT_GPIO (gpio_north_base + GPIO_CAMERASB06) //GPIO_CAMERASB04
#define WACOM_MAGTOUCH_RESET_GPIO (gpio_southwest_base + MF_SMB_DATA) 

#define MAGTOUCH_NAME "WAC_I2C_EMR"
#define MGT_NAME 	"MG_I2C_EMR"

#define WACOM_VTG_MIN_UV		3300000
#define WACOM_VTG_MAX_UV		3300000
#define WACOM_I2C_VTG_MIN_UV	1800000
#define WACOM_I2C_VTG_MAX_UV	1800000

struct wacom_platform_data {
	int gpio_int;
	int gpio_reset;
};


#endif 
