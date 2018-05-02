#ifndef __LINUX_BQ27X00_BATTERY_H__
#define __LINUX_BQ27X00_BATTERY_H__

/**
 * struct bq27000_plaform_data - Platform data for bq27000 devices
 * @name: Name of the battery. If NULL the driver will fallback to &quot;bq27000&quot;.
 * @read: HDQ read callback.
 *	This function should provide access to the HDQ bus the battery is
 *	connected to.
 *	The first parameter is a pointer to the battery device, the second the
 *	register to be read. The return value should either be the content of
 *	the passed register or an error value.
 */
struct bq27000_platform_data {
	const char *name;
	int (*read)(struct device *dev, unsigned int);
};

/*
 * bq27x00 platform specific data:
 * soc_int_irq (optional)
 * bat_low_irq (optional)
 * board rev specific thermistor translation hook (optional)
 */
struct bq27x00_platform_data {
	int soc_int_irq;
	int bat_low_irq;
	int (*translate_temp)(int temperature);
};
 
#endif
