
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/power/intel_pmic_ccsm.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#define CHR_DBG
#ifdef CHR_DBG
#define chr_dbg(format, args...)	printk(KERN_ERR "%s:"format, \
		__FUNCTION__, ##args)
#else
#define chr_dbg(fmt, args...)
#endif

#define CHRLEDCTRL_REG 0x5E1F
#define CHRLEDFSM_REG  0x5E20
#define CHRLEDPWM_REG  0x5E21

/* CHRLEDCTRL_REG */
#define CHRLEDFN_SW		(1<<0)
#define CHRLEDFN_HW		(0<<0)

#define SWLEDON_ON		(1<<1)
#define SWLEDON_OFF		(0<<1)         //disable CHRLED circuit
#define SWLEDON_MASK	 0x02

#define CHRLEDI_2_5MA	(2<<2)

#define CHRLEDF_1_4_HZ	(0<<4)
#define CHRLEDF_1_2_HZ	(1<<4)
#define CHRLEDF_1_HZ	(2<<4)
#define CHRLEDF_2_HZ	(3<<4)
#define CHRLEDF_MASK	 0x30

/* CHRLEDFSM_REG */
#define CHRLEDCIP_UNSET		(0<<0)

#define LEDEFF_ON			(1<<1)     //always on
#define LEDEFF_BLINKING		(2<<1)
#define LEDEFF_BREATHING	(3<<1)
#define LEDEFF_MASK     	 0x06

void chrled_set_enable(int on)
{
	u8 temp = 0;

	temp = intel_soc_pmic_readb(CHRLEDCTRL_REG);

	temp &= ~SWLEDON_MASK;
	if (on) {
		temp |= (SWLEDON_ON & SWLEDON_MASK);
	} else {
		temp |= (SWLEDON_OFF & SWLEDON_MASK);
	}
	intel_soc_pmic_writeb(CHRLEDCTRL_REG, temp);
}
EXPORT_SYMBOL(chrled_set_enable);

void ChrLedSW_set_enable(int on)
{
	u8 temp = 0;

	temp = intel_soc_pmic_readb(CHRLEDCTRL_REG);

	temp &= ~SWLEDON_MASK;
	if (on) {
		temp |= (SWLEDON_ON |CHRLEDFN_SW);
	} else {
		temp |= (SWLEDON_OFF | CHRLEDFN_SW);
	}
	intel_soc_pmic_writeb(CHRLEDCTRL_REG, temp);
}
EXPORT_SYMBOL(ChrLedSW_set_enable);

u8 chrled_get_enable(void)
{
	u8 temp;
	temp = intel_soc_pmic_readb(CHRLEDCTRL_REG);
	temp &= SWLEDON_MASK;
	return temp;
}

/* freq: CHRLEDF_1_4_HZ/CHRLEDF_1_2_HZ/CHRLEDF_1_HZ/CHRLEDF_2_HZ */
void chrled_set_freq(int freq)
{
	u8 temp;
	printk(KERN_ERR "%s,freq:%d",__func__,freq);
	temp = intel_soc_pmic_readb(CHRLEDCTRL_REG);
	temp &= ~CHRLEDF_MASK;
	temp |= freq & CHRLEDF_MASK;
	intel_soc_pmic_writeb(CHRLEDCTRL_REG, temp);
}
EXPORT_SYMBOL(chrled_set_freq);

int chrled_get_freq(void)
{
	u8 temp;
	temp = intel_soc_pmic_readb(CHRLEDCTRL_REG);
	temp &= CHRLEDF_MASK;
	return temp;
}

/* effect: LEDEFF_ON/LEDEFF_BLINKING/LEDEFF_BREATHING */
void chrled_set_effect(int effect)
{
	intel_soc_pmic_writeb(CHRLEDFSM_REG, effect);
}
EXPORT_SYMBOL(chrled_set_effect);

int chrled_get_effect(void)
{
	u8 temp;
	temp = intel_soc_pmic_readb(CHRLEDFSM_REG);
	chr_dbg("chrled_get_effect effect = 0x%x\n", temp);
	temp &= LEDEFF_MASK;
	chr_dbg("chrled_get_effect effect = 0x%x\n", temp);
	return temp;
}

static ssize_t chr_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	u8 enable = chrled_get_enable();
	return sprintf(buf, "enable = %u\n", (SWLEDON_ON == enable)? 1: 0);
}

static ssize_t chr_freq_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	u8 freq = chrled_get_freq();
	switch (freq) {
	case CHRLEDF_1_4_HZ:
		return sprintf(buf, "freq = 1/4HZ\n");
	case CHRLEDF_1_2_HZ:
		return sprintf(buf, "freq = 1/2HZ\n");
	case CHRLEDF_1_HZ:
		return sprintf(buf, "freq = 1HZ\n");
	case CHRLEDF_2_HZ:
		return sprintf(buf, "freq = 2HZ\n");
	default:
		return sprintf(buf, "freq null!\n");
	}
}

static ssize_t chr_effect_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	u8 effect = chrled_get_effect();
	chr_dbg("chr_effect_show effect = %d\n", effect);
	switch (effect) {
	case LEDEFF_ON:
		return sprintf(buf, "effect = on\n");
	case LEDEFF_BLINKING:
		return sprintf(buf, "effect = blinking\n");
	case LEDEFF_BREATHING:
		return sprintf(buf, "effect = breathing\n");
	default:
		return sprintf(buf, "effect unknown!\n");
	}
}

static ssize_t chr_enable_store(struct device *dev, struct device_attribute *data,
			    const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	chr_dbg("chr_enable_store val = %d\n", val);

	if (val) {
		chrled_set_enable(1);
	} else {
		chrled_set_enable(0);
	}

	return count;
}

static ssize_t chr_freq_store(struct device *dev, struct device_attribute *data,
			    const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val==0) {
		chrled_set_freq(CHRLEDF_1_4_HZ);
	} else if (val==1) {
		chrled_set_freq(CHRLEDF_1_2_HZ);
	} else if (val==2) {
		chrled_set_freq(CHRLEDF_1_HZ);
	} else if (val==3) {
		chrled_set_freq(CHRLEDF_2_HZ);
	}

	return count;
}

static ssize_t chr_effect_store(struct device *dev, struct device_attribute *data,
			    const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val==0) {
		chrled_set_effect(LEDEFF_ON);
	} else if (val==1) {
		chrled_set_effect(LEDEFF_BLINKING);
	} else if (val==2) {
		chrled_set_effect(LEDEFF_BREATHING);
	}

	return count;
}

static DEVICE_ATTR(chr_enable, 0660, chr_enable_show, chr_enable_store);
static DEVICE_ATTR(chr_freq, 0660, chr_freq_show, chr_freq_store);
static DEVICE_ATTR(chr_effect, 0660, chr_effect_show, chr_effect_store);

static struct attribute *chr_led_attributes[] = {
	&dev_attr_chr_enable.attr,
	&dev_attr_chr_freq.attr,
	&dev_attr_chr_effect.attr,
	NULL,
};

static struct attribute_group chr_led_attribute_group = {
	.attrs = chr_led_attributes,
};

#define GPLEDCTRL_REG	0x4FDF
#define GPLEDFSM_REG	0x4FE0
#define GPLEDPWM_REG	0x4FE1

/* GPLEDCTRL_REG */
#define GPLEDON_ON		(1<<0)
#define GPLEDON_OFF		(0<<0)
#define GPLEDON_MASK	(1<<0)

#define GPLEDI_MA2_5	(2<<2)

#define GPLEDF_1_4_HZ	(0<<4)
#define GPLEDF_1_2_HZ	(1<<4)
#define GPLEDF_1_HZ		(2<<4)
#define GPLEDF_2_HZ		(3<<4)
#define GPLEDF_MASK		 0x30

/* GPLEDFSM_REG */
#define GPLEDFF_ON			(1<<1)     //always on
#define GPLEDFF_BLINKING	(2<<1)
#define GPLEDFF_BREATHING	(3<<1)
#define GPLEDFF_MASK     	 0x06

void gpled_set_enable(int on)
{
	u8 temp;
	temp = intel_soc_pmic_readb(GPLEDCTRL_REG);
	temp &= ~GPLEDON_MASK;
	if (on) {
		temp |= (GPLEDON_ON & GPLEDON_MASK);
	} else {
		temp |= (GPLEDON_OFF & GPLEDON_MASK);
	}
	intel_soc_pmic_writeb(GPLEDCTRL_REG, temp);
}
EXPORT_SYMBOL(gpled_set_enable);

u8 gpled_get_enable(void)
{
	u8 temp;
	temp = intel_soc_pmic_readb(GPLEDCTRL_REG);
	temp &= GPLEDON_MASK;
	return temp;
}

static ssize_t gp_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	u8 enable = gpled_get_enable();
	return sprintf(buf, "enable = %u\n", (GPLEDON_ON == enable)? 1: 0);
}

static ssize_t gp_enable_store(struct device *dev, struct device_attribute *data,
			    const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	chr_dbg("gp_enable_store val = %d\n", val);

	if (val) {
		gpled_set_enable(1);
	} else {
		gpled_set_enable(0);
	}

	return count;
}

/* GPLEDF_1_4_HZ/GPLEDF_1_2_HZ/GPLEDF_1_HZ/GPLEDF_2_HZ */
void gpled_set_freq(int freq)
{
	u8 temp;
	temp = intel_soc_pmic_readb(GPLEDCTRL_REG);
	temp &= ~GPLEDF_MASK;
	temp |= freq & GPLEDF_MASK;
	intel_soc_pmic_writeb(GPLEDCTRL_REG, temp);
}
EXPORT_SYMBOL(gpled_set_freq);

int gpled_get_freq(void)
{
	u8 temp;
	temp = intel_soc_pmic_readb(GPLEDCTRL_REG);
	temp &= GPLEDF_MASK;
	return temp;
}

static ssize_t gp_freq_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	u8 freq = gpled_get_freq();
	switch (freq) {
	case GPLEDF_1_4_HZ:
		return sprintf(buf, "gp freq = 1/4HZ\n");
	case GPLEDF_1_2_HZ:
		return sprintf(buf, "gp freq = 1/2HZ\n");
	case GPLEDF_1_HZ:
		return sprintf(buf, "gp freq = 1HZ\n");
	case GPLEDF_2_HZ:
		return sprintf(buf, "gp freq = 2HZ\n");
	default:
		return sprintf(buf, "gp freq null!\n");
	}
}

static ssize_t gp_freq_store(struct device *dev, struct device_attribute *data,
			    const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val==0) {
		gpled_set_freq(CHRLEDF_1_4_HZ);
	} else if (val==1) {
		gpled_set_freq(CHRLEDF_1_2_HZ);
	} else if (val==2) {
		gpled_set_freq(CHRLEDF_1_HZ);
	} else if (val==3) {
		gpled_set_freq(CHRLEDF_2_HZ);
	}

	return count;
}

/* effect: GPLEDFF_ON/GPLEDFF_BLINKING/GPLEDFF_BREATHING */
void gpled_set_effect(int effect)
{
	intel_soc_pmic_writeb(GPLEDFSM_REG, effect);
}
EXPORT_SYMBOL(gpled_set_effect);

int gpled_get_effect(void)
{
	u8 temp;
	temp = intel_soc_pmic_readb(GPLEDFSM_REG);
	temp &= GPLEDFF_MASK;
	return temp;
}

static ssize_t gp_effect_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	u8 effect = gpled_get_effect();
	chr_dbg("gp_effect_show effect = %d\n", effect);
	switch (effect) {
	case GPLEDFF_ON:
		return sprintf(buf, "gp effect = on\n");
	case GPLEDFF_BLINKING:
		return sprintf(buf, "gp effect = blinking\n");
	case GPLEDFF_BREATHING:
		return sprintf(buf, "gp effect = breathing\n");
	default:
		return sprintf(buf, "gp effect unknown!\n");
	}
}

static ssize_t gp_effect_store(struct device *dev, struct device_attribute *data,
			    const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val==0) {
		gpled_set_effect(GPLEDFF_ON);
	} else if (val==1) {
		gpled_set_effect(GPLEDFF_BLINKING);
	} else if (val==2) {
		gpled_set_effect(GPLEDFF_BREATHING);
	}

	return count;
}

static DEVICE_ATTR(gp_enable, 0660, gp_enable_show, gp_enable_store);
static DEVICE_ATTR(gp_freq, 0660, gp_freq_show, gp_freq_store);
static DEVICE_ATTR(gp_effect, 0660, gp_effect_show, gp_effect_store);

static struct attribute *gp_led_attributes[] = {
	&dev_attr_gp_enable.attr,
	&dev_attr_gp_freq.attr,
	&dev_attr_gp_effect.attr,
	NULL,
};

static struct attribute_group gp_led_attribute_group = {
	.attrs = gp_led_attributes,
};

static int charger_gp_led_probe(struct platform_device *pdev)
{
	int ret;
	chr_dbg("charger_gp_led_probe...... wanghow\n");
	ret = sysfs_create_group(&pdev->dev.kobj, &chr_led_attribute_group);
	ret = sysfs_create_group(&pdev->dev.kobj, &gp_led_attribute_group);
	intel_soc_pmic_writeb(CHRLEDCTRL_REG, 0x19);//CHRLEDF_1_2_HZ|CHRLEDFN_SW|CHRLEDI_2_5MA|SWLEDON_OFF);
	intel_soc_pmic_writeb(CHRLEDPWM_REG,0xFF); 
	gpled_set_effect(GPLEDFF_ON);
	gpled_set_enable(0);
	return 0;
}

static int charger_gp_led_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &chr_led_attribute_group);
	sysfs_remove_group(&pdev->dev.kobj, &gp_led_attribute_group);
	return 0;
}

static void charger_gp_led_shutdown(struct platform_device *pdev)
{
	chr_dbg("charger_gp_led_shutdown...... wanghow\n");

	return 0;
}

static struct platform_device charger_gp_led_pdev = {
	.name           = "charger_gp_led",
	.id             = 0,
};

static struct platform_driver charger_gp_led_driver = {
	.probe		= charger_gp_led_probe,
	.remove		= charger_gp_led_remove,
	.shutdown	= charger_gp_led_shutdown,
	.driver = {
		.name   = "charger_gp_led",
		.owner  = THIS_MODULE,
	}
};

static int __init charger_led_init(void)
{
	platform_device_register(&charger_gp_led_pdev);
	return platform_driver_register(&charger_gp_led_driver);
}

static void __exit charger_led_exit(void)
{
	return;
}

MODULE_AUTHOR("how.wang@intel.com");
MODULE_DESCRIPTION("charger led and pmic GPled driver");
MODULE_LICENSE("GPL");

//fs_initcall(charger_led_init);
late_initcall(charger_led_init);
module_exit(charger_led_exit);

