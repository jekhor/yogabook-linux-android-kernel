/*
 * TS3A227E Autonomous Audio Accessory Detection and Configuration Switch
 *
 * Copyright (C) 2014 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include <linux/extcon.h>
#include "../intel/board/cht_bl_dpcm_rt5677.h"
#include "../intel/board/cht_bl_pinctrl.h"

enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

struct ts3a227e {
	struct regmap *regmap;
	struct regmap *regmap_physical;
	struct snd_soc_jack *jack;
	struct input_dev *button_dev;
	bool plugged;
	bool mic_present;

	struct extcon_dev edev;
	int id;
	unsigned int int_reg;
	unsigned int kp_int_reg;
	unsigned int acc_reg;
	unsigned int new_jack_plugin;
	struct device *dev;
	struct delayed_work hs_det_work;
	struct delayed_work enable_key_detect_work;
	struct delayed_work long_press_work;
};

/* TS3A227E registers */
#define TS3A227E_REG_DEVICE_ID		0x00
#define TS3A227E_REG_INTERRUPT		0x01
#define TS3A227E_REG_KP_INTERRUPT	0x02
#define TS3A227E_REG_INTERRUPT_DISABLE	0x03
#define TS3A227E_REG_SETTING_1		0x04
#define TS3A227E_REG_SETTING_2		0x05
#define TS3A227E_REG_SETTING_3		0x06
#define TS3A227E_REG_SWITCH_CONTROL_1	0x07
#define TS3A227E_REG_SWITCH_CONTROL_2	0x08
#define TS3A227E_REG_SWITCH_STATUS_1	0x09
#define TS3A227E_REG_SWITCH_STATUS_2	0x0a
#define TS3A227E_REG_ACCESSORY_STATUS	0x0b
#define TS3A227E_REG_ADC_OUTPUT		0x0c
#define TS3A227E_REG_KP_THRESHOLD_1	0x0d
#define TS3A227E_REG_KP_THRESHOLD_2	0x0e
#define TS3A227E_REG_KP_THRESHOLD_3	0x0f

/* TS3A227E_REG_INTERRUPT 0x01 */
#define INS_REM_EVENT 0x01
#define DETECTION_COMPLETE_EVENT 0x02

/* TS3A227E_REG_KP_INTERRUPT 0x02 */
#define PRESS_MASK(idx) (0x01 << (2 * (idx)))
#define RELEASE_MASK(idx) (0x02 << (2 * (idx)))

/* TS3A227E_REG_INTERRUPT_DISABLE 0x03 */
#define INS_REM_INT_DISABLE 0x01
#define DETECTION_COMPLETE_INT_DISABLE 0x02
#define ADC_COMPLETE_INT_DISABLE 0x04
#define INTB_DISABLE 0x08

/* TS3A227E_REG_SETTING_2 0x05 */
#define KP_ENABLE 0x04

/* TS3A227E_REG_ACCESSORY_STATUS  0x0b */
#define TYPE_3_POLE 0x01
#define TYPE_4_POLE_OMTP 0x02
#define TYPE_4_POLE_STANDARD 0x04
#define JACK_INSERTED 0x08
#define EITHER_MIC_MASK (TYPE_4_POLE_OMTP | TYPE_4_POLE_STANDARD)

/* TS3A227E_REG_SETTING_1 bit4 */
#define FORCE_DETECTION_TRIGGER     0x10

/* Button values to be reported on the jack */
static int ts3a227e_keycodes[] = {
	KEY_MEDIA,
	KEY_VOLUMEUP,
	KEY_VOICECOMMAND,
	KEY_VOLUMEDOWN,
	/*KEY_PREVIOUSSONG,*/
	/*KEY_NEXTSONG*/
};

enum KEY_STATUS {
	KEY_IS_UP = 0,
	KEY_IS_DOWN = 1,
	KEY_IS_WAIT_UP = 2,
};

struct ts3a227e_keystatus {
	int key_num;
	volatile int key_value;
};
struct   ts3a227e_keystatus key_status_map[] = {
	{KEY_MEDIA,  KEY_IS_UP},
	{KEY_VOLUMEUP,  KEY_IS_UP},
	{KEY_VOICECOMMAND, KEY_IS_UP},
	{KEY_VOLUMEDOWN, KEY_IS_UP},
};

static bool ts3a227e_readable_register(struct device *dev, unsigned int reg)
{
	pr_err("%s, reg:%x\n", __func__, reg);

	switch (reg) {
	case 0x00:
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x04:
	case 0x05:
	case 0x06:
	case 0x07:
	case 0x08:
	case 0x09:
	case 0x0A:
	case 0x0B:
	case 0x0C:
	case 0x0D:
	case 0x0E:
	case 0x0F:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config ts3a227e_regmap_physical = {
	.name = "physical",
	.reg_bits = 8,
	.val_bits = 16,

	.max_register = 0x0F + 1,
	.readable_reg = ts3a227e_readable_register,

	.cache_type = REGCACHE_NONE,
};

#define TS3A227E_NUM_KEYS ARRAY_SIZE(ts3a227e_keycodes)
#define TS3A227E_JACK_MASK (SND_JACK_HEADPHONE | SND_JACK_MICROPHONE)

static const struct reg_default ts3a227e_reg_defaults[] = {
	{ TS3A227E_REG_DEVICE_ID, 0x10 },
	{ TS3A227E_REG_INTERRUPT, 0x00 },
	{ TS3A227E_REG_KP_INTERRUPT, 0x00 },
	{ TS3A227E_REG_INTERRUPT_DISABLE, 0x08 },
	{ TS3A227E_REG_SETTING_1, 0x27 },
	{ TS3A227E_REG_SETTING_2, 0x00 },
	{ TS3A227E_REG_SETTING_3, 0x3f },
	{ TS3A227E_REG_SWITCH_CONTROL_1, 0x00 },
	{ TS3A227E_REG_SWITCH_CONTROL_2, 0x00 },
	{ TS3A227E_REG_SWITCH_STATUS_1, 0x0c },
	{ TS3A227E_REG_SWITCH_STATUS_2, 0x00 },
	{ TS3A227E_REG_ACCESSORY_STATUS, 0x00 },
	{ TS3A227E_REG_ADC_OUTPUT, 0x00 },
	{ TS3A227E_REG_KP_THRESHOLD_1, 0x20 },
	{ TS3A227E_REG_KP_THRESHOLD_2, 0x40 },
	{ TS3A227E_REG_KP_THRESHOLD_3, 0x68 },
};

static bool ts3a227e_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TS3A227E_REG_DEVICE_ID ... TS3A227E_REG_KP_THRESHOLD_3:
		return true;
	default:
		return false;
	}
}

static bool ts3a227e_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TS3A227E_REG_INTERRUPT_DISABLE ... TS3A227E_REG_SWITCH_CONTROL_2:
	case TS3A227E_REG_KP_THRESHOLD_1 ... TS3A227E_REG_KP_THRESHOLD_3:
		return true;
	default:
		return false;
	}
}

static bool ts3a227e_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TS3A227E_REG_INTERRUPT ... TS3A227E_REG_INTERRUPT_DISABLE:
	case TS3A227E_REG_SETTING_2:
	case TS3A227E_REG_SWITCH_STATUS_1 ... TS3A227E_REG_ADC_OUTPUT:
		return true;
	default:
		return false;
	}
}

static void ts3a227e_jack_report(struct ts3a227e *ts3a227e)
{
	pr_err("%s\n", __func__);

	if (ts3a227e->plugged) {
		pr_err("%s, HEADPHONE plug\n", __func__);
		//report = SND_JACK_HEADPHONE;

		if (ts3a227e->mic_present) {
			printk("%s, MICPHONE plug\n", __func__);
			//report |= SND_JACK_MICROPHONE;
			extcon_set_state(&ts3a227e->edev, BIT_HEADSET);

			rt5677_enable_micbias(true);
		}
		else {
			rt5677_enable_micbias(false);
			extcon_set_state(&ts3a227e->edev, BIT_HEADSET_NO_MIC);
		}
	}
	else {
		pr_err("%s, HEADSET unplug\n", __func__);
		                /* disable key press detection. */
                regmap_update_bits(ts3a227e->regmap, TS3A227E_REG_SETTING_2, KP_ENABLE, 0);
		extcon_set_state(&ts3a227e->edev, BIT_NO_HEADSET);
		rt5677_enable_micbias(false);
	}

	////snd_soc_jack_report(ts3a227e->jack, report, TS3A227E_JACK_MASK);
	//snd_soc_jack_report(ts3a227e->jack, report, SND_SOC_HEADSET);
}

static void ts3a227e_new_jack_state(struct ts3a227e *ts3a227e, unsigned acc_reg)
{
	bool plugged, mic_present;
	unsigned int i;

	plugged = !!(acc_reg & JACK_INSERTED);
	mic_present = plugged && !!(acc_reg & EITHER_MIC_MASK);

	ts3a227e->plugged = plugged;

	pr_err("%s, mic_present old status:%d\n", __func__, ts3a227e->mic_present);
	pr_err("%s, mic_present:%d , plugged is %d\n", __func__, mic_present, plugged);
	if (mic_present != ts3a227e->mic_present) {
		ts3a227e->mic_present = mic_present;
		for (i = 0; i < TS3A227E_NUM_KEYS; i++)
			input_report_key(ts3a227e->button_dev,
					 ts3a227e_keycodes[i], 0);

		/* Enable key press detection. */
		/*regmap_update_bits(ts3a227e->regmap, TS3A227E_REG_SETTING_2,
				   //KP_ENABLE, mic_present ? KP_ENABLE : 0);
				   KP_ENABLE, mic_present ? KP_ENABLE : KP_ENABLE); */
	}

	if(mic_present && plugged) {
		schedule_delayed_work(&ts3a227e->enable_key_detect_work, msecs_to_jiffies(2000));
	}
}


static void long_press_func(struct work_struct *work)
{
	unsigned int i;
	struct ts3a227e *ts3a227e = container_of(work,
		struct ts3a227e, long_press_work.work);

	pr_err("%s, id:%x\n", __func__, ts3a227e->id);

	/*volume up&down  long press */
	for (i = 0; i < TS3A227E_NUM_KEYS;  i++) {
		if (key_status_map[i].key_num == KEY_VOLUMEUP &&
			key_status_map[i].key_value == KEY_IS_DOWN) {
			key_status_map[i].key_value = KEY_IS_WAIT_UP;
			pr_err("%s, i:%x , key_lastsong down %d ,value:%d\n", __func__, i,
				KEY_PREVIOUSSONG, key_status_map[i].key_value);
			input_report_key(ts3a227e->button_dev, KEY_PREVIOUSSONG, 1);
			input_sync(ts3a227e->button_dev);
		}

		if (key_status_map[i].key_num == KEY_VOLUMEDOWN &&
			key_status_map[i].key_value == KEY_IS_DOWN) {
			key_status_map[i].key_value = KEY_IS_WAIT_UP;
			pr_err("%s, i:%x , key_nextsong down %d, value:%d\n", __func__, i,
				KEY_NEXTSONG, key_status_map[i].key_value);
			input_report_key(ts3a227e->button_dev, KEY_NEXTSONG, 1);
			input_sync(ts3a227e->button_dev);
		}
	}
}

static void check_jack_status(struct ts3a227e *ts3a227e)
{
	struct regmap *regmap = ts3a227e->regmap;
	unsigned int i;
	bool long_press_det = false;

	/* Check for plug/unplug. */
	if (ts3a227e->int_reg & (DETECTION_COMPLETE_EVENT | INS_REM_EVENT)) {
		//regmap_read(regmap, TS3A227E_REG_ACCESSORY_STATUS, &acc_reg);
		printk("%s acc_reg(0x0b): %0x\n", __func__, ts3a227e->acc_reg);
		ts3a227e_new_jack_state(ts3a227e, ts3a227e->acc_reg);
		/* this value filter input_report_key event when first plug into jack */
		ts3a227e->new_jack_plugin = 2;
	}

	if (ts3a227e->new_jack_plugin == 0) {
		/* Report any key events. */
		for (i = 0; i < TS3A227E_NUM_KEYS; i++) {
			if (ts3a227e->kp_int_reg & PRESS_MASK(i)) {
				pr_err("%s, kp %d down \n", __func__, i);
				if (key_status_map[i].key_num  != KEY_VOLUMEUP
					&&  key_status_map[i].key_num != KEY_VOLUMEDOWN)
					input_report_key(ts3a227e->button_dev, ts3a227e_keycodes[i], 1);
				else {
					key_status_map[i].key_value = KEY_IS_DOWN;
					long_press_det = true;
				}
			}

			if (ts3a227e->kp_int_reg & RELEASE_MASK(i)) {

				pr_err("%s, kp %d up\n", __func__, i);
				/* just for volume up & down long press */
				if (key_status_map[i].key_value == KEY_IS_DOWN) {
						pr_err("%s, kp %d short up \n", __func__, i);
						cancel_delayed_work_sync(&ts3a227e->long_press_work);
						input_report_key(ts3a227e->button_dev,
						 ts3a227e_keycodes[i], 1);
						key_status_map[i].key_value = KEY_IS_UP;
				} else if (key_status_map[i].key_value == KEY_IS_WAIT_UP) {
					pr_err("%s, kp %d long up  code:%d\n", __func__, i, key_status_map[i].key_num);
					if (key_status_map[i].key_num == KEY_VOLUMEUP)
						input_report_key(ts3a227e->button_dev, KEY_PREVIOUSSONG, 0);
					else if (key_status_map[i].key_num == KEY_VOLUMEDOWN)
						input_report_key(ts3a227e->button_dev, KEY_NEXTSONG, 0);

					key_status_map[i].key_value = KEY_IS_UP;
				}
				input_report_key(ts3a227e->button_dev,
						 ts3a227e_keycodes[i], 0);
			}
		}

		if (long_press_det == true) {
			cancel_delayed_work_sync(&ts3a227e->long_press_work);
			schedule_delayed_work(&ts3a227e->long_press_work,
		      msecs_to_jiffies(500));
		}
	}

	if (ts3a227e->new_jack_plugin) ts3a227e->new_jack_plugin--;

	input_sync(ts3a227e->button_dev);
	ts3a227e_jack_report(ts3a227e);
}

static void hs_detect_func(struct work_struct *work)
{
	struct ts3a227e *ts3a227e = container_of(work,
		struct ts3a227e, hs_det_work.work);

	pr_err("%s, id:%x\n", __func__, ts3a227e->id);
	check_jack_status(ts3a227e);
}

static void enable_key_detect_func(struct work_struct *work)
{
        struct ts3a227e *ts3a227e = container_of(work,
                struct ts3a227e, enable_key_detect_work.work);

                /* Enable key press detection. */
        regmap_update_bits(ts3a227e->regmap, TS3A227E_REG_SETTING_2,
                          //KP_ENABLE, mic_present ? KP_ENABLE : 0);
                          KP_ENABLE, ts3a227e->mic_present ? KP_ENABLE : KP_ENABLE);
}


static irqreturn_t ts3a227e_interrupt(int irq, void *data)
{
	struct ts3a227e *ts3a227e = (struct ts3a227e *)data;
	struct regmap *regmap = ts3a227e->regmap;
	unsigned int kp_enable = 0;
	//unsigned int int_reg, kp_int_reg, acc_reg;

	/* Check for plug/unplug. */
	regmap_read(regmap, TS3A227E_REG_INTERRUPT, &(ts3a227e->int_reg));
	pr_err("%s ent, int_reg(0x01): %0x\n", __func__, ts3a227e->int_reg);

	if (ts3a227e->int_reg & (DETECTION_COMPLETE_EVENT | INS_REM_EVENT)) {
		regmap_read(regmap, TS3A227E_REG_ACCESSORY_STATUS, &(ts3a227e->acc_reg));
		pr_err("%s acc_reg(0x0b): %0x\n", __func__, ts3a227e->acc_reg);
		//ts3a227e_new_jack_state(ts3a227e, acc_reg);
	}

	/* Report any key events. */
	regmap_read(regmap, TS3A227E_REG_SETTING_2, &kp_enable);
	regmap_read(regmap, TS3A227E_REG_KP_INTERRUPT, &(ts3a227e->kp_int_reg));
	if(!(kp_enable & KP_ENABLE)){// key press detect is disabled
            ts3a227e->kp_int_reg = 0;
	    pr_err("%s, clear kp_int_reg, as key press is disabled\n",__func__);
        }

	cancel_delayed_work_sync(&ts3a227e->hs_det_work);
	schedule_delayed_work(&ts3a227e->hs_det_work,
		      msecs_to_jiffies(5));

	return IRQ_HANDLED;
}

/**
 * ts3a227e_enable_jack_detect - Specify a jack for event reporting
 *
 *  <at> codec:  codec to register the jack with
 *  <at> jack: jack to use to report headphone/mic plug/unplug on
 *
 * After this function has been called the headset insert/remove  will be routed
 * to the given jack.  Jack can be null to stop reporting.
 */
int ts3a227e_enable_jack_detect(struct snd_soc_codec *codec,
				struct snd_soc_jack *jack)
{

	return 0;
}
EXPORT_SYMBOL_GPL(ts3a227e_enable_jack_detect);

static struct snd_soc_codec_driver ts3a227e_codec_driver;

static const struct regmap_config ts3a227e_regmap_config = {
	.val_bits = 8,
	.reg_bits = 8,

	.max_register = TS3A227E_REG_KP_THRESHOLD_3,
	.readable_reg = ts3a227e_readable_reg,
	.writeable_reg = ts3a227e_writeable_reg,
	.volatile_reg = ts3a227e_volatile_reg,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = ts3a227e_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ts3a227e_reg_defaults),
};

static int ts3a227e_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct ts3a227e *ts3a227e;
	struct device *dev = &i2c->dev;
	unsigned int i;
	int ret;

	printk("%s\n", __func__);
	ts3a227e = devm_kzalloc(&i2c->dev, sizeof(*ts3a227e), GFP_KERNEL);
	if (!ts3a227e)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ts3a227e);
	ts3a227e->id = 0x3227e;
	ts3a227e->dev = &i2c->dev;

	ts3a227e->button_dev = devm_input_allocate_device(dev);
	if (!ts3a227e->button_dev) {
		dev_err(dev, "Failed to allocate button_dev\n");
		return -ENOMEM;
	}

	ts3a227e->button_dev->evbit[0] = BIT_MASK(EV_KEY);
	for (i = 0; i < TS3A227E_NUM_KEYS; i++)
		ts3a227e->button_dev->keybit[BIT_WORD(ts3a227e_keycodes[i])] |=
				BIT_MASK(ts3a227e_keycodes[i]);

	input_set_capability(ts3a227e->button_dev, EV_KEY, KEY_PREVIOUSSONG);
	input_set_capability(ts3a227e->button_dev, EV_KEY, KEY_NEXTSONG);

	ts3a227e->button_dev->name = "ts3a227e headset buttons";
	ret = input_register_device(ts3a227e->button_dev);
	if (ret) {
		dev_err(dev, "Failed to register input dev %d\n", ret);
		return ret;
	}

	ts3a227e->regmap_physical = devm_regmap_init_i2c(i2c,
					&ts3a227e_regmap_physical);
	if (IS_ERR(ts3a227e->regmap_physical)) {
		ret = PTR_ERR(ts3a227e->regmap_physical);
		//dev_err(&i2c->dev, "%s, Failed to allocate register map: %d\n",
		pr_err ("%s, Failed to allocate register map: %d\n",
			__func__, ret);
		return ret;
	}

	ts3a227e->regmap = devm_regmap_init_i2c(i2c, &ts3a227e_regmap_config);
	if (IS_ERR(ts3a227e->regmap))
		return PTR_ERR(ts3a227e->regmap);

	ret = gpio_request_one(GPIO_MIC_SWITCH_IRQ, GPIOF_DIR_IN | GPIOF_INIT_HIGH, "ti switch");
	i2c->irq = gpio_to_irq(GPIO_MIC_SWITCH_IRQ);
	printk("%s, hs_irq:%d\n", __func__, i2c->irq);
	INIT_DELAYED_WORK(&ts3a227e->hs_det_work, hs_detect_func);
	INIT_DELAYED_WORK(&ts3a227e->enable_key_detect_work, enable_key_detect_func);
	INIT_DELAYED_WORK(&ts3a227e->long_press_work, long_press_func);
	if (i2c->irq != -1) {
		ret = devm_request_threaded_irq(dev, i2c->irq, NULL, ts3a227e_interrupt,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"TS3A227E", ts3a227e);
		if (ret) {
			dev_err(dev, "Cannot request irq %d (%d)\n", i2c->irq, ret);
			return ret;
		}
	}

	ret = snd_soc_register_codec(&i2c->dev, &ts3a227e_codec_driver,
				     NULL, 0);
	if (ret) {
		printk("%s failed to reg codec\n", __func__);
		return ret;
	}

	/* Enable interrupts except for ADC complete. */
	regmap_update_bits(ts3a227e->regmap, TS3A227E_REG_INTERRUPT_DISABLE,
			   INTB_DISABLE | ADC_COMPLETE_INT_DISABLE,
			   ADC_COMPLETE_INT_DISABLE);

	ts3a227e->edev.name = "h2w";
	ret = extcon_dev_register(&ts3a227e->edev);
	if (ret < 0) {
		printk("extcon_dev_register() failed: %d\n", ret);
		return ret;
	}

	printk("%s: the HW version is: %d \n", __func__, yeti_hw_ver);
	if (yeti_hw_ver < 2) {
		ts3a227e_keycodes[1] = KEY_VOICECOMMAND;
		ts3a227e_keycodes[2] = KEY_VOLUMEUP;
		key_status_map[1].key_num = KEY_VOICECOMMAND;
		key_status_map[2].key_num = KEY_VOLUMEUP;
	}

        regmap_update_bits(ts3a227e->regmap, TS3A227E_REG_SETTING_1,
                FORCE_DETECTION_TRIGGER, FORCE_DETECTION_TRIGGER);

	printk("%s successed\n", __func__);
	return 0;
}

static int ts3a227e_i2c_remove(struct i2c_client *i2c)
{
	struct ts3a227e *ts3a227e = i2c_get_clientdata(i2c);

	input_unregister_device(ts3a227e->button_dev);
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static const struct i2c_device_id ts3a227e_i2c_ids[] = {
	{ "ts3a227e", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ts3a227e_i2c_ids);

static const struct of_device_id ts3a227e_of_match[] = {
	{ .compatible = "ti,ts3a227e", },
	{ }
};
MODULE_DEVICE_TABLE(of, ts3a227e_of_match);

static struct i2c_driver ts3a227e_driver = {
	.driver = {
		.name = "ts3a227e",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ts3a227e_of_match),
	},
	.probe = ts3a227e_i2c_probe,
	.remove = ts3a227e_i2c_remove,
	.id_table = ts3a227e_i2c_ids,
};
module_i2c_driver(ts3a227e_driver);
#if 1
static int __init ts3a227e_i2c_dev_init(void)
{
	int i2c_busnum = 0;
	struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
	void *pdata = NULL;

	printk("%s ...\n", __func__);

	memset(&i2c_info, 0, sizeof(i2c_info));
	strncpy(i2c_info.type, "ts3a227e", strlen("ts3a227e"));

	i2c_info.addr = 0x3b;

	printk(KERN_ERR "%s I2C bus = %d, name = %s, irq = 0x%2x, addr = 0x%x\n",
	            __func__,
	            i2c_busnum,
	            i2c_info.type,
	            i2c_info.irq,
	            i2c_info.addr);

	if(pdata != NULL)
		i2c_info.platform_data = pdata;
	else
		printk("%s, pdata is NULL\n", __func__);

	adapter = i2c_get_adapter(i2c_busnum);
	if (adapter) {
		if (i2c_new_device(adapter, &i2c_info)) {
			printk("add new i2c device %s , addr 0x%x\n", "ts3a227e", i2c_info.addr);
			return 0;
		}else{
			printk("add new i2c device %s , addr 0x%x fail !!!\n", "ts3a227e", i2c_info.addr);
		}
	}else{
		printk("[%s]get adapter %d fail\n",__func__, i2c_busnum);
		return -EINVAL;
	}
}
late_initcall_sync(ts3a227e_i2c_dev_init);
#endif

MODULE_DESCRIPTION("ASoC ts3a227e driver");
MODULE_AUTHOR("Dylan Reid <dgreid <at> chromium.org>");
MODULE_LICENSE("GPL v2");

