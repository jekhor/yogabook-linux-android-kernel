/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2013 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/unaligned.h>
#include <linux/input/wacom.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#define YETI
//#define KEY_FEATURE	//Modify by xucm1,20160721,for YETIM-7053 remove this feature.

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif //CONFIG_HAS_EARLYSUSPEND

#define WACOM_CMD_QUERY0	0x04
#define WACOM_CMD_QUERY1	0x00
#define WACOM_CMD_QUERY2	0x33
#define WACOM_CMD_QUERY3	0x02
#define WACOM_CMD_THROW0	0x05
#define WACOM_CMD_THROW1	0x00
#define WACOM_QUERY_SIZE	19
#define WACOM_INPUT_SIZE        10
#define AA_OFFSET               100

#define DEBUG   0

#if  DEBUG
#define WACOM_INFO(a,arg...)     printk("Wacom EMR" a,##arg)
#else
#define WACOM_INFO(a,arg...)     do {} while (0)
#endif


#ifdef YETI
#define PEN_REPORT_ID           2
#define KEY_REPORT_ID           22
#endif

#define KERNEL_OLDER_33          1

struct wacom_features {
	int x_max;
	int y_max;
	int pressure_max;
	int  fw_version;
};

struct wacom_i2c {
	struct i2c_client *client;
	struct input_dev *input;
	struct wacom_features features;
	struct wacom_platform_data *pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	u8 data[WACOM_QUERY_SIZE];
	bool prox;
	int tool;
	int is_suspended;
	int tp_mode;
};

#ifdef KEY_FEATURE
/*
KEY_DELETE
KEY_F20
KEY_LEFTMETA
KEY_ESC
KEY_LEFTCTRL
KEY_LEFTALT
*/
#define WAC_CTRL_ALT 0x05
#define WAC_DELETE   0x4c
#define WAC_L_WIN    0x08
#define WAC_F20      0x6f
#define WAC_ESC      0x29

static u16 wac_keys[3][2] = {
	{WAC_CTRL_ALT, WAC_DELETE}, 
	{WAC_L_WIN, WAC_F20}, 
	{0, WAC_ESC},
};

static u16 event_keys[3][2] = {
	{KEY_PREVIOUS, KEY_RESERVED}, 
	{KEY_NEXT, KEY_RESERVED}, 
	{KEY_RESERVED, KEY_SAVE},
};

#endif

static void wacom_i2c_set_sleep(struct device *dev, bool into_sleep);

static int wacom_gpio_setup(int gpio, int output, int state)
{
	int retval = 0;
	unsigned char buf[16];

	WACOM_INFO("%s, gpio=%d, configure=%d, dir=%d, state=%d\n", __func__, gpio, output, state);

	snprintf(buf, PAGE_SIZE, "wacom_gpio_%u\n", gpio);

	retval = gpio_request(gpio, buf);
	if (retval) {
		pr_err("%s: Failed to get gpio %d (code: %d)",
				__func__, gpio, retval);
		return retval;
	}

	if (output == 0)
		retval = gpio_direction_input(gpio);
	else
		retval = gpio_direction_output(gpio, state);
	if (retval) {
		pr_err("%s: Failed to set gpio %d direction",
				__func__, gpio);
		return retval;
	}
	return retval;
}

static int wacom_power_on(struct wacom_i2c *data, bool on)
{
	int rc=0;

	if (!on)
		goto power_off;

  	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
        return rc;
	}
	
	//msleep(50); //don`t delay, vdd VPROG3B have opened by KB before wacom.

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
        return rc;
	}

	msleep(20);
    return 0;
	

power_off:
    
    
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
    return 0;    	
}

static int wacom_power_init(struct wacom_i2c *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	if (yeti_hw_ver >= 2 )
		data->vdd = regulator_get(&data->client->dev, "VPROG3B");
	else
		data->vdd = regulator_get(&data->client->dev, "VPROG4D");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator VPROG6A 3.3 v get failed vdd rc=%d\n", rc);
		goto err;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, WACOM_VTG_MIN_UV,
					   WACOM_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg VPROG6A failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}



	data->vcc_i2c = regulator_get(&data->client->dev, "VPROG5B");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_put;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, WACOM_I2C_VTG_MIN_UV,
					   WACOM_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}


	return 0;
    

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);

reg_vdd_put:
	regulator_put(data->vdd);
err:
	return rc;
pwr_deinit:
	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, WACOM_I2C_VTG_MAX_UV);
		regulator_put(data->vcc_i2c);
		
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, WACOM_VTG_MAX_UV);
		regulator_put(data->vdd);
	return 0;
}

static int wacom_reset_device(struct wacom_i2c *data)
{
        int error;

        WACOM_INFO("%s \n", __func__);
        error = gpio_direction_output(data->pdata->gpio_reset, 0);
        if (error) {
                pr_err("%s: Failed to set gpio %d direction",
                    __func__, data->pdata->gpio_reset);
                return error;
        }
        msleep(100);

        error = gpio_direction_output(data->pdata->gpio_reset, 1);
        if (error) {
                pr_err("%s: Failed to set gpio %d direction",
                    __func__, data->pdata->gpio_reset);
                    return error;
        }

}

static int wacom_query_device(struct i2c_client *client,
			      struct wacom_features *features)
{
	int ret;
	u8 cmd1[] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1,
			WACOM_CMD_QUERY2, WACOM_CMD_QUERY3 };
	u8 cmd2[] = { WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
	u8 data[WACOM_QUERY_SIZE];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd2),
			.buf = cmd2,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	features->x_max = get_unaligned_le16(&data[3]);
	features->y_max = get_unaligned_le16(&data[5]);
	features->pressure_max = get_unaligned_le16(&data[11]);
	features->fw_version = get_unaligned_le16(&data[13]);

	dev_dbg(&client->dev,
		"x_max:%d, y_max:%d, pressure:%d, fw:%d\n",
		features->x_max, features->y_max,
		features->pressure_max, features->fw_version);

	return 0;
}

/*Eliminating the digitizer AA offset; this makes the coordination exactly fit to the LCD size*/
static void set_offset(int *x, int *y, int x_max, int y_max)
{
	int temp_coord = *x - AA_OFFSET;
	
	if (temp_coord < 0)
		*x = 0;
	else if (temp_coord > x_max)
		*x = x_max;
	else
		*x = temp_coord;
	
	temp_coord = *y - AA_OFFSET;
	
	if (temp_coord < 0)
		*y = 0;
	else if (temp_coord > y_max)
		*y = y_max;
	else
		*y = temp_coord;
}

#ifdef YETI
#ifdef KEY_FEATURE
static void wacom_i2c_key(struct wacom_i2c *wac_i2c, struct input_dev *input, u8 *data)
{
	int i, j;

	for (i = 0; i < 3; i++)
		for (j = 0; j < 2; j++) {
			input_report_key(input, event_keys[i][j], (wac_keys[i][j] == data[j * 2 + 3]));
			input_sync(input);
		}
}
#endif
//#define FW_1428

static void wacom_i2c_emr(struct wacom_i2c *wac_i2c, struct input_dev *input, u8 *data)
{
	unsigned int x, y, pressure, temp;
	unsigned char tsw, f1, f2, ers;

	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);

#ifdef FW_1428
	temp = x;
	x = y;
	y = (wac_i2c->features.y_max - temp);
#endif

	set_offset(&x, &y, wac_i2c->features.x_max, wac_i2c->features.y_max);

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;

	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_sync(input);

	return;
}

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	int error;
	u8 *data = wac_i2c->data;

	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, WACOM_INPUT_SIZE);
	if (error < 0)
		goto out;

	switch (data[2]) {
	case PEN_REPORT_ID:
		wacom_i2c_emr(wac_i2c, input, data);
		break;
#ifdef KEY_FEATURE
	case KEY_REPORT_ID:
		wacom_i2c_key(wac_i2c, input, data);
		break;
#endif
	}

out:
	return IRQ_HANDLED;
}
#else
static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	unsigned int x, y, pressure;
	unsigned char tsw, f1, f2, ers;
	int error;
	u8 *data = wac_i2c->data;

	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, WACOM_INPUT_SIZE);
	if (error < 0)
		goto out;

	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);

	set_offset(&x, &y, wac_i2c->features.x_max, wac_i2c->features.y_max);

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;

	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_sync(input);


out:
	return IRQ_HANDLED;


#endif

static int wacom_i2c_open(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	enable_irq(client->irq);

	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	disable_irq(client->irq);
}


#ifdef CONFIG_PM_SLEEP
static int wacom_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *ts = i2c_get_clientdata(client);

	if (ts->is_suspended == 0 ) {
		disable_irq(client->irq);
		wacom_i2c_set_sleep(dev, true);
	}
	ts->is_suspended = 1;
	return 0;
}

static int wacom_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *ts = i2c_get_clientdata(client);

	if (ts->tp_mode == 2 ) {
		printk("%s tp mode %d\n", __func__, ts->tp_mode);
		return 0;
	}
	if (ts->is_suspended) {
		wacom_i2c_set_sleep(dev, false);
		enable_irq(client->irq);
	}
	ts->is_suspended = 0;
	return 0;
}
static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

static void wacom_i2c_set_sleep(struct device *dev, bool into_sleep)
{
	int ret = -1;
	char cmd[4] = {0x04, 0x00, 0x01, 0x08};
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *ts = i2c_get_clientdata(client);

	if (!into_sleep)
		cmd[2] = 0x00;

	ret = i2c_master_send(client, cmd, 4);
	if (ret != 4) {
		printk("Failed: %s into %d\n", __func__, into_sleep);
        if (!into_sleep)
            wacom_reset_device(ts);
	} else {
		printk("Succeeded: %s\n", __func__);
	}

	return;
}
#ifdef CONFIG_HAS_EARLYSUSPEND

static void wacom_i2c_early_suspend(struct early_suspend *handler)
{
	struct wacom_i2c *wac_i2c = container_of(handler, struct wacom_i2c, early_suspend);
	struct device *dev = &wac_i2c->client->dev;

	printk("%s()\r\n", __func__);
	wacom_i2c_suspend(dev);
	wacom_i2c_set_sleep(dev, true);
	
	return;
}

static void wacom_i2c_late_resume(struct early_suspend *handler)
{
	struct wacom_i2c *wac_i2c = container_of(handler, struct wacom_i2c, early_suspend);
	struct device *dev = &wac_i2c->client->dev;

	printk("%s()\r\n", __func__);
	wacom_i2c_set_sleep(dev, false);
	wacom_i2c_resume(dev);

	return;
}
#endif //CONFIG_HAS_EARLYSUSPEND
#endif //CONFIG_PM_SLEEP


static ssize_t tp_mode_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int tp_mode;
    struct i2c_client *client = to_i2c_client(dev); 
    struct wacom_i2c *ts = i2c_get_clientdata(client);

    tp_mode = simple_strtoul(buf, NULL, 0);
    switch(tp_mode)
    {
	case 2:
	ts->tp_mode = tp_mode;
        wacom_i2c_suspend(dev);
	break;
	case 3:
	ts->tp_mode = tp_mode;
        wacom_i2c_resume(dev);
	break;
	default:
	printk("%s invalid mode \n",__func__);
	break;
    }
    return count;
}

static ssize_t tp_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev); 
    struct wacom_i2c *ts = i2c_get_clientdata(client);
    return scnprintf(buf, PAGE_SIZE, "%d\n",ts->is_suspended?2:3);
}



static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct wacom_i2c *ts = i2c_get_clientdata(client);
    return scnprintf(buf, PAGE_SIZE, "%x\n",ts->features.fw_version);
}


static ssize_t enable_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{

        unsigned int val;
        int error;

        error = kstrtouint(buf, 10, &val);
        if (error)
                return error;

        printk("%s %d\n", __func__, val);
        return count;

}
//static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, NULL, enable_store);
static DEVICE_ATTR(tp_mode, S_IRUGO|S_IWUSR|S_IWGRP, tp_mode_show, tp_mode_write);
static DEVICE_ATTR(fw_ver, S_IRUGO|S_IRUSR|S_IRGRP, fw_ver_show, NULL);

static struct attribute *wacom_attributes[] = {
   /* &dev_attr_enable.attr, */
   &dev_attr_tp_mode.attr,
   &dev_attr_fw_ver.attr,
   NULL
   };

static const struct attribute_group wacom_attr_group = {
   .attrs = wacom_attributes,
   };


static int wacom_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct wacom_i2c *wac_i2c;
	struct input_dev *input;
	struct wacom_features features = { 0 };
	int error;
#ifdef KEY_FEATURE
	int i, j;
#endif


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}


	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	input = input_allocate_device();
	if (!wac_i2c || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	wac_i2c->client = client;
	wac_i2c->input = input;
    wac_i2c->pdata = client->dev.platform_data;

	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x56a;
	input->id.version = features.fw_version;
	input->dev.parent = &client->dev;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    error = wacom_power_init(wac_i2c, true);
	if (error) {
		dev_err(&client->dev, "power init failed");
		goto err_free_mem;
	}
    error = wacom_power_on(wac_i2c, true);
	if (error) {
		dev_err(&client->dev, "power on failed");
		goto err_free_mem;
	}
	WACOM_INFO("power on wacom device\n");

	error = wacom_query_device(client, &features);
	if (error)
		goto err_free_mem;
	
#if 1
	if(gpio_is_valid(wac_i2c->pdata->gpio_reset)){
		error = gpio_request(wac_i2c->pdata->gpio_reset, "wacom_reset");
		if (error) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
				__func__, wac_i2c->pdata->gpio_reset, error);
			return error;
		}
#if 0
		error = gpio_direction_output(wac_i2c->pdata->gpio_reset, 0);
		if (error) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, wac_i2c->pdata->gpio_reset);
			return error;
		}
		msleep(30);
#endif

		error = gpio_direction_output(wac_i2c->pdata->gpio_reset, 1);
		if (error) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, wac_i2c->pdata->gpio_reset);
			return error;
		}
		msleep(10);
	}
#endif

	if(gpio_is_valid(wac_i2c->pdata->gpio_int)){
		error = gpio_request(wac_i2c->pdata->gpio_int, "wacom_interrupt");
		if (error) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
				__func__, wac_i2c->pdata->gpio_int, error);
			return error;
		}
		error = gpio_direction_input(wac_i2c->pdata->gpio_int);
		if (error) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, wac_i2c->pdata->gpio_int);
			return error;
		}
		wac_i2c->client->irq = gpio_to_irq (wac_i2c->pdata->gpio_int);
		WACOM_INFO(" %s irq %d\n", __func__, wac_i2c->client->irq);
	}

	WACOM_INFO("set wacom gpio\n");

	error = wacom_query_device(client, &features);
	if (error){
		printk("wacom_query_device fail\n");
		wacom_power_init(wac_i2c, false);
		gpio_free(wac_i2c->pdata->gpio_int);
		gpio_free(wac_i2c->pdata->gpio_reset);
		return error;
	}

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);
	
#ifdef KEY_FEATURE
	for (i = 0; i < 3; i++)
		for (j = 0; j < 2; j++)
			__set_bit(event_keys[i][j], input->keybit);
#endif

	/*Setting maximum coordinate values  */
	/*eliminating 1mm offset on each side*/
	features.x_max -= (AA_OFFSET * 2);
	features.y_max -= (AA_OFFSET * 2);

#ifdef FW_1428
	temp = features.x_max;
	features.x_max = features.y_max;
	features.y_max = temp;
#endif

	/*Save all features to the struct pointer*/
	wac_i2c->features = features;

	printk("feature_xmax: %d feature_ymax: %d \n", wac_i2c->features.x_max, wac_i2c->features.y_max);

	input_set_abs_params(input, ABS_X, 0, features.x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, features.y_max, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE,
			     0, features.pressure_max, 0, 0);

	input_set_drvdata(input, wac_i2c);

	error = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "wacom_i2c", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_mem;
	}

	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
	disable_irq(client->irq);
	wac_i2c->is_suspended = 0;

	error = input_register_device(wac_i2c->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_irq;
	}

	i2c_set_clientdata(client, wac_i2c);

#ifdef CONFIG_HAS_EARLYSUSPEND
	wac_i2c->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	wac_i2c->early_suspend.suspend = wacom_i2c_early_suspend;
	wac_i2c->early_suspend.resume  = wacom_i2c_late_resume;
	register_early_suspend(&wac_i2c->early_suspend);
	printk("%s: early_suspend registered \n", __func__);
#endif //CONFIG_HAS_EARLYSUSPEND

	error = sysfs_create_group(&client->dev.kobj, &wacom_attr_group);
        if (error) {
                printk("%s create attr fail \n", __func__);
        }
	printk("##C Face:EMR pen probe success##\n");

	return 0;
	
err_free_irq:
	free_irq(client->irq, wac_i2c);
err_free_mem:
	input_free_device(input);
	kfree(wac_i2c);

	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&wac_i2c->early_suspend);
	printk("%s: early_suspend unregistered \n", __func__);
#endif //CONFIG_HAS_EARLYSUSPEND

	free_irq(client->irq, wac_i2c);
	input_unregister_device(wac_i2c->input);
	kfree(wac_i2c);

	return 0;
}

static const struct i2c_device_id wacom_i2c_id[] = {
	{ MAGTOUCH_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.owner	= THIS_MODULE,
		.pm	= &wacom_i2c_pm,
	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.id_table	= wacom_i2c_id,
};

#if KERNEL_OLDER_33
static int __init wacom_i2c_init(void)
{
	return i2c_add_driver(&wacom_i2c_driver);
}

static void __exit wacom_i2c_exit(void)
{
	i2c_del_driver(&wacom_i2c_driver);
}

//module_init(wacom_i2c_init);
late_initcall(wacom_i2c_init);
module_exit(wacom_i2c_exit);
#else
module_i2c_driver(wacom_i2c_driver);
#endif

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");
