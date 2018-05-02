
/*******************************************************************************
 * Copyright (C) 2014 HiDeep, Inc.
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
 *******************************************************************************/


#include <linux/input/ist520e.h>


/*******************************************************************************
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 * 0x00: menu key
 * 0x01: home key
 * 0x02: back key
 *-----------------------------------------------------------------------------*/


struct ist510e *ts;
unsigned int g_loglevel = 0;

#ifdef IST510E_KEYBUTTON
static int tsp_keycodes[] = {KEY_MENU, KEY_HOMEPAGE, KEY_BACK};
#endif

static int istcore_suspend(struct device *dev, pm_message_t mesg);
static int istcore_resume(struct device *dev);
#if 1
extern void hideep_set_pmic_power(bool flag, int delay);
#else
extern void hideep_set_pmic_power_1V8(bool flag, int delay);
extern void hideep_set_pmic_power_3V3(bool flag, int delay);
#endif
extern int hideep_gpio_setup(int gpio, bool configure, int dir, int state);

/*******************************************************************************
 * interface
 *******************************************************************************/
void hideep_disable_irq_nosync(void)
{
	ISTCORE_INFO("istcore_ enter ts->irq_enable = %d\n", ts->irq_enable);
	disable_irq_nosync(ts->client->irq);
	ts->irq_enable = false;
	return;
}
void hideep_irq_enable(bool en)
{
    ISTCORE_INFO("istcore_ enter en = %d, ts->irq_enable = %d\n", en, ts->irq_enable);
    if(en == ts->irq_enable)
        return;
    if(en)
        enable_irq(ts->client->irq);
    else
        disable_irq(ts->client->irq);
    ts->irq_enable = en;
    return;
}

#define I2C_RETRY	10
int istcore_i2c_read(struct ist510e *ts, u16 addr, u16 len, u8 *buf)
{
#ifdef HIDEEP_REPEAT_START
    int retry = 0;
    int ret;
    struct i2c_msg xfer[2];
    struct i2c_client *client = ts->client;

    ISTCORE_I2C("repeat start addr=0x%02x, len=%d\n", addr, len);
    mutex_lock(&ts->i2c_mutex);
    xfer[0].addr = client->addr;
    xfer[0].flags = 0;
    xfer[0].len = 2;
    xfer[0].buf = &addr;
    xfer[1].addr = client->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = len;
    xfer[1].buf = buf;

    retry = 0;
    do{
        ret = i2c_transfer(client->adapter, xfer,2);
        if(ret == 2)
            break;
        msleep(2);
        ISTCORE_ERR("retry = %d\n", retry+1);
    }while(++retry<I2C_RETRY);
    mutex_unlock(&ts->i2c_mutex);
    return retry<I2C_RETRY?0:-1;
#else
    int ret = -1;
    int retry = 0;
    struct i2c_client *client = ts->client;

    ISTCORE_I2C("normal addr=0x%02x, len=%d\n", addr, len);
    mutex_lock(&ts->i2c_mutex);

	for (retry = 0; retry < I2C_RETRY; retry++) {
		ret = i2c_master_send(client, (char *) &addr, 2 );
		if (ret < 0){
			ISTCORE_ERR("i2c send error, ret = %d\n", ret);
			goto i2c_err;
		}

		ret = i2c_master_recv(client, (char *) buf,  len);
		if (ret < 0){
			ISTCORE_ERR("i2c recv error, ret = %d\n", ret);
			goto i2c_err;
		}

		break;
i2c_err:
		msleep(4);
	}

    mutex_unlock(&ts->i2c_mutex);
    return retry < I2C_RETRY ? 0:-1;
#endif
}
/*------------------------------------------------------------------------------
 *i2c write : not include address
 *-----------------------------------------------------------------------------*/
int istcore_i2c_write(struct ist510e *ts, u16 addr, u16 len, u8 *buf)
{
    int ret = -1;
    int retry = 0;
    struct i2c_client *client = ts->client;

    ISTCORE_I2C( "addr=0x%02x, len=%d \n", addr, len);
    mutex_lock(&ts->i2c_mutex);
    ts->seg_buff[0] = (addr >> 0) & 0xFF;
    ts->seg_buff[1] = (addr >> 8) & 0xFF;
    memcpy( &ts->seg_buff[2], buf, len);

    for(retry = 0; retry < I2C_RETRY; retry++) {
		ret = i2c_master_send(client, (char *) ts->seg_buff, len+2);
		if (ret < 0){
			ISTCORE_ERR("i2c send error, ret = %d\n", ret);
			msleep(4);
			continue;
		}
		break;
	}

    mutex_unlock(&ts->i2c_mutex);
    return retry < I2C_RETRY ? 0:-1;
}
/*******************************************************************************
 * kernel task for input subsystem.
 *******************************************************************************/
static void pops_mt(struct input_dev *input_dev, struct ist510e_touch_evt *finger, s32 nr)
{
#ifdef HIDEEP_TYPE_B_PROTOCOL
    s32 id;
    s32 i;
    for (i = 0; i < nr; i++){
        id      = (finger[i].index >> 0 ) & 0x0F;
        input_mt_slot(input_dev, id);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
        input_report_key(input_dev, BTN_TOUCH, false );
    }
#else
    input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync(input_dev);
#endif
    input_sync(input_dev);
    return;
}
/*------------------------------------------------------------------------------
 * TODO : how to handle btn_touch event..
 *-----------------------------------------------------------------------------*/
static void push_mt(struct ist510e *ts, struct input_dev *input_dev, struct ist510e_touch_evt *finger, s32 nr)
{
    s32 id;
    s32 i;
    bool  btn_up ;
    bool  btn_dn ;
    bool  btn_mv ;
    int  evt = 0;

    // load multi-touch event to input system
    for (i = 0; i < nr; i++){
        id      = (finger[i].index >> 0               ) & 0x0F;
        btn_up  = (finger[i].flag  >> EV_RELEASED     ) & 0x01;
        btn_dn  = (finger[i].flag  >> EV_FIRST_CONTACT) & 0x01;
        btn_mv  = (finger[i].flag  >> EV_DRAG_MOVE    ) & 0x01;
        if(btn_up)
            clear_bit(id, &ts->tch_bit);
		else
            __set_bit(id, &ts->tch_bit);
		//ISTCORE_XY("i=%d, x=%d, y=%d,z=%d\n",i, finger[i].x, finger[i].y, finger[i].z);
#ifdef HIDEEP_TYPE_B_PROTOCOL
        input_mt_slot(input_dev, id);
        if (btn_up){
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
		}else{
            input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,  true);
#else
        if (btn_up){
			input_mt_sync(input_dev);
		}else{
			input_report_abs(input_dev, ABS_MT_TRACKING_ID ,  id);
#endif
			if(finger[i].z<10)
				finger[i].z = 10;
            input_report_abs(input_dev, ABS_MT_POSITION_X ,  finger[i].x);
            input_report_abs(input_dev, ABS_MT_POSITION_Y ,  finger[i].y);
            input_report_abs(input_dev, ABS_MT_PRESSURE   ,  finger[i].z);
            input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,  finger[i].w);
            evt++;
#ifndef HIDEEP_TYPE_B_PROTOCOL
			input_mt_sync(input_dev);
#endif
        }
    }
    // sync
    if(ts->tch_bit == 0)
        evt = 0;
    input_report_key(input_dev, BTN_TOUCH,  evt);
    input_sync(input_dev);
    return;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static void pops_ky(struct input_dev *input_dev, struct ist510e_touch_key *key, s32 nr)
{
    s32 i;

    for (i = 0; i < nr; i++){
        input_report_key(input_dev, tsp_keycodes[i], false);
        input_report_key(input_dev, BTN_TOUCH,       false);
    }
    input_sync(input_dev);
    return;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static void push_ky(struct input_dev *input_dev, struct ist510e_touch_key *key, s32 nr)
{
    s32 i;
    s32 pressed ;
    s32 key_code;
    s32 key_status;

    for (i = 0; i < nr; i++){
        key_code   = key[i].flag & 0x0F;
        key_status = key[i].flag & 0xF0;
        pressed = key_status & KY_PRESSED_MASK? true: false;
        input_report_key(input_dev, tsp_keycodes[key_code], pressed);
        input_report_key(input_dev, BTN_TOUCH,              pressed);
    }
    input_sync(input_dev);
    return;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static void istcore_put_event(struct ist510e *ts)
{
    if(ts->input_evt_nr > 0)
        push_mt(ts, ts->input_dev, ts->input_evt, ts->input_evt_nr);
    if(ts->input_key_nr > 0)
        push_ky(ts->input_dev, ts->input_key, ts->input_key_nr);
    return;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static s32 istcore_get_event(struct ist510e *ts)
{
    s32 ret;
    u8 i2c_buff[2];
    s32 touch_count;
    uint32_t info_size;
    u8 cmd;

    ret = istcore_i2c_read(ts, TOUCH_COUNT_ADDR, 2, (u8*)&i2c_buff);
    if(ret < 0){
        ISTCORE_ERR("read i2c err\n");
        goto i2c_err;
    }
    ts->input_evt_nr = i2c_buff[0];
    ts->input_key_nr = i2c_buff[1]&0x0f;
    ts->input_lpm_nr = i2c_buff[1]&0xf0;
    ts->input_evt_nr = ts->input_evt_nr>TOUCH_MAX_COUNT?0:ts->input_evt_nr;
    ts->input_key_nr = ts->input_key_nr>KEYS_MAX_COUNT?0:ts->input_key_nr;
    touch_count      = ts->input_evt_nr + ts->input_key_nr;
    ISTCORE_XY("mt = %d, key = %d, lpm = 0x%x, suspend = %d\n",ts->input_evt_nr, ts->input_key_nr,ts->input_lpm_nr, ts->suspended);

	if (ts->suspended && ts->gesture_enable) {
		if (ts->input_lpm_nr == 0x10) {
			input_report_key(ts->input_dev, KEY_WAKEUP, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_WAKEUP, 0);
			input_sync(ts->input_dev);

			ts->suspended = false;
			istcore_reset_ic();
		} else {
			cmd = 1;
			//istcore_reset_ic();
			msleep(50);
			ISTCORE_INFO("retry zombie suspend\n");
			istcore_i2c_write(ts, IST_ENTER_ZOMBIE, 1, &cmd);
		}
		return 0;
	}

    if (ts->input_evt_nr > 0){
        info_size = ts->input_evt_nr *sizeof(struct ist510e_touch_evt);
        ret = istcore_i2c_read(ts, TOUCH_READ_START_ADDR, info_size, (u8*)ts->input_evt);
        if (ret < 0){
            ISTCORE_ERR("read i2c err\n");
            goto i2c_err;
        }
    }
#ifdef IST_KEY
    if (ts->input_key_nr > 0){
        info_size = ts->input_key_nr*sizeof(struct ist510e_touch_key);
        ret = istcore_i2c_read(ts, KEY_READ_START_ADDR  , info_size, (u8*)ts->input_key);
        if (ret < 0){
            ISTCORE_ERR("read i2c err\n");
            goto i2c_err;
        }
    }
#endif
    return touch_count;

i2c_err:
    return -1;
}


#ifdef ISTCORE_IF_DEVICE
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static s32 istcore_get_image(struct ist510e *ts)
{
    s32 ret = 0;
    struct ist510e_debug_dev  *debug_dev = &ts->debug_dev;

    ret = istcore_i2c_read(ts, VR_ADDR_IMAGE, debug_dev->im_size, debug_dev->im_buff);
    if(ret < 0){
        ISTCORE_ERR("read i2c err\n");
        goto i2c_err;
    }
    ISTCORE_INFO("load image from sensor image size is %d\n", debug_dev->im_size);
    return ret;
i2c_err:
    return ret;
}
#endif

/*------------------------------------------------------------------------------
 * kernel task top functions
 *
 *-----------------------------------------------------------------------------*/
static irqreturn_t istcore_irq_task(int irq, void *handle)
{
    struct ist510e *ts = (struct ist510e *) handle;
#ifdef ISTCORE_IF_DEVICE
    struct ist510e_debug_dev  *debug_dev = &ts->debug_dev;
#endif

    if(ts->gesture_enable==false && ts->suspended) {
        ISTCORE_INFO("ist940e suspend and consume input event.\n");
        /* consume the input event, chenyong5 */
        istcore_get_event(ts);
        /* end */
        return IRQ_HANDLED;
    }

    //mutex_lock(&ts->dev_mutex);
#ifdef ISTCORE_IF_DEVICE
    if(debug_dev->im_r_en == 1){
    	ISTCORE_INFO("reading image\n");
        istcore_get_image(ts);
        ts->debug_dev.i_rdy = 1;
        wake_up_interruptible(&ts->debug_dev.i_packet);
        goto istcore_irq_task_finished;
    }
#endif
    if(istcore_get_event(ts))
        istcore_put_event(ts);
istcore_irq_task_finished:
    //mutex_unlock(&ts->dev_mutex);
    return IRQ_HANDLED;
}


/*******************************************************************************
 * porting point
 *******************************************************************************/
static const struct i2c_device_id istcore_dev_idtable[]=
{
    { ISTCORE_I2C_NAME, 0 },
    { }
};

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int istcore_capability(struct ist510e *ts)
{
    int i;

    ISTCORE_INFO("enter\n");
    ts->input_dev->name       = HIDEEP_TS_NAME;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor  = 0x00;//dwz->pannel.vendor ;
    ts->input_dev->id.product = 0x00;//dwz->pannel.product;
    ts->input_dev->id.version = 0x00;//dwz->pannel.version;
#ifdef HIDEEP_LPM
    set_bit(KEY_LPM_WAKEUP,           ts->input_dev->keybit);
#endif //HIDEEP_LPM
    set_bit(EV_ABS,               ts->input_dev->evbit);
    set_bit(EV_KEY,               ts->input_dev->evbit);
    set_bit(EV_SYN,               ts->input_dev->evbit);
    set_bit(BTN_TOUCH,            ts->input_dev->keybit);
    set_bit(MT_TOOL_FINGER,       ts->input_dev->keybit);
    set_bit(INPUT_PROP_DIRECT,    ts->input_dev->propbit);
#ifdef IST510E_KEYBUTTON
    for (i = 0 ; i < ARRAY_SIZE(tsp_keycodes); i++)
        set_bit(tsp_keycodes[i], ts->input_dev->keybit);
#endif //IST510E_KEYBUTTON
#ifdef HIDEEP_TYPE_B_PROTOCOL
    input_mt_init_slots(ts->input_dev, TOUCH_MAX_COUNT, INPUT_MT_DIRECT);
#endif
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,  0, 1599, 0, 0);
    /*  dwz->pannel.dp_h*/
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,  0, 2559, 0, 0);
    /*  ts->pdata->max_z*/
    input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE  ,  0, 32767, 0, 0);
    /*  ts->pdata->max_w*/
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    /*  TOUCH_MAX_COUNT*/
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
    return 0;
}

void istcore_reset_ic(void)
{
    u8 cmd = 1;
    ISTCORE_INFO("enter\n");
		//hideep_irq_enable(false);
    gpio_direction_output(HIDEEP_RESET_GPIO, 1);
    msleep(1);
    gpio_direction_output(HIDEEP_RESET_GPIO, 0);
    msleep(5);
    gpio_direction_output(HIDEEP_RESET_GPIO, 1);
    msleep(450);
    istcore_i2c_write(ts, 0x0800, 1, &cmd);
    msleep(25);
		//hideep_irq_enable(true);
}


/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int istcore_init_hw(struct i2c_client *client)
{
    int ret = 0;
    ISTCORE_INFO("enter\n");
    //gpio_request(HIDEEP_ATTN_GPIO, "hideep_irq");
    //gpio_request(HIDEEP_RESET_GPIO, "hideep_reset");
    //gpio_direction_input(HIDEEP_ATTN_GPIO);
    //gpio_direction_output(HIDEEP_RESET_GPIO, 0);

    //hideep_gpio_setup(HIDEEP_RESET_GPIO, true, 1, 1);
    hideep_gpio_setup(HIDEEP_ATTN_GPIO, true, 0, 0);
    istcore_reset_ic();
    return ret;
}


/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
// add fw update function. oujf1 2015-05-14
#ifdef DO_STARTUP_FW_UPDATE
static void fwu_startup_fw_update_work(struct work_struct *work)
{
    int ret;

    hideep_irq_enable(false);
    mutex_lock(&ts->dev_mutex);

    ts->manually_update = false;
    ISTCORE_INFO("hideep starting fw update...\n");
    ISTCORE_INFO("firmware vr version     : %04x\n", ts->dwz_info.ver_v);
    ISTCORE_INFO("vendor ID     : %02x\n", ts->dwz_info.factory_id);
    if (ts->dwz_info.factory_id == 0x01) {
		ISTCORE_INFO("touch vendor ofilm fw[%s]\n", OFM_IST940E_FW);
		ret = ist_load_ucode(&ts->client->dev, OFM_IST940E_FW, START_FIRMWARE_UPDATE);
	} else if (ts->dwz_info.factory_id == 0x06) {
		ISTCORE_INFO("touch vendor GIS fw[%s]\n", GIS_IST940E_FW);
		ret = ist_load_ucode(&ts->client->dev, GIS_IST940E_FW, START_FIRMWARE_UPDATE);
	}

    if (ret)
        dev_err(&ts->client->dev, "The firmware update failed(%d)\n", ret);

    istcore_reset_ic();

    mutex_unlock(&ts->dev_mutex);
    hideep_irq_enable(true);
    ISTCORE_INFO("hideep done fw update.\n");

    return;
}
#endif

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int istcore_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct istcore_platform_data *pdata = client->dev.platform_data;
    int   ret= 0;

    // saint check for i2c bus
    ISTCORE_INFO("enter\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;
    // init driver's informations
    ts = kmalloc(sizeof(struct ist510e), GFP_KERNEL);
    if(!ts)
    {
        ret = -ENOMEM;
        goto err_free_mem;
    }
    ts->dev_state = ISTCORE_PWR_NORMAL;
    ts->client = client;
    ts->pdata  = pdata;
    ts->gesture_enable = false;
    ts->irq_enable = true;
    ts->client->irq = gpio_to_irq(HIDEEP_ATTN_GPIO);
    ISTCORE_INFO("ts irq: %d\n", ts->client->irq);
    if (ts->pdata->power == NULL)
        ts->power = ts->pdata->power;
    mutex_init(&ts->i2c_mutex);
    mutex_init(&ts->dev_mutex);

    hideep_gpio_setup(HIDEEP_RESET_GPIO, true, 1, 1);
    gpio_direction_output(HIDEEP_RESET_GPIO, 1);
    msleep(5);
#if 1
    hideep_set_pmic_power(true, 20);
#else
    hideep_set_pmic_power_3V3(true, 20);
    msleep(100);
    hideep_set_pmic_power_1V8(true, 20);
    msleep(100);
#endif
	ret = ist_load_dwz(ts);
	if(ret < 0)
	{
		ISTCORE_ERR("fail to sync dwz\n");
		goto err_input_dev_mem;
	}

    // probe & init hardware..
    ret = istcore_init_hw(client);
    if(ret != 0)
    {
        goto err_probe_ic;
    }
    ts->input_dev = input_allocate_device();
    if (!ts->input_dev){
        ISTCORE_ERR("alloc for device.\n");
        ret = -ENOMEM;
        goto err_input_dev_mem;
    }
    istcore_capability(ts);
    ret = input_register_device(ts->input_dev);
    if(ret){
        ISTCORE_ERR("initial device ret= 0x%x\n",ret);
        ret = -ENOMEM;
        goto err_input_dev_reg;
    }
#ifdef ISTCORE_IF_DEVICE
    ISTCORE_INFO("register for iface thread\n");
    istcore_iface_init(ts);
#endif
#if 0
	/* move irq register to the end of probe. chenyong5 */
    ISTCORE_INFO("register for irq = %d thread\n", ts->client->irq);
    if (ts->client->irq){
        ret = request_threaded_irq(ts->client->irq, NULL, istcore_irq_task,
                (IRQF_TRIGGER_LOW | IRQF_ONESHOT),
                /*(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),*/
                ts->client->name,
                ts);

        if (ret<0){
            ISTCORE_ERR("fail to irq ret = 0x%08x\n", ret);
            ret = -EBUSY;
            goto err_irq_req;
        }
    }
    ISTCORE_INFO("ok to irq\n");
#endif
    i2c_set_clientdata(client, ts);
    input_set_drvdata(ts->input_dev, ts);
    ret = istcore_sysfs_init(ts);
    if(ret){
        ISTCORE_ERR("sys file initiation is error.\n");
        goto err_irq_req;
    }
    // add fw update function. oujf1 2015-05-14
#ifdef DO_STARTUP_FW_UPDATE
    ISTCORE_INFO("register for auto-update thread\n");
    ts->fwu_workqueue = create_singlethread_workqueue("fwu_workqueue");
    INIT_DELAYED_WORK(&ts->fwu_work, fwu_startup_fw_update_work);
    queue_delayed_work(ts->fwu_workqueue, &ts->fwu_work, msecs_to_jiffies(FWUPDATE_WORK_DELAY_MS));
#endif
    ISTCORE_INFO("probe success.\n");

#ifdef CONFIG_PM_SLEEP
	ts->suspended = false;
	ts->power_hal_want_suspend = false;
//	mutex_init(&ts->power_hal_lock);
	ret = hideep_power_hal_suspend_init(&client->dev);
	if (ret < 0)
		dev_err(&client->dev, "Unable to register for power hal");
#endif
    ISTCORE_INFO("register for irq = %d thread\n", ts->client->irq);
    if (ts->client->irq){
        ret = request_threaded_irq(ts->client->irq, NULL, istcore_irq_task,
                (IRQF_TRIGGER_LOW | IRQF_ONESHOT),
                /*(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),*/
                ts->client->name,
                ts);

        if (ret<0){
            ISTCORE_ERR("fail to irq ret = 0x%08x\n", ret);
            ret = -EBUSY;
            goto err_irq_req;
        }
    }
    ISTCORE_INFO("ok to irq\n");

    return 0;

err_irq_req      :
    free_irq(client->irq, ts);
err_input_dev_reg:
	input_free_device(ts->input_dev);

err_input_dev_mem:
    kfree(ts);
err_free_mem     :
err_probe_ic     :

    return -1;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
istcore_remove(struct i2c_client *client)
{
    struct ist510e *ts = i2c_get_clientdata(client);

#ifdef DO_STARTUP_FW_UPDATE
    //cancel_work_sync(&ts->fwu_work);
    cancel_delayed_work_sync(&ts->fwu_work);
    flush_workqueue(ts->fwu_workqueue);
    destroy_workqueue(ts->fwu_workqueue);
#endif
#if 1
#ifdef CONFIG_PM_SLEEP
	hideep_power_hal_suspend_destroy(&ts->client->dev);
#endif
#endif
#if 1
    hideep_set_pmic_power(false, 0);
#else
    hideep_set_pmic_power_3V3(false, 0);
    hideep_set_pmic_power_1V8(false, 0);
#endif
    hideep_gpio_setup(HIDEEP_RESET_GPIO, false, 0, 0);
    hideep_gpio_setup(HIDEEP_ATTN_GPIO, false, 0, 0);
    free_irq(client->irq, ts);

    input_unregister_device(ts->input_dev);
    istcore_sysfs_exit(ts);

#ifdef ISTCORE_IF_DEVICE
    istcore_iface_uninit(ts);
#endif

    kfree(ts);

    return 0;
}


//#ifdef CONFIG_PM_SLEEP
static int hideep_suspend(struct device *dev)
{
    u8 cmd = 0;
    struct i2c_client *client = to_i2c_client(dev);
    struct ist510e *ts = i2c_get_clientdata(client);

    ISTCORE_INFO("hideep_suspend start : suspended = %d\n", ts->suspended);

    if(ts->gesture_enable == false && !ts->suspended)
        hideep_irq_enable(false);

    mutex_lock(&ts->dev_mutex);

    if (ts->suspended)
        goto out;

    if(ts->gesture_enable == true) {
    	cmd = 1;
        ISTCORE_INFO("zombie suspend\n");
        istcore_i2c_write(ts, IST_ENTER_ZOMBIE, 1, &cmd);
    } else {
        ISTCORE_INFO("normal suspend\n");
        istcore_i2c_write(ts, IST_ENTER_RIP, 1, &cmd);
    }
out:
    ts->suspended = true;
    mutex_unlock(&ts->dev_mutex);
    ISTCORE_INFO("hideep_suspend  complete : suspended = %d\n", ts->suspended);
    return 0;
}

static int hideep_resume(struct device *dev)
{
    u8 repeat    = 3;
    u8 cmd = 0;
    struct i2c_client *client = to_i2c_client(dev);
    struct hideep_data *data = i2c_get_clientdata(client);
    int ret;

    ISTCORE_INFO("hideep_resume start : suspended = %d\n", ts->suspended);
    mutex_lock(&ts->dev_mutex);

    if (!ts->suspended)
        goto out;

    if(ts->gesture_enable == true) {
        ISTCORE_INFO("zombie resume\n");
    } else {
        ISTCORE_INFO("normal resume\n");
        istcore_i2c_read(ts, IST_ENTER_RIP, 1, &cmd);
        //hideep_irq_enable(true);
    }

out:
    ISTCORE_INFO("out\n");
    ts->suspended = false;
    if(ISTCORE_PWR_PGM != ts->dev_state)
        istcore_reset_ic();
    mutex_unlock(&ts->dev_mutex);

    ISTCORE_INFO("enable irq\n");
    if(ts->gesture_enable == false)
        hideep_irq_enable(true);

    ISTCORE_INFO("hideep_resume  complete : suspended = %d\n", ts->suspended);
    return 0;
}

#if 1
#ifdef CONFIG_PM_SLEEP
static int hideep_power_hal_suspend_init(struct device *dev)
{
	return register_power_hal_suspend_device(dev);
}

static void hideep_power_hal_suspend_destroy(struct device *dev)
{
	unregister_power_hal_suspend_device(dev);
}

void hideep_power_hal_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct ist510e *ts = i2c_get_clientdata(client);

	ISTCORE_INFO("hideep_power_hal_suspend\n");
//	mutex_lock(&ts->power_hal_lock);
	if (ts->power_hal_want_suspend)
		goto out;
	hideep_suspend(dev);
	ts->power_hal_want_suspend = true;
out:
	ISTCORE_INFO("power_hal_want_suspend = true\n");
//	mutex_unlock(&ts->power_hal_lock);
}

void hideep_power_hal_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
    struct ist510e *ts = i2c_get_clientdata(client);

	ISTCORE_INFO("hideep_power_hal_resume\n");
//	mutex_lock(&ts->power_hal_lock);
	if (!ts->power_hal_want_suspend)
		goto out;
	ts->power_hal_want_suspend = false;
	hideep_resume(dev);
out:
	ISTCORE_INFO("power_hal_want_suspend = false\n");
//	mutex_unlock(&ts->power_hal_lock);
}
#endif
#endif

//static SIMPLE_DEV_PM_OPS(hideep_pm_ops, hideep_suspend, hideep_resume);

/*******************************************************************************
 * register device driver..
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#ifdef IST510E_I2C_DEV_REGISTER
static struct i2c_client *g_client;
#endif

static struct of_device_id hideep_match_table[] = {
	{ .compatible = ISTCORE_I2C_NAME,},
	{ },
};

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static struct i2c_driver istcore_driver =
{
    .probe    = istcore_probe,
    .remove   = istcore_remove,
    .id_table = istcore_dev_idtable,
    .driver =
    {
       .owner  = THIS_MODULE,
       .of_match_table = hideep_match_table,
       .name   = ISTCORE_I2C_NAME,
//       .pm     = &hideep_pm_ops,
    },
};


/*------------------------------------------------------------------------------
 * driver register
 *-----------------------------------------------------------------------------*/
static int istcore_init(void)
{
    int ret;

#ifdef IST_I2C_DEV_REGISTER
    struct i2c_adapter *adapter;
    adapter = i2c_get_adapter(IST510e_I2C_BUS);
    g_client = i2c_new_device(adapter, &ist510e_info);
    i2c_put_adapter(adapter);
    if(g_client == NULL)
        return -1;
#endif
    ret = i2c_add_driver(&istcore_driver);
    if(ret != 0){
        ISTCORE_ERR("unable to add i2c driver.\n");
        return -1;
    }
    return ret;
}

/*------------------------------------------------------------------------------
 * driver unregister
 *-----------------------------------------------------------------------------*/
static void istcore_exit(void)
{
    i2c_del_driver(&istcore_driver);
#ifdef IST510E_I2C_DEV_REGISTER
    if(g_client != NULL)
        i2c_unregister_device(g_client);
#endif
    return;
}

module_init(istcore_init);
module_exit(istcore_exit);

MODULE_DESCRIPTION("Driver for HiDeep Touchscreen Controller");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

