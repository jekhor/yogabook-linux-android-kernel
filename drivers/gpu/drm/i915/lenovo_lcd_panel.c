#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <drm/drmP.h>

#include "lenovo_lcd_panel.h"

#include <linux/gpio.h>
//#include "linux/mfd/intel_mid_pmic.h"
#include <linux/mfd/intel_soc_pmic.h>

#define NAME_SIZE 25

static struct lcd_panel lenovo_lcd_panel;
//struct mutex lcd_mutex;
#if 1
#define GPIO1P0 376
#define LCD_ID GPIO1P0
#define PMIC_GPIO1P0_BASE_ADDRESS 0x3b
#define PMIC_GPIO1P0_OFFSET 0x0
#define PMIC_GPIO1P0_ADDRESS PMIC_GPIO1P0_BASE_ADDRESS+PMIC_GPIO1P0_OFFSET
#define GPIO_INPUT_NO_DRV 0x0
#endif
ssize_t lenovo_lcd_get_cabc(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = -1;
	int index = 0;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct hal_panel_ctrl_data ctrl;

	DRM_DEBUG_DRIVER("[LCD]: %s: line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL)
		return ret;

	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("cabc");
		if(index < 0){
			DRM_ERROR("[LCD]: %s: Not support cabc function\n",__func__);
			return ret;
			}
	}

	ctrl.index = index;

	 lcd_panel->hal_panel_ctrl.level = ctrl.level;
	  lcd_panel->hal_panel_ctrl.index = ctrl.index;

	if(lcd_panel->get_current_level)
		ret = lcd_panel->get_current_level(&ctrl);


	sprintf(buf, "%d\n", ctrl.panel_data.effect[index].level);

	ret = strlen(buf)+1;

	return ret;
}

ssize_t lenovo_lcd_set_cabc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int index = 0, ret = -1;
	long val;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct intel_dsi *dsi = lcd_panel->dsi;
	struct hal_panel_ctrl_data ctrl;

    	printk("[LCD]: %s: %s, status:%d, ON=%d\n", __func__, buf, lcd_panel->status, ON);
	if(lcd_panel == NULL || lcd_panel->status != ON)
		return ret;

	if(strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;

	ctrl.level = val;
	
	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("cabc");
		if(index < 0 ){
			DRM_ERROR("[LCD]: %s: Not support cabc function\n",__func__);
			return ret;
			}
	}

	ctrl.index = index;
	 lcd_panel->hal_panel_ctrl.level = ctrl.level;
	  lcd_panel->hal_panel_ctrl.index = ctrl.index;

	DRM_DEBUG_DRIVER("[LCD]: %s: line=%d ctrl.level=%d\n",__func__,__LINE__,ctrl.level);
	if(lcd_panel->set_effect(&ctrl, dsi)!= 0){
		DRM_ERROR("[LCD]: errored from set effect\n");
		return ret;
	}

	return count;
}

ssize_t lenovo_lcd_get_cabc_and_ce(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = -1;
	int index = 1;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct hal_panel_ctrl_data ctrl;

	DRM_DEBUG_DRIVER("[LCD]: %s: line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL)
		return ret;

	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("cabc_and_ce");
		if(index < 0){
			DRM_ERROR("[LCD]: %s: Not support cabc_and_ce function\n",__func__);
			return ret;
			}
	}

	ctrl.index = index;
	 lcd_panel->hal_panel_ctrl.level = ctrl.level;
	  lcd_panel->hal_panel_ctrl.index = ctrl.index;

	if(lcd_panel->get_current_level)
		ret = lcd_panel->get_current_level(&ctrl);


	sprintf(buf, "level %d --%s %s\n", ctrl.panel_data.effect[index].level,
		ctrl.panel_data.effect[index].level&0x01 ? "cabc on":"cabc off",
		ctrl.panel_data.effect[index].level&0x02 ? "ce on":"ce off");

	ret = strlen(buf)+1;

	return ret;
}

ssize_t lenovo_lcd_set_cabc_and_ce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int index = 0, ret = -1;
	long val;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct intel_dsi *dsi = lcd_panel->dsi;
	struct hal_panel_ctrl_data ctrl;

    	printk("[LCD]: %s: %s, status:%d, ON=%d\n", __func__, buf, lcd_panel->status, ON);
	if(lcd_panel == NULL || lcd_panel->status != ON)
		return ret;

	if(strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;

	ctrl.level = val;
	
	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("cabc_and_ce");
		if(index < 0 ){
			DRM_ERROR("[LCD]: %s: Not support cabc_and_ce  function\n",__func__);
			return ret;
			}
	}

	ctrl.index = index;
	 lcd_panel->hal_panel_ctrl.level = ctrl.level;
	  lcd_panel->hal_panel_ctrl.index = ctrl.index;

	DRM_DEBUG_DRIVER("[LCD]: %s: line=%d ctrl.level=%d\n",__func__,__LINE__,ctrl.level);
	if(lcd_panel->set_effect(&ctrl, dsi)!= 0){
		DRM_ERROR("[LCD]: errored from set effect\n");
		return ret;
	}

	return count;
}


ssize_t lenovo_lcd_get_ce(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = -1;
	int index = 1;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct hal_panel_ctrl_data ctrl;

	DRM_DEBUG_DRIVER("[LCD]: %s:line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL || lcd_panel->status != ON)
		return ret;

	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("ce");
		if(index < 0){
			DRM_ERROR("[LCD]: %s: Not support ce function\n",__func__);
			return ret;
		}
	}

	ctrl.index = index;
	 lcd_panel->hal_panel_ctrl.level = ctrl.level;
	  lcd_panel->hal_panel_ctrl.index = ctrl.index;

	if(lcd_panel->get_current_level)
		ret = lcd_panel->get_current_level(&ctrl);

	sprintf(buf, "%d\n", ctrl.panel_data.effect[index].level);


	ret = strlen(buf)+1;

	DRM_DEBUG_DRIVER("[LCD]: %s:buf=%s  ret=%d  line=%d\n",__func__,buf,ret,__LINE__);
	return ret;
}

ssize_t lenovo_lcd_set_ce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int index = 0, ret = -1;
	long val;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct intel_dsi *dsi = lcd_panel->dsi;
	struct hal_panel_ctrl_data ctrl;

    	printk("[LCD]: %s: %s, status:%d, ON=%d\n", __func__, buf, lcd_panel->status, ON);
	if(lcd_panel == NULL || lcd_panel->status != ON)
		return ret;


	if(strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;

	ctrl.level = val;

	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("ce");
		if(index < 0){
			DRM_ERROR("[LCD]: %s: Not support ce function\n",__func__);
			return ret;
		}
	}

	ctrl.index = index;
	 lcd_panel->hal_panel_ctrl.level = ctrl.level;
	  lcd_panel->hal_panel_ctrl.index = ctrl.index;

	if(lcd_panel->set_effect(&ctrl, dsi)!= 0)
		DRM_ERROR("[LCD]: errored from set effect\n");

	DRM_DEBUG_DRIVER("[LCD]: %s:line=%d ctrl.level=%d\n",__func__,__LINE__,ctrl.level);
	return count;
}
int lcd_gamma_index = 0;
ssize_t lenovo_lcd_get_gamma(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = -1;
	
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	

	DRM_DEBUG_DRIVER("[LCD]: %s:line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL || lcd_panel->status != ON)
		return ret;

	
	sprintf(buf, "%d\n", lcd_gamma_index);


	ret = strlen(buf)+1;

	DRM_DEBUG_DRIVER("[LCD]: %s:buf=%s  ret=%d  line=%d\n",__func__,buf,ret,__LINE__);
	return ret;
}

ssize_t lenovo_lcd_set_gamma(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = -1;
	long val;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct intel_dsi *dsi = lcd_panel->dsi;
	

    	DRM_DEBUG_DRIVER("[LCD]: %s: %s\n",__func__, buf);
	if(lcd_panel == NULL || lcd_panel->status != ON)
		return ret;


	if(strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;

	lcd_gamma_index = val;
       lcd_panel->set_gamma(lcd_gamma_index,dsi);
	//intel_dsi_nt35523_get_gamma(lcd_gamma_index);	

	DRM_DEBUG_DRIVER("[LCD]: %s:line=%d lcd_gamma_index=%d\n",__func__,__LINE__,lcd_gamma_index);
	return count;
}


static int lcd_id = -1;
 int chv_get_lcd_id(void)
{
    //int ret=0;
        //if(lcd_id==-1)
        {  
#if 0        
        ret = gpio_request(LCD_ID, "lcd_id");
        if (ret < 0)
                 printk("[lcd_id] request fail\n");
        ret = gpio_direction_input(LCD_ID);
        if(ret< 0)
                printk("[lcd_id]: Failed to config gpio lcd_id\n");
#endif
         //set gpio1p2 function state
        intel_soc_pmic_writeb(0x3b, 0);
        intel_soc_pmic_setb(0x3b, 1);
         msleep(5);
        //lcd_id = gpio_get_value_cansleep(LCD_ID);
        //printk("[lcd_id]:%s,gpio1P0 value: 0x%x\n",__func__,lcd_id);
        lcd_id = intel_soc_pmic_readb(0x43);
        //printk("[lcd_id]:%s,reg-gpio1P0 value: 0x%x\n",__func__,lcd_id);
        lcd_id =lcd_id&0x1;  
      }
      return lcd_id;
}
ssize_t lenovo_lcd_get_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
#if 0
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;

    /*printk("[LCD]: %s: ==jinjt==line=%d\n",__func__,__LINE__);*/
	if(lcd_panel == NULL){
		printk("[LCD]: %s: Not have registed lcd panel\n",__func__);
		return ret;
	}

	sprintf(buf, "%s\n", lcd_panel->name);
#else
#if 0
	int lcd_id = 0 ;
	ret = gpio_request(LCD_ID, "lcd_id");
        if (ret < 0)
                 printk("[lcd_id] request fail\n");
        ret = gpio_direction_input(LCD_ID);
        if(ret< 0)
        	printk("[lcd_id]: Failed to config gpio lcd_id\n");
 
         //set gpio1p2 function state
        intel_soc_pmic_writeb(0x3b, 0);
        // msleep(5);
        lcd_id = gpio_get_value_cansleep(LCD_ID);
        printk("[lcd_id]:%s,gpio1P0 value: 0x%x\n",__func__,lcd_id);
        lcd_id = intel_soc_pmic_readb(0x43);
        printk("[lcd_id]:%s,reg-gpio1P0 value: 0x%x\n",__func__,lcd_id);

#endif       
     lcd_id= chv_get_lcd_id();
    if((lcd_id&1)==0)
    	{
		sprintf(buf, "%s\n", "AUO_1920X1200_panel" );
    	}
	else
	{
		sprintf(buf, "%s\n", "INL_1920X1200_panel" );
	}

	//sprintf(buf, "lcd_type: %d\n", lcd_id);
#endif
	ret = strlen(buf) + 1;

    /*printk("[LCD]: %s: ==jinjt==name=%s   line=%d\n",__func__,buf,__LINE__);*/
	return ret;
}
#if 0
ssize_t lenovo_lcd_get_dpst(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int index = 1;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct hal_panel_ctrl_data ctrl;

	DRM_DEBUG_DRIVER("[LCD]: %s: line=%d\n",__func__,__LINE__);
	if(lcd_panel == NULL)
		return ret;

	if(lcd_panel->get_effect_index_by_name){
		index = lcd_panel->get_effect_index_by_name("cabc");
		if(index < 0){
			DRM_ERROR("[LCD]: %s: Not support cabc function\n",__func__);
			return ret;
			}
	}

	ctrl.index = index;
	 lcd_panel->hal_panel_ctrl.level = ctrl.level;
	  lcd_panel->hal_panel_ctrl.index = ctrl.index;

	if(lcd_panel->get_current_level)
		ret = lcd_panel->get_current_level(&ctrl);


	sprintf(buf, "%d\n", ctrl.panel_data.effect[index].level);

	ret = strlen(buf)+1;

	return ret;

}

ssize_t lenovo_lcd_set_dpst(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	long val;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct hal_panel_ctrl_data ctrl;
	int index = 0;
	struct intel_dsi *dsi = lcd_panel->dsi;
	struct drm_device *drmdev = dsi->base.base.dev;
	struct drm_i915_private *dev_priv = drmdev->dev_private;
	struct intel_panel *panel = &dev_priv->dpst.connector->panel;
	 int panelid = dsi->dev.sub_panel_id;


	if(strict_strtol(buf, 10, &val) < 0)
		return -EINVAL;


	if(panelid == MIPI_DSI_NT35523_PANEL_ID) {
		if(lcd_panel == NULL ) {
			DRM_ERROR("[LCD]: %s:lcd_panel is NULL\n",__func__);
			return ret;
		}
		if(!panel->backlight.enabled || lcd_panel-> status !=  ON){
			DRM_ERROR("[LCD]: %s lcd already powered down  %d   %d\n", __func__, panel->backlight.enabled, lcd_panel-> status);
			return ret;
		}

		if(lcd_panel->get_effect_index_by_name) {
			index = lcd_panel->get_effect_index_by_name("cabc");
			// 	 index = lcd_panel->get_effect_index_by_name("suspend");
			if(index < 0 ) {
				DRM_ERROR("[LCD]: %s: Not support cabc function\n",__func__);
				return ret;
			}
		}

		ctrl.index = index;
		 lcd_panel->hal_panel_ctrl.level = ctrl.level;
	 	 lcd_panel->hal_panel_ctrl.index = ctrl.index;
		if (val == 1) {
			ctrl.level = 0;//cabc off
			if(lcd_panel->set_effect(&ctrl, dsi)!= 0){
				DRM_ERROR("[LCD]: errored from set effect\n");
				return ret;
			}
			DRM_DEBUG_DRIVER("cabc off\n");
			/*i915_dpst_switch(true);*/
		}
		else
		{
			/*i915_dpst_switch(false);*/
			ctrl.level = 1;//cabc on
			if(lcd_panel->set_effect(&ctrl, dsi)!= 0){
				DRM_ERROR("[LCD]: errored from set effect\n");
				return ret;
			}
				DRM_DEBUG_DRIVER("cabc on\n");
		}
			ret = count;
	}
	return ret;

}

extern int isl9860_ext_read_byte(u8 reg);
ssize_t lenovo_ISL9860_get_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

        ret=isl9860_ext_read_byte(0x8);

	if(ret <0)
	{
	   sprintf(buf, "%s\n", "unknow lcd bios ic" );
	}	
	else
	{
		 sprintf(buf, "%s\n", "ISL98608" );
	} 

	ret = strlen(buf) + 1;

  	return ret;
}
extern int read_backlight_id(void);

ssize_t lenovo_LP8557_get_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;

        ret=read_backlight_id();

	if(ret <0)
	{
	   sprintf(buf, "%s\n", "unknow lcd backligt ic" );
	}	
	else
	{
		 sprintf(buf, "%s\n", "LP85557" );
	} 

	ret = strlen(buf) + 1;

  	return ret;
}
#endif
int lenovo_lcd_panel_register(struct lcd_panel_dev *lcd_panel_device)
{
	int ret = 0;

	if(lcd_panel_device->name)
		DRM_INFO("[LCD]:@@@@@@%s: @@@@@@%s \n",__func__, lcd_panel_device->name);
	if(lcd_panel_device == NULL)
	{
		DRM_ERROR("[LCD]:%s, invalid parameter \n",__func__);
		return ret;
	}

	lenovo_lcd_panel.lcd_device = lcd_panel_device;

	return ret;

}

#if 1
static DEVICE_ATTR(cabc_onoff, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_cabc, lenovo_lcd_set_cabc);
static DEVICE_ATTR(cabc_and_ce, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_cabc_and_ce, lenovo_lcd_set_cabc_and_ce);
static DEVICE_ATTR(ce_onoff, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_ce, lenovo_lcd_set_ce);
static DEVICE_ATTR(lcd_name, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_name, NULL);
static DEVICE_ATTR(lcd_gamma, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_gamma, lenovo_lcd_set_gamma);
#if 0
static DEVICE_ATTR(dpst_onoff, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_lcd_get_dpst, lenovo_lcd_set_dpst);

static DEVICE_ATTR(ISL9860_name, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_ISL9860_get_name, NULL);
static DEVICE_ATTR(LP8557_name, S_IRUGO | S_IWUSR | S_IWGRP, lenovo_LP8557_get_name, NULL);
#endif
static struct attribute *lenovo_lcd_attrs[] = {
	&dev_attr_cabc_onoff.attr,
	&dev_attr_cabc_and_ce.attr,
	&dev_attr_ce_onoff.attr,
	&dev_attr_lcd_name.attr,
#if 0
	&dev_attr_dpst_onoff.attr,

	&dev_attr_ISL9860_name.attr,
	&dev_attr_LP8557_name.attr,
#endif
	&dev_attr_lcd_gamma.attr,
	NULL,
};

static struct attribute_group lenovo_lcd_attr_group = {
	.attrs = lenovo_lcd_attrs,
};

static int lenovo_lcd_panel_open(struct inode *inode, struct file *filp)
{
	int ret = -1;
	struct lcd_panel_dev *lcd_panel =  lenovo_lcd_panel.lcd_device;

    /*printk("[LCD]: %s:line=%d\n",__func__,__LINE__);*/
	if(lcd_panel == NULL)
		return ret;

	if(lcd_panel->status != ON)
	{
		DRM_ERROR("[LCD]: panel have powered down\n");
		//return ret;
	}

	return 0;
}

static long lenovo_lcd_panel_ioctl(struct file *filp, unsigned int cmd, unsigned long argp)
{
	int ret = 0;
	struct lcd_panel_dev *lcd_panel = lenovo_lcd_panel.lcd_device;
	struct intel_dsi *dsi = lcd_panel->dsi;
	struct hal_panel_ctrl_data *hal_panel_data = (struct hal_panel_ctrl_data *)argp;
	 lcd_panel->hal_panel_ctrl = *hal_panel_data;
	//struct drm_device *dev = dsi->base.base.dev;
	//struct drm_i915_private *dev_priv = dev->dev_private;
	//struct intel_panel *panel = &dev_priv->dpst.connector->panel;

/*printk("[LCD]: %s:line=%d LCD_IOCTL_GET_SUPPORTED_EFFECT=%d\n",__func__,__LINE__,LCD_IOCTL_GET_SUPPORTED_EFFECT);*/
/*printk("[LCD]: %s:line=%d LCD_IOCTL_GET_EFFECT_LEVELS=%d\n",__func__,__LINE__,LCD_IOCTL_GET_EFFECT_LEVELS);*/
/*printk("[LCD]: %s:line=%d LCD_IOCTL_GET_SUPPORTED_MODE=%d\n",__func__,__LINE__,LCD_IOCTL_GET_SUPPORTED_MODE);*/
/*printk("[LCD]: %s:line=%d LCD_IOCTL_SET_EFFECT=%d\n",__func__,__LINE__,LCD_IOCTL_SET_EFFECT);*/
/*printk("[LCD]: %s:line=%d LCD_IOCTL_SET_MODE=%d\n",__func__,__LINE__,LCD_IOCTL_SET_MODE);*/
/*printk("[LCD]: %s:line=%d LCD_IOCTL_GET_CURRENT_LEVEL=%d\n",__func__,__LINE__,LCD_IOCTL_GET_CURRENT_LEVEL);*/
/*#define LCD_IOCTL_GET_SUPPORTED_EFFECT     _IOW(LCD_IOCTL_MAGIC, 1, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_GET_EFFECT_LEVELS        _IOW(LCD_IOCTL_MAGIC, 2, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_GET_SUPPORTED_MODE       _IOW(LCD_IOCTL_MAGIC, 3, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_SET_EFFECT               _IOW(LCD_IOCTL_MAGIC, 4, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_SET_MODE                 _IOW(LCD_IOCTL_MAGIC, 5, struct hal_panel_ctrl_data)*/
/*#define LCD_IOCTL_GET_CURRENT_LEVEL        _IOW(LCD_IOCTL_MAGIC, 6, struct hal_panel_ctrl_data)*/
#if 1
	        if(lcd_panel == NULL || lcd_panel->status != ON){
		return ret;
	}
#endif
	switch (cmd){
		case LCD_IOCTL_GET_SUPPORTED_EFFECT:
			DRM_DEBUG_DRIVER("[LCD]: %s:LCD_IOCTL_GET_SUPPORTED_EFFECT=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->get_supported_effect != NULL)
			{
				ret = lcd_panel->get_supported_effect(hal_panel_data);
				if (ret < 0 ){
					DRM_ERROR("[lcd]: Error from get_supported_effect\n");
					return ret;
				}
			}else{
				DRM_ERROR("[LCD]: get_supported_effect have not implemented\n");
				return ret;
			}
			break;
		case LCD_IOCTL_GET_EFFECT_LEVELS:
			DRM_DEBUG_DRIVER("[LCD]: %s: LCD_IOCTL_GET_EFFECT_LEVELS=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->get_effect_levels != NULL)
			{
				ret = lcd_panel->get_effect_levels(hal_panel_data);
				if(ret < 0){
					DRM_ERROR("[LCD]: Error from get_effect_levels\n");
					return ret;
				}
			}else{
				DRM_ERROR("[LCD]: get_effect_levels have not implemented\n");
				return ret;
			}
			break;
#if 1 
        	case LCD_IOCTL_GET_CURRENT_LEVEL:
			DRM_DEBUG_DRIVER("[LCD]: %s: LCD_IOCTL_GET_CURRENT_LEVEL=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->get_current_level != NULL)
			{
				ret = lcd_panel->get_current_level(hal_panel_data);
				if(ret < 0){
					DRM_ERROR("[LCD]: Error from get_supported_level\n");
					return ret;
				}
			}else{
				DRM_ERROR("[LCD]: get_supported_level have not implemented\n");
				return ret;
			}
			break;
#endif
		case LCD_IOCTL_GET_SUPPORTED_MODE:
			DRM_DEBUG_DRIVER("[LCD]: %s: LCD_IOCTL_GET_SUPPORTED_MODE=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->get_supported_mode != NULL)
			{
				ret = lcd_panel->get_supported_mode(hal_panel_data);
				if(ret < 0){
					DRM_ERROR("[LCD]:Error from get_supported_mode\n");
					return ret;
				}
			}else{
				DRM_ERROR("[LCD]: get_supported_mode have not implemented\n");
				return ret;
			}
			break;
		case LCD_IOCTL_SET_EFFECT:
			DRM_INFO("[LCD]: %s: LCD_IOCTL_SET_EFFECT=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			 lcd_panel->hal_panel_ctrl.level = hal_panel_data->level;
	 		 lcd_panel->hal_panel_ctrl.index = hal_panel_data->index;
			if(lcd_panel->set_effect != NULL)
			{
				ret = lcd_panel->set_effect(hal_panel_data, dsi);
				if(ret < 0){
					DRM_ERROR("[LCD]:Error from set_effect\n");
					return ret;
				}
			}else{
				DRM_ERROR("[LCD]: set_effect have not implemented\n");
				return ret;
			}
			break;
		case LCD_IOCTL_SET_MODE:
			DRM_DEBUG_DRIVER("[LCD]: %s: LCD_IOCTL_SET_MODE=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			if(lcd_panel->set_mode!= NULL)
			{
				 lcd_panel->hal_panel_ctrl.level = hal_panel_data->level;
	 			 lcd_panel->hal_panel_ctrl.index = hal_panel_data->index;
				ret = lcd_panel->set_mode(hal_panel_data, dsi);
				if(ret < 0){
					DRM_ERROR("[LCD]: Error from set_mode\n");
					return ret;
				}
			}else{
				DRM_ERROR("[LCD]:set_mode have not implemented\n");
				return ret;
			}
			break;
		default: 
			DRM_ERROR("[LCD]: %s: unknow default=cmd=%d  line=%d\n",__func__,cmd,__LINE__);
			break;
	}

	return ret;
}

struct file_operations lcd_panel_fops = {
	.owner = THIS_MODULE,
	.open = lenovo_lcd_panel_open,
	.unlocked_ioctl = lenovo_lcd_panel_ioctl,
        // .compat_ioctl = lenovo_lcd_panel_ioctl,
};

static int lenovo_lcd_panel_dev_init(struct cdev *cdev, dev_t *devno)
{
	int ret = 0;
	dev_t dev_no; 

	ret = alloc_chrdev_region(devno, 0 , 1, "lenovo_lcd_panel");
	if(ret < 0){
		DRM_ERROR("[LCD]: Failed to alloc chrdev no. \n");
		return ret;
	}
	
	dev_no = *devno;
	cdev_init(cdev, &lcd_panel_fops);
	cdev->owner = THIS_MODULE;
	cdev_add(cdev, dev_no, 1);

	return ret;
}

static int __init lcd_panel_init(void)
{
	int ret = 0;
	struct device *dev;
	struct cdev *lcd_cdev;
	dev_t *lcd_devno;
//	struct kobject *lcd_kobject;

	lcd_cdev  = &lenovo_lcd_panel.lcd_panel_cdev;
	lcd_devno = &lenovo_lcd_panel.lcd_panel_devno;
//    mutex_init(&lcd_mutex);


	ret = lenovo_lcd_panel_dev_init(lcd_cdev, lcd_devno);
	if(ret < 0)
		return ret;

	/*create sys class for lcd panel*/
	lenovo_lcd_panel.lcd_panel_class = class_create(THIS_MODULE, "lcd_class");
	if(IS_ERR(lenovo_lcd_panel.lcd_panel_class))
	{
		DRM_ERROR("[LCD]: failed to create lcd panel class");
		return -1;
		}

	/*create dev file */
	dev = device_create(lenovo_lcd_panel.lcd_panel_class, NULL, *lcd_devno,NULL,"lcd%d",0);

	ret = sysfs_create_group(&dev->kobj, &lenovo_lcd_attr_group);
	if(ret){
		DRM_ERROR("[LCD]: failed to create group\n");
		return ret;
	}

/*
	//add kobject to sys filesystem
	lcd_kobject = kobject_create_and_add("lcd_panel",NULL);
		if(ret){
			DRM_ERROR("[LCD]: failed to add kobject\n");
			return ret;
		}else
			lenovo_lcd_panel.lcd_kobj = lcd_kobject;

	ret = sysfs_create_group(lcd_kobject, &lenovo_lcd_attr_group);
	if(ret){
		DRM_ERROR("[LCD]: failed to create group\n");
		kobject_put(lcd_kobject);
		return ret;
	}
*/
	return ret;
} 

static void __exit lcd_panel_exit(void)
{

	struct cdev *lcd_cdev = &lenovo_lcd_panel.lcd_panel_cdev;
	struct class *lcd_class = lenovo_lcd_panel.lcd_panel_class;
	dev_t lcd_devno =  lenovo_lcd_panel.lcd_panel_devno;
//	struct kobject *lcd_kobject = lenovo_lcd_panel.lcd_kobj;

	cdev_del(lcd_cdev);

	device_destroy(lcd_class, lcd_devno); //delete the dev node under /dev

	class_destroy(lcd_class);

	unregister_chrdev_region(lcd_devno, 1);

//	kobject_put(lcd_kobject);

	return;

}

module_init(lcd_panel_init);
module_exit(lcd_panel_exit);
#endif
