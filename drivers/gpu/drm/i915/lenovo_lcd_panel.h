#ifndef __LENOVO_LCD_PANEL_H__
#define __LENOVO_LCD_PANEL_H__

#include <linux/types.h>
#include "intel_dsi.h"

#define LCD_IOCTL_MAGIC 'L'
#define NAME_SIZE 25
#define EFFECT_COUNT 16
#define MODE_COUNT 8

#define LCD_IOCTL_GET_SUPPORTED_EFFECT     _IOW(LCD_IOCTL_MAGIC, 1, struct hal_panel_ctrl_data)
#define LCD_IOCTL_GET_EFFECT_LEVELS        _IOW(LCD_IOCTL_MAGIC, 2, struct hal_panel_ctrl_data)
#define LCD_IOCTL_GET_SUPPORTED_MODE       _IOW(LCD_IOCTL_MAGIC, 3, struct hal_panel_ctrl_data)
#define LCD_IOCTL_SET_EFFECT               _IOW(LCD_IOCTL_MAGIC, 4, struct hal_panel_ctrl_data)
#define LCD_IOCTL_SET_MODE                 _IOW(LCD_IOCTL_MAGIC, 5, struct hal_panel_ctrl_data)
#define LCD_IOCTL_GET_CURRENT_LEVEL        _IOW(LCD_IOCTL_MAGIC, 6, struct hal_panel_ctrl_data)
enum power_status{
	OFF,
	ON,
};
// For hal layer
struct hal_lcd_effect {
	char name[NAME_SIZE];
	int max_level;
	int level;
};

struct hal_lcd_mode {
	char name[NAME_SIZE];
};

struct hal_panel_data {
	struct hal_lcd_effect effect[EFFECT_COUNT];
	struct hal_lcd_mode mode[MODE_COUNT];
	int effect_cnt;
	int mode_cnt;
	int current_mode;
};

struct hal_panel_ctrl_data {
	struct hal_panel_data panel_data;
	int level;
	int mode;
	int index;
};

/*Kernel abstraction layer*/
struct lcd_panel_dev{
	char *name;
	enum power_status  status; //indicating the lcd power state
	struct intel_dsi *dsi;
	int (*get_effect_index_by_name)(char *name);
	int (*get_mode_index_by_name)(char *name);
	int (*get_supported_effect)(struct hal_panel_ctrl_data *);
	int (*get_effect_levels)(struct hal_panel_ctrl_data *);
	int (*get_supported_mode)(struct hal_panel_ctrl_data *);
	int (*get_current_level)(struct hal_panel_ctrl_data *);
	int (*get_mode_status)(struct hal_panel_ctrl_data *);
	int (*set_effect)(struct hal_panel_ctrl_data *, struct intel_dsi *);
	int (*set_mode)(struct hal_panel_ctrl_data *, struct intel_dsi *);
	int (*set_gamma)(int newgamma, struct intel_dsi *);
	struct hal_panel_ctrl_data hal_panel_ctrl;
};

struct lcd_panel{
	struct cdev lcd_panel_cdev;
	struct class *lcd_panel_class;
	struct kobject *lcd_kobj;
	dev_t lcd_panel_devno;
	int count;
	struct lcd_panel_dev *lcd_device;
	u8 attached_id;
	char *name[NAME_SIZE];
	u8 state;
};
struct lcd_cmd{
	u8 cmds[16];
	u8 len;
};

struct lcd_effect_cmd{
	int cmd_nums;
	struct lcd_cmd *lcd_cmds;	
};

struct lcd_mode_cmd{
	int cmd_nums;
	struct lcd_cmd *lcd_cmds;
};

struct lcd_effect{
	char *name;
	int max_level;
	int current_level;
	struct lcd_effect_cmd *lcd_effect_cmds;
};

struct lcd_effect_data{
	struct lcd_effect *lcd_effects;
	int supported_effects;
};

struct lcd_mode{
	char *name;
	struct lcd_mode_cmd *lcd_mode_cmds;
	int mode_status;
};

struct lcd_mode_data{
	struct lcd_mode *lcd_modes;
	int supported_modes;
};

struct lcd_data {
	struct lcd_effect_data *lcd_effects;
	struct lcd_mode_data *lcd_modes;
};

int lenovo_lcd_panel_register(struct lcd_panel_dev * lcd_panel_device);
#endif
