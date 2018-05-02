#ifndef _LINUX_MG_I2C_MTOUCH_H
#define _LINUX_MG_I2C_MTOUCH_H

#define CAP_X_CORD 			2368
#define CAP_Y_CORD 			1344

#define CAP_MAX_X			CAP_X_CORD-1
#define CAP_MAX_Y			CAP_Y_CORD-1

#define DIG_MAX_X			0x29FF
#define DIG_MAX_Y			0x43FF
#define DIG_MAX_P			0x07ff

#define REPORT_MAX_X		0x29FF//0x37ff	//
#define REPORT_MAX_Y		0x43FF//0x1fff	//

#define RD_COMMAND_BIT			8

#define COMMAND_COUSE				7
#define COMMAND_BYTE				5
#define FIRMWARE_UPDATE_LENGTH	13
#define ACK_BIT						5

static u_int8_t command_list[COMMAND_COUSE][COMMAND_BYTE] = 
	{
		{0x01, 0x02, 0x04 , 0x03 , 0x10 },	// 0 Turn to Bootloader
		{0x01, 0x02, 0x04 , 0x03 , 0x00 },	// 1 Turn to App
		{0x01, 0x02, 0x06 , 0x03 , 0x02 },	// 2 read firmware version
		{0x01, 0x02, 0x05 , 0x03 , 0x01 },	// 3 read product ID
		{0x01, 0x02, 0x01 , 0xA1 , 0xA0 },	// 4 write flash
		{0xDE, 0x55, 0x00 , 0x00 , 0x00 },	// 5 sleep
		{0xDE, 0x5A, 0x00 , 0x00 , 0x00 },  	// 6 wake up
	};

enum mg_capac_report
	{
		MG_CAP_MODE = 0x0,
		TOUCH_KEY_CODE,
		ACTUAL_TOUCH_POINTS,
		
		MG_CONTACT_ID,
		MG_CAP_STATUS,
		MG_POS_X_LOW,
		MG_POS_X_HI,
		MG_POS_Y_LOW,
		MG_POS_Y_HI,

		MG_CONTACT_ID2,
		MG_STATUS2,
		MG_POS_X_LOW2,
		MG_POS_X_HI2,
		MG_POS_Y_LOW2,
		MG_POS_Y_HI2,
		
		MG_CONTACT_IDS3,
		MG_POS_X_LOW3,
		MG_POS_X_HI3,
		MG_POS_Y_LOW3,
		MG_POS_Y_HI3,	
		
		MG_CONTACT_IDS4,
		MG_POS_X_LOW4,
		MG_POS_X_HI4,
		MG_POS_Y_LOW4,
		MG_POS_Y_HI4,	

		MG_CONTACT_IDS5,
		MG_POS_X_LOW5,
		MG_POS_X_HI5,
		MG_POS_Y_LOW5,
		MG_POS_Y_HI5,
		
		MG_CONTACT_IDS6,
		MG_POS_X_LOW6,
		MG_POS_X_HI6,
		MG_POS_Y_LOW6,
		MG_POS_Y_HI6,	
		
		MG_CONTACT_IDS7,
		MG_POS_X_LOW7,
		MG_POS_X_HI7,
		MG_POS_Y_LOW7,
		MG_POS_Y_HI7,	
		
		MG_CONTACT_IDS8,
		MG_POS_X_LOW8,
		MG_POS_X_HI8,
		MG_POS_Y_LOW8,
		MG_POS_Y_HI8,	
		
		MG_CONTACT_IDS9,
		MG_POS_X_LOW9,
		MG_POS_X_HI9,
		MG_POS_Y_LOW9,
		MG_POS_Y_HI9,	
		
		MG_CONTACT_IDS10,
		MG_POS_X_LOW10,
		MG_POS_X_HI10,
		MG_POS_Y_LOW10,
		MG_POS_Y_HI10,	
	};

enum mg_dig_report 
	{
		MG_DIG_MODE = 0x0,
		MG_DIG_STATUS,
		MG_DIG_X_LOW,
		MG_DIG_X_HI,
		MG_DIG_Y_LOW,
		MG_DIG_Y_HI,
		/* Z represents pressure value */
		MG_DIG_Z_LOW,
		MG_DIG_Z_HI,
	};

enum mg_dig_state 
	{
		MG_OUT_RANG = 0,
		MG_IN_RANG = 0x10,
		MG_TIP_SWITCH = 0x11,
		MG_CUSTOM_BTN = 0x12,
		MG_RIGHT_BTN = 0x13,
	};

enum mg_int_mode 
	{
		MG_INT_PERIOD = 0x0,
		MG_INT_FMOV,
		MG_INT_FTOUCH,
	};

enum mg_int_trig 
	{
		MG_INT_TRIG_LOW = 0x0,
		MG_INT_TRIG_HI,
	};

struct mg_platform_data 
	{
	/* add platform dependent data here */
	};

#endif 	/* _LINUX_MG_I2C_MTOUCH_H */

