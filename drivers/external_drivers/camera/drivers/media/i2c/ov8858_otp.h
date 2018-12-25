#ifndef _OV8858_OTP_H_
#define _OV8858_OTP_H_

//#define CAMERA_OTP_DEBUG
#ifdef CAMERA_OTP_DEBUG
#define OTP_LOG(fmt, args...) printk(fmt, ##args)
#else
#define OTP_LOG(fmt, args...) do { } while (0)
#endif

#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <linux/acpi.h>
#ifdef CONFIG_GMIN_INTEL_MID
#include <linux/atomisp_gmin_platform.h>
#else
#include <media/v4l2-chip-ident.h>
#endif


//#define DATA_BUF_SIZE 1024



struct otp_struct {
	int flag; // bit[7]: info, bit[6]:wb, bit[5]:vcm, bit[4]:lenc
	int module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int lenc[240];
	int checksum;
	int VCM_start;
	int VCM_end;
	int VCM_dir;
};


extern int read_otp(struct i2c_client *client, struct otp_struct *otp_ptr);
extern int apply_otp(struct otp_struct *otp_ptr);





#endif
