
#include "ov8858_otp.h"
#include <linux/moduleparam.h>

#define I2C_MSG_LENGTH		0x2
#define OV8858_SHORT_MAX			16

//when you want open ov8858_otp debug log,please set debug_level = 3
static unsigned int debug_level = 3;
module_param(debug_level, int, 0644);

#define ov8858_dbg(level, a, ...)\
	do{\
		if(level < debug_level)\
			printk(a,##__VA_ARGS__);\
	}while (0)

int otp_R_gain = 1024;
int otp_B_gain = 1024;
int otp_G_gain = 1024;

enum ov8858_tok_type {
	OV8858_8BIT  = 0x0001,
	OV8858_16BIT = 0x0002
};

static int RG_Ratio_Typical = 306;//read it from 0x7018 & 0x701A
static int BG_Ratio_Typical = 353;//read it from 0x7019 & 0x701A
module_param(RG_Ratio_Typical, int, S_IRUGO|S_IWUSR);
module_param(BG_Ratio_Typical, int, S_IRUGO|S_IWUSR);

static struct i2c_client *client_for_otp = NULL;
static unsigned char ov8858_lsc_array[242] = {0x58,0x00};

static int ov8858_i2c_read(struct i2c_client *client, u16 len, u16 addr,
			   u8 *buf)
{
	struct i2c_msg msg[2];
	u8 address[2];
	int err;

	if (!client->adapter) {
		printk("[OV8858 OTP]--%s error, no adapter\n", __func__);
		return -ENODEV;
	}

	memset(msg, 0, sizeof(msg));
	address[0] = (addr >> 8) & 0xff;
	address[1] = addr & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = address;

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		goto error;
	}

	return 0;
error:
	printk("[OV8858 OTP]--reading from address 0x%x error %d", addr, err);
	return err;
}

static int ov8858_read_reg(struct i2c_client *client, u16 type, u16 reg,
			   u16 *val)
{
	u8 data[OV8858_SHORT_MAX];
	int err;

	/* read only 8 and 16 bit values */
	if (type != OV8858_8BIT && type != OV8858_16BIT) {
		printk("[OV8858 OTP]--%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(data, 0, sizeof(data));

	err = ov8858_i2c_read(client, type, reg, data);
	if (err)
		goto error;

	/* high byte comes first */
	if (type == OV8858_8BIT)
		*val = (u8)data[0];
	else
		*val = data[0] << 8 | data[1];
	ov8858_dbg(3, "[OV8858 OTP]%s:  reg = 0x%04x , val = 0x%04x\n", __func__,reg, *val);

	return 0;

error:
	printk("[OV8858 OTP]--read from offset 0x%x error %d", reg, err);
	return err;
}

static int ov8858_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret == num_msg ? 0 : -EIO;
}

static int
ov8858_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */
		
	if (!client->adapter) {
		printk("[OV8858 OTP]--%s error, no adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != OV8858_8BIT && data_length != OV8858_16BIT) {
		printk("[OV8858 OTP]--%s error, invalid length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);

	if (data_length == OV8858_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* OV8858_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = be16_to_cpu(val);
	}

	ret = ov8858_i2c_write(client, len, data);
	if (ret)
		printk(
			"[OV8858 OTP]--write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}


void OV8858_R2A_write_i2c(u16 addr, int data)
{
	ov8858_write_reg(client_for_otp, OV8858_8BIT, addr, data);
}

void Delay(int times)
{
	msleep(times);
}

int OV8858_R2A_read_i2c(int addr)
{
	int ret;
	u16 val = 0;
	ret = ov8858_read_reg(client_for_otp, OV8858_8BIT, addr, &val);
	if(ret) {
		printk("[OV8858 OTP]--read otp from %d failed\n", addr);
	}

	return val;
}


// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc/invalid otp lenc, 1 valid otp lenc
int read_otp(struct i2c_client *client, struct otp_struct *otp_ptr)
{
	
	int otp_flag, addr, temp, i;
	int  sumtotal=0;
	int checksum1,checksum2,checksum3;
	//set 0x5002[3] to "0"
	int temp1;
	client_for_otp = client;
dev_err(&client->dev, "ov8858 :%s\n", __func__);
	temp1 = OV8858_R2A_read_i2c(0x5002);
	OV8858_R2A_write_i2c(0x5002, (0x00 & 0x08) | (temp1 & (~0x08)));

	// read OTP into buffer
	OV8858_R2A_write_i2c(0x3d84, 0xC0);
	OV8858_R2A_write_i2c(0x3d88, 0x70); // OTP start address
	OV8858_R2A_write_i2c(0x3d89, 0x10);
	OV8858_R2A_write_i2c(0x3d8A, 0x72); // OTP end address
	OV8858_R2A_write_i2c(0x3d8B, 0x0E);
	OV8858_R2A_write_i2c(0x3d81, 0x01); // load otp into buffer
	Delay(10);

	// OTP base information and WB calibration data
	otp_flag = OV8858_R2A_read_i2c(0x7010);
	addr = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x7011; // base address of info group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		//addr = 0x7019; // base address of info group 2
		addr = 0x701B; // Update base address of gropu 2, according to OV8858R2A_OTP_Guide_Ver15
	}
	if(addr != 0) {
		(*otp_ptr).flag = 0xC0; // valid info and AWB in OTP
		(*otp_ptr).module_integrator_id = OV8858_R2A_read_i2c(addr);
		(*otp_ptr).lens_id = OV8858_R2A_read_i2c( addr + 1);
		(*otp_ptr).production_year = OV8858_R2A_read_i2c( addr + 2);
		(*otp_ptr).production_month = OV8858_R2A_read_i2c( addr + 3);
		(*otp_ptr).production_day = OV8858_R2A_read_i2c(addr + 4);
		temp = OV8858_R2A_read_i2c(addr + 9);
		(*otp_ptr).rg_ratio = (OV8858_R2A_read_i2c(addr + 5)<<2) + ((temp>>6) & 0x03);
		(*otp_ptr).bg_ratio = (OV8858_R2A_read_i2c(addr + 6)<<2) + ((temp>>4) & 0x03);
		/*Need hard code RG/BG Typical,not read from the reg*/
//		RG_Ratio_Typical = (OV8858_R2A_read_i2c(addr + 7) << 2) + ((temp >> 2) & 0x03);
//		BG_Ratio_Typical = (OV8858_R2A_read_i2c(addr + 8) << 2) + (temp & 0x03);
	}
	else {
		(*otp_ptr).flag = 0x00; // not info and AWB in OTP
		(*otp_ptr).module_integrator_id = 0;
		(*otp_ptr).lens_id = 0;
		(*otp_ptr).production_year = 0;
		(*otp_ptr).production_month = 0;
		(*otp_ptr).production_day = 0;
		(*otp_ptr).rg_ratio = 0;
		(*otp_ptr).bg_ratio = 0;
	}
	ov8858_dbg(1,"[OV8858 OTP] flag = %d\n", (*otp_ptr).flag);
	ov8858_dbg(1,"[OV8858 OTP] module_integrator_id = %d\n", (*otp_ptr).module_integrator_id);
	ov8858_dbg(1,"[OV8858 OTP] lens_id = %d\n", (*otp_ptr).lens_id);
	ov8858_dbg(1,"[OV8858 OTP] production_year = %d\n", (*otp_ptr).production_year);
	ov8858_dbg(1,"[OV8858 OTP] production_month = %d\n", (*otp_ptr).production_month);
	ov8858_dbg(1,"[OV8858 OTP] production_day  = %d\n", (*otp_ptr).production_day);
	ov8858_dbg(0,"[OV8858 OTP] rg_ratio  = %d\n", (*otp_ptr).rg_ratio);
	ov8858_dbg(0,"[OV8858 OTP] bg_ratio  = %d\n",(*otp_ptr).bg_ratio);
	ov8858_dbg(0,"[OV8858 OTP] hard_code_rg_typical  = %d\n", RG_Ratio_Typical);
	ov8858_dbg(0,"[OV8858 OTP] hard_code_bg_typical  = %d\n", BG_Ratio_Typical);

	// OTP VCM Calibration
	otp_flag = OV8858_R2A_read_i2c(0x7205);	
	addr = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x7206; // base address of VCM Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x7209; // base address of VCM Calibration group 2
	}
	if(addr != 0) {
		(*otp_ptr).flag |= 0x20;
		temp = OV8858_R2A_read_i2c(addr + 2);
		(* otp_ptr).VCM_start = (OV8858_R2A_read_i2c(addr)<<2) | ((temp>>6) & 0x03);
		(* otp_ptr).VCM_end = (OV8858_R2A_read_i2c(addr + 1) << 2) | ((temp>>4) & 0x03);
		(* otp_ptr).VCM_dir = (temp>>2) & 0x03;
		
	}
	else {
		(* otp_ptr).VCM_start = 0;
		(* otp_ptr).VCM_end = 0;
		(* otp_ptr).VCM_dir = 0;
	}
    ov8858_dbg(2,"[OV8858 OTP] VCM_start = %d\n", (* otp_ptr).VCM_start);
	ov8858_dbg(2,"[OV8858 OTP] VCM_end  = %d\n",(* otp_ptr).VCM_end);
	ov8858_dbg(2,"[OV8858 OTP] VCM_dir = %d\n", (* otp_ptr).VCM_dir);

	// OTP Lenc Calibration
	otp_flag = OV8858_R2A_read_i2c(0x7010);
	addr = 0;	
	if((otp_flag & 0x0c) == 0x04) {
		addr = 0x7025; // base address of Lenc Calibration group 1
	}
	else if((otp_flag & 0x03) == 0x01) {
		addr = 0x7115; // base address of Lenc Calibration group 2
	}
	if(addr != 0) {
		for(i=0;i<240;i++) {
			(* otp_ptr).lenc[i]=OV8858_R2A_read_i2c(addr + i); 
            ov8858_lsc_array[i + 2] = (* otp_ptr).lenc[i];
		}
		
		(*otp_ptr).flag |= 0x10;
	}
	else {
		for(i=0;i<240;i++) {
			(* otp_ptr).lenc[i]=0;
		}
	}
	
	//check sum
	checksum1 = OV8858_R2A_read_i2c(0x720c);
	checksum2 = OV8858_R2A_read_i2c(0x720d);
	checksum3 = OV8858_R2A_read_i2c(0x720e);
	for(i = 0x7010; i <= 0x720b ; i++ ){
		sumtotal += OV8858_R2A_read_i2c(i);
	}
	sumtotal = (sumtotal)%255 +1;
	ov8858_dbg(2,"[OV8858 OTP]sumtotal:%d,checksum1:%d,checksum2:%d,checksum3:%d\n",sumtotal,checksum1,checksum2,checksum3);
	if(checksum3 == sumtotal){
		(* otp_ptr).checksum = checksum3;
	}else if((checksum2 = sumtotal)&&(checksum3 == 0)){
		(* otp_ptr).checksum = checksum2;
	}else if((checksum1 = sumtotal)&&(checksum2 == 0)&&(checksum3 == 0)){
		(* otp_ptr).checksum = checksum1;
	}else{
		//error
		printk("[OV8858 OTP]-error---sum is err\n");
		(*otp_ptr).flag &= 0x00;
	}
	
	//clear otp buffer
	for(i=0x7010;i<=0x720e;i++) {
		OV8858_R2A_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
	}

	//set 0x5002[3] to "1"
	temp1 = OV8858_R2A_read_i2c(0x5002);
	OV8858_R2A_write_i2c(0x5002, (0x08 & 0x08) | (temp1 & (~0x08)));

	return (*otp_ptr).flag;
}


// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int apply_otp(struct otp_struct *otp_ptr)
{
    int ret, rg, bg, R_gain, G_gain, B_gain, Base_gain, temp;
    struct i2c_msg lsc_msg;
	pr_err("ov8858 :%s\n", __func__);
	// apply OTP WB Calibration
	if ((*otp_ptr).flag & 0x40) {
		rg = (*otp_ptr).rg_ratio;
		bg = (*otp_ptr).bg_ratio;
		
	ov8858_dbg(1,"[OV8858 OTP] dynamic_debug_rg_typical  = %d\n", RG_Ratio_Typical);
	ov8858_dbg(1,"[OV8858 OTP] dynamic_debug_bg_typical  = %d\n", BG_Ratio_Typical);
		//calculate G gain
		R_gain = (RG_Ratio_Typical*1000) / rg;
		B_gain = (BG_Ratio_Typical*1000) / bg;
		G_gain = 1000;
	
		if (R_gain < 1000 || B_gain < 1000)
		{
	  		 if (R_gain < B_gain)
	    		  	 Base_gain = R_gain;
			  else
	      			 Base_gain = B_gain;
		}
		else
		{
			 Base_gain = G_gain;
		}
	
		R_gain = 0x400 * R_gain / (Base_gain);
		B_gain = 0x400 * B_gain / (Base_gain);
		G_gain = 0x400 * G_gain / (Base_gain);
		ov8858_dbg(0,"[OV8858 OTP]:apply awb otp:R_GAIN=0x%x,B_GAIN=0x%x,G_GAIN=0x%x\n",
                        R_gain,B_gain,G_gain);
		//update R/B/G_gain to set_exppsure for mix calculate with digit gain
		otp_R_gain = R_gain;
		otp_B_gain = B_gain;
		otp_G_gain = G_gain;

		// update sensor WB gain
		if (R_gain>0x400) {
			OV8858_R2A_write_i2c(0x5032, R_gain>>8);
			OV8858_R2A_write_i2c(0x5033, R_gain & 0x00ff);
		}

		if (G_gain>0x400) {
			OV8858_R2A_write_i2c(0x5034, G_gain>>8);
			OV8858_R2A_write_i2c(0x5035, G_gain & 0x00ff);
		}

		if (B_gain>0x400) {
			OV8858_R2A_write_i2c(0x5036, B_gain>>8);
			OV8858_R2A_write_i2c(0x5037, B_gain & 0x00ff);
		}
	}

	// apply OTP Lenc Calibration
	if ((*otp_ptr).flag & 0x10) {
		temp = OV8858_R2A_read_i2c(0x5000);
		temp = 0x80 | temp;
		OV8858_R2A_write_i2c(0x5000, temp);
#if 0
        for(i=0;i<240;i++) {
			OV8858_R2A_write_i2c(0x5800 + i, (*otp_ptr).lenc[i]);
			//printk("zhouyw--i:%d-(*otp_ptr).lenc:%d--\n",i,(*otp_ptr).lenc[i]);
		}
#endif
	    lsc_msg.addr = client_for_otp->addr;
	    lsc_msg.flags = 0;
	    lsc_msg.len = sizeof(ov8858_lsc_array);
	    lsc_msg.buf = ov8858_lsc_array;
	    ret = i2c_transfer(client_for_otp->adapter, &lsc_msg, 1);
	    if (ret != 1) {
            printk("[OV8858 OTP]: lsc otp applied failed\n");
	    } else {
            printk("[OV8858 OTP]: lsc otp applied succeed\n");
	    }
	}

	return (*otp_ptr).flag;
}
