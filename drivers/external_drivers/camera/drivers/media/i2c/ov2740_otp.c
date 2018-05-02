
#include "ov2740_otp.h"

#define I2C_MSG_LENGTH		0x2
#define OV2740_SHORT_MAX	16
#define OV2740_BYTE_MAX		32

static int log_level = 1;
#define OV2740_OTP_LOG(level, a , ...)\
        do {\
                if(level <  log_level)\
                        printk(a,## __VA_ARGS__);\
        }while(0)
static struct i2c_client *client_for_otp = NULL;

//this arrays golden_lenc should be initialized 
//	through dump golden module' lsc data.
static int golden_lenc[240] = {0x33,0x24,0x20,0x1e,0x1e,0x20,0x23,0x31,0x1a,0x16,0x12,0x10,0x10,0x12,0x16,0x1a,0x12,0xc,0xa,0x8,0x8,0x9,0xc,0x11,0xc,0x8,0x5,0x3,0x3,0x5,0x7,0xb,0x9,0x5,0x2,0x1,0x0,0x1,0x5,0x9,
0x9,0x5,0x2,0x0,0x1,0x2,0x5,0x9,0xb,0x8,0x5,0x3,0x3,0x4,0x7,0xb,0x10,0xc,0x9,0x8,0x7,0x8,0xc,0x10,0x1b,0x14,0x10,0xf,0xf,0x10,0x13,0x1b,0x2d,0x22,0x1d,0x1c,0x1c,0x1e,0x20,0x2b,
0x6f,0x77,0x76,0x78,0x77,0x77,0x77,0x7f,0x78,0x7d,0x7d,0x7e,0x7e,0x7d,0x7d,0x7d,0x75,0x7c,0x7c,0x7d,0x7d,0x7d,0x7c,0x7d,0x75,0x7c,0x7d,0x7f,0x80,0x7e,0x7d,0x7d,0x75,0x7e,0x7f,0x80,0x81,0x80,0x7f,0x7e,
0x75,0x7e,0x7f,0x80,0x80,0x80,0x7e,0x7e,0x76,0x7c,0x7d,0x7e,0x7e,0x7c,0x7c,0x7c,0x77,0x7d,0x7c,0x7c,0x7c,0x7b,0x7b,0x7c,0x78,0x7e,0x7d,0x7d,0x7c,0x7b,0x7b,0x7c,0x7f,0x80,0x7e,0x80,0x7f,0x7e,0x7b,0x7b,
0xa6,0xb2,0xb2,0xb5,0xb3,0xb3,0xb3,0xa5,0xb5,0xb5,0xb3,0xb3,0xb3,0xb3,0xb4,0xb7,0xb4,0xae,0xa8,0xa4,0xa4,0xa7,0xad,0xb2,0xac,0xa1,0x93,0x8b,0x8b,0x92,0x9f,0xac,0xa7,0x95,0x87,0x80,0x80,0x86,0x94,0xa5,
0xa7,0x96,0x87,0x80,0x80,0x86,0x94,0xa6,0xad,0xa2,0x94,0x8c,0x8c,0x92,0xa0,0xab,0xb5,0xb0,0xaa,0xa5,0xa6,0xa8,0xae,0xb5,0xb4,0xb6,0xb5,0xb5,0xb4,0xb4,0xb7,0xb4,0xb0,0xb1,0xb7,0xb8,0xb8,0xb5,0xb1,0xad};

static unsigned char ov2740_lsc_array[242] = {0x58,0x00};
enum ov2740_tok_type {
	OV2740_8BIT  = 0x0001,
	OV2740_16BIT = 0x0002,
	};


static int ov2740_i2c_write(struct i2c_client *client, u16 len, u8 *data)
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
ov2740_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (data_length != OV2740_8BIT && data_length != OV2740_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == OV2740_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* OV2740_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = ov2740_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

void OV2740_write_i2c(u16 addr, int data)
{
	ov2740_write_reg(client_for_otp, OV2740_8BIT, addr, data);
}

static int
ov2740_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[OV2740_SHORT_MAX] = {0};
	int err, i;

	if (len > OV2740_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		goto error;
	}

	/* high byte comes first */
	if (len == OV2740_8BIT)
		*val = (u8)data[0];
	else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}
//	printk("zhouyw--%s:  reg = 0x%04x , val = 0x%04x\n", __func__,reg, *val);
	
	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

int OV2740_read_i2c(int addr)
{
	int ret;
	u16 val = 0;

	ret = ov2740_read_reg(client_for_otp, OV2740_8BIT, addr, &val);

	if(ret) {
		printk("read otp from %d failed\n", addr);
	}

	return val;
}

void Delay(int times)
{
	msleep(times);
}


// return value:
// bit[7]: 0 no otp info, 1 valid otp info/AWB
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int read_otp(struct i2c_client *client,struct otp_struct *otp_ptr)
{
    int otp_flag, addr,  i;
    //set 0x5000[5] to "0"
    int temp1;
    int module_id_info;
    int checksum1 = 0;
    client_for_otp = client;
	dev_err(&client->dev, "ov2740 :%s\n", __func__);
    temp1 = OV2740_read_i2c(0x5000);
    OV2740_write_i2c(0x5000, (0x00 & 0x20) | (temp1 & (~0x20)));

    // read OTP into buffer
    OV2740_write_i2c(0x3d84, 0xC0);
    OV2740_write_i2c(0x3d88, 0x70); // OTP start address
    OV2740_write_i2c(0x3d89, 0x10);
    OV2740_write_i2c(0x3d8A, 0x71); // OTP end address
    OV2740_write_i2c(0x3d8B, 0xFF);
    OV2740_write_i2c(0x3d81, 0x01); // load otp into buffer
    Delay(10);

    // OTP base information
    otp_flag = OV2740_read_i2c(0x7010);
    OV2740_OTP_LOG(0,"ov2740_otp:Reg[0x7010]=0x%x\n",otp_flag);

    addr = 0;
    if((otp_flag & 0xc0) == 0x40) {
	addr = 0x7011; // base address of info group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
	addr = 0x7018; // base address of info group 2
    }
    if (addr != 0) {
	(*otp_ptr).flag = 0xc0;
	(*otp_ptr).module_integrator_id = OV2740_read_i2c(addr);
	(*otp_ptr).lens_id = OV2740_read_i2c(addr + 1);
	(*otp_ptr).production_year = OV2740_read_i2c(addr + 2)>>2;
	(*otp_ptr).production_month = ((OV2740_read_i2c(addr + 2) & 0x03)<<2) + (OV2740_read_i2c(addr
		    + 3)>>6);
	(*otp_ptr).production_day = OV2740_read_i2c(addr + 3) & 0x3f;
	(*otp_ptr).rg_ratio = (OV2740_read_i2c(addr + 4)<<2) + ((OV2740_read_i2c(addr + 6)>>6) &
		0x03);
	(*otp_ptr).bg_ratio = (OV2740_read_i2c(addr + 5)<<2) + ((OV2740_read_i2c(addr + 6)>>4) &
		0x03);
    }else {
	(*otp_ptr).module_integrator_id = 0;
	(*otp_ptr).lens_id = 0;
	(*otp_ptr).production_year = 0;
	(*otp_ptr).production_month = 0;
	(*otp_ptr).production_day = 0;
	(*otp_ptr).rg_ratio = 0;
	(*otp_ptr).bg_ratio =0;
    }

	OV2740_OTP_LOG(1,"ov2740_otp:module_id=%d\n",(*otp_ptr).module_integrator_id);
        OV2740_OTP_LOG(1,"ov2740_otp:lens_id=%d\n",(*otp_ptr).lens_id);
        OV2740_OTP_LOG(1,"ov2740_otp:production_year=%d\n",(*otp_ptr).production_year);
        OV2740_OTP_LOG(1,"ov2740_otp:production_month=%d\n",(*otp_ptr).production_month);
        OV2740_OTP_LOG(1,"ov2740_otp:production_day=%d\n",(*otp_ptr).production_day);
        OV2740_OTP_LOG(0,"ov2740_otp:rg_ratio=%d\n",(*otp_ptr).rg_ratio);
        OV2740_OTP_LOG(0,"ov2740_otp:bg_ratio=%d\n",(*otp_ptr).bg_ratio);
        OV2740_OTP_LOG(0,"ov2740_otp:rg_typical=%d\n",RG_Ratio_Typical);
        OV2740_OTP_LOG(0,"ov2740_otp:bg_typical=%d\n",BG_Ratio_Typical);

	//debug code, to remove later
	for(i = 0x7000;i <= 0x700f;i++){
		module_id_info = OV2740_read_i2c(i);
		pr_err("dump_infor:%x=%x\n", i, module_id_info);
	}
	for(i = 0x7011;i <= 0x7014;i++){
		module_id_info = OV2740_read_i2c(i);
		pr_err("dump_infor:%x=%x\n", i, module_id_info);
	}
 /* Yeti DVT2 OV2740 1st supply module WA:
Module_id within below range had swampped rg_ratio and bg_ratio;
Fix-me in PVT version, when these module is replaced with correct version.*/
	module_id_info = OV2740_read_i2c(0x7011);
	if (0x11 <= module_id_info && module_id_info <= 0x2d){
		temp1 = (*otp_ptr).rg_ratio;
		(*otp_ptr).rg_ratio = (*otp_ptr).bg_ratio;
		(*otp_ptr).bg_ratio = temp1;
		OV2740_OTP_LOG(0,"ov2740_otp:force swap,rg_ratio=%d\n",(*otp_ptr).rg_ratio);
        	OV2740_OTP_LOG(0,"ov2740_otp:force swap,bg_ratio=%d\n",(*otp_ptr).bg_ratio);
	}



    // OTP Lenc Calibration
    otp_flag = OV2740_read_i2c(0x7010);
    addr = 0;
    if ((otp_flag & 0x0c) == 0x4) {
	addr = 0x701f; // base address of Lenc Calibration group 1
    }
    if (addr != 0) {
	for(i=0;i<240;i++) {
	    (* otp_ptr).lenc[i]=OV2740_read_i2c(addr + i);
	    ov2740_lsc_array[i + 2] = (* otp_ptr).lenc[i];
	}
	(*otp_ptr).flag |= 0x10;
	(*otp_ptr).lsc_valid = 1;
    } else {
	for(i=0;i<240;i++) {
	    (* otp_ptr).lenc[i]=0;
	}
	(*otp_ptr).lsc_valid = 0;
    }

    //checksum for otp (0x7011~0x710E)%255
    for(i = 0x7011;i <= 0x710E;i++){
	checksum1 += OV2740_read_i2c(i);
    }
    checksum1 = (checksum1)%255;
    (* otp_ptr).checksum = OV2740_read_i2c(0x710F);
    OV2740_OTP_LOG(0,"ov2740_otp:checksum1:%d---0x710F:%d\n",checksum1,(* otp_ptr).checksum);
    if((checksum1 != (* otp_ptr).checksum) || ((*otp_ptr).lsc_valid == 0)){
	//(*otp_ptr).flag &= 0x0;
	//if checksum is error; write the golden moduls 'lsc to this module
	OV2740_OTP_LOG(0,"ov2740_otp:lsc error,apply golden val!!!\n");

		for(i=0;i<240;i++) {
			(* otp_ptr).lenc[i] = golden_lenc[i];
		}
			OV2740_OTP_LOG(0,"ov2740_otp:error--checksum\n");
	(*otp_ptr).flag |= 0x10;
    }

    for(i=0;i<240;i++){
	if (i %40 == 0){
		OV2740_OTP_LOG(1,"\n");
		OV2740_OTP_LOG(1,"ov2740_otp lsc data:");
	}
	OV2740_OTP_LOG(1,"0x%x,",(* otp_ptr).lenc[i]);	
    }	
    OV2740_OTP_LOG(1,"\n");
	
    for(i=0x7010;i<=0x71FF;i++) {
	OV2740_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelerate
    }
    
    //set 0x5000[5] to "1"					
    temp1 = OV2740_read_i2c(0x5000);
    OV2740_write_i2c(0x5000, (0x01 & 0x20) | (temp1 & (~0x20)));
    return (*otp_ptr).flag;
}


// return value:
// bit[7]: 0 no otp info, 1 valid otp info/AWB
// bit[4]: 0 no otp lenc, 1 valid otp lenc
int apply_otp(struct otp_struct *otp_ptr)
{
    int ret, rg, bg, R_gain, G_gain, B_gain, Base_gain, temp;
    struct i2c_msg lsc_msg;

	pr_err("ov2740 :%s\n", __func__);
    // apply OTP WB Calibration
    if ((*otp_ptr).flag & 0x40) {
	rg = (*otp_ptr).rg_ratio;
	bg = (*otp_ptr).bg_ratio;
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
	} else{
	    Base_gain = G_gain;
	}


	R_gain = 0x400 * R_gain / (Base_gain);
	B_gain = 0x400 * B_gain / (Base_gain);
	G_gain = 0x400 * G_gain / (Base_gain);
	OV2740_OTP_LOG(0,"ov2740_otp:apply awb otp:R_GAIN=0x%x,B_GAIN=0x%x,G_GAIN=0x%x\n",
                        R_gain,B_gain,G_gain);
	// update sensor WB gain
	if (R_gain>0x400) {
	    OV2740_write_i2c(0x500A, R_gain>>8);
	    OV2740_write_i2c(0x500B, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
	    OV2740_write_i2c(0x500C, G_gain>>8);
	    OV2740_write_i2c(0x500D, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
	    OV2740_write_i2c(0x500E, B_gain>>8);
	    OV2740_write_i2c(0x500F, B_gain & 0x00ff);
	}
    }
    // apply OTP Lenc Calibration
    if ((*otp_ptr).flag & 0x10) {
	    temp = OV2740_read_i2c(0x5000);
	    temp = 0x80 | temp;

	    OV2740_write_i2c(0x5000, temp);
#if 0
	    for(i=0;i<240;i++) {
	        OV2740_write_i2c(0x5800 + i, (*otp_ptr).lenc[i]);
	        //printk("zhouyw--lenc[%d]:%d-\n",i,(*otp_ptr).lenc[i]);
	    }
#endif
	    lsc_msg.addr = client_for_otp->addr;
	    lsc_msg.flags = 0;
	    lsc_msg.len = sizeof(ov2740_lsc_array);
	    lsc_msg.buf = ov2740_lsc_array;
	    ret = i2c_transfer(client_for_otp->adapter, &lsc_msg, 1);
	    if (ret != 1) {
		    OV2740_OTP_LOG(0, "ov2740_otp: lsc otp write failed\n");
        } else {
	            OV2740_OTP_LOG(0,"ov2740_otp: lsc otp applied\n");
               }
    } 
    return (*otp_ptr).flag;
}

