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
#include "hideep.h"
#include "hideep_swd.h"

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/gpio.h>

//#include <plat/cpu.h>

#define HIDEEP_ROM_MAX_SIZE								0xc000
#define HIDEEP_WHOLE_START_ADDR 				0
#define HIDEEP_WHOLE_LENGHT 						HIDEEP_ROM_MAX_SIZE
#define HIDEEP_BOOTLOADER_START_ADDR 		0
#define HIDEEP_BOOTLOADER_LENGHT 				0x300
#define HIDEEP_CODE_START_ADDR 					0x2c0
#define HIDEEP_CODE_LENGTH 						(HIDEEP_BOOTLOADER_START_ADDR-HIDEEP_CODE_START_ADDR)
#define HIDEEP_CRC_START_ADDR 					0x2c8
#define HIDEEP_CRC_LENGTH								8
#define HIDEEP_CUSTOM_INFO_START_ADDR 	0x2d8
#define HIDEEP_CUSTOM_INFO_LENGHT 			8
#define HIDEEP_CUSTOM_START_ADDR 				0x0
#define HIDEEP_CUSTOM_LENGTH 						HIDEEP_ROM_MAX_SIZE
#define HIDEEP_VR_INFO_START_ADDR 			0x2d8
#define HIDEEP_VR_INFO_LENGHT 					8
#define HIDEEP_VR_START_ADDR 						0x300
#define HIDEEP_VR_LENGTH								HIDEEP_ROM_MAX_SIZE


#define HIDEEP_BOOTLOADER_VERSION_ADDR_BASE     0x2e8
#define HIDEEP_BOOTLOADER_MAJOR_VERSION			(HIDEEP_BOOTLOADER_VERSION_ADDR_BASE+__HIDEEP_MAJOR_VERSION__)
#define HIDEEP_BOOTLOADER_MINOR_VERSION			(HIDEEP_BOOTLOADER_VERSION_ADDR_BASE+__HIDEEP_MINOR_VERSION__)
#define HIDEEP_CODE_VERSION_ADDR_BASE           0x2ea
#define HIDEEP_CODE_MAJOR_VERSION			    (HIDEEP_CODE_VERSION_ADDR_BASE+__HIDEEP_MAJOR_VERSION__)
#define HIDEEP_CODE_MINOR_VERSION			    (HIDEEP_CODE_VERSION_ADDR_BASE+__HIDEEP_MINOR_VERSION__)
#define HIDEEP_CUSTOM_VERSION_ADDR_BASE         0x2ec
#define HIDEEP_CUSTOM_MAJOR_VERSION			    (HIDEEP_CUSTOM_VERSION_ADDR_BASE+__HIDEEP_MAJOR_VERSION__)
#define HIDEEP_CUSTOM_MINOR_VERSION			    (HIDEEP_CUSTOM_VERSION_ADDR_BASE+__HIDEEP_MINOR_VERSION__)
#define HIDEEP_VR_VERSION_ADDR_BASE             0x2ee
#define HIDEEP_VR_MAJOR_VERSION				    (HIDEEP_VR_VERSION_ADDR_BASE+__HIDEEP_MAJOR_VERSION__)
#define HIDEEP_VR_MINOR_VERSION				    (HIDEEP_VR_VERSION_ADDR_BASE+__HIDEEP_MINOR_VERSION__)
#define HIDEEP_FACTORY_ID		  				0x2f0
#define HIDEEP_UPDATE_ENTER_PGM_RETRY_TIME		10
#define HIDEEP_UPDATE_READ_ID_RETRY_TIME		5
#define HIDEEP_VERIFY_ENABLE
#define HIDEEP_AUTO_UPDATE_FROM_BIN
//#define HIDEEP_CHECK_INT_AFTER_REBOOT
int hideep_swd_reset(void);

#define HIDEEP_CHECK_VER_FROM_VR	3

struct s_hideep_being_updated
{
	unsigned int start;
	unsigned int length;
};
const struct s_hideep_being_updated hideep_bootloader_block[]=
{
		{HIDEEP_WHOLE_START_ADDR,HIDEEP_WHOLE_LENGHT},
};
const struct s_hideep_being_updated hideep_code_block[]=
{
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CUSTOM_INFO_START_ADDR,HIDEEP_CUSTOM_INFO_LENGHT},
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CUSTOM_INFO_START_ADDR,HIDEEP_CUSTOM_INFO_LENGHT},
		{HIDEEP_CODE_START_ADDR,HIDEEP_CODE_LENGTH},
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CUSTOM_INFO_START_ADDR,HIDEEP_CUSTOM_INFO_LENGHT},
};
const struct s_hideep_being_updated hideep_custom_block[]=
{
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CUSTOM_INFO_START_ADDR,HIDEEP_CUSTOM_INFO_LENGHT},
		{HIDEEP_CUSTOM_START_ADDR,HIDEEP_CUSTOM_LENGTH},
};
const struct s_hideep_being_updated hideep_vr_block[]=
{
		{HIDEEP_VR_START_ADDR,HIDEEP_VR_LENGTH},
		{HIDEEP_BOOTLOADER_START_ADDR,HIDEEP_BOOTLOADER_LENGHT},
};
struct s_hideep_fw_update_block_info
{
	int block_count;
	struct s_hideep_being_updated *pblock;
};

const struct s_hideep_fw_update_block_info hideep_fw_update_blocks_info[]=
{
		{
				sizeof(hideep_bootloader_block)/sizeof(struct s_hideep_being_updated),
				(struct s_hideep_being_updated *)hideep_bootloader_block,
		},
		{
				sizeof(hideep_code_block)/sizeof(struct s_hideep_being_updated),
				(struct s_hideep_being_updated *)hideep_code_block,
		},
		{
				sizeof(hideep_custom_block)/sizeof(struct s_hideep_being_updated),
				(struct s_hideep_being_updated *)hideep_custom_block,
		},
		{
				sizeof(hideep_vr_block)/sizeof(struct s_hideep_being_updated),
				(struct s_hideep_being_updated *)hideep_vr_block,
		},
};

static unsigned short hideep_version_addr[]={
	TOUCH_READ_BL_VER_ADDR,
	TOUCH_READ_CODE_VER_ADDR,
	TOUCH_READ_CUST_VER_ADDR,
	TOUCH_READ_VR_VER_ADDR,
	TOUCH_READ_FACTORY_ID_ADDR
};

int check_version(struct i2c_client *client, u8 *val)
{
    int ret = 0;
    int i = 0;
    unsigned char buf[2];
    int count;
    int retry;

    dbg_fun();
    //bootloader...
	count = sizeof(hideep_version_addr)/sizeof(unsigned short);
    for (i = 0; i < count; i++){
        for(retry = 0;retry <5;retry++){
            ret = hideep_i2c_read(client, hideep_version_addr[i], 2, buf);
            if(buf[0] || buf[1])
                break;
        }
        dbg_log("addr = 0x%x,buf = %2x,%2x",hideep_version_addr[i],buf[0],buf[1]);
        if (ret >= 0){
            val[i*2+0] = buf[__HIDEEP_MINOR_VERSION__];
            val[i*2+1] = buf[__HIDEEP_MAJOR_VERSION__];
    		dbg_log("version = %d.%2d",val[i*2+1],val[i*2+0]);
        }
        else
        	break;
    }
    if (ret < 0){
        dbg_err("i2c read fail[%d] \n", ret);
				goto error;
    }
    return ret;
error:
    return ret;
}
int hideep_pgm_calculate_crc(struct i2c_client *client,unsigned char * buf, unsigned int file_length, unsigned char *data_buf)
{
    int i = 0, j = 0, offset_count =0, crc_count;
    unsigned short us_crc;
    int ret = 0;

    us_crc = 0xffff;
    crc_count = 0;
    offset_count = 0;
    for (i = 0x300; i < file_length; i++){
        //dbg_log("addr = 0x%08x, us_crc = 0x%04x, rom_data = 0x%02x",i, us_crc, buf[i]);
        us_crc ^= buf[i];
        for (j = 0; j < 8; j++){
            if (us_crc & 0x01)
                us_crc  =   (us_crc >> 1) ^ 0xA001;
            else
                us_crc  =   us_crc >> 1;
        }
        offset_count++;
        if (offset_count >= 0x2000 || i >= file_length - 1){
            dbg_log("the number (%d) us_crc = 0x%04x", crc_count, us_crc);
            data_buf[crc_count*2+0] = us_crc & 0xff;
            data_buf[crc_count*2+1] = (us_crc>>8) & 0xff;
            us_crc          =   0xFFFF;
            offset_count    =   0;
            crc_count++;
        }
    }

    //kfree(buf);
    return ret;
}

void get_fw_version(unsigned char *pres, unsigned short *pver)
{
	pver[0] = (pres[HIDEEP_BOOTLOADER_MAJOR_VERSION]<<8) | pres[HIDEEP_BOOTLOADER_MINOR_VERSION];
	pver[1] = (pres[HIDEEP_CODE_MAJOR_VERSION]<<8)       | pres[HIDEEP_CODE_MINOR_VERSION];
	pver[2] = (pres[HIDEEP_CUSTOM_MAJOR_VERSION]<<8)     | pres[HIDEEP_CUSTOM_MINOR_VERSION];
	pver[3] = (pres[HIDEEP_VR_MAJOR_VERSION]<<8)         | pres[HIDEEP_VR_MINOR_VERSION];
	pver[4] = pres[HIDEEP_FACTORY_ID];
}

unsigned int hideep_swd_is_flash_ready(void)
{
	unsigned int tmp;
	if(hideep_swd_read(NVM_FLASH_STA_32, &tmp)==0)
		return tmp & 0x01;
	else
	{
	    private_ts->pdata->swd_uninit();
	    private_ts->pdata->swd_init();
        hideep_swd_reset();
		return 0;
	}
}
int hideep_swd_set_flash_pio(unsigned int data)
{
	return hideep_swd_write(NVM_FLASH_CON_8, ((data) << 1)|1);
}
int hideep_swd_set_pio_sig(unsigned int addr, unsigned int data)
{
	return hideep_swd_write((FLASH_BASE + 0x400000)+addr, data);
}
int hideep_swd_set_flash_hw_control(void)
{
	return hideep_swd_write(NVM_FLASH_CON_8,0);
}
int hideep_set_protect_mode(unsigned int protect)
{
	int ret = 0;
	int i = 0;

	unsigned int pioaddr = 0;
	unsigned int tmp[4];

	ret = hideep_swd_write(NVM_FLASH_TIM_8, 3);
	if(ret != 0)goto error;
	ret = hideep_swd_write(NVM_FLASH_CFG_8, 0x02);
	if(ret != 0)goto error;
	for(i = 0;i<4;i++){
		ret = hideep_swd_read(0x00000000+i*4,&tmp[i]);
		if(ret != 0)goto error;
	}
	ret = hideep_swd_write(NVM_FLASH_CFG_8, 0x06);
	if(ret != 0)goto error;
	for(i = 0;i<4;i++){
		ret = hideep_swd_read(0x00000000+i*4,&tmp[i]);
		if(ret != 0)goto error;
	}
	udelay(20);
	if (protect == 0){
		if(!(hideep_swd_is_flash_ready())){
			ret = -1;
			goto error;
		}
		ret = hideep_swd_write(NVM_FLASH_CFG_8, 1);
		if(ret != 0)goto error;
		ret = hideep_swd_set_flash_pio(0);
		if(ret != 0)goto error;
		ret = hideep_swd_set_flash_pio(1);
		if(ret != 0)goto error;
		ret = hideep_swd_set_pio_sig(pioaddr + 12, 0x00310000);
		if(ret != 0)goto error;
		ret = hideep_swd_set_flash_pio(0);
		if(ret != 0)goto error;
		ret = hideep_swd_write(NVM_FLASH_CFG_8, 0);
		if(ret != 0)goto error;
		ret = hideep_swd_set_flash_hw_control();
		if(ret != 0)goto error;
		udelay(30);
	}
	else{
		if(!(hideep_swd_is_flash_ready())){
			ret = -1;
			goto error;
		}
		ret = hideep_swd_write(NVM_FLASH_CFG_8, 0);
		if(ret != 0)goto error;
		ret = hideep_swd_write(NVM_FLASH_CFG_8, 6);
		if(ret != 0)goto error;

		for(i = 0;i<3;i++){
			ret = hideep_swd_read(0x00000000+i*4,&tmp[i]);
			if(ret != 0)goto error;
		}
		ret = hideep_swd_write(NVM_FLASH_CFG_8, 0);
		if(ret != 0)goto error;
		ret = hideep_swd_write(NVM_FLASH_CFG_8, 6);
		if(ret != 0)goto error;

		pioaddr = PROG;

		ret = hideep_swd_set_flash_pio(0);
		if(ret != 0)goto error;
		ret = hideep_swd_set_flash_pio(1);
		if(ret != 0)goto error;
		ret = hideep_swd_set_pio_sig(pioaddr + 0, 0xFFFFFFFF);
		if(ret != 0)goto error;
		for(i = 0;i<3;i++){
			ret = hideep_swd_set_pio_sig(pioaddr + i*4, tmp[i]);
			if(ret != 0)goto error;
		}
		if(protect){
			ret = hideep_swd_set_pio_sig(pioaddr + 12, (0x00310000 | _PROT_MODE));
			if(ret != 0)goto error;
		}
		else{
			ret = hideep_swd_set_pio_sig(pioaddr + 12, 0x00310000);
			if(ret != 0)goto error;
		}

		ret = hideep_swd_set_pio_sig(0, 0x00000000);
		if(ret != 0)goto error;
		ret = hideep_swd_set_flash_pio(0);
		if(ret != 0)goto error;
		udelay(30);
		ret = hideep_swd_write(NVM_FLASH_CFG_8, 0);
		if(ret != 0)goto error;
		ret = hideep_swd_set_flash_hw_control();
		if(ret != 0)goto error;
		udelay(30);
	}
	return ret;
error:
	return ret;
}

int hideep_nvm_write_page(unsigned int addr, unsigned char *buf, unsigned int len)
{
	int i = 0;
	int ret = 0;
	unsigned int pioaddr;
	unsigned int data;
  unsigned int verify_addr;
#ifdef HIDEEP_VERIFY_ENABLE
	unsigned int read;
#endif
	
	//dbg_fun();
	if(!(hideep_swd_is_flash_ready())){
		ret = -1;
		goto error;
	}
	if(addr>0xc000 && addr < 0xc3ff){
		addr -= 0xc000;
		pioaddr = PROG | INF;
		verify_addr = _EEPROM_MEM_BASE + 0xc000;
	}
	else{
		pioaddr = PROG;
		verify_addr = _EEPROM_MEM_BASE;
	}
	//program
	ret = hideep_swd_set_flash_pio(0);
	if(ret != 0)goto error;
	ret = hideep_swd_set_flash_pio(1);
	if(ret != 0)goto error;
	
	data =  (buf[3]<<24)+(buf[2]<<16)+(buf[1]<<8)+buf[0];
	ret = hideep_swd_set_pio_sig(pioaddr + (0xFF80 & addr), data);
    dbg_log("addr = 0x%04x,data = 0x%08x",addr,data);
	for (i = 0; i < len; i += 4) {
		data =  (buf[i+3]<<24)+(buf[i+2]<<16)+(buf[i+1]<<8)+buf[i+0];
		//dbg_log("0x%04x,data=0x%08x",addr+i,data);
		ret = hideep_swd_set_pio_sig(pioaddr + (0xFF80 & addr) + i, data);
        //mdelay(2);
		if(ret != 0)goto error;
	}
	ret = hideep_swd_set_pio_sig(0, data);
	udelay(5);
	hideep_swd_set_flash_pio(0);
	if(ret != 0)goto error;
	for (i = 0; i < 20; i++){
		if (hideep_swd_is_flash_ready())
		{
			ret = hideep_swd_set_flash_hw_control();
			if(ret != 0)goto error;
            dbg_fun();
			break;
		}
		udelay(50);
	}
	dbg_log("i=%d",i);
	//verify
#ifdef HIDEEP_VERIFY_ENABLE
	for (i = 0; i < len; i += 4) {
		data =  (buf[i+3]<<24)+(buf[i+2]<<16)+(buf[i+1]<<8)+buf[i+0];
		ret = hideep_swd_read(verify_addr + (addr & 0xFF80) + i, &read);
		if(ret != 0)goto error;
		//dbg_log("0x%04x,data=0x%08x read = 0x%08x",addr+i,data, read);
		if(data != read){
			dbg_err("data = 0x%08x",data);
			dbg_err("read = 0x%08x",read);
			ret = hideep_swd_write(_YRAM_BASE + 12, read);
			ret = -2;
			goto error;			
		}
	}
#endif
	return ret;
error:
    dbg_err();
	private_ts->pdata->swd_uninit();
	private_ts->pdata->swd_init();
    hideep_swd_reset();
	return ret;
}

int hideep_swd_program(unsigned int start, unsigned char *buf, unsigned int len)
{
	unsigned int count,offset;
	int ret = 0;
	unsigned int size;
	int retry;

	size = len;
	offset = 0;
	dbg_fun();
	while (size > 0) {
		if (size >= 128) {
			count = 128;
			size -= 128;
		}
		else {
			count = size;
			size = 0;
		}
		for(retry = 0; retry < 3; retry++){
			ret = hideep_nvm_write_page(start, &buf[offset], count);
			if(ret == 0)break;
            dbg_log("write retry = %d", retry);
			//if(ret == -2) continue;
			//goto error;
		}
        if(ret !=0)
            goto error;
		offset += count;
		start += count;
	}
	return ret;
error:
	dbg_err("ret = %d",ret);
	return ret;
}

int hideep_check_which_part_be_updated(unsigned char *pres, int *index)
{
	unsigned short cur_ver[5];
	unsigned short res_ver[5];
	int ret;
	int update_index;

	//check which part is need to be updated.
	private_ts->pdata->int_enable(private_ts, 0);
	private_ts->pdata->reset();
	msleep(10);
	ret = check_version(private_ts->client,(unsigned char *)cur_ver);
	private_ts->pdata->int_enable(private_ts, 1);
	if (ret < 0){
		dbg_err("i2c error, ret = 0x%08x", ret);
		goto error;;
	}
	get_fw_version(pres, res_ver); 	
	for(update_index = HIDEEP_CHECK_VER_FROM_VR; update_index < 4; update_index++){
        dbg_log("%d [0=bl; 1=code; 2=cus; 3=vr] cur=0x%x, res=0x%x", update_index,cur_ver[update_index], res_ver[update_index]);
	  	if(cur_ver[update_index]<res_ver[update_index]){	      		
	      		break;
	  	}
	}
	if(update_index >= 4){
  		dbg_err("no need updated.");
	}
	*index = update_index;
	return ret;
error:
	return ret;
}
int hideep_swd_reset(void)
{
    int ret;
	int id_ok;
	int retry;
	int id_retry;
	unsigned int tmp;
	id_ok = 0;
	for(retry = 0; retry < HIDEEP_UPDATE_ENTER_PGM_RETRY_TIME;retry++){
		ret = hideep_blaster_init();
		if(ret != 0){
			dbg_err("CANNOT enter swd mode.");
			goto error_swd_ini;
		}
		for(id_retry = 0; id_retry < HIDEEP_UPDATE_READ_ID_RETRY_TIME;id_retry++){
			ret = hideep_swd_read(0x5200007C,&tmp);
			if(ret != 0)goto error;
            if((tmp > 5) &&	(tmp < 11)){
				id_ok = 1;
				break;
			}
			msleep(2);
		}
		if(id_ok)
			break;
		msleep(2);
	}
	if(!id_ok){
		dbg_err("ID[0x%8d] is not correct", tmp);
		goto error;
	}
	ret = hideep_set_protect_mode(0);
    return ret;
error:
error_swd_ini:
    dbg_err();
    return ret;
}
int hideep_update_function(unsigned char *pres, unsigned int len, int update_index)
{
	struct i2c_client *i2c = private_ts->client;
	//unsigned int tmp;
	unsigned char *pdata;
	unsigned int size = len;
	const struct s_hideep_fw_update_block_info *pfub;
	struct s_hideep_being_updated *pbu;
	int32_t ret;
	int i;
    
	dbg_fun();
	private_ts->updating = 1;
	pdata = kmalloc(len,GFP_KERNEL);
	if(pdata == NULL){
		dbg_err("alloc memory!");
		goto error_alloc_memory;
	}
	memcpy(pdata,pres,len);
	// chip specific code for flash fuse
	dbg_fun("fw tp info = 0x%x",pdata[HIDEEP_FACTORY_ID]);
	private_ts->pdata->int_enable(private_ts, 0);
	hideep_pgm_calculate_crc(i2c, pdata, size, (unsigned char *)&pdata[HIDEEP_CRC_START_ADDR]);

  	//initial swd.
	private_ts->pdata->swd_init();
    ret = hideep_swd_reset();
	if(ret != 0)goto error_swd_ini;
	//updating....
	pfub = &hideep_fw_update_blocks_info[update_index];
	pbu = pfub->pblock;
	dbg_log("update_index=%d,pfub->block_count=%d",update_index, pfub->block_count);
	for(i =0;i<pfub->block_count;i++)
	{
        if(pbu->length > len)
            size = len;
        else
            size = pbu->length;
			dbg_log("block[%d], start = 0x%x, length = 0x%08x",i, pbu->start,size);
			ret = hideep_swd_program(pbu->start, pdata+pbu->start, size);
			if(ret != 0)goto error;
		pbu++;
	}
	dbg_log("i=%d,pfub->block_count=%d",i, pfub->block_count);
	dbg_log("success.");
	private_ts->pdata->swd_uninit();
	gpio_direction_input(private_ts->pdata->gpio_int);
	private_ts->pdata->int_enable(private_ts, 1);
	private_ts->pdata->reset();
	msleep(10);
	//-------------------------------------------
	kfree(pdata);
	private_ts->updating = 0;
	return 0;
error:
error_swd_ini:
	private_ts->pdata->swd_uninit();
	gpio_direction_input(private_ts->pdata->gpio_int);
	private_ts->pdata->int_enable(private_ts, 1);
	private_ts->pdata->reset();
	msleep(10);
	kfree(pdata);
error_alloc_memory:
	private_ts->updating = 0;
	return -1;
}

int hideep_update(struct device *dev, const char *fn)
{
	const struct firmware *fw_entry;
	int ret = -1;
	int update_index;
    update_index = 0;
    dbg_fun();
	ret = request_firmware(&fw_entry, fn, dev);
	if(ret != 0){
		dev_err(dev, "request_firmware : fail(%d)\n", ret);
		goto exit;
	}
    #if 0
	ret = hideep_check_which_part_be_updated((unsigned char *)fw_entry->data, &update_index);
	if(ret<0)
		goto error_fm;
    #endif
	ret = hideep_update_function((unsigned char *)fw_entry->data,(unsigned int)fw_entry->size, update_index);
//error_fm:
	release_firmware(fw_entry);
exit:
	return ret;
}

void hideep_update_fw_thread(struct work_struct *work)
{
	int ret;
#ifdef HIDEEP_CHECK_INT_AFTER_REBOOT
	int i;
#endif
	int update_index;
    unsigned char *fw_buf;
    unsigned int fw_length;
#ifdef HIDEEP_AUTO_UPDATE_FROM_BIN
	const struct firmware *fw_entry;
#endif

    //goto exit ;
    private_ts->updating = 1;
	private_ts->pdata->int_enable(private_ts, 0);
	private_ts->pdata->reset();
	private_ts->wait_int = 0;
	private_ts->pdata->int_enable(private_ts, 1);
#ifdef HIDEEP_CHECK_INT_AFTER_REBOOT
	dbg_log("check interrupt handle be triggle");
	for(i = 0; i< 200;i++){
		mdelay(50);
		dbg_log("wait %d msec",i*50);
		if(private_ts->wait_int)
			break;
	}
#else
    private_ts->wait_int=1;
#endif
    #if 1
    #ifdef HIDEEP_AUTO_UPDATE_FROM_BIN
    #ifdef	HIDEEP_TP_VENDOR
    dbg_fun();
    if(private_ts->pdata->tp_vendor()){
    	dbg_fun("%s",HIDEEP_FW);
	    ret = request_firmware(&fw_entry, HIDEEP_FW, &private_ts->client->dev);
	}
	else{
    	dbg_fun("%s",HIDEEP_MT_FW);
			ret = request_firmware(&fw_entry, HIDEEP_MT_FW, &private_ts->client->dev);
		}
    #else
        dbg_fun("not use to destinguish TP");
		ret = request_firmware(&fw_entry, HIDEEP_FW, &private_ts->client->dev);
	#endif
	if(ret != 0){
	    dbg_err("request_firmware : fail(%d)\n", ret);
	    goto exit;
	}
    fw_buf = (unsigned char *)fw_entry->data;
    fw_length = (unsigned int)fw_entry->size;
    #else
    #ifdef	HIDEEP_TP_VENDOR
    if(private_ts->pdata->tp_vendor()){
    	dbg_fun();
    	fw_buf = (unsigned char *)HiDeep_FW;
    	fw_length = (unsigned int)HiDeep_FW_nLength;
    }
    else{
    	dbg_fun();
    	fw_buf = (unsigned char *)HiDeep_MT_FW;
    	fw_length = (unsigned int)HiDeep_MT_FW_nLength;
    }
    #else
    fw_buf = (unsigned char *)HiDeep_FW;
    fw_length = (unsigned int)HiDeep_FW_nLength;
    #endif
    #endif
    if(fw_length<0x300){
        dbg_err("fw is not correct len = %d !", fw_length);
        goto exit;
    }
	if(private_ts->wait_int==0){
		dbg_err("CANNOT get the interrupt signal");
		ret = hideep_update_function(fw_buf,fw_length, 0);
	}
	else{
		ret = hideep_check_which_part_be_updated(fw_buf, &update_index);
		if(ret<0)
			goto exit;
		if(update_index>=4)
			goto exit;
			
	  ret = hideep_update_function(fw_buf,fw_length, update_index);
	}
    #ifdef HIDEEP_AUTO_UPDATE_FROM_BIN
	release_firmware(fw_entry);
    #endif
    private_ts->updating = 0;
    return;
exit:
    #endif
    private_ts->updating = 0;
	return;
}

