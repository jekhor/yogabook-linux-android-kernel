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
 * basic operation to access crimson
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *mount -t vfat32 /vendor /dev/block/mmcblk1p1
 *-----------------------------------------------------------------------------*/
#define PGM_BURST_WR
#define PGM_VERIFY

/*------------------------------------------------------------------------------
 * register map
 *-----------------------------------------------------------------------------*/
#define YRAM_BASE                    (0x40000000)
#define PERIPHERAL_BASE              (0x50000000)
#define ESI_BASE                     (PERIPHERAL_BASE + 0x00000000)
#define FLASH_BASE                   (PERIPHERAL_BASE + 0x01000000)
#define SYSCON_BASE                  (PERIPHERAL_BASE + 0x02000000)

#define SYSCON_MOD_CON               (SYSCON_BASE + 0x0000)
#define SYSCON_SPC_CON               (SYSCON_BASE + 0x0004)
#define SYSCON_CLK_CON               (SYSCON_BASE + 0x0008)
#define SYSCON_CLK_ENA               (SYSCON_BASE + 0x000C)
#define SYSCON_RST_CON               (SYSCON_BASE + 0x0010)
#define SYSCON_WDT_CON               (SYSCON_BASE + 0x0014)
#define SYSCON_WDT_CNT               (SYSCON_BASE + 0x0018)
#define SYSCON_PWR_CON               (SYSCON_BASE + 0x0020)
#define SYSCON_PGM_ID                (SYSCON_BASE + 0x00F4)

#define FLASH_CON                    (FLASH_BASE  + 0x0000)
#define FLASH_STA                    (FLASH_BASE  + 0x0004)
#define FLASH_CFG                    (FLASH_BASE  + 0x0008)
#define FLASH_TIM                    (FLASH_BASE  + 0x000C)
#define FLASH_CACHE_CFG              (FLASH_BASE  + 0x0010)

#define ESI_TX_INVALID				 (ESI_BASE	  + 0x0008)

/*------------------------------------------------------------------------*//**
 * flash commands
 *//*--------------------------------------------------------------------------*/
#define MERASE                       (0x00010000)
#define SERASE                       (0x00020000)
#define PERASE                       (0x00040000)
#define PROG                         (0x00080000)
#define WRONLY                       (0x00100000)
#define INF                          (0x00200000)

#define NVM_PAGE_SIZE                (128)

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
typedef struct pgm_packet
{
    union
    {
        u8   b[8];
        u32  w[2];
    }header;

    u32 payload[NVM_PAGE_SIZE/sizeof(u32)];
}PGM_PACKET;

/*******************************************************************************
 * basic operation to access crimson
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
pgm_r_reg(struct i2c_client *client, u32 addr, u32 *val)
{
    int ret = 0;
    u32 packet[3];
    u8* bulk = (u8*)packet + 3;

    packet[0] = htonl(0x00);
    packet[1] = htonl(addr);

    ret = i2c_master_send(client, bulk,              5);             //write address
    if (ret < 0)
    {
        goto err;
    }
    mdelay(1);
    ret = i2c_master_recv(client, (u8*)&(packet[2]), 4);
    if (ret < 0)
    {
        goto err;
    }

    *val = ntohl(packet[2]);

err:
    return ret;

}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
pgm_w_reg(struct i2c_client *client, u32 addr, u32 data)
{
    int ret = 0;
    u32 packet[3];
    u8* bulk = (u8*)packet + 3;

    packet[0] = htonl(0x80);
    packet[1] = htonl(addr);
    packet[2] = htonl(data);

    //i2c_master_send
    ret = i2c_master_send(client, bulk+0, 5);
    if (ret < 0)
    {
        goto err;
    }

    ret = i2c_master_send(client, bulk+5, 4);
    if (ret < 0)
    {
        goto err;
    }

 err:
    return ret;
}

#ifdef PGM_BURST_WR
/*------------------------------------------------------------------------------
 * burst write
 *-----------------------------------------------------------------------------*/
static int
pgm_w_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len)
{
    int ret   = 0;
    int i;

    if(len % 4 != 0)
        return -1;

    //---------------------------------
    // header
    packet->header.w[0] = htonl( (0x80|(len/4-1)));
    packet->header.w[1] = htonl(addr);

    for(i=0; i<NVM_PAGE_SIZE/sizeof(u32); i++)
        packet->payload[i] = htonl(packet->payload[i]);

    //i2c_master_send
    ret = i2c_master_send(client, (u8*)&packet->header.b[3],(len+5));
    if (ret < 0)
    {
        goto err;
    }

 err:
    return ret;
}

/*------------------------------------------------------------------------------
 * burst read
 *-----------------------------------------------------------------------------*/
static int
pgm_r_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len)
{
    int ret   = 0;
    int i;

    if(len % 4 != 0)
        return -1;

    //---------------------------------
    // header
    packet->header.w[0] = htonl( (0x00|(len/4-1)));
    packet->header.w[1] = htonl(addr);

    ret = i2c_master_send(client, (u8*)&packet->header.b[3], 5);             //write address
    if (ret < 0)
    {
        goto err;
    }
    //i2c_master_send
    ret = i2c_master_recv(client, (u8*)packet->payload,    len);
    if (ret < 0)
    {
        goto err;
    }

    for(i=0; i<NVM_PAGE_SIZE/sizeof(u32); i++)
        packet->payload[i] = htonl(packet->payload[i]);

 err:
    return ret;
}

#endif

/*******************************************************************************
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static void
ist_sw_reset(struct i2c_client *client, u32 food)
{
    pgm_w_reg(client, SYSCON_WDT_CNT, food);
    pgm_w_reg(client, SYSCON_WDT_CON, 0x03);
    pgm_w_reg(client, SYSCON_WDT_CON, 0x01);

    ISTCORE_INFO("sw reset\n");

    return;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
ist_enter_pgm(struct i2c_client *client)
{
    s32 ret = 0;
    u32 status;
    u32 pattern = 0xDF9DAF39;

    //--------------------------------
    // pgm pattern
    i2c_master_send(client, (u8*)&pattern, 4);
    mdelay(1);

	// flush invalid Tx load register
	pgm_w_reg(client, ESI_TX_INVALID, 0x01);

    pgm_r_reg(client, SYSCON_PGM_ID, &status);

    if( status != htonl(pattern))
    {
        ISTCORE_ERR("enter_pgm : error(%08x):\n", status);
        return -1;
    }

    //--------------------------------
    pgm_w_reg(client, SYSCON_WDT_CON, 0x00);
    pgm_w_reg(client, SYSCON_SPC_CON, 0x00);                    // remap
    pgm_w_reg(client, SYSCON_CLK_ENA, 0xFF);                    // Clock Enable for YRAM/DEMOD/ACU/MSP/CM0
    pgm_w_reg(client, SYSCON_CLK_CON, 0x01);                    // Select MOSC
    pgm_w_reg(client, SYSCON_PWR_CON, 0x01);                    //
    pgm_w_reg(client, FLASH_TIM,      0x03);
    pgm_w_reg(client, FLASH_CACHE_CFG,0x00);                    // cache disable
    pgm_w_reg(client, FLASH_CACHE_CFG,0x02);                    // cache flush..


    mdelay(1);

    return ret;
}


/*******************************************************************************
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define SET_FLASH_PIO(CE)        pgm_w_reg(client, FLASH_CON                    , 0x01 | ((CE) << 1))
#define SET_PIO_SIG(X, Y)        pgm_w_reg(client, (FLASH_BASE + 0x400000) + (X),                  Y)
#define SET_FLASH_HWCONTROL      pgm_w_reg(client, FLASH_CON, 0x00)
#define NVM_DEFAULT_PAGE         0
#define NVM_SFR_WPAGE            1
/*-----------------------------------------------------------------------------
 *  Program Page in Flash Memory
 *-----------------------------------------------------------------------------*/
static s32
ist_program_page( struct i2c_client *client, u16 addr, struct pgm_packet *packet_w)
{
    u32  pio_cmd = PROG;
    u32  status;
#ifndef PGM_BURST_WR
    s32   i;
#endif

    pgm_r_reg(client, FLASH_STA, &status);                  // flash status
    if(status == 0)
        return -1;

    addr = addr & ~(NVM_PAGE_SIZE-1);                   // address mask

    //-------------------------------------------
    // fusing begin
    SET_FLASH_PIO(0);
    SET_FLASH_PIO(1);

    SET_PIO_SIG(pio_cmd+addr, htonl(packet_w->payload[0]));
#ifdef PGM_BURST_WR
    pgm_w_mem(client, (FLASH_BASE + 0x400000) + pio_cmd, packet_w, NVM_PAGE_SIZE);
#else
    for (i = 0; i < NVM_PAGE_SIZE/4; i++)
    {
        SET_PIO_SIG(pio_cmd + (i<<2), packet_w->payload[i]);
    }
#endif
    SET_PIO_SIG(124,          htonl(packet_w->payload[31]));

    SET_FLASH_PIO(0);

    mdelay(3);                                         //optimize.......
    while(1)
    {
        pgm_r_reg(client, FLASH_STA, &status);
        if( (status) != 0)
            break;
    }

    pgm_w_reg(client, FLASH_CON, 0);
    // fusing end..
    //-------------------------------------------


    return 0;
}

/*******************************************************************************
 * chip specific fuse
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static struct pgm_packet packet_w;
static struct pgm_packet packet_r;

/*------------------------------------------------------------------------------
 *
 * ----------------------------------------------------------------------------*/

int read_trim_data(struct i2c_client *client)
{
    unsigned int tmp;
    pgm_w_reg(client, 0x51000008, 2);
    pgm_r_reg(client, 0x00000000, (unsigned int*)&tmp);
    tmp = (tmp&0x0000F000)>>12;
    pgm_w_reg(client, 0x51000008,0);
    return tmp;
}

#define NVM_W_SFR(x,y)    { SET_FLASH_PIO(1); SET_PIO_SIG(x,y); SET_FLASH_PIO(0); }
int ist_nvm_unlock(struct i2c_client *client)
{
	int ret = 0;
	unsigned char trim_data = 0x08;
#if 1	
	trim_data = read_trim_data(client);
#endif
	pgm_w_reg(client, FLASH_TIM, 3);
	pgm_w_reg(client, FLASH_CFG, NVM_SFR_WPAGE);
	SET_FLASH_PIO(0);
#if 1
	NVM_W_SFR(0,  0x28170EA0 |((trim_data&0x0f)<<12));
	NVM_W_SFR(4,  0x0A0E03FF);
	NVM_W_SFR(8,  0x8C203D0C);
#endif	
	NVM_W_SFR(12, 0x0030027B);
	SET_FLASH_HWCONTROL;
	pgm_w_reg(client, FLASH_CFG, NVM_DEFAULT_PAGE);
	return ret;
}

void ist_reset_ic(void)
{
    ISTCORE_INFO("enter\n");
    gpio_direction_output(HIDEEP_RESET_GPIO, 1);
    msleep(1);
    gpio_direction_output(HIDEEP_RESET_GPIO, 0);
    msleep(5);
    gpio_direction_output(HIDEEP_RESET_GPIO, 1);
    msleep(220);
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
ist_program_nvm(struct i2c_client *client, const u8 *ucode, size_t len, int offset)
{
    size_t i;
    s32 ret;
    u32 pages ;
    u32 addr;
    s32 retry = 4;
    s32 len_r;
    s32 len_w;

    //------------------------------------------
    // enter pgm
    while(retry--)
    {
        ISTCORE_DBG("enter_pgm : %d\n", (4-retry));
        ret = ist_enter_pgm(client);
        if(ret >= 0)
            break;
        msleep(200);
        ist_reset_ic();
    }
    if(retry <= 0)
    {
        ISTCORE_ERR("enter_pgm : failed\n");
        return -1;
    }

	ist_nvm_unlock(client);
    //------------------------------------------
    // fusing nvm
    pages      = (len + NVM_PAGE_SIZE - 1)/ NVM_PAGE_SIZE;
    addr       = offset;
    len_r      = len;
    len_w      = len_r;

    for(i=0; i<pages; i++)
    {
        if(len_r >= NVM_PAGE_SIZE)
            len_w = NVM_PAGE_SIZE;
        memcpy(packet_w.payload, &(ucode[addr]), len_w);
        ret = ist_program_page(client, i * NVM_PAGE_SIZE+offset, &packet_w);
        if(ret < 0)
        {
            ISTCORE_ERR("ist510e_program_nvm : error(%08x):\n", addr);
        }
        addr   += NVM_PAGE_SIZE;
        len_r  -= NVM_PAGE_SIZE;
        len_w   = len_r;
    }
    return ret;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
ist_verify_nvm(struct i2c_client *client, const u8 *ucode, size_t len, int offset)
{
    s32 i;
    s32 j;
    s32 ret = 0;
    u32 addr  = offset;
    u32 pages = (len + NVM_PAGE_SIZE - 1)/ NVM_PAGE_SIZE;
    s32 len_r = len;
    s32 len_v = len_r;

    for(i=0; i<pages; i++)
    {
        if(len_r >= NVM_PAGE_SIZE)
            len_v = NVM_PAGE_SIZE;

#ifdef PGM_BURST_WR
        pgm_r_mem(client, 0x00000000 + addr, &packet_r ,NVM_PAGE_SIZE);
#else
        for (j = 0; j < NVM_PAGE_SIZE/4; j++)
        {
            pgm_r_reg(client, addr+(j<<2), &(packet_r.payload[j]));
        }
#endif
        ret  = memcmp(&(ucode[addr]), packet_r.payload, len_v);
        if(ret != 0)
        {
            u8 *read = (u8*) packet_r.payload;

            for(j=0; j<NVM_PAGE_SIZE; j++)
                ISTCORE_ERR("%02x : %02x\n", ucode[addr+j], read[j]);

            ISTCORE_ERR("verify : error(addr : %d)\n", addr);

            ret = -1;
        }
        addr   += NVM_PAGE_SIZE;
        len_r  -= NVM_PAGE_SIZE;
        len_v  = len_r        ;
    }

  return ret;
}

/*------------------------------------------------------------------------------
 * get dwz info. from binary file
 *-----------------------------------------------------------------------------*/
static
void get_dwz_from_binary(unsigned char *pres, DWZ_INFO_T *dwz_info)
{
    memcpy(dwz_info, pres + DWZ_ADDR, sizeof(DWZ_INFO_T));
}

/*------------------------------------------------------------------------------
 * get dwz info. from binary file
 *-----------------------------------------------------------------------------*/
static
void set_dwz_from_binary(unsigned char *pres, DWZ_INFO_T *dwz_info)
{
    memcpy(pres + DWZ_ADDR, dwz_info, sizeof(DWZ_INFO_T));
}


/*******************************************************************************
 * fusing flash
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
ist_fuse_ucode(struct i2c_client *client, u8 *code, unsigned int len)
{
    s32 ret;
    s32 retry = 3;
    u16 b,c,d,v;
    DWZ_INFO_T bin_dwz_info;

    get_dwz_from_binary(code, &bin_dwz_info);

    b = bin_dwz_info.ver_b;
    c = bin_dwz_info.ver_c;
    d = bin_dwz_info.ver_d;
    v = bin_dwz_info.ver_v;
    bin_dwz_info.ver_b = 0;  // temporary reset the version
    bin_dwz_info.ver_c = 0;
    bin_dwz_info.ver_d = 0;
    bin_dwz_info.ver_v = 0;

    set_dwz_from_binary(code, &bin_dwz_info);

    //-------------------------------------------
    ist_reset_ic();

    //-------------------------------------------
    // chip specific code for flash fuse

#ifdef PGM_VERIFY
    while(retry--)
    {
        ret = ist_program_nvm(client, code, len, 0);
        if(ret != 0)
            return ret;
        ret = ist_verify_nvm (client, code, len, 0);
        if(ret == 0)
            break;
        ISTCORE_ERR(" download uc failed(%d)\n", 3-retry);
    }
    if(retry != 0)
    {
        ISTCORE_INFO("download uc success\n");
    }
#else
    ret = ist_program_nvm(client, code, len, 0);
    if(ret != 0)
        return ret;
#endif
    bin_dwz_info.ver_b = b;  // restore the version.
    bin_dwz_info.ver_c = c;
    bin_dwz_info.ver_d = d;
    bin_dwz_info.ver_v = v;

    set_dwz_from_binary(code, &bin_dwz_info);
    //-------------------------------------------
    ist_reset_ic();
    //-------------------------------------------
    // chip dwz fuse
#ifdef PGM_VERIFY
    while(retry--)
    {
        ret = ist_program_nvm(client, code, BOOT_OFFS-0x280,0x280);
        if(ret != 0)
            return ret;
        ret = ist_verify_nvm (client, code, BOOT_OFFS-0x280,0x280);
        if(ret == 0)
            break;
        ISTCORE_ERR(" download version info failed(%d)\n", 3-retry);
    }
    if(retry != 0)
    {
        ISTCORE_INFO("download version info success\n");
    }
#else
    ret = ist_program_nvm(client, code, BOOT_OFFS-0x280,0x280);
    if(ret != 0)
        return ret;
#endif
    //--------------------------------
    // reset..
    ist_sw_reset(client, 1000);                                 // enable wdr reset

    //-------------------------------------------

    return ret;
}


/*------------------------------------------------------------------------------
 * update checking - whether it requires or not.
 *-----------------------------------------------------------------------------*/
static
int is_update_necessary(struct ist510e *ts, unsigned char *pres)
{
	int ret;
	int ver_cmp = 0;
    DWZ_INFO_T bin_dwz_info;

	get_dwz_from_binary(pres, &bin_dwz_info);

        //---------------------------------
    ISTCORE_INFO("binary boot version   : %04x\n", bin_dwz_info.ver_b);
    ISTCORE_INFO("binary core version   : %04x\n", bin_dwz_info.ver_c);
    ISTCORE_INFO("binary custom version : %04x\n", bin_dwz_info.ver_d);
    ISTCORE_INFO("binary vr version     : %04x\n", bin_dwz_info.ver_v);

    ver_cmp |= (ts->dwz_info.ver_b < bin_dwz_info.ver_b);
    ver_cmp |= ((ts->dwz_info.ver_c < bin_dwz_info.ver_c) << 1);
    ver_cmp |= ((ts->dwz_info.ver_d < bin_dwz_info.ver_d) << 2);
    ver_cmp |= ((ts->dwz_info.ver_v < bin_dwz_info.ver_v) << 3);

    if (ver_cmp != 0)
    {
        ISTCORE_INFO("the version information is not same, f/w update necessary : %04x\n", ver_cmp);
    }

	return ver_cmp;
}

/*------------------------------------------------------------------------------
 * flash programming sequence
 *-----------------------------------------------------------------------------*/
int
ist_load_ucode(struct device *dev, const char *fn, int mode)
{
    struct ist510e *ts_drv = dev_get_drvdata(dev);
    const struct firmware *fw_entry;
    unsigned char *fw_buf;
    unsigned int fw_length;
    int update_index;
    s32 ret;

    //-------------------------------------------
    // load binary from user space
    ret = request_firmware(&fw_entry, fn, dev);
    if(ret != 0)
    {
        dev_err(dev, "request_firmware : fail(%d)\n", ret);
        return ret;
    }
    fw_length = (unsigned int)fw_entry->size;
	ISTCORE_INFO("LEN = %d", fw_length);
    fw_buf = kmalloc(fw_length, GFP_KERNEL);
    if(!fw_buf){
        ISTCORE_ERR("can't malloc\n");
        goto ist_load_ucode_exit;
    }
    memcpy(fw_buf,(unsigned char *)fw_entry->data,fw_length);
    ret = is_update_necessary(ts_drv, fw_buf);
    if((!ts->manually_update)&&(false == ret))
    {
        //no need to update firmware, because the version between firmware and binary is the same.
        ISTCORE_INFO("no need to update.");
        goto ist_load_ucode_exit;
    }
    mutex_lock(&ts_drv->i2c_mutex);
    ts_drv->dev_state = ISTCORE_PWR_PGM  ;
    ret = ist_fuse_ucode(ts_drv->client, fw_buf, fw_length);
    ts_drv->dev_state =ISTCORE_PWR_NORMAL;
    mutex_unlock(&ts_drv->i2c_mutex);
ist_load_ucode_exit:
    release_firmware(fw_entry);
    if(fw_buf)
        kfree(fw_buf);
    return ret;
}

/*------------------------------------------------------------------------------
 *  get dwz info. from touch ic
 *-----------------------------------------------------------------------------*/
int
ist_load_dwz(struct ist510e *ts)
{
    s32 ret = 0;
    int retry = 4;
    //------------------------------------------
    // enter pgm
    while(retry--)
    {
        ISTCORE_DBG("enter_pgm : %d\n", retry);
        ret = ist_enter_pgm(ts->client);
        if(ret >= 0)
            break;
        //do not reset after entering pgm mode.
        //ist_sw_reset(client, 10);
    }
    if(retry <= 0)
    {
        ISTCORE_ERR("enter_pgm : failed\n");
        return -1;
    }

    mdelay(50);

    ret = pgm_r_mem(ts->client, DWZ_ADDR, &packet_r, sizeof(DWZ_INFO_T));
    if(ret < 0)
        goto i2c_err;

    memcpy((u8*)&ts->dwz_info, packet_r.payload, sizeof(DWZ_INFO_T));

    ist_sw_reset(ts->client, 10);

    //---------------------------------
    ISTCORE_INFO("firmware boot version   : %04x\n", ts->dwz_info.ver_b);
    ISTCORE_INFO("firmware core version   : %04x\n", ts->dwz_info.ver_c);
    ISTCORE_INFO("firmware custom version : %04x\n", ts->dwz_info.ver_d);
    ISTCORE_INFO("firmware vr version     : %04x\n", ts->dwz_info.ver_v);
    ISTCORE_INFO("factory ID              : %04x\n", ts->dwz_info.factory_id);

    mdelay(50);

    return 0;
i2c_err:
    return -1;
}
