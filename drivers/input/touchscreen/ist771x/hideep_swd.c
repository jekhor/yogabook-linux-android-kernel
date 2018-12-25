#include "hideep.h"
#include "hideep_swd.h"


tComponentID	g_cmpid;
tPeripheralID   g_perid;

tComponentID	g_compbase_cmpid[128];
tPeripheralID   g_compbase_perid[128];
unsigned int g_u32compbase[128] = {0,};

void hideep_gpio_i2c_start(void)
{
	//dbg_fun();
	hideep_gpio_i2c_SDA(1);
	hideep_gpio_i2c_SCL(1);
	udelay(10);
	hideep_gpio_i2c_SDA(0);
	udelay(10);
	hideep_gpio_i2c_SCL(0);
	udelay(10);
	return;
}
void hideep_gpio_i2c_stop(void)
{
	//dbg_fun();
	hideep_gpio_i2c_SDA(0);
	udelay(10);
	hideep_gpio_i2c_SCL(1);
	udelay(10);
	hideep_gpio_i2c_SDA(1);
	udelay(10);
	return;
}
int hideep_gpio_i2c_ack(void)
{
	int ret=1;
	int i=0;
	//dbg_fun();
	i=0;
	ret = hideep_gpio_i2c_SDA_GET();
	while(ret != 0){
		i++;
		if(i>10)break;
		udelay(10);
		ret = hideep_gpio_i2c_SDA_GET();
	};
	hideep_gpio_i2c_SCL(1);
	udelay(10);
	hideep_gpio_i2c_SCL(0);
	udelay(10);
	return ret;
}
void hideep_gpio_i2c_send_byte( unsigned char data)
{
	int i = 0;
	int bit=0x80;
	//dbg_fun();
	for(i = 0;i<8;i++){
		if(data & bit)
			hideep_gpio_i2c_SDA(1);
		else
			hideep_gpio_i2c_SDA(0);
		hideep_gpio_i2c_SCL(1);
		udelay(10);
		hideep_gpio_i2c_SCL(0);
		udelay(10);
		bit = bit >> 1;
	}
}

void hideep_gpio_i2c_read_byte(unsigned char *pdata)
{
	int i=0;
	int ret = 0;
	unsigned char bit = 0x80;
	unsigned char buf=0;
	//dbg_fun();
	hideep_gpio_i2c_SDA(1);
	for(i=0;i<8;i++){
		hideep_gpio_i2c_SCL(0);
		udelay(10);
		hideep_gpio_i2c_SCL(1);
		udelay(10);
		ret = hideep_gpio_i2c_SDA_GET();
		if(ret)
			buf |= bit;
		bit = bit>>1;
	}
	hideep_gpio_i2c_SCL(0);
	udelay(10);
	*pdata = buf;
}

int hideep_gpio_i2c_send_pgm( unsigned char *pdata, int len)
{
	int ret;
	int i;
	unsigned char buf;
	unsigned char bit=0;

	hideep_gpio_i2c_start();
	for(i = 0;i< len;i++){
		buf = pdata[i];
		hideep_gpio_i2c_send_byte(buf);
		ret = hideep_gpio_i2c_ack();
		if(ret)bit |= 1<<i;
	}
	hideep_gpio_i2c_stop();
	ret = bit;
	return ret;
}

int hideep_gpio_i2c_send( unsigned char addr, unsigned short reg, unsigned char *pdata, int len)
{
	int ret;
	int i;
	unsigned char buf;

	//dbg_fun();
	//addr
	hideep_gpio_i2c_start();
	buf = addr<<1;
	hideep_gpio_i2c_send_byte(buf);
	ret = hideep_gpio_i2c_ack();
	if(ret)
		goto err_no_ack;
	//reg
	buf = reg&0xff;
	hideep_gpio_i2c_send_byte(buf);
	ret = hideep_gpio_i2c_ack();
	if(ret)
		goto err_no_ack;
	buf = (reg>>8)&0xff;
	hideep_gpio_i2c_send_byte(buf);
	ret = hideep_gpio_i2c_ack();
	if(ret)
		goto err_no_ack;
	//data
	for(i = 0;i< len;i++){
		buf = pdata[i];
		hideep_gpio_i2c_send_byte(buf);
		ret = hideep_gpio_i2c_ack();
		//dbg_log("ack = %d", ret);
	}
	hideep_gpio_i2c_stop();
	return ret;
err_no_ack:
	hideep_gpio_i2c_stop();
	return ret;
}
int hideep_gpio_i2c_read( unsigned char addr, unsigned short reg, unsigned char *pdata, int len)
{
	int ret;
	int i;
	unsigned char buf;

	//dbg_fun();
	//addr
	hideep_gpio_i2c_start();
	buf = addr<<1;
	hideep_gpio_i2c_send_byte(buf);
	ret = hideep_gpio_i2c_ack();
	if(ret)
		goto err_no_ack;
	//reg
	buf = reg&0xff;
	hideep_gpio_i2c_send_byte(buf);
	ret = hideep_gpio_i2c_ack();
	if(ret)
		goto err_no_ack;
	buf = (reg>>8)&0xff;
	hideep_gpio_i2c_send_byte(buf);
	ret = hideep_gpio_i2c_ack();
	if(ret)
		goto err_no_ack;
	//data
	for(i = 0;i< len;i++){
		hideep_gpio_i2c_read_byte(&pdata[i]);
		ret = hideep_gpio_i2c_ack();
		//dbg_log("ack = %d", ret);
	}
	hideep_gpio_i2c_stop();
	return ret;
err_no_ack:
	hideep_gpio_i2c_stop();
	return ret;
}

void hideep_swd_send_bit(unsigned char bit)
{
	HIDEEP_SWD_CLK_SET(0);
	HIDEEP_SWD_DAT_SET(bit?1:0);
	HIDEEP_SWD_CLK_SET(1);
}
void hideep_swd_recv_bit(unsigned int *bit)
{
	unsigned int ret;
	HIDEEP_SWD_CLK_SET(0);
	ret = HIDEEP_SWD_DAT_GET();
	HIDEEP_SWD_CLK_SET(1);
	*bit = ret?1:0;
	//dbg_log("*bit = %d", *bit);
	return;
}

unsigned char hideep_blaster_get_cmd_content(unsigned char port, 	unsigned char addr, 	unsigned char rw)
{
	unsigned char cmd = 0x81;
	unsigned char parity = 0;
	if(port){
		cmd |= 2;
		parity++;
	}
	if(rw){
		cmd |= 4;
		parity++;
	}
	cmd |= ((addr & 0x0c) <<1);
	if(addr & 4)
		parity++;
	if(addr & 8)
		parity++;
	cmd |= ((parity&1)?1:0)<<5;
	//dbg_log("port = %d, addr = %d, rw = %d, parity = %d, cmd = 0x%02x", port, addr, rw, parity?1:0, cmd);
    return cmd;
}

unsigned char hideep_swd_data_send( unsigned int data,  unsigned int  size)
{
    unsigned int len = 0;
    unsigned char parity = 0;

    for (len = 0; len < size; len++) {
        parity ^= ((data & (1 << len) )== 0) ? 0 : 1;
        hideep_swd_send_bit ((data & (1 << len)) == 0 ? 0 : 1);
    }
    return parity;
}

unsigned char hideep_swd_data_recv( unsigned int *pdata,  unsigned int  size)
{
	unsigned int len = 0;
	unsigned char parity = 0;
	int data = 0;
	unsigned int bit;

	for (len = 0; len < size; len++) {
		hideep_swd_recv_bit(&bit);
		parity ^= (bit & 0x01) ? 0 : 1;
		data |= (bit & 0x01) << len;
	}
	*pdata = data;
	return parity;
}

unsigned char hideep_blaster_get_ack(unsigned char rw)
{
	unsigned int data;
	unsigned char ret;
	//1st one is dummy bit
	hideep_swd_data_recv(&data,1);
	//next 3 bits is ack.
	hideep_swd_data_recv(&data,3);
	ret = data & 0x7;
	//if write, send a dummy.
	if (rw == DIR_WRITE) {
		hideep_swd_data_send(1,1);
	}
	return ret;
}

void hideep_blaster_reset(void)
{
	unsigned int data = 0xFFFFFFFF;

	//dbg_fun();
	HIDEEP_SWD_DAT_SET(1);
	HIDEEP_SWD_CLK_SET(1);

	mdelay(2);
	hideep_swd_data_send(data, 32);
	hideep_swd_data_send(data, 24);
}

int hideep_blaster_set_dp_ap_register(	unsigned char port,	unsigned char target_register,	unsigned int data	)
{
	int retry = 0;
	unsigned char ack;
	unsigned char cmd;
	unsigned parity;

	//dbg_fun();
retry_to_get_ack:
	//dbg_log("retry = %d", retry);
	retry++;
	cmd = hideep_blaster_get_cmd_content(port,target_register,DIR_WRITE);
	udelay(2);
	hideep_swd_data_send(cmd,8);
	ack = hideep_blaster_get_ack(DIR_WRITE);
	//dbg_log("ack = 0x%2x", ack);
	switch (ack) {
		case ACK_NORMAL:
			//dbg_log("ack ok!");
			break;
		case ACK_WAIT:
			if( retry < 30){
				//dbg_log("wait ack retry = %d", retry);
				goto retry_to_get_ack;
			}
			else{
				goto error;
			}
			break;
		case ACK_FAULT:
			if(retry > 3){
				//dbg_log("ack fault retry = %d", retry);
				goto error;
			}
			hideep_blaster_reset();
			mdelay(1);
			goto retry_to_get_ack;
		default:
			goto error;
	}
	parity = hideep_swd_data_send(data,32);
	hideep_swd_data_send(parity,1);
	udelay(2);
	//dbg_log("success!");
	return 0;
error:
	hideep_blaster_reset();
	mdelay(2);
	return -1;
}

int hideep_blaster_get_dp_ap_register(unsigned char port, 	unsigned char target_register, 	unsigned int *pdata, unsigned char idle)
{
	int retry = 0;
	unsigned int ret=0;
	unsigned char ack=0;
	unsigned char cmd=0;

	//dbg_fun();
	retry = 0;
retry_to_get_ack:
	retry++;
	//dbg_log("retry = %d", retry);
	cmd = hideep_blaster_get_cmd_content(port,target_register,DIR_READ);
	udelay(2);
	if (idle)
		hideep_swd_data_send(0,8);

	hideep_swd_data_send(cmd,8);
	ack = hideep_blaster_get_ack(DIR_READ);
	//dbg_log("ack = 0x%2x", ack);
	switch (ack) {
		case ACK_NORMAL:
			//dbg_log("ack ok!");
			break;
		case ACK_WAIT:
			if( retry < 30){
				//dbg_log("wait ack retry = %d", retry);
				goto retry_to_get_ack;
			}
			else
				goto error;
			break;
		case ACK_FAULT:
			if(retry > 3){
				//dbg_log("ack fault retry = %d", retry);
				goto error;
			}
			hideep_blaster_reset();
			mdelay(1);
			goto retry_to_get_ack;
		default:
			goto error;
	}
	hideep_swd_data_recv(pdata,32);
	hideep_swd_data_recv(&ret,1);
	hideep_swd_data_send(1,1);
	udelay(2);
	return 0;
error:
	hideep_blaster_reset();
	mdelay(10);
	return -1;
}

void hideep_blaster_jtag_swd(	void )
{
	unsigned short flag = 0xE79E;
	hideep_swd_data_send(flag, 16);
}

int  hideep_blastr_get_debug_base_address(unsigned int *pdata)
{
	int ret = 0;
	//dbg_fun();
	ret = hideep_blaster_set_dp_ap_register(PORT_DP, RDP_SELECT, RAP_DBGBASE & 0xF0);	//Set SELECT[31:24]
	if(ret != 0)
		goto error;
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DBGBASE & 0x0F, pdata, 0);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_get_dp_ap_register(PORT_DP, RDP_RDBUFF, pdata, 0);  //0xE00FF003
	if(ret != 0)
		goto error;
	*pdata = *pdata&0xFFFFF000;
	return ret;
error:
	dbg_err();
	return ret;
}
int hideep_blaster_get_component_base_address(unsigned int * pid, unsigned int addr, unsigned int index)
{
	unsigned int temp;
	int ret = 0;
	////dbg_fun();
	temp = (addr | OFFSET_ROMTABLE) + (index << 2);
	ret = hideep_blaster_set_dp_ap_register(PORT_AP, RAP_TAR, temp);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_get_dp_ap_register(PORT_DP, RDP_RDBUFF,&temp, 0);
	if(ret != 0)
		goto error;
	//dbg_log("temp = 0x%x", temp);
	if (temp == 0x00000000){
		*pid = 0;
		return 0;
	}
	//2's complement
	temp &= 0xFFFFF000;
	temp = ~temp;
	temp += 0x01000;
	temp ^= addr;
	*pid = temp & 0xFFFFF000;
	return ret;
error:
	dbg_err("ret = %d",ret);
	return ret;

}

int GetPeripheralID(tPeripheralID *ppid, unsigned int debug_base_addr )
{
	tPeripheralID	perid;
	unsigned int temp = 0;
	int ret;

	//dbg_fun();
	memset(&perid, 0, sizeof(tPeripheralID));

	//BASE Address + 0x00000FD0  //Pheripheral ID registers
	ret = hideep_blaster_set_dp_ap_register( PORT_AP, RAP_TAR, debug_base_addr | OFFSET_PERIID);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x00000000	 //the value of PERIID4
	if(ret != 0)
		goto error;
	perid.u32hiword |= (temp & 0xFF);
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x00000004   //the value of PERIID5
	if(ret != 0)
		goto error;
	perid.u32hiword |= ((temp & 0xFF) << 8);
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x00000000   //the value of PERIID6
	if(ret != 0)
		goto error;
	perid.u32hiword |= ((temp & 0xFF) << 16);
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x00000000   //the value of PERIID7
	if(ret != 0)
		goto error;
	perid.u32hiword |= ((temp & 0xFF) << 24);
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x00000000   //the value of PERIID0
	if(ret != 0)
		goto error;
	perid.u32loword |= (temp & 0xFF);
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x00000071   //the value of PERIID1
	if(ret != 0)
		goto error;
	perid.u32loword |= ((temp & 0xFF) << 8);
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x000000B4   //the value of PERIID2
	if(ret != 0)
		goto error;
	perid.u32loword |= ((temp & 0xFF) << 16);
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x0000000B   //the value of PERIID3
	if(ret != 0)
		goto error;
	perid.u32loword |= ((temp & 0xFF) << 24);
	ret = hideep_blaster_get_dp_ap_register(PORT_DP, RDP_RDBUFF, &temp, 0);  //0x00000000
	if(ret != 0)
		goto error;
	perid.u16partnum = perid.u32loword & 0xFFF;
	if (perid.u32loword & 0x80000) {
		perid.u16idcodeJEP106 = ((perid.u32loword & 0x0007F000) >> 12) & 0xFFFF;
		perid.u16idcodeJEP106 |= (perid.u32hiword & 0x0F) << 12;
	}

	perid.u8revision = ((perid.u32loword & 0xF00000) >> 20) & 0xFF;
	perid.u8customer = ((perid.u32loword & 0xF000000) >> 24) & 0xFF;
	perid.u8revand = ((perid.u32loword & 0xF0000000) >> 28) & 0xFF;
	perid.u8cnt4kb = ((perid.u32hiword & 0xF0) >> 4) & 0xFF;

	*ppid = perid;
	return ret;
error:
	dbg_err("ret = %d",ret);
	return ret;
}

int GetComponentID(tComponentID *pcid, unsigned int debug_base_addr )
{
	tComponentID	cmpid;
	unsigned int temp = 0;
	int ret = 0;

	//dbg_fun();
	cmpid.u8compclass = 0;
	cmpid.u32preamble = 0;
	//BASE Address + 0x00000FF0  //Component ID registers
	ret = hideep_blaster_set_dp_ap_register( PORT_AP, RAP_TAR, debug_base_addr | OFFSET_COMPID);
	if(ret != 0)
		goto error;
	//dbg_log("DBG Base 0x%08x.\n", debug_base_addr);
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x0000000D	 //the value of COMPID0
	if(ret != 0)
		goto error;

	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x00000010
	if(ret != 0)
		goto error;
	cmpid.u32preamble |= (temp & 0xFF);
	//dbg_log("COMPID0 0x%08x.\n", temp);

	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x00000005	 //the value of COMPID2
	if(ret != 0)
		goto error;
	cmpid.u32preamble |= ((temp & 0x0F) << 8);
	cmpid.u8compclass = ((temp & 0xF0) >> 4);
	//dbg_log("COMPID1 0x%08x.\n", temp);

	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, &temp, 0);  //0x000000B1	 //the value of COMPID3
	if(ret != 0)
		goto error;
	cmpid.u32preamble |= ((temp & 0xFF) << 16);
	//dbg_log("COMPID2 0x%08x.\n", temp);


	ret = hideep_blaster_get_dp_ap_register(PORT_DP, RDP_RDBUFF, &temp, 0);
	if(ret != 0)
		goto error;
	cmpid.u32preamble |= ((temp & 0xFF) << 24);
	//dbg_log("COMPID3 0x%08x.\n", temp);
	*pcid = cmpid;
	return ret;
error:
	dbg_err("ret = %d",ret);
	return ret;
}

int CM0_RunControl( unsigned int compaddr,  unsigned int  action )
{
	unsigned int   temp = 0;
	int ret = 0;
		//dbg_fun();
	//DHCSR Debug Halting Control and Status Register in the ARMv6-M ARM
	ret = hideep_blaster_set_dp_ap_register(PORT_AP, RAP_TAR, compaddr | DHCSR_OFFSET);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_set_dp_ap_register(PORT_AP, RAP_DRW, action);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_get_dp_ap_register(PORT_DP, RDP_RDBUFF,&temp,0);
	if(ret != 0)
		goto error;
	return ret;
error:
	dbg_err("ret = %d",ret);
	return ret;
}

int hideep_swd_read(unsigned int addr, unsigned int *pdata)
{
	int ret = 0;
	ret = hideep_blaster_set_dp_ap_register(PORT_AP, RAP_TAR, addr);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_get_dp_ap_register(PORT_AP, RAP_DRW, pdata, 0);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_get_dp_ap_register(PORT_DP, RDP_RDBUFF, pdata, 0);
	if(ret != 0)
		goto error;
	return ret;
error:
	dbg_err("ret = %d",ret);
	return ret;
}
int hideep_swd_write(unsigned int addr, unsigned int data)
{
	int ret = 0;
	ret = hideep_blaster_set_dp_ap_register(PORT_AP, RAP_TAR, addr);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_set_dp_ap_register(PORT_AP, RAP_DRW, data);
	if(ret != 0)
		goto error;
	HIDEEP_SWD_DAT_SET(1);
	return ret;
error:
	dbg_err("ret = %d",ret);
	return ret;
}

int hideep_blaster_init(void)
{
	int ret;
	unsigned int code;
	unsigned int temp;
	unsigned int debug_base_addr;
	unsigned int i;
	unsigned char buf[5];
	unsigned int ack=0;
	
	//dbg_fun();
	private_ts->pdata->reset();
	mdelay(50);
	buf[0] = 0x6c<<1;
	buf[1] = 0x39;
	ret = hideep_gpio_i2c_send_pgm(buf,2);
	ack = ret;
	mdelay(1);
	buf[1] = 0xaf;
	ret = hideep_gpio_i2c_send_pgm(buf,2);
	ack = ack + (ret<<2);
	mdelay(1);
	buf[1] = 0x9d;
	ret = hideep_gpio_i2c_send_pgm(buf,2);
	ack = ack + (ret<<4);
	mdelay(1);
	buf[1] = 0xdf;
	ret = hideep_gpio_i2c_send_pgm(buf,2);
	ack = ack + (ret<<6);
	mdelay(1);

	ret =0;
	hideep_blaster_reset();
	udelay(10);
	hideep_blaster_jtag_swd();
	udelay(10);
	hideep_blaster_reset();

	ret = hideep_blaster_get_dp_ap_register(PORT_DP, RDP_IDCODE, &code, 1);
	if(ret != 0)
		goto error;
	if (code != 0x0bb11477) {
		dbg_log("ID CODE was not correct.%x",code);
		ret = -1;
		goto error;
	}
	udelay(10);
	ret = hideep_blaster_set_dp_ap_register(PORT_DP, RDP_ABORT,SB_CLEAR_FLAG);
	if(ret != 0)
		goto error;
	//APSEL to 0
	ret = hideep_blaster_set_dp_ap_register(PORT_DP, RDP_SELECT, BANKSEL_CTRLSTAT);
	if(ret != 0)
		goto error;
	udelay(10);
	ret = hideep_blaster_set_dp_ap_register(PORT_DP,RDP_CTRLSTAT, 0x50000F00);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_set_dp_ap_register(PORT_AP, RAP_CSW, 0x23000050 | CSW_WORD);
	if(ret != 0)
		goto error;
	ret = hideep_blaster_get_dp_ap_register(PORT_DP, RDP_RDBUFF, &temp, 0);
	if(ret != 0)
		goto error;
	dbg_log("temp = 0x%8x",temp);
	ret = hideep_blastr_get_debug_base_address(&debug_base_addr);
	if(ret != 0)
		goto error;
	dbg_log("debug_base_addr = 0x%8x",debug_base_addr);
	//5. Write 0x00000000 to SELECT (select AHB-AP, bank 0x0)
	ret = hideep_blaster_set_dp_ap_register(PORT_DP, RDP_SELECT, 0x00000000);
	if(ret != 0)
		goto error;

	ret = GetComponentID(&g_cmpid, debug_base_addr);
	if(ret != 0)
		goto error;
	ret = GetPeripheralID(&g_perid, debug_base_addr);
	if(ret != 0)
		goto error;

	if (g_cmpid.u8compclass != 0) {
    		dbg_log("found sub component");
		for (i = 0; i < 10/*0x3C0*/; i++) {
			ret = hideep_blaster_get_component_base_address(&g_u32compbase[i],debug_base_addr, i);
			if(ret != 0)
				goto error;
			printk("sub component [%d] Addr (0x%08x)", i, g_u32compbase[i]);
			if (g_u32compbase[i] == 0) break;
			ret = GetComponentID(&g_compbase_cmpid[i], g_u32compbase[i]);
			if(ret != 0)
				goto error;
			ret = GetPeripheralID(&g_compbase_perid[i], g_u32compbase[i]);
			if(ret != 0)
				goto error;
		}
	}
	CM0_RunControl(g_u32compbase[0], DHCSR_STOP);
	dbg_log("PGM mode success!");
	return ret;
error:
	dbg_err("ret = %d",ret);
	return ret;
}

void hideep_blaster_exit(void)
{
	return;
}
int hideep_swd_init(void)
{
	int ret = 0;
	ret = hideep_blaster_init();
	if(ret != 0){
		goto hideep_swd_init_exit;
	}
hideep_swd_init_exit:
	return ret;
}
int hideep_swd_exit(void	)
{
	int ret = 0;

	return ret;
}
