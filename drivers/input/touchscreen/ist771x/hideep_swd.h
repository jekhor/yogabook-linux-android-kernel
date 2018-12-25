/*
 * hideep_swd.h
 *
 *  Created on: 2014-10-10
 *      Author: kim
 */

#ifndef HIDEEP_SWD_H_
#define HIDEEP_SWD_H_
//*****************************************************************************
//
//! - SWCLK - PD2
//! - SWDIO - PD3
//
//*****************************************************************************

//*****************************************************************************
//	  LATCH RULE
//! - CLK LOW - DATA Ready
//! - CLK HIGH - LATCH
//
//*****************************************************************************

//*****************************************************************************
//	  SEQ RULE  (Little endian)
//! - WRITE DATA - LSB
//! - READ DATA - LSB
//
//*****************************************************************************
#define	_YRAM_BASE		0x40000000
#define _EEPROM_MEM_BASE	0x00000000 // REV1
#define _EEPROM_MEM_SIZE	0xC000 // 48kB
#define _EEPROM_INF_BASE	_EEPROM_MEM_BASE + _EEPROM_MEM_SIZE
#define _EEPROM_INF_SIZE	0x400 // 1kB

#define	EEPROM_NORMAL_MODE	0x00
#define	TM_SFR_WRITE		0x01
#define	TM_SFR_READ		0x02
#define	TM_MCCM			0x03
#define	TM_RCCM			0x04
#define	TM_SPP_RWE		0x06
#define	TM_SCCM			0x0B

#define NVM_WRONLY    	0x0
#define NVM_PROG      	0x1
#define	MERASE		0x00010000
#define	SERASE		0x00020000
#define	PERASE		0x00040000
#define	PROG		0x00080000
#define	WRONLY		0x00100000
#define	INF		0x00200000

//------------------------------------------------------------------------------------------------------------------
// SFR/SPP Table
//------------------------------------------------------------------------------------------------------------------
#define _DISPUMP     (1 << 22)
#define _TVEEEN     (1 << 24)
#define _TVPPEN     (1 << 25)
#define _PROT_MODE  ( _TVPPEN | _TVEEEN | _DISPUMP )
//------------------------------------------------------------------------------------------------------------------

					
#define	PERIPHERAL_BASE				0x50000000
#define	FLASH_BASE				(PERIPHERAL_BASE + 0x01000000)						
#define	NVM_FLASH_CON_8				(FLASH_BASE + 0x0000)	
#define	NVM_FLASH_STA_32			(FLASH_BASE + 0x0004)
#define	NVM_FLASH_CFG_8				(FLASH_BASE + 0x0008)
#define	NVM_FLASH_TIM_8				(FLASH_BASE + 0x000C)
#define	SYSCON_BASE				(PERIPHERAL_BASE + 0x02000000)
//*****************************************************************************
//
//! Component/Peripheral ID Defines
//
//*****************************************************************************
#define OFFSET_COMPID	0xFF0
#define OFFSET_PERIID	0xFD0
#define OFFSET_MEMTYPE	0xFCC
#define OFFSET_ROMTABLE	0x000

#define COMPID0	0xFF0
#define COMPID1	0xFF4
#define COMPID2	0xFF8
#define COMPID3	0xFFC

#define PHERID0	0xFE0
#define PERIID1	0xFE4
#define PERIID2	0xFE8
#define PERIID3	0xFEC
#define PERIID4	0xFD0
#define PERIID5	0xFD4
#define PERIID6	0xFD8
#define PERIID7	0xFDC

#define COMPCLASS_GENERICVERIFY	0x00
#define COMPCLASS_ROM_TABLE		0x01
#define COMPCLASS_DEBUG			0x09
#define COMPCLASS_PTB			0x0B
#define COMPCLASS_DESS			0x0D
#define COMPCLASS_GENERICIP		0x0E
#define COMPCLASS_PRIMECELL		0x0F

#define PORT_DP		0
#define PORT_AP		1
#define DIR_WRITE	0
#define DIR_READ	1

#define ACK_NORMAL 	1
#define ACK_WAIT	2
#define ACK_FAULT	4

//Debug Port Register
#define	RDP_IDCODE		0x0			//READ ONLY
#define	RDP_ABORT		0x0			//WRITE ONLY
#define	RDP_CTRLSTAT		0x4			//R/W
#define	RDP_SELECT		0x8		    //WRITE ONLY
#define	RDP_RESEND		0x8			//READ ONLY
#define	RDP_RDBUFF		0xc		    //READ ONLY
#define CSYSPWRUPACK	0x80000000
#define CSYSPWRUPREQ	0x40000000
#define CDBGPWRUPACK	0x20000000
#define CDBGPWRUPREQ	0x10000000

//AHB-AP Register
#define RAP_CSW				0x00
#define RAP_TAR				0x04
#define RAP_DRW				0x0c
#define RAP_DBGBASE			0xF8
#define RAP_IDR				0xFC

#define CSW_BYTE		0
#define CSW_HALFWORD	1
#define CSW_WORD		2

#define IDLE_DAT		0x00
#define SB_CLEAR_FLAG 	0x0000001E
#define BANKSEL_CTRLSTAT 			0x0
#define BANKSEL_DATALINK_CTRL 		0x1
#define BANKSEL_TARGETID 			0x2
#define BANKSEL_DATALINK_PROTOCOLID 0x3

#define CPUID_OFFSET	0x00000D00
#define AIRCR_OFFSET	0x00000D0C
#define DHCSR_OFFSET	0x00000DF0
#define DEMCR_OFFSET	0x00000DFC

#define DHCSR_RUN	0xA05F0001
#define DHCSR_STOP	0xA05F0003
#define DHCSR_STEP	0xA05F0005


typedef struct {
    unsigned char u8compclass;
    unsigned int u32preamble;
} tComponentID;

//*****************************************************************************
//
//! This structure contains the state of a single instance of a PeripheralID module.
//
//*****************************************************************************
typedef struct {
    unsigned short u16partnum;  //b11:0
    unsigned char u8usedJEP106;
    unsigned short u16idcodeJEP106;
    unsigned char u8revision;  //b23:20
    unsigned char u8customer;  //b27:24
    unsigned char u8revand;  //b31:28
    unsigned char u8cnt4kb;  //b39:36
    unsigned int u32loword;
    unsigned int u32hiword;
} tPeripheralID;

void hideep_swd_send_bit(unsigned char bit);
void hideep_swd_recv_bit(unsigned int *bit	);
unsigned char hideep_blaster_get_cmd_content(unsigned char port, 	unsigned char addr, 	unsigned char rw);
unsigned char hideep_swd_data_send( unsigned int data,  unsigned int  size);
unsigned char hideep_swd_data_recv( unsigned int *pdata,  unsigned int  size);
unsigned char hideep_blaster_get_ack(	unsigned char rw);
void hideep_blaster_reset(void);
int hideep_blaster_set_dp_ap_register(	unsigned char port,	unsigned char target_register,	unsigned int data	);
int hideep_blaster_get_dp_ap_register(unsigned char port, 	unsigned char target_register, 	unsigned int *pdata, unsigned char idle);
void hideep_blaster_jtag_swd(	void );
int  hideep_blastr_get_debug_base_address(unsigned int *pdata);
int GetPeripheralID(tPeripheralID *ppid, unsigned int debug_base_addr );
int GetComponentID(tComponentID *pcid, unsigned int debug_base_addr );
int hideep_blaster_get_component_base_address(unsigned int * pid, unsigned int addr, unsigned int index);
int CM0_RunControl( unsigned int compaddr,  unsigned int  action );
int hideep_swd_read(unsigned int addr, unsigned int *pdata);
int hideep_swd_write(unsigned int addr, unsigned int data);
int hideep_blaster_init(void);
void hideep_blaster_exit(void);
int hideep_swd_init(void);
int hideep_swd_exit(void);



#endif /* HIDEEP_SWD_H_ */
