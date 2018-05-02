
struct fm_ld_drv_register {
    /**
    * @priv_data: priv data holder for the FMR driver. This is filled by
    * the FMR driver data during registration, and sent back on
    * ld_drv_reg_complete_cb and cmd_handler members.
    */
void *priv_data;

    /**
    * @ ld_drv_reg_complete_cb: Line Discipline Driver registration complete
    * callback.
    *
    * This function is called by the Line discipline driver when
    * registration is completed.
    *
    * Arg parameter is the fm driver private data
    * data parameter is status of LD registration In
    * progress/Registered/Unregistered.
    */
void (*ld_drv_reg_complete_cb)(void *arg, char data);

    /**
    * @fm_cmd_handler: BT Cmd/Event and IRQ informer
    *
    * This function is called by the line descipline driver, to signal the FM
    * Radio driver on host when FM packet is available. This includes command
    * complete and IRQ forwarding. However the processing of the buff contents,
    * do not happen in the LD driver context.
    *
    * arg parameter is the fm driver private data
    * data parameter is status of LD registration In
    * Progress/Registered/Unregistered.
    */
long (*fm_cmd_handler)(void *arg, struct sk_buff *);

    /**
    * @fm_cmd_write: FM Command Send function
    *
    * This field should be populated by the LD driver. This function is used by
    * FM Radio driver to send the command buffer
    */
long (*fm_cmd_write)(struct sk_buff *skb);
};


extern long register_fmdrv_to_ld_driv
		(struct fm_ld_drv_register *fm_ld_drv_reg);

extern long unregister_fmdrv_from_ld_driv
		(struct fm_ld_drv_register *fm_ld_drv_reg);

/*-------------MACRO---------------*/

#define LPM_PKT	0xF1

/*---- HCI data types ------------*/
#define HCI_COMMAND_PKT 0x01
#define HCI_ACLDATA_PKT 0x02
#define HCI_SCODATA_PKT 0x03
#define HCI_EVENT_PKT   0x04
#define PM_PKT		0x00

/* ---- HCI Packet structures ---- */
#define HCI_COMMAND_HDR_SIZE	3
#define HCI_EVENT_HDR_SIZE	2
#define HCI_ACL_HDR_SIZE	4
#define HCI_SCO_HDR_SIZE	3

#define INVALID -1
#define HCI_COMMAND_COMPLETE_EVENT		0x0E
#define HCI_COMMAND_HARDWARE_ERROR		0x10
#define HCI_INTEL_BOOTUP_EVENT			0xFF
#define HCI_FATAL_EXCEPTION			0xFF
#define HCI_DEBUG_EXCEPTION			0xFF
#define HCI_INTEL_BOOTUP_EVENT_ID		0x02
#define HCI_INTEL_BOOTUP_REASON_HCI_RESET	0x00
#define HCI_INTEL_BOOTUP_REASON_HW_RESET	0x01
#define HCI_INTEL_BOOTUP_REASON_INTEL_RESET	0x02
#define HCI_INTEL_BOOTUP_REASON_WATCHDOG_EXCEP	0x03
#define HCI_INTEL_BOOTUP_REASON_FATAL_EXCEP	0x04
#define HCI_INTEL_BOOTUP_REASON_SYSTEM_EXCEP	0x05
#define HCI_INTEL_BOOTUP_REASON_UNKNW_EXCEP	0xFF
#define HCI_FATAL_EXCEPTION_EVENT_ID		0x01
#define HCI_DEBUG_EXCEPTION_EVENT_ID		0x08
#define HCI_INTERRUPT_EVENT			0xFF
#define HCI_COMMAND_STATUS_EVT			0x0F
#define FMR_DEBUG_EVENT_ID			0x2B
#define OPERATIONAL				0x01
#define BOOTLOADER				0x00

/* ---- PM PKT RESP ---- */
#define ACK_RSP			0x02
#define WAKE_UP_RSP		0x03
#define TX_HOST_NOTIFICATION	0x00

/*-----------D States-----------*/

#define D0			0
#define D2			1
#define D3			2
#define D0_TO_D2		3
#define D2_TO_D0		4

/*-----------System States-----*/
#define S3			1
#define S0			0

/*----------- FMR Command ----*/
#define FMR_WRITE       0xFC58
#define FMR_READ	0xFC59
#define FMR_SET_POWER   0xFC5A
#define FMR_SET_AUDIO   0xFC5B
#define FMR_IRQ_CONFIRM 0xFC5C
#define FMR_TOP_WRITE   0xFC5D
#define FMR_TOP_READ    0xFC5E

/*-----------BT CMD----------*/
#define BT_HCI_RESET	0x0C03
#define BT_SECURE_SEND	0xFC09

#define HCI_RESET_COMPL_RCV	2
#define HCI_RESET_SEND		1
#define HCI_REST_CMD		0
#define HCI_CMD_TOUT		0
#define HCI_HW_ERR			1


#define STREAM_TO_UINT16(u16, p) { (p) += 4; u16 = ((unsigned int)(*(p)) + \
					(((unsigned int)(*((p) + 1))) << 8)); }

#define STREAM_TO_UINT16_OPCODE(u16, p) { (p) += 1; u16 = \
				((unsigned int)(*(p)) + \
				(((unsigned int)(*((p) + 1))) << 8)); }
#define MAX_BT_CHNL_IDS 4

/* ----- HCI receiver states ---- */
#define LBF_W4_H4_HDR   0
#define LBF_W4_PKT_HDR  1
#define LBF_W4_DATA     2

#define N_INTEL_LDISC_BUF_SIZE		4096
#define TTY_THRESHOLD_THROTTLE		128 /* now based on remaining room */
#define TTY_THRESHOLD_UNTHROTTLE	128

#define BT_FW_DOWNLOAD_INIT		_IOR('L', 1, uint64_t)
#define BT_FW_DOWNLOAD_COMPLETE		_IOW('L', 2, uint64_t)
#define BT_FMR_IDLE			_IOW('L', 3, uint64_t)
#define BT_FMR_LPM_ENABLE		_IOW('L', 4, uint64_t)
#define BT_FMR_HW_RESET			_IOW('L', 5, uint64_t)
#define BT_HOST_WAKE_INTR_COUNT		_IOW('L', 6, uint64_t)
#define BT_HOST_WAKE_INTR_COUNT_RESET	_IOW('L', 7, uint64_t)
#ifdef TEST_CODE
#define BT_FMR_HW_ERR			_IOW('L', 8, uint64_t)
#define BT_FMR_BOOTUP_ERR		_IOW('L', 9, uint64_t)
#endif

enum {
	LOW,
	HIGH
};

enum {
	MODULE_HW_ERR,
	MODULE_CMD_TIMEOUT
};

#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

#define ENABLE  1
#define DISABLE 0
#define IDLE	0
#define ACTIVE	1
