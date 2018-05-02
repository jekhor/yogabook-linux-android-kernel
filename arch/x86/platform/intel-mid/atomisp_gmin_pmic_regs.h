#ifndef __ATOMISP_GMIN_PMIC_REGS__
#define __ATOMISP_GMIN_PMIC_REGS__

#define VPROG_2P8V 0x5D
#define VPROG_1P8V 0x57
#define VPROG_ENABLE 0x2
#define VPROG_DISABLE 0x0
#define VPROG4D 0x09f
#define VPROG4D_VSEL 0xcf
#define VPROG5B 0xA1
#define VPROG5B_VSEL 0xD1
#define VPROG_1P2SX 0x5A
#define VPROG_1P2SX_VSEL 0xC3
#define VPROG_1P2A 0x59
#define VPROG_1P2A_VSEL 0xC4
#define VPROG_3P3A 0x9A
enum camera_pmic_pin {
        CAMERA_1P8V,
        CAMERA_2P8V,
        CAMERA_VPROG4D,
        CAMERA_VPROG5B,
        CAMERA_1P2SX,
        CAMERA_1P2A,
		CAMERA_3P3A,
        CAMERA_POWER_NUM,
};

int camera_set_pmic_power(enum camera_pmic_pin pin, bool flag);


#endif
