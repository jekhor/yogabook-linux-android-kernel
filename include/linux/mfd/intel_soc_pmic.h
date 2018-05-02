/*
 * intel_soc_pmic.h - Intel SoC PMIC Driver
 *
 * Copyright (C) 2012-2014 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Yang, Bin <bin.yang@intel.com>
 */

#ifndef __INTEL_SOC_PMIC_H__
#define __INTEL_SOC_PMIC_H__

#include <linux/gpio.h>

#define	INTEL_PMIC_IRQBASE	456
#define INTEL_NGPIO_SCORE	102
#define INTEL_NGPIO_NCORE	28

#define UPDATE_TYPE		(1 << 0)
#define UPDATE_MASK		(1 << 1)

#define GPIO0IRQ		0x0b
#define GPIO1IRQ		0x0c
#define MGPIO0IRQS0		0x19
#define MGPIO1IRQS0		0x1a
#define MGPIO0IRQSX		0x1b
#define MGPIO1IRQSX		0x1c
#define GPIO0P0CTLO		0x2b
#define GPIO0P0CTLI		0x33
#define GPIO1P0CTLO		0x3b
#define GPIO1P0CTLI		0x43

#define CTLI_INTCNT_NE		(1 << 1)
#define CTLI_INTCNT_PE		(2 << 1)
#define CTLI_INTCNT_BE		(3 << 1)

#define CTLO_DIR_OUT		(1 << 5)
#define CTLO_DRV_CMOS		(0 << 4)
#define CTLO_DRV_OD		(1 << 4)
#define CTLO_DRV_REN		(1 << 3)
#define CTLO_RVAL_2KDW		(0)
#define CTLO_RVAL_2KUP		(1 << 1)
#define CTLO_RVAL_50KDW		(2 << 1)
#define CTLO_RVAL_50KUP		(3 << 1)

#define CTLO_INPUT_DEF	(CTLO_DRV_CMOS | CTLO_DRV_REN | CTLO_RVAL_2KUP)
#define CTLO_OUTPUT_DEF	(CTLO_DIR_OUT | CTLO_INPUT_DEF)

#define CRYSTAL_COVE 0x0
#define WHISKEY_COVE 0x1
#define DOLLAR_COVE 0x2

struct trip_config_map {
	u16 irq_reg;
	u16 irq_en;
	u16 evt_stat;
	u8 irq_mask;
	u8 irq_en_mask;
	u8 evt_mask;
	u8 trip_num;
};

struct thermal_irq_map {
	char handle[20];
	int num_trips;
	struct trip_config_map *trip_config;
};

struct pmic_thermal_data {
	struct thermal_irq_map *maps;
	int num_maps;
};

struct pmic_gpio_data {
	int type;
	int num_gpio;
	int num_vgpio;
};

struct pmic_gpio {
	struct mutex		buslock; /* irq_bus_lock */
#ifdef CONFIG_GPIOLIB
	struct gpio_chip	chip;
#endif
	int			irq;
	int			irq_base;
	int			update;
	int			trigger_type;
	int			irq_mask;
	struct pmic_gpio_data	*gpio_data;
	int			print_wakeup;
};

int intel_soc_pmic_readb(int reg);
int intel_soc_pmic_writeb(int reg, u8 val);
int intel_soc_pmic_setb(int reg, u8 mask);
int intel_soc_pmic_clearb(int reg, u8 mask);
int intel_soc_pmic_update(int reg, u8 val, u8 mask);
int intel_soc_pmic_set_pdata(const char *name, void *data, int len, int id);
struct device *intel_soc_pmic_dev(void);

#endif	/* __INTEL_SOC_PMIC_H__ */
