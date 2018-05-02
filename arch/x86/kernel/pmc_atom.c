/*
 * Intel Atom SOC Power Management Controller Driver
 * Copyright (c) 2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/suspend.h>

#include <linux/syscore_ops.h>
#include <asm/pmc_atom.h>

#define	DRIVER_NAME	KBUILD_MODNAME

#define SC_NUM_DEVICES 36
#define FUNC_DIS_OFFSET 4
#define FUNC_DIS2_OFFSET 1

struct pmc_dev {
	u32 base_addr;
	void __iomem *regmap;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dbgfs_dir;
#endif /* CONFIG_DEBUG_FS */

};

static struct pmc_dev pmc_device;
static u32 acpi_base_addr;

struct pmc_bit_map {
	const char *name;
	u32 d3_sts_bit;
	u32 fn_dis_bit;
};
struct pmc_counters {
	u64 prev_s0ir_tmr;
	u64 prev_s0i1_tmr;
	u64 prev_s0i2_tmr;
	u64 prev_s0i3_tmr;
	u64 prev_s0_tmr;
};

#ifdef CONFIG_PM_SLEEP
struct susp_stats {
	u64 residency;
	u64 tmr_before_susp;
	u64 tmr_after_susp;
	bool suspend;
};

static struct susp_stats legacy_suspend = {0, 0, 0, false};
#endif /* CONFIG_PM_SLEEP */

static struct pmc_counters s0ix_counters = {0, 0, 0, 0, 0};
static  struct pmc_bit_map *dev_map;
static  struct pmc_bit_map *pss_map;
static int dev_num;
static int pss_num;

static  struct pmc_bit_map byt_dev_map[] = {
	{"0  - LPSS1_F0_DMA",		BIT_LPSS1_F0_DMA},
	{"1  - LPSS1_F1_PWM1",		BIT_LPSS1_F1_PWM1},
	{"2  - LPSS1_F2_PWM2",		BIT_LPSS1_F2_PWM2},
	{"3  - LPSS1_F3_HSUART1",	BIT_LPSS1_F3_HSUART1},
	{"4  - LPSS1_F4_HSUART2",	BIT_LPSS1_F4_HSUART2},
	{"5  - LPSS1_F5_SPI",		BIT_LPSS1_F5_SPI},
	{"6  - LPSS1_F6_Reserved",	BIT_LPSS1_F6_XXX},
	{"7  - LPSS1_F7_Reserved",	BIT_LPSS1_F7_XXX},
	{"8  - SCC_EMMC",		BIT_SCC_EMMC},
	{"9  - SCC_SDIO",		BIT_SCC_SDIO},
	{"10 - SCC_SDCARD",		BIT_SCC_SDCARD},
	{"11 - SCC_MIPI",		BIT_SCC_MIPI},
	{"12 - HDA",			BIT_HDA},
	{"13 - LPE",			BIT_LPE},
	{"14 - OTG",			BIT_OTG},
	{"15 - USH",			BIT_USH},
	{"16 - GBE",			BIT_GBE},
	{"17 - SATA",			BIT_SATA},
	{"18 - USB_EHCI",		BIT_USB_EHCI},
	{"19 - SEC",			BIT_SEC},
	{"20 - PCIE_PORT0",		BIT_PCIE_PORT0},
	{"21 - PCIE_PORT1",		BIT_PCIE_PORT1},
	{"22 - PCIE_PORT2",		BIT_PCIE_PORT2},
	{"23 - PCIE_PORT3",		BIT_PCIE_PORT3},
	{"24 - LPSS2_F0_DMA",		BIT_LPSS2_F0_DMA},
	{"25 - LPSS2_F1_I2C1",		BIT_LPSS2_F1_I2C1},
	{"26 - LPSS2_F2_I2C2",		BIT_LPSS2_F2_I2C2},
	{"27 - LPSS2_F3_I2C3",		BIT_LPSS2_F3_I2C3},
	{"28 - LPSS2_F3_I2C4",		BIT_LPSS2_F4_I2C4},
	{"29 - LPSS2_F5_I2C5",		BIT_LPSS2_F5_I2C5},
	{"30 - LPSS2_F6_I2C6",		BIT_LPSS2_F6_I2C6},
	{"31 - LPSS2_F7_I2C7",		BIT_LPSS2_F7_I2C7},
	{"32 - SMB",			BIT_SMB},
	{"33 - OTG_SS_PHY",		BIT_OTG_SS_PHY_BYT},
	{"34 - USH_SS_PHY",		BIT_USH_SS_PHY_BYT},
	{"35 - DFX",			BIT_DFX_BYT},
};

static struct pmc_bit_map pss_map_vlv[] = {
	{"0  - GBE",			PMC_PSS_BIT_GBE},
	{"1  - SATA",			PMC_PSS_BIT_SATA},
	{"2  - HDA",			PMC_PSS_BIT_HDA},
	{"3  - SEC",			PMC_PSS_BIT_SEC},
	{"4  - PCIE",			PMC_PSS_BIT_PCIE},
	{"5  - LPSS",			PMC_PSS_BIT_LPSS},
	{"6  - LPE",			PMC_PSS_BIT_LPE},
	{"7  - DFX",			PMC_PSS_BIT_DFX},
	{"8  - USH_CTRL",		PMC_PSS_BIT_USH_CTRL},
	{"9  - USH_SUS",		PMC_PSS_BIT_USH_SUS},
	{"10 - USH_VCCS",		PMC_PSS_BIT_USH_VCCS},
	{"11 - USH_VCCA",		PMC_PSS_BIT_USH_VCCA},
	{"12 - OTG_CTRL",		PMC_PSS_BIT_OTG_CTRL},
	{"13 - OTG_VCCS",		PMC_PSS_BIT_OTG_VCCS},
	{"14 - OTG_VCCA_CLK",		PMC_PSS_BIT_OTG_VCCA_CLK},
	{"15 - OTG_VCCA",		PMC_PSS_BIT_OTG_VCCA},
	{"16 - USB",			PMC_PSS_BIT_USB},
	{"17 - USB_SUS",		PMC_PSS_BIT_USB_SUS},
};

static struct pmc_bit_map cht_dev_map[] = {
	{"0  - LPSS1_F0_DMA",		BIT_LPSS1_F0_DMA},
	{"1  - LPSS1_F1_PWM1",		BIT_LPSS1_F1_PWM1},
	{"2  - LPSS1_F2_PWM2",		BIT_LPSS1_F2_PWM2},
	{"3  - LPSS1_F3_HSUART1",	BIT_LPSS1_F3_HSUART1},
	{"4  - LPSS1_F4_HSUART2",	BIT_LPSS1_F4_HSUART2},
	{"5  - LPSS1_F5_SPI",		BIT_LPSS1_F5_SPI},
	{"6  - LPSS1_F6_Reserved",	BIT_LPSS1_F6_XXX},
	{"7  - LPSS1_F7_Reserved",	BIT_LPSS1_F7_XXX},
	{"8  - SCC_EMMC",		BIT_SCC_EMMC},
	{"9  - SCC_SDIO",		BIT_SCC_SDIO},
	{"10 - SCC_SDCARD",		BIT_SCC_SDCARD},
	{"11 - SCC_MIPI",		BIT_SCC_MIPI},
	{"12 - HDA",			BIT_HDA},
	{"13 - LPE",			BIT_LPE},
	{"14 - OTG",			BIT_OTG},
	{"15 - USH",			BIT_USH},
	{"16 - GBE",			BIT_GBE},
	{"17 - SATA",			BIT_SATA},
	{"18 - USB_EHCI",		BIT_USB_EHCI},
	{"19 - SEC",			BIT_SEC},
	{"20 - PCIE_PORT0",		BIT_PCIE_PORT0},
	{"21 - PCIE_PORT1",		BIT_PCIE_PORT1},
	{"22 - PCIE_PORT2",		BIT_PCIE_PORT2},
	{"23 - PCIE_PORT3",		BIT_PCIE_PORT3},
	{"24 - LPSS2_F0_DMA",		BIT_LPSS2_F0_DMA},
	{"25 - LPSS2_F1_I2C1",		BIT_LPSS2_F1_I2C1},
	{"26 - LPSS2_F2_I2C2",		BIT_LPSS2_F2_I2C2},
	{"27 - LPSS2_F3_I2C3",		BIT_LPSS2_F3_I2C3},
	{"28 - LPSS2_F3_I2C4",		BIT_LPSS2_F4_I2C4},
	{"29 - LPSS2_F5_I2C5",		BIT_LPSS2_F5_I2C5},
	{"30 - LPSS2_F6_I2C6",		BIT_LPSS2_F6_I2C6},
	{"31 - LPSS2_F7_I2C7",		BIT_LPSS2_F7_I2C7},
	{"32 - SMB",	BIT_SMB},
	{"33 - GMM",	BIT_GMM_CHT,	BIT_GMM_FD_CHT},
	{"34 - ISH",	BIT_ISH_CHT,	BIT_ISH_FD_CHT},
};

static struct pmc_bit_map pss_map_cht[] = {
	{"1  - SATA",			PMC_PSS_BIT_SATA},
	{"2  - HDA",			PMC_PSS_BIT_HDA},
	{"3  - SEC",			PMC_PSS_BIT_SEC},
	{"4  - PCIE",			PMC_PSS_BIT_PCIE},
	{"5  - LPSS",			PMC_PSS_BIT_LPSS},
	{"6  - LPE",			PMC_PSS_BIT_LPE},
	{"7  - UFS",			PMC_PSS_BIT_UFS},
	{"11  - UXD(xDCI)",		PMC_PSS_BIT_UXD},
	{"12  - UXD FD SX(xDCI)",		PMC_PSS_BIT_UXD_FD},
	{"15 -  UX Engine(xHCI) ",		PMC_PSS_BIT_UX_ENG},
	{"16 - USB SUS ",		PMC_PSS_BIT_USB_SUS_CHT},
	{"17 - GMM",		PMC_PSS_BIT_GMM},
	{"18 - ISH",		PMC_PSS_BIT_ISH},
	{"26 - DFX Master",		PMC_PSS_BIT_DFX_MASTER},
	{"27 - DFX Cluster1",		PMC_PSS_BIT_DFX_CLSTR1},
	{"28 - DFX Cluster2",		PMC_PSS_BIT_DFX_CLSTR2},
	{"29 - DFX Cluster3",		PMC_PSS_BIT_DFX_CLSTR3},
	{"30 - DFX Cluster4",		PMC_PSS_BIT_DFX_CLSTR4},
	{"31 - DFX Cluster5",		PMC_PSS_BIT_DFX_CLSTR5},
};

static inline u32 pmc_reg_read(struct pmc_dev *pmc, int reg_offset)
{
	return readl(pmc->regmap + reg_offset);
}

static inline void pmc_reg_write(struct pmc_dev *pmc, int reg_offset, u32 val)
{
	writel(val, pmc->regmap + reg_offset);
}

static void pmc_power_off(void)
{
	u16	pm1_cnt_port;
	u32	pm1_cnt_value;

	pr_info("Preparing to enter system sleep state S5\n");

	pm1_cnt_port = acpi_base_addr + PM1_CNT;

	pm1_cnt_value = inl(pm1_cnt_port);
	pm1_cnt_value &= SLEEP_TYPE_MASK;
	pm1_cnt_value |= SLEEP_TYPE_S5;
	pm1_cnt_value |= SLEEP_ENABLE;

	outl(pm1_cnt_value, pm1_cnt_port);
}

static void pmc_hw_reg_setup(struct pmc_dev *pmc)
{
	/*
	 * Disable PMC S0IX_WAKE_EN events coming from:
	 * - LPC clock run
	 * - GPIO_SUS ored dedicated IRQs
	 * - GPIO_SCORE ored dedicated IRQs
	 * - GPIO_SUS shared IRQ
	 * - GPIO_SCORE shared IRQ
	 */
	pmc_reg_write(pmc, PMC_S0IX_WAKE_EN, (u32)PMC_WAKE_EN_SETTING);
}

#ifdef CONFIG_PM_SLEEP
static u64 read_pmc_s0i3_tmr_reg(void)
{
	struct pmc_dev *pmc = &pmc_device;
	u64 val = 0;

	/* Check if pmc is valid */
	if (pmc != NULL) {
		val = (u64) pmc_reg_read(pmc, PMC_S0I3_TMR) << PMC_TMR_SHIFT;
		return val;
	} else {
		pr_err("PMC_S0I3_TMR register read failed; pmc dev pointer is NULL\n");
		return -1;
	}
}

static int pm_suspend_prep_event(void)
{
	u64 tmr = 0;

	if (!legacy_suspend.suspend) {
		/* Get the snapshot of s0i3_tmr before suspend */
		tmr = read_pmc_s0i3_tmr_reg();
		if (tmr < 0)
			pr_err("Before Suspend: PMC_S0I3_TMR register read returned negative value\n");
		else
			legacy_suspend.tmr_before_susp = tmr;
	}
	legacy_suspend.suspend = true;
	return NOTIFY_OK;
}

static int pm_suspend_exit_event(void)
{
	u64 tmr = 0;

	if (legacy_suspend.suspend) {
		/* Get the snapshot of s0i3_tmr post suspend */
		tmr = read_pmc_s0i3_tmr_reg();
		if (tmr < 0) {
			pr_err("Post Suspend: PMC_S0I3_TMR register read returned negative value\n");
		} else {
			legacy_suspend.tmr_after_susp = tmr;
			pr_info("Sleep residency in the last suspend cycle = %llu ms",
			legacy_suspend.tmr_after_susp -
			legacy_suspend.tmr_before_susp);
			/* Compute the time spent in suspend */
			legacy_suspend.residency +=
			 (legacy_suspend.tmr_after_susp -
				 legacy_suspend.tmr_before_susp);
		}
	}
	legacy_suspend.suspend = false;
	return NOTIFY_OK;
}

static int pm_notification(struct notifier_block *this,
				 unsigned long event, void *ptr)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		return pm_suspend_prep_event();
	case PM_POST_SUSPEND:
		return pm_suspend_exit_event();
	}
	return NOTIFY_DONE;
}

static struct notifier_block pm_event_notifier = {
		.notifier_call = pm_notification,
};
#endif /* CONFIG_PM_SLEEP */


static void pmc_dev_state(void *seq_file)
{
	struct pmc_dev *pmc;
	struct seq_file *s = (struct seq_file *)seq_file;
	u32 func_dis, func_dis_2, func_dis_index;
	u32 d3_sts_0, d3_sts_1, d3_sts_index;
	int dev_index, reg_index;

	if (s)
		pmc = s->private;
	else
		pmc = &pmc_device;

	func_dis = pmc_reg_read(pmc, PMC_FUNC_DIS);
	func_dis_2 = pmc_reg_read(pmc, PMC_FUNC_DIS_2);
	d3_sts_0 = pmc_reg_read(pmc, PMC_D3_STS_0);
	d3_sts_1 = pmc_reg_read(pmc, PMC_D3_STS_1);

	for (dev_index = 0; dev_index < dev_num; dev_index++) {
		reg_index = dev_index / PMC_REG_BIT_WIDTH;

		if (reg_index) {
			func_dis_index =
				(func_dis_2 & dev_map[dev_index].fn_dis_bit);
			d3_sts_index = d3_sts_1;
		} else {
			func_dis_index =
				func_dis & dev_map[dev_index].fn_dis_bit;
			d3_sts_index = d3_sts_0;
		}

		if ((pm_suspend_debug) && (s == NULL)) {
			if (!(dev_map[dev_index].d3_sts_bit & d3_sts_index) &&
				!func_dis_index)
				pr_info("%s in SC is in D0 prior to sleep\n",
					dev_map[dev_index].name);

		} else if (s) {
			seq_printf(s, "Dev: %-32s\tState: %s [%s]\n",
			dev_map[dev_index].name,
			func_dis_index ?
			"Disabled" : "Enabled ",
			dev_map[dev_index].d3_sts_bit & d3_sts_index ?
			"D3" : "D0");
		}
	}
	return;
}

#ifdef CONFIG_DEBUG_FS

static int pmc_dev_state_show(struct seq_file *s, void *unused)
{
	pmc_dev_state(s);
	return 0;
}

static int pmc_dev_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_dev_state_show, inode->i_private);
}

static const struct file_operations pmc_dev_state_ops = {
	.open		= pmc_dev_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int pmc_pss_state_show(struct seq_file *s, void *unused)
{
	struct pmc_dev *pmc = s->private;
	u32 pss = pmc_reg_read(pmc, PMC_PSS);
	int pss_index;

	for (pss_index = 0; pss_index < pss_num; pss_index++) {
		seq_printf(s, "Island: %-32s\tState: %s\n",
			pss_map[pss_index].name,
			pss_map[pss_index].d3_sts_bit & pss ? "Off" : "On");
	}
	return 0;
}

static int pmc_pss_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_pss_state_show, inode->i_private);
}

static const struct file_operations pmc_pss_state_ops = {
	.open		= pmc_pss_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int pmc_sleep_tmr_show(struct seq_file *s, void *unused)
{
	struct pmc_dev *pmc = s->private;
	u64 s0ir_tmr, s0i1_tmr, s0i2_tmr, s0i3_tmr, s0_tmr;
#ifdef CONFIG_PM_SLEEP
	u64 suspend_time;
#endif

	s0_tmr = (u64)pmc_reg_read(pmc, PMC_S0_TMR) << PMC_TMR_SHIFT;
	s0_tmr = s0_tmr - s0ix_counters.prev_s0_tmr;
	do_div(s0_tmr, MSEC_PER_SEC);

	s0ir_tmr = (u64)pmc_reg_read(pmc, PMC_S0IR_TMR) << PMC_TMR_SHIFT;
	s0ir_tmr = s0ir_tmr -  s0ix_counters.prev_s0ir_tmr;
	do_div(s0ir_tmr, MSEC_PER_SEC);

	s0i1_tmr = (u64)pmc_reg_read(pmc, PMC_S0I1_TMR) << PMC_TMR_SHIFT;
	s0i1_tmr = s0i1_tmr -  s0ix_counters.prev_s0i1_tmr;
	do_div(s0i1_tmr, MSEC_PER_SEC);

	s0i2_tmr = (u64)pmc_reg_read(pmc, PMC_S0I2_TMR) << PMC_TMR_SHIFT;
	s0i2_tmr = s0i2_tmr - s0ix_counters.prev_s0i2_tmr;
	do_div(s0i2_tmr, MSEC_PER_SEC);

	s0i3_tmr = (u64)pmc_reg_read(pmc, PMC_S0I3_TMR) << PMC_TMR_SHIFT;
	s0i3_tmr =  s0i3_tmr -  s0ix_counters.prev_s0i3_tmr;
#ifdef CONFIG_PM_SLEEP
	/* Case of legacy_suspend residency being higher than s0i3_tmr doesn't
	 * arise as s0i3_tmr value read from PMC register is the sum of time
	 * spent in s0i3 and suspend.
	 * Hence s0i3_tmr will always be higher or equal.
	 */
	s0i3_tmr = (s0i3_tmr - legacy_suspend.residency);
#endif
	do_div(s0i3_tmr, MSEC_PER_SEC);

	seq_puts(s , "       Residency Time\n");
	seq_printf(s , "S0:  %13.2llu ms\n", s0_tmr);
	seq_printf(s , "S0IR:%13.2llu ms\n", s0ir_tmr);
	seq_printf(s , "S0I1:%13.2llu ms\n", s0i1_tmr);
	seq_printf(s , "S0I2:%13.2llu ms\n", s0i2_tmr);
	seq_printf(s , "Idle-S0I3:%13.2llu ms\n", s0i3_tmr);
#ifdef CONFIG_PM_SLEEP
	suspend_time = legacy_suspend.residency;
	do_div(suspend_time, MSEC_PER_SEC);
	seq_printf(s , "legacy-suspend:%13.2llu ms\n", suspend_time);
#endif /* CONFIG_PM_SLEEP */
	return 0;
}

static ssize_t pmc_sleep_tmr_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[32];
	struct pmc_dev *pmc = &pmc_device;
	int buf_size = min(count, sizeof(buf)-1);
	char *clear_msg = "clear";
	int clear_msg_len = strlen(clear_msg);

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	buf[buf_size] = 0;
	if (((clear_msg_len + 1) == buf_size) &&
		!strncmp(buf, clear_msg, clear_msg_len)) {
		s0ix_counters.prev_s0ir_tmr =
			(u64)pmc_reg_read(pmc, PMC_S0IR_TMR) << PMC_TMR_SHIFT;
		s0ix_counters.prev_s0i1_tmr =
			(u64)pmc_reg_read(pmc, PMC_S0I1_TMR) << PMC_TMR_SHIFT;
		s0ix_counters.prev_s0i2_tmr =
			(u64)pmc_reg_read(pmc, PMC_S0I2_TMR) << PMC_TMR_SHIFT;
		s0ix_counters.prev_s0i3_tmr =
			(u64)pmc_reg_read(pmc, PMC_S0I3_TMR) << PMC_TMR_SHIFT;
		s0ix_counters.prev_s0_tmr =
			(u64)pmc_reg_read(pmc, PMC_S0_TMR) << PMC_TMR_SHIFT;
#ifdef CONFIG_PM_SLEEP
		/* Reset the legacy suspend counter value as well */
		legacy_suspend.residency = 0;
#endif /* CONFIG_PM_SLEEP */
	}
	return buf_size;
}
static int pmc_sleep_tmr_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_sleep_tmr_show, inode->i_private);
}

static int pmc_func_dis_show(struct seq_file *s, void *unused)
{
	return 0;
}

static const struct file_operations pmc_sleep_tmr_ops = {
	.open		= pmc_sleep_tmr_open,
	.read		= seq_read,
	.write		= pmc_sleep_tmr_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int pmc_func_dis_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_func_dis_show, inode->i_private);
}

static ssize_t pmc_func_dis_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	int val;
	unsigned int buf_size;
	u32 fd;
	struct pmc_dev *pmc = &pmc_device;

	buf_size = count < 64 ? count : 64;

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%d", &val) != 1)
		return -EFAULT;

	if (val < (SC_NUM_DEVICES - FUNC_DIS_OFFSET)) {
		fd = pmc_reg_read(pmc, PMC_FUNC_DIS);
		fd |= (1 << val);
		pmc_reg_write(pmc, PMC_FUNC_DIS, fd);

	} else if (val < (SC_NUM_DEVICES - FUNC_DIS2_OFFSET)) {
		fd = pmc_reg_read(pmc, PMC_FUNC_DIS_2);
		fd |= dev_map[val].fn_dis_bit;
		pmc_reg_write(pmc, PMC_FUNC_DIS_2, fd);
	} else
		pr_err("Invalid South Complex IP\n");

	return count;
}

static const struct file_operations pmc_func_dis_ops = {
	.open		= pmc_func_dis_open,
	.read		= seq_read,
	.write		= pmc_func_dis_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static void pmc_dbgfs_unregister(struct pmc_dev *pmc)
{
	if (!pmc->dbgfs_dir)
		return;

	debugfs_remove_recursive(pmc->dbgfs_dir);
	pmc->dbgfs_dir = NULL;
}

static int pmc_dbgfs_register(struct pmc_dev *pmc, struct pci_dev *pdev)
{
	struct dentry *dir, *f;

	dir = debugfs_create_dir("pmc_atom", NULL);
	if (!dir)
		return -ENOMEM;

	f = debugfs_create_file("dev_state", S_IFREG | S_IRUGO,
				dir, pmc, &pmc_dev_state_ops);
	if (!f) {
		dev_err(&pdev->dev, "dev_state register failed\n");
		goto err;
	}
	f = debugfs_create_file("sleep_state", S_IFREG | S_IRUGO,
				dir, pmc, &pmc_sleep_tmr_ops);
	if (!f) {
		dev_err(&pdev->dev, "sleep_state register failed\n");
		goto err;
	}
	f = debugfs_create_file("pss_state", S_IFREG | S_IRUGO,
				dir, pmc, &pmc_pss_state_ops);
	if (!f) {
		dev_err(&pdev->dev, "pss_state register failed\n");
		goto err;
	}

	f = debugfs_create_file("func_dis", S_IFREG | S_IRUGO,
				dir, pmc, &pmc_func_dis_ops);
	if (!f)	{
		dev_err(&pdev->dev, "func_dis register failed\n");
		goto err;
	}
	pmc->dbgfs_dir = dir;
	return 0;
err:
	pmc_dbgfs_unregister(pmc);
	return -ENODEV;
}
#endif /* CONFIG_DEBUG_FS */

#define	PMC_RTC_STS	(1 << 10)
static void pmc_resume(void)
{
	u32 value;

	if (!acpi_base_addr)
		return;

	value = inl(acpi_base_addr + 0);

	value &= 0xFFFF;
	pr_info("pmc: wakeup status[%x]\n", value);
	if (value & PMC_RTC_STS)
		pr_info("pmc: rtc might wake up system!\n");

	return;
}

static struct syscore_ops pmc_syscore_ops = {
	.resume = pmc_resume,
};

static int pmc_setup_dev(struct pci_dev *pdev)
{
	struct pmc_dev *pmc = &pmc_device;
	int ret;
	int dev_index, reg_index;

	/* Obtain ACPI base address */
	pci_read_config_dword(pdev, ACPI_BASE_ADDR_OFFSET, &acpi_base_addr);
	acpi_base_addr &= ACPI_BASE_ADDR_MASK;

	/* Install power off function */
	if (acpi_base_addr != 0 && pm_power_off == NULL)
		pm_power_off = pmc_power_off;

	pci_read_config_dword(pdev, PMC_BASE_ADDR_OFFSET, &pmc->base_addr);
	pmc->base_addr &= PMC_BASE_ADDR_MASK;

	pmc->regmap = ioremap_nocache(pmc->base_addr, PMC_MMIO_REG_LEN);
	if (!pmc->regmap) {
		dev_err(&pdev->dev, "error: ioremap failed\n");
		return -ENOMEM;
	}

	/* PMC hardware registers setup */
	pmc_hw_reg_setup(pmc);

	for (dev_index = 0; dev_index < dev_num; dev_index++) {
		reg_index = dev_index / PMC_REG_BIT_WIDTH;
		if (!dev_map[dev_index].fn_dis_bit)
				dev_map[dev_index].fn_dis_bit =
					dev_map[dev_index].d3_sts_bit;
	}

#ifdef CONFIG_PM_SLEEP
	ret = register_pm_notifier(&pm_event_notifier);
	if (ret)
		dev_err(&pdev->dev, "error: Registering for PM suspend/resume notifiers failed\n");
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_DEBUG_FS
	ret = pmc_dbgfs_register(pmc, pdev);
	if (ret) {
		iounmap(pmc->regmap);
		return ret;
	}

	sc_dev_state = pmc_dev_state;
#endif /* CONFIG_DEBUG_FS */
	return 0;
}

/*
 * Data for PCI driver interface
 *
 * This data only exists for exporting the supported
 * PCI ids via MODULE_DEVICE_TABLE.  We do not actually
 * register a pci_driver, because lpc_ich will register
 * a driver on the same PCI id.
 */
static const struct pci_device_id pmc_pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_BYT_PMC) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_CHT_PMC) },
	{ 0, },
};

MODULE_DEVICE_TABLE(pci, pmc_pci_ids);

static int __init pmc_atom_init(void)
{
	int err = -ENODEV;
	struct pci_dev *pdev = NULL;
	const struct pci_device_id *ent;

	register_syscore_ops(&pmc_syscore_ops);

	/* We look for our device - PCU PMC
	 * we assume that there is max. one device.
	 *
	 * We can't use plain pci_driver mechanism,
	 * as the device is really a multiple function device,
	 * main driver that binds to the pci_device is lpc_ich
	 * and have to find & bind to the device this way.
	 */
	for_each_pci_dev(pdev) {
		ent = pci_match_id(pmc_pci_ids, pdev);
		if (ent) {
			switch (ent->device) {
			case PCI_DEVICE_ID_BYT_PMC:
				dev_map = byt_dev_map;
				dev_num = ARRAY_SIZE(byt_dev_map);
				pss_map = pss_map_vlv;
				pss_num = ARRAY_SIZE(pss_map_vlv);
				break;
			case PCI_DEVICE_ID_CHT_PMC:
				dev_map = cht_dev_map;
				dev_num = ARRAY_SIZE(cht_dev_map);
				pss_map = pss_map_cht;
				pss_num = ARRAY_SIZE(pss_map_cht);
				break;
			}
			err = pmc_setup_dev(pdev);
			goto out;
		}
	}
	/* Device not found. */
out:
	return err;
}

module_init(pmc_atom_init);
/* no module_exit, this driver shouldn't be unloaded */

MODULE_AUTHOR("Aubrey Li <aubrey.li@linux.intel.com>");
MODULE_DESCRIPTION("Intel Atom SOC Power Management Controller Interface");
MODULE_LICENSE("GPL v2");
