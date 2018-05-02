/*
 * Intel Vendor Defined XHCI extended capability
 *
 * Copyright (C) 2014 Intel Corp.
 *
 * Author: Wu, Hao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 */

#include <linux/usb/phy.h>
#include <linux/usb/otg.h>
#include <linux/acpi.h>
#include <linux/pci.h>

#include "xhci.h"
#include "xhci-intel-cap.h"

void xhci_change_ssic_regs(struct xhci_hcd *xhci, bool enable)
{

	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	u32 data;

	data = readl(hcd->regs + SSIC_CONFIG2);
	xhci_dbg(xhci, "%s(read): 0x%p = 0x%08x\n", __func__,
			(hcd->regs + SSIC_CONFIG2), data);

	if (enable)
		data |= RETRAIN;
	else
		data &= ~RETRAIN;

	writel(data, hcd->regs + SSIC_CONFIG2);
	xhci_dbg(xhci, "%s(write): 0x%p = 0x%08x\n", __func__,
			(hcd->regs + SSIC_CONFIG2),
			readl(hcd->regs + SSIC_CONFIG2));
}

bool xhci_intel_ssic_port_check(struct xhci_hcd *xhci, int port)
{
	/* Check if device present and the port matches */
	return (xhci->ssic_device_present) &&
			(port == xhci->ssic_port_number);
}

int xhci_intel_need_disable_stall(struct xhci_hcd *xhci)
{
	struct acpi_device *acpi_dev;
	acpi_status modem_status;
	union acpi_object *modem_data;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct pci_dev  *pdev = to_pci_dev(xhci_to_hcd(xhci)->self.controller);
	int modem_present, modem_type;

	/* Get ACPI device */
	acpi_dev = ACPI_COMPANION(&pdev->dev);
	if (!acpi_dev || !acpi_dev->handle) {
		dev_err(&pdev->dev, "No ACPI device!\n");
		return 0;
	}

	/* Get modem status from MINF function */
	modem_status = acpi_evaluate_object(acpi_dev->handle, "MINF",
			NULL, &buffer);
	if (ACPI_FAILURE(modem_status)) {
		dev_err(&pdev->dev, "No MINF method!\n");
		return 0;
	}

	/* Get the data from the buffer */
	modem_data = buffer.pointer;

	/* Assign the values */
	modem_present = modem_data->package.elements[0].integer.value;
	xhci->ssic_device_present = modem_present;
	xhci->ssic_port_number = modem_data->package.elements[1].integer.value;
	modem_type = modem_data->package.elements[2].integer.value;

	/* Evaluate the values from ACPI */
	return modem_present && modem_type == SSIC_MODEM_7260;
}

int xhci_intel_vendor_cap_init(struct xhci_hcd *xhci)
{
	struct usb_hcd *hcd;
	int ext_offset, retval;

	if (xhci->quirks & XHCI_SSIC_DISABLE_STALL)
		xhci_change_ssic_regs(xhci, false);

	ext_offset = XHCI_HCC_EXT_CAPS(readl(&xhci->cap_regs->hcc_params));
	ext_offset = xhci_find_ext_cap_by_id(&xhci->cap_regs->hc_capbase,
			ext_offset << 2, XHCI_EXT_CAPS_INTEL_HOST_CAP);
	if (!ext_offset)
		return -ENODEV;

	xhci->phy_mux_regs = &xhci->cap_regs->hc_capbase + (ext_offset >> 2);

	xhci_dbg(xhci, "Intel Vendor Defined Cap %d initialization\n",
					XHCI_EXT_CAPS_INTEL_HOST_CAP);
	xhci_dbg(xhci, "regs offset = 0x%x, phy_mux_regs = 0x%p\n",
					ext_offset, xhci->phy_mux_regs);

	/* If This capbility is found, register host on PHY for OTG purpose */
	hcd = xhci_to_hcd(xhci);
	hcd->phy = usb_get_phy(USB_PHY_TYPE_USB2);

	if (!IS_ERR_OR_NULL(hcd->phy)) {
		retval = otg_set_host(hcd->phy->otg, &hcd->self);
		if (retval)
			usb_put_phy(hcd->phy);
		else {
			/* if register for OTG succeed, then set mux to device
			 * mode by default for OTG purpose */
			xhci_dbg(xhci, "set mux to device mode as default\n");
			xhci_intel_phy_mux_switch(xhci, 1);
		}
	} else {
		hcd->phy = NULL;
	}

	xhci_dbg(xhci, "capability init done\n");

	return 0;
}

/* Only used for device mode */
int xhci_intel_phy_vbus_valid(struct xhci_hcd *xhci, int vbus_valid)
{
	u32		data;

	if (!xhci || !xhci->phy_mux_regs)
		return -ENODEV;

	xhci_dbg(xhci, "vbus valid for phy mux is %d\n", vbus_valid);

	data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG0);

	if (vbus_valid)
		data |= SW_VBUS_VALID;
	else
		data &= ~SW_VBUS_VALID;

	writel(data, xhci->phy_mux_regs + DUAL_ROLE_CFG0);

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_intel_phy_vbus_valid);

int xhci_intel_phy_mux_switch(struct xhci_hcd *xhci, int is_device_on)
{
	unsigned long	timeout;
	u32		data;

	if (!xhci || !xhci->phy_mux_regs) {
		pr_err("No XHCI or Not support phy mux capability\n");
		return -EINVAL;
	}

	xhci_dbg(xhci, "XHCI phy mux switch to %s path\n",
				is_device_on ? "dev" : "Host");

	/* Check and set mux to SW controlled mode */
	data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG0);
	if (!(data & SW_IDPIN_EN)) {
		data |= SW_IDPIN_EN;
		writel(data, xhci->phy_mux_regs + DUAL_ROLE_CFG0);
	}

	/* Configure CFG0 to switch the mux and VBUS_VALID bit is required
	 * for device mode */
	data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG0);
	if (is_device_on)
		data |= SW_IDPIN;
	else
		data &= ~SW_IDPIN;
	writel(data, xhci->phy_mux_regs + DUAL_ROLE_CFG0);

	/* Polling CFG1 for safety, most case it takes about 600ms to finish
	 * mode switching, set TIMEOUT long enough */
	timeout = jiffies + msecs_to_jiffies(DUAL_ROLE_CFG1_POLL_TIMEOUT);

	/* Polling on CFG1 register to confirm mode switch.*/
	while (!time_after(jiffies, timeout)) {
		data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG1);
		if (is_device_on) {
			if (!(data & SW_MODE))
				break;
		} else {
			if (data & SW_MODE)
				break;
		}
		/* interval for polling is set to about 5ms */
		usleep_range(5000, 5100);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_intel_phy_mux_switch);

/* This function is only used as one workaround for Cherrytrail XHCI spurious
   pme issue */
void xhci_intel_clr_internal_pme_flag(struct xhci_hcd *xhci)
{
	u32	data;

	if (!xhci || !xhci->phy_mux_regs)
		return;

	xhci_dbg(xhci, "spurious PME issue workaround\n");

	/* clear internal PME flag, write 1 to PMCTRL.INT_PME_FLAG_CLR */
	data = readl(xhci->phy_mux_regs + PMCTRL);
	data |= INT_PME_FLAG_CLR;
	writel(data, xhci->phy_mux_regs + PMCTRL);
}
EXPORT_SYMBOL_GPL(xhci_intel_clr_internal_pme_flag);

/* This function is only used as one workaround for Cherrytrail XHCI spurious
   pme and HCRST hangs issue */
void xhci_intel_ssic_port_unused(struct xhci_hcd *xhci, bool unused)
{
	int ext_start, ext_offset, i;
	void __iomem *reg;
	u32 data;

	xhci_dbg(xhci, "ssic port - %s\n", unused ? "unused" : "used");

	ext_start = XHCI_HCC_EXT_CAPS(readl(&xhci->cap_regs->hcc_params));

	/* WORKAROUND: Register Bank Valid bit is lost after controller enters
	 * D3, need to set it back, otherwise SSIC RRAP commands can't be sent
	 * and link training will fail.
	 */
	if (!unused) {
		ext_offset = xhci_find_ext_cap_by_id(
				&xhci->cap_regs->hc_capbase, ext_start << 2,
				XHCI_EXT_CAPS_INTEL_SSIC_PROFILE);
		if (!ext_offset) {
			xhci_err(xhci, "intel ssic profile ext caps not found\n");
			return;
		}

		reg = &xhci->cap_regs->hc_capbase +
				((ext_offset + SSIC_ACCESS_CTRL) >> 2);
		for (i = 0; i < SSIC_PORT_NUM; i++) {
			data = readl(reg);
			data |= SSIC_ACCESS_CTRL_REGISTER_BANK_VALID;
			writel(data, reg);

			reg += SSIC_ACCESS_CTRL_OFFSET;
		}

		/* Setting SSIC ports to "used" will be done by xHCI
		 * Save & Restore, so return here.
		 */
		return;
	}

	ext_offset = xhci_find_ext_cap_by_id(&xhci->cap_regs->hc_capbase,
			ext_start << 2, XHCI_EXT_CAPS_INTEL_SSIC);

	if (ext_offset) {
		reg = &xhci->cap_regs->hc_capbase +
				((ext_offset + SSIC_PORT_CFG2) >> 2);
		for (i = 0; i < SSIC_PORT_NUM; i++) {
			data = readl(reg);
			data &= ~PROG_DONE;
			writel(data, reg);

			data = readl(reg);
			data |= SSIC_PORT_UNUSED;
			writel(data, reg);

			data = readl(reg);
			data |= PROG_DONE;
			writel(data, reg);

			reg += SSIC_PORT_CFG2_OFFSET;
		}
	} else
		xhci_err(xhci, "intel ssic ext caps not found\n");

}
EXPORT_SYMBOL_GPL(xhci_intel_ssic_port_unused);

/*
 * This function enables/disables the PIPE 4.1 synchronous phystatus.
 */
void xhci_intel_pipe_sync_phystatus_quirk(struct xhci_hcd *xhci, bool enable)
{
	u32	data;

	if (!xhci || !xhci->phy_mux_regs) {
		pr_err("no xhci or phy_mux_regs\n");
		return;
	}

	xhci_dbg(xhci, "pipe 4.1 phystatus: %s\n",
		enable ? "enable" : "disable");

	data = readl(xhci->phy_mux_regs + DUAL_ROLE_CFG0);
	if (enable)
		data |= EN_PIPE_4_1_SYNC_PHY_STATUS;
	else
		data &= ~EN_PIPE_4_1_SYNC_PHY_STATUS;
	writel(data, xhci->phy_mux_regs + DUAL_ROLE_CFG0);
}
EXPORT_SYMBOL_GPL(xhci_intel_pipe_sync_phystatus_quirk);
