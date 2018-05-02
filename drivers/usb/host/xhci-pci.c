/*
 * xHCI host controller driver PCI Bus Glue.
 *
 * Copyright (C) 2008 Intel Corp.
 *
 * Author: Sarah Sharp
 * Some code borrowed from the Linux EHCI driver.
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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/module.h>

#include "xhci.h"
#include "xhci-trace.h"
#include "xhci-intel-cap.h"

/* Device for a quirk */
#define PCI_VENDOR_ID_FRESCO_LOGIC	0x1b73
#define PCI_DEVICE_ID_FRESCO_LOGIC_PDK	0x1000
#define PCI_DEVICE_ID_FRESCO_LOGIC_FL1400	0x1400

#define PCI_VENDOR_ID_ETRON		0x1b6f
#define PCI_DEVICE_ID_ASROCK_P67	0x7023

#define PCI_DEVICE_ID_INTEL_LYNXPOINT_XHCI	0x8c31
#define PCI_DEVICE_ID_INTEL_LYNXPOINT_LP_XHCI	0x9c31

#define PCI_DEVICE_ID_INTEL_CHT_XHCI	0x22b5

#define SSIC_SS_PORT_LINK_CTRL 0x80ec
#define SSIC_SS_PORT_LINK_CTRL_U3_MASK (0x7 << 9)

struct xhci_hcd *p_xhci = NULL; 
ssize_t ssic_port_reset(int val)
{
	struct xhci_hcd *xhci = p_xhci; 
	__le32 __iomem **port_array;
	int size, port = 4;  /* Only CHT SSIC port need this enable/disable */
	u32 temp;
	int total_time;

	if(xhci == NULL)
		return -EINVAL;

	port_array = xhci->usb3_ports;
	temp = readl(port_array[port]);

	if (val == 1) {

		xhci_dbg(xhci, "Enable SS port %d\n", port);

		if ((temp & PORT_PLS_MASK) != USB_SS_PORT_LS_SS_DISABLED)
				return size;

		xhci_set_link_state(xhci, port_array, port,
				USB_SS_PORT_LS_RX_DETECT);
		temp = readl(port_array[port]);

	} else if (val == 0) {

		xhci_dbg(xhci, "Disable SS port %d\n", port);

		if ((temp & PORT_PLS_MASK) == USB_SS_PORT_LS_SS_DISABLED)
				return size;
		/*
		 * Clear all change bits, so that we get a new
		 * connection event.
		 */
		temp |= PORT_CSC | PORT_PEC | PORT_WRC |
			PORT_OCC | PORT_RC | PORT_PLC |
			PORT_CEC;
		writel(temp | PORT_PE, port_array[port]);
		temp = readl(port_array[port]);

		/*
		 * Wait until port is disabled
		 */
		for (total_time = 0; ; total_time += 25) {
			temp = readl(port_array[port]);

			if ((temp & PORT_PLS_MASK) == USB_SS_PORT_LS_SS_DISABLED)
				break;
			if (total_time >= 200)
				break;
			msleep(20);
		}
		if (total_time >= 200) {
			xhci_warn(xhci, "Disable port %d failed after %d ms\n",
					port, total_time);
			return -EBUSY;
		}
	} else {
		return -EINVAL;
	}
	return size;
}
static ssize_t ssic_port_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	__le32 __iomem **port_array;
	int val, port = 4;  /* Only CHT SSIC port need this enable/disable */
	u32 temp;
	int total_time;
	port_array = xhci->usb3_ports;
	temp = readl(port_array[port]);

	if (sscanf(buf, "%d", &val) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	if (val == 1) {

		xhci_dbg(xhci, "Enable SS port %d\n", port);

		if ((temp & PORT_PLS_MASK) != USB_SS_PORT_LS_SS_DISABLED)
				return size;

		xhci_set_link_state(xhci, port_array, port,
				USB_SS_PORT_LS_RX_DETECT);
		temp = readl(port_array[port]);

	} else if (val == 0) {

		xhci_dbg(xhci, "Disable SS port %d\n", port);

		if ((temp & PORT_PLS_MASK) == USB_SS_PORT_LS_SS_DISABLED)
				return size;
		/*
		 * Clear all change bits, so that we get a new
		 * connection event.
		 */
		temp |= PORT_CSC | PORT_PEC | PORT_WRC |
			PORT_OCC | PORT_RC | PORT_PLC |
			PORT_CEC;
		writel(temp | PORT_PE, port_array[port]);
		temp = readl(port_array[port]);

		/*
		 * Wait until port is disabled
		 */
		for (total_time = 0; ; total_time += 25) {
			temp = readl(port_array[port]);

			if ((temp & PORT_PLS_MASK) == USB_SS_PORT_LS_SS_DISABLED)
				break;
			if (total_time >= 200)
				break;
			msleep(20);
		}
		if (total_time >= 200) {
			xhci_warn(xhci, "Disable port %d failed after %d ms\n",
					port, total_time);
			return -EBUSY;
		}
	} else {
		return -EINVAL;
	}
	return size;
}

static ssize_t ssic_port_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	__le32 __iomem **port_array;
	int port = 4;
	u32 temp;
	port_array = xhci->usb3_ports;
	temp = readl(port_array[port]);

	if ((temp & PORT_PLS_MASK) == USB_SS_PORT_LS_SS_DISABLED)
		return sprintf(buf, "%s\n", "disabled");
	else
		return sprintf(buf, "%s\n", "enabled");
}
static DEVICE_ATTR_RW(ssic_port_enable);

static const char hcd_name[] = "xhci_hcd";

/* called after powerup, by probe or system-pm "wakeup" */
static int xhci_pci_reinit(struct xhci_hcd *xhci, struct pci_dev *pdev)
{
	/*
	 * TODO: Implement finding debug ports later.
	 * TODO: see if there are any quirks that need to be added to handle
	 * new extended capabilities.
	 */

	/* Init Intel vendor defined extended capability */
	if (pdev->vendor == PCI_VENDOR_ID_INTEL) {
		if (!xhci_intel_vendor_cap_init(xhci))
			xhci_dbg(xhci, "Intel Vendor Capability init done\n");
	}

	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
				pdev->device == PCI_DEVICE_ID_INTEL_CHT_XHCI) {
		struct usb_hcd *hcd = xhci_to_hcd(xhci);
		u32 data;

		/* Clear bit 11:9 of Superspeed Port Link Control reg */
		data = readl(hcd->regs + SSIC_SS_PORT_LINK_CTRL);
		data &= ~SSIC_SS_PORT_LINK_CTRL_U3_MASK;
		writel(data, hcd->regs + SSIC_SS_PORT_LINK_CTRL);
	}

	/* PCI Memory-Write-Invalidate cycle support is optional (uncommon) */
	if (!pci_set_mwi(pdev))
		xhci_dbg(xhci, "MWI active\n");

	xhci_dbg(xhci, "Finished xhci_pci_reinit\n");
	return 0;
}

static void xhci_pci_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	struct pci_dev		*pdev = to_pci_dev(dev);

	/* Look for vendor-specific quirks */
	if (pdev->vendor == PCI_VENDOR_ID_FRESCO_LOGIC &&
			(pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_PDK ||
			 pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_FL1400)) {
		if (pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_PDK &&
				pdev->revision == 0x0) {
			xhci->quirks |= XHCI_RESET_EP_QUIRK;
			xhci_dbg_trace(xhci, trace_xhci_dbg_quirks,
				"QUIRK: Fresco Logic xHC needs configure"
				" endpoint cmd after reset endpoint");
		}
		if (pdev->device == PCI_DEVICE_ID_FRESCO_LOGIC_PDK &&
				pdev->revision == 0x4) {
			xhci->quirks |= XHCI_SLOW_SUSPEND;
			xhci_dbg_trace(xhci, trace_xhci_dbg_quirks,
				"QUIRK: Fresco Logic xHC revision %u"
				"must be suspended extra slowly",
				pdev->revision);
		}
		/* Fresco Logic confirms: all revisions of this chip do not
		 * support MSI, even though some of them claim to in their PCI
		 * capabilities.
		 */
		xhci->quirks |= XHCI_BROKEN_MSI;
		xhci_dbg_trace(xhci, trace_xhci_dbg_quirks,
				"QUIRK: Fresco Logic revision %u "
				"has broken MSI implementation",
				pdev->revision);
		xhci->quirks |= XHCI_TRUST_TX_LENGTH;
	}

	if (pdev->vendor == PCI_VENDOR_ID_NEC)
		xhci->quirks |= XHCI_NEC_HOST;

	if (pdev->vendor == PCI_VENDOR_ID_AMD && xhci->hci_version == 0x96)
		xhci->quirks |= XHCI_AMD_0x96_HOST;

	/* AMD PLL quirk */
	if (pdev->vendor == PCI_VENDOR_ID_AMD && usb_amd_find_chipset_info())
		xhci->quirks |= XHCI_AMD_PLL_FIX;

	if (pdev->vendor == PCI_VENDOR_ID_AMD)
		xhci->quirks |= XHCI_TRUST_TX_LENGTH;

	if (pdev->vendor == PCI_VENDOR_ID_INTEL) {
		xhci->quirks |= XHCI_LPM_SUPPORT;
		xhci->quirks |= XHCI_INTEL_HOST;
		xhci->quirks |= XHCI_AVOID_BEI;
	}
	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
			pdev->device == PCI_DEVICE_ID_INTEL_PANTHERPOINT_XHCI) {
		xhci->quirks |= XHCI_EP_LIMIT_QUIRK;
		xhci->limit_active_eps = 64;
		xhci->quirks |= XHCI_SW_BW_CHECKING;
		/*
		 * PPT desktop boards DH77EB and DH77DF will power back on after
		 * a few seconds of being shutdown.  The fix for this is to
		 * switch the ports from xHCI to EHCI on shutdown.  We can't use
		 * DMI information to find those particular boards (since each
		 * vendor will change the board name), so we have to key off all
		 * PPT chipsets.
		 */
		xhci->quirks |= XHCI_SPURIOUS_REBOOT;
	}
	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
	    (pdev->device == PCI_DEVICE_ID_INTEL_LYNXPOINT_XHCI ||
	     pdev->device == PCI_DEVICE_ID_INTEL_LYNXPOINT_LP_XHCI)) {
		/* Workaround for occasional spurious wakeups from S5 (or
		 * any other sleep) on Haswell machines with LPT and LPT-LP
		 * with the new Intel BIOS
		 */
		/* Limit the quirk to only known vendors, as this triggers
		 * yet another BIOS bug on some other machines
		 * https://bugzilla.kernel.org/show_bug.cgi?id=66171
		 */
		if (pdev->subsystem_vendor == PCI_VENDOR_ID_HP)
			xhci->quirks |= XHCI_SPURIOUS_WAKEUP;

		xhci->quirks |= XHCI_SPURIOUS_REBOOT;
	}
	if (pdev->vendor == PCI_VENDOR_ID_INTEL &&
			pdev->device == PCI_DEVICE_ID_INTEL_CHT_XHCI) {
		xhci->quirks |= XHCI_SPURIOUS_PME;
		xhci->quirks |= XHCI_PIPE_4_1_SYNC_PHYSTAT_TOGGLE;

		/* Initialize the Disable Stall quirk if necessary */
		if (xhci_intel_need_disable_stall(xhci))
			xhci->quirks |= XHCI_SSIC_DISABLE_STALL;
	}

	if (pdev->vendor == PCI_VENDOR_ID_ETRON &&
			pdev->device == PCI_DEVICE_ID_ASROCK_P67) {
		xhci->quirks |= XHCI_RESET_ON_RESUME;
		xhci_dbg_trace(xhci, trace_xhci_dbg_quirks,
				"QUIRK: Resetting on resume");
		xhci->quirks |= XHCI_TRUST_TX_LENGTH;
	}
	if (pdev->vendor == PCI_VENDOR_ID_RENESAS &&
			pdev->device == 0x0015)
		xhci->quirks |= XHCI_RESET_ON_RESUME;
	if (pdev->vendor == PCI_VENDOR_ID_VIA)
		xhci->quirks |= XHCI_RESET_ON_RESUME;
}

/* called during probe() after chip reset completes */
static int xhci_pci_setup(struct usb_hcd *hcd)
{
	struct xhci_hcd		*xhci;
	struct pci_dev		*pdev = to_pci_dev(hcd->self.controller);
	int			retval;

	retval = xhci_gen_setup(hcd, xhci_pci_quirks);
	if (retval)
		return retval;

	xhci = hcd_to_xhci(hcd);
	if (!usb_hcd_is_primary_hcd(hcd))
		return 0;

	pci_read_config_byte(pdev, XHCI_SBRN_OFFSET, &xhci->sbrn);
	xhci_dbg(xhci, "Got SBRN %u\n", (unsigned int) xhci->sbrn);

	/* Find any debug ports */
	retval = xhci_pci_reinit(xhci, pdev);
	if (!retval)
		return retval;

	kfree(xhci);
	return retval;
}

/*
 * We need to register our own PCI probe function (instead of the USB core's
 * function) in order to create a second roothub under xHCI.
 */
static int xhci_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int retval;
	struct xhci_hcd *xhci;
	struct hc_driver *driver;
	struct usb_hcd *hcd;

	driver = (struct hc_driver *)id->driver_data;

	/* Prevent runtime suspending between USB-2 and USB-3 initialization */
	pm_runtime_get_noresume(&dev->dev);

	/* Register the USB 2.0 roothub.
	 * FIXME: USB core must know to register the USB 2.0 roothub first.
	 * This is sort of silly, because we could just set the HCD driver flags
	 * to say USB 2.0, but I'm not sure what the implications would be in
	 * the other parts of the HCD code.
	 */
	retval = usb_hcd_pci_probe(dev, id);

	if (retval)
		goto put_runtime_pm;

	/* USB 2.0 roothub is stored in the PCI device now. */
	hcd = dev_get_drvdata(&dev->dev);
	xhci = hcd_to_xhci(hcd);
	xhci->shared_hcd = usb_create_shared_hcd(driver, &dev->dev,
				pci_name(dev), hcd);
	if (!xhci->shared_hcd) {
		retval = -ENOMEM;
		goto dealloc_usb2_hcd;
	}

	/* Set the xHCI pointer before xhci_pci_setup() (aka hcd_driver.reset)
	 * is called by usb_add_hcd().
	 */
	*((struct xhci_hcd **) xhci->shared_hcd->hcd_priv) = xhci;
	p_xhci = xhci;
	retval = usb_add_hcd(xhci->shared_hcd, dev->irq,
			IRQF_SHARED);
	if (retval)
		goto put_usb3_hcd;
	/* Roothub already marked as USB 3.0 speed */

	if (device_create_file(&dev->dev, &dev_attr_ssic_port_enable))
		dev_err(&dev->dev, "can't create ssic_port_enable attribute\n");

	/* USB-2 and USB-3 roothubs initialized, allow runtime pm suspend */
	pm_runtime_put_noidle(&dev->dev);
	return 0;

put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);
dealloc_usb2_hcd:
	usb_hcd_pci_remove(dev);
put_runtime_pm:
	pm_runtime_put_noidle(&dev->dev);
	return retval;
}

static void xhci_pci_remove(struct pci_dev *dev)
{
	struct xhci_hcd *xhci;

	xhci = hcd_to_xhci(pci_get_drvdata(dev));
	if (xhci->shared_hcd) {
		usb_remove_hcd(xhci->shared_hcd);
		usb_put_hcd(xhci->shared_hcd);
	}
	usb_hcd_pci_remove(dev);

	/* Workaround for spurious wakeups at shutdown with HSW */
	if (xhci->quirks & XHCI_SPURIOUS_WAKEUP)
		pci_set_power_state(dev, PCI_D3hot);

	kfree(xhci);
}

#ifdef CONFIG_PM
static int xhci_pci_suspend(struct usb_hcd *hcd, bool do_wakeup)
{
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	struct pci_dev		*pdev = to_pci_dev(hcd->self.controller);
	int			retval = 0;

	/*
	 * Systems with the TI redriver that loses port status change events
	 * need to have the registers polled during D3, so avoid D3cold.
	 */
	if (xhci_compliance_mode_recovery_timer_quirk_check())
		pdev->no_d3cold = true;

	retval = xhci_suspend(xhci, do_wakeup);

	/* This is SW workaround for spurious PME issue and HCRST hang problem
	 * It required to set anc clear SSIC_PORT_UNUSED bit in D3 entry and
	 * D3 exit. */
	if (!retval && xhci->quirks & XHCI_SPURIOUS_PME)
		xhci_intel_ssic_port_unused(xhci, 1);

	return retval;
}

static int xhci_pci_resume(struct usb_hcd *hcd, bool hibernated)
{
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);
	struct pci_dev		*pdev = to_pci_dev(hcd->self.controller);
	int			retval = 0;

	/* Due to one HW bug, XHCI will keep generating PME wakeups and fail
	 * to stay in runtime suspended state, so required to clear the internal
	 * PME flag once it is back to D0 as the software workaround */
	if (xhci->quirks & XHCI_SPURIOUS_PME) {
		xhci_intel_clr_internal_pme_flag(xhci);
		xhci_intel_ssic_port_unused(xhci, 0);
	}

	/* The BIOS on systems with the Intel Panther Point chipset may or may
	 * not support xHCI natively.  That means that during system resume, it
	 * may switch the ports back to EHCI so that users can use their
	 * keyboard to select a kernel from GRUB after resume from hibernate.
	 *
	 * The BIOS is supposed to remember whether the OS had xHCI ports
	 * enabled before resume, and switch the ports back to xHCI when the
	 * BIOS/OS semaphore is written, but we all know we can't trust BIOS
	 * writers.
	 *
	 * Unconditionally switch the ports back to xHCI after a system resume.
	 * It should not matter whether the EHCI or xHCI controller is
	 * resumed first. It's enough to do the switchover in xHCI because
	 * USB core won't notice anything as the hub driver doesn't start
	 * running again until after all the devices (including both EHCI and
	 * xHCI host controllers) have been resumed.
	 */

	if (pdev->vendor == PCI_VENDOR_ID_INTEL)
		usb_enable_intel_xhci_ports(pdev);

	retval = xhci_resume(xhci, hibernated);
	return retval;
}
#endif /* CONFIG_PM */

static const struct hc_driver xhci_pci_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"xHCI Host Controller",
	.hcd_priv_size =	sizeof(struct xhci_hcd *),

	/*
	 * generic hardware linkage
	 */
	.irq =			xhci_irq,
	.flags =		HCD_MEMORY | HCD_USB3 | HCD_SHARED,

	/*
	 * basic lifecycle operations
	 */
	.reset =		xhci_pci_setup,
	.start =		xhci_run,
#ifdef CONFIG_PM
	.pci_suspend =          xhci_pci_suspend,
	.pci_resume =           xhci_pci_resume,
#endif
	.stop =			xhci_stop,
	.shutdown =		xhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		xhci_urb_enqueue,
	.urb_dequeue =		xhci_urb_dequeue,
	.alloc_dev =		xhci_alloc_dev,
	.free_dev =		xhci_free_dev,
	.alloc_streams =	xhci_alloc_streams,
	.free_streams =		xhci_free_streams,
	.add_endpoint =		xhci_add_endpoint,
	.drop_endpoint =	xhci_drop_endpoint,
	.endpoint_reset =	xhci_endpoint_reset,
	.check_bandwidth =	xhci_check_bandwidth,
	.reset_bandwidth =	xhci_reset_bandwidth,
	.address_device =	xhci_address_device,
	.enable_device =	xhci_enable_device,
	.update_hub_device =	xhci_update_hub_device,
	.reset_device =		xhci_discover_or_reset_device,

	/*
	 * scheduling support
	 */
	.get_frame_number =	xhci_get_frame,

	/* Root hub support */
	.hub_control =		xhci_hub_control,
	.hub_status_data =	xhci_hub_status_data,
	.bus_suspend =		xhci_bus_suspend,
	.bus_resume =		xhci_bus_resume,
	/*
	 * call back when device connected and addressed
	 */
	.update_device =        xhci_update_device,
	.set_usb2_hw_lpm =	xhci_set_usb2_hardware_lpm,
	.enable_usb3_lpm_timeout =	xhci_enable_usb3_lpm_timeout,
	.disable_usb3_lpm_timeout =	xhci_disable_usb3_lpm_timeout,
	.find_raw_port_number =	xhci_find_raw_port_number,
};

/*-------------------------------------------------------------------------*/

/* PCI driver selection metadata; PCI hotplugging uses this */
static const struct pci_device_id pci_ids[] = { {
	/* handle any USB 3.0 xHCI controller */
	PCI_DEVICE_CLASS(PCI_CLASS_SERIAL_USB_XHCI, ~0),
	.driver_data =	(unsigned long) &xhci_pci_hc_driver,
	},
	{ /* end: all zeroes */ }
};
MODULE_DEVICE_TABLE(pci, pci_ids);

/* pci driver glue; this is a "new style" PCI driver module */
static struct pci_driver xhci_pci_driver = {
	.name =		(char *) hcd_name,
	.id_table =	pci_ids,

	.probe =	xhci_pci_probe,
	.remove =	xhci_pci_remove,
	/* suspend and resume implemented later */

	.shutdown = 	usb_hcd_pci_shutdown,
#ifdef CONFIG_PM
	.driver = {
		.pm = &usb_hcd_pci_pm_ops
	},
#endif
};

int __init xhci_register_pci(void)
{
	return pci_register_driver(&xhci_pci_driver);
}

void xhci_unregister_pci(void)
{
	pci_unregister_driver(&xhci_pci_driver);
}
