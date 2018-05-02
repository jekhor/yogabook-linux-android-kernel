/*
 *  button.c - ACPI Button Driver
 *
 *  Copyright (C) 2001, 2002 Andy Grover <andrew.grover@intel.com>
 *  Copyright (C) 2001, 2002 Paul Diefenbaugh <paul.s.diefenbaugh@intel.com>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <acpi/button.h>

#define PREFIX "ACPI: "

#define ACPI_BUTTON_CLASS		"button"
#define ACPI_BUTTON_FILE_INFO		"info"
#define ACPI_BUTTON_FILE_STATE		"state"
#define ACPI_BUTTON_TYPE_UNKNOWN	0x00
#define ACPI_BUTTON_NOTIFY_STATUS	0x80

#define ACPI_BUTTON_SUBCLASS_POWER	"power"
#define ACPI_BUTTON_HID_POWER		"PNP0C0C"
#define ACPI_BUTTON_DEVICE_NAME_POWER	"Power Button"
#define ACPI_BUTTON_TYPE_POWER		0x01

#define ACPI_BUTTON_SUBCLASS_SLEEP	"sleep"
#define ACPI_BUTTON_HID_SLEEP		"PNP0C0E"
#define ACPI_BUTTON_DEVICE_NAME_SLEEP	"Sleep Button"
#define ACPI_BUTTON_TYPE_SLEEP		0x03

#define ACPI_BUTTON_SUBCLASS_LID	"lid"
#define ACPI_BUTTON_HID_LID		"PNP0C0D"
#define ACPI_BUTTON_DEVICE_NAME_LID	"Lid Switch"
#define ACPI_BUTTON_TYPE_LID		0x05

/* Intel-defined power button interface implemented via the _DSM
 * method on the PWRB object.  It has three functions as of version 0:
 * the first returns a bitmask of supported functions, the second
 * registers to receive an ACPI notification on power button release
 * events, and the third is a synchronous poll of power button state.
 * Not all systems will support notification. */
#define ACPI_PWRB_DSM_UUID "9c355bcb-35fa-44f7-8a67-447359c36a03"
#define ACPI_PWRB_DSM_VERSION 0
#define ACPI_PWRB_DSM_RELEASE 0xc0

enum {
	ACPI_PWRB_DSM_PROBE	= 0,
	ACPI_PWRB_DSM_REGISTER	= 1,
	ACPI_PWRB_DSM_LEVEL	= 2,
};

#define _COMPONENT		ACPI_BUTTON_COMPONENT
ACPI_MODULE_NAME("button");

MODULE_AUTHOR("Paul Diefenbaugh");
MODULE_DESCRIPTION("ACPI Button Driver");
MODULE_LICENSE("GPL");

static bool dsm_notify = true;
module_param(dsm_notify, bool, 0644);
MODULE_PARM_DESC(dsm_notify, "Enable _DSM notification of power button release");

static bool dsm_poll = true;
module_param(dsm_poll, bool, 0644);
MODULE_PARM_DESC(dsm_poll, "Enable _DSM polling for power button release");

static const struct acpi_device_id button_device_ids[] = {
	{ACPI_BUTTON_HID_LID,    0},
	{ACPI_BUTTON_HID_SLEEP,  0},
	{ACPI_BUTTON_HID_SLEEPF, 0},
	{ACPI_BUTTON_HID_POWER,  0},
	{ACPI_BUTTON_HID_POWERF, 0},
	{"", 0},
};
MODULE_DEVICE_TABLE(acpi, button_device_ids);

static int acpi_button_add(struct acpi_device *device);
static int acpi_button_remove(struct acpi_device *device);
static void acpi_button_notify(struct acpi_device *device, u32 event);

#ifdef CONFIG_PM_SLEEP
static int acpi_button_resume(struct device *dev);
#else
#define acpi_button_resume NULL
#endif
static SIMPLE_DEV_PM_OPS(acpi_button_pm, NULL, acpi_button_resume);

static struct acpi_driver acpi_button_driver = {
	.name = "button",
	.class = ACPI_BUTTON_CLASS,
	.ids = button_device_ids,
	.ops = {
		.add = acpi_button_add,
		.remove = acpi_button_remove,
		.notify = acpi_button_notify,
	},
	.drv.pm = &acpi_button_pm,
};

struct acpi_button {
	unsigned int type;
	struct acpi_device *acpi_dev;
	struct input_dev *input;
	char phys[32];			/* for input device */
	unsigned long pushed;
	bool dsm_notify;
	bool dsm_poll;
};

static BLOCKING_NOTIFIER_HEAD(acpi_lid_notifier);
static struct acpi_device *lid_device;

static struct acpi_pwrbtn_poll_dev *pwrbtn_poll;
static DEFINE_SPINLOCK(pwrbtn_lock);

/* Polling frequency.  Intel ICH hardware documents a 12ms debounce
 * timer, and 60Hz is about right generally. */
#define PWRBTN_POLL_JIFFIES	max(HZ/60, 1)

/* Safety valve vs. buggy (potentially in firmware) power button
 * polling implementations.  4 seconds is the default system reset
 * timer on Intel hardware. */
#define PWRBTN_POLL_MAX		(HZ*4)

/* --------------------------------------------------------------------------
                              FS Interface (/proc)
   -------------------------------------------------------------------------- */

static struct proc_dir_entry *acpi_button_dir;
static struct proc_dir_entry *acpi_lid_dir;

static int acpi_button_state_seq_show(struct seq_file *seq, void *offset)
{
	struct acpi_device *device = seq->private;
	acpi_status status;
	unsigned long long state;

	status = acpi_evaluate_integer(device->handle, "_LID", NULL, &state);
	seq_printf(seq, "state:      %s\n",
		   ACPI_FAILURE(status) ? "unsupported" :
			(state ? "open" : "closed"));
	return 0;
}

static int acpi_button_state_open_fs(struct inode *inode, struct file *file)
{
	return single_open(file, acpi_button_state_seq_show, PDE_DATA(inode));
}

static const struct file_operations acpi_button_state_fops = {
	.owner = THIS_MODULE,
	.open = acpi_button_state_open_fs,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int acpi_button_add_fs(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);
	struct proc_dir_entry *entry = NULL;
	int ret = 0;

	/* procfs I/F for ACPI lid device only */
	if (button->type != ACPI_BUTTON_TYPE_LID)
		return 0;

	if (acpi_button_dir || acpi_lid_dir) {
		printk(KERN_ERR PREFIX "More than one Lid device found!\n");
		return -EEXIST;
	}

	/* create /proc/acpi/button */
	acpi_button_dir = proc_mkdir(ACPI_BUTTON_CLASS, acpi_root_dir);
	if (!acpi_button_dir)
		return -ENODEV;

	/* create /proc/acpi/button/lid */
	acpi_lid_dir = proc_mkdir(ACPI_BUTTON_SUBCLASS_LID, acpi_button_dir);
	if (!acpi_lid_dir) {
		ret = -ENODEV;
		goto remove_button_dir;
	}

	/* create /proc/acpi/button/lid/LID/ */
	acpi_device_dir(device) = proc_mkdir(acpi_device_bid(device), acpi_lid_dir);
	if (!acpi_device_dir(device)) {
		ret = -ENODEV;
		goto remove_lid_dir;
	}

	/* create /proc/acpi/button/lid/LID/state */
	entry = proc_create_data(ACPI_BUTTON_FILE_STATE,
				 S_IRUGO, acpi_device_dir(device),
				 &acpi_button_state_fops, device);
	if (!entry) {
		ret = -ENODEV;
		goto remove_dev_dir;
	}

done:
	return ret;

remove_dev_dir:
	remove_proc_entry(acpi_device_bid(device),
			  acpi_lid_dir);
	acpi_device_dir(device) = NULL;
remove_lid_dir:
	remove_proc_entry(ACPI_BUTTON_SUBCLASS_LID, acpi_button_dir);
remove_button_dir:
	remove_proc_entry(ACPI_BUTTON_CLASS, acpi_root_dir);
	goto done;
}

static int acpi_button_remove_fs(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);

	if (button->type != ACPI_BUTTON_TYPE_LID)
		return 0;

	remove_proc_entry(ACPI_BUTTON_FILE_STATE,
			  acpi_device_dir(device));
	remove_proc_entry(acpi_device_bid(device),
			  acpi_lid_dir);
	acpi_device_dir(device) = NULL;
	remove_proc_entry(ACPI_BUTTON_SUBCLASS_LID, acpi_button_dir);
	remove_proc_entry(ACPI_BUTTON_CLASS, acpi_root_dir);

	return 0;
}

/* --------------------------------------------------------------------------
                                Driver Interface
   -------------------------------------------------------------------------- */

static int pwrb_dsm_call(acpi_handle *handle, int func)
{
	u8 uuid[16];
	struct acpi_buffer buf = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_object_list olist;
	union acpi_object args[4], *out;
	acpi_status err;
	int i, ret = -1;

	acpi_str_to_uuid(ACPI_PWRB_DSM_UUID, uuid);
	args[0].type = ACPI_TYPE_BUFFER;
	args[0].buffer.length = sizeof(uuid);
	args[0].buffer.pointer = uuid;

	args[1].type = ACPI_TYPE_INTEGER;
	args[1].integer.value = ACPI_PWRB_DSM_VERSION;

	args[2].type = ACPI_TYPE_INTEGER;
	args[2].integer.value = func;

	args[3].type = ACPI_TYPE_PACKAGE;
	args[3].package.count = 0;
	args[3].package.elements = NULL;

	olist.count = ARRAY_SIZE(args);
	olist.pointer = args;

	if (func == ACPI_PWRB_DSM_PROBE) {
		/* PROBE returns a bitmask of supported functions as a
		 * buffer.  Return a failure here as 0: method not
		 * present or not functioning correctly means "no
		 * functions supported" */
		ret = 0;
		err = acpi_evaluate_object(handle, "_DSM", &olist, &buf);
		out = buf.pointer;
		if (!ACPI_FAILURE(err) && out->type == ACPI_TYPE_BUFFER) {
			for (i = 0; i < out->buffer.length; i++)
				ret |= (out->buffer.pointer[i] << (i*8));
			kfree(buf.pointer);
		}
	} else {
		unsigned long long acpiret;
		err = acpi_evaluate_integer(handle, "_DSM", &olist, &acpiret);
		if (ACPI_FAILURE(err))
			ret = -1;
		ret = (int)acpiret;
	}

	return ret;
}

static void pwrb_dsm_init(struct acpi_button *btn)
{
	struct acpi_device *dev = btn->acpi_dev;
	int funcs = pwrb_dsm_call(dev->handle, ACPI_PWRB_DSM_PROBE);

	if (dsm_notify && (funcs & ACPI_PWRB_DSM_REGISTER)) {
		if (pwrb_dsm_call(dev->handle, ACPI_PWRB_DSM_REGISTER))
			dev_err(&dev->dev, "PWRB release notification registration failed");
		else
			btn->dsm_notify = true;
	}

	if (dsm_poll && !btn->dsm_notify && (funcs & ACPI_PWRB_DSM_LEVEL))
		btn->dsm_poll = true;
}

static int pwrb_dsm_poll(struct acpi_button *button)
{
	struct acpi_device *dev = button->acpi_dev;
	return pwrb_dsm_call(dev->handle, ACPI_PWRB_DSM_LEVEL);
}

int acpi_lid_notifier_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&acpi_lid_notifier, nb);
}
EXPORT_SYMBOL(acpi_lid_notifier_register);

int acpi_lid_notifier_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&acpi_lid_notifier, nb);
}
EXPORT_SYMBOL(acpi_lid_notifier_unregister);

int acpi_lid_open(void)
{
	acpi_status status;
	unsigned long long state;

	if (!lid_device)
		return -ENODEV;

	status = acpi_evaluate_integer(lid_device->handle, "_LID", NULL,
				       &state);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	return !!state;
}
EXPORT_SYMBOL(acpi_lid_open);

static int acpi_lid_send_state(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);
	unsigned long long state;
	acpi_status status;
	int ret;

	status = acpi_evaluate_integer(device->handle, "_LID", NULL, &state);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	/* input layer checks if event is redundant */
	input_report_switch(button->input, SW_LID, !state);
	input_sync(button->input);

	if (state)
		pm_wakeup_event(&device->dev, 0);

	ret = blocking_notifier_call_chain(&acpi_lid_notifier, state, device);
	if (ret == NOTIFY_DONE)
		ret = blocking_notifier_call_chain(&acpi_lid_notifier, state,
						   device);
	if (ret == NOTIFY_DONE || ret == NOTIFY_OK) {
		/*
		 * It is also regarded as success if the notifier_chain
		 * returns NOTIFY_OK or NOTIFY_DONE.
		 */
		ret = 0;
	}
	return ret;
}

static void pwrbtn_timer(unsigned long arg)
{
	struct acpi_button *button = (struct acpi_button *)arg;
	int pressed;
	int toolong = (jiffies - pwrbtn_poll->started) > PWRBTN_POLL_MAX;
	unsigned long flags;

	if (toolong)
		dev_err(&button->input->dev, "Power button poll failed to detect release");

	spin_lock_irqsave(&pwrbtn_lock, flags);
	if (button->dsm_poll)
		pressed = pwrb_dsm_poll(button);
	else
		pressed = pwrbtn_poll->poll(pwrbtn_poll);
	spin_unlock_irqrestore(&pwrbtn_lock, flags);

	if (!pressed || toolong) {
		input_report_key(button->input, KEY_POWER, 0);
		input_sync(button->input);
	} else {
		mod_timer(&pwrbtn_poll->timer, jiffies + PWRBTN_POLL_JIFFIES);
	}
}

int acpi_pwrbtn_poll_register(struct acpi_pwrbtn_poll_dev *dev)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&pwrbtn_lock, flags);
	if (pwrbtn_poll)
		ret = -EBUSY;
	else
		pwrbtn_poll = dev;

	init_timer(&pwrbtn_poll->timer);
	pwrbtn_poll->timer.function = pwrbtn_timer;

	spin_unlock_irqrestore(&pwrbtn_lock, flags);
	return ret;
}

int acpi_pwrbtn_poll_unregister(struct acpi_pwrbtn_poll_dev *dev)
{
	unsigned long flags;
	if (!pwrbtn_poll || pwrbtn_poll != dev)
		return -EINVAL;
	del_timer_sync(&pwrbtn_poll->timer);
	spin_lock_irqsave(&pwrbtn_lock, flags);
	pwrbtn_poll = NULL;
	spin_unlock_irqrestore(&pwrbtn_lock, flags);
	return 0;
}

static int start_pwrbtn_poll(struct acpi_button *button)
{
	int ret;
	pwrbtn_poll->timer.data = (unsigned long)button;
	pwrbtn_poll->started = jiffies;
	return mod_timer(&pwrbtn_poll->timer, jiffies + PWRBTN_POLL_JIFFIES);
}

static void acpi_button_notify(struct acpi_device *device, u32 event)
{
	struct acpi_button *button = acpi_driver_data(device);
	struct input_dev *input;

	switch (event) {
	case ACPI_FIXED_HARDWARE_EVENT:
		event = ACPI_BUTTON_NOTIFY_STATUS;
		/* fall through */
	case ACPI_BUTTON_NOTIFY_STATUS:
		input = button->input;
		if (button->type == ACPI_BUTTON_TYPE_LID) {
			acpi_lid_send_state(device);
		} else {
			int keycode = test_bit(KEY_SLEEP, input->keybit) ?
						KEY_SLEEP : KEY_POWER;
			bool pwr = button->type == ACPI_BUTTON_TYPE_POWER;

			input_report_key(input, keycode, 1);
			input_sync(input);

			if (pwr && !button->dsm_notify
			    && (pwrbtn_poll || button->dsm_poll)) {
				if (start_pwrbtn_poll(button)) {
					/* Error, fall back to
					 * synchronous event */
					input_report_key(input, keycode, 0);
					input_sync(input);
				}
			} else if (!pwr || !button->dsm_notify) {
				/* No release detection; emit the UP
				 * synchronously */
				input_report_key(input, keycode, 0);
				input_sync(input);
			}

			pm_wakeup_event(&device->dev, 0);
			acpi_bus_generate_netlink_event(
					device->pnp.device_class,
					dev_name(&device->dev),
					event, ++button->pushed);
		}
		break;
	case ACPI_PWRB_DSM_RELEASE:
		input_report_key(button->input, KEY_POWER, 0);
		input_sync(button->input);
		break;
	default:
		ACPI_DEBUG_PRINT((ACPI_DB_INFO,
				  "Unsupported event [0x%x]\n", event));
		break;
	}
}

#ifdef CONFIG_PM_SLEEP
static int acpi_button_resume(struct device *dev)
{
	struct acpi_device *device = to_acpi_device(dev);
	struct acpi_button *button = acpi_driver_data(device);

	if (button->type == ACPI_BUTTON_TYPE_LID)
		return acpi_lid_send_state(device);
	return 0;
}
#endif

static int acpi_button_add(struct acpi_device *device)
{
	struct acpi_button *button;
	struct input_dev *input;
	const char *hid = acpi_device_hid(device);
	char *name, *class;
	int error;

	button = kzalloc(sizeof(struct acpi_button), GFP_KERNEL);
	if (!button)
		return -ENOMEM;

	button->acpi_dev = device;
	device->driver_data = button;

	button->input = input = input_allocate_device();
	if (!input) {
		error = -ENOMEM;
		goto err_free_button;
	}

	name = acpi_device_name(device);
	class = acpi_device_class(device);

	if (!strcmp(hid, ACPI_BUTTON_HID_POWER) ||
	    !strcmp(hid, ACPI_BUTTON_HID_POWERF)) {
		button->type = ACPI_BUTTON_TYPE_POWER;
		strcpy(name, ACPI_BUTTON_DEVICE_NAME_POWER);
		sprintf(class, "%s/%s",
			ACPI_BUTTON_CLASS, ACPI_BUTTON_SUBCLASS_POWER);
		pwrb_dsm_init(button);

	} else if (!strcmp(hid, ACPI_BUTTON_HID_SLEEP) ||
		   !strcmp(hid, ACPI_BUTTON_HID_SLEEPF)) {
		button->type = ACPI_BUTTON_TYPE_SLEEP;
		strcpy(name, ACPI_BUTTON_DEVICE_NAME_SLEEP);
		sprintf(class, "%s/%s",
			ACPI_BUTTON_CLASS, ACPI_BUTTON_SUBCLASS_SLEEP);
	} else if (!strcmp(hid, ACPI_BUTTON_HID_LID)) {
		button->type = ACPI_BUTTON_TYPE_LID;
		strcpy(name, ACPI_BUTTON_DEVICE_NAME_LID);
		sprintf(class, "%s/%s",
			ACPI_BUTTON_CLASS, ACPI_BUTTON_SUBCLASS_LID);
	} else {
		printk(KERN_ERR PREFIX "Unsupported hid [%s]\n", hid);
		error = -ENODEV;
		goto err_free_input;
	}

	error = acpi_button_add_fs(device);
	if (error)
		goto err_free_input;

	snprintf(button->phys, sizeof(button->phys), "%s/button/input0", hid);

	input->name = name;
	input->phys = button->phys;
	input->id.bustype = BUS_HOST;
	input->id.product = button->type;
	input->dev.parent = &device->dev;

	switch (button->type) {
	case ACPI_BUTTON_TYPE_POWER:
		input_set_capability(input, EV_KEY, KEY_POWER);
		break;

	case ACPI_BUTTON_TYPE_SLEEP:
		input_set_capability(input, EV_KEY, KEY_SLEEP);
		break;

	case ACPI_BUTTON_TYPE_LID:
		input_set_capability(input, EV_SW, SW_LID);
		break;
	}

	error = input_register_device(input);
	if (error)
		goto err_remove_fs;
	if (button->type == ACPI_BUTTON_TYPE_LID) {
		acpi_lid_send_state(device);
		/*
		 * This assumes there's only one lid device, or if there are
		 * more we only care about the last one...
		 */
		lid_device = device;
	}

	printk(KERN_INFO PREFIX "%s [%s]\n", name, acpi_device_bid(device));
	return 0;

 err_remove_fs:
	acpi_button_remove_fs(device);
 err_free_input:
	input_free_device(input);
 err_free_button:
	kfree(button);
	return error;
}

static int acpi_button_remove(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);

	acpi_button_remove_fs(device);
	input_unregister_device(button->input);
	kfree(button);
	return 0;
}

module_acpi_driver(acpi_button_driver);
