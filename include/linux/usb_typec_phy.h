/*
 * usb_typec_phy.h: usb type-c phy layer header file
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. Seee the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Kannappan, R <r.kannappan@intel.com>
 */

#ifndef __USB_TYPEC_PHY_H__
#define __USB_TYPEC_PHY_H__

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/errno.h>

enum typec_state {
	TYPEC_STATE_UNKNOWN,
	TYPEC_STATE_POWERED,
	TYPEC_STATE_UNATTACHED_UFP,
	TYPEC_STATE_UNATTACHED_DFP,
	TYPEC_STATE_ATTACHED_UFP,
	TYPEC_STATE_ATTACHED_DFP,
	TYPEC_STATE_ATTACHED_AUDIO_ACC,
	TYPEC_STATE_ATTACHED_DEBUG_ACC,
	/* CC pull-down to pull-up during PR SWAP*/
	TYPEC_STATE_PD_PU_SWAP,
	/* CC pull-up to pull-down during PR SWAP*/
	TYPEC_STATE_PU_PD_SWAP,
};

enum typec_cc_pull {
	TYPEC_CC_PULL_NONE,
	TYPEC_CC_PULL_UP,
	TYPEC_CC_PULL_DOWN,
};

enum typec_cc_pin {
	TYPEC_PIN_CC1 = 1,
	TYPEC_PIN_CC2 = 2,
};

enum typec_orientation {
	TYPEC_POS_DISCONNECT,
	TYPEC_POS_NORMAL,
	TYPEC_POS_SWAP,
};

enum typec_current {
	TYPEC_CURRENT_UNKNOWN = 0,
	TYPEC_CURRENT_USB = 500,
	TYPEC_CURRENT_1500 = 1500,
	TYPEC_CURRENT_3000 = 3000
};

enum typec_cc_level {
	USB_TYPEC_CC_VRD_UNKNOWN = -1,
	USB_TYPEC_CC_VRA,
	USB_TYPEC_CC_VRD_USB,
	USB_TYPEC_CC_VRD_1500,
	USB_TYPEC_CC_VRD_3000,
};

enum typec_fifo {
	FIFO_TYPE_TX = 1,
	FIFO_TYPE_RX = 2,
};

enum typec_event {
	TYPEC_EVENT_UNKNOWN,
	TYPEC_EVENT_VBUS,
	TYPEC_EVENT_DRP,
	TYPEC_EVENT_UFP,
	TYPEC_EVENT_DFP,
	TYPEC_EVENT_TIMER,
	TYPEC_EVENT_NONE,
	TYPEC_EVENT_DEV_REMOVE,
};

enum typec_mode {
	TYPEC_MODE_UNKNOWN,
	TYPEC_MODE_DFP,
	TYPEC_MODE_UFP,
	TYPEC_MODE_DRP,
};

enum typec_type {
	USB_TYPE_C
};

enum typec_phy_evts {
	PROT_PHY_EVENT_NONE,
	PROT_PHY_EVENT_GOODCRC,
	PROT_PHY_EVENT_TX_SENT,
	PROT_PHY_EVENT_COLLISION,
	PROT_PHY_EVENT_HARD_RST,
	PROT_PHY_EVENT_SOFT_RST,
	PROT_PHY_EVENT_RESET,
	PROT_PHY_EVENT_TX_FAIL,
	PROT_PHY_EVENT_SOFT_RST_FAIL,
	PROT_PHY_EVENT_TX_HARD_RST,
	PROT_PHY_EVENT_GOODCRC_SENT,
	PROT_PHY_EVENT_MSG_RCV,
};

enum typec_phy_dpm_evts {
	PHY_DPM_EVENT_VBUS_ON,
	PHY_DPM_EVENT_VBUS_OFF,
};

enum {
	PD_DATA_ROLE_UFP,
	PD_DATA_ROLE_DFP,
};

enum {
	PD_PWR_ROLE_SINK,
	PD_PWR_ROLE_SRC,
};

enum {
	PD_POWER_ROLE_PROVIDER,
	PD_POWER_ROLE_PROVIER_CONSUMER,
	PD_POWER_ROLE_CONSUMER,
	PD_POWER_ROLE_CONSUMER_PROVIDER,
};

enum typec_cc_vrd {
	TYPEC_CC_VRD_UNKNOWN = -1,
	TYPEC_CC_VRA,
	TYPEC_CC_VRD_USB,
	TYPEC_CC_VRD_1500,
	TYPEC_CC_VRD_3000
};

enum typec_dp_cable_type {
	TYPEC_DP_TYPE_NONE,
	TYPEC_DP_TYPE_2X,
	TYPEC_DP_TYPE_4X,
};

struct typec_cc_psy {
	enum typec_cc_level v_rd;
	enum typec_current cur;
};

struct cc_pin {
	enum typec_cc_pin id;
	int valid;
	int rd;
	int cur;
};

#define MAX_LABEL_SIZE		16

struct typec_phy;

struct typec_ops {
	/* Callback for setting host-current */
	int (*set_host_current)(struct typec_phy *phy, enum typec_current cur);
	/* Callback for getting host-current */
	enum typec_current (*get_host_current)(struct typec_phy *phy);
	/* Callback for measuring cc */
	int (*measure_cc)(struct typec_phy *phy, struct cc_pin *pin);
	/* Callback for switching between pull-up & pull-down */
	int (*switch_mode)(struct typec_phy *phy, enum typec_mode mode);
	/* Callback for setting-up cc */
	int (*setup_cc)(struct typec_phy *phy, enum typec_cc_pin cc,
					enum typec_state state);
	int (*enable_valid_pu)(struct typec_phy *phy);
	bool (*is_vconn_enabled)(struct typec_phy *phy);
	int (*enable_vconn)(struct typec_phy *phy, bool en);
};


struct typec_phy {
	const char *label;
	struct device *dev;
	struct typec_ops ops;
	enum typec_type type;
	enum typec_state state;
	enum typec_cc_pin valid_cc;
	enum typec_dp_cable_type dp_type;
	int valid_rd;
	struct pd_prot *proto;
	struct list_head list;
	spinlock_t irq_lock;
	struct atomic_notifier_head notifier;
	struct atomic_notifier_head prot_notifier;
	struct cc_pin cc1;
	struct cc_pin cc2;

	bool support_drp_toggle;
	bool support_auto_goodcrc;
	bool support_retry;

	int (*notify_connect)(struct typec_phy *phy, enum typec_cc_level lvl);
	int (*notify_disconnect)(struct typec_phy *phy);

	int (*init)(struct typec_phy *phy);
	int (*shutdown)(struct typec_phy *phy);
	int (*get_pd_version)(struct typec_phy *phy);
	int (*phy_reset)(struct typec_phy *phy);
	int (*flush_fifo)(struct typec_phy *phy, enum typec_fifo fifo_type);
	int (*send_packet)(struct typec_phy *phy, u8 *msg, int len);
	int (*recv_packet)(struct typec_phy *phy, u8 *msg);
	int (*setup_role)(struct typec_phy *phy, int data_role, int pwr_role);
	void (*notify_protocol)(struct typec_phy *phy, unsigned long event);
	bool (*is_pd_capable)(struct typec_phy *phy);
	int (*enable_autocrc)(struct typec_phy *phy, bool en);
	int (*set_pu_pd)(struct typec_phy *phy, enum typec_cc_pull pull);
	int (*set_swap_state)(struct typec_phy *phy, bool swap);
	int (*enable_detection)(struct typec_phy *phy, bool en);
	bool (*is_vbus_on)(struct typec_phy *phy);
	int (*enable_auto_retry)(struct typec_phy *phy, bool en);
};

extern struct typec_phy *typec_get_phy(int type);
extern int typec_add_phy(struct typec_phy *phy);
extern int typec_remove_phy(struct typec_phy *phy);
struct typec_phy *typec_get_phy(int type);
int typec_get_cc_orientation(struct typec_phy *phy);

static inline int typec_phy_init(struct typec_phy *phy)
{
	if (phy && phy->init)
		return phy->init(phy);
	return -ENOTSUPP;
}

static inline int typec_phy_shutdown(struct typec_phy *phy)
{
	if (phy && phy->shutdown)
		return phy->shutdown(phy);
	return -ENOTSUPP;
}

static inline int typec_set_host_current(struct typec_phy *phy,
		enum typec_current cur)
{
	if (phy && phy->ops.set_host_current)
		return phy->ops.set_host_current(phy, cur);
	return -ENOTSUPP;
}

static inline bool typec_is_vconn_enabled(struct typec_phy *phy)
{
	if (phy && phy->ops.is_vconn_enabled)
		return phy->ops.is_vconn_enabled(phy);

	return false;
}

static inline int typec_enable_vconn(struct typec_phy *phy, bool en)
{
	if (phy && phy->ops.enable_vconn)
		return phy->ops.enable_vconn(phy, en);

	return -ENOTSUPP;
}

static inline int typec_enable_valid_pu(struct typec_phy *phy)
{
	if (phy && phy->ops.enable_valid_pu)
		return phy->ops.enable_valid_pu(phy);
	return -ENOTSUPP;
}

static inline enum typec_current typec_get_host_current(struct typec_phy *phy)
{
	if (phy && phy->ops.get_host_current)
		return phy->ops.get_host_current(phy);
	return -ENOTSUPP;
}

static inline int typec_notify_connect(struct typec_phy *phy,
			enum typec_cc_level lvl)
{
	if (phy && phy->notify_connect)
		return phy->notify_connect(phy, lvl);
	return -ENOTSUPP;
}

static inline int typec_notify_disconnect(struct typec_phy *phy)
{
	if (phy && phy->notify_disconnect)
		return phy->notify_disconnect(phy);
	return -ENOTSUPP;
}

static inline int
typec_measure_cc(struct typec_phy *phy, struct cc_pin *pin)
{
	if (phy && phy->ops.measure_cc)
		return phy->ops.measure_cc(phy, pin);
	return -ENOTSUPP;
}


static inline int
typec_register_notifier(struct typec_phy *phy, struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&phy->notifier, nb);
}

static inline int
typec_unregister_notifier(struct typec_phy *phy, struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&phy->notifier, nb);
}

static inline int
typec_switch_mode(struct typec_phy *phy, enum typec_mode mode)
{
	if (phy && phy->ops.switch_mode)
		return phy->ops.switch_mode(phy, mode);
	return -ENOTSUPP;
}

static inline int typec_setup_cc(struct typec_phy *phy, enum typec_cc_pin cc,
					enum typec_state state)
{
	if (phy && phy->ops.setup_cc)
		return phy->ops.setup_cc(phy, cc, state);
	return -ENOTSUPP;
}

static inline int typec_enable_autocrc(struct typec_phy *phy, bool en)
{
	if (phy && phy->enable_autocrc)
		return phy->enable_autocrc(phy, en);

	return -ENOTSUPP;
}

static inline int typec_enable_auto_retry(struct typec_phy *phy, bool en)
{
	if (phy && phy->enable_auto_retry)
		return phy->enable_auto_retry(phy, en);

	return -ENOTSUPP;
}

static inline int typec_set_swap_state(struct typec_phy *phy, bool swap)
{
	if (phy && phy->set_swap_state)
		return phy->set_swap_state(phy, swap);

	return -ENOTSUPP;
}

static inline int typec_register_prot_notifier(struct typec_phy *phy,
						struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&phy->prot_notifier, nb);
}

static inline int typec_unregister_prot_notifier(struct typec_phy *phy,
						struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&phy->prot_notifier, nb);
}

#endif /* __USB_TYPEC_PHY_H__ */
