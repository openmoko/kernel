/*
 * Copyright 2009 Novell.  All Rights Reserved.
 *
 * PCI to Virtual-Bus Bridge
 *
 * Author:
 *      Gregory Haskins <ghaskins@novell.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef _LINUX_VBUS_PCI_H
#define _LINUX_VBUS_PCI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define VBUS_PCI_ABI_MAGIC 0xbf53eef5
#define VBUS_PCI_ABI_VERSION 2
#define VBUS_PCI_HC_VERSION 1

enum {
	VBUS_PCI_BRIDGE_NEGOTIATE,
	VBUS_PCI_BRIDGE_QREG,
	VBUS_PCI_BRIDGE_SLOWCALL,
	VBUS_PCI_BRIDGE_FASTCALL_ADD,
	VBUS_PCI_BRIDGE_FASTCALL_DROP,

	VBUS_PCI_BRIDGE_MAX, /* must be last */
};

enum {
	VBUS_PCI_HC_DEVOPEN,
	VBUS_PCI_HC_DEVCLOSE,
	VBUS_PCI_HC_DEVCALL,
	VBUS_PCI_HC_DEVSHM,

	VBUS_PCI_HC_MAX,      /* must be last */
};

struct vbus_pci_bridge_negotiate {
	__u32 magic;
	__u32 version;
	__u64 capabilities;
};

struct vbus_pci_deviceopen {
	__u32 devid;
	__u32 version; /* device ABI version */
	__u64 handle; /* return value for devh */
};

struct vbus_pci_devicecall {
	__u64 devh;   /* device-handle (returned from DEVICEOPEN */
	__u32 func;
	__u32 len;
	__u32 flags;
	__u64 datap;
};

struct vbus_pci_deviceshm {
	__u64 devh;   /* device-handle (returned from DEVICEOPEN */
	__u32 id;
	__u32 len;
	__u32 flags;
	struct {
		__u32 offset;
		__u32 prio;
		__u64 cookie; /* token to pass back when signaling client */
	} signal;
	__u64 datap;
};

struct vbus_pci_call_desc {
	__u32 vector;
	__u32 len;
	__u64 datap;
};

struct vbus_pci_fastcall_desc {
	struct vbus_pci_call_desc call;
	__u32                     result;
};

struct vbus_pci_regs {
	struct vbus_pci_call_desc bridgecall;
	__u8                      pad[48];
};

struct vbus_pci_signals {
	__u32 eventq;
	__u32 fastcall;
	__u32 shmsignal;
	__u8  pad[20];
};

struct vbus_pci_eventqreg {
	__u32 count;
	__u64 ring;
	__u64 data;
};

struct vbus_pci_busreg {
	__u32 count;  /* supporting multiple queues allows for prio, etc */
	struct vbus_pci_eventqreg eventq[1];
};

enum vbus_pci_eventid {
	VBUS_PCI_EVENT_DEVADD,
	VBUS_PCI_EVENT_DEVDROP,
	VBUS_PCI_EVENT_SHMSIGNAL,
	VBUS_PCI_EVENT_SHMCLOSE,
};

#define VBUS_MAX_DEVTYPE_LEN 128

struct vbus_pci_add_event {
	__u64 id;
	char  type[VBUS_MAX_DEVTYPE_LEN];
};

struct vbus_pci_handle_event {
	__u64 handle;
};

struct vbus_pci_event {
	__u32 eventid;
	union {
		struct vbus_pci_add_event    add;
		struct vbus_pci_handle_event handle;
	} data;
};

#endif /* _LINUX_VBUS_PCI_H */
