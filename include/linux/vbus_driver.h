/*
 * Copyright 2009 Novell.  All Rights Reserved.
 *
 * Mediates access to a host VBUS from a guest kernel by providing a
 * global view of all VBUS devices
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

#ifndef _LINUX_VBUS_DRIVER_H
#define _LINUX_VBUS_DRIVER_H

#include <linux/device.h>
#include <linux/shm_signal.h>
#include <linux/ioq.h>

struct vbus_device_proxy;
struct vbus_driver;

struct vbus_device_proxy_ops {
	int (*open)(struct vbus_device_proxy *dev, int version, int flags);
	int (*close)(struct vbus_device_proxy *dev, int flags);
	int (*shm)(struct vbus_device_proxy *dev, const char *name,
		   int id, int prio,
		   void *ptr, size_t len,
		   struct shm_signal_desc *sigdesc, struct shm_signal **signal,
		   int flags);
	int (*call)(struct vbus_device_proxy *dev, u32 func,
		    void *data, size_t len, int flags);
	void (*release)(struct vbus_device_proxy *dev);
};

struct vbus_device_proxy {
	char                          *type;
	u64                            id;
	void                          *priv; /* Used by drivers */
	struct vbus_device_proxy_ops  *ops;
	struct device                  dev;
};

int vbus_device_proxy_register(struct vbus_device_proxy *dev);
void vbus_device_proxy_unregister(struct vbus_device_proxy *dev);

struct vbus_device_proxy *vbus_device_proxy_find(u64 id);

struct vbus_driver_ops {
	int (*probe)(struct vbus_device_proxy *dev);
	int (*remove)(struct vbus_device_proxy *dev);
};

struct vbus_driver {
	char                          *type;
	struct module                 *owner;
	struct vbus_driver_ops        *ops;
	struct device_driver           drv;
};

int vbus_driver_register(struct vbus_driver *drv);
void vbus_driver_unregister(struct vbus_driver *drv);

/*
 * driver-side IOQ helper - allocates device-shm and maps an IOQ on it
 */
int vbus_driver_ioq_alloc(struct vbus_device_proxy *dev, const char *name,
			  int id, int prio, size_t ringsize, struct ioq **ioq);

#endif /* _LINUX_VBUS_DRIVER_H */
