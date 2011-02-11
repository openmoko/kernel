/*
 * Copyright 2009 Novell.  All Rights Reserved.
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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vbus_driver.h>

MODULE_AUTHOR("Gregory Haskins");
MODULE_LICENSE("GPL");

#define VBUS_PROXY_NAME "vbus-proxy"

static struct vbus_device_proxy *to_dev(struct device *_dev)
{
	return _dev ? container_of(_dev, struct vbus_device_proxy, dev) : NULL;
}

static struct vbus_driver *to_drv(struct device_driver *_drv)
{
	return container_of(_drv, struct vbus_driver, drv);
}

/*
 * This function is invoked whenever a new driver and/or device is added
 * to check if there is a match
 */
static int vbus_dev_proxy_match(struct device *_dev, struct device_driver *_drv)
{
	struct vbus_device_proxy *dev = to_dev(_dev);
	struct vbus_driver *drv = to_drv(_drv);

	return !strcmp(dev->type, drv->type);
}

static int vbus_dev_proxy_uevent(struct device *_dev, struct kobj_uevent_env *env)
{
	struct vbus_device_proxy *dev = to_dev(_dev);

	if (add_uevent_var(env, "MODALIAS=vbus-proxy:%s", dev->type))
		return -ENOMEM;

	return 0;
}

/*
 * This function is invoked after the bus infrastructure has already made a
 * match.  The device will contain a reference to the paired driver which
 * we will extract.
 */
static int vbus_dev_proxy_probe(struct device *_dev)
{
	int ret = 0;
	struct vbus_device_proxy *dev = to_dev(_dev);
	struct vbus_driver *drv = to_drv(_dev->driver);

	if (drv->ops->probe)
		ret = drv->ops->probe(dev);

	return ret;
}

static struct bus_type vbus_proxy = {
	.name   = VBUS_PROXY_NAME,
	.match  = vbus_dev_proxy_match,
	.uevent = vbus_dev_proxy_uevent,
};

static struct device vbus_proxy_rootdev = {
	.parent    = NULL,
	.init_name = VBUS_PROXY_NAME,
};

static int __init vbus_init(void)
{
	int ret;

	ret = bus_register(&vbus_proxy);
	BUG_ON(ret < 0);

	ret = device_register(&vbus_proxy_rootdev);
	BUG_ON(ret < 0);

	return 0;
}

postcore_initcall(vbus_init);

static void device_release(struct device *dev)
{
	struct vbus_device_proxy *_dev;

	_dev = container_of(dev, struct vbus_device_proxy, dev);

	_dev->ops->release(_dev);
}

static ssize_t _show_modalias(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "vbus-proxy:%s\n", to_dev(dev)->type);
}
static DEVICE_ATTR(modalias, S_IRUSR | S_IRGRP | S_IROTH, _show_modalias, NULL);

int vbus_device_proxy_register(struct vbus_device_proxy *new)
{
	int ret;

	new->dev.parent  = &vbus_proxy_rootdev;
	new->dev.bus     = &vbus_proxy;
	new->dev.release = &device_release;

	ret = device_register(&new->dev);
	if (ret < 0)
		return ret;

	ret = device_create_file(&new->dev, &dev_attr_modalias);
	if (ret < 0) {
		device_unregister(&new->dev);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(vbus_device_proxy_register);

void vbus_device_proxy_unregister(struct vbus_device_proxy *dev)
{
	device_remove_file(&dev->dev, &dev_attr_modalias);
	device_unregister(&dev->dev);
}
EXPORT_SYMBOL_GPL(vbus_device_proxy_unregister);

static int match_device_id(struct device *_dev, void *data)
{
	struct vbus_device_proxy *dev = to_dev(_dev);
	u64 id = *(u64 *)data;

	return dev->id == id;
}

struct vbus_device_proxy *vbus_device_proxy_find(u64 id)
{
	struct device *dev;

	dev = bus_find_device(&vbus_proxy, NULL, &id, &match_device_id);

	return to_dev(dev);
}
EXPORT_SYMBOL_GPL(vbus_device_proxy_find);

int vbus_driver_register(struct vbus_driver *new)
{
	new->drv.bus   = &vbus_proxy;
	new->drv.name  = new->type;
	new->drv.owner = new->owner;
	new->drv.probe = vbus_dev_proxy_probe;

	return driver_register(&new->drv);
}
EXPORT_SYMBOL_GPL(vbus_driver_register);

void vbus_driver_unregister(struct vbus_driver *drv)
{
	driver_unregister(&drv->drv);
}
EXPORT_SYMBOL_GPL(vbus_driver_unregister);

/*
 *---------------------------------
 * driver-side IOQ helper
 *---------------------------------
 */
static void
vbus_driver_ioq_release(struct ioq *ioq)
{
	kfree(ioq->head_desc);
	kfree(ioq);
}

static struct ioq_ops vbus_driver_ioq_ops = {
	.release = vbus_driver_ioq_release,
};


int vbus_driver_ioq_alloc(struct vbus_device_proxy *dev, const char *name,
			  int id, int prio, size_t count, struct ioq **ioq)
{
	struct ioq           *_ioq;
	struct ioq_ring_head *head = NULL;
	struct shm_signal    *signal = NULL;
	size_t                len = IOQ_HEAD_DESC_SIZE(count);
	int                   ret = -ENOMEM;

	_ioq = kzalloc(sizeof(*_ioq), GFP_KERNEL);
	if (!_ioq)
		goto error;

	head = kzalloc(len, GFP_KERNEL | GFP_DMA);
	if (!head)
		goto error;

	head->magic     = IOQ_RING_MAGIC;
	head->ver	= IOQ_RING_VER;
	head->count     = cpu_to_le32(count);

	ret = dev->ops->shm(dev, name, id, prio, head, len,
			    &head->signal, &signal, 0);
	if (ret < 0)
		goto error;

	ioq_init(_ioq,
		 &vbus_driver_ioq_ops,
		 ioq_locality_north,
		 head,
		 signal,
		 count);

	*ioq = _ioq;

	return 0;

 error:
	kfree(_ioq);
	kfree(head);

	if (signal)
		shm_signal_put(signal);

	return ret;
}
EXPORT_SYMBOL_GPL(vbus_driver_ioq_alloc);
