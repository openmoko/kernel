/* Smedia Glamo 336x/337x driver
 *
 * Copyright (C) 2009 Openmoko, Inc. Jorge Luis Zapata <turran@openmoko.com>
 * Copyright (C) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#include <linux/module.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>

#define DRIVER_AUTHOR           "Openmoko, Inc."
#define DRIVER_NAME             "glamo-drm"
#define DRIVER_DESC             "SMedia Glamo 3362"
#define DRIVER_DATE             "20090217"

int glamodrm_firstopen(struct drm_device *dev)
{
	DRM_DEBUG("\n");
	return 0;
}

int glamodrm_open(struct drm_device *dev, struct drm_file *fh)
{
	DRM_DEBUG("\n");
	return 0;
}

void glamodrm_preclose(struct drm_device *dev, struct drm_file *fh)
{
	DRM_DEBUG("\n");
}

void glamodrm_postclose(struct drm_device *dev, struct drm_file *fh)
{
	DRM_DEBUG("\n");
}

void glamodrm_lastclose(struct drm_device *dev)
{
	DRM_DEBUG("\n");
}

static int glamodrm_master_create(struct drm_device *dev, struct drm_master *master)
{
	DRM_DEBUG("\n");

        return 0;
}

static void glamodrm_master_destroy(struct drm_device *dev, struct drm_master *master)
{
	DRM_DEBUG("\n");
}

static struct drm_driver glamodrm_drm_driver = {
	.driver_features = DRIVER_IS_PLATFORM,
	.firstopen = glamodrm_firstopen,
	.open = glamodrm_open,
	.preclose = glamodrm_preclose,
	.postclose = glamodrm_postclose,
	.lastclose = glamodrm_lastclose,
	.reclaim_buffers = drm_core_reclaim_buffers,
	.get_map_ofs = drm_core_get_map_ofs,
	.get_reg_ofs = drm_core_get_reg_ofs,
	.master_create = glamodrm_master_create,
	.master_destroy = glamodrm_master_destroy,
	/* TODO GEM interface
	.gem_init_object = glamodrm_gem_init_object,
	.gem_free_object = glamodrm_gem_free_object,
	.gem_vm_ops = &glamodrm_gem_vm_ops,
	*/
	.fops = {
		.owner = THIS_MODULE,
		.open = drm_open,
		.release = drm_release,
		.ioctl = drm_ioctl,
		.mmap = drm_mmap,
		.poll = drm_poll,
		.fasync = drm_fasync,
	},
	.major = 0,
	.minor = 1,
	.patchlevel = 0,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
};


static int glamodrm_probe(struct platform_device *pdev)
{
	struct resource *r;

	printk(KERN_INFO "SMedia Glamo DRM driver\n");
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					  "glamo-work-mem");

	printk("MEM = %08x\n", r->start);
	drm_platform_init(&glamodrm_drm_driver, pdev);
	return 0;
}


static int glamodrm_remove(struct platform_device *pdev)
{
	drm_exit(&glamodrm_drm_driver);
	return 0;
}

static struct platform_driver glamodrm_driver = {
	.probe          = glamodrm_probe,
	.remove         = glamodrm_remove,
	.driver         = {
		.name   = "glamo-cmdq",
		.owner  = THIS_MODULE,
	},
};

static int __devinit glamodrm_init(void)
{
	return platform_driver_register(&glamodrm_driver);
}

static void __exit glamodrm_exit(void)
{
	platform_driver_unregister(&glamodrm_driver);
}

module_init(glamodrm_init);
module_exit(glamodrm_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

