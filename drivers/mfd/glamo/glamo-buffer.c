/*
 * SMedia Glamo 336x/337x memory management
 *
 * Copyright (c) 2009 Thomas White <taw@bitwiz.org.uk>
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
 *
 */


#include <drm/drmP.h>
#include <drm/glamo_drm.h>

#include "glamo-drm-private.h"


int glamo_ioctl_gem_create(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_create\n");
	return 0;
}


int glamo_ioctl_gem_mmap(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_mmap\n");
	return 0;
}


int glamo_ioctl_gem_pin(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_pin\n");
	return 0;
}


int glamo_ioctl_gem_unpin(struct drm_device *dev, void *data,
				 struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_unpin\n");
	return 0;
}


int glamo_ioctl_gem_pread(struct drm_device *dev, void *data,
				 struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_pread\n");
	return 0;
}


int glamo_ioctl_gem_pwrite(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	printk(KERN_INFO "glamo_ioctl_gem_pwrite\n");
	return 0;
}
