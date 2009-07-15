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


struct drm_gem_object *glamo_gem_object_alloc(struct drm_device *dev, int size,
                                              int alignment)
{
	struct drm_gem_object *obj;
	struct glamodrm_handle *gdrm;
	struct drm_glamo_gem_object *gobj;

	gdrm = dev->dev_private;

	size = roundup(size, PAGE_SIZE);

	obj = drm_gem_object_alloc(dev, size);
	if (obj == NULL) return NULL;

	/* See glamodrm_gem_init_object() below */
	gobj = obj->driver_private;

	/* Allocate memory for this object in VRAM */
	gobj->block = drm_mm_search_free(gdrm->mmgr, size, alignment, 1);
	if (!gobj->block) {
		goto fail;
	}
	gobj->block = drm_mm_get_block(gobj->block, size, alignment);
	if (!gobj->block) {
		goto fail;
	}

	return obj;

fail:
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);
	printk(KERN_INFO "[glamo-drm] Failed to allocate object\n");

	return NULL;
}


int glamo_ioctl_gem_create(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	struct drm_glamo_gem_create *args = data;
	struct drm_gem_object *obj;
	struct drm_glamo_gem_object *gobj;
	int handle, ret;

	/* Create an object */
	obj = glamo_gem_object_alloc(dev, args->size, args->alignment);
	if ( obj == NULL ) return -ENOMEM;

	/* Create a handle for it */
	ret = drm_gem_handle_create(file_priv, obj, &handle);
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_handle_unreference(obj);
	mutex_unlock(&dev->struct_mutex);
	if (ret) goto fail;
	
	/* Watchpoint */
	gobj = obj->driver_private;
	printk(KERN_INFO "[glamo-drm] GEM object %i: %li bytes at 0x%lx\n",
			  handle, gobj->block->size, gobj->block->start);
	
	/* Return */
	args->handle = handle;
	return 0;

fail:
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);
	printk(KERN_INFO "[glamo-drm] Failed to allocate object\n");
	return ret;
}


int glamo_ioctl_gem_mmap(struct drm_device *dev, void *data,
			 struct drm_file *file_priv)
{
	struct drm_glamo_gem_mmap *args = data;
	struct drm_gem_object *obj;
	loff_t offset;
	unsigned long addr;

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (obj == NULL)
		return -EBADF;

	offset = args->offset;

	down_write(&current->mm->mmap_sem);
	addr = do_mmap(obj->filp, 0, args->size,
		       PROT_READ | PROT_WRITE, MAP_SHARED,
		       args->offset);
	up_write(&current->mm->mmap_sem);
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);
	if (IS_ERR((void *)addr))
		return addr;

	args->addr_ptr = (uint64_t) addr;

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


int glamodrm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	return VM_FAULT_SIGBUS;
}


int glamodrm_gem_init_object(struct drm_gem_object *obj)
{
	struct drm_glamo_gem_object *gobj;

	/* Allocate a private structure */
	gobj = drm_calloc(1, sizeof(*gobj), DRM_MEM_DRIVER);
	if (!gobj) return -ENOMEM;

	obj->driver_private = gobj;
	gobj->obj = obj;

	return 0;
}


void glamodrm_gem_free_object(struct drm_gem_object *obj)
{
	struct drm_glamo_gem_object *gobj;

	gobj = obj->driver_private;

	/* Free the VRAM */
	drm_mm_put_block(gobj->block);

	/* Free the private structure */
	drm_free(obj->driver_private, 1, DRM_MEM_DRIVER);
}


/* Memory management initialisation */
int glamo_buffer_init(struct glamodrm_handle *gdrm)
{
	gdrm->mmgr = drm_calloc(1, sizeof(struct drm_mm), DRM_MEM_DRIVER);
	drm_mm_init(gdrm->mmgr, 0, gdrm->vram_size);
	return 0;
}


/* Memory management finalisation */
int glamo_buffer_final(struct glamodrm_handle *gdrm)
{
	drm_mm_takedown(gdrm->mmgr);
	drm_free(gdrm->mmgr , 1, DRM_MEM_DRIVER);
	return 0;
}
