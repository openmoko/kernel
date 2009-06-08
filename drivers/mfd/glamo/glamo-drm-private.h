/* Smedia Glamo 336x/337x DRM private bits
 *
 * Copyright (C) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 * Copyright (C) 2009 Andreas Pokorny <andreas.pokorny@gmail.com>
 * Based on xf86-video-glamo
 * Copyright  2007 OpenMoko, Inc.
 * Copyright © 2009 Lars-Peter Clausen <lars@metafoo.de>
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

#ifndef __GLAMO_DRMPRIV_H
#define __GLAMO_DRMPRIV_H


#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>

#include "glamo-core.h"


struct glamodrm_handle {

	/* This device */
	struct device *dev;
	/* The parent device handle */
	struct glamo_core *glamo_core;

	/* Command queue registers */
	struct resource *reg;
	char __iomem *reg_base;

	/* VRAM region */
	struct resource *vram;
	char __iomem *vram_base;

	/* Command queue region */
	struct resource *cmdq;
	char __iomem *cmdq_base;

	ssize_t vram_size;

	/* Memory management */
	struct drm_mm *mmgr;

	/* semaphore against concurrent ioctl */
	struct semaphore add_to_ring;
};

/* Private data.  This is where we keep our memory management bits */
struct drm_glamo_gem_object {
	struct drm_gem_object *obj;	/* The GEM object this refers to */
	struct drm_mm_node *block;	/* Block handle for drm_mm */
};



#endif /* __GLAMO_DRMPRIV_H */
