/*
 * SMedia Glamo 336x/337x command queue handling
 *
 * Copyright (C) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 * Based on xf86-video-glamo (see below for details)
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
 *
 * Command queue handling functions based on those from xf86-video-glamo, to
 * which the following licence applies:
 *
 * Copyright  2007 OpenMoko, Inc.
 * Copyright © 2009 Lars-Peter Clausen <lars@metafoo.de>
 *
 * This driver is based on Xati,
 * Copyright  2004 Eric Anholt
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */


#include <drm/drmP.h>
#include <drm/glamo_drm.h>

#include "glamo-core.h"
#include "glamo-drm-private.h"
#include "glamo-regs.h"


static void reg_write(struct glamodrm_handle *gdrm,
		      u_int16_t reg, u_int16_t val)
{
	iowrite16(val, gdrm->reg_base + reg);
}


static u16 reg_read(struct glamodrm_handle *gdrm, u_int16_t reg)
{
	return ioread16(gdrm->reg_base + reg);
}


static void
glamo_cmdq_wait(struct glamodrm_handle *gdrm, enum glamo_engine engine)
{
	u16 mask, val, status;
	int i;

	switch (engine)
	{
		case GLAMO_ENGINE_CMDQ:
			mask = 0x3;
			val  = mask;
			break;
		case GLAMO_ENGINE_ISP:
			mask = 0x3 | (1 << 8);
			val  = 0x3;
			break;
		case GLAMO_ENGINE_2D:
			mask = 0x3 | (1 << 4);
			val  = 0x3;
			break;
		case GLAMO_ENGINE_3D:
			mask = 0x3 | (1 << 5);
			val  = 0x3;
			break;
		case GLAMO_ENGINE_ALL:
		default:
			mask = 1 << 2;
			val  = mask;
			break;
	}

	for ( i=0; i<1000; i++ ) {
		status = reg_read(gdrm, GLAMO_REG_CMDQ_STATUS);
		if ((status & mask) == val) break;
		mdelay(1);
	}
	if ( i == 1000 ) {
		size_t ring_read;
		printk(KERN_WARNING "[glamo-drm] CmdQ timeout!\n");
		printk(KERN_WARNING "[glamo-drm] status = %x\n", status);
		ring_read = reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRL);
		ring_read |= ((reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRH)
				& 0x7) << 16);
		printk(KERN_INFO "[glamo-drm] ring_read now 0x%x\n",
				 ring_read);
	}
}


/* Add commands to the ring buffer */
static int glamo_add_to_ring(struct glamodrm_handle *gdrm, u16 *addr,
			     unsigned int count)
{
	size_t ring_write, ring_read;
	size_t new_ring_write;

	ring_write = reg_read(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRL);
	ring_write |= (reg_read(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRH) << 16);

	/* Calculate where we'll end up */
	new_ring_write = (ring_write + count) % GLAMO_CMDQ_SIZE;

	/* Wait until there is enough space to queue the cmd buffer */
	if (new_ring_write > ring_write) {
		/* Loop while the read pointer is between the old and new
		 * positions */
		do {
			ring_read = reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRL);
			ring_read |= ((reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRH)
					& 0x7) << 16);
		} while (ring_read > ring_write && ring_read < new_ring_write);
    	} else {
		/* Same, but kind of inside-out */
		do {
	        	ring_read = reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRL);
			ring_read |= ((reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRH)
					& 0x7) << 16);
		} while (ring_read > ring_write || ring_read < new_ring_write);
	}

	/* Are we about to wrap around? */
	if (ring_write >= new_ring_write) {

		size_t rest_size;
		int i;
		printk(KERN_INFO "[glamo-drm] CmdQ wrap-around...\n");
		/* Wrap around */
		rest_size = GLAMO_CMDQ_SIZE - ring_write; /* Space left */

		/* Write from current position to end */
		for ( i=0; i<rest_size; i++ ) {
			iowrite16(*(addr+i), gdrm->cmdq_base+ring_write+(i*2));
		}

		/* Write from start */
		for ( i=0; i<(count-rest_size); i++ ) {
			iowrite16(*(addr+rest_size+i), gdrm->cmdq_base+(i*2));
		}

		/* ring_write being 0 will result in a deadlock because the
		 * cmdq read will never stop. To avoid such an behaviour insert
		 * an empty instruction. */
		if (new_ring_write == 0) {
			iowrite16(0x0000, gdrm->cmdq_base);
			iowrite16(0x0000, gdrm->cmdq_base + 2);
			new_ring_write = 4;
		}

		/* Suppose we just filled the WHOLE ring buffer, and so the
		 * write position ends up in the same place as it started.
		 * No change in pointer means no activity from the command
		 * queue engine.  So, insert a no-op */
		if (ring_write == new_ring_write) {
			iowrite16(0x0000, gdrm->cmdq_base + new_ring_write);
			iowrite16(0x0000, gdrm->cmdq_base + new_ring_write + 2);
			new_ring_write += 4;
		}

	} else {

		int i;
		/* The easy case */
		for ( i=0; i<count/2; i++ ) { /* Number of words */
			iowrite16(*(addr+i), gdrm->cmdq_base+ring_write+(i*2));
		}

	}

	/* Before changing write position, read has to stop
	 * (Brought across from xf86-video-glamo)
	 * TAW: Really?  Is pausing the clock not enough? */
	glamo_cmdq_wait(gdrm, GLAMO_ENGINE_CMDQ);
	/* Note that CLOCK_2D_EN_M6CLK has nothing to do with the 2D engine */
	glamo_engine_clkreg_set(gdrm->glamo_core, GLAMO_ENGINE_2D,
				GLAMO_CLOCK_2D_EN_M6CLK, 0x0000);
	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRH,
					(new_ring_write >> 16) & 0x7f);
	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRL,
					new_ring_write & 0xffff);
	glamo_engine_clkreg_set(gdrm->glamo_core, GLAMO_ENGINE_2D,
				GLAMO_CLOCK_2D_EN_M6CLK, 0xffff);

	return 0;
}


/* Return true for a legal sequence of commands, otherwise false */
static int glamo_sanitize_buffer(u16 *cmds, unsigned int count)
{
	/* XXX FIXME TODO: Implementation... */
	return 1;
}


/* Substitute the real addresses in VRAM for any required buffer objects */
static int glamo_do_relocation(struct glamodrm_handle *gdrm,
			       drm_glamo_cmd_buffer_t *cbuf, u16 *cmds,
			       struct drm_device *dev,
			       struct drm_file *file_priv)
{
	uint32_t *handles;
	int *offsets;
	int nobjs =  cbuf->nobjs;
	int i;

	if ( nobjs > 32 ) return -EINVAL;	/* Get real... */

	handles = drm_alloc(nobjs*sizeof(uint32_t), DRM_MEM_DRIVER);
	if ( handles == NULL ) return -1;
	if ( copy_from_user(handles, cbuf->objs, nobjs*sizeof(uint32_t)) )
		return -1;

	offsets = drm_alloc(nobjs*sizeof(int), DRM_MEM_DRIVER);
	if ( offsets == NULL ) return -1;
	if ( copy_from_user(offsets, cbuf->obj_pos, nobjs*sizeof(int)) )
		return -1;

	for ( i=0; i<nobjs; i++ ) {

		uint32_t handle = handles[i];
		int offset = offsets[i];
		struct drm_gem_object *obj;
		struct drm_glamo_gem_object *gobj;
		u32 addr;
		u16 addr_low, addr_high;

		printk(KERN_INFO "[glamo-drm] Relocating object handle %i "
				 "at position 0x%x\n", handle, offset);

		if ( offset > cbuf->bufsz ) {
			printk(KERN_WARNING "[glamo-drm] Offset out of range "
					    "for this relocation!\n");
			goto fail;
		}

		obj = drm_gem_object_lookup(dev, file_priv, handle);
		if ( obj == NULL ) return -1;

		/* Unref the object now, or it'll never get freed.
		 * This should really happen after the GPU has finished
		 * the commands which are about to be submitted. */
		drm_gem_object_unreference(obj);

		gobj = obj->driver_private;
		if ( gobj == NULL ) {
			printk(KERN_WARNING "[glamo-drm] This object has no "
					    "private data!\n");
			goto fail;
		}

		addr = GLAMO_OFFSET_FB + gobj->block->start;
		addr_low = addr & 0xffff;
		addr_high = (addr >> 16) & 0x7f;
		printk(KERN_INFO "Addr low 0x%x, high 0x%x\n",
				  addr_low, addr_high);

		/* FIXME: Should really check that the register is a
		 * valid one for this relocation. */

		*(cmds+(offset/2)+1) = addr_low;
		*(cmds+(offset/2)+3) = addr_high;

	}

	drm_free(handles, 1, DRM_MEM_DRIVER);
	drm_free(offsets, 1, DRM_MEM_DRIVER);
	return 0;

fail:
	drm_free(handles, 1, DRM_MEM_DRIVER);
	drm_free(offsets, 1, DRM_MEM_DRIVER);
	return -1;
}


/* This is DRM_IOCTL_GLAMO_CMDBUF */
int glamo_ioctl_cmdbuf(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	struct glamodrm_handle *gdrm;
	unsigned int count;
	drm_glamo_cmd_buffer_t *cbuf = data;
	u16 *cmds;

	gdrm = dev->dev_private;

	count = cbuf->bufsz;

	if ( count > PAGE_SIZE ) return -EINVAL;

	cmds = drm_alloc(count, DRM_MEM_DRIVER);
	if ( cmds == NULL ) return -ENOMEM;
	if ( copy_from_user(cmds, cbuf->buf, count) ) return -EINVAL;

	/* Check the buffer isn't going to tell Glamo to enact naughtiness */
	if ( !glamo_sanitize_buffer(cmds, count) ) return -EINVAL;

	/* Perform relocation, if necessary */
	if ( cbuf->nobjs ) {
		if ( glamo_do_relocation(gdrm, cbuf, cmds, dev, file_priv) )
			return -EINVAL;	/* Relocation failed */
	}

	glamo_add_to_ring(gdrm, cmds, count);

	/* Having the CPU wait for the CPU is, to put it mildly,
	 * less than optimal.
	 * TODO: Avoid doing this, by some suitable means */
	glamo_cmdq_wait(gdrm, GLAMO_ENGINE_ALL);

	drm_free(cmds, 1, DRM_MEM_DRIVER);

	return 0;
}


int glamo_cmdq_init(struct glamodrm_handle *gdrm)
{
	unsigned int i;

	/* Enable 2D and 3D */
	glamo_engine_enable(gdrm->glamo_core, GLAMO_ENGINE_2D);
	glamo_engine_reset(gdrm->glamo_core, GLAMO_ENGINE_2D);

	/* Start by zeroing the command queue memory */
	for ( i=0; i<GLAMO_CMDQ_SIZE; i+=2 ) {
		iowrite16(0x0000, gdrm->cmdq_base+i);
	}

	glamo_engine_enable(gdrm->glamo_core, GLAMO_ENGINE_CMDQ);
	glamo_engine_reset(gdrm->glamo_core, GLAMO_ENGINE_CMDQ);

	/* Set up command queue location */
	reg_write(gdrm, GLAMO_REG_CMDQ_BASE_ADDRL,
					GLAMO_OFFSET_CMDQ & 0xffff);
	reg_write(gdrm, GLAMO_REG_CMDQ_BASE_ADDRH,
					(GLAMO_OFFSET_CMDQ >> 16) & 0x7f);

	/* Length of command queue in 1k blocks, minus one */
	reg_write(gdrm, GLAMO_REG_CMDQ_LEN, (GLAMO_CMDQ_SIZE >> 10)-1);
	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRH, 0);
	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRL, 0);
	reg_write(gdrm, GLAMO_REG_CMDQ_READ_ADDRH, 0);
	reg_write(gdrm, GLAMO_REG_CMDQ_READ_ADDRL, 0);
	reg_write(gdrm, GLAMO_REG_CMDQ_CONTROL,
					 1 << 12 |	/* Turbo flip (?) */
					 5 << 8 |	/* No interrupt */
					 8 << 4);	/* HQ threshold */

	/* Wait for things to settle down */
	glamo_cmdq_wait(gdrm, GLAMO_ENGINE_ALL);

	return 0;
}


void glamo_cmdq_suspend(struct glamodrm_handle *gdrm)
{
	/* Placeholder... */
}


void glamo_cmdq_resume(struct glamodrm_handle *gdrm)
{
	glamo_cmdq_init(gdrm);
}
