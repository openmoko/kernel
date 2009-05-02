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


#define CQ_LEN (GLAMO_CMDQ_SIZE)
#define CQ_MASK ((CQ_LEN + 1) * 1024 - 1)
#define CQ_MASKL (CQ_MASK & 0xffff)
#define CQ_MASKH (CQ_MASK >> 16)


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

	printk(KERN_INFO "[glamo-drm] Waiting for engine idle...\n");
	for ( i=0; i<1000; i++ ) {
		status = reg_read(gdrm, GLAMO_REG_CMDQ_STATUS);
		if ((status & mask) == val) break;
		mdelay(1);
	}
	if ( i == 1000 ) {
		size_t ring_read;
		printk(KERN_WARNING "[glamo-drm] CmdQ timeout!\n");
		printk(KERN_WARNING "[glamo-drm] status = %x\n", status);
		ring_read = reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRL)
				& CQ_MASKL;
		ring_read |= ((reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRH)
				& CQ_MASKH) << 16);
		printk(KERN_INFO "[glamo-drm] ring_read now 0x%x\n",
				 ring_read);
	}
}


/* This is DRM_IOCTL_GLAMO_CMDBUF */
int glamo_ioctl_cmdbuf(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	size_t ring_write, ring_read;
	size_t new_ring_write;
	struct glamodrm_handle *gdrm;
	size_t count;
	drm_glamo_cmd_buffer_t *cbuf = data;
	u16 *addr;

	printk(KERN_INFO "[glamo-drm] glamo_ioctl_cmdbuf: %i bytes\n",
			 cbuf->bufsz);
	gdrm = dev->dev_private;

	count = cbuf->bufsz;
	addr = (u16 *)cbuf->buf;

	/* TODO: Sanitise buffer before doing anything else */

	ring_write = reg_read(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRL);
	ring_write |= (reg_read(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRH) << 16);
	printk(KERN_INFO "[glamo-drm] Old write pointer = 0x%x\n",
			 ring_write);

	/* Calculate where we'll end up */
	new_ring_write = (ring_write + count) % GLAMO_CMDQ_SIZE;
	printk(KERN_INFO "[glamo-drm] New write pointer = 0x%x\n",
			 new_ring_write);

	/* Wait until there is enough space to queue the cmd buffer */
	if (new_ring_write > ring_write) {
		do {
			ring_read = reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRL)
					& CQ_MASKL;
			ring_read |= ((reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRH)
					& CQ_MASKH) << 16);
			printk(KERN_INFO "[glamo-drm] ring_read now 0x%x\n",
					 ring_read);
		/* Loop while the read pointer is between the old and new
		 * positions */
		} while (ring_read > ring_write && ring_read < new_ring_write);
    	} else {
    		do {
	        	ring_read = reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRL)
					& CQ_MASKL;
			ring_read |= ((reg_read(gdrm, GLAMO_REG_CMDQ_READ_ADDRH)
					& CQ_MASKH) << 16);
			printk(KERN_INFO "[glamo-drm] ring_read now 0x%x\n",
					 ring_read);
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

		/* Before changing write position, read has to stop */
		glamo_cmdq_wait(gdrm, GLAMO_ENGINE_CMDQ);

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
			printk(KERN_INFO "[glamo-drm] CmdQ write: 0x%x : %4x\n",
				ring_write+(i*2), *(addr+i));
			iowrite16(*(addr+i), gdrm->cmdq_base+ring_write+(i*2));
		}
		glamo_cmdq_wait(gdrm, GLAMO_ENGINE_CMDQ);

	}

	/* Finally, update the write pointer */
	glamo_engine_clkreg_set(gdrm->glamo_core, GLAMO_ENGINE_2D,
				GLAMO_CLOCK_2D_EN_M6CLK, 0x0000);
	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRH,
					(new_ring_write >> 16) & 0x7f);
	reg_write(gdrm, GLAMO_REG_CMDQ_WRITE_ADDRL,
					new_ring_write & 0xffff);
	glamo_engine_clkreg_set(gdrm->glamo_core, GLAMO_ENGINE_2D,
				GLAMO_CLOCK_2D_EN_M6CLK, 0xffff);

	glamo_cmdq_wait(gdrm, GLAMO_ENGINE_ALL);

	return 0;
}


int glamo_cmdq_init(struct glamodrm_handle *gdrm)
{
	unsigned int i;

	/* Start by zeroing the command queue memory */
	for ( i=0; i<GLAMO_CMDQ_SIZE; i+=2 ) {
		iowrite16(0x0000, gdrm->cmdq_base+i);
	}

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
