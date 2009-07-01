/*
 * SMedia Glamo 336x/337x display
 *
 * Copyright (C) 2008-2009 Thomas White <taw@bitwiz.org.uk>
 *
 * Based on glamo-fb.c (C) 2007-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
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
 *
 * Based on intel_display.c and intel_crt.c from drivers/gpu/drm/i915
 *  to which the following licence applies:
 *
 * Copyright © 2006-2007 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *
 */

#define DEBUG 1

#include <drm/drmP.h>
#include <drm/glamo_drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_crtc.h>

#include "glamo-core.h"
#include "glamo-drm-private.h"
#include "glamo-regs.h"
#include "glamo-kms-fb.h"
#include <linux/glamofb.h>


#define GLAMO_LCD_WIDTH_MASK 0x03FF
#define GLAMO_LCD_HEIGHT_MASK 0x03FF
#define GLAMO_LCD_PITCH_MASK 0x07FE
#define GLAMO_LCD_HV_TOTAL_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_START_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_END_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_DISP_START_MASK 0x03FF
#define GLAMO_LCD_HV_RETR_DISP_END_MASK 0x03FF


struct glamofb_par {
	struct drm_device *dev;
	struct drm_display_mode *our_mode;
	struct glamo_framebuffer *glamo_fb;
	int crtc_count;
	/* crtc currently bound to this */
	uint32_t crtc_ids[2];
};


static int reg_read_lcd(struct glamodrm_handle *gdrm, u_int16_t reg)
{
	int i = 0;

	for (i = 0; i != 2; i++)
		nop();

	return ioread16(gdrm->lcd_base + reg);
}


static void reg_write_lcd(struct glamodrm_handle *gdrm,
                          u_int16_t reg, u_int16_t val)
{
	int i = 0;

	for (i = 0; i != 2; i++)
		nop();
	
	iowrite16(val, gdrm->lcd_base + reg);
}


static void reg_set_bit_mask_lcd(struct glamodrm_handle *gdrm,
                                 u_int16_t reg, u_int16_t mask,
                                 u_int16_t val)
{
	u_int16_t tmp;

	val &= mask;

	tmp = reg_read_lcd(gdrm, reg);
	tmp &= ~mask;
	tmp |= val;
	reg_write_lcd(gdrm, reg, tmp);
}


/* Note that this has nothing at all to do with the engine command queue
 * in glamo-cmdq.c */
static inline int glamo_lcd_cmdq_empty(struct glamodrm_handle *gdrm)
{
	/* DGCMdQempty -- 1 == command queue is empty */
	return reg_read_lcd(gdrm, GLAMO_REG_LCD_STATUS1) & (1 << 15);
}


/* call holding gfb->lock_cmd  when locking, until you unlock */
int glamo_lcd_cmd_mode(struct glamodrm_handle *gdrm, int on)
{
	int timeout = 2000000;

	if (gdrm->glamo_core->suspending) {
		dev_err(&gdrm->glamo_core->pdev->dev,
		                "IGNORING glamofb_cmd_mode while"
		                " suspended\n");
		return -EBUSY;
	}

	dev_dbg(gdrm->dev, "glamofb_cmd_mode(on=%d)\n", on);
	if (on) {

		while ((!glamo_lcd_cmdq_empty(gdrm)) && (timeout--))
			/* yield() */;
		if (timeout < 0) {
			printk(KERN_ERR "*************"
			                " LCD command queue never got empty "
			                "*************\n");
			return -EIO;
		}

		/* display the entire frame then switch to command */
		reg_write_lcd(gdrm, GLAMO_REG_LCD_COMMAND1,
			  GLAMO_LCD_CMD_TYPE_DISP |
			  GLAMO_LCD_CMD_DATA_FIRE_VSYNC);

		/* wait until lcd idle */
		timeout = 2000000;
		while ((!reg_read_lcd(gdrm, GLAMO_REG_LCD_STATUS2) & (1 << 12))
		          && (timeout--))
			/* yield() */;
		if (timeout < 0) {
			printk(KERN_ERR"*************"
				       " LCD never idle "
				       "*************\n");
			return -EIO;
		}

		mdelay(100);

	} else {
		/* RGB interface needs vsync/hsync */
		int mode;
		mode = reg_read_lcd(gdrm, GLAMO_REG_LCD_MODE3);
		if ( mode & GLAMO_LCD_MODE3_RGB)
			reg_write_lcd(gdrm, GLAMO_REG_LCD_COMMAND1,
				  GLAMO_LCD_CMD_TYPE_DISP |
				  GLAMO_LCD_CMD_DATA_DISP_SYNC);

		reg_write_lcd(gdrm, GLAMO_REG_LCD_COMMAND1,
			  GLAMO_LCD_CMD_TYPE_DISP |
			  GLAMO_LCD_CMD_DATA_DISP_FIRE);
	}

	return 0;
}


static struct glamo_script lcd_init_script[] = {
	{ GLAMO_REG_LCD_MODE1, 0x0020 },
	/* no display rotation, no hardware cursor, no dither, no gamma,
	 * no retrace flip, vsync low-active, hsync low active,
	 * no TVCLK, no partial display, hw dest color from fb,
	 * no partial display mode, LCD1, software flip,  */
	{ GLAMO_REG_LCD_MODE2, 0x9020 },
	  /* video flip, no ptr, no ptr, dhclk off,
	   * normal mode,  no cpuif,
	   * res, serial msb first, single fb, no fr ctrl,
	   * cpu if bits all zero, no crc
	   * 0000 0000 0010  0000 */
	{ GLAMO_REG_LCD_MODE3, 0x0b40 },
	  /* src data rgb565, res, 18bit rgb666
	   * 000 01 011 0100 0000 */
	{ GLAMO_REG_LCD_POLARITY, 0x440c },
	  /* DE high active, no cpu/lcd if, cs0 force low, a0 low active,
	   * np cpu if, 9bit serial data, sclk rising edge latch data
	   * 01 00 0 100 0 000 01 0 0 */
	/* The following values assume 640*480@16bpp */
	{ GLAMO_REG_LCD_A_BASE1, 0x0000 }, /* display A base address 15:0 */
	{ GLAMO_REG_LCD_A_BASE2, 0x0000 }, /* display A base address 22:16 */
	{ GLAMO_REG_LCD_B_BASE1, 0x6000 }, /* display B base address 15:0 */
	{ GLAMO_REG_LCD_B_BASE2, 0x0009 }, /* display B base address 22:16 */
	{ GLAMO_REG_LCD_CURSOR_BASE1, 0xC000 }, /* cursor base address 15:0 */
	{ GLAMO_REG_LCD_CURSOR_BASE2, 0x0012 }, /* cursor base address 22:16 */
	{ GLAMO_REG_LCD_COMMAND2, 0x0000 }, /* display page A */
};


static int glamo_run_lcd_script(struct glamodrm_handle *gdrm,
                                struct glamo_script *script, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		struct glamo_script *line = &script[i];

		if (line->reg == 0xffff)
			return 0;
		else if (line->reg == 0xfffe)
			msleep(line->val);
		else
			reg_write_lcd(gdrm, script[i].reg, script[i].val);
	}

	return 0;
}


#if 0
static void notify_blank(struct drm_crtc *crtc, int mode)
{
	struct fb_event event;

	event.info = info;
	event.data = &blank_mode;
	fb_notifier_call_chain(FB_EVENT_CONBLANK, &event);
}
#endif


extern void jbt6k74_action(int val);


/* Power on/off */
static void glamo_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct glamodrm_handle *gdrm;
	struct glamo_crtc *glamo_crtc = to_glamo_crtc(crtc);

	printk(KERN_CRIT "glamo_crtc_dpms(%u)\n", mode);
	gdrm = glamo_crtc->gdrm;

	switch (mode) {
	case DRM_MODE_DPMS_OFF:
		/* LCM need notification before pixel clock is stopped */
		//jbt6k74_action(0);

		/* disable the pixel clock */
//		glamo_engine_clkreg_set(gdrm->glamo_core, GLAMO_ENGINE_LCD,
//					GLAMO_CLOCK_LCD_EN_DCLK, 0);
		glamo_crtc->blank_mode = mode;
		break;
	case DRM_MODE_DPMS_ON:
		/* enable the pixel clock if off */
		if (glamo_crtc->blank_mode == DRM_MODE_DPMS_OFF)
			glamo_engine_clkreg_set(gdrm->glamo_core,
			                        GLAMO_ENGINE_LCD,
			                        GLAMO_CLOCK_LCD_EN_DCLK,
			                        GLAMO_CLOCK_LCD_EN_DCLK);

		//jbt6k74_action(1);
		glamo_crtc->blank_mode = mode;
		break;
	}
}


static bool glamo_crtc_mode_fixup(struct drm_crtc *crtc,
                                  struct drm_display_mode *mode,
                                  struct drm_display_mode *adjusted_mode)
{
	printk(KERN_CRIT "glamo_crtc_mode_fixup\n");
	return true;
}


static void glamo_crtc_mode_set(struct drm_crtc *crtc,
                                struct drm_display_mode *mode,
                                struct drm_display_mode *adjusted_mode,
                                int x, int y,
                                struct drm_framebuffer *old_fb)
{
	struct glamodrm_handle *gdrm;
	struct glamo_crtc *gcrtc;
	int retr_start, retr_end, disp_start, disp_end;

	printk(KERN_CRIT "glamo_crtc_mode_set\n");

	/* Dig out our handle */
	gcrtc = to_glamo_crtc(crtc);
	gdrm = gcrtc->gdrm;	/* Here it is! */

	glamo_lcd_cmd_mode(gdrm, 1);
	glamo_engine_reclock(gdrm->glamo_core, GLAMO_ENGINE_LCD, mode->clock);

	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_WIDTH,
	                     GLAMO_LCD_WIDTH_MASK, mode->hdisplay);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HEIGHT,
	                     GLAMO_LCD_HEIGHT_MASK, mode->vdisplay);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_PITCH,
	                     GLAMO_LCD_PITCH_MASK, mode->hdisplay*2);

	/* Convert "X modeline timings" into "Glamo timings" */
	retr_start = 0;
	retr_end = retr_start + mode->hsync_end - mode->hsync_start;
	disp_start = mode->htotal - mode->hsync_start;
	disp_end = disp_start + mode->hdisplay;
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_TOTAL,
	                     GLAMO_LCD_HV_TOTAL_MASK, mode->htotal);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_RETR_START,
	                     GLAMO_LCD_HV_RETR_START_MASK, retr_start);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_RETR_END,
	                     GLAMO_LCD_HV_RETR_END_MASK, retr_end);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_DISP_START,
	                     GLAMO_LCD_HV_RETR_DISP_START_MASK, disp_start);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_HORIZ_DISP_END,
	                     GLAMO_LCD_HV_RETR_DISP_END_MASK, disp_end);

	/* The same in the vertical direction */
	retr_start = 0;
	retr_end = retr_start + mode->vsync_end - mode->vsync_start;
	disp_start = mode->vtotal - mode->vsync_start;
	disp_end = disp_start + mode->vdisplay;
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_TOTAL,
	                     GLAMO_LCD_HV_TOTAL_MASK, mode->vtotal);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_RETR_START,
	                     GLAMO_LCD_HV_RETR_START_MASK, retr_start);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_RETR_END,
	                     GLAMO_LCD_HV_RETR_END_MASK, retr_end);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_DISP_START,
	                     GLAMO_LCD_HV_RETR_DISP_START_MASK, disp_start);
	reg_set_bit_mask_lcd(gdrm, GLAMO_REG_LCD_VERT_DISP_END,
	                     GLAMO_LCD_HV_RETR_DISP_END_MASK, disp_end);

	glamo_lcd_cmd_mode(gdrm, 0);
}


static void glamo_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
                                     struct drm_framebuffer *old_fb)
{
	printk(KERN_CRIT "glamo_crtc_mode_set_base\n");
}


static void glamo_crtc_prepare(struct drm_crtc *crtc)
{
	struct drm_crtc_helper_funcs *crtc_funcs = crtc->helper_private;
	printk(KERN_CRIT "glamo_crtc_prepare\n");
	crtc_funcs->dpms(crtc, DRM_MODE_DPMS_OFF);
}


static void glamo_crtc_commit(struct drm_crtc *crtc)
{
	struct drm_crtc_helper_funcs *crtc_funcs = crtc->helper_private;
	printk(KERN_CRIT "glamo_crtc_commit\n");
	crtc_funcs->dpms(crtc, DRM_MODE_DPMS_ON);
}


static int glamo_crtc_cursor_set(struct drm_crtc *crtc,
                                 struct drm_file *file_priv,
                                 uint32_t handle,
                                 uint32_t width, uint32_t height)
{
	printk(KERN_CRIT "glamo_crtc_cursor_set\n");
	return 0;
}


static int glamo_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	printk(KERN_CRIT "glamo_crtc_cursor_move\n");
	return 0;
}


static void glamo_crtc_gamma_set(struct drm_crtc *crtc, u16 *red, u16 *green,
                                 u16 *blue, uint32_t size)
{
	printk(KERN_CRIT "glamo_crtc_gamma_set\n");
}


static void glamo_crtc_destroy(struct drm_crtc *crtc)
{
	struct glamo_crtc *glamo_crtc = to_glamo_crtc(crtc);
	printk(KERN_CRIT "glamo_crtc_destroy\n");
	drm_crtc_cleanup(crtc);
	kfree(glamo_crtc);
}


static enum drm_connector_status
glamo_connector_detect(struct drm_connector *connector)
{
	/* One hopes it hasn't been de-soldered... */
	printk(KERN_CRIT "glamo_connector_detect\n");
	return connector_status_connected;
}


static void glamo_connector_destroy(struct drm_connector *connector)
{
	printk(KERN_CRIT "glamo_connector_destroy\n");
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}


static int glamo_connector_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct glamofb_platform_data *mach_info;
	struct glamo_output *goutput = to_glamo_output(connector);
	struct glamodrm_handle *gdrm = goutput->gdrm;

	/* Dig out the record which will tell us about the hardware */
	mach_info = gdrm->glamo_core->pdev->dev.platform_data;

	printk(KERN_CRIT "glamo_connector_get_modes\n");

	mode = drm_mode_create(connector->dev);
	if (!mode)
		return 0;
	/* Fill in 'mode' here */
	mode->type = DRM_MODE_TYPE_DEFAULT | DRM_MODE_TYPE_PREFERRED;

	/* Convert framebuffer timings into KMS timings */
	mode->clock = 1000000 / mach_info->pixclock;	/* ps -> Hz */
	mode->hdisplay = mach_info->xres.defval;
	mode->hsync_start = mach_info->right_margin + mode->hdisplay;
	mode->hsync_end = mode->hsync_start + mach_info->hsync_len;
	mode->htotal = mode->hsync_end + mach_info->left_margin;
	mode->hskew = 0;
	
	mode->vdisplay = mach_info->yres.defval;
	mode->vsync_start = mach_info->lower_margin + mode->vdisplay;
	mode->vsync_end = mode->vsync_start + mach_info->vsync_len;
	mode->vtotal = mode->vsync_end + mach_info->upper_margin;
	mode->vscan = 0;

	mode->width_mm = mach_info->width;
	mode->height_mm = mach_info->height;

	printk(KERN_CRIT "Modeline \"%ix%i\" %i   %i %i %i %i    %i %i %i %i\n",
	       mode->hdisplay, mode->vdisplay, mode->clock,
	       mode->hdisplay, mode->hsync_start, mode->hsync_end, mode->htotal,
	       mode->vdisplay, mode->vsync_start, mode->vsync_end, mode->vtotal);

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return 1;	/* one mode, for now */
}


static int glamo_connector_set_property(struct drm_connector *connector,
				  struct drm_property *property,
				  uint64_t value)
{
	printk(KERN_CRIT "glamo_connector_set_property\n");
	return 0;
}


static int glamo_connector_mode_valid(struct drm_connector *connector,
                                      struct drm_display_mode *mode)
{
	printk(KERN_CRIT "glamo_connector_mode_valid\n");
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	return MODE_OK;
}


struct drm_encoder *
glamo_connector_best_encoder(struct drm_connector *connector)
{
	struct glamo_output *glamo_output = to_glamo_output(connector);
	printk(KERN_CRIT "glamo_connector_best_encoder\n");
	return &glamo_output->enc;
}


static void glamo_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	printk(KERN_CRIT "glamo_encoder_dpms\n");
}


static bool glamo_encoder_mode_fixup(struct drm_encoder *encoder,
                                 struct drm_display_mode *mode,
                                 struct drm_display_mode *adjusted_mode)
{
	printk(KERN_CRIT "glamo_encoder_mode_fixup\n");
	return true;
}


void glamo_encoder_prepare(struct drm_encoder *encoder)
{
	printk(KERN_CRIT "glamo_encoder_prepare\n");
}


void glamo_encoder_commit(struct drm_encoder *encoder)
{
	printk(KERN_CRIT "glamo_encoder_commit\n");
}


static void glamo_encoder_mode_set(struct drm_encoder *encoder,
                               struct drm_display_mode *mode,
                               struct drm_display_mode *adjusted_mode)
{
	printk(KERN_CRIT "glamo_encoder_mode_set\n");
}


static void glamo_encoder_destroy(struct drm_encoder *encoder)
{
	printk(KERN_CRIT "glamo_encoder_destroy\n");
	drm_encoder_cleanup(encoder);
}


static void glamo_framebuffer_destroy(struct drm_framebuffer *fb)
{
	struct glamo_framebuffer *glamo_fb = to_glamo_framebuffer(fb);
	struct drm_device *dev = fb->dev;
	printk(KERN_CRIT "glamo_user_framebuffer_destroy\n");

	drm_framebuffer_cleanup(fb);
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(glamo_fb->obj);
	mutex_unlock(&dev->struct_mutex);

	kfree(glamo_fb);
}

static int glamo_framebuffer_create_handle(struct drm_framebuffer *fb,
						struct drm_file *file_priv,
						unsigned int *handle)
{
	struct glamo_framebuffer *glamo_fb = to_glamo_framebuffer(fb);
	struct drm_gem_object *object = glamo_fb->obj;
	printk(KERN_CRIT "glamo_user_framebuffer_create_handle\n");

	return drm_gem_handle_create(file_priv, object, handle);
}


static const struct drm_framebuffer_funcs glamo_fb_funcs = {
	.destroy = glamo_framebuffer_destroy,
	.create_handle = glamo_framebuffer_create_handle,
};


int glamo_framebuffer_create(struct drm_device *dev,
			     struct drm_mode_fb_cmd *mode_cmd,
			     struct drm_framebuffer **fb,
			     struct drm_gem_object *obj)
{
	struct glamo_framebuffer *glamo_fb;
	int ret;

	printk(KERN_CRIT "glamo_framebuffer_create\n");
	glamo_fb = kzalloc(sizeof(*glamo_fb), GFP_KERNEL);
	if (!glamo_fb)
		return -ENOMEM;

	ret = drm_framebuffer_init(dev, &glamo_fb->base, &glamo_fb_funcs);
	if (ret) {
		DRM_ERROR("framebuffer init failed %d\n", ret);
		return ret;
	}

	drm_helper_mode_fill_fb_struct(&glamo_fb->base, mode_cmd);

	glamo_fb->obj = obj;

	*fb = &glamo_fb->base;

	return 0;
}


static struct drm_framebuffer *
glamo_user_framebuffer_create(struct drm_device *dev,
			      struct drm_file *filp,
			      struct drm_mode_fb_cmd *mode_cmd)
{
	struct drm_gem_object *obj;
	struct drm_framebuffer *fb;
	int ret;
	printk(KERN_CRIT "glamo_user_framebuffer_create\n");

	obj = drm_gem_object_lookup(dev, filp, mode_cmd->handle);
	if (!obj)
		return NULL;

	ret = glamo_framebuffer_create(dev, mode_cmd, &fb, obj);
	if (ret) {
		drm_gem_object_unreference(obj);
		return NULL;
	}

	return fb;
}


int glamo_fbchanged(struct drm_device *dev)
{
	printk(KERN_CRIT "glamo_fb_changed\n");
	return 0;
}


/* CRTC functions */
static const struct drm_crtc_funcs glamo_crtc_funcs = {
	.cursor_set = glamo_crtc_cursor_set,
	.cursor_move = glamo_crtc_cursor_move,
	.gamma_set = glamo_crtc_gamma_set,
	.set_config = drm_crtc_helper_set_config,
	.destroy = glamo_crtc_destroy,
};


/* CRTC helper functions */
static const struct drm_crtc_helper_funcs glamo_crtc_helper_funcs = {
	.dpms = glamo_crtc_dpms,
	.mode_fixup = glamo_crtc_mode_fixup,
	.mode_set = glamo_crtc_mode_set,
	.mode_set_base = glamo_crtc_mode_set_base,
	.prepare = glamo_crtc_prepare,
	.commit = glamo_crtc_commit,
};


/* Connector functions */
static const struct drm_connector_funcs glamo_connector_funcs = {
	.detect = glamo_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = glamo_connector_destroy,
	.set_property = glamo_connector_set_property,
};


/* Connector helper functions */
static const struct drm_connector_helper_funcs glamo_connector_helper_funcs = {
	.mode_valid = glamo_connector_mode_valid,
	.get_modes = glamo_connector_get_modes,
	.best_encoder = glamo_connector_best_encoder,
};


/* Encoder functions */
static const struct drm_encoder_funcs glamo_encoder_funcs = {
	.destroy = glamo_encoder_destroy,
};


/* Encoder helper functions */
static const struct drm_encoder_helper_funcs glamo_encoder_helper_funcs = {
	.dpms = glamo_encoder_dpms,
	.mode_fixup = glamo_encoder_mode_fixup,
	.prepare = glamo_encoder_prepare,
	.commit = glamo_encoder_commit,
	.mode_set = glamo_encoder_mode_set,
};


/* Mode functions */
static const struct drm_mode_config_funcs glamo_mode_funcs = {
	.fb_create = glamo_user_framebuffer_create,
	.fb_changed = glamo_fbchanged
};


static struct drm_mode_set kernelfb_mode;


/* Restore's the kernel's fbcon mode, used for panic path */
void glamo_display_restore(void)
{
	drm_crtc_helper_set_config(&kernelfb_mode);
}


static int glamo_display_panic(struct notifier_block *n, unsigned long ununsed,
                               void *panic_str)
{
	DRM_ERROR("panic occurred, switching back to text console\n");

	glamo_display_restore();
	return 0;
}


static struct notifier_block paniced = {
	.notifier_call = glamo_display_panic,
};


int glamo_display_init(struct drm_device *dev)
{
	struct glamodrm_handle *gdrm;
	struct glamo_crtc *glamo_crtc;
	struct glamo_output *glamo_output;
	struct drm_connector *connector;
	struct glamo_framebuffer *glamo_fb;
	struct fb_info *info;
	struct glamofb_par *par;
	struct drm_mode_set *modeset;

	gdrm = dev->dev_private;

	printk(KERN_CRIT "glamo_display_init\n");

	glamo_engine_enable(gdrm->glamo_core, GLAMO_ENGINE_LCD);
	glamo_engine_reset(gdrm->glamo_core, GLAMO_ENGINE_LCD);

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.funcs = (void *)&glamo_mode_funcs;

	/* Initialise our CRTC object.
	 * Only one connector per CRTC.  We know this: it's kind of soldered. */
	glamo_crtc = kzalloc(sizeof(struct glamo_crtc)
	                   + sizeof(struct drm_connector *), GFP_KERNEL);
	if (glamo_crtc == NULL) return 1;
	glamo_crtc->gdrm = gdrm;
	glamo_crtc->blank_mode = DRM_MODE_DPMS_OFF;
	drm_crtc_init(dev, &glamo_crtc->base, &glamo_crtc_funcs);
	drm_crtc_helper_add(&glamo_crtc->base, &glamo_crtc_helper_funcs);

	glamo_crtc->mode_set.crtc = &glamo_crtc->base;
	glamo_crtc->mode_set.connectors =
	                              (struct drm_connector **)(glamo_crtc + 1);
	glamo_crtc->mode_set.num_connectors = 0;

	/* Create our "output" object: consists of an output and an encoder */
	glamo_output = kzalloc(sizeof(struct glamo_output), GFP_KERNEL);
	if (glamo_output == NULL) return 1;
	connector = &glamo_output->base;
	glamo_output->gdrm = gdrm;

	/* Initialise the connector */
	drm_connector_init(dev, connector, &glamo_connector_funcs,
	                   DRM_MODE_CONNECTOR_Unknown);
	drm_sysfs_connector_add(connector);
	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	/* Initialise the encoder */
	drm_encoder_init(dev, &glamo_output->enc, &glamo_encoder_funcs,
	                 DRM_MODE_ENCODER_DAC);
	glamo_output->enc.possible_crtcs = 1 << 0;
	drm_mode_connector_attach_encoder(&glamo_output->base,
	                                  &glamo_output->enc);

	drm_encoder_helper_add(&glamo_output->enc, &glamo_encoder_helper_funcs);
	drm_connector_helper_add(connector, &glamo_connector_helper_funcs);

	drm_helper_initial_config(dev, false);

	/* Initial setup of the LCD controller */
	glamo_run_lcd_script(gdrm, lcd_init_script,
	                           ARRAY_SIZE(lcd_init_script));

	if (list_empty(&dev->mode_config.fb_kernel_list)) {
		int ret;
		ret = glamofb_create(dev, 480, 640, 480, 640, &glamo_fb);
		if (ret) return -EINVAL;
	}

	info = glamo_fb->base.fbdev;
	par = info->par;

	modeset = &glamo_crtc->mode_set;
	modeset->fb = &glamo_fb->base;
	modeset->connectors[0] = connector;

	par->crtc_ids[0] = glamo_crtc->base.base.id;

	modeset->num_connectors = 1;
	modeset->mode = modeset->crtc->desired_mode;

	par->crtc_count = 1;

	if (register_framebuffer(info) < 0)
		return -EINVAL;

	printk(KERN_INFO "[glamo-drm] fb%d: %s frame buffer device\n",
	       info->node, info->fix.id);

	/* Switch back to kernel console on panic */
	kernelfb_mode = *modeset;
	atomic_notifier_chain_register(&panic_notifier_list, &paniced);
	printk(KERN_INFO "[glamo-drm] registered panic notifier\n");

	return 0;
}
