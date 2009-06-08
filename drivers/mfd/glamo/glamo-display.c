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


#include <drm/drmP.h>
#include <drm/glamo_drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_crtc.h>

#include "glamo-core.h"
#include "glamo-drm-private.h"
#include "glamo-regs.h"


static void notify_blank(struct drm_crtc *crtc, int mode)
{
	struct fb_event event;

	event.info = info;
	event.data = &blank_mode;
	fb_notifier_call_chain(FB_EVENT_CONBLANK, &event);
}


/* Power on/off */
static void glamo_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	dev_dbg(gfb->dev, "glamofb_blank(%u)\n", blank_mode);

	switch (mode) {
	case DRM_MODE_DPMS_OFF:
		/* Simulating FB_BLANK_NORMAL allow turning off backlight */
		if (gfb->blank_mode != FB_BLANK_NORMAL)
			notify_blank(info, FB_BLANK_NORMAL);

		/* LCM need notification before pixel clock is stopped */
		notify_blank(crtc, blank_mode);

		/* disable the pixel clock */
		glamo_engine_clkreg_set(gcore, GLAMO_ENGINE_LCD,
					GLAMO_CLOCK_LCD_EN_DCLK, 0);
		gfb->blank_mode = blank_mode;
		break;
	case DRM_MODE_DPMS_ON:
		/* enable the pixel clock if off */
		if (gfb->blank_mode == DRM_MODE_DPMS_OFF)
			glamo_engine_clkreg_set(gcore,
					GLAMO_ENGINE_LCD,
					GLAMO_CLOCK_LCD_EN_DCLK,
					GLAMO_CLOCK_LCD_EN_DCLK);

		notify_blank(info, blank_mode);
		gfb->blank_mode = blank_mode;
		break;
	}

}


static bool glamo_crtc_mode_fixup(struct drm_crtc *crtc,
                                  struct drm_display_mode *mode,
                                  struct drm_display_mode *adjusted_mode)
{
	return true;
}


static void glamo_crtc_mode_set(struct drm_crtc *crtc,
                                struct drm_display_mode *mode,
                                struct drm_display_mode *adjusted_mode,
                                int x, int y,
                                struct drm_framebuffer *old_fb)
{
}



static void glamo_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
                                     struct drm_framebuffer *old_fb)
{
}


static void glamo_crtc_prepare(struct drm_crtc *crtc)
{
	struct drm_crtc_helper_funcs *crtc_funcs = crtc->helper_private;
	crtc_funcs->dpms(crtc, DRM_MODE_DPMS_OFF);
}


static void glamo_crtc_commit(struct drm_crtc *crtc)
{
	struct drm_crtc_helper_funcs *crtc_funcs = crtc->helper_private;
	crtc_funcs->dpms(crtc, DRM_MODE_DPMS_ON);
}


static int glamo_crtc_cursor_set(struct drm_crtc *crtc,
                                 struct drm_file *file_priv,
                                 uint32_t handle,
                                 uint32_t width, uint32_t height)
{
	return 0;
}


static int glamo_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	return 0;
}


static void glamo_crtc_gamma_set(struct drm_crtc *crtc, u16 *red, u16 *green,
                                 u16 *blue, uint32_t size)
{
}


static void glamo_crtc_destroy(struct drm_crtc *crtc)
{
	struct glamo_crtc *glamo_crtc = to_glamo_crtc(crtc);

	drm_crtc_cleanup(crtc);
	kfree(glamo_crtc);
}


static enum drm_connector_status
glamo_connector_detect(struct drm_connector *connector)
{
	/* One hopes it hasn't been de-soldered... */
	return connector_status_connected;
}


static void glamo_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}


static int glamo_connector_get_modes(struct drm_connector *connector)
{
	return false;
}


static int glamo_connector_set_property(struct drm_connector *connector,
				  struct drm_property *property,
				  uint64_t value)
{
	return 0;
}


static int glamo_connector_mode_valid(struct drm_connector *connector,
                                      struct drm_display_mode *mode)
{
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	if (mode->clock > 400000 || mode->clock < 25000)
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}


struct drm_encoder *
glamo_connector_best_encoder(struct drm_connector *connector)
{
	struct glamo_output *glamo_output = to_glamo_output(connector);

	return &glamo_output->enc;
}


static void glamo_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}


static bool glamo_encoder_mode_fixup(struct drm_encoder *encoder,
                                 struct drm_display_mode *mode,
                                 struct drm_display_mode *adjusted_mode)
{
	return true;
}


void glamo_encoder_prepare(struct drm_encoder *encoder)
{
}


void glamo_encoder_commit(struct drm_encoder *encoder)
{
}


static void glamo_encoder_mode_set(struct drm_encoder *encoder,
                               struct drm_display_mode *mode,
                               struct drm_display_mode *adjusted_mode)
{
}


static void glamo_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}


static void glamo_user_framebuffer_destroy(struct drm_framebuffer *fb)
{
	struct glamo_framebuffer *glamo_fb = to_glamo_framebuffer(fb);
	struct drm_device *dev = fb->dev;

	drm_framebuffer_cleanup(fb);
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(glamo_fb->obj);
	mutex_unlock(&dev->struct_mutex);

	kfree(glamo_fb);
}

static int glamo_user_framebuffer_create_handle(struct drm_framebuffer *fb,
						struct drm_file *file_priv,
						unsigned int *handle)
{
	struct glamo_framebuffer *glamo_fb = to_glamo_framebuffer(fb);
	struct drm_gem_object *object = glamo_fb->obj;

	return drm_gem_handle_create(file_priv, object, handle);
}



static const struct drm_framebuffer_funcs glamo_fb_funcs = {
	.destroy = glamo_user_framebuffer_destroy,
	.create_handle = glamo_user_framebuffer_create_handle,
};


int glamo_framebuffer_create(struct drm_device *dev,
			     struct drm_mode_fb_cmd *mode_cmd,
			     struct drm_framebuffer **fb,
			     struct drm_gem_object *obj)
{
	struct glamo_framebuffer *glamo_fb;
	int ret;

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


int glamo_fb_changed(struct drm_device *dev)
{
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
	.fb_changed = glamofb_fbchanged
};


int glamo_display_init(struct drm_device *dev)
{
	struct glamodrm_handle *gdrm;
	struct glamo_crtc *glamo_crtc;
	struct glamo_output *glamo_output;
	struct drm_connector *connector;

	gdrm = dev->dev_private;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.funcs = (void *)&glamo_mode_funcs;

	/* Initialise our CRTC object */
	glamo_crtc = kzalloc(sizeof(struct glamo_crtc)
	                   + sizeof(struct drm_connector *), GFP_KERNEL);
	if (glamo_crtc == NULL) return 1;
	drm_crtc_init(dev, &glamo_crtc->base, &glamo_crtc_funcs);
	drm_crtc_helper_add(&glamo_crtc->base, &glamo_crtc_helper_funcs);

	/* Create our "output" object: consists of an output and an encoder */
	glamo_output = kzalloc(sizeof(struct glamo_output), GFP_KERNEL);
	if (glamo_output == NULL) return 1;
	connector = &glamo_output->base;

	/* Initialise the connector */
	drm_connector_init(dev, &glamo_output->base, &glamo_connector_funcs,
	                   DRM_MODE_CONNECTOR_Unknown);
	drm_sysfs_connector_add(connector);
	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	/* Initialise the encoder */
	drm_encoder_init(dev, &glamo_output->enc, &glamo_encoder_funcs,
	                 DRM_MODE_ENCODER_DAC);
	drm_mode_connector_attach_encoder(&glamo_output->base,
	                                  &glamo_output->enc);

	drm_encoder_helper_add(&glamo_output->enc, &glamo_encoder_helper_funcs);
	drm_connector_helper_add(connector, &glamo_connector_helper_funcs);

	return 0;
}
