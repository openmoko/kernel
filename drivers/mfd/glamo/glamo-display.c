/*
 * SMedia Glamo 336x/337x display
 *
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


static void glamo_crtc_dpms(struct drm_crtc *crtc, int mode)
{
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



static void glamo_pipe_set_base(struct drm_crtc *crtc, int x, int y,
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
	.mode_set_base = glamo_pipe_set_base,
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


int glamo_display_init(struct drm_device *dev)
{
	struct glamodrm_handle *gdrm;
	struct glamo_crtc *glamo_crtc;
	struct glamo_output *glamo_output;
	struct drm_connector *connector;

	gdrm = dev->dev_private;

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
