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


static const struct drm_crtc_helper_funcs glamo_helper_funcs = {
	.dpms = glamo_crtc_dpms,
	.mode_fixup = glamo_crtc_mode_fixup,
	.mode_set = glamo_crtc_mode_set,
	.mode_set_base = glamo_pipe_set_base,
	.prepare = glamo_crtc_prepare,
	.commit = glamo_crtc_commit,
};


static const struct drm_crtc_funcs glamo_crtc_funcs = {
	.cursor_set = glamo_crtc_cursor_set,
	.cursor_move = glamo_crtc_cursor_move,
	.gamma_set = glamo_crtc_gamma_set,
	.set_config = drm_crtc_helper_set_config,
	.destroy = glamo_crtc_destroy,
};


int glamo_display_init(struct drm_device *dev)
{
	struct glamodrm_handle *gdrm;
	struct glamo_crtc *glamo_crtc;

	glamo_crtc = kzalloc(sizeof(struct glamo_crtc)
	                   + sizeof(struct drm_connector *), GFP_KERNEL);
	if (glamo_crtc == NULL)
		return 1;

	gdrm = dev->dev_private;

	drm_crtc_init(dev, &glamo_crtc->base, &glamo_crtc_funcs);
	drm_crtc_helper_add(&glamo_crtc->base, &glamo_helper_funcs);

	return 0;
}
