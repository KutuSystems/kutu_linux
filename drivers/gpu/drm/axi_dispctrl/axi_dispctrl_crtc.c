/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma/xilinx_dma.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include "axi_dispctrl_crtc.h"
#include "axi_dispctrl_drv.h"
#include "axi_dispctrl_encoder.h"

struct axi_dispctrl_crtc {
	struct drm_crtc drm_crtc;
	struct dma_chan *dma;
	struct xilinx_dma_config dma_config;
	int mode;
};

static inline struct axi_dispctrl_crtc *to_axi_dispctrl_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct axi_dispctrl_crtc, drm_crtc);
}

static int axi_dispctrl_crtc_update(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_framebuffer *fb = crtc->primary->fb;
	struct dma_async_tx_descriptor *desc;
	struct drm_gem_cma_object *obj;
	size_t offset;

	if (!mode || !fb)
		return -EINVAL;

	dmaengine_terminate_all(axi_dispctrl_crtc->dma);

	if (axi_dispctrl_crtc->mode == DRM_MODE_DPMS_ON) {
		obj = drm_fb_cma_get_gem_obj(fb, 0);
		if (!obj)
			return -EINVAL;

		axi_dispctrl_crtc->dma_config.hsize = mode->hdisplay * fb->bits_per_pixel / 8;
		axi_dispctrl_crtc->dma_config.vsize = mode->vdisplay;
		axi_dispctrl_crtc->dma_config.stride = fb->pitches[0];

//		dmaengine_device_control(axi_dispctrl_crtc->dma, DMA_SLAVE_CONFIG,
//			(unsigned long)&axi_dispctrl_crtc->dma_config);
		dmaengine_slave_config(axi_dispctrl_crtc->dma,
			(struct dma_slave_config *)&axi_dispctrl_crtc->dma_config);

		offset = crtc->x * fb->bits_per_pixel / 8 + crtc->y * fb->pitches[0];

		desc = dmaengine_prep_slave_single(axi_dispctrl_crtc->dma,
					obj->paddr + offset,
					mode->vdisplay * fb->pitches[0],
					DMA_MEM_TO_DEV, 0);
		if (!desc) {
			pr_err("Failed to prepare DMA descriptor\n");
			return -ENOMEM;
		} else {
			dmaengine_submit(desc);
			dma_async_issue_pending(axi_dispctrl_crtc->dma);
		}
	}

	return 0;
}

static void axi_dispctrl_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);

	if (axi_dispctrl_crtc->mode != mode) {
		axi_dispctrl_crtc->mode = mode;
		axi_dispctrl_crtc_update(crtc);
	}
}

static void axi_dispctrl_crtc_prepare(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);

	dmaengine_terminate_all(axi_dispctrl_crtc->dma);
}

static void axi_dispctrl_crtc_commit(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);

	axi_dispctrl_crtc->mode = DRM_MODE_DPMS_ON;
	axi_dispctrl_crtc_update(crtc);
}

static bool axi_dispctrl_crtc_mode_fixup(struct drm_crtc *crtc,
	const struct drm_display_mode *mode,
	struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int axi_dispctrl_crtc_mode_set(struct drm_crtc *crtc,
	struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode,
	int x, int y, struct drm_framebuffer *old_fb)
{
	/* We do everything in commit() */
	return 0;
}

static int axi_dispctrl_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
	struct drm_framebuffer *old_fb)
{
	return axi_dispctrl_crtc_update(crtc);
}

static void axi_dispctrl_crtc_load_lut(struct drm_crtc *crtc)
{
}

static struct drm_crtc_helper_funcs axi_dispctrl_crtc_helper_funcs = {
	.dpms		= axi_dispctrl_crtc_dpms,
	.prepare	= axi_dispctrl_crtc_prepare,
	.commit		= axi_dispctrl_crtc_commit,
	.mode_fixup	= axi_dispctrl_crtc_mode_fixup,
	.mode_set	= axi_dispctrl_crtc_mode_set,
	.mode_set_base	= axi_dispctrl_crtc_mode_set_base,
	.load_lut	= axi_dispctrl_crtc_load_lut,
};

static void axi_dispctrl_crtc_destroy(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);

	drm_crtc_cleanup(crtc);
	kfree(axi_dispctrl_crtc);
}

static struct drm_crtc_funcs axi_dispctrl_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.destroy	= axi_dispctrl_crtc_destroy,
};

struct drm_crtc *axi_dispctrl_crtc_create(struct drm_device *dev)
{
	struct axi_dispctrl_private *p = dev->dev_private;
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;
	struct drm_crtc *crtc;

	axi_dispctrl_crtc = kzalloc(sizeof(*axi_dispctrl_crtc), GFP_KERNEL);
	if (!axi_dispctrl_crtc) {
		DRM_ERROR("failed to allocate axi_hdmi crtc\n");
		return ERR_PTR(-ENOMEM);
	}

	crtc = &axi_dispctrl_crtc->drm_crtc;

	axi_dispctrl_crtc->dma = p->dma;

	drm_crtc_init(dev, crtc, &axi_dispctrl_crtc_funcs);
	drm_crtc_helper_add(crtc, &axi_dispctrl_crtc_helper_funcs);

	return crtc;
}
