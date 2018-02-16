/*
 * AXI display controller DRM driver
 * based on Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2014 Antmicro Ltd <www.antmicro.com>
 *
 * Author(s):
 *   Karol Gugala <kgugala@antmicro.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>

#include "axi_dispctrl_drv.h"
#include "axi_dispctrl_encoder.h"

static unsigned int axi_dispctrl_clk_divider(unsigned int divide);
static unsigned int axi_dispctrl_clk_count_calc(unsigned int divide);
static void axi_dispctrl_encoder_find_clock_parms(unsigned int freq, struct clk_mode *best_pick);

struct axi_dispctrl_encoder {
	struct drm_encoder_slave encoder;
	struct drm_connector connector;
};

static inline struct axi_dispctrl_encoder *to_axi_dispctrl_encoder(struct drm_encoder *enc)
{
	return container_of(enc, struct axi_dispctrl_encoder, encoder.base);
}

static inline struct drm_encoder *connector_to_encoder(struct drm_connector *connector)
{
	struct axi_dispctrl_encoder *enc = container_of(connector, struct axi_dispctrl_encoder, connector);
	return &enc->encoder.base;
}

static int axi_dispctrl_connector_init(struct drm_device *dev,
	struct drm_connector *connector, struct drm_encoder *encoder);

static void axi_dispctrl_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct axi_dispctrl_private *private = encoder->dev->dev_private;
	uint32_t reg;

	reg = readl(private->base + OFST_DISPLAY_CTRL);
	switch (mode) {
	case DRM_MODE_DPMS_ON:
		/* start the display */
		reg |= 1 << BIT_DISPLAY_START;
		writel(reg , private->base + OFST_DISPLAY_CTRL);
		break;
	default:
		reg &= ~(1 << BIT_DISPLAY_START);
		/* stop the display */
		writel(reg, private->base + OFST_DISPLAY_CTRL);
		break;
	}

}

static bool axi_dispctrl_encoder_mode_fixup(struct drm_encoder *encoder,
	const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	/* do nothing */
	return true;
}

unsigned int axi_dispctrl_clk_count_calc(unsigned int divide)
{
        unsigned int output = 0;
        unsigned int divCalc = 0;

        divCalc = axi_dispctrl_clk_divider(divide);
        if (divCalc == ERR_CLKDIVIDER)
                output = ERR_CLKCOUNTCALC;
        else
                output = (0xFFF & divCalc) | ((divCalc << 10) & 0x00C00000);
        return output;
}


static unsigned int axi_dispctrl_clk_divider(unsigned int divide)
{
        unsigned int output = 0;
        unsigned int highTime = 0;
        unsigned int lowTime = 0;

        if ((divide < 1) || (divide > 128))
                return ERR_CLKDIVIDER;

        if (divide == 1)
                return 0x1041;

        highTime = divide / 2;
        if (divide & 0b1) //if divide is odd
        {
                lowTime = highTime + 1;
                output = 1 << CLK_BIT_WEDGE;
        }
        else
        {
                lowTime = highTime;
        }

        output |= 0x03F & lowTime;
        output |= 0xFC0 & (highTime << 6);
        return output;
}

static unsigned int axi_dispctrl_clk_find_reg(struct clk_config *reg_values, struct clk_mode *clk_params)
{
        if ((clk_params->fbmult < 2) || clk_params->fbmult > 64 )
                return 0;

        reg_values->clk0L = axi_dispctrl_clk_count_calc(clk_params->clkdiv);
        if (reg_values->clk0L == ERR_CLKCOUNTCALC)
                return 0;

        reg_values->clkFBL = axi_dispctrl_clk_count_calc(clk_params->fbmult);
        if (reg_values->clkFBL == ERR_CLKCOUNTCALC)
                return 0;

        reg_values->clkFBH_clk0H = 0;

        reg_values->divclk = axi_dispctrl_clk_divider(clk_params->maindiv);
        if (reg_values->divclk == ERR_CLKDIVIDER)
                return 0;

        reg_values->lockL = (unsigned int) (lock_lookup[clk_params->fbmult - 1] & 0xFFFFFFFF);

        reg_values->fltr_lockH = (unsigned int) ((lock_lookup[clk_params->fbmult - 1] >> 32) & 0x000000FF);
        reg_values->fltr_lockH |= ((filter_lookup_low[clk_params->fbmult - 1] << 16) & 0x03FF0000);

        return 1;
}

static void axi_dispctrl_encoder_find_clock_parms(unsigned int freq, struct clk_mode *best_pick)
{
        unsigned int div;
        unsigned int currFreq = 0;
        unsigned int minFb, maxFb;
        unsigned int currMult;
        unsigned int clkDiv = 1;
        int currErr = 0x7fffffff;
        int err;

        for(div = 1; div < 10; div++)
        {
                minFb = div * 6;
                maxFb = div * 12;
                if (maxFb > 64)
                        maxFb = 64;
                currMult = minFb;
                while (currMult <= maxFb)
                {
                        for (clkDiv = 1; clkDiv <128; clkDiv++) {
                                currFreq = (IN_FREQ * currMult) / (div * clkDiv);
                                err = freq - currFreq;
                                /* get abs val */
                                if (err < 0) err = -err;
                                if (err < currErr)
                                {
                                        currErr = err;
                                        best_pick->clkdiv = clkDiv;
                                        best_pick->fbmult = currMult;
                                        best_pick->maindiv = div;
                                        best_pick->freq = currFreq;
                                }
                                if(currFreq == freq) return;
                        }
                        currMult++;
                }
        }
        return;
}


static void axi_dispctrl_encoder_mode_set(struct drm_encoder *encoder,
	struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	struct axi_dispctrl_private *private = encoder->dev->dev_private;
	uint32_t vgaReg[5];
	uint32_t clock;
	uint32_t i;
	uint32_t hpol = 1, vpol = 1;
	uint32_t reg;
	struct clk_mode clk_params;
	struct clk_config clk_regs;

	if(mode->flags & DRM_MODE_FLAG_NHSYNC)
		hpol = 0;
	if(mode->flags & DRM_MODE_FLAG_NHSYNC)
		vpol = 0;

	if(private->invert_pix_clk) {
		/* invert pxl_clk polarization */
		reg = readl(private->base + OFST_DISPLAY_CTRL);
		reg |= 1 << BIT_DISPLAY_INVERT_PIX_CLOCK;
        	writel(reg , private->base + OFST_DISPLAY_CTRL);
	}
	/*pr_dev_info("Mode is:\nhdisplay = %d\nhsync_start = %d\nhsync_end = %d\nhtotal = %d\nhskew = %d\n vdisplay = %d\nvsync_start=%d\nvsync_end = %d\nvtotal = %d\nvscan = %d\n", mode->hdisplay,
		     mode->hsync_start, mode->hsync_end, mode->htotal,
		     mode->hskew, mode->vdisplay, mode->vsync_start,
		     mode->vsync_end, mode->vtotal, mode->vscan); */

	vgaReg[0] = (mode->hdisplay << 16) | (mode->vdisplay);
	vgaReg[1] = (mode->hsync_start << 16) | (mode->hsync_end);
	vgaReg[2] = (hpol << 16) | (mode->htotal);
	vgaReg[3] = (mode->vsync_start << 16) | (mode->vsync_end);
	vgaReg[4] = (vpol << 16) | (mode->vtotal);

    	for (i = 0; i < 5; i++) {
		writel(vgaReg[i], private->base + OFST_DISPLAY_VIDEO_START + (i * 4) );
	}
	/* calc and set clocks */
	/* clk in MHz, and for the HDMI we need 5 times faster clk */
	clock = mode->clock * 1000;
	/*if(!private->lcd_mode)*/ clock *= 5;
	axi_dispctrl_encoder_find_clock_parms (clock, &clk_params);

	axi_dispctrl_clk_find_reg(&clk_regs, &clk_params);

	writel(clk_regs.clk0L, private->base + OFST_DISPLAY_CLK_L);
	writel(clk_regs.clkFBL, private->base + OFST_DISPLAY_FB_L);
	writel(clk_regs.clkFBH_clk0H, private->base + OFST_DISPLAY_FB_H_CLK_H);
	writel(clk_regs.divclk, private->base + OFST_DISPLAY_DIV);
	writel(clk_regs.lockL, private->base + OFST_DISPLAY_LOCK_L);
	writel(clk_regs.fltr_lockH, private->base + OFST_DISPLAY_FLTR_LOCK_H);
}

static void axi_dispctrl_encoder_commit(struct drm_encoder *encoder)
{
	axi_dispctrl_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void axi_dispctrl_encoder_prepare(struct drm_encoder *encoder)
{
	axi_dispctrl_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static struct drm_crtc *axi_dispctrl_encoder_get_crtc(struct drm_encoder *encoder)
{
	return encoder->crtc;
}

static struct drm_encoder_helper_funcs axi_dispctrl_encoder_helper_funcs = {
	.dpms		= axi_dispctrl_encoder_dpms,
	.mode_fixup	= axi_dispctrl_encoder_mode_fixup,
	.mode_set	= axi_dispctrl_encoder_mode_set,
	.prepare	= axi_dispctrl_encoder_prepare,
	.commit		= axi_dispctrl_encoder_commit,
	.get_crtc	= axi_dispctrl_encoder_get_crtc,
};

static void axi_dispctrl_encoder_destroy(struct drm_encoder *encoder)
{
	struct axi_dispctrl_encoder *axi_dispctrl_encoder =
		to_axi_dispctrl_encoder(encoder);

	drm_encoder_cleanup(encoder);
	encoder->dev->mode_config.num_encoder--;
	kfree(axi_dispctrl_encoder);
}

static struct drm_encoder_funcs axi_dispctrl_encoder_funcs = {
	.destroy = axi_dispctrl_encoder_destroy,
};

struct drm_encoder *axi_dispctrl_encoder_create(struct drm_device *dev)
{
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct axi_dispctrl_encoder *axi_dispctrl_encoder;

	axi_dispctrl_encoder = kzalloc(sizeof(*axi_dispctrl_encoder), GFP_KERNEL);
	if (!axi_dispctrl_encoder) {
		DRM_ERROR("failed to allocate encoder\n");
		return NULL;
	}

	encoder = &axi_dispctrl_encoder->encoder.base;
	encoder->possible_crtcs = 1;

	drm_encoder_init(dev, encoder, &axi_dispctrl_encoder_funcs,
			DRM_MODE_ENCODER_TMDS,NULL);
	drm_encoder_helper_add(encoder, &axi_dispctrl_encoder_helper_funcs);

	connector = &axi_dispctrl_encoder->connector;

	axi_dispctrl_connector_init(dev, connector, encoder);

	return encoder;
}

static int axi_dispctrl_connector_get_modes(struct drm_connector *connector)
{
	struct axi_dispctrl_private *private = connector->dev->dev_private;
	struct drm_display_mode *mode;

	int count = 0;
	/* If we are in lcd mode use fixed modes */
	if ( private->lcd_mode ) {
		mode = drm_mode_duplicate(connector->dev, private->lcd_fixed_mode);
		if(!mode)
			return 0;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
		count++;
	}
	/* Get edid otherwise */
	else {
		//XXX: Use real edid
		drm_mode_connector_update_edid_property(connector, (struct edid *) samsung_edid);
		count = drm_add_edid_modes(connector, (struct edid *) samsung_edid);
	}

	return count;
}

static int axi_dispctrl_connector_mode_valid(struct drm_connector *connector,
	struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	return MODE_OK;
}

static struct drm_encoder *axi_dispctrl_best_encoder(struct drm_connector *connector)
{
	return connector_to_encoder(connector);
}

static struct drm_connector_helper_funcs axi_dispctrl_connector_helper_funcs = {
	.get_modes	= axi_dispctrl_connector_get_modes,
	.mode_valid	= axi_dispctrl_connector_mode_valid,
	.best_encoder	= axi_dispctrl_best_encoder,
};

static enum drm_connector_status axi_dispctrl_connector_detect(
	struct drm_connector *connector, bool force)
{
	enum drm_connector_status status = connector_status_connected;

	//XXX: add connection detect
	//Always connected?
	return status;
}

static void axi_dispctrl_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs axi_dispctrl_connector_funcs = {
	.dpms		= drm_helper_connector_dpms,
	.fill_modes	= drm_helper_probe_single_connector_modes,
	.detect		= axi_dispctrl_connector_detect,
	.destroy	= axi_dispctrl_connector_destroy,
};

static int axi_dispctrl_connector_init(struct drm_device *dev,
	struct drm_connector *connector, struct drm_encoder *encoder)
{
	int type;
	int err;
	struct axi_dispctrl_private *private = dev->dev_private;

	if (private->lcd_mode)
		type = DRM_MODE_CONNECTOR_VGA;
	else
		type = DRM_MODE_CONNECTOR_HDMIA;
	connector->polled = DRM_CONNECTOR_POLL_CONNECT |
				DRM_CONNECTOR_POLL_DISCONNECT;

	drm_connector_init(dev, connector, &axi_dispctrl_connector_funcs, type);
	drm_connector_helper_add(connector, &axi_dispctrl_connector_helper_funcs);

	err = drm_connector_register(connector);
	if (err)
		goto err_connector;

	connector->encoder = encoder;

	err = drm_mode_connector_attach_encoder(connector, encoder);
	if (err) {
		DRM_ERROR("failed to attach a connector to a encoder\n");
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	drm_connector_unregister(connector);
err_connector:
	drm_connector_cleanup(connector);
	return err;
}
