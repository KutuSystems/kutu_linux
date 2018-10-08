/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/clk.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_edid.h>

#include "kutu_hdmi_drv.h"

#include "../i2c/adv7511.h"

#define AXI_HDMI_STATUS_VMDA_UNDERFLOW	BIT(4)
#define AXI_HDMI_STATUS_VMDA_OVERFLOW	BIT(3)
#define AXI_HDMI_STATUS_VMDA_BE_ERROR	BIT(2)
#define AXI_HDMI_STATUS_VMDA_TPM_OOS	BIT(1)
#define AXI_HDMI_STATUS_HDMI_TPM_OOS	BIT(0)

#define AXI_HDMI_COLOR_PATTERN_ENABLE	BIT(24)

#define AXI_HDMI_REG_RESET		0x040
#define AXI_HDMI_REG_CTRL		0x044
#define AXI_HDMI_REG_SOURCE_SEL		0x048
#define AXI_HDMI_REG_COLORPATTERN	0x04c
#define AXI_HDMI_REG_STATUS		0x05c
#define AXI_HDMI_REG_VDMA_STATUS	0x060
#define AXI_HDMI_REG_TPM_STATUS		0x064
#define AXI_HDMI_REG_HTIMING1		0x400
#define AXI_HDMI_REG_HTIMING2		0x404
#define AXI_HDMI_REG_HTIMING3		0x408
#define AXI_HDMI_REG_VTIMING1		0x440
#define AXI_HDMI_REG_VTIMING2		0x444
#define AXI_HDMI_REG_VTIMING3		0x448

#define AXI_HDMI_RESET_ENABLE		BIT(0)

#define AXI_HDMI_CTRL_SS_BYPASS		BIT(2)
#define AXI_HDMI_CTRL_FULL_RANGE	BIT(1)
#define AXI_HDMI_CTRL_CSC_BYPASS	BIT(0)

#define AXI_HDMI_SOURCE_SEL_COLORPATTERN	0x3
#define AXI_HDMI_SOURCE_SEL_TESTPATTERN		0x2
#define AXI_HDMI_SOURCE_SEL_NORMAL		0x1
#define AXI_HDMI_SOURCE_SEL_NONE		0x0

static const struct debugfs_reg32 kutu_hdmi_encoder_debugfs_regs[] = {
	{ "Reset", AXI_HDMI_REG_RESET },
	{ "Control", AXI_HDMI_REG_CTRL },
	{ "Source select", AXI_HDMI_REG_SOURCE_SEL },
	{ "Colorpattern", AXI_HDMI_REG_COLORPATTERN },
	{ "Status", AXI_HDMI_REG_STATUS },
	{ "VDMA status", AXI_HDMI_REG_VDMA_STATUS },
	{ "TPM status", AXI_HDMI_REG_TPM_STATUS },
	{ "HTiming1", AXI_HDMI_REG_HTIMING1 },
	{ "HTiming2", AXI_HDMI_REG_HTIMING2 },
	{ "HTiming3", AXI_HDMI_REG_HTIMING3 },
	{ "VTiming1", AXI_HDMI_REG_VTIMING1 },
	{ "VTiming2", AXI_HDMI_REG_VTIMING2 },
	{ "VTiming3", AXI_HDMI_REG_VTIMING3 },
};

static const uint16_t adv7511_csc_ycbcr_to_rgb[] = {
	0x0734, 0x04ad, 0x0000, 0x1c1b,
	0x1ddc, 0x04ad, 0x1f24, 0x0135,
	0x0000, 0x04ad, 0x087c, 0x1b77,
};

static const struct drm_display_mode hdmi_1080p60_mode = {
	.clock = 148500,
	.hdisplay = 1920,
	.hsync_start = 1920 + 88,
	.hsync_end = 1920 + 88 + 44,
	.htotal = 2200,
	.vdisplay = 1080,
	.vsync_start = 1080 + 4,
	.vsync_end = 1080 + 4 + 5,
	.vtotal = 1125,
	.vrefresh = 60,
};

struct kutu_hdmi_encoder {
	struct drm_encoder_slave encoder;
	struct drm_connector connector;

#ifdef CONFIG_DEBUG_FS
	struct debugfs_regset32 regset;
#endif
};

static inline struct kutu_hdmi_encoder *to_kutu_hdmi_encoder(struct drm_encoder *enc)
{
	return container_of(enc, struct kutu_hdmi_encoder, encoder.base);
}

static inline struct drm_encoder *connector_to_encoder(struct drm_connector *connector)
{
	struct kutu_hdmi_encoder *enc = container_of(connector, struct kutu_hdmi_encoder, connector);
	return &enc->encoder.base;
}

static int kutu_hdmi_connector_init(struct drm_device *dev,
	struct drm_connector *connector, struct drm_encoder *encoder);

static const struct drm_encoder_slave_funcs *get_slave_funcs(
	struct drm_encoder *enc)
{
	if (enc->bridge)
		return NULL;

	return to_encoder_slave(enc)->slave_funcs;
}


static void kutu_hdmi_encoder_enable(struct drm_encoder *encoder)
{
	struct kutu_hdmi_encoder *kutu_hdmi_encoder = to_kutu_hdmi_encoder(encoder);
	struct drm_connector *connector;
	struct kutu_hdmi_private *private = encoder->dev->dev_private;
	const struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct adv7511_video_config config;
	struct edid *edid;

	if (!private->clk_enabled) {
		clk_prepare_enable(private->hdmi_clock);
		private->clk_enabled = true;
	}
	writel(AXI_HDMI_RESET_ENABLE, private->base + AXI_HDMI_REG_RESET);

	if (!sfuncs)
		return;

	connector = &kutu_hdmi_encoder->connector;
	edid = drm_connector_get_edid(connector);

	if (edid)
		config.hdmi_mode = drm_detect_hdmi_monitor(edid);
	else
		config.hdmi_mode = false;

	hdmi_avi_infoframe_init(&config.avi_infoframe);

	config.avi_infoframe.scan_mode = HDMI_SCAN_MODE_UNDERSCAN;

	if (private->is_rgb) {
			config.csc_enable = false;
			config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
	} else {
		config.csc_scaling_factor = ADV7511_CSC_SCALING_4;
		config.csc_coefficents = adv7511_csc_ycbcr_to_rgb;

		if ((connector->display_info.color_formats & DRM_COLOR_FORMAT_YCRCB422) &&
			config.hdmi_mode) {
			config.csc_enable = false;
			config.avi_infoframe.colorspace = HDMI_COLORSPACE_YUV422;
		} else {
			config.csc_enable = true;
			config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
		}
	}

	if (sfuncs->set_config)
		sfuncs->set_config(encoder, &config);

	if (sfuncs->dpms)
		sfuncs->dpms(encoder, DRM_MODE_DPMS_ON);
}

static void kutu_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct kutu_hdmi_private *private = encoder->dev->dev_private;
	const struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);

	writel(0, private->base + AXI_HDMI_REG_RESET);
	if (private->clk_enabled) {
		clk_disable_unprepare(private->hdmi_clock);
		private->clk_enabled = false;
	}

	if (sfuncs && sfuncs->dpms)
		sfuncs->dpms(encoder, DRM_MODE_DPMS_OFF);
}

/*
static void kutu_hdmi_encoder_mode_set(struct drm_encoder *encoder,
	struct drm_crtc_state *crtc_state,
	struct drm_connector_state *conn_state)
{
	const struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct kutu_hdmi_private *private = encoder->dev->dev_private;
	struct drm_display_mode *mode = &crtc_state->mode;
	unsigned int h_de_min, h_de_max;
	unsigned int v_de_min, v_de_max;
	unsigned int val;

	if (sfuncs && sfuncs->mode_set)
		sfuncs->mode_set(encoder, mode, &crtc_state->adjusted_mode);

	h_de_min = mode->htotal - mode->hsync_start;
	h_de_max = h_de_min + mode->hdisplay;
	v_de_min = mode->vtotal - mode->vsync_start;
	v_de_max = v_de_min + mode->vdisplay;

	val = (mode->hdisplay << 16) | mode->htotal;
	writel(val,  private->base + AXI_HDMI_REG_HTIMING1);
	val = mode->hsync_end - mode->hsync_start;
	writel(val,  private->base + AXI_HDMI_REG_HTIMING2);
	val = (h_de_max << 16) | h_de_min;
	writel(val,  private->base + AXI_HDMI_REG_HTIMING3);

	val = (mode->vdisplay << 16) | mode->vtotal;
	writel(val,  private->base + AXI_HDMI_REG_VTIMING1);
	val = mode->vsync_end - mode->vsync_start;
	writel(val,  private->base + AXI_HDMI_REG_VTIMING2);
	val = (v_de_max << 16) | v_de_min;
	writel(val,  private->base + AXI_HDMI_REG_VTIMING3);

	clk_set_rate(private->hdmi_clock, mode->clock * 1000);
}
*/

static const struct drm_encoder_helper_funcs kutu_hdmi_encoder_helper_funcs = {
	.enable = kutu_hdmi_encoder_enable,
	.disable = kutu_hdmi_encoder_disable,
//	.atomic_mode_set = kutu_hdmi_encoder_mode_set,
};

static void kutu_hdmi_encoder_destroy(struct drm_encoder *encoder)
{
	const struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	struct kutu_hdmi_encoder *kutu_hdmi_encoder =
		to_kutu_hdmi_encoder(encoder);

	if (sfuncs && sfuncs->destroy)
		sfuncs->destroy(encoder);

	drm_encoder_cleanup(encoder);
	kfree(kutu_hdmi_encoder);
}

static const struct drm_encoder_funcs kutu_hdmi_encoder_funcs = {
	.destroy = kutu_hdmi_encoder_destroy,
};

struct drm_encoder *kutu_hdmi_encoder_create(struct drm_device *dev)
{
   struct drm_encoder *encoder;
   struct kutu_hdmi_encoder *kutu_hdmi_encoder;
   struct drm_connector *connector;

   kutu_hdmi_encoder = kzalloc(sizeof(*kutu_hdmi_encoder), GFP_KERNEL);
   if (!kutu_hdmi_encoder)
      return NULL;

   pr_err("kutu_hdmi_encoder_create alloc succeeded\n");

   encoder = &kutu_hdmi_encoder->encoder.base;
   encoder->possible_crtcs = 1;

   drm_encoder_init(dev, encoder, &kutu_hdmi_encoder_funcs, DRM_MODE_ENCODER_TMDS, NULL);
   drm_encoder_helper_add(encoder, &kutu_hdmi_encoder_helper_funcs);

   connector = &kutu_hdmi_encoder->connector;
   kutu_hdmi_connector_init(dev, connector, encoder);

   return encoder;
}

static int kutu_hdmi_connector_get_modes(struct drm_connector *connector)
{
//	struct drm_encoder *encoder = connector_to_encoder(connector);
//	const struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	int ret = 0;

/*	if (sfuncs && sfuncs->get_modes)
		count += sfuncs->get_modes(encoder, connector);

	return count;*/

//	ret = drm_add_modes_noedid(connector, 1920, 1080);

   pr_err("kutu_hdmi get_modes has %d modes\n",ret);

   // return 0;

	/* And prefer a mode pretty much anyone can handle */
	drm_set_preferred_mode(connector, 1920, 1080);

 //  return 0;

  return 1;
}

static int kutu_hdmi_connector_mode_valid(struct drm_connector *connector,
	struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	return MODE_OK;
}

static struct drm_encoder *kutu_hdmi_best_encoder(struct drm_connector *connector)
{
	return connector_to_encoder(connector);
}

static struct drm_connector_helper_funcs kutu_hdmi_connector_helper_funcs = {
	.get_modes	= kutu_hdmi_connector_get_modes,
	.mode_valid	= kutu_hdmi_connector_mode_valid,
	.best_encoder	= kutu_hdmi_best_encoder,
};

static enum drm_connector_status kutu_hdmi_connector_detect(
	struct drm_connector *connector, bool force)
{
	enum drm_connector_status status = connector_status_unknown;
	struct drm_encoder *encoder = connector_to_encoder(connector);
	const struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);

	if (sfuncs && sfuncs->detect)
		status = sfuncs->detect(encoder, connector);

	return status;
}

static void kutu_hdmi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs kutu_hdmi_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = kutu_hdmi_connector_detect,
	.destroy = kutu_hdmi_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int kutu_hdmi_connector_init(struct drm_device *dev,
	struct drm_connector *connector, struct drm_encoder *encoder)
{
	int type;
	int err;

	type = DRM_MODE_CONNECTOR_HDMIA;

	connector->polled = DRM_CONNECTOR_POLL_CONNECT | DRM_CONNECTOR_POLL_DISCONNECT;

	drm_connector_init(dev, connector, &kutu_hdmi_connector_funcs, type);
	drm_connector_helper_add(connector, &kutu_hdmi_connector_helper_funcs);

	err = drm_connector_register(connector);
	if (err)
		goto err_connector;

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
