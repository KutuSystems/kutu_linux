/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/of_graph.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_atomic_helper.h>

#include "kutu_hdmi_drv.h"

#define DRIVER_NAME	"kutu_hdmi_drm"
#define DRIVER_DESC	"KUTU HDMI DRM"
#define DRIVER_DATE	"20180514"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static void kutu_hdmi_output_poll_changed(struct drm_device *dev)
{
	struct kutu_hdmi_private *private = dev->dev_private;
	drm_fbdev_cma_hotplug_event(private->fbdev);
}

static struct drm_mode_config_funcs axi_hdmi_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = kutu_hdmi_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void kutu_hdmi_mode_config_init(struct drm_device *dev)
{
	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &axi_hdmi_mode_config_funcs;
}

static int kutu_hdmi_load(struct drm_device *dev, unsigned long flags)
{
	struct kutu_hdmi_private *private = dev_get_drvdata(dev->dev);
	struct drm_encoder *encoder;
	int ret;


   pr_err("started kutu_hdmi_load\n");

	private->drm_dev = dev;

	dev->dev_private = private;

	kutu_hdmi_mode_config_init(dev);

	private->crtc = kutu_hdmi_crtc_create(dev);
	if (IS_ERR(private->crtc)) {
		ret = PTR_ERR(private->crtc);
      pr_err("kutu_hdmi_crtc_create failed\n");
		goto err_crtc;
	}
   pr_err("kutu_hdmi_crtc_create succeeded\n");

	encoder = kutu_hdmi_encoder_create(dev);
	if (IS_ERR(encoder)) {
	    ret = PTR_ERR(encoder);
       pr_err("kutu_hdmi_encoder_create failed\n");
	    goto err_crtc;
	}
   pr_err("kutu_hdmi_encoder_create succeeded\n");

	drm_mode_config_reset(dev);

	private->fbdev = drm_fbdev_cma_init(dev, 32, 1, 1);
	if (IS_ERR(private->fbdev)) {
		DRM_ERROR("failed to initialize drm fbdev\n");
		ret = PTR_ERR(private->fbdev);
		goto err_crtc;
	}

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

   pr_err("kutu_hdmi_load succeeded\n");

	return 0;

err_crtc:
	drm_mode_config_cleanup(dev);
	return ret;
}

static int kutu_hdmi_unload(struct drm_device *dev)
{
	struct kutu_hdmi_private *private = dev->dev_private;

	drm_fbdev_cma_fini(private->fbdev);
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);

	return 0;
}

static void kutu_hdmi_lastclose(struct drm_device *dev)
{
	struct kutu_hdmi_private *private = dev->dev_private;
	drm_fbdev_cma_restore_mode(private->fbdev);
}

static const struct file_operations kutu_hdmi_driver_fops = {
	.owner            = THIS_MODULE,
	.open	            = drm_open,
	.mmap	            = drm_gem_cma_mmap,
	.poll	            = drm_poll,
	.read	            = drm_read,
	.unlocked_ioctl   = drm_ioctl,
	.release	         = drm_release,
};

static struct drm_driver kutu_hdmi_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_ATOMIC,
	.load             = kutu_hdmi_load,
	.unload           = kutu_hdmi_unload,
	.lastclose        = kutu_hdmi_lastclose,
	.gem_free_object  = drm_gem_cma_free_object,
	.gem_vm_ops       = &drm_gem_cma_vm_ops,
	.dumb_create      = drm_gem_cma_dumb_create,
	.dumb_map_offset  = drm_gem_cma_dumb_map_offset,
	.dumb_destroy     = drm_gem_dumb_destroy,
	.fops	            = &kutu_hdmi_driver_fops,
	.name	            = DRIVER_NAME,
	.desc             = DRIVER_DESC,
	.date             = DRIVER_DATE,
	.major            = DRIVER_MAJOR,
	.minor            = DRIVER_MINOR,
};

static const struct of_device_id kutu_hdmi_encoder_of_match[] = {
	{
		.compatible = "kutu,hdmi-display",
	},
	{},
};
MODULE_DEVICE_TABLE(of, kutu_hdmi_encoder_of_match);

static int kutu_hdmi_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct device_node *np = pdev->dev.of_node;
	struct kutu_hdmi_private *private;
	struct resource *res;
	int ret;

	private = devm_kzalloc(&pdev->dev, sizeof(*private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;

   pr_err("Started kutu_hdmi probe\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	private->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(private->base)) {
      private->base = 0;
      pr_err("kutu hdmi no base address\n");
	//	return PTR_ERR(private->base);
   }

	private->hdmi_clock = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(private->hdmi_clock)) {
      pr_err("kutu hdmi failed clock probe\n");
	//	return -EPROBE_DEFER;
	}

   pr_err("kutu_hdmi probe stage 2\n");

   private->is_rgb = of_property_read_bool(np, "adi,is-rgb");

   id = of_match_node(kutu_hdmi_encoder_of_match, np);

	private->dma = dma_request_slave_channel(&pdev->dev, "video");
	if (private->dma == NULL) {
      pr_err("kutu_hdmi dma failed\n");
		return -EPROBE_DEFER;
   } else {
      pr_err("kutu_hdmi dma succeeded\n");
   }

	platform_set_drvdata(pdev, private);

	ret = drm_platform_init(&kutu_hdmi_driver, pdev);

   pr_err("kutu_hdmi finshed probe call\n");

   return ret;
}

static int kutu_hdmi_platform_remove(struct platform_device *pdev)
{
	struct kutu_hdmi_private *private = platform_get_drvdata(pdev);

	drm_put_dev(private->drm_dev);
	dma_release_channel(private->dma);
	return 0;
}

static struct platform_driver kutu_hdmi_encoder_driver = {
	.driver = {
		.name = "kutu-hdmi",
		.owner = THIS_MODULE,
		.of_match_table = kutu_hdmi_encoder_of_match,
	},
	.probe = kutu_hdmi_platform_probe,
	.remove = kutu_hdmi_platform_remove,
};
module_platform_driver(kutu_hdmi_encoder_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Smart <greg.smart@kutu.com.au>");
MODULE_DESCRIPTION("");
