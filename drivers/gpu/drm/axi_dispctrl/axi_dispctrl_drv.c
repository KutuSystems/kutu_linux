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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "axi_dispctrl_drv.h"
#include "axi_dispctrl_crtc.h"
#include "axi_dispctrl_encoder.h"

#define DRIVER_NAME "axi_dispctrl_drm"
#define DRIVER_DESC "AXI DISPCTRL DRM"
#define DRIVER_DATE "20141128"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

static void axi_dispctrl_output_poll_changed(struct drm_device *dev)
{
        struct axi_dispctrl_private *private = dev->dev_private;
        drm_fbdev_cma_hotplug_event(private->fbdev);
}

static struct drm_mode_config_funcs axi_dispctrl_mode_config_funcs = { 
        .fb_create = drm_fb_cma_create,
        .output_poll_changed = axi_dispctrl_output_poll_changed,
};

static void axi_dispctrl_mode_config_init(struct drm_device *dev)
{
        dev->mode_config.min_width = 0;
        dev->mode_config.min_height = 0;

        dev->mode_config.max_width = 4096;
        dev->mode_config.max_height = 4096;

        dev->mode_config.funcs = &axi_dispctrl_mode_config_funcs;
}

static int axi_dispctrl_load(struct drm_device *dev, unsigned long flags)
{
	struct axi_dispctrl_private *private = dev_get_drvdata(dev->dev);
	struct drm_encoder *encoder;
	int ret;

	private->drm_dev = dev;
	dev->dev_private = private;

	drm_mode_config_init(dev);

	drm_kms_helper_poll_init(dev);

	axi_dispctrl_mode_config_init(dev);

	private->crtc = axi_dispctrl_crtc_create(dev);
        if (IS_ERR(private->crtc)) {
                ret = PTR_ERR(private->crtc);
                goto err_crtc;
        }

        encoder = axi_dispctrl_encoder_create(dev);
        if (IS_ERR(encoder)) {
            ret = PTR_ERR(encoder);
            goto err_crtc;
        }

        private->fbdev = drm_fbdev_cma_init(dev, 32, 1, 1);
        if (IS_ERR(private->fbdev)) {
                DRM_ERROR("failed to initialize drm fbdev\n");
                ret = PTR_ERR(private->fbdev);
                goto err_crtc;
        }

        return 0;

err_crtc:
        drm_mode_config_cleanup(dev);
        return ret;
}


static int axi_dispctrl_unload(struct drm_device *dev)
{
	return 0;
}

static void axi_dispctrl_lastclose(struct drm_device *dev)
{
	return;
}

static const struct file_operations axi_dispctrl_driver_fops = {
        .owner          = THIS_MODULE,
        .open           = drm_open,
        .mmap           = drm_gem_cma_mmap,
        .poll           = drm_poll,
        .read           = drm_read,
        .unlocked_ioctl = drm_ioctl,
        .release        = drm_release,
};

static struct drm_driver axi_dispctrl_driver = {
        .driver_features        = DRIVER_MODESET | DRIVER_GEM,
        .load                   = axi_dispctrl_load,
        .unload                 = axi_dispctrl_unload,
        .lastclose              = axi_dispctrl_lastclose,
        .gem_free_object        = drm_gem_cma_free_object,
        .gem_vm_ops             = &drm_gem_cma_vm_ops,
        .dumb_create            = drm_gem_cma_dumb_create,
        .dumb_map_offset        = drm_gem_cma_dumb_map_offset,
        .dumb_destroy           = drm_gem_dumb_destroy,
        .fops                   = &axi_dispctrl_driver_fops,
        .name                   = DRIVER_NAME,
        .desc                   = DRIVER_DESC,
        .date                   = DRIVER_DATE,
        .major                  = DRIVER_MAJOR,
        .minor                  = DRIVER_MINOR,
};

static const struct of_device_id axi_dispctrl_encoder_of_match[] = {
        {
                .compatible = "ant,axi-dispctrl-hdmi-tx",
        },
	{
		.compatible = "ant,axi-dispctrl-lcd",
	},
        {},
};
MODULE_DEVICE_TABLE(of, axi_dispctrl_encoder_of_match);

static int axi_dispctrl_platform_probe(struct platform_device *pdev)
{
	struct axi_dispctrl_private *private;
	struct device_node *child, *np = pdev->dev.of_node;
	const struct of_device_id *id;
	struct resource *res;
	int err;

	private = devm_kzalloc(&pdev->dev, sizeof(*private), GFP_KERNEL);
        if (!private) {
		pr_dev_info("Mem alloc for private data failed \n");
                return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        private->base = devm_ioremap_resource(&pdev->dev, res);
        if (IS_ERR(private->base))
                return PTR_ERR(private->base);

	private->dma = dma_request_slave_channel(&pdev->dev, "video");
        if (private->dma == NULL) 
                return -EPROBE_DEFER;

	private->lcd_mode = of_device_is_compatible(np, "ant,axi-dispctrl-lcd");
	pr_dev_info("We are%sin lcd_mode \n", private->lcd_mode?" ":" not ");

	if (private->lcd_mode) {
		for_each_child_of_node(np, child) {
			id = of_match_node(displays_of_match, child);
			/* check for the predefined LCD's */
			if(id) {
				private->lcd_fixed_mode = devm_kzalloc(&pdev->dev, sizeof(*private->lcd_fixed_mode), GFP_KERNEL);
				if(!private->lcd_fixed_mode)
					return -ENOMEM;
				memcpy((void*)private->lcd_fixed_mode, (void*)id->data, sizeof(*private->lcd_fixed_mode));
			
			/* if any predefined wasn't found check if the custom
		    	   setting are provided */
			}else if (of_device_is_compatible(child, "custom")) {
				private->lcd_fixed_mode = devm_kzalloc(&pdev->dev, sizeof(*private->lcd_fixed_mode), GFP_KERNEL);
				if(!private->lcd_fixed_mode) 
					return -ENOMEM;
				/* get custom LCD properties */
				if((err = of_property_read_u32(child, "lcd,clock", (u32*)&(private->lcd_fixed_mode->clock)))) return err;
				if((err = of_property_read_u32(child, "lcd,hdisplay", (u32*)&(private->lcd_fixed_mode->hdisplay)))) return err;
				if((err = of_property_read_u32(child, "lcd,hsync_start", (u32*)&(private->lcd_fixed_mode->hsync_start)))) return err;
				if((err = of_property_read_u32(child, "lcd,hsync_end", (u32*)&(private->lcd_fixed_mode->hsync_end)))) return err;
				if((err = of_property_read_u32(child, "lcd,htotal", (u32*)&(private->lcd_fixed_mode->htotal)))) return err;
				if((err = of_property_read_u32(child, "lcd,vdisplay", (u32*)&(private->lcd_fixed_mode->vdisplay)))) return err;
				if((err = of_property_read_u32(child, "lcd,vsync_start", (u32*)&(private->lcd_fixed_mode->vsync_start)))) return err;
				if((err = of_property_read_u32(child, "lcd,vsync_end", (u32*)&(private->lcd_fixed_mode->vsync_end)))) return err;
				if((err = of_property_read_u32(child, "lcd,vtotal", (u32*)&(private->lcd_fixed_mode->vtotal)))) return err;
				if((err = of_property_read_u32(child, "lcd,vrefresh", (u32*)&(private->lcd_fixed_mode->vrefresh)))) return err;
				if((err = of_property_read_u32(child, "lcd,flags", (u32*)&(private->lcd_fixed_mode->flags)))) return err;
			}
			/* We can handle only one LCD display */
			if(private->lcd_fixed_mode) {
				private->invert_pix_clk = of_property_read_bool(child, "lcd,invert-pxl-clock");
				pr_dev_info("invert clock is%sset \n", private->invert_pix_clk?" ":" not ");
				break;
			}
		}
		/* check if any display was found */
		if (!private->lcd_fixed_mode) {
			pr_dev_info("No LCD display provided, or provided unknown model\n");
			return -EINVAL;
		}
	}

	platform_set_drvdata(pdev, private);

	return drm_platform_init(&axi_dispctrl_driver, pdev);
}

static int axi_dispctrl_platform_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver axi_dispctrl_encoder_driver = {
        .driver = {
                .name = "axi-dispctrl",
                .owner = THIS_MODULE,
                .of_match_table = axi_dispctrl_encoder_of_match,
        },
        .probe = axi_dispctrl_platform_probe,
        .remove = axi_dispctrl_platform_remove,
};
module_platform_driver(axi_dispctrl_encoder_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Karol Gugala <kgugala@antmicro.com>");
MODULE_DESCRIPTION("");


