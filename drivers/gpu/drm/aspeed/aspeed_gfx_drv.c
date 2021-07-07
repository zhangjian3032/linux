// SPDX-License-Identifier: GPL-2.0+
// Copyright 2018 IBM Corporation

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_drv.h>

#include "aspeed_gfx.h"

/**
 * DOC: ASPEED GFX Driver
 *
 * This driver is for the ASPEED BMC SoC's 'GFX' display hardware, also called
 * the 'SOC Display Controller' in the datasheet. This driver runs on the ARM
 * based BMC systems, unlike the ast driver which runs on a host CPU and is for
 * a PCIe graphics device.
 *
 * The AST2500 supports a total of 3 output paths:
 *
 *   1. VGA output, the output target can choose either or both to the DAC
 *   or DVO interface.
 *
 *   2. Graphics CRT output, the output target can choose either or both to
 *   the DAC or DVO interface.
 *
 *   3. Video input from DVO, the video input can be used for video engine
 *   capture or DAC display output.
 *
 * Output options are selected in SCU2C.
 *
 * The "VGA mode" device is the PCI attached controller. The "Graphics CRT"
 * is the ARM's internal display controller.
 *
 * The driver only supports a simple configuration consisting of a 40MHz
 * pixel clock, fixed by hardware limitations, and the VGA output path.
 *
 * The driver was written with the 'AST2500 Software Programming Guide' v17,
 * which is available under NDA from ASPEED.
 */

static const struct drm_mode_config_funcs aspeed_gfx_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create,
	.output_poll_changed = drm_fb_helper_output_poll_changed,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static void aspeed_gfx_setup_mode_config(struct drm_device *drm)
{
	struct aspeed_gfx *priv = drm->dev_private;

	drm_mode_config_init(drm);	

	drm->mode_config.min_width = 0;
	drm->mode_config.min_height = 0;

	if (priv->version == GFX_AST2600) {
		if (priv->dp_support) {
			drm->mode_config.max_width = 1280;
			drm->mode_config.max_height = 1024;
		} else {
			drm->mode_config.max_width = 1024;
			drm->mode_config.max_height = 768;
		}
	} else {
		drm->mode_config.max_width = 800;
		drm->mode_config.max_height = 600;
	}

	drm->mode_config.funcs = &aspeed_gfx_mode_config_funcs;
}

static irqreturn_t aspeed_gfx_irq_handler(int irq, void *data)
{
	struct drm_device *drm = data;
	struct aspeed_gfx *priv = drm->dev_private;
	u32 reg;

	reg = readl(priv->base + CRT_CTRL1);

	if (reg & CRT_CTRL_VERTICAL_INTR_STS) {
		drm_crtc_handle_vblank(&priv->pipe.crtc);
		if(priv->version == GFX_AST2600)
			writel(CRT_CTRL_VERTICAL_INTR_STS, priv->base + CRT_STATUS);
		else
			writel(reg, priv->base + CRT_CTRL1);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static const struct of_device_id aspeed_gfx_match[] = {
	{ .compatible = "aspeed,ast2500-gfx",  .data = (void *) GFX_AST2500},
	{ .compatible = "aspeed,ast2600-gfx",  .data = (void *) GFX_AST2600},
	{ }
};

static int aspeed_gfx_load(struct drm_device *drm)
{
	struct platform_device *pdev = to_platform_device(drm->dev);
	const struct of_device_id *dev_id;	
	struct aspeed_gfx *priv;
	struct resource *res;
	int ret;
	u32 reg = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	drm->dev_private = priv;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(drm->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);


	dev_id = of_match_device(aspeed_gfx_match, &pdev->dev);
	if (!dev_id)
		return -EINVAL;

	priv->version = (int)dev_id->data;
	priv->scu = syscon_regmap_lookup_by_compatible("aspeed,aspeed-scu");
	if (IS_ERR(priv->scu)) {
		dev_err(&pdev->dev, "failed to find SCU regmap\n");
		return PTR_ERR(priv->scu);
	}

	ret = of_reserved_mem_device_init(drm->dev);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to initialize reserved mem: %d\n", ret);
		return ret;
	}

	ret = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "failed to set DMA mask: %d\n", ret);
		return ret;
	}

	priv->crt_rst = devm_reset_control_get(&pdev->dev, "crt");
	if (IS_ERR(priv->crt_rst)) {
		dev_err(&pdev->dev,
			"missing or invalid crt reset controller device tree entry");
		return PTR_ERR(priv->crt_rst);
	}

	reset_control_deassert(priv->crt_rst);

	priv->engine_rst = devm_reset_control_get(&pdev->dev, "engine");
	if (IS_ERR(priv->engine_rst)) {
		dev_err(&pdev->dev,
			"missing or invalid engine reset controller device tree entry");
		return PTR_ERR(priv->engine_rst);
	}

	reset_control_deassert(priv->engine_rst);

	priv->clk = devm_clk_get(drm->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev,
			"missing or invalid clk device tree entry");
		return PTR_ERR(priv->clk);
	}
	clk_prepare_enable(priv->clk);

	if (priv->version == GFX_AST2600) {

		/* check AST DP is executed or not*/
		ret = regmap_read(priv->scu, 0x100, &reg);

		if (((reg>>8) & DP_EXECUTE) == DP_EXECUTE) {
			priv->dp_support = 0x1;

			priv->dp = syscon_regmap_lookup_by_compatible(DP_CP_NAME);
			if (IS_ERR(priv->dp)) {
				dev_err(&pdev->dev, "failed to find DP regmap\n");
				return PTR_ERR(priv->dp);
			}

			priv->dpmcu = syscon_regmap_lookup_by_compatible(DP_MCU_CP_NAME);
			if (IS_ERR(priv->dpmcu)) {
				dev_err(&pdev->dev, "failed to find DP MCU regmap\n");
				return PTR_ERR(priv->dpmcu);
			}

			/* change the dp setting is coming from soc display */
			regmap_update_bits(priv->dp, 0xb8, DP_CONTROL_FROM_SOC, DP_CONTROL_FROM_SOC);
		} else {
			priv->dp_support = 0x0;
			priv->dp = NULL;
			priv->dpmcu = NULL;
		}
	}

	/* Sanitize control registers */
	writel(0, priv->base + CRT_CTRL1);
	writel(0, priv->base + CRT_CTRL2);

	aspeed_gfx_setup_mode_config(drm);

	ret = drm_vblank_init(drm, 1);
	if (ret < 0) {
		dev_err(drm->dev, "Failed to initialise vblank\n");
		return ret;
	}

	ret = aspeed_gfx_create_output(drm);
	if (ret < 0) {
		dev_err(drm->dev, "Failed to create outputs\n");
		return ret;
	}

	ret = aspeed_gfx_create_pipe(drm);
	if (ret < 0) {
		dev_err(drm->dev, "Cannot setup simple display pipe\n");
		return ret;
	}

	ret = devm_request_irq(drm->dev, platform_get_irq(pdev, 0),
			       aspeed_gfx_irq_handler, 0, "aspeed gfx", drm);
	if (ret < 0) {
		dev_err(drm->dev, "Failed to install IRQ handler\n");
		return ret;
	}

	drm_mode_config_reset(drm);

	drm_kms_helper_poll_init(drm);

	return 0;
}

static void aspeed_gfx_unload(struct drm_device *drm)
{
	struct aspeed_gfx *priv = drm->dev_private;

	/* change the dp setting is coming from host side */
	if (priv->dp_support)
		regmap_update_bits(priv->dp, 0xb8, DP_CONTROL_FROM_SOC, 0);

	drm_kms_helper_poll_fini(drm);
	drm_mode_config_cleanup(drm);

	drm->dev_private = NULL;
}

DEFINE_DRM_GEM_CMA_FOPS(fops);

static struct drm_driver aspeed_gfx_driver = {
	.driver_features        = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.gem_create_object	= drm_cma_gem_create_object_default_funcs,
	.dumb_create		= drm_gem_cma_dumb_create,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_mmap		= drm_gem_prime_mmap,
	.fops = &fops,
	.name = "aspeed-gfx-drm",
	.desc = "ASPEED GFX DRM",
	.date = "20180319",
	.major = 1,
	.minor = 0,
};

static ssize_t dac_mux_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct aspeed_gfx *priv = drm->dev_private;
	u32 val;
	int rc;

	if(priv->version == GFX_AST2600)
		rc = regmap_update_bits(priv->scu, 0xc0, BIT(16), val << 16);
	else 
		rc = regmap_update_bits(priv->scu, 0x2c, BIT(16), val << 16);
	if (rc < 0)
		return 0;

	return count;
}

static ssize_t dac_mux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct aspeed_gfx *priv = drm->dev_private;
	u32 reg;
	int rc;

	if(priv->version == GFX_AST2600)
		rc = regmap_read(priv->scu, 0xc0, &reg);
	else 
		rc = regmap_read(priv->scu, 0x2c, &reg);

	if (rc)
		return rc;

	return sprintf(buf, "%u\n", (reg >> 16) & 0x3);
}
static DEVICE_ATTR_RW(dac_mux);

static ssize_t
vga_pw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct aspeed_gfx *priv = drm->dev_private;
	u32 reg;
	int rc;

	if(priv->version == GFX_AST2600)
		rc = regmap_read(priv->scu, 0xe00, &reg);
	else
		rc = regmap_read(priv->scu, 0x50, &reg);
	if (rc)
		return rc;

	return sprintf(buf, "%u\n", reg & 1);
}
static DEVICE_ATTR_RO(vga_pw);

static struct attribute *aspeed_sysfs_entries[] = {
	&dev_attr_vga_pw.attr,
	&dev_attr_dac_mux.attr,
	NULL,
};

static struct attribute_group aspeed_sysfs_attr_group = {
	.attrs = aspeed_sysfs_entries,
};

static int aspeed_gfx_probe(struct platform_device *pdev)
{
	struct drm_device *drm;
	int ret;

	drm = drm_dev_alloc(&aspeed_gfx_driver, &pdev->dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	ret = aspeed_gfx_load(drm);
	if (ret)
		goto err_free;

	dev_set_drvdata(&pdev->dev, drm);

	ret = sysfs_create_group(&pdev->dev.kobj, &aspeed_sysfs_attr_group);
	if (ret)
		return ret;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_unload;

	drm_fbdev_generic_setup(drm, 32);
	return 0;

err_unload:
	sysfs_remove_group(&pdev->dev.kobj, &aspeed_sysfs_attr_group);
	aspeed_gfx_unload(drm);

err_free:
	drm_dev_put(drm);

	return ret;
}

static int aspeed_gfx_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &aspeed_sysfs_attr_group);
	drm_dev_unregister(drm);
	aspeed_gfx_unload(drm);
	drm_dev_put(drm);

	return 0;
}

static struct platform_driver aspeed_gfx_platform_driver = {
	.probe		= aspeed_gfx_probe,
	.remove		= aspeed_gfx_remove,
	.driver = {
		.name = "aspeed_gfx",
		.of_match_table = aspeed_gfx_match,
	},
};

module_platform_driver(aspeed_gfx_platform_driver);

MODULE_AUTHOR("Joel Stanley <joel@jms.id.au>");
MODULE_DESCRIPTION("ASPEED BMC DRM/KMS driver");
MODULE_LICENSE("GPL");
