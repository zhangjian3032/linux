#include <linux/io.h>
#include <linux/fs.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/dma-mapping.h>

#define SSP_FILE_NAME			"ast2600_ssp.bin"
#define AST2600_CVIC_TRIGGER		0x28
#define AST2600_CVIC_PENDING_STATUS	0x18
#define AST2600_CVIC_PENDING_CLEAR	0x1C

#define SSP_CTRL_REG			0xa00
#define SSP_CTRL_RESET_ASSERT		BIT(1)
#define SSP_CTRL_EN			BIT(0)

#define SSP_MEM_BASE_REG		0xa04
#define SSP_IMEM_LIMIT_REG		0xa08
#define SSP_DMEM_LIMIT_REG		0xa0c
#define SSP_CACHE_RANGE_REG		0xa40
#define SSP_CACHE_INVALID_REG		0xa44
#define SSP_CACHE_CTRL_REG		0xa48
#define SSP_CACHE_CLEAR_ICACHE		BIT(2)
#define SSP_CACHE_CLEAR_DCACHE		BIT(1)
#define SSP_CACHE_EN			BIT(0)

#define SSP_TOTAL_MEM_SZ		(32 * 1024 * 1024)
#define SSP_CACHED_MEM_SZ		(16 * 1024 * 1024)
#define SSP_UNCACHED_MEM_SZ		(SSP_TOTAL_MEM_SZ - SSP_CACHED_MEM_SZ)
#define SSP_CACHE_RANGE_CONFIG		(SSP_CACHED_MEM_SZ >> 24)

struct ast2600_ssp {
	struct device		*dev;
	struct regmap 		*scu;
	dma_addr_t 		hw_addr;
	void __iomem		*ssp_mem;
	void __iomem 		*cvic;
	int			irq[16];
	int			n_irq;
};

static int ast_ssp_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "Device File Opened...!!!\n");
	return 0;
}

static int ast_ssp_release(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "Device File Closed...!!!\n");
    return 0;
}

static struct file_operations ast_ssp_fops =
{       
	.owner			= THIS_MODULE,
	.open			= ast_ssp_open,
	.release		= ast_ssp_release,
	.llseek 		= no_llseek,
};

struct miscdevice ast_ssp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-ssp",
	.fops = &ast_ssp_fops,
};

static irqreturn_t ast2600_ssp_interrupt(int irq, void *dev_id)
{
	struct ast2600_ssp *priv = dev_id;
	writel(readl(priv->cvic + AST2600_CVIC_PENDING_STATUS),
	       priv->cvic + AST2600_CVIC_PENDING_CLEAR);

	return IRQ_HANDLED;
}
static int ast_ssp_probe(struct platform_device *pdev)
{
	const struct firmware *firmware;
	struct device_node *np, *mnode = dev_of_node(&pdev->dev);
	struct ast2600_ssp *priv;
	int i, ret;

	printk("ast2600 SSP init\n");
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}

	priv->dev = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	ret = misc_register(&ast_ssp_misc);
	if (ret) {
		pr_err("can't misc_register :(\n");
		return -EIO;
	}
	dev_set_drvdata(ast_ssp_misc.this_device, pdev);

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
		dev_err(priv->dev,
			"failed to initialize reserved mem: %d\n", ret);
	}
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	priv->ssp_mem = dma_alloc_coherent(priv->dev, SSP_TOTAL_MEM_SZ,
					   &priv->hw_addr, GFP_KERNEL);

	printk("virtual addr = 0x%08x, phy_addr = 0x%08x\n",
	       (uint32_t)priv->ssp_mem, priv->hw_addr);
	if (request_firmware(&firmware, SSP_FILE_NAME, &pdev->dev) < 0) {
		dev_err(&pdev->dev, "don't have %s\n", SSP_FILE_NAME);
		release_firmware(firmware);
		return 0;
	}

	memcpy(priv->ssp_mem, (void *)firmware->data, firmware->size);
	release_firmware(firmware);

	printk("init inter-processor IRQs\n");
	np = of_parse_phandle(mnode, "aspeed,cvic", 0);
	if (!np) {
		dev_err(&pdev->dev, "can't find CVIC\n");
		return -EINVAL;
	}

	priv->cvic = devm_of_iomap(&pdev->dev, np, 0, NULL);
	if (IS_ERR(priv->cvic)) {
		dev_err(&pdev->dev, "can't map CVIC\n");
		return -EINVAL;
	}	

	i = 0;
	while(0 != (priv->irq[i] = irq_of_parse_and_map(mnode, i))) {
		ret = request_irq(priv->irq[i], ast2600_ssp_interrupt, 0,
				  "ssp-sw-irq", priv);
		i++;
	}
	priv->n_irq = i;
	printk("%d SSP ISR registered\n", priv->n_irq);

	printk("init CM3 memory bases\n");
	priv->scu = syscon_regmap_lookup_by_compatible("aspeed,aspeed-scu");
	regmap_write(priv->scu, SSP_CTRL_REG, 0);
	mdelay(1);
	regmap_write(priv->scu, SSP_MEM_BASE_REG, priv->hw_addr);
	regmap_write(priv->scu, SSP_CACHE_CTRL_REG,
		     SSP_CACHE_CLEAR_DCACHE | SSP_CACHE_EN);
	mdelay(1);
	regmap_write(priv->scu, SSP_CACHE_CTRL_REG, SSP_CACHE_EN);
	regmap_write(priv->scu, SSP_IMEM_LIMIT_REG, priv->hw_addr + SSP_CACHED_MEM_SZ);
	regmap_write(priv->scu, SSP_DMEM_LIMIT_REG, priv->hw_addr + SSP_TOTAL_MEM_SZ);

	regmap_write(priv->scu, SSP_CACHE_RANGE_REG, SSP_CACHE_RANGE_CONFIG);

	regmap_write(priv->scu, SSP_CTRL_REG, SSP_CTRL_RESET_ASSERT);
	mdelay(1);
	regmap_write(priv->scu, SSP_CTRL_REG, 0);
	mdelay(1);
	regmap_write(priv->scu, SSP_CTRL_REG, SSP_CTRL_EN);
	printk("CM3 init done\n");
	return 0;
}

static int ast_ssp_remove(struct platform_device *pdev)
{
	struct ast2600_ssp *priv = platform_get_drvdata(pdev);
	int i;

	printk("SSP module removed\n");
	regmap_write(priv->scu, SSP_CTRL_REG, 0);
	for (i = 0; i < priv->n_irq; i++) {
		free_irq(priv->irq[i], priv);
	}

	dma_free_coherent(priv->dev, SSP_TOTAL_MEM_SZ, priv->ssp_mem, priv->hw_addr);
	kfree(priv);

	misc_deregister((struct miscdevice *)&ast_ssp_misc);

	return 0;
}

static const struct of_device_id of_ast_ssp_match_table[] = {
	{ .compatible = "aspeed,ast2600-ssp", },
	{},
};
MODULE_DEVICE_TABLE(of, of_ast_ssp_match_table);

static struct platform_driver ast_ssp_driver = {
	.probe		= ast_ssp_probe,
	.remove 	= ast_ssp_remove,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = of_ast_ssp_match_table,
	},
};

module_platform_driver(ast_ssp_driver);

MODULE_LICENSE("Dual BSD/GPL");
