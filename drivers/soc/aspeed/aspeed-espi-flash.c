// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 */
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/module.h>

#include <soc/aspeed/espi.h>
#include <uapi/linux/aspeed-espi.h>

#define DEVICE_NAME	"aspeed-espi-flash"

enum aspeed_espi_flash_safs_mode {
	SAFS_MODE_MIX,
	SAFS_MODE_SW,
	SAFS_MODE_HW,
	SAFS_MODES,
};

struct aspeed_espi_flash_dma {
	void *tx_virt;
	dma_addr_t tx_addr;
	void *rx_virt;
	dma_addr_t rx_addr;
};

struct aspeed_espi_flash {
	struct regmap *map;

	int irq;
	int irq_reset;

	uint32_t safs_mode;

	uint32_t dma_mode;
	struct aspeed_espi_flash_dma dma;

	uint32_t rx_ready;
	wait_queue_head_t wq;

	struct mutex get_rx_mtx;
	struct mutex put_tx_mtx;

	spinlock_t lock;

	struct miscdevice mdev;

	uint32_t version;
};

static long aspeed_espi_flash_get_rx(struct file *fp,
				     struct aspeed_espi_ioc *ioc,
				     struct aspeed_espi_flash *espi_flash)
{
	int i, rc = 0;
	unsigned long flags;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint8_t *pkt;
	uint32_t pkt_len;
	struct espi_comm_hdr *hdr;

	if (fp->f_flags & O_NONBLOCK) {
		if (mutex_trylock(&espi_flash->get_rx_mtx))
			return -EBUSY;

		if (!espi_flash->rx_ready) {
			rc = -ENODATA;
			goto unlock_mtx_n_out;
		}
	}
	else {
		mutex_lock(&espi_flash->get_rx_mtx);

		if (!espi_flash->rx_ready) {
			rc = wait_event_interruptible(espi_flash->wq,
						      espi_flash->rx_ready);
			if (rc == -ERESTARTSYS) {
				rc = -EINTR;
				goto unlock_mtx_n_out;
			}
		}
	}

	/* common header (i.e. cycle type, tag, and length) is taken by HW */
	regmap_read(espi_flash->map, ESPI_FLASH_RX_CTRL, &reg);
	cyc = (reg & ESPI_FLASH_RX_CTRL_CYC_MASK) >> ESPI_FLASH_RX_CTRL_CYC_SHIFT;
	tag = (reg & ESPI_FLASH_RX_CTRL_TAG_MASK) >> ESPI_FLASH_RX_CTRL_TAG_SHIFT;
	len = (reg & ESPI_FLASH_RX_CTRL_LEN_MASK) >> ESPI_FLASH_RX_CTRL_LEN_SHIFT;

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	switch (cyc) {
	case ESPI_FLASH_READ:
	case ESPI_FLASH_WRITE:
	case ESPI_FLASH_ERASE:
		pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) +
			  sizeof(struct espi_flash_rwe);
		break;
	case ESPI_FLASH_SUC_CMPLT_D_MIDDLE:
	case ESPI_FLASH_SUC_CMPLT_D_FIRST:
	case ESPI_FLASH_SUC_CMPLT_D_LAST:
	case ESPI_FLASH_SUC_CMPLT_D_ONLY:
		pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) +
			  sizeof(struct espi_flash_cmplt);
		break;
	case ESPI_FLASH_SUC_CMPLT:
	case ESPI_FLASH_UNSUC_CMPLT:
		pkt_len = len + sizeof(struct espi_flash_cmplt);
		break;
	default:
		rc = -EFAULT;
		goto unlock_mtx_n_out;
	}

	if (ioc->pkt_len < pkt_len) {
	    rc = -EINVAL;
		goto unlock_mtx_n_out;
	}

	pkt = vmalloc(pkt_len);
	if (!pkt) {
	    rc = -ENOMEM;
		goto unlock_mtx_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;
	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	if (espi_flash->dma_mode) {
		memcpy(hdr + 1, espi_flash->dma.rx_virt,
		       pkt_len - sizeof(*hdr));
	}
	else {
		for (i = sizeof(*hdr); i < pkt_len; ++i) {
			regmap_read(espi_flash->map,
				    ESPI_FLASH_RX_PORT, &reg);
			pkt[i] = reg & 0xff;
		}
	}

	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	spin_lock_irqsave(&espi_flash->lock, flags);

	regmap_write_bits(espi_flash->map, ESPI_FLASH_RX_CTRL,
			  ESPI_FLASH_RX_CTRL_PEND_SERV,
			  ESPI_FLASH_RX_CTRL_PEND_SERV);

	espi_flash->rx_ready = 0;

	spin_unlock_irqrestore(&espi_flash->lock, flags);

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&espi_flash->get_rx_mtx);

	return rc;
}

static long aspeed_espi_flash_put_tx(struct file *fp,
				     struct aspeed_espi_ioc *ioc,
				     struct aspeed_espi_flash *espi_flash)
{
	int i, rc = 0;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint8_t *pkt;
	struct espi_comm_hdr *hdr;

	if (!mutex_trylock(&espi_flash->put_tx_mtx))
		return -EAGAIN;

	regmap_read(espi_flash->map, ESPI_FLASH_TX_CTRL, &reg);
	if (reg & ESPI_FLASH_TX_CTRL_TRIGGER) {
		rc = -EBUSY;
		goto unlock_mtx_n_out;
	}

	pkt = vmalloc(ioc->pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_mtx_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	if (espi_flash->dma_mode) {
		memcpy(espi_flash->dma.tx_virt, hdr + 1,
		       ioc->pkt_len - sizeof(*hdr));
		dma_wmb();
	}
	else {
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			regmap_write(espi_flash->map,
				     ESPI_FLASH_TX_PORT, pkt[i]);
	}

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = ((cyc << ESPI_FLASH_TX_CTRL_CYC_SHIFT) & ESPI_FLASH_TX_CTRL_CYC_MASK)
		| ((tag << ESPI_FLASH_TX_CTRL_TAG_SHIFT) & ESPI_FLASH_TX_CTRL_TAG_MASK)
		| ((len << ESPI_FLASH_TX_CTRL_LEN_SHIFT) & ESPI_FLASH_TX_CTRL_LEN_MASK)
		| ESPI_FLASH_TX_CTRL_TRIGGER;

	regmap_write(espi_flash->map, ESPI_FLASH_TX_CTRL, reg);

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&espi_flash->put_tx_mtx);

	return rc;
}

static long aspeed_espi_flash_ioctl(struct file *fp, unsigned int cmd,
				    unsigned long arg)
{
	struct aspeed_espi_ioc ioc;
	struct aspeed_espi_flash *espi_flash = container_of(
			fp->private_data,
			struct aspeed_espi_flash,
			mdev);

	if (copy_from_user(&ioc, (void __user *)arg, sizeof(ioc)))
	    return -EFAULT;

	if (ioc.pkt_len > ESPI_PKT_LEN_MAX)
		return -EINVAL;

	switch (cmd) {
	case ASPEED_ESPI_FLASH_GET_RX:
		return aspeed_espi_flash_get_rx(fp, &ioc, espi_flash);
	case ASPEED_ESPI_FLASH_PUT_TX:
		return aspeed_espi_flash_put_tx(fp, &ioc, espi_flash);
	};

	return -EINVAL;
}

static const struct file_operations aspeed_espi_flash_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aspeed_espi_flash_ioctl,
};

static irqreturn_t aspeed_espi_flash_isr(int irq, void *arg)
{
	uint32_t sts;
	unsigned long flags;
	struct aspeed_espi_flash *espi_flash = arg;

	regmap_read(espi_flash->map, ESPI_INT_STS, &sts);

	if (!(sts & ESPI_INT_STS_FLASH_BITS))
		return IRQ_NONE;

	if (sts & ESPI_INT_STS_FLASH_RX_CMPLT) {
		spin_lock_irqsave(&espi_flash->lock, flags);
		espi_flash->rx_ready = 1;
		spin_unlock_irqrestore(&espi_flash->lock, flags);
		wake_up_interruptible(&espi_flash->wq);
	}

	regmap_write(espi_flash->map, ESPI_INT_STS, sts & ESPI_INT_STS_FLASH_BITS);

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_espi_flash_reset_isr(int irq, void *arg)
{
	struct aspeed_espi_flash *espi_flash = arg;
	struct aspeed_espi_flash_dma *dma = &espi_flash->dma;

	if (espi_flash->dma_mode) {
		regmap_write(espi_flash->map, ESPI_FLASH_TX_DMA, dma->tx_addr);
		regmap_write(espi_flash->map, ESPI_FLASH_RX_DMA, dma->rx_addr);
		regmap_update_bits(espi_flash->map, ESPI_CTRL,
				   ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN,
				   ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN);
	}

	return IRQ_HANDLED;
}

static int aspeed_espi_flash_dma_init(struct platform_device *pdev,
				      struct aspeed_espi_flash *espi_flash)
{
	struct device *dev = &pdev->dev;
	struct aspeed_espi_flash_dma *dma = &espi_flash->dma;

	dma->tx_virt = dma_alloc_coherent(dev, PAGE_SIZE,
					  &dma->tx_addr, GFP_KERNEL);
	if (!dma->tx_virt) {
		dev_err(dev, "cannot allocate DMA TX buffer\n");
		return -ENOMEM;
	}

	dma->rx_virt = dma_alloc_coherent(dev, PAGE_SIZE,
					  &dma->rx_addr, GFP_KERNEL);
	if (!dma->rx_virt) {
		dev_err(dev, "cannot allocate DMA RX buffer\n");
		return -ENOMEM;
	}

	regmap_write(espi_flash->map, ESPI_FLASH_TX_DMA, dma->tx_addr);
	regmap_write(espi_flash->map, ESPI_FLASH_RX_DMA, dma->rx_addr);
	regmap_update_bits(espi_flash->map, ESPI_CTRL,
			   ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN,
			   ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN);

	return 0;
}

static int aspeed_espi_flash_init(struct platform_device *pdev,
				  struct aspeed_espi_flash *espi_flash)
{
	int rc = 0;
	struct device *dev = &pdev->dev;

	regmap_update_bits(espi_flash->map, ESPI_CTRL,
			   ESPI_CTRL_FLASH_SW_MODE_MASK,
			   (espi_flash->safs_mode << ESPI_CTRL_FLASH_SW_MODE_SHIFT));

	if (espi_flash->dma_mode) {
		rc = aspeed_espi_flash_dma_init(pdev, espi_flash);
		if (rc)
			return rc;
	}

	rc = devm_request_irq(dev, espi_flash->irq, aspeed_espi_flash_isr,
			      0, DEVICE_NAME, espi_flash);
	if (rc) {
		dev_err(dev, "cannot request eSPI flash channel irq\n");
		return rc;
	}

	rc = devm_request_irq(dev, espi_flash->irq_reset, aspeed_espi_flash_reset_isr,
			      IRQF_SHARED, DEVICE_NAME, espi_flash);
	if (rc) {
		dev_err(dev, "cannot request eSPI channel reset irq\n");
		return rc;
	}

	return rc;
}

static int aspeed_espi_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device *dev = &pdev->dev;
	struct aspeed_espi_flash *espi_flash;

	espi_flash = devm_kzalloc(dev, sizeof(*espi_flash), GFP_KERNEL);
	if (!espi_flash)
		return -ENOMEM;

	espi_flash->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(espi_flash->map)) {
		dev_err(dev, "cannot get regmap\n");
		return PTR_ERR(espi_flash->map);
	}

	espi_flash->irq = platform_get_irq(pdev, 0);
	if (espi_flash->irq < 0)
		return espi_flash->irq;

	espi_flash->irq_reset = platform_get_irq(pdev, 1);
	if (espi_flash->irq_reset < 0)
		return espi_flash->irq_reset;

	espi_flash->version = (uint32_t)of_device_get_match_data(dev);

	if (of_property_read_bool(dev->of_node, "dma-mode"))
		espi_flash->dma_mode = 1;

	of_property_read_u32(dev->of_node, "safs-mode", &espi_flash->safs_mode);
	if (espi_flash->safs_mode >= SAFS_MODES) {
		dev_err(dev, "invalid SAFS mode\n");
		return -EINVAL;
	}

	rc = aspeed_espi_flash_init(pdev, espi_flash);
	if (rc)
		return rc;

	init_waitqueue_head(&espi_flash->wq);

	spin_lock_init(&espi_flash->lock);

	mutex_init(&espi_flash->put_tx_mtx);
	mutex_init(&espi_flash->get_rx_mtx);

	espi_flash->mdev.parent = dev;
	espi_flash->mdev.minor = MISC_DYNAMIC_MINOR;
	espi_flash->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", DEVICE_NAME);
	espi_flash->mdev.fops = &aspeed_espi_flash_fops;
	rc = misc_register(&espi_flash->mdev);
	if (rc) {
		dev_err(dev, "cannot register device\n");
		return rc;
	}

	dev_set_drvdata(dev, espi_flash);

	dev_info(dev, "module loaded\n");

	return 0;
}

static int aspeed_espi_flash_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_espi_flash *espi_flash = dev_get_drvdata(dev);
	struct aspeed_espi_flash_dma *dma;

	if (!espi_flash->dma_mode)
		return 0;

	dma = &espi_flash->dma;

	if (dma->tx_virt)
		dma_free_coherent(dev, PAGE_SIZE, dma->tx_virt, dma->tx_addr);

	if (dma->rx_virt)
		dma_free_coherent(dev, PAGE_SIZE, dma->rx_virt, dma->rx_addr);

	mutex_destroy(&espi_flash->put_tx_mtx);
	mutex_destroy(&espi_flash->get_rx_mtx);

	return 0;
}

static const struct of_device_id aspeed_espi_flash_match[] = {
	{ .compatible = "aspeed,ast2500-espi-flash", .data = (void *)ESPI_AST2500, },
	{ .compatible = "aspeed,ast2600-espi-flash", .data = (void *)ESPI_AST2600, },
	{ }
};

static struct platform_driver aspeed_espi_flash_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = aspeed_espi_flash_match,
	},
	.probe  = aspeed_espi_flash_probe,
	.remove = aspeed_espi_flash_remove,
};

module_platform_driver(aspeed_espi_flash_driver);

MODULE_AUTHOR("Chia-Wei Wang <chiawei_wang@aspeedtech.com>");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Control of Aspeed eSPI flash channel");
MODULE_LICENSE("GPL v2");
