// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 */
#include <linux/io.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <soc/aspeed/espi.h>
#include <uapi/linux/aspeed-espi.h>

#define DEVICE_NAME	"aspeed-espi-oob"

/* DMA descriptor is supported since AST2600 */
#define OOB_DMA_DESC_MAX_NUM	1024
#define OOB_DMA_UNLOCK		0x45535049

/* TX DMA descriptor type */
#define OOB_DMA_TX_DESC_CUST	0x04

struct oob_tx_dma_desc {
	uint32_t data_addr;
	uint8_t cyc;
	uint16_t tag : 4;
	uint16_t len : 12;
	uint8_t msg_type : 3;
	uint8_t raz0 : 1;
	uint8_t pec : 1;
	uint8_t int_en : 1;
	uint8_t pause : 1;
	uint8_t raz1 : 1;
	uint32_t raz2;
	uint32_t raz3;
} __attribute__((packed));

struct oob_rx_dma_desc {
	uint32_t data_addr;
	uint8_t cyc;
	uint16_t tag : 4;
	uint16_t len : 12;
	uint8_t raz : 7;
	uint8_t dirty : 1;
} __attribute__((packed));

struct aspeed_espi_oob_dma {
	uint32_t tx_desc_num;
	uint32_t rx_desc_num;

	struct oob_tx_dma_desc *tx_desc;
	dma_addr_t tx_desc_addr;

	struct oob_rx_dma_desc *rx_desc;
	dma_addr_t rx_desc_addr;

	void *tx_virt;
	dma_addr_t tx_addr;

	void *rx_virt;
	dma_addr_t rx_addr;
};

struct aspeed_espi_oob {
	struct regmap *map;

	int irq;
	int irq_reset;

	uint32_t dma_mode;
	struct aspeed_espi_oob_dma dma;

	uint32_t rx_ready;
	wait_queue_head_t wq;

	struct mutex get_rx_mtx;
	struct mutex put_tx_mtx;

	spinlock_t lock;

	struct miscdevice mdev;

	uint32_t version;
};

/* descriptor-based RX DMA handling */
static long aspeed_espi_oob_dma_desc_get_rx(struct file *fp,
					    struct aspeed_espi_ioc *ioc,
					    struct aspeed_espi_oob *espi_oob)
{
	int rc = 0;
	unsigned long flags;
	uint32_t reg;
	uint32_t rptr, wptr, sptr;
	uint8_t *pkt;
	uint32_t pkt_len;
	struct espi_comm_hdr *hdr;
	struct oob_rx_dma_desc *d;

	regmap_read(espi_oob->map, ESPI_OOB_RX_DMA_WS_PTR, &reg);
	wptr = (reg & ESPI_OOB_RX_DMA_WS_PTR_WP_MASK) >> ESPI_OOB_RX_DMA_WS_PTR_WP_SHIFT;
	sptr = (reg & ESPI_OOB_RX_DMA_WS_PTR_SP_MASK) >> ESPI_OOB_RX_DMA_WS_PTR_SP_SHIFT;

	regmap_read(espi_oob->map, ESPI_OOB_RX_DMA_RD_PTR, &rptr);

	d = &espi_oob->dma.rx_desc[sptr];

	if (!d->dirty)
		return -EFAULT;

	pkt_len = ((d->len)? d->len : 0x1000) + sizeof(struct espi_comm_hdr);

	if (ioc->pkt_len < pkt_len)
		return -EINVAL;

	pkt = vmalloc(pkt_len);
	if (!pkt)
		return -ENOMEM;

	hdr = (struct espi_comm_hdr *)pkt;
	hdr->cyc = d->cyc;
	hdr->tag = d->tag;
	hdr->len_h = d->len >> 8;
	hdr->len_l = d->len & 0xff;
	memcpy(hdr + 1, espi_oob->dma.rx_virt + (PAGE_SIZE * sptr), pkt_len - sizeof(*hdr));

	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/* make the descriptor available again */
	d->dirty = 0;
	sptr = (sptr + 1) % espi_oob->dma.rx_desc_num;
	wptr = (wptr + 1) % espi_oob->dma.rx_desc_num;

	reg = ((wptr << ESPI_OOB_RX_DMA_WS_PTR_WP_SHIFT) & ESPI_OOB_RX_DMA_WS_PTR_WP_MASK)
		| ((sptr << ESPI_OOB_RX_DMA_WS_PTR_SP_SHIFT) & ESPI_OOB_RX_DMA_WS_PTR_SP_MASK)
		| ESPI_OOB_RX_DMA_WS_PTR_RECV_EN;

	spin_lock_irqsave(&espi_oob->lock, flags);

	regmap_write(espi_oob->map, ESPI_OOB_RX_DMA_WS_PTR, reg);

	/* set ready flag base on the next RX descriptor */
	espi_oob->rx_ready = espi_oob->dma.rx_desc[sptr].dirty;

	spin_unlock_irqrestore(&espi_oob->lock, flags);

free_n_out:
	vfree(pkt);

	return rc;
}

static long aspeed_espi_oob_get_rx(struct file *fp,
				   struct aspeed_espi_ioc *ioc,
				   struct aspeed_espi_oob *espi_oob)
{
	int i, rc = 0;
	unsigned long flags;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint8_t *pkt;
	uint32_t pkt_len;
	struct espi_comm_hdr *hdr;

	if (fp->f_flags & O_NONBLOCK) {
		if (mutex_trylock(&espi_oob->get_rx_mtx))
			return -EBUSY;

		if (!espi_oob->rx_ready) {
			rc = -ENODATA;
			goto unlock_mtx_n_out;
		}
	}
	else {
		mutex_lock(&espi_oob->get_rx_mtx);

		if (!espi_oob->rx_ready) {
			rc = wait_event_interruptible(espi_oob->wq,
						      espi_oob->rx_ready);
			if (rc == -ERESTARTSYS) {
				rc = -EINTR;
				goto unlock_mtx_n_out;
			}
		}
	}

	if (espi_oob->dma_mode && espi_oob->version != ESPI_AST2500) {
		rc = aspeed_espi_oob_dma_desc_get_rx(fp, ioc, espi_oob);
		goto unlock_mtx_n_out;
	}

	/* common header (i.e. cycle type, tag, and length) is taken by HW */
	regmap_read(espi_oob->map, ESPI_OOB_RX_CTRL, &reg);
	cyc = (reg & ESPI_OOB_RX_CTRL_CYC_MASK) >> ESPI_OOB_RX_CTRL_CYC_SHIFT;
	tag = (reg & ESPI_OOB_RX_CTRL_TAG_MASK) >> ESPI_OOB_RX_CTRL_TAG_SHIFT;
	len = (reg & ESPI_OOB_RX_CTRL_LEN_MASK) >> ESPI_OOB_RX_CTRL_LEN_SHIFT;

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	pkt_len = ((len)? len : ESPI_PLD_LEN_MAX) + sizeof(struct espi_comm_hdr);

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

	if (espi_oob->dma_mode) {
		memcpy(hdr + 1, espi_oob->dma.rx_virt,
		       pkt_len - sizeof(*hdr));
	}
	else {
		for (i = sizeof(*hdr); i < pkt_len; ++i) {
			regmap_read(espi_oob->map,
				    ESPI_OOB_RX_PORT, &reg);
			pkt[i] = reg & 0xff;
		}
	}

	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	spin_lock_irqsave(&espi_oob->lock, flags);

	regmap_write_bits(espi_oob->map, ESPI_OOB_RX_CTRL,
			  ESPI_OOB_RX_CTRL_PEND_SERV,
			  ESPI_OOB_RX_CTRL_PEND_SERV);

	espi_oob->rx_ready = 0;

	spin_unlock_irqrestore(&espi_oob->lock, flags);

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&espi_oob->get_rx_mtx);

	return rc;
}

/* descriptor-based TX DMA handling */
static long aspeed_espi_oob_dma_desc_put_tx(struct file *fp,
					    struct aspeed_espi_ioc *ioc,
					    struct aspeed_espi_oob *espi_oob)
{
	int rc = 0;
	uint32_t rptr, wptr;
	uint8_t *pkt;
	struct espi_comm_hdr *hdr;
	struct oob_tx_dma_desc *d;

	pkt = vzalloc(ioc->pkt_len);
	if (!pkt)
		return -ENOMEM;

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/* kick HW to reflect the up-to-date read/write pointer */
	regmap_write(espi_oob->map, ESPI_OOB_TX_DMA_RD_PTR,
		     ESPI_OOB_TX_DMA_RD_PTR_UPDATE);

	regmap_read(espi_oob->map, ESPI_OOB_TX_DMA_RD_PTR, &rptr);
	regmap_read(espi_oob->map, ESPI_OOB_TX_DMA_WR_PTR, &wptr);

	if (((wptr + 1) % espi_oob->dma.tx_desc_num) == rptr)
		return -EBUSY;

	d = &espi_oob->dma.tx_desc[wptr];
	d->cyc = hdr->cyc;
	d->tag = hdr->tag;
	d->len = (hdr->len_h << 8) | (hdr->len_l & 0xff);
	d->msg_type = OOB_DMA_TX_DESC_CUST;

	memcpy(espi_oob->dma.tx_virt + (PAGE_SIZE * wptr), hdr + 1,
	       ioc->pkt_len - sizeof(*hdr));

	dma_wmb();

	wptr = (wptr + 1) % espi_oob->dma.tx_desc_num;
	wptr |= ESPI_OOB_TX_DMA_WR_PTR_SEND_EN;
	regmap_write(espi_oob->map, ESPI_OOB_TX_DMA_WR_PTR, wptr);

free_n_out:
	vfree(pkt);

	return rc;
}

static long aspeed_espi_oob_put_tx(struct file *fp,
				   struct aspeed_espi_ioc *ioc,
				   struct aspeed_espi_oob *espi_oob)
{
	int i, rc = 0;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint8_t *pkt;
	struct espi_comm_hdr *hdr;

	if (!mutex_trylock(&espi_oob->put_tx_mtx))
		return -EBUSY;

	if (espi_oob->dma_mode && espi_oob->version != ESPI_AST2500) {
		rc = aspeed_espi_oob_dma_desc_put_tx(fp, ioc, espi_oob);
		goto unlock_mtx_n_out;
	}

	regmap_read(espi_oob->map, ESPI_OOB_TX_CTRL, &reg);
	if (reg & ESPI_OOB_TX_CTRL_TRIGGER) {
		rc = -EBUSY;
		goto unlock_mtx_n_out;
	}

	if (ioc->pkt_len > ESPI_PKT_LEN_MAX) {
		rc = -EINVAL;
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
	if (espi_oob->dma_mode) {
		memcpy(espi_oob->dma.tx_virt, hdr + 1,
		       ioc->pkt_len - sizeof(*hdr));
		dma_wmb();
	}
	else {
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			regmap_write(espi_oob->map,
				     ESPI_OOB_TX_PORT, pkt[i]);
	}

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = ((cyc << ESPI_OOB_TX_CTRL_CYC_SHIFT) & ESPI_OOB_TX_CTRL_CYC_MASK)
		| ((tag << ESPI_OOB_TX_CTRL_TAG_SHIFT) & ESPI_OOB_TX_CTRL_TAG_MASK)
		| ((len << ESPI_OOB_TX_CTRL_LEN_SHIFT) & ESPI_OOB_TX_CTRL_LEN_MASK)
		| ESPI_OOB_TX_CTRL_TRIGGER;

	regmap_write(espi_oob->map, ESPI_OOB_TX_CTRL, reg);

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&espi_oob->put_tx_mtx);

	return rc;
}

static long aspeed_espi_oob_ioctl(struct file *fp, unsigned int cmd,
				    unsigned long arg)
{
	struct aspeed_espi_ioc ioc;
	struct aspeed_espi_oob *espi_oob = container_of(
			fp->private_data,
			struct aspeed_espi_oob,
			mdev);

	if (copy_from_user(&ioc, (void __user *)arg, sizeof(ioc)))
	    return -EFAULT;

	if (ioc.pkt_len > ESPI_PKT_LEN_MAX)
		return -EINVAL;

	switch (cmd) {
	case ASPEED_ESPI_OOB_GET_RX:
		return aspeed_espi_oob_get_rx(fp, &ioc, espi_oob);
	case ASPEED_ESPI_OOB_PUT_TX:
		return aspeed_espi_oob_put_tx(fp, &ioc, espi_oob);
	};

	return -EINVAL;
}

static const struct file_operations aspeed_espi_oob_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aspeed_espi_oob_ioctl,
};

static irqreturn_t aspeed_espi_oob_isr(int irq, void *arg)
{
	int i;
	uint32_t sts;
	unsigned long flags;
	struct aspeed_espi_oob *espi_oob = arg;
	struct aspeed_espi_oob_dma *dma = &espi_oob->dma;

	regmap_read(espi_oob->map, ESPI_INT_STS, &sts);

	if (!(sts & (ESPI_INT_STS_HW_RST_DEASSERT | ESPI_INT_STS_OOB_BITS)))
		return IRQ_NONE;

	if (sts & ESPI_INT_STS_HW_RST_DEASSERT) {
		if(espi_oob->dma_mode) {
			if (espi_oob->version == ESPI_AST2500) {
				regmap_write(espi_oob->map, ESPI_OOB_TX_DMA, dma->tx_addr);
				regmap_write(espi_oob->map, ESPI_OOB_RX_DMA, dma->rx_addr);
			}
			else {
				regmap_update_bits(espi_oob->map, ESPI_CTRL,
						   ESPI_CTRL_OOB_TX_DMA_EN | ESPI_CTRL_OOB_RX_DMA_EN,
						   ESPI_CTRL_OOB_TX_DMA_EN | ESPI_CTRL_OOB_RX_DMA_EN);

				for (i = 0; i < dma->rx_desc_num; ++i)
					dma->rx_desc[i].dirty = 0;

				regmap_write(espi_oob->map, ESPI_OOB_TX_DMA, dma->tx_desc_addr);
				regmap_write(espi_oob->map, ESPI_OOB_TX_DMA_RB_SIZE, dma->tx_desc_num);
				regmap_write(espi_oob->map, ESPI_OOB_TX_DMA_RD_PTR, OOB_DMA_UNLOCK);
				regmap_write(espi_oob->map, ESPI_OOB_TX_DMA_WR_PTR, 0);

				regmap_write(espi_oob->map, ESPI_OOB_RX_DMA, dma->rx_desc_addr);
				regmap_write(espi_oob->map, ESPI_OOB_RX_DMA_RB_SIZE, dma->rx_desc_num);
				regmap_write(espi_oob->map, ESPI_OOB_RX_DMA_RD_PTR, OOB_DMA_UNLOCK);
				regmap_write(espi_oob->map, ESPI_OOB_RX_DMA_WS_PTR, 0);

				regmap_update_bits(espi_oob->map, ESPI_OOB_RX_DMA_WS_PTR,
						   ESPI_OOB_RX_DMA_WS_PTR_RECV_EN,
						   ESPI_OOB_RX_DMA_WS_PTR_RECV_EN);
		    }
		}

		regmap_update_bits(espi_oob->map, ESPI_CTRL,
				   ESPI_CTRL_OOB_FW_RDY, ESPI_CTRL_OOB_FW_RDY);
	}

	if (sts & ESPI_INT_STS_OOB_RX_CMPLT) {
		spin_lock_irqsave(&espi_oob->lock, flags);
		espi_oob->rx_ready = 1;
		spin_unlock_irqrestore(&espi_oob->lock, flags);

		wake_up_interruptible(&espi_oob->wq);
	}

	regmap_write(espi_oob->map, ESPI_INT_STS, sts & ESPI_INT_STS_OOB_BITS);

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_espi_oob_reset_isr(int irq, void *arg)
{
	struct aspeed_espi_oob *espi_oob = arg;

	regmap_update_bits(espi_oob->map, ESPI_CTRL,
			   ESPI_CTRL_OOB_FW_RDY | ESPI_CTRL_OOB_RX_SW_RST, 0);

	if(espi_oob->dma_mode)
		regmap_update_bits(espi_oob->map, ESPI_CTRL,
				   ESPI_CTRL_OOB_TX_DMA_EN | ESPI_CTRL_OOB_RX_DMA_EN, 0);
	else
		regmap_write(espi_oob->map, ESPI_OOB_RX_CTRL, ESPI_OOB_RX_CTRL_PEND_SERV);

	regmap_update_bits(espi_oob->map, ESPI_CTRL,
			   ESPI_CTRL_OOB_RX_SW_RST, ESPI_CTRL_OOB_RX_SW_RST);

	return IRQ_HANDLED;
}

static int aspeed_espi_oob_dma_init(struct platform_device *pdev,
				struct aspeed_espi_oob *espi_oob)
{
	int i;
	struct device *dev = &pdev->dev;
	struct aspeed_espi_oob_dma *dma = &espi_oob->dma;

	if (espi_oob->version != ESPI_AST2500) {
		dma->tx_desc = dma_alloc_coherent(dev, sizeof(*dma->tx_desc) * dma->tx_desc_num,
						  &dma->tx_desc_addr, GFP_KERNEL);
		if (!dma->tx_desc) {
			dev_err(dev, "cannot allocate DMA TX descriptor\n");
			return -ENOMEM;
		}

		dma->rx_desc = dma_alloc_coherent(dev, sizeof(*dma->rx_desc) * dma->rx_desc_num,
						  &dma->rx_desc_addr, GFP_KERNEL);
		if (!dma->rx_desc) {
			dev_err(dev, "cannot allocate DMA RX descriptor\n");
			return -ENOMEM;
		}
	}

	dma->tx_virt = dma_alloc_coherent(dev, PAGE_SIZE * dma->tx_desc_num,
					  &dma->tx_addr, GFP_KERNEL);
	if (!dma->tx_virt) {
		dev_err(dev, "cannot allocate DMA TX buffer\n");
		return -ENOMEM;
	}

	dma->rx_virt = dma_alloc_coherent(dev, PAGE_SIZE * dma->rx_desc_num,
					  &dma->rx_addr, GFP_KERNEL);
	if (!dma->rx_virt) {
		dev_err(dev, "cannot allocate DMA RX buffer\n");
		return -ENOMEM;
	}

	if (espi_oob->version == ESPI_AST2500) {
		regmap_write(espi_oob->map, ESPI_OOB_TX_DMA, dma->tx_addr);
		regmap_write(espi_oob->map, ESPI_OOB_RX_DMA, dma->rx_addr);
	}
	else {
		for (i = 0; i < dma->tx_desc_num; ++i)
			dma->tx_desc[i].data_addr = dma->tx_addr + (i * PAGE_SIZE);

		for (i = 0; i < dma->rx_desc_num; ++i) {
			dma->rx_desc[i].data_addr = dma->rx_addr + (i * PAGE_SIZE);
			dma->rx_desc[i].dirty = 0;
		}

		regmap_write(espi_oob->map, ESPI_OOB_TX_DMA, dma->tx_desc_addr);
		regmap_write(espi_oob->map, ESPI_OOB_TX_DMA_RB_SIZE, dma->tx_desc_num);

		regmap_write(espi_oob->map, ESPI_OOB_RX_DMA, dma->rx_desc_addr);
		regmap_write(espi_oob->map, ESPI_OOB_RX_DMA_RB_SIZE, dma->rx_desc_num);
		regmap_update_bits(espi_oob->map, ESPI_OOB_RX_DMA_WS_PTR,
				   ESPI_OOB_RX_DMA_WS_PTR_RECV_EN,
				   ESPI_OOB_RX_DMA_WS_PTR_RECV_EN);
	}

	regmap_update_bits(espi_oob->map, ESPI_CTRL,
			   ESPI_CTRL_OOB_TX_DMA_EN | ESPI_CTRL_OOB_RX_DMA_EN,
			   ESPI_CTRL_OOB_TX_DMA_EN | ESPI_CTRL_OOB_RX_DMA_EN);

	return 0;
}

static int aspeed_espi_oob_init(struct platform_device *pdev,
				struct aspeed_espi_oob *espi_oob)
{
	int rc = 0;
	struct device *dev = &pdev->dev;

	if (espi_oob->dma_mode) {
		rc = aspeed_espi_oob_dma_init(pdev, espi_oob);
		if (rc)
			return rc;
	}

	rc = devm_request_irq(dev, espi_oob->irq, aspeed_espi_oob_isr,
			      0, DEVICE_NAME, espi_oob);
	if (rc) {
		dev_err(dev, "cannot request eSPI out-of-band channel irq\n");
		return rc;
	}

	rc = devm_request_irq(dev, espi_oob->irq_reset, aspeed_espi_oob_reset_isr,
			      IRQF_SHARED, DEVICE_NAME, espi_oob);
	if (rc) {
		dev_err(dev, "cannot request eSPI channel reset irq\n");
		return rc;
	}

	regmap_update_bits(espi_oob->map, ESPI_CTRL,
			   ESPI_CTRL_OOB_FW_RDY,
			   ESPI_CTRL_OOB_FW_RDY);

	return 0;
}

static int aspeed_espi_oob_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device *dev = &pdev->dev;
	struct aspeed_espi_oob *espi_oob;
	struct aspeed_espi_oob_dma *dma;

	espi_oob = devm_kzalloc(dev, sizeof(*espi_oob), GFP_KERNEL);
	if (!espi_oob)
		return -ENOMEM;

	espi_oob->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(espi_oob->map)) {
		dev_err(dev, "cannot get regmap\n");
		return PTR_ERR(espi_oob->map);
	}

	espi_oob->irq = platform_get_irq(pdev, 0);
	if (espi_oob->irq < 0)
		return espi_oob->irq;

	espi_oob->irq_reset = platform_get_irq(pdev, 1);
	if (espi_oob->irq_reset < 0)
		return espi_oob->irq_reset;

	espi_oob->version = (uint32_t)of_device_get_match_data(dev);

	if (of_property_read_bool(dev->of_node, "dma-mode"))
		espi_oob->dma_mode = 1;

	if (espi_oob->dma_mode) {
		dma = &espi_oob->dma;

		/* Descriptor based OOB DMA is supported since AST2600 */
		if (espi_oob->version != ESPI_AST2500) {
			of_property_read_u32(dev->of_node, "dma-tx-desc-num",
					     &dma->tx_desc_num);
			of_property_read_u32(dev->of_node, "dma-rx-desc-num",
					     &dma->rx_desc_num);

			if (!dma->tx_desc_num || !dma->rx_desc_num) {
				dev_err(dev, "invalid zero number of DMA channels\n");
				return -EINVAL;
			}

			if (dma->tx_desc_num >= OOB_DMA_DESC_MAX_NUM
			    || dma->rx_desc_num >= OOB_DMA_DESC_MAX_NUM) {
				dev_err(dev, "too many number of DMA channels\n");
				return -EINVAL;
			}
		}

		/*
		 * DMA descriptors are consumed in the circular
		 * queue paradigm. Therefore, one dummy slot is
		 * reserved to detect the full condition.
		 *
		 * For AST2500 without DMA descriptors supported,
		 * the number of the queue slot should be 1 here.
		 */
		dma->tx_desc_num += 1;
		dma->rx_desc_num += 1;
	}

	rc = aspeed_espi_oob_init(pdev, espi_oob);
	if (rc)
		return rc;

	init_waitqueue_head(&espi_oob->wq);

	spin_lock_init(&espi_oob->lock);

	mutex_init(&espi_oob->put_tx_mtx);
	mutex_init(&espi_oob->get_rx_mtx);

	espi_oob->mdev.parent = dev;
	espi_oob->mdev.minor = MISC_DYNAMIC_MINOR;
	espi_oob->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", DEVICE_NAME);
	espi_oob->mdev.fops = &aspeed_espi_oob_fops;
	rc = misc_register(&espi_oob->mdev);
	if (rc) {
		dev_err(dev, "cannot register device\n");
		return rc;
	}

	dev_set_drvdata(dev, espi_oob);

	dev_info(dev, "module loaded\n");

	return 0;
}

static int aspeed_espi_oob_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_espi_oob *espi_oob = dev_get_drvdata(dev);
	struct aspeed_espi_oob_dma *dma;

	if (!espi_oob->dma_mode)
		return 0;

	dma = &espi_oob->dma;

	if (dma->tx_desc)
		dma_free_coherent(dev, sizeof(*dma->tx_desc) * dma->tx_desc_num,
				  dma->tx_desc, dma->tx_desc_addr);

	if (dma->rx_desc)
		dma_free_coherent(dev, sizeof(*dma->rx_desc) * dma->rx_desc_num,
				  dma->rx_desc, dma->rx_desc_addr);

	if (dma->tx_virt)
		dma_free_coherent(dev, PAGE_SIZE * dma->tx_desc_num,
				  dma->tx_virt, dma->tx_addr);

	if (dma->rx_virt)
		dma_free_coherent(dev, PAGE_SIZE * dma->rx_desc_num,
				  dma->rx_virt, dma->rx_addr);

	mutex_destroy(&espi_oob->put_tx_mtx);
	mutex_destroy(&espi_oob->get_rx_mtx);

	return 0;
}

static const struct of_device_id aspeed_espi_oob_match[] = {
	{ .compatible = "aspeed,ast2500-espi-oob", .data = (void *)ESPI_AST2500, },
	{ .compatible = "aspeed,ast2600-espi-oob", .data = (void *)ESPI_AST2600, },
	{ }
};

static struct platform_driver aspeed_espi_oob_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = aspeed_espi_oob_match,
	},
	.probe  = aspeed_espi_oob_probe,
	.remove = aspeed_espi_oob_remove,
};

module_platform_driver(aspeed_espi_oob_driver);

MODULE_AUTHOR("Chia-Wei Wang <chiawei_wang@aspeedtech.com>");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Control of Aspeed eSPI out-of-band channel");
MODULE_LICENSE("GPL v2");
