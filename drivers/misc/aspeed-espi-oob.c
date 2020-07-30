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
#include <linux/module.h>

#include "regs-aspeed-espi.h"

#define DEVICE_NAME     "espi-oob"

#define OOB_TXCMD_DESC_NUM	2
#define OOB_RXCMD_DESC_NUM	8
#define OOB_TX_BUF_NUM		2
#define OOB_RX_BUF_NUM		8
#define OOB_BUFF_SIZE		256
#define ESPI_OOB_MESSAGE			0x21

struct aspeed_oob_rx_cmd {
	dma_addr_t dma_addr;
	u32	cmd;
};

struct aspeed_oob_tx_cmd {
	dma_addr_t dma_addr;
	u32	cmd0;
	u32	cmd1;
	u32	cmd2;
};

struct aspeed_espi_oob {
	struct regmap *map;
	struct miscdevice miscdev;	
	struct bin_attribute	bin;
	struct kernfs_node	*kn;	
	int 				irq;					//LPC IRQ number	
	int					rest_irq;
	int		espi_version;
	int						dma_mode;		/* o:disable , 1:enable */	

	dma_addr_t oob_tx_cmd_dma;
	struct aspeed_oob_tx_cmd 	*oob_tx_cmd;
	dma_addr_t oob_tx_buff_dma;
	u8 *oob_tx_buff;
	u8	oob_tx_idx;

	dma_addr_t oob_rx_cmd_dma;
	struct aspeed_oob_rx_cmd 	*oob_rx_cmd;
	dma_addr_t oob_rx_buff_dma;
	u8 *oob_rx_buff;
	u8	oob_rx_full;		//for PIO mode , it will be length 

};

static ssize_t aspeed_oob_channel_rx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	u32 service_pt = 0;
	u32 write_pt = 0;
	struct aspeed_espi_oob *espi_oob = dev_get_drvdata(container_of(kobj, struct device, kobj));

	if((espi_oob->espi_version == ESPI_AST2600) && (espi_oob->dma_mode)) {
		regmap_read(espi_oob->map, ASPEED_ESPI_OOB_RX_WRITE_PT, &write_pt);
		
		service_pt = (write_pt >> 16) & 0x3ff;
		write_pt &= 0x3ff;
//		printk("write_pt %x , service_pt %x cmd %x, dma addr %x \n", write_pt, service_pt, espi_oob->oob_rx_cmd[service_pt].cmd, espi_oob->oob_rx_cmd[service_pt].dma_addr);
		if(espi_oob->oob_rx_cmd[service_pt].cmd & BIT(31)) {
			u8	*rx_buff = &espi_oob->oob_rx_buff[service_pt * OOB_BUFF_SIZE];
			u32 rx_len = (espi_oob->oob_rx_cmd[service_pt].cmd >> 12) & 0xfff;
			memcpy(buf, rx_buff, rx_len);
			count = rx_len;
			espi_oob->oob_rx_cmd[service_pt].cmd &= ~BIT(31);
			service_pt++;
			service_pt %= OOB_RX_BUF_NUM;
			write_pt++;
			write_pt %= OOB_RX_BUF_NUM;
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_WRITE_PT, BIT(31) | (service_pt << 16) | (write_pt));
		} else {
//			printk("rx empty \n");
			return 0;			
		}
	} else {
		if(espi_oob->oob_rx_buff) {
			count = espi_oob->oob_rx_full;
			memcpy(buf, espi_oob->oob_rx_buff, count);
			espi_oob->oob_rx_full = 0;
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_CTRL, ESPI_TRIGGER_PACKAGE);
		} else {
			count = 0;
		}
	}
	return count;
}

/*	
	16 byte header : 
	generic SMBus : 0x1
		buf[0] = tag, 		 buf[1] = type 0x1, 	buf[2] = pec enable, 	buf[3] = oob_length
		buf[4] = dest_s_addr buf[5] : cmd code, 	buf[6] : byte count buf[7 ~ 15] = reserved 

	mctp : 0x2
		buf[0] = tag, 		 buf[1] = type 0x2, 	buf[2] = pec enable, 	buf[3] = oob_length
		buf[4] = dest_s_addr buf[5] : cmd code, 	buf[6] : byte count 	buf[7] : src slave addr 
		buf[8] : hdr ver.	 buf[9] = dest epid,    buf[10] = src epid		buf[11] : msg tag,		
		buf[12 ~ 15] : reserved

	Customize type : 0x4
		buf[0] = tag,		 buf[1] = type 0x4, 	buf[2] = pec enable,	buf[3] = oob_length
		buf[4 ~ 15] : reserved 

	DATA begin from buf[16]		
*/
static ssize_t aspeed_oob_channel_tx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	int i = 0;
	u8 *tx_buff = 0;
	u32 tx_rd_pt, tx_ctrl, ch_sts;
	struct aspeed_espi_oob *espi_oob = dev_get_drvdata(container_of(kobj, struct device, kobj));
	regmap_read(espi_oob->map, ASPEED_ESPI_CH2CAP_CONFIG, &ch_sts);
	if(!(ch_sts & BIT(0))) {
		printk("channel not enable \n");
		return -1;
	}
	
	if((espi_oob->espi_version == ESPI_AST2600) && (espi_oob->dma_mode)) {
		//ast2600 tx dma
		regmap_write(espi_oob->map, ASPEED_ESPI_OOB_TX_READ_PT, BIT(31));
		regmap_read(espi_oob->map, ASPEED_ESPI_OOB_TX_READ_PT, &tx_rd_pt);
		printk("tx_rd_pt %d , espi_oob->oob_tx_idx %d \n", tx_rd_pt, espi_oob->oob_tx_idx);
		if(((espi_oob->oob_tx_idx + 1) % OOB_TX_BUF_NUM) == tx_rd_pt) {
			printk("TX full \n");
			return 0;
		} else {
			if(count > OOB_BUFF_SIZE)
				return 0;
			
			tx_buff = &espi_oob->oob_tx_buff[(espi_oob->oob_tx_idx) * OOB_BUFF_SIZE];
			memcpy(tx_buff, buf, count);
			switch(tx_buff[1]) {
				case 0x04:	//customize
					break;
				case 0x01:	//generic SMBus 
					espi_oob->oob_tx_cmd[espi_oob->oob_tx_idx].cmd1 = (tx_buff[5] << 8) | (tx_buff[4] << 1);
					break;
				case 0x02:	//mctp
					espi_oob->oob_tx_cmd[espi_oob->oob_tx_idx].cmd1 = (tx_buff[8] << 24) | (tx_buff[7] << 17) | BIT(16) | (tx_buff[5] << 8) | (tx_buff[4] << 1);
					espi_oob->oob_tx_cmd[espi_oob->oob_tx_idx].cmd2 = (tx_buff[11] << 16) | (tx_buff[10] << 8) | tx_buff[9];
					break;
				default:
					return 0;
					break;
			}

			espi_oob->oob_tx_cmd[espi_oob->oob_tx_idx].dma_addr = espi_oob->oob_tx_buff_dma + ((espi_oob->oob_tx_idx) * OOB_BUFF_SIZE) + 0x10;
			printk("dma addr %x , size = %d \n", espi_oob->oob_tx_cmd[espi_oob->oob_tx_idx].dma_addr, count);

			for(i = 0; i < count; i++)
				printk("[%d] : %x \n", i, tx_buff[i]);

			espi_oob->oob_tx_cmd[espi_oob->oob_tx_idx].cmd0 = (tx_buff[2] << 28) | (tx_buff[1] << 24) | (tx_buff[3] << 12) | (tx_buff[0] << 8) | ESPI_OOB_MESSAGE;
			printk("cmd0 %x \n", espi_oob->oob_tx_cmd[espi_oob->oob_tx_idx].cmd0);
			
			espi_oob->oob_tx_idx++;
			espi_oob->oob_tx_idx %= OOB_TX_BUF_NUM;
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_TX_WRITE_PT, espi_oob->oob_tx_idx | BIT(31));
		}
	} else {
		regmap_read(espi_oob->map, ASPEED_ESPI_OOB_TX_CTRL, &tx_ctrl);
		if(tx_ctrl & ESPI_TRIGGER_PACKAGE)
			return 0;
		else {
			for (i = 0; i < buf[0]; i++) {
//				printk("[%d] : %x \n", i, buf[i]);
				regmap_write(espi_oob->map, ASPEED_ESPI_OOB_TX_DATA, buf[16 + i]);
			}
			//PIO mode count is data payload, if package eanble pec, it need calulate by sw pec include in packet. 
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_TX_CTRL, ESPI_TRIGGER_PACKAGE |  (buf[3] << 12) | (buf[0] << 8) | ESPI_OOB_MESSAGE);
		}
	}
	return count;
}

static void
aspeed_espi_oob_rx(struct aspeed_espi_oob *espi_oob)
{
	int i = 0;
	u32 ctrl = 0;
	u32 rx_buf;

	if ((espi_oob->dma_mode) && (espi_oob->espi_version == ESPI_AST2600)) {

	} else {
		//old ast2500 or PIO mode
		regmap_read(espi_oob->map, ASPEED_ESPI_OOB_RX_CTRL, &ctrl);
//		printk("cycle type = %x , tag = %x, len = %d byte \n", ESPI_GET_CYCLE_TYPE(ctrl), ESPI_GET_TAG(ctrl), ESPI_GET_LEN(ctrl));
		espi_oob->oob_rx_full = ESPI_GET_LEN(ctrl);
		
		if(!espi_oob->dma_mode) {
			for (i = 0; i < ESPI_GET_LEN(ctrl); i++) {
				regmap_read(espi_oob->map, ASPEED_ESPI_OOB_RX_DATA, &rx_buf);
				espi_oob->oob_rx_buff[i] = rx_buf;
			}
		}		
	}
}

static irqreturn_t aspeed_espi_oob_reset_irq(int irq, void *arg)
{
	struct aspeed_espi_oob *espi_oob = arg;

	if(espi_oob->espi_version == ESPI_AST2600)
		regmap_update_bits(espi_oob->map, ASPEED_ESPI_CTRL, ESPI_CTRL_OOB_FW_RDY | BIT(28), 0); 

	if(espi_oob->dma_mode) {
		regmap_update_bits(espi_oob->map, ASPEED_ESPI_CTRL, GENMASK(21, 20), 0);
	} else 
		regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_CTRL, ESPI_TRIGGER_PACKAGE);

	if(espi_oob->espi_version == ESPI_AST2600)
		regmap_update_bits(espi_oob->map, ASPEED_ESPI_CTRL, BIT(28), BIT(28)); 

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_espi_oob_irq(int irq, void *arg)
{
	u32 sts, oob_isr;
	int i;
	struct aspeed_espi_oob *espi_oob = arg;

	regmap_read(espi_oob->map, ASPEED_ESPI_ISR, &sts);
//	printk("aspeed_espi_oob_irq %x\n", sts);


	if (sts & ESPI_ISR_HW_RESET) {
		if(espi_oob->dma_mode) {
			regmap_update_bits(espi_oob->map, ASPEED_ESPI_CTRL, GENMASK(21, 20), ESPI_CTRL_OOB_RX_DMA | ESPI_CTRL_OOB_TX_DMA);
			//init rx cmd desc
			for(i = 0; i < OOB_RXCMD_DESC_NUM; i++) {
				espi_oob->oob_rx_cmd[i].cmd = 0;
//				espi_oob->oob_rx_cmd[i].dma_addr = espi_oob->oob_rx_buff + (i * OOB_BUFF_SIZE);
			}
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_DMA, espi_oob->oob_rx_cmd_dma);
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_RING_SIZE, OOB_RXCMD_DESC_NUM);
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_WRITE_PT, BIT(31));

			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_TX_DMA, espi_oob->oob_tx_cmd_dma);
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_TX_RING_SIZE, OOB_TXCMD_DESC_NUM);

			espi_oob->oob_tx_idx = 0;
			espi_oob->oob_rx_buff = espi_oob->oob_tx_buff + (OOB_BUFF_SIZE * OOB_TX_BUF_NUM);
			espi_oob->oob_rx_buff_dma = espi_oob->oob_tx_buff_dma + (OOB_BUFF_SIZE * OOB_TX_BUF_NUM);
		}
		regmap_update_bits(espi_oob->map, ASPEED_ESPI_CTRL, ESPI_CTRL_OOB_FW_RDY, ESPI_CTRL_OOB_FW_RDY); 
	}

	oob_isr = sts & (ESPI_ISR_OOB_TX_ERR | ESPI_ISR_OOB_TX_ABORT | ESPI_ISR_OOB_RX_ABORT | ESPI_ISR_OOB_TX_COMP | ESPI_ISR_OOB_RX_COMP);
	if (oob_isr) {
		if (sts & ESPI_ISR_OOB_TX_ERR) {
			printk("ESPI_ISR_OOB_TX_ERR \n");
		}

		if (sts & ESPI_ISR_OOB_TX_ABORT) {
			printk("ESPI_ISR_OOB_TX_ABORT \n");
		}

		if (sts & ESPI_ISR_OOB_RX_ABORT) {
			printk("ESPI_ISR_OOB_RX_ABORT");
		}

		if (sts & ESPI_ISR_OOB_TX_COMP) {
			printk("ESPI_ISR_OOB_TX_COMP \n");
		}

		if (sts & ESPI_ISR_OOB_RX_COMP) {
			printk("ESPI_ISR_OOB_RX_COMP \n");
			aspeed_espi_oob_rx(espi_oob);
		}
		regmap_write(espi_oob->map, ASPEED_ESPI_ISR, oob_isr);
		return IRQ_HANDLED;		
	} else 
		return IRQ_NONE;

}

static const struct of_device_id aspeed_espi_oob_match[] = {
	{ .compatible = "aspeed,ast2600-espi-oob", .data = (void *) ESPI_AST2600, },
	{ .compatible = "aspeed,ast2500-espi-oob", .data = (void *) ESPI_AST2500, },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_espi_oob_match);


static int aspeed_espi_oob_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_espi_oob *espi_oob;
	const struct of_device_id *dev_id;	
	int rc, i;

	espi_oob = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_espi_oob), GFP_KERNEL);
	if (!espi_oob)
		return -ENOMEM;

	dev_id = of_match_device(aspeed_espi_oob_match, &pdev->dev);
	if (!dev_id)
		return -EINVAL;

	espi_oob->espi_version = (unsigned long)dev_id->data;

	if (of_property_read_bool(pdev->dev.of_node, "dma-mode"))
		espi_oob->dma_mode = 1;

	espi_oob->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(espi_oob->map)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	espi_oob->irq = platform_get_irq(pdev, 0);
	if (espi_oob->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_oob->irq;
	}

	sysfs_bin_attr_init(&espi_oob->bin);
	espi_oob->bin.attr.name = DEVICE_NAME;
	espi_oob->bin.attr.mode = S_IRUSR | S_IWUSR;
	espi_oob->bin.read = aspeed_oob_channel_rx;
	espi_oob->bin.write = aspeed_oob_channel_tx;
	espi_oob->bin.size = OOB_BUFF_SIZE;

	rc = sysfs_create_bin_file(&pdev->dev.kobj, &espi_oob->bin);
	if (rc) {
		printk("error for bin file ");
		return rc;
	}

	espi_oob->kn = kernfs_find_and_get(dev->kobj.sd, espi_oob->bin.attr.name);
	if (!espi_oob->kn) {
		sysfs_remove_bin_file(&dev->kobj, &espi_oob->bin);
		return -EFAULT;
	}
	
	dev_set_drvdata(dev, espi_oob);
	
	//aspeed_oob_channel_init
	if(espi_oob->dma_mode) {
		if(espi_oob->espi_version == ESPI_AST2600) {
			regmap_update_bits(espi_oob->map, ASPEED_ESPI_CTRL, GENMASK(21, 20), ESPI_CTRL_OOB_RX_DMA | ESPI_CTRL_OOB_TX_DMA);

			//tx cmd desc
			espi_oob->oob_tx_cmd = dma_alloc_coherent(NULL,
										  ((OOB_TX_BUF_NUM + OOB_RX_BUF_NUM) * OOB_BUFF_SIZE) + 
										  (sizeof(struct aspeed_oob_tx_cmd) * OOB_TXCMD_DESC_NUM) + 
										  (sizeof(struct aspeed_oob_rx_cmd) * OOB_RXCMD_DESC_NUM),
										  &espi_oob->oob_tx_cmd_dma, GFP_KERNEL);
//			printk("espi_oob->oob_tx_cmd %x , espi_oob->oob_tx_cmd_dma %x size of tx cmd %x \n", espi_oob->oob_tx_cmd, espi_oob->oob_tx_cmd_dma, sizeof(struct aspeed_oob_tx_cmd));
			
			//rx cmd desc
			espi_oob->oob_rx_cmd = (struct aspeed_oob_rx_cmd *) ((u32)espi_oob->oob_tx_cmd + (sizeof(struct aspeed_oob_tx_cmd) * OOB_TXCMD_DESC_NUM));
			espi_oob->oob_rx_cmd_dma = espi_oob->oob_tx_cmd_dma + (sizeof(struct aspeed_oob_tx_cmd) * OOB_TXCMD_DESC_NUM);
//			printk("espi_oob->oob_rx_cmd %x , espi_oob->oob_rx_cmd_dma %x rx cmd size %x \n", espi_oob->oob_rx_cmd, espi_oob->oob_rx_cmd_dma, sizeof(struct aspeed_oob_rx_cmd));

			//init tx cmd buffer
			espi_oob->oob_tx_buff = (u8 *) ((u32)espi_oob->oob_rx_cmd + (sizeof(struct aspeed_oob_rx_cmd) * OOB_RXCMD_DESC_NUM));
			espi_oob->oob_tx_buff_dma = espi_oob->oob_rx_cmd_dma + (sizeof(struct aspeed_oob_rx_cmd) * OOB_RXCMD_DESC_NUM);
//			printk("espi_oob->oob_tx_buff %x , espi_oob->oob_tx_buff_dma %x \n", espi_oob->oob_tx_buff, espi_oob->oob_tx_buff_dma);

			//init tx cmd desc
			for(i = 0; i < OOB_TXCMD_DESC_NUM; i++) {
				espi_oob->oob_tx_cmd[i].dma_addr = (dma_addr_t) (espi_oob->oob_tx_buff + (i * OOB_BUFF_SIZE));
//				printk("tx idx %d : dma %x \n", i, espi_oob->oob_tx_cmd[i].dma_addr);
			}

			espi_oob->oob_tx_idx = 0;
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_TX_DMA, espi_oob->oob_tx_cmd_dma);
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_TX_RING_SIZE, OOB_TXCMD_DESC_NUM);

			//init rx cmd buffer
			espi_oob->oob_rx_buff = espi_oob->oob_tx_buff + (OOB_BUFF_SIZE * OOB_TX_BUF_NUM);
			espi_oob->oob_rx_buff_dma = espi_oob->oob_tx_buff_dma + (OOB_BUFF_SIZE * OOB_TX_BUF_NUM);
//			printk("espi_oob->oob_rx_buff %x , espi_oob->oob_rx_buff_dma %x \n", espi_oob->oob_rx_buff, espi_oob->oob_rx_buff_dma);

			//init rx cmd desc
			for(i = 0; i < OOB_RXCMD_DESC_NUM; i++) {
				espi_oob->oob_rx_cmd[i].cmd = 0;
				espi_oob->oob_rx_cmd[i].dma_addr = (dma_addr_t) (espi_oob->oob_rx_buff + (i * OOB_BUFF_SIZE));
//				printk("rx idx %d : dma %x \n", i, espi_oob->oob_rx_cmd[i].dma_addr);
			}
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_DMA, espi_oob->oob_rx_cmd_dma);
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_RING_SIZE, OOB_RXCMD_DESC_NUM);
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_WRITE_PT, BIT(31));
		} else {
			espi_oob->oob_tx_buff = dma_alloc_coherent(NULL,
										  (MAX_XFER_BUFF_SIZE * 2),
										  &espi_oob->oob_tx_buff_dma, GFP_KERNEL);
			espi_oob->oob_rx_buff = espi_oob->oob_tx_buff + MAX_XFER_BUFF_SIZE;
			espi_oob->oob_rx_buff_dma = espi_oob->oob_tx_buff_dma + MAX_XFER_BUFF_SIZE;
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_RX_DMA, espi_oob->oob_rx_buff_dma);
			regmap_write(espi_oob->map, ASPEED_ESPI_OOB_TX_DMA, espi_oob->oob_tx_buff_dma);
		}
		espi_oob->oob_rx_full = 0;
	} else {
		// non-dma mode 
		espi_oob->oob_rx_buff = kzalloc(MAX_XFER_BUFF_SIZE * 2, GFP_KERNEL);
		espi_oob->oob_tx_buff = espi_oob->oob_rx_buff + MAX_XFER_BUFF_SIZE;
	}

	rc = devm_request_irq(&pdev->dev, espi_oob->irq, aspeed_espi_oob_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_oob);
	if (rc) {
		printk("espi oob Unable to get IRQ \n");
		return rc;
	}

	espi_oob->rest_irq = platform_get_irq(pdev, 1);
	if (espi_oob->rest_irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		return espi_oob->rest_irq;
	}
	rc = devm_request_irq(&pdev->dev, espi_oob->rest_irq, aspeed_espi_oob_reset_irq, IRQF_SHARED,
				dev_name(&pdev->dev), espi_oob);
	if (rc) {
		printk("espi peripheral Unable to get reset IRQ \n");
		return rc;
	}

	//set oob ready 
	regmap_update_bits(espi_oob->map, ASPEED_ESPI_CTRL, ESPI_CTRL_OOB_FW_RDY, ESPI_CTRL_OOB_FW_RDY);
	
	espi_oob->miscdev.minor = MISC_DYNAMIC_MINOR;
	espi_oob->miscdev.name = DEVICE_NAME;
//	espi_oob->miscdev.fops = &aspeed_espi_oob_fops;
	espi_oob->miscdev.parent = dev;
	rc = misc_register(&espi_oob->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	pr_info("aspeed espi oob loaded \n");

	return 0;
}

static int aspeed_espi_oob_remove(struct platform_device *pdev)
{
	struct aspeed_espi_oob *espi_oob = dev_get_drvdata(&pdev->dev);

	misc_deregister(&espi_oob->miscdev);
	return 0;
}


static struct platform_driver aspeed_espi_oob_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = aspeed_espi_oob_match,
	},
	.probe  = aspeed_espi_oob_probe,
	.remove = aspeed_espi_oob_remove,
};
module_platform_driver(aspeed_espi_oob_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed ESPI OOB Driver");
