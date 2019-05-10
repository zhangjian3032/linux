// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2019-2020 ASPEED Technology Inc.

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/peci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

/* Pilot PECI registers */
#define PILOT_PECICTL		0x00
#define PILOT_PECISTS		0x04
#define PILOT_PECIOPTRATE	0x08
#define PILOT_PECIPTRS		0x0C
#define PILOT_PECIHWFCS		0x10
#define PILOT_PECIRXOFF		0x14
#define PILOT_PECIBITBANG	0x18
#define PILOT_PECIPULFLTR	0x1C
#define PILOT_PECI_TXRX_Q0	0x20
#define PILOT_PECI_TXRX_Q1	0x24
#define PILOT_PECI_TXRX_Q2	0x28
#define PILOT_PECI_TXRX_Q3	0x2C
#define PILOT_PECI_TXRX_Q4	0x30
#define PILOT_PECI_TXRX_Q5	0x34
#define PILOT_PECI_TXRX_Q6	0x38
#define PILOT_PECI_TXRX_Q7	0x3C
#define PILOT_PECIPOLLCTRL	0x60
#define PILOT_PECIPOLLINFO	0x64
#define PILOT_PECIPOLLDLY	0x68

#define PECI_IDLE_CHECK_TIMEOUT_USEC    50000
#define PECI_IDLE_CHECK_INTERVAL_USEC   10000
#define PECI_CMD_TIMEOUT_MS_MAX         60000
#define PECI_CMD_TIMEOUT_MS_DEFAULT	1000
#define PILOT_BIT00_MSK		(0x1 << 0)
#define PILOT_BIT01_MSK		(0x1 << 1)
#define PILOT_BIT02_MSK		(0x1 << 2)
#define PILOT_BIT03_MSK		(0x1 << 3)
#define PILOT_BIT04_MSK		(0x1 << 4)
#define PILOT_BIT05_MSK		(0x1 << 5)
#define PILOT_BIT06_MSK		(0x1 << 6)
#define PILOT_BIT07_MSK		(0x1 << 7)
#define PILOT_BIT08_MSK		(0x1 << 8)
#define PILOT_BIT09_MSK		(0x1 << 9)
#define PILOT_BIT10_MSK		(0x1 << 10)
#define PILOT_BIT11_MSK		(0x1 << 11)
#define PILOT_BIT12_MSK		(0x1 << 12)
#define PILOT_BIT13_MSK		(0x1 << 13)
#define PILOT_BIT14_MSK		(0x1 << 14)
#define PILOT_BIT15_MSK		(0x1 << 15)
#define PILOT_BIT16_MSK		(0x1 << 16)
#define PILOT_BIT17_MSK		(0x1 << 17)
#define PILOT_BIT18_MSK		(0x1 << 18)
#define PILOT_BIT19_MSK		(0x1 << 19)
#define PILOT_BIT20_MSK		(0x1 << 20)
#define PILOT_BIT21_MSK		(0x1 << 21)
#define PILOT_BIT22_MSK		(0x1 << 22)
#define PILOT_BIT23_MSK		(0x1 << 23)
#define PILOT_BIT24_MSK		(0x1 << 24)
#define PILOT_BIT25_MSK		(0x1 << 25)
#define PILOT_BIT26_MSK		(0x1 << 26)
#define PILOT_BIT27_MSK		(0x1 << 27)
#define PILOT_BIT28_MSK		(0x1 << 28)
#define PILOT_BIT29_MSK		(0x1 << 29)
#define PILOT_BIT30_MSK		(0x1 << 30)
#define PILOT_BIT31_MSK		(0x1 << 31)

#define PILOT_PECI_CMD_DONE	PILOT_BIT01_MSK	
#define PILOT_POLL_DONE		PILOT_BIT03_MSK //Wont be set
#define PILOT_POLL_ON		PILOT_BIT06_MSK //Wont be set
#define PILOT_ABORT_FCS		PILOT_BIT07_MSK //Wont be set
struct pilot_peci {
	struct peci_adapter	*adapter;
	struct device		*dev;
	struct regmap		*regmap;
	/*
	struct clk		*clk;
	struct reset_control	*rst;
	*/
	int			irq;
	spinlock_t		lock; /* to sync completion status handling */
	struct completion	xfer_complete;
	u32			status;
	u32			cmd_timeout_ms;
};

static const struct regmap_config pilot_peci_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = PILOT_PECIPOLLDLY,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.fast_io = true,
};
static void pilot_reset_peci(struct pilot_peci *priv)
{
        unsigned long flags;
        unsigned int peci_ctrl = 0;
        int i  = 0, j = 0;
	bool change;
        /*unsigned int start_addr = (SE_PECI_VA_BASE+0x20);*/
        local_irq_save(flags);
#if 0
        peci_ctrl = *((volatile unsigned long *)(SE_PECI_VA_BASE+PECICTL_REG));
        *((volatile unsigned long *)(SE_PECI_VA_BASE+PECICTL_REG)) &= ~(PECI_ENABLE);
        *((volatile unsigned long *)(SE_PECI_VA_BASE+PECICTL_REG)) = peci_ctrl & ~(PECI_TRIGGER);
        /* Clear Interrupt */
        *((volatile unsigned long *)(SE_PECI_VA_BASE+PECISTS_REG)) |= PECI_INTR_STATUS;
#else
	/* In order to ensure the PECI controller is in sane state
	 * it is advised to disable and enable it once before starting
	 * any transactions*/
	/* Disable PECI*/
	regmap_update_bits_base(priv->regmap, PILOT_PECICTL,
                                0x1, 0x0, &change, false, true);

	usleep_range(PECI_IDLE_CHECK_INTERVAL_USEC, 
			PECI_IDLE_CHECK_INTERVAL_USEC+ 3000);
	/* Enable PECI*/
	regmap_update_bits_base(priv->regmap, PILOT_PECICTL,
                                0x1, 0x1, &change, false, true);
	
	regmap_read(priv->regmap, PILOT_PECICTL, &peci_ctrl);
	/* Clear Trigger*/
	regmap_update_bits_base(priv->regmap, PILOT_PECICTL,
                                (peci_ctrl| (1<< 1)), 
				(peci_ctrl | ~(1 << 1)), 
				 &change, false, true);

#endif
        /* Restore all registers with reset values */
        for(i = 0;i < 19; i++) {
		 regmap_update_bits_base(priv->regmap, 
				(PILOT_PECI_TXRX_Q0+j),
                                0xff, 0x0,
                                &change, false, true);

            /*    *(volatile unsigned long*)(start_addr + j) = 0;*/
                j = j + 4;
        }
	regmap_write(priv->regmap, PILOT_PECICTL, 0x40);
	regmap_write(priv->regmap, PILOT_PECIOPTRATE, 0x000004E2);
	regmap_write(priv->regmap, PILOT_PECIPTRS, 0x04E20000);
	regmap_write(priv->regmap, PILOT_PECIHWFCS, 0x0);
	regmap_write(priv->regmap, PILOT_PECIRXOFF, 0x00040003);
	regmap_write(priv->regmap, PILOT_PECIBITBANG, 0x0);
	regmap_write(priv->regmap, PILOT_PECIPULFLTR, 0x3);
	regmap_write(priv->regmap, PILOT_PECIPOLLINFO, 0x0000FF00);
	/*
        *((volatile unsigned long *)(SE_PECI_VA_BASE + PECICTL_REG))  = 0x40;
        *((volatile unsigned long *)(SE_PECI_VA_BASE + PECIOPTRATE_REG)) = 0x000004E2;
        *((volatile unsigned long *)(SE_PECI_VA_BASE + PECIPTRS_REG)) = 0x04E20000;
        *((volatile unsigned long *)(SE_PECI_VA_BASE + PECIHWFCS_REG)) = 0;
        *((volatile unsigned long *)(SE_PECI_VA_BASE + PECIRXOFF_REG)) = 0x00040003;
        *((volatile unsigned long *)(SE_PECI_VA_BASE + PECIBITBANG_REG)) = 0;
        *((volatile unsigned long *)(SE_PECI_VA_BASE + PECIPULFLTR)) = 0x00000003;
        *((volatile unsigned long *)(SE_PECI_VA_BASE + PECI_POLLINFO_REG)) = 0x0000FF00;
	*/
        local_irq_restore(flags);
        return;
}

static irqreturn_t pilot_peci_irq_handler(int irq, void *arg)
{
	struct pilot_peci *priv = arg;
	u32 status_ack = 0;
	u32 status;
	bool change;

	spin_lock(&priv->lock);
	regmap_read(priv->regmap, PILOT_PECISTS, &status);

	if (status & PILOT_PECI_CMD_DONE) {
		dev_dbg(priv->dev, "PILOT_PECI_CMD_DONE\n");
		status_ack |= PILOT_PECI_CMD_DONE;
		priv->status = PILOT_PECI_CMD_DONE;
		complete(&priv->xfer_complete);
	}
	if (status & PILOT_POLL_DONE) {
		dev_dbg(priv->dev, "PILOT_POLL_DONE\n");
		status_ack |= PILOT_POLL_DONE;
	}
	if (status & PILOT_POLL_ON) {
		dev_dbg(priv->dev, "PILOT_POLL_ON\n");
		status_ack |= PILOT_POLL_ON;
	}
	if (status & PILOT_ABORT_FCS) {
		dev_dbg(priv->dev, "PILOT_ABORT_FCS\n");
		status_ack |= PILOT_ABORT_FCS;
	}

	regmap_update_bits_base(priv->regmap, PILOT_PECISTS,
                   status_ack, status_ack,
                   &change, false, true);
	spin_unlock(&priv->lock);
	return IRQ_HANDLED;
}

static int pilot_peci_init_ctrl(struct pilot_peci *priv)
{
	u32 msg_timing, addr_timing, rd_sampling_point;
	u32 clk_freq, clk_divisor, clk_div_val = 0;
	int ret;
	
	pilot_reset_peci(priv);
	ret = of_property_read_u32(priv->dev->of_node, "cmd-timeout-ms",
				   &priv->cmd_timeout_ms);
	if (ret || priv->cmd_timeout_ms > PECI_CMD_TIMEOUT_MS_MAX ||
	    priv->cmd_timeout_ms == 0) {
		if (!ret)
			dev_warn(priv->dev,
				 "Invalid cmd-timeout-ms : %u. Use default : %u\n",
				 priv->cmd_timeout_ms,
				 PECI_CMD_TIMEOUT_MS_DEFAULT);
		priv->cmd_timeout_ms = PECI_CMD_TIMEOUT_MS_DEFAULT;
	}
#if 0
	priv->clk = devm_clk_get(priv->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(priv->dev, "Failed to get clk source.\n");
		return PTR_ERR(priv->clk);
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(priv->dev, "Failed to enable clock.\n");
		return ret;
	}

	ret = of_property_read_u32(priv->dev->of_node, "clock-frequency",
				   &clk_freq);
	if (ret) {
		dev_err(priv->dev,
			"Could not read clock-frequency property.\n");
		clk_disable_unprepare(priv->clk);
		return ret;
	}

	clk_divisor = clk_get_rate(priv->clk) / clk_freq;

	while ((clk_divisor >> 1) && (clk_div_val < PECI_CLK_DIV_MAX))
		clk_div_val++;

	ret = of_property_read_u32(priv->dev->of_node, "msg-timing",
				   &msg_timing);
	if (ret || msg_timing > PECI_MSG_TIMING_MAX) {
		if (!ret)
			dev_warn(priv->dev,
				 "Invalid msg-timing : %u, Use default : %u\n",
				 msg_timing, PECI_MSG_TIMING_DEFAULT);
		msg_timing = PECI_MSG_TIMING_DEFAULT;
	}

	ret = of_property_read_u32(priv->dev->of_node, "addr-timing",
				   &addr_timing);
	if (ret || addr_timing > PECI_ADDR_TIMING_MAX) {
		if (!ret)
			dev_warn(priv->dev,
				 "Invalid addr-timing : %u, Use default : %u\n",
				 addr_timing, PECI_ADDR_TIMING_DEFAULT);
		addr_timing = PECI_ADDR_TIMING_DEFAULT;
	}

	ret = of_property_read_u32(priv->dev->of_node, "rd-sampling-point",
				   &rd_sampling_point);
	if (ret || rd_sampling_point > PECI_RD_SAMPLING_POINT_MAX) {
		if (!ret)
			dev_warn(priv->dev,
				 "Invalid rd-sampling-point : %u. Use default : %u\n",
				 rd_sampling_point,
				 PECI_RD_SAMPLING_POINT_DEFAULT);
		rd_sampling_point = PECI_RD_SAMPLING_POINT_DEFAULT;
	}


	regmap_write(priv->regmap, ASPEED_PECI_CTRL,
		     FIELD_PREP(PECI_CTRL_CLK_DIV_MASK, PECI_CLK_DIV_DEFAULT) |
		     PECI_CTRL_PECI_CLK_EN);

	/**
	 * Timing negotiation period setting.
	 * The unit of the programmed value is 4 times of PECI clock period.
	 */
	regmap_write(priv->regmap, ASPEED_PECI_TIMING,
		     FIELD_PREP(PECI_TIMING_MESSAGE_MASK, msg_timing) |
		     FIELD_PREP(PECI_TIMING_ADDRESS_MASK, addr_timing));

	/* Clear interrupts */
	regmap_write(priv->regmap, ASPEED_PECI_INT_STS, PECI_INT_MASK);

	/* Enable interrupts */
	regmap_write(priv->regmap, ASPEED_PECI_INT_CTRL, PECI_INT_MASK);

	/* Read sampling point and clock speed setting */
	regmap_write(priv->regmap, ASPEED_PECI_CTRL,
		     FIELD_PREP(PECI_CTRL_SAMPLING_MASK, rd_sampling_point) |
		     FIELD_PREP(PECI_CTRL_CLK_DIV_MASK, clk_div_val) |
		     PECI_CTRL_PECI_EN | PECI_CTRL_PECI_CLK_EN);
#endif
	return 0;
}
static void pilot_write_txq(struct pilot_peci *priv, u8 addr, u8 tx_len, u8 rx_len, u8 *data)
{
        unsigned long tx0;
        int i, j;
        int curr_tx_pos = PILOT_PECI_TXRX_Q0;

        /* Fill the initial params */
        tx0 = (rx_len << 16) | (tx_len << 8) | addr;
        if (tx_len >= 1)
                tx0 |= (data[0] << 24);
        //*((volatile unsigned long *)(SE_PECI_VA_BASE+PECITXQ_REG(0))) = tx0;
	regmap_write(priv->regmap, curr_tx_pos, tx0);
	if (tx_len > 1)
	{
		/* We already used the data[0](Command) and TXQ(0) */
		/*TxQ = ((unsigned char *)(SE_PECI_VA_BASE+PECITXQ_REG(1)));
                for(i=1;i<tx_len;i++)
                        TxQ[i-1] = data[i];*/

                tx_len -=1; /* calc. the remaining tx bytes */
                i = 1;
                j = 1;
                while (tx_len)
                {
			curr_tx_pos +=4;
			//*((volatile unsigned long *)(SE_PECI_VA_BASE+PECITXQ_REG(i))) = *((volatile unsigned long *)&data[j]);
			regmap_write(priv->regmap, curr_tx_pos, *((volatile u32 *) &data[j]));
                        if ( tx_len>4)
                        {
                                i++;
                                j +=4;
                                tx_len -= 4;
                        }
                        else
                        {
                                break;
                        }

                }

        }
        return;
}

static int pilot_peci_xfer(struct peci_adapter *adapter,
			    struct peci_xfer_msg *msg)
{
	struct pilot_peci *priv = peci_get_adapdata(adapter);
	u32 cmd_sts, flags;
	int i=0; 
	int rc;
	long err, timeout = msecs_to_jiffies(priv->cmd_timeout_ms);
	bool change;
	u8 rx_offset, rx_bytes_to_skip;
	u8 temp_bytes[3 + msg->tx_len];
	u8 tlen = 3 + msg->tx_len + msg->rx_len;
        int curr_rx_pos = PILOT_PECI_TXRX_Q0;

	//First check if the previous commands have completed
	//and the controller is in Idle state
	rc = regmap_read_poll_timeout(priv->regmap, PILOT_PECISTS, cmd_sts,
				      !(cmd_sts & PILOT_BIT00_MSK),
				      PECI_IDLE_CHECK_INTERVAL_USEC,
				      PECI_IDLE_CHECK_TIMEOUT_USEC);
	
	//return aspeed_peci_xfer_native(priv, msg);
	//Set the Interrupt enable bit and the FCS Error Enable bit	
	regmap_update_bits_base(priv->regmap, PILOT_PECICTL,
                                (PILOT_BIT02_MSK | PILOT_BIT10_MSK),
				(PILOT_BIT02_MSK | PILOT_BIT10_MSK),
				&change, false, true);
	regmap_write(priv->regmap, PILOT_PECIPOLLCTRL, 0x0);
	regmap_write(priv->regmap, PILOT_PECIPOLLINFO, 0x0);
	regmap_write(priv->regmap, PILOT_PECIPOLLDLY, 0x0);

	//Set the optimum rate
	regmap_write(priv->regmap, PILOT_PECIOPTRATE, 0x500);

	//Write to the tx queue
	pilot_write_txq(priv, msg->addr, msg->tx_len, msg->rx_len, msg->tx_buf);

	//Since the tx will feedback to rx as the queue is same
	//ignore the tx data and the first 3 bytes which contains
	//address, tx_len and rx_len
	rx_bytes_to_skip = rx_offset = 3 +  msg->tx_len;
	regmap_write(priv->regmap, PILOT_PECIRXOFF,
			(rx_bytes_to_skip | (rx_offset << 8)));

	priv->status = 0;
	//Now trigger the command
	regmap_update_bits_base(priv->regmap, PILOT_PECICTL,
                                (PILOT_BIT01_MSK), (PILOT_BIT01_MSK),
				&change, false, true);

	//Wait for command to complete
	err = wait_for_completion_interruptible_timeout(&priv->xfer_complete,
							timeout);
	spin_lock_irqsave(&priv->lock, flags);

	if(priv->status != PILOT_PECI_CMD_DONE){
		if (err < 0) { /* -ERESTARTSYS */
			rc = (int)err;
			goto err_irqrestore;
		} else if (err == 0) {
			dev_dbg(priv->dev, "Timeout waiting for a response!\n");
			rc = -ETIMEDOUT;
			goto err_irqrestore;
		}
	}
	while(tlen){
		regmap_read(priv->regmap, curr_rx_pos,
				(volatile u32*)&temp_bytes[i]);
		i +=4;
		curr_rx_pos +=4;
	}
	memcpy(msg->rx_buf, &temp_bytes[rx_bytes_to_skip - 1], msg->rx_len);

err_irqrestore:
	spin_unlock_irqrestore(&priv->lock, flags);
	return rc;
}

static int pilot_peci_probe(struct platform_device *pdev)
{
	struct peci_adapter *adapter;
	struct pilot_peci *priv;
	struct resource *res;
	void __iomem *base;
	u32 sts;
	int ret;

	adapter = peci_alloc_adapter(&pdev->dev, sizeof(*priv));
	if (!adapter)
		return -ENOMEM;

	priv = peci_get_adapdata(adapter);
	priv->adapter = adapter;
	priv->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, priv);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		goto err_put_adapter_dev;
	}

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &pilot_peci_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		goto err_put_adapter_dev;
	}

	/**
	 * We check that the regmap works on this very first access,
	 * but as this is an MMIO-backed regmap, subsequent regmap
	 * access is not going to fail and we skip error checks from
	 * this point.
	 */
	ret = regmap_read(priv->regmap, PILOT_PECISTS, &sts);
	if (ret) {
		ret = -EIO;
		goto err_put_adapter_dev;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (!priv->irq) {
		ret = -ENODEV;
		goto err_put_adapter_dev;
	}

	ret = devm_request_irq(&pdev->dev, priv->irq, pilot_peci_irq_handler,
			       0, "peci-pilot-irq", priv);
	if (ret)
		goto err_put_adapter_dev;

	init_completion(&priv->xfer_complete);
	spin_lock_init(&priv->lock);

	priv->adapter->owner = THIS_MODULE;
	priv->adapter->dev.of_node = of_node_get(dev_of_node(priv->dev));
	strlcpy(priv->adapter->name, pdev->name, sizeof(priv->adapter->name));
	priv->adapter->xfer = pilot_peci_xfer;
#if 0
	priv->rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(priv->rst)) {
		dev_err(&pdev->dev,
			"missing or invalid reset controller entry");
		ret = PTR_ERR(priv->rst);
		goto err_put_adapter_dev;
	}
	reset_control_deassert(priv->rst);
#else
#endif
	ret = pilot_peci_init_ctrl(priv);
	if (ret)
		goto err_put_adapter_dev;

	ret = peci_add_adapter(priv->adapter);
	if (ret)
		goto err_put_adapter_dev;

	dev_info(&pdev->dev, "peci bus %d registered, irq %d\n",
		 priv->adapter->nr, priv->irq);

	return 0;

err_put_adapter_dev:
	put_device(&adapter->dev);
	return ret;
}
static int pilot_peci_remove(struct platform_device *pdev)
{
	struct pilot_peci *priv = dev_get_drvdata(&pdev->dev);

	pilot_reset_peci(priv);
	//clk_disable_unprepare(priv->clk);
	//reset_control_assert(priv->rst);
	peci_del_adapter(priv->adapter);
	of_node_put(priv->adapter->dev.of_node);

	return 0;
}

static const struct of_device_id pilot_peci_of_table[] = {
	{ .compatible = "aspeed,pilot-peci", },
	{ }
};

MODULE_DEVICE_TABLE(of, pilot_peci_of_table);

static struct platform_driver pilot_peci_driver = {
	.probe  = pilot_peci_probe,
	.remove = pilot_peci_remove,
	.driver = {
		.name           = "peci-pilot",
		.of_match_table = of_match_ptr(pilot_peci_of_table),
	},
};

module_platform_driver(pilot_peci_driver);
MODULE_AUTHOR("Shivah Shankar S <shivahs@apeedtech.com>");
MODULE_DESCRIPTION("ASPEED PILOT PECI driver");
MODULE_LICENSE("GPL v2");
