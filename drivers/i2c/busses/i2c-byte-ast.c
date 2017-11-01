/*
 * i2c-byte-ast.c - I2C Byte mode driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h> 

#include "i2c-ast.h"
/***************************************************************************/
#define AST_I2C_S_DEBUG 1

#ifdef AST_I2C_S_DEBUG
#define I2C_S_DBUG(fmt, args...) printk(fmt, ## args)
#else
#define I2C_S_DBUG(fmt, args...)
#endif

//#define AST_I2C_M_DEBUG 1

#ifdef AST_I2C_M_DEBUG
#define I2C_M_DBUG(fmt, args...) printk(fmt, ## args)
#else
#define I2C_M_DBUG(fmt, args...)
#endif

//AST2400 buffer mode issue , force I2C slave write use byte mode , read use buffer mode 
/* Use platform_data instead of module parameters */
/* Fast Mode = 400 kHz, Standard = 100 kHz */
//static int clock = 100; /* Default: 100 kHz */
/***************************************************************************/
#define I2C_S_BUF_SIZE 		256
#define I2C_S_RX_BUF_NUM 	20

#define AST_LOCKUP_DETECTED (0x1 << 15)
#define AST_I2C_LOW_TIMEOUT 0x07
/***************************************************************************/
struct ast_byte_i2c_bus {
	struct device		*dev;
	void __iomem		*reg_base;			/* virtual */	
	int 				irq;				//I2C IRQ number 
	struct clk 			*clk;
	u32					apb_clk;
	u32					bus_frequency;	
	struct ast_i2c_bus_config	*bus_config;
	u32					state;				//I2C xfer mode state matchine 
	u32					bus_recover;
	struct i2c_adapter	adap;
//master	
	u8					master_operation;
	int					xfer_last;			//cur xfer is last msgs for stop msgs
	struct i2c_msg 		*master_msgs;		//cur xfer msgs
	int					master_xfer_len;			//cur xfer len 
	int					master_xfer_cnt;			//total xfer count
	struct completion	cmd_complete;
	int					cmd_err;
	u8 					blk_r_flag; 		//for smbus block read
//Slave structure	
	u8					slave_operation;
	u8					slave_event;
	struct i2c_msg		*slave_msgs; 		//cur slave xfer msgs
	int 				slave_xfer_len;			
	int 				slave_xfer_cnt;		
	u8					slave_en;
#ifdef CONFIG_AST_I2C_SLAVE_MODE
	u8					slave_rx_full;	
	u8					slave_rx_idx;	
	u8					slave_ioctl_idx;
	struct i2c_msg		slave_rx_msg[I2C_S_RX_BUF_NUM];
	struct i2c_msg		slave_tx_msg;	
#endif
};

static inline void
ast_byte_i2c_write(struct ast_byte_i2c_bus *i2c_bus, u32 val, u32 reg)
{
//	dev_dbg(i2c_bus->dev, "ast_byte_i2c_write : val: %x , reg : %x \n",val,reg);	
	writel(val, i2c_bus->reg_base+ reg);
}

static inline u32
ast_byte_i2c_read(struct ast_byte_i2c_bus *i2c_bus, u32 reg)
{
#if 0
	u32 val = readl(i2c_bus->reg_base + reg);
	printk("R : reg %x , val: %x \n",reg, val);
	return val;
#else	
	return readl(i2c_bus->reg_base + reg);
#endif
}


#ifdef CONFIG_AST_I2C_SLAVE_MODE
/* AST I2C Slave mode  */
static void ast_slave_issue_alert(struct ast_byte_i2c_bus *i2c_bus, u8 enable)
{
	//only support dev0~3
	if(i2c_bus->adap.nr > 3)
		return;
	else {
		if(enable)
			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) | AST_I2CD_S_ALT_EN, I2C_CMD_REG);
		else
			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) & ~AST_I2CD_S_ALT_EN, I2C_CMD_REG);
	}
}

static void ast_slave_mode_enable(struct ast_byte_i2c_bus *i2c_bus, struct i2c_msg *msgs)
{
	if(msgs->buf[0] == 1) {
		i2c_bus->slave_en = 1;
		dev_dbg(i2c_bus->dev, "slave enable msgs->addr %x \n", msgs->addr);
		ast_byte_i2c_write(i2c_bus, msgs->addr, I2C_DEV_ADDR_REG);	
		if(i2c_bus->master_operation) {
			dev_dbg(i2c_bus->dev, "master is ongoing\n");
		} else {
			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_FUN_CTRL_REG) | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);		
		}
	} else {
		i2c_bus->slave_en = 0;
		ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_FUN_CTRL_REG) & ~AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
	}
}

//for memory buffer initial
static void ast_i2c_slave_buff_init(struct ast_byte_i2c_bus *i2c_bus)
{
	int i;
	//Tx buf  1 
	i2c_bus->slave_tx_msg.len = I2C_S_BUF_SIZE;
	i2c_bus->slave_tx_msg.buf = kzalloc(I2C_S_BUF_SIZE, GFP_KERNEL);
	//Rx buf 4
	for(i=0; i<I2C_S_RX_BUF_NUM; i++) {
		i2c_bus->slave_rx_msg[i].addr = 0;	//mean ~BUFF_ONGOING
		i2c_bus->slave_rx_msg[i].flags = 0;	//mean empty buffer 
		i2c_bus->slave_rx_msg[i].len = I2C_S_BUF_SIZE;
		i2c_bus->slave_rx_msg[i].buf = kzalloc(I2C_S_BUF_SIZE, GFP_KERNEL);
	}	
	//assign cur msg is #0
	i2c_bus->slave_msgs = &i2c_bus->slave_rx_msg[0];
	i2c_bus->slave_rx_full = 0;
	i2c_bus->slave_rx_idx = 0;
	i2c_bus->slave_ioctl_idx = 0;
	
}

static void ast_byte_i2c_slave_rdwr_xfer(struct ast_byte_i2c_bus *i2c_bus)
{
	switch(i2c_bus->slave_event) {
		case I2C_SLAVE_EVENT_START_WRITE:
			dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_START_WRITE\n");
			if(i2c_bus->slave_msgs->addr) {
				//use repeat start, use over write buffer .. 
				dev_err(i2c_bus->dev, "slave ERROR S[%d], sts[%x] 0x00[%x]~~ TODO ~~~~!!\n", i2c_bus->slave_operation, (ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) >> 19) & 0xf, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG));
				dev_err(i2c_bus->dev, "slave msg flag [%x] len [%d]\n", i2c_bus->slave_msgs->flags, i2c_bus->slave_msgs->len);
				dev_err(i2c_bus->dev, "slave ioctl idx [%d] rx_idx [%d] rx_full [%d]\n", i2c_bus->slave_ioctl_idx, i2c_bus->slave_rx_idx, i2c_bus->slave_rx_full);
			} else {
				i2c_bus->slave_msgs->addr = BUFF_ONGOING;
			}
			if(i2c_bus->slave_rx_msg[(i2c_bus->slave_rx_idx + 1) % I2C_S_RX_BUF_NUM].flags == BUFF_FULL) {
				i2c_bus->slave_rx_full = 1;
				dev_err(i2c_bus->dev, "buffer full-disable [%x] TODO ~~\n",ast_byte_i2c_read(i2c_bus, I2C_CMD_REG));
			}
//			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) & ~(AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);	//will enable when slave rx stop	
			break;
		case I2C_SLAVE_EVENT_START_READ:
			printk("I2C_SLAVE_EVENT_START_READ ERROR .. not imple \n");
			i2c_bus->slave_msgs = &i2c_bus->slave_tx_msg;
			break;
		case I2C_SLAVE_EVENT_WRITE:
			//check buffer full ? 
			if(i2c_bus->slave_msgs->len > I2C_S_BUF_SIZE)
				dev_dbg(i2c_bus->dev, "ERROR : buffer is out ~~!! \n");
			break;
		case I2C_SLAVE_EVENT_READ:
			printk("I2C_SLAVE_EVENT_READ ERROR .. not imple \n");
			dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_READ ERROR ... \n");
			i2c_bus->slave_msgs = &i2c_bus->slave_tx_msg;
			break;
		case I2C_SLAVE_EVENT_NACK:
			printk("I2C_SLAVE_EVENT_NACK ERROR .. not imple \n");
			dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_NACK ERROR ... \n");
			i2c_bus->slave_msgs = &i2c_bus->slave_tx_msg;
			break;
		case I2C_SLAVE_EVENT_STOP:
			dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_STOP buf idx %d, rx len : %d\n", i2c_bus->slave_rx_idx, i2c_bus->slave_msgs->len);
			i2c_bus->slave_msgs->addr = 0;
			i2c_bus->slave_msgs->flags = BUFF_FULL;
			i2c_bus->slave_rx_idx++;
			i2c_bus->slave_rx_idx %= I2C_S_RX_BUF_NUM;
			i2c_bus->slave_msgs = &i2c_bus->slave_rx_msg[i2c_bus->slave_rx_idx];
			i2c_bus->slave_msgs->len = 0;				
			if(i2c_bus->slave_rx_full) {
				printk("slave full check !!! \n");
			} else {
				ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) | (AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);	//will enable when slave rx stop
			}
			i2c_bus->slave_operation = 0;
			break;
	}

}

static int ast_byte_i2c_slave_ioctl_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs)
{
	struct ast_byte_i2c_bus *i2c_bus = adap->algo_data;
	struct i2c_msg *slave_rx_msg = &i2c_bus->slave_rx_msg[i2c_bus->slave_ioctl_idx];
	int ret=0;	
//	int i;

	switch(msgs->flags) {
		case 0:
			if((slave_rx_msg->addr == 0) && (slave_rx_msg->flags == BUFF_FULL)) {
				dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_STOP buf idx %d : len %d \n", i2c_bus->slave_ioctl_idx, slave_rx_msg->len);
#if 0
				printk("buff \n");
				for(i=0; i < slave_rx_msg->len; i++)
					printk("%x ", slave_rx_msg->buf[i]);
				printk("\n");
#endif
				memcpy(msgs->buf, slave_rx_msg->buf, slave_rx_msg->len);
				msgs->len = slave_rx_msg->len;
				slave_rx_msg->len = 0;
				slave_rx_msg->flags = 0;	
				i2c_bus->slave_ioctl_idx++;
				i2c_bus->slave_ioctl_idx %= I2C_S_RX_BUF_NUM;
				if(i2c_bus->slave_rx_full) {
					dev_err(i2c_bus->dev, "slave re-enable \n");
					i2c_bus->slave_rx_full = 0;
					if(ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) & AST_I2CD_SLAVE_EN) {
						printk("buffer handle error !! ~~~~\n");
					} else {
						if((i2c_bus->slave_en) && (!i2c_bus->master_operation)) {
							printk("\n %d-slave enable watch out TODO\n", i2c_bus->adap.nr);
							ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
						} else {
							printk("\n TODO ~~~~~~~~~~~~~~");
						}						
					}
				}
			} else {
//				printk("%d-No Buff\n", i2c_bus->adap.nr);
				msgs->len = 0;
				ret = -1;
			}	
			break;
		case I2C_M_RD:	//slave write
			dev_dbg(i2c_bus->dev, "slave write \n");
			memcpy(msgs->buf, i2c_bus->slave_tx_msg.buf, I2C_S_BUF_SIZE);
			break;
		case I2C_S_EN:
			dev_dbg(i2c_bus->dev, "I2C_S_EN msgs->addr %x \n", msgs->addr);
			if((msgs->addr < 0x1) || (msgs->addr > 0xff)) {
				printk("addrsss not correct !! \n");
				ret = -1;
			} else {
				if(msgs->len != 1) printk("ERROR \n");
				ast_slave_mode_enable(i2c_bus, msgs);
			}
			break;
		case I2C_S_ALT:
			printk("slave issue alt\n");
			if(msgs->len != 1) printk("ERROR \n");
			if(msgs->buf[0]==1)
				ast_slave_issue_alert(i2c_bus, 1);
			else
				ast_slave_issue_alert(i2c_bus, 0);
			break;
		default:
			printk("slave xfer error \n");
			break;

	}
	return ret;
}

#endif

static u8 
ast_byte_i2c_bus_error_recover(struct ast_byte_i2c_bus *i2c_bus)
{
	u32 sts;
	int r;		
	u32 i = 0;
	dev_dbg(i2c_bus->dev, "ast_byte_i2c_bus_error_recover \n");

	ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) & ~(AST_I2CD_MASTER_EN | AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);	
	ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) | (AST_I2CD_MASTER_EN | AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);		
	//Check 0x14's SDA and SCL status
	sts = ast_byte_i2c_read(i2c_bus,I2C_CMD_REG);
	
	if ((sts & AST_I2CD_SDA_LINE_STS) && (sts & AST_I2CD_SCL_LINE_STS)) {
		//Means bus is idle.
		dev_dbg(i2c_bus->dev, "I2C bus (%d) is idle. I2C slave doesn't exist?! [%x]\n", i2c_bus->adap.nr, sts);
		return -1;
	}

	dev_dbg(i2c_bus->dev, "ERROR!! I2C(%d) bus hanged, try to recovery it!\n", i2c_bus->adap.nr);
	
	
	if ((sts & AST_I2CD_SDA_LINE_STS) && !(sts & AST_I2CD_SCL_LINE_STS)) {
		//if SDA == 1 and SCL == 0, it means the master is locking the bus.
		//Send a stop command to unlock the bus.		
		dev_dbg(i2c_bus->dev, "I2C's master is locking the bus, try to stop it.\n");
//
		init_completion(&i2c_bus->cmd_complete);
		i2c_bus->cmd_err = 0;

		ast_byte_i2c_write(i2c_bus, AST_I2CD_M_STOP_CMD, I2C_CMD_REG);

		r = wait_for_completion_interruptible_timeout(&i2c_bus->cmd_complete,
													   i2c_bus->adap.timeout*HZ);

		if(i2c_bus->cmd_err) {
			dev_dbg(i2c_bus->dev, "recovery error \n");
			return -1;
		}
			
		if (r == 0) {
			 dev_dbg(i2c_bus->dev, "recovery timed out\n");
			 return -1;
		} else {
			dev_dbg(i2c_bus->dev, "Recovery successfully\n");		
			return 0;
		}


	} else if (!(sts & AST_I2CD_SDA_LINE_STS)) {
		//else if SDA == 0, the device is dead. We need to reset the bus
		//And do the recovery command.
		dev_dbg(i2c_bus->dev, "I2C's slave is dead, try to recover it\n");
		//Let's retry 10 times
		for (i = 0; i < 10; i++) {
//			ast_byte_i2c_bus_init(i2c_bus);
			//Do the recovery command BIT11
			init_completion(&i2c_bus->cmd_complete);			
			ast_byte_i2c_write(i2c_bus, AST_I2CD_BUS_RECOVER_CMD_EN, I2C_CMD_REG);
			i2c_bus->bus_recover = 1;
			r = wait_for_completion_interruptible_timeout(&i2c_bus->cmd_complete,
														   i2c_bus->adap.timeout*HZ);			
			if (i2c_bus->cmd_err != 0) {
				dev_dbg(i2c_bus->dev, "ERROR!! Failed to do recovery command(0x%08x)\n", i2c_bus->cmd_err);
				return -1;
			}
			//Check 0x14's SDA and SCL status
			sts = ast_byte_i2c_read(i2c_bus,I2C_CMD_REG);
			if (sts & AST_I2CD_SDA_LINE_STS) //Recover OK
				break;
		}
		if (i == 10) {
			dev_dbg(i2c_bus->dev, "ERROR!! recover failed\n");
			return -1;
		}
	} else {
		dev_dbg(i2c_bus->dev, "Don't know how to handle this case?!\n");
		return -1;
	}
	dev_dbg(i2c_bus->dev, "Recovery successfully\n");
	return 0;
}

static void ast_master_alert_recv(struct ast_byte_i2c_bus *i2c_bus)	
{
	printk("ast_master_alert_recv bus id %d, Disable Alt, Please Imple \n",i2c_bus->adap.nr);
}

static int ast_byte_i2c_wait_bus_not_busy(struct ast_byte_i2c_bus *i2c_bus)
{
	int timeout = 10;

	while(ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) & (AST_I2CD_BUS_BUSY_STS)) {
		if(timeout<=0) {
			dev_dbg(i2c_bus->dev, "%d-bus busy %x \n",i2c_bus->adap.nr, ast_byte_i2c_read(i2c_bus,I2C_CMD_REG));
			ast_byte_i2c_bus_error_recover(i2c_bus);
			return -EAGAIN;
		}
		timeout--;
		mdelay(10);
	}

	return 0;
}
static void ast_byte_i2c_do_byte_xfer(struct ast_byte_i2c_bus *i2c_bus)
{
	u8 *xfer_buf;
	u32 cmd = 0;

	dev_dbg(i2c_bus->dev, "ast_byte_i2c_do_byte_xfer \n");	
	if(i2c_bus->slave_operation == 1) { 
		i2c_bus->slave_xfer_len = 1;
		dev_dbg(i2c_bus->dev,"S cnt %d, xf len %d \n",i2c_bus->slave_xfer_cnt, i2c_bus->slave_msgs->len);
		if(i2c_bus->slave_msgs->flags & I2C_M_RD) {
			//READ <-- TX
			dev_dbg(i2c_bus->dev, "(<--) slave(tx) buf %d [%x]\n", i2c_bus->slave_xfer_cnt, i2c_bus->slave_msgs->buf[i2c_bus->slave_xfer_cnt]);
			ast_byte_i2c_write(i2c_bus, i2c_bus->slave_msgs->buf[i2c_bus->slave_xfer_cnt], I2C_BYTE_BUF_REG);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_S_TX_CMD, I2C_CMD_REG); 
		} else {
			// Write -->Rx
			//no need to handle in byte mode
			dev_dbg(i2c_bus->dev, "(-->) slave(rx) BYTE do nothing\n");

		}
	} else {
		i2c_bus->master_xfer_len = 1;
	
		dev_dbg(i2c_bus->dev,"M cnt %d, xf len %d \n",i2c_bus->master_xfer_cnt, i2c_bus->master_msgs->len);
		if(i2c_bus->master_xfer_cnt == -1) {
			//first start 
			dev_dbg(i2c_bus->dev, " %sing %d byte%s %s 0x%02x\n",
							i2c_bus->master_msgs->flags & I2C_M_RD ? "read" : "write",
							i2c_bus->master_msgs->len, i2c_bus->master_msgs->len > 1 ? "s" : "",
							i2c_bus->master_msgs->flags & I2C_M_RD ? "from" : "to", i2c_bus->master_msgs->addr);
			
			if(i2c_bus->master_msgs->flags & I2C_M_RD) 
				ast_byte_i2c_write(i2c_bus, (i2c_bus->master_msgs->addr <<1) |0x1, I2C_BYTE_BUF_REG);
			else
				ast_byte_i2c_write(i2c_bus, (i2c_bus->master_msgs->addr <<1), I2C_BYTE_BUF_REG);

			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_INTR_CTRL_REG) |
								AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);			

			dev_dbg(i2c_bus->dev, " trigger cmd %x \n", AST_I2CD_M_TX_CMD | AST_I2CD_M_START_CMD);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_M_TX_CMD | AST_I2CD_M_START_CMD, I2C_CMD_REG);
		} else if (i2c_bus->master_xfer_cnt < i2c_bus->master_msgs->len){
			xfer_buf = i2c_bus->master_msgs->buf;
			if(i2c_bus->master_msgs->flags & I2C_M_RD) {
				//Rx data
				cmd = AST_I2CD_M_RX_CMD;
				if((i2c_bus->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_bus->master_xfer_cnt == 0)) {
					dev_dbg(i2c_bus->dev, "I2C_M_RECV_LEN \n");
					ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_INTR_CTRL_REG) |
										AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);			

				} else if((i2c_bus->xfer_last == 1) && (i2c_bus->master_xfer_cnt + 1 == i2c_bus->master_msgs->len)) {
					cmd |= AST_I2CD_M_S_RX_CMD_LAST | AST_I2CD_M_STOP_CMD;
	//				disable rx_dwn isr
					ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_INTR_CTRL_REG) &
										~AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
				} else {
					ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_INTR_CTRL_REG) |
										AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);			
				}

				dev_dbg(i2c_bus->dev, "(<--) rx byte, cmd = %x \n",cmd);
				
				ast_byte_i2c_write(i2c_bus, cmd, I2C_CMD_REG);


			} else {
				//Tx data
				dev_dbg(i2c_bus->dev, "(-->) xfer byte data index[%02x]:%02x  \n",i2c_bus->master_xfer_cnt, *(xfer_buf + i2c_bus->master_xfer_cnt));
				ast_byte_i2c_write(i2c_bus, *(xfer_buf + i2c_bus->master_xfer_cnt), I2C_BYTE_BUF_REG);
				if((i2c_bus->xfer_last == 1) && (i2c_bus->master_xfer_cnt + 1 == i2c_bus->master_msgs->len)) {
					ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_INTR_CTRL_REG) &
										~AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);				
					ast_byte_i2c_write(i2c_bus, AST_I2CD_M_TX_CMD | AST_I2CD_M_STOP_CMD, I2C_CMD_REG);
				} else {
					ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);			
					ast_byte_i2c_write(i2c_bus, AST_I2CD_M_TX_CMD, I2C_CMD_REG);
				}
			}
					
		} else {
			//should send next msg 
			if(i2c_bus->master_xfer_cnt != i2c_bus->master_msgs->len)
				printk("CNT ERROR \n");
			
			dev_dbg(i2c_bus->dev, "ast_byte_i2c_do_byte_xfer complete \n");
			i2c_bus->cmd_err = 0;
			complete(&i2c_bus->cmd_complete);		
			
		}
	}
		
}

static void ast_byte_i2c_slave_xfer_done(struct ast_byte_i2c_bus *i2c_bus)
{
	u32 xfer_len = 0;

	if (i2c_bus->slave_msgs->flags & I2C_M_RD) {
		dev_dbg(i2c_bus->dev, "slave tx \n");
		//tx done , only check tx count ...
		xfer_len = 1;
	} else {
		//rx done
		i2c_bus->slave_msgs->buf[i2c_bus->slave_msgs->len] = ast_byte_i2c_read(i2c_bus,I2C_BYTE_BUF_REG) >> 8;
		dev_dbg(i2c_bus->dev, "ast_byte_i2c_slave_xfer_done [%x]\n", i2c_bus->slave_msgs->buf[i2c_bus->slave_msgs->len]);
		i2c_bus->slave_msgs->len++;
	}

}

//TX/Rx Done
static void ast_byte_i2c_master_xfer_done(struct ast_byte_i2c_bus *i2c_bus)
{
	u32 xfer_len = 0;

	dev_dbg(i2c_bus->dev, "ast_byte_i2c_master_xfer_done mode %s\n", i2c_bus->master_msgs->flags & I2C_M_RD ? "read" : "write");	
		
	if (i2c_bus->master_msgs->flags & I2C_M_RD) {
		if(i2c_bus->master_xfer_cnt == -1) {
			xfer_len = 1;
			dev_dbg(i2c_bus->dev, "goto next_xfer \n");
			goto next_xfer;
		}
		if ((i2c_bus->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_bus->blk_r_flag == 0)) {
			i2c_bus->master_msgs->len += (ast_byte_i2c_read(i2c_bus,I2C_BYTE_BUF_REG) & AST_I2CD_RX_BYTE_BUFFER) >> 8; 
			i2c_bus->blk_r_flag = 1;
			dev_dbg(i2c_bus->dev, "I2C_M_RECV_LEN %d \n", i2c_bus->master_msgs->len -1);			
		}
		xfer_len = 1;
		i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt] = (ast_byte_i2c_read(i2c_bus,I2C_BYTE_BUF_REG) & AST_I2CD_RX_BYTE_BUFFER) >> 8;
	} else {
		xfer_len = 1;
	}

next_xfer:

	if(xfer_len != i2c_bus->master_xfer_len) {
		//TODO..
		printk(" ** xfer_len = %d  !=  master_xfer_len = %d  \n", xfer_len, i2c_bus->master_xfer_len);
		//should goto stop....
		i2c_bus->cmd_err = 1;
		goto done_out;
	} else
		i2c_bus->master_xfer_cnt += i2c_bus->master_xfer_len; 

	if(i2c_bus->master_xfer_cnt != i2c_bus->master_msgs->len) {
		dev_dbg(i2c_bus->dev,"do next cnt \n");
		ast_byte_i2c_do_byte_xfer(i2c_bus);
	} else {
#if 0	
		int i;		
		printk(" ===== \n");
		for(i=0;i<i2c_bus->master_msgs->len;i++)
			printk("rx buf i,[%x]\n",i,i2c_bus->master_msgs->buf[i]);
		printk(" ===== \n");	
#endif		
		i2c_bus->cmd_err = 0;

done_out:
		dev_dbg(i2c_bus->dev,"msgs complete \n");
		complete(&i2c_bus->cmd_complete);					
	}
}

#ifdef CONFIG_AST_I2C_SLAVE_MODE	
static void ast_byte_i2c_slave_addr_match(struct ast_byte_i2c_bus *i2c_bus)
{
	u8 match;

	i2c_bus->slave_operation = 1;
	i2c_bus->slave_xfer_cnt = 0;
	match = ast_byte_i2c_read(i2c_bus,I2C_BYTE_BUF_REG) >> 8;
	i2c_bus->slave_msgs->buf[0] = match;
	i2c_bus->slave_msgs->len = 1;
	dev_dbg(i2c_bus->dev, "S Start Addr match [%x] \n",match);

	if(match & 1) {
		i2c_bus->slave_event = I2C_SLAVE_EVENT_START_READ;
	} else {
		i2c_bus->slave_event = I2C_SLAVE_EVENT_START_WRITE;
	}
	ast_byte_i2c_slave_rdwr_xfer(i2c_bus);
	ast_byte_i2c_do_byte_xfer(i2c_bus);

}

static irqreturn_t ast_byte_i2c_slave_handler(struct ast_byte_i2c_bus *i2c_bus)
{
	u32 sts = ast_byte_i2c_read(i2c_bus,I2C_INTR_STS_REG);
	
	if(i2c_bus->bus_config->ast_g5_i2c) {
		if(AST_I2CD_INTR_STS_SLAVE_TO & sts) {
			//drop package			
			dev_err(i2c_bus->dev, "AST_I2CD_INTR_STS_SLAVE_TO M [%x], S [%x] \n", i2c_bus->master_operation, i2c_bus->slave_operation); 			
			i2c_bus->slave_msgs->addr = 0;
			i2c_bus->slave_msgs->len = 0;
			i2c_bus->slave_operation = 0;
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_SLAVE_TO, I2C_INTR_STS_REG);	
			return IRQ_HANDLED;
		}
	}

	if(AST_I2CD_INTR_STS_SDA_DL_TO & sts) {
		//drop package			
		dev_err(i2c_bus->dev, "AST_I2CD_INTR_STS_SDA_DL_TO M [%x], S [%x] \n", i2c_bus->master_operation, i2c_bus->slave_operation);				
		i2c_bus->slave_msgs->addr = 0;
		i2c_bus->slave_msgs->len = 0;			
		i2c_bus->slave_operation = 0;
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_SDA_DL_TO, I2C_INTR_STS_REG); 
		return IRQ_HANDLED;
	}
	
	if(AST_I2CD_INTR_STS_SCL_TO & sts) {
		//drop package 
		dev_err(i2c_bus->dev, "AST_I2CD_INTR_STS_SCL_TO M [%x], S [%x] \n", i2c_bus->master_operation, i2c_bus->slave_operation);		
		i2c_bus->slave_msgs->addr = 0;
		i2c_bus->slave_msgs->len = 0;			
		i2c_bus->slave_operation = 0;
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_SCL_TO, I2C_INTR_STS_REG);	
		return IRQ_HANDLED; 	
	}	

	if(AST_I2CD_INTR_STS_ABNORMAL & sts) {
		//drop package
		dev_err(i2c_bus->dev, "AST_I2CD_INTR_STS_SCL_TO M [%x], S [%x] \n", i2c_bus->master_operation, i2c_bus->slave_operation);		
		i2c_bus->slave_msgs->addr = 0;
		i2c_bus->slave_msgs->len = 0;
		i2c_bus->slave_operation = 0;
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_ABNORMAL, I2C_INTR_STS_REG);	
		return IRQ_HANDLED;
	}	

	if((sts & (AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH)) == (AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH)) {
		if(sts & AST_I2CD_INTR_STS_NORMAL_STOP) {
			I2C_S_DBUG("%d-P|S|W\n",i2c_bus->adap.nr);				
			i2c_bus->slave_event = I2C_SLAVE_EVENT_STOP;
			ast_byte_i2c_slave_rdwr_xfer(i2c_bus);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
			sts &= ~AST_I2CD_INTR_STS_NORMAL_STOP; 
			if(!(ast_byte_i2c_read(i2c_bus,I2C_FUN_CTRL_REG) & AST_I2CD_SLAVE_EN)) {
				printk("full - disable \n");
				return IRQ_HANDLED;
			}
		} 
		ast_byte_i2c_slave_addr_match(i2c_bus);
		dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH = %x\n",sts);
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);	
		sts &= ~(AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH);			
	}
	
	if(!sts) 
		return IRQ_HANDLED;
	
	switch(sts) {
		case AST_I2CD_INTR_STS_TX_ACK:
			i2c_bus->slave_event = I2C_SLAVE_EVENT_READ;
			ast_byte_i2c_slave_xfer_done(i2c_bus);
			dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CD_INTR_STS_TX_ACK = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_TX_ACK, I2C_INTR_STS_REG);
			break;
		case AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP:
			printk("ast_i2c: S	TX_ACK | NORMAL_STOP;  TODO ~~~");
		break;	
		case AST_I2CD_INTR_STS_TX_NAK:
			printk("ast_i2c: S	TX_NAK ~~\n");
			i2c_bus->slave_event = I2C_SLAVE_EVENT_NACK;
			ast_byte_i2c_slave_rdwr_xfer(i2c_bus);
			dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CD_INTR_STS_TX_NAK = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_TX_NAK, I2C_INTR_STS_REG);
			break;
		case AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP:
			printk("SLAVE TODO .... \n");
			break;

		//Issue : Workaround for I2C slave mode
		case AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_SLAVE_MATCH:
			i2c_bus->slave_event = I2C_SLAVE_EVENT_NACK;
			printk("isr AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_SLAVE_MATCH TODO ~~~~~~\n");
			ast_byte_i2c_slave_xfer_done(i2c_bus);
			ast_byte_i2c_slave_addr_match(i2c_bus);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_SLAVE_MATCH , I2C_INTR_STS_REG);
			break;
		case AST_I2CD_INTR_STS_RX_DOWN:
			i2c_bus->slave_event = I2C_SLAVE_EVENT_WRITE;
			ast_byte_i2c_slave_xfer_done(i2c_bus);
			dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CD_INTR_STS_RX_DOWN = %x\n",sts);
			ast_byte_i2c_slave_rdwr_xfer(i2c_bus);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_RX_DOWN, I2C_INTR_STS_REG);
			break;

		case AST_I2CD_INTR_STS_NORMAL_STOP:
			i2c_bus->slave_event = I2C_SLAVE_EVENT_STOP;
			ast_byte_i2c_slave_rdwr_xfer(i2c_bus);
			dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
			dev_dbg(i2c_bus->dev, "state [%x] \n",i2c_bus->state);
			break;
		case (AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP):
			i2c_bus->slave_event = I2C_SLAVE_EVENT_WRITE;
			ast_byte_i2c_slave_xfer_done(i2c_bus);
			dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CD_INTR_STS_RX_DOWN = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_RX_DOWN, I2C_INTR_STS_REG);
			i2c_bus->slave_event = I2C_SLAVE_EVENT_STOP;
			ast_byte_i2c_slave_rdwr_xfer(i2c_bus);
			dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
			break;
		default:
			printk("%d-Slave No one care : isr :%x, state %x\n", i2c_bus->adap.nr, sts, i2c_bus->state);
			break;
	}	
	return IRQ_HANDLED;	
}
#endif
static irqreturn_t ast_byte_i2c_master_handler(struct ast_byte_i2c_bus *i2c_bus)
{
	u32 sts = ast_byte_i2c_read(i2c_bus,I2C_INTR_STS_REG);
	
	if(AST_I2CD_INTR_STS_ARBIT_LOSS & sts) {
		dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_ARBIT_LOSS = %x\n",sts);
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_ARBIT_LOSS, I2C_INTR_STS_REG);
		i2c_bus->cmd_err = AST_I2CD_INTR_STS_ARBIT_LOSS;
		complete(&i2c_bus->cmd_complete);					
		return IRQ_HANDLED;
	}
	
	if((sts & (AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH)) == (AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH)) {
		dev_dbg(i2c_bus->dev, "TOTOTO~~   return nak [%x] \n",sts);
		//return slave nak 
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);			
		sts &= ~(AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH); 
	}
	if(AST_I2CD_INTR_STS_ABNORMAL & sts) {
		printk("M %d-AST_I2CD_INTR_STS_ABNORMAL \n", i2c_bus->adap.nr);
		i2c_bus->cmd_err = AST_I2CD_INTR_STS_ABNORMAL;
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_ABNORMAL, I2C_INTR_STS_REG);
		complete(&i2c_bus->cmd_complete);	
		return IRQ_HANDLED;
	}	
	
	if(AST_I2CD_INTR_STS_SDA_DL_TO & sts) {
		dev_err(i2c_bus->dev, "AST_I2CD_INTR_STS_SDA_DL_TO M [%x], S [%x] \n", i2c_bus->master_operation, i2c_bus->slave_operation);				
		i2c_bus->cmd_err = AST_I2CD_INTR_STS_SDA_DL_TO;
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_SDA_DL_TO, I2C_INTR_STS_REG);
		complete(&i2c_bus->cmd_complete);				
		return IRQ_HANDLED;
	}
	
	if(AST_I2CD_INTR_STS_SCL_TO & sts) {
		dev_err(i2c_bus->dev, "TODO AST_I2CD_INTR_STS_SCL_TO M [%x], S [%x] \n", i2c_bus->master_operation, i2c_bus->slave_operation);		
		i2c_bus->cmd_err = AST_I2CD_INTR_STS_SCL_TO;
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_SCL_TO, I2C_INTR_STS_REG);
		complete(&i2c_bus->cmd_complete);							
		return IRQ_HANDLED; 	
	}	
	
	switch(sts) {
		case AST_I2CD_INTR_STS_TX_ACK:
			dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_TX_ACK = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_TX_ACK, I2C_INTR_STS_REG);
			ast_byte_i2c_master_xfer_done(i2c_bus);
			break;
		case AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP:
			if(i2c_bus->xfer_last) {
				dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP= %x\n",sts);
				ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				//take care
				ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_INTR_CTRL_REG) |
									AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
				ast_byte_i2c_master_xfer_done(i2c_bus);
	
			} else {
				printk("ast_i2c:  TX_ACK | NORMAL_STOP TODO ~~~~\n");
			}
			break;
	
		case AST_I2CD_INTR_STS_TX_NAK:
			dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_TX_NAK = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_TX_NAK, I2C_INTR_STS_REG);
			if(i2c_bus->master_msgs->flags == I2C_M_IGNORE_NAK) {
				dev_dbg(i2c_bus->dev, "I2C_M_IGNORE_NAK next send\n");
				i2c_bus->cmd_err = 0;
			} else {
				dev_dbg(i2c_bus->dev, "NAK error\n");
				i2c_bus->cmd_err = AST_I2CD_INTR_STS_TX_NAK;
			}
			complete(&i2c_bus->cmd_complete);
			break;
	
		case AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP:
			dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_TX_NAK| AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
			dev_dbg(i2c_bus->dev, "M TX NAK | NORMAL STOP \n");
			i2c_bus->cmd_err = AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP;
			complete(&i2c_bus->cmd_complete);
			break;
		case AST_I2CD_INTR_STS_RX_DOWN:
			dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_RX_DOWN = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_RX_DOWN, I2C_INTR_STS_REG);
			ast_byte_i2c_master_xfer_done(i2c_bus);
			break;
	
		case AST_I2CD_INTR_STS_NORMAL_STOP:
			dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
			ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
			i2c_bus->cmd_err = 0;
			complete(&i2c_bus->cmd_complete);
			break;
		case (AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP):
			if(i2c_bus->xfer_last) {	
				dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
				ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				//take care
				ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus,I2C_INTR_CTRL_REG) |
									AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
				ast_byte_i2c_master_xfer_done(i2c_bus);
			} else {
				printk("TODO .. slave .. AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP\n");
			}
			break;
		default:
			printk("%d-Master No one care : isr :%x, state %x, ctrl : %x, M[%d], S[%d] \n", i2c_bus->adap.nr, sts, i2c_bus->state, ast_byte_i2c_read(i2c_bus,I2C_FUN_CTRL_REG), i2c_bus->master_operation, i2c_bus->slave_operation);
			printk("isr ~~~ %x ~~~ \n", ast_byte_i2c_read(i2c_bus,I2C_INTR_STS_REG));
			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) & ~(AST_I2CD_MASTER_EN | AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);
			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
			complete(&i2c_bus->cmd_complete);				
			break;
	}		
	return IRQ_HANDLED; 	
}

static irqreturn_t ast_byte_i2c_handler(int irq, void *dev_id)
{
	struct ast_byte_i2c_bus *i2c_bus = dev_id;
	u32 sts = ast_byte_i2c_read(i2c_bus,I2C_INTR_STS_REG);
	i2c_bus->state = (ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) >> 19) & 0xf;

	dev_dbg(i2c_bus->dev,"ISR : %x\n",sts);

//	dev_dbg(i2c_bus->dev,"sts machine %x, slave_op %d \n", xfer_sts,i2c_bus->slave_operation);
	if(i2c_bus->slave_operation && i2c_bus->master_operation) {
		printk("%d-M-S error !!!! isr : %x, sts : %x ~~~~~~~~~~~~~~~~~~~\n ", i2c_bus->adap.nr, sts, i2c_bus->state);
	}
		
	if(AST_I2CD_INTR_STS_SMBUS_ALT & sts) {
		printk("AST_I2CD_INTR_STS_SMBUS_ALT \n");
		dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_SMBUS_ALT= %x\n",sts);
		//Disable ALT INT
		ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_INTR_CTRL_REG) &
					~AST_I2CD_SMBUS_ALT_INTR_EN,
					I2C_INTR_CTRL_REG);
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_SMBUS_ALT, I2C_INTR_STS_REG);
		ast_master_alert_recv(i2c_bus);
		sts &= ~AST_I2CD_SMBUS_ALT_INTR_EN; 
		return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_BUS_RECOVER & sts) {
		dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CD_INTR_STS_BUS_RECOVER= %x\n",sts);
		ast_byte_i2c_write(i2c_bus, AST_I2CD_INTR_STS_BUS_RECOVER, I2C_INTR_STS_REG);
		i2c_bus->cmd_err = 0;
		if(i2c_bus->bus_recover) {
			complete(&i2c_bus->cmd_complete);				
			i2c_bus->bus_recover = 0;
		} else {
			printk("Error !! Bus revover\n");
		}
		return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_SMBUS_DEF_ADDR & sts) {
		printk("AST_I2CD_INTR_STS_SMBUS_DEF_ADDR TODO \n");
		return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_SMBUS_DEV_ALT & sts) {
		printk("AST_I2CD_INTR_STS_SMBUS_DEV_ALT TODO \n");		
		return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_SMBUS_ARP_ADDR & sts) {
		printk("AST_I2CD_INTR_STS_SMBUS_ARP_ADDR TODO \n");		
		return IRQ_HANDLED;
	}

	sts &= (AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP |
			AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH);

	if(i2c_bus->master_operation) {
		return ast_byte_i2c_master_handler(i2c_bus);
	} else {
#ifdef CONFIG_AST_I2C_SLAVE_MODE
		return ast_byte_i2c_slave_handler(i2c_bus);
#endif		
	}

	return IRQ_HANDLED;

}

static int ast_byte_i2c_do_msgs_xfer(struct ast_byte_i2c_bus *i2c_bus, struct i2c_msg *msgs, int num)
{
	int i;
	int ret = 0;	
	long timeout = 0;

	dev_dbg(i2c_bus->dev, "ast_byte_i2c_do_msgs_xfer\n");
	//request
	for (i=0; i < num; i++) {
		i2c_bus->blk_r_flag = 0;
		i2c_bus->master_msgs = &msgs[i];
		if(num == i+1)
			i2c_bus->xfer_last = 1;
		else
			i2c_bus->xfer_last = 0;

		i2c_bus->blk_r_flag = 0;
		init_completion(&i2c_bus->cmd_complete);
		i2c_bus->cmd_err = 0;

		if(i2c_bus->master_msgs->flags & I2C_M_NOSTART)
			i2c_bus->master_xfer_cnt = 0;
		else
			i2c_bus->master_xfer_cnt = -1;

		ast_byte_i2c_do_byte_xfer(i2c_bus);

		timeout = wait_for_completion_interruptible_timeout(&i2c_bus->cmd_complete,
													   i2c_bus->adap.timeout*HZ);
	
		if (timeout == 0) {
			dev_err(i2c_bus->dev, "controller timed out \n");
			i2c_bus->state = (ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) >> 19) & 0xf;
			//TODO 
			if(ast_byte_i2c_read(i2c_bus,I2C_INTR_STS_REG) & AST_I2CD_INTR_STS_RX_DOWN) {
				goto stop;
			}
			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) & ~AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
			dev_err(i2c_bus->dev, "0x14 [%x] sts [%x], isr sts [%x] \n",ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) , i2c_bus->state, ast_byte_i2c_read(i2c_bus,I2C_INTR_STS_REG));
			dev_err(i2c_bus->dev, "0x00 [%x] M [%x], S [%x] \n",ast_byte_i2c_read(i2c_bus,I2C_FUN_CTRL_REG) , i2c_bus->master_operation, i2c_bus->slave_operation);
			goto out;
		}

		if(i2c_bus->cmd_err != 0) {
//			printk("%d-cmd_err %d \n",i2c_bus->adap.nr, i2c_bus->cmd_err);
			if(i2c_bus->cmd_err == (AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP)) {
				dev_dbg(i2c_bus->dev, "go out \n");
				ret = -ETIMEDOUT;
				goto out;
			} else if (i2c_bus->cmd_err == AST_I2CD_INTR_STS_ABNORMAL) {
				dev_dbg(i2c_bus->dev, "go out \n");
				ret = -ETIMEDOUT;
				goto out;
			} else {
				dev_dbg(i2c_bus->dev, "send stop \n");			
				ret = -EAGAIN;
				goto stop;
			}
		}
		ret++;
	}

	if(i2c_bus->cmd_err == 0)
		goto out;
stop:
	init_completion(&i2c_bus->cmd_complete);
	ast_byte_i2c_write(i2c_bus, AST_I2CD_M_STOP_CMD, I2C_CMD_REG);
	timeout = wait_for_completion_interruptible_timeout(&i2c_bus->cmd_complete,
											   i2c_bus->adap.timeout*HZ);
	if (timeout == 0) {
		dev_dbg(i2c_bus->dev, "send stop timed out\n");
		i2c_bus->state = (ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) >> 19) & 0xf;
		dev_dbg(i2c_bus->dev, "sts [%x], isr sts [%x] \n",i2c_bus->state, ast_byte_i2c_read(i2c_bus,I2C_INTR_STS_REG));
		ret = -ETIMEDOUT;
	}

out:
	//Free ..
	dev_dbg(i2c_bus->dev, "end xfer ret = %d\n",ret);
	return ret;

}

static int ast_byte_i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct ast_byte_i2c_bus *i2c_bus =  i2c_get_adapdata(adap);
	int ret, i;
	
#ifdef CONFIG_AST_I2C_SLAVE_MODE
	u32 ctrl = ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG);
	ctrl &= ~AST_I2CD_SLAVE_EN;

	if(i2c_bus->slave_operation) {
		if(ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) & AST_I2CD_BUS_BUSY_STS) {
			dev_dbg(i2c_bus->dev, "slave mode busy return \n");
			return -EAGAIN;
		} else {
			dev_err(i2c_bus->dev, "slave TODO Check ~~\n");
			i2c_bus->slave_operation = 0;
			i2c_bus->slave_msgs->addr = 0;				
		}
	} 

	ast_byte_i2c_write(i2c_bus, ctrl, I2C_FUN_CTRL_REG);	//will enable when slave rx stop
	i2c_bus->state = (ast_byte_i2c_read(i2c_bus,I2C_CMD_REG) >> 19) & 0xf;
	if (i2c_bus->state) {
		if(!i2c_bus->slave_rx_full)
			ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
		dev_dbg(i2c_bus->dev, "full return \n");
		return -EAGAIN;
	}	
#endif

	i2c_bus->master_operation = 1;

	/* Wait for the bus to become free.  */
	ret = ast_byte_i2c_wait_bus_not_busy(i2c_bus);
	if (ret) {
		dev_dbg(i2c_bus->dev, "i2c_ast: timeout waiting for bus free\n");
		goto out;
	}

	for (i = adap->retries; i >= 0; i--) {
		ret = ast_byte_i2c_do_msgs_xfer(i2c_bus, msgs, num);
		if (ret != -EAGAIN)
			goto out;
		dev_dbg(i2c_bus->dev, "Retrying transmission [%d]\n",i);
		udelay(100);
	}
		
	ret = -EREMOTEIO;
out:
	dev_dbg(i2c_bus->dev, "%d-m: end \n", i2c_bus->adap.nr);
	i2c_bus->master_operation = 0;
#ifdef CONFIG_AST_I2C_SLAVE_MODE	
	if((i2c_bus->slave_en) && (!i2c_bus->slave_rx_full)) {
		ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
	}
#endif	
	I2C_M_DBUG("%d-m-end \n", i2c_bus->adap.nr);
	dev_dbg(i2c_bus->dev, "master _xfer ret %x\n", ret);
//	printk("%d-dwn 0x14[%x] M[%d] S[%d]\n", i2c_bus->adap.nr, ast_byte_i2c_read(i2c_bus,I2C_CMD_REG), i2c_bus->master_operation, i2c_bus->slave_operation);
	return ret;
}

static u32 ast_byte_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm i2c_ast_algorithm = {
	.master_xfer	= ast_byte_i2c_master_xfer,
#ifdef CONFIG_AST_I2C_SLAVE_MODE		
	.slave_xfer		= ast_byte_i2c_slave_ioctl_xfer,
#endif	
	.functionality	= ast_byte_i2c_functionality,
};

static u32 select_i2c_clock(struct ast_byte_i2c_bus *i2c_bus)
{
	int i;
	u32 data;

	for(i = 0; i < i2c_bus->bus_config->timing_table_size; i++) {
		if((i2c_bus->apb_clk / i2c_bus->bus_config->timing_table[i].divisor) < i2c_bus->bus_frequency) {
			break;
		}
	}
	data = i2c_bus->bus_config->timing_table[i].timing;
	//printk("divisor [%d], timing [%x] \n", i2c_bus->bus_config->timing_table[i].divisor, i2c_bus->bus_config->timing_table[i].timing);

	return data;
}

static void ast_byte_i2c_bus_init(struct ast_byte_i2c_bus *i2c_bus)
{
	//I2CG Reset 
	ast_byte_i2c_write(i2c_bus, 0, I2C_FUN_CTRL_REG);

	if(i2c_bus->bus_config->ast_g5_i2c) {
		//ast_g5 soc support 
		ast_byte_i2c_write(i2c_bus, AST_I2CD_BUS_AUTO_RELEASE | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
	} else {
		ast_byte_i2c_write(i2c_bus, AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
	}


	//High SPEED mode
#if 0
	ast_byte_i2c_write(i2c_bus, ast_byte_i2c_read(i2c_bus, I2C_FUN_CTRL_REG) |
						AST_I2CD_M_HIGH_SPEED_EN |
						AST_I2CD_M_SDA_DRIVE_1T_EN |
						AST_I2CD_SDA_DRIVE_1T_EN 
						, I2C_FUN_CTRL_REG);
	
#endif 
	/* Set AC Timing */
	ast_byte_i2c_write(i2c_bus, select_i2c_clock(i2c_bus), I2C_AC_TIMING_REG1);
	ast_byte_i2c_write(i2c_bus, AST_NO_TIMEOUT_CTRL, I2C_AC_TIMING_REG2);
//	ast_byte_i2c_write(i2c_bus, AST_I2C_LOW_TIMEOUT, I2C_AC_TIMING_REG2);
//	ast_byte_i2c_write(i2c_bus, 0x77743335, I2C_AC_TIMING_REG1);

	//Clear Interrupt
	ast_byte_i2c_write(i2c_bus, 0xfffffff, I2C_INTR_STS_REG);

	/* Set interrupt generation of I2C controller */
	if(i2c_bus->bus_config->ast_g5_i2c) {
		ast_byte_i2c_write(i2c_bus,
			AST_I2CD_INTR_STS_SLAVE_TO_EN | 
			AST_I2CD_SDA_DL_TO_INTR_EN |	
			AST_I2CD_BUS_RECOVER_INTR_EN |	
			AST_I2CD_SMBUS_ALT_INTR_EN |
			AST_I2CD_SCL_TO_INTR_EN |
			AST_I2CD_ABNORMAL_INTR_EN |
			AST_I2CD_NORMAL_STOP_INTR_EN |
			AST_I2CD_ARBIT_LOSS_INTR_EN |
			AST_I2CD_RX_DOWN_INTR_EN |
			AST_I2CD_TX_NAK_INTR_EN |
			AST_I2CD_TX_ACK_INTR_EN,
			I2C_INTR_CTRL_REG);
	} else {
		ast_byte_i2c_write(i2c_bus,
			AST_I2CD_SDA_DL_TO_INTR_EN |	
			AST_I2CD_BUS_RECOVER_INTR_EN |	
			AST_I2CD_SMBUS_ALT_INTR_EN |
			AST_I2CD_SCL_TO_INTR_EN |
			AST_I2CD_ABNORMAL_INTR_EN |
			AST_I2CD_NORMAL_STOP_INTR_EN |
			AST_I2CD_ARBIT_LOSS_INTR_EN |
			AST_I2CD_RX_DOWN_INTR_EN |
			AST_I2CD_TX_NAK_INTR_EN |
			AST_I2CD_TX_ACK_INTR_EN,
			I2C_INTR_CTRL_REG);
	}
}

static int ast_byte_i2c_probe(struct platform_device *pdev)
{
	struct ast_byte_i2c_bus *i2c_bus;
	struct resource *res;
	const struct of_device_id *match;
	int bus_nr;
	int ret;

	dev_dbg(&pdev->dev, "ast_byte_i2c_probe \n");

	i2c_bus = devm_kzalloc(&pdev->dev, sizeof(*i2c_bus), GFP_KERNEL);
	if (!i2c_bus) {
		return -ENOMEM;
	}
	i2c_bus->dev = &pdev->dev;
	init_completion(&i2c_bus->cmd_complete);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto free_mem;
	}

	i2c_bus->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!i2c_bus->reg_base) {
		ret = -EIO;
		goto release_mem;
	}

	i2c_bus->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (i2c_bus->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto free_irq;
	}
	
	match = of_match_node(ast_byte_i2c_bus_of_table, pdev->dev.of_node);
	if (!match) {
		ret = -ENOENT;
		goto free_irq;
	}
	i2c_bus->bus_config = (struct ast_i2c_bus_config *)match->data;

	platform_set_drvdata(pdev, i2c_bus);
	
	i2c_bus->clk = devm_clk_get(i2c_bus->dev, NULL);
	if (IS_ERR(i2c_bus->clk)) {
		dev_err(i2c_bus->dev, "no clock defined\n");
		return -ENODEV;
	}
	i2c_bus->apb_clk = clk_get_rate(i2c_bus->clk);
	dev_dbg(i2c_bus->dev, "i2c_bus->apb_clk %d \n", i2c_bus->apb_clk);
	
	ret = of_property_read_u32(pdev->dev.of_node,
				   "bus-frequency", &i2c_bus->bus_frequency);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Could not read bus-frequency property\n");
		i2c_bus->bus_frequency = 100000;
	}



	/* Initialize the I2C adapter */
	i2c_set_adapdata(&i2c_bus->adap, i2c_bus);
	i2c_bus->adap.owner = THIS_MODULE;
	i2c_bus->adap.class = I2C_CLASS_DEPRECATED;
	i2c_bus->adap.algo = &i2c_ast_algorithm;
//	i2c_bus->adap.quirks = &;
	i2c_bus->adap.dev.parent = i2c_bus->dev;
	i2c_bus->adap.timeout = 3;
	i2c_bus->adap.dev.of_node = pdev->dev.of_node;
	i2c_bus->adap.algo_data = i2c_bus;
//	i2c_bus->adap.retries = 0;	
	i2c_bus->adap.retries = 3;	

	/* Try to set the I2C adapter number from dt */
	bus_nr = of_alias_get_id(pdev->dev.of_node, "i2c");
	
	snprintf(i2c_bus->adap.name, sizeof(i2c_bus->adap.name), "ast_i2c.%u",
		 bus_nr);

	i2c_bus->slave_operation = 0;
	i2c_bus->blk_r_flag = 0; 

	ast_byte_i2c_bus_init(i2c_bus);

#ifdef CONFIG_AST_I2C_SLAVE_MODE
	ast_i2c_slave_buff_init(i2c_bus);
#endif

	ret = devm_request_irq(&pdev->dev, i2c_bus->irq, ast_byte_i2c_handler,
			       0, dev_name(&pdev->dev), i2c_bus);
	if (ret) {
		printk(KERN_INFO "I2C: Failed request irq %d\n", i2c_bus->irq);
		goto unmap;
	}


	ret = i2c_add_adapter(&i2c_bus->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto free_irq;
	}
	printk("id == %d %d xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n", pdev->id, pdev->dev.id);

	printk(KERN_INFO "I2C: %s [%d]: AST I2C byte mode adapter [%d khz] \n",
	       pdev->dev.of_node->name, i2c_bus->adap.nr, i2c_bus->bus_frequency/1000);

	return 0;

free_irq:
	free_irq(i2c_bus->irq, i2c_bus);	
unmap:
	iounmap(i2c_bus->reg_base);
release_mem:
	release_mem_region(res->start, resource_size(res));
free_mem:
	kfree(i2c_bus);
	
	return ret;
}

static int ast_byte_i2c_remove(struct platform_device *pdev)
{
	struct ast_byte_i2c_bus *i2c_bus = platform_get_drvdata(pdev);
	struct resource *res;

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&i2c_bus->adap);
	
	free_irq(i2c_bus->irq, i2c_bus);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(i2c_bus->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);

	kfree(i2c_bus);

	return 0;
}

#ifdef CONFIG_PM
static int ast_byte_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	//TODO
	return 0;
}

static int ast_byte_i2c_resume(struct platform_device *pdev)
{
	//TODO
	//Should reset i2c ??? 
	return 0;
}
#endif

MODULE_DEVICE_TABLE(of, ast_byte_i2c_bus_of_table);

static struct platform_driver ast_byte_i2c_bus_driver = {
	.probe		= ast_byte_i2c_probe,
	.remove		= ast_byte_i2c_remove,
#ifdef CONFIG_PM	
	.suspend	= ast_byte_i2c_suspend,
	.resume		= ast_byte_i2c_resume,
#endif	
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = ast_byte_i2c_bus_of_table,
	},
};

module_platform_driver(ast_byte_i2c_bus_driver);


MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED AST I2C Bus Driver");
MODULE_LICENSE("GPL");
