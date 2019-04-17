/*
 *  Aspeed PilotXX I2C Controller.
 *  (C) Copyright 2019 Vishal C Nigade (vishal.nigade@aspeedtech.com)
 *  Copyright (c) 2019, Aspeed Technologies Inc.
 *  SPDX-License-Identifier:     GPL-2.0+
 */
#include "i2c-pilot.h"
static int fifo_depth[ BUS_COUNT ] =
{
#if (BUS_COUNT>=1)
	36,
#endif
#if (BUS_COUNT>=2)
	36,
#endif
#if (BUS_COUNT>=3)
	36,
#endif
#if (BUS_COUNT>=4)
	36,
#endif
#if (BUS_COUNT>=5)
	36,
#endif
#if (BUS_COUNT>=6)
	128,
#endif
#if (BUS_COUNT>=7)
	128,
#endif
#if (BUS_COUNT>=8)
	128,
#endif
#if (BUS_COUNT>=9)
	128,
#endif
#if (BUS_COUNT>=10)
	36
#endif

};



extern void pilot_i2c_disable_interrupt(struct pilot_i2c_bus *, unsigned long);
extern void pilot_i2c_enable_interrupt(struct pilot_i2c_bus *, unsigned long);
extern int i2c_pilot_ii_disable_slave(struct pilot_i2c_bus *);
extern unsigned int Get_Activity_info(struct pilot_i2c_bus *);
extern unsigned int Get_Clk_Streaching_info(struct pilot_i2c_bus *);
irqreturn_t pilot_ii_handler(struct pilot_i2c_bus *);

	static void
i2cTXEMPTY_slave_process(struct pilot_i2c_bus * bus, u32 status)
{
	unsigned volatile int fifo_status ;
#if defined(DEBUG)
	printk("I HAVE ADDED :: i2cTXEMPTY_slave_process\n");
#endif


	fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
	
#if defined(DEBUG)
	printk("I HAVE ADDED :: i2cTXEMPTY_slave_process FIFO STS = %x\n",fifo_status);
#endif
	while( (fifo_status & PILOT_TFNF)&&(bus->Slave_TX_index < bus->Slave_TX_len))
	{
		writel((bus->Slave_TX_data[bus->Slave_TX_index]&0xff) ,bus->base + PILOT_I2C_CMD_REG);
		bus->Slave_TX_index ++;
		fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
	#if defined (DEBUG)
		printk("I HAVE ADDED :: i2cTXEMPTY_slave_process FIFO STS = %x lengh %x index %x\n",fifo_status, bus->Slave_TX_len, bus->Slave_TX_index);
	#endif
	}


	if ( bus->Slave_TX_index >= bus->Slave_TX_len)
	{
		pilot_i2c_disable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);
	}

	return;
}

static void i2cTXEMPTY_master_process(struct pilot_i2c_bus * bus, u32 status)
{
	unsigned volatile int fifo_status ;
#if defined (DEBUG)
	printk("I HAVE ADDED :: i2cTXEMPTY_master_process\n");
#endif
	if(bus->master_xmit_recv_mode_flag==MASTER_XMIT)
	{
		fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
#if defined (DEBUG)
		printk("FIFO status : %x\n",fifo_status);
#endif
		while((fifo_status & PILOT_TFNF)&& (bus->TX_index < bus->TX_len -1))
		{
			writel(bus->TX_data[bus->TX_index],bus->base + PILOT_I2C_CMD_REG);
#if defined (DEBUG)
			printk("DATA : %x\n",bus->TX_data[bus->TX_index]);
#endif
			bus->TX_index ++;
			fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
		}

		if ((fifo_status & PILOT_TFNF) && (bus->TX_index >= bus->TX_len - 1))
		{
			writel((bus->TX_data[bus->TX_index] | PILOT_I2CD_M_STOP_CMD), bus->base + PILOT_I2C_CMD_REG);
			//printk("DATA : %x\n",bus->TX_data[bus->TX_index]);
			bus->TX_index ++;
			pilot_i2c_disable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);
		}
	}


	//Master Reads
	else if(bus->master_xmit_recv_mode_flag == MASTER_RECV)
	{
		fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
		while( (bus->Master_rd_cmd_index < bus->MasterRX_len - 1)&&(fifo_status & PILOT_TFNF) )
		{
			writel(0x100,bus->base + PILOT_I2C_CMD_REG);
			bus->Master_rd_cmd_index++;
			fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
		}

		if((fifo_status & PILOT_TFNF) && (bus->Master_rd_cmd_index >= bus->MasterRX_len -1))
		{
			writel((0x100 | PILOT_I2CD_M_STOP_CMD),bus->base + PILOT_I2C_CMD_REG);
			bus->Master_rd_cmd_index++;
			pilot_i2c_disable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);
		}

	}
	//Restart MWR
	else if(bus->master_xmit_recv_mode_flag == RESTART_MWMR)
	{
		if( bus->TX_index < bus->TX_len)
		{
			fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
			while((fifo_status & PILOT_TFNF) && (bus->TX_index < bus->TX_len))
			{
				writel(bus->TX_data[bus->TX_index], bus->base + PILOT_I2C_CMD_REG);
				bus->TX_index ++;
				fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
			}
		}
		else
		{

			fifo_status = readl(bus->base +PILOT_I2C_STATUS_REG);
			while( (bus->Master_rd_cmd_index<bus->MasterRX_len - 1) && (fifo_status & PILOT_TFNF) )
			{
				writel(0x100, bus->base + PILOT_I2C_CMD_REG);
				bus->Master_rd_cmd_index++;
				fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
			}

			if((fifo_status & PILOT_TFNF) && (bus->Master_rd_cmd_index >= bus->MasterRX_len - 1))
			{
				writel((0x100 | PILOT_I2CD_M_STOP_CMD),bus->base + PILOT_I2C_CMD_REG);
				bus->Master_rd_cmd_index++;
				pilot_i2c_disable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);
			}
		}
	}
	//Restart MRW
	else if( (bus->master_xmit_recv_mode_flag==RESTART_MRMW) )
	{
		if(bus->Master_rd_cmd_index<bus->MasterRX_len)
		{
			fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
			while( (bus->Master_rd_cmd_index<bus->MasterRX_len)&&(fifo_status &PILOT_TFNF) )
			{
				writel(0x100, bus->base + PILOT_I2C_CMD_REG);
				bus->Master_rd_cmd_index++;
				fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
			}
		}
		else
		{
			fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
			while((fifo_status & PILOT_TFNF)&& (bus->TX_index < bus->TX_len - 1))
			{
				writel(bus->TX_data[bus->TX_index], bus->base + PILOT_I2C_CMD_REG);
				bus->TX_index ++;
				fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG);
			}
			if ((fifo_status & PILOT_TFNF) && (bus->TX_index >= bus->TX_len - 1))
			{
				writel((bus->TX_data[bus->TX_index] | PILOT_I2CD_M_STOP_CMD),bus->base +  PILOT_I2C_CMD_REG);
				bus->TX_index ++;
				pilot_i2c_disable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);
			}
		}
	}
	else
	{
		pilot_i2c_disable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);
	}

	return;
}

	static int
i2cRXFULL_master_process(struct pilot_i2c_bus * bus,u32 status)
{
	unsigned int            i;
	unsigned int RxFlr;
#if defined (DEBUG)
	printk("I HAVE ADDED :: i2cRXFULL_master_process\n");
#endif
	RxFlr = readl(bus->base + PILOT_I2C_RXFLR_REG);
#if defined (DEBUG)
	printk("I HAVE ADDED :: i2cRXFULL_master_process %x\n",RxFlr);
#endif
	for(i=0;i<RxFlr;i++)
	{

		bus->MasterRX_data[bus->MasterRX_index] =readl(bus->base + PILOT_I2C_CMD_REG);
#if defined (DEBUG)
	printk("I HAVE ADDED :: i2cRXFULL_master_process rxflr= %x RXFLR=%x data = %x\n",RxFlr,readl(bus->base + PILOT_I2C_RXFLR_REG), bus->MasterRX_data[bus->MasterRX_index]);
#endif

		bus->MasterRX_index ++;
	}

	return 0;
}

static int
i2cRXFULL_slave_process(struct pilot_i2c_bus * bus, u32 status)
{
	unsigned int            i;
	unsigned int RxFlr;
#if defined (DEBUG)
	printk("I HAVE ADDED :: i2cRXFULL_slave_process\n");
#endif

	RxFlr = readl(bus->base + PILOT_I2C_RXFLR_REG);
	for(i=0;i<RxFlr;i++)
	{
		bus->Linear_SlaveRX_data[bus->Linear_SlaveRX_index] =readl(bus->base + PILOT_I2C_CMD_REG);
#if defined (DEBUG)
	printk("I HAVE ADDED :: i2cRXFULL_slave_process rxflr= %x RXFLR=%x data = %x\n",RxFlr,readl(bus->base + PILOT_I2C_RXFLR_REG),bus->Linear_SlaveRX_data[bus->Linear_SlaveRX_index] );
#endif
		bus->Linear_SlaveRX_index++;
	}
	return 0;
}

static void
i2cSTOPDET_slave_process(struct pilot_i2c_bus *bus,u32 status)
{
	int FifoPtr;
	unsigned char *DataBuffer;
	unsigned long length = 0;
	i2cRXFULL_slave_process(bus, status);
#if defined (DEBUG)
	printk("I HAVE ADDED :: i2cSTOPDET_slave_process\n");
#endif

	spin_lock( &bus->data_lock );


	if ((bus->SlaveRX_Entries == MAX_FIFO_LEN)||(bus->Linear_SlaveRX_index==0x0))
	{
		/* Return back */
		spin_unlock( &bus->data_lock );
		return;
	}

	FifoPtr = bus->SlaveRX_Writer;

	bus->SlaveRX_index[FifoPtr] = 0;

	DataBuffer = bus->SlaveRX_data[FifoPtr];
	length = bus->Linear_SlaveRX_index;

	/* First byte of buffer should be filled with slave address */
	DataBuffer[bus->SlaveRX_index[FifoPtr]++] =readl(bus->base + PILOT_I2C_M_SAR) << 1;


	/* Read the Length and oopy to buffer */
	if(length)
		memcpy(&DataBuffer[1],bus->Linear_SlaveRX_data,length);
	bus->SlaveRX_index[FifoPtr] = bus->SlaveRX_len[FifoPtr]= length+1;
	bus->Linear_SlaveRX_index = 0;

	{
		if ((++bus->SlaveRX_Writer) == MAX_FIFO_LEN)
			bus->SlaveRX_Writer = 0;
		bus->SlaveRX_Entries++;

		/* Check if buffer full. If so disable slave */
		if (bus->SlaveRX_Entries == MAX_FIFO_LEN)
		{
			i2c_pilot_ii_disable_slave(bus);
		}
	}
	spin_unlock( &bus->data_lock );
	return;
}

irqreturn_t pilot_ii_handler(struct pilot_i2c_bus * bus)
{
	unsigned long flags;
	unsigned int Txflr=0;
	u32 status;
	u32 fifo_size=fifo_depth[bus->adap.nr];
	u32 M_Rx_Len=bus->MasterRX_len;
	u32 M_Tx_Len=bus->TX_len;
	u32 OP_MODE=bus->master_xmit_recv_mode_flag;

	spin_lock_irqsave( &bus->i2c_irq_lock, flags);

	/* Read interrupt status */
	status = readl( bus->base + PILOT_I2C_INTR_STS_REG );

#if defined (DEBUG)
	printk("%d 0x%x %x\n", bus->adap.nr, status, OP_MODE);
#endif

	if(status & PILOT_I2CD_INTR_RX_OVER)
	{
#if defined (DEBUG)
		printk("ERROR: I2C%d: PILOT_I2CD_INTR_RX_OVER\n",bus->adap.nr);
#endif
		readl(bus->base + PILOT_I2C_CLR_RX_OVER_REG);
	}

	Txflr=readl(bus->base + PILOT_I2C_TXFLR_REG);
#if defined (DEBUG)
	printk(" From Handler Interrupt Status %x\n",status);
#endif
	/* RX_FULL */
	if(status & PILOT_I2CD_INTR_RX_FULL)
	{

		if(OP_MODE == MASTER_RECV)
		{
			if( ( (Txflr<fifo_size)&&(Txflr<M_Rx_Len) )&&(( ( (M_Rx_Len-Txflr)==1)&&((Get_Clk_Streaching_info(bus)&0x6)==0x6) )==0x0)  )
			{
				i2cRXFULL_master_process(bus,status);
			}
		}
		else if( (OP_MODE == RESTART_MWMR)&&(Txflr<M_Rx_Len) )
		{
			i2cRXFULL_master_process(bus,status);
		}
		else if( (OP_MODE == RESTART_MRMW) )
		{
			if( ( (Txflr<fifo_size)&&(Txflr<(M_Rx_Len+M_Tx_Len)) )&&(( ( (M_Rx_Len-Txflr)==1)&&((Get_Clk_Streaching_info(bus)&0x6)==0x6) )==0x0)  ) {
				i2cRXFULL_master_process(bus,status);
			}
		}
		else
		{
			i2cRXFULL_slave_process(bus,status);
		}
	}

	/* PILOT_I2CD_INTR_TX_EMPTY generated as master*/
	if(status & PILOT_I2CD_INTR_TX_EMPTY)
	{
		if(bus->master_xmit_recv_mode_flag == SLAVE_XMIT)
		{
			i2cTXEMPTY_slave_process(bus,status);
		}
		else
		{
			i2cTXEMPTY_master_process(bus,status);
		}
	}

	Txflr=readl(bus->base + PILOT_I2C_TXFLR_REG);
	/* STOP_DET generated */
	if((status & PILOT_I2CD_INTR_STOP_DET) || (status & PILOT_I2CD_INTR_RESTART_DET))
	{
		// The slave can receive this interrupt after a 10-bit Master read phase. In that case just
		// ignore the interrupt
		if ((status & PILOT_I2CD_INTR_RESTART_DET) && ((readl(bus->base +  PILOT_I2C_FUN_CTRL_REG) & PILOT_I2CF_10BITADDR_SLAVE) == PILOT_I2CF_10BITADDR_SLAVE))
		{
			// Check if TX index was 0, RX index was 0 and the FLR was all 0s
			if ((bus->Slave_TX_index == 0) && (bus->Linear_SlaveRX_index ==0) &&
					(readl(bus->base +  PILOT_I2C_TXFLR_REG) == 0) && (readl(bus->base +  PILOT_I2C_RXFLR_REG) == 0) )
			{
				readl(bus->base +  PILOT_I2C_CLR_RESTART_DET_REG);
				spin_unlock_irqrestore(&bus->i2c_irq_lock, flags);
				return IRQ_HANDLED;
			}
		}

		if (status & PILOT_I2CD_INTR_STOP_DET)
			readl(bus->base +  PILOT_I2C_CLR_STOP_DET_REG);
		else
			readl(bus->base +  PILOT_I2C_CLR_RESTART_DET_REG);

		bus->RD_REQ_Pending=0x0;

		if(  ( (OP_MODE == MASTER_RECV)||(OP_MODE== RESTART_MRMW)||(OP_MODE==RESTART_MWMR))&&\
				(Txflr<M_Rx_Len) && ( ( ((M_Rx_Len-Txflr)==1)&& ((Get_Clk_Streaching_info(bus)&0x6)==0x6) )==0x0 )   )
		{
			i2cRXFULL_master_process(bus, status);
			bus->op_status=status;
			bus->master_xmit_recv_mode_flag = SLAVE;
			wake_up( &bus->pilot_i2c_wait );

		}
		else if( ( (OP_MODE==MASTER_XMIT)||(OP_MODE==SLAVE_XMIT)||(OP_MODE==RESTART_MRMW))&&\
				(bus->TX_index >= M_Tx_Len) )
		{
			bus->op_status=status;
			bus->master_xmit_recv_mode_flag = SLAVE;
			wake_up( &bus->pilot_i2c_wait );
		}
		i2cSTOPDET_slave_process(bus,status);
	}

	//RD_REQ handler should be always after stop handler
	//there could be re-start case started by master as soon as we emptied the fifo
	//to get packet demarcation we need to read fifo after moving previous packet.
	if(status & PILOT_I2CD_INTR_RD_REQ)
	{
		bus->RD_REQ_Pending=1;
		readl(bus->base +  PILOT_I2C_CLR_RD_REQ_REG);
		i2cSTOPDET_slave_process(bus,status);
		if(bus->master_xmit_recv_mode_flag == SLAVE_XMIT)
		{
			i2cTXEMPTY_slave_process(bus,status);
			bus->RD_REQ_Pending=0x0;

			if(bus->Slave_TX_len>bus->Slave_TX_index)
			{
				pilot_i2c_enable_interrupt(bus,PILOT_I2CD_INTR_TX_EMPTY);
			}
		}
	}

	/* TX_ABRT geenrated as master */
	if(status & PILOT_I2CD_INTR_TX_ABRT)
	{
		bus->abort_status= readl(bus->base + PILOT_I2C_ABRT_SOURCE_REG );
		readl(bus->base +  PILOT_I2C_CLR_TX_ABRT_REG );
		//printk("I2C%d: Received TX ABORT = 0x%x\n",bus,bus->abort_status);

		pilot_i2c_disable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);
		bus->op_status = status;
		bus->master_xmit_recv_mode_flag = SLAVE;
		wake_up( &bus->pilot_i2c_wait );
	}

	spin_unlock_irqrestore(&bus->i2c_irq_lock, flags);
#if defined (DEBUG)
	printk("Returning from Handler and Status %x \n",status);
#endif
	return IRQ_HANDLED;
}


