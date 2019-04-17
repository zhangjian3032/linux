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


int pilot_i2c_abrt_info(struct pilot_i2c_bus *bus);
int i2c_pilot_ii_disable_slave(struct pilot_i2c_bus *bus);
void pilot_i2c_disable_interrupt(struct pilot_i2c_bus *, unsigned long );
void i2c_init_internal_data(struct pilot_i2c_bus *);
static int perform_pilot_i2c_recovery(struct pilot_i2c_bus *);
void pilot_i2c_hw_reset(struct pilot_i2c_bus *);
void pilot_i2c_enable_interrupt(struct pilot_i2c_bus *, unsigned long );
unsigned int Get_Activity_info(struct pilot_i2c_bus *);
unsigned int Get_Clk_Streaching_info(struct pilot_i2c_bus *);

extern irqreturn_t pilot_ii_handler(struct pilot_i2c_bus * );

#define PILOT_I2C_DEBUG_STS		0x944
#define PILOT_I2C_DEBUG_STS_1		0x978

struct regmap *syscon_i2c_regmap; 

static u32 pilot_read_reg(struct regmap *regm, u32 reg)
{
        unsigned int val = 0;
        int rc = regmap_read(regm, reg, &val);

        if (rc)
                printk("pilot_read_reg Error\n");

        return val;
}

static void pilot_write_reg(struct regmap *regm, u32 data, u32 reg)
{
        int rc = regmap_write(regm, reg, data);

        if (rc)
                printk("pilot_write_reg Error\n");
}



unsigned int Get_Activity_info(struct pilot_i2c_bus * bus)
{
        unsigned int activity=0;
	unsigned int bus_num = bus->adap.nr;
        if ((bus_num >= 0) && (bus_num <= 7))
                activity = pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_DEBUG_STS) >> (bus_num*4);
        else if ((bus_num == 8) || (bus_num ==9))
                activity = pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_DEBUG_STS_1) >> ((bus_num-8)*4);

        activity=(activity&0xf);

        return(activity);
}


unsigned int Get_Clk_Streaching_info(struct pilot_i2c_bus * bus)
{
  unsigned int Clk_Streaching=0;
  unsigned int bus_num = bus->adap.nr;
        if ((bus_num >= 0) && (bus_num <= 7))
                Clk_Streaching = pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_DEBUG_STS) >> (bus_num*4);
        else if ((bus_num == 8) || (bus_num ==9))
                Clk_Streaching = pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_DEBUG_STS_1) >> ((bus_num-8)*4);

  return(Clk_Streaching);

}



static void
update_i2c_pct_tar(struct pilot_i2c_bus *bus,unsigned long tar_ovrrd_mask, unsigned long i2cpct_offset)
{
	volatile unsigned long i2c_pin_ctrl;

	i2c_pin_ctrl = pilot_read_reg(syscon_i2c_regmap , i2cpct_offset);

	/* Set the TAR bit */
	i2c_pin_ctrl |= tar_ovrrd_mask;

	pilot_write_reg(syscon_i2c_regmap , i2c_pin_ctrl, i2cpct_offset);
}

static void
clear_bitbang(struct pilot_i2c_bus * bus,unsigned long bitbang, unsigned long i2cpct_offset)
{
	volatile unsigned long i2c_pin_ctrl;

	i2c_pin_ctrl = pilot_read_reg(syscon_i2c_regmap , i2cpct_offset);

	/* Clear bit-bang */
	i2c_pin_ctrl &= ~(bitbang);

	pilot_write_reg(syscon_i2c_regmap ,i2c_pin_ctrl,  i2cpct_offset);
}


static void pilot_i2c_init_i2cpct(struct pilot_i2c_bus *bus)
{
	unsigned long i2c_tar_bitmask[BUS_COUNT] =
	{(1<<4), (1<<12), (1<<20), (1<<28),
	(1<<4), (1<<12), (1<<4), (1<<12),
	(1<<4), (1<<12)};

	unsigned long i2c_bitbang_bitmask[BUS_COUNT] =
	{(1<<0), (1<<8), (1<<16), (1<<24),
	(1<<0), (1<<8), (1<<0), (1<<8),
	(1<<0), (1<<8)};

        unsigned long pct_regs[BUS_COUNT] =
	{PILOT_I2CPCT0, PILOT_I2CPCT0,
	PILOT_I2CPCT0, PILOT_I2CPCT0,
	PILOT_I2CPCT1, PILOT_I2CPCT1,
	PILOT_I2C_CTRL_2, PILOT_I2C_CTRL_2,
	PILOT_I2C_CTRL_3, PILOT_I2C_CTRL_3};

	update_i2c_pct_tar(bus,i2c_tar_bitmask[bus->adap.nr], pct_regs[bus->adap.nr]);
	clear_bitbang(bus, i2c_bitbang_bitmask[bus->adap.nr], pct_regs[bus->adap.nr]);
}



void pilot_i2c_enable_bus(struct pilot_i2c_bus *bus){
	unsigned short temp=0;
	/* Enable I2C interface */
	temp=readl(bus->base + PILOT_I2C_ENABLE_REG);
	temp = temp | 1;
	writel(temp ,bus->base + PILOT_I2C_ENABLE_REG);
}

void pilot_i2c_disable_bus(struct pilot_i2c_bus *bus){
	unsigned short temp=0;
	/* Disable I2C interface */
	temp=readl(bus->base + PILOT_I2C_ENABLE_REG);
	temp&=0xfffe;
	writel((temp),bus->base + PILOT_I2C_ENABLE_REG);
}

void pilot_i2c_disable_interrupt(struct pilot_i2c_bus *bus, unsigned long mask)
{
  unsigned long current_mask;

  current_mask = readl(bus->base + PILOT_I2C_INTR_CTRL_REG);
  writel((current_mask & ~mask),bus->base + PILOT_I2C_INTR_CTRL_REG);

  return;
}

void pilot_i2c_enable_interrupt(struct pilot_i2c_bus *bus, unsigned long mask)
{
  unsigned long current_mask;

  current_mask = readl(bus->base + PILOT_I2C_INTR_CTRL_REG);
  writel((current_mask | mask),bus->base + PILOT_I2C_INTR_CTRL_REG);
  return;
}


static void reset_i2c_pin_control(struct pilot_i2c_bus * bus, unsigned long bitmask, unsigned long i2cpct_offset)
{
	volatile unsigned long i2c_pin_ctrl;

	i2c_pin_ctrl = pilot_read_reg(syscon_i2c_regmap , i2cpct_offset);

	/* I2C Module Reset, TAR Update without disabling the I2C module */
	i2c_pin_ctrl |= bitmask;

	pilot_write_reg( syscon_i2c_regmap , i2c_pin_ctrl, i2cpct_offset);

	/* This is not needed. But just to ensure that the reset pulse
	 * is active for atleast one clock cycle */
	udelay(1);

	/* Write back 0 in Module Reset bit to complete the reset.
	At the same time set I2C pins controlled by I2C state Machine */
	i2c_pin_ctrl &= ~bitmask;

	pilot_write_reg( syscon_i2c_regmap , i2c_pin_ctrl, i2cpct_offset);
}

void pilot_i2c_hw_reset(struct pilot_i2c_bus * bus)
{
	unsigned long i2c_reset_bitmask[BUS_COUNT] =
	{(1<<7), (1<<15), (1<<23), (1<<31),
		(1<<7), (1<<15), (1<<7), (1<<15),
		(1<<7), (1<<15)};

	unsigned long pct_regs[BUS_COUNT] =
	{PILOT_I2CPCT0, PILOT_I2CPCT0, PILOT_I2CPCT0, PILOT_I2CPCT0,
	PILOT_I2CPCT1, PILOT_I2CPCT1, PILOT_I2C_CTRL_2, PILOT_I2C_CTRL_2,
	PILOT_I2C_CTRL_3, PILOT_I2C_CTRL_3};

	reset_i2c_pin_control(bus, i2c_reset_bitmask[bus->adap.nr], pct_regs[bus->adap.nr]);
}


static void set_1_to_i2c_pin_control_reg0 (struct pilot_i2c_bus * bus,unsigned long bitmask){
        unsigned long i2c_pin_ctrl0;

        i2c_pin_ctrl0 =pilot_read_reg(syscon_i2c_regmap , PILOT_I2CPCT0);

        i2c_pin_ctrl0 |= bitmask;

        pilot_write_reg(syscon_i2c_regmap , i2c_pin_ctrl0, PILOT_I2CPCT0);
}


static void set_0_to_i2c_pin_control_reg0 (struct pilot_i2c_bus * bus, unsigned long bitmask){
        unsigned long i2c_pin_ctrl0;

        i2c_pin_ctrl0 =pilot_read_reg(syscon_i2c_regmap , PILOT_I2CPCT0);

        i2c_pin_ctrl0 &= bitmask;

        pilot_write_reg( syscon_i2c_regmap , i2c_pin_ctrl0, PILOT_I2CPCT0);
}

static void pilot_i2c_bit_bang_deselect(struct pilot_i2c_bus * bus){
	unsigned long i2c0_bitmask = ~(1<<0);
	unsigned long i2c1_bitmask = ~(1<<8);
	if (bus->adap.nr == 0)
		set_0_to_i2c_pin_control_reg0(bus, i2c0_bitmask);

	if (bus->adap.nr == 1)
		set_0_to_i2c_pin_control_reg0(bus, i2c1_bitmask);
}

static void pilot_i2c_bit_bang_select(struct pilot_i2c_bus * bus){
	unsigned long i2c0_bitmask = 1<<0;
	unsigned long i2c1_bitmask = 1<<8;
	if (bus->adap.nr == 0)
		set_1_to_i2c_pin_control_reg0(bus, i2c0_bitmask);

	if (bus->adap.nr == 1)
		set_1_to_i2c_pin_control_reg0(bus, i2c1_bitmask);
}

static void drive_data_low (struct pilot_i2c_bus * bus){

	unsigned long i2c0_bitmask = 1<<5;
	unsigned long i2c1_bitmask = 1<<13;
	if (bus->adap.nr == 0)
		set_1_to_i2c_pin_control_reg0(bus, i2c0_bitmask);

	if (bus->adap.nr == 1)
		set_1_to_i2c_pin_control_reg0(bus, i2c1_bitmask);
}

static void drive_data_high (struct pilot_i2c_bus * bus){

	unsigned long i2c0_bitmask = ~(1<<5);
	unsigned long i2c1_bitmask = ~(1<<13);
	if (bus->adap.nr == 0)
		set_0_to_i2c_pin_control_reg0(bus, i2c0_bitmask);

	if (bus->adap.nr == 1)
		set_0_to_i2c_pin_control_reg0(bus, i2c1_bitmask);
}

static void drive_clock_low (struct pilot_i2c_bus * bus){

	unsigned long i2c0_bitmask = 1<<1;
	unsigned long i2c1_bitmask = 1<<9;
	if (bus->adap.nr == 0)
		set_1_to_i2c_pin_control_reg0(bus,i2c0_bitmask);

	if (bus->adap.nr == 1)
		set_1_to_i2c_pin_control_reg0(bus,i2c1_bitmask);
}

static void drive_clock_high (struct pilot_i2c_bus * bus){

	unsigned long i2c0_bitmask = ~(1<<1);
	unsigned long i2c1_bitmask = ~(1<<9);
	if (bus->adap.nr == 0)
		set_0_to_i2c_pin_control_reg0(bus,i2c0_bitmask);

	if (bus->adap.nr == 1)
		set_0_to_i2c_pin_control_reg0(bus,i2c1_bitmask);
}


static unsigned long get_clk_status(struct pilot_i2c_bus * bus)
{
	unsigned long retval = 0;

	unsigned long i2c0_bitmask = 1<<2;
	unsigned long i2c1_bitmask = 1<<10;

	if (bus->adap.nr == 0)
		retval = pilot_read_reg(syscon_i2c_regmap , PILOT_I2CPCT0) & i2c0_bitmask;

	if (bus->adap.nr == 1)
		retval = pilot_read_reg(syscon_i2c_regmap , PILOT_I2CPCT0) & i2c1_bitmask;


        return retval;
}

static unsigned long get_sda_status(struct pilot_i2c_bus * bus)
{
	unsigned long retval = 0;

	unsigned long i2c0_bitmask = 1<<6;
	unsigned long i2c1_bitmask = 1<<14;

	if (bus->adap.nr == 0)
		retval = pilot_read_reg(syscon_i2c_regmap , PILOT_I2CPCT0) & i2c0_bitmask;

	if (bus->adap.nr == 1)
		retval = pilot_read_reg(syscon_i2c_regmap , PILOT_I2CPCT0) & i2c1_bitmask;


        return retval;
}

static int perform_slave_recovery(struct pilot_i2c_bus * bus)
{
	int pulse_period = 5;
	int num_clock_pulses = DEFAULT_NUM_PULSES;
	
	pulse_period = (int) (500000/DEFAULT_FREQ);

	if (ENABLE_CLOCK_PULSE){
		printk("I2C%d: Slave recovery-Generate Clock Pulses...\n",bus->adap.nr);

		while (num_clock_pulses > 0)
		{
			num_clock_pulses -= 1;

			/* make SCL low */
			drive_clock_low (bus);

			udelay (pulse_period);

			/* make SCL high */
			drive_clock_high (bus);

			udelay (pulse_period);
			if (0 != (get_sda_status(bus)))
			{
				printk("I2C%d: Slave recovery-Bus Recovered\n",bus->adap.nr);
				break;
			}
		}
	}
	if ((0 != get_sda_status(bus))||(0 != get_clk_status(bus)))
	{
		if (DISABLE_FORCE_STOP)
		{
			printk("I2C%d: Slave recovery-Creating Forced STOP Condition\n",bus->adap.nr);

			/* make Data Low */
			drive_data_low(bus);
			/* make SCL high */
			drive_clock_high (bus);

			/* make Data High */
			drive_data_high (bus);

			udelay (5);
		}
	}

	if ((0 != get_sda_status(bus))||(0 != get_clk_status(bus)))
		return 1;

	return 0;
}



static int pilot_i2c_recover_bus(struct pilot_i2c_bus *bus)
{
	unsigned long time_left, flags;
	int ret = 0;
	u32 command;
	int SDAStatusCheckTimes;
	int SCLStatusCheckTimes;
	unsigned long SDAStatus;
	unsigned long SCLStatus;
	int bus_status = 0;

	spin_lock_irqsave(&bus->lock, flags);
	/* Set Data output line so that it's not driven by bit-bang */
	drive_data_high(bus);

	/* Set Clock output line so that it's not driven by bit-bang */
	drive_clock_high(bus);

	/* Set bit bang. Disconnect ourselves from the bus */
	pilot_i2c_bit_bang_select(bus);



	command = get_sda_status(bus);

	SDAStatusCheckTimes = 100;
	while (SDAStatusCheckTimes){
		touch_softlockup_watchdog();
		SDAStatus = get_sda_status(bus);
		if (0 != SDAStatus){
			break;
		}
		else
			SDAStatusCheckTimes--;
		udelay (25);
	}

	SCLStatusCheckTimes = 100;
	while (SCLStatusCheckTimes){
		touch_softlockup_watchdog();
		SCLStatus = get_clk_status(bus);
		if (0 != SCLStatus){
			break;
		}
		else
			SCLStatusCheckTimes--;
		udelay (25);
	}

	 if ((SDAStatusCheckTimes == 0) || (SCLStatusCheckTimes == 0))
          {
            //printk("I2C%d: data or clock low by another device\n",bus);
            /* data is pulled low by a slave on the bus */
            bus_status = perform_slave_recovery(bus);
            if(bus_status != 0)
              {
                //printk("ERROR: I2C%d: Slave recovery did not succeed\n",bus->adap.nr);
                pilot_i2c_bit_bang_deselect(bus);
                return -EIO;
              }
          }

        pilot_i2c_bit_bang_deselect(bus);

	/****** Pilot-I2C recovery *****/
        SDAStatusCheckTimes = 100;
        while (SDAStatusCheckTimes)
        {
                touch_softlockup_watchdog();
                SDAStatus = get_sda_status(bus);
                if (0 != SDAStatus) { break; }
                else SDAStatusCheckTimes--;
                udelay (25);
        }

        SCLStatusCheckTimes = 100;
        while (SCLStatusCheckTimes)
        {
                touch_softlockup_watchdog();
                SCLStatus = get_clk_status(bus);
                if (0 != SCLStatus) { break; }
                else SCLStatusCheckTimes--;
                udelay (25);
        }

        if ((SDAStatusCheckTimes == 0) || (SCLStatusCheckTimes == 0))
        {
                //printk("I2C%d: Recover Pilot-II: Data or Clock is stuck by Pilot-II\n",bus);
                bus_status = perform_pilot_i2c_recovery(bus);
                if(bus_status != 0)
                {
                        //printk("ERROR: I2C%d: Pilot-ii recovery did not succeed\n",bus->adap.nr);
                        return -EIO;
                }
        }

        return 0;
}


#if IS_ENABLED(CONFIG_I2C_SLAVE)


static bool pilot_i2c_slave_irq(struct pilot_i2c_bus *bus)
{
	u32 command, status, status_ack = 0;
	struct i2c_client *slave = bus->slave;
	bool irq_handled = true;
	u8 value;
	unsigned int rxflr=0;
	static unsigned int rxflr_flag_pending = 0;
	unsigned volatile int fifo_status ;
	unsigned long flags;
	spin_lock_irqsave(&bus->lock,flags);
	if (!slave) {
		irq_handled = false;
		goto out;
	}

	if(bus->slave_disable){
		irq_handled = false;
		goto out;
	}	

	status = readl(bus->base + PILOT_I2C_INTR_STS_REG);
#ifdef DEBUG
	printk("I HAVE ADDED::: SLAVE IRQ %d 0x%x txflr = %d rxflr= %d\n", bus->adap.nr, status,readl(bus->base + PILOT_I2C_TXFLR_REG),readl(bus->base + PILOT_I2C_RXFLR_REG));
#endif
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
				goto out1;
			}
		}

		if (status & PILOT_I2CD_INTR_STOP_DET){
			rxflr = readl(bus->base + PILOT_I2C_RXFLR_REG);
			bus->slave_state = PILOT_I2C_SLAVE_STOP;
		}else{
			readl(bus->base +  PILOT_I2C_CLR_RESTART_DET_REG);
			bus->slave_state = PILOT_I2C_SLAVE_START;
		}

#ifdef DEBUG
		printk("I HAVE ADDED::: SLAVE IRQ STOP %d 0x%x SAR = %x\n", bus->adap.nr, status,readl(bus->base + PILOT_I2C_M_SAR));
#endif
	}
out1:
	if( status  & PILOT_I2CD_INTR_RD_REQ){
#ifdef DEBUG
			printk("I HAVE ADDED::: SLAVE IRQ RD_REQ\n");
#endif
		if (bus->slave_state == PILOT_I2C_SLAVE_START)
			bus->slave_state = PILOT_I2C_SLAVE_WRITE_REQUESTED;
		else if (bus->slave_state != PILOT_I2C_SLAVE_READ_PROCESSED)
			bus->slave_state = PILOT_I2C_SLAVE_READ_REQUESTED;
		else
			bus->slave_state = PILOT_I2C_SLAVE_READ_PROCESSED;
	}
	else if((bus->slave_state == PILOT_I2C_SLAVE_START)){
		bus->slave_state = PILOT_I2C_SLAVE_WRITE_REQUESTED;
	}
	else if(bus->slave_state == PILOT_I2C_SLAVE_STOP && rxflr > 1)
		bus->slave_state = PILOT_I2C_SLAVE_WRITE_REQUESTED;
	else if(bus->slave_state == PILOT_I2C_SLAVE_STOP && rxflr == 1)
		bus->slave_state = PILOT_I2C_SLAVE_WRITE_REQUESTED;
	else if(status  & PILOT_I2CD_INTR_RX_FULL){
#ifdef DEBUG
			printk("I HAVE ADDED::: SLAVE IRQ RX_FULL\n");
#endif
		bus->slave_state = PILOT_I2C_SLAVE_WRITE_REQUESTED;
	}
	else
	bus->slave_state = PILOT_I2C_SLAVE_STOP;

	if( status  & PILOT_I2CD_INTR_TX_ABRT){
		pilot_i2c_abrt_info(bus);
	}

	switch(bus->slave_state){
		case PILOT_I2C_SLAVE_READ_REQUESTED:
			bus->slave_state = PILOT_I2C_SLAVE_READ_PROCESSED;
			i2c_slave_event(slave, I2C_SLAVE_READ_REQUESTED, &value);
			writel(value & ~(0x100) ,bus->base + PILOT_I2C_CMD_REG);
			readl(bus->base +  PILOT_I2C_CLR_RD_REQ_REG);
			break;

		case PILOT_I2C_SLAVE_READ_PROCESSED:
			i2c_slave_event(slave, I2C_SLAVE_READ_PROCESSED, &value);
			writel(value & ~(0x100) ,bus->base + PILOT_I2C_CMD_REG);
			readl(bus->base +  PILOT_I2C_CLR_RD_REQ_REG);
			break;

		case PILOT_I2C_SLAVE_WRITE_REQUESTED:
			value = readl(bus->base + PILOT_I2C_CMD_REG);
#ifdef DEBUG
			printk("Addr = %x rxflr = %d\n",value,readl(bus->base + PILOT_I2C_RXFLR_REG));
#endif
			i2c_slave_event(slave, I2C_SLAVE_WRITE_RECEIVED, &value);
			bus->slave_state = PILOT_I2C_SLAVE_WRITE_RECEIVED;

		case PILOT_I2C_SLAVE_WRITE_RECEIVED:
			for (rxflr==0 ?rxflr:rxflr--;rxflr;rxflr--){
				value = readl(bus->base + PILOT_I2C_CMD_REG);
#ifdef DEBUG
				printk("value = 0x%x RXFLR =%d rxflr=%d\n",value,readl(bus->base + PILOT_I2C_RXFLR_REG),rxflr);
#endif

				i2c_slave_event(slave, I2C_SLAVE_WRITE_RECEIVED, &value);
			}
			break;
		case PILOT_I2C_SLAVE_STOP:
			readl(bus->base +  PILOT_I2C_CLR_STOP_DET_REG);
			i2c_slave_event(slave,I2C_SLAVE_STOP, &value);
			bus->slave_state = 11;

#ifdef DEBUG
			printk("I HAVE ADDED::: SLAVE IRQ SENT STOP EVENT\n");
#endif
			break;
		default:
			bus->slave_state = 11;
			dev_err( &bus->adap.dev, "Unkonown Slave state requested\n" );
			break;
	}

out:
	status = readl(bus->base + PILOT_I2C_INTR_STS_REG);

#ifdef DEBUG
	printk("I HAVE ADDED::: RETURNING SLAVE IRQ %d 0x%x %c\n", bus->adap.nr, status, value);
#endif
	spin_unlock_irqrestore(&bus->lock, flags);
	return irq_handled;
}
#endif /* CONFIG_I2C_SLAVE */

int pilot_i2c_abrt_info(struct pilot_i2c_bus *bus)
{
	unsigned int Target;
	unsigned int abrt_source;
	Target = 0x3FF & readl(bus->base + PILOT_I2C_M_TAR);
	abrt_source =  bus->abort_status;


	/* Check for Address NACK */
	if( abrt_source & (PILOT_ABRT_7B_ADDR_NOACK | PILOT_ABRT_10ADDR1_NOACK | PILOT_ABRT_10ADDR2_NOACK))
	{
		dev_err( bus->dev, "NACK on address transmission to 0x%x src:0x%08x\n", Target, abrt_source);
		return( -EREMOTEIO );
	}

	/* Check for NACK */
	if( abrt_source & (PILOT_ABRT_TXDATA_NOACK))
	{
		dev_err( bus->dev, "NACK on data transmission to 0x%x src:0x%08x\n", Target, abrt_source);
		return( -EREMOTEIO );
	}

	/* Check for lost arbitration */
	if( abrt_source & PILOT_ARB_LOST )
	{
		dev_err( bus->dev, "Arbitration lost on data transmission src:0x%08x\n", abrt_source);
		return( -EREMOTEIO );
	}

	/* Check for using disabled Master */
	if( abrt_source & PILOT_ARB_MASTER_DIS )
	{
		dev_err( bus->dev, "Attempted to use disabled Master on data transmission src:0x%08x\n", abrt_source);
		return( -EREMOTEIO );
	}

	/* Check for the master sends a read command in 10bit addressing mode when the restart is disable */
	if( abrt_source & PILOT_ABRT_10B_RD_NORSTRT )
	{
		dev_err( bus->dev, "Master issued a 10-bit Read command with IC_RESTART disabled src:0x%08x\n", abrt_source);
		return( -EREMOTEIO );
	}

	/* Check for the user is trying to send a start byte when the restart is disable */
	if( abrt_source & PILOT_ABRT_SBYTE_NORSTRT )
	{
		dev_err( bus->dev, "start byte sent with restart disabled src:0x%08x\n", abrt_source);
		return( -EREMOTEIO );
	}

	/* Check for the user is trying to use the master to send data in High speed mode when restart is disable */
	if( abrt_source & PILOT_ABRT_HS_NORSTRT )
	{
		dev_err( bus->dev, "Master transmitting in HS mode with IC_RESTART disabled src:0x%08x\n", abrt_source);
		return( -EREMOTEIO );
	}

	/* Check for Master has sent a start byte and teh start byte was acknowledged (wrong behavior) */
	if( abrt_source & PILOT_ABRT_SBYTE_ACKDET )
	{
		dev_err( bus->dev, "Master's start byte was ACK'ed!!! src:0x%08x\n", abrt_source);
		return( -EREMOTEIO );
	}

	/* Check for Master is in high speed mode and the high speed master code was acknowledged */
	if( abrt_source & PILOT_ABRT_HS_ACKDET )
	{
		dev_err( bus->dev, "HS Master code was ACK'ed src:0x%08x\n", abrt_source);
		return( -EREMOTEIO );
	}

	/* Check for Master sent a general call but the user programmed the byte following the G.Call to be read rom the bus*/
	if( abrt_source & PILOT_ABRT_GCALL_READ )
	{
		dev_err( bus->dev, "Master sent a general call but the user programmed the byte following the G.Call to be read from the bus  on data transmission\n" );
		return( -EREMOTEIO );
	}

	/* Check for Master sent a general call and no slave on the bus responded with an ack*/
	if( abrt_source & PILOT_ABRT_GCALL_NOACK )
	{
		dev_err( bus->dev, "Master sent a general call but and no slave on the bus responded with an ack on data transmission\n" );
		return( -EREMOTEIO );
	}

	dev_err( bus->dev, "INTERNAL ERROR: Unknown Abort Source %x\n",abrt_source);
	return (-EREMOTEIO);
}


static irqreturn_t pilot_i2c_bus_irq(int irq, void *dev_id)
{
	struct pilot_i2c_bus *bus = dev_id;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	if (pilot_i2c_slave_irq(bus)) {
		dev_dbg(bus->dev, "irq handled by slave.\n");
		return IRQ_HANDLED;
	}
#endif /* CONFIG_I2C_SLAVE */

	return pilot_ii_handler(bus) ? IRQ_HANDLED : IRQ_NONE;
}

static int i2c_wait_for_bus_free(struct pilot_i2c_bus *bus){
	int   retries = DEFAULT_BB_RETRIES * DEFAULT_TIMEOUT;

	readl(bus->base + PILOT_I2C_CLR_ACTIVITY_REG);

	while ( (readl(bus->base + PILOT_I2C_STATUS_REG) & PILOT_I2C_STATUS_ACTIVITY ) && (--retries  > 0))
	{
		readl(bus->base + PILOT_I2C_CLR_ACTIVITY_REG);
		mdelay (1);
	}
	if(retries == 0)
	{
		return 1;
	}

	return 0;
}

static u32 pilot_i2c_wait_for_int( struct pilot_i2c_bus *bus , int ms_timeout)
{
	if (wait_event_timeout(bus->pilot_i2c_wait,bus->op_status,(msecs_to_jiffies(ms_timeout) ) ) == 0)
	{
		bus->op_status = readl( bus->base + PILOT_I2C_INTR_STS_REG );
	}
	return bus->op_status;
}

static int pilot_i2c_send( struct pilot_i2c_bus *bus,
		struct i2c_msg msg, int messages_left )
{
	int status;
	int i;

	for (i=0; i<msg.len; i++){
		bus->TX_data[i] = msg.buf[i];

#ifdef DEBUG
		printk("send bytes %x lengh %d\n",msg.buf[i],msg.len);
#endif
	}
	bus->TX_len = msg.len;
	bus->TX_index = 0;

	bus->master_xmit_recv_mode_flag = MASTER_XMIT;

	/* Unmask TX empty interrupt */
	bus->op_status = 0;

	pilot_i2c_enable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);
	/* Wait for interrupt */

	status = pilot_i2c_wait_for_int( bus, (msg.len*300)+3000);


	bus->master_xmit_recv_mode_flag = SLAVE;

	pilot_i2c_disable_interrupt(bus,PILOT_I2CD_INTR_TX_EMPTY);

	/* Check for signal pending */
	if( status == PILOT_II_SIGNAL_RECEIVED )
	{
		//dev_info( &i2c_adap->dev, "send_bytes: Signal pending\n" );
		return( -ERESTARTSYS );
	}

	if (status & PILOT_I2CD_INTR_TX_ABRT)
		return pilot_i2c_abrt_info(bus);

	/* Check for timeout */
	if( status == 0 )
	{
		dev_err( &bus->adap.dev, "master send_bytes: Timed out sending data\n" );
		dev_err( &bus->adap.dev, "master send_bytes: Got status: 0x%08x\n", status );
		return( -EREMOTEIO );
	}

	return( 0 );
}

static int pilot_i2c_receive_bytes( struct pilot_i2c_bus *bus,
		struct i2c_msg msg, int messages_left )
{
	DEFINE_SPINLOCK(se_temp_lock);
	unsigned long		flags;
	int status;
	int i;
	int fifo_size=fifo_depth[bus->adap.nr];

	//printk("%s %x\n",__FUNCTION__, msg.len);
	if(msg.len<=0x0)
	{
		return( -ERESTARTSYS );
	}

	if((readl(bus->base + PILOT_I2C_RXFLR_REG)))
	{
		return( -EREMOTEIO );
	}

	bus->MasterRX_index = 0;
	bus->MasterRX_len = msg.len;
	bus->Master_rd_cmd_index=0x0;
	bus->TX_len = 0;
	bus->TX_index=0x0;

	bus->op_status = 0;

	//wait if our slave is busy
	// while(Get_Activity_info(i2c_adap->nr)&0x1);
	//  while(i2c_pilot_ii_read_reg(i2c_adap->nr,I2C_RXFLR_REG));

	spin_lock_irqsave( &se_temp_lock,flags);

	bus->master_xmit_recv_mode_flag = MASTER_RECV;

	if(msg.len<=fifo_size)
	{
		/* send read command */
		for(i=0; i < msg.len -1; i++)
		{
			writel(0x100, bus->base + PILOT_I2C_CMD_REG);
			bus->Master_rd_cmd_index++;
		}

		// Program the stop bit
		writel((0x100 | PILOT_I2CD_M_STOP_CMD), bus->base + PILOT_I2C_CMD_REG);
		bus->Master_rd_cmd_index++;
	}
	else
	{
		//send read cmds
		for(i = 0; i<fifo_size;i++)
		{
			writel(0x100, bus->base + PILOT_I2C_CMD_REG);
			bus->Master_rd_cmd_index++;
		}

		pilot_i2c_enable_interrupt(bus,PILOT_I2CD_INTR_TX_EMPTY);
	}
	spin_unlock_irqrestore(&se_temp_lock, flags);


	/* Wait for interrupt */
	status = pilot_i2c_wait_for_int(bus,(msg.len*300)+3000);

	bus->master_xmit_recv_mode_flag = SLAVE;

	/* Check for signal pending */
	if( status == PILOT_II_SIGNAL_RECEIVED )
	{
		dev_info( &bus->adap.dev, "pilot_i2c_receive_bytes: Signal pending\n" );
		return( -ERESTARTSYS );
	}

	if (status & PILOT_I2CD_INTR_TX_ABRT)
		return pilot_i2c_abrt_info(bus);

	/* check if the receive buffer is empty */
	if(  status & PILOT_I2CD_INTR_RX_UNDER )
	{
		/* read this register to clear RX_UNDER interrupt */
		readl(bus->base + PILOT_I2C_CLR_RX_UNDER_REG);
		/* Got empty buffer when read */
		dev_err( &bus->adap.dev, "pilot_i2c_receive_bytes: Empty receiver buffer on data read\n" );
		return( -EREMOTEIO );
	}

	/* check if the receive buffer is completely filled and more data arrived */
	if(  status & PILOT_I2CD_INTR_RX_OVER )
	{
		/* read this register to clear RX_OVER interrupt */
		readl(bus->base + PILOT_I2C_CLR_RX_OVER_REG);
		dev_err( &bus->adap.dev, "pilot_i2c_receive_bytes: The receiver buffer is completely filled, more data arrived is lost\n" );
		return( -EREMOTEIO );
	}

	if( status == 0 )
	{
		dev_err( &bus->adap.dev, "pilot_i2c_receive_bytes: Timed out receiving data\n" );
		dev_err( &bus->adap.dev, "pilot_i2c_receive_bytes: Got status: 0x%08x\n", status );
		//printk("rd cmds %d \n",bus->Master_rd_cmd_index);
		return( -EREMOTEIO );
	}

	for ( i = 0; i<msg.len; i++)
		msg.buf[i] = bus->MasterRX_data[i];

	return( 0 );
}


static int send_slave_addr( struct pilot_i2c_bus *bus, int addr, int flags )
{
	u16 convalue;
	convalue = readl(bus->base + PILOT_I2C_FUN_CTRL_REG);

	pilot_i2c_disable_bus(bus);

	/* Check for 10-bit slave addresses  */
	if( flags & I2C_M_TEN )
	{
		addr |= PILOT_I2CT_10BITADDR_MASTER;

		// Check if 10-bit addressing mode is already set
		if (convalue & PILOT_I2CF_10BITADDR_MASTER)
			goto PROGRAM_TAR;

		convalue |= PILOT_I2CF_10BITADDR_MASTER;
		/* Write addressing mode back to control register */
		writel(convalue,bus->base + PILOT_I2C_FUN_CTRL_REG);
	}
	else
	{
		addr &= ~PILOT_I2CT_10BITADDR_MASTER;

		// Check if 10-bit addressing mode is disabled
		if ((convalue & PILOT_I2CF_10BITADDR_MASTER) == 0)
			goto PROGRAM_TAR;

		convalue &= ~PILOT_I2CF_10BITADDR_MASTER;
		writel(convalue,bus->base + PILOT_I2C_FUN_CTRL_REG);
	}

PROGRAM_TAR:

	/* Program TAR register */
	writel(addr, bus->base + PILOT_I2C_M_TAR);
	pilot_i2c_enable_bus(bus);
	return 0;
}


static int pilot_i2c_master_xfer(struct i2c_adapter *adap,
		struct i2c_msg *msgs, int num)
{
	struct pilot_i2c_bus *bus = i2c_get_adapdata(adap);

	int i = 0;
	//  	int j = 0;
	int retval = 0;
	int status;
	unsigned long	flags;
	int fifo_size=fifo_depth[bus->adap.nr];
	unsigned volatile int fifo_status=0x0;
	bus->MasterRX_len = 0x0;
	bus->MasterRX_index = 0x0;
	bus->Master_rd_cmd_index=0x0;

	bus->TX_len = 0;
	bus->TX_index=0x0;

	/* Check if bus busy */
	if(i2c_wait_for_bus_free(bus) == 1)
	{
		/* Bus busy. Perform a recovery */
		if(pilot_i2c_recover_bus(bus) != 0)
		{
			printk("I2C%d: Master-Xfer failed. Bus busy\n",bus->adap.nr);
			return -EIO;
		}
	}
	bus->slave_disable = true;
	/* Loop across all the messages */
	/* Send the destination slave address onto the bus */

	//clear any abort status set from prior transactions	
	bus->abort_status=0x0;
	if(num == 1)
	{
#ifdef DEBUG
		printk("number of message %d \n",num);
#endif
		for (i=0;i<num;i++)
		{
			retval = send_slave_addr(bus, msgs[i].addr, msgs[i].flags);
			if(retval != 0){
				bus->slave_disable = false;
				return -EIO;
			}

			if (msgs[i].flags & I2C_M_RD)
				retval = pilot_i2c_receive_bytes(bus, msgs[i], num-(i+1));
			else
				retval = pilot_i2c_send(bus, msgs[i],num-(i+1));
		}

		bus->slave_disable = false;
		return(retval == 0?i:retval);
	}


	/* ????????????????? */
	if (num != 2)
	{
#ifdef DEBUG
		printk("number of message %d \n",num);
#endif
		bus->master_xmit_recv_mode_flag = SLAVE;
		bus->slave_disable = false;
		return (-EINVAL);
	}

	/* ---------------Num == 2  Repeated Start Condition --------------------------*/


	retval = send_slave_addr(bus, msgs[0].addr, msgs[0].flags);
	if(retval != 0){
		bus->slave_disable = false;
		return -EIO;
	}
#ifdef DEBUG
	printk("slave address : %x\n",msgs[0].addr);
#endif
	if (msgs[1].len == 0){
		retval = pilot_i2c_send(bus, msgs[0],0);
		bus->slave_disable = false;
		return(retval == 0?2:retval);
	}
	else if (msgs[1].flags & I2C_M_RD)
	{
		//MASTER WRITE-MASTER READ RESTART CASE
		/* set RX buffer len as requested bytes lengh */
#ifdef DEBUG
		printk("number of message %d \n",num);
#endif
		bus->MasterRX_len = msgs[1].len;
		bus->MasterRX_index = 0;
			bus->Master_rd_cmd_index=0x0;

		bus->op_status = 0;

		//wait if our slave is busy
		//	    while(Get_Activity_info(bus));
		//while(readl(bus->base + PILOT_I2C_RXFLR_REG));

		if(readl(bus->base + PILOT_I2C_RXFLR_REG))
		{
		bus->slave_disable = false;
			return( -EREMOTEIO );
		}


		spin_lock_irqsave(&bus->lock,flags);
		bus->master_xmit_recv_mode_flag = RESTART_MWMR;


#ifdef DEBUG
		printk("lengh %d \n",msgs[1].len);
#endif
		if(msgs[0].len<fifo_size)
		{
			//send write data
			for (i = 0; i<msgs[0].len;i++){
				writel(msgs[0].buf[i], bus->base + PILOT_I2C_CMD_REG);
#ifdef DEBUG
		printk("0 lengh %d %x\n",msgs[0].len, msgs[0].buf[i]);
#endif
			}

			bus->TX_len = msgs[0].len;
			bus->TX_index = msgs[0].len;

			
			/* send read command , program STOPBIT along with last read command*/
			fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG); 
			while( ((bus->Master_rd_cmd_index) < (bus->MasterRX_len - 1))&&(fifo_status &PILOT_TFNF) )
			{
				writel(0x100, bus->base + PILOT_I2C_CMD_REG);
				bus->Master_rd_cmd_index++;
				fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG); 
			}
			if ((bus->Master_rd_cmd_index == bus->MasterRX_len - 1) && (fifo_status &PILOT_TFNF))
			{
				writel((0x100 | PILOT_I2CD_M_STOP_CMD), bus->base + PILOT_I2C_CMD_REG);
				bus->Master_rd_cmd_index++;
#ifdef DEBUG
				printk("SENT STOP BIT\n");
#endif
			}
			if(bus->Master_rd_cmd_index < bus->MasterRX_len)
			{
				pilot_i2c_enable_interrupt(bus,PILOT_I2CD_INTR_TX_EMPTY);
			}

		}
		else
		{
			//send write cmd
			for (i = 0; i<fifo_size;i++){
				writel( msgs[0].buf[i], bus->base + PILOT_I2C_CMD_REG);
#ifdef DEBUG
		printk("cmd 0 lengh %d %x\n",msgs[0].len, msgs[0].buf[i]);
#endif
			}

			for (i=0; i<(msgs[0].len-fifo_size); i++)
				bus->TX_data[i] = msgs[0].buf[i+fifo_size];


			bus->TX_len = msgs[0].len-fifo_size;
			bus->TX_index = 0x0;

			pilot_i2c_enable_interrupt(bus,PILOT_I2CD_INTR_TX_EMPTY);
		}

		spin_unlock_irqrestore(&bus->lock, flags);

		
  		unsigned int current_mask = readl(bus->base + PILOT_I2C_INTR_STS_REG);
		status = pilot_i2c_wait_for_int(bus,(msgs[1].len*300)+3000);
		bus->master_xmit_recv_mode_flag = SLAVE;
	}
	else
	{
		//MASTER READ-MASTER-WRITE-RE-START-CASE
		/* set RX buffer len as requested bytes lengh */
		//printk(" message flag else %d \n",msgs[1].flags);
		bus->MasterRX_len = msgs[0].len;
		bus->MasterRX_index = 0;
		bus->Master_rd_cmd_index=0x0;

		bus->TX_len = msgs[1].len;
		bus->TX_index = 0x0;

		for (i=0; i<msgs[1].len; i++)
			bus->TX_data[i] = msgs[1].buf[i];

		bus->op_status = 0;

		//wait if our slave is busy
		//while(Get_Activity_info(bus));
		//while(readl(bus->base + PILOT_I2C_RXFLR_REG));
		if((readl(bus->base + PILOT_I2C_RXFLR_REG)))
		{
		bus->slave_disable = false;
			return( -EREMOTEIO );
		}


		spin_lock_irqsave(&bus->lock,flags);
		bus->master_xmit_recv_mode_flag = RESTART_MRMW;

		if(msgs[0].len<fifo_size)
		{
			/* send read command */
			for(i = 0; i<msgs[0].len;i++)
			{
				writel(0x100, bus->base + PILOT_I2C_CMD_REG);
				bus->Master_rd_cmd_index++;
			}
			fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG); 

			// Program STOP_BIT along with last byte to be written
			while((fifo_status & PILOT_TFNF)&& (bus->TX_index < bus->TX_len - 1))
			{
				writel(bus->TX_data[bus->TX_index],bus->base + PILOT_I2C_CMD_REG);
				bus->TX_index ++;
				fifo_status = readl(bus->base + PILOT_I2C_STATUS_REG); 
			}
			if ((fifo_status & PILOT_TFNF)&& (bus->TX_index == bus->TX_len - 1))
			{
				writel(((bus->TX_data[bus->TX_index]) | PILOT_I2CD_M_STOP_CMD), 
						bus->base + PILOT_I2C_CMD_REG);
				bus->TX_index++;
			}
			if(bus->TX_index < bus->TX_len)
			{
				pilot_i2c_enable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);
			}
		}
		else
		{
			/* send read command */
			for(i =0; i<fifo_size;i++)
			{
				writel(0x100, bus->base + PILOT_I2C_CMD_REG);
				bus->Master_rd_cmd_index++;
			}
		}
		pilot_i2c_enable_interrupt(bus, PILOT_I2CD_INTR_TX_EMPTY);

		spin_unlock_irqrestore(&bus->lock, flags);


		status = pilot_i2c_wait_for_int(bus,(msgs[1].len*300)+3000);
		
		//printk("interrupt satus else %d \n",status);
		bus->master_xmit_recv_mode_flag = SLAVE;
	}



	/* Check for signal pending */
	if( status == PILOT_II_SIGNAL_RECEIVED )
	{
		//dev_info( &i2c_adap->dev, "send_bytes: Signal pending\n" );
		bus->slave_disable = false;
		return( -ERESTARTSYS );
	}

	if (status & PILOT_I2CD_INTR_TX_ABRT)	
	{
		bus->slave_disable = false;
		return pilot_i2c_abrt_info(bus);
	}

	/* Check for timeout */
	if( status == 0 )
	{
		dev_err( &bus->adap.dev, "pilot_ii_repeated start: Timed out sending data\n" );
		//printk("rd_cmds %d \n",bus->Master_rd_cmd_index);
		spin_unlock_irqrestore(&bus->lock, flags);
		bus->slave_disable = false;
		return( -EREMOTEIO );
	}


	/* check if the receive buffer is empty */
	if(  status & PILOT_I2CD_INTR_RX_UNDER )
	{
		/* read this register to clear RX_UNDER interrupt */
		readl(bus->base + PILOT_I2C_CLR_RX_UNDER_REG);
		dev_err( &bus->adap.dev, "pilot_ii_repeated_start: Empty receiver buffer on data read\n" );
		bus->slave_disable = false;
		return( -EREMOTEIO );
	}


	/* check if the receive buffer is completely filled and more data arrived */
	if(  status & PILOT_I2CD_INTR_RX_OVER )
	{
		/* read this register to clear RX_OVER interrupt */
		readl(bus->base + PILOT_I2C_CLR_RX_OVER_REG);
		dev_err( &bus->adap.dev, "pilot_ii_repeated_start: The receiver buffer is completely filled, more data arrived is lost\n" );
		bus->slave_disable = false;
		return( -EREMOTEIO );
	}
	if (msgs[1].flags & I2C_M_RD)
	{
		for ( i = 0; i<msgs[1].len; i++){
			msgs[1].buf[i] = bus->MasterRX_data[i];
#ifdef DEBUG
	printk("Kernel Done, Passing to Application Layer read : %x\n",msgs[1].buf[i]);
#endif
		}

	}
	else
	{
		for ( i = 0; i<msgs[0].len; i++)
		{
			msgs[0].buf[i] = bus->MasterRX_data[i];
#ifdef DEBUG
	printk("Kernel Done, Passing to Application Layer write : %x\n", msgs[0].buf[i]);
#endif
		}
	}
	bus->slave_disable = false;
	return(num);
}




static u32 pilot_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
/* precondition: bus.lock has been acquired. */
static void __pilot_i2c_reg_slave(struct pilot_i2c_bus *bus, u16 slave_addr)
{
	u32 addr_reg_val, func_ctrl_reg_val;

	/* Set slave addr. */
	writel(slave_addr , bus->base + PILOT_I2C_M_SAR);

	/* Turn on slave mode. */
	func_ctrl_reg_val = readl(bus->base + PILOT_I2C_FUN_CTRL_REG);
	func_ctrl_reg_val &= ~PILOT_I2CF_SLAVE_EN;
	writel(func_ctrl_reg_val, bus->base + PILOT_I2C_FUN_CTRL_REG);
	bus->slave_disable = false;
	printk("Slave address registered\n");
}

static int pilot_i2c_reg_slave(struct i2c_client *client)
{
	// unsigned int temp=0;
	/* Save the slave address for future uses */
	printk("Slave address %X\n", client->addr);
	struct pilot_i2c_bus *bus = i2c_get_adapdata(client->adapter);
	unsigned long flags;
	spin_lock_irqsave(&bus->lock, flags);

	/* Disable I2C interface first */
	pilot_i2c_disable_bus(bus);

	/* set slave mode */
	__pilot_i2c_reg_slave(bus, client->addr);
	send_slave_addr(bus,client->addr,client->flags);
	bus->slave = client;
	bus->slave_state = PILOT_I2C_SLAVE_STOP;

	/* Enable I2C interface */
	pilot_i2c_enable_bus(bus);
	spin_unlock_irqrestore(&bus->lock, flags);
	return 0;
}

#if 0
static int pilot_i2c_reg_slave(struct i2c_client *client)
{
	struct pilot_i2c_bus *bus = i2c_get_adapdata(client->adapter);
	unsigned long flags;

	spin_lock_irqsave(&bus->lock, flags);
	if (bus->slave) {
		spin_unlock_irqrestore(&bus->lock, flags);
		return -EINVAL;
	}

	__pilot_i2c_reg_slave(bus, client->addr);

	bus->slave = client;
	bus->slave_state = PILOT_I2C_SLAVE_STOP;
	spin_unlock_irqrestore(&bus->lock, flags);

	return 0;
}
#endif
static int pilot_i2c_unreg_slave(struct i2c_client *client)
{
	struct pilot_i2c_bus *bus = i2c_get_adapdata(client->adapter);
	u32 func_ctrl_reg_val;
	unsigned long flags;

	spin_lock_irqsave(&bus->lock, flags);
	if (!bus->slave) {
		spin_unlock_irqrestore(&bus->lock, flags);
		return -EINVAL;
	}

	/* Turn off slave mode. */
	func_ctrl_reg_val = readl(bus->base + PILOT_I2C_FUN_CTRL_REG);
	func_ctrl_reg_val |= PILOT_I2CF_SLAVE_EN;
	writel(func_ctrl_reg_val, bus->base + PILOT_I2C_FUN_CTRL_REG);

	bus->slave = NULL;
	spin_unlock_irqrestore(&bus->lock, flags);

	return 0;
}
#endif /* CONFIG_I2C_SLAVE */

static const struct i2c_algorithm pilot_i2c_algo = {
	.master_xfer	= pilot_i2c_master_xfer,
	.functionality	= pilot_i2c_functionality,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave	= pilot_i2c_reg_slave,
	.unreg_slave	= pilot_i2c_unreg_slave,
#endif /* CONFIG_I2C_SLAVE */
};

static u32 pilot_i2c_get_clk_val(u32 clk_high_low_max, u32 divisor)
{
	//printk("Unused part has doubt\n");
	u32 base_clk, clk_high, clk_low, tmp;

	/*
	 * The actual clock frequency of SCL is:
	 *	SCL_freq = APB_freq / (base_freq * (SCL_high + SCL_low))
	 *		 = APB_freq / divisor
	 * where base_freq is a programmable clock divider; its value is
	 *	base_freq = 1 << base_clk
	 * SCL_high is the number of base_freq clock cycles that SCL stays high
	 * and SCL_low is the number of base_freq clock cycles that SCL stays
	 * low for a period of SCL.
	 * The actual register has a minimum SCL_high and SCL_low minimum of 1;
	 * thus, they start counting at zero. So
	 *	SCL_high = clk_high + 1
	 *	SCL_low	 = clk_low + 1
	 * Thus,
	 *	SCL_freq = APB_freq /
	 *		((1 << base_clk) * (clk_high + 1 + clk_low + 1))
	 * The documentation recommends clk_high >= clk_high_max / 2 and
	 * clk_low >= clk_low_max / 2 - 1 when possible; this last constraint
	 * gives us the following solution:
	 */
	base_clk = divisor > clk_high_low_max ?
		ilog2((divisor - 1) / clk_high_low_max) + 1 : 0;
	tmp = (divisor + (1 << base_clk) - 1) >> base_clk;
	clk_low = tmp / 2;
	clk_high = tmp - clk_low;

	if (clk_high)
		clk_high--;

	if (clk_low)
		clk_low--;
	return 400;
#if 0
		return ((clk_high << PILOT_I2CD_TIME_SCL_HIGH_SHIFT)
			& PILOT_I2CD_TIME_SCL_HIGH_MASK)
		| ((clk_low << PILOT_I2CD_TIME_SCL_LOW_SHIFT)
				& PILOT_I2CD_TIME_SCL_LOW_MASK)
		| (base_clk & PILOT_I2CD_TIME_BASE_DIVISOR_MASK);
#endif		
}

static u32 pilot_i2c_get_clk_reg_val(u32 divisor)
{
	/*
	 * clk_high and clk_low are each 3 bits wide, so each can hold a max
	 * value of 8 giving a clk_high_low_max of 16.
	 */
	//printk("Getting A clock Value\n");
	return pilot_i2c_get_clk_val(16, divisor);
}
/*used for bus 6,7 ,but argument bus should be 0,1*/
int i2c_pilot_ii_disable_slave(struct pilot_i2c_bus *bus)
{
	u32 ctrl_bits;
	unsigned long flags;

	if(i2c_wait_for_bus_free(bus) != 0)
	{
		printk("ERROR: I2C%d: Disable slave failed due to bus-busy\n",bus);
		return 1;
	}

	local_save_flags(flags);
	local_irq_disable();

	ctrl_bits = readl(bus->base + PILOT_I2C_FUN_CTRL_REG);
	if (!(ctrl_bits & PILOT_I2CF_SLAVE_EN))
	{
		pilot_i2c_disable_bus(bus);
		ctrl_bits |= PILOT_I2CF_SLAVE_EN;
		writel(ctrl_bits, bus->base + PILOT_I2C_FUN_CTRL_REG );
		pilot_i2c_enable_bus(bus);
		//      i2c_pilot_ii_write_reg( bus,1,I2C_ENABLE_REG );
	}

	local_irq_restore(flags);

	return 0;
}

/* precondition: bus.lock has been acquired. */
static int pilot_i2c_init_clk(struct pilot_i2c_bus *bus,int speed)
{
	unsigned int speed_in_nsec=0x0;
	unsigned short temp=0;
	unsigned int HCNT_REG;
	unsigned int LCNT_REG;
	unsigned int hs_i2c = 0;
	unsigned int hcnt_val;
	unsigned int lcnt_val;
	bus->bus_speed=speed;

	if ((bus->adap.nr == 0x5) || (bus->adap.nr == 0x6) || (bus->adap.nr == 0x7) || (bus->adap.nr == 0x8))
		hs_i2c = 1;
	if((speed > 400) && !hs_i2c)
		return 1;

	temp=readl(bus->base + PILOT_I2C_FUN_CTRL_REG);
	temp&=0xfff9;
	speed_in_nsec=(1000000/speed);

	if(speed<=100)
	{
		temp |=(1<<1);
		HCNT_REG = PILOT_I2C_SS_SCL_HCNT_REG;
		LCNT_REG = PILOT_I2C_SS_SCL_LCNT_REG;
		/* Based on the loading conditions on Orion the calculated values are programmed */
		/* for known frequencies. */
		if (speed == 100)
		{
			hcnt_val = 0xE3;
			lcnt_val = 0x106;
		}
		else
			hcnt_val = lcnt_val = ((speed_in_nsec/20)/2);
	}
	else if((speed>100) && (speed<=400))
	{
/*		if(hs_i2c){
			temp |=(3<<1);
		HCNT_REG = PILOT_I2C_HS_SCL_HCNT_REG;
		LCNT_REG = PILOT_I2C_HS_SCL_LCNT_REG;
		}
		else{*/
			temp |=(2<<1);
		HCNT_REG = PILOT_I2C_FS_SCL_HCNT_REG;
		LCNT_REG = PILOT_I2C_FS_SCL_LCNT_REG;
/*	}*/
		/* If it is 400 then program the known values as per the spec,
		 * Otherwise calculate
		 */
		if (speed == 400)
		{
			hcnt_val = 0x26;
			lcnt_val = 0x49;
		}
		else
			hcnt_val = lcnt_val = ((speed_in_nsec/20)/2);
	}
#if 1
	else
	{
		temp |=(3<<1);
		HCNT_REG = PILOT_I2C_HS_SCL_HCNT_REG;
		LCNT_REG = PILOT_I2C_HS_SCL_LCNT_REG;
		// If it is 3400 then program the known values as per the spec,
                // Otherwise calculate
                if (speed == 3400)
                {
                        hcnt_val = 0x06;
                        lcnt_val = 0x10;
                }
                else
                        hcnt_val = lcnt_val = ((speed_in_nsec/10)/2);
        }

        // The above calculations where speed is less than 400k was done assuming SS/FS I2C.
        // So if it is HS I2C multiply it back by 2 since the input clock is 100Mhz for HS I2C.
        if (hs_i2c && (speed <= 400))
        {
                hcnt_val *= 2;
                lcnt_val *= 2;
	#if 0
		HCNT_REG = PILOT_I2C_HS_SCL_HCNT_REG;
		LCNT_REG = PILOT_I2C_HS_SCL_LCNT_REG;
	#endif
        }
#endif
	/* Disable I2C interface first */
	pilot_i2c_disable_bus(bus);
	/* Program the speed mode to the IC_CON register */
	writel(temp, bus->base + PILOT_I2C_FUN_CTRL_REG);

	writel(lcnt_val, bus->base + LCNT_REG);
	writel(hcnt_val, bus->base + HCNT_REG);
	//printk("from I2C %d CLK : HCNT : %x , LCNT : %x ,FUN_CTRL %x\n",
			//bus->adap.nr, hcnt_val ,lcnt_val, temp);
	pilot_i2c_enable_bus(bus);

	return 0;
}
void i2c_init_internal_data(struct pilot_i2c_bus *bus)
{
	int i;
	static bool kernel_init_done[BUS_COUNT] = {false,};

	/* Initialize locks, queues and variables */
	if (!kernel_init_done[bus->adap.nr])
	{
		spin_lock_init( &bus->data_lock );
		spin_lock_init( &bus->i2c_irq_lock );
		spin_lock_init( &bus->to_irq_lock );
		init_waitqueue_head( &(bus->pilot_i2c_wait));
		kernel_init_done[bus->adap.nr] = true;
	}
	bus->op_status = 0;
	bus->abort_status = 0;

	bus->TX_len = 0;
	bus->TX_index = 0;

	bus->MasterRX_len = 0;
	bus->MasterRX_index = 0;
	bus->Master_rd_cmd_index = 0;


	bus->Linear_SlaveRX_len = 0;
	bus->Linear_SlaveRX_index = 0;

	for(i=0;i<MAX_FIFO_LEN;i++)
	{
		bus->SlaveRX_len[i] = 0;
		bus->SlaveRX_index[i] = 0;
	}
	bus->SlaveRX_Writer = 0;
	bus->SlaveRX_Reader = 0;
	bus->SlaveRX_Entries = 0;

	bus->master_xmit_recv_mode_flag = SLAVE;

	bus->start_detected = 0;
	bus->RD_REQ_Pending = 0x0;
}

/* precondition: bus.lock has been acquired. */
static int pilot_i2c_init(struct pilot_i2c_bus *bus, struct platform_device *pdev)	
{
	int ret;
	u32 fun_ctrl_reg;

	/* Clear Intr Sts and Disbale Time out interrupts */
	if ((bus->adap.nr >= 0) && (bus->adap.nr <= 7)){
	pilot_write_reg(syscon_i2c_regmap , 
		PILOT_I2C_TO_BITS << ((bus->adap.nr)*4) ,PILOT_I2C_COUNTER_STS_REG);

	pilot_write_reg( syscon_i2c_regmap , pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_COUNTER_INTR_EN_REG) &
			~(PILOT_I2C_TO_BITS << ((bus->adap.nr)*4)),
			PILOT_I2C_COUNTER_INTR_EN_REG);
	}
	else if(bus->adap.nr == 8 || bus->adap.nr ==9){
	pilot_write_reg(syscon_i2c_regmap , 
		PILOT_I2C_TO_BITS << ((bus->adap.nr - 8)*4) ,PILOT_I2C_COUNTER_STS_REG_1);

	pilot_write_reg( syscon_i2c_regmap , pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_COUNTER_INTR_EN_REG_1) &
			~(PILOT_I2C_TO_BITS << ((bus->adap.nr - 8)*4)),
			PILOT_I2C_COUNTER_INTR_EN_REG_1);

	}
	/*Disbale timeout counters*/
	if ((bus->adap.nr >= 0) && (bus->adap.nr <= 7)){
	pilot_write_reg(syscon_i2c_regmap , pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_COUNTER_EN) &
			~(PILOT_I2C_TO_BITS << ((bus->adap.nr)*4)),PILOT_I2C_COUNTER_EN);
	}
	else if ((bus->adap.nr == 8) || (bus->adap.nr == 9)){
	pilot_write_reg(syscon_i2c_regmap , pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_COUNTER_EN_1) &
			~(PILOT_I2C_TO_BITS << ((bus->adap.nr - 8)*4)),PILOT_I2C_COUNTER_EN_1);
	}

	/*I2C Features enable register*/
	if ((bus->adap.nr >= 0) && (bus->adap.nr <= 3)){
	pilot_write_reg(syscon_i2c_regmap , pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_FEATURE_EN_0) |
			(PILOT_I2C_FEATURES << ((bus->adap.nr) * 8)),
			PILOT_I2C_FEATURE_EN_0);
	}
	else if ((bus->adap.nr >= 4) && (bus->adap.nr <= 7)){
	pilot_write_reg(syscon_i2c_regmap , pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_FEATURE_EN_1) |
			(PILOT_I2C_FEATURES << ((bus->adap.nr - 4) * 8)),
			PILOT_I2C_FEATURE_EN_1);
	}
	else if ((bus->adap.nr == 8) || (bus->adap.nr == 9)){
	pilot_write_reg(syscon_i2c_regmap , pilot_read_reg(syscon_i2c_regmap , PILOT_I2C_FEATURE_EN_2) |
			(PILOT_I2C_FEATURES << ((bus->adap.nr - 8) * 8)),
			PILOT_I2C_FEATURE_EN_2);
	}

	/*Disable all interrupts*/
	writel(0, bus->base + PILOT_I2C_INTR_CTRL_REG);

	/* Init I2CPCT registers for TAR bit and I2C mode */
	pilot_i2c_init_i2cpct(bus);

	/* Disable everything. */
	writel(0, bus->base + PILOT_I2C_ENABLE_REG);	/*I2C DISBLE*/

	/* Default slave address */
	writel(DEFAULT_SLAVE_ADDR, bus->base + PILOT_I2C_M_SAR);

	/*Threshold for RX and TX FiFo*/
	writel(DEFAULT_TX_THRESHOLD, bus->base + PILOT_I2C_TX_TL_REG);
	writel(DEFAULT_RX_THRESHOLD, bus->base + PILOT_I2C_RX_TL_REG);
	/*Enable 7Bit addr by Target Register*/
	writel(readl(bus->base + PILOT_I2C_M_TAR) & 
			~PILOT_I2CT_10BITADDR_MASTER,
			bus->base + PILOT_I2C_M_TAR);
		/*
		 *Master enable, fast mode, Restart enable, enable 7bit address	
		 *enable stop detction
		 *enable clock strectching
		 */
	fun_ctrl_reg = PILOT_I2CF_MASTER_EN | PILOT_I2CF_RESTART_EN;
	fun_ctrl_reg &= (~PILOT_I2CF_10BITADDR_SLAVE );
	fun_ctrl_reg |= PILOT_STOP_DET_IFADDRESSED;
	fun_ctrl_reg |= PILOT_I2C_RXFIFO_CLK_STRETCH;
	writel(fun_ctrl_reg, bus->base + PILOT_I2C_FUN_CTRL_REG);




	/* initialize clk scl for fast speed*/
	ret = pilot_i2c_init_clk(bus, DEFAULT_I2C_SPEED);
	if (ret < 0)
		return ret;

	

	
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	/* If slave has already been registered, re-enable it. */
	if (bus->slave)
		__pilot_i2c_reg_slave(bus, bus->slave->addr);
#endif /* CONFIG_I2C_SLAVE */

	/* Set interrupt generation of I2C controller */
	writel(PILOT_I2CD_INTR_ALL, bus->base + PILOT_I2C_INTR_CTRL_REG);
	
	pilot_i2c_enable_bus(bus);

	return 0;
}

static int pilot_i2c_reset(struct pilot_i2c_bus *bus)
{
	struct platform_device *pdev = to_platform_device(bus->dev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&bus->lock, flags);

	/* Disable and ack all interrupts. */
	writel(0, bus->base + PILOT_I2C_INTR_CTRL_REG);
	readl(bus->base + PILOT_I2C_INTR_CLR_REG);

	ret = pilot_i2c_init(bus, pdev);

	spin_unlock_irqrestore(&bus->lock, flags);

	return ret;
}
static int perform_pilot_i2c_recovery(struct pilot_i2c_bus *bus)
{
	 struct platform_device *pdev = to_platform_device(bus->dev);
	i2c_init_internal_data(bus);
	pilot_i2c_hw_reset(bus);
	pilot_i2c_init(bus,pdev);

	if ((0 != get_sda_status(bus))||(0 != get_clk_status(bus)))
	{
		//printk("ERROR: I2C%d: Pilot-ii recovery failed\n",bus->adap.nr);
		return 1;
	}

	printk("I2C%d: Pilot-ii recovery succeeded\n",bus->adap.nr);
	return 0;
}
static const struct of_device_id pilot_i2c_bus_of_table[10] = {
	{
		.compatible = "aspeed,pilot-i2c-0",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,pilot-i2c-1",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,pilot-i2c-2",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,pilot-i2c-3",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,pilot-i2c-4",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,pilot-i2c-5",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,pilot-i2c-6",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,pilot-i2c-7",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,pilot-i2c-8",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,pilot-i2c-9",
		.data = pilot_i2c_get_clk_reg_val,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, pilot_i2c_bus_of_table);

static int pilot_i2c_probe_bus(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct pilot_i2c_bus *bus;
	struct clk *parent_clk;
	struct resource *res,*res1;
	struct device_node *np = pdev->dev.of_node;
	int irq, ret;
	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bus->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bus->base))
		return PTR_ERR(bus->base);
	
	parent_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(parent_clk))
		return PTR_ERR(parent_clk);
	bus->parent_clk_frequency = clk_get_rate(parent_clk);
	/* We just need the clock rate, we don't actually use the clk object. */
	devm_clk_put(&pdev->dev, parent_clk);
	/*
	   bus->rst = devm_reset_control_get_shared(&pdev->dev, NULL);
	   if (IS_ERR(bus->rst)) {
	   dev_err(&pdev->dev,
	   "missing or invalid reset controller device tree entry");
	   return PTR_ERR(bus->rst);
	   }
	   reset_control_deassert(bus->rst);
	 */
	ret = of_property_read_u32(pdev->dev.of_node,
			"bus-frequency", &bus->bus_frequency);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"Could not read bus-frequency property\n");
		bus->bus_frequency = 100000;
	}


	syscon_i2c_regmap = syscon_regmap_lookup_by_phandle(np,
                                "syscon");
        if (IS_ERR(syscon_i2c_regmap)) {
                dev_err(&pdev->dev, "Couldn't get regmap\n");
                return -ENODEV;
        }


	match = of_match_node(pilot_i2c_bus_of_table, pdev->dev.of_node);
	if (!match)
		bus->get_clk_reg_val = pilot_i2c_get_clk_reg_val;
	else
		bus->get_clk_reg_val = match->data;

	/* Initialize the I2C adapter */
	spin_lock_init(&bus->lock);
	init_completion(&bus->cmd_complete);
	bus->adap.owner = THIS_MODULE;
	bus->adap.retries = 0;
	bus->adap.timeout = 5 * HZ;
	bus->adap.algo = &pilot_i2c_algo;
	bus->adap.dev.parent = &pdev->dev;
	bus->adap.dev.of_node = pdev->dev.of_node;
	strlcpy(bus->adap.name, pdev->name, sizeof(bus->adap.name));
	i2c_set_adapdata(&bus->adap, bus);

	bus->dev = &pdev->dev;

	/* Clean up any left over interrupt state. */
	writel(0, bus->base + PILOT_I2C_INTR_CTRL_REG);
	ret = readl(bus->base + PILOT_I2C_INTR_CLR_REG);
	/*
	 * bus.lock does not need to be held because the interrupt handler has
	 * not been enabled yet.
	 */
	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	ret = devm_request_irq(&pdev->dev, irq, pilot_i2c_bus_irq,
			0, dev_name(&pdev->dev), bus);
	if (ret < 0)
		return ret;

	ret = i2c_add_adapter(&bus->adap);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, bus);
	ret = pilot_i2c_init(bus,pdev);
	if (ret < 0)
		return ret;


	dev_info(bus->dev, "i2c bus %d registered, irq %d\n",
			bus->adap.nr, irq);
	i2c_init_internal_data(bus);
	return 0;
}

static int pilot_i2c_remove_bus(struct platform_device *pdev)
{
	struct pilot_i2c_bus *bus = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&bus->lock, flags);

	/* Disable everything. */
	writel(0, bus->base + PILOT_I2C_FUN_CTRL_REG);
	writel(0, bus->base + PILOT_I2C_INTR_CTRL_REG);

	spin_unlock_irqrestore(&bus->lock, flags);

	reset_control_assert(bus->rst);

	i2c_del_adapter(&bus->adap);

	return 0;
}

static struct platform_driver pilot_i2c_bus_driver = {
	.probe		= pilot_i2c_probe_bus,
	.remove		= pilot_i2c_remove_bus,
       .driver         = {
               .name           = "PILOT-I2C-BUS",
               .of_match_table = pilot_i2c_bus_of_table,
       },
};
module_platform_driver(pilot_i2c_bus_driver);

MODULE_AUTHOR("Vishal C Nigade <vishal.nigade@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed Pilot I2C Bus Driver");
MODULE_LICENSE("GPL v2");
