/*
 * Copyright 2018 ASpeed Technology.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/serial_8250.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/cacheflush.h>
#include <asm/smp_scu.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "pilot4_asic.h"

void __iomem *p4smp_base_addr = NULL;
void __iomem *p4smp_scu_base = NULL;

volatile void * wdt_reg;
static unsigned int timer3_base;
static u32 ddr_single_bit_error;
static u32 ddr_multi_bit_error;

void reinit_spi(void);
static void start_wdt2(void);
static void wdt2_ping(void);
extern void soft_restart(unsigned long);

#define SE_TIMER_VA_BASE	timer3_base
#define CLOCK_TICK_RATE		(25*1000*1000)

#define SE_SYS_CLK_VA_BASE	IO_ADDRESS(0x40100100)
#define SE_BOOT_SPI_VA_BASE	IO_ADDRESS(0x40200000)
#define SE_USB_1_VA_BASE 	IO_ADDRESS(0x40800000)
#define	SE_PILOT_SPEC_VA_BASE	IO_ADDRESS(0x40426500)

#define SPI_ADDR    (SE_BOOT_SPI_VA_BASE + 0x0)
#define SPI_CMD     (SE_BOOT_SPI_VA_BASE + 0x8)
#define SPI_STS     (SE_BOOT_SPI_VA_BASE + 0x18)
#define SPI_FIFO    (SE_BOOT_SPI_VA_BASE + 0x30)
#define SPI_MISC	(SE_BOOT_SPI_VA_BASE + 0x1C)
#define SPI_CLK_MAX 		0x111
#define SPI_CLK_DEFAULT 	0x444

#define SYSWCR              0x90
#define SYSWRERL            0x94
#define SYSWRERH            0x98
#define SYSWCFR             0x9C
#define WDT_TCLK            381
#define WDT_EN              0x800000
#define PRE_TRIG_EN         0x400000
#define TRIG_WDT            0x1
#define RST_PULSE_WIDTH     0xC	/*using default value 0xC*(40.96us)=491.52us*/
#define PRETIMEOUT          20
#define HEARTBEAT           172

#define DSSYSES 			0x8
#define DERRRDADD 			0x24
#define single_bit_err 		(1 << 3)
#define multi_bit_err 		(1 << 4)

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.flags          =  UPF_LOW_LATENCY | UPF_SKIP_TEST,
		.iotype         = UPIO_MEM32,
		.regshift       = 2,
	},
};

static struct platform_device serial_device = {
	.name	= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM,
	.dev	= {
				.platform_data  = serial_platform_data,
	},
};

static const char *const p4_dt_match[] __initconst = {
	"pilot4,ASIC",
	NULL,
};

static const struct of_device_id pilot_debug_uart_dt[] __initconst = {
	{ .compatible = "pilot,ns16550a" },
	{ }
};

unsigned int IO_ADDRESS(unsigned int addr)
{
	return (0xFD000000 | addr);
}
EXPORT_SYMBOL(IO_ADDRESS);

void __init se_pilot4_init(void)
{
	struct device_node *dn;
	struct resource res;
	unsigned int val;
	unsigned int enable_highspeed_uart=0;
	unsigned int uart_clk=1843200;
	int ret = -EINVAL;
#ifdef CONFIG_PILOT4_SDHC
	unsigned int rerl=0;
	unsigned int rerh=0;
#endif

	dn = of_find_compatible_node(NULL, NULL, "pilot,ns16550a");
	if (!dn)
		return;

	ret = of_address_to_resource(dn, 0, &res);
	if (ret)
		return;

	of_property_read_u32(dn, "highspeed-uart", &enable_highspeed_uart);
	of_property_read_u32(dn, "clock-frequency", &uart_clk);
	serial_platform_data[0].membase = of_iomap(dn, 0);
	serial_platform_data[0].mapbase = res.start;
	serial_platform_data[0].irq = irq_of_parse_and_map(dn, 0);
	serial_platform_data[0].uartclk = uart_clk;

	if(enable_highspeed_uart)
	{
		printk("High speed UART is enabled. clock-frequecy is %d\n", uart_clk);
		/* As suggested Set Bit0 of 0x40426510 and Bit 18 of 0x4010012C to use 50MHZ clock to debug UART */
		val = *(volatile unsigned int*)(SE_PILOT_SPEC_VA_BASE + 0x10);//50MHZ Clock to UART
		val = val | 1;
		*(volatile unsigned int*)(SE_PILOT_SPEC_VA_BASE + 0x10) = val;
		val = *(volatile unsigned int*)(SE_SYS_CLK_VA_BASE + 0x2C);
		val = val | (1 << 18);
		*(volatile unsigned int*)(SE_SYS_CLK_VA_BASE + 0x2C) = val;
	} else {
		/*
		 * The Debug UART uses 25MHz/14 = 1.786MHz, the standard clock needed is 1.846MHz.
		 * The difference in the clock is within the error margin. By shifting the point when
		 * we latch the incoming RXD to the left of the center will improves the overall error
		 * margin. This allows the transmitter clock to have more margin at the highest
		 * supported rate of 115.2K Baud rate. */

		printk("UART clock-frequecy is %d\n", uart_clk);
		/* As suggested Clear Bit0 of 0x40426510 and Clear Bit 18 of 0x4010012C to Disable 50MHZ clock to debug UART */
		val = *(volatile unsigned int*)(SE_PILOT_SPEC_VA_BASE + 0x10);//Disable 50MHZ Clock to UART
		val = val & ~1;
		*(volatile unsigned int*)(SE_PILOT_SPEC_VA_BASE + 0x10) = val;
		val = *(volatile unsigned int*)(SE_SYS_CLK_VA_BASE + 0x2C);
		val = val & ~(7 << 16);
		val = val | (4 << 16);//as suggested by h/w  RX sampling shifted left by 2 Clocks
		*(volatile unsigned int*)(SE_SYS_CLK_VA_BASE + 0x2C) = val;
	}

	platform_device_register(&serial_device);
#ifdef CONFIG_PILOT4_SDHC
	/*Reset SDHC/EMMC Host controllers. This is needed incase they are used in u-boot
	  Dont reset SDHC0/SDHC1 Host controllers if SSP claims them. check ssp dev lock bit*/
	if( (*(volatile unsigned int *)(SE_USB_1_VA_BASE + 0x0718) & 0x1) != 0x1 )
	{
		rerl = (0x1 << 30);
		rerh = 0x1;
	}
	rerh |= (0x1 << 5);
	*(volatile unsigned int *)(SE_SYS_CLK_VA_BASE + 0x754) = rerl;
	*(volatile unsigned int *)(SE_SYS_CLK_VA_BASE + 0x758) = rerh;
	*(volatile unsigned int *)(SE_SYS_CLK_VA_BASE + 0x750) = 0xD01;
#endif
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static struct map_desc pilot4_std_desc[] __initdata =
{
	{
		.virtual        = 0xFD000000,
		.pfn            = __phys_to_pfn(0x40000000),
		.length         = SZ_16M,
		.type           = MT_DEVICE,
	},
	/*
	{
		.virtual        = 0xFD100000,
		.pfn            = __phys_to_pfn(0x40100000),
		.length         = SZ_8K,
		.type           = MT_DEVICE,
	},
	{
		.virtual        = 0xFD000000 | scu_a9_get_base(),
		//.pfn            = __phys_to_pfn(0x40100000),
		.pfn            = __phys_to_pfn(scu_a9_get_base()),
		.length         = SZ_8K,
		.type           = MT_DEVICE,
	},
	*/
};

void __init pilot4_map_io(void)
{
	iotable_init(pilot4_std_desc, ARRAY_SIZE(pilot4_std_desc));
	//p4smp_base_addr = ioremap(0x40100D00 , SZ_128);
	p4smp_base_addr = (void __iomem *)0xFD100D00;
	p4smp_scu_base = (void __iomem *)(0xFD000000 | scu_a9_get_base());
}

void dump_ddr(int halt)
{
	volatile u32 ddr_base;
	volatile u32 phys_ddr_base;
	volatile u32 *ddr_base_ptr;
	int i = 0;

	ddr_base = (volatile u32)IO_ADDRESS(0x40300000);
	phys_ddr_base = 0x40300000;
	ddr_base_ptr = (void*)ddr_base;
	for(i = 0;i < 33;i++) {
		if((i%4) == 0)
			printk("\n%x:", (phys_ddr_base + i*4));
		printk("%08x ", *(ddr_base_ptr + i));
	}
	printk("\n");
	ddr_base = (volatile u32)SE_SYS_CLK_VA_BASE;
	ddr_base_ptr = (void*)ddr_base;
	phys_ddr_base = 0x40100100;
	for(i = 0;i < 66;i++) {
		if((i%4) == 0)
			printk("\n%x:", (phys_ddr_base + i*4));
		printk("%08x ", *(ddr_base_ptr + i));
	}
	printk("\n");

	ddr_base = (volatile u32)IO_ADDRESS(0x40308000);
	ddr_base_ptr = (void*)ddr_base;
	phys_ddr_base = 0x40308000;
	for(i = 0;i < 6;i++) {
		if((i%4) == 0)
			printk("\n%x:", (phys_ddr_base + i*4));
		printk("%08x ", *(ddr_base_ptr + i));
	}

	printk("\n");
	ddr_base = (volatile u32)IO_ADDRESS(0x4030c000);
	ddr_base_ptr = (void*)ddr_base;
	phys_ddr_base = 0x4030c000;
	for(i = 0;i < 192;i++) {
		if((i%4) == 0)
			printk("\n%x:", (phys_ddr_base + i*4));
		printk("%08x ", *(ddr_base_ptr + i));
	}
	printk("\n");
	if(halt)
		kernel_halt();
}

irqreturn_t ddr_stat_handler (int irq, void *unused_for_now)
{
	volatile u32 ddr_base;
	u32 val;
	int single_bit_error = 0, multi_bit_error = 0;
	int halt = 1;

	ddr_base = (volatile u32)IO_ADDRESS(0x40300000);
	val = *(volatile unsigned int*)(ddr_base + DSSYSES);
	printk("DDR Interrupt DSSYSES %x\n", val);
	if(val & single_bit_err) {
		printk("Single bit error address %x\n", *(volatile unsigned int*)(ddr_base + DERRRDADD));
		ddr_single_bit_error++;
		single_bit_error = 1;
		halt = 0;
	}

	if(val & multi_bit_err) {
		printk("Multi bit error address %x\n", *(volatile unsigned int*)(ddr_base + DERRRDADD));
		ddr_multi_bit_error++;
		multi_bit_error = 1;
	}
	if(single_bit_error && multi_bit_error) {
		val = (val | multi_bit_err | single_bit_err);
	} else if(single_bit_error) {
		val = (val | single_bit_err);
	} else if(multi_bit_error) {
		val = (val | multi_bit_err);
	}
	*(volatile unsigned int*)(ddr_base + DSSYSES) = val;
	dump_ddr(halt);
	return IRQ_HANDLED;
}

static int proc_ddr_show(struct seq_file *m, void *unused_v)
{
	seq_printf(m, "SingleBitErrors:%d\n", ddr_single_bit_error);
	seq_printf(m, "MultiBitErrors:%d\n", ddr_multi_bit_error);
	return 0;
}

static int proc_ddr_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_ddr_show, NULL);
}

static const struct file_operations proc_ddr_ops = {
	.owner      = THIS_MODULE,
	.open       = proc_ddr_open,
	.read       = seq_read,
	.release    = single_release,
};

void pilot4_restart(enum reboot_mode mode, const char *cmd)
{
	volatile unsigned char * reset_base = (volatile unsigned char *)SE_SYS_CLK_VA_BASE;
	local_irq_disable();

	*(volatile unsigned int *)(reset_base) &= 0xFFFFFFFC;
	reinit_spi();
	/* Delay to allow the serial port to show the message */
	mdelay(50);
	*(volatile unsigned int *)(reset_base + 0x754) = 0x3;
	*(volatile unsigned int *)(reset_base + 0x758) = 0;     /*Make sure we dont reset other modules*/
	*(volatile unsigned int *)(reset_base + 0x750) = 0xD01;
	while (1)
		wfe();
/*	We will never reach here, however if we reach pls check wait_for_wfe in uboot */
	soft_restart(0);
/*	We'll take a jump through zero as a poor second */
}

static void wait_for_spi_ready(void) {
	while ((*(volatile unsigned int *)(SPI_STS)) & 0x01);
}

static void wait_till_ready(void)
{
	volatile unsigned char rdsts = 0;
	wait_for_spi_ready();
	*(volatile unsigned int *)(SPI_CMD) = 0x80000105;
	wait_for_spi_ready();

	rdsts = *(volatile unsigned char*)(SPI_FIFO);

	while ((rdsts & 0x1) == 0x1)
		rdsts = *(volatile unsigned char*)(SPI_FIFO);
}

void reinit_spi(void)
{
	*(volatile unsigned int*)(SPI_MISC) = 0xC0100804;
	*(volatile unsigned int*)(SPI_ADDR) = 0;

	wait_till_ready();
/*	If 3B strap is set then send disable 4B mode; just-in-case */
	if ((*(volatile unsigned int*)(SE_SYS_CLK_VA_BASE + 0x0C) & 0x40) != 0x40)
	{
		wait_for_spi_ready();
		*(volatile unsigned int *)(SPI_CMD) = 0x80001106;
		wait_for_spi_ready();
		wait_till_ready();
		wait_for_spi_ready();
		*(volatile unsigned int *)(SPI_CMD) = 0x800011e9;
		wait_for_spi_ready();
		wait_till_ready();

		/*	clear BMISC */
		*(volatile unsigned int*)(SPI_MISC) &= ~(1 << 24);
	}
	*(volatile unsigned long*)(SE_SYS_CLK_VA_BASE+0x20) &= 0xFFFFF000;
	*(volatile unsigned long*)(SE_SYS_CLK_VA_BASE+0x20) |= 0x222;

	*(volatile unsigned int *)(SPI_CMD) &= ~(0xFF000000);
	wait_for_spi_ready();
	printk("reinit_spi done\n");
}

extern void execute_smp_wfe(void);

void do_bmc_cpu_reset(void)
{
#ifdef CONFIG_USE_SSP_RESET
	local_irq_disable();
#endif
	execute_smp_wfe();
#ifndef CONFIG_USE_SSP_RESET
	reinit_spi();

	*(volatile unsigned int *)(SE_SYS_CLK_VA_BASE + 0x754) = 3;
	*(volatile unsigned int *)(SE_SYS_CLK_VA_BASE + 0x750) = 0xc01;
#endif

	for(;;)
		wfe();
	printk("Issue came out of wfe\n");
}

static irqreturn_t pilot_timer3_interrupt(int irq, void *dev_id)
{
	volatile u32 *sysrst_status_ptr;
	volatile unsigned long dummy_read;
	unsigned long flags;

	local_irq_save(flags);

	/* Reload watchdog duration */
	dummy_read = *((volatile unsigned long*) (SE_TIMER_VA_BASE+0x0c + 0x28));
#ifndef CONFIG_USE_SSP_RESET
	wdt2_ping();
#endif
	sysrst_status_ptr = (void*)(SE_SYS_CLK_VA_BASE + 0x7B0);
	/*  printk("Timer3 BMC bit_ten read word %x\n", *(volatile u32 *)sysrst_status_ptr); */

	if(*(volatile u32*)sysrst_status_ptr & (1 << 10))
	{
#ifdef CONFIG_USE_SSP_RESET
		local_irq_disable();
#else
		*(volatile u32*)sysrst_status_ptr = (1 << 10);
		printk("Resetting BMC CPU\n");
#endif
		do_bmc_cpu_reset();
	}

	local_irq_restore(flags);
	return IRQ_HANDLED;
}

void start_wdt2(void)
{
	volatile u32 reg;
	volatile u32 wdt_cnt;

	wdt_reg = (volatile void *)(SE_SYS_CLK_VA_BASE + 0x700);

	/* Set watchdog duration */
	wdt_cnt = ((WDT_TCLK * PRETIMEOUT) << 16) | (WDT_TCLK * HEARTBEAT);
	writel(wdt_cnt, wdt_reg + SYSWCFR);

	/* Enable ARM reset on watchdog */
	writel(0x3, wdt_reg + SYSWRERL);
	writel(0x0, wdt_reg + SYSWRERH);

	/* Trigger reset on watchdog */
	reg = readl(wdt_reg + SYSWCR);
	reg |= WDT_EN;
	writel(reg, wdt_reg + SYSWCR);
	/* Enable watchdog timer */
	reg = readl(wdt_reg + SYSWCR);
	reg &=~(0xff<<8);
	reg |= PRE_TRIG_EN;
	reg |= TRIG_WDT;
	reg |= RST_PULSE_WIDTH << 8;

	writel(reg, wdt_reg + SYSWCR);
	reg = readl(wdt_reg + SYSWCR);
}

static void wdt2_ping(void)
{
	volatile u32 reg;
	volatile u32 wdt_cnt;

	/* Reload watchdog duration */
	wdt_cnt = ((WDT_TCLK * PRETIMEOUT) << 16) | (WDT_TCLK * HEARTBEAT);
	writel(wdt_cnt, wdt_reg + SYSWCFR);

	/* Trigger reset on watchdog */
	reg = readl(wdt_reg + SYSWCR);
	reg |= WDT_EN;
	writel(reg, wdt_reg + SYSWCR);
}

static void start_pilot_timer3(void)
{
	volatile unsigned long dummy_read;
	unsigned long ctrl;
	volatile u32 *bmccpu_rerl_ptr;
	int err;
	int irq;
	struct device_node *dn;

	dn = of_find_compatible_node(NULL, NULL, "pilot,timer3-cpu-reset");
	if (!dn)
		return;

	timer3_base = (unsigned int)of_iomap(dn, 0);
	irq = irq_of_parse_and_map(dn, 0);

	bmccpu_rerl_ptr = (void*)(SE_SYS_CLK_VA_BASE + 0x7B4);
	*(volatile u32*)bmccpu_rerl_ptr &= ~(1);    //mask ARM reset

#ifndef CONFIG_USE_SSP_RESET
	start_wdt2();
#endif

	/* Disable timer and interrups */
	*((volatile unsigned long *)(SE_TIMER_VA_BASE+0x08 + 0x28)) = 0x0;
	dummy_read = *((volatile unsigned long*) (SE_TIMER_VA_BASE+0x0c + 0x28));
	/* Load counter values 100ms*/
	*((volatile unsigned long *)(SE_TIMER_VA_BASE + 0x28)) = ((CLOCK_TICK_RATE + 10/2) / 10);
	ctrl = 0x3;
	*((volatile unsigned long *)(SE_TIMER_VA_BASE+0x08 + 0x28)) = ctrl;
	printk("Timer3 100ms periodic timer startedd\n");

	err = request_irq(irq, &pilot_timer3_interrupt, IRQF_SHARED, "timer3-cpu-reset", (void*)0xDEADBEEF);
	if(err < 0) {
		printk(KERN_ERR "Error requesting Timer3 irq\n");
	}
}

static int ddr_ecc_init(void)
{
	u32 val;
	int ret = -1;
	volatile u32 ddr_base;
	volatile u32 ddr_irq;
	struct device_node *dn;

	dn = of_find_compatible_node(NULL, NULL, "pilot,pilot-ddr");
	if (!dn)
		return -1;

	ddr_base = (unsigned int)of_iomap(dn, 0);
	ddr_irq = irq_of_parse_and_map(dn, 0);

	val = *(volatile unsigned int*)(ddr_base + DSSYSES);
	if(val & 1) {
		ret = request_irq(ddr_irq, ddr_stat_handler, 0, "ddr_error_stats", NULL);
		if(ret) {
			WARN(1, "Error registering DDR Interrupt\n");
		}
		proc_create("pilot_ddr_stats", 0, NULL, &proc_ddr_ops);
		val = val | (1 << 5) | ( 1<< 6);//Enable single/multi bit error interrupts
		*(volatile unsigned int*)(ddr_base + DSSYSES) = val;
	}

	start_pilot_timer3();

	return 0;
}

late_initcall(ddr_ecc_init);

DT_MACHINE_START(ast_dt, "Pilot4 BMC SoC")
	.dt_compat              = p4_dt_match,
	.restart                = pilot4_restart,
	.map_io                 = pilot4_map_io,
//	.smp            = smp_ops(pilot4_smp_ops), // This is NOT required as we are exporting macro in platsmp.c
	.init_machine   = se_pilot4_init,
MACHINE_END
