/*
 * ast-cam-scu.c - CAM SCU driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <mach/ast-cam-scu.h>
#include <mach/regs-cam-scu.h>

//#define ASPEED_SCU_LOCK
//#define ASPEED_SCU_DEBUG

#ifdef ASPEED_SCU_DEBUG
#define SCUDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SCUDBUG(fmt, args...)
#endif

#define SCUMSG(fmt, args...) printk(fmt, ## args)

void __iomem *ast_scu_base;

spinlock_t ast_scu_lock;

static inline u32 
ast_scu_read(u32 reg)
{
	u32 val;
		
	val = readl(ast_scu_base + reg);
	
	SCUDBUG("ast_scu_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_scu_write(u32 val, u32 reg) 
{
	SCUDBUG("ast_scu_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
#ifdef CONFIG_AST_SCU_LOCK
	//unlock 
	writel(SCU_PROTECT_UNLOCK, ast_scu_base);
	writel(val, ast_scu_base + reg);
	//lock
	writel(0xaa,ast_scu_base);	
#else
	writel(SCU_PROTECT_UNLOCK, ast_scu_base);
	writel(val, ast_scu_base + reg);
#endif
}

//SoC mapping Table 
struct soc_id {
	const char	* name;
	u32	rev_id;
};

#define SOC_ID(str, rev) { .name = str, .rev_id = rev, }

static struct soc_id soc_map_table[] = {
	SOC_ID("AST1220-A0", 0x04030000),
};
//***********************************Initial control***********************************
extern void
ast_scu_reset_jpeg(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_JPEG, AST_SCU_RESET);
	udelay(100);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_JPEG, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast_scu_reset_jpeg);

extern void
ast_scu_init_jpeg(u8 dynamic_en)
{

#if 0
	//Video Engine Clock Enable and Reset
	//  Enable Clock & ECLK = inverse of (M-PLL / 2)
	if(dynamic_en)
		ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_VIDEO_SLOW_MASK) | SCU_CLK_VIDEO_SLOW_EN | SCU_CLK_VIDEO_SLOW_SET(0), AST_SCU_CLK_SEL);
	else {
		if(GET_CHIP_REVISION(ast_scu_read(AST_SCU_REVISION_ID)) == 4)
			ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_MASK | SCU_CLK_VIDEO_SLOW_EN)), AST_SCU_CLK_SEL);
		else
			ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_EN)) | SCU_ECLK_SOURCE(2), AST_SCU_CLK_SEL);
	}
#endif	
	// Enable CLK
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~(SCU_JCLK_STOP_EN), AST_SCU_CLK_STOP);	
	mdelay(10);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_JPEG, AST_SCU_RESET);
	udelay(100);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_JPEG, AST_SCU_RESET);
}
EXPORT_SYMBOL(ast_scu_init_jpeg);

extern void
ast_scu_init_usb_port1(void)
{
	/* EHCI controller engine init. Process similar to VHub. */
	/* Following reset sequence can resolve "vhub dead on first power on" issue on V4 board. */
	//reset USB20
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_USB20, AST_SCU_RESET);

	//enable USB20 clock
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_USB20_PHY_CLK_EN, AST_SCU_CLK_STOP);
	mdelay(10);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_USB20, AST_SCU_RESET);
}


EXPORT_SYMBOL(ast_scu_init_usb_port1);


extern void
ast_scu_init_i2c(void)
{
	spin_lock(&ast_scu_lock);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_I2C, AST_SCU_RESET);
	udelay(3);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_I2C, AST_SCU_RESET);
	
	spin_unlock(&ast_scu_lock);		
}

EXPORT_SYMBOL(ast_scu_init_i2c);

extern void
ast_scu_init_pwm(void)
{
	spin_lock(&ast_scu_lock);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_PWM, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_PWM, AST_SCU_RESET);
	
	spin_unlock(&ast_scu_lock);	
}

EXPORT_SYMBOL(ast_scu_init_pwm);

ast_scu_init_hace(void)
{
	//enable YCLK for HAC
	spin_lock(&ast_scu_lock);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) &
					~(SCU_YCLK_STOP_EN), 
					AST_SCU_CLK_STOP);
	mdelay(1);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) &
					~SCU_RESET_HACE, 
					AST_SCU_RESET);
	spin_unlock(&ast_scu_lock);	

}
EXPORT_SYMBOL(ast_scu_init_hace);

extern void
ast_scu_init_h264(void)
{

}
EXPORT_SYMBOL(ast_scu_init_h264);

extern void
ast_scu_clk_stop(u32 clk_name,u8 stop_enable)
{
	switch(clk_name){
		default:
			SCUMSG("ERRO clk_name :%d \n",clk_name);
			break;
	}
}

EXPORT_SYMBOL(ast_scu_clk_stop);

//***********************************CLK Information***********************************

extern u32
ast_get_clk_source(void)
{
}
EXPORT_SYMBOL(ast_get_clk_source);

extern u32
ast_get_h_pll_clk(void)
{
	printk("TODO ~~\n");

	return 0;
}

EXPORT_SYMBOL(ast_get_h_pll_clk);

extern u32
ast_get_m_pll_clk(void)
{
	printk("TODO ~~\n");

	return 0;

}

EXPORT_SYMBOL(ast_get_m_pll_clk);

extern u32
ast_get_ahbclk(void)
{
	printk("TODO ~~\n");

	return 0;
}

EXPORT_SYMBOL(ast_get_ahbclk);

extern u32
ast_get_d_pll_clk(void)
{
	return 10000000;
}
EXPORT_SYMBOL(ast_get_d_pll_clk);

extern u32
ast_get_pclk(void)
{
	return (10000000);
}

EXPORT_SYMBOL(ast_get_pclk);

extern u32
ast_get_lhclk(void)
{
		return 0;
}

EXPORT_SYMBOL(ast_get_lhclk);

extern void
ast_scu_osc_clk_output(void)
{

}

EXPORT_SYMBOL(ast_scu_osc_clk_output);

extern u32
ast_get_sd_clock_src(void)
{
	printk("TODO ~~ ast_get_sd_clock_src \n");
#ifdef CONFIG_AST_CAM_FPGA
	return 50000000;
#else
	//return 54000000;
	return 108000000;
#endif
}

EXPORT_SYMBOL(ast_get_sd_clock_src);

extern void
ast_scu_show_system_info (void)
{


	return ;
}

EXPORT_SYMBOL(ast_scu_show_system_info);

//*********************************** Multi-function pin control ***********************************
extern void
ast_scu_multi_func_uart(u8 uart)
{
}

extern void
ast_scu_multi_func_eth(void)
{
}

extern void
ast_scu_multi_func_romcs(u8 num)
{
}

extern void
ast_scu_multi_func_i2c(u8 bus_no)
{
}	

EXPORT_SYMBOL(ast_scu_multi_func_i2c);

extern void
ast_scu_multi_func_pwm(void)
{
	spin_lock(&ast_scu_lock);	
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) | 0xff, AST_SCU_FUN_PIN_CTRL3);
	spin_unlock(&ast_scu_lock);	
}	

EXPORT_SYMBOL(ast_scu_multi_func_pwm);

//0 : usb 2.0 hub mode, 1:usb 2.0 host2 controller
extern void
ast_scu_multi_func_usb_port1_mode(u8 mode)
{
	if(mode)
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB20_HOST, 
					AST_SCU_FUN_PIN_CTRL5);
	else
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB20_HOST, 
					AST_SCU_FUN_PIN_CTRL5);
}	

EXPORT_SYMBOL(ast_scu_multi_func_usb_port1_mode);

extern void
ast_scu_multi_func_sdhc_slot(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) |SCU_FUC_PIN_SDIO | SCU_FUC_PIN_SD1 | SCU_FUC_PIN_SD2, 
				AST_SCU_FUN_PIN_CTRL5);						
}	

EXPORT_SYMBOL(ast_scu_multi_func_sdhc_slot);

extern void
ast_scu_multi_nic_switch(u8 enable)
{
		
}

//***********************************Information ***********************************
extern u32
ast_scu_revision_id(void)
{
	int i;
	u32 rev_id = ast_scu_read(AST_SCU_REVISION_ID);
	for(i=0;i<ARRAY_SIZE(soc_map_table);i++) {
		if(rev_id == soc_map_table[i].rev_id)
			break;
	}
	if(i == ARRAY_SIZE(soc_map_table))
		SCUMSG("UnKnow-SOC : %x \n",rev_id);
	else
		SCUMSG("SOC : %4s \n",soc_map_table[i].name);
	
	return rev_id;
}	

EXPORT_SYMBOL(ast_scu_revision_id);

extern void
ast_scu_sys_rest_info(void)
{
	u32 rest = ast_scu_read(AST_SCU_SYS_CTRL);

	if (rest & SCU_SYS_WDT1_RESET_FLAG) {
		SCUMSG("RST : WDT1 \n");		
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_WDT1_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
#ifdef SCU_SYS_WDT2_RESET_FLAG		
	if (rest & SCU_SYS_WDT2_RESET_FLAG) {
		SCUMSG("RST : WDT2 - 2nd Boot \n");
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_WDT2_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
#endif
#ifdef SCU_SYS_WDT3_RESET_FLAG
	if (rest & SCU_SYS_WDT3_RESET_FLAG) {
		SCUMSG("RST : WDT3 - Boot\n");
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_WDT3_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
#endif		
	if (rest & SCU_SYS_PWR_RESET_FLAG) {
		SCUMSG("RST : Power On \n");
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_PWR_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
}	

extern u32
ast_scu_get_soc_dram_base(void)
{
	u32 rev_id = ast_scu_read(AST_SCU_REVISION_ID);
	if((rev_id >> AST_SOC_GEN) > 3) 
		return AST_DRAM_BASE_8;
	else
		return AST_DRAM_BASE_4;
}

extern void
ast_scu_hw_random_enable(u8 enable)
{
	if(enable)
		ast_scu_write(ast_scu_read(AST_SCU_RAMDOM_GEN) | RNG_ENABLE, AST_SCU_RAMDOM_GEN);
	else
		ast_scu_write(ast_scu_read(AST_SCU_RAMDOM_GEN) & ~RNG_ENABLE, AST_SCU_RAMDOM_GEN);
}
EXPORT_SYMBOL(ast_scu_hw_random_enable);

extern u32
ast_scu_hw_random_read(void)
{
	return (ast_scu_read(AST_SCU_RAMDOM_DATA));
}
EXPORT_SYMBOL(ast_scu_hw_random_read);

extern u8
ast_scu_get_hw_random_type(void)
{
	return (RNG_GET_TYPE(ast_scu_read(AST_SCU_RAMDOM_GEN)));
}
EXPORT_SYMBOL(ast_scu_get_hw_random_type);

extern void
ast_scu_set_hw_random_type(u8 type)
{
	ast_scu_write(((ast_scu_read(AST_SCU_RAMDOM_GEN) & ~RNG_TYPE_MASK) | RNG_SET_TYPE(type)), AST_SCU_RAMDOM_GEN);
}
EXPORT_SYMBOL(ast_scu_set_hw_random_type);

static int ast_cam_scu_probe(struct platform_device *pdev)
{
	int ret = 0;
	int irq = 0;
	struct resource *res;
	struct device_node *np;
	u32 idx;
	u32 reset_mask;

	SCUDBUG("\n");	

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ast_scu_base = devm_ioremap_resource(&pdev->dev, res);

#if 0 //def CONFIG_AST_RUNTIME_DMA_UARTS
	if(of_machine_is_compatible("aspeed,ast2500")) {
		if((CONFIG_AST_RUNTIME_DMA_UARTS > 2) || (CONFIG_SERIAL_8250_RUNTIME_UARTS > 4)) {
			ast_scu_uartx_init();
		}
	}
#endif

	for_each_compatible_node(np, NULL, "aspeed,ast-mac") {
		printk("aspeed,ast-mac found in SCU, ");
		ast_scu_multi_func_eth();
	}

	if(of_find_compatible_node(NULL, NULL, "aspeed,ast-cam-pwm")) {
		printk("aspeed,ast-cam-pwm found in SCU \n");
		//SCU Pin-MUX	//PWM & TACHO 
		ast_scu_multi_func_pwm();		
		//SCU PWM CTRL Reset
		ast_scu_init_pwm();
	}

	//SCU I2C Reset 
	//ast_scu_init_i2c();
	//ast_scu_multi_func_i2c(idx);
	//I2C 1/9
	ast_scu_write(ast_scu_read(0x80) | (0x3 << 24), 0x80);
	ast_scu_write(ast_scu_read(0x90) | (0x1 <<  8), 0x90);

	ast_scu_write(ast_scu_read(0x88) | (0x3 << 30), 0x88);
	ast_scu_write(ast_scu_read(0x90) | (0x1 << 21), 0x90);

#if 1
	if(of_find_compatible_node(NULL, NULL, "aspeed,ast-ehci")) {
		printk("aspeed,ast-ehci found in SCU \n");
		ast_scu_multi_func_usb_port1_mode(1);
	}
#else
	if(of_find_compatible_node(NULL, NULL, "aspeed,ast-udc")) {
		printk("aspeed,ast-udc found in SCU \n");
		ast_scu_multi_func_usb_port1_mode(0);
	}
#endif
	if(of_find_compatible_node(NULL, NULL, "aspeed,ast-jpeg")) {
		printk("aspeed,ast-jpeg found in SCU, ");
		ast_scu_init_jpeg(0);
	}

	if(of_find_compatible_node(NULL, NULL, "aspeed,ast-crypto")) {
		printk("aspeed,ast-crypto found in SCU \n");
		ast_scu_init_hace();
	}

out:
	return ret;
}

static const struct of_device_id ast_cam_scu_of_match[] = {
	{ .compatible = "aspeed,ast-cam-scu", },
	{ }
};

static struct platform_driver ast_cam_scu_driver = {
	.probe = ast_cam_scu_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = ast_cam_scu_of_match,
	},
};

static int ast_cam_scu_init(void)
{
	return platform_driver_register(&ast_cam_scu_driver);
}

core_initcall(ast_cam_scu_init);
