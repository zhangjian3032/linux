/********************************************************************************
* File Name     : arch/arm/plat-aspeed/ast-scu.c 
* Author         : Ryan Chen
* Description   : AST SCU 
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 

CLK24M 
 |
 |--> H-PLL -->HCLK
 |
 |--> M-PLL -xx->MCLK
 |
 |--> V-PLL1 -xx->DCLK
 |
 |--> V-PLL2 -xx->D2CLK
 |
 |--> USB2PHY -->UTMICLK


*   History      : 
*    1. 2012/08/15 Ryan Chen Create
* 
********************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <mach/platform.h>
#include <asm/io.h>

#include <mach/hardware.h>

#include <plat/ast-scu.h>
#include <plat/regs-cam-scu.h>

//#define ASPEED_SCU_LOCK
//#define ASPEED_SCU_DEBUG

#ifdef ASPEED_SCU_DEBUG
#define SCUDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SCUDBUG(fmt, args...)
#endif

#define SCUMSG(fmt, args...) printk(fmt, ## args)

static u32 ast_scu_base = IO_ADDRESS(AST_SCU_BASE);

spinlock_t ast_scu_lock;

static inline u32 
ast_scu_read(u32 reg)
{
	u32 val;
		
	val = readl((void *)(ast_scu_base + reg));
	
	SCUDBUG("ast_scu_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_scu_write(u32 val, u32 reg) 
{
	SCUDBUG("ast_scu_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
#ifdef CONFIG_AST_SCU_LOCK
	//unlock 
	writel(SCU_PROTECT_UNLOCK, (void *)ast_scu_base);
	writel(val, (void *) (ast_scu_base + reg));
	//lock
	writel(0xaa,ast_scu_base);	
#else
	writel(SCU_PROTECT_UNLOCK, (void *) ast_scu_base);
	writel(val, (void *) (ast_scu_base + reg));
#endif
}

//SoC mapping Table 
struct soc_id {
	const char	* name;
	u32	rev_id;
};

#define SOC_ID(str, rev) { .name = str, .rev_id = rev, }

static struct soc_id soc_map_table[] = {
	SOC_ID("AST1100/AST2050-A0", 0x00000200),
	SOC_ID("AST1100/AST2050-A1", 0x00000201),
	SOC_ID("AST1100/AST2050-A2,3/AST2150-A0,1", 0x00000202),
	SOC_ID("AST1510/AST2100-A0", 0x00000300),
	SOC_ID("AST1510/AST2100-A1", 0x00000301),
	SOC_ID("AST1510/AST2100-A2,3", 0x00000302),
	SOC_ID("AST2200-A0,1", 0x00000102),
	SOC_ID("AST2300-A0", 0x01000003),
	SOC_ID("AST2300-A1", 0x01010303),
	SOC_ID("AST1300-A1", 0x01010003),
	SOC_ID("AST1050-A1", 0x01010203),
	SOC_ID("AST2400-A0", 0x02000303),
	SOC_ID("AST2400-A1", 0x02010303),
	SOC_ID("AST1010-A0", 0x03000003),
	SOC_ID("AST1010-A1", 0x03010003),
	SOC_ID("AST3200-A0", 0x04002003),
	SOC_ID("AST3200-A1", 0x04012003),
	SOC_ID("AST3200-A2", 0x04032003),
	SOC_ID("AST1520-A0", 0x03000203),	
	SOC_ID("AST1520-A1", 0x03010203),
	SOC_ID("AST2510-A0", 0x04000103),
	SOC_ID("AST2510-A1", 0x04010103),
	SOC_ID("AST2510-A2", 0x04030103),	
	SOC_ID("AST2520-A0", 0x04000203),
	SOC_ID("AST2520-A1", 0x04010203),
	SOC_ID("AST2520-A2", 0x04030203),
	SOC_ID("AST2500-A0", 0x04000303),	
	SOC_ID("AST2500-A1", 0x04010303),
	SOC_ID("AST2500-A2", 0x04030303),	
	SOC_ID("AST2530-A0", 0x04000403),
	SOC_ID("AST2530-A1", 0x04010403),
	SOC_ID("AST2530-A2", 0x04030403),	
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

#ifdef SCU_UART1CLK_STOP_EN
extern void
ast_scu_init_uart(u8 uart)
{
	u32 clk_stop_en = 0;	

	//uart 1
	if(uart & 0x2) {
		clk_stop_en |= SCU_UART1CLK_STOP_EN;
	}

	if(uart & 0x4) {
		clk_stop_en |= SCU_UART2CLK_STOP_EN;
	}

	if(uart & 0x8) {
		clk_stop_en |= SCU_UART3CLK_STOP_EN;
	}

	if(uart & 0x10) {
		clk_stop_en |= SCU_UART4CLK_STOP_EN;
	}
	
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~(clk_stop_en), AST_SCU_CLK_STOP);
	
}
EXPORT_SYMBOL(ast_scu_init_uart);
#endif

extern void
ast_scu_init_eth(u8 num)
{
	
}

#ifdef SCU_RESET_USB20
extern void
ast_scu_init_usb_port1(void)
{
	/* EHCI controller engine init. Process similar to VHub. */
	/* Following reset sequence can resolve "vhub dead on first power on" issue on V4 board. */
	//reset USB20
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_USB20, AST_SCU_RESET);

	//enable USB20 clock
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) | SCU_USB20_PHY_CLK_EN, AST_SCU_CLK_STOP);
	mdelay(10);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_USB20, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast_scu_init_usb_port1);
#endif

extern void
ast_scu_init_sdhci(void)
{

}
EXPORT_SYMBOL(ast_scu_init_sdhci);

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
ast_scu_init_hace(void)
{
}
EXPORT_SYMBOL(ast_scu_init_hace);

extern void
ast_scu_init_h264(void)
{

}
EXPORT_SYMBOL(ast_scu_init_h264);

/* 0:disable spi 1: enable spi master 2:enable spi master and spi slave to ahb 3: enable spi pass-through*/
extern void
ast_scu_spi_master(u8 mode)
{

}

EXPORT_SYMBOL(ast_scu_spi_master);

//***********************************CLK control***********************************
extern void
ast_scu_uart_div(void)
{
}

EXPORT_SYMBOL(ast_scu_uart_div);

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
	u32 clk=0;
	u32 h_pll_set = ast_scu_read(AST_SCU_H_PLL);

	if(h_pll_set & SCU_H_PLL_OFF)
		return 0;
	
	// Programming
	clk = ast_get_clk_source();
	if(h_pll_set & SCU_H_PLL_BYPASS_EN) {
		return clk;
	} else {
		//P = SCU24[18:13]
		//M = SCU24[12:5]
		//N = SCU24[4:0]
		//hpll = 24MHz * [(M+1) /(N+1)] / (P+1)
		clk = ((clk * (SCU_H_PLL_GET_MNUM(h_pll_set) + 1)) / (SCU_H_PLL_GET_NNUM(h_pll_set) + 1)) /(SCU_H_PLL_GET_PNUM(h_pll_set) + 1);
	}
	SCUDBUG("h_pll = %d\n",clk);
	return clk;
}

EXPORT_SYMBOL(ast_get_h_pll_clk);

extern u32
ast_get_m_pll_clk(void)
{
	u32 clk=0;
	u32 m_pll_set = ast_scu_read(AST_SCU_M_PLL);

	if(m_pll_set & SCU_M_PLL_OFF)
		return 0;
	
	// Programming
	clk = ast_get_clk_source();
	if(m_pll_set & SCU_M_PLL_BYPASS) {
		return clk;
	} else {
		//PD  == SCU20[13:18]
		//M  == SCU20[5:12]	
		//N  == SCU20[0:4]		
		//mpll =  24MHz * [(M+1) /(N+1)] / (P+1)
		clk = ((clk * (SCU_M_PLL_GET_MNUM(m_pll_set) + 1)) / (SCU_M_PLL_GET_NNUM(m_pll_set) + 1))/(SCU_M_PLL_GET_PDNUM(m_pll_set) + 1);
	}
	SCUDBUG("m_pll = %d\n",clk);
	return clk;
}

EXPORT_SYMBOL(ast_get_m_pll_clk);

extern u32
ast_get_ahbclk(void)
{
	unsigned int axi_div, ahb_div, hpll;

	hpll = ast_get_h_pll_clk();
	//AST2500 A1 fix 
	axi_div = 2;
	ahb_div = (SCU_HW_STRAP_GET_AXI_AHB_RATIO(ast_scu_read(AST_SCU_HW_STRAP1)) + 1); 
	
	SCUDBUG("HPLL=%d, AXI_Div=%d, AHB_DIV = %d, AHB CLK=%d\n", hpll, axi_div, ahb_div, (hpll/axi_div)/ahb_div);	
	return ((hpll/axi_div)/ahb_div);

}

EXPORT_SYMBOL(ast_get_ahbclk);

extern u32
ast_get_d2_pll_clk(void)
{
	return 10000000;
}

EXPORT_SYMBOL(ast_get_d2_pll_clk);

extern void
ast_set_d2_pll_clk(u32 pll_setting)
{
}

EXPORT_SYMBOL(ast_set_d2_pll_clk);

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

	return 0;
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
ast_scu_multi_func_eth(u8 num)
{
}

extern void
ast_scu_multi_func_nand(void)
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
ast_scu_multi_func_pwm_tacho(void)
{

}	

EXPORT_SYMBOL(ast_scu_multi_func_pwm_tacho);

//0 : usb 2.0 hub mode, 1:usb 2.0 host2 controller
extern void
ast_scu_multi_func_usb_port1_mode(u8 mode)
{
}	

EXPORT_SYMBOL(ast_scu_multi_func_usb_port1_mode);

//0 : 1.1 hid 1, 1.1 host , 2, 2.0 host 3, 2.0 device 
extern void
ast_scu_multi_func_usb_port2_mode(u8 mode)
{

}	

EXPORT_SYMBOL(ast_scu_multi_func_usb_port2_mode);

//0 : gpioQ6,7 mode , 1: usb1.1 host port 4 mode
extern void
ast_scu_multi_func_usb_port34_mode(u8 mode)
{

}	

EXPORT_SYMBOL(ast_scu_multi_func_usb_port34_mode);

//0 : 1: SD1 function 
extern void
ast_scu_multi_func_sdhc_8bit_mode(void)
{

}	

EXPORT_SYMBOL(ast_scu_multi_func_sdhc_8bit_mode);

extern void
ast_scu_multi_func_sdhc_slot(u8 slot)
{

}	

EXPORT_SYMBOL(ast_scu_multi_func_sdhc_slot);

//0: VGA , 1 : CRT, 2 : PASS through Port -A, 3 : PASS through Port -B  
extern void
ast_scu_set_crt_source(u8 dac_soource)
{

}

EXPORT_SYMBOL(ast_scu_set_crt_source);

extern void
ast_scu_multi_nic_switch(u8 enable)
{
		
}

extern void
ast_scu_multi_func_sgpio(void)
{

}	

EXPORT_SYMBOL(ast_scu_multi_func_sgpio);

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

	if(rest & SCU_SYS_EXT_RESET_FLAG) {
		SCUMSG("RST : External \n");
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_EXT_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
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


/*
* D[15:11] in 0x1E6E2040 is NCSI scratch from U-Boot. D[15:14] = MAC1, D[13:12] = MAC2
* The meanings of the 2 bits are:
* 00(0): Dedicated PHY
* 01(1): ASPEED's EVA + INTEL's NC-SI PHY chip EVA
* 10(2): ASPEED's MAC is connected to NC-SI PHY chip directly
* 11: Reserved
*/

extern u32
ast_scu_get_phy_config(u8 mac_num)
{

}
EXPORT_SYMBOL(ast_scu_get_phy_config);

extern u32
ast_scu_get_phy_interface(u8 mac_num)
{

}
EXPORT_SYMBOL(ast_scu_get_phy_interface);

extern u32
ast_scu_get_soc_dram_base(void)
{
	u32 rev_id = ast_scu_read(AST_SCU_REVISION_ID);
	if((rev_id >> AST_SOC_GEN) > 3) 
		return AST_DRAM_BASE_8;
	else
		return AST_DRAM_BASE_4;
}

extern int
ast_scu_2nd_wdt_mode(void)
{
	return(ast_scu_read(AST_SCU_HW_STRAP1) & SCU_HW_STRAP_2ND_BOOT_WDT);
}

extern u8
ast_scu_get_superio_addr_config(void)
{
	if(ast_scu_read(AST_SCU_HW_STRAP1) & SCU_HW_STRAP_SUPER_IO_CONFIG)
		return 0x4E;
	else
		return 0x2E;
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

#ifdef AST_SCU_OTP_READ_CTRL
extern u8
ast_scu_otp_read(u8 reg)
{
        ast_scu_write(SCU_OTP_TRIGGER | SCU_OTP_READ_ADDR(reg), AST_SCU_OTP_READ_CTRL);
        while(SCU_OTP_TRIGGER_STS & ast_scu_read(AST_SCU_OTP_READ_CTRL));
        return (SCU_OTP_READ_DATA(ast_scu_read(AST_SCU_OTP_READ_CTRL)));
}

EXPORT_SYMBOL(ast_scu_otp_read);
#endif
