/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _AST_CAM_PLATFORM_H_
#define _AST_CAM_PLATFORM_H_

#define AST_DRAM_BASE					0x80000000

#define AST_SRAM_SIZE					(SZ_64K)

#define AST_AHBC_BASE					0x1E600000	/* AHB CONTROLLER */

#define AST_FMC_BASE        				0x1E620000	/* NEW SMC CONTROLLER */
#define AST_FMC_SPI0_BASE				0x1E630000	/* NEW SMC CONTROLLER */
#define AST_ISP0_BASE					0x1E640000	/* ISP1 Controller */
#define AST_ISP1_BASE					0x1E642000	/* ISP2 Controller */
#define AST_ISP2_BASE					0x1E644000	/* ISP3 Controller */
#define AST_ISP3_BASE					0x1E646000	/* ISP4 Controller */
#define AST_ISP4_BASE					0x1E648000	/* ISP5 Controller */
#define AST_ISP5_BASE					0x1E64A000	/* ISP6 Controller */

#define AST_MAC0_BASE					0x1E660000	/* MAC1 */ 

#define AST_VIC_BASE					0x1E6C0000	/* VIC */
#define AST_CVIC_BASE					0x1E6C2000	/* CVIC */

#define AST_UDC_BASE					0x1E6A0000	/* USB 2.0*/

#define AST_SDMC_BASE					0x1E6E0000	/* MMC SDRAM*/

#define AST_SCU_BASE					0x1E6E2000	/* SCU */
#define AST_CRYPTO_BASE				0x1E6E3000	/* Crypto */

#define AST_I2S_BASE					0x1E6E5000	/* I2S */

#define AST_MIPI0_BASE					0x1E6E8000	/* MIPI1 */
#define AST_MIPI1_BASE					0x1E6E9000	/* MIPI2 */
#define AST_MIPI2_BASE					0x1E6EA000	/* MIPI3 */
#define AST_MIPI3_BASE					0x1E6EB000	/* MIPI4 */
#define AST_MIPI4_BASE					0x1E6EC000	/* MIPI5 */
#define AST_MIPI5_BASE					0x1E6ED000	/* MIPI6 */

/* */

#define AST_JPEG_BASE              			0x1E700000	/* JPEG ENGINE */
#define AST_SRAM_BASE               			0x1E720000	/* SRAM */
#define AST_SDIO_BASE					0x1E740000	/* SDIO General Info */
#define AST_SDIO_SLOT0_BASE			0x1E740100	/* SDIO Slot 0*/

#define AST_SDHCI_BASE					0x1E760000	/* SDHCI General Info */
#define AST_SDHCI_SLOT0_BASE			0x1E760100	/* SDHCI0 Slot 0*/
#define AST_SDHCI_SLOT1_BASE			0x1E760200	/* SDHCI1 Slot 1*/

#define AST_GPIO_BASE					0x1E780000	/* GPIO */

#define AST_RTC_BASE					0x1E781000	/* RTC */
#define AST_TIMER_BASE              			0x1E782000	/* TIMER #0~2*/
#define AST_UART0_BASE              			0x1E783000	/* UART1 */
#define AST_UART1_BASE              			0x1E784000	/* UART2 */
#define AST_WDT_BASE                			0x1E785000	/* WDT */

#define AST_D3D_BASE					0x1E787000	/* D3D */
#define AST_PDM_BASE					0x1E788000	/* PDM */
#define AST_POST_BASE					0x1E789000	/* POST */

#define AST_I2C_BASE					0x1E78A000	/* I2C */
#define AST_I2C_DEV0_BASE				0x1E78A040	/* I2C DEV1 */
#define AST_I2C_DEV1_BASE				0x1E78A080	/* I2C DEV2 */
#define AST_I2C_DEV2_BASE				0x1E78A0C0	/* I2C DEV3 */
#define AST_I2C_DEV3_BASE				0x1E78A100	/* I2C DEV4 */
#define AST_I2C_DEV4_BASE				0x1E78A140	/* I2C DEV5 */
#define AST_I2C_DEV5_BASE				0x1E78A180	/* I2C DEV6 */
#define AST_I2C_DEV6_BASE				0x1E78A1C0	/* I2C DEV7 */
#define AST_I2C_DEV7_BASE				0x1E78A300	/* I2C DEV8 */
#define AST_I2C_DEV8_BASE				0x1E78A340	/* I2C DEV9 */
#define AST_I2C_DEV9_BASE				0x1E78A380	/* I2C DEV10 */
#define AST_I2C_DEV10_BASE				0x1E78A3C0	/* I2C DEV11 */
#define AST_I2C_DEV11_BASE				0x1E78A400	/* I2C DEV12 */
#define AST_I2C_DEV12_BASE				0x1E78A440	/* I2C DEV13 */
#define AST_I2C_DEV13_BASE				0x1E78A480	/* I2C DEV14 */

#define AST_UART2_BASE					0x1E78D000	/* UART3 */

#define AST_UART_SDMA_BASE			0x1E78E000    /* UART SDMA */

#define AST_H264_BASE					0x1E7A0000	/* H.264 */

#define AST_ZSP_BASE					0x1E7C0000	/* ZSP */

#define AST_FMC_CS0_BASE				0x20000000	/* CS0 */
#define AST_FMC_CS1_BASE				0x28000000	/* CS1 */
#define AST_FMC_CS2_BASE				0x2a000000	/* CS2 */

#define AST_SPI0_CS0_BASE				0x30000000	/* SPI 2 Flash CS 0 Memory */
#define AST_SPI0_CS1_BASE				0x32000000	/* SPI 2 Flash CS 1 Memory */

#define AST_SPI1_CS0_BASE				0x38000000	/* SPI 3 Flash CS 0 Memory */
#define AST_SPI1_CS1_BASE				0x3a000000	/* SPI 3 Flash CS 1 Memory */

#define AST_JPEG_MEM_SIZE				0x4000000	//64MB
#define AST_JPEG_MEM 					(AST_DRAM_BASE + SZ_256M)	//(AST_DRAM_BASE + SZ_256M)

#define AST_H264_MEM_SIZE				0x1100000 		
#define AST_H264_MEM_BASE 				(AST_JPEG_MEM + AST_JPEG_MEM_SIZE)

#endif
