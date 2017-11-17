/*
 * This header provides ASPEED AST SOC constants for binding
 */

#ifndef _DT_BINDINGS_RESET_AST_CAM_H
#define _DT_BINDINGS_RESET_AST_CAM_H

/* AST_SCU_RESET : 0x04	 - system reset control register */
#define SCU_RESET_MIPI				(20)
#define SCU_RESET_PWM				(19)
#define SCU_RESET_SC				(18)
#define SCU_RESET_OSD				(17) 
#define SCU_RESET_EE				(16)
#define SCU_RESET_WDR				(15)
#define SCU_RESET_HE				(14)
#define SCU_RESET_PP				(13)
#define SCU_RESET_USB20				(12)
#define SCU_RESET_MAC				(11)
#define SCU_RESET_SDHCI				(10)
#define SCU_RESET_SDIO				(9)
#define SCU_RESET_I2S				(8)
#define SCU_RESET_H264				(7)
#define SCU_RESET_JPEG				(6)
#define SCU_RESET_3D				(5)
#define SCU_RESET_HACE				(4)
#define SCU_RESET_ISP				(3)
#define SCU_RESET_I2C				(2)
#define SCU_RESET_AHB				(1)
#define SCU_RESET_SRAM_CTRL			(0)

#endif	/* _DT_BINDINGS_RESET_AST_CAM_H */
