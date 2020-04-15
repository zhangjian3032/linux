#ifndef __ESPI_ASPEED_H_INCLUDED
#define __ESPI_ASPEED_H_INCLUDED

enum aspeed_espi_version {
	ESPI_AST2500,
	ESPI_AST2600,
};

struct aspeed_espi_xfer {
	unsigned int header;		//[23:12] len, [11:8] tag, [7:0] cycle type
	unsigned int buf_len;		//xfer buff len
	unsigned char	*xfer_buf;
};

struct espi_ch_data {
	int 	full;
	u32		header;	//[23:12] len, [11:8] tag, [7:0] cycle type
	u32		buf_len;
	u8		*buff;
	dma_addr_t dma_addr;
};

#define MAX_XFER_BUFF_SIZE	0x80		//128

#define ASPEED_ESPI_CTRL			0x00		/* Engine Control */
#define ASPEED_ESPI_STS				0x04		/* Engine Status */
#define ASPEED_ESPI_ISR				0x08		/* Interrupt Status */
#define ASPEED_ESPI_IER				0x0C		/* Interrupt Enable */
#define ASPEED_ESPI_PCP_RX_DMA		0x10		/* DMA Address of Peripheral Channel Posted Rx Package */
#define ASPEED_ESPI_PCP_RX_CTRL		0x14		/* Control of Peripheral Channel Posted Rx Package */
#define ASPEED_ESPI_PCP_RX_DATA		0x18		/* Data Port of Peripheral Channel Posted Rx Package */
#define ASPEED_ESPI_PCP_TX_DMA		0x20		/* DMA Address of Peripheral Channel Posted Tx Package */
#define ASPEED_ESPI_PCP_TX_CTRL		0x24		/* Control of Peripheral Channel Posted Tx Package */
#define ASPEED_ESPI_PCP_TX_DATA		0x28		/* Data Port of Peripheral Channel Posted Tx Package */
#define ASPEED_ESPI_PCNP_TX_DMA		0x30		/* DMA Address of Peripheral Channel Non-Posted Tx Package */
#define ASPEED_ESPI_PCNP_TX_CTRL	0x34		/* Control of Peripheral Channel Non-Posted Tx Package */
#define ASPEED_ESPI_PCNP_TX_DATA	0x38		/* Data Port of Peripheral Channel Non-Posted Tx Package */

#define ASPEED_ESPI_OOB_RX_DMA		0x40		/* DMA Address of OOB Channel Rx Package */
#define ASPEED_ESPI_OOB_RX_CTRL		0x44		/* Control of OOB Channel Rx Package */

#define ASPEED_ESPI_OOB_RX_DATA		0x48		/* Date port of OOB Channel Rx Package */
#define ASPEED_ESPI_OOB_TX_DMA		0x50		/* DMA Address of OOB Channel Tx Package */
#define ASPEED_ESPI_OOB_TX_CTRL		0x54		/* Control of OOB Channel Tx Package */
#define ASPEED_ESPI_OOB_TX_DATA		0x58		/* Date port of OOB Channel Tx Package */

#define ASPEED_ESPI_FLASH_RX_DMA	0x60		/* DMA Address of Flash Channel Rx Package */
#define ASPEED_ESPI_FLASH_RX_CTRL	0x64		/* Control of Flash Channel Rx Package */
#define ASPEED_ESPI_FLASH_RX_DATA	0x68		/* Date port of Flash Channel Rx Package */
#define ASPEED_ESPI_FLASH_TX_DMA	0x70		/* DMA Address of Flash Channel Tx Package */
#define ASPEED_ESPI_FLASH_TX_CTRL	0x74		/* Control of Flash Channel Tx Package */
#define ASPEED_ESPI_FLASH_TX_DATA	0x78		/* Date port of Flash Channel Tx Package */
#define ASPEED_ESPI_CTRL2			0x80		/* Engine Control 2 */
#define ASPEED_ESPI_PC_RX_SADDR		0x84		/* Mapping Source Address of Peripheral Channel Rx Package */
#define ASPEED_ESPI_PC_RX_TADDR		0x88		/* Mapping Target Address of Peripheral Channel Rx Package */
#define ASPEED_ESPI_PC_RX_TADDRM	0x8c		/* Mapping Target Address Mask of Peripheral Channel Rx Package */

#define ASPEED_ESPI_FLASH_TADDRM	0x90		/* Mapping Target Address Mask of Flash Channel */
#define ASPEED_ESPI_SYS_IER			0x94		/* Interrupt enable of System Event from Master */
#define ASPEED_ESPI_SYS_EVENT		0x98		/* System Event from and to Master */
#define ASPEED_ESPI_GPIO_VIRTCH		0x9C		/* GPIO through Virtual Wire Cahnnel  */

#define ASPEED_ESPI_GCAP_CONFIG		0xA0		/* General Capabilities and Configuration  */
#define ASPEED_ESPI_CH0CAP_CONFIG	0xA4		/* Channel 0 Capabilities and Configuration  */
#define ASPEED_ESPI_CH1CAP_CONFIG	0xA8		/* Channel 1 Capabilities and Configuration  */
#define ASPEED_ESPI_CH2CAP_CONFIG	0xAC		/* Channel 2 Capabilities and Configuration  */
#define ASPEED_ESPI_CH3CAP_CONFIG	0xB0		/* Channel 3 Capabilities and Configuration  */

#define ASPEED_ESPI_GPIO_DIR_VIRTCH	0xB4		/* GPIO Direction of Virtual Wire Channel  */
#define ASPEED_ESPI_GPIO_SEL_VIRTCH	0xB8		/* GPIO Selection of Virtual Wire Channel  */
#define ASPEED_ESPI_GPIO_REST_VIRTCH	0xBC		/* GPIO Reset Selection of Virtual Wire Channel  */

#define ASPEED_ESPI_SYS1_IER			0x100		/* Interrupt enable of System Event from Master */
#define ASPEED_ESPI_SYS1_EVENT			0x104		/* Interrupt enable of System Event from Master */

#define ASPEED_ESPI_SYS_INT_T0		0x110
#define ASPEED_ESPI_SYS_INT_T1		0x114
#define ASPEED_ESPI_SYS_INT_T2		0x118
#define ASPEED_ESPI_SYS_EVENT_ISR	0x11C


#define ASPEED_ESPI_SYS1_INT_T0		0x120
#define ASPEED_ESPI_SYS1_INT_T1		0x124
#define ASPEED_ESPI_SYS1_INT_T2		0x128
#define ASPEED_ESPI_SYS1_INT_STS	0x12C

#define ASPEED_ESPI_OOB_RX_RING_SIZE	0x130
#define ASPEED_ESPI_OOB_RX_READ_PT		0x134
#define ASPEED_ESPI_OOB_RX_WRITE_PT		0x138

#define ASPEED_ESPI_OOB_TX_RING_SIZE	0x140
#define ASPEED_ESPI_OOB_TX_READ_PT		0x144
#define ASPEED_ESPI_OOB_TX_WRITE_PT		0x148


/* ASPEED_ESPI_CTRL	-	0x00	:Engine Control */
#define ESPI_CTRL_FLASH_TX_SW_RESET		BIT(31)
#define ESPI_CTRL_FLASH_RX_SW_RESET		BIT(30)
#define ESPI_CTRL_OOB_TX_SW_RESET		BIT(29)
#define ESPI_CTRL_OOB_RX_SW_RESET		BIT(28)
#define ESPI_CTRL_PCNP_TX_SW_RESET		BIT(27)
#define ESPI_CTRL_PCNP_RX_SW_RESET		BIT(26)
#define ESPI_CTRL_PCP_TX_SW_RESET		BIT(25)
#define ESPI_CTRL_PCP_RX_SW_RESET		BIT(24)
#define ESPI_CTRL_FLASH_TX_DMA			BIT(23)
#define ESPI_CTRL_FLASH_RX_DMA			BIT(22)
#define ESPI_CTRL_OOB_TX_DMA			BIT(21)
#define ESPI_CTRL_OOB_RX_DMA			BIT(20)
#define ESPI_CTRL_PCNP_TX_DMA			BIT(19)
/* */
#define ESPI_CTRL_PCP_TX_DMA			BIT(17)
#define ESPI_CTRL_PCP_RX_DMA			BIT(16)
/* */
#define ESPI_CTRL_DIR_RESET				BIT(13)
#define ESPI_CTRL_VAL_RESET				BIT(12)
#define ESPI_CTRL_SW_FLASH_READ			BIT(10)
#define ESPI_CTRL_SW_GPIO_VIRTCH		BIT(9)

#define ESPI_CTRL_OOB_FW_RDY			BIT(4)

/* #define ASPEED_ESPI_ISR				0x08		Interrupt Status */
#define  ESPI_ISR_HW_RESET				BIT(31)
/* */
#define  ESPI_ISR_VIRTW_SYS1			BIT(22)
#define  ESPI_ISR_FLASH_TX_ERR			BIT(21)
#define  ESPI_ISR_OOB_TX_ERR			BIT(20)
#define  ESPI_ISR_FLASH_TX_ABORT		BIT(19)
#define  ESPI_ISR_OOB_TX_ABORT			BIT(18)
#define  ESPI_ISR_PCNP_TX_ABORT			BIT(17)
#define  ESPI_ISR_PCP_TX_ABORT			BIT(16)
#define  ESPI_ISR_FLASH_RX_ABORT		BIT(15)
#define  ESPI_ISR_OOB_RX_ABORT			BIT(14)
#define  ESPI_ISR_PCNP_RX_ABORT			BIT(13)
#define  ESPI_ISR_PCP_RX_ABORT			BIT(12)
#define  ESPI_ISR_PCNP_TX_ERR			BIT(11)
#define  ESPI_ISR_PCP_TX_ERR			BIT(10)
#define  ESPI_ISR_VIRTW_GPIO			BIT(9)
#define  ESPI_ISR_VIRTW_SYS				BIT(8)
#define  ESPI_ISR_FLASH_TX_COMP			BIT(7)
#define  ESPI_ISR_FLASH_RX_COMP			BIT(6)
#define  ESPI_ISR_OOB_TX_COMP			BIT(5)
#define  ESPI_ISR_OOB_RX_COMP			BIT(4)
#define  ESPI_ISR_PCNP_TX_COMP			BIT(3)
/* */
#define  ESPI_ISR_PCP_TX_COMP			BIT(1)
#define  ESPI_ISR_PCP_RX_COMP			BIT(0)

/* ASPEED_ESPI_PCP_RX_CTRL	-0x14	:	Control of Peripheral Channel Posted Rx Package */
#define ESPI_TRIGGER_PACKAGE			BIT(31)

#define ESPI_GET_CYCLE_TYPE(x)			(x & 0xff)
#define ESPI_GET_TAG(x)					((x >> 8) & 0xf)
#define ESPI_GET_LEN(x)					((x >> 12) & 0xfff)


/* #define ASPEED_ESPI_CTRL2			0x80		Engine Control 2 */
#define ESPI_DISABLE_PERP_MEM_READ		BIT(6)
#define ESPI_DISABLE_PERP_MEM_WRITE		BIT(4)


/* ASPEED_ESPI_SYS1_EVENT			0x104 : Interrupt enable of System Event from Master */
/* ASPEED_ESPI_SYS1_INT_STS		0x12C		*/
#define ESPI_SYS_SUS_ACK		BIT(20)

#define ESPI_SYS_SUS_WARN		BIT(0)

/* ASPEED_ESPI_SYS_EVENT			0x98		System Event from and to Master */
/* ASPEED_ESPI_SYS_EVENT_ISR		0x11C	System Event from and to Master interrupt sts */

#define ESPI_HOST_REST_ACK		BIT(27)

#define ESPI_REST_CPU_INIT		BIT(26)

#define ESPI_BOOT_STS			BIT(23)
#define ESPI_NFATEL_ERR			BIT(22)
#define ESPI_FATEL_ERR			BIT(21)
#define ESPI_BOOT_DWN			BIT(20)
#define ESPI_OOB_REST_ACK		BIT(16)

#define ESPI_HOST_NMI_OUT		BIT(10)
#define ESPI_HOST_SMI_OUT		BIT(9)

#define ESPI_HOST_RST_WARN		BIT(8)

#define ESPI_OOB_RST_WARN		BIT(6)

#define ESPI_SYS_S5_SLEEP		BIT(2)
#define ESPI_SYS_S4_SLEEP		BIT(1)
#define ESPI_SYS_S3_SLEEP		BIT(0)

/* ASPEED_ESPI_GCAP_CONFIG	0xA0		General Capabilities and Configuration  */
#define GET_GCAP_IO_MODE(x)		((x >> 26) & 0x3)
#define GET_GCAP_OP_FREQ(x)		((x >> 20) & 0x7)
#define GET_GCAP_CH_SUPPORT(x)	(x & 0xf)

#endif
