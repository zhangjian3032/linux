/********************************************************************************
* File Name     : i2c-irq.c
* Author        : Ryan chen
* Description   : ASPEED I2C Device
*
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2012/07/30 ryan chen create this file
*
********************************************************************************/

#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <mach/ast_i2c.h>
#include <mach/ast-scu.h>
/*******************************************************************/
#define AST_I2CG_ISR				0x00

#define AST_I2C_DEV0_IRQ		0x1 
#define AST_I2C_DEV1_IRQ		(0x1 << 1)
#define AST_I2C_DEV2_IRQ		(0x1 << 2)
#define AST_I2C_DEV3_IRQ		(0x1 << 3)
#define AST_I2C_DEV4_IRQ		(0x1 << 4)
#define AST_I2C_DEV5_IRQ		(0x1 << 5)
#define AST_I2C_DEV6_IRQ		(0x1 << 6)
#define AST_I2C_DEV7_IRQ		(0x1 << 7)
#define AST_I2C_DEV8_IRQ		(0x1 << 8)
#define AST_I2C_DEV9_IRQ		(0x1 << 9)
#define AST_I2C_DEV10_IRQ		(0x1 << 10)
#define AST_I2C_DEV11_IRQ		(0x1 << 11)
#define AST_I2C_DEV12_IRQ		(0x1 << 12)
#define AST_I2C_DEV13_IRQ		(0x1 << 13)
#define AST_I2C_DEV14_IRQ		(0x1 << 14)

#define AST_I2CG_ISR_TARGET		0x08
#ifdef AST_SOC_G5
#define AST_I2CG_CTRL			0x0C
#define I2C_SRAM_BUFF_EN	0x1
#endif
/*******************************************************************/
//#define AST_I2C_IRQ_DEBUG

#ifdef AST_I2C_IRQ_DEBUG
#define DEBUG
#define I2C_IRQ_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define I2C_IRQ_DBUG(fmt, args...)
#endif

/*******************************************************************/
/*******************************************************************/

#if defined (AST_SOC_CAM)
struct buf_page page_info;

static void pool_buff_page_init(u32 buf_pool_addr) 
{
	page_info.flag = 0;
	page_info.page_size = AST_I2C_PAGE_SIZE;
	page_info.page_addr_point = 0;
	page_info.page_addr = buf_pool_addr;

}

extern u8 request_pool_buff_page(struct buf_page **req_page)
{
	*req_page = &page_info;
	return 0;

}

//TODO check free ?
extern void free_pool_buff_page(struct buf_page *req_page)
{
	return;
}

#elif defined (AST_SOC_G5)
struct buf_page page_info;

static void pool_buff_page_init(u32 buf_pool_addr) 
{
	page_info.flag = 0;
	page_info.page_size = AST_I2C_PAGE_SIZE;
	page_info.page_addr_point = 0;
	page_info.page_addr = buf_pool_addr;

}

extern u8 request_pool_buff_page(struct buf_page **req_page)
{
	*req_page = &page_info;
	return 0;

}

//TODO check free ?
extern void free_pool_buff_page(struct buf_page *req_page)
{
	return;
}

#elif defined(AST_SOC_G4)
#define I2C_PAGE_SIZE 8
struct buf_page page_info[I2C_PAGE_SIZE] = 
{  
	[0] = {
		.flag = 0,
		.page_no = 0,
		.page_size = 256,
		.page_addr_point = 0,	
	},
	[1] = {
		.flag = 0,
		.page_no = 1,			
		.page_size = 256,
		.page_addr_point = 0,		
	},
	[2] = {
		.flag = 0,
		.page_no = 2,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[3] = {
		.flag = 0,
		.page_no = 3,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[4] = {
		.flag = 0,
		.page_no = 4,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[5] = {
		.flag = 0,
		.page_no = 5,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[6] = {
		.flag = 0,
		.page_no = 6,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[7] = {
		.flag = 0,
		.page_no = 7,			
		.page_size = 256,
		.page_addr_point = 0,
	},
};

static void pool_buff_page_init(u32 buf_pool_addr) 
{
	u32 offset;
	int i ,j;

	for(i=0;i<I2C_PAGE_SIZE;i++) {
		offset = 0;
		for(j=0;j<i;j++)
			offset += page_info[i].page_size;
		
		page_info[i].page_addr = buf_pool_addr + offset;
//		I2CDBUG( "page[%d],addr :%x \n", i, page_info[i].page_addr);
	}

}

extern u8 request_pool_buff_page(struct buf_page **req_page)
{
	int i;
	//TODO
	spinlock_t	lock;
	spin_lock(&lock);	
	for(i=0;i<I2C_PAGE_SIZE;i++) {
		if(page_info[i].flag ==0) {
			page_info[i].flag = 1;
			*req_page = &page_info[i];
//			I2CDBUG( "request page addr %x \n", page_info[i].page_addr);
			break;
		}
	}
	spin_unlock(&lock);	
	return 0;
}

extern void free_pool_buff_page(struct buf_page *req_page)
{
	req_page->flag = 0;
//	I2CDBUG( "free page addr %x \n", req_page->page_addr);	
	req_page = NULL;
}

#elif defined (AST_SOC_G3)
#define I2C_PAGE_SIZE 5

struct buf_page page_info[I2C_PAGE_SIZE] = 
{  
	[0] = {
		.flag = 0,
		.page_size = 128,
	},
	[1] = {
		.flag = 0,
		.page_size = 32,
	},
	[2] = {
		.flag = 0,
		.page_size = 32,
	},
	[3] = {
		.flag = 0,
		.page_size = 32,
	},
	[4] = {
		.flag = 0,
		.page_size = 32,
	},
};

static void pool_buff_page_init(u32 buf_pool_addr) 
{

	u32 offset;
	int i ,j;

	for(i=0;i<I2C_PAGE_SIZE;i++) {
		offset = 0;
		for(j=0;j<i;j++)
			offset += page_info[i].page_size;
		
		page_info[i].page_addr = buf_pool_addr + offset;
		page_info[i].page_addr_point = page_info[i].page_addr/4;
//		I2C_IRQ_DBUG("page[%d],addr :%x , point : %d\n", i, page_info[i].page_addr, page_info[i].page_addr_point);
	}
}

extern u8 request_pool_buff_page(struct buf_page **req_page)
{
	int i;
	//TODO
	spinlock_t	lock;
	spin_lock(&lock);	
	for(i=0;i<I2C_PAGE_SIZE;i++) {
		if(page_info[i].flag ==0) {
			page_info[i].flag = 1;
			*req_page = &page_info[i];
			spin_unlock(&lock);
			return 1;
		}
	}
	spin_unlock(&lock);	
	return 0;

}

//TODO check free ?
extern void free_pool_buff_page(struct buf_page *req_page)
{
	req_page->flag = 0;
	req_page = NULL;
}

#else 
//DO nothing
static void pool_buff_page_init(void) {}
extern u8 request_pool_buff_page(struct buf_page **req_page) {return 0;}
extern void free_pool_buff_page(struct buf_page *req_page) {}
#endif
/*******************************************************************/



/*******************************************************************/



#if 0
#if defined (AST_I2C_POOL_BUFF_2048)
	buf_pool= ioremap(AST_I2C_BASE+0x800, 2048);
	if (!buf_pool) {
		printk("buf_pool ERROR \n");
		return -1;
	}
	pool_buff_page_init((u32)buf_pool); 
	
#elif defined (AST_I2C_POOL_BUFF_256)
	buf_pool = ioremap(AST_I2C_BASE+0x200, 256);
	if (!buf_pool) {
		printk("buf_pool ERROR \n");
		return -1;
	}
	pool_buff_page_init((u32)buf_pool); 

#elif defined (AST_I2C_POOL_BUFF_16)
	buf_pool = ioremap(AST_I2C_BASE+0x200, 224);
	if (!buf_pool) {
		printk("buf_pool ERROR \n");
		return -1;
	}
	pool_buff_page_init((u32)buf_pool); 

	
#else
	buf_pool = 0;
#endif
	
	return 0;
}
#endif
void ast_i2c_sram_buff_enable(struct ast_i2c_irq *i2c_irq) {
	writel(I2C_SRAM_BUFF_EN, i2c_irq->regs + AST_I2CG_CTRL);
};
EXPORT_SYMBOL(ast_i2c_sram_buff_enable);

static void ast_i2c_irq_handler(struct irq_desc *desc)
{
	struct ast_i2c_irq *i2c_irq = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status;
	unsigned int bus_irq;

	chained_irq_enter(chip, desc);
	status = readl(i2c_irq->regs + AST_I2CG_ISR);
	for_each_set_bit(bit, &status, i2c_irq->bus_num) {
		bus_irq = irq_find_mapping(i2c_irq->irq_domain, bit);
		generic_handle_irq(bus_irq);
	}
	chained_irq_exit(chip, desc);
}

static void noop(struct irq_data *data) { }

static unsigned int noop_ret(struct irq_data *data)
{
	return 0;
}

struct irq_chip i2c_irq_chip = {
	.name		= "i2c-irq",
	.irq_startup	= noop_ret,
	.irq_shutdown	= noop,
	.irq_enable	= noop,
	.irq_disable	= noop,
	.irq_ack	= noop,
	.irq_mask	= noop,
	.irq_unmask	= noop,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static int ast_i2c_map_irq_domain(struct irq_domain *domain,
					unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &i2c_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops ast_i2c_irq_domain_ops = {
	.map = ast_i2c_map_irq_domain,
};

static int __init ast_i2c_irq_of_init(struct device_node *node,
					struct device_node *parent)
{
	struct ast_i2c_irq *i2c_irq;

	i2c_irq = kzalloc(sizeof(*i2c_irq), GFP_KERNEL);
	if (!i2c_irq)
		return -ENOMEM;

	i2c_irq->regs = of_iomap(node, 0);
	if (IS_ERR(i2c_irq->regs))
		return PTR_ERR(i2c_irq->regs);

	node->data = i2c_irq;
	if (of_property_read_u32(node, "bus_num", &i2c_irq->bus_num) == 0) {
		printk("i2c_irq->bus_num = %d \n", i2c_irq->bus_num);
	}

	i2c_irq->parent_irq = irq_of_parse_and_map(node, 0);
	if (i2c_irq->parent_irq < 0)
		return i2c_irq->parent_irq;

	i2c_irq->irq_domain = irq_domain_add_linear(
			node, i2c_irq->bus_num,
			&ast_i2c_irq_domain_ops, NULL);
	if (!i2c_irq->irq_domain)
		return -ENOMEM;

	i2c_irq->irq_domain->name = "ast-i2c-domain";

	irq_set_chained_handler_and_data(i2c_irq->parent_irq,
					 ast_i2c_irq_handler, i2c_irq);

	pr_info("i2c-irq controller registered, irq %d\n", i2c_irq->parent_irq);

	return 0;
}

IRQCHIP_DECLARE(ast_i2c_irq, "aspeed,ast-i2c-irq", ast_i2c_irq_of_init);
