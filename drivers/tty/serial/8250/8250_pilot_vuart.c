// SPDX-License-Identifier: GPL-2.0+
/*
 *  Serial Port driver for Pilot VUART device
 *
 *    Copyright (C) 2019 Aspeed Technology Inc
 *    Shivah Shankar S <shivahs@aspeedtech.com>
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "8250.h"

struct pilot_vuart {
	struct device		*dev;
	void __iomem		*regs;
	struct regmap		*upctl_reg;
	struct clk		*clk;
	int			line;
	//struct timer_list	unthrottle_timer; dont need this!!
	struct uart_8250_port	*port;
};

/*
 * The VUART is basically two UART 'front ends' connected by their FIFO
 * (no actual serial line in between). One is on the BMC side (management
 * controller) and one is on the host CPU side.
 *
 * It allows the BMC to provide to the host a "UART" that pipes into
 * the BMC itself and can then be turned by the BMC into a network console
 * of some sort for example.
 *
 * This driver is for the BMC side. The sysfs files allow the BMC
 * userspace which owns the system configuration policy, to specify
 * at what IO port and interrupt number the host side will appear
 * to the host on the Host <-> BMC LPC bus. It could be different on a
 * different system (though most of them use 3f8/4).
 */
#define UPCTL_OFFSET	0x2C
#define VUART1		0x3
#define VUART2		0xC
static void pilot_vuart_enable(struct pilot_vuart *vuart, bool enabled)
{

	//TODO - Get the address of the UPCTL register based 
	//on which UART enable the appropriate bits
	if (enabled){

		printk("Enable line %d\n", vuart->line);
		if(vuart->line == 1) {
			regmap_update_bits(vuart->upctl_reg, UPCTL_OFFSET,
							VUART1, 0);
		} else {
			regmap_update_bits(vuart->upctl_reg, UPCTL_OFFSET,
							VUART2, 0);
		}

	} else {
		printk("Disable Vuart line %d\n", vuart->line);
		if(vuart->line == 1){
			regmap_update_bits(vuart->upctl_reg, UPCTL_OFFSET,
							VUART1, VUART1);
		} else {
			regmap_update_bits(vuart->upctl_reg, UPCTL_OFFSET,
							VUART2, VUART2);
		}
	}
}

static int pilot_vuart_probe(struct platform_device *pdev)
{
	struct uart_8250_port port;
	struct pilot_vuart *vuart;
	struct device_node *np;
	struct resource *res;
	u32 clk, prop;
	int rc;

	np = pdev->dev.of_node;

	vuart = devm_kzalloc(&pdev->dev, sizeof(*vuart), GFP_KERNEL);
	if (!vuart)
		return -ENOMEM;

	vuart->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vuart->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(vuart->regs))
		return PTR_ERR(vuart->regs);

	memset(&port, 0, sizeof(port));
	port.port.private_data = vuart;
	port.port.membase = vuart->regs;
	port.port.mapbase = res->start;
	port.port.mapsize = resource_size(res);
#if 0
	port.port.startup = aspeed_vuart_startup;
	port.port.shutdown = aspeed_vuart_shutdown;
	port.port.throttle = aspeed_vuart_throttle;
	port.port.unthrottle = aspeed_vuart_unthrottle;
#endif
	port.port.status = UPSTAT_SYNC_FIFO;
	port.port.dev = &pdev->dev;

	of_property_read_u32(np, "clock-frequency", &clk);

#if 0
	//TODO - DO whatever was done for debug UART in dts file
	if (of_property_read_u32(np, "clock-frequency", &clk)) {
		vuart->clk = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(vuart->clk)) {
			dev_warn(&pdev->dev,
				"clk or clock-frequency not defined\n");
			rc = PTR_ERR(vuart->clk);
			goto err_ret;
		}

		rc = clk_prepare_enable(vuart->clk);
		if (rc < 0)
			goto err_ret;

		clk = clk_get_rate(vuart->clk);
	}
#endif
	/* If current-speed was set, then try not to change it. */
	if (of_property_read_u32(np, "current-speed", &prop) == 0)
		port.port.custom_divisor = clk / (16 * prop);

	/* Check for shifted address mapping */
	if (of_property_read_u32(np, "reg-offset", &prop) == 0)
		port.port.mapbase += prop;

	/* Check for registers offset within the devices address range */
	if (of_property_read_u32(np, "reg-shift", &prop) == 0)
		port.port.regshift = prop;

	/* Check for fifo size */
	if (of_property_read_u32(np, "fifo-size", &prop) == 0)
		port.port.fifosize = prop;

	/* Check for a fixed line number */
	rc = of_alias_get_id(np, "serial");
	if (rc >= 0)
		port.port.line = rc;

	//Get the UPCTL resource
#if 0
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	vuart->upctl_reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(vuart->upctl_reg))
		return PTR_ERR(vuart->upctl_reg);
	vuart->upctl_reg = syscon_node_to_regmap(
                        pdev->dev.parent->of_node);
#endif
	vuart->upctl_reg = syscon_regmap_lookup_by_phandle(np,
		"syscon");
	 if (IS_ERR(vuart->upctl_reg)) {
                dev_err(&pdev->dev, "Couldn't get regmap\n");
                return -ENODEV;
        }

	port.port.irq = irq_of_parse_and_map(np, 0);
	port.port.irqflags = IRQF_SHARED;
	//port.port.handle_irq = aspeed_vuart_handle_irq; may not be needed
	port.port.iotype = UPIO_MEM;
	port.port.type = PORT_16550A;
	port.port.uartclk = clk;
	port.port.flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF
		| UPF_FIXED_PORT | UPF_FIXED_TYPE | UPF_NO_THRE_TEST;

	if (of_property_read_bool(np, "no-loopback-test"))
		port.port.flags |= UPF_SKIP_TEST;

	if (port.port.fifosize)
		port.capabilities = UART_CAP_FIFO;

	if (of_property_read_bool(np, "auto-flow-control"))
		port.capabilities |= UART_CAP_AFE;

	rc = serial8250_register_8250_port(&port);
	if (rc < 0)
		goto err_clk_disable;

	vuart->line = rc;

	pilot_vuart_enable(vuart, true);
	platform_set_drvdata(pdev, vuart);

	return 0;

err_clk_disable:
	//clk_disable_unprepare(vuart->clk);
	irq_dispose_mapping(port.port.irq);
err_ret:
	return rc;
}

static int pilot_vuart_remove(struct platform_device *pdev)
{
	struct pilot_vuart *vuart = platform_get_drvdata(pdev);

	//TODO - Check if this is needed
	//del_timer_sync(&vuart->unthrottle_timer);
	pilot_vuart_enable(vuart, false);
	serial8250_unregister_port(vuart->line);
	//TODO - Check if this is needed
	//clk_disable_unprepare(vuart->clk);

	return 0;
}

static const struct of_device_id pilot_vuart_table[] = {
	{ .compatible = "aspeed,pilot-vuart" },
	{ },
};

static struct platform_driver pilot_vuart_driver = {
	.driver = {
		.name = "pilot-vuart",
		.of_match_table = pilot_vuart_table,
	},
	.probe = pilot_vuart_probe,
	.remove = pilot_vuart_remove,
};

module_platform_driver(pilot_vuart_driver);

MODULE_AUTHOR("Shivah Shankar S <shivahs@aspeedtech.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for Aspeed's Pilot VUART device");
