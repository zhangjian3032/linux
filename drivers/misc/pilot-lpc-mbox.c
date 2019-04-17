
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 Aspeed Technology Inc
 * Shivah Shankar S <shivahs@aspeedtech.com>
 *
 */

#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
/*
#define DEVICE_NAME	"pilot-mbox"
*/
#define DEVICE_NAME	"aspeed-mbox"

#define PILOT_MBOX_NUM_REGS 16
#define MBDAT015	0x1420
#define MBST0		0x1430
#define MBST1		0x1431
#define MBBINT		0x1432
#define MBHINT		0x1433
#define MBHIE0		0x1434
#define MBHIE1		0x1435
#define MBBIE0		0x1436
#define MBBIE1		0x1437
#define KCSISR		0x1438
#define KCSIER		0x1439

struct pilot_mbox {
	struct miscdevice	miscdev;
	struct regmap		*regmap;
	unsigned int		base;
	wait_queue_head_t	queue;
	struct mutex		mutex;
};

static atomic_t pilot_mbox_open_count = ATOMIC_INIT(0);

static void pilot_regmap_read8(struct regmap * map, 
					u32 offset, u8 * val)
{
	u32 temp_val;
#if 0
	u8 b_pos = offset & 3;
	regmap_read(map, (offset&~(3)), &temp_val);
	printk("Read val is %x\n", temp_val);
	*val = ((temp_val) >> (b_pos << 3)) & 0xFF;
#endif
	regmap_read(map, offset,  &temp_val);
	*val = (temp_val & 0xFF);
}
static void pilot_regmap_write8(struct regmap * map,
					u32 offset, u8 mask, u8 val)
{
	u32 temp_val;
	bool change;
#if 0
	bool change;
	u32 msk=0;
	u8 b_pos = offset & 3;
	regmap_read(map, offset&~(3), &temp_val);
	temp_val |= (((val) << (b_pos << 3)));
	printk("The value written is %x to be written is %x\n", temp_val, val);
	temp_val =0;
	temp_val |= (((val) << (b_pos << 3)));
	msk |= (((mask) << (b_pos << 3)));
	printk("mask is %x tval is %x val is %x\n", msk, temp_val, val);
	regmap_update_bits_base(map, (offset&~(3)), msk, temp_val,
				&change, false, true);
	/*regmap_update_bits(map, (offset&~(3)), temp_val, temp_val);*/
#endif
	temp_val = val;
	regmap_update_bits_base(map, offset, mask, temp_val,
				&change, false, true);
}
static u8 pilot_mbox_inb(struct pilot_mbox *mbox, int reg)
{
	u8 val = 0; 
#if 0
	int rc = regmap_read(mbox->regmap, mbox->base + reg, &val);

	if (rc)
		dev_err(mbox->miscdev.parent, "regmap_read() failed with "
				"%d (reg: 0x%08x)\n", rc, reg);
#endif
	pilot_regmap_read8(mbox->regmap, reg, &val);
	return val;
} 

static void pilot_mbox_outb(struct pilot_mbox *mbox, unsigned int reg, 
						u8 mask, u8 data)
{
	pilot_regmap_write8(mbox->regmap, reg, mask, data);
#if 0
	int rc = regmap_write(mbox->regmap, mbox->base + reg, data);

	if (rc)
		dev_err(mbox->miscdev.parent, "regmap_write() failed with "
				"%d (data: %u reg: 0x%08x)\n", rc, data, reg);
#endif
}

static struct pilot_mbox *file_mbox(struct file *file)
{
	return container_of(file->private_data, struct pilot_mbox, miscdev);
}

static int pilot_mbox_open(struct inode *inode, struct file *file)
{
	struct pilot_mbox *mbox = file_mbox(file);
	u8 val;
	if (atomic_inc_return(&pilot_mbox_open_count) == 1) {

		//Clear all old status
		pilot_mbox_outb(mbox, MBST0, (0xFF), 0xFF);
		pilot_mbox_outb(mbox, MBST1, (0xFF), 0xFF);

		//Clear any old interrupts from Host	
		pilot_mbox_outb(mbox, MBBINT, 0x1, (u8)~(0x1));
		pilot_mbox_outb(mbox, MBBINT, 0x80, (0x80));

		//Enable all data register interrupts
		pilot_mbox_outb(mbox, MBBIE0, (0xFF), 0xFF);
		pilot_mbox_outb(mbox, MBBIE1, (0xFF), 0xFF);

		//Unmask interrupts from Host
		pilot_mbox_outb(mbox, MBBINT, 0x2, (u8)~(0x2));

		//Enable the top most level interrupt from Mailbox
		val = pilot_mbox_inb(mbox, KCSIER);
		pilot_mbox_outb(mbox, KCSIER, (0x80 | val), (0x80 | val));
		
		return 0;
	}

	atomic_dec(&pilot_mbox_open_count);
	return -EBUSY;
}

static ssize_t pilot_mbox_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct pilot_mbox *mbox = file_mbox(file);
	char __user *p = buf;
	ssize_t ret;
	int i;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	if (count + *ppos > PILOT_MBOX_NUM_REGS)
		return -EINVAL;

#if 1
	if (file->f_flags & O_NONBLOCK) {
		if ((!(pilot_mbox_inb(mbox, MBST0) &
				0xFF)) && (!(pilot_mbox_inb(mbox, MBST1) &
                                0xFF)))
			return -EAGAIN;
	} else if (wait_event_interruptible(mbox->queue,
				(!(pilot_mbox_inb(mbox, MBST0) &
                                0xFF) && !(pilot_mbox_inb(mbox, MBST1) &
                                0xFF)))) {
		return -ERESTARTSYS;
	}
#endif

	mutex_lock(&mbox->mutex);

	for (i = *ppos; count > 0 && i < PILOT_MBOX_NUM_REGS; i++) {
		uint8_t reg = pilot_mbox_inb(mbox, MBDAT015 + (i));

		ret = __put_user(reg, p);
		if (ret)
			goto out_unlock;

		//Write a 1 to the appropriate bits status register to clear
		if(i <= 7)
			pilot_mbox_outb(mbox, MBST0, (1<<i), (1<<i));
		else
			pilot_mbox_outb(mbox, MBST1, (1<<(i-8)), (1<<(i-8)));

		p++;
		count--;
	}
	//Enable all data register interrupts that we had disabled in irq routine
	pilot_mbox_outb(mbox, MBBIE0, 0xFF, (0xFF));
	pilot_mbox_outb(mbox, MBBIE1, 0xFF, (0xFF));
	
	ret = p - buf;

out_unlock:
	//Unmask the interrupt from host now as we are ready
	pilot_mbox_outb(mbox, MBBINT, 2, ~(2));
	
	mutex_unlock(&mbox->mutex);
	return ret;
}


static ssize_t pilot_mbox_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct pilot_mbox *mbox = file_mbox(file);
	const char __user *p = buf;
	ssize_t ret;
	char c;
	int i;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	if (count + *ppos > PILOT_MBOX_NUM_REGS)
		return -EINVAL;

	mutex_lock(&mbox->mutex);

	for (i = *ppos; count > 0 && i < PILOT_MBOX_NUM_REGS; i++) {
		ret = __get_user(c, p);
		if (ret)
			goto out_unlock;

		pilot_mbox_outb(mbox, MBDAT015 + (i), 0xFF, c);
		p++;
		count--;
	}
	ret = p - buf;

out_unlock:
	mutex_unlock(&mbox->mutex);
	return ret;
}
static unsigned int pilot_mbox_poll(struct file *file, poll_table *wait)
{
	struct pilot_mbox *mbox = file_mbox(file);
	unsigned int mask = 0;

	poll_wait(file, &mbox->queue, wait);

	if (((pilot_mbox_inb(mbox, MBST0) & 0xFF)) || 
		((pilot_mbox_inb(mbox, MBST1) & 0xFF))) {
		
		mask |= POLLIN;

	}

	return mask;
}


static int pilot_mbox_release(struct inode *inode, struct file *file)
{
	atomic_dec(&pilot_mbox_open_count);
	return 0;
}

static const struct file_operations pilot_mbox_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_seek_end_llseek,
	.read		= pilot_mbox_read,
	.write		= pilot_mbox_write,
	.open		= pilot_mbox_open,
	.release	= pilot_mbox_release,
	.poll		= pilot_mbox_poll,
};

static irqreturn_t pilot_mbox_irq(int irq, void *arg)
{
	struct pilot_mbox *mbox = arg;
	/* TODO - Check if it is Mailbox interrupt, otherwise return IRQ_NONE*/
	/* TODO - Also if there is data to be read mask off those interrupt 
	 * that way we can clear it once the data is read
	 */

	printk("Inside mbox IRQ\n");
	if(!(pilot_mbox_inb(mbox, KCSISR) & 0x80)){
		return IRQ_NONE;
	}
	if (((pilot_mbox_inb(mbox, MBST0) & 0xFF)) || 
		((pilot_mbox_inb(mbox, MBST1) & 0xFF))) {

		//Note here we ignore the interrupt from MBBINT
		//as this driver framework seems to not use that interrupt
		//However we Mask interrupts from Host just in case
		pilot_mbox_outb(mbox, MBBINT, 0x2, (0x2));

		/* Disable the interrupts from data register till we process
		 * them in the queue and then enable them after processing
		 * them */
		pilot_mbox_outb(mbox, MBBIE0,0xFF, (u8) ~(0xFF));
		pilot_mbox_outb(mbox, MBBIE1, 0xFF, (u8)~(0xFF));

		printk("Got MBOX interrrupt waking the queue\n");
		wake_up(&mbox->queue);
	}
	return IRQ_HANDLED;
}
static irqreturn_t pilot_swc_irq(int irq, void *arg)
{
	struct pilot_mbox *mbox = arg;
	u8 swc_val;
	u8 val;
	bool change;

	pilot_regmap_read8(mbox->regmap, 0xa, &swc_val);
	//Acknowledge the SWC interrupt
	regmap_update_bits_base(mbox->regmap, 0xa, (1<<2), (1<<2),
				&change, false, true);
	if(swc_val & 0x8){
#if 0
		//Clear all old status
		pilot_mbox_outb(mbox, MBST0, 0xFF, (0xFF));
		pilot_mbox_outb(mbox, MBST1, 0xFF, (0xFF));

		//Clear any old interrupts from Host	
		pilot_mbox_outb(mbox, MBBINT, 0x1, (u8)~(0x1));
		pilot_mbox_outb(mbox, MBBINT, 0x80, (0x80));
#endif
		//Enable all data register interrupts
		pilot_mbox_outb(mbox, MBBIE0, 0xFF, (u8)(0xFF));
		pilot_mbox_outb(mbox, MBBIE1, 0xFF, (u8)(0xFF));

		//Mask interrupts from Host
		pilot_mbox_outb(mbox, MBBINT, 0x2, (u8)~(0x2));

		val = pilot_mbox_inb(mbox, KCSIER);
		//Disable the top most level interrupt from Mailbox
		pilot_mbox_outb(mbox, KCSIER, (0x80|val), (0x80 | val));
	}else{
		//do nothing
	}
	return IRQ_HANDLED;
}
static irqreturn_t pilot_lpcrst_irq(int irq, void *arg)
{
	struct pilot_mbox *mbox = arg;
	u8 swc_val;
	u8 val;
	bool change;

	printk("Got LPC reset interrupt in MBOX\n");
	pilot_regmap_read8(mbox->regmap, 0x1523, &swc_val);
	if((swc_val & 0x1) != 0x1){
		printk("LPC reset NOT active in MBOX\n");
		regmap_update_bits_base(mbox->regmap, 0x1523, 
				(0xf7), ((swc_val)),
				&change, false, true);
		return IRQ_HANDLED;
	}
	regmap_update_bits_base(mbox->regmap, 0x1523, (0xf7), ((swc_val)),
				&change, false, true);
#if 0
	//Clear all old status
	pilot_mbox_outb(mbox, MBST0, (0xFF), 0xFF);
	pilot_mbox_outb(mbox, MBST1, (0xFF), 0xFF);

	//Clear any old interrupts from Host	
	pilot_mbox_outb(mbox, MBBINT, 0x1, (u8)~(0x1));
	pilot_mbox_outb(mbox, MBBINT, 0x80, (0x80));
#endif
	//Enable all data register interrupts
	pilot_mbox_outb(mbox, MBBIE0, (0xFF), 0xFF);
	pilot_mbox_outb(mbox, MBBIE1, (0xFF), 0xFF);

	//For testing
	//Unmask interrupts from Host
	//pilot_mbox_outb(mbox, MBBINT, 0x2, (u8)~(0x2));
	pilot_mbox_outb(mbox, MBBINT, 0x2, (u8)(0x2));

	//Enable the top most level interrupt from Mailbox
	val = pilot_mbox_inb(mbox, KCSIER);
	pilot_mbox_outb(mbox, KCSIER, (0x80 | val), (0x80 | val));
	return IRQ_HANDLED;
}
static int pilot_mbox_config_irq(struct pilot_mbox *mbox,
		struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc, irq;

	irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!irq)
		return -ENODEV;

	rc = devm_request_irq(dev, irq, pilot_mbox_irq,
			IRQF_SHARED, DEVICE_NAME, mbox);
	if (rc < 0) {
		dev_err(dev, "Unable to request IRQ %d\n", irq);
		return rc;
	}

	irq = irq_of_parse_and_map(dev->of_node, 1);
	if (!irq)
		return -ENODEV;

	/* Here enable the SWC interrupt*/
	pilot_regmap_write8(mbox->regmap, 0x4, 0x1, 0x1);

	rc = devm_request_irq(dev, irq, pilot_swc_irq,
			IRQF_SHARED, DEVICE_NAME, mbox);
	if (rc < 0) {
		dev_err(dev, "Unable to request IRQ %d\n", irq);
		return rc;
	}

	irq = irq_of_parse_and_map(dev->of_node, 2);
	if (!irq)
		return -ENODEV;

	rc = devm_request_irq(dev, irq, pilot_lpcrst_irq,
			IRQF_SHARED, DEVICE_NAME, mbox);
	if (rc < 0) {
		dev_err(dev, "Unable to request IRQ %d\n", irq);
		return rc;
	}

	//Clear all old status
	pilot_mbox_outb(mbox, MBST0, 0xFF, (0xFF));
	pilot_mbox_outb(mbox, MBST1, 0xFF, (0xFF));

	//Clear any old interrupts from Host	
	pilot_mbox_outb(mbox, MBBINT, 0x1, (u8)~(0x1));
	pilot_mbox_outb(mbox, MBBINT, 0x80, (0x80));

	//Disable all data register interrupts
	pilot_mbox_outb(mbox, MBBIE0, 0xFF, (u8)~(0xFF));
	pilot_mbox_outb(mbox, MBBIE1, 0xFF, (u8)~(0xFF));

	//Mask interrupts from Host
	pilot_mbox_outb(mbox, MBBINT, 0x2, (0x2));

	//Disable the top most level interrupt from Mailbox
	pilot_mbox_outb(mbox, KCSIER, 0x80, (u8)~(0x80));

	return 0;
}
static struct regmap_config pilot_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_stride = 1,
};
static int pilot_mbox_probe(struct platform_device *pdev)
{
	struct pilot_mbox *mbox;
	struct device *dev;
	int rc;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *node;

	dev = &pdev->dev;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, mbox);
#if 0
	rc = of_property_read_u32(dev->of_node, "reg", &mbox->base);
	if (rc) {
		dev_err(dev, "Couldn't read reg device-tree property\n");
		return rc;
	}
	mbox->regmap = syscon_node_to_regmap(
			pdev->dev.parent->of_node);
	mbox->regmap = syscon_regmap_lookup_by_phandle(np, "syscon-lpc");
	if (IS_ERR(mbox->regmap)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}
#endif
	node = of_find_compatible_node(NULL, NULL, "aspeed,pilot-syscon");
	
	mbox->base = of_iomap(node, 0);

	mbox->regmap = regmap_init_mmio(NULL, mbox->base, &pilot_config);

	mutex_init(&mbox->mutex);
	init_waitqueue_head(&mbox->queue);

	mbox->miscdev.minor = MISC_DYNAMIC_MINOR;
	mbox->miscdev.name = DEVICE_NAME;
	mbox->miscdev.fops = &pilot_mbox_fops;
	mbox->miscdev.parent = dev;
	rc = misc_register(&mbox->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	rc = pilot_mbox_config_irq(mbox, pdev);
	if (rc) {
		dev_err(dev, "Failed to configure IRQ\n");
		misc_deregister(&mbox->miscdev);
		return rc;
	}

	return 0;
}

static int pilot_mbox_remove(struct platform_device *pdev)
{
	struct pilot_mbox *mbox = dev_get_drvdata(&pdev->dev);

	misc_deregister(&mbox->miscdev);

	return 0;
}

static const struct of_device_id pilot_mbox_match[] = {
	{ .compatible = "aspeed,pilot-mbox" },
	{ },
};

static struct platform_driver pilot_mbox_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.of_match_table = pilot_mbox_match,
	},
	.probe = pilot_mbox_probe,
	.remove = pilot_mbox_remove,
};

module_platform_driver(pilot_mbox_driver);

MODULE_DEVICE_TABLE(of, pilot_mbox_match);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Shivah Shankar S <shivahs@aspeedtech.com>");
MODULE_DESCRIPTION("Pilot mailbox device driver");
