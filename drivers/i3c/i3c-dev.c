#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i3c/master.h>
#include <linux/i3c/device.h>
#include <linux/i3c/ccc.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/compat.h>

#define I3C_MAJOR	89		/* Device major number		*/

#define I3C_NAME_SIZE	20
#define I3C_MODULE_PREFIX "i2c:"

extern struct bus_type i3c_bus_type;

///

#define to_i3c_adapter(d) container_of(d, struct i3c_adapter, dev)


struct i3c_msg {
	__u16 addr;	/* slave address			*/
	__u16 flags;
#define I3C_M_RD		0x0001	/* read data, from slave to master */
					/* I3C_M_RD is guaranteed to be 0x0001! */
#define I3C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I3C_M_DMA_SAFE		0x0200	/* the buffer of this message is DMA safe */
					/* makes only sense in kernelspace */
					/* userspace buffers are copied anyway */
#define I3C_M_RECV_LEN		0x0400	/* length will be first received byte */
#define I3C_M_NO_RD_ACK		0x0800	/* if I3C_FUNC_PROTOCOL_MANGLING */
#define I3C_M_IGNORE_NAK	0x1000	/* if I3C_FUNC_PROTOCOL_MANGLING */
#define I3C_M_REV_DIR_ADDR	0x2000	/* if I3C_FUNC_PROTOCOL_MANGLING */
#define I3C_M_NOSTART		0x4000	/* if I3C_FUNC_NOSTART */
#define I3C_M_STOP		0x8000	/* if I3C_FUNC_PROTOCOL_MANGLING */
	__u16 len;		/* msg length				*/
	__u8 *buf;		/* pointer to msg data			*/
};

struct i3c_adapter {
	struct module *owner;
	unsigned int class;		  /* classes to allow probing for */
	const struct i3c_master_controller_ops *ops; 

	struct device dev;		/* the adapter device */

	int nr;
	char name[48];
	struct completion dev_released;

	struct mutex userspace_clients_lock;
	struct list_head userspace_clients;

};
#define to_i2c_adapter(d) container_of(d, struct i2c_adapter, dev)

void i3c_put_adapter(struct i3c_adapter *adap)
{
	if (!adap)
		return;

	put_device(&adap->dev);
	module_put(adap->owner);
}

static inline void *i3c_get_adapdata(const struct i3c_adapter *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void i3c_set_adapdata(struct i3c_adapter *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}

///
struct i3c_dev {
	struct list_head list;
	struct i3c_adapter *adap;
	struct device *dev;
	struct cdev cdev;
};

struct i3c_client {
	unsigned short flags;		/* div., see below		*/
	unsigned short addr;		/* chip address - NOTE: 7bit	*/
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	char name[I2C_NAME_SIZE];
	struct i3c_adapter *adapter;	/* the adapter we sit on	*/
	struct device dev;		/* the device structure		*/
	int init_irq;			/* irq set at initialization	*/
	int irq;			/* irq issued by device		*/
	struct list_head detected;
};

#define I3C_MINORS	(MINORMASK + 1)
static LIST_HEAD(i3c_dev_list);
static DEFINE_SPINLOCK(i3c_dev_list_lock);

static struct i3c_dev *i3c_dev_get_by_minor(unsigned index)
{
	struct i3c_dev *i3c_dev;

	spin_lock(&i3c_dev_list_lock);
	list_for_each_entry(i3c_dev, &i3c_dev_list, list) {
		if (i3c_dev->adap->nr == index)
			goto found;
	}
	i3c_dev = NULL;
found:
	spin_unlock(&i3c_dev_list_lock);
	return i3c_dev;
}

static DEFINE_IDR(i3c_adapter_idr);

static DEFINE_MUTEX(core_lock);

int i3c_for_each_dev(void *data, int (*fn)(struct device *, void *))
{
	int res;

	mutex_lock(&core_lock);
	res = bus_for_each_dev(&i2c_bus_type, NULL, data, fn);
	mutex_unlock(&core_lock);

	return res;
}

struct i3c_adapter *i3c_get_adapter(int nr)
{
	struct i3c_adapter *adapter;

	mutex_lock(&core_lock);
	adapter = idr_find(&i3c_adapter_idr, nr);
	if (!adapter)
		goto exit;

	if (try_module_get(adapter->owner))
		get_device(&adapter->dev);
	else
		adapter = NULL;

 exit:
	mutex_unlock(&core_lock);
	return adapter;
}

static struct i3c_dev *get_free_i3c_dev(struct i3c_adapter *adap)
{
	struct i3c_dev *i3c_dev;

	if (adap->nr >= I3C_MINORS) {
		printk(KERN_ERR "i3c-dev: Out of device minors (%d)\n",
		       adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i3c_dev = kzalloc(sizeof(*i3c_dev), GFP_KERNEL);
	if (!i3c_dev)
		return ERR_PTR(-ENOMEM);
	i3c_dev->adap = adap;

	spin_lock(&i3c_dev_list_lock);
	list_add_tail(&i3c_dev->list, &i3c_dev_list);
	spin_unlock(&i3c_dev_list_lock);
	return i3c_dev;
}

static void put_i3c_dev(struct i3c_dev *i3c_dev)
{
	spin_lock(&i3c_dev_list_lock);
	list_del(&i3c_dev->list);
	spin_unlock(&i3c_dev_list_lock);
	kfree(i3c_dev);
}

static ssize_t name_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct i3c_dev *i3c_dev = i3c_dev_get_by_minor(MINOR(dev->devt));

	if (!i3c_dev)
		return -ENODEV;
	return sprintf(buf, "%s\n", i3c_dev->adap->name);
}
static DEVICE_ATTR_RO(name);

static struct attribute *i3c_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};
ATTRIBUTE_GROUPS(i3c);

/* ------------------------------------------------------------------------- */
static ssize_t i3cdev_read(struct file *file, char __user *buf, size_t count,
		loff_t *offset)
{
	char *tmp;
	int ret;
	printk("i3cdev_read \n");
	struct i3c_client *client = file->private_data;

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	pr_debug("i3c-dev: i3c-%d reading %zu bytes.\n",
		iminor(file_inode(file)), count);

#if 0
///
	int ret;
	struct i3c_msg msg = {
		.addr = client->addr,
		.flags = flags | (client->flags & I2C_M_TEN),
		.len = count,
		.buf = buf,
	};

	ret = adap->algo->master_xfer(client->adapter, &msg, 1);
	if (ret != -EAGAIN)
		break;

	ret = i3c_transfer(client->adapter, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg transferred), return #bytes
	 * transferred, else error code.
	 */
	return (ret == 1) ? count : ret;

///
	ret = i3c_master_recv(client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;
#endif
	kfree(tmp);
	return ret;
}

static ssize_t i3cdev_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	int ret;
	char *tmp;
	printk("i3cdev_write \n");
#if 0	
	struct i3c_client *client = file->private_data;

	if (count > 8192)
		count = 8192;

	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	pr_debug("i3c-dev: i3c-%d writing %zu bytes.\n",
		iminor(file_inode(file)), count);

	ret = i3c_master_send(client, tmp, count);
	kfree(tmp);
#endif	
	return ret;
}

static noinline int i3cdev_ioctl_rdwr(struct i3c_client *client,
		unsigned nmsgs, struct i3c_msg *msgs)
{
	u8 __user **data_ptrs;
	int i, res;
	
	printk("i3cdev_ioctl_rdwr \n");
	data_ptrs = kmalloc_array(nmsgs, sizeof(u8 __user *), GFP_KERNEL);
	if (data_ptrs == NULL) {
		kfree(msgs);
		return -ENOMEM;
	}

	res = 0;
	for (i = 0; i < nmsgs; i++) {
		/* Limit the size of the message to a sane amount */
		if (msgs[i].len > 8192) {
			res = -EINVAL;
			break;
		}

		data_ptrs[i] = (u8 __user *)msgs[i].buf;
		msgs[i].buf = memdup_user(data_ptrs[i], msgs[i].len);
		if (IS_ERR(msgs[i].buf)) {
			res = PTR_ERR(msgs[i].buf);
			break;
		}
		/* memdup_user allocates with GFP_KERNEL, so DMA is ok */
		msgs[i].flags |= I3C_M_DMA_SAFE;

		/*
		 * If the message length is received from the slave (similar
		 * to SMBus block read), we must ensure that the buffer will
		 * be large enough to cope with a message length of
		 * I3C_SMBUS_BLOCK_MAX as this is the maximum underlying bus
		 * drivers allow. The first byte in the buffer must be
		 * pre-filled with the number of extra bytes, which must be
		 * at least one to hold the message length, but can be
		 * greater (for example to account for a checksum byte at
		 * the end of the message.)
		 */
	}
	if (res < 0) {
		int j;
		for (j = 0; j < i; ++j)
			kfree(msgs[j].buf);
		kfree(data_ptrs);
		kfree(msgs);
		return res;
	}

	//todo
//	res = i3c_transfer(client->adapter, msgs, nmsgs);
	while (i-- > 0) {
		if (res >= 0 && (msgs[i].flags & I3C_M_RD)) {
			if (copy_to_user(data_ptrs[i], msgs[i].buf,
					 msgs[i].len))
				res = -EFAULT;
		}
		kfree(msgs[i].buf);
	}
	kfree(data_ptrs);
	kfree(msgs);

	return res;
}


#define I3C_RDWR	0x0707	/* Combined R/W transfer (one STOP only) */

/* This is the structure as used in the I3C_RDWR ioctl call */
struct i3c_rdwr_ioctl_data {
	struct i3c_msg __user *msgs;	/* pointers to i3c_msgs */
	__u32 nmsgs;			/* number of i3c_msgs */
};

#define  I3C_RDWR_IOCTL_MAX_MSGS	42

static long i3cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i3c_client *client = file->private_data;
	unsigned long funcs;
	printk("i3cdev_ioctl \n");

	switch (cmd) {
	case I3C_RDWR: {
		struct i3c_rdwr_ioctl_data rdwr_arg;
		struct i3c_msg *rdwr_pa;

		if (copy_from_user(&rdwr_arg,
				   (struct i3c_rdwr_ioctl_data __user *)arg,
				   sizeof(rdwr_arg)))
			return -EFAULT;

		/* Put an arbitrary limit on the number of messages that can
		 * be sent at once */
		if (rdwr_arg.nmsgs > I3C_RDWR_IOCTL_MAX_MSGS)
			return -EINVAL;

		rdwr_pa = memdup_user(rdwr_arg.msgs,
				      rdwr_arg.nmsgs * sizeof(struct i3c_msg));
		if (IS_ERR(rdwr_pa))
			return PTR_ERR(rdwr_pa);

		return i3cdev_ioctl_rdwr(client, rdwr_arg.nmsgs, rdwr_pa);
	}

	default:
		/* NOTE:  returning a fault code here could cause trouble
		 * in buggy userspace code.  Some old kernel bugs returned
		 * zero in this case, and userspace code might accidentally
		 * have depended on that bug.
		 */
		return -ENOTTY;
	}

	return 0;
}

static int i3cdev_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	struct i3c_client *client;
	struct i3c_adapter *adap;
	printk("i3cdev_open minor %d \n", minor);

	adap = i3c_get_adapter(minor);
	if (!adap)
		return -ENODEV;

	printk("i3cdev_open 0\n");

	/* This creates an anonymous i3c_client, which may later be
	 * pointed to some address using I3C_SLAVE or I3C_SLAVE_FORCE.
	 *
	 * This client is ** NEVER REGISTERED ** with the driver model
	 * or I2C core code!!  It just holds private copies of addressing
	 * information and maybe a PEC flag.
	 */
	printk("i3cdev_open 1\n");
	 
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		i3c_put_adapter(adap);
		return -ENOMEM;
	}
	snprintf(client->name, I3C_NAME_SIZE, "i3c-dev %d", adap->nr);
	printk("i3cdev_open 2\n");

	client->adapter = adap;
	file->private_data = client;

	printk("i3cdev_open end\n");

	return 0;
}

static int i3cdev_release(struct inode *inode, struct file *file)
{
	struct i3c_client *client = file->private_data;

	kfree(client);
	file->private_data = NULL;

	return 0;
}

static const struct file_operations i3cdev_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= i3cdev_read,
	.write		= i3cdev_write,
	.unlocked_ioctl	= i3cdev_ioctl,
	.open		= i3cdev_open,
	.release	= i3cdev_release,
};

/* ------------------------------------------------------------------------- */

static struct class *i3c_dev_class;

static int i3cdev_attach_adapter(struct device *dev, void *dummy)
{
	struct i3c_adapter *adap;
	struct i3c_dev *i3c_dev;
	int res;

//	if (dev->type != &i3c_adapter_type)
//		return 0;
	adap = to_i3c_adapter(dev);

	i3c_dev = get_free_i3c_dev(adap);
	if (IS_ERR(i3c_dev))
		return PTR_ERR(i3c_dev);

	cdev_init(&i3c_dev->cdev, &i3cdev_fops);
	i3c_dev->cdev.owner = THIS_MODULE;
	res = cdev_add(&i3c_dev->cdev, MKDEV(I3C_MAJOR, adap->nr), 1);
	if (res)
		goto error_cdev;

	/* register this i3c device with the driver core */
	i3c_dev->dev = device_create(i3c_dev_class, &adap->dev,
				     MKDEV(I3C_MAJOR, adap->nr), NULL,
				     "i3c-%d", adap->nr);
	if (IS_ERR(i3c_dev->dev)) {
		res = PTR_ERR(i3c_dev->dev);
		goto error;
	}

	pr_debug("i3c-dev: adapter [%s] registered as minor %d\n",
		 adap->name, adap->nr);
	return 0;
error:
	cdev_del(&i3c_dev->cdev);
error_cdev:
	put_i3c_dev(i3c_dev);
	return res;
}

static int i3cdev_detach_adapter(struct device *dev, void *dummy)
{
	struct i3c_adapter *adap;
	struct i3c_dev *i3c_dev;

//	if (dev->type != &i3c_adapter_type)
//		return 0;
	adap = to_i3c_adapter(dev);

	i3c_dev = i3c_dev_get_by_minor(adap->nr);
	if (!i3c_dev) /* attach_adapter must have failed */
		return 0;

	cdev_del(&i3c_dev->cdev);
	put_i3c_dev(i3c_dev);
	device_destroy(i3c_dev_class, MKDEV(I3C_MAJOR, adap->nr));

	pr_debug("i3c-dev: adapter [%s] unregistered\n", adap->name);
	return 0;
}

static int i3cdev_notifier_call(struct notifier_block *nb, unsigned long action,
			 void *data)
{
	struct device *dev = data;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		return i3cdev_attach_adapter(dev, NULL);
	case BUS_NOTIFY_DEL_DEVICE:
		return i3cdev_detach_adapter(dev, NULL);
	}

	return 0;
}

static struct notifier_block i3cdev_notifier = {
	.notifier_call = i3cdev_notifier_call,
};

/*
 * module load/unload record keeping
 */
static int __init i3c_dev_init(void)
{
	int res;
printk("i3c_dev_init =============xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
	printk(KERN_INFO "i3c /dev entries driver\n");

	res = register_chrdev_region(MKDEV(I3C_MAJOR, 0), I3C_MINORS, "i3c");
	if (res)
		goto out;

	i3c_dev_class = class_create(THIS_MODULE, "i3c-dev");
	if (IS_ERR(i3c_dev_class)) {
		res = PTR_ERR(i3c_dev_class);
		goto out_unreg_chrdev;
	}
	i3c_dev_class->dev_groups = i3c_groups;

	/* Keep track of adapters which will be added or removed later */
	res = bus_register_notifier(&i3c_bus_type, &i3cdev_notifier);
	if (res)
		goto out_unreg_class;

	/* Bind to already existing adapters right away */
	i3c_for_each_dev(NULL, i3cdev_attach_adapter);

	return 0;

out_unreg_class:
	class_destroy(i3c_dev_class);
out_unreg_chrdev:
	unregister_chrdev_region(MKDEV(I3C_MAJOR, 0), I3C_MINORS);
out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
	return res;
}

static void __exit i3c_dev_exit(void)
{
	bus_unregister_notifier(&i3c_bus_type, &i3cdev_notifier);
	i3c_for_each_dev(NULL, i3cdev_detach_adapter);
	class_destroy(i3c_dev_class);
	unregister_chrdev_region(MKDEV(I3C_MAJOR, 0), I3C_MINORS);
}

MODULE_AUTHOR("Frodo Looijaard <frodol@dds.nl> and "
		"Simon G. Vogl <simon@tk.uni-linz.ac.at>");
MODULE_DESCRIPTION("I2C /dev entries driver");
MODULE_LICENSE("GPL");

module_init(i3c_dev_init);
module_exit(i3c_dev_exit);
