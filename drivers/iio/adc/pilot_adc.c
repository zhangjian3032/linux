/*
 * Aspeed Pilot ADC
 *
 * Copyright (C) 2019 Aspeed Technology Inc.
 * Shivah Shankar <shivahs@aspeedtech.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iopoll.h>

#define PILOT_RESOLUTION_BITS		10

#define VESTS 0x0
#define VESTS_A7_CHAN_RDY	(1<<0)
#define VCNT 0x8
#define VCNT_CHAN_FLTRS_BYPASS0	(1<<16)
#define VCNT_CHAN_FLTRS_BYPASS1 (1<<17)
#define VCNT_CHAN_FLTRS_BYPASS2 (1<<18)
#define VCNT_CHAN_FLTRS_BYPASS3 (1<<19)
#define VCNT_CHAN_FLTRS_BYPASS4 (1<<20)
#define VCNT_CHAN_FLTRS_BYPASS5 (1<<21)
#define VCNT_CHAN_FLTRS_BYPASS6 (1<<22)
#define VCNT_CHAN_FLTRS_BYPASS7 (1<<23)
#define VCNT_CHAN_FLTRS_BYPASS8 (1<<24)
#define VCNT_SEL_GEN_COUNT	(5<<4)
#define VCNT_START_CONVERT	(1<<1)
#define VCNT_VLM_ENABLE		(1<<0)	
#define VCHNE 0xC
#define VCHNE_CHAN_7_SAMPLE	(1<<15)
#define VCHNE_CHAN_TEMP_ENABLE	(1<<8)
#define VCHNE_CHAN_7_ENABLE	(1<<7)
#define VCHNE_CHAN_6_ENABLE	(1<<6)
#define VCHNE_CHAN_5_ENABLE	(1<<5)
#define VCHNE_CHAN_4_ENABLE	(1<<4)
#define VCHNE_CHAN_3_ENABLE	(1<<3)
#define VCHNE_CHAN_2_ENABLE	(1<<2)
#define VCHNE_CHAN_1_ENABLE	(1<<1)
#define VCHNE_CHAN_0_ENABLE	(1<<0)

#define VCHRS00 0x60
#define VCHRS01 0x64
#define VCHRS02 0x68
#define VCHRS03 0x6C
#define VCHRS04 0x70

struct pilot_adc_model_data {
	const char *model_name;
	unsigned int vref_voltage;	// mV
	bool wait_init_sequence;
};
struct pilot_adc_data {
	struct device		*dev;
	void __iomem		*base;
};
#define PILOT_CHAN(_idx, _data_reg_addr) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.address = (_data_reg_addr),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW | 		\
			IIO_CHAN_INFO_PROCESSED),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), 	\
}
static const struct iio_chan_spec pilot_adc_iio_channels[] = {
	PILOT_CHAN(0, 0x60),
	PILOT_CHAN(1, 0x62),
	PILOT_CHAN(2, 0x64),
	PILOT_CHAN(3, 0x66),
	PILOT_CHAN(4, 0x68),
	PILOT_CHAN(5, 0x6A),
	PILOT_CHAN(6, 0x6C),
	PILOT_CHAN(7, 0x6E),
};
#define PILOT_RESOLUTION_BITS	10
static int pilot_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct pilot_adc_data *data = iio_priv(indio_dev);
	const struct pilot_adc_model_data *model_data =
			of_device_get_match_data(data->dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/* On EVB Some of them may be connected to the VDD power domain
		 * Hence enabling them before may NOT read the correct values
		 * Write to the raw register everytime your try to read
		 * the voltage value
		 */
		*val = readw(data->base + chan->address);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		if( chan->channel != 7){
			*val = (readw(data->base + chan->address) *
				model_data->vref_voltage)/1024;
		}else{
			*val = (readw(data->base + chan->address) *
				model_data->vref_voltage *4)/(1024 *3);
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if( chan->channel == 7){
		/* As per spec for channel 7
		 * AVIN_BAT_7 = (Channel Raw Register * VREF) / 1024)*(4/3)
		 * 1024 part is already taken care as we are giving ref
		 * voltage in milli volts so just normal mult willdo
		 */
			*val = model_data->vref_voltage *4;
			*val2 = 3;
			return IIO_VAL_FRACTIONAL;
		}else{
		/* As per spec for channel 0 -6
		 * AVIN[6:0] = (Channel Raw Register * VREF) / 1024 
		 * 1024 part is already taken care as we are giving ref voltage
		 * voltage in milli volts so just normal mult willdo
		 */
			*val = model_data->vref_voltage;
			return IIO_VAL_INT;
		}

	default:
		return -EINVAL;
	}
}

static int pilot_adc_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_RAW:
		/*
		 * The device is pre programmed
		 * no provision to program these parameters.
		 */
		return -EPERM;

	default:
		return -EINVAL;
	}
}

static int pilot_adc_reg_access(struct iio_dev *indio_dev,
				 unsigned int reg, unsigned int writeval,
				 unsigned int *readval)
{
	struct pilot_adc_data *data = iio_priv(indio_dev);

	if (!readval || reg % 4 || reg > 0x74)
		return -EINVAL;

	*readval = readl(data->base + reg);

	return 0;
}

static const struct iio_info pilot_adc_iio_info = {
	.read_raw = pilot_adc_read_raw,
	.write_raw = pilot_adc_write_raw,
	.debugfs_reg_access = pilot_adc_reg_access,
};
static int pilot_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct pilot_adc_data *data;
	const struct pilot_adc_model_data *model_data;
	struct resource *res;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);
	model_data = of_device_get_match_data(&pdev->dev);
	/* Enable all channels*/
	writel((VCNT_SEL_GEN_COUNT|VCNT_START_CONVERT|VCNT_VLM_ENABLE), data->base + VCNT);
	writel(0x81FF, data->base + VCHNE);
	indio_dev->name = model_data->model_name;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &pilot_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = pilot_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(pilot_adc_iio_channels);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto iio_register_error;

	return 0;

iio_register_error:
	/* Disable all channels*/
	writel(0x0, data->base + VCHNE);
	/* Disable VLM */
	writel(~(VCNT_SEL_GEN_COUNT|VCNT_START_CONVERT|VCNT_VLM_ENABLE), data->base + VCNT);
	return ret;
}
static int pilot_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct pilot_adc_data *data;
	
	data = iio_priv(indio_dev);
	iio_device_unregister(indio_dev);
	/* Disable all channels*/
	writel(0x0, data->base + VCHNE);
	/* Disable VLM */
	writel(~(VCNT_SEL_GEN_COUNT|VCNT_START_CONVERT|VCNT_VLM_ENABLE), data->base + VCNT);
	return 0;
}
static const struct pilot_adc_model_data pilot_model_data = {
	.model_name = "pilot-adc",
	.vref_voltage = 2500, // mV
};
static const struct of_device_id pilot_adc_matches[] = {
	{ .compatible = "aspeed,pilot-adc", .data = &pilot_model_data },
	{},
};
MODULE_DEVICE_TABLE(of, pilot_adc_matches);

static struct platform_driver pilot_adc_driver = {
	.probe = pilot_adc_probe,
	.remove = pilot_adc_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = pilot_adc_matches,
	}
};

module_platform_driver(pilot_adc_driver);

MODULE_AUTHOR("Shivah Shankar S <shivahs@aspeedtech.com>");
MODULE_DESCRIPTION("Aspeed Pilot ADC Driver");
MODULE_LICENSE("GPL");
