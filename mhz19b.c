// SPDX-License-Identifier: GPL-2.0
/*
 * mh-z19b co2 sensor driver
 *
 * Copyright (c) 2025 Gyeyoung Baek <gye976@gmail.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serdev.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/mutex.h>
#include <linux/cleanup.h>

struct mhz19b_data {
	struct serdev_device *serdev;
	struct mutex lock;

	/* serdev receive buffer */
	char buf[9];
	int buf_idx;
};

/* ABC logig on/off */
#define MHZ19B_ABC_LOGIC_CMD		0x79

/* Read CO2 concentration */
#define MHZ19B_READ_CO2_CMD		0x86

/* Calibrate Zero Point */
#define MHZ19B_ZERO_POINT_CMD		0x87

/* Calibrate Span Point */
#define MHZ19B_SPAN_POINT_CMD		0x88

/* Set sensor detection range */
#define MHZ19B_DETECTION_RANGE_CMD	0x99

#define MHZ19B_CMD_SIZE 9

static int mhz19b_serdev_cmd(struct iio_dev *indio_dev, int cmd, const char *str);

static int mhz19b_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, 
	int *val, int *val2, long mask)
{
	struct mhz19b_data *mhz19b;
	int ret;

	mhz19b = iio_priv(indio_dev);

	ret = mhz19b_serdev_cmd(indio_dev, MHZ19B_READ_CO2_CMD, NULL);
	if (ret < 0)
		return ret;

	*val = ret;
	return IIO_VAL_INT;
}

static const struct iio_info mhz19b_info = {
	.read_raw = mhz19b_read_raw, 
};

static ssize_t mhz19b_zero_point_cal_write(struct iio_dev *iiodev,
	uintptr_t private, const struct iio_chan_spec *chan,
	const char *buf, size_t len)
{
	int ret;

	ret = mhz19b_serdev_cmd(iiodev, MHZ19B_ZERO_POINT_CMD, buf);
	if (ret < 0) 
		return ret;
	
	return len;
}

static ssize_t mhz19b_span_point_cal_write(struct iio_dev *iiodev,
	uintptr_t private, const struct iio_chan_spec *chan,
	const char *buf, size_t len)
{
	int ret;

	ret = mhz19b_serdev_cmd(iiodev, MHZ19B_SPAN_POINT_CMD, buf);
	if (ret < 0) 
		return ret;
	
	return len;
}

static ssize_t mhz19b_abc_logic_write(struct iio_dev *iiodev,
	uintptr_t private, const struct iio_chan_spec *chan,
	const char *buf, size_t len)
{
	int ret;

	ret = mhz19b_serdev_cmd(iiodev, MHZ19B_ABC_LOGIC_CMD, buf);
	if (ret < 0) 
		return ret;
	
	return len;
}

struct iio_chan_spec_ext_info mhz19b_co2_ext_info[] = {
	{
		.name = "zero_point",
		.write = mhz19b_zero_point_cal_write,
	},
	{
		.name = "span_point",
		.write = mhz19b_span_point_cal_write,
	},
	{
		.name = "abc_logic",
		.write = mhz19b_abc_logic_write,
	},
	{}	
};

static const struct iio_chan_spec mhz19b_channels[] = {
	{ 
		.type = IIO_CONCENTRATION,
		.channel2 = IIO_MOD_CO2,
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),

		.ext_info = mhz19b_co2_ext_info,
	}, 
};

static uint8_t mhz19b_get_checksum(uint8_t *packet)
{
	uint8_t i, checksum = 0; 
	
	for (i = 1; i < 8; i++)
		checksum += packet[i];
	
	checksum = 0xff - checksum;
	checksum += 1;
	
	return checksum;
}

static int mhz19b_serdev_cmd(struct iio_dev *indio_dev, int cmd, const char *str)
{
	int ret = 0;
	struct serdev_device *serdev;
	struct mhz19b_data *mhz19b;
	struct device *dev;

	mhz19b = iio_priv(indio_dev);
	serdev = mhz19b->serdev;
	dev = &indio_dev->dev;

	/*
	 * commands have following format:
	 *
	 * +------+------+-----+------+------+------+------+------+-------+
	 * | 0xFF | 0x01 | cmd | arg0 | arg1 | 0x00 | 0x00 | 0x00 | cksum |
	 * +------+------+-----+------+------+------+------+------+-------+
	 */
	uint8_t cmd_buf[MHZ19B_CMD_SIZE] = {
		0xFF, 0x01, cmd,
	};
	
	switch (cmd) {
	case MHZ19B_ABC_LOGIC_CMD:
	{
		bool enable;

		ret = kstrtobool(str, &enable);
		if (ret)
			return ret;

		cmd_buf[3] = enable ? 0xA0 : 0x00;
		break;
	}
	case MHZ19B_SPAN_POINT_CMD:
	{
		uint16_t ppm;

		ret = kstrtou16(str, 10, &ppm);
		if (ret)
			return ret;

		/* at least 1000ppm */
		if (ppm < 1000 || ppm > 5000) {
			dev_dbg(&indio_dev->dev, "span point ppm should be 1000~5000");
			return -EINVAL;
		}

		cmd_buf[3] = ppm / 256;
		cmd_buf[4] = ppm % 256;
		break;
	}
	case MHZ19B_DETECTION_RANGE_CMD:
	{
		uint16_t range;

		ret = kstrtou16(str, 10, &range);
		if (ret)
			return ret;

		/* Detection Range should be 2000 or 5000 */
		if (!(range == 2000 || range == 5000)) {
			dev_dbg(&indio_dev->dev, "detection range should be 2000 or 5000");
			return -EINVAL;
		}

		cmd_buf[3] = range / 256;
		cmd_buf[4] = range % 256;
		break;
	}
	default:
		break;
	}	
	cmd_buf[MHZ19B_CMD_SIZE - 1] = mhz19b_get_checksum(cmd_buf);
	
	scoped_guard(mutex, &mhz19b->lock) {
		ret = serdev_device_write(serdev, cmd_buf, MHZ19B_CMD_SIZE, 0);  
		mhz19b->buf_idx = 0;

		if (ret != MHZ19B_CMD_SIZE) {
			dev_err(dev, "write err, %d bytes written", ret);
			return -EINVAL;
		}

		switch (cmd) {
		case MHZ19B_READ_CO2_CMD:
			if (mhz19b->buf[MHZ19B_CMD_SIZE - 1] != mhz19b_get_checksum(mhz19b->buf)) {
				dev_err(dev, "checksum err");
				return -EINVAL;
			}

			ret = (mhz19b->buf[2] << 8) + mhz19b->buf[3];
			break;
		default:
			/* no response commands. */
			ret = 0;
			break;
		}
	}

	return ret;
}

static int mhz19b_core_probe(struct device *dev) 
{
	int ret;

	struct serdev_device *serdev;
	struct mhz19b_data *mhz19b;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(struct mhz19b_data));
	if (indio_dev == NULL) 
		return ret;

	dev_set_drvdata(dev, indio_dev);

	mhz19b = iio_priv(indio_dev);

	mhz19b->buf_idx = 0;
	ret = devm_mutex_init(dev, &mhz19b->lock);
	if (ret) 
		return ret;
		
	serdev = container_of(dev, struct serdev_device, dev);	

	mhz19b->serdev = serdev;

	indio_dev->name = "mh-z19b";
	indio_dev->channels = mhz19b_channels;
	indio_dev->num_channels = ARRAY_SIZE(mhz19b_channels);
	indio_dev->info = &mhz19b_info;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
 	       return ret;

	return 0;
}

static int mhz19b_receive_buf(struct serdev_device *serdev, const unsigned char *data, size_t len)
{
	struct iio_dev *indio_dev;
	struct mhz19b_data *mhz19b;

	indio_dev = dev_get_drvdata(&serdev->dev);
	mhz19b = iio_priv(indio_dev);

	for (int i = 0; i < len; i++) {
		mhz19b->buf[mhz19b->buf_idx++] = data[i];
	}

	return len;
}
static void mhz19b_write_wakeup(struct serdev_device *serdev)
{
	struct iio_dev *indio_dev;
	indio_dev = dev_get_drvdata(&serdev->dev);

	dev_dbg(&indio_dev->dev, "mhz19b_write_wakeup");
}

static const struct serdev_device_ops mhz19b_ops = {
	.receive_buf = mhz19b_receive_buf,
	.write_wakeup = mhz19b_write_wakeup,
};

static int mhz19b_probe(struct serdev_device *serdev)
{
    int ret;

    struct device *dev;

    dev = &serdev->dev;
    serdev_device_set_client_ops(serdev, &mhz19b_ops);

    ret = devm_serdev_device_open(dev, serdev);
    if (ret)
	return ret;

    ret = serdev_device_set_baudrate(serdev, 9600);
    
	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret < 0) 
		return ret;

    ret = mhz19b_core_probe(dev);
    if (ret)
        return ret;

    return 0;
}

static const struct of_device_id mhz19b_of_match[] = {
    { .compatible = "winsen,mhz19b", },
    {   }
};

MODULE_DEVICE_TABLE(of, mhz19b_of_match);

static struct serdev_device_driver mhz19b_driver = {
    .driver = {
        .name = "mhz19b",
        .of_match_table = mhz19b_of_match,
    },
    .probe = mhz19b_probe,
};

module_serdev_device_driver(mhz19b_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gyeyoung Baek");
MODULE_DESCRIPTION("MH-Z19B CO2 sensor driver using serdev interface");
