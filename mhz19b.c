// SPDX-License-Identifier: GPL-2.0
/*
 * mh-z19b co2 sensor driver
 *
 * Copyright (c) 2025 Gyeyoung Baek <gye976@gmail.com>
 *
 * Datasheet:
 * https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf
 *
 * TODO:
 *  - vin supply regulator
 */

#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
//#include <linux/unaligned.h>
#include <asm-generic/unaligned.h> // TODO

struct mhz19b_state {
	struct serdev_device *serdev;

	/* TO DO, nothing for now.*/
	struct regulator *vin_supply;

	/* serdev receive buffer.
	 * When data is received from the MH-Z19B,
	 * the 'mhz19b_receive_buf' callback function is called and fills this buffer.
	 */
	char buf[9];
	int buf_idx;

	/* must wait the 'buf' is filled with 9 bytes.*/
	struct completion buf_ready;

	/* protect access to mhz19b_state */
	struct mutex lock;
};

/*
 * commands have following format:
 *
 * +------+------+-----+------+------+------+------+------+-------+
 * | 0xFF | 0x01 | cmd | arg0 | arg1 | 0x00 | 0x00 | 0x00 | cksum |
 * +------+------+-----+------+------+------+------+------+-------+
 *
 * The following commands are defined in the datasheet.
 * https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf
 */
#define MHZ19B_CMD_SIZE 9

#define MHZ19B_ABC_LOGIC_CMD		0x79
#define MHZ19B_READ_CO2_CMD		0x86
#define MHZ19B_ZERO_POINT_CMD		0x87
#define MHZ19B_SPAN_POINT_CMD		0x88
#define MHZ19B_DETECTION_RANGE_CMD	0x99

/* ABC logic in MHZ19B means auto calibration.
 */

#define MHZ19B_SERDEV_TIMEOUT	msecs_to_jiffies(100)

static uint8_t mhz19b_get_checksum(uint8_t *packet)
{
	uint8_t i, checksum = 0;

	for (i = 1; i < 8; i++)
		checksum += packet[i];

	checksum = 0xff - checksum;
	checksum += 1;

	return checksum;
}

static int mhz19b_serdev_cmd(struct iio_dev *indio_dev,
	int cmd, void *arg)
{
	int ret = 0;
	struct mhz19b_state *st = iio_priv(indio_dev);
	struct serdev_device *serdev = st->serdev;
	struct device *dev = &indio_dev->dev;

	/* commands format is described above. */
	uint8_t cmd_buf[MHZ19B_CMD_SIZE] = {
		0xFF, 0x01, cmd,
	};

	switch (cmd) {
	case MHZ19B_ABC_LOGIC_CMD: {
		bool enable = *((bool *)arg);

		cmd_buf[3] = enable ? 0xA0 : 0x00;
		break;
	} case MHZ19B_SPAN_POINT_CMD: {
		uint16_t ppm = *((uint16_t *)arg);

		put_unaligned_be16(ppm, &cmd_buf[3]);
		break;
	} case MHZ19B_DETECTION_RANGE_CMD: {
		uint16_t range = *((uint16_t *)arg);

		put_unaligned_be16(range, &cmd_buf[3]);
		break;
	} default:
		break;
	}
	cmd_buf[MHZ19B_CMD_SIZE - 1] = mhz19b_get_checksum(cmd_buf);

	scoped_guard(mutex, &st->lock) {
		/* write buf to uart ctrl syncronously */
		ret = serdev_device_write(serdev, cmd_buf, MHZ19B_CMD_SIZE, 0);
		if (ret != MHZ19B_CMD_SIZE) {
			dev_err(dev, "write err, %d bytes written", ret);
			return -EINVAL;
		}

		switch (cmd) {
		case MHZ19B_READ_CO2_CMD:
			ret = wait_for_completion_interruptible_timeout(&st->buf_ready,
				MHZ19B_SERDEV_TIMEOUT);
			if (ret < 0)
				return ret;
			if (!ret)
				return -ETIMEDOUT;

			ret = mhz19b_get_checksum(st->buf);
			if (st->buf[MHZ19B_CMD_SIZE - 1] != mhz19b_get_checksum(st->buf)) {
				dev_err(dev, "checksum err");
				return -EINVAL;
			}

			ret = get_unaligned_be16(&st->buf[2]);
			return ret;
		default:
			/* no response commands. */
			return 0;
		}
	}
}

static int mhz19b_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan,
	int *val, int *val2, long mask)
{
	int ret = mhz19b_serdev_cmd(indio_dev, MHZ19B_READ_CO2_CMD, NULL);

	if (ret < 0)
		return ret;

	*val = ret;
	return IIO_VAL_INT;
}

/* MHZ19B only supports writing configuration values. */

static ssize_t calibration_auto_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	bool enable;

	int ret = kstrtobool(buf, &enable);

	if (ret)
		return ret;

	ret = mhz19b_serdev_cmd(indio_dev, MHZ19B_ABC_LOGIC_CMD, &enable);
	if (ret < 0)
		return ret;

	return len;
}
static IIO_DEVICE_ATTR_WO(calibration_auto_enable, 0);

/* write 0		: zero point calibration_auto_enable
 *	(make sure the sensor had been worked under 400ppm for over 20 minutes.)
 *
 * write 1000-5000	: span point calibration:
 *	(make sure the sensor had been worked under a certain level co2 for over 20 minutes.)
 */
static ssize_t calibration_forced_value_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	uint16_t ppm;
	int cmd;

	int ret = kstrtou16(buf, 10, &ppm);

	if (ret)
		return ret;

	/* at least 1000ppm */
	if (ppm) {
		if (ppm < 1000 || ppm > 5000) {
			dev_dbg(&indio_dev->dev,
				"span point ppm should be 1000~5000");
			return -EINVAL;
		}

		cmd = MHZ19B_SPAN_POINT_CMD;
	} else {
		cmd = MHZ19B_ZERO_POINT_CMD;
	}

	ret = mhz19b_serdev_cmd(indio_dev, cmd, &ppm);
	if (ret < 0)
		return ret;

	return len;
}
static IIO_CONST_ATTR(calibration_forced_value_available,
	"0 1000-5000");
static IIO_DEVICE_ATTR_WO(calibration_forced_value, 0);

/* MH-Z19B supports a measurement range adjustment feature.
 * It can measure up to 2000 ppm or up to 5000 ppm.
 */
static ssize_t co2_range_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	int ret;
	uint16_t range;

	ret = kstrtou16(buf, 10, &range);
	if (ret)
		return ret;

	/* Detection Range should be 2000 or 5000 */
	if (!(range == 2000 || range == 5000)) {
		dev_dbg(&indio_dev->dev, "detection range should be 2000 or 5000");
		return -EINVAL;
	}


	ret = mhz19b_serdev_cmd(indio_dev, MHZ19B_DETECTION_RANGE_CMD, &range);
	if (ret < 0)
		return ret;

	return len;
}
static IIO_CONST_ATTR(co2_range_available,
	"2000 5000");
static IIO_DEVICE_ATTR_WO(co2_range, 0);

static struct attribute *mhz19b_attrs[] = {
	&iio_const_attr_calibration_forced_value_available.dev_attr.attr,
	&iio_const_attr_co2_range_available.dev_attr.attr,
	&iio_dev_attr_calibration_auto_enable.dev_attr.attr,
	&iio_dev_attr_calibration_forced_value.dev_attr.attr,
	&iio_dev_attr_co2_range.dev_attr.attr,
	NULL
};

static const struct attribute_group mhz19b_attr_group = {
	.attrs = mhz19b_attrs,
};

static const struct iio_info mhz19b_info = {
	.attrs = &mhz19b_attr_group,
	.read_raw = mhz19b_read_raw,
};

static const struct iio_chan_spec mhz19b_channels[] = {
	{
		.type = IIO_CONCENTRATION,
		.channel2 = IIO_MOD_CO2,
		.modified = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static int mhz19b_receive_buf(struct serdev_device *serdev, const unsigned char *data, size_t len) // TODO
{
	struct iio_dev *indio_dev = dev_get_drvdata(&serdev->dev);
	struct mhz19b_state *st = iio_priv(indio_dev);

	for (int i = 0; i < len; i++)
		st->buf[st->buf_idx++] = data[i];

	if (st->buf_idx == MHZ19B_CMD_SIZE) {
		st->buf_idx = 0;
		complete(&st->buf_ready);
	}

	return len;
}

/* The 'serdev_device_write' function returns -EINVAL if the 'write_wakeup' member is NULL,
 * so it must be mandatory.
 */
static void mhz19b_write_wakeup(struct serdev_device *serdev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&serdev->dev);

	dev_dbg(&indio_dev->dev, "mhz19b_write_wakeup");
}

static const struct serdev_device_ops mhz19b_ops = {
	.receive_buf = mhz19b_receive_buf,
	.write_wakeup = mhz19b_write_wakeup,
};

static int mhz19b_probe(struct serdev_device *serdev)
{
	int ret;
	struct device *dev = &serdev->dev;
	struct iio_dev *indio_dev;
	struct mhz19b_state *st;

	serdev_device_set_client_ops(serdev, &mhz19b_ops);

	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return ret;

	ret = serdev_device_set_baudrate(serdev, 9600);
	if (ret < 0)
		return ret;

	/* void type func, no return */
	serdev_device_set_flow_control(serdev, false);

	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret < 0)
		return ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(struct mhz19b_state));
	if (indio_dev == NULL)
		return ret;
	dev_set_drvdata(dev, indio_dev);

	st = iio_priv(indio_dev);
	st->serdev = serdev;

	init_completion(&st->buf_ready);
	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	/* TO DO:
	 *  - vin supply
	 */

	indio_dev->name = "mh-z19b";
	indio_dev->channels = mhz19b_channels;
	indio_dev->num_channels = ARRAY_SIZE(mhz19b_channels);
	indio_dev->info = &mhz19b_info;
	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id mhz19b_of_match[] = {
	{ .compatible = "winsen,mhz19b", },
	{ }
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

MODULE_AUTHOR("Gyeyoung Baek");
MODULE_DESCRIPTION("MH-Z19B CO2 sensor driver using serdev interface");
MODULE_LICENSE("GPL v2");
