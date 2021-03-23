/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *  Driver for Si468x AM/FM/DAB Radio Receiver on x3399headunit board
 *
 *  Copyright (C) 2021 Oleksii Gulchenko <alexey.gulchenko@gmail.com>
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>

#include "si468x.h"

extern const struct video_device si468x_viddev;

/*
 * si468x_xfer - IO
 */
static int si468x_xfer(struct si468x_device *radio,
    void *write_ptr, size_t write_sz, void *read_ptr, size_t read_sz)
{
    uint8_t addr = radio->client->addr;
    int msgs_num = (read_sz > 0) ? 2 : 1;
    int ret;

	struct i2c_msg msgs[2] = {
		{ .addr = addr, .flags = 0,      .len = write_sz, .buf = write_ptr },
        { .addr = addr,	.flags = I2C_M_RD, .len = read_sz, .buf = read_ptr },
	};

    mutex_lock(&radio->i2c_lock);
    ret = i2c_transfer(radio->client->adapter, msgs, msgs_num);
    mutex_unlock(&radio->i2c_lock);

	if (ret != msgs_num) {
        dev_err(&radio->client->dev, "i2c_transfer failed, ret %d\n", ret);
        return ret;
    }
    return 0;
}

/*
 * si468x_i2c_interrupt - interrupt handler
 */
static irqreturn_t si468x_i2c_interrupt(int irq, void *dev_id)
{
	struct si468x_device *radio = dev_id;
    struct device *dev = &radio->client->dev;
    int ret;
    uint8_t status[4], flags = 0;

    /* Read status */
    status[0] = SI468X_RD_REPLY;
    ret = radio->si468x_xfer(radio, &status, 1, status, 4);
    if (ret) {
        dev_err(dev, "%s: SI468X_RD_REPLY failed: %d\n", __func__, ret);
        return IRQ_HANDLED;
    }

    dev_dbg(dev, "%s: %02x %02x %02x %02x",
        __func__, status[0], status[1], status[2], status[3]);

    if(status[0] & (1 << 0)) /* STCINT */
        flags |= TUNE_STATUS_FLAG_STC;

    if(status[0] & (1 << 1)) /* ACFINT */
        flags |= TUNE_STATUS_FLAG_ACF;

    if(status[0] & (1 << 2)) /* RDSINT */
        flags |= TUNE_STATUS_FLAG_RDS;

    if(status[0] & (1 << 3)) /* RSQINT */
        flags |= TUNE_STATUS_FLAG_RSQ;

    mutex_lock(&radio->irq_flags_lock);
    radio->irq_flags = flags;
    mutex_unlock(&radio->irq_flags_lock);

    if(flags & TUNE_STATUS_FLAG_STC) {
        wake_up_interruptible(&radio->status_queue);
    }

	return IRQ_HANDLED;
}

static int si468x_fops_release(struct file *file)
{
    struct si468x_device *radio = video_drvdata(file);

    if (radio->gpio_reset) {
        gpiod_set_value_cansleep(radio->gpio_reset, 1);
    }

    radio->mode = SI468X_MODE_UNKNOWN;
    return 0;
}

/*
 * si468x_i2c_probe - probe for the device
 */
static int si468x_i2c_probe(struct i2c_client *client)
{
	struct si468x_device *radio;
	int ret = 0;

	radio = devm_kzalloc(&client->dev, sizeof(*radio), GFP_KERNEL);
	if (!radio) {
        dev_err(&client->dev, "devm_kzalloc for context failed\n");
		return -ENOMEM;
	}

    radio->mode = SI468X_MODE_UNKNOWN;
	radio->client = client;

    mutex_init(&radio->i2c_lock);
	mutex_init(&radio->irq_flags_lock);
    init_waitqueue_head(&radio->status_queue);

	radio->fops_release = si468x_fops_release;
    radio->si468x_xfer = si468x_xfer;

	ret = v4l2_device_register(&client->dev, &radio->v4l2_dev);
	if (ret < 0) {
		dev_err(&client->dev, "couldn't register v4l2_device\n");
		goto err_device_register;
	}

	/* video device initialization */
	radio->videodev = si468x_viddev;
	radio->videodev.v4l2_dev = &radio->v4l2_dev;
	radio->videodev.release = video_device_release_empty;
	radio->videodev.device_caps = V4L2_CAP_READWRITE | V4L2_CAP_TUNER |
                                  V4L2_CAP_RADIO | V4L2_CAP_RDS_CAPTURE;
	video_set_drvdata(&radio->videodev, radio);

	radio->gpio_reset = devm_gpiod_get(&client->dev, "reset",
						    GPIOD_OUT_HIGH); /* Activate reset */
	if (IS_ERR(radio->gpio_reset)) {
		ret = PTR_ERR(radio->gpio_reset);
		dev_err(&client->dev, "Failed to request gpio: %d\n", ret);
		goto err_all;
	}

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					   si468x_i2c_interrupt,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   DRIVER_NAME, radio);
	if (ret) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_all;
	}

	/* register video device */
	ret = video_register_device(&radio->videodev, VFL_TYPE_RADIO, -1);
	if (ret) {
		dev_warn(&client->dev, "Could not register video device\n");
		goto err_all;
	}
	i2c_set_clientdata(client, radio);

    dev_err(&client->dev, "Radio device driver initialized\n");
	return 0;

err_all:
	v4l2_device_unregister(&radio->v4l2_dev);
err_device_register:
	return ret;
}

/*
 * si468x_i2c_remove - remove the device
 */
static int si468x_i2c_remove(struct i2c_client *client)
{
	struct si468x_device *radio = i2c_get_clientdata(client);

	video_unregister_device(&radio->videodev);

	if (radio->gpio_reset) {
		gpiod_set_value_cansleep(radio->gpio_reset, 1);

        /* Released reset pin for debug purposes */
        msleep(10);
        gpiod_set_value_cansleep(radio->gpio_reset, 0);
    }

 	v4l2_device_unregister(&radio->v4l2_dev);
    return 0;
}

/*
 * si468x_i2c_driver - i2c driver interface
 */
static const struct of_device_id si468x_of_match[] = {
	{ .compatible = "x3399-headunit,si468x" },
	{ },
};
MODULE_DEVICE_TABLE(of, si468x_of_match);

static struct i2c_driver si468x_i2c_driver = {
	.driver = {
		.name		= "si468x",
		.of_match_table = of_match_ptr(si468x_of_match),
	},
	.probe_new		= si468x_i2c_probe,
	.remove			= si468x_i2c_remove,
};

module_i2c_driver(si468x_i2c_driver);

MODULE_AUTHOR("Oleksii Gulchenko <alexey.gulchenko@gmail.com>");
MODULE_DESCRIPTION("x3399 module based headunit RADIO driver");
MODULE_LICENSE("GPL");
