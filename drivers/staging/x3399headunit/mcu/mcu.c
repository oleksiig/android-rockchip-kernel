/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *  Driver for STM32L052 MCU on x3399headunit board
 *
 *  Copyright (C) 2021 Oleksii Gulchenko <alexey.gulchenko@gmail.com>
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>

#define STATUS_ADC_UPDATED      0x8000
#define STATUS_KEY_EVENT        0x4000
#define STATUS_KEY_TUNENEXT     0x0001
#define STATUS_KEY_TUNEPREV     0x0002
#define STATUS_KEY_VOLUMEUP     0x0004
#define STATUS_KEY_VOLUMEDOWN   0x0008
#define STATUS_KEY_EJECTCD      0x0010

#define MCU_TOTAL_KEYS          16

struct mcu_key_data {
    uint16_t mask;
    uint16_t code;
    uint16_t adc_avg;
    bool     pressed;
    bool     released;
    bool     encoder;
};

struct mcu_driver_data {
	struct i2c_client *client;
	struct input_dev *input;
    struct mcu_key_data keymap[MCU_TOTAL_KEYS];
    uint8_t regs[8];
};

const static struct mcu_key_data k_mcu_keymap[MCU_TOTAL_KEYS] = {
    { .mask = STATUS_KEY_TUNENEXT, .code = KEY_NEXTSONG,
        .pressed = false, .released = false, .encoder = true },
    { .mask = STATUS_KEY_TUNEPREV, .code = KEY_PREVIOUSSONG,
        .pressed = false, .released = false, .encoder = true },
    { .mask = STATUS_KEY_VOLUMEUP, .code = KEY_VOLUMEUP,
        .pressed = false, .released = false, .encoder = true },
    { .mask = STATUS_KEY_VOLUMEDOWN, .code = KEY_VOLUMEDOWN,
        .pressed = false, .released = false, .encoder = true },
    { .mask = STATUS_KEY_EJECTCD, .code = KEY_EJECTCD,
        .pressed = false, .released = false },

    /* ADC keys */
    { .code = KEY_BACK,     .adc_avg = 231,
                                .pressed = false, .released = false},
    { .code = KEY_MUTE,     .adc_avg = 580,
                                .pressed = false, .released = false},
    { .code = KEY_RADIO,    .adc_avg = 488,
                                .pressed = false, .released = false},
    { .code = KEY_F1,       .adc_avg = 399,
                                .pressed = false, .released = false},
    { .code = KEY_F2,       .adc_avg = 640,
                                .pressed = false, .released = false},
    { .code = KEY_F3,       .adc_avg = 0,
                                .pressed = false, .released = false},
    { .code = KEY_F4,       .adc_avg = 0,
                                .pressed = false, .released = false},
    { .code = KEY_F5,       .adc_avg = 0,
                                .pressed = false, .released = false},
};

static int x3399headunit_mcu_xfer(struct mcu_driver_data *mdata)
{
    uint8_t reg = 0;
    struct i2c_client *i2c = mdata->client;
    struct i2c_msg msg[2] = {
		{
			.addr = i2c->addr,
			.flags = i2c->flags,
			.len = 1,
			.buf = (char *)&reg,
		},{
			.addr = i2c->addr,
			.flags = i2c->flags | I2C_M_RD,
			.len = sizeof(mdata->regs),
			.buf = (char *)&mdata->regs,
		},
	};

    return i2c_transfer(i2c->adapter, msg, 2);
}

/* Threaded IRQ handler */
static irqreturn_t x3399headunit_mcu_irq_handler(int irq, void *dev_id)
{
    int ret, keyIdx, adcIdx;
	struct mcu_driver_data *mdata = dev_id;
    struct mcu_key_data *keymap = &mdata->keymap[0], *key;
    struct device *dev = &mdata->client->dev;
    struct input_dev *input = mdata->input;
    uint16_t status, adc_value;

    /* Sync data from MCU */
    ret = x3399headunit_mcu_xfer(mdata);
    if (ret < 0) {
        dev_err(dev, "Unable to sync data from the MCU, error %d\n", ret);
		return IRQ_HANDLED;
    }

    /* Check if invalid status */
    if(mdata->regs[6] == 0xff &&  mdata->regs[7] == 0xff) {
        dev_err(dev, "Invalid MCU status (0x%02x 0x%02x)\n",
            mdata->regs[0], mdata->regs[1]);
        return IRQ_HANDLED;
    }

    status = mdata->regs[6] << 8 |
             mdata->regs[7];

/*
    Desired space between keys ranges 10 points
    Measured ADC values with built in keys:
    0x280 eq    640
    0x3c        60 - 10 / 2 = +-25
    0x244 mute  580
    0x5c        92 - 10 / 2 = +-41
    0x1e8 band  488
    0x59        89 - 10 / 2 = +-39
    0x18f navi  399
    0xa8        168 - 10 / 2 = +- 79
    0x0e7 home  231
    Minimal value of diviation +- 25 points
*/

    if(status & STATUS_ADC_UPDATED) /* ADC data updated event occured */
    {
        for(adcIdx = 0; adcIdx < 3; adcIdx++)
        {
            adc_value = mdata->regs[adcIdx] << 8 |
                        mdata->regs[adcIdx + 1];

            if(adcIdx == 0 || adcIdx == 1) {
                /* TODO: To implement processing of external keys */
            }
            else if(adcIdx == 2)
            {
                for(keyIdx = 0; keyIdx < MCU_TOTAL_KEYS; keyIdx++) {
                    key = &keymap[keyIdx];
                    if(key->mask != 0 || key->adc_avg == 0) {
                        continue; /* Skip non ADC keys */
                    }

                    if(adc_value >= (key->adc_avg - 25) &&
                       adc_value <= (key->adc_avg + 25))
                    {
                        key->pressed = true;
                        input_event(input, EV_KEY, key->code, 1);
                        input_sync(input);

                        dev_info(dev, "key=%d adc=%d pressed\n", key->code, key->adc_avg);
                        break;
                    }
                }

                /* end of list reached */
                if(keyIdx == MCU_TOTAL_KEYS)
                {
                    for(keyIdx = 0; keyIdx < MCU_TOTAL_KEYS; keyIdx++) {
                        key = &keymap[keyIdx];
                        if(key->mask != 0 || key->adc_avg == 0) {
                            continue; /* Skip non ADC keys */
                        }

                        if(key->pressed) {
                            key->pressed = false;
                            input_event(input, EV_KEY, key->code, 0);
                            input_sync(input);

                            dev_info(dev, "key=%d adc=%d released\n", key->code, key->adc_avg);
                        }
                    }
                }
            }
        }
    }

    if(status & STATUS_KEY_EVENT) /* key event occured */
    {
        for(keyIdx = 0; keyIdx < MCU_TOTAL_KEYS; keyIdx++) {
            key = &keymap[keyIdx];

            if(status & key->mask) {
                key->pressed = true;

                input_event(input, EV_KEY, key->code, 1);
                input_sync(input);
            }
        }

        msleep(5);

        /* Simulate release for pressed keys */
        for(keyIdx = 0; keyIdx < MCU_TOTAL_KEYS; keyIdx++) {
            key = &keymap[keyIdx];

            if(key->pressed) {
                key->pressed = false;

                input_event(input, EV_KEY, key->code, 0);
                input_sync(input);
            }
        }
    }

    dev_dbg(dev, "MCU regs: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
        mdata->regs[0], mdata->regs[1], mdata->regs[2], mdata->regs[3],
        mdata->regs[4], mdata->regs[5], mdata->regs[6], mdata->regs[7]);

	return IRQ_HANDLED;
}

static int x3399headunit_mcu_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct mcu_driver_data *mdata;
	struct input_dev *input;
    int ret = 0;

	mdata = devm_kzalloc(dev, sizeof(*mdata), GFP_KERNEL);
	if (!mdata)
		return -ENOMEM;

    mdata->client = client;
    memcpy(&mdata->keymap, &k_mcu_keymap, sizeof(mdata->keymap));

    /* Trying to touch the MCU */
    ret = x3399headunit_mcu_xfer(mdata);
    if (ret < 0) {
        dev_err(dev, "Unable to access the MCU, error %d\n", ret);
		return -ENODEV;
    }

	/* Configure input device */
	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

    mdata->input = input;
    input_set_drvdata(input, mdata);

	input->name = "mcu-keys";
	input->id.bustype = BUS_I2C;
	input->id.vendor  = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

    ret = devm_request_threaded_irq(dev, client->irq, NULL,
            x3399headunit_mcu_irq_handler, IRQF_SHARED | IRQF_ONESHOT,
                                    client->name, mdata);
	if (ret) {
		dev_err(dev, "Unable to claim irq %d, error %d\n",
			client->irq, ret);
		return ret;
	}

    input_set_capability(input, EV_KEY, KEY_HOME);
    input_set_capability(input, EV_KEY, KEY_BACK);
    input_set_capability(input, EV_KEY, KEY_MUTE);
    input_set_capability(input, EV_KEY, KEY_VOLUMEDOWN);
    input_set_capability(input, EV_KEY, KEY_VOLUMEUP);
    input_set_capability(input, EV_KEY, KEY_POWER);
    input_set_capability(input, EV_KEY, KEY_PLAYPAUSE);
    input_set_capability(input, EV_KEY, KEY_RADIO);
    input_set_capability(input, EV_KEY, KEY_NEXTSONG);
    input_set_capability(input, EV_KEY, KEY_PREVIOUSSONG);
    input_set_capability(input, EV_KEY, KEY_F1); /* navigation */
    input_set_capability(input, EV_KEY, KEY_F2);
    input_set_capability(input, EV_KEY, KEY_F3);
    input_set_capability(input, EV_KEY, KEY_F4);
    input_set_capability(input, EV_KEY, KEY_F5);

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "Unable to register input device, error: %d\n", ret);
	}

    return ret;
}

static const struct of_device_id x3399headunit_mcu_dt_ids[] = {
	{ .compatible = "x3399-headunit,mcu", },
	{ }
};
MODULE_DEVICE_TABLE(of, x3399headunit_mcu_dt_ids);

static struct i2c_driver x3399headunit_mcu_driver = {
	.driver = {
		.name	= "x3399headunit_mcu",
		.of_match_table = x3399headunit_mcu_dt_ids,
	},
	.probe		= x3399headunit_mcu_probe,
};

static int __init x3399headunit_mcu_init(void)
{
	return i2c_add_driver(&x3399headunit_mcu_driver);
}
subsys_initcall(x3399headunit_mcu_init);

static void __exit x3399headunit_mcu_exit(void)
{
	i2c_del_driver(&x3399headunit_mcu_driver);
}
module_exit(x3399headunit_mcu_exit);

MODULE_AUTHOR("Oleksii Gulchenko <alexey.gulchenko@gmail.com>");
MODULE_DESCRIPTION("x3399 module based headunit MCU driver");
MODULE_LICENSE("GPL");
