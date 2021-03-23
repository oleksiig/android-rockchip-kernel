/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *  Driver for Si468x AM/FM/DAB Radio Receiver on x3399headunit board
 *
 *  Copyright (C) 2021 Oleksii Gulchenko <alexey.gulchenko@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>

#include "si468x.h"

int si468x_get_sys_mode(struct si468x_device *radio)
{
    struct device *dev = &radio->client->dev;
    uint8_t cmd = SI468X_GET_SYS_STATE;
    uint8_t buf[6];

    if(radio->si468x_xfer(radio, &cmd, sizeof(cmd), buf, sizeof(buf)))
        return -EIO;

    dev_dbg(dev, "%s: %02x %02x %02x %02x %02x %02x",
        __func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

    switch(buf[4])
	{
		case 0:
			return SI468X_MODE_BOOT;
		case 1:
		case 4:
			return SI468X_MODE_FM;
		case 2:
		case 3:
			return SI468X_MODE_DAB;
		case 5:
		case 6:
			return SI468X_MODE_AM;
	}
    return SI468X_MODE_UNKNOWN;
}

static int si468x_read_status(struct si468x_device *radio, char *buf, size_t len)
{
    struct device *dev = &radio->client->dev;
    unsigned long stop_jiffies = jiffies + 2*HZ;
    int ret;
    uint8_t cmd = SI468X_RD_REPLY;

    if(!buf || len < 4) {
        return -EINVAL;
    }

    while(time_before(jiffies, stop_jiffies))
    {
        ret = radio->si468x_xfer(radio, &cmd, 1, buf, len);
        if (ret) {
            dev_err(dev, "%s: SI468X_RD_REPLY failed: %d\n", __func__, ret);
            return ret;
        }

        if (buf[0] & 0x80) {
            return 0;
        }

        if (buf[0] & 0x40) {
            return -EIO;
        }

        msleep(1);
    }

    dev_err(dev, "%s: Timeout waiting for CTS\n", __func__);
    return -ETIME;
}

static int si468x_check_status(struct si468x_device *radio)
{
    struct device *dev = &radio->client->dev;
    uint8_t status[4];
    int ret;

    ret = si468x_read_status(radio, status, 4);
    if (ret) {
        dev_err(dev, "%s: read status failed, ret %d\n", __func__, ret);
        return ret;
    }

    if (status[0] & (1 << 6))
    {
        if (status[3] & (1 << 4)) {
            dev_err(dev, "DSPERR: The DSP has encountered a frame overrun.\n");
            ret = -EBUSY;
        }
        if (status[3] & (1 << 3)) {
            dev_err(dev, "REPOFERR: Control interface has dropped data during a reply read.\n");
            ret = -EBUSY;
        }
        if (status[3] & (1 << 2)) {
            dev_err(dev, "CMDOFERR: Control interface has dropped data during a command write.\n");
            ret = -EAGAIN;
        }
        if (status[3] & (1 << 1)) {
            dev_err(dev, "ARBERR: Arbiter error has occurred.\n");
            ret = -EIO;
        }
        if (status[3] & (1 << 0)) {
            dev_err(dev, "ERRNR: Non-recoverable error has occurred.\n");
            ret = -EINVAL;
        }
    }

    dev_dbg(dev, "STATUS: %02x %02x %02x %02x ret=%d\n",
            status[0], status[1], status[2], status[3], ret);
    return ret;
}

static int si468x_load_init(struct si468x_device *radio)
{
    struct device *dev = &radio->client->dev;
    int ret;
    uint8_t cmd[2] = {SI468X_LOAD_INIT, 0 };

    ret = radio->si468x_xfer(radio, cmd, 2, NULL ,0);
    if(ret) {
        dev_err(dev, "%s: SI468X_LOAD_INIT failed, ret %d\n", __func__, ret);
        return ret;
    }

    msleep(4);

    return si468x_check_status(radio);
}

static int si468x_boot(struct si468x_device *radio)
{
    struct device *dev = &radio->client->dev;
    int ret, boot_attempts = 5;
    uint8_t cmd[2] = {SI468X_BOOT, 0 };

    do {
        ret = radio->si468x_xfer(radio, cmd, 2, NULL, 0);
        if(ret) {
            dev_err(dev, "%s: SI468X_BOOT failed, ret %d\n", __func__, ret);
            return ret;
        }

        msleep(250); // 63ms at analog fm, 198ms at DAB

        ret = si468x_check_status(radio);
    } while ((boot_attempts--) && (ret));

    return ret;
}

static int si468x_flash_load(struct si468x_device *radio, uint32_t offset)
{
    struct device *dev = &radio->client->dev;
    int ret;
    uint8_t payload[12];

    payload[0] = SI468X_FLASH_LOAD;
    payload[1] = 0;
    payload[2] = 0;
    payload[3] = 0;
    payload[4] = (offset & 0xff);
    payload[5] = (offset >> 8) & 0xff;
    payload[6] = (offset >> 16) & 0xff;
    payload[7] = (offset >> 24) & 0xff;
    payload[8] = 0;
    payload[9] = 0;
    payload[10] = 0;
    payload[11] = 0;

    ret = radio->si468x_xfer(radio, payload, 12, NULL, 0);
    if(ret) {
        dev_err(dev, "%s: SI468X_FLASH_LOAD failed, ret %d\n", __func__, ret);
        return ret;
    }

    return si468x_check_status(radio);
}

static int si468x_host_load_data(struct si468x_device *radio,
                                const uint8_t *buf, size_t len)
{
    struct device *dev = &radio->client->dev;
    int ret;
    size_t payload_size = len + 4;
    uint8_t *payload = kzalloc(payload_size, GFP_DMA);

    if (!payload) {
        dev_err(dev, "%s: alloc %zu bytes for payload buffer failed\n",
            __func__, payload_size);
        return -ENOMEM;
    }

    payload[0] = SI468X_HOST_LOAD;
    payload[1] = 0;
    payload[2] = 0;
    payload[3] = 0;

    memcpy(payload + 4, buf, len);

    ret = radio->si468x_xfer(radio, payload, payload_size, NULL, 0);

    kfree(payload);
    return ret;
}

static int si468x_upload_firmware_patch(struct si468x_device *radio)
{
    const char *fw_name = "si468x_patch.bin";
    const struct firmware *fw_p;
    struct device *dev = &radio->client->dev;
    int ret, remaining_bytes, count_to;

    ret = si468x_load_init(radio);
    if (ret) {
        dev_err(dev, "%s: Loader init failed, ret %d\n", __func__, ret);
        return ret;
    }

    ret = request_firmware(&fw_p, fw_name, dev);
    if (ret) {
        dev_err(dev, "%s: patch FW %s not found\n", __func__, fw_name);
        return ret;
    }

    dev_info(dev, "Loading patch FW %s (%zu bytes)\n", fw_name, fw_p->size);

    remaining_bytes = fw_p->size;
    while(remaining_bytes) {
        if(remaining_bytes >= 4096) {
            count_to = 4096;
        } else {
            count_to = remaining_bytes;
        }

        ret = si468x_host_load_data(radio, fw_p->data + (fw_p->size - remaining_bytes), count_to);
        if(ret) {
            dev_err(dev, "%s: patch FW transfer failed.\n", __func__);
            goto err_host_load;
        }

        remaining_bytes -= count_to;
    }

    msleep(4);

    ret = si468x_check_status(radio);
    if (ret) {
        dev_err(dev, "patch FW load error, ret %d\n", ret);
        goto err_host_load;
    }

    msleep(4);

    dev_info(dev, "%s: patch FW %s loaded\n", __func__, fw_name);

err_host_load:
    release_firmware(fw_p);
    return ret;
}

static int si468x_powerup(struct si468x_device *radio)
{
    struct device *dev = &radio->client->dev;
    int ret;
    uint8_t payload[16];

    payload[0] = SI468X_POWER_UP;
    //payload[1] = 0x80;              // ARG1 CTSIEN=1
    payload[1] = 0x00;              // ARG1 CTSIEN=0
    payload[2] = (1<<4) | (7<<0);   // ARG2 CLK_MODE=0x1 TR_SIZE=0x7
    payload[3] = 0x48;              // ARG3 IBIAS=0x28
    payload[4] = 0x00;              // ARG4 XTAL_FREQ
    payload[5] = 0xF9;              // ARG5 XTAL_FREQ
    payload[6] = 0x24;              // ARG6 XTAL_FREQ
    payload[7] = 0x01;              // ARG7 XTAL_FREQ 19.2MHz
    payload[8] = 0x1F;              // ARG8 CTUN
    payload[9] = 0x00 | (1<<4);     // ARG9
    payload[10] = 0x00;             // ARG10
    payload[11] = 0x00;             // ARG11
    payload[12] = 0x00;             // ARG12
    payload[13] = 0x00;             // ARG13 IBIAS_RUN
    payload[14] = 0x00;             // ARG14
    payload[15] = 0x00;             // ARG15

    ret = radio->si468x_xfer(radio, payload, sizeof(payload), NULL, 0);
    if (ret) {
        dev_err(dev, "%s: SI468X_POWER_UP failed, ret %d\n", __func__, ret);
        return ret;
    }

    msleep(1); /* Wait least 20 us */
    return si468x_check_status(radio);
}

int si468x_set_property(struct si468x_device *radio, uint16_t property_id, uint16_t value)
{
    struct device *dev = &radio->client->dev;
    int ret;
    uint8_t payload[6];

    dev_dbg(dev, "%s: prop:0x%04x val:0x%04x\n", __func__, property_id, value);

    payload[0] = SI468X_SET_PROPERTY;
    payload[1] = 0;
    payload[2] = property_id & 0xFF;
    payload[3] = (property_id >> 8) & 0xFF;
    payload[4] = value & 0xFF;
    payload[5] = (value >> 8) & 0xFF;

    ret = radio->si468x_xfer(radio, payload, 6, NULL, 0);
    if (ret) {
        dev_err(dev, "SI468X_SET_PROPERTY failed: %d\n", ret);
        return ret;
    }

    ret = si468x_check_status(radio);
    if (ret) {
        dev_err(dev, "%s: failed: %d\n", __func__, ret);
    }

    return ret;
}

static int si468x_init_boot_mode(struct si468x_device *radio)
{
    int ret;
    struct device *dev = &radio->client->dev;

    /* Reset the chip */
    gpiod_set_value_cansleep(radio->gpio_reset, 1);
    msleep(10);
    gpiod_set_value_cansleep(radio->gpio_reset, 0);
    msleep(10);

    ret = si468x_powerup(radio);
    if (ret) {
        dev_err(dev, "Power up failed, ret %d\n", ret);
        return ret;
    }

    ret = si468x_upload_firmware_patch(radio);
    if (ret) {
        dev_err(dev, "Patch load failed, ret %d\n", ret);
        return ret;
    }

    return 0;
}

static int si468x_boot_flash(struct si468x_device *radio, int offset)
{
    struct device *dev = &radio->client->dev;
    int ret;

    dev_info(dev, "Boot from NVFLASH, offset 0x%x\n", offset);

    ret = si468x_load_init(radio);
    if (ret) {
        dev_err(dev, "LOAD_INIT failed: %d\n", ret);
        return ret;
    }

    ret = si468x_flash_load(radio, offset);
    if (ret) {
        dev_err(dev, "FLASH_LOAD failed: %d\n", ret);
        return ret;
    }

    ret = si468x_boot(radio);
    if (ret) {
        dev_err(dev, "BOOT failed: %d\n", ret);
        return ret;
    }

    dev_info(dev, "%s: done\n", __func__);
    return 0;
}

int si468x_start_fm(struct si468x_device *radio, struct si468x_band_desc *band_desc)
{
    struct device *dev = &radio->client->dev;
    int ret, offset = SI468X_FLASH_OFFSET_FM;
    uint16_t deemphasis;

    if (radio->mode == SI468X_MODE_FM)
        return 0;

    radio->mode = SI468X_MODE_UNKNOWN;

    ret = si468x_init_boot_mode(radio);
    if (ret) {
        dev_err(dev, "si468x_init_boot_mode failed: %d\n", ret);
        return ret;
    }

    ret = si468x_boot_flash(radio, offset);
    if (ret) {
        dev_err(dev, "si468x_boot_flash FM mode failed: %d\n" \
            "Make sure that flash offset 0x%x is programmed!\n", ret, offset);
        return ret;
    }

    si468x_get_sys_mode(radio);

    si468x_set_property(radio, SI468X_PIN_CONFIG_ENABLE,
        (1 << 15) |     // INTBOUTEN
        (1 << 1));      // I2SOUTEN

    si468x_set_property(radio, SI468X_FM_RDS_CONFIG, 0x0001); // enable RDS

    si468x_set_property(radio, SI468X_FM_RDS_INTERRUPT_SOURCE,
        (1 << 3) |      // RDSPI
        (1 << 1) |      // RDSSYNC
        (1 << 0));      // RDSRECV

    si468x_set_property(radio, SI468X_FM_RSQ_INTERRUPT_SOURCE,
        (1 << 3) |      // SNRHINT
        (1 << 2) |      // SNRLINT
        (1 << 1) |      // RSSIHINT
        (1 << 0));      // RSSILINT

    si468x_set_property(radio, SI468X_FM_ACF_INTERRUPT_SOURCE,
        (1 << 2));      // BLEND_INTEN

    /* */
    if(band_desc->deemphasis == RADIO_DEEMPHASIS_D50) {
        deemphasis = SI468X_AUDIO_DE_EMPHASIS_EU;
    } else if(band_desc->deemphasis == RADIO_DEEMPHASIS_D75) {
        deemphasis = SI468X_AUDIO_DE_EMPHASIS_US;
    } else {
        deemphasis = SI468X_AUDIO_DE_EMPHASIS_OFF;
    }
    si468x_set_property(radio, SI468X_FM_AUDIO_DE_EMPHASIS, deemphasis);

    /* */
    si468x_set_property(radio, SI468X_FM_SOFTMUTE_SNR_LIMITS, 0x0000); // set the SNR limits for soft mute attenuation
    si468x_set_property(radio, SI468X_FM_TUNE_FE_CFG, 1); // VHFSW front end switch open

    /* */
    si468x_set_property(radio, SI468X_FM_SEEK_BAND_BOTTOM,
        band_desc->rangelow_khz / 10);

    si468x_set_property(radio, SI468X_FM_SEEK_BAND_TOP,
        band_desc->rangehigh_khz / 10);

    /* I2S sample rate */
    si468x_set_property(radio, SI468X_DIGITAL_IO_OUTPUT_SAMPLE_RATE,
        48000);

    /* I2S slave */
    si468x_set_property(radio, SI468X_DIGITAL_IO_OUTPUT_SELECT,
        0x0);

    /* I2S format */
    si468x_set_property(radio, SI468X_DIGITAL_IO_OUTPUT_FORMAT,
        (16 << 8) |     // sample size 16
        (4 << 4) |      // slot size 16
        (0 << 0));      // right_j mode

    /* Enable required IRQs */
    si468x_set_property(radio, SI468X_INT_CTL_ENABLE,
        //(1 << 3) |      // RSQIEN
        (1 << 2) |      // RDSIEN
        //(1 << 1) |      // ACFINT
        (1 << 0));      // STCIEN

    /* */
    radio->mode = SI468X_MODE_FM;
    dev_info(dev, "FM mode initialized\n");
    return 0;
}

int si468x_start_am(struct si468x_device *radio, struct si468x_band_desc *band_desc)
{
    struct device *dev = &radio->client->dev;
    int ret, offset = SI468X_FLASH_OFFSET_AM;

    if (radio->mode == SI468X_MODE_AM)
        return 0;

    radio->mode = SI468X_MODE_UNKNOWN;

    ret = si468x_init_boot_mode(radio);
    if (ret) {
        dev_err(dev, "si468x_init_boot_mode failed: %d\n", ret);
        return ret;
    }

    ret = si468x_boot_flash(radio, offset);
    if (ret) {
        dev_err(dev, "si468x_boot_flash AM mode failed: %d\n" \
                "FLASH offset 0x%x is programmed?\n", ret, offset);
        return ret;
    }

    si468x_set_property(radio, SI468X_PIN_CONFIG_ENABLE,
        (1 << 15) |     // INTBOUTEN
        (1 << 1));      // I2SOUTEN

    /* */
    si468x_set_property(radio, SI468X_AM_AVC_MIN_GAIN, 0xF800);
    si468x_set_property(radio, SI468X_AM_AVC_MAX_GAIN, 32767); /* 193dB */
    si468x_set_property(radio, SI468X_AM_VALID_SNR_THRESHOLD, 0x80); /* 10 dB */ // -128
    si468x_set_property(radio, SI468X_AM_VALID_RSSI_THRESHOLD, 0x80); /* 30 dBuV */ // -128
    //si468x_set_property(radio, SI468X_AM_CHBW_OVERRIDE_BW, 55);

#if 0
    SI473X_SetProperty(SI473X_PROP_AM_CHANNEL_FILTER, 1); /*4 kHz Bandwidth*/
    SI473X_SetProperty(SI473X_PROP_AM_RSQ_SNR_HIGH_THRESHOLD, 0x0a); /* 10 dB */
    SI473X_SetProperty(SI473X_PROP_AM_RSQ_SNR_LOW_THRESHOLD, 0x0a); /* 10 dB */
    SI473X_SetProperty(SI473X_PROP_AM_RSQ_RSSI_HIGH_THRESHOLD, 0x1e); /* 30 dBuV */
    SI473X_SetProperty(SI473X_PROP_AM_RSQ_RSSI_LOW_THRESHOLD, 0x0a); /* 10 dBuV */
    SI473X_SetProperty(SI473X_PROP_AM_SOFT_MUTE_MAX_ATTENUATION, 0x0a); /* 10 dB */
    SI473X_SetProperty(SI473X_PROP_AM_SOFT_MUTE_SNR_THRESHOLD, 0x09); /* 9 dB */

   // SI473X_SetProperty(SI473X_PROP_AM_SEEK_SNR_THRESHOLD, 0x0b); /* 11 dB */
   // SI473X_SetProperty(SI473X_PROP_AM_SEEK_FREQ_SPACING, 0x05); /* 10 kHz */
   // SI473X_SetProperty(SI473X_PROP_AM_SEEK_RSSI_THRESHOLD, 0x2a); /* 42 dBuV */
#endif


    /* */
    si468x_set_property(radio, SI468X_AM_SEEK_BAND_BOTTOM,
        band_desc->rangelow_khz);

    si468x_set_property(radio, SI468X_AM_SEEK_BAND_TOP,
        band_desc->rangehigh_khz);

    si468x_set_property(radio, SI468X_AM_SEEK_FREQUENCY_SPACING,
        1); // 1kHz

    /* I2S sample rate */
    si468x_set_property(radio, SI468X_DIGITAL_IO_OUTPUT_SAMPLE_RATE,
        48000);
    /* I2S slave */
    si468x_set_property(radio, SI468X_DIGITAL_IO_OUTPUT_SELECT,
        0x0);
    /* I2S format */
    si468x_set_property(radio, SI468X_DIGITAL_IO_OUTPUT_FORMAT,
        (16 << 8) |     // sample size 16
        (4 << 4) |      // slot size 16
        (0 << 0));      // right_j mode

    /* Enable required IRQs */
    si468x_set_property(radio, SI468X_INT_CTL_ENABLE,
        (1 << 3) |      // RSQIEN
        (1 << 0));      // STCIEN

    /* */
    radio->mode = SI468X_MODE_AM;
    dev_info(dev, "AM mode initialized\n");
    return 0;
}

int si468x_start_dab(struct si468x_device *radio, struct si468x_band_desc *band_desc)
{
    pr_info("%s: unsupported \n", __func__);
    return -1;
}

int si468x_tune(struct si468x_device *radio, uint32_t freqKhz)
{
    struct device *dev = &radio->client->dev;
    int ret;
    uint8_t payload[6];
    uint16_t antCap = 0;

    dev_dbg(dev, "%s: freqKhz=%d\n", __func__, freqKhz);

    if (radio->mode == SI468X_MODE_AM) {
        payload[0] = SI468X_AM_TUNE_FREQ;
        antCap = 4096;
    }
    else if (radio->mode == SI468X_MODE_FM) {
        payload[0] = SI468X_FM_TUNE_FREQ;
        freqKhz /= 10;
    } else {
        dev_err(dev, "%s: Invalid mode %d\n", __func__, radio->mode);
        return -ENODATA;
    }

    mutex_lock(&radio->irq_flags_lock);
    radio->irq_flags = 0;
    mutex_unlock(&radio->irq_flags_lock);

    payload[1] = 0; // Normal, Tune and render audio asap, injection auto
    payload[2] = (freqKhz & 0xff);
    payload[3] = (freqKhz >> 8) & 0xff;
    payload[4] = (antCap & 0xff);
    payload[5] = (antCap >> 8) & 0xff;

    ret = radio->si468x_xfer(radio, payload, 6, NULL, 0);
    if(ret) {
        dev_err(dev, "%s: SI468X_TUNE_FREQ failed, ret %d\n", __func__, ret);
        return ret;
    }

    ret = si468x_check_status(radio);
    if (ret) {
        dev_err(dev, "%s: failed: %d\n", __func__, ret);
    }

    return ret;
}

int si468x_seek_start(struct si468x_device *radio, uint8_t up, uint8_t wrap)
{
    struct device *dev = &radio->client->dev;
    int ret;
    uint8_t payload[6];

    dev_dbg(dev, "%s: up=%d, wrap=%d\n", __func__, (int)up, (int)wrap);

    if (radio->mode == SI468X_MODE_AM)
        payload[0] = SI468X_AM_SEEK_START;
    else if (radio->mode == SI468X_MODE_FM)
        payload[0] = SI468X_FM_SEEK_START;
    else {
        dev_err(dev, "%s: Invalid mode %d\n", __func__, radio->mode);
        return -ENODATA;
    }

    mutex_lock(&radio->irq_flags_lock);
    radio->irq_flags = 0;
    mutex_unlock(&radio->irq_flags_lock);

    payload[1] = 0; // Normal, Tune and render audio asap, injection auto
    payload[2] = (!!up) << 1 | (!!wrap);
    payload[3] = 0;
    payload[4] = 0; // ANTCAP=0 automatic
    payload[5] = 0; // ANTCAP=0 automatic

    ret = radio->si468x_xfer(radio, payload, 6, NULL, 0);
    if(ret) {
        dev_err(dev, "%s: SI468X_SEEK_START failed, ret %d\n", __func__, ret);
        return ret;
    }

    ret = si468x_check_status(radio);
    if (ret) {
        dev_err(dev, "%s: failed: %d\n", __func__, ret);
    }

    return ret;
}

static uint8_t si46xx_rds_parse(uint16_t *blocks, struct si468x_rds_status *status)
{
	uint8_t addr;
	status->program_id = blocks[0];

	if((blocks[1] & 0xF800) == 0x00) // group 0A
	{
        addr = blocks[1] & 0x03;
		status->ps_name[addr * 2]     = (blocks[3] & 0xFF00)>>8;
		status->ps_name[addr * 2 + 1] =  blocks[3] & 0xFF;
		status->group_0a_flags |= (1<<addr);
	}
    else if(((blocks[1] & 0xF800) >> 11) == 0x04) // group 2A
    {
		addr = blocks[1] & 0x0F;
		if((blocks[1] & 0x10) == 0x00) // parse only string A
        {
			status->radiotext[addr * 4]     = (blocks[2] & 0xFF00) >> 8;
			status->radiotext[addr * 4 + 1] = (blocks[2] & 0xFF);
			status->radiotext[addr * 4 + 2] = (blocks[3] & 0xFF00) >> 8;
			status->radiotext[addr * 4 + 3] = (blocks[3] & 0xFF);

			if(status->radiotext[addr * 4] == '\r')
            {
				status->radiotext[addr * 4] = 0;
				status->group_2a_flags = 0xFFFF;
			}

			if(status->radiotext[addr * 4 + 1] == '\r')
            {
				status->radiotext[addr * 4 + 1] = 0;
				status->group_2a_flags = 0xFFFF;
			}

			if(status->radiotext[addr * 4 + 2] == '\r')
            {
				status->radiotext[addr * 4 + 2] = 0;
				status->group_2a_flags = 0xFFFF;
			}

			if(status->radiotext[addr*4+3] == '\r')
            {
				status->radiotext[addr * 4 + 3] = 0;
				status->group_2a_flags = 0xFFFF;
			}

			status->group_2a_flags |= (1 << addr);
		}
	}

	if(status->group_0a_flags == 0x0F && status->group_2a_flags == 0xFFFF)
    {
		status->ps_name[8] = 0;
		status->radiotext[128] = 0;
		return 1;
	}

	return 0;
}

int si468x_fm_read_rds_status(struct si468x_device *radio)
{
    struct device *dev = &radio->client->dev;
    struct si468x_rds_status *status = &radio->status.rds;
    uint16_t blocks[4];
    uint8_t payload[20];
    unsigned long stop_jiffies = jiffies + 1*HZ;
    int ret;

    if (radio->mode != SI468X_MODE_FM) {
        dev_err(dev, "%s: Invalid mode %d\n", __func__, radio->mode);
        return -ENODATA;
    }

    while(time_before(jiffies, stop_jiffies))
    {
        payload[0] = SI468X_FM_RDS_STATUS;
        payload[1] = 1;

        ret = radio->si468x_xfer(radio, payload, 2, NULL, 0);
        if (ret) {
            dev_err(dev, "SI468X_FM_RDS_STATUS failed: %d\n", ret);
            return ret;
        }

        ret = si468x_read_status(radio, payload, 20);
        if (ret) {
            dev_err(dev, "%s: failed: %d\n", __func__, ret);
            return ret;
        }

        blocks[0] = payload[12] | (payload[13] << 8);
        blocks[1] = payload[14] | (payload[15] << 8);
        blocks[2] = payload[16] | (payload[17] << 8);
        blocks[3] = payload[18] | (payload[19] << 8);

        if(payload[5] & (1 << 3)) { // PIVALID
            status->program_id = payload[8] | (payload[9] << 8);
        }

        if(payload[5] & (1 << 4)) { // TPPTYVALID
            status->pty_code = payload[6] & 0x1f;
            status->tp = (payload[6] >> 5) & 0x1;
        }

        if(!(payload[5] & (1 << 1))) // RDSSYNC
            break;

        if (si46xx_rds_parse(blocks, status))
            break;

        msleep(1);
    }

    if (time_after(jiffies, stop_jiffies)) {
      //  dev_warn(dev, "Timeout wait for RDS data sync.\n");
        return -ETIME;
    }

    return 0;
}

int si468x_read_rsq_status(struct si468x_device *radio)
{
    struct device *dev = &radio->client->dev;
    struct si468x_rsq_status *status = &radio->status.rsq;
    int ret;
    uint8_t payload[22];

    if (radio->mode == SI468X_MODE_AM)
        payload[0] = SI468X_AM_RSQ_STATUS;
    else if (radio->mode == SI468X_MODE_FM)
        payload[0] = SI468X_FM_RSQ_STATUS;
    else {
        dev_err(dev, "%s: Invalid mode %d\n", __func__, radio->mode);
        return -ENODATA;
    }

    payload[1] = (1 << 0) |  /*STCACK*/
                 (1 << 3);   /*RSQACK*/

    ret = radio->si468x_xfer(radio, payload, 2, NULL, 0);
    if (ret) {
        dev_err(dev, "SI468X_RSQ_STATUS failed: %d\n", ret);
        return ret;
    }

    ret = si468x_read_status(radio, payload, 22);
    if (ret) {
        dev_err(dev, "%s: failed: %d\n", __func__, ret);
        return ret;
    }

    status->freq_khz            = (payload[6] | (payload[7] << 8));

    if (radio->mode == SI468X_MODE_FM)
        status->freq_khz *= 10;

    status->freq_offset_ppm     = payload[8];
    status->rssi_dbuv           = payload[9];
    status->snr_db              = payload[10];
    status->ant_cap             = (payload[12] | (payload[13] << 8));
    status->multipath_ind       = payload[11];

    return ret;
}

int si468x_fm_read_acf_status(struct si468x_device *radio)
{
    struct device *dev = &radio->client->dev;
    struct si468x_acf_status *status = &radio->status.acf;
    int ret;
    uint8_t payload[11];

    if (radio->mode != SI468X_MODE_FM) {
        dev_err(dev, "%s: Invalid mode %d\n", __func__, radio->mode);
        return -ENODATA;
    }

    payload[0] = SI468X_FM_ACF_STATUS;
    payload[1] = (1 << 0);   /*ACFACK*/

    ret = radio->si468x_xfer(radio, payload, 2, NULL, 0);
    if (ret) {
        dev_err(dev, "SI468X_FM_ACF_STATUS failed: %d\n", ret);
        return ret;
    }

    ret = si468x_read_status(radio, payload, 11);
    if (ret) {
        dev_err(dev, "%s: failed: %d\n", __func__, ret);
        return ret;
    }

    status->blend_state     = (payload[5] >> 2) & 0x01;
    status->pilot           = (payload[8] >> 7) & 0x01;
    status->stereo_level    = (payload[8] & 0x7f);

    return ret;
}
