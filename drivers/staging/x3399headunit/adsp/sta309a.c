/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *  Driver for STA309A audio processor on x3399headunit board
 *
 *  Copyright (C) 2021 Oleksii Gulchenko <alexey.gulchenko@gmail.com>
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/clk.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/soc-component.h>
#include <sound/tlv.h>

#include "sta309a-regs.h"

/* #define DEBUG_COEFICIENTS */

/* ------------------------------------------------------------------ */
enum sta309a_mix_route {
    e_mix_route_none = 0,
    e_mix_route_radio = 1,
    e_mix_route_dvd = 2
};

struct sta309a_data {
    struct i2c_client *client;
    struct gpio_desc *powerdown_gpio;
    struct gpio_desc *reset_gpio;
    struct clk *mclk;
    struct snd_soc_dai *amp_dai;
    struct mutex lock;

    enum sta309a_mix_route route;
};

/* ------------------------------------------------------------------ */
static int sta309a_i2c_xfer(struct sta309a_data *adsp,
    const void *write_ptr, size_t write_sz, void *read_ptr, size_t read_sz)
{
    uint8_t addr = adsp->client->addr;
    int msgs_num = (read_sz > 0) ? 2 : 1;
    int ret;

    struct i2c_msg msgs[2] = {
        { .addr = addr, .flags = 0,      .len = write_sz, .buf = (uint8_t*)write_ptr },
        { .addr = addr, .flags = I2C_M_RD, .len = read_sz, .buf = (uint8_t*)read_ptr },
    };

    mutex_lock(&adsp->lock);
    ret = i2c_transfer(adsp->client->adapter, msgs, msgs_num);
    mutex_unlock(&adsp->lock);

    if (ret != msgs_num) {
        dev_err(&adsp->client->dev, "i2c_transfer failed, ret %d\n", ret);
        return ret;
    }

    return 0;
}

static int sta309a_reg8_write(struct sta309a_data *adsp, uint8_t reg, uint8_t val)
{
    struct device *dev = &adsp->client->dev;
    const uint8_t reg_val[2] = {reg, val};
    int ret;

    dev_info(dev, "%s: reg=0x%02x val=0x%02x\n", __func__, reg, val);

    ret = sta309a_i2c_xfer(adsp, &reg_val, 2, NULL, 0);
    if (ret) {
        dev_err(dev, "%s: reg (0x%02x) write failed: %d\n",
            __func__, reg, ret);
    }
    return ret;
}

#ifdef DEBUG_COEFICIENTS
static int sta309a_coef_read(struct sta309a_data *adsp, uint16_t coef_addr, uint32_t *coef_value)
{
    uint8_t payload[3];
    int ret;

    /* clear before read */
    *coef_value = 0;

    /* write 10-bits of coefficient address at address 0x3B */
    payload[0] = 0x3B;
    payload[1] = (coef_addr >> 8) & 0x03;
    payload[2] = coef_addr & 0xff;

    ret = sta309a_i2c_xfer(adsp, &payload, 3, NULL, 0);
    if (ret) {
        dev_err(&adsp->client->dev,
            "%s: coeficient addr (0x%03x) write failed: %d\n", __func__,
                coef_addr, ret);
        return ret;
    }

    /* read 24-bits of coefficient address at address 0x3D */
    payload[0] = 0x3D;
    ret = sta309a_i2c_xfer(adsp, &payload, 1, payload, 3);
    if (ret) {
        dev_err(&adsp->client->dev,
            "%s: coeficient (0x%03x) read failed: %d\n", __func__,
                coef_addr, ret);
    } else {
        *coef_value = payload[0] << 16 |
                      payload[1] << 8 |
                      payload[2];

    }

    return ret;
}
#endif

static int sta309a_coef_write(struct sta309a_data *adsp, uint16_t coef_addr, uint32_t coef_value)
{
    uint8_t payload[4];
    int ret;

    /* write 10-bits of coefficient address at address 0x3B */
    payload[0] = 0x3B;
    payload[1] = (coef_addr >> 8) & 0x03;
    payload[2] = coef_addr & 0xff;

    ret = sta309a_i2c_xfer(adsp, &payload, 3, NULL, 0);
    if (ret) {
        dev_err(&adsp->client->dev, "%s: reg 0x%02x write failed: %d\n",
            __func__, payload[0], ret);
        return ret;
    }

    /* write 24-bits of coefficient at address 0x3D */
    payload[0] = 0x3D;
    payload[1] = (coef_value >> 16) & 0xff;
    payload[2] = (coef_value >> 8) & 0xff;
    payload[3] = (coef_value) & 0xff;
    ret = sta309a_i2c_xfer(adsp, &payload, 4, NULL, 0);
    if (ret) {
        dev_err(&adsp->client->dev, "%s: reg 0x%02x write failed: %d\n",
            __func__, payload[0], ret);
    }

    /* Commit changes, write 1 to W1 bit in I2C address 0x4C */
    payload[0] = 0x4C;
    payload[1] = 0x01; /* W1 bit */
    ret = sta309a_i2c_xfer(adsp, &payload, 2, NULL, 0);
    if (ret) {
        dev_err(&adsp->client->dev, "%s: reg 0x%02x write failed: %d\n",
            __func__, payload[0], ret);
    }

    return ret;
}

static unsigned int sta309a_component_read(struct snd_soc_component *component,
                 unsigned int reg)
{
    struct sta309a_data *adsp = snd_soc_component_get_drvdata(component);
    unsigned int val = 0;
    int ret;

    ret = sta309a_i2c_xfer(adsp, &reg, 1, &val, 1);
    if (ret) {
        dev_err(&adsp->client->dev, "%s: reg (0x%02x) read failed: %d\n",
            __func__, reg, ret);
        return ret;
    }

    return val;
}

static int sta309a_component_write(struct snd_soc_component *component,
             unsigned int reg, unsigned int val)
{
    struct sta309a_data *adsp = snd_soc_component_get_drvdata(component);
    uint8_t reg_val[2] = {reg, val};
    int ret;

    dev_info(component->dev, "%s() reg=0x%02x val=0x%02x\n", __func__, reg, (uint8_t)val);

    ret = sta309a_i2c_xfer(adsp, &reg_val, 2, NULL, 0);
    if (ret) {
        dev_err(&adsp->client->dev, "%s: reg (0x%02x) write failed: %d\n",
            __func__, reg, ret);
    }
    return ret;
}

/* ------------------------------------------------------------------ */
static int sta309a_hw_params(struct snd_pcm_substream *substream,
                 struct snd_pcm_hw_params *params,
                 struct snd_soc_dai *dai)
{
    struct snd_soc_component *component = dai->component;
    struct sta309a_data *adsp = snd_soc_component_get_drvdata(component);
    unsigned int rate = params_rate(params);
    unsigned int width = params_width(params);
    u8 confa = 0;
    int ret;

    if(substream->stream > 0) /* nothing to config for capture stream */
        return 0;

    dev_dbg(component->dev, "%s() rate=%u width=%u\n", __func__, rate, width);

    /* All settings for fs=256 */

    /* Master clock select */
    switch (rate) {
    case 32000:
    case 44100:
    case 48000:
        confa = STA309A_CONFA_MCS_0 | STA309A_CONFA_MCS_1;
        break;
    case 88200:
    case 96000:
    case 176400:
    case 192000:
        confa = STA309A_CONFA_MCS_0;
        break;
    default:
        dev_err(component->dev, "Unsupported sample rate: %u\n", rate);
        return -EINVAL;
    }

    /* Interpolation ratio bits */
    switch (rate) {
    /* 32000, 44100, 48000: IR=0 4-times oversampling */
    case 88200:
    case 96000:
        confa |= STA309A_CONFA_IR_0; /*  2-times oversampling */
        break;
    case 176400:
    case 192000:
        confa |= STA309A_CONFA_IR_1; /* Pass-through */
        break;
    }

    /* CKOUT frequency: PLL output / 16 */
    confa |= STA309A_CONFA_COS_PLL_DIV16;

    /* commit changes */
    ret = snd_soc_component_write(component, STA309A_REG_CONFA, confa);
    if(ret < 0) {
        dev_err(component->dev,
            "%s() STA309A_REG_CONFA write failed, ret %d\n", __func__, ret);
    }

    /* */
    if (adsp->amp_dai && adsp->amp_dai->driver) {
        struct snd_soc_dai_driver *amp_driver = adsp->amp_dai->driver;

        if (amp_driver->ops && amp_driver->ops->hw_params)
            ret = amp_driver->ops->hw_params(substream, params, adsp->amp_dai);
    }

    return 0;
}

static int sta309a_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    struct snd_soc_component *component = dai->component;
    struct sta309a_data *adsp = snd_soc_component_get_drvdata(component);
    int ret = 0;

    dev_dbg(component->dev, "%s() fmt=0x%0x\n", __func__, fmt);

    /* clock masters */
    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
    case SND_SOC_DAIFMT_CBS_CFS:
        break;
    default:
        dev_err(component->dev, "Supported only clock slave mode\n");
        return -EINVAL;
    }

    /* signal polarity */
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
    case SND_SOC_DAIFMT_NB_NF:
        break;
    default:
        dev_err(component->dev, "Invalid DAI clock signal polarity\n");
        return -EINVAL;
    }

    /* */
    if (adsp->amp_dai && adsp->amp_dai->driver) {
        struct snd_soc_dai_driver *amp_driver = adsp->amp_dai->driver;

        if (amp_driver->ops && amp_driver->ops->set_fmt)
            ret = amp_driver->ops->set_fmt(adsp->amp_dai, fmt);
    }

    return ret;
}

static int sta309a_set_bias_level(struct snd_soc_component *component,
                  enum snd_soc_bias_level level)
{
    struct sta309a_data *adsp = snd_soc_component_get_drvdata(component);
    int ret = 0;

    dev_dbg(component->dev, "%s() level=%d\n", __func__, level);

    switch (level) {
    case SND_SOC_BIAS_ON:
        /* Put to normal state external apmplifier, EAPD=1 */
        snd_soc_component_write(component, STA309A_REG_CONFI, STA309A_CONFI_EAPD);
        break;
    case SND_SOC_BIAS_PREPARE:
        if (IS_ERR(adsp->mclk))
            break;

        if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_ON) {
            clk_disable_unprepare(adsp->mclk);
        } else {
            ret = clk_prepare_enable(adsp->mclk);
            if (ret)
                return ret;
        }

        break;
    case SND_SOC_BIAS_STANDBY:
        if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_OFF) {
          //  if(adsp->powerdown_gpio)
           //     gpiod_set_value_cansleep(adsp->powerdown_gpio, 0);
        }
        break;
    case SND_SOC_BIAS_OFF:
       // if(adsp->powerdown_gpio)
         //   gpiod_set_value_cansleep(adsp->powerdown_gpio, 1);

        /* Put to normal state external apmplifier, EAPD=1 */
        //snd_soc_component_write(component, STA309A_REG_CONFI, STA309A_CONFI_EAPD);
        break;
    }

    if(adsp->amp_dai) {
        ret = snd_soc_component_set_bias_level(adsp->amp_dai->component, level);
    }

    return ret;
}

static int sta309a_component_probe(struct snd_soc_component *component)
{
    struct sta309a_data *adsp = snd_soc_component_get_drvdata(component);
    struct snd_soc_card *card = component->card;
    struct snd_soc_aux_dev *aux_dev;

    if(card->num_aux_devs) {
        aux_dev = card->aux_dev;
        if (aux_dev) {
            adsp->amp_dai = snd_soc_find_dai(&aux_dev->dlc);
            if(adsp->amp_dai) {
                dev_info(component->dev, "%s() found aux device %s\n",
                    __func__, adsp->amp_dai->name);
            }
        }
    }

    return 0;
}

static const struct snd_soc_dapm_widget sta309a_dapm_widgets[] =
{
    /* */
    SND_SOC_DAPM_INPUT("MICL"),
    SND_SOC_DAPM_INPUT("MICR"),

    /* */
    SND_SOC_DAPM_OUTPUT("SPKDAT1L"),
    SND_SOC_DAPM_OUTPUT("SPKDAT1R"),
    SND_SOC_DAPM_OUTPUT("SPKDAT2L"),
    SND_SOC_DAPM_OUTPUT("SPKDAT2R"),
    SND_SOC_DAPM_OUTPUT("SPKDAT3L"),
    SND_SOC_DAPM_OUTPUT("SPKDAT3R"),
    SND_SOC_DAPM_OUTPUT("SPKDAT4L"),
    SND_SOC_DAPM_OUTPUT("SPKDAT4R"),
};

static const struct snd_soc_dapm_route sta309a_dapm_routes[] =
{
    /* */
    {"Capture", NULL, "MICL"},
    {"Capture", NULL, "MICR"},

    /* */
    { "SPKDAT1L", NULL, "Playback" },
    { "SPKDAT1R", NULL, "Playback" },
    { "SPKDAT2L", NULL, "Playback" },
    { "SPKDAT2R", NULL, "Playback" },
    { "SPKDAT3L", NULL, "Playback" },
    { "SPKDAT3R", NULL, "Playback" },
    { "SPKDAT4L", NULL, "Playback" },
    { "SPKDAT4R", NULL, "Playback" },
};

/*

 Channel mixer

 Writing a single coefficient to RAM
1. write top 2-bits of address to I2C register 0x3B
2. write bottom 8-bits of address to I2C register 0x3C

3. write top 8-bits of coefficient in I2C address 0x3D
4. write middle 8-bits of coefficient in I2C address 0x3E
5. write bottom 8-bits of coefficient in I2C address 0x3F

6. write 1 to W1 bit in I2C address 0x4C

Reading a set of coefficients from RAM
1. write top 2-bits of address to I2C register 0x3B
2. write bottom 8-bits of address to I2C register 0x3C
*
3. read top 8-bits of coefficient in I2C address 0x3D
4. read middle 8-bits of coefficient in I2C address 0x3E
5. read bottom 8-bits of coefficient in I2C address 0x3F

0x1A0 Channel 1 - mix#1 1 C1MX11 0x7FFFFF
0x1A1 Channel 1 - mix#1 2 C1MX12 0x000000
0x1A2 Channel 1 - mix#1 3 C1MX13 0x000000
0x1A3 Channel 1 - mix#1 4 C1MX14 0x000000
0x1A4 Channel 1 - mix#1 5 C1MX15 0x000000
0x1A5 Channel 1 - mix#1 6 C1MX16 0x000000
0x1A6 Channel 1 - mix#1 7 C1MX17 0x000000
0x1A7 Channel 1 - mix#1 8 C1MX18 0x000000

0x1A8 Channel 2 - mix#1 1 C2MX11 0x000000
0x1A9 Channel 2 - mix#1 2 C2MX12 0x7FFFFF
0x1AA Channel 2 - mix#1 3 C2MX13 0x000000
0x1AB Channel 2 - mix#1 4 C2MX14 0x000000
0x1AC Channel 2 - mix#1 5 C2MX15 0x000000
0x1AD Channel 2 - mix#1 6 C2MX16 0x000000
0x1AE Channel 2 - mix#1 7 C2MX17 0x000000
0x1AF Channel 2 - mix#1 8 C2MX18 0x000000

0x1E0 Channel 1 - mix#2 1 C1MX21 0x7FFFFF
0x1E1 Channel 1 - mix#2 2 C1MX22 0x000000
0x1E2 Channel 1 - mix#2 3 C1MX23 0x000000
0x1E3 Channel 1 - mix#2 4 C1MX24 0x000000
0x1E4 Channel 1 - mix#2 5 C1MX25 0x000000
0x1E5 Channel 1 - mix#2 6 C1MX26 0x000000
0x1E6 Channel 1 - mix#2 7 C1MX27 0x000000
0x1E7 Channel 1 - mix#2 8 C1MX28 0x000000

0x1E8 Channel 2 - mix#2 1 C2MX21 0x000000
0x1E9 Channel 2 - mix#2 2 C2MX22 0x7FFFFF

0x21F Channel 8 - mix#2 8 C8MX28 0x7FFFFF
*/

/* */
static const char * const mixer_route_sel[] = {
    "None", "Radio", "DVD"
};

static SOC_ENUM_SINGLE_EXT_DECL(mixer_route_enum, mixer_route_sel);

static int mixer_route_get(struct snd_kcontrol * kcontrol,
             struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
    struct sta309a_data *adsp = snd_soc_component_get_drvdata(component);

    ucontrol->value.enumerated.item[0] = adsp->route;
    return 0;
}

static int mixer_route_put(struct snd_kcontrol *kcontrol,
            struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
    struct sta309a_data *adsp = snd_soc_component_get_drvdata(component);
    struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
    unsigned int *item = ucontrol->value.enumerated.item;
    uint16_t coef_addr = 0;
    uint32_t coef_value = 0;
    int ret;

    if (item[0] >= e->items)
        return -EINVAL;

    switch(item[0]) {
        case e_mix_route_none:
        {
            /* Clear radio (ch56) mixer coeficients */
            coef_addr = 0x1A4;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);
            coef_addr = 0x1AD;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);

            /* Clear DVD (ch78) mixer coeficients */
            coef_addr = 0x1A6;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);
            coef_addr = 0x1AF;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);
            break;
        }
        case e_mix_route_radio:
        {
            /* Clear DVD (ch78) mixer coeficients */
            coef_value = 0;
            coef_addr = 0x1A6;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);
            coef_addr = 0x1AF;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);

            /* Write radio (ch56) mixer coeficients */
            coef_value = 0x7fffff;
            coef_addr = 0x1A4;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);
            coef_addr = 0x1AD;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);
            break;
        }
        case e_mix_route_dvd:
        {
            /* Clear radio (ch56) mixer coeficients */
            coef_value = 0;
            coef_addr = 0x1A4;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);
            coef_addr = 0x1AD;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);

            /* Write DVD (ch78) mixer coeficients */
            coef_value = 0x7fffff;
            coef_addr = 0x1A6;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);
            coef_addr = 0x1AF;
            ret = sta309a_coef_write(adsp, coef_addr, coef_value);
            break;
        }
        default:
            return -EINVAL;
    }

    if(ret)
        return ret;

    adsp->route = item[0];
    return 0;
}

#ifdef DEBUG_COEFICIENTS
static int mixer_dump_put(struct snd_kcontrol *kcontrol,
            struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
    struct sta309a_data *adsp = snd_soc_component_get_drvdata(component);
    uint16_t coef_addr = 0x1A0;
    uint32_t coef_value = 0;
    int chan_dst;

    for( chan_dst = 1; coef_addr < 0x1E0; coef_addr++, chan_dst++)
    {
        sta309a_coef_read(adsp, coef_addr, &coef_value);
        pr_info("%d: addr[0x%03x] coef[0x%06x]\n",
            chan_dst, coef_addr, coef_value);
        chan_dst %= 8;
    }

    return 0;
}
#endif

/* 0 dB <-> -79.5 dB, step 0.5 */
static DECLARE_TLV_DB_SCALE(master_vol_tlv, -7950, 50, 0);
/* +48 dB <-> -79.5 dB, step 1 */
static DECLARE_TLV_DB_SCALE(channel_vol_tlv, -7950, 100, 0);
/* Bass/Treble: +12dB <-> -12 dB, step 1.5 */
static DECLARE_TLV_DB_SCALE(bt_level_tlv, -12000, 150, 0);
/* EQ band: +16dB <-> -15 dB, step 1 */
static DECLARE_TLV_DB_SCALE(eq_level_tlv, -15000, 100, 0);

/* Preset EQ settings */
static const char *eq_presets_sel[] = {
    "Flat", "Rock", "Soft Rock", "Jazz", "Classical", "Dance",
    "Pop", "Soft", "Hard", "Party", "Vocal", "Hip-Hop", "Dialog",
    "Bass-boost-1", "Bass-boost-2", "Bass-boost-3"
};

static SOC_ENUM_SINGLE_DECL(sta309a_eq_presets,
    STA309A_REG_PREEQ, 0, eq_presets_sel);

/* */
static const char *automode_sel[] = {
    "User", "Preset", "Graphics"
};
static SOC_ENUM_SINGLE_DECL(sta309a_auto_modes,
    STA309A_REG_AUTO1, 0, automode_sel);

/* Input/Output channels mapping selection */
static const char *channel_map_sel[] = {
    "1", "2", "3", "4", "5", "6", "7", "8"
};

static SOC_ENUM_DOUBLE_DECL(sta309a_input_ch12_map,
    STA309A_REG_CH1_CH2_IM, 0, 4, channel_map_sel);
static SOC_ENUM_DOUBLE_DECL(sta309a_input_ch34_map,
    STA309A_REG_CH3_CH4_IM, 0, 4, channel_map_sel);
static SOC_ENUM_DOUBLE_DECL(sta309a_input_ch56_map,
    STA309A_REG_CH5_CH6_IM, 0, 4, channel_map_sel);
static SOC_ENUM_DOUBLE_DECL(sta309a_input_ch78_map,
    STA309A_REG_CH7_CH8_IM, 0, 4, channel_map_sel);

static SOC_ENUM_DOUBLE_DECL(sta309a_output_ch12_map,
    STA309A_REG_CH1_CH2_OM, 0, 4, channel_map_sel);
static SOC_ENUM_DOUBLE_DECL(sta309a_output_ch34_map,
    STA309A_REG_CH3_CH4_OM, 0, 4, channel_map_sel);
static SOC_ENUM_DOUBLE_DECL(sta309a_output_ch56_map,
    STA309A_REG_CH5_CH6_OM, 0, 4, channel_map_sel);
static SOC_ENUM_DOUBLE_DECL(sta309a_output_ch78_map,
    STA309A_REG_CH7_CH8_OM, 0, 4, channel_map_sel);

/* */
static const struct snd_kcontrol_new sta309a_snd_controls[] =
{
    SOC_SINGLE("STA309 DSP Bypass",
        STA309A_REG_CONFA, 5, 1, 0),

    SOC_ENUM("STA309 Automode EQ", sta309a_auto_modes),

    SOC_DOUBLE("STA309 EQ Bypass Ch12",
        STA309A_REG_EQBP, 0, 1, 1, 0),
    SOC_DOUBLE("STA309 EQ Bypass Ch34",
        STA309A_REG_EQBP, 2, 3, 1, 0),
    SOC_DOUBLE("STA309 EQ Bypass Ch56",
        STA309A_REG_EQBP, 4, 5, 1, 0),
    SOC_DOUBLE("STA309 EQ Bypass Ch78",
        STA309A_REG_EQBP, 6, 7, 1, 0),

    SOC_ENUM("STA309 EQ Preset", sta309a_eq_presets),

    SOC_SINGLE_TLV("STA309 Graphic EQ 80Hz",
        STA309A_REG_AGEQ, 0, 0x1f, 0, eq_level_tlv),
    SOC_SINGLE_TLV("STA309 Graphic EQ 300Hz",
        STA309A_REG_BGEQ, 0, 0x1f, 0, eq_level_tlv),
    SOC_SINGLE_TLV("STA309 Graphic EQ 1KHz",
        STA309A_REG_CGEQ, 0, 0x1f, 0, eq_level_tlv),
    SOC_SINGLE_TLV("STA309 Graphic EQ 3KHz",
        STA309A_REG_DGEQ, 0, 0x1f, 0, eq_level_tlv),
    SOC_SINGLE_TLV("STA309 Graphic EQ 8KHz",
        STA309A_REG_EGEQ, 0, 0x1f, 0, eq_level_tlv),

    SOC_DOUBLE("STA309 Tone Bypass Ch12",
        STA309A_REG_TONEBP, 0, 1, 1, 0),
    SOC_DOUBLE("STA309 Tone Bypass Ch34",
        STA309A_REG_TONEBP, 2, 3, 1, 0),
    SOC_DOUBLE("STA309 Tone Bypass Ch56",
        STA309A_REG_TONEBP, 4, 5, 1, 0),
    SOC_DOUBLE("STA309 Tone Bypass Ch78",
        STA309A_REG_TONEBP, 6, 7, 1, 0),

    SOC_SINGLE_TLV("STA309 Tone Bass",
        STA309A_REG_TONE, 0, 0xf, 0, bt_level_tlv),
    SOC_SINGLE_TLV("STA309 Tone Treble",
        STA309A_REG_TONE, 4, 0xf, 0, bt_level_tlv),

    SOC_SINGLE("STA309 Master Mute",
        STA309A_REG_MASTER_MUTE, 0, 1, 0),

    SOC_SINGLE_TLV("STA309 Master Volume",
        STA309A_REG_MASTER_VOL, 0, 0xb0, 1, master_vol_tlv),

    SOC_DOUBLE_R_TLV("STA309 Ch12 Volume",
        STA309A_REG_CH_1_VOL, STA309A_REG_CH_2_VOL,
        0, STA309A_CHXVOL_MASK, 1, channel_vol_tlv),
    SOC_DOUBLE_R_TLV("STA309 Ch34 Volume",
        STA309A_REG_CH_3_VOL, STA309A_REG_CH_4_VOL,
        0, STA309A_CHXVOL_MASK, 1, channel_vol_tlv),
    SOC_DOUBLE_R_TLV("STA309 Ch56 Volume",
        STA309A_REG_CH_5_VOL, STA309A_REG_CH_6_VOL,
        0, STA309A_CHXVOL_MASK, 1, channel_vol_tlv),
    SOC_DOUBLE_R_TLV("STA309 Ch78 Volume",
        STA309A_REG_CH_7_VOL, STA309A_REG_CH_8_VOL,
        0, STA309A_CHXVOL_MASK, 1, channel_vol_tlv),

    SOC_ENUM("STA309 Ch12 Serial In From", sta309a_input_ch12_map),
    SOC_ENUM("STA309 Ch34 Serial In From", sta309a_input_ch34_map),
    SOC_ENUM("STA309 Ch56 Serial In From", sta309a_input_ch56_map),
    SOC_ENUM("STA309 Ch78 Serial In From", sta309a_input_ch78_map),

    SOC_ENUM("STA309 Ch12 Serial Out From", sta309a_output_ch12_map),
    SOC_ENUM("STA309 Ch34 Serial Out From", sta309a_output_ch34_map),
    SOC_ENUM("STA309 Ch56 Serial Out From", sta309a_output_ch56_map),
    SOC_ENUM("STA309 Ch78 Serial Out From", sta309a_output_ch78_map),

    SOC_ENUM_EXT("STA309 Mix Route",
        mixer_route_enum, mixer_route_get, mixer_route_put),

#ifdef DEBUG_COEFICIENTS
    SOC_ENUM_EXT("STA309 Dump mix route",
        mixer_route_enum, mixer_route_get, mixer_dump_put),
#endif
};

/* ------------------------------------------------------------------ */
static struct snd_soc_component_driver soc_codec_dev_sta309a = {
    .probe                  = sta309a_component_probe,
    .set_bias_level         = sta309a_set_bias_level,
    .controls               = sta309a_snd_controls,
    .num_controls           = ARRAY_SIZE(sta309a_snd_controls),
    .dapm_widgets           = sta309a_dapm_widgets,
    .num_dapm_widgets       = ARRAY_SIZE(sta309a_dapm_widgets),
    .dapm_routes            = sta309a_dapm_routes,
    .num_dapm_routes        = ARRAY_SIZE(sta309a_dapm_routes),
    .endianness             = 1,
    .non_legacy_dai_naming  = 1,
    .use_pmdown_time        = 1,
    .read                   = sta309a_component_read,
    .write                  = sta309a_component_write
};

static const struct snd_soc_dai_ops sta309a_dai_ops = {
    .hw_params  = sta309a_hw_params,
    .set_fmt    = sta309a_set_dai_fmt,
};

static struct snd_soc_dai_driver sta309a_dai[] = {
    {
        .name = "HeadUnit DAI",
        .playback = {
            .stream_name = "STA309A Playback",
            .channels_min = 2,
            .channels_max = 4,
            .rates = STA309A_RATES,
            .formats = STA309A_FORMATS,
        },
        .capture = { /* PCM1808 with external MIC */
            .stream_name = "PCM1808 Capture",
            .channels_min = 2,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_96000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
        .ops = &sta309a_dai_ops,
    },
};

/* ------------------------------------------------------------------ */
static int sta309a_i2c_probe(struct i2c_client *client,
                 const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct sta309a_data *adsp;
    int ret;

    dev_info(dev, "Probing..\n");

    adsp = devm_kzalloc(dev, sizeof(*adsp), GFP_KERNEL);
    if (!adsp)
        return -ENOMEM;

    adsp->client = client;
    adsp->route = e_mix_route_none;

    mutex_init(&adsp->lock);
    dev_set_drvdata(dev, adsp);

    adsp->mclk = devm_clk_get(dev, "mclk");
    if (IS_ERR(adsp->mclk)) {
        dev_err(dev, "Can't retrieve i2s master clock\n");
    }

    adsp->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
                              GPIOD_OUT_LOW);
    if (IS_ERR(adsp->powerdown_gpio)) {
        if (PTR_ERR(adsp->powerdown_gpio) == -EPROBE_DEFER)
            return -EPROBE_DEFER;
        dev_info(dev, "failed to get powerdown GPIO: %ld\n",
            PTR_ERR(adsp->powerdown_gpio));
        adsp->powerdown_gpio = NULL;
    }

    adsp->reset_gpio = devm_gpiod_get_optional(dev, "reset",
                              GPIOD_OUT_LOW);
    if (IS_ERR(adsp->reset_gpio)) {
        if (PTR_ERR(adsp->reset_gpio) == -EPROBE_DEFER)
            return -EPROBE_DEFER;
        dev_info(dev, "failed to get reset GPIO: %ld\n",
            PTR_ERR(adsp->reset_gpio));
        adsp->reset_gpio = NULL;
    } else {
        /* Deassert reset pin and load defaults */
        msleep(10);
        gpiod_set_value_cansleep(adsp->reset_gpio, 0);
        msleep(10);
    }

    /* I2S, MSB first, 64*fs */
    ret = sta309a_reg8_write(adsp, STA309A_REG_CONFC, 0x00);
    if (ret) {
        dev_err(dev, "STA309A_REG_CONFC: write failed, ret %d\n", ret);
        return ret;
    }

    /* Config output: I2S, no CLKOUT, no PWM, AM reduction */
    ret = sta309a_reg8_write(adsp, STA309A_REG_CONFG,
                STA309A_CONFG_PWM_DIS | STA309A_CONFG_CLKO_DIS |
                STA309A_CONFG_AM_ENA | STA309A_CONFG_AM2_ENA);
    if (ret) {
        dev_err(dev, "STA309A_REG_CONFG: write failed, ret %d\n", ret);
        return ret;
    }

    /* W/A: */
    ret = sta309a_reg8_write(adsp, STA309A_REG_CONFH,
                STA309A_CONFH_ZC_ENA | STA309A_CONFH_SV_ENA |
                STA309A_CONFH_ZD_ENA | STA309A_CONFH_LDT_ENA |
                STA309A_CONFH_ECL_ENA);
    if (ret) {
        dev_err(dev, "STA309A_REG_CONFH: write failed, ret %d\n", ret);
        return ret;
    }

    ret = sta309a_reg8_write(adsp, STA309A_REG_MASTER_VOL, 120);
    if (ret) {
        dev_err(dev, "STA309A_REG_MASTER_VOL: write failed, ret %d\n", ret);
        return ret;
    }

    /* Map all amplifier 4 channes to first 2chs of I2S port by default */
    ret = sta309a_reg8_write(adsp, STA309A_REG_CH1_CH2_IM,
                STA309A_IM_CH1_TO(STA309A_IN_CH_1) |
                STA309A_IM_CH2_TO(STA309A_IN_CH_2));
    if (ret) {
        dev_err(dev, "STA309A_REG_CH1_CH2_IM: write failed, ret %d\n", ret);
        return ret;
    }

    ret = sta309a_reg8_write(adsp, STA309A_REG_CH3_CH4_IM,
                STA309A_IM_CH3_TO(STA309A_IN_CH_1) |
                STA309A_IM_CH4_TO(STA309A_IN_CH_2));
    if (ret) {
        dev_err(dev, "STA309A_REG_CH3_CH4_IM: write failed, ret %d\n", ret);
        return ret;
    }

    /* -------------------------------------------------------------- */
    ret = devm_snd_soc_register_component(dev, &soc_codec_dev_sta309a,
                     sta309a_dai, ARRAY_SIZE(sta309a_dai));
    if (ret < 0) {
        dev_err(dev, "unable to register codec component, ret %d\n", ret);
        return ret;
    }

    return 0;
}

static int sta309a_i2c_remove(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct sta309a_data *sta309a = dev_get_drvdata(dev);

    /* put the codec in stand-by */
    if (sta309a->powerdown_gpio)
        gpiod_set_value_cansleep(sta309a->powerdown_gpio, 0);

    return 0;
}

static const struct of_device_id sta309a_of_ids[] = {
    { .compatible = "x3399-headunit,sta309a", },
    { },
};
MODULE_DEVICE_TABLE(of, sta309a_of_ids);

static struct i2c_driver sta309a_i2c_driver = {
    .driver = {
        .name = "sta309a",
        .of_match_table = of_match_ptr(sta309a_of_ids),
    },
    .probe = sta309a_i2c_probe,
    .remove = sta309a_i2c_remove,
};
module_i2c_driver(sta309a_i2c_driver);

MODULE_AUTHOR("Oleksii Gulchenko <alexey.gulchenko@gmail.com>");
MODULE_DESCRIPTION("x3399-HeadUnit STA309A Audio DSP driver");
MODULE_LICENSE("GPL v2");
