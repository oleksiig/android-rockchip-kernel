/*
 * Oleksii Gulchenko <alexey.gulchenko@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>

#include "ti-sn65dsi83.h"

struct sn65dsi83 {
    struct i2c_client *i2c;
    struct gpio_desc *enable_gpio;
    struct display_timings *disp;
    int dsi_lanes;
    int dsi_format;
};

static int sn65dsi83_config(struct sn65dsi83 *ctx)
{
    struct display_timing* dt = display_timings_get(ctx->disp, 0);
    u32 pixelclock = (dt->pixelclock.typ / 1000); /* KHz */
    u8 val;

    dev_info(&ctx->i2c->dev, "dsi_format=%d, dsi_lanes=%d, pixel_clock=%dKHz\n",
        ctx->dsi_format, ctx->dsi_lanes, pixelclock);

    /* Soft reset and disable PLL */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_SOFT_RESET, 0x01);
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_PLL_EN, 0x00);

    /* Select clock mode */
    /* LVDS pixel clock derived from MIPI D-PHY channel A HS continuous clock */
    val = 0x1;

    if (pixelclock <= 37500) {
        /* nothing */
    } else if (pixelclock <= 62500) {
        val |= (0x01 << 1);
    } else if (pixelclock <= 87500) {
        val |= (0x02 << 1);
    } else if (pixelclock <= 112500) {
        val |= (0x03 << 1);
    } else if (pixelclock <= 137500) {
        val |= (0x04 << 1);
    } else {
        val |= (0x05 << 1);
    }

    /* 0x0A */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CORE_PLL, val);

    /* 0x0B */
    /* REFCLK_MULTIPLIER = 0 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_PLL_DIV, 0x18);  // Divide DSI_CLK by 4.
//  i2c_smbus_write_byte_data(ctx->i2c, DSI83_PLL_DIV, 0x00);  // Multiply REFCLK by 1.

    /* 4xDSI lanes with single channel */
    /* 0x10 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_DSI_CFG, 0x26);
    /* 0x11 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_DSI_EQ, 0x00);

    /* set DSI clock range*/
    /* TODO: rework hardcode
     * DSI freq = 240, freq STEP = 5
     * */
/*  tmp = 100;
    while(tmp > 0) {
        if ((tmp * 5) == 240) {
            break;
        }
        tmp--;
    }
    pr_info("reg 0x12: 0x%x\n", tmp);
*/
    /* 0x12 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_DSI_CLK_RNG, 0x30);//(pixelclock * 3 / 5000));
    /* 0x13 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHB_DSI_CLK_RNG, 0);

    /* set LVDS for single channel, 24 bit mode, HS/VS low, DE high */
/*
    DISPLAY_FLAGS_HSYNC_LOW     = BIT(0),
    DISPLAY_FLAGS_HSYNC_HIGH    = BIT(1),
    DISPLAY_FLAGS_VSYNC_LOW     = BIT(2),
    DISPLAY_FLAGS_VSYNC_HIGH    = BIT(3),

    DISPLAY_FLAGS_DE_LOW        = BIT(4),
    DISPLAY_FLAGS_DE_HIGH       = BIT(5),
*/
    /* 0x18 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_LVDS_MODE, 0x78);

    /* set LVDS 200 Ohm termination and max differential swing voltage */
    /* 0x19 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_LVDS_SIGN, 0x04);
    /* 0x1A */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_LVDS_TERM, 0x00);

    /* x resolution high/low for channel A */
    /* 0x20 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_LINE_LEN_LO, (dt->hactive.typ & 0x00FF));
    /* 0x21 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_LINE_LEN_HI, (dt->hactive.typ & 0xFF00) >> 8);

    /* SYNC delay high/low for channel A */
    /* 0x28 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_SYNC_DELAY_LO, 0x00);
    /* 0x29 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_SYNC_DELAY_HI, 0x02);

    /* HSYNC width high/low for channel A */
    /* 0x2C */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_HSYNC_WIDTH_LO, (dt->hsync_len.typ & 0x00FF));
    /* 0x2D */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_HSYNC_WIDTH_HI, (dt->hsync_len.typ & 0xFF00) >> 8);

    /* VSYNC width high/low for channel A */
    /* 0x30 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_VSYNC_WIDTH_LO, (dt->vsync_len.typ & 0x00FF));
    /* 0x31 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_VSYNC_WIDTH_HI, (dt->vsync_len.typ & 0xFF00) >> 8);

    /* Horizontal BackPorch for channel A */
    /* 0x34 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_HORZ_BACKPORCH, (dt->hback_porch.typ & 0x00FF));

    /* */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_IRQ_EN, 0x01);

    /*
     * TEST PATTERN GENERATION PURPOSE ONLY
     */
#if 0
    /* y resolution high/low for channel A */
    /* 0x24 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_VERT_LINES_LO, (dt->vactive.typ & 0x00FF));
    /* 0x25 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_VERT_LINES_HI, (dt->vactive.typ & 0xFF00) >> 8);

    /* Vertical BackPorch for channel A */
    /* 0x36 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_VERT_BACKPORCH, (dt->vback_porch.typ & 0x00FF));

    /* Horizontal FrontPorch for channel A */
    /* 0x38 */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_HORZ_FRONTPORCH, (dt->hfront_porch.typ & 0x00FF));

    /* Vertical FrontPorch for channel A */
    /* 0x3A */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_VERT_FRONTPORCH, (dt->vfront_porch.typ & 0x00FF));

    /* 0x3C */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_CHA_TEST_PATTERN, 0x11);
    /* -------------------------------------------------------------- */
#endif

    /* Soft reset and enable PLL */
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_SOFT_RESET, 0x01);
    i2c_smbus_write_byte_data(ctx->i2c, DSI83_PLL_EN, 0x01);

    return 0;
}

/* ------------------------------------------------------------------ */
static int sn65dsi83_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    int ret;
    struct sn65dsi83 *ctx;
    struct device *dev = &i2c->dev;
    struct device_node *dsi_np, *panel_np;
    struct display_timings *disp;
    u32 val;

    dsi_np = of_parse_phandle(dev->of_node, "dsi-host", 0);
    if (!dsi_np) {
        dev_err(dev, "failed to find MIPI-DSI host node\n");
        return -1;
    }

    panel_np = of_get_child_by_name(dsi_np, "panel");
    if (!panel_np) {
        dev_err(dev, "failed to find panel node\n");
        return -1;
    }

    disp = of_get_display_timings(panel_np);
    if (!disp || !disp->num_timings) {
        dev_err(dev, "failed to get display-timings\n");
        return -1;
    }

    ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;

    ctx->i2c = i2c;
    ctx->disp = disp;

    if (!of_property_read_u32(panel_np, "dsi,lanes", &val))
        ctx->dsi_lanes = val;

    if (!of_property_read_u32(panel_np, "dsi,format", &val))
        ctx->dsi_format = val;

    /* Get SN65DSI83 EN pin control GPIO */
    ctx->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
    if (IS_ERR(ctx->enable_gpio)) {
        ret = PTR_ERR(ctx->enable_gpio);
        dev_err(dev, "failed to request 'enable' GPIO: %d\n", ret);
        goto err_free_ctx;
    }

    /* Power On and initialize */
    gpiod_set_value(ctx->enable_gpio, 1);
    msleep(5);

    /* Try to access SN65DSI83 on I2C */
    ret = i2c_smbus_write_byte_data(i2c, DSI83_SOFT_RESET, 0x01);
    if (ret < 0) {
        ret = -EPROBE_DEFER;
        dev_err(dev, "probe deffered\n");
        goto err_init;
    }

    ret = sn65dsi83_config(ctx);
    if (ret) {
        dev_err(dev, "sn65dsi83 config failed: %d\n", ret);
        goto err_init;
    }

    i2c_set_clientdata(i2c, ctx);

    dev_info(dev, "%s probed\n", __func__);
    return 0;

err_init:
    gpiod_set_value(ctx->enable_gpio, 0);
    devm_gpiod_put(dev, ctx->enable_gpio);

err_free_ctx:
    display_timings_release(ctx->disp);
    devm_kfree(dev, ctx);
    return ret;
}

static int sn65dsi83_remove(struct i2c_client *i2c)
{
    struct sn65dsi83 *ctx = i2c_get_clientdata(i2c);

    gpiod_set_value(ctx->enable_gpio, 0);
    devm_gpiod_put(&i2c->dev, ctx->enable_gpio);
    display_timings_release(ctx->disp);
    devm_kfree(&i2c->dev, ctx);
    return 0;
}

static const struct of_device_id sn65dsi83_of_match[] = {
    { .compatible = "ti,sn65dsi83" },
    { }
};
MODULE_DEVICE_TABLE(of, sn65dsi83_of_match);

static struct i2c_driver sn65dsi83_driver = {
    .driver = {
        .name = "sn65dsi83",
        .of_match_table = sn65dsi83_of_match,
    },
    .probe = sn65dsi83_probe,
    .remove = sn65dsi83_remove,
};

static int __init sn65dsi83_init(void)
{
    return i2c_add_driver(&sn65dsi83_driver);
}
module_init(sn65dsi83_init);

static void __exit sn65dsi83_exit(void)
{
    i2c_del_driver(&sn65dsi83_driver);
}
module_exit(sn65dsi83_exit);

MODULE_AUTHOR("Oleksii Gulchenko <alexey.gulchenko@gmail.com>");
MODULE_DESCRIPTION("TI SN65DSI83 MIPI® DSI to FlatLink™ LVDS bridge driver");
MODULE_LICENSE("GPL v2");
