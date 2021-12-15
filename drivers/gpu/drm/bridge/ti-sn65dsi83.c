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
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>

#include "ti-sn65dsi83.h"

struct sn65dsi83_i2c {
    struct device               *dev;
    struct regmap               *regmap;
    struct gpio_desc            *enable_gpio;
};

struct sn65dsi83_dsi {
    struct device *dev;
    struct mipi_dsi_device *dsi;

    struct drm_bridge bridge;
    struct drm_connector connector;
    struct drm_panel *panel;

    struct sn65dsi83_i2c *i2c_ctx;
};

/* -------------------------------------------------------------------------- */
static const struct regmap_range sn65dsi83_volatile_ranges[] = {
    { .range_min = 0, .range_max = 0xFF },
};

static const struct regmap_access_table sn65dsi83_volatile_table = {
    .yes_ranges = sn65dsi83_volatile_ranges,
    .n_yes_ranges = ARRAY_SIZE(sn65dsi83_volatile_ranges),
};

static const struct regmap_config sn65dsi83_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .volatile_table = &sn65dsi83_volatile_table,
    .cache_type = REGCACHE_NONE,
};

static void sn65dsi83_write_u16(struct sn65dsi83_i2c *ctx,
                   unsigned int reg, u16 val)
{
    regmap_write(ctx->regmap, reg, val & 0xFF);
    regmap_write(ctx->regmap, reg + 1, val >> 8);
}

/* -------------------------------------------------------------------------- */
static int sn65dsi83_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{
    struct sn65dsi83_i2c *ctx;
    int ret;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        DRM_ERROR("device doesn't support I2C\n");
        return -ENODEV;
    }

    ctx = devm_kzalloc(&client->dev, sizeof(struct sn65dsi83_i2c), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;

    ctx->dev = &client->dev;

    ctx->regmap = devm_regmap_init_i2c(client, &sn65dsi83_regmap_config);
    if (IS_ERR(ctx->regmap)) {
        DRM_ERROR("regmap i2c init failed\n");
        return PTR_ERR(ctx->regmap);
    }

    ctx->enable_gpio = devm_gpiod_get(ctx->dev, "enable", GPIOD_OUT_HIGH);
    if (IS_ERR(ctx->enable_gpio)) {
        DRM_ERROR("failed to get enable gpio from DT\n");
        ret = PTR_ERR(ctx->enable_gpio);
        return ret;
    }

    pm_runtime_enable(ctx->dev);

    i2c_set_clientdata(client, ctx);

    return 0;
}

static int sn65dsi83_remove(struct i2c_client *client)
{
    struct sn65dsi83_i2c *ctx = i2c_get_clientdata(client);

    if (!ctx)
        return -EINVAL;

    pm_runtime_disable(ctx->dev);

    return 0;
}

/* -------------------------------------------------------------------------- */
static int sn65dsi83_bridge_connector_get_modes(struct drm_connector *connector)
{
    struct sn65dsi83_dsi *ctx = container_of(connector, struct sn65dsi83_dsi, connector);

    return drm_panel_get_modes(ctx->panel);
}

static enum drm_mode_status sn65dsi83_bridge_connector_mode_valid(
    struct drm_connector *connector, struct drm_display_mode *mode)
{
    /* Min 25 MHz Max 154 MHz */
    if (mode->clock < 25000)
        return MODE_CLOCK_LOW;

    if (mode->clock > 154000)
        return MODE_CLOCK_HIGH;

    return MODE_OK;
}

static struct drm_connector_helper_funcs sn65dsi83_bridge_connector_helper_funcs = {
    .get_modes                  = sn65dsi83_bridge_connector_get_modes,
    .mode_valid                 = sn65dsi83_bridge_connector_mode_valid,
};

static const struct drm_connector_funcs sn65dsi83_bridge_connector_funcs = {
    .fill_modes                 = drm_helper_probe_single_connector_modes,
    .destroy                    = drm_connector_cleanup,
    .reset                      = drm_atomic_helper_connector_reset,
    .atomic_duplicate_state     = drm_atomic_helper_connector_duplicate_state,
    .atomic_destroy_state       = drm_atomic_helper_connector_destroy_state,
};

static int sn65dsi83_bridge_attach(struct drm_bridge *bridge)
{
    struct sn65dsi83_dsi *ctx = container_of(bridge, struct sn65dsi83_dsi, bridge);
    int ret;

    ret = drm_connector_init(bridge->dev, &ctx->connector,
                 &sn65dsi83_bridge_connector_funcs,
                 DRM_MODE_CONNECTOR_LVDS);

    if (ret) {
        DRM_ERROR("Failed to initialize DRM connector\n");
        return ret;
    }

    drm_connector_helper_add(&ctx->connector,
                 &sn65dsi83_bridge_connector_helper_funcs);

    drm_connector_attach_encoder(&ctx->connector, bridge->encoder);

    drm_panel_attach(ctx->panel, &ctx->connector);

    ctx->connector.funcs->reset(&ctx->connector);
    drm_connector_register(&ctx->connector);

    return 0;
}

static void sn65dsi83_bridge_detach(struct drm_bridge *bridge)
{
    struct sn65dsi83_dsi *ctx = container_of(bridge, struct sn65dsi83_dsi, bridge);

    drm_connector_unregister(&ctx->connector);
    drm_panel_detach(ctx->panel);
    ctx->panel = NULL;
    drm_connector_put(&ctx->connector);
}

static void sn65dsi83_bridge_pre_enable(struct drm_bridge *bridge)
{
    struct sn65dsi83_dsi *ctx = container_of(bridge, struct sn65dsi83_dsi, bridge);
    int ret;

    dev_info(ctx->dev, "pre_enable()\n");

    pm_runtime_get_sync(ctx->dev);

    ret = drm_panel_prepare(ctx->panel);
    if (ret < 0)
        dev_err(ctx->dev, "panel prepare failed (err %d)\n", ret);
}

static void sn65dsi83_bridge_enable(struct drm_bridge *bridge)
{
    struct sn65dsi83_dsi *ctx = container_of(bridge, struct sn65dsi83_dsi, bridge);
    struct sn65dsi83_i2c *i2c_ctx = ctx->i2c_ctx;
    struct drm_display_mode *mode = &ctx->bridge.encoder->crtc->state->adjusted_mode;
    int ret;
    uint8_t val;

    dev_info(ctx->dev, "enable(): dsi_format=%d, dsi_lanes=%d, lvds_clock=%dKHz\n",
        ctx->dsi->format, ctx->dsi->lanes, mode->clock);

    /* Soft reset and disable PLL */
    regmap_write(i2c_ctx->regmap, DSI83_SOFT_RESET, 0x01);
    regmap_write(i2c_ctx->regmap, DSI83_PLL_EN, 0x00);

    /* Select clock mode */
    /* LVDS pixel clock derived from MIPI D-PHY channel A HS continuous clock */
    val = 0x1;

    if (mode->clock <= 37500) {
        /* nothing */
    } else if (mode->clock <= 62500) {
        val |= (0x01 << 1);
    } else if (mode->clock <= 87500) {
        val |= (0x02 << 1);
    } else if (mode->clock <= 112500) {
        val |= (0x03 << 1);
    } else if (mode->clock <= 137500) {
        val |= (0x04 << 1);
    } else {
        val |= (0x05 << 1);
    }

    /* 0x0A */
    regmap_write(i2c_ctx->regmap, DSI83_CORE_PLL, val);

    /* 0x0B */
    /* REFCLK_MULTIPLIER = 0 */
    regmap_write(i2c_ctx->regmap, DSI83_PLL_DIV, 0x18); // Divide DSI_CLK by 4.
//  regmap_write(i2c_ctx->regmap, DSI83_PLL_DIV, 0x00);  // Multiply REFCLK by 1.

    /* 0x10 */
    /* 0 – Single bit errors are tolerated for the start of transaction SoT leader sequence (default): 0 */
    /* 2-1 Reserved  : 3 */
    /* 3-4 DSI lanes : 4*/
    val = 0x26;
    if(ctx->dsi->lanes == 4) {
        val = 0x26 | (0x0 << 3);
    } else if(ctx->dsi->lanes == 3) {
        val = 0x26 | (0x1 << 3);
    } else if(ctx->dsi->lanes == 2) {
        val = 0x26 | (0x2 << 3);
    } else if(ctx->dsi->lanes == 1) {
        val = 0x26 | (0x3 << 3);
    } else {
        dev_err(ctx->dev, "enable(): unsupported dsi_lanes:%d, using defaults 4\n",
            ctx->dsi->lanes);
    }

    regmap_write(i2c_ctx->regmap, DSI83_DSI_CFG, val);

    /* 0x11 */
    regmap_write(i2c_ctx->regmap, DSI83_DSI_EQ, 0x00);

    /* set DSI clock range*/
    /* TODO: rework hardcode
     * DSI freq = 240, freq STEP = 5
     * */
/*  tmp = 100;
    //while(tmp > 0) {
        //if ((tmp * 5) == 240) {
            //break;
        //}
        //tmp--;
    //}
    //pr_info("reg 0x12: 0x%x\n", tmp);
*/
    /* 0x12 */
    regmap_write(i2c_ctx->regmap, DSI83_CHA_DSI_CLK_RNG, 0x30);//(pixelclock * 3 / 5000));

    /* 0x13 */
    regmap_write(i2c_ctx->regmap, DSI83_CHB_DSI_CLK_RNG, 0x00);

    /* set LVDS for single channel, 24 bit mode, HS/VS low, DE high */

    /* 0x18 */
    regmap_write(i2c_ctx->regmap, DSI83_LVDS_MODE, 0x78);

    /* set LVDS 200 Ohm termination and max differential swing voltage */
    /* 0x19 */
    regmap_write(i2c_ctx->regmap, DSI83_LVDS_SIGN, 0x04);
    /* 0x1A */
    regmap_write(i2c_ctx->regmap, DSI83_LVDS_TERM, 0x00);

    /* x resolution high/low for channel A */
    /* 0x20 */
    
    /* 0x21, 0x22 */
    sn65dsi83_write_u16(i2c_ctx, DSI83_CHA_LINE_LEN_LO, mode->hdisplay);

    /* SYNC delay high/low for channel A */
    /* 0x28, 0x29 */
    sn65dsi83_write_u16(i2c_ctx, DSI83_CHA_SYNC_DELAY_LO, 0x0200);

    /* HSYNC width high/low for channel A */
    /* 0x2C, 0x2D */
    sn65dsi83_write_u16(i2c_ctx, DSI83_CHA_HSYNC_WIDTH_LO,
                               (mode->hsync_end - mode->hsync_start) & 0xFF);

    /* VSYNC width high/low for channel A */
    /* 0x30, 0x31 */
    sn65dsi83_write_u16(i2c_ctx, DSI83_CHA_VSYNC_WIDTH_LO,
                                (mode->vsync_end - mode->vsync_start) & 0xFF);

    /* Horizontal BackPorch for channel A */
    /* 0x34 */
    regmap_write(i2c_ctx->regmap, DSI83_CHA_HORZ_BACKPORCH,
                                (mode->htotal - mode->hsync_end) & 0xFF);

    /* */
    regmap_write(i2c_ctx->regmap, DSI83_IRQ_EN, 0x01);

    /*
     * TEST PATTERN GENERATION PURPOSE ONLY
     */
#ifdef CONFIG_DRM_TI_SN65DSI83_TEST_PATTERN
    dev_warn(ctx->dev, "!!! TEST PATTERN GENERATION ENABLED !!!\n");

    /* 0x24, 0x25 */
    sn65dsi83_write_u16(i2c_ctx, DSI83_CHA_VERT_LINES_LO, mode->vdisplay);

    /* Vertical BackPorch for channel A */
    /* 0x36 */
    regmap_write(i2c_ctx->regmap, DSI83_CHA_VERT_BACKPORCH,
                                (mode->vtotal - mode->vsync_end) & 0xFF);

    /* Horizontal FrontPorch for channel A */
    /* 0x38 */
    regmap_write(i2c_ctx->regmap, DSI83_CHA_HORZ_FRONTPORCH,
                                (mode->hsync_start - mode->hdisplay) & 0xFF);

    /* Vertical FrontPorch for channel A */
    /* 0x3A */
    regmap_write(i2c_ctx->regmap, DSI83_CHA_VERT_FRONTPORCH,
                                (mode->vsync_start - mode->vdisplay) & 0xFF);

    /* 0x3C */
    regmap_write(i2c_ctx->regmap, DSI83_CHA_TEST_PATTERN, 0x11);
    /* -------------------------------------------------------------- */
#endif

    /* Soft reset and enable PLL */
    regmap_write(i2c_ctx->regmap, DSI83_PLL_EN, 0x01);
    usleep_range(3000, 5000);
    regmap_write(i2c_ctx->regmap, DSI83_SOFT_RESET, 0x01);

    ret = drm_panel_enable(ctx->panel);
    if (ret < 0)
        dev_err(ctx->dev, "panel enable failed (err %d)\n", ret);
}

static void sn65dsi83_bridge_disable(struct drm_bridge *bridge)
{
    struct sn65dsi83_dsi *ctx = container_of(bridge, struct sn65dsi83_dsi, bridge);
    struct sn65dsi83_i2c *i2c_ctx = ctx->i2c_ctx;
    int ret;

    dev_info(ctx->dev, "disable()\n");

    ret = drm_panel_disable(ctx->panel);
    if (ret < 0)
        DRM_ERROR("panel disable failed (err %d)\n", ret);

    regmap_write(i2c_ctx->regmap, DSI83_SOFT_RESET, 0x01);
    regmap_write(i2c_ctx->regmap, DSI83_PLL_EN, 0x00);
}

static void sn65dsi83_bridge_post_disable(struct drm_bridge *bridge)
{
    struct sn65dsi83_dsi *ctx = container_of(bridge, struct sn65dsi83_dsi, bridge);
    int ret;

    ret = drm_panel_unprepare(ctx->panel);
    if (ret < 0)
        DRM_ERROR("panel unprepare failed (err %d)\n", ret);

    pm_runtime_put_sync(ctx->dev);

    dev_info(ctx->dev, "post_disable()\n");
}

static const struct drm_bridge_funcs sn65dsi83_bridge_funcs = {
    .attach = sn65dsi83_bridge_attach,
    .detach = sn65dsi83_bridge_detach,
    .pre_enable = sn65dsi83_bridge_pre_enable,
    .enable = sn65dsi83_bridge_enable,
    .disable = sn65dsi83_bridge_disable,
    .post_disable = sn65dsi83_bridge_post_disable,
};

static int sn65dsi83_dsi_probe(struct mipi_dsi_device *dsi)
{
    struct sn65dsi83_dsi *ctx;
    struct i2c_client *i2c;
    struct device_node *of_node = dsi->dev.of_node;
    uint32_t val;
    int ret;

    i2c = of_find_i2c_device_by_node(of_parse_phandle(of_node, "i2c,client", 0));
    if (!i2c) {
        DRM_ERROR("failed to find I2C client device\n");
        return -ENODEV;
    }

    ctx = devm_kzalloc(&dsi->dev, sizeof(struct sn65dsi83_dsi), GFP_KERNEL);

    mipi_dsi_set_drvdata(dsi, ctx);

    ctx->dev = &dsi->dev;
    ctx->dsi = dsi;

    ctx->i2c_ctx = i2c_get_clientdata(i2c);
    put_device(&i2c->dev);

    if (!of_property_read_u32(of_node, "dsi,lanes", &val))
        dsi->lanes = val;

    if (!of_property_read_u32(of_node, "dsi,format", &val))
        dsi->format = val;

    if (!of_property_read_u32(of_node, "dsi,flags", &val))
        dsi->mode_flags = val;

    ret = drm_of_find_panel_or_bridge(dsi->dev.of_node, 1, 0,
                      &ctx->panel, NULL);
    if (ret) {
        DRM_ERROR("could not find the panel node\n");
        return ret;
    }

    ctx->bridge.funcs = &sn65dsi83_bridge_funcs;
    ctx->bridge.of_node = dsi->dev.of_node;

    drm_bridge_add(&ctx->bridge);

    ret = mipi_dsi_attach(dsi);
    if (ret < 0) {
        drm_bridge_remove(&ctx->bridge);
        DRM_ERROR("failed to attach DSI\n");
    }

    return ret;
}

static int sn65dsi83_dsi_remove(struct mipi_dsi_device *dsi)
{
    struct sn65dsi83_dsi *ctx = mipi_dsi_get_drvdata(dsi);

    mipi_dsi_detach(dsi);
    drm_bridge_remove(&ctx->bridge);
    return 0;
}

/* -------------------------------------------------------------------------- */
static int __maybe_unused sn65dsi83_i2c_resume(struct device *dev)
{
    struct sn65dsi83_i2c *ctx = dev_get_drvdata(dev);

    dev_info(ctx->dev, "resume()\n");

    gpiod_set_value(ctx->enable_gpio, 1);
    return 0;
}

static int __maybe_unused sn65dsi83_i2c_suspend(struct device *dev)
{
    struct sn65dsi83_i2c *ctx = dev_get_drvdata(dev);

    dev_info(ctx->dev, "suspend()\n");

    gpiod_set_value(ctx->enable_gpio, 0);

    return 0;
}

static const struct dev_pm_ops sn65dsi83_pm_ops = {
    SET_RUNTIME_PM_OPS(sn65dsi83_i2c_suspend, sn65dsi83_i2c_resume, NULL)
};

/* -------------------------------------------------------------------------- */
static const struct of_device_id sn65dsi83_of_match[] = {
    {.compatible = "ti,sn65dsi83"},
    {},
};
MODULE_DEVICE_TABLE(of, sn65dsi83_of_match);

static struct i2c_driver sn65dsi83_i2c_driver = {
    .driver = {
        .name = "sn65dsi83-i2c",
        .of_match_table = sn65dsi83_of_match,
        .pm = &sn65dsi83_pm_ops,
    },
    .probe = sn65dsi83_probe,
    .remove = sn65dsi83_remove,
};

static struct mipi_dsi_driver sn65dsi83_dsi_driver = {
    .driver = {
        .name = "sn65dsi83-dsi",
        .of_match_table = sn65dsi83_of_match,
    },
    .probe = sn65dsi83_dsi_probe,
    .remove = sn65dsi83_dsi_remove,
};

static int __init sn65dsi83_init(void)
{
    if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
        mipi_dsi_driver_register(&sn65dsi83_dsi_driver);

    return i2c_add_driver(&sn65dsi83_i2c_driver);
}
module_init(sn65dsi83_init);

static void __exit sn65dsi83_exit(void)
{
    i2c_del_driver(&sn65dsi83_i2c_driver);

    if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
        mipi_dsi_driver_unregister(&sn65dsi83_dsi_driver);
}
module_exit(sn65dsi83_exit);

MODULE_AUTHOR("Oleksii Gulchenko <alexey.gulchenko@gmail.com>");
MODULE_DESCRIPTION("TI SN65DSI83 MIPI® DSI to FlatLink™ LVDS bridge driver");
MODULE_LICENSE("GPL v2");
