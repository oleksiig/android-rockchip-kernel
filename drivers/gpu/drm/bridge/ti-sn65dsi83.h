/*
 * Oleksii Gulchenko <alexey.gulchenko@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __TI_SN65DSI83_H__
#define __TI_SN65DSI83_H__

/* Register addresses */
#define DSI83_SOFT_RESET            0x09
#define DSI83_CORE_PLL              0x0A
#define DSI83_PLL_DIV               0x0B
#define DSI83_PLL_EN                0x0D
#define DSI83_DSI_CFG               0x10
#define DSI83_DSI_EQ                0x11
#define DSI83_CHA_DSI_CLK_RNG       0x12
#define DSI83_CHB_DSI_CLK_RNG       0x13
#define DSI83_LVDS_MODE             0x18
#define DSI83_LVDS_SIGN             0x19
#define DSI83_LVDS_TERM             0x1A
#define DSI83_CHA_LVDS_CM_ADJUST    0x1B
#define DSI83_CHA_LINE_LEN_LO       0x20
#define DSI83_CHA_LINE_LEN_HI       0x21
#define DSI83_CHA_VERT_LINES_LO     0x24 /* TEST PATTERN GENERATION PURPOSE ONLY */
#define DSI83_CHA_VERT_LINES_HI     0x25 /* TEST PATTERN GENERATION PURPOSE ONLY */
#define DSI83_CHA_SYNC_DELAY_LO     0x28
#define DSI83_CHA_SYNC_DELAY_HI     0x29
#define DSI83_CHA_HSYNC_WIDTH_LO    0x2C
#define DSI83_CHA_HSYNC_WIDTH_HI    0x2D
#define DSI83_CHA_VSYNC_WIDTH_LO    0x30
#define DSI83_CHA_VSYNC_WIDTH_HI    0x31
#define DSI83_CHA_HORZ_BACKPORCH    0x34
#define DSI83_CHA_VERT_BACKPORCH    0x36 /* TEST PATTERN GENERATION PURPOSE ONLY */
#define DSI83_CHA_HORZ_FRONTPORCH   0x38 /* TEST PATTERN GENERATION PURPOSE ONLY */
#define DSI83_CHA_VERT_FRONTPORCH   0x3A /* TEST PATTERN GENERATION PURPOSE ONLY */
#define DSI83_CHA_TEST_PATTERN      0x3C /* TEST PATTERN GENERATION PURPOSE ONLY */
#define DSI83_IRQ_EN                0xE0

#endif /* __TI_SN65DSI83_H__ */
