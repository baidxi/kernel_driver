// SPDX-License-Identifier: GPL-2.0+
/*
 * FB driver for the ST7796U LCD Controller
 *
 * Copyright (C) 2023 jeck.chen
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <video/mipi_display.h>

#include "fbtft.h"

#define DRVNAME "fb_st7796u"

#define DEFAULT_GAMMA   \
    "f0 03 0a 11 14 1c 3b 55 4a 0a 13 14 1c 1f\n" \
    "f0 03 0a 0c 0c 09 36 54 49 0f 1b 18 1f f0"


#define MADCTL_BGR BIT(3) /* bitmask for RGB/BGR order */
#define MADCTL_MV BIT(5) /* bitmask for page/column order */
#define MADCTL_MX BIT(6) /* bitmask for column address order */
#define MADCTL_MY BIT(7) /* bitmask for page address order */

static int init_display(struct fbtft_par *par)
{
    /* turn off sleep mode */
    write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);

    mdelay(20);

    /* set pixel format to RGB-565 */
    write_reg(par, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);

    write_reg(par, 0xe6, 0x0f, 0xf2, 0x3f, 0x4f, 0x4f, 0x28, 0x0e, 0x00);
    write_reg(par, 0xc5, 0x20);

    write_reg(par, 0xb4, 0x01);

    write_reg(par, MIPI_DCS_SET_DISPLAY_ON);
    write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);

    return 0;
}

static int set_var(struct fbtft_par *par)
{
	u8 madctl_par = 0;

	if (par->bgr)
		madctl_par |= MADCTL_BGR;
	switch (par->info->var.rotate) {
	case 0:
		break;
	case 90:
		madctl_par |= (MADCTL_MV | MADCTL_MY);
		break;
	case 180:
		madctl_par |= (MADCTL_MX | MADCTL_MY);
		break;
	case 270:
		madctl_par |= (MADCTL_MV | MADCTL_MX);
		break;
	default:
		return -EINVAL;
	}

	write_reg(par, MIPI_DCS_SET_ADDRESS_MODE, madctl_par);
    
	return 0;
}

static int set_gamma(struct fbtft_par *par, u32 *curves)
{
    int i, j, k;

    static const u8 gamma_par_mask[] = {
        0xff,
        0x3f,
        0x3f,
        0x0f,
        0x0f,
        0x1f,
        0x7f,
        0x77,
        0x7f,
        0x3f,
        0x1f,
        0x1f,
        0x3f,
        0x3f,
    };

    for (i = 0; i < par->gamma.num_curves; i++) {
        k = i * par->gamma.num_values;
        for (j = 0; j < par->gamma.num_values; j++)
            curves[k + j] &= gamma_par_mask[j];
        write_reg(par, 0xe0 + i,
            curves[k + 0], curves[k + 1], curves[k + 2],
            curves[k + 3], curves[k + 4], curves[k + 5],
            curves[k + 6], curves[k + 7], curves[k + 8],
            curves[k + 9], curves[k + 10], curves[k + 11],
            curves[k + 12], curves[k + 13]);
    }
    return 0;
}

static int blank(struct fbtft_par *par, bool on)
{
    if (on)
        write_reg(par, MIPI_DCS_SET_DISPLAY_OFF);
    else
        write_reg(par, MIPI_DCS_SET_DISPLAY_ON);

    return 0;
}

static struct fbtft_display display = {
    .regwidth = 8,
    .width = 320,
    .height = 480,
    .gamma_num = 2,
    .gamma_len = 14,
    .gamma = DEFAULT_GAMMA,
    .fbtftops = {
        .init_display = init_display,
        .set_var = set_var,
        .set_gamma = set_gamma,
        .blank = blank,
    },
};

FBTFT_REGISTER_DRIVER(DRVNAME, "sitronix,st7796u", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:st7796u");
MODULE_ALIAS("platform:st7796u");

MODULE_DESCRIPTION("FB driver for the ST7796U LCD Controller");
MODULE_AUTHOR("jeck.chen");
MODULE_LICENSE("GPL");
