// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for Sitronix ST7796u panels
 *
 * Copyright 2023 jeck.chen <jeck.chen@dbappsecurity.com.cn>
 *
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/tinydrm/mipi-dbi.h>
#include <drm/tinydrm/tinydrm-helpers.h>
#include <video/mipi_display.h>
#include <linux/dma-mapping.h>

enum st7796_id {
    ST7796U,
    ST7796S,
};

#define DEFAULT_GAMMA   \
    "f0 03 0a 11 14 1c 3b 55 4a 0a 13 14 1c 1f\n" \
    "f0 03 0a 0c 0c 09 36 54 49 0f 1b 18 1f f0"


struct st7796_device {
    struct mipi_dbi mipi;
    struct spi_device *spi;
    struct {
            u32 num_curves;
            u32 num_values;
            u8 curves[128];
    }gamma;
};

#define ST7796_MADCTL_BGR	BIT(3)
#define ST7796_MADCTL_MV	BIT(5)
#define ST7796_MADCTL_MX	BIT(6)
#define ST7796_MADCTL_MY	BIT(7)

static inline struct st7796_device *to_st7796_device(struct mipi_dbi *mipi)
{
    return container_of(mipi, struct st7796_device, mipi);
}

static void st7796_enable(struct drm_simple_display_pipe *pipe, 
            struct drm_crtc_state *crtc_state,
            struct drm_plane_state *plane_state)
{
    struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
    struct mipi_dbi *mipi = mipi_dbi_from_tinydrm(tdev);
    struct st7796_device *dev = to_st7796_device(mipi);
    u8 *curves = dev->gamma.curves;
    u8 addr_mode;
    int i, j, k;
    const u8 gamma_par_mask[] = {
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

    for (i = 0; i < dev->gamma.num_curves; i++) {
        k = i * dev->gamma.num_values;
        for (j = 0; j < dev->gamma.num_values; j++)
            curves[k + j] &= gamma_par_mask[j];
        
        mipi_dbi_command(mipi, 0xe0 + i, 
                    curves[k + 0], curves[k + 1], curves[k + 2],
                    curves[k + 3], curves[k + 4], curves[k + 5],
                    curves[k + 6], curves[k + 7], curves[k + 8],
                    curves[k + 9], curves[k + 10], curves[k + 11],
                    curves[k + 12], curves[k + 13]);
    }

    switch(mipi->rotation) {
        default:
            addr_mode = ST7796_MADCTL_MX;
            break;
        case 90:
            addr_mode = ST7796_MADCTL_MV;
            break;
        case 180:
            addr_mode = ST7796_MADCTL_MY;
            break;
        case 270:
            addr_mode = ST7796_MADCTL_MV | ST7796_MADCTL_MY | ST7796_MADCTL_MX;
            break;
    }

    addr_mode |= ST7796_MADCTL_BGR;
    mipi_dbi_command(mipi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
    mipi_dbi_enable_flush(mipi, crtc_state, plane_state);
}

static int st7796_prepare_fb(struct drm_simple_display_pipe *pipe,
					      struct drm_plane_state *plane_state)
{
    struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
    struct mipi_dbi *mipi = mipi_dbi_from_tinydrm(tdev);
	int ret;

    ret = mipi_dbi_poweron_conditional_reset(mipi);
    if (ret < 0)
        return ret;

    mipi_dbi_command(mipi, MIPI_DCS_SET_DISPLAY_OFF);
	mipi_dbi_command(mipi, MIPI_DCS_EXIT_SLEEP_MODE);

    mipi_dbi_command(mipi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);

    mipi_dbi_command(mipi, 0xe6, 0x0f, 0xf2, 0x3f, 0x4f, 0x4f, 0x28, 0x0e, 0x00);
    mipi_dbi_command(mipi, 0xc5, 0x20);
    mipi_dbi_command(mipi, 0xb4, 0x01);

    mipi_dbi_command(mipi, MIPI_DCS_EXIT_SLEEP_MODE);
 	mipi_dbi_command(mipi, MIPI_DCS_SET_DISPLAY_ON);

	return drm_gem_fb_prepare_fb(&pipe->plane, plane_state);
}

static const struct drm_simple_display_pipe_funcs st7796_pipe_funcs = {
    .enable     =   st7796_enable,
    .disable    =   mipi_dbi_pipe_disable,
    .update     =   tinydrm_display_pipe_update,
    .prepare_fb =	st7796_prepare_fb,
};

static const struct drm_display_mode st7796_mode[] = {
    [ST7796U] = {
        TINYDRM_MODE(320, 480, 56, 84),
    },
};

DEFINE_DRM_GEM_CMA_FOPS(st7796_fops);

static struct drm_driver st7796_driver = {
    .driver_features    =   DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME | DRIVER_ATOMIC,
    .fops               =   &st7796_fops,
    .debugfs_init       =   mipi_dbi_debugfs_init,
    .name               =   "st7796",
    .desc               =   "Sitronix st7796",
    .date               =   "20231214",
    .major              =   1,
    .minor              =   0,
    TINYDRM_GEM_DRIVER_OPS,
};

static void *st7796_dma_alloc(struct device *dev, size_t size, dma_addr_t *dma_handle, gfp_t gfp, unsigned  long attrs)
{
	return kzalloc(size, gfp);
}

static void st7796_dma_free(struct device *dev, size_t size, void *vaddr, dma_addr_t dma_handle, unsigned long attrs)
{
	kfree(vaddr);
}

static const struct dma_map_ops st7796_dma_map_ops = {
    .alloc  = st7796_dma_alloc,
    .free   = st7796_dma_free,
};

static int st7796_probe(struct spi_device *spi)
{
    struct st7796_device *dev;
    struct mipi_dbi *mipi;
    struct gpio_desc *dc;
    u32 rotation = 0;
    int ret;
    const struct drm_display_mode *mode = of_device_get_match_data(&spi->dev);    

    dev = devm_kzalloc(&spi->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dev->spi = spi;
    mipi = &dev->mipi;

    mipi->reset = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(mipi->reset)) {
        DRM_DEV_ERROR(&spi->dev, "Failed to get gpio 'reset' \n");
        return PTR_ERR(mipi->reset);
    }

    dc = devm_gpiod_get_optional(&spi->dev, "dc", GPIOD_OUT_LOW);
    if (IS_ERR(dc)) {
        DRM_DEV_ERROR(&spi->dev, "Failed to get gpio 'dc' \n");
        return PTR_ERR(dc);
    }

    mipi->backlight = of_find_backlight(&spi->dev);
    if (IS_ERR(mipi->backlight)) {
        dev_warn(&spi->dev, "backlight device not found\n");
        return -EPROBE_DEFER;
    }

    of_property_read_u32(spi->dev.of_node, "rotation", &rotation);

    if (!spi->dev.coherent_dma_mask) {
        spi->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        spi->dev.dma_mask = &spi->dev.coherent_dma_mask;
        #if defined(CONFIG_ARM64)
        if (!spi->dev.dma_ops) {
            if (get_dma_ops(&spi->dev) == &dummy_dma_ops) {
                set_dma_ops(&spi->dev, &st7796_dma_map_ops);
            }
        }
        #endif
    }


    if (of_property_read_u32(spi->dev.of_node, "gamma,num_curves", &dev->gamma.num_curves)) {
        dev_warn(&spi->dev, "use default gamma set\n");
        dev->gamma.num_curves = 2;
        dev->gamma.num_values = 14;
        memcpy(dev->gamma.curves, DEFAULT_GAMMA, sizeof(DEFAULT_GAMMA));
        goto skip;
    }
    
    if ((ret = of_property_read_u32(spi->dev.of_node, "gamma,num_values", &dev->gamma.num_values))) {
        DRM_DEV_ERROR(&spi->dev, "Failed to get gamma,num_values\n");
        return ret;
    }

    if ((ret = of_property_read_u8_array(spi->dev.of_node, "gamma", dev->gamma.curves, dev->gamma.num_curves * dev->gamma.num_values))) {
        DRM_DEV_ERROR(&spi->dev, "Failed to get gamma\n");
        return ret;
    }


skip:
    ret = mipi_dbi_spi_init(spi, mipi, dc);
    if (ret)
        return ret;

    ret = mipi_dbi_init(&spi->dev, mipi, &st7796_pipe_funcs, 
                &st7796_driver, mode, rotation);

    if (ret)
        return ret;

    spi_set_drvdata(spi, mipi);

    dev_info(&spi->dev, "register tinydrm \n");

    return devm_tinydrm_register(&mipi->tinydrm);
}

static void st7796_shutdown(struct spi_device *spi)
{
    struct mipi_dbi *mipi = spi_get_drvdata(spi);
    tinydrm_shutdown(&mipi->tinydrm);
}

static const struct of_device_id st7796_of_ids[] = {
    { .compatible = "sitronix,st7796u", .data = &st7796_mode[ST7796U] },
    {},
};
MODULE_DEVICE_TABLE(of, st7796_of_ids);

static struct spi_driver st7796_spi_driver = {
    .driver = {
        .name = "st7796",
        .of_match_table = st7796_of_ids,
    },
    .probe      = st7796_probe,
    .shutdown   = st7796_shutdown,
};
module_spi_driver(st7796_spi_driver);


MODULE_DESCRIPTION("Sitronix ST7796 DRM driver");
MODULE_AUTHOR("jeck.chen <jeck.chen@dbappsecurity.com.cn");
MODULE_LICENSE("GPL");

