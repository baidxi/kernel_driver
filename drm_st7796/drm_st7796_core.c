#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper.h>

#include "drm_st7796.h"

static const struct drm_display_mode drm_st7796_mode = {
    DRM_SIMPLE_MODE(320, 480, 56, 84),
};

DEFINE_DRM_GEM_CMA_FOPS(drm_st7796_fops);

static struct drm_driver drm_st7796_driver = {
    .driver_features = DRIVER_GEM | DRIVER_ATOMIC | DRIVER_MODESET,
    .fops = &drm_st7796_fops,
    DRM_GEM_CMA_DRIVER_OPS,
    .dumb_map_offset = drm_st7796_gem_dumb_map_offset,
    .debugfs_init = drm_st7796_debugfs_init,
    .name = "st7796",
    .desc = "Sitronix st7796",
    .date = "20240721",
    .major = 1,
    .minor = 0,
};

static void drm_st7796_dev_reset(struct drm_st7796_dev *dev)
{
    gpiod_set_value_cansleep(dev->reset_gpio, 0);
    usleep_range(1000, 2000);
    gpiod_set_value_cansleep(dev->reset_gpio, 1);
}

static int drm_st7796_probe(struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    struct drm_st7796_dev *st7796_dev;
    struct drm_device *drm;
    struct gpio_desc *dc_gpio;
    u32 rotation = 0;
    int ret;

    st7796_dev = devm_drm_dev_alloc(dev, &drm_st7796_driver, struct drm_st7796_dev, drm);
    if (IS_ERR(st7796_dev))
        return PTR_ERR(st7796_dev);

    st7796_dev->spi = spi;
    st7796_dev->reset = drm_st7796_dev_reset;

    drm = &st7796_dev->drm;

    st7796_dev->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(st7796_dev->reset_gpio)) {
        DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
        return PTR_ERR(st7796_dev->reset_gpio);
    }

    dc_gpio = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
    if (!IS_ERR(dc_gpio)) {
        st7796_dev->dc_gpio = dc_gpio;
    }

    if (IS_ERR(dc_gpio)) {
        if (!of_property_read_bool(spi->dev.of_node, "3wire_mode")) {
            dev_err(&spi->dev, "Not 3 wire mode must set DC gpio\n");
            return -EINVAL;
        }
    }

    st7796_dev->backlight = devm_of_find_backlight(dev);
    if (IS_ERR(st7796_dev->backlight)) {
        DRM_DEV_ERROR(dev, "backlight Not set\n");
        st7796_dev->backlight = NULL;
    }

    if (of_property_read_u32(spi->dev.of_node, "gamma,num_curves", &st7796_dev->gamma.num_curves)) {
        dev_warn(&spi->dev, "use default gamma set\n");
        st7796_dev->gamma.num_curves = 2;
        st7796_dev->gamma.num_values = 14;
        memcpy(st7796_dev->gamma.curves, DEFAULT_GAMMA, sizeof(DEFAULT_GAMMA));
        goto skip;
    }

    if ((ret = of_property_read_u32(spi->dev.of_node, \
        "gamma,num_values", \
        &st7796_dev->gamma.num_values))) 
    {
        dev_err(&spi->dev, "Failed to get gamma,num_values\n");
        return ret;
    }

    if ((ret = of_property_read_u8_array(spi->dev.of_node,      \
            "gamma", st7796_dev->gamma.curves, \
            st7796_dev->gamma.num_curves * st7796_dev->gamma.num_values))) 
    {
                dev_err(&spi->dev, "Failed to get gamma\n");
                return ret;
    }
skip:

    st7796_dev->swap_bytes = of_property_read_bool(spi->dev.of_node, "swap,bytes");

    device_property_read_u32(dev, "rotation", &rotation);

    drm->mode_config.preferred_depth = 16;

    ret = drm_st7796_dev_init_with_formats(st7796_dev, &drm_st7796_mode, &drm_st7796_pipe_func, rotation);
    if (ret)
        return ret;

    drm_st7796_spi_init(st7796_dev);

    drm_mode_config_reset(drm);

    ret = drm_dev_register(drm, 0);
    if (ret)
        return ret;

    spi_set_drvdata(spi, drm);

    drm_fbdev_generic_setup(drm, 0);
    
    return 0;
}

static int drm_st7796_remove(struct spi_device *spi)
{
    struct drm_device *drm = spi_get_drvdata(spi);

    drm_dev_unplug(drm);

    drm_atomic_helper_shutdown(drm);

    return 0;
}

static void drm_st7796_shutdown(struct spi_device *spi)
{
    drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static const struct of_device_id st7796_of_ids[] = {
    { .compatible = "sitronix,st7796"},
    {},
};

MODULE_DEVICE_TABLE(of, st7796_of_ids);

static struct spi_driver drm_st7796_spi_driver = {
    .probe = drm_st7796_probe,
    .remove = drm_st7796_remove,
    .shutdown = drm_st7796_shutdown,
    .driver = {
        .name = "drm_st7796",
        .of_match_table = of_match_ptr(st7796_of_ids),
    },
};

module_spi_driver(drm_st7796_spi_driver);

MODULE_LICENSE("GPL");