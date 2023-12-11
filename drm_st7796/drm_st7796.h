#ifndef _DRM_ST7796_H_
#define _DRM_ST7796_H_

#include <linux/spi/spi.h>
#include <drm/drm_device.h>
#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_simple_kms_helper.h>

struct drm_st7796_dev {
    struct drm_device drm;
    struct drm_simple_display_pipe pipe;
    struct drm_connector connector;
    struct drm_display_mode mode;
    unsigned int rotation;
    unsigned int top_offset;
    unsigned int left_offset;
    struct backlight_device *backlight;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *dc_gpio;
    struct spi_device *spi;
    int (*command)(struct drm_st7796_dev *dev, u8 cmd, void *buf, size_t len);
    bool swap_bytes;
    void *tx_buf;
    struct {
            u32 num_curves;
            u32 num_values;
            u8 curves[128];
    }gamma;
    void (*reset)(struct drm_st7796_dev *dev);
};

#define DEFAULT_GAMMA   \
    "f0 03 0a 11 14 1c 3b 55 4a 0a 13 14 1c 1f\n" \
    "f0 03 0a 0c 0c 09 36 54 49 0f 1b 18 1f f0"

#define drm_st7796_command(dev, cmd, seq...)    \
({\
    const u8 d[] = { seq }; \
    dev->command(dev, cmd, (void *)d, ARRAY_SIZE(d));  \
})

extern const struct drm_simple_display_pipe_funcs drm_st7796_pipe_func;

static inline struct drm_st7796_dev *drm_to_drm_st7796_dev(struct drm_connector *connector)
{
    return container_of(connector, struct drm_st7796_dev, connector);
}

int drm_st7796_dev_init_with_formats(struct drm_st7796_dev *dev, \
    const struct drm_display_mode *mode, \
    const struct drm_simple_display_pipe_funcs *funcs, \
    unsigned int rotation);
void drm_st7796_debugfs_init(struct drm_minor *minor);
void drm_st7796_spi_init(struct drm_st7796_dev *dev);
int drm_st7796_gem_dumb_map_offset(struct drm_file *file, struct drm_device *drm, u32 handle, u64 *offset);

#endif