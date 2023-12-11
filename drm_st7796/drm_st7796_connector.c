#include <drm/drm_fourcc.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>

#include "drm_st7796.h"


static int drm_st7796_connector_get_modes(struct drm_connector *connector)
{
    struct drm_st7796_dev *dev = drm_to_drm_st7796_dev(connector);
    struct drm_display_mode *mode;

    mode = drm_mode_duplicate(&dev->drm, &dev->mode);
    if (!mode) {
        DRM_ERROR("Failed to duplicate mode\n");
        return 0;
    }

    if (mode->name[0] == '\0')
        drm_mode_set_name(mode);

    mode->type |= DRM_MODE_TYPE_PREFERRED;
    drm_mode_probed_add(connector, mode);

    if (mode->width_mm) {
        connector->display_info.width_mm = mode->width_mm;
        connector->display_info.height_mm = mode->height_mm;
    }

    return 1;
}

static const struct drm_connector_helper_funcs drm_st7796_connector_hfuncs = {
    .get_modes = drm_st7796_connector_get_modes,
};

static const struct drm_connector_funcs drm_st7796_connector_funcs = {
    .reset = drm_atomic_helper_connector_reset,
    .fill_modes = drm_helper_probe_single_connector_modes,
    .destroy = drm_connector_cleanup,
    .atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
    .atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const uint32_t drm_st7796_formsts[] = {
    DRM_FORMAT_RGB565,
    DRM_FORMAT_XRGB8888,
};

static const struct drm_mode_config_funcs drm_st7796_mode_config_funcs = {
    .fb_create = drm_gem_fb_create_with_dirty,
    .atomic_check = drm_atomic_helper_check,
    .atomic_commit = drm_atomic_helper_commit,
};

int drm_st7796_dev_init_with_formats(struct drm_st7796_dev *dev, 
    const struct drm_display_mode *mode, 
    const struct drm_simple_display_pipe_funcs *funcs, 
    unsigned int rotation)
{
    static const uint64_t modifiers[] = {
        DRM_FORMAT_MOD_LINEAR,
        DRM_FORMAT_MOD_INVALID,
    };
    struct drm_device *drm = &dev->drm;
    int ret;

    ret = drmm_mode_config_init(drm);
    if (ret)
        return ret;

    drm_mode_copy(&dev->mode, mode);

    drm_connector_helper_add(&dev->connector, &drm_st7796_connector_hfuncs);

    ret = drm_connector_init(drm, &dev->connector, &drm_st7796_connector_funcs, DRM_MODE_CONNECTOR_SPI);
    if (ret)
        return ret;

    dev->connector.status = connector_status_connected;

    ret = drm_simple_display_pipe_init(drm, &dev->pipe, funcs, drm_st7796_formsts, ARRAY_SIZE(drm_st7796_formsts), modifiers, &dev->connector);
    if (ret)
        return ret;


    drm_plane_enable_fb_damage_clips(&dev->pipe.plane);

    drm->mode_config.funcs = &drm_st7796_mode_config_funcs,
    drm->mode_config.min_width = dev->mode.hdisplay;
    drm->mode_config.max_width = dev->mode.hdisplay;
    drm->mode_config.min_height = dev->mode.vdisplay;
    drm->mode_config.max_height = dev->mode.vdisplay;
    dev->rotation = rotation;

    DRM_DEBUG_KMS("rotation = %u\n", rotation);

    return 0;
}