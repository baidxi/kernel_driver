#include <linux/dma-buf.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_drv.h>
#include <drm/drm_format_helper.h>

#include <video/mipi_display.h>

#include "drm_st7796.h"
#include "linux/backlight.h"

#define ST7796_MADCTL_BGR	BIT(3)
#define ST7796_MADCTL_MV	BIT(5)
#define ST7796_MADCTL_MX	BIT(6)
#define ST7796_MADCTL_MY	BIT(7)

static void drm_st7796_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect);
static void drm_st7796_set_window_address(struct drm_st7796_dev *dev, 
                unsigned int xs, unsigned int xe,
                unsigned int ys, unsigned int ye);

static void drm_st7796_enable_flush(struct drm_st7796_dev *dev, 
                            struct drm_crtc_state *crtc_state,
                            struct drm_plane_state *plane_state)
{
    struct drm_framebuffer *fb = plane_state->fb;
    struct drm_rect rect = {
        .x1 = 0,
        .x2 = fb->width,
        .y1 = 0,
        .y2 = fb->height,
    };

    int idx;

    if (!drm_dev_enter(&dev->drm, &idx))
        return;

    drm_st7796_fb_dirty(fb, &rect);

    if (dev->backlight)
        backlight_enable(dev->backlight);

    drm_dev_exit(idx);
}

static void drm_st7796_blank(struct drm_st7796_dev *dev)
{
    struct drm_device *drm = &dev->drm;
    u16 height = drm->mode_config.min_height;
    u16 width = drm->mode_config.min_width;
    size_t len = width * height * 2;
    int idx;

    if (!drm_dev_enter(drm, &idx))
        return;

    memset(dev->tx_buf, 0, len);
    drm_st7796_set_window_address(dev, 0, width -1, 0, height - 1);

    dev->command(dev, MIPI_DCS_WRITE_MEMORY_START, dev->tx_buf, len);

    drm_dev_exit(idx);
}

static void drm_st7796_enable(struct drm_simple_display_pipe *pipe,
                struct drm_crtc_state *crtc_state,
                struct drm_plane_state *plane_state)
{
    struct drm_st7796_dev *dev = container_of(pipe, struct drm_st7796_dev, pipe);
    u8 *curves = dev->gamma.curves;
    u8 addr_mode = 0;
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

        drm_st7796_command(dev, 0xe0 + i,
                    curves[k + 0], curves[k + 1], curves[k + 2],
                    curves[k + 3], curves[k + 4], curves[k + 5],
                    curves[k + 6], curves[k + 7], curves[k + 8],
                    curves[k + 9], curves[k + 10], curves[k + 11],
                    curves[k + 12], curves[k + 13]
        );
    }

    switch(dev->rotation) {
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
    drm_st7796_command(dev, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
    drm_st7796_enable_flush(dev, crtc_state, plane_state);
}

static void drm_st7796_disable(struct drm_simple_display_pipe *pipe)
{
    struct drm_st7796_dev *dev = container_of(pipe, struct drm_st7796_dev, pipe);
    if (dev->backlight)
        backlight_disable(dev->backlight);
    else
        drm_st7796_blank(dev);
}

static void drm_st7796_set_window_address(struct drm_st7796_dev *dev, 
                unsigned int xs, unsigned int xe,
                unsigned int ys, unsigned int ye)
{
    xs += dev->left_offset;
    xe += dev->left_offset;
    ys += dev->top_offset;
    ye += dev->top_offset;

    drm_st7796_command(dev, MIPI_DCS_SET_COLUMN_ADDRESS, (xs >> 8) & 0xff, 
                        xs & 0xff, (xe >> 8) & 0xff, xe & 0xff);
    drm_st7796_command(dev, MIPI_DCS_SET_PAGE_ADDRESS, (ys >> 8) & 0xff,
                        ys & 0xff, (ye >> 8) & 0xff, ye & 0xff);
}

static int drm_st7796_buf_copy(void *dst, struct drm_framebuffer *fb, struct drm_rect *clip, bool swap)
{
    struct drm_gem_object *gem = drm_gem_fb_get_obj(fb, 0);
    struct drm_gem_cma_object *cma_obj = to_drm_gem_cma_obj(gem);
    struct dma_buf_attachment *import_attach = gem->import_attach;
    struct drm_format_name_buf format_name;
    void *src = cma_obj->vaddr;
    int ret;


    if (import_attach) {
        ret = dma_buf_begin_cpu_access(import_attach->dmabuf, DMA_FROM_DEVICE);
        if (ret)
            return ret;
    }

    switch(fb->format->format) {
        case DRM_FORMAT_RGB565:
            if (swap)
                drm_fb_swab(dst, src, fb, clip, !import_attach);
            else
                drm_fb_memcpy(dst, src, fb, clip);
            
            break;
        case DRM_FORMAT_XRGB8888:
            drm_fb_xrgb8888_to_rgb565(dst, src, fb, clip, swap);
            break;
        default:
            drm_err_once(fb->dev, "Format is not supported:%s\n", drm_get_format_name(fb->format->format, &format_name));
            return -EINVAL;
    }

    if (import_attach)
        ret = dma_buf_end_cpu_access(import_attach->dmabuf, DMA_FROM_DEVICE);
    
    return ret;
}

static void drm_st7796_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
    struct drm_gem_object *gem = drm_gem_fb_get_obj(fb, 0);
    struct drm_gem_cma_object *cma_obj = to_drm_gem_cma_obj(gem);
    struct drm_st7796_dev *dev = container_of(fb->dev, struct drm_st7796_dev, drm);
    unsigned int height = rect->y2 - rect->y1;
    unsigned int width = rect->x2 - rect->x1;
    bool swap = dev->swap_bytes;
    int idx, ret = 0;
    bool full;
    void *tr;

    if (WARN_ON(!fb))
        return;

    if (!drm_dev_enter(fb->dev, &idx))
        return;

    full = width == fb->width && height == fb->height;

    DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id, DRM_RECT_ARG(rect));

    if (!dev->dc_gpio || !full || swap ||
        fb->format->format == DRM_FORMAT_XRGB8888) {
            tr = dev->tx_buf;
            ret = drm_st7796_buf_copy(dev->tx_buf, fb, rect, swap);
            if (ret)
                goto err_msg;
    } else {
        tr = cma_obj->vaddr;
    }

    drm_st7796_set_window_address(dev, rect->x1, rect->x2 -1, rect->y1, rect->y2-1);

    ret = dev->command(dev, MIPI_DCS_WRITE_MEMORY_START, tr, width * height * 2);

err_msg:
    if (ret)
        drm_err_once(&dev->drm, "Failed to update display %d\n", ret);

    drm_dev_exit(idx);
}

static void drm_st7796_update(struct drm_simple_display_pipe *pipe, struct drm_plane_state *old_state)
{
    struct drm_plane_state *state = pipe->plane.state;
    struct drm_rect rect;

    if (!pipe->crtc.state->active)
        return;

    if (drm_atomic_helper_damage_merged(old_state, state, &rect))
        drm_st7796_fb_dirty(state->fb, &rect);
}

static int drm_st7796_prepare_fb(struct drm_simple_display_pipe *pipe, struct drm_plane_state *plane_state)
{
    struct drm_st7796_dev *dev = container_of(pipe, struct drm_st7796_dev, pipe);

    dev->reset(dev);

    drm_st7796_command(dev, MIPI_DCS_SET_DISPLAY_OFF);
    drm_st7796_command(dev, MIPI_DCS_EXIT_SLEEP_MODE);
    drm_st7796_command(dev, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);

    drm_st7796_command(dev, 0xe6, 0x0f, 0xf2, 0x3f, 0x4f, 0x4f, 0x28, 0x0e, 0x00);
    drm_st7796_command(dev, 0xc5, 0x20);
    drm_st7796_command(dev, 0xb4, 0x00);

    drm_st7796_command(dev, MIPI_DCS_EXIT_SLEEP_MODE);
    drm_st7796_command(dev, MIPI_DCS_SET_DISPLAY_ON);

    return drm_gem_fb_prepare_fb(&pipe->plane, plane_state);
} 

int drm_st7796_gem_dumb_map_offset(struct drm_file *file, struct drm_device *drm, u32 handle, u64 *offset)
{
    struct drm_gem_object *obj;
    int ret;

    obj = drm_gem_object_lookup(file, handle);
    if (!obj) {
        drm_err(drm, "gem object not found\n");
        return -ENOENT;
    }

    if (obj->import_attach) {
        ret = -EINVAL;
        drm_err(drm, "Don't allow imported objects to be mapped\n");
        goto out;
    }

    ret = drm_gem_create_mmap_offset(obj);
    if (ret) {
        drm_err(drm, "drm_gem_create_mmap_offset err:%d\n", ret);
        goto out;
    }

    *offset = drm_vma_node_offset_addr(&obj->vma_node);

out:
    drm_gem_object_put(obj);

    return ret;
}

const struct drm_simple_display_pipe_funcs drm_st7796_pipe_func = {
    .enable = drm_st7796_enable,
    .disable = drm_st7796_disable,
    .update = drm_st7796_update,
    .prepare_fb = drm_st7796_prepare_fb,
};