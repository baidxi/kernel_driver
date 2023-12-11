#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dma-map-ops.h>
#include <asm/page.h>

#include <drm/drm_print.h>

#include "drm_st7796.h"

static void *drm_st7796_dma_alloc(struct device *dev, size_t size, dma_addr_t *dma_handle, gfp_t gfp, unsigned  long attrs)
{
    struct page *page;
    int cur_order;
    
    size = PAGE_ALIGN(size);

    cur_order = get_order(size);

    page = alloc_pages(gfp, cur_order);
    if (!page)
        return page;

    *dma_handle = page_to_phys(page);

    return page_to_virt(page);
}

static void drm_st7796_dma_free(struct device *dev, size_t size, void *vaddr, dma_addr_t dma_handle, unsigned long attrs)
{
    struct page *page = virt_to_page(vaddr);
    int cur_order;

    size = PAGE_ALIGN(size);
    cur_order = get_order(size);

    __free_pages(page, cur_order);
}

static const struct dma_map_ops drm_st7796_dma_ops = {
    .alloc = drm_st7796_dma_alloc,
    .free = drm_st7796_dma_free,
};

static size_t drm_st7796_dev_spi_max_transfer_size(struct spi_device *spi, size_t max_len)
{
    size_t ret;

    ret = min(spi_max_transfer_size(spi), spi->master->max_dma_len);
    if (max_len)
        ret = min(ret, max_len);
    
    ret &= ~0x3;
    if (ret < 4)
        ret = 4;

    return ret;
}

static int drm_st7796_dev_spi_transfer(struct spi_device *spi, u32 hz, 
        struct spi_transfer *header, u8 bpw, const void *buf, size_t len)
{

    struct spi_transfer t = {
        .bits_per_word = bpw,
        .speed_hz = hz,
    };
    struct spi_message m;
    size_t max_chunk;
    size_t chunk;
    int ret;

    max_chunk = drm_st7796_dev_spi_max_transfer_size(spi, 0);

    spi_message_init(&m);

    t.tx_buf = buf;
    t.len = len;

    if (header)
        spi_message_add_tail(header, &m);

    spi_message_add_tail(&t, &m);

    while(len) {
        chunk = min(len, max_chunk);
        t.tx_buf = buf;
        t.len = chunk;

        buf += chunk;
        len -= chunk;

        ret = spi_sync(spi, &m);
        if (ret < 0) {
            dev_err(&spi->dev, "spi sync err:%d\n", ret);
            return ret;
        }
    }

    return 0;
}

static u32 drm_st7796_dev_spi_max_speed(struct spi_device *spi, size_t len)
{
    return len > 64 ? 0 : min_t(u32, 10000000, spi->max_speed_hz);
}

static int drm_st7796_spi_command(struct drm_st7796_dev *dev, u8 cmd, void *buf, size_t num)
{
    struct spi_device *spi = dev->spi;
    u32 speed_hz;
    int ret;

    if (dev->dc_gpio) {
        gpiod_set_value_cansleep(dev->dc_gpio, 0);
    }

    speed_hz = drm_st7796_dev_spi_max_speed(spi, 1);

    ret = drm_st7796_dev_spi_transfer(spi, speed_hz, NULL, 8, &cmd, 1);

    if (ret || !num) {
        return ret;
    }

    if (dev->dc_gpio)
        gpiod_set_value_cansleep(dev->dc_gpio, 1);

    speed_hz = drm_st7796_dev_spi_max_speed(spi, num);

    return drm_st7796_dev_spi_transfer(spi, speed_hz, NULL, 8, buf, num);
}

void drm_st7796_spi_init(struct drm_st7796_dev *dev)
{
    struct spi_device *spi = dev->spi;
    struct drm_device *drm = &dev->drm;

    if (!spi->dev.coherent_dma_mask) {
        spi->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        spi->dev.dma_mask = &spi->dev.coherent_dma_mask;
        if (!get_dma_ops(&spi->dev)) {
            set_dma_ops(&spi->dev, &drm_st7796_dma_ops);
        }
    }
    dev->command = drm_st7796_spi_command;

    dev->tx_buf = devm_kzalloc(drm->dev, drm->mode_config.max_height * drm->mode_config.max_width * 2, GFP_KERNEL);
    if (!dev->tx_buf) {
        drm_err(drm, "alloc tx buf error\n");
    }
}