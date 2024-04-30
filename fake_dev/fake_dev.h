#ifndef FAKE_DEV_H
#define FAKE_DEV_H

#include <linux/dma-buf.h>
#include <linux/miscdevice.h>

struct miscdevice;
struct device_attribute;

struct fake_device_ioctl_args {
    unsigned int type;
    int fd;
    char name[32];
};

int fake_dev_dmabuf_alloc(struct page ***ret, unsigned int count);
void fake_dev_dmabuf_free(struct page **pages, unsigned int count);
int _fake_device_ioctl(struct miscdevice *misc_dev, unsigned int cmd, unsigned long args);
int fake_device_page_count_update(struct miscdevice *dev, int count, size_t size);
int fake_device_page_count_get(struct miscdevice *dev);
struct dma_buf *fake_dev_dmabuf_export(const char *name, unsigned int type, void *priv_data);
struct dma_buf *fake_dev_dmabuf_lookup_by_type(unsigned int type);
bool fake_dev_dmabuf_type_is_support(unsigned int type);
struct sg_table *fake_device_map_dmabuf(struct miscdevice *m, struct dma_buf_attachment *at, enum dma_data_direction dir);
void fake_device_release_dmabuf(struct dma_buf *dmabuf);
struct page *fake_device_dmabuf_page_get(struct miscdevice *m, int pgoff);
extern struct device_attribute dev_attr_bufsize;

#endif