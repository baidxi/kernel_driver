#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/version.h>

#include "fake_dev.h"
#include "linux/miscdevice.h"

struct fake_device {
    struct miscdevice misc_dev;
    atomic_t ref_count;
    struct mutex lock;
    unsigned int page_count;
    struct page **pages;
    size_t size;
};

static inline struct fake_device *to_fake_device(struct miscdevice *m)
{
    return container_of(m, struct fake_device, misc_dev);
}

static void fake_dev_free_pages(struct miscdevice *m)
{
    struct fake_device *fake_dev = to_fake_device(m);
    int i;

    for (i = 0; i < fake_dev->page_count; i++)
        put_page(fake_dev->pages[i]);

    kfree(fake_dev->pages);

    fake_dev->pages = NULL;
}

struct sg_table *fake_device_map_dmabuf(struct miscdevice *m, struct dma_buf_attachment *at, enum dma_data_direction dir)
{
    struct fake_device *fake_dev = to_fake_device(m);
    struct sg_table *sgt;
    int err;

    sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
    if (!sgt)
        return ERR_PTR(-ENOMEM);
    
    err = sg_alloc_table_from_pages(sgt, fake_dev->pages, fake_dev->page_count, 0, fake_dev->size, GFP_KERNEL);
    if (err) {
        kfree(sgt);
        return ERR_PTR(err);
    }

    err = dma_map_sgtable(m->this_device, sgt, dir, 0);
    if (err) {
        sg_free_table(sgt);
        return ERR_PTR(err);
    }

    return sgt;
    
}

void fake_device_release_dmabuf(struct dma_buf *dmabuf)
{
    struct miscdevice *m = dmabuf->priv;
    fake_dev_free_pages(m);
}

struct page *fake_device_dmabuf_page_get(struct miscdevice *m, int pgoff)
{
    struct fake_device *fake_dev = to_fake_device(m);

    if (pgoff >= fake_dev->page_count)
        return NULL;

    return fake_dev->pages[pgoff];
}

static int fake_device_open(struct inode *inode, struct file *pfile)
{
    struct fake_device *fake_dev = to_fake_device(pfile->private_data);
    int err;

    mutex_lock(&fake_dev->lock);
    if (atomic_read(&fake_dev->ref_count)) {
        atomic_inc(&fake_dev->ref_count);
        mutex_unlock(&fake_dev->lock);
        return 0;
    }

    err = fake_dev_dmabuf_alloc(&fake_dev->pages, fake_dev->page_count);

    if (!err)
        atomic_inc(&fake_dev->ref_count);

    mutex_unlock(&fake_dev->lock);

    return err;
}

static int fake_device_release(struct inode *inode, struct file *pfile)
{
    struct fake_device *fake_dev = to_fake_device(pfile->private_data);

    if (atomic_dec_and_test(&fake_dev->ref_count)) {
        fake_dev_dmabuf_free(fake_dev->pages, fake_dev->page_count);
    }
    return 0;
}

static long fake_device_ioctl(struct file *pfile, unsigned int cmd, unsigned long args)
{
    return _fake_device_ioctl(pfile->private_data, cmd, args);
}

int fake_device_page_count_update(struct miscdevice *dev, int count, size_t size)
{
    struct fake_device *fake_dev = to_fake_device(dev);
    if (atomic_read(&fake_dev->ref_count))
        return -EBUSY;

    mutex_lock(&fake_dev->lock);
    fake_dev->page_count = count;
    fake_dev->size = size;
    mutex_unlock(&fake_dev->lock);

    return 0;
}

int fake_device_page_count_get(struct miscdevice *dev)
{
    struct fake_device *fake_dev = to_fake_device(dev);

    return fake_dev->page_count;
}

static struct file_operations fake_device_ops = {
    .open = fake_device_open,
    .release = fake_device_release,
    .unlocked_ioctl = fake_device_ioctl,
};

static struct fake_device fakedev;

static struct attribute *fake_device_attrs[] = {
    &dev_attr_bufsize.attr,
    NULL,
};

ATTRIBUTE_GROUPS(fake_device);

static int __init fake_dev_drv_init(void)
{
    struct miscdevice *miscdev = &fakedev.misc_dev;

    miscdev->fops = &fake_device_ops;
    miscdev->name = "fake_dev";
    miscdev->parent = NULL;
    miscdev->groups = fake_device_groups;

    atomic_set(&fakedev.ref_count, 0);
    mutex_init(&fakedev.lock);

    return misc_register(miscdev);
}

static void __exit fake_dev_drv_exit(void)
{
    misc_deregister(&fakedev.misc_dev);
}

module_init(fake_dev_drv_init);
module_exit(fake_dev_drv_exit);

#if LINUX_VERSION_CODE  > KERNEL_VERSION(5, 15, 0)
MODULE_IMPORT_NS(DMA_BUF);
#endif

MODULE_LICENSE("GPL");