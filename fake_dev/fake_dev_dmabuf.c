#include <linux/kernel.h>

#include "fake_dev.h"

struct fake_device_dmabuf {
    struct list_head list;
    struct dma_buf *dmabuf;
    unsigned int type;
};

static LIST_HEAD(fake_device_dmabuf_list);

int fake_dev_dmabuf_alloc(struct page ***ret, unsigned int count)
{
    struct page **pages;
    int i;

    pages = kcalloc(count, sizeof(struct page *), GFP_KERNEL);
    if (!pages)
        return -ENOMEM;

    for (i = 0; i < count; i++) {
        pages[i] = alloc_page(__GFP_ZERO | GFP_ATOMIC);
        if (!pages[i])
            goto err_out;
        get_page(pages[i]);
    }

    *ret = pages;

    return 0;

err_out:
    for (; i < 0; i--) {
        if (pages[i]) {
            put_page(pages[i]);
        }
    }
    return -ENOMEM;
}

void fake_dev_dmabuf_free(struct page **pages, unsigned int count)
{
    int i;

    for (i = 0; i < count; i++) {
        put_page(pages[i]);
    }
}

static struct sg_table *fake_dev_map_dmabuf(struct dma_buf_attachment *at, enum dma_data_direction dir)
{
    struct miscdevice *misc_dev = at->dmabuf->priv;

    return fake_device_map_dmabuf(misc_dev, at, dir);
}

static void fake_dev_unmap_dmabuf(struct dma_buf_attachment *at, struct sg_table *sgt, enum dma_data_direction dir)
{
    struct miscdevice *misc_dev = at->dmabuf->priv;

    dma_unmap_sgtable(misc_dev->this_device, sgt, dir, 0);
    sg_free_table(sgt);
    kfree(sgt);
}

static void fake_dev_release_dmabuf(struct dma_buf *dmabuf)
{
    fake_device_release_dmabuf(dmabuf);
}

static vm_fault_t fake_dev_dmabuf_vm_fault(struct vm_fault *vmf)
{
    struct vm_area_struct *vma = vmf->vma;
    struct miscdevice *m = vma->vm_private_data;

    vmf->page = fake_device_dmabuf_page_get(m, vmf->pgoff);
    if (!vmf->page)
        return VM_FAULT_SIGBUS;

    get_page(vmf->page);

    return 0;
}

static const struct vm_operations_struct fake_dev_dmabuf_vm_ops = {
    .fault = fake_dev_dmabuf_vm_fault,
};

static int fake_dev_mmap_dmabuf(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
    struct miscdevice *m = dmabuf->priv;

    if ((vma->vm_flags & VM_SHARED) == 0)
        return -EINVAL;

    vma->vm_ops = &fake_dev_dmabuf_vm_ops;
    vma->vm_private_data = m;

    return 0;
}

static struct dma_buf_ops fake_device_dmabuf_ops = {
    .map_dma_buf = fake_dev_map_dmabuf,
    .unmap_dma_buf = fake_dev_unmap_dmabuf,
    .release = fake_dev_release_dmabuf,
    .mmap = fake_dev_mmap_dmabuf,
};

struct dma_buf *fake_dev_dmabuf_export(const char *name, unsigned int type, void *priv_data)
{
    struct fake_device_dmabuf *fake_dmabuf;
    struct dma_buf *dmabuf;
    const struct dma_buf_export_info info = {
        .exp_name = kstrdup(name, GFP_KERNEL),
        .flags = O_CLOEXEC,
        .priv = priv_data,
        .size = sizeof(struct miscdevice),
        .ops = &fake_device_dmabuf_ops,
    };

    dmabuf = fake_dev_dmabuf_lookup_by_type(type);
    if (dmabuf)
        return dmabuf;

    fake_dmabuf = kzalloc(sizeof(*fake_dmabuf), GFP_KERNEL);
    if (!fake_dmabuf)
        return ERR_PTR(-ENOMEM);

    fake_dmabuf->dmabuf = dma_buf_export(&info);
    if (IS_ERR(fake_dmabuf->dmabuf)) {
        kfree(fake_dmabuf);
        return fake_dmabuf->dmabuf;
    }

    fake_dmabuf->type = type;

    list_add_tail(&fake_dmabuf->list, &fake_device_dmabuf_list);

    return fake_dmabuf->dmabuf;
}

struct dma_buf *fake_dev_dmabuf_lookup_by_type(unsigned int type)
{
    struct fake_device_dmabuf *fake_dmabuf;

    list_for_each_entry(fake_dmabuf, &fake_device_dmabuf_list, list) {
        if (fake_dmabuf->type == type)
            return fake_dmabuf->dmabuf;
    }

    return NULL;
}

bool fake_dev_dmabuf_type_is_support(unsigned int type)
{
    return true;
}