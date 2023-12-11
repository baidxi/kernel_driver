#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/poll.h>

#include "misc_device.h"
#include "u_misc.h"

static DECLARE_WAIT_QUEUE_HEAD(msg_wait);

static ssize_t misc_device_read (struct file *fp, char __user *to, size_t size, loff_t *off)
{
    struct misc_device *misc = fp->private_data;
    struct misc_device_ringbuf *ring = &misc->rx_ring;
    size_t count = (ring->head & ring->mask) - (ring->tail & ring->mask);
    char *start =ring->buf +  (ring->tail & ring->mask);
    int ret;

    if (misc->addr) {
        return misc->actual;
    }

    if (count == 0)
        return 0;

    if (size > count ) {
        ret = copy_to_user(to, start, count);
        if (ret)
            return ret;

        ring->tail += count;
        return count;
    }

    ret = copy_to_user(to, start, size);
    if (ret)
        return ret;

    ring->tail += size;
    return size;
}

static unsigned int misc_device_poll(struct file *fp, poll_table *wait)
{
    unsigned int mask = 0;

    poll_wait(fp, &msg_wait, wait);

    mask = POLL_IN | POLLRDNORM;

    return mask;
}

static void misc_device_transfer_complete(struct usb_ep *ep, struct usb_request *req)
{
    int status = req->status;
    struct misc_device *misc = req->context;

    switch(status) {
        case 0:
            if (!misc->use_mmap)
                kfree(req->buf);
            usb_ep_free_request(ep, req);
            break;
        default:
            dev_err(misc->dev, "FAILED to transfer:%d\n", status);
    }
}

static ssize_t misc_device_write (struct file *fp, const char __user *user, size_t size, loff_t *off)
{
    struct misc_device *misc = fp->private_data;
    struct usb_request *req;
    struct misc_device_mmap_buf *mmap;
    char *buf;
    int ret;

    if (misc->use_mmap) {
        if (size != sizeof(*mmap))
            return -EINVAL;
    }

    req = usb_ep_alloc_request(misc->ep_in, GFP_ATOMIC);
    if (!req)
        return -ENOMEM;

    buf = kzalloc(size, GFP_KERNEL);
    if (!buf) {
        usb_ep_free_request(misc->ep_in, req);
        return -ENOMEM;
    }

    ret = copy_from_user(buf, user, size);
    if (ret) {
        kfree(buf);
        usb_ep_free_request(misc->ep_in, req);
        return ret;
    }

    if (misc->use_mmap) {
        mmap = (struct misc_device_mmap_buf *)buf;
        req->buf = (unsigned long *)mmap->addr;
        req->length = mmap->size;
    } else {
        req->length = size;
        req->buf = buf;
    }

    req->context = misc;
    req->complete = misc_device_transfer_complete;


    ret = usb_ep_queue(misc->ep_in, req, GFP_ATOMIC);
    if (ret) {
        kfree(buf);
        usb_ep_free_request(misc->ep_in, req);
        return ret;
    }

    return size;
}

static void misc_device_rx_complete(struct usb_ep *ep, struct usb_request *req)
{
    int status = req->status;
    struct misc_device *misc = req->context;
    struct misc_device_ringbuf *ring = &misc->rx_ring;

    switch(status) {
        case 0:
            ring->head += req->actual;
            misc->actual = req->actual;
            usb_ep_free_request(ep, req);
            wake_up_interruptible(&msg_wait);
            break;
        default:
            dev_err(misc->dev, "rx failed:%d\n", status);
            usb_ep_set_halt(misc->ep_out);
            return;
    }

    schedule_work(&misc->work);
}

static void misc_device_worker(struct work_struct *work)
{
    struct misc_device *misc = container_of(work, struct misc_device, work);
    struct usb_request *req = usb_ep_alloc_request(misc->ep_out, GFP_ATOMIC);
    struct misc_device_ringbuf *ring = &misc->rx_ring;

    if (!req) {
        dev_err(misc->dev, "alloc ep FAIL\n");
        return;
    }

    if (misc->addr) {
        req->buf = (void *)misc->addr;
        req->length = misc->size;
    } else {
        if ((ring->size - (ring->head & ring->mask)) < misc->packsize) {
            req->buf = ring->buf;
        } else {
            req->buf = ring->buf + (ring->head & ring->mask);
        }

        req->length = misc->packsize;
    }

    req->context = misc;
    req->complete = misc_device_rx_complete;

    usb_ep_queue(misc->ep_out, req, GFP_ATOMIC);
}

static int misc_device_open (struct inode *inode, struct file *fp)
{
    struct misc_device *misc = container_of(inode->i_cdev, struct misc_device, chrdev);
    struct usb_request *req;
    int ret;

    if (!misc->is_ready) {
        return -ENOSYS;
    }

    if (misc->is_open)
        return -EBUSY;

    INIT_WORK(&misc->work, misc_device_worker);

    misc->rx_ring.buf = kzalloc(misc->rx_ring.size, GFP_KERNEL);
    if(!misc->rx_ring.buf) {
        return -ENOMEM;
    }

    misc->rx_ring.head = misc->rx_ring.tail = 0;
    misc->rx_ring.mask = misc->rx_ring.size - 1;

    req = usb_ep_alloc_request(misc->ep_out, GFP_ATOMIC);
    if (!req) {
        ret = -ENOMEM;
        goto err_out;
    }

    req->buf = misc->rx_ring.buf;
    req->context = misc;
    req->length = misc->packsize;
    req->complete = misc_device_rx_complete;

    ret = usb_ep_queue(misc->ep_out, req, GFP_ATOMIC);
    if (ret) {
        goto err_out;
    }

    misc->is_open++;

    fp->private_data = misc;

    return 0;

err_out:
    usb_ep_free_request(misc->ep_out, req);
    kfree(misc->rx_ring.buf);
    return ret;
}

static int misc_device_release (struct inode *inode, struct file *fp)
{
    struct misc_device *misc = fp->private_data;

    if (!misc->is_open)
        return 0;

    kfree(misc->rx_ring.buf);

    misc->is_open--;

    fp->private_data = NULL;

    return 0;
}

static const struct file_operations misc_device_ops = {
    .read = misc_device_read,
    .write = misc_device_write,
    .open = misc_device_open,
    .release = misc_device_release,
    .poll = misc_device_poll,
};

static ssize_t misc_device_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EOPNOTSUPP;
}

static ssize_t misc_device_addr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct misc_device *misc = dev_get_drvdata(dev);
    char *str = kstrdup(buf, GFP_KERNEL);
    char *t = str;
    char *s;
    unsigned long val;
    int ret;

    s = strsep(&t, ",");

    if (!s) {
        ret = -EINVAL;
        goto out;
    }

    ret = kstrtoul(s, 0, &val);
    if (ret)
        goto out;

    misc->addr = val;

    ret = kstrtoul(t, 0, &val);
    if (ret)
        goto out;

    misc->size = val;

    kfree(str);

    return size;
    
out:
    misc->addr = 0;
    misc->size = 0;
    kfree(str);
    return ret;
}

DEVICE_ATTR(addr, 0644, misc_device_addr_show, misc_device_addr_store);

static ssize_t misc_device_use_mmap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct misc_device *misc = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", misc->use_mmap);
}

static ssize_t misc_device_use_mmap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct misc_device *misc = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;

    misc->use_mmap = !!val;

    return size;
}

DEVICE_ATTR(use_mmap, 0644, misc_device_use_mmap_show, misc_device_use_mmap_store);

static ssize_t misc_device_rx_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct misc_device *misc = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", misc->rx_ring.head);
}

static ssize_t misc_device_rx_count_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    return -EOPNOTSUPP;
}

DEVICE_ATTR(rx_count, 0644, misc_device_rx_count_show, misc_device_rx_count_store);

static ssize_t misc_device_bufsize_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct misc_device *misc = dev_get_drvdata(dev);
    return sprintf(buf, "0x%x\n", misc->rx_ring.size);
}

static ssize_t misc_device_bufsize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct misc_device *misc = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    if (misc->is_open)
        return -EBUSY;

    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;

    misc->rx_ring.size = val;

    return size;
}

DEVICE_ATTR(bufsize, 0644, misc_device_bufsize_show, misc_device_bufsize_store);

static struct attribute *misc_device_attrs[] = {
    &dev_attr_bufsize.attr,
    &dev_attr_rx_count.attr,
    &dev_attr_use_mmap.attr,
    &dev_attr_addr.attr,
    NULL,
};

ATTRIBUTE_GROUPS(misc_device);

int misc_device_create(struct misc_device *misc, struct f_misc_device_opts *opts)
{
    cdev_init(&misc->chrdev, &misc_device_ops);
    cdev_add(&misc->chrdev, opts->dev, 1);

    misc->dev = device_create_with_groups(opts->cls, NULL, opts->dev, misc, misc_device_groups, "misc_device%d", MINOR(opts->dev));

    if (IS_ERR(misc->dev)) {
        ERROR(misc->cdev, "FAILED to create device:%ld\n", PTR_ERR(misc->dev));
        return PTR_ERR(misc->dev);
    }

    return 0;
}

void misc_device_destroy(struct misc_device *misc, struct f_misc_device_opts *opts)
{
    cdev_del(&misc->chrdev);
    device_destroy(opts->cls, opts->dev);
}