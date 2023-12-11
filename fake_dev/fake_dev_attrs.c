#include <linux/kernel.h>
#include <linux/device.h>

#include "fake_dev.h"

static ssize_t fake_device_bufsize_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = fake_device_page_count_get(dev_get_drvdata(dev));

    return sprintf(buf, "%ld\n", count * PAGE_SIZE);
}

static ssize_t fake_device_bufsize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int err;
    unsigned long val;
    unsigned long sz;

    err = kstrtoul(buf, 0, &sz);
    if (err)
        return err;

    val = DIV_ROUND_UP(sz, PAGE_SIZE);

    err = fake_device_page_count_update(dev_get_drvdata(dev), val, sz);
    return err ? err : size;
}

DEVICE_ATTR(bufsize, 0644, fake_device_bufsize_show, fake_device_bufsize_store);
