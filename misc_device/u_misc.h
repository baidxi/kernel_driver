#ifndef U_MISC_H
#define U_MISC_H

#include <linux/usb/composite.h>
#include <linux/configfs.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/list.h>

struct f_misc_device_opts {
    struct usb_function_instance inst;
    struct list_head list;
    struct class *cls;
    dev_t dev;
};

static inline struct f_misc_device_opts *to_misc_device_opts(struct usb_function_instance *fi)
{
    return container_of(fi, struct f_misc_device_opts, inst);
}

#endif