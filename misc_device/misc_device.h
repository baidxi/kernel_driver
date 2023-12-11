#ifndef MISC_DEVICE_H
#define MISC_DEVICE_H

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/configfs.h>
#include <linux/workqueue.h>
#include "u_misc.h"

struct misc_device_ringbuf {
    char *buf;
    size_t size;
    size_t head;
    size_t tail;
    size_t mask;
};

struct misc_device{
    struct usb_ep *ep_in, *ep_out;
    struct usb_composite_dev *cdev;
    struct device *dev;
    unsigned alt;
    int is_open;
    struct list_head list;
    struct config_group group;
    struct f_misc_device_opts *opts;
    int bound;
    bool is_ready;
    struct cdev chrdev;
    struct misc_device_ringbuf rx_ring;
    size_t packsize;
    struct work_struct work;
    bool use_mmap;
    dma_addr_t addr;
    size_t size;
    size_t actual;
};

struct misc_device_mmap_buf {
    dma_addr_t addr;
    size_t size;
};

extern int misc_device_create(struct misc_device *misc, struct f_misc_device_opts *opts);
extern void misc_device_destroy(struct misc_device *misc, struct f_misc_device_opts *opts);

#endif