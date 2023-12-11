#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/composite.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/usb/gadget.h>
#include <linux/usb/ch9.h>

#include "u_misc.h"
#include "misc_device.h"

struct f_misc_device {
    struct usb_function func;
    struct f_misc_device_opts *opts;
    struct misc_device usb_dev;
};

static struct usb_interface_descriptor usb_intf_alt0 = {
    .bLength            =   USB_DT_INTERFACE_SIZE,
    .bDescriptorType    =   USB_DT_INTERFACE,
    .bNumEndpoints      =   2,
    .bAlternateSetting  =   0,
    .bInterfaceClass    = USB_CLASS_VENDOR_SPEC,
};

/* Full speed */
static struct usb_endpoint_descriptor fs_misc_in_desc = {
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,
    .bEndpointAddress   =   USB_DIR_IN,
    .bmAttributes       =   USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_misc_out_desc = {
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,
    .bEndpointAddress   =   USB_DIR_OUT,
    .bmAttributes       =   USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_misc_desc[] = {
    (struct usb_descriptor_header *)&usb_intf_alt0,
    (struct usb_descriptor_header *)&fs_misc_in_desc,
    (struct usb_descriptor_header *)&fs_misc_out_desc,
    NULL,
};

/* hight speed */
static struct usb_endpoint_descriptor hs_misc_in_desc = {
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,
    .bmAttributes       =   USB_ENDPOINT_XFER_BULK,
    .bEndpointAddress   =   USB_DIR_IN,
    .wMaxPacketSize     =   cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_misc_out_desc = {
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,
    .bmAttributes       =   USB_ENDPOINT_XFER_BULK,
    .bEndpointAddress   =   USB_DIR_OUT,
    .wMaxPacketSize     =   cpu_to_le16(512),
};

static struct usb_descriptor_header *hs_misc_desc[] = {
    (struct usb_descriptor_header *)&usb_intf_alt0,
    (struct usb_descriptor_header *)&hs_misc_in_desc,
    (struct usb_descriptor_header *)&hs_misc_out_desc,
    NULL,
};

/* super speed */
static struct usb_endpoint_descriptor ss_misc_in_desc = {
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,
    .bmAttributes       =   USB_ENDPOINT_XFER_BULK,
    .bEndpointAddress   =   USB_DIR_IN,
    .wMaxPacketSize     =   cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor ss_misc_out_desc = {
    .bLength            =   USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    =   USB_DT_ENDPOINT,
    .bmAttributes       =   USB_ENDPOINT_XFER_BULK,
    .bEndpointAddress   =   USB_DIR_OUT,
    .wMaxPacketSize     =   cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor ss_misc_comp_desc = {
    .bLength            =   USB_DT_SS_EP_COMP_SIZE,
    .bDescriptorType    =   USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *ss_misc_desc[] = {
    (struct usb_descriptor_header *)&usb_intf_alt0,
    (struct usb_descriptor_header *)&ss_misc_in_desc,
    (struct usb_descriptor_header *)&ss_misc_comp_desc,
    (struct usb_descriptor_header *)&ss_misc_out_desc,
    (struct usb_descriptor_header *)&ss_misc_comp_desc,
    NULL,
};

extern struct config_item_type f_misc_device_root_type;

/* function specific strings */
static struct usb_string strings_f_misc[] = {
    [0].s = "USB misc device",
    {}
};

static struct usb_gadget_strings stringtab_f_misc = {
    .language   =   0x0409,
    .strings    =   strings_f_misc,
};

static struct usb_gadget_strings *f_misc_strings[] = {
    &stringtab_f_misc,
    NULL,
};

static inline struct f_misc_device *to_f_misc_device(struct usb_function *f)
{
    return container_of(f, struct f_misc_device, func);
}

static int f_misc_device_bind(struct usb_configuration *c, struct usb_function *f)
{
    struct usb_composite_dev *cdev = f->config->cdev;
    struct f_misc_device *misc_dev = to_f_misc_device(f);
    int ret, id;

    id = usb_interface_id(c, f);

    if (id < 0) {
        return id;
    }

    usb_intf_alt0.bInterfaceNumber = id;

    misc_dev->usb_dev.ep_in = usb_ep_autoconfig(cdev->gadget, &fs_misc_in_desc);
    if (!misc_dev->usb_dev.ep_in) {
autoconfig_fail:
        ERROR(cdev, "%s: can't autoconfig on %s\n", f->name, cdev->gadget->name);
        return -EIO;
    }

    misc_dev->usb_dev.ep_out = usb_ep_autoconfig(cdev->gadget, &fs_misc_out_desc);
    if (!misc_dev)
        goto autoconfig_fail;

    hs_misc_in_desc.bEndpointAddress = fs_misc_in_desc.bEndpointAddress;
    hs_misc_out_desc.bEndpointAddress = fs_misc_out_desc.bEndpointAddress;

    ss_misc_in_desc.bEndpointAddress = fs_misc_in_desc.bEndpointAddress;
    ss_misc_out_desc.bEndpointAddress = fs_misc_out_desc.bEndpointAddress;

    ret = usb_assign_descriptors(f, fs_misc_desc, hs_misc_desc, ss_misc_desc, ss_misc_desc);

    if (ret)
        return ret;

    misc_dev->usb_dev.cdev = cdev;
    f->strings = f_misc_strings;

    INFO(cdev, "%s speed %s: IN/%s, OUT/%s",
        (gadget_is_superspeed_plus(cdev->gadget) ? "super plus" :
        (gadget_is_superspeed(cdev->gadget) ? "super" :
        (gadget_is_dualspeed(cdev->gadget) ? "dual" : "full"))),
        f->name, misc_dev->usb_dev.ep_in->name, misc_dev->usb_dev.ep_out->name);

    misc_dev->usb_dev.bound = true;
    misc_dev->usb_dev.packsize = \
                gadget_is_superspeed_plus(cdev->gadget) ? ss_misc_in_desc.wMaxPacketSize : \
                gadget_is_superspeed(cdev->gadget) ? ss_misc_in_desc.wMaxPacketSize : \
                gadget_is_dualspeed(cdev->gadget) ? hs_misc_in_desc.wMaxPacketSize : 512;

    return 0;
}

static void f_misc_device_disable(struct usb_function *f)
{
    struct f_misc_device *f_misc = to_f_misc_device(f);
    struct misc_device *misc = &f_misc->usb_dev;

    usb_ep_disable(misc->ep_in);
    usb_ep_disable(misc->ep_out);

    misc->is_ready = false;
}

static int f_misc_device_enable(struct usb_function *f, unsigned intf, unsigned alt)
{
    struct f_misc_device *f_misc = to_f_misc_device(f);
    struct usb_composite_dev *cdev = f->config->cdev;
    // struct f_misc_device_opts *opts = f_misc->opts;
    struct usb_ep *ep;
    int ret;

    ep = f_misc->usb_dev.ep_in;

    ret = config_ep_by_speed(cdev->gadget, f, ep);
    if (ret) {
        ERROR(cdev, "FAILED to config ep %s:%d\n", ep->name, ret);
        return ret;
    }

    ret = usb_ep_enable(ep);
    if (ret) {
        ERROR(cdev, "FAILED to enable ep %s:%d\n", ep->name, ret);
        return ret;
    }

    ep->driver_data = &f_misc->usb_dev;

    ep = f_misc->usb_dev.ep_out;
    ret = config_ep_by_speed(cdev->gadget, f, ep);
    if (ret) {
        usb_ep_disable(f_misc->usb_dev.ep_in);
        ERROR(cdev, "FAILED to config ep %s:%d\n", ep->name, ret);
        return ret;
    }

    ret = usb_ep_enable(ep);
    if (ret) {
        usb_ep_disable(f_misc->usb_dev.ep_in);
        ERROR(cdev, "FAILED to enable ep %s:%d\n", ep->name, ret);
        return ret;
    }

    ep->driver_data = &f_misc->usb_dev;
    f_misc->usb_dev.alt = alt;
    f_misc->usb_dev.is_ready = true;

    return ret;
}

static int f_misc_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
    f_misc_device_disable(f);
    return f_misc_device_enable(f, intf, alt);
}

static int f_misc_get_alt(struct usb_function *f, unsigned intf)
{
    struct f_misc_device *dev = to_f_misc_device(f);
    return dev->usb_dev.alt;
}

static int f_misc_device_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
    INFO(f->config->cdev, "bReqesut:%02x wIndex:%02x wLength:%02x wValue:%02x\n", 
                        ctrl->bRequest, ctrl->wIndex, ctrl->wLength, ctrl->wValue);
    return 0;
}

static void f_misc_device_func_free(struct usb_function *f)
{
    struct f_misc_device *dev = to_f_misc_device(f);
    misc_device_destroy(&dev->usb_dev, dev->opts);
    kfree(dev);
}

static void f_misc_device_instace_free(struct usb_function_instance *fi)
{
    struct f_misc_device_opts *opts = to_misc_device_opts(fi);
    class_destroy(opts->cls);
    unregister_chrdev_region(opts->dev, 1);
    kfree(opts);
}

static struct usb_function_instance *f_misc_device_alloc_inst(void)
{
    struct f_misc_device_opts *opts;
    int ret;

    opts = kzalloc(sizeof(*opts), GFP_KERNEL);
    if (!opts)
        return ERR_PTR(-ENOMEM);

    opts->inst.free_func_inst = f_misc_device_instace_free;
    INIT_LIST_HEAD(&opts->list);

    opts->cls = class_create(THIS_MODULE, "misc_device");

    if (IS_ERR(opts->cls)) {
        void *r = opts->cls;
        kfree(opts);
        return r;
    }

    ret = alloc_chrdev_region(&opts->dev, 0, 1, "misc_device");

    if (ret) {
        goto err_out;
    }

    config_group_init_type_name(&opts->inst.group, "", &f_misc_device_root_type);

    return &opts->inst;
err_out:
    class_destroy(opts->cls);
    kfree(opts);
    return ERR_PTR(ret);
}

static struct usb_function *f_misc_device_alloc_func(struct usb_function_instance *fi)
{
    struct f_misc_device *f_misc;
    struct f_misc_device_opts *opts;
    int ret;

    opts = to_misc_device_opts(fi);

    f_misc = kzalloc(sizeof(*f_misc), GFP_KERNEL);
    if (!f_misc) {
        return ERR_PTR(-ENOMEM);
    }

    f_misc->func.name = "misc_device";
    f_misc->func.bind = f_misc_device_bind;
    f_misc->func.set_alt = f_misc_set_alt;
    f_misc->func.get_alt = f_misc_get_alt;
    f_misc->func.disable = f_misc_device_disable;
    f_misc->func.setup = f_misc_device_setup;
    f_misc->func.free_func = f_misc_device_func_free;
    f_misc->opts = opts;
    f_misc->usb_dev.rx_ring.size = 0xffff + 1;
    f_misc->usb_dev.rx_ring.mask = 0xffff;

    ret = misc_device_create(&f_misc->usb_dev, opts);

    if (ret) {
        kfree(f_misc);
        return ERR_PTR(ret);
    }

    return &f_misc->func;
}

DECLARE_USB_FUNCTION_INIT(misc_device, f_misc_device_alloc_inst, f_misc_device_alloc_func);

MODULE_LICENSE("GPL");
