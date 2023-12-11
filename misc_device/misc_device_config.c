#include <linux/configfs.h>
#include <linux/module.h>
#include <linux/usb/composite.h>
#include <linux/device.h>

#include "misc_device.h"
#include "u_misc.h"

/*
static struct configfs_attribute *f_misc_device_attrs[] = {
    NULL,
};

static void f_misc_device_release(struct config_item *item)
{
    struct misc_device *misc = container_of(to_config_group(item), struct misc_device, group);
    kfree(misc);
}

static struct configfs_item_operations f_misc_device_item_ops = {
    .release = f_misc_device_release,
};

static struct config_item_type f_misc_device_root_type = {
    .ct_item_ops = &f_misc_device_item_ops,
    .ct_attrs    = f_misc_device_attrs,
    .ct_owner    = THIS_MODULE,
};
*/
static struct config_group *f_misc_device_func_make(struct config_group *group, const char *name)
{
    return ERR_PTR(-EOPNOTSUPP);
}

static void f_misc_device_func_drop(struct config_group *group, struct config_item *item)
{
    /*
    struct misc_device *misc = container_of(to_config_group(item), struct misc_device, group);
    list_del(&misc->list);
    config_item_put(item);
    */
}

static struct configfs_group_operations f_misc_device_root_group_ops = {
    .make_group = f_misc_device_func_make,
    .drop_item  = f_misc_device_func_drop,
};

static void f_misc_device_root_release(struct config_item *item)
{
    struct f_misc_device_opts *opts = container_of(to_config_group(item), struct f_misc_device_opts, inst.group);
    usb_put_function_instance(&opts->inst);
}

static struct configfs_item_operations f_misc_device_root_item_ops = {
    .release = f_misc_device_root_release,
};

static struct configfs_attribute *f_misc_device_root_attrs[] = {
    NULL,
};

const struct config_item_type f_misc_device_root_type = {
    .ct_group_ops = &f_misc_device_root_group_ops,
    .ct_item_ops  = &f_misc_device_root_item_ops,
    .ct_attrs     = f_misc_device_root_attrs,
    .ct_owner     = THIS_MODULE,
};