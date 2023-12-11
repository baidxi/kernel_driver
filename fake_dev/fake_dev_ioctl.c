#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>

#include "fake_dev.h"


#define FAKE_DEVICE_DMABUF_EXPORT   _IOW('f', 0, struct fake_device_ioctl_args)

int _fake_device_ioctl(struct miscdevice *misc_dev, unsigned int cmd, unsigned long args)
{
    struct fake_device_ioctl_args arg;
    struct dma_buf *dmabuf;

    switch(cmd) {
        case FAKE_DEVICE_DMABUF_EXPORT:
            if (copy_from_user(&arg, (void *)args, sizeof(arg)))
                return -EFAULT;

            if (!fake_dev_dmabuf_type_is_support(arg.type))
                return -EINVAL;

            if (!strlen(arg.name))
                return -EINVAL;
            
            dmabuf = fake_dev_dmabuf_export(arg.name, arg.type, misc_dev);
            if (IS_ERR(dmabuf)) {
                return PTR_ERR(dmabuf);
            }

            arg.fd = dma_buf_fd(dmabuf, 0);

            if (copy_to_user((void *)args, &arg, sizeof(arg))) {
                return -EFAULT;
            }
            break;
        default:
            return -EINVAL;
    }

    return 0;
}
