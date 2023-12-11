#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>



static void mfd_dma_device_release(struct device *dev)
{

}

static struct resource mfd_dma_device_res[] = {
    DEFINE_RES_IO_NAMED(0, 10, "num_vchan"),
    DEFINE_RES_IO_NAMED(0, 2, "num_pchan"),
    DEFINE_RES_MEM(0x10000, 0x200),
    DEFINE_RES_MEM(0x20000, 0x200),
    DEFINE_RES_IRQ(10),
};

static struct platform_device mfd_dma_device = {
    .name = "mfd_dma_device",
    .dev = {
        .release = mfd_dma_device_release,
    },
    .resource = mfd_dma_device_res,
    .num_resources = ARRAY_SIZE(mfd_dma_device_res),
};

static int __init mfd_dma_device_init(void)
{
    return platform_device_register(&mfd_dma_device);
}

static void __exit mfd_dma_device_exit(void)
{
    platform_device_unregister(&mfd_dma_device);
}

module_init(mfd_dma_device_init);
module_exit(mfd_dma_device_exit);

MODULE_LICENSE("GPL");