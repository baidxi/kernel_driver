#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/dmaengine.h>

struct mfd_dma_vchan {
    struct dma_chan chan;
    void __iomem *base;
};

struct mfd_dma_vchan_ring {
    struct mfd_dma_vchan *curr;
    struct mfd_dma_vchan *tail;
    struct mfd_dma_vchan *end;
};

struct mfd_dma_pchan {
    void __iomem *base;
    int valid;
    struct mfd_dma_vchan_ring ring;
};

struct mfd_dma_device {
    struct device *dev;
    struct dma_device slave;
    int num_pchan;
    int num_vchan;
    struct mfd_dma_vchan *vchan;
    struct mfd_dma_pchan *pchan;
};

#define VCHAN_OFFSET    0x8

static int mfd_dma_probe(struct platform_device *pdev)
{
    struct mfd_dma_device *priv;
    struct resource *res;
    struct mfd_dma_vchan_ring *ring;
    struct mfd_dma_vchan *vchan;
    int i, j;

    priv = kzalloc(sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    priv->dev = &pdev->dev;

    platform_set_drvdata(pdev, priv);

    res = platform_get_resource_byname(pdev, IORESOURCE_IO, "num_vchan");
    if (!res) {
        dev_err(&pdev->dev, "vchan err\n");
        goto chan_err;
    }

    priv->num_vchan = resource_size(res);

    res = platform_get_resource_byname(pdev, IORESOURCE_IO, "num_pchan");
    if (!res) {
        dev_err(&pdev->dev, "pchan err\n");
        goto chan_err;
    }

    priv->num_pchan = resource_size(res);

    priv->pchan = kcalloc(priv->num_pchan, sizeof(*priv->pchan), GFP_KERNEL);
    if (!priv->pchan) {
        dev_err(&pdev->dev, "alloc pchan err\n");
        goto chan_err;
    }

    priv->vchan = kcalloc(priv->num_vchan * 2, sizeof(*priv->vchan), GFP_KERNEL);
    if (!priv->vchan) {
        dev_err(&pdev->dev, "alloc vchan err\n");
        goto free_pchan;
    }

    vchan = priv->vchan;

    for (i = 0; i < priv->num_pchan; i++) {
        priv->pchan[i].base = devm_platform_ioremap_resource(pdev, i);
    
        if (!priv->pchan[i].base) {
            dev_err(&pdev->dev, "ioremap err\n");
            goto free_vchan;
        }

        for (j = 0; j < priv->num_vchan; j++) {
            vchan->base = priv->pchan[i].base + VCHAN_OFFSET;
            vchan->chan.device = &priv->slave;
            list_add_tail(&vchan->chan.device_node, &priv->slave.channels);
            vchan++;
        }
    }

    return 0;

free_vchan:
    kfree(priv->vchan);
free_pchan:
    kfree(priv->pchan);
chan_err:
    kfree(priv);
    return -ENOMEM;
}

static int mfd_dma_remove(struct platform_device *pdev)
{
    struct mfd_dma_device *priv = platform_get_drvdata(pdev);

    kfree(priv->pchan);
    kfree(priv->vchan);

    kfree(priv);


    return 0;
}

static struct platform_driver mfd_dma_drv = {
    .probe = mfd_dma_probe,
    .remove = mfd_dma_remove,
    .driver = {
        .name = "mfd_dma_device",
    }
};

module_platform_driver(mfd_dma_drv);
MODULE_LICENSE("GPL");