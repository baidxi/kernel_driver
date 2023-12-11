#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/device/class.h>
#include <net/net_namespace.h>
#include <linux/notifier.h>
#include <net/netns/generic.h>
#include <linux/sysfs.h>
#include <linux/skbuff.h>
#include <linux/circ_buf.h>
#include <asm/barrier.h>
#include <linux/task_work.h>
#include <linux/dma-buf.h>
#include <linux/etherdevice.h>
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/stat.h>

#define RING_BUF_SIZE   128

struct benchmark_ringbuf {
    struct sk_buff *buf[RING_BUF_SIZE];
    unsigned long head;
    unsigned long tail;
    unsigned long size;
};

enum benchmark_pkt_type {
    BM_PKT_TYPE_TCP = 0,
    BM_PKT_TYPE_UDP,
    BM_PKT_TYPE_ICMP,
    BM_PKT_TYPE_FILE,
};

enum benchmark_pkt_source {
    BM_PKT_SRC_FILE = 0,
    BM_PKT_SRC_DEVICE,
    BM_PKT_SRC_RANDOM,
};

enum benchmark_pkt_mode {
    BM_PKT_MODE_ONCE,
    BM_PKT_MODE_CONTINUE,
};

const char *benchmark_pkt_type_str[] = {
    "tcp",
    "udp",
    "icmp",
    "from file",
};

const char *benchmark_pkt_mode_str[] = {
    "once",
    "continue",
};

const char *benchmark_ptk_source_str[] = {
    "file",
    "device",
    "random",
};

struct benchmark_device {
    struct net_device *out_dev;
    struct device *device;
    struct class *cls;
    struct cdev cdev;
    dev_t dev;
    size_t buf_size;
    struct net *net;
    bool idle;
    unsigned int id;
    int is_open;
    void *buf;
    dma_addr_t dma_addr;
    struct benchmark_ringbuf ring_buf;
    struct tasklet_struct tasklet;
    enum benchmark_pkt_source src;
    enum benchmark_pkt_type type;
    enum benchmark_pkt_mode mode;
    bool zerocopy;
    char *file_path;
    struct file *file;
    char *dev_name;
    bool running;
    int (*start)(struct benchmark_device *);
    int num_thread;
    int n_repeat;
};

struct benchmark_data {
    struct benchmark_device *bm_dev;
    int cpu;
};

static DEFINE_PER_CPU(struct work_struct, benchmark_works);
DEFINE_PER_CPU_ALIGNED(struct benchmark_data, benchmark_data);

static void bm_dev_tasklet_func(struct tasklet_struct *t)
{
    struct benchmark_device *bm_dev = container_of(t, struct benchmark_device, tasklet);
    struct benchmark_ringbuf *buf = &bm_dev->ring_buf;
    struct net_device *dev = bm_dev->out_dev;
    unsigned long head = smp_load_acquire(&buf->head);
    unsigned long tail = buf->tail;

    if (!dev)
        return;

    if (CIRC_CNT(head, tail, buf->size) >= 1) {
        struct sk_buff *skb = buf->buf[tail];

        if (dev->netdev_ops->ndo_start_xmit)
            dev->netdev_ops->ndo_start_xmit(skb, dev);

        smp_store_release(&buf->tail, (tail + 1) & (buf->size -1));
    }
}

static void pkt_close_file(struct benchmark_device *bm_dev)
{
    filp_close(bm_dev->file, 0);
    bm_dev->file = NULL;
    kfree(bm_dev->file_path);
    bm_dev->file_path = NULL;
}

static int bm_dev_open(struct inode *inode, struct file *filp)
{
    struct benchmark_device *bm_dev = container_of(inode->i_cdev, struct benchmark_device, cdev);

    if (bm_dev->is_open > 0) {
        return -EBUSY;
    }

    bm_dev->is_open++;
    bm_dev->ring_buf.head = 0;
    bm_dev->ring_buf.tail = 0;
    bm_dev->ring_buf.size = RING_BUF_SIZE;

    tasklet_setup(&bm_dev->tasklet, bm_dev_tasklet_func);

    filp->private_data = bm_dev;

    return 0;
}

static int bm_dev_release(struct inode *inode, struct file *filp)
{
    struct benchmark_device *bm_dev = filp->private_data;
    struct benchmark_ringbuf *ringbuf = &bm_dev->ring_buf;

    bm_dev->is_open--;

    filp->private_data = NULL;

    while(CIRC_CNT(ringbuf->head, ringbuf->head, RING_BUF_SIZE)) {

    }

    return 0;
}

static __poll_t bm_dev_poll(struct file *filp, struct poll_table_struct *wait_table)
{
    return 0;
}

static ssize_t bm_dev_write(struct file *filp, const char __user *user, size_t size, loff_t *off)
{
    struct benchmark_device *bm_dev = filp->private_data;
    struct benchmark_ringbuf *buf = &bm_dev->ring_buf;
    unsigned long head = buf->head;
    unsigned long tail = READ_ONCE(buf->tail);
    struct sk_buff *skb;
    int ret;

    if (!bm_dev->out_dev)
        return -ENOENT;

    if (size > bm_dev->out_dev->mtu) {
        return -ERANGE;
    }

    if (bm_dev->src != BM_PKT_SRC_DEVICE)
        return -EINVAL;

    if(CIRC_SPACE(head, tail, buf->size) >= 1) {

        skb = netdev_alloc_skb(bm_dev->out_dev, size);
        if (!skb)
            return -ENOMEM;

        ret = copy_from_user(skb_put(skb, size), user, size);
        if (ret) {
            dev_kfree_skb_any(skb);
            return -EFAULT;
        }

        skb->protocol = eth_type_trans(skb, bm_dev->out_dev);

        buf->buf[head] = skb;

        smp_store_release(&buf->head, (head + 1) & (buf->size -1));

        tasklet_schedule(&bm_dev->tasklet);
    }

    return size;
}

static int bm_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct benchmark_device *bm_dev = filp->private_data;
    size_t size = vma->vm_end - vma->vm_start;
    unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
    unsigned long pfn_start;
    unsigned long virt_start;
    int ret;

    if (!size)
        return -EINVAL;

    if (!bm_dev->out_dev)
        return -ENOENT;

    bm_dev->buf = kvmalloc(size, GFP_KERNEL);

    if (!bm_dev->buf)
        return -ENOMEM;

    pfn_start = (virt_to_phys(bm_dev->buf) >> PAGE_SHIFT) + vma->vm_pgoff;
    virt_start =(unsigned long) bm_dev->buf + offset;

    ret = remap_pfn_range(vma, vma->vm_start, pfn_start, size, vma->vm_page_prot);

    if (ret) {
        dev_err(bm_dev->device, "FAILED to remap_pfn_range at [0x%lx 0x%lx]\n", vma->vm_start, vma->vm_end);
        kfree(bm_dev->buf);
        return ret;
    }

    dev_info(bm_dev->device, "map 0x%lx to 0x%lx, size 0x%x\n", virt_start, vma->vm_start, size);

    return 0;
}

static const struct file_operations bm_dev_ops = {
    .open = bm_dev_open,
    .release = bm_dev_release,
    .poll = bm_dev_poll,
    .write = bm_dev_write,
    .mmap = bm_dev_mmap,
};

static ssize_t bm_device_out_dev_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);

    if (!bm_dev->out_dev) {
        return sprintf(buf, "none\n");
    }

    return snprintf(buf, IFNAMSIZ, "%s\n", bm_dev->dev_name);
}

static ssize_t bm_device_out_dev_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    struct net_device *netdev;
    char *name;

    if (size > IFNAMSIZ)
        return -ERANGE;

    name = kmemdup(buf, size, GFP_KERNEL);

    name[size-1] = '\0';

    netdev = dev_get_by_name(bm_dev->net, name);

    if (!netdev) {
        dev_err(dev, "%s not found\n", name);
        kfree(name);
        return -ENOENT;
    }

    if (bm_dev->dev_name)
        kfree(bm_dev->dev_name);

    bm_dev->dev_name = name;

    if (bm_dev->out_dev) {
        dev_put(bm_dev->out_dev);
    }

    bm_dev->out_dev = netdev;

    return size;
}

static ssize_t bm_device_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EOPNOTSUPP;
}

static ssize_t bm_device_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 0, &val);

    if (ret)
        return ret;

    if (val) {
        bm_dev->buf_size = 0;

        if (bm_dev->out_dev) {
            dev_put(bm_dev->out_dev);
            bm_dev->out_dev = NULL;
            kfree(bm_dev->dev_name);
            bm_dev->dev_name = NULL;
        }

        if (bm_dev->src == BM_PKT_SRC_FILE) {
            pkt_close_file(bm_dev);
            bm_dev->src = BM_PKT_SRC_RANDOM;
            bm_dev->type = BM_PKT_TYPE_TCP;
        }
    }

    return size;
}

static ssize_t bm_device_bufsize_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE-1, "%d\n", bm_dev->buf_size);
}

static ssize_t bm_device_bufsize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    if (!buf || !size)
        return -EINVAL;

    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;

    bm_dev->buf_size = val;

    return size;
}

static ssize_t bm_device_pkt_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", benchmark_pkt_type_str[bm_dev->type]);
}

static ssize_t bm_device_pkt_type_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    int i;

    for (i = 0; i < ARRAY_SIZE(benchmark_pkt_type_str); i++) {
        if (strcmp(buf, benchmark_pkt_type_str[i]) == 0) {
            bm_dev->type = i;
            return size;
        }
    }

    return -EOPNOTSUPP;
}

static ssize_t pkt_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);

    switch(bm_dev->src) {
        case BM_PKT_SRC_RANDOM:
        case BM_PKT_SRC_DEVICE:
            return sprintf(buf, "%s\n", benchmark_ptk_source_str[bm_dev->src]);
        default:
            return sprintf(buf, "%s\n", bm_dev->file_path);
    }
}

static ssize_t pkt_file_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    struct file *file;
    char *path;
    int ret;

    if (buf[0] != '/' && size < 6)
        return -EINVAL;

    if (buf[0] != '/' && strcmp(buf, "random") == 0) {
        bm_dev->src = BM_PKT_SRC_RANDOM;
        if (bm_dev->file) {
            pkt_close_file(bm_dev);
        }
    } else if (buf[0] == '/') {
        path = kstrndup(buf, size-1, GFP_KERNEL);
        if (!path)
            return -ENOMEM;

        if (bm_dev->file) {
            pkt_close_file(bm_dev);
        }
        file = filp_open(path, O_RDONLY, 0);
        if (IS_ERR(bm_dev->file)) {
            ret = PTR_ERR(bm_dev->file);
            bm_dev->file = NULL;
            return ret;
        }

        if (strncmp(buf, "/dev", 4) == 0) {
            bm_dev->src = BM_PKT_SRC_DEVICE;
        } else {
            bm_dev->src = BM_PKT_SRC_FILE;
        }

        if (!file->f_path.dentry) {
            dev_err(bm_dev->device, "open %s ERROR\n", path);
            kfree(path);
            return -EFAULT;
        }

        bm_dev->file_path = path;
        bm_dev->file = file;
        bm_dev->type = BM_PKT_TYPE_FILE;
        bm_dev->n_repeat = 1;
    } else {
        return -EINVAL;
    }

    return size;
}

static ssize_t bm_device_pkt_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", benchmark_pkt_mode_str[bm_dev->mode]);
}

static ssize_t bm_device_pkt_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    int i;

    for (i = 0; i < ARRAY_SIZE(benchmark_pkt_mode_str); i++) {
        if (strcmp(buf, benchmark_pkt_mode_str[i]) == 0) {
            bm_dev->mode = i;
            return size;
        }
    }
    return -EOPNOTSUPP;
}

static ssize_t bm_device_pkt_start_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", bm_dev->running ? "true" : "false");
}

static ssize_t bm_device_pkt_start_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;

    if (val) {
        bm_dev->running = true;
        ret = bm_dev->start(bm_dev);
        if (ret)
            return ret;
    }

    return size;
}

static ssize_t bm_device_num_threads_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", bm_dev->num_thread);
}

static ssize_t bm_device_num_threads_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;

    bm_dev->num_thread = val;

    return size;
}

static ssize_t bm_device_n_repeat_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", bm_dev->n_repeat);
}

static ssize_t bm_device_n_repeat_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct benchmark_device *bm_dev = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;

    if (val) {
        bm_dev->n_repeat = val;
    }

    return size;
}

DEVICE_ATTR(n_repeat, 0644, bm_device_n_repeat_show, bm_device_n_repeat_store);
DEVICE_ATTR(num_threads, 0644, bm_device_num_threads_show, bm_device_num_threads_store);
DEVICE_ATTR(pkt_start, 0644, bm_device_pkt_start_show, bm_device_pkt_start_store);
DEVICE_ATTR(pkt_mode, 0644, bm_device_pkt_mode_show, bm_device_pkt_mode_store);
DEVICE_ATTR_WO(pkt_file);
DEVICE_ATTR_RO(pkt_source);
DEVICE_ATTR(pkt_type, 0644, bm_device_pkt_type_show, bm_device_pkt_type_store);
DEVICE_ATTR(bufsize, 0644, bm_device_bufsize_show, bm_device_bufsize_store);
DEVICE_ATTR(reset, 0644, bm_device_reset_show, bm_device_reset_store);
DEVICE_ATTR(out_dev, 0644, bm_device_out_dev_show, bm_device_out_dev_store);

static struct attribute *bm_device_attrs[] = {
    &dev_attr_out_dev.attr,
    &dev_attr_reset.attr,
    &dev_attr_bufsize.attr,
    &dev_attr_pkt_type.attr,
    &dev_attr_pkt_source.attr,
    &dev_attr_pkt_file.attr,
    &dev_attr_pkt_mode.attr,
    &dev_attr_pkt_start.attr,
    &dev_attr_num_threads.attr,
    &dev_attr_n_repeat.attr,
    NULL,
};
ATTRIBUTE_GROUPS(bm_device);

static unsigned int bm_mark_id __read_mostly;

static int bm_device_start(struct benchmark_device *bm_dev)
{
    int i;
    static cpumask_t cpus;
    if (!bm_dev->out_dev)
        return -ENOENT;

    if ((bm_dev->src == BM_PKT_SRC_FILE || bm_dev->src == BM_PKT_SRC_DEVICE) && !bm_dev->file)
        return -EINVAL;

    cpus_read_lock();

    cpumask_clear(&cpus);

    if (bm_dev->src == BM_PKT_SRC_RANDOM) {
        for_each_online_cpu(i) {
            queue_work_on(i, system_highpri_wq, per_cpu_ptr(&benchmark_works, i));
            cpumask_set_cpu(i, &cpus);
        }
    } else if (bm_dev->src == BM_PKT_SRC_FILE) {
        queue_work(system_highpri_wq, this_cpu_ptr(&benchmark_works));
    }

    cpus_read_unlock();

    return 0;
}

static int benchmark_send_buf(struct net_device *dev, char *buf, size_t size)
{
    struct sk_buff *skb = netdev_alloc_skb(dev, size);
    int ret;

    if (!skb)
        return -ENOMEM;

    memcpy(skb_put(skb, size), buf, size);

    ret = dev->netdev_ops->ndo_start_xmit(skb, dev);

    if (ret) {
        dev_kfree_skb_any(skb);
    }

    return ret;
}

static void benchmark_from_file(struct benchmark_device *bm_dev)
{
    struct kstat stat;
    struct file *file;
    struct net_device *netdev = bm_dev->out_dev;
    int romain = 0;
    char *buf;
    int ret;
    loff_t off;
    size_t size;

    if (!bm_dev->file)
        return;
    
    file = bm_dev->file;

    ret = vfs_getattr(&file->f_path, &stat, STATX_BASIC_STATS, 0);

    if (ret) {
        dev_err(bm_dev->device, "FAILED to stat %s:%d\n", bm_dev->file_path, ret);
        return;
    }

    buf = kzalloc(netdev->mtu, GFP_KERNEL);
    if (!buf) {
        dev_err(bm_dev->device, "FAILED to out of memory\n");
        return;
    }

    while((bm_dev->n_repeat || bm_dev->mode == BM_PKT_MODE_CONTINUE) && bm_dev->running) {
        u32 loop, i;
        romain = stat.size;
        off = 0;
        loop = romain / netdev->mtu;

        for (i = 0; i < loop; i++) {
            size = kernel_read(bm_dev->file, buf, netdev->mtu, &off);
            if (size <= 0) {
                dev_err(bm_dev->device, "FAILED to read %s offset %lld errno:%d\n", bm_dev->file_path, off, size);
                break;
            }
            romain -= size;
            off += size;
            ret = benchmark_send_buf(netdev, buf, size);
            if (ret) {
                dev_err(bm_dev->device, "FAILED to send buf:%d\n", ret);
                goto out;
            }
        }

        if (romain > 0 && romain < netdev->mtu) {
            size = kernel_read(bm_dev->file, buf, romain, loop ? &off : NULL);
            if (size <= 0) {
                dev_err(bm_dev->device, "FAILED to read %s:%d\n", bm_dev->file_path, size);
                goto out;
            }

            ret = benchmark_send_buf(netdev, buf, size);
            if (ret) {
                dev_err(bm_dev->device, "FAILED to send buf:%d\n", ret);
                goto out;
            }
        }

        if (bm_dev->mode != BM_PKT_MODE_CONTINUE)
            bm_dev->n_repeat--;
    }

out:
    kfree(buf);
}

static void benchmark_from_random(struct benchmark_device *bm_dev)
{

}

static void benchmark_device_works(struct work_struct *work)
{
    struct benchmark_data *bmd;
    struct benchmark_device *bm_dev;

    local_bh_disable();
    bmd = this_cpu_ptr(&benchmark_data);
    bm_dev = bmd->bm_dev;
    
    if (bm_dev->src == BM_PKT_SRC_RANDOM)
        benchmark_from_random(bm_dev);
    else
        benchmark_from_file(bm_dev);

    local_bh_enable();
}

static int __net_init bm_net_init(struct net *net)
{
    int ret = 0;
    int cpu;
    struct benchmark_device *bm_dev = net_generic(net, bm_mark_id);

    pr_info("bh mark net init\n");

    bm_dev->net = net;
    bm_dev->start = bm_device_start;

    bm_dev->cls = class_create(THIS_MODULE, "bm_dev");
    if (IS_ERR(bm_dev->cls)) {
        ret = PTR_ERR(bm_dev->cls);
        pr_err("FAILED to init bh mark net:%d\n", ret);
        goto err_out;
    }

    ret = alloc_chrdev_region(&bm_dev->dev, 0, 1, "benchmark");
    if (ret) {
        pr_err("FAILED to alloc_chrdev_region:%d\n", ret);
        goto destroy_class;
    }

    cdev_init(&bm_dev->cdev, &bm_dev_ops);
    cdev_add(&bm_dev->cdev, bm_dev->dev, 1);

    bm_dev->device = device_create_with_groups(bm_dev->cls, NULL, bm_dev->dev, bm_dev, bm_device_groups, "bm_device%d", MINOR(bm_dev->dev));
    if (IS_ERR(bm_dev->device)) {
        ret = PTR_ERR(bm_dev->device);
        pr_err("FAILED to create bm device:%d\n", ret);
        goto del_cdev;
    }

    bm_dev->id = bm_mark_id;
    bm_dev->src = BM_PKT_SRC_RANDOM;

    for_each_possible_cpu(cpu) {
        struct work_struct *work = per_cpu_ptr(&benchmark_works, cpu);
        struct benchmark_data *bmd = &per_cpu(benchmark_data, cpu);

        INIT_WORK(work, benchmark_device_works);

        bmd->bm_dev = bm_dev;
        bmd->cpu = cpu;
    }

    return 0;
del_cdev:
    cdev_del(&bm_dev->cdev);
destroy_class:
    class_destroy(bm_dev->cls);
err_out:
    return ret;
}

static void __net_exit bm_net_exit(struct net *net)
{
    struct benchmark_device *bm_dev = net_generic(net, bm_mark_id);

    device_destroy(bm_dev->cls, bm_dev->dev);
    cdev_del(&bm_dev->cdev);
	unregister_chrdev_region(bm_dev->dev, 1);
	class_destroy(bm_dev->cls);

    if (bm_dev->buf && bm_dev->out_dev) {

    }

    if (bm_dev->file) {
        pkt_close_file(bm_dev);
    }
}

static int bm_device_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
    struct net_device *netdev = netdev_notifier_info_to_dev(ptr);
    struct benchmark_device *bm_dev = net_generic(dev_net(netdev), bm_mark_id);

    if (!bm_dev->out_dev)
        return NOTIFY_DONE;

    switch(event) {
        case NETDEV_CHANGE:
            break;
        case NETDEV_UNREGISTER:
            if (netdev == bm_dev->out_dev) {
                tasklet_kill(&bm_dev->tasklet);
                dev_put(bm_dev->out_dev);
                bm_dev->out_dev = NULL;
            }
            break;
    }
    return NOTIFY_DONE;
}

static struct notifier_block benchmark_notifier_block = {
    .notifier_call = bm_device_event,
};

static struct pernet_operations benchmark_ns_ops = {
    .init = bm_net_init,
    .exit = bm_net_exit,
    .id = &bm_mark_id,
    .size = sizeof(struct benchmark_device),
};

static __init int benchmark_init(void)
{
    int ret = 0;
    pr_info("network test benchmark init\n");
    ret = register_pernet_subsys(&benchmark_ns_ops);
    if (ret)
        return ret;

    ret = register_netdevice_notifier(&benchmark_notifier_block);
    if (ret)
        unregister_pernet_subsys(&benchmark_ns_ops);

    return ret;
}

static __exit void benchmark_exit(void)
{
    unregister_netdevice_notifier(&benchmark_notifier_block);
    unregister_pernet_subsys(&benchmark_ns_ops);
}

module_init(benchmark_init);
module_exit(benchmark_exit);

MODULE_LICENSE("GPL");
