#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#define FW_VER      0xa6

struct cst128_device {
    struct i2c_adapter *adapter;
    struct device *dev;
    struct input_dev *input_device;
    int max_x,max_y;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *irq_gpio;
    struct work_struct work;
    struct workqueue_struct *wq;
    int irq;
    struct device_node *np;
    bool is_open;
	struct regmap *regmap;
    bool swap_xy;
};

#define MAX_CONTACTS 5

static const struct of_device_id hyn_ts_ids[] = {
    {.compatible = "hyn,cst128a"},
    {},
};

static irqreturn_t cst128_irq_handler(int irq, void *data)
{
    struct cst128_device *cst128 = data;
    if (!cst128->is_open)
        return IRQ_NONE;

    if (!work_pending(&cst128->work))
    {
        queue_work(cst128->wq, &cst128->work);
    }

    return IRQ_HANDLED;
}


static void cst128_worker(struct work_struct *work)
{
	struct cst128_device *cst128 = container_of(work, struct cst128_device, work);
	u8 buf[0x3c] = {};
	int err,num_touch, i;
	int x, y, id, /*pressure = 200 ,*/ event;

	if (!cst128->is_open)
		return;

	err = regmap_bulk_read(cst128->regmap, 0x2, buf, 0x3c);
	if (err) {
		dev_err(cst128->dev, "read reg failed:%d\n", err);
		return;
	}

	num_touch = buf[0] & 0xf;
	for (i = 0; i < num_touch; i++) {
        x = (buf[1 + 6 * i] & 0xf) << 8 | buf[2 + 6 * i];
        y = (buf[3 + 6 * i] & 0xf) << 8 | buf[4 + 6 * i];
        id = (buf[3 + 6 * i]) >> 4;
        event = (buf[1 + 6 * i]) >> 6;
        input_mt_slot(cst128->input_device, id);
        input_mt_report_slot_state(cst128->input_device, MT_TOOL_FINGER, 
								   event == 1 ? false : true);

        if (cst128->swap_xy) {
            input_report_abs(cst128->input_device, ABS_MT_POSITION_X, y);
            input_report_abs(cst128->input_device, ABS_MT_POSITION_Y, x);
        } else {
            input_report_abs(cst128->input_device, ABS_MT_POSITION_X, x);
            input_report_abs(cst128->input_device, ABS_MT_POSITION_Y, y);
        }

        input_report_abs(cst128->input_device, ABS_MT_WIDTH_MAJOR, 1);
        input_report_abs(cst128->input_device, ABS_MT_TRACKING_ID, id);
	}

    input_mt_report_pointer_emulation(cst128->input_device, true);
    input_sync(cst128->input_device);
}

static int cst128_device_open(struct input_dev *dev)
{
    struct cst128_device *cst128 = input_get_drvdata(dev);
    if (cst128->is_open)
        return -EBUSY;

    cst128->is_open = true;
    return 0;
}

static void cst128_device_close(struct input_dev *dev)
{
    struct cst128_device *cst128 = input_get_drvdata(dev);
    if (cst128->is_open) {
        cst128->is_open = false;
    }
}

static bool cst128a_regmap_reg_writeable(struct device *dev, unsigned int reg)
{
	switch(reg) {
		case 0:
		case 0x80 ... 0xa0:
		case 0xaa ... 0xaf:
			break;
		default:
			return false;
	};

	return true;
}

static bool cst128a_regmap_reg_readable(struct device *dev, unsigned int reg)
{
	switch(reg) {
		case 0x0 ... 0x3e:
		case 0x80 ... 0xff:
			break;
		default:
			return false;
	}

	return true;
}

const struct regmap_config cst128a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = cst128a_regmap_reg_writeable,
	.readable_reg = cst128a_regmap_reg_readable,
};

static int hyn_cst128_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct cst128_device *cst128;
    struct input_dev *input_device;
    struct device *dev = &client->dev;
	struct regmap *regmap;
    u32 fw_ver;
    int err;
    int irq;

	regmap = devm_regmap_init_i2c(client, &cst128a_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "regmap init failed:%d\n", err);
		return PTR_ERR(regmap);
	}

    err = regmap_read(regmap, FW_VER, &fw_ver);
    if (err) {
        dev_err(dev, "get firmware version error:%d\n", err);
        return err;
    }

    dev_info(dev, "FW VER:%d\n", fw_ver);

    cst128 = devm_kzalloc(dev, sizeof(*cst128), GFP_KERNEL);

    if (!cst128)
        return -ENOMEM;

    cst128->adapter = client->adapter;
    cst128->dev = dev;
    cst128->np = dev->of_node;
	cst128->regmap = regmap;

    cst128->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(cst128->reset_gpio)) {
        err = PTR_ERR(cst128->reset_gpio);
        dev_err(dev, "get reset GPIO failed:%d\n", err);
        goto err_out;
    }

    cst128->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
    if (IS_ERR(cst128->irq_gpio)) {
        err = PTR_ERR(cst128->irq_gpio);
        dev_err(dev, "get irq GPIO failed:%d\n", err);
        goto err_out;
    }

    if (of_property_read_u32(cst128->np, "max_x", &cst128->max_x)) {
        dev_err(cst128->dev, "max_x property not set\n");
        err = -EINVAL;
        goto err_out;
    }

    if (of_property_read_u32(cst128->np, "max_y", &cst128->max_y)) {
        dev_err(dev, "max_y property not set\n");
        err = -EINVAL;
        goto err_out;
    }

    cst128->swap_xy = of_property_read_bool(cst128->np, "xy_swap");

    gpiod_direction_output(cst128->reset_gpio, 1);
    msleep(100);
    gpiod_direction_output(cst128->reset_gpio, 0);

    input_device = devm_input_allocate_device(dev);
    if (!input_device) {
        dev_err(dev, "allocate input device failed\n");
        err = -ENOMEM;
        goto err_out;
    }
    
    input_device->name = "cst128a";
    input_device->id.bustype = BUS_I2C;
    input_device->dev.parent = cst128->dev;
    input_device->open = cst128_device_open;
    input_device->close = cst128_device_close;

    input_set_drvdata(input_device, cst128);

    input_set_abs_params(input_device, ABS_MT_TRACKING_ID, 0, (MAX_CONTACTS + 1), 0, 0);
    set_bit(EV_ABS, input_device->evbit);
    set_bit(EV_KEY, input_device->evbit);
    __set_bit(INPUT_PROP_DIRECT, input_device->propbit);
    input_device->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    set_bit(ABS_MT_POSITION_X, input_device->absbit);
    set_bit(ABS_MT_POSITION_Y, input_device->absbit);
    set_bit(ABS_MT_TOUCH_MAJOR, input_device->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, input_device->absbit);

    input_set_abs_params(input_device, ABS_MT_POSITION_X, 0, cst128->max_x, 0, 0);
    input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0, cst128->max_y, 0, 0);
    input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
    input_set_abs_params(input_device, ABS_MT_TRACKING_ID, 0, MAX_CONTACTS, 0, 0);

    err = input_mt_init_slots(input_device, MAX_CONTACTS, 0);
    if (err) {
        dev_err(dev, "init slot error:%d\n", err);
        goto err_out;
    }

    cst128->wq = create_singlethread_workqueue("cst128a_ts");
    if (!cst128->wq) {
        dev_err(dev, "Could not create workqueue\n");
        err = -ENOMEM;
        goto err_out;
    }

    flush_workqueue(cst128->wq);
    INIT_WORK(&cst128->work, cst128_worker);

    err = input_register_device(input_device);
    if (err) {
        dev_err(dev, "register cst128a failed:%d\n", err);
        goto destroy_workqueue;
    }

    cst128->input_device = input_device;

    irq = gpiod_to_irq(cst128->irq_gpio);
    if (irq < 0) {
        dev_err(cst128->dev, "gpio to IRQ failed:%d\n", irq);
        err = irq;
        goto destroy_workqueue;
    }

    err = devm_request_irq(&input_device->dev, irq, cst128_irq_handler, IRQF_SHARED | IRQF_ONESHOT | IRQF_TRIGGER_RISING, "hyn_ts", cst128);
    if (err) {
        dev_err(cst128->dev, "request IRQ failed:%d\n", err);
        goto destroy_workqueue;
    }

    cst128->irq = irq;

    i2c_set_clientdata(client, cst128);

    return 0;
destroy_workqueue:
    destroy_workqueue(cst128->wq);
err_out:
    return err;
}

static int hyn_cst128_remove(struct i2c_client *client)
{
    struct cst128_device *cst128 = i2c_get_clientdata(client);
    cancel_work_sync(&cst128->work);
    destroy_workqueue(cst128->wq);
    return 0;
}


static struct i2c_driver hyn_cst128_drv = {
    .driver = {
        .name = "cst128a",
        .of_match_table = of_match_ptr(hyn_ts_ids),
    },
    .probe = hyn_cst128_probe,
    .remove = hyn_cst128_remove,
};

module_i2c_driver(hyn_cst128_drv);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("juno baidxi404629@gmail.com");


