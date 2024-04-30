fake_dev-y += fake_dev_core.o fake_dev_dmabuf.o fake_dev_ioctl.o fake_dev_attrs.o
obj-m += fake_dev.o

KDIR:=/lib/modules/$(shell uname -r)/build

default:
	make -C $(KDIR) M=$(shell pwd) modules
clean:
	make -C $(KDIR) M=$(shell pwd) clean