obj-m += mfd_dma_device.o
obj-m += mfd_dma_driver.o

KDIR:=/lib/modules/$(shell uname -r)/build

default:
	make -C $(KDIR) M=$(shell pwd) modules
clean:
	make -C $(KDIR) M=$(shell pwd) clean