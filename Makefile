TARGET = ethplc
ifndef KERNELDIR
	KERNELDIR  := /lib/modules/$(shell uname -r)/build
endif

obj-m += $(TARGET).o
ethplc-objs := ethplc.o pl360_ops.o pl360_hw.o
ccflags-y := -std=gnu99 -Wno-declaration-after-statement

all:
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) clean

install:
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) modules_install
	/sbin/depmod -A

dts:
	dtc -@ -I dts -O dtb -o $(TARGET).dtbo $(TARGET)-overlay.dts
	cp $(TARGET).dtbo /boot/overlays/
