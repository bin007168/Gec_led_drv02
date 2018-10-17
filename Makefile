

obj-m = my_cdev.o

CROSS_COMPILE:=/home/bin1/6818GEC/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi-

KDIR = /home/bin1/6818GEC/kernel

default:

	$(MAKE) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)  -C $(KDIR) M=$$PWD 

clean:
	rm -rf modules.order *.o Module.* *.mod.* *.ko


