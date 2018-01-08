INSTALL_DIR ?= ../release
PWD ?= $(shell pwd)
GIT_VERSION = $(shell cd $(PWD) && git describe --always --dirty --long)
#KBUILD_EXTRA_SYMBOLS=/home/qba/rtos/wrzesien/ads1256-rtdm/Module.symvers
obj-m += ads1256-rtdm.o
ads1256-rtdm-y := ../ksrc/ads1256-rtdm.o
ccflags-y += -I$(KERNEL_DIR)/include/xenomai
ccflags-y += -DGIT_VERSION=\"$(GIT_VERSION)\"
ccflags-y += -std=gnu99

.PHONY: all build clean install

all: build install

build:
	make -C $(KERNEL_DIR) $(KBUILD_EXTRA_SYMBOLS) M=$(PWD) modules

info:
	modinfo ads1256-rtdm.ko

install:
	@mkdir -p $(INSTALL_DIR)
	cp ads1256-rtdm.ko $(INSTALL_DIR)/ads1256-rtdm.ko
	cp Module.symvers $(INSTALL_DIR)/Module.symvers


clean:
	@make -C $(KERNEL_DIR) M=$(PWD) clean
	@rm -f ../ksrc/*.o
	@rm -f ../ksrc/.*.o.*
