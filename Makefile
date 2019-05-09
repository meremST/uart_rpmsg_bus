# Makefile for simple external out-of-tree Linux kernel module example
# KERNEL_SRC_PATH must be defined
# Object file(s) to be built
obj-m := uart_rpmsg_bus.o

# Path to the directory that contains the Linux kernel source code
# and the configuration file (.config)
KERNEL_DIR ?=  $(KERNEL_SRC_PATH)

# Path to the directory that contains the generated objects
DESTDIR ?= $(KERNEL_DIR)/install_artifact

# Path to the directory that contains the source file(s) to compile
PWD := $(shell pwd) 
  
default:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) INSTALL_MOD_PATH=$(DESTDIR) modules_install

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
