SHELL := /bin/bash
export ROOTDIR ?= $(PWD)/../..

## Sample apps for Microsemi VPROC Devices Host Bus Interface
## Current Makefile compiles app using user-space driver for linux
## platform.
## user can modify OBJS variable to point to platform independent ## HBI driver for non-linux platform

## These should always be included as defines Driver configuration
## parameter
include $(ROOTDIR)/Makefile.globals
include $(ROOTDIR)/config.mk

##TOOLCHAIN path for userspace build. 
##Uncomment if doing cross compilation and set to path containing toolchain
#SYSROOT=$(TOOLSPATH)/arm-linux-gnueabihf/libc

##link to hbi user space driver for linux platform
##user can modify OBJS to link platform independent 
##hbi driver objs
OBJS = $(INSTALL_LIB_PATH)/hbi_u.o $(INSTALL_LIB_PATH)/hbi_tw.o

SRC = 
##default TARGET built is hbi_test
ifneq ($(HBI_TEST),)
APPS = hbi_test
endif

ifneq ($(HBI_HELLO),)
APPS = hbi_hello
endif

INCLUDES += -I$(ROOTDIR)/drivers/hbi/inc 
INCLUDES += -I$(ROOTDIR)/apps/C/utils 
INCLUDES += -I$(ROOTDIR)/apps/C

##uncomment is you have updated SYSROOT
#LDFLAGS += --sysroot=$(SYSROOT)

ifneq ($(DEBUG),)
LDFLAGS += -g
endif

ifneq ($(HBI_LOAD_FIRMWARE),)
APPS += hbi_load_firmware
SRC = utils/app_util.c
endif

ifneq ($(HBI_LOAD_GRAMMAR),)
APPS += hbi_load_grammar
endif

.PHONY: $(APPS)
all: $(APPS)

$(APPS):
	$(CC)  $(OBJS) $(SRC) $@.c $(EXTRA_CFLAGS) $(INCLUDES) $(LDFLAGS) -o $@	

clean:
	rm -f hbi_load_firmware hbi_load_grammar hbi_test *.out *.o

help:
	@echo "TEST MACROS:"
	@echo "HBI_LOAD_FIRMWARE - builds sample app to load firmware and configuration record to device. Sample app support both dynamic and static loading of images."
	@echo "HBI_LOAD_GRAMMAR - builds sample app to load ASR grammar to device. "
	@echo "To build static firmware, define LOAD_FWR_STATIC, and set FWR to filename included for compilation."
	@echo "To build configuration record, define LOAD_CFGREC_STATIC and set CFGREC to C-file of configuration  in apps/makefile"
		
