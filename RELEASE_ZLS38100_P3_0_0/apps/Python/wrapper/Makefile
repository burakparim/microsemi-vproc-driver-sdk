SHELL := /bin/bash
export ROOTDIR ?= $(PWD)/../../..

include $(ROOTDIR)/Makefile.globals
include $(ROOTDIR)/config.mk

OBJS=$(INSTALL_LIB_PATH)/hbi_u.o $(INSTALL_LIB_PATH)/hbi_tw.o
INCLUDES += -I/usr/include/python2.7 
INCLUDES += -I$(ROOTDIR)/drivers/hbi/inc
INCLUDES += -I$(ROOTDIR)/include
INCLUDES += -I$(PLATFORM_DIR)/include

all:
	@if [ ! -d $(ROOTDIR)/libs ] ; then \
	    echo "Compile the SDK first"; \
	    false; \
	fi
	@PKG_OK=$$(dpkg-query -W --showformat='$${Status}\n' swig|grep "install ok installed"); \
	if [ "" == "$$PKG_OK" ]; then \
	    echo "Swig is missing, do a 'sudo apt-get install swig'"; \
	    false; \
	fi
	@PKG_OK=$$(dpkg-query -W --showformat='$${Status}\n' python-dev|grep "install ok installed"); \
	if [ "" == "$$PKG_OK" ]; then \
	    echo "Python-dev is missing, do a 'sudo apt-get install python-dev'"; \
	    false; \
	fi
	swig -o hbi_wrapper.c -python hbi.i
	gcc -c -fpic hbi_wrapper.c -o hbi_wrapper.o $(INCLUDES) $(EXTRA_CFLAGS)
	gcc -shared $(OBJS) hbi_wrapper.o $(EXTRA_CFLAGS) $(INCLUDES) -o _hbi.so
	mv *.so hbi.py $(INSTALL_LIB_PATH)
	rm -f hbi_wrapper.c hbi_wrapper.o

clean:
	rm -f $(INSTALL_LIB_PATH)/_hbi.so $(INSTALL_LIB_PATH)/hbi.py $(INSTALL_LIB_PATH)/hbi.pyc

