include $(ROOTDIR)/Makefile.globals
include $(ROOTDIR)/config.mk


OBJS := hbi_u.o

ifeq ($(TARGET),TW)
	OBJS += hbi_tw.o
endif

CFLAGS += $(EXTRA_CFLAGS)
CFLAGS += -I$(ROOTDIR)/include \
	-I$(ROOTDIR)/drivers/hbi/inc \
	-I$(ROOTDIR)/lnxdrivers/lnxhbi/lnxkernel/inc

.PHONY: all $(SRC)

all: $(OBJS)
	mv *.o $(INSTALL_LIB_PATH)/

clean:
	rm -rf   *.o 
