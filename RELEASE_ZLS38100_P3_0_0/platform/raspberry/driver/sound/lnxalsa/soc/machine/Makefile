include $(ROOTDIR)/Makefile.globals
include $(ROOTDIR)/config.mk
LOCAL_PWD=$(ROOTDIR)/platform/$(PLATFORM)/driver/sound/lnxalsa/soc/machine

snd-soc-microsemi-dac-objs := microsemi-dac.o
obj-m += snd-soc-microsemi-dac.o

all:
	make -C $(KSRC) M=$(LOCAL_PWD)
	mv *.ko $(INSTALL_LIB_PATH)/

clean:
	rm -rf   *.o *.ko .*cmd *.mod.c .tmp_versions modules.order Module.symvers *.a ../*.o ../*.mod.* ../.*cmd
