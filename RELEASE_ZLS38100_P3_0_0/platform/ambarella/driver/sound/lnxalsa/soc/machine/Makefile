include $(ROOTDIR)/Makefile.globals
include $(ROOTDIR)/config.mk
LOCAL_PWD=$(PLATFORM_DIR)/driver/sound/lnxalsa/soc/machine

snd-soc-s2l-objs := s2l_zl380xx_audio.o

obj-m	+= snd-soc-s2l.o
#
# Makefile for I2S Controller driver under slave mode 
#

all:
	$(MAKE) -C $(KSRC) M=$(LOCAL_PWD)
	mv *.ko $(INSTALL_LIB_PATH)/

clean:
	rm -rf   *.o *.ko .*cmd *.mod.c .tmp_versions modules.order Module.symvers
