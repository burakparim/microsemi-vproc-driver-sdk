include $(ROOTDIR)/Makefile.globals

SUBDIRS := dts

.PHONY: all $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@ all

ifneq ($(CROSS_COMPILE),)
CROSSCOMPILE:
	@echo "Cross Compile"
	$(MAKE) -f Makefile.crosscompile
else
CROSSCOMPILE:
endif

all:CROSSCOMPILE $(SUBDIRS)

clean:
	@( \
	 for dir in $(SUBDIRS);do \
	 DIR=$${dir}/; \
	 echo "Cleaning DIR="$$DIR; \
	 $(MAKE) -C $$DIR clean; \
	 done	\
	)

help:
	@echo "SHELL="$(SHELL) 
	@echo "ROOTDIR="$(ROOTDIR) 
	@echo "SUBDIRS="$(SUBDIRS) 
	

