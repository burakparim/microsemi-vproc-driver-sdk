include $(ROOTDIR)/Makefile.globals

SUBDIRS := kernel driver

.PHONY: all $(SUBDIRS)

all: $(SUBDIRS) 

$(SUBDIRS):
	if [ -d $@ ]; then \
	$(MAKE) -C $@ all; \
     fi

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
	
