include $(ROOTDIR)/Makefile.globals

SUBDIRS := soc

.PHONY: all $(SUBDIRS)

all: $(SUBDIRS) 

$(SUBDIRS):
	$(MAKE) -C $@ all

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
	
