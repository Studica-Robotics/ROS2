TOPTARGETS := all clean run

# Don't run the remote tests
DISCLUDED_TARGETS := remote/.

SUBDIRS := $(filter-out $(DISCLUDED_TARGETS),$(wildcard */.))

$(TOPTARGETS): $(SUBDIRS)

$(SUBDIRS):
	make -C $@ $(MAKECMDGOALS)

# Define the clean target
clean:
	@for dir in $(SUBDIRS); do \
		make -C $$dir clean; \
	done

.PHONY: $(TOPTARGETS) $(SUBDIRS) clean
