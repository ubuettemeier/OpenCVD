
.PHONY: clean

val=

ifdef cv4
val=cv4=1
endif

SAMPLE_PATH = basic/ edge/ filtering/ findcontours/ imread/ short_sample/ demhist/ HoughCircles/ HoughLines/

all:	
	for dir in $(SAMPLE_PATH); do \
	$(MAKE) $(val) -C $$dir; \
	echo "-------------"; \
	done

clean:
	for dir in $(SAMPLE_PATH); do \
	make clean -C $$dir; \
	echo "----------"; \
	done

