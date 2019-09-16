
.PHONY: clean

SAMPLE_PATH = 	basic/ \
		edge/ \
		filtering/ \
		findcontours/ \
		imread/ \
		short_sample/ \
		demhist/ \
		HoughCircles/ \
		HoughLines/ \
                buildPyramid/ \
                LSD/ \
                matchTemplate/ \
                grabCut/ \
		matchShapes/ \
		sepFilter2D/ \
		cornerDetection/ \
		cornerEigenValsAndVecs/


all:	
	for dir in $(SAMPLE_PATH); do \
	$(MAKE) -C $$dir; \
	echo "-------------"; \
	done

clean:
	for dir in $(SAMPLE_PATH); do \
	make -C $$dir clean; \
	echo "----------"; \
	done

