
.PHONY: clean

all:	
ifdef cv4
	$(MAKE)  cv4=1 -C basic/
	$(MAKE)  cv4=1 -C edge/
	$(MAKE)  cv4=1 -C filtering/
	$(MAKE)  cv4=1 -C findcontours/
	$(MAKE)  cv4=1 -C imread/
	$(MAKE)  cv4=1 -C short_sample/
	$(MAKE)  cv4=1 -C demhist/
	$(MAKE)  cv4=1 -C HoughCircles/
	$(MAKE)  cv4=1 -C HoughLines/
else
	$(MAKE)  -C basic/
	$(MAKE)  -C edge/
	$(MAKE)  -C filtering/
	$(MAKE)  -C findcontours/
	$(MAKE)  -C imread/
	$(MAKE)  -C short_sample/
	$(MAKE)  -C demhist/
	$(MAKE)  -C HoughCircles/
	$(MAKE)  -C HoughLines/
endif

clean:
	$(MAKE)  -C basic/  clean
	$(MAKE)  -C edge/  clean
	$(MAKE)  -C filtering/  clean
	$(MAKE)  -C findcontours/  clean
	$(MAKE)  -C imread/  clean
	$(MAKE)  -C short_sample/  clean
	$(MAKE)  -C demhist/  clean
	$(MAKE)  -C HoughCircles/ clean
	$(MAKE)  -C HoughLines/ clean

