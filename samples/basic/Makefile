CC = g++
# --- gnu++11 uses GNU extensions. ---
# FLAGS = --std=gnu++11
FLAGS = --std=c++11

OPENCVD = ../../include/opencvd.hpp
OPENCVD += ../../include/opencvd_basic.hpp
OPENCVD += ../../include/opencvd_func.hpp
OPENCVD += ../../include/opencvd_types.hpp

# ---- set pkg-config file -----
# Example OpenCV 4.x: make cv4=1

OPENCV_LIBS = opencv

ifdef cv4
OPENCV_LIBS = opencv4
endif


CFLAGS = -Wall -c -O0 -DNDEBUG
CFLAGS += $(shell pkg-config --cflags $(OPENCV_LIBS))
CFLAGS += -I../../include \

LDFLAGS += $(shell pkg-config --libs $(OPENCV_LIBS))
LDFLAGS += -lpthread

FILENAME = main
BUILDFILE = basic_test

SOURCE = $(FILENAME).cpp
HEADER = 
QUELLE = $(SOURCE)  $(HEADER)

OBJ = $(FILENAME).o
BIN = $(BUILDFILE)

.PHONEY: all
all: $(BIN)

$(BIN): $(OBJ)
	$(CC) -o $@ $< $(LDFLAGS) 

$(OBJ): $(SOURCE) $(HEADER) $(OPENCVD)
	$(CC) $(CFLAGS) $(FLAGS) $(INC) $<

.PHONEY: clean
clean:	
	$(RM) -r -f $(OBJ)	
	$(RM) -r -f $(BUILDFILE)

