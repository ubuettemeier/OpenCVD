CC = g++
# --- gnu++11 uses GNU extensions. ---
# FLAGS = --std=gnu++11
FLAGS = --std=c++11

OPENCVD = ../../include/opencvd.hpp
OPENCVD += ../../include/opencvd_basic.hpp
OPENCVD += ../../include/opencvd_func.hpp
OPENCVD += ../../include/opencvd_mat.hpp
OPENCVD += ../../include/opencvd_cvdstd.hpp
OPENCVD += ../../include/opencvd_types.hpp
OPENCVD += ../../include/specdef.hpp

# ---- get pkg-config files -----
ifeq ($(shell pkg-config --list-all | grep -ow opencv4), opencv4)
OPENCV_LIBS := opencv4
else
OPENCV_LIBS := opencv
endif

CFLAGS = -Wall -c -O0 -DNDEBUG
CFLAGS += $(shell pkg-config --cflags $(OPENCV_LIBS))
CFLAGS += -I../../include \

LDFLAGS += $(shell pkg-config --libs $(OPENCV_LIBS))
LDFLAGS += -lpthread

FILENAME = gabor
BUILDFILE = $(FILENAME)

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

