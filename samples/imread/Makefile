CC = g++
# gnu++11 uses GNU extensions.
# FLAGS = --std=gnu++11
FLAGS = --std=c++11

OPENCVD = ../../include/opencvd.hpp
OPENCVD += ../../include/opencvd_basic.hpp
OPENCVD += ../../include/opencvd_func.hpp
OPENCVD += ../../include/opencvd_types.hpp

CFLAGS = -Wall -c -O0 -DNDEBUG
CFLAGS += \
-I../../include \
-I/usr/local/include \
-I/usr/include \
-I/usr/local/include/opencv2 \
-I/usr/local/include/opencv

LDFLAGS += -L/usr/local/lib \
-lopencv_core \
-lopencv_imgcodecs \
-lopencv_highgui \
-lopencv_imgproc \
-lopencv_video \
-lopencv_videoio \
-lopencv_features2d \
-lpthread 

FILENAME = imread
BUILDFILE = $(FILENAME)

SOURCE = $(FILENAME).cpp
HEADER = 
QUELLE = $(SOURCE)  $(HEADER)

OBJ = $(FILENAME).o
BIN = $(BUILDFILE)

$(BIN): $(OBJ)
	$(CC) -o $(BIN) $(OBJ) $(LDFLAGS)

$(OBJ): $(SOURCE) $(HEADER) $(OPENCVD)
	$(CC) $(CFLAGS) $(FLAGS) $(INC) $(SOURCE)

.PHONEY:	clean
clean:	
	$(RM) -r -f $(OBJ)	
	$(RM) -r -f $(BUILDFILE)

