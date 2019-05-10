TEMPLATE = app
# CONFIG += console gnu++11
CONFIG += console C++11
CONFIG -= app_bundle
CONFIG -= qt

# INCLUDEPATH += pkg-config --cflags opencv
INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += \
    ../../../include

LIBS += -L/usr/local/lib \
-lpthread

# LIBS += 'pkg-config --libs opencv'
LIBS += \
-lopencv_core \
-lopencv_imgcodecs \
-lopencv_highgui \
-lopencv_imgproc \
-lopencv_video \
-lopencv_videoio \

SOURCES += main.cpp

HEADERS += \
    ../../../include/opencvd.hpp \
    ../../../include/opencvd_types.hpp \
    ../../../include/opencvd_basic.hpp \
    ../../../include/opencvd_func.hpp

DISTFILES += \
    ../../../Readme.txt
