TEMPLATE = app
# CONFIG += console gnu++11
CONFIG += console C++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += \
    ../../../include

INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/opencv2

# INCLUDEPATH += /home/ulrich/c_source/opencv/mysource/ueye_for_opencv

LIBS += -L/usr/local/lib \
-lopencv_core \
-lopencv_imgcodecs \
-lopencv_highgui \
-lopencv_imgproc \
-lopencv_video \
-lopencv_videoio \
-lpthread \
-lrt \
-lm
# -lueye_api \
# -lubtools

SOURCES += main.cpp

HEADERS += \
    ../../../include/opencvd.hpp \
    ../../../include/opencvd_types.hpp \
    ../../../include/opencvd_basic.hpp \
    ../../../include/opencvd_func.hpp

DISTFILES += \
    ../../../Readme.txt
