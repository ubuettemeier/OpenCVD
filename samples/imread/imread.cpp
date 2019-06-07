#include "opencv2/opencv.hpp"
#include <stdio.h>

#define USE_CVD             // using namespace cvd
#include "opencvd.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{    
    cv::namedWindow("image_1");     // init a output window
    
    uint8_t ende = 0;
    while (!ende) {
        CVD::Mat a;
        a = CVD::imread ("../../images/lena_.png");
        CVD::medianBlur(a, a, 29);

        if (a.empty())
            a = CVD::Mat(200, 200, CV_8UC1, cv::Scalar(128));    // fiktive cv::Mat erzeugen !)

        if (!a.empty())
            cv::imshow ("image_1", a);

        int taste = cv::waitKey(10);
        if (taste != -1)
            if (taste == 27) 
                ende = 1;
    }    
    return 0;
}
