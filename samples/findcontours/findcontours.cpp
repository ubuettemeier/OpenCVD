#include "opencv2/opencv.hpp"
#include <stdio.h>

#define USE_CVD             // using namespace cvd
#include "opencvd.hpp"

using namespace cv;
using namespace std;

int thresh = 100;

int main( int argc, char** argv )
{    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    cv::namedWindow("image_1");     // init a output window 1
    cv::namedWindow("image_2");     // init a output window 2

    cv::VideoCapture cap(0);        // open the default camera
    if(!cap.isOpened()) {           // check if we succeeded
        printf ("NO CAMERA\n");
        return -1;
    }
    
    uint8_t ende = 0;
    while (!ende) {
        CVD::Mat src, a, b;
        cap >> src;                   // Bildeinzug
        
        CVD::cvtColor (src, a, cv::COLOR_BGR2GRAY);
        CVD::medianBlur(a, a, 7);
        CVD::Canny( a, a, thresh, thresh*2, 3 );  
        // CVD::findContours( a, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, Point(0, 0) );
        CVD::findContours( a, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, Point(0, 0) );    // overload
        if (contours.size()) {
            src.copyTo(b);
            for (int i=0; i<(int)contours.size(); i++) {
                std::vector<cv::Point> poly;
                CVD::approxPolyDP( contours[i], poly, 5, true);
                cv::polylines( b, poly, true, cv::Scalar(0, 0, 255), 2 );
            }
        }

        if (!b.empty())
            cv::imshow ("image_2", b);
        if (!a.empty())
            cv::imshow ("image_1", a);

        int taste = cv::waitKey(10);
        if (taste != -1)
            if (taste == 27) ende = 1;
    }    
    return 0;
}
