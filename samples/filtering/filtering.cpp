#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <ctype.h>
#include <stdio.h>
#include <iostream>

#define USE_CVD
#include "opencvd.hpp"

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
    uint8_t ende = 0;
    
    cv::namedWindow("image_1");

    cv::VideoCapture cap(0);        // open the default camera
    if(!cap.isOpened()) {           // check if we succeeded
        printf ("NO CAMERA\n");
        return -1;
    }

    /*
    cap.set(CAP_PROP_MODE, cv::CAP_MODE_GRAY);  // does not work on raspberry pi.
    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);
    */

    while (!ende) {
        CVD::Mat a;
        cap >> a;                   // Bildeinzug
        
        CVD::medianBlur(a, a, 21);
        CVD::blur (a, a, cv::Size (3, 3));
        
        if (!a.empty())
            cv::imshow ("image_1", a);

        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;
        }
    }
    
    return 0;
}
