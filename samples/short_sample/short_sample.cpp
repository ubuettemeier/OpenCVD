#include <cv.h>
#include <stdio.h>

#define USE_CVD             // using namespace cvd
#include <opencvd.hpp>     

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{    
    cv::namedWindow("image_1");     // init a output window
    cv::VideoCapture cap(0);        // open the default camera
    if(!cap.isOpened()) {           // check if we succeeded
        printf ("NO CAMERA\n");
        return -1;
    }
    
    uint8_t ende = 0;
    while (!ende) {
        cv::Mat a;
        cap >> a;                   // Bildeinzug
        
        CVD::cvtColor (a, a, cv::COLOR_BGR2GRAY);
        CVD::medianBlur(a, a, 7);        
        
        if (!a.empty())
            cv::imshow ("image_1", a);

        int taste = cv::waitKey(10);
        if (taste != -1)
            if (taste == 27) ende = 1;
    }    
    return 0;
}
