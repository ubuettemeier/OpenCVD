#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

#define USE_CVD         // using namespace cvd
#include "opencvd.hpp"


int main( int argc, const char** argv )
{
    uint8_t ende = 0;

    cv::namedWindow("buildPyramid", cv::WINDOW_AUTOSIZE);

    while (!ende) {     // CVD - loop 
        CVD::Mat src;
        src = CVD::imread("../../images/lena.png");

        if (src.empty()) {
            cout << "no image\n";
            return -1;
        }

        int maxVal = 4;
        vector<Mat> dstVect;
        CVD::buildPyramid(src, dstVect, maxVal);

        int show_vec = 2;		// 0..4
        if (!dstVect[show_vec].empty())
            cv::imshow ("buildPyramid", dstVect[show_vec] );

        int taste = waitKey(10);        
        if (taste != -1)
            if (taste == 27)        	// Break with ESC
                ende = 1;
    }
    return 0;
}
