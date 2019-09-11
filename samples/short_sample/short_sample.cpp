#define USE_CVD            // using namespace cvd
#include "opencvd.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{    
    uint8_t ende = 0;
    while (!ende) {
        CVD::Mat a = CVD::imread("../../images/baboon.jpg");        
        if (!a.empty()) {
		CVD::cvtColor (a, a, cv::COLOR_BGR2GRAY);
		CVD::blur(a, a, cv::Size(5, 5));                
		CVD::Canny(a, a, 70, 70);
		cv::imshow ("image_1", a);
	}
        int taste = cv::waitKey(10);
        if (taste != -1)
            if (taste == 27) ende = 1;
    }    
    return 0;
}
