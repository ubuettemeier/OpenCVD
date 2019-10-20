
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define USE_CVD
#include "opencvd.hpp"

using namespace cv;
using namespace std;

int main()
{
    cv::Mat src1, src2, dst;

    namedWindow( "src1", 1 );
    namedWindow( "src2", 1 );
    namedWindow( "dst", 1 );

    src1 = cv::imread("../../images/src1.jpg");
    src2 = cv::imread("../../images/src2.jpg");

    if (src1.size != src2.size) {
        cout << "ERROR: src1.size != src2.size" << endl;
        return EXIT_FAILURE;
    }

    uint8_t ende = 0;
    while (!ende) {
        CVD::addWeighted(src1, 0.5, src2, 0.5, 0.0, dst);

        if (!src1.empty())
            imshow("src1", src1);

        if (!src2.empty())
            imshow("src2", src2);

        if (!dst.empty())
            imshow("dst", dst);

        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;          // ESC
        }
    }

    return EXIT_SUCCESS;
}
