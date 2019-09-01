#include <iostream>

#define USE_CVD

#include <opencvd.hpp>
#include <specdef.hpp>

using namespace std;
using namespace cv;

#define USE_FILTER2D_

int main()
{
    int ende = 0;

    cv::namedWindow("src");
    cv::namedWindow("dst");

    cv::Mat src, dst;

#ifdef USE_FILTER2D
    cv::Mat kernel;
    int kernel_size;
#else
    cv::Mat kernelX, kernelY;
    int x_kernel_size, y_kernel_size;
#endif

    while (!ende) {
        src = CVD::imread("../../images/testbild.jpg");

        if (src.data) {
            CVD::resize(src, src, cv::Size(), 0.5, 0.5);

#ifdef USE_FILTER2D
            kernel_size = get_numval<int>(3, "kernel_size");
            kernel = Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);

            CVD::filter2D(src, dst, -1 , kernel, cv::Point(-1, -1), 0., cv::BORDER_REFLECT_101 );
#else
            x_kernel_size = get_numval<int>(3, "x_kernel_size");
            y_kernel_size = get_numval<int>(3, "y_kernel_size");

            kernelX = Mat::ones( 1, x_kernel_size, CV_32F ) / (float)(x_kernel_size * x_kernel_size);   // default 1/9
            kernelY = Mat::ones( y_kernel_size, 1, CV_32F ) / (float)(y_kernel_size * y_kernel_size);   // default 1/9

            CVD::sepFilter2D(src, dst, -1 , kernelX, kernelY, cv::Point(-1, -1), 0., cv::BORDER_REFLECT_101 );
#endif
            if (!dst.empty())
                cv::imshow ("dst", dst);

            if (!src.empty())
                cv::imshow ("src", src);
        }

        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;  // ESC
        }
    }
}
