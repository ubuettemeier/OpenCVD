#include <iostream>

#define USE_CVD
#include <opencvd.hpp>

using namespace std;
using namespace cv;

int main( )
{
    int ende = 0;
    cv::namedWindow("Image");
    cv::namedWindow("Segmented Image");

    cv::Mat image;
    cv::Mat result;             // segmentation result (4 possible values)
    cv::Mat bgModel,fgModel;    // the models (internally used)

    cv::Rect rectangle(200, 60, 80, 80);

    while (!ende) {
        image = CVD::imread("../../images/messi5.jpg");

        if (image.data) {
            CVD::grabCut(image,             // input image
                 result,                    // segmentation result
                 rectangle,                 // rectangle containing foreground
                 bgModel, fgModel,          // models
                 1,                         // number of iterations
                 cv::GC_INIT_WITH_RECT);    // use rectangle

            // Get the pixels marked as likely foreground
            cv::compare( result, cv::Scalar(cv::GC_PR_FGD), result, cv::CMP_EQ );
            // Generate output image
            cv::Mat foreground(image.size(), CV_8UC3, cv::Scalar(255,255,255));     // foreground fill with white
            image.copyTo(foreground, result);                                       // bg pixels not copied !!!

	        cv::rectangle (image, rectangle, cv::Scalar(0, 0, 255), 3);     // draw rectngle
            cv::imshow("Image", image);
            cv::imshow("Segmented Image", foreground);      // display result
        }

        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;      // ESC
        }
    }
    return 0;
}

