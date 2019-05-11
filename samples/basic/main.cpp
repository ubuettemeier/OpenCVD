#include <iostream>
#include "opencv2/opencv.hpp"

#define USE_CVD
#include "opencvd.hpp"

using namespace std;
using namespace cv;

void my_erode (Mat &src, Mat &dst, int iters, int type)
{
    int erosion_size = iters;
    /*
    Mat element = CVD::getStructuringElement(type,                   // Create a structuring element
             cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
             cv::Point(erosion_size, erosion_size) );
    */
    Mat element = CVD::getStructuringElement(type,                   // Create a structuring element
             cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1));
    CVD::erode(src, dst, element);                               // Apply erosion or dilation on the image
}

void my_dilate (cv::Mat &src, cv::Mat &dst, int iters, int type)
{
    int erosion_size = iters;
    /*
    Mat element = CVD::getStructuringElement(type,                       // Create a structuring element
             cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
             cv::Point(erosion_size, erosion_size) );
    */
    Mat element = CVD::getStructuringElement(type,                       // Create a structuring element
             cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1));
    CVD::dilate(src, dst, element);                                  // Apply erosion or dilation on the image
}

void my_morphologyEx (cv::Mat &src, cv::Mat &dst, int op)
{
    Mat kernel = CVD::getStructuringElement(cv::MORPH_RECT,                       // Create a structuring element
             cv::Size(3, 3));

    CVD::morphologyEx(src, dst, op, kernel);
}

int main()
{
    uint8_t ende = 0;

    cv::namedWindow("image_1");

    cv::VideoCapture cap(0);        // open the default camera
    if(!cap.isOpened()) {           // check if we succeeded
        printf ("NO CAMERA\n");
        return -1;
    }

    while (!ende) {
        cv::Mat a;        
        cap >> a;                   // Bildeinzug

        CVD::cvtColor(a, a, cv::COLOR_BGR2GRAY);
        CVD::convertScaleAbs( a, a);

        my_morphologyEx (a, a, cv::MORPH_OPEN);

        cv::Mat b;
        CVD::Laplacian( a, b, 0);
        /*
        cv::Mat b, d;
        my_dilate (a, b, 1, cv::MORPH_ELLIPSE);
        my_erode(b, d, 1, cv::MORPH_ELLIPSE);


        CVD::blur(a, a, cv::Size(3, 3));
        CVD::medianBlur(a, a, 3);
        */
        /*
        CVD::medianBlur(a, a, 3);
        CVD::GaussianBlur(a, a, cv::Size(3, 3), 3);
        CVD::cvtColor(a, a, cv::COLOR_BGR2GRAY);
        CVD::threshold(a, a, 80, 224, THRESH_BINARY);

        cv::Mat c;
        CVD::Canny(a, c, 80, 255);
        */
        if (!b.empty())
            cv::imshow ("image_1", b);

        int taste = cv::waitKey(10);
        if (taste != -1) {
            // printf ("%i\n", taste);
            if (taste == 27) ende = 1;          // ESC
            if (taste == 97) show_func_list();  // a
        }
    }
    KILL_CVD
}
