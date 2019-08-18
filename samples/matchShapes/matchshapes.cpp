#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#define USE_CVD

#include <opencvd.hpp>
#include <specdef.hpp>

using namespace cv;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief build_image_1
//! \param image1
//! \param mode
//!
void build_image_1 (cv::Mat &image1, uint8_t mode)
{
    if (mode == 0)                                                                      // Circle
        cv::circle(image1, cv::Point(200, 200), 80, Scalar( 0 ), 2 );
    if (mode == 1)                                                                      // Rectangle
        cv::rectangle(image1, cv::Rect(100, 100, 200, 200), Scalar( 0 ), 2);
    if (mode == 2) {                                                                    // Triangle
        cv::line (image1, cv::Point(100, 300), cv::Point(200, 100), Scalar( 0 ), 2);
        cv::line (image1, cv::Point(200, 100), cv::Point(300, 300), Scalar( 0 ), 2);
        cv::line (image1, cv::Point(300, 300), cv::Point(100, 300), Scalar( 0 ), 2);
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief build_image_2
//! \param image2
//!
void build_image_2 (cv::Mat &image2)
{
    static double angle = 45.0;
    static int delta_axis_size = 0;
    static bool count_up = true;

    cv::ellipse( image2, cv::Point(200, 200),
                 Size( 5+delta_axis_size, 120-delta_axis_size ),
                 angle, 0, 360, Scalar( 0 ), 2, 8 );

    if (angle < 359.0) angle += 1.0;
        else angle = 0.0;

    if (count_up) {
        if (delta_axis_size < 48) delta_axis_size++;
        else count_up = false;
    } else {
        if (delta_axis_size > 0) delta_axis_size--;
        else count_up = true;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief main
//! \return
//!
int main()
{
    char buf[256];
    uint8_t mode = 0;
    int64 t0 = cv::getTickCount();                              // get start time

    cv::namedWindow("image_1");
    cv::namedWindow("image_2");

    int ende = 0;
    while (!ende) {
        int64 t1 = cv::getTickCount();
        double runtime = (t1-t0)/cv::getTickFrequency();        // control time
        if (runtime > 2.0) {                                    // 2 seconds
            t0 = cv::getTickCount();
            mode = (mode < 2) ? mode+1 : 0;
        }

        Mat image1 = Mat(400, 400, CV_8UC1, 255);
        Mat image2 = Mat(400, 400, CV_8UC1, 255);

        build_image_1 (image1, mode);
        build_image_2 (image2);

        double result = CVD::matchShapes(image1, image2, CV_CONTOURS_MATCH_I1, 0);          // match the images
        sprintf (buf, "%1.8f", result);

        putText(image2, buf, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0), 1, CV_AA);

        cv::imshow ("image_1", image1);
        cv::imshow ("image_2", image2);

        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;                              // ESC
        }
    }
}
