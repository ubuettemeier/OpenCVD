#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

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
void draw_image_1 (cv::Mat &image1, uint8_t &mode)
{
    static cv::Point sp[10];
    static cv::Point ep[10];

    if (mode == 0)                                                                      // Circle
        cv::circle(image1, cv::Point(200, 200), 80, cv::Scalar( 0 ), 2 );
    if (mode == 1)                                                                      // Rectangle
        cv::rectangle(image1, cv::Rect(100, 100, 200, 200), cv::Scalar( 0 ), 2);
    if (mode == 2) {                                                                    // Triangle
        cv::line (image1, cv::Point(100, 300), cv::Point(200, 100), cv::Scalar( 0 ), 2);
        cv::line (image1, cv::Point(200, 100), cv::Point(300, 300), cv::Scalar( 0 ), 2);
        cv::line (image1, cv::Point(300, 300), cv::Point(100, 300), cv::Scalar( 0 ), 2);
    }
    if (mode == 3) {                                                                    // cross
        cv::line (image1, cv::Point(100, 100), cv::Point(300, 300), cv::Scalar( 0 ), 2);
        cv::line (image1, cv::Point(100, 300), cv::Point(300, 100), cv::Scalar( 0 ), 2);
    }
    if (mode == 4) {
        cv::line (image1, cv::Point(150, 100), cv::Point(150, 300), cv::Scalar( 0 ), 2);
        cv::line (image1, cv::Point(250, 100), cv::Point(250, 300), cv::Scalar( 0 ), 2);

        cv::line (image1, cv::Point(100, 150), cv::Point(300, 150), cv::Scalar( 0 ), 2);
        cv::line (image1, cv::Point(100, 250), cv::Point(300, 250), cv::Scalar( 0 ), 2);
    }
    if (mode == 5) {
        for (int i=0; i<10; i++) {
            sp[i].x = rand() % 400 + 1;
            sp[i].y = rand() % 400 + 1;
            ep[i].x = rand() % 400 + 1;
            ep[i].y = rand() % 400 + 1;
        }
        mode = 6;
    }
    if (mode == 6) {
        for (int i=0; i<10; i++)
            cv::line (image1, sp[i], ep[i], cv::Scalar( 0 ), 2);
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief build_image_2
//! \param image2
//!
void draw_image_2 (cv::Mat &image2)
{
    static double angle = 45.0;
    static int delta_axis_size = 0;
    static bool count_up = true;

    cv::ellipse( image2, cv::Point(200, 200),
                 cv::Size( 5+delta_axis_size, 120-delta_axis_size ),
                 angle, 0, 360, cv::Scalar( 0 ), 2, 8 );

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

    srand(time(NULL));

    cv::namedWindow("image_1");
    cv::namedWindow("image_2");

    int ende = 0;
    while (!ende) {
        int64 t1 = cv::getTickCount();
        double runtime = (t1-t0)/cv::getTickFrequency();        // control time
        if (runtime > 2.0) {                                    // 2 seconds
            t0 = cv::getTickCount();
            mode = (mode < 5) ? mode+1 : 0;
        }

        cv::Mat image1 = cv::Mat(400, 400, CV_8UC1, 168);
        cv::Mat image2 = cv::Mat(400, 400, CV_8UC1, 168);

        draw_image_1 (image1, mode);
        draw_image_2 (image2);

        double result = CVD::matchShapes(image1, image2, cv::CONTOURS_MATCH_I1, 0);          // match the images
        sprintf (buf, "%1.8f", result);

        cv::putText(image2, buf, cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0), 1);

        cv::imshow ("image_1", image1);
        cv::imshow ("image_2", image2);

        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;                              // ESC
        }
    }
}
