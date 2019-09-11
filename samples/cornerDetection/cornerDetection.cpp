
#include <iostream>

#define USE_CVD

#include <opencvd.hpp>
#include <specdef.hpp>

using namespace std;
using namespace cv;

int main ()
{    
    int ende = 0;
    cv::TickMeter tm;
    char buf[256];

    while (!ende) {
        cv::Mat img = CVD::imread("../../images/box_in_scene.png", cv::IMREAD_COLOR);

        if (!img.empty()) {
            tm.reset(); tm.start();

            cv::Mat gray = cv::Mat();
            cv::cvtColor(img, gray, CV_BGR2GRAY);

            cv::Mat eigen = cv::Mat(img.rows, img.cols, CV_32FC(6));
            CVD::cornerEigenValsAndVecs(gray, eigen, 15, 3);

            int d = set_trackbar<int>(12, "resolution", 4, 20, 1);          //! \see <specdef.hpp>

            for (int y=(d/2); y<img.rows; y+=d) {
                 for (int x=(d/2); x<img.cols; x+=d) {
                     if (x < img.cols && y < img.rows) {
                         cv::Point p(x, y);
                         float dx = eigen.at<Vec6f>(p)[4] * (d/2);
                         float dy = eigen.at<Vec6f>(p)[5] * (d/2);

                         cv::Point p0(p.x - dx, p.y - dy);
                         cv::Point p1(p.x + dx, p.y + dy);
                         cv::line(img, p0, p1, cv::Scalar(0, 0, 255), 1);
                      }
                 }
            }
            tm.stop();
            sprintf (buf, "time=%6.1f ms", tm.getTimeMilli());
            CVD::putText (img, buf, cv::Point (10, 20), cv::FONT_HERSHEY_PLAIN, 1.3, cv::Scalar(0, 255, 255), 2);

            cv::imshow("FLOW", img);
        }

        int taste = cv::waitKey(10);
        if (taste != -1)
            if (taste == 27) ende = 1;                              // ESC
    }
    return 0;
}

