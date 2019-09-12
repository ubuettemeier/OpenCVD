
#include <iostream>

#define USE_CVD

#include <opencvd.hpp>
#include <specdef.hpp>

using namespace std;
using namespace cv;

// #define SHOW_FIELD 2

int main ()
{    
    int ende = 0;
    cv::TickMeter tm;
    char buf[256];

    while (!ende) {
        int SHOW_FIELD = get_numval<int>(3, "SHOW_FIELD");

        cv::Mat img = CVD::imread("../../images/box_in_scene.png", cv::IMREAD_COLOR);

        if (!img.empty()) {
            tm.reset(); tm.start();

            cv::Mat gray = cv::Mat();
            cv::cvtColor(img, gray, CV_BGR2GRAY);

            cv::Mat eigen = cv::Mat(img.rows, img.cols, CV_32FC(6));
            CVD::cornerEigenValsAndVecs(gray, eigen, 15, 3);

            cv::Scalar col(0, 0, 0, 0);
            cv::Mat vectorfield (img.rows, img.cols, CV_8UC3, col);

            int d = set_trackbar<int>(12, "resolution", 4, 20, 1);          //! \see <specdef.hpp>

            for (int y=(d/2); y<img.rows; y+=d) {
                 for (int x=(d/2); x<img.cols; x+=d) {
                     if (x < img.cols && y < img.rows) {
                        cv::Point p(x, y);

                        float dx1 = eigen.at<Vec6f>(p)[2] * (d/2);
                        float dy1 = eigen.at<Vec6f>(p)[3] * (d/2);

                        cv::Point p10(p.x - dx1, p.y - dy1);
                        cv::Point p11(p.x + dx1, p.y + dy1);

                        if (SHOW_FIELD & 1) {
                             cv::line(vectorfield, p10, p11,
                                      cv::Scalar(
                                      img.at<Vec3b>(cv::Point(x, y))[0],
                                      img.at<Vec3b>(cv::Point(x, y))[1],
                                      img.at<Vec3b>(cv::Point(x, y))[2]
                                      ),
                                     1);
                        }
                        // ------------------------------------------------------------
                        float dx2 = eigen.at<Vec6f>(p)[4] * (d/2);
                        float dy2 = eigen.at<Vec6f>(p)[5] * (d/2);

                        cv::Point p20(p.x - dx2, p.y - dy2);
                        cv::Point p21(p.x + dx2, p.y + dy2);
                        if (SHOW_FIELD & 2) {
                             cv::line(vectorfield, p20, p21,
                                      cv::Scalar(
                                      img.at<Vec3b>(cv::Point(x, y))[0],
                                      img.at<Vec3b>(cv::Point(x, y))[1],
                                      img.at<Vec3b>(cv::Point(x, y))[2]
                                      ),
                                     1);
                        }
                        if (SHOW_FIELD & 1)
                            cv::line(img, p10, p11, cv::Scalar(0, 255, 255), 1);
                        if (SHOW_FIELD & 2)
                            cv::line(img, p20, p21, cv::Scalar(0, 0, 255), 1);

                     }
                 }
            }
            tm.stop();
            sprintf (buf, "time=%6.1f ms", tm.getTimeMilli());
            CVD::putText (img, buf, cv::Point (10, 20), cv::FONT_HERSHEY_PLAIN, 1.3, cv::Scalar(0, 255, 255), 2);

            cv::imshow("VectorField", vectorfield);
            cv::imshow("FLOW", img);
        }

        int taste = cv::waitKey(10);
        if (taste != -1)
            if (taste == 27) ende = 1;                              // ESC
    }
    return 0;
}

