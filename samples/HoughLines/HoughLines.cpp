
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define USE_CVD_
#include "opencvd.hpp"

using namespace cv;
using namespace std;

void my_morphologyEx (cv::Mat &src, cv::Mat &dst, int op)
{
    CVD::Mat kernel = CVD::getStructuringElement(cv::MORPH_RECT,                       // Create a structuring element
                                            cv::Size(3, 3));
    CVD::morphologyEx( src, dst, op, kernel );
}

int main(int argc, char** argv)
{
    CVD::Mat src, mor, dst, color_dst;

    uint8_t ende = 0;
    while (ende == 0) {
        src = CVD::imread("../../images/street.jpg", 0);
        if (src.empty()) {
            cout << "no image\n";
            return -1;
        }
        CVD::resize(src, src, cv::Size(480, 320));

        my_morphologyEx (src, mor, cv::MORPH_OPEN);
        CVD::Canny( mor, dst, 50, 200, 3 );
        CVD::cvtColor( dst, color_dst, COLOR_GRAY2BGR );


    #if 1
        vector<Vec2f> lines;
        CVD::HoughLines( dst, lines, 1, CV_PI/180, 100 );	// red line
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0];
            float theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            Point pt1(cvRound(x0 + 1000*(-b)),
                      cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                      cvRound(y0 - 1000*(a)));
            line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
	    if (lines.size() < 20) 
	    	line( src, pt1, pt2, Scalar(0,0,255), 3, 8 );
        }

        CVD::HoughLines( dst, lines, 1, CV_PI/180, 100 );	// green line
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0];
            float theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            Point pt1(cvRound(x0 + 1000*(-b)),
                      cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                      cvRound(y0 - 1000*(a)));
            line( color_dst, pt1, pt2, Scalar(0,255,0), 3, 8 );
	    if (lines.size() < 20)
            	line( src, pt1, pt2, Scalar(0,255,0), 3, 8 );
        }
    #else
        vector<Vec4i> lines;
        CVD::HoughLinesP( dst, lines, 1, CV_PI/180, 80, 30, 10 );
        for( size_t i = 0; i < lines.size(); i++ )
        {
            line( color_dst, Point(lines[i][0], lines[i][1]),
                Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
        }
    #endif
        namedWindow( "Source", 1 );
        imshow( "Source", src );

        namedWindow( "Detected Lines", 1 );
        imshow( "Detected Lines", color_dst );

        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;          // ESC
        }
    }
    return 0;
}

