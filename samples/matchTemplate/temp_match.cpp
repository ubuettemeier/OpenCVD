#define USE_CVD             // using namespace cvd
#include "opencvd.hpp"

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	cv::Mat src_img, template_img;
	cv::Mat result_mat;
	cv::Mat debug_img;
        bool imread_ok = true;
        char buf[256];

	while(true) {
            imread_ok = true;

            template_img = CVD::imread("../../images/lena_eye.png", IMREAD_GRAYSCALE);
            if (template_img.data == NULL) {
                    printf("cv::imread() failed...\n");
                    imread_ok = false;
            }

            src_img = CVD::imread("../../images/lena.png", IMREAD_GRAYSCALE);
            if (src_img.data == NULL) {
                    printf("cv::imread() failed...\n");
                    imread_ok = false;
            }
            if (imread_ok) {
                CVD::cvtColor(src_img, debug_img, COLOR_GRAY2BGR);

                // method: TM_SQDIFF, TM_SQDIFF_NORMED, TM _CCORR, TM_CCORR_NORMED, TM_CCOEFF, TM_CCOEFF_NORMED
                int match_method = TM_CCORR_NORMED;
                CVD::matchTemplate(src_img, template_img, result_mat, match_method);
                CVD::normalize(result_mat, result_mat,
                               0.0, 1.0,
                               cv::NORM_MINMAX, -1, cv::Mat());

                double minVal; double maxVal;
                cv::Point minLoc, maxLoc, matchLoc;
                cv::minMaxLoc(result_mat, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
                if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
                    matchLoc = minLoc;
                else
                    matchLoc = maxLoc;

                cv::rectangle(
                        debug_img,
                        matchLoc,
                        cv::Point(matchLoc.x + template_img.cols , matchLoc.y + template_img.rows),
                        CV_RGB(255,0,0),
                        3);

                sprintf (buf, "x=%i  y=%i", matchLoc.x, matchLoc.y);
                cv::putText(debug_img, buf, Point2f(20,20), FONT_HERSHEY_PLAIN, 1.2,  Scalar(0,255,255,255), 2 , 8 , false);


                if (!debug_img.empty())
                    cv::imshow("debug_img", debug_img);
            }

            int c = cv::waitKey(1);
            if (c == 27) break;
	}
	return 0;
}
