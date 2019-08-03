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

	while(true) {
            imread_ok = true;

            template_img = CVD::imread("../../images/lena_eye.png", CV_LOAD_IMAGE_GRAYSCALE);
            if (template_img.data == NULL) {
                    printf("cv::imread() failed...\n");
                    imread_ok = false;
            }

            src_img = CVD::imread("../../images/lena.png", CV_LOAD_IMAGE_GRAYSCALE);
            if (src_img.data == NULL) {
                    printf("cv::imread() failed...\n");
                    imread_ok = false;
            }
            if (imread_ok) {
                CVD::cvtColor(src_img, debug_img, CV_GRAY2BGR);

                // method: CV_TM_SQDIFF, CV_TM_SQDIFF_NORMED, CV_TM _CCORR, CV_TM_CCORR_NORMED, CV_TM_CCOEFF, CV_TM_CCOEFF_NORMED
                int match_method = CV_TM_CCORR_NORMED;
                CVD::matchTemplate(src_img, template_img, result_mat, match_method);
                CVD::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

                double minVal; double maxVal;
                cv::Point minLoc, maxLoc, matchLoc;
                cv::minMaxLoc(result_mat, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
                if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )  matchLoc = minLoc;
                else matchLoc = maxLoc;

                cv::rectangle(
                        debug_img,
                        matchLoc,
                        cv::Point(matchLoc.x + template_img.cols , matchLoc.y + template_img.rows),
                        CV_RGB(255,0,0),
                        3);

                if (!debug_img.empty())
                    cv::imshow("debug_img", debug_img);
            }

            int c = cv::waitKey(1);
            if (c == 27) break;
	}

	return 0;
}
