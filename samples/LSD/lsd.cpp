#include <iostream>

#define USE_CVD

#include <opencvd.hpp>

using namespace std;
using namespace cv;

void show_time_text (Mat dst, double time_val)
{
    CVD::String strout = "duration = ";     // build text
    strout += to_string(time_val);
    strout += "ms";

    cv::putText(dst,                        // target image
                strout,                     // cv::String
                cv::Point(20, 20),          // top-left position
                cv::FONT_HERSHEY_DUPLEX,    // fontFace
                0.7,                        // fontScale
                CV_RGB(255, 0, 0),          // font color
                2);                         // thickness
}

int main(int argc, char** argv)
{
    int ende = 0;
    cv::namedWindow("Camera");
    cv::namedWindow("OUT");
    bool overlay = 0;
    bool use_canny = 1;

    cv::VideoCapture cap(0);        // open the default camera
    if(!cap.isOpened()) {           // check if we succeeded
        printf ("NO CAMERA\n");
        return -1;
    }

    cout << "o = overlay Camera with Line (default = 0)\n";
    cout << "c = use Canny (default = 1)\n";

    while (!ende) {
        CVD::Mat cam;

        cap >> cam;
        if (!cam.empty()) {
            CVD::Mat can;

            CVD::cvtColor (cam, can, COLOR_BGR2GRAY);
            if (use_canny)
                CVD::Canny(can, can, 50, 200, 3);                               // Apply Canny edge detector

            CVD::Mat out(cam.rows, cam.cols, CV_8UC3, cv::Scalar(0, 0, 0));     // black Mat

            double start = double(getTickCount());       // get starttime

            Ptr<LineSegmentDetector> ls = CVD::createLineSegmentDetector(LSD_REFINE_STD);   // Create and LSD detector with standard or no refinement.
            vector<Vec4f> lines_std;
            ls->detect(can, lines_std);             // detect lines_std from can
            ls->drawSegments(out, lines_std);       // draw lines_std in red
            ls->~LineSegmentDetector();             // delete Detector

            double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();  // calc time

            if (!out.empty()) {
                if (overlay)
                    out |= cam;

                cv::imshow("OUT", out);
            }

            show_time_text (cam, duration_ms);      // draw time in cam-picture
            cv::imshow ("Camera", cam);

        }
        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;
            if (taste == 'o') overlay = !overlay;
            if (taste == 'c') use_canny = !use_canny;
            cout << "overlay = " << overlay << "\nuse_canny = " << use_canny << endl;
        }
    }
    return 0;
}
