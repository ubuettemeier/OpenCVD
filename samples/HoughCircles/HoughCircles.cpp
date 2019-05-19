#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

#define USE_CVD             // using namespace cvd
#include "opencvd.hpp"

static void help()
{
    cout << "\nThis program demonstrates circle finding with the Hough transform.\n"
            "Usage:\n"
            "./houghcircles <image_name>, Default is ../../images/coins.jpg\n" << endl;
}
int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv,
        "{help h ||}{@image|../../images/coins.jpg|}"
    );
    if (parser.has("help"))
    {
        help();
        return 0;
    }
    string filename = parser.get<string>("@image");

    Mat img, gray;
    vector<Vec3f> circles;

    int ende = 0;
    while (!ende) {
            img = CVD::imread(filename, IMREAD_COLOR);
	    if(img.empty())
	    {
		help();
		cout << "can not open " << filename << endl;
		return -1;
	    }	    
            CVD::cvtColor(img, gray, COLOR_BGR2GRAY);
	    CVD::medianBlur(gray, gray, 5);	    
            CVD::HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                              gray.rows/16,     // change this value to detect circles with different distances to each other
                              100, 30, 25, 50); // change the last two parameters
                                                // (min_radius & max_radius) to detect larger circles

	    for( size_t i = 0; i < circles.size(); i++ )
	    {
		Vec3i c = circles[i];
		circle( img, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, LINE_AA);
		circle( img, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, LINE_AA);
	    }

            imshow("detected circles", img);
            img = 0;

            int taste = waitKey(10);        
            if (taste != -1)
                if (taste == 27)        // Break with ESC
                    ende = 1;

    }
    return 0;
}
