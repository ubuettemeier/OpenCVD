#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

#define USE_CVD             // using namespace cvd
#include "opencvd.hpp"

int _brightness = 100;
int _contrast = 100;

Mat image;

/* brightness/contrast callback function */
static void updateBrightnessContrast( int /*arg*/, void* )
{
    int histSize = 64;
    int brightness = _brightness - 100;
    int contrast = _contrast - 100;

    /*
     * The algorithm is by Werner D. Streidt
     * (http://visca.com/ffactory/archives/5-99/msg00021.html)
     */
    double a, b;
    if( contrast > 0 )
    {
        double delta = 127.*contrast/100;
        a = 255./(255. - delta*2);
        b = a*(brightness - delta);
    }
    else
    {
        double delta = -128.*contrast/100;
        a = (256.-delta*2)/255.;
        b = a*brightness + delta;
    }

    CVD::Mat dst, hist;
    image.convertTo(dst, CV_8U, a, b);
    imshow("image", dst);

    CVD::calcHist(&dst, 1, NULL, CVD::Mat(), hist, 1, &histSize, 0);
    Mat histImage = Mat::ones(200, 320, CV_8U)*255;

    CVD::normalize(hist, hist, 1.00, histImage.rows, NORM_MINMAX, CV_32F);

    histImage = Scalar::all(255);
    int binW = cvRound((double)histImage.cols/histSize);

    for( int i = 0; i < histSize; i++ )
        rectangle( histImage, Point(i*binW, histImage.rows),
                   Point((i+1)*binW, histImage.rows - cvRound(hist.at<float>(i))),
                   Scalar::all(0), -1, 8, 0 );
    imshow("histogram", histImage);
}
static void help()
{
    std::cout << "\nThis program demonstrates the use of calcHist() -- histogram creation.\n"
              << "Usage: \n" << "demhist [image_name -- Defaults to ../../images/baboon.jpg]" << std::endl;
}

const char* keys =
{
    "{help h||}{@image|../../images/baboon.jpg|input image file}"
};

int main( int argc, const char** argv )
{
    CommandLineParser parser(argc, argv, keys);
    if (parser.has("help"))
    {
        help();
        return 0;
    }
    string inputImage = parser.get<string>(0);

    uint8_t ende = 0;
    while (!ende) {        
        image = CVD::imread( inputImage, 0 );   // Load the source image. HighGUI use.
        if(!image.empty())
        {                    
            namedWindow("image", 0);
            namedWindow("histogram", 0);

            updateBrightnessContrast (0, 0);
        }
        int taste = waitKey(10);        
        if (taste != -1)
            if (taste == 27)        // Break with ESC
                ende = 1;
    }
    return 0;
}
