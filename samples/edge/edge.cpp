#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <stdio.h>

using namespace cv;
using namespace std;

#define USE_CVD         // using namespace cvd
#include "opencvd.hpp"

int edgeThresh = 1;
int edgeThreshScharr=1;
CVD::Mat image, gray, blurImage, edge1, edge2, cedge;
const char* window_name1 = "Edge map : Canny default (Sobel gradient)";
const char* window_name2 = "Edge map : Canny with custom gradient (Scharr)";

void doit() {
    CVD::blur(gray, blurImage, Size(3,3));
    // Run the edge detector on grayscale
    CVD::Canny(blurImage, edge1, edgeThresh, edgeThresh*3, 3);
    cedge = Scalar::all(0);
    image.copyTo(cedge, edge1);
    imshow(window_name1, cedge);
    CVD::Mat dx,dy;
    CVD::Scharr(blurImage,dx,CV_16S,1,0);
    CVD::Scharr(blurImage,dy,CV_16S,0,1);
    CVD::Canny( dx,dy, edge2, edgeThreshScharr, edgeThreshScharr*3 );
    cedge = Scalar::all(0);
    image.copyTo(cedge, edge2);
    imshow(window_name2, cedge);
}

static void help()
{
    printf("\nThis sample demonstrates Canny edge detection\n"
           "Call:\n"
           "    /.edge [image_name -- Default is ../../images/lena.png]\n\n");
}

const char* keys =
{
    "{help h||}{@image |../../images/lena.png|input image name}"
};

int main( int argc, const char** argv )
{
    help();
    CommandLineParser parser(argc, argv, keys);
    string filename = parser.get<string>(0);
    uint8_t ende = 0;
    while (!ende) {     // CVD - loop 
        image = CVD::imread(filename, IMREAD_COLOR);
        if(!image.empty()) {            
            cedge.create(image.size(), image.type());
            CVD::cvtColor(image, gray, COLOR_BGR2GRAY);

            namedWindow(window_name1, 1);
            namedWindow(window_name2, 1);
            
            doit();     // main work
        }        
        int taste = waitKey(10);        
        if (taste != -1)
            if (taste == 27)        // Break with ESC
                ende = 1;
    }
    return 0;
}
