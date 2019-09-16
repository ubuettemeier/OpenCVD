#include <iostream>

#define USE_CVD

#include <opencvd.hpp>
#include <specdef.hpp>

using namespace std;
using namespace cv;

cv::Mat src, src_gray;

int thresh = 200;
int max_thresh = 255;
const char* source_window = "Source image";
const char* corners_window = "Corners detected";


/////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief cornerHarris_demo
//!
void cornerHarris_demo( int, void* )
{
    Mat dst = Mat::zeros( src.size(), CV_32FC1 );

    CVD::cornerHarris( src_gray, dst, 2, 3, 0.04 );

    Mat dst_norm, dst_norm_scaled;
    CVD::normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    CVD::convertScaleAbs( dst_norm, dst_norm_scaled );

    thresh = set_trackbar<int>(thresh, "Threshold", 0, 255);    // CVD Trackbar
    for( int i = 0; i < dst_norm.rows ; i++ ) {
        for( int j = 0; j < dst_norm.cols; j++ ) {
            if( (int) dst_norm.at<float>(i,j) > thresh ) {
                circle( dst_norm_scaled, Point(j,i), 5,  Scalar(0), 2, 8, 0 );
            }
        }
    }
    namedWindow( corners_window );
    imshow( corners_window, dst_norm_scaled );
}

/////////////////////////////////////////////////////////////////////////////////////////
//! \brief main
//! \param argc
//! \param argv
//! \return
//!
int main( int argc, char** argv )
{
    int ende = 0;
    namedWindow( source_window );

    while (!ende) {
        src = CVD::imread( "../../images/box_in_scene.png" );
        if ( !src.empty() ) {
            CVD::cvtColor( src, src_gray, COLOR_BGR2GRAY );
            imshow( source_window, src );
            cornerHarris_demo( 0, 0 );
        }

        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;                              // ESC
        }
    }
    return 0;
}


