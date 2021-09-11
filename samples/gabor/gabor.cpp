/**
 * @file gabor.cpp
 * @date 2021-09-11
 */

#include <iostream>

#define USE_CVD

#include <opencvd.hpp>

using namespace std;
using namespace cv;

const int gamm_max = 100;

int size_slider = 8;
int sigma = 2;
int theta = 120;
int lambd = 24;
int gamm = 0;
int psi = 0;

Mat image, image_in_float, dest, dest2;

int main()
{
    image=imread("../../images/eye.jpg");
    image.convertTo(image_in_float, CV_32F);
    
    size_slider = 21;
    
    namedWindow("input image", WINDOW_AUTOSIZE);
    namedWindow("output image Gabor", WINDOW_AUTOSIZE); 

    double kernel_size = 2* (double) size_slider +1;

    int ende = 0;
    while (!ende) {
        Mat kernel = CVD::getGaborKernel(Size(kernel_size,kernel_size), sigma, theta, lambd, gamm/gamm_max, psi);
        CVD::filter2D(image_in_float, dest, image_in_float.depth(), kernel);
        dest.convertTo(dest2, CV_8U, 1.0/255, 0);
            
        imshow("input image", image);
        imshow( "output image Gabor", dest2);

        int taste = waitKey(10);
        if (taste == 27)
            ende = 1;
    }

    return 0;
}