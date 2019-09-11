#include <iostream>

#define USE_CVD

#include <opencvd.hpp>
#include <specdef.hpp>

using namespace std;
using namespace cv;

int main()
{
    int ende = 0;
    double myHarris_minVal, myHarris_maxVal;

    cv::namedWindow("src");
    cv::namedWindow("Mc");
    cv::Mat src;
    cv::Mat Mc;

    while (!ende) {
        src = CVD::imread( "../../images/box_in_scene.png");
        if ( !src.empty() ) {
            cvtColor( src, src, COLOR_BGR2GRAY );
            cv::Mat dst;
            CVD::cornerEigenValsAndVecs( src, dst, 3, 5 );      // dst-type = CV_32FC(6) => (lamda1, lamda2, x1, y1, x2, y2)

            // calculate Mc
            Mc = Mat( src.size(), CV_32FC1 );                   // 1 channel
            for( int i = 0; i < src.rows; i++ ) {
                for( int j = 0; j < src.cols; j++ ) {
                    float lambda_1 = dst.at<Vec6f>(i, j)[0];    // the type of dst is CV_32FC(6) // #define CV_32FC(n) CV_MAKETYPE(CV_32F,(n))
                    float lambda_2 = dst.at<Vec6f>(i, j)[1];
                    Mc.at<float>(i, j) = lambda_1*lambda_2 - 0.04f * pow(( lambda_1 + lambda_2 ), 2 ); // pow = (lamda_1*lamda_2)^2
                }
            }
            minMaxLoc( Mc, &myHarris_minVal, &myHarris_maxVal );
            cout << "min=" << myHarris_minVal << "  max=" << myHarris_maxVal << endl;
            cv::imshow ("Mc", Mc);
            cv::imshow ("src", src);
        }
        int taste = cv::waitKey(10);
        if (taste != -1) {
            if (taste == 27) ende = 1;                              // ESC
        }
    }
    return 0;
}


/*
// https://dennis2society.de/opencvs-textureflow-example-in-c

#include <iostream>

#define USE_CVD

#include <opencvd.hpp>
#include <specdef.hpp>

using namespace std;
using namespace cv;

int main (int argc, char** argv)
{
    cv::TickMeter tm;
    tm.start();
    cv::Mat img = cv::imread(argv[1]);
    cv::Mat gray = cv::Mat();
    cv::cvtColor(img, gray, CV_BGR2GRAY);
    // to preserve the original image
    cv::Mat flow = gray.clone();
    int width = img.cols;
    int height = img.rows;
    int graySize = width * height;
    // "brighten" the flow image
    // C++ version of:
    // vis[:] = (192 + np.uint32(vis)) / 2
    for (unsigned int i=0; i<(graySize*3); ++i)
    {
         img.data[i] = (uchar)((192 + (int)img.data[i]) / 2);
    }
    cv::Mat eigen = cv::Mat(height, width, CV_32FC(6));
    cv::cornerEigenValsAndVecs(gray, eigen, 15, 3);
    // this is the equivalent to all the numpy's reshaping etc. to
    // generate the flow arrays
    // simply use channel 4 and 5 as the actual flow array in C++
    std::vector<> channels;
    cv::split(eigen, channels);

    int d = 12;
    cv::Scalar col(0, 0, 0);
    // C++ version of:
    // points =  np.dstack( np.mgrid[d/2:w:d, d/2:h:d] ).reshape(-1, 2)
    // including the actual line drawing part
    for (unsigned int y=(d/2); y<flow.rows; y+=d)
    {
         for (unsigned int x=(d/2); x<flow.cols; x+=d)
         {
             if (x < flow.cols && y < flow.rows)
             {
                 cv::Point p(x, y);
                 float dx = channels[4].at(p) * (d/2);
                 float dy = channels[5].at(p) * (d/2);
                 cv::Point p0(p.x - dx, p.y - dy);
                 cv::Point p1(p.x + dx, p.y + dy);
                 cv::line(flow, p0, p1, col, 1);
              }
         }
    }
    tm.stop();
    std::cout<<"Flow image generated in "<<tm.getTimeMilli()<<" ms."<<std::endl;
    cv::imshow("FLOW", flow);
    cv::waitKey();
    return 0;
}
*/

