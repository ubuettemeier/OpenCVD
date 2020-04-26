//!
//! @author Ulrich Buettemeier
//!

#ifndef OPENCVD_FUNC_HPP
#define OPENCVD_FUNC_HPP

#include "opencv2/opencv.hpp"
#include "opencvd_mat.hpp"

#define USE_BUILDIN

//!
//! @brief  Macro included Line-Number.
//! @see    https://gcc.gnu.org/onlinedocs/gcc/Other-Builtins.html
//!
#ifdef USE_BUILDIN
    #define BUILDIN  int line_nr = __builtin_LINE(), \
                     const char *src_file = __builtin_FILE()   
#else
    #define BUILDIN  int line_nr = 0, \
                     const char *src_file = nullptr
#endif

//! -------------------------------------------------------------
#define BUILDIN_FUNC  , int line_nr, \
                      const char *src_file

#define ONLY_BUILD_FUNC int line_nr, \
                        const char *src_file

#define BUILIN_PARA line_nr, src_file


/** @todo   eventuell BUILDIN mit __builtin_FILE und __builtin_FUNCTION erweitern
 *  @example
#ifdef USE_BUILDIN
    #define BUILDIN  , int line_nr = __builtin_LINE(), \
                       const char *src_file = __builtin_FILE(), \
                       const char *func_name = __builtin_FUNCTION()

    #define BUILDIN_FUNC  , int line_nr, \
                          const char *src_file, \
                          const char *func_name
#else
    #define BUILDIN
    #define BUILDIN_FUNC
#endif
*/

namespace cvd {


//! \todo
//! cv::warpAffine()
//! cv::warpPerspective()
//! cv::getRotationMatrix2D()
//! cv::getPerspectiveTransform()
//! cv::getAffineTransform()
//! cv::invertAffineTransform()
//! cv::goodFeatureToTrack()

CV_EXPORTS_W void calcOpticalFlowPyrLK( cv::InputArray prevImg, cv::InputArray nextImg,
                                        cv::InputArray prevPts, cv::InputOutputArray nextPts,
                                        cv::OutputArray status, cv::OutputArray err,
                                        cv::Size winSize = cv::Size(21,21), int maxLevel = 3,
                                        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                        int flags = 0, double minEigThreshold = 1e-4,
                                        BUILDIN);

CV_EXPORTS_W void calcOpticalFlowFarneback( cv::InputArray prev, cv::InputArray next, cv::InputOutputArray flow,
                                            double pyr_scale, int levels, int winsize,
                                            int iterations, int poly_n, double poly_sigma,
                                            int flags,
                                            BUILDIN);

CV_EXPORTS_W void getDerivKernels( cv::OutputArray kx, cv::OutputArray ky,
                                   int dx, int dy, int ksize,
                                   bool normalize = false, int ktype = CV_32F,
                                   BUILDIN);

CV_EXPORTS_W Mat getGaborKernel( cv::Size ksize, double sigma, double theta, double lambd,
                                 double gamma, double psi = CV_PI*0.5, int ktype = CV_64F ,
                                 BUILDIN);

CV_EXPORTS_W Mat getGaussianKernel( int ksize, double sigma, int ktype = CV_64F,
                                    BUILDIN);

CV_EXPORTS_W void sepFilter2D( cv::InputArray src, cv::OutputArray dst, int ddepth,
                               cv::InputArray kernelX, cv::InputArray kernelY,
                               cv::Point anchor = cv::Point(-1,-1),
                               double delta = 0, int borderType = cv::BORDER_DEFAULT,
                               BUILDIN);

CV_EXPORTS_W void filter2D( cv::InputArray src, cv::OutputArray dst, int ddepth,
                            cv::InputArray kernel, cv::Point anchor = cv::Point(-1,-1),
                            double delta = 0, int borderType = cv::BORDER_DEFAULT,
                            BUILDIN);

CV_EXPORTS_W void putText( cv::InputOutputArray img, const cv::String& text, cv::Point org,
                         int fontFace, double fontScale, cv::Scalar color,
                         int thickness = 1, int lineType = cv::LINE_8,
                         bool bottomLeftOrigin = false,
                         BUILDIN);

CV_EXPORTS_W void sqrBoxFilter( cv::InputArray _src, cv::OutputArray _dst, int ddepth,
                                cv::Size ksize, cv::Point anchor = cv::Point(-1, -1),
                                bool normalize = true,
                                int borderType = cv::BORDER_DEFAULT,
                                BUILDIN);

CV_EXPORTS_W void boxFilter( cv::InputArray src, cv::OutputArray dst, int ddepth,
                             cv::Size ksize, cv::Point anchor = cv::Point(-1,-1),
                             bool normalize = true,
                             int borderType = cv::BORDER_DEFAULT,
                             BUILDIN);

CV_EXPORTS_W double matchShapes( cv::InputArray contour1, cv::InputArray contour2,
                                 int method, double parameter,
                                 BUILDIN);

CV_EXPORTS_W void grabCut( cv::InputArray img, cv::InputOutputArray mask, cv::Rect rect,
                           cv::InputOutputArray bgdModel, cv::InputOutputArray fgdModel,
                           int iterCount, int mode = cv::GC_EVAL,
                           BUILDIN);

CV_EXPORTS_W void matchTemplate( cv::InputArray image, cv::InputArray templ,
                                 cv::OutputArray result, int method, cv::InputArray mask = cv::noArray(),
                                 BUILDIN);

CV_EXPORTS_W void distanceTransform( cv::InputArray src, cv::OutputArray dst,
                                     int distanceType, int maskSize, int dstType = CV_32F,
                                     BUILDIN);

CV_EXPORTS_W void pyrMeanShiftFiltering( cv::InputArray src, cv::OutputArray dst,
                                         double sp, double sr, int maxLevel = 1,
                                         cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,5,1),
                                         BUILDIN );

CV_EXPORTS_W cv::Ptr<cv::LineSegmentDetector> createLineSegmentDetector(
                        int _refine = cv::LSD_REFINE_STD, double _scale = 0.8,
                        double _sigma_scale = 0.6, double _quant = 2.0, double _ang_th = 22.5,
                        double _log_eps = 0, double _density_th = 0.7, int _n_bins = 1024,
                        BUILDIN);

CV_EXPORTS_W void rectangle(cv::InputOutputArray img, cv::Point pt1, cv::Point pt2,
                          const cv::Scalar& color, int thickness = 1,
                          int lineType = cv::LINE_8, int shift = 0,
                          BUILDIN);

CV_EXPORTS void rectangle(CV_IN_OUT cv::Mat& img, cv::Rect rec,
                          const cv::Scalar& color, int thickness = 1,
                          int lineType = cv::LINE_8, int shift = 0,
                          BUILDIN);

CV_EXPORTS_W void fitLine( cv::InputArray points, cv::OutputArray line, int distType,
                           double param, double reps, double aeps,
                           BUILDIN) ;

//! \todo
CV_EXPORTS_W void cornerMinEigenVal( cv::InputArray src, cv::OutputArray dst,
                                     int blockSize, int ksize = 3,
                                     int borderType = cv::BORDER_DEFAULT,
                                     BUILDIN );

//! \todo
CV_EXPORTS_W void cornerEigenValsAndVecs( cv::InputArray src, cv::OutputArray dst,
                                          int blockSize, int ksize,
                                          int borderType = cv::BORDER_DEFAULT,
                                          BUILDIN );

CV_EXPORTS_W void cornerHarris( cv::InputArray src, cv::OutputArray dst, int blockSize,
                                int ksize, double k,
                                int borderType = cv::BORDER_DEFAULT,
                                BUILDIN );

CV_EXPORTS_W void preCornerDetect( cv::InputArray src, cv::OutputArray dst, int ksize,
                                   int borderType = cv::BORDER_DEFAULT,
                                   BUILDIN );

CV_EXPORTS_W void cornerSubPix( cv::InputArray image, cv::InputOutputArray corners,
                                cv::Size winSize, cv::Size zeroZone,
                                cv::TermCriteria criteria,
                                BUILDIN );

CV_EXPORTS_W void pyrUp( cv::InputArray src, cv::OutputArray dst,
                           const cv::Size& dstsize = cv::Size(), int borderType = cv::BORDER_DEFAULT,
                           BUILDIN );

CV_EXPORTS_W void pyrDown( cv::InputArray src, cv::OutputArray dst,
                           const cv::Size& dstsize = cv::Size(), int borderType = cv::BORDER_DEFAULT,
                           BUILDIN );

CV_EXPORTS void buildPyramid( cv::InputArray src, cv::OutputArrayOfArrays dst,
                              int maxlevel, int borderType = cv::BORDER_DEFAULT,
                              BUILDIN );

CV_EXPORTS_W void resize( cv::InputArray src, cv::OutputArray dst,
                          cv::Size dsize, double fx = 0, double fy = 0,
                          int interpolation = cv::INTER_LINEAR,
                          BUILDIN );

CV_EXPORTS_W void medianBlur( cv::InputArray src, cv::OutputArray dst, int ksize,
                              BUILDIN);

CV_EXPORTS_W void blur( cv::InputArray src, cv::OutputArray dst,
                        cv::Size ksize, cv::Point anchor = cv::Point(-1,-1),
                        int borderType = cv::BORDER_DEFAULT,
                        BUILDIN);

CV_EXPORTS_W void GaussianBlur( cv::InputArray src, cv::OutputArray dst, cv::Size ksize,
                                double sigmaX, double sigmaY = 0,
                                int borderType = cv::BORDER_DEFAULT,
                                BUILDIN);

CV_EXPORTS_W void bilateralFilter( cv::InputArray src, cv::OutputArray dst, int d,
                                   double sigmaColor, double sigmaSpace,
                                   int borderType = cv::BORDER_DEFAULT,
                                   BUILDIN);

CV_EXPORTS_W void Sobel( cv::InputArray src, cv::OutputArray dst, int ddepth,
                         int dx, int dy, int ksize = 3,
                         double scale = 1, double delta = 0,
                         int borderType = cv::BORDER_DEFAULT,
                         BUILDIN);

CV_EXPORTS_W void Canny( cv::InputArray image, cv::OutputArray edges,
                         double threshold1, double threshold2,
                         int apertureSize = 3, bool L2gradient = false,
                         BUILDIN);

CV_EXPORTS_W void Canny( cv::InputArray dx, cv::InputArray dy,          // Canny Typ 2
                         cv::OutputArray edges,
                         double threshold1, double threshold2,
                         bool L2gradient = false,
                         BUILDIN);

CV_EXPORTS_W void adaptiveThreshold( cv::InputArray src, cv::OutputArray dst,
                                     double maxValue, int adaptiveMethod,
                                     int thresholdType, int blockSize, double C,
                                     BUILDIN);

CV_EXPORTS_W double threshold( cv::InputArray src, cv::OutputArray dst,
                               double thresh, double maxval, int type,
                               BUILDIN);

CV_EXPORTS_W void cvtColor( cv::InputArray src, cv::OutputArray dst, int code, int dstCn=0,
                            BUILDIN);

CV_EXPORTS_W void dilate( cv::InputArray src, cv::OutputArray dst, cv::InputArray kernel,
                           cv::Point anchor = cv::Point(-1,-1), int iterations = 1,
                           int borderType = cv::BORDER_CONSTANT,
                           const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue(),    // morphologyDefaultBorderValue() = Scalar::all(DBL_MAX)
                           BUILDIN);

CV_EXPORTS_W void erode( cv::InputArray src, cv::OutputArray dst, cv::InputArray kernel,
                         cv::Point anchor = cv::Point(-1,-1), int iterations = 1,
                         int borderType = cv::BORDER_CONSTANT,
                         const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue(),      // morphologyDefaultBorderValue() = Scalar::all(DBL_MAX)
                         BUILDIN);

CV_EXPORTS_W void morphologyEx( cv::InputArray src, cv::OutputArray dst,
                                int op, cv::InputArray kernel,
                                cv::Point anchor = cv::Point(-1,-1), int iterations = 1,
                                int borderType = cv::BORDER_CONSTANT,
                                const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue(),
                                BUILDIN);

CV_EXPORTS_W Mat getStructuringElement(int shape, cv::Size ksize, cv::Point anchor = cv::Point(-1,-1),
                                           BUILDIN);

CV_EXPORTS_W void scaleAdd(cv::InputArray src1, double alpha, cv::InputArray src2, cv::OutputArray dst,
                           BUILDIN);

CV_EXPORTS_W void addWeighted(cv::InputArray src1, double alpha, cv::InputArray src2,
                              double beta, double gamma, cv::OutputArray dst, int dtype = -1,
                              BUILDIN);

CV_EXPORTS_W void convertScaleAbs(cv::InputArray src, cv::OutputArray dst,
                                  double alpha = 1, double beta = 0,
                                  BUILDIN);

CV_EXPORTS_W void findContours( cv::InputOutputArray image,
                                cv::OutputArrayOfArrays contours,
                                cv::OutputArray hierarchy,
                                int mode,
                                int method, cv::Point offset = cv::Point(),
                                BUILDIN);

CV_EXPORTS void findContours( cv::InputOutputArray image, cv::OutputArrayOfArrays contours,     // overload
                              int mode, int method, cv::Point offset = cv::Point(),
                              BUILDIN);

CV_EXPORTS_W void approxPolyDP( cv::InputArray curve,
                                cv::OutputArray approxCurve,
                                double epsilon, bool closed,
                                BUILDIN);

CV_EXPORTS_W void Laplacian( cv::InputArray src, cv::OutputArray dst, int ddepth,
                             int ksize = 1, double scale = 1, double delta = 0,
                             int borderType = cv::BORDER_DEFAULT,
                             BUILDIN);

CV_EXPORTS_W Mat imread( const cv::String& filename, int flags = cv::IMREAD_COLOR,
                             BUILDIN);

CV_EXPORTS_W void normalize( cv::InputArray src, cv::InputOutputArray dst, double alpha = 1, double beta = 0,
                             int norm_type = cv::NORM_L2, int dtype = -1, cv::InputArray mask = cv::noArray(),
                             BUILDIN);

CV_EXPORTS void normalize( const cv::SparseMat& src, cv::SparseMat& dst, double alpha, int normType,
                           BUILDIN);

CV_EXPORTS void calcHist( const cv::Mat* images, int nimages,
                          const int* channels, cv::InputArray mask,
                          cv::OutputArray hist, int dims, const int* histSize,
                          const float** ranges, bool uniform = true, bool accumulate = false,
                          BUILDIN);

CV_EXPORTS_W void HoughCircles( cv::InputArray image, cv::OutputArray circles,
                               int method, double dp, double minDist,
                               double param1 = 100, double param2 = 100,
                               int minRadius = 0, int maxRadius = 0,
                               BUILDIN);

CV_EXPORTS_W void HoughLines( cv::InputArray image, cv::OutputArray lines,
                              double rho, double theta, int threshold,
                              double srn = 0, double stn = 0,
                              double min_theta = 0, double max_theta = CV_PI,
                              BUILDIN);

CV_EXPORTS_W void HoughLinesP( cv::InputArray image, cv::OutputArray lines,
                               double rho, double theta, int threshold,
                               double minLineLength = 0, double maxLineGap = 0,
                               BUILDIN);

CV_EXPORTS_W void Scharr( cv::InputArray src, cv::OutputArray dst, int ddepth,
                          int dx, int dy, double scale = 1, double delta = 0,
                          int borderType = cv::BORDER_DEFAULT,
                          BUILDIN);

//////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief calcOpticalFlowPyrLK
/// \param prevImg
/// \param nextImg
/// \param prevPts
/// \param nextPts
/// \param status
/// \param err
/// \param winSize
/// \param maxLevel
/// \param criteria
/// \param flags
/// \param minEigThreshold
///
CV_EXPORTS_W void calcOpticalFlowPyrLK( cv::InputArray prevImg, cv::InputArray nextImg,
                                        cv::InputArray prevPts, cv::InputOutputArray nextPts,
                                        cv::OutputArray status, cv::OutputArray err,
                                        cv::Size winSize, int maxLevel,
                                        cv::TermCriteria criteria,
                                        int flags, double minEigThreshold
                                        BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::calcOpticalFlowPyrLK( prevImg, nextImg,
                              prevPts, nextPts,
                              status, err,
                              winSize, maxLevel,
                              criteria,
                              flags, minEigThreshold);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for getGaborKernel
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CALCOPTICALFLOWPYRLK, "calcOpticalFlowPyrLK()",
                                PARAMETER,              // Parameter, Source
                                BUILIN_PARA);

        func.push_back( foo );

        struct _point_int_ ws = {winSize.width, 1, 20000, winSize.height, 1, 20000};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ws, "winSize");      // cv::Size winSize

        struct _int_para_ lev = {maxLevel, 0, 1000};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&lev, "maxLevel" );

        struct _enum_para_ fl = {flags, "FlowFlags"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&fl, "FlowFlags" );

        struct _double_para_ met = {minEigThreshold, 0.0, 0.9999, 4};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&met, "minEigThreshold" );
    }

    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------
    try {
        struct _point_int_ *ws = (struct _point_int_ *)foo->para[0]->data;
        cv::calcOpticalFlowPyrLK( prevImg, nextImg,
                                  prevPts, nextPts,
                                  status, err,
                                  cv::Size(ws->x, ws->y),           // ksize
                                  *(int *)foo->para[1]->data,       // maxLevels
                                  criteria,
                                  *(int *)foo->para[2]->data,       // flags enum <FlowFlags>
                                  *(double *)foo->para[3]->data);   // minEigThreshold

    } catch( cv::Exception& e ) {
        foo->error_flag |= FUNC_ERROR;
    }
    foo->control_func_run_time ();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief calcOpticalFlowFarneback
/// \param prev
/// \param next
/// \param flow
/// \param pyr_scale 0.0 ... 0.999
/// \param levels
/// \param winsize
/// \param iterations
/// \param poly_n
/// \param poly_sigma
/// \param flags    see: OPTFLOW_USE_INITIAL_FLOW, OPTFLOW_FARNEBACK_GAUSSIAN
///
CV_EXPORTS_W void calcOpticalFlowFarneback( cv::InputArray prev, cv::InputArray next, cv::InputOutputArray flow,
                                            double pyr_scale, int levels, int winsize,
                                            int iterations, int poly_n, double poly_sigma,
                                            int flags
                                            BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::calcOpticalFlowFarneback( prev, next, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for getGaborKernel
    opencvd_func *foo = nullptr;


    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CALCOPTICALFLOWFARNEBACK, "calcOpticalFlowFarneback()",
                                PARAMETER,              // Parameter, Source
                                BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ ps = {pyr_scale, 0.0, 0.9999, 4};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ps, "pyr_scale" );

        struct _int_para_ lev = {levels, 1, 12};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&lev, "levels" );

        struct _int_para_ ws = {winsize, -40000, 40000};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ws, "winsize" );

        struct _int_para_ ite = {iterations, 1, 100};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ite, "iterations" );

        struct _int_para_ pon = {poly_n, 1, 50};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&pon, "poly_n" );

        struct _double_para_ poly = {poly_sigma, -10.0, 10.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&poly, "poly_sigma" );

        struct _enum_para_ fl = {flags, "FlowFlags"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&fl, "FlowFlags" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------
    try {
        cv::calcOpticalFlowFarneback(prev, next, flow,
                                    *(double *)foo->para[0]->data,      // pyr_scale
                                    *(int *)foo->para[1]->data,         // levels
                                    *(int *)foo->para[2]->data,         // winsize
                                    *(int *)foo->para[3]->data,         // iterations
                                    *(int *)foo->para[4]->data,         // poly_n
                                    *(double *)foo->para[5]->data,      // poly_sigma
                                    *(int *)foo->para[6]->data);        // flags enum <FlowFlags>

    } catch( cv::Exception& e ) {
        foo->error_flag |= FUNC_ERROR;
    }
    foo->control_func_run_time ();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief getDerivKernels
//! \param kx
//! \param ky
//! \param dx Derivative order in respect of x.
//! \param dy Derivative order in respect of y.
//! \param ksize Aperture size. It can be CV_SCHARR=-1, 1, 3, 5, or 7.
//! \param normalize normalize Flag indicating whether to normalize (scale down) the filter coefficients or not.
//!        Theoretically, the coefficients should have the denominator \f$=2^{ksize*2-dx-dy-2}\f$. If you are
//!        going to filter floating-point images, you are likely to use the normalized kernels. But if you
//!        compute derivatives of an 8-bit image, store the results in a 16-bit image, and wish to preserve
//!        all the fractional bits, you may want to set normalize=false .
//! \param ktype Type of filter coefficients. It can be CV_32f or CV_64F .
//!
CV_EXPORTS_W void getDerivKernels( cv::OutputArray kx, cv::OutputArray ky,
                                   int dx, int dy, int ksize,
                                   bool normalize, int ktype
                                   BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::getDerivKernels( kx, ky, dx, dy, ksize, normalize, ktype);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for getGaborKernel
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GETDERIVKERNELS, "getDerivKernels()",
                                PARAMETER,              // Menu
                                BUILIN_PARA);

        func.push_back( foo );

        struct _int_para_ d_x = {dx, -40000, 40000};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&d_x, "dx" );

        struct _int_para_ d_y = {dy, -40000, 40000};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&d_y, "dy" );

        struct _int_para_ sp = {ksize, -1, 7};    // -1, 1, 3, 5, 7 // eventuell ein enum bauen !!!
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "ksize" );

        struct _enum_para_ un = {normalize, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&un, "normalize" );

        struct _enum_para_ dd = {ktype, "filterdepth_CV_32F_CV_64F"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "ktype" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------
    try {
        cv::getDerivKernels(kx, ky,
                            *(int *)foo->para[0]->data,       // dx
                            *(int *)foo->para[1]->data,       // dy
                            *(int *)foo->para[2]->data,       // ksize
                            *(int *)foo->para[3]->data,       // normalize
                            *(int *)foo->para[4]->data);      // ktype
    } catch( cv::Exception& e ) {
        foo->error_flag |= FUNC_ERROR;
    }
    foo->control_func_run_time ();
} // getDerivKernels

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief getGaborKernel
//! \param ksize Size of the filter returned.
//! \param sigma Standard deviation of the gaussian envelope.
//! \param theta Orientation of the normal to the parallel stripes of a Gabor function.
//! \param lambd Wavelength of the sinusoidal factor.
//! \param gamma Spatial aspect ratio.
//! \param psi Phase offset.
//! \param ktype Type of filter coefficients. It can be CV_32F or CV_64F .
//! \return
//!
CV_EXPORTS_W Mat getGaborKernel( cv::Size ksize, double sigma, double theta, double lambd,
                                 double gamma, double psi, int ktype
                                 BUILDIN_FUNC)
{
    if (cvd_off) {
        Mat ret;
        ret = cv::getGaborKernel( ksize, sigma, theta, lambd, gamma, psi, ktype );
        return ret;
    }

    Mat ret;
    static std::vector<opencvd_func *> func{};  // reg vector for getGaborKernel
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GETGABORKERNEL, "getGaborKernel()",
                                PARAMETER,              // Menu
                                BUILIN_PARA);

        func.push_back( foo );

        struct _point_int_ ks = {ksize.width, 1, 20000, ksize.height, 1, 20000};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ks, "ksize");      // Matrix w, h

        struct _double_para_ psp = {sigma, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&psp, "sigma" );

        struct _double_para_ th = {theta, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&th, "theta" );

        struct _double_para_ lm = {lambd, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&lm, "lambd" );

        struct _double_para_ gm = {gamma, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&gm, "gamma" );

        struct _double_para_ ps = {psi, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ps, "psi" );

        struct _enum_para_ dd = {ktype, "filterdepth_CV_32F_CV_64F"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "ktype" );

    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------
    try {
        struct _point_int_ *ks = (struct _point_int_ *)foo->para[0]->data;
        ret = cv::getGaborKernel( cv::Size(ks->x, ks->y),           // ksize
                                 *(double *)foo->para[1]->data,     // sigma
                                 *(double *)foo->para[2]->data,     // theta
                                 *(double *)foo->para[3]->data,     // lambd
                                 *(double *)foo->para[4]->data,     // gamma
                                 *(double *)foo->para[5]->data,     // psi
                                 *(int *)foo->para[6]->data);       // ktype
    } catch( cv::Exception& e ) {
        foo->error_flag |= FUNC_ERROR;
    }
    foo->control_func_run_time ();
    return ret;
} // getGaborKernel

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief getGaussianKernel
//! \param ksize Aperture size. It should be odd ( \f$\texttt{ksize} \mod 2 = 1\f$ ) and positive.
//! \param sigma Gaussian standard deviation. If it is non-positive, it is computed from ksize as
//!              `sigma = 0.3*((ksize-1)*0.5 - 1) + 0.8`.
//! \param ktype Type of filter coefficients. It can be CV_32F or CV_64F .
//! \return filter Mat
//! \example Mat foo = CVD::getGaussianKernel( 5, 3.5 );
CV_EXPORTS_W Mat getGaussianKernel( int ksize, double sigma, int ktype
                                    BUILDIN_FUNC)
{
    if (cvd_off) {
        Mat ret;
        ret = cv::getGaussianKernel( ksize, sigma, ktype );
        return ret;
    }

    Mat ret;
    static std::vector<opencvd_func *> func{};  // reg vector for boxFilter
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GETGAUSSIANKERNEL, "getGaussianKernel()",
                                PARAMETER,              // Menu
                                BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ sp = {ksize, 1, 31};    // 1, 3, 5, 7, ...
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "ksize" );

        struct _double_para_ psp = {sigma, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&psp, "sigma" );

        struct _enum_para_ dd = {ktype, "filterdepth_CV_32F_CV_64F"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "ktype" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------------
    try {
        ret = cv::getGaussianKernel( *(int *)foo->para[0]->data,        // ksize,
                                     *(double *)foo->para[1]->data,     // sigma,
                                     *(int *)foo->para[2]->data);       // ktype
    } catch( cv::Exception& e ) {
        foo->error_flag |= FUNC_ERROR;
    }
    foo->control_func_run_time ();
    return ret;
} // getGaussianKernel

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief sepFilter2D
//! \param src
//! \param dst Destination image of the same size and the same number of channels as src .
//! \param ddepth Destination image depth, see @ref filter_depths "combinations"
//! \param kernelX Coefficients for filtering each row.
//! \param kernelY Coefficients for filtering each column.
//! \param anchor Anchor position within the kernel. The default value \f$(-1,-1)\f$ means that the anchor
//!               is at the kernel center.
//! \param delta Value added to the filtered results before storing them.
//! \param borderType Pixel extrapolation method, see cv::BorderTypes
//! \sa  filter2D, Sobel, GaussianBlur, boxFilter, blur
CV_EXPORTS_W void sepFilter2D( cv::InputArray src, cv::OutputArray dst, int ddepth,
                               cv::InputArray kernelX, cv::InputArray kernelY,
                               cv::Point anchor,
                               double delta , int borderType
                               BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::sepFilter2D( src, dst, ddepth, kernelX, kernelY, anchor, delta, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for boxFilter
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), SEQFILTER2D, "sepFilter2D()",
                                PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                                BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ dd = {ddepth, "Sobel_filterdepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "ddepth" );

        struct _point_int_ ip = {anchor.x, -1, 20000, anchor.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _double_para_ psp = {delta, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&psp, "delta" );

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                struct _point_int_ *ac = (struct _point_int_ *)foo->para[1]->data;      // anchor

                cv::sepFilter2D( src, out,
                              *(int *)foo->para[0]->data,       // ddepth,
                              kernelX, kernelY,
                              cv::Point(ac->x, ac->y),          // anchor,
                              *(double*)foo->para[2]->data,     // delta,
                              *(int *)foo->para[3]->data);      // borderType

            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            // ------------- show break image ----------------
            foo->control_imshow( out );                 // output
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            struct _point_int_ *ac = (struct _point_int_ *)foo->para[1]->data;      // anchor

            cv::sepFilter2D( src, dst,
                          *(int *)foo->para[0]->data,       // ddepth,
                          kernelX, kernelY,
                          cv::Point(ac->x, ac->y),          // anchor,
                          *(double*)foo->para[2]->data,     // delta,
                          *(int *)foo->para[3]->data);      // borderType

        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );  // show Image
} // sepFilter2D

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief filter2D
//! \param src input image.
//! \param dst output image of the same size and the same number of channels as src.
//! \param ddepth desired depth of the destination image, see @ref filter_depths "combinations"
//! \param kernel convolution kernel (or rather a correlation kernel), a single-channel floating point
//!        matrix; if you want to apply different kernels to different channels, split the image into
//!        separate color planes using split and process them individually.
//! \param anchor of the kernel that indicates the relative position of a filtered point within
//!        the kernel; the anchor should lie within the kernel; default value (-1,-1) means that the anchor
//!        is at the kernel center.
//! \param delta optional value added to the filtered pixels before storing them in dst.
//! \param borderType pixel extrapolation method, see cv::BorderTypes
//! \example    Mat_ <char> kernel(1, 5);
//!             kernel << 1, 0, -1;                 // Gradientenfilter
//!             CVD::filter2D( src, dst, CV_32F, kernel);
//!             dst.convertTo( result, CV_8U);
//!
CV_EXPORTS_W void filter2D( cv::InputArray src, cv::OutputArray dst, int ddepth,
                            cv::InputArray kernel, cv::Point anchor,
                            double delta, int borderType
                            BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::filter2D( src, dst, ddepth, kernel, anchor, delta, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for boxFilter
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), FILTER2D, "filter2D()",
                                PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                                BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ dd = {ddepth, "Sobel_filterdepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "ddepth" );

        struct _point_int_ ip = {anchor.x, -1, 20000, anchor.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _double_para_ psp = {delta, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&psp, "delta" );

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                struct _point_int_ *ac = (struct _point_int_ *)foo->para[1]->data;      // anchor

                cv::filter2D( src, out,
                              *(int *)foo->para[0]->data,       // ddepth,
                              kernel,
                              cv::Point(ac->x, ac->y),          // anchor,
                              *(double*)foo->para[2]->data,     // delta,
                              *(int *)foo->para[3]->data);      // borderType

            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            // ------------- show break image ----------------
            foo->control_imshow( out );                 // output
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            struct _point_int_ *ac = (struct _point_int_ *)foo->para[1]->data;      // anchor

            cv::filter2D( src, dst,
                          *(int *)foo->para[0]->data,       // ddepth,
                          kernel,
                          cv::Point(ac->x, ac->y),          // anchor,
                          *(double*)foo->para[2]->data,     // delta,
                          *(int *)foo->para[3]->data);      // borderType

        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );  // show Image
} // filter2D

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief putText
//! \param img
//! \param text
//! \param org
//! \param fontFace Font type, see cv::HersheyFonts.
//! \param fontScale Font scale factor that is multiplied by the font-specific base size.
//! \param color
//! \param thickness
//! \param lineType
//! \param bottomLeftOrigin When true, the image data origin is at the bottom-left corner. Otherwise,
//!                         it is at the top-left corner.
//! \example CVD::putText (a, "text output", cv::Point (20, 20), cv::FONT_HERSHEY_PLAIN, 1.3, cv::Scalar(255, 0, 0), 2);
//!
CV_EXPORTS_W void putText( cv::InputOutputArray img, const cv::String& text, cv::Point org,
                         int fontFace, double fontScale, cv::Scalar color,
                         int thickness, int lineType,
                         bool bottomLeftOrigin
                         BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::putText( img, text, org, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for boxFilter
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), PUTTEXT, "putText()",
                                PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                                BUILIN_PARA);
        func.push_back( foo );

        struct _point_int_ ip = {org.x, -1, 20000, org.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "Point");

        struct _enum_para_ un = {fontFace, "HersheyFonts"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&un, "fontFace" );

        struct _double_para_ psp = {fontScale, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&psp, "fontScale" );

        struct _scalar_double_ sc = {color.val[0], color.val[1], color.val[2], color.val[3]};
        foo->new_para ( SCALAR_PARA, sizeof(struct _scalar_double_), (uint8_t*)&sc, "color" );

        struct _int_para_ tn = {thickness, 1, 255};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&tn, "thickness" );

        struct _enum_para_ lt = {lineType, "LineTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&lt, "lineType" );

        struct _enum_para_ bo = {bottomLeftOrigin, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bo, "bottomLeftOrigin" );

    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out (img.rows(), img.cols(), img.type(), cv::Scalar(255, 255, 255));
            try {
                struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
                struct _scalar_double_ *sc = (struct _scalar_double_*) foo->para[3]->data;

                cv::putText( out, text,
                             cv::Point(ip->x, ip->y),           // Point,
                             *(int*)foo->para[1]->data,         // fontFace,
                             *(double*)foo->para[2]->data,      // fontScale
                             cv::Scalar(sc->val[0], sc->val[1], sc->val[2], sc->val[3] ),    // color
                             *(int*)foo->para[4]->data,         // thickness,
                             *(int*)foo->para[5]->data,         // lineType,
                             *(int*)foo->para[6]->data);        // bottomLeftOrigin

            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            // ------------- show break image ----------------
            foo->control_imshow( out );                 // output
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        // nothing to do, no text
    } else {
        try {
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            struct _scalar_double_ *sc = (struct _scalar_double_*) foo->para[3]->data;

            cv::putText( img, text,
                         cv::Point(ip->x, ip->y),           // Point,
                         *(int*)foo->para[1]->data,         // fontFace,
                         *(double*)foo->para[2]->data,      // fontScale
                         cv::Scalar(sc->val[0], sc->val[1], sc->val[2], sc->val[3] ),    // color
                         *(int*)foo->para[4]->data,         // thickness,
                         *(int*)foo->para[5]->data,         // lineType,
                         *(int*)foo->para[6]->data);        // bottomLeftOrigin

        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( img );  // show Image
} // putText

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief box_Filter
//! \param src
//! \param dst
//! \param ddepth
//! \param ksize
//! \param anchor
//! \param normalize
//! \param borderType
//! \param para_type
//!
void box_Filter( cv::InputArray src, cv::OutputArray dst,
                 int ddepth,            // -1, CV_8U, CV_16U, ...
                 cv::Size ksize, cv::Point anchor,
                 bool normalize,
                 int borderType,
                 int para_type,          // BOXFILTER or SQRBOXFILTER
                 int line_nr,
                 const char *src_file)
{
    if (cvd_off) {
        if (para_type == BOXFILTER)
            cv::boxFilter( src, dst, ddepth, ksize, anchor, normalize, borderType );
        if (para_type == SQRBOXFILTER)
            cv::sqrBoxFilter( src, dst, ddepth, ksize, anchor, normalize, borderType );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for boxFilter
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0),
                               para_type,
                               (para_type == BOXFILTER) ? "boxFilter()" : "sqrboxFilter()",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        struct _enum_para_ dd = {ddepth, "Sobel_filterdepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "ddepth" );

        struct _point_int_ ks = {ksize.width, 1, 20000, ksize.height, 1, 20000};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ks, "ksize");      // Matrix w, h

        struct _point_int_ ip = {anchor.x, -1, 20000, anchor.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _enum_para_ un = {normalize, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&un, "normalize" );

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                struct _point_int_ *ks = (struct _point_int_ *)foo->para[1]->data;
                struct _point_int_ *ip = (struct _point_int_ *)foo->para[2]->data;
                if (para_type == BOXFILTER)
                    cv::boxFilter( src, out,
                                   *(int *)foo->para[0]->data,      // ddepth
                                   cv::Size(ks->x, ks->y),          // ksize
                                   cv::Point(ip->x, ip->y),         // anchor
                                   *(int*)foo->para[3]->data,       // normalize
                                   *(int*)foo->para[4]->data);      // borderType

                if (para_type == SQRBOXFILTER)
                    cv::sqrBoxFilter( src, out,
                                   *(int *)foo->para[0]->data,      // ddepth
                                   cv::Size(ks->x, ks->y),          // ksize
                                   cv::Point(ip->x, ip->y),         // anchor
                                   *(int*)foo->para[3]->data,       // normalize
                                   *(int*)foo->para[4]->data);      // borderType

            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            // ------------- show image ----------------
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            struct _point_int_ *ks = (struct _point_int_ *)foo->para[1]->data;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[2]->data;
            if (para_type == BOXFILTER)
                cv::boxFilter( src, dst,
                               *(int *)foo->para[0]->data,      // ddepth
                               cv::Size(ks->x, ks->y),          // ksize
                               cv::Point(ip->x, ip->y),         // anchor
                               *(int*)foo->para[3]->data,       // normalize
                               *(int*)foo->para[4]->data);      // borderType
            if (para_type == SQRBOXFILTER)
                cv::sqrBoxFilter( src, dst,
                                  *(int *)foo->para[0]->data,      // ddepth
                                  cv::Size(ks->x, ks->y),          // ksize
                                  cv::Point(ip->x, ip->y),         // anchor
                                  *(int*)foo->para[3]->data,       // normalize
                                  *(int*)foo->para[4]->data);      // borderType

        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );  // show Image
} // box_Filter

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief boxFilter
//! \param src input image
//! \param dst output image of the same size and type as src.
//! \param ddepth the output image depth (-1 to use src.depth()).
//! \param ksize kernel size
//! \param anchor kernel anchor point. The default value of Point(-1, -1) denotes that the anchor is at the kernel
//!        center.
//! \param normalize flag, specifying whether the kernel is to be normalized by it's area or not.
//! \param borderType border mode used to extrapolate pixels outside of the image, see cv::BorderTypes
//! \sa sqrBoxFilter
//! \example    CVD::boxFilter (src, dst, -1, cv::Size(3, 6));
//!
CV_EXPORTS_W void boxFilter( cv::InputArray src, cv::OutputArray dst, int ddepth,
                             cv::Size ksize, cv::Point anchor,
                             bool normalize,
                             int borderType
                             BUILDIN_FUNC)
{
    box_Filter (src, dst, ddepth, ksize, anchor, normalize, borderType, BOXFILTER, line_nr, src_file);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief sqrBoxFilter
//! \param _src
//! \param _dst output image of the same size and type as _src
//! \param ddepth the output image depth (-1 to use src.depth())
//! \param ksize kernel size
//! \param anchor kernel anchor point. The default value of Point(-1, -1) denotes that the anchor is at the kernel
//!        center.
//! \param normalize flag, specifying whether the kernel is to be normalized by it's area or not.
//! \param borderType border mode used to extrapolate pixels outside of the image, see cv::BorderTypes
//! \sa boxFilter
//! \example    CVD::sqrBoxFilter (src, dst, -1, cv::Size(3, 3));   // filtert sehr stark
//!
CV_EXPORTS_W void sqrBoxFilter( cv::InputArray _src, cv::OutputArray _dst, int ddepth,
                                cv::Size ksize, cv::Point anchor,
                                bool normalize,
                                int borderType
                                BUILDIN_FUNC)
{
    box_Filter (_src, _dst, ddepth, ksize, anchor, normalize, borderType, SQRBOXFILTER, line_nr, src_file);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief matchShapes
//! \param contour1 First contour or grayscale image.
//! \param contour2 Second contour or grayscale image.
//! \param method Comparison method, see cv::ShapeMatchModes
//! \param parameter Method-specific parameter (not supported now).
//! \return 0..1
//!
CV_EXPORTS_W double matchShapes( cv::InputArray contour1, cv::InputArray contour2,
                                 int method, double parameter
                                 BUILDIN_FUNC)
{
    if (cvd_off) {
        return cv::matchShapes(contour1, contour2, method, parameter);
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;
    double ret = 1.0;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MATCHSHAPES, "matchShapes()",
                               PARAMETER | FUNC_OFF,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ mt = {method, "ShapeMatchModes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&mt, "method" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    // no break func
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        // do nothing. matchShapes = 1.0
    } else {
        try {
            ret = cv::matchShapes(contour1, contour2,
                                  *(int *)foo->para[0]->data,      // method
                                  parameter);

        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }

    return ret;
} // matchShapes

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief show_grabcut_result function used by grabCut()
//! \param foo
//! \param img
//! \param mask
//! \param r = cv::Rect
//!
void show_grabcut_result (opencvd_func *foo, cv::InputArray img, cv::InputOutputArray mask, cv::Rect r)
{
    cv::Mat result_mask;
    cv::compare( mask, cv::Scalar(cv::GC_PR_FGD), result_mask, cv::CMP_EQ );
    cv::Mat result;
    img.copyTo( result, result_mask );                            // result herausfiltern.
    cv::addWeighted(img, 0.5, result, 1.0, 1.0, result);
    cv::rectangle(result, r, cv::Scalar(255, 255, 255), 1);
    foo->control_imshow( result );  // show Image
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief grabCut
//! \param img Input 8-bit 3-channel image.
//! \param mask Input/output 8-bit single-channel mask. The mask is initialized by the function when
//!             mode is set to GC_INIT_WITH_RECT. Its elements may have one of the cv::GrabCutClasses.
//! \param rect
//! \param bgdModel Temporary array for the background model. Do not modify it while you are
//!                 processing the same image.
//! \param fgdModel Temporary arrays for the foreground model. Do not modify it while you are
//!                 processing the same image.
//! \param iterCount iterCount Number of iterations the algorithm should make before returning the result. Note
//!                  that the result can be refined with further calls with mode==GC_INIT_WITH_MASK or
//!                  mode==GC_EVAL .
//! \param mode Operation mode that could be one of the cv::GrabCutModes
//!
CV_EXPORTS_W void grabCut( cv::InputArray img, cv::InputOutputArray mask, cv::Rect rect,
                           cv::InputOutputArray bgdModel, cv::InputOutputArray fgdModel,
                           int iterCount, int mode
                           BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::grabCut(img, mask, rect, bgdModel, fgdModel, iterCount, mode);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GRABCUT, "grabCut()",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _rect_int_ ri = {(int)rect.x, (int)rect.y, (int)rect.width, (int)rect.height, -100000, 100000};
        foo->new_para (RECT_INT_PARA, sizeof(struct _rect_int_), (uint8_t*)&ri, "Rect(x,y,w,h)" );

        struct _int_para_ ms = {iterCount, -65536, 65536};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ms, "iterCount" );

        struct _enum_para_ mt = {mode, "GrabCutModes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&mt, "mode" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            struct _rect_int_ *rec_data = (struct _rect_int_ *)foo->para[0]->data;
            cv::Rect r(rec_data->x, rec_data->y, rec_data->w, rec_data->h);

            cv::Mat out_mask;
            try {
                cv::grabCut(img, out_mask,
                            r,                              // Rect
                            bgdModel, fgdModel,
                            *(int *)foo->para[1]->data,     // iterCount
                            *(int *)foo->para[2]->data);    // mode

            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            // ------------- show image ----------------
            show_grabcut_result (foo, img, out_mask, r);
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    struct _rect_int_ *rec_data = (struct _rect_int_ *)foo->para[0]->data;
    cv::Rect r(rec_data->x, rec_data->y, rec_data->w, rec_data->h);

    if (foo->state.flag.func_off) {
        // do nothing
    } else {
        try {
            cv::grabCut(img, mask,
                        r,                              // rect
                        bgdModel, fgdModel,
                        *(int *)foo->para[1]->data,     // iterCount
                        *(int *)foo->para[2]->data);    // mode

        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    // ------------- show image ----------------
    show_grabcut_result (foo, img, mask, r);
} // grabCut

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief matchTemplate
//! \param image
//! \param templ
//! \param result
//! \param method
//! \param mask
//!
CV_EXPORTS_W void matchTemplate( cv::InputArray image, cv::InputArray templ,
                                 cv::OutputArray result, int method, cv::InputArray mask
                                 BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::matchTemplate( image, templ, result, method, mask);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MATCHTEMPLATE, "matchTemplate()",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ mt = {method, "TemplateMatchModes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&mt, "method" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::matchTemplate( image, templ, out,
                                   *(int*)foo->para[0]->data,
                                   mask);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        result.create(cv::Size(image.cols()-templ.cols()+1,
                               image.rows()-templ.rows()+1),
                      CV_32FC1);
        result.setTo(cv::Scalar::all(0));           // fill the result with zero
    } else {
        try {
            cv::matchTemplate( image, templ, result,
                               *(int*)foo->para[0]->data,
                               mask);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( result );  // show Image
} // matchTemplate

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief distanceTransform
//! \param src
//! \param dst
//! \param distanceType Type of distance, see cv::DistanceTypes
//! \param maskSize Size of the distance transform mask, see cv::DistanceTransformMasks. In case of the
//!                 DIST_L1 or DIST_C distance type, the parameter is forced to 3 because a \f$3\times 3\f$ mask gives
//!                 the same result as \f$5\times 5\f$ or any larger aperture.
//! \param dstType Type of output image. It can be CV_8U or CV_32F. Type CV_8U can be used only for
//!                 the first variant of the function and distanceType == DIST_L1.
//!
CV_EXPORTS_W void distanceTransform( cv::InputArray src, cv::OutputArray dst,
                                     int distanceType, int maskSize, int dstType
                                     BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::distanceTransform( src, dst, distanceType, maskSize, dstType );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), DISTANCETRANSFORM, "distanceTransform()",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ dt = {distanceType, "DistanceTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dt, "distanceType" );

        struct _int_para_ ms = {maskSize, -65536, 65536};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ms, "maskSize" );

        struct _enum_para_ dst = {dstType, "depth_for_distanceTransform"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dst, "dstType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::distanceTransform (src, out,
                                       *(int*)foo->para[0]->data,
                                       *(int*)foo->para[1]->data,
                                       *(int*)foo->para[2]->data);
                // cv::normalize(out, out, 0, 1.0, cv::NORM_MINMAX);       // A normalized image better represents the result.
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            cv::distanceTransform (src, dst,
                                       *(int*)foo->para[0]->data,        //
                                       *(int*)foo->para[1]->data,        //
                                       *(int*)foo->para[2]->data);       //
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );  // show Image
} // distanceTransform

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief pyrMeanShiftFiltering
//! \param src The source 8-bit, 3-channel image. CV_8UC3
//! \param dst The destination image of the same format and the same size as the source.
//! \param sp The spatial window radius.
//! \param sr The color window radius.
//! \param maxLevel Maximum level of the pyramid for the segmentation.
//! \param termcrit Termination criteria: when to stop meanshift iterations (Durchlaufe).
//!
CV_EXPORTS_W void pyrMeanShiftFiltering( cv::InputArray src, cv::OutputArray dst,
                                         double sp, double sr, int maxLevel,
                                         cv::TermCriteria termcrit
                                         BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::pyrMeanShiftFiltering (src, dst, sp, sr, maxLevel, termcrit );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), PYRMEANSHIFTFILTERING, "pyrMeanShiftFiltering()",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ psp = {sp, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&psp, "sp" );

        struct _double_para_ psr = {sr, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&psr, "sr" );

        struct _int_para_ ml = {maxLevel, 0, 65536};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ml, "maxLevel" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::pyrMeanShiftFiltering (src, out,
                                           *(double*)foo->para[0]->data,        // sp,
                                           *(double*)foo->para[1]->data,        // sr,
                                           *(int*)foo->para[2]->data,           // maxLevel,
                                           termcrit );
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            cv::pyrMeanShiftFiltering (src, dst,
                                       *(double*)foo->para[0]->data,        // sp,
                                       *(double*)foo->para[1]->data,        // sr,
                                       *(int*)foo->para[2]->data,           // maxLevel,
                                       termcrit );
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );  // show Image
} // pyrMeanShiftFiltering

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief createLineSegmentDetector
//! \param _refine
//! \param _scale
//! \param _sigma_scale
//! \param _quant
//! \param _ang_th
//! \param _log_eps
//! \param _density_th
//! \param _n_bins
//! \return
//!
CV_EXPORTS_W cv::Ptr<cv::LineSegmentDetector> createLineSegmentDetector(
                        int _refine, double _scale,
                        double _sigma_scale, double _quant, double _ang_th ,
                        double _log_eps, double _density_th, int _n_bins
                        BUILDIN_FUNC)
{
    if (cvd_off) {
        return cv::createLineSegmentDetector( _refine, _scale,
                                              _sigma_scale, _quant, _ang_th,
                                              _log_eps, _density_th, _n_bins );        
    }

    cv::Ptr<cv::LineSegmentDetector> ret; //  = nullptr;

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CREATELINESEGMENTDETECTOR, "createLineSegmentDetector()",
                               PARAMETER | FUNC_OFF | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ rf = {_refine, "LineSegmentDetectorModes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&rf, "_refine" );

        struct _double_para_ sc = {_scale, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sc, "_scale" );

        struct _double_para_ sic = {_sigma_scale, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sic, "_sigma_scale" );

        struct _double_para_ qa = {_quant, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&qa, "_quant" );

        struct _double_para_ at = {_ang_th, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&at, "_ang_th" );

        struct _double_para_ le = {_log_eps, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&le, "_log_eps" );

        struct _double_para_ dth = {_density_th, -100000.0, 100000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dth, "_density_th" );

        struct _int_para_ nb = {_n_bins, 0, 65536};
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&nb, "_n_bins" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------


    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        return cv::createLineSegmentDetector( _refine, _scale,
                                              _sigma_scale, _quant, _ang_th,
                                              _log_eps, _density_th, _n_bins );
    } else {
        try {
            ret = cv::createLineSegmentDetector( *(int*)foo->para[0]->data,      // _refine,
                                                 *(double*)foo->para[1]->data,      // _scale,
                                                 *(double*)foo->para[2]->data,      // _sigma_scale,
                                                 *(double*)foo->para[3]->data,      // _quant,
                                                 *(double*)foo->para[4]->data,      // _ang_th,
                                                 *(double*)foo->para[5]->data,      // _log_eps,
                                                 *(double*)foo->para[6]->data,      // _density_th,
                                                 *(int*)foo->para[7]->data);      // _n_bins
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    // foo->control_imshow( img );  // show Image

    return ret;
} // createLineSegmentDetector

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief rectangle
//! \param img
//! \param pt1
//! \param pt2
//! \param color
//! \param thickness
//! \param lineType
//! \param shift
//!
CV_EXPORTS_W void rectangle(cv::InputOutputArray img, cv::Point pt1, cv::Point pt2,
                          const cv::Scalar& color, int thickness,
                          int lineType, int shift
                          BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::rectangle( img, pt1, pt2, color, thickness, lineType, shift );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), RECTANGLE_2, "rectangle(pt1, pt2)",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ tn = {thickness, -1, 10000};     // -1 = fill rec;  0 = thickness 1
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&tn, "thickness" );

        struct _enum_para_ lt = {lineType, "LineTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&lt, "lineType" );

        struct _int_para_ sh = {shift, 0, 16};      // ab 17 gibt es ein error
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sh, "shift" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            img.copyTo( out );
            try {
                cv::rectangle( out, pt1, pt2, color,
                                *(int*)foo->para[0]->data,      // thickness
                                *(int*)foo->para[1]->data,      // lineType
                                *(int*)foo->para[2]->data);     // shift
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        // do nothing
    } else {
        try {
            cv::rectangle( img, pt1, pt2, color,
                            *(int*)foo->para[0]->data,      // thickness
                            *(int*)foo->para[1]->data,      // lineType
                            *(int*)foo->para[2]->data);     // shift
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( img );  // show Image
} // rectangle

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief rectangle
//! \param img
//! \param rec
//! \param color
//! \param thickness Thickness of lines that make up the rectangle. Negative values, like CV_FILLED ,
//!        mean that the function has to draw a filled rectangle.
//! \param lineType Type of the line. See the line description.
//! \param shift Number of fractional bits in the point coordinates.
//!
CV_EXPORTS void rectangle(CV_IN_OUT cv::Mat& img, cv::Rect rec,
                          const cv::Scalar& color, int thickness,
                          int lineType, int shift
                          BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::rectangle( img, rec, color, thickness, lineType, shift );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), RECTANGLE_1, "rectangle(rec)",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ tn = {thickness, -1, 10000};     // -1 = fill rec;  0 = thickness 1
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&tn, "thickness" );

        struct _enum_para_ lt = {lineType, "LineTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&lt, "lineType" );

        struct _int_para_ sh = {shift, 0, 16};      // ab 17 gibt es ein error
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sh, "shift" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out = img;
            try {
                cv::rectangle( out, rec, color,
                                *(int*)foo->para[0]->data,      // thickness
                                *(int*)foo->para[1]->data,      // lineType
                                *(int*)foo->para[2]->data);     // shift
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        // do nothing
    } else {
        try {
            cv::rectangle( img, rec, color,
                            *(int*)foo->para[0]->data,      // thickness
                            *(int*)foo->para[1]->data,      // lineType
                            *(int*)foo->para[2]->data);     // shift
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( img );  // show Image
} // rectangle

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief fitLine
//! \param points
//! \param line Output line parameters. In case of 2D fitting, it should be a vector of 4 elements
//!         (like Vec4f) - (vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and
//!         (x0, y0) is a point on the line. In case of 3D fitting, it should be a vector of 6 elements (like
//!         Vec6f) - (vx, vy, vz, x0, y0, z0), where (vx, vy, vz) is a normalized vector collinear to the line
//!         and (x0, y0, z0) is a point on the line.
//! \param distType Distance used by the M-estimator, see cv::DistanceTypes
//! \param param Numerical parameter ( C ) for some types of distances. If it is 0, an optimal value
//! \param reps Sufficient accuracy for the radius (distance between the coordinate origin and the line).
//! \param aeps Sufficient accuracy for the angle. 0.01 would be a good default value for reps and aeps.
//! \example    std::vector<cv::Point> points;
//!             ...
//!             points.push_back( cv::Point{x, y} );    // die Liste muss mit Punkten gefuellt werden !
//!             ...
//!             cv::Vec4f line;
//!             CVD::fitLine( points, line, CV_DIST_L12, 0, 0.001, 0.001);
//!
CV_EXPORTS_W void fitLine( cv::InputArray points, cv::OutputArray line, int distType,
                           double param, double reps, double aeps
                           BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::fitLine( points, line, distType, param, reps, aeps );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), FITLINE, "fitLine",
                               PARAMETER | FUNC_OFF,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ bt = {distType, "DistanceTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "distType" );

        struct _double_para_ pa = {param, -100000.0, 100000.0, 2};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&pa, "param" );

        struct _double_para_ re = {reps, -100000.0, 100000.0, 4};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&re, "reps" );

        struct _double_para_ ae = {aeps, -100000.0, 100000.0, 4};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ae, "aeps" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    // NO break
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        // do nothing
    } else {
        try {
            cv::fitLine( points, line,
                            *(int*)foo->para[0]->data,          // distType
                            *(double*)foo->para[1]->data,       // param
                            *(double*)foo->para[2]->data,       // reps
                            *(int*)foo->para[3]->data);         // aeps
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    // foo->control_imshow( line );  // NO show Image
} // fitLine

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cornerEigenVal (cv::InputArray src, cv::OutputArray dst,
                     int blockSize, int ksize,
                     int borderType,
                     int para_type,          // CORNERMINEIGENVAL | COREREIGENVALANDVECS
                     int line_nr,
                     const char *src_file)
{
    if (cvd_off) {
        if (para_type == CORNERMINEIGENVAL)
            cv::cornerMinEigenVal(src, dst, blockSize, ksize, borderType);
        if (para_type == COREREIGENVALANDVECS)
            cv::cornerEigenValsAndVecs(src, dst, blockSize, ksize, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for boxFilter
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0),
                               para_type,
                               (para_type == CORNERMINEIGENVAL) ? "cornerMinEigenVal()" : "cornerEigenValsAndVecs()",
                               PARAMETER | FUNC_OFF,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        struct _int_para_ bs = {blockSize, 0, 255};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&bs, "blockSize");

        struct _int_para_ ks = {ksize, 1, 31};
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&ks, "ksize" );

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    // NO break
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            if (para_type == CORNERMINEIGENVAL) {
                cv::cornerMinEigenVal( src, dst,
                                       *(int*)foo->para[0]->data,          // blockSize
                                       *(int*)foo->para[1]->data,          // ksize
                                       *(int*)foo->para[2]->data);         // borderType
            }
            if (para_type == COREREIGENVALANDVECS) {
                cv::cornerEigenValsAndVecs( src, dst,
                                            *(int*)foo->para[0]->data,          // blockSize
                                            *(int*)foo->para[1]->data,          // ksize
                                            *(int*)foo->para[2]->data);         // borderType
            }
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
} // cornerEigenVal

////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief cornerMinEigenVal
//! @param src Input single-channel 8-bit or floating-point image.
//! @param dst Image to store the minimal eigenvalues. It has the type CV_32FC1 and the same size as
//!        src .
//! @param blockSize Neighborhood size (see the details on cornerEigenValsAndVecs ).
//! @param ksize Aperture parameter for the Sobel operator.
//! @param borderType Pixel extrapolation method. See cv::BorderTypes.
//!
CV_EXPORTS_W void cornerMinEigenVal( cv::InputArray src, cv::OutputArray dst,
                                     int blockSize, int ksize,
                                     int borderType
                                     BUILDIN_FUNC )
{
    cornerEigenVal (src, dst, blockSize, ksize, borderType, CORNERMINEIGENVAL, line_nr, src_file);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief cornerEigenValsAndVecs
//! @param src Input single-channel 8-bit or floating-point image.
//! @param dst Image to store the results. It has the same size as src and the type CV_32FC(6) .
//! @param blockSize Neighborhood size (see details below).
//! @param ksize Aperture parameter for the Sobel operator.
//! @param borderType Pixel extrapolation method. See cv::BorderTypes.
//! \see https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html#cornereigenvalsandvecs
//!
CV_EXPORTS_W void cornerEigenValsAndVecs( cv::InputArray src, cv::OutputArray dst,
                                          int blockSize, int ksize,
                                          int borderType
                                          BUILDIN_FUNC )
{
    cornerEigenVal (src, dst, blockSize, ksize, borderType, COREREIGENVALANDVECS, line_nr, src_file);
}
//!
//! \brief cornerSubPix
//! \param image Input image.
//! \param corners corners Initial coordinates of the input corners and refined coordinates provided for output.
//! \param winSize Half of the side length of the search window. For example, if winSize=Size(5,5) ,
//!        then a \f$5*2+1 \times 5*2+1 = 11 \times 11\f$ search window is used.
//! \param zeroZone Half of the size of the dead region in the middle of the search zone over which
//!        the summation in the formula below is not done. It is used sometimes to avoid possible
//!        singularities of the autocorrelation matrix. The value of (-1,-1) indicates that there is no such
//!        a size.
//! \param criteria Criteria for termination of the iterative process of corner refinement. That is,
//!        the process of corner position refinement stops either after criteria.maxCount iterations or when
//!        the corner position moves by less than criteria.epsilon on some iteration.
//!
CV_EXPORTS_W void cornerSubPix( cv::InputArray image, cv::InputOutputArray corners,
                                cv::Size winSize, cv::Size zeroZone,
                                cv::TermCriteria criteria
                                BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::cornerSubPix( image, corners, winSize, zeroZone, criteria);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CORNERSUBPIX, "cornerSubPix",
                               PARAMETER | FUNC_OFF,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _point_int_ ws = {winSize.width, 0, 20000, winSize.height, 0, 20000};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ws, "winSize");      // winSize

        struct _point_int_ zz = {zeroZone.width, -1, 20000, zeroZone.height, -1, 20000};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&zz, "winSize");      // zeroZone

    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    // NO break
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        // nothing to do
    } else {
        try {
            struct _point_int_ *ws = (struct _point_int_ *)foo->para[0]->data;
            struct _point_int_ *zz = (struct _point_int_ *)foo->para[1]->data;
            cv::cornerSubPix( image, corners,
                              cv::Size(ws->x, ws->y),           // winSize
                              cv::Size(zz->x, zz->y),           // zeroZone
                              criteria);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
}

//!
//! \brief preCornerDetect
//! \param src Source single-channel 8-bit of floating-point image.
//! \param dst Output image that has the type CV_32F and the same size as src .
//! \param ksize %Aperture size of the Sobel .
//! \param borderType Pixel extrapolation method. See cv::BorderTypes.
//!
CV_EXPORTS_W void preCornerDetect( cv::InputArray src, cv::OutputArray dst, int ksize,
                                   int borderType
                                   BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::preCornerDetect( src, dst, ksize, borderType );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), PRECORNERDETECT, "preCornerDetect",
                               PARAMETER | FUNC_OFF,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ ks = {ksize, 1, 31};
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&ks, "ksize" );

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    // NO break
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        // nothing to do
    } else {
        try {
            cv::preCornerDetect( src, dst,
                                *(int*)foo->para[0]->data,          // ksize
                                *(int*)foo->para[1]->data);         // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief cornerHarris
//! \param src Input single-channel 8-bit or floating-point image.
//! \param dst Image to store the Harris detector responses. It has the type CV_32FC1 and the same
//!        size as src .
//! \param blockSize Neighborhood size (see the details on cornerEigenValsAndVecs ).
//! \param ksize Aperture parameter for the Sobel operator.
//! \param k Harris detector free parameter. See the formula below.
//! \param borderType
//!
CV_EXPORTS_W void cornerHarris( cv::InputArray src, cv::OutputArray dst, int blockSize,
                                int ksize, double k,
                                int borderType
                                BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::cornerHarris( src, dst, blockSize, ksize, k, borderType );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CORNERHARRIS, "cornerHarris",
                               PARAMETER | FUNC_OFF,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ bs = {blockSize, 0, 255};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&bs, "blockSize");

        struct _int_para_ ks = {ksize, 1, 31};
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&ks, "ksize" );

        struct _double_para_ kp = {k, -1000.0, 1000.0, 2};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&kp, "k" );

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    // NO break
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            cv::cornerHarris( src, dst,
                            *(int*)foo->para[0]->data,          // blockSize
                            *(int*)foo->para[1]->data,          // ksize
                            *(double*)foo->para[2]->data,       // k
                            *(int*)foo->para[3]->data);         // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    // foo->control_imshow( dst ); // NO show Image
} // cornerHarris

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief pyrUp
//! \param src
//! \param dst
//! \param dstsize
//! \param borderType
//! \param line_nr
//! \param src_file
//!
CV_EXPORTS_W void pyrUp( cv::InputArray src, cv::OutputArray dst,
                           const cv::Size& dstsize, int borderType
                           BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::pyrUp (src, dst, dstsize, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), PYRUP, "pyrUp",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu 0x000F
                               BUILIN_PARA);
        func.push_back( foo );

        struct _point_int_ ip = {dstsize.width, 0, 0xFFFF, dstsize.height, 0, 0xFFFF};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "dstsize");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            try {
                cv::pyrUp( src, out,
                             cv::Size(ip->x, ip->y),
                            *(int*)foo->para[1]->data);          // borderType
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        try {
            cv::pyrUp( src, dst,
                         cv::Size(ip->x, ip->y),
                        *(int*)foo->para[1]->data);          // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief buildPyramid
//! \param src
//! \param dst
//! \param maxlevel
//! \param borderType
//!
CV_EXPORTS void buildPyramid( cv::InputArray src, cv::OutputArrayOfArrays dst,
                              int maxlevel, int borderType
                              BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::buildPyramid (src, dst, maxlevel, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrDown
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), BUILDPYRAMID, "buildPyramid",
                               PARAMETER | SHOW_IMAGE | FUNC_OFF | BREAK,    // Menu 0x000F
                               BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ ml = {maxlevel, 0, 255};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ml, "maxlevel");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            std::vector<cv::Mat> out;
            try {
                cv::buildPyramid( src, out,
                                  *(int*)foo->para[0]->data,        // maxlevel
                                  *(int*)foo->para[1]->data);       // borderType
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out[ *(int*)foo->para[0]->data] );     // show maxlevel image
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        dst.create( *(int*)foo->para[0]->data + 1, 1, 0 );      // ist das korrekt ???
        /*
        for (int i=0; i <= *(int*)foo->para[0]->data; i++) {
            cv::Mat a = dst.getMat(i);
            src.copyTo(a);                                      // hat keine Wirkung
        }
        */
    } else {
        try {
            cv::buildPyramid( src, dst,
                              *(int*)foo->para[0]->data,        // maxlevel
                              *(int*)foo->para[1]->data);       // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    cv::Mat a = dst.getMat( *(int*)foo->para[0]->data );    // get maxlevel image
    foo->control_imshow( a );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief pyrDown
//! \param src
//! \param dst
//! \param dstsize
//! \param borderType
//! \param line_nr
//! \param src_file
//!
CV_EXPORTS_W void pyrDown( cv::InputArray src, cv::OutputArray dst,
                           const cv::Size& dstsize, int borderType
                           BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::pyrDown (src, dst, dstsize, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrDown
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), PYRDOWN, "pyrDown",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu 0x000F
                               BUILIN_PARA);
        func.push_back( foo );

        struct _point_int_ ip = {dstsize.width, 0, 0xFFFF, dstsize.height, 0, 0xFFFF};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "dstsize");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            try {
                cv::pyrDown( src, out,
                             cv::Size(ip->x, ip->y),            // Size
                            *(int*)foo->para[1]->data);         // borderType
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );              // dst = src
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        try {
            cv::pyrDown( src, dst,
                         cv::Size(ip->x, ip->y),            // Size
                        *(int*)foo->para[1]->data);         // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief calcHist
//!        Calculates a histogram of a set of arrays.
//! \param images
//! \param nimages      Number of source images.
//! \param channels
//! \param mask
//! \param hist
//! \param dims         Histogram dimensionality that must be positive and not greater than CV_MAX_DIMS
//!                     (equal to 32 in the current OpenCV version).
//! \param histSize
//! \param ranges
//! \param uniform
//! \param accumulate
//!
//! \todo overload functions still need to be implemented.
//!
CV_EXPORTS void calcHist( const cv::Mat* images, int nimages,
                          const int* channels, cv::InputArray mask,
                          cv::OutputArray hist, int dims,
                          const int* histSize, const float** ranges,
                          bool uniform, bool accumulate
                          BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::calcHist (images, nimages, channels, mask, hist, dims, histSize, ranges, uniform, accumulate);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for calcHist
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CALCHIST, "calcHist",
                               0x0001,              // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
                               BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ ni = {nimages, 0, 255};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ni, "nimages");

        struct _int_para_ di = {dims, 0, 255};              // Wertebereich 0 ... CV_MAX_DIMS
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&di, "dims");

        struct _enum_para_ un = {uniform, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&un, "uniform" );

        struct _enum_para_ ac = {accumulate, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ac, "accumulate" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    try {
        cv::calcHist (images,
                      *(int*)foo->para[0]->data,    // nimages
                      channels, mask, hist,
                      *(int*)foo->para[1]->data,    // dims
                      histSize, ranges,
                      *(int*)foo->para[2]->data,    // uniform
                      *(int*)foo->para[3]->data);   // accumulate
    } catch( cv::Exception& e ) {
        foo->error_flag |= FUNC_ERROR;
    }
    foo->control_func_run_time ();
} // calcHist

//!
//! \brief normalize
//! \param src
//! \param dst
//! \param alpha norm value to normalize to or the lower range boundary in case of the range normalization.
//! \param normType normalization type (see cv::NormTypes).
//!
CV_EXPORTS void normalize( const cv::SparseMat& src, cv::SparseMat& dst,
                           double alpha, int normType
                           BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::normalize (src, dst, alpha, normType);
        return;
    }
    static std::vector<opencvd_func *> func{};  // reg vector for erode
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), NORMALIZE_2, "normalize",
                               PARAMETER | FUNC_OFF,    // Menu
                               BUILIN_PARA);  // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
        func.push_back( foo );

        struct _double_para_ al = {alpha, -1000.0, 1000.0, 2};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha" );

        struct _enum_para_ ep = {normType, "NormTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "normType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );                     // dst = src
    } else {
        try {
            cv::normalize (src, dst,
                           *(double*)foo->para[0]->data,    // alpha
                           *(int*)foo->para[1]->data);      // normType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
}

//!
//! \brief normalize
//!        Normalizes the norm or value range of an array.
//! \param src
//! \param dst
//! \param alpha
//! \param beta
//! \param norm_type
//! \param dtype
//! \param mask
//!
CV_EXPORTS_W void normalize( cv::InputArray src, cv::InputOutputArray dst,
                             double alpha, double beta,
                             int norm_type, int dtype, cv::InputArray mask
                             BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::normalize (src, dst, alpha, beta, norm_type, dtype, mask);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for erode
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), NORMALIZE, "normalize",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE,    // Menu
                               BUILIN_PARA);  // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
        func.push_back( foo );

        struct _double_para_ al = {alpha, -1000.0, 1000.0, 2};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha" );

        struct _double_para_ be = {beta, -1000.0, 1000.0, 2};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&be, "beta" );

        struct _enum_para_ ep = {norm_type, "NormTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "norm_type" );

        struct _enum_para_ dd = {dtype, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "dtype" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );                     // dst = src
        // dst.setTo(cv::Scalar::all(0));      // set result to zero ?
    } else {
        try {
            cv::normalize (src, dst,
                        *(double*)foo->para[0]->data,
                        *(double*)foo->para[1]->data,
                        *(int*)foo->para[2]->data,
                        *(int*)foo->para[3]->data,
                        mask);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // normalize

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! @brief  getStructuringElement
//!         Returns a structuring element of the specified size and shape for morphological operations.
//!         Es wird die Masken Matrix für die Elemente MORPH_ELLIPSE, MORPH_CROSS oder MORPH_RECT erstellt.
//! @param  shape: See cv::MorphShapes
//! @param  ksize: Größe der Matrix w, h
//! @param  anchor:
//!         Ankerposition innerhalb des Elements. Der Standardwert \f$(-1, -1)\f$ bedeutet, dass sich der Anker in der Mitte befindet.
//!         Beachten Sie, dass nur die Form eines kreuzförmigen Elements vom Anker abhängt.
//!         Position.
//!         In anderen Fällen regelt der Anker nur, wie stark sich das Ergebnis der morphologischen Operation verschiebt.
//!
//! @see    https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
//!
CV_EXPORTS_W Mat getStructuringElement(int shape, cv::Size ksize, cv::Point anchor
                                           BUILDIN_FUNC)
{
    if (cvd_off) {
        Mat ret;
        ret = cv::getStructuringElement (shape, ksize, anchor);
        return ret;
    }

    Mat ret;
    static std::vector<opencvd_func *> func{};  // reg vector for erode
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GETSTRUCTURINGELEMENT, "getStructuringElement",
                               0x0001,          // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
                               BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ ep = {shape, "MorphShapes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "shape" );

        struct _point_int_ ip = {ksize.width, 1, 20000, ksize.height, 1, 20000};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "ksize");      // Matrix w, h

        struct _point_int_ ac = {anchor.x, -1, 20000, anchor.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ac, "anchor");     // Ankerpunkt default -1 / -1
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
    struct _point_int_ *ip = (struct _point_int_ *)foo->para[1]->data;
    struct _point_int_ *ac = (struct _point_int_ *)foo->para[2]->data;
    try {
        ret = cv::getStructuringElement ( *(int*)foo->para[0]->data,
                                          cv::Size(ip->x, ip->y),
                                          cv::Point(ac->x, ac->y) );
    } catch( cv::Exception& e ) {
        foo->error_flag |= FUNC_ERROR;
    }
    foo->control_func_run_time ();
    return ret;
} // getStructuringElement

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! @brief  Erodes an image by using a specific structuring element.
//! @param  kernel = Mat (anchor.x, anchor.y)
//! @param  default borderValue = __MAX_DBL__ = std::numeric_limits<double>::max()
//! @example  CVD::erode(src, dst, Mat(), Point(-1, -1), 2, 1, 1);
//!
CV_EXPORTS_W void erode( cv::InputArray src, cv::OutputArray dst, cv::InputArray kernel,
                         cv::Point anchor, int iterations,
                         int borderType,
                         const cv::Scalar& borderValue
                         BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::erode (src, dst, kernel, anchor, iterations, borderType, borderValue);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for erode
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), ERODE, "erode", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _point_int_ ip = {anchor.x, -1, 20000, anchor.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _int_para_ sp = {iterations, -10000, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "iterations");

        struct _enum_para_ ep = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "borderType" );

        // cv::Scalar& borderValue   not implemented
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            try {
                cv::erode( src, out, kernel,
                            cv::Point(ip->x, ip->y),
                            *(int*)foo->para[1]->data,
                            *(int*)foo->para[2]->data,
                            borderValue);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        try {
            cv::erode( src, dst, kernel,
                        cv::Point(ip->x, ip->y),
                        *(int*)foo->para[1]->data,
                        *(int*)foo->para[2]->data,
                        borderValue);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );    
} // erode

//! \brief dilate
//!        Dilates an image by using a specific structuring element.
//! \param src
//! \param dst
//! \param kernel
//!        kernel = Mat (anchor.x, anchor.y)
//! \param anchor
//! \param iterations
//! \param borderType
//! \param borderValue
//!        default borderValue = __MAX_DBL__ = std::numeric_limits<double>::max()
//! \example CVD::dilate(src, dst, Mat(), Point(-1, -1), 2, 1, 1);
//!
CV_EXPORTS_W void dilate( cv::InputArray src, cv::OutputArray dst, cv::InputArray kernel,
                               cv::Point anchor, int iterations,
                               int borderType,
                               const cv::Scalar& borderValue
                               BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::dilate (src, dst, kernel, anchor, iterations, borderType, borderValue);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for dilate
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), DILATE, "dilate", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _point_int_ ip = {anchor.x, -1, 20000, anchor.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _int_para_ sp = {iterations, -10000, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "iterations");

        struct _enum_para_ ep = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "borderType" );

        // cv::Scalar& borderValue   not implemented
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            try {
                cv::dilate( src, out, kernel,
                            cv::Point(ip->x, ip->y),
                            *(int*)foo->para[1]->data,
                            *(int*)foo->para[2]->data,
                            borderValue);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        try {
            cv::dilate( src, dst, kernel,
                        cv::Point(ip->x, ip->y),
                        *(int*)foo->para[1]->data,
                        *(int*)foo->para[2]->data,
                        borderValue);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // dilate

//! \brief morphologyEx Performs advanced morphological transformations.
//! \param src
//! \param dst
//! \param op
//! \param kernel
//! \param anchor
//! \param iterations
//! \param borderType
//! \param borderValue
//!
CV_EXPORTS_W void morphologyEx( cv::InputArray src, cv::OutputArray dst,
                                int op, cv::InputArray kernel,
                                cv::Point anchor, int iterations,
                                int borderType,
                                const cv::Scalar& borderValue
                                BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::morphologyEx( src, dst,
                          op,
                          kernel,
                          anchor,
                          iterations,
                          borderType,
                          borderValue );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for morphologyEx
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MORPHOLOGYEX, "morphologyEx", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ ep = {op, "MorphTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "op" );

        struct _point_int_ ip = {anchor.x, -1, 20000, anchor.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _int_para_ sp = {iterations, -10000, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "iterations");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );

        // cv::Scalar& borderValue   not implemented
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[1]->data;
            try {
                cv::morphologyEx( src, out,
                            *(int*)foo->para[0]->data,      // op
                            kernel,
                            cv::Point(ip->x, ip->y),        // anchor
                            *(int*)foo->para[2]->data,      // iterations
                            *(int*)foo->para[3]->data,      // borderType
                            borderValue);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[1]->data;
        try {
            cv::morphologyEx( src, dst,
                        *(int*)foo->para[0]->data,      // op
                        kernel,
                        cv::Point(ip->x, ip->y),        // anchor
                        *(int*)foo->para[2]->data,      // iterations
                        *(int*)foo->para[3]->data,      // borderType
                        borderValue);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // morphologyEx

//! \brief cvtColor Converts an image from one color space to another.
//! \param src
//! \param dst
//! \param code
//! \param dstCn
//! \example: CVD::cvtColor (src, dst, cv::COLOR_BGR2GRAY);
//!
CV_EXPORTS_W void cvtColor( cv::InputArray src, cv::OutputArray dst, int code, int dstCn
                            BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::cvtColor (src, dst, code, dstCn);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for cvtColor
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CVTCOLOR, "cvtColor", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ ep = {code, "ColorConversionCodes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "code" );

        struct _int_para_ sp = {dstCn, 0, 10};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "dstCn");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::cvtColor(src, out,
                             *(int*)foo->para[0]->data,
                             *(int*)foo->para[1]->data);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        try {
            cv::cvtColor(src, dst,
                         *(int*)foo->para[0]->data,
                         *(int*)foo->para[1]->data);
        }
        catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // cvtColor


//!
//! \brief adaptiveThreshold
//! \param src
//! \param dst
//! \param maxValue
//! \param adaptiveMethod
//! \param thresholdType
//! \param blockSize
//! \param C
//! \example CVD::adaptiveThreshold(roi, roi, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 3, 4.0);
//!
CV_EXPORTS_W void adaptiveThreshold( cv::InputArray src, cv::OutputArray dst,
                                     double maxValue, int adaptiveMethod,
                                     int thresholdType, int blockSize, double C
                                     BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::adaptiveThreshold (src, dst, maxValue, adaptiveMethod, thresholdType, blockSize, C);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for threshold
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), ADAPTIVETHRESHOLD, "adaptiveThreshold",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _slide_double_para_ mv = {maxValue, 0.0, 255.0, 1.0, maxValue};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&mv, "maxValue" );

        struct _enum_para_ aty = { adaptiveMethod, "AdaptiveThresholdTypes" };
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&aty, "adaptiveMethod" );

        struct _enum_para_ ep = {thresholdType, "ThresholdTypes_2"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "thresholdType" );

        struct _int_para_ bs = {blockSize, 3, 31};
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&bs, "blockSize" );

        struct _double_para_ cv = {C, -1000.0, 1000.0, 2};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&cv, "C" );

    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::adaptiveThreshold (src, out,
                                       *(double*)foo->para[0]->data,    // maxValue,
                                       *(int*)foo->para[1]->data,       // adaptiveMethod,
                                       *(int*)foo->para[2]->data,       // thresholdType,
                                       *(int*)foo->para[3]->data,       // blockSize,
                                       *(double*)foo->para[4]->data);   // C
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        try {
            cv::adaptiveThreshold (src, dst,
                                   *(double*)foo->para[0]->data,    // maxValue,
                                   *(int*)foo->para[1]->data,       // adaptiveMethod,
                                   *(int*)foo->para[2]->data,       // thresholdType,
                                   *(int*)foo->para[3]->data,       // blockSize,
                                   *(double*)foo->para[4]->data);   // C
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
}

//!
//! \brief threshold Applies a fixed-level threshold to each array element.
//! \param src input array (multiple-channel, 8-bit or 32-bit floating point).
//! \param dst output array of the same size  and type and the same number of channels as src.
//! \param thresh
//! \param maxval
//! \param type
//! \return
//! \example CVD::threshold( src, dst, 80, 230, THRESH_BINARY );
//!
CV_EXPORTS_W double threshold( cv::InputArray src, cv::OutputArray dst,
                               double thresh, double maxval, int type
                               BUILDIN_FUNC)
{
    if (cvd_off) {
        return cv::threshold (src, dst, thresh, maxval, type);
    }

    static std::vector<opencvd_func *> func{};  // reg vector for threshold
    opencvd_func *foo = nullptr;
    double ret = 0.0;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), THRESHOLD, "threshold",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        double min, max;
        cv::minMaxLoc (src, &min, &max);

        // ------------ init thresh ---------------------
        struct _slide_double_para_ dp;
        if ((src.depth() == CV_32F) && (min >= 0.0) && (max <= 1.0))
            dp = {thresh, 0.0, 1.0, 100.0, thresh};     // 32-bit floating point => 0.0 ... 1.0  divisor 100.0
        else
            dp = {thresh, 0.0, 255.0, 1.0, thresh};     // 8-bit

        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&dp, "thresh" );

        // ------------ init maxval ---------------------
        struct _slide_double_para_ dp2;
        if ((src.depth() == CV_32F) && (min >= 0.0) && (max <= 1.0))
            dp2 = {maxval, 0.0, 1.0, 100.0, maxval};    // 32-bit floating point => 0.0 ... 1.0  divisor 100.0
        else
            dp2 = {maxval, 0.0, 255.0, 1.0, maxval};    // 8-bit

        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&dp2, "maxval" );

        // ------------ init type ----------------------------
        struct _enum_para_ ep = {type, "ThresholdTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "type" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                ret = cv::threshold( src, out,
                                   *(double*)foo->para[0]->data,
                                   *(double*)foo->para[1]->data,
                                   *(int*)foo->para[2]->data);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // --------------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        try {
            ret = cv::threshold( src, dst,
                               *(double*)foo->para[0]->data,
                               *(double*)foo->para[1]->data,
                               *(int*)foo->para[2]->data);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );    
    return ret;
} // threshold

//!
//! \brief Sobel
//! \param src
//! \param dst
//! \param ddepth output image depth, see @ref filter_depths "combinations"; in the case of
//!               8-bit input images it will result in truncated derivatives.
//!        @link https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#filter_depths
//! \param dx  dx < ksize
//! \param dy  dy < ksize
//! \param ksize
//! \param scale
//! \param delta
//! \param borderType
//! \example    CVD::Sobel(src, dst, CV_8U, 1, 1); // CV_8UC1
//!             CVD::Sobel(top, sobel, CV_8U, 1, 0, 3, 0.4, 128);   // Sobel X
//!
CV_EXPORTS_W void Sobel( cv::InputArray src, cv::OutputArray dst, int ddepth,
                         int dx, int dy, int ksize,
                         double scale, double delta ,
                         int borderType
                         BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::Sobel ( src, dst, ddepth, dx, dy, ksize, scale, delta, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), SOBEL, "Sobel", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ dd = {ddepth, "Sobel_filterdepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "ddepth" );

        struct _int_para_ sx = {dx, 0, 30};                                             // dx < ksize
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sx, "dx" );

        struct _int_para_ sy = {dy, 0, 30};                                             // dy < ksize
        foo->new_para ( INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sy, "dy" );

        struct _int_para_ sp = {ksize, 1, 31};    // 1, 3, 5, 7, ...
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "ksize" );

        struct _double_para_ sc = {scale, -100000.0, 100000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sc, "scale");

        struct _double_para_ dl = {delta, -100000.0, 100000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dl, "delta");

        struct _enum_para_ bt = {borderType, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // ----------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::Sobel (src, out,
                           *(int*)foo->para[0]->data,       // ddepth,
                           *(int*)foo->para[1]->data,       // dx,
                           *(int*)foo->para[2]->data,       // dy,
                           *(int*)foo->para[3]->data,       // ksize,
                           *(double*)foo->para[4]->data,    // scale,
                           *(double*)foo->para[5]->data,    // delta,
                           *(int*)foo->para[6]->data);      // borderType);

            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------

    if (foo->state.flag.func_off) {         // Function OFF
        src.copyTo ( dst );
    } else {
        try {
            cv::Sobel (src, dst,
                       *(int*)foo->para[0]->data,       // ddepth,
                       *(int*)foo->para[1]->data,       // dx,
                       *(int*)foo->para[2]->data,       // dy,
                       *(int*)foo->para[3]->data,       // ksize,
                       *(double*)foo->para[4]->data,    // scale,
                       *(double*)foo->para[5]->data,    // delta,
                       *(int*)foo->para[6]->data);      // borderType);

        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // Sobel

//!
//! \brief Canny Finds edges in an image using the Canny algorithm @cite Canny86 .
//! \param image
//! \param edges
//! \param threshold1
//! \param threshold2
//! \param apertureSize
//! \param L2gradient
//! \example CVD::Canny( roi, roi, 47, 81, 3, false );
//!
CV_EXPORTS_W void Canny( cv::InputArray image, cv::OutputArray edges,
                         double threshold1, double threshold2,
                         int apertureSize, bool L2gradient
                         BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::Canny (image, edges, threshold1, threshold2, apertureSize, L2gradient);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CANNY, "Canny", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _slide_double_para_ dp = {threshold1, 0.0, 255.0, 1.0, threshold1};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&dp, "threshold1" );

        struct _slide_double_para_ dp2 = {threshold2, 0.0, 255.0, 1.0, threshold2};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&dp2, "threshold2" );

        struct _int_para_ sp = {apertureSize, 3, 7};    // 3, 5, 7
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "apertureSize" );

        struct _enum_para_ ep = {L2gradient, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "L2gradient" );

    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::Canny (image, out,
                           *(double*)foo->para[0]->data,
                           *(double*)foo->para[1]->data,
                           *(int*)foo->para[2]->data,
                           *(int*)foo->para[3]->data);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------
    if (foo->state.flag.func_off) {         // Function OFF
        image.copyTo(edges);                // 2019-11-20 NEW
        /**** 2019-11-20 commented out ****
        cv::Mat a = image.getMat();
        a.convertTo(edges, CV_8UC1);
        cv::Mat b = edges.getMat();
        b = cv::Scalar(0);
        */
    } else {
        try {
            cv::Canny (image, edges,
                       *(double*)foo->para[0]->data,
                       *(double*)foo->para[1]->data,
                       *(int*)foo->para[2]->data,
                       *(int*)foo->para[3]->data);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }        
    foo->control_imshow( edges );
} // Canny

//!
//! \brief Canny
//!        Finds edges in an image using the Canny algorithm @cite Canny86. Canny Typ 2
//! \param dx
//! \param dy
//! \param edges
//! \param threshold1
//! \param threshold2
//! \param L2gradient
//! \example    CVD::Canny( gray, edge, 50, 150, 3);
//!
CV_EXPORTS_W void Canny( cv::InputArray dx, cv::InputArray dy,                      // Canny Typ 2
                         cv::OutputArray edges,
                         double threshold1, double threshold2,
                         bool L2gradient
                         BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::Canny (dx, dy, edges, threshold1, threshold2, L2gradient);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CANNY_2, "Canny Typ 2", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _slide_double_para_ dp = {threshold1, 0.0, 255.0, 1.0, threshold1};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&dp, "threshold1" );

        struct _slide_double_para_ dp2 = {threshold2, 0.0, 255.0, 1.0, threshold2};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&dp2, "threshold2" );

        struct _enum_para_ ep = {L2gradient, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "L2gradient" );

    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::Canny (dx, dy, out,
                           *(double*)foo->para[0]->data,
                           *(double*)foo->para[1]->data,
                           *(int*)foo->para[2]->data);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {                 // Function OFF
        (cv::Mat&)edges = cv::Scalar(0);
    } else {
        try {
            cv::Canny (dx, dy, edges,
                       *(double*)foo->para[0]->data,
                       *(double*)foo->para[1]->data,
                       *(int*)foo->para[2]->data);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( edges );   
} // Canny

//!
//! \brief resize
//! \param src
//! \param dst
//! \param dsize output image size; if it equals zero, it is computed as:
//!        \f[\texttt{dsize = Size(round(fx*src.cols), round(fy*src.rows))}\f]
//!        Either dsize or both fx and fy must be non-zero.
//! \param fx scale factor along the horizontal axis; when it equals 0, it is computed as
//!        \f[\texttt{(double)dsize.width/src.cols}\f]
//! \param fy scale factor along the vertical axis; when it equals 0, it is computed as
//!        \f[\texttt{(double)dsize.height/src.rows}\f]
//! \param interpolation method, see cv::InterpolationFlags
//!
CV_EXPORTS_W void resize( cv::InputArray src, cv::OutputArray dst,
                          cv::Size dsize, double fx, double fy,
                          int interpolation
                          BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::resize (src, dst, dsize, fx, fy, interpolation);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), RESIZE, "resize", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _point_int_ ip = {dsize.width, 0, 0xFFFF, dsize.height, 0, 0xFFFF};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "dsize");

        struct _double_para_ sx = {fx, 0.0, 1000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sx, "fx");

        struct _double_para_ sy = {fy, 0.0, 1000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sy, "fy");

        struct _enum_para_ bt = {interpolation, "InterpolationFlags"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "interpolation");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    // -------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            try {
                cv::resize (src, out,
                            cv::Point(ip->x, ip->y),        // dsize
                            *(double*)foo->para[1]->data,   // fx
                            *(double*)foo->para[2]->data,   // fy
                            *(int *)foo->para[3]->data);    // interpolation
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -------------------------------

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        try {
            cv::resize (src, dst,
                        cv::Point(ip->x, ip->y),        // dsize
                        *(double*)foo->para[1]->data,   // fx
                        *(double*)foo->para[2]->data,   // fy
                        *(int *)foo->para[3]->data);    // interpolation
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );     // Bildausgabe
} // resize

//!
//! \brief medianBlur
//!        Blurs an image using the median filter.
//! \param src input 1-, 3-, or 4-channel image; when ksize is 3 or 5, the image depth should be
//!            CV_8U, CV_16U, or CV_32F, for larger aperture sizes, it can only be CV_8U.
//! \param dst destination array of the same size and type as src.
//! \param ksize aperture linear size; it must be odd and greater than 1, for example: 3, 5, 7 ...
//! \sa  bilateralFilter, blur, boxFilter, GaussianBlur
//! \example CVD::medianBlur(src, dst, 7);
//!
CV_EXPORTS_W void medianBlur( cv::InputArray src, cv::OutputArray dst, int ksize
                              BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::medianBlur (src, dst, ksize);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MEDIANBLUR, "medianBlur", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ sp = {ksize, 1, 31};
        foo->new_para (SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "ksize");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;            
            try {
                cv::medianBlur (src, out,
                                *(int*)foo->para[0]->data);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {        
        try {
            cv::medianBlur (src, dst,
                            *(int*)foo->para[0]->data);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // medianBlur

//!
//! \brief blur
//!        Blurs an image using the normalized box filter.
//! \param src
//! \param dst
//! \param ksize
//! \param anchor
//! \param borderType
//! \example CVD::blur(src, dst, cv::Size(3, 3));
//!
CV_EXPORTS_W void blur( cv::InputArray src, cv::OutputArray dst,
                        cv::Size ksize, cv::Point anchor,
                        int borderType
                        BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::blur (src, dst, ksize, anchor, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), BLUR_FUNC, "blur", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ sw = {ksize.width, 1, 31};
        foo->new_para (SLIDE_INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sw, "ksize width");

        struct _int_para_ sh = {ksize.height, 1, 31};
        foo->new_para (SLIDE_INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sh, "ksize height");

        struct _point_int_ ip = {anchor.x, -1, 20000, anchor.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[2]->data;
            try {
                cv::blur( src, out,
                          cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                          cv::Point(ip->x, ip->y),                                          // anchor
                          *(int*)foo->para[3]->data );                                      // borderType
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[2]->data;
        try {
            cv::blur( src, dst,
                      cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                      cv::Point(ip->x, ip->y),                                          // anchor
                      *(int*)foo->para[3]->data );                                      // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );    
} // blur

//!
//! \brief bilateralFilter
//! \param src Source 8-bit or floating-point, 1-channel or 3-channel image.
//! \param dst is the same size as src, but it ist an other.
//! \param d Diameter of each pixel neighborhood that is used during filtering. If it is non-positive,
//!          it is computed from sigmaSpace.
//! \param sigmaColor Filter sigma in the color space. A larger value of the parameter means that
//!                   farther colors within the pixel neighborhood (see sigmaSpace) will be mixed together, resulting
//!                   in larger areas of semi-equal color.
//! \param sigmaSpace Filter sigma in the coordinate space. A larger value of the parameter means that
//!                   farther pixels will influence each other as long as their colors are close enough (see sigmaColor
//!                   ). When d\>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, d is
//!                   proportional to sigmaSpace.
//! \param borderType border mode used to extrapolate pixels outside of the image, see cv::BorderTypes
//! \example CVD::Mat bf;
//!          CVD::bilateralFilter (src, bf, 15, 75, 75);
//!
CV_EXPORTS_W void bilateralFilter( cv::InputArray src, cv::OutputArray dst, int d,
                                   double sigmaColor, double sigmaSpace,
                                   int borderType
                                   BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::bilateralFilter (src, dst, d, sigmaColor, sigmaSpace, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), BILATERALFILTER, "bilateralFilter()",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ vd = {d, -100000, 100000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&vd, "d");

        struct _double_para_ sc = {sigmaColor, -1000.0, 1000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sc, "sigmaColor");

        struct _double_para_ ss = {sigmaSpace, -1000.0, 1000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ss, "sigmaSpace");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -------------------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::bilateralFilter( src, out,
                                  *(int*)foo->para[0]->data,        // d
                                  *(double*)foo->para[1]->data,     // sigmaColor
                                  *(double*)foo->para[2]->data,     // sigmaSpace
                                  *(int*)foo->para[3]->data );      // borderType
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            cv::bilateralFilter( src, dst,
                              *(int*)foo->para[0]->data,        // d
                              *(double*)foo->para[1]->data,     // sigmaColor
                              *(double*)foo->para[2]->data,     // sigmaSpace
                              *(int*)foo->para[3]->data );      // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );  // show Image
}

//!
//! \brief GaussianBlur
//!        Blurs an image using a Gaussian filter.
//! \param src
//! \param dst
//! \param ksize
//! \param sigmaX
//! \param sigmaY
//! \param borderType
//! \example CVD::GaussianBlur(src, dst, cv::Size(9, 9), 1.0);
//!
CV_EXPORTS_W void GaussianBlur( cv::InputArray src, cv::OutputArray dst, cv::Size ksize,
                                double sigmaX, double sigmaY,
                                int borderType
                                BUILDIN_FUNC)
{       
    if (cvd_off) {
        cv::GaussianBlur (src, dst, ksize, sigmaX, sigmaY, borderType);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GAUSSIANBLUR, "GaussianBlur", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ sw = {ksize.width, 1, 31};
        foo->new_para (SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sw, "ksize width");

        struct _int_para_ sh = {ksize.height, 1, 31};
        foo->new_para (SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sh, "ksize height");

        struct _double_para_ sx = {sigmaX, 0.0, 1000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sx, "sigmaX");

        struct _double_para_ sy = {sigmaY, 0.0, 1000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sy, "sigmaY");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::GaussianBlur( src, out,
                                  cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                                  *(double*)foo->para[2]->data,                                     // sigmaX
                                  *(double*)foo->para[3]->data,                                     // sigmaY
                                  *(int*)foo->para[4]->data );                                      // borderType
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            cv::GaussianBlur( src, dst,
                              cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                              *(double*)foo->para[2]->data,                                     // sigmaX
                              *(double*)foo->para[3]->data,                                     // sigmaY
                              *(int*)foo->para[4]->data );                                      // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );    
} // GaussianBlur

//!
//! \brief scaleAdd
//! \param src1
//! \param alpha scale factor for the first array (src1)
//! \param src2
//! \param dst
//!
CV_EXPORTS_W void scaleAdd(cv::InputArray src1, double alpha, cv::InputArray src2, cv::OutputArray dst
                           BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::scaleAdd( src1, alpha, src2, dst );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), SCALEADD, "scaleAdd",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu 0x000F
                               BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ al = {alpha, 0.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::scaleAdd( src1,
                              *(double*)foo->para[0]->data,      // alpha
                              src2, out);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src1.copyTo( dst );             // ???
    } else {
        try {
            cv::scaleAdd( src1,
                          *(double*)foo->para[0]->data,      // alpha
                          src2, dst);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // scaleAdd

//!
//! \brief addWeighted Calculates the weighted sum of two arrays.
//!                    dst = src1*alpha + src2*beta + gamma
//! \param src1
//! \param alpha
//! \param src2
//! \param beta
//! \param gamma
//! \param dst
//! \param dtype dtype optional depth of the output array; when both input arrays have the same depth, dtype
//!              can be set to -1, which will be equivalent to src1.depth().
//! \example CVD::addWeighted(src1, 0.5, src2, 0.5, 0.0, dst);
//!
CV_EXPORTS_W void addWeighted(cv::InputArray src1, double alpha, cv::InputArray src2,
                              double beta, double gamma, cv::OutputArray dst, int dtype
                              BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::addWeighted( src1, alpha, src2, beta, gamma, dst, dtype );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), ADDWEIGHTED, "addWeighted",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE | BREAK,    // Menu 0x000F,
                               BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ al = {alpha, -10000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha");

        struct _double_para_ be = {beta, -10000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&be, "beta");

        struct _double_para_ ga = {gamma, -10000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ga, "gamma");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::addWeighted( src1,
                                 *(double*)foo->para[0]->data,      // alpha
                                 src2,
                                 *(double*)foo->para[1]->data,      // beta
                                 *(double*)foo->para[2]->data,      // gamma
                                 out,
                                 dtype);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // ----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src1.copyTo( dst );
    } else {
        try {
            cv::addWeighted( src1,
                             *(double*)foo->para[0]->data,      // alpha
                             src2,
                             *(double*)foo->para[1]->data,      // beta
                             *(double*)foo->para[2]->data,      // gamma
                             dst,
                             dtype);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // addWeighted

//!
//! \brief cvd::convertScaleAbs
//!        Scales, calculates absolute values, and converts the result to 8-bit.
//! \param src
//! \param dst
//! \param alpha
//! \param beta
//!
CV_EXPORTS_W void convertScaleAbs(cv::InputArray src, cv::OutputArray dst,
                                  double alpha, double beta
                                  BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::convertScaleAbs( src, dst, alpha, beta );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CONVERTSCALEABS, "convertScaleAbs", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ al = {alpha, -10000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha");

        struct _double_para_ be = {beta, -10000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&be, "beta");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -----------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::convertScaleAbs( src, out,
                                     *(double*)foo->para[0]->data,      // alpha
                                     *(double*)foo->para[1]->data);     // beta
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -----------------------------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            cv::convertScaleAbs( src, dst,
                                 *(double*)foo->para[0]->data,      // alpha
                                 *(double*)foo->para[1]->data);     // beta
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // convertScaleAbs

//!
//! \brief findContours
//!        Finds contours in a binary image.
//! \param image
//! \param contours
//! \param hierarchy
//! \param mode
//! \param method
//! \param offset
//!
CV_EXPORTS_W void findContours( cv::InputOutputArray image,         // CVD::Mat
                                cv::OutputArrayOfArrays contours,   // vector< vector<Point> >
                                cv::OutputArray hierarchy,          // vector< Vec4i >
                                int mode,
                                int method, cv::Point offset
                                BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::findContours( image, contours, hierarchy, mode, method, offset );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), FINDCONTOURS, "findContours", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ bt = {mode, "RetrievalModes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "mode");

        struct _enum_para_ bt2 = {method, "ContourApproximationModes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt2, "method");

        struct _point_int_ ip = {offset.x, -20000, 20000, offset.y, -20000, 20000};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "offset");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden

        vector<vector<cv::Point> > break_contours;
        vector<cv::Vec4i> break_hierarchy;

        while (foo->state.flag.func_break) {
            struct _point_int_ *ac = (struct _point_int_ *)foo->para[2]->data;  // offset
            try {
                cv::findContours( image, break_contours, break_hierarchy,
                                  *(int*)foo->para[0]->data,      // mode,
                                  *(int*)foo->para[1]->data,      // method,
                                  cv::Point(ac->x, ac->y) );
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
              foo->control_contours_imshow ( image, break_contours, break_hierarchy );    // Contours Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        contours.clear();
        hierarchy.clear();
        // src.copyTo( dst );
    } else {
        struct _point_int_ *ac = (struct _point_int_ *)foo->para[2]->data;  // offset
        try {
            cv::findContours( image, contours, hierarchy,
                              *(int*)foo->para[0]->data,      // mode,
                              *(int*)foo->para[1]->data,      // method,
                              cv::Point(ac->x, ac->y) );
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }

    foo->control_contours_imshow ( image, contours, hierarchy );    // Contours Ausgabe
} // findContours

//!
//! @overload
//!
CV_EXPORTS void findContours( cv::InputOutputArray image, cv::OutputArrayOfArrays contours,
                              int mode, int method, cv::Point offset
                              BUILDIN_FUNC)
{
    vector<cv::Vec4i> hierarchy;

    cvd::findContours (image, contours, hierarchy, mode, method, offset, line_nr, src_file);
}

//!
//! \brief approxPolyDP
//! \param curve
//! \param approxCurve
//! \param epsilon Parameter specifying the approximation accuracy. This is the maximum distance
//!         between the original curve and its approximation.
//! \param closed If true, the approximated curve is closed (its first and last vertices are
//!         connected). Otherwise, it is not closed.
//!
CV_EXPORTS_W void approxPolyDP( cv::InputArray curve,           // vector < cv::Point >
                                cv::OutputArray approxCurve,    // vector< cv::Point >
                                double epsilon, bool closed
                                BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::approxPolyDP( curve, approxCurve, epsilon, closed );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for pyrUp
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), APPROXPOLYPD, "approxPolyDP",
                               PARAMETER | FUNC_OFF,    // Menu
                               BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ ep = {epsilon, 0.0, 100000.0, 2};                                      // epsilon
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ep, "epsilon" );

        struct _enum_para_ bt = {closed, "boolType"};                                               // closed
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "closed" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    // NO break
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        // do nothing
    } else {
        try {
            cv::approxPolyDP( curve, approxCurve,
                            *(double*)foo->para[0]->data,       // epsilon
                            *(int*)foo->para[1]->data);          // closed
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    // foo->control_approxPolyDP_imshow( approxCurve );     // No show
}

//!
//! \brief Laplacian
//!        Calculates the Laplacian of an image.
//! \param src
//! \param dst
//! \param ddepth
//! \param ksize
//! \param scale
//! \param delta
//! \param borderType
//! \example    CVD::Laplacian(foo, dummy, CV_16S, 3, 1, 0);
//!             CVD::convertScaleAbs( dummy, foo );           // converting back to CV_8U
//!
CV_EXPORTS_W void Laplacian( cv::InputArray src, cv::OutputArray dst, int ddepth,
                             int ksize, double scale, double delta,
                             int borderType
                             BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::Laplacian( src, dst, ddepth, ksize, scale, delta, borderType );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), LAPLACIAN, "Laplacian", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ ep = {ddepth, "ddepth"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "ddepth");

        struct _int_para_ sp = {ksize, 1, 31};
        foo->new_para (SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "ksize");

        struct _double_para_ dv = {scale, 0.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dv, "scale");

        struct _double_para_ dd = {delta, 0.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dd, "delta");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::Laplacian( src, out,
                                  *(int*)foo->para[0]->data,        // ddepth
                                  *(int*)foo->para[1]->data,        // ksize
                                  *(double*)foo->para[2]->data,     // scale
                                  *(double*)foo->para[3]->data,     // delta
                                  *(int*)foo->para[4]->data);       // borderType
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            cv::Laplacian( src, dst,
                              *(int*)foo->para[0]->data,        // ddepth
                              *(int*)foo->para[1]->data,        // ksize
                              *(double*)foo->para[2]->data,     // scale
                              *(double*)foo->para[3]->data,     // delta
                              *(int*)foo->para[4]->data);       // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
}

//!
//! \brief imread
//!        Loads an image from a file.
//! \param filename
//! \param flags
//! \return
//! \example src = imread ( "images/board.jpg", cv::IMREAD_UNCHANGED );
//!
CV_EXPORTS_W Mat imread( const cv::String& filename, int flags
                             BUILDIN_FUNC)
{
    Mat ret;

    if (cvd_off) {
        ret = cv::imread( filename, flags );
        return ret;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), IMREAD, "imread", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        assert (filename.length() < MAX_STRING_VAL_LEN-1);
        struct _string_para_ al;
        strcpy (al.val, filename.c_str());
        foo->new_para (STRING_PARA, sizeof(struct _string_para_), (uint8_t*)&al, "filename");

        struct _enum_para_ ep = {flags, "ImreadModes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "flags");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    // ------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat buf_image;
            assert (strlen((char *)foo->para[0]->data) < MAX_STRING_VAL_LEN-1);
            struct _string_para_ *al = (struct _string_para_ *)foo->para[0]->data;
            try {
                buf_image = cv::imread( al->val,
                                  *(int*)foo->para[1]->data );
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }

            if (buf_image.empty()) {                                      // Es ist kein Bild geladen worden !
                cv::Mat out(200, 200, CV_8UC1, cv::Scalar(128));    // fiktive cv::Mat erzeugen !
                foo->control_imshow( out );                         // fiktive cv::Mat ausgeben.
            } else
                foo->control_imshow( buf_image );

            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // ------------------------------------------------------------
    if (foo->state.flag.func_off) {     // imread ist ausgeschaltet.
        return ret;                     // return ist empty !!!
    } else {
        try {
            assert (strlen((char *)foo->para[0]->data) < MAX_STRING_VAL_LEN-1);
            struct _string_para_ *al = (struct _string_para_ *)foo->para[0]->data;
            ret = cv::imread( al->val,
                              *(int*)foo->para[1]->data );
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }

    if (ret.empty()) {                                      // Es ist kein Bild geladen worden !
        cv::Mat out(200, 200, CV_8UC1, cv::Scalar(128));    // fiktive cv::Mat erzeugen !
        foo->control_imshow( out );                         // fiktive cv::Mat ausgeben.
    } else
        foo->control_imshow( ret );

    return ret;
} // imread

//!
//! \brief HoughCircles
//! \param image
//!        8-bit, single-channel, grayscale
//! \param circles
//! \param method
//!        Erkennungsmethode, siehe cv::HoughModes. Derzeit ist die einzige implementierte Methode HOUGH_GRADIENT.
//! \param dp
//! \param minDist
//!        minDist Mindestabstand zwischen den Mittelpunkten der erfassten Kreise.
//!        Wenn der Parameter zu klein ist, können neben einem echten auch mehrere Nachbarkreise falsch erkannt werden.
//!        Wenn er zu groß ist, können einige Kreise übersehen werden.
//! \param param1
//!        Erster methodenspezifischer Parameter. Im Falle von CV_HOUGH_GRADIENT ist es der höhere Schwellenwert der beiden,
//!        die an den Canny-Kantendetektor übergeben werden (der niedrigere ist zweimal kleiner).
//! \param param2
//!        Zweiter methodenspezifischer Parameter. Im Falle von CV_HOUGH_GRADIENT ist es die Akkumulatorschwelle für die Kreismittelpunkte
//!        in der Erkennungsphase. Je kleiner sie ist, desto mehr falsche Kreise können erkannt werden. Kreise,
//!        die den größeren Akkumulatorwerten entsprechen, werden als kehrte zuerst zurück.
//! \param minRadius
//! \param maxRadius
//!
CV_EXPORTS_W void HoughCircles( cv::InputArray image, cv::OutputArray circles,
                               int method, double dp, double minDist,
                               double param1, double param2,
                               int minRadius, int maxRadius
                               BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::HoughCircles( image, circles, method, dp, minDist, param1, param2, minRadius, maxRadius);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), HOUGHCIRCLES, "HoughCircles", 0x0003, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ bt = {method, "HoughModes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "method");

        struct _double_para_ al = {dp, 0.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "dp");

        struct _double_para_ md = {minDist, 0.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&md, "minDist");

        struct _slide_double_para_ p1 = {param1, 1.0, 255.0, 1.0, param1};            // threshold
        foo->new_para (SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&p1, "param1");

        struct _slide_double_para_ p2 = {param2, 1.0, 255.0, 1.0, param2};
        foo->new_para (SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&p2, "param2");

        struct _int_para_ minr = {minRadius, 0, 200000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&minr, "minRadius");

        struct _int_para_ maxr = {maxRadius, 0, 200000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&maxr, "maxRadius");

    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_off) {
        circles.clear();    // do nothing
    } else {
        try {
            cv::HoughCircles( image, circles,
                              *(int*)foo->para[0]->data,         // method,
                              *(double*)foo->para[1]->data,      // dp,
                              *(double*)foo->para[2]->data,      // minDist,
                              *(double*)foo->para[3]->data,      // param1
                              *(double*)foo->para[4]->data,      // param2
                              *(int*)foo->para[5]->data,         // minRadius,
                              *(int*)foo->para[6]->data);        // maxRadius);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    // foo->control_imshow( image );
} // HoughCircles

//!
//! \brief HoughLinesP
//! \param image: 8-bit, single-channel binary source image. The image may be modified by the function.
//! \param lines
//! \param rho: Distance resolution of the accumulator in pixels.
//! \param theta: Angle resolution of the accumulator in radians.
//! \param threshold:
//! \param minLineLength: Minimum line length. Line segments shorter than that are rejected.
//! \param maxLineGap: Maximum allowed gap between points on the same line to link them.
//!
CV_EXPORTS_W void HoughLinesP( cv::InputArray image, cv::OutputArray lines,
                               double rho, double theta, int threshold,
                               double minLineLength, double maxLineGap
                               BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::HoughLinesP( image, lines, rho, theta, threshold, minLineLength, maxLineGap);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), HOUGHLINESP, "HoughLinesP", 0x0003, BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ ro = {rho, -100000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ro, "rho");

        struct _double_para_ ta = {theta, -100000.0, std::numeric_limits<double>::max(), 4};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ta, "theta");

        struct _int_para_ sw = {threshold, 0, 255};
        foo->new_para (SLIDE_INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sw, "threshold");

        struct _double_para_ mll = {minLineLength, -100000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&mll, "minLineLength");

        struct _double_para_ mlg = {maxLineGap, -100000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&mlg, "maxLineGap");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_off) {
        lines.clear();    // do nothing
    } else {
        try {
            cv::HoughLinesP( image, lines,
                             *(double*)foo->para[0]->data,      // rho
                             *(double*)foo->para[1]->data,      // theta
                             *(int*)foo->para[2]->data,         // threshold
                             *(double*)foo->para[3]->data,      // minLineLength
                             *(double*)foo->para[4]->data);     // maxLineGap
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
} // HoughLinesP

//!
//! \brief HoughLines
//! \param image 8-bit, single-channel binary source image. The image may be modified by the function.
//! \param lines
//! \param rho Distance resolution of the accumulator in pixels.
//! \param theta Angle resolution of the accumulator in radians.
//! \param threshold
//! \param srn For the multi-scale Hough transform, it is a divisor for the distance resolution rho .
//! The coarse accumulator distance resolution is rho and the accurate accumulator resolution is
//! rho/srn . If both srn=0 and stn=0 , the classical Hough transform is used. Otherwise, both these
//! parameters should be positive.
//! \param stn For the multi-scale Hough transform, it is a divisor for the distance resolution theta.
//! \param min_theta For standard and multi-scale Hough transform, minimum angle to check for lines. Must fall between 0 and max_theta.
//! \param max_theta For standard and multi-scale Hough transform, maximum angle to check for lines. Must fall between min_theta and CV_PI.
//! \example    vector<Vec2f> lines;
//!             CVD::HoughLines( src, lines, 1, 0.0174533, 32, 0, 0, 0, 3.14159 );
//!
CV_EXPORTS_W void HoughLines( cv::InputArray image, cv::OutputArray lines,
                              double rho, double theta, int threshold,
                              double srn, double stn,
                              double min_theta, double max_theta
                              BUILDIN_FUNC )
{
    if (cvd_off) {
        cv::HoughLines( image, lines, rho, theta, threshold, srn, stn, min_theta, max_theta);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), HOUGHLINES, "HoughLines", 0x0003, BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ ro = {rho, -100000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ro, "rho");

        struct _double_para_ ta = {theta, -100000.0, std::numeric_limits<double>::max(), 4};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ta, "theta");

        struct _int_para_ sw = {threshold, 0, 255};
        foo->new_para (SLIDE_INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sw, "threshold");

        struct _double_para_ sr = {srn, 0.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sr, "srn");

        struct _double_para_ st = {stn, 0.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&st, "stn");

        struct _slide_double_para_ mit = {min_theta, 0.0, CV_PI, 100.0, min_theta};
        foo->new_para (SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&mit, "min_theta");

        struct _slide_double_para_ mat = {max_theta, 0.0, CV_PI, 100.0, max_theta};
        foo->new_para (SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&mat, "max_theta");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_off) {
        lines.clear();    // do nothing
    } else {
        try {
            cv::HoughLines( image, lines,
                            *(double*)foo->para[0]->data,      // rho
                            *(double*)foo->para[1]->data,      // theta
                            *(int*)foo->para[2]->data,         // threshold
                            *(double*)foo->para[3]->data,      // srn
                            *(double*)foo->para[4]->data,      // stn
                            *(double*)foo->para[5]->data,      // min_theta
                            *(double*)foo->para[6]->data);     // max_theta
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
} // HoughLines

//!
//! \brief Scharr input image.
//! \param src dst output image of the same size and the same number of channels as src.
//! \param dst output image of the same size and the same number of channels as src.
//! \param ddept houtput image depth, see @ref filter_depths "combinations"
//! \param dx order of the derivative x.
//! \param dy order of the derivative y.
//! \param scale optional scale factor for the computed derivative values; by default, no scaling is
//!        applied (see getDerivKernels for details).
//! \param delta optional delta value that is added to the results prior to storing them in dst.
//! \param borderType pixel extrapolation method, see cv::BorderTypes
//! \example    CVD::Scharr(foo[i], dummy[0], CV_8UC1, 1, 0, 1); // CV_8UC1
//!             CVD::Scharr(foo[i], dummy[1], CV_8UC1, 1, 0, -1); // CV_8UC1
//!
CV_EXPORTS_W void Scharr( cv::InputArray src, cv::OutputArray dst, int ddepth,
                          int dx, int dy, double scale, double delta,
                          int borderType
                          BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::Scharr( src, dst, ddepth, dx, dy, scale, delta, borderType );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = nullptr;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == nullptr) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), SCHARR, "Scharr", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ ep = {ddepth, "ddepth"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "ddepth");

        struct _int_para_ sw = {dx, -10000, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sw, "dx");

        struct _int_para_ sh = {dy, -10000, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sh, "sh");

        struct _double_para_ sc = {scale, -100000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sc, "scale");

        struct _double_para_ dt = {delta, -100000.0, std::numeric_limits<double>::max(), 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dt, "delta");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType");

    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // ---------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::Scharr( src, out,
                            *(int*)foo->para[0]->data,          // ddepth,
                            *(int*)foo->para[1]->data,          // dx,
                            *(int*)foo->para[2]->data,          // dy,
                            *(double*)foo->para[3]->data,       // scale,
                            *(double*)foo->para[4]->data,       // delta,
                            *(int*)foo->para[5]->data);         // borderType
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // ---------------------------------------------------------
    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        try {
            cv::Scharr( src, dst,
                        *(int*)foo->para[0]->data,          // ddepth,
                        *(int*)foo->para[1]->data,          // dx,
                        *(int*)foo->para[2]->data,          // dy,
                        *(double*)foo->para[3]->data,       // scale,
                        *(double*)foo->para[4]->data,       // delta,
                        *(int*)foo->para[5]->data);         // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
} // Scharr


} // namespace cvd

#endif // OPENCVD_FUNC_HPP
