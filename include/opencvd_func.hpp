#ifndef OPENCVD_FUNC_HPP
#define OPENCVD_FUNC_HPP

#include "opencv2/opencv.hpp"


namespace cvd {

#define USE_BUILDIN

//!
//! @brief  Macro included Line-Number.
//! @see    https://gcc.gnu.org/onlinedocs/gcc/Other-Builtins.html
//!
#ifdef USE_BUILDIN
    #define BUILDIN  , int line_nr = __builtin_LINE(), \
                     const char *src_file = __builtin_FILE()

    #define BUILDIN_FUNC  , int line_nr, \
                          const char *src_file
#else
    #define BUILDIN  , int line_nr = 0 \
                     const char *src_file = NULL
    #define BUILDIN_FUNC  , int line_nr \
                          const char *src_file
#endif

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

CV_EXPORTS_W void medianBlur( cv::InputArray src, cv::OutputArray dst, int ksize
                              BUILDIN);

CV_EXPORTS_W void blur( cv::InputArray src, cv::OutputArray dst,
                        cv::Size ksize, cv::Point anchor = cv::Point(-1,-1),
                        int borderType = cv::BORDER_DEFAULT
                        BUILDIN);

CV_EXPORTS_W void GaussianBlur( cv::InputArray src, cv::OutputArray dst, cv::Size ksize,
                                double sigmaX, double sigmaY = 0,
                                int borderType = cv::BORDER_DEFAULT
                                BUILDIN);

CV_EXPORTS_W void Canny( cv::InputArray image, cv::OutputArray edges,
                         double threshold1, double threshold2,
                         int apertureSize = 3, bool L2gradient = false
                         BUILDIN);

CV_EXPORTS_W void Canny( cv::InputArray dx, cv::InputArray dy,          // Canny Typ 2
                         cv::OutputArray edges,
                         double threshold1, double threshold2,
                         bool L2gradient = false
                         BUILDIN);

CV_EXPORTS_W double threshold( cv::InputArray src, cv::OutputArray dst,
                               double thresh, double maxval, int type
                               BUILDIN);

CV_EXPORTS_W void cvtColor( cv::InputArray src, cv::OutputArray dst, int code, int dstCn=0
                            BUILDIN);

CV_EXPORTS_W void dilate( cv::InputArray src, cv::OutputArray dst, cv::InputArray kernel,
                           cv::Point anchor = cv::Point(-1,-1), int iterations = 1,
                           int borderType = cv::BORDER_CONSTANT,
                           const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue()    // morphologyDefaultBorderValue() = Scalar::all(DBL_MAX)
                           BUILDIN);

CV_EXPORTS_W void erode( cv::InputArray src, cv::OutputArray dst, cv::InputArray kernel,
                         cv::Point anchor = cv::Point(-1,-1), int iterations = 1,
                         int borderType = cv::BORDER_CONSTANT,
                         const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue()      // morphologyDefaultBorderValue() = Scalar::all(DBL_MAX)
                         BUILDIN);

CV_EXPORTS_W void morphologyEx( cv::InputArray src, cv::OutputArray dst,
                                int op, cv::InputArray kernel,
                                cv::Point anchor = cv::Point(-1,-1), int iterations = 1,
                                int borderType = cv::BORDER_CONSTANT,
                                const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue()
                                BUILDIN);

CV_EXPORTS_W cv::Mat getStructuringElement(int shape, cv::Size ksize, cv::Point anchor = cv::Point(-1,-1)
                                           BUILDIN);

CV_EXPORTS_W void convertScaleAbs(cv::InputArray src, cv::OutputArray dst,
                                  double alpha = 1, double beta = 0
                                  BUILDIN);

CV_EXPORTS_W void findContours( cv::InputOutputArray image,
                                cv::OutputArrayOfArrays contours,
                                cv::OutputArray hierarchy,
                                int mode,
                                int method, cv::Point offset = cv::Point()
                                BUILDIN);

CV_EXPORTS void findContours( cv::InputOutputArray image, cv::OutputArrayOfArrays contours,     // overload
                              int mode, int method, cv::Point offset = cv::Point()
                              BUILDIN);

CV_EXPORTS_W void Laplacian( cv::InputArray src, cv::OutputArray dst, int ddepth,
                             int ksize = 1, double scale = 1, double delta = 0,
                             int borderType = cv::BORDER_DEFAULT
                             BUILDIN);

CV_EXPORTS_W cv::Mat imread( const cv::String& filename, int flags = cv::IMREAD_COLOR
                             BUILDIN);

CV_EXPORTS_W void normalize( cv::InputArray src, cv::InputOutputArray dst, double alpha = 1, double beta = 0,
                             int norm_type = cv::NORM_L2, int dtype = -1, cv::InputArray mask = cv::noArray()
                             BUILDIN);

CV_EXPORTS void calcHist( const cv::Mat* images, int nimages,
                          const int* channels, cv::InputArray mask,
                          cv::OutputArray hist, int dims, const int* histSize,
                          const float** ranges, bool uniform = true, bool accumulate = false
                          BUILDIN);

CV_EXPORTS_W void HoughCircles( cv::InputArray image, cv::OutputArray circles,
                               int method, double dp, double minDist,
                               double param1 = 100, double param2 = 100,
                               int minRadius = 0, int maxRadius = 0
                               BUILDIN);

CV_EXPORTS_W void HoughLinesP( cv::InputArray image, cv::OutputArray lines,
                               double rho, double theta, int threshold,
                               double minLineLength = 0, double maxLineGap = 0
                               BUILDIN);

/*
class CV_EXPORTS Mat : public cv::Mat
{
public:
    Mat();
    void convertTo( cv::OutputArray m, int rtype, double alpha=1, double beta=0 ) const;
};

cvd::Mat::Mat() : cv::Mat() {}

void cvd::Mat::convertTo( cv::OutputArray m, int rtype, double alpha, double beta ) const
{
    printf ("bin drin\n");
    cv::Mat::convertTo(m, rtype, alpha, beta);
}
*/

//!
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

    static std::vector<opencvd_func *> func{};  // reg vector for erode
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CALCHIST, "calcHist", 0x0001, BUILIN_PARA);  // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
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
    foo->error_flag = 0;

    try {
        cv::calcHist (images,
                      *(int*)foo->para[0]->data,    // nimages
                      channels, mask, hist,
                      *(int*)foo->para[1]->data,    // dims
                      histSize, ranges,
                      *(int*)foo->para[2]->data,    // uniform
                      *(int*)foo->para[3]->data);   // accumulate
    } catch( cv::Exception& e ) {
        foo->error_flag = 1;
    }
    foo->control_func_run_time ();
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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), NORMALIZE, "normalize", 0x0003, BUILIN_PARA);  // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
        func.push_back( foo );

        struct _double_para_ al = {alpha, -1000.0, 1000.0};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha" );

        struct _double_para_ be = {beta, -1000.0, 1000.0};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&be, "beta" );

        struct _enum_para_ ep = {norm_type, "NormTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "norm_type" );

        struct _enum_para_ dd = {dtype, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "dtype" );
    }
    foo->error_flag = 0;

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        try {
            cv::normalize (src, dst,
                        *(double*)foo->para[0]->data,
                        *(double*)foo->para[1]->data,
                        *(int*)foo->para[2]->data,
                        *(int*)foo->para[3]->data,
                        mask);
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    // foo->control_imshow( dst );
}

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
CV_EXPORTS_W cv::Mat getStructuringElement(int shape, cv::Size ksize, cv::Point anchor
                                           BUILDIN_FUNC)
{
    if (cvd_off) {
        return cv::getStructuringElement (shape, ksize, anchor);
    }

    cv::Mat ret;
    static std::vector<opencvd_func *> func{};  // reg vector for erode
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GETSTRUCTURINGELEMENT, "getStructuringElement", 0x0001, BUILIN_PARA);  // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
        func.push_back( foo );

        struct _enum_para_ ep = {shape, "MorphShapes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "shape" );

        struct _point_int_ ip = {ksize.width, 1, 20000, ksize.height, 1, 20000};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "ksize");      // Matrix w, h

        struct _point_int_ ac = {anchor.x, -1, 20000, anchor.y, -1, 20000};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ac, "anchor");     // Ankerpunkt default -1 / -1
    }
    foo->error_flag = 0;

    // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
    struct _point_int_ *ip = (struct _point_int_ *)foo->para[1]->data;
    struct _point_int_ *ac = (struct _point_int_ *)foo->para[2]->data;
    try {
        ret = cv::getStructuringElement ( *(int*)foo->para[0]->data,
                                          cv::Size(ip->x, ip->y),
                                          cv::Point(ac->x, ac->y) );
    } catch( cv::Exception& e ) {
        foo->error_flag = 1;
    }
    foo->control_func_run_time ();
    return ret;
}

//! @brief  Erodes an image by using a specific structuring element.
//! @param  kernel = Mat (anchor.x, anchor.y)
//! @param  default borderValue = __MAX_DBL__ = std::numeric_limits<double>::max()
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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
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
    foo->error_flag = 0;

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
                cv::erode( src, out, kernel,
                            cv::Point(ip->x, ip->y),
                            *(int*)foo->para[1]->data,
                            *(int*)foo->para[2]->data,
                            borderValue);
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
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
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            cv::erode( src, dst, kernel,
                        cv::Point(ip->x, ip->y),
                        *(int*)foo->para[1]->data,
                        *(int*)foo->para[2]->data,
                        borderValue);
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );    
}

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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
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
    foo->error_flag = 0;

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
                cv::dilate( src, out, kernel,
                            cv::Point(ip->x, ip->y),
                            *(int*)foo->para[1]->data,
                            *(int*)foo->para[2]->data,
                            borderValue);
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
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
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            cv::dilate( src, dst, kernel,
                        cv::Point(ip->x, ip->y),
                        *(int*)foo->para[1]->data,
                        *(int*)foo->para[2]->data,
                        borderValue);
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
}

//! \brief morphologyEx
//!        Performs advanced morphological transformations.
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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
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
    foo->error_flag = 0;

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                struct _point_int_ *ip = (struct _point_int_ *)foo->para[1]->data;
                cv::morphologyEx( src, out,
                            *(int*)foo->para[0]->data,      // op
                            kernel,
                            cv::Point(ip->x, ip->y),        // anchor
                            *(int*)foo->para[2]->data,      // iterations
                            *(int*)foo->para[3]->data,      // borderType
                            borderValue);
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
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
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[1]->data;
            cv::morphologyEx( src, dst,
                        *(int*)foo->para[0]->data,      // op
                        kernel,
                        cv::Point(ip->x, ip->y),        // anchor
                        *(int*)foo->para[2]->data,      // iterations
                        *(int*)foo->para[3]->data,      // borderType
                        borderValue);
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
}

//! \brief cvtColor
//!        Converts an image from one color space to another.
//! \param src
//! \param dst
//! \param code
//! \param dstCn
//!
CV_EXPORTS_W void cvtColor( cv::InputArray src, cv::OutputArray dst, int code, int dstCn
                            BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::cvtColor (src, dst, code, dstCn);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector for cvtColor
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CVTCOLOR, "cvtColor", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ ep = {code, "ColorConversionCodes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "code" );

        struct _int_para_ sp = {dstCn, 0, 10};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "dstCn");
    }
    foo->error_flag = 0;

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::cvtColor(src, out,
                             *(int*)foo->para[0]->data,
                             *(int*)foo->para[1]->data);
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
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
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
}
//! -------------------------------------------------------------------------
//! @brief Applies a fixed-level threshold to each array element.
//! -------------------------------------------------------------------------
CV_EXPORTS_W double threshold( cv::InputArray src, cv::OutputArray dst,
                               double thresh, double maxval, int type
                               BUILDIN_FUNC)
{
    if (cvd_off) {
        return cv::threshold (src, dst, thresh, maxval, type);
    }

    static std::vector<opencvd_func *> func{};  // reg vector for threshold
    opencvd_func *foo = NULL;
    double ret = 0.0;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), THRESHOLD, "threshold", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ dp = {thresh, 0.0, 255.0};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dp, "thresh" );

        struct _double_para_ dp2 = {maxval, 0.0, 255};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dp2, "maxval" );

        struct _enum_para_ ep = {type, "ThresholdTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "type" );
    }
    foo->error_flag = 0;

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
                foo->error_flag = 1;
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
            ret = cv::threshold( src, dst,
                               *(double*)foo->para[0]->data,
                               *(double*)foo->para[1]->data,
                               *(int*)foo->para[2]->data);
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );    
    return ret;
}
//!
//! \brief Canny
//!        Finds edges in an image using the Canny algorithm @cite Canny86 .
//! \param image
//! \param edges
//! \param threshold1
//! \param threshold2
//! \param apertureSize
//! \param L2gradient
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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CANNY, "Canny", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ dp = {threshold1, 0.0, 255.0};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dp, "threshold1" );

        struct _double_para_ dp2 = {threshold2, 0.0, 255.0};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dp2, "threshold2" );

        struct _int_para_ sp = {apertureSize, 3, 7};    // 3, 5, 7
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "apertureSize" );


        struct _enum_para_ ep = {L2gradient, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "L2gradient" );

    }
    foo->error_flag = 0;

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
                foo->error_flag = 1;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {         // Function OFF
        cv::Mat a = image.getMat();
        a.convertTo(edges, CV_8UC1);
        cv::Mat b = edges.getMat();
        b = cv::Scalar(0);
    } else {
        try {
            cv::Canny (image, edges,
                       *(double*)foo->para[0]->data,
                       *(double*)foo->para[1]->data,
                       *(int*)foo->para[2]->data,
                       *(int*)foo->para[3]->data);
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }        
    foo->control_imshow( edges );
}
//!
//! \brief Canny
//!        Finds edges in an image using the Canny algorithm @cite Canny86. Canny Typ 2
//! \param dx
//! \param dy
//! \param edges
//! \param threshold1
//! \param threshold2
//! \param L2gradient
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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CANNY_2, "Canny Typ 2", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ dp = {threshold1, 0.0, 255.0};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dp, "threshold1" );

        struct _double_para_ dp2 = {threshold2, 0.0, 255.0};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dp2, "threshold2" );

        struct _enum_para_ ep = {L2gradient, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "L2gradient" );

    }
    foo->error_flag = 0;

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
                foo->error_flag = 1;
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
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( edges );   
}
//!
//! \brief medianBlur
//!        Blurs an image using the median filter.
//! \param src
//! \param dst
//! \param ksize
//!
CV_EXPORTS_W void medianBlur( cv::InputArray src, cv::OutputArray dst, int ksize
                              BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::medianBlur (src, dst, ksize);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MEDIANBLUR, "medianBlur", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ sp = {ksize, 1, 31};
        foo->new_para (SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "ksize");
    }
    foo->error_flag = 0;

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;            
            try {
                cv::medianBlur (src, out,
                                *(int*)foo->para[0]->data);
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
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
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
}


//!
//! \brief blur
//!        Blurs an image using the normalized box filter.
//! \param src
//! \param dst
//! \param ksize
//! \param anchor
//! \param borderType
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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
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
    foo->error_flag = 0;

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                struct _point_int_ *ip = (struct _point_int_ *)foo->para[2]->data;
                cv::blur( src, out,
                          cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                          cv::Point(ip->x, ip->y),                                          // anchor
                          *(int*)foo->para[3]->data );                                      // borderType
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
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
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[2]->data;
            cv::blur( src, dst,
                      cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                      cv::Point(ip->x, ip->y),                                          // anchor
                      *(int*)foo->para[3]->data );                                      // borderType
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );    
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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GAUSSIANBLUR, "GaussianBlur", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _int_para_ sw = {ksize.width, 1, 31};
        foo->new_para (SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sw, "ksize width");

        struct _int_para_ sh = {ksize.height, 1, 31};
        foo->new_para (SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sh, "ksize height");

        struct _double_para_ sx = {sigmaX, 0.0, 1000.0};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sx, "sigmaX");

        struct _double_para_ sy = {sigmaY, 0.0, 1000.0};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sy, "sigmaY");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType");
    }
    foo->error_flag = 0;

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
                foo->error_flag = 1;
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
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );    
}

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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CONVERTSCALEABS, "convertScaleAbs", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ al = {alpha, 0.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha");

        struct _double_para_ be = {beta, 0.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&be, "beta");
    }
    foo->error_flag = 0;

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                cv::convertScaleAbs( src, out,
                                     *(double*)foo->para[0]->data,      // alpha
                                     *(double*)foo->para[1]->data);     // beta
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
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
            cv::convertScaleAbs( src, dst,
                                 *(double*)foo->para[0]->data,      // alpha
                                 *(double*)foo->para[1]->data);     // beta
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( dst );
}

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
CV_EXPORTS_W void findContours( cv::InputOutputArray image,
                                cv::OutputArrayOfArrays contours,
                                cv::OutputArray hierarchy,
                                int mode,
                                int method, cv::Point offset
                                BUILDIN_FUNC)
{
    if (cvd_off) {
        cv::findContours( image, contours, hierarchy, mode, method, offset );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), FINDCONTOURS, "findContours", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ bt = {mode, "RetrievalModes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "mode");

        struct _enum_para_ bt2 = {method, "ContourApproximationModes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt2, "method");

        struct _point_int_ ip = {offset.x, -20000, 20000, offset.y, -20000, 20000};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "offset");
    }
    foo->error_flag = 0;

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden

        vector<vector<cv::Point> > break_contours;
        vector<cv::Vec4i> break_hierarchy;

        while (foo->state.flag.func_break) {
            try {
                struct _point_int_ *ac = (struct _point_int_ *)foo->para[2]->data;  // offset
                cv::findContours( image, break_contours, break_hierarchy,
                                  *(int*)foo->para[0]->data,      // mode,
                                  *(int*)foo->para[1]->data,      // method,
                                  cv::Point(ac->x, ac->y) );
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
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
        try {
            struct _point_int_ *ac = (struct _point_int_ *)foo->para[2]->data;  // offset
            cv::findContours( image, contours, hierarchy,
                              *(int*)foo->para[0]->data,      // mode,
                              *(int*)foo->para[1]->data,      // method,
                              cv::Point(ac->x, ac->y) );
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }

    foo->control_contours_imshow ( image, contours, hierarchy );    // Contours Ausgabe
}

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
//! \brief Laplacian
//!        Calculates the Laplacian of an image.
//! \param src
//! \param dst
//! \param ddepth
//! \param ksize
//! \param scale
//! \param delta
//! \param borderType
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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), LAPLACIAN, "Laplacian", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ ep = {ddepth, "ddepth"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "ddepth");

        struct _int_para_ sp = {ksize, 1, 31};
        foo->new_para (SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "ksize");

        struct _double_para_ dv = {scale, 0.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dv, "scale");

        struct _double_para_ dd = {delta, 0.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dd, "delta");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );
    }
    foo->error_flag = 0;

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
                foo->error_flag = 1;
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
            foo->error_flag = 1;
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
//!
CV_EXPORTS_W cv::Mat imread( const cv::String& filename, int flags
                             BUILDIN_FUNC)
{
    cv::Mat ret;

    if (cvd_off) {
        return cv::imread( filename, flags );
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), IMREAD, "imread", 0x000F, BUILIN_PARA);
        func.push_back( foo );

        struct _string_para_ al;
        strcpy (al.val, filename.c_str());
        foo->new_para (STRING_PARA, sizeof(struct _string_para_), (uint8_t*)&al, "filename");

        struct _enum_para_ ep = {flags, "ImreadModes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "flags");
    }
    foo->error_flag = 0;


    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat buf_image;
            struct _string_para_ *al = (struct _string_para_ *)foo->para[0]->data;
            try {
                buf_image = cv::imread( al->val,
                                  *(int*)foo->para[1]->data );
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
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

    if (foo->state.flag.func_off) {     // imread ist ausgeschaltet.
        return ret;                     // return ist empty !!!
    } else {
        try {
            struct _string_para_ *al = (struct _string_para_ *)foo->para[0]->data;
            ret = cv::imread( al->val,
                              *(int*)foo->para[1]->data );
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }

    if (ret.empty()) {                                      // Es ist kein Bild geladen worden !
        cv::Mat out(200, 200, CV_8UC1, cv::Scalar(128));    // fiktive cv::Mat erzeugen !
        foo->control_imshow( out );                         // fiktive cv::Mat ausgeben.
    } else
        foo->control_imshow( ret );

    return ret;
}

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
    // static uint32_t anz_error = 0;

    if (cvd_off) {
        cv::HoughCircles( image, circles, method, dp, minDist, param1, param2, minRadius, maxRadius);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), HOUGHCIRCLES, "HoughCircles", 0x0003, BUILIN_PARA);
        func.push_back( foo );

        struct _enum_para_ bt = {method, "HoughModes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "method");

        struct _double_para_ al = {dp, 0.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "dp");

        struct _double_para_ md = {minDist, 0.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&md, "minDist");

        struct _double_para_ p1 = {param1, 1.0, 255.0};            // threshold
        foo->new_para (SLIDE_DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&p1, "param1");

        struct _double_para_ p2 = {param2, 1.0, 255.0};
        foo->new_para (SLIDE_DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&p2, "param2");

        struct _int_para_ minr = {minRadius, 0, 200000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&minr, "minRadius");

        struct _int_para_ maxr = {maxRadius, 0, 200000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&maxr, "maxRadius");

    }
    foo->error_flag = 0;

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
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    // foo->control_imshow( image );
}

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
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), HOUGHLINESP, "HoughLinesP", 0x0003, BUILIN_PARA);
        func.push_back( foo );

        struct _double_para_ ro = {rho, -100000.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ro, "rho");

        struct _double_para_ ta = {theta, -100000.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&ta, "theta");

        struct _int_para_ sw = {threshold, 0, 255};
        foo->new_para (SLIDE_INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sw, "threshold");

        struct _double_para_ mll = {minLineLength, -100000.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&mll, "minLineLength");

        struct _double_para_ mlg = {maxLineGap, -100000.0, std::numeric_limits<double>::max()};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&mlg, "maxLineGap");
    }
    foo->error_flag = 0;

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
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
}


} // namespace cvd

#endif // OPENCVD_FUNC_HPP
