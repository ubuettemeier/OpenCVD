#ifndef OPENCVD_FUNC_HPP
#define OPENCVD_FUNC_HPP

#include <opencv.hpp>

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

        printf ("alpha=%f\n", alpha);
        struct _double_para_ al = {alpha, -1000.0, 1000.0};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha" );

        struct _double_para_ be = {beta, -1000.0, 1000.0};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&be, "beta" );

        struct _enum_para_ ep = {norm_type, "NormTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "norm_type" );

        struct _enum_para_ dd = {dtype, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "dtype" );
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        cv::normalize (src, dst,
                    *(double*)foo->para[0]->data,
                    *(double*)foo->para[1]->data,
                    *(int*)foo->para[2]->data,
                    *(int*)foo->para[3]->data,
                    mask);

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

        struct _point_int_ ip = {ksize.width, ksize.height};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "ksize");      // Matrix w, h

        struct _point_int_ ac = {anchor.x, anchor.y};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ac, "anchor");     // Ankerpunkt default -1 / -1
    }

    // Achtung: Funktion hat kein ON/OFF, kein Break und kein show !!!
    struct _point_int_ *ip = (struct _point_int_ *)foo->para[1]->data;
    struct _point_int_ *ac = (struct _point_int_ *)foo->para[2]->data;
    ret = cv::getStructuringElement ( *(int*)foo->para[0]->data,
                                      cv::Size(ip->x, ip->y),
                                      cv::Point(ac->x, ac->y) );
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

        struct _point_int_ ip = {anchor.x, anchor.y};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _int_para_ sp = {iterations, -10000, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "iterations");

        struct _enum_para_ ep = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "borderType" );

        // cv::Scalar& borderValue   not implemented
    }

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            cv::erode( src, out, kernel,
                        cv::Point(ip->x, ip->y),
                        *(int*)foo->para[1]->data,
                        *(int*)foo->para[2]->data,
                        borderValue);
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        cv::erode( src, dst, kernel,
                    cv::Point(ip->x, ip->y),
                    *(int*)foo->para[1]->data,
                    *(int*)foo->para[2]->data,
                    borderValue);
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

        struct _point_int_ ip = {anchor.x, anchor.y};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _int_para_ sp = {iterations, -10000, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "iterations");

        struct _enum_para_ ep = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "borderType" );

        // cv::Scalar& borderValue   not implemented
    }

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            cv::dilate( src, out, kernel,
                        cv::Point(ip->x, ip->y),
                        *(int*)foo->para[1]->data,
                        *(int*)foo->para[2]->data,
                        borderValue);
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        cv::dilate( src, dst, kernel,
                    cv::Point(ip->x, ip->y),
                    *(int*)foo->para[1]->data,
                    *(int*)foo->para[2]->data,
                    borderValue);
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

        struct _point_int_ ip = {anchor.x, anchor.y};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _int_para_ sp = {iterations, -10000, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "iterations");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType" );

        // cv::Scalar& borderValue   not implemented
    }

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[1]->data;
            cv::morphologyEx( src, out,
                        *(int*)foo->para[0]->data,      // op
                        kernel,
                        cv::Point(ip->x, ip->y),        // anchor
                        *(int*)foo->para[2]->data,      // iterations
                        *(int*)foo->para[3]->data,      // borderType
                        borderValue);
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[1]->data;
        cv::morphologyEx( src, dst,
                    *(int*)foo->para[0]->data,      // op
                    kernel,
                    cv::Point(ip->x, ip->y),        // anchor
                    *(int*)foo->para[2]->data,      // iterations
                    *(int*)foo->para[3]->data,      // borderType
                    borderValue);
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

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            cv::cvtColor(src, out,
                         *(int*)foo->para[0]->data,
                         *(int*)foo->para[1]->data);
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        cv::cvtColor(src, dst,
                     *(int*)foo->para[0]->data,
                     *(int*)foo->para[1]->data);
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

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            ret = cv::threshold( src, out,
                               *(double*)foo->para[0]->data,
                               *(double*)foo->para[1]->data,
                               *(int*)foo->para[2]->data);
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        ret = cv::threshold( src, dst,
                           *(double*)foo->para[0]->data,
                           *(double*)foo->para[1]->data,
                           *(int*)foo->para[2]->data);
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

        struct _int_para_ sp = {apertureSize, 3, 7};
        foo->new_para ( SLIDE_INT_TWO_STEP_PARA, sizeof(struct _int_para_), (uint8_t*)&sp, "apertureSize" );

        struct _enum_para_ ep = {L2gradient, "boolType"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&ep, "L2gradient" );

    }

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            cv::Canny (image, out,
                       *(double*)foo->para[0]->data,
                       *(double*)foo->para[1]->data,
                       *(int*)foo->para[2]->data,
                       *(int*)foo->para[3]->data);
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
        cv::Canny (image, edges,
                   *(double*)foo->para[0]->data,
                   *(double*)foo->para[1]->data,
                   *(int*)foo->para[2]->data,
                   *(int*)foo->para[3]->data);
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

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            cv::Canny (dx, dy, out,
                       *(double*)foo->para[0]->data,
                       *(double*)foo->para[1]->data,
                       *(int*)foo->para[2]->data);
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {                 // Function OFF
        (cv::Mat&)edges = cv::Scalar(0);
    } else {
        cv::Canny (dx, dy, edges,
                   *(double*)foo->para[0]->data,
                   *(double*)foo->para[1]->data,
                   *(int*)foo->para[2]->data);
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

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            cv::medianBlur (src, out,
                        *(int*)foo->para[0]->data);
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        cv::medianBlur (src, dst,
                    *(int*)foo->para[0]->data);
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

        struct _point_int_ ip = {anchor.x, anchor.y};
        foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "anchor");

        struct _enum_para_ bt = {borderType, "BorderTypes"};
        foo->new_para (ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&bt, "borderType");
    }

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;

            struct _point_int_ *ip = (struct _point_int_ *)foo->para[2]->data;
            cv::blur( src, out,
                      cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                      cv::Point(ip->x, ip->y),                                          // anchor
                      *(int*)foo->para[3]->data );                                      // borderType

            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo ( dst );
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[2]->data;
        cv::blur( src, dst,
                  cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                  cv::Point(ip->x, ip->y),                                          // anchor
                  *(int*)foo->para[3]->data );                                      // borderType
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

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            cv::GaussianBlur( src, out,
                              cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                              *(double*)foo->para[2]->data,                                     // sigmaX
                              *(double*)foo->para[3]->data,                                     // sigmaY
                              *(int*)foo->para[4]->data );                                      // borderType
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        cv::GaussianBlur( src, dst,
                          cv::Size(*(int*)foo->para[0]->data, *(int*)foo->para[1]->data),   // ksize
                          *(double*)foo->para[2]->data,                                     // sigmaX
                          *(double*)foo->para[3]->data,                                     // sigmaY
                          *(int*)foo->para[4]->data );                                      // borderType
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

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            cv::convertScaleAbs( src, out,
                                 *(double*)foo->para[0]->data,      // alpha
                                 *(double*)foo->para[1]->data);     // beta

            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        cv::convertScaleAbs( src, dst,
                             *(double*)foo->para[0]->data,      // alpha
                             *(double*)foo->para[1]->data);     // beta

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

        struct _point_int_ ip = {offset.x, offset.y};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "offset");
    }

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden

        vector<vector<cv::Point> > break_contours;
        vector<cv::Vec4i> break_hierarchy;

        while (foo->state.flag.func_break) {

            struct _point_int_ *ac = (struct _point_int_ *)foo->para[2]->data;  // offset
            cv::findContours( image, break_contours, break_hierarchy,
                              *(int*)foo->para[0]->data,      // mode,
                              *(int*)foo->para[1]->data,      // method,
                              cv::Point(ac->x, ac->y) );

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
        cv::findContours( image, contours, hierarchy,
                          *(int*)foo->para[0]->data,      // mode,
                          *(int*)foo->para[1]->data,      // method,
                          cv::Point(ac->x, ac->y) );
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

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            cv::Laplacian( src, out,
                              *(int*)foo->para[0]->data,        // ddepth
                              *(int*)foo->para[1]->data,        // ksize
                              *(double*)foo->para[2]->data,     // scale
                              *(double*)foo->para[3]->data,     // delta
                              *(int*)foo->para[4]->data);       // borderType
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }

    if (foo->state.flag.func_off) {
        src.copyTo( dst );
    } else {
        cv::Laplacian( src, dst,
                          *(int*)foo->para[0]->data,        // ddepth
                          *(int*)foo->para[1]->data,        // ksize
                          *(double*)foo->para[2]->data,     // scale
                          *(double*)foo->para[3]->data,     // delta
                          *(int*)foo->para[4]->data);       // borderType
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


    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat buf_image;
            struct _string_para_ *al = (struct _string_para_ *)foo->para[0]->data;
            buf_image = cv::imread( al->val,
                              *(int*)foo->para[1]->data );

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
        struct _string_para_ *al = (struct _string_para_ *)foo->para[0]->data;
        ret = cv::imread( al->val,
                          *(int*)foo->para[1]->data );
        foo->control_func_run_time ();
    }

    if (ret.empty()) {                                      // Es ist kein Bild geladen worden !
        cv::Mat out(200, 200, CV_8UC1, cv::Scalar(128));    // fiktive cv::Mat erzeugen !
        foo->control_imshow( out );                         // fiktive cv::Mat ausgeben.
    } else
        foo->control_imshow( ret );

    return ret;
}


} // namespace cvd

#endif // OPENCVD_FUNC_HPP
