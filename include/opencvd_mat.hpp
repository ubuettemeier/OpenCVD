#ifndef OPENCV_MAT_HPP
#define OPENCV_MAT_HPP

#include "opencv2/opencv.hpp"

namespace cvd {

//!
//! \brief The Mat class
//!
class CV_EXPORTS Mat : public cv::Mat
{
public:
    Mat() : cv::Mat() {}

    Mat(const cv::Mat& m, const cv::Rect& roi, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    void convertTo( cv::OutputArray m, int rtype, double alpha=1, double beta=0, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE() ) const;

    Mat(cv::Size size, int type, const cv::Scalar& s, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    Mat(int rows, int cols, int type, const cv::Scalar& s, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());

    using cv::Mat::operator =;     // Mat& operator = (const Mat& m);

};

//!
//! \brief The MatExpr class
//!
class CV_EXPORTS MatExpr : public cv::MatExpr
{
public:
    using cv::MatExpr::MatExpr;
};

//!
//! \brief Mat::Mat
//! \param size
//! \param type
//! \param s
//! \param line_nr
//! \param src_file
//!
Mat::Mat(cv::Size size, int type, const cv::Scalar& s, int line_nr, const char *src_file)
{
    if (cvd_off) {
        *this = cv::Mat (size, type, s);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MAT_SIZE_TYPE_SCALAR, "Mat(Size, type, Scalar)", 0x000F, line_nr, src_file);
        func.push_back( foo );

        struct _point_int_ ip = {size.width, 0, 0xFFFF, size.height, 0, 0xFFFF};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "size");

        struct _enum_para_ dd = {type, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "type" );

        struct _rect_double_ rd = {s.val[0], s.val[1], s.val[2], s.val[3], -100000.0, 100000.0};
        foo->new_para (RECT_DOUBLE_PARA, sizeof(struct _rect_double_), (uint8_t*)&rd, "Mat(.., Scalar&)");
    }
    foo->error_flag = 0;
    // --------------------------------------------

    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            struct _rect_double_ *s_dat = (struct _rect_double_ *)foo->para[2]->data;
            try {
                out = cv::Mat (cv::Size(ip->x, ip->y),            // cv::Size
                               *(int*)foo->para[1]->data,         // type,
                               cv::Scalar(s_dat->x, s_dat->y, s_dat->w, s_dat->h));   // cv::Scalar

            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }


    // --------------------------------------------
    if (foo->state.flag.func_off) {
        *this = cv::Mat();          // leere Matrix
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        struct _rect_double_ *s_dat = (struct _rect_double_ *)foo->para[2]->data;
        try {
            *this = cv::Mat (cv::Size(ip->x, ip->y),            // cv::Size
                             *(int*)foo->para[1]->data,         // type,
                             cv::Scalar(s_dat->x, s_dat->y, s_dat->w, s_dat->h));   // cv::Scalar
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( *this );     // Bildausgabe
}
//!
//! \brief Mat::Mat
//! \param rows
//! \param cols
//! \param type
//! \param s
//!
Mat::Mat(int rows, int cols, int type, const cv::Scalar& s, int line_nr, const char *src_file )
{
    if (cvd_off) {
        *this = cv::Mat (rows, cols, type, s);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MAT_ROWS_COLS_TYPE_SCALAR, "Mat(rows, cols, type, Scalar)", 0x000F, line_nr, src_file);
        func.push_back( foo );

        struct _int_para_ ro = {rows, 0, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ro, "rows");

        struct _int_para_ co = {cols, 0, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&co, "cols");

        struct _enum_para_ dd = {type, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "type" );

        struct _rect_double_ rd = {s.val[0], s.val[1], s.val[2], s.val[3], -100000.0, 100000.0};
        foo->new_para (RECT_DOUBLE_PARA, sizeof(struct _rect_double_), (uint8_t*)&rd, "Mat(.., Scalar&)");
    }
    foo->error_flag = 0;
    // --------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _rect_double_ *s_dat = (struct _rect_double_ *)foo->para[3]->data;
            try {
                out = cv::Mat (*(int*)foo->para[0]->data,         // rows,
                        *(int*)foo->para[1]->data,         // cols,
                        *(int*)foo->para[2]->data,         // type,
                        cv::Scalar(s_dat->x, s_dat->y, s_dat->w, s_dat->h));

            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // --------------------------------------------
    if (foo->state.flag.func_off) {
        *this = cv::Mat();          // leere Matrix
    } else {
        struct _rect_double_ *s_dat = (struct _rect_double_ *)foo->para[3]->data;
        try {
            *this = cv::Mat (*(int*)foo->para[0]->data,         // rows,
                             *(int*)foo->para[1]->data,         // cols,
                             *(int*)foo->para[2]->data,         // type,
                             cv::Scalar(s_dat->x, s_dat->y, s_dat->w, s_dat->h));
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( *this );     // Bildausgabe
}

//!
//! \brief Mat::Mat
//! \param m
//! \param roi
//! \param line_nr
//! \param src_file
//!
Mat::Mat(const cv::Mat& m, const cv::Rect& roi, int line_nr, const char *src_file)
{
    if (cvd_off) {
        cv::Mat(m, roi).copyTo( *this );  // cv::Mat (Mat &m, Rect *roi)
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MAT_ROI, "Mat(Mat& Rect&)", 0x000F, line_nr, src_file);
        func.push_back( foo );

        struct _rect_int_ ri = {roi.x, roi.y, roi.width, roi.height, -10000, 10000};
        foo->new_para (RECT_INT_PARA, sizeof(struct _rect_int_), (uint8_t*)&ri, "Mat(Mat&, Rect&)");
    }
    foo->error_flag = 0;
    // -------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _rect_int_ *rec_data = (struct _rect_int_ *)foo->para[0]->data;
            try {
                cv::Rect r(rec_data->x, rec_data->y, rec_data->w, rec_data->h);
                cv::Mat(m, r).copyTo( out );  // cv::Mat (Mat &m, Rect *roi)
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -------------------------------------
    if (foo->state.flag.func_off) {
        m.copyTo( *this );
    } else {
        struct _rect_int_ *rec_data = (struct _rect_int_ *)foo->para[0]->data;
        try {
            cv::Rect r(rec_data->x, rec_data->y, rec_data->w, rec_data->h);
            cv::Mat(m, r).copyTo( *this);  // cv::Mat (Mat &m, Rect *roi)
        } catch( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( *this );     // Bildausgabe
}
//!
//! \brief Mat::convertTo
//! \param m
//! \param rtype desired output matrix type or, rather, the depth since the number of channels are the
//!        same as the input has; if rtype is negative, the output matrix will have the same type as the input.
//! \param alpha
//! \param beta
//!
void Mat::convertTo( cv::OutputArray m, int rtype, double alpha, double beta, int line_nr, const char *src_file ) const
{   
    if (cvd_off) {
        this->cv::Mat::convertTo(m, rtype, alpha, beta);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MAT_CONVERTTO, "Mat::converTo()", 0x000F, line_nr, src_file);
        func.push_back( foo );

        struct _enum_para_ dd = {rtype, "Sobel_filterdepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "ddepth" );

        struct _double_para_ al = {alpha, -100000.0, 100000.0};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha");

        struct _double_para_ be = {beta, -100000.0, 100000.0};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&be, "beta");
    }
    foo->error_flag = 0;
    // -------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                this->cv::Mat::convertTo(out,
                                         *(int*)foo->para[0]->data,         // rtype,
                                         *(double*)foo->para[1]->data,      // alpha,
                                         *(double*)foo->para[2]->data);     // beta
            } catch( cv::Exception& e ) {
                foo->error_flag = 1;
            }
            foo->control_imshow( out );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        this->copyTo( m );
    } else {
        try {
            this->cv::Mat::convertTo(m,
                                     *(int*)foo->para[0]->data,         // rtype,
                                     *(double*)foo->para[1]->data,      // alpha,
                                     *(double*)foo->para[2]->data);     // beta
        } catch ( cv::Exception& e ) {
            foo->error_flag = 1;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( m );     // Bildausgabe
}

} // namespace cvd

#endif // OPENCV_MAT_HPP
