//!
//! \author Ulrich Buettemeier
//! \todo CVD::Mat zero = CVD::Mat::zeros(250, 300, CV_8U);  funktioniert nicht !!!
//!

#ifndef OPENCV_MAT_HPP
#define OPENCV_MAT_HPP

#include "opencv2/opencv.hpp"

namespace cvd {

//!
//! \brief The MatExpr class
//!
class CV_EXPORTS MatExpr : public cv::MatExpr
{
public:
    // MatExpr() : cv::MatExpr() {}
    using cv::MatExpr::MatExpr;

};


//!
//! \brief The Mat class
//!
class CV_EXPORTS Mat : public cv::Mat
{
public:
    Mat() : cv::Mat() {}

    Mat(int rows, int cols, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    Mat(cv::Size size, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    Mat(int rows, int cols, int type, const cv::Scalar& s, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    Mat(cv::Size size, int type, const cv::Scalar& s, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());

    Mat(const cv::Mat& m, const cv::Rect& roi, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    // ---------------------------------------------------------------------------------------
    static cv::MatExpr zeros(int rows, int cols, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    static cv::MatExpr zeros(cv::Size size, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    static cv::MatExpr zeros(int ndims, const int* sz, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());

    static cv::MatExpr ones(int rows, int cols, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    static cv::MatExpr ones(cv::Size size, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    static cv::MatExpr ones(int ndims, const int* sz, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());

    static cv::MatExpr eye(int rows, int cols, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    static cv::MatExpr eye(cv::Size size, int type, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());

    static cv::MatExpr func_type_1(int rows, int cols, int type,
                                 int line_nr, const char *src_file,
                                 int cv_type, const char *func_name,
                                 uint64_t builtin_retunr_adresse);

    static cv::MatExpr func_type_2(cv::Size size, int type,
                                 int line_nr, const char *src_file,
                                 int cv_type, const char *func_name,
                                 uint64_t builtin_retunr_adresse);

    static cv::MatExpr func_type_3(int ndims, const int* sz, int type,
                             int line_nr, const char *src_file,
                             int cv_type, const char *func_name,
                             uint64_t builtin_retunr_adresse);
    // ---------------------------------------------------------------------------------------
    void convertTo( cv::OutputArray m, int rtype, double alpha=1, double beta=0, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE() ) const;
    void assignTo( cv::Mat& m, int type=-1, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE() ) const;

    using cv::Mat::operator =;     // Mat& operator = (const Mat& m);    

    Mat& operator *= (double);
    Mat operator * (double);

    /*
    Mat& operator = (const MatExpr& e)
    {
        printf ("dsksdfsd\n");
        e.op->assign(e, *this);
        return *this;
    }
    */
};

//!
//! \brief The MSER class
//!
class CV_EXPORTS_W MSER : public cv::MSER
{
public:
    CV_WRAP static cv::Ptr<cv::MSER> create( int _delta=5, int _min_area=60, int _max_area=14400,
                                            double _max_variation=0.25, double _min_diversity=.2,
                                            int _max_evolution=200, double _area_threshold=1.01,
                                            double _min_margin=0.003, int _edge_blur_size=5,
                                            int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
};

//!
//! \brief MSER::create
//! \param _delta
//! \param _min_area
//! \param _max_area
//! \param _max_variation
//! \param _min_diversity
//! \param _max_evolution
//! \param _area_threshold
//! \param _min_margin
//! \param _edge_blur_size
//! \return
//!
CV_WRAP cv::Ptr<cv::MSER> MSER::create( int _delta, int _min_area, int _max_area,
                                        double _max_variation, double _min_diversity,
                                        int _max_evolution, double _area_threshold,
                                        double _min_margin, int _edge_blur_size,
                                        int line_nr, const char *src_file)
{
    if (line_nr) {}
    if (src_file) {}
    cv::Ptr<cv::MSER> ms = cv::MSER::create(_delta, _min_area, _max_area, _max_variation, _min_diversity, _max_evolution, _area_threshold, _min_margin, _edge_blur_size);
    return ms;
}

//!
//! \brief Mat::operator *
//! \param val
//! \return
//!
Mat Mat::operator * (double val)
{
    printf ("* %f\n", val);
    /*
    if (!this->empty()) {
        try {
            cv::Mat *a = static_cast <cv::Mat *>(this);
            *a *= val;
        } catch( cv::Exception& e ) {
            printf ("ERROR\n");
        }
    }
    */
    return *this *= val;
}

//!
//! \brief Mat::operator *=
//! \param val
//! \return
//!
Mat& Mat::operator *= (double val)
{
    printf ("*= %f\n", val);
    if (!this->empty()) {
        try {
            cv::Mat *a = static_cast <cv::Mat *>(this);
            *a *= val;
        } catch( cv::Exception& e ) {
            printf ("ERROR\n");
        }
    }
    return *this;
}

//!
//! \brief Mat::Mat
//! \param size
//! \param type
//! \param line_nr
//! \param src_file
//!
Mat::Mat(cv::Size size, int type, int line_nr, const char *src_file)
{
    if (cvd_off) {
        *this = cv::Mat (size, type);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MAT_SIZE_TYPE, "Mat(size, type)", 0x000F, line_nr, src_file);
        func.push_back( foo );

        struct _point_int_ ip = {size.width, 0, 0xFFFF, size.height, 0, 0xFFFF};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "size");

        struct _enum_para_ dd = {type, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "type" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            try {
                out = cv::Mat (cv::Size(ip->x, ip->y),            // cv::Size
                               *(int*)foo->para[1]->data);        // type,

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
        *this = cv::Mat();          // leere Matrix
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        try {
            *this = cv::Mat (cv::Size(ip->x, ip->y),            // cv::Size
                             *(int*)foo->para[1]->data);        // type
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( *this );     // Bildausgabe
}

//!
//! \brief Mat
//! \param rows
//! \param cols
//! \param type
//! \param line_nr
//! \param src_file
//!
Mat::Mat(int rows, int cols, int type, int line_nr, const char *src_file)
{
    if (cvd_off) {
        *this = cv::Mat (rows, cols, type);
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MAT_ROWS_COLS_TYPE, "Mat(rows, cols, type)", 0x000F, line_nr, src_file);
        func.push_back( foo );

        struct _int_para_ ro = {rows, 0, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ro, "rows");

        struct _int_para_ co = {cols, 0, 10000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&co, "cols");

        struct _enum_para_ dd = {type, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&dd, "type" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // --------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                out = cv::Mat (*(int*)foo->para[0]->data,         // rows,
                               *(int*)foo->para[1]->data,         // cols,
                               *(int*)foo->para[2]->data);        // type,

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
        *this = cv::Mat();          // leere Matrix
    } else {
        try {
            *this = cv::Mat (*(int*)foo->para[0]->data,         // rows,
                             *(int*)foo->para[1]->data,         // cols,
                             *(int*)foo->para[2]->data);        // type,
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( *this );     // Bildausgabe
}

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
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
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
                foo->error_flag |= FUNC_ERROR;
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
            foo->error_flag |= FUNC_ERROR;
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
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
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
                foo->error_flag |= FUNC_ERROR;
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
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( *this );     // Bildausgabe
}

//!
//! \brief zeros
//! \param size
//! \param type
//! \param line_nr
//! \param src_file
//! \return
//! type 2
cv::MatExpr Mat::zeros(cv::Size size, int type, int line_nr, const char *src_file)
{
    uint64_t ret_addr = (uint64_t)__builtin_return_address(0);

    cv::MatExpr ret = func_type_2 (size, type,
                               line_nr, src_file,
                               MAT_ZEROS_2, "Mat::zeros [type 2]",
                               ret_addr);
    return ret;
}
//!
//! \brief Mat::ones
//! \param size
//! \param type
//! \return
//! type 2
cv::MatExpr Mat::ones(cv::Size size, int type, int line_nr, const char *src_file)
{
    uint64_t ret_addr = (uint64_t)__builtin_return_address(0);

    cv::MatExpr ret = func_type_2 (size, type,
                               line_nr, src_file,
                               MAT_ONES_2, "Mat::ones [type 2]",
                               ret_addr);
    return ret;
}
//!
//! \brief Mat::ones
//! \param size
//! \param type
//! \return
//! type 2
cv::MatExpr Mat::eye(cv::Size size, int type, int line_nr, const char *src_file)
{
    uint64_t ret_addr = (uint64_t)__builtin_return_address(0);

    cv::MatExpr ret = func_type_2 (size, type,
                               line_nr, src_file,
                               MAT_EYE_2, "Mat::eye [type 2]",
                               ret_addr);
    return ret;
}

// ------------------------------------------------------------------------------------------
cv::MatExpr Mat::func_type_2(cv::Size size, int type,
                             int line_nr, const char *src_file,
                             int cv_type, const char *func_name,
                             uint64_t builtin_retunr_adresse)
{
    cv::MatExpr ret;

    if (cvd_off) {
        if (cv_type == MAT_ONES_2)
            return cv::Mat::ones( size, type ); // return static_cast<MatExpr>(cv::Mat::ones( size, type ));
        if (cv_type == MAT_ZEROS_2)
            return cv::Mat::zeros( size, type ); // return static_cast<MatExpr>(cv::Mat::zeros( size, type ));
        if (cv_type == MAT_EYE_2)
            return cv::Mat::eye( size, type ); // return static_cast<MatExpr>(cv::Mat::eye( size, type ));
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, builtin_retunr_adresse)) == NULL) {
        foo = new opencvd_func(builtin_retunr_adresse, cv_type, func_name,
                               PARAMETER | FUNC_OFF | BREAK | SHOW_IMAGE,    // Menu
                               line_nr, src_file);

        func.push_back( foo );

        struct _point_int_ ip = {size.width, 0, 0xFFFF, size.height, 0, 0xFFFF};
        foo->new_para (POINT_INT, sizeof(struct _point_int_), (uint8_t*)&ip, "size");

        struct _enum_para_ tp = {type, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&tp, "type" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear FUNC_ERROR
    // -------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::MatExpr result;
            struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
            try {
                if (cv_type == MAT_ONES_2) {
                    result = cv::Mat::ones(cv::Size(ip->x, ip->y),          // cv::Size
                                           *(int*)foo->para[1]->data);      // type
                }
                if (cv_type == MAT_ZEROS_2) {
                    result = cv::Mat::zeros(cv::Size(ip->x, ip->y),          // cv::Size
                                           *(int*)foo->para[1]->data);      // type
                }
                if (cv_type == MAT_EYE_2) {
                    result = cv::Mat::eye(cv::Size(ip->x, ip->y),          // cv::Size
                                           *(int*)foo->para[1]->data);      // type
                }
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( static_cast<cv::Mat>(result) );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -------------------------------------
    if (foo->state.flag.func_off) {
        return static_cast<cv::MatExpr>(cv::Mat());
    } else {
        struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
        try {
            if (cv_type == MAT_ONES_2) {
                ret = cv::Mat::ones(cv::Size(ip->x, ip->y),         // cv::Size
                                    *(int*)foo->para[1]->data);     // type
            }
            if (cv_type == MAT_ZEROS_2) {
                ret = cv::Mat::zeros(cv::Size(ip->x, ip->y),         // cv::Size
                                     *(int*)foo->para[1]->data);     // type
            }
            if (cv_type == MAT_EYE_2) {
                ret = cv::Mat::eye(cv::Size(ip->x, ip->y),         // cv::Size
                                   *(int*)foo->para[1]->data);     // type
            }
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( static_cast<cv::Mat>(ret) );     // Bildausgabe

    return ret; // return static_cast<MatExpr>(ret);
} // MatExpr Mat::func_type_2()

//!
//! \brief Mat::ones
//! \param ndims Array dimensionality. // max. 2 ???
//! \param sz Array of integers specifying the array shape.
//! \param type Created matrix type.
//! \return
//! type 3
cv::MatExpr Mat::ones(int ndims, const int* sz, int type, int line_nr, const char *src_file)
{
    uint64_t ret_addr = (uint64_t)__builtin_return_address(0);

    cv::MatExpr ret = func_type_3 (ndims, sz, type,
                                   line_nr, src_file,
                                   MAT_ONES_3, "Mat::ones [type 3]",
                                   ret_addr);
    return ret;
}
//!
//! \brief Mat::zeros
//! \param ndims
//! \param sz
//! \param type
//! \return
//! type 3
cv::MatExpr Mat::zeros(int ndims, const int* sz, int type, int line_nr, const char *src_file)
{
    uint64_t ret_addr = (uint64_t)__builtin_return_address(0);

    cv::MatExpr ret = func_type_3 (ndims, sz, type,
                                   line_nr, src_file,
                                   MAT_ZEROS_3, "Mat::zeros [type 3]",
                                   ret_addr);
    return ret;
}

// ------------------------------------------------------------------------------------------
cv::MatExpr Mat::func_type_3(int ndims, const int* sz, int type,
                             int line_nr, const char *src_file,
                             int cv_type, const char *func_name,
                             uint64_t builtin_retunr_adresse)
{
    cv::MatExpr ret;
    int this_size[2] = {0, 0};

    if (cvd_off) {
        if (cv_type == MAT_ONES_3)
            return cv::Mat::ones( ndims, sz, type );  // return static_cast<MatExpr>(cv::Mat::ones( ndims, sz, type ));
        if (cv_type == MAT_ZEROS_3)
            return cv::Mat::zeros( ndims, sz, type );  // return static_cast<MatExpr>(cv::Mat::zeros( ndims, sz, type ));
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, builtin_retunr_adresse)) == NULL) {
        foo = new opencvd_func(builtin_retunr_adresse, cv_type, func_name,
                               PARAMETER | FUNC_OFF | BREAK | SHOW_IMAGE,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        struct _int_para_ nd = {ndims, 0, 2};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&nd, "ndims");

        if (ndims >= 1)
            this_size[0] = sz[0];
        struct _int_para_ s0 = {this_size[0], 0, 1000000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&s0, "sz[0]");

        if (ndims >= 2)
            this_size[1] = sz[1];
        struct _int_para_ s1 = {this_size[1], 0, 1000000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&s1, "sz[1]");

        struct _enum_para_ tp = {type, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&tp, "type" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear FUNC_ERROR
    // -------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::MatExpr result;
            this_size[0] = *(int*)foo->para[1]->data;
            this_size[1] = *(int*)foo->para[2]->data;
            try {
                if (cv_type == MAT_ONES_3)
                    result = cv::Mat::ones(*(int*)foo->para[0]->data,
                                           this_size,
                                           *(int*)foo->para[3]->data);
                if (cv_type == MAT_ZEROS_3)
                    result = cv::Mat::zeros(*(int*)foo->para[0]->data,
                                           this_size,
                                           *(int*)foo->para[3]->data);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( static_cast<cv::Mat>(result) );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -------------------------------------
    if (foo->state.flag.func_off) {
        return static_cast<cv::MatExpr>(cv::Mat());
    } else {
        this_size[0] = *(int*)foo->para[1]->data;
        this_size[1] = *(int*)foo->para[2]->data;
        try {
            if (cv_type == MAT_ONES_3)
                ret = cv::Mat::ones(*(int*)foo->para[0]->data,
                                    this_size,
                                    *(int*)foo->para[3]->data);
            if (cv_type == MAT_ZEROS_3)
                ret = cv::Mat::zeros(*(int*)foo->para[0]->data,
                                    this_size,
                                    *(int*)foo->para[3]->data);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( static_cast<cv::Mat>(ret) );     // Bildausgabe

    return ret; // static_cast<MatExpr>(ret);
} // MatExpr Mat::func_type_3()

//!
//! \brief Mat::ones    Gibt ein Array aller Einsen der angegebenen Größe und des angegebenen Typs zurück.
//! \param rows Number of rows.
//! \param cols Number of cols.
//! \param type Created matrix type.
//! \example Mat A = Mat::ones(100, 100, CV_8U)*3; // make 100x100 matrix filled with 3.
//! \return
//! type 1
cv::MatExpr Mat::ones(int rows, int cols, int type, int line_nr, const char *src_file)
{
    uint64_t ret_addr = (uint64_t)__builtin_return_address(0);

    cv::MatExpr ret = func_type_1 (rows, cols, type,
                               line_nr, src_file,
                               MAT_ONES, "Mat::ones [type 1]",
                               ret_addr);
    return ret;
}

//!
//! \brief Mat::zero
//! \param rows Number of rows.
//! \param cols Number of cols.
//! \param type Created matrix type.
//! \return
//! type 1
cv::MatExpr Mat::zeros(int rows, int cols, int type, int line_nr, const char *src_file)
{
    uint64_t ret_addr = (uint64_t)__builtin_return_address(0);

    cv::MatExpr ret = func_type_1 (rows, cols, type,
                               line_nr, src_file,
                               MAT_ZEROS, "Mat::zero [type 1]",
                               ret_addr);
    return ret;
}

//!
//! \brief Mat::eye
//! \param rows
//! \param cols
//! \param type
//! \example CVD::Mat A = CVD::Mat::eye(4, 4, CV_32F)*0.1;
//! \return
//! type 1
cv::MatExpr Mat::eye(int rows, int cols, int type, int line_nr, const char *src_file)
{
    uint64_t ret_addr = (uint64_t)__builtin_return_address(0);

   cv::MatExpr ret = func_type_1 (rows, cols, type,
                               line_nr, src_file,
                               MAT_EYE, "Mat::eye [type 1]",
                               ret_addr);
    return ret;
}

// --------------------------------------------------------------------------------------------------------
cv::MatExpr Mat::func_type_1(int rows, int cols, int type,
                             int line_nr, const char *src_file,
                             int cv_type, const char *func_name,
                             uint64_t builtin_retunr_adresse)
{
    cv::MatExpr ret;

    if (cvd_off) {
        if (cv_type == MAT_ONES)
            return cv::Mat::ones( rows, cols, type ); // return static_cast<MatExpr>(cv::Mat::ones( rows, cols, type ));
        if (cv_type == MAT_ZEROS)
            return cv::Mat::zeros( rows, cols, type ); // return static_cast<MatExpr>(cv::Mat::zeros( rows, cols, type ));
        if (cv_type == MAT_EYE)
            return cv::Mat::eye( rows, cols, type ); // return static_cast<MatExpr>(cv::Mat::eye( rows, cols, type ));

    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, builtin_retunr_adresse)) == NULL) {
        foo = new opencvd_func(builtin_retunr_adresse, cv_type, func_name,
                               PARAMETER | FUNC_OFF | BREAK | SHOW_IMAGE,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        struct _int_para_ ro = {rows, 0, 1000000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ro, "rows");

        struct _int_para_ co = {cols, 0, 1000000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&co, "cols");

        struct _enum_para_ tp = {type, "ddepth"};
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&tp, "type" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear FUNC_ERROR
    // -------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::MatExpr result;
            try {
                if (cv_type == MAT_ONES)
                    result = cv::Mat::ones(*(int*)foo->para[0]->data,
                                           *(int*)foo->para[1]->data,
                                           *(int*)foo->para[2]->data);
                if (cv_type == MAT_ZEROS)
                    result = cv::Mat::zeros(*(int*)foo->para[0]->data,
                                            *(int*)foo->para[1]->data,
                                            *(int*)foo->para[2]->data);
                if (cv_type == MAT_EYE)
                    result = cv::Mat::eye(*(int*)foo->para[0]->data,
                                          *(int*)foo->para[1]->data,
                                          *(int*)foo->para[2]->data);
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
            }
            foo->control_imshow( static_cast<cv::Mat>(result) );                 // Ausgabe
            cv::waitKey(10);
            foo->control_func_run_time ();
        }
    }
    // -------------------------------------
    if (foo->state.flag.func_off) {
        return static_cast<cv::MatExpr>(cv::Mat()); // return static_cast<MatExpr>(Mat());
    } else {
        try {
            if (cv_type == MAT_ONES)
                ret = cv::Mat::ones(*(int*)foo->para[0]->data,
                                    *(int*)foo->para[1]->data,
                                    *(int*)foo->para[2]->data);
            if (cv_type == MAT_ZEROS)
                ret = cv::Mat::zeros(*(int*)foo->para[0]->data,
                                     *(int*)foo->para[1]->data,
                                     *(int*)foo->para[2]->data);
            if (cv_type == MAT_EYE)
                ret = cv::Mat::eye(*(int*)foo->para[0]->data,
                                   *(int*)foo->para[1]->data,
                                   *(int*)foo->para[2]->data);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( static_cast<cv::Mat>(ret) );     // Bildausgabe

    return ret; // return static_cast<MatExpr>(ret);
} // Mat::func_type_1

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
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
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
                foo->error_flag |= FUNC_ERROR;
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
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( *this );     // Bildausgabe
}

//!
//! \brief Mat::assignTo
//! \param m Destination array.
//! \param type Desired destination array depth (or -1 if it should be the same as the source type).
//!
void Mat::assignTo( cv::Mat& m, int type, int line_nr, const char *src_file ) const
{
    if (cvd_off) {
        this->cv::Mat::assignTo( m, type );
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), MAT_ASSIGNTO, "Mat::assignTo",
                               PARAMETER | FUNC_OFF | BREAK | SHOW_IMAGE,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        struct _int_para_ ty = {type, -1, 1000000};
        foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ty, "type");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // -------------------------------------------------------------------
    if (foo->state.flag.func_break) {                   // Break
        foo->state.flag.show_image = 1;                 // Fenster automatisch einblenden
        while (foo->state.flag.func_break) {
            cv::Mat out;
            try {
                this->cv::Mat::assignTo(out,
                                        *(int*)foo->para[0]->data);         // type,
            } catch( cv::Exception& e ) {
                foo->error_flag |= FUNC_ERROR;
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
            this->cv::Mat::assignTo(m,
                                    *(int*)foo->para[0]->data);         // type,
        } catch ( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( m );     // Bildausgabe
} // Mat::assignTo

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

        struct _double_para_ al = {alpha, -100000.0, 100000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&al, "alpha");

        struct _double_para_ be = {beta, -100000.0, 100000.0, 2};
        foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&be, "beta");
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
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
                foo->error_flag |= FUNC_ERROR;
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
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    foo->control_imshow( m );     // Bildausgabe
} // Mat::convertTo

} // namespace cvd

#endif // OPENCV_MAT_HPP
