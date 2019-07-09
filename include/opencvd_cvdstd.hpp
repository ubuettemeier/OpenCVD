//!
//! \author Ulrich Buettemeier
//! \todo CVD::Mat zero = CVD::Mat::zeros(250, 300, CV_8U);  funktioniert nicht !!!
//! \date 07-06-2019
//!

#ifndef OPENCV_CVDSTD_HPP
#define OPENCV_CVDSTD_HPP

#include "opencv2/opencv.hpp"
#include <type_traits>
#include <memory>

#define BUILD_IN_PROTO  int line_nr = __builtin_LINE(), \
                        const char *src_file = __builtin_FILE()

namespace cvd {

//////////////////////////////// Point_ ////////////////////////////////
//!
//! \brief The Point_ class
//!
template<typename _Tp>
class Point_ : public cv::Point_<_Tp>
{
public:
    using cv::Point_<_Tp>::Point_;

    Point_(_Tp _x, _Tp _y, BUILD_IN_PROTO);
};

typedef Point_<int> Point2i;
typedef Point_<int64> Point2l;
typedef Point_<float> Point2f;
typedef Point_<double> Point2d;
typedef Point2i Point;

//!
//! \brief Point_<_Tp>::Point_
//! \param _x
//! \param _y
//!
template<typename _Tp>
Point_<_Tp>::Point_(_Tp _x, _Tp _y, int line_nr, const char *src_file) : cv::Point_<_Tp>::Point_(_x, _y)
{
    if (cvd_off) {
        return;
    }

    int type = CVD_POINT_TYPE_1_INT;
    char fname[64] = "cvd::Point<int>";

    if (std::is_same<_Tp, int64>::value) {
        type = CVD_POINT_TYPE_1_INT64;
        strcpy (fname, "cvd::Point<int64>");
    }

    if (std::is_same<_Tp, float>::value) {
        type = CVD_POINT_TYPE_1_FLOAT;
        strcpy (fname, "cvd::Point<float>");
    }

    if (std::is_same<_Tp, double>::value) {
        type = CVD_POINT_TYPE_1_DOUBLE;
        strcpy (fname, "cvd::Point<double>");
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), type, fname,
                               PARAMETER | FUNC_OFF,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        switch (type) {
        case CVD_POINT_TYPE_1_INT64:
        case CVD_POINT_TYPE_1_INT: {
            struct _point_int_ ip = {(int)_x, -20000, 20000, (int)_y, -20000, 20000};
            foo->new_para (POINT_INT_XY, sizeof(struct _point_int_), (uint8_t*)&ip, "Point(x, y)");
            }
            break;
        case CVD_POINT_TYPE_1_FLOAT:
        case CVD_POINT_TYPE_1_DOUBLE: {
            struct _point_double_ dp = {(double)_x, -1000000.0, 1000000.0, 3, (double)_y, -1000000.0, 1000000.0, 3};
            foo->new_para (POINT_DOUBLE_XY, sizeof(struct _point_double_), (uint8_t*)&dp, "Point(x, y)");
            }
            break;
        }
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // ----------------------------------------------------------------------------------------------
    // no break-function
    // ----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        // nothing to do
    } else {
        try {
            switch (type) {
            case CVD_POINT_TYPE_1_INT64:
            case CVD_POINT_TYPE_1_INT: {
                struct _point_int_ *ip = (struct _point_int_ *)foo->para[0]->data;
                this->x = ip->x;
                this->y = ip->y;
                }
                break;
            case CVD_POINT_TYPE_1_FLOAT:
            case CVD_POINT_TYPE_1_DOUBLE: {
                struct _point_double_ *dp = (struct _point_double_ *) foo->para[0]->data;
                this->x = dp->x;
                this->y = dp->y;
                }
                break;
            }
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
}

//////////////////////////////// Scalar_ ////////////////////////////////
//!
//! \brief The Scalar_ class
//!
template<typename _Tp>
class Scalar_ : public cv::Scalar_<_Tp>
{
public:
    using cv::Scalar_<_Tp>::Scalar_;

    Scalar_(_Tp v0, _Tp v1, _Tp v2=0, _Tp v3=0, BUILD_IN_PROTO);
    Scalar_(_Tp v0, BUILD_IN_PROTO);
};

typedef Scalar_<double> Scalar;

//!
//! \brief Scalar_<_Tp>::Scalar_
//! \param v0
//!
template<typename _Tp>
Scalar_<_Tp>::Scalar_(_Tp v0, int line_nr, const char *src_file) : cv::Scalar_<_Tp>::Scalar_(v0)
{
    if (cvd_off) {
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CVD_SCALAR_2, "cvd::Scalar",
                               PARAMETER | FUNC_OFF,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        struct _double_para_ dv = {(double)v0, -1000000.0, 1000000.0, 3};
        foo->new_para ( DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&dv, "v0" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // ----------------------------------------------------------------------------------------------
    // no break-function
    // ----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        // nothing to do
    } else {
        try {
            this->val[0] = *(double *)foo->para[0]->data;
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }

}

//!
//! \brief Scalar_<_Tp>::Scalar_
//! \param v0
//! \param v1
//! \param v2
//! \param v3
//!
template<typename _Tp>
Scalar_<_Tp>::Scalar_(_Tp v0, _Tp v1, _Tp v2, _Tp v3, int line_nr, const char *src_file) : cv::Scalar_<_Tp>::Scalar_(v0, v1, v2, v3)
{
    if (cvd_off) {
        return;
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), CVD_SCALAR_1, "cvd::Scalar",
                               PARAMETER | FUNC_OFF,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        struct _rect_double_ rd = {(double)v0, (double)v1, (double)v2, (double)v3, -1000000.0, 1000000.0};
        foo->new_para (RECT_DOUBLE_PARA, sizeof(struct _rect_double_), (uint8_t*)&rd, "Scalar<double>" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // ----------------------------------------------------------------------------------------------
    // no break-function
    // ----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        // nothing to do
    } else {
        try {
            struct _rect_double_ *s_dat = (struct _rect_double_ *)foo->para[0]->data;
            this->val[0] = s_dat->x;
            this->val[1] = s_dat->y;
            this->val[2] = s_dat->w;
            this->val[3] = s_dat->h;
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
}

//////////////////////////////// Rect_ ////////////////////////////////
//!
//! \brief The Rect_ class
//!
template<typename _Tp>
class Rect_ : public cv::Rect_<_Tp>
{
public:
    using cv::Rect_<_Tp>::Rect_;

    Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height, BUILD_IN_PROTO);
};

typedef Rect_<int> Rect2i;
typedef Rect_<float> Rect2f;
typedef Rect_<double> Rect2d;
typedef Rect2i Rect;

//!
//! \brief Rect_<_Tp>::Rect_
//! \param _x
//! \param _y
//! \param _width
//! \param _height
//!
template<typename _Tp>
Rect_<_Tp>::Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height, int line_nr, const char *src_file) : cv::Rect_<_Tp>::Rect_(_x, _y, _width, _height)
{
    if (cvd_off) {
        return;
    }

    int type = CVD_RECT_TYPE_1_INT;
    char func_name[128] = "Rect<int>";

    if (std::is_same<_Tp, float>::value) {
        type = CVD_RECT_TYPE_1_FLOAT;
        strcpy (func_name, "Rect<float>");
    }
    if (std::is_same<_Tp, double>::value) {
        type = CVD_RECT_TYPE_1_DOUBLE;
        strcpy (func_name, "Rect<double>");
    }

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), type, "cvd::Rect",
                               PARAMETER | FUNC_OFF,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        if (type == CVD_RECT_TYPE_1_INT) {
            struct _rect_int_ ri = {(int)_x, (int)_y, (int)_width, (int)_height, -100000, 100000};
            foo->new_para (RECT_INT_PARA, sizeof(struct _rect_int_), (uint8_t*)&ri, func_name );
        }

        if ((type == CVD_RECT_TYPE_1_FLOAT) || (type == CVD_RECT_TYPE_1_DOUBLE)){
            struct _rect_double_ rd = {(double)_x, (double)_y, (double)_width, (double)_height, -100000.0, 100000.0};
            foo->new_para (RECT_DOUBLE_PARA, sizeof(struct _rect_double_), (uint8_t*)&rd, func_name );
        }
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // ----------------------------------------------------------------------------------------------
    // no break-function
    // ----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {
        // nothing to do
    } else {
        try {
            if ((type == CVD_RECT_TYPE_1_FLOAT) || (type == CVD_RECT_TYPE_1_DOUBLE)){
                struct _rect_double_ *s_dat = (struct _rect_double_ *)foo->para[0]->data;
                this->x = s_dat->x;
                this->y = s_dat->y;
                this->width = s_dat->w;
                this->height = s_dat->h;
            }
            if (type == CVD_RECT_TYPE_1_INT) {
                struct _rect_int_ *s_dat = (struct _rect_int_ *)foo->para[0]->data;
                this->x = s_dat->x;
                this->y = s_dat->y;
                this->width = s_dat->w;
                this->height = s_dat->h;
            }
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
}

//////////////////////////////// String_ ////////////////////////////////
//!
//! \brief The String class
//!
class CV_EXPORTS String : public cv::String
{
public:
    using cv::String::String;

    String(const char* s, BUILD_IN_PROTO );         // example: CVD::String str = "text";
    String(const String& str, BUILD_IN_PROTO );     // example: CVD::String basic_str = "basic_str"; CVD::String k(basic_str);
    String(const std::string& str, BUILD_IN_PROTO );

    String& operator=(const char* str);     // example: CVD::String str; str = "text";
    // String& operator+=(const char* str);    // example: CVD::String str = "test+"; str += "text";

    // String toLowerCase() const;

private:
    char *string_func (const char *s,
                              uint64_t addr, uint16_t type, const char *f_name,
                              int line_nr, const char *src_file);
};
// ----------------------------------------------------------------------------------------------------------
//!
//! \brief String::String
//! \param str
//!
String::String(const char* str, int line_nr, const char *src_file) : cv::String( str )
{
    if (cvd_off) {
        *this = str;
        return;
    }

    char *ret = NULL;
    ret = string_func (str,
                 (uint64_t)__builtin_return_address(0),
                 STRING_FUNC,
                 "String(char *)",
                 line_nr,
                 src_file);
    *this = ret;
}

//!
//! \brief String::String
//! \param str
//!
String::String(const String& str, int line_nr, const char *src_file ) : cv::String (str)
{
    if (cvd_off) {
        *this = str.c_str();
        return;
    }
    char *ret = NULL;
    ret = string_func (str.c_str(),
                 (uint64_t)__builtin_return_address(0),
                 STRING_FUNC,
                 "String(String&)",
                 line_nr,
                 src_file);

    *this = ret;
}

//!
//! \brief String::String
//! \param str
//!
String::String(const std::string& str, int line_nr, const char *src_file ) : cv::String(str)
{
    if (cvd_off) {
        *this = str.c_str();
        return;
    }

    char *ret = NULL;
    ret = string_func (str.c_str(),
                 (uint64_t)__builtin_return_address(0),
                 STRING_FUNC,
                 "String(std::string&)",
                 line_nr,
                 src_file);

    *this = ret;
}

//!
//! \brief String::operator =
//! \param str
//! \return
//!
String& String::operator =(const char *str)
{
    cv::String *s = this;   // s is pointer to this
    *s = str;
    return *this;
}

/*
String& String::operator+=(const char* str)
{
    // printf ("String::operator+=\n");
    cv::String *s = this;
    *s += str;
    return *this;
}



String String::toLowerCase() const
{
    cv::String *s = static_cast<cv::String*>(this);
    if (!s->cstr_)
        return String();
    // printf ("String String::toLowerCase() const\n");
    // printf ("this = %s\n", this->c_str());

    s = s.toLowerCase();
    printf ("s = %s\n", s.c_str());
    // *this = s.c_str();
    // return static_cast<String>(s.toLowerCase());
    return *this;
}
*/

//!
//! \brief String::string_func
//! \param s Basic String
//! \param addr
//! \param type see: enum _data_types_
//! \param f_name
//! \return
//!
char *String::string_func (const char *s,
                                  uint64_t addr, uint16_t type, const char *f_name,
                                  int line_nr, const char *src_file)
{
    assert (strlen(s) < MAX_STRING_VAL_LEN-1);  // length is limited

    static char str[MAX_STRING_VAL_LEN] = "";
    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, addr)) == NULL) {
        foo = new opencvd_func(addr, type, f_name,
                               PARAMETER | FUNC_OFF,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        struct _string_para_ al;
        strcpy (al.val, s);
        foo->new_para ( STRING_PARA, sizeof(struct _string_para_), (uint8_t*)&al, "cvd::String" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    // ----------------------------------------------------------------------------------------------
    // no break-function
    // ----------------------------------------------------------------------------------------------
    if (foo->state.flag.func_off) {     // imread ist ausgeschaltet.
        strcpy (str, s);
    } else {
        try {
            assert (strlen((char *)foo->para[0]->data) < MAX_STRING_VAL_LEN-1);
            struct _string_para_ *al = (struct _string_para_ *)foo->para[0]->data;
            strcpy (str, al->val);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    return str;
} // string_func

} // namespace cvd

#endif // OPENCV_CVDSTD_HPP
