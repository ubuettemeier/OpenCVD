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
    if (foo->state.flag.func_off) {     // imread ist ausgeschaltet.
        // strcpy (str, s);
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
