//!
//! \author Ulrich Buettemeier
//! \todo CVD::Mat zero = CVD::Mat::zeros(250, 300, CV_8U);  funktioniert nicht !!!
//! \date 07-06-2019
//!

#ifndef OPENCV_CVDSTD_HPP
#define OPENCV_CVDSTD_HPP

#include "opencv2/opencv.hpp"
#include <memory>

#define BUILD_IN_PROTO  int line_nr = __builtin_LINE(), \
                        const char *src_file = __builtin_FILE()

namespace cvd {

class CV_EXPORTS String : public cv::String
{
public:
    using cv::String::String;

    String(const char* s, BUILD_IN_PROTO );         // example: CVD::String str = "text";
    String(const String& str, BUILD_IN_PROTO );     // example: CVD::String basic_str = "basic_str"; CVD::String k(basic_str);
    String(const std::string& str, BUILD_IN_PROTO );

    String& operator=(const char* str);     // example: CVD::String str; str = "text";
    String& operator+=(const char* str);    // example: CVD::String str = "test+"; str += "text";

    String toLowerCase() const;

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
String::String(const char* str, int line_nr, const char *src_file)
{
    // printf ("String::String(const char* str)\n");
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
//! \brief String::operator +=
//! \param str
//! \return
//!
String& String::operator+=(const char* str)
{
    // printf ("String::operator+=\n");
    cv::String *s = this;
    *s += str;
    return *this;
}

//!
//! \brief String::operator =
//! \param str
//! \return
//!
String& String::operator =(const char *str)
{
    // printf ("String::operator =\n");
    cv::String *s = this;   // s is pointer to this
    *s = str;
    return *this;
}

//!
//! \brief String::toLowerCase
//! \return
//!
String String::toLowerCase() const
{
    // printf ("String String::toLowerCase() const\n");
    cv::String s = *this;
    return static_cast<String>(s.toLowerCase());
}

//!
//! \brief String::string_func
//! \param s
//! \param addr
//! \param type
//! \param f_name
//! \param line_nr
//! \param src_file
//! \return
//!
char *String::string_func (const char *s,
                                  uint64_t addr, uint16_t type, const char *f_name,
                                  int line_nr, const char *src_file)
{
    static char str[512] = "";
    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, addr)) == NULL) {
        foo = new opencvd_func(addr, type, f_name,
                               PARAMETER,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        struct _string_para_ al;
        strcpy (al.val, s);
        foo->new_para ( STRING_PARA, sizeof(struct _string_para_), (uint8_t*)&al, "cvd::String" );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    if (foo->state.flag.func_off) {     // imread ist ausgeschaltet.
        // return ret;                     // return ist empty !!!
    } else {
        try {
            struct _string_para_ *al = (struct _string_para_ *)foo->para[0]->data;
            strcpy (str, al->val);
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }
    return str;
}

} // namespace cvd

#endif // OPENCV_CVDSTD_HPP
