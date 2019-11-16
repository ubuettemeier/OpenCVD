//!
//! \brief Special function for working with Opencv
//!        the following functions are partially implemented:
//!        - int get_enumval
//!        - template<typename T> get_numval
//!        - template<typename T> set_trackbar
//!        - void get_filename
//!        - void set_cam_param
//!
//! these functions do not belong to any namespace
//!
//! \author Ulrich Büttemeier
//!

#ifndef SPECDEF_HPP
#define SPECDEF_HPP

#include "opencvd.hpp"
#include <type_traits>

// ---------------------- Achtung: #define löschen -------------------------------------------------
// #define USE_CVD

#ifdef USE_CVD
    void set_cam_para (cv::VideoCapture &cap,
                        int line_nr  = __builtin_LINE(),
                        const char *src_file = __builtin_FILE());

    //! \brief enumlist_name are defined in "enum.xml"
    int get_enumval (const char *enumlist_name, int val, const char *val_name = "",
                     int line_nr  = __builtin_LINE(),
                     const char *src_file = __builtin_FILE());

    template<typename T>
    T get_numval (T a, const char *val_name = "",
                  T min=std::numeric_limits<T>::min(),
                  T max=std::numeric_limits<T>::max(),  // <unsigned int>::max() = <int>::max()
                  int line_nr = __builtin_LINE(),
                  const char *src_file = __builtin_FILE());

    //! \example int d = set_trackbar <int> (12, "resolution", 4, 20, 1);
    template<typename T>
    T set_trackbar (T a, const char *val_name = "noname",
                    T min=0, T max=255, T div=1,
                    int line_nr = __builtin_LINE(),
                    const char *src_file = __builtin_FILE());

    template<typename T>
    void get_filename (T *fname, const char *val_name = "noname",
                       int line_nr = __builtin_LINE(),
                       const char *src_file = __builtin_FILE());
#else
    void set_cam_para (cv::VideoCapture &cap);

    int get_enumval (const char *enumlist_name, int val, const char *val_name = "");

    template<typename T>
    T get_numval (T a, const char *val_name = "", T min=0, T max=0);

    template<typename T>
    T set_trackbar (T a, const char *val_name = "",
                    T min=0, T max=255, T div=1 );

    template<typename T>
    void get_filename (T *fname, const char *val_name = "noname");

#endif

#ifdef USE_CVD
//////////////////////////////////////////////////////////////////////////////////////////////////////
//! \brief set_cam_param
//! \param cap
//!
void set_cam_para (cv::VideoCapture &cap,
                    int line_nr,
                    const char *src_file)
{
    if (!cap.isOpened() || cvd_off)
        return;

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        foo = new opencvd_func((uint64_t)__builtin_return_address(0), SET_CAM_PARA, "set_cam_para",
                               PARAMETER | FUNC_OFF | SHOW_IMAGE,    // Menu
                               line_nr, src_file);
        func.push_back( foo );

        double buf = cap.get(cv::CAP_PROP_BRIGHTNESS);
        struct _slide_double_para_ bri = {buf, 0.0, 1.0, 100.0, buf};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&bri, "BRIGHTNESS" );

        buf=cap.get(cv::CAP_PROP_CONTRAST);
        struct _slide_double_para_ con = {buf, 0.0, 1.0, 100.0, buf};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&con, "CONTRAST" );

        buf = cap.get(cv::CAP_PROP_SATURATION);
        struct _slide_double_para_ sat = {buf, 0.0, 1.0, 100.0, buf};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&sat, "SATURATION" );

        buf = cap.get(cv::CAP_PROP_HUE);
        struct _slide_double_para_ hu = {buf, 0.0, 1.0, 100.0, buf};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&hu, "HUE" );

        buf = cap.get(cv::CAP_PROP_GAIN);
        struct _slide_double_para_ ga = {buf, 0.0, 1.0, 100.0, buf};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&ga, "GAIN" );

        /* funktioniert nicht !!
        buf = cap.get(cv::CAP_PROP_AUTO_EXPOSURE);
        struct _slide_double_para_ ae= {buf, 0.0, 1.0, 100.0, buf};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&ae, "AUTO EXPOSURE" );
        */
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    if (foo->state.flag.func_off) {
        // nothing to do
    } else {
        try {
            int cap_para[5] = {
                cv::CAP_PROP_BRIGHTNESS,
                cv::CAP_PROP_CONTRAST,
                cv::CAP_PROP_SATURATION,
                cv::CAP_PROP_HUE,
                cv::CAP_PROP_GAIN
                // cv::CAP_PROP_AUTO_EXPOSURE
            };
            int i;
            for (i=0; i<5; i++) {
                struct _slide_double_para_ *buf = (struct _slide_double_para_ *)foo->para[i]->data;
                if (buf->last_value != buf->value) {
                    cap.set(cap_para[i], *(double*)foo->para[i]->data);     // CAP_PROP_BRIGHTNESS
                    buf->last_value = buf->value;
                }
            }
        } catch( cv::Exception& e ) {
            foo->error_flag |= FUNC_ERROR;
        }
        foo->control_func_run_time ();
    }

    cv::Mat dst;
    if (foo->state.flag.show_image)
        cap >> dst;

    foo->control_imshow( dst );  // show Image
}
#else
void set_cam_para (cv::VideoCapture &cap)
{
    std::ignore = cap;
}
#endif

#ifdef USE_CVD
///////////////////////////////////////// get_filename ///////////////////////////////////////////////
//!
//! \brief get_filename
//! \param fname types: std:string; cv::String
//! \param val_name
//!
template<typename T>
void get_filename (T *fname, const char *val_name,
                   int line_nr,
                   const char *src_file)
{
    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        assert ((std::is_same<T, cv::String>::value |
                 std::is_same<T, std::string>::value));

        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GET_FILENAME, val_name,
                               PARAMETER,    // Menu 0x000F
                               line_nr, src_file);
        func.push_back( foo );


        assert (fname->length() < MAX_STRING_VAL_LEN-1);
        struct _string_para_ al;
        strcpy (al.val, fname->c_str());
        foo->new_para (STRING_PARA, sizeof(struct _string_para_), (uint8_t*)&al, "filename", 1);    // extra_para = 1; see also imread()
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    foo->control_func_run_time ();      // triggert alle 400ms den server

    struct _string_para_ *al = (struct _string_para_ *)foo->para[0]->data;
    *fname = al->val;
}
#else
template<typename T>
void get_filename (T *fname, const char *val_name) {    // do nothing
    std::ignore = fname;
    std::ignore = val_name;
}
#endif


#ifdef USE_CVD
///////////////////////////////////////// set_trackbar ///////////////////////////////////////////////
//! \example int d = set_trackbar<int>(12, "resolution", 4, 20, 1);
template<typename T>
T set_trackbar (T a, const char *val_name,
                T min, T max, T div,
                int line_nr,
                const char *src_file)
{
#ifndef USE_CVD
    return a;
#endif

    if (!(std::is_same<T, int>::value |
          std::is_same<T, unsigned int>::value |
          std::is_same<T, double>::value |
          std::is_same<T, float>::value)) {
        printf ("unbekannter type\n");
        return a;
    }

    T ret;

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        std::string str = "set_trackbar(";
        str += val_name;
        str += ")";

        foo = new opencvd_func((uint64_t)__builtin_return_address(0), SET_TRACKBAR, str.c_str(),
                               PARAMETER,    // Menu 0x000F
                               line_nr, src_file);
        func.push_back( foo );

        double val = static_cast<double>(a);
        double _min = static_cast<double>(min);
        double _max = static_cast<double>(max);
        double _div = static_cast<double>(div);

        struct _slide_double_para_ dp = {val, _min, _max, _div};
        foo->new_para ( SLIDE_DOUBLE_PARA, sizeof(struct _slide_double_para_), (uint8_t*)&dp, val_name );
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error
    foo->control_func_run_time ();      // triggert alle 400ms den server

    ret = static_cast<T>(*(double*)foo->para[0]->data);

    return ret;
}
#else

template<typename T>
T set_trackbar (T a, const char *val_name,      // do nothing
                T min, T max, T div) {
    std::ignore = min;
    std::ignore = max;
    std::ignore = div;
    std::ignore = val_name;

    return a;
}
#endif

#ifdef USE_CVD
///////////////////////////////////////// get_numval ///////////////////////////////////////////////
//! \brief get_numval offers the possibility to manipulate a numeric value with the OpenCVD server.
//! \param a from type <int, double, float>
//! \return return nummeric value
//! \example int a = get_numval<int>(34, "value a", 0, 1000);
//!
template<typename T>
T get_numval (T a, const char *val_name,
              T min, T max,
              int line_nr,
              const char *src_file)
{
    #ifndef USE_CVD
        return a;
    #endif

    if (!(std::is_same<T, int>::value |
          std::is_same<T, unsigned int>::value |
          std::is_same<T, double>::value |
          std::is_same<T, float>::value |
          std::is_same<T, bool>::value)) {
        printf ("unbekannter type\n");
        return a;
    }

    T ret;

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        std::string str = "get_numval(";
        str += val_name;
        str += ")";

        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GET_NUMVAL, str.c_str(),
                               PARAMETER,    // Menu 0x000F
                               line_nr, src_file);
        func.push_back( foo );

        if (std::is_same<T, bool>::value) {                                                                     // bool
            struct _enum_para_ un = {static_cast<bool>(a), "boolType"};
            foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&un, "val<bool>" );
        }

        if (std::is_same<T, int>::value) {                                                                      // int
            struct _int_para_ ro = {static_cast<int>(a), static_cast<int>(min), static_cast<int>(max)};
            foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ro, "val<int>");
        }
        if (std::is_same<T, unsigned int>::value) {                                                             // unsigned int. Kann mit SpinBox nicht korrekt abgebildet werden !
            if (max > std::numeric_limits<int>::max())
                        max = std::numeric_limits<int>::max();              // <unsigned int>::max() = <int>::max()

            struct _int_para_ uni = {static_cast<int>(a), static_cast<int>(min), static_cast<int>(max)};
            foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&uni, "val<unsigned int>");
        }

        if (std::is_same<T, double>::value |                                                                    // double
            std::is_same<T, float>::value) {                                                                    // float
            struct _double_para_ sc = {static_cast<double>(a), static_cast<double>(min), static_cast<double>(max), 4};
            foo->new_para (DOUBLE_PARA, sizeof(struct _double_para_), (uint8_t*)&sc, "val<double / float>");
        }
    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    foo->control_func_run_time ();      // triggert alle 400ms den server

    if (foo->para[0]->para_type == INT_PARA)
        ret = *(int*)foo->para[0]->data;
    if (foo->para[0]->para_type == ENUM_DROP_DOWN)
        ret = *(bool*)foo->para[0]->data;
    if (foo->para[0]->para_type == DOUBLE_PARA)
        ret = *(double*)foo->para[0]->data;

    return ret;
} // T set_val()

#else
template<typename T>
T get_numval (T a, const char *val_name,
              T min, T max) {      // do nothing
    std::ignore = val_name;
    std::ignore = min;
    std::ignore = max;
    return a;
}
#endif

#ifdef USE_CVD
//!
//! \brief get_enumval
//! \param enumlist_name are define in "enum.xml"
//! \param val
//! \param val_name
//! \return
//! \example int match_method = get_enumval( "TemplateMatchMode", 3, "match_methode" );
//!          bool show_color = get_enumval( "boolType", false, "show_color" );
//!
int get_enumval (const char *enumlist_name, int val, const char *val_name,
                 int line_nr,
                 const char *src_file)
{
    #ifndef USE_CVD
        return val;
    #endif

    static std::vector<opencvd_func *> func{};  // reg vector
    opencvd_func *foo = NULL;

    if ((foo = opencvd_func::grep_func(func, (uint64_t)__builtin_return_address(0))) == NULL) {
        std::string str = "get_enumval(";
        str += val_name;
        str += ")";

        foo = new opencvd_func((uint64_t)__builtin_return_address(0), GET_ENUMVAL, str.c_str(),
                               PARAMETER,               // Menu
                               line_nr, src_file);

        func.push_back( foo );

        struct _enum_para_ mt;
        mt.value = val;
        strcpy (mt.enum_name, enumlist_name);
        foo->new_para ( ENUM_DROP_DOWN, sizeof(struct _enum_para_), (uint8_t*)&mt, val_name );

    }
    foo->error_flag &= ~FUNC_ERROR;     // clear func_error

    foo->control_func_run_time ();      // triggert alle 400ms den server

    return *(int*)foo->para[0]->data;
}
#else

int get_enumval (const char *enumlist_name, int val, const char *val_name)
{
    std::ignore = enumlist_name;
    enumlist_name = val_name;

    return val;
}


#endif // #ifdef USE_CVD

#endif // SPECDEF_HPP
