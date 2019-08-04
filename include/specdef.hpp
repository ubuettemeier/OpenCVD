//!
//! \brief Special function for working with Opencv
//! \author Ulrich Büttemeier
//!

#include "opencvd.hpp"
#include <type_traits>

// ---------------------- Achtung: #define löschen -------------------------------------------------
#define USE_CVD

#ifdef USE_CVD
    //!  @brief enumlist_name are def in "enum.xml"
    int get_enumval (const char *enumlist_name, int val, const char *val_name = "",
                     int line_nr  = __builtin_LINE(),
                     const char *src_file = __builtin_FILE());

    template<typename T>
    T get_numval (T a, const char *val_name = "",
                  int line_nr = __builtin_LINE(),
                  const char *src_file = __builtin_FILE());

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
    int get_enumval (const char *enumlist_name, int val, const char *val_name = "");

    template<typename T>
    T get_numval (T a, const char *val_name = "");

    template<typename T>
    T set_trackbar (T a, const char *val_name = "",
                    T min=0, T max=255, T div=1 );

    template<typename T>
    void get_filename (T *fname, const char *val_name = "noname");

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
void get_filename (T *fname, const char *val_name) {
    std::ignore = fname;
    std::ignore = val_name;
}
#endif


#ifdef USE_CVD
///////////////////////////////////////// set_trackbar ///////////////////////////////////////////////
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
T set_trackbar (T a, const char *val_name,
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
//! \example int a = get_numval<int>(34, "value a");
//!
template<typename T>
T get_numval (T a, const char *val_name,
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
            struct _int_para_ ro = {static_cast<int>(a), std::numeric_limits<int>::min(), std::numeric_limits<int>::max()};
            foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&ro, "val<int>");
        }
        if (std::is_same<T, unsigned int>::value) {                                                             // unsigned int
            struct _int_para_ uni = {static_cast<int>(a), 0, std::numeric_limits<int>::max()};
            foo->new_para (INT_PARA, sizeof(struct _int_para_), (uint8_t*)&uni, "val<unsigned int>");
        }

        if (std::is_same<T, double>::value |                                                                    // double
            std::is_same<T, float>::value) {                                                                    // float
            struct _double_para_ sc = {static_cast<double>(a), std::numeric_limits<double>::min(), std::numeric_limits<double>::max(), 4};
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
T get_numval (T a, const char *val_name) {
    std::ignore = val_name;
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
//! \example int match_method = get_enumval("TemplateMatchMode", 3, "match_methode" );
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
