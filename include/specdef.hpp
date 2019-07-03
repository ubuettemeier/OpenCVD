//!
//! \brief Special function for working with Opencv
//! \author Ulrich Büttemeier
//!

#include "opencvd.hpp"
#include <type_traits>

#ifdef USE_CVD
    template<typename T>
    T set_numval (T a, const char *val_name = "",
                  int line_nr = __builtin_LINE(),
                  const char *src_file = __builtin_FILE());
#else
    template<typename T>
    T set_numval (T a, const char *val_name = "");
#endif

//! \brief set_numval offers the possibility to manipulate a numeric value with the OpenCVD server.
//! \param a from type <int, double, float>
//! \return return nummeric value
//!
#ifdef USE_CVD

template<typename T>
T set_numval (T a, const char *val_name,
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
        std::string str = "set_numval(";
        str += val_name;
        str += ")";

        foo = new opencvd_func((uint64_t)__builtin_return_address(0), SET_NUMVAL, str.c_str(),
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

    foo->control_func_run_time ();

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
T set_numval (T a, const char *val_name) {
    std::ignore = val_name;
    return a;
}

#endif // #ifdef USE_CVD
