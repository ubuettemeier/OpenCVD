#ifndef OPENCVD_HPP
#define OPENCVD_HPP

#ifdef USE_CVD
    #define CVD cvd
    #define KILL_CVD delete_cvd();
#else
    #define CVD cv
    #define KILL_CVD
#endif

#include "opencv2/opencv.hpp"

#include "opencvd_types.hpp"
#include "opencvd_basic.hpp"
#include "opencvd_func.hpp"
#include "opencvd_mat.hpp"
#include "opencvd_cvdstd.hpp"

#endif // OPENCVD_HPP
