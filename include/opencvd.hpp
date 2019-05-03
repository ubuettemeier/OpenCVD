#ifndef OPENCVD_HPP
#define OPENCVD_HPP

#ifdef USE_CVD
    #define CVD cvd
    #define KILL_CVD delete_cvd();
#else
    #define CVD cv
    #define KILL_CVD
#endif

#include <opencv.hpp>

#include <opencvd_types.hpp>
#include <opencvd_basic.hpp>
#include <opencvd_func.hpp>

#endif // OPENCVD_HPP
