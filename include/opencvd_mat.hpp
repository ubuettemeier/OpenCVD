#ifndef OPENCV_MAT_HPP
#define OPENCV_MAT_HPP

#include "opencv2/opencv.hpp"

namespace cvd {


//!
//! \brief The Mat class
//!
class Mat : public cv::Mat
{
public:
    Mat() : cv::Mat() {}

    Mat(const cv::Mat& m, const cv::Rect& roi, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE());
    void convertTo( cv::OutputArray m, int rtype, double alpha=1, double beta=0, int line_nr = __builtin_LINE(), const char *src_file = __builtin_FILE() ) const;

};

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

} // namespace cvd

#endif // OPENCV_MAT_HPP
