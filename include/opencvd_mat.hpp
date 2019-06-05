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
    void convertTo( cv::OutputArray m, int rtype, double alpha=1, double beta=0 ) const;

};

//!
//! \brief Mat::convertTo
//! \param m
//! \param rtype
//! \param alpha
//! \param beta
//!
void Mat::convertTo( cv::OutputArray m, int rtype, double alpha, double beta ) const
{
    printf ("treffer\n");
    this->cv::Mat::convertTo(m, rtype, alpha, beta);
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
