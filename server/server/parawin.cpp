//!
//! \file   parawin.cpp
//! \author Ulrich Buettemeier
//!

#include <iostream>
#include <unistd.h>
#include <QKeyEvent>
#include <QFileDialog>
#include <QTreeWidgetItem>

#include "mainwindow.h"
#include "parawin.h"

ParaWin *parawin = nullptr;

//!
//! \brief ParaWin::ParaWin
//! \param main_win
//! \param parent
//!
ParaWin::ParaWin(MainWindow *main_win, QWidget *parent) : QWidget(parent)
{
    mw = main_win;
}

//!
//! \brief ParaWin::ParaWin
//! \param c
//! \param foo
//! \param main_win
//! \param parent
//!

#define LEFT_POS 15

ParaWin::ParaWin(QTcpSocket *c, struct _cvd_func_ *foo, MainWindow *main_win, QWidget *parent)
{
    Q_UNUSED (parent);

    this->setGeometry(380, 180, 400, 300);
    client = c;
    cf = foo;
    this->setWindowTitle(cf->func_name);
    mw = main_win;

    switch (cf->type) {
    case SET_CAM_PARA:
        new Slide (client, cf->first_para, LEFT_POS, 10+55*0, this );           // CAP_PROP_BRIGHTNESS
        new Slide (client, cf->first_para->next, LEFT_POS, 10+55*1, this );           // CAP_PROP_CONTRAST
        new Slide (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );           // CAP_PROP_SATURATION
        new Slide (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );           // CAP_PROP_HUE
        new Slide (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );           // CAP_PROP_GAIN

        // new DoubleEdit (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );     // WIDTH
        // new DoubleEdit (client, cf->first_para->next->next->next->next->next->next, LEFT_POS, 10+55*6, this );     // HEIGHT

        // new Slide (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );           // CAP_PROP_AUTO_EXPOSURE 0,25 < > 0,75 Funktioniert nicht !
        set_param_win( 5, 260 );
        break;
    case NORMALIZE_2:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );      // alpha
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );  // normtype
        set_param_win( 2, 260 );
        break;
    case CALCOPTICALFLOWFARNEBACK:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );      // pyr_scale
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );     // levels
        new IntEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );     // winsize
        new IntEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );     // iterations
        new IntEdit (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );     // poly_n
        new DoubleEdit (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );      // poly_sigma
        new IntEdit (client, cf->first_para->next->next->next->next->next->next, LEFT_POS, 10+55*6, this );     // flags
        set_param_win( 7, 260 );
        break;
    case GETDERIVKERNELS:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );     // dx
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );     // dy
        new Slide (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );           // ksize
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );        // normalize
        new EnumDrop (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );        // ktype
        set_param_win( 5, 260 );
        break;
    case GETGABORKERNEL:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );   // ksize
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );        // sigma
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );        // theta
        new DoubleEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );        // lambd
        new DoubleEdit (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );        // gamma
        new DoubleEdit (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );        // psi
        new EnumDrop (client, cf->first_para->next->next->next->next->next->next, LEFT_POS, 10+55*6, this );        // ktype
        set_param_win( 7, 260 );
        break;
    case GETGAUSSIANKERNEL:
        new Slide (client, cf->first_para, LEFT_POS, 10+55*0, this );                   // ksize
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );        // sigma
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );    // ktype
        set_param_win( 3, 260 );
        break;
    case FILTER2D:
    case SEQFILTER2D:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );                // ddepth
        new PointInt ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );         // annchor
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );  // delta
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );  // borderType
        set_param_win( 4, 260 );
        break;
    case PUTTEXT:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );           // Point
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );      // fontFace
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );                  // fontScale
        new ScalarDouble ( client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );         // color
        new IntEdit (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );         // thickness
        new EnumDrop (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );  // lineType
        new EnumDrop (client, cf->first_para->next->next->next->next->next->next, LEFT_POS, 10+55*6, this );    // bottomLeftOrigin
        set_param_win( 7, 360 );
        break;
    case SQRBOXFILTER:
    case BOXFILTER:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );
        new PointInt ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );   // ksize
        new PointInt ( client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );   // annchor
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );
        new EnumDrop (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );
        set_param_win( 5, 260 );
        break;
    case MATCHSHAPES:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );    //
        set_param_win( 1, 260 );
        break;
    case MATCHTEMPLATE:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );    //
        set_param_win( 1, 260 );
        break;
    case DISTANCETRANSFORM:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );    //
        // new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );                 //
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );           //
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );    //
        set_param_win( 3, 260 );
        break;
    case PYRMEANSHIFTFILTERING:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );              //
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );        //
        new IntEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );     //
        set_param_win( 3, 260 );
        break;
    case BILATERALFILTER:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );     // _n_bins
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );  // _scale
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );  // _scale
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );  // _refine
        set_param_win( 4, 260 );
        break;
    case CREATELINESEGMENTDETECTOR:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );  // _refine
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );  // _scale
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );  // _sigma_scale
        new DoubleEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );  // _quant
        new DoubleEdit (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );  // _ang_th
        new DoubleEdit (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );  // _log_eps
        new DoubleEdit (client, cf->first_para->next->next->next->next->next->next, LEFT_POS, 10+55*6, this );  // _density_th
        new IntEdit (client, cf->first_para->next->next->next->next->next->next->next, LEFT_POS, 10+55*7, this );     // _n_bins
        set_param_win( 8, 260 );
        break;
    case CVD_POINT_TYPE_1_FLOAT:
    case CVD_POINT_TYPE_1_DOUBLE:
        new PointDouble ( client, cf->first_para, LEFT_POS, 10+55*0, this );       // x, y
        set_param_win( 1, 260 );
        break;
    case CVD_POINT_TYPE_1_INT64:
    case CVD_POINT_TYPE_1_INT:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );       // x, y
        set_param_win( 1, 260 );
        break;
    case CVD_SCALAR_2:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );          // Scalar
        set_param_win( 1, 260 );
        break;
    case CVD_SCALAR_1:
        new RectDoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );      // Scalar
        set_param_win( 1, 360 );
        break;
    case RECTANGLE_2:
    case RECTANGLE_1:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );     // thickness
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );  // lineType
        new IntEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );     // shift
        set_param_win( 3, 260 );
        break;
    case CVD_RECT_TYPE_1_DOUBLE:
    case CVD_RECT_TYPE_1_FLOAT:
        new RectDoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );       // Rect
        set_param_win( 1, 360 );
        break;
    case CVD_RECT_TYPE_1_INT:
        new RectIntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );          // Rect
        set_param_win( 1, 320 );
        break;
    case STRING_FUNC:
        new StringEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );
        set_param_win( 1, 320 );
        break;
    case GET_FILENAME:
        new FileNameEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );
        set_param_win( 1, 320 );
        break;
    case SET_TRACKBAR:
        new Slide ( client, cf->first_para, LEFT_POS, 10+55*0, this );         // Slider
        set_param_win( 1, 260 );
        break;
    case GET_NUMVAL:
        if (cf->first_para->type == INT_PARA)
            new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );     // val
        if (cf->first_para->type == ENUM_DROP_DOWN)
            new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );    // boolType
        if (cf->first_para->type == DOUBLE_PARA)
            new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );  // val
        set_param_win( 1, 260 );
        break;
    case GET_ENUMVAL:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );
        set_param_win( 1, 260 );
        break;
    case BUILDPYRAMID:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );     // maslevel
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );  // BorderType
        set_param_win( 2, 260 );
        break;
    case MAT_ZEROS_3:
    case MAT_ONES_3:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );
        new IntEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );
        set_param_win( 4, 260 );
        break;
    case MAT_EYE_2:
    case MAT_ZEROS_2:
    case MAT_ONES_2:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );               // size
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );          // type
        set_param_win( 2, 260 );
        break;
    case MAT_EYE:
    case MAT_ZEROS:
    case MAT_ONES:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );
        set_param_win( 3, 260 );
        break;
    case SCALEADD:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );              // alpha
        set_param_win( 1, 260 );
        break;
    case MAT_ASSIGNTO:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );                 // type
        set_param_win( 1, 260 );
        break;
    case APPROXPOLYPD:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );              // epsilon
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );          // closed
        set_param_win( 2, 260 );
        break;
    case FITLINE:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );                        // distType
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );                // param
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );          // reps
        new DoubleEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );    // aeps
        set_param_win( 4, 260 );
        break;
    case CORNERMINEIGENVAL:
    case COREREIGENVALANDVECS:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );                     // blockSize
        new Slide (client, cf->first_para->next, LEFT_POS, 10+55*1, this );                 // ksize 3, 5, ...
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );        // borderType
        set_param_win( 3, 260 );
        break;
    case PRECORNERDETECT:
        new Slide (client, cf->first_para, LEFT_POS, 10+55*0, this );                       // ksize 3, 5, ...
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );              // borderType
        set_param_win( 2, 260 );
        break;
    case CORNERSUBPIX:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );                   // winSize
        new PointInt ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );             // zeroZone
        set_param_win( 2, 260 );
        break;
    case CORNERHARRIS:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );                     // blockSize
        new Slide (client, cf->first_para->next, LEFT_POS, 10+55*1, this );                 // ksize 3, 5, ...
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );      // k
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );  // borderType
        set_param_win( 4, 260 );
        break;
    case PYRUP:
    case PYRDOWN:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );                   // size
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );              // borderType
        set_param_win( 2, 260 );
        break;
    case MAT_SIZE_TYPE:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );                   // size
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );              // type
        set_param_win( 2, 260 );
        break;
    case MAT_ROWS_COLS_TYPE:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );                     // rows
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );               // cols        
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );        // type
        set_param_win( 3, 260 );
        break;
    case MAT_SIZE_TYPE_SCALAR:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );                   // size
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );              // type
        new RectDoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );  // Scalar
        set_param_win( 3, 340 );
        break;
    case MAT_ROWS_COLS_TYPE_SCALAR:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );                     // rows
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );               // cols
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );        // type
        new RectDoubleEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );        // Scalar
        set_param_win( 4, 340 );
        break;
    case MAT_CONVERTTO:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );                    // rtype
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );            // alpha
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );      // alpha
        set_param_win( 3, 260 );
        break;
    case MAT_ROI:
        new RectIntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );          // Rect
        set_param_win( 1, 320 );
        break;
    case SOBEL:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );          // ddepth  Sobel_filterdepth
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );                 // dx
        new IntEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );                 // dy
        new Slide (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );                 // ksize
        new DoubleEdit (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );            // scale
        new DoubleEdit (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );            // delta
        new EnumDrop (client, cf->first_para->next->next->next->next->next->next, LEFT_POS, 10+55*6, this );          // BorderType

        set_param_win( 7, 260 );
        break;
    case RESIZE:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );                       // dsize
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );                // fx
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );          // fy
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );          // DropDown maxValue

        set_param_win( 4, 260 );
        break;
    case ADAPTIVETHRESHOLD:
        new Slide (client, cf->first_para, LEFT_POS, 10+55*0, this );                   // threshold
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );          // DropDown maxValue
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );    // DropDown adaptiveMethod
        new Slide ( client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );          // blockSize
        new DoubleEdit (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );      // C

        set_param_win( 5, 260 );
        break;
    case HOUGHLINES:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );            // rho
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );      // theta
        new Slide (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );      // threshold
        new DoubleEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );      // srn
        new DoubleEdit (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );      // stn
        new Slide (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );      // min_theta
        new Slide (client, cf->first_para->next->next->next->next->next->next, LEFT_POS, 10+55*6, this );      // max_theta
        /*
        new DoubleEdit (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );      // min_theta
        new DoubleEdit (client, cf->first_para->next->next->next->next->next->next, LEFT_POS, 10+55*6, this );      // max_theta
        */

        set_param_win( 7, 260 );
        break;
    case HOUGHLINESP:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );            // rho
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );      // theta
        new Slide (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );      // threshold
        new DoubleEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );            // minLineLength
        new DoubleEdit (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );      // maxLineGap

        set_param_win( 5, 260 );
        break;
    case HOUGHCIRCLES:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );                    // method
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );            // dp
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );      // minDist
        new Slide (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );      // param1
        new Slide (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );      // param2

        new IntEdit (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );                 // minRadius
        new IntEdit (client, cf->first_para->next->next->next->next->next->next, LEFT_POS, 10+55*6, this );                 // maxRadius

        set_param_win( 7, 260 );
        break;
    case CALCHIST:
        new IntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );                       // nimages
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );                 // dims
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );          // uniform
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );    // accumulate

        set_param_win( 4, 260 );
        break;
    case NORMALIZE:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );   // alpha
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );   // beta
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );   // norm_type
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );   // ddepth

        set_param_win( 4, 260 );
        break;
    case GETSTRUCTURINGELEMENT:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );   // shape = DropDown enum MorphShapes
        new PointInt ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );   // ksize
        new PointInt ( client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );   // annchor

        set_param_win( 3, 260 );
        break;
    case GRABCUT:
        new RectIntEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );     // Iteration
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );   // DropDown enum ImreadModes

        set_param_win( 3, 320 );
        break;
    case IMREAD:
        new FileNameEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );   // DropDown enum ImreadModes

        set_param_win( 2, 330 );
        break;
    case OPERATOR_INT_MUL_EQUAL:
        new IntEdit (client, cf->first_para, LEFT_POS, 10, this );   // Double Edit für scale
        setGeometry(glob_mw->para_win_pos.x(), glob_mw->para_win_pos.y(), 260, 80);
        break;
    case OPERATOR_FLOAT_MUL_EQUAL:
        new FloatEdit (client, cf->first_para, LEFT_POS, 10, this );   // Float Edit für scale. Achtung:
        setGeometry(glob_mw->para_win_pos.x(), glob_mw->para_win_pos.y(), 260, 80);
        break;
    case OPERATOR_DOUBLE_MUL_EQUAL:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10, this );   // Double Edit für scale
        setGeometry(glob_mw->para_win_pos.x(), glob_mw->para_win_pos.y(), 260, 80);
        break;
    case CONVERTTO:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10, this );   // DropDown enum ddepth
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 65, this );   // Double Edit für scale
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 65+55, this );   // Double Edit für delta
        setGeometry(glob_mw->para_win_pos.x(), glob_mw->para_win_pos.y(), 260, 80+55+55);
        break;
    case ADDWEIGHTED:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );              // Double Edit für alpha
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );        // Double Edit für beta
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );  // Double Edit für gamma

        set_param_win( 3, 260 );
        break;
    case CONVERTSCALEABS:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );            // Double Edit für alpha
        new DoubleEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );   // Double Edit für beta

        set_param_win( 2, 260 );
        break;
    case CANNY_2:
        new Slide ( client, cf->first_para, LEFT_POS, 10+55*0, this );                     // Slider für threshold1
        new Slide ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );            // Slider für threshold2
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );   // DropDown enum L2gradient (bool)

        set_param_win( 3, 260 );
        break;
    case CANNY:
        new Slide ( client, cf->first_para, LEFT_POS, 10+55*0, this );                    // Slider für threshold1
        new Slide ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );              // Slider für threshold2
        new Slide ( client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );        // Slider für apertureSize 3.. ???
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );   // DropDown enum L2gradient (bool)

        set_param_win( 4, 260 );
        break;
    case LAPLACIAN:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );   // DropDown enum
        new Slide ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );   // Slider für kzise
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );   // Double Edit für scale
        new DoubleEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );   // Double Edit für delta
        new EnumDrop (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );   // DropDown enum BorderType

        set_param_win( 5, 260 );
        break;
    case CVTCOLOR:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );   // DropDown enum
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );     // Iteration

        set_param_win( 2, 260 );
        break;
    case GAUSSIANBLUR:
        new Slide ( client, cf->first_para, LEFT_POS, 10+55*0, this );                     // Slider für ksize.width
        new Slide ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );            // Slider für ksize.height
        new DoubleEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );               // sigmaX
        new DoubleEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );      // sigmaY
        new EnumDrop (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );   // DropDown enum BorderType

        set_param_win( 5, 260 );
        break;
    case BLUR_FUNC:
        new Slide ( client, cf->first_para, LEFT_POS, 10+55*0, this );            // Slider für ksize.width
        new Slide ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );      // Slider für ksize.height
        new PointInt ( client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );      // Point
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );   // DropDown enum BorderType

        set_param_win( 4, 260 );
        break;
    case MEDIANBLUR:
        new Slide ( client, cf->first_para, LEFT_POS, 10+55*0, this );   // Slider für kzise

        set_param_win( 1, 260 );
        break;
    case THRESHOLD:
        // printf ("extra_para=%i\n", cf->first_para->extra_para);
        new Slide ( client, cf->first_para, LEFT_POS, 10+55*0, this );         // Slider für thresh
        new Slide ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );   // Slider für maxval
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );   // DropDown enum

        set_param_win( 3, 260 );
        break;
    case SCALAR_FUNC_4:
        new ScalarDouble ( client, cf->first_para, LEFT_POS, 10+55*0, this );  //
        setGeometry(glob_mw->para_win_pos.x(), glob_mw->para_win_pos.y(), 370, 10+55*1+20);
        break;
    case SCALAR_ALL:
        new DoubleEdit (client, cf->first_para, LEFT_POS, 10+55*0, this );    // v0
        setGeometry(glob_mw->para_win_pos.x(), glob_mw->para_win_pos.y(), 370, 10+55*1+20);
        break;
    case ERODE:
    case DILATE:
        new PointInt ( client, cf->first_para, LEFT_POS, 10+55*0, this );         // Point
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );     // Iteration
        new EnumDrop (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );   // DropDown enum

        set_param_win( 3, 260 );
        break;
    case MORPHOLOGYEX:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );   // op
        new PointInt ( client, cf->first_para->next, LEFT_POS, 10+55*1, this );   // annchor
        new IntEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );     // Iteration
        new EnumDrop (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );   // borderType

        set_param_win( 4, 260 );
        break;
    case FINDCONTOURS:
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );                  // DropDown enum RetrievalModes
        new EnumDrop (client, cf->first_para->next, LEFT_POS, 10+55*1, this );            // DropDown enum ContourApproximationModes
        new PointInt ( client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );     // Point offset

        set_param_win( 3, 260 );
        break;
    case SCHARR: {
        new EnumDrop (client, cf->first_para, LEFT_POS, 10+55*0, this );                  // DropDown enum filterdepth_CV_8U | filterdepth_CV_16U_CV_16S
        new IntEdit (client, cf->first_para->next, LEFT_POS, 10+55*1, this );             // dx
        new IntEdit (client, cf->first_para->next->next, LEFT_POS, 10+55*2, this );       // dy
        new DoubleEdit (client, cf->first_para->next->next->next, LEFT_POS, 10+55*3, this );            // scale
        new DoubleEdit (client, cf->first_para->next->next->next->next, LEFT_POS, 10+55*4, this );      // delta
        new EnumDrop (client, cf->first_para->next->next->next->next->next, LEFT_POS, 10+55*5, this );  // borderType

        set_param_win( 6, 260 );
        }
        break;
    default:
        printf ("unbekannte Funktion\n");
        break;
    }

    this->show();
}

//!
//! \brief ParaWin::set_param_win
//! \param y_pos
//!
void ParaWin::set_param_win(int y_pos, int width)
{
    new mButton (client, cf, LEFT_POS, 10+55*y_pos+10, mCLOSE, this, mw );                              // Close
    new mButton (client, cf, LEFT_POS+m_button[mCLOSE].width+10, 10+55*y_pos+10, mRESET, this, mw );    // Reset
    setGeometry(glob_mw->para_win_pos.x(), glob_mw->para_win_pos.y(), width, 10+55*(y_pos + 1));
}

//!
//! \brief ParaWin::~ParaWin
//!
ParaWin::~ParaWin()
{
    QRect r = this->geometry();
    glob_mw->para_win_pos = {r.x(), r.y()};
    parawin = nullptr;
}

//!
//! \brief ParaWin::close_parawin
//!        Funktion schließt Parameterfenster
//!
void ParaWin::close_parawin()
{
    if (parawin != nullptr) {
        if (mw != nullptr) {
            QTreeWidgetItem *tw = (QTreeWidgetItem *)cf->para_pointer;
            tw->setIcon(0, QIcon());
        }
        delete parawin;     // close Parameter Window
    }
    parawin = nullptr;
}

//!
//! \brief ParaWin::rewrite_para_data
//!        rewrite_para_data sendet Parameter-Daten an den Client
//! \param foo
//! \return
//!
int ParaWin::rewrite_para_data (struct _cvd_para_ *foo)
{
    int anz = 0;
    struct _para_data_transfer_ pd;

    pd.len = sizeof(struct _para_data_transfer_);
    pd.type = foo->type;
    pd.func_addr = foo->func_addr;
    pd.para_id = foo->para_id;
    strncpy (pd.para_name, foo->para_name, MAX_PARA_NAME_LEN);
    memcpy (pd.data, foo->data, MAX_PARA_DATA);
    // printf ("Sende bef=%4X\n", pd.type);
    mw->ack_detected = 0;
    anz = mw->write_data((const char *)&pd, sizeof(struct _para_data_transfer_));

    if (parawin != nullptr) {
        parawin->cf->func_is_modifyt = 0;
        struct _cvd_para_ *p = parawin->cf->first_para;
        while (p != nullptr) {                                 // Alle Parameter untersuchen.
            if (memcmp(p->data, p->reset_data, MAX_PARA_DATA) != 0)
                parawin->cf->func_is_modifyt = 1;
            p = p->next;
        }
    }

    return anz;
}

//!
//! \brief ParaWin::closeEvent
//! \param event
//!
void ParaWin::closeEvent(QCloseEvent *event)
{
    Q_UNUSED (event);

    if (cf != nullptr) {
        QTreeWidgetItem *i = (QTreeWidgetItem *)cf->para_pointer;
        if ((i != nullptr) & (mw != nullptr))
            i->setIcon(0, QIcon());
    }
    delete this;
}

//!
//! \brief ParaWin::keyPressEvent
//! \param event
//!
void ParaWin::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Escape)
        close();
}

//!
//! \brief StringEdit::StringEdit
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
StringEdit::StringEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;

    struct _string_para_ *val = (struct _string_para_ *)cp->data;

    out_str = new QLabel;
    out_str->setGeometry(x, y, 200, 20);
    out_str->setParent( parent );

    ledit = new QLineEdit();
    ledit->setText(val->val);

    ledit->setGeometry(x, y+20, 250, 30);
    ledit->setParent( parent );

    refresh_out_str (ledit->text());

    connect (ledit, SIGNAL(editingFinished()), this, SLOT(ledit_finish()));
    connect (ledit, SIGNAL(textChanged(QString)), this, SLOT(ledit_text_changed(QString)));
}

//!
//! \brief StringEdit::refresh_out_str
//! \param s
//!
void StringEdit::refresh_out_str (QString s)
{
    QFileInfo f( s );
    out_str->setText ( QString("String=%1").arg(f.fileName()) );
}

//!
//! \brief StringEdit::ledit_finish
//!
void StringEdit::ledit_finish()
{
    if (text_changed) {
        // printf ("edit finished\n");
        Q_ASSERT (ledit->text().toStdString().length() < MAX_STRING_VAL_LEN-1);
        struct _string_para_ *sp = (struct _string_para_ *)cp->data;
        strncpy (sp->val, ledit->text().toStdString().c_str(), MAX_STRING_VAL_LEN);

        parawin->rewrite_para_data( cp );
        refresh_out_str (ledit->text());
    }
    text_changed = false;
}

//!
//! \brief StringEdit::ledit_text_changed
//! \param s
//!
void StringEdit::ledit_text_changed( const QString & s)
{
    Q_UNUSED (s);
    text_changed = true;
}

//!
//! \brief FileNameEdit::FileNameEdit
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
FileNameEdit::FileNameEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;

    struct _string_para_ *val = (struct _string_para_ *)cp->data;

    // out_str = new QLabel (QString("imread Dateiname=%1").arg(QString(val->val)));
    out_str = new QLabel;
    out_str->setGeometry(x, y, 200, 20);
    out_str->setParent( parent );
    // refresh_out_str (QString(val->val));

    ledit = new QLineEdit();
    ledit->setText(val->val);

    ledit->setGeometry(x, y+20, 250, 30);
    ledit->setParent( parent );

    pb = new QPushButton();
    pb->setText("...");
    pb->setGeometry(x+260, y+20, 30, 30);
    pb->setParent( parent );

    refresh_out_str (ledit->text());

    connect (ledit, SIGNAL(editingFinished()), this, SLOT(ledit_finish()));
    connect (ledit, SIGNAL(textChanged(QString)), this, SLOT(ledit_text_changed(QString)));
    connect (pb, SIGNAL(clicked(bool)), this, SLOT(pb_pushed()));
}

//!
//! \brief FileNameEdit::refresh_out_str
//! \param s
//!
void FileNameEdit::refresh_out_str (QString s)
{
    QFileInfo f( s );
    if (cp->extra_para == 1)
        out_str->setText ( QString("File-Name=%1").arg(f.fileName()) );         // get_filename
    else
        out_str->setText ( QString("imread Dateiname=%1").arg(f.fileName()) );  // imread
}

//!
//! \brief FileNameEdit::ledit_finish
//!
void FileNameEdit::ledit_finish()
{
    if (text_changed) {
        // printf ("edit finished\n");
        Q_ASSERT (ledit->text().toStdString().length() < MAX_STRING_VAL_LEN-1);
        struct _string_para_ *sp = (struct _string_para_ *)cp->data;
        strncpy (sp->val, ledit->text().toStdString().c_str(), MAX_STRING_VAL_LEN);

        parawin->rewrite_para_data( cp );
        refresh_out_str (ledit->text());
    }
    text_changed = false;
}

//!
//! \brief FileNameEdit::ledit_text_changed
//! \param s
//!
void FileNameEdit::ledit_text_changed( const QString & s)
{
    Q_UNUSED (s);
    text_changed = true;
}

//!
//! \brief FileNameEdit::pb_pushed
//!
void FileNameEdit::pb_pushed()
{
    QString file = QFileDialog::getOpenFileName( this,
                                                 "Select File",
                                                 "",
                                                 tr("all (*.*);; \
                                                     Bitmaps (*.bmp *.dib);;  \
                                                     JPEG files (*.jpeg *.jpg);; \
                                                     Portable Network Graphics (*.png);;  \
                                                     TIFF files (*.tiff *.tif)" \
                                                    ));
    if (!file.isNull()) {
        ledit->setText(file);
        text_changed = true;
        ledit_finish();
    }
}
//!
//! \brief RectDoubleEdit::RectDoubleEdit
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
RectDoubleEdit::RectDoubleEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;
    struct _rect_double_ *val = (struct _rect_double_ *)cp->data;

    out_str = new QLabel (QString("%1 %2 %3 %4 %5").arg(QString(cp->para_name))
                          .arg(QString::number(val->x))
                          .arg(QString::number(val->y))
                          .arg(QString::number(val->w))
                          .arg(QString::number(val->h)));

    out_str->setGeometry(x, y, 300, 20);
    out_str->setParent( parent );

    for (int i=0; i<4; i++) {
        dedit[i] = new QDoubleSpinBox();
        dedit[i]->setRange( val->min_val, val->max_val );
        dedit[i]->setValue( *(double*)(cp->data + sizeof(double)*i) );
        dedit[i]->setMinimum( val->min_val);
        dedit[i]->setMaximum( val->max_val );
        dedit[i]->setGeometry(x+80*i, y+20, 75, 30);
        dedit[i]->setParent( parent );

        connect (dedit[i], SIGNAL(editingFinished()), this, SLOT(double_rect_finish()));
    }
}
//!
//!//! \brief oubleEdit::double_rect_finish
//!
void RectDoubleEdit::double_rect_finish()
{
    struct _rect_double_ *data = (struct _rect_double_ *)cp->data;

    data->x = dedit[0]->value();
    data->y = dedit[1]->value();
    data->w = dedit[2]->value();
    data->h = dedit[3]->value();

    out_str->setText (QString("%1 %2 %3 %4 %5").arg(QString(cp->para_name))
                      .arg(QString::number(data->x))
                      .arg(QString::number(data->y))
                      .arg(QString::number(data->w))
                      .arg(QString::number(data->h)));

    parawin->rewrite_para_data( cp );
}
//!
//! \brief RectIntEdit::RectIntEdit
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
RectIntEdit::RectIntEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;
    struct _rect_int_ *val = (struct _rect_int_ *)cp->data;

    out_str = new QLabel (QString("%1 %2 %3 %4 %5").arg(QString(cp->para_name))
                          .arg(QString::number(val->x))
                          .arg(QString::number(val->y))
                          .arg(QString::number(val->w))
                          .arg(QString::number(val->h)));
    out_str->setGeometry(x, y, 300, 20);
    out_str->setParent( parent );

    for (int i=0; i<4; i++) {
        iedit[i] = new QSpinBox();
        iedit[i]->setRange( val->min_val, val->max_val );
        iedit[i]->setValue( *(int*)(cp->data + sizeof(int)*i) );
        iedit[i]->setMinimum( val->min_val);
        iedit[i]->setMaximum( val->max_val );
        iedit[i]->setGeometry(x+70*i, y+20, 65, 30);
        iedit[i]->setParent( parent );

        connect (iedit[i], SIGNAL(editingFinished()), this, SLOT(int_rect_finish()));
    }
}

void RectIntEdit::int_rect_finish()
{
    struct _rect_int_ *data = (struct _rect_int_ *)cp->data;

    data->x = iedit[0]->value();
    data->y = iedit[1]->value();
    data->w = iedit[2]->value();
    data->h = iedit[3]->value();

    out_str->setText (QString("%1 %2 %3 %4 %5").arg(QString(cp->para_name))
                      .arg(QString::number(data->x))
                      .arg(QString::number(data->y))
                      .arg(QString::number(data->w))
                      .arg(QString::number(data->h)));

    parawin->rewrite_para_data( cp );
}
//!
//! \brief IntEdit::IntEdit
//!        Integer Spinbox
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
IntEdit::IntEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;

    struct _int_para_ *val = (struct _int_para_ *)cp->data;

    out_str = new QLabel (QString("%1=%2  min=%3  max=%4")
                          .arg(QString(cp->para_name))
                          .arg(QString::number(val->value))
                          .arg(QString::number(val->min))
                          .arg(QString::number(val->max)));
    out_str->setGeometry(x, y, 600, 20);
    out_str->setParent( parent );

    iedit = new QSpinBox();     // Mit SpinBox kann kein korrektes unsigned int abgebildet werden. Als max ist maximal nur <int>::max() möglich.
    iedit->setRange( val->min, val->max );
    iedit->setValue( val->value );
    iedit->setMinimum( val->min );
    iedit->setMaximum( val->max );  // <unsigned int>::max() = <int>::max()

    iedit->setReadOnly( cp->flags & READ_ONLY );
    iedit->setVisible( !(cp->flags & READ_ONLY) );  // s.auch: setHidden

    iedit->setGeometry(x, y+20, 200, 30);
    iedit->setParent( parent );

    connect (iedit, SIGNAL(editingFinished()), this, SLOT(int_edit_finish()));
}
//!
//! \brief IntEdit::int_edit_finish
//!
void IntEdit::int_edit_finish()
{
    struct _int_para_ *val = (struct _int_para_ *)cp->data;

    val->value = iedit->value();
    out_str->setText(QString("%1=%2  min=%3  max=%4")
                      .arg(QString(cp->para_name))
                      .arg(QString::number(val->value))
                      .arg(QString::number(val->min))
                      .arg(QString::number(val->max)));

    parawin->rewrite_para_data( cp );
}

//!
//! \brief IntEdit::int_edit_para_button_pushed
//!
void IntEdit::int_edit_para_button_pushed ()
{
    printf ("IntEdit Treffer\n");
}

//!
//! \brief FloatEdit::FloatEdit
//!        Float Spinbox. Achtung: Es wird die DoubleSpinBox verwendet.
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
FloatEdit::FloatEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;

    struct _float_para_ *val = (struct _float_para_ *)cp->data;

    out_str = new QLabel (QString("%1=%2").arg(QString(cp->para_name)).arg(QString::number(val->value)));
    out_str->setGeometry(x, y, 200, 20);
    out_str->setParent( parent );

    dedit = new QDoubleSpinBox();
    dedit->setRange( val->min, val->max );
    dedit->setValue( val->value );
    dedit->setMinimum( val->min );
    dedit->setMaximum( val->max );
    dedit->setGeometry(x, y+20, 200, 30);
    dedit->setParent( parent );

    connect (dedit, SIGNAL(editingFinished()), this, SLOT(float_edit_finish()));
}

//!
//! \brief FloatEdit::float_edit_finish
//!
void FloatEdit::float_edit_finish()
{
    struct _float_para_ *val = (struct _float_para_ *)cp->data;

    val->value = dedit->value();
    out_str->setText(QString("%1=%2").arg(QString(cp->para_name)).arg(QString::number(val->value)));

    parawin->rewrite_para_data( cp );
}

//!
//! \brief DoubleEdit::DoubleEdit
//!        Double Spinbox
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
DoubleEdit::DoubleEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;

    struct _double_para_ *val = (struct _double_para_ *)cp->data;

    out_str = new QLabel (QString("%1=%2  min=%3  max=%4")
                          .arg(QString(cp->para_name))
                          .arg(QString::number(val->value))
                          .arg(QString::number(val->min))
                          .arg(QString::number(val->max)));
    out_str->setGeometry(x, y, 600, 20);
    out_str->setParent( parent );

    // printf ("%f\n", val->value);

    dedit = new QDoubleSpinBox();
    dedit->setRange( val->min, val->max );
    dedit->setDecimals ( val->decimals );   // Nachkommastellen
    dedit->setValue( val->value );
    dedit->setMinimum( val->min );
    dedit->setMaximum( val->max );
    dedit->setGeometry(x, y+20, 200, 30);
    dedit->setParent( parent );

    connect (dedit, SIGNAL(editingFinished()), this, SLOT(double_edit_finish()));

    /*
    para_button = new QPushButton();
    para_button->setIcon(glob_mw->iconlist[2]);
    para_button->setGeometry(x+210, y+25, 20, 20);
    para_button->setParent( parent );
    para_button->setToolTip( "Eigenschaft" );
    connect (para_button, SIGNAL(clicked(bool)), this, SLOT(double_edit_para_button_pushed()));
    */
}

//!
//! \brief DoubleEdit::double_edit_finish
//!
void DoubleEdit::double_edit_finish()
{
    struct _double_para_ *val = (struct _double_para_ *)cp->data;

    val->value = dedit->value();
    out_str->setText(QString("%1=%2  min=%3  max=%4")
                     .arg(QString(cp->para_name))
                     .arg(QString::number(val->value))
                     .arg(QString::number(val->min))
                     .arg(QString::number(val->max)));

    parawin->rewrite_para_data( cp );
}

//!
//! \brief DoubleEdit::double_edit_para_button_pushed
//!
void DoubleEdit::double_edit_para_button_pushed ()    // Eigenschaft
{
    // printf ("Treffer double_edit_para_button_pushed\n");
}

//!
//! \brief EnumDrop::EnumDrop
//!        DropDown Menue
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
EnumDrop::EnumDrop (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;
    int current_index = 0;

    struct _enum_para_ *ep = (struct _enum_para_ *)cp->data;
    QStringList items;
    QDomNodeList nl = doc.elementsByTagName(ep->enum_name);
    // qDebug() << "EnumDrop " << ep->enum_name << "  " << nl.length();

    if (nl.length()) {                                  // es ist ein Element gefunden worden.
        QDomElement e = nl.at(0).toElement();           // 1.Element verwenden
        QDomElement c = e.firstChild().toElement();     // 1.Child vom 1.Element
        int n = 0;
        while (c.parentNode() == e) {
            if (c.text().toInt() == ep->value) {
                current_index = n;
            }
            // qDebug()  << c.tagName();
            items += c.tagName();                       // tagName-Text in stringlist eintragen
            c = c.nextSibling().toElement();
            n++;
        }
    }
    int *val = (int *)cp->data;
    out_str = new QLabel(QString("%1 = %2").arg(QString(cp->para_name)).arg(QString::number(*val)));
    out_str->setGeometry(x, y, 200, 20);
    out_str->setParent( parent );

    drop = new QComboBox();
    drop->addItems( items );                    // QStringList insert.
    drop->setCurrentIndex( current_index );
    drop->setGeometry(x, y+20, 200, 30);
    drop->setParent( parent );

    connect (drop, SIGNAL(currentIndexChanged(QString)), this, SLOT(new_enum_select(const QString &)) );
}

//!
//! \brief EnumDrop::new_enum_select
//! \param s
//!
void EnumDrop::new_enum_select(const QString &s)
{
    QDomNodeList nl = doc.elementsByTagName(s);
    if (nl.length()) {
        int new_val = nl.at(0).toElement().text().toInt();              // int erzeugen
        // qDebug() << new_val;
        struct _enum_para_ *sp = (struct _enum_para_ *)cp->data;
        sp->value = new_val;
        out_str->setText(QString("%1 = %2").arg(QString(cp->para_name)).arg(QString::number(sp->value)));

        parawin->rewrite_para_data( cp );
    }
}

//////////////////////////////////////////// Slide //////////////////////////////////////////////////////
//!
//! \brief Slide::Slide
//!        Horizontal Slider
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
Slide::Slide(QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent) : QWidget (parent)
{
    cp = foo;
    client = c;    

    switch (cp->type) {
    case SLIDE_INT_TWO_STEP_PARA:
    case SLIDE_INT_PARA: {
        struct _int_para_ *sp = (struct _int_para_ *)cp->data;

        if (cp->type == SLIDE_INT_TWO_STEP_PARA) stepwidth = 2;

        l = new QLabel (QString("%1=%2  min=%3  max=%4").arg(QString(cp->para_name))
                                                         .arg(QString::number(sp->value))
                                                         .arg(QString::number(sp->min))
                                                         .arg(QString::number(sp->max)));
        l->setGeometry(x, y, 300, 20);
        l->setParent( parent );

        s = new QSlider ( Qt::Horizontal );
        s->setPageStep( 1 );
        s->setSingleStep( 1 );
        s->setTickInterval( 1 );
        s->setGeometry(x, y+20, 200, 30);

        delta = (sp->max - sp->min) / stepwidth;
        s->setMinimum( 0 );
        s->setMaximum( delta );
        s->setSliderPosition( (sp->value - sp->min) / stepwidth );    // sp->value = sp->min + (position * stepwidth);


        s->setStyleSheet("QSlider::groove:horizontal { "            // Slider Aussehen veraendern.
                              "border: 1px solid #999999; "
                              "height: 15px; "
                              "background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4); "
                              "margin: 2px 0; "
                              "} "
                              "QSlider::handle:horizontal { "
                              "background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f); "
                              "border: 1px solid #5c5c5c; "
                              "width: 15px; "
                              "margin: -2px 0px; "
                              "} ");

        s->setParent( parent );

        connect (s, SIGNAL(valueChanged(int)), this, SLOT(slide_value_changed(int)));        
        }
        break;

    case SLIDE_DOUBLE_PARA: {
        struct _slide_double_para_ *sp = (struct _slide_double_para_ *)cp->data;
        l = new QLabel (QString("%1=%2  min=%3  max=%4").arg(QString(cp->para_name))
                                                         .arg(QString::number(sp->value))
                                                         .arg(QString::number(sp->min))
                                                         .arg(QString::number(sp->max)));
        l->setGeometry(x, y, 300, 20);
        l->setParent( parent );

        s = new QSlider ( Qt::Horizontal );
        s->setPageStep( 1 );
        s->setSingleStep( 1 );
        s->setTickInterval( 1 );
        s->setGeometry(x, y+20, 200, 30);

        s->setMinimum( sp->min * sp->divisor);
        s->setMaximum( sp->max * sp->divisor);
        s->setSliderPosition( sp->value * sp->divisor);

        s->setStyleSheet("QSlider::groove:horizontal { "            // Slider Aussehen veraendern.
                              "border: 1px solid #999999; "
                              "height: 15px; "
                              "background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #D1D1D1, stop:1 #E4E4E4); "
                              "margin: 2px 0; "
                              "} "
                              "QSlider::handle:horizontal { "
                              "background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f); "
                              "border: 1px solid #5c5c5c; "
                              "width: 15px; "
                              "margin: -2px 0px; "
                              "} ");

        s->setParent( parent );

        connect (s, SIGNAL(valueChanged(int)), this, SLOT(slide_value_changed(int)));

        }
        break;
    }

    /*
    para_button = new QPushButton();
    para_button->setIcon(glob_mw->iconlist[2]);
    para_button->setGeometry(x+210, y+25, 20, 20);
    para_button->setParent( parent );
    para_button->setToolTip( "Eigenschaft" );
    connect (para_button, SIGNAL(clicked(bool)), this, SLOT(slide_para_button_pushed()));   // Eigenschaft
    */
}

//!
//! \brief Slide::slide_value_changed
//! \param val
//!
void Slide::slide_value_changed (int val)
{
    // qDebug() << "slide_value_changed";
    switch (cp->type) {
    case SLIDE_INT_TWO_STEP_PARA:
    case SLIDE_INT_PARA: {
        struct _int_para_ *sp = (struct _int_para_ *)cp->data;
        sp->value = sp->min + (val * stepwidth);
        l->setText(QString("%1=%2  min=%3  max=%4").arg(QString(cp->para_name))
                                                    .arg(QString::number(sp->value))
                                                    .arg(QString::number(sp->min))
                                                    .arg(QString::number(sp->max)));
        }
        break;
    case SLIDE_DOUBLE_PARA: {
        struct _slide_double_para_ *sp = (struct _slide_double_para_ *)cp->data;
        sp->value = (double)val / sp->divisor;
        l->setText(QString("%1=%2 min=%3 max=%4").arg(QString(cp->para_name))
                                                    .arg(QString::number(sp->value))
                                                    .arg(QString::number(sp->min))
                                                    .arg(QString::number(sp->max)));
        }
        break;
    }

    parawin->rewrite_para_data( cp );
}

//!
//! \brief Slide::slide_para_button_pushed
//!
void Slide::slide_para_button_pushed ()
{
    // printf ("Treffer\n");
}

PointDouble::PointDouble(QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;

    struct _point_double_ *val = (struct _point_double_ *)cp->data;

    l = new QLabel();
    set_Text( val->x, val->y );

    l->setGeometry(x, y, 200, 20);
    l->setParent( parent );

    x_koor = new QDoubleSpinBox();
    x_koor->setDecimals ( val->decimals_x);
    x_koor->setMinimum( val->min_x );
    x_koor->setMaximum( val->max_x );
    x_koor->setValue( val->x );
    x_koor->setGeometry(x, y+20, 100, 30);
    x_koor->setParent( parent );

    y_koor = new QDoubleSpinBox();
    y_koor->setDecimals ( val->decimals_y);
    y_koor->setMinimum( val->min_y );
    y_koor->setMaximum( val->max_y );
    y_koor->setValue( val->y );
    y_koor->setGeometry(x+120, y+20, 100, 30);
    y_koor->setParent( parent );

    connect (x_koor, SIGNAL(editingFinished()), this, SLOT(X_edit_finish()));
    connect (y_koor, SIGNAL(editingFinished()), this, SLOT(Y_edit_finish()));
}

void PointDouble::set_Text( double x, double y )
{
    l->setText (QString("%1 ( w=%2, h=%3 )").arg(QString(cp->para_name))
                                            .arg(QString::number(x))
                                            .arg(QString::number(y)));
}

void PointDouble::X_edit_finish ()
{
    struct _point_double_ *val = (struct _point_double_ *)cp->data;

    val->x = x_koor->value();
    set_Text (val->x, val->y);

    parawin->rewrite_para_data( cp );
}

//!
//! \brief PointInt::Y_edit_finish
//!
void PointDouble::Y_edit_finish ()
{
    struct _point_double_ *val = (struct _point_double_ *)cp->data;

    val->y = y_koor->value();
    set_Text (val->x, val->y);

    parawin->rewrite_para_data( cp );
}


//!
//! \brief PointInt::PointInt
//!        2D Point einlesen
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
PointInt::PointInt(QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;

    struct _point_int_ *val = (struct _point_int_ *)cp->data;

    l = new QLabel();
    set_Text( val->x, val->y );

    l->setGeometry(x, y, 200, 20);
    l->setParent( parent );

    x_koor = new QSpinBox();
    x_koor->setMinimum( val->min_x );
    x_koor->setMaximum( val->max_x );
    x_koor->setValue( val->x );
    x_koor->setGeometry(x, y+20, 100, 30);
    x_koor->setParent( parent );

    y_koor = new QSpinBox();
    y_koor->setMinimum( val->min_y );
    y_koor->setMaximum( val->max_y );
    y_koor->setValue( val->y );
    y_koor->setGeometry(x+120, y+20, 100, 30);
    y_koor->setParent( parent );

    connect (x_koor, SIGNAL(editingFinished()), this, SLOT(X_edit_finish()));
    connect (y_koor, SIGNAL(editingFinished()), this, SLOT(Y_edit_finish()));

    // connect (x_koor, SIGNAL(valueChanged()), this, SLOT(X_edit_finish()));
    // connect (y_koor, SIGNAL(valueChanged()), this, SLOT(Y_edit_finish()));
}

//!
//! \brief PointInt::set_Text
//! \param x
//! \param y
//!
void PointInt::set_Text( int x, int y )
{
    if (cp->type == POINT_INT)
        l->setText (QString("%1 ( w=%2, h=%3 )").arg(QString(cp->para_name))
                                                .arg(QString::number(x))
                                                .arg(QString::number(y)));
    if (cp->type == POINT_INT_XY)
        l->setText (QString("%1 ( x=%2, y=%3 )").arg(QString(cp->para_name))
                                                .arg(QString::number(x))
                                                .arg(QString::number(y)));
}

//!
//! \brief PointInt::X_edit_finish
//!
void PointInt::X_edit_finish ()
{
    struct _point_int_ *val = (struct _point_int_ *)cp->data;

    val->x = x_koor->value();
    set_Text (val->x, val->y);

    parawin->rewrite_para_data( cp );
}

//!
//! \brief PointInt::Y_edit_finish
//!
void PointInt::Y_edit_finish ()
{
    struct _point_int_ *val = (struct _point_int_ *)cp->data;

    val->y = y_koor->value();
    set_Text (val->x, val->y);

    parawin->rewrite_para_data( cp );
}

//!
//! \brief ScalarDouble::ScalarDouble
//!        cv::Scalar
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param parent
//!
ScalarDouble::ScalarDouble(QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent)
{
    cp = foo;
    client = c;

    struct _scalar_double_ *val = (struct _scalar_double_ *)cp->data;

    l = new QLabel (QString("%1 (v0=%2, v1=%3, v2=%4, v3=%5 )").arg(QString(cp->para_name))
                                                               .arg(QString::number(val->val[0]))
                                                               .arg(QString::number(val->val[1]))
                                                               .arg(QString::number(val->val[2]))
                                                               .arg(QString::number(val->val[3])));
    l->setGeometry(x, y, 350, 20);
    l->setParent( parent );

    for (int i=0; i<4; i++) {
        e[i] = new QDoubleSpinBox();
        e[i]->setMinimum( -100000.0 );
        e[i]->setMaximum( 100000.0 );
        e[i]->setRange (-100000.0, 100000.0);
        e[i]->setValue( val->val[i] );
        e[i]->setGeometry(x+85*i, y+20, 75, 25);
        e[i]->setParent( parent );
    }
    connect (e[0], SIGNAL(editingFinished()), this, SLOT(e0_edit_finish()));
    connect (e[1], SIGNAL(editingFinished()), this, SLOT(e1_edit_finish()));
    connect (e[2], SIGNAL(editingFinished()), this, SLOT(e2_edit_finish()));
    connect (e[3], SIGNAL(editingFinished()), this, SLOT(e3_edit_finish()));
}

//!
//! \brief ScalarDouble::Refresh_Val
//! \param index
//!
void ScalarDouble::Refresh_Val (int index)
{
    struct _scalar_double_ *val = (struct _scalar_double_ *)cp->data;

    val->val[index] = e[index]->value();
    l->setText(QString("%1 (v0=%2, v1=%3, v2=%4, v3=%5 )").arg(QString(cp->para_name))
               .arg(QString::number(val->val[0]))
               .arg(QString::number(val->val[1]))
               .arg(QString::number(val->val[2]))
               .arg(QString::number(val->val[3])));

    parawin->rewrite_para_data( cp );
}
//!
//! \brief ScalarDouble::e0_edit_finish
//!
void ScalarDouble::e0_edit_finish () { Refresh_Val( 0 ); }
void ScalarDouble::e1_edit_finish () { Refresh_Val( 1 ); }
void ScalarDouble::e2_edit_finish () { Refresh_Val( 2 ); }
void ScalarDouble::e3_edit_finish () { Refresh_Val( 3 ); }

//!
//! \brief mButton::mButton
//!        Close / Reset
//! \param c
//! \param foo
//! \param x
//! \param y
//! \param b_type
//! \param parent
//! \param main_win
//!
mButton::mButton(QTcpSocket *c, struct _cvd_func_ *foo, int x, int y, int b_type, QWidget *parent, MainWindow *main_win)
{
    mw = main_win;
    cf = foo;
    client = c;
    button_type = b_type;

    push_button = new QPushButton();    
    if (button_type == mRESET) {        
        push_button->setStyleSheet("QPushButton {background-color: #A3C1DA; color: red;}");
    }
    push_button->setText (m_button[button_type].text);
    push_button->setGeometry(x, y, m_button[button_type].width, 30);
    push_button->setParent( parent );

    connect (push_button, SIGNAL(clicked(bool)), this, SLOT(mpb_pushed()));
}

//!
//! \brief mButton::mpb_pushed
//!        close OR reset pushed
//!
void mButton::mpb_pushed()
{
    switch (button_type) {
        case mCLOSE:
            parawin->close_parawin();                   // Parameter Fenster schliessen
            // parawin->close();
            break;
        case mRESET: {
                struct _cvd_func_ *cf = parawin->cf;        // cf retten
                struct _cvd_para_ *cp = cf->first_para;
                while (cp != nullptr) {
                    memcpy (cp->data, cp->reset_data, MAX_PARA_DATA);
                    parawin->rewrite_para_data( cp );
                    cp = cp->next;
                }                

                parawin->close_parawin();                   // Parameter Fenster schliessen
                // parawin->close();
                parawin = nullptr;
                if (parawin == nullptr) {
                    QTreeWidgetItem *i = (QTreeWidgetItem *)cf->para_pointer;
                    if (i != nullptr)
                        i->setIcon(0, mw->iconlist[OK_ICON]);
                    parawin = new ParaWin(client, cf, mw);      // neues Parameter-Fenster oeffnen.                    
                }
                parawin->cf->func_is_modifyt = 0;                    // Func reset. Func is not modifyt.
            }
            break;
        default:
            printf ("unbekannter case\n");
            break;
    }
}
