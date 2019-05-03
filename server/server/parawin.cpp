#include <iostream>
#include <unistd.h>
#include <QKeyEvent>
#include <QFileDialog>
#include <QTreeWidgetItem>

#include "mainwindow.h"
#include "parawin.h"

ParaWin *parawin = NULL;

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
ParaWin::ParaWin(QTcpSocket *c, struct _cvd_func_ *foo, MainWindow *main_win, QWidget *parent)
{
    Q_UNUSED (parent);

    this->setGeometry(380, 180, 400, 300);
    client = c;
    cf = foo;
    this->setWindowTitle(cf->func_name);
    mw = main_win;

    switch (cf->type) {
    case NORMALIZE:
        new DoubleEdit (client, cf->first_para, 20, 10+55*0, this );   // alpha
        new DoubleEdit (client, cf->first_para->next, 20, 10+55*1, this );   // beta
        new EnumDrop (client, cf->first_para->next->next, 20, 10+55*2, this );   // norm_type
        new EnumDrop (client, cf->first_para->next->next->next, 20, 10+55*3, this );   // ddepth

        new mButton (client, cf, 20, 10+55*4+10, mCLOSE, this, mw );                            // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*4+10, mRESET, this, mw );  // Reset
        setGeometry(320, 150, 260, 10+55*5);
        break;
    case GETSTRUCTURINGELEMENT:
        new EnumDrop (client, cf->first_para, 20, 10+55*0, this );   // shape = DropDown enum MorphShapes
        new PointInt ( client, cf->first_para->next, 20, 10+55*1, this );   // ksize
        new PointInt ( client, cf->first_para->next->next, 20, 10+55*2, this );   // annchor

        new mButton (client, cf, 20, 10+55*3+10, mCLOSE, this, mw );                            // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*3+10, mRESET, this, mw );  // Reset
        setGeometry(320, 150, 260, 10+55*4);
        break;
    case GRABCUT:
        new IntEdit (client, cf->first_para, 20, 10+55*0, this );     // Iteration
        new EnumDrop (client, cf->first_para->next, 20, 10+55*1, this );   // DropDown enum ImreadModes
        setGeometry(320, 150, 320, 10+55*2+20);
        break;
    case IMREAD:
        new StringEdit (client, cf->first_para, 20, 10+55*0, this );
        new EnumDrop (client, cf->first_para->next, 20, 10+55*1, this );   // DropDown enum ImreadModes

        new mButton (client, cf, 20, 10+55*2+10, mCLOSE, this, mw );                            // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*2+10, mRESET, this, mw );  // Reset
        setGeometry(320, 150, 330, 10+55*3);
        break;
    case OPERATOR_INT_MUL_EQUAL:
        new IntEdit (client, cf->first_para, 20, 10, this );   // Double Edit für scale
        setGeometry(340, 150, 260, 80);
        break;
    case OPERATOR_FLOAT_MUL_EQUAL:
        new FloatEdit (client, cf->first_para, 20, 10, this );   // Float Edit für scale. Achtung:
        setGeometry(340, 150, 260, 80);
        break;
    case OPERATOR_DOUBLE_MUL_EQUAL:
        new DoubleEdit (client, cf->first_para, 20, 10, this );   // Double Edit für scale
        setGeometry(340, 150, 260, 80);
        break;
    case CONVERTTO:
        new EnumDrop (client, cf->first_para, 20, 10, this );   // DropDown enum ddepth
        new DoubleEdit (client, cf->first_para->next, 20, 65, this );   // Double Edit für scale
        new DoubleEdit (client, cf->first_para->next->next, 20, 65+55, this );   // Double Edit für delta
        setGeometry(320, 150, 260, 80+55+55);
        break;
    case CONVERTSCALEABS:
        new DoubleEdit (client, cf->first_para, 20, 10+55*0, this );            // Double Edit für alpha
        new DoubleEdit (client, cf->first_para->next, 20, 10+55*1, this );   // Double Edit für beta

        new mButton (client, cf, 20, 10+55*2+10, mCLOSE, this, mw );                            // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*2+10, mRESET, this, mw );  // Reset
        setGeometry(320, 150, 260, 10+55*3);
        break;
    case CANNY_2:
        new Slide ( client, cf->first_para, 20, 10+55*0, this );                     // Slider für threshold1
        new Slide ( client, cf->first_para->next, 20, 10+55*1, this );            // Slider für threshold2
        new EnumDrop (client, cf->first_para->next->next, 20, 10+55*2, this );   // DropDown enum L2gradient (bool)

        new mButton (client, cf, 20, 10+55*3+10, mCLOSE, this, mw );                            // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*3+10, mRESET, this, mw );  // Reset
        setGeometry(320, 150, 260, 10+55*4);
        break;
    case CANNY:
        new Slide ( client, cf->first_para, 20, 10+55*0, this );                     // Slider für threshold1
        new Slide ( client, cf->first_para->next, 20, 10+55*1, this );            // Slider für threshold2
        new Slide ( client, cf->first_para->next->next, 10, 10+55*2, this );   // Slider für apertureSize 3.. ???
        new EnumDrop (client, cf->first_para->next->next->next, 20, 10+55*3, this );   // DropDown enum L2gradient (bool)

        new mButton (client, cf, 20, 10+55*4+10, mCLOSE, this, mw );                            // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*4+10, mRESET, this, mw );  // Reset
        setGeometry(320, 150, 260, 10+55*5);
        break;
    case LAPLACIAN:
        new EnumDrop (client, cf->first_para, 20, 10+55*0, this );   // DropDown enum
        new Slide ( client, cf->first_para->next, 20, 10+55*1, this );   // Slider für kzise
        new DoubleEdit (client, cf->first_para->next->next, 20, 10+55*2, this );   // Double Edit für scale
        new DoubleEdit (client, cf->first_para->next->next->next, 20, 10+55*3, this );   // Double Edit für delta
        new EnumDrop (client, cf->first_para->next->next->next->next, 20, 10+55*4, this );   // DropDown enum BorderType

        new mButton (client, cf, 20, 10+55*5+10, mCLOSE, this, mw );                            // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*5+10, mRESET, this, mw );  // Reset
        setGeometry(320, 150, 260, 10+55*6);
        break;
    case CVTCOLOR:
        new EnumDrop (client, cf->first_para, 20, 10+55*0, this );   // DropDown enum
        new IntEdit (client, cf->first_para->next, 20, 10+55*1, this );     // Iteration

        new mButton (client, cf, 20, 10+55*2+10, mCLOSE, this, mw );   // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*2+10, mRESET, this, mw );   // Reset
        setGeometry(320, 150, 260, 10+55*3);
        break;
    case GAUSSIANBLUR:
        new Slide ( client, cf->first_para, 20, 10+55*0, this );                     // Slider für ksize.width
        new Slide ( client, cf->first_para->next, 20, 10+55*1, this );            // Slider für ksize.height
        new DoubleEdit (client, cf->first_para->next->next, 20, 10+55*2, this );               // sigmaX
        new DoubleEdit (client, cf->first_para->next->next->next, 20, 10+55*3, this );      // sigmaY
        new EnumDrop (client, cf->first_para->next->next->next->next, 20, 10+55*4, this );   // DropDown enum BorderType

        new mButton (client, cf, 20, 10+55*5+10, mCLOSE, this, mw );   // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*5+10, mRESET, this, mw );   // Reset
        setGeometry(320, 150, 260, 10+55*6);
        break;
    case BLUR_FUNC:
        new Slide ( client, cf->first_para, 20, 10+55*0, this );            // Slider für ksize.width
        new Slide ( client, cf->first_para->next, 20, 10+55*1, this );      // Slider für ksize.height
        new PointInt ( client, cf->first_para->next->next, 20, 10+55*2, this );      // Point
        new EnumDrop (client, cf->first_para->next->next->next, 20, 10+55*3, this );   // DropDown enum BorderType

        new mButton (client, cf, 20, 10+55*4+10, mCLOSE, this, mw );   // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*4+10, mRESET, this, mw );   // Reset
        setGeometry(320, 150, 260, 10+55*5);
        break;
    case MEDIANBLUR:
        new Slide ( client, cf->first_para, 20, 10+55*0, this );   // Slider für kzise

        new mButton (client, cf, 20, 10+55*1+10, mCLOSE, this, mw );   // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*1+10, mRESET, this, mw );   // Reset
        setGeometry(320, 150, 260, 10+55*2);
        break;
    case THRESHOLD:
        new Slide ( client, cf->first_para, 20, 10+55*0, this );         // Slider für thresh
        new Slide ( client, cf->first_para->next, 20, 10+55*1, this );   // Slider für maxval
        new EnumDrop (client, cf->first_para->next->next, 20, 10+55*2, this );   // DropDown enum

        new mButton (client, cf, 20, 10+55*3+10, mCLOSE, this, mw );     // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*3+10, mRESET, this, mw );   // Reset
        setGeometry(320, 150, 260, 10+55*4);
        break;
    case SCALAR_FUNC_4:
        new ScalarDouble ( client, cf->first_para, 20, 10+55*0, this );  //
        setGeometry(320, 150, 370, 10+55*1+20);
        break;
    case SCALAR_ALL:
        new DoubleEdit (client, cf->first_para, 20, 10+55*0, this );    // v0
        setGeometry(320, 150, 370, 10+55*1+20);
        break;
    case ERODE:
    case DILATE:
        new PointInt ( client, cf->first_para, 20, 10+55*0, this );         // Point
        new IntEdit (client, cf->first_para->next, 20, 10+55*1, this );     // Iteration
        new EnumDrop (client, cf->first_para->next->next, 20, 10+55*2, this );   // DropDown enum

        new mButton (client, cf, 20, 10+55*3+10, mCLOSE, this, mw );     // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*3+10, mRESET, this, mw );   // Reset
        setGeometry(320, 150, 260, 10+55*4);
        break;
    case MORPHOLOGYEX:
        new EnumDrop (client, cf->first_para, 20, 10+55*0, this );   // op
        new PointInt ( client, cf->first_para->next, 20, 10+55*1, this );   // annchor
        new IntEdit (client, cf->first_para->next->next, 20, 10+55*2, this );     // Iteration
        new EnumDrop (client, cf->first_para->next->next->next, 20, 10+55*3, this );   // borderType

        new mButton (client, cf, 20, 10+55*4+10, mCLOSE, this, mw );     // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*4+10, mRESET, this, mw );   // Reset
        setGeometry(320, 150, 260, 10+55*5);
        break;
    case FINDCONTOURS:
        new EnumDrop (client, cf->first_para, 20, 10+55*0, this );                  // DropDown enum RetrievalModes
        new EnumDrop (client, cf->first_para->next, 20, 10+55*1, this );            // DropDown enum ContourApproximationModes
        new PointInt ( client, cf->first_para->next->next, 20, 10+55*2, this );     // Point offset

        new mButton (client, cf, 20, 10+55*3+10, mCLOSE, this, mw );     // Close
        new mButton (client, cf, 20+m_button[mCLOSE].width+10, 10+55*3+10, mRESET, this, mw );   // Reset
        setGeometry(320, 150, 260, 10+55*4);
        break;
    case SCHARR: {
        new EnumDrop (client, cf->first_para, 20, 10+55*0, this );                  // DropDown enum filterdepth_CV_8U | filterdepth_CV_16U_CV_16S
        new IntEdit (client, cf->first_para->next, 20, 10+55*1, this );             // dx
        new IntEdit (client, cf->first_para->next->next, 20, 10+55*2, this );       // dy
        new DoubleEdit (client, cf->first_para->next->next->next, 20, 10+55*3, this );
        new DoubleEdit (client, cf->first_para->next->next->next->next, 20, 10+55*4, this );
        new EnumDrop (client, cf->first_para->next->next->next->next->next, 20, 10+55*5, this );
        setGeometry(320, 150, 370, 10+55*6+20);
        }
        break;
    default:
        printf ("unbekannte Funktion\n");
        break;
    }

    this->show();
}

//!
//! \brief ParaWin::~ParaWin
//!
ParaWin::~ParaWin()
{
    parawin = NULL;
}

//!
//! \brief ParaWin::close_parawin
//!        Funktion schließt Parameterfenster
//!
void ParaWin::close_parawin()
{
    if (parawin != NULL) {
        if (mw != NULL) {
            QTreeWidgetItem *tw = (QTreeWidgetItem *)cf->para_pointer;
            tw->setIcon(0, QIcon());
        }
        delete parawin;     // close Parameter Window
    }
    parawin = NULL;
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
    strcpy (pd.para_name, foo->para_name);
    memcpy (pd.data, foo->data, MAX_PARA_DATA);
    // printf ("Sende bef=%4X\n", pd.type);
    mw->ack_detected = 0;
    anz = mw->write_data((const char *)&pd, sizeof(struct _para_data_transfer_));

    if (parawin != NULL) {
        parawin->cf->func_is_modifyt = 0;
        struct _cvd_para_ *p = parawin->cf->first_para;
        while (p != NULL) {                                 // Alle Parameter untersuchen.
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

    if (cf != NULL) {
        QTreeWidgetItem *i = (QTreeWidgetItem *)cf->para_pointer;
        if ((i != NULL) & (mw != NULL))
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
//! \brief StringEdit::refresh_out_str
//! \param s
//!
void StringEdit::refresh_out_str (QString s)
{
    QFileInfo f( s );
    out_str->setText ( QString("imread Dateiname=%1").arg(f.fileName()) );
}

//!
//! \brief StringEdit::ledit_finish
//!
void StringEdit::ledit_finish()
{
    if (text_changed) {
        // printf ("edit finished\n");
        struct _string_para_ *sp = (struct _string_para_ *)cp->data;
        strcpy (sp->val, ledit->text().toStdString().c_str());

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
//! \brief StringEdit::pb_pushed
//!
void StringEdit::pb_pushed()
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

    out_str = new QLabel (QString("%1=%2").arg(QString(cp->para_name)).arg(QString::number(val->value)));
    out_str->setGeometry(x, y, 200, 20);
    out_str->setParent( parent );

    iedit = new QSpinBox();
    iedit->setRange( val->min, val->max );
    iedit->setValue( val->value );
    iedit->setMinimum( val->min );
    iedit->setMaximum( val->max );

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
    out_str->setText(QString("%1=%2").arg(QString(cp->para_name)).arg(QString::number(val->value)));

    parawin->rewrite_para_data( cp );
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

    out_str = new QLabel (QString("%1=%2").arg(QString(cp->para_name)).arg(QString::number(val->value)));
    out_str->setGeometry(x, y, 200, 20);
    out_str->setParent( parent );

    // printf ("%f\n", val->value);

    dedit = new QDoubleSpinBox();
    dedit->setRange( val->min, val->max );
    dedit->setValue( val->value );
    dedit->setMinimum( val->min );
    dedit->setMaximum( val->max );
    dedit->setGeometry(x, y+20, 200, 30);
    dedit->setParent( parent );

    connect (dedit, SIGNAL(editingFinished()), this, SLOT(double_edit_finish()));
}

//!
//! \brief DoubleEdit::double_edit_finish
//!
void DoubleEdit::double_edit_finish()
{
    struct _double_para_ *val = (struct _double_para_ *)cp->data;

    val->value = dedit->value();
    out_str->setText(QString("%1=%2").arg(QString(cp->para_name)).arg(QString::number(val->value)));

    parawin->rewrite_para_data( cp );
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
        int n =0;
        while (c.parentNode() == e) {
            if (c.text().toInt() == ep->value) {
                current_index = n;
            }
            // qDebug()  << c.tagName();
            items += c.tagName();
            c = c.nextSibling().toElement();
            n++;
        }
    }
    int *val = (int *)cp->data;
    out_str = new QLabel(QString("%1 = %2").arg(QString(cp->para_name)).arg(QString::number(*val)));
    out_str->setGeometry(x, y, 200, 20);
    out_str->setParent( parent );

    drop = new QComboBox();
    drop->addItems( items );
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
        s->setGeometry(x, y+20, 200, 30);

        s->setMinimum( sp->min);
        s->setMaximum( sp->max);
        s->setSliderPosition( sp->value);
        s->setTickInterval( stepwidth );
        s->setSingleStep( stepwidth );
        s->setPageStep( stepwidth );

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
        struct _double_para_ *sp = (struct _double_para_ *)cp->data;
        l = new QLabel (QString("%1=%2  min=%3  max=%4").arg(QString(cp->para_name))
                                                         .arg(QString::number(sp->value))
                                                         .arg(QString::number(sp->min))
                                                         .arg(QString::number(sp->max)));
        l->setGeometry(x, y, 300, 20);
        l->setParent( parent );

        s = new QSlider ( Qt::Horizontal );
        s->setPageStep( 1 );
        s->setSingleStep( 1 );
        s->setGeometry(x, y+20, 200, 30);

        s->setMinimum( sp->min);
        s->setMaximum( sp->max);
        s->setSliderPosition( sp->value);
        s->setTickInterval( stepwidth );
        s->setSingleStep( stepwidth );
        s->setPageStep( stepwidth );

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
        if (cp->type == SLIDE_INT_PARA)
            sp->value = val;
        else {
            if (val % 2)            // in zweiter Schritten arbeiten 1, 3, 5, ...
                sp->value = val;
        }
        l->setText(QString("%1=%2  min=%3  max=%4").arg(QString(cp->para_name))
                                                    .arg(QString::number(sp->value))
                                                    .arg(QString::number(sp->min))
                                                    .arg(QString::number(sp->max)));
        }
        break;
    case SLIDE_DOUBLE_PARA: {
        struct _double_para_ *sp = (struct _double_para_ *)cp->data;
        sp->value = val;
        l->setText(QString("%1=%2  min=%3  max=%4").arg(QString(cp->para_name))
                                                    .arg(QString::number(sp->value))
                                                    .arg(QString::number(sp->min))
                                                    .arg(QString::number(sp->max)));
        }
        break;
    }

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
    x_koor->setMinimum( -1000 );
    x_koor->setMaximum( 1000 );
    x_koor->setValue( val->x );
    x_koor->setGeometry(x, y+20, 100, 30);
    x_koor->setParent( parent );

    y_koor = new QSpinBox();
    y_koor->setMinimum( -1000 );
    y_koor->setMaximum( 1000 );
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
                while (cp != NULL) {
                    memcpy (cp->data, cp->reset_data, MAX_PARA_DATA);
                    parawin->rewrite_para_data( cp );
                    cp = cp->next;
                }                

                parawin->close_parawin();                   // Parameter Fenster schliessen
                // parawin->close();
                parawin = NULL;
                if (parawin == NULL) {
                    QTreeWidgetItem *i = (QTreeWidgetItem *)cf->para_pointer;
                    if (i != NULL)
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
