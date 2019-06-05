#ifndef PARAWIN_H
#define PARAWIN_H

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QTcpSocket>
#include <QDomDocument>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>

#include "mainwindow.h"
#include "opencvd_types.hpp"

enum mButton_type {
    mCLOSE = 0,
    mRESET = 1,
};

struct _m_button_ {
    char text[30];
    int width;
} const m_button[2] = {{"Close", 60},
                       {"Reset", 60}};

//!
//! \brief The ParaWin class
//!
class ParaWin : public QWidget
{
    Q_OBJECT
public:
    explicit ParaWin(MainWindow *main_win = NULL, QWidget *parent = nullptr);
    explicit ParaWin(QTcpSocket *c, struct _cvd_func_ *cf, MainWindow *main_win, QWidget *parent = nullptr);
    ~ParaWin();
    void set_param_win(int y_pos, int width);
    void close_parawin( void );
    int rewrite_para_data (struct _cvd_para_ *foo);

public:
    struct _cvd_func_ *cf;
    // struct _cvd_para_ *cp;      // wofür wird cp benötigt ???
    QTcpSocket *client;

signals:

public slots:
    void closeEvent(QCloseEvent *event);
    void keyPressEvent(QKeyEvent *event);

private:
    MainWindow *mw = NULL;
};

extern ParaWin *parawin;

//!
//! \brief The StringEdit class
//!
class StringEdit : public QWidget {
    Q_OBJECT
public:
    StringEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent);

    struct _cvd_para_ *cp;
    QTcpSocket *client;
    QLabel *out_str;
    QLineEdit *ledit;
    QPushButton *pb;
private slots:
    void ledit_finish();
    void ledit_text_changed( const QString & );
    void pb_pushed();
private:
    void refresh_out_str (QString s);
    bool text_changed = false;
};
//!
//! \brief The RectIntEdit class
//!
class RectIntEdit : public QWidget {
    Q_OBJECT
public:
    RectIntEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent);
    struct _cvd_para_ *cp;
    QTcpSocket *client;
    QLabel *out_str;
    QSpinBox *iedit[4];
private slots:
    void int_rect_finish();
};
//!
//! \brief The IntEdit class
//!
class IntEdit : public QWidget {
    Q_OBJECT
public:
    IntEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent);

    struct _cvd_para_ *cp;
    QTcpSocket *client;
    QLabel *out_str;
    QSpinBox *iedit;
    QPushButton *para_button;

private slots:
    void int_edit_finish();
    void int_edit_para_button_pushed ();    // Eigenschaft
};
//!
//! \brief The FloatEdit class
//!
class FloatEdit : public QWidget {
    Q_OBJECT
public:
    FloatEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent = 0);

    struct _cvd_para_ *cp;
    QTcpSocket *client;
    QLabel *out_str;
    QDoubleSpinBox *dedit;
private slots:
    void float_edit_finish();
};

//!
//! \brief The DoubleEdit class
//!
class DoubleEdit : public QWidget {
    Q_OBJECT
public:
    DoubleEdit (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent = 0);

    struct _cvd_para_ *cp;
    QTcpSocket *client;
    QLabel *out_str;
    QDoubleSpinBox *dedit;
    QPushButton *para_button;
private slots:
    void double_edit_finish();
    void double_edit_para_button_pushed ();    // Eigenschaft
    // void new_double_val (double val);
};

//!
//! \brief The EnumDrop class
//!
class EnumDrop : public QWidget {
    Q_OBJECT
public:
    EnumDrop (QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent = 0);

    struct _cvd_para_ *cp;
    QTcpSocket *client;
    QLabel *out_str;
    QComboBox *drop;
private slots:
    void new_enum_select(const QString &s);
};

//!
//! \brief The Slide class
//!        foo->data ist nach "struct _step_para_" organisiert.
//!
class Slide : public QWidget {
    Q_OBJECT
public:
    Slide(QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent = 0);

    struct _cvd_para_ *cp;
    QSlider *s;
    QTcpSocket *client;
    QPushButton *para_button;
private slots:
    void slide_value_changed (int val);
    void slide_para_button_pushed ();       // Eigenschaft

private:
    QLabel *l;
    uint8_t stepwidth = 1;
};

//!
//! \brief The PointInt class
//!        foo->data ist nach "struct _size_int_" organisiert.
//!
class PointInt : public QWidget {
    Q_OBJECT
public:
    PointInt(QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent = 0);

    struct _cvd_para_ *cp;
    QSpinBox *x_koor;
    QSpinBox *y_koor;
    QTcpSocket *client;
private slots:
    void X_edit_finish();
    void Y_edit_finish();
private:
    void set_Text( int x, int y );
private:
    QLabel *l;
};

//!
//! \brief The ScalarDouble class
//!
class ScalarDouble : public QWidget {
    Q_OBJECT
public:
    ScalarDouble(QTcpSocket *c, struct _cvd_para_ *foo, int x, int y, QWidget *parent = 0);

    struct _cvd_para_ *cp;
    QDoubleSpinBox *e[4];
    QTcpSocket *client;

    void Refresh_Val (int index);

private slots:
    void e0_edit_finish();
    void e1_edit_finish();
    void e2_edit_finish();
    void e3_edit_finish();

private:
    QLabel *l;
};

//!
//! \brief The mButton class
//!
class mButton : public QWidget {
    Q_OBJECT
public:
    mButton(QTcpSocket *c, struct _cvd_func_ *foo, int x, int y, int b_type, QWidget *parent = 0, MainWindow *main_win = NULL);
    // void close_win();

    struct _cvd_func_ *cf;
    QPushButton *push_button;
    QTcpSocket *client;
    int button_type;    

private slots:
    void mpb_pushed();      // close OR reset pushed
private:
    MainWindow *mw = NULL;
};

#endif // PARAWIN_H
