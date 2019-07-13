//!
//! \file   mainwindow.cpp
//! \author Ulrich Buettemeier
//! \mainclass MainWindow
//!
//! \see decimal separartor: in bash eistellen.
//!      export LC_NUMERIC="en_US.UTF-8"
//!      locale
//!
//! \todo - Anzahl Nachkommastellen bei class DoubleEdit als Parameter festlegen
//!       - close Parameter-Window, by Parameter-Node clicked.
//!       - bei client close Menue: all Functio ON/OFF auf ON setzen !!!
//!

#define VERSION "v0.5"

#include <cstring>
#include <iostream>
#include <unistd.h>
#include <thread>

// #include "dialog.h"
#include "sourcewin.h"
#include "parawin.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;

QDomDocument doc;           // XML doc global. Fuer alle classen sichtbar
#ifdef USE_PARAM_XML
    QDomDocument para;          // XML Parameter
#endif

QDir home_dir;
QDir current_dir;
QDir data_dir;
QDir icon_dir;

MainWindow *glob_mw = NULL;

//!
//! \brief MainWindow::MainWindow
//! \param parent
//!
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    glob_mw = this;
    this->setWindowTitle("OpenCVD");
    this->setGeometry(0, 0, 220, 760);        

    tcpServer = new QTcpServer(this);
    if (!tcpServer->listen(QHostAddress::Any, 51717)) {
        close();
        return;
    }
    connect (tcpServer, SIGNAL(newConnection()), this, SLOT(new_connect()));

    get_sys_path();

    QFile file( data_dir.path()+"/enum.xml" );
    if (!file.open(QIODevice::ReadOnly))        // open
        ui->textEdit->insertPlainText("File enum.xml nicht gefunden\n");
    else {
        ui->textEdit->insertPlainText("enum.xml geöffnet\n");
        if (doc.setContent( &file )) {
            ui->textEdit->insertPlainText("enum.xml angelegt\n");
        }
        file.close();
    }

#ifdef USE_PARAM_XML
    QFile param_file( data_dir.path()+"/param.xml" );
    if (!param_file.open(QIODevice::ReadOnly))        // open
        ui->textEdit->insertPlainText("File param.xml nicht gefunden\n");
    else {
        ui->textEdit->insertPlainText("param.xml geöffnet\n");
        if (para.setContent( &param_file )) {
            ui->textEdit->insertPlainText("param.xml angelegt\n");
        }
        param_file.close();
    }
    // check_param_list ();    // it's a test
#endif

    QTimer *timer = new QTimer ( this );
    connect ( timer, SIGNAL (timeout()), this, SLOT(trigger_timer()));
    timer->start( 1000 );

    get_icon();     // load icon's from icon_dir
}

//!
//! \brief MainWindow::trigger_timer 1000ms
//!
void MainWindow::trigger_timer ( void )
{
    struct _cvd_func_ *foo = first_func;

    while (foo != NULL) {
        QTreeWidgetItem *t = (QTreeWidgetItem *)foo->tree_pointer;      // Tree Eintrag für Funktionsnamen besorgen.
        if (t != NULL) {            
            if (foo->func_is_modifyt != 0)                              // Sind etweigige parameter verändert ?
                t->setTextColor(0, QColor("red"));
            else
                t->setTextColor(0, QColor("black"));
        }
        foo = foo->next;
    }
}

//!
//! \brief MainWindow::get_icon
//!
void MainWindow::get_icon ()
{
    iconlist.push_back(QIcon(icon_dir.path()+"/func_on.ico"));              // OK_ICON  s. auch <enum _icon_name_>
    iconlist.push_back(QIcon(icon_dir.path()+"/func_off.ico"));             // PAUSE_ICON
    iconlist.push_back(QIcon(icon_dir.path()+"/eigenschaft_icon.ico"));     // EIGENSCHAFT_ICON
    iconlist.push_back(QIcon(icon_dir.path()+"/check_in_icon.ico"));        // CHECK_IN_ICON
    iconlist.push_back(QIcon(icon_dir.path()+"/alarm.ico"));                // ALARM = 4

    aktiv_icon.push_back(QIcon(icon_dir.path()+"/aktiv_0.ico"));
    aktiv_icon.push_back(QIcon(icon_dir.path()+"/aktiv_1.ico"));
    aktiv_icon.push_back(QIcon(icon_dir.path()+"/aktiv_2.ico"));
    aktiv_icon.push_back(QIcon(icon_dir.path()+"/aktiv_3.ico"));
}

//!
//! \brief MainWindow::get_sys_path
//!
#define SHOW_GET_PATH_RESULT_
void MainWindow::get_sys_path()
{
    home_dir = QDir(QCoreApplication::applicationDirPath());
    current_dir = QDir(QDir::currentPath());

    QDir foo(home_dir.path());
    foo.cdUp();                             // Basisverzeichniss
    data_dir = QDir(foo.path() + "/data");
    icon_dir = QDir(foo.path() + "/icon");

#ifdef SHOW_GET_PATH_RESULT
    qDebug() << "home_dir: " << home_dir.path() << " " << home_dir.exists();
    qDebug() << "current_dir: " << current_dir.path() << " " << home_dir.exists();
    qDebug() << "data_dir: " << data_dir.path() << " " << data_dir.exists();
    qDebug() << "icon_dir: " << icon_dir.path() << " " << icon_dir.exists();
#endif

}

//!
//! \brief MainWindow::~MainWindow
//!
MainWindow::~MainWindow()
{
    qDebug() << "~MainWindow\n";
    delete ui;
}

//!
//! \brief MainWindow::write_data
//! \param data
//! \param len
//! \return
//!
int MainWindow::write_data (const char *data, uint32_t len)
{
    int anz = 0;
    if (client) {
        anz = client->write((const char *)data, len);
        client->flush();
        client->waitForBytesWritten(3000);
    }

    return anz;
}

//!
//! \brief MainWindow::get_level
//!        Funktion ermittelt den QTreeWidgetItem Level.
//!        0 = oberster Level, dh. 1.Element der Struktur
//! \param item
//! \return
//!
int MainWindow::get_level (QTreeWidgetItem *item)
{
    int n = 0;

    while ((item = item->parent()) != NULL)
        n++;

    return n;
}

//!
//! \brief MainWindow::on_actionBeenden_triggered
//!
void MainWindow::on_actionBeenden_triggered()
{    
    close();
}

//!
//! \brief MainWindow::new_connect
//!
void MainWindow::new_connect()
{
    client = tcpServer->nextPendingConnection();
    connect (client, SIGNAL(readyRead()), this, SLOT(client_read_ready()));
    connect (client, SIGNAL(disconnected()), this, SLOT(client_discontect()));
}

//!
//! \brief MainWindow::client_read_ready
//!
void MainWindow::client_read_ready()
{
    uint32_t *len;
    uint16_t *bef;

    QByteArray buf = client->readAll();    // Daten einlesen

    while (buf.size() >= 4) {
        len = (uint32_t*)buf.data();
        if ((uint32_t)buf.size() >= *len) {
            bef = (uint16_t*)(buf.data()+4);
            // printf ("bef = %4X\n", *bef);
            // ---------------------------------------------------------------------------------------------------
            if ((*bef >= 0x1000) && (*bef <= 0x1FFF)) {     // func data
                struct _func_data_transfer_ foo;
                memcpy (&foo, buf.data(), sizeof(struct _func_data_transfer_));   // gelesene Daten kopieren
                // std::cout << "func " << std::hex << *bef << " " << foo.func_name << endl;
                new_func (&foo);                                        // Daten anlegen !
            }
            // ---------------------------------------------------------------------------------------------------
            if ((*bef >= 0x2000) && (*bef <= 0x2FFF)) {     // para data
                struct _para_data_transfer_ foo;
                memcpy (&foo, buf.data(), sizeof(struct _para_data_transfer_));   // gelesene Daten kopieren                
                struct _cvd_func_ *cf = grep_func_addr( foo.func_addr );    // Funktion finden
                if (cf) {
                    new_para (cf, &foo);
                }
                // std::cout << "para " << std::hex << *bef << " " << foo.para_name << endl;
            }
            // ---------------------------------------------------------------------------------------------------
            if ((*bef >= 0x3000) && (*bef <= 0x3FFF)) {     // special data                
                switch (*bef) {
                case TIME_TRIGGER: {
                        struct _time_trigger_ tt;
                        memcpy (&tt, buf.data(), sizeof(struct _time_trigger_));
                        struct _cvd_func_ *cf = grep_func_addr( tt.func_addr );    // Funktion finden
                        // std::cout << "special message: TIME_TRIGGER: " << cf->func_name << std::endl;

                        if (cf) {
                            cf->aktiv_icon = (cf->aktiv_icon < 6) ? cf->aktiv_icon+1 : 0;                                   // calc icon number
                            QTreeWidgetItem *tw = (QTreeWidgetItem *)cf->tree_pointer;                                      // get tree pointer
                            if (tt.error_flag == 0) {                                                                       // no ERROR detected
                                tw->setIcon(0, aktiv_icon[ (cf->aktiv_icon < 4) ? cf->aktiv_icon : 7-cf->aktiv_icon ]);     // set icon
                            } else {
                                tw->setIcon(0, iconlist[ ALARM ]);      // ERROR: Alaram Icon anzeigen
                            }
                        }
                    }
                    break;
                default:
                    std::cout << "special message: 0x" << std::hex << *bef << endl;
                    break;
                }
            }
            // ---------------------------------------------------------------------------------------------------
            if (*bef >= 0xF000) {                          // system data                
                switch (*bef) {
                case SOCKET_ACK:
                    printf ("ACK\n");
                    ack_detected = 1;
                    break;
                case CLOSE_CLIENT:
                    std::cout << "system message: CLOSE_CLIENT" << std::endl;
                    clear_system();
                    break;
                case SET_CV_VERSION:    // 0xF004
                    struct _cvd_string_ cv;
                    memcpy (&cv, buf.data(), sizeof(struct _cvd_string_));
                    printf ("%s\n", cv.val);
                    ui->textEdit->insertPlainText(QString(cv.val));
                    ui->textEdit->insertPlainText("\n");
                    break;
                default:
                    std::cout << "system message: 0x" << std::hex << *bef << std::endl;
                    break;
                }
            } // if (*bef >= 0xF000)
        } // if ((uint32_t)buf.size() >= *len)
        buf.remove(0, *len);            // Stream: cut the first bytes
    } // while (buf.size() >= 4)
}

//!
//! \brief MainWindow::client_discontect
//!
void MainWindow::client_discontect()
{
    printf ("client disconnect\n");
    if (sourcewin != NULL)
        delete sourcewin;

    sourcewin = NULL;

    if (parawin != NULL)
        delete parawin;

    parawin = NULL;

    clear_system();
    client = NULL;
}

//!
//! \brief MainWindow::clear_system
//!
void MainWindow::clear_system()
{
    // on_actionAlle_Fanster_ausblenden_triggered();
    kill_all_func();                // Alle Funktionen loeschen
    printf ("clear system\n");
    ui->actionCVD_OFF->setChecked( false );
    ui->textEdit->clear();
    ui->treeWidget->clear();
    ui->actionOpenCv_Version->setEnabled( false );
}

//!
//! \brief MainWindow::new_func
//! \param cf
//! \return
//!
struct _cvd_func_ * MainWindow::new_func (struct _func_data_transfer_ *cf)
{
    // printf ("--- new_func\n");
    struct _cvd_func_ *foo = (struct _cvd_func_ *) malloc (sizeof(struct _cvd_func_));

    foo->len = cf->len;
    foo->type = cf->type;
    foo->func_addr = cf->func_addr;
    foo->line_nr = cf->line_nr;
    strcpy (foo->func_name, cf->func_name);
    strcpy (foo->filename, cf->file_name);
    foo->state.val = cf->state.val;
    foo->aktiv_icon = 0;
    foo->func_is_modifyt = 0;

    foo->first_para = foo->end_para = NULL;
    // Level 0  func name
    QTreeWidgetItem *tw = new QTreeWidgetItem(ui->treeWidget);
    tw->setText(0, QString("%1 %2").arg(QString::number(foo->line_nr)).arg(QString(foo->func_name)));
    tw->setIcon(0, aktiv_icon[ foo->aktiv_icon ]);
    foo->tree_pointer = (void *)tw;

    // Level 1 Parameter
    if (cf->state.flag.use_parameter == 0) {        // 0x01
        foo->para_pointer =NULL;
    } else {
        QTreeWidgetItem *pa = new QTreeWidgetItem();
        pa->setText(0, QString("Parameter"));       // Parameter
        tw->addChild( pa );
        foo->para_pointer = (void *)pa;
    }

    // Level 1 Function ON/OFF
    if (cf->state.flag.func_off == 0) {             // 0x02
        foo->func_off = NULL;
    } else {
        QTreeWidgetItem *fo = new QTreeWidgetItem();
        fo->setText(0, QString("Function OFF"));    // func_off
        tw->addChild( fo );
        foo->func_off = (void *)fo;
    }

    // Level 1 Show Image
    if (cf->state.flag.show_image == 0) {           // 0x04
        foo->show_image = NULL;
    } else {
        QTreeWidgetItem *si = new QTreeWidgetItem();
        si->setText(0, QString("Show Image"));      // show Image
        tw->addChild( si );
        foo->show_image = (void *)si;
    }

    if (cf->state.flag.func_break == 0) {           // 0x08
        foo->break_func = NULL;
    } else {
        QTreeWidgetItem *fb = new QTreeWidgetItem();
        fb->setText(0, QString("Break"));           // Break
        tw->addChild( fb );
        foo->break_func = (void *)fb;
    }

    if (strlen(foo->filename) == 0)
        foo->source_pointer = NULL;
    else {        
        QFile qf(foo->filename);            // Prüfen, ob File existiert !
        if (!qf.exists()) {                 // file not found
            foo->source_pointer = NULL;
        } else {
            QTreeWidgetItem *fb = new QTreeWidgetItem();
            fb->setText(0, QString("Source"));              // Source
            tw->addChild( fb );
            foo->source_pointer = (void *)fb;
        }
    }

    foo->state.val = 0x0000;

    foo->next = foo->prev = NULL;
    if (first_func == NULL) first_func = last_func = foo;
    else {
        last_func->next = foo;
        foo->prev = last_func;
        last_func = foo;
    }

    ui->actionOpenCv_Version->setEnabled( true );

    return foo;
}

//!
//! \brief MainWindow::kill_func
//! \param foo
//!
void MainWindow::kill_func (struct _cvd_func_ *foo)
{
    if (foo) {
        if (foo->next != NULL) foo->next->prev = foo->prev;
        if (foo->prev != NULL) foo->prev->next = foo->next;
        if (foo == last_func) last_func = foo->prev;
        if (foo == first_func) first_func = foo->next;

        free (foo);
    }
}

//!
//! \brief MainWindow::kill_all_func
//!
void MainWindow::kill_all_func ()
{
    while (first_func != NULL)
        kill_func (first_func);
}

//!
//! \brief MainWindow::grep_func_addr
//! \param addr
//! \return
//!
struct _cvd_func_ *MainWindow::grep_func_addr (uint64_t addr)
{
    struct _cvd_func_ *foo = first_func;

    while ((foo != NULL) && (foo->func_addr != addr))
        foo = foo->next;

    return foo;
}

//!
//! \brief MainWindow::new_para
//! \param cf
//! \param cp
//! \return
//!
#define CREATE_EINZEL_PARAMETER_
struct _cvd_para_ *MainWindow::new_para (struct _cvd_func_ *cf, struct _para_data_transfer_ *cp)
{
    if (cf == NULL)
       return NULL;

    struct _cvd_para_ *foo = (struct _cvd_para_ *) malloc (sizeof(struct _cvd_para_));
    foo->len = cp->len;
    foo->type = cp->type;
    foo->func_addr = cp->func_addr;
    foo->para_id = cp->para_id;
    foo->flags = 0; // cp->flags;
    strcpy (foo->para_name, cp->para_name);
    memcpy (foo->data, cp->data, MAX_PARA_DATA);
    memcpy (foo->reset_data, cp->data, MAX_PARA_DATA);

#ifdef CREATE_EINZEL_PARAMETER
    QTreeWidgetItem *tw = new QTreeWidgetItem();
    tw->setText(0, QString(foo->para_name));
    QTreeWidgetItem *pw = (QTreeWidgetItem *)cf->para_pointer;  // Pointer vom Eintrag Parameter holen !
    pw->addChild( tw );                                         // Als Paramter-Child anlegen !
    foo->tree_pointer = (void*)tw;
#else
    foo->tree_pointer = NULL;
#endif

    foo->next = foo->prev = NULL;
    if (cf->first_para == NULL) cf->first_para = cf->end_para = foo;
    else {
        cf->end_para->next = foo;
        foo->prev = cf->end_para;
        cf->end_para = foo;
    }

    return foo;
}

//!
//! \brief MainWindow::grep_func_by_para_pointer
//! \param item
//! \return
//!
struct _cvd_func_ *MainWindow::grep_func_by_para_pointer (QTreeWidgetItem *item)
{
    struct _cvd_func_ *foo = first_func;

    while ((foo != NULL) && (foo->para_pointer != item))
        foo = foo->next;

    return foo;
}

//!
//! \brief MainWindow::grep_func_by_func_off_pointer
//! \param item
//! \return
//!
struct _cvd_func_ *MainWindow::grep_func_by_func_off_pointer (QTreeWidgetItem *item)
{
    struct _cvd_func_ *foo = first_func;

    while ((foo != NULL) && (foo->func_off != item))
        foo = foo->next;

    return foo;
}

//!
//! \brief MainWindow::grep_func_by_show_image_pointer
//! \param item
//! \return
//!
struct _cvd_func_ *MainWindow::grep_func_by_show_image_pointer (QTreeWidgetItem *item)
{
    struct _cvd_func_ *foo = first_func;

    while ((foo != NULL) && (foo->show_image != item))
        foo = foo->next;

    return foo;
}

//!
//! \brief MainWindow::grep_func_by_break_func_pointer
//! \param item
//! \return
//!
struct _cvd_func_ *MainWindow::grep_func_by_break_func_pointer (QTreeWidgetItem *item)
{
    struct _cvd_func_ *foo = first_func;

    while ((foo != NULL) && (foo->break_func != item))
        foo = foo->next;

    return foo;
}

//!
//! \brief MainWindow::grep_func_by_source_pointer
//! \param item
//! \return
//!
struct _cvd_func_ *MainWindow::grep_func_by_source_pointer (QTreeWidgetItem *item)         // Source
{
    struct _cvd_func_ *foo = first_func;

    while ((foo != NULL) && (foo->source_pointer != item))
        foo = foo->next;

    return foo;
}

//!
//! \brief MainWindow::grep_para_by_tree_pointer
//! \param item
//! \return
//!
struct _cvd_para_ *MainWindow::grep_para_by_tree_pointer (QTreeWidgetItem *item)
{
    struct _cvd_para_ *ret = NULL;
    struct _cvd_func_ *cf = first_func;
    struct _cvd_para_ *cp;

    while ((cf != NULL) && (ret == NULL)) {
        cp = cf->first_para;
        while ((cp != NULL) && (ret == NULL)) {
            if (cp->tree_pointer == (void*)item)
                ret = cp;
            else
                cp = cp->next;
        }
        cf = cf->next;
    }

    return ret;
}

//!
//! \brief MainWindow::closeEvent
//! \param event
//!
void MainWindow::closeEvent(QCloseEvent *event)
{
    Q_UNUSED (event);

    ui->actionCVD_OFF->setChecked( false );
    printf ("Prog will by closed\n ");

    struct _cvd_header_ h;
    h.len = sizeof (struct _cvd_header_);
    h.bef = CLOSE_SERVER;

    write_data ((const char *)&h, sizeof(struct _cvd_header_));
    // client->write((const char *)&h, sizeof(struct _cvd_header_));
    // client->flush();

    for (int i=0; i<5000; i++)
        usleep (100);

    if (parawin != NULL)
        delete parawin;     // close Parameter Window

    parawin = NULL;
}

//!
//! \brief MainWindow::write_state
//!        state wird and den client uebertragen.
//! \param cf
//!
void MainWindow::write_state (struct _cvd_func_ *cf)
{
    struct _cvd_flags_ cfl;
    cfl.len = sizeof(struct _cvd_flags_);
    cfl.type = FUNC_FLAGS;
    cfl.func_addr = cf->func_addr;
    cfl.state = cf->state;

    write_data((const char *)&cfl, sizeof(struct _cvd_flags_));
}

//!
//! \brief MainWindow::on_treeWidget_itemDoubleClicked
//!        tree double click
//! \param item
//! \param column
//!
void MainWindow::on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column)
{
    Q_UNUSED (*item);
    Q_UNUSED (column);
}

//!
//! \brief MainWindow::on_treeWidget_itemClicked
//!        tree click
//! \param item
//! \param column
//!
void MainWindow::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
{
    Q_UNUSED (column);
    struct _cvd_func_ *cf = NULL;

    int n = get_level( item );    

    if (n == 1) {
        cf = grep_func_by_para_pointer ( item );            // Parameter
        if (cf) {
            if (parawin != NULL) {          // es ist ein Parameter-Fenster offen !
                if (parawin->cf != cf) {
                    QTreeWidgetItem *i = (QTreeWidgetItem *)parawin->cf->para_pointer;
                    if (i != NULL)                        
                        i->setIcon(0, QIcon());
                    delete parawin;
                    parawin = NULL;
                }
            }

            if (parawin == NULL) {
                if (item != NULL)
                    item->setIcon(0, iconlist[OK_ICON]);
                parawin = new ParaWin(client, cf, this);      // neues Parameter-Fenster oeffnen.
            }
            return;
        }

        cf = grep_func_by_func_off_pointer( item );         // Function ON/OFF
        if ( cf ) {
            cf->state.flag.func_off = (cf->state.flag.func_off == 0) ? 1 : 0;
            if (cf->state.flag.func_off == 1) {                
                item->setIcon(0, iconlist[OK_ICON]);
                item->setTextColor(0, QColor("red"));
            } else {
                item->setIcon(0, QIcon());
                item->setTextColor(0, QColor("black"));
            }
            write_state  ( cf );
            return;
        }

        cf = grep_func_by_show_image_pointer( item );       // Show Image
        if ( cf ) {
            cf->state.flag.show_image = (cf->state.flag.show_image) ? 0 : 1;
            if (cf->state.flag.show_image != 0) {
                item->setIcon(0, iconlist[OK_ICON]);        // Show OK Icon
                item->setTextColor(0, QColor("green"));
            } else {
                item->setIcon(0, QIcon());                  // Clear Icon
                item->setTextColor(0, QColor("black"));
            }
            write_state  ( cf );
            return;
        }

        cf = grep_func_by_break_func_pointer( item );       // Break func
        if ( cf ) {
            if (cf->state.flag.func_break == 0) {
                on_actionall_Breakpoint_s_OFF_triggered();     // Alle Break Flags auf 0 setzen !
                cf->state.flag.func_break = 1;
            } else cf->state.flag.func_break = 0;

            if (cf->state.flag.func_break != 0) {           // Break ON
                item->setIcon(0, iconlist[OK_ICON]);        // Show OK Icon
                item->setTextColor(0, QColor("red"));                
            } else {                                        // Break OFF
                item->setIcon(0, QIcon());                  // Clear Icon
                item->setTextColor(0, QColor("black"));
            }
            write_state  ( cf );
            return;
        }

        cf = grep_func_by_source_pointer( item );           // Source Window
        if ( cf ) {
            // Achtung: Die beiden Abfragen 1) und 2) dürfen nicht zusammengefasst/optimiert werden !
            if ((sourcewin != NULL) && (sourcewin->cf != cf)) { // Abfrage 1)
                on_actionSource_Window_schlie_en_triggered();
            }
            if ((sourcewin != NULL) && (sourcewin->cf == cf)) { // Abfrage 2)
                on_actionSource_Window_schlie_en_triggered();
                return;
            }
            if (sourcewin == NULL) {
                item->setIcon(0, iconlist[OK_ICON]);
                sourcewin = new Sourcewin (cf, this);
            }
        } // if ( cf )
    }
}

//!
//! \brief MainWindow::grep_enum_text
//! \param group_name
//! \param enum_val
//! \return
//!
QString MainWindow::grep_enum_text (QString group_name, int enum_val)
{
    QString foo = {};

    QDomNodeList nl = doc.elementsByTagName(group_name);
    if (nl.length()) {                                  // es ist ein Element gefunden worden.
        QDomElement e = nl.at(0).toElement();           // 1.Element verwenden
        QDomElement c = e.firstChild().toElement();     // 1.Child vom 1.Element
        uint8_t ende = 0;
        while ((c.parentNode() == e) && !ende) {
            if (c.text().toInt() == enum_val) {                
                // foo = "cv::"+c.tagName();        // Achtung: tagName ohne cv:: namespace übergeben.
                foo = c.tagName();
                ende = 1;
            } else
                c = c.nextSibling().toElement();
        }
    }
    return foo;
}

//!
//! \brief MainWindow::build_source_line_comment
//! \param cf
//! \return
//! \todo insert CVD_COMMENT
//! \remark
//!
#define CVD_COMMENT "// "

QString MainWindow::build_source_line_comment ( struct _cvd_func_ *cf )
{
    QString s;

    switch (cf->type) {
        case CREATELINESEGMENTDETECTOR: {
            QString bt = grep_enum_text("LineSegmentDetectorModes", *(int*)cf->first_para->data);   // _refine
            s = QString ("// CVD::createLineSegmentDetector(%1, %2, %3, %4, %5, %6, %7, %8);")
                         .arg(bt)                                                                                       // _refine
                         .arg(QString::number(*(double*)cf->first_para->next->data))                                    // _scale
                         .arg(QString::number(*(double*)cf->first_para->next->next->data))                              // _sigma_scale
                         .arg(QString::number(*(double*)cf->first_para->next->next->next->data))                        // _quant
                         .arg(QString::number(*(double*)cf->first_para->next->next->next->next->data))                  // _ang_th
                         .arg(QString::number(*(double*)cf->first_para->next->next->next->next->next->data))            // _log_eps
                         .arg(QString::number(*(double*)cf->first_para->next->next->next->next->next->next->data))      // _density_th
                         .arg(QString::number(*(int*)cf->first_para->next->next->next->next->next->next->next->data));  // _n_bins
            }
            break;
        case CVD_POINT_TYPE_1_FLOAT:
        case CVD_POINT_TYPE_1_DOUBLE: {
            struct _point_double_ *ip = (struct _point_double_ *)cf->first_para->data;
            s = QString ("// CVD::Point(%1, %2);")
                         .arg(QString::number(ip->x))
                         .arg(QString::number(ip->y));
            }
            break;
        case CVD_POINT_TYPE_1_INT64:
        case CVD_POINT_TYPE_1_INT: {
            struct _point_int_ *ip = (struct _point_int_ *)cf->first_para->data;
            s = QString ("// CVD::Point(%1, %2);")
                         .arg(QString::number(ip->x))
                         .arg(QString::number(ip->y));
            }
            break;
        case CVD_SCALAR_2: {
            s = QString ("// CVD::Scalar(%1);")
                        .arg(QString::number(*(double*)cf->first_para->data));
            }
            break;
        case CVD_SCALAR_1: {
            struct _rect_double_ *r = (struct _rect_double_ *)cf->first_para->data;
            s = QString ("// CVD::Scalar(%1, %2, %3, %4);")
                        .arg(QString::number(r->x))
                        .arg(QString::number(r->y))
                        .arg(QString::number(r->w))
                        .arg(QString::number(r->h));
            }
            break;
        case RECTANGLE_2: {
            QString bt = grep_enum_text("LineTypes", *(int*)cf->first_para->next->data);   // closed
            s = QString ("// CVD::rectangle ( img, pt1, pt2, color, %1, %2, %3);")
                         .arg(QString::number(*(int*)cf->first_para->data))
                         .arg(bt)
                         .arg(QString::number(*(int*)cf->first_para->next->next->data));
            }
            break;
        case RECTANGLE_1: {
            QString bt = grep_enum_text("LineTypes", *(int*)cf->first_para->next->data);   // closed
            s = QString ("// CVD::rectangle ( img, rec, color, %1, %2, %3);")
                         .arg(QString::number(*(int*)cf->first_para->data))
                         .arg(bt)
                         .arg(QString::number(*(int*)cf->first_para->next->next->data));
            }
            break;
        case CVD_RECT_TYPE_1_INT: {
            struct _rect_int_ *r = (struct _rect_int_ *)cf->first_para->data;
            s = QString ("// CVD::Rect (%1, %2, %3, %4);")
                        .arg(QString::number(r->x))
                        .arg(QString::number(r->y))
                        .arg(QString::number(r->w))
                        .arg(QString::number(r->h));
            }
            break;
        case CVD_RECT_TYPE_1_FLOAT:
        case CVD_RECT_TYPE_1_DOUBLE: {
            QString func_name;
            func_name = (cf->type == CVD_RECT_TYPE_1_FLOAT) ? "Rect2f" : "Rect2d";
            struct _rect_double_ *r = (struct _rect_double_ *)cf->first_para->data;
            s = QString ("// CVD::%5 (%1, %2, %3, %4);")
                        .arg(QString::number(r->x))
                        .arg(QString::number(r->y))
                        .arg(QString::number(r->w))
                        .arg(QString::number(r->h))
                        .arg(func_name);
            }
            break;
        case STRING_FUNC: {
            s = QString ("// (%1%2%3);  new String")
                        .arg(QChar('"'))
                        .arg(QString((char *)cf->first_para->data))
                        .arg(QChar('"'));
            break;
            }
        case IMREAD: {
            s = QString ("// CVD::imread(%1%2%3);")
                         .arg(QChar('"'))
                         .arg(QString((char *)cf->first_para->data))
                         .arg(QChar('"'));
            break;
            }
        case SET_NUMVAL: {
            QString val;
            if (cf->first_para->type == ENUM_DROP_DOWN)
                val = QString("static_cast<bool>(%1)").arg(QString::number(*(int*)cf->first_para->data));
            if (cf->first_para->type == INT_PARA)
                val = QString("static_cast<int>(%1)").arg(QString::number(*(int*)cf->first_para->data));
            if (cf->first_para->type == DOUBLE_PARA)
                val = QString("static_cast<double>(%1)").arg(QString::number(*(double*)cf->first_para->data));

            s = QString ("// set_val (%1);")
                        .arg(val);
            }
            break;
        case BUILDPYRAMID: {
            QString bt = grep_enum_text("BorderTypes", *(int*)cf->first_para->next->data);   // borderType
            s = QString ("// CVD::buildPyramid (src, dst, %1, %2);")
                        .arg(QString::number(*(int*)cf->first_para->data))
                        .arg(bt);
            }
            break;
        case MAT_ZEROS_3:
        case MAT_ONES_3: {
            QString func_name = "unbekannte Funktion";
            if (cf->type == MAT_ZEROS_2)
                func_name = "zeros";
            if (cf->type == MAT_ONES_2)
                func_name = "ones";

            QString bt = grep_enum_text("ddepth", *(int*)cf->first_para->next->next->next->data);   // closed
            s = QString ("// int sz[2] = {%1, %2};\n// CVD::Mat::%5 (%3, sz, %4);")
                        .arg(QString::number(*(int*)cf->first_para->next->data))
                        .arg(QString::number(*(int*)cf->first_para->next->next->data))
                        .arg(QString::number(*(int*)cf->first_para->data))
                        .arg(bt)
                        .arg(func_name);
            }
            break;
        case MAT_EYE_2:
        case MAT_ZEROS_2:
        case MAT_ONES_2: {
            QString func_name = "unbekannte Funktion";
            if (cf->type == MAT_EYE_2)
                func_name = "eye";
            if (cf->type == MAT_ZEROS_2)
                func_name = "zeros";
            if (cf->type == MAT_ONES_2)
                func_name = "ones";

            struct _point_int_ *ip = (struct _point_int_ *)cf->first_para->data;        // cv::Size
            QString bt = grep_enum_text("ddepth", *(int*)cf->first_para->next->data);   // closed
            s = QString ("// CVD::Mat::%4 (cv::Size(%1, %2), %3);")
                        .arg(QString::number(ip->x))
                        .arg(QString::number(ip->y))
                        .arg(bt)
                        .arg(func_name);
            }
            break;
        case MAT_EYE:
        case MAT_ZEROS:
        case MAT_ONES: {
            QString func_name = "unbekannte Funktion";
            if (cf->type == MAT_EYE)
                func_name = "eye";
            if (cf->type == MAT_ZEROS)
                func_name = "zeros";
            if (cf->type == MAT_ONES)
                func_name = "ones";
            QString bt = grep_enum_text("ddepth", *(int*)cf->first_para->next->next->data);   // closed
            s = QString ("// CVD::Mat::%4 (%1, %2, %3);")
                        .arg(QString::number(*(int*)cf->first_para->data))
                        .arg(QString::number(*(int*)cf->first_para->next->data))
                        .arg(bt)
                        .arg(func_name);
            }
            break;
        case SCALEADD:
            s = QString ("// CVD::scaleADD (src1, %1, src2, dst);")
                         .arg(QString::number(*(double*)cf->first_para->data));     // alpha
            break;
        case MAT_ASSIGNTO:
            s = QString ("// src.assigTo (m,%1);")
                        .arg(QString::number(*(int*)cf->first_para->data));     // type
            break;
        case APPROXPOLYPD: {
            QString bt = grep_enum_text("boolType", *(int*)cf->first_para->next->data);   // closed
            s = QString ("// CVD::approxPolyDP(curve, approxCurve, %1, %2);")
                        .arg(QString::number(*(double*)cf->first_para->data))     // epsilon
                        .arg(bt);
            }
            break;
        case FITLINE: {
            QString bt = grep_enum_text("DistanceTypes", *(int*)cf->first_para->data);   // distType
            s = QString ("// CVD::fitLine(points, line, %1, %2, %3, %4);")
                        .arg(bt)
                        .arg(QString::number(*(double*)cf->first_para->next->data))     //
                        .arg(QString::number(*(double*)cf->first_para->next->next->data))     //
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->data));     //
            }
            break;
        case CORNERHARRIS: {
            QString bt = grep_enum_text("BorderTypes", *(int*)cf->first_para->next->next->next->data);   // borderType
            s = QString ("// CVD::cornerHarris(src, dst, %1, %2, %3, %4);")
                        .arg(QString::number(*(int*)cf->first_para->data))     // blockSize
                        .arg(QString::number(*(int*)cf->first_para->next->data))     // ksize
                        .arg(QString::number(*(double*)cf->first_para->next->next->data))     // k
                        .arg(bt);
            }
            break;
        case PYRUP: {
            struct _point_int_ *ip = (struct _point_int_ *)cf->first_para->data;        // cv::Size
            QString bt = grep_enum_text("BorderTypes", *(int*)cf->first_para->next->data);   // borderType
            s = QString ("// CVD::pyrUp(src, dst, cv::size(%1, %2), %3);")
                    .arg(QString::number(ip->x))
                    .arg(QString::number(ip->y))
                    .arg(bt);
            }
            break;
        case PYRDOWN: {
            struct _point_int_ *ip = (struct _point_int_ *)cf->first_para->data;        // cv::Size
            QString bt = grep_enum_text("BorderTypes", *(int*)cf->first_para->next->data);   // borderType
            s = QString ("// CVD::pyrDown(src, dst, cv::size(%1, %2), %3);")
                    .arg(QString::number(ip->x))
                    .arg(QString::number(ip->y))
                    .arg(bt);
            }
            break;
        case MAT_SIZE_TYPE: {
            struct _point_int_ *ip = (struct _point_int_ *)cf->first_para->data;        // size
            QString bt = grep_enum_text("ddepth", *(int*)cf->first_para->next->data);   // rtype
            s = QString ("// CVD::Mat(cv::size(%1, %2), %3);")
                    .arg(QString::number(ip->x))
                    .arg(QString::number(ip->y))
                    .arg(bt);
            }
            break;
        case MAT_ROWS_COLS_TYPE: {
            QString bt = grep_enum_text("ddepth", *(int*)cf->first_para->next->next->data);   // rtype
            s = QString ("// CVD::Mat(%1, %2, %3);")
                        .arg(QString::number(*(int*)cf->first_para->data))
                        .arg(QString::number(*(int*)cf->first_para->next->data))
                        .arg(bt);
            }
            break;
        case MAT_SIZE_TYPE_SCALAR: {
            struct _point_int_ *ip = (struct _point_int_ *)cf->first_para->data;            // dsize
            QString bt = grep_enum_text("ddepth", *(int*)cf->first_para->next->data);   // rtype
            struct _rect_double_ *r = (struct _rect_double_ *)cf->first_para->next->next->data; // Scalar

            s = QString ("// CVD::Mat(cv::Size(%1 %2), %3, cv::Scalar(%4, %5, %6, %7));")
                        .arg(QString::number(ip->x))
                        .arg(QString::number(ip->y))
                        .arg(bt)
                        .arg(QString::number(r->x))
                        .arg(QString::number(r->y))
                        .arg(QString::number(r->w))
                        .arg(QString::number(r->h));
            }
            break;
        case MAT_ROWS_COLS_TYPE_SCALAR: {
            QString bt = grep_enum_text("ddepth", *(int*)cf->first_para->next->next->data);   // rtype
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->next->next->data);

            struct _rect_double_ *r = (struct _rect_double_ *)cf->first_para->next->next->next->data;

            s = QString ("// CVD::Mat(%1, %2, %3, cv::Scalar(%4, %5, %6, %7));")
                        .arg(QString::number(*(int*)cf->first_para->data))                      // rows
                        .arg(QString::number(*(int*)cf->first_para->next->data))             // cols
                        .arg(bt)
                        .arg(QString::number(r->x))
                        .arg(QString::number(r->y))
                        .arg(QString::number(r->w))
                        .arg(QString::number(r->h));
            }
            break;
        case MAT_CONVERTTO: {
            QString bt = grep_enum_text("Sobel_filterdepth", *(int*)cf->first_para->data);   // rtype
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->data);

            s = QString ("// src.convertTo (dst, %1, %2, %3);")
                        .arg(bt)
                        .arg(QString::number(*(double*)cf->first_para->next->data))           // alpha
                        .arg(QString::number(*(double*)cf->first_para->next->next->data));    // beta
            }
            break;
        case MAT_ROI: {
            struct _rect_int_ *r = (struct _rect_int_ *)cf->first_para->data;
            s = QString ("// CVD::Mat dst(src, cv::Rect(%1, %2, %3, %4);")
                        .arg(QString::number(r->x))
                        .arg(QString::number(r->y))
                        .arg(QString::number(r->w))
                        .arg(QString::number(r->h));
            }
            break;
        case CONVERTSCALEABS:
            s = QString ("// CVD::convertScaleAbs ( src, dst, %1, %2 );")
                        .arg(QString::number(*(double*)cf->first_para->data))           // alpha
                        .arg(QString::number(*(double*)cf->first_para->next->data));    // beta
            break;
        case THRESHOLD: {
            QString bt = grep_enum_text("ThresholdTypes", *(int*)cf->first_para->next->next->data);   // type
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->next->next->data);

            s = QString ("// CVD::threshold ( src, dst, %1, %2, %3 );")
                        .arg(QString::number(*(double*)cf->first_para->data))           // thresh
                        .arg(QString::number(*(double*)cf->first_para->next->data))     // maxval
                        .arg(bt);                                                       // type
            }
            break;
        case ADAPTIVETHRESHOLD: {
            QString bt = grep_enum_text("AdaptiveThresholdTypes", *(int*)cf->first_para->next->data);   // adaptiveMethod
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->next->data);

            QString tt = grep_enum_text("ThresholdTypes_2", *(int*)cf->first_para->next->next->data);   // thresholdType
            if (tt.length() == 0) tt = QString::number(*(int*)cf->first_para->next->next->data);

            s = QString ("// CVD::adaptiveThreshold ( src, dst, %1, %2, %3, %4, %5 );")
                        .arg(QString::number(*(double*)cf->first_para->data))                           // maxValue
                        .arg(bt)              // adaptiveMethod
                        .arg(tt)              // thresholdType
                        .arg(QString::number(*(int*)cf->first_para->next->next->next->data))            // blockSize
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->next->data));  // C
            }
            break;

        case CVTCOLOR: {
            QString bt = grep_enum_text("ColorConversionCodes", *(int*)cf->first_para->data);   // ddepth
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->data);

            s = QString ("// CVD::cvtColor ( src, dst, %1, %2 );")
                        .arg(bt)                                                            // code
                        .arg(QString::number(*(int*)cf->first_para->next->data));           // dstCn
            }
            break;
        case SOBEL: {
            QString bt = grep_enum_text("Sobel_filterdepth", *(int*)cf->first_para->data);          // ddepth
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->data);

            QString bs = grep_enum_text("boolType", *(int*)cf->first_para->next->next->next->next->next->next->data);     // borderType
            if (bt.length() == 0) bs = QString::number(*(int*)cf->first_para->next->next->next->next->next->next->data);

            s = QString ("// CVD::Sobel( src, dst, %1, %2, %3, %4, %5, %6, %7 );")
                        .arg(bt)                                                                    // ddepth
                        .arg(QString::number(*(int*)cf->first_para->next->data))                    // dx
                        .arg(QString::number(*(int*)cf->first_para->next->next->data))              // dy
                        .arg(QString::number(*(int*)cf->first_para->next->next->next->data))              // ksize
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->next->data))              // scale
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->next->next->data))              // delta
                        .arg(bs);
            }
            break;
        case RESIZE: {
            struct _point_int_ *ip = (struct _point_int_ *)cf->first_para->data;            // dsize

            QString bt = grep_enum_text("InterpolationFlags", *(int*)cf->first_para->next->next->next->data);  // interpolation
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->next->next->next->data);

            s = QString ("// CVD::resize( src, dst, cv::Size(%1,%2), %3, %4, %5 );")
                    .arg(QString::number(ip->x))                                                // dsize
                    .arg(QString::number(ip->y))
                    .arg(QString::number(*(double*)cf->first_para->next->data))                 // fx
                    .arg(QString::number(*(double*)cf->first_para->next->next->data))           // fy
                    .arg(bt);                                                                   // interpolation
            }
            break;
        case MEDIANBLUR:
            s = QString("// CVD::medianBlur( src, dst, %2);")
                        .arg(QString(CVD_COMMENT))
                        .arg(QString::number(*(int*)cf->first_para->data));
            break;
        case BLUR_FUNC: {
            struct _point_int_ *ip = (struct _point_int_ *)cf->first_para->next->next->data;            // anchor

            QString bt = grep_enum_text("BorderTypes", *(int*)cf->first_para->next->next->next->data);  // borderType
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->next->next->next->data);

            s = QString ("// CVD::blur( src, dst, cv::Size(%1,%2), cv::Point(%3,%4), %5 );")
                        .arg(QString::number(*(int*)cf->first_para->data))                          // ksize width
                        .arg(QString::number(*(int*)cf->first_para->next->data))                    // ksize height
                        .arg(QString::number(ip->x))                                                // anchor
                        .arg(QString::number(ip->y))
                        .arg(bt);                                                                   // borderType
            }
            break;
        case GAUSSIANBLUR: {
            QString bt = grep_enum_text("BorderTypes", *(int*)cf->first_para->next->next->next->next->data);    // borderType
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->next->next->next->next->data);

            s = QString("// CVD::GaussianBlur (src, dst, cv::Size(%1,%2), %3, %4, %5 );")
                        .arg(QString::number(*(int*)cf->first_para->data))
                        .arg(QString::number(*(int*)cf->first_para->next->data))
                        .arg(QString::number(*(double*)cf->first_para->next->next->data))
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->data))
                        .arg(bt);                                                                   // // borderType
            }
            break;
        case CANNY: {
            QString L2 = grep_enum_text ("boolType", *(int*)cf->first_para->next->next->next->data);        // L2gradient
            if (L2.length() == 0) L2 = QString::number(*(int*)cf->first_para->next->next->next->data);

            s = QString("// CVD::Canny( image, edges, %1, %2, %3, %4 );")
                        .arg(QString::number(*(double*)cf->first_para->data))                               // threshold1
                        .arg(QString::number(*(double*)cf->first_para->next->data))                         // threshold2
                        .arg(QString::number(*(int*)cf->first_para->next->next->data))                      // apertureSize
                        .arg(L2);                                                                           // L2gradient
            }
            break;
        case CANNY_2: {
            QString L2 = grep_enum_text ("boolType", *(int*)cf->first_para->next->next->data);              // L2gradient
            if (L2.length() == 0) L2 = QString::number(*(int*)cf->first_para->next->next->data);

            s = QString("// CVD::Canny( dx, dy, edges, %1, %2, %3 );")
                        .arg(QString::number(*(double*)cf->first_para->data))                               // threshold1
                        .arg(QString::number(*(double*)cf->first_para->next->data))                         // threshold2
                        .arg(L2);
            }
            break;
        case HOUGHLINES:
            s = QString("// CVD::HoughLines( image, lines, %1, %2, %3, %4, %5, %6, %7 );")
                        .arg(QString::number(*(double*)cf->first_para->data))                               // rho
                        .arg(QString::number(*(double*)cf->first_para->next->data))                         // theta
                        .arg(QString::number(*(int*)cf->first_para->next->next->data))                      // threshold
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->data))             // srn
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->next->data))       // stn
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->next->next->data))         // min_theta
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->next->next->next->data));  // max_theta
            break;
        case HOUGHLINESP:
            s = QString("// CVD::HoughLinesP( image, lines, %1, %2, %3, %4, %5 );")
                        .arg(QString::number(*(double*)cf->first_para->data))                               // rho
                        .arg(QString::number(*(double*)cf->first_para->next->data))                         // theta
                        .arg(QString::number(*(int*)cf->first_para->next->next->data))                      // threshold
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->data))             // minLineLength
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->next->data));      // maxLineGap
            break;
        case HOUGHCIRCLES: {
            QString bt = grep_enum_text("HoughModes", *(int*)cf->first_para->data);   // method
            if (bt.length() == 0) bt = QString::number(*(int*)cf->first_para->data);

            s = QString ("// CVD::HoughCircles ( image, circles, %1, %2, %3, %4, %5, %6, %7);")
                        .arg(bt)
                        .arg(QString::number(*(double*)cf->first_para->next->data))                         // dp
                        .arg(QString::number(*(double*)cf->first_para->next->next->data))                         // minDist
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->data)) //
                        .arg(QString::number(*(double*)cf->first_para->next->next->next->next->data)) //
                        .arg(QString::number(*(int*)cf->first_para->next->next->next->next->next->data)) //
                        .arg(QString::number(*(int*)cf->first_para->next->next->next->next->next->next->data));
            }
            break;
        default:
            s = "// unbekannte Funktion";
            break;
    }
    // qDebug() << s << "\n";
    return s;
}

//!
//! \brief MainWindow::refresh_source_win
//! \param cf
//!
void MainWindow::refresh_source_win( _cvd_func_ *cf )
{
    on_actionSource_Window_schlie_en_triggered();
    // item->setIcon(0, iconlist[OK_ICON]);
    sourcewin = new Sourcewin (cf, this);
}

//!
//! \brief MainWindow::set_all_source_icon
//! \param wert
//!
void MainWindow::set_all_source_icon (bool wert)
{
    struct _cvd_func_ *foo = first_func;

    while (foo != NULL) {
        if (foo->source_pointer != NULL) {
            QTreeWidgetItem *i = (QTreeWidgetItem *)foo->source_pointer;
            if (i != NULL) {
                if (wert == false)
                    i->setIcon(0, QIcon());
            }
        }
        foo = foo->next;
    }
}

//!
//! \brief MainWindow::on_actionBaumstruktur_zuklappen_triggered
//!        Baumstruktur zuklappen
//!
void MainWindow::on_actionBaumstruktur_zuklappen_triggered()
{
    ui->treeWidget->collapseAll();
}

//!
//! \brief MainWindow::on_actionSpeichern_triggered
//!        Datei / Speichern
//!
void MainWindow::on_actionSpeichern_triggered()
{
    struct _cvd_func_ *foo = first_func;
    struct _cvd_para_ *p;

    while (foo != NULL) {
        printf ("%s ( ", foo->func_name);
        p = foo->first_para;
        while (p != NULL) {
            if (p != foo->first_para)
                printf (", ");
            printf ("%s", p->para_name);

            p = p->next;
        }
        printf (" )\n");
        foo = foo->next;
    }
}

//!
//! \brief MainWindow::on_actionAlle_Fenster_schli_en_triggered
//!        Ansicht / Alle Fenster schließen
//!
void MainWindow::on_actionAlle_Fenster_schli_en_triggered()
{
    struct _cvd_func_ *foo = first_func;

    while (foo != NULL) {
        if (foo->state.flag.show_image != 0) {
            foo->state.flag.show_image = 0;
            if (foo->show_image != NULL) {
                QTreeWidgetItem *item = (QTreeWidgetItem *)foo->show_image;
                item->setIcon(0, QIcon());
                item->setTextColor(0, QColor("black"));
            }
            write_state  ( foo );
        }
        foo = foo->next;
    }
}

//!
//! \brief MainWindow::write_header_bef
//! \param befehl
//!
void MainWindow::write_header_bef (int befehl)
{
    struct _cvd_header_ h;
    h.len = sizeof (struct _cvd_header_);
    h.bef = befehl;

    write_data((const char *)&h, sizeof(struct _cvd_header_));
}

//!
//! \brief MainWindow::on_actionCVD_OFF_triggered
//!        Extra / set CVD OFF/ON
//!
void MainWindow::on_actionCVD_OFF_triggered()
{
    if (ui->actionCVD_OFF->isChecked())
        write_header_bef (SET_CVD_OFF);
    else
        write_header_bef (SET_CVD_ON);
}

//!
//! \brief MainWindow::on_actionset_all_Function_OFF_triggered
//!        Extra / all Function OFF/ON
//!
void MainWindow::on_actionset_all_Function_OFF_triggered()
{
    struct _cvd_func_ *foo = first_func;

    if (ui->actionset_all_Function_OFF->isChecked()) {
        while (foo != NULL) {
            if (foo->func_off != NULL) {
                if (foo->state.flag.func_off == 0) {
                    foo->state.flag.func_off = 1;
                    QTreeWidgetItem *item = (QTreeWidgetItem *)foo->func_off;
                    item->setIcon(0, iconlist[OK_ICON]);
                    item->setTextColor(0, QColor("red"));
                    write_state ( foo );                    // client benachrichtigen !
                }
            }
            foo = foo->next;
        }
    } else {
        while (foo != NULL) {
            if (foo->func_off != NULL) {
                if (foo->state.flag.func_off != 0) {
                    foo->state.flag.func_off = 0;
                    QTreeWidgetItem *item = (QTreeWidgetItem *)foo->func_off;
                    item->setIcon(0, QIcon());
                    item->setTextColor(0, QColor("black"));
                    write_state  ( foo );                   // client benachrichtigen !
                }
            }
            foo = foo->next;
        }
    }
}

//!
//! \brief MainWindow::on_actionall_Breakpoint_s_OFF_triggered
//!        Extra / all Breakpoint's OFF
//!
void MainWindow::on_actionall_Breakpoint_s_OFF_triggered()
{
    struct _cvd_func_ *foo = first_func;

    while (foo != NULL) {
        if (foo->state.flag.func_break) {
            foo->state.flag.func_break = 0;
            if (foo->state.flag.show_image) {
                foo->state.flag.show_image = 0;
                if (foo->show_image != NULL) {
                    QTreeWidgetItem *item = (QTreeWidgetItem *)foo->show_image;
                    item->setIcon(0, QIcon());                                      // Icon entfernen
                    item->setTextColor(0, QColor("black"));
                }
                write_state( foo );     // client benachrichtigen !
            }
            if (foo->break_func != NULL) {
                QTreeWidgetItem *item = (QTreeWidgetItem *)foo->break_func;     // QTree Item besorgen
                item->setIcon(0, QIcon());                                      // Icon entfernen
                item->setTextColor (0, QColor("black"));                        // Icon Text schwarz
            }
            write_state( foo );     // client benachrichtigen !
        }
        foo = foo->next;
    }
}

//!
//! \brief MainWindow::on_actionAbout_triggered
//!        Help / About
//!
void MainWindow::on_actionAbout_triggered()
{
    char buf[4096];

    sprintf (buf, "OpenCVD Server\n" \
                  "Version %s\n" \
                  "Date: %s, %s",
                  VERSION, __DATE__, __TIME__);

    QMessageBox::information ( this, "About", buf, QMessageBox::Ok );
}

//!
//! \brief MainWindow::on_actionSource_Window_schlie_en_triggered
//!
void MainWindow::on_actionSource_Window_schlie_en_triggered()
{
    delete sourcewin;
    set_all_source_icon (false);
}

//!
//! \brief MainWindow::grep_enum
//! \param enum_name
//! \return  -1  FAILURE
//!
int MainWindow::grep_enum (const char *enum_name)
{
    if (strcmp(enum_name, "INITFUNC") == 0) return INITFUNC;
    if (strcmp(enum_name, "MEDIANBLUR") == 0) return MEDIANBLUR;
    if (strcmp(enum_name, "THRESHOLD") == 0) return THRESHOLD;
    if (strcmp(enum_name, "CVTCOLOR") == 0) return CVTCOLOR;
    if (strcmp(enum_name, "LAPLACIAN") == 0) return LAPLACIAN;
    if (strcmp(enum_name, "CANNY") == 0) return CANNY;
    if (strcmp(enum_name, "CANNY_2") == 0) return CANNY_2;
    if (strcmp(enum_name, "CONVERTTO") == 0) return CONVERTTO;
    if (strcmp(enum_name, "OPERATOR_INT_MUL_EQUAL") == 0) return OPERATOR_INT_MUL_EQUAL;
    if (strcmp(enum_name, "OPERATOR_FLOAT_MUL_EQUAL") == 0) return OPERATOR_FLOAT_MUL_EQUAL;
    if (strcmp(enum_name, "OPERATOR_DOUBLE_MUL_EQUAL") == 0) return OPERATOR_DOUBLE_MUL_EQUAL;
    if (strcmp(enum_name, "GAUSSIANBLUR") == 0) return GAUSSIANBLUR;
    if (strcmp(enum_name, "BLUR_FUNC") == 0) return BLUR_FUNC;
    if (strcmp(enum_name, "SCALAR_FUNC_4") == 0) return SCALAR_FUNC_4;
    if (strcmp(enum_name, "DILATE") == 0) return DILATE;
    if (strcmp(enum_name, "ERODE") == 0) return ERODE;
    if (strcmp(enum_name, "FINDCONTOURS") == 0) return FINDCONTOURS;
    if (strcmp(enum_name, "CONVERTSCALEABS") == 0) return CONVERTSCALEABS;
    if (strcmp(enum_name, "SCALAR_ALL") == 0) return SCALAR_ALL;
    if (strcmp(enum_name, "SCHARR") == 0) return SCHARR;
    if (strcmp(enum_name, "IMREAD") == 0) return IMREAD;
    if (strcmp(enum_name, "GRABCUT") == 0) return GRABCUT;
    if (strcmp(enum_name, "GETSTRUCTURINGELEMENT") == 0) return GETSTRUCTURINGELEMENT;
    if (strcmp(enum_name, "MORPHOLOGYEX") == 0) return MORPHOLOGYEX;
    if (strcmp(enum_name, "NORMALIZE") == 0) return NORMALIZE;
    if (strcmp(enum_name, "CALCHIST") == 0) return CALCHIST;
    if (strcmp(enum_name, "HOUGHCIRCLES") == 0) return HOUGHCIRCLES;
    if (strcmp(enum_name, "HOUGHLINESP") == 0) return HOUGHLINESP;
    if (strcmp(enum_name, "HOUGHLINES") == 0) return HOUGHLINES;
    if (strcmp(enum_name, "GETBUILDINFORMATION") == 0) return GETBUILDINFORMATION;
    if (strcmp(enum_name, "ADAPTIVETHRESHOLD") == 0) return ADAPTIVETHRESHOLD;
    if (strcmp(enum_name, "RESIZE") == 0) return RESIZE;
    if (strcmp(enum_name, "SOBEL") == 0) return SOBEL;
    if (strcmp(enum_name, "PYRDOWN") == 0) return PYRDOWN;
    if (strcmp(enum_name, "PYRUP") == 0) return PYRUP;
    if (strcmp(enum_name, "CORNERHARRIS") == 0) return CORNERHARRIS;
    if (strcmp(enum_name, "FITLINE") == 0) return FITLINE;
    if (strcmp(enum_name, "APPROXPOLYPD") == 0) return APPROXPOLYPD;
    if (strcmp(enum_name, "SCALEADD") == 0) return SCALEADD;
    if (strcmp(enum_name, "BUILDPYRAMID") == 0) return BUILDPYRAMID;
    if (strcmp(enum_name, "RECTANGLE_1") == 0) return RECTANGLE_1;

    if (strcmp(enum_name, "CVD_RECT_TYPE_1_INT") == 0) return CVD_RECT_TYPE_1_INT;
    if (strcmp(enum_name, "CVD_RECT_TYPE_1_FLOAT") == 0) return CVD_RECT_TYPE_1_FLOAT;
    if (strcmp(enum_name, "CVD_RECT_TYPE_1_DOUBLE") == 0) return CVD_RECT_TYPE_1_DOUBLE;


    if (strcmp(enum_name, "CVD_SCALAR_1") == 0) return CVD_SCALAR_1;
    if (strcmp(enum_name, "CVD_SCALAR_2") == 0) return CVD_SCALAR_2;

    if (strcmp(enum_name, "CVD_POINT_TYPE_1_INT") == 0) return CVD_POINT_TYPE_1_INT;
    if (strcmp(enum_name, "CVD_POINT_TYPE_1_INT64") == 0) return CVD_POINT_TYPE_1_INT64;
    if (strcmp(enum_name, "CVD_POINT_TYPE_1_FLOAT") == 0) return CVD_POINT_TYPE_1_FLOAT;
    if (strcmp(enum_name, "CVD_POINT_TYPE_1_DOUBLE") == 0) return CVD_POINT_TYPE_1_DOUBLE;

    if (strcmp(enum_name, "SET_NUMVAL") == 0) return SET_NUMVAL;

    if (strcmp(enum_name, "MAT_ROI") == 0) return MAT_ROI;
    if (strcmp(enum_name, "MAT_CONVERTTO") == 0) return MAT_CONVERTTO;
    if (strcmp(enum_name, "MAT_ROWS_COLS_TYPE_SCALAR") == 0) return MAT_ROWS_COLS_TYPE_SCALAR;
    if (strcmp(enum_name, "MAT_SIZE_TYPE_SCALAR") == 0) return MAT_SIZE_TYPE_SCALAR;
    if (strcmp(enum_name, "MAT_ROWS_COLS_TYPE") == 0) return MAT_ROWS_COLS_TYPE;
    if (strcmp(enum_name, "MAT_SIZE_TYPE") == 0) return MAT_SIZE_TYPE;
    if (strcmp(enum_name, "MAT_ASSIGNTO") == 0) return MAT_ASSIGNTO;
    if (strcmp(enum_name, "MAT_ONES") == 0) return MAT_ONES;
    if (strcmp(enum_name, "MAT_ONES_2") == 0) return MAT_ONES_2;
    if (strcmp(enum_name, "MAT_ONES_3") == 0) return MAT_ONES_3;
    if (strcmp(enum_name, "MAT_ZEROS") == 0) return MAT_ZEROS;
    if (strcmp(enum_name, "MAT_ZEROS_2") == 0) return MAT_ZEROS_2;
    if (strcmp(enum_name, "MAT_ZEROS_3") == 0) return MAT_ZEROS_3;
    if (strcmp(enum_name, "MAT_EYE") == 0) return MAT_EYE;
    if (strcmp(enum_name, "MAT_EYE_2") == 0) return MAT_EYE_2;

    if (strcmp(enum_name, "SLIDE_INT_TWO_STEP_PARA") == 0) return SLIDE_INT_TWO_STEP_PARA;
    if (strcmp(enum_name, "SLIDE_INT_PARA") == 0) return SLIDE_INT_PARA;
    if (strcmp(enum_name, "SLIDE_DOUBLE_PARA") == 0) return SLIDE_DOUBLE_PARA;
    if (strcmp(enum_name, "DOUBLE_PARA") == 0) return DOUBLE_PARA;
    if (strcmp(enum_name, "INT_PARA") == 0) return INT_PARA;
    if (strcmp(enum_name, "ENUM_DROP_DOWN") == 0) return ENUM_DROP_DOWN;
    if (strcmp(enum_name, "FLOAT_PARA") == 0) return FLOAT_PARA;
    if (strcmp(enum_name, "POINT_INT") == 0) return POINT_INT;
    if (strcmp(enum_name, "SCALAR_PARA") == 0) return SCALAR_PARA;
    if (strcmp(enum_name, "STRING_PARA") == 0) return STRING_PARA;
    if (strcmp(enum_name, "POINT_INT_XY") == 0) return POINT_INT_XY;
    if (strcmp(enum_name, "RECT_INT_PARA") == 0) return RECT_INT_PARA;
    if (strcmp(enum_name, "RECT_DOUBLE_PARA") == 0) return RECT_DOUBLE_PARA;

    if (strcmp(enum_name, "FUNC_FLAGS") == 0) return FUNC_FLAGS;
    if (strcmp(enum_name, "TIME_TRIGGER") == 0) return TIME_TRIGGER;
    if (strcmp(enum_name, "SET_CVD_OFF") == 0) return SET_CVD_OFF;
    if (strcmp(enum_name, "SET_CVD_ON") == 0) return SET_CVD_ON;

    if (strcmp(enum_name, "CLOSE_CLIENT") == 0) return CLOSE_CLIENT;
    if (strcmp(enum_name, "CLOSE_SERVER") == 0) return CLOSE_SERVER;
    if (strcmp(enum_name, "SOCKET_ACK") == 0) return SOCKET_ACK;

    return -1;
}

//!
//! \brief MainWindow::get_enum_text
//! \param val
//! \return
//!
char *MainWindow::get_enum_text (int val)
{
    static char buf[64];

    strcpy (buf, "");

    if (val == INITFUNC) strcpy (buf, "INITFUNC");

    if (val == MEDIANBLUR) strcpy (buf, "MEDIANBLUR");
    if (val == THRESHOLD) strcpy (buf, "THRESHOLD");
    if (val == CVTCOLOR) strcpy (buf, "CVTCOLOR");
    if (val == LAPLACIAN) strcpy (buf, "LAPLACIAN");
    if (val == CANNY) strcpy (buf, "CANNY");
    if (val == CANNY_2) strcpy (buf, "CANNY_2");
    if (val == CONVERTTO) strcpy (buf, "CONVERTTO");
    if (val == OPERATOR_INT_MUL_EQUAL) strcpy (buf, "OPERATOR_INT_MUL_EQUAL");
    if (val == OPERATOR_FLOAT_MUL_EQUAL) strcpy (buf, "OPERATOR_FLOAT_MUL_EQUAL");
    if (val == OPERATOR_DOUBLE_MUL_EQUAL) strcpy (buf, "OPERATOR_DOUBLE_MUL_EQUAL");
    if (val == GAUSSIANBLUR) strcpy (buf, "GAUSSIANBLUR");
    if (val == BLUR_FUNC) strcpy (buf, "BLUR_FUNC");
    if (val == SCALAR_FUNC_4) strcpy (buf, "SCALAR_FUNC_4");
    if (val == DILATE) strcpy (buf, "DILATE");
    if (val == ERODE) strcpy (buf, "ERODE");
    if (val == FINDCONTOURS) strcpy (buf, "FINDCONTOURS");
    if (val == CONVERTSCALEABS) strcpy (buf, "CONVERTSCALEABS");
    if (val == SCALAR_ALL) strcpy (buf, "SCALAR_ALL");
    if (val == SCHARR) strcpy (buf, "SCHARR");
    if (val == IMREAD) strcpy (buf, "IMREAD");
    if (val == GRABCUT) strcpy (buf, "GRABCUT");
    if (val == GETSTRUCTURINGELEMENT) strcpy (buf, "GETSTRUCTURINGELEMENT");
    if (val == MORPHOLOGYEX) strcpy (buf, "MORPHOLOGYEX");
    if (val == NORMALIZE) strcpy (buf, "NORMALIZE");
    if (val == CALCHIST) strcpy (buf, "CALCHIST");
    if (val == HOUGHCIRCLES) strcpy (buf, "HOUGHCIRCLES");
    if (val == HOUGHLINESP) strcpy (buf, "HOUGHLINESP");
    if (val == HOUGHLINES) strcpy (buf, "HOUGHLINES");
    if (val == GETBUILDINFORMATION) strcpy (buf, "GETBUILDINFORMATION");
    if (val == ADAPTIVETHRESHOLD) strcpy (buf, "ADAPTIVETHRESHOLD");
    if (val == RESIZE) strcpy (buf, "RESIZE");
    if (val == SOBEL) strcpy (buf, "SOBEL");
    if (val == PYRDOWN) strcpy (buf, "PYRDOWN");
    if (val == PYRUP) strcpy (buf, "PYRUP");
    if (val == CORNERHARRIS) strcpy (buf, "CORNERHARRIS");
    if (val == FITLINE) strcpy (buf, "FITLINE");
    if (val == APPROXPOLYPD) strcpy (buf, "APPROXPOLYPD");
    if (val == SCALEADD) strcpy (buf, "SCALEADD");
    if (val == BUILDPYRAMID) strcpy (buf, "BUILDPYRAMID");
    if (val == RECTANGLE_1) strcpy (buf, "RECTANGLE_1");

    if (val == CVD_RECT_TYPE_1_INT) strcpy (buf, "CVD_RECT_TYPE_1_INT");
    if (val == CVD_RECT_TYPE_1_FLOAT) strcpy (buf, "CVD_RECT_TYPE_1_FLOAT");
    if (val == CVD_RECT_TYPE_1_DOUBLE) strcpy (buf, "CVD_RECT_TYPE_1_DOUBLE");

    if (val == CVD_SCALAR_1) strcpy (buf, "CVD_SCALAR_1");
    if (val == CVD_SCALAR_2) strcpy (buf, "CVD_SCALAR_2");

    if (val == CVD_POINT_TYPE_1_INT) strcpy (buf, "CVD_POINT_TYPE_1_INT");
    if (val == CVD_POINT_TYPE_1_INT64) strcpy (buf, "CVD_POINT_TYPE_1_INT64");
    if (val == CVD_POINT_TYPE_1_FLOAT) strcpy (buf, "CVD_POINT_TYPE_1_FLOAT");
    if (val == CVD_POINT_TYPE_1_DOUBLE) strcpy (buf, "CVD_POINT_TYPE_1_DOUBLE");

    if (val == SET_NUMVAL) strcpy (buf, "SET_NUMVAL");

    if (val == MAT_ROI) strcpy (buf, "MAT_ROI");
    if (val == MAT_CONVERTTO) strcpy (buf, "MAT_CONVERTTO");
    if (val == MAT_ROWS_COLS_TYPE_SCALAR) strcpy (buf, "MAT_ROWS_COLS_TYPE_SCALAR");
    if (val == MAT_SIZE_TYPE_SCALAR) strcpy (buf, "MAT_SIZE_TYPE_SCALAR");
    if (val == MAT_ROWS_COLS_TYPE) strcpy (buf, "MAT_ROWS_COLS_TYPE");
    if (val == MAT_SIZE_TYPE) strcpy (buf, "MAT_SIZE_TYPE");
    if (val == MAT_ASSIGNTO) strcpy (buf, "MAT_ASSIGNTO");
    if (val == MAT_ONES) strcpy (buf, "MAT_ONES");
    if (val == MAT_ONES_2) strcpy (buf, "MAT_ONES_2");
    if (val == MAT_ONES_3) strcpy (buf, "MAT_ONES_3");
    if (val == MAT_ZEROS) strcpy (buf, "MAT_ZEROS");
    if (val == MAT_ZEROS_2) strcpy (buf, "MAT_ZEROS_2");
    if (val == MAT_ZEROS_3) strcpy (buf, "MAT_ZEROS_3");
    if (val == MAT_EYE) strcpy (buf, "MAT_EYE");
    if (val == MAT_EYE_2) strcpy (buf, "MAT_EYE_2");

    if (val == SLIDE_INT_TWO_STEP_PARA) strcpy (buf, "SLIDE_INT_TWO_STEP_PARA");
    if (val == SLIDE_INT_PARA) strcpy (buf, "SLIDE_INT_PARA");
    if (val == SLIDE_DOUBLE_PARA) strcpy (buf, "SLIDE_DOUBLE_PARA");
    if (val == DOUBLE_PARA) strcpy (buf, "DOUBLE_PARA");
    if (val == INT_PARA) strcpy (buf, "INT_PARA");
    if (val == ENUM_DROP_DOWN) strcpy (buf, "ENUM_DROP_DOWN");
    if (val == FLOAT_PARA) strcpy (buf, "FLOAT_PARA");
    if (val == POINT_INT) strcpy (buf, "POINT_INT");
    if (val == SCALAR_PARA) strcpy (buf, "SCALAR_PARA");
    if (val == STRING_PARA) strcpy (buf, "STRING_PARA");
    if (val == POINT_INT_XY) strcpy (buf, "POINT_INT_XY");
    if (val == RECT_INT_PARA) strcpy (buf, "RECT_INT_PARA");
    if (val == RECT_DOUBLE_PARA) strcpy (buf, "RECT_DOUBLE_PARA");

    if (val == FUNC_FLAGS) strcpy (buf, "FUNC_FLAGS");
    if (val == TIME_TRIGGER) strcpy (buf, "TIME_TRIGGER");
    if (val == SET_CVD_OFF) strcpy (buf, "SET_CVD_OFF");
    if (val == SET_CVD_ON) strcpy (buf, "SET_CVD_ON");

    if (val == CLOSE_CLIENT) strcpy (buf, "CLOSE_CLIENT");
    if (val == CLOSE_SERVER) strcpy (buf, "CLOSE_SERVER");
    if (val == SOCKET_ACK) strcpy (buf, "SOCKET_ACK");

    return buf;
}

#ifdef USE_PARAM_XML
void MainWindow::check_param_list ()
{
    QDomNodeList nl = para.elementsByTagName("BLUR_FUNC");  // Elemet suchen

    // if (nl.length()) {  // es ist ein Element gefunden worden.
    if (!nl.isEmpty()) {
        // qDebug() << "size=" << nl.length();
        QDomElement e = nl.at(0).toElement();           // 1.Element verwenden
        qDebug() << e.tagName();
        QDomElement c = e.firstChild().toElement();     // 1.Child vom 1.Element
        int n = 0;

        while (c.parentNode() == e) {                   // Parameterliste abarbeiten.
            n++;
            if (c.tagName() == "parameter") {           // parameter gefunden
                if (c.hasAttributes()) {
                    std::cout << "Name=" << c.attributeNode("Name").nodeValue().toStdString() << "\n";
                    // QString dummy = QString::fromLocal8Bit(c.attributeNode("showtype").nodeValue().toStdString().c_str());
                    // std::cout << "dummy=" << c.attributeNode("showtype").nodeValue().toStdString().c_str() << "\n";
                    // qDebug() << "dummy=" << dummy;
                    int type = grep_enum(c.attributeNode("showtype").nodeValue().toStdString().c_str());
                    // std::cout << "showtype=" << std::hex << std::uppercase << type << "\n";
                    std::cout << "showtype=" << type << "\n";
                    std::cout << "type=" << c.attributeNode("type").nodeValue().toStdString() << "\n";
                    std::cout << "min=" << c.attributeNode("min").nodeValue().toStdString() << "\n";
                    std::cout << "max=" << c.attributeNode("max").nodeValue().toStdString() << "\n";
                }
                std::cout << "\n";
            }
            c = c.nextSibling().toElement();            // nächster Parameter
        }
        qDebug() << n << " Parameter gefunden";
    } else
        qDebug() << "kein Element gefunden";
}
#endif

//!
//! \brief MainWindow::on_actionOpenCv_Version_triggered
//!        Ansicht / OpenCV Version
//!
void MainWindow::on_actionOpenCv_Version_triggered()
{
    struct _cvd_header_ h;
    h.len = sizeof (struct _cvd_header_);
    h.bef = GET_CV_VERSION;

    write_data ((const char *)&h, sizeof(struct _cvd_header_));
}
