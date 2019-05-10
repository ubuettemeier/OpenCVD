#define VERSION "v0.1"

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

QDir home_dir;
QDir current_dir;
QDir data_dir;
QDir icon_dir;

//!
//! \brief MainWindow::MainWindow
//! \param parent
//!
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Server");
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

    QTimer *timer = new QTimer ( this );
    connect ( timer, SIGNAL (timeout()), this, SLOT(trigger_timer()));
    timer->start( 1000 );

    iconlist.push_back(QIcon(icon_dir.path()+"/func_on.ico"));              // OK_ICON  s. auch <enum _icon_name_>
    iconlist.push_back(QIcon(icon_dir.path()+"/func_off.ico"));             // PAUSE_ICON
    iconlist.push_back(QIcon(icon_dir.path()+"/eigenschaft_icon.ico"));     // EIGENSCHAFT_ICON
    iconlist.push_back(QIcon(icon_dir.path()+"/check_in_icon.ico"));        // CHECK_IN_ICON

    aktiv_icon.push_back(QIcon(icon_dir.path()+"/aktiv_0.ico"));
    aktiv_icon.push_back(QIcon(icon_dir.path()+"/aktiv_1.ico"));
    aktiv_icon.push_back(QIcon(icon_dir.path()+"/aktiv_2.ico"));
    aktiv_icon.push_back(QIcon(icon_dir.path()+"/aktiv_3.ico"));
}

//!
//! \brief MainWindow::trigger_timer
//!
void MainWindow::trigger_timer ( void )
{
    struct _cvd_func_ *foo = first_func;

    while (foo != NULL) {
        QTreeWidgetItem *t = (QTreeWidgetItem *)foo->tree_pointer;      // Tree Eintrag besorgen.
        if (t != NULL) {            
            if (foo->func_is_modifyt != 0)
                t->setTextColor(0, QColor("red"));
            else
                t->setTextColor(0, QColor("black"));
        }
        foo = foo->next;
    }
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
                            cf->aktiv_icon = (cf->aktiv_icon < 6) ? cf->aktiv_icon+1 : 0;
                            QTreeWidgetItem *tw = (QTreeWidgetItem *)cf->tree_pointer;
                            tw->setIcon(0, aktiv_icon[ (cf->aktiv_icon < 4) ? cf->aktiv_icon : 7-cf->aktiv_icon ]);
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
                    clear_system();     // ???
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
//!        Extra / set CVD OFF
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
//!        Extra / all Function OFF
//!
void MainWindow::on_actionset_all_Function_OFF_triggered()
{
    struct _cvd_func_ *foo = first_func;

    while (foo != NULL) {
        if (foo->func_off != NULL) {
            if (foo->state.flag.func_off == 0) {
                foo->state.flag.func_off = 1;
                QTreeWidgetItem *item = (QTreeWidgetItem *)foo->func_off;
                item->setIcon(0, iconlist[OK_ICON]);
                item->setTextColor(0, QColor("red"));
                write_state( foo );     // client benachrichtigen !
            }
        }
        foo = foo->next;
    }
}

//!
//! \brief MainWindow::on_actionset_all_Function_ON_triggered
//!        Extra / all Function ON
//!
void MainWindow::on_actionset_all_Function_ON_triggered()
{
    struct _cvd_func_ *foo = first_func;

    while (foo != NULL) {
        if (foo->func_off != NULL) {
            if (foo->state.flag.func_off != 0) {
                foo->state.flag.func_off = 0;
                QTreeWidgetItem *item = (QTreeWidgetItem *)foo->func_off;
                item->setIcon(0, QIcon());
                item->setTextColor(0, QColor("black"));
                write_state  ( foo );
            }
        }
        foo = foo->next;
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

void MainWindow::on_actionSource_Window_schlie_en_triggered()
{
    delete sourcewin;
    set_all_source_icon (false);
}
