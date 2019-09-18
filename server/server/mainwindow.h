#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define USE_PARAM_XML_      // the USE_PARAM_XML flag is not needed

#include <QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QIODevice>
#include <QTreeWidgetItem>
#include <QDialog>
#include <QDomDocument>
#include <QFile>
#include <QVector>
#include <QDir>
#include <QTimer>
#include <QMessageBox>
#include <QFontMetrics>

#include "opencvd_types.hpp"

/* ----------------- Umleitung funktioniert nicht ---------------------
//! @brief  control debug outputs
#define DEBUG_OUTPUTS_                        // controls debug outputs

#ifdef DEBUG_OUTPUTS
    std::ostream &dout = std::cout;         // enable debug outputs to std::cout
#else
    std::ofstream dev_null("/dev/null");    // disenable debug control outputs
    std::ostream &dout = dev_null;
#endif
*/

enum _icon_name_ {
    OK_ICON = 0,            // Haken Zeichen
    PAUSE_ICON = 1,         // Pause Zeichen
    EIGENSCHAFT_ICON = 2,
    CHECK_IN_ICON = 3,
    ALARM = 4
};

extern QDomDocument doc;        // XML Datei
extern QDomDocument para;       // XML Parameter

extern QDir home_dir;
extern QDir data_dir;
extern QDir icon_dir;
extern QDir current_dir;

namespace Ui {
class MainWindow;
}

//!
//! \brief The MainWindow class
//!
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void clear_system();
    int write_data (const char *data, uint32_t len);
    int get_level (QTreeWidgetItem *item);
    bool ack_detected = 0;
    void set_all_source_icon (bool wert);
    void refresh_source_win( _cvd_func_ *cf );

    QString grep_enum_text (QString group_name, int enum_val);      // return the enum text of enum_val. see: enum.xml
    QString build_source_line_comment ( struct _cvd_func_ *cf );    // source einbinden

private:
    void get_sys_path( void );
    void get_icon ( void );

public:
    static uint64_t min_fps_time;
    bool d_is_aktiv;
    QVector<QIcon> iconlist;        // see: enum _icon_name_
    QVector<QIcon> aktiv_icon;      // 0..3

    QPoint para_win_pos = {320, 150};
    QRect source_win_pos = {350, 10, 700, 300};         // Source Window Position and Size. See: Konstruktor class Sourcewin

    struct _cvd_func_ *first_func = nullptr, *last_func = nullptr;

private slots:
    void closeEvent(QCloseEvent *event);
    void new_connect();
    void client_read_ready();
    void client_discontect();

    void on_treeWidget_itemClicked(QTreeWidgetItem *item, int column);          // tree click
    void on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column);    // tree double click

    void on_actionSpeichern_triggered();                // Datei / Speichern    not yet implemented
    void on_actionBeenden_triggered();                  // Datei / Beenden  Strg+Q

    void on_actionBaumstruktur_zuklappen_triggered();   // Ansicht / Baumstruktur zuklappen ????
    void on_actionAlle_Fenster_schli_en_triggered();    // Ansicht / Alle Fenster schließen
    void on_actionSource_Window_schlie_en_triggered();  // Ansicht / Source Window schließen
    void on_actionOpenCv_Version_triggered();           // Ansicht / OpenCV Version
    void on_actionshow_FPS_triggered();                 // show FPS

    void on_actionCVD_OFF_triggered();                  // Extra / set CVD OFF/ON
    void on_actionset_all_Function_OFF_triggered();     // Extra / all Function OFF/ON
    void on_actionall_Breakpoint_s_OFF_triggered();     // Extra / all Breakpoint's OFF

    void on_actionAbout_triggered();                    // Help / About

    void trigger_timer ( void );                        // Timer 1s        

private:
    struct _cvd_func_ *new_func (struct _func_data_transfer_ *cf);

    Ui::MainWindow *ui;
    QTcpServer *tcpServer;
    QTcpSocket *client = nullptr;

    void write_state (struct _cvd_func_ *cf);
    void write_header_bef (int befehl);


    void kill_func (struct _cvd_func_ *foo);
    void kill_all_func (void);
    struct _cvd_func_ *grep_func_addr (uint64_t addr);

    struct _cvd_para_ *new_para (struct _cvd_func_ *cf, struct _para_data_transfer_ *cp);
    struct _cvd_func_ *grep_func_by_para_pointer (QTreeWidgetItem *item);           // Parameter
    struct _cvd_func_ *grep_func_by_func_off_pointer (QTreeWidgetItem *item);       // Function OFF
    struct _cvd_func_ *grep_func_by_show_image_pointer (QTreeWidgetItem *item);     // Show Image
    struct _cvd_func_ *grep_func_by_break_func_pointer (QTreeWidgetItem *item);     // Break
    struct _cvd_func_ *grep_func_by_source_pointer (QTreeWidgetItem *item);         // Source

    struct _cvd_para_ *grep_para_by_tree_pointer (QTreeWidgetItem *item);           // Wird nicht mehr benötigt.

    int grep_enum (const char *str);    // emittelt den Wert der Variablen
    char *get_enum_text (int val);

#ifdef USE_PARAM_XML
    void check_param_list ( void );
#endif

};  // class MainWindow

extern MainWindow *glob_mw;

#endif // MAINWINDOW_H
