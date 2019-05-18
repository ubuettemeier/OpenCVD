#ifndef OPENCVD_BASIC_HPP
#define OPENCVD_BASIC_HPP

#include <stdlib.h>
#include <linux/limits.h>

#include <iostream>
#include <thread>
#include <sys/time.h>


#ifdef _WIN32
    #include <winsock.h>
    #include <io.h>
#else
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <netdb.h>
    #include <arpa/inet.h>
    #include <unistd.h>
#endif

using namespace std;

// ------------------ Global Value --------------------------------
int sock = -1;
struct sockaddr_in server;
struct hostent *host_info;
unsigned long addr;
uint8_t cvd_off = 0;
uint8_t read_thread_end = 0;
uint8_t write_data_blocked = 0;         // Atomic
std::thread *client_thread = NULL;

uint32_t para_id_counter = 0x00000001;

// --------------- Func Deklaration ------------------------------
int init_socket ( void );
int write_data (uint8_t *data, uint32_t len);

int write_ack (void);
int read_data ( int sockfd, unsigned char *data, int data_len );
void control_socket ();
void read_thread ( void );
void show_func_list( void );
void delete_cvd( void );

signed long long cvd_difference_micro (struct timeval *start, struct timeval *stop);
signed long long cvd_time_differnce (struct timeval *start);

//!
//! \brief The opencvd_start class
//!
class opencvd_start {
public:
    opencvd_start();        // init the server. Init the read Thread
    ~opencvd_start();       // disconnect the server. Kill read Thread
};

opencvd_start cvd_start;    // Autostart

//!
//! \brief The opencvd_para class
//!
class opencvd_para {
public:
    opencvd_para(uint16_t type, uint64_t addr, void *p, const char *p_name, uint8_t *dat, int len);
    ~opencvd_para();
    void write_para( void );
public:
    uint16_t para_type;                 // See: enum _data_types_ 0x2000 ... 0x2FFFF
    uint64_t func_addr;
    uint32_t para_id = 0x00000000;
    char para_name[MAX_PARA_NAME_LEN];
    void *func_pointer;                 // zeigt auf class cvd_func
    uint8_t data[MAX_PARA_DATA];        // buffer für Parameter-Wert
    uint8_t data_buf[MAX_PARA_DATA];    // dient zum Sichern der Anfangswerte. Kann bei Reset zurückgeholt werden.

};

//!
//! \brief The opencvd_func class
//!
class opencvd_func {
public:
    opencvd_func (uint64_t addr, uint16_t type, const char *f_name, uint8_t start_flags=0x0007, int line_no=0, const char *src_file=NULL);
    ~opencvd_func ();
    void new_para (uint16_t type, int len, uint8_t *data, const char *p_name);
    void write_func ( void );
    void control_imshow ( cv::OutputArray dst );
    void control_contours_imshow ( cv::InputOutputArray image,
                                   cv::OutputArrayOfArrays contours,
                                   cv::OutputArray hierarchy );

    static opencvd_func *grep_func(std::vector<opencvd_func *>func,  uint64_t addr);
    static opencvd_func *grep_func_from_liste (uint64_t addr);
    static opencvd_para *grep_para_id (uint32_t id);
    int control_func_run_time ( void );

public:
    uint16_t func_type;
    uint64_t func_addr;
    union _stateflags_ state;
    uint32_t line_nr;
    char func_name[MAX_FUNC_NAME_LEN];
    char file_name[MAX_FILENAME_LEN];

    cv::String window_name;             // func_name + addr. used by cv::namedWindow()
    vector <opencvd_para *> para{};     // Leere Parameterliste erzeugen.
    uint8_t window_is_create = 0;       // sobald das Ausgabefenster mit namedWindow() erzeugt wird, ändert das Flag seinen Wert auf 1
                                        // See: control_imshow()
private:
    struct timeval time_stemp;

};

vector <opencvd_func *> func_list{};    // empty list (Global)

//!
//! \brief init_socket
//! \return succes=0  failure=1
//!
int init_socket ()
{
    sock = socket ( AF_INET, SOCK_STREAM, 0 );
    if (sock < 0) {
        cout << "ERROR socket\n";
        return EXIT_FAILURE;
    }

    memset ( &server, 0, sizeof(server) );

    if ((addr = inet_addr( SERVER_ADDR )) != INADDR_NONE) {
        memcpy ( (char*)&server.sin_addr, &addr, sizeof(addr));
    } else {
        host_info = gethostbyname( SERVER_ADDR );
        if (NULL == host_info) {
            cout << "unknown server\n";
            return EXIT_FAILURE;
        }
        memcpy ( (char*)&server.sin_addr, host_info->h_addr, host_info->h_length );
    }
    server.sin_family = AF_INET;
    server.sin_port = htons( PORT );

    if (connect( sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
        cout << "no connection to the server\n";
        close (sock);
        sock = -1;
        return EXIT_FAILURE;
    } else {
        cout << "I connect to the server on localhost\n";
    }

    client_thread = new std::thread( read_thread );
    client_thread->detach();

    return EXIT_SUCCESS;
}
//!
//! \brief write_data
//! \param data
//! \param len
//! \return
//!
#define USE_SETSOCKOPT_
int write_data (uint8_t *data, uint32_t len)
{
    write_data_blocked = 1;
    static int anz_error = 0;
    if (sock < 0 )
        return 0;

    int anz = write ( sock, data, len );

    if ((uint32_t)anz != len) {                 // ERROR
        if (anz_error < 6) anz_error++;
        if (anz_error < 5)
            cout << "opencvd_client::write ERROR\n";
        else if (anz_error == 5)
            cout << "opencvd_client::write ERROR ...\n";
    }

    write_data_blocked = 0;
    return anz;
}

//!
//! \brief write_ack
//! \return Anzahl gesendeter Bytes
//!
int write_ack (void)
{
    // struct _cvd_header_ ch = {sizeof(struct _cvd_header_), SOCKET_ACK};
    // return write_data ((uint8_t*)&ch, sizeof(struct _cvd_header_));
    return 0;
}

//!
//! \brief read_data
//!        Funktion liest Daten vom socket.
//!        Der read ist auf 10ms limitiert.
//!        Die Funktion wird vom main-Thread aufgerufen.
//! \param sockfd
//! \param data
//! \param data_len
//! \see   void control_socket();
//! \return Anzahl der gelesen Bytes.
//!
int read_data ( int sockfd, unsigned char *data, int data_len )
{
    int n = 0;                  // default
    fd_set fds;
    struct timeval tv;

    tv.tv_sec = 0;              // do socket initialization etc.
    tv.tv_usec = 10000;         // 10ms

    if (sockfd < 0)
        return 0;

    FD_ZERO(&fds);              // tv now represents 1.5 seconds
    FD_SET(sockfd, &fds);       // adds sock to the file descriptor set

    select(sockfd+1, &fds, NULL, NULL, &tv);      // wait 10ms for any data to be read from any single socket

    if (FD_ISSET(sockfd, &fds)) {   // es sind Daten eingetroffen
        if ( (n = read(sockfd, data, data_len) ) < 0 ) {		// liest Daten vom Handle/Descriptor sockfd. Die Funktion liest blockierend
            printf ("ERROR reading from socket");
        } /* else
            printf ("%i Byte gelesen\n", n); */
    }
    return n;                   // n==0 => default, keine Daten eingetroffen
                                // n>0 => n-Byte vorhanden
                                // n<0 => Read-ERROR    
}

//!
//! \brief control_socket
//!        control_socket() wird von read_thread() aufgerufen !
//!        Funktion ueberwacht den Dateneingang
//!
#define MAX_READ_BUFFER  8192			// Groesse Read-Buffer
void control_socket ()
{
    uint8_t buffer[MAX_READ_BUFFER];	// Im buffer werden die gelesenen Blöcke abgelegt,
    // uint32_t anz_data = 0;
    int anz_data = 0;
    uint32_t *len;
    uint16_t *bef;
    uint32_t pointer = 0;

    anz_data = read_data ( sock, buffer, MAX_READ_BUFFER);	    // nicht blockierend (10ms).    
    while (anz_data >= 4) {
        // printf ("anz_data=%i\n", anz_data);
        len = (uint32_t*)&buffer[pointer];
        if (*len > (uint32_t)anz_data)
            return;
        else {  // if (*len <= (uint32_t)anz_data) {
            bef = (uint16_t*)(&buffer[4+pointer]);
            // printf ("Befehl = 0x%4X\n", *bef);
            switch (*bef) {            

            case SCALAR_PARA:
            case POINT_INT:
            case POINT_INT_XY:
            case INT_PARA:            
            case FLOAT_PARA:
            case DOUBLE_PARA:
            case ENUM_DROP_DOWN:
            case STRING_PARA:
            case SLIDE_DOUBLE_PARA:
            case SLIDE_INT_PARA:
            case SLIDE_INT_TWO_STEP_PARA: {
                struct _para_data_transfer_ *foo = (struct _para_data_transfer_ *)&buffer[pointer];   // buffer casten
                opencvd_para *cp = opencvd_func::grep_para_id ( foo->para_id );              // Parameter finden. Es werden in allen Funktionen die Parameter durchgefummelt.
                if (cp != NULL) {
                    // printf ("para %s\n", cp->para_name);
                    switch (cp->para_type) {
                    case STRING_PARA: {
                        struct _string_para_ *foo_sp = (struct _string_para_ *)foo->data;   // pointer to buffer->data
                        struct _string_para_ *cp_sp = (struct _string_para_ *)cp->data;     // pointer to parameter->data
                        strcpy (cp_sp->val, foo_sp->val);
                        write_ack();
                        }
                        break;
                    case SCALAR_PARA: {
                        struct _scalar_double_ *foo_sp = (struct _scalar_double_ *)foo->data;   // pointer to buffer->data
                        struct _scalar_double_ *cp_sp = (struct _scalar_double_ *)cp->data;     // pointer to parameter->data
                        memcpy (cp_sp->val, foo_sp->val, sizeof(double) * 4);
                        write_ack();
                        }
                        break;
                    case POINT_INT_XY:
                    case POINT_INT: {
                        struct _point_int_ *foo_sp = (struct _point_int_ *)foo->data;   // pointer to buffer->data
                        struct _point_int_ *cp_sp = (struct _point_int_ *)cp->data;     // pointer to parameter->data
                        cp_sp->x = foo_sp->x;                                           // value neu setzen !
                        cp_sp->y = foo_sp->y;                                           // value neu setzen !
                        write_ack();
                        }
                        break;
                    case INT_PARA:                  // Achtung: "struct _int_para_" ist identisch mit "struct _int_step_para_"
                    case SLIDE_INT_PARA:
                    case SLIDE_INT_TWO_STEP_PARA: {
                        struct _int_para_ *foo_sp = (struct _int_para_ *)foo->data;   // pointer to buffer->data
                        struct _int_para_ *cp_sp = (struct _int_para_ *)cp->data;     // pointer to parameter->data
                        cp_sp->value = foo_sp->value;                                           // value neu setzen !
                        write_ack();
                        }
                        break;
                    case DOUBLE_PARA:               // Achtung: "struct _double_para_" und "struct _double_step_para_" sind identisch !!!
                    case SLIDE_DOUBLE_PARA: {
                        struct _double_para_ *foo_sp = (struct _double_para_ *)foo->data;   // pointer to buffer->data
                        struct _double_para_ *cp_sp = (struct _double_para_ *)cp->data;     // pointer to parameter->data
                        cp_sp->value = foo_sp->value;                                           // value neu setzen !
                        write_ack();
                        }
                        break;
                    case FLOAT_PARA: {
                        struct _float_para_ *foo_sp = (struct _float_para_ *)foo->data;   // pointer to buffer->data
                        struct _float_para_ *cp_sp = (struct _float_para_ *)cp->data;     // pointer to parameter->data
                        cp_sp->value = foo_sp->value;                                           // value neu setzen !
                        write_ack();
                        }
                        break;
                    case ENUM_DROP_DOWN: {
                        struct _enum_para_ *foo_sp = (struct _enum_para_ *)foo->data;   // pointer to buffer->data
                        struct _enum_para_ *cp_sp = (struct _enum_para_ *)cp->data;     // pointer to parameter->data
                        cp_sp->value = foo_sp->value;                                           // value neu setzen !
                        write_ack();
                        }
                        break;
                    }

                } else
                    printf ("kein Parameter gefunden\n");
                }
                break;
            case FUNC_FLAGS: {          // 0x3000 ... 0x3FFF
                struct _cvd_flags_ *foo = (struct _cvd_flags_ *)&buffer[pointer];     // buffer casten
                opencvd_func *f = opencvd_func::grep_func_from_liste( foo->func_addr );    // Funktion ermitteln
                if (f != NULL) {
                    f->state.val = foo->state.val;
                    // printf ("es wurden flags empfangen %s %4X\n", f->func_name, f->state.val);
                    // printf ("%s\nbild stop = %i\nshow image = %i\n", f->func_name, f->state.flag.func_off, f->state.flag.show_image);
                }
                }
                break;
            case SET_CVD_OFF:
                cvd_off = 1;
                break;
            case SET_CVD_ON:
                cvd_off = 0;
                break;
            case CLOSE_SERVER:                      // 0xF000 ... 0xFffff
                close (sock);
                sock = -1;
                for (int i=0; i<(int)func_list.size(); i++) {
                    func_list[i]->state.flag.show_image = 0;
                    func_list[i]->state.flag.func_off = 0;
                    func_list[i]->state.flag.func_break = 0;
                }

                printf ("server is closed\n");
                break;
            default:
                printf ("unbekanntes Datenpaket\n");
                break;
            }   // switch
        } // if (*len >= (uint32_t)anz_data) {
        pointer += *len;
        anz_data -= *len;
    }   // while
}

//!
//! \brief read_thread
//!
void read_thread ()
{
    cout << "read_thread start\n";
    while (!read_thread_end) {
        control_socket();
    }
    cout << "read_thread end\n";
    read_thread_end = 2;
}

//!
//! \brief show_func_list
//!
void show_func_list( void )
{
    for (int i=0; i<(int)func_list.size(); i++) {
        cout << func_list[i]->func_name << " anz Para: " << func_list[i]->para.size() << "\n";
        for (int j=0; j<(int)func_list[i]->para.size(); j++) {
            opencvd_para *cp = func_list[i]->para[j];
            cout << "-- " << cp->para_name << " id=" << cp->para_id << endl;
        }
    }
}

//!
//! \brief delete_cvd
//!
void delete_cvd( void )
{
    // cout << "delete_cvd muss " << func_list.size() << " Funktionen löschen\n";

    for (int i=0; i<(int)func_list.size(); i++) {
        opencvd_func *foo = func_list[i];
        if (foo->state.flag.show_image) {
            foo->state.flag.show_image = 0;
            cv::destroyWindow ( foo->window_name );
        }
        delete func_list[i];
    }
}

//!
//! \brief cvd_difference_micro
//! \param start
//! \param stop
//! \return Zeitdifferenz im Microsekunden
//!
signed long long cvd_difference_micro (struct timeval *start, struct timeval *stop)
{
    return ((signed long long) stop->tv_sec * 1000000ll +
           (signed long long) stop->tv_usec) -
           ((signed long long) start->tv_sec * 1000000ll +
           (signed long long) start->tv_usec);
}

//!
//! \brief cvd_time_differnce
//! \param start
//! \return
//!
signed long long cvd_time_differnce (struct timeval *start)
{
    struct timeval akt;

    gettimeofday (&akt, NULL);
    return cvd_difference_micro(start, &akt);
}

//!
//! \brief opencvd_start::opencvd_start
//!        OpenCVD Start
//!
opencvd_start::opencvd_start()
{
    cout << "app gestartet\n";
    init_socket ();
}

//!
//! \brief opencvd_start::~opencvd_start
//!        OpenCVD beenden
//!
opencvd_start::~opencvd_start()
{
    struct _cvd_header_ h;
    h.len = sizeof (struct _cvd_header_);
    h.bef = CLOSE_CLIENT;
    write_data ( (uint8_t*)&h, h.len );

    if (client_thread != NULL) {            // read_thread beenden
        read_thread_end = 1;
        while (read_thread_end == 1) {      // wait for for thread end
            usleep (100);
        }
    }

    cout << "app beendet\n";
}

//!
//! \brief opencvd_func::opencvd_func
//! \param addr
//! \param type
//! \param f_name = Funktionsname
//! \param start_flags
//!
opencvd_func::opencvd_func (uint64_t addr, uint16_t type, const char *f_name, uint8_t start_flags, int line_no, const char *src_file)
{
    func_addr = addr;
    func_type = type;
    line_nr = line_no;
    state.val = start_flags;     // alle Menüpunkte EIN

    if (src_file == NULL)       // Filename bearbeiten.
        strcpy (file_name, "");
    else {
        char foo[4096*2];
        // GetCurrentDir (foo, sizeof(foo));
        // char *path;
        // _get_pgmptr (&path);
        getcwd ( foo, 4096*2 );
        strcat (foo, "/");
        strcat (foo, src_file);     // make filename

        if (strlen(foo) > MAX_FILENAME_LEN) {
            strcpy (file_name, "");
            cout << "ERROR: file_name to long\n";
        } else
            strcpy (file_name, foo);
    }    
    // printf ("file_name=%s len=%li\n", file_name, strlen(file_name));

    if (strlen(f_name) < MAX_FUNC_NAME_LEN)
        strcpy (func_name, f_name);
    else
        cout << "func name to long\n";

    func_list.push_back( this );

    gettimeofday(&time_stemp, NULL);
    window_name = std::string(func_name) + std::string("_") + std::to_string(func_addr & 0x0000000000FFFFFF);   // create Window Name

    write_func ();
    state.val = 0x0000;
}

//!
//! \brief opencvd_func::~opencvd_func
//!
opencvd_func::~opencvd_func ()
{
    for (int i=0; i<(int)para.size(); i++)
        delete para[i];

    // cout << func_name << " gelöscht\n---------\n";
}

//!
//! \brief opencvd_func::grep_func
//! \param f
//! \param addr
//! \return
//!
opencvd_func *opencvd_func::grep_func(std::vector<opencvd_func *>f,  uint64_t addr)
{
    for (int i=0; i<static_cast<int>(f.size()); i++) {
        if (f[i]->func_addr == addr) {
            return f[i];
        }
    }
    return NULL;
}

//!
//! \brief opencvd_func::grep_func_from_liste
//! \param addr
//! \return
//!
opencvd_func *opencvd_func::grep_func_from_liste (uint64_t addr)
{
    for (int i=0; i<(int)func_list.size(); i++) {
        if (func_list[i]->func_addr == addr)
            return func_list[i];
    }

    return NULL;
}

//!
//! \brief opencvd_func::grep_para_id
//! \param id
//! \return
//!
opencvd_para *opencvd_func::grep_para_id(uint32_t id)
{
    // printf ("grep_para id=%i\n", id);
    for (int i=0; i<(int)func_list.size(); i++) {
        opencvd_func *cf = func_list[i];
        for (int j=0; j<(int)cf->para.size(); j++) {
            opencvd_para *cp = cf->para[j];
            if (cp->para_id == id) {
                return cp;
            }
        }
    }

    return NULL;
}

//!
//! \brief opencvd_func::write_func
//!
void opencvd_func::write_func ()
{
    struct _func_data_transfer_ fd;

    fd.len = sizeof(struct _func_data_transfer_);
    fd.type = func_type;
    fd.func_addr = func_addr;
    fd.line_nr = line_nr;
    fd.state.val = state.val;

    strcpy (fd.func_name, func_name);
    strcpy (fd.file_name, file_name);

    // printf ("%i %s\n", strlen(fd.file_name), fd.file_name);

    write_data( (uint8_t*)&fd, sizeof(struct _func_data_transfer_) );
}

//!
//! \brief opencvd_func::control_imshow
//! \param dst
//!
void opencvd_func::control_imshow ( cv::OutputArray dst )
{
    if (state.flag.show_image) {
        if (!dst.empty()) {
            if (window_is_create == 0) {
                cv::namedWindow ( window_name );
                window_is_create = 1;
            }
            cv::imshow( window_name, dst );
        }
    } else
        if (window_is_create != 0)
            cv::destroyWindow ( window_name );
}

//!
//! \brief opencvd_func::control_contours_imshow
//! \param image
//! \param contours
//! \param hierarchy
//!
void opencvd_func::control_contours_imshow ( cv::InputOutputArray image,
                                             cv::OutputArrayOfArrays contours,
                                             cv::OutputArray hierarchy )
{
    if (state.flag.show_image) {
        if (!image.empty()) {
            if (window_is_create == 0) {
                cv::namedWindow ( window_name );
                window_is_create = 1;
            }

            cv::Mat dst = cv::Mat::zeros(image.rows(), image.cols(), CV_8UC3);
            dst = cv::Scalar (0, 0, 0);

            for (int i=0; i<hierarchy.size().width; i++) {
                cv::Scalar color( std::rand()&255, std::rand()&255, std::rand()&255 );
                cv::drawContours( dst, contours, i, color); // , CV_FILLED, 8, hierarchy );
            }
            if (!dst.empty())
                cv::imshow (window_name, dst);
        }
    } else
        if (window_is_create != 0)
            cv::destroyWindow ( window_name );
}

//!
//! \brief opencvd_func::new_para
//!        new_para create object opencvd_para and insert this in para-list.
//! \param type
//! \param len
//! \param data
//! \param p_name
//!
void opencvd_func::new_para (uint16_t type, int len, uint8_t *data, const char *p_name)
{
    opencvd_para *cp = new opencvd_para( type, func_addr, (void*)this, p_name, data, len );
    para.push_back( cp );
}

//!
//! \brief opencvd_func::control_func_run_time
//! \return
//!
#define TRIGGER_TIME 400000  // 600000
int opencvd_func::control_func_run_time ()
{

    // signed long long delta;

    // if ((delta = cvd_time_differnce (&time_stemp)) > TRIGGER_TIME) {
    if ((cvd_time_differnce (&time_stemp)) > TRIGGER_TIME) {
        // printf ("%s=%lli\n", cf->func_name, delta);
        gettimeofday(&time_stemp, NULL);
        struct _time_trigger_ tt;
        tt.func_addr = func_addr;
        tt.type = TIME_TRIGGER;
        tt.len = sizeof(struct _time_trigger_);
        write_data( (uint8_t*)&tt, sizeof(struct _time_trigger_));
    }

    return 0;
}

//!
//! \brief opencvd_para::opencvd_para
//! \param type
//! \param addr
//! \param p
//! \param p_name
//! \param dat
//! \param len
//!
opencvd_para::opencvd_para(uint16_t type, uint64_t addr, void *p, const char *p_name, uint8_t *dat, int len)
{
    para_type = type;
    func_addr = addr;
    func_pointer = p;               // pointer to class opencvd_func
    para_id = para_id_counter;
    para_id_counter++;
    if (strlen(p_name) < MAX_PARA_NAME_LEN)
        strcpy (para_name, p_name);
    else
        cout << "para name to long\n";
    memcpy (data, dat, len);
    memcpy (data_buf, dat, len);

    write_para();
}

//!
//! \brief opencvd_para::~opencvd_para
//!
opencvd_para::~opencvd_para()
{
    // cout << "Parameter " << para_name << " gelöscht\n";
}

//!
//! \brief opencvd_para::write_para
//!        write_para() move the func-parameter to the server
//!
void opencvd_para::write_para()
{
    struct _para_data_transfer_ pd;

    pd.len = sizeof(struct _para_data_transfer_);
    pd.type = para_type;
    pd.func_addr = func_addr;
    pd.para_id = para_id;
    strcpy (pd.para_name, para_name);
    memcpy (pd.data, data, MAX_PARA_DATA);

    write_data( (uint8_t*)&pd, sizeof(struct _para_data_transfer_) );
}

#endif // OPENCVD_BASIC_HPP
