#ifndef OPENCVD_TYPES_HPP
#define OPENCVD_TYPES_HPP

#define PORT            51717
#define SERVER_ADDR     "127.0.0.1"
#define BUFFERSIZE      8192


#define MAX_FUNC_NAME_LEN 64
#define MAX_PARA_NAME_LEN 64
#define MAX_PARA_DATA 256
#define MAX_FILENAME_LEN 256        // used for c-source filename

#define MAX_STRING_VAL_LEN 256      // used for cvd::imread, cvd::String, cvd::get_filename

//!
//! \brief The _data_types_ enum
//! \todo if you change it, please also update the functions
//!       grep_enum()
//!       get_enum_text()
//!
enum _data_types_ {
    // ---- Function 0x1000...0x1FFF -------
    INITFUNC = 0x1000,          // ????
    MEDIANBLUR = 0x1001,        // *
    THRESHOLD = 0x1002,         // *
    CVTCOLOR = 0x1003,          // *
    LAPLACIAN = 0x1004,
    CANNY = 0x1005,             // *
    CANNY_2 = 0x1006,           // *
    CONVERTTO = 0x1007,
    OPERATOR_INT_MUL_EQUAL = 0x1008,        // operator *= <int>
    OPERATOR_FLOAT_MUL_EQUAL = 0x1009,      // operator *= <float>
    OPERATOR_DOUBLE_MUL_EQUAL = 0x100A,     // operator *= <double>
    GAUSSIANBLUR = 0x100B,          // *
    BLUR_FUNC = 0x100C,             // *
    SCALAR_FUNC_4 = 0x100D,
    DILATE = 0x100E,
    ERODE = 0x100F,
    FINDCONTOURS = 0x1010,
    CONVERTSCALEABS = 0x1011,       // *
    SCALAR_ALL = 0x1012,
    SCHARR = 0x1013,
    IMREAD = 0x1014,                // *
    GRABCUT = 0x1015,               // *
    GETSTRUCTURINGELEMENT = 0x1016,    
    MORPHOLOGYEX = 0x1017,
    NORMALIZE = 0x1018,
    CALCHIST = 0x1019,
    HOUGHCIRCLES = 0x101A,          // *
    HOUGHLINESP = 0x101B,           // *
    HOUGHLINES = 0x101C,            // *
    GETBUILDINFORMATION = 0x101D,
    ADAPTIVETHRESHOLD = 0x101E,     // *
    RESIZE = 0x101F,                // *
    SOBEL = 0x1020,                 // *
    PYRDOWN = 0x1021,               // *
    PYRUP = 0x1022,                 // *
    CORNERHARRIS = 0x1023,          // *
    FITLINE = 0x1024,               // *
    APPROXPOLYPD = 0x1025,          // *
    SCALEADD = 0x1026,              // *
    BUILDPYRAMID = 0x1027,          // *
    RECTANGLE_1 = 0x1028,
    RECTANGLE_2 = 0x1029,
    CREATELINESEGMENTDETECTOR = 0x102A,     // *
    BILATERALFILTER = 0x102B,       // *
    PYRMEANSHIFTFILTERING = 0x102C, // *
    DISTANCETRANSFORM = 0x102D,     // *
    MATCHTEMPLATE = 0x102E,         // *
    MATCHSHAPES = 0x102F,           // *
    BOXFILTER = 0x1030,             // *
    SQRBOXFILTER = 0x1031,          // *
    PUTTEXT = 0x1032,               // *
    FILTER2D = 0x1033,              // *
    SEQFILTER2D = 0x1034,           // *
    GETGAUSSIANKERNEL = 0x1035,     // *
    GETGABORKERNEL = 0x1036,

    CVD_RECT_TYPE_1_INT = 0x1500,       // *
    CVD_RECT_TYPE_1_FLOAT = 0x1501,     // *
    CVD_RECT_TYPE_1_DOUBLE = 0x1502,    // *

    CVD_SCALAR_1 = 0x1540,          // *
    CVD_SCALAR_2 = 0x1541,          // *

    CVD_POINT_TYPE_1_INT = 0x1580,
    CVD_POINT_TYPE_1_INT64 = 0x1581,
    CVD_POINT_TYPE_1_FLOAT = 0x1582,
    CVD_POINT_TYPE_1_DOUBLE = 0x1583,

    GET_NUMVAL = 0x1700,            // *
    SET_TRACKBAR = 0x1701,          // *
    GET_FILENAME = 0x1702,          // *
    GET_ENUMVAL = 0x1703,           // *

    MAT_ROI = 0x1800,               // *
    MAT_CONVERTTO = 0x1801,         // *
    MAT_ROWS_COLS_TYPE_SCALAR = 0x1802,     // *
    MAT_SIZE_TYPE_SCALAR = 0x1803,  // *
    MAT_ROWS_COLS_TYPE = 0x1804,    // *
    MAT_SIZE_TYPE = 0x1805,         // *
    MAT_ASSIGNTO = 0x1806,          // *
    MAT_ONES = 0x1807,              // *
    MAT_ONES_2 = 0x1808,            // *
    MAT_ONES_3 = 0x1809,            // *
    MAT_ZEROS = 0x180A,             // *
    MAT_ZEROS_2 = 0x180B,           // *
    MAT_ZEROS_3 = 0x180C,           // *
    MAT_EYE = 0x180D,               // *
    MAT_EYE_2 = 0x180E,             // *

    STRING_FUNC = 0x1900,           // *

    // ----- Parameter 0x2000...0x2FFF --------
    SLIDE_INT_TWO_STEP_PARA = 0x2000,       // z.B. 1, 3, 5, ... 21 (int)
    SLIDE_INT_PARA = 0x2001,                // z.B. 0..255 (int)
    SLIDE_DOUBLE_PARA = 0x2002,             // z.B  0.0 ... 255.0
    DOUBLE_PARA = 0x2003,                   // double value, min, max
    INT_PARA = 0x2004,                      // int value
    ENUM_DROP_DOWN = 0x2005,                // es muss der Name der enum Deklaration uebergeben werden. Z.B.: "ThresholdTypes". S.auch <struct _enum_para_>
    FLOAT_PARA = 0x2006,                    // float value, min, max
    POINT_INT = 0x2007,                     // {W int value, min, max, H int value, min, max} Anzeige erfolgt in W=..., H=....  Parameter ohne Grenzwerte
    SCALAR_PARA = 0x2008,                   // s.auch: struct _scalar_double_
    STRING_PARA = 0x2009,                   // "String"
    POINT_INT_XY = 0x200A,                  // {X int value, min, max, Y int value, min, max} Anzeige erfolgt in X=..., Y=....  Parameter ohne Grenzwerte
    RECT_INT_PARA = 0x200B,                 // sx, sy, ex, ey
    RECT_DOUBLE_PARA = 0x200C,
    POINT_DOUBLE_XY = 0x200D,               // X, min, max, decimal_x, Y, min, max, decimal_y
    // ----- Special -----------
    FUNC_FLAGS = 0x3000,                    // s.auch struct _cvd_flags_
    TIME_TRIGGER = 0x3001,                  // s.auch struct _time_trigger_
    SET_CVD_OFF = 0x3002,
    SET_CVD_ON = 0x3003,    
    // ----- System 0xF000...0xFFFF --------
    CLOSE_CLIENT = 0xF000,      // Zeigt dem Server, das der client beendet wird. Verbindung kann geschlossen werden.
    CLOSE_SERVER = 0xF001,      // Zeigt dem Client, das der Server beendet wird.
    SOCKET_ACK = 0xF002,        // currently unused
    GET_CV_VERSION = 0xF003,    // Server to Client
    SET_CV_VERSION = 0xF004,    // Client to Server
};

//!
//! \brief The _menu_types_ enum
//!         value for server menu
//! \see class opencvd_func, union _stateflags_ state.
//!
enum _menu_types_ {
    PARAMETER = 1,
    FUNC_OFF = 2,
    SHOW_IMAGE = 4,
    BREAK = 8,
};

enum _error_flags_ {        // used for opencvd_func::error_flag
    FUNC_ERROR = 0x01,
    IMSHOW_ERORR = 0x02,
};

#pragma pack(1)
//!
//! \brief The _point_int_ struct
//!
struct _point_int_ {
    int x;
    int min_x;
    int max_x;
    int y;
    int min_y;
    int max_y;
};

struct _point_double_ {
    double x;
    double min_x, max_x;
    int decimals_x;         // Nachkommastellen

    double y;
    double min_y, max_y;
    int decimals_y;         // Nachkommastellen
};
//!
//! \brief The _rect_int_ struct
//!
struct _rect_int_ {
    int x, y, w, h;
    int min_val, max_val;
};
//!
//! \brief The _double_int_ struct
//!
struct _rect_double_ {
    double x, y, w, h;
    double min_val, max_val;
};
//!
//! \brief The _int_para_ struct
//!
struct _int_para_ {     // ist identisch mit struct _int_step_para_
    int value;
    int min;
    int max;
};

//!
//! \brief The _float_para_ struct
//!
struct _float_para_ {     // ist identisch mit struct _int_step_para_
    float value;
    float min;
    float max;
};

//!
//! \brief The _double_para_ struct
//!
struct _double_para_ {
    double value;
    double min;
    double max;
    int decimals;   // Nachkommastellen
};

//!
//! \brief The _slide_double_para_ struct
//!
struct _slide_double_para_ {
    double value;
    double min;
    double max;
    double divisor;     // z.B  min=0.0 max=3.14 ====> divisor = 100.0
};

//!
//! \brief The _enum_para_ struct
//!
struct _enum_para_ {
    int value;              // z.B.: 0 für cv::THRESH_BINARY
    char enum_name[64];     // z.B.: "ThresholdTypes"
};
//!
//! \brief The _string_para_ struct
//!
struct _string_para_ {              // wird z.B. für Filename verwendet ( s. imread() ).
    char val[ MAX_STRING_VAL_LEN ];
};

//!
//! \brief The _scalar_double_ struct
//!
struct _scalar_double_ {
    double val[4];
};

//!
//! \brief The _cvd_header_ struct
//!
struct _cvd_header_ {
    uint32_t len;
    uint16_t bef;
};

//!
//! \brief The _flags_ struct
//!
struct _flags_ {                        // s.auch <struct _cvd_func_ *new_func()>
    unsigned use_parameter : 1;         // zeigt das Menü Parameter an.
    unsigned func_off : 1;              // Function ausschalten. Sie wird überbrückt.
    unsigned show_image : 1;            // Zeigt das Ergebnisbild.
    unsigned func_break : 1;            // Das Programm stoppt an dieser Stelle.
    unsigned reset_para : 1;            // Parameter werden auf Ursprungswert zurueckgesetzt. not used
    unsigned rest : 11;                 // uint16_t - 5
};

union _stateflags_ {
    struct _flags_ flag;
    uint16_t val;
};
//!
//! \brief The _func_data_transfer_ struct
//!        Strukturen für die Datenübertragung
//!
struct _func_data_transfer_ {    // Funktions-Daten
    uint32_t len;       // length of this struct
    uint16_t type;      // See: enum _data_types_
    uint64_t func_addr;
    uint32_t line_nr;                           // Zeilen Nr aus source code
    union _stateflags_ state;                   // default = 0x0000
    char func_name[MAX_FUNC_NAME_LEN];
    char file_name[MAX_FILENAME_LEN];
};
//!
//! \brief The _para_data_transfer_ struct
//!
struct _para_data_transfer_ {    // Parameter-Daten
    uint32_t len;       // length of this struct
    uint16_t type;      // See: enum _data_types_
    uint64_t func_addr;
    uint32_t para_id;
    uint16_t extra_para;
    char para_name[MAX_PARA_NAME_LEN];
    uint8_t data[MAX_PARA_DATA];
};

#pragma pack()

//!
//! \brief The _cvd_para_ struct
//!
#pragma pack(1)
struct _cvd_para_ {
    uint32_t len;
    uint16_t type;                  // siehe enum _bef_
    uint64_t func_addr;             // Funktions Adresse
    uint32_t para_id;
    uint16_t flags;
    uint16_t extra_para;
    char para_name[64];
    void *tree_pointer;             // wird auf Server-Seite benötigt !
    struct _cvd_para_ *next, *prev;
    uint8_t data[MAX_PARA_DATA];    // erster Eintrag ist value
    uint8_t reset_data[MAX_PARA_DATA];
};
#pragma pack()

//!
//! \brief The _cvd_func_ struct
//!
#pragma pack(1)
struct _cvd_func_ {
    uint32_t len;
    uint16_t type;                              // THRESHOLD, MEDIANBLUR, ...
    uint64_t func_addr;                         // Funktions Adresse
    uint32_t line_nr;                           // Zeilen Nr aus source code
    union _stateflags_ state;                   // default = 0x0000
    struct timeval time_stemp;
    uint8_t aktiv_icon;
    uint8_t func_is_modifyt;

    char func_name[64];                         // Funktionsname
    char filename[MAX_FILENAME_LEN];
    char window_name[72];

    void *tree_pointer;                         // QTreeWidgetItem *
    void *para_pointer;                         // QTreeWidgetItem *Parameter
    void *func_off;                             // QTreeWidgetItem *Funktion ON/OFF
    void *show_image;                           // QTreeWidgetItem *Show Image
    void *break_func;                           // QTreeWidgetItem *Break
    void *source_pointer;                       // QTreeWidgetItem *Source

    struct _cvd_para_ *first_para, *end_para;   // Parameter-Liste

    struct _cvd_func_ *next, *prev;
};
#pragma pack()

//!
//! \brief The _cvd_flags_ struct
//!
#pragma pack(1)
struct _cvd_flags_ {                            // Daten kommen vom server und werden in <void Control_thread::control_socket ()> verabeitet !!!
    uint32_t len;
    uint16_t type;                              // THRESHOLD, MEDIANBLUR, ...
    uint64_t func_addr;                         // Funktions Adresse
    union _stateflags_ state;                   // default = 0x0000
};

//!
//! \brief The _para_flags_ enum
//!
enum _para_flags_ {
    READ_ONLY = 0x0001,
};

//!
//! \brief The _time_trigger_ struct
//!        mit diesen Daten wird dem server gesagt, das die Funktion <func_addr> aufgerufen worden ist !
//!        s.auch: control_func_run_time()
//!
struct _time_trigger_ {                         // s.auch enum _bef_ TIME_TRIGGER
    uint32_t len;
    uint16_t type;                              // THRESHOLD, MEDIANBLUR, ...
    uint64_t func_addr;                         // Funktions Adresse
    uint8_t error_flag;                         // 0=no error
};

struct _cvd_string_ {
    uint32_t len;
    uint16_t type;
    char val[MAX_STRING_VAL_LEN ];
};
#pragma pack()

#endif // OPENCVD_TYPES_HPP
