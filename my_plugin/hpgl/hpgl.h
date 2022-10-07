#ifndef _HPGL_H
#define _HPGL_H

#include <stdint.h>
#include <stdbool.h>

//#define HPGL_DEBUG

#ifndef HPGL_DEVICE_IDENTIFICATION
#define HPGL_DEVICE_IDENTIFICATION "HP7574A"
#endif

#define SCRATCHPAD_SIZE 64

#define MAX_X_A4    11040
#define MAX_Y_A4    7721
#define P1X_A4      603
#define P1Y_A4      521
#define P2X_A4      10603
#define P2Y_A4      7721

#define MAX_X_A3    16158
#define MAX_Y_A3    11040
#define P1X_A3      170
#define P1Y_A3      602
#define P2X_A3      15370
#define P2Y_A3      10602

#ifdef HPGL_A3
#define MAX_X       MAX_X_A3
#define MAX_Y       MAX_Y_A3
#define P1X         P1X_A3
#define P1Y         P1Y_A3
#define P2Y         P2Y_A3
#define P2X         P2X_A3
#else
#define MAX_X       MAX_X_A4
#define MAX_Y       MAX_Y_A4
#define P1X         P1X_A4
#define P1Y         P1Y_A4
#define P2Y         P2Y_A4
#define P2X         P2X_A4
#endif

typedef enum {
    ERR_None = 0,
    ERR_UnknownCommand,
    ERR_WrongParams,
    ERR_BadParam,
    ERR_Unused1,
    ERR_UnknownCharset,
    ERR_PosOverflow,
    ERR_Unused2,
    Err_WheelsUp
} hpgl_error_t;

/// HPGL commands. Returned by hpgl_char() when there is data and handled by do_stuff() in motori.c
/// @see do_stuff()
/// @see hpgl_char()
typedef enum {
    CMD_CONT = 0,               ///< Continue, do nothing (data coming)
    CMD_ERR,                    ///< Error
    CMD_LB0,                    ///< Mark label start!!
    CMD_AA = 'A' << 8 | 'A',    ///< AA: Arc absolute
    CMD_AR = 'A' << 8 | 'R',    ///< AR: Arc relative
    CMD_AS = 'A' << 8 | 'S',    ///< AS: Acceleration Select: 0 = no acceleration (nonstandard)
    CMD_CA = 'C' << 8 | 'A',    ///< CA: Designate alternate character set
    CMD_CI = 'C' << 8 | 'I',    ///< CI: Circle
    CMD_CP = 'C' << 8 | 'P',    ///< CP: Character plot
    CMD_CS = 'C' << 8 | 'S',    ///< CS: Designate standard character set
    CMD_DI = 'D' << 8 | 'I',    ///< DI: Label direction: numpad[0]=sin(theta), numpad[1]=cos(theta)
    CMD_DF = 'D' << 8 | 'F',    ///< DF: Set default values
    CMD_DT = 'D' << 8 | 'T',    ///< DT: Set text terminator
    CMD_DV = 'D' << 8 | 'V',    ///< DV: Vertical label direction
    CMD_EA = 'E' << 8 | 'A',    ///< EA: Rectangle absolute
    CMD_ER = 'E' << 8 | 'R',    ///< EV: Rectangle relative
    CMD_ES = 'E' << 8 | 'S',    ///< ES: Extra Space
    CMD_EW = 'E' << 8 | 'W',    ///< EW: Edge Wedge
    CMD_IN = 'I' << 8 | 'N',    ///< IN: Initialize
    CMD_IM = 'I' << 8 | 'M',    ///< IM: Input Mask
    CMD_IP = 'I' << 8 | 'P',    ///< IP: Initialize plotter
    CMD_LB = 'L' << 8 | 'B',    ///< LB: Label text
    CMD_LT = 'L' << 8 | 'T',    ///< LT: Line type

    CMD_OA = 'O' << 8 | 'A',    ///< OA: Output Actual Position and Pen Status
    CMD_OC = 'O' << 8 | 'C',    ///< OC: Output Commanded Position and Pen Status
    CMD_OD = 'O' << 8 | 'D',    ///< OD: Output Digitized Point and Pen Status
    CMD_OE = 'O' << 8 | 'E',    ///< OE: Output Error
    CMD_OF = 'O' << 8 | 'F',    ///< OF: Output Factors
    CMD_OH = 'O' << 8 | 'H',    ///< OH: Output Hard-clip Limits
    CMD_OI = 'O' << 8 | 'I',    ///< OI: Output Identification
    CMD_OO = 'O' << 8 | 'O',    ///< OO: Output Options
   
    CMD_OP = 'O' << 8 | 'P',    ///< OP: Output Parameters P1 & P2
    
    CMD_OS = 'O' << 8 | 'S',    ///< OS: Output Status
    CMD_OW = 'O' << 8 | 'W',    ///< OW: Output Window
 
    CMD_PA = 'P' << 8 | 'A',    ///< PA: Move to returned coordinates
    CMD_PD = 'P' << 8 | 'D',    ///< PD: Pen down
    CMD_PR = 'P' << 8 | 'R',    ///< PR: Nove to relative position
    CMD_PT = 'P' << 8 | 'T',    ///< PT: Pen thickness
    CMD_PU = 'P' << 8 | 'U',    ///< PU: Pen up
    CMD_SA = 'S' << 8 | 'A',    ///< SA: Select alternate character set
    CMD_SC = 'S' << 8 | 'C',    ///< SC: Scale
    CMD_SI = 'S' << 8 | 'I',    ///< SI: Absolute character size
    CMD_SP = 'S' << 8 | 'P',    ///< SP: Select pen
    CMD_SR = 'S' << 8 | 'R',    ///< SR: Relative character size
    CMD_SS = 'S' << 8 | 'S',    ///< SS: Select standard character set
    CMD_VS = 'V' << 8 | 'S',    ///< VS: Velocity Select: 0 = fastest (nonstandard)
    CMD_SEEK0 = 'H' << 8 | 'S'  ///< Locate home position
} hpgl_command_t;

typedef enum {
    Pen_NoAction = 0,
    Pen_Up,
    Pen_Down,
    Pen_Timeout,
    Pen_Unknown = 255
} pen_status_t;

///< Absolute coordinates used for stepper motion.
///< Negative means invalid.
typedef int16_t hpgl_coord_t;      

///< User coordinates used in input, arc calculation etc.
typedef float user_coord_t;

typedef struct {
    hpgl_coord_t x;
    hpgl_coord_t y;
} hpgl_point_t;

typedef struct {
    user_coord_t x;
    user_coord_t y;
} user_point_t;

typedef struct {
    float width;
    float height;
} char_size_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t pen_down        :1,
                p1p2_changed    :1,
                point_available :1,
                initialized     :1,
                ready           :1,
                error           :1,
                service         :1,
                unused          :1;
    };
} hpgl_status_flags;

typedef union {
    uint8_t value;
    struct {
        uint8_t enable_dtr   :1,
                unused       :1,
                monitor_mode :1,
                monitor_on   :1,
                block_mode   :1,
                unassigned   :3;
    };
} hpgl_comm_flags;

typedef struct {
    float chord_angle;
    float pen_thickness;
    bool plot_relative;
    uint8_t etxchar;
    char term[3];
    uint8_t pattern_type;
    float pattern_length;
    bool use_alt_charset;
    bool text_vertical;
    char_size_t character_size;
    const char *const *charset;
    const char *const *charset_std;
    const char *const *charset_alt;
    user_point_t cr_loc;
    hpgl_error_t first_error;
    hpgl_error_t last_error;
    uint8_t errmask;
    uint8_t alertmask;
    float numpad[4];
    // The following values are not changed on a reset to default values, ip_pad must be first!
    int32_t ip_pad[4];
    int32_t sc_pad[4];
    int32_t esc_pad[8];
    user_point_t user_loc;
    hpgl_status_flags flags;
    hpgl_comm_flags comm;
} hpgl_state_t;

extern hpgl_state_t hpgl_state;
typedef hpgl_command_t (*hpgl_char_ptr)(char c, hpgl_point_t *target, uint8_t *lb);

/// Initialize the scanner.
void hpgl_init();

/// Handle next character from the input. When action is determined, return one of the _hpgl_command values.
/// Pass target coordinates in x and y.
/// @param c    input char
/// @param x    output: destination x (returns -1 if no data)
/// @param y    output: destination y (returns -1 if no data)
/// @param lb   output: next label character (see CMD_LB)
/// @returns    _hpgl_command
extern hpgl_char_ptr hpgl_char;

hpgl_error_t hpgl_get_error (void);
void hpgl_set_error (hpgl_error_t errnum);

#endif
