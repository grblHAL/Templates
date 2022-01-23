#ifndef _HPGL_H
#define _HPGL_H

#include <stdint.h>
#include <stdbool.h>

//#define HPGL_DEBUG

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
    CMD_ERR = -1,       ///< Error
    CMD_CONT = 0,       ///< Continue, do nothing (data coming)
    CMD_AA = 1,         ///< Start arc absolute
    CMD_AR,             ///< Start arc relative
    CMD_AS,             ///< Acceleration Select: 0 = no acceleration (nonstandard)
    CMD_CI,             ///< Circle
    CMD_DI,             ///< Label direction: numpad[0]=sin(theta), numpad[1]=cos(theta)
    CMD_IN,             ///< Initialize
    CMD_EA,             ///< Rectangle absolute
    CMD_ER,             ///< Rectangle relative
    CMD_LB0,            ///< Mark label start
    CMD_LB,             ///< Label text
    CMD_LT,             ///< Line type
    CMD_PA,             ///< Move to returned coordinates
    CMD_PD,             ///< Pen down
    CMD_PR,             ///< Nove to relative position
    CMD_PT,             ///< Pen thickness
    CMD_PU,             ///< Pen up
    CMD_SI,             ///< Absolute character size
    CMD_SP,             ///< Select pen
    CMD_SR,             ///< Relative character size
    CMD_VS,             ///< Velocity Select: 0 = fastest (nonstandard)
    CMD_SEEK0           ///< Locate home position
} hpgl_command_t;

/// Internal scanner state. 
/// @see hpgl_char()
typedef enum {
    STATE_EXP1 = 0,     ///< Expect first char of a command
    STATE_EXP_A,
    STATE_EXP_C,
    STATE_EXP_D,
    STATE_EXP_E,
    STATE_EXP_I,
    STATE_EXP_L,
    STATE_EXP_O,
    STATE_EXP_P,
    STATE_EXP_S,
    STATE_EXP_V,

    STATE_AA,           ///< Arc absolute
    STATE_AR,           ///< Arc relative
    STATE_AS,           ///< Acceleration Select (nonstandard: 0/1)
    STATE_CI,           ///< plot circle
    STATE_DF,           ///< Reset to default values
    STATE_DI,           ///< label direction
    STATE_DT,           ///< label terminator char
    STATE_EA,
    STATE_ER,
    STATE_IN,           ///< init plotter
    STATE_IM,           ///< set input mask
    STATE_IP,           ///< Coordinates
    STATE_LB,           ///< Label text
    STATE_LT,           ///< set linetype
    STATE_OP,           ///< Output coordinates P1 & P2
    STATE_PT,           ///< set pen thickness
    STATE_SC,           ///< Scale
    STATE_SI,           ///< absolute character size in cm
    STATE_SP,           ///< Select pen
    STATE_SR,           ///< relative character size
    STATE_VS,           ///< Velocity Select (nonstandard: value = skip steps, the more the slower)

    STATE_X,
    STATE_Y,
    STATE_EXP4,         ///< Expect 4 numbers (like for AA, IP, SC)
    STATE_SKIP_END,     ///< Skip all until semicolon
    STATE_AWAIT_TERMINATOR,     ///< Skip whitespace until semicolon
} scanner_state_t;

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

typedef struct {
    float chord_angle;
    float pen_thickness;
    bool plot_relative;
    uint8_t etxchar;
    uint8_t pattern_type;
    float pattern_length;
    char_size_t character_size;
    user_point_t cr_loc;
    hpgl_error_t last_error;
    uint8_t errmask;
    uint8_t alertmask;
    float numpad[4];
    // The following values are not changed on a reset to default values, ip_pad must be first!
    int32_t ip_pad[4];
    int32_t sc_pad[4];
    user_point_t user_loc;
} hpgl_state_t;

extern hpgl_state_t hpgl_state;

/// Initialize the scanner.
void hpgl_init();

/// Handle next character from the input. When action is determined, return one of the _hpgl_command values.
/// Pass target coordinates in x and y.
/// @param c    input char
/// @param x    output: destination x (returns -1 if no data)
/// @param y    output: destination y (returns -1 if no data)
/// @param lb   output: next label character (see CMD_LB)
/// @returns    _hpgl_command
hpgl_command_t hpgl_char(char c, hpgl_point_t *target, uint8_t *lb);

hpgl_error_t hpgl_get_error (void);
void hpgl_set_error (hpgl_error_t errnum);

#endif
