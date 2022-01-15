#ifndef _HPGL_H
#define _HPGL_H

/// HPGL commands. Returned by hpgl_char() when there is data and handled by do_stuff() in motori.c
/// @see do_stuff()
/// @see hpgl_char()
typedef enum {
    CMD_ERR = -1,       ///< Error
    CMD_CONT = 0,       ///< Continue, do nothing (data coming)
    CMD_PA = 1,         ///< Move to returned coordinates
    CMD_PD,             ///< Pen down
    CMD_PU,             ///< Pen up
    CMD_ARCABS,         ///< Start arc absolute
    CMD_INIT,           ///< Initialize
    CMD_SEEK0,          ///< Locate home position
    CMD_EA,             ///< Rectangle
    CMD_LB0,            ///< Mark label start
    CMD_LB,             ///< Label text
    CMD_SI,             ///< Absolute character size
    CMD_SR,             ///< Relative character size
    CMD_DI,             ///< Label direction: numpad[0]=sin(theta), numpad[1]=cos(theta)
    CMD_AS,             ///< Acceleration Select: 0 = no acceleration (nonstandard)
    CMD_VS,             ///< Velocity Select: 0 = fastest (nonstandard)
} hpgl_command_t;

/// Internal scanner state. 
/// @see hpgl_char()
typedef enum {
    STATE_EXP1 = 0,     ///< Expect first char of a command
    STATE_EXP_P,
    STATE_EXP_S,
    STATE_EXP_I,
    STATE_EXP_A,
    STATE_EXP_E,
    STATE_EXP_L,
    STATE_EXP_D,
    STATE_EXP_V,
    
    STATE_X,
    STATE_Y,
    STATE_SP,           ///< Select pen
    STATE_SC,           ///< Scale
    STATE_IP,           ///< Coordinates
    STATE_LB,           ///< Label text
    STATE_SI,           ///< absolute character size in cm
    STATE_SR,           ///< relative character size
    STATE_DT,           ///< label terminator char
    STATE_DI,           ///< label direction
    
    STATE_AS,           ///< Acceleration Select (nonstandard: 0/1)
    STATE_VS,           ///< Velocity Select (nonstandard: value = skip steps, the more the slower)
    
    STATE_EXP4,         ///< Expect 4 numbers (like for AA, IP, SC)
    STATE_ARC,          ///< Arc
    STATE_SKIP_END,     ///< Skip all until semicolon
} scanner_state_t;

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


#endif
