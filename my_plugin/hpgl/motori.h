#ifndef _MOTORI_H
#define _MOTORI_H

#include <stdbool.h>

#define PEN_DOWN_DELAY  20
#define PEN_LIFT_DELAY  50

#define PSTR(s) s
#define printf_P printf
#define DELAY_MS(ms) hal.delay_ms(ms, NULL)

typedef enum {
    Pen_Up = 0,
    Pen_Down = 1,
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

/// Controls the pen position. Both servo and solenoid actuators.
/// Causes immediate delay to allow for the pen to settle.
///
/// @param down true if pen should be down, 0 if raised
void pen_control (pen_status_t state);

/// Initialize plotter state. Move to home position, then reset everything, including motors and timers.
/// Reset user scale and translation, raise the pen.
void plotter_init();

#endif
