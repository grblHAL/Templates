#ifndef _MOTORI_H
#define _MOTORI_H

#include "hpgl.h"
#include "grbl/hal.h"

#define PEN_DOWN_DELAY  20
#define PEN_LIFT_DELAY  50

#define PSTR(s) s
#define printf_P printf
#define DELAY_MS(ms) hal.delay_ms(ms, NULL)

/// Controls the pen position. Both servo and solenoid actuators.
/// Causes immediate delay to allow for the pen to settle.
///
/// @param down true if pen should be down, 0 if raised
void pen_control (pen_status_t state);

pen_status_t get_pen_status (void);

/// Initialize plotter state. Move to home position, then reset everything, including motors and timers.
/// Reset user scale and translation, raise the pen.
void plotter_init();

coord_data_t *get_origin (void);

#endif
