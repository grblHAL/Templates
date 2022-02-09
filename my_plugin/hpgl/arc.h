#ifndef _ARC_H
#define _ARC_H

#include "hpgl.h"

/// Initialize the arc based on current location and scratchpad data.
/// @see numpad
/// @see user_loc
/// @returns 0 if the arc is degenerate (0 degrees or R=0)
bool arc_init (void);

bool wedge_init (void);

bool circle_init (hpgl_point_t *target);

/// Calculate the next chord. 
/// @param x next x in absolute stepper coordinates
/// @param y next y in absolute stepper coordinates
/// @returns 0 if this is the last chord
bool arc_next (hpgl_point_t *target);

#endif
