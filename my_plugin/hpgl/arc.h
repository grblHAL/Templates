#ifndef _ARC_H
#define _ARC_H

#include <inttypes.h>
#include <stdbool.h>

/// Initialize the arc based on current location and scratchpad data.
/// @see numpad
/// @see user_loc
/// @returns 0 if the arc is degenerate (0 degrees or R=0)
int16_t arc_init(void);

/// Calculate the next chord. 
/// @param x next x in absolute stepper coordinates
/// @param y next y in absolute stepper coordinates
/// @returns 0 if this is the last chord
bool arc_next (hpgl_point_t *target);

#endif
