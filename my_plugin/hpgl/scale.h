#ifndef _SCALE_H
#define _SCALE_H

#include "motori.h"

#define STEPSCALE_X 1.0f        //!< scale coefficient X
#define STEPSCALE_Y 1.0f        //!< scale coefficient Y


/// Initialize translation and scale.
///
/// Translation is not implemented.
void translate_init(void);

/// Use IP and SC data to calculate the scale and translation.
///
/// Translation is not implemented. 
void translate_scale(void);

/// Transform user coordinates (fx,fy) into plotter coordinates (x,y) according to 
/// the transform defined by IP/SC. Then do reverse transform, round and assign
/// resulting values to (ox,oy).
/// @param	fx 	user coordinates x
/// @param	fy	user coordinates y
/// @param  *x 	(output) absolute stepper x
/// @param  *y 	(output) absolute stepper y
/// @param	*ox (output) corrected fx
/// @param	*oy (output) corrected fy
///
/// @see STEPSCALE_X
/// @see STEPSCALE_Y
void userscale (user_point_t src, hpgl_point_t *target, user_point_t *out);


/// Something that shouldn't be used 
void userprescale (user_point_t abs, user_point_t *out);

/// Something else that should not be used
user_point_t scale_P1P2(void);

#endif
