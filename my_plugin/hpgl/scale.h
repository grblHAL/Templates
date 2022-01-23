#ifndef _SCALE_H
#define _SCALE_H

#include "hpgl.h"

/// Initialize translation and scale.
///
/// Translation is not implemented.
void translate_init_ip (void);
void translate_init_sc (void);

/// Use IP and SC data to calculate the scale and translation.
///
/// Translation is not implemented. 
void translate_scale (void);

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
void userscale (user_point_t src, hpgl_point_t *target, user_point_t *out);

void userscalerelative (user_point_t src, hpgl_point_t *target, user_point_t *out);

void usertohpgl (user_point_t src, hpgl_point_t *target);

/// Something that shouldn't be used 
void userprescale (user_point_t abs, user_point_t *out);

/// Something else that should not be used
user_point_t range_P1P2 (void);

void output_P1P2 (void);

#endif
