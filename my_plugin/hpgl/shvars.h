#ifndef _SHVARS_H
#define _SHVARS_H

#include "motori.h"

extern char scratchpad[];		///< character scratchpad used by scanner
extern float numpad[];		///< stored parameters of AA and similar commands
extern int32_t ip_pad[];		///< stored parameters of IP command (4)
extern int32_t sc_pad[];		///< stored parameters of SC command (4)

extern hpgl_point_t stepper_loc;      ///< absolute stepper coordinates

extern user_point_t user_loc;		///< rounded location in user coordinate system (used for arc calculation)

#endif
