#include <inttypes.h>

#include "motori.h"

#define SCRATCHPAD_SIZE 64

char scratchpad[SCRATCHPAD_SIZE];
float numpad[4];
int32_t ip_pad[4];
int32_t sc_pad[4];

hpgl_point_t stepper_loc;

user_point_t user_loc;
