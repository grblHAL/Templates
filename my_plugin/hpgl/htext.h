#ifndef _HTEXT_H
#define _HTEXT_H

#include <stdint.h>
#include <stdbool.h>

#include "motori.h"

void text_init(void);
void text_setscale(float sx, float sy);
void text_scale_cm(float cx, float cy);
void text_scale_rel(float rx, float ry);
void text_direction(float cost, float sint);

void text_beginlabel();
bool text_char (uint8_t c, hpgl_point_t *target, pen_status_t *pen);
bool text_pos (float x, float y, hpgl_point_t *target);

#endif
