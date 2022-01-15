#include <inttypes.h>
#include <math.h>

#include <stdio.h>

#include "scale.h"
#include "shvars.h"

static user_point_t user_scale, user_translate;

void translate_init (void)
{
    user_scale.x = user_scale.y = 1.0f;
    user_translate.x = user_translate.y = 0.0f;
    ip_pad[0] = 0;
    ip_pad[1] = 0;
    ip_pad[2] = 9500;
    ip_pad[3] = 7000;
    sc_pad[0] = 0;
    sc_pad[1] = 9500;
    sc_pad[2] = 0;
    sc_pad[3] = 7000;
}

// use IP and SC data to calculate the scale
void translate_scale (void)
{
    int32_t ipxrange = ip_pad[2] - ip_pad[0];
    int32_t ipyrange = ip_pad[3] - ip_pad[1];
    int32_t scxrange = sc_pad[1] - sc_pad[0];   // xmax - xmin
    int32_t scyrange = sc_pad[3] - sc_pad[2];   // ymax - ymin
    
    user_scale.x = ((float)ipxrange) / ((float)scxrange);
    user_scale.y = ((float)ipyrange) / ((float)scyrange);
    //user_xscale = ((float)scxrange)/((float)ipxrange);
    //user_yscale = ((float)scyrange)/((float)ipyrange);
    user_translate.x = -sc_pad[0] * user_scale.x;
    user_translate.y = -sc_pad[2] * user_scale.y;
    
//    printf_P(PSTR("Scale set to: (%f,%f) translate (%f,%f)"), user_xscale, user_yscale, user_translate_x, user_translate_y);
}

void userprescale (user_point_t abs, user_point_t *out)
{
    out->x = abs.x / user_scale.x;
    out->y = abs.y / user_scale.y;
}

void userscale (user_point_t src, hpgl_point_t *target, user_point_t *out)
{
    target->x = (int16_t)roundf(src.x * STEPSCALE_X * user_scale.x);
    target->y = (int16_t)roundf(src.y * STEPSCALE_Y * user_scale.y);
    
    out->x = (target->x) / (user_scale.x * STEPSCALE_X);
    out->y = (target->y) / (user_scale.y * STEPSCALE_Y);
}

user_point_t scale_P1P2 ()
{
    user_point_t p;
    p.x = ip_pad[2] - ip_pad[0];
    p.y = ip_pad[3] - ip_pad[1];
    
    return p;
}
