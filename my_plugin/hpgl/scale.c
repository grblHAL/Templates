#include <math.h>
#include <stdio.h>

#include "scale.h"

#include "grbl/hal.h"

static user_point_t user_scale, user_translate;

void translate_init_ip (void)
{
    hpgl_state.ip_pad[0] = 0;
    hpgl_state.ip_pad[1] = 0;
    hpgl_state.ip_pad[2] = MAX_X;
    hpgl_state.ip_pad[3] = MAX_Y;
}

void translate_init_sc (void)
{
    user_scale.x = user_scale.y = 1.0f;
    user_translate.x = user_translate.y = 0.0f;

    hpgl_state.sc_pad[0] = 0;
    hpgl_state.sc_pad[1] = MAX_X;
    hpgl_state.sc_pad[2] = 0;
    hpgl_state.sc_pad[3] = MAX_Y;
}

// use IP and SC data to calculate the scale
void translate_scale (void)
{
    user_point_t iprange = range_P1P2();

    int32_t scxrange = hpgl_state.sc_pad[1] - hpgl_state.sc_pad[0];   // xmax - xmin
    int32_t scyrange = hpgl_state.sc_pad[3] - hpgl_state.sc_pad[2];   // ymax - ymin
    
    user_scale.x = iprange.x / (float)scxrange;
    user_scale.y = iprange.y / (float)scyrange;
    //user_xscale = ((float)scxrange)/((float)ipxrange);
    //user_yscale = ((float)scyrange)/((float)ipyrange);
    user_translate.x = -hpgl_state.sc_pad[0] * user_scale.x;
    user_translate.y = -hpgl_state.sc_pad[2] * user_scale.y;
    
//    printf_P(PSTR("Scale set to: (%f,%f) translate (%f,%f)"), user_xscale, user_yscale, user_translate_x, user_translate_y);
}

void userprescale (user_point_t abs, user_point_t *out)
{
    out->x = abs.x / user_scale.x;
    out->y = abs.y / user_scale.y;
}

void usertohpgl (user_point_t src, hpgl_point_t *target)
{
    target->x = (int16_t)roundf(src.x * user_scale.x);
    target->y = (int16_t)roundf(src.y * user_scale.y);
}

void userscalerelative (user_point_t src, hpgl_point_t *target, user_point_t *out)
{
    target->x = (int16_t)roundf(out->x + src.x * user_scale.x);
    target->y = (int16_t)roundf(out->y + src.y * user_scale.y);

    if(out) {
        out->x = (float)target->x / user_scale.x;
        out->y = (float)target->y / user_scale.y;
    }
}

void userscale (user_point_t src, hpgl_point_t *target, user_point_t *out)
{
    target->x = (int16_t)roundf(src.x * user_scale.x);
    target->y = (int16_t)roundf(src.y * user_scale.y);

    if(out) {
        out->x = (float)target->x / user_scale.x;
        out->y = (float)target->y / user_scale.y;
    }
}

user_point_t range_P1P2 (void)
{
    user_point_t p;

    p.x = (float)(hpgl_state.ip_pad[2] - hpgl_state.ip_pad[0]);
    p.y = (float)(hpgl_state.ip_pad[3] - hpgl_state.ip_pad[1]);
    
    return p;
}

void output_P1P2 (void)
{
    hal.stream.write(uitoa(hpgl_state.ip_pad[0]));
    hal.stream.write(",");
    hal.stream.write(uitoa(hpgl_state.ip_pad[1]));
    hal.stream.write(",");
    hal.stream.write(uitoa(hpgl_state.ip_pad[2]));
    hal.stream.write(",");
    hal.stream.write(uitoa(hpgl_state.ip_pad[3]));
    hal.stream.write(ASCII_EOL);
}
