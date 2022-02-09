#include <math.h>

#include "hpgl.h"
#include "arc.h"
#include "scale.h"

static int16_t arc_step;
static uint_fast8_t is_wedge = 0;
static float arc_stepangle, arc_xc, arc_yc, arc_a0, arc_r, arc_phi;

static bool arc_cfg (user_point_t user_loc)
{
    arc_phi = hpgl_state.numpad[2] * M_PI / 180.0f;

    arc_stepangle = fabsf(hpgl_state.numpad[3]);
    if (arc_phi < -0.0f)
        arc_stepangle = -arc_stepangle;
    arc_stepangle *= M_PI / 180.0f;

    arc_step = 0;
    arc_xc = hpgl_state.numpad[0];
    arc_yc = hpgl_state.numpad[1];
    arc_a0 = atan2f(hpgl_state.user_loc.y - arc_yc, hpgl_state.user_loc.x - arc_xc);
    if (arc_a0 < -0.0f)
        arc_a0 += 2.0f * M_PI;

    arc_r = hypotf(hpgl_state.user_loc.x - arc_xc, hpgl_state.user_loc.y - arc_yc);

    //printf_P(PSTR("AA: (%f,%f) r=%f stepangle=%f a0=%f aend=%f\n"),
    //  arc_xc, arc_yc, arc_r, arc_stepangle, arc_a0*180/M_PI, (arc_a0+arc_phi)*180/M_PI);

    return arc_phi != 0.0f && arc_r != 0.0f;
}


bool arc_init (void)
{
    is_wedge = 0;
    return arc_cfg(hpgl_state.user_loc);
}

bool circle_init (hpgl_point_t *target)
{
    user_point_t d;

    is_wedge = 0;
    d.x = hpgl_state.user_loc.x + hpgl_state.numpad[0];
    d.y = hpgl_state.user_loc.y;

    hpgl_state.numpad[2] = 360.0f;
    hpgl_state.numpad[3] = hpgl_state.numpad[1];
    hpgl_state.numpad[0] = hpgl_state.user_loc.x;
    hpgl_state.numpad[1] = hpgl_state.user_loc.y;

    userscale(d, target, &hpgl_state.user_loc);

    return arc_cfg(d);
}

bool wedge_init (void)
{
    is_wedge = 2;
    float r = hpgl_state.numpad[0];
    arc_phi = hpgl_state.numpad[1] * M_PI / 180.0f;

    hpgl_state.numpad[0] = hpgl_state.user_loc.x;
    hpgl_state.numpad[1] = hpgl_state.user_loc.y;
    hpgl_state.user_loc.y += r * sinf(arc_phi);
    hpgl_state.user_loc.x += r * cosf(arc_phi);

    return arc_cfg(hpgl_state.user_loc);
}

bool arc_next (hpgl_point_t *target)
{
    bool cont = true;
    user_point_t d;

    if(is_wedge == 2) {

        is_wedge = 1;
        d.x = hpgl_state.numpad[0];
        d.y = hpgl_state.numpad[1];

    } else {

        arc_step++;
        float alpha = arc_step * arc_stepangle;

        if (fabsf(alpha) > fabsf(arc_phi)) {
            alpha = arc_phi;
            cont = false;
        }

        alpha += arc_a0;
        d.x = arc_xc + arc_r * cosf(alpha);
        d.y = arc_yc + arc_r * sinf(alpha);
    }

    if(!cont && is_wedge == 1) {
        cont = true;
        is_wedge = 3;
    }

    if(is_wedge == 3) {
        cont = false;
        d.x = hpgl_state.numpad[0];
        d.y = hpgl_state.numpad[1];
    }

    //printf_P(PSTR("ARC SPAN: (%6.1f %6.1f)-(%6.1f %6.1f)\n"), user_loc.x, user_loc.y, xd, yd);    

    userscale(d, target, &hpgl_state.user_loc);

    return cont;//arc_step < arc_steps ? 1 : 0;
}
