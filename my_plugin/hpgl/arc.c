#include <inttypes.h>
#include <math.h>

#include "shvars.h"
#include "arc.h"
#include "scale.h"

static int16_t arc_step;
static float arc_stepangle, arc_xc, arc_yc, arc_a0, arc_r, arc_phi;

int16_t arc_init (void)
{
    arc_phi = numpad[2] * M_PI / 180.0f;

    arc_stepangle = fabsf(numpad[3]);
    if (arc_phi < -0.0f)
        arc_stepangle = -arc_stepangle;
    arc_stepangle *= M_PI / 180.0f;

    arc_step = 0;
    arc_xc = numpad[0];
    arc_yc = numpad[1];
    arc_a0 = atan2f(user_loc.y - arc_yc, user_loc.x - arc_xc);
    if (arc_a0 < -0.0f)
        arc_a0 += 2.0f * M_PI;

    arc_r = hypotf(user_loc.x - arc_xc, user_loc.y - arc_yc);

    //printf_P(PSTR("AA: (%f,%f) r=%f stepangle=%f a0=%f aend=%f\n"),
    //  arc_xc, arc_yc, arc_r, arc_stepangle, arc_a0*180/M_PI, (arc_a0+arc_phi)*180/M_PI);

    return arc_phi != 0.0f && arc_r != 0.0f;
}

bool arc_next (hpgl_point_t *target)
{
    bool cont = true;
    user_point_t d;

    arc_step++;
    float alpha = arc_step * arc_stepangle;
    if (fabsf(alpha) > fabsf(arc_phi)) {
        alpha = arc_phi;
        cont = false;
    }
    alpha += arc_a0;

    d.x = arc_xc + arc_r * cosf(alpha);
    d.y = arc_yc + arc_r * sinf(alpha);

    //printf_P(PSTR("ARC SPAN: (%6.1f %6.1f)-(%6.1f %6.1f)\n"), user_loc.x, user_loc.y, xd, yd);    

    userscale(d, target, &user_loc);

    return cont;//arc_step < arc_steps ? 1 : 0;
}
