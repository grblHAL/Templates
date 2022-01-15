#include <stdio.h>
#include <math.h>

#include "motori.h"
#include "shvars.h"
#include "scale.h"
#include "htext.h"

#include "../grbl/hal.h"

user_point_t fontscale;
user_point_t charorigin;
user_point_t labelorigin;

user_coord_t sintheta, costheta;

static char *coffs;

void text_init (void)
{
    text_setscale(10.0f, 10.0f);
    text_direction(0.0f, 1.0f);
}

void text_setscale (float sx, float sy)
{
    fontscale.x = sx;
    fontscale.y = sy;

    printf_P(PSTR("Text scale is (%f, %f)\n"), fontscale.x, fontscale.y);
}

void text_scale_cm (float cx, float cy)
{
    user_point_t s;

    s.x = cx * 10.0f / 7.0f / 0.025;
    s.y = cy * 10.0f / 10.0f / 0.025;

    userprescale(s, &fontscale);
    text_setscale(fontscale.x, fontscale.y);
}

void text_scale_rel (float rx, float ry)
{
    user_point_t prect = scale_P1P2();
    float sx = rx / 100.0f * prect.x / 7.0f; // character size in plotter units
    float sy = ry / 100.0f * prect.y / 10.0f;

    text_setscale(sx, sy);
}

void text_direction (float cost, float sint)
{
    sintheta = -sint;
    costheta = cost;
    
    printf_P(PSTR("Label rotation: sin=%f cos=%f\n"), sintheta, costheta);
}

static void rotate (float* x, float* y)
{
    float xc = *x - charorigin.x;
    float yc = *y - charorigin.y;
    float xrot = xc * costheta + yc * sintheta;
    float yrot = -xc * sintheta + yc * costheta;
    
    *x = xrot + charorigin.x;
    *y = yrot + charorigin.y;
}

void text_beginlabel (void)
{
    labelorigin = user_loc;
    printf_P(PSTR("Label origin: (%f,%f)\n"), labelorigin.x, labelorigin.y);
}

bool text_char (uint8_t c, hpgl_point_t *target, pen_status_t *pen)
{
    user_point_t d;
    uint8_t encoded;
    static bool noadvance = false;
    
    if (c != 0) {
        noadvance = false;
        switch (c) {
        case '\r':
            printf_P(PSTR("CR"));
            d.x = labelorigin.x;
            d.y = labelorigin.y;
            //rotate(&xd, &yd);
            userscale(d, target, &user_loc);
            charorigin = user_loc;
            *pen = Pen_Up;
            encoded = 1;
            coffs = charset0[0];
            noadvance = true;
            break;
        case '\n':
            printf_P(PSTR("LF"));
            d.x = user_loc.x;
            d.y = user_loc.y - fontscale.y * 10.0f;
            rotate(&d.x, &d.y);
            userscale(d, target, &user_loc);
            charorigin = user_loc;
            labelorigin = user_loc;
            *pen = Pen_Up;
            encoded = 1;
            coffs = charset0[0];
            noadvance = true;
            break;
        default:
            coffs = charset0[c];
            *pen = Pen_Up;
            charorigin = user_loc;
            encoded = 1;
            //printf_P(PSTR("coffs=%x first=%o"), charset0[c], *charset0[c]);
            break;
        } 

    } else {

        encoded = *coffs++;
        *pen = encoded & 0b10000000 ? Pen_Down : Pen_Up;
        
        if (encoded) {
            d.x = charorigin.x + fontscale.x * ((encoded >> 4) & 0b111);
            d.y = charorigin.y + fontscale.y * ((encoded & 0b1111) - 4);
        } else if (!noadvance) {
            d.x = charorigin.x + fontscale.x * 5.0f;
            d.y = charorigin.y;
        }

        hal.stream.write("CH:");
        hal.stream.write(ftoa(d.x, 3));
        hal.stream.write(",");
        hal.stream.write(ftoa(d.y, 3));
        hal.stream.write(ASCII_EOL);

        if (!noadvance) {
            rotate(&d.x, &d.y);
            userscale(d, target, &user_loc);
        }

        hal.stream.write("CT:");
        hal.stream.write(uitoa(target->x));
        hal.stream.write(",");
        hal.stream.write(uitoa(target->y));
        hal.stream.write(ASCII_EOL);
    }
    
    return encoded != 0;
}
