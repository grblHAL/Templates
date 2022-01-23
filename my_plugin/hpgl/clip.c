#include <stdint.h>
#include <stdbool.h>

// https://educatech.in/c-program-to-implement-sutherland-hodgemann-polygon-clipping-algorithm/

enum {
    TOP = 0x1,
    BOTTOM = 0x2,
    RIGHT = 0x4,
    LEFT = 0x8
};

typedef uint_fast8_t outcode;

float xwmin,xwmax,ywmin,ywmax;

outcode CompOutCode(float x, float y)
{
    outcode code = 0;
    if(y > ywmax)
        code |=TOP;
    else if(y < ywmin)
        code |= BOTTOM;

    if(x > xwmax)
        code |= RIGHT;
    else if(x < xwmin)
        code |= LEFT;

    return code;
}

void clip (float x0, float y0, float x1, float y1)
{
    outcode outcode0,outcode1,outcodeOut;
    bool accept = false, done = false;
    outcode0 = CompOutCode(x0, y0);
    outcode1 = CompOutCode(x1, y1);

    do {
        if(!(outcode0 | outcode1)) {
            accept = true;
            done = true;
        }
        else if(!(done = !!(outcode0 & outcode1))) {

            float x, y;

            outcodeOut = outcode0 ? outcode0 : outcode1;

            if(outcodeOut & TOP) {
                x = x0 + (x1 - x0) * (ywmax - y0) / (y1 - y0);
                y = ywmax;
            } else if(outcodeOut & BOTTOM) {
                x = x0 + (x1 - x0) * (ywmin - y0) / (y1 - y0);
                y = ywmin;
            } else if(outcodeOut & RIGHT) {
                y = y0 + (y1 - y0) * (xwmax - x0) / (x1 - x0);
                x = xwmax;
            } else {
                y = y0 + (y1 - y0) * (xwmin -x0) / (x1 - x0);
                x = xwmin;
            }
            if(outcodeOut == outcode0) {
                x0 = x;
                y0 = y;
                outcode0 = CompOutCode(x0,y0);
            } else {
                x1 = x;
                y1 = y;
                outcode1 = CompOutCode(x1,y1);
            }
        }
    } while(!done);
/*
    if(accept)
    line(x0,y0,x1,y1);
    outtextxy(150,20,"POLYGON AFTER CLIPPING");
    rectangle(xwmin,ywmin,xwmax,ywmax);
    */
}
