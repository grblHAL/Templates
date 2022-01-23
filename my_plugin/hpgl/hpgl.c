#include <stdio.h>
#include <math.h>
#include <string.h>

#include "hpgl.h"
#include "scale.h"

#include "../grbl/stream.h"
#include "../grbl/nuts_bolts.h"

static const hpgl_state_t defaults = {
    .chord_angle = 5.0f,
    .pen_thickness = .3f,
    .plot_relative = false,
    .etxchar = ASCII_ETX, // ^C
    .pattern_type = 0,
    .pattern_length = 4.0f,
#ifdef HPGL_A3
    .character_size.width = 0.187f,
    .character_size.height = 0.269f,
#else
    .character_size.width = 0.285f,
    .character_size.height = 0.375f,
#endif
    .last_error = ERR_None,
    .errmask = 0,
    .alertmask = 223,
    .ip_pad[0] = 0,
    .ip_pad[1] = 0,
    .ip_pad[2] = MAX_X,
    .ip_pad[3] = MAX_Y,
    .sc_pad[0] = 0,
    .sc_pad[1] = MAX_X,
    .sc_pad[2] = 0,
    .sc_pad[3] = MAX_Y,
    .user_loc.x = 0.0f,
    .user_loc.y = 0.0f
};

hpgl_state_t hpgl_state;

static scanner_state_t pstate, nstate;
static char scratchpad[SCRATCHPAD_SIZE];

__attribute__((weak)) void alert_led (bool on)
{
}

void hpgl_init (void)
{
    pstate = STATE_EXP1;

    memcpy(&hpgl_state, &defaults, sizeof(hpgl_state_t));

    hpgl_set_error(hpgl_state.last_error);

    translate_init_sc();
}

static void hpgl_defaults (void)
{
    memcpy(&hpgl_state, &defaults, offsetof(hpgl_state_t, ip_pad));

    hpgl_set_error(hpgl_state.last_error);
}

void hpgl_set_error (hpgl_error_t errnum)
{
    if(errnum == ERR_None)
        hpgl_state.errmask = 0;
    else
        hpgl_state.errmask |= (1 << errnum);

    hpgl_state.last_error = errnum;

    alert_led(!!(hpgl_state.errmask & hpgl_state.alertmask));
}

hpgl_error_t hpgl_get_error (void)
{
    return hpgl_state.last_error;
}

bool valid_input (char c, uint_fast8_t idx)
{
    // TODO: add check for single decimal point in input?
    return idx < sizeof(scratchpad) - 2 && ((c >= '0' && c <= '9') || c == '.' || (idx == 0 && (c == '+' || c == '-')));
}

hpgl_command_t hpgl_char (char c, hpgl_point_t *target, uint8_t *lb)
{
    static uint_fast8_t i, si;
    static uint_fast8_t numpad_idx;
    static hpgl_command_t cmd = CMD_ERR;
    static user_point_t f;
    
    target->x = target->y = -1;
    *lb = 0;
    
lolbel: 
    switch (pstate) {

        case STATE_EXP1:    // expect first letter
            numpad_idx = si = 0;
            cmd = CMD_CONT;
            switch (c) {
                case ' ':
                case '\n':
                case '\r':
                case '\t':
                case ';': //??
                    break;
                case 'A':
                    pstate = STATE_EXP_A;
                    break;
                case 'C':
                    pstate = STATE_EXP_C;
                    break;
                case 'D':
                    pstate = STATE_EXP_D;
                    break;
                case 'E':
                    pstate = STATE_EXP_E;
                    break;
                case 'I':
                    pstate = STATE_EXP_I;
                    break;
                case 'L':
                    pstate = STATE_EXP_L;
                    break;
                case 'O':
                    pstate = STATE_EXP_O;
                    break;
                case 'P':
                    pstate = STATE_EXP_P;
                    break;
                case 'S':
                    pstate = STATE_EXP_S;
                    break;
                case 'V':
                    pstate = STATE_EXP_V;
                    break;
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

// Skip illegal commmands or wait for command terminator

      case STATE_SKIP_END:
            if (c == ';') {
                pstate = STATE_EXP1;
                cmd = CMD_ERR;
                hpgl_set_error(ERR_UnknownCommand);
            }
            break;

        case STATE_AWAIT_TERMINATOR:
            switch (c) {
                case ' ':
                case '\n':
                case '\r':
                case '\t':
                    break;
                case ';':
                    pstate = nstate;
                    goto lolbel;
                    break;
                default:
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_WrongParams);
                    break;
            }
            break;

// Process second command character

        case STATE_EXP_A:
            switch (c) {
                case ' ':
                case '\n':
                case '\r':
                case '\t':
                    break;
                case 'A':   // Arc Absolute
                    numpad_idx = 0;
                    pstate = STATE_EXP4;
                    nstate = STATE_AA;
                    break;
                case 'R':   // Arc Relative
                    numpad_idx = 0;
                    pstate = STATE_EXP4;
                    nstate = STATE_AR;
                    break;
                case 'S':   // Acceleration Select
                    numpad_idx = 0;
                    pstate = STATE_EXP4;
                    nstate = STATE_AS;
                    break;
            }
            break;

        case STATE_EXP_C:
            switch (c) {
                case 'I':  // CI: plot circle
                    nstate = STATE_CI;
                    pstate = STATE_EXP4;
                    numpad_idx = 0;
                    break;
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

        case STATE_EXP_D:
            switch (c) {
                case 'F':
                    nstate = STATE_DF; // DF: reset to defaults
                    pstate = STATE_AWAIT_TERMINATOR;
                    break;
                case 'T':   // DT: label terminator
                    pstate = STATE_DT;
                    break;
                case 'I':   // DI: label direction
                    pstate = STATE_EXP4;
                    nstate = STATE_DI;
                    numpad_idx = 0;
                    break;
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

        case STATE_EXP_E:
            si = 0;
            switch (c) {
                case 'A':
                    nstate = STATE_EA;
                    pstate = STATE_EXP4;
                    numpad_idx = 0;
                    break;
                case 'R':
                    nstate = STATE_ER;
                    pstate = STATE_EXP4;
                    numpad_idx = 0;
                    break;
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

        case STATE_EXP_I:
            switch (c) {
                case 'P':   // IP: input coordinates
                    nstate = STATE_IP;
                    pstate = STATE_EXP4;
                    numpad_idx = 0;
                    break;
                case 'N':   // IN: init plotter
                    nstate = STATE_IN;
                    pstate = STATE_AWAIT_TERMINATOR;
                    break;
                case 'M':   // IM: input mask
                    nstate = STATE_IM;
                    pstate = STATE_EXP4;
                    break;
                case 'H':   // IH: seek home (not HPGL)
                    pstate = STATE_EXP1;
                    cmd = CMD_SEEK0;
                    break;
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

        case STATE_EXP_L:
            switch (c) {
                case 'B':   // LB: text label
                    pstate = STATE_LB;
                    cmd = CMD_LB0;
                    break;
                case 'T':   // LT: line type
                    pstate = STATE_EXP4;
                    nstate = STATE_LT;
                    break;
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

        case STATE_EXP_O:
            switch (c) {
                case 'P':
                    nstate = STATE_OP; // OP: output P1 & P2
                    pstate = STATE_AWAIT_TERMINATOR;
                    break;
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

        case STATE_EXP_P:
            si = 0;
            switch (c) {
                case 'U':
                    cmd = CMD_PU;
                    pstate = STATE_X;
                    break;
                case 'D':
                    cmd = CMD_PD;
                    pstate = STATE_X;
                    break;
                case 'T': // PT: pen thickness
                    nstate = STATE_PT;
                    pstate = STATE_EXP4;
          //          numpad_idx = 0;
                    break;
                case 'A':
                    cmd = CMD_PA;
                    pstate = STATE_X;
                    hpgl_state.plot_relative = false;
                    break;
                case 'G':   // PG: feed page/home
                    target->x = target->y = 0;
                    hpgl_state.user_loc.x = hpgl_state.user_loc.y = 0.0f;
                    cmd = CMD_PU;
                    pstate = STATE_SKIP_END;
                    break;
                case 'R':
                    cmd = CMD_PR;
                    pstate = STATE_X;
                    hpgl_state.plot_relative = true;
                    break;
                case 'W':
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

        case STATE_EXP_S:
            switch (c) {
                case 'P':  // SP: select pen
                    nstate = STATE_SP;
                    pstate = STATE_EXP4;
                    numpad_idx = 0;
                    break;
                case 'C':
                    nstate = STATE_SC; // SC: scale
                    pstate = STATE_EXP4;
                    numpad_idx = 0;
                    break;
                case 'I':   // SI: absolute character size in cm
                    nstate = STATE_SI;
                    pstate = STATE_EXP4;
                    numpad_idx = 0;
                    break;
                case 'R':
                    nstate = STATE_SR;
                    pstate = STATE_EXP4;
                    numpad_idx = 0;
                    break;
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

        case STATE_EXP_V:
            switch (c) {
                case 'S':   // VS: Velocity Select
                    pstate = STATE_EXP4;
                    nstate = STATE_VS;
                    numpad_idx = 0;
                    break;
                default:
                    pstate = STATE_SKIP_END;
                    break;
            }
            break;

// Validate and preprocess parameters for commands

        case STATE_AA:  // AA: Arc Absolute - xc, yc, CentralAngle [,Degrees per step]
            cmd = CMD_AA;
            if (numpad_idx == 3) {
                hpgl_state.numpad[3] = hpgl_state.chord_angle;
            }
            target->x = target->y = -1;
            pstate = STATE_EXP1;
            //for (i = 0; i < 4; i++) printf_P(PSTR("%f "), hpgl_state.numpad[i]);
            break;

        case STATE_AR: // AR: Arc Relative - xc, yc, CentralAngle [,Degrees per step]
            cmd = CMD_AR;
            if(numpad_idx < 3) {
                cmd = CMD_ERR;
                hpgl_set_error(ERR_WrongParams);
            } else {
                if (numpad_idx == 3)
                    hpgl_state.numpad[3] = hpgl_state.chord_angle;
                user_point_t offset = { 0.0f, 0.0f };
                f.x = hpgl_state.numpad[0];
                f.y = hpgl_state.numpad[1];
                userscalerelative(f, target, &offset);
                hpgl_state.numpad[0] = hpgl_state.user_loc.x + offset.x;
                hpgl_state.numpad[1] = hpgl_state.user_loc.y + offset.y;
                target->x = target->y = -1;
                cmd = CMD_AR;
                pstate = STATE_EXP1;
            }
            break;

        case STATE_AS: // AS: Acceleration Select
            cmd = CMD_AS;
            pstate = STATE_EXP1;
            break;

        case STATE_CI: // CI: Circle - radius [,Degrees per step]
            cmd = CMD_CI;
            if(numpad_idx == 1)
                hpgl_state.numpad[1] = hpgl_state.chord_angle;
            userscale(hpgl_state.user_loc, target, &hpgl_state.user_loc);
            pstate = STATE_EXP1;
            break;

        case STATE_DF: // DF: Set default values
            hpgl_defaults();
            cmd = CMD_CONT;
            pstate = STATE_EXP1;
            break;

        case STATE_DI: // DI: Absolute text direction // numpad contains sin(theta), cos(theta)
            if(numpad_idx == 0) {
                hpgl_state.numpad[0] = 1.0f;
                hpgl_state.numpad[1] = 0.0f;
                numpad_idx == 2;
            }
            if(numpad_idx >= 2) {
            // fabs(hpgl_state.numpad[0] + hpgl_state.numpad[1]) > 0.0004f
            cmd = CMD_DI;
            }
            pstate = STATE_EXP1;
            break;

        case STATE_DT: // DT: Set text terminator
            hpgl_state.etxchar = c;
            pstate = STATE_EXP1;
            break;

        case STATE_EA: // EA: Edge Absolute
            if(numpad_idx < 2) {
                cmd = CMD_ERR;
                hpgl_set_error(ERR_WrongParams);
            } else {
                cmd = CMD_EA;
                f.x = hpgl_state.numpad[0];
                f.y = hpgl_state.numpad[1];
                userscale(f, target, NULL);
                if(numpad_idx > 2)
                    hpgl_set_error(ERR_WrongParams);
            }
            pstate = STATE_EXP1;
            break;

        case STATE_ER: // ER: Edge Relative
            if(numpad_idx < 2) {
                cmd = CMD_ERR;
                hpgl_set_error(ERR_WrongParams);
            } else {
                cmd = CMD_ER;
                f.x = hpgl_state.user_loc.x + hpgl_state.numpad[0];
                f.y = hpgl_state.user_loc.y + hpgl_state.numpad[1];
                userscale(f, target, NULL);
                if(numpad_idx > 2)
                    hpgl_set_error(ERR_WrongParams);
            }
            pstate = STATE_EXP1;
            break;

        case STATE_IM: // IM: Input Mask
            cmd = CMD_CONT;
            pstate = STATE_EXP1;
            if(numpad_idx > 0 && hpgl_state.numpad[0] < 256.0f) {
                hpgl_state.alertmask = truncf(hpgl_state.numpad[0]);
                alert_led(!!(hpgl_state.errmask & hpgl_state.alertmask));
            }
            break;

        case STATE_IN: // IN: Initialize plotter
            cmd = CMD_IN;
            pstate = STATE_EXP1;
            hpgl_set_error(ERR_None);
            break;

        case STATE_IP: // IP: Initialize plotter
            if(numpad_idx == 0)
                translate_init_ip();
            else if(numpad_idx == 2 || numpad_idx == 4) {
                bool valid;
                i = numpad_idx;
                do {
                    i--;
                    valid = hpgl_state.numpad[i] >= 0.0f && (int32_t)truncf(hpgl_state.numpad[i]) <= (i & 0b1 ? MAX_Y : MAX_X);
                } while(i && valid);

                if(valid) {

                    user_point_t ip = range_P1P2();

                    for (i = 0; i < numpad_idx; i++)
                        hpgl_state.ip_pad[i] = (int32_t)truncf(hpgl_state.numpad[i]);

                    if(numpad_idx == 2) {
                        hpgl_state.ip_pad[2] = hpgl_state.ip_pad[0] + ip.x; // clip!
                        hpgl_state.ip_pad[3] = hpgl_state.ip_pad[1] + ip.y;
                    }
                }
            }
            pstate = STATE_EXP1;
            break;

        case STATE_LB: // LB: Label character
            // scanning text
            cmd = CMD_LB;
            if (c == hpgl_state.etxchar) {
                pstate = STATE_EXP1;
                *lb = 0;
            } else {
                target->x = target->y = -1;
                *lb = c;
            }
            break;

        case STATE_LT: // LT: Line Type
            if(numpad_idx == 0) {
                hpgl_state.pattern_type = 0;
                hpgl_state.pattern_length = 4.0f;
                cmd = CMD_LT;
            } else if((cmd = fabsf(hpgl_state.numpad[0]) >= 128.0f ? CMD_ERR : CMD_LT) == CMD_LT) {
                hpgl_state.numpad[0] = truncf(hpgl_state.numpad[0]);
                if(hpgl_state.numpad[0] <= 6.0f)
                    hpgl_state.pattern_type = hpgl_state.numpad[0] <= 0.0f ? 0 : (uint8_t)hpgl_state.numpad[0];
                if(numpad_idx > 1) {
                   if(hpgl_state.numpad[1] >= 0.0f && hpgl_state.numpad[1] < 128.0f)
                       hpgl_state.pattern_length = hpgl_state.numpad[1];
                   else
                       cmd = CMD_ERR;
                } else
                    hpgl_state.pattern_length = 4.0f;
            }
            if(cmd == CMD_ERR)
                hpgl_set_error(ERR_BadParam);
            pstate = STATE_EXP1;
            break;

        case STATE_OP: // OP: Output Parameters P1 & P2
            output_P1P2();
            pstate = STATE_EXP1;
            break;

        case STATE_PT: // PT: pen thickness
            if(numpad_idx == 0)
                hpgl_state.pen_thickness = 0.3f;
            else {
                if(hpgl_state.numpad[0] < 0.1f || hpgl_state.numpad[0] > 5.0f) {
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_BadParam);
                } else
                    hpgl_state.pen_thickness = hpgl_state.numpad[0];
                if(cmd != CMD_ERR && numpad_idx > 1) {
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_WrongParams);
                }
            }
            pstate = STATE_EXP1;
            break;

        case STATE_SC: // Scale
            if(numpad_idx == 0)
                translate_init_sc();
            else if(numpad_idx == 4) {
                for (i = 0; i < 4; i++)
                    hpgl_state.sc_pad[i] = (int32_t)truncf(hpgl_state.numpad[i]);
                translate_scale();
            }
            pstate = STATE_EXP1;
            break;

        case STATE_SI: // SI: Absolute character size
            if(numpad_idx == 0) {
                hpgl_state.numpad[0] = 0.187f;
                hpgl_state.numpad[1] = 0.269f;
            }
            if((cmd = (numpad_idx == 1 ? CMD_ERR : CMD_SI)) == CMD_ERR)
                hpgl_set_error(ERR_WrongParams);
            else {
                hpgl_state.character_size.width = hpgl_state.numpad[0];
                hpgl_state.character_size.height = hpgl_state.numpad[1];
            }
            pstate = STATE_EXP1;
            break;

        case STATE_SP: // SP: Select Pen
            if(numpad_idx == 0)
                hpgl_state.numpad[0] = 0.0f;
            cmd = CMD_SP;
            hpgl_state.pen_thickness = 0.3; // only if pen changed...
            pstate = STATE_EXP1;
            break;

        case STATE_SR: // SR: Relative character size
            if(numpad_idx == 0) {
                hpgl_state.numpad[0] = 0.75f;
                hpgl_state.numpad[1] = 1.5f;
            }
            if((cmd = (numpad_idx == 1 ? CMD_ERR : CMD_SI)) == CMD_ERR)
                hpgl_set_error(ERR_WrongParams);
            else {
               user_point_t range = range_P1P2();
               hpgl_state.character_size.width = range.x * hpgl_state.numpad[0] / 100.0f;
               hpgl_state.character_size.height = range.y * hpgl_state.numpad[1] / 100.0f;
            }
            pstate = STATE_EXP1;
            break;

        case STATE_VS: // VS: Velocity Set
            cmd = CMD_VS;
            pstate = STATE_EXP1;
            break;

// Fetch parameters

        case STATE_X:
            switch (c) {
                case ' ':
                case '\n':
                case '\r':
                case '\t':
                    break;
                case ',':
                    if(si != 0) {
                        scratchpad[si] = '\0';
                        si = 0;
                        read_float(scratchpad, &si, &f.x);
                        si = 0;
                        pstate = STATE_Y;
                    }
                    break;
                case ';':
                    pstate = STATE_EXP1;
                    if(si > 0) {
                        cmd = CMD_ERR;
                        hpgl_set_error(ERR_WrongParams);
                    }
                    break;
                default:
                    if(valid_input(c, si))
                        scratchpad[si++] = c;
                    else if(c == 'P' && si == 0)
                        pstate = STATE_EXP_P;
                    break;
            }
            break;

        case STATE_Y:
            switch (c) {
                case ' ':
                case '\n':
                case '\r':
                case '\t':
                    break;
                case ',':
                case ';':
                    scratchpad[si] = '\0';
                    si = 0;
                    read_float(scratchpad, &si, &f.y);
                    si = 0;
                    if(hpgl_state.plot_relative)
                        userscalerelative(f, target, &hpgl_state.user_loc);
                    else
                        userscale(f, target, &hpgl_state.user_loc);
                    hpgl_state.cr_loc = hpgl_state.user_loc;
                    break;
                default:
                    if(valid_input(c, si))
                        scratchpad[si++] = c;
                    break;
            }
            switch (c) {
                case ',':
                    pstate = STATE_X;
                    break;
                case ';':
                    pstate = STATE_EXP1;
                    break;
            }
            break;

        case STATE_EXP4:
            switch (c) {
                case ' ':
                case '\n':
                case '\r':
                case '\t':
                      break;
                case ';':
                    if(si == 0) {
                        pstate = nstate;
                        goto lolbel;
                    }
                case ',':
                    scratchpad[si] = '\0';
                    si = 0;
                    read_float(scratchpad, &si, &f.x);
                    si = 0;
                    hpgl_state.numpad[numpad_idx] = f.x;
                    numpad_idx++;
                    if (numpad_idx == 4 || c == ';') {
                        pstate = nstate;
                        goto lolbel;
                    }
                    break;
                default:
                    if(valid_input(c, si))
                        scratchpad[si++] = c;
                    break;
            }
            break;
    }
    
    return cmd;
}
