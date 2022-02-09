#include <stdio.h>
#include <math.h>
#include <string.h>

#include "hpgl.h"
#include "scale.h"

#include "../grbl/stream.h"
#include "../grbl/nuts_bolts.h"

extern const char *const charset0[256];
extern const char *const charset173[256];

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
    .use_alt_charset = false,
    .text_vertical = false,
    .charset = &charset0[0],
    .charset_std = &charset0[0],
    .charset_alt = &charset173[0],
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
hpgl_char_ptr hpgl_char;

static hpgl_command_t hpgl_char_inp (char c, hpgl_point_t *target, uint8_t *lb);
static hpgl_command_t hpgl_char_esc (char c, hpgl_point_t *target, uint8_t *lb);

static char scratchpad[SCRATCHPAD_SIZE];

__attribute__((weak)) void alert_led (bool on)
{
}

void hpgl_init (void)
{
//    pstate = STATE_EXP1;
    hpgl_char = hpgl_char_inp;

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

bool is_numeric (char c, uint_fast8_t idx)
{
    // TODO: add check for single decimal point in input?
    return idx < sizeof(scratchpad) - 2 && ((c >= '0' && c <= '9') || c == '.' || (idx == 0 && (c == '+' || c == '-')));
}

bool is_whitespace (char c)
{
    return c == ' ' || c == '\n' || c == '\r' || c == 't';
}

bool is_command_char (char c)
{
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

bool is_separator (char *c)
{
    bool ok;

    if((ok = *c == ',' || *c == ' ' || *c == '+' || *c == '-'))
        *c = '\0';
        
    return ok;
}

bool is_terminator (char *c)
{
    bool ok;
    
    if((ok = *c == ';'))
        *c = '\0';
    
    return ok || is_command_char(*c);
}

bool get_parameter (char *c, uint_fast8_t *idx)
{
    if(is_numeric(*c, *idx)) {
        scratchpad[*idx] = *c;
        (*idx)++;
        *c = '\0';
    }
    
    return *idx > 0 && *c != '\0' && ((is_terminator(c)) || is_separator(c));
}

hpgl_command_t get_instruction (char c)
{
    static hpgl_command_t command = 0;

    hpgl_command_t instruction = 0;

    if(is_command_char(c)) {
        command = command << 8 | CAPS(c);
    } else if(!is_whitespace(c))
        command = 0;

    if(command > 255) {
        instruction = command;
        command = 0;
    }

    return instruction;
}

hpgl_command_t hpgl_char_inp (char c, hpgl_point_t *target, uint8_t *lb)
{
    static uint_fast8_t si, numpad_idx;
    static hpgl_command_t command = CMD_CONT;

    static bool is_plotting = false, is_labeling = false, is_labelterminator = false;

    *lb = 0;
    target->x = target->y = -1;

    if(c == ASCII_ESC) {
        hpgl_char = hpgl_char_esc;
        return CMD_CONT;
    }

    if(is_labeling) {
        if((is_labeling = c != hpgl_state.etxchar)) {
            *lb = c;
            return command;
        }
        return command = CMD_CONT;
    }

    if(is_labelterminator) { // DT: Set text terminator
        hpgl_state.etxchar = c;
        is_labelterminator = false;
        return command = CMD_CONT;
    }

    if(is_whitespace(c))
        return CMD_CONT;

    char t = c;
    bool terminated = false;
    hpgl_command_t cmd = CMD_CONT;

    if(command == CMD_CONT) {
        t = '\0';
        if((command = get_instruction(c)) != CMD_CONT) {

            si = 0;
            is_plotting = command == CMD_PA || command == CMD_PD || command == CMD_PR || command == CMD_PU;
            is_labelterminator = command == CMD_DT;

            if(command == CMD_PA || command == CMD_PR)
                hpgl_state.plot_relative = command == CMD_PR;
 
            if((is_labeling = command == CMD_LB))
                cmd = CMD_LB0;

            numpad_idx = 4;
            do {
                hpgl_state.numpad[--numpad_idx] = 0.0f;
            } while(numpad_idx);
        }
    } else if(get_parameter(&c, &si)) {

        if((terminated = is_command_char(c)))
            get_instruction(c);

        scratchpad[si] = '\0';
        si = 0;

        if(numpad_idx < sizeof(hpgl_state.numpad)) {
            read_float(scratchpad, &si, &hpgl_state.numpad[numpad_idx++]);
            si = 0;
        }

        if(is_plotting && numpad_idx == 2)
            t = ';';

    } else if((terminated = is_command_char(c)))
        get_instruction(c);

    if(t && is_terminator(&t)) {

        switch(command) {

            case CMD_AA:
                cmd = CMD_AA;
                if (numpad_idx == 3)
                    hpgl_state.numpad[3] = hpgl_state.chord_angle;
                cmd = command;
                break;

            case CMD_AR: // AR: Arc Relative - xc, yc, CentralAngle [,Degrees per step]
                if(numpad_idx < 3) {
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_WrongParams);
                } else {
                    if (numpad_idx == 3)
                        hpgl_state.numpad[3] = hpgl_state.chord_angle;
                    user_point_t f, offset = { 0.0f, 0.0f };
                    f.x = hpgl_state.numpad[0];
                    f.y = hpgl_state.numpad[1];
                    userscalerelative(f, target, &offset);
                    hpgl_state.numpad[0] = hpgl_state.user_loc.x + offset.x;
                    hpgl_state.numpad[1] = hpgl_state.user_loc.y + offset.y;
                    target->x = target->y = -1;
                    cmd = command;
                }
                break;

            case CMD_CA: // CA: Designate alternate character set
                if(numpad_idx == 0) {
                    hpgl_state.numpad[0] = 0.0f;
                    numpad_idx = 1;
                }
                if(numpad_idx == 1) {
                    hpgl_state.charset_alt = hpgl_state.numpad[0] == 0.0f ? &charset0[0] : &charset173[0];
                    if(hpgl_state.use_alt_charset)
                        hpgl_state.charset = hpgl_state.charset_alt;
                } else {
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_UnknownCharset);
                }
                break;

            case CMD_CI: // CI: Circle - radius [,Degrees per step]
                if(numpad_idx > 0) {
                    if(numpad_idx == 1)
                        hpgl_state.numpad[1] = hpgl_state.chord_angle;
                    userscale(hpgl_state.user_loc, target, &hpgl_state.user_loc);
                    cmd = command;
                } else {
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_WrongParams);
                }
                break;

            case CMD_CP: // CP: Character Plot
                if(numpad_idx == 0) {
                    hpgl_state.numpad[0] = NAN;
                    hpgl_state.numpad[1] = NAN;
                }
                if(numpad_idx == 1)
                    hpgl_state.numpad[1] = 0.0f;
                cmd = command;
                break;

            case CMD_CS: // CA: Designate standard character set
                if(numpad_idx == 0) {
                    hpgl_state.numpad[0] = 0.0f;
                    numpad_idx = 1;
                }
                if(numpad_idx == 1) {
                    hpgl_state.charset_std = hpgl_state.numpad[0] == 0.0f ? &charset0[0] : &charset173[0];
                    if(!hpgl_state.use_alt_charset)
                        hpgl_state.charset = hpgl_state.charset_std;
                } else {
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_UnknownCharset);
                }
                break;
                
            case CMD_DF: // DF: Set default values
                hpgl_defaults();
                cmd = command;
                break;

            case CMD_DI: // DI: Absolute text direction // numpad contains sin(theta), cos(theta)
                if(numpad_idx == 0) {
                    hpgl_state.numpad[0] = 1.0f;
                    hpgl_state.numpad[1] = 0.0f;
                    numpad_idx = 2;
                }
                if(numpad_idx >= 2) {
                // fabs(hpgl_state.numpad[0] + hpgl_state.numpad[1]) > 0.0004f
                }
                cmd = command;
                break;

            case CMD_DV: // DV: Vertical label direction
                if(numpad_idx == 1) {
                    hpgl_state.text_vertical = truncf(hpgl_state.numpad[0]) != 0.0f;
                } else {
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_WrongParams);
                }
                break;

            case CMD_EA: // EA: Edge Absolute
                if(numpad_idx < 2) {
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_WrongParams);
                } else {
                    user_point_t f;
                    f.x = hpgl_state.numpad[0];
                    f.y = hpgl_state.numpad[1];
                    userscale(f, target, NULL);
                    if(numpad_idx > 2)
                        hpgl_set_error(ERR_WrongParams);
                    cmd = command;
                }
                break;

            case CMD_ER: // ER: Edge Relative
                if(numpad_idx < 2) {
                    cmd = CMD_ERR;
                    hpgl_set_error(ERR_WrongParams);
                } else {
                    user_point_t f;
                    f.x = hpgl_state.user_loc.x + hpgl_state.numpad[0];
                    f.y = hpgl_state.user_loc.y + hpgl_state.numpad[1];
                    userscale(f, target, NULL);
                    if(numpad_idx > 2)
                        hpgl_set_error(ERR_WrongParams);
                    cmd = command;
                }
                break;

            case CMD_ES: // ES: Extra Space
                if(numpad_idx == 0) {
                    hpgl_state.numpad[0] = 0.0f;
                    hpgl_state.numpad[1] = 0.0f;
                    numpad_idx = 2;
                } else {
// TODO:
                    if(numpad_idx > 2)
                        hpgl_set_error(ERR_WrongParams);
                    cmd = command;
                }
                break;

            case CMD_EW: // EW: Edge Wedge
                if(numpad_idx > 0) {
                    if(numpad_idx < 3) {
                        cmd = CMD_ERR;
                        hpgl_set_error(ERR_WrongParams);
                    } else {
                        if (numpad_idx == 3)
                            hpgl_state.numpad[3] = hpgl_state.chord_angle;
/*                        user_point_t f, offset = { 0.0f, 0.0f };
                        f.x = hpgl_state.numpad[0];
                        f.y = hpgl_state.numpad[1];
                        userscalerelative(f, target, &offset);
                        hpgl_state.numpad[0] = hpgl_state.user_loc.x + offset.x;
                        hpgl_state.numpad[1] = hpgl_state.user_loc.y + offset.y;
                        target->x = target->y = -1; */
                        cmd = command;
                    }
                }
                break;

              case CMD_IM: // IM: Input Mask
                if(numpad_idx > 0 && hpgl_state.numpad[0] < 256.0f) {
                    hpgl_state.alertmask = truncf(hpgl_state.numpad[0]);
                    alert_led(!!(hpgl_state.errmask & hpgl_state.alertmask));
                }
                break;

            case CMD_IN: // IN: Initialize plotter
                hpgl_set_error(ERR_None);
                cmd = command;
                break;

            case CMD_IP: // IP: Initialize plotter
                if(numpad_idx == 0)
                    translate_init_ip();
                else if(numpad_idx == 2 || numpad_idx == 4) {
                    bool valid;
                    uint_fast8_t i = numpad_idx;
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
                cmd = command;
                break;

            case CMD_LB: // LB: Label character
                // scanning text
                cmd = CMD_LB;
                if (!(is_labeling = c != hpgl_state.etxchar)) {
                    *lb = 0;
                    command = CMD_CONT;
                } else {
                    target->x = target->y = -1;
                    *lb = c;
                }
                cmd = command;
                break;

            case CMD_LT: // LT: Line Type
                if(numpad_idx == 0) {
                    hpgl_state.pattern_type = 0;
                    hpgl_state.pattern_length = 4.0f;
                    cmd = command;
                } else if((cmd = fabsf(hpgl_state.numpad[0]) >= 128.0f ? CMD_ERR : command) == command) {
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
                break;

            case CMD_OP: // OP: Output Parameters P1 & P2
                output_P1P2();
                break;

            case CMD_PA:
            case CMD_PD:
            case CMD_PR:
            case CMD_PU:
                switch(numpad_idx) {
                    case 0:
                        cmd = command;
                        command = CMD_CONT;
                        break;
                    case 2:
                        {
                            user_point_t f;
                            f.x = hpgl_state.numpad[0];
                            f.y = hpgl_state.numpad[1];
                            if(hpgl_state.plot_relative)
                                userscalerelative(f, target, &hpgl_state.user_loc);
                            else
                                userscale(f, target, &hpgl_state.user_loc);
                            cmd = command;
                        }
                        break;
                    default:
                        cmd = CMD_ERR;
                        command = CMD_CONT;
                        hpgl_set_error(ERR_BadParam);
                        break;
                }
                break;

            case CMD_PT: // PT: pen thickness
                cmd = command;
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
                break;

            case CMD_SA: // SA: select alternate charset
                hpgl_state.use_alt_charset = true;
                hpgl_state.charset = hpgl_state.charset_alt;
                break;

            case CMD_SC: // Scale
                if(numpad_idx == 0)
                    translate_init_sc();
                else if(numpad_idx == 4) {
                    for (uint_fast8_t i = 0; i < 4; i++)
                        hpgl_state.sc_pad[i] = (int32_t)truncf(hpgl_state.numpad[i]);
                    translate_scale();
                }
                cmd = numpad_idx == 0 || numpad_idx == 4 ? command : CMD_ERR;
                break;

            case CMD_SI: // SI: Absolute character size
                if(numpad_idx == 0) {
                    hpgl_state.numpad[0] = 0.187f;
                    hpgl_state.numpad[1] = 0.269f;
                    numpad_idx = 2;
                }
                if((cmd = (numpad_idx == 1 ? CMD_ERR : command)) == CMD_ERR)
                    hpgl_set_error(ERR_WrongParams);
                else {
                    hpgl_state.character_size.width = hpgl_state.numpad[0];
                    hpgl_state.character_size.height = hpgl_state.numpad[1];
                    if(numpad_idx > 2)
                        hpgl_set_error(ERR_WrongParams);
                }
                break;

            case CMD_SP: // SP: Select Pen
                if(numpad_idx == 0)
                    hpgl_state.numpad[0] = 0.0f;
                hpgl_state.pen_thickness = 0.3; // only if pen changed...
                cmd = command;
                break;

            case CMD_SR: // SR: Relative character size
                if(numpad_idx == 0) {
                    hpgl_state.numpad[0] = 0.75f;
                    hpgl_state.numpad[1] = 1.5f;
                    numpad_idx = 2;
                }
                if((cmd = (numpad_idx == 1 ? CMD_ERR : command)) == CMD_ERR)
                    hpgl_set_error(ERR_WrongParams);
                else {
                    user_point_t range = range_P1P2();
                    hpgl_state.character_size.width = range.x * hpgl_state.numpad[0] / 100.0f;
                    hpgl_state.character_size.height = range.y * hpgl_state.numpad[1] / 100.0f;
                    if(numpad_idx > 2)
                        hpgl_set_error(ERR_WrongParams);
                }
                break;

            case CMD_SS: // SA: select standard charset
                hpgl_state.use_alt_charset = false;
                hpgl_state.charset = hpgl_state.charset_std;
                break;

            case CMD_VS: // VS: Velocity Set
                cmd = numpad_idx ? command : CMD_ERR;
                break;

            default:
                cmd = CMD_ERR;
                hpgl_set_error(ERR_UnknownCommand);
                break;
        }

        numpad_idx = si = 0;
        if(terminated || !is_plotting || cmd == CMD_ERR)
            command = CMD_CONT;
    }

    return cmd;
}

hpgl_command_t hpgl_char_esc (char c, hpgl_point_t *target, uint8_t *lb)
{
    static uint_fast8_t idx = 0;
    static char scratch[SCRATCHPAD_SIZE];

    hpgl_command_t cmd = CMD_CONT;

    *lb = 0;
    target->x = target->y = -1;

    if(c == '.') {
        idx = 0;
    } else switch (c) {

        case 'B':
            cmd = CMD_BUFFER_SPACE;
            break;

        case 'L':
            cmd = CMD_BUFFER_SIZE;
            break;

        default:
            cmd = CMD_ERR;
            break;
    }

    if(cmd != CMD_CONT)
        hpgl_char = hpgl_char_inp;

    return cmd;
}
