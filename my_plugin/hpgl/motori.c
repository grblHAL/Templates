///\file motori.c
/// This is the main module of Mot&ouml;ri the Plotter firmware.
/// Basic I/O, initialization, interrupt handlers and main program loop are here.
///
/// For the main loop see do_stuff()
///
/// @author Viacheslav Slavinsky
///
/// \mainpage Plotter Firmware
/// \section Description
/// This is the firmware for Mot&ouml;ri the Plotter. It consists of basic ATmega644 I/O configuration code, 
/// low-level motor and pen control, basic linear motion algorithms and a scanner/parser that can
/// understand a limited subset of HPGL and translate it into motor steps. Linear motion is achieved by 
/// the means of Bresenham's line algorithm. 
///
/// To prevent shaking and mechanical oscillations motor speed is ramped up and down smoothly.
///
/// In my hardware the motors are very different (one from a Canon printer, other from Epson). The
/// Y-axis motor has lower resolution, or more travel per step. This is why there are not only separate
/// scale coefficients for X and Y axes, but X and Y maximal speeds are also different.
///
/// Since plotter is a very slow device, the communication with computer requires flow control.
/// This implementation uses RTS/CTS flow control, although the plotter wouldn't bother if the host
/// PC can't accept data.
///
/// \section The Code
///  - motori.h, motori.c   this is the main module with I/O, main loop, interrupts etc
///  - line.h               has the Bresenham's
///  - arc.h                converts an arc description into a series of chords
///  - hpgl.h               scans the input and parses commands
///  - usrat.h              serial i/o
///  - shvars.h             global variables shared among modules
///  - configs.h            global defines
///
/// See the Files section for the complete reference.
///
/// \section License
/// BSD License. Just use it. It will burn your motors though.
///
/// \section Permanent Location
/// This documentation, source code and other files can be found at http://sensi.org/~svo
///
/// Yours truly,\n
///    Viacheslav Slavinsky

// 2022-01-15 : Modified by Terje Io to make it a grblHAL plugin. Added EA command and refactored code.

#include <inttypes.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "shvars.h"
#include "arc.h"
#include "htext.h"
#include "scale.h"
#include "hpgl.h"
#include "motori.h"

#include "../grbl/hal.h"
#include "../grbl/protocol.h"
#include "../grbl/motion_control.h"
#include "../grbl/state_machine.h"


static volatile bool xoff = false;
static volatile pen_status_t pen_status = Pen_Unknown;          ///< pen status: 0 = up
static io_stream_t stream;
static enqueue_realtime_command_ptr enqueue_realtime_command;
static on_execute_realtime_ptr on_execute_realtime, process;
static on_report_options_ptr on_report_options;
static coord_data_t target = {0}, origin = {0};


#define SEEK0_NULL  0
#define SEEK0_SEEK  1
#define SEEK0_DONE  0x80
volatile uint8_t seeking0;              ///< state of seeking home position

float feed_rate = 1000; // mm/min

void grinding_halt() {  
    printf_P(PSTR("\n\007HALT\n"));
    for(;;);
}

void plotter_init()
{
    stepper_loc.x = stepper_loc.y = 0;
    user_loc.x = user_loc.y = 0.0f;

    text_init();
    translate_init();
    pen_control(Pen_Up);
    
    seeking0 = SEEK0_NULL;
}

// cm/sek -> mm/min
void set_speed (float value)
{
    xoff = false;
    feed_rate = value * 10.0f * 60.0f;
}

pen_status_t get_pen_status (void)
{
    return pen_status;
}

void pen_control (pen_status_t state)
{
    static const spindle_state_t on = { .on = true },  off = { .on = false };

    if (pen_status != state) {
        protocol_buffer_synchronize();
        sync_position();
        if (state == Pen_Down) {
            DELAY_MS(PEN_DOWN_DELAY);
            hal.spindle.set_state(on, 1000.0f);
            pen_status = Pen_Down;
        } else {
            hal.spindle.set_state(off, 0.0f);
            pen_status = Pen_Up;
        }

        DELAY_MS(PEN_LIFT_DELAY);
    }
}

static inline bool valid_target (hpgl_point_t target)
{
    return !(target.x < 0 || target.y < 0);
}

bool moveto (hpgl_coord_t x, hpgl_coord_t y)
{
    plan_line_data_t plan_data = {0};

    if(x < 0 || y < 0)
        return false;

//    if(x > 8000 || y > 8000)
//        return false;

    plan_data.feed_rate = feed_rate;
    plan_data.condition.rapid_motion = get_pen_status() == Pen_Up;

    target.x = origin.x + (float)x * 0.025f;
    target.y = origin.y + (float)y * 0.025f;
/*
    hal.stream.write(uitoa(x));
    hal.stream.write(",");
    hal.stream.write(uitoa(y));
    hal.stream.write(ASCII_EOL);
*/
    return mc_line(target.values, &plan_data);
}

void poll_stuff (sys_state_t state)
{
    on_execute_realtime(state);

    if(process) {
        process(state);
        return;
    }
}

/// Main loop routine.
///
/// Could be re-implemented as a state machine as complexity increases. So far there are only 3 states:
///     - Default: receive uart data, pass it to scanner and handle commands
///
///     - Arc tesselation: happens when an arc is being drawn
///
///     - Initialization: waits for the head to get home before doing a complete reset
void do_stuff (void)
{
    hpgl_point_t target;
    uint8_t labelchar;
    uint8_t finish_path = 0;
    static uint8_t initializing = 0;
    hpgl_command_t cmd = 0;

    target.x = target.y = -1;

    if (stream.get_rx_buffer_count()) {

        char c = stream.read();

        if(c == ASCII_CAN) {

            // Restore stream handling and exit back to normal operation

            protocol_buffer_synchronize();
            sync_position();

            memcpy(&hal.stream, &stream, sizeof(io_stream_t));
            hal.stream.set_enqueue_rt_handler(enqueue_realtime_command);

            stream.write("Bye..." ASCII_EOL);

            if(grbl.on_execute_realtime == poll_stuff) {
                grbl.on_execute_realtime = on_execute_realtime;
                on_execute_realtime = NULL;
            }

            return;
        }

//        if(c >= ' ')
//            hal.stream.write_char(c);

        switch((cmd = hpgl_char(c, &target, &labelchar))) {

            case CMD_PU:
                if (pen_status != 0) {
                    finish_path = 1;
                }
                break;

            case CMD_PD:
                if (pen_status != 1) {
                    finish_path = 2;
                }
                break;

            case CMD_PA:
                break;

            case CMD_EA:
                {
                    static hpgl_point_t origin;
                    if(valid_target(target)) {
                        finish_path = get_pen_status();
                        pen_control(Pen_Down);
                        moveto(target.x, origin.y);
                        moveto(target.x, target.y);
                        moveto(origin.x, target.y);
                        moveto(origin.x, origin.y);
                        target.x = -1;
                    } else {
                        origin.x = user_loc.x;
                        origin.y = user_loc.y;
                    }
                }
                break;

            case CMD_ARCABS:
                if(arc_init()) {
                    while(arc_next(&target))
                        moveto(target.x, target.y);
                    moveto(target.x, target.y);
                }
                break;

            case CMD_INIT:
                // 1. home
                // 2. init scale etc
                target.x = target.y = 0;
                initializing = 1;
                finish_path = 1;
                // Get current position.
                system_convert_array_steps_to_mpos(origin.values, sys.position);
                break;

            case CMD_SEEK0:
                seeking0 = SEEK0_SEEK;
                finish_path = 1;
                break;

            case CMD_LB0:
                finish_path = 1;
                text_beginlabel();
                break;

            case CMD_LB:
                if (labelchar != 0) {
                    pen_status_t penny;
                    //printf_P(PSTR("[%c]"), labelchar);
                    while(text_char(labelchar, &target, &penny)) {
                        labelchar = 0;
                        pen_control(penny);
                        moveto(target.x, target.y);
                    }
                    pen_control(penny);
                    moveto(target.x, target.y);
                    target.x = -1;
                }
                //text_active = 1;
                break;

            case CMD_SI:
                text_scale_cm(numpad[0], numpad[1]);
                break;

            case CMD_SR:
                text_scale_rel(numpad[0], numpad[1]);
                break;
            case CMD_DI:
                text_direction(numpad[0], numpad[1]);
                break;

            case CMD_VS:
                set_speed(numpad[0]);
                break;

            case CMD_AS:
                //set_acceleration_mode(numpad[0]);
                break;

            default:
                break;
        }
    }

//    if(cmd != CMD_CONT)
//        hal.stream.write(cmd == CMD_ERR ? "error" ASCII_EOL : "ok" ASCII_EOL);
/*
    if (!finish_path && dstx != -1 && dsty != -1) {
        // Path can be continued because pen state stays the same
        // Check if the angle permits doing so
        float a = path_angle_to(dstx, dsty);
#ifdef SIM
        fprintf(stderr, "angle=%3.1f\n", a);
#endif
#define CURVEANGLE 15
        if (a > CURVEANGLE) {// && a < (360-CURVEANGLE)) {
            finish_path = 3;
        }
    }
*/
/*
OutputGCode("G1X&1Y&2F&4":U, vector.x1, vector.y1, ?, feedrate).
OutputGCode("G3X&1Y&2F&4":U, vector.x2, vector.y2, (vector.x2 - vector.x1) / 2.0, (vector.y2 - vector.y1) / 2.0, ?, feedrate).
 */
    if (finish_path) {
        protocol_buffer_synchronize();
        sync_position();
        switch (finish_path) {
            case 1:
                pen_control(Pen_Up);
                break;    /* pen up */
            case 2:
                pen_control(Pen_Down);
                break;    /* pen down */
            case 3:
                break;                    /* sharp angle */
        }
    }

    if(valid_target(target))
        moveto(target.x, target.y);
}

static void wait (sys_state_t state)
{
    //NOOP
}

void stream_write_null (const char *s)
{
    UNUSED(s);
}

static void await_homed (sys_state_t state)
{
    if(state_get() == STATE_IDLE) {
        sync_position();
        process = NULL;

        // Get current position.
        system_convert_array_steps_to_mpos(origin.values, sys.position);

        plotter_init();

        hal.stream.write = stream.write;
        hal.stream.write_all = stream.write_all;
        hal.stream.write("Ready..." ASCII_EOL);
    }
}

static void go_home (void)
{
    char cmd[LINE_BUFFER_SIZE] = "$H";
    plan_line_data_t plan_data = {0};

    process = wait;
    hal.stream.write = hal.stream.write_all = stream_write_null;

    system_execute_line(cmd);

    plan_data.feed_rate = 4000.0f;
    target.values[X_AXIS] = settings.axis[X_AXIS].max_travel; //
    target.values[Y_AXIS] = 0.0f;
    process = mc_line(target.values, &plan_data) ? await_homed : NULL;
}


static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:HPGL v0.01]" ASCII_EOL);
}

volatile uint16_t rx_count, xoff_count = 0, xon_count = 0;

int16_t stream_get_data (void)
{
    rx_count = stream.get_rx_buffer_count();

    if (process == NULL && rx_count) {

        do_stuff();

        if(xoff && rx_count < 400) {
            xoff = false;
            xon_count++;
            hal.stream.write_char(ASCII_XON);
        }
    }

    return SERIAL_NO_DATA;
}

ISR_CODE bool ISR_FUNC(stream_insert_buffer)(char c)
{
    if(!xoff) {
        if((xoff = stream.get_rx_buffer_free() < 200)) {
            hal.stream.write_char(ASCII_XOFF);
            xoff_count++;
        }
    }

    return false;
}

status_code_t hpgl_start (sys_state_t state, char *args)
{
    plotter_init();

    hpgl_init();

    seeking0 = SEEK0_SEEK; // SEEK0_SEEK; SEEK0_SEEK = home?

    memcpy(&stream, &hal.stream, sizeof(io_stream_t));
    hal.stream.read = stream_get_data;
    enqueue_realtime_command = hal.stream.set_enqueue_rt_handler(stream_insert_buffer);

    stream.write("Motori HPGL v0.01" ASCII_EOL);

    if(on_execute_realtime == NULL) {
        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = poll_stuff;
    }

    go_home();

    return Status_OK;
}

const sys_command_t hpgl_command_list[] = {
    {"HPGL", true, hpgl_start}
};

static sys_commands_t hpgl_commands = {
    .n_commands = sizeof(hpgl_command_list) / sizeof(sys_command_t),
    .commands = hpgl_command_list
};

sys_commands_t *hpgl_get_commands()
{
    return &hpgl_commands;
}

void my_plugin_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    hpgl_commands.on_get_commands = grbl.on_get_commands;
    grbl.on_get_commands = hpgl_get_commands;
}
