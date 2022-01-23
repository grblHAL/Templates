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
///  - arc.h                converts an arc description into a series of chords
///  - hpgl.h               scans the input and parses commands
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

// 2022-01-21 : Modified by Terje Io to make it a grblHAL plugin. Added many commands and refactored code.

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

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
static uint32_t last_action = 0;
static volatile pen_status_t pen_status = Pen_Unknown;          ///< pen status: 0 = up
static io_stream_t stream;
static enqueue_realtime_command_ptr enqueue_realtime_command;
static on_execute_realtime_ptr on_execute_realtime, process;
static on_report_options_ptr on_report_options;
static on_state_change_ptr on_state_change;
static coord_data_t target = {0}, origin = {0};


#define SEEK0_NULL  0
#define SEEK0_SEEK  1
#define SEEK0_DONE  0x80

float feed_rate = 1000; // mm/min

bool moveto (hpgl_coord_t x, hpgl_coord_t y);

__attribute__((weak)) void select_pen (uint_fast16_t pen)
{
    if(pen == 0) {
        hpgl_state.user_loc.x = hpgl_state.user_loc.y = 0.0f;
        moveto(0, 0);
    }
}

__attribute__((weak)) void pen_led (bool on)
{
}

__attribute__((weak)) bool is_plotter_online (void)
{
    return true;
}

coord_data_t *get_origin (void)
{
    return &origin;
}

void grinding_halt() {  
    printf_P(PSTR("\n\007HALT\n"));
    for(;;);
}

void plotter_init()
{
    hpgl_init();
    text_init();
    pen_control(Pen_Up);
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

        if (state != Pen_Timeout) {
            protocol_buffer_synchronize();
            sync_position();
        }

        if (state == Pen_Down) {
            DELAY_MS(PEN_DOWN_DELAY);
            hal.spindle.set_state(on, 1000.0f);
            pen_status = Pen_Down;
            last_action = hal.get_elapsed_ticks();
        } else {
            hal.spindle.set_state(off, 0.0f);
            pen_status = Pen_Up;
        }

        pen_led(pen_status == Pen_Down);

        if (state != Pen_Timeout)
            DELAY_MS(PEN_LIFT_DELAY);
    }
}

static inline bool valid_target (hpgl_point_t target)
{
    return !(target.x < 0 || target.y < 0 || target.x > MAX_X || target.y > MAX_Y);
}

bool moveto (hpgl_coord_t x, hpgl_coord_t y)
{
    plan_line_data_t plan_data = {0};

    if(x < hpgl_state.ip_pad[0] || y < hpgl_state.ip_pad[1])
        return false;

    if(x > hpgl_state.ip_pad[2] || y > hpgl_state.ip_pad[3])
       return false;

    plan_data.feed_rate = feed_rate;
    plan_data.condition.rapid_motion = get_pen_status() == Pen_Up;

    target.x = origin.x + (float)x * 0.025f;
    target.y = origin.y + (float)y * 0.025f;

#ifdef HPGL_DEBUG
    hal.stream.write(uitoa(x));
    hal.stream.write(",");
    hal.stream.write(uitoa(y));
    hal.stream.write(ASCII_EOL);
#endif

    last_action = hal.get_elapsed_ticks();

    return mc_line(target.values, &plan_data);
}

void state_changed (sys_state_t state)
{   
    static sys_state_t prev_state = STATE_IDLE;

    if(state == STATE_IDLE && prev_state == STATE_JOG) {
        coord_data_t position;
        system_convert_array_steps_to_mpos(position.values, sys.position);
        hpgl_state.user_loc.x = (position.x - origin.x) / 0.025f;
        hpgl_state.user_loc.y = (position.y - origin.y) / 0.025f;
    }

    if(state == STATE_IDLE || state == STATE_JOG)
        prev_state = state;
}

void poll_stuff (sys_state_t state)
{
    on_execute_realtime(state);

    if(process) {
        process(state);
        return;
    }

    // If no motion for 55s lift pen
    if(get_pen_status() == Pen_Down && (hal.get_elapsed_ticks() - last_action) >= 55000)
        pen_control(Pen_Timeout);
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
    pen_status_t on_finish_path = Pen_NoAction;
    hpgl_command_t cmd = 0;

    target.x = target.y = -1;

    if (stream.get_rx_buffer_count()) {

        char c = stream.read();

        if(c == ASCII_CAN) {

            // Restore stream handling and exit back to normal operation

            protocol_buffer_synchronize();
            sync_position();

            pen_control(Pen_Up);

            memcpy(&hal.stream, &stream, sizeof(io_stream_t));
            hal.stream.set_enqueue_rt_handler(enqueue_realtime_command);

            stream.write("Bye..." ASCII_EOL);

            if(grbl.on_execute_realtime == poll_stuff) {
                grbl.on_execute_realtime = on_execute_realtime;
                on_execute_realtime = NULL;
            }

            return;
        }

        if(!is_plotter_online())
            return;

//        if(c >= ' ')
//            hal.stream.write_char(c);

        switch((cmd = hpgl_char(c, &target, &labelchar))) {

            case CMD_AA:
            case CMD_AR: // AR: Arc relative
                if(arc_init()) {
                    while(arc_next(&target))
                        moveto(target.x, target.y);
                    moveto(target.x, target.y);
                }
                break;

            case CMD_AS:
                //set_acceleration_mode(hpgl_state.numpad[0]);
                break;

            case CMD_CI:
                {
                    hpgl_point_t point;
                    user_point_t org = hpgl_state.user_loc;
                    if(circle_init(&point)) {
                        on_finish_path = get_pen_status();
                        pen_control(Pen_Up);
                        moveto(point.x, point.y);
                        pen_control(Pen_Down);
                        while(arc_next(&point))
                            moveto(point.x, point.y);
                        moveto(point.x, point.y);
                        pen_control(Pen_Up);
                        moveto(target.x, target.y);
                        target.x = -1;
                        hpgl_state.user_loc = org;
                    }
                }
                break;

            case CMD_DI:
                text_direction(hpgl_state.numpad[0], hpgl_state.numpad[1]);
                break;

            case CMD_EA:
            case CMD_ER:
                {
                    on_finish_path = get_pen_status();
                    pen_control(Pen_Down);
                    moveto(target.x, hpgl_state.user_loc.y);
                    moveto(target.x, target.y);
                    moveto(hpgl_state.user_loc.x, target.y);
                    moveto(hpgl_state.user_loc.x, hpgl_state.user_loc.y);
                    target.x = -1;
                }
                break;

            case CMD_IN:
                // 1. home
                // 2. init scale etc
                target.x = target.y = 0;
                on_finish_path = Pen_Up;
                break;

            case CMD_LB0:
                on_finish_path = Pen_Down;
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

            case CMD_PA:
            case CMD_PR:
                break;

            case CMD_PD:
                if (get_pen_status() != Pen_Down)
                    on_finish_path = Pen_Down;
                break;

            case CMD_PU:
                if (get_pen_status() != Pen_Up)
                    on_finish_path = Pen_Up;
                break;

            case CMD_SEEK0:
               // go_home();
                break;

            case CMD_SI:
                text_scale_cm(hpgl_state.numpad[0], hpgl_state.numpad[1]);
                break;

            case CMD_SP: // Select pen
                on_finish_path = Pen_Up;
                break;

            case CMD_SR:
                text_scale_rel(hpgl_state.numpad[0], hpgl_state.numpad[1]);
                break;

            case CMD_VS:
                set_speed(hpgl_state.numpad[0]);
                break;

            case CMD_ERR:
//                hal.stream.write("error ");
//                hal.stream.write(uitoa((uint32_t)hpgl_get_error()));
//                hal.stream.write(ASCII_EOL);
                break;

            default:
                break;
        }
    }

    if (on_finish_path != Pen_NoAction) {
        protocol_buffer_synchronize();
        sync_position();
        pen_control(on_finish_path);
    }

    switch(cmd) {

        case CMD_IN:
            plotter_init();
            // Get current position.
            system_convert_array_steps_to_mpos(origin.values, sys.position);
            break;

        case CMD_SP: // Select pen
            select_pen((uint_fast16_t)truncf(hpgl_state.numpad[0]));
            break;

        default:
            break;
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
        hal.stream.write("[PLUGIN:HPGL v0.02" ASCII_EOL);
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

    if(c == CMD_JOG_CANCEL && (state_get() & STATE_JOG))
        system_set_exec_state_flag(EXEC_MOTION_CANCEL);

    return false;
}

status_code_t hpgl_start (sys_state_t state, char *args)
{
    plotter_init();

    memcpy(&stream, &hal.stream, sizeof(io_stream_t));
    hal.stream.read = stream_get_data;
    enqueue_realtime_command = hal.stream.set_enqueue_rt_handler(stream_insert_buffer);

    stream.write("Motori HPGL v0.02" ASCII_EOL);

    if(on_execute_realtime == NULL) {
        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = poll_stuff;
        on_state_change = grbl.on_state_change;
        grbl.on_state_change = state_changed;
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
