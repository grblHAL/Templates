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

// 2022-01-25 : Modified by Terje Io to make it a grblHAL plugin. Added many commands and refactored code.

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "arc.h"
#include "htext.h"
#include "scale.h"
#include "hpgl.h"
#include "motori.h"

#include "grbl/protocol.h"
#include "grbl/motion_control.h"
#include "grbl/state_machine.h"

#define VERSION "v0.07"
#define DC_VALUES_MAX 12

typedef enum {
    IOError_None = 0,
    IOError_DCI_Overlap = 10,
    IOError_DCI_InvalidCmd = 11,
    IOError_DCI_InvalidData = 12,
    IOError_OutOfRange = 13,
    IOError_TooManyParameters = 14,
    IOError_FormatError = 15,
    IOError_Overflow = 16
} io_error_t;


static char pollc = 0;
static float feed_rate = 1000; // mm/min
static volatile uint16_t rx_count;
static volatile bool xoff = false;
static uint32_t last_action = 0;
static volatile pen_status_t pen_status = Pen_Unknown;          ///< pen status: 0 = up
static io_stream_t stream;
static enqueue_realtime_command_ptr enqueue_realtime_command, base_handler;
static on_execute_realtime_ptr on_execute_realtime, process;
static on_report_options_ptr on_report_options;
static on_state_change_ptr on_state_change;
static coord_data_t target = {0}, origin = {0};
static struct {
    uint_fast8_t i;
    uint_fast8_t j;
    io_error_t error;
    char cmd;
    char value[DC_VALUES_MAX][9];
} dc_values = {0};
static struct {
    uint_fast16_t block_size;
    uint_fast16_t xoff_threshold;
    uint_fast16_t xon_level;
    uint_fast16_t turnaround_delay;
    uint_fast16_t intercharacter_delay;
    uint_fast8_t handshake_mode;
    char enquiry;
    char output_trigger;
    char echo_terminator;
    char output_initiator;
    char xon_ack_response[11];
    char xoff_immediate_response[11];
} dc_data = {0};

#define SEEK0_NULL  0
#define SEEK0_SEEK  1
#define SEEK0_DONE  0x80

static void go_home (void);
static ISR_CODE bool ISR_FUNC(stream_insert_buffer)(char c);
static ISR_CODE bool ISR_FUNC(stream_insert_buffer_xoff)(char c);
static ISR_CODE bool ISR_FUNC(stream_insert_buffer_enq)(char c);

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

__attribute__((weak)) void online_led (bool on)
{
}

__attribute__((weak)) void alert_led (bool on)
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
#if GRBL_BUILD >= 20230201
            spindle_get(0)->set_state(on, 1000.0f);
#else
            hal.spindle.set_state(on, 1000.0f);
#endif
            pen_status = Pen_Down;
            last_action = hal.get_elapsed_ticks();
        } else {
#if GRBL_BUILD >= 20230201
            spindle_get(0)->set_state(off, 0.0f);
#else
            hal.spindle.set_state(off, 0.0f);
#endif
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
    plan_line_data_t plan_data;

    if(x < hpgl_state.ip_pad[0] || y < hpgl_state.ip_pad[1])
        return false;

    if(x > hpgl_state.ip_pad[2] || y > hpgl_state.ip_pad[3])
       return false;

    plan_data_init(&plan_data);
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
void do_stuff (char c)
{
    hpgl_point_t target;
    uint8_t labelchar;
    pen_status_t on_finish_path = Pen_NoAction;
    hpgl_command_t cmd = 0;

    target.x = target.y = -1;

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
        pollc = 0;
        return;
    }

    if(!is_plotter_online()) {
        pollc = 0;
        return;
    }

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

        case CMD_CP:
            text_pos(hpgl_state.numpad[0], hpgl_state.numpad[1], &target);
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

        case CMD_EW: // EW: Edge Wedge
            if(wedge_init()) {
                while(arc_next(&target)) {
                    moveto(target.x, target.y);
                            protocol_buffer_synchronize();
                }
                moveto(target.x, target.y);
            }
            break;

        case CMD_IN:
            // 1. home
            // 2. init scale etc
#ifdef GO_HOME_ON_IN
             if(settings.homing.flags.enabled)
                go_home();
#endif
            target.x = target.y = 0;
            on_finish_path = Pen_Up;
            break;

        case CMD_LB0:
            on_finish_path = Pen_Up;
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
            go_home();
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

    pollc = 0;
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
    static bool run_ok = false;

    if(state == STATE_CYCLE)
        run_ok = true;

    if(run_ok && state != STATE_CYCLE) {

        process = NULL;
        run_ok = false;

        if(state == STATE_IDLE) {

            sync_position();

            // Get current position.
            system_convert_array_steps_to_mpos(origin.values, sys.position);

            plotter_init();
        }

        hal.stream.write = stream.write;
        hal.stream.write_all = stream.write_all;
        hal.stream.write(state == STATE_IDLE ? "Ready..." ASCII_EOL : "Failed..." ASCII_EOL);
    }
}

static void go_home (void)
{
    char cmd[LINE_BUFFER_SIZE] = "$H";
    plan_line_data_t plan_data;

    process = wait;
    hal.stream.write = hal.stream.write_all = stream_write_null;

    pen_control(Pen_Up);
    system_execute_line(cmd);

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    target.values[X_AXIS] = settings.axis[X_AXIS].max_travel; //
    target.values[Y_AXIS] = 0.0f;
    process = mc_line(target.values, &plan_data) ? await_homed : NULL;
}

//
// Device Control Instructions handling (ESC . ...)
//

void report_buffer_free (sys_state_t state)
{
    hal.stream.write(uitoa(hal.stream.get_rx_buffer_free()));
    hal.stream.write(ASCII_EOL);
}

void report_buffer_size (sys_state_t state)
{
    hal.stream.write(uitoa(hal.rx_buffer_size - 1));
    hal.stream.write(ASCII_EOL);
}

void report_extended_error (sys_state_t state)
{
    hal.stream.write(uitoa(dc_values.error));
    hal.stream.write(ASCII_EOL);

    if(dc_values.error == IOError_None)
        alert_led(false);

    dc_values.error = IOError_None; //??
}

void report_extended_status (sys_state_t state)
{
    union {
        uint8_t status;
        struct {
            uint8_t unused       :2,
                    buffer_empty :1,
                    ready        :2,
                    unused2      :3;
        };
    } hpgl_status = {0};
    
    hpgl_status.buffer_empty = hal.stream.get_rx_buffer_free() == hal.rx_buffer_size - 1;

    hal.stream.write(uitoa(hpgl_status.status));
    hal.stream.write(ASCII_EOL);
}

static inline bool get_value (char *s, int32_t *v)
{
    bool ok;
    float val;
    uint_fast8_t cc = 0;

    if((ok = read_float(s, &cc, &val)))
        *v = (int32_t)val;

    return ok;
}

static ISR_CODE void ISR_FUNC(set_handshake_mode)(void)
{
    if(dc_data.enquiry)
        base_handler = stream_insert_buffer_enq;
    else if(*dc_data.xon_ack_response && *dc_data.xoff_immediate_response)
        base_handler = stream_insert_buffer_xoff;
    else
        base_handler = stream_insert_buffer;

    hpgl_state.comm.enable_dtr = base_handler == stream_insert_buffer && stream_get_flags(hal.stream).rts_handshake;
    hal.stream.set_enqueue_rt_handler(base_handler);
}

static ISR_CODE void ISR_FUNC(process_dci)(void)
{
    int32_t val[DC_VALUES_MAX];
    bool errors = false, error[DC_VALUES_MAX], present[DC_VALUES_MAX] = {0};
    uint_fast8_t i = dc_values.j;

    if(i > 0) {
        do {
            i--;
            if((present[i] = *dc_values.value[i])) {
                if((error[i] = !get_value(dc_values.value[i], &val[i])))
                    errors = true;
            }
        } while(i);
    }

    if(errors)
        dc_values.error = IOError_DCI_InvalidData;

    else switch(dc_values.cmd) {

        case '@':
            // val[0]: optional max buffer size

            if(present[1] && val[1] <= 255)
                hpgl_state.comm.value = (uint8_t)val[1];
            else
                dc_values.error = IOError_OutOfRange;
            break;

        case 'H':
        case 'I':
            if((dc_data.enquiry = present[1] ? val[1] : 0))
                dc_data.block_size = present[0] ? min(val[0], hal.rx_buffer_size) : 80;
            else {
                dc_data.xoff_threshold = present[0] ? min(val[0], hal.rx_buffer_size) : 0;
                if(dc_data.xoff_threshold > 512)
                    dc_data.xon_level = hal.rx_buffer_size - (dc_data.xoff_threshold + 1);
            }
            for(i = 2; i < dc_values.j; i++)
                dc_data.xon_ack_response[i - 2] = (uint8_t)val[i];
            dc_data.xon_ack_response[i - 2] = '\0';
            if(*dc_data.xon_ack_response == '\0')
                dc_data.enquiry = 0;
            dc_data.handshake_mode = dc_data.enquiry && dc_data.xon_ack_response ? (dc_values.cmd == 'I' ? 2 : 1) : 0;
            set_handshake_mode();
            break;

        case 'M':
            dc_data.turnaround_delay = present[0] ? (uint_fast16_t)((float)((uint32_t)((float)val[0] * 1.1875f) % 65536) / 1.2f) : 0;
            dc_data.output_trigger = present[1] ? (uint8_t)val[1] : '\0';
            dc_data.echo_terminator = present[2] ? (uint8_t)val[2] : '\0';
            hpgl_state.term[0] = present[3] ? (uint8_t)val[3] : '\r';
            hpgl_state.term[1] = present[4] ? (uint8_t)val[4] : '\0';
            dc_data.output_initiator = present[5] ? (uint8_t)val[5] : '\0';
            break;

        case 'N':
            dc_data.intercharacter_delay = present[0] ? (uint_fast16_t)((float)((uint32_t)((float)val[0] * 1.1875f) % 65536) / 1.2f) : 0;
            for(i = 1; i < dc_values.j; i++)
                dc_data.xoff_immediate_response[i - 1] = (uint8_t)val[i];
            dc_data.xoff_immediate_response[i - 1] = '\0';
            set_handshake_mode();
            break;
    }
}

static ISR_CODE bool ISR_FUNC(await_colon)(char c)
{
    if(c == ':' || (c == ASCII_ESC && dc_values.j >= DC_VALUES_MAX)) {
        hal.stream.set_enqueue_rt_handler(base_handler);
        dc_values.value[dc_values.j][dc_values.i] = '\0';
        if(*dc_values.value[dc_values.j])
            dc_values.j++;
        process_dci();
    } else if(c == ';') {
        dc_values.value[dc_values.j++][dc_values.i] = '\0';
        dc_values.i = 0;
        if(dc_values.j < DC_VALUES_MAX)
            dc_values.value[dc_values.j][dc_values.i] = '\0';
        else
            dc_values.error = IOError_TooManyParameters;
    } else if(c == CMD_JOG_CANCEL && (state_get() & STATE_JOG))
        system_set_exec_state_flag(EXEC_MOTION_CANCEL);
    else if (dc_values.j < DC_VALUES_MAX && dc_values.i < 8)
        dc_values.value[dc_values.j][dc_values.i++] = c;
    else
        dc_values.error = dc_values.i == 8 ? IOError_Overflow : IOError_TooManyParameters;

    return true;
}

static ISR_CODE bool ISR_FUNC(stream_parse_esc)(char c)
{
    static bool ok = false;

    if(c == '.')
        ok = true;

    else if(ok) {
        
        bool wait_for_colon = false;

        ok = false;
        
        switch(c) {
        
            case '(':
            case 'Y':
                online_led(true);
                break;
        
            case ')':
            case 'Z':
                online_led(false);
                break;

            case '@':
                wait_for_colon = true;
                break;

            case 'B':
                protocol_enqueue_rt_command(report_buffer_free);
                break;
                
            case 'E':
                protocol_enqueue_rt_command(report_extended_error);
                break;
        
            case 'H':
            case 'I':
                wait_for_colon = true;
                break;
        
            case 'J':
                break;
  
            case 'K':
                // flush buffer and reset parser
                break;

            case 'L':   
                protocol_enqueue_rt_command(report_buffer_size);
                break;

            case 'M':
                wait_for_colon = true;
                break;
      
            case 'N':
                wait_for_colon = true;
                break;
     
            case 'O':
                protocol_enqueue_rt_command(report_extended_status);
                break;

            case 'R':
                memset(&dc_data, 0, sizeof(dc_data));
                dc_data.block_size = 80;
                dc_data.xon_level = 512;
                set_handshake_mode();
                break;

            default:
                dc_values.error = c == ASCII_ESC ? IOError_DCI_Overlap : IOError_DCI_InvalidCmd;
                break;

        }
        
        if(wait_for_colon) {
            dc_values.cmd = c;
            dc_values.i = dc_values.j = 0;
            hal.stream.set_enqueue_rt_handler(await_colon);
        } else
            hal.stream.set_enqueue_rt_handler(base_handler);
    } else {
        hal.stream.set_enqueue_rt_handler(base_handler);
        dc_values.error = c == ASCII_ESC ? IOError_DCI_Overlap : IOError_DCI_InvalidData;
    }

    if(c == CMD_JOG_CANCEL && (state_get() & STATE_JOG))
        system_set_exec_state_flag(EXEC_MOTION_CANCEL);

    return true;
}

// End of Device Control Instructions handling

// Normal mode data transfer

int16_t stream_get_data (void)
{
    if(pollc == 0 && stream.get_rx_buffer_count()) {
        pollc = stream.read();
        do_stuff(pollc);
    }

    return SERIAL_NO_DATA;
}

static ISR_CODE bool ISR_FUNC(stream_insert_buffer)(char c)
{
    bool claim = false;

    switch(c) {

        case ASCII_ESC:
            claim = true;
            hal.stream.set_enqueue_rt_handler(stream_parse_esc);
            break;

        case ASCII_ENQ:
            if((claim = dc_data.enquiry == '\0'))
                hal.stream.write_char(ASCII_ACK);
            break;

        case CMD_JOG_CANCEL:
            if((claim = !!(state_get() & STATE_JOG)))
                system_set_exec_state_flag(EXEC_MOTION_CANCEL);
            break;
    }

    return claim;
}

// Xon/Xoff mode data transfer

int16_t stream_get_data_xon (void)
{
    static volatile bool lock = false;

    if(lock)
        return SERIAL_NO_DATA;

    lock = true;

    rx_count = stream.get_rx_buffer_count();

    if (process == NULL && rx_count) {

        if(pollc == 0) {
            pollc = stream.read();
            do_stuff(pollc);
        }

        if(xoff && rx_count <= dc_data.xon_level) {
            xoff = false;
            hal.stream.write(dc_data.xon_ack_response);
            hal.stream.read = stream_get_data;
        }
    }

    lock = false;

    return SERIAL_NO_DATA;
}

static ISR_CODE bool ISR_FUNC(stream_insert_buffer_xoff)(char c)
{
    bool claim = false;

    if(!xoff) {
        if((xoff = stream.get_rx_buffer_free() < dc_data.xoff_threshold)) {
            hal.stream.write(dc_data.xoff_immediate_response);
            hal.stream.read = stream_get_data_xon;
        }
    }

    switch(c) {

        case ASCII_ESC:
            claim = true;
            hal.stream.set_enqueue_rt_handler(stream_parse_esc);
            break;

        case ASCII_ENQ:
            if((claim = dc_data.enquiry == '\0'))
                hal.stream.write_char(ASCII_ACK);
            break;

        case CMD_JOG_CANCEL:
            if((claim = !!(state_get() & STATE_JOG)))
                system_set_exec_state_flag(EXEC_MOTION_CANCEL);
            break;

    }

    return claim;
}

// ENQ/ACK mode data transfer

int16_t stream_get_data_ack (void)
{
    static volatile bool lock = false;

    if(lock)
        return SERIAL_NO_DATA;

    lock = true;

    rx_count = stream.get_rx_buffer_count();

    if (process == NULL && rx_count) {

        if(pollc == 0) {
            pollc = stream.read();
            do_stuff(pollc);
        }

        if(stream.get_rx_buffer_free() >= dc_data.block_size) {
            hal.stream.write(dc_data.xon_ack_response);
            if(dc_data.handshake_mode != 2)
                hal.stream.write(hpgl_state.term);
            hal.stream.read = stream_get_data;
        }
    }

    lock = false;

    return SERIAL_NO_DATA;
}

static ISR_CODE void ISR_FUNC(stream_send_ack)(void)
{
    if(hal.stream.get_rx_buffer_free() >= dc_data.block_size) {
        hal.stream.write(dc_data.xon_ack_response);
        if(dc_data.handshake_mode != 2)
            hal.stream.write(hpgl_state.term);
    } else
        hal.stream.read = stream_get_data_ack;
}

static ISR_CODE bool ISR_FUNC(stream_await_echo_terminator)(char c)
{
    if(c == dc_data.echo_terminator)
        hal.stream.set_enqueue_rt_handler(base_handler);

    return true;
}

static ISR_CODE void ISR_FUNC(stream_send_initiator)(void)
{
    hal.stream.set_enqueue_rt_handler(dc_data.echo_terminator && dc_data.handshake_mode != 2 ? stream_await_echo_terminator : base_handler);

    if(dc_data.handshake_mode == 0 && dc_data.output_initiator)
        hal.stream.write_char(dc_data.output_initiator);

    stream_send_ack();
}

static ISR_CODE bool ISR_FUNC(stream_await_trigger)(char c)
{
    if(c == dc_data.output_trigger)
        hal.delay_ms(dc_data.turnaround_delay + dc_data.intercharacter_delay, stream_send_initiator);

    return true;
}

static ISR_CODE bool ISR_FUNC(stream_insert_buffer_enq)(char c)
{
    bool claim = false;

    if(c == dc_data.enquiry) {

        if(dc_data.handshake_mode != 2 && dc_data.output_trigger) {
            hal.stream.set_enqueue_rt_handler(stream_await_trigger);
            return true;
        } else if(dc_data.turnaround_delay) {
            hal.delay_ms(dc_data.turnaround_delay + dc_data.intercharacter_delay, stream_send_initiator);
            return true;
        } else if(dc_data.handshake_mode != 2 && dc_data.echo_terminator)
            hal.stream.set_enqueue_rt_handler(stream_await_echo_terminator);

        if(dc_data.handshake_mode == 0 && dc_data.output_initiator)
            hal.stream.write_char(dc_data.output_initiator);

        if(*dc_data.xoff_immediate_response) {
            hal.stream.write(dc_data.xoff_immediate_response);
            if(dc_data.handshake_mode != 2)
                hal.stream.write(hpgl_state.term);
        }

        stream_send_ack();

    } else switch(c) {

        case ASCII_ESC:
            claim = true;
            hal.stream.set_enqueue_rt_handler(stream_parse_esc);
            break;

        case ASCII_ENQ:
            hal.stream.set_enqueue_rt_handler(stream_await_trigger);
            break;

        case CMD_JOG_CANCEL:
            if((claim = !!(state_get() & STATE_JOG)))
                system_set_exec_state_flag(EXEC_MOTION_CANCEL);
            break;
    }

    return claim;
}

//

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:HPGL " VERSION ASCII_EOL);
}

status_code_t hpgl_start (sys_state_t state, char *args)
{
    plotter_init();

    if(stream.write == NULL)
        memcpy(&stream, &hal.stream, sizeof(io_stream_t));
    else {
        hal.stream.write = stream.write;
        hal.stream.write_all = stream.write_all;
    }
    hal.stream.read = stream_get_data;
    base_handler = stream_insert_buffer;
    enqueue_realtime_command = hal.stream.set_enqueue_rt_handler(base_handler);

    stream.write("Motori HPGL " VERSION ASCII_EOL);

    if(on_execute_realtime == NULL) {
        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = poll_stuff;
        on_state_change = grbl.on_state_change;
        grbl.on_state_change = state_changed;
    }

    go_home();

    return Status_OK;
}

void hpgl_boot (sys_state_t state)
{
    hpgl_start(state, NULL);
}

const sys_command_t hpgl_command_list[] = {
    {"HPGL", hpgl_start, { .noargs = On }}
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

    stream.write = NULL;
#ifdef HPGL_BOOT
    protocol_enqueue_rt_command(hpgl_boot);
    memcpy(&stream, &hal.stream, sizeof(io_stream_t));
    hal.stream.write = hal.stream.write_all = stream_write_null;
#endif
}
