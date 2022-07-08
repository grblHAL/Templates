/*

  my_plugin.c - plugin for M17 & M18 (M84) Marlin style stepper enable/disable commands.

  Part of grblHAL

  Public domain.

  Usage:
    M17[X][Y][Z]
    M18[X][Y][Z][S<delay>]
    M18[X][Y][Z][S<delay>]

  If no axis words are specified all axes are enabled/disabled
  If no delay is specified disable is immediate, else delay is number of seconds.

  https://marlinfw.org/docs/gcode/M017.html
  https://marlinfw.org/docs/gcode/M018.html

*/

#include <math.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"

static uint32_t delay_until = 0;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static stepper_enable_ptr on_stepper_enable;
static axes_signals_t stepper_enabled = {0};
static on_execute_realtime_ptr on_execute_realtime;

static void stepper_enable (axes_signals_t enable)
{
    if(on_execute_realtime) {
        grbl.on_execute_realtime = on_execute_realtime;
        on_execute_realtime = NULL; // terminate any delayed disable
    }

    stepper_enabled.mask = enable.mask;

    on_stepper_enable(enable);
}

static void disable_steppers (sys_state_t state)
{
    if(delay_until - hal.get_elapsed_ticks() <= 0)
        stepper_enable(stepper_enabled);
}

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)17 || mcode == (user_mcode_t)18 || mcode == (user_mcode_t)84
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t mcode_validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    switch((uint16_t)gc_block->user_mcode) {

        case 17:
            gc_block->words.x = gc_block->words.y = gc_block->words.z = Off;
#ifdef A_AXIS
            gc_block->words.a = Off;
#endif
#ifdef B_AXIS
            gc_block->words.b = Off;
#endif
#ifdef C_AXIS
            gc_block->words.c = Off;
#endif
            break;

        case 18:
        case 84:
            if(gc_block->words.s && isnanf(gc_block->values.s))
                state = Status_BadNumberFormat;
            gc_block->words.s = gc_block->words.x = gc_block->words.y = gc_block->words.z = Off;
#ifdef A_AXIS
            gc_block->words.a = Off;
#endif
#ifdef B_AXIS
            gc_block->words.b = Off;
#endif
#ifdef C_AXIS
            gc_block->words.c = Off;
#endif
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    static const parameter_words_t axis_words = {
        .x = On,
        .y = On,
        .z = On
#ifdef A_AXIS
      , .a = On
#endif
#ifdef B_AXIS
      , .b = On
#endif
#ifdef C_AXIS
      , .c = On
#endif
    };

    if (state != STATE_CHECK_MODE)
      switch((uint16_t)gc_block->user_mcode) {

        case 17: // stepper enable
            {
                if(gc_block->words.mask & axis_words.mask) {
                    if(gc_block->words.x)
                        stepper_enabled.x = On;
                    if(gc_block->words.y)
                        stepper_enabled.y = On;
                    if(gc_block->words.z)
                        stepper_enabled.z = On;
#ifdef A_AXIS
                    if(gc_block->words.a)
                        stepper_enabled.a = On;
#endif
#ifdef B_AXIS
                    if(gc_block->words.b)
                        stepper_enabled.b = On;
#endif
#ifdef C_AXIS
                    if(gc_block->words.c)
                        stepper_enabled.c = On;
#endif
                } else
                    stepper_enabled.mask = AXES_BITMASK;

                stepper_enable(stepper_enabled);
            }
            break;

        case 18: // stepper disable
        case 84:
            {
                if(gc_block->words.mask & axis_words.mask) {
                    if(gc_block->words.x)
                        stepper_enabled.x = Off;
                    if(gc_block->words.y)
                        stepper_enabled.y = Off;
                    if(gc_block->words.z)
                        stepper_enabled.z = Off;
#ifdef A_AXIS
                    if(gc_block->words.a)
                        stepper_enabled.a = Off;
#endif
#ifdef B_AXIS
                    if(gc_block->words.b)
                        stepper_enabled.b = Off;
#endif
#ifdef C_AXIS
                    if(gc_block->words.c)
                        stepper_enabled.c = Off;
#endif
                } else
                    stepper_enabled.mask = 0;

                if(gc_block->words.s && gc_block->values.s > 0.0f) {
                    if(on_execute_realtime == NULL) {
                        on_execute_realtime = grbl.on_execute_realtime;
                        grbl.on_execute_realtime = disable_steppers;
                    }
                    delay_until = hal.get_elapsed_ticks() + (uint32_t)gc_block->values.s * 1000;
                }
                else
                    stepper_enable(stepper_enabled);
            }
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Stepper enable v0.01]" ASCII_EOL);
}

void my_plugin_init (void)
{
    memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

    hal.user_mcode.check = mcode_check;
    hal.user_mcode.validate = mcode_validate;
    hal.user_mcode.execute = mcode_execute;

    on_stepper_enable = hal.stepper.enable;
    hal.stepper.enable = stepper_enable;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}
