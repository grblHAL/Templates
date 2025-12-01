/*
  my_plugin.c - plugin for M17 & M18 (M84) Marlin style stepper enable/disable commands.

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  Usage:
    M17[X][Y][Z] - enable steppers
    M18[X][Y][Z][S<delay>] - disable steppers
    M84[X][Y][Z][S<delay>] - disable steppers

  If no axis words are specified all axes are enabled/disabled
  If no delay is specified disable is immediate, else delay is number of seconds.

  https://marlinfw.org/docs/gcode/M017.html
  https://marlinfw.org/docs/gcode/M018.html
*/

#include <math.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"

static bool await_disable = false;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static stepper_enable_ptr stepper_enable;
static axes_signals_t stepper_enabled = {0};

static void disable_steppers (void *data);

static void stepperEnable (axes_signals_t enable, bool hold)
{
    if(await_disable) {
        await_disable = false;
        task_delete(disable_steppers, NULL);
    }

    if((stepper_enabled.mask = enable.mask))
        sys.steppers_deenergize = false;

    stepper_enable(enable, hold);
}

static void disable_steppers (void *data)
{
    await_disable = false;
    hal.stepper.enable(*((axes_signals_t *)data), false);
}

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)17 || mcode == (user_mcode_t)18 || mcode == (user_mcode_t)84
                     ? UserMCode_NoValueWords
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    switch((uint16_t)gc_block->user_mcode) {

        case 17:
            gc_block->user_mcode_sync = On;
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
            gc_block->user_mcode_sync = On;
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

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
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

    if(state != STATE_CHECK_MODE)
      switch((uint16_t)gc_block->user_mcode) {

        case 17: // stepper enable
            {
                bool changed;
                axes_signals_t enable = { .bits = stepper_enabled.bits };

                if(gc_block->words.mask & axis_words.mask) {
                    if(gc_block->words.x)
                        enable.x = On;
                    if(gc_block->words.y)
                        enable.y = On;
                    if(gc_block->words.z)
                        enable.z = On;
#ifdef A_AXIS
                    if(gc_block->words.a)
                        enable.a = On;
#endif
#ifdef B_AXIS
                    if(gc_block->words.b)
                        enable.b = On;
#endif
#ifdef C_AXIS
                    if(gc_block->words.c)
                        enable.c = On;
#endif
                } else
                    enable.bits = AXES_BITMASK;

                changed = !sys.steppers_enabled || enable.bits != stepper_enabled.bits;
                hal.stepper.enable(enable, false);
                if(changed && enable.bits == AXES_BITMASK && settings.stepper_enable_delay)
                    hal.delay_ms(settings.stepper_enable_delay, NULL);
            }
            break;

        case 18: // stepper disable
        case 84:
            {
                static axes_signals_t enable;

                enable.bits = stepper_enabled.bits;

                if(gc_block->words.mask & axis_words.mask) {
                    if(gc_block->words.x)
                        enable.x = Off;
                    if(gc_block->words.y)
                        enable.y = Off;
                    if(gc_block->words.z)
                        enable.z = Off;
#ifdef A_AXIS
                    if(gc_block->words.a)
                        enable.a = Off;
#endif
#ifdef B_AXIS
                    if(gc_block->words.b)
                        enable.b = Off;
#endif
#ifdef C_AXIS
                    if(gc_block->words.c)
                        enable.c = Off;
#endif
                } else
                    enable.mask = 0;

                if(gc_block->words.s && gc_block->values.s > 0.0f) {
                    if(!await_disable)
                        await_disable = task_add_delayed(disable_steppers, &enable, (uint32_t)(gc_block->values.s * 1000.0f));
                } else
                    hal.stepper.enable(enable, false);
            }
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void reportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Stepper enable", "0.04");
}

void my_plugin_init (void)
{
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;

    stepper_enable = hal.stepper.enable;
    hal.stepper.enable = stepperEnable;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = reportOptions;
}
