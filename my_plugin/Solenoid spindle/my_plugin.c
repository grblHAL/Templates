/*

  solenoid.c - sets PWM output to full power on spindle on, reduces it after a short delay.

  Part of grblHAL

  Public domain

*/

#include "driver.h"

#include <string.h>

#define SOLENOID_HOLD_DELAY 50 // ms
#define SOLENOID_HOLD_FACTOR 0.25f

static uint32_t power_down = 0;

static spindle_ptrs_t pwm_spindle;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;
static on_spindle_select_ptr on_spindle_select;

static void solenoid_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Solenoid spindle v1.00]" ASCII_EOL);
}

static void solenoid_reduce_current (sys_state_t state)
{
    if(power_down && hal.get_elapsed_ticks() - power_down >= SOLENOID_HOLD_DELAY)
        pwm_spindle.set_state(pwm_spindle.get_state(), pwm_spindle.rpm_max * SOLENOID_HOLD_FACTOR);

    on_execute_realtime(state);
}

static void solenoid_set_state (spindle_state_t state, float rpm)
{
    power_down = state.on && rpm > 0.0f ? hal.get_elapsed_ticks() : 0;

    pwm_spindle.set_state(state, power_down ? pwm_spindle.rpm_max : rpm);
}

static bool solenoid_spindle_select (spindle_id_t spindle_id)
{
#if GRBL_BUILD >= 20221014
    if(hal.spindle.type == SpindleType_PWM) {
#else
    if(hal.spindle.cap.laser) {
#endif
        memcpy(&pwm_spindle, &hal.spindle, sizeof(spindle_ptrs_t));
        hal.spindle.set_state = solenoid_set_state;
        hal.spindle.cap.laser = Off;
        sys.mode = Mode_Standard;
    }

    if(on_spindle_select)
        on_spindle_select(spindle_id);

    return true;
}

void my_plugin_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = solenoid_options;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = solenoid_reduce_current;

    on_spindle_select = grbl.on_spindle_select;
    grbl.on_spindle_select = solenoid_spindle_select;
}
