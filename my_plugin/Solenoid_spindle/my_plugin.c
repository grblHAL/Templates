/*
  my_plugin.c - sets PWM output to full power on spindle on, reduces it after a short delay.

  Part of grblHAL

  Public domain
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
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

static void solenoid_reduce_current (sys_state_t state)
{
    if(power_down && hal.get_elapsed_ticks() - power_down >= SOLENOID_HOLD_DELAY)
        pwm_spindle.set_state(&pwm_spindle, pwm_spindle.get_state(&pwm_spindle), pwm_spindle.rpm_max * SOLENOID_HOLD_FACTOR);

    on_execute_realtime(state);
}

static void solenoid_set_state (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    power_down = state.on && rpm > 0.0f ? hal.get_elapsed_ticks() : 0;

    pwm_spindle.set_state(&pwm_spindle, state, power_down ? pwm_spindle.rpm_max : rpm);
}

static bool solenoid_spindle_select (spindle_ptrs_t *spindle)
{
    if(spindle->type == SpindleType_PWM) {
        memcpy(&pwm_spindle, spindle, sizeof(spindle_ptrs_t));
        spindle->set_state = solenoid_set_state;
        spindle->cap.laser = Off;
    }

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Solenoid spindle", "1.04");
}


void my_plugin_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = solenoid_reduce_current;

    on_spindle_select = grbl.on_spindle_select;
    grbl.on_spindle_select = solenoid_spindle_select;
}
