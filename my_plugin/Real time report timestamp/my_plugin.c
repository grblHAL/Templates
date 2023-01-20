/*

  my_plugin.c.c - Real time report timestamp

  Adds timestamp in |TS element to real time report, millisecond resolution.

  Use:
  M101 to reset to 0 and keep current mode.
  M101P0 to reset to zero and exit synchrounous mode.
  M101P1 to reset to zero and enter synchrounous mode (default).

  When synchrounous mode is active delay reset until buffered motions has been completed.

  Part of grblHAL

  Public domain.

*/

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "grbl/hal.h"

static uint32_t offset = 0;
static bool mcode_sync = true;
static on_realtime_report_ptr on_realtime_report;
static on_report_options_ptr on_report_options;
static user_mcode_ptrs_t user_mcode;

static user_mcode_t check (user_mcode_t mcode)
{
    return mcode == UserMCode_Generic1
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case UserMCode_Generic1:
            if(gc_block->words.p) {
                if(isnanf(gc_block->values.p) || !isintf(gc_block->values.p))
                    state = Status_BadNumberFormat;
                else
                    mcode_sync = gc_block->values.p != 0.0f;
                gc_block->words.p = Off;
            }
            gc_block->user_mcode_sync = mcode_sync;
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void execute (sys_state_t state, parser_block_t *gc_block) {

    bool handled = true;

    switch(gc_block->user_mcode) {

        case UserMCode_Generic1:
            offset = hal.get_elapsed_ticks();
            break;

        default:
            handled = false;
            break;
    }


    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    static char buf[30] = {0};

    uint32_t ts = hal.get_elapsed_ticks() - offset;
    uint32_t ms, s;

    ms = ts % 1000;
    ts -= ms;
    s = (ts % 60000);
    ts -= s;

    sprintf(buf, "|TS:%lu:%02lu,%lu", ts / 60000, s / 1000, ms);

    stream_write(buf);

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN: RT timestamp v0.02]" ASCII_EOL);
}

void my_plugin_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = onRealtimeReport;

    memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

    hal.user_mcode.check = check;
    hal.user_mcode.validate = validate;
    hal.user_mcode.execute = execute;
}
