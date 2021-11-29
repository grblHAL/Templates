/*

  my_plugin.c.c - MCU load estimator

  Counts number of iterations of protocol idle loop per 10ms and add count to real time report.

  Higher number is better. E.g. the iMXRT1062 reports > 20000 when idle -> less than 500ns per iteration.

  Part of grblHAL

  Public domain.

*/

#include "grbl/hal.h"

static uint32_t count = 0;
static bool add_report = false;
static on_execute_realtime_ptr on_execute_realtime;
static on_realtime_report_ptr on_realtime_report;
static on_report_options_ptr on_report_options;

void onExecuteRealtime (uint_fast16_t state)
{
    static uint32_t last_ms, last_count;

    uint32_t ms = hal.get_elapsed_ticks();

    if(ms - last_ms >= 10) {
        last_ms = ms;
        count = last_count;
        last_count = 0;
        add_report = true;
    } else
        last_count++;

    on_execute_realtime(state);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    static char buf[20] = {0};

    if (add_report) {

        add_report = false;

        strcpy(buf, "|LOAD:");
        strcat(buf, uitoa(count));

        stream_write(buf);
    }

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:MCU Load v0.01]" ASCII_EOL);
}

void my_plugin_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = onExecuteRealtime;

    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = onRealtimeReport;
}
