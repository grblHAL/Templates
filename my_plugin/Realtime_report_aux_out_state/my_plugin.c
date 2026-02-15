/*
  my_plugin.c - Add changed aux port output states to real time report

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  NOTES: Only ports available via M62-M65 and M67-M68 are reported.
         The reported state is the logical state, not the actual output.
*/

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/task.h"

static uint32_t ao_enabled = 0, ao_changed = 0;
static uint32_t do_enabled = 0, do_changed = 0, do_state = 0;
static float ao_state[N_AUX_AOUT_MAX] = {0};

static on_port_out_ptr on_port_out;
static on_realtime_report_ptr on_realtime_report;
static on_report_options_ptr on_report_options;

void onPortOut (uint8_t port, io_port_type_t type, float value)
{
    if(type == Port_Digital) {
        uint8_t bit = 1 << port;
        if(!!(do_state & bit) != (value == 1.0f)) {
            do_changed |= bit;
            if(value == 0.0f)
                do_state &= ~(1 << port);
            else
                do_state |= (1 << port);
        }
    } else if(type == Port_Analog) {
        if(value != ao_state[port]) {
            ao_changed |= (1 << port);
            ao_state[port] = value;
        }
    }

    if(on_port_out)
        on_port_out(port, type, value);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(on_realtime_report)
        on_realtime_report(stream_write, report);

    if(report.all) {
        ao_changed = ao_enabled;
        do_changed = do_enabled;
    }

    if(!(ao_changed || do_changed))
        return;

    uint8_t port;
    bool add_port = false;
    char buf[20];

    stream_write("|AUX:");

    if(ao_changed) {

        port = 0;

        do {
            if(ao_changed & 1) {
                strcpy(buf, add_port ? ";E" : "E");
                strcat(buf, uitoa(port));
                strcat(buf, ",");
                strcat(buf, ftoa(ao_state[port], 1));
                add_port = true;
                stream_write(buf);
            }
            port++;
        } while(ao_changed >>= 1);
    }

    if(do_changed) {

        port = 0;
        add_port = false;

        do {
            if(do_changed & 1) {
                strcpy(buf, add_port ? ";P" : "P");
                strcat(buf, uitoa(port));
                strcat(buf, ",");
                strcat(buf, uitoa(!!(do_state & (1 << port))));
                add_port = true;
                stream_write(buf);
            }
            port++;
        } while(do_changed >>= 1);
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Aux port state", "0.01");
}

static void setup (void *data)
{
    uint8_t n_ports;

    n_ports = ioports_unclaimed(Port_Digital, Port_Output);
    while(n_ports--)
        do_enabled |= (do_enabled << 1) | 1;

    n_ports = ioports_unclaimed(Port_Analog, Port_Output);
    while(n_ports--)
        ao_enabled |= (ao_enabled << 1) | 1;
}

void my_plugin_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = onRealtimeReport;

    on_port_out = grbl.on_port_out;
    grbl.on_port_out = onPortOut;

    task_run_on_startup(setup, NULL);
}
