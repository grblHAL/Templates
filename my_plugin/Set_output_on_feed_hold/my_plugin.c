/*

  my_plugin.c - plugin template for setting auxillary output on feed hold

  Part of grblHAL

  Public domain.

*/

#include <string.h>

#include "driver.h"
#include "grbl/protocol.h"

static uint8_t port;

static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;

static void onStateChanged (sys_state_t state)
{
    static sys_state_t last_state = STATE_IDLE;

    if(state != last_state) {
        last_state = state;
        hal.port.digital_out(port, state == STATE_HOLD);
    }

    if(on_state_change)         // Call previous function in the chain.
        on_state_change(state);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);  // Call previous function in the chain.

    if(!newopt)                 // Add info about us to the $I report.
        hal.stream.write("[PLUGIN:MY PLUGIN Template 2]" ASCII_EOL);
}

void my_plugin_init (void)
{
    if(hal.port.num_digital_out == 0)                   // This plugin requires one digital output port,
        protocol_enqueue_foreground_task(report_warning, "An output port is required for my_plugin!",);    // complain if not available.

    else {
        port = hal.port.num_digital_out - 1;            // Claim the
        hal.port.num_digital_out--;                     // last free port.

        if(hal.port.set_pin_description)                // Add label to the pin for the $pins command
            hal.port.set_pin_description(Port_Digital, Port_Output, port, "Feed hold out");

        on_state_change = grbl.on_state_change;         // Subscribe to the state changed event by saving away the original
        grbl.on_state_change = onStateChanged;          // function pointer and adding ours to the chain.

        on_report_options = grbl.on_report_options;     // Add our plugin to to the options report chain
        grbl.on_report_options = onReportOptions;       // to tell the user we are active.
    }
}
