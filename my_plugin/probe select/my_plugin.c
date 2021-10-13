/*

  my_plugin.c - plugin template for using an auxillary output to control a probe selection relay.
  
  M401/M402 can be used to control the output.
  Automatic switching on a tool change will also take place if the $341 tool change mode is 2 or 3.

  Use the $pins command to find out which output pin is used, it will be labeled "Probe relay".

  NOTE: If no auxillary output is available it will not install itself.

  Part of grblHAL

  Public domain.

*/

#ifdef ARDUINO
#include "../src/grbl/hal.h"
#include "../src/grbl/protocol.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#endif

static uint8_t relay_port;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)401 || mcode == (user_mcode_t)402
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t mcode_validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    switch((uint16_t)gc_block->user_mcode) {

        case 401:
        // no break
        case 402:
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

    if (state != STATE_CHECK_MODE)
      switch((uint16_t)gc_block->user_mcode) {

        case 401:
            hal.port.digital_out(relay_port, 1);
            break;

        case 402:
            hal.port.digital_out(relay_port, 0);
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void probe_fixture (bool on)
{
    hal.port.digital_out(relay_port, on);
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Probe select v0.01]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Probe select plugin failed to initialize!", Message_Warning);
}

void my_plugin_init (void)
{
    if(hal.port.num_digital_out > 0) {

        relay_port = --hal.port.num_digital_out;        // "Claim" the port, M62-M65 cannot be used
//        relay_port = hal.port.num_digital_out - 1;    // Do not "claim" the port, M62-M65 can be used

        if(hal.port.set_pin_description)
            hal.port.set_pin_description(Port_Digital, Port_Output, relay_port, "Probe relay");

        hal.user_mcode.check = mcode_check;
        hal.user_mcode.validate = mcode_validate;
        hal.user_mcode.execute = mcode_execute;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        grbl.on_probe_fixture = probe_fixture;          // Comment out this line if automatic switching on a tool change is not wanted.

    } else
        protocol_enqueue_rt_command(warning_msg);
}
