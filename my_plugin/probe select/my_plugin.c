/*

  my_plugin.c - plugin template for using an auxillary output to control a probe selection relay.
  
  Use the $pins command to find out which output pin is used, it will be labeled "Probe relay".

  NOTE: If no auxillary output is available it will not install itself.

  Part of grblHAL

  Public domain.

  M401   - switch on relay immediately.
  M401Q0 - set mode to switch on relay when probing @ G59.3 (default)
  M401Q1 - set mode to switch on relay when probing @ G59.3 while changing tool (executing M6 when $341 tool change mode is 1, 2 or 3)
  M401Q2 - set mode to switch on relay when probing while changing tool (executing M6)
  M401Q3 - set mode to always switch on relay when probing
  M401Q4 - set mode to never switch on relay when probing
  M402   - switch off relay immediately.

  NOTES: The symbol TOOLSETTER_RADIUS (defined in grbl/config.h, default 5.0mm) is the tolerance for checking "@ G59.3".
         When $341 tool change mode 1 or 2 is active it is possible to jog to/from the G59.3 position.

  Tip: Set default mode at startup by adding M401Qx to a startup script ($N0 or $N1)

*/

#include "grbl/hal.h"
#include "grbl/protocol.h"

#include <math.h>
#include <string.h>

typedef enum {
    ProbeMode_AtG59_3 = 0,
    ProbeMode_ToolChangeAtG59_3,
    ProbeMode_ToolChange,
    ProbeMode_Always,
    ProbeMode_Never,
    ProbeMode_MaxValue = ProbeMode_Never,
} probe_mode_t;

static uint8_t relay_port;
static probe_mode_t probe_mode = ProbeMode_AtG59_3;
static driver_reset_ptr driver_reset;
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
            if(gc_block->words.q) {
                if(isnanf(gc_block->values.q))
                    state = Status_BadNumberFormat;
                else {
                    if(!isintf(gc_block->values.q) || gc_block->values.q < 0.0f || (probe_mode_t)gc_block->values.q > ProbeMode_MaxValue)
                        state = Status_GcodeValueOutOfRange;
                    gc_block->words.q = Off;
                }
            }
            break;

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
            if(gc_block->words.q)
                probe_mode = (probe_mode_t)gc_block->values.q;
            else {
                hal.port.digital_out(relay_port, 1);
                hal.delay_ms(50, NULL); // Delay a bit to let any contact bounce settle.
            }
            break;

        case 402:
            hal.port.digital_out(relay_port, 0);
            hal.delay_ms(50, NULL); // Delay a bit to let any contact bounce settle.
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

// When called from "normal" probing tool is always NULL, when called from within
// a tool change sequence (M6) then tool is a pointer to the selected tool.
bool probe_fixture (tool_data_t *tool, bool at_g59_3, bool on)
{
    if(on) switch(probe_mode) {

        case ProbeMode_AtG59_3:
            on = at_g59_3;
            break;

        case ProbeMode_ToolChangeAtG59_3:
            on = tool != NULL && at_g59_3;
            break;

        case ProbeMode_ToolChange:
            on = tool != NULL;
            break;

        case ProbeMode_Never:
            on = false;
            break;

        default:
            break;
    }

    hal.port.digital_out(relay_port, on);
    hal.delay_ms(50, NULL); // Delay a bit to let any contact bounce settle.

    return on;
}

static void probe_reset (void)
{
    driver_reset();

    hal.port.digital_out(relay_port, false);
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Probe select v0.03]" ASCII_EOL);
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

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = mcode_check;
        hal.user_mcode.validate = mcode_validate;
        hal.user_mcode.execute = mcode_execute;

        driver_reset = hal.driver_reset;
        hal.driver_reset = probe_reset;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        grbl.on_probe_fixture = probe_fixture;

    } else
        protocol_enqueue_rt_command(warning_msg);
}
