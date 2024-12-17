/*

  my_plugin.c - plugin template for using an auxillary output to control a probe selection relay.

  Use the $pins command to find out which output port/pin is used, it will be labeled "Probe relay".
  If driver supports aux port remapping a setting for selecting which port to use will be
  available, see below.

  NOTE: If no auxillary output is available it will not install itself.

  Part of grblHAL

  Public domain.

  M401   - switch on relay immediately.
  M401Q0 - set mode to switch on relay when probing @ G59.3 (default).
  M401Q1 - set mode to switch on relay when probing @ G59.3 while changing tool (executing M6 when $341 tool change mode is 1, 2 or 3).
  M401Q2 - set mode to switch on relay when probing while changing tool (executing M6).
  M401Q3 - set mode to always switch on relay when probing.
  M401Q4 - set mode to never switch on relay when probing.
  M401Q5 - set mode to leave relay in current state when probing.
  M402   - switch off relay immediately.

  NOTES: The symbol TOOLSETTER_RADIUS (defined in grbl/config.h, default 5.0mm) is the tolerance for checking "@ G59.3".
         When $341 tool change mode 1 or 2 is active it is possible to jog to/from the G59.3 position.
         Automatic relay switching when probing at the G59.3 position requires the machine to be homed (X and Y).

  Tip: Set default mode at startup by adding M401Qx to a startup script ($N0 or $N1)

*/

#include "grbl/hal.h"
#include "grbl/protocol.h"

#include <math.h>
#include <string.h>

#define RELAY_DEBOUNCE 50 // ms - increase if relay is slow and/or bouncy

typedef enum {
    ProbeMode_AtG59_3 = 0,
    ProbeMode_ToolChangeAtG59_3,
    ProbeMode_ToolChange,
    ProbeMode_Always,
    ProbeMode_Never,
    ProbeMode_Manual,
    ProbeMode_MaxValue = ProbeMode_Manual,
} probe_mode_t;

static uint8_t relay_port;
static bool relay_on = false;
static probe_mode_t probe_mode = ProbeMode_AtG59_3; // Default mode
static driver_reset_ptr driver_reset;
static on_probe_toolsetter_ptr on_probe_toolsetter;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;


// Later versions of grblHAL and the driver may allow configuring which aux port to use for relay control.
// If possible the plugin adds a $setting and delay claiming the port until settings has been loaded.
// The default setting number is Setting_UserDefined_0 ($450), this can be changed by
// modifying the RELAY_PLUGIN_SETTING symbol below.

#include "grbl/nvs_buffer.h"

#define RELAY_PLUGIN_ADVANCED
#define RELAY_PLUGIN_SETTING Setting_UserDefined_0

static uint8_t n_ports;
static char max_port[4];

typedef struct {
    uint8_t port;
} relay_settings_t;

static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static relay_settings_t relay_settings;

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == (user_mcode_t)401 || mcode == (user_mcode_t)402
                     ? UserMCode_Normal
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate (parser_block_t *gc_block)
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

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
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
                relay_on = true;
                hal.port.digital_out(relay_port, 1);
                hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
            }
            break;

        case 402:
            relay_on = false;
            hal.port.digital_out(relay_port, 0);
            hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
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
bool probeToolSetter (tool_data_t *tool, coord_data_t *position, bool at_g59_3, bool on)
{
    if(on_probe_toolsetter)
        on_probe_toolsetter(tool, position, at_g59_3, on);

    if(on) switch(probe_mode) {

        case ProbeMode_AtG59_3:
            relay_on = at_g59_3;
            break;

        case ProbeMode_ToolChangeAtG59_3:
            relay_on = tool != NULL && at_g59_3;
            break;

        case ProbeMode_ToolChange:
            relay_on = tool != NULL;
            break;

        case ProbeMode_Never:
            relay_on = false;
            break;

        case ProbeMode_Always:
            relay_on = true;
            break;

        default:
            break;

    } else if(probe_mode != ProbeMode_Manual)
        relay_on = false;

    hal.port.digital_out(relay_port, relay_on);
    hal.delay_ms(RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.

    return relay_on;
}

static void probe_reset (void)
{
    driver_reset();

    relay_on = false;
    hal.port.digital_out(relay_port, false);
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Probe select", "0.07");
}

#ifdef RELAY_PLUGIN_ADVANCED

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.

static const setting_group_detail_t user_groups [] = {
    { Group_Root, Group_UserSettings, "Probe relay"}
};

static const setting_detail_t user_settings[] = {
    { RELAY_PLUGIN_SETTING, Group_UserSettings, "Relay aux port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &relay_settings.port, NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t relay_settings_descr[] = {
    { RELAY_PLUGIN_SETTING, "Aux port number to use for probe relay control.\\n\\n"
                            "NOTE: A hard reset of the controller is required after changing this setting."
    },
};

#endif

// Write settings to non volatile storage (NVS).
static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&relay_settings, sizeof(relay_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
// Default is highest numbered free port.
static void plugin_settings_restore (void)
{
    relay_settings.port = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&relay_settings, sizeof(relay_settings_t), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&relay_settings, nvs_address, sizeof(relay_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();

    // Sanity check
    if(relay_settings.port >= n_ports)
        relay_settings.port = n_ports - 1;

    relay_port = relay_settings.port;

    if(ioport_claim(Port_Digital, Port_Output, &relay_port, "Probe relay")) {

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        driver_reset = hal.driver_reset;
        hal.driver_reset = probe_reset;

        on_probe_toolsetter = grbl.on_probe_toolsetter;
        grbl.on_probe_toolsetter = probeToolSetter;

    } else
        protocol_enqueue_foreground_task(report_warning, "Relay plugin: configured port number is not available");
}

// Settings descriptor used by the core when interacting with this plugin.
static setting_details_t setting_details = {
    .groups = user_groups,
    .n_groups = sizeof(user_groups) / sizeof(setting_group_detail_t),
    .settings = user_settings,
    .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = relay_settings_descr,
    .n_descriptions = sizeof(relay_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = plugin_settings_save,
    .load = plugin_settings_load,
    .restore = plugin_settings_restore
};

void my_plugin_init (void)
{
    bool ok = false;

    if(!ioport_can_claim_explicit()) {

        // Driver does not support explicit pin claiming, claim the highest numbered port instead.

        if((ok = hal.port.num_digital_out > 0)) {

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

            on_probe_toolsetter = grbl.on_probe_toolsetter;
            grbl.on_probe_toolsetter = probeToolSetter;
        }

    } else if((ok = (n_ports = ioports_available(Port_Digital, Port_Output)) > 0 && (nvs_address = nvs_alloc(sizeof(relay_settings_t))))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        settings_register(&setting_details);

        // Used for setting value validation
        strcpy(max_port, uitoa(n_ports - 1));
    }

    if(!ok)
        protocol_enqueue_foreground_task(report_warning, "Probe select plugin failed to initialize!");
}

#else

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
        protocol_enqueue_foreground_task(report_warning, "Probe select plugin failed to initialize!");
}

#endif
