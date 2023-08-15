/*

  my_plugin.c - plugin template for using an auxillary input for a second probe input.
                optionally a second input can be assigned for an overtravel input (by compile time option).

  Use the $pins command to find out which input port/pin is used, it will be labeled "Probe 2".

  NOTE: If no auxillary input is available it will not install itself.

  Part of grblHAL

  Public domain.

  M401   - switch to probe2 immediately.
  M401Q0 - set mode to switch to probe2 when probing @ G59.3 (default).
  M401Q1 - set mode to switch to probe2 when probing @ G59.3 while changing tool (executing M6 when $341 tool change mode is 1, 2 or 3).
  M401Q2 - set mode to switch to probe2 when probing while changing tool (executing M6).
  M401Q3 - set mode to always use probe2  when probing.
  M401Q4 - set mode to never use probe2 when probing.
  M401Q5 - set mode to leave probe2 in current state when probing.
  M402   - switch off probe2 use immediately.

  NOTES: The symbol TOOLSETTER_RADIUS (defined in grbl/config.h, default 5.0mm) is the tolerance for checking "@ G59.3".
         When $341 tool change mode 1 or 2 is active it is possible to jog to/from the G59.3 position.
         Automatic relay switching when probing at the G59.3 position requires the machine to be homed (X and Y).

  Tip: Set default mode at startup by adding M401Qx to a startup script ($N0 or $N1)

*/

#include "grbl/hal.h"
#include "grbl/protocol.h"

#include <math.h>
#include <string.h>

#define PROBE2_OVERTRAVEL Off

typedef enum {
    ProbeMode_AtG59_3 = 0,
    ProbeMode_ToolChangeAtG59_3,
    ProbeMode_ToolChange,
    ProbeMode_Always,
    ProbeMode_Never,
    ProbeMode_Manual,
    ProbeMode_MaxValue = ProbeMode_Manual,
} probe_mode_t;

static uint8_t probe_port, probe_settings_bit;
static bool use_probe2 = false;
static probe_state_t probe = {
    .connected = On
};
static probe_mode_t probe_mode = ProbeMode_AtG59_3; // Default mode
static driver_reset_ptr driver_reset;
static probe_configure_ptr probe_configure;
static probe_get_state_ptr probe_get_state;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;
#if PROBE2_OVERTRAVEL
static uint8_t overtravel_port;
static control_signals_get_state_ptr control_signals_get_state;
#endif

// Later versions of grblHAL and the driver may allow configuring which aux port to use for relay control.
// If possible the plugin adds a $setting and delay claiming the port until settings has been loaded.
// The default setting number is Setting_UserDefined_0 ($450), this can be changed by
// modifying the PROBE_PLUGIN_SETTING symbol below.

#include "grbl/nvs_buffer.h"

#define PROBE_PLUGIN_SETTING Setting_UserDefined_0
#define PROBE_PLUGIN_SETTING1 Setting_UserDefined_1

static uint8_t n_ports;
static char max_port[4];

typedef struct {
    uint8_t probe_port;
    uint8_t overtravel_port;
} probe2_settings_t;

static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static probe2_settings_t probe2_settings;

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
            else
                use_probe2 = true;
            break;

        case 402:
            use_probe2 = false;
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
            use_probe2 = at_g59_3;
            break;

        case ProbeMode_ToolChangeAtG59_3:
            use_probe2 = tool != NULL && at_g59_3;
            break;

        case ProbeMode_ToolChange:
            use_probe2 = tool != NULL;
            break;

        case ProbeMode_Never:
            use_probe2 = false;
            break;

        case ProbeMode_Always:
            use_probe2 = true;
            break;

        default:
            break;

    } else if(probe_mode != ProbeMode_Manual)
        use_probe2 = false;

    return use_probe2;
}

#if PROBE2_OVERTRAVEL

static control_signals_t signalsGetState (void)
{
    control_signals_t signals = control_signals_get_state();

    signals.probe_overtravel = hal.port.wait_on_input(Port_Digital, overtravel_port, WaitMode_Immediate, 0.0f);

    return signals;
}

static void on_overtravel (uint8_t port, bool state)
{
    hal.control.interrupt_callback(signalsGetState());
}

#endif

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    uint32_t invert = !!(setting_get_int_value(setting_get_details(Settings_IoPort_InvertIn, NULL), 0) & probe_settings_bit);

    probe.inverted = is_probe_away ? !invert : invert;
    probe.triggered = Off;
    probe.is_probing = probing;

    probe_configure(is_probe_away, probing);
}

// Returns the probe connected and triggered pin states.
static probe_state_t probeGetState (void)
{
    probe_state_t state = probe_get_state();

    if(use_probe2) {
        state.triggered = hal.port.wait_on_input(Port_Digital, probe_port, WaitMode_Immediate, 0.0f) ^ probe.inverted;
        state.connected = probe.connected;
    }

    return state;
}

static void probeReset (void)
{
    driver_reset();

    use_probe2 = false;
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Probe select 2 v0.01]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Probe select plugin failed to initialize!", Message_Warning);
}

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.
static const setting_detail_t user_settings[] = {
    { PROBE_PLUGIN_SETTING, Group_Probing, "Probe 2 aux port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &probe2_settings.probe_port, NULL, NULL, { .reboot_required = On } },
#if PROBE2_OVERTRAVEL
    { PROBE_PLUGIN_SETTING1, Group_Probing, "Probe 2 overtravel aux port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &probe2_settings.overtravel_port, NULL, NULL, { .reboot_required = On } }
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t probe2_settings_descr[] = {
    { PROBE_PLUGIN_SETTING, "Aux port number to use for second probe input." },
#if PROBE2_OVERTRAVEL
    { PROBE_PLUGIN_SETTING1, "Aux port number to use for second probe overtravel input.\\nIf asserted Z hard limit alarm will raised." },
#endif
};

#endif

// Write settings to non volatile storage (NVS).
static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&probe2_settings, sizeof(probe2_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
// Default is highest numbered free port.
static void plugin_settings_restore (void)
{
    probe2_settings.probe_port = hal.port.num_digital_out ? hal.port.num_digital_out - 1 : 0;
    probe2_settings.overtravel_port = probe2_settings.probe_port >= 1 ? probe2_settings.probe_port - 1 : 0;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&probe2_settings, sizeof(probe2_settings_t), true);
}

static void warning_no_port (uint_fast16_t state)
{
    report_message("Probe select plugin: configured port number is not available", Message_Warning);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&probe2_settings, nvs_address, sizeof(probe2_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();

    // Sanity check
    if(probe2_settings.probe_port >= n_ports)
        probe2_settings.probe_port = n_ports - 1;

    probe_port = probe2_settings.probe_port;
    probe_settings_bit = 1 << probe_port;

    if(ioport_claim(Port_Digital, Port_Input, &probe_port, "Probe 2")) {

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = mcode_check;
        hal.user_mcode.validate = mcode_validate;
        hal.user_mcode.execute = mcode_execute;

        driver_reset = hal.driver_reset;
        hal.driver_reset = probeReset;

        probe_configure = hal.probe.configure;
        hal.probe.configure = probeConfigure;

        probe_get_state = hal.probe.get_state;
        hal.probe.get_state = probeGetState;

        grbl.on_probe_fixture = probe_fixture;

#if PROBE2_OVERTRAVEL
        overtravel_port = probe2_settings.overtravel_port;
        xbar_t *portinfo = hal.port.get_pin_info(Port_Digital, Port_Input, overtravel_port);

        if(portinfo && !portinfo->cap.claimed && (portinfo->cap.irq_mode & IRQ_Mode_Change) && ioport_claim(Port_Digital, Port_Input, &overtravel_port, "Probe 2 overtravel")) {

            uint32_t invert = !!(setting_get_int_value(setting_get_details(Settings_IoPort_InvertIn, NULL), 0) & (1 << probe2_settings.overtravel_port));

            hal.port.register_interrupt_handler(overtravel_port, invert ? IRQ_Mode_Falling : IRQ_Mode_Rising, on_overtravel);

            control_signals_get_state = hal.control.get_state;
            hal.control.get_state = signalsGetState;
        }
#endif
    } else
        protocol_enqueue_rt_command(warning_no_port);
}

// Settings descriptor used by the core when interacting with this plugin.
static setting_details_t setting_details = {
    .settings = user_settings,
    .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = probe2_settings_descr,
    .n_descriptions = sizeof(probe2_settings_descr) / sizeof(setting_descr_t),
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

        if((ok = hal.port.num_digital_in > 0)) {

            probe_port = --hal.port.num_digital_in;    // "Claim" the port, M66 cannot be used
            probe_settings_bit = 1 << probe_port;

            if(hal.port.set_pin_description)
                hal.port.set_pin_description(Port_Digital, Port_Input, probe_port, "Probe 2");

            memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

            hal.user_mcode.check = mcode_check;
            hal.user_mcode.validate = mcode_validate;
            hal.user_mcode.execute = mcode_execute;

            on_report_options = grbl.on_report_options;
            grbl.on_report_options = report_options;

            driver_reset = hal.driver_reset;
            hal.driver_reset = probeReset;

            probe_configure = hal.probe.configure;
            hal.probe.configure = probeConfigure;

            probe_get_state = hal.probe.get_state;
            hal.probe.get_state = probeGetState;

            grbl.on_probe_fixture = probe_fixture;
        }

    } else if((ok = (n_ports = ioports_available(Port_Digital, Port_Input)) > 0 && (nvs_address = nvs_alloc(sizeof(probe2_settings_t))))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        settings_register(&setting_details);

        // Used for setting value validation
        strcpy(max_port, uitoa(n_ports - 1));
    }

    if(!ok)
        protocol_enqueue_rt_command(warning_msg);
}
