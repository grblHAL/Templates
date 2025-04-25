/*
  my_plugin.c - plugin template for using an auxiliary input for a second probe input.
                optionally a second input can be assigned for an overtravel input (by compile time option).

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  Use the $pins command to find out which input port/pin is used, it will be labeled "Probe 2".

  NOTE: If no auxiliary input is available it will not install itself.

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
#include "grbl/task.h"

#include <math.h>
#include <string.h>

typedef enum {
    ProbeMode_AtG59_3 = 0,
    ProbeMode_ToolChangeAtG59_3,
    ProbeMode_ToolChange,
    ProbeMode_Always,
    ProbeMode_Never,
    ProbeMode_Manual,
    ProbeMode_MaxValue = ProbeMode_Manual,
} probe_mode_t;

static uint8_t probe_port;
static bool use_probe2 = false;
static probe_state_t probe = {
    .connected = On
};
static probe_mode_t probe_mode = ProbeMode_AtG59_3; // Default mode
static driver_reset_ptr driver_reset;
static probe_configure_ptr probe_configure;
static probe_get_state_ptr probe_get_state;
static on_probe_toolsetter_ptr on_probe_toolsetter;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;
static uint8_t overtravel_port;
static control_signals_get_state_ptr control_signals_get_state;

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
bool probeToolSetter (tool_data_t *tool, coord_data_t *position, bool at_g59_3, bool on)
{
    if(on_probe_toolsetter)
        on_probe_toolsetter(tool, position, at_g59_3, on);

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

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    xbar_t *portinfo = hal.port.get_pin_info(Port_Digital, Port_Input, probe_port);

    probe.inverted = is_probe_away ? !portinfo->mode.inverted : portinfo->mode.inverted;
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

static status_code_t set_port (setting_id_t setting, float value)
{
    status_code_t status;

    if((status = isintf(value) ? Status_OK : Status_BadNumberFormat) == Status_OK)
      switch(setting) {

        case PROBE_PLUGIN_SETTING:
            probe2_settings.probe_port = value < 0.0f ? 255 : (uint8_t)value;
            break;

        case PROBE_PLUGIN_SETTING1:
            probe2_settings.overtravel_port = value < 0.0f ? 255 : (uint8_t)value;
            break;

        default:
            break;
    }

    return status;
}

static float get_port (setting_id_t setting)
{
    float value = 0.0f;

    switch(setting) {

        case PROBE_PLUGIN_SETTING:
            value = probe2_settings.probe_port >= n_ports ? -1.0f : (float)probe2_settings.probe_port;
            break;

        case PROBE_PLUGIN_SETTING1:
            value = probe2_settings.overtravel_port >= n_ports ? -1.0f : (float)probe2_settings.overtravel_port;
            break;

        default:
            break;
    }

    return value;
}

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.
static const setting_detail_t user_settings[] = {
    { PROBE_PLUGIN_SETTING, Group_Probing, "Probe 2 aux port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } },
    { PROBE_PLUGIN_SETTING1, Group_Probing, "Probe 2 overtravel aux port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t probe2_settings_descr[] = {
    { PROBE_PLUGIN_SETTING, "Aux port number to use for second probe input. Set to -1 to disable." },
    { PROBE_PLUGIN_SETTING1, "Aux port number to use for second probe overtravel input. Set to -1 to disable.\\n"
                             "If asserted Z hard limit alarm will raised.\\n\\n"
                             "NOTE: if input inversion is changed with $370 a hard reset is required to reconfigure the port!"
    }
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
    probe2_settings.overtravel_port = 0xFF;

    // Find highest numbered port, or keep the current one if found.
    probe2_settings.probe_port = ioport_find_free(Port_Digital, Port_Input, (pin_cap_t){ .claimable = On }, "Probe 2");

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&probe2_settings, sizeof(probe2_settings_t), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&probe2_settings, nvs_address, sizeof(probe2_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();

    // Sanity checks
    if(probe2_settings.probe_port >= n_ports)
        probe2_settings.probe_port = 0xFF;
    if(probe2_settings.overtravel_port >= n_ports)
        probe2_settings.overtravel_port = 0xFF;

    if((probe_port = probe2_settings.probe_port) != 0xFF) {

        xbar_t *portinfo = ioport_get_info(Port_Digital, Port_Input, probe_port);

        if(portinfo && ioport_claim(Port_Digital, Port_Input, &probe_port, "Probe 2")) {

            memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

            grbl.user_mcode.check = mcode_check;
            grbl.user_mcode.validate = mcode_validate;
            grbl.user_mcode.execute = mcode_execute;

            driver_reset = hal.driver_reset;
            hal.driver_reset = probeReset;

            probe_configure = hal.probe.configure;
            hal.probe.configure = probeConfigure;

            probe_get_state = hal.probe.get_state;
            hal.probe.get_state = probeGetState;

            on_probe_toolsetter = grbl.on_probe_toolsetter;
            grbl.on_probe_toolsetter = probeToolSetter;

        } else
            task_run_on_startup(report_warning, "Probe select plugin: probe port is not available");
    }

    if((overtravel_port = probe2_settings.overtravel_port) != 0xFF) {

        xbar_t *portinfo = ioport_get_info(Port_Digital, Port_Input, overtravel_port);
        if(portinfo && (portinfo->cap.irq_mode & IRQ_Mode_Change) && ioport_claim(Port_Digital, Port_Input, &overtravel_port, "Probe 2 overtravel")) {

            hal.port.register_interrupt_handler(overtravel_port, portinfo->mode.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising, on_overtravel);

            control_signals_get_state = hal.control.get_state;
            hal.control.get_state = signalsGetState;
        } else
            task_run_on_startup(report_warning, "Probe select plugin: overtravel port is not available");
    }
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Probe select 2", "0.05");
}

void my_plugin_init (void)
{
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

    if(ioport_can_claim_explicit() &&
       (n_ports = ioports_available(Port_Digital, Port_Input)) &&
        (nvs_address = nvs_alloc(sizeof(probe2_settings_t)))) {

        // Used for setting value validation
        strcpy(max_port, uitoa(n_ports - 1));

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        settings_register(&setting_details);

    } else
        task_run_on_startup(report_warning, "Probe select plugin failed to initialize!");
}
