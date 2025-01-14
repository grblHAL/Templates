/*

  my_plugin.c - plugin for binding macros to aux input pins

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  Up to 4 macros can be bound to input pins by changing the N_MACROS symbol below.
  Each macro can be up to 127 characters long, blocks (lines) are separated by a vertical bar character: |
  Setting numbers $450 - $453 are for defining the macro content.
  Setting numbers $454 - $457 are for configuring which aux input port to assign to each macro.
  NOTES: If the driver does not support mapping of port numbers settings $454 - $457 will not be available.
         The mapped pins has to be interrupt capable and support falling interrupt mode.
         The controller must be in Idle mode when starting macros.

  Examples:
    $450=G0Y5|G1X0F50
    $451=G0x0Y0Z0

  Tip: use the $pins command to check the port mapping.
*/

#define N_MACROS 2 // MAX 4

#include <stdio.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/platform.h"

#if N_MACROS > 4
#undef N_MACROS
#define N_MACROS 4
#endif

typedef struct {
    uint8_t port;
    char data[128];
} macro_setting_t;

typedef struct {
    macro_setting_t macro[N_MACROS];
} macro_settings_t;

static bool is_executing = false;
static uint8_t n_ports;
uint8_t port[N_MACROS];
static char max_port[4], *command;
static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static macro_settings_t plugin_settings;
static stream_read_ptr stream_read;
static driver_reset_ptr driver_reset;

static int16_t get_macro_char (void);

// Ends macro execution if currently running
// and restores normal operation.
static void end_macro (void)
{
    is_executing = false;
    if(hal.stream.read == get_macro_char) {
        hal.stream.read = stream_read;
        report_init_fns();
    }
}

// Called on a soft reset so that normal operation can be restored.
static void plugin_reset (void)
{
    end_macro();    // End macro if currently running.
    driver_reset(); // Call the next reset handler in the chain.
}

// Macro stream input function.
// Reads character by character from the macro and returns them when
// requested by the foreground process.
static int16_t get_macro_char (void)
{
    static bool eol_ok = false;

    if(*command == '\0') {                          // End of macro?
        end_macro();                                // If end reading from it
        return eol_ok ? SERIAL_NO_DATA : ASCII_LF;  // and return a linefeed if the last character was not a linefeed.
    }

    char c = *command++;    // Get next character.

    if((eol_ok = c == '|')) // If character is vertical bar
        c = ASCII_LF;       // return a linefeed character.

    return (uint16_t)c;
}

// This code will be executed after each command is sent to the parser,
// If an error is detected macro execution will be stopped and the status_code reported.
static status_code_t trap_status_report (status_code_t status_code)
{
    if(status_code != Status_OK) {
        char msg[30];
        sprintf(msg, "error %d in macro", (uint8_t)status_code);
        report_message(msg, Message_Warning);
        end_macro();
    }

    return status_code;
}

// Actual start of macro execution.
static void run_macro (void *data)
{
    if(state_get() == STATE_IDLE && hal.stream.read != get_macro_char) {
        stream_read = hal.stream.read;                      // Redirect input stream to read from the macro instead of
        hal.stream.read = get_macro_char;                   // the active stream. This ensures that input streams are not mingled.
        grbl.report.status_message = trap_status_report;    // Add trap for status messages so we can terminate on errors.
    }
}

// On falling interrupt run macro if machine is in Idle state.
// Since this function runs in an interrupt context actual start of execution
// is registered as a single run task to be started from the foreground process.
ISR_CODE static void execute_macro (uint8_t irq_port, bool is_high)
{
    if(!is_high && !is_executing && state_get() == STATE_IDLE) {

        // Determine macro to run from port number
        uint_fast8_t idx = N_MACROS;
        do {
            idx--;
        } while(idx && port[idx] != irq_port);

        command = plugin_settings.macro[idx].data;
        if(!(*command == '\0' || *command == 0xFF)) {    // If valid command
            is_executing = true;
            protocol_enqueue_foreground_task(run_macro, NULL);     // register run_macro function to be called from foreground process.
        }
    }
}

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.

static const setting_group_detail_t macro_groups [] = {
    { Group_Root, Group_UserSettings, "Macros"}
};

static status_code_t set_port (setting_id_t setting, float value)
{
    status_code_t status;
    uint_fast8_t idx = setting - Setting_UserDefined_4;

    if((status = isintf(value) ? Status_OK : Status_BadNumberFormat) == Status_OK)
        plugin_settings.macro[idx].port = value < 0.0f ? 255 : (uint8_t)value;

    return status;
}

static float get_port (setting_id_t setting)
{
    uint_fast8_t idx = setting - Setting_UserDefined_4;

    return plugin_settings.macro[idx].port;
}

static const setting_detail_t macro_settings[] = {
    { Setting_UserDefined_0, Group_UserSettings, "Macro 1", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[0].data, NULL, NULL },
#if N_MACROS > 1
    { Setting_UserDefined_1, Group_UserSettings, "Macro 2", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[1].data, NULL, NULL },
#endif
#if N_MACROS > 2
    { Setting_UserDefined_2, Group_UserSettings, "Macro 3", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[2].data, NULL, NULL },
#endif
#if N_MACROS > 3
    { Setting_UserDefined_3, Group_UserSettings, "Macro 4", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[3].data, NULL, NULL },
#endif
    { Setting_UserDefined_4, Group_AuxPorts, "Macro 1 port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } },
#if N_MACROS > 1
    { Setting_UserDefined_5, Group_AuxPorts, "Macro 2 port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } },
#endif
#if N_MACROS > 2
    { Setting_UserDefined_6, Group_AuxPorts, "Macro 3 port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } },
#endif
#if N_MACROS > 3
    { Setting_UserDefined_7, Group_AuxPorts, "Macro 4 port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } },
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t macro_settings_descr[] = {
    { Setting_UserDefined_0, "Macro content for macro 1, separate blocks (lines) with the vertical bar character |." },
#if N_MACROS > 1
    { Setting_UserDefined_1, "Macro content for macro 2, separate blocks (lines) with the vertical bar character |." },
#endif
#if N_MACROS > 2
    { Setting_UserDefined_2, "Macro content for macro 3, separate blocks (lines) with the vertical bar character |." },
#endif
#if N_MACROS > 3
    { Setting_UserDefined_3, "Macro content for macro 4, separate blocks (lines) with the vertical bar character |." },
#endif
    { Setting_UserDefined_4, "Aux port number to use for the Macro 1 start pin input. Set to -1 to disable." },
#if N_MACROS > 1
    { Setting_UserDefined_5, "Aux port number to use for the Macro 2 start pin input. Set to -1 to disable." },
#endif
#if N_MACROS > 2
    { Setting_UserDefined_6, "Aux port number to use for the Macro 3 start pin input. Set to -1 to disable." },
#endif
#if N_MACROS > 3
    { Setting_UserDefined_7, "Aux port number to use for the Macro 4 start pin input. Set to -1 to disable." },
#endif
};

#endif

// Write settings to non volatile storage (NVS).
static void macro_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(macro_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
static void macro_settings_restore (void)
{
    uint_fast8_t idx = N_MACROS, port = n_ports > N_MACROS ? n_ports - N_MACROS : 0;

    // Register empty macro strings and set default port numbers if mapping is available.
    for(idx = 0; idx < N_MACROS; idx++) {
         plugin_settings.macro[idx].port = port++;
        *plugin_settings.macro[idx].data = '\0';
    };

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(macro_settings_t), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void macro_settings_load (void)
{
    uint_fast8_t idx = N_MACROS, n_ok = 0, n_enabled = 0;
    xbar_t *pin = NULL;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(macro_settings_t), true) != NVS_TransferResult_OK)
        macro_settings_restore();

    do {
        idx--;
        if((port[idx] = plugin_settings.macro[idx].port) == 0xFF)
            continue;
        n_enabled++;
        if((pin = ioport_get_info(Port_Digital, Port_Input, port[idx])) && !(pin->cap.irq_mode & IRQ_Mode_Falling)) // Is port interrupt capable?
            port[idx] = 0xFF;                                                                                       // No, flag it as not claimed.
        else if(ioport_claim(Port_Digital, Port_Input, &port[idx], "Macro pin")) {                                  // Try to claim the port.
            if(pin->cap.debounce) {
                gpio_in_config_t config = {
                    .debounce = On,
                    .pull_mode = PullMode_Up
                };
                pin->config(pin, &config, false);
            }
        } else
            port[idx] = 0xFF;
    } while(idx);

    // Then try to register the interrupt handler.
    idx = N_MACROS;
    do {
        idx--;
        if(port[idx] != 0xFF && hal.port.register_interrupt_handler(port[idx], IRQ_Mode_Falling, execute_macro))
            n_ok++;
    } while(idx);

    if(n_ok < n_enabled)
        protocol_enqueue_foreground_task(report_warning, "Macro plugin failed to claim all needed ports!");
}

// Add info about our plugin to the $I report.
static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Macro plugin (PD)", "0.03");
}

// A call my_plugin_init will be issued automatically at startup.
// There is no need to change any source code elsewhere.
void my_plugin_init (void)
{
    // Settings descriptor used by the core when interacting with this plugin.
    static setting_details_t setting_details = {
        .groups = macro_groups,
        .n_groups = sizeof(macro_groups) / sizeof(setting_group_detail_t),
        .settings = macro_settings,
        .n_settings = sizeof(macro_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = macro_settings_descr,
        .n_descriptions = sizeof(macro_settings_descr) / sizeof(setting_descr_t),
    #endif
        .save = macro_settings_save,
        .load = macro_settings_load,
        .restore = macro_settings_restore
    };

    // If resources available register our plugin with the core.
    if(ioport_can_claim_explicit() &&
       (n_ports = ioports_available(Port_Digital, Port_Input)) &&
        (nvs_address = nvs_alloc(sizeof(macro_settings_t)))) {

        // Used for setting value validation.
        strcpy(max_port, uitoa(n_ports - 1));

        // Register settings.
        settings_register(&setting_details);

        // Add our plugin to the $I options report.
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        // Hook into the driver reset chain so we
        // can restore normal operation if a reset happens
        // when a macro is running.
        driver_reset = hal.driver_reset;
        hal.driver_reset = plugin_reset;
    } else
        protocol_enqueue_foreground_task(report_warning, "Macro plugin failed to initialize!");
}
