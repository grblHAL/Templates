/*

  my_plugin.c - plugin template for setting auxiliary output on feed hold

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <string.h>

#include "driver.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

#define PLUGIN_SETTING Setting_UserDefined_9

static uint8_t port;
static uint8_t n_ports;
static char max_port[4];

typedef struct {
    uint8_t port;
} plugin_settings_t;

static nvs_address_t nvs_address;
static plugin_settings_t plugin_settings;
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

static status_code_t set_port (setting_id_t setting, float value)
{
    if(!isintf(value))
        return Status_BadNumberFormat;

    plugin_settings.port = value < 0.0f ? 0xFF : (uint8_t)value;

    return Status_OK;
}

static float get_port (setting_id_t setting)
{
    return plugin_settings.port >= n_ports ? -1.0f : (float) plugin_settings.port;
}

static const setting_detail_t user_settings[] = {
    { PLUGIN_SETTING, Group_AuxPorts, "Feed hold aux port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t plugin_settings_descr[] = {
    { PLUGIN_SETTING, "Aux port number to use for feed hold output. Set to -1 to disable." }
};

#endif

// Write settings to non volatile storage (NVS).
static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(plugin_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
static void plugin_settings_restore (void)
{
    // Find highest numbered port, or keep the current one if found.
    plugin_settings.port = ioport_find_free(Port_Digital, Port_Output, (pin_cap_t){ .claimable = On }, "Feed hold out");

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(plugin_settings_t), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(plugin_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();

    // Sanity check
    if(plugin_settings.port >= n_ports)
        plugin_settings.port = 0xFF;

    if((port = plugin_settings.port) != 0xFF) {
        if(ioport_claim(Port_Digital, Port_Output, &port, "Feed hold out")) {
            on_state_change = grbl.on_state_change;         // Subscribe to the state changed event by saving away the original
            grbl.on_state_change = onStateChanged;          // function pointer and adding ours to the chain.
        } else
            protocol_enqueue_foreground_task(report_warning, "Feed hold plugin: configured port number is not available");
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);  // Call previous function in the chain.

    if(!newopt)                 // Add info about us to the $I report.
        report_plugin("PLUGIN Template 2", "v0.02");
}

void my_plugin_init (void)
{
    // Settings descriptor used by the core when interacting with this plugin.
    static setting_details_t setting_details = {
        .settings = user_settings,
        .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = plugin_settings_descr,
        .n_descriptions = sizeof(plugin_settings_descr) / sizeof(setting_descr_t),
    #endif
        .save = plugin_settings_save,
        .load = plugin_settings_load,
        .restore = plugin_settings_restore
    };

    if(ioport_can_claim_explicit() &&
       (n_ports = ioports_available(Port_Digital, Port_Output)) &&
        (nvs_address = nvs_alloc(sizeof(plugin_settings_t)))) {

        strcpy(max_port, uitoa(n_ports - 1));

        settings_register(&setting_details);

        on_report_options = grbl.on_report_options;     // Add our plugin to to the options report chain
        grbl.on_report_options = onReportOptions;       // to tell the user we are active.
    }
}
