/*

  my_plugin.c - user defined plugin template with settings handling

  Part of grblHAL

  Public domain.

*/

/*
 * NOTE: this plugin does not add any other fuctionality than settings handling, attach to other HAL entry points to provide that.
 *       See the mcode.c template or standard/template plugins for how to do this.
 */

#include "grbl/hal.h"
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"

typedef struct {
    float fvalue;
    uint16_t ivalue;
} plugin_settings_t;

static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static plugin_settings_t my_settings;

static status_code_t set_my_setting1 (setting_id_t id, uint16_t value)
{
    my_settings.ivalue = value;

    // do some stuff related to changes in the setting value

    return Status_OK;
}

static uint16_t get_my_setting1 (setting_id_t setting)
{
    return my_settings.ivalue;
}

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.
static const setting_group_detail_t user_groups [] = {
    { Group_Root, Group_UserSettings, "My settings"}
};

// Setting_UserDefined_0 is read and written by the core via the &my_settings.fvalue pointer.
// Setting_UserDefined_1 is read and written via calls to get_my_setting1() and set_my_setting1().
// - this is useful for settings that requires immediate action on a change and/or value transformation.
static const setting_detail_t user_settings[] = {
    { Setting_UserDefined_0, Group_UserSettings, "My setting 1", NULL, Format_Decimal, "#0.0", "0", "15", Setting_NonCore, &my_settings.fvalue, NULL, NULL },
    { Setting_UserDefined_1, Group_UserSettings, "My setting 2", "milliseconds", Format_Int16, "####0", "50", "250", Setting_NonCoreFn, set_my_setting1, get_my_setting1, NULL }
};

// Write settings to non volatile storage (NVS).
static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
static void plugin_settings_restore (void)
{
    my_settings.fvalue = 3.1f;
    my_settings.ivalue = 2;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&my_settings, nvs_address, sizeof(plugin_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();
}

// Settings descriptor used by the core when interacting with this plugin.
static setting_details_t setting_details = {
    .groups = user_groups,
    .n_groups = sizeof(user_groups) / sizeof(setting_group_detail_t),
    .settings = user_settings,
    .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
    .save = plugin_settings_save,
    .load = plugin_settings_load,
    .restore = plugin_settings_restore
};

// Add info about our plugin to the $I report.
static void on_report_my_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Settings template plugin v1.03]" ASCII_EOL);
}

// A call my_plugin_init will be issued automatically at startup.
// There is no need to change any source code elsewhere.
void my_plugin_init (void)
{
    // Try to allocate space for our settings in non volatile storage (NVS).
    if((nvs_address = nvs_alloc(sizeof(plugin_settings_t)))) {

        // Add info about our plugin to the $I report.
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = on_report_my_options;

        // Register our settings with the core.
        settings_register(&setting_details);

        // "Hook" into other HAL pointers here to provide functionality.
    }
}
