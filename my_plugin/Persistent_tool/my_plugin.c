/*
  my_plugin.c - user defined plugin for keeping current number tool over reboot

  Set $485=1 to enable, $485=0 to disable.

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "grbl/hal.h"
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"
#include "grbl/protocol.h"

#if GRBL_BUILD < 20231210
#error Persistent tool plugin requires build 20231210 or later!
#endif

typedef struct {
    bool keep_tool;
    tool_id_t tool_id;
} plugin_settings_t;

static nvs_address_t nvs_address;
static plugin_settings_t my_settings;
static on_report_options_ptr on_report_options;
static on_tool_changed_ptr on_tool_changed;
static on_parser_init_ptr on_parser_init;
static const setting_detail_t user_settings[] = {
    { Setting_EnableToolPersistence, Group_Toolchange, "Keep tool number over reboot", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtended, &my_settings.keep_tool, NULL, NULL }
};

// Write settings to non volatile storage (NVS).
static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
static void plugin_settings_restore (void)
{
    my_settings.keep_tool = false;
    my_settings.tool_id = 0;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);
}

// Load settings from non volatile storage (NVS).
// If load fails restore to default values.
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&my_settings, nvs_address, sizeof(plugin_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();
}

// Add info about our plugin to the $I report.
static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Persistent tool", "0.03");
}

static void onToolChanged (tool_data_t *tool)
{
    if(on_tool_changed)
        on_tool_changed(tool);

    if(my_settings.keep_tool) {
        my_settings.tool_id = tool->tool_id;
        plugin_settings_save();
    }
}

static void onParserInit (parser_state_t *gc_state)
{
    if(on_parser_init)
        on_parser_init(gc_state);

    if(sys.cold_start && my_settings.keep_tool) {
      #if N_TOOLS
        if(my_settings.tool_id <= N_TOOLS)
            gc_state->tool = &grbl.tool_table.tool[my_settings.tool_id];
      #else
        gc_state->tool->tool_id = gc_state->tool_pending = my_settings.tool_id;
      #endif
    }
}

// A call my_plugin_init will be issued automatically at startup.
// There is no need to change any source code elsewhere.
void my_plugin_init (void)
{
    // Settings descriptor used by the core when interacting with this plugin.
    static setting_details_t setting_details = {
        .settings = user_settings,
        .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
        .save = plugin_settings_save,
        .load = plugin_settings_load,
        .restore = plugin_settings_restore
    };
 
    // Try to allocate space for our settings in non volatile storage (NVS).
    if((nvs_address = nvs_alloc(sizeof(plugin_settings_t)))) {

        // Add info about our plugin to the $I report.
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        // Subscribe to parser init event, sets current tool number to stored value on a cold start.
        on_parser_init = grbl.on_parser_init;
        grbl.on_parser_init = onParserInit;

        // Subscribe to tool changed event, write tool number (tool_id) to NVS when triggered.
        on_tool_changed =  grbl.on_tool_changed;
        grbl.on_tool_changed = onToolChanged;

        // Register our settings with the core.
        settings_register(&setting_details);
    }
}
