/*
  my_plugin.c - tool offset for probing @ G59.3

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

  Set radius: M102 P<slot> T<tool> R<tool radius>
  Clear slot: M102 P<slot> T0
*/

#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/nvs_buffer.h"
#else
#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#endif

#define N_TOOL_RADIUS 5

typedef struct {
    tool_id_t tool_id;
    float radius; // approx. radius of cutting edges
} tool_radius_t;

static nvs_address_t nvs_address;
static tool_radius_t tool_radius[N_TOOL_RADIUS];
static on_probe_toolsetter_ptr on_probe_toolsetter;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static on_report_ngc_parameters_ptr on_report_ngc_parameters;

static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&tool_radius, sizeof(tool_radius), true);
}

static void plugin_settings_restore (void)
{
    memset(tool_radius, 0, sizeof(tool_radius));

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&tool_radius, sizeof(tool_radius), true);
}

static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&tool_radius, nvs_address, sizeof(tool_radius), true) != NVS_TransferResult_OK)
        plugin_settings_restore();
}

static setting_details_t setting_details = {
//    .settings = user_settings,
    .n_settings = 0, //sizeof(user_settings) / sizeof(setting_detail_t),
    .save = plugin_settings_save,
    .load = plugin_settings_load,
    .restore = plugin_settings_restore
};

static user_mcode_type_t check (user_mcode_t mcode)
{
    return mcode == UserMCode_Generic2 ? UserMCode_Normal : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t validate (parser_block_t *gc_block)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

        case UserMCode_Generic2:
            if(gc_block->words.p && !(isnanf(gc_block->values.p) || isintf(gc_block->values.p)))
                state = Status_BadNumberFormat;

            if(gc_block->words.r && isnanf(gc_block->values.r))
                state = Status_BadNumberFormat;

            if(state != Status_BadNumberFormat && gc_block->words.p && gc_block->words.t) {
                if(gc_block->values.p >= 0.0f && gc_block->values.p <= (float)N_TOOL_RADIUS) {
                    state = Status_OK;
                    gc_block->words.p = gc_block->words.r = gc_block->words.t = Off;
                    gc_block->user_mcode_sync = true;
                } else
                    state = Status_GcodeValueOutOfRange;
            }
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void execute (sys_state_t state, parser_block_t *gc_block) {

    bool handled = true;

    switch(gc_block->user_mcode) {

        case UserMCode_Generic2:
            tool_radius[(uint32_t)gc_block->values.p].tool_id = gc_block->words.t ? gc_block->values.t : 0;
            tool_radius[(uint32_t)gc_block->values.p].radius = gc_block->words.r ? gc_block->values.r : 0.0f;
            plugin_settings_save();
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static bool onProbeToolsetter (tool_data_t *tool, coord_data_t *position, bool at_g59_3, bool on)
{
    if(tool && tool->tool_id > 0 && position && on) {
        uint_fast8_t idx;
        for(idx = 0; idx < N_TOOL_RADIUS; idx++) {
            if(tool_radius[idx].tool_id == tool->tool_id) {
                position->x += tool_radius[idx].radius;
                break;
            }
        }
    }

    return on_probe_toolsetter ? on_probe_toolsetter(tool, position, at_g59_3, on) : true;
}

void onReportNgcParameters (void)
{
    uint_fast8_t idx;
    for(idx = 0; idx < N_TOOL_RADIUS; idx++) {
        if(tool_radius[idx].tool_id > 0) {
            hal.stream.write("[TRA:");
            hal.stream.write(uitoa(idx));
            hal.stream.write(",");
            hal.stream.write(uitoa(tool_radius[idx].tool_id));
            hal.stream.write(",");
            hal.stream.write(ftoa(tool_radius[idx].radius, ngc_float_decimals()));
            hal.stream.write("]" ASCII_EOL);
        }
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Toolsetter tool offset", "0.01");
}

void my_plugin_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(tool_radius)))) {

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

        grbl.user_mcode.check = check;
        grbl.user_mcode.validate = validate;
        grbl.user_mcode.execute = execute;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_probe_toolsetter = grbl.on_probe_toolsetter;
        grbl.on_probe_toolsetter = onProbeToolsetter;

        on_report_ngc_parameters = grbl.on_report_ngc_parameters;
        grbl.on_report_ngc_parameters = onReportNgcParameters;

        settings_register(&setting_details);
    }
}
