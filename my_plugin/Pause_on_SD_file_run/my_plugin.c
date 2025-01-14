/*
  my_plugin.c - plugin for pausing (enter feed hold) when a SD file is run.
  
  A cycle start command has to be issued to start execution.

  NOTE: build 20210928 or later is required for this plugin.
  
  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "grbl/hal.h"
#include "grbl/gcode.h"

static on_stream_changed_ptr on_stream_changed;
static on_report_options_ptr on_report_options;

static void stream_changed (stream_type_t type)
{
    if(on_stream_changed)
        on_stream_changed(type);

    if(type == StreamType_SDCard)
        gc_execute_block("M1");
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("SD Pause", "0.01");
}

void my_plugin_init (void)
{
    on_stream_changed = grbl.on_stream_changed;
    grbl.on_stream_changed = stream_changed;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}
