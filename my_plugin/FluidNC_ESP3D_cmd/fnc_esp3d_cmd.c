/*
  fnc_esp3d_cmd.c - adds commands to allow the FluidNC fork of ESP32 to run with no(?) errors.

  Part of grblHAL

  Public domain
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "driver.h"
#include "grbl/task.h"

#include <stdio.h>

#if 1

static on_user_command_ptr on_user_command;
static on_report_options_ptr on_report_options;

extern status_code_t stream_file (sys_state_t state, char *fname);

static status_code_t write_response (char *response)
{
    hal.stream.write(response);
    hal.stream.write(ASCII_EOL);

    return Status_OK;
}

static status_code_t write_info (char *info)
{
    hal.stream.write("[MSG:INFO: ");
    hal.stream.write(info);
    hal.stream.write("]" ASCII_EOL);

    return Status_OK;
}

static status_code_t run_sd_file (sys_state_t state, char *args)
{
    return stream_file(state, args);
}

static status_code_t show_startup_log (sys_state_t state, char *args)
{
    char buf[50];

    write_info("FluidNC v3.9.9 https://github.com/bdring/FluidNC");
    write_info("Compiled with ESP32 SDK:v4.4.7-dirty");
    write_info("Local filesystem type is littlefs");
    write_info("Configuration file:config.yaml");
    write_info("Machine Default (Test Drive no I/O)");
    write_info("Board None");
    write_info("Stepping:RMT Pulse:4us Dsbl Delay:0us Dir Delay:0us Idle Delay:255ms");

    sprintf(buf, "Axis count %d", N_AXIS);
    write_info(buf);

    sprintf(buf, "Axis X (%.3f,%.3f)", settings.axis[0].max_travel, 0.0f);
    write_info(buf);
    sprintf(buf, "Axis Y (%.3f,%.3f)", settings.axis[1].max_travel, 0.0f);
    write_info(buf);
    sprintf(buf, "Axis Z (%.3f,%.3f)", settings.axis[2].max_travel, 0.0f);
    write_info(buf);

    write_info("Kinematic system: Cartesian");
    write_info("WiFi on");

/*
    [MSG:INFO: Axis X (-1000.000,0.000)]
    [MSG:INFO:   Motor0]
    [MSG:INFO: Axis Y (-1000.000,0.000)]
    [MSG:INFO:   Motor0]
    [MSG:INFO: Axis Z (-1000.000,0.000)]
    [MSG:INFO:   Motor0]
    [MSG:INFO: Kinematic system: Cartesian]
    [MSG:INFO: Connecting to STA SSID:io-engineering]
    [MSG:INFO: Connecting.]
    [MSG:INFO: Connecting..]
    [MSG:INFO: Connected - IP is 10.0.0.70]
    [MSG:INFO: WiFi on]
    [MSG:INFO: Start mDNS with hostname:http://fluidnc.local/]
    [MSG:INFO: HTTP started on port 80]
    [MSG:INFO: Telnet started on port 23]
*/

    return Status_OK;
}

static void send_report (void *data)
{
    grbl.enqueue_realtime_command(CMD_STATUS_REPORT);

    task_add_delayed(send_report, data, (uint32_t)*((float *)data));
}

static status_code_t set_report_interval (sys_state_t state, char *args)
{
    static float parameter = 0.0f;

    uint_fast8_t counter = 0;

    if(read_float(args, &counter, &parameter)) {

        task_delete(send_report, NULL);

        if(parameter > 0.0f)
            task_add_delayed(send_report, (void *)&parameter, (uint32_t)parameter);
    }

    return Status_OK;
}


// Returns output as html response
static status_code_t onUserCommand (char *cmd)
{
    typedef struct {
        setting_id_t id;
        uint_fast16_t offset;
        const char *name;
    } fluidnc_setting_t;

    static const fluidnc_setting_t fnc_setting[] = {
        { Setting_AxisMaxTravel, 0, "$/axes/x/max_travel_mm" },
        { Setting_AxisMaxTravel, 1, "$/axes/y/max_travel_mm" },
        { Setting_AxisMaxTravel, 2, "$/axes/z/max_travel_mm" },
        { Setting_AxisExtended8, 0, "$/axes/x/homing/positive_direction" },
        { Setting_AxisExtended8, 1, "$/axes/y/homing/positive_direction" },
        { Setting_AxisExtended8, 2, "$/axes/z/homing/positive_direction" },
        { Setting_AxisExtended9, 0, "$/axes/x/homing/mpos_mm" },
        { Setting_AxisExtended9, 1, "$/axes/y/homing/mpos_mm" },
        { Setting_AxisExtended9, 2, "$/axes/z/homing/mpos_mm" },
    };

    vfs_file_t *file;

    if(!strncmp(cmd, "$/", 2) && (file = vfs_open("/stream/qry.txt", "w")) != NULL) {

        char *value = NULL;
        uint_fast32_t idx = sizeof(fnc_setting) / sizeof(fluidnc_setting_t);
        const setting_detail_t *setting = NULL;

        do {
            if(!strcmp(cmd, fnc_setting[--idx].name)) switch(fnc_setting[idx].id) {

                case Setting_AxisExtended8:
                    value = uitoa(bit_istrue(settings.homing.dir_mask.value, bit(fnc_setting[idx].offset)));
                    break;

                case Setting_AxisExtended9:
                    value = ftoa(sys.home_position[fnc_setting[idx].offset], 3);
                    break;

                default:
                    if((setting = setting_get_details(fnc_setting[idx].id, NULL)))
                        value = setting_get_value(setting, fnc_setting[idx].offset);
                    break;
            }
        } while(idx && value == NULL);

        if(value) {
            vfs_puts(fnc_setting[idx].name, file);
            vfs_puts("=", file);
            vfs_puts(value, file);
        }

        vfs_close(file);

        return Status_OK;
    }

    return Status_Unhandled;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("FluidNC $-commands for ESP3D", "0.01");
}

void my_plugin_init (void)
{
    PROGMEM static const sys_command_t fnc_command_list[] = {
        {"SS", show_startup_log, { .noargs = On }, { .str = "show startup log" } },
        {"SD/RUN", run_sd_file, { .noargs = Off }, { .str = "run file on SD card" } },
        {"REPORT/INTERVAL", set_report_interval, { .noargs = Off }, { .str = "set auto report interval" } },
    };

    static sys_commands_t fnc_commands = {
        .n_commands = sizeof(fnc_command_list) / sizeof(sys_command_t),
        .commands = fnc_command_list
    };

    system_register_commands(&fnc_commands);

    on_user_command = grbl.on_user_command;
    grbl.on_user_command = onUserCommand;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#endif
