/*

  modbus_cmd.c - plugin for interacting with Modbus devices via system command

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

*/

/*

This plugin implements the $MODBUSCMD system command.
The following Modbus functions are supported with the following syntax:

Functions 1-4, read many:
$MODBUSCMD=<modbus address>,<function>,<register address base>{,<number of registers>}
<number of registers> defaults to 1 if not specified, max 3.
Returns number of registers read along with their values.

Functions 5-6, write:
$MODBUSCMD=<modbus address>,<function>,<register address>,<value>}
<number of registers> defaults to one if not specified, max 3.
Returns number of registers written.

Functions 7, get exception status:
$MODBUSCMD=<modbus address>,7
Returns exception status

Functions 15-16, write many:
$MODBUSCMD=<modbus address>,<function>,<register address base>,<value>{,<value>{,<value>}}

Both decimal and hexadecimal arguments can be used. Some examples:

$MODBUSCMD=1,4,0,2          // Read status register from a H100 VFD
$MODBUSCMD=1,3,0x200B       // Read status register from a YL620 VFD
$MODBUSCMD=1,6,0x0201,1000  // Set frequency register on a H100 VFD

*/

#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/modbus.h"

static on_report_options_ptr on_report_options;

static void response_handler (modbus_response_t *response)
{
    char buf[100];

    if(response->exception == ModBus_Timeout)
        sprintf(buf, "Modbus timeout: %u", (uint16_t)response->exception);
    else if(response->exception)
        sprintf(buf, "Modbus exception: %u", (uint16_t)response->exception);
    else {
        uint_fast8_t idx;
        sprintf(buf, "Modbus: fn=%hu", response->function);
        for(idx = 0; idx < response->num_values; idx++)
            sprintf(strchr(buf, '\0'), ",%hu(0x%0hx)", response->values[idx], response->values[idx]);
    }

    report_message(buf, 0);
}

static status_code_t modbus_command (sys_state_t state, char *args)
{
    status_code_t status;
    int32_t argc, device, function, address, ivalues[3];

    if(!modbus_isup().ok)
        status = Status_BadNumberFormat;
    else if((argc = sscanf(args, "%li,%li,%li,%li,%li", &device, &function, &address, &ivalues[0], &ivalues[1])) >= 2) {

        const modbus_function_properties_t *fn = modbus_get_function_properties((modbus_function_t)function);

        uint16_t values[3];

        values[0] = (uint16_t)ivalues[0];
        values[1] = (uint16_t)ivalues[1];
        values[1] = (uint16_t)ivalues[2];

        if(fn == NULL || (function != ModBus_ReadExceptionStatus && argc < 3))
            status = Status_InvalidStatement;
        else {
            status = modbus_message((uint8_t)device, fn->function, (uint16_t)address, values, fn->single_register ? 1 : (fn->is_write ? argc - 3 : (argc == 3 ? 1 : values[0])), response_handler);
        }
    } else
        status = Status_BadNumberFormat;

    return status;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin(modbus_isup().ok
                       ? "Modbus command"
                       : "Modbus command (offline)", "0.01");
}

void my_plugin_init (void)
{
    static const sys_command_t command_list[] = {
        {"MODBUSCMD", modbus_command, { .allow_blocking = On }, { .str = "send Modbus message" } },
    };

    static sys_commands_t commands = {
        .n_commands = sizeof(command_list) / sizeof(sys_command_t),
        .commands = command_list
    };

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    system_register_commands(&commands);
}
