/*

  modbus_cmd.c - plugin for interacting with Modbus devices via system commands

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

This plugin implements the $MODBUSCMD and $MODBUSDBG system commands.

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

---

$MODBUSDBG - enable debug output, this outputs messages containing the transmitted and received data, an example:

[MSG:TX: 01 03 21 03 00 01 7E 36]
[MSG:RX: 01 03 02 00 00 B8 44]

$MODBUSDBG=0 - disable debug output.

*/

#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/modbus.h"
#include "grbl/task.h"

static modbus_rtu_stream_t stream;

static on_report_options_ptr on_report_options;

struct {
    uint_fast8_t idx;
    uint8_t adu[MODBUS_MAX_ADU_SIZE];
} modbus_rx;

// $MODBUSDBG

static void modbus_rx_complete (void *data)
{
    static char buf[3 * MODBUS_MAX_ADU_SIZE + 3] = "RX:";

    if(modbus_rx.idx) {

        uint_fast8_t idx;
        char *p = buf + 3;

        for(idx = 0; idx < modbus_rx.idx; idx ++)
            p += sprintf(p, " %02X", modbus_rx.adu[idx]);

        modbus_rx.idx = 0;
        task_add_immediate(report_plain, buf);
    }
}

static int32_t modbus_read (void)
{
    int32_t c = stream.read();

    if(c != SERIAL_NO_DATA && modbus_rx.idx < MODBUS_MAX_ADU_SIZE) {
        modbus_rx.adu[modbus_rx.idx++] = (uint8_t)c;
        if(modbus_rx.idx > 3) {
            task_delete(modbus_rx_complete, NULL);
            task_add_delayed(modbus_rx_complete, NULL, 3);
        }
    }

    return c;
}

static void modbus_write (const uint8_t *s, uint16_t len)
{
    static char buf[3 * MODBUS_MAX_ADU_SIZE + 3] = "TX:";

    uint_fast8_t idx;
    char *p = buf + 3;

    stream.write(s, len);

    for(idx = 0; idx < len; idx ++)
        p += sprintf(p, " %02X", s[idx]);

    task_add_immediate(report_plain, buf);
}

static void modbus_set_direction (bool tx)
{
    if(stream.set_direction)
        stream.set_direction(tx);

    task_delete(modbus_rx_complete, NULL);

    if(tx && modbus_rx.idx)
        modbus_rx_complete(NULL);
}

static status_code_t modbus_debug (sys_state_t state, char *args)
{
    modbus_rtu_stream_t *s;

    if(*args == '0' && *(args + 1) == '\0') {
        s = modbus_get_rtu_stream();
        if(s && s->read == modbus_read) {
            s->read = stream.read;
            s->write = stream.write;
            s->set_direction = stream.set_direction;
            stream.read = NULL;
        }
    } else if(*args == '\0' && stream.read == NULL && (s = modbus_get_rtu_stream())) {

        memcpy(&stream, s, sizeof(stream));

        s->read = modbus_read;
        s->write = modbus_write;
        s->set_direction = modbus_set_direction;
    }

    return Status_OK;
}

// $MODBUSCMD

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

//

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
        {"MODBUSDBG", modbus_debug, { .allow_blocking = On }, { .str = "send Modbus message" } }
    };

    static sys_commands_t commands = {
        .n_commands = sizeof(command_list) / sizeof(sys_command_t),
        .commands = command_list
    };

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    system_register_commands(&commands);
}
