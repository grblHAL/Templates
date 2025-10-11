/*
  my_plugin.c - plugin for monitoring motor power

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  On power loss alarm 17 is raised (Motor fault).
  On alarm cleared or soft reset a M122I command is issued to reinit Trinamic drivers if power is back on.

  Setting $450 is for configuring which aux input port to assign for monitoring.
  NOTE: If the driver does not support mapping of port number settings $450 will not be available.
        The mapped pin has to be interrupt capable and support change (falling and rising) interrupt mode.

  Tip: use the $pins command to check the port mapping.
*/

#include <stdio.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/task.h"

typedef enum {
    Power_On = 0,
    Power_Alarm,
    Power_Lost
} power_state_t;

typedef struct {
    uint8_t port;
} power_settings_t;

static uint8_t port;
static io_port_cfg_t d_in;
static power_state_t power_state = Power_On;
static nvs_address_t nvs_address;
static power_settings_t plugin_settings;

static on_report_options_ptr on_report_options;

static status_code_t set_port (setting_id_t setting, float value)
{
    return d_in.set_value(&d_in, &plugin_settings.port, (pin_cap_t){ .irq_mode = IRQ_Mode_Change }, value);;
}

static float get_port (setting_id_t setting)
{
    return d_in.get_value(&d_in, plugin_settings.port);
}

// Use a float (decimal) setting with getter/setter functions so -1 can be used to disable the plugin.
static const setting_detail_t power_settings[] = {
    { Setting_UserDefined_0, Group_AuxPorts, "Power monitor port", NULL, Format_Decimal, "-#0", "-1", d_in.port_maxs, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } }
};

static const setting_descr_t power_settings_descr[] = {
    { Setting_UserDefined_0, "Auxiliary port to use for stepper power monitoring. Set to -1 to disable." }
};

// Write settings to non volatile storage (NVS).
static void power_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(power_settings_t), true);
}

static void check_power_restored (void *data)
{
    if(hal.port.wait_on_input(Port_Digital, port, WaitMode_Immediate, 0.0f) == 1) {

        power_state = Power_On;

        report_message("Motor power restored", Message_Info);
        // Reset stepper drivers
        if(hal.stepper.status)
            hal.stepper.status(true);

    } else
        task_add_delayed(check_power_restored, NULL, 250);
}

// Raise motor fault alarm.
static void raise_power_alarm (void *data)
{
    if(power_state == Power_Alarm) {
        system_raise_alarm(Alarm_MotorFault);
        task_add_delayed(check_power_restored, NULL, 250);
    }

    power_state = Power_Lost;
}

// Called when power sensing pin changes state.
// Raises alarm or sets up wait for alarm reset before taking action.
static void on_power_change (uint8_t port, bool state)
{
    if(!state) {
        if(power_state == Power_On) {
            protocol_enqueue_foreground_task(raise_power_alarm, NULL);
            power_state = Power_Alarm;
        }
    }
}

// Restore default settings and write to non volatile storage (NVS).
static void power_settings_restore (void)
{
    // Find highest numbered port that supports change interrupt, or keep the current one if found.
    plugin_settings.port = d_in.get_next(&d_in, IOPORT_UNASSIGNED, "Motor supply monitor", (pin_cap_t){ .irq_mode = IRQ_Mode_Change });

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(power_settings_t), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void power_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(power_settings_t), true) != NVS_TransferResult_OK)
        power_settings_restore();

    // Sanity check
    if(plugin_settings.port >= d_in.n_ports)
        plugin_settings.port = IOPORT_UNASSIGNED;

    // If port is valid try claiming it, if successful register an interrupt handler.
    if((port = plugin_settings.port) != IOPORT_UNASSIGNED) {
        if(d_in.claim(&d_in, &port, "Motor supply monitor", (pin_cap_t){ .irq_mode = IRQ_Mode_Change })) {
            ioport_enable_irq(port, IRQ_Mode_Change, on_power_change);
            // TODO add check for power present and raise alarm if not?
        } else
            protocol_enqueue_foreground_task(report_warning, "Motor supply monitor plugin failed to claim needed port!");
    }
}

// Add info about our plugin to the $I report.
static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Motor supply monitor", "0.04");
}

// A call my_plugin_init will be issued automatically at startup.
// There is no need to change any source code elsewhere.
void my_plugin_init (void)
{
    // Settings descriptor used by the core when interacting with this plugin.
    static setting_details_t setting_details = {
        .settings = power_settings,
        .n_settings = sizeof(power_settings) / sizeof(setting_detail_t),
        .descriptions = power_settings_descr,
        .n_descriptions = sizeof(power_settings_descr) / sizeof(setting_descr_t),
        .save = power_settings_save,
        .load = power_settings_load,
        .restore = power_settings_restore
    };

    // If auxiliary input available and enough free non volatile memory register our plugin with the core.
    if(ioports_cfg(&d_in, Port_Digital, Port_Input)->n_ports && (nvs_address = nvs_alloc(sizeof(power_settings_t)))) {

        // Register settings.
        settings_register(&setting_details);

        // Add our plugin to the $I options report.
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;
    } else
        protocol_enqueue_foreground_task(report_warning, "Motor supply monitor plugin failed to claim needed port!");
}
