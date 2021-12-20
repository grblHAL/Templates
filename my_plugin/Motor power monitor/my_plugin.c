/*

  my_plugin.c - plugin for monitoring motor power

  Part of grblHAL

  Public domain.

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

typedef enum {
    Power_On = 0,
    Power_Alarm,
    Power_Lost
} power_state_t;

typedef struct {
    uint8_t port;
} power_settings_t;

static uint8_t port, n_ports;
static char max_port[4];
static power_state_t power_state = Power_On;
static nvs_address_t nvs_address;
static power_settings_t plugin_settings;
static on_report_options_ptr on_report_options;
static settings_changed_ptr settings_changed;
static on_state_change_ptr on_state_change = NULL;
static driver_reset_ptr driver_reset;

static const setting_detail_t power_settings[] = {
    { Setting_UserDefined_0, Group_AuxPorts, "Macro 1 port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &plugin_settings.port, NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t power_settings_descr[] = {
    { Setting_UserDefined_0, "Macro content for macro 1, separate blocks (lines) with the vertical bar character |." },
};

#endif

// Write settings to non volatile storage (NVS).
static void power_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(power_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
static void power_settings_restore (void)
{
    if(ioport_can_claim_explicit()) {

        xbar_t *portinfo;
        uint8_t port = n_ports;

        plugin_settings.port = 0;

        // Find highest numbered port that supports change interrupt.
        if(port > 0) do {
            port--;
            if((portinfo = hal.port.get_pin_info(Port_Digital, Port_Input, port))) {
                if(!portinfo->cap.claimed && (portinfo->cap.irq_mode & IRQ_Mode_Change)) {
                    plugin_settings.port = port;
                    break;
                }
            }
        } while(port);
    }

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(power_settings_t), true);
}

static void check_power_restored (void)
{
    if(hal.port.wait_on_input(Port_Digital, port, WaitMode_Immediate, 0.0f) == 1) {

        power_state = Power_On;

        report_message("Motor power restored", Message_Info);
        grbl.enqueue_gcode("M122I");

        if(on_state_change != NULL) {
            grbl.on_state_change = on_state_change;
            on_state_change = NULL;
        }
    }
}

static void on_driver_reset (void)
{
    driver_reset();

    if(power_state != Power_On)
        check_power_restored();
}

static void no_ports (uint_fast16_t state)
{
    report_message("Motor supply monitor plugin failed to claim needed port!", Message_Warning);
}

// Wait for idle state (alarm cleared) before issuing command.
static void await_idle (sys_state_t state)
{
    if(state == STATE_IDLE)
        check_power_restored();
}

// Raise motor fault alarm.
static void raise_power_alarm (uint_fast16_t state)
{
    if(power_state == Power_Alarm)
        system_raise_alarm(Alarm_MotorFault);

    power_state = Power_Lost;
}

// Called when power sensing pin changes state.
// Raises alarm or sets up wait for alarm reset before taking action.
static void on_power_change (uint8_t port, bool state)
{
    if(!state) {
        if(power_state == Power_On) {
            protocol_enqueue_rt_command(raise_power_alarm);
            power_state = Power_Alarm;
        }
    } else if(on_state_change == NULL && power_state == Power_Lost) {
        // power is back on, wait for idle status
        on_state_change = grbl.on_state_change;
        grbl.on_state_change = await_idle;
    }
}

// Called when settings changes, we have to (re)register our interrupt handler.
static void on_settings_changed (settings_t *settings)
{
    settings_changed(settings);

    if(port != 0xFF)
        hal.port.register_interrupt_handler(port, IRQ_Mode_Change, on_power_change);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void power_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(power_settings_t), true) != NVS_TransferResult_OK)
        power_settings_restore();

    if(plugin_settings.port >= n_ports)
        plugin_settings.port = n_ports - 1;

    port = plugin_settings.port;

    xbar_t *portinfo = hal.port.get_pin_info(Port_Digital, Port_Input, port);

    if(portinfo && !portinfo->cap.claimed && (portinfo->cap.irq_mode & IRQ_Mode_Change) && ioport_claim(Port_Digital, Port_Input, &port, "Motor supply monitor")) {
        // (Re)register interrupt on settings change.
        settings_changed = hal.settings_changed;
        hal.settings_changed = on_settings_changed;
    } else {
        port = 0xFF;
        protocol_enqueue_rt_command(no_ports);
    }
}

// Settings descriptor used by the core when interacting with this plugin.
static setting_details_t setting_details = {
    .settings = power_settings,
    .n_settings = sizeof(power_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = power_settings_descr,
    .n_descriptions = sizeof(power_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = power_settings_save,
    .load = power_settings_load,
    .restore = power_settings_restore
};

// Add info about our plugin to the $I report.
static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Motor supply monitor v0.01]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Motor supply monitor plugin failed to initialize!", Message_Warning);
}

// A call my_plugin_init will be issued automatically at startup.
// There is no need to change any source code elsewhere.
void my_plugin_init (void)
{
    bool ok = (n_ports = ioports_available(Port_Digital, Port_Input));

    if(ok) {

        if(!ioport_can_claim_explicit()) {

            // Driver does not support explicit pin claiming, claim the highest numbered port instead.

            if((ok = (n_ports = hal.port.num_digital_in) > 0 && (nvs_address = nvs_alloc(sizeof(power_settings_t)))))
                plugin_settings.port = --hal.port.num_digital_in;

        } else if((ok = (n_ports = ioports_available(Port_Digital, Port_Input)) > 0 && (nvs_address = nvs_alloc(sizeof(power_settings_t)))))
            strcpy(max_port, uitoa(n_ports - 1));
    }

    // If enough free non volatile memory register our plugin with the core.
    if(ok) {
        // Register settings.
        settings_register(&setting_details);

        // Used for setting value validation.
        strcpy(max_port, uitoa(n_ports - 1));

        // Add our plugin to the $I options report.
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        driver_reset = hal.driver_reset;
        hal.driver_reset = on_driver_reset;
    } else
        protocol_enqueue_rt_command(warning_msg);
}
