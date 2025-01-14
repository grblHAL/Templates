/*
  my_plugin.c - some MQTT fun

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "driver.h"

#if MQTT_ENABLE

#include <string.h>

#include "networking/networking.h"

static bool mqtt_connected = false;
static const char *client_id = NULL;
static coolant_set_state_ptr coolant_set_state_;
static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;
static on_program_completed_ptr on_program_completed;
static on_mqtt_client_connected_ptr on_client_connected;
static on_mqtt_message_received_ptr on_message_received;

static const char *msg_alarm = "Alarm %d! - %s";
static const char *msg_job_complete = "job completed!";
static const char *msg_coolant_on = "turn on water cooler!";
static const char *msg_coolant_off = "turn off water cooler!";

static void onStateChanged (sys_state_t state)
{
    static sys_state_t last_state = STATE_IDLE;

    if(state != last_state) {

        last_state = state;

        if((state & STATE_ALARM) && mqtt_connected) {

            char *msg;
            const char *alarm;
            if((msg = malloc(strlen((alarm = alarms_get_description(sys.alarm)) + strlen(msg_alarm) + 3)))) {
                sprintf(msg, msg_alarm, sys.alarm, alarm);
                mqtt_publish_message("grblHALxx", msg, strlen(msg), 1, false);
                free(msg);
            }
        }
    }

    if(on_state_change)
        on_state_change(state);
}

void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    if(!check_mode && mqtt_connected)
        mqtt_publish_message("grblHALxx", msg_job_complete, strlen(msg_job_complete), 1, false);

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void onCoolantSetState (coolant_state_t state)
{
    static coolant_state_t last_state = {0};

    coolant_set_state_(state);

    if(state.flood != last_state.flood && mqtt_connected) {
        if(state.flood)
            mqtt_publish_message("grblHALxx", msg_coolant_on, strlen(msg_coolant_on), 1, false);
        else
            mqtt_publish_message("grblHALxx", msg_coolant_off, strlen(msg_coolant_off), 1, false);
    }

    last_state = state;
}

static void onMQTTconnected (bool connected)
{
    if(on_client_connected)
        on_client_connected(connected);

    if((mqtt_connected = connected)) {
        mqtt_subscribe_topic("grblHAL", 1, NULL);
        client_id = networking_get_info()->mqtt_client_id;
    } else
        client_id = NULL;
}

static bool onMQTTmessage (const char *topic, const void *payload, size_t payload_length)
{
/*
    hal.stream.write(topic);
    hal.stream.write(ASCII_EOL);
    hal.stream.write((char *)payload);
    hal.stream.write(ASCII_EOL);
*/
    if(!strcmp((char *)payload, "stop job"))
        grbl.enqueue_realtime_command(CMD_STOP);

    return on_message_received == NULL || on_message_received(topic, payload, payload_length);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("MQTT Demo", "v0.01");
}

void my_plugin_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = onStateChanged;

    coolant_set_state_ = hal.coolant.set_state;
    hal.coolant.set_state = onCoolantSetState;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;

    on_client_connected = mqtt_events.on_client_connected;
    mqtt_events.on_client_connected = onMQTTconnected;

    on_message_received = mqtt_events.on_message_received;
    mqtt_events.on_message_received = onMQTTmessage;
}

#endif
