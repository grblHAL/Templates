/*

  my_plugin.c - user defined plugin that blinks the LED on a STM32F411 Blackpill

  Part of grblHAL

  Public domain

*/

#include "driver.h"

static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;

// Add info about our plugin to the $I report.
static void on_report_my_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Blink LED v1.00]" ASCII_EOL);
}

static void blink_led (sys_state_t state)
{
    static bool led_on = false;
    static uint32_t ms = 0;

    if(hal.get_elapsed_ticks() >= ms) {
        ms = hal.get_elapsed_ticks() + 500; //ms
        led_on = !led_on;
        if(led_on)
            GPIOC->ODR |= GPIO_PIN_13;
        else
            GPIOC->ODR &= ~GPIO_PIN_13;
    }

    on_execute_realtime(state);
}

void my_plugin_init (void)
{
    // Add info about our plugin to the $I report.
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = on_report_my_options;

    // Add blink LED function to grblHAL foreground process
    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = blink_led;

    // Enable PC13 as output
    GPIO_InitTypeDef GPIO_InitStructure = {
        .Mode      = GPIO_MODE_OUTPUT_PP,
        .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
        .Pin       = GPIO_PIN_13,
    };
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}
