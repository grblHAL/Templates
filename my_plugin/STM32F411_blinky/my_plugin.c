/*

  my_plugin.c - user defined plugin that blinks the LED on a STM32F411 Blackpill

  Part of grblHAL

  Public domain

*/

#include "driver.h"
#include "grbl/task.h"

static on_report_options_ptr on_report_options;

// Add info about our plugin to the $I report.
static void on_report_my_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Blink LED v2.00]" ASCII_EOL);
}

static void blink_led (void *data)
{
    static bool led_on = false;

    if((led_on = !led_on))
        GPIOC->ODR |= GPIO_PIN_13;
    else
        GPIOC->ODR &= ~GPIO_PIN_13;

    // Reschedule the blink LED function
    task_add_delayed(blink_led, NULL, 500);
}

void my_plugin_init (void)
{
    // Add info about our plugin to the $I report.
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = on_report_my_options;

    // Schedule blink LED function to be run from the grblHAL foreground process
    task_add_delayed(blink_led, NULL, 500); // 500 ms

    // Enable PC13 as output
    GPIO_InitTypeDef GPIO_InitStructure = {
        .Mode      = GPIO_MODE_OUTPUT_PP,
        .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
        .Pin       = GPIO_PIN_13,
    };
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}
