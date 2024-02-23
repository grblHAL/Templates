/*

  rgb.c - plugin for M150, Marlin style RGB command

  Part of grblHAL

  Public domain.

  Usage: M150 [B<intensity>] [I<pixel>] [K] [P<intensity>] [R<intensity>] [S0] [U<intensity>] [W<intensity>]

    B<intensity> - blue component, 0 - 255
    I<pixel>     - NeoPixel index, available if number of pixels > 1
    K            - keep unspecified values
    P<intensity> - brightness, 0 - 255
    S0           - write values to all LEDs in strip
    R<intensity> - red component, 0 - 255
    U<intensity> - red component, 0 - 255

  https://marlinfw.org/docs/gcode/M150.html

  $536 - length of strip.

*/

#include "driver.h"

#if N_AXIS > 4 || AXIS_REMAP_ABC2UVW || LATHE_UVW_OPTION
#error "RGB plugin is not supported in this configuration!"
#else

#include <math.h>
#include <string.h>

static bool is_neopixels;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == RGB_WriteLEDs
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t parameter_validate (float *value)
{
    status_code_t state = Status_OK;

    if(isnanf(*value) || !isintf(*value))
        state = Status_BadNumberFormat;

    if(*value < -0.0f || *value > 255.0f)
        state = Status_GcodeValueOutOfRange;

    return state;
}

static status_code_t mcode_validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case RGB_WriteLEDs:

            if(gc_block->words.b && (state = parameter_validate(&gc_block->values.b)) != Status_OK)
                return state;

            if(gc_block->words.r && (state = parameter_validate(&gc_block->values.r)) != Status_OK)
                return state;

            if(gc_block->words.u && (state = parameter_validate(&gc_block->values.u)) != Status_OK)
                return state;

            if(gc_block->words.w && (state = parameter_validate(&gc_block->values.w)) != Status_OK)
                return state;

            if(gc_block->words.p && is_neopixels && (state = parameter_validate(&gc_block->values.p)) != Status_OK)
                return state;

            if(!(gc_block->words.r || gc_block->words.u || gc_block->words.b || gc_block->words.w || gc_block->words.p))
                return Status_GcodeValueWordMissing;

            if(gc_block->words.i && hal.rgb.num_devices > 1) {
                if(gc_block->values.ijk[0] < -0.0f || gc_block->values.ijk[0] > (float)(hal.rgb.num_devices - 1))
                    state = Status_GcodeValueOutOfRange;
                else
                    gc_block->words.i = Off;
            }

            if(gc_block->words.p && is_neopixels)
                gc_block->words.p = Off;

            gc_block->words.k = gc_block->words.b = gc_block->words.r = gc_block->words.u = gc_block->words.w = gc_block->words.s = Off;
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    static rgb_color_t color = {0}; // TODO: allocate for all leds?

    bool handled = true;

    if(state != STATE_CHECK_MODE)
      switch(gc_block->user_mcode) {

         case RGB_WriteLEDs:;

             bool set_colors;
             uint16_t device = gc_block->words.i ? (uint16_t)gc_block->values.ijk[0] : 0;
             rgb_color_mask_t mask = { .value = 0xFF };
             rgb_color_t new_color;

             if((set_colors = gc_block->words.r || gc_block->words.u || gc_block->words.b || gc_block->words.w)) {
                 if(!gc_block->words.k)
                     color.value = 0;
                 else {
                     mask.R = gc_block->words.r;
                     mask.G = gc_block->words.u;
                     mask.B = gc_block->words.b;
                     mask.W = gc_block->words.w;
                 }
             }

             if(gc_block->words.w) {
                 if(hal.rgb.cap.W)
                     color.W = (uint8_t)(gc_block->words.p ? gc_block->values.p : gc_block->values.w);
                 else
                     color.R = color.G = color.B = gc_block->values.w;
             }

             if(!gc_block->words.w || hal.rgb.cap.W) {
                 if(gc_block->words.r)
                     color.R = (uint8_t)gc_block->values.r;
                 if(gc_block->words.u)
                     color.G = (uint8_t)gc_block->values.u;
                 if(gc_block->words.b)
                     color.B = (uint8_t)gc_block->values.b;
             }

             new_color.value = color.value;

             if(gc_block->words.p) {

                 if(hal.rgb.set_intensity)
                     hal.rgb.set_intensity((uint8_t)gc_block->values.p);
                 else
                     new_color = rgb_set_intensity(color, (uint8_t)gc_block->values.p);
             }

             if(set_colors || (gc_block->words.p && hal.rgb.set_intensity == NULL)) {
                 if(gc_block->words.s && hal.rgb.num_devices > 1) {
                     for(device = 0; device < hal.rgb.num_devices; device++) {
                         if(hal.rgb.out_masked)
                             hal.rgb.out_masked(device, new_color, mask);
                         else
                             hal.rgb.out(device, new_color);
                     }
                 } else {
                     if(hal.rgb.out_masked)
                         hal.rgb.out_masked(device, new_color, mask);
                     else
                         hal.rgb.out(device, new_color);
                 }
             }

             if(set_colors && hal.rgb.num_devices > 1 && hal.rgb.write)
                 hal.rgb.write();

             break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false;

    switch(setting->id) {

        case Setting_RGB_StripLengt0:
            available = !!hal.rgb.out;

        default:
            break;
    }

    return available;
}

static const setting_group_detail_t rgb_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t rgb_settings[] = {
    { Setting_RGB_StripLengt0, Group_AuxPorts, "Neopixel strip 1 length", NULL, Format_Int8, "##0", NULL, "255", Setting_NonCore, &settings.rgb_strip0_length, NULL, is_setting_available },
    { Setting_RGB_StripLengt1, Group_AuxPorts, "Neopixel strip 2 length", NULL, Format_Int8, "##0", NULL, "255", Setting_NonCore, &settings.rgb_strip1_length, NULL, is_setting_available },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t rgb_settings_descr[] = {
    { Setting_RGB_StripLengt0, "Number of LEDS in strip 1." },
    { Setting_RGB_StripLengt1, "Number of LEDS in strip 2." },
};

#endif

void rgb_setting_changed (settings_t *settings, settings_changed_flags_t changed)
{
    hal.settings_changed(settings, changed);
}

static setting_details_t setting_details = {
    .groups = rgb_groups,
    .n_groups = sizeof(rgb_groups) / sizeof(setting_group_detail_t),
    .settings = rgb_settings,
    .n_settings = sizeof(rgb_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = rgb_settings_descr,
    .n_descriptions = sizeof(rgb_settings_descr) / sizeof(setting_descr_t),
#endif
    .on_changed = rgb_setting_changed,
    .save = settings_write_global
};

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:RGB LED (M150) v0.02]" ASCII_EOL);
}

void my_plugin_init (void)
{
    if(hal.rgb.out) {

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = mcode_check;
        hal.user_mcode.validate = mcode_validate;
        hal.user_mcode.execute = mcode_execute;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        if((is_neopixels = hal.rgb.cap.R == 255 && hal.rgb.cap.G == 255 && hal.rgb.cap.B == 255))
            settings_register(&setting_details);
    }
}

#endif
