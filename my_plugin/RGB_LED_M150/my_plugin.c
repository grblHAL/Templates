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
    static rgb_color_t color = {0};

    bool handled = true;

    if(state != STATE_CHECK_MODE)
      switch(gc_block->user_mcode) {

         case RGB_WriteLEDs:;

             uint16_t device = gc_block->words.i ? (uint16_t)gc_block->values.ijk[0] : 0;
             rgb_color_mask_t mask = {0};

             if(!gc_block->words.k) {
                 mask.value = 0xFF;
                 memset(&color, 0, sizeof(rgb_color_t));
             } else {
                 mask.R = gc_block->words.r;
                 mask.G = gc_block->words.u;
                 mask.B = gc_block->words.b;
                 mask.W = gc_block->words.w;
             }

             if(gc_block->words.w) {
                 if(hal.rgb.cap.W)
                     color.W = (uint8_t)(gc_block->words.p ? gc_block->values.p : gc_block->values.w);
                 else
                     color.R = color.G = color.B = (uint8_t)(gc_block->words.p ? gc_block->values.p : gc_block->values.w);
             }

             if(!gc_block->words.w || hal.rgb.cap.W) {
                 if(gc_block->words.r)
                     color.R = (uint8_t)(gc_block->words.p ? gc_block->values.p : gc_block->values.r);
                 if(gc_block->words.u)
                     color.G = (uint8_t)(gc_block->words.p ? gc_block->values.p : gc_block->values.u);
                 if(gc_block->words.b)
                     color.B = (uint8_t)(gc_block->words.p ? gc_block->values.p : gc_block->values.b);
             }

             if(gc_block->words.s && hal.rgb.num_devices > 1) {
                 for(device = 0; device < hal.rgb.num_devices; device++) {
                     if(hal.rgb.out_masked)
                         hal.rgb.out_masked(device, color, mask);
                     else
                         hal.rgb.out(device, color);
                 }
             } else {
                 if(hal.rgb.out_masked)
                     hal.rgb.out_masked(device, color, mask);
                 else
                     hal.rgb.out(device, color);
             }

             if(hal.rgb.write)
                 hal.rgb.write();
             break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:RGB LED (M150) v0.01]" ASCII_EOL);
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

        is_neopixels = hal.rgb.cap.R == 255 && hal.rgb.cap.G == 255 && hal.rgb.cap.B == 255;
    }
}

#endif
