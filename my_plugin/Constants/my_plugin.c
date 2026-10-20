/*
  my_plugin.c - named parameter constants

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "grbl/hal.h"
#include "grbl/ngc_params.h"

static on_report_options_ptr on_report_options;

static float _pi (void)
{
    return M_PI;
}

static float _raddeg (void)
{
    return RADDEG;
}

static float _m2i (void)
{
    return 25.4f;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Constants", "0.01");
}

void my_plugin_init (void)
{
    ngc_virtual_ro_param_add("_pi", _pi);
    ngc_virtual_ro_param_add("_raddeg", _raddeg);
    ngc_virtual_ro_param_add("_m2i", _m2i);

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}
