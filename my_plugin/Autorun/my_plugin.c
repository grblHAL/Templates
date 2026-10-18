/*
  my_plugin.c - Autorun file in local storage

  Runs file with name starting with "!" and ending with ".nc" on file system mount.

  Set $450=1 to enable autorun.
  Set $39=0 to allow ?, ! and ~ in $ commands to allow starting the file from a sender.

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "driver.h"

#include <string.h>

#include "grbl/task.h"
#include "grbl/vfs.h"
#include "grbl/nvs_buffer.h"

typedef struct {
    bool autorun;
} plugin_settings_t;

static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static plugin_settings_t my_settings;

static on_vfs_mount_ptr on_vfs_mount;
static on_report_options_ptr on_report_options;

static void autorun (void *path)
{
    vfs_dir_t *dir;

    if(my_settings.autorun && (dir = vfs_opendir((char *)path))) {

        char *tmp;
        vfs_dirent_t *dirent;

        // Scan files
        while(true) {

            if((dirent = vfs_readdir(dir)) == NULL || dirent->name[0] == '\0')
                break;
            if(vfs_errno)
                break;

            if(!dirent->st_mode.directory && *dirent->name == '!' && (tmp = strrchr(dirent->name, '.')) && !strcasecmp(tmp, ".nc")) {
                if((tmp = malloc(strlen((char *)path) + strlen(dirent->name) + 4))) {
                    strcat(strcat(strcpy(tmp, "$F="), (char *)path), dirent->name);
                    grbl.enqueue_gcode(tmp);
                    free(tmp);
                }
                break;
            }
        }
        vfs_closedir(dir);
    }
}

static void onMount (const char *path, const vfs_t *fs, vfs_st_mode_t mode)
{
    if(sys.driver_started)
        task_add_immediate(autorun, (void *)path);
    else
        task_run_on_startup(autorun, (void *)path);

    if(on_vfs_mount)
        on_vfs_mount(path, fs, mode);
}

static const setting_detail_t user_settings[] = {
    { Setting_UserDefined_0, Group_General, "Autorun", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &my_settings.autorun }
};

PROGMEM static const setting_descr_t my_settings_descr[] = {
    { Setting_UserDefined_0, "When SD card is inserted autorun file with name starting with \"!\" and ending with \".nc\"." }
};

static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);
}

static void plugin_settings_restore (void)
{
    my_settings.autorun = false;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);
}

static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&my_settings, nvs_address, sizeof(plugin_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Autorun", "0.01");
}

void my_plugin_init (void)
{
    static setting_details_t setting_details = {
       .settings = user_settings,
       .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
       .descriptions = my_settings_descr,
       .n_descriptions = sizeof(my_settings_descr) / sizeof(setting_descr_t),
       .save = plugin_settings_save,
       .load = plugin_settings_load,
       .restore = plugin_settings_restore
    };

    on_vfs_mount = vfs.on_mount;
    vfs.on_mount = onMount;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    if((nvs_address = nvs_alloc(sizeof(plugin_settings_t))))
        settings_register(&setting_details);
}
