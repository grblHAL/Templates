/*
  my_plugin.c - embedded files example

  Adds two P65 macros to the "/embedded" VFS mount (directory).

  Run with G65P400 or G65P401

  Part of grblHAL

  Public domain.
  This code is is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "grbl/grbl.h"
#include "grbl/fs_embedded.h"

PROGMEM static const embedded_file_t P400_macro = {
    .name = "P400.macro",
    .size = 40,
    .data = { 40, 112, 114, 105, 110, 116, 44, 32, 73, 32, 97, 109, 32, 80, 52, 48,
              48, 44, 32, 97, 110, 32, 101, 109, 98, 101, 100, 100, 101, 100, 32, 109,
              97, 99, 114, 111, 33, 41, 13, 10 }
};

PROGMEM static const embedded_file_t P401_macro = {
    .name = "P401.macro",
    .size = 45,
    .data = { 40, 112, 114, 105, 110, 116, 44, 32, 73, 32, 97, 109, 32, 80, 52, 48,
              49, 44, 32, 97, 110, 111, 116, 104, 101, 114, 32, 101, 109, 98, 101, 100,
              100, 101, 100, 32, 109, 97, 99, 114, 111, 33, 41, 13, 10 }
};

void my_plugin_init (void)
{
    // Array of pointers to files, NULL terminated
    static const embedded_file_t *files[] = {
        &P400_macro,
        &P401_macro,
        NULL
    };

    fs_embedded_mount(files);
}
