/*
  serial.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Template driver code for ARM processors

  Part of grblHAL

  By Terje Io, public domain

*/

#ifndef _HAL_SERIAL_H_
#define _HAL_SERIAL_H_

#include "grbl/hal.h"

const io_stream_t *serialInit (uint32_t baud_rate);

// If baud rate selection is not inplemented implement this signature instead:
// const io_stream_t *serialInit (void);

#endif
