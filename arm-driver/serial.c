/*
  serial.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Template driver code for ARM processors

  Part of grblHAL

  By Terje Io, public domain

*/

#include "grbl/grbl.h"

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0}, rxbackup;

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    int16_t data;
    uint_fast16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    data = rxbuffer.data[bptr++];                   // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

    return data;
}

//
// Returns number of characters in serial input buffer
//
uint16_t serialRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - serialRxCount();
}

//
// Flushes the serial input buffer.
// NOTE: If the peripheral has an output FIFO that should be flushed as well.
//
static void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = false;
}
//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel (void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Writes a character to the serial output stream.
//
static bool serialPutC (const char c)
{
    uint_fast16_t next_head;

    // NOTE: If buffer and transmit register are empty buffering may be bypassed.
    //       See actual drivers for examples.

    next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);     // Get and update head pointer

    while(txbuffer.tail == next_head) {                         // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;                           // Add data to buffer
    txbuffer.head = next_head;                                  // and update head pointer

    UART_TX_IRQ_ENABLE();                                       // Enable TX interrupts

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

// ********************************************
// Optional functions, not required by the core
// ********************************************

// Some plugins will refuse to activate if not implemented.

//
// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
void serialWrite(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial2PutC(*ptr++);
}

//
// Suspend or reading from the input buffer or restore backup copy of it.
// Used by the manual tool change protocol.
//
static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

//
// Returns number of characters in the input buffer.
//
static inline uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters pending transmission.
//
uint16_t serialTxCount (void) {

    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) /* + remaining bytes in any FIFO and/or transmit register */;
}

//
// Flush the serial output buffer.
//
void serial2TxFlush (void)
{
    // Disable TX interrupts here.
    // Flush caracters in any transmit FIFO too.
    txbuf2.tail = txbuf2.head;
}

//
// Disable/enable reception.
//
static bool serialDisable (bool disable)
{
    if(disable)
// Disable RX interrupt.
    else
// Enable RX interrupt.

    return true;
}

//
// Sets the baud rate.
//
static bool serialSetBaudRate (uint32_t baud_rate)
{

    return true;
}

// **********************
// End optional functions
// **********************

//
// Configure serial peripheral
//
const io_stream_t *serialInit (uint32_t baud_rate)
{
    // Optional functions can be commented out, deleted or set to NULL if not implemented.
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .connected = true,
        .read = serialGetC,
        .write = serialWriteS,
        .write_all = serialWriteS,
        .get_rx_buffer_free = serialRxFree,
        .write_char = serialPutC,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .write_n = serialWrite,                 // Optional, required for Modbus plugin support.
        .get_rx_buffer_count = serialRxCount,   // Optional, required for Modbus plugin support.
        .get_tx_buffer_count = serialTxCount,   // Optional, required for Modbus plugin support.
        .reset_write_buffer = serialTxFlush,    // Optional, required for Modbus plugin support.
        .suspend_read = serialSuspendInput,     // Optional, required for manual tool change support.
        .disable = serialDisable,               // Optional, recommended for some plugins.
        .set_baud_rate = serialSetBaudRate      // Optional, required for Modbus and Bluetooth plugin support.
    };

    // Add code to initialize peripheral here, including enabling RX interrupts.

    return &stream;
}

static void uart_interrupt_handler (void)
{
    uint_fast16_t bptr;
    int32_t data;
    uint32_t iflags;

//    iflags = UART_GET_IRQSSTATE(); // Get UART interrupt flags.

    if(iflags & UART_IRQ_TX) {

        bptr = txbuffer.tail;

        if(txbuffer.head != bptr) {

            // UART_TX_WRITE(UARTCH, txbuffer.data[bptr++]);    // Put character in TXT register
            bptr &= (TX_BUFFER_SIZE - 1);                       // and update tmp tail pointer.

            txbuffer.tail = bptr;                               //  Update tail pointer.

            if(bptr == txbuffer.head)                           // Disable TX interrups
               // UART_TX_IRQ_DISABLE();                        // when TX buffer empty.
        }
    }

    if(iflags & (UART_IRQ_RX)) {

        bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer.

        if(bptr == rxbuffer.tail) {                         // If buffer full
            rxbuffer.overflow = 1;                          // flag overflow.
            // UART_RX_IRQ_DISABLE(); Clear RX interrupt, may be done by a dummy read of the RX register
        } else {
            // data = UART_GET(); Read received character to data varable, clear RX interrupt if not done automatically by read.
            if(data == CMD_TOOL_ACK && !rxbuffer.backup) {  // If tool change acknowledged
                stream_rx_backup(&rxbuf);                   // save current RX buffer     
                hal.stream.read = serialGetC;               // and restore normal input.
            } else if(!hal.stream.enqueue_realtime_command((char)data)) {
                rxbuffer.data[rxbuffer.head] = (char)data;  // Add data to buffer.
                rxbuffer.head = bptr;                       // and update pointer.
            }
        }
    }
}
