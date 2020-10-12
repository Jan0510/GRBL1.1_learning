#ifndef serial_h
#define serial_h
#include "stdint.h"

#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 64
#endif

#define SERIAL_NO_DATA 0xff

void serial_init(void);

// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data);

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read(void);

// Reset and empty data in read buffer. Used by e-stop and reset.
void serial_reset_read_buffer(void);

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count(void);

// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count(void);

void printReceiveBufferCount(void);

#endif
