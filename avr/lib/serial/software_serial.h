#ifndef SOFTWARE_SERIAL_H
#define SOFTWARE_SERIAL_H

#include <avr/interrupt.h>
#include <avr/io.h>

//The buffer size; if separate buffer sizes are not defined for rx and tx, this one is
// used for both RX and TX buffers.  Defaults to 64 bytes.  You can change it by 
// redefining SOFTWARE_SERIAL_BUFFER_SIZE in your makefile (in the CDEFS variable,
// beside where F_CPU is defined).
#ifdef SOFTWARE_SERIAL_BUFFER_SIZE
#if SOFTWARE_SERIAL_BUFFER_SIZE > 255
#undef SOFTWARE_SERIAL_BUFFER_SIZE
#define SOFTWARE_SERIAL_BUFFER_SIZE 255
#endif
#else
#define SOFTWARE_SERIAL_BUFFER_SIZE 64
#endif

/*
 * (Re)Initializes the software serial state machine with the given parameters.  Valid arguments include:
 *  baud (tx only): 115200, 9600.  (Note that RX is hard coded to run at 9600)
 */
void software_serial_init(uint32_t baud);

/*
 * Reads a single character from the serial port.  Pass in a pointer to a byte, and
 * the function will write a single byte to that pointer.  If the read was successful,
 * return 1; otherwise return 0.  Implementations MAY block until a byte is received.
 */
uint8_t software_serial_read_b(uint8_t *b);

/*
 * Writes a byte array to the serial port.  Implementations MAY block until
 * all bytes are written.
 */
void software_serial_write_a(uint8_t *data, uint8_t len);

/*
 * Writes a single byte to the serial port.  Implementations MAY block until the 
 * write is completed.
 */
uint8_t software_serial_write_b(uint8_t data);

/*
 * Checks if any bytes are available for read.  Returns 0 when no bytes are available; 
 * returns non-zero when there are any bytes avilable.  Implementations MUST NOT block.
 * Implementations MAY return the total number of bytes available.
 */
uint8_t software_serial_available();

#endif
