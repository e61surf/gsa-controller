/*
 * An implementation of software serial, loosely based on AVR304 application note.
 * The transmit code here is blocking and delay based, and can run at 9600 or 115200 baud;
 * the receive code is interrupt / timer based and is hard coded to 9600 baud.
 */

#include <util/atomic.h>

#include "software_serial.h"
#include <ArrayStream.h>

using namespace digitalcave;

static ArrayStream rx_buffer(SOFTWARE_SERIAL_BUFFER_SIZE);

static uint32_t baud;
static uint8_t delayCycles;
static uint8_t delayCyclesLong;

static volatile uint8_t state = 0;
static volatile uint8_t rxByte = 0;
static volatile uint8_t rxBitCounter = 0;

//Number of ticks between bits and between 1.5 bits (9600 baud)
// These values are different than what AVR304 says.  Shrug?
#define CLOCK_TICKS_BETWEEN_BITS		205
#define CLOCK_TICKS_BETWEEN_1_5_BITS	254

#define STATE_IDLE			0
#define STATE_RX			1

#define TX_PORT		PORTD
#define TX_DDR		DDRD
#define TX_NUM		PORTD0

#define RX_PORT		PORTD
#define RX_DDR		DDRD
#define RX_PIN		PIND
#define RX_NUM		PORTD1

#define disableInt1()			( EIMSK &= ~_BV(INT1) )
#define enableInt1()			( EIMSK |= _BV(INT1) )

#define disableTimer0Int()		( TIMSK0 &= ~_BV(OCIE0A) )
#define enableTimer0Int()		( TIMSK0 |= _BV(OCIE0A) )
#define clearTimer0Int()		( TIFR0 |= _BV(OCF0A) )


void software_serial_init(uint32_t b){
	baud = b;

	state = STATE_IDLE;

	//We require a 16MHz clock for these values to work.
	switch(baud){
// 		case 19200:
// 			delayCycles = 3;
// 			delayCyclesLong = 93;
// 			break;
// 		case 38400:
// 			delayCycles = 5;
// 			delayCyclesLong = 40;
// 			break;
// 		case 57600:
// 			delayCycles = 2;
// 			delayCyclesLong = 24;
// 			break;
		case 115200:
			delayCycles = 13;
			delayCyclesLong = 0;
			break;
		default:
			//9600 baud is default
			delayCycles = 0;
			delayCyclesLong = 199;
	}

	//TX pin is in output mode, high state
	TX_DDR |= _BV(TX_NUM);
	TX_PORT |= _BV(TX_NUM);

	//RX pin is in input mode, with external interrupt 1 enabled
	RX_DDR &= ~_BV(RX_NUM);
	EICRA = _BV(ISC11);		//Falling edge of INT1 generates an interrupt
	enableInt1();			//Enable INT1 interrupts

}

static inline void software_serial_delay(){
	if (delayCycles){
		for (uint8_t delay = 0; delay < delayCycles; delay++){
			asm volatile ("nop"::);
		}
	}
	if (delayCyclesLong){
		for (uint8_t delay = 0; delay < delayCyclesLong; delay++){
			asm volatile ("nop"::);
			asm volatile ("nop"::);
			asm volatile ("nop"::);
			asm volatile ("nop"::);
		}
	}
}

uint8_t software_serial_write_b(uint8_t data){
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		//Start bit
		TX_PORT &= ~_BV(TX_NUM);
		software_serial_delay();

		for (uint8_t i = 0; i < 8; i++){
			if (data & 0x01){
				TX_PORT |= _BV(TX_NUM);
			}
			else {
				TX_PORT &= ~_BV(TX_NUM);
			}
			data = data >> 1;
			software_serial_delay();
		}

		//Stop bit
		TX_PORT |= _BV(TX_NUM);
		software_serial_delay();
	}

	return 1;
}

void software_serial_write_a(uint8_t *data, uint8_t len){
	for (uint8_t i = 0; i < len; i++){
		software_serial_write_b(data[i]);
	}
}

uint8_t software_serial_read_b(uint8_t *b){
	if (!rx_buffer.isEmpty()){
		rx_buffer.read(b);
		return 1;
	}
	return 0;
}

uint8_t software_serial_available() {
	return !rx_buffer.isEmpty();
}

//Listen for start bit.
ISR(INT1_vect){
	state = STATE_RX;							// We saw the falling line of the start bit
	disableInt1();								// Disable int1 interrupts for now

	disableTimer0Int();							// Disable timer0 interrupts for now

	TCCR0B &= ~_BV(CS01);						// Reset prescaler counter (disable clock).

	TCNT0 = 0;									// Clear counter register, plus add on constant value of how long it takes to execute ISR (9)

	TCCR0A = _BV(WGM01);						// CTC Mode
	TCCR0B = _BV(CS01);							// Start prescaler clock (div 8)

	OCR0A = CLOCK_TICKS_BETWEEN_1_5_BITS;		// Count one and a half period into the future.

	rxBitCounter = 0;							// Clear received bit counter.
	clearTimer0Int();							// Clear timer interrupt bit

	enableTimer0Int();

}

ISR(TIMER0_COMPA_vect){
	switch (state) {
		//Receive Byte.
		case STATE_RX:
			OCR0A = CLOCK_TICKS_BETWEEN_BITS;		// Count one period after the falling edge is trigged.
			//Receiving, LSB first.
			if( rxBitCounter < 8 ) {
				rxBitCounter++;
				rxByte = (rxByte >> 1);				// Shift due to receiving LSB first.
				if (RX_PIN & _BV(RX_NUM)) {
					rxByte |= 0x80;					// If a logical 1 is read, let the data mirror this.
				}
			}

			//Done receiving
			else {
				state = STATE_IDLE;					// We are finished...
				if (!rx_buffer.isFull()){
					rx_buffer.write(rxByte);			//Buffer if there is space
				}
				disableTimer0Int();					// Disable this interrupt.
				EIFR |= _BV(INTF1);					// Reset flag not to enter the ISR one extra time.
				enableInt1();						// Enable interrupt to receive more bytes.
			}
			break;

		// Unknown state.
		default:
			state = STATE_IDLE;							// Error, should not occur. Going to a safe state.
	}
}
