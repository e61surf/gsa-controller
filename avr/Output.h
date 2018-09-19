#ifndef OUTPUT_H
#define OUTPUT_H

#include <avr/io.h>
#include <stdlib.h>

/*
 * This class controls a single output pin (normally used to control a LED, but could also do horn, etc), and lets the program turn the pin on, off, or blinking (at a given frequency).  Init values include the 
 * port / pin.  Calling on or off does nothing immediately; rather, it marks the correct state for update in poll().
 *
 * Instances of this class should have their poll() function called repeatedly to control state.
 */
class Output {

private:
	//Config variables
	volatile uint8_t *port;
	uint8_t pin;

	//State variables
	uint16_t flash_time;			        //The accumulation of time (in millis) since the LED last flashed
	uint16_t flash_interval;			//The interval (in millis) at which the LED should flash.  Set to 0 to turn off, or 0xFFFF to turn on.

public:
	/*
	 * Creates a new instance to control status and blinking
	 */
	Output(volatile uint8_t *port, uint8_t pin);

	/*
	 * Checks the current state, and changes pins if needed.  The time argument is the elapsed time (in millis) since last poll.
	 */
	void poll(uint16_t elapsed_time);

	/*
		* Turns the output on (next time poll() is called)
		*/
	void on();

	/*
		* Turns the output on (next time poll() is called)
		*/
	void off();

	/*
		* Toggles the output at the specified interval
		*/
	void flash(uint16_t interval);
};

#endif
