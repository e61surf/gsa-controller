#ifndef BUTTONS_H
#define BUTTONS_H

#include <avr/io.h>
#include <stdlib.h>

/*
 * Button class which includes interactions between buttons (i.e. it is smart about things like pressing multiple buttons, etc)
 */
class Buttons {

//The time (in millis) required to register a button press / longpress.  Hold continually fires after longpress time.
#define PRESS_TIME        100
#define LONGPRESS_TIME    2000

//The indexes into the various storage arrays for each of these buttons
#define BUTTON_SURF_LEFT		0
#define BUTTON_RESET			1
#define BUTTON_SURF_RIGHT		2
//Count of all normal buttons
#define BUTTON_COUNT      3

//Count of all normal buttons plus the RF button
#define INPUT_COUNT       (BUTTON_COUNT)


private:
	//Config variables
	volatile uint8_t *ports[INPUT_COUNT];
	uint8_t pins[INPUT_COUNT];

	//State variables.  These are reset every poll cycle, and can be read by calling pressed() and held().
	uint8_t pressed_state = 0;
	uint8_t released_state = 0;
	uint8_t longpress_state = 0;

	//Start counting down when the button is pressed.  If it gets above PRESS_TIME without interruption it registers as a press (ditto for HOLD_TIME and hold)
	int16_t press_timer[INPUT_COUNT];

public:
	/*
	 * Init the buttons class with references to button1, 2, 3, and RF remote.
	 */
	Buttons(volatile uint8_t *port1, uint8_t pin1, volatile uint8_t *port2, uint8_t pin2, volatile uint8_t *port3, uint8_t pin3);

	/*
	 * Poll each of the buttons
	 */
	void poll(uint16_t elapsed_time);

	/*
	 * Returns the mask of buttons that are newly debounced and pressed.  This will return all buttons which have been pressed longer than PRESS_TIME and have not been fired since the last release.
	 */
	uint8_t falling_edge();

	/*
	 * Returns the mask of buttons that are newly debounced and released.  This will return all buttons which have been previously pressed longer than PRESS_TIME but shorter than LONGPRESS_TIME, but have since been released.
	 */
	uint8_t rising_edge();

	/*
	 * Returns the mask of buttons that are newly debounced and long pressed.  This will return all buttons which have been pressed longer than LONGPRESS_TIME and have not been fired since the last release.
	 */
	uint8_t longpressed();

	/*
		* Returns the mask of buttons that are currently debounced and held.  This will return all buttons which have been pressed longer than HOLD_TIME.
		*/
	uint8_t held();
};

#endif
