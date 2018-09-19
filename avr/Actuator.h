#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <avr/io.h>
#include <stdlib.h>

//The amount of time that it takes to start spinning up the motor.  This is a fudge factor to ensure that small movements are not under-represented in the total.
#define STARTUP_TIME		10

/*
 * This class controls a single linear actuator.  Init values include the 2 ports / pins used to control the H Bridge,
 * and the PWM port / pin used to control speed (although for now we just drive it solid high).  When the functions
 * are called to extend or retract, this class takes note of the expected runtime.  Movement (turning on or off) is
 * controlled in the poll() function; the movement functions only make note of the time.
 *
 * Instances of this class should have their poll() function called repeatedly to monitor movement.
 */
class Actuator {

private:
	//Config variables
	volatile uint8_t *port1;
	uint8_t pin1;
	volatile uint8_t *port2;
	uint8_t pin2;
	volatile uint8_t *portPwm;
	uint8_t pinPwm;
    uint8_t csAdc;
	int16_t fullExtensionTime;	//The time that it takes to go from full retraction to full extension (with a bit of overhead to compensate for resistance)

	//State variables

	//The sum of all extend times minus all retract times.  This will obviously drift
	// over time, but it will give a reasonably good estimate of total extension,
	// based on the percentage of FULL_EXTENSION_TIME.
	int32_t total_extension_time;

	int32_t extend_time;			//The time remaining to extend
	int32_t retract_time;			//The time remaining to retract
	int8_t manual_movement;                 //Manual movement is enabled.  Negative numbers mean retraction, positive numbers mean extension, zero means manual movement disabled.

public:
	/*
	 * Creates a new Actuator class to control extension and retraction times.
	 */
	Actuator(volatile uint8_t *port1, uint8_t pin1, volatile uint8_t *port2, uint8_t pin2, volatile uint8_t *portPwm, uint8_t pinPwm, uint8_t csAdc);

	/*
	 * Checks the current state, and changes pins if needed.  The poll length argument is the time (in millis) since poll() was last called.
	 * On each iteration through poll(), non-zero extend_time and retract_time are decremented by poll_length.  Once they reach zero,
	 * the pins are reset to turn movement off.
	 */
	void poll(uint16_t elapsed_time);

	/*
	 * Sets the actuator to extend to the given length (a percentage).  Set to 100 for full extension, 0 for full retraction, and 50 for
	 * half extension.  This value is translated to a run time and a direction, based on the current (estimated) extension combined with
	 * the time required for max extension (FULL_EXTENSION_TIME).
	 */
	void set_extension(int8_t percent);

	/*
	 * Returns the estimated position of the actuator, based on the total_extension_time variable combined with the max extension time.
	 * This is only as accurate as the actuator speed combined with the estimate of full extension time.  If the estimated position is
	 * less than 5%, return zero.
	 */
	uint8_t get_extension();

	/*
	 * Adjust the extension by the specified percentage.  This is shorthand for set_extension(get_extension() + percent).
	 */
	void adjust_extension(int8_t percent);

	/*
	 * Starts manual extension / retraction when movement is set to 1 / -1 respectively.  Set movement to 0 (or call set_extension) to stop
	 * manual movement.
	 */
	void set_manual_movement(int8_t movement);

	/*
	 * Returns the time remaining for the current movement.  Extend times are positive, retract times are negative.
	 */
	int32_t get_outstanding_time();

	//Get / set the full extension time, in milliseconds.  Must be between 3000 and 8000 (3 - 8 seconds).
	uint32_t get_full_extension_time();
	void set_full_extension_time(uint16_t full_extension_time);
};

#endif
