#ifndef GPS_H
#define GPS_H

#include <avr/io.h>
#include <stdlib.h>

#include "lib/serial/software_serial.h"
#include "lib/tinygps/TinyGPS.h"

//Time (in millis) without a message before the GPS signal is considered to be lost.
#define GPS_TIMEOUT        10000

//This is a global variable declared in SurfControl.cpp.  If it is true, then
// we work in demo mode, and don't actually read the GPS.
extern uint8_t demoMode;

/*
 * This class encapsulates state of a GPS module.  It uses the TinyGPS library to parse the GPS serial stream.  This class
 * tracks the time since the last received GPS signal (which indicates the 'locked' state), as well as last known speed, etc.
 *
 * Instances of this class should have their poll() function called repeatedly to update state.
 */
class Gps {

private:
	TinyGPS gps;                                    //The Tiny GPS object

	//State variables
	uint16_t message_time = 0xFFFF;	                //The accumulation of time (in millis) since a GPS message was received.  Max value (0xFFFF) means there has not been a GPS message received since GPS_TIMEOUT.
	uint8_t last_speed;                             //Last known speed.  If GPS is not locked, this may be inaccurate.

public:
	Gps();

	/*
	 * Checks the current state, and changes pins if needed.  The time argument is the current time (in millis).
	 */
	void poll(uint16_t elapsed_time);

	/*
	 * Returns 1 if locked, 0 otherwise
	 */
	uint8_t get_locked();

	/*
	 * Returns the last parsed speed.
	 */
	uint8_t get_speed();
};

#endif
