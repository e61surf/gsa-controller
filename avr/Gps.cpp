#include "Gps.h"

Gps::Gps(){
	//Be sure to initialize serial in main...
}

void Gps::poll(uint16_t elapsed_time){
	if (!demoMode){
		//Keep track of how long it has been since a message was last received.
		if (this->message_time != 0xFFFF) this->message_time += elapsed_time;

		//Check for new GPS messages on the software serial port
		while (software_serial_available()){
			uint8_t data;
			uint8_t result = software_serial_read_b(&data);		// load the data into a variable...
			if (result && gps.encode(data)){	// if there is a new valid sentence...
				this->last_speed = gps.speed() / 100;

				this->message_time = 0;
			}
		}

		if (this->message_time > GPS_TIMEOUT){
			this->message_time = 0xFFFF;
		}
	}
}

uint8_t Gps::get_locked(){
	if (demoMode){
		return 1;		//In demo mode we are always locked.
	}
	else {
		return this->message_time == 0xFFFF ? 0 : 1;
	}
}

uint8_t Gps::get_speed(){
	if (demoMode){
		return 9;		//In demo mode we always go 9 knots
	}
	else {
		return this->last_speed;
	}
}
