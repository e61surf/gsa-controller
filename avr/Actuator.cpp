#include "Actuator.h"

Actuator::Actuator(volatile uint8_t *port1, uint8_t pin1, volatile uint8_t *port2, uint8_t pin2, volatile uint8_t *portPwm, uint8_t pinPwm, uint8_t csAdc){
	//Store the port / pin values
	this->port1 = port1;
	this->pin1 = pin1;
	this->port2 = port2;
	this->pin2 = pin2;
	this->portPwm = portPwm;
	this->pinPwm = pinPwm;
	this->csAdc = csAdc;

	fullExtensionTime = 3500;

	//Set to logic low
	*port1 &= ~_BV(pin1);
	*port2 &= ~_BV(pin2);
	*portPwm &= ~_BV(pinPwm);

	//Set DDR to output for all three pins
	*(port1 - 0x1) |= _BV(pin1);
	*(port2 - 0x1) |= _BV(pin2);
	*(portPwm - 0x1) |= _BV(pinPwm);
}

void Actuator::poll(uint16_t elapsed_time){
	//If both extend and retract are finished, and manual movement is not enabled, then turn off all the pins
	if (extend_time <= 0 && retract_time <= 0 && manual_movement == 0){
		*port1 &= ~_BV(pin1);
		*port2 &= ~_BV(pin2);
		*portPwm |= _BV(pinPwm);

		//Reset the timers to zero.	This will be a nop EXCEPT when poll is called for the first time
		// after the timers going negative (signalling completion).
		extend_time = 0;
		retract_time = 0;
	}
	else if (extend_time > 0 || manual_movement > 0){
		//Set pin1 low, pin2 high, and pwm high
		*port1 &= ~_BV(pin1);
		*port2 |= _BV(pin2);
		*portPwm |= _BV(pinPwm);

		if (manual_movement == 0) extend_time -= elapsed_time;
		total_extension_time += elapsed_time;
	}
	else if (retract_time > 0 || manual_movement < 0){
		//Set pin1 high, pin2 low, and pwm high
		*port1 |= _BV(pin1);
		*port2 &= ~_BV(pin2);
		*portPwm |= _BV(pinPwm);

		if (manual_movement == 0) retract_time -= elapsed_time;
		total_extension_time -= elapsed_time;
	}

	//If we have tried to overrun the extension, then limit it to the range 0..fullExtensionTime.	This will (somewhat) allow for recalibration of
	// the setpoint over time, assuming that we try to reach max or min extension during operation.
	if (total_extension_time <= 0) total_extension_time = 0;
	else if (total_extension_time >= fullExtensionTime) total_extension_time = fullExtensionTime;
}

void Actuator::set_extension(int8_t percent){
	if (percent > 100) percent = 100;
	else if (percent < 0) percent = 0;

	manual_movement = 0;	//Stop any ongoing manual movement

	uint8_t currently_moving = 0;
	if (extend_time != 0 || retract_time != 0) currently_moving = 1;

	int32_t target_time = ((int32_t) fullExtensionTime * percent) / 100;
	if (target_time < total_extension_time){
		retract_time = total_extension_time - target_time;
		extend_time = 0;
	}
	else {
		extend_time = target_time - total_extension_time;
		retract_time = 0;
	}

	//If we are just starting the movement, add an extra bit of time to compensate for motor spin-up.
	if (!currently_moving){
		if (extend_time > 0) total_extension_time -= STARTUP_TIME;
		else if (retract_time > 0) total_extension_time += STARTUP_TIME;
	}

	//Throw some extra time to extend / retract if we are going right to the end.  This can help
	// to allow users to re-set the accumulated error over time.
	if (extend_time > 0 && percent == 100) extend_time += 2500;
	else if (retract_time > 0 && percent == 0) retract_time += 2500;
}

void Actuator::set_manual_movement(int8_t movement){
	manual_movement = movement;
}

uint8_t Actuator::get_extension(){
	int16_t result = (total_extension_time * 100) / fullExtensionTime;
	if (result < 5) return 0;
	if (result > 98) return 100;
	return result;
}

void Actuator::adjust_extension(int8_t percent){
	set_extension(get_extension() + percent);
}

int32_t Actuator::get_outstanding_time(){
	if (extend_time > 0) return extend_time;
	else if (retract_time > 0) return retract_time * -1;
	else return 0;
}

uint32_t Actuator::get_full_extension_time(){
	return fullExtensionTime;
}
void Actuator::set_full_extension_time(uint16_t newFullExtensionTime){
	if (newFullExtensionTime < 3000) {
		newFullExtensionTime = 3000;
	}
	else if (newFullExtensionTime > 8000){
		newFullExtensionTime = 8000;
	}

	fullExtensionTime = newFullExtensionTime;
}
