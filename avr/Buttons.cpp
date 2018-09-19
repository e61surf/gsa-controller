#include "Buttons.h"
#include <util/delay.h>

Buttons::Buttons(volatile uint8_t *port1, uint8_t pin1, volatile uint8_t *port2, uint8_t pin2, volatile uint8_t *port3, uint8_t pin3){
	//Store the button mappings
	this->ports[BUTTON_SURF_LEFT] = port1;
	this->pins[BUTTON_SURF_LEFT] = pin1;
	this->ports[BUTTON_RESET] = port2;
	this->pins[BUTTON_RESET] = pin2;
	this->ports[BUTTON_SURF_RIGHT] = port3;
	this->pins[BUTTON_SURF_RIGHT] = pin3;

	for (uint8_t i = 0; i < INPUT_COUNT; i++){
		//Set DDR to input (it should default to this, but let's be sure)
		*(this->ports[i] - 0x1) &= ~_BV(this->pins[i]);
	}
	for (uint8_t i = 0; i < BUTTON_COUNT; i++){
		//Set pullup resistors to prevent floating inputs
		*this->ports[i] |= _BV(this->pins[i]);
	}
}

void Buttons::poll(uint16_t elapsed_time) {
	this->released_state = 0;
	this->pressed_state = 0;
	this->longpress_state = 0;

	//Read all normal buttons.	If they read low (i.e. pressed) then increment the press timer, otherwise reset it.
	for (uint8_t i = 0; i < BUTTON_COUNT; i++){
		if (*(this->ports[i] - 0x2) & _BV(this->pins[i])){
			//If this button press exceeds press time but is not over long press time, and it is now released, then
			// mark it as rising edge.
			if (this->press_timer[i] >= PRESS_TIME && this->press_timer[i] < LONGPRESS_TIME){
				this->released_state |= _BV(i);
			}
			this->press_timer[i] = 0;
		}
		else {
			if (this->press_timer[i] < PRESS_TIME && this->press_timer[i] + elapsed_time >= PRESS_TIME){
				this->pressed_state |= _BV(i);
			}
			else if (this->press_timer[i] < LONGPRESS_TIME && this->press_timer[i] + elapsed_time >= LONGPRESS_TIME){
				this->longpress_state |= _BV(i);
			}
			this->press_timer[i] += elapsed_time;
		}
	}

	//Check for interaction between normal buttons.	If one button has started to be pressed and another one has already
	// been pressed, reset the higher of the two buttons to the timer state of the lower one.	This will ensure that the
	// press events fire at the same time.
	int16_t min_press_time = 0x7FFF;
	for (uint8_t i = 0; i < BUTTON_COUNT; i++){
		if (this->press_timer[i] > 0 && this->press_timer[i] < min_press_time) min_press_time = this->press_timer[i];
	}
	if (min_press_time < 0x7FFF){
		for (uint8_t i = 0; i < BUTTON_COUNT; i++){
			if (this->press_timer[i] > 0) this->press_timer[i] = min_press_time;
		}
	}
}

uint8_t Buttons::falling_edge() {
	return this->pressed_state;
}

uint8_t Buttons::rising_edge() {
	return this->released_state;
}

uint8_t Buttons::longpressed() {
	return this->longpress_state;
}

uint8_t Buttons::held() {
	uint8_t result = 0x00;
	for (uint8_t i = 0; i < INPUT_COUNT; i++){
		if (this->press_timer[i] > LONGPRESS_TIME) result |= _BV(i);
	}
	return result;
}
