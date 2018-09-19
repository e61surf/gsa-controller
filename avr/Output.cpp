#include "Output.h"

Output::Output(volatile uint8_t *port, uint8_t pin){
	//Store the port / pin values
	this->port = port;
	this->pin = pin;
		
	//Set to logic low
	*port &= ~_BV(pin);
	
	//Set DDR to output
	*(port - 0x1) |= _BV(pin);
}

void Output::poll(uint16_t elapsed_time){
	//Flash lights at the correct frequency, or turn on / off
	if (this->flash_interval == 0x0000){
		*this->port &= ~_BV(this->pin);	//Turn off output
		this->flash_time = 0;
	}
	else if (this->flash_interval == 0xFFFF){
		*this->port |= _BV(this->pin);	//Turn on output
		this->flash_time = 0;
	}
	else if (this->flash_time >= this->flash_interval){
		*this->port ^= _BV(this->pin);	//Toggle output
		this->flash_time = 0;
	}
	this->flash_time += elapsed_time;
}

void Output::on(){
	this->flash_interval = 0xFFFF;
}

void Output::off(){
	this->flash_interval = 0x0000; 
}

void Output::flash(uint16_t interval){
	this->flash_interval = interval; 
}
