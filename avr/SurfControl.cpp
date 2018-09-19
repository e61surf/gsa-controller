/*

This software is the property of E61 Incorporated. All use of any part of this software is prohibited without the express written consent of E61 Incorporated.

*/

#include <math.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/power.h>

#include "SurfControl.h"

#include <dcmath.h>
#include <FramedSerialProtocol.h>
#include <SerialAVR.h>
#include <SerialUSB.h>

#include "lib/analog/analog.h"
#include "lib/timer/timer.h"
#include "lib/serial/software_serial.h"

#include "Actuator.h"
#include "Buttons.h"
#include "Gps.h"
#include "Output.h"

//GPS wrapper uses Serial1
Gps gps;

using namespace digitalcave;

//Instantiate actuator control objects
Actuator actuatorPort(&MOTOR_PORT_PORT1, MOTOR_PORT_PIN1, &MOTOR_PORT_PORT2, MOTOR_PORT_PIN2, &MOTOR_PORT_PORTPWM, MOTOR_PORT_PINPWM, MOTOR_PORT_CS_ADC);
Actuator actuatorStarboard(&MOTOR_STARBOARD_PORT1, MOTOR_STARBOARD_PIN1, &MOTOR_STARBOARD_PORT2, MOTOR_STARBOARD_PIN2, &MOTOR_STARBOARD_PORTPWM, MOTOR_STARBOARD_PINPWM, MOTOR_STARBOARD_CS_ADC);

//If centerMotor is true, the actuator will be used.  Otherwise, the outputs will.
Actuator actuatorCenter(&MOTOR_CENTER_PORT1, MOTOR_CENTER_PIN1, &MOTOR_CENTER_PORT2, MOTOR_CENTER_PIN2, &MOTOR_CENTER_PORTPWM, MOTOR_CENTER_PINPWM, MOTOR_CENTER_CS_ADC);
Output horn(&MOTOR_CENTER_PORT1, MOTOR_CENTER_PIN1);
Output lights(&MOTOR_CENTER_PORT2, MOTOR_CENTER_PIN2);

//Instantiate LED control objects
Output ledStarboard(&LED_RIGHT_PORT, LED_RIGHT_PIN);
Output ledCenter(&LED_CENTER_PORT, LED_CENTER_PIN);
Output ledPort(&LED_LEFT_PORT, LED_LEFT_PIN);

//The three input buttons.	The constructor for this class takes care of input mode, pullups, etc.
Buttons buttons(&BUTTON1_PORT, BUTTON1_PIN, &BUTTON2_PORT, BUTTON2_PIN, &BUTTON3_PORT, BUTTON3_PIN);

//This cannot be a class variable, since it needs to be accessed by an ISR
SerialAVR serialAvr(115200, 8, 0, 1, 1, 64);		//BLE (UART1), default 115200 baud
#ifdef DEBUG
SerialUSB serialUSB;
#endif

FramedSerialProtocol protocol(64);
FramedSerialMessage request(0, 64);

//These are global config variables, and are loaded from EEPROM in setup()
uint8_t centerMotor = 0x00;
uint8_t demoMode = 0x00;
uint8_t wakeTransferDelay = 0x00;		//This value is in 10ths of a second.

char buf[60];

#ifdef DEBUG
void debug(const char* buf){
	//Send debug message via USB up to ending 0x00 byte
	if (serialUSB.isConnected()){
		for (uint8_t i = 0; i < 60; i++){
			if (buf[i] == 0){
				serialUSB.write((uint8_t) '\r');
				serialUSB.write((uint8_t) '\n');
				serialUSB.write((uint8_t*) buf, i);
				break;	//Stop once we reach the null end of string char
			}
		}
	}

	//Send debug messages to mobile
	FramedSerialMessage m(MSG_DEBUG, 64);
	for (uint8_t i = 0; i < 80; i++){
		m.append(buf[i]);
		if (buf[i] == 0){
			break;	//Stop once we reach the null end of string char
		}
	}
	protocol.write(&serialAvr, &m);
}
#endif

void setup() {
	//Disable watchdog timer
	wdt_disable();

	//Set clock to run at full speed
	clock_prescale_set(clock_div_1);

	// DISABLE JTAG - take control of F port
	MCUCR = _BV(JTD);
	MCUCR = _BV(JTD);

	//We write the magic number at the beginning of the eeprom as a
	// (very primitive) verification that we wrote this data, rather
	// than just being random.
	if (eeprom_read_byte((uint8_t*) EEPROM_OFFSET) == 0x42){
		centerMotor = eeprom_read_byte((uint8_t*) (EEPROM_OFFSET + 1));
		demoMode = eeprom_read_byte((uint8_t*) (EEPROM_OFFSET + 2));
		wakeTransferDelay = eeprom_read_byte((uint8_t*) (EEPROM_OFFSET + 3));
		uint16_t fullExtensionTime = 100 * eeprom_read_byte((uint8_t*) (EEPROM_OFFSET + 4));
		actuatorPort.set_full_extension_time(fullExtensionTime);
		actuatorStarboard.set_full_extension_time(fullExtensionTime);
		actuatorCenter.set_full_extension_time(fullExtensionTime);
	}

	//Add pullups on POT1 ADC pin.
	//Important: This statement assumes that the two pots are both on PORTF (where the ADC number
	// matches the pin number).  If this changes in a future hardware revision, be sure to
	// change this code too!
	PORTF |= _BV(POT1_ADC);
	PORTF |= _BV(POT1_ADC) | _BV(POT2_ADC);		//This would enable pullup for POT2 as well...

	bluetooth_init();

	if (!centerMotor){
		//Set PWM constantly high for horn / light output
		MOTOR_CENTER_PORTPWM |= _BV(MOTOR_CENTER_PINPWM);
	}

	//Flash all lights on solid for 4 seconds to indicate restart.  We also retract both motors during this period.
	ledStarboard.on();
	ledCenter.on();
	ledPort.on();
	ledStarboard.poll(0);
	ledCenter.poll(0);
	ledPort.poll(0);

	//Start retracting motors
	actuatorPort.set_manual_movement(-1);
	actuatorStarboard.set_manual_movement(-1);
	if (centerMotor){
		actuatorCenter.set_extension(100);  //Fully extend center tab
		actuatorCenter.poll(0);
	}
	actuatorPort.poll(0);
	actuatorStarboard.poll(0);

	//Init ADC pins with reference of AVCC (with external cap at AREF).
	uint8_t analog_pins[5];
	analog_pins[POT1_INDEX] = POT1_ADC;
	analog_pins[POT2_INDEX] = POT2_ADC;
	analog_pins[MOTOR_PORT_CS_INDEX] = MOTOR_PORT_CS_ADC;
	analog_pins[MOTOR_STARBOARD_CS_INDEX] = MOTOR_STARBOARD_CS_ADC;
	analog_pins[MOTOR_CENTER_CS_INDEX] = MOTOR_CENTER_CS_ADC;
	analog_init(analog_pins, 5, ANALOG_AVCC);

	software_serial_init(9600);		//GPS, NEMA mode

	_delay_ms(4000);

	//Stop retracting motors
	actuatorPort.set_manual_movement(0);
	actuatorStarboard.set_manual_movement(0);
	if (centerMotor){
		actuatorCenter.poll(4000);
	}
	actuatorPort.poll(4000);
	actuatorStarboard.poll(4000);

	timer_init();

	#ifdef DEBUG
	debug("Startup completed; enabling watchdog timer");
 	#endif
	wdt_enable(WDTO_1S);
}

void loop() {
	//Reset WDT
	wdt_reset();

	//***** State variables *****//

	//Current operation mode
	static uint8_t mode = MODE_READY;

	//Scheduled time to perform a wake transfer operation.
	static uint32_t transferTime = 0;


	//***** Time how long each iteration through the loop takes.	Various poll methods use the elapsed time to control delays. *****//

	static uint32_t last_time = 0;
	static uint32_t currentTime = 0;
	last_time = currentTime;
	currentTime = timer_millis();
	uint32_t elapsedTime = currentTime - last_time;

	//***** Look for new incoming messages from the bluetooth.
	uint8_t cmd = 0x00;
	if (protocol.read(&serialAvr, &request)){
		cmd = request.getCommand();
		#ifdef DEBUG
		snprintf(buf, sizeof(buf), "Message received: Command 0x%02X", cmd);
		debug(buf);
		#endif
	}


	//***** Poll hardware.	This includes starting / stopping actuators, toggling outputs, reading GPS, etc. *****//

	//Actuator movement polls
	actuatorPort.poll(elapsedTime);
	actuatorStarboard.poll(elapsedTime);

	if (centerMotor){
		actuatorCenter.poll(elapsedTime);
	}
	else {
		horn.poll(elapsedTime);
		lights.poll(elapsedTime);
	}

	//Button state + software debouncing
	buttons.poll(elapsedTime);

	//Output polls
	ledStarboard.poll(elapsedTime);
	ledCenter.poll(elapsedTime);
	ledPort.poll(elapsedTime);

	//GPS polls
	gps.poll(elapsedTime);

	static uint8_t pot1Percentage = 0;
	static uint8_t lastPot1Percentage = 0;
	//These are only used if centerMotor is true
	static uint8_t pot2Percentage = 0;
	static uint8_t lastPot2Percentage = 0;

	//Every ~200ms, read the potentiometers to find desired extension percentage (from 75% - 100% for POT1, and from 0% - 100% for POT2).
	static uint32_t lastAdcTime = 0;
	if (currentTime - lastAdcTime >= 200){
		// To combat noise, we read it 8 times and take the average.
		double totalPercentage = 0;
		for (uint8_t i = 0; i < 8; i++){
			totalPercentage += 75 + (analog_read_p(POT1_INDEX) / 40.92);
		}
		pot1Percentage = totalPercentage / 8;
		if (centerMotor){
			totalPercentage = 0;
			for (uint8_t i = 0; i < 8; i++){
				totalPercentage += analog_read_p(POT2_INDEX) / 10.23;
			}
			pot2Percentage = totalPercentage / 8;
		}

		uint16_t currentSense[3];
		currentSense[0] = analog_read_p(MOTOR_PORT_CS_INDEX);
		currentSense[1] = analog_read_p(MOTOR_STARBOARD_CS_INDEX);
		currentSense[2] = analog_read_p(MOTOR_CENTER_CS_INDEX);
		if (currentSense[0] > 0 || currentSense[1] > 0 || currentSense[2] > 0){
			FramedSerialMessage m(MSG_CURRENT_SENSE, 3);
			m.append((uint8_t) (currentSense[0] >> 2));
			m.append((uint8_t) (currentSense[1] >> 2));
			m.append((uint8_t) (currentSense[2] >> 2));
			protocol.write(&serialAvr, &m);
		}

		lastAdcTime = currentTime;
	}

	//Every ~2 seconds, send the mobile client the current program state.  Alternatively,
	// other logic can set lastStateTime to 0 to force it to send immediately (this is
	// done when we know the state has changed, and want to inform the mobile
	// client immediately).
	static uint32_t lastStateTime = 0;
	uint16_t statusTimeout = (mode == MODE_READY ? 2000 : 100);
	if (currentTime - lastStateTime >= statusTimeout){
		FramedSerialMessage m(MSG_STATE, 9);
		m.append(mode);
		m.append(actuatorPort.get_extension());
		m.append(actuatorStarboard.get_extension());
		m.append(centerMotor ? actuatorCenter.get_extension() : 0);
		m.append(pot1Percentage);
		m.append(pot2Percentage);
		m.append(gps.get_locked());
		m.append(gps.get_speed());
		if (transferTime < currentTime){
			m.append(0);
		}
		else {
			m.append((currentTime - transferTime) / 100);	//Send wake transfer time in 10ths of a second
		}
		protocol.write(&serialAvr, &m);
		lastStateTime = currentTime;
	}

	//***** Convert inputs + mode to action (state change definitions) *****//

	//Check the pressed and held state of all three buttons.	This will return non-zero values if a button has just been debounced.
	uint8_t released = buttons.rising_edge();
	uint8_t longpressed = buttons.longpressed();
	uint8_t held = buttons.held();

#ifdef DEBUG
	if (released || longpressed || held){
		snprintf(buf, sizeof(buf), "Release: %d; Long Press: %d; Held: %d", released, longpressed, held);
		debug(buf);
	}
#endif

	//Reset can happen in any mode at any time.
	if (released == _BV(BUTTON_RESET) || cmd == MSG_RESET) {
		#ifdef DEBUG
		debug("Reset");
		#endif
		actuatorPort.set_extension(0);
		actuatorStarboard.set_extension(0);
		if (centerMotor){
			if (pot2Percentage == 0) {
				actuatorCenter.set_extension(0);
				lastPot2Percentage = pot2Percentage;
			}
			else {
				actuatorCenter.set_extension(100);
			}
		}
		transferTime = 0;
		mode = MODE_AUTOMATIC_CHANGE;
	}
	//The mobile device can send MSG_STATE to request the current state be sent immediately.
	else if (cmd == MSG_STATE){
		lastStateTime = 0;
	}
	//The mobile client can ask for current config values at any time, and we immediately respond.
	else if (cmd == MSG_GET_CONFIG){
		FramedSerialMessage m(MSG_GET_CONFIG, 4);
		m.append(centerMotor);
		m.append(demoMode);
		m.append(wakeTransferDelay);
		m.append(actuatorPort.get_full_extension_time() / 100);	//We assume all actuators use the same value...
		protocol.write(&serialAvr, &m);
	}
	//Set config can happen in any mode at any time.
	else if (cmd == MSG_SET_CONFIG && request.getLength() == 4){
		#ifdef DEBUG
		debug("Set config and restart");
		#endif
		eeprom_update_byte((uint8_t*) (EEPROM_OFFSET + 1), request.getData()[0]);	//Center Motor
		eeprom_update_byte((uint8_t*) (EEPROM_OFFSET + 2), request.getData()[1]);	//Demo mode
		eeprom_update_byte((uint8_t*) (EEPROM_OFFSET + 3), request.getData()[2]);	//Wake transfer delay (10ths of a second)
		uint8_t fullExtensionTime = request.getData()[3];
		if (fullExtensionTime < 30) {
			fullExtensionTime = 30;
		}
		else if (fullExtensionTime > 80) {
			fullExtensionTime = 80;
		}
		eeprom_update_byte((uint8_t*) (EEPROM_OFFSET + 4), fullExtensionTime);	//Full extension time (10ths of a second, from 3 to 8 seconds)
		eeprom_update_byte((uint8_t*) EEPROM_OFFSET, 0x42);

		//After writing the data, start an infinite loop and wait for the watchdog timer to reboot.
		while (1){}
	}
	//These four manual adjustment modes can happen regardless of GPS lock
	else if (held == (_BV(BUTTON_SURF_LEFT) | _BV(BUTTON_RESET))){
		#ifdef DEBUG
		snprintf(buf, sizeof(buf), "Hold manual retract port; extension: %d", actuatorPort.get_extension());
		debug(buf);
		#endif

		mode = MODE_MANUAL_RETRACT_PORT;
		actuatorPort.set_manual_movement(-1);
	}
	else if (held == (_BV(BUTTON_SURF_RIGHT) | _BV(BUTTON_RESET))){
		#ifdef DEBUG
		snprintf(buf, sizeof(buf), "Hold manual retract starboard; extension: %d", actuatorStarboard.get_extension());
		debug(buf);
		#endif

		mode = MODE_MANUAL_RETRACT_STARBOARD;
		actuatorStarboard.set_manual_movement(-1);
	}
	else if (held == _BV(BUTTON_SURF_LEFT)){
		#ifdef DEBUG
		snprintf(buf, sizeof(buf), "Hold manual extend port; extension: %d", actuatorPort.get_extension());
		debug(buf);
		#endif

		mode = MODE_MANUAL_EXTEND_PORT;
		actuatorPort.set_manual_movement(1);
	}
	else if (held == _BV(BUTTON_SURF_RIGHT)){
		#ifdef DEBUG
		snprintf(buf, sizeof(buf), "Hold manual extend starboard; extension: %d", actuatorStarboard.get_extension());
		debug(buf);
		#endif

		mode = MODE_MANUAL_EXTEND_STARBOARD;
		actuatorStarboard.set_manual_movement(1);
	}

	//More manual commands which can happen regardless of GPS lock
	else if (cmd == MSG_MANUAL_RETRACT || cmd == MSG_MANUAL_EXTEND){
		if (request.getData()[0] == ACTUATOR_PORT){
			#ifdef DEBUG
			debug("Manual adjust port");
			#endif
			uint8_t extension = actuatorPort.get_extension();
			if (cmd == MSG_MANUAL_RETRACT) extension -= 5;
			else if (cmd == MSG_MANUAL_EXTEND) extension += 5;
			if (extension <= 0) extension = 0;
			else if (extension >= 100) extension = 100;
			actuatorPort.set_extension(extension);
		}
		else if (request.getData()[0] == ACTUATOR_STARBOARD){
			#ifdef DEBUG
			debug("Manual adjust starboard");
			#endif
			uint8_t extension = actuatorStarboard.get_extension();
			if (cmd == MSG_MANUAL_RETRACT) extension -= 5;
			else if (cmd == MSG_MANUAL_EXTEND) extension += 5;
			if (extension <= 0) extension = 0;
			else if (extension >= 100) extension = 100;
			actuatorStarboard.set_extension(extension);
		}
		else if (request.getData()[0] == ACTUATOR_CENTER){
			#ifdef DEBUG
			debug("Manual adjust center");
			#endif
			uint8_t extension = actuatorCenter.get_extension();
			if (cmd == MSG_MANUAL_RETRACT) extension -= 5;
			else if (cmd == MSG_MANUAL_EXTEND) extension += 5;
			if (extension <= 0) extension = 0;
			else if (extension >= 100) extension = 100;
			actuatorCenter.set_extension(extension);
		}
		mode = MODE_AUTOMATIC_CHANGE;
		lastStateTime = 0;		//Send new state to mobile
	}

	//All other modes require a GPS lock and no other concurrent changes
	else if (mode == MODE_READY && gps.get_locked()){
		//Perform wake transfer
		if (transferTime != 0 && transferTime < currentTime && ((actuatorStarboard.get_extension() > 0) ^ (actuatorPort.get_extension() > 0))){		//Execute a wake transfer if one is scheduled for now.
			#ifdef DEBUG
			debug("Wake transfer");
			#endif
			actuatorPort.set_extension(actuatorPort.get_extension() == 0 ? pot1Percentage : 0);
			actuatorStarboard.set_extension(actuatorStarboard.get_extension() == 0 ? pot1Percentage : 0);
			lastPot1Percentage = pot1Percentage;
			transferTime = 0;
			mode = MODE_AUTOMATIC_CHANGE;
		}

		//Initiate launch mode
		else if (cmd == MSG_LAUNCH || longpressed == (_BV(BUTTON_SURF_LEFT) | _BV(BUTTON_SURF_RIGHT))){		//Longpress both surf buttons or send MSG_LAUNCH to initiate launch mode
			#ifdef DEBUG
			debug("Launch mode");
			#endif
			actuatorPort.set_extension(100);
			actuatorStarboard.set_extension(100);
			mode = MODE_AUTOMATIC_CHANGE;
		}

		//Digital trimming of POT_1
		else if (cmd == MSG_POT && request.getData()[0] == POT_1 && ((actuatorStarboard.get_extension() > 0) ^ (actuatorPort.get_extension() > 0))){		//Set new absolute value for the currently deployed tab
			if (actuatorStarboard.get_extension() > 0){
				uint8_t extension = request.getData()[1];
				if (extension <= 70) extension = 70;
				else if (extension >= 100) extension = 100;
				#ifdef DEBUG
				snprintf(buf, sizeof(buf), "Set starboard tab to %d", extension);
				debug(buf);
				#endif
				actuatorStarboard.set_extension(extension);

			}
			else if (actuatorPort.get_extension() > 0){
				uint8_t extension = request.getData()[1];
				if (extension <= 70) extension = 70;
				else if (extension >= 100) extension = 100;
				#ifdef DEBUG
				snprintf(buf, sizeof(buf), "Set port tab to %d", extension);
				debug(buf);
				#endif
				actuatorPort.set_extension(extension);
			}
			mode = MODE_AUTOMATIC_CHANGE;
			lastStateTime = 0;		//Send new state to mobile
		}
		else if (cmd == MSG_POT_TRIM && request.getData()[0] == POT_1 && ((actuatorStarboard.get_extension() > 0) ^ (actuatorPort.get_extension() > 0))){	//Set new relative value for the currently deployed tab
			if (actuatorStarboard.get_extension() > 0){
				#ifdef DEBUG
				debug("Trim starboard tab");
				#endif
				uint8_t extension = actuatorStarboard.get_extension();
				if (request.getData()[1] == DIRECTION_DOWN) extension -= 3;
				else if (request.getData()[1] == DIRECTION_UP) extension += 3;
				if (extension <= 70) extension = 70;
				else if (extension >= 100) extension = 100;
				actuatorStarboard.set_extension(extension);
			}
			else if (actuatorPort.get_extension() > 0){
				#ifdef DEBUG
				debug("Trim port tab");
				#endif
				uint8_t extension = actuatorPort.get_extension();
				if (request.getData()[1] == DIRECTION_DOWN) extension -= 3;
				else if (request.getData()[1] == DIRECTION_UP) extension += 3;
				if (extension <= 70) extension = 70;
				else if (extension >= 100) extension = 100;
				actuatorPort.set_extension(extension);
			}
			mode = MODE_AUTOMATIC_CHANGE;
			lastStateTime = 0;		//Send new state to mobile
		}

		//Initial surf left / surf right
		else if ((released == _BV(BUTTON_SURF_LEFT) || cmd == MSG_SURF_LEFT) && actuatorPort.get_extension() == 0 && actuatorStarboard.get_extension() == 0){	//Starting from a reset, surf left
			#ifdef DEBUG
			debug("Surf Left");
			#endif
			actuatorStarboard.set_extension(pot1Percentage);
			actuatorPort.set_extension(0);
			lastPot1Percentage = pot1Percentage;
			mode = MODE_AUTOMATIC_CHANGE;
		}
		else if ((released == _BV(BUTTON_SURF_RIGHT) || cmd == MSG_SURF_RIGHT) && actuatorPort.get_extension() == 0 && actuatorStarboard.get_extension() == 0){	//Starting from a reset, surf right
			#ifdef DEBUG
			debug("Surf Right");
			#endif
			actuatorStarboard.set_extension(0);
			actuatorPort.set_extension(pot1Percentage);
			lastPot1Percentage = pot1Percentage;
			mode = MODE_AUTOMATIC_CHANGE;
		}

		//This section allows scheduling wake transfers
		else if ((released == _BV(BUTTON_SURF_LEFT) || cmd == MSG_SURF_LEFT) && actuatorPort.get_extension() > 0 && actuatorStarboard.get_extension() == 0){	//If we are currently surfing right and we ask for surf left, schedule a wake transfer.
			#ifdef DEBUG
			debug("Wake transfer (Surf Left)");
			#endif
			transferTime = currentTime + wakeTransferDelay * 100;
		}
		else if ((released == _BV(BUTTON_SURF_RIGHT) || cmd == MSG_SURF_RIGHT) && actuatorPort.get_extension() == 0 && actuatorStarboard.get_extension() > 0){	//If we are currently surfing left and we ask for a surf right, schedule a wake transfer.
			#ifdef DEBUG
			debug("Wake transfer (Surf Right)");
			#endif
			transferTime = currentTime + wakeTransferDelay * 100;
		}
		else if (cmd == MSG_SCHEDULE_WAKE_TRANSFER && ((actuatorStarboard.get_extension() > 0) ^ (actuatorPort.get_extension() > 0))){	//If we are currently surfing left or right (but not launch mode or reset) and we ask for a wake transfer, schedule one.
			#ifdef DEBUG
			debug("Wake transfer");
			#endif
			transferTime = currentTime + wakeTransferDelay * 100;
		}

		//Read physical potentiometer POT_1
		else if (fabs((int16_t) lastPot1Percentage - pot1Percentage) > 1.5){		//Check if the pot has moved more than +/- 1.5% from last setpoint
			if (actuatorPort.get_extension() != 0 && actuatorStarboard.get_extension() == 0){
				#ifdef DEBUG
				snprintf(buf, sizeof(buf), "Trim port tab to %d", pot1Percentage);
				debug(buf);
				#endif
				actuatorPort.set_extension(pot1Percentage);
				lastPot1Percentage = pot1Percentage;
				mode = MODE_AUTOMATIC_CHANGE;
			}
			else if (actuatorPort.get_extension() == 0 && actuatorStarboard.get_extension() != 0){
				#ifdef DEBUG
				snprintf(buf, sizeof(buf), "Trim starboard tab to %d", pot1Percentage);
				debug(buf);
				#endif
				actuatorStarboard.set_extension(pot1Percentage);
				lastPot1Percentage = pot1Percentage;
				mode = MODE_AUTOMATIC_CHANGE;
			}
			else {
				//Nothing is currently extended; just update the last analog value
				lastPot1Percentage = pot1Percentage;
			}
		}

		//Control center tab
		else if (centerMotor && pot2Percentage > 0 && actuatorPort.get_extension() > 0 && actuatorStarboard.get_extension() > 0 && gps.get_speed() < CENTER_EXTEND_SPEED && actuatorCenter.get_extension() < 100){
			#ifdef DEBUG
			debug("Extend center tab (Launch)");
			#endif
			actuatorCenter.set_extension(100);
			mode = MODE_AUTOMATIC_CHANGE;
		}
		else if (centerMotor && pot2Percentage > 0 && ((actuatorPort.get_extension() > 0) ^ (actuatorStarboard.get_extension() > 0)) && gps.get_speed() < CENTER_EXTEND_SPEED && actuatorCenter.get_extension() < 100){
			#ifdef DEBUG
			debug("Extend center tab (Surf)");
			#endif
			actuatorCenter.set_extension(100);
			mode = MODE_AUTOMATIC_CHANGE;
		}
		else if (centerMotor && cmd == MSG_POT && request.getData()[0] == POT_2 && ((actuatorPort.get_extension() > 0) ^ (actuatorStarboard.get_extension() > 0)) && gps.get_speed() >= CENTER_EXTEND_SPEED){
			#ifdef DEBUG
			debug("Trim center tab via mobile (Surf)");
			#endif
			uint8_t digitalPotPercentage = request.getData()[1];
			if (digitalPotPercentage > 100) digitalPotPercentage = 100;
			actuatorCenter.set_extension(digitalPotPercentage);
			mode = MODE_AUTOMATIC_CHANGE;
		}
		else if (centerMotor && pot2Percentage > 0 && (actuatorPort.get_extension() > 0 || actuatorStarboard.get_extension() > 0) && gps.get_speed() >= CENTER_EXTEND_SPEED && fabs((int16_t) lastPot2Percentage - pot2Percentage) > 2.5){
			#ifdef DEBUG
			debug("Trim center tab via POT2 (Surf)");
			#endif
			actuatorCenter.set_extension(pot2Percentage);
			lastPot2Percentage = pot2Percentage;
			mode = MODE_AUTOMATIC_CHANGE;
		}
		else if (centerMotor && pot2Percentage > 0 && actuatorPort.get_extension() == 0 && actuatorStarboard.get_extension() == 0 && gps.get_speed() < CENTER_EXTEND_SPEED && actuatorCenter.get_extension() < 100){
			#ifdef DEBUG
			debug("Extend center tab (Normal)");
			#endif
			actuatorCenter.set_extension(100);
			mode = MODE_AUTOMATIC_CHANGE;
		}
		else if (centerMotor && cmd == MSG_POT && request.getData()[0] == POT_2 && actuatorPort.get_extension() == 0 && actuatorStarboard.get_extension() == 0 && gps.get_speed() >= CENTER_EXTEND_SPEED){
			#ifdef DEBUG
			debug("Trim center tab via mobile (Normal)");
			#endif
			uint8_t digitalPotPercentage = request.getData()[1];
			if (digitalPotPercentage > 100) digitalPotPercentage = 100;
			actuatorCenter.set_extension(digitalPotPercentage);
			mode = MODE_AUTOMATIC_CHANGE;
		}
		else if (centerMotor && pot2Percentage > 0 && actuatorPort.get_extension() == 0 && actuatorStarboard.get_extension() == 0 && gps.get_speed() >= CENTER_EXTEND_SPEED && fabs((int16_t) lastPot2Percentage - pot2Percentage) > 2.5){
			#ifdef DEBUG
			debug("Trim center tab (Normal)");
			#endif
			actuatorCenter.set_extension(pot2Percentage);
			lastPot2Percentage = pot2Percentage;
			mode = MODE_AUTOMATIC_CHANGE;
		}
	}

	//Perform the automatic changes
	else if (mode == MODE_AUTOMATIC_CHANGE){
		//When the automatic action is finished, show extension and go back to start mode
		if (centerMotor){
			if (actuatorPort.get_outstanding_time() == 0 && actuatorStarboard.get_outstanding_time() == 0 && actuatorCenter.get_outstanding_time() == 0){
				#ifdef DEBUG
				snprintf(buf, sizeof(buf), "Port extension: %d; Starboard extension: %d; Center extension: %d", actuatorPort.get_extension(), actuatorStarboard.get_extension(), actuatorCenter.get_extension());
				debug(buf);
				#endif
				mode = MODE_READY;
				lastStateTime = 0;		//Send new state to mobile
			}
		}
		else {
			if (actuatorPort.get_outstanding_time() == 0 && actuatorStarboard.get_outstanding_time() == 0){
				#ifdef DEBUG
				snprintf(buf, sizeof(buf), "Port extension: %d; Starboard extension: %d", actuatorPort.get_extension(), actuatorStarboard.get_extension());
				debug(buf);
				#endif
				mode = MODE_READY;
				lastStateTime = 0;		//Send new state to mobile
			}
		}
	}

	//Perform the manual changes
	else if (mode == MODE_MANUAL_RETRACT_PORT){
		if (held != (_BV(BUTTON_SURF_LEFT) | _BV(BUTTON_RESET))){
			#ifdef DEBUG
			snprintf(buf, sizeof(buf), "Release manual retract port; extension: %d", actuatorPort.get_extension());
			debug(buf);
			#endif
			mode = MODE_READY;
			actuatorPort.set_manual_movement(0);
			lastStateTime = 0;		//Send new state to mobile
		}
	}
	else if (mode == MODE_MANUAL_RETRACT_STARBOARD){
		if (held != (_BV(BUTTON_SURF_RIGHT) | _BV(BUTTON_RESET))){
			#ifdef DEBUG
			snprintf(buf, sizeof(buf), "Release manual retract starboard; extension: %d", actuatorStarboard.get_extension());
			debug(buf);
			#endif
			mode = MODE_READY;
			actuatorStarboard.set_manual_movement(0);
			lastStateTime = 0;		//Send new state to mobile
		}
	}
	else if (mode == MODE_MANUAL_EXTEND_PORT){
		if (held != (_BV(BUTTON_SURF_LEFT))){
			#ifdef DEBUG
			snprintf(buf, sizeof(buf), "Release manual extend port; extension: %d", actuatorPort.get_extension());
			debug(buf);
			#endif
			mode = MODE_READY;
			actuatorPort.set_manual_movement(0);
			lastStateTime = 0;		//Send new state to mobile
		}
	}
	else if (mode == MODE_MANUAL_EXTEND_STARBOARD){
		if (held != (_BV(BUTTON_SURF_RIGHT))){
			#ifdef DEBUG
			snprintf(buf, sizeof(buf), "Release manual extend starboard; extension: %d", actuatorStarboard.get_extension());
			debug(buf);
			#endif
			mode = MODE_READY;
			actuatorStarboard.set_manual_movement(0);
			lastStateTime = 0;		//Send new state to mobile
		}
	}

	//Check GPS speed.	If we are going too fast, reset.
	if (gps.get_locked() && gps.get_speed() > MAX_SPEED){
		#ifdef DEBUG
		debug("GPS Overspeed Reset");
		#endif
		actuatorPort.set_extension(0);
		actuatorStarboard.set_extension(0);
		if (centerMotor){
			actuatorCenter.set_extension(pot2Percentage);
			lastPot2Percentage = pot2Percentage;
		}
		mode = MODE_AUTOMATIC_CHANGE;
		lastStateTime = 0;		//Send new state to mobile
	}

	//***** Update the outputs based on motion, position of actuators, and GPS state *****//

	if (actuatorPort.get_outstanding_time() == 0) {
		if (actuatorPort.get_extension() == 0) ledPort.off();	//No light for retracted
		else ledPort.on();		//Solid light for extended
	}
	else ledPort.flash(100);	//Flashing light for moving

	if (actuatorStarboard.get_outstanding_time() == 0) {
		if (actuatorStarboard.get_extension() == 0) ledStarboard.off();	//No light for retracted
		else ledStarboard.on();		//Solid light for extended
	}
	else ledStarboard.flash(100);	//Flashing light for moving


	if (actuatorPort.get_outstanding_time() < 0 || actuatorStarboard.get_outstanding_time() < 0) ledCenter.on();	//Retracting
	else if (actuatorPort.get_outstanding_time() > 0 || actuatorStarboard.get_outstanding_time() > 0) ledCenter.off();	//Extending
	else {
		if (gps.get_locked()) ledCenter.on();
		else ledCenter.flash(100);
	}

	if (!centerMotor){
		if (transferTime > 0){
			horn.flash(500);
			lights.flash(200);
		}
		else if (actuatorPort.get_outstanding_time() != 0 || actuatorStarboard.get_outstanding_time() != 0){
			lights.flash(100);
			horn.off();
		}
		else {
			horn.off();
			lights.off();
		}
	}

	//***** Main loop delay *****//

	//This delay should be large enough to swamp any variability in time needed to execute this function, but
	// low enough to not cause excessive delays.	Adjust as needed.
	_delay_ms(50);
}

void bluetooth_command(const char* command){
	//Handle the timing logic to send BLE commands.  We need about 10ms in between each
	// character, and 50ms at the end, so we have to send things in a horribly awkward way.  Boo.
	for (uint8_t i = 0; i < 80; i++){		//At most 80 chars; this is more than enough.
		if (command[i] == 0){
			serialAvr.write('\r');
			_delay_ms(10);
			break;	//Stop once we reach the null end of string char
		}
		else {
			serialAvr.write((uint8_t) command[i]);
			_delay_ms(10);
		}
	}
	_delay_ms(50);
}

void bluetooth_init(){
	//Initialize the bluetooth module to transparent serial mode.
	_delay_ms(250);

	//Enter command mode
	bluetooth_command("$$$");

	//Set the name to 'e61'
	bluetooth_command("S-,E61");

	//Set device info and UART transparent services
	bluetooth_command("SS,C0");

	//Connection parameters.
	bluetooth_command("ST,0064,0300,0002,0040");

	//Reset device to commit changes
	bluetooth_command("R,1");

	//Consume all responses
	while (serialAvr.read((uint8_t*) buf)){
		_delay_ms(50);
	}
}

ISR(USART1_RX_vect){
	serialAvr.isr();
}

// Properly handle software resets from the WDT
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void) {
	MCUSR = 0;
	wdt_disable();
	return;
}

int main(){
	setup();
	while(1){
		loop();
	}

	return 0;
}
