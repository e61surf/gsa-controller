#ifndef SURF_CONTROL_H
#define SURF_CONTROL_H

//Comment out DEBUG definition to hide debug messages
#define DEBUG

//Define the location in EEPROM for the data storage
#define EEPROM_OFFSET	0x100

//Maximum speed (in kts) before automatic retraction
#define MAX_SPEED							13
//Maximum speed for center tab auto extension
#define CENTER_EXTEND_SPEED					6

/*********** INPUT ***********/

//The analog pins connected to the potentiometers
#define POT1_INDEX							0
#define POT1_ADC							1
#define POT2_INDEX							1
#define POT2_ADC							7

//Port / pin definitions for the three physical buttons
#define BUTTON1_PORT						PORTF
#define BUTTON1_PIN							0
#define BUTTON2_PORT						PORTB
#define BUTTON2_PIN							0
#define BUTTON3_PORT						PORTB
#define BUTTON3_PIN							1

/*********** OUTPUT ***********/

// Define Misc. Output Pins
//Left is LED3 in schematic
#define LED_LEFT_PORT						PORTB
#define LED_LEFT_PIN						7
//Center is LED2 in schematic
#define LED_CENTER_PORT						PORTB
#define LED_CENTER_PIN						3
//Right is LED1 in schematic
#define LED_RIGHT_PORT						PORTB
#define LED_RIGHT_PIN						2

// Define Motor Driver Pins
#define MOTOR_PORT_PORT1					PORTE
#define MOTOR_PORT_PIN1						2
#define MOTOR_PORT_PORT2					PORTC
#define MOTOR_PORT_PIN2						7
#define MOTOR_PORT_PORTPWM					PORTC
#define MOTOR_PORT_PINPWM					6
#define MOTOR_PORT_CS_INDEX					2
#define MOTOR_PORT_CS_ADC					4

#define MOTOR_STARBOARD_PORT1				PORTB
#define MOTOR_STARBOARD_PIN1				6
#define MOTOR_STARBOARD_PORT2				PORTB
#define MOTOR_STARBOARD_PIN2				5
#define MOTOR_STARBOARD_PORTPWM				PORTB
#define MOTOR_STARBOARD_PINPWM				4
#define MOTOR_STARBOARD_CS_INDEX			3
#define MOTOR_STARBOARD_CS_ADC				5

#define MOTOR_CENTER_PORT1					PORTD
#define MOTOR_CENTER_PIN1					7
#define MOTOR_CENTER_PORT2					PORTD
#define MOTOR_CENTER_PIN2					6
#define MOTOR_CENTER_PORTPWM				PORTD
#define MOTOR_CENTER_PINPWM					4
#define MOTOR_CENTER_CS_INDEX				4
#define MOTOR_CENTER_CS_ADC					6



//Available modes
#define MODE_READY							0
#define MODE_AUTOMATIC_CHANGE				1
#define MODE_MANUAL_RETRACT_PORT			2
#define MODE_MANUAL_RETRACT_STARBOARD		3
#define MODE_MANUAL_EXTEND_PORT				4
#define MODE_MANUAL_EXTEND_STARBOARD		5


//Available actuator values; can be used for messages with [Actuator] data bytes.
#define ACTUATOR_PORT						0x01
#define ACTUATOR_STARBOARD					0x02
#define ACTUATOR_CENTER						0x03

//Available directions; can be used for message with [Direction] data bytes
#define DIRECTION_DOWN						0x01
#define DIRECTION_UP						0x02

//Available potentiometers; can be used for message with [Pot Index] data bytes
#define POT_1								0x01
#define POT_2								0x02

//Available messages

//No data byte (command only)
//Use this message to reset the tabs to the fully retracted position.
#define MSG_RESET						0x01
//Use this message to put the tabs into launch mode (fully extended)
#define MSG_LAUNCH						0x02
//Use these messages to enable SURF_LEFT or SURF_RIGHT modes.  Must be sent
// from fully retracted mode.
#define MSG_SURF_LEFT					0x03
#define MSG_SURF_RIGHT					0x04
//One data byte: [Actuator]
//Use these messages to manually retract / extend the tabs.  Each time it is
// sent it adjusts by 5% up or down.
#define MSG_MANUAL_RETRACT				0x05
#define MSG_MANUAL_EXTEND				0x06
//Two data bytes: [Pot Index][Pot Value].  Use this message to set a new
// value for the digital potentiometer.  Values can be between 75 and 100
// for POT_1, and between 0 and 100 for POT_2.
#define MSG_POT							0x07
//One data byte: [Pot Index][Direction].  Use this message to perform a digital adjustment
// of the potentiometer relative to where it is now.  PotIndex is either POT_1 or POT_2.
// Direction is either DIRECTION_RETRACT or DIRECTION_EXTEND.
#define MSG_POT_TRIM					0x08
//No data.  The SCHEDULE_WAKE_TRANSFER message is only valid if currently in one of SURF_RIGHT / SURF_LEFT modes.
#define MSG_SCHEDULE_WAKE_TRANSFER		0x09
//X data bytes containing ASCII text.  This is sent from the MCU to the
// mobile device to communicate debug messages.
#define MSG_DEBUG						0x0A
//Data bytes: [Mode][PortActuatorValue][StarboardActuatorValue][CenterActuatorValue][Pot1Value][Pot2Value][GPS locked][GPSspeed][WakeTransferTimer]
// This is sent from the MCU to the mobile device, and contains all the program state, so that the mobile app can update the UI
// The mobile device can send this message (no need for any data bytes) to request that the MCU sends the MSG_SSTATE immediately.
#define MSG_STATE						0x0B
//Data bytes: [PortCS][StarboardCS][CenterCS]
// This is sent from the MCU to the mobile device to send the current sense values.  The
// current values are each 8 bit unsigned integers (the 8 MSB of the 10 bit analog values).
#define MSG_CURRENT_SENSE				0x0C
//The mobile client sends this to request current config values.  No data bytes.
//The MCU responds with the same message with current config values.  3 data bytes,
// same order and semantics as MSG_SET_CONFIG.
#define MSG_GET_CONFIG					0x0D
//Data bytes: [CenterMotor][DemoMode][WakeTransferDelay][FullExtensionTime]
//Write the CenterMotor and DemoMode config variables to EEPROM and reset the MCU.
// CenterMotor and DemoMode are boolean (1 = Enabled / True, 0 = Disabled / False).
// WakeTransferDelay is uint8_t, expressed in 10ths of a second (i.e. a value of 25 = 2.5 seconds)
// FullExtensionTime is uint8_t, expressed in 10ths of a second (i.e. 35 = 3.5 seconds).  This
// value must be between 30 and 80 (3.0 and 8.0 seconds).
#define MSG_SET_CONFIG					0x0E

//One data byte: [LegacyCommand] single data byte containing the legacy command
#define MSG_LEGACY						0xFF



//Used to keep the setup() method cleaner
void bluetooth_init();

#endif
