# Surf System Controller - or whatever you want to control...

Feel free to make use of the hardware and software designs here for your project. It is ideally suited for robotics, small electric mobility decives, linear actuator control, or basically anything that needs to drive somewhat high-current motors and needs GPS and Bluetooth. The Atmel 32u4 based, high current board design utilizes:
3 ST Micro motor drivers,
1 Lynx GPS Module, and
1 Microchip BLE Module.

It is designed to fit into a Cinch ModeICE enclosure, which is fully waterproof.

The hardware/software design supports up to a 10 wire, 5 volt control cable and is currently utlizing 2 potentiometers, 3 momentary switches, and 3 LED indicators, but the Atmel 32u4 processor pins can be utlized for many different perposes. 

It can drive up to 3 12v motors/actuators.  Or one of the ST Micro chips can be configured as two 12v outputs for lights, horns, etc...

The board is designed to run on 12v power from a marine or automotive alternator, so it'll handle fluctuations and noise typical in rugged applications.

The BLE module and supporting AVR code supports a custom framed serial protocol for control via a mobile app.  See the documents for more info.
