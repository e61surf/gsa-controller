# Surf System Controller - or whatever you want to control...
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/61ba1ccdf0874b5f88fdd2c0c29fdbb0)](https://www.codacy.com/app/david_78/gsa-controller?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=e61surf/gsa-controller&amp;utm_campaign=Badge_Grade)

Feel free to make use of the hardware and software designs here for your project. The board design utilizes:
3 ST Micro motor drivers,
1 Lynx GPS Module, and
1 Microchip BLE Module.

It is designed to fit into a Cinch ModeICE enclosure.

The hardware/software design supports up to a 10 wire, 5 volt control cable with 2 potentiometers, 3 momentary switches, and 3 LED indicators.

It can drive up to 3 12v motors/actuators.  Or one of the ST Micro chips can be configured as two 12v outputs for lights, horns, etc...

The board is designed to run on 12v power from a marine or automotive alternator, so it'll handle fluctuations and noise.

The BLE module and supporting AVR code supports a custom framed serial protocol for control via a mobile app.  See the documents for more info.

Contact david@e61.io for more information or to request changes or enhancements. 
