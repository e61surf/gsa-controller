# SurfControl Firmware API

This document describes the public interfaces exposed by the SurfControl AVR firmware. It covers the application entry points, messaging contracts, hardware abstraction classes, and supporting utility libraries. Each section calls out responsibilities, configuration options, and example usage patterns.

## Quick Start
- **Initialization**: Call `setup()` once after reset to configure hardware, load persisted settings, and initialize communications.
- **Main loop**: Repeatedly call `loop()`; it orchestrates input sampling, actuator control, messaging, and safety logic.
- **Polling**: Every hardware abstraction (`Actuator`, `Buttons`, `Output`, `Gps`) requires periodic `poll()` calls with the elapsed milliseconds since the previous invocation.
- **Serial ISR**: Provide an `ISR(USART1_RX_vect)` that calls `serialAvr.isr()` so UART1 receive events reach the BLE link.
- **Watchdog**: The firmware expects the watchdog timer to run; `loop()` regularly resets it.

```c++
#include "SurfControl.h"

int main() {
  setup();
  while (true) {
    loop();
  }
}
```

## Application-Level API (`SurfControl.h`)

### System Configuration Constants
- **EEPROM layout**: `EEPROM_OFFSET` defines the starting address for persisted configuration; offsets 1-4 store center-motor enable, demo mode, wake-transfer delay, and actuator extension time.
- **Speed thresholds**: `MAX_SPEED` and `CENTER_EXTEND_SPEED` gate automatic tab movements based on GPS speed.
- **I/O wiring**: `BUTTON*`, `LED_*`, and `MOTOR_*` macros map AVR ports/pins to buttons, LEDs, and H-bridge channels.

### Operating Modes
- `MODE_READY` — Idle state awaiting input.
- `MODE_AUTOMATIC_CHANGE` — Automatic transitions (launch/reset/auto-trim) in progress.
- `MODE_MANUAL_RETRACT_PORT` / `MODE_MANUAL_RETRACT_STARBOARD` — Manual retraction while corresponding button combination is held.
- `MODE_MANUAL_EXTEND_PORT` / `MODE_MANUAL_EXTEND_STARBOARD` — Manual extension while button is held.

### Actuator, Direction, and Potentiometer Identifiers
- `ACTUATOR_PORT`, `ACTUATOR_STARBOARD`, `ACTUATOR_CENTER` — Select specific tab actuators when encoding BLE messages.
- `DIRECTION_DOWN`, `DIRECTION_UP` — Specify trim direction for potentiometer adjustments.
- `POT_1`, `POT_2` — Identify digital potentiometer channels when using `MSG_POT` or `MSG_POT_TRIM`.

### Message Protocol
Incoming and outgoing BLE messages use framed serial packets (see `FramedSerialProtocol`). Command identifiers and payload shapes are defined via `MSG_*` macros:

| Command | Payload | Usage |
| --- | --- | --- |
| `MSG_RESET` | none | Retract port/starboard actuators; conditionally extend center tab. |
| `MSG_LAUNCH` | none | Fully extend both actuators for launch. |
| `MSG_SURF_LEFT` / `MSG_SURF_RIGHT` | none | Enter surf mode (requires full retraction). |
| `MSG_MANUAL_RETRACT` / `MSG_MANUAL_EXTEND` | `[Actuator]` | Adjust selected actuator by ±5%. |
| `MSG_POT` | `[Pot Index][Pot Value]` | Set digital potentiometer target. |
| `MSG_POT_TRIM` | `[Pot Index][Direction]` | Trim potentiometer relative to current value. |
| `MSG_SCHEDULE_WAKE_TRANSFER` | none | Queue wake-transfer (valid in surf modes). |
| `MSG_DEBUG` | ASCII text | Emitted by firmware for diagnostics. |
| `MSG_STATE` | `[Mode][Port%][Starboard%][Center%][Pot1][Pot2][GpsLock][GpsSpeed][WakeTransferTimer]` | Periodic state broadcast; request by sending command with no data. |
| `MSG_CURRENT_SENSE` | `[PortCS][StarboardCS][CenterCS]` | Current-sense snapshot (8-bit values). |
| `MSG_GET_CONFIG` | none | Request persisted configuration; device responds with same command and 4 data bytes. |
| `MSG_SET_CONFIG` | `[CenterMotor][DemoMode][WakeTransferDelay][FullExtensionTime]` | Persist new configuration and reset MCU. |
| `MSG_LEGACY` | `[LegacyCommand]` | Compatibility command forwarded as-is. |

### Bluetooth Bootstrap
- `void bluetooth_init();` — Initializes the RN4870 BLE module to transparent serial mode. Internally issues command-mode strings via `bluetooth_command()` and drains responses.

### Entry Points
- `void setup();` — Hardware initialization, EEPROM load, analog setup, and peripheral bootstrap.
- `void loop();` — Main control loop (state machine, messaging, actuator logic).

> **Note:** `bluetooth_command()` is file-local; only `bluetooth_init()` is exposed in the header.

## Actuator Control (`Actuator.h`)

`Actuator` encapsulates a single linear actuator with PWM control and run-time tracking.

### Constructor
```c++
Actuator::Actuator(volatile uint8_t *port1,
                   uint8_t pin1,
                   volatile uint8_t *port2,
                   uint8_t pin2,
                   volatile uint8_t *portPwm,
                   uint8_t pinPwm,
                   uint8_t csAdc);
```

- `port1`/`pin1` and `port2`/`pin2` control H-bridge direction pins.
- `portPwm`/`pinPwm` drives the PWM enable line.
- `csAdc` indexes the analog current-sense channel.
- Automatically configures DDR and sets outputs low; initializes full-extension time to 3.5 s.

### Runtime API
- `void poll(uint16_t elapsed_ms);` — Drives the H-bridge according to pending extend/retract durations or manual override, using elapsed time since the previous call.
- `void set_extension(int8_t percent);` — Schedule actuator to reach a target position (0–100%). Clamps input, compensates for motor spin-up, and adjusts calibration when operating at limits.
- `void adjust_extension(int8_t delta_percent);` — Convenience wrapper around `set_extension(get_extension() + delta_percent)`.
- `uint8_t get_extension();` — Estimates current extension (0 or 100 returned when near extremes, otherwise 5–98% range).
- `void set_manual_movement(int8_t movement);` — Direct control: `1` extends, `-1` retracts, `0` disables manual override. Manual motion continues until cleared.
- `int32_t get_outstanding_time();` — Remaining movement in milliseconds (positive for extension, negative for retraction).
- `uint32_t get_full_extension_time();` / `void set_full_extension_time(uint16_t ms);` — Retrieve or configure travel time (clamped to 3000–8000 ms).

### Usage Example
```c++
Actuator trim(&PORTB, 2, &PORTB, 3, &PORTD, 6, POT1_ADC);

void service_actuator(uint16_t elapsed_ms) {
  trim.poll(elapsed_ms);
  if (need_full_extension && trim.get_extension() < 95) {
    trim.set_extension(100);
  }
}
```

## Button Handling (`Buttons.h`)

`Buttons` debounces and classifies up to three physical buttons.

### Constructor
```c++
Buttons::Buttons(volatile uint8_t *port1, uint8_t pin1,
                 volatile uint8_t *port2, uint8_t pin2,
                 volatile uint8_t *port3, uint8_t pin3);
```

- Configures pins as inputs with pull-ups.
- Button indices (`BUTTON_SURF_LEFT`, `BUTTON_RESET`, `BUTTON_SURF_RIGHT`) map into the event masks returned by accessors.

### Polling & Queries
- `void poll(uint16_t elapsed_ms);` — Sample inputs, update timers, and derive edge/hold states.
- `uint8_t falling_edge();` — Buttons newly pressed (held longer than `PRESS_TIME` without prior release).
- `uint8_t rising_edge();` — Buttons newly released after a qualifying press.
- `uint8_t longpressed();` — Buttons surpassing `LONGPRESS_TIME` without release.
- `uint8_t held();` — Buttons currently held beyond long-press threshold; used for continuous manual modes.

### Usage Example
```c++
Buttons controls(&PORTF, 0, &PORTB, 0, &PORTB, 1);

void service_buttons(uint16_t elapsed_ms) {
  controls.poll(elapsed_ms);
  if (controls.falling_edge() & _BV(BUTTON_RESET)) {
    trigger_reset_sequence();
  }
  if (controls.held() & _BV(BUTTON_SURF_LEFT)) {
    actuatorPort.set_manual_movement(1);
  }
}
```

## Output Control (`Output.h`)

`Output` manages a single digital output with on/off/flash states.

- Constructor: `Output(volatile uint8_t *port, uint8_t pin);` — Sets DDR and initializes pin low.
- `void poll(uint16_t elapsed_ms);` — Applies current flash state: off (`0x0000`), solid on (`0xFFFF`), or toggling at configured interval.
- `void on();`, `void off();`, `void flash(uint16_t interval_ms);` — Queue desired state changes, applied on next `poll()`.

### Usage Example
```c++
Output statusLed(&PORTB, 7);

void indicate_status(uint16_t elapsed_ms, bool critical) {
  if (critical) statusLed.flash(200);
  else statusLed.on();
  statusLed.poll(elapsed_ms);
}
```

## GPS Wrapper (`Gps.h`)

Provides a thin layer over `TinyGPS` for speed and lock status tracking.

- Constructor: `Gps();` (the actual serial configuration must occur in user code).
- `void poll(uint16_t elapsed_ms);` — Reads bytes via `software_serial_*` helpers, feeds the TinyGPS parser, and tracks time since last valid sentence (`GPS_TIMEOUT` defines the lock threshold).
- `uint8_t get_locked();` — Returns `1` when valid sentences arrive within timeout, `0` otherwise. Always `1` in demo mode.
- `uint8_t get_speed();` — Last parsed speed in knots (integer). Returns `9` in demo mode.

### Usage Example
```c++
Gps gps;

void update_gps(uint16_t elapsed_ms) {
  gps.poll(elapsed_ms);
  if (!gps.get_locked()) {
    enter_fail_safe();
  }
}
```

## Analog Utilities (`lib/analog/analog.h`)

- `void analog_init(uint8_t analog_pins[], uint8_t count, uint8_t aref);` — Configure ADC channels. `aref` accepts `ANALOG_AREF`, `ANALOG_AVCC`, or `ANALOG_INTERNAL_256` constants.
- `void analog_read_a(uint16_t *buffer);` — Fill `buffer` with current values for the configured channel sequence.
- `uint16_t analog_read_p(uint8_t index);` — Read a single configured channel synchronously.

Ensure `F_CPU` matches the MCU clock (16 MHz in this project) so default prescaler values yield accurate conversions.

## Timer Utilities (`lib/timer/timer.h`)

- `void timer_init();` — Configure Timer1 in CTC mode for 1 kHz interrupts (automatic `sei()` unless `NO_INTERRUPT_ENABLE` is defined).
- `uint32_t timer_millis();` — Milliseconds since initialization (wraps ~49 days).
- `uint32_t timer_micros();` — Microseconds since initialization (wraps ~71 minutes).

Use `timer_millis()` to compute elapsed times for `poll()` calls.

## Software Serial (`lib/serial/software_serial.h`)

Implements half-duplex software UART on PD0 (TX) / PD1 (RX) with interrupt-driven receive.

- `void software_serial_init(uint32_t baud);` — Supported baud rates: 9600 (default) or 115200 (TX only). Requires 16 MHz system clock.
- `uint8_t software_serial_read_b(uint8_t *byte);` — Non-blocking read; returns 1 when a byte is available.
- `void software_serial_write_a(uint8_t *data, uint8_t len);` — Blocking write of a byte buffer.
- `uint8_t software_serial_write_b(uint8_t data);` — Blocking single-byte write.
- `uint8_t software_serial_available();` — Returns non-zero when receive buffer contains data.

Example: Poll within the GPS service loop before feeding characters to `TinyGPS`.

## Hardware Serial (`inc/avr/Serial/SerialAVR.h`)

`digitalcave::SerialAVR` adapts AVR hardware UARTs to the `Stream` interface.

- Constructor signature:
  ```c++
  SerialAVR(uint32_t baud,
            uint8_t dataBits = 8,
            uint8_t parity = 0,
            uint8_t stopBits = 1,
            uint8_t serialPort = 0,
            uint8_t bufferSize = 64);
  ```
  - `serialPort`: 0 → USART0, 1 → USART1 (additional ports can be added).
  - Constructor enables RX interrupts and TX output using double-speed mode when beneficial.
- `uint8_t read(uint8_t *b);` — Non-blocking; disables RX interrupt temporarily to protect the ring buffer.
- `uint8_t write(uint8_t data);` — Blocking transmit.
- `void isr();` — Must be invoked from the corresponding UART RX ISR to push incoming bytes into the buffer.

### Usage Example
```c++
SerialAVR serialAvr(115200, 8, 0, 1, 1, 128);

ISR(USART1_RX_vect) {
  serialAvr.isr();
}
```

## USB Serial (`inc/avr/Serial/SerialUSB.h`)

`digitalcave::SerialUSB` exposes CDC ACM over USB (Teensy-compatible implementation).

- Constructor: `SerialUSB();` — Configures USB hardware, enables interrupts, and starts the CDC stack.
- `uint8_t isConnected();` — Non-zero when USB host has enumerated the device.
- `uint8_t available();` — Count of buffered bytes in RX endpoint.
- `void flushInput();`, `void flushOutput();` — Clear buffers and force immediate transmission.
- `uint8_t read(uint8_t *b);`, `uint8_t write(uint8_t data);`, `uint8_t write(uint8_t* data, uint8_t len);` — Stream operations (all non-blocking for reads, blocking for writes as required by USB semantics).

Standalone C helpers (e.g., `usb_init`, `usb_serial_write_b`) mirror PJRC's API for legacy code.

## Stream Abstractions (`inc/common/Stream`)

- `digitalcave::Stream` — Abstract base class defining `read(uint8_t*)` and `write(uint8_t)` plus utility overloads:
  - `uint8_t read(uint8_t* buffer, uint8_t len);`
  - `uint8_t write(char* data);`
  - `uint8_t write(const char* data);`
  - `uint8_t write(uint8_t* data, uint8_t len);`

- `digitalcave::ArrayStream` — Ring buffer implementation:
  - `ArrayStream(uint8_t capacity);`, `~ArrayStream();`
  - `uint8_t read(uint8_t *b);`, `uint8_t write(uint8_t b);`
  - Buffer status helpers: `remaining()`, `size()`, `isEmpty()`, `isFull()`.

- `digitalcave::NullStream` — Sink/source that drops all writes and never yields data; useful for optional logging targets.

## Framed Serial Protocol (`inc/common/FramedSerialProtocol`)

Encapsulates HDLC-like framing with byte stuffing for robust transport over `Stream` instances.

### `digitalcave::FramedSerialMessage`
- Constructors:
  - `FramedSerialMessage(uint8_t command, uint8_t maxSize);` — Allocate writable frame buffer.
  - `FramedSerialMessage(uint8_t command, uint8_t* message, uint8_t length);` — Wrap existing payload (used internally).
- Destructor frees owned buffer.
- `void clone(FramedSerialMessage* other);` — Copy command/data/length into current instance.
- Accessors: `uint8_t getCommand();`, `uint8_t getLength();`, `uint8_t* getData();`.
- `void append(uint8_t byte);` — Append payload bytes.

### `digitalcave::FramedSerialProtocol`
- Constructor: `FramedSerialProtocol(uint8_t maxPayload);` — Allocate receive buffer and clear state.
- `uint8_t read(Stream* transport, FramedSerialMessage* result);` — Consume bytes, return `1` when a full frame is decoded, and populate `result`.
- `void write(Stream* transport, FramedSerialMessage* message);` — Emit a framed packet with escaping and checksum.
- `uint8_t getError();` — Retrieve the last decoder error (`NO_ERROR` when clear).

### Usage Example
```c++
FramedSerialProtocol protocol(64);
FramedSerialMessage request(0, 64);

if (protocol.read(&serialAvr, &request)) {
  handle_command(request.getCommand(), request.getData(), request.getLength());
}

FramedSerialMessage response(MSG_STATE, 9);
response.append(mode);
response.append(actuatorPort.get_extension());
// ... populate remaining bytes ...
protocol.write(&serialAvr, &response);
```

## TinyGPS Wrapper (`lib/tinygps/TinyGPS.h`)

`TinyGPS` is a third-party library included in the repository. Key public methods:
- `bool encode(char c);` — Feed NMEA stream one character at a time.
- `void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0);`
- `void get_datetime(unsigned long *date, unsigned long *time, unsigned long *age = 0);`
- `long altitude();`, `unsigned long speed();`, `unsigned long course();`
- Float helpers: `f_get_position`, `f_speed_kmph`, `f_course`, etc.
- Static utilities: `distance_between`, `course_to`, `cardinal`.

Consult the upstream TinyGPS documentation for comprehensive coverage; this codebase primarily uses `encode()` and `speed()`.

## DigitalCave Utilities (`inc/common/dcutil`)

- `dcmath.h`: Lookup-table trigonometric helpers (`acos_f`, `cos_f`, `sin_f`), integer square root (`sqrt_f`), and fast inverse square root (`invSqrt`).
- `delay.h`: Busy-wait delay helpers `delay_ms(uint32_t)` and `delay_us(uint32_t)` (preserve interrupts).
- `persist.h`: Abstract persistent storage reads/writes (`persist_write`, `persist_read`) returning zero on success.

These utilities are C-friendly with `extern "C"` guards for optional C++ builds.

## Putting It Together

```c++
#include "SurfControl.h"
#include "Actuator.h"
#include "Buttons.h"
#include "Output.h"
#include "Gps.h"
#include <FramedSerialProtocol.h>

using namespace digitalcave;

Actuator port(...);
Buttons buttons(...);
Output status(...);
Gps gps;
FramedSerialProtocol protocol(64);
FramedSerialMessage inbound(0, 64);

void loop_once() {
  static uint32_t last_ms = timer_millis();
  uint32_t now = timer_millis();
  uint16_t elapsed = now - last_ms;
  last_ms = now;

  port.poll(elapsed);
  buttons.poll(elapsed);
  status.poll(elapsed);
  gps.poll(elapsed);

  if (protocol.read(&serialAvr, &inbound)) {
    handle_command(inbound);
  }
}
```

This example highlights the polling model shared across the firmware: capture elapsed time via `timer_millis()`, service each peripheral, and process framed serial commands as they arrive.

