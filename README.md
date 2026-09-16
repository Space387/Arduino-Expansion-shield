# CAN Bus Expansion Shield for Megasquirt

An Arduino expansion shield for building custom vehicle controller nodes on a Megasquirt CAN bus. Designed for the [Arduino Nano R4](https://store.arduino.cc/nano-r4) using the Nano's **native CAN controller** (500 kbps — no external CAN chip required).

## Board Hardware

The shield breaks out the Nano R4's pins into vehicle-grade connections:

| Pin Group | Pins | Function |
|-----------|------|----------|
| Analog inputs | A0–A7 | Sensor inputs — temperature, pressure, potentiometers |
| High-current PWM | D3, D6, D9, D10 | MOSFET outputs for fans, pumps, relays |
| Low-current switches | D8, D11, D12, D13 | On/off control for lower-load devices |
| 12V detect inputs | D2, D7 | High-frequency 12V inputs — hall sensors, switches |
| CAN bus | — | Native R4 CAN at 500 kbps |

> **Note:** D0 and D1 are not broken out to the connector. The current board uses a 26-pin SuperSeal connector, and there aren't enough pins to accommodate them. D0/D1 are also the hardware serial lines, so they're unavailable if `Serial` is used for debug output. A future board revision may upgrade to a larger SuperSeal connector to bring these out — not confirmed yet.
>
> **D4 and D5** are occupied by the Nano R4's native CAN controller (CAN_TX/CAN_RX) and are not available for general-purpose I/O.

## Example Code

The included `Corvette_Nano_CANBus_V5.ino` is a full working implementation from a 1990 Corvette C4 running Megasquirt-3. It demonstrates how to use the shield as a custom vehicle controller.

### Pins used in this example

| Pin | Assignment | Type |
|-----|-----------|------|
| A0 | TEMP1 | Analog input |
| A1 | TEMP2 | Analog input |
| A2 | PRESS1 | Analog input |
| A3 | PRESS2 | Analog input |
| A4 | AC pressure sensor (0.5V–4.5V, 0–438 PSI) | Analog input |
| A5 | AC NTC thermistor | Analog input |
| D3 | Low-speed fan (MOSFET) | Digital output |
| D6 | High-speed fan (MOSFET) | Digital output |
| D9 | Alternator cut relay (MOSFET) | Digital output |

The remaining analog inputs (A6–A7), low-current outputs (D8, D11–D13), and 12V detect inputs (D2, D7) are available on the board but not used in this example. Add them as your build requires.

### What the code does

**Receives Megasquirt realtime data over CAN:**

| CAN ID | Megasquirt Group | Data Parsed |
|--------|-----------------|------------|
| 1520 | Group 00 | RPM (bytes 6–7) |
| 1522 | Group 02 | MAP (bytes 2–3), CLT (bytes 6–7) |
| 1523 | Group 03 | TPS (bytes 0–1), Battery voltage (bytes 2–3) |
| 1562 | Group 42 | Vehicle speed (bytes 0–1) |
| 1572 | Group 52 | CANOUT bitfield — fan control bits from MS (byte 1) |

If you're running different CAN broadcast settings in TunerStudio, adjust the `MS_REALTIME_*` constants at the top of the sketch to match.

**Also receives from the CCM (custom controller module):**

| CAN ID | Data Parsed |
|--------|------------|
| 1500 | AC request signal (bytes 0–1) |

**Transmits sensor data back to the bus at 10 Hz:**

| CAN ID | Payload |
|--------|---------|
| 1502 | TEMP1, TEMP2, PRESS1, PRESS2 (raw ADC, 16-bit each) |
| 1503 | AC pressure (PSI × 10), AC temperature (°F × 10), compressor status flag |

**Fan control** — Two-stage fan control with hysteresis. Reads MS CANOUT bits for ECU-driven fan requests and independently triggers on AC head pressure. Disables both fans above 40 MPH (forced airflow).

**Alternator control** — Cuts the alternator during cranking, WOT pulls, steady-state cruise, and overcharge conditions. Automatic 10-second cut timer with 5-second recovery lockout. Safety floor at 12.4V — if battery drops below, charging resumes regardless of other conditions.

**AC compressor cycling** — Reads AC request from the CCM and cycles the compressor with pressure and temperature protections: high-pressure cutout at 375 PSI, freeze protection at 33°F, with OEM-style hysteresis bands on re-engagement.

## Getting Started

### Hardware

- Arduino Nano R4
- CAN Bus Expansion Shield (V3)
- Megasquirt-3 with CAN bus enabled at 500 kbps
- CAN termination resistor (120Ω) at each end of the bus

### Software

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (2.x or later)
2. Install the **Arduino Nano R4** board package — the `Arduino_CAN.h` library is included with the core, no separate install needed
3. Open `Corvette_Nano_CANBus_V5.ino`
4. Select **Arduino Nano R4** as your board
5. Upload

### Adapting the code

The top of the sketch defines all pin assignments, CAN IDs, and control thresholds as constants. To adapt for your build:

- **Sensors on different pins** — change the `*_PIN` constants
- **Different CAN broadcast IDs** — change the `MS_REALTIME_*` constants to match your TunerStudio configuration
- **Different fan/alternator thresholds** — adjust the `WOT_*`, `CRUISE_*`, `VOLT_*`, and `AC_*` constants
- **Add more inputs/outputs** — declare new pins from the unused board pins listed above

The example uses six analog inputs and three digital outputs. The board supports up to eight analog inputs, four high-current PWM outputs, four low-current switches, and two 12V detect inputs — wire what your vehicle needs and extend the code accordingly.
