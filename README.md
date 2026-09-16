# CAN Bus Expansion Shield for Megasquirt

An Arduino expansion shield for building custom vehicle controller nodes on a Megasquirt CAN bus. Designed for the [Arduino Nano R4](https://store.arduino.cc/nano-r4) and built around the Nano's **native CAN controller** (500 kbps, no external CAN chip required).

The board provides:

- **A0–A7** — analog or digital sensor inputs with onboard pullup resistor slots
- **D3, D6, D9, D10** — medium-load (5A) PWM switches via onboard MOSFETs
- **D8, D11, D12, D13** — low-load (0.5A) switches
- **D2, D7** — 12V detect inputs
- **Native CAN interface** — 500 kbps via the Nano R4's built-in CAN controller (CAN_H/CAN_L on the shield terminal block)

> **Note:** D0/D1 are available as basic on/off outputs but become non-functional if `Serial` is used for debug output.

---

## Example Code

The included `Corvette_Nano_CANBus_V5.ino` is a full working implementation from a 1990 Corvette C4 running Megasquirt-3. It demonstrates several common tasks the shield is built for:

### Reading Megasquirt realtime data over CAN

The code listens for Megasquirt realtime broadcast frames at 500 kbps and parses engine data:

| CAN ID | Megasquirt Group | Data Parsed |
|--------|-----------------|------------|
| 1520 | Group 00 | RPM (bytes 6–7) |
| 1522 | Group 02 | MAP (bytes 2–3), CLT (bytes 6–7) |
| 1523 | Group 03 | TPS (bytes 0–1), Battery voltage (bytes 2–3) |
| 1562 | Group 42 | Vehicle speed (bytes 0–1) |
| 1572 | Group 52 | CANOUT bitfield — fan control bits from MS (byte 1) |

These are standard Megasquirt-3 realtime CAN broadcast IDs. If you're running different CAN broadcast settings in TunerStudio, adjust the `MS_REALTIME_*` constants at the top of the sketch to match your configuration.

### Broadcasting sensor data back to the bus

The shield transmits two custom CAN frames at 10 Hz (every 100 ms):

- **CAN ID 1502** — four raw analog sensor values (two temperature, two pressure) packed as 16-bit integers
- **CAN ID 1503** — AC line pressure (scaled ×10), AC evaporator temperature (scaled ×10), and the AC compressor command bit

Megasquirt can receive these as generic CAN inputs — map them in TunerStudio under CAN Remote Variables to use sensor data in your fuel/spark tables or output strategies.

### Fan control

Two-stage fan control with MOSFET outputs on D3 (low) and D6 (high). The logic combines Megasquirt CANOUT fan request bits with local AC pressure thresholds and vehicle speed — fans shut off above 40 mph since airflow handles cooling at speed.

### Alternator control

A relay output on D9 cuts alternator field charging during cranking, WOT pulls, steady-state cruise, and overvoltage conditions — reducing parasitic load when you need the power elsewhere. Safety floor prevents cut if battery voltage drops below 12.4V, and a 10-second time limit prevents continuous cut.

### AC compressor control

Reads an AC request signal from Megasquirt (CAN ID 1500), monitors line pressure and evaporator temperature via analog inputs, and cycles the compressor with hysteresis on both pressure (375 PSI cutout / 325 PSI cutin) and temperature (33°F freeze protection / 38.5°F re-engage). The compressor command is broadcast back to Megasquirt on CAN ID 1503.

---

## Getting Started

### Hardware

- CAN Bus Expansion Shield V3 (sold separately)
- Arduino Nano R4 (not included with the board)
- Megasquirt-3 (or compatible) with CAN broadcast enabled at 500 kbps

### Software

1. Open `Corvette_Nano_CANBus_V5.ino` in the Arduino IDE
2. The code uses the `Arduino_CAN.h` library — this is included with the Arduino Nano R4 board package (no separate library install needed)
3. Review the pin declarations and CAN IDs at the top of the sketch — these map to specific Corvette C4 sensor locations and Megasquirt CAN IDs
4. Modify pin assignments, thresholds, and CAN IDs to match your vehicle and Megasquirt configuration
5. Upload to your Nano R4

### Adapting the code

This example is purpose-built for a Corvette C4, but the patterns are reusable:

- **Pin assignments** — change the `*_PIN` constants to match how you've wired your sensors and outputs to the shield
- **CAN IDs** — update the `MS_REALTIME_*` constants to match your Megasquirt CAN broadcast configuration in TunerStudio
- **Thresholds** — fan temperatures, alternator cut parameters, and AC hysteresis values are all defined as constants at the top for easy adjustment
- **Sensor scaling** — pressure and temperature conversions use standard formulas; replace with your sensor's calibration data

---

## License

This example code is provided as a starting point for purchasers of the CAN Bus Expansion Shield. Modify and adapt freely for your own projects.

---

## About

Designed and built by [Space387](https://github.com/Space387). Hardware available separately — this repository contains example firmware only.
