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
| CAN bus | D4,D5 | Native R4 CAN at 500 kbps |

> **Note:** D0 and D1 are not broken out to the connector. The current board uses a 26-pin SuperSeal connector, and there aren't enough pins to accommodate them. D0/D1 are also the hardware serial lines, so they're unavailable if `Serial` is used for debug output. A future board revision may upgrade to a larger SuperSeal connector to bring these out — not confirmed yet.
>
> **D4 and D5** are occupied by the Nano R4's native CAN controller (CAN_TX/CAN_RX) and are not available for general-purpose I/O.

## Example Code

The included `Corvette_Nano_CANBus_V6.7.ino` is a full working implementation from a 1990 Corvette C4 running Megasquirt-3. It demonstrates how to use the shield as a custom vehicle controller.

### Pins used in this example

| Pin | Assignment | Type |
|-----|-----------|------|
| A0 | TEMP1 — Fuel temp | Analog input |
| A1 | TEMP2 — Oil temp | Analog input |
| A2 | PRESS1 — Fuel pressure | Analog input |
| A3 | PRESS2 — Oil pressure | Analog input |
| A4 | PRESS3 — AC pressure (0.5V–4.5V, 0–438 PSI) | Analog input |
| A5 | TEMP3 — AC evap temp (NTC thermistor, 10kΩ β3950) | Analog input |
| D8 | Low-speed fan (MOSFET) | Digital output |
| D10 | AC compressor clutch (IRLZ44N direct drive) | Digital output |
| D11 | High-speed fan (MOSFET) | Digital output |

The remaining analog inputs (A6–A7), high-current PWM outputs (D3, D6, D9), low-current outputs (D12, D13), and 12V detect inputs (D2, D7) are available on the board but not used in this example. Add them as your build requires.

### CAN bus traffic

**Receives Megasquirt realtime data:**

| CAN ID | Megasquirt Group | Data Parsed |
|--------|-----------------|------------|
| 1520 | Group 00 | RPM (bytes 6–7) |
| 1522 | Group 02 | MAP (bytes 2–3), CLT (bytes 6–7) |
| 1523 | Group 03 | TPS (bytes 0–1), Battery voltage (bytes 2–3) |
| 1562 | Group 42 | Vehicle speed (bytes 0–1) |
| 1572 | Group 52 | CANOUT bitfield — fan control bits from MS (byte 1) |

**Receives from the CCM (custom controller module):**

| CAN ID | Data Parsed |
|--------|------------|
| 1500 | AC request signal (bytes 0–1) |

**Transmits sensor data and diagnostics at 10 Hz:**

| CAN ID | Payload |
|--------|---------|
| 1502 | Fuel temp, oil temp, fuel pressure, oil pressure (raw ADC, 16-bit each) |
| 1503 | AC pressure (PSI × 10), AC evap temp (°F × 10), compressor status flag |
| 1511 | Inter-module failure registry — sensor and system fault bitfields (see below) |

If you're running different CAN broadcast settings in TunerStudio, adjust the `MS_REALTIME_*` constants at the top of the sketch to match.

### 1511 Failure Registry

CAN ID 1511 is the inter-module diagnostic broadcast. The CCM reads these bytes to populate the DIC failure list on the dash display. Two bytes, ten flags:

**`data[0]` — Sensor Fault Byte**

| Bit | Flag | Sensor |
|------|------|--------|
| 0 | Fuel temp sensor fault | TEMP1 (A0) |
| 1 | Oil temp sensor fault | TEMP2 (A1) |
| 2 | Fuel pressure sensor fault | PRESS1 (A2) |
| 3 | Oil pressure sensor fault | PRESS2 (A3) |
| 4 | AC pressure sensor fault | PRESS3 (A4) |
| 5 | Evap temp sensor fault | TEMP3 (A5) |
| 6–7 | Spare | — |

**`data[1]` — System Fault Byte**

| Bit | Flag | Description |
|------|------|-------------|
| 0 | CAN timeout | No inbound CAN for >1s while engine running |
| 1 | Clutch engagement failure | Compressor commanded on, no ≥15 PSI pressure rise within 5s (after 2s clutch settle). Latches OFF until fresh AC request. |
| 2 | Evap temp no response | Compressor on >12s, evap temp never dropped ≥2°F. Advisory — no action. |
| 3 | Repeated high-pressure trip | >2 high-pressure cutouts in a rolling 30s window. Pattern flag — points at fans, condenser airflow, or overcharge. Advisory. |
| 4–7 | Spare | — |

Sensor faults are detected by open/short monitoring — ADC reads ≤25 (short to ground) or ≥1000 (open circuit) set the corresponding bit. System faults are cleared automatically when conditions return to normal, except the clutch engagement fault which latches until the driver drops and re-raises the AC request.

### What the code does

**AC compressor protection** — Two-stage safety tree. Tree 1 checks mechanical and external constraints (AC request active, engine running, TPS <70%, RPM <2200, CAN data fresh, clutch engagement proven). Tree 2 checks thermal and fluid constraints (CLT <235°F, evap temp >34°F, head pressure <375 PSI, thermistor healthy). All must pass for the compressor to engage. OEM-style hysteresis bands on every cutout/re-engagement pair. A 5-second minimum off-time prevents short cycling. A CAN watchdog kills the compressor locally if network data goes stale — no running on frozen values.

**AC output-failure diagnostics** — Watches the compressor after the safety tree commands it and verifies the system actually responded. Three flags:

- *Clutch engagement failure* — on each compressor rising edge, captures head pressure baseline. After a 2-second clutch settle, requires ≥15 PSI rise within 5 seconds. Failure kills the compressor and latches until the driver re-requests AC. Skipped if the AC pressure sensor itself is faulted.
- *Evap temp no response* — compressor on past 12 seconds and evap temp hasn't dropped 2°F from its engage baseline. Advisory only.
- *Repeated high-pressure trip* — tracks the last three high-pressure cutout events in a rolling 30-second window. Three trips in the window sets the flag. Advisory only — the safety tree already protects on each individual trip.

**Fan control** — Two-stage fan control with hysteresis. Reads MS CANOUT bits for ECU-driven fan requests and independently triggers on AC head pressure. Disables both fans above 40 MPH (forced airflow).

**Sensor fault monitoring** — All six analog inputs checked each loop for open/short conditions. Faults are packed into the sensor fault byte and broadcast on 1511 for the CCM to display.

## Getting Started

### Hardware

- Arduino Nano R4
- CAN Bus Expansion Shield (V3)
- Megasquirt-3 with CAN bus enabled at 500 kbps
- CAN termination resistor (120Ω) at each end of the bus

### Software

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (2.x or later)
2. Install the **Arduino Nano R4** board package — the `Arduino_CAN.h` library is included with the core, no separate install needed
3. Open `Corvette_Nano_CANBus_V6.7.ino`
4. Select **Arduino Nano R4** as your board
5. Upload

### Adapting the code

The top of the sketch defines all pin assignments, CAN IDs, and control thresholds as constants. To adapt for your build:

- **Sensors on different pins** — change the `*_PIN` constants
- **Different CAN broadcast IDs** — change the `MS_REALTIME_*` constants to match your TunerStudio configuration
- **Different fan/AC thresholds** — adjust the `AC_*`, `CLT_*`, and `HP_*` constants
- **Add more inputs/outputs** — declare new pins from the unused board pins listed above

The example uses six analog inputs and three digital outputs. The board supports up to eight analog inputs, four high-current PWM outputs, four low-current switches, and two 12V detect inputs — wire what your vehicle needs and extend the code accordingly.

### Version History

| Version | Key Changes |
|---------|-------------|
| V6.7 | AC output-failure diagnostics (clutch engagement, evap response, HP cycling). 1511 failure registry with 10 flags. CLT overheat moved to internal-only. CAN write retry with timeout. |
| V6.6 | Single analogRead per pin per loop. CAN timeout acts locally in safety tree. AC relay renamed to clutch (IRLZ44N direct drive). |
| V6.5 | Sensor fault monitoring. 1511 diagnostic broadcast. CAN watchdog. |
| V5 | Original release. Fan control, alternator cut, AC cycling, sensor telemetry. |
