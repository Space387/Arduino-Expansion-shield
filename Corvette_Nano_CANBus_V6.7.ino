/*
===================================================================
      CORVETTE NANO CANBUS — V6.7
      Changes from V6.6:
        - AC output-failure diagnostics (three new flags on 1511):
            bit 1: Clutch engagement failure (commanded, no pressure
                   rise >=15 psi within 5s after 2s settle -> kill
                   clutch, latch until fresh AC request)
            bit 2: Evap temp no response (compressor on 12s, evap
                   temp never fell >=2F — advisory, no action)
            bit 3: Repeated high-pressure trip (>2 cutouts in a
                   rolling 30s window — pattern flag, no action)
        - CLT overheat removed from 1511 (internal safety only;
          MS3 already broadcasts CLT for the CCM display)
        - headPressureTripped moved to global scope so the
          diagnostic layer can watch its rising edges
      1511 CONTRACT (read by CCM for DIC failure list):
        data[0] sensor faults: b0 fuel temp, b1 oil temp,
               b2 fuel press, b3 oil press, b4 AC press, b5 evap temp
        data[1] system faults: b0 CAN timeout, b1 clutch engage,
               b2 evap no-response, b3 repeated HP trip
===================================================================
Pin functions:
A0-A7 Analog inputs
D3, D6, D9, D10 - High load PWM
D8, D11, D12, D13 - Medium load non PWM
D2, D7 - 12V inputs
*/

#include <Arduino_CAN.h> // Native R4 Core CAN library
#include <math.h>        // Required for Log() Functions

// Pin Declarations
const int TEMP1_PIN  = A0;  // Fuel temp
const int TEMP2_PIN  = A1;  // Oil temp
const int PRESS1_PIN = A2;  // Fuel pressure
const int PRESS2_PIN = A3;  // Oil pressure
const int PRESS3_PIN = A4;  // AC pressure
const int TEMP3_PIN  = A5;  // AC temp

const int FAN_LOW_PIN   = 8;   // Low-Speed MOSFET Output
const int FAN_HIGH_PIN  = 11;  // High-Speed MOSFET Output
const int AC_CLUTCH_PIN = 10;  // Compressor clutch, IRLZ44N direct drive

// CAN Configuration IDs (Our Outbound Signals)
const uint32_t CAN_ID_1502 = 1502;
const uint32_t CAN_ID_1503 = 1503;
const uint32_t CAN_ID_1511 = 1511; // Inter-Module Master Failure Registry Address

// MegaSquirt Realtime Stream IDs Mapped From Custom Vehicle Telemetry
const uint32_t CCM_REALTIME_GRP00 = 1500; // Data output from the CCM group 00
const uint32_t MS_REALTIME_GRP00  = 1520; // Group 00 (RPM on Bytes 6-7)
const uint32_t MS_REALTIME_GRP02  = 1522; // Group 02 (MAP, CLT)
const uint32_t MS_REALTIME_GRP03  = 1523; // Group 03 (TPS, Batt)
const uint32_t MS_REALTIME_VSS    = 1562; // Group 42 Base (VSS)
const uint32_t MS_REALTIME_CANOUT = 1572; // Group 52 Base (CANOUT 1-8 bitfield)

// AC control constants
const uint16_t SERIES_RESISTOR     = 4700;
const uint16_t NOMINAL_RESISTANCE  = 10000;
const uint16_t NOMINAL_TEMPERATURE = 25;
const uint16_t BETA_COEFFICIENT    = 3950;

// AC TARGET CONFIGURATIONS (With OEM Hysteresis Bands)
const uint16_t AC_PRESS_CUTOUT   = 375;  // Maximum head pressure cutout
const uint16_t AC_PRESS_CUTIN    = 325;  // Re-engage at 325 PSI (375 - 50 hysteresis)
const float    AC_TEMP_CUTOUT    = 34.0; // Freeze protection threshold
const float    AC_TEMP_CUTIN     = 46.0; // Re-engage threshold

// Engine Coolant Temperature Safety Thresholds
const int16_t  CLT_SAFETY_CUTOUT   = 2350; // Cut AC if engine reaches 235.0°F
const int16_t  CLT_SAFETY_RECOVERY = 2250; // Re-allow AC at 225.0°F

// CAN WATCHDOG
const unsigned long CAN_TIMEOUT_MS = 1000; // No inbound CAN for 1s = stale data

// AC DIAGNOSTIC THRESHOLDS (output failure detection)
const unsigned long ENGAGE_SETTLE_MS    = 2000; // Clutch magnetic settle time
const unsigned long ENGAGE_WINDOW_MS    = 5000; // Pressure must rise within this after settle
const uint16_t      ENGAGE_MIN_RISE_PSI = 15;   // Minimum pressure rise proving the clutch pumped
const unsigned long EVAP_RESPONSE_MS    = 12000;// Evap temp must start falling within this
const float         EVAP_MIN_DROP_F     = 2.0;  // Minimum evap temp drop proving airflow/refrigerant
const unsigned long HP_CYCLE_WINDOW_MS  = 30000;// Rolling window for repeated-trip detection
const uint8_t       HP_CYCLE_MAX_TRIPS  = 3;    // More than 2 trips in the window = fault

// Global tracking variables
uint16_t engineRPM      = 0;
uint16_t engineMAP      = 0;   // Multiplied by 10 (650 = 65.0 kPa)
uint16_t engineTPS      = 0;   // Multiplied by 10 (900 = 90.0% TPS)
uint16_t batteryVoltage = 0;   // Multiplied by 10 (138 = 13.8V)
int16_t  engineCLT      = 0;
uint16_t rawACRequest   = 0;
float    vehicleSpeed   = 0.0;
bool     engineRunning  = false;
bool     lowFanState    = false;
bool     highFanState   = false;
bool     canTimedOut    = false; // Set each loop from lastIncomingCANTime

// Local System State registers
bool    isLowFanON            = false;
bool    isHighFanON           = false;
bool    isCompressorOn        = false;
float   temperatureFahrenheit = 0.0; // Final smoothed temperature calculation
int16_t txTempx10             = 0;

// Safety-tree state (global: the diagnostic layer watches headPressureTripped)
bool headPressureTripped = false;

// Module state trackers
unsigned long lastTxTime           = 0;
unsigned long lastThermistorTime   = 0;
unsigned long lastIncomingCANTime  = 0;
unsigned long compressorOffTime    = 0;
bool          thermistorFault      = false;
uint8_t       sensorFaultByte      = 0x00;

// AC DIAGNOSTIC STATE (output failure detection)
bool          clutchEngageFault    = false;  // Latched: clutch commanded, no pressure rise
bool          evapNoResponseFault  = false;  // Transient: compressor on, evap temp never fell
bool          hpCycleFault         = false;  // Pattern: >2 high-pressure trips in rolling 30s
unsigned long compressorEngageTime = 0;
uint16_t      engageBaselinePsi    = 0;
float         engageBaselineEvap   = 0.0;
unsigned long hpTripTimes[3]       = {0, 0, 0};
uint8_t       hpTripIndex          = 0;

// CAN Setup
CanMsg canMsgOut1502;
CanMsg canMsgOut1503;
CanMsg canMsgOut1511;
CanMsg canMsgIn;

// PRESSURE SENSOR CALIBRATION (Aftermarket 0-100 psi sensors 0.5V-4.5V nominal)
float calibratePressure(uint16_t rawADC) {
  uint16_t adc = constrain(rawADC, 102, 920);
  return -0.00004776 * (float)adc * (float)adc + 0.17858 * (float)adc - 18.50;
}

// COMPRESSOR PROTECTION SAFETY TREES
void processACSafetyTrees(uint16_t scaledACPsi, unsigned long currentTime) {
  const uint16_t AC_TPS_CUTOUT     = 700;   // 70.0% TPS
  const uint16_t AC_TPS_CUTIN      = 600;   // 60.0% TPS
  const uint16_t AC_RPM_CUTOUT     = 2200;  // 2200 RPM Cutoff
  const uint16_t AC_RPM_CUTIN      = 2100;  // 2100 RPM Recovery
  const unsigned long AC_MIN_OFF_TIME_MS = 5000;

  static bool tpsSafetyTripped   = false;
  static bool rpmSafetyTripped   = false;
  static bool cltSafetyTripped   = false;
  static bool evapFreezeTripped  = false;

  bool acRequestActive = (rawACRequest < 512);

  // TREE 1: MECHANICAL & EXTERNAL CONSTRAINTS
  if (engineTPS >= AC_TPS_CUTOUT)       tpsSafetyTripped = true;
  else if (engineTPS <= AC_TPS_CUTIN)   tpsSafetyTripped = false;

  if (engineRPM >= AC_RPM_CUTOUT)       rpmSafetyTripped = true;
  else if (engineRPM <= AC_RPM_CUTIN)   rpmSafetyTripped = false;

  // CAN WATCHDOG: stale network data kills the compressor locally.
  // canTimedOut is checked directly in externalTreePassed — it no
  // longer forces tpsSafetyTripped/rpmSafetyTripped, so the diagnostic
  // print only shows conditions that actually triggered.

  // A proven-dead clutch stays out until the driver drops and re-raises
  // the AC request — one retry per fresh demand, not a 5s hammer loop.
  if (!acRequestActive) clutchEngageFault = false;

  bool externalTreePassed = (acRequestActive && engineRunning &&
                             !canTimedOut &&
                             !tpsSafetyTripped && !rpmSafetyTripped &&
                             !clutchEngageFault);

  // TREE 2: THERMAL & FLUID SAFETY CONSTRAINTS
  if (engineCLT >= CLT_SAFETY_CUTOUT)        cltSafetyTripped = true;
  else if (engineCLT <= CLT_SAFETY_RECOVERY) cltSafetyTripped = false;

  if (temperatureFahrenheit <= AC_TEMP_CUTOUT) evapFreezeTripped = true;
  else if (temperatureFahrenheit >= AC_TEMP_CUTIN)  evapFreezeTripped = false;

  if (scaledACPsi >= AC_PRESS_CUTOUT)        headPressureTripped = true;
  else if (scaledACPsi <= AC_PRESS_CUTIN)    headPressureTripped = false;

  bool thermalTreePassed = (!cltSafetyTripped && !evapFreezeTripped &&
                            !headPressureTripped && !thermistorFault);

  // COMPRESSOR DECISION & LOCK TIMERS
  bool prevCompressorOn = isCompressorOn;

  if (externalTreePassed && thermalTreePassed) {
    if (!isCompressorOn && (currentTime - compressorOffTime >= AC_MIN_OFF_TIME_MS)) {
      isCompressorOn = true;
    }
  } else {
    if (isCompressorOn) {
      compressorOffTime = currentTime;
    }
    isCompressorOn = false;
  }

  // SERIAL DEBUG OUTPUT
  if (isCompressorOn != prevCompressorOn) {
    Serial.println("=== AC COMPRESSOR STATE CHANGE ===");
    Serial.println(isCompressorOn ? "  -> STATUS: COMPRESSOR ENGAGED" : "  -> STATUS: COMPRESSOR DISENGAGED");

    if (!isCompressorOn) {
      Serial.println("  TRIPPED BY CAUSE:");
      if (canTimedOut) {
        Serial.print("    - CAN Timeout (Stale ");
        Serial.print((currentTime - lastIncomingCANTime) / 1000.0, 1);
        Serial.println("s)");
      }
      if (!acRequestActive)    Serial.println("    - Dashboard Request Absent");
      if (!engineRunning)      Serial.println("    - Engine Below Run Threshold (<500 RPM)");
      if (tpsSafetyTripped)    Serial.println("    - TPS Cutout (>70% WOT Protection)");
      if (rpmSafetyTripped)    Serial.println("    - RPM Cutout (>2200 RPM Limit Passed)");
      if (cltSafetyTripped)    Serial.println("    - Engine CLT Overheat Protection Active");
      if (evapFreezeTripped)   Serial.println("    - Evaporator Freeze Protection Cutout");
      if (headPressureTripped) Serial.println("    - High Head Pressure Cutout");
      if (clutchEngageFault)   Serial.println("    - AC Diagnostic: Clutch Engagement Failure (no pressure rise)");
      if (thermistorFault)     Serial.println("    - Thermistor Signal Lost (Open/Short Fault)");
      if (currentTime - compressorOffTime < AC_MIN_OFF_TIME_MS) Serial.println("    - Short-Cycle Protection Active");
    }

    Serial.print("  Stats -> TPS: "); Serial.print((float)engineTPS / 10.0, 1);
    Serial.print("% | RPM: ");        Serial.print(engineRPM);
    Serial.print(" | Evap: ");       Serial.print(temperatureFahrenheit, 1);
    Serial.print("F | Head PSI: ");  Serial.print(scaledACPsi);
    Serial.print(" | CLT: ");        Serial.println((float)engineCLT / 10.0, 1);
    Serial.println("=================================");
  }
}

// AC OUTPUT-FAILURE DIAGNOSTICS
// Watches the compressor the safety trees just commanded and verifies
// the system actually responded. Runs AFTER the trees each loop, so a
// kill here takes effect on this loop's digitalWrite.
void processACDiagnostics(uint16_t scaledACPsi, unsigned long currentTime) {
  static bool prevDiagCompressorOn = false;
  static bool prevHeadTripped      = false;

  bool acRequestActive  = (rawACRequest < 512);
  bool pressureSensorOK = !(sensorFaultByte & (1 << 4)); // PRESS3 healthy

  // ---- FLAG: CLUTCH ENGAGEMENT FAILURE ----
  // Rising edge: capture the baseline the clutch must beat.
  if (isCompressorOn && !prevDiagCompressorOn) {
    compressorEngageTime = currentTime;
    engageBaselinePsi    = scaledACPsi;
    engageBaselineEvap   = temperatureFahrenheit;
  }
  prevDiagCompressorOn = isCompressorOn;

  // After settle + window: pressure must have risen by ENGAGE_MIN_RISE_PSI.
  // Fail -> latch, kill the clutch now (tree honors the latch next loop).
  if (isCompressorOn && pressureSensorOK && !clutchEngageFault &&
      (currentTime - compressorEngageTime) > (ENGAGE_SETTLE_MS + ENGAGE_WINDOW_MS) &&
      scaledACPsi < (engageBaselinePsi + ENGAGE_MIN_RISE_PSI)) {
    clutchEngageFault = true;
    isCompressorOn    = false;
    compressorOffTime = currentTime;
    Serial.println("!!! AC DIAGNOSTIC: Clutch commanded but no pressure rise — compressor killed");
  }

  // ---- FLAG: EVAP TEMP NO RESPONSE (advisory) ----
  // Compressor has been on past the response window and the evaporator
  // hasn't cooled: airflow through the core or refrigerant charge suspect.
  evapNoResponseFault = (isCompressorOn && !thermistorFault &&
                         (currentTime - compressorEngageTime) > EVAP_RESPONSE_MS &&
                         temperatureFahrenheit > (engageBaselineEvap - EVAP_MIN_DROP_F));

  // ---- FLAG: REPEATED HIGH-PRESSURE TRIP (pattern) ----
  // Each rising edge of headPressureTripped is one cutout event. Keep the
  // last three; if all three fall inside the rolling window, the system
  // is cycling — fans, condenser airflow, or overcharge. Advisory only:
  // the safety tree already protects on each individual trip.
  if (headPressureTripped && !prevHeadTripped) {
    hpTripTimes[hpTripIndex] = currentTime;
    hpTripIndex = (hpTripIndex + 1) % 3;
  }
  prevHeadTripped = headPressureTripped;

  uint8_t tripsInWindow = 0;
  for (uint8_t i = 0; i < 3; i++) {
    if (hpTripTimes[i] != 0 && (currentTime - hpTripTimes[i]) <= HP_CYCLE_WINDOW_MS) {
      tripsInWindow++;
    }
  }
  hpCycleFault = (tripsInWindow >= HP_CYCLE_MAX_TRIPS);
}

// HYSTERESIS RADIATOR FAN MANAGEMENT ENGINE
void processFanControl(uint16_t currentACPsi) {
  if (!engineRunning) {
    isLowFanON = false;
    isHighFanON = false;
    return;
  }

  // Low Speed Fan Hysteresis
  if (isLowFanON) {
    if (vehicleSpeed >= 40.0 || (currentACPsi < 75 && !lowFanState)) {
      isLowFanON = false;
    }
  } else {
    if (vehicleSpeed < 35.0 && (currentACPsi > 100 || lowFanState)) {
      isLowFanON = true;
    }
  }

  // High Speed Fan Hysteresis
  if (isHighFanON) {
    if (vehicleSpeed >= 40.0 || (currentACPsi < 200 && !highFanState)) {
      isHighFanON = false;
    }
  } else {
    if (vehicleSpeed < 35.0 && (highFanState || currentACPsi > 250)) {
      isHighFanON = true;
    }
  }
}

// INBOUND CAN NETWORK PACKET PROCESSING
void processIncomingCAN() {
  while (CAN.available()) {
    canMsgIn = CAN.read();
    lastIncomingCANTime = millis();

    switch (canMsgIn.id) {
      case CCM_REALTIME_GRP00:
        rawACRequest = (canMsgIn.data[0] << 8) | canMsgIn.data[1];
        break;
      case MS_REALTIME_GRP00:
        engineRPM = (canMsgIn.data[6] << 8) | canMsgIn.data[7];
        engineRunning = (engineRPM >= 500);
        break;
      case MS_REALTIME_GRP02:
        engineMAP = (canMsgIn.data[2] << 8) | canMsgIn.data[3];
        engineCLT = (canMsgIn.data[6] << 8) | canMsgIn.data[7];
        break;
      case MS_REALTIME_GRP03:
        engineTPS = (canMsgIn.data[0] << 8) | canMsgIn.data[1];
        batteryVoltage = (canMsgIn.data[2] << 8) | canMsgIn.data[3];
        break;
      case MS_REALTIME_CANOUT:
        lowFanState  = (canMsgIn.data[1] & 0x01);
        highFanState = (canMsgIn.data[1] & 0x02);
        break;
      case MS_REALTIME_VSS: {
        uint16_t rawVSS = (canMsgIn.data[0] << 8) | canMsgIn.data[1];
        vehicleSpeed = ((rawVSS / 10.0) * 2.23694);
        break;
      }
    }
  }
}

// AUTOMATED SENSOR OUT-OF-RANGE FAULT MONITOR
// Takes the loop's samples as arguments — no re-reading of pins.
// Bit order: 0=TEMP1, 1=TEMP2, 2=PRESS1, 3=PRESS2, 4=PRESS3, 5=TEMP3
void evaluateSensorFaults(uint16_t rT1, uint16_t rT2, uint16_t rP1,
                          uint16_t rP2, uint16_t rP3, uint16_t rT3) {
  const uint16_t SHORT_TO_GND_ADC = 25;
  const uint16_t OPEN_CIRCUIT_ADC = 1000;

  uint16_t samples[6] = { rT1, rT2, rP1, rP2, rP3, rT3 };

  for (uint8_t i = 0; i < 6; i++) {
    if (samples[i] <= SHORT_TO_GND_ADC || samples[i] >= OPEN_CIRCUIT_ADC) {
      sensorFaultByte |= (1 << i);
    } else {
      sensorFaultByte &= ~(1 << i);
    }
  }
}

// INTER-MODULE DIAGNOSTIC OVERRIDE BROADCAST
// data[0]: sensor fault byte (see evaluateSensorFaults)
// data[1]: system fault byte —
//   bit 0: CAN timeout          bit 2: evap temp no response
//   bit 1: clutch engage fail   bit 3: repeated HP trip
// CLT overheat is internal-only now: MS3 broadcasts CLT itself.
void executeDiagnosticBroadcast() {
  canMsgOut1511.data[0] = sensorFaultByte;

  uint8_t systemFaultByte = 0x00;
  if (canTimedOut && engineRunning) systemFaultByte |= (1 << 0);
  if (clutchEngageFault)            systemFaultByte |= (1 << 1);
  if (evapNoResponseFault)          systemFaultByte |= (1 << 2);
  if (hpCycleFault)                 systemFaultByte |= (1 << 3);
  canMsgOut1511.data[1] = systemFaultByte;

  for (uint8_t i = 2; i < 8; i++) {
    canMsgOut1511.data[i] = 0x00;
  }

  uint32_t timeout1511 = millis();
  while(CAN.write(canMsgOut1511) != 1 && (millis() - timeout1511 < 5)) {
    delayMicroseconds(50);
  }
}

// MATH HELPER: EVAP THERMISTOR CONVERSION
void readEvapTemperature(uint16_t rawTemp3Read) {
  float thermistorResistance = (float)SERIES_RESISTOR / ((1023.0 / (float)rawTemp3Read) - 1.0);
  float steinhart = log(thermistorResistance / (float)NOMINAL_RESISTANCE) / (float)BETA_COEFFICIENT;
  steinhart += 1.0 / ((float)NOMINAL_TEMPERATURE + 273.15);
  temperatureFahrenheit = ((1.0 / steinhart - 273.15) * 9.0 / 5.0) + 32.0;
}

void setup() {
  Serial.begin(115200);

  pinMode(FAN_LOW_PIN, OUTPUT);
  pinMode(FAN_HIGH_PIN, OUTPUT);
  pinMode(AC_CLUTCH_PIN, OUTPUT);

  digitalWrite(FAN_LOW_PIN, LOW);
  digitalWrite(FAN_HIGH_PIN, LOW);
  digitalWrite(AC_CLUTCH_PIN, LOW);

  analogReference(AR_DEFAULT);

  if (!CAN.begin(CanBitRate::BR_500k)) {
    Serial.println("CAN.init failed.");
    for (;;) {}
  }

  Serial.println("MegaSquirt-3 Vehicle Control Network Online...");

  canMsgOut1502.id = CAN_ID_1502; canMsgOut1502.data_length = 8;
  canMsgOut1503.id = CAN_ID_1503; canMsgOut1503.data_length = 8;
  canMsgOut1511.id = CAN_ID_1511; canMsgOut1511.data_length = 8;
}

void loop() {
  unsigned long currentTime = millis();

  processIncomingCAN();

  // SINGLE SAMPLE PASS — read each pin exactly once per loop.
  // These locals feed the fault evaluator, the calibration, the
  // safety trees and the CAN packing alike: one sample, one truth.
  uint16_t rawTemp1  = analogRead(TEMP1_PIN);
  uint16_t rawTemp2  = analogRead(TEMP2_PIN);
  uint16_t rawPress1 = analogRead(PRESS1_PIN);
  uint16_t rawPress2 = analogRead(PRESS2_PIN);
  uint16_t rawPress3 = analogRead(PRESS3_PIN);
  uint16_t rawTemp3  = analogRead(TEMP3_PIN);

  evaluateSensorFaults(rawTemp1, rawTemp2, rawPress1,
                       rawPress2, rawPress3, rawTemp3);

  // CAN watchdog evaluated once per loop, before any decisions
  canTimedOut = (currentTime - lastIncomingCANTime > CAN_TIMEOUT_MS);

  uint16_t clampedADC  = constrain(rawPress3, 102, 920);
  uint16_t scaledACPsi = map(clampedADC, 102, 920, 0, 438);

  // Read and parse thermistor calculations
  thermistorFault = (sensorFaultByte & (1 << 5));
  if (thermistorFault) {
    temperatureFahrenheit = 0.0;
  } else if (currentTime - lastThermistorTime >= 100) {
    lastThermistorTime = currentTime;
    readEvapTemperature(rawTemp3);
  }

  processACSafetyTrees(scaledACPsi, currentTime);
  processACDiagnostics(scaledACPsi, currentTime);
  processFanControl(scaledACPsi);

  digitalWrite(FAN_LOW_PIN,  isLowFanON  ? HIGH : LOW);
  digitalWrite(FAN_HIGH_PIN, isHighFanON ? HIGH : LOW);
  digitalWrite(AC_CLUTCH_PIN, isCompressorOn ? HIGH : LOW);

  // Network TX Telemetry Stream (10 Hz Window)
  if (currentTime - lastTxTime >= 100) {
    lastTxTime = currentTime;

    uint16_t fuelPsi_x10 = (uint16_t)(calibratePressure(rawPress1) * 10.0);
    uint16_t oilPsi_x10  = (uint16_t)(calibratePressure(rawPress2) * 10.0);
    txTempx10            = (int16_t)(temperatureFahrenheit * 10.0);
    uint16_t acREQ       = isCompressorOn ? 0 : 1023;

    // Outbound Pack 1502
    canMsgOut1502.data[0] = rawTemp1 >> 8;     canMsgOut1502.data[1] = rawTemp1;
    canMsgOut1502.data[2] = rawTemp2 >> 8;     canMsgOut1502.data[3] = rawTemp2;
    canMsgOut1502.data[4] = fuelPsi_x10 >> 8;  canMsgOut1502.data[5] = fuelPsi_x10;
    canMsgOut1502.data[6] = oilPsi_x10 >> 8;   canMsgOut1502.data[7] = oilPsi_x10;

    uint32_t timeout1502 = millis();
    while(CAN.write(canMsgOut1502) != 1 && (millis() - timeout1502 < 5)) {
      delayMicroseconds(50);
    }

    // Outbound Pack 1503
    canMsgOut1503.data[0] = rawPress3 >> 8;    canMsgOut1503.data[1] = rawPress3;
    canMsgOut1503.data[2] = txTempx10 >> 8;    canMsgOut1503.data[3] = txTempx10;
    canMsgOut1503.data[4] = acREQ >> 8;        canMsgOut1503.data[5] = acREQ;
    canMsgOut1503.data[6] = 0x00;              canMsgOut1503.data[7] = 0x00;

    uint32_t timeout1503 = millis();
    while(CAN.write(canMsgOut1503) != 1 && (millis() - timeout1503 < 5)) {
      delayMicroseconds(50);
    }

    executeDiagnosticBroadcast();

    // Telemetry Debug Printout
    Serial.print("Evap Temp: ");      Serial.print(temperatureFahrenheit, 1);
    Serial.print("F | Head PSI: ");   Serial.print(scaledACPsi);
    Serial.print(" | Fault Mask: B"); Serial.print(sensorFaultByte, BIN);
    Serial.print(" | Sys Flags: B");  Serial.print(canMsgOut1511.data[1], BIN);
    Serial.print(" | Oil PSI: ");     Serial.print(oilPsi_x10 / 10.0, 1);
    Serial.print(" | Fuel PSI: ");    Serial.print(fuelPsi_x10 / 10.0, 1);
    Serial.print(" | RPM: ");          Serial.print(engineRPM);
    Serial.print(" | TPS: ");          Serial.print((float)engineTPS / 10.0, 1);
    Serial.print(" | VBatt: ");        Serial.print(batteryVoltage / 10.0, 1);
    Serial.println(engineCLT >= CLT_SAFETY_CUTOUT ? " | !!! OVERHEAT FAILSAFE ACTIVE !!!" : "");
  }
}
