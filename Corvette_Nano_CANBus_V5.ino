/*
===================================================================
      UPDATED VEHICLE CONTROLLER (WITH INTEGRATED ALTERNATOR CONTROL)
      Includes AC temperature monitoring & Compressor CAN broadcast
===================================================================
Pin functions:
A0-A7 Analog inputs
D3, D6, D9, D10 - High load PWM 
D8, D11, D12, D13 - Medium load non PWM
D2, D7 - 12V inputs

*/

#include <Arduino_CAN.h> // Native R4 Core CAN library
#include <math.h> // Required for Log() Functions

// Pin Declarations
const int TEMP1_PIN  = A0;  //fuel temp
const int TEMP2_PIN  = A1;  //oil temp
const int PRESS1_PIN = A2;  //fuel pressure
const int PRESS2_PIN = A3;  //oil pressure
const int PRESS3_PIN = A4;  // AC pressure
const int TEMP3_PIN  = A5;  //AC temp

const int FAN_LOW_PIN   = 8;  // Low-Speed MOSFET Output
const int FAN_HIGH_PIN  = 11;  // High-Speed MOSFET Output
const int ALT_RELAY_PIN = 12;  // NEW: Alternator Cut Relay (Triggers High to Cut NC contacts)
const int AC_RELAY_PIN  = 13;  // NEW: Compressor relay trigger

// CAN Configuration IDs (Our Outbound Signals)
const uint32_t CAN_ID_1502 = 1502; 
const uint32_t CAN_ID_1503 = 1503; 

// MegaSquirt Realtime Stream IDs Mapped From Custom Vehicle Telemetry
const uint32_t CCM_REALTIME_GRP00 = 1500; // Data output from the CCM group 00
const uint32_t MS_REALTIME_GRP00  = 1520; // Group 00 (RPM on Bytes 6-7, MAP on Bytes 0-1)
const uint32_t MS_REALTIME_GRP01  = 1521; // Group 01 (Spk, Bits, AFRtgt1, AFRtgt2)
const uint32_t MS_REALTIME_GRP02  = 1522; // Group 02 (Baro, MAP, MAT, CLT)
const uint32_t MS_REALTIME_GRP03  = 1523; // Group 03 (TPS, Batt, EGO1,2)
const uint32_t MS_REALTIME_VSS    = 1562; // Group 42 Base (VSS)
const uint32_t MS_REALTIME_CANOUT = 1572; // Group 52 Base (CANOUT 1-8 bitfield)

//=================AC control constants ===================
const uint16_t SERIES_RESISTOR = 4700;
const uint16_t NOMINAL_RESISTANCE = 10000;
const uint16_t NOMINAL_TEMPERATURE = 25;
const uint16_t BETA_COEFFICIENT = 3950;
const uint16_t ADC_MAX = 1023;

// NEW AC TARGET CONFIGURATIONS (With OEM Hysteresis Bands)
const uint16_t AC_PRESS_CUTOUT   = 375; // Maximum head pressure cutout
const uint16_t AC_PRESS_CUTIN    = 325; // Re-engage at 325 PSI (375 - 50 hysteresis)
const float    AC_TEMP_CUTOUT    = 34.0; // Freeze protection threshold
const float    AC_TEMP_CUTIN     = 46.0; // Re-engage threshold (12°F hysteresis swing)
static float   tempAvg           = 0;
static uint8_t tempSamples       = 0;
const uint8_t  AVG_WINDOW        = 10; // ~10 reads at loop speed ≈ 50-100ms depending on loop rate
float      temperatureFahrenheit = 0;

// NEW: Engine Coolant Temperature Safety Thresholds (MegaSquirt values are Celsius or Fahrenheit x10)
const int16_t  CLT_SAFETY_CUTOUT  = 2350; // Cut AC if engine reaches 235.0°F
const int16_t  CLT_SAFETY_RECOVERY = 2250; // Re-allow AC once high-speed fans pull it down to 225.0°F

// Fixed Custom Alternator Cut Thresholds (ZF6 / 3.33 / CC503 Profile)
const uint16_t WOT_RPM_MIN    = 3500;
const uint16_t WOT_TPS_MIN    = 900;   // 90.0% TPS
const uint16_t CRUISE_RPM_MIN = 1200;
const uint16_t CRUISE_RPM_MAX = 1900;
const uint16_t CRUISE_MAP_MIN = 480;   // 48.0 kPa
const uint16_t CRUISE_MAP_MAX = 680;   // 68.0 kPa
const uint16_t CRUISE_TPS_MAX = 180;   // 15.0% TPS
const uint16_t DECEL_MAP_MAX  = 380;   // 38.0 kPa (Aggressive Lift-Off Zone)
const uint16_t CRANK_RPM_MAX  = 400;   // Engine Cranking Window
const uint16_t VOLT_LOW_SAFE  = 122;   // 11.5V Low Safety Floor
const uint16_t VOLT_HIGH_SAFE = 155;   // 15.5V Overcharge Safety Cut

// Global tracking variables
uint16_t engineRPM      = 0;
uint16_t engineMAP      = 0;   // NEW: Multiplied by 10 (650 = 65.0 kPa)
uint16_t engineTPS      = 0;   // NEW: Multiplied by 10 (900 = 90.0% TPS)
uint16_t batteryVoltage = 0; // NEW: Multiplied by 10 (138 = 13.8V)
int16_t  engineCLT      = 0;
uint16_t rawACRequest   = 0; 
bool engineRunning      = false;
float vehicleSpeed      = 0.0; 
bool lowFanState        = false;
bool highFanState       = false;
bool isLowFanON         = false;
bool isHighFanON        = false;

// NEW: Module state trackers
bool isAltCutActive       = false;
bool isCompressorOn       = false; // Track real-time status of the AC Clutch command
unsigned long altCutTimer = 0;
unsigned long lastTxTime = 0;
unsigned long lastThermistorTime = 0; // NEW: Track 10Hz sampling execution window																				  
unsigned long compressorOffTime = 0;
// CAN Setup 
CanMsg canMsgOut1502;
CanMsg canMsgOut1503;
CanMsg canMsgIn;

  // -------------------------------------------------------------------------
// PRESSURE SENSOR CALIBRATION
// Aftermarket 0-100 psi sensors (0.5V-4.5V nominal)
// Air-tested on one unit, quadratic fit applied to both (same batch)
// Max error < 2.5 psi across 0-100 psi range
// -------------------------------------------------------------------------
float calibratePressure(uint16_t rawADC) {
  uint16_t adc = constrain(rawADC, 102, 920);
  return -0.00004776 * (float)adc * (float)adc + 0.17858 * (float)adc - 18.50;
} 

void setup() {
  Serial.begin(115200);
    
  pinMode(FAN_LOW_PIN, OUTPUT);
  pinMode(FAN_HIGH_PIN, OUTPUT);
  pinMode(ALT_RELAY_PIN, OUTPUT); // NEW
  pinMode(AC_RELAY_PIN, OUTPUT);
  
  // Force startup safety states to OFF / PASSIVE CHARGING
  digitalWrite(FAN_LOW_PIN, LOW);
  digitalWrite(FAN_HIGH_PIN, LOW);
  digitalWrite(ALT_RELAY_PIN, LOW); // Pin LOW leaves Normally Closed contacts CLOSED (Charging active)
  digitalWrite(AC_RELAY_PIN, LOW);

  analogReference(AR_EXTERNAL);
  analogReadResolution(10);
  
if (!CAN.begin(CanBitRate::BR_500k)) {
  Serial.println("CAN.init failed.");
  for (;;) {}
}
  
  Serial.println("MegaSquirt-3 Vehicle Control Network Online...");
  
  canMsgOut1502.id  = CAN_ID_1502;
  canMsgOut1502.data_length = 8;
  
  canMsgOut1503.id  = CAN_ID_1503;
  canMsgOut1503.data_length = 8;
}

void loop() {
  unsigned long currentTime = millis();

  uint16_t rawTemp1  = analogRead(TEMP1_PIN);  
  uint16_t rawTemp2  = analogRead(TEMP2_PIN);  
  uint16_t rawPress1 = analogRead(PRESS1_PIN); 
  uint16_t rawPress2 = analogRead(PRESS2_PIN); 
  uint16_t rawPress3 = analogRead(PRESS3_PIN); 
  uint16_t rawTemp3  = analogRead(TEMP3_PIN);

  uint16_t clampedADC  = constrain(rawPress3, 102, 920);
  uint16_t scaledACPsi = map(clampedADC, 102, 920, 0, 438);

  // -------------------------------------------------------------------------
  // 1. RECEIVE TRAFFIC: Passive Hex Grid Parsing
  // -------------------------------------------------------------------------
  while (CAN.available()) {
      canMsgIn = CAN.read(); // FIXED: Removed shadow redeclaration 'CanMsg const'

    // Parse ID 1500 (CCM 00) -> Extract AC Request
    if (canMsgIn.id == CCM_REALTIME_GRP00) {
      rawACRequest = (int16_t)((canMsgIn.data[0] << 8) | canMsgIn.data[1]);
    }
    // Parse ID 1520 (Group 00) -> Engine RPM (Bytes 6-7)
    if (canMsgIn.id == MS_REALTIME_GRP00) {
      engineRPM = (canMsgIn.data[6] << 8) | canMsgIn.data[7];
      engineRunning = (engineRPM >= 500); 
    }

    // Parse ID 1522 (Group 02) -> Extract True Coolant Temperature
    if (canMsgIn.id == MS_REALTIME_GRP02) {
      engineMAP = (int16_t)((canMsgIn.data[2] << 8) | canMsgIn.data[3]); // NEW
      engineCLT = (int16_t)((canMsgIn.data[6] << 8) | canMsgIn.data[7]);
    }

    // NEW: Parse ID 1523 (Group 03) -> Extract TPS (Bytes0-1) and Battery Volts (Bytes 2-3)
    if (canMsgIn.id == MS_REALTIME_GRP03) {
      engineTPS = (int16_t)((canMsgIn.data[0] << 8) | canMsgIn.data[1]);
      batteryVoltage = (int16_t)((canMsgIn.data[2] << 8) | canMsgIn.data[3]);
    }

    // Parse ID 1572 (Group 52) -> Extract Fan Commands
    if (canMsgIn.id == MS_REALTIME_CANOUT) {
      uint8_t canoutByte = canMsgIn.data[1]; 
      lowFanState  = (canoutByte & 0x01); 
      highFanState = (canoutByte & 0x02); 
    }

    // Parse ID 1562 (Group 42) -> Extract Vehicle Speed
    if (canMsgIn.id == MS_REALTIME_VSS) {
      uint16_t rawVSS = (canMsgIn.data[0] << 8) | canMsgIn.data[1];
      vehicleSpeed = ((rawVSS / 10.0) * 2.23694); 
    }
  }

  // -------------------------------------------------------------------------
  // 2. HARDWARE CONTROL LOGIC: Fans
  // -------------------------------------------------------------------------
  if (!engineRunning) {
    isLowFanON = false;
    isHighFanON = false;
  } 
  else {
    // === LOW SPEED FAN HYSTERESIS ENGINE ===
    if (isLowFanON) {
      if (vehicleSpeed >= 40.0) { isLowFanON = false; } 
      else if (scaledACPsi < 75 && !lowFanState) { isLowFanON = false; }
    } else {
      if (vehicleSpeed < 35.0 && (scaledACPsi > 100 || lowFanState)) { isLowFanON = true; }
    }

    // === HIGH SPEED FAN HYSTERESIS ENGINE ===
    if (isHighFanON) {
      if (vehicleSpeed >= 40.0) { isHighFanON = false; } 
      else if (scaledACPsi < 200 && !highFanState) { isHighFanON = false; }
    } else {
      if (vehicleSpeed < 35.0 && (highFanState || scaledACPsi > 250)) { isHighFanON = true; }
    }
  }

  digitalWrite(FAN_LOW_PIN,  isLowFanON  ? HIGH : LOW);
  digitalWrite(FAN_HIGH_PIN, isHighFanON ? HIGH : LOW);


  // -------------------------------------------------------------------------
  // NEW: 2B. HARDWARE CONTROL LOGIC: Alternator Cut Evaluation
  // -------------------------------------------------------------------------
  bool isCranking      = (engineRPM > 0 && engineRPM < CRANK_RPM_MAX);
  bool isWotPull       = (engineTPS >= WOT_TPS_MIN && engineRPM >= WOT_RPM_MIN);
  bool isSteadyCruise  = (engineRPM >= CRUISE_RPM_MIN && engineRPM <= CRUISE_RPM_MAX &&
                          engineMAP >= CRUISE_MAP_MIN && engineMAP <= CRUISE_MAP_MAX &&
                          engineTPS <= CRUISE_TPS_MAX);
  bool isOvercharging  = (batteryVoltage >= VOLT_HIGH_SAFE);
  
  // Active Failsafes
  bool voltageDroppedTooLow = (batteryVoltage <= VOLT_LOW_SAFE);
  bool continuousTimeLimit  = (isAltCutActive && (currentTime - altCutTimer >= 10000)); // 10s maximum

  if ((isCranking || isWotPull || isSteadyCruise || isOvercharging) && !voltageDroppedTooLow && !continuousTimeLimit) {
    if (!isAltCutActive) {
      altCutTimer = currentTime; // Lock start timestamp
      isAltCutActive = true;
    }
    digitalWrite(ALT_RELAY_PIN, HIGH); // Open NC Contact -> Cut charging load
  } else {
    isAltCutActive = false;
    digitalWrite(ALT_RELAY_PIN, LOW);  // Close NC Contact -> Resume normal charge state
  }

   //---------------------AC Thermistor Calculation----------------------------------
  static bool thermistorFault = false;
  static bool thermistorFaultActive = false;  // for one-shot serial print

  // Check for sensor/wire failure (pullup means open = 1023, short = 0)
  if (rawTemp3 >= 1020 || rawTemp3 <= 3) {
    thermistorFault = true;
  } else {
    thermistorFault = false;
  }

  // Print on transition into or out of fault (no delay, no flooding)
  if (thermistorFault && !thermistorFaultActive) {
    thermistorFaultActive = true;
    Serial.println("=== THERMISTOR FAULT ===");
    if (rawTemp3 >= 1020) {
      Serial.println("  CAUSE: Open circuit (rawTemp3=1023) — sensor or wire disconnected");
    } else {
      Serial.println("  CAUSE: Short to ground (rawTemp3=0) — wire chafed to chassis");
    }
    Serial.println("  AC compressor forced OFF until sensor recovers");
    Serial.println("=========================");
  } else if (!thermistorFault && thermistorFaultActive) {
    thermistorFaultActive = false;
    Serial.println("=== THERMISTOR RECOVERED ===");
    Serial.println("  Sensor reading restored, AC logic resumed");
    Serial.println("=============================");
  }

  if (thermistorFault) {
    // Sensor dead — force compressor off, skip Steinhart to avoid NaN poison
    isCompressorOn = false;
    compressorOffTime = currentTime;
  } else {
    // NEW: Enforce explicit 10Hz sampling execution window (10 readings per second)
    if (currentTime - lastThermistorTime >= 100) {
      lastThermistorTime = currentTime;
							   
										 
															 
								

      float thermistorResistance = (float)SERIES_RESISTOR / ((1023.0 / (float)rawTemp3) - 1.0);
      float steinhart;
      steinhart = thermistorResistance / (float)NOMINAL_RESISTANCE;
      steinhart = log(steinhart);
      steinhart /= (float)BETA_COEFFICIENT;
      steinhart += 1.0 / ((float)NOMINAL_TEMPERATURE + 273.15);
      steinhart = 1.0 / steinhart;

      // Convert Kelvin to Fahrenheit and apply sliding average
      float temperatureCelsius = steinhart - 273.15;
      float tempF = (temperatureCelsius * 9.0 / 5.0) + 32.0;

      tempAvg += tempF;
      tempSamples++;
      if (tempSamples >= AVG_WINDOW) {
        temperatureFahrenheit = tempAvg / (float)tempSamples;
        tempAvg = 0;
        tempSamples = 0;
      }
    }
  }

  bool AC_REQUEST = (rawACRequest < 512);

  // -------------------------------------------------------------------------
  // NEW: Evaluate Engine Coolant Overheat Failsafe State
  // -------------------------------------------------------------------------
  static bool cltSafetyTripped = false;
  if (engineCLT >= CLT_SAFETY_CUTOUT) {
    cltSafetyTripped = true; // Engine hit 235°F. Kill AC to drop thermodynamic load.
  } else if (engineCLT <= CLT_SAFETY_RECOVERY) {
    cltSafetyTripped = false; // High-speed fans cooled it down below 225°F. Safe to resume.
  }

   // -------------------------------------------------------------------------
  // FIXED: 2C. SEPARATED AC CONTROL LOGIC (Safety, Pressure, and Temp Hysteresis)
  //   --- AC state-change serial debug ---
  //   Prints to serial whenever isCompressorOn changes, with the cause and
  //   all relevant sensor values at the moment of the transition.
  // -------------------------------------------------------------------------
  bool prevCompressorOn = isCompressorOn;  // capture state before logic runs
  
  if (AC_REQUEST && engineRunning && !cltSafetyTripped) {
    if (isCompressorOn) {
      // COMPRESSOR RUNNING: Run until we blow an upper limit or drop below freeze point
      if (scaledACPsi >= AC_PRESS_CUTOUT || temperatureFahrenheit <= AC_TEMP_CUTOUT) {
        isCompressorOn = false;
        compressorOffTime = currentTime; // Record compressor shutdown
      } 
    } 
    else { 
      // COMPRESSOR OFF: Hold off until head pressure drops and core warms up
      if (scaledACPsi < AC_PRESS_CUTIN && temperatureFahrenheit >= AC_TEMP_CUTIN) {
        isCompressorOn = true; 
      }
    } // end compressor state check
  } else {
    // FIXED: Ensure baseline shutdown clock triggers anytime a running compressor gets cut by safety limits
    if (isCompressorOn) {
      compressorOffTime = currentTime; // record forced shutdown
    }
    // Force off if no request, engine off, or coolant safety tripped
    isCompressorOn = false;
  } // end safety if
 
														

// --- AC state-change debug output ---
  if (isCompressorOn != prevCompressorOn) {
    Serial.println("=== AC COMPRESSOR STATE CHANGE ===");
    if (isCompressorOn) {
      Serial.println("  -> COMPRESSOR ENGAGED");
    } else {
      Serial.println("  -> COMPRESSOR DISENGAGED");
    }
    // Print the cause
    if (!AC_REQUEST) {
      Serial.println("  CAUSE: AC request lost (rawACRequest >= 512)");
    } else if (!engineRunning) {
      Serial.println("  CAUSE: Engine not running");
    } else if (cltSafetyTripped) {
      Serial.println("  CAUSE: CLT overheat safety tripped");
    } else if (isCompressorOn) {
      // Just turned ON — must have met both cutin conditions
      if (temperatureFahrenheit >= AC_TEMP_CUTIN && scaledACPsi < AC_PRESS_CUTIN) {
        Serial.println("  CAUSE: Both temp and pressure cutin met");
      }
    } else {
      // Just turned OFF while AC requested and engine running — check which limit tripped
      if (scaledACPsi >= AC_PRESS_CUTOUT) {
        Serial.println("  CAUSE: Head pressure cutout");
      }
      if (temperatureFahrenheit <= AC_TEMP_CUTOUT) {
        Serial.println("  CAUSE: Evap freeze cutout");
      }
    }
    // Print all relevant values at the moment of transition
    Serial.print("  rawACRequest: ");   Serial.print(rawACRequest);
    Serial.print("  | AC_REQUEST: ");   Serial.print(AC_REQUEST ? "YES" : "NO");
    Serial.print("  | RPM: ");           Serial.print(engineRPM);
    Serial.print("  | EvapTempRaw: ");   Serial.print(temperatureFahrenheit, 1);
    Serial.print("  | AC_Pressure: ");   Serial.print(scaledACPsi);
    Serial.print("  | CLT: ");          Serial.print((float)engineCLT / 10.0, 1);
    Serial.print("  | cltSafety: ");    Serial.print(cltSafetyTripped ? "TRIPPED" : "OK");
    Serial.print("  | timeSinceOff: ");  Serial.print((currentTime - compressorOffTime) / 1000.0, 1);
    Serial.println("s");
    Serial.println("=================================");
  }
  // -------------------------------------------------------------------------
  // 3. TRANSMIT TRAFFIC: Sensor Broadcast (10 Hz / 100ms)
  // -------------------------------------------------------------------------
  if (currentTime - lastTxTime >= 100) {
    lastTxTime = currentTime;

    // Calibrate pressures before transmit
    uint16_t fuelPsi_x10 = (uint16_t)(calibratePressure(rawPress1) * 10.0);
    uint16_t oilPsi_x10  = (uint16_t)(calibratePressure(rawPress2) * 10.0);

    // Build FRAME 1502 Data Pack
    canMsgOut1502.data[0] = (rawTemp1 >> 8) & 0xFF;  
    canMsgOut1502.data[1] = rawTemp1 & 0xFF;         
    canMsgOut1502.data[2] = (rawTemp2 >> 8) & 0xFF;  
    canMsgOut1502.data[3] = rawTemp2 & 0xFF;         
    canMsgOut1502.data[4] = (fuelPsi_x10 >> 8) & 0xFF;  
    canMsgOut1502.data[5] = fuelPsi_x10 & 0xFF;        
    canMsgOut1502.data[6] = (oilPsi_x10 >> 8) & 0xFF; 
    canMsgOut1502.data[7] = oilPsi_x10 & 0xFF;        
    CAN.write(canMsgOut1502);

    // Build FRAME 1503 Data Pack
    // AC pressure out
    canMsgOut1503.data[0] = (rawPress3 >> 8) & 0xFF; 
    canMsgOut1503.data[1] = rawPress3 & 0xFF;        

    // AC Temp out
      int16_t txTempx10 = temperatureFahrenheit * 10; // FIXED: Using temperatureFahrenheit2 (sliding average) to update 10 times/sec synchronously with the loop
    canMsgOut1503.data[2] = (txTempx10 >> 8) & 0xFF;
    canMsgOut1503.data[3] = txTempx10 & 0xFF;

    //AC Request
    uint16_t acREQ = isCompressorOn ? 0: 1023;  //true - 0v, false 5v
    canMsgOut1503.data[4] = (acREQ >> 8) & 0xFF; 
    canMsgOut1503.data[5] = acREQ &0xFF;
    
    // remaining bytes
    canMsgOut1503.data[6] = 0x00; 
    canMsgOut1503.data[7] = 0x00;
    CAN.write(canMsgOut1503);

    // Debugging printouts
    Serial.print("AC Request RAW: ");        Serial.print(rawACRequest);
    Serial.print(" | AC Temp (Smooth): ");     Serial.print(temperatureFahrenheit);
    Serial.print(" | AC Pressure: ");          Serial.println(scaledACPsi);
    Serial.print(" Oil pressure: ");         Serial.print(oilPsi_x10 / 10.0, 1);
    Serial.print(" | Fuel pressure: ");      Serial.println(fuelPsi_x10 / 10.0, 1);
    Serial.print(" RPM: ");                Serial.print(engineRPM);
    Serial.print(" | MAP: ");                Serial.print((float)engineMAP / 10.0, 1);
    Serial.print(" | TPS: ");                Serial.print((float)engineTPS / 10.0, 1);
    Serial.print(" | VBatt: ");              Serial.println(batteryVoltage /10.0,1);
    Serial.print(" | ALT STATE: ");          Serial.print(isAltCutActive ? "CUT" : "CHARGING");
      if(cltSafetyTripped) {
      Serial.println(" | AC SAFETY: OVERHEAT CUTOUT ACTIVE!");
    } else {
      Serial.println();
    }
  }
}
