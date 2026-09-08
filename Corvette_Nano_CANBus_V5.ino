/*
===================================================================
      UPDATED VEHICLE CONTROLLER (WITH INTEGRATED ALTERNATOR CONTROL)
      Includes AC temperature monitoring & Compressor CAN broadcast
===================================================================
*/

#include <SPI.h>
#include <mcp2515.h> // Library: "arduino-mcp2515" by autowp
#include <math.h> // Required for Log() Functions

/*Pin functions 
A0-A7 - analog inputs
D3, 5, 6, 9 - PWM through Mosfet
D4, 7, 8 - Direct to Nano on/ off output
D0, D1 are non functional if serial is called else Basic on/off

Future pinouts for the V3 board
A0-A7 Analog inputs
D3, D6, D9, D10 - High load PWM 
D8, D11, D12, D13 - Medium load non PWM
D2, D7 - 12V inputs
*/

// Pin Declarations
const int TEMP1_PIN  = A0;
const int TEMP2_PIN  = A1;
const int PRESS1_PIN = A2;
const int PRESS2_PIN = A3;
const int PRESS3_PIN = A4; // AC Pressure Sensor (0.5V = 0 PSI, 4.5V = 438 PSI)
const int TEMP3_PIN  = A5; // AC NTC Thermistor
const int CS_PIN     = 10;

const int FAN_LOW_PIN   = 5;  // Low-Speed MOSFET Output
const int FAN_HIGH_PIN  = 6;  // High-Speed MOSFET Output
const int ALT_RELAY_PIN = 9;  // Alternator Cut Relay (Triggers High to Cut NC contacts)

// CAN Configuration IDs (Our Outbound Signals)
const uint32_t CAN_ID_1502 = 1502; 
const uint32_t CAN_ID_1503 = 1503; 

// MegaSquirt Realtime Stream IDs Mapped From Custom Vehicle Telemetry
const uint32_t CCM_REALTIME_GRP00 = 1500; // Data output from the CCM group 00
const uint32_t MS_REALTIME_GRP00  = 1520; // Group 00 (RPM on Bytes 6-7)
const uint32_t MS_REALTIME_GRP02  = 1522; // Group 02 (MAP)
const uint32_t MS_REALTIME_GRP03  = 1523; // Group 03 (TPS, Batt)
const uint32_t MS_REALTIME_VSS    = 1562; // Group 42 Base (VSS)
const uint32_t MS_REALTIME_CANOUT = 1572; // Group 52 Base (CANOUT 1-8 bitfield)

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

//=================AC control constants ===================
const uint16_t SERIES_RESISTOR = 4700;
const uint16_t NOMINAL_RESISTANCE = 10000;
const uint16_t NOMINAL_TEMPERATURE = 25;
const uint16_t BETA_COEFFICIENT = 3950;
const uint16_t ADC_MAX = 1023;

// NEW AC TARGET CONFIGURATIONS (With OEM Hysteresis Bands)
const uint16_t AC_PRESS_CUTOUT   = 375; // Maximum head pressure cutout
const uint16_t AC_PRESS_CUTIN    = 325; // Re-engage at 325 PSI (375 - 50 hysteresis)
const float    AC_TEMP_CUTOUT    = 33.0; // Freeze protection threshold
const float    AC_TEMP_CUTIN     = 38.5; // Re-engage threshold (5.5°F hysteresis swing)

MCP2515 mcp2515(CS_PIN);     

struct can_frame canMsgOut1502;
struct can_frame canMsgOut1503;
struct can_frame canMsgIn;

// Global tracking variables
uint16_t engineRPM      = 0;
uint16_t engineMAP      = 0;   
uint16_t engineTPS      = 0;   
uint16_t batteryVoltage = 0; 
int16_t  engineCLT      = 0; 
uint16_t rawACRequest   = 0;
bool engineRunning      = false;
float vehicleSpeed      = 0.0; 
bool lowFanState        = false;
bool highFanState       = false;
bool isLowFanON         = false;
bool isHighFanON        = false;

// Alternator & AC State Trackers
bool isAltCutActive       = false;
bool isCompressorOn       = false; // Track real-time status of the AC Clutch command
unsigned long altCutTimer = 0;
unsigned long lastTxTime  = 0;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  
  pinMode(FAN_LOW_PIN, OUTPUT);
  pinMode(FAN_HIGH_PIN, OUTPUT);
  pinMode(ALT_RELAY_PIN, OUTPUT); 
  
  // Force startup safety states to OFF / PASSIVE CHARGING
  digitalWrite(FAN_LOW_PIN, LOW);
  digitalWrite(FAN_HIGH_PIN, LOW);
  digitalWrite(ALT_RELAY_PIN, LOW); 

  analogReadResolution(10);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("MegaSquirt-3 Vehicle Control Network Online...");
  
  canMsgOut1502.can_id  = CAN_ID_1502;
  canMsgOut1502.can_dlc = 8;
  
  canMsgOut1503.can_id  = CAN_ID_1503;
  canMsgOut1503.can_dlc = 8;
}

void loop() {
  unsigned long currentTime = millis();

  uint16_t rawTemp1  = analogRead(TEMP1_PIN);  
  uint16_t rawTemp2  = analogRead(TEMP2_PIN);  
  uint16_t rawPress1 = analogRead(PRESS1_PIN); 
  uint16_t rawPress2 = analogRead(PRESS2_PIN); 
  uint16_t rawPress3 = analogRead(PRESS3_PIN); 
  uint16_t rawTemp3  = analogRead(TEMP3_PIN);
  
  // 1. Unified Local Transduction calculations (No duplicate maps)
  uint16_t clampedADC  = constrain(rawPress3, 102, 920);
  uint16_t scaledACPsi = map(clampedADC, 102, 920, 0, 438);
     
  // -------------------------------------------------------------------------
  // 1. RECEIVE TRAFFIC: Passive Hex Grid Parsing
  // -------------------------------------------------------------------------
  while (mcp2515.readMessage(&canMsgIn) == MCP2515::ERROR_OK) {
    if (canMsgIn.can_id == CCM_REALTIME_GRP00) {
      rawACRequest = (int16_t)((canMsgIn.data[0] << 8) | canMsgIn.data[1]);
    }
    if (canMsgIn.can_id == MS_REALTIME_GRP00) {
      engineRPM = (canMsgIn.data[6] << 8) | canMsgIn.data[7];
      engineRunning = (engineRPM >= 500); 
    }
    if (canMsgIn.can_id == MS_REALTIME_GRP02) {
      engineMAP = (int16_t)((canMsgIn.data[2] << 8) | canMsgIn.data[3]); 
      engineCLT = (int16_t)((canMsgIn.data[6] << 8) | canMsgIn.data[7]);
    }
    if (canMsgIn.can_id == MS_REALTIME_GRP03) {
      engineTPS = (int16_t)((canMsgIn.data[0] << 8) | canMsgIn.data[1]);
      batteryVoltage = (int16_t)((canMsgIn.data[2] << 8) | canMsgIn.data[3]);
    }
    if (canMsgIn.can_id == MS_REALTIME_CANOUT) {
      uint8_t canoutByte = canMsgIn.data[1]; 
      lowFanState  = (canoutByte & 0x01); 
      highFanState = (canoutByte & 0x02); 
    }
    if (canMsgIn.can_id == MS_REALTIME_VSS) {
      uint16_t rawVSS = (canMsgIn.data[0] << 8) | canMsgIn.data[1];
      vehicleSpeed = (float)rawVSS / 10.0; 
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
    if (isLowFanON) {
      if (vehicleSpeed >= 40.0) { isLowFanON = false; } 
      else if (scaledACPsi < 75 && !lowFanState) { isLowFanON = false; }
    } else {
      if (vehicleSpeed < 35.0 && (scaledACPsi > 100 || lowFanState)) { isLowFanON = true; }
    }

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
  // 2B. HARDWARE CONTROL LOGIC: Alternator Cut Evaluation
  // -------------------------------------------------------------------------
  bool isCranking      = (engineRPM > 0 && engineRPM < CRANK_RPM_MAX);
  bool isWotPull       = (engineTPS >= WOT_TPS_MIN && engineRPM >= WOT_RPM_MIN);
  bool isSteadyCruise  = (engineRPM >= CRUISE_RPM_MIN && engineRPM <= CRUISE_RPM_MAX &&
                          engineMAP >= CRUISE_MAP_MIN && engineMAP <= CRUISE_MAP_MAX &&
                          engineTPS <= CRUISE_TPS_MAX);
  bool isOvercharging  = (batteryVoltage >= VOLT_HIGH_SAFE);
  
  bool voltageDroppedTooLow = (batteryVoltage <= VOLT_LOW_SAFE);
  bool continuousTimeLimit  = (isAltCutActive && (currentTime - altCutTimer >= 10000)); 

  if ((isCranking || isWotPull || isSteadyCruise || isOvercharging) && !voltageDroppedTooLow && !continuousTimeLimit) {
    if (!isAltCutActive) {
      altCutTimer = currentTime; 
      isAltCutActive = true;
    }
    digitalWrite(ALT_RELAY_PIN, HIGH); 
  } else {
    isAltCutActive = false;
    digitalWrite(ALT_RELAY_PIN, LOW);  
  }

  //---------------------AC Thermistor Calculation----------------------------------
  if (rawTemp3 == 0) {
    Serial.println("Error: ADC reading is 0. Check your wiring!");
    delay(1000);
    return;
  }

  float thermistorResistance = (float)SERIES_RESISTOR / (((float)ADC_MAX / (float)rawTemp3) - 1.0);
  float steinhart;
  steinhart = thermistorResistance / (float)NOMINAL_RESISTANCE;     
  steinhart = log(steinhart);                                
  steinhart /= (float)BETA_COEFFICIENT;                             
  steinhart += 1.0 / ((float)NOMINAL_TEMPERATURE + 273.15);         
  steinhart = 1.0 / steinhart;                               

  float temperatureCelsius = steinhart - 273.15;
  float temperatureFahrenheit = (temperatureCelsius * 9.0 / 5.0) + 32.0;
  
  bool AC_REQUEST = (rawACRequest < 512);

  // -------------------------------------------------------------------------
  // 2C. HARDWARE CONTROL LOGIC: AC Compressor Cycling Engine
  // -------------------------------------------------------------------------
  if (AC_REQUEST && engineRunning) {
    if (isCompressorOn) {
      // If compressor is currently active, evaluate CUT-OUT boundaries
      if (scaledACPsi >= AC_PRESS_CUTOUT || temperatureFahrenheit <= AC_TEMP_CUTOUT) {
        isCompressorOn = false;
      }
    } else {
      // If compressor is currently inactive, evaluate safe CUT-IN conditions
      if (scaledACPsi < AC_PRESS_CUTIN && temperatureFahrenheit >= AC_TEMP_CUTIN) {
        isCompressorOn = true;
      }
    }
  } else {
    // Force compressor off instantly if there is no user request or if the engine stalls
    isCompressorOn = false;
  }

  // -------------------------------------------------------------------------
  // 3. TRANSMIT TRAFFIC: Sensor Broadcast (10 Hz / 100ms)
  // -------------------------------------------------------------------------
  if (currentTime - lastTxTime >= 100) {
    lastTxTime = currentTime;

    // Build FRAME 1502 Data Pack
    canMsgOut1502.data[0] = (rawTemp1 >> 8) & 0xFF;  
    canMsgOut1502.data[1] = rawTemp1 & 0xFF;         
    canMsgOut1502.data[2] = (rawTemp2 >> 8) & 0xFF;  
    canMsgOut1502.data[3] = rawTemp2 & 0xFF;         
    canMsgOut1502.data[4] = (rawPress1 >> 8) & 0xFF;  
    canMsgOut1502.data[5] = rawPress1 & 0xFF;        
    canMsgOut1502.data[6] = (rawPress2 >> 8) & 0xFF; 
    canMsgOut1502.data[7] = rawPress2 & 0xFF;        
    mcp2515.sendMessage(&canMsgOut1502);

    // Build FRAME 1503 Data Pack (Optimized Payload Allocation)
    // Send exact mapped PSI value scaled x10 over CAN (e.g. 1550 = 155.0 PSI)
    uint16_t transmittedPsiX10 = scaledACPsi * 10;
    canMsgOut1503.data[0] = (transmittedPsiX10 >> 8) & 0xFF; 
    canMsgOut1503.data[1] = transmittedPsiX10 & 0xFF;        
    
    // Byte 2 and 3 mapped for AC temprature
    int16_t txTempx10 = temperatureFahrenheit *10;
	  canMsgOut1503.data[2] = (txTempx10 >> 8) &0xFF;
	  canMsgOut1503.data[3] = txTempx10 & 0xFF;
	
	  // Byte 2 acts as our real-time AC digital state command bit to MegaSquirt
    canMsgOut1503.data[4] = isCompressorOn ? 0x01 : 0x00;

	
    
    // Flatten out remaining unused data array lines efficiently
    for (int i = 5; i < 8; i++) { canMsgOut1503.data[i] = 0x00; }
    mcp2515.sendMessage(&canMsgOut1503);

    // Debugging printouts
    Serial.print("Thermistor Resistance: "); Serial.print(thermistorResistance);
    Serial.print(" Ohms | Temp: ");          Serial.print(temperatureFahrenheit);
    Serial.print(" °F |");
    Serial.print(" AC Request:");            Serial.print(AC_REQUEST ? "YES" : "NO");
    Serial.print(" | COMPRESSOR: ");         Serial.println(isCompressorOn ? "RUNNING" : "OFF");
    Serial.print("RPM: ");                   Serial.print(engineRPM);
    Serial.print(" | MAP: ");                Serial.print((float)engineMAP / 10.0, 1);
    Serial.print(" | TPS: ");                Serial.print((float)engineTPS / 10.0, 1);
    Serial.print(" | VBatt: ");              Serial.println(batteryVoltage /10.0,1);
    Serial.print("AC PSI: ");                Serial.print(scaledACPsi);
    Serial.print(" | ALT STATE: ");          Serial.print(isAltCutActive ? "CUT" : "CHARGING");
    Serial.print(" | Relay_Low: ");          Serial.print(isLowFanON ? "ON" : "OFF");
    Serial.print(" | Relay_High: ");         Serial.println(isHighFanON ? "ON" : "OFF");
  }
}
