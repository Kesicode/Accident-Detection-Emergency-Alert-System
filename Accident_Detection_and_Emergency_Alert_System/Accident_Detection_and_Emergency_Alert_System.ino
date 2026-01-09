#include <Wire.h>
#include <SoftwareSerial.h>

// ============================================================================
// ADXL345 REGISTER DEFINITIONS
// ============================================================================
#define ADXL345_ADDRESS 0x53
#define ADXL345_DEVID   0x00     // Device ID register (should read 0xE5)
#define POWER_CTL       0x2D
#define DATA_FORMAT     0x31
#define DATAX0          0x32
#define BW_RATE         0x2C
#define FIFO_CTL        0x38

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define BUZZER 8
#define CANCEL_BUTTON 7
#define RED_LED 6
#define GREEN_LED 5
#define GSM_TX 10
#define GSM_RX 11

// ============================================================================
// DETECTION PARAMETERS
// ============================================================================
#define SAMPLING_RATE_HZ 200
#define SAMPLE_INTERVAL_MS (1000 / SAMPLING_RATE_HZ)
#define IMPACT_THRESHOLD_G 3.5
#define NORMAL_DRIVING_MAX_G 1.5
#define HARD_BRAKE_MAX_G 2.5
#define SEVERE_IMPACT_G 5.0
#define MIN_IMPACT_DURATION_MS 10
#define MAX_IMPACT_DURATION_MS 100
#define POST_IMPACT_STILLNESS_MS 500
#define JERK_THRESHOLD 50.0
#define VARIANCE_THRESHOLD 2.0
#define RMS_THRESHOLD 4.0
#define ROLLOVER_Z_THRESHOLD 0.3
#define ALERT_DELAY 10000
#define BUZZER_FREQUENCY 2000

// ============================================================================
// EMERGENCY CONTACTS
// ============================================================================
String emergencyContacts[] = {
  "+919072052163",
  "+919072052163",
  "+919072052163"
};
int numContacts = 3;
String primaryNumber = "+919072052163";

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
SoftwareSerial gsmSerial(GSM_TX, GSM_RX);

// Accelerometer data
float ax, ay, az;
float ax_prev, ay_prev, az_prev;
float axOffset = 0, ayOffset = 0, azOffset = 0;

// Processed signals
float resultantAccel;
float jerk;
float rmsAccel;
float variance;

// Circular buffer
#define BUFFER_SIZE 50
float accelBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// State machine
enum CrashState {
  MONITORING,
  IMPACT_DETECTED,
  CRASH_CONFIRMED,
  ALERT_COUNTDOWN,
  ALERT_SENT
};
CrashState crashState = MONITORING;

// Timing
unsigned long lastSampleTime = 0;
unsigned long impactStartTime = 0;
unsigned long impactDuration = 0;
unsigned long stillnessStartTime = 0;
unsigned long alertStartTime = 0;
unsigned long lastStatusTime = 0;

// Crash data
float maxImpact = 0;
float maxJerk = 0;
float crashRMS = 0;
float crashVariance = 0;
bool rolloverDetected = false;

// Statistics
int totalSamples = 0;
int impactEvents = 0;
int confirmedCrashes = 0;
int falsePositives = 0;
bool systemReady = false;
bool gsmReady = false;
bool accelReady = false;

// Serial command buffer
String serialCommand = "";

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  gsmSerial.begin(9600);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(CANCEL_BUTTON, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  
  Serial.println(F("\n============================================"));
  Serial.println(F("  ACCIDENT DETECTION SYSTEM v3.2"));
  Serial.println(F("  Fixed ADXL345 & SMS Implementation"));
  Serial.println(F("============================================\n"));
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000);  // Start with 100kHz for reliability
  delay(100);
  
  // Initialize ADXL345 with validation
  Serial.println(F("Initializing ADXL345..."));
  if (initADXL345()) {
    accelReady = true;
    Serial.println(F("✓ ADXL345 initialization successful"));
    
    // Calibrate
    calibrateAccelerometer();
    
    // Initialize buffer
    for (int i = 0; i < BUFFER_SIZE; i++) {
      accelBuffer[i] = 1.0;
    }
  } else {
    Serial.println(F("✗ ADXL345 initialization FAILED"));
    Serial.println(F("\nTroubleshooting steps:"));
    Serial.println(F("1. Check wiring (VCC=3.3V, GND, SDA=A4, SCL=A5)"));
    Serial.println(F("2. Verify SDO is connected to GND"));
    Serial.println(F("3. Type 'SCAN' to check I2C bus"));
    Serial.println(F("4. Try adding 4.7kΩ pull-up resistors"));
    errorBlink();
    // Don't halt - continue so user can use diagnostic commands
  }
  
  // Initialize GSM
  Serial.println(F("\nInitializing SIM800L..."));
  if (initGSM()) {
    gsmReady = true;
    Serial.println(F("✓ GSM module ready"));
  } else {
    Serial.println(F("⚠ GSM initialization failed"));
    Serial.println(F("  System will monitor without alerts"));
  }
  
  systemReady = accelReady;
  
  if (systemReady) {
    digitalWrite(GREEN_LED, HIGH);
    playStartupTone();
    
    Serial.println(F("\n============================================"));
    Serial.println(F("  SYSTEM READY - MONITORING ACTIVE"));
    Serial.println(F("============================================"));
    Serial.print(F("Sampling Rate: "));
    Serial.print(SAMPLING_RATE_HZ);
    Serial.println(F(" Hz"));
    Serial.print(F("Impact Threshold: "));
    Serial.print(IMPACT_THRESHOLD_G);
    Serial.println(F("g"));
    Serial.print(F("GSM Status: "));
    Serial.println(gsmReady ? "READY" : "NOT READY");
  } else {
    digitalWrite(RED_LED, HIGH);
    Serial.println(F("\n⚠ SYSTEM NOT READY - ACCELEROMETER FAILED"));
    Serial.println(F("Use diagnostic commands to troubleshoot"));
  }
  
  Serial.println(F("\nAvailable Commands:"));
  Serial.println(F("  TEST, STATUS, SIMULATE, SCAN, READ, HELP"));
  Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  // Check serial commands
  checkSerialCommands();
  
  // Only run detection if accelerometer is working
  if (accelReady && systemReady) {
    // High-frequency sampling
    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
      lastSampleTime = currentTime;
      
      ax_prev = ax;
      ay_prev = ay;
      az_prev = az;
      
      if (readAccelerometer()) {
        totalSamples++;
        
        // Apply calibration offsets
        ax -= axOffset;
        ay -= ayOffset;
        az -= azOffset;
        
        resultantAccel = sqrt(ax * ax + ay * ay + az * az);
        
        float dt = SAMPLE_INTERVAL_MS / 1000.0;
        jerk = calculateJerk(dt);
        
        accelBuffer[bufferIndex] = resultantAccel;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        
        processCrashDetection(currentTime);
        
        // Status every 2 seconds
        if (currentTime - lastStatusTime >= 2000 && crashState == MONITORING) {
          lastStatusTime = currentTime;
          displayStatus();
        }
      }
    }
  }
  
  // Handle alert countdown
  if (crashState == ALERT_COUNTDOWN) {
    handleAlertCountdown();
  }
}

// ============================================================================
// ADXL345 INITIALIZATION WITH VALIDATION
// ============================================================================
bool initADXL345() {
  // Test I2C communication
  Wire.beginTransmission(ADXL345_ADDRESS);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print(F("  ✗ I2C communication failed (error "));
    Serial.print(error);
    Serial.println(F(")"));
    return false;
  }
  
  // Read and verify device ID
  byte devId = readRegister(ADXL345_DEVID);
  Serial.print(F("  Device ID: 0x"));
  Serial.print(devId, HEX);
  
  if (devId != 0xE5) {
    Serial.println(F(" - INVALID!"));
    return false;
  }
  Serial.println(F(" - Valid ADXL345"));
  
  // Configure device
  writeRegister(POWER_CTL, 0x00);     // Standby mode first
  delay(10);
  
  writeRegister(BW_RATE, 0x0B);       // 200Hz data rate
  writeRegister(DATA_FORMAT, 0x0B);   // Full resolution, ±16g
  writeRegister(FIFO_CTL, 0x00);      // Bypass FIFO
  writeRegister(POWER_CTL, 0x08);     // Measurement mode
  
  delay(100);
  
  // Verify we can read data
  int16_t testX, testY, testZ;
  if (!readRawAccel(&testX, &testY, &testZ)) {
    Serial.println(F("  ✗ Cannot read accelerometer data"));
    return false;
  }
  
  Serial.println(F("  ✓ Configuration successful"));
  Serial.println(F("  ✓ Range: ±16g, Rate: 200Hz"));
  
  return true;
}

// ============================================================================
// READ/WRITE REGISTER FUNCTIONS
// ============================================================================
byte readRegister(byte reg) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDRESS, (byte)1);
  
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// ============================================================================
// READ RAW ACCELEROMETER DATA
// ============================================================================
bool readRawAccel(int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(DATAX0);
  byte error = Wire.endTransmission(false);
  
  if (error != 0) return false;
  
  Wire.requestFrom(ADXL345_ADDRESS, 6);
  
  if (Wire.available() >= 6) {
    *x = Wire.read() | (Wire.read() << 8);
    *y = Wire.read() | (Wire.read() << 8);
    *z = Wire.read() | (Wire.read() << 8);
    return true;
  }
  
  return false;
}

// ============================================================================
// READ ACCELEROMETER (WITH ERROR CHECKING)
// ============================================================================
bool readAccelerometer() {
  int16_t x, y, z;
  
  if (readRawAccel(&x, &y, &z)) {
    // Convert to g (256 LSB/g at full resolution)
    ax = x / 256.0;
    ay = y / 256.0;
    az = z / 256.0;
    return true;
  }
  
  return false;
}

// ============================================================================
// CALIBRATE ACCELEROMETER
// ============================================================================
void calibrateAccelerometer() {
  Serial.println(F("\nCalibrating..."));
  Serial.println(F("Keep device flat and still for 3 seconds"));
  
  // Visual countdown
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    Serial.print(F("... "));
    delay(1000);
  }
  Serial.println(F("Sampling..."));
  
  float xSum = 0, ySum = 0, zSum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < 100; i++) {
    if (readAccelerometer()) {
      xSum += ax;
      ySum += ay;
      zSum += az;
      validSamples++;
    }
    delay(10);
  }
  
  if (validSamples > 50) {
    axOffset = xSum / validSamples;
    ayOffset = ySum / validSamples;
    azOffset = (zSum / validSamples) - 1.0;  // Remove 1g gravity
    
    Serial.println(F("✓ Calibration complete"));
    Serial.print(F("  Offsets: X="));
    Serial.print(axOffset, 3);
    Serial.print(F("g Y="));
    Serial.print(ayOffset, 3);
    Serial.print(F("g Z="));
    Serial.print(azOffset, 3);
    Serial.println(F("g"));
  } else {
    Serial.println(F("⚠ Calibration incomplete - not enough valid samples"));
  }
}

// ============================================================================
// SERIAL COMMAND HANDLING
// ============================================================================
void checkSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialCommand.length() > 0) {
        processCommand(serialCommand);
        serialCommand = "";
      }
    } else {
      serialCommand += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  Serial.print(F("\n> "));
  Serial.println(cmd);
  
  if (cmd == "TEST") {
    sendTestSMS();
  } 
  else if (cmd == "STATUS") {
    sendStatusSMS();
  }
  else if (cmd == "SIMULATE") {
    simulateAccident();
  }
  else if (cmd == "SCAN") {
    scanI2C();
  }
  else if (cmd == "READ") {
    readRawValues();
  }
  else if (cmd == "HELP") {
    showHelp();
  }
  else {
    Serial.println(F("Unknown command. Type HELP for list."));
  }
}

// ============================================================================
// DIAGNOSTIC: I2C SCANNER
// ============================================================================
void scanI2C() {
  Serial.println(F("\nScanning I2C bus (0x01-0x7F)..."));
  int found = 0;
  
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(F("  ✓ Device at 0x"));
      if (addr < 16) Serial.print(F("0"));
      Serial.print(addr, HEX);
      
      if (addr == 0x53 || addr == 0x1D) {
        Serial.print(F(" <- Possible ADXL345"));
      }
      Serial.println();
      found++;
    }
  }
  
  if (found == 0) {
    Serial.println(F("  ✗ No devices found"));
    Serial.println(F("\n  Check:"));
    Serial.println(F("  - Wiring (SDA=A4, SCL=A5)"));
    Serial.println(F("  - Power (VCC=3.3V)"));
    Serial.println(F("  - Pull-up resistors (4.7kΩ)"));
  } else {
    Serial.print(F("\nTotal: "));
    Serial.print(found);
    Serial.println(F(" device(s)"));
  }
}

// ============================================================================
// DIAGNOSTIC: READ RAW VALUES
// ============================================================================
void readRawValues() {
  Serial.println(F("\nReading raw accelerometer..."));
  
  int16_t rawX, rawY, rawZ;
  if (readRawAccel(&rawX, &rawY, &rawZ)) {
    Serial.print(F("  Raw: X="));
    Serial.print(rawX);
    Serial.print(F(" Y="));
    Serial.print(rawY);
    Serial.print(F(" Z="));
    Serial.println(rawZ);
    
    float gX = rawX / 256.0;
    float gY = rawY / 256.0;
    float gZ = rawZ / 256.0;
    
    Serial.print(F("  g:   X="));
    Serial.print(gX, 3);
    Serial.print(F(" Y="));
    Serial.print(gY, 3);
    Serial.print(F(" Z="));
    Serial.println(gZ, 3);
    
    Serial.println(F("✓ Accelerometer is working!"));
  } else {
    Serial.println(F("✗ Failed to read data"));
  }
}

// ============================================================================
// SMS FUNCTIONS
// ============================================================================
void sendTestSMS() {
  if (!gsmReady) {
    Serial.println(F("✗ GSM not ready"));
    return;
  }
  
  Serial.println(F("\n📱 Sending test SMS..."));
  
  String msg = "TEST: Accident Detection System v3.2\n";
  msg += "Status: OPERATIONAL\n";
  msg += "GSM: CONNECTED\n";
  msg += "Accel: " + String(accelReady ? "OK" : "FAILED") + "\n";
  msg += "Uptime: " + String(millis() / 1000) + "s";
  
  if (sendSMS(primaryNumber, msg)) {
    Serial.println(F("✓ Test SMS sent"));
    tone(BUZZER, 1500, 200);
  } else {
    Serial.println(F("✗ SMS failed"));
    tone(BUZZER, 500, 500);
  }
}

void sendStatusSMS() {
  if (!gsmReady) {
    Serial.println(F("✗ GSM not ready"));
    return;
  }
  
  Serial.println(F("\n📊 Sending status..."));
  
  String msg = "STATUS REPORT\n";
  msg += "Uptime: " + String(millis() / 1000) + "s\n";
  msg += "Accel: " + String(resultantAccel, 2) + "g\n";
  msg += "Samples: " + String(totalSamples) + "\n";
  msg += "Events: " + String(impactEvents) + "\n";
  msg += "Crashes: " + String(confirmedCrashes) + "\n";
  msg += "False+: " + String(falsePositives);
  
  if (sendSMS(primaryNumber, msg)) {
    Serial.println(F("✓ Status sent"));
    tone(BUZZER, 1500, 200);
  } else {
    Serial.println(F("✗ SMS failed"));
  }
}

// ============================================================================
// GSM INITIALIZATION (IMPROVED)
// ============================================================================
bool initGSM() {
  Serial.println(F("  Waiting for SIM800L (5s)..."));
  delay(5000);
  
  // Test AT
  for (int i = 0; i < 3; i++) {
    gsmSerial.println(F("AT"));
    if (waitForResponse(2000, "OK")) {
      // Set text mode
      gsmSerial.println(F("AT+CMGF=1"));
      waitForResponse(1000, "OK");
      return true;
    }
    delay(1000);
  }
  
  return false;
}

bool sendSMS(String number, String message) {
  gsmSerial.println("AT+CMGS=\"" + number + "\"");
  delay(500);
  
  gsmSerial.print(message);
  delay(500);
  
  gsmSerial.write(26);  // Ctrl+Z
  
  return waitForResponse(10000, "OK");
}

bool waitForResponse(unsigned long timeout, String expected) {
  unsigned long start = millis();
  String response = "";
  
  while (millis() - start < timeout) {
    if (gsmSerial.available()) {
      char c = gsmSerial.read();
      response += c;
      if (response.indexOf(expected) != -1) {
        return true;
      }
    }
  }
  return false;
}

// ============================================================================
// CRASH DETECTION ALGORITHM
// ============================================================================
void processCrashDetection(unsigned long currentTime) {
  switch (crashState) {
    case MONITORING:
      if (resultantAccel > IMPACT_THRESHOLD_G) {
        crashState = IMPACT_DETECTED;
        impactStartTime = currentTime;
        maxImpact = resultantAccel;
        maxJerk = jerk;
        impactEvents++;
        
        Serial.println(F("\n🔴 IMPACT DETECTED"));
        Serial.print(F("   "));
        Serial.print(resultantAccel, 2);
        Serial.println(F("g"));
      }
      checkRollover(currentTime);
      break;
    
    case IMPACT_DETECTED:
      impactDuration = currentTime - impactStartTime;
      
      if (resultantAccel > maxImpact) maxImpact = resultantAccel;
      if (jerk > maxJerk) maxJerk = jerk;
      
      if (impactDuration >= MIN_IMPACT_DURATION_MS && 
          impactDuration <= MAX_IMPACT_DURATION_MS) {
        
        variance = calculateVariance();
        rmsAccel = calculateRMS();
        
        bool highImpact = (maxImpact > IMPACT_THRESHOLD_G);
        bool highJerk = (maxJerk > JERK_THRESHOLD);
        bool chaotic = (variance > VARIANCE_THRESHOLD);
        bool highEnergy = (rmsAccel > RMS_THRESHOLD);
        
        int conditions = highImpact + highJerk + chaotic + highEnergy;
        
        if (conditions >= 3) {
          crashState = CRASH_CONFIRMED;
          stillnessStartTime = currentTime;
          crashRMS = rmsAccel;
          crashVariance = variance;
          
          Serial.println(F("🚨 CRASH CONFIRMED"));
          Serial.print(F("   Conditions: "));
          Serial.print(conditions);
          Serial.println(F("/4"));
        }
      }
      
      if (impactDuration > MAX_IMPACT_DURATION_MS) {
        resetToMonitoring();
      }
      break;
    
    case CRASH_CONFIRMED:
      if (resultantAccel < NORMAL_DRIVING_MAX_G) {
        unsigned long stillness = currentTime - stillnessStartTime;
        
        if (stillness >= POST_IMPACT_STILLNESS_MS) {
          crashState = ALERT_COUNTDOWN;
          alertStartTime = currentTime;
          confirmedCrashes++;
          
          digitalWrite(RED_LED, HIGH);
          digitalWrite(GREEN_LED, LOW);
          
          Serial.println(F("\n*** ACCIDENT CONFIRMED ***"));
          printCrashReport();
        }
      } else {
        stillnessStartTime = currentTime;
      }
      
      if (currentTime - stillnessStartTime > 2000) {
        resetToMonitoring();
      }
      break;
    
    case ALERT_SENT:
      // Stay here until manual reset
      break;
  }
}

// ============================================================================
// ALERT COUNTDOWN (FIXED TIMING)
// ============================================================================
void handleAlertCountdown() {
  static unsigned long lastBeep = 0;
  static bool beepState = false;
  
  unsigned long elapsed = millis() - alertStartTime;
  unsigned long remaining = (ALERT_DELAY - elapsed) / 1000 + 1;
  
  // Check cancellation
  if (digitalRead(CANCEL_BUTTON) == LOW) {
    cancelAlert();
    return;
  }
  
  // Beep pattern (non-blocking)
  if (elapsed < ALERT_DELAY) {
    if (millis() - lastBeep >= 200) {
      lastBeep = millis();
      beepState = !beepState;
      
      if (beepState) {
        tone(BUZZER, BUZZER_FREQUENCY);
      } else {
        noTone(BUZZER);
        
        // Print countdown
        Serial.print(F("⏱  "));
        Serial.print(remaining);
        Serial.println(F("s [Press button to CANCEL]"));
      }
    }
  }
  
  // Send alert
  if (elapsed >= ALERT_DELAY) {
    noTone(BUZZER);
    sendEmergencyAlert();
    crashState = ALERT_SENT;
  }
}

void cancelAlert() {
  Serial.println(F("\n*** ALERT CANCELLED ***"));
  crashState = MONITORING;
  noTone(BUZZER);
  
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  
  tone(BUZZER, 1000, 200);
  delay(250);
  tone(BUZZER, 800, 200);
}

// ============================================================================
// SEND EMERGENCY ALERT
// ============================================================================
void sendEmergencyAlert() {
  Serial.println(F("\n📡 SENDING EMERGENCY ALERTS"));
  
  String msg = "EMERGENCY! ACCIDENT DETECTED\n\n";
  msg += "Impact: " + String(maxImpact, 1) + "g\n";
  msg += "Jerk: " + String(maxJerk, 0) + " g/s\n";
  msg += "Energy: " + String(crashRMS, 1) + "g\n";
  msg += "Rollover: " + String(rolloverDetected ? "YES" : "NO") + "\n\n";
  msg += "SEND HELP IMMEDIATELY!";
  
  for (int i = 0; i < numContacts; i++) {
    Serial.print(F("Sending to: "));
    Serial.println(emergencyContacts[i]);
    sendSMS(emergencyContacts[i], msg);
    delay(3000);
  }
  
  for (int i = 0; i < 5; i++) {
    tone(BUZZER, 3000, 100);
    delay(150);
  }
  
  Serial.println(F("✓ Alerts sent\n"));
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
float calculateJerk(float dt) {
  float dax = (ax - ax_prev) / dt;
  float day = (ay - ay_prev) / dt;
  float daz = (az - az_prev) / dt;
  return sqrt(dax * dax + day * day + daz * daz);
}

float calculateVariance() {
  float mean = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) mean += accelBuffer[i];
  mean /= BUFFER_SIZE;
  
  float var = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    float diff = accelBuffer[i] - mean;
    var += diff * diff;
  }
  return var / BUFFER_SIZE;
}

float calculateRMS() {
  float sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += accelBuffer[i] * accelBuffer[i];
  }
  return sqrt(sum / BUFFER_SIZE);
}

void checkRollover(unsigned long currentTime) {
  if (abs(az) < ROLLOVER_Z_THRESHOLD) {
    if (!rolloverDetected) {
      rolloverDetected = true;
      Serial.println(F("\n⚠ ROLLOVER DETECTED"));
      crashState = CRASH_CONFIRMED;
      stillnessStartTime = currentTime;
      maxImpact = SEVERE_IMPACT_G;
    }
  } else {
    rolloverDetected = false;
  }
}

void resetToMonitoring() {
  crashState = MONITORING;
  impactDuration = 0;
  maxImpact = 0;
  maxJerk = 0;
  falsePositives++;
  Serial.println(F("   False positive - resuming\n"));
}

void simulateAccident() {
  Serial.println(F("\n⚠ SIMULATING ACCIDENT"));
  
  maxImpact = 4.5;
  maxJerk = 75.0;
  crashRMS = 4.8;
  crashVariance = 3.2;
  rolloverDetected = false;
  impactDuration = 50;
  
  crashState = ALERT_COUNTDOWN;
  alertStartTime = millis();
  confirmedCrashes++;
  
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  
  printCrashReport();
}

void printCrashReport() {
  Serial.println(F("╔════════════════════════════════╗"));
  Serial.println(F("║      CRASH EVENT REPORT        ║"));
  Serial.println(F("╠════════════════════════════════╣"));
  Serial.print(F("║ Impact:    "));
  Serial.print(maxImpact, 2);
  Serial.println(F(" g"));
  Serial.print(F("║ Jerk:      "));
  Serial.print(maxJerk, 1);
  Serial.println(F(" g/s"));
  Serial.print(F("║ RMS:       "));
  Serial.print(crashRMS, 2);
  Serial.println(F(" g"));
  Serial.print(F("║ Variance:  "));
  Serial.println(crashVariance, 2);
  Serial.print(F("║ Duration:  "));
  Serial.print(impactDuration);
  Serial.println(F(" ms"));
  Serial.print(F("║ Rollover:  "));
  Serial.println(rolloverDetected ? "YES" : "NO");
  Serial.println(F("╚════════════════════════════════╝"));
}

void displayStatus() {
  Serial.print(F("📊 "));
  Serial.print(resultantAccel, 2);
  Serial.print(F("g | J="));
  Serial.print(jerk, 1);
  Serial.print(F(" | S="));
  Serial.print(totalSamples);
  Serial.print(F(" | E="));
  Serial.print(impactEvents);
  Serial.print(F(" | FP="));
  Serial.println(falsePositives);
}

void showHelp() {
  Serial.println(F("\n╔══════════════════════════════════╗"));
  Serial.println(F("║      AVAILABLE COMMANDS          ║"));
  Serial.println(F("╠══════════════════════════════════╣"));
  Serial.println(F("║ TEST     - Send test SMS         ║"));
  Serial.println(F("║ STATUS   - Send status SMS       ║"));
  Serial.println(F("║ SIMULATE - Simulate accident     ║"));
  Serial.println(F("║ SCAN     - Scan I2C bus          ║"));
  Serial.println(F("║ READ     - Read raw accel data   ║"));
  Serial.println(F("║ HELP     - Show this message     ║"));
  Serial.println(F("╚══════════════════════════════════╝"));
}

void errorBlink() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(RED_LED, HIGH);
    tone(BUZZER, 2000, 100);
    delay(200);
    digitalWrite(RED_LED, LOW);
    delay(200);
  }
}

void playStartupTone() {
  tone(BUZZER, 1000, 100);
  delay(150);
  tone(BUZZER, 1500, 100);
  delay(150);
  tone(BUZZER, 2000, 100);
}