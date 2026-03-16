#include "BluetoothSerial.h"
#include <Preferences.h> 

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
Preferences preferences; 

// --- Motor Pins ---
const int PWMA = 18; const int AIN1 = 19; const int AIN2 = 21;
const int PWMB = 23; const int BIN1 = 22; const int BIN2 = 16;

// --- Multiplexer Pins ---
const int S0 = 13;
const int S1 = 12;
const int S2 = 14;
const int S3 = 27;
const int SIG_PIN = 34; // The single analog pin reading the multiplexer

// --- Sensor Setup (Full 8 Sensors!) ---
const int NUM_SENSORS = 8;
const int weights[NUM_SENSORS] = {-40, -30, -20, -10, 10, 20, 30, 40}; 

// --- Auto-Calibration Variables ---
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThresholds[NUM_SENSORS]; 

// --- PID & Dynamic Speed Variables ---
float Kp, Ki, Kd, Kb; 
int baseSpeed; 

float P, I, D = 0;
float previousError = 0;
bool isRunning = false; 

unsigned long lastSendTime = 0; 

// --- Multiplexer Read Function ---
int readMux(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
  
  delayMicroseconds(10); 
  return analogRead(SIG_PIN);
}

// --- Motor Control (REVERSED DIRECTION FIX) ---
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Reversed: Forward is now LOW/HIGH
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, leftSpeed);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); 
    analogWrite(PWMA, -leftSpeed);
  }

  // Reversed: Forward is now LOW/HIGH
  if (rightSpeed >= 0) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, rightSpeed);
  } else {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); 
    analogWrite(PWMB, -rightSpeed);
  }
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// --- The Spin Calibration Function ---
void calibrateSensors() {
  SerialBT.println(">>> CALIBRATING: SPINNING NOW <<<");
  
  for(int i=0; i<NUM_SENSORS; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  unsigned long startTime = millis();
  
  setMotors(100, -100); 

  while (millis() - startTime < 2500) { 
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = readMux(i); 
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    delay(10); 
  }
  
  stopMotors();
  
  SerialBT.println("--- New Thresholds Saved ---");
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorThresholds[i] = (sensorMin[i] + sensorMax[i]) / 2;
    String key = "th" + String(i);
    preferences.putInt(key.c_str(), sensorThresholds[i]);
  }
  SerialBT.println(">>> CALIBRATION COMPLETE <<<");
}

// --- Bluetooth Command Parser ---
void handleBluetoothCommands() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim(); 
    command.toUpperCase();

    if (command == "START" || command == "1") {
      isRunning = true;
      SerialBT.println(">>> RACING MODE ACTIVATED <<<");
    } 
    else if (command == "STOP" || command == "0") {
      isRunning = false;
      stopMotors();
      I = 0; 
      SerialBT.println(">>> EMERGENCY BRAKE <<<");
    }
    else if (command == "CAL") {
      calibrateSensors();
    }
    else if (command.startsWith("KP=")) { 
      Kp = command.substring(3).toFloat(); 
      preferences.putFloat("kp", Kp); 
      SerialBT.println("Saved Kp: " + String(Kp)); 
    }
    else if (command.startsWith("KI=")) { 
      Ki = command.substring(3).toFloat(); 
      preferences.putFloat("ki", Ki); 
      SerialBT.println("Saved Ki: " + String(Ki)); 
    }
    else if (command.startsWith("KD=")) { 
      Kd = command.substring(3).toFloat(); 
      preferences.putFloat("kd", Kd); 
      SerialBT.println("Saved Kd: " + String(Kd)); 
    }
    else if (command.startsWith("KB=")) { 
      Kb = command.substring(3).toFloat(); 
      preferences.putFloat("kb", Kb); 
      SerialBT.println("Saved Kb: " + String(Kb)); 
    }
    else if (command.startsWith("SPEED=")) { 
      baseSpeed = command.substring(6).toInt(); 
      preferences.putInt("speed", baseSpeed); 
      SerialBT.println("Saved Speed: " + String(baseSpeed)); 
    }
    else if (command.startsWith("THRESH=")) { 
      int manualThresh = command.substring(7).toInt(); 
      for (int i = 0; i < NUM_SENSORS; i++) {
        sensorThresholds[i] = manualThresh;
        String key = "th" + String(i);
        preferences.putInt(key.c_str(), manualThresh); 
      }
      SerialBT.println(">>> MANUAL OVERRIDE: All thresholds set to " + String(manualThresh) + " <<<"); 
    }
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_LineFollower"); 

  preferences.begin("robot", false); 
  
  Kp = preferences.getFloat("kp", 1.0);
  Ki = preferences.getFloat("ki", 0.0);
  Kd = preferences.getFloat("kd", 0.5);
  Kb = preferences.getFloat("kb", 1.5);
  baseSpeed = preferences.getInt("speed", 130);

  // Load individual thresholds, default to 2000
  for(int i=0; i < NUM_SENSORS; i++) {
    String key = "th" + String(i);
    sensorThresholds[i] = preferences.getInt(key.c_str(), 2000);
  }

  // Motor Pins Setup
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  stopMotors();

  // Multiplexer Pins Setup
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG_PIN, INPUT);
}

// --- Main Loop ---
void loop() {
  handleBluetoothCommands();

  int error = 0;
  int activeSensors = 0;
  String sensorString = ""; 
  String threshString = ""; 

  // 1. Read All 8 Analog Sensors via Multiplexer
  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = readMux(i); 
    
    // Build the strings for the HUD
    sensorString += String(val) + " ";
    threshString += String(sensorThresholds[i]) + " ";

    // Logic Check
    if (val > sensorThresholds[i]) {
      error += weights[i];
      activeSensors++;
    }
  }

  // 2. Average Error
  if (activeSensors > 0) {
    error = error / activeSensors;
  } else {
    // If line is entirely lost, maintain the last known direction
    error = previousError; 
  }

  // 3. Send Compact Live Data HUD
  if (millis() - lastSendTime > 200) {
    SerialBT.println("Val: [ " + sensorString + "]");
    SerialBT.println("Thr: [ " + threshString + "]");
    SerialBT.println("Err: " + String(error) + " | Kp: " + String(Kp, 1) + " | Kd: " + String(Kd, 1) + " | Kb: " + String(Kb, 1) + " | Spd: " + String(baseSpeed));
    SerialBT.println("--------------------------------------------------"); 
    
    lastSendTime = millis();
  }

  // 4. PID & Dynamic Auto-Brake Math
  if (isRunning) {
    P = error;
    I = I + error;
    D = error - previousError;
    
    float pidValue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    int dynamicSpeed = baseSpeed - (abs(error) * Kb);
    dynamicSpeed = constrain(dynamicSpeed, 60, 255); 

    int leftMotorSpeed = dynamicSpeed - pidValue;
    int rightMotorSpeed = dynamicSpeed + pidValue;

    setMotors(leftMotorSpeed, rightMotorSpeed);
  }
}