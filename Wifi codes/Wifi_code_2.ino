#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// --- Motor Pins ---
const int PWMA = 18; const int AIN1 = 19; const int AIN2 = 21;
const int PWMB = 23; const int BIN1 = 22; const int BIN2 = 16;

// --- Sensor Pins ---
const int sensorPins[8] = {36, 39, 34, 25, 26, 35, 32, 33};
const bool isAnalog[8]  = {true, true, true, false, false, true, true, true};
const int weights[8]    = {-40, -30, -20, -10, 10, 20, 30, 40}; 

// --- PID & Dynamic Speed Variables ---
float Kp = 1.0; 
float Ki = 0.5; 
float Kd = 0.5;
float Kb = 1.5; // NEW: The Auto-Brake Multiplier
int baseSpeed = 130; 
int threshold = 2000; 

float P, I, D = 0;
float previousError = 0;
bool isRunning = false; 

unsigned long lastSendTime = 0; 

// --- Motor Control ---
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); 
    analogWrite(PWMA, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, rightSpeed);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); 
    analogWrite(PWMB, -rightSpeed);
  }
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
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
    else if (command.startsWith("KP=")) Kp = command.substring(3).toFloat();
    else if (command.startsWith("KI=")) Ki = command.substring(3).toFloat();
    else if (command.startsWith("KD=")) Kd = command.substring(3).toFloat();
    else if (command.startsWith("KB=")) Kb = command.substring(3).toFloat(); // NEW COMMAND
    else if (command.startsWith("SPEED=")) baseSpeed = command.substring(6).toInt();
    else if (command.startsWith("THRESH=")) threshold = command.substring(7).toInt();
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_LineFollower"); 

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  stopMotors();

  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

// --- Main Loop ---
void loop() {
  handleBluetoothCommands();

  int error = 0;
  int activeSensors = 0;
  String dataString = ""; 

  // 1. Read Sensors
  for (int i = 0; i < 8; i++) {
    bool seesBlackLine = false;

    if (isAnalog[i]) {
      int val = analogRead(sensorPins[i]);
      dataString += String(val) + " ";
      if (val > threshold) seesBlackLine = true;
    } else {
      int val = digitalRead(sensorPins[i]);
      dataString += String(val == HIGH ? "B" : "W") + " "; 
      if (val == HIGH) seesBlackLine = true;
    }

    if (seesBlackLine) {
      error += weights[i];
      activeSensors++;
    }
  }

  // 2. Average Error
  if (activeSensors > 0) {
    error = error / activeSensors;
  } else {
    error = previousError; 
  }

  // 3. Send Compact Live Data HUD
  if (millis() - lastSendTime > 200) {
    // Added KB to your live HUD!
    dataString += "|E:" + String(error) + "|P:" + String(Kp, 1) + "|D:" + String(Kd, 1) + "|B:" + String(Kb, 1); 
    SerialBT.println(dataString);
    lastSendTime = millis();
  }

  // 4. PID & Dynamic Auto-Brake Math
  if (isRunning) {
    P = error;
    I = I + error;
    D = error - previousError;
    
    float pidValue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    // --- THE AUTO-BRAKE LOGIC ---
    // If error is high (sharp turn), subtract speed. If error is 0 (straight), subtract nothing.
    int dynamicSpeed = baseSpeed - (abs(error) * Kb);
    
    // Prevent the motors from stalling completely on sharp turns by keeping a minimum speed of 60
    dynamicSpeed = constrain(dynamicSpeed, 60, 255); 

    int leftMotorSpeed = dynamicSpeed - pidValue;
    int rightMotorSpeed = dynamicSpeed + pidValue;

    setMotors(leftMotorSpeed, rightMotorSpeed);
  }
  
  // Notice the delay is gone! The ESP32 is now running at maximum calculation speed.
}