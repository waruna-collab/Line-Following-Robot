/*
 * ESP32 PID Line Follower Robot Code
 * Driver: TB6612FNG
 * Sensors: 8-Channel Analog Array (QTR-8A style)
 * * Pinout:
 * - Motors: AIN1(12), AIN2(14), PWMA(13), BIN1(26), BIN2(25), PWMB(27)
 * - Sensors: 36, 39, 34, 35, 32, 33, 4, 15
 * - Buttons: 23, 22, 21, 19
 */

#include <Arduino.h>

// --- PIN DEFINITIONS ---
// Motor Driver (TB6612FNG)
#define PWMA 13  // Left Motor Speed
#define AIN1 12  // Left Motor Dir 1
#define AIN2 14  // Left Motor Dir 2
#define PWMB 27  // Right Motor Speed
#define BIN1 26  // Right Motor Dir 1
#define BIN2 25  // Right Motor Dir 2

// Sensors (8 Channels)
// Note: We use specific ADC1 and ADC2 pins based on your diagram
const int sensorPins[8] = {36, 39, 34, 35, 32, 33, 4, 15};
int sensorValues[8];
int sensorMin[8];
int sensorMax[8];

// Buttons (Active Low)
#define BTN_CALIBRATE 23
#define BTN_START     22
#define BTN_STOP      21
#define BTN_TEST      19

// LEDs
#define LED_RED   18
#define LED_GREEN 5

// --- PID CONSTANTS (YOU MUST TUNE THESE!) ---
float Kp = 0.08;   // Proportional (Start with 0.05 - 0.1)
float Ki = 0.0001; // Integral (Keep very small)
float Kd = 1.0;    // Derivative (Start with 1.0 - 5.0)

// PID Variables
int P, I, D;
int lastError = 0;
int error = 0;

// Motor Speed Settings (0 - 255)
const int baseSpeed = 150; // Base speed for going straight
const int maxSpeed = 220;  // Maximum speed limit
const int turnSpeed = 100; // Speed for calibration turning

// PWM Properties for ESP32
const int freq = 5000;
const int resolution = 8;
const int pwmChannelA = 0;
const int pwmChannelB = 1;

// State Machine
bool isRunning = false;

void setup() {
  Serial.begin(115200);

  // Setup Motor Pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  // Setup PWM for ESP32
  ledcSetup(pwmChannelA, freq, resolution);
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(PWMA, pwmChannelA);
  ledcAttachPin(PWMB, pwmChannelB);

  // Setup Buttons
  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(BTN_TEST, INPUT_PULLUP);

  // Setup LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Initialize Sensor Calibration Arrays
  for (int i = 0; i < 8; i++) {
    sensorMin[i] = 4095; // Max possible value
    sensorMax[i] = 0;    // Min possible value
    pinMode(sensorPins[i], INPUT);
  }

  digitalWrite(LED_RED, HIGH); // Power ON indicator
}

void loop() {
  // --- BUTTON HANDLERS ---
  
  if (digitalRead(BTN_CALIBRATE) == LOW) {
    delay(200); // Debounce
    calibrateSensors();
  }

  if (digitalRead(BTN_START) == LOW) {
    delay(200);
    isRunning = true;
    digitalWrite(LED_GREEN, HIGH);
  }

  if (digitalRead(BTN_STOP) == LOW) {
    delay(200);
    stopMotors();
    isRunning = false;
    digitalWrite(LED_GREEN, LOW);
  }

  // --- MAIN RUN LOOP ---
  if (isRunning) {
    runPID();
  }
}

// --- CALIBRATION ROUTINE ---
void calibrateSensors() {
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);
  Serial.println("Calibrating...");

  // Rotate robot back and forth to scan the line
  for (int i = 0; i < 100; i++) {
    if (i < 25 || i >= 75) setMotorSpeed(turnSpeed, -turnSpeed); // Turn Right
    else setMotorSpeed(-turnSpeed, turnSpeed); // Turn Left

    // Record Min/Max
    for (int j = 0; j < 8; j++) {
      int val = analogRead(sensorPins[j]);
      if (val < sensorMin[j]) sensorMin[j] = val;
      if (val > sensorMax[j]) sensorMax[j] = val;
    }
    delay(20);
  }
  
  stopMotors();
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  delay(500);
  digitalWrite(LED_GREEN, LOW);
  Serial.println("Calibration Done.");
}

// --- SENSOR READING & POSITION CALCULATION ---
int readLinePosition() {
  long weightedSum = 0;
  long sum = 0;
  bool onLine = false;

  for (int i = 0; i < 8; i++) {
    // Read raw value
    int val = analogRead(sensorPins[i]);
    
    // Map to 0-1000 based on calibration
    // (Value - Min) * 1000 / (Max - Min)
    int denominator = sensorMax[i] - sensorMin[i];
    if (denominator == 0) denominator = 1; // Avoid divide by zero
    
    int calibratedVal = constrain(((long)(val - sensorMin[i]) * 1000 / denominator), 0, 1000);
    
    // Invert if line is black on white (Assuming high value = black line)
    // If your sensors read LOW for black, remove this line or change logic.
    // Usually IR sensors read HIGH (4095) on Black and LOW on White.
    
    if (calibratedVal > 200) onLine = true; // Threshold check

    weightedSum += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;
  }

  if (sum == 0) return lastError > 0 ? 7000 : 0; // If lost line, use last known side

  int position = weightedSum / sum;
  
  // Position ranges from 0 (Left sensor) to 7000 (Right sensor)
  // Center is 3500
  return position;
}

// --- PID CONTROL LOOP ---
void runPID() {
  int position = readLinePosition();
  
  // Calculate Error (Setpoint is 3500, the center of 8 sensors)
  error = position - 3500;

  // PID Calculations
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;

  // Calculate Correction
  int correction = (Kp * P) + (Ki * I) + (Kd * D);

  // Apply to motors
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  setMotorSpeed(leftSpeed, rightSpeed);
}

// --- MOTOR CONTROL HELPER ---
void setMotorSpeed(int speedL, int speedR) {
  // Left Motor Logic
  if (speedL >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speedL = -speedL;
  }

  // Right Motor Logic
  if (speedR >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speedR = -speedR;
  }

  // Send PWM
  ledcWrite(pwmChannelA, speedL);
  ledcWrite(pwmChannelB, speedR);
}

void stopMotors() {
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}