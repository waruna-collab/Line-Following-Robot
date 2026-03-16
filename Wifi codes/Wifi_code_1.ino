#include <WiFi.h>
#include <WebServer.h>

// --- Wi-Fi Credentials ---
const char* ssid = "Robo_Setup";
const char* password = "password123";
WebServer server(80);

// --- Motor Pins (TB6612FNG) ---
const int PWMA = 14; const int AIN1 = 27; const int AIN2 = 26;
const int PWMB = 32; const int BIN1 = 25; const int BIN2 = 33;

// --- Sensor Pins (From Logic Converter A-Pins) ---
const int sensorPins[8] = {13, 4, 16, 17, 18, 19, 22, 23};
// Sensor weights: negative for left, positive for right
const int weights[8] = {-40, -30, -20, -10, 10, 20, 30, 40}; 

// --- PID & Control Variables ---
float Kp = 1.0; 
float Ki = 0.0; 
float Kd = 0.5;
int baseSpeed = 120; // Safe starting speed

float P, I, D = 0;
float previousError = 0;
bool isRunning = false; // Robot starts in STOP mode

// --- Web Page HTML (Unchanged) ---
String getHTML() {
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:sans-serif; text-align:center; background:#222; color:#fff;} ";
  html += "input{width:80px; font-size:18px; padding:5px; margin:5px; text-align:center;} ";
  html += ".btn{font-size:20px; padding:10px 20px; margin:10px; border-radius:10px; border:none; color:white; font-weight:bold; cursor:pointer;} ";
  html += ".start{background:#28a745;} .stop{background:#dc3545;} .update{background:#007bff; width:100%;} ";
  html += ".card{background:#333; padding:20px; border-radius:10px; margin:10px auto; max-width:350px;} </style></head><body>";
  
  html += "<h2>PID Tuning Interface</h2>";
  html += "<div class='card'>";
  html += "<button class='btn start' onclick='fetch(\"/start\")'>START ROBOT</button>";
  html += "<button class='btn stop' onclick='fetch(\"/stop\")'>EMERGENCY STOP</button>";
  html += "</div>";

  html += "<div class='card'><form action='/update' method='GET'>";
  html += "Kp: <input type='number' step='0.01' name='kp' value='" + String(Kp) + "'><br>";
  html += "Ki: <input type='number' step='0.01' name='ki' value='" + String(Ki) + "'><br>";
  html += "Kd: <input type='number' step='0.01' name='kd' value='" + String(Kd) + "'><br>";
  html += "Base Speed: <input type='number' name='speed' value='" + String(baseSpeed) + "'><br><br>";
  html += "<input type='submit' class='btn update' value='UPDATE PID'>";
  html += "</form></div>";
  
  html += "</body></html>";
  return html;
}

// --- Web Handlers ---
void handleRoot() { server.send(200, "text/html", getHTML()); }
void handleStart() { isRunning = true; server.send(200, "text/plain", "Started"); }
void handleStop() { isRunning = false; stopMotors(); server.send(200, "text/plain", "Stopped"); }

void handleUpdate() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  if (server.hasArg("speed")) baseSpeed = server.arg("speed").toInt();
  
  I = 0; // Reset integral windup on update
  server.sendHeader("Location", "/"); 
  server.send(303);
}

// --- Motor Control Functions ---
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

// --- Setup ---
void setup() {
  Serial.begin(115200); // Start Serial Monitor for debugging

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  stopMotors();

  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  WiFi.softAP(ssid, password);
  
  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/update", handleUpdate);
  server.begin();

  Serial.println("Robot Ready! Connect to 'Robo_Setup' and go to 192.168.4.1");
}

// --- Main Loop ---
void loop() {
  server.handleClient(); 

  int error = 0;
  int activeSensors = 0;

  // 1. Read the line position and print to Serial Monitor
  Serial.print("Sensors: ");
  for (int i = 0; i < 8; i++) {
    int val = digitalRead(sensorPins[i]);
    Serial.print(val);
    Serial.print(" ");
    
    // BLACK LINE LOGIC: Assuming LOW (0) means black line detected.
    if (val == LOW) {
      error += weights[i];
      activeSensors++;
    }
  }

  // 2. Calculate average error
  if (activeSensors > 0) {
    error = error / activeSensors;
  } else {
    error = previousError; // Remember last known position if line is lost
  }

  // Print the calculated error to the Serial Monitor
  Serial.print(" | Error: ");
  Serial.println(error);

  // 3. PID Math & Motor Control (Only runs if START was pressed on phone)
  if (isRunning) {
    P = error;
    I = I + error;
    D = error - previousError;
    
    float pidValue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    int leftMotorSpeed = baseSpeed + pidValue;
    int rightMotorSpeed = baseSpeed - pidValue;

    setMotors(leftMotorSpeed, rightMotorSpeed);
  }
  
  // Small delay so the Serial Monitor doesn't scroll too fast to read
  delay(50); 
}