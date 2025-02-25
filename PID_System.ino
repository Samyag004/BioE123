#include <Arduino.h>

/** Pins & Constants **/
const int MOTOR_PIN = 3;       // PWM to motor driver
const int COMP_PIN  = 2;       // Comparator output (photodiode)
const float MAX_RPM = 1500.0;  // Safety limit
const int MAX_SEC   = 600;     // 10 minutes

/** PID Settings (tune these) **/
float Kp = 0.2, Ki = 0.05, Kd = 0.1; 
float setRPM = 0, currentRPM = 0;
float errorSum = 0, lastError = 0;
unsigned long lastPIDTime = 0, SAMPLE_MS = 200; // PID update every 200 ms

/** Pulse Counting **/
volatile unsigned long pulseCount = 0;

/** Duration Control **/
unsigned long runDuration = 0, startTime = 0;
bool running = false;

/** Interrupt: Count pulses on CHANGE **/
void ISR_countPulses() { pulseCount++; }

/** Setup **/
void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(COMP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COMP_PIN), ISR_countPulses, CHANGE);

  // Prompt user
  Serial.println("Enter target RPM (<=1500):");
  while (!Serial.available()) {}
  setRPM = Serial.parseFloat();
  if (setRPM > MAX_RPM) setRPM = MAX_RPM;

  Serial.println("Enter run time in seconds (<=600):");
  while (!Serial.available()) {}
  runDuration = (unsigned long)Serial.parseInt();
  if (runDuration > MAX_SEC) runDuration = MAX_SEC;
  runDuration *= 1000;

  startTime = millis();
  running = true;
  Serial.println("Starting...");
}

/** Loop: Run PID & Check Time **/
void loop() {
  if (!running) return;
  
  // Stop if time is up
  if (millis() - startTime >= runDuration) {
    analogWrite(MOTOR_PIN, 0);
    running = false;
    Serial.println("Done.");
    return;
  }

  // PID update every SAMPLE_MS
  if (millis() - lastPIDTime >= SAMPLE_MS) {
    noInterrupts();
    unsigned long pulses = pulseCount; 
    pulseCount = 0;
    interrupts();

    // Calculate currentRPM: pulses/sec → pulses/(SAMPLE_MS/1000) → pulses * (1000/SAMPLE_MS)
    // If 1 pulse per rev, RPM = pps * 60
    float pps = pulses * (1000.0 / SAMPLE_MS);
    currentRPM = pps * 60.0; // Adjust if >1 pulse/rev

    // PID computations
    float error = setRPM - currentRPM;
    errorSum += error * (SAMPLE_MS / 1000.0);
    float dError = (error - lastError) / (SAMPLE_MS / 1000.0);
    float output = (Kp * error) + (Ki * errorSum) + (Kd * dError);
    lastError = error;

    // Convert PID output to PWM
    float pwmVal = constrain(output, 0, 255);
    analogWrite(MOTOR_PIN, (int)pwmVal);

    lastPIDTime = millis();
  }
}