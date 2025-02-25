#include <Arduino.h>

/* Pins & Local Constants */
const int MOTOR_PIN = 3;       // PWM to motor driver
const int COMP_PIN  = 2;       // Comparator output (photodiode)
const double MAX_RPM = 1500.0;  // Safety limit
const int MAX_SEC   = 600;     // 10 minutes
const unsigned long SAMPLE_MS = 200; // PID update every 200 ms

/* PID Settings */
double Kp = 0.2; //
double Ki = 0.05; //integral coefficient
double Kd = 0.1; //differential coefficient

/* RPM values initialized */
double setRPM = 0; //initialize RPM to 0
double currentRPM = 0; //initialize current RPM to 0

/* initialize error */
double errorSum = 0; 
double lastError = 0;

/* initialize time settings */
unsigned long lastPIDTime = 0;
unsigned long runDuration = 0;
unsigned long startTime = 0;

/* initialize pulse count */
volatile unsigned long pulseCount = 0;
bool running = false;
 
/* ISR_countPulses is our interrupt protocol, 
 * it will update pulseCount every time the interrupt 
 * is called on CHANGE. 
 */
void ISR_countPulses() { 
  pulseCount++; 
}

/* setup() initializes the centrifuge run, and directly 
 * communicates with the user through the serial terminal 
 */
void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(COMP_PIN, INPUT_PULLUP);

  /* prompts the user */
  Serial.println("Enter target RPM (<=1500):");
  //while (!Serial.available()) {}
  setRPM = Serial.parseFloat();
  if (setRPM > MAX_RPM){
    setRPM = MAX_RPM;
  }
  Serial.println("Enter run time in seconds (<=600):");
  //while (!Serial.available()) {}
  runDuration = (unsigned long)Serial.parseInt();
  if (runDuration > MAX_SEC){
    runDuration = MAX_SEC;
  }
  runDuration *= 1000; //convert to milliseconds
  startTime = millis();
  running = true; //change boolean running value
  Serial.println("Starting...");
}

/* loop() does the following:
 * 1. if not running, return the loop. Ensures default is OFF
 * 2. if duration is over, motor should stop spinning
 * 3. every sample rate, the PID control system should update the count.
 * 4. 
 */
void loop() {
  if (!running){
    return;
  }
  
  /* centrifuge stops if time is up */
  if (millis() - startTime >= runDuration) {
    analogWrite(MOTOR_PIN, 0);
    running = false;
    Serial.println("RUN IS OVER!!!");
    return;
  }

  /* PID system updates count every SAMPLE_MS */
  if (millis() - lastPIDTime >= SAMPLE_MS) {
    noInterrupts();
    unsigned long pulses = pulseCount; 
    pulseCount = 0;

    /* read from interrupts() */
    interrupts();

    /* currentRPM = (pulses_sec/sec) * 60.0 */
    double pulses_sec = pulses * (1000.0 / SAMPLE_MS); 
    currentRPM = pulses_sec * 30.0; //divide 60 by 2 b/c 2 signals per rotation. 

    /* update the PID computations*/ 
    double error = setRPM - currentRPM;
    errorSum += error * (SAMPLE_MS / 1000.0);
    double dError = (error - lastError) / (SAMPLE_MS / 1000.0);
    double output = (Kp * error) + (Ki * errorSum) + (Kd * dError);
    lastError = error;

    /* Convert PID output to PWM to update the motor pin*/
    double pwmVal = constrain(output, 0, 255);
    analogWrite(MOTOR_PIN, (int)pwmVal);

    lastPIDTime = millis(); //update time
  }
}