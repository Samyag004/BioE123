/* Pins & Local Constants */
const int MOTOR_PIN = 2;       // digital PWM to motor driver
const int COMP_PIN = 3;       // Comparator output, is a digital signal (photodiode)
const double MAX_RPM = 1500.0;  // Safety limit
const int MAX_SEC = 600;     // 10 minutes
const unsigned long SAMPLE_MS = 200; // PID update every 200 ms

/* PID Settings */
double Kp = 0.1; //proportional gain
double Ki = 10; //integral gain
double Kd = 1; //differential gain

/* RPM values initialized */
double setRPM = 0; //initialize RPM to 0
double currentRPM = 0; //initialize current RPM to 0
double errorSum = 0; 
double lastError = 0;

/* initialize time settings */
unsigned long previousTime = 0;
unsigned long runDuration = 0;
unsigned long startTime = 0;

/* initialize pulse count */
volatile unsigned long pulseCount = 0;

/* machine states for operating centrifuge */
enum State { WAITING, RUNNING, RAMP_DOWN, FINISHED };
State state = WAITING;  // start in WAITING state
 
/* ISR_countPulses is our interrupt protocol, 
 * it will update pulseCount every time the interrupt 
 * is called on CHANGE. 
 */
void ISR_countPulses(){ 
  pulseCount++; 
}

/* setup() initializes the centrifuge run, and directly 
 * communicates with the user through the serial terminal 
 */
void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(COMP_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(COMP_PIN), ISR_countPulses, CHANGE);

  /* prompts the user */
  Serial.println("Enter target RPM (<=1500):");
  while (Serial.available() == 0) {
    ;
  }
  setRPM = Serial.parseFloat();
  if (setRPM > MAX_RPM){
    setRPM = MAX_RPM;
  }
  Serial.println("Enter run time in seconds (<=600):");
  while (Serial.available() == 0) {
    ;
  }

  runDuration = (unsigned long)Serial.parseInt();
  if (runDuration > MAX_SEC){
    runDuration = MAX_SEC;
  }
  runDuration *= 1000; //convert to milliseconds

  /* CHECK THAT ALL IS WORKING */
  Serial.print("Target RPM: ");
  Serial.println(setRPM);
  Serial.print("Run Duration (ms): ");
  Serial.println(runDuration);

  /* initialize time */
  startTime = millis();
  previousTime = millis();
  state = RUNNING; //change boolean running value
  Serial.println("Starting...");
}

/* loop() does the following:
 * 1. if not running, return the loop. Ensures default is OFF
 * 2. if duration is over, motor should stop spinning
 * 3. every sample rate, the PID control system should update the count.
 * 4. update the PID computations 
 * 5. use computed outputs as error level for new PWM output
 */
void loop() {
  unsigned long currentTime = millis();
  /* State: RUNNING, count interrupts */ 
  if (state == RUNNING) {
    // Check if the run duration has elapsed
    if (currentTime - startTime >= runDuration) {
      state = RAMP_DOWN;
      Serial.println("Run is over. Initiating ramp down...");
      return;
    }
    // RUNNING: Update the PID loop every SAMPLE_MS milliseconds
    if (currentTime - previousTime >= SAMPLE_MS) {
      noInterrupts();
      unsigned long pulses = pulseCount; 
      pulseCount = 0;
      /* read from interrupts() */
      interrupts();

      /* currentRPM = (pulses_sec/sec) * 60.0 */
      double pulses_sec = pulses * (1000.0 / SAMPLE_MS); 
      currentRPM = pulses_sec * 30.0; //divide 60 by 2 b/c 2 signals per rotation. 

      /* update the PID computations */
      double error = setRPM - currentRPM;
      errorSum += error * (SAMPLE_MS / 1000.0);
      double dError = (error - lastError) / (SAMPLE_MS / 1000.0);
      lastError = error;
      double output = (Kp * error) + (Ki * errorSum) + (Kd * dError);
      if (output > 500.0) {
        output = 500.0; //half if max out
      }
      if (output < 30.0) {
        output = 30.0;
      }
      double pwmVal = output * (255.0 / 500.0);
      analogWrite(MOTOR_PIN, (int) pwmVal);

      previousTime = currentTime;
      Serial.print("Pulses: ");
      Serial.println(pulses);
      Serial.print("Current RPM: ");
      Serial.println(currentRPM);
      Serial.print("PID Output: ");
      Serial.println(output);

      if ((output == 500.00) && (currentRPM == 0.00) && (pulses == 0.00)){
        analogWrite(MOTOR_PIN, 0);
        setRPM = 0;
        state = RAMP_DOWN;
        Serial.println("Safety: No light detected. Motor stopped.");
       return;
      }
    }
  }
  /* State: RAMP_DOWN, shut off the motor */ 
  else if (state == RAMP_DOWN) {
    // Immediately set PWM to 0 to stop driving the motor
    analogWrite(MOTOR_PIN, 0);
    Serial.println("Motor PWM set to 0 for ramp down.");
    if (currentRPM < 20){
      state = FINISHED;
    }
  }
  /* State: FINISHED, awaits next run */ 
  else if (state == FINISHED){
    Serial.println("Centrifugation complete. System waiting.");
    state = WAITING;
  }
  else if (state == WAITING) {
    
  }
}