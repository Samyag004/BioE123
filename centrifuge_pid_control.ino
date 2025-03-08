// centrifuge_pid_control.ino

// Define states
enum State { WAITING, RAMP_UP, RUNNING, RAMP_DOWN };
State state = WAITING; // initial state is WAITING

// Pins and constants
const int motor_gate_pin = 5; // PWM output to motor driver
const int comparator_pin = 2; // Input from comparator

const float max_RPM = 1500.0; // Maximum RPM 

// Timing constants (in milliseconds)
const unsigned long comparator_sample_interval = 1000; // PID updates every 1s 
const unsigned long RAMP_UP_TIME = 5000; // Ramp-up period (motor at full power for 5s)

// Centrifugation duration (in milliseconds)
float duration = 0;   // user-specified centrifugation time

// PID controller variables
float set_RPM = 0;       // Target RPM (user input)
float measured_RPM = 0;  // Measured RPM (computed from pulse count)
float error = 0;
float integral = 0;
float derivative = 0;
float previous_error = 0;
float pid_output = 0;    // PID controller output
float baseline_duty = 0; // Theoretical goal PWM value (feedforward)
float pwm_value = 0;     // Final PWM output

// PID tuning constants
float Kp = 0.06;
float Ki = 0.002;
float Kd = 0.001;

// Timing variables
unsigned long start_time = 0;           // Start time for RUNNING state
unsigned long ramp_up_start_time = 0;   // Start time of ramp-up phase (5s)
unsigned long last_pid_time = 0;        // Last PID update time

// Pulse counter (used in the ISR)
volatile int comparator_counter = 0;
int comparator_counter_previous = 0;

// Number of magnets that pass the sensor per revolution:
const float magnet = 2.0;   // Adjust if your rotor uses a different magnet count

// ISR: Increment the pulse counter each time a pulse is detected.
void comparatorInterruptHandler() {
  comparator_counter++;
}

void setup() {
  pinMode(motor_gate_pin, OUTPUT);
  pinMode(comparator_pin, INPUT_PULLUP);
  
  Serial.begin(9600);
  while (!Serial);  // wait for Serial Monitor to open
  Serial.println("Welcome to the V2 Centrifuge :), Enter desired motor speed in RPM (max 1500 RPM):");
  
  attachInterrupt(digitalPinToInterrupt(comparator_pin), comparatorInterruptHandler, FALLING);
}

void loop() {
  unsigned long current_time = millis();
  
  // STATE: WAITING – Wait for user input.
  if (state == WAITING) {
    analogWrite(motor_gate_pin, 0); // ensure motor is off
    if (Serial.available() > 0) {
      // Read target RPM from serial
      set_RPM = Serial.parseFloat();
      Serial.print("Target RPM: ");
      Serial.println(set_RPM);
      
      Serial.println("Enter centrifugation time in seconds (max 600s):");
      while (Serial.available() == 0) ; // wait for time input
      duration = Serial.parseFloat();
      Serial.print("Centrifugation time: ");
      Serial.print(duration);
      Serial.println(" seconds");
      duration *= 1000;  // convert seconds to milliseconds
      
      // Compute a feedforward (baseline) PWM value assuming linear mapping.
      // (e.g., if set_RPM == max_RPM, baseline_duty becomes 255.)
      baseline_duty = (set_RPM / max_RPM) * 255;
      
      // Prepare for the ramp-up phase.
      ramp_up_start_time = current_time;
      // Reset PID variables and pulse counter.
      integral = 0;
      previous_error = 0;
      noInterrupts();
      comparator_counter = 0;
      comparator_counter_previous = 0;
      interrupts();
      
      state = RAMP_UP;
      Serial.println("Ramping up, at max speed for 5s...");
      analogWrite(motor_gate_pin, 255); // run at full power during ramp-up
    }
  }
  // STATE: RAMP_UP – Run motor at full power for a fixed period.
  else if (state == RAMP_UP) {
    analogWrite(motor_gate_pin, 255);
    if (current_time - ramp_up_start_time >= RAMP_UP_TIME) {
      // End ramp-up: reset pulse counter and start PID control.
      noInterrupts();
      comparator_counter = 0;
      comparator_counter_previous = 0;
      interrupts();
      
      last_pid_time = current_time;
      start_time = current_time;  // mark the beginning of PID-controlled run
      state = RUNNING;
      Serial.println("Ramp-up complete. Starting PID control...");
    }
  }
  // STATE: RUNNING – Use PID control to adjust motor speed.
  else if (state == RUNNING) {
    // Check if centrifugation duration has elapsed.
    if (current_time - start_time >= duration) {
      state = RAMP_DOWN;
      Serial.println("Centrifugation duration complete. Ramp down initiated. Please wait...");
    }
    else {
      // Update the PID loop at the defined sample interval.
      if (current_time - last_pid_time >= comparator_sample_interval) {
        
        // Calculate pulses during the last interval.
        int pulses = comparator_counter - comparator_counter_previous;
        comparator_counter_previous = comparator_counter;
        
        // Elapsed time in milliseconds for the last interval
        float elapsed_time = (float)(current_time - last_pid_time);
        
        // Compute the measured RPM with the specified formula:
        // current RPM = pulses / float(magnet) * (60,000.0 / elapsed_time)
        measured_RPM = (pulses / magnet) * (60000.0 / elapsed_time);

        // Now proceed with the standard PID error terms.
        // Convert elapsed_time (ms) to seconds for the integral/derivative calculations.
        float dt = elapsed_time / 1000.0;
        
        // Compute PID error (target RPM minus measured RPM).
        error = set_RPM - measured_RPM;
        integral += error * dt;
        derivative = (error - previous_error) / dt;
        previous_error = error;
        
        // Compute the PID controller output.
        pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        
        // Adjust the PWM duty cycle by adding the PID correction to the feedforward baseline.
        pwm_value = baseline_duty + pid_output;
        // Constrain the PWM value to [0, 255].
        if (pwm_value > 255) pwm_value = 255;
        // (Note: The following if statement in C++ does not work as "180 > pwm_value > 20" 
        //  is not a chained comparison. If you want a range check, use:
        //  if (pwm_value > 20 && pwm_value < 180) ...)
        if (180 > pwm_value > 20) pwm_value = pwm_value + 50;
        if (pwm_value < 20) pwm_value = 40; // experimentally determined lower bound
        
        analogWrite(motor_gate_pin, (int)pwm_value);
        
        // Debug output.
        Serial.print("Measured RPM: ");
        Serial.print(measured_RPM);
        Serial.print(" | Error: ");
        Serial.print(error);
        Serial.print(" | PID Out: ");
        Serial.print(pid_output);
        Serial.print(" | PWM: ");
        Serial.println(pwm_value);
        
        last_pid_time = current_time;
      }
    }
  }
  // STATE: RAMP_DOWN – Turn off the motor.
  else if (state == RAMP_DOWN) {
    analogWrite(motor_gate_pin, 0);
    Serial.println("Motor turned off. Centrifugation complete.");
    delay(2000);  // allow time for any final processing
    state = WAITING;
    Serial.println("Welcome to the V2 Centrifuge :), Enter desired motor speed in RPM (max 1500 RPM):");
  }
}
