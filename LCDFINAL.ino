#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ----------------- LCD Setup ----------------- //
// Adjust address & size (20x4 vs. 16x2) to match your hardware.
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ----------------- Centrifuge PID Setup ----------------- //
enum State { WAITING, RAMP_UP, RUNNING, RAMP_DOWN };
State state = WAITING;  // initial state

// Pins and constants
const int motor_gate_pin = 5;   // PWM output to motor driver
const int comparator_pin = 7;   // Input from comparator (Note: conflict if using Micro's I2C!)
const float max_RPM = 1500.0;   // Max RPM

// Timing
const unsigned long comparator_sample_interval = 100; // PID update every 100 ms
const unsigned long RAMP_UP_TIME = 5000;              // 5 s ramp-up
float duration = 0;                                   // user-set run time in ms

// PID variables
float set_RPM = 0;       
float measured_RPM = 0;  
float error = 0;
float integral = 0;
float derivative = 0;
float previous_error = 0;
float pid_output = 0;
float baseline_duty = 0;
float pwm_value = 0;

// PID tuning
float Kp = 0.03;
float Ki = 0.0008;
float Kd = 0.0005;
const float INTEGRAL_CLAMP = 1000.0;

// Time-tracking
unsigned long start_time = 0;
unsigned long ramp_up_start_time = 0;
unsigned long last_pid_time = 0;

// Pulse counters
volatile int comparator_counter = 0;
int comparator_counter_previous = 0;

// If your rotor uses 2 magnets per revolution:
const float magnet = 2.0;

// Interrupt: increment pulse count
void comparatorInterruptHandler() {
  comparator_counter++;
}

void setup() {
  // -------- Serial --------
  Serial.begin(9600);
  while (!Serial) {
    // wait for serial monitor (useful on Micro/Leonardo)
  }
  Serial.println("Welcome to the V2 Centrifuge :), Enter desired motor speed in RPM (max 1500 RPM):");

  // -------- Pins --------
  pinMode(motor_gate_pin, OUTPUT);
  pinMode(comparator_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(comparator_pin), comparatorInterruptHandler, FALLING);

  // -------- LCD --------
  Wire.begin();       // Start I2C bus
  lcd.init();         // Initialize LCD
  lcd.backlight();    // Turn on backlight
  lcd.clear();        // Clear display

  // Display initial message
  lcd.setCursor(0, 0);
  lcd.print("V2 Centrifuge");
  lcd.setCursor(0, 1);
  lcd.print("Ready for Input");
}

void loop() {
  unsigned long current_time = millis();

  // ----------------- WAITING State ----------------- //
  if (state == WAITING) {
    // Motor off
    analogWrite(motor_gate_pin, 0);

    // Check for user input over Serial
    if (Serial.available() > 0) {
      set_RPM = Serial.parseFloat();
      Serial.print("Target RPM: ");
      Serial.println(set_RPM);
      
      // Flush leftover chars (like newline)
      while (Serial.available() > 0) {
        Serial.read();
      }
      
      Serial.println("Enter centrifugation time in seconds (max 600s):");

      // Keep asking until valid time is entered
      while (true) {
        while (Serial.available() == 0) {
          delay(50);
        }
        String timeStr = Serial.readStringUntil('\n');
        timeStr.trim();
        duration = timeStr.toFloat();
        if (duration > 0) {
          break;
        } else {
          Serial.println("Invalid input. Please enter a positive time in seconds:");
        }
      }

      Serial.print("Centrifugation time: ");
      Serial.print(duration);
      Serial.println(" seconds");
      duration *= 1000; // convert to ms

      // Feedforward baseline
      baseline_duty = (set_RPM / max_RPM) * 255;

      // Prepare for ramp-up
      ramp_up_start_time = millis();
      integral = 0;
      previous_error = 0;

      // Reset pulse counters
      noInterrupts();
      comparator_counter = 0;
      comparator_counter_previous = 0;
      interrupts();

      // Move to RAMP_UP
      state = RAMP_UP;
      Serial.println("Ramping up, at max speed for 5s...");
      analogWrite(motor_gate_pin, 255); // full power

      // Update LCD to show new state
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Ramping to:");
      lcd.setCursor(0, 1);
      lcd.print((int)set_RPM);
      lcd.print(" RPM");
      lcd.setCursor(0, 2);
      lcd.print("Time: ");
      lcd.print((int)(duration / 1000));
      lcd.print("s");
      lcd.setCursor(0, 3);
      lcd.print("Full power...");
    }
  }

  // ----------------- RAMP_UP State ----------------- //
  else if (state == RAMP_UP) {
    // Keep motor at full power
    analogWrite(motor_gate_pin, 255);

    // Check if 5s of ramp-up is done
    if (current_time - ramp_up_start_time >= RAMP_UP_TIME) {
      // Reset counters before PID
      noInterrupts();
      comparator_counter = 0;
      comparator_counter_previous = 0;
      interrupts();

      last_pid_time = current_time;
      start_time = current_time;
      state = RUNNING;

      Serial.println("Ramp-up complete. Starting PID control...");

      // Update LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("PID Control Active");
      lcd.setCursor(0, 1);
      lcd.print("Target RPM: ");
      lcd.print((int)set_RPM);
    }
  }

  // ----------------- RUNNING State (PID) ----------------- //
  else if (state == RUNNING) {
    // Check if desired time is up
    if (current_time - start_time >= duration) {
      state = RAMP_DOWN;
      Serial.println("Centrifugation duration complete. Ramp down initiated. Please wait...");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Time's Up!");
      lcd.setCursor(0, 1);
      lcd.print("Ramping Down...");
    }
    else {
      // PID update every 100 ms
      if (current_time - last_pid_time >= comparator_sample_interval) {
        
        noInterrupts();
        int pulses = comparator_counter - comparator_counter_previous;
        comparator_counter_previous = comparator_counter;
        interrupts();

        float elapsed_time = (float)(current_time - last_pid_time); // ms
        float dt = elapsed_time / 1000.0; // convert ms -> seconds

        // Calculate RPM
        //  pulses / magnet = revolutions
        //  revolve/sec = (pulses/magnet) / (elapsed_time/1000)
        //  revolve/min = revolve/sec * 60
        //  So, measured_RPM = (pulses / magnet) * (60000 / elapsed_time)
        measured_RPM = (pulses / magnet) * (60000.0 / elapsed_time);

        // PID error
        error = set_RPM - measured_RPM;

        // Integral (clamped)
        integral += error * dt;
        if (integral > INTEGRAL_CLAMP)  integral = INTEGRAL_CLAMP;
        if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP;

        // Derivative
        derivative = (error - previous_error) / dt;
        previous_error = error;

        // PID output
        pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // Final PWM = feedforward + PID
        pwm_value = baseline_duty + pid_output;
        if (pwm_value > 255) pwm_value = 255;
        if (pwm_value < 0)   pwm_value = 0;

        // Output to motor
        analogWrite(motor_gate_pin, (int)pwm_value);

        // Debug to Serial
        Serial.print("Measured RPM: ");
        Serial.print(measured_RPM);
        Serial.print(" | Error: ");
        Serial.print(error);
        Serial.print(" | PID Out: ");
        Serial.print(pid_output);
        Serial.print(" | PWM: ");
        Serial.println(pwm_value);

        // ----- Update LCD with key data ----- //
        // We'll just overwrite lines 2 & 3 for real-time data
        lcd.setCursor(0, 2);
        lcd.print("RPM: ");
        lcd.print((int)measured_RPM);
        lcd.print("    "); // extra spaces to clear old digits if fewer digits now

        lcd.setCursor(0, 3);
        lcd.print("PWM: ");
        lcd.print((int)pwm_value);
        lcd.print("    ");

        last_pid_time = current_time;
      }
    }
  }

  // ----------------- RAMP_DOWN State ----------------- //
  else if (state == RAMP_DOWN) {
    // Turn motor off
    analogWrite(motor_gate_pin, 0);
    Serial.println("Motor turned off. Centrifugation complete.");

    // Update LCD
    lcd.setCursor(0, 2);
    lcd.print("Motor OFF");

    delay(2000);

    // Return to WAITING
    state = WAITING;
    Serial.println("Welcome to the V2 Centrifuge :), Enter desired motor speed in RPM (max 1500 RPM):");

    // LCD back to "Ready" screen
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("V2 Centrifuge Ready");
  }
}
