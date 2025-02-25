float e=2.71828;
bool stateCheck = false;

/*digital and analog pin initialized*/
int motorPin = 3; // Digial Pin 3, sends PWM to control motor
int comparatorPin = A0; // Analog Pin 0, receives V_out from from comparator

/*initialize counts*/
volatile int comparator_counter = 0; 
int comparator_counter_previous = 0;
float current_time = 0; // time--> calls millis() which gives milliseconds since last reset 
float comparator_time = 0; // current time
float comparator_time_previous = 0; // last time we reported the speed (# interrupts/time passed)
float comparator_time_delta = 0; // encoder_time - encoder_time_previous
float comparator_time_tracker = 0;
float time_change = 0;
float previous_time = 0;

// scaling factor from counts to rpm of motor (with gears taken into account)
float comparator_sample_interval = 1000; //how long (in ms) we wait before checking speed 

//define variables
//analog input + pwm variables
float duration = 0;
float elapsed = 0;
float start_time = 0;
float write_duty = 55; // duty cycle to send to mosfet to run motor (PWM)
float frequency = 0;
float angular_velocity = 0;
float PPR = 2;

float set_RPM = 0;
float measured_RPM = 0;
float new_RPM = 0;
float error = 0;
float max_RPM = 2910;

float Kp = 0.025;
float k = 0;

char tmp_str[7]; // temporary variable used in convert function

//interrupt service routine
// quick, called after each interrupt signal
void comparatorInterruptHandler() {
  ++comparator_counter;
}

void setup() {
  pinMode(motorPin, OUTPUT);
  pinMode()
   // must be on pullup because encoder stays floating (locked at 5 from arduino) unless sensor is triggered (allows pin to be pulled down to 0)
  
  Serial.begin(9600);
  while (!Serial);  // wait until the serial monitor is open before proceeding
  Serial.println("Please enter desired motor speed in RPM: ");

  attachInterrupt(digitalPinToInterrupt(comparator_pin),comparatorInterruptHandler,RISING); // triggered every time phototransistor blocked
  current_time = millis();
}

void loop() {
  if (!stateCheck) {
    analogWrite(motorPin, 0);
    if(Serial.available()>0){
      set_RPM = Serial.parseInt();
      new_RPM = set_RPM;
      write_duty = (new_RPM/max_RPM)*255;
      Serial.println((String)"Target RPM: "+set_RPM);
      Serial.println((String)"Enter desired centrifugation time in seconds: ");
      //Serial.println(Serial.available());
      if (Serial.available() > 0) {
        float junk = Serial.read();
      }
      while (Serial.available()==0);
      duration = Serial.parseInt();
      Serial.println((String)"Centrifugation time: "+duration+" seconds");
      duration *= 1000; // convert to millis
      start_time = millis();
      comparator_time_tracker = 0;
      state = ON; // now turned on!
      analogWrite(motor_gate_pin, write_duty);
    } else {
      Serial.println("Please enter desired motor speed in RPM: ");
      if (Serial.available() > 0) {
        float junk = Serial.read();
      }
      while (Serial.available()==0);
    }
  }
  if (state == ON) {
    current_time = millis();
    elapsed = current_time - start_time;
    if (elapsed > duration) { // time to turn off the centrifuge!
      analogWrite(motor_gate_pin, 0);
      Serial.println("Centrifugation period finished. Ramp down initiated.");
      float comparator_counter_current = comparator_counter;
      delay(1000);
      while (comparator_counter > comparator_counter_current) {
        float comparator_counter_delta = comparator_counter - comparator_counter_current; // this value will be the "# pulses"
        float time_passed = 1; // to convert to seconds
        frequency = comparator_counter_delta / time_passed; // this value is "f"

        // tachometer frequency (f) to RPM formula: RPM = (f * 60) / PPR
        // where PPR = pulses per revolution
        // in the case of our centrifuge, the PPR = 2, because there are 2 light blockers, meaning that there will be two "pulses" per revolution
        angular_velocity = (frequency*60) / PPR;
        comparator_counter_previous = comparator_counter_current;
        measured_RPM = angular_velocity;
        Serial.println(measured_RPM);
        comparator_counter_current = comparator_counter;
        delay(1000);
      }
      Serial.println("Ramp down completed. Thank you for using Team 9's awesome centrifuge :)");
      if (Serial.available() > 0) {
        float junk = Serial.read();
      }
      state = OFF;
    } else {
      time_change = current_time - previous_time;
      comparator_time_tracker += time_change;
      if (comparator_time_tracker >= comparator_sample_interval) {
        float comparator_counter_current = comparator_counter;
        float comparator_counter_delta = comparator_counter_current - comparator_counter_previous; // this value will be the "# pulses"
        float time_passed = comparator_time_tracker / 1000; // to convert to seconds
        frequency = comparator_counter_delta / time_passed; // this value is "f"

        // tachometer frequency (f) to RPM formula: RPM = (f * 60) / PPR
        // where PPR = pulses per revolution
        // in the case of our centrifuge, the PPR = 2, because there are 2 light blockers, meaning that there will be two "pulses" per revolution
        angular_velocity = (frequency*60) / PPR;
        comparator_counter_previous = comparator_counter_current;
        measured_RPM = angular_velocity;
        Serial.println(measured_RPM);
        
        error = measured_RPM - set_RPM;
        if (error > 0) {
          k = -(error*Kp);
        } else if (error < 0) {
          k = abs((error*Kp));
        } else if (error = 0) {
          k = 0;
        } 
        new_RPM += k;
        write_duty = 13.8*(pow(e, (0.00101*new_RPM))); 
        if (write_duty > 255) {
          write_duty = 255;
        }
        if (write_duty < 0) {
          write_duty = 0;
        }
        analogWrite(motor_gate_pin, write_duty);
        comparator_time_tracker = 0; // reset time tracker
      }
      previous_time = current_time;
    }
  }
}