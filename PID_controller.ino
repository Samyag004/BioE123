#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Constants for pins and intervals
const int POT_pin = A1;
const int PWM_pin = 6;
const int encoder_pin = 3;
const long interval = 100;

// Constants for PID control
const float Kp = 0.5; // Adjust this value for optimal performance
const float Ki = 5; // Adjust this value for optimal performance
const float Kd = 0.002; // Adjust this value for optimal performance
const float Tm = 0.1;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variables
volatile int contador = 0;
unsigned long previousMillis = 0;

float sp = 0;
float pv = 0;
float cv = 0;
float cv1 = 0;
float error = 0;
float error1 = 0;
float error2 = 0;

void setup() {
  pinMode(PWM_pin, OUTPUT);
  pinMode(encoder_pin, INPUT);
  Serial.begin(112500);
  attachInterrupt(digitalPinToInterrupt(encoder_pin), interruption, RISING);

  lcd.init();
  lcd.backlight();
  lcd.begin(16, 2);
}

void loop() {
  unsigned long currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval) {
    previousMillis = currentMillis;
    pv = 10*contador*(60.0/234.3); // Calculate in RPM
    contador = 0;
  }

  // Read the setpoint from the potentiometer
  sp = analogRead(POT_pin) * (280.0 / 1023); // Map potentiometer value to RPM range

  error = sp - pv;

  // PID Control Calculation
  cv = cv1 + (Kp + Kd / Tm) * error + (-Kp + Ki * Tm - 2 * Kd / Tm) * error1 + (Kd / Tm) * error2;
  cv1 = cv;
  error2 = error1;
  error1 = error;

  // Limit the output of the PID controller
  if (cv > 500)
  {
    cv = 500.0;
  }

  if(cv<30.0)
  {
    cv = 30.0;
  }

  // Apply the control signal to the PWM output
  analogWrite(PWM_pin,cv*(255.0/500.0));//0-255

  // Print data to Serial monitor
  Serial.print("Setpoint (RPM): ");
  Serial.print(sp);
  Serial.print(", Process Variable (RPM): ");
  Serial.println(pv);
  delay(100);

  // Display data on the LCD
  lcd.setCursor(0, 0);
  lcd.print("SP (RPM): ");
  lcd.print(sp);
  lcd.setCursor(0, 1);
  lcd.print("PV (RPM): ");
  lcd.print(pv);
}

void interruption() {
  contador++;
}
