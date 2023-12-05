#include <Arduino.h>

const int MAX_PWM = 255;

const int PWM_0_PIN = A0;
const int PWM_1_PIN = A1;
const int DIGITAL_0_PIN = 0;
const int DIGITAL_1_PIN = 1;


float motor_0, motor_1;

void recieve_serial(float *value_a, float *value_b)
{
  if(Serial.available() > 0)
  {
    String get_data = Serial.readStringUntil('\n');
    int comma = get_data.indexOf(',');

    if(comma != -1)
    {
      String motor_0_str = get_data.substring(0, comma);
      String motor_1_str = get_data.substring(comma+1);

      *value_a = motor_0_str.toFloat();
      *value_b = motor_1_str.toFloat();
    }
  }
}

void set_motor(int pwm_pin, int digital_pin, float get_value)
{

  int motor_value = abs(get_value) * 255;
  analogWrite(pwm_pin, motor_value);
  if(get_value > 0.0)
  {
    digitalWrite(digital_pin, HIGH);
  }
  else
  {
    digitalWrite(digital_pin, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:
  motor_0 = 0.0;
  motor_1 = 0.0;

  pinMode(PWM_0_PIN, OUTPUT);
  pinMode(PWM_1_PIN, OUTPUT);
  pinMode(DIGITAL_0_PIN, OUTPUT);
  pinMode(DIGITAL_1_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  recieve_serial(&motor_0, &motor_1);

  set_motor(PWM_0_PIN, DIGITAL_0_PIN, motor_0);
  set_motor(PWM_1_PIN, DIGITAL_1_PIN, motor_1);
}
