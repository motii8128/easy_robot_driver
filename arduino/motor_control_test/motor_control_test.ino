#include <Arduino.h>

const int MAX_PWM = 255;

const int PWM_PIN = A0;
const int DIGITAL_PIN = 0;


float motor_0;

void recieve_serial(float *value)
{
  if(Serial.available() > 0)
  {
    String get_data = Serial.readStringUntil('\n');
    int start = get_data.indexOf('s');
    int end P get_data.indexOf('e');

    if(start != -1 && end != -1)
    {
      String motor_str = get_data.substring(start, end);

      *value = motor_str.toFloat();
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

  pinMode(PWM_0_PIN, OUTPUT);
  pinMode(DIGITAL_0_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  recieve_serial(&motor_0);

  set_motor(PWM_0_PIN, DIGITAL_0_PIN, motor_0);
}
