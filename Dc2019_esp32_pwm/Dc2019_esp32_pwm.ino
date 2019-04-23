const int PWM1pin = 14;//A0;   // GPIO pin 14
const int DIR1pin = 32;   // GPIO pin 32
const int PWM1channel = 0;
const int freq = 5000;
const int resolution = 8;
const int dutyCycle = 5000;

void setup() {
  ledcSetup(PWM1channel,5000,8); // pwm channel, frequency, resolution in bits
  ledcAttachPin(PWM1pin,PWM1channel); // pin, pwm channel

  pinMode(DIR1pin,OUTPUT); // set direction pin to output

  ledcWrite(PWM1channel,0); // pwm channel, speed 0-255
  digitalWrite(DIR1pin, LOW); // set direction to cw/ccw
}

void loop() {
  ledcWrite(PWM1channel,128); // full speed
  digitalWrite(DIR1pin, LOW); // cw/ccw

  delay(3000);

  ledcWrite(PWM1channel,64); // full speed
  digitalWrite(DIR1pin, HIGH); // ccw/cw

  delay(3000);
}

//void setup() {
//  ledcSetup(PWM1channel, freq, resolution);
//  ledcAttachPin(A0, PWM1channel);
//  pinMode(DIR1pin, OUTPUT);
//}
//
//void loop() {
//  digitalWrite(DIR1pin, LOW);
//  for(int d = 0; d < dutyCycle; ++d) {
//    ledcWrite(PWM1channel, d);
//  }
//
//  delay(2000);
//  digitalWrite(DIR1pin, HIGH);
//
//  delay(2000);
//}
