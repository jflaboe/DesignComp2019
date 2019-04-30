const int PWM1pin = 14;//A0;   // GPIO pin 14
const int PWM2pin = 13;
const int DIR1pin = 32;   // GPIO pin 32
const int DIR2pin = 27;
const int PWM1channel = 1;
const int PWM2channel = 0;
const int freq = 5000;
const int resolution = 8;
const int dutyCycle = 5000;

int PWM_motor1 = 0;
int PWM_motor2 = 0;
int motor1_dir = HIGH;
int motor2_dir = HIGH;

/*
 * Expects serial to have 8 digits.
 * First three digits are motor 1's PWM value.
 * Last three digits are motor 2's PWM value.
 * sign-#-#-#-sign-#-#-#
 */
int readPWMString(int* newPWMs) {
  String value = "";
  while(Serial.available()) { // while serial has anything inputted
    int inChar = Serial.read();
    if(isDigit(inChar)) {
      Serial.print(inChar);
      Serial.println();
      value += (char)inChar;
    }
    if(inChar == '\n') {
      Serial.print("Number is");
      Serial.println(value.toInt());
    }
  }
  return value.toInt();
}

void setup() {
  Serial.begin(9600);
  ledcSetup(PWM1channel,5000,8); // pwm channel, frequency, resolution in bits
  ledcSetup(PWM2channel, 5000, 8);
  ledcAttachPin(PWM1pin,PWM1channel); // pin, pwm channel
  ledcAttachPin(PWM2pin,PWM2channel); // pin, pwm channel

  pinMode(DIR1pin,OUTPUT); // set direction pin to output
  pinMode(DIR2pin,OUTPUT); // set direction pin to output

  ledcWrite(PWM1channel,PWM_motor1); // pwm channel, speed 0-255
  ledcWrite(PWM2channel,PWM_motor2); // pwm channel, speed 0-255
  digitalWrite(DIR1pin, motor1_dir); // set direction to cw/ccw
  digitalWrite(DIR2pin, motor2_dir); // set direction to cw/ccw
}

void loop() {
  ledcWrite(PWM1channel,PWM_motor1);
  ledcWrite(PWM2channel,PWM_motor2);
  digitalWrite(DIR1pin, motor1_dir); // cw/ccw
  digitalWrite(DIR2pin, motor2_dir); // cw/ccw

  int newPWMs[2];
  if(Serial.available()) {
    readPWMString(&newPWMs);

    int new_PWM_motor2 = readPWMString();
//    if(PWM_motor1 != newPWMs[0]) {
      Serial.print("pwm 1 val (new, old):");
      Serial.print(newPWMs[0]);
      Serial.print(", ");
      Serial.print(PWM_motor1);
      Serial.println();
      if(newPWMs[0] < 0) {
        motor1_dir = LOW;
        digitalWrite(DIR1pin, motor1_dir); // cw/ccw
      } else if (newPWMs[0] >= 0) {
        digitalWrite(DIR1pin, HIGH); // cw/ccw
        motor1_dir = HIGH;
        digitalWrite(DIR1pin, motor1_dir); // cw/ccw
      }
  
      ledcWrite(PWM1channel,new_PWM_motor1);
      PWM_motor1 = new_PWM_motor1;
//    }
    
//    if(PWM_motor2 != newPWMs[1]) {
      Serial.print("pwm 2 val (new, old):");
      Serial.print(newPWMs[1]);
      Serial.print(", ");
      Serial.print(PWM_motor2);
      Serial.println();
      if(newPWMs[1] < 0) {
        motor2_dir = LOW;
        digitalWrite(DIR2pin, motor2_dir); // cw/ccw
      } else if(newPWMs[1] >= 0) {
         motor2_dir = HIGH;
         digitalWrite(DIR2pin, HIGH); // cw/ccw
      }
         
      ledcWrite(PWM2channel,new_PWM_motor2); // full speed
      PWM_motor2 = new_PWM_motor2;
//    }
  }

    Serial.print("pwm 1 val:");
    Serial.println(PWM_motor1);

    Serial.print("pwm 2 val:");
    Serial.println(PWM_motor2);
  
  delay(3000);
}
