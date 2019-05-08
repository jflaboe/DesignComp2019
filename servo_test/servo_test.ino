#include <ESP32Servo.h>

int servoPin = 21;
//Servo myServo;
//int angle = 0;

int freq = 50;      //servos operate @ 50Hz so we ned to set the freq to that value
int channel = 0;    //which channel do we want to use
int resolution = 8; //set out resolution for a resonable value
int angle = 15; //this is about the halfway point (90deg)

void setup() {
  Serial.begin(115200);
  pinMode(servoPin, OUTPUT);
//  myServo.attach(servoPin);
//  myServo.write(0);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(servoPin, channel);
  ledcWrite(channel, angle);
}

void loop() {
  char data_string[100];
  int counter = 0;
  while(Serial.available()) {
    int inChar = Serial.read();
//    Serial.println((char)inChar);
    data_string[counter] = inChar;
    counter++;

    if(inChar == '\n') {
      sscanf(data_string, "%d", &angle);
    }
  }
  Serial.println(angle);
//  myServo.write(angle);
  ledcWrite(channel, angle);
  
  delay(50);

  myServo.write(angle);
//  ledcWrite(channel, angle);
//  delay(50);
}
