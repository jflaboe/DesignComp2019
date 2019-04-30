#include "BluetoothSerial.h"

#define triggerPin 14
#define echoPin 12
#define LED1Pin 15
#define LED2Pin 27

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32test_"); //Bluetooth device name
  pinMode(triggerPin,OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED1Pin,OUTPUT);
  pinMode(LED2Pin,OUTPUT);
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  digitalWrite(triggerPin, HIGH);  // sonar pulse out
  delayMicroseconds(10);           // 10 uS delay 
  digitalWrite(triggerPin, LOW);   // turn off triggerPin

  // read pulse at echoPin
  float duration = pulseIn(echoPin, HIGH);
  float distance = duration/148.0;  // in inches
  Serial.print(distance);
  Serial.println();
//  if(SerialBT.available()) {
  SerialBT.print(distance);
  SerialBT.println(" in");
//  Serial.println("Written to bluetooth");
//  }
  if(distance < 4.0) {
    digitalWrite(LED1Pin, HIGH);
    digitalWrite(LED2Pin, LOW);

  } else {
    digitalWrite(LED2Pin, HIGH);
    digitalWrite(LED1Pin, LOW);
  }

  delay(200);
}
 
//int trigPin = 11;    // Trigger
//int echoPin = 12;    // Echo
//long duration, cm, inches;
 
//void setup() {
//  //Serial Port begin
//  Serial.begin (115200);
//  //Define inputs and outputs
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//}
// 
//void loop() {
//  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
//  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(5);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
// 
//  // Read the signal from the sensor: a HIGH pulse whose
//  // duration is the time (in microseconds) from the sending
//  // of the ping to the reception of its echo off of an object.
//  pinMode(echoPin, INPUT);
//  duration = pulseIn(echoPin, HIGH);
// 
//  // Convert the time into a distance
//  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
//  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
//  
//  Serial.print(inches);
//  Serial.print("in, ");
//  Serial.print(cm);
//  Serial.print("cm");
//  Serial.println();
//  
//  delay(125);
//}
