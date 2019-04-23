#include "BluetoothSerial.h"

#define triggerPin 32
#define echoPin 14
#define LED1Pin 15
#define LED2Pin 12

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
//  if (Serial.available()) {
//    SerialBT.write(Serial.read());
//  }
  digitalWrite(triggerPin, HIGH);  // sonar pulse out
  delayMicroseconds(10);           // 10 uS delay 
  digitalWrite(triggerPin, LOW);   // turn off triggerPin

  // read pulse at echoPin
  float duration = pulseIn(echoPin, HIGH);
  float distance = duration/148.0;  // in inches
  Serial.print(distance);
  Serial.println();
  if(SerialBT.available()) {
    SerialBT.print(distance);
    //SerialBT.println(" in");
    Serial.println("Written to bluetooth");
  }
  if(distance < 4.0) {
    digitalWrite(LED1Pin, HIGH);
    digitalWrite(LED2Pin, LOW);

  } else {
    digitalWrite(LED2Pin, HIGH);
    digitalWrite(LED1Pin, LOW);
  }

  delay(200);
}
