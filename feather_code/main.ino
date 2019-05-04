#include <stdio.h>
#include <math.h>


void setup() {

  
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  //Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
}

