#include "BluetoothSerial.h"
char val = 'a';
BluetoothSerial SerialBT;
void setup() {
  // put your setup code here, to run once:
  SerialBT.begin("Blue Hawaiian");
}

void loop() {
  // put your main code here, to run repeatedly:
  SerialBT.write(val);
}
