// this code uses the Vive sensors to calculate position (of two sensors)
// and prints over Serial3 (TX3=pin 8, RX3 = pin 7)
// NDM 5/9/19

// for vive
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define Vive1PIN 22 //19 // the signal from the sensor
#define Vive2PIN 11 // the signal from the sensor
#define DEG_PER_US 0.0216 // equal to (180 deg) / (8333 us)
#define DEG_TO_RADIAN 0.0174533 // pi/180
#define LIGHTHOUSEHEIGHT 3.5 // in feet
#define LIGHTHOUSEANGLE 30 // in degrees

// for ToF
#include <Wire.h>
#include "VL53L1X.h"

// structure to store the sensor data
typedef struct {
  unsigned long changeTime[11];
  double horzAng;
  double vertAng;
  int useMe;
  int collected;
} viveSensor;

// variables for the sensor data and filter
volatile viveSensor V1, V2;
double xPos1, yPos1, xPos2, yPos2;
double xOld1 = 0, yOld1 = 0, xFilt1 = 0, yFilt1 = 0;
double xOld2 = 0, yOld2 = 0, xFilt2 = 0, yFilt2 = 0;

// for ToF
VL53L1X Distance_Sensor;

void setup(void) {
  Serial.begin(9600); // to talk to computer

  // vive
  V1.horzAng = 0;
  V1.vertAng = 0;
  V1.useMe = 0;
  V1.collected = 0;
  pinMode(Vive1PIN, INPUT); // to read the sensor
  attachInterrupt(digitalPinToInterrupt(Vive1PIN), ISRVive1, CHANGE);
  V2.horzAng = 0;
  V2.vertAng = 0;
  V2.useMe = 0;
  V2.collected = 0;
  pinMode(Vive2PIN, INPUT); // to read the sensor
  attachInterrupt(digitalPinToInterrupt(Vive2PIN), ISRVive2, CHANGE);

  Serial3.begin(9600); // to talk to ESP32

  // to start I2C
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // ToF setup
  Distance_Sensor.setTimeout(500);
  if (!Distance_Sensor.init())
  {
    Serial.println("Failed to initialize VL53L1X Distance_Sensor!");
    while (1);
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode
  Distance_Sensor.setDistanceMode(VL53L1X::Short);
  Distance_Sensor.setMeasurementTimingBudget(50000);
  
  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  Distance_Sensor.startContinuous(50);
}

void loop() {

  // get vive position
  if (V1.useMe == 1) {
    V1.useMe = 0;
    // calculate the position and filter it
    xPos1 = tan((V1.vertAng - LIGHTHOUSEANGLE) * DEG_TO_RADIAN) * LIGHTHOUSEHEIGHT;
    yPos1 = LIGHTHOUSEHEIGHT / cos((V1.vertAng - LIGHTHOUSEANGLE) * DEG_TO_RADIAN)  * tan((V1.horzAng - 90.0) * DEG_TO_RADIAN);
    xFilt1 = xOld1 * 0.5 + xPos1 * 0.5; // filter
    yFilt1 = yOld1 * 0.5 + yPos1 * 0.5; // filter
    xOld1 = xFilt1; // remember for next loop
    yOld1 = yFilt1; // remember for next loop

    Serial3.print("a ");
    Serial3.print(xFilt1);
    Serial3.print(" ");
    Serial3.println(yFilt1);
/* */
    Serial.print("V1: ");
    Serial.print(xFilt1);
    Serial.print(" ");
    Serial.println(yFilt1);
/**/
  }
  if (V2.useMe == 1) {
    V2.useMe = 0;

    // calculate the position and filter it
    xPos2 = tan((V2.vertAng - LIGHTHOUSEANGLE) * DEG_TO_RADIAN) * LIGHTHOUSEHEIGHT;
    yPos2 = LIGHTHOUSEHEIGHT / cos((V2.vertAng - LIGHTHOUSEANGLE) * DEG_TO_RADIAN)  * tan((V2.horzAng - 90.0) * DEG_TO_RADIAN);
    xFilt2 = xOld2 * 0.5 + xPos2 * 0.5; // filter
    yFilt2 = yOld2 * 0.5 + yPos2 * 0.5; // filter
    xOld2 = xFilt2; // remember for next loop
    yOld2 = yFilt2; // remember for next loop

    Serial3.print("b ");
    Serial3.print(xFilt2);
    Serial3.print(" ");
    Serial3.println(yFilt2);
/**/
    Serial.print("     V2: ");
    Serial.print(xFilt2);
    Serial.print(" ");
    Serial.println(yFilt2);
/**/
  }
//  noInterrupts(); // don't fuck with the tof reading
//  if(Distance_Sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial3.print("c ");
  Serial3.print(Distance_Sensor.read());
  Serial3.print(" ");
  Serial3.println(Distance_Sensor.ranging_data.range_status);
  
  Serial.print("tof: ");
  Serial.println(Distance_Sensor.read());
  Serial.print(" ");
  Serial.println(Distance_Sensor.ranging_data.range_status);
  
//  interrupts(); // restart vive checks
  delay(10);
}

// the sensor interrupts
void ISRVive1() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V1.changeTime[i] = V1.changeTime[i + 1];
  }
  V1.changeTime[10] = mic;

  // if the buffer is full
  if (V1.collected < 11) {
    V1.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V1.changeTime[1] - V1.changeTime[0] > 7000) && (V1.changeTime[3] - V1.changeTime[2] > 7000) && (V1.changeTime[6] - V1.changeTime[5] < 50) && (V1.changeTime[10] - V1.changeTime[9] < 50)) {
      V1.horzAng = (V1.changeTime[5] - V1.changeTime[4]) * DEG_PER_US;
      V1.vertAng = (V1.changeTime[9] - V1.changeTime[8]) * DEG_PER_US;
      V1.useMe = 1;
    }
  }
}
void ISRVive2() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V2.changeTime[i] = V2.changeTime[i + 1];
  }
  V2.changeTime[10] = mic;

  // if the buffer is full
  if (V2.collected < 11) {
    V2.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V2.changeTime[1] - V2.changeTime[0] > 7000) && (V2.changeTime[3] - V2.changeTime[2] > 7000) && (V2.changeTime[6] - V2.changeTime[5] < 50) && (V2.changeTime[10] - V2.changeTime[9] < 50)) {
      V2.horzAng = (V2.changeTime[5] - V2.changeTime[4]) * DEG_PER_US;
      V2.vertAng = (V2.changeTime[9] - V2.changeTime[8]) * DEG_PER_US;
      V2.useMe = 1;
    }
  }
}
