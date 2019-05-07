#include "BluetoothSerial.h"
#include "UserInput.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

//sensor variables
BluetoothSerial SerialBT;
unsigned long prev_time = millis();
unsigned long next_time = millis();

const int sonar_trig_pin = 26; // A0 pin 
const int sonar_L_echo_pin = 15;
const int sonar_R_echo_pin = 33;
const int sonar_L_val = 0;
const int sonar_R_val = 0;


//motor variabel initialization
const int PWM1pin = 14;
const int PWM2pin = 13;
const int DIR1pin = 32;
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

//mode
int mode = 0;

//Bluetooth read variables
char data_in[100];
char next = 'q';
int str_index = 0;
UserInput user;

void triggerSonar() {
  digitalWrite(sonar_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonar_trig_pin, LOW);
  sonar_R_val = pulseIn(sonar_R_echo_pin, HIGH) / 148;  // in inches
  sonar_L_val = pulseIn(sonar_L_echo_pin, HIGH) / 148;  // in inches
}

void setup() {
  Serial.begin(115200);

  //set up bluetooth
  SerialBT.begin("Blue Hawaiian");

  //set up sensors
  pinMode(sonar_trig_pin, OUTPUT);
  pinMode(sonar_echo_pin, INPUT);

  //set up motors
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

  
  //update sensor value variables
  prev_time = next_time;
  next_time = millis();
  /*
   * Every loop, we should check if the sensor is ready to be read. If we end up reading the value
   * of the sensor, we should mark a flag saying that we did
   */
 

  //send sensor data through bluetooth
  
  /*
   * Send the current CPU time and all the sensor data that has been read this loop.
   * The data should be sent in JSON format, with keys that are the name of the sensor or 
   * the name of the piece of data being sent, and the value should be the value of that sensor
   */


  //perform update based on mode

  switch (mode){

    //wait mode
    /*
     * In this mode, we read one instruction from the bluetooth device if available
     * and set the mode to the corresponding value. If there is no instruction available,
     * then keep in wait mode
     */
    case 0: 
      if (SerialBT.available())
      {
        //go until end of a command or until there is nothing left to read
        
        while (SerialBT.available() && next != '\n')
        {
          next = SerialBT.read();
          

          
          data_in[str_index] = next;
          str_index += 1;
        }

        //process command
        if (next == '\n')
        {

          Serial.println(data_in);
          sscanf(data_in, "%d %d %d %d %d\n", &user.command, &user.data1, &user.data2, &user.data3, &user.data4);
          memset(data_in, 0, sizeof data_in);
          next = 'q';
          str_index = 0;
          mode = user.command;
          
          
        }
      }
      


      break;



    //forward mode
    /*
     * In this mode, we try to move both wheels such that the vehicle moves perfectly straight
     * forward until it reaches the destination. The instruction will either contain a distance
     * or a location that is along the current directional vector of the robot. PID is used to
     * control the speed (distance from location) and PWM to each wheel (negative cosine of the angle between
     * the desired vector and the current vector). When the robot is within tolerance of it's destination,
     * the instruction is complete, and the mode is change to recalibrate mode
     */
    case 1:


      break;



    //rotate mode
    /*
     * In this mode, we rotate the robot by moving the wheels opposite directions. We use
     * PID (negative cosine of the angle between the current vector and the desired vector)
     * in order to determine how fast the wheels move. When the angle is withing a tolerance,
     * the instruction is finished and the mode is set to recalibrate.
     */
    case 2:


      break;




    //calibrate mode
    /*
     * This mode is only run while the robot is at a pre-determined position on the table.
     * When the mode is activated, the robot will drive forward to collect N data points.
     * When the N data points are collected, the robot stops and the position of the Vive
     * Sensors are calculated. We assume our z-position is 0 and constant. The mode should
     * be changed to wait after this mode is finished.
     */
    case 3:


      break;




    //recalibrate mode
    /*
     * In this mode, we attempt to reset the error accumulated from the accelerometer.
     * We store N data points from the Vive sensors, and take the average of those data points
     * to determine our location. After the N data points our received, and our position reset,
     * we change our mode to wait.
     */
    case 4:


      break;

    //lever mode
    /*
     * In this mode, we set the PWM of the lever Servo. This should only take one operation,
     * so the mode should change directly to wait. We also do not need to recalibrate after this 
     * mode.
     */
    case 5:


      break;

    //direct drive mode
    case 6:
      ledcWrite(PWM1channel,user.data1); // pwm channel, speed 0-255
      ledcWrite(PWM2channel,user.data2); // pwm channel, speed 0-255
      digitalWrite(DIR1pin, user.data3); // set direction to cw/ccw
      digitalWrite(DIR2pin, user.data4); // set direction to cw/ccw
      mode = 0;
      

      break;
  }

}
