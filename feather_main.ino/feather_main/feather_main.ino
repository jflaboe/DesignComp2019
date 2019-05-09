#include "BluetoothSerial.h"
#include "UserInput.h"
#include <stdio.h>
#include <math.h>
#include <String>
#include <ArduinoJson.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

//sensor variables
BluetoothSerial SerialBT;
MPU6050 mpu6050(Wire);
unsigned long prev_time = millis();
unsigned long next_time = millis();

//motor variable initialization
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
int timer = 0;
int time_delay = 0;

//servo setup
int servoPin = 21;
int servo_freq = 50;      //servos operate @ 50Hz so we ned to set the freq to that value
int channel = 2;    //which channel do we want to use
int servo_resolution = 8; //set out resolution for a resonable value
int angle = 15; //this is about the halfway point (90deg)

//mode
int command = 0;
bool in_command = false;
int last_completed = 0;

//Bluetooth read variables
char data_in[100];
char next = 'q';
int str_index = 0;
UserInput user;
UserInput next_command;
int last_send = millis();
const int SEND_DELAY = 500;

// json
const int capacity = JSON_ARRAY_SIZE(2) + JSON_ARRAY_SIZE(6) + JSON_OBJECT_SIZE(4);
String json_out = "";


//laser
bool laser_on = false;


void setup() {
  user.command = 0;
  Serial.begin(115200);
  

  //set up bluetooth
  SerialBT.begin("Blue Hawaiian");

  

  //set up motors
  Serial.println("Setting up motors");
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

  //set up lever
  Serial.println("Setting up lever");
  pinMode(servoPin, OUTPUT);
  ledcSetup(channel, servo_freq, servo_resolution);
  ledcAttachPin(servoPin, channel);
  ledcWrite(channel, angle);

  
  Serial.println("Finished setup");
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
  if(next_time - last_send > SEND_DELAY)
  {
    
    last_send = next_time;
    StaticJsonDocument<capacity> all_data; // will hold all sensor data + CPU time
    all_data["last_executed"] = last_completed;
    all_data["mode"] = command;
   
    serializeJson(all_data, json_out);
    SerialBT.println(json_out);
    //Serial.println(json_out);
    json_out = "";
  }
  
  
  /*
   * Send the current CPU time and all the sensor data that has been read this loop.
   * The data should be sent in JSON format, with keys that are the name of the sensor or 
   * the name of the piece of data being sent, and the value should be the value of that sensor
   */



  if (SerialBT.available())
      {
        
        //go until end of a command or until there is nothing left to read
        
        while (SerialBT.available() && next != '\n')
        {
          next = SerialBT.read();
          

          
          data_in[str_index] = next;
          //Serial.println(data_in);
          str_index += 1;
        }

        //process command
        if (next == '\n')
        {

          Serial.println(data_in);
          sscanf(data_in, "%d %d %d %d %d %d %d\n", &next_command.id, &next_command.command, &next_command.data1, &next_command.data2, &next_command.data3, &next_command.data4, &next_command.data5);
          memset(data_in, 0, sizeof data_in);
          next = 'q';
          str_index = 0;
          
          if (next_command.command == 0 || (!in_command && next_command.id > last_completed)){
            Serial.println("data accepted!");
            last_completed = user.id;
            user.id = next_command.id;
            user.command = next_command.command;
            user.data1 = next_command.data1;
            user.data2 = next_command.data2;
            user.data3 = next_command.data3;
            user.data4 = next_command.data4;
            user.data5 = next_command.data5;
          }
  
          
        }
      }
      


      
  //perform update based on mode

  switch (user.command){ 
      

    //hault mode
    /*
     * Stop the motors
     */
     case 0:
      PWM_motor1 = 0;
      PWM_motor2 = 0;
      ledcWrite(PWM1channel, 0); // pwm channel, speed 0-255
      ledcWrite(PWM2channel, 0); // pwm channel, speed 0-255
      digitalWrite(DIR1pin, 0); // set direction to cw/ccw
      digitalWrite(DIR2pin, 0);
      in_command = false;
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
      if (!in_command)
      {
        Serial.println("setting in_command to true");
        in_command = true;
        PWM_motor1 = user.data1;
        PWM_motor2 = user.data2;
        ledcWrite(PWM1channel,user.data1); // pwm channel, speed 0-255
        ledcWrite(PWM2channel,user.data2); // pwm channel, speed 0-255
        digitalWrite(DIR1pin, user.data3); // set direction to cw/ccw
        digitalWrite(DIR2pin, user.data4); // set direction to cw/ccw
        time_delay = user.data5 + millis();
      }
      else if(millis() > time_delay){
        time_delay = 0;
        ledcWrite(PWM1channel,0); // pwm channel, speed 0-255
        ledcWrite(PWM2channel,0); // pwm channel, speed 0-255
        digitalWrite(DIR1pin, 0); // set direction to cw/ccw
        digitalWrite(DIR2pin, 0); // set direction to cw/ccw
        user.command = 0;
        in_command = false;
        last_completed = user.id; 
        user.command = 0;
      }
      //PID control
      else{
        
      }

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
      if (!in_command){
        angle = user.data1;
        ledcWrite(channel, angle);
        last_completed=user.id;
        user.command = 0;
        Serial.println(last_completed);
      }
      


      break;

    //direct drive mode
    case 6:
      if (!in_command){
        PWM_motor1 = user.data1;
        PWM_motor2 = user.data2;
        ledcWrite(PWM1channel,user.data1); // pwm channel, speed 0-255
        ledcWrite(PWM2channel,user.data2); // pwm channel, speed 0-255
        digitalWrite(DIR1pin, user.data3); // set direction to cw/ccw
        digitalWrite(DIR2pin, user.data4); // set direction to cw/ccw
        last_completed=user.id;
        user.command = 0;
      }
      
      

      break;
  }

}
