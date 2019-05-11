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
//Time of Flight Initialization
VL53L1X Distance_Sensor;
Wire.begin();
Wire.setClock(400000);
Distance_Sensor.setTimeout(500);
// Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
// You can change these settings to adjust the performance of the sensor, but
// the minimum timing budget is 20 ms for short distance mode
Distance_Sensor.setDistanceMode(VL53L1X::Short);
Distance_Sensor.setMeasurementTimingBudget(50000);

// Start continuous readings at a rate of one measurement every 50 ms (the
// inter-measurement period). This period should be at least as long as the
// timing budget.
Distance_Sensor.startContinuous(50);



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
        
        int Eint=0;
        float edot=0;
        float u=0;
        float error=0;
        float unew=0;
        float error_prev=0;
        float kp=0;
        float ki=0;
        float kdot=0;

        //Read in from (TX3=pin 8, RX3 = pin 7)
        double x_pos_1=serial3.read() ;
        double y_pos_1=serial3.read() ;
        double x_pos_2=serial.read() ;
        double y_pos_2=serial.read() ;
        error =  ;
        Eint += error;
        edot= error-error_prev;
        u = kp_P*error + ki_P*Eint +kd_P*edot;
        error_prev=error;
      
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
    case 7: //forward
      if (!in_command)
      {
        Serial.println("setting in_command to true");
        in_command = true;
        x_pos_init=x_pos;//x_pos is a serial read at the top of the code
        y_pos_init=y_pos;//y_pos is a serial read at the top of the code
        x_pos_f=user.data1;
        y_pos_f=user.data2;
        i_vec_init=x_f-x_pos_init;
        j_vec_init=y_f-y_pos_init;
        distance_init=math.sqrt(i_vec_init*i_vec_init + j_vec_init*j_vec_init);
        i_vec_init/=distance_init;
        j_vec_init/=distance_init;
        //PID variables
        float edot_d=0,edot_theta_e=0,edot_theta_o=0;
        float u_d=0,u_theta_e=0,u_theta_o=0;
        float error_d=0,error_theta_e=0,error_theta_o=0;
        float error_prev_d=0,error_prev_theta_e=0,error_prev_theta_o=0;
        float kp_d=0,kp_theta_e=0,kp_theta_o=0;
        float ki_d=0,ki_theta_e=0,ki_theta_o=0;
        float kd_d=0,kd_theta_e=0,kd_theta_o=0;
        float kdot_d=0,kdot_theta_e=0,kdot_theta_o=0;
        float Eint_d=0,Eint_theta_e=0,Eint_theta_o=0;
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

        //Read in from (TX3=pin 8, RX3 = pin 7)

        //Distance control
        //given x_pos_init,y_pos_init,x_pos,y_pos,i_vec,j_vec,d
        //error_d is just current distance from point

        i_vec_current=x_pos_f-x_pos;
        j_vec_current=y_pos_f-y_pos;
        distance_current=math.sqrt(i_vec_current*i_vec_current + j_vec_current*j_vec_current);
        error_d = distance_current;
        Eint_d += error_d;
        edot_d= error_d-error_prev_d;
        u_d = kp_d*error_d + ki_d*Eint_d +kd_d*edot_d;
        error_prev_d=error_d;

      //Path control 1 for init and current angle
      disp_unit_i=i_vec_current/distance_current;
      disp_unit_j=j_vec_current/distance_current;
      sin_theta_e = cross_product(disp_unit_i,disp_unit_j,i_vec_init,j_vec_init);
      error_theta_e = math.asin(sin_theta_e);//probably don't need arcsin
      Eint_theta_e += error_theta_e;
      edot_theta_e= error_theta_e-error_prev_theta_e;
      u_theta_e = kp_theta_e*error_theta_e + ki_theta_e*Eint_theta_e +kd_theta_e*edot_theta_e;
      error_prev_theta_e=error_theta_e;
      //Path control 2 for orientation
      sin_theta_o = cross_product(ivec,jvec,i_vec_init,j_vec_init);//ivec,jvec are from the vive and give it's orientation
      error_theta_o = math.asin(sin_theta_o);//probably don't need arcsin
      Eint_theta_o += error_theta_o;
      edot_theta_o= error_theta_o-error_prev_theta_o;
      u_theta_o = kp_theta_o*error_theta_o + ki_theta_o*Eint_theta_o +kd_theta_o*edot_theta_o;
      error_prev_theta_o=error_theta_o;


      }
  }
      
}











    case 8: //Rotation
      if (!in_command)
      {
        Serial.println("setting in_command to true");
        in_command = true;
        x_pos_init=x_pos;//x_pos is a serial read at the top of the code
        y_pos_init=y_pos;//y_pos is a serial read at the top of the code
        x_pos_f=user.data1;
        y_pos_f=user.data2;
        i_vec_init=x_f-x_pos_init;
        j_vec_init=y_f-y_pos_init;
        distance_init=math.sqrt(i_vec_init*i_vec_init + j_vec_init*j_vec_init);
        i_vec_init/=distance_init;
        j_vec_init/=distance_init;
        //PID variables
        int Eint_rot=0;
        float edot_rot=0;
        float u_rot=0;
        float error_rot=0;
        float unew_rot=0;
        float error_prev_rot=0;
        float kp_d_rot=0;
        float ki_d_rot=0;
        float kdot_d_rot=0;
        float kp_p_rot=0;
        float ki_p_rot=0;
        float kdot_p_rot=0;
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
      //PID control for rotation
      else{
//desired_angle will be inputted
orientation_angle = vector_angle(ivec,jvec)
error_rot = desired_angle-orientation angle;
Eint_rot += error_rot;
edot_rot= error_rot-error_prev_rot;
u_rot = kp_rot*error_rot + ki_rot*Eint_rot +kd_rot*edot_rot;
error_prev_rot=error_rot;
//if u_rot is positive then need to go counterclockwise,so drive left wheel backwards and right wheel forwards
//if u_rot is negative need to go clockwise, so drive left wheel forward and right wheel forwards


      }
  }
      
}

    case 9: //Arc
      if (!in_command)
      {
        Serial.println("setting in_command to true");
        in_command = true;
        x_pos_init=x_pos;//x_pos is a serial read at the top of the code
        y_pos_init=y_pos;//y_pos is a serial read at the top of the code
        x_pos_f=user.data1;
        y_pos_f=user.data2;
        i_vec_init=x_f-x_pos_init;
        j_vec_init=y_f-y_pos_init;
        distance_init=math.sqrt(i_vec_init*i_vec_init + j_vec_init*j_vec_init);
        i_vec_init/=distance_init;
        j_vec_init/=distance_init;
        //PID variables
        float edot_d=0,edot_r=0,edot_theta_o=0;
        float u_d=0,u_r=0,u_theta_o=0;
        float error_d=0,error_r=0,error_theta_o=0;
        float error_prev_d=0,error_prev_r=0,error_prev_theta_o=0;
        float kp_d=0,kp_r=0,kp_theta_o=0;
        float ki_d=0,ki_r=0,ki_theta_o=0;
        float kd_d=0,kd_r=0,kd_theta_o=0;
        float kdot_d=0,kdot_r=0,kdot_theta_o=0;
        float Eint_d=0,Eint_r=0,Eint_theta_o=0;


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
      //PID control for rotation
      else{
//PID for arc
//given arc center, angle to go on arc
        //Distance control
        //given x_pos_init,y_pos_init,x_pos,y_pos,i_vec,j_vec,d
        //error_d is just current distance from point

        i_vec_current=x_pos_f-x_pos;
        j_vec_current=y_pos_f-y_pos;
        distance_current=math.sqrt(i_vec_current*i_vec_current + j_vec_current*j_vec_current);
        error_d = distance_current;
        Eint_d += error_d;
        edot_d= error_d-error_prev_d;
        u_d = kp_d*error_d + ki_d*Eint_d +kd_d*edot_d;
        error_prev_d=error_d;

      //Path control 1 for init and current angle
      disp_unit_i=i_vec_current/distance_current;
      disp_unit_j=j_vec_current/distance_current;
      radius_init=sqrt((x_arc_cent-x_init)*(x_arc_cent-x_init)+(y_arc_cent-y_init)*(y_arc_cent-y_init));
      radius_current=sqrt((x_arc_cent-x_pos)*(x_arc_cent-x_pos)+(y_arc_cent-y_pos)*(y_arc_cent-y_pos))
      error_r = radius_current-radius_init;
      Eint_r += error_r;
      edot_r= error_r-error_prev_r;
      u_r = kp_r*error_r + ki_r*Eint_r +kd_r*edot_r;
      error_prev_r=error_r;



      //Path control 2 for orientation
      sin_theta_o = cross_product(ivec,jvec,disp_unit_i,disp_unit_j);//ivec,jvec are from the vive and give it's orientation
      //crossed with tangent vector from arc which is (x,y)=(disp_unit_i,disp_unit_j)
      error_theta_o = math.asin(sin_theta_o);//probably don't need arcsin
      Eint_theta_o += error_theta_o;
      edot_theta_o= error_theta_o-error_prev_theta_o;
      u_theta_o = kp_theta_o*error_theta_o + ki_theta_o*Eint_theta_o +kd_theta_o*edot_theta_o;
      error_prev_theta_o=error_theta_o;

      }
  }
      
}


float vector_angle(float ivec,float jvec){
  float angle;
  if (jvec>0){
  angle = math.arcos(ivec);
  }
  else  if (jvec<0){
  angle = 360 - math.arcos(ivec);
  }
  return angle;
}

float dot_product(float x_1,float y_1,float x_2,float y_2){
    float cos_x;
    sin_x = x_1*x_2-y_1*y_2;
    return cos_x;
}
float cross_product(float x_1,float y_1,float x_2,float y_2){
    float sin_x;
    sin_x = x_1*y_2-x_2*y_1;
    return sin_x;
}
