#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);
double offset_AccX = 0;
double offset_AccY = 0;
double offset_AccZ = 0;
double x_speed = 0;
double y_speed = 0;
double z_speed = 0;
double x_loc = 0;
double y_loc = 0;
double z_loc = 0;
long next_time = 0;
long timer = 0;
long diff = 0;
long small_diff = 0;

void setup() {
  Serial.begin(9600);
  int SDA = 23;
  int SCL = 22;
  Wire.begin(23, 22);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  offset_AccX = mpu6050.getAccX();
  offset_AccY = mpu6050.getAccY();
  offset_AccZ = mpu6050.getAccZ();
  delay(20);
  for (int i = 0; i < 100; i++)
  {
    mpu6050.update();
    offset_AccX = 0.9 * offset_AccX + mpu6050.getAccX() * 0.1;
    offset_AccY = 0.9 * offset_AccY + mpu6050.getAccY() * 0.1;
    offset_AccZ = 0.9 * offset_AccZ + mpu6050.getAccZ() * 0.1;

    delay(20);
    
  }
  timer = micros();
  
  
}

void loop() {
  mpu6050.update();
  next_time = micros();
  small_diff = next_time - timer - diff;
  diff = next_time - timer;

  x_speed = x_speed + small_diff * 9.8 * 2 * 0.000001 * (mpu6050.getAccX() - offset_AccX);
  y_speed = y_speed + small_diff * 9.8 * 2 * 0.000001 * (mpu6050.getAccY() - offset_AccY);
  z_speed = z_speed + small_diff * 9.8 * 2 * 0.000001 * (mpu6050.getAccZ() - offset_AccZ);

  x_loc = x_loc + small_diff * 0.000001 * x_speed;
  y_loc = y_loc = small_diff * 0.000001 * y_speed;
  z_loc = z_loc = small_diff * 0.000001 * z_speed;
  if(next_time - timer > 50000){
    
    
    Serial.print("micro seconds: ");Serial.print(small_diff);
    Serial.print("\taccX : ");Serial.print(x_speed);
    Serial.print("\taccY : ");Serial.print(y_speed);
    Serial.print("\taccZ : ");Serial.println(z_speed);
  
//    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
//    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
//    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
//  
//    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
//    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
//  
//    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
//    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
//    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
//    
//    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
//    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
//    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    
    timer = micros();
    
  }

}
