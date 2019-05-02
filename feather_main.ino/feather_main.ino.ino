
//sensor variables

//mode
int mode = 0;

void setup() {
  //set up bluetooth
  //set up sensors

}

void loop() {
  //update sensor value variables
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
     * Sensors are calculated. We assume our z-position is 0 and constant.
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
  }

}
