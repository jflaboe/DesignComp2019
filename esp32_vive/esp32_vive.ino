// this code communicates with a Teensy that reads Vive sensors to calculate position (of two sensors)
// NDM 5/9/19

char message1[50];
int m1 = 0;
float xpos1 = 0, ypos1 = 0, xpos2 = 0, ypos2 = 0;

// for ToF
int tof_dist = 0;
int tof_status = 0;

void setup() {
  Serial.begin(9600); // for computer
  Serial2.begin(9600,SERIAL_8N1, 16, 17); // for teensy. Tx1 = pin 17, Rx1 = pin 16
}

void loop() {
  checkTeensy();

  Serial.print(xpos1);
  Serial.print(" ");
  Serial.print(ypos1);
  
  Serial.print('\t');
  Serial.print(xpos2);
  Serial.print(" ");
  Serial.print(ypos2);

  Serial.print('\t');
  Serial.print(tof_dist);
  Serial.print(" ");
  Serial.println(tof_status);

  delay(10);

}

void checkTeensy(){
  char type = ' ';
  float val1 = 0, val2 = 0;
  
  while(Serial2.available()){
    message1[m1] = Serial2.read();
    if (message1[m1] == '\n'){
      sscanf(message1,"%c %f %f", &type, &val1, &val2);
      if (type == 'a'){
        xpos1 = val1;
        ypos1 = val2;
      }
      else if (type == 'b'){
        xpos2 = val1;
        ypos2 = val2;
      }
      else if (type == 'c'){
        tof_dist = val1;
        tof_status = val2;
      }
      m1 = 0;
      int iii;
      for(iii=0;iii<50;iii++){
        message1[iii] = 0;
      }
    }
    else {
      m1++;
    }
  }
}
