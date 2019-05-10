// this code communicates with a Teensy that reads Vive sensors to calculate position (of two sensors)
// NDM 5/9/19

char message1[50];
int m1 = 0;
float xpos1 = 0, ypos1 = 0, xpos2 = 0, ypos2 = 0;

void setup() {
  Serial.begin(9600); // for computer
  Serial2.begin(9600,SERIAL_8N1, 16, 17); // for teensy. Tx1 = pin 17, Rx1 = pin 16
}

void loop() {
  checkVive();

  Serial.print(xpos1);
  Serial.print(" ");
  Serial.print(ypos1);
  
  Serial.print('\t');
  Serial.print(xpos2);
  Serial.print(" ");
  Serial.println(ypos2);

  delay(10);

}

void checkVive(){
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
