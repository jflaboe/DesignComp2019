const int laserPin = 4;   // pin A5 used as output to laser
const int phototransistorPin = 39;  // pin A3 (INPUT only)
int reading = 0;
void setup() {
  Serial.begin(9600);
  pinMode(laserPin, OUTPUT);
  pinMode(phototransistorPin, INPUT);
}

void loop() {
  digitalWrite(laserPin, HIGH);
  delay(50);
  digitalWrite(laserPin, LOW);
  delay(50);

  reading = analogRead(phototransistorPin);
  Serial.println(reading);
}
