
void setup() {
  Serial.begin(9600);
}

void loop() {
  int r;
  if(Serial.available()>0)
  {
    Serial.print("enter r value");
  r = Serial.read();
  Serial.print("r VALUE IS= %d");
  
  }
}


