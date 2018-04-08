void forwardright(void)
{
  digitalWrite(8, LOW);
  digitalWrite(12, HIGH);
}

void forwardleft(void)
{
  digitalWrite(2, LOW);
  digitalWrite(4, HIGH);


}

void backwardleft(void)
{
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
}

void backwardright(void)
{
  digitalWrite(8, HIGH);
  digitalWrite(12, LOW);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);

}

void loop() {
  forwardright();
  delay(100);
  forwardleft();
  delay(100);
  backwardright();
  delay(100);
  backwardleft();
  delay(100);
  // put your main code here, to run repeatedly:

}
