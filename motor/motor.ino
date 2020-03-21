void setup() {
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
}

void loop() {
  MotorA();
  MotorB();
  

}

void MotorA(){
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  //delay(10000);
  //digitalWrite(4, HIGH);
  //digitalWrite(5, LOW);
  //delay(10000);

}

void MotorB(){
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  //delay(10000);
  //digitalWrite(6, HIGH);
  //digitalWrite(7, LOW);
  //delay(10000);

}
