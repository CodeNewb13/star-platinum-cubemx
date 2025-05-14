const int enA = 9, in1 = 8, in2 = 7;

void setup() {
  
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // power up logic outputs
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  delay(1000);

  // 1) Drive enable only
  analogWrite(enA, 255);

  // 2) Forward direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(3000);

/*
  // 3) Reverse direction

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(3000);

  */

  /*

  // 4) Off

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

*/

}

void loop() { }
