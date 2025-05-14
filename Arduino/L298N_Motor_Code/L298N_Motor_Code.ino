const int enA = 3, in1A = 1, in2A = 2, enB = 9, in1B = 8, in2B = 7; 
const int in1pin = A0, in2pin = A1, in3pin = A2; //input data

void setup() {
  
  pinMode(enA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  pinMode(in1pin, INPUT); //input signal from stm32 to arduino
  pinMode(in2pin, INPUT);
  pinMode(in3pin, INPUT);


  // power up logic outputs
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  analogWrite(enA, 0);
}

void forward(int in1, int in2, int en) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(en, 255);
}

<<<<<<< Updated upstream:Arduino/L298N_Motor_Code/L298N_Motor_Code.ino
/*
  // 3) Reverse direction

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(3000);

  */
=======
void reverse(int in1, int in2, int en) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(en, 255);
}
>>>>>>> Stashed changes:L298N_Motor_Driver/L298N_Motor_Code/L298N_Motor_Code.ino

int AnalogtoDigital(int analogPin) {
  int value = analogRead(analogPin);  // Read the analog input

<<<<<<< Updated upstream:Arduino/L298N_Motor_Code/L298N_Motor_Code.ino
  // 4) Off

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

*/
=======
  // Convert the analog value to digital (0 or 1)
  if (value > 512) {
    return 1;  // High signal (greater than 512 is considered HIGH)
  } else {
    return 0;  // Low signal (less than or equal to 512 is considered LOW)
  }
}

int ReadInput(){
  int input1 = AnalogtoDigital(in1pin);
  int input2 = AnalogtoDigital(in2pin);
  int input3 = AnalogtoDigital(in3pin);
  int inputCombination = (input1 << 2) | (input2 << 1) | input3;
  return inputCombination;
}

void ConveyerForward(){
  forward(in1A, in2A, enA);
  forward(in1B, in2B, enB);
}

void ConveyerReverse(){
  reverse(in1A, in2A, enA);
  reverse(in1B, in2B, enB);
}

void ConveyerStop(){
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, HIGH);
  analogWrite(enA, 0);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, HIGH);
  analogWrite(enB, 0);
}

void up(){
>>>>>>> Stashed changes:L298N_Motor_Driver/L298N_Motor_Code/L298N_Motor_Code.ino

}

void down(){

}

void RestVertical(){

}
/*
Input 1 2 3
      0 0 0 -> Nothing moving
      1 0 0 -> Conveyer forward
      0 1 0 -> Conveyer reverse
      0 0 1 -> Conveyer stop
      1 1 0 -> Up
      1 0 1 -> Down
      0 1 1 -> Rest Vertically
      1 1 1



*/
void loop() {
  int signal = ReadInput();
  switch(signal) {
    case 0b000: // 000
    ConveyerStop();
    RestVertical();
      break;
    case 0b001: // 001
    ConveyerForward();
    RestVertical();
      break;
    case 0b010: // 010
    ConveyerReverse();
    RestVertical();
      break;
    case 0b011: // 011
    ConveyerStop();
      break;
    case 0b100: // 100
    Up();
    ConveyerStop();
      break;
    case 0b101: // 101
    Down();
    ConveyerStop();
      break;
    case 0b110: // 110
    RestVertical();
      break;
    case 0b111: // 111

      break;
    default:
      break;
  }
 }
