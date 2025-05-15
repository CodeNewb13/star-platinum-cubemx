const int enA = 3, in1 = 1, in2 = 2, enB = 9, in3 = 7, in4 = 8; 
const int in1pin = A0, in2pin = A1, in3pin = A2; //input data

void setup() {
  
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in1pin, INPUT); //input signal from stm32 to arduino
  pinMode(in2pin, INPUT);
  pinMode(in3pin, INPUT);


  // power up logic outputs
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}

void forward(int IN1, int IN2, int en) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(en, 255);
}


void reverse(int IN1, int IN2, int en) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(en, 255);
}


int AnalogtoDigital(int analogPin) {
  int value = analogRead(analogPin);  // Read the analog input
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

void ConveyorForward(){
  forward(in1, in2, enA);
  forward(in3, in4, enB);
}

void ConveyorReverse(){
  reverse(in1, in2, enA);
  reverse(in3, in4, enB);
}

void ConveyorStop(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 0);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 0);
}

void Up(){

}

void Down(){

}

void RestVertical(){

}
/*
Input 1 2 3
      0 0 0 -> Nothing moving
      0 0 1 -> Conveyor forward
      0 1 0 -> Conveyor reverse
      0 1 1 -> 
      1 0 0 -> Rest vertical 
      1 0 1 -> Down
      1 1 0 -> Up
      1 1 1 ->



*/
void loop() {
  int signal = ReadInput();
  switch(signal) {
    case 0b000: // 000
    ConveyorStop();
    RestVertical();
      break;
    case 0b001: // 001
    ConveyorForward();
    RestVertical();
      break;
    case 0b010: // 010
    ConveyorReverse();
    RestVertical();
      break;
    case 0b011: // 011
    
    ConveyorStop();
      break;
    case 0b100: // 100
    RestVertical();
    ConveyorStop();
      break;
    case 0b101: // 101
    Down();
    ConveyorStop();
      break;
    case 0b110: // 110
    Up();
    ConveyorStop();
      break;
    case 0b111: // 111

      break;
    default:
      break;
  }
 }
