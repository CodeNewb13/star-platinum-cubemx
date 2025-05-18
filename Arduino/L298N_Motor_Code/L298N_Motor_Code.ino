const int enA = 3, enB = 11, enC = 9; 
const int in1 = 1, in2 = 2, in3 = 12, in4 = 13, in5 = 7, in6 = 8; 
const int trigPin = 5, echoPin = 6;
const int in1pin = A0, in2pin = A1, in3pin = A2; //input data
long duration;
int distance;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in1pin, INPUT); //input signal from stm32 to arduino
  pinMode(in2pin, INPUT);
  pinMode(in3pin, INPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input


  // power up logic outputs
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
  analogWrite(enC, 0);
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
  forward(in5, in6, enC);
  delay(1500);
  ConveyorStop();
}

void ConveyorReverse(){
  reverse(in1, in2, enA);
  reverse(in5, in6, enC);
  delay(1500);
  ConveyorStop();
}

void ConveyorStop(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 0);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, HIGH);
  analogWrite(enC, 0);
}

void Up(){
  forward(in3, in4, enB);
}

void Down(){
  reverse(in3, in4, enB);
}

void RestVertical(){
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 0);
}

void CheckSensorUp() {
  while (1) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 30000); // timeout after 30ms (max 5 meters)
    distance = (duration * 0.0343) / 2;

    // Ignore invalid readings
    if (distance < 2 || distance > 40) {
      continue; // Skip and try again
    }

    // Return if object is too close or too far
    if (distance >= 30) {
      return;
    }

    delay(50); // Reduce polling rate
  }
}

void CheckSensorDown() {
  while (1) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 30000); // timeout after 30ms (max 5 meters)
    distance = (duration * 0.0343) / 2;

    // Ignore invalid readings
    if (distance < 2 || distance > 40) {
      continue; // Skip and try again
    }

    // Return if object is too close or too far
    if (distance <= 8) {
      return;
    }

    delay(50); // Reduce polling rate
  }
}
/*
Input 1 2 3
      0 0 0 -> Nothing moving
      0 0 1 -> Conveyor forward
      0 1 0 -> Conveyor reverse
      0 1 1 -> Down
      1 0 0 -> Rest vertical 
      1 0 1 -> Down until sensor
      1 1 0 -> Up until sensor
      1 1 1 -> Up



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
      reverse(in3, in4, enB);
      break;
    case 0b100: // 100
      RestVertical();
      ConveyorStop();
      break;
    case 0b101: // 101
      Down();
      ConveyorStop();
      CheckSensorDown();
      RestVertical();
      break;
    case 0b110: // 110
      Up();
      ConveyorStop();
      CheckSensorUp();
      RestVertical();
      break;
    case 0b111: // 111
      forward(in3, in4, enB);
      break;
    default:
      break;
  }
 }
