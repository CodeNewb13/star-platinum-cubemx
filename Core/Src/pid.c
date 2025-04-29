#include "pid.h"
#include "Bsp.h"

// TODO: look up how this function can be implemented and utilized
int balance_UP(float Angle, float Mechanical_balance, float Gyro) {}
uint8_t sensor[7];
// The sensors we are using is from IO12-IO6
void readGreyscale() {
  sensor[0] = Read_IO12;//left
  sensor[1] = Read_IO11;
  sensor[2] = Read_IO10;
  sensor[3] = Read_IO9;
  sensor[4] = Read_IO8;
  sensor[5] = Read_IO7;
  sensor[6] = Read_IO6;//right
  invertSensor();
}

// Invert for testing purpose on map in B1
void invertSensor (){
  for (int i = 0; i<7; i++){
    sensor[i] = !sensor[i];
  }
}
// TODO: check the algorithm
// TODO: implement fuzzy PID ???
int pos = 0;
int PID() {
  int sensor_average = 0;
  int sensor_sum = 0;
  static int p = 0;
  static int i = 0;
  static int d = 0;
  static int lp = 0;
  int Kp = 15;
  int Ki = 0;
  int Kd = 0; // (Kp-1) * 10
   pos = 0;
  int error = 0;
  int sp = 0;
  int correction = 0;

  readGreyscale();
  for (int j = -3; j <= 3; j++) {
    sensor_average += !sensor[j + 3] * j; // weighted mean
    sensor_sum += !sensor[j + 3];//left positive, right negative
  }

  pos = (int)(sensor_average / sensor_sum);

  error = pos - sp;

  p = error;
  i += p;
  d = p - lp;

  lp = p;

  return correction = (int)(Kp * p + Ki * i + Kd * d);
}

bool FourLineCross() {
  readGreyscale();
  for (int i = 0; i <= 6; i++) {
      if (sensor[i] == 1) {
          return false;  // Not all sensors are over the line
      }
  }
  return true;
}

int LinePositionStatus() {
  readGreyscale();

  int whiteCountCenter = 0;
  for (int i = 2; i <= 4; i++) {
      if (sensor[i] == 0) whiteCountCenter++;
  }

  int whiteCountSides = 0;
  for (int i = 0; i <= 6; i++) {
      if (i <= 1 || i >= 5) {  // side sensors only
          if (sensor[i] == 0) whiteCountSides++;
      }
  }

  if (whiteCountCenter >= 1 && whiteCountSides == 0) {
      return 2;  // Line is centered
  }
  else if (whiteCountSides > 0) {
      return 1;  // Line is at sides
  }
  else {
      return 0;  // No line detected
  }
}