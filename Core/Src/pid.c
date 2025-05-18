#include "pid.h"
#include "bsp.h"
#include "mpu6050.h"
#include "stm32f1xx_hal_gpio.h"
#include <math.h>
#include <stdint.h>

// TODO: look up how this function can be implemented and utilized
int error = 0;
uint8_t sensor[7];
uint8_t sensor_back[5];
uint8_t sensor_left[4];
uint8_t sensor_right[4];
// NOTE: sensor left and right lights turned on convention is the opposite of
// back sensor, it turns on if it cant receive the light back to the sensor
// (there's a black color)
// Reading of left and right are 1 if there's no black line, the opposite of
// back sensor

void readGreyscale(void) {
  readGreyscaleLeft();
  readGreyscaleBack();
  readGreyscaleRight();
  // invertSensor(); // Uncomment for black on white (competition scenario)
}
/* Sensor mounting illustration
 * 3 |           | 3 // Pin delegated for back sensor
 * 2 |           | 2
 * 1 |           | 1
 * 0 |           | 0
 *   =============
 *     0 1 2 3 4
 */
void readGreyscaleLeft(void) {
  sensor_left[0] = Read_IO11; // CCW (left of center axis)
  sensor_left[1] = Read_IO10;
  sensor_left[2] = Read_IO12;
  sensor_left[3] = Read_IO9; // CW (right of center axis)
  invertSensorLeft();
}

void readGreyscaleBack(void) {
  sensor_back[0] = Read_IO6; // CW
  sensor_back[1] = Read_IO9;
  sensor_back[2] = Read_IO7;
  sensor_back[3] = Read_IO5;
  sensor_back[4] = Read_IO8; // CCW
  // invertSensorBack();
}

void readGreyscaleRight(void) {
  sensor_right[0] = Read_IO2; // CW
  sensor_right[1] = Read_IO3;
  sensor_right[2] = Read_IO4;
  sensor_right[3] = Read_IO5; // CCW
  invertSensorRight();
}

// Invert for testing purpose on map in B1
void invertSensor(void) {
  invertSensorLeft();
  invertSensorBack();
  invertSensorRight();
}

void invertSensorLeft(void) {
  for (int i = 0; i < 4; i++) {
    sensor_left[i] = !sensor_left[i];
  }
}

void invertSensorRight(void) {
  for (int i = 0; i < 4; i++) {
    sensor_right[i] = !sensor_right[i];
  }
}

void invertSensorBack(void) {
  for (int i = 0; i < 5; i++) {
    sensor_back[i] = !sensor_back[i];
  }
}

// TODO: check the algorithm
// TODO: implement fuzzy PID ???
int pos = 0;
int PID_Back(void) {
  int sensor_average = 0;
  int sensor_sum = 0;
  static int p = 0;
  static int i = 0;
  static int d = 0;
  static int lp = 0;
  int Kp = 50;
  float Ki = 0;
  int Kd = 0; // (Kp-1) * 10
  error = 0;
  int correction = 0;

  const int8_t weightArr[5] = {-2, -1, 0, 1, 2};
  for (int j = 0; j < 5; j++) {
    sensor_average += sensor_back[j] * weightArr[j]; // weighted mean
    sensor_sum += sensor_back[j]; // left positive, right negative
  }

  if (sensor_sum > 0)
    error = (int)(sensor_average / sensor_sum);
  else
    error = 0;

  p = error;
  i += p;
  d = p - lp;

  lp = p;

  // Limit integrate
  if (i > 100)
    i = 100;
  return correction = (int)(Kp * p + Ki * i + Kd * d);
}

void PID_Calibrate(int *correctionArr) {
  // int tempSum = 0;
  // int tempAvg = 0;
  int correction = 0;
  // const int weight = 2;
  // int p = 0, d;
  // static int il = 0, ir = 0, ib = 0;
  // const int kp = 5;
  // const float ki = 0.1;

  // Read sensor left
  // const int8_t leftWeightArr[3] = {2, 0, -2};
  // for (int i = 0; i < 3; i++) {
  //   tempAvg += sensor_left[i] * leftWeightArr[i] * weight;
  //   tempSum += sensor_left[i];
  //   if (tempSum >= 4 || tempAvg == 0 || tempSum == 0) {
  //     tempAvg = 0;
  //     tempSum = 0;
  //   }
  // }
  // if (tempSum > 0)
  //   correction = ((kp + 10)) * (tempAvg / tempSum);
  // else
  //   correction = 0;
  // correctionArr[0] += +correction;
  // correctionArr[1] += -correction;
  // correctionArr[2] += -correction;
  // correctionArr[3] += +correction;
  //
  // Read sensor right
  // tempSum = 0;
  // tempAvg = 0;
  // const int8_t rightWeightArr[3] = {-2, 0, 2};
  // for (int i = 0; i < 3; i++) {
  //   tempAvg += sensor_right[i] * rightWeightArr[i] * weight;
  //   tempSum += sensor_right[i];
  //   if (tempSum >= 4 || tempAvg == 0) {
  //     tempAvg = 0;
  //     tempSum = 0;
  //   }
  // }
  // if (tempSum > 0)
  //   correction = ((kp + 10) / 2) * (tempAvg / tempSum);
  // else
  //   correction = 0;
  // correctionArr[0] += -correction;
  // correctionArr[1] += +correction;
  // correctionArr[2] += +correction;
  // correctionArr[3] += -correction;

  // Read sensor back
  // tempSum = 0;
  // tempAvg = 0;
  // const int8_t backWeightArr[5] = {-2, -1, 0, 1, 2};
  // for (int i = 0; i < 5; i++) {
  //   tempAvg += sensor_back[i] * backWeightArr[i] * weight;
  //   tempSum += sensor_back[i];
  //   if (tempSum >= 3 || tempAvg == 0) {
  //     tempAvg = 0;
  //     tempSum = 0;
  //   }
  // }
  // if (tempSum > 0)
  //   correction = (p + 20) * (tempAvg / tempSum);
  // else
  //   correction = 0;
  // correctionArr[0] += -correction;
  // correctionArr[1] += -correction;
  // correctionArr[2] += +correction;
  // correctionArr[3] += +correction;

  // Left
  correction = PID_Left() * 15 / 10;
  correctionArr[0] += correction / 2;
  correctionArr[1] -= correction / 2;
  correctionArr[2] += correction / 2;
  correctionArr[3] -= correction / 2;

  // Right
  correction = PID_Right() * 15 / 10;
  correctionArr[0] -= 20 - (correction / 2);
  correctionArr[1] += -(20 + correction / 2);
  correctionArr[2] -= 20 + (correction / 2);
  correctionArr[3] += -(20 - correction / 2);

  // Back
  correction = PID_Back() * 15 / 10;
  correctionArr[0] += 20 - correction;
  correctionArr[1] += 20 - correction;
  correctionArr[2] += 20 + correction;
  correctionArr[3] += 20 + correction;

  // if (correctionArr[0] > 0)
  //   correctionArr[0] += 50;
  // else if (correctionArr[0] < 0)
  //   correctionArr[0] -= 50;
  // if (correctionArr[1] > 0)
  //   correctionArr[1] += 50;
  // else if (correctionArr[1] < 0)
  //   correctionArr[1] -= 50;
  // if (correctionArr[2] > 0)
  //   correctionArr[2] += 50;
  // else if (correctionArr[2] < 0)
  //   correctionArr[2] -= 50;
  // if (correctionArr[3] > 0)
  //   correctionArr[3] += 50;
  // else if (correctionArr[3] < 0)
  //   correctionArr[3] -= 50;
}

int PID_Left(void) {
  int sensor_average = 0;
  int sensor_sum = 0;
  static int p = 0;
  static int i = 0;
  static int d = 0;
  static int lp = 0;
  int Kp = 30;
  float Ki = 0;
  int Kd = 0; // (Kp-1) * 10
  error = 0;
  int correction = 0;

  const int8_t weight[3] = {-1, 0, 1};
  for (int j = 0; j < 3; j++) {
    sensor_average += sensor_left[j] * weight[j] * 2; // weighted mean
    sensor_sum += sensor_left[j]; // left positive, right negative
  }

  if (sensor_sum > 0)
    error = (int)(sensor_average / sensor_sum);
  else
    error = 0;

  p = error;
  i += p;
  d = p - lp;

  lp = p;

  // Limit integrate
  if (i > 100)
    i = 100;
  return correction = (int)(Kp * p + Ki * i + Kd * d);
}

int PID_Right(void) {
  int sensor_average = 0;
  int sensor_sum = 0;
  static int p = 0;
  static int i = 0;
  static int d = 0;
  static int lp = 0;
  int Kp = 30;
  float Ki = 0;
  int Kd = 0; // (Kp-1) * 10
  error = 0;
  int correction = 0;

  const int8_t weight[3] = {1, 0, -1};
  for (int j = 0; j < 3; j++) {
    sensor_average += sensor_right[j] * weight[j] * 2; // weighted mean
    sensor_sum += sensor_right[j]; // left positive, right negative
  }

  if (sensor_sum > 0)
    error = (int)(sensor_average / sensor_sum);
  else
    error = 0;

  p = error;
  i += p;
  d = p - lp;

  lp = p;

  // Limit integrate
  if (i > 100)
    i = 100;
  return correction = (int)(Kp * p + Ki * i + Kd * d);
}

int getForwardBackwardError(void) {
  int sum = 0;
  int count = 0;
  int tempCount = 0;
  int tempSum = 0;
  const int weight = 2;
  const int8_t weightArr[4] = {-3, -1, 1, 3};

  // Left sensor
  for (int i = 0; i < 4; i++) {
    tempSum += sensor_left[i] * weightArr[i] * weight;
    tempCount += sensor_left[i];
    if (tempCount >= 4 && tempSum == 0) {
      tempSum = 0;
      tempCount = 0;
    }
  }
  sum += tempSum;
  count += tempCount;
  tempSum = 0;
  tempCount = 0;

  // Right sensor
  for (int i = 0; i < 4; i++) {
    tempSum += sensor_right[i] * weightArr[i] * weight;
    tempCount += sensor_right[i];
    if (tempCount >= 4 && tempSum == 0) {
      tempSum = 0;
      tempCount = 0;
    }
  }
  sum += tempSum;
  count += tempCount;

  if (count > 0)
    error = (int)sum / count; // Get the average
  else
    error = 0;
  return error;
}

int getLeftRightError(void) {
  int sum = 0;
  int count = 0;
  const int weight = 2;
  const int8_t weightArr[3] = {1, 0, -1};

  for (int i = 0; i < 3; i++) {
    sum += sensor_back[i] * weightArr[i] * weight;
    count += sensor_back[i];
    if (count >= 3 && sum == 0) {
      sum = 0;
      count = 0;
    }
  }
  if (count > 0)
    error = (int)sum / count; // Get the average
  else
    error = 0;
  return error;
}

int getOrientationError(void) {
  int sum = 0;
  int avg = 0;
  int tempSum = 0;
  int tempAvg = 0;
  const int weight = 2;

  int offset = 2;
  // Read sensor left
  const int8_t leftWeightArr[3] = {-1, 0, 1};
  for (int i = 0; i < 3; i++) {
    if (i == 0) {
      offset = 1;
      continue;
    }
    tempAvg += sensor_left[i + offset] * leftWeightArr[i] * weight;
    tempSum += sensor_left[i + offset];
    if (tempSum >= 4 || tempAvg == 0) {
      tempAvg = 0;
      tempSum = 0;
    }
  }

  // Read sensor right
  sum += tempSum;
  avg += tempAvg;
  tempSum = 0;
  tempAvg = 0;
  offset = 2;
  const int8_t rightWeightArr[3] = {1, 0, -1};
  for (int i = 0; i < 4; i++) {
    if (i == 0) {
      offset = 1;
      continue;
    }
    tempAvg += sensor_right[i + offset] * rightWeightArr[i] * weight;
    tempSum += sensor_right[i + offset];
    if (tempSum >= 4 || tempAvg == 0) {
      tempAvg = 0;
      tempSum = 0;
    }
  }

  // Read sensor back
  // sum += tempSum;
  // avg += tempAvg;
  // tempSum = 0;
  // tempAvg = 0;
  // const int8_t backWeightArr[3] = {1, 0, -1};
  // for (int i = 0; i < 2; i++) {
  //   tempAvg += sensor_back[i + 1] * backWeightArr[i] * weight;
  //   tempSum += sensor_back[i + 1];
  //   if (tempSum >= 3 || tempAvg == 0) {
  //     tempAvg = 0;
  //     tempSum = 0;
  //   }
  // }
  sum += tempSum;
  avg += tempAvg;

  if (sum > 0)
    error = (int)avg / sum;
  else
    error = 0;

  // Fixed value
  if (error > 0)
    error = 1;
  else if (error < 0)
    error = -1;
  return error;
}

int calibrationPID(void) {
  static int p = 0;
  static int i = 0;
  static int d = 0;
  static int lp = 0;
  int Kp = 65;
  float Ki = 0;
  int Kd = 0; // (Kp-1) * 10
  error = 0;
  int correction = 0;

  readGreyscale();

  error = getOrientationError();

  p = error;
  i += p;
  d = p - lp;

  lp = p;

  // Limit integrate
  if (i > 100)
    i = 100;
  return correction = (int)(Kp * p + Ki * i + Kd * d);
}

bool FourLineCross(void) {
  readGreyscale();
  for (int i = 0; i <= 6; i++) {
    if (sensor[i] == 1) {
      return false; // Not all sensors are over the line
    }
  }
  return true;
}

bool isLeftEmpty(void) {
  return !sensor_left[1] && !sensor_left[2] && !sensor_left[3];
}

bool isRightEmpty(void) {
  return !sensor_right[1] && !sensor_right[2] && !sensor_right[3];
}

bool isLeftRightSame(void) {
  bool temp;
  for (int i = 0; i < 3; i++) {
    temp |= (sensor_left[i] == sensor_right[i]);
  }
  return temp;
}

bool isLeftCentered(void) {
  return sensor_left[0] == 0 && sensor_left[1] == 1 && sensor_left[2] == 0;
}

bool isBackCentered(void) {
  return sensor_back[0] == 0 && sensor_back[1] == 1 && sensor_back[2] == 1 &&
         sensor_back[3] == 1 && sensor_back[4] == 0;
}

bool isRightCentered(void) {
  return sensor_right[0] == 0 && sensor_right[1] == 1 && sensor_right[2] == 0;
}

bool isCentered(void) {
  return isLeftCentered() && isBackCentered() && isRightCentered();
}

int LinePositionStatus(void) {
  readGreyscale();

  int whiteCountCenter = 0;
  for (int i = 2; i <= 4; i++) {
    if (sensor[i] == 0)
      whiteCountCenter++;
  }

  int whiteCountSides = 0;
  for (int i = 0; i <= 6; i++) {
    if (i <= 1 || i >= 5) { // side sensors only
      if (sensor[i] == 0)
        whiteCountSides++;
    }
  }

  if (whiteCountCenter >= 1 && whiteCountSides == 0) {
    return 2; // Line is centered
  } else if (whiteCountSides > 0) {
    return 1; // Line is at sides
  } else {
    return 0; // No line detected
  }
}

// NOTE: AI Generated

// Initialize PID controller
void PID_Init(PID_Controller *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->prev_error = 0.0f;
  pid->integral = 0.0f;
}

// Compute PID output
float PID_Compute(PID_Controller *pid, float setpoint, float measured,
                  float dt) {
  // Clockwise error: Positive, Counter CW: negative
  float error = measured - setpoint;
  pid->integral += error * dt;
  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

// Complementary filter for sensor fusion
float Complementary_Filter(float accel_angle, float gz, float dt, float alpha) {

  // Integrate gyroscope rate to estimate angle
  static float fused_angle = 0.0f; // Initialize fused angle
  fused_angle = alpha * (fused_angle + gz * dt) + (1.0f - alpha) * accel_angle;

  return fused_angle;
}

float fused_angle;
float pid_output;
float filtered_angle;
float gyro_angle;
float accel_angle;
float curr_angle = 0;
// Example usage
void Update_PID(float ax, float ay, float rateCalibrateYaw, float dt,
                float alpha, PID_Controller *pid, KalmanFilter *kf) {
  // Calculate accelerometer angle
  accel_angle = atan2((int)100 * ay, (int)100 * ax) * (180.0f / M_PI);
  // fused_angle = Complementary_Filter(accel_angle, gz, dt, alpha);
  // filtered_angle = Kalman_Update(kf, accel_angle, gz, dt);
  gyro_angle = getCalibratedYawRate(rateCalibrateYaw) * dt;
  curr_angle += gyro_angle;
  // pid_output =
  //     PID_Compute(pid, 0.0f, filtered_angle, dt); // Target setpoint is 0.0
  // Use pid_output to control motors or actuators
}

void Kalman_Init(KalmanFilter *kf) {
  kf->angle = 0.0f;
  kf->bias = 0.0f;
  kf->rate = 0.0f;
  kf->P[0][0] = 0.0f;
  kf->P[0][1] = 0.0f;
  kf->P[1][0] = 0.0f;
  kf->P[1][1] = 0.0f;
  kf->Q_angle = 0.001f;  // Tunable parameter
  kf->Q_bias = 0.003f;   // Tunable parameter
  kf->R_measure = 0.03f; // Tunable parameter
}

// Update Kalman filter
float Kalman_Update(KalmanFilter *kf, float new_angle, float new_rate,
                    float dt) {
  // Predict step
  kf->rate = new_rate - kf->bias;
  kf->angle += dt * kf->rate;

  kf->P[0][0] +=
      dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
  kf->P[0][1] -= dt * kf->P[1][1];
  kf->P[1][0] -= dt * kf->P[1][1];
  kf->P[1][1] += kf->Q_bias * dt;

  // Measurement update
  float S = kf->P[0][0] + kf->R_measure; // Estimate error
  float K[2];                            // Kalman gain
  K[0] = kf->P[0][0] / S;
  K[1] = kf->P[1][0] / S;

  float y = new_angle - kf->angle; // Angle difference
  kf->angle += K[0] * y;
  kf->bias += K[1] * y;

  float P00_temp = kf->P[0][0];
  float P01_temp = kf->P[0][1];

  kf->P[0][0] -= K[0] * P00_temp;
  kf->P[0][1] -= K[0] * P01_temp;
  kf->P[1][0] -= K[1] * P00_temp;
  kf->P[1][1] -= K[1] * P01_temp;

  return kf->angle;
}
