#include "pid.h"
#include "bsp.h"
#include "mpu6050.h"
#include <math.h>

// TODO: look up how this function can be implemented and utilized
int error = 0;
uint8_t sensor_back[3];
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
void readGreyscaleLeft(void) {
  sensor_left[0] = Read_IO12; // CCW (left of center axis)
  sensor_left[1] = Read_IO11;
  sensor_left[2] = Read_IO10;
  sensor_left[3] = Read_IO9; // CW (right of center axis)
  invertSensorLeft();
}

void readGreyscaleBack(void) {
  sensor_back[0] = Read_IO6; // CW
  sensor_back[1] = Read_IO7;
  sensor_back[2] = Read_IO8; // CCW
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
  for (int i = 0; i < 3; i++) {
    sensor_back[i] = !sensor_back[i];
  }
}

// TODO: check the algorithm
// TODO: implement fuzzy PID ???
int pos = 0;
int PID(void) {
  int sensor_average = 0;
  int sensor_sum = 0;
  static int p = 0;
  static int i = 0;
  static int d = 0;
  static int lp = 0;
  int Kp = 65;
  float Ki = 0.1;
  int Kd = 0; // (Kp-1) * 10
  error = 0;
  int correction = 0;

  readGreyscale();
  for (int j = -3; j <= 3; j++) {
    sensor_average += !sensor[j + 3] * j; // weighted mean
    sensor_sum += !sensor[j + 3];         // left positive, right negative
    if (sensor_sum >= 3)
      sensor_average = 0; // Ignore when sensor reading is
    // more than 3
  }

  error = (int)(sensor_average / sensor_sum);

  p = error;
  i += p;
  d = p - lp;

  lp = p;

  // Limit integrate
  if (i > 100)
    i = 100;
  return correction = (int)(Kp * p + Ki * i + Kd * d);
}

int getOrientationError(void) {
  int sum = 0;
  int avg = 0;
  int tempSum = 0;
  int tempAvg = 0;
  const int weight = 2;

  int offset = 2;
  // Read sensor left
  for (int i = -2; i <= 2; i++) {
    if (i == 0) {
      offset = 1;
      continue;
    }
    tempAvg += sensor_left[i + offset] * i * weight;
    tempSum += sensor_left[i + offset];
    if (tempSum >= 3 || tempAvg == 0) {
      tempAvg = 0;
      tempSum = 0;
    }
  }

  // Read sensor right
  sum += tempSum;
  avg += tempAvg;
  offset = 2;
  for (int i = -2; i <= 2; i++) {
    if (i == 0) {
      offset = 1;
      continue;
    }
    tempAvg += sensor_right[i + offset] * i * weight * (-1);
    tempSum += sensor_right[i + offset];
    if (tempSum >= 3 || tempAvg == 0) {
      tempAvg = 0;
      tempSum = 0;
    }
  }

  // Read sensor back
  sum += tempSum;
  avg += tempAvg;
  for (int i = -1; i <= 1; i++) {
    tempAvg += sensor_back[i + 1] * i * weight * (-1);
    tempSum += sensor_back[i + 1];
    if (tempSum >= 3 || tempAvg == 0) {
      tempAvg = 0;
      tempSum = 0;
    }
  }
  sum += tempSum;
  avg += tempAvg;

  if (sum > 0)
    error = (int)avg / sum;
  else
    error = 0;
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

bool isCentered(void) {}

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
