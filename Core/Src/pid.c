#include "pid.h"
#include "bsp.h"
#include "mpu6050.h"
#include <math.h>

// TODO: look up how this function can be implemented and utilized
int error = 0;
uint8_t sensor[7];
// The sensors we are using is from IO12-IO6
void readGreyscale(void) {
  sensor[0] = Read_IO12; // left
  sensor[1] = Read_IO11;
  sensor[2] = Read_IO10;
  sensor[3] = Read_IO9;
  sensor[4] = Read_IO8;
  sensor[5] = Read_IO7;
  sensor[6] = Read_IO6; // right
  invertSensor();
}

// Invert for testing purpose on map in B1
void invertSensor(void) {
  for (int i = 0; i < 7; i++) {
    sensor[i] = !sensor[i];
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

bool FourLineCross(void) {
  readGreyscale();
  for (int i = 0; i <= 6; i++) {
    if (sensor[i] == 1) {
      return false; // Not all sensors are over the line
    }
  }
  return true;
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
