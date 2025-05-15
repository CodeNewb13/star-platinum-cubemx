#ifndef __PID_H
#define __PID_H

#include <stdbool.h>
#include <stdint.h>

extern uint8_t sensor[7];

// PID structure
typedef struct {
  float kp;         // Proportional gain
  float ki;         // Integral gain
  float kd;         // Derivative gain
  float prev_error; // Previous error
  float integral;   // Integral term
} PID_Controller;

// Kalman filter structure
typedef struct {
  float angle;     // Estimated angle
  float bias;      // Estimated bias
  float rate;      // Unbiased rate
  float P[2][2];   // Error covariance matrix
  float Q_angle;   // Process noise variance for the angle
  float Q_bias;    // Process noise variance for the bias
  float R_measure; // Measurement noise variance
} KalmanFilter;

void readGreyscale(void);
void readGreyscaleLeft(void);
void readGreyscaleBack(void);
void readGreyscaleRight(void);
void invertSensorLeft(void);
void invertSensorBack(void);
void invertSensorRight(void);
void invertSensor(void);
int PID(void);
int getForwardBackwardError(void);
int getLeftRightError(void);
int getOrientationError(void);
int calibrationPID(void);
bool isLeftCentered(void);
bool isBackCentered(void);
bool isRightCentered(void);
bool isCentered(void);
bool FourLineCross(void);
int LinePositionStatus(void);
void PID_Init(PID_Controller *pid, float kp, float ki, float kd);
float PID_Compute(PID_Controller *pid, float setpoint, float measured,
                  float dt);
float Complementary_Filter(float accel_angle, float gz, float dt, float alpha);
void Update_PID(float ax, float ay, float rateCalibrateYaw, float dt,
                float alpha, PID_Controller *pid, KalmanFilter *kf);
void Kalman_Init(KalmanFilter *kf);
float Kalman_Update(KalmanFilter *kf, float new_angle, float new_rate,
                    float dt);
#endif
