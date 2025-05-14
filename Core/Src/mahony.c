//--------------------------------------------------------------------------------------------------
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vector (gravity) and measured one.
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// last update 07/09/2020 SJR minor edits
//--------------------------------------------------------------------------------------------------
// IMU algorithm update
#include "math.h"
#include "mpu6050.h"

const float Kp = 30;
const float Ki = 0.0;
extern float q[4]; // From main.c

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz,
                   float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez; // error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0; // integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0) {

    // Normalise accelerometer (assumed to measure the direction of gravity in
    // body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided
    // out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of
    // gravity in body frame (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat; // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix; // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, q cross gyro term
  deltat = 0.5 * deltat;
  gx *= deltat; // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

// Global variables from main.c
extern short ax, ay, az;
extern short gx, gy, gz;
extern float Axyz[3], Gxyz[3];
extern float A_cal[6], G_off[3];
extern float yaw, pitch, roll; // Euler angle output
extern float prev_yaw;
#define gscale                                                                 \
  ((500. / 32768.0) * (M_PI / 180.0)) // gyro default 500 LSB per d/s -> rad/s
// Define alpha for the low-pass filter
#define ALPHA 0.9

void updateYawPitchRoll(void) {
  float deltat = 0;                      // loop time in seconds
  static unsigned int now = 0, last = 0; // HAL_GetTick() timers

  float ax_avg, ay_avg, az_avg;
  for (int i = 0; i < 25; i++) {
    MPU_Get_Accelerometer(&ax, &ay, &az);
    ax_avg += ax;
    ay_avg += ay;
    az_avg += az;
  }
  ax = ax_avg / 25;
  ay = ay_avg / 25;
  az = az_avg / 25;
  MPU_Get_Gyroscope(&gx, &gy, &gz);
  // HAL_Delay(500);
  // apply offsets and scale factors from Magneto

  for (int i = 0; i < 3; i++)
    Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

  Gxyz[0] =
      ((float)gx - G_off[0]) * gscale; // 250 LSB(d/s) default to radians/s
  Gxyz[1] = ((float)gy - G_off[1]) * gscale;
  Gxyz[2] = ((float)gz - G_off[2]) * gscale;

  now = HAL_GetTick();
  deltat = (now - last) * 1.0e-6; // seconds since last update
  last = now;

  // Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
  // deltat);
  Mahony_update(Axyz[0], Axyz[1], Axyz[2], gx, gy, gz, deltat);
  // Compute Tait-Bryan angles.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North
  // (or true North if corrected for local declination, looking down on the
  // sensor positive yaw is counterclockwise, which is not conventional for
  // NED navigation. Pitch is angle between sensor x-axis and Earth ground
  // plane, toward the Earth is positive, up toward the sky is negative. Roll
  // is angle between sensor y-axis and Earth ground plane, y-axis up is
  // positive roll. These arise from the definition of the homogeneous
  // rotation matrix constructed from quaternions. Tait-Bryan angles as well
  // as Euler angles are non-commutative; that is, the get the correct
  // orientation the rotations must be applied in the correct order which for
  // this configuration is yaw, pitch, and then roll.
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.

  roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  // conventional yaw increases clockwise from North. Not that the MPU-6050
  // knows where North is.
  yaw = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
  // to degrees
  yaw *= 180.0 / M_PI;
  // if (yaw < 0)
  //   yaw += 360.0; // compass circle
  // if (abs((int)(100 * (yaw - prev_yaw))) <=
  //     10) // Cutoff if difference is less than 0.01
  //   yaw = prev_yaw;
  // prev_yaw = yaw;
  // correct for local magnetic declination here
  // Apply low-pass filter to yaw
  float filtered_yaw = ALPHA * prev_yaw + (1 - ALPHA) * yaw;

  // Update prev_yaw for the next iteration
  prev_yaw = filtered_yaw;

  // Use filtered_yaw for further calculations
  yaw = filtered_yaw;
  pitch *= 180.0 / M_PI;
  roll *= 180.0 / M_PI;
}

void resetQuaternions(void) {
  q[0] = 1.0;
  q[1] = 0;
  q[2] = 0;
  q[3] = 0;
}
