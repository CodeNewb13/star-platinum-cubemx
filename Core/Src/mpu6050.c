#include "mpu6050.h"
#include "bsp.h"
#include "i2c.h"
#include "math.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

//////////////////////////////////////////////////////////////////////////////////
// This program is for educational purposes only and is not authorized for any
// other use without the author's permission.
//
// ALIENTEK STM32F407 Development Board
// MPU6050 Driver Code
// Author: ALIENTEK
// Technical Forum: www.openedv.com
// Creation Date: 2014/5/9
// Version: V1.0
// All rights reserved. Unauthorized reproduction is prohibited.
// Copyright (C) GuangZhou XingYin Electronics Technology Co., Ltd. 2014-2024
//////////////////////////////////////////////////////////////////////////////////

// Initialize MPU6050
// Return value: 0, success
// Others, error code
u8 MPU_Init(void) {
  u8 res;
  HAL_I2C_Init(&hi2c1);                    // Initialize IIC bus
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // Reset MPU6050
  HAL_Delay(100);
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // Wake up MPU6050
  // MPU_Write_Byte(MPU_CFG_REG, 0x05);       // Low pass filter
  MPU_Set_Gyro_Fsr(3);                     // Gyroscope range, ±2000dps
  MPU_Set_Accel_Fsr(0);                    // Accelerometer range, ±2g
  MPU_Set_Rate(200);                       // Set sampling rate to 50Hz
  MPU_Write_Byte(MPU_INT_EN_REG, 0X00);    // Disable all interrupts
  MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // Disable I2C master mode
  MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   // Disable FIFO
  MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT pin active low
  res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
  // USART_SendData(USART1,res);
  if (res == MPU_ADDR) // Correct device ID
  {
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,
                   0X01); // Set CLKSEL, PLL X-axis as reference
    MPU_Write_Byte(MPU_PWR_MGMT2_REG,
                   0X00); // Enable accelerometer and gyroscope
    MPU_Set_Rate(200);    // Set sampling rate to 50Hz
  } else
    return 1;
  return 0;
}

// Set the gyroscope full-scale range for MPU6050
// fsr: 0, ±250dps; 1, ±500dps; 2, ±1000dps; 3, ±2000dps
// Return value: 0, success
//              Others, failure
u8 MPU_Set_Gyro_Fsr(u8 fsr) {
  return MPU_Write_Byte(MPU_GYRO_CFG_REG,
                        fsr << 3); // Set gyroscope full-scale range
}

// Set the accelerometer full-scale range for MPU6050
// fsr: 0, ±2g; 1, ±4g; 2, ±8g; 3, ±16g
// Return value: 0, success
//              Others, failure
u8 MPU_Set_Accel_Fsr(u8 fsr) {
  return MPU_Write_Byte(MPU_ACCEL_CFG_REG,
                        fsr << 3); // Set accelerometer full-scale range
}

// Set the digital low-pass filter (DLPF) for MPU6050
// lpf: Digital low-pass filter frequency (Hz)
// Return value: 0, success
//              Others, failure
u8 MPU_Set_LPF(u16 lpf) {
  u8 data = 0;
  if (lpf >= 188)
    data = 1;
  else if (lpf >= 98)
    data = 2;
  else if (lpf >= 42)
    data = 3;
  else if (lpf >= 20)
    data = 4;
  else if (lpf >= 10)
    data = 5;
  else
    data = 6;
  return MPU_Write_Byte(MPU_CFG_REG, data); // Set digital low-pass filter
}

// Set the sampling rate for MPU6050 (assuming Fs=1KHz)
// rate: 4~1000 (Hz)
// Return value: 0, success
//              Others, failure
u8 MPU_Set_Rate(u16 rate) {
  u8 data;
  if (rate > 1000)
    rate = 1000;
  if (rate < 4)
    rate = 4;
  data = 1000 / rate - 1;
  data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); // Set sampling rate
  return MPU_Set_LPF(rate /
                     2); // Automatically set LPF to half the sampling rate
}

// Get temperature value
// Return value: Temperature value
float MPU_Get_Temperature(void) {
  short raw;
  float temp;
  raw = ((u16)MPU_Read_Byte(MPU_TEMP_OUTH_REG) << 8) |
        MPU_Read_Byte(MPU_TEMP_OUTL_REG);
  temp = 36.53 + ((double)raw) / 340;
  return temp;
}

// Get gyroscope values (raw values)
// gx, gy, gz: Gyroscope x, y, z-axis raw readings (signed)
// Return value: 0, success
//              Others, error code
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz) {
  u8 buf[6], res;
  res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
  if (res == 0) {
    *gx = ((u16)buf[0] << 8) | buf[1];
    *gy = ((u16)buf[2] << 8) | buf[3];
    *gz = ((u16)buf[4] << 8) | buf[5];
  }
  return res;
}

void calibrateGyro(float *G_off) {
  // Calculate the current reading for 2 seconds
  short gx, gy, gz = 0;
  for (int i = 0; i < 500; i++) {
    MPU_Get_Gyroscope(&gx, &gy, &gz);
    G_off[0] += gx;
    G_off[1] += gy;
    G_off[2] += gz;
    HAL_Delay(2);
  }
  for (int i = 0; i < 3; i++) {
    G_off[i] /= 2000; // Take the avg
  }
}

void getScaledRate(float *Gxyz) {
  short gx, gy, gz = 0;
  MPU_Get_Gyroscope(&gx, &gy, &gz);
  Gxyz[0] = (float)gx / 2000;
  Gxyz[1] = (float)gy / 2000;
  Gxyz[2] = (float)gz / 2000;
}

float getRawYawRate(void) {
  short gx, gy, gz = 0;
  MPU_Get_Gyroscope(&gx, &gy, &gz);
  return (float)(gz / 16.4f); // Based on FSR configuration (from data sheet)
}

float getCalibratedYawRate(float rateCalibrateYaw) {
  return getRawYawRate() - rateCalibrateYaw;
}

// Get accelerometer values (raw values)
// ax, ay, az: Accelerometer x, y, z-axis raw readings (signed)
// Return value: 0, success
//              Others, error code
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az) {
  u8 buf[6], res;
  res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, &buf[0]);
  if (res == 0) {
    *ax = ((u16)buf[0] << 8) | buf[1];
    *ay = ((u16)buf[2] << 8) | buf[3];
    *az = ((u16)buf[4] << 8) | buf[5];
  }
  return res;
}

u8 getAccRate(float *accX, float *accY, float *accZ) {
  u8 res;
  short ax, ay, az;
  res = MPU_Get_Accelerometer(&ax, &ay, &az);
  if (res == 0) {
    // Convert to g's (based on FSR and data sheet)
    *accX = (float)ax / 16384 + 0.06;
    *accY = (float)ay / 16384;
    *accZ = (float)az / 16384 + 0.02;
  }
  return res;
}

void calibrateAcc(float *A_cal) {
  // Calculate the current reading for 2 seconds
  short ax, ay, az = 0;
  for (int i = 0; i < 2000; i++) {
    MPU_Get_Accelerometer(&ax, &ay, &az);
    A_cal[0] += ax;
    A_cal[1] += ay;
    A_cal[2] += az;
    HAL_Delay(2);
  }
  for (int i = 0; i < 3; i++) {
    A_cal[i] /= 2000; // Take the avg
  }
}

// I2C continuous write
// addr: Device address
// reg: Register address
// len: Length of data to write
// buf: Data buffer
// Return value: 0, normal
//              Others, error code
//              AI Generated
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf) {
  // Send the register address
  if (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1), &reg, 1, HAL_MAX_DELAY) !=
      HAL_OK) {
    return 1; // Return error if transmission fails
  }

  // Send the data buffer
  if (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1), buf, len, HAL_MAX_DELAY) !=
      HAL_OK) {
    return 1; // Return error if transmission fails
  }

  return 0; // Return success
}

// I2C continuous read
// addr: Device address
// reg: Register address to read from
// len: Length of data to read
// buf: Buffer to store read data
// Return value: 0, normal
//              Others, error code
//              NOTE: AI Generated
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf) {
  // Send the register address
  if (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1), &reg, 1, HAL_MAX_DELAY) !=
      HAL_OK) {
    return 1; // Return error if transmission fails
  }

  // Receive the data buffer
  if (HAL_I2C_Master_Receive(&hi2c1, (addr << 1) | 1, buf, len,
                             HAL_MAX_DELAY) != HAL_OK) {
    return 1; // Return error if reception fails
  }

  return 0; // Return success
}

// I2C write a single byte
// reg: Register address
// data: Data to write
// Return value: 0, normal
//              Others, error code
//              NOTE: AI Generated
u8 MPU_Write_Byte(u8 reg, u8 data) {
  // Send the register address and data in one transaction
  u8 buf[2] = {reg, data};
  if (HAL_I2C_Master_Transmit(&hi2c1, (MPU_ADDR << 1), buf, 2, HAL_MAX_DELAY) !=
      HAL_OK) {
    return 1; // Return error if transmission fails
  }

  return 0; // Return success
}

// I2C read a single byte
// reg: Register address
// Return value: Data read
// NOTE: AI Generated
u8 MPU_Read_Byte(u8 reg) {
  u8 res;

  // Send the register address
  if (HAL_I2C_Master_Transmit(&hi2c1, (MPU_ADDR << 1), &reg, 1,
                              HAL_MAX_DELAY) != HAL_OK) {
    return 0; // Return 0 if transmission fails
  }

  // Receive the data
  if (HAL_I2C_Master_Receive(&hi2c1, (MPU_ADDR << 1) | 1, &res, 1,
                             HAL_MAX_DELAY) != HAL_OK) {
    return 0; // Return 0 if reception fails
  }

  return res; // Return the received byte
}
