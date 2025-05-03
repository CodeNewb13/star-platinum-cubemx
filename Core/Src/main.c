/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mahony.h"
#include "math.h"
#include "motor.h"
#include "movement.h"
#include "mpu6050.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define gscale                                                                 \
  ((500. / 32768.0) * (M_PI / 180.0)) // gyro default 500 LSB per d/s -> rad/s
// Define alpha for the low-pass filter
#define ALPHA 0.9
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float q[4] = {1.0, 0.0, 0.0, 0.0};
float A_cal[6] = {0.0,   0.0,   0.0,
                  1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3];                         // Gyroscope offsets
float yaw, pitch, roll;                 // Euler angle output
float prev_yaw = 0;

PID_Controller pid;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  MPU_Init();
  PID_Init(&pid, 10.0, 0.0, 0.0);

  KalmanFilter kf;
  Kalman_Init(&kf);
  float dt = 0.01f;
  float alpha = 0.98f;
  // Raw data
  short ax = 0, ay = 0, az = 0;
  short gx = 0, gy = 0, gz = 0;

  // Scaled data as vectors
  float Axyz[3];
  float Gxyz[3];

  float deltat = 0;               // loop time in seconds
  unsigned int now = 0, last = 0; // HAL_GetTick() timers

  calibrateGyro(G_off);
  // calibrateAcc(A_cal);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // for (uint8_t pos = 0; pos < 8; pos++) {
    //   CH452_SetDigit(pos, 0x00);
    // }
    // moveForward(100);
    // Set_Motor3_RPM(-50);
    // Set_Motor4_RPM(0);
    // Set_Motor1_RPM(0);
    // Set_Motor2_RPM(0);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
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

    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);
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

    roll =
        atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    // conventional yaw increases clockwise from North. Not that the MPU-6050
    // knows where North is.
    yaw =
        -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
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

    // moveForward(100);
    // moveLeft(100);
    // moveRight(150);
    // if (yaw < 90) { // we can leave rotations to sensors instead
    //   motorCW(100);
    // } else
    //   stopMotor();

    // Update_PID(ay, ax, gz, dt, alpha, &pid, &kf);
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
