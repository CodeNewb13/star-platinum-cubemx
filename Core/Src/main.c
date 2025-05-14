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
#include "stm32f1xx_hal.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mahony.h"
#include "math.h"
#include "motor.h"
#include "movement.h"
#include "mpu6050.h"
#include "pid.h"
#include "types.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int testPID = 0;
float q[4] = {1.0, 0.0, 0.0, 0.0};
float A_cal[6] = {0.0,   0.0,   0.0,
                  1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3];                         // Gyroscope offsets
float yaw, pitch, roll;                 // Euler angle output
float prev_yaw = 0;

int counter = 0;
PID_Controller pid;
// Scaled data as vectors
float Axyz[3];
float Gxyz[3];
uint8_t moveCount = 0;
// Raw data
short ax = 0, ay = 0, az = 0;
short gx = 0, gy = 0, gz = 0;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  MPU_Init();
  // // Raw data
  // short ax = 0, ay = 0, az = 0;
  // short gx = 0, gy = 0, gz = 0;

  // // Scaled data as vectors
  // float Axyz[3];
  // float Gxyz[3];

  // blue yellow,
  // grey red
  // red orange
  // For arduino connection PA0, PA1, PA2
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
  // HAL_Delay(2000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
  // HAL_Delay(2000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // Test tt motor
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  calibrateGyro(G_off);
  // calibrateAcc(A_cal);

  // BotInstruction instructions[50] = {movr,  movf,  movl, movb,  rotl,
  //                                    rotr,  rotb,  take, puth0, puth1,
  //                                    puth2, puth3, stop};
  BotInstruction instructions[50] = {movf, stop};
  uint8_t index = 0;
  bool calibrateFlag = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    testPID = calibrationPID();
    updateYawPitchRoll();
    switch (instructions[index]) {
    case movr:
      break;
    case movf:
      moveForward(100);
      if (moveCount++ >= 30) {
        if (checkForwardEnd()) {
          calibrateFlag = true;
          moveCount = 0;
          index++;
        }
      }
      break;
    case movl:
      break;
    case movb:
      break;

    case rotl:
      break;
    case rotr:
      break;
    case rotb:
      break;

    case take:
      break;

    case puth0:
      break;
    case puth1:
      break;
    case puth2:
      break;
    case puth3:
      break;

    case stop:
      stopMotor();
      break;
    }

    if (calibrateFlag) {
      calibrateOrientation();
      resetQuaternions();
      calibrateFlag = false;
    }

    counter++; // Delay without interrupting other processes
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
